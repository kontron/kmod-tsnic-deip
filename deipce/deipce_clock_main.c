/** @file
 */

/*

   Flexibilis Real-Time Clock Linux driver

   Copyright (C) 2009 Flexibilis Oy

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>

#include "deipce_main.h"
#include "deipce_fpts_main.h"
#include "deipce_clock_types.h"
#include "deipce_clock_nco.h"
#include "deipce_clock_ptp.h"
#include "deipce_clock_regs.h"
#include "deipce_clock_main.h"

/// Driver privates.
struct flx_frtc_drv_priv flx_frtc_drv_priv;

/// Get access to driver privates.
struct flx_frtc_drv_priv *flx_frtc_get_drv(void)
{
    return &flx_frtc_drv_priv;
}

/**
 * Platform devices and probe functions.
 */
static const struct of_device_id flx_frtc_match[] = {
    { .compatible = "flx,frtc" },
    { },
};

/**
 * FRTC platform device initialization function.
 */
static int flx_frtc_dev_init(struct platform_device *pdev)
{
    int ret = -ENXIO;
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
    struct flx_frtc_dev_priv *dp = NULL;
    struct resource *res = NULL;
#ifdef CONFIG_PTP_1588_CLOCK
    struct device_node *node = NULL;
#endif

    dev_dbg(&pdev->dev, "probe device\n");

    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_warn(&pdev->dev, "kmalloc failed\n");
        ret = -ENOMEM;
        goto err_alloc;
    }
    *dp = (struct flx_frtc_dev_priv){
        .pdev = pdev,
    };
    spin_lock_init(&dp->lock);

    INIT_LIST_HEAD(&dp->list);
    list_add(&dp->list, &drv->devices);

#ifdef CONFIG_PTP_1588_CLOCK
    node = of_parse_phandle(pdev->dev.of_node, "ext-ts", 0);
    if (node) {
        dp->event.fpts = deipce_fpts_of_get_device_by_node(node);
        if (!dp->event.fpts)
            dev_warn(&pdev->dev, "Failed to get FPTS\n");
        of_node_put(node);
    }

#ifdef PTP_PTP_OFFSET_PRECISE
    // Record cross-device timestamping trigger node for resolving them later.
    dp->event.trigger_node =
        of_parse_phandle(pdev->dev.of_node, "ext-ts-trigger-clock", 0);
#endif
#endif

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "I/O registers not defined\n");
        goto err_regs;
    }

    dp->ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->ioaddr) {
        dev_warn(&pdev->dev, "Component ioremap failed\n");
        ret = -ENOMEM;
        goto err_ioremap;
    }

    flx_frtc_nco_init_registers(dp);
    ret = flx_frtc_ptp_init(dp);
    if (ret)
        goto err_ptp;

    return 0;

err_ptp:
    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

err_ioremap:
err_regs:
    list_del(&dp->list);
    dp->pdev = NULL;
    kfree(dp);

err_alloc:
    return ret;
}

/**
 * FRTC platform device cleanup function.
 */
static void flx_frtc_dev_cleanup(struct flx_frtc_dev_priv *dp)
{
    struct platform_device *pdev = dp->pdev;

    dev_dbg(&pdev->dev, "Cleanup\n");

    flx_frtc_ptp_cleanup(dp);

    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

    list_del(&dp->list);
    dp->pdev = NULL;

    kfree(dp);

    dev_dbg(&pdev->dev, "Cleanup done\n");

    return;
}

static struct platform_driver frtc_dev_driver = {
    .driver = {
        .name = "flx_frtc",
        .owner = THIS_MODULE,
        .of_match_table = flx_frtc_match,
    },
    .probe = &flx_frtc_dev_init,
};

/**
 * Driver initialization.
 * @return 0 if success.
 */
int __init deipce_clock_init_driver(void)
{
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
#ifdef CONFIG_PTP_1588_CLOCK
#ifdef PTP_PTP_OFFSET_PRECISE
    static struct flx_frtc_dev_priv *dp = NULL;
    struct ptp_clock_info *info = NULL;
#endif
#endif

    pr_debug(DRV_NAME ": Initialize clock driver\n");

    INIT_LIST_HEAD(&drv->devices);

    platform_driver_register(&frtc_dev_driver);

#ifdef CONFIG_PTP_1588_CLOCK
#ifdef PTP_PTP_OFFSET_PRECISE
    // Resolve cross-device timestamping setups.
    list_for_each_entry(dp, &drv->devices, list) {
        if (dp->event.trigger_node) {
            info = deipce_clock_of_get_phc(dp->event.trigger_node);
            dp->event.trigger =
                container_of(info, struct flx_frtc_dev_priv, ptp_info);
            if (!dp->event.trigger) {
                dev_warn(&dp->pdev->dev,
                         "Failed to get ext TS trigger clock\n");
            }
            else {
                flx_frtc_ptp_init_devxtstamp(dp);
                // Ignore errors.
            }

            of_node_put(dp->event.trigger_node);
            dp->event.trigger_node = NULL;
        }
    }
#endif
#endif

    return 0;
}

/**
 * Driver cleanup.
 */
void deipce_clock_cleanup_driver(void)
{
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
    static struct flx_frtc_dev_priv *dp = NULL;
    static struct flx_frtc_dev_priv *tmp = NULL;

    pr_debug(DRV_NAME ": Cleanup clock driver\n");

    list_for_each_entry_safe(dp, tmp, &drv->devices, list) {
        flx_frtc_dev_cleanup(dp);
    }

    platform_driver_unregister(&frtc_dev_driver);

    return;
}

