/** @file
 */

/*

   Flexibilis Inter-Block Configuration Linux driver

   Copyright (C) 2017 Flexibilis Oy

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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "deipce_main.h"
#include "deipce_clock_ptp.h"
#include "deipce_ibc_types.h"
#include "deipce_ibc_if.h"
#include "deipce_ibc_main.h"

// Driver private data
static struct deipce_ibc_drv_priv deipce_ibc_drv_priv;

/**
 * Get access to driver privates.
 */
static struct deipce_ibc_drv_priv *deipce_ibc_get_drv_priv(void)
{
    return &deipce_ibc_drv_priv;
}

/**
 * Get IBC device by its node.
 * @param node IBC node.
 * @return IBC device privates.
 */
struct deipce_ibc_dev_priv *deipce_ibc_of_get_device_by_node(
        struct device_node *node)
{
    struct deipce_ibc_drv_priv *drv = deipce_ibc_get_drv_priv();
    static struct deipce_ibc_dev_priv *dp = NULL;

    list_for_each_entry(dp, &drv->devices, list) {
        if (dp->pdev->dev.of_node == node)
            return dp;
    }

    return NULL;
}

/**
 * Get information about selectable clocks.
 * @param dp Device privates.
 * @param phc_list Place for available PHCs.
 * @param count Number of items in phc_list.
 * @param time_sel Place for current time interface number or NULL
 * @param gp_sel Place for current GP interface number or NULL
 * @return Number of selectable clocks updated in phc_list.
 */
unsigned int deipce_ibc_get_clocks(struct deipce_ibc_dev_priv *dp,
                                   struct deipce_ibc_phc_info *phc_list,
                                   unsigned int count,
                                   unsigned int *time_sel,
                                   unsigned int *gp_sel)
{
    unsigned int i;

    if (count > ARRAY_SIZE(dp->phc))
        count = ARRAY_SIZE(dp->phc);

    for (i = 0; i < count; i++)
        phc_list[i] = dp->phc[i];

    if (time_sel || gp_sel) {
        mutex_lock(&dp->lock);

        if (gp_sel)
            *gp_sel =
                deipce_ibc_read_reg(dp, IBC_REG_GP_MUX_CTRL) & IBC_MUX_MASK;
        if (time_sel)
            *time_sel =
                deipce_ibc_read_reg(dp, IBC_REG_TIME_MUX_CTRL) & IBC_MUX_MASK;

        mutex_unlock(&dp->lock);
    }

    return count;
}

/**
 * Set IBC MUXes.
 * @param dp Device privates.
 * @param time_sel Time interface to use.
 * @param gp_sel GP interface to use.
 */
int deipce_ibc_set(struct deipce_ibc_dev_priv *dp,
                   unsigned int time_sel, unsigned int gp_sel)
{
    int ret = -EBUSY;

    dev_dbg(&dp->pdev->dev, "%s() Set time 0x%x GP 0x%x\n",
            __func__, time_sel, gp_sel);

    if (time_sel > 1 || gp_sel > 1)
        return -EINVAL;

    mutex_lock(&dp->lock);

    deipce_ibc_write_reg(dp, IBC_REG_GP_MUX_CTRL, gp_sel);
    deipce_ibc_write_reg(dp, IBC_REG_TIME_MUX_CTRL, time_sel);

    mutex_unlock(&dp->lock);

    return ret;
}

/**
 * Set IBC MUXes.
 * @param dp Device privates.
 * @param time_sel Time interface to use.
 * @param gp_sel GP interface to use.
 */
int deipce_ibc_set_by_phc(struct deipce_ibc_dev_priv *dp, int phc_index)
{
    int ret = -EINVAL;
    int mux_sel = -1;

    mutex_lock(&dp->lock);

    if (phc_index == dp->phc[0].index)
        mux_sel = 0;
    else if (phc_index == dp->phc[1].index)
        mux_sel = 1;

    if (mux_sel >= 0) {
        deipce_ibc_write_reg(dp, IBC_REG_GP_MUX_CTRL, mux_sel);
        deipce_ibc_write_reg(dp, IBC_REG_TIME_MUX_CTRL, mux_sel);
        ret = 0;
    }

    mutex_unlock(&dp->lock);

    dev_dbg(&dp->pdev->dev, "%s() Set for PHC %i time 0x%x GP 0x%x\n",
            __func__, phc_index, mux_sel, mux_sel);

    return ret;
}

/**
 * Function to initialise IBC platform devices.
 * @param pdev Platform device
 * @return 0 on success or negative error code.
 */
static int deipce_ibc_device_init(struct platform_device *pdev)
{
    struct deipce_ibc_drv_priv *drv = &deipce_ibc_drv_priv;
    struct deipce_ibc_dev_priv *dp;
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    struct device_node *node = NULL;
    unsigned int i;
    uint32_t version = 0;
    uint16_t dev_id = 0;
    int ret = -ENXIO;

    dev_dbg(&pdev->dev, "Init device\n");

    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        return -ENOMEM;
    }

    *dp = (struct deipce_ibc_dev_priv) {
        .pdev = pdev,
    };

    mutex_init(&dp->lock);
    INIT_LIST_HEAD(&dp->list);
    list_add(&dp->list, &drv->devices);

    dp->ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->ioaddr) {
        dev_printk(KERN_WARNING, &pdev->dev,
                   ": ioremap failed for device address 0x%llx/0x%llx\n",
                   (unsigned long long int)res->start,
                   (unsigned long long int)resource_size(res));
        goto err_ioremap;
    }

    // Verify device.
    dev_id =
        (deipce_ibc_read_reg(dp, IBC_REG_DEV_ID0) >> 8) |
        (deipce_ibc_read_reg(dp, IBC_REG_DEV_ID1) << 8);

    for (i = 0; i < ARRAY_SIZE(dp->phc); i++) {
        node = of_parse_phandle(pdev->dev.of_node, "mux-clocks", i);
        if (node) {
            dp->phc[i].info = deipce_clock_of_get_phc(node);
            of_node_put(node);
            dp->phc[i].index = deipce_clock_get_phc_index(dp->phc[i].info);
        }
        else {
            dp->phc[i].index = -1;
        }
        dev_dbg(&pdev->dev, "time_sel %u -> PHC index %i\n",
                i, dp->phc[i].index);
    }

    if (dev_id != 0x00e0) {
        dev_err(&pdev->dev, "Device ID 0x%x does not match IBC ID 0x%x\n",
                dev_id, IBC_DEV_ID);
        ret = -ENODEV;
        goto err_dev_id;
    }

    version =
        (uint32_t)(deipce_ibc_read_reg(dp, IBC_REG_INT_ID0) << 0) |
        (uint32_t)(deipce_ibc_read_reg(dp, IBC_REG_INT_ID1) << 16);

    dev_printk(KERN_DEBUG, &pdev->dev, "IBC version %u\n", version);

    return 0;

err_dev_id:
    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

err_ioremap:
    list_del(&dp->list);
    dp->pdev = NULL;
    kfree(dp);

    return ret;
}

/**
 * Function to clean device data
 */
static void deipce_ibc_device_cleanup(struct deipce_ibc_dev_priv *dp)
{
    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

    list_del(&dp->list);
    dp->pdev = NULL;
    mutex_destroy(&dp->lock);

    kfree(dp);

    return;
}

/*
 * Platform device driver match table.
 */
static const struct of_device_id deipce_ibc_match[] = {
    { .compatible = "flx,ibc" },
    { },
};

/**
 * Platform Driver definition for Linux core.
 */
static struct platform_driver deipce_ibc_dev_driver = {
    .driver = {
        .name = "flx_ibc",
        .owner = THIS_MODULE,
        .of_match_table = deipce_ibc_match,
    },
    .probe = &deipce_ibc_device_init,
};

/**
 * Module init.
 * @return 0 if success.
 */
int __init deipce_ibc_init_driver(void)
{
    struct deipce_ibc_drv_priv *drv = deipce_ibc_get_drv_priv();
    int ret = 0;

    INIT_LIST_HEAD(&drv->devices);

    ret = platform_driver_register(&deipce_ibc_dev_driver);
    if (ret)
        return ret;

    return 0;
}

/**
 * Module exit.
 * Cleanup everything.
 */
void deipce_ibc_cleanup_driver(void)
{
    struct deipce_ibc_drv_priv *drv = deipce_ibc_get_drv_priv();
    struct deipce_ibc_dev_priv *dp = NULL;
    struct deipce_ibc_dev_priv *tmp = NULL;

    list_for_each_entry_safe(dp, tmp, &drv->devices, list) {
        deipce_ibc_device_cleanup(dp);
    }

    platform_driver_unregister(&deipce_ibc_dev_driver);

    return;
}

