/** @file
 */

/*

   Flexibilis PPx Time Stamper Linux driver

   Copyright (C) 2015 Flexibilis Oy

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
#include "deipce_fpts_types.h"
#include "deipce_fpts_if.h"
#include "deipce_fpts_main.h"

// Driver private data
static struct deipce_fpts_drv_priv deipce_fpts_drv_priv;

/**
 * Get access to driver privates.
 */
static struct deipce_fpts_drv_priv *deipce_fpts_get_drv_priv(void)
{
    return &deipce_fpts_drv_priv;
}

/**
 * Get FPTS device of by its node.
 * @param node FPTS node.
 * @return FPTS device privates.
 */
struct deipce_fpts_dev_priv *deipce_fpts_of_get_device_by_node(
        struct device_node *node)
{
    struct deipce_fpts_drv_priv *drv = deipce_fpts_get_drv_priv();
    static struct deipce_fpts_dev_priv *dp = NULL;

    list_for_each_entry(dp, &drv->devices, list) {
        if (dp->pdev->dev.of_node == node)
            return dp;
    }

    return NULL;
}

/**
 * Enable FPTS to record events and generate interrupts.
 * @param dp Device privates.
 */
void deipce_fpts_enable(struct deipce_fpts_dev_priv *dp)
{
    dev_dbg(&dp->pdev->dev, "%s() Start recording events\n", __func__);

    deipce_fpts_write16(dp, FPTS_REG_TS_CTRL, FPTS_TS_CTRL_GET_TS);

    return;
}

/**
 * Stop FPTS from recording events and generating interrupts.
 * @param dp Device privates.
 */
void deipce_fpts_disable(struct deipce_fpts_dev_priv *dp)
{
    dev_dbg(&dp->pdev->dev, "%s() Stop recording events\n", __func__);

    deipce_fpts_write16(dp, FPTS_REG_TS_CTRL, 0);

    return;
}

/**
 * Read event from FPTS to driver's representation.
 * @param dp Device privates.
 * @param event Place for event data.
 */
static void deipce_fpts_read_event(struct deipce_fpts_dev_priv *dp,
                                   struct deipce_fpts_event *event)
{
    event->time.tv_sec = deipce_fpts_read32(dp, FPTS_REG_TS_SEC0);
    event->time.tv_sec |=
        (uint64_t)deipce_fpts_read16(dp, FPTS_REG_TS_SEC2) << 32;
    event->time.tv_sec &= FPTS_TS_SEC_MASK;

    event->time.tv_nsec = deipce_fpts_read32(dp, FPTS_REG_TS_NSEC0);
    event->time.tv_nsec &= FPTS_TS_NSEC_MASK;

    event->counter = deipce_fpts_read32(dp, FPTS_REG_PCNT0);

    dp->last_event = *event;

    return;
}

/**
 * Get new event from FPTS registers if event is available.
 * If event was available, deipce_fpts_enable must be called again
 * to get the next event.
 * @param dp Device privates.
 * @param event Place for event information.
 */
int deipce_fpts_get_event(struct deipce_fpts_dev_priv *dp,
                          struct deipce_fpts_event *event)
{
    uint16_t data;

    dev_dbg(&dp->pdev->dev, "%s() Check for new event\n", __func__);

    // Verify that we really have a new event.
    data = deipce_fpts_read16(dp, FPTS_REG_TS_CTRL);
    if (data & FPTS_TS_CTRL_GET_TS) {
        dev_dbg(&dp->pdev->dev, "%s() No new events\n", __func__);
        return -EAGAIN;
    }

    dev_dbg(&dp->pdev->dev, "%s() Get new event\n", __func__);

    deipce_fpts_read_event(dp, event);

    dev_dbg(&dp->pdev->dev, "%s() Event %lli.%09li s count %u\n",
            __func__, (long long int)event->time.tv_sec, event->time.tv_nsec,
            event->counter);

    return 0;
}

/**
 * Init MMIO register access.
 * @param dp Device privates
 * @return 0 on success.
 */
static int deipce_fpts_mmio_init_device(struct deipce_fpts_dev_priv *dp)
{
    struct resource *res = platform_get_resource(dp->pdev, IORESOURCE_MEM, 0);
    int ret = 0;

    dev_printk(KERN_DEBUG, &dp->pdev->dev,
               "Setup device %u for memory mapped access\n",
               dp->dev_num);

    if (!res) {
        dev_err(&dp->pdev->dev, "No I/O memory defined\n");
        return -ENXIO;
    }

    dp->ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->ioaddr) {
        dev_printk(KERN_WARNING, &dp->pdev->dev,
                   ": ioremap failed for device address 0x%llx/0x%llx\n",
                   (unsigned long long int)res->start,
                   (unsigned long long int)resource_size(res));
        ret = -ENXIO;
        goto out;
    }

    dev_printk(KERN_DEBUG, &dp->pdev->dev,
               "Device uses memory mapped access: 0x%llx/0x%llx\n",
               (unsigned long long int) res->start,
               (unsigned long long int) resource_size(res));

out:
    return ret;
}

/**
 * Cleanup MMIO register access.
 * @param dp Device privates.
 */
static void deipce_fpts_mmio_cleanup_device(struct deipce_fpts_dev_priv *dp)
{
    dev_dbg(&dp->pdev->dev, "Cleanup device memory mapped access\n");

    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

    return;
}

/**
 * Function to initialise FPTS platform devices.
 * @param pdev Platform device
 * @return 0 on success or negative error code.
 */
static int deipce_fpts_device_init(struct platform_device *pdev)
{
    struct deipce_fpts_drv_priv *drv = &deipce_fpts_drv_priv;
    struct deipce_fpts_dev_priv *dp;
    unsigned long int dev_num = 0;
    int ret = -ENXIO;

    // use pdev->id if provided, if only one, pdev->id == -1
    if (pdev->id >= 0) {
        dev_num = pdev->id;
    } else {
        dev_num = find_first_zero_bit(drv->used_devices,
                                      DEIPCE_FPTS_MAX_DEVICES);
    }
    if (dev_num >= DEIPCE_FPTS_MAX_DEVICES) {
        dev_err(&pdev->dev, "Too many FPTS devices\n");
        return -ENODEV;
    }
    if (test_bit(dev_num, drv->used_devices)) {
        dev_err(&pdev->dev, "Device already initialized\n");
        return -ENODEV;
    }

    dev_dbg(&pdev->dev, "Init device %lu\n", dev_num);

    /// Allocate device private
    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        ret = -ENOMEM;
        goto err_alloc;
    }

    *dp = (struct deipce_fpts_dev_priv) {
        .drv = drv,
        .pdev = pdev,
        .dev_num = dev_num,
    };

    set_bit(dp->dev_num, drv->used_devices);
    INIT_LIST_HEAD(&dp->list);
    list_add(&dp->list, &drv->devices);

    ret = deipce_fpts_mmio_init_device(dp);
    if (ret)
        goto err_init;

    // Make sure interrupts from FPTS are disabled.
    deipce_fpts_write16(dp, FPTS_REG_INT_MASK, 0);

    return 0;

err_init:
    list_del(&dp->list);
    clear_bit(dp->dev_num, drv->used_devices);
    dp->pdev = NULL;
    kfree(dp);

err_alloc:
    return ret;
}

/**
 * Function to clean device data
 */
static void deipce_fpts_device_cleanup(struct deipce_fpts_dev_priv *dp)
{
    struct deipce_fpts_drv_priv *drv = deipce_fpts_get_drv_priv();

    deipce_fpts_disable(dp);
    deipce_fpts_mmio_cleanup_device(dp);

    list_del(&dp->list);
    clear_bit(dp->dev_num, drv->used_devices);
    dp->pdev = NULL;

    kfree(dp);

    return;
}

/*
 * Platform device driver match table.
 */
static const struct of_device_id deipce_fpts_match[] = {
    { .compatible = "flx,fpts" },
    { },
};

/**
 * Platform Driver definition for Linux core.
 */
static struct platform_driver deipce_fpts_dev_driver = {
    .driver = {
        .name = "flx_fpts",
        .owner = THIS_MODULE,
        .of_match_table = deipce_fpts_match,
    },
    .probe = &deipce_fpts_device_init,
};

/**
 * Module init.
 * @return 0 if success.
 */
int __init deipce_fpts_init_driver(void)
{
    struct deipce_fpts_drv_priv *drv = deipce_fpts_get_drv_priv();
    int ret = 0;

    INIT_LIST_HEAD(&drv->devices);

    // Register platform driver
    ret = platform_driver_register(&deipce_fpts_dev_driver);
    if (ret)
        goto err_reg_driver;

    return 0;

err_reg_driver:
    return ret;
}

/**
 * Module exit.
 * Cleanup everything.
 */
void deipce_fpts_cleanup_driver(void)
{
    struct deipce_fpts_drv_priv *drv = deipce_fpts_get_drv_priv();
    struct deipce_fpts_dev_priv *dp = NULL;
    struct deipce_fpts_dev_priv *tmp = NULL;

    list_for_each_entry_safe(dp, tmp, &drv->devices, list) {
        deipce_fpts_device_cleanup(dp);
    }

    platform_driver_unregister(&deipce_fpts_dev_driver);

    return;
}

