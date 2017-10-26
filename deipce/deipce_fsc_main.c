/** @file
 */

/*

   Flexibilis Scheduling Controller Linux driver

   Copyright (C) 2016 Flexibilis Oy

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
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "deipce_main.h"
#include "deipce_clock_ptp.h"
#include "deipce_fsc_types.h"
#include "deipce_fsc_hw.h"
#include "deipce_fsc_if.h"

// Driver private data
static struct deipce_fsc_drv_priv deipce_fsc_drv_priv;

/**
 * Get access to driver privates.
 */
struct deipce_fsc_drv_priv *deipce_fsc_get_drv_priv(void)
{
    return &deipce_fsc_drv_priv;
}

/**
 * Get FSC device by its node.
 * @param node FSC node.
 * @return FSC device privates.
 */
struct deipce_fsc_dev_priv *deipce_fsc_of_get_device_by_node(
        struct device_node *node)
{
    struct deipce_fsc_drv_priv *drv = deipce_fsc_get_drv_priv();
    static struct deipce_fsc_dev_priv *dp = NULL;

    list_for_each_entry(dp, &drv->devices, list) {
        if (dp->pdev->dev.of_node == node)
            return dp;
    }

    return NULL;
}

/**
 * Inform driver which PHC is used for FSC.
 * @param dp Device privates.
 * @param phc PHC clock which is in use for this FSC.
 */
int deipce_fsc_set_clock(struct deipce_fsc_dev_priv *dp,
                         struct ptp_clock_info *phc)
{
    mutex_lock(&dp->lock);

    dp->phc = phc;

    mutex_unlock(&dp->lock);

    return 0;
}

/**
 * Init MMIO register access.
 * @param dp Device privates
 * @return 0 on success.
 */
static int deipce_fsc_mmio_init_device(struct deipce_fsc_dev_priv *dp)
{
    struct resource *res = platform_get_resource(dp->pdev, IORESOURCE_MEM, 0);

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
        return -ENXIO;
    }

    dev_printk(KERN_DEBUG, &dp->pdev->dev,
               "Device uses memory mapped access: 0x%llx/0x%llx\n",
               (unsigned long long int)res->start,
               (unsigned long long int)resource_size(res));

    return 0;
}

/**
 * Get device configuration.
 * @param dp Device privates.
 * @param pdev Platform_device.
 */
static int deipce_fsc_device_config(struct deipce_fsc_dev_priv *dp,
                                    struct platform_device *pdev)
{
    int ret = -ENXIO;

#ifdef CONFIG_OF
    uint32_t value = 0;
    struct device_node *node = NULL;

    ret = of_property_read_u32(pdev->dev.of_node, "schedulers", &value);
    if (ret) {
        dev_err(&pdev->dev, "Number of schedulers is missing\n");
        goto err_of;
    }
    if (value < 1 || value > DEIPCE_FSC_MAX_SCHEDULERS) {
        dev_err(&pdev->dev, "Number of schedulers %u is invalid\n", value);
        goto err_of;
    }
    dp->num_sched = value;

    ret = of_property_read_u32(pdev->dev.of_node, "outputs", &value);
    if (ret) {
        dev_err(&pdev->dev, "Number of outputs is missing\n");
        goto err_of;
    }
    if (value < 1 || value > 64) {
        dev_err(&pdev->dev, "Number of outputs %u is invalid\n", value);
        goto err_of;
    }
    dp->num_outputs = value;
    switch (dp->num_outputs) {
    case 5:
    case 9:
        dp->hold_output_mask = 1ull << (dp->num_outputs - 1);
        break;
    default:
        dp->hold_output_mask = 0;
    }

    ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &value);
    if (ret == 0) {
        if (value == 0) {
            dev_err(&pdev->dev, "Invalid clock frequency %u\n", value);
            goto err_of;
        }
        dp->clock_cycle_length = 1000000000u/value;
    }

    ret = of_property_read_u32(pdev->dev.of_node, "rows", &value);
    if (ret) {
        dev_err(&pdev->dev, "Number of rows is missing\n");
        goto err_of;
    }
    switch (value) {
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
        dp->num_rows = value;
        break;
    default:
        dev_err(&pdev->dev, "Number of rows %u is invalid\n", value);
        goto err_of;
    }

    // Setting clock via device tree is optional.
    node = of_parse_phandle(pdev->dev.of_node, "schedule-clock", 0);
    if (node) {
        dp->phc = deipce_clock_of_get_phc(node);
        of_node_put(node);
        if (!dp->phc)
            dev_err(&pdev->dev, "Failed to get PHC\n");
        dev_dbg(&pdev->dev, "Using default PHC index %i\n",
                deipce_clock_get_phc_index(dp->phc));
    }

#else
    dev_err(&pdev->dev, "Device tree support required\n");
    return -ENXIO;
#endif

    /*
     * Set output mask: first dp->num_output bits to 1.
     * Valid range is 1 - 64.
     */
    dp->output_mask =
        (1ull << (dp->num_outputs - 1)) |
        ((1ull << (dp->num_outputs - 1)) - 1);

    dev_printk(KERN_DEBUG, &pdev->dev,
               "FSC with %u schedulers, %u outputs, %u rows, "
               "clock cycle %u ns\n",
               dp->num_sched, dp->num_outputs, dp->num_rows,
               dp->clock_cycle_length);

    return 0;

#ifdef CONFIG_OF
err_of:
#endif
    return ret;
}

/**
 * Cleanup MMIO register access.
 */
static void deipce_fsc_mmio_cleanup_device(struct deipce_fsc_dev_priv *dp)
{
    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;
    return;
}

/**
 * Initialize scheduler for use.
 * @param dp Device privates.
 * @param sched_num Scheduler number to initialize.
 */
static int deipce_fsc_init_sched(struct deipce_fsc_dev_priv *dp,
                                 struct deipce_fsc_sched *sched)
{
    unsigned int table_num;
    unsigned int row;
    uint16_t dc_speed;
    uint16_t table_gen;
    int ret;

    dev_dbg(&dp->pdev->dev, "%s() Init scheduler %u\n", __func__, sched->num);

    INIT_DELAYED_WORK(&sched->start_work, &deipce_fsc_start_work);

    // Set valid default cycle time.
    sched->admin.param.cycle_time.numerator = 1;
    sched->admin.param.cycle_time.denominator = 2;
    deipce_fsc_split_fraction(&sched->admin.param.cycle_time,
                              &sched->admin.param.cycle_time_fsc);

    // Initialize all rows.
    for (table_num = 0; table_num < DEIPCE_FSC_MAX_TABLES; table_num++) {
        table_gen = deipce_fsc_read16(dp,
                                      FSC_SCHED_TBL_BASE(sched->num,
                                                         table_num) +
                                      FSC_SCHED_TBL_GEN);
        if (table_gen & FSC_SCHED_TBL_GEN_IN_USE)
            continue;

        for (row = 0; row < dp->num_rows; row++) {
            ret = deipce_fsc_write_row(dp, sched->num, table_num, row, 0, 0);
            if (ret)
                return ret;
        }
    }

    dev_dbg(&dp->pdev->dev,
            "%s() Scheduler %u emergency disable %s outputs 0x%llx\n",
            __func__, sched->num,
            deipce_fsc_read16(dp, FSC_SCHED_BASE(sched->num) +
                              FSC_SCHED_EME_DIS_CTRL) &
            FSC_SCHED_EME_DIS_CTRL_ENABLE ? "active" : "inactive",
            deipce_fsc_read_outputs(dp, FSC_SCHED_BASE(sched->num) +
                                    FSC_SCHED_EME_DIS_STAT0));

    // Set all outputs open, except hold.
    deipce_fsc_write_outputs(dp, FSC_SCHED_BASE(sched->num) +
                             FSC_SCHED_EME_DIS_STAT0,
                             ~dp->hold_output_mask);
    deipce_fsc_write16(dp, FSC_SCHED_BASE(sched->num) +
                       FSC_SCHED_EME_DIS_CTRL, FSC_SCHED_EME_DIS_CTRL_ENABLE);

    // Configure clock frequency.
    switch (dp->clock_cycle_length) {
    case 0:
        // default 8 ns, old version without downcounter speed register
        dp->clock_cycle_length = 8;
        return 0;
    case 8:
        dc_speed = FSC_SCHED_DC_SPEED_125MHZ;
        break;
    case 10:
        dc_speed = FSC_SCHED_DC_SPEED_100MHZ;
        break;
    default:
        dev_err(&dp->pdev->dev, "Unsupported clock frequency\n");
        return -EINVAL;
    }

    deipce_fsc_write16(dp, FSC_SCHED_BASE(sched->num) + FSC_SCHED_DC_SPEED,
                       dc_speed);

    dev_dbg(&dp->pdev->dev, "%s() DC speed 0x%x expected 0x%x\n",
            __func__,
            deipce_fsc_read16(dp,
                              FSC_SCHED_BASE(sched->num) + FSC_SCHED_DC_SPEED),
            dc_speed);

    return 0;
}

/**
 * Function to initialise platform device.
 * @param pdev Platform device
 * @return 0 on success or negative error code.
 */
static int deipce_fsc_device_init(struct platform_device *pdev)
{
    struct deipce_fsc_drv_priv *drv = deipce_fsc_get_drv_priv();
    struct deipce_fsc_dev_priv *dp = NULL;
    struct deipce_fsc_sched *sched = NULL;
    struct deipce_fsc_table *table = NULL;
    unsigned int sched_num;
    unsigned int table_num;
    uint16_t table_gen;
    int ret = -ENXIO;

    dev_dbg(&pdev->dev, "Init device\n");

    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        ret = -ENOMEM;
        goto err_alloc;
    }

    *dp = (struct deipce_fsc_dev_priv) {
        .pdev = pdev,
        .clock_cycle_length = 8,
    };

    mutex_init(&dp->lock);
    INIT_LIST_HEAD(&dp->list);
    list_add(&dp->list, &drv->devices);

    ret = deipce_fsc_device_config(dp, pdev);
    if (ret) {
        dev_err(&dp->pdev->dev, "Failed to configure device\n");
        goto err_config;
    }

    ret = deipce_fsc_mmio_init_device(dp);
    if (ret)
        goto err_init;

    for (sched_num = 0; sched_num < dp->num_sched; sched_num++) {
        sched = &dp->sched[sched_num];
        sched->num = sched_num;

        for (table_num = 0; table_num < DEIPCE_FSC_MAX_TABLES; table_num++) {
            table = &sched->table[table_num];
            table->num = table_num;

            table_gen = deipce_fsc_read16(dp,
                                          FSC_SCHED_TBL_BASE(sched->num,
                                                             sched->table_num) +
                                          FSC_SCHED_TBL_GEN);
            if (table_gen & FSC_SCHED_TBL_GEN_IN_USE) {
                sched->table_num = table_num + 1;
                if (sched->table_num >= DEIPCE_FSC_MAX_TABLES)
                    sched->table_num = 0;
            }
            else if (table_gen & FSC_SCHED_TBL_GEN_CAN_USE) {
                sched->table_num = table_num;
                sched->config_change = true;
            }
        }

        ret = deipce_fsc_init_sched(dp, sched);
        if (ret)
            goto err_sched_init;
    }

    return 0;

err_sched_init:
    deipce_fsc_mmio_cleanup_device(dp);

err_init:
err_config:
    list_del(&dp->list);
    dp->pdev = NULL;
    kfree(dp);

err_alloc:
    return ret;
}

/**
 * Function to clean device data
 */
static void deipce_fsc_device_cleanup(struct deipce_fsc_dev_priv *dp)
{
    dev_dbg(&dp->pdev->dev, "Cleanup device\n");

    deipce_fsc_mmio_cleanup_device(dp);

    list_del(&dp->list);
    dp->pdev = NULL;

    kfree(dp);
}

/*
 * Platform device driver match table.
 */
static const struct of_device_id deipce_fsc_match[] = {
    { .compatible = "flx,fsc" },
    { },
};

/**
 * Platform Driver definition for Linux core.
 */
static struct platform_driver deipce_fsc_dev_driver = {
    .driver = {
        .name = "flx_fsc",
        .owner = THIS_MODULE,
        .of_match_table = deipce_fsc_match,
    },
    .probe = &deipce_fsc_device_init,
};

/**
 * Module init.
 * @return 0 if success.
 */
int __init deipce_fsc_init_driver(void)
{
    struct deipce_fsc_drv_priv *drv = deipce_fsc_get_drv_priv();
    int ret = 0;

    INIT_LIST_HEAD(&drv->devices);

    // Register platform driver
    ret = platform_driver_register(&deipce_fsc_dev_driver);
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
void deipce_fsc_cleanup_driver(void)
{
    struct deipce_fsc_drv_priv *drv = deipce_fsc_get_drv_priv();
    struct deipce_fsc_dev_priv *dp = NULL;
    struct deipce_fsc_dev_priv *tmp = NULL;

    list_for_each_entry_safe(dp, tmp, &drv->devices, list) {
        deipce_fsc_device_cleanup(dp);
    }

    platform_driver_unregister(&deipce_fsc_dev_driver);

    return;
}

