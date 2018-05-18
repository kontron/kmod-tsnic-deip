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
#include "deipce_clock_main.h"
#include "deipce_time.h"
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

struct deipce_fsc_dev_priv *deipce_fsc_get_device_by_dev(
        struct device *dev)
{
    struct deipce_fsc_drv_priv *drv = deipce_fsc_get_drv_priv();
    static struct deipce_fsc_dev_priv *dp = NULL;

    list_for_each_entry(dp, &drv->devices, list) {
        if (&dp->pdev->dev == dev)
            return dp;
    }

    return NULL;
}

/**
 * Inform driver which PHC is used for FSC.
 * @param dp Device privates.
 * @param phc PHC clock which is in use for this FSC.
 */
int deipce_fsc_set_time(struct deipce_fsc_dev_priv *dp,
                        struct deipce_time *dt)
{
    mutex_lock(&dp->lock);

    dp->time = dt;

    mutex_unlock(&dp->lock);

    return 0;
}

/**
 * Get device configuration.
 * @param dp Device privates.
 */
static int deipce_fsc_device_config(struct deipce_fsc_dev_priv *dp)
{
    unsigned int value = 0;
    int ret = -ENXIO;

    dp->num_sched = deipce_fsc_read16(dp, FSC_GENERICS_SCHEDULERS);
    dp->num_sched &= FSC_GENERICS_SCHEDULERS_MASK;
    if (dp->num_sched == 0 || dp->num_sched > DEIPCE_FSC_MAX_SCHEDULERS)
        return ret;

    dp->num_outputs = deipce_fsc_read16(dp, FSC_GENERICS_OUTPUTS);
    dp->num_outputs &= FSC_GENERICS_OUTPUTS_MASK;
    if (dp->num_outputs == 0 || dp->num_outputs > DEIPCE_FSC_MAX_OUTPUTS)
        return ret;

    switch (dp->num_outputs) {
    case 5:
    case 9:
        dp->hold_output_mask = 1ull << (dp->num_outputs - 1);
        break;
    default:
        dp->hold_output_mask = 0;
    }

    /*
     * Set output mask: first dp->num_output bits to 1.
     * Valid range is 1 - 64.
     */
    dp->output_mask =
        (1ull << (dp->num_outputs - 1)) |
        ((1ull << (dp->num_outputs - 1)) - 1);

    value = deipce_fsc_read16(dp, FSC_GENERICS_ROWS);
    value &= FSC_GENERICS_ROWS_MASK;
    dp->num_rows = 1u << value;
    if (dp->num_rows == 0 || dp->num_rows > DEIPCE_FSC_MAX_ROWS)
        return ret;

    value = deipce_fsc_read16(dp, FSC_GENERICS_CLK_FREQ);
    value &= FSC_GENERICS_CLK_FREQ_MASK;
    if (value == 0)
        return ret;
    dp->clock_cycle_length = 1000u / value;

    dev_printk(KERN_DEBUG, &dp->pdev->dev,
               "FSC with %u schedulers, %u outputs, %u rows, "
               "clock cycle %u ns\n",
               dp->num_sched, dp->num_outputs, dp->num_rows,
               dp->clock_cycle_length);

    return 0;
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
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    struct deipce_fsc_drv_priv *drv = deipce_fsc_get_drv_priv();
    struct deipce_fsc_dev_priv *dp = NULL;
    struct deipce_fsc_sched *sched = NULL;
    struct deipce_fsc_table *table = NULL;
    unsigned int sched_num;
    unsigned int table_num;
    uint16_t table_gen;
    int ret = -ENXIO;

    dev_dbg(&pdev->dev, "Init device\n");

    if (!res) {
        dev_err(&pdev->dev, "No I/O memory defined\n");
        return ret;
    }

    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        return -ENOMEM;
    }

    *dp = (struct deipce_fsc_dev_priv) {
        .pdev = pdev,
    };

    mutex_init(&dp->lock);
    INIT_LIST_HEAD(&dp->list);
    list_add(&dp->list, &drv->devices);

    dp->ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->ioaddr) {
        dev_printk(KERN_WARNING, &dp->pdev->dev,
                   ": ioremap failed for device address 0x%llx/0x%llx\n",
                   (unsigned long long int)res->start,
                   (unsigned long long int)resource_size(res));
        goto err_ioremap;
    }

    dev_printk(KERN_DEBUG, &dp->pdev->dev, "Registers at 0x%llx/0x%llx\n",
               (unsigned long long int)res->start,
               (unsigned long long int)resource_size(res));

    ret = deipce_fsc_device_config(dp);
    if (ret) {
        dev_err(&dp->pdev->dev, "Failed to configure device\n");
        goto err_config;
    }

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
err_config:
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
static void deipce_fsc_device_cleanup(struct deipce_fsc_dev_priv *dp)
{
    dev_dbg(&dp->pdev->dev, "Cleanup device\n");

    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

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

