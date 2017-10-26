/** @file
 */

/*

   DE-IP Core Edge Linux driver

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

#include <linux/sysfs.h>
#include <linux/io.h>

#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_shaper.h"
#include "deipce_sysfs_common.h"
#include "deipce_fqtss_sysfs.h"

#define DEIPCE_MAX_IDLE_SLOPE_LENGTH \
    (DEIPCE_MAX_PRIO_QUEUES * sizeof(uint64_t))

/**
 * Check validity of idle slope table read/write operation.
 * @param ofs offset of read/write operation.
 * @param count Size of read/write operation.
 * @param prio Place for priority this request is targeted at.
 */
static int deipce_fqtss_validate_idle_slope_op(loff_t ofs, size_t count,
                                               unsigned int *prio)
{
    // Allow access to one 64 bit value at a time. Offset is limited already.
    if ((unsigned int)ofs % sizeof(uint64_t))
        return -EINVAL;
    if (count != sizeof(uint64_t))
        return -EINVAL;

    *prio = (unsigned int)ofs / sizeof(uint64_t);

    return 0;
}

// adminIdleSlopeTable

static ssize_t deipce_fqtss_sysfs_admin_idle_slope_table_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int prio;
    int ret;

    netdev_dbg(pp->netdev, "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    ret = deipce_fqtss_validate_idle_slope_op(ofs, count, &prio);
    if (ret)
        return ret;

    spin_lock(&pp->shaper.lock);

    memcpy(buf, &pp->shaper.bitrate[prio], sizeof(*pp->shaper.bitrate));

    spin_unlock(&pp->shaper.lock);

    return count;
}

static ssize_t deipce_fqtss_sysfs_admin_idle_slope_table_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    uint64_t bitrate;
    unsigned int prio;
    int ret = -EINVAL;
    uint16_t addend;

    netdev_dbg(pp->netdev, "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    ret = deipce_fqtss_validate_idle_slope_op(ofs, count, &prio);
    if (ret)
        return ret;

    memcpy(&bitrate, buf, sizeof(bitrate));

    if (bitrate > U32_MAX)
        addend = deipce_shaper_addend(dp->features.clock_freq, U32_MAX);
    else
        addend = deipce_shaper_addend(dp->features.clock_freq,
                                      (uint32_t)bitrate);

    spin_lock(&pp->shaper.lock);

    pp->shaper.bitrate[prio] = bitrate;
    deipce_write_port_reg(pp, PORT_REG_SHAPER(prio), addend);

    spin_unlock(&pp->shaper.lock);

    return count;
}

static DEIPCE_BIN_ATTR_RW(adminIdleSlopeTable,
                          &deipce_fqtss_sysfs_admin_idle_slope_table_read,
                          &deipce_fqtss_sysfs_admin_idle_slope_table_write,
                          DEIPCE_MAX_IDLE_SLOPE_LENGTH);

// operIdleSlopeTable

static ssize_t deipce_fqtss_sysfs_oper_idle_slope_table_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    uint64_t bitrate;
    unsigned int prio;
    int ret;
    uint16_t addend;

    netdev_dbg(pp->netdev, "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    ret = deipce_fqtss_validate_idle_slope_op(ofs, count, &prio);
    if (ret)
        return ret;

    addend = deipce_read_port_reg(pp, PORT_REG_SHAPER(prio));
    bitrate = deipce_shaper_bitrate(dp->features.clock_freq, addend);

    memcpy(buf, &bitrate, sizeof(bitrate));

    return count;
}

static DEIPCE_BIN_ATTR_RO(operIdleSlopeTable,
                          &deipce_fqtss_sysfs_oper_idle_slope_table_read,
                          DEIPCE_MAX_IDLE_SLOPE_LENGTH);

static const struct attribute_group *deipce_fqtss_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021Fqtss",
        .bin_attrs = (struct bin_attribute*[]){
            &bin_attr_adminIdleSlopeTable,
            &bin_attr_operIdleSlopeTable,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize device sysfs files for shaper support.
 * @param pp Port privates.
 */
int deipce_fqtss_sysfs_init(struct deipce_port_priv *pp)
{
    int ret = 0;

    netdev_dbg(pp->netdev, "%s() Init sysfs for FQTSS\n", __func__);

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_fqtss_sysfs_attr_groups);
    if (ret)
        return ret;

    return 0;
}

/**
 * Cleanup device sysfs files for shaper support.
 * @param dp Device privates.
 */
void deipce_fqtss_sysfs_cleanup(struct deipce_port_priv *pp)
{
    netdev_dbg(pp->netdev, "%s() Cleanup sysfs for FQTSS\n", __func__);

    sysfs_remove_groups(&pp->netdev->dev.kobj,
                        deipce_fqtss_sysfs_attr_groups);

    return;
}

