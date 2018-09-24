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

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_ethtool.h"
#include "deipce_preempt.h"
#include "deipce_sysfs_common.h"
#include "deipce_preempt_sysfs.h"

// Preemption/preemptionActive

static ssize_t deipce_preempt_sysfs_preemption_active_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool active = deipce_preempt_get_status_port(pp);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", active);

    return ret;
}

static DEIPCE_ATTR_RO(preemptionActive,
                      &deipce_preempt_sysfs_preemption_active_show);

// Preemption/holdAdvance

static ssize_t deipce_preempt_sysfs_hold_advance_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    uint32_t advance;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    advance = deipce_preempt_get_hold_advance(pp);
    ret = sprintf(buf, "%u\n", advance);

    return ret;
}

static DEIPCE_ATTR_RO(holdAdvance,
                      &deipce_preempt_sysfs_hold_advance_show);

// Preemption/releaseAdvance

static ssize_t deipce_preempt_sysfs_release_advance_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    uint32_t advance;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    advance = deipce_preempt_get_release_advance(pp);
    ret = sprintf(buf, "%u\n", advance);

    return ret;
}

static DEIPCE_ATTR_RO(releaseAdvance,
                      &deipce_preempt_sysfs_release_advance_show);

// MacMerge/support

static ssize_t deipce_preempt_sysfs_mac_merge_support_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    bool supported = dp->features.preempt_ports & (1u << pp->port_num);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", supported);

    return ret;
}

static DEIPCE_ATTR_RO(support,
                      &deipce_preempt_sysfs_mac_merge_support_show);

// MacMerge/addFragSize

static ssize_t deipce_preempt_sysfs_mac_merge_add_frag_size_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int size = deipce_preempt_get_min_frag_size(pp);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", size);

    return ret;
}

static ssize_t deipce_preempt_sysfs_mac_merge_add_frag_size_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int size;
    int ret = kstrtouint(buf, 0, &size);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    ret = deipce_preempt_set_min_frag_size(pp, size);
    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(addFragSize,
                      &deipce_preempt_sysfs_mac_merge_add_frag_size_show,
                      &deipce_preempt_sysfs_mac_merge_add_frag_size_store);

// framePreemptionStatusTable

#define DEIPCE_PREEMPT_SYSFS_EXPRESS 0x01
#define DEIPCE_PREEMPT_SYSFS_PREEMPTABLE 0x02

/**
 * Validate accesses to framePreemptionStatusTable.
 * Allow access to only one element (priority) at a time.
 */
static int deipce_preempt_sysfs_status_table_access(loff_t ofs, size_t count,
                                                    unsigned int *prio)
{
    if (ofs + count > DEIPCE_MAX_PRIO_QUEUES)
        return -EINVAL;
    if (count != 1)
        return -EINVAL;

    *prio = (unsigned int)ofs;

    return 0;
}

static ssize_t deipce_preempt_sysfs_status_table_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    uint8_t *p = (uint8_t *)buf;
    unsigned int tx_prio_enable;
    unsigned int prio;
    int ret = deipce_preempt_sysfs_status_table_access(ofs, count, &prio);

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (ret)
        return ret;

    tx_prio_enable = deipce_preempt_get_enable(pp);

    if (tx_prio_enable & (1u << prio))
        p[0] = DEIPCE_PREEMPT_SYSFS_PREEMPTABLE;
    else
        p[0] = DEIPCE_PREEMPT_SYSFS_EXPRESS;

    return count;
}

static ssize_t deipce_preempt_sysfs_status_table_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    const uint8_t *p = (uint8_t *)buf;
    unsigned int preemptable_mask = 0;
    unsigned int express_mask = 0;
    unsigned int prio;
    int ret = deipce_preempt_sysfs_status_table_access(ofs, count, &prio);

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (ret)
        return ret;

    switch (p[0]) {
    case DEIPCE_PREEMPT_SYSFS_PREEMPTABLE:
        preemptable_mask |= 1u << prio;
        break;
    case DEIPCE_PREEMPT_SYSFS_EXPRESS:
        express_mask |= 1u << prio;
        break;
    default:
        return -EINVAL;
    }

    deipce_preempt_set_enable(pp, preemptable_mask, express_mask);

    return count;
}

static DEIPCE_BIN_ATTR_RW(framePreemptionStatusTable,
                          &deipce_preempt_sysfs_status_table_read,
                          &deipce_preempt_sysfs_status_table_write,
                          DEIPCE_MAX_PRIO_QUEUES);

// MacMerge/enableTx

static ssize_t deipce_preempt_sysfs_mac_merge_enable_tx_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool enable = deipce_preempt_get_enable_port(pp);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", enable);

    return ret;
}

static ssize_t deipce_preempt_sysfs_mac_merge_enable_tx_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool enable;
    int ret = kstrtobool(buf, &enable);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    deipce_preempt_set_enable_port(pp, enable);

    return count;
}

static DEIPCE_ATTR_RW(enableTx,
                      &deipce_preempt_sysfs_mac_merge_enable_tx_show,
                      &deipce_preempt_sysfs_mac_merge_enable_tx_store);

// MacMerge/statusTx

enum deipce_preempt_sysfs_status_tx {
    DEIPCE_PREEMPT_SYSFS_STATUS_TX_UNKNOWN = 1,
    DEIPCE_PREEMPT_SYSFS_STATUS_TX_INACTIVE,
    DEIPCE_PREEMPT_SYSFS_STATUS_TX_ACTIVE,
};

static ssize_t deipce_preempt_sysfs_mac_merge_status_tx_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool active = deipce_preempt_get_status_port(pp);
    enum deipce_preempt_sysfs_status_tx status;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (active)
        status = DEIPCE_PREEMPT_SYSFS_STATUS_TX_ACTIVE;
    else
        status = DEIPCE_PREEMPT_SYSFS_STATUS_TX_INACTIVE;
    ret = sprintf(buf, "%u\n", status);

    return ret;
}

static DEIPCE_ATTR_RO(statusTx,
                      &deipce_preempt_sysfs_mac_merge_status_tx_show);


// MacMerge/verifyDisableTx

static ssize_t deipce_preempt_sysfs_mac_merge_verify_disable_tx_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool disable = !deipce_preempt_get_verify(pp);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", disable);

    return ret;
}

static ssize_t deipce_preempt_sysfs_mac_merge_verify_disable_tx_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool disable = false;
    int ret = kstrtobool(buf, &disable);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    deipce_preempt_set_verify(pp, !disable);

    return count;
}

static DEIPCE_ATTR_RW(verifyDisableTx,
                      &deipce_preempt_sysfs_mac_merge_verify_disable_tx_show,
                      &deipce_preempt_sysfs_mac_merge_verify_disable_tx_store);

// MacMerge/verifyTime

static ssize_t deipce_preempt_sysfs_mac_merge_verify_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int verify_time = deipce_preempt_get_verify_time(pp);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", verify_time);

    return ret;
}

static ssize_t deipce_preempt_sysfs_mac_merge_verify_time_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int verify_time = 0;
    int ret = kstrtouint(buf, 0, &verify_time);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    ret = deipce_preempt_set_verify_time(pp, verify_time);
    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(verifyTime,
                      &deipce_preempt_sysfs_mac_merge_verify_time_show,
                      &deipce_preempt_sysfs_mac_merge_verify_time_store);

// MacMerge/statusVerify

static ssize_t deipce_preempt_sysfs_mac_merge_status_verify_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    enum deipce_preempt_verify_status status = deipce_preempt_get_status(pp);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", status);

    return ret;
}

static DEIPCE_ATTR_RO(statusVerify,
                      &deipce_preempt_sysfs_mac_merge_status_verify_show);

static ssize_t deipce_preempt_sysfs_counter_show(struct device *dev,
                                                 unsigned int counter,
                                                 char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    uint64_t value;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() counter %s\n",
               __func__, deipce_stat_info[counter].name);

    mutex_lock(&pp->stats_lock);

    deipce_update_port_stats(pp);

    value = pp->stats[counter];

    mutex_unlock(&pp->stats_lock);

    ret = sprintf(buf, "%llu\n", value);

    return ret;
}

// MacMerge/frameAssErrorCount

static ssize_t deipce_preempt_sysfs_mac_merge_frame_ass_error_count_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    return deipce_preempt_sysfs_counter_show(dev, FRS_CNT_RX_PREEMPT_ASS_ERR,
                                             buf);
}

static DEIPCE_ATTR_RO(frameAssErrorCount,
                      &deipce_preempt_sysfs_mac_merge_frame_ass_error_count_show);

// MacMerge/frameAssOkCount

static ssize_t deipce_preempt_sysfs_mac_merge_frame_ass_ok_count_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    return deipce_preempt_sysfs_counter_show(dev, FRS_CNT_RX_PREEMPT_ASS_OK,
                                             buf);
}

static DEIPCE_ATTR_RO(frameAssOkCount,
                      &deipce_preempt_sysfs_mac_merge_frame_ass_ok_count_show);

// MacMerge/frameSmdErrorCount

static ssize_t deipce_preempt_sysfs_mac_merge_frame_smd_error_count_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    return deipce_preempt_sysfs_counter_show(dev, FRS_CNT_RX_PREEMPT_SMD_ERR,
                                             buf);
}

static DEIPCE_ATTR_RO(frameSmdErrorCount,
                      &deipce_preempt_sysfs_mac_merge_frame_smd_error_count_show);

// MacMerge/fragCountRx

static ssize_t deipce_preempt_sysfs_mac_merge_frag_count_rx_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    return deipce_preempt_sysfs_counter_show(dev, FRS_CNT_RX_PREEMPT_FRAG,
                                             buf);
}

static DEIPCE_ATTR_RO(fragCountRx,
                      &deipce_preempt_sysfs_mac_merge_frag_count_rx_show);

// MacMerge/fragCountTx

static ssize_t deipce_preempt_sysfs_mac_merge_frag_count_tx_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    return deipce_preempt_sysfs_counter_show(dev, FRS_CNT_TX_PREEMPT_FRAG,
                                             buf);
}

static DEIPCE_ATTR_RO(fragCountTx,
                      &deipce_preempt_sysfs_mac_merge_frag_count_tx_show);

/// Attribute groups
static const struct attribute_group *deipce_preempt_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021Preemption",
        .attrs = (struct attribute*[]){
            &dev_attr_preemptionActive.attr,
            &dev_attr_holdAdvance.attr,
            &dev_attr_releaseAdvance.attr,
            NULL,
        },
        .bin_attrs = (struct bin_attribute*[]){
            &bin_attr_framePreemptionStatusTable,
            NULL,
        },
    },
    &(struct attribute_group){
        .name = "ieee8023MacMerge",
        .attrs = (struct attribute*[]){
            &dev_attr_support.attr,
            &dev_attr_addFragSize.attr,
            &dev_attr_enableTx.attr,
            &dev_attr_statusTx.attr,
            &dev_attr_verifyDisableTx.attr,
            &dev_attr_verifyTime.attr,
            &dev_attr_statusVerify.attr,
            &dev_attr_frameAssErrorCount.attr,
            &dev_attr_frameAssOkCount.attr,
            &dev_attr_frameSmdErrorCount.attr,
            &dev_attr_fragCountRx.attr,
            &dev_attr_fragCountTx.attr,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize port sysfs files.
 * @param dp Port privates.
 */
int deipce_preempt_sysfs_init(struct deipce_port_priv *pp)
{
    int ret = 0;

    netdev_dbg(pp->netdev, "%s() Init sysfs for preemption\n", __func__);

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_preempt_sysfs_attr_groups);
    if (ret)
        return ret;

    return 0;
}

/**
 * Cleanup port sysfs files.
 * @param pp Port privates.
 */
void deipce_preempt_sysfs_cleanup(struct deipce_port_priv *pp)
{
    netdev_dbg(pp->netdev, "%s() Cleanup sysfs for preemption\n", __func__);

    sysfs_remove_groups(&pp->netdev->dev.kobj,
                        deipce_preempt_sysfs_attr_groups);

    return;
}

