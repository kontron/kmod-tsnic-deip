/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2018 Flexibilis Oy

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

/*
 * NOTE: None of these will be available without CONFIG_NET_SWITCHDEV.
 * Do not put any function here which is needed without it.
 */

/// Uncomment to enable debug messages
//#define DEBUG

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_hw.h"
#include "deipce_mstp.h"
#include "deipce_sysfs_common.h"
#include "deipce_bridge_sysfs.h"
#include "deipce_mstp_sysfs.h"

/// maxMsti

static ssize_t deipce_mstp_sysfs_max_msti_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (!dp)
        return -ENODEV;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    ret = sprintf(buf, "%hu\n", dp->features.fids);

    return ret;
}

static DEIPCE_ATTR_RO(maxMsti,
                      &deipce_mstp_sysfs_max_msti_show);

/// createMsti

static ssize_t deipce_mstp_sysfs_create_msti_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    uint16_t mstid;
    int ret = kstrtou16(buf, 0, &mstid);

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (!dp)
        return -ENODEV;

    if (ret)
        return ret;

    dev_dbg(dp->this_dev, "%s() mstid %hu\n", __func__, mstid);

    ret = deipce_mstp_add_msti(dp, mstid);
    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_WO(createMsti,
                      &deipce_mstp_sysfs_create_msti_store);

/// deleteMsti

static ssize_t deipce_mstp_sysfs_delete_msti_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    uint16_t mstid;
    int ret = kstrtou16(buf, 0, &mstid);

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (!dp)
        return -ENODEV;

    if (ret)
        return ret;

    dev_dbg(dp->this_dev, "%s() mstid %hu\n", __func__, mstid);

    ret = deipce_mstp_remove_msti(dp, mstid);
    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_WO(deleteMsti,
                      &deipce_mstp_sysfs_delete_msti_store);

/// fid2msti

#define DEIPCE_MSTP_SYSFS_FID2MSTI_ENTRY_SIZE (sizeof(unsigned int))

/**
 * Validate accesses to fid2msti file.
 * @param ofs Access offset.
 * @param count Access size.
 * @param vid Place for VID number access is targeted at.
 */
static int deipce_mstp_sysfs_validate_fid2msti_op(
        loff_t ofs,
        size_t count,
        uint16_t *fid)
{
    if ((size_t)ofs % DEIPCE_MSTP_SYSFS_FID2MSTI_ENTRY_SIZE)
        return -EINVAL;
    if (count != DEIPCE_MSTP_SYSFS_FID2MSTI_ENTRY_SIZE)
        return -EINVAL;
    if (ofs + count >
        VLAN_VID_MASK * DEIPCE_MSTP_SYSFS_FID2MSTI_ENTRY_SIZE)
        return -EINVAL;

    *fid = (size_t)ofs/DEIPCE_MSTP_SYSFS_FID2MSTI_ENTRY_SIZE;

    if (*fid == 0)
        return -EINVAL;

    return 0;
}

static ssize_t deipce_mstp_sysfs_fid2msti_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int *mstid = (unsigned int *)buf;
    uint16_t fid_mstid;
    uint16_t fid;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (!dp)
        return -ENODEV;

    ret = deipce_mstp_sysfs_validate_fid2msti_op(ofs, count, &fid);
    if (ret)
        return ret;

    netdev_dbg(to_net_dev(dev), "%s() fid %u\n", __func__, fid);

    ret = deipce_mstp_get_fid_msti(dp, fid, &fid_mstid);

    if (ret)
        return ret;

    *mstid = fid_mstid;

    return count;
}

static ssize_t deipce_mstp_sysfs_fid2msti_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    const unsigned int *mstid = (unsigned int *)buf;
    uint16_t fid;
    int ret = 0;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (!dp)
        return -ENODEV;

    ret = deipce_mstp_sysfs_validate_fid2msti_op(ofs, count, &fid);
    if (ret)
        return ret;

    netdev_dbg(to_net_dev(dev), "%s()   fid %hu mstid %u\n",
               __func__, fid, *mstid);

    if (*mstid > U16_MAX)
        return -EINVAL;

    ret = deipce_mstp_set_fid_msti(dp, fid, (uint16_t)*mstid);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_BIN_ATTR_RW(fid2msti,
                          &deipce_mstp_sysfs_fid2msti_read,
                          &deipce_mstp_sysfs_fid2msti_write,
                          (DEIPCE_MAX_FIDS + 1) *
                          DEIPCE_MSTP_SYSFS_FID2MSTI_ENTRY_SIZE);

/// mstiList

static ssize_t deipce_mstp_sysfs_msti_list_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int *msti_list = (unsigned int *)buf;
    ssize_t ret = -EINVAL;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (!dp)
        return -ENODEV;

    if ((size_t)ofs % sizeof(*msti_list))
        return -EINVAL;
    if (count % sizeof(*msti_list))
        return -EINVAL;

    ret = deipce_mstp_fill_msti_list(dp, msti_list,
                                  (size_t)ofs / sizeof(*msti_list),
                                  count / sizeof(*msti_list));
    if (ret)
        goto out;

    ret = count;

out:
    return ret;
}

static DEIPCE_BIN_ATTR_RO(mstiList,
                          &deipce_mstp_sysfs_msti_list_read,
                          DEIPCE_MAX_FIDS * (sizeof(unsigned int)));

/// Detect if feature is supported and should thus be visible

static umode_t deipce_mstp_sysfs_is_switch_visible(struct kobject *kobj,
                                                   struct attribute *attr,
                                                   int index)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);

    netdev_dbg(to_net_dev(dev), "%s() attr %s index %i\n",
               __func__, attr->name, index);

    if (dp->features.fids == 0)
        return 0;

    return attr->mode;
}

/// Switch attribute groups
static const struct attribute_group *deipce_mstp_switch_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021Mstp",
        .is_visible = &deipce_mstp_sysfs_is_switch_visible,
        .attrs = (struct attribute*[]){
            &dev_attr_maxMsti.attr,
            &dev_attr_createMsti.attr,
            &dev_attr_deleteMsti.attr,
            NULL,
        },
        .bin_attrs = (struct bin_attribute*[]){
            &bin_attr_fid2msti,
            &bin_attr_mstiList,
            NULL,
        },
    },
    NULL,
};

int deipce_mstp_sysfs_init_switch(struct deipce_dev_priv *dp)
{
    struct net_device *master = dp->switchdev.master;
    int ret;

    dev_dbg(dp->this_dev, "%s() Using %s\n", __func__, netdev_name(master));

    ret = sysfs_create_groups(&master->dev.kobj,
                              deipce_mstp_switch_sysfs_attr_groups);

    return ret;
}

void deipce_mstp_sysfs_cleanup_switch(struct deipce_dev_priv *dp)
{
    struct net_device *master = dp->switchdev.master;

    dev_dbg(dp->this_dev, "%s() Using %s\n", __func__, netdev_name(master));

    sysfs_remove_groups(&master->dev.kobj,
                        deipce_mstp_switch_sysfs_attr_groups);

    return;
}

// flushTree

static ssize_t deipce_port_sysfs_mstp_flush_tree_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    unsigned int mstid;
    int ret = kstrtouint(buf, 0, &mstid);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    ret = deipce_mstp_flush_msti(pp, mstid);
    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_WO(flushTree,
                      &deipce_port_sysfs_mstp_flush_tree_store);

// treePortState

#define DEIPCE_MSTP_SYSFS_TREE_PORT_STATE_ENTRY_SIZE (sizeof(unsigned int))

/**
 * Validate accesses to treePortState file.
 * @param ofs Access offset.
 * @param count Access size.
 * @param mstid Place for MSTID number access is targeted at.
 */
static int deipce_mstp_sysfs_validate_tree_port_state_op(
        loff_t ofs,
        size_t count,
        uint16_t *mstid)
{
    if ((size_t)ofs % DEIPCE_MSTP_SYSFS_TREE_PORT_STATE_ENTRY_SIZE)
        return -EINVAL;
    if (count != DEIPCE_MSTP_SYSFS_TREE_PORT_STATE_ENTRY_SIZE)
        return -EINVAL;
    if (ofs + count >
        (DEIPCE_MSTP_MSTID_MAX + 1) *
        DEIPCE_MSTP_SYSFS_TREE_PORT_STATE_ENTRY_SIZE)
        return -EINVAL;

    *mstid = (size_t)ofs/DEIPCE_MSTP_SYSFS_TREE_PORT_STATE_ENTRY_SIZE;

    return 0;
}

static ssize_t deipce_port_sysfs_mstp_tree_port_state_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int *state = (unsigned int *)buf;
    uint16_t mstid;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    ret = deipce_mstp_sysfs_validate_tree_port_state_op(ofs, count, &mstid);
    if (ret)
        return ret;

    ret = deipce_mstp_get_port_state(pp, mstid, state);
    if (ret)
        return ret;

    return count;
}

static ssize_t deipce_port_sysfs_mstp_tree_port_state_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    const unsigned int *state = (unsigned int *)buf;
    uint16_t mstid;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    ret = deipce_mstp_sysfs_validate_tree_port_state_op(ofs, count, &mstid);
    if (ret)
        return ret;

    ret = deipce_mstp_set_port_state(pp, mstid, *state);
    if (ret)
        return ret;

    return count;
}

static DEIPCE_BIN_ATTR_RW(treePortState,
                          &deipce_port_sysfs_mstp_tree_port_state_read,
                          &deipce_port_sysfs_mstp_tree_port_state_write,
                          (DEIPCE_MSTP_MSTID_MAX + 1) * sizeof(unsigned int));

/// Detect if feature is supported and should thus be visible

static umode_t deipce_mstp_sysfs_is_port_visible(struct kobject *kobj,
                                                 struct attribute *attr,
                                                 int index)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;

    netdev_dbg(to_net_dev(dev), "%s() attr %s index %i\n",
               __func__, attr->name, index);

    if (dp->features.fids == 0)
        return 0;

    return attr->mode;
}

/// Port attribute groups
static const struct attribute_group *deipce_mstp_port_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021Mstp",
        .is_visible = &deipce_mstp_sysfs_is_port_visible,
        .attrs = (struct attribute*[]){
            &dev_attr_flushTree.attr,
            NULL,
        },
        .bin_attrs = (struct bin_attribute*[]){
            &bin_attr_treePortState,
            NULL,
        },
    },
    NULL,
};

int deipce_mstp_sysfs_init_port(struct deipce_port_priv *pp)
{
    int ret;

    netdev_dbg(pp->netdev, "%s()\n", __func__);

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_mstp_port_sysfs_attr_groups);

    return ret;
}

void deipce_mstp_sysfs_cleanup_port(struct deipce_port_priv *pp)
{
    netdev_dbg(pp->netdev, "%s()\n", __func__);

    sysfs_remove_groups(&pp->netdev->dev.kobj,
                        deipce_mstp_port_sysfs_attr_groups);

    return;
}

