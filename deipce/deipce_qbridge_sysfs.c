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

#include <linux/if_vlan.h>

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_hw.h"
#include "deipce_sysfs_common.h"
#include "deipce_bridge_sysfs.h"
#include "deipce_qbridge_sysfs.h"

/// MaxSupportedVlans

static ssize_t deipce_qbridge_sysfs_max_supported_vlans(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (!dp)
        return -ENODEV;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", dp->features.fids);

    return ret;
}

static DEIPCE_ATTR_RO(MaxSupportedVlans,
                      &deipce_qbridge_sysfs_max_supported_vlans);

/// MaxVlanId

static ssize_t deipce_qbridge_sysfs_max_vlan_id(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (!dp)
        return -ENODEV;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", VLAN_VID_MASK - 1);

    return ret;
}

static DEIPCE_ATTR_RO(MaxVlanId,
                      &deipce_qbridge_sysfs_max_vlan_id);

/// NumVlans

static ssize_t deipce_qbridge_sysfs_num_vlans(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int vlan_count;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    if (!dp)
        return -ENODEV;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    rtnl_lock();
    vlan_count = dp->vlan.count;
    rtnl_unlock();

    ret = sprintf(buf, "%u\n", vlan_count);

    return ret;
}

static DEIPCE_ATTR_RO(NumVlans,
                      &deipce_qbridge_sysfs_num_vlans);

/// vid2fid

#define DEIPCE_QBRIDGE_SYSFS_VID2FID_ENTRY_SIZE (sizeof(unsigned int))

/**
 * Validate accesses to vid2fid file.
 * @param ofs Access offset.
 * @param count Access size.
 * @param vid Place for VID number access is targeted at.
 */
static int deipce_qbridge_sysfs_validate_vid2fid_op(
        loff_t ofs,
        size_t count,
        uint16_t *vid)
{
    if ((size_t)ofs % DEIPCE_QBRIDGE_SYSFS_VID2FID_ENTRY_SIZE)
        return -EINVAL;
    if (count != DEIPCE_QBRIDGE_SYSFS_VID2FID_ENTRY_SIZE)
        return -EINVAL;
    if (ofs + count >
        VLAN_VID_MASK * DEIPCE_QBRIDGE_SYSFS_VID2FID_ENTRY_SIZE)
        return -EINVAL;

    *vid = (size_t)ofs/DEIPCE_QBRIDGE_SYSFS_VID2FID_ENTRY_SIZE;

    if (*vid == 0)
        return -EINVAL;

    return 0;
}

static ssize_t deipce_qbridge_sysfs_vid2fid_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int *fid = (unsigned int *)buf;
    uint16_t vid;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (!dp)
        return -ENODEV;

    ret = deipce_qbridge_sysfs_validate_vid2fid_op(ofs, count, &vid);
    if (ret)
        return ret;

    netdev_dbg(to_net_dev(dev), "%s() vid %u\n", __func__, vid);

    rtnl_lock();

    if (deipce_get_vlan_ports(dp, vid))
        *fid = deipce_get_vlan_fid(dp, vid);
    else
        *fid = 0;

    rtnl_unlock();

    return count;
}

static ssize_t deipce_qbridge_sysfs_vid2fid_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int *fid = (unsigned int *)buf;
    uint16_t vid;
    int ret = 0;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (!dp)
        return -ENODEV;

    ret = deipce_qbridge_sysfs_validate_vid2fid_op(ofs, count, &vid);
    if (ret)
        return ret;

    netdev_dbg(to_net_dev(dev), "%s()   vid %u fid %u\n", __func__, vid, *fid);

    if (*fid > VLAN_VID_MASK)
        return -EINVAL;

    rtnl_lock();

    ret = deipce_set_vlan_fid(dp, vid, *fid);

    rtnl_unlock();

    if (ret)
        return ret;

    return count;
}

static DEIPCE_BIN_ATTR_RW(vid2fid,
                          &deipce_qbridge_sysfs_vid2fid_read,
                          &deipce_qbridge_sysfs_vid2fid_write,
                          VLAN_VID_MASK *
                          DEIPCE_QBRIDGE_SYSFS_VID2FID_ENTRY_SIZE);

/// fid2vid

static ssize_t deipce_qbridge_sysfs_fid2vid_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int *fid_vid_map = (unsigned int *)buf;
    const size_t vidlist_size = dp->features.fids * sizeof(*fid_vid_map);
    const size_t fid2vid_size = (dp->features.fids + 1) * vidlist_size;
    ssize_t ret = -EINVAL;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    if (!dp)
        return -ENODEV;

    if (ofs + count > fid2vid_size)
        return -EINVAL;
    if ((size_t)ofs % sizeof(*fid_vid_map))
        return -EINVAL;
    if (count % sizeof(*fid_vid_map))
        return -EINVAL;

    rtnl_lock();

    ret = deipce_fill_fid_vid_map(dp, fid_vid_map,
                                  (unsigned int)ofs / sizeof(*fid_vid_map),
                                  count / sizeof(*fid_vid_map));
    if (ret)
        goto out;

    ret = count;

out:
    rtnl_unlock();

    return ret;
}

static DEIPCE_BIN_ATTR_RO(fid2vid,
                          &deipce_qbridge_sysfs_fid2vid_read,
                          (DEIPCE_MAX_FIDS + 1) *
                          DEIPCE_MAX_FIDS *
                          (sizeof(unsigned int)));

/// Detect if feature is supported and should thus be visible

static umode_t deipce_qbridge_sysfs_is_visible(struct kobject *kobj,
                                               struct attribute *attr,
                                               int index)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);

    netdev_dbg(to_net_dev(dev), "%s() attr %s index %i\n",
               __func__, attr->name, index);

    if (attr == &bin_attr_vid2fid.attr && dp->features.fids == 0)
        return 0;

    return attr->mode;
}

/// Attribute groups
static const struct attribute_group *deipce_qbridge_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021QBridge",
        .is_visible = &deipce_qbridge_sysfs_is_visible,
        .attrs = (struct attribute*[]){
            &dev_attr_MaxSupportedVlans.attr,
            &dev_attr_MaxVlanId.attr,
            &dev_attr_NumVlans.attr,
            NULL,
        },
        .bin_attrs = (struct bin_attribute*[]){
            &bin_attr_vid2fid,
            &bin_attr_fid2vid,
            NULL,
        },
    },
    NULL,
};

int deipce_qbridge_sysfs_init(struct deipce_dev_priv *dp)
{
    struct net_device *master = dp->switchdev.master;
    int ret;

    dev_dbg(dp->this_dev, "%s() Using %s\n", __func__, netdev_name(master));

    ret = sysfs_create_groups(&master->dev.kobj,
                              deipce_qbridge_sysfs_attr_groups);
    if (ret)
        dp->switchdev.master = NULL;

    return ret;
}

void deipce_qbridge_sysfs_cleanup(struct deipce_dev_priv *dp)
{
    struct net_device *master = dp->switchdev.master;

    dev_dbg(dp->this_dev, "%s() Using %s\n", __func__, netdev_name(master));

    sysfs_remove_groups(&master->dev.kobj, deipce_qbridge_sysfs_attr_groups);

    return;
}

