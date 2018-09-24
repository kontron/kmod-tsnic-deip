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

#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/rtnetlink.h>

#include "deipce_main.h"
#include "deipce_netdevif.h"
#include "deipce_hw.h"
#include "deipce_sysfs_common.h"
#include "deipce_bridge_sysfs.h"
#include "deipce_edgex_sysfs.h"

/// mirrorPort

static ssize_t deipce_edgex_sysfs_mirror_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    int mirror_port;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    rtnl_lock();

    mirror_port = deipce_get_mirror_port(pp);

    rtnl_unlock();

    ret = sprintf(buf, "%i\n", mirror_port);

    return ret;
}

static ssize_t deipce_edgex_sysfs_mirror_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    int mirror_port;
    int ret = kstrtoint(buf, 0, &mirror_port);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    rtnl_lock();

    ret = deipce_set_mirror_port(pp, mirror_port);

    rtnl_unlock();

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(mirrorPort,
                   &deipce_edgex_sysfs_mirror_show,
                   &deipce_edgex_sysfs_mirror_store);

/// cutThrough

static ssize_t deipce_edgex_sysfs_ct_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool enable;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    enable = deipce_get_cutthrough(pp);
    ret = sprintf(buf, "%u\n", enable);

    return ret;
}

static ssize_t deipce_edgex_sysfs_ct_store(
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

    deipce_set_cutthrough(pp, enable);

    return count;
}

static DEIPCE_ATTR_RW(cutThrough,
                      &deipce_edgex_sysfs_ct_show,
                      &deipce_edgex_sysfs_ct_store);

/**
 * Determine if attribute is visible.
 * @param kobj kobject of the device.
 * @param attr Attribute.
 * @param index Attribute index.
 */
static umode_t deipce_edgex_sysfs_is_visible(struct kobject *kobj,
                                             struct attribute *attr,
                                             int index)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;

    netdev_dbg(pp->netdev, "%s() attr %s index %i\n",
               __func__, attr->name, index);

    if (attr == &dev_attr_cutThrough.attr &&
        !(dp->features.ct_ports & (1u << pp->port_num)))
            return 0;

    return attr->mode;
}

/// Port attribute groups
static const struct attribute_group *deipce_edgex_port_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "edgex-ext",
        .is_visible = &deipce_edgex_sysfs_is_visible,
        .attrs = (struct attribute*[]){
            &dev_attr_mirrorPort.attr,
            &dev_attr_cutThrough.attr,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize edgex port sysfs files.
 * @param pp Port privates.
 */
int deipce_edgex_sysfs_init_port(struct deipce_port_priv *pp)
{
    int ret;

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_edgex_port_sysfs_attr_groups);
    return ret;
}

/**
 * Cleanup edgex port sysfs files.
 * @param pp Port privates.
 */
void deipce_edgex_sysfs_cleanup_port(struct deipce_port_priv *pp)
{
    sysfs_remove_groups(&pp->netdev->dev.kobj,
                        deipce_edgex_port_sysfs_attr_groups);

    return;
}

#ifdef CONFIG_NET_SWITCHDEV

/// mgmtTrafficClass

static ssize_t deipce_edgex_sysfs_tc_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int tc;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    rtnl_lock();

    tc = deipce_get_mgmt_tc(dp);

    rtnl_unlock();

    ret = sprintf(buf, "%u\n", tc);

    return ret;
}

static ssize_t deipce_edgex_sysfs_tc_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_dev_priv *dp = deipce_bridge_sysfs_get_priv(dev);
    unsigned int tc;
    int ret = kstrtouint(buf, 0, &tc);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    rtnl_lock();

    ret = deipce_set_mgmt_tc(dp, tc);

    rtnl_unlock();

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(mgmtTrafficClass,
                   &deipce_edgex_sysfs_tc_show,
                   &deipce_edgex_sysfs_tc_store);

/// Bridge attribute groups
static const struct attribute_group *deipce_edgex_bridge_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "edgexbr-ext",
        .attrs = (struct attribute*[]){
            &dev_attr_mgmtTrafficClass.attr,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize edgex bridge sysfs files.
 * @param dp Switch privates.
 */
int deipce_edgex_sysfs_init_switch(struct deipce_dev_priv *dp)
{
    struct net_device *master = dp->switchdev.master;
    int ret;

    ret = sysfs_create_groups(&master->dev.kobj,
                              deipce_edgex_bridge_sysfs_attr_groups);
    return ret;
}

/**
 * Cleanup edgex bridge sysfs files.
 * @param dp Switch privates.
 */
void deipce_edgex_sysfs_cleanup_switch(struct deipce_dev_priv *dp)
{
    struct net_device *master = dp->switchdev.master;

    sysfs_remove_groups(&master->dev.kobj,
                        deipce_edgex_bridge_sysfs_attr_groups);

    return;
}

#endif
