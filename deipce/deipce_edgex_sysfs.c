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
#include "deipce_sysfs_common.h"
#include "deipce_edgex_sysfs.h"

/// mgmtTrafficClassOverride

static ssize_t deipce_edgex_sysfs_tc_override_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int prio;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    rtnl_lock();

    deipce_get_mgmt_prio(pp, &prio, NULL);

    rtnl_unlock();

    ret = sprintf(buf, "%u\n", prio);

    return ret;
}

static ssize_t deipce_edgex_sysfs_tc_override_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int prio;
    bool enable;
    int ret = kstrtouint(buf, 0, &prio);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    rtnl_lock();

    deipce_get_mgmt_prio(pp, NULL, &enable);
    ret = deipce_set_mgmt_prio(pp, prio, enable);

    rtnl_unlock();

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(mgmtTrafficClassOverride,
                   &deipce_edgex_sysfs_tc_override_show,
                   &deipce_edgex_sysfs_tc_override_store);

/// mgmtTrafficClassOverrideEnable

static ssize_t deipce_edgex_sysfs_tc_override_enable_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    bool enable;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    rtnl_lock();

    deipce_get_mgmt_prio(pp, NULL, &enable);

    rtnl_unlock();

    ret = sprintf(buf, "%u\n", enable);

    return ret;
}

static ssize_t deipce_edgex_sysfs_tc_override_enable_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int prio;
    bool enable;
    int ret = kstrtobool(buf, &enable);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    rtnl_lock();

    deipce_get_mgmt_prio(pp, &prio, NULL);
    ret = deipce_set_mgmt_prio(pp, prio, enable);

    rtnl_unlock();

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(mgmtTrafficClassOverrideEnable,
                      &deipce_edgex_sysfs_tc_override_enable_show,
                      &deipce_edgex_sysfs_tc_override_enable_store);

/// Attribute groups
static const struct attribute_group *deipce_edgex_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "edgex-ext",
        .attrs = (struct attribute*[]){
            &dev_attr_mgmtTrafficClassOverride.attr,
            &dev_attr_mgmtTrafficClassOverrideEnable.attr,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize device sysfs files.
 * @param dp Device privates.
 */
int deipce_edgex_sysfs_init(struct deipce_port_priv *pp)
{
    int ret;

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_edgex_sysfs_attr_groups);
    return ret;
}

/**
 * Cleanup device sysfs files.
 * @param dp Device privates.
 */
void deipce_edgex_sysfs_cleanup(struct deipce_port_priv *pp)
{
    sysfs_remove_groups(&pp->netdev->dev.kobj, deipce_edgex_sysfs_attr_groups);

    return;
}

