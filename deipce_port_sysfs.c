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

#include <linux/rtnetlink.h>

#include "deipce_main.h"
#include "deipce_hw.h"
#include "deipce_sysfs_common.h"
#include "deipce_edgex_sysfs.h"
#include "deipce_phys_sysfs.h"
#include "deipce_mstp_sysfs.h"
#include "deipce_port_sysfs.h"

/// ieee8021Bridge/portNumTrafficClasses

static ssize_t deipce_port_sysfs_port_num_tc_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    ret = sprintf(buf, "%u\n", dp->features.prio_queues);

    return ret;
}

static DEIPCE_ATTR_RO(portNumTrafficClasses,
                   &deipce_port_sysfs_port_num_tc_show);

/// ieee8021Bridge/portDefaultUserPriority

static ssize_t deipce_port_sysfs_port_default_user_prio_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int pcp;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    // Accesses to VLAN configs are synchronized via rtnl lock.
    rtnl_lock();
    pcp = deipce_get_default_vlan_pcp(pp);
    rtnl_unlock();

    ret = sprintf(buf, "%u\n", pcp);

    return ret;
}

static ssize_t deipce_port_sysfs_port_default_user_prio_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int pcp;
    int ret = kstrtouint(buf, 0, &pcp);

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    if (ret)
        return ret;

    // Accesses to VLAN configs are synchronized via rtnl lock.
    rtnl_lock();
    ret = deipce_set_default_vlan_pcp(pp, pcp);
    rtnl_unlock();

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(portDefaultUserPriority,
                   &deipce_port_sysfs_port_default_user_prio_show,
                   &deipce_port_sysfs_port_default_user_prio_store);

/// Attribute groups
static const struct attribute_group *deipce_port_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021Bridge",
        .attrs = (struct attribute*[]){
            &dev_attr_portNumTrafficClasses.attr,
            &dev_attr_portDefaultUserPriority.attr,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize bridge port sysfs files.
 * @param pp Port privates.
 */
int deipce_port_sysfs_init(struct deipce_port_priv *pp)
{
    int ret;

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_port_sysfs_attr_groups);
    if (ret)
        return ret;

    ret = deipce_edgex_sysfs_init_port(pp);
    if (ret)
        goto err_edgex;

    ret = deipce_phys_sysfs_init(pp);
    if (ret)
        goto err_phys;

    ret = deipce_mstp_sysfs_init_port(pp);
    if (ret)
        goto err_mstp;

    return ret;

err_mstp:
    deipce_phys_sysfs_cleanup(pp);

err_phys:
    deipce_edgex_sysfs_cleanup_port(pp);

err_edgex:
    sysfs_remove_groups(&pp->netdev->dev.kobj, deipce_port_sysfs_attr_groups);

    return ret;
}

/**
 * Cleanup bridge port sysfs files.
 * @param pp Port privates.
 */
void deipce_port_sysfs_cleanup(struct deipce_port_priv *pp)
{
    deipce_mstp_sysfs_cleanup_port(pp);
    deipce_phys_sysfs_cleanup(pp);
    deipce_edgex_sysfs_cleanup_port(pp);
    sysfs_remove_groups(&pp->netdev->dev.kobj, deipce_port_sysfs_attr_groups);

    return;
}

