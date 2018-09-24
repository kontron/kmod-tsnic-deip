/** @file
 */

/*

   DE-IP Core Edge Linux driver

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

/*
 * NOTE: None of these will be available without CONFIG_NET_SWITCHDEV.
 * Do not put any function here which is needed without it.
 */

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/version.h>
#include <net/switchdev.h>
#include <net/netlink.h>

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_hw.h"
#include "deipce_netdev.h"
#include "deipce_bridge_sysfs.h"
#include "deipce_switchdev.h"

/**
 * Uncomment if this patch has been applied to Linux < 4.4.4:
 * switchdev: Require RTNL mutex to be held when sending FDB notifications
 * It is highly recommended.
 */
#define DEIPCE_SWITCHDEV_RTNL_FIXED

/// FDB (FES MAC address table) change notification interval in jiffies
#define DEIPCE_SWITCHDEV_NOTIFY_FDB_INTERVAL (8*HZ)

/// Default bridge port flags
#define DEIPCE_DEFAULT_BR_FLAGS (BR_LEARNING | BR_LEARNING_SYNC | BR_FLOOD)

/**
 * Convert switchdev transaction phase to string.
 */
static const char *deipce_switchdev_trans_str(struct switchdev_trans *trans)
{
    if (switchdev_trans_ph_prepare(trans))
        return "prepare";
    if (switchdev_trans_ph_commit(trans))
        return "commit";
    return "";
}

/**
 * Notify upper layers of switchdev events.
 * Must call with rtnl lock held.
 * RTNL lock usage changed in Linux 4.4.4.
 * This wrapper releases rtnl lock temporarily if necessary.
 */
static inline int deipce_switchdev_call_notifiers(
        unsigned long int val,
        struct net_device *dev,
        struct switchdev_notifier_info *info)
{
    int ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,4) && !defined(DEIPCE_SWITCHDEV_RTNL_FIXED)
#warning "RTNL locking patch is highly recommended with Linux < 4.4.4"
#warning "Look for and apply this patch:"
#warning "switchdev: Require RTNL mutex to be held when sending FDB notifications"
#warning "and uncomment #define DEIPCE_SWITCHDEV_RTNL_FIXED (or update kernel)."
    // Deadlock is still possible.
    rtnl_unlock();
    ret = call_switchdev_notifiers(val, dev, info);
    rtnl_lock();
#else
    ret = call_switchdev_notifiers(val, dev, info);
#endif

    return ret;
}

/**
 * Get switch ID.
 * @param dp FES device privates.
 * @param phys_id Place for switch ID.
 */
static int deipce_switchdev_id(struct deipce_dev_priv *dp,
                               struct netdev_phys_item_id *phys_id)
{
    int ret;

    // "<driver name><device number>"
    ret = snprintf(&phys_id->id[0], sizeof(phys_id->id), "%s%u",
                   DRV_NAME, dp->dev_num);
    if (ret < 0 || ret >= sizeof(phys_id->id))
        return -ENOBUFS;

    phys_id->id_len = ret;

    return 0;
}

/**
 * Get physical port name, unique within the switch.
 * @param netdev FES port netdevice.
 * @param name Place for physical name.
 * @param length Number of bytes room in name.
 */
int deipce_switchdev_get_phys_port_name(struct net_device *netdev,
                                        char *name, size_t length)
{
    int ret = -ENOBUFS;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;

    ret = snprintf(name, length, "p%u", pp->port_num);
    if (ret < 0 || ret >= length)
        return -ENOBUFS;

    return 0;
}

/**
 * Convert ageing time from register value representation to clock_t.
 * @param ageing Ageing time from DE-IP Core Edge.
 * @return Ageing time in clock_t.
 */
static inline clock_t deipce_ageing_time(uint16_t ageing)
{
    unsigned int msec = ((unsigned int)ageing + 1) * 16000;

    return jiffies_to_clock_t(msecs_to_jiffies(msec));
}

/**
 * Convert ageing time from clock_t to register value representation.
 * @param ageing_time Ageing time in clock_t.
 * @return Ageing time for DE-IP Core Edge.
 */
static inline uint16_t deipce_ageing_value(clock_t ageing_time)
{
    unsigned int sec =
        jiffies_to_msecs(clock_t_to_jiffies(ageing_time))/1000;
    unsigned int ageing;

    // Select closest from (n + 1) * 16 seconds.
    if (sec < 16)
        return 0;

    ageing = (sec - 16 + 8)/16;
    if (ageing > FRS_AGING_MASK)
        return FRS_AGING_MASK;

    return (uint16_t)ageing;
}

/**
 * Write ageing time to switch. Use fastest ageing time of what is requested
 * for ports on the switch.
 * @param dp Device privates.
 */
static int deipce_switchdev_set_ageing(struct deipce_dev_priv *dp)
{
    struct deipce_port_priv *pp = NULL;
    struct deipce_netdev_priv *np = NULL;
    clock_t ageing_time = deipce_ageing_time(FRS_AGING_MASK);
    uint16_t data;
    unsigned int i;

    // Determine fastest ageing time.
    for (i = 0; i < dp->num_of_ports; i++) {
        pp = dp->port[i];
        if (!pp)
            continue;

        np = netdev_priv(pp->netdev);
        if (ageing_time > np->switchdev.ageing_time)
            ageing_time = np->switchdev.ageing_time;
        dev_dbg(dp->this_dev, "%s() Ageing time port %s %lu switch %lu\n",
                __func__, netdev_name(pp->netdev),
                np->switchdev.ageing_time, ageing_time);
    }

    if (dp->switchdev.ageing_time != ageing_time) {
        dp->switchdev.ageing_time = ageing_time;

        data = deipce_read_switch_reg(dp, FRS_REG_AGING);
        data &= ~FRS_AGING_MASK;
        data |= deipce_ageing_value(ageing_time);

        deipce_write_switch_reg(dp, FRS_REG_AGING, data);

        dev_dbg(dp->this_dev, "%s() Change ageing time to %lu -> %u (%lu)\n",
                __func__, ageing_time,
                deipce_ageing_value(ageing_time),
                deipce_ageing_time(deipce_ageing_value(ageing_time)));
    }

    return 0;
}

/**
 * Switchdev function to get attributes.
 * @param netdev FES port netdevice.
 * @param attr Attribute to get.
 */
static int deipce_switchdev_port_attr_get(struct net_device *netdev,
                                          struct switchdev_attr *attr)
{
    int ret = -EOPNOTSUPP;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    netdev_dbg(netdev, "%s() id 0x%x\n", __func__, attr->id);

    switch (attr->id) {
    case SWITCHDEV_ATTR_ID_PORT_PARENT_ID:
        ret = deipce_switchdev_id(dp, &attr->u.ppid);
        break;
    case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
        attr->u.stp_state = deipce_get_port_stp_state(netdev);
        ret = 0;
        break;
    case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
        attr->u.brport_flags = np->switchdev.brport_flags;
        ret = 0;
        break;
    case SWITCHDEV_ATTR_ID_BRIDGE_AGEING_TIME:
        mutex_lock(&dp->common_reg_lock);
        attr->u.ageing_time = np->switchdev.ageing_time;
        mutex_unlock(&dp->common_reg_lock);
        break;
    default:
        return -EOPNOTSUPP;
    }

    return ret;
}

/**
 * Switchdev function to set attributes.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param attr Attribute to set.
 */
static int deipce_switchdev_port_attr_set(struct net_device *netdev,
                                          const struct switchdev_attr *attr,
                                          struct switchdev_trans *trans)

{
    int ret = -EOPNOTSUPP;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    switch (attr->id) {
    case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
        netdev_dbg(netdev, "%s() set STP state to %u ph %s\n",
                   __func__, attr->u.stp_state,
                   deipce_switchdev_trans_str(trans));
        if (!switchdev_trans_ph_prepare(trans))
            ret = deipce_set_port_stp_state(netdev, attr->u.stp_state);
        else
            ret = 0;
        break;
    case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
        netdev_dbg(netdev, "%s() set flags to 0x%lx %s\n",
                   __func__, attr->u.brport_flags,
                   deipce_switchdev_trans_str(trans));
        // Learning and flooding cannot be disabled.
        if ((attr->u.brport_flags & (BR_LEARNING | BR_FLOOD)) !=
            (BR_LEARNING | BR_FLOOD))
            return -EINVAL;
        if (!switchdev_trans_ph_prepare(trans))
            np->switchdev.brport_flags = attr->u.brport_flags;
        ret = 0;
        break;
    case SWITCHDEV_ATTR_ID_BRIDGE_AGEING_TIME:
        netdev_dbg(netdev,
                   // ageing_time changed to clock_t in Linux 4.7
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
                   "%s() Set ageing time to %u ph %s\n",
#else
                   "%s() Set ageing time to %lu ph %s\n",
#endif
                   __func__, attr->u.ageing_time,
                   deipce_switchdev_trans_str(trans));
        if (!switchdev_trans_ph_prepare(trans)) {
            mutex_lock(&dp->common_reg_lock);
            np->switchdev.ageing_time = attr->u.ageing_time;
            ret = deipce_switchdev_set_ageing(dp);
            mutex_unlock(&dp->common_reg_lock);
        }
        else {
            ret = 0;
        }
        break;
    default:
        netdev_dbg(netdev, "%s() id 0x%x not supported ph %s\n",
                   __func__, attr->id, deipce_switchdev_trans_str(trans));
        return -EOPNOTSUPP;
    }

    return ret;
}

/**
 * Add static FDB entry.
 * This writes an SMAC table entry, updating an existing entry
 * or using an unused entry, if possible.
 * @param netdev FES port netdevice.
 * @param fdb FDB information.
 * @param trans Switchdev transaction information.
 */
static int deipce_switchdev_port_fdb_add(
        struct net_device *netdev,
        const struct switchdev_obj_port_fdb *fdb,
        struct switchdev_trans *trans)
{
    int ret = -EIO;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    struct frs_smac_table_entry old_entry = { .column = 0 };
    struct frs_smac_table_entry new_entry = {
        .column = FRS_SMAC_TABLE_COLS,
        .config = FRS_SMAC_CONFIG_ENABLED,
        .fwd_mask = 1u << pp->port_num,
        .vlan = fdb->vid & VLAN_VID_MASK,
    };
    uint16_t row = 0;
    uint16_t col = FRS_SMAC_TABLE_COLS;

    // Keep VLAN ID always zero if VLAN match is not enabled.
    if (fdb->vid)
        new_entry.config |= FRS_SMAC_CONFIG_VLAN;
    ether_addr_copy(new_entry.mac_address, fdb->addr);

    netdev_dbg(netdev, "%s() ADD %pM VID %hu %s\n",
               __func__, fdb->addr, fdb->vid,
               deipce_switchdev_trans_str(trans));

    mutex_lock(&dp->smac_table_lock);

    ret = deipce_get_smac_pos(dp, &new_entry, &old_entry, &row, &col);
    if (ret)
        goto out;

    new_entry.fwd_mask |= old_entry.fwd_mask;

    if (!switchdev_trans_ph_prepare(trans)) {
        ret = deipce_write_smac_entry(dp, row, col, &new_entry);
        if (ret == 0) {
            deipce_update_smac_usage(dp, row, col,
                                     new_entry.config, old_entry.config);
        }
    }

out:
    mutex_unlock(&dp->smac_table_lock);

    return ret;
}

/**
 * Add VLAN entries.
 * This updates FES VLAN configuration.
 * @param netdev FES port netdevice.
 * @param vlan VLAN information.
 * @param trans Switchdev transaction information.
 */
static int deipce_switchdev_port_vlan_add(
        struct net_device *netdev,
        const struct switchdev_obj_port_vlan *vlan,
        struct switchdev_trans *trans)
{
    int ret = 0;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;

    netdev_dbg(netdev, "%s() ADD %u .. %u flags 0x%x ph %s\n",
               __func__, vlan->vid_begin, vlan->vid_end, vlan->flags,
               deipce_switchdev_trans_str(trans));

    if (switchdev_trans_ph_prepare(trans)) {
        ret = deipce_prepare_add_vlans(dp, vlan->vid_begin, vlan->vid_end,
                                       vlan->flags);
    }
    else {
        deipce_commit_add_vlans(dp, pp, vlan->vid_begin, vlan->vid_end,
                                vlan->flags);
    }

    return ret;
}

/**
 * Switchdev object add callback function.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param obj Object to add.
 * @param trans Switchdev transaction information.
 */
static int deipce_switchdev_port_obj_add(struct net_device *netdev,
                                         const struct switchdev_obj *obj,
                                         struct switchdev_trans *trans)
{
    int ret = 0;

    switch (obj->id) {
    case SWITCHDEV_OBJ_ID_PORT_VLAN:
        ret = deipce_switchdev_port_vlan_add(netdev,
                                             SWITCHDEV_OBJ_PORT_VLAN(obj),
                                             trans);
        break;
    case SWITCHDEV_OBJ_ID_PORT_FDB:
        ret = deipce_switchdev_port_fdb_add(netdev,
                                            SWITCHDEV_OBJ_PORT_FDB(obj),
                                            trans);
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

/**
 * Delete static FDB entry.
 * This updates an SMAC table entry.
 * @param netdev FES port netdevice.
 * @param fdb FDB information.
 */
static int deipce_switchdev_port_fdb_del(
        struct net_device *netdev,
        const struct switchdev_obj_port_fdb *fdb)
{
    int ret = 0;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    uint16_t row = 0;
    struct frs_smac_table_entry upd_entry = { .config = 0 };
    struct frs_smac_table_entry ref_entry = {
        .config = 0,
        .vlan = fdb->vid & VLAN_VID_MASK,
    };
    unsigned int col = 0;

    netdev_dbg(netdev, "%s() DEL %pM VID %hu\n",
               __func__, fdb->addr, fdb->vid);

    // Keep VLAN ID always zero if VLAN match is not enabled.
    if (fdb->vid)
        ref_entry.config |= FRS_SMAC_CONFIG_VLAN;
    ether_addr_copy(ref_entry.mac_address, fdb->addr);

    mutex_lock(&dp->smac_table_lock);

    // Find column.
    for (col = 0; col < FRS_SMAC_TABLE_COLS; col++) {
        ref_entry.column = col;
        row = deipce_get_smac_row(dp, &ref_entry);
        if (!deipce_is_smac_used(dp, row, col))
            continue;

        ret = deipce_read_smac_entry(dp, row, col, &upd_entry);
        if (ret)
            goto out;

        if (!(upd_entry.config & FRS_SMAC_CONFIG_ENABLED))
            continue;

        if (!deipce_match_smac_entries(&ref_entry, &upd_entry))
            continue;

        if (!(upd_entry.fwd_mask & (1u << pp->port_num)))
            continue;

        // Remember original config bits for awhile.
        ref_entry.config = upd_entry.config;

        // Remove for this port. If no ports are left, clear the whole entry.
        upd_entry.fwd_mask &= ~(1u << pp->port_num);
        if (upd_entry.fwd_mask == 0)
            upd_entry = (struct frs_smac_table_entry){ .column = 0 };

        ret = deipce_write_smac_entry(dp, row, col, &upd_entry);
        if (ret)
            goto out;

        deipce_update_smac_usage(dp, row, col,
                                 upd_entry.config, ref_entry.config);

        break;
    }

    // It is okay if not found.

out:
    mutex_unlock(&dp->smac_table_lock);

    return ret;
}

/**
 * Delete VLAN entries.
 * This updates FES VLAN configuration.
 * @param netdev FES port netdevice.
 * @param vlan VLAN information.
 * @param trans Switchdev transaction information.
 */
static int deipce_switchdev_port_vlan_del(
        struct net_device *netdev,
        const struct switchdev_obj_port_vlan *vlan)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;

    netdev_dbg(netdev, "%s() DEL %u .. %u flags 0x%x\n",
               __func__, vlan->vid_begin, vlan->vid_end, vlan->flags);

    deipce_del_vlans(dp, pp, vlan->vid_begin, vlan->vid_end);

    return 0;
}

/**
 * Switchdev object delete callback function.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param obj Object to delete.
 */
static int deipce_switchdev_port_obj_del(struct net_device *netdev,
                                         const struct switchdev_obj *obj)
{
    int ret = 0;

    switch (obj->id) {
    case SWITCHDEV_OBJ_ID_PORT_VLAN:
        ret = deipce_switchdev_port_vlan_del(netdev,
                                             SWITCHDEV_OBJ_PORT_VLAN(obj));
        break;
    case SWITCHDEV_OBJ_ID_PORT_FDB:
        ret = deipce_switchdev_port_fdb_del(netdev,
                                            SWITCHDEV_OBJ_PORT_FDB(obj));
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

/**
 * Dump static FDB information.
 * This gets information from SMAC table.
 * @param netdev FES port netdevice.
 * @param fdb FDB information.
 * @param cb Callback function to use for dumping.
 */
static int deipce_switchdev_port_fdb_dump(struct net_device *netdev,
                                          struct switchdev_obj_port_fdb *fdb,
                                          switchdev_obj_dump_cb_t *cb)
{
    int ret = 0;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    struct frs_smac_table_entry entry = { .config = 0 };
    unsigned int row = 0;
    unsigned int col = 0;

    netdev_dbg(netdev, "%s() DUMP\n", __func__);

    // Dump static FDB entries.
    mutex_lock(&dp->smac_table_lock);

    for (row = 0; row < dp->features.smac_rows; row++) {
        for (col = 0; col < FRS_SMAC_TABLE_COLS; col++) {
            if (!deipce_is_smac_used(dp, row, col))
                continue;

            ret = deipce_read_smac_entry(dp, row, col, &entry);
            if (ret)
                break;

            if (!(entry.config & FRS_SMAC_CONFIG_ENABLED))
                continue;

            if (!(entry.fwd_mask & (1u << pp->port_num)))
                continue;

            ether_addr_copy(fdb->addr, entry.mac_address);
            fdb->ndm_state = NUD_REACHABLE | NUD_NOARP;
            if (entry.config & FRS_SMAC_CONFIG_VLAN)
                fdb->vid = entry.vlan;
            else
                fdb->vid = 0;

            ret = cb(&fdb->obj);
            if (ret)
                break;
        }
    }

    mutex_unlock(&dp->smac_table_lock);

    if (ret)
        return ret;

    return ret;
}

/**
 * Dump port VLAN information.
 * @param netdev FES port netdevice.
 * @param vlan VLAN information.
 * @param cb Callback function to use for dumping.
 */
static int deipce_switchdev_port_vlan_dump(
        struct net_device *netdev,
        struct switchdev_obj_port_vlan *vlan,
        switchdev_obj_dump_cb_t *cb)
{
    int ret = -EOPNOTSUPP;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    uint16_t port_vlan = 0;
    uint16_t pvid = 0;
    uint16_t port_mask = 0;
    uint16_t port_tag = 0;
    uint16_t vid = 0;

    netdev_dbg(netdev, "%s() DUMP\n", __func__);

    port_vlan = deipce_read_port_reg(pp, PORT_REG_VLAN);
    pvid = port_vlan & VLAN_VID_MASK;

    for (vid = 1; vid < VLAN_VID_MASK; vid++) {
        port_mask = deipce_get_vlan_ports(dp, vid);
        if (!(port_mask & (1u << pp->port_num)))
            continue;

        port_tag = deipce_read_switch_reg(dp, FRS_VLAN_TAG(vid));

        vlan->vid_begin = vid;
        vlan->vid_end = vid;
        vlan->flags = 0;

        if (vid == pvid)
            vlan->flags |= BRIDGE_VLAN_INFO_PVID;
        if (!(port_vlan & PORT_VLAN_TAGGED))
            vlan->flags |= BRIDGE_VLAN_INFO_UNTAGGED;
        else if (port_tag & (1u << pp->port_num))
            vlan->flags |= BRIDGE_VLAN_INFO_UNTAGGED;

        ret = cb(&vlan->obj);
        if (ret) {
            netdev_dbg(netdev, "%s() cb failed at VLAN %u error %i\n",
                       __func__, vlan->vid_begin, ret);
            goto out;
        }
    }

    ret = 0;

out:
    return ret;
}

/**
 * Switchdev object dump callback function.
 * Called with rtnl lock held.
 * @param netdev FES port netdevice.
 * @param obj Object to dump.
 * @param cb Callback function to use for dumping.
 */
static int deipce_switchdev_port_obj_dump(struct net_device *netdev,
                                          struct switchdev_obj *obj,
                                          switchdev_obj_dump_cb_t *cb)
{
    int ret = 0;

    switch (obj->id) {
    case SWITCHDEV_OBJ_ID_PORT_VLAN:
        ret = deipce_switchdev_port_vlan_dump(netdev,
                                              SWITCHDEV_OBJ_PORT_VLAN(obj),
                                              cb);
        break;
    case SWITCHDEV_OBJ_ID_PORT_FDB:
        ret = deipce_switchdev_port_fdb_dump(netdev,
                                             SWITCHDEV_OBJ_PORT_FDB(obj),
                                             cb);
        break;
    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

/// Switchdev operations
static const struct switchdev_ops deipce_switchdev_ops = {
    .switchdev_port_attr_get = &deipce_switchdev_port_attr_get,
    .switchdev_port_attr_set = &deipce_switchdev_port_attr_set,
    .switchdev_port_obj_add = &deipce_switchdev_port_obj_add,
    .switchdev_port_obj_del = &deipce_switchdev_port_obj_del,
    .switchdev_port_obj_dump = &deipce_switchdev_port_obj_dump,
};

/**
 * Initialize switchdev support for FES port netdevice.
 * This is called very early, netdevice is not yet operational.
 * @param netdev FES port netdevice.
 */
int deipce_switchdev_setup_netdev(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);

    netdev->switchdev_ops = &deipce_switchdev_ops;

    netdev->features |= NETIF_F_NETNS_LOCAL; // | NETIF_F_HW_VLAN_CTAG_FILTER

    np->switchdev.brport_flags = DEIPCE_DEFAULT_BR_FLAGS;

    return 0;
}

/**
 * Compare two dynamic MAC address table entries for sorting by FID.
 * @param lhs First entry.
 * @param rhs Second entry.
 * @return -1 if lhs is "smaller" than rhs, 1 if the other way around,
 * and zero if entries are equal.
 */
static inline int deipce_switchdev_cmp_fdb(
        const struct deipce_dmac_entry *lhs,
        const struct deipce_dmac_entry *rhs)
{
    if (lhs->fid < rhs->fid)
        return -1;
    if (lhs->fid > rhs->fid)
        return 1;
    return memcmp(lhs->mac_address, rhs->mac_address, sizeof(lhs->mac_address));
}

/**
 * Insert new dynamic FDB entry to internal FDB.
 * Entries are stored in FID order.
 * @param dp Switch privates.
 * @param new_entry Entry to insert. Must not exist already.
 */
static void deipce_switchdev_insert_fdb(
        struct deipce_dev_priv *dp,
        struct deipce_switchdev_fdb_entry *new_entry)
{
    struct deipce_switchdev *switchdev = &dp->switchdev;
    struct rb_node **link = &switchdev->fdb.rb_node;
    struct rb_node *parent = NULL;
    struct deipce_switchdev_fdb_entry *fdb_entry;
    int ret;

    while (*link) {
        parent = *link;
        fdb_entry = rb_entry(parent, struct deipce_switchdev_fdb_entry, node);
        ret = deipce_switchdev_cmp_fdb(&fdb_entry->dmac, &new_entry->dmac);

        if (ret > 0)
            link = &(*link)->rb_left;
        else
            link = &(*link)->rb_right;
    }

    dev_dbg(dp->this_dev, "%s() Add %pM FID %hu\n",
            __func__, new_entry->dmac.mac_address, new_entry->dmac.fid);

    rb_link_node(&new_entry->node, parent, link);
    rb_insert_color(&new_entry->node, &switchdev->fdb);

    return;
}

/**
 * Find dynamic FDB entry.
 * @param dp Switch privates.
 * @param match Dynamic MAC address table entry to look for.
 * @return Found FDB entry or NULL.
 */
static struct deipce_switchdev_fdb_entry *deipce_switchdev_find_fdb(
        struct deipce_dev_priv *dp,
        const struct deipce_dmac_entry *match)
{
    struct rb_node *node = dp->switchdev.fdb.rb_node;
    struct deipce_switchdev_fdb_entry *fdb_entry;
    int ret;

    while (node) {
        fdb_entry = rb_entry(node, struct deipce_switchdev_fdb_entry, node);
        ret = deipce_switchdev_cmp_fdb(&fdb_entry->dmac, match);

        if (ret > 0)
            node = node->rb_left;
        else if (ret < 0)
            node = node->rb_right;
        else
            return fdb_entry;
    }

    return NULL;
}

/**
 * Determine VID to use for FDB notifications for a given FID.
 * A single VID from all VIDs allocated to FID is used to send FDB
 * notifications for given FID.
 * @param dp Switch privates.
 * @param fid FID number.
 * @param vid Place for VID to be used with FID.
 */
static int deipce_switchdev_get_fdb_notify_vid(struct deipce_dev_priv *dp,
                                               uint16_t fid, uint16_t *vid)
{
    unsigned int new_vid;
    int ret;

    // Use first VID for this FID.
    ret = deipce_fill_fid_vid_map(dp, &new_vid, fid * dp->features.fids, 1);
    if (ret)
        return ret;

    *vid = (uint16_t)new_vid;

    return 0;
}

/**
 * Send switchdev FDB notifications taking allocations to FIDs into account.
 * @param dp Switch privates.
 * @param pp Port privates.
 * @param val Notification.
 * @param fid FID number of FDB entry.
 */
static void deipce_switchdev_notify_fdb(
        struct deipce_dev_priv *dp,
        struct deipce_port_priv *pp,
        unsigned long int val,
        uint16_t fid)
{
    struct deipce_switchdev_notify_fdb *notify = &dp->switchdev.notify;

    if (fid == 0) {
        /*
         * No support for FIDs, send FDB notification of address
         * without VLAN information.
         */
        notify->fdb.vid = 0;
        netdev_dbg(pp->netdev, "%s()   VID %hu\n", __func__, notify->fdb.vid);
        deipce_switchdev_call_notifiers(val, pp->netdev, &notify->fdb.info);
        return;
    }

    if (notify->vid[fid] == 0)
        return;

    notify->fdb.vid = notify->vid[fid];
    netdev_dbg(pp->netdev, "%s()   VID %hu\n", __func__, notify->fdb.vid);
    deipce_switchdev_call_notifiers(val, pp->netdev, &notify->fdb.info);

    return;
}

/**
 * Adapt FDB notification handling for new VID in FID.
 * RTNL mutex must be held when calling this.
 * @param dp Switch privates.
 * @param vid VID number that was allocated to FID.
 * @param fid FID number VID was allocated to.
 */
void deipce_switchdev_add_vlan_fid(struct deipce_dev_priv *dp,
                                   uint16_t vid, uint16_t fid)
{
    struct deipce_switchdev_notify_fdb *notify = &dp->switchdev.notify;

    if (notify->vid[fid] == 0) {
        deipce_switchdev_get_fdb_notify_vid(dp, fid, &notify->vid[fid]);

        dev_dbg(dp->this_dev,
                "%s() Use VID %hu for FID %hu FDB notifications\n",
                __func__, notify->vid[fid], fid);
    }

    return;
}

/**
 * Adapt FDB notification handling when VID is removed from FID.
 * RTNL mutex must be held when calling this.
 * @param dp Switch privates.
 * @param vid VID number that was removed from FID.
 * @param fid FID number VID was allocated to.
 */
void deipce_switchdev_del_vlan_fid(struct deipce_dev_priv *dp,
                                   uint16_t vid, uint16_t fid)
{
    struct deipce_switchdev_notify_fdb *notify = &dp->switchdev.notify;
    struct rb_node *node = rb_first(&dp->switchdev.fdb);
    struct deipce_switchdev_fdb_entry *fdb_entry;
    struct deipce_port_priv *pp;
    struct deipce_netdev_priv *np;
    uint16_t new_vid;
    int ret;

    // Nothing to do if VID is not the one used for notifications.
    if (vid != notify->vid[fid])
        return;

    // Determine new VID for notifications.
    ret = deipce_switchdev_get_fdb_notify_vid(dp, fid, &new_vid);
    if (ret)
        return;

    // Move all notified FDB entries in FID from old VID to new VID.
    while (node) {
        fdb_entry = rb_entry(node, struct deipce_switchdev_fdb_entry, node);
        node = rb_next(node);

        dev_dbg(dp->this_dev, "%s() cmp %pM FID %hu against FID %hu\n",
                __func__, fdb_entry->dmac.mac_address, fdb_entry->dmac.fid,
                fid);

        if (fdb_entry->dmac.fid < fid)
            continue;

        if (fdb_entry->dmac.fid > fid)
            break;

        pp = dp->port[fdb_entry->dmac.port_num];

        // No notifications for nonexistent ports.
        if (!pp || !pp->netdev)
            continue;

        np = netdev_priv(pp->netdev);
        if (!(np->switchdev.brport_flags & BR_LEARNING_SYNC))
            continue;

        notify->fdb.addr = fdb_entry->dmac.mac_address;
        notify->fdb.vid = notify->vid[fid];
        dev_dbg(dp->this_dev, "%s()   FDB DEL %pM VID %hu %s\n",
                __func__, notify->fdb.addr, notify->fdb.vid,
                netdev_name(pp->netdev));
        deipce_switchdev_call_notifiers(SWITCHDEV_FDB_DEL, pp->netdev,
                                        &notify->fdb.info);

        if (new_vid) {
            notify->fdb.vid = new_vid;
            dev_dbg(dp->this_dev, "%s()   FDB ADD %pM VID %hu %s\n",
                    __func__, notify->fdb.addr, notify->fdb.vid,
                    netdev_name(pp->netdev));
            deipce_switchdev_call_notifiers(SWITCHDEV_FDB_ADD, pp->netdev,
                                            &notify->fdb.info);
        }
    }

    // Use new VID in the future.
    notify->vid[fid] = new_vid;

    dev_dbg(dp->this_dev,
            "%s() Use VID %hu for FID %hu FDB notifications\n",
            __func__, notify->vid[fid], fid);

    return;
}

/**
 * Check FES MAC address table entry.
 * Adds new entries to red-black tree if not already there,
 * and marks already existing entries as being still valid.
 * @param dp FES device privates.
 * @param dmac FES dynamic MAC address table entry.
 * @param arg Notifier FDB info to use.
 */
static void deipce_switchdev_check_fdb_entry(struct deipce_dev_priv *dp,
                                             struct deipce_dmac_entry *dmac,
                                             void *arg)
{
    struct deipce_switchdev_notify_fdb *notify = arg;
    struct deipce_port_priv *pp = NULL;
    struct deipce_netdev_priv *np = NULL;
    struct deipce_switchdev_fdb_entry *fdb = NULL;

    dev_dbg(dp->this_dev, "%s() port %u address %pM\n",
            __func__, dmac->port_num, dmac->mac_address);

    // Ignore entries of unknown ports.
    if (dmac->port_num >= DEIPCE_MAX_PORTS)
        return;

    pp = dp->port[dmac->port_num];
    if (!pp) {
        dev_warn(dp->this_dev, "%s() port %u unknown\n",
                 __func__, dmac->port_num);
        return;
    }

    if (!pp->netdev) {
        // This can happen when driver is unloaded.
        dev_warn(dp->this_dev, "%s() port %u netdev missing\n",
                 __func__, dmac->port_num);
        return;
    }

    if (!(pp->flags & DEIPCE_HAS_MASTER))
        return;

    np = netdev_priv(pp->netdev);
    if (!(np->switchdev.brport_flags & BR_LEARNING))
        return;

    fdb = deipce_switchdev_find_fdb(dp, dmac);

    netdev_dbg(pp->netdev, "%s() FDB %s %pM FID %hu\n",
               __func__, fdb ? "REFRESH" : "ADD",
               notify->fdb.addr, dmac->fid);

    if (fdb) {
        fdb->last_seen = notify->scan_count;
    }
    else {
        fdb = kmalloc(sizeof(*fdb), GFP_KERNEL);
        if (!fdb) {
            dev_err(dp->this_dev, "Failed to allocate FDB entry\n");
            return;
        }

        *fdb = (struct deipce_switchdev_fdb_entry){
            .last_seen = notify->scan_count,
            .dmac = *dmac,
        };

        deipce_switchdev_insert_fdb(dp, fdb);
    }

    return;
}

/**
 * FDB notifier work.
 * @param work Work within FES device privates.
 */
static void deipce_switchdev_notify_fdb_work(struct work_struct *work)
{
    int ret = -EIO;
    struct deipce_switchdev_notify_fdb *notify =
        container_of(work, struct deipce_switchdev_notify_fdb, fdb_work.work);
    struct deipce_dev_priv *dp =
        container_of(notify, struct deipce_dev_priv, switchdev.notify);
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_port_priv *pp = NULL;
    struct deipce_netdev_priv *np = NULL;
    struct deipce_switchdev_fdb_entry *fdb_entry = NULL;
    struct deipce_switchdev_fdb_entry *prev_fdb_entry = NULL;
    struct rb_node *node;

    // Locking order: 1. RTNL 2. FES.
    rtnl_lock();

    notify->scan_count++;

    /*
     * Add new and update existing entries, send notifications later
     * in FID order, for performance reasons.
     */
    ret = deipce_get_mac_table(dp, &deipce_switchdev_check_fdb_entry, notify);
    if (ret < 0) {
        dev_dbg(dp->this_dev, "%s() Failed to read MAC address table\n",
                __func__);
    }

    // Inform upper layers of new and aged entries and drop aged entries.
    node = rb_first(&dp->switchdev.fdb);
    while (node) {
        fdb_entry = rb_entry(node, struct deipce_switchdev_fdb_entry, node);
        node = rb_next(node);

        pp = dp->port[fdb_entry->dmac.port_num];

        dev_dbg(dp->this_dev, "%s() FDB check %pM FID %hu %s\n",
                __func__, fdb_entry->dmac.mac_address, fdb_entry->dmac.fid,
                pp ? netdev_name(pp->netdev) : "");

        // No notifications for nonexistent ports.
        if (pp && pp->netdev) {
            np = netdev_priv(pp->netdev);

            if (fdb_entry->last_seen == notify->scan_count) {
                // Entry has not been aged (new or needs refreshing).
                if (np->switchdev.brport_flags & BR_LEARNING_SYNC) {
                    notify->fdb.addr = fdb_entry->dmac.mac_address;
                    netdev_dbg(pp->netdev,
                               "%s()   FDB ADD/REFRESH %pM FID %hu\n",
                               __func__, notify->fdb.addr, fdb_entry->dmac.fid);
                    deipce_switchdev_notify_fdb(dp, pp, SWITCHDEV_FDB_ADD,
                                                fdb_entry->dmac.fid);
                }

                prev_fdb_entry = fdb_entry;
                continue;
            }

            // This entry has been aged by FES.
            if (np->switchdev.brport_flags & BR_LEARNING_SYNC) {
                notify->fdb.addr = fdb_entry->dmac.mac_address;
                netdev_dbg(pp->netdev, "%s()   FDB DEL %pM FID %hu\n",
                           __func__, notify->fdb.addr, fdb_entry->dmac.fid);
                deipce_switchdev_notify_fdb(dp, pp, SWITCHDEV_FDB_DEL,
                                            fdb_entry->dmac.fid);
            }
        }

        dev_dbg(dp->this_dev, "%s()   FDB remove %pM FID %hu\n",
                __func__, fdb_entry->dmac.mac_address, fdb_entry->dmac.fid);

        rb_erase(&fdb_entry->node, &dp->switchdev.fdb);
        kfree(fdb_entry);

        // Tree may have been rebalanced, must find next from the beginning.
        if (prev_fdb_entry) {
            fdb_entry = deipce_switchdev_find_fdb(dp, &prev_fdb_entry->dmac);
            node = rb_next(&fdb_entry->node);
        }
        else {
            node = rb_first(&dp->switchdev.fdb);
        }
    }

    rtnl_unlock();

    queue_delayed_work(drv->wq_low, &dp->switchdev.notify.fdb_work,
                       DEIPCE_SWITCHDEV_NOTIFY_FDB_INTERVAL);

    return;
}

/**
 * Setup FES port for switchdev support when enabling it.
 * This is called when port netdevice is brought up.
 * @param netdev FES port netdevice.
 */
int deipce_switchdev_enable(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    struct deipce_drv_priv *drv = deipce_get_drv_priv();

    netdev_dbg(netdev, "%s()\n", __func__);

    if (dp->switchdev.enabled_port_count++ == 0) {
        dev_dbg(dp->this_dev, "%s() Start polling MAC address table\n",
                __func__);
        queue_delayed_work(drv->wq_low, &dp->switchdev.notify.fdb_work,
                           DEIPCE_SWITCHDEV_NOTIFY_FDB_INTERVAL);
    }

    return 0;
}

/**
 * Cleanup FES port for switchdev support when disabling it.
 * This is called when port netdevice is brought down.
 * @param netdev FES port netdevice.
 */
void deipce_switchdev_disable(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    netdev_dbg(netdev, "%s()\n", __func__);

    // Retain learned FDB entries as long as FES remembers them.

    if (--dp->switchdev.enabled_port_count == 0) {
        dev_dbg(dp->this_dev, "%s() Stop polling MAC address table\n",
                __func__);
        cancel_delayed_work_sync(&dp->switchdev.notify.fdb_work);
    }

    return;
}

/**
 * Initialize port to defaults when adding it to bridge.
 * - Remove port from all VLANs.
 * - Map VLAN ID 0 to 1.
 * - Set default port VLAN ID to 1.
 * @param netdev FES port netdevice.
 * @param master Master netdevice (bridge we are joining).
 */
static int deipce_switchdev_join_bridge(struct net_device *netdev,
                                        struct net_device *master)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    int ret = 0;

    // Refuse to join ports of the same switch to different bridges.
    if (dp->switchdev.master && dp->switchdev.master != master)
        return -EINVAL;

    // Log this.
    netdev_info(netdev, "Joining %s, resetting port config\n",
                netdev_name(master));

    // Remove port from all VLANs.
    deipce_reset_port_vlan_config(pp, false);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
    switchdev_port_fwd_mark_set(netdev, master, true);
#endif

    np->switchdev.brport_flags = DEIPCE_DEFAULT_BR_FLAGS;

    if (dp->switchdev.join_count++ == 0) {
        dp->switchdev.master = master;
        ret = deipce_bridge_sysfs_init(dp);
        if (ret)
            goto err_sysfs;
    }

    return 0;

err_sysfs:
    dp->switchdev.master = NULL;
    dp->switchdev.join_count--;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
    switchdev_port_fwd_mark_set(netdev, master, false);
#endif

    return ret;
}

/**
 * Initialize port to defaults when removing it from bridge.
 * This uses FES defaults, for compatibility without switchdev support.
 * - Add port to all VLANs.
 * - Map VLAN ID 0 to 0.
 * - Set default port VLAN ID to 4095.
 * @param netdev FES port netdevice.
 * @param master Master netdevice (bridge we are leaving).
 */
static int deipce_switchdev_leave_bridge(struct net_device *netdev,
                                         struct net_device *master)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    int ret = 0;

    // Log this.
    netdev_info(netdev, "Leaving %s, restoring default port config\n",
                netdev_name(master));

    if (--dp->switchdev.join_count == 0) {
        deipce_bridge_sysfs_cleanup(dp);
        dp->switchdev.master = NULL;
    }

    // Reset port VLAN configuration to driver init time defaults.
    deipce_reset_port_vlan_config(pp, true);

    // Restore also port STP state.
    ret = deipce_set_port_stp_state(netdev, BR_STATE_FORWARDING);
    if (ret)
        return ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
    switchdev_port_fwd_mark_set(netdev, master, false);
#endif

    np->switchdev.brport_flags = DEIPCE_DEFAULT_BR_FLAGS;

    return 0;
}

/**
 * Handle netdevice notifications to detect port topology changes.
 * @param nb Notifier block.
 * @paramm event Type of notification event.
 * @param arg Notifier block type specific argument.
 * @return One of NOTIFY_xxx values.
 */
static int deipce_switchdev_netdev_event(struct notifier_block *nb,
                                         unsigned long int event, void *arg)
{
    struct net_device *netdev = netdev_notifier_info_to_dev(arg);
    struct netdev_notifier_changeupper_info *info = NULL;
    struct deipce_netdev_priv *np = NULL;
    struct deipce_port_priv *pp = NULL;
    int ret = NOTIFY_DONE;

    if (!deipce_is_port(netdev))
        return NOTIFY_DONE;

    switch (event) {
    case NETDEV_CHANGEUPPER:
        info = arg;
        if (!info->master)
            break;
        np = netdev_priv(netdev);
        pp = np->pp;
        if (info->linking) {
            pp->flags |= DEIPCE_HAS_MASTER;
            ret = deipce_switchdev_join_bridge(netdev, info->upper_dev);
        }
        else {
            pp->flags &= ~DEIPCE_HAS_MASTER;
            ret = deipce_switchdev_leave_bridge(netdev, info->upper_dev);
        }
        if (ret) {
            netdev_warn(netdev,
                        "Failed to reflect master change (error %i)\n",
                        ret);
            ret = NOTIFY_BAD;
        }
        else {
            ret = NOTIFY_OK;
        }
        break;
    }

    return ret;
}

/**
 * Initialize switchdev functionality for switch.
 * @param dp Switch privates.
 */
int deipce_switchdev_init_switch(struct deipce_dev_priv *dp)
{
    struct deipce_switchdev_notify_fdb *notify = &dp->switchdev.notify;
    struct netdev_phys_item_id phys_id = { .id_len = 0 };
    uint16_t data;

    dp->switchdev.fdb = RB_ROOT;
    INIT_DELAYED_WORK(&notify->fdb_work, &deipce_switchdev_notify_fdb_work);

    if (dp->features.fids > 0) {
        notify->vid = kzalloc((dp->features.fids + 1) * sizeof(*notify->vid),
                              GFP_KERNEL);
        if (!notify->vid)
            return -ENOMEM;
    }

    // Print switch ID in sysfs phys_switch_id format.
    deipce_switchdev_id(dp, &phys_id);
    dev_info(dp->this_dev, "Switch with id %*phN\n",
             (int)phys_id.id_len, phys_id.id);

    // Record ageing time.
    data = deipce_read_switch_reg(dp, FRS_REG_AGING);
    dp->switchdev.ageing_time = deipce_ageing_time(data & FRS_AGING_MASK);

    return 0;
}

/**
 * Initialize switchdev functionality for switch port.
 * @param dp Switch privates.
 * @param pp Port privates.
 */
int deipce_switchdev_init_port(struct deipce_dev_priv *dp,
                               struct deipce_port_priv *pp)
{
    struct deipce_netdev_priv *np = netdev_priv(pp->netdev);

    np->switchdev.ageing_time = dp->switchdev.ageing_time;

    return 0;
}

/**
 * Cleanup switchdev functionality for switch.
 * @param dp Switch privates.
 */
void deipce_switchdev_cleanup_switch(struct deipce_dev_priv *dp)
{
    struct deipce_switchdev_fdb_entry *fdb_entry;
    struct deipce_switchdev_fdb_entry *tmp;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    dev_dbg(dp->this_dev, "%s() Stop polling MAC address table\n", __func__);
    cancel_delayed_work_sync(&dp->switchdev.notify.fdb_work);

    rbtree_postorder_for_each_entry_safe(fdb_entry, tmp,
                                         &dp->switchdev.fdb, node) {
        kfree(fdb_entry);
    }

    if (dp->switchdev.notify.vid) {
        kfree(dp->switchdev.notify.vid);
        dp->switchdev.notify.vid = NULL;
    }

    return;
}

/// Notifier block for receiving netdevice notifications
static struct notifier_block deipce_switchdev_netdev_nb __read_mostly = {
    .notifier_call = &deipce_switchdev_netdev_event,
};

/**
 * Initialize driver switchdev support.
 */
int deipce_switchdev_init_driver(void)
{
    int ret = -EOPNOTSUPP;

    ret = register_netdevice_notifier(&deipce_switchdev_netdev_nb);

    return ret;
}

/**
 * Cleanup driver switchdev support.
 */
void deipce_switchdev_cleanup_driver(void)
{
    unregister_netdevice_notifier(&deipce_switchdev_netdev_nb);

    return;
}

