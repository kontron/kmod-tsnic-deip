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

    data = deipce_read_switch_reg(dp, FRS_REG_AGING);
    data &= ~FRS_AGING_MASK;
    data |= deipce_ageing_value(ageing_time);

    deipce_write_switch_reg(dp, FRS_REG_AGING, data);

    dev_dbg(dp->this_dev, "%s() Set ageing time %lu -> %u (%lu)\n",
            __func__, ageing_time,
            deipce_ageing_value(ageing_time),
            deipce_ageing_time(deipce_ageing_value(ageing_time)));

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
        mutex_lock(&np->link_mode_lock);
        attr->u.stp_state = np->stp_state;
        mutex_unlock(&np->link_mode_lock);
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
    struct deipce_port_priv *pp = np->pp;

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
        // Do not allow learning and flooding unicast on internal ports.
        if (pp->flags & DEIPCE_PORT_CPU) {
            if (attr->u.brport_flags &
                (BR_LEARNING | BR_LEARNING_SYNC | BR_FLOOD))
                return -EINVAL;
        }
        else {
            // Learning and flooding cannot be disabled.
            if ((attr->u.brport_flags & (BR_LEARNING | BR_FLOOD)) !=
                (BR_LEARNING | BR_FLOOD))
                return -EINVAL;
        }
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

    if (dp->features.smac_rows == 0)
        return -EOPNOTSUPP;

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
    int ret = -EINVAL;
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    bool port_vlan_change = false;
    uint16_t port_vlan = 0;
    uint16_t port_vlan0_map = 0;
    uint16_t port_mask = 0;
    uint16_t vid = 0;

    netdev_dbg(netdev, "%s() ADD %u .. %u flags 0x%x ph %s\n",
               __func__, vlan->vid_begin, vlan->vid_end, vlan->flags,
               deipce_switchdev_trans_str(trans));

    // VLAN membership configuration. Cannot fail.
    if (!switchdev_trans_ph_prepare(trans)) {
        for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {
            port_mask = deipce_read_switch_reg(dp, FRS_VLAN_CFG(vid));
            port_mask |= 1u << pp->port_num;
            deipce_write_switch_reg(dp, FRS_VLAN_CFG(vid), port_mask);
        }
    }

    // VLAN flags are BRIDGE_VLAN_INFO_xxx values.

    // Untagging tagged frames at egress configuration.
    if (vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED) {
        port_vlan = deipce_read_port_reg(pp, PORT_REG_VLAN);

        /*
         * FES removes VLAN tag from either only the default VLAN or
         * from all VLANs at egress.
         * VLAN ID 4095 is actually not allowed.
         */
        if (vlan->vid_begin <= 1 && vlan->vid_end >= VLAN_VID_MASK - 1) {
            port_vlan0_map = vlan->vid_begin;
            port_vlan &= ~PORT_VLAN_TAGGED;
            port_vlan |= VLAN_VID_MASK;
        }
        else if (vlan->vid_begin == vlan->vid_end) {
            port_vlan0_map = vlan->vid_begin & VLAN_VID_MASK;
            port_vlan |= PORT_VLAN_TAGGED;
            port_vlan &= ~VLAN_VID_MASK;
            port_vlan |= port_vlan0_map;
        }
        else {
            // FES does not support removing tag from arbitrary VLANs.
            ret = -EINVAL;
            goto error;
        }

        port_vlan_change = true;
    }

    // Tagging untagged frames at ingress configuration.
    if (vlan->flags & BRIDGE_VLAN_INFO_PVID) {
        // There can be only one.
        if (vlan->vid_begin != vlan->vid_end) {
            ret = -EINVAL;
            goto error;
        }

        port_vlan = deipce_read_port_reg(pp, PORT_REG_VLAN);

         // PVID defines also VLAN ID for frames with VLAN ID 0.
        port_vlan0_map = vlan->vid_begin & VLAN_VID_MASK;

        port_vlan |= PORT_VLAN_TAGGED;
        port_vlan &= ~VLAN_VID_MASK;
        port_vlan |= port_vlan0_map;

        port_vlan_change = true;
    }

    if (!switchdev_trans_ph_prepare(trans) && port_vlan_change) {
        deipce_write_port_reg(pp, PORT_REG_VLAN, port_vlan);
        deipce_write_port_reg(pp, PORT_REG_VLAN0_MAP, port_vlan0_map);
    }

    ret = 0;

error:
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

    if (dp->features.smac_rows == 0)
        return -EOPNOTSUPP;

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
 * Add VLAN entries.
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
    uint16_t port_mask = 0;
    uint16_t vid = 0;

    netdev_dbg(netdev, "%s() DEL %u .. %u flags 0x%x\n",
               __func__, vlan->vid_begin, vlan->vid_end, vlan->flags);

    // VLAN membership configuration.
    for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {
        port_mask = deipce_read_switch_reg(dp, FRS_VLAN_CFG(vid));
        port_mask &= ~(1u << pp->port_num);
        deipce_write_switch_reg(dp, FRS_VLAN_CFG(vid), port_mask);
    }

    // Port VLAN configuration: nothing to do.

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
    struct deipce_switchdev_fdb_entry *fdb_entry = NULL;
    int bkt;

    if (dp->features.smac_rows == 0)
        return -EOPNOTSUPP;

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
            fdb->ndm_state = NUD_REACHABLE;
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

    // Dump learned FDB entries.
    hash_for_each(dp->switchdev.fdb, bkt, fdb_entry, hlist) {
        if (fdb_entry->key.port_num != pp->port_num)
            continue;

        ether_addr_copy(fdb->addr, fdb_entry->key.mac_address);
        fdb->ndm_state = NUD_REACHABLE;
        fdb->vid = 0;

        ret = cb(&fdb->obj);
        if (ret)
            break;
    }

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
    uint16_t port_mask = 0;
    uint16_t vid = 0;

    netdev_dbg(netdev, "%s() DUMP\n", __func__);

    port_vlan = deipce_read_port_reg(pp, PORT_REG_VLAN);

    // VLAN ID 4095 is not allowed here?
    for (vid = 1; vid < VLAN_N_VID; vid++) {
        port_mask = deipce_read_switch_reg(dp, FRS_VLAN_CFG(vid));
        if (!(port_mask & (1u << pp->port_num)))
            continue;

        vlan->vid_begin = vid;
        vlan->vid_end = vid;
        vlan->flags = 0;

        if (vid == (port_vlan & VLAN_VID_MASK))
            vlan->flags |= BRIDGE_VLAN_INFO_UNTAGGED | BRIDGE_VLAN_INFO_PVID;
        if (!(port_vlan & PORT_VLAN_TAGGED))
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
 * Helper function to calculate hash for an FDB entry.
 * @param dmac FES MAC address table entry.
 * @return Hash key for FDB hash table.
 */
static inline uint32_t deipce_switchdev_calc_fdb_hkey(
        const struct deipce_dmac_entry *dmac)
{
    return crc32(~0, dmac, sizeof(*dmac));
}

/**
 * Find static FDB entry from hash table.
 * @param dp FES device privates.
 * @param match MAC address and port number of FDB entry to find.
 * @return Existing FDB entry or NULL.
 */
static struct deipce_switchdev_fdb_entry *deipce_switchdev_find_fdb(
        struct deipce_dev_priv *dp,
        const struct deipce_dmac_entry *match)
{
    struct deipce_switchdev_fdb_entry *fdb = NULL;
    uint32_t hkey = deipce_switchdev_calc_fdb_hkey(match);

    hash_for_each_possible(dp->switchdev.fdb, fdb, hlist, hkey) {
        if (memcmp(&fdb->key, match, sizeof(fdb->key)) == 0)
            return fdb;
    }

    return NULL;
}

/**
 * Check FES MAC address table entry.
 * Notifies upper layers when new entry is learned,
 * and marks existing entries as still valid.
 * @param dp FES device privates.
 * @param dmac FES dynamic MAC address table entry.
 * @param arg Notifier FDB info to use.
 */
static void deipce_switchdev_check_fdb_entry(struct deipce_dev_priv *dp,
                                             struct deipce_dmac_entry *dmac,
                                             void *arg)
{
    struct switchdev_notifier_fdb_info *info = arg;
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

    // Do not learn MAC addresses from CPU port, crashes Linux 4.4.
    if (pp->flags & DEIPCE_PORT_CPU)
        return;

    np = netdev_priv(pp->netdev);
    if (!(np->switchdev.brport_flags & BR_LEARNING))
        return;

    info->addr = dmac->mac_address;
    info->vid = 0;

    fdb = deipce_switchdev_find_fdb(dp, dmac);

    netdev_dbg(pp->netdev, "%s() FDB %s %pM VID %hu\n",
               __func__, fdb ? "REFRESH" : "ADD", info->addr, info->vid);

    if (fdb) {
        fdb->last_seen = dp->switchdev.fdb_counter;
    }
    else {
        fdb = kmalloc(sizeof(*fdb), GFP_KERNEL);
        if (!fdb) {
            dev_err(dp->this_dev, "Failed to allocate FDB entry\n");
            return;
        }

        *fdb = (struct deipce_switchdev_fdb_entry){
            .hkey = deipce_switchdev_calc_fdb_hkey(dmac),
            .last_seen = dp->switchdev.fdb_counter,
            .key = *dmac,
        };

        hash_add(dp->switchdev.fdb, &fdb->hlist, fdb->hkey);
    }

    if (!(np->switchdev.brport_flags & BR_LEARNING_SYNC))
        return;

    deipce_switchdev_call_notifiers(SWITCHDEV_FDB_ADD, pp->netdev,
                                    &info->info);

    return;
}

/**
 * FDB notifier work.
 * @param work Work within FES device privates.
 */
static void deipce_switchdev_notify_fdb_work(struct work_struct *work)
{
    int ret = -EIO;
    struct deipce_dev_priv *dp =
        container_of(work, struct deipce_dev_priv, switchdev.notify_fdb.work);
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_port_priv *pp = NULL;
    struct deipce_netdev_priv *np = NULL;
    struct switchdev_notifier_fdb_info info = {
        .vid = 0,
    };
    struct deipce_switchdev_fdb_entry *fdb_entry = NULL;
    struct hlist_node *tmp = NULL;
    int bkt;

    // Locking order: 1. RTNL 2. FES.
    rtnl_lock();

    dp->switchdev.fdb_counter++;

    ret = deipce_get_mac_table(dp, &deipce_switchdev_check_fdb_entry, &info);
    if (ret < 0) {
        dev_dbg(dp->this_dev, "%s() Failed to read MAC address table\n",
                __func__);
    }

    // Inform upper layers of aged entries and drop them.
    hash_for_each_safe(dp->switchdev.fdb, bkt, tmp, fdb_entry, hlist) {
        if (fdb_entry->last_seen == dp->switchdev.fdb_counter)
            continue;

        // This entry has been aged by FES.
        pp = dp->port[fdb_entry->key.port_num];

        // No notifications for nonexistent ports.
        if (pp && pp->netdev) {
            np = netdev_priv(pp->netdev);
            if (np->switchdev.brport_flags & BR_LEARNING_SYNC) {
                info.addr = fdb_entry->key.mac_address;
                netdev_dbg(pp->netdev, "%s() FDB DEL %pM VID %hu\n",
                           __func__, info.addr, info.vid);
                deipce_switchdev_call_notifiers(SWITCHDEV_FDB_DEL,
                                                pp->netdev,
                                                &info.info);
            }
        }

        hash_del(&fdb_entry->hlist);
        kfree(fdb_entry);
    }

    rtnl_unlock();

    queue_delayed_work(drv->wq_low, &dp->switchdev.notify_fdb,
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
        queue_delayed_work(drv->wq_low, &dp->switchdev.notify_fdb,
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
        cancel_delayed_work_sync(&dp->switchdev.notify_fdb);
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

    // Log this.
    netdev_info(netdev, "Joining %s, resetting port config\n",
                netdev_name(master));

    // Remove port from all VLANs.
    deipce_reset_port_vlan_config(pp, false);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
    switchdev_port_fwd_mark_set(netdev, master, true);
#endif

    // Do not allow learning and flooding unicast on internal ports.
    if (pp->flags & DEIPCE_PORT_CPU)
        np->switchdev.brport_flags = 0;
    else
        np->switchdev.brport_flags = DEIPCE_DEFAULT_BR_FLAGS;

    return 0;
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
    int ret = 0;

    // Log this.
    netdev_info(netdev, "Leaving %s, restoring default port config\n",
                netdev_name(master));

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
            goto out;
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
        if (ret)
            netdev_warn(netdev,
                        "Failed to reflect master change (error %i)\n",
                        ret);
        break;
    }

out:
    return NOTIFY_DONE;
}

/**
 * Initialize FES device for switchdev support.
 * @param dp FES device privates.
 */
int deipce_switchdev_init_device(struct deipce_dev_priv *dp)
{
    struct netdev_phys_item_id phys_id = { .id_len = 0 };
    struct deipce_port_priv *pp = NULL;
    struct deipce_netdev_priv *np = NULL;
    clock_t ageing_time = 0;
    uint16_t data;
    unsigned int i;

    hash_init(dp->switchdev.fdb);
    INIT_DELAYED_WORK(&dp->switchdev.notify_fdb,
                      &deipce_switchdev_notify_fdb_work);

    // Print switch ID in sysfs phys_switch_id format.
    deipce_switchdev_id(dp, &phys_id);
    dev_info(dp->this_dev, "Switch with id %*phN\n",
             (int)phys_id.id_len, phys_id.id);

    // Record ageing time.
    data = deipce_read_switch_reg(dp, FRS_REG_AGING);
    ageing_time = deipce_ageing_time(data & FRS_AGING_MASK);

    for (i = 0; i < dp->num_of_ports; i++) {
        pp = dp->port[i];
        if (!pp)
            continue;

        np = netdev_priv(pp->netdev);
        np->switchdev.ageing_time = ageing_time;
    }

    return 0;
}

/**
 * Cleanup FES device switchdev support.
 * @param dp FES device privates.
 */
void deipce_switchdev_cleanup_device(struct deipce_dev_priv *dp)
{
    struct deipce_switchdev_fdb_entry *fdb_entry = NULL;
    struct hlist_node *tmp = NULL;
    int bkt;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    dev_dbg(dp->this_dev, "%s() Stop polling MAC address table\n", __func__);
    cancel_delayed_work_sync(&dp->switchdev.notify_fdb);

    hash_for_each_safe(dp->switchdev.fdb, bkt, tmp, fdb_entry, hlist) {
        hash_del(&fdb_entry->hlist);
        kfree(fdb_entry);
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

