/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2013 Flexibilis Oy

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

#ifndef DEIPCE_SWITCHDEV_H
#define DEIPCE_SWITCHDEV_H

#include <linux/jiffies.h>
#include <linux/version.h>

#if IS_ENABLED(CONFIG_NET_SWITCHDEV)

#include <linux/rbtree.h>
#include <net/switchdev.h>

#include "deipce_hw_type.h"

struct deipce_dev_priv;
struct deipce_port_priv;

/**
 * Learned FDB entry for FES MAC table addresses.
 */
struct deipce_switchdev_fdb_entry {
    struct rb_node node;                ///< stored in a red-black tree
    unsigned int last_seen;             ///< last seen counter value
    struct deipce_dmac_entry dmac;      ///< dynamic MAC address table entry
};

/**
 * FDB notification information.
 */
struct deipce_switchdev_notify_fdb {
    struct delayed_work fdb_work;       ///< notify work for FDB changes
    unsigned int scan_count;            ///< current FDB scan number
    struct switchdev_notifier_fdb_info fdb;    ///< FDB notification
    uint16_t *vid;                      ///< VID to use in FDB notifications,
                                        ///< one for each FID
};

/**
 * FES context for switchdev support.
 */
struct deipce_switchdev {
    clock_t ageing_time;                ///< current ageing time
    unsigned int enabled_port_count;    ///< number of enabled port netdevices
    struct rb_root fdb;                 ///< learned FDB entries,
                                        ///< synchronized via RTNL lock
    struct deipce_switchdev_notify_fdb notify;  ///< FDB notification info
    unsigned int join_count;            ///< number of ports joined to master
    struct net_device *master;          ///< master (Linux bridge) device
};

/**
 * FES port context for switchdev support.
 */
struct deipce_switchdev_port {
    unsigned long int brport_flags;     ///< bridge port flags
    clock_t ageing_time;                ///< requested ageing time
};

int deipce_switchdev_init_driver(void);
int deipce_switchdev_init_switch(struct deipce_dev_priv *dp);
int deipce_switchdev_init_port(struct deipce_dev_priv *dp,
                               struct deipce_port_priv *pp);
int deipce_switchdev_setup_netdev(struct net_device *netdev);
int deipce_switchdev_enable(struct net_device *netdev);
void deipce_switchdev_disable(struct net_device *netdev);
void deipce_switchdev_add_vlan_fid(struct deipce_dev_priv *dp,
                                   uint16_t vid, uint16_t fid);
void deipce_switchdev_del_vlan_fid(struct deipce_dev_priv *dp,
                                   uint16_t vid, uint16_t fid);

int deipce_switchdev_get_phys_port_name(struct net_device *netdev,
                                        char *name, size_t len);

void deipce_switchdev_cleanup_switch(struct deipce_dev_priv *dp);
void deipce_switchdev_cleanup_driver(void);

#else // CONFIG_NET_SWITCHDEV

struct deipce_dev_priv;
struct deipce_port_priv;

/**
 * Dummy FES context for switchdev support.
 */
struct deipce_switchdev {
};

/**
 * Dummy FES port context for switchdev support.
 */
struct deipce_switchdev_port {
};

static inline int deipce_switchdev_init_driver(void)
{ return 0; }

static inline int deipce_switchdev_init_switch(struct deipce_dev_priv *dp)
{ return 0; }

static inline int deipce_switchdev_init_port(struct deipce_dev_priv *dp,
                                             struct deipce_port_priv *pp)
{ return 0; }

static inline int deipce_switchdev_setup_netdev(struct net_device *netdev)
{ return 0; }

static inline int deipce_switchdev_enable(struct net_device *netdev)
{ return 0; }

static inline void deipce_switchdev_disable(struct net_device *netdev)
{ }

static inline void deipce_switchdev_add_vlan_fid(struct deipce_dev_priv *dp,
                                                 uint16_t vid, uint16_t fid)
{ }

static inline void deipce_switchdev_del_vlan_fid(struct deipce_dev_priv *dp,
                                                 uint16_t vid, uint16_t fid)
{ }

static inline void deipce_switchdev_cleanup_switch(
        struct deipce_dev_priv *dp)
{ }

static inline void deipce_switchdev_cleanup_driver(void)
{ }

#endif // CONFIG_NET_SWITCHDEV

#endif
