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

#ifndef DEIPCE_NETDEV_H
#define DEIPCE_NETDEV_H

#include "deipce_types.h"
#include "deipce_hw_type.h"

/// Link check interval in jiffies
#define DEIPCE_LINK_CHECK_INTERVAL (1*HZ)

// Forward declarations
struct sk_buff;

int deipce_netdev_init(struct deipce_dev_priv *dp,
                       struct deipce_cfg *frs_cfg);
void deipce_netdev_cleanup(struct deipce_dev_priv *dp);
void deipce_rx_frame(struct sk_buff *rx_frame);
int deipce_update_port_mode(struct net_device *netdev,
                            enum link_mode link_mode);
bool deipce_is_port(struct net_device *netdev);

#endif
