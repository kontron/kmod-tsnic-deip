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

#ifndef DEIPCE_HW_TYPE_H
#define DEIPCE_HW_TYPE_H

#include <linux/types.h>
#include <linux/if_ether.h>

/**
 * Link modes.
 */
enum link_mode {
    LM_DOWN,
    LM_10FULL,
    LM_100FULL,
    LM_1000FULL,
};

/// Number of link modes
#define DEIPCE_LINK_MODE_COUNT (LM_1000FULL + 1)

/**
 * FRS dynamic MAC address table entry.
 */
struct deipce_dmac_entry {
    uint16_t port_num;                  ///< port number
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
    uint16_t fid;                       ///< FID number
};

/**
 * Configuration time switch information.
 */
struct deipce_switch_config {
    const char *mac_name;               ///< underlying MAC net device name
    const char *ep_name;                ///< default endpoint net device name
};

/**
 * Configuration time switch port information.
 */
struct deipce_port_config {
    const char *name;                   ///< default switch port name
};

#endif
