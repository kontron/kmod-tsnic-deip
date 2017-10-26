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

#include <linux/phy.h>

/// Maximum number of ports in one FRS
#define DEIPCE_MAX_PORTS 16

/// Size of FRS switch register address region
#define DEIPCE_SWITCH_IOSIZE   0x8000
/// Size of FRS port register address region
#define DEIPCE_PORT_IOSIZE     0x10000
/// Size of FRS port adapter register address region
#define DEIPCE_ADAPTER_IOSIZE  0x100

// Port flags
#define DEIPCE_PORT_ADDR_VALID         (1u << 0)       ///< Port address ok
#define DEIPCE_ADAPTER_ADDR_VALID      (1u << 1)       ///< Adapter address ok
#define DEIPCE_PORT_CPU                (1u << 2)       ///< CPU port
#define DEIPCE_PORT_SPEED_EXT          (1u << 5)       ///< Auto/ext speed sel
#define DEIPCE_ADAPTER_SGMII_PHY_MODE  (1u << 6)       ///< SGMII in PHY mode
#define DEIPCE_SFP_EEPROM              (1u << 7)       ///< Use SFP EEPROM
#define DEIPCE_HAS_PHY                 (1u << 8)       ///< PHY configured
#define DEIPCE_HAS_SFP_PHY             (1u << 9)      ///< SFP PHY configured
#define DEIPCE_HAS_SEPARATE_SFP        (1u << 10)      ///< Separate SFP port
#define DEIPCE_HAS_MASTER              (1u << 11)      ///< Attached to master

// For sanity checks

#define DEIPCE_MAX_PRIO_QUEUES         8       ///< limit for priority queues
#define DEIPCE_MAX_POLICERS            4096    ///< limit for policers

/**
 * Medium types.
 */
enum deipce_medium_type {
    DEIPCE_MEDIUM_NONE = 0,    ///< Indicates unused port
    DEIPCE_MEDIUM_SFP = 1,     ///< SFP module
    DEIPCE_MEDIUM_PHY = 2,     ///< Normal PHY
    DEIPCE_MEDIUM_NOPHY = 5,   ///< External port, no PHY
};

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
 * Traffic delays for both directions.
 */
struct deipce_delay {
    unsigned long int tx;       ///< TX delay in ns
    unsigned long int rx;       ///< RX delay in ns
};

/**
 * FRS port initialization data structure.
 */
struct deipce_port_cfg {
    const char *if_name;        ///< Interface name
    enum deipce_medium_type medium_type;       ///< medium type
    uint32_t baseaddr;          ///< Port register base address
    uint32_t adapter_baseaddr;  ///< Adapter base address
    uint32_t flags;             ///< Flags for driver internal use
    struct device_node *ext_phy_node;   ///< PHY device tree node
    struct device_node *sfp_phy_node;   ///< SFP PHY device tree node
    struct device_node *sfp_eeprom_node;        ///< SFP EEPROM I2C device node
    phy_interface_t ext_phy_if; ///< External PHY interface
    phy_interface_t sfp_phy_if; ///< SFP PHY interface
    struct device_node *fsc_node;       ///< FSC instance
    unsigned int sched_num;     ///< FSC scheduler number
    struct deipce_delay phy_delay[DEIPCE_LINK_MODE_COUNT];
};

/**
 * FRS component initialisation data structure.
 * This information needs to be provided through platform_device
 * platform_data.
 */
struct deipce_cfg {
    const char *mac_name;       ///< MAC, connected to FRS, net_device name
    uint32_t baseaddr;          ///< Switch register base address
    const char *if_name;        ///< Switch (endpoint) interface name
    struct device_node *ibc_node;       ///< IBC node or NULL
    struct device_node *phc_node;       ///< PTP hardware clock node or NULL

    unsigned int num_of_ports;  ///< Number of FRS ports
    struct deipce_port_cfg port[DEIPCE_MAX_PORTS];
};

/**
 * FRS dynamic MAC address table entry.
 */
struct deipce_dmac_entry {
    uint16_t port_num;                  ///< port number
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
};

#endif
