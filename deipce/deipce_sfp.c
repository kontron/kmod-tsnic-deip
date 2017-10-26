/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2015 Flexibilis Oy

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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/i2c.h>

#include "deipce_ethtool.h"
#include "deipce_types.h"
#include "deipce_sfp.h"

/**
 * Convert SFP type to string representation.
 * @param sfp SFP type.
 * @return String representation of SFP type
 */
const char *deipce_sfp_type_str(enum deipce_sfp_type sfp)
{
    switch (sfp) {
    case DEIPCE_SFP_NONE: return "NONE";
    case DEIPCE_SFP_1000BASEX: return "1000Base-X";
    case DEIPCE_SFP_100BASEFX: return "100Base-FX";
    case DEIPCE_SFP_1000BASET: return "1000Base-T";
    case DEIPCE_SFP_100BASET: return "100Base-T";
    case DEIPCE_SFP_UNSUPPORTED: return "UNSUPPORTED";
    }

    return "(invalid)";
}

/**
 * Set FRS port for given SFP type.
 * Link mode lock must be held.
 * @param pp FRS port privates.
 * @param sfp SFP type detected on port.
 * @return true if SFP module change detected
 */
bool deipce_set_sfp(struct deipce_port_priv *pp, enum deipce_sfp_type sfp)
{
    bool changed = false;

    if (sfp != pp->sfp.type) {
        // Log only when sure.
        if (pp->flags & DEIPCE_SFP_EEPROM) {
            netdev_info(pp->netdev, "SFP type changed from %s to %s\n",
                        deipce_sfp_type_str(pp->sfp.type),
                        deipce_sfp_type_str(sfp));
        }
        changed = true;
    }

    pp->sfp.type = sfp;

    switch (sfp) {
    case DEIPCE_SFP_NONE:
        pp->sfp.supported = DEIPCE_ETHTOOL_SUPPORTED;
        pp->netdev->if_port = IF_PORT_UNKNOWN;
        break;
    case DEIPCE_SFP_1000BASEX:
        pp->sfp.supported =
            SUPPORTED_FIBRE |
            SUPPORTED_1000baseT_Full |
            SUPPORTED_Autoneg |
            0;
        pp->netdev->if_port = IF_PORT_UNKNOWN;
        break;
    case DEIPCE_SFP_100BASEFX:
        pp->sfp.supported =
            SUPPORTED_FIBRE |
            SUPPORTED_100baseT_Full |
            SUPPORTED_Autoneg |
            0;
        pp->netdev->if_port = IF_PORT_100BASEFX;
        break;
    case DEIPCE_SFP_1000BASET:
        pp->sfp.supported =
            SUPPORTED_MII |
            SUPPORTED_TP |
            SUPPORTED_1000baseT_Full |
            SUPPORTED_100baseT_Full |
            SUPPORTED_10baseT_Full |
            SUPPORTED_Autoneg |
            0;
        pp->netdev->if_port = IF_PORT_UNKNOWN;
        break;
    case DEIPCE_SFP_100BASET:
        pp->sfp.supported =
            SUPPORTED_MII |
            SUPPORTED_TP |
            SUPPORTED_100baseT_Full |
            SUPPORTED_10baseT_Full |
            SUPPORTED_Autoneg |
            0;
        pp->netdev->if_port = IF_PORT_100BASET;
        break;
    case DEIPCE_SFP_UNSUPPORTED:
    default:
        pp->sfp.type = DEIPCE_SFP_UNSUPPORTED;
        pp->sfp.supported = DEIPCE_ETHTOOL_SUPPORTED;
        pp->netdev->if_port = IF_PORT_UNKNOWN;
        break;
    }

    return changed;
}

/**
 * Initialize FRS port SFP handling.
 * @param pp FRS port privates
 */
int deipce_init_sfp(struct deipce_port_priv *pp)
{
#ifdef CONFIG_OF
    if (pp->sfp.eeprom_node) {
        pp->sfp.eeprom = of_find_i2c_device_by_node(pp->sfp.eeprom_node);
        if (pp->sfp.eeprom)
            netdev_dbg(pp->netdev, "Using SFP EEPROM\n");
        else
            netdev_warn(pp->netdev, "SFP EEPROM node not found\n");
    }
#endif

    pp->sfp.type = DEIPCE_SFP_NONE;
    deipce_set_sfp(pp, DEIPCE_SFP_NONE);

    return 0;
}

/**
 * Detect SFP type from SFP EEPROM.
 * Does not modify FRS port, so link_mode_lock need not be held.
 * @param pp FRS port privates.
 * @return Detected SFP module type.
 */
enum deipce_sfp_type deipce_detect_sfp(struct deipce_port_priv *pp)
{
    enum deipce_sfp_type sfp = DEIPCE_SFP_NONE;
    int ret;

    // Physical device identifier
    ret = i2c_smbus_read_byte_data(pp->sfp.eeprom, DEIPCE_SFP_REG_ID);
    if (ret < 0) {
        netdev_dbg(pp->netdev, "SFP EEPROM: ID read failed\n");
        goto done;
    }
    if (ret != 0x03) {
        // Not SFP nor SFP+
        netdev_dbg(pp->netdev, "SFP EEPROM: not SFP[+]\n");
        goto done;
    }

    // It is an SFP module
    sfp = DEIPCE_SFP_UNSUPPORTED;

    // Physical device extended identifier
    ret = i2c_smbus_read_byte_data(pp->sfp.eeprom, DEIPCE_SFP_REG_EXT_ID);
    if (ret < 0) {
        netdev_dbg(pp->netdev, "SFP EEPROM: EXT ID read failed\n");
        goto done;
    }
    if (ret != 0x04) {
        // Require function is defined by two-wire interface EEPROM
        netdev_dbg(pp->netdev, "SFP EEPROM: function not defined by EEPROM\n");
        goto done;
    }

    // Transceiver Ethernet compliance codes
    ret = i2c_smbus_read_byte_data(pp->sfp.eeprom, DEIPCE_SFP_REG_ETH);
    if (ret < 0) {
        netdev_dbg(pp->netdev, "SFP EEPROM: Ethernet code read failed\n");
        goto done;
    }

    if (ret & (1u << 0)) {
        // 1000Base-SX
        sfp = DEIPCE_SFP_1000BASEX;
        netdev_dbg(pp->netdev, "SFP EEPROM: 1000Base-SX\n");
    }
    else if (ret & (1u << 1)) {
        // 1000Base-LX
        sfp = DEIPCE_SFP_1000BASEX;
        netdev_dbg(pp->netdev, "SFP EEPROM: 1000Base-LX\n");
    }
    else if (ret & (1u << 2)) {
        // 1000Base-CX
        sfp = DEIPCE_SFP_1000BASEX;
        netdev_dbg(pp->netdev, "SFP EEPROM: 1000Base-CX\n");
    }
    else if (ret & (1u << 3)) {
        // 1000Base-T
        sfp = DEIPCE_SFP_1000BASET;
        netdev_dbg(pp->netdev, "SFP EEPROM: 1000Base-T\n");
    }
    else if (ret & (1u << 4)) {
        // 100Base-LX / 100Base-LX10

        // Detect 100Base-LX vs. 100Base-T from connector type.
        // 100Base-TX pretend to be 100Base-FX.
        ret = i2c_smbus_read_byte_data(pp->sfp.eeprom, DEIPCE_SFP_REG_CONN);
        if (ret < 0) {
            netdev_dbg(pp->netdev, "SFP EEPROM: Connector type read failed\n");
            goto done;
        }
        if (ret == 0x22) {
            // RJ45
            sfp = DEIPCE_SFP_100BASET;
            netdev_dbg(pp->netdev, "SFP EEPROM: 100Base-T\n");
        }
        else {
            sfp = DEIPCE_SFP_100BASEFX;
            netdev_dbg(pp->netdev, "SFP EEPROM: 100Base-LX[10]\n");
        }
    }
    else if (ret & (1u << 5)) {
        // 100Base-FX

        // Detect 100Base-FX vs. 100Base-T from connector type.
        // 100Base-TX pretends to be 100Base-FX.
        ret = i2c_smbus_read_byte_data(pp->sfp.eeprom, DEIPCE_SFP_REG_CONN);
        if (ret < 0) {
            netdev_dbg(pp->netdev, "SFP EEPROM: Connector type read failed\n");
            goto done;
        }
        if (ret == 0x22) {
            // RJ45
            sfp = DEIPCE_SFP_100BASET;
            netdev_dbg(pp->netdev, "SFP EEPROM: 100Base-T\n");
        }
        else {
            sfp = DEIPCE_SFP_100BASEFX;
            netdev_dbg(pp->netdev, "SFP EEPROM: 100Base-FX\n");
        }
    }
    else if (ret & (1u << 6) || (ret & (1u << 7))) {
        // Base-PX or Base-BX10
        ret = i2c_smbus_read_byte_data(pp->sfp.eeprom, DEIPCE_SFP_REG_BR);
        if (ret < 0) {
            netdev_dbg(pp->netdev, "SFP EEPROM: Connector type read failed\n");
            goto done;
        }

        if (ret == 0xc || ret == 0xd) {
            // 1200 - 1300 MBd
            sfp = DEIPCE_SFP_1000BASEX;
            netdev_dbg(pp->netdev,
                       "SFP EEPROM: 1000Base-PX or 1000Base-BX10\n");
        }
        else if (ret == 0x01 || ret == 0x02) {
            // 100 - 200 MBd
            sfp = DEIPCE_SFP_100BASEFX;
            netdev_dbg(pp->netdev, "SFP EEPROM: 100Base-FX\n");
        }
        else {
            netdev_dbg(pp->netdev, "SFP EEPROM: Not recognized\n");
            goto done;
        }
    }

done:
    return sfp;
}

/**
 * Cleanup FRS port SFP handling.
 * @param pp FRS port privates.
 */
void deipce_cleanup_sfp(struct deipce_port_priv *pp)
{
    deipce_set_sfp(pp, DEIPCE_SFP_NONE);

    if (pp->sfp.eeprom) {
        put_device(&pp->sfp.eeprom->dev);
        pp->sfp.eeprom = NULL;
    }

    return;
}

