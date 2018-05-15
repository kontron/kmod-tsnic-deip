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

#ifndef DEIPCE_SFP_H
#define DEIPCE_SFP_H

#include "deipce_types.h"

// Common SFP registers

#define DEIPCE_SFP_REG_ID                      0x00
#define DEIPCE_SFP_REG_EXT_ID                  0x01
#define DEIPCE_SFP_REG_CONN                    0x02
#define DEIPCE_SFP_REG_ETH_10G_INFINIBAND      0x03
#define DEIPCE_SFP_REG_ESCON_SONET1            0x04
#define DEIPCE_SFP_REG_ESCON_SONET2            0x05
#define DEIPCE_SFP_REG_ETH                     0x06
#define DEIPCE_SFP_REG_FIBER1                  0x07
#define DEIPCE_SFP_REG_FIBER2                  0x08
#define DEIPCE_SFP_REG_FIBER_MEDIA             0x09
#define DEIPCE_SFP_REG_FIBER_SPEED             0x0a
#define DEIPCE_SFP_REG_BR                      0x0a
#define DEIPCE_SFP_REG_MAX_BR                  0x42

int deipce_init_sfp(struct deipce_port_priv *pp);
const char *deipce_sfp_type_str(enum deipce_sfp_type sfp);
bool deipce_set_sfp(struct deipce_port_priv *pp, enum deipce_sfp_type sfp);
enum deipce_sfp_type deipce_detect_sfp(struct deipce_port_priv *pp);
void deipce_cleanup_sfp(struct deipce_port_priv *pp);

#endif
