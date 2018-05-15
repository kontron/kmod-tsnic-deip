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

#ifndef DEIPCE_SHAPER_H
#define DEIPCE_SHAPER_H

#include <linux/types.h>

uint32_t deipce_shaper_bitrate(uint32_t clk, uint16_t addend);
uint16_t deipce_shaper_addend(uint32_t clk, uint32_t rate);

int deipce_shaper_init(struct deipce_port_priv *pp);
void deipce_shaper_cleanup(struct deipce_port_priv *pp);

#endif
