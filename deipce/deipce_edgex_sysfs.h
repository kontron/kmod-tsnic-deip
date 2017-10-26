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

#ifndef DEIPCE_EDGEX_SYSFS_H
#define DEIPCE_EDGEX_SYSFS_H

#include "deipce_types.h"

int deipce_edgex_sysfs_init(struct deipce_port_priv *pp);
void deipce_edgex_sysfs_cleanup(struct deipce_port_priv *pp);

#endif
