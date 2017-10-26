/** @file
 */

/*

   Flexibilis Inter-Block Configuration Linux driver

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

#ifndef DEIPCE_IBC_IF_H
#define DEIPCE_IBC_IF_H

#define IBC_DEV_ID              0x00e0  ///< IBC device ID value

// IBC registers

#define IBC_REG_DEV_ID0         0x0000  ///< IP core identification (LSb)
#define IBC_REG_DEV_ID1         0x0002  ///< IP core identification (MSb)
#define IBC_REG_INT_ID0         0x0004  ///< IP core version information (LSb)
#define IBC_REG_INT_ID1         0x0006  ///< IP core version information (MSb)
#define IBC_REG_GP_MUX_CTRL     0x1000  ///< GP MUX control
#define IBC_REG_TIME_MUX_CTRL   0x1100  ///< time interface MUX control
#define IBC_MUX_MASK            0x0003  ///< bitmask for MUX control

#endif
