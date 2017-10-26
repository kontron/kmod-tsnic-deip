/** @file
 */

/*

   Flexibilis PPx Time Stamper Linux driver

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

#ifndef DEIPCE_FPTS_IF_H
#define DEIPCE_FPTS_IF_H

// Time stamper registers

#define FPTS_REG_TS_CTRL        0x1000  ///< control register
#define FPTS_TS_CTRL_GET_TS     0x0001  ///< get timestamp

#define FPTS_REG_INT_MASK       0x1008  ///< interrupt mask
#define FPTS_REG_INT_STAT       0x1010  ///< interrupt status
#define FPTS_INT_TS             0x0001  ///< timestamp interrupt

#define FPTS_REG_TS_NSEC0       0x1104  ///< time stamp nanoseconds LSBs
#define FPTS_REG_TS_NSEC1       0x1106  ///< time stamp nanoseconds MSBs
#define FPTS_TS_NSEC_MASK       0x3fffffff      ///< nanoseconds bitmask

#define FPTS_REG_TS_SEC0        0x1108  ///< time stamp seconds LSBs
#define FPTS_REG_TS_SEC1        0x110a  ///< time stamp seconds
#define FPTS_REG_TS_SEC2        0x110c  ///< time stamp seconds MSBs
#define FPTS_TS_SEC_MASK        0xffffffffffffull       ///< seconds bitmask

#define FPTS_REG_PCNT0          0x1110  ///< pulse counter LSBs
#define FPTS_REG_PCNT1          0x1112  ///< pulse counter MSBs

#endif
