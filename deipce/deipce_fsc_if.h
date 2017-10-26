/** @file
 */

/*

   Flexibilis Scheduling Controller Linux driver

   Copyright (C) 2016 Flexibilis Oy

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

#ifndef DEIPCE_FSC_IF_H
#define DEIPCE_FSC_IF_H

// Common registers

#define FSC_DEV_ID0                     0x0000  ///< device ID0
#define FSC_DEV_ID1                     0x0002  ///< device ID1

#define FSC_INT_ID0                     0x0004  ///< internal revision LSB
#define FSC_INT_ID1                     0x0006  ///< internal revision MSB

#define FSC_ROW_ACCESS_CMD0             0x1000  ///< row access command
#define FSC_ROW_ACCESS_CMD0_SCHED_MASK  0x0007
#define FSC_ROW_ACCESS_CMD0_TABLE_MASK  0x0001
#define FSC_ROW_ACCESS_CMD0_TABLE_SHIFT 8
#define FSC_ROW_ACCESS_CMD0_ACCESS_ERR  (1u << 13)
#define FSC_ROW_ACCESS_CMD0_READ        (0u << 14)
#define FSC_ROW_ACCESS_CMD0_WRITE       (1u << 14)
#define FSC_ROW_ACCESS_CMD0_TRANSFER    (1u << 15)

#define FSC_ROW_ACCESS_CMD1             0x1002  ///< row number
#define FSC_ROW_ACCESS_CMD1_ROW_MASK    0x03ff

#define FSC_ROW_DATA_OUT0               0x1010  ///< row data, outputs, LSB
#define FSC_ROW_DATA_OUT1               0x1012  ///< row data, outputs
#define FSC_ROW_DATA_OUT2               0x1014  ///< row data, outputs
#define FSC_ROW_DATA_OUT3               0x1016  ///< row data, outputs, MSB
#define FSC_ROW_DATA_CYCLES             0x1018  ///< time to stay on this row
#define FSC_ROW_DATA_CYCLES_MIN         64      ///< min. number of cycles
#define FSC_ROW_DATA_CYCLES_MAX         U16_MAX ///< max. number of cycles

// Scheduler registers

/// Offset to scheduler n registers
#define FSC_SCHED_BASE(n)               (0X8000 + (n)*0x1000)

// Scheduler common registers
#define FSC_SCHED_GEN                   0x000   ///< SCH_GEN, downcounter
#define FSC_SCHED_GEN_DC_SPEED_MASK     0x0003  ///< downcounter speed mask
#define FSC_SCHED_GEN_DC_SPEED_1000     0x0000  ///< 1 Gb/s (1/1 clk)
#define FSC_SCHED_GEN_DC_SPEED_100      0x0001  ///< 100 Mb/s (1/10 clk)
#define FSC_SCHED_GEN_DC_SPEED_10       0x0002  ///< 10 Mb/s (1/100 clk)
#define FSC_SCHED_GEN_DC_START_MASK     0x07ff  ///< downcounter start mask
#define FSC_SCHED_GEN_DC_START_SHIFT    4       ///< downcounter start shift

#define FSC_SCHED_DC_SPEED              0x002   ///< downcounter speed
#define FSC_SCHED_DC_SPEED_MASK         (0x7 << 6)      ///< bitmask
#define FSC_SCHED_DC_SPEED_125MHZ       (0x4 << 6)      ///< 125 MHz clock
#define FSC_SCHED_DC_SPEED_100MHZ       (0x5 << 6)      ///< 100 MHz clock

#define FSC_SCHED_EME_DIS_CTRL          0X0020  ///< emergency disable control
#define FSC_SCHED_EME_DIS_CTRL_DISABLE  (0u << 0)
#define FSC_SCHED_EME_DIS_CTRL_ENABLE   (1u << 0)
#define FSC_SCHED_EME_DIS_CTRL_ACTIVE   (1u << 1)
#define FSC_SCHED_EME_DIS_CTRL_INACTIVE (0u << 1)

#define FSC_SCHED_EME_DIS_STAT0         0X0030  ///< emergency disable state
#define FSC_SCHED_EME_DIS_STAT1         0X0032  ///< emergency disable state
#define FSC_SCHED_EME_DIS_STAT2         0X0034  ///< emergency disable state
#define FSC_SCHED_EME_DIS_STAT3         0X0036  ///< emergency disable state

// Scheduler table registers

/// Offset to scheduler n table t registers
#define FSC_SCHED_TBL_BASE(n, t)        (FSC_SCHED_BASE(n) + 0x800 + (t)*0x100)

#define FSC_SCHED_TBL_GEN               0x000   ///< control and status
#define FSC_SCHED_TBL_GEN_CAN_USE       (1u << 0)       ///< can be used
#define FSC_SCHED_TBL_GEN_IN_USE        (1u << 1)       ///< table in use
#define FSC_SCHED_TBL_GEN_STOP_AT_LAST  (1u << 8)       ///< stop at last cycle
#define FSC_SCHED_TBL_GEN_LAST_REACHED  (1u << 9)       ///< last cycle reached
#define FSC_SCHED_TBL_GEN_UPDATE        (1u << 15)      ///< update info

#define FSC_SCHED_TBL_START_TIME_NS     0x014   ///< start time, nsec
#define FSC_SCHED_TBL_START_TIME_NS_MASK 0x3fffffff ///< bitmask for nsec

#define FSC_SCHED_TBL_START_TIME_SEC    0x018   ///< start time, seconds
#define FSC_SCHED_TBL_START_TIME_SEC_MASK 0xffull               ///< bitmask for seconds

#define FSC_SCHED_TBL_CYCLE_TIME_SUBNS  0x020   ///< cycle time, subnsec
#define FSC_SCHED_TBL_CYCLE_TIME_SUBNS_MASK 0xffffff00 ///< bitmask for subnsec

#define FSC_SCHED_TBL_CYCLE_TIME_NS     0x024   ///< cycle time, nsec
#define FSC_SCHED_TBL_CYCLE_TIME_NS_MASK 0x3fffffff ///< bitmask for nsec

#define FSC_SCHED_TBL_CYCLE_TS_NS       0x034   ///< cycle timestamp, nsec
#define FSC_SCHED_TBL_CYCLE_TS_NS_MASK  0x3fffffff ///< bitmask for nsec

#define FSC_SCHED_TBL_CYCLE_TS_SEC      0x038   ///< cycle timestamp, seconds
#define FSC_SCHED_TBL_CYCLE_TS_SEC_MASK 0x00ff  ///< bitmask for seconds

#define FSC_SCHED_TBL_CYCLE_COUNT       0x040   ///< cycle count
#define FSC_SCHED_TBL_LAST_CYCLE        0x044   ///< last cycle

#endif
