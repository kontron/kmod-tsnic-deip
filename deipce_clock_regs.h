/** @file
 */

/*

   Flexibilis Real-Time Clock Linux driver

   Copyright (C) 2009 Flexibilis Oy

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

#ifndef DEIPCE_CLOCK_REGS_H
#define DEIPCE_CLOCK_REGS_H

// FRTC registers

/**
 * GENERAL register
 */
#define GENERAL_REG             0x00000000
#define REVID_MASK              0xff
#define REVID_SHIFT             0
#define DEVID_MASK              0xffff
#define DEVID_SHIFT             8
#define RESET_SHIFT             31
#define RESET_BIT               (1u << RESET_SHIFT)

/**
 * CUR_NSEC register
 */
#define NCO_SUBNSEC_REG         0x00001000
#define NCO_SUBNSEC_MASK        0x0000ffff

#define NCO_NSEC_REG            0x00001004
#define NCO_NSEC_MASK           0x3fffffff

/**
 * CUR_SEC register
 */
#define NCO_SEC_REG             0x00001008
#define NCO_SEC_HI_REG          0x0000100C
#define NCO_SEC_MASK            0xffffffffffffULL
#define NCO_SEC_HI_MASK         ((uint32_t)(NCO_SEC_MASK >> 32))

/**
 * TIME_CC register
 */
#define NCO_CCCNT_REG           0x00001010
#define NCO_CCCNT_HI_REG        0x00001014
#define NCO_CC_MASK             0xffffffffffffULL

/**
 * STEP_SIZE register
 */
#define NCO_STEP_SUBNSEC_REG    0x00001020
#define NCO_STEP_NSEC_REG       0x00001024
#define NCO_STEP_NSEC_MASK      0x3f

/**
 * ADJUST_NSEC register
 */
#define NCO_ADJ_NSEC_REG        0x00001034
#define NCO_ADJ_NSEC_MASK       0x3fffffff

/**
 * ADJUST_SEC register
 */
#define NCO_ADJ_SEC_REG         0x00001038
#define NCO_ADJ_SEC_HI_REG      0x0000103c
#define NCO_ADJ_SEC_MASK        0xffffffffffffULL
#define NCO_ADJ_SEC_HI_MASK     ((uint32_t)(NCO_ADJ_SEC_MASK >> 32))

/**
 * TIME_CMD register
 */
#define NCO_CMD_REG             0x00001040
#define NCO_CMD_ADJUST_CLOCK    0x1
#define NCO_CMD_ADJUST_STEP     0x2
#define NCO_CMD_READ            0x4

/**
 * GENERICS registers
 */
#define NCO_GENERICS_STEP_SUBNSEC_REG   0x00002000
#define NCO_GENERICS_STEP_NSEC_REG      0x00002004

#endif
