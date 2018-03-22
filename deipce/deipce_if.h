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

#ifndef DEIPCE_IF_H
#define DEIPCE_IF_H

#ifndef ETH_P_HSR_SUPERVISION
/// Ethernet type used for HSR supervision
#define ETH_P_HSR_SUPERVISION 0x88FB
#endif

#define FRS_CONFIG_ID_DS      0x0007

/**
 * FRS Switch Generics Registers
 */
#define FRS_REG_GENERIC                 0x6000

#define FRS_REG_GENERIC_PORT_HIGH       (FRS_REG_GENERIC + 0x000)
#define FRS_REG_GENERIC_COUNTERS        (FRS_REG_GENERIC + 0x002)
#define FRS_REG_GENERIC_CT_PORTS        (FRS_REG_GENERIC + 0x004)
#define FRS_REG_GENERIC_TS_PORTS        (FRS_REG_GENERIC + 0x006)
#define FRS_REG_GENERIC_SMAC_ROWS       (FRS_REG_GENERIC + 0x008)
#define FRS_REG_GENERIC_SMAC_ROWS_MASK  0xf
#define FRS_REG_GENERIC_POLICING        (FRS_REG_GENERIC + 0x00a)
#define FRS_REG_GENERIC_POLICING_MASK   0x3
#define FRS_REG_GENERIC_POLICERS        (FRS_REG_GENERIC + 0x00c)
#define FRS_REG_GENERIC_POLICERS_MASK   0xf
#define FRS_REG_GENERIC_PRIO_QUEUES     (FRS_REG_GENERIC + 0x00e)
#define FRS_REG_GENERIC_PRIO_QUEUES_MASK 0xf
#define FRS_REG_GENERIC_BUFFER_SIZE     (FRS_REG_GENERIC + 0x010)
#define FRS_REG_GENERIC_SHAPERS         (FRS_REG_GENERIC + 0x012)
#define FRS_REG_GENERIC_GIGABIT         (FRS_REG_GENERIC + 0x014)
#define FRS_REG_GENERIC_HSR_PORTS       (FRS_REG_GENERIC + 0x016)
#define FRS_REG_GENERIC_PRP_PORTS       (FRS_REG_GENERIC + 0x018)
#define FRS_REG_GENERIC_SCHED_PORTS     (FRS_REG_GENERIC + 0x01a)
#define FRS_REG_GENERIC_PREEMPT_PORTS   (FRS_REG_GENERIC + 0x01c)
#define FRS_REG_GENERIC_MACSEC_PORTS    (FRS_REG_GENERIC + 0x01e)
#define FRS_REG_GENERIC_MACSEC_CIPHER   (FRS_REG_GENERIC + 0x020)
#define FRS_REG_GENERIC_MGMT_PORTS      (FRS_REG_GENERIC + 0x022)
#define FRS_REG_GENERIC_CLK_FREQ        (FRS_REG_GENERIC + 0x100)
#define FRS_REG_GENERIC_CLK_FREQ_MASK   0xff
#define FRS_REG_GENERIC_DEBUG_IN        (FRS_REG_GENERIC + 0x110)

#define FRS_REG_GENERIC_BOOL_MASK       0x1
#define FRS_REG_GENERIC_PORT_MASK       0xfff

/**
 * FRS Switch Configuration Registers.
 */
#define FRS_REG_ID0                 0x0000      ///< FRS ID0 register
#define FRS_REG_ID1                 0x0001      ///< FRS ID1 register
#define FRS_REG_CONFIG_ID           0x0002      ///< FRS register
#define FRS_REG_CONFIG_SVN_ID       0x0003      ///< FRS register
#define FRS_REG_BODY_SVN_ID         0x0004      ///< FRS register
#define FRS_REG_GEN                 0x0008      ///< FRS GENERAL register
#define FRS_REG_MAC_TABLE_CLEAR_MASK    0x0009  ///< FRS MAC table clear mask
//#define FRS_REG_CMEM_FILL_LEVEL     0x000A
#define FRS_REG_DMEM_FILL_LEVEL     0x000B
#define FRS_REG_SEQ_MEM_FILL_LEVEL  0x000C
#define FRS_REG_AGING               0x0010      ///< FRS AGING register
#define FRS_REG_AGING_BASE_TIME_LO  0x0011
#define FRS_REG_AGING_BASE_TIME_HI  0x0012
#define FRS_REG_AUTH_STATUS         0x0013
#define FRS_REG_TS_CTRL_TX          0x0014      ///< Frame timestamp control, TX
#define FRS_REG_TS_CTRL_RX          0x0015      ///< Frame timestamp control, RX
#define FRS_REG_INTMASK             0x0016
#define FRS_REG_INTSTAT             0x0017
#define FRS_REG_MAC_TABLE(n)        (0x100 + (n))       ///< n = 0 .. 3

#define FRS_AGING_MASK              0x007f      ///< bitmask for ageing time

// FRS Switch Configuration Registers bits, General.
#define FRS_GEN_MDIO_POLL_DISABLE   (1u << 0)   ///< disable MDIO polling
#define FRS_GEN_MDIO_POLL_ENABLED   (1u << 1)   ///< polling status bit
#define FRS_GEN_POLICER             (1u << 8)   ///< policer configuration
#define FRS_GEN_MGMT_TRAILER_LEN    (1u << 2)   ///< management trailer length
#define FRS_GEN_MGMT_TRAILER_OFFSET (1u << 4)   ///< management trailer offset
#define FRS_GEN_POLICE_SMAC         (1u << 8)   ///< static MAC address table based policing
#define FRS_GEN_TIME_TRAILER        (1u << 9)   ///< port 0 time trailer
#define FRS_GEN_PTP_MODIFY_SYNC     (1u << 10)  ///< PTP modify SYNC
#define FRS_GEN_PTP_UDP             0x0000      ///< PTP mode UDP/IPv4
#define FRS_GEN_PTP_ETH             (1u << 12)  ///< PTP mode Ethernet
#define FRS_GEN_PTP_MASK            (3u << 11)  ///< PTP mode bitmask
#define FRS_GEN_PTP_DISABLE_CORRECTION (1u << 13) ///< PTP disable correction field modification
#define FRS_GEN_CLEAR_MAC_TABLE     (1u << 14)  ///< clear MAC address table
#define FRS_GEN_RESET               (1u << 15)  ///< SW reset

#define FRS_REG_SMAC_CMD            0x0110      ///< SMAC command register
#define FRS_SMAC_CMD_TRANSFER       (1u << 15)  ///< initiate read/write access
#define FRS_SMAC_CMD_READ           (0u << 14)  ///< read operation selection
#define FRS_SMAC_CMD_WRITE          (1u << 14)  ///< write operation selection
#define FRS_SMAC_CMD_COLUMN_MASK    0x0003      ///< bits for table column
#define FRS_SMAC_CMD_COLUMN_SHIFT   12          ///< bit offset for column
#define FRS_SMAC_CMD_ROW_MASK       0x0FFF      ///< bits for table row
#define FRS_SMAC_CMD_ROW_SHIFT      0           ///< bit offset for row

#define FRS_REG_SMAC_TABLE          0x0111      ///< SMAC table config register
#define FRS_SMAC_TABLE_ROW_SEL_MASK 0x0003      ///< row selection bitmask
#define FRS_SMAC_TABLE_XOR_VLAN_DISABLE 0x0     ///< XOR with VLAN disabled
#define FRS_SMAC_TABLE_XOR_VLAN_ENABLE 0x1      ///< XOR with VLAN enabled
/// make row selection bits for column col
#define FRS_SMAC_TABLE_FROM_ROW_SEL(col,val) \
    (((val) & FRS_SMAC_TABLE_ROW_SEL_MASK) << ((col)*2))
/// get row selection bits for column col
#define FRS_SMAC_TABLE_TO_ROW_SEL(col,val) \
    (((val) >> ((col)*2)) & FRS_SMAC_TABLE_ROW_SEL_MASK)

#define FRS_REG_SMAC_CONFIG         0x0118      ///< SMAC flags register (SMAC_TABLE0)
#define FRS_SMAC_CONFIG_ENABLED     (1u << 15)  ///< enable/disable SMAC entry
#define FRS_SMAC_CONFIG_NO_PRP      (1u << 14)  ///< no PRP trailer
#define FRS_SMAC_CONFIG_LOW_PRIO    (1u << 13)  ///< policer priority
#define FRS_SMAC_CONFIG_VLAN        (1u << 12)  ///< check frame for matching VLAN
#define FRS_SMAC_CONFIG_PRIOMOD     (1u << 11)  ///< enable/disable priority modification
#define FRS_SMAC_CONFIG_FRAME_PRIO_MASK 0x0007  ///< bits for frame priority
#define FRS_SMAC_CONFIG_FRAME_PRIO_SHIFT 8      ///< bits for frame priority

#define FRS_REG_SMAC_ADDR(x)        (0x0119 + (x)) ///< MAC address bytes (SMAC_TABLE1..3)
#define FRS_REG_SMAC_FWD_MASK       0x011C      ///< port forward mask (SMAC_TABLE4)
#define FRS_REG_SMAC_POLICED_MASK   0x011D      ///< policed port (SMAC_TABLE5)
#define FRS_REG_SMAC_POLICER        0x011E      ///< policer number
#define FRS_REG_SMAC_VLAN           0x011F      ///< VLAN for which the entry is enabled (SMAC_TABLE7)

#define FRS_SMAC_TABLE_MAX_ROWS     4096        ///< max. number of SMAC table rows
#define FRS_SMAC_TABLE_COLS         4           ///< columns in SMAC table

/**
 * Timestamp registers.
 */
#define FRS_TS_TX_BASE              0x1000
#define FRS_TS_RX_BASE              0x1100
#define FRS_TS_NS_LO                0x0000      ///< offset from timestamp start
#define FRS_TS_NS_HI                0x0001      ///< offset from timestamp start
#define FRS_TS_S_LO                 0x0002      ///< offset from timestamp start
#define FRS_TS_TX_OFS(i)            FRS_TS_TX_NS_LO(i)  ///< TX timestamper start
#define FRS_TS_RX_OFS(i)            FRS_TS_RX_NS_LO(i)  ///< RX timestamper start
#define FRS_TS_TX_NS_LO(i)          (FRS_TS_TX_BASE + (i)*0x40)
#define FRS_TS_TX_NS_HI(i)          (FRS_TS_TX_BASE + (i)*0x40 + 1)
#define FRS_TS_TX_S_LO(i)           (FRS_TS_TX_BASE + (i)*0x40 + 2)
#define FRS_TS_TX_S_HI(i)           (FRS_TS_TX_BASE + (i)*0x40 + 3)
#define FRS_TS_TX_HDR(i,p)          (FRS_TS_TX_BASE + (i)*0x40 + 7 + (p))
#define FRS_TS_RX_NS_LO(i)          (FRS_TS_RX_BASE + (i)*0x40)
#define FRS_TS_RX_NS_HI(i)          (FRS_TS_RX_BASE + (i)*0x40 + 1)
#define FRS_TS_RX_S_LO(i)           (FRS_TS_RX_BASE + (i)*0x40 + 2)
#define FRS_TS_RX_S_HI(i)           (FRS_TS_RX_BASE + (i)*0x40 + 3)
#define FRS_TS_RX_HDR(i,p)          (FRS_TS_RX_BASE + (i)*0x40 + 7 + (p))
#define FRS_TS_SEC_MASK             0xfu        ///< bitmask for seconds
#define FRS_TS_NSEC_MASK            0x3fffffffu ///< bitmask for nanoseconds
#define FRS_TS_NUMBER_TIMESTAMPS    4
#define FRS_TS_HDR_LEN              32          ///< Number of HDR bytes to use

/**
 * FRS Interrupt registers.
 */
#define FRS_INT_TX_TSTAMP           (1u << 0)
#define FRS_INT_RX_TSTAMP           (1u << 1)
#define FRS_INT_RX_ERROR            (1u << 2)
#define FRS_INT_CONGESTED           (1u << 3)

/**
 * FRS MAC table registers
 */
#define FRS_MAC_TABLE0_PORT_MASK    0xf         ///< Port number mask
#define FRS_MAC_TABLE0_TRANSFER     (1u << 15)  ///< Transfer next addr

/**
 * VLAN Configuration registers.
 */
#define FRS_VLAN_BASE               0x2000
#define FRS_VLAN_CFG(id)            (FRS_VLAN_BASE + (id))
#define FRS_VLAN_DISABLE(port,data) ((data) &= ~(1u << (port)))
#define FRS_VLAN_ENABLE(port,data)  ((data) |= (1u << (port)))

/**
 * FRS Port Configuration Registers: General.
 */
#define PORT_REG_STATE            0x0000        ///< Port state
#define PORT_REG_VLAN             0x0008        ///< port-based VLAN
#define PORT_REG_VLAN0_MAP        0x0009        ///< VLAN0 mapping
#define PORT_REG_FWD_PORT_MASK    0x000A        ///< Forward port mask
#define PORT_REG_VLAN_PRIO        0x000B        ///< VLAN priority mapping
#define PORT_REG_VLAN_PRIO_HI     0x000C        ///< VLAN priority mapping, high bits in DS

/// FRS Port Configuration Register bits, PORT_STATE.
// Bits 1-0, Port State
#define PORT_STATE_FORWARDING         0x0000    ///< port forwarding
#define PORT_STATE_LEARNING           0x0001    ///< port learning
#define PORT_STATE_DISABLED           0x0002    ///< port disabled
#define PORT_STATE_STATE_MASK         0x0003    ///< port state mask
// Bits 3-2, port management status
#define PORT_STATE_NORMAL_MODE        0x0000    ///< normal mode port
#define PORT_STATE_MANAGEMENT         0x0004    ///< management port
// Bits 5-4, Port HW mode
#define PORT_STATE_MII                0x0000    ///< 10/100
#define PORT_STATE_GMII               0x0020    ///< gigabit
// Bits 9-8, Speed select
#define PORT_STATE_1000MBPS           0x0100    ///< gigabit
#define PORT_STATE_100MBPS            0x0200    ///< 100Mb
#define PORT_STATE_10MBPS             0x0300    ///< 10Mb
#define PORT_STATE_SPEED_EXT          0x0000    ///< external/automatic speed
#define PORT_STATE_SPEED_MASK         0x0300    ///< mask for speed setting
// Bits 11-10, Current speed
#define PORT_STATE_CURRENT_MASK       0x0c00    ///< mask for current speed
#define PORT_STATE_CURRENT_1000MBPS   0x0400    ///< gigabit
#define PORT_STATE_CURRENT_100MBPS    0x0800    ///< 100Mb
#define PORT_STATE_CURRENT_10MBPS     0x0C00    ///< 10Mb
// Bit 15, RX cut-through
#define PORT_STATE_RX_CT              0x8000    ///< RX cut-through enabled

/**
 * FRS Port VLAN configuration register bits (PORT_VLAN & VLAN0_MAPPING).
 */
#define PORT_VLAN_ID(id)          ((id) & 0xFFF)
#define PORT_VLAN_PCP(pcp)        (((pcp) << 12) & 0x7000)
#define PORT_VLAN_PCP_GET(x)      (((x) & 0x7000) >> 12)
#define PORT_VLAN_TAGGED          0x8000
#define PORT_VLAN_UNTAGGED        0x0000

/**
 * FRS Port VLAN priority register.
 */
#define PORT_VLAN_PRIO(pcp,prio) \
    (((prio) & 0x3) << (((pcp) & 0x7) * 2))
#define PORT_VLAN_PRIO_HI(pcp,prio) \
    ((((prio) >> 2) & 0x1) << (((pcp) & 0x7) * 2))

/**
 * FRS Port cut-through registers.
 */
#define PORT_REG_TX_CT            0x000d        ///< TX cut-through enable

/**
 * FRS Port preemption registers.
 */
#define PORT_REG_TX_PREE0         0x000e        ///< preemptable/express select
#define PORT_REG_TX_PREE1         0x000f        ///< preemption settings
#define PORT_TX_PREE1_FRAG_MASK   0x0003        ///< min. fragment size mask
#define PORT_TX_PREE1_FRAG_60     0x0000        ///< min. fragment size 60 B
#define PORT_TX_PREE1_FRAG_124    0x0001        ///< min. fragment size 124 B
#define PORT_TX_PREE1_FRAG_188    0x0002        ///< min. fragment size 188 B
#define PORT_TX_PREE1_FRAG_252    0x0003        ///< min. fragment size 252 B

/**
 * FRS Port HSR/PRP Configuration Registers.
 */
#define PORT_REG_HSR_CFG          0x1000

// PORT_REG_HSR_CFG bits
#define HSR_PRP_ENABLE            0x0001        ///< enable bit for redundancy
#define HSR_CFG_MODE_HSR          0x0000        ///< HSR mode
#define HSR_CFG_MODE_PRP          0x0100        ///< PRP mode
#define HSR_CFG_MODE_INTERLINK    0x0200        ///< Interlink config
#define HSR_CFG_MODE_LANA         0x0000        ///< LAN A config
#define HSR_CFG_MODE_LANB         0x0400        ///< LAN B config
#define HSR_CFG_NETID(netid)      (((netid) & 0x7)<<11) ///< Macro for setting NETID

/**
 * FRS Port PTP Configuration Registers.
 */
#define PORT_REG_PTP_BASE         0x2000
#define PORT_REG_PTP_DELAY_SN     (PORT_REG_PTP_BASE + 0x0000)  ///< PTP delay subns
#define PORT_REG_PTP_DELAY_NSL    (PORT_REG_PTP_BASE + 0x0001)  ///< PTP delay ns low
#define PORT_REG_PTP_DELAY_NSH    (PORT_REG_PTP_BASE + 0x0002)  ///< PTP delay ns high
#define PORT_REG_PTP_RX_DELAY_SN  (PORT_REG_PTP_BASE + 0x0004)  ///< PTP RX delay subns
#define PORT_REG_PTP_RX_DELAY_NS  (PORT_REG_PTP_BASE + 0x0005)  ///< PTP RX delay ns
#define PORT_REG_PTP_TX_DELAY_SN  (PORT_REG_PTP_BASE + 0x0008)  ///< PTP TX delay subns
#define PORT_REG_PTP_TX_DELAY_NS  (PORT_REG_PTP_BASE + 0x0009)  ///< PTP TX delay ns

/**
 * FRS Port policer configuration registers.
 */
#define PORT_REG_POLICER_CMD            0x0010          ///< policer command
#define PORT_POLICER_CMD_READ           (0u << 14)      ///< read policer
#define PORT_POLICER_CMD_WRITE          (1u << 14)      ///< write policer
#define PORT_POLICER_CMD_TRANSFER       (1u << 15)      ///< initiate policer command
#define PORT_REG_POLICER0               0x0014          ///< policer limit
#define PORT_REG_POLICER1               0x0015          ///< policer rate
#define PORT_POLICER1_DROPPED_LOW_PRIO  (1u << 14)      ///< low priority frames policed
#define PORT_POLICER1_DROPPED_NORM_PRIO (1u << 15)      ///< normal priority frames policed
#define PORT_POLICER1_BASIC_RATE_MASK   0xffu           ///< bitmask for basic rate
#define PORT_POLICER1_SCALE_SHIFT       8               ///< shift for rate scale
#define PORT_POLICER1_SCALE_MASK        0x7u            ///< bitmask for rate scale

/**
 * FRS Port shaper configuration registers.
 */
#define PORT_REG_SHAPER(n)      (0x0018 + (n))  ///< SHAPER n

/**
 * FRS Port MACSec Configuration Registers.
 */
#define PORT_REG_MACSEC_BASE            0x5000
#define PORT_REG_MACSEC_CONFIG          (PORT_REG_MACSEC_BASE + 0x0000) ///< MACSec configuration
#define PORT_MACSEC_CONFIG_ENABLE       (1u << 0)       ///< MACSec enabled
#define PORT_MACSEC_CONFIG_SC           (1u << 3)       ///< include SCI in SecTAG
#define PORT_MACSEC_CONFIG_TXKEY_SHIFT  4               ///< bitshift for TX key number
#define PORT_MACSEC_CONFIG_TXKEY_MASK   0x1             ///< bitmask for TX key number
#define PORT_MACSEC_CONFIG_RESET_PN_TX_SHIFT    8       ///< bitshift for TX SA PN counter reset mask
#define PORT_MACSEC_CONFIG_RESET_PN_RX_SHIFT    12      ///< bitshift for RX SA PN counter reset mask
#define PORT_MACSEC_CONFIG_RESET_PN_MASK \
    ((1u << (PORT_MACSEC_CONFIG_TXKEY_MASK + 1)) - 1u)  ///< bitmask for RX SA PN counter reset mask
#define PORT_REG_MACSEC_SCI_TX(x) ((PORT_REG_MACSEC_BASE + 0x0080) | (x))  ///< Bits 0-63 of TX SCI
#define PORT_REG_MACSEC_SCI_RX(x) ((PORT_REG_MACSEC_BASE + 0x00C0) | (x))  ///< Bits 0-63 of RX SCI
#define PORT_REG_MACSEC_KEY_TX(n,x) ((PORT_REG_MACSEC_BASE + 0x0100 + (n)*0x20) | (x)) ///< Bits 0-255 of TX key
#define PORT_REG_MACSEC_KEY_RX(n,x) ((PORT_REG_MACSEC_BASE + 0x0180 + (n)*0x20) | (x)) ///< Bits 0-255 of RX key

/**
 * FRS Port Frame Timestamp Registers.
 */
#define PORT_TS_TX_BASE                 0x6000
#define PORT_TS_RX_BASE                 0x6100
#define PORT_TS_TX_OFS(i)               PORT_TS_TX_NS_LO(i)     ///< TX timestamper start
#define PORT_TS_RX_OFS(i)               PORT_TS_RX_NS_LO(i)     ///< RX timestamper start
#define PORT_TS_TX_NS_LO(i)             (PORT_TS_TX_BASE + (i)*0x40)
#define PORT_TS_TX_NS_HI(i)             (PORT_TS_TX_BASE + (i)*0x40 + 1)
#define PORT_TS_TX_S_LO(i)              (PORT_TS_TX_BASE + (i)*0x40 + 2)
#define PORT_TS_TX_S_HI(i)              (PORT_TS_TX_BASE + (i)*0x40 + 3)
#define PORT_TS_TX_HDR(i,p)             (PORT_TS_TX_BASE + (i)*0x40 + 7 + (p))
#define PORT_TS_RX_NS_LO(i)             (PORT_TS_RX_BASE + (i)*0x40)
#define PORT_TS_RX_NS_HI(i)             (PORT_TS_RX_BASE + (i)*0x40 + 1)
#define PORT_TS_RX_S_LO(i)              (PORT_TS_RX_BASE + (i)*0x40 + 2)
#define PORT_TS_RX_S_HI(i)              (PORT_TS_RX_BASE + (i)*0x40 + 3)
#define PORT_TS_RX_HDR(i,p)             (PORT_TS_RX_BASE + (i)*0x40 + 7 + (p))
#define PORT_TS_NUMBER_TIMESTAMPS       FRS_TS_NUMBER_TIMESTAMPS
#define PORT_TS_HDR_LEN                 FRS_TS_HDR_LEN  ///< Number of HDR bytes to use
#define PORT_REG_TS_CTRL_TX             (PORT_TS_TX_BASE + 0x200)       ///< Frame timestamp control, TX
#define PORT_REG_TS_CTRL_RX             (PORT_TS_TX_BASE + 0x201)       ///< Frame timestamp control, RX

/**
 * FRS Port frame size registers, n is queue number (priority).
 */
#define PORT_REG_FRAMESIZE(n)           (0x0020 + (n))
#define PORT_FRAMESIZE_MIN              60              ///< min. frame size
#define PORT_FRAMESIZE_MAX              0x7ffu          ///< max. frame size

/**
 * FRS Port Counter Registers
 */
#define PORT_REG_CNT                    0x3000  ///< Port counters
#define PORT_REG_CNT_CTRL               (PORT_REG_CNT + 0x0000) ///< Control
#define PORT_REG_CNT_CTRL_CAPTURE       (1u << 0)       ///< Capture counters

#define PORT_REG_RX_GOOD_L              (PORT_REG_CNT + 0x0100) ///< RX good octets low
#define PORT_REG_RX_GOOD_H              (PORT_REG_CNT + 0x0101) ///< RX good octets high
#define PORT_REG_RX_BAD_L               (PORT_REG_CNT + 0x0102) ///< RX bad octets low
#define PORT_REG_RX_BAD_H               (PORT_REG_CNT + 0x0103) ///< RX bad octets high
#define PORT_REG_RX_UNICAST_L           (PORT_REG_CNT + 0x0104) ///< RX unicast frames low
#define PORT_REG_RX_UNICAST_H           (PORT_REG_CNT + 0x0105) ///< RX unicast frames high
#define PORT_REG_RX_BROADCAST_L         (PORT_REG_CNT + 0x0106) ///< RX broadcast frames low
#define PORT_REG_RX_BROADCAST_H         (PORT_REG_CNT + 0x0107) ///< RX broadcast frames high
#define PORT_REG_RX_MULTICAST_L         (PORT_REG_CNT + 0x0108) ///< RX multicast frames low
#define PORT_REG_RX_MULTICAST_H         (PORT_REG_CNT + 0x0109) ///< RX multicast frames high
#define PORT_REG_RX_UNDERSIZE_L         (PORT_REG_CNT + 0x010a) ///< RX undersize frames low
#define PORT_REG_RX_UNDERSIZE_H         (PORT_REG_CNT + 0x010b) ///< RX undersize frames high
#define PORT_REG_RX_FRAGMENT_L          (PORT_REG_CNT + 0x010c) ///< RX fragment frames low
#define PORT_REG_RX_FRAGMENT_H          (PORT_REG_CNT + 0x010d) ///< RX fragment frames high
#define PORT_REG_RX_OVERSIZE_L          (PORT_REG_CNT + 0x010e) ///< RX oversize frames low
#define PORT_REG_RX_OVERSIZE_H          (PORT_REG_CNT + 0x010f) ///< RX oversize frames high
#define PORT_REG_RX_JABBER_L            (PORT_REG_CNT + 0x0110) ///< RX jabber frames low
#define PORT_REG_RX_JABBER_H            (PORT_REG_CNT + 0x0111) ///< RX jabber frames high
#define PORT_REG_RX_ERR_L               (PORT_REG_CNT + 0x0112) ///< RX error frames low
#define PORT_REG_RX_ERR_H               (PORT_REG_CNT + 0x0113) ///< RX error frames high
#define PORT_REG_RX_CRC_L               (PORT_REG_CNT + 0x0114) ///< RX CRC error frames low
#define PORT_REG_RX_CRC_H               (PORT_REG_CNT + 0x0115) ///< RX CRC error frames high

#define PORT_REG_RX_64_L                (PORT_REG_CNT + 0x0116) ///< RX 64 octet frames low
#define PORT_REG_RX_64_H                (PORT_REG_CNT + 0x0117) ///< RX 64 octet frames high
#define PORT_REG_RX_65_127_L            (PORT_REG_CNT + 0x0118) ///< RX 65-127 octet frames low
#define PORT_REG_RX_65_127_H            (PORT_REG_CNT + 0x0119) ///< RX 65-127 octet frames high
#define PORT_REG_RX_128_255_L           (PORT_REG_CNT + 0x011a) ///< RX 128-255 octet frames low
#define PORT_REG_RX_128_255_H           (PORT_REG_CNT + 0x011b) ///< RX 128-255 octet frames high
#define PORT_REG_RX_256_511_L           (PORT_REG_CNT + 0x011c) ///< RX 255-511 octet frames low
#define PORT_REG_RX_256_511_H           (PORT_REG_CNT + 0x011d) ///< RX 255-511 octet frames high
#define PORT_REG_RX_512_1023_L          (PORT_REG_CNT + 0x011e) ///< RX 512-1023 octet frames low
#define PORT_REG_RX_512_1023_H          (PORT_REG_CNT + 0x011f) ///< RX 512-1023 octet frames high
#define PORT_REG_RX_1024_1536_L         (PORT_REG_CNT + 0x0120) ///< RX 1024-1536 octet frames low
#define PORT_REG_RX_1024_1536_H         (PORT_REG_CNT + 0x0121) ///< RX 1024-1536 octet frames high

#define PORT_REG_RX_HSRPRP_L            (PORT_REG_CNT + 0x0122) ///< RX HSR/PRP frames low
#define PORT_REG_RX_HSRPRP_H            (PORT_REG_CNT + 0x0123) ///< RX HSR/PRP frames high
#define PORT_REG_RX_WRONGLAN_L          (PORT_REG_CNT + 0x0124) ///< RX wrong LAN PRP frames low
#define PORT_REG_RX_WRONGLAN_H          (PORT_REG_CNT + 0x0125) ///< RX wrong LAN PRP frames high
#define PORT_REG_RX_DUPLICATE_L         (PORT_REG_CNT + 0x0126) ///< RX duplicate HSR/PRP frames low
#define PORT_REG_RX_DUPLICATE_H         (PORT_REG_CNT + 0x0127) ///< RX duplicate HSR/PRP frames high

#define PORT_REG_RX_MEM_FULL_DROP_L     (PORT_REG_CNT + 0x012A)  ///< RX memory full low
#define PORT_REG_RX_MEM_FULL_DROP_H     (PORT_REG_CNT + 0x012B)  ///< RX memory full high
#define PORT_REG_RX_POLICED_L           (PORT_REG_CNT + 0x012C)  ///< RX policed frames low
#define PORT_REG_RX_POLICED_H           (PORT_REG_CNT + 0x012D)  ///< RX policed frames high
#define PORT_REG_RX_MACSEC_UNTAGGED_L   (PORT_REG_CNT + 0x0130)  ///< RX frames w/o MACsec Ethertype low
#define PORT_REG_RX_MACSEC_UNTAGGED_H   (PORT_REG_CNT + 0x0131)  ///< RX frames w/O MACsec Ethertype high
#define PORT_REG_RX_MACSEC_NOTSUPP_L    (PORT_REG_CNT + 0x0132)  ///< RX frames w/ unsupported SecTAG low
#define PORT_REG_RX_MACSEC_NOTSUPP_H    (PORT_REG_CNT + 0x0133)  ///< RX frames w/ unsupported SecTAG high
#define PORT_REG_RX_MACSEC_UNKNOWN_SCI_L (PORT_REG_CNT + 0x0134)  ///< RX frames w/ unknown MACsec SCI low
#define PORT_REG_RX_MACSEC_UNKNOWN_SCI_H (PORT_REG_CNT + 0x0135)  ///< RX frames w/ unknown MACsec SCI high
#define PORT_REG_RX_MACSEC_NOTVALID_L   (PORT_REG_CNT + 0x0136)  ///< RX frames failed validity check low
#define PORT_REG_RX_MACSEC_NOTVALID_H   (PORT_REG_CNT + 0x0137)  ///< RX frames failed validity check high
#define PORT_REG_RX_MACSEC_LATE_L       (PORT_REG_CNT + 0x0138)  ///< RX late MACsec frames low
#define PORT_REG_RX_MACSEC_LATE_H       (PORT_REG_CNT + 0x0139)  ///< RX late MACsec frames high

#define PORT_REG_RX_PREEMPT_SMD_ERR_L   (PORT_REG_CNT + 0x012e) ///< RX preemption SMD errors low
#define PORT_REG_RX_PREEMPT_SMD_ERR_H   (PORT_REG_CNT + 0x012f) ///< RX preemption SMD errors high

#define PORT_REG_RX_PREEMPT_ASS_ERR_L   (PORT_REG_CNT + 0x013a) ///< RX preemption assembly errors low
#define PORT_REG_RX_PREEMPT_ASS_ERR_H   (PORT_REG_CNT + 0x013b) ///< RX preemption assembly errors high
#define PORT_REG_RX_PREEMPT_ASS_OK_L    (PORT_REG_CNT + 0x013c) ///< RX preemption assembly OK low
#define PORT_REG_RX_PREEMPT_ASS_OK_H    (PORT_REG_CNT + 0x013d) ///< RX preemption assembly OK high
#define PORT_REG_RX_PREEMPT_FRAG_L      (PORT_REG_CNT + 0x013e) ///< RX preemption extra fragments low
#define PORT_REG_RX_PREEMPT_FRAG_H      (PORT_REG_CNT + 0x013f) ///< RX preemption extra fragments high

#define PORT_REG_TX_L                   (PORT_REG_CNT + 0x0140) ///< TX octets low
#define PORT_REG_TX_H                   (PORT_REG_CNT + 0x0141) ///< TX octets high
#define PORT_REG_TX_UNICAST_L           (PORT_REG_CNT + 0x0142) ///< RX unicast low
#define PORT_REG_TX_UNICAST_H           (PORT_REG_CNT + 0x0143) ///< RX unicast high
#define PORT_REG_TX_BROADCAST_L         (PORT_REG_CNT + 0x0144) ///< RX broadcast low
#define PORT_REG_TX_BROADCAST_H         (PORT_REG_CNT + 0x0145) ///< RX broadcast high
#define PORT_REG_TX_MULTICAST_L         (PORT_REG_CNT + 0x0146) ///< RX multicast low
#define PORT_REG_TX_MULTICAST_H         (PORT_REG_CNT + 0x0147) ///< RX multicast high

#define PORT_REG_TX_HSRPRP_L            (PORT_REG_CNT + 0x0148) ///< RX HSR/PRP frames low
#define PORT_REG_TX_HSRPRP_H            (PORT_REG_CNT + 0x0149) ///< RX HSR/PRP frames high

#define PORT_REG_TX_OVERRUN_L           (PORT_REG_CNT + 0x0150) ///< TX gate close events during transmission low
#define PORT_REG_TX_OVERRUN_H           (PORT_REG_CNT + 0x0151) ///< TX gate close events during transmission high

#define PORT_REG_TX_PREEMPT_FRAG_L      (PORT_REG_CNT + 0x0152) ///< TX preemption extra fragments low
#define PORT_REG_TX_PREEMPT_FRAG_H      (PORT_REG_CNT + 0x0153) ///< TX preemption extra fragments high

#define PORT_REG_TX_PRIQ_DROP_L         (PORT_REG_CNT + 0x0160) ///< TX priority queue full low
#define PORT_REG_TX_PRIQ_DROP_H         (PORT_REG_CNT + 0x0161) ///< TX priority queue full high
#define PORT_REG_TX_EARLY_DROP_L        (PORT_REG_CNT + 0x0162) ///< TX internal memory full low
#define PORT_REG_TX_EARLY_DROP_H        (PORT_REG_CNT + 0x0163) ///< TX internal memory full high

/**
 * FRS Port Inbound Policy (IPO) Registers.
 */
#define PORT_REG_IPO_BASE               0x4000
#define PORT_REG_ETH_FILTER_CFG         (PORT_REG_IPO_BASE + 0x0000)    ///< IPO filter cfg

// FRS Port IPO Configuration Register bits, Ethernet filter
#define PORT_ETH_ADDR_ENABLE            0x0001  ///< entry enable
#define PORT_ETH_ADDR_SOURCE            0x0002  ///< match on source address
#define PORT_ETH_ADDR_DEST              0x0000  ///< match on destination address
#define PORT_ETH_ADDR_NO_HSR_TAG        0x0400
#define PORT_ETH_ADDR_NO_PRP_TRAILER    0x0800
#define PORT_ETH_ADDR_CMP_LENGTH(l)     (((l) << 2) & 0x00FC)   ///< set compare length
#define PORT_ETH_ADDR_CMP_LENGTH_GET(l) (((l) >> 2) & 0x003F)   ///< set compare length

#define PORT_ETH_ADDR_FROM_PRIO(p) \
    (((((p) >> 1) & 0x3) << 12) | (((p) & 0x1) << 15))          ///< set priority
#define PORT_ETH_ADDR_TO_PRIO(p) \
    (((((p) >> 12) & 0x3) << 1) | (((p) >> 15) & 0x1))          ///< get priority

#define PORT_ETH_ADDR_PRIORITY_MASK     PORT_ETH_ADDR_FROM_PRIO(7)      ///< bitmask for priority
#define PORT_ETH_ADDR_PRESERVE_PRIORITY 0x4000                  ///< preserve priority

// FRS Port IPO CFG1 register bits
#define PORT_ETH_ADDR_POLICER_MASK              0x003f  ///< IPO CFG1 bitmask for policer
#define PORT_ETH_ADDR_CMP_ORDER                 0x4000  ///< IPO CFG1 bit compare order (1=MSb first)
#define PORT_ETH_ADDR_CT_DISABLE                0x8000  ///< IPO CFG1 cut-through disable

#define PORT_REG_ETH_ADDR_CFG0_OFFSET           0x0000  ///< IPO filter addr cfg0
#define PORT_REG_ETH_ADDR_FWD_ALLOW_OFFSET      0x0001  ///< IPO filter fwd allow
#define PORT_REG_ETH_ADDR_FWD_MIRROR_OFFSET     0x0002  ///< IPO filter mirror
#define PORT_REG_ETH_ADDR_CFG1_OFFSET           0x0003  ///< IPO filter addr cfg1
#define PORT_REG_ETH_ADDR_0_OFFSET              0x0004  ///< IPO filter address part 0
#define PORT_REG_ETH_ADDR_1_OFFSET              0x0005  ///< IPO filter address part 1
#define PORT_REG_ETH_ADDR_2_OFFSET              0x0006  ///< IPO filter address part 2

// Valid IPO addresses are from 0 to 15 (entry)

/// IPO filter addr cfg0
#define PORT_REG_ETH_ADDR_CFG0(entry)       (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_CFG0_OFFSET)
/// IPO filter addr cfg1
#define PORT_REG_ETH_ADDR_CFG1(entry)       (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_CFG1_OFFSET)
/// IPO filter fwd allow
#define PORT_REG_ETH_ADDR_FWD_ALLOW(entry)  (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_FWD_ALLOW_OFFSET)
/// IPO filter fwd mirror
#define PORT_REG_ETH_ADDR_FWD_MIRROR(entry) (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_FWD_MIRROR_OFFSET)
/// IPO filter address part 0
#define PORT_REG_ETH_ADDR_0(entry)          (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_0_OFFSET)
/// IPO filter address part 1
#define PORT_REG_ETH_ADDR_1(entry)          (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_1_OFFSET)
/// IPO filter address part 2
#define PORT_REG_ETH_ADDR_2(entry)          (PORT_REG_ETH_FILTER_CFG | ((entry) << 4) | PORT_REG_ETH_ADDR_2_OFFSET)

// For compatibility
#define PORT_REG_ETH_ADDR_CFG(entry)         PORT_REG_ETH_ADDR_CFG0(entry)

#endif
