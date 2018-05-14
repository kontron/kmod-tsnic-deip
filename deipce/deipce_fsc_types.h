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

#ifndef DEIPCE_FSC_TYPES_H
#define DEIPCE_FSC_TYPES_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/workqueue.h>

#include "deipce_types.h"

#define DEIPCE_FSC_MAX_SCHEDULERS  8
#define DEIPCE_FSC_MAX_OUTPUTS     64
#define DEIPCE_FSC_MAX_TABLES      2
#define DEIPCE_FSC_MAX_ROWS        1024

struct ptp_clock_info;

/**
 * Data type for representing IEEE 802.1Qbv rational numbers.
 */
struct deipce_fsc_fraction {
    uint32_t numerator;
    uint32_t denominator;
};

/**
 * Data type for representing sub-nanosecond precision times for FSC.
 */
struct deipce_fsc_time {
    uint64_t sec;                       ///< seconds part
    uint32_t nsec;                      ///< nanoseconds part
    uint32_t subnsec;                   ///< subnanoseconds part (2^-32)
};

/**
 * FSC table context.
 */
struct deipce_fsc_table {
    unsigned int num;                   ///< table number
    unsigned int num_rows_used;         ///< number of rows in use
};

/**
 * FSC user parameters.
 */
struct deipce_fsc_sched_param {
    struct timespec64 base_time;        ///< base time for schedule start
    struct deipce_fsc_time cycle_time_fsc;      ///< cycle time in FSC format
    struct deipce_fsc_fraction cycle_time;      ///< cycle time in user format
    uint32_t cycle_time_ext;            ///< max. cycle time extension (ns)
    bool cycle_time_overflow;           ///< requested cycle time is bigger
                                        ///< than what is supported
};

/**
 * FSC user parameters context.
 */
struct deipce_fsc_sched_param_ctx {
    struct deipce_fsc_sched_param param;
};

/**
 * FSC scheduler context.
 */
struct deipce_fsc_sched {
    unsigned int num;                   ///< scheduler number
    unsigned int table_num;             ///< current schedule table number
                                        ///< (administrative)
    bool config_change;                 ///< schedule change in progress
    struct deipce_fsc_sched_param_ctx admin;    ///< administrative parameters
    struct deipce_fsc_sched_param_ctx oper;     ///< operational parameters
    struct delayed_work start_work;     ///< start schedule in the future
    enum link_mode link_mode;           ///< current link mode
    struct timespec64 start_time;       ///< schedule table start time
    struct timespec64 change_time;      ///< schedule change time for user
    bool start_time_in_past;            ///< start_time is in the past
    uint64_t errors;                    ///< config change error counter

    struct deipce_fsc_table table[DEIPCE_FSC_MAX_TABLES];       ///< tables
};

/**
 * FSC device private information structure.
 */
struct deipce_fsc_dev_priv {
    struct list_head list;              ///< linked list
    struct platform_device *pdev;       ///< FSC platform device

    unsigned int clock_cycle_length;    ///< clock cycle length in nanoseconds
    unsigned int num_sched;             ///< number of schedulers
    unsigned int num_outputs;           ///< number of outputs
    unsigned int num_rows;              ///< number of scheduler table rows
    uint64_t output_mask;               ///< bitmask of valid output bits
    uint64_t hold_output_mask;          ///< bitmask of hold output

    struct mutex lock;                  ///< synchronize access to registers
    void __iomem *ioaddr;               ///< memory mapped I/O registers
    struct deipce_time *time;           ///< associated time interface

    /// schedulers of this FSC
    struct deipce_fsc_sched sched[DEIPCE_FSC_MAX_SCHEDULERS];
};

/**
 * FSC driver private information structure.
 */
struct deipce_fsc_drv_priv {
    struct list_head devices;           ///< our devices
};

struct deipce_fsc_drv_priv *deipce_fsc_get_drv_priv(void);

static inline uint16_t deipce_fsc_read16(struct deipce_fsc_dev_priv *dp,
                                         unsigned int addr)
{
    return ioread16(dp->ioaddr + addr);
}

static inline void deipce_fsc_write16(struct deipce_fsc_dev_priv *dp,
                                      unsigned int addr, uint16_t value)
{
    iowrite16(value, dp->ioaddr + addr);
}

static inline uint32_t deipce_fsc_read32(struct deipce_fsc_dev_priv *dp,
                                         unsigned int addr)
{
    return ioread32(dp->ioaddr + addr);
}

static inline void deipce_fsc_write32(struct deipce_fsc_dev_priv *dp,
                                      unsigned int addr, uint32_t value)
{
    iowrite32(value, dp->ioaddr + addr);
}

static inline uint64_t deipce_fsc_read48(struct deipce_fsc_dev_priv *dp,
                                         unsigned int addr)
{
    return
        (uint64_t)ioread32(dp->ioaddr + addr) |
        (uint64_t)ioread16(dp->ioaddr + addr + 4) << 32;
}

static inline void deipce_fsc_write48(struct deipce_fsc_dev_priv *dp,
                                      unsigned int addr, uint64_t value)
{
    iowrite32((uint32_t)value, dp->ioaddr + addr);
    iowrite16((uint32_t)(value >> 32), dp->ioaddr + addr + 4);
}

static inline uint64_t deipce_fsc_read64(struct deipce_fsc_dev_priv *dp,
                                         unsigned int addr)
{
    return
        (uint64_t)ioread32(dp->ioaddr + addr) |
        (uint64_t)ioread32(dp->ioaddr + addr + 4) << 32;
}

static inline void deipce_fsc_write64(struct deipce_fsc_dev_priv *dp,
                                      unsigned int addr, uint64_t value)
{
    iowrite32((uint32_t)value, dp->ioaddr + addr);
    iowrite32((uint32_t)(value >> 32), dp->ioaddr + addr + 4);
}

uint64_t deipce_fsc_read_outputs(struct deipce_fsc_dev_priv *dp,
                                 unsigned int addr);

void deipce_fsc_write_outputs(struct deipce_fsc_dev_priv *dp,
                              unsigned int addr, uint64_t value);

#endif
