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

#ifndef DEIPCE_CLOCK_TYPES_H
#define DEIPCE_CLOCK_TYPES_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/io.h>

/**
 * Time information from clock device.
 */
struct flx_frtc_time_data {
    uint64_t counter;           ///< counter value
    /// timestamp (NCO time) when counter value has been updated
    struct timespec64 timestamp;
};

/**
 * Private FRTC device information.
 */
struct flx_frtc_dev_priv {
    struct list_head list;              ///< linked list

    struct platform_device *pdev;       ///< associated platform device

    spinlock_t lock;                    ///< synchronize access
    void __iomem *ioaddr;               ///< component ioaddr

#ifdef CONFIG_PTP_1588_CLOCK
    struct ptp_clock_info ptp_info;     ///< Linux PTP hardware clock support
    struct ptp_clock *ptp_clock;        ///< PHC instance
    struct {
        struct deipce_fpts_dev_priv *fpts;      ///< external timestamper
        /// FPTS trigger clock node for 2nd PHC patch cross-device timestamping
        struct device_node *trigger_node;
        /// FPTS trigger clock for 2nd PHC patch cross-device timestamping
        struct flx_frtc_dev_priv *trigger;
    } event;                            ///< FPTS event handling
#endif

    uint32_t step_nsec;         ///< nominal step size nanoseconds part
    uint32_t step_subnsec;      ///< nominal step size subnanoseconds part
    uint32_t adjust_scale_factor;       ///< scaling factor for freq adjust
    uint32_t cur_step_nsec;     ///< current step size nanoseconds part
};

/**
 * Driver privates.
 */
struct flx_frtc_drv_priv {
    struct list_head devices;   ///< list of devices
};

struct flx_frtc_drv_priv *flx_frtc_get_drv(void);

/**
 * Read NCO register value using configured access method.
 */
static inline uint32_t flx_nco_read32(struct flx_frtc_dev_priv *dp,
                                      uint32_t addr)
{
    return ioread32(dp->ioaddr + addr);
}

/**
 * Write NCO register using configured access method.
 */
static inline void flx_nco_write32(struct flx_frtc_dev_priv *dp,
                                   uint32_t addr, uint32_t value)
{
    iowrite32(value, dp->ioaddr + addr);
}

#endif
