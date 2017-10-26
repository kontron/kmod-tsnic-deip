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

#ifndef DEIPCE_FPTS_TYPES_H
#define DEIPCE_FPTS_TYPES_H

#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/ptp_clock.h>

#define DEIPCE_FPTS_MAX_DEVICES  32

struct deipce_fpts_drv_priv;
struct deipce_fpts_dev_priv;

/**
 * Event information structure.
 */
struct deipce_fpts_event {
    //struct ptp_clock_time time;         ///< event time
    struct timespec64 time;             ///< event time
    uint32_t counter;                   ///< total event count
};

/**
 * FPTS device private information structure.
 */
struct deipce_fpts_dev_priv {
    struct list_head list;              ///< linked list
    struct deipce_fpts_drv_priv *drv;   ///< back reference
    struct platform_device *pdev;       ///< FPTS platform device
    unsigned int dev_num;               ///< number of this device
    void __iomem *ioaddr;               ///< memory mapped I/O address
    struct deipce_fpts_event last_event;   ///< last read event information
};

/**
 * FPTS driver private information structure.
 */
struct deipce_fpts_drv_priv {
    struct list_head devices;           ///< our devices
    /// used device numbers
    DECLARE_BITMAP(used_devices, DEIPCE_FPTS_MAX_DEVICES);
};

static inline uint16_t deipce_fpts_read16(struct deipce_fpts_dev_priv *dp,
                                          unsigned int reg)
{
    return ioread16(dp->ioaddr + reg);
}

static inline uint32_t deipce_fpts_read32(struct deipce_fpts_dev_priv *dp,
                                          unsigned int reg)
{
    return ioread32(dp->ioaddr + reg);
}

static inline void deipce_fpts_write16(struct deipce_fpts_dev_priv *dp,
                                       unsigned int reg, uint16_t value)
{
    iowrite16(value, dp->ioaddr + reg);
}

#endif
