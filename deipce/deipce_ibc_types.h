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

#ifndef DEIPCE_IBC_TYPES_H
#define DEIPCE_IBC_TYPES_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "deipce_ibc_main.h"

struct ptp_clock_info;

/**
 * IBC device private information structure.
 */
struct deipce_ibc_dev_priv {
    struct list_head list;              ///< linked list
    struct platform_device *pdev;       ///< IBC platform device

    void __iomem *ioaddr;               ///< memory mapped I/O address
    struct mutex lock;                  ///< synchronize access and changes
    /// selectable clocks
    struct deipce_ibc_phc_info phc[DEIPCE_IBC_MAX_CLOCKS];
};

/**
 * IBC driver private information structure.
 */
struct deipce_ibc_drv_priv {
    struct list_head devices;           ///< our devices
};

static inline uint16_t deipce_ibc_read_reg(struct deipce_ibc_dev_priv *dp,
                                           unsigned int reg)
{
    return ioread16(dp->ioaddr + reg);
}

static inline void deipce_ibc_write_reg(struct deipce_ibc_dev_priv *dp,
                                        unsigned int reg, uint16_t value)
{
    iowrite16(value, dp->ioaddr + reg);
}

#endif
