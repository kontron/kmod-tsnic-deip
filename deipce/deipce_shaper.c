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

/// Uncomment to enable debug messages
//#define DEBUG

#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_fqtss_sysfs.h"
#include "deipce_shaper.h"

/**
 * Calculate actual shaper bit rate.
 * @param clk Switch clock frequency.
 * @param addend Addend value as in register, in 1/4096 credits.
 * @return Rate in bits per second.
 */
uint32_t deipce_shaper_bitrate(uint32_t clk, uint16_t addend)
{
    // addend x clk x 8 / (8 x 4096) = addend x clk x 2^-12
    return (uint32_t)(((uint64_t)addend * (uint64_t)clk) >> 12);
}

/**
 * Calculate shaper addend value for a given bitrate.
 * If some traffic is wanted through, select smallest possible rate.
 * @param clk Switch clock frequency.
 * @param rate Rate in bits per second.
 * @return Addend value for bitrate.
 */
uint16_t deipce_shaper_addend(uint32_t clk, uint32_t rate)
{
    uint64_t addend_tmp;
    uint32_t min_nonzero_rate = deipce_shaper_bitrate(clk, 1);

    if (rate > 0 && rate < min_nonzero_rate)
        return 1;

    // rate x 8 x 4096 / (clk x 8) = (rate x 2^12) / clk
    // Round up.
    addend_tmp = ((uint64_t)rate << 12) + clk - 1u;
    do_div(addend_tmp, clk);

    return (uint16_t)addend_tmp;
}

/**
 * Initialize shaper support.
 * @param pp Port privates.
 */
int deipce_shaper_init(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    unsigned int i;
    int ret = 0;
    uint16_t addend;

    if (!(dp->features.flags & FLX_FRS_FEAT_SHAPER))
        return 0;

    netdev_dbg(pp->netdev, "%s() Init shapers\n", __func__);

    spin_lock_init(&pp->shaper.lock);
    for (i = 0; i < dp->features.prio_queues; i++) {
        addend = deipce_read_port_reg(pp, PORT_REG_SHAPER(i));
        pp->shaper.bitrate[i] = deipce_shaper_bitrate(dp->features.clock_freq,
                                                      addend);
    }

    ret = deipce_fqtss_sysfs_init(pp);
    if (ret)
        return ret;

    return 0;
}

/**
 * Cleanup shaper support.
 * @param pp Port privates.
 */
void deipce_shaper_cleanup(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;

    if (!(dp->features.flags & FLX_FRS_FEAT_SHAPER))
        return;

    netdev_dbg(pp->netdev, "%s() Cleanup shapers\n", __func__);

    deipce_fqtss_sysfs_cleanup(pp);

    return;
}

