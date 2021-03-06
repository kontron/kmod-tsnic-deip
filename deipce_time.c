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

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/types.h>

#include "deipce_main.h"
#include "deipce_clock_ptp.h"
#include "deipce_clock_main.h"
#include "deipce_ibc_main.h"
#include "deipce_time.h"
#include "deipce_time_priv.h"

/**
 * Get PHC index of clock by clock selector.
 * @param dt Time interface.
 * @param sel Clock selector.
 */
int deipce_time_get_phc(struct deipce_time *dt, enum deipce_time_sel sel)
{
    struct flx_frtc_dev_priv *clk = deipce_time_get_clock(dt, sel);

    if (!clk)
        return -1;

    return deipce_clock_get_phc_index(clk);
}

/**
 * Get PHC index of an available clock.
 * @param dt Time interface.
 * @param num Clock number.
 */
int deipce_time_get_avail_phc(struct deipce_time *dt, unsigned int num)
{
    struct flx_frtc_dev_priv *clk = dt->clock[num];

    if (!clk)
        return -1;

    return deipce_clock_get_phc_index(clk);
}

/**
 * Set clock to use for given purpose.
 * @param dt Time interface.
 * @param sel Clock selector.
 * @param phc_index PHC index of the clock to use.
 */
int deipce_time_set_phc(struct deipce_time *dt, enum deipce_time_sel sel,
                        int phc_index)
{
    unsigned long int flags;
    unsigned int new_clock;
    struct flx_frtc_dev_priv *clk;
    int ret = -EINVAL;

    pr_debug(DRV_NAME ": %s() Try set clock for %u to phc %i\n",
             __func__, sel, phc_index);

    for (new_clock = 0; new_clock < DEIPCE_TIME_MAX_CLOCKS; new_clock++) {
        clk = dt->clock[new_clock];
        if (clk && deipce_clock_get_phc_index(clk) == phc_index)
            break;
    }

    pr_debug(DRV_NAME ": %s()   new clock index %u\n", __func__, new_clock);

    if (new_clock >= DEIPCE_TIME_MAX_CLOCKS)
        return -EINVAL;

    spin_lock_irqsave(&dt->lock, flags);

    pr_debug(DRV_NAME ": %s()   old clock index %u\n",
             __func__, dt->cur_clock[sel]);

    if (dt->cur_clock[sel] == new_clock) {
        ret = 0;
        goto out;
    }

    pr_debug(DRV_NAME ": %s()   %s\n", __func__, dt->ibc ? "mux" : "no mux");

    switch (sel) {
    case DEIPCE_TIME_SEL_TS:
        ret = -EOPNOTSUPP;
        break;
    case DEIPCE_TIME_SEL_WRK:
        if (dt->capability.can_mux_wrk)
            ret = deipce_ibc_set(dt->ibc, new_clock, new_clock);
        else
            ret = -EOPNOTSUPP;
        break;
    }

    if (ret)
        goto out;

    dt->cur_clock[sel] = new_clock;

out:
    spin_unlock_irqrestore(&dt->lock, flags);

    pr_debug(DRV_NAME ": %s() Set clock for %u to phc %i clock index %u\n",
             __func__, sel, phc_index, new_clock);

    return ret;
}

/**
 * Get current time.
 * @param dt Time interface.
 * @param sel Clock selector.
 * @param time Place for current time from clock.
 */
int deipce_time_get_time(struct deipce_time *dt, enum deipce_time_sel sel,
                         struct timespec64 *time)
{
#if IS_ENABLED(CONFIG_PTP_1588_CLOCK)
    struct flx_frtc_dev_priv *clk = deipce_time_get_clock(dt, sel);

    if (!clk)
        return -ENODEV;

    return clk->ptp_info.gettime64(&clk->ptp_info, time);
#else
    return -EOPNOTSUPP;
#endif
}

/**
 * Get time granularity.
 * @param dt Time interface.
 * @param sel Clock selector.
 * @param granularity Place for granularity, unit is tenths of nanoseconds.
 */
int deipce_time_get_granularity(struct deipce_time *dt,
                                enum deipce_time_sel sel,
                                uint32_t *granularity)
{
    struct flx_frtc_dev_priv *clk = deipce_time_get_clock(dt, sel);

    if (!clk)
        return -ENODEV;

    deipce_clock_get_granularity(clk, granularity);

    return 0;
}

// Until autoconfig is in place
#ifndef DEIPCE_AUTOCONFIG

/// List of all time interfaces.
static struct list_head deipce_time_list;

/**
 * Initialize time interface functionality.
 */
void deipce_time_init_driver(void)
{
    INIT_LIST_HEAD(&deipce_time_list);

    return;
}

/**
 * Find time interface for a given clock.
 * @param clk Clock instance.
 * @return Time interface which manages given clock, or NULL.
 */
static struct deipce_time *deipce_time_find_by_clock(
        struct flx_frtc_dev_priv *clk)
{
    struct deipce_time *dt;
    unsigned int clock_num;

    list_for_each_entry(dt, &deipce_time_list, list) {
        for (clock_num = 0; clock_num < DEIPCE_TIME_MAX_CLOCKS; clock_num++) {
            if (dt->clock[clock_num] == clk)
                return dt;
        }
    }

    return NULL;
}

/**
 * Create new time interface instance.
 */
static struct deipce_time *deipce_time_create(void)
{
    static struct deipce_time *dt;

    dt = kmalloc(sizeof(*dt), GFP_KERNEL);
    if (!dt)
        return NULL;

    *dt = (struct deipce_time){
        .capability = {
            .can_mux_ts = false,
        },
    };

    INIT_LIST_HEAD(&dt->list);
    spin_lock_init(&dt->lock);

    list_add(&dt->list, &deipce_time_list);

    return dt;
}

/**
 * Create or update time interface for given setup.
 * @param clk1 First clock.
 * @param clk2 Seconds clock or NULL.
 * @param fpts FPTS instance or NULL.
 */
int deipce_time_create_by_clocks(struct flx_frtc_dev_priv *clk1,
                                 struct flx_frtc_dev_priv *clk2,
                                 struct deipce_fpts_dev_priv *fpts)
{
    struct deipce_time *dt = deipce_time_find_by_clock(clk1);
    int ret = 0;

    if (!dt) {
        dt = deipce_time_create();
        if (!dt)
            return -ENOMEM;

        pr_debug(DRV_NAME ": %s() New clk1 %i clk2 %i %s\n",
                 __func__,
                 clk1 ? deipce_clock_get_phc_index(clk1) : -1,
                 clk2 ? deipce_clock_get_phc_index(clk2) : -1,
                 fpts ? "fpts" : "");

        dt->clock[0] = clk1;
    }

    if (clk2) {
        pr_debug(DRV_NAME ": %s() Add clk1 %i clk2 %i %s\n",
                 __func__,
                 clk1 ? deipce_clock_get_phc_index(clk1) : -1,
                 clk2 ? deipce_clock_get_phc_index(clk2) : -1,
                 fpts ? "fpts" : "");

        dt->clock[1] = clk2;

        /*
         * Use second clock for worker by default.
         * This also determines the assumed setup in case there is no MUX.
         */
        dt->cur_clock[DEIPCE_TIME_SEL_WRK] = 1;
    }

    if (clk2 && fpts) {
        ret = flx_frtc_ptp_init_devxtstamp(clk2);
        if (ret == 0)
            dt->fpts = fpts;
    }

    pr_debug(DRV_NAME ": %s() Cur clk1 %i clk2 %i %s %s %s\n",
             __func__,
             dt->clock[0] ? deipce_clock_get_phc_index(dt->clock[0]) : -1,
             dt->clock[1] ? deipce_clock_get_phc_index(dt->clock[1]) : -1,
             dt->fpts ? "fpts" : "",
             dt->capability.can_mux_ts ? "mux-ts" : "",
             dt->capability.can_mux_wrk ? "mux-wrk" : "");

    return ret;
}

/**
 * Create or update time interface for given setup.
 * @param clk1 First clock.
 * @param clk2 Seconds clock or NULL.
 * @param fpts FPTS instance or NULL.
 */
void deipce_time_remove_clock(struct flx_frtc_dev_priv *clk)
{
    struct deipce_time *dt;
    unsigned int clock_num;

    pr_debug(DRV_NAME ": %s() Remove time interface for PHC %i\n",
             __func__, deipce_clock_get_phc_index(clk));

    list_for_each_entry(dt, &deipce_time_list, list) {
        for (clock_num = 0; clock_num < DEIPCE_TIME_MAX_CLOCKS; clock_num++) {
            if (dt->clock[clock_num] == clk) {
                list_del(&dt->list);
                kfree(dt);
                return;
            }
        }
    }

    return;
}

/**
 * Update IBC information to time interface.
 * @param clk Clock instance whose time interface to update.
 * @param ibc IBC instance used for time interface.
 */
int deipce_time_add_mux(struct flx_frtc_dev_priv *clk,
                        struct deipce_ibc_dev_priv *ibc)
{
    struct deipce_time *dt = deipce_time_find_by_clock(clk);
    unsigned int cur_clock;
    int ret;

    if (!dt)
        return -ENODEV;

    pr_debug(DRV_NAME ": %s() Add ibc\n", __func__);

    dt->ibc = ibc;
    dt->capability.can_mux_wrk = true;

    // Ensure MUX matches our idea or current clock selection.
    cur_clock = dt->cur_clock[DEIPCE_TIME_SEL_WRK];
    ret = deipce_ibc_set(dt->ibc, cur_clock, cur_clock);

    return ret;
}

/**
 * Remove IBC information from time interface.
 * @param ibc IBC instance.
 */
void deipce_time_remove_mux(struct deipce_ibc_dev_priv *ibc)
{
    struct deipce_time *dt;

    pr_debug(DRV_NAME ": %s() Remove ibc\n", __func__);

    list_for_each_entry(dt, &deipce_time_list, list) {
        if (dt->ibc == ibc) {
            dt->ibc = NULL;
            dt->capability.can_mux_wrk = false;
            break;
        }
    }

    return;
}

/**
 * Get time interface for a given clock.
 * @param clk Clock instance.
 * @return Time interface which manages given clock.
 */
struct deipce_time *deipce_time_get_by_clock(struct flx_frtc_dev_priv *clk)
{
    return deipce_time_find_by_clock(clk);
}

#endif

