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

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/sched.h>
#include <linux/platform_device.h>

#include "deipce_types.h"
#include "deipce_main.h"
#include "deipce_time.h"
#include "deipce_fsc_types.h"
#include "deipce_fsc_if.h"
#include "deipce_fsc_hw.h"

/// Downcounter start value to use, maps to number of bytes
#define FSC_DC_START 1540

/**
 * Maximum number of seconds to set FSC table start time register value to be
 * in the future. For longer times a work is scheduled for later.
 */
#define FSC_START_WORK_SEC              64

/**
 * Minimum number of seconds from delayed start work to table start time,
 * in order to not delay for too long because of clock differences.
 * Must be <= FSC_START_WORK_SEC.
 */
#define FSC_START_WORK_MIN_ADVANCE_SEC  FSC_START_WORK_SEC

/**
 * Read 1-64 bits (depending on number of outputs) from given register address.
 * @param dp Device privates.
 * @param addr Register address to read from.
 */
uint64_t deipce_fsc_read_outputs(struct deipce_fsc_dev_priv *dp,
                                 unsigned int addr)
{
    uint64_t value;

    // Do not read reserved registers.
    if (dp->num_outputs > 3*16) {
        value = deipce_fsc_read64(dp, addr);
    }
    else if (dp->num_outputs > 2*16) {
        value =
            (uint64_t)deipce_fsc_read32(dp, addr) |
            ((uint64_t)deipce_fsc_read16(dp, addr + sizeof(uint32_t)) << 32);
    }
    else if (dp->num_outputs > 16) {
        value = deipce_fsc_read32(dp, addr);
    }
    else {
        value = deipce_fsc_read16(dp, addr);
    }

    // Mask non-existent bits to zero.
    value &= dp->output_mask;

    return value;
}

/**
 * Write 1-64 bits (depending on number of outputs) to given register address.
 * @param dp Device privates.
 * @param addr Register address to write to.
 * @param value Value to write.
 */
void deipce_fsc_write_outputs(struct deipce_fsc_dev_priv *dp,
                              unsigned int addr, uint64_t value)
{
    // Mask non-existent bits to zero.
    value &= dp->output_mask;

    // Do not write to reserved registers.
    if (dp->num_outputs > 3*16) {
        deipce_fsc_write64(dp, addr, value);
    }
    else if (dp->num_outputs > 2*16) {
        deipce_fsc_write32(dp, addr, (uint32_t)value);
        deipce_fsc_write16(dp, addr + sizeof(uint32_t),
                           (uint16_t)(value >> 32));
    }
    else if (dp->num_outputs > 16) {
        deipce_fsc_write32(dp, addr, (uint32_t)value);
    }
    else {
        deipce_fsc_write16(dp, addr, (uint16_t)value);
    }

    return;
}

/**
 * Calculate seconds, nanoseconds and subnanoseconds parts from fraction
 * representing time in seconds.
 * @param fract Fraction to split.
 * @param time Place for result.
 */
void deipce_fsc_split_fraction(const struct deipce_fsc_fraction *fract,
                               struct deipce_fsc_time *time)
{
    uint64_t num = fract->numerator;
    uint32_t rem = do_div(num, fract->denominator);

    time->sec = num;

    num = (uint64_t)rem * NSEC_PER_SEC;
    rem = do_div(num, fract->denominator);
    time->nsec = num;

    num = (uint64_t)rem << 32;
    rem = do_div(num, fract->denominator);
    time->subnsec = num;

    return;
}

/**
 * Get effective cycle time in rational form.
 * @param cycle_time Place for effective cycle time.
 * @param param Scheduler parameter structure.
 */
void deipce_fsc_effective_cycle_time(
        struct deipce_fsc_fraction *cycle_time,
        const struct deipce_fsc_sched_param *param)
{
    if (param->cycle_time_overflow) {
        // Cycle time is less than 1 s and an integral number of nanoseconds.
        cycle_time->numerator = param->cycle_time_fsc.nsec;
        cycle_time->denominator = NSEC_PER_SEC;
    }
    else {
        *cycle_time = param->cycle_time;
    }

    return;
}

/**
 * Wait until scheduler table row read or write has completed.
 * Checks also for access errors.
 * @param dp Device privates.
 */
static int deipce_fsc_wait_row_access(struct deipce_fsc_dev_priv *dp)
{
    unsigned int timeout = 100;
    uint16_t cmd = 0;

    // Wait read to complete.
    do {
        if (timeout-- == 0) {
            dev_err(&dp->pdev->dev, "Row access timeout\n");
            return -EBUSY;
        }
        cpu_relax();

        cmd = deipce_fsc_read16(dp, FSC_ROW_ACCESS_CMD0);
    } while (cmd & FSC_ROW_ACCESS_CMD0_TRANSFER);

    if (cmd & FSC_ROW_ACCESS_CMD0_ACCESS_ERR) {
        dev_err(&dp->pdev->dev, "Row access error, table in use\n");
        return -EBUSY;
    }

    return 0;
}

/**
 * Read scheduler table row.
 * @param dp Device privates.
 * @param sched_num Scheduler number.
 * @param table_num Table number.
 * @param row_num Row number.
 * @param cycles Place for time in clock cycles.
 * @param outputs Place for bitmask of outputs.
 */
int deipce_fsc_read_row(struct deipce_fsc_dev_priv *dp,
                        unsigned int sched_num, unsigned int table_num,
                        unsigned int row_num,
                        uint16_t *cycles, uint64_t *outputs)
{
    int ret = -ENODEV;
    uint16_t cmd = 0;

    deipce_fsc_write16(dp, FSC_ROW_ACCESS_CMD1,
                       row_num & FSC_ROW_ACCESS_CMD1_ROW_MASK);

    cmd =
        (sched_num & FSC_ROW_ACCESS_CMD0_SCHED_MASK) |
        ((table_num & FSC_ROW_ACCESS_CMD0_TABLE_MASK) <<
         FSC_ROW_ACCESS_CMD0_TABLE_SHIFT) |
        FSC_ROW_ACCESS_CMD0_READ |
        FSC_ROW_ACCESS_CMD0_TRANSFER;

    deipce_fsc_write16(dp, FSC_ROW_ACCESS_CMD0, cmd);

    ret = deipce_fsc_wait_row_access(dp);
    if (ret)
        return ret;

    *outputs = deipce_fsc_read_outputs(dp, FSC_ROW_DATA_OUT0);
    *cycles = deipce_fsc_read16(dp, FSC_ROW_DATA_CYCLES);

    return 0;
}

/**
 * Write scheduler table row.
 * @param dp Device privates.
 * @param sched_num Scheduler number.
 * @param table_num Table number.
 * @param row_num Row number.
 * @param cycles Time in clock cycles.
 * @param outputs Bitmask of outputs.
 */
int deipce_fsc_write_row(struct deipce_fsc_dev_priv *dp,
                         unsigned int sched_num, unsigned int table_num,
                         unsigned int row_num,
                         uint16_t cycles, uint64_t outputs)
{
    int ret = -ENODEV;
    uint16_t cmd = 0;

    deipce_fsc_write16(dp, FSC_ROW_ACCESS_CMD1,
                       row_num & FSC_ROW_ACCESS_CMD1_ROW_MASK);

    // Do not write to reserved registers.
    deipce_fsc_write_outputs(dp, FSC_ROW_DATA_OUT0, outputs);
    deipce_fsc_write16(dp, FSC_ROW_DATA_CYCLES, cycles);

    cmd =
        (sched_num & FSC_ROW_ACCESS_CMD0_SCHED_MASK) |
        ((table_num & FSC_ROW_ACCESS_CMD0_TABLE_MASK) <<
         FSC_ROW_ACCESS_CMD0_TABLE_SHIFT) |
        FSC_ROW_ACCESS_CMD0_WRITE |
        FSC_ROW_ACCESS_CMD0_TRANSFER;
    deipce_fsc_write16(dp, FSC_ROW_ACCESS_CMD0, cmd);

    ret = deipce_fsc_wait_row_access(dp);
    if (ret)
        return ret;

    return 0;
}

/**
 * Clear all scheduler table rows by writing zeros.
 * @param dp Device privates.
 * @param sched_num Scheduler number.
 * @param table_num Table number.
 */
int deipce_fsc_clear_rows(struct deipce_fsc_dev_priv *dp,
                          unsigned int sched_num, unsigned int table_num)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    struct deipce_fsc_table *table = &sched->table[table_num];
    unsigned int row_num;
    int ret = -ENODEV;

    for (row_num = 0; row_num < dp->num_rows; row_num++) {
        ret = deipce_fsc_write_row(dp, sched_num, table_num,
                                   row_num, 0, 0);
        if (ret)
            return ret;
    }

    table->num_rows_used = 0;

    return 0;
}

/**
 * Calculate total time of all rows in schedule time.
 * @param dp Device privates.
 * @param sched_num Scheduler number.
 * @param table_num Scheduler table number.
 * @param total_time Place for calculated total time.
 */
static int deipce_fsc_calc_row_time_sum(struct deipce_fsc_dev_priv *dp,
                                        unsigned int sched_num,
                                        unsigned int table_num,
                                        struct deipce_fsc_time *total_time)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    struct deipce_fsc_table *table = &sched->table[table_num];
    uint16_t cycles = 0;
    uint64_t outputs = 0;
    unsigned int row_num;
    int ret;

    total_time->sec = 0;
    total_time->nsec = 0;
    total_time->subnsec = 0;

    for (row_num = 0; row_num < table->num_rows_used; row_num++) {
        ret = deipce_fsc_read_row(dp, sched_num, table_num, row_num,
                                  &cycles, &outputs);
        if (ret)
            return ret;

        // Total time cannot exceed 1 s.
        total_time->nsec += cycles * dp->clock_cycle_length;
    }

    return 0;
}

/**
 * Calculate time advance (downcounter delay) for given link speed.
 * @param dp Device privates.
 * @param sched Scheduler.
 * @return Time advance in nanoseconds.
 */
static long int deipce_fsc_calc_time_advance(struct deipce_fsc_dev_priv *dp,
                                             struct deipce_fsc_sched *sched)
{
    // Effect of downcounter.
    uint32_t advance = FSC_DC_START;

    switch (sched->link_mode) {
    case LM_1000FULL:
        break;
    case LM_100FULL:
        advance *= 10;
        break;
    case LM_10FULL:
    case LM_DOWN:
        advance *= 100;
        break;
    }

    advance *= dp->clock_cycle_length;
    switch (dp->clock_cycle_length) {
    case 8:
        break;
    case 10:
        // Divide by 1.25.
        advance = (advance * 4) / 5;
        break;
    }

    // Additional fixed delay.
    advance += 10 * dp->clock_cycle_length;

    return advance;
}

/**
 * Advance start time by N x cycle time until it becomes big enough.
 * @param start_time Start time which to advance, must be less than min_time.
 * @param min_time Point of time start_time is advanced at least to.
 * @param cycle_time Advance increment.
 */
static void deipce_fsc_calc_future_start_time(
        struct timespec64 *start_time,
        const struct timespec64 *min_time,
        const struct deipce_fsc_fraction *cycle_time)
{
    struct timespec64 delta = timespec64_sub(*min_time, *start_time);
    uint64_t count;
    uint64_t count_sec;
    uint32_t count_sec_rem;
    uint64_t count_nsec;
    uint32_t count_nsec_rem;
    uint64_t count_res;
    uint64_t num;
    uint32_t rem;
    struct timespec64 advance;

    /*
     * Calculate number of times cycle time must be added to initial start time.
     * Basically N = count = delta / (numerator/denominator) [round up].
     * That could overflow with 64 bit arithmetic if nanoseconds were used
     * directly. Calculate N separately for seconds and nanoseconds,
     * taking remainders into account, and round up the result. So
     *
     * N = count = (delta * denominator) / numerator =
     * (delta_sec * denominator) / numerator +
     * (delta_nsec * denominator) / (numerator * NSEC_PER_SEC)
     *
     * With integer division returning quotient as whole number and remainder,
     * this can be written as
     *
     * count = count_sec + count_sec_rem / numerator +
     * (count_nsec + count_nsec_rem / numerator) / NSEC_PER_SEC
     * = count_sec +
     * (NSEC_PER_SEC * count_sec_rem + count_nsec * numerator + count_nsec_rem) /
     * (NSEC_PER_SEC * numerator)
     *
     * That could still overflow, but only when start_time is very much
     * (in the order of 2^32 seconds, i.e. about 136 years) behind min_time.
     */
    count_sec = delta.tv_sec * cycle_time->denominator;
    count_sec_rem = do_div(count_sec, cycle_time->numerator);

    count_nsec = (uint64_t)delta.tv_nsec * cycle_time->denominator;
    count_nsec_rem = do_div(count_nsec, cycle_time->numerator);

    count_res = (uint64_t)count_sec_rem * NSEC_PER_SEC +
        count_nsec * cycle_time->numerator + count_nsec_rem;
    // Round up.
    count_res += (uint64_t)cycle_time->numerator * NSEC_PER_SEC - 1;
    do_div(count_res, cycle_time->numerator);
    do_div(count_res, NSEC_PER_SEC);

    count = count_sec + count_res;

    // Add calculated number of cycle times to start time.
    num = count * cycle_time->numerator;
    rem = do_div(num, cycle_time->denominator);
    advance.tv_sec = num;

    num = (uint64_t)rem * NSEC_PER_SEC;
    do_div(num, cycle_time->denominator);
    advance.tv_nsec = num;

    pr_debug(DRV_NAME " %s() start0 %lli.%09li cycle_time %u/%u\n",
             __func__, (long long int)start_time->tv_sec, start_time->tv_nsec,
             cycle_time->numerator, cycle_time->denominator);
    pr_debug(DRV_NAME " %s() count %llu advance %lli.%09li\n",
             __func__, count, (long long int)advance.tv_sec, advance.tv_nsec);

    *start_time = timespec64_add(*start_time, advance);

    return;
}

/**
 * Calculate table start time.
 * @param dp Device privates.
 * @param sched Scheduler.
 * @param param Parameters to use for start time calculation.
 * @param future_sec Place for number of seconds calculated start time is
 * in the future.
 */
int deipce_fsc_calc_start_time(struct deipce_fsc_dev_priv *dp,
                               struct deipce_fsc_sched *sched,
                               struct deipce_fsc_sched_param *param,
                               time64_t *future_sec)
{
    struct timespec64 cur_time;
    long int time_advance;
    int ret = 0;

    /*
     * Calculate future table start time:
     * BaseTime + N x CycleTime - time_advance.
     */
    ret = deipce_time_get_time(dp->time, DEIPCE_TIME_SEL_WRK, &cur_time);
    if (ret)
        return ret;

    time_advance = deipce_fsc_calc_time_advance(dp, sched);

    set_normalized_timespec64(&sched->start_time,
                              param->base_time.tv_sec,
                              param->base_time.tv_nsec - time_advance);
    dev_dbg(&dp->pdev->dev,
            "%s() base %lli.%09li tadv 0.%09li start0 %lli.%09li\n",
            __func__,
            (long long int)param->base_time.tv_sec, param->base_time.tv_nsec,
            time_advance,
            (long long int)sched->start_time.tv_sec, sched->start_time.tv_nsec);

    if (sched->start_time.tv_sec < cur_time.tv_sec ||
        (sched->start_time.tv_sec == cur_time.tv_sec &&
         (sched->start_time.tv_nsec <= cur_time.tv_nsec))) {
        // Would be in the past, advance N x CycleTime.
        struct deipce_fsc_fraction cycle_time;

        deipce_fsc_effective_cycle_time(&cycle_time, param);
        deipce_fsc_calc_future_start_time(&sched->start_time, &cur_time,
                                          &cycle_time);
        sched->start_time_in_past = true;
    }
    else {
        sched->start_time_in_past = false;
    }

    set_normalized_timespec64(&sched->change_time,
                              sched->start_time.tv_sec,
                              sched->start_time.tv_nsec + time_advance);

    *future_sec = sched->start_time.tv_sec - cur_time.tv_sec;

    dev_dbg(&dp->pdev->dev,
            "%s() Now %lli.%09li base %lli.%09li tadv 0.%09li "
            "start %lli.%09li in %s\n",
            __func__,
            (long long int)cur_time.tv_sec, cur_time.tv_nsec,
            (long long int)param->base_time.tv_sec, param->base_time.tv_nsec,
            time_advance,
            (long long int)sched->start_time.tv_sec, sched->start_time.tv_nsec,
            sched->start_time_in_past ? "past" : "future");
    dev_dbg(&dp->pdev->dev,
            "%s() Now %i.%09li base %i.%09li tadv 0.%09li "
            "start %i.%09li (8 bit sec)\n",
            __func__,
            (int)(cur_time.tv_sec & FSC_SCHED_TBL_START_TIME_SEC_MASK),
            cur_time.tv_nsec,
            (int)(param->base_time.tv_sec & FSC_SCHED_TBL_START_TIME_SEC_MASK),
            param->base_time.tv_nsec,
            time_advance,
            (int)(sched->start_time.tv_sec & FSC_SCHED_TBL_START_TIME_SEC_MASK),
            sched->start_time.tv_nsec);

    return 0;
}

/**
 * Append row with zero time, but retaining outputs.
 * This makes scheduler stop on that row until next cycle,
 * in case of clock jumps or with inaccuracies (prevents glitches).
 * @param dp Device privates.
 * @param sched Scheduler whose administrative table to fix.
 */
static int deipce_fsc_fix_last_row(struct deipce_fsc_dev_priv *dp,
                                   struct deipce_fsc_sched *sched)
{
    struct deipce_fsc_table *table = &sched->table[sched->table_num];
    uint16_t cycles;
    uint64_t outputs;
    int ret;

    if (table->num_rows_used == 0)
        return 0;

    ret = deipce_fsc_read_row(dp, sched->num, table->num,
                              table->num_rows_used - 1, &cycles, &outputs);
    if (ret)
        return ret;

    dev_dbg(&dp->pdev->dev,
            "%s() Scheduler %u table %u append row %u "
            "interval %u outputs 0x%llx\n",
            __func__, sched->num, table->num, table->num_rows_used,
            0, outputs);

    ret = deipce_fsc_write_row(dp, sched->num, table->num,
                               table->num_rows_used, 0, outputs);
    if (ret)
        return ret;

    return ret;
}

/**
 * Start new schedule.
 * Alternates between the two schedule tables.
 * Schedule table rows must have been written already.
 * @param dp Device privates.
 * @param sched Scheduler whose schedule to start.
 */
static int deipce_fsc_start_schedule(struct deipce_fsc_dev_priv *dp,
                                     struct deipce_fsc_sched *sched)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    struct deipce_fsc_time *cycle_time_fsc = &param->cycle_time_fsc;
    time64_t future_sec;
    int ret = -ENODEV;
    unsigned int oper_table_num =
        (sched->table_num + 1) & (DEIPCE_FSC_MAX_TABLES - 1);
    uint16_t ctrl;
    uint16_t dc;
    uint16_t table_gen;
    uint16_t last_cycle;

    dev_dbg(&dp->pdev->dev, "%s() Switch scheduler %u to table %u\n",
            __func__, sched->num, sched->table_num);

    sched->config_change = true;

    deipce_fsc_split_fraction(&param->cycle_time, cycle_time_fsc);

    /*
     * Non-standard behavior: in case requested cycle time is longer than
     * supported, use the total length of the schedule as cycle time.
     * FLEXDE-668.
     */
    if (cycle_time_fsc->sec > 0) {
        ret = deipce_fsc_calc_row_time_sum(dp, sched->num, sched->table_num,
                                           cycle_time_fsc);
        if (ret)
            return ret;

        param->cycle_time_overflow = true;
    }
    else {
        param->cycle_time_overflow = false;
    }

    ctrl = deipce_fsc_read16(dp, FSC_SCHED_BASE(sched->num) +
                             FSC_SCHED_EME_DIS_CTRL);
    dev_dbg(&dp->pdev->dev, "%s() Scheduler %u emergency disable %s\n",
            __func__, sched->num,
            ctrl & FSC_SCHED_EME_DIS_CTRL_ENABLE ? "active" : "inactive");

    if (ctrl & FSC_SCHED_EME_DIS_CTRL_ENABLE)
        return 0;

    // Set cycle time.
    deipce_fsc_write32(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_CYCLE_TIME_SUBNS,
                       cycle_time_fsc->subnsec &
                       FSC_SCHED_TBL_CYCLE_TIME_SUBNS_MASK);
    deipce_fsc_write32(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_CYCLE_TIME_NS,
                       cycle_time_fsc->nsec);

    // Set downcounter start value and speed.
    dc = FSC_DC_START << FSC_SCHED_GEN_DC_START_SHIFT;

    switch (sched->link_mode) {
    case LM_1000FULL:
        dc |= FSC_SCHED_GEN_DC_SPEED_1000;
        break;
    case LM_100FULL:
        dc |= FSC_SCHED_GEN_DC_SPEED_100;
        break;
    case LM_10FULL:
    case LM_DOWN:
        dc |= FSC_SCHED_GEN_DC_SPEED_10;
        break;
    }

    deipce_fsc_write16(dp, FSC_SCHED_BASE(sched->num) + FSC_SCHED_GEN, dc);

    // Set table start time.
    ret = deipce_fsc_calc_start_time(dp, sched, param, &future_sec);
    if (ret)
        return ret;

    if (future_sec > FSC_START_WORK_SEC) {
        // Too far away in the future to be handled now.
        unsigned long int work_sec = FSC_START_WORK_SEC;

        if (work_sec > future_sec - FSC_START_WORK_MIN_ADVANCE_SEC)
            work_sec = future_sec - FSC_START_WORK_MIN_ADVANCE_SEC;

        dev_dbg(&dp->pdev->dev,
                "%s() scheduler %u start delayed work in %lu s\n",
                __func__, sched->num, work_sec);

        mod_delayed_work(drv->wq_low, &sched->start_work,
                         msecs_to_jiffies(work_sec * MSEC_PER_SEC));

        return 0;
    }

    deipce_fsc_write16(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_START_TIME_SEC,
                       (uint16_t)sched->start_time.tv_sec &
                       FSC_SCHED_TBL_START_TIME_SEC_MASK);
    deipce_fsc_write32(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_START_TIME_NS,
                       sched->start_time.tv_nsec);

    deipce_fsc_write16(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_CYCLE_COUNT, 0);

    table_gen = deipce_fsc_read16(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                         sched->table_num) +
                                  FSC_SCHED_TBL_GEN);

    ret = deipce_fsc_fix_last_row(dp, sched);
    if (ret)
        return ret;

    table_gen |= FSC_SCHED_TBL_GEN_CAN_USE;
    table_gen &= ~FSC_SCHED_TBL_GEN_STOP_AT_LAST;
    deipce_fsc_write16(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_GEN,
                       table_gen);

    // Handle cycle time extension.
    if (param->cycle_time_ext > 0 && cycle_time_fsc->nsec > 0) {
        uint16_t last_cycle_inc =
            param->cycle_time_ext / cycle_time_fsc->nsec;

        table_gen = deipce_fsc_read16(dp,
                                      FSC_SCHED_TBL_BASE(sched->num,
                                                         oper_table_num) +
                                      FSC_SCHED_TBL_GEN);
        table_gen |= FSC_SCHED_TBL_GEN_STOP_AT_LAST;
        last_cycle = deipce_fsc_read16(dp,
                                       FSC_SCHED_TBL_BASE(sched->num,
                                                          oper_table_num) +
                                       FSC_SCHED_TBL_CYCLE_COUNT);
        last_cycle += last_cycle_inc;
        deipce_fsc_write16(dp, FSC_SCHED_TBL_BASE(sched->num, oper_table_num) +
                           FSC_SCHED_TBL_CYCLE_COUNT,
                           last_cycle);
        deipce_fsc_write16(dp, FSC_SCHED_TBL_BASE(sched->num, oper_table_num) +
                           FSC_SCHED_TBL_GEN,
                           table_gen);
    }

    return 0;
}

/**
 * Work function to handle delayed table change.
 * This is used when start time is too far in the future to be written
 * directly into FSC.
 * @param work Start work within scheduler.
 */
void deipce_fsc_start_work(struct work_struct *work)
{
    struct deipce_fsc_sched *sched =
        container_of(work, struct deipce_fsc_sched, start_work.work);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    int ret;

    dev_dbg(&dp->pdev->dev, "%s() scheduler %u check for start\n",
            __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    if (!sched->config_change)
        goto out;

    deipce_fsc_start_schedule(dp, sched);

out:
    mutex_unlock(&dp->lock);

    return;
}

/**
 * Switch to new schedule.
 * Starts schedule with current administrative parameters.
 * Schedule table rows must have been written already.
 * @param dp Device privates.
 * @param sched_num Scheduler number whose schedule to change.
 */
int deipce_fsc_exchange_schedule(struct deipce_fsc_dev_priv *dp,
                                 unsigned int sched_num)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    int ret;

    dev_dbg(&dp->pdev->dev, "%s() Switch scheduler %u to table %u\n",
            __func__, sched->num, sched->table_num);

    ret = deipce_fsc_start_schedule(dp, sched);

    return ret;
}

/**
 * Try to cancel schedule change.
 * @param dp Device privates.
 * @param sched_num Scheduler number whose schedule change to cancel.
 */
int deipce_fsc_cancel_schedule_exchange(struct deipce_fsc_dev_priv *dp,
                                        unsigned int sched_num)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    uint16_t table_gen;

    dev_dbg(&dp->pdev->dev, "%s() Cancel scheduler %u switch to table %u\n",
            __func__, sched->num, sched->table_num);

    cancel_delayed_work(&sched->start_work);

    table_gen = deipce_fsc_read16(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                         sched->table_num) +
                                  FSC_SCHED_TBL_GEN);
    table_gen &= ~FSC_SCHED_TBL_GEN_CAN_USE;
    deipce_fsc_write16(dp, FSC_SCHED_TBL_BASE(sched->num, sched->table_num) +
                       FSC_SCHED_TBL_GEN,
                       table_gen);

    table_gen = deipce_fsc_read16(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                         sched->table_num) +
                                  FSC_SCHED_TBL_GEN);
    if (table_gen & FSC_SCHED_TBL_GEN_IN_USE) {
        // Missed.
        return -EBUSY;
    }

    sched->config_change = false;

    return 0;
}

/**
 * Copy used rows from one table to the other.
 * @param dp Device privates.
 * @param sched_num Scheduler number.
 * @param src_table_num Source table number.
 * @param dst_table_num Destination table number.
 */
static int deipce_fsc_copy_schedule_table(struct deipce_fsc_dev_priv *dp,
                                          unsigned int sched_num,
                                          unsigned int src_table_num,
                                          unsigned int dst_table_num)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    struct deipce_fsc_table *dst_table = &sched->table[dst_table_num];
    const struct deipce_fsc_table *src_table = &sched->table[src_table_num];
    uint16_t cycles = 0;
    uint64_t outputs = 0;
    unsigned int row_num;
    int ret;

    for (row_num = 0; row_num < src_table->num_rows_used; row_num++) {
        ret = deipce_fsc_read_row(dp, sched_num, src_table_num, row_num,
                                  &cycles, &outputs);
        if (ret)
            return ret;

        ret = deipce_fsc_write_row(dp, sched_num, dst_table_num, row_num,
                                   cycles, outputs);
        if (ret)
            return ret;
    }

    dst_table->num_rows_used = src_table->num_rows_used;

    return 0;
}

/**
 * Check if schedule change has been completed.
 * Updates internal state information.
 * @param dp Device privates.
 * @param sched_num Scheduler number whose schedule change to cancel.
 */
int deipce_fsc_check_schedule(struct deipce_fsc_dev_priv *dp,
                              unsigned int sched_num)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    unsigned int old_table_num = sched->table_num;
    uint16_t table_gen;
    int ret;

    if (!sched->config_change)
        return 0;

    table_gen = deipce_fsc_read16(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                         sched->table_num) +
                                  FSC_SCHED_TBL_GEN);
    if (!(table_gen & FSC_SCHED_TBL_GEN_IN_USE))
        return 0;

    // New table has been taken into use.
    if (!sched->start_time_in_past) {
        struct timespec64 actual_start_time;

        actual_start_time.tv_sec =
            deipce_fsc_read16(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                     old_table_num) +
                              FSC_SCHED_TBL_START_TIME_SEC);
        actual_start_time.tv_sec &= FSC_SCHED_TBL_START_TIME_SEC_MASK;
        actual_start_time.tv_nsec =
            deipce_fsc_read32(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                     old_table_num) +
                              FSC_SCHED_TBL_START_TIME_NS);
        actual_start_time.tv_nsec &= FSC_SCHED_TBL_START_TIME_NS_MASK;
        dev_dbg(&dp->pdev->dev,
                "%s() Scheduler %u started %lli.%09li scheduled %lli.%09li "
                "truncated %lli.%09li\n",
                __func__, sched->num,
                (long long int)actual_start_time.tv_sec,
                actual_start_time.tv_nsec,
                (long long int)sched->start_time.tv_sec,
                sched->start_time.tv_nsec,
                (long long int)sched->start_time.tv_sec &
                FSC_SCHED_TBL_START_TIME_SEC_MASK,
                sched->start_time.tv_nsec);

        if (actual_start_time.tv_sec != (sched->start_time.tv_sec &
                                         FSC_SCHED_TBL_START_TIME_SEC_MASK) ||
            actual_start_time.tv_nsec != sched->start_time.tv_nsec)
            sched->start_time_in_past = true;
    }
    if (sched->start_time_in_past)
        sched->errors++;

    /*
     * Non-standard behavior: in case requested cycle time was longer than
     * supported and what has been taken into use, increase ConfigChangeError.
     * FLEXDE-668.
     */
    if (param->cycle_time_overflow)
        sched->errors++;

    if (++sched->table_num >= DEIPCE_FSC_MAX_TABLES)
        sched->table_num = 0;
    sched->config_change = false;

    table_gen = deipce_fsc_read16(dp, FSC_SCHED_TBL_BASE(sched->num,
                                                         sched->table_num) +
                                  FSC_SCHED_TBL_GEN);
    if ((table_gen & FSC_SCHED_TBL_GEN_IN_USE) ||
        (table_gen & FSC_SCHED_TBL_GEN_CAN_USE)) {
        dev_warn(&dp->pdev->dev, "Switch to scheduler %u table %u failed\n",
                 sched_num, sched->table_num);
        return -EINVAL;
    }

    sched->oper.param = sched->admin.param;
    ret = deipce_fsc_copy_schedule_table(dp, sched_num,
                                         old_table_num, sched->table_num);
    if (ret)
        return ret;

    dev_dbg(&dp->pdev->dev, "%s() Scheduler %u switched to table %u\n",
            __func__, sched->num, old_table_num);

    return 0;
}

/**
 * Check if schedule change has been completed and prepare for changes.
 * Updates internal state information.
 * If configuration change is still pending, it will be cancelled
 * and deipce_fsc_exchange_schedule must be called after changing parameters.
 * @param dp Device privates.
 * @param sched_num Scheduler number whose schedule is being updated.
 * @param Place for configuration still did not yet complete flag.
 */
int deipce_fsc_update_schedule(struct deipce_fsc_dev_priv *dp,
                               unsigned int sched_num, bool *pending)
{
    struct deipce_fsc_sched *sched = &dp->sched[sched_num];
    int ret;

    ret = deipce_fsc_check_schedule(dp, sched_num);
    if (ret)
        return ret;

    *pending = sched->config_change;

    if (*pending) {
        ret = deipce_fsc_cancel_schedule_exchange(dp, sched_num);
        if (ret)
            return ret;
    }

    return 0;
}

/**
 * Record new link mode to be used for FSC downcounter when changing config.
 * @param fsc FSC device privates.
 * @param sched_num Scheduler number connected to port.
 * @param netdev Switch port net device of the port connected to scheduler.
 */
void deipce_fsc_update_link(struct deipce_fsc_dev_priv *fsc,
                            unsigned int sched_num,
                            struct net_device *netdev)
{
    struct deipce_fsc_sched *sched = &fsc->sched[sched_num];
    struct deipce_netdev_priv *np = netdev_priv(netdev);

    dev_dbg(&fsc->pdev->dev, "%s() Update scheduler %u link mode %i\n",
            __func__, sched->num, np->link_mode);

    mutex_lock(&fsc->lock);

    sched->link_mode = np->link_mode;

    mutex_unlock(&fsc->lock);

    return;
}

