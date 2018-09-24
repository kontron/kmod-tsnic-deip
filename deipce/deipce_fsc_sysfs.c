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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/io.h>

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_ethtool.h"
#include "deipce_time.h"
#include "deipce_fsc_types.h"
#include "deipce_fsc_if.h"
#include "deipce_fsc_hw.h"
#include "deipce_sysfs_common.h"
#include "deipce_fsc_sysfs.h"

#define to_deipce_fsc_dev_priv(dev) (to_deipce_port_priv(dev)->sched.fsc)
#define to_deipce_fsc_sched(dev) \
    (&to_deipce_fsc_dev_priv(dev)->sched[to_deipce_port_priv(dev)->sched.num])

/**
 * Get access to current table with administrative values.
 * @param sched Scheduler instance.
 */
static inline struct deipce_fsc_table *deipce_fsc_admin_table(
        struct deipce_fsc_sched *sched)
{
    return &sched->table[sched->table_num];
}

/**
 * Get access to current table with operational values.
 * @param sched Scheduler instance.
 */
static inline struct deipce_fsc_table *deipce_fsc_oper_table(
        struct deipce_fsc_sched *sched)
{
    return &sched->table[(sched->table_num + 1) % DEIPCE_FSC_MAX_TABLES];
}

/**
 * Convert string with nanosecond precision timestamp to struct timespec64.
 * @param buf String to convert, in format "%lli.%09li",
 * with optional trailing newline.
 * @param time Place for converted time.
 */
static int deipce_fsc_strtotimespec64(const char *buf, struct timespec64 *time)
{
    char str[48];
    char *dot;
    size_t len = strlen(buf);
    long long int sec = 0;
    int ret;

    if (len > sizeof(str))
        return -EINVAL;

    strcpy(str, buf);
    dot = strchr(str, '.');

    // "sec.nsec\n"
    if (dot)
        *dot = '\0';
    ret = kstrtoll(str, 10, &sec);
    if (ret)
        return ret;
    if (sec < 0)
        return -EINVAL;

    time->tv_sec = sec;

    // Treat nanoseconds part as optional.
    if (dot) {
        // Add missing trailing zero digits for correct scaling.
        size_t digits = strspn(dot + 1, "0123456789");

        if (digits < 9) {
            // Number of trailing zeros to add.
            size_t add = 9 - digits;
            // Number of characters after digits (including NUL) to move.
            size_t move = strlen(dot + 1) - digits + 1;

            // Move trailing non-digits (newline) by number of zeros to add.
            while (move > 0) {
                dot[digits + add + move] = dot[digits + move];
                move--;
            }

            // Add trailing zero digits.
            while (add-- > 0)
                dot[1 + digits + add] = '0';
        }

        // Ensure correct interpretation of leading zeros.
        *dot = '1';
        ret = kstrtol(dot, 10, &time->tv_nsec);
        if (ret)
            return ret;
        time->tv_nsec -= NSEC_PER_SEC;
        if (time->tv_nsec < 0 || time->tv_nsec >= NSEC_PER_SEC)
            return -EINVAL;
    }
    else {
        time->tv_nsec = 0;
    }

    return 0;
}

// GateEnabled

static ssize_t deipce_fsc_sysfs_gate_enabled_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    uint16_t ctrl = 0;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    ctrl = deipce_fsc_read16(dp, FSC_SCHED_BASE(sched->num) +
                             FSC_SCHED_EME_DIS_CTRL);
    ret = sprintf(buf, "%u\n", !(ctrl & FSC_SCHED_EME_DIS_CTRL_ENABLE));

    return ret;
}

static ssize_t deipce_fsc_sysfs_gate_enabled_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    uint16_t ctrl = 0;
    bool enable = false;
    bool pending = false;
    int ret = kstrtobool(buf, &enable);

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    if (ret)
        return ret;

    mutex_lock(&dp->lock);

    ctrl = deipce_fsc_read16(dp, FSC_SCHED_BASE(sched->num) +
                             FSC_SCHED_EME_DIS_CTRL);

    if (enable == !(ctrl & FSC_SCHED_EME_DIS_CTRL_ENABLE))
        goto out;

    if (enable) {
        // Leave emergency disable.
        ctrl &= ~FSC_SCHED_EME_DIS_CTRL_ENABLE;
        ret = deipce_fsc_update_schedule(dp, sched->num, &pending);
        if (ret)
            goto out;
    }
    else {
        // Enter emergency disable.
        ctrl |= FSC_SCHED_EME_DIS_CTRL_ENABLE;
    }

    deipce_fsc_write16(dp, FSC_SCHED_BASE(sched->num) +
                       FSC_SCHED_EME_DIS_CTRL, ctrl);

    if (pending)
        ret = deipce_fsc_exchange_schedule(dp, sched->num);

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(GateEnabled,
                      &deipce_fsc_sysfs_gate_enabled_show,
                      &deipce_fsc_sysfs_gate_enabled_store);

// AdminGateStates

static ssize_t deipce_fsc_sysfs_admin_gate_states_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    uint64_t outputs = 0;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    outputs = deipce_fsc_read_outputs(dp, FSC_SCHED_BASE(sched->num) +
                                      FSC_SCHED_EME_DIS_STAT0);
    ret = sprintf(buf, "0x%llx\n", outputs);

    return ret;
}

static ssize_t deipce_fsc_sysfs_admin_gate_states_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    uint64_t outputs = 0;
    int ret = kstrtou64(buf, 0, &outputs);

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    if (ret)
        return ret;

    if (outputs & ~((1ull << DEIPCE_MAX_PRIO_QUEUES) - 1))
        return -EINVAL;

    outputs &= dp->output_mask & ~dp->hold_output_mask;

    mutex_lock(&dp->lock);

    deipce_fsc_write_outputs(dp, FSC_SCHED_BASE(sched->num) +
                             FSC_SCHED_EME_DIS_STAT0, outputs);

    mutex_unlock(&dp->lock);

    return count;
}

static DEIPCE_ATTR_RW(AdminGateStates,
                      &deipce_fsc_sysfs_admin_gate_states_show,
                      &deipce_fsc_sysfs_admin_gate_states_store);

// AdminControlListLength

static ssize_t deipce_fsc_sysfs_admin_control_list_length_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_table *table;
    unsigned int rows;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    table = deipce_fsc_admin_table(sched);
    rows = table->num_rows_used;

out:
    mutex_unlock(&dp->lock);

    if (ret == 0)
        ret = sprintf(buf, "%u\n", rows);

    return ret;
}

static ssize_t deipce_fsc_sysfs_admin_control_list_length_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    bool pending;
    struct deipce_fsc_table *table;
    unsigned int rows = 0;
    unsigned int row;
    int ret = kstrtouint(buf, 0, &rows);

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    if (ret)
        return ret;

    if (rows > dp->num_rows)
        return -EINVAL;

    mutex_lock(&dp->lock);

    ret = deipce_fsc_update_schedule(dp, sched->num, &pending);
    if (ret)
        goto out;

    table = deipce_fsc_admin_table(sched);

    // Clear excess rows.
    for (row = rows; row < dp->num_rows; row++) {
        ret = deipce_fsc_write_row(dp, sched->num, table->num, row, 0, 0);
        if (ret)
            goto out;
    }

    table->num_rows_used = rows;

    if (pending)
        ret = deipce_fsc_exchange_schedule(dp, sched->num);

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(AdminControlListLength,
                      &deipce_fsc_sysfs_admin_control_list_length_show,
                      &deipce_fsc_sysfs_admin_control_list_length_store);

// OperControlListLength

static ssize_t deipce_fsc_sysfs_oper_control_list_length_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_table *table;
    unsigned int rows;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    table = deipce_fsc_oper_table(sched);
    rows = table->num_rows_used;

out:
    mutex_unlock(&dp->lock);

    if (ret == 0)
        ret = sprintf(buf, "%u\n", rows);

    return ret;
}

static DEIPCE_ATTR_RO(OperControlListLength,
                      &deipce_fsc_sysfs_oper_control_list_length_show);

// AdminControlList

/**
 * Control list entry structure in user space interface.
 */
struct deipce_fsc_sysfs_control_entry {
    uint32_t interval;          ///< in nanoseconds
    uint8_t op;                 ///< operation code
    uint8_t gates;              ///< gate state values
    uint16_t padding;           ///< for alignment
} __attribute__((packed));

/// Number of bytes in an entry: operation, outputs, time in nanoseconds
#define DEIPCE_FSC_SYSFS_ROW_LEN sizeof(struct deipce_fsc_sysfs_control_entry)

/**
 * Check validity of control list read/write operation.
 * We only support reading/writing integral number of whole entries at once.
 * @param dp FSC device privates.
 * @param ofs Offset of read operation.
 * @param count Size of read operation.
 * @param write True for write, false for read.
 * @param first_row Place for first row number.
 */
static int deipce_fsc_sysfs_validate_control_list_op(
        struct deipce_fsc_dev_priv *dp,
        loff_t ofs,
        size_t count,
        bool write,
        unsigned int *first_row)
{
    if (ofs >= dp->num_rows * DEIPCE_FSC_SYSFS_ROW_LEN)
        return -EINVAL;
    if ((unsigned int)ofs % DEIPCE_FSC_SYSFS_ROW_LEN)
        return -EINVAL;
    if (count < DEIPCE_FSC_SYSFS_ROW_LEN)
        return -EINVAL;
    if (write) {
        if (ofs + count > dp->num_rows * DEIPCE_FSC_SYSFS_ROW_LEN)
            return -EINVAL;
        if (((unsigned int)count % DEIPCE_FSC_SYSFS_ROW_LEN))
            return -EINVAL;
    }

    *first_row = (unsigned int)ofs/DEIPCE_FSC_SYSFS_ROW_LEN;

    return 0;
}

/**
 * Determine hold signal value from previous FSC row.
 * @param dp Device privates.
 * @param sched Scheduler instance.
 * @param table_num Table number.
 * @param row_num Row number for which to get the hold signal value,
 * from its predecessor.
 * @param hold Place for hold signal value.
 */
static int deipce_fsc_sysfs_get_prev_hold(struct deipce_fsc_dev_priv *dp,
                                          struct deipce_fsc_sched *sched,
                                          unsigned int table_num,
                                          unsigned int row_num,
                                          bool *hold)
{
    uint16_t cycles;
    uint64_t outputs;
    int ret;

    if (row_num == 0) {
        // Wrapping hold is checked and adjusted for at config change time.
        *hold = false;
    }
    else {
        ret = deipce_fsc_read_row(dp, sched->num, table_num, row_num - 1,
                                  &cycles, &outputs);
        if (ret)
            return ret;

        *hold = outputs & sched->hold_output_mask;
    }

    return 0;
}

/**
 * Encode single row into sysfs format.
 * @param dp FSC device privates.
 * @param sched Scheduler instance.
 * @param cycles Row time in number of clock cycles.
 * @param outputs Bitmask of outputs.
 * @param hold Pointer to previous hold signal value, which will be updated.
 * @param buf Pointer to buffer to write entry to,
 * with room for at least DEIPCE_FSC_SYSFS_ROW_LEN bytes.
 */
static int deipce_fsc_sysfs_encode_row(
        struct deipce_fsc_dev_priv *dp,
        struct deipce_fsc_sched *sched,
        uint16_t cycles,
        uint64_t outputs,
        bool *hold,
        struct deipce_fsc_sysfs_control_entry *user_entry)
{
    user_entry->interval = cycles * dp->clock_cycle_length;
    if (!*hold && (outputs & sched->hold_output_mask))
        user_entry->op = DEIPCE_FSC_OP_HOLD_MAC;
    else if (*hold && !(outputs & sched->hold_output_mask))
        user_entry->op = DEIPCE_FSC_OP_RELEASE_MAC;
    else
        user_entry->op = DEIPCE_FSC_OP_SET_GATES;
    user_entry->gates = (uint8_t)outputs;
    user_entry->padding = 0;

    *hold = outputs & sched->hold_output_mask;

    return 0;
}

/**
 * Decode single row from sysfs format.
 * @param dp FSC device privates.
 * @param sched Scheduler instance.
 * @param cycles Place for row time in number of clock cycles.
 * @param outputs Place for bitmask of outputs.
 * @param hold Pointer to previous hold signal value, which will be updated.
 * @param buf Pointer to buffer to read entry from,
 * with at least DEIPCE_FSC_SYSFS_ROW_LEN bytes.
 * @return Zero on success, 1 when accepted but modified, or negative on error.
 */
static int deipce_fsc_sysfs_decode_row(
        struct deipce_fsc_dev_priv *dp,
        struct deipce_fsc_sched *sched,
        uint16_t *cycles,
        uint64_t *outputs,
        bool *hold,
        const struct deipce_fsc_sysfs_control_entry *user_entry)
{
    uint32_t cycles_tmp;
    int ret = 0;

    *outputs = user_entry->gates;

    switch (user_entry->op) {
    case DEIPCE_FSC_OP_SET_GATES:
        if (*hold)
            *outputs |= sched->hold_output_mask;
        break;
    case DEIPCE_FSC_OP_HOLD_MAC:
        *outputs |= sched->hold_output_mask;
        break;
    case DEIPCE_FSC_OP_RELEASE_MAC:
        break;
    default:
        /*
         * Non-standard behavior: interpret unknown operation as
         * Set-Gate-States, all gates open. FLEXDE-819.
         */
        *outputs = dp->output_mask & ~dp->hold_output_mask;
        ret = 1;
    }

    // Record actual hold signal value.
    *hold = *outputs & sched->hold_output_mask;

    cycles_tmp = user_entry->interval / dp->clock_cycle_length;

    /*
     * Non-standard behavior: maximum supported value is used for bigger values
     * than what can be supported. The same for too small values. FLEXDE-669.
     */
    if (cycles_tmp < FSC_ROW_DATA_CYCLES_MIN) {
        *cycles = FSC_ROW_DATA_CYCLES_MIN;
        ret = 1;
    }
    else if (cycles_tmp > FSC_ROW_DATA_CYCLES_MAX) {
        *cycles = FSC_ROW_DATA_CYCLES_MAX;
        ret = 1;
    }
    else {
        *cycles = (uint16_t)cycles_tmp;
    }

    return ret;
}

/**
 * Read control list entries from scheduler rows.
 * @param dp FSC device privates.
 * @param sched Scheduler to read from.
 * @param table Table to read from.
 * @param row First row to read.
 * @param buf Place for writing entries.
 * @param count Number of bytes room in buf.
 * @return Number of bytes written to buf, or negative error code.
 */
static ssize_t deipce_fsc_sysfs_read_control_list(
        struct deipce_fsc_dev_priv *dp,
        struct deipce_fsc_sched *sched,
        struct deipce_fsc_table *table,
        unsigned int row,
        struct deipce_fsc_sysfs_control_entry *user_entry,
        size_t count)
{
    bool hold;
    size_t pos = 0;
    uint16_t cycles;
    uint64_t outputs;
    int ret = 0;

    ret = deipce_fsc_sysfs_get_prev_hold(dp, sched, table->num, row, &hold);
    if (ret)
        return ret;

    for (;
         row < table->num_rows_used && pos + DEIPCE_FSC_SYSFS_ROW_LEN <= count;
         row++) {
        ret = deipce_fsc_read_row(dp, sched->num, table->num, row,
                                  &cycles, &outputs);
        if (ret)
            break;

        ret = deipce_fsc_sysfs_encode_row(dp, sched, cycles, outputs, &hold,
                                          user_entry);
        if (ret)
            break;

        pos += DEIPCE_FSC_SYSFS_ROW_LEN;
        user_entry++;
    }

    if (ret)
        return ret;

    return pos;
}

static ssize_t deipce_fsc_sysfs_admin_control_list_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_table *table;
    struct deipce_fsc_sysfs_control_entry *user_entry =
        (struct deipce_fsc_sysfs_control_entry *)buf;
    unsigned int row = 0;
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %p ofs %llu count %zu\n",
               __func__, sched->num, buf, ofs, count);

    ret = deipce_fsc_sysfs_validate_control_list_op(dp, ofs, count, false,
                                                    &row);
    if (ret)
        return ret;

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    table = deipce_fsc_admin_table(sched);
    ret = deipce_fsc_sysfs_read_control_list(dp, sched, table, row,
                                             user_entry, count);

out:
    mutex_unlock(&dp->lock);

    return ret;
}

static ssize_t deipce_fsc_sysfs_admin_control_list_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_table *table;
    bool pending;
    unsigned int first_row;
    unsigned int row;
    const struct deipce_fsc_sysfs_control_entry *user_entry =
        (const struct deipce_fsc_sysfs_control_entry *)buf;
    size_t pos = 0;
    bool hold = false;
    uint16_t cycles;
    uint64_t outputs;
    bool adapted = false;
    int ret = 0;

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %p ofs %llu count %zu\n",
               __func__, sched->num, buf, ofs, count);

    ret = deipce_fsc_sysfs_validate_control_list_op(dp, ofs, count, true,
                                                    &first_row);
    if (ret)
        return ret;

    // Check all values first.
    for (row = first_row;
         row < dp->num_rows && pos + DEIPCE_FSC_SYSFS_ROW_LEN <= count;
         row++) {
        ret = deipce_fsc_sysfs_decode_row(dp, sched, &cycles, &outputs, &hold,
                                          user_entry);
        if (ret < 0)
            return ret;

        pos += DEIPCE_FSC_SYSFS_ROW_LEN;
        user_entry++;
    }

    mutex_lock(&dp->lock);

    ret = deipce_fsc_update_schedule(dp, sched->num, &pending);
    if (ret)
        goto out;

    table = deipce_fsc_admin_table(sched);
    ret = deipce_fsc_sysfs_get_prev_hold(dp, sched, table->num, first_row,
                                         &hold);
    if (ret)
        goto out;

    pos = 0;
    user_entry = (const struct deipce_fsc_sysfs_control_entry *)buf;
    for (row = first_row;
         row < dp->num_rows && pos + DEIPCE_FSC_SYSFS_ROW_LEN <= count;
         row++) {
        ret = deipce_fsc_sysfs_decode_row(dp, sched, &cycles, &outputs, &hold,
                                          user_entry);
        if (ret < 0)
            goto out;
        else if (ret > 0)
            adapted = true;

        ret = deipce_fsc_write_row(dp, sched->num, table->num, row,
                                   cycles, outputs);
        if (ret)
            goto out;

        // Record original gate operations.
        table->gate_op[row] = user_entry->op;

        pos += DEIPCE_FSC_SYSFS_ROW_LEN;
        user_entry++;
    }

    if (row >= dp->num_rows) {
        ret = -EINVAL;
        goto out;
    }

    // Clear excess rows.
    for (; row < dp->num_rows; row++) {
        ret = deipce_fsc_write_row(dp, sched->num, table->num, row, 0, 0);
        if (ret)
            goto out;
    }

    if (pending)
        ret = deipce_fsc_exchange_schedule(dp, sched->num);

out:
    /*
     * Non-standard behavior: ConfigChangeError is incremented when value
     * is accepted but modified. FLEXDE-669.
     */
    if (adapted)
        sched->errors++;

    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return pos;
}

static DEIPCE_BIN_ATTR_RW(AdminControlList,
                          &deipce_fsc_sysfs_admin_control_list_read,
                          &deipce_fsc_sysfs_admin_control_list_write,
                          DEIPCE_FSC_MAX_ROWS * (1 + 1 + sizeof(uint32_t)));

// OperControlList

static ssize_t deipce_fsc_sysfs_oper_control_list_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_table *table;
    struct deipce_fsc_sysfs_control_entry *user_entry =
        (struct deipce_fsc_sysfs_control_entry *)buf;
    unsigned int row;
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %p ofs %llu count %zu\n",
               __func__, sched->num, buf, ofs, count);

    ret = deipce_fsc_sysfs_validate_control_list_op(dp, ofs, count, false,
                                                    &row);
    if (ret)
        return ret;

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    table = deipce_fsc_oper_table(sched);
    ret = deipce_fsc_sysfs_read_control_list(dp, sched, table, row,
                                             user_entry, count);

out:
    mutex_unlock(&dp->lock);

    return ret;
}

static DEIPCE_BIN_ATTR_RO(OperControlList,
                          &deipce_fsc_sysfs_oper_control_list_read,
                          DEIPCE_FSC_MAX_ROWS * (1 + 1 + sizeof(uint32_t)));

// AdminCycleTime

static ssize_t deipce_fsc_sysfs_admin_cycle_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    struct deipce_fsc_fraction cycle_time;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    cycle_time = param->cycle_time;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u/%u\n",
                  cycle_time.numerator, cycle_time.denominator);

    return ret;
}

static ssize_t deipce_fsc_sysfs_admin_cycle_time_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    bool pending;
    struct deipce_fsc_fraction cycle_time;
    int ret = -ERANGE;

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    ret = sscanf(buf, "%u/%u", &cycle_time.numerator, &cycle_time.denominator);
    if (ret != 2)
        return -EINVAL;
    if (cycle_time.denominator == 0)
        return -EINVAL;
    if (cycle_time.numerator == 0)
        return -EINVAL;

    mutex_lock(&dp->lock);

    ret = deipce_fsc_update_schedule(dp, sched->num, &pending);
    if (ret)
        goto out;

    param->cycle_time = cycle_time;

    if (pending)
        ret = deipce_fsc_exchange_schedule(dp, sched->num);

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(AdminCycleTime,
                      &deipce_fsc_sysfs_admin_cycle_time_show,
                      &deipce_fsc_sysfs_admin_cycle_time_store);

// OperCycleTime

static ssize_t deipce_fsc_sysfs_oper_cycle_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->oper.param;
    struct deipce_fsc_fraction cycle_time;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    deipce_fsc_effective_cycle_time(&cycle_time, param);

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u/%u\n",
                  cycle_time.numerator, cycle_time.denominator);

    return ret;
}

static DEIPCE_ATTR_RO(OperCycleTime,
                      &deipce_fsc_sysfs_oper_cycle_time_show);

// AdminCycleTimeExtension

static ssize_t deipce_fsc_sysfs_admin_cycle_time_ext_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    uint32_t nsec;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    nsec = param->cycle_time_ext;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u\n", nsec);

    return ret;
}

static ssize_t deipce_fsc_sysfs_admin_cycle_time_ext_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    bool pending;
    uint32_t nsec = 0;
    int ret = kstrtou32(buf, 0, &nsec);

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    if (ret)
        return -EINVAL;

    mutex_lock(&dp->lock);

    ret = deipce_fsc_update_schedule(dp, sched->num, &pending);
    if (ret)
        goto out;

    param->cycle_time_ext = nsec;

    if (pending)
        ret = deipce_fsc_exchange_schedule(dp, sched->num);

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(AdminCycleTimeExtension,
                      &deipce_fsc_sysfs_admin_cycle_time_ext_show,
                      &deipce_fsc_sysfs_admin_cycle_time_ext_store);

// OperCycleTimeExtension

static ssize_t deipce_fsc_sysfs_oper_cycle_time_ext_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->oper.param;
    uint32_t nsec;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    nsec = param->cycle_time_ext;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u\n", nsec);

    return ret;
}

static DEIPCE_ATTR_RO(OperCycleTimeExtension,
                      &deipce_fsc_sysfs_oper_cycle_time_ext_show);

// AdminBaseTime

static ssize_t deipce_fsc_sysfs_admin_base_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    struct timespec64 base_time;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    base_time = param->base_time;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%lli.%09li\n",
                  (long long int)base_time.tv_sec, base_time.tv_nsec);

    return ret;
}

static ssize_t deipce_fsc_sysfs_admin_base_time_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->admin.param;
    bool pending;
    struct timespec64 base_time = { .tv_sec = 0 };
    int ret = -EINVAL;

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    ret = deipce_fsc_strtotimespec64(buf, &base_time);
    if (ret)
        return ret;

    mutex_lock(&dp->lock);

    ret = deipce_fsc_update_schedule(dp, sched->num, &pending);
    if (ret)
        goto out;

    param->base_time = base_time;

    if (pending)
        ret = deipce_fsc_exchange_schedule(dp, sched->num);

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(AdminBaseTime,
                      &deipce_fsc_sysfs_admin_base_time_show,
                      &deipce_fsc_sysfs_admin_base_time_store);

// OperBaseTime

static ssize_t deipce_fsc_sysfs_oper_base_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct deipce_fsc_sched_param *param = &sched->oper.param;
    struct timespec64 base_time;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    base_time = param->base_time;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%lli.%09li\n",
                  (long long int)base_time.tv_sec, base_time.tv_nsec);

    return ret;
}

static DEIPCE_ATTR_RO(OperBaseTime,
                      &deipce_fsc_sysfs_oper_base_time_show);

// ConfigChange

static ssize_t deipce_fsc_sysfs_config_change_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    bool value = false;
    uint16_t ctrl;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    // Internal flag is kept set until table change is complete.
    if (sched->config_change) {
        ctrl = deipce_fsc_read16(dp, FSC_SCHED_BASE(sched->num) +
                                 FSC_SCHED_EME_DIS_CTRL);
        if (ctrl & FSC_SCHED_EME_DIS_CTRL_ENABLE)
            value = sched->config_change;
    }

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u\n", value);

    return ret;
}

static ssize_t deipce_fsc_sysfs_config_change_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    bool value = false;
    int ret = kstrtobool(buf, &value);

    netdev_dbg(to_net_dev(dev), "%s() sched %u buf %s count %zu\n",
               __func__, sched->num, buf, count);

    if (ret)
        return ret;

    // 802.1Qbv 8.6.9.4.7 ConfigChange can only be set true by user.
    if (!value)
        return -EINVAL;

    mutex_lock(&dp->lock);

    // Internal flag is kept set until table change is complete.
    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    if (!sched->config_change && value) {
        // 0 -> 1
        ret = deipce_fsc_exchange_schedule(dp, sched->num);
    }

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_ATTR_RW(ConfigChange,
                      &deipce_fsc_sysfs_config_change_show,
                      &deipce_fsc_sysfs_config_change_store);

// ConfigChangeError

static ssize_t deipce_fsc_sysfs_config_change_error_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    uint64_t errors;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    errors = sched->errors;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%llu\n", errors);

    return ret;
}

static DEIPCE_ATTR_RO(ConfigChangeError,
                      &deipce_fsc_sysfs_config_change_error_show);

// ConfigPending

static ssize_t deipce_fsc_sysfs_config_pending_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    bool value = false;
    uint16_t ctrl;
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    if (sched->config_change) {
        ctrl = deipce_fsc_read16(dp, FSC_SCHED_BASE(sched->num) +
                                 FSC_SCHED_EME_DIS_CTRL);
        if (!(ctrl & FSC_SCHED_EME_DIS_CTRL_ENABLE))
            value = true;
    }

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u\n", value);

    return ret;
}

static DEIPCE_ATTR_RO(ConfigPending,
                      &deipce_fsc_sysfs_config_pending_show);

// ConfigChangeTime

static ssize_t deipce_fsc_sysfs_config_change_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct timespec64 change_time = { .tv_sec = 0 };
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_fsc_check_schedule(dp, sched->num);
    if (ret)
        goto out;

    change_time = sched->change_time;

out:
    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%lli.%09li\n",
                  (long long int)change_time.tv_sec, change_time.tv_nsec);

    return ret;
}

static DEIPCE_ATTR_RO(ConfigChangeTime,
                      &deipce_fsc_sysfs_config_change_time_show);

// SupportedListMax

static ssize_t deipce_fsc_sysfs_supported_list_max_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    int ret;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    // One row is used internally.
    ret = sprintf(buf, "%u\n", dp->num_rows - 1);

    return ret;
}

static DEIPCE_ATTR_RO(SupportedListMax,
                      &deipce_fsc_sysfs_supported_list_max_show);

// queueMaxSDUTable

static ssize_t deipce_fsc_sysfs_queue_max_sdu_table_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    uint32_t data[DEIPCE_MAX_PRIO_QUEUES] = { 0 };
    unsigned int i;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    mutex_lock(&pp->port_reg_lock);

    for (i = 0; i < dp->features.prio_queues; i++) {
        data[i] = deipce_read_port_reg(pp, PORT_REG_FRAMESIZE(i));
    }

    mutex_unlock(&pp->port_reg_lock);

    memcpy(buf, (char *)data + ofs, count);

    return count;
}

static ssize_t deipce_fsc_sysfs_queue_max_sdu_table_write(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    uint32_t data[DEIPCE_MAX_PRIO_QUEUES] = { 0 };
    unsigned int i;
    int ret = -EINVAL;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    mutex_lock(&pp->port_reg_lock);

    for (i = 0; i < dp->features.prio_queues; i++) {
        data[i] = deipce_read_port_reg(pp, PORT_REG_FRAMESIZE(i));
    }

    memcpy((char *)data + ofs, buf, count);

    for (i = 0; i < dp->features.prio_queues; i++) {
        if (data[i] == 0)
            data[i] = PORT_FRAMESIZE_MAX;
        else if (data[i] < PORT_FRAMESIZE_MIN)
            goto out;
        else if (data[i] > PORT_FRAMESIZE_MAX)
            goto out;
    }

    for (i = 0; i < dp->features.prio_queues; i++) {
        deipce_write_port_reg(pp, PORT_REG_FRAMESIZE(i), data[i]);
    }

    ret = 0;

out:
    mutex_unlock(&pp->port_reg_lock);

    if (ret)
        return ret;

    return count;
}

static DEIPCE_BIN_ATTR_RW(queueMaxSDUTable,
                          &deipce_fsc_sysfs_queue_max_sdu_table_read,
                          &deipce_fsc_sysfs_queue_max_sdu_table_write,
                          DEIPCE_MAX_PRIO_QUEUES * sizeof(uint32_t));

// transmissionOverrunTable

static ssize_t deipce_fsc_sysfs_transmission_overrun_table_read(
        struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t ofs, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;
    uint64_t data[DEIPCE_MAX_PRIO_QUEUES] = { 0 };
    unsigned int i;

    netdev_dbg(to_net_dev(dev), "%s() buf %p ofs %llu count %zu\n",
               __func__, buf, ofs, count);

    mutex_lock(&pp->stats_lock);

    deipce_update_port_stats(pp);

    for (i = 0; i < dp->features.prio_queues; i++) {
        // Separate statistics for each queue is not available.
        data[i] = pp->stats[FRS_CNT_TX_OVERRUN];
    }

    mutex_unlock(&pp->stats_lock);

    memcpy(buf, (char *)data + ofs, count);

    return count;
}

static DEIPCE_BIN_ATTR_RO(transmissionOverrunTable,
                          &deipce_fsc_sysfs_transmission_overrun_table_read,
                          DEIPCE_MAX_PRIO_QUEUES * sizeof(uint64_t));

#ifdef CONFIG_PTP_1588_CLOCK

// CurrentTime

static ssize_t deipce_fsc_sysfs_current_time_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    struct timespec64 cur_time = { .tv_sec = 0 };
    int ret = -EOPNOTSUPP;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_time_get_time(dp->time, DEIPCE_TIME_SEL_WRK, &cur_time);

    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%lli.%09li\n",
                  (long long int)cur_time.tv_sec, cur_time.tv_nsec);

    return ret;
}

static DEIPCE_ATTR_RO(CurrentTime,
                      &deipce_fsc_sysfs_current_time_show);

// TickGranularity

static ssize_t deipce_fsc_sysfs_tick_granularity_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_fsc_sched *sched = to_deipce_fsc_sched(dev);
    struct deipce_fsc_dev_priv *dp =
        container_of(sched, struct deipce_fsc_dev_priv, sched[sched->num]);
    uint32_t granularity = 0;
    int ret = -EOPNOTSUPP;

    netdev_dbg(to_net_dev(dev), "%s() sched %u\n", __func__, sched->num);

    mutex_lock(&dp->lock);

    ret = deipce_time_get_granularity(dp->time, DEIPCE_TIME_SEL_WRK,
                                      &granularity);

    mutex_unlock(&dp->lock);

    if (ret)
        return ret;

    ret = sprintf(buf, "%u\n", granularity);

    return ret;
}

static DEIPCE_ATTR_RO(TickGranularity,
                      &deipce_fsc_sysfs_tick_granularity_show);

#endif

static const struct attribute_group *deipce_fsc_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "ieee8021ST",
        .attrs = (struct attribute*[]){
            &dev_attr_GateEnabled.attr,
            &dev_attr_AdminGateStates.attr,
            &dev_attr_AdminControlListLength.attr,
            &dev_attr_OperControlListLength.attr,
            &dev_attr_AdminCycleTime.attr,
            &dev_attr_OperCycleTime.attr,
            &dev_attr_AdminCycleTimeExtension.attr,
            &dev_attr_OperCycleTimeExtension.attr,
            &dev_attr_AdminBaseTime.attr,
            &dev_attr_OperBaseTime.attr,
            &dev_attr_ConfigChange.attr,
            &dev_attr_ConfigChangeError.attr,
            &dev_attr_ConfigPending.attr,
            &dev_attr_ConfigChangeTime.attr,
            &dev_attr_SupportedListMax.attr,
#ifdef CONFIG_PTP_1588_CLOCK
            &dev_attr_CurrentTime.attr,
            &dev_attr_TickGranularity.attr,
#endif
            NULL,
        },
        .bin_attrs = (struct bin_attribute*[]){
            &bin_attr_queueMaxSDUTable,
            &bin_attr_transmissionOverrunTable,
            &bin_attr_AdminControlList,
            &bin_attr_OperControlList,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize device sysfs files.
 * @param dp Device privates.
 */
int deipce_fsc_sysfs_dev_init(struct deipce_fsc_dev_priv *fsc,
                              unsigned int sched_num,
                              struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    struct deipce_fsc_sched *sched = &fsc->sched[sched_num];
    int ret = 0;

    netdev_dbg(pp->netdev, "%s() Init sysfs for FSC %s scheduler %u\n",
               __func__, dev_name(&fsc->pdev->dev), sched_num);

    if (dp->features.preempt_ports & (1u << pp->port_num))
        sched->hold_output_mask = fsc->hold_output_mask;

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_fsc_sysfs_attr_groups);
    if (ret)
        return ret;

    return 0;
}

/**
 * Cleanup device sysfs files.
 * @param dp Device privates.
 */
void deipce_fsc_sysfs_dev_cleanup(struct deipce_fsc_dev_priv *fsc,
                                  unsigned int sched_num,
                                  struct deipce_port_priv *pp)
{
    netdev_dbg(pp->netdev, "%s() Cleanup sysfs for FSC %s scheduler %u\n",
               __func__, dev_name(&fsc->pdev->dev), sched_num);

    sysfs_remove_groups(&pp->netdev->dev.kobj, deipce_fsc_sysfs_attr_groups);

    return;
}

