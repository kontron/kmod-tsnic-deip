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

#ifndef DEIPCE_FSC_HW_H
#define DEIPCE_FSC_HW_H

#include "deipce_fsc_types.h"

int deipce_fsc_read_row(struct deipce_fsc_dev_priv *dp, unsigned int sched_num,
                        unsigned int table_num, unsigned int row_num,
                        uint16_t *cycles, uint64_t *outputs);
int deipce_fsc_write_row(struct deipce_fsc_dev_priv *dp, unsigned int sched_num,
                         unsigned int table_num, unsigned int row_num,
                         uint16_t cycles, uint64_t outputs);
int deipce_fsc_clear_rows(struct deipce_fsc_dev_priv *dp,
                          unsigned int sched_num, unsigned int table_num);

void deipce_fsc_split_fraction(const struct deipce_fsc_fraction *fract,
                               struct deipce_fsc_time *time);
void deipce_fsc_effective_cycle_time(
        struct deipce_fsc_fraction *fract,
        const struct deipce_fsc_sched_param *param);
int deipce_fsc_exchange_schedule(struct deipce_fsc_dev_priv *dp,
                                 unsigned int sched_num);
int deipce_fsc_cancel_schedule_exchange(struct deipce_fsc_dev_priv *dp,
                                        unsigned int sched_num);
int deipce_fsc_check_schedule(struct deipce_fsc_dev_priv *dp,
                              unsigned int sched_num);
int deipce_fsc_update_schedule(struct deipce_fsc_dev_priv *dp,
                               unsigned int sched_num, bool *pending);
void deipce_fsc_start_work(struct work_struct *work);

#endif
