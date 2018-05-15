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

#ifndef DEIPCE_TIME_H
#define DEIPCE_TIME_H

#include <linux/types.h>

struct timespec64;
struct deipce_time;

/**
 * Clock selectors.
 */
enum deipce_time_sel {
    DEIPCE_TIME_SEL_TS,         ///< timestamping clock
    DEIPCE_TIME_SEL_WRK,        ///< worker clock
};

int deipce_time_get_phc(struct deipce_time *dt, enum deipce_time_sel sel);
int deipce_time_get_avail_phc(struct deipce_time *dt, unsigned int num);
int deipce_time_set_phc(struct deipce_time *dt, enum deipce_time_sel sel,
                        int phc_index);

int deipce_time_get_time(struct deipce_time *dt, enum deipce_time_sel sel,
                         struct timespec64 *time);
int deipce_time_get_granularity(struct deipce_time *dt,
                                enum deipce_time_sel sel,
                                uint32_t *granularity);

// Until autoconfig is in place
#ifndef DEIPCE_AUTOCONFIG

struct flx_frtc_dev_priv;
struct deipce_fpts_dev_priv;
struct deipce_ibc_dev_priv;
struct ptp_clock_info;

void deipce_time_init_driver(void);
int deipce_time_create_by_clocks(struct flx_frtc_dev_priv *clk1,
                                 struct flx_frtc_dev_priv *clk2,
                                 struct deipce_fpts_dev_priv *fpts);
void deipce_time_remove_clock(struct flx_frtc_dev_priv *clk);
int deipce_time_add_mux(struct flx_frtc_dev_priv *clk,
                        struct deipce_ibc_dev_priv *ibc);
void deipce_time_remove_mux(struct deipce_ibc_dev_priv *ibc);
struct deipce_time *deipce_time_get_by_clock(struct flx_frtc_dev_priv *clk);

#endif

#endif

