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

#ifndef DEIPCE_FSC_MAIN_H
#define DEIPCE_FSC_MAIN_H

#include <linux/init.h>

struct deipce_fsc_dev_priv;
struct ptp_clock_info;

int __init deipce_fsc_init_driver(void);
void deipce_fsc_cleanup_driver(void);

struct deipce_fsc_dev_priv *deipce_fsc_of_get_device_by_node(
        struct device_node *node);
int deipce_fsc_set_clock(struct deipce_fsc_dev_priv *fsc,
                         struct ptp_clock_info *phc);
struct ptp_clock_info *deipce_fsc_get_clock(struct deipce_fsc_dev_priv *fsc);
void deipce_fsc_update_link(struct deipce_fsc_dev_priv *fsc,
                            unsigned int sched_num,
                            struct net_device *netdev);

#endif
