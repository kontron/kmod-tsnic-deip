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

#ifndef DEIPCE_IBC_MAIN_H
#define DEIPCE_IBC_MAIN_H

#define DEIPCE_IBC_MAX_CLOCKS 2

struct deipce_ibc_dev_priv;
struct ptp_clock_info;

struct deipce_ibc_phc_info {
    struct ptp_clock_info *info;
    int index;
};

int __init deipce_ibc_init_driver(void);
void deipce_ibc_cleanup_driver(void);

struct deipce_ibc_dev_priv *deipce_ibc_of_get_device_by_node(
        struct device_node *node);
unsigned int deipce_ibc_get_clocks(struct deipce_ibc_dev_priv *dp,
                                   struct deipce_ibc_phc_info *phc_list,
                                   unsigned int count,
                                   unsigned int *time_sel,
                                   unsigned int *gp_sel);
int deipce_ibc_set(struct deipce_ibc_dev_priv *dp,
                   unsigned int time_sel, unsigned int gp_sel);
int deipce_ibc_set_by_phc(struct deipce_ibc_dev_priv *dp, int phc_index);

#endif
