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

#ifndef DEIPCE_FPTS_MAIN_H
#define DEIPCE_FPTS_MAIN_H

#include "deipce_fpts_types.h"

struct deipce_fpts_dev_priv;
struct deipce_fpts_event;

int __init deipce_fpts_init_driver(void);
void deipce_fpts_cleanup_driver(void);

struct deipce_fpts_dev_priv *deipce_fpts_of_get_device_by_node(
        struct device_node *node);
void deipce_fpts_enable(struct deipce_fpts_dev_priv *dp);
void deipce_fpts_disable(struct deipce_fpts_dev_priv *dp);
int deipce_fpts_get_event(struct deipce_fpts_dev_priv *dp,
                          struct deipce_fpts_event *event);

#endif
