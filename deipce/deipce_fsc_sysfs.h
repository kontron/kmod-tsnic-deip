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

#ifndef DEIPCE_FSC_SYSFS_H
#define DEIPCE_FSC_SYSFS_H

#include "deipce_fsc_types.h"

struct deipce_port_priv;

#ifdef CONFIG_SYSFS

int deipce_fsc_sysfs_dev_init(struct deipce_fsc_dev_priv *fsc,
                              unsigned int sched_num,
                              struct deipce_port_priv *pp);
void deipce_fsc_sysfs_dev_cleanup(struct deipce_fsc_dev_priv *fsc,
                                  unsigned int sched_num,
                                  struct deipce_port_priv *pp);

#else

static inline int deipce_fsc_sysfs_dev_init(struct deipce_fsc_dev_priv *fsc,
                                            unsigned int sched_num,
                                            struct deipce_port_priv *pp)
{ return 0; }

static inline void deipce_fsc_sysfs_dev_cleanup(
        struct deipce_fsc_dev_priv *fsc,
        unsigned int sched_num,
        struct deipce_port_priv *pp);
{ return; }

#endif

#endif
