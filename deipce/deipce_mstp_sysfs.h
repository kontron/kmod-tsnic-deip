/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2018 Flexibilis Oy

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

#ifndef DEIPCE_MSTP_SYSFS_H
#define DEIPCE_MSTP_SYSFS_H

struct deipce_dev_priv;
struct deipce_port_priv;

#ifdef CONFIG_SYSFS
#ifdef CONFIG_NET_SWITCHDEV

int deipce_mstp_sysfs_init_switch(struct deipce_dev_priv *dp);
void deipce_mstp_sysfs_cleanup_switch(struct deipce_dev_priv *dp);

int deipce_mstp_sysfs_init_port(struct deipce_port_priv *dp);
void deipce_mstp_sysfs_cleanup_port(struct deipce_port_priv *dp);

#else

static inline int deipce_mstp_sysfs_init_port(struct deipce_port_priv *dp)
{ return 0; }

static inline void deipce_mstp_sysfs_cleanup_port(struct deipce_port_priv *dp)
{ return; }

#endif // CONFIG_NET_SWITCHDEV
#endif // CONFIG_SYSFS

#endif
