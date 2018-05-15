/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2013 Flexibilis Oy

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

#ifndef DEIPCE_DEBUGFS_H
#define DEIPCE_DEBUGFS_H

#include "deipce_types.h"

#if IS_ENABLED(CONFIG_DEBUG_FS)

int __init deipce_debugfs_init_driver(struct deipce_drv_priv *drv);
void deipce_debugfs_cleanup_driver(struct deipce_drv_priv *drv);

int deipce_debugfs_init_device(struct deipce_dev_priv *dp);
void deipce_debugfs_cleanup_device(struct deipce_dev_priv *dp);

#else

static inline int deipce_debugfs_init_driver(struct deipce_drv_priv *drv)
{ return 0; }

static inline void deipce_debugfs_cleanup_driver(struct deipce_drv_priv *drv)
{ return; }

static inline int deipce_debugfs_init_device(struct deipce_dev_priv *dp)
{ return 0; }

static inline void deipce_debugfs_cleanup_device(struct deipce_dev_priv *dp)
{ return; }

#endif // CONFIG_DEBUG_FS

#endif
