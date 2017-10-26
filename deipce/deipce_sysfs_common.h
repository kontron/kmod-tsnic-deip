/** @file
 */

/*

   DE-IP Core Edge Linux driver

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

#ifndef DEIPCE_SYSFS_COMMON_H
#define DEIPCE_SYSFS_COMMON_H

#include <linux/sysfs.h>
#include <linux/version.h>

#include "deipce_types.h"

#define DEIPCE_ATTR_RW(name, show, store) \
    DEVICE_ATTR(name, (S_IWUSR | S_IRUGO), show, store)

#define DEIPCE_ATTR_RO(name, show) \
    DEVICE_ATTR(name, S_IRUGO, show, NULL)

#define DEIPCE_BIN_ATTR_RW(name, read, write, size) \
    BIN_ATTR(name, (S_IWUSR | S_IRUGO), read, write, size)

#define DEIPCE_BIN_ATTR_RO(name, read, size) \
    BIN_ATTR(name, S_IRUGO, read, NULL, size)

#define to_deipce_netdev_priv(dev) \
    ((struct deipce_netdev_priv *)netdev_priv(to_net_dev(dev)))

#define to_deipce_port_priv(dev) (to_deipce_netdev_priv(dev)->pp)

// kstrtobool appeared in Linux 4.6.
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
#define kstrtobool(buf, value) strtobool(buf, value)
#endif

#endif
