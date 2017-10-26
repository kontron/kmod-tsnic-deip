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

#ifndef DEIPCE_MMIO_H
#define DEIPCE_MMIO_H

int deipce_mmio_init_device(struct deipce_dev_priv *dp,
                            struct platform_device *pdev,
                            struct deipce_cfg *frs_cfg);
void deipce_mmio_cleanup_device(struct deipce_dev_priv *dp);

#endif
