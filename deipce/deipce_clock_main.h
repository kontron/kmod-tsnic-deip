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

#ifndef DEIPCE_CLOCK_MAIN_H
#define DEIPCE_CLOCK_MAIN_H

int __init deipce_clock_init_driver(void);
void deipce_clock_cleanup_driver(void);

#endif
