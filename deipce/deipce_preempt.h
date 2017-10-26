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

#ifndef DEIPCE_PREEMPT_H
#define DEIPCE_PREEMPT_H

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/sched.h>

/// Number of bits in management trailer for preemption
#define DEIPCE_TRAILER_PREEMPT_BITS 2

struct deipce_port_priv;

/**
 * Preemption verification status.
 */
enum deipce_preempt_verify_status {
    DEIPCE_PREEMPT_VERIFY_DISABLED,     ///< verification is disabled
    DEIPCE_PREEMPT_VERIFY_IDLE,         ///< waiting verification to started
    DEIPCE_PREEMPT_VERIFY_IN_PROGRESS,  ///< in progress
    DEIPCE_PREEMPT_VERIFY_FAILED,       ///< failed
    DEIPCE_PREEMPT_VERIFY_OK,           ///< succeeded
};

/**
 * FES port preemption context.
 * Locking rules:
 * - changes to preemption management (mgmt member) require acquiring mutex and
 *   stopping verification timer
 * - changes to verify member require acquiring spinlock
 * - starting timer requires acquiring mutex, if timer is not already running
 * - spinlock can be held when starting timer
 * - stopping timer requires acquiring mutex
 * - spinlock must not be held when stopping timer
 * - timer can be requested to stop by setting verify.status
 *   to DEIPCE_PREEMPT_VERIFY_IDLE
 * Accessor functions below do things correctly.
 */
struct deipce_preempt {
    struct  {
        struct mutex lock;              ///< synchronize management changes
        struct {
            unsigned int tx_prio_enable : 8;    ///< enabled priority queues
            unsigned int enable_verify : 1;     ///< enable verification
            unsigned int link : 1;              ///< link status
        };
        unsigned long int interval;     ///< verification time interval in ns
        unsigned long int slack;        ///< interval slack +/- ns
        unsigned int verify_limit;      ///< verification counter limit or 0
    } mgmt;                             ///< preemption management
    struct {
        spinlock_t lock;                ///< synchronize state access
        enum deipce_preempt_verify_status status;       ///< status and result
        struct tasklet_struct send_tasklet;     ///< tasklet for SMD-V sending
        unsigned int count;             ///< number of verifications attempted
        struct hrtimer timer;           ///< SMD-V send timer
    } verify;                           ///< verification state data
};

void deipce_preempt_init(struct deipce_port_priv *pp);
void deipce_preempt_cleanup(struct deipce_port_priv *pp);
int deipce_preempt_set_min_frag_size(struct deipce_port_priv *pp,
                                     unsigned int size);
unsigned int deipce_preempt_get_min_frag_size(struct deipce_port_priv *pp);
void deipce_preempt_set_enable(struct deipce_port_priv *pp,
                               unsigned int prio_mask);
unsigned int deipce_preempt_get_enable(struct deipce_port_priv *pp);
void deipce_preempt_set_verify(struct deipce_port_priv *pp, bool enable);
bool deipce_preempt_get_verify(struct deipce_port_priv *pp);
void deipce_preempt_set_verify_time(struct deipce_port_priv *pp,
                                    unsigned int ms);
unsigned int deipce_preempt_get_verify_time(struct deipce_port_priv *pp);
void deipce_preempt_set_verify_limit(struct deipce_port_priv *pp,
                                     unsigned int limit);
unsigned int deipce_preempt_get_verify_limit(struct deipce_port_priv *pp);
unsigned int deipce_preempt_get_verify_count(struct deipce_port_priv *pp);
void deipce_preempt_update_link(struct deipce_port_priv *pp, bool link);
enum deipce_preempt_verify_status deipce_preempt_get_status(
        struct deipce_port_priv *pp);

uint16_t deipce_preempt_trailer_mask(struct deipce_dev_priv *dp);

bool deipce_preempt_rx_frame(struct deipce_port_priv *pp,
                             struct sk_buff *rx_frame, uint16_t trailer);

#endif
