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

#ifndef DEIPCE_MSTP_H
#define DEIPCE_MSTP_H

#include <linux/types.h>

// MSTID values

#define DEIPCE_MSTP_MSTID_CIST          0       ///< CIST-MSTID
#define DEIPCE_MSTP_MSTID_USER_MAX      0xffb   ///< max. user MSTID value
#define DEIPCE_MSTP_MSTID_MAX           0xfff   ///< max. MSTID value

struct deipce_dev_priv;
struct deipce_port_priv;

/**
 * Switch MSTP context.
 */
struct deipce_mstp {
    uint16_t mstid_count;       ///< number of MSTIDs in use
    uint16_t *mstids;           ///< array of used MSTIDs, including CIST
    uint16_t *fid_mstid;        ///< MSTID of each FID
};

/**
 * Port MSTP context.
 */
struct deipce_port_mstp {
    uint8_t *mstid_stp_state;   ///< STP state for MSTIDs, BR_STATE_xxx
};

int deipce_mstp_init_switch(struct deipce_dev_priv *dp);
void deipce_mstp_cleanup_switch(struct deipce_dev_priv *dp);
int deipce_mstp_init_port(struct deipce_dev_priv *dp,
                          struct deipce_port_priv *pp);
void deipce_mstp_cleanup_port(struct deipce_dev_priv *dp,
                              struct deipce_port_priv *pp);

int deipce_mstp_add_msti(struct deipce_dev_priv *dp, uint16_t mstid);
int deipce_mstp_remove_msti(struct deipce_dev_priv *dp, uint16_t mstid);
int deipce_mstp_fill_msti_list(struct deipce_dev_priv *dp,
                               unsigned int *msti_list,
                               unsigned int first,
                               unsigned int count);

int deipce_mstp_set_fid_msti(struct deipce_dev_priv *dp, uint16_t fid,
                             uint16_t mstid);
int deipce_mstp_get_fid_msti(struct deipce_dev_priv *dp, uint16_t fid,
                             uint16_t *mstid);

int deipce_mstp_flush_msti(struct deipce_port_priv *pp, uint16_t mstid);
int deipce_mstp_set_port_state(struct deipce_port_priv *pp, uint16_t mstid,
                               unsigned int stp_state);
int deipce_mstp_get_port_state(struct deipce_port_priv *pp, uint16_t mstid,
                               unsigned int *stp_state);

#endif
