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

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/if_bridge.h>

#include "deipce_main.h"
#include "deipce_hw.h"
#include "deipce_mstp.h"

/**
 * Write port forwarding state for a FID.
 * @param pp Port privates.
 * @param fid FID number.
 * @param state Port FID forwarding state bits,
 * see PORT_STATE_xxxx and PORT_STATE_STATE_MASK.
 */
static void deipce_mstp_write_port_fid_cfg(struct deipce_port_priv *pp,
                                           uint16_t fid, uint16_t state)
{
    unsigned int shift = PORT_FID_CFG_SHIFT(fid - 1);
    unsigned int reg = PORT_REG_FID_CFG(fid - 1);
    uint16_t fid_cfg = deipce_read_port_reg(pp, reg);

    fid_cfg &= ~(PORT_STATE_STATE_MASK << shift);
    fid_cfg |= state << shift;

    deipce_write_port_reg(pp, reg, fid_cfg);

    return;
}

/**
 * Find given MSTID value from internal list of MSTIDs.
 * @param dp Switch privates.
 * @param mstid MSTID value to look for.
 * @param index Place for writing index to MSTID list when found.
 */
static int deipce_mstp_find_mstid(struct deipce_dev_priv *dp,
                                  uint16_t mstid, unsigned int *index)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;

    for (*index = 0; *index < mstp->mstid_count; (*index)++) {
        if (mstp->mstids[*index] == mstid)
            return 0;
    }

    return -EINVAL;
}

/**
 * Initialize MSTP support for switch.
 * @param dp Switch privates.
 */
int deipce_mstp_init_switch(struct deipce_dev_priv *dp)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;
    unsigned int mstid_elems = dp->features.fids + 1;
    unsigned int fid_elems = mstid_elems;
    int ret = -ENOMEM;

    mstp->mstid_count = 0;
    mstp->mstids = kzalloc(mstid_elems * sizeof(*mstp->mstids), GFP_KERNEL);
    if (!mstp->mstids)
        return ret;

    // Allocate all FIDs to CIST initially.
    mstp->fid_mstid = kzalloc(fid_elems * sizeof(*mstp->fid_mstid), GFP_KERNEL);
    if (!mstp->fid_mstid)
        goto err_fid_mstid;

    mstp->mstids[mstp->mstid_count++] = DEIPCE_MSTP_MSTID_CIST;

    return 0;

err_fid_mstid:
    kfree(mstp->mstids);
    mstp->mstids = NULL;

    return ret;
}

/**
 * Cleanup MSTP support for switch.
 * @param dp Switch privates.
 */
void deipce_mstp_cleanup_switch(struct deipce_dev_priv *dp)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;

    mstp->mstid_count = 0;

    kfree(mstp->fid_mstid);
    mstp->fid_mstid = NULL;

    kfree(mstp->mstids);
    mstp->mstids = NULL;

    return;
}

/**
 * Initialize MSTP support for port.
 * @param dp Switch privates.
 * @param pp Port privates.
 */
int deipce_mstp_init_port(struct deipce_dev_priv *dp,
                          struct deipce_port_priv *pp)
{
    unsigned int mstid_elems = dp->features.fids + 1;

    pp->mstp.mstid_stp_state = kzalloc(mstid_elems, GFP_KERNEL);
    if (!pp->mstp.mstid_stp_state)
        return -ENOMEM;

    // Default state when FID is initially assigned to user MSTID.
    memset(pp->mstp.mstid_stp_state, BR_STATE_FORWARDING, mstid_elems);

    return 0;
}

/**
 * Cleanup MSTP support for port.
 * @param dp Switch privates.
 * @param pp Port privates.
 */
void deipce_mstp_cleanup_port(struct deipce_dev_priv *dp,
                              struct deipce_port_priv *pp)
{
    if (pp->mstp.mstid_stp_state) {
        kfree(pp->mstp.mstid_stp_state);
        pp->mstp.mstid_stp_state = NULL;
    }

    return;
}

/**
 * Add new user MSTID.
 * @param dp Switch privates.
 * @param mstid MSTID to add.
 */
int deipce_mstp_add_msti(struct deipce_dev_priv *dp, uint16_t mstid)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;
    unsigned int index;
    int ret = -ENOSPC;

    dev_dbg(dp->this_dev, "%s() mstid %hu\n", __func__, mstid);

    // CIST is always there, accept it silently.
    if (mstid == DEIPCE_MSTP_MSTID_CIST)
        return 0;

    if (mstid > DEIPCE_MSTP_MSTID_USER_MAX)
        return -EINVAL;

    rtnl_lock();

    if (mstp->mstid_count >= dp->features.fids)
         goto out;

    // Must not exist already.
    ret = deipce_mstp_find_mstid(dp, mstid, &index);
    if (ret == 0) {
        ret = -EINVAL;
        goto out;
    }

    mstp->mstids[mstp->mstid_count++] = mstid;
    ret = 0;

out:
    rtnl_unlock();

    return ret;
}

/**
 * Remove user MSTID.
 * @param dp Switch privates.
 * @param mstid MSTID to remove.
 */
int deipce_mstp_remove_msti(struct deipce_dev_priv *dp, uint16_t mstid)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;
    unsigned int index;
    uint16_t fid;
    int ret;

    dev_dbg(dp->this_dev, "%s() mstid %hu\n", __func__, mstid);

    // CIST is always there, reject its removal.
    if (mstid == DEIPCE_MSTP_MSTID_CIST)
        return -EINVAL;

    rtnl_lock();

    ret = deipce_mstp_find_mstid(dp, mstid, &index);
    if (ret)
        goto out;

    // Not allowed if it still has FIDs.
    for (fid = 1; fid <= dp->features.fids; fid++) {
        if (mstp->fid_mstid[fid] == mstid) {
            ret = -ENOTEMPTY;
            goto out;
        }
    }

    // No holes in our MSTID list.
    if (index < mstp->mstid_count - 1) {
        struct deipce_port_priv *pp;
        unsigned int port_num;

        mstp->mstids[index] = mstp->mstids[mstp->mstid_count - 1];

        // Keep port MSTID indices aligned.
        for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
            pp = dp->port[port_num];
            if (!pp)
                continue;

            pp->mstp.mstid_stp_state[index] =
                pp->mstp.mstid_stp_state[mstp->mstid_count - 1];
        }
    }

    mstp->mstid_count--;
    ret = 0;

out:
    rtnl_unlock();

    return ret;
}

/**
 * Fill list of MSTIDs.
 * @param dp Switch privates.
 * @param msti_list List to fill. List has a fixed sized list of MSTID values
 * like in array msti_list[INDEX], where
 * INDEX = 0,1,2,3 .. FIDS.
 * Value of UINT_MAX denotes an unused MSTID.
 * @param first Index of first entry in list.
 * @param count Number of entries to fill.
 */
int deipce_mstp_fill_msti_list(struct deipce_dev_priv *dp,
                               unsigned int *msti_list,
                               unsigned int first,
                               unsigned int count)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;
    unsigned int end = first + count;
    unsigned int index;

    dev_dbg(dp->this_dev, "%s() first %u count %u end %u\n",
            __func__, first, count, end);

    if (first + count > dp->features.fids)
        return -EINVAL;

    rtnl_lock();

    for (index = first; index < end; index++) {
        if (index >= mstp->mstid_count)
            msti_list[index - first] = UINT_MAX;
        else
            msti_list[index - first] = mstp->mstids[index];
    }

    rtnl_unlock();

    return 0;
}

/**
 * Determine FID specific port forwarding state register value from STP state.
 * @param stp_state STP state, BR_STATE_xxx.
 * @param state Pace for port forwarding state to use in registers.
 */
static int deipce_mstp_port_state(unsigned int stp_state, uint16_t *state)
{
    int ret = 0;

    switch (stp_state) {
    case BR_STATE_FORWARDING:
        *state = PORT_STATE_FORWARDING;
        break;
    case BR_STATE_LEARNING:
        *state = PORT_STATE_LEARNING;
        break;
    case BR_STATE_DISABLED:
    case BR_STATE_BLOCKING:
    case BR_STATE_LISTENING:
        *state = PORT_STATE_DISABLED;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

/**
 * Set FID to MSTID mapping.
 * @param dp Switch privates.
 * @param fid FID number.
 * @param mstid MSTID number.
 */
int deipce_mstp_set_fid_msti(struct deipce_dev_priv *dp, uint16_t fid,
                             uint16_t mstid)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;
    struct deipce_port_priv *pp;
    unsigned int port_num;
    // Default FID specific port forwarding state is for CIST.
    uint16_t state = PORT_FID_CFG_NO_OVERRIDE;
    unsigned int index;
    int ret = -EINVAL;

    dev_dbg(dp->this_dev, "%s() fid %hu mstid %hu\n", __func__, fid, mstid);

    if (!deipce_is_vlan_fid_valid(dp, fid))
        return -EINVAL;

    rtnl_lock();

    ret = deipce_mstp_find_mstid(dp, mstid, &index);
    if (ret)
        goto out;

    mstp->fid_mstid[fid] = mstid;

    // Update FID specific port forwarding state to match the new MSTID.
    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        pp = dp->port[port_num];
        if (!pp)
            continue;

        if (mstid != DEIPCE_MSTP_MSTID_CIST)
            deipce_mstp_port_state(pp->mstp.mstid_stp_state[index], &state);

        netdev_dbg(pp->netdev, "%s()   FID %hu state 0x%x\n",
                   __func__, fid, state);
        deipce_mstp_write_port_fid_cfg(pp, fid, state);
    }

out:
    rtnl_unlock();

    return ret;
}

/**
 * Get FID to MSTID mapping.
 * @param dp Switch privates.
 * @param fid FID number.
 * @param mstid Place for MSTID number.
 */
int deipce_mstp_get_fid_msti(struct deipce_dev_priv *dp, uint16_t fid,
                             uint16_t *mstid)
{
    struct deipce_mstp *mstp = &dp->vlan.mstp;

    dev_dbg(dp->this_dev, "%s() fid %hu\n", __func__, fid);

    if (!deipce_is_vlan_fid_valid(dp, fid))
        return -EINVAL;

    rtnl_lock();

    *mstid = mstp->fid_mstid[fid];

    rtnl_unlock();

    return 0;
}

/**
 * Clear dynamic MAC addresses of all FIDs allocated to given MSTID.
 * @param pp Port privates.
 * @param mstid MSTID number.
 */
int deipce_mstp_flush_msti(struct deipce_port_priv *pp, uint16_t mstid)
{
    struct deipce_dev_priv *dp = pp->dp;
    struct deipce_mstp *mstp = &dp->vlan.mstp;
    unsigned int index;
    uint16_t fid;
    int ret = -EINVAL;

    dev_dbg(dp->this_dev, "%s() mstid %hu\n", __func__, mstid);

    rtnl_lock();

    ret = deipce_mstp_find_mstid(dp, mstid, &index);
    if (ret)
        goto out;

    for (fid = 1; fid <= dp->features.fids; fid++) {
        if (mstid != mstp->fid_mstid[fid])
            continue;

        dev_dbg(dp->this_dev, "%s()   fid %hu\n", __func__, fid);
        ret = deipce_clear_mac_table(dp, fid, 1u << pp->port_num);
        if (ret)
            break;
    }

out:
    rtnl_unlock();

    return ret;
}

/**
 * Set port forwarding state for each FID allocated to given MSTID.
 * @param pp Port privates.
 * @param mstid MSTID number.
 * Using this interface for CIST leaves state in Linux bridge out of sync.
 * @param stp_state BR_STATE_xxx value.
 */
int deipce_mstp_set_port_state(struct deipce_port_priv *pp, uint16_t mstid,
                               unsigned int stp_state)
{
    struct deipce_dev_priv *dp = pp->dp;
    uint16_t state;
    int ret = -EINVAL;

    dev_dbg(dp->this_dev, "%s() mstid %hu stp_state %u\n",
            __func__, mstid, stp_state);

    ret = deipce_mstp_port_state(stp_state, &state);
    if (ret)
        return ret;

    rtnl_lock();

    if (mstid == DEIPCE_MSTP_MSTID_CIST) {
        ret = deipce_set_port_stp_state(pp->netdev, stp_state);
    }
    else {
        struct deipce_mstp *mstp = &dp->vlan.mstp;
        unsigned int index;
        uint16_t fid;

        ret = deipce_mstp_find_mstid(dp, mstid, &index);
        if (ret)
            goto out;

        for (fid = 1; fid <= dp->features.fids; fid++) {
            if (mstp->fid_mstid[fid] != mstid)
                continue;

            netdev_dbg(pp->netdev, "%s()   fid %hu state 0x%x\n",
                       __func__, fid, state);

            deipce_mstp_write_port_fid_cfg(pp, fid, state);
        }

        pp->mstp.mstid_stp_state[index] = (uint8_t)stp_state;
    }

out:
    rtnl_unlock();

    return ret;
}

/**
 * Get port forwarding state for a given MSTID.
 * @param pp Port privates.
 * @param mstid MSTID number.
 * @param stp_state Place for BR_STATE_xxx value.
 */
int deipce_mstp_get_port_state(struct deipce_port_priv *pp, uint16_t mstid,
                               unsigned int *stp_state)
{
    struct deipce_dev_priv *dp = pp->dp;
    int ret = 0;

    dev_dbg(dp->this_dev, "%s() mstid %hu\n", __func__, mstid);

    rtnl_lock();

    if (mstid == DEIPCE_MSTP_MSTID_CIST) {
        *stp_state = deipce_get_port_stp_state(pp->netdev);
    }
    else {
        unsigned int index;

        ret = deipce_mstp_find_mstid(dp, mstid, &index);
        if (ret)
            goto out;

        *stp_state = pp->mstp.mstid_stp_state[index];
    }

out:
    rtnl_unlock();

    return ret;
}

