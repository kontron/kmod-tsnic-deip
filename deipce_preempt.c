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

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/types.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/version.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_netdevif.h"
#include "deipce_preempt_sysfs.h"
#include "deipce_preempt.h"

/// Preemption verification counter limit
#define DEIPCE_PREEMPT_VERIFY_LIMIT 3

/// Minimum allowed verify time in ms
#define DEIPCE_PREEMPT_VERIFY_TIME_MIN 1

/// Maximum allowed verify time in ms
#define DEIPCE_PREEMPT_VERIFY_TIME_MAX 128

/// Uncomment to enable preemption frame dumps
//#define DEBUG_PREEMPT_FRAMES

/**
 * Management trailer SMD bits for preemptable traffic.
 */
enum deipce_preempt_smd {
    DEIPCE_PREEMPT_SFD = 0x0,           ///< SFD
    DEIPCE_PREEMPT_SMDV = 0x2,          ///< SMD-V (verify)
    DEIPCE_PREEMPT_SMDR = 0x3,          ///< SMD-R (respond)
};

/// SMD-V and SMD-R frame data for checking received frames
static const uint8_t deipce_preempt_verify_frame[ETH_ZLEN] = { 0 };

/**
 * Get management trailer with SMD bits set for given IEEE 802.3br packets.
 * @param dp FRS device privates.
 * @return Management trailer which has only relevant SMD bits set.
 */
static uint16_t deipce_preempt_smd_to_trailer(struct deipce_dev_priv *dp,
                                              enum deipce_preempt_smd smd)
{
    if (dp->features.preempt_ports) {
        // Next to MACsec bit, regardless of MACsec feature.
        return (uint16_t)smd << (dp->trailer_len*8 - 1 - 2);
    }

    return 0;
}

/**
 * Get management trailer mask for SMD bits.
 * @param dp FRS device privates.
 * param trailer Management trailer value.
 */
uint16_t deipce_preempt_trailer_mask(struct deipce_dev_priv *dp)
{
    return deipce_preempt_smd_to_trailer(dp, DEIPCE_PREEMPT_SMDR);
}

/**
 * Determine IEEE 802.3br packet type from management trailer.
 * @param dp FRS device privates.
 * param trailer Management trailer value.
 */
static enum deipce_preempt_smd deipce_preempt_trailer_to_smd(
        struct deipce_dev_priv *dp, uint16_t trailer)
{
    if (dp->features.preempt_ports) {
        // Next to MACsec bit, regardless of MACsec feature.
        trailer >>= dp->trailer_len*8 - 1 - 2;

        switch (trailer & DEIPCE_PREEMPT_SMDR) {
        case DEIPCE_PREEMPT_SMDV: return DEIPCE_PREEMPT_SMDV;
        case DEIPCE_PREEMPT_SMDR: return DEIPCE_PREEMPT_SMDR;
        default: ;
        }
    }

    return DEIPCE_PREEMPT_SFD;
}

/**
 * Construct a new verify (SMD-V) or response (SMD-R) mPacket.
 * @param pp Port to use for sending.
 * @param smd Type of SMD to construct.
 * @return New frame or NULL on error.
 */
static struct sk_buff *deipce_preempt_make_frame(struct deipce_port_priv *pp,
                                                 enum deipce_preempt_smd smd)
{
    struct deipce_dev_priv *dp = pp->dp;
    // Both MACsec and preemption are not allowed on the same port.
    uint16_t trailer = pp->trailer | deipce_preempt_smd_to_trailer(dp, smd);
    struct sk_buff *skb = dev_alloc_skb(ETH_ZLEN + 2);
    struct ethhdr *eth;
    void *data;
    int ret;

    if (!skb)
        return NULL;

    eth = (struct ethhdr *)skb_push(skb, ETH_HLEN);
    skb_set_mac_header(skb, 0);
    skb_set_network_header(skb, skb->len);
    memset(eth, 0, sizeof(*eth));
    data = skb_push(skb, ETH_ZLEN - ETH_HLEN);
    memset(data, 0, ETH_ZLEN - ETH_HLEN);
    skb->protocol = htons(ETH_P_802_3);

    ret = deipce_skb_add_trailer(skb, trailer, dp->trailer_len);
    if (ret) {
        dev_kfree_skb(skb);
        return  NULL;
    }

#ifdef DEBUG_PREEMPT_FRAMES
    netdev_dbg(pp->netdev, "%s() trailer: port 0x%x smd 0x%x final 0x%x\n",
               __func__, pp->trailer, deipce_preempt_smd_to_trailer(dp, smd),
               trailer);
    netdev_dbg(pp->netdev,
               "%s() skb len %u dat head %p mac %p data %p tail %p end %p\n",
               __func__, skb->len, skb->head, skb_mac_header(skb),
               skb->data, skb_tail_pointer(skb), skb_end_pointer(skb));
#endif

    return skb;
}

/**
 * Send preemption verification frame (mPacket with SMD-V).
 * @param pp Port privates.
 */
static void deipce_preempt_send_smdv(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    struct sk_buff *skb = deipce_preempt_make_frame(pp, DEIPCE_PREEMPT_SMDV);

    netdev_dbg(pp->netdev, "preempt: send SMD-V status %u count %u/%u\n",
               pp->preempt.verify.status,
               pp->preempt.verify.count,
               DEIPCE_PREEMPT_VERIFY_LIMIT);

    if (!skb)
        return;

    // Send it.
    skb->dev = dp->real_netdev;
    dev_queue_xmit(skb);

    return;
}

/**
 * Setup preemption verification timer interval.
 * @param preempt Preemption context.
 * @param ms Preemption interval in milliseconds.
 */
static inline void deipce_preempt_timer_setup(struct deipce_preempt *preempt,
                                              unsigned int ms)
{
    /*
     * Verify timer should expire ms +/- 20 % after being started.
     */
    preempt->mgmt.interval = ms*NSEC_PER_MSEC;
    preempt->mgmt.slack = (preempt->mgmt.interval*20u)/100u;

    return;
}

/**
 * Helper function to start verification timer with correct recurring.
 * @param preempt Preemption context.
 */
static void deipce_preempt_timer_start(struct deipce_preempt *preempt)
{
    ktime_t interval =
        ns_to_ktime(preempt->mgmt.interval - preempt->mgmt.slack);

    hrtimer_start_range_ns(&preempt->verify.timer,
                           interval, 2*preempt->mgmt.slack,
                           HRTIMER_MODE_REL);
    return;
}

/**
 * Adapt verification state machine to updated input signals.
 * Must be called with mutex held and spinlock unheld.
 * @param pp Port privates.
 */
static void deipce_preempt_update_verification(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned long flags;
    bool timer_should_run;
    uint16_t active = 0;

    netdev_dbg(pp->netdev, "preempt: update %s prio 0x%02x %s link %i\n",
               preempt->mgmt.tx_enable ? "enable" : "disable",
               preempt->mgmt.tx_prio_enable,
               preempt->mgmt.enable_verify ? "verify" : "",
               preempt->mgmt.link_mode);

    cancel_delayed_work(&preempt->verify.retry_work);
    hrtimer_cancel(&preempt->verify.timer);

    spin_lock_irqsave(&preempt->verify.lock, flags);

    if (preempt->mgmt.tx_enable &&
        preempt->mgmt.tx_prio_enable != 0 &&
        !preempt->mgmt.enable_verify)
        active = preempt->mgmt.tx_prio_enable;
    deipce_write_port_reg(pp, PORT_REG_TX_PREE0, active);

    timer_should_run =
        preempt->mgmt.tx_enable &&
        preempt->mgmt.tx_prio_enable != 0 &&
        preempt->mgmt.enable_verify &&
        preempt->mgmt.link_mode != LM_DOWN;
    if (timer_should_run) {
        // Do not restart verification unnecessarily.
        if (preempt->verify.status == DEIPCE_PREEMPT_VERIFY_SUCCEEDED) {
            timer_should_run = false;
        }
        else {
            preempt->verify.status = DEIPCE_PREEMPT_VERIFY_VERIFYING;
            preempt->verify.count = 1;
        }
    }
    else {
        if (preempt->mgmt.enable_verify)
            preempt->verify.status = DEIPCE_PREEMPT_VERIFY_INITIAL;
        else
            preempt->verify.status = DEIPCE_PREEMPT_VERIFY_DISABLED;
        preempt->verify.count = 0;
    }

    spin_unlock_irqrestore(&preempt->verify.lock, flags);

    if (timer_should_run) {
        netdev_dbg(pp->netdev, "preempt: update starting timer\n");
        deipce_preempt_send_smdv(pp);
        deipce_preempt_timer_start(preempt);
    }

    return;
}

/**
 * Preemption verification timer callback function.
 * @param timer Preemption context timer in port privates.
 * @return Whether or not to restart timer.
 */
static enum hrtimer_restart deipce_preempt_verify_timer(struct hrtimer *timer)
{
    struct deipce_preempt *preempt =
        container_of(timer, struct deipce_preempt, verify.timer);
    unsigned long int flags;

    spin_lock_irqsave(&preempt->verify.lock, flags);

    // Ignore and stop recurring if timer should no longer run.
    if (preempt->verify.status != DEIPCE_PREEMPT_VERIFY_VERIFYING) {
        spin_unlock_irqrestore(&preempt->verify.lock, flags);
        return HRTIMER_NORESTART;
    }

    if (preempt->verify.count >= DEIPCE_PREEMPT_VERIFY_LIMIT) {
        // Retry verification after 100 * aMACMergeVerifyTime, FLEXDE-828.
        struct deipce_drv_priv *drv = deipce_get_drv_priv();
        unsigned long int retry_interval =
            msecs_to_jiffies((preempt->mgmt.interval/NSEC_PER_MSEC) * 100);

        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_FAILED;
        queue_delayed_work(drv->wq_low, &preempt->verify.retry_work,
                           retry_interval);
        spin_unlock_irqrestore(&preempt->verify.lock, flags);
        return HRTIMER_NORESTART;
    }

    preempt->verify.count++;

    spin_unlock_irqrestore(&preempt->verify.lock, flags);

    /*
     * Cannot send frame directly from hrtimer callback,
     * use tasklet for its low latency.
     */
    tasklet_schedule(&preempt->verify.send_tasklet);

    // hrtimer_forward_now would cause timer to expire much too soon.
    deipce_preempt_timer_start(preempt);

    return HRTIMER_RESTART;
}

/**
 * Calculate updated hold and release advance values.
 * This is called when link speed or minimum fragment size changes.
 * @param pp Port privates.
 */
static void deipce_preempt_update_advances(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    struct deipce_preempt *preempt = &pp->preempt;
    uint32_t clock_cycle_length = NSEC_PER_SEC / dp->features.clock_freq;
    uint32_t mii_cycle_length = 0;
    uint32_t frag_size_delay = 128 + deipce_preempt_get_min_frag_size(pp) * 64;
    uint16_t hold = 0;
    uint16_t release = 0;
    uint32_t clk_cycles;
    uint32_t mii_cycles;

    switch (preempt->mgmt.link_mode) {
    case LM_1000FULL:
        hold = deipce_read_generic(dp, FRS_REG_GENERIC_HOLD_ADV_1000, U16_MAX);
        release = deipce_read_generic(dp, FRS_REG_GENERIC_REL_ADV_1000,
                                      U16_MAX);
        mii_cycle_length = 8;
        break;
    case LM_100FULL:
        hold = deipce_read_generic(dp, FRS_REG_GENERIC_HOLD_ADV_100, U16_MAX);
        release = deipce_read_generic(dp, FRS_REG_GENERIC_REL_ADV_100, U16_MAX);
        frag_size_delay *= 2;
        mii_cycle_length = 40;
        break;
    case LM_10FULL:
    case LM_DOWN:
        hold = deipce_read_generic(dp, FRS_REG_GENERIC_HOLD_ADV_10, U16_MAX);
        release = deipce_read_generic(dp, FRS_REG_GENERIC_REL_ADV_10, U16_MAX);
        frag_size_delay *= 2;
        mii_cycle_length = 400;
        break;
    }

    clk_cycles = hold & FRS_REG_GENERIC_DELAY_CLK_MASK;
    mii_cycles = (hold >> FRS_REG_GENERIC_DELAY_MII_SHIFT) &
        FRS_REG_GENERIC_DELAY_MII_MASK;
    preempt->mgmt.hold_advance = clock_cycle_length * clk_cycles +
        mii_cycle_length * (frag_size_delay + mii_cycles);

    clk_cycles = release & FRS_REG_GENERIC_DELAY_CLK_MASK;
    mii_cycles = (release >> FRS_REG_GENERIC_DELAY_MII_SHIFT) &
        FRS_REG_GENERIC_DELAY_MII_MASK;
    preempt->mgmt.release_advance = clock_cycle_length * clk_cycles +
        mii_cycle_length * mii_cycles;

    return;
}

/**
 * Handle preemption verification retry work.
 * Preemption verification is retried at 100 x aMACMergeVerifyTime intervals so
 * that verification can actually succeed also even when link partners do not
 * detect link at the same time, within the time it takes to complete a single
 * verification sequence
 * (verifyLimit x aMACMergeVerifyTime = 3 x [1 ms .. 128 ms +/- 20 %]),
 * FLEXDE-828.
 * @param work Verification retry work wihin port privates preemption context.
 */
static void deipce_preempt_verify_retry_work(struct work_struct *work)
{
    struct deipce_preempt *preempt =
        container_of(work, struct deipce_preempt, verify.retry_work.work);
    struct deipce_port_priv *pp =
        container_of(preempt, struct deipce_port_priv, preempt);

    netdev_dbg(pp->netdev, "%s() preempt: retry\n", __func__);

    mutex_lock(&preempt->mgmt.lock);

    deipce_preempt_update_verification(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Set minimum preemptable fragment size.
 * @param pp Port privates.
 * @param frag_size Minimum preemptable fragment size code,
 * fragment size in bytes is (frag_size + 1) x 64x - 4 (i.e. without mCRC).
 */
int deipce_preempt_set_min_frag_size(struct deipce_port_priv *pp,
                                     unsigned int frag_size)
{
    struct deipce_preempt *preempt = &pp->preempt;

    if (frag_size > PORT_TX_PREE1_FRAG_252)
        return -EINVAL;

    mutex_lock(&preempt->mgmt.lock);

    deipce_write_port_reg(pp, PORT_REG_TX_PREE1, frag_size);
    deipce_preempt_update_advances(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return 0;
}

/**
 * Get minimum preemptable fragment size.
 * @param pp Port privates.
 * @return Minimum preemptable fragment size code,
 * fragment size in bytes is (frag_size + 1) x 64x - 4 (i.e. without mCRC).
 */
unsigned int deipce_preempt_get_min_frag_size(struct deipce_port_priv *pp)
{
    unsigned int frag_size = deipce_read_port_reg(pp, PORT_REG_TX_PREE1);

    return frag_size & PORT_TX_PREE1_FRAG_MASK;
}

/**
 * Enable or disable preemption of priority queues.
 * @param pp Port privates.
 * @param preemptable_mask Bit mask of priority queues on which to enable
 * preemption.
 * @param express_mask Bit mask of priority queues on which to disable
 * preemption.
 */
void deipce_preempt_set_enable(struct deipce_port_priv *pp,
                               unsigned int preemptable_mask,
                               unsigned int express_mask)
{
    struct deipce_preempt *preempt = &pp->preempt;
    struct deipce_dev_priv *dp = pp->dp;

    netdev_dbg(pp->netdev, "%s() preemptable 0x%x express 0x%x\n",
               __func__, preemptable_mask, express_mask);

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.tx_prio_enable |=
        preemptable_mask & ((1u << dp->features.prio_queues) - 1u);
    preempt->mgmt.tx_prio_enable &= ~express_mask;
    deipce_preempt_update_verification(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Get preemption of priority queues.
 * @param pp Port privates.
 * @return Bit mask of priority queues on which to enable preemption.
 */
unsigned int deipce_preempt_get_enable(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned int prio_mask;

    mutex_lock(&preempt->mgmt.lock);
    prio_mask = preempt->mgmt.tx_prio_enable;
    mutex_unlock(&preempt->mgmt.lock);

    return prio_mask;
}

/**
 * Enable or disable preemption on port.
 * @param pp Port privates.
 * @param enable True to enable preemption.
 */
void deipce_preempt_set_enable_port(struct deipce_port_priv *pp, bool enable)
{
    struct deipce_preempt *preempt = &pp->preempt;

    netdev_dbg(pp->netdev, "%s() %s\n",
               __func__, enable ? "enable" : "disable");

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.tx_enable = enable;
    deipce_preempt_update_verification(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Get preemption of priority queues.
 * @param pp Port privates.
 * @return Bit mask of priority queues on which to enable preemption.
 */
bool deipce_preempt_get_enable_port(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    bool enable;

    mutex_lock(&preempt->mgmt.lock);
    enable = preempt->mgmt.tx_enable;
    mutex_unlock(&preempt->mgmt.lock);

    return enable;
}

/**
 * Get preemption status of port.
 * @param pp Port privates.
 * @return True if preemption is enabled.
 */
bool deipce_preempt_get_status_port(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    uint16_t active = 0;

    if (dp->features.preempt_ports & (1u << pp->port_num)) {
        active = deipce_read_port_reg(pp, PORT_REG_TX_PREE0);
        active &= (1u << dp->features.prio_queues) - 1u;
    }

    return active != 0;
}

/**
 * Enable or disable preemption verification.
 * @param pp Port privates.
 * @param enable Whether or not to enable verification.
 */
void deipce_preempt_set_verify(struct deipce_port_priv *pp, bool enable)
{
    struct deipce_preempt *preempt = &pp->preempt;

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.enable_verify = enable;
    deipce_preempt_update_verification(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Get preemption verification setting.
 * @param pp Port privates.
 * @return Whether or not verification is enabled.
 */
bool deipce_preempt_get_verify(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    bool enable;

    mutex_lock(&preempt->mgmt.lock);
    enable = preempt->mgmt.enable_verify;
    mutex_unlock(&preempt->mgmt.lock);

    return enable;
}

/**
 * Set preemption verification interval.
 * @param pp Port privates.
 * @param ms Time in milliseconds.
 */
int deipce_preempt_set_verify_time(struct deipce_port_priv *pp,
                                   unsigned int ms)
{
    struct deipce_preempt *preempt = &pp->preempt;

    if (ms < DEIPCE_PREEMPT_VERIFY_TIME_MIN ||
        ms > DEIPCE_PREEMPT_VERIFY_TIME_MAX)
        return -EINVAL;

    mutex_lock(&preempt->mgmt.lock);

    // Stop timer temporarily.
    hrtimer_cancel(&preempt->verify.timer);

    deipce_preempt_timer_setup(preempt, ms);

    // Continue timer if it still should be running.
    if (preempt->verify.status == DEIPCE_PREEMPT_VERIFY_VERIFYING)
        deipce_preempt_timer_start(preempt);

    mutex_unlock(&preempt->mgmt.lock);

    return 0;
}

/**
 * Get preemption verification interval.
 * @param pp Port privates.
 * @return Verification time interval in milliseconds.
 */
unsigned int deipce_preempt_get_verify_time(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned int ms;

    mutex_lock(&preempt->mgmt.lock);
    ms = preempt->mgmt.interval/NSEC_PER_MSEC;
    mutex_unlock(&preempt->mgmt.lock);

    return ms;
}

/**
 * Get current hold advance in nanoseconds.
 * @param pp Port privates.
 */
uint32_t deipce_preempt_get_hold_advance(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    uint32_t advance;

    mutex_lock(&preempt->mgmt.lock);
    advance = preempt->mgmt.hold_advance;
    mutex_unlock(&preempt->mgmt.lock);

    return advance;
}

/**
 * Get current release advance in nanoseconds.
 * @param pp Port privates.
 */
uint32_t deipce_preempt_get_release_advance(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    uint32_t advance;

    mutex_lock(&preempt->mgmt.lock);
    advance = preempt->mgmt.release_advance;
    mutex_unlock(&preempt->mgmt.lock);

    return advance;
}

/**
 * Update preemption verification state machine with new link status.
 * @param pp Port privates.
 * @parm link True if link became up, false when link was lost.
 */
void deipce_preempt_update_link(struct deipce_port_priv *pp,
                                enum link_mode link_mode)
{
    struct deipce_preempt *preempt = &pp->preempt;

    netdev_dbg(pp->netdev, "preempt: link %i status %u count %u/%u\n",
               link_mode, preempt->verify.status,
               preempt->verify.count, DEIPCE_PREEMPT_VERIFY_LIMIT);

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.link_mode = link_mode;
    deipce_preempt_update_advances(pp);
    deipce_preempt_update_verification(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Get current preemption verification status.
 * @param pp Port privates.
 * @return Preemption verification status.
 */
enum deipce_preempt_verify_status deipce_preempt_get_status(
        struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned long int flags;
    enum deipce_preempt_verify_status status;

    spin_lock_irqsave(&preempt->verify.lock, flags);
    status = pp->preempt.verify.status;
    spin_unlock_irqrestore(&preempt->verify.lock, flags);

    return status;
}

/**
 * Verify frame contents match a verify mPacket.
 * @param skb Frame.
 * @return True if frame actually is a verify mPacket.
 */
static bool deipce_preempt_is_verify_skb(struct sk_buff *skb)
{
    const void *data = eth_hdr(skb);

    if (skb->len != ETH_ZLEN)
        return false;

    return memcmp(data, deipce_preempt_verify_frame, skb->len) == 0;
}

/**
 * Handle received SMD-V frames.
 * @param pp Receiving port.
 * @param skb Received frame.
 */
static void deipce_preempt_handle_smdv(struct deipce_port_priv *pp,
                                       struct sk_buff *skb)
{
    struct deipce_dev_priv *dp = pp->dp;
    unsigned int offset = skb->data - skb_mac_header(skb);

    // Get back the link layer header.
    skb_push(skb, offset);

    netdev_dbg(pp->netdev, "preempt: RX SMD-V len %u status %u\n",
               skb->len, pp->preempt.verify.status);
#ifdef DEBUG_PREEMPT_FRAMES
    netdev_dbg(pp->netdev,
               "%s() skb len %u new head %p mac %p data %p tail %p end %p\n",
               __func__, skb->len, skb->head, skb_mac_header(skb),
               skb->data, skb_tail_pointer(skb), skb_end_pointer(skb));
#endif

    if (!deipce_preempt_is_verify_skb(skb)) {
        netdev_dbg(pp->netdev, "Corrupt SMD-V frame length %u\n", skb->len);
        return;
    }

    skb = deipce_preempt_make_frame(pp, DEIPCE_PREEMPT_SMDR);
    if (!skb)
        return;

    // Send it.
    skb->dev = dp->real_netdev;
    dev_queue_xmit(skb);

    return;
}

/**
 * Handle received SMD-R frames.
 * @param pp Receiving port.
 * @param skb Received frame.
 */
static void deipce_preempt_handle_smdr(struct deipce_port_priv *pp,
                                       struct sk_buff *skb)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned int offset = skb->data - skb_mac_header(skb);
    unsigned long int flags;

    // Get back the link layer header.
    skb_push(skb, offset);

    netdev_dbg(pp->netdev, "preempt: RX SMD-R len %u status %u\n",
               skb->len, preempt->verify.status);
#ifdef DEBUG_PREEMPT_FRAMES
    print_hex_dump_bytes("RX SMD-R: ", DUMP_PREFIX_OFFSET,
                         &skb->data[0], skb->len);
#endif

    if (!deipce_preempt_is_verify_skb(skb)) {
        netdev_dbg(pp->netdev, "Corrupt SMD-R frame length %u\n", skb->len);
        return;
    }

    spin_lock_irqsave(&preempt->verify.lock, flags);

    if (preempt->verify.status == DEIPCE_PREEMPT_VERIFY_VERIFYING) {
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_SUCCEEDED;
        deipce_write_port_reg(pp, PORT_REG_TX_PREE0,
                              preempt->mgmt.tx_prio_enable);
    }

    spin_unlock_irqrestore(&preempt->verify.lock, flags);

    return;
}

/**
 * Check and handle received preemption verification frames.
 * @param pp Port privates.
 * @param rx_frame Received frame.
 * @param trailer Management trailer.
 * @return True if frame was a preemption verification frame and
 * should be freed, false otherwise.
 */
bool deipce_preempt_rx_frame(struct deipce_port_priv *pp,
                             struct sk_buff *rx_frame, uint16_t trailer)
{
    struct deipce_dev_priv *dp = pp->dp;
    enum deipce_preempt_smd smd = deipce_preempt_trailer_to_smd(dp, trailer);

    switch (smd) {
    case DEIPCE_PREEMPT_SFD: return false;
    case DEIPCE_PREEMPT_SMDV: deipce_preempt_handle_smdv(pp, rx_frame); break;
    case DEIPCE_PREEMPT_SMDR: deipce_preempt_handle_smdr(pp, rx_frame); break;
    }

    return true;
}

/**
 * Tasklet callback function to send SMD-V frames.
 * @param arg Port privates.
 */
static void deipce_preempt_send_action(unsigned long int arg)
{
    struct deipce_port_priv *pp = (struct deipce_port_priv *)arg;

    deipce_preempt_send_smdv(pp);
    return;
}

/**
 * Initialize port preemption state information.
 * @param preempt Preemption context.
 */
void deipce_preempt_init(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;

    netdev_dbg(pp->netdev, "preempt: init\n");

    *preempt = (struct deipce_preempt){
        .mgmt = {
            // Verification is enabled by default.
            .enable_verify = true,
        },
        .verify = {
            .status = DEIPCE_PREEMPT_VERIFY_INITIAL,
        },
    };
    mutex_init(&preempt->mgmt.lock);

    spin_lock_init(&preempt->verify.lock);
    tasklet_init(&preempt->verify.send_tasklet, &deipce_preempt_send_action,
                 (unsigned long int)pp);
    hrtimer_init(&preempt->verify.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    preempt->verify.timer.function = &deipce_preempt_verify_timer;
    // Default verification time is 10 ms.
    deipce_preempt_timer_setup(preempt, 10);

    INIT_DELAYED_WORK(&preempt->verify.retry_work,
                      &deipce_preempt_verify_retry_work);

    deipce_write_port_reg(pp, PORT_REG_TX_PREE0, 0);

    deipce_preempt_sysfs_init(pp);

    return;
}

/**
 * Reset preemption state machine.
 * This must be used when bringing interface administratively down.
 * @param pp Port privates.
 */
void deipce_preempt_reset(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned long int flags;

    mutex_lock(&preempt->mgmt.lock);

    cancel_delayed_work_sync(&preempt->verify.retry_work);
    hrtimer_cancel(&preempt->verify.timer);

    spin_lock_irqsave(&preempt->verify.lock, flags);

    if (preempt->mgmt.enable_verify)
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_INITIAL;
    else
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_DISABLED;
    preempt->verify.count = 0;

    spin_unlock_irqrestore(&preempt->verify.lock, flags);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Cleanup preemption state machine.
 * @param pp Port privates.
 */
void deipce_preempt_cleanup(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;

    netdev_dbg(pp->netdev, "preempt: cleanup\n");

    deipce_preempt_sysfs_cleanup(pp);
    cancel_delayed_work_sync(&preempt->verify.retry_work);
    hrtimer_cancel(&preempt->verify.timer);

    mutex_destroy(&preempt->mgmt.lock);

    return;
}
