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
#include "deipce_preempt.h"

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
uint16_t deipce_preempt_smd_to_trailer(struct deipce_dev_priv *dp,
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
enum deipce_preempt_smd deipce_preempt_trailer_to_smd(
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
               pp->preempt.mgmt.verify_limit);

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
static void inline deipce_preempt_timer_setup(struct deipce_preempt *preempt,
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
static void deipce_preempt_start_verification(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned long flags;
    bool timer_should_run;
    uint16_t active = 0;

    netdev_dbg(pp->netdev, "preempt: update prio 0x%02x %s link %s\n",
               preempt->mgmt.tx_prio_enable,
               preempt->mgmt.enable_verify ? "verify" : "",
               preempt->mgmt.link ? "UP" : "DOWN");

    hrtimer_cancel(&preempt->verify.timer);

    spin_lock_irqsave(&preempt->verify.lock, flags);

    if (preempt->mgmt.tx_prio_enable != 0 && !preempt->mgmt.enable_verify)
        active = preempt->mgmt.tx_prio_enable;
    deipce_write_port_reg(pp, PORT_REG_TX_PREE0, active);

    timer_should_run = 
        preempt->mgmt.tx_prio_enable != 0 &&
        preempt->mgmt.enable_verify &&
        preempt->mgmt.link;
    if (timer_should_run) {
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_IN_PROGRESS;
        preempt->verify.count = 1;
    }
    else {
        if (preempt->mgmt.enable_verify)
            preempt->verify.status = DEIPCE_PREEMPT_VERIFY_IDLE;
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
    if (preempt->verify.status != DEIPCE_PREEMPT_VERIFY_IN_PROGRESS) {
        spin_unlock_irqrestore(&preempt->verify.lock, flags);
        return HRTIMER_NORESTART;
    }

    if (preempt->mgmt.verify_limit > 0 &&
        preempt->verify.count >= preempt->mgmt.verify_limit) {
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_FAILED;
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
 * Set minimum preemptable fragment size.
 * @param pp Port privates.
 * @param size Minimum preemptable fragment size in bytes including FSC.
 */
int deipce_preempt_set_min_frag_size(struct deipce_port_priv *pp,
                                     unsigned int size)
{
    struct deipce_preempt *preempt = &pp->preempt;

    switch (size) {
    case 64: size = PORT_TX_PREE1_FRAG_60; break;
    case 128: size = PORT_TX_PREE1_FRAG_124; break;
    case 192: size = PORT_TX_PREE1_FRAG_188; break;
    case 256: size = PORT_TX_PREE1_FRAG_252; break;
    default: return -EINVAL;
    }

    mutex_lock(&preempt->mgmt.lock);

    deipce_write_port_reg(pp, PORT_REG_TX_PREE1, size);

    mutex_unlock(&preempt->mgmt.lock);

    return 0;
}

/**
 * Get minimum preemptable fragment size.
 * @param pp Port privates.
 * @return Minimum preemptable fragment size in bytes including FSC.
 */
unsigned int deipce_preempt_get_min_frag_size(struct deipce_port_priv *pp)
{
    unsigned int size = deipce_read_port_reg(pp, PORT_REG_TX_PREE1);

    size &= PORT_TX_PREE1_FRAG_MASK;
    size++;
    size *= 64;

    return size;
}

/**
 * Enable or disable preemption of priority queues.
 * @param pp Port privates.
 * @param prio_mask Bit mask of priority queues on which to enable preemption.
 */
void deipce_preempt_set_enable(struct deipce_port_priv *pp,
                               unsigned int prio_mask)
{
    struct deipce_dev_priv *dp = pp->dp;
    struct deipce_preempt *preempt = &pp->preempt;

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.tx_prio_enable =
        prio_mask & ((1u << dp->features.prio_queues) -1u);
    deipce_preempt_start_verification(pp);

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
 * Enable or disable preemption verification.
 * @param pp Port privates.
 * @param enable Whether or not to enable verification.
 */
void deipce_preempt_set_verify(struct deipce_port_priv *pp, bool enable)
{
    struct deipce_preempt *preempt = &pp->preempt;

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.enable_verify = enable;
    deipce_preempt_start_verification(pp);

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
void deipce_preempt_set_verify_time(struct deipce_port_priv *pp,
                                    unsigned int ms)
{
    struct deipce_preempt *preempt = &pp->preempt;

    mutex_lock(&preempt->mgmt.lock);

    // Stop timer temporarily.
    hrtimer_cancel(&preempt->verify.timer);

    deipce_preempt_timer_setup(preempt, ms);

    // Continue timer if it still should be running.
    if (preempt->verify.status == DEIPCE_PREEMPT_VERIFY_IN_PROGRESS)
        deipce_preempt_timer_start(preempt);

    mutex_unlock(&preempt->mgmt.lock);

    return;
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
 * Set preemption verification counter limit.
 * @param pp Port privates.
 * @param limit Counter limit or zero for no limit.
 */
void deipce_preempt_set_verify_limit(struct deipce_port_priv *pp,
                                     unsigned int limit)
{
    struct deipce_preempt *preempt = &pp->preempt;

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.verify_limit = limit;
    deipce_preempt_start_verification(pp);

    mutex_unlock(&preempt->mgmt.lock);

    return;
}

/**
 * Get preemption verification counter limit.
 * @param pp Port privates.
 * @return Verification counter limit, zero means unlimited.
 */
unsigned int deipce_preempt_get_verify_limit(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned int limit;

    mutex_lock(&preempt->mgmt.lock);
    limit = preempt->mgmt.verify_limit;
    mutex_unlock(&preempt->mgmt.lock);

    return limit;
}

/**
 * Get current preemption verification counter value.
 * @param pp Port privates.
 * @return Verification counter value.
 */
unsigned int deipce_preempt_get_verify_count(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;
    unsigned int count;
    unsigned long flags;

    spin_lock_irqsave(&preempt->verify.lock, flags);
    count = preempt->verify.count;
    spin_unlock_irqrestore(&preempt->verify.lock, flags);

    return count;
}

/**
 * Update preemption verification state machine with new link status.
 * @param pp Port privates.
 * @parm link True if link became up, false when link was lost.
 */
void deipce_preempt_update_link(struct deipce_port_priv *pp, bool link)
{
    struct deipce_preempt *preempt = &pp->preempt;

    netdev_dbg(pp->netdev, "preempt: link %s status %u count %u/%u\n",
               link ? "UP" : "DOWN", preempt->verify.status,
               preempt->verify.count, preempt->mgmt.verify_limit);

    mutex_lock(&preempt->mgmt.lock);

    preempt->mgmt.link = link;
    deipce_preempt_start_verification(pp);

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

    if (preempt->verify.status == DEIPCE_PREEMPT_VERIFY_IN_PROGRESS) {
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_OK;
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
 * Initialization port preemption state information.
 * @param preempt Preemption context.
 */
void deipce_preempt_init(struct deipce_port_priv *pp)
{
    struct deipce_preempt *preempt = &pp->preempt;

    netdev_dbg(pp->netdev, "preempt: init\n");

    *preempt = (struct deipce_preempt){
        .mgmt = {
            // Number of verifications is defined to be constant 3.
            .verify_limit = 3,
        },
        .verify = {
            .status = DEIPCE_PREEMPT_VERIFY_DISABLED,
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

    if (pp->dp->features.preempt_ports & (1u << pp->port_num)) {
        // Verification is enabled by default.
        preempt->mgmt.enable_verify = true;
        preempt->verify.status = DEIPCE_PREEMPT_VERIFY_IDLE;

        deipce_write_port_reg(pp, PORT_REG_TX_PREE0, 0);
    }

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

    hrtimer_cancel(&preempt->verify.timer);
    mutex_destroy(&preempt->mgmt.lock);

    return;
}
