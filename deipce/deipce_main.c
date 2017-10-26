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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/hardirq.h>
#include <linux/if_vlan.h>
#include <linux/of.h>
#include <linux/of_net.h>

#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_mmio.h"
#include "deipce_netdev.h"
#include "deipce_hw.h"
#include "deipce_netdevif.h"
#include "deipce_switchdev.h"
#include "deipce_clock_main.h"
#include "deipce_clock_ptp.h"
#include "deipce_fsc_main.h"
#include "deipce_fpts_main.h"
#include "deipce_ibc_main.h"
#include "deipce_debugfs.h"
#include "deipce_main.h"

// Module description information
MODULE_DESCRIPTION("DE-IP Core Edge driver");
MODULE_AUTHOR("TTTech Flexibilis Oy");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);

// Module parameters

/// bitmask of devices, 1 means use port timestamper (for 802.1AS)
static unsigned int port_ts = ~0u;
module_param(port_ts, uint, S_IRUGO);
MODULE_PARM_DESC(port_ts,
                 "Select between 802.1AS (1) and 1588 (0)"
                 " (bitmask of switch devices, 802.1AS is the default)");

// Driver private data
static struct deipce_drv_priv deipce_drv_priv = {
    .wq_low = NULL,
};

/**
 * Get access to driver privates.
 */
struct deipce_drv_priv *deipce_get_drv_priv(void)
{
    return &deipce_drv_priv;
}

/**
 * Init device private data.
 * @param dp Device private data
 * @param id Device id
 */
static void init_dev_privates(struct deipce_dev_priv *dp, int id)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    int i;

    dp->dev_num = id;
    drv->dev_priv[id] = dp;

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        dp->port[i] = NULL;
    }

    spin_lock_init(&dp->rx_stamper.lock);
    mutex_init(&dp->common_reg_lock);
    mutex_init(&dp->smac_table_lock);

    return;
}

#ifdef CONFIG_OF

/**
 * Get switch features from device tree.
 * @param dp Device privates whose features to set.
 */
static int deipce_of_get_features(struct deipce_dev_priv *dp)
{
    struct device_node *frs_node = dp->this_dev->of_node;
    struct device_node *node = NULL;
    uint32_t value = 0;

    // Older kernels do not have of_get_child_by_name.
    for_each_child_of_node(frs_node, node) {
        if (strcmp("features", node->name) == 0)
            break;
    }

    if (!node) {
        // features node is missing.
        return -ENOENT;
    }

    if (of_property_read_u32(node, "clock-frequency", &value) == 0)
        dp->features.clock_freq = value;
    else
        dp->features.clock_freq = 0;

    dp->features.flags = 0;
    if (of_property_read_bool(node, "gigabit"))
        dp->features.flags |= FLX_FRS_FEAT_GIGABIT;
    if (of_property_read_bool(node, "statistics-counters"))
        dp->features.flags |= FLX_FRS_FEAT_STATS;
    if (of_property_read_bool(node, "mac-address-table"))
        dp->features.flags |= FLX_FRS_FEAT_MAC_TABLE;
    if (of_property_read_bool(node, "vlan"))
        dp->features.flags |= FLX_FRS_FEAT_VLAN;
    if (of_property_read_bool(node, "traffic-shaper"))
        dp->features.flags |= FLX_FRS_FEAT_SHAPER;

    if (of_property_read_u32(node, "priority-queues", &value) == 0)
        dp->features.prio_queues = value;
    else
        dp->features.prio_queues = 0;
    if (of_property_read_u32(node, "traffic-policers", &value) == 0)
        dp->features.policers = value;
    else
        dp->features.policers = 0;

    if (of_property_read_u32(node,
                             "static-mac-address-table-rows",
                             &value) == 0)
        dp->features.smac_rows = value;
    else
        dp->features.smac_rows = 0;

    if (of_property_read_u32(node, "hsr-ports", &value) == 0)
        dp->features.hsr_ports = value;
    else
        dp->features.hsr_ports = 0;
    if (of_property_read_u32(node, "prp-ports", &value) == 0)
        dp->features.prp_ports = value;
    else
        dp->features.prp_ports = 0;
    if (of_property_read_u32(node, "macsec-ports", &value) == 0)
        dp->features.macsec_ports = value;
    else
        dp->features.macsec_ports = 0;
    if (of_property_read_u32(node, "scheduled-ports", &value) == 0)
        dp->features.sched_ports = value;
    else
        dp->features.sched_ports = 0;
    if (of_property_read_u32(node, "timestamper-ports", &value) == 0)
        dp->features.ts_ports = value;
    else
        dp->features.ts_ports = 0;
    if (of_property_read_u32(node, "cut-through-ports", &value) == 0)
        dp->features.ct_ports = value;
    else
        dp->features.ct_ports = 0;
    if (of_property_read_u32(node, "preemptable-ports", &value) == 0)
        dp->features.preempt_ports = value;
    else
        dp->features.preempt_ports = 0;

    return 0;
}

#endif

/**
 * Sanitize switch features.
 * @param dp FRS device privates.
 */
static void deipce_sanitize_features(struct deipce_dev_priv *dp)
{
    unsigned int column;

    if (dp->features.prio_queues > DEIPCE_MAX_PRIO_QUEUES)
        dp->features.prio_queues = DEIPCE_MAX_PRIO_QUEUES;
    if (dp->features.prio_queues == 0)
        dp->features.flags &= ~FLX_FRS_FEAT_SHAPER;

    if (dp->features.policers > DEIPCE_MAX_POLICERS)
        dp->features.policers = DEIPCE_MAX_POLICERS;

    if (dp->features.smac_rows > FRS_SMAC_TABLE_MAX_ROWS)
        dp->features.smac_rows = FRS_SMAC_TABLE_MAX_ROWS;
    for (column = 0; column < FRS_SMAC_TABLE_COLS; column++) {
        dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_NO_VLAN;
    }

    return;
}

/**
 * Get PHY delays from device tree.
 * @param node Device tree node with "phy-delay" property.
 * @param delay Array of delays, enum link_mode as array index.
 */
static int deipce_of_get_phy_delays(struct deipce_dev_priv *dp,
                                    struct deipce_port_cfg *port_cfg,
                                    struct device_node *node)
{
    // Triplettes of speed, TX-delay and RX-delay.
    const unsigned int values_per_delay = 3;
    const unsigned int delay_size = values_per_delay * sizeof(uint32_t);
    int length = 0;
    const __be32 *values;
    unsigned int delay_count;
    unsigned int value_index = 0;
    unsigned int speed;
    enum link_mode link_mode;
    struct deipce_delay *delay;
    unsigned int delay_num;

    // Delays are optional.
    values = of_get_property(node, "phy-delay", &length);
    if (!values)
        return 0;

    if (length % delay_size)
        return -EINVAL;

    delay_count = length / delay_size;
    for (delay_num = 0; delay_num < delay_count; delay_num++) {
        speed = be32_to_cpu(values[value_index + 0]);
        switch (speed) {
        case 1000: link_mode = LM_1000FULL; break;
        case 100: link_mode = LM_100FULL; break;
        case 10: link_mode = LM_10FULL; break;
        case 0: link_mode = LM_DOWN; break;
        default:
            return -EINVAL;
        }

        delay = &port_cfg->phy_delay[link_mode];
        delay->tx = be32_to_cpu(values[value_index + 1]);
        delay->rx = be32_to_cpu(values[value_index + 2]);

        dev_dbg(dp->this_dev, "%s() PHY delays @%u Mbps TX %lu RX %lu\n",
                __func__, speed, delay->tx, delay->rx);

        value_index += values_per_delay;
    }

    port_cfg->phy_delay[LM_DOWN] = port_cfg->phy_delay[LM_10FULL];

    return 0;
}

/**
 * Configure FRS device.
 * @param dp FRS device privates.
 * @param pdev FRS platform_device.
 * @param frs_cfg Temporary storage for building FRS config.
 */
static int deipce_device_config(struct deipce_dev_priv *dp,
                                struct platform_device *pdev,
                                struct deipce_cfg *frs_cfg)
{
#ifdef CONFIG_OF
    struct resource *res = NULL;
    struct device *dev = &pdev->dev;
    struct device_node *child_n = NULL;
    const __be32 *reg = NULL;
    int length = 0;
    const char *svalue = NULL;
    struct resource *irq_res =
        platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    int ret = 0;

    if (!irq_res) {
        // Allow operation without IRQ.
        dev_warn(dp->this_dev, "No IRQ defined\n");
        dp->irq = 0;
    }
    else {
        dp->irq = irq_res->start;
    }

    ret = deipce_of_get_features(dp);
    if (ret == -ENOENT) {
        dev_warn(dev, "Missing features in device tree\n");
    }

    // Underlying Ethernet MAC name
    if (of_property_read_string(dev->of_node, "mac_name", &svalue)) {
        dev_printk(KERN_DEBUG, dp->this_dev, "unable to get MAC name\n");
        return -ENODEV;
    }
    frs_cfg->mac_name = svalue;

    // Default PTP hardware clock for all ports and endpoint
    frs_cfg->phc_node = of_parse_phandle(dev->of_node, "ptp-clock", 0);
    if (frs_cfg->phc_node) {
        dp->phc.info = deipce_clock_of_get_phc(frs_cfg->phc_node);
        if (!dp->phc.info)
            dev_warn(dp->this_dev, "Failed to get PHC\n");
        dp->phc.index = deipce_clock_get_phc_index(dp->phc.info);
        of_node_put(frs_cfg->phc_node);
    }

    frs_cfg->ibc_node = of_parse_phandle(dev->of_node, "ibc", 0);
    if (frs_cfg->ibc_node) {
        dp->ibc = deipce_ibc_of_get_device_by_node(frs_cfg->ibc_node);
        if (!dp->ibc)
            dev_warn(dp->this_dev, "Failed to get IBC\n");
        of_node_put(frs_cfg->ibc_node);
    }

    // Register access
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(dev, "No I/O memory defined\n");
        return -ENODEV;
    }
    else {
        frs_cfg->baseaddr = res->start;
    }

    // Interface name for switch
    if (of_property_read_string(dev->of_node, "if_name", &svalue) == 0) {
        frs_cfg->if_name = svalue;
    }

    // Config all ports
    frs_cfg->num_of_ports = 0;
    for_each_child_of_node(dev->of_node, child_n) {
        struct deipce_port_cfg *port_cfg = NULL;
        struct of_phandle_args args = { .np = NULL };
        int port_num = -1;
        int phy_mode = -1;

        if (strncmp("port", child_n->name, 4) != 0) {
            continue;
        }

        if (kstrtoint(child_n->name + 4, 10, &port_num) ||
            port_num < 0 || port_num >= ARRAY_SIZE(frs_cfg->port)) {
            dev_warn(dp->this_dev, "Invalid port %s\n",
                     child_n->name);
            continue;
        }

        port_cfg = &frs_cfg->port[port_num];

        if (port_cfg->medium_type != DEIPCE_MEDIUM_NONE) {
            dev_warn(dp->this_dev, "Already defined port %i\n",
                     port_num);
            continue;
        }

        dev_dbg(dp->this_dev, "Reading node %s index %i port %i\n",
                child_n->name, port_num, port_num);

        // Interface name for port
        if (of_property_read_string(child_n, "if_name", &svalue) == 0) {
            port_cfg->if_name = svalue;
        }

        if (of_property_read_bool(child_n, "cpu-port")) {
            port_cfg->flags |= DEIPCE_PORT_CPU;
        }
        if (of_property_read_bool(child_n, "auto-speed-select")) {
            port_cfg->flags |= DEIPCE_PORT_SPEED_EXT;
        }

        // Port and port adapter register addresses
        reg = of_get_property(child_n, "reg", &length);
        if (!reg ||
            (length != 2*sizeof(uint32_t) &&
             length != 4*sizeof(uint32_t))) {
            dev_printk(KERN_DEBUG, dp->this_dev,
                       "unable to get port %i"
                       " memory mapped I/O addresses\n",
                       port_num);
        } else {
            port_cfg->baseaddr = be32_to_cpu(reg[0]);
            if (length == 16) {
                port_cfg->adapter_baseaddr = be32_to_cpu(reg[2]);
                port_cfg->flags |=
                    DEIPCE_PORT_ADDR_VALID |
                    DEIPCE_ADAPTER_ADDR_VALID;
            }
            else {
                port_cfg->flags |= DEIPCE_PORT_ADDR_VALID;
            }
        }

        // Default medium type
        port_cfg->medium_type = DEIPCE_MEDIUM_NOPHY;

        // Ext PHY
        port_cfg->ext_phy_node = of_parse_phandle(child_n,
                                                  "phy-handle", 0);
        port_cfg->ext_phy_if = PHY_INTERFACE_MODE_NA;
        if (port_cfg->ext_phy_node) {
            port_cfg->medium_type = DEIPCE_MEDIUM_PHY;
            port_cfg->flags |= DEIPCE_HAS_PHY;

            phy_mode = of_get_phy_mode(child_n);
            if (phy_mode >= 0)
                port_cfg->ext_phy_if = phy_mode;
        }

        // Ext PHY delays
        ret = deipce_of_get_phy_delays(dp, port_cfg, child_n);
        if (ret) {
            dev_warn(dp->this_dev, "Invalid PHY delays for port %u\n",
                     port_num);
        }

        // SFP EEPROM for SFP type detection
        port_cfg->sfp_eeprom_node = of_parse_phandle(child_n,
                                                     "sfp-eeprom", 0);
        if (port_cfg->sfp_eeprom_node) {
            port_cfg->medium_type = DEIPCE_MEDIUM_SFP;
            port_cfg->flags |= DEIPCE_SFP_EEPROM;
        }

        // SFP PHY, always use SGMII with SFP PHY.
        port_cfg->sfp_phy_if = PHY_INTERFACE_MODE_SGMII;
        port_cfg->sfp_phy_node = of_parse_phandle(child_n,
                                                  "sfp-phy-handle", 0);
        if (port_cfg->sfp_phy_node) {
            port_cfg->medium_type = DEIPCE_MEDIUM_SFP;
            port_cfg->flags |= DEIPCE_HAS_SFP_PHY;
        }

        // SGMII mode for SGMII adapter
        if (of_property_read_bool(child_n, "sgmii-phy-mode")) {
            port_cfg->flags |= DEIPCE_ADAPTER_SGMII_PHY_MODE;
            dev_printk(KERN_DEBUG, dp->this_dev,
                       "port %i SGMII %s mode\n",
                       port_num, "PHY");
        }

        if ((port_cfg->flags & DEIPCE_HAS_PHY) &&
            (port_cfg->flags & DEIPCE_HAS_SFP_PHY))
            port_cfg->flags |= DEIPCE_HAS_SEPARATE_SFP;

        ret = of_parse_phandle_with_fixed_args(child_n, "scheduler", 1,
                                               0, &args);
        if (ret >= 0) {
            port_cfg->fsc_node = args.np;
            port_cfg->sched_num = args.args[0];
            dp->features.sched_ports |= 1u << port_num;
        }

        if (port_num + 1 > frs_cfg->num_of_ports)
            frs_cfg->num_of_ports = port_num + 1;
    }
#endif

    deipce_sanitize_features(dp);

    return 0;
}

/**
 * Initialize device port contexts using configuration information.
 * @param dp Device privates.
 * @param frs_cfg Device config.
 */
static int deipce_init_ports(struct deipce_dev_priv *dp,
                             struct deipce_cfg *frs_cfg)
{
    int i = 0;
    struct deipce_port_priv *pp = NULL;
    struct deipce_port_cfg *port_cfg = NULL;
    struct deipce_ibc_phc_info phc_list[DEIPCE_IBC_MAX_CLOCKS] = {
        { .info = NULL },
    };
    struct deipce_fsc_dev_priv *last_fsc = NULL;
    unsigned int ibc_time_sel = dp->use_port_ts ? 1 : 0;

    dp->num_of_ports = frs_cfg->num_of_ports;

    /*
     * Determine management trailer length.
     * MACsec needs 1 bit, preemption needs 2 bits (just before MACsec bit,
     * regardless of whether or not MACsec is enabled).
     */
    dp->trailer_len = dp->num_of_ports;
    if (dp->features.preempt_ports)
        dp->trailer_len +=
            DEIPCE_TRAILER_MACSEC_BITS + DEIPCE_TRAILER_PREEMPT_BITS;
    else if (dp->features.macsec_ports)
        dp->trailer_len += DEIPCE_TRAILER_MACSEC_BITS;
    dp->trailer_len = (dp->trailer_len + 7) / 8;
    if (dp->trailer_len > sizeof(uint16_t)) {
        dp->trailer_len = sizeof(uint16_t);
        dev_warn(dp->this_dev, "Too many ports\n");
    }

    if (dp->ibc)
        deipce_ibc_get_clocks(dp->ibc, phc_list, ARRAY_SIZE(phc_list),
                              NULL, NULL);

    for (i = 0; i < frs_cfg->num_of_ports; i++) {
        port_cfg = &frs_cfg->port[i];

        // Ignore unconfigured ports.
        if (port_cfg->medium_type == DEIPCE_MEDIUM_NONE)
            continue;

        // Port addresses are required.
        if (!(port_cfg->flags & DEIPCE_PORT_ADDR_VALID))
            continue;

        pp = kmalloc(sizeof(*pp), GFP_KERNEL);
        if (!pp) {
            dev_err(dp->this_dev, "kmalloc failed\n");
            goto fail;
        }

        *pp = (struct deipce_port_priv){
            .dp = dp,
            .port_num = i,
            .medium_type = port_cfg->medium_type,
            .flags = port_cfg->flags,
            .ext_phy.node = port_cfg->ext_phy_node,
            .sfp.phy.node = port_cfg->sfp_phy_node,
            .sfp.eeprom_node = port_cfg->sfp_eeprom_node,

            .ext_phy.interface = port_cfg->ext_phy_if,
            .sfp.phy.interface = port_cfg->sfp_phy_if,
            .sfp.supported = DEIPCE_ETHTOOL_SUPPORTED,

            .mgmt_prio =
                PORT_ETH_ADDR_PRESERVE_PRIORITY |
                PORT_ETH_ADDR_FROM_PRIO(0),
            .rx_delay = 0,
            .tx_delay = 0,
            .p2p_delay = 0,

            .sched = {
                .fsc = deipce_fsc_of_get_device_by_node(port_cfg->fsc_node),
                .num = port_cfg->sched_num,
            },

            // Management trailer for sending.
            .trailer = 1u << i,
        };

        if (port_cfg->fsc_node) {
            of_node_put(port_cfg->fsc_node);
            if (pp->sched.fsc && pp->sched.fsc != last_fsc) {
                deipce_fsc_set_clock(pp->sched.fsc,
                                     phc_list[ibc_time_sel].info);
                last_fsc = pp->sched.fsc;
            }
        }

        if (pp->flags & DEIPCE_PORT_CPU)
            dp->cpu_port_mask |= pp->trailer;

        if (dp->features.macsec_ports & pp->trailer)
            pp->trailer |= deipce_get_macsec_trailer(dp);

        memcpy(pp->ext_phy.delay, port_cfg->phy_delay,
               sizeof(port_cfg->phy_delay));

        mutex_init(&pp->stats_lock);
        mutex_init(&pp->port_reg_lock);
        spin_lock_init(&pp->tx_stamper.lock);
        dp->port[i] = pp;
    }

    return 0;

fail:
    for (i = 0; i < frs_cfg->num_of_ports; i++) {
        if (dp->port[i]) {
            kfree(dp->port[i]);
            dp->port[i] = NULL;
        }
    }

    return -ENOMEM;
}

/**
 * Do a software reset to switch.
 * @param dp Device privates.
 */
static int deipce_sw_reset(struct deipce_dev_priv *dp)
{
    unsigned int timeout = 100;
    uint16_t data;

    // SW Reset
    deipce_write_switch_reg(dp, FRS_REG_GEN, FRS_GEN_RESET);

    // Wait reset to complete.
    do {
        if (timeout-- == 0) {
            dev_err(dp->this_dev, "SW reset failed: timeout\n");
            return -EBUSY;
        }
        cpu_relax();

        data = deipce_read_switch_reg(dp, FRS_REG_GEN);
    } while (data & FRS_GEN_RESET);

    dev_printk(KERN_DEBUG, dp->this_dev, "SW reset done\n");

    data &= ~FRS_GEN_TIME_TRAILER;
    data &= ~(FRS_GEN_MGMT_TRAILER_LEN | FRS_GEN_MGMT_TRAILER_OFFSET);

    if (dp->trailer_len > 1) {
        data |= FRS_GEN_MGMT_TRAILER_LEN;
        if (dp->trailer_offset > 0)
            data |= FRS_GEN_MGMT_TRAILER_OFFSET;
    }

    deipce_write_switch_reg(dp, FRS_REG_GEN, data);

    return 0;
}

/**
 * Drop all VLAN memberships.
 * @param dp Device privates.
 */
void deipce_drop_all_vlans(struct deipce_dev_priv *dp)
{
    unsigned int vlan_id;

    for (vlan_id = 0; vlan_id <= VLAN_VID_MASK; vlan_id++) {
        deipce_write_switch_reg(dp, FRS_VLAN_CFG(vlan_id), 0);
    }

    return;
}

/**
 * Run SW reset and init default settings.
 * @param dp FRS device privates.
 */
static int deipce_init_registers(struct deipce_dev_priv *dp)
{
    struct deipce_port_priv *port = NULL;
    unsigned int port_num;
    int ret = -ENODEV;

    ret = deipce_sw_reset(dp);
    if (ret)
        return ret;

    // Init timestampers.
    if (dp->use_port_ts) {
        for (port_num = 0; port_num < DEIPCE_MAX_PORTS; port_num++) {
            if (!(dp->features.ts_ports & (1u << port_num)))
                continue;

            port = dp->port[port_num];
            if (!port)
                continue;

            port->rx_stamper = 0;
            port->tx_stamper.next = 0;
            port->tx_stamper.next_skb = 0;

            deipce_write_port_reg(port, PORT_REG_TS_CTRL_RX,
                                  (1u << PORT_TS_NUMBER_TIMESTAMPS) - 1);
            deipce_write_port_reg(port, PORT_REG_TS_CTRL_TX,
                                  (1u << PORT_TS_NUMBER_TIMESTAMPS) - 1);
        }
    }
    else if (deipce_dev_has_cpu_port(dp)) {
        dp->tx_stamper = 0;
        dp->rx_stamper.next = 0;
        dp->rx_stamper.next_skb = 0;

        deipce_write_switch_reg(dp, FRS_REG_TS_CTRL_RX,
                                (1u << FRS_TS_NUMBER_TIMESTAMPS) - 1);
        deipce_write_switch_reg(dp, FRS_REG_TS_CTRL_TX,
                                (1u << FRS_TS_NUMBER_TIMESTAMPS) - 1);
    }

    // Enable needed interrupts.
    deipce_write_switch_reg(dp, FRS_REG_INTMASK, deipce_get_intmask(dp));

    // Make sure to use driver defaults for VLAN configuration.
    deipce_drop_all_vlans(dp);

    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        port = dp->port[port_num];
        if (!port)
            continue;

        deipce_reset_port_vlan_config(port, true);

        /*
         * Use 1:1 mapping from VLAN PCP to output priority queue by default
         * when there are enough queues.
         */
        if (dp->features.prio_queues == DEIPCE_MAX_PRIO_QUEUES) {
            deipce_write_port_reg(port, PORT_REG_VLAN_PRIO,
                                  PORT_VLAN_PRIO(0, 0) |
                                  PORT_VLAN_PRIO(1, 1) |
                                  PORT_VLAN_PRIO(2, 2) |
                                  PORT_VLAN_PRIO(3, 3) |
                                  PORT_VLAN_PRIO(4, 4) |
                                  PORT_VLAN_PRIO(5, 5) |
                                  PORT_VLAN_PRIO(6, 6) |
                                  PORT_VLAN_PRIO(7, 7));
            deipce_write_port_reg(port, PORT_REG_VLAN_PRIO_HI,
                                  PORT_VLAN_PRIO_HI(0, 0) |
                                  PORT_VLAN_PRIO_HI(1, 1) |
                                  PORT_VLAN_PRIO_HI(2, 2) |
                                  PORT_VLAN_PRIO_HI(3, 3) |
                                  PORT_VLAN_PRIO_HI(4, 4) |
                                  PORT_VLAN_PRIO_HI(5, 5) |
                                  PORT_VLAN_PRIO_HI(6, 6) |
                                  PORT_VLAN_PRIO_HI(7, 7));
        }
    }

    return 0;
}

/**
 * Function to initialise FRS platform devices.
 * @param pdev Platform device
 * @return 0 on success or negative error code.
 */
static int deipce_device_init(struct platform_device *pdev)
{
    int ret = -ENOMEM;
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_dev_priv *dp = NULL;
    struct deipce_cfg *frs_cfg = NULL;
    uint32_t pdev_id = 0;

    dev_info(&pdev->dev, "Init device\n");

    frs_cfg = kmalloc(sizeof(*frs_cfg), GFP_KERNEL);
    if (!frs_cfg) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        return -ENOMEM;
    }
    *frs_cfg = (struct deipce_cfg){
        .mac_name = NULL,
    };

    // use pdev->id if provided, if only one, pdev->id == -1
    if (pdev->id >= 0) {
        pdev_id = pdev->id;
    } else {
        // With device tree pdev->id may always be -1.
        while (pdev_id < ARRAY_SIZE(drv->dev_priv)) {
            if (!drv->dev_priv[pdev_id])
                break;
            pdev_id++;
        }
    }
    if (pdev_id >= ARRAY_SIZE(drv->dev_priv)) {
        dev_err(&pdev->dev, "Too many FRS devices\n");
        ret = -ENODEV;
        goto err_too_many;
    }
    // Allocate device private
    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        ret = -ENOMEM;
        goto err_alloc;
    }

    *dp = (struct deipce_dev_priv) {
        .this_dev = &pdev->dev,
        .dev_num = pdev_id,
        .tstamp_cfg = {
            .tx_type = HWTSTAMP_TX_OFF,
            .rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT,
        },
    };
    init_dev_privates(dp, pdev_id);

    ret = deipce_device_config(dp, pdev, frs_cfg);
    if (ret) {
        dev_err(dp->this_dev, "Failed to configure device\n");
        goto err_device_config;
    }

    if (dp->features.smac_rows > 0) {
        unsigned int bits = dp->features.smac_rows * FRS_SMAC_TABLE_COLS;
        size_t alloc_size = BITS_TO_LONGS(bits) * sizeof(*dp->smac.used);
        dp->smac.used = kzalloc(alloc_size, GFP_KERNEL);
        if (!dp->smac.used) {
            dev_err(dp->this_dev, "Failed to allocate SMAC usage bitmap\n");
            ret = -ENOMEM;
            goto err_smac;
        }
    }

    // Allow use of port timestampers only when they are available.
    if (dp->features.ts_ports == 0)
        port_ts &= ~(1u << dp->dev_num);
    if (port_ts & (1u << dp->dev_num))
        dp->use_port_ts = true;
    else
        dp->use_port_ts = false;
    if (dp->ibc) {
        unsigned int time_sel = dp->use_port_ts ? 1 : 0;
        unsigned int gp_sel = dp->use_port_ts ? 1 : 0;

        deipce_ibc_set(dp->ibc, time_sel, gp_sel);
        // FSC clock is set in deipce_init_ports.
    }

    ret = deipce_init_ports(dp, frs_cfg);
    if (ret)
        goto err_init_ports;

    ret = deipce_mmio_init_device(dp, pdev, frs_cfg);
    if (ret)
        goto err_reg_access;

    ret = deipce_irq_init(dp);
    if (ret)
        goto err_irq;

    ret = deipce_init_registers(dp);
    if (ret)
        goto err_registers;

    ret = deipce_netdev_init(dp, frs_cfg);
    if (ret)
        goto err_netdev;

    ret = deipce_netdevif_init(dp);
    if (ret)
        goto err_netdevif;

    ret = deipce_switchdev_init_device(dp);
    if (ret)
        goto err_switchdev;

    ret = deipce_debugfs_init_device(dp);
    if (ret)
        goto err_debugfs;

    return 0;

err_debugfs:
err_switchdev:
    deipce_netdevif_cleanup(dp);

err_netdevif:
    deipce_netdev_cleanup(dp);

err_netdev:
err_registers:
    deipce_irq_cleanup(dp);

err_irq:
    deipce_mmio_cleanup_device(dp);

err_reg_access:
err_init_ports:
    if (dp->smac.used) {
        kfree(dp->smac.used);
        dp->smac.used = NULL;
    }

err_smac:
err_device_config:
    drv->dev_priv[pdev_id] = NULL;
    kfree(dp);

err_alloc:
err_too_many:
    kfree(frs_cfg);

    return ret;
}

/**
 * Function to clean device data
 */
static void deipce_device_cleanup(struct deipce_dev_priv *dp)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    unsigned int i;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    deipce_irq_cleanup(dp);
    deipce_debugfs_cleanup_device(dp);
    deipce_switchdev_cleanup_device(dp);
    deipce_netdevif_cleanup(dp);
    deipce_netdev_cleanup(dp);

    if (dp->smac.used) {
        kfree(dp->smac.used);
        dp->smac.used = NULL;
    }

    deipce_mmio_cleanup_device(dp);

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct deipce_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        mutex_destroy(&pp->stats_lock);
        mutex_destroy(&pp->port_reg_lock);

        pp->dp = NULL;
        kfree(pp);
    }

    drv->dev_priv[dp->dev_num] = NULL;

    mutex_destroy(&dp->common_reg_lock);
    mutex_destroy(&dp->smac_table_lock);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    dp->this_dev = NULL;
    kfree(dp);

    return;
}

/**
 * Function to remove FRS platform devices.
 * @param pdev Platform device
 * @return 0 on success or negative error code.
 */
static int deipce_device_remove(struct platform_device *pdev)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_dev_priv *dp = NULL;
    unsigned int i;

    printk(KERN_DEBUG DRV_NAME ": Remove device %s\n", dev_name(&pdev->dev));

    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        dp = drv->dev_priv[i];
        if (!dp)
            continue;

        if (dp->this_dev == &pdev->dev) {
            deipce_device_cleanup(dp);
            return 0;
        }
    }

    pr_err(DRV_NAME ": Device %s not found\n", dev_name(&pdev->dev));

    return -ENODEV;
}

/*
 * Platform device driver match table.
 */
static const struct of_device_id deipce_match[] = {
    { .compatible = "ttt,deipce", },
    { },
};

/**
 * Platform Driver definition for Linux core.
 */
static struct platform_driver deipce_dev_driver = {
    .driver = {
        .name = "deipce",
        .owner = THIS_MODULE,
        .of_match_table = deipce_match,
    },
    .probe = &deipce_device_init,
    .remove = &deipce_device_remove,
};

/**
 * Module init.
 * @return 0 if success.
 */
static int __init deipce_init(void)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    int ret = 0;

    ret = deipce_debugfs_init_driver(drv);
    if (ret)
        goto err_init_debugfs;

    /*
     * Ordering dependencies:
     * - clock depends on FPTS
     * - FSC depends on clock
     * - IBC depends on clocks
     */
    ret = deipce_fpts_init_driver();
    if (ret) {
        pr_warn(DRV_NAME ": Failed to initialize FPTS driver\n");
        // We can work without.
    }
    ret = deipce_clock_init_driver();
    if (ret) {
        pr_warn(DRV_NAME ": Failed to initialize clock driver\n");
        // We can work without.
    }
    ret = deipce_fsc_init_driver();
    if (ret) {
        pr_warn(DRV_NAME ": Failed to initialize FSC driver\n");
        // We can work without.
    }
    ret = deipce_ibc_init_driver();
    if (ret) {
        pr_warn(DRV_NAME ": Failed to initialize IBC driver\n");
        // We can work without.
    }

    ret = deipce_init_crc40(drv);
    if (ret)
        goto err_init_crc40;

    drv->wq_low = alloc_workqueue(DRV_NAME, WQ_MEM_RECLAIM, 0);
    if (!drv->wq_low) {
        pr_err(DRV_NAME ": Failed to create work queue\n");
        ret = -ENOMEM;
        goto err_workqueue_lowpri;
    }

    drv->wq_high = alloc_workqueue(DRV_NAME, WQ_HIGHPRI | WQ_MEM_RECLAIM, 0);
    if (!drv->wq_high) {
        pr_err(DRV_NAME ": Failed to create work queue\n");
        ret = -ENOMEM;
        goto err_workqueue_highpri;
    }

    ret = platform_driver_register(&deipce_dev_driver);
    if (ret)
        goto err_reg_driver;

    ret = deipce_switchdev_init_driver();
    if (ret)
        goto err_init_switchdev;

    return 0;

err_init_switchdev:
err_reg_driver:
    destroy_workqueue(drv->wq_high);
    drv->wq_high = NULL;

err_workqueue_highpri:
    destroy_workqueue(drv->wq_low);
    drv->wq_low = NULL;

err_workqueue_lowpri:
    deipce_cleanup_crc40(drv);

err_init_crc40:
    deipce_ibc_cleanup_driver();
    deipce_fsc_cleanup_driver();
    deipce_clock_cleanup_driver();
    deipce_fpts_cleanup_driver();
    deipce_debugfs_cleanup_driver(drv);

err_init_debugfs:
    return ret;
}

/**
 * Module exit.
 * Cleanup everything.
 */
static void __exit deipce_cleanup(void)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();

    printk(KERN_DEBUG DRV_NAME ": Module cleanup\n");

    deipce_switchdev_cleanup_driver();

    platform_driver_unregister(&deipce_dev_driver);

    destroy_workqueue(drv->wq_high);
    drv->wq_high = NULL;
    destroy_workqueue(drv->wq_low);
    drv->wq_low = NULL;

    deipce_cleanup_crc40(drv);

    deipce_ibc_cleanup_driver();
    deipce_fsc_cleanup_driver();
    deipce_clock_cleanup_driver();
    deipce_fpts_cleanup_driver();

    deipce_debugfs_cleanup_driver(drv);

    pr_debug(DRV_NAME ": %s() done\n", __func__);

    return;
}

// Module init and exit function
module_init(deipce_init);
module_exit(deipce_cleanup);

