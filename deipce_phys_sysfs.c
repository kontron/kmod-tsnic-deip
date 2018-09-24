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

#include "deipce_main.h"
#include "deipce_hw.h"
#include "deipce_sysfs_common.h"
#include "deipce_phys_sysfs.h"

/**
 * Output delays in user format.
 * @param buf Buffer to write to.
 * @param delay Array of delays in nanoseconds, index is enum link_mode.
 * @return Number of bytes written to buf or error code.
 */
static ssize_t deipce_phys_delays_to_user(
        char *buf,
        const unsigned int delay[DEIPCE_LINK_MODE_COUNT])
{
    ssize_t ret;

    ret = sprintf(buf, "%u,%u,%u\n",
                  delay[LM_10FULL],
                  delay[LM_100FULL],
                  delay[LM_1000FULL]);

    return ret;
}

/**
 * Get delays from user format.
 * @param buf Buffer to read delays from.
 * @param count Number of bytes in buffer.
 * @param delay Array for writing delay values, index is enum link_mode.
 * @return Number of bytes read from buf or error code.
 */
static ssize_t deipce_phys_delays_from_user(
        const char *buf, size_t count,
        unsigned int delay[DEIPCE_LINK_MODE_COUNT])
{
    /*
     * Check for garbage at the end,
     * but allow any number of trailing white space.
     */
    int len = count;
    int ret = sscanf(buf, "%u,%u,%u %n",
                     &delay[LM_10FULL],
                     &delay[LM_100FULL],
                     &delay[LM_1000FULL],
                     &len);

    if (ret != 3 || len != count)
        return -EINVAL;

    return count;
}

// phydev_rx_delay_ns

static ssize_t deipce_phys_sysfs_phy_rx_delay_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    deipce_get_delays(pp, DEIPCE_DELAY_PHY, DEIPCE_DIR_RX, delay);
    ret = deipce_phys_delays_to_user(buf, delay);

    return ret;
}

static ssize_t deipce_phys_sysfs_phy_rx_delay_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    ret = deipce_phys_delays_from_user(buf, count, delay);
    if (ret > 0)
        deipce_set_delays(pp, DEIPCE_DELAY_PHY, DEIPCE_DIR_RX, delay);

    return ret;
}

static DEIPCE_ATTR_RW(phydev_rx_delay_ns,
                      &deipce_phys_sysfs_phy_rx_delay_show,
                      &deipce_phys_sysfs_phy_rx_delay_store);

// phydev_tx_delay_ns

static ssize_t deipce_phys_sysfs_phy_tx_delay_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    deipce_get_delays(pp, DEIPCE_DELAY_PHY, DEIPCE_DIR_TX, delay);
    ret = deipce_phys_delays_to_user(buf, delay);

    return ret;
}

static ssize_t deipce_phys_sysfs_phy_tx_delay_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    ret = deipce_phys_delays_from_user(buf, count, delay);
    if (ret > 0)
        deipce_set_delays(pp, DEIPCE_DELAY_PHY, DEIPCE_DIR_TX, delay);

    return ret;
}

static DEIPCE_ATTR_RW(phydev_tx_delay_ns,
                      &deipce_phys_sysfs_phy_tx_delay_show,
                      &deipce_phys_sysfs_phy_tx_delay_store);

// adapter_rx_delay_ns

static ssize_t deipce_phys_sysfs_adapter_rx_delay_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    deipce_get_delays(pp, DEIPCE_DELAY_ADAPTER, DEIPCE_DIR_RX, delay);
    ret = deipce_phys_delays_to_user(buf, delay);

    return ret;
}

static ssize_t deipce_phys_sysfs_adapter_rx_delay_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    ret = deipce_phys_delays_from_user(buf, count, delay);
    if (ret > 0)
        deipce_set_delays(pp, DEIPCE_DELAY_ADAPTER, DEIPCE_DIR_RX, delay);

    return ret;
}

static DEIPCE_ATTR_RW(adapter_rx_delay_ns,
                      &deipce_phys_sysfs_adapter_rx_delay_show,
                      &deipce_phys_sysfs_adapter_rx_delay_store);

// adapter_tx_delay_ns

static ssize_t deipce_phys_sysfs_adapter_tx_delay_show(
        struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s()\n", __func__);

    deipce_get_delays(pp, DEIPCE_DELAY_ADAPTER, DEIPCE_DIR_TX, delay);
    ret = deipce_phys_delays_to_user(buf, delay);

    return ret;
}

static ssize_t deipce_phys_sysfs_adapter_tx_delay_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    unsigned int delay[DEIPCE_LINK_MODE_COUNT];
    ssize_t ret;

    netdev_dbg(to_net_dev(dev), "%s() buf %s count %zu\n",
               __func__, buf, count);

    ret = deipce_phys_delays_from_user(buf, count, delay);
    if (ret > 0)
        deipce_set_delays(pp, DEIPCE_DELAY_ADAPTER, DEIPCE_DIR_TX, delay);

    return ret;
}

static DEIPCE_ATTR_RW(adapter_tx_delay_ns,
                      &deipce_phys_sysfs_adapter_tx_delay_show,
                      &deipce_phys_sysfs_adapter_tx_delay_store);

/**
 * Determine if attribute is visible.
 * @param kobj kobject of the device.
 * @param attr Attribute.
 * @param index Attribute index.
 */
static umode_t deipce_phys_sysfs_is_visible(struct kobject *kobj,
                                            struct attribute *attr,
                                            int index)
{
    struct device *dev = kobj_to_dev(kobj);
    struct deipce_port_priv *pp = to_deipce_port_priv(dev);
    struct deipce_dev_priv *dp = pp->dp;

    netdev_dbg(pp->netdev, "%s() attr %s index %i\n",
               __func__, attr->name, index);

    if (!(dp->features.ts_ports & (1u << pp->port_num)))
        return 0;

    return attr->mode;
}

/// Attribute groups
static const struct attribute_group *deipce_phys_sysfs_attr_groups[] = {
    &(struct attribute_group){
        .name = "physical_layer",
        .is_visible = &deipce_phys_sysfs_is_visible,
        .attrs = (struct attribute*[]){
            &dev_attr_phydev_rx_delay_ns.attr,
            &dev_attr_phydev_tx_delay_ns.attr,
            &dev_attr_adapter_rx_delay_ns.attr,
            &dev_attr_adapter_tx_delay_ns.attr,
            NULL,
        },
    },
    NULL,
};

/**
 * Initialize sysfs files.
 * @param pp Port privates.
 */
int deipce_phys_sysfs_init(struct deipce_port_priv *pp)
{
    int ret;

    ret = sysfs_create_groups(&pp->netdev->dev.kobj,
                              deipce_phys_sysfs_attr_groups);

    return ret;
}

/**
 * Cleanup sysfs files.
 * @param pp Port privates.
 */
void deipce_phys_sysfs_cleanup(struct deipce_port_priv *pp)
{
    sysfs_remove_groups(&pp->netdev->dev.kobj, deipce_phys_sysfs_attr_groups);

    return;
}

