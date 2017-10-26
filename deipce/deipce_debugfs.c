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

#include "deipce_types.h"
#include "deipce_main.h"
#include "deipce_preempt.h"
#include "deipce_fsc_main.h"
#include "deipce_debugfs.h"

/**
 * Helper function to get boolean value from user space
 * (write operation from user space).
 * @param user_buf User space buffer to read from.
 * @param size Maximum number of bytes to read.
 * @param f_pos Current buffer position.
 * @param value Place for value.
 * @return Number of bytes written or negative error code.
 */
static ssize_t deipce_debugfs_write_bool(const char __user *user_buf,
                                         size_t size,
                                         loff_t *f_pos,
                                         bool *value)
{
    char buf[8] = "";
    ssize_t len = -EFAULT;
    int ret = -ERANGE;

    len = simple_write_to_buffer(buf, sizeof(buf) - 1, f_pos, user_buf, size);
    if (len < 0)
        return len;

    buf[sizeof(buf) - 1] = '\0';

    // kstrtobool appeared in Linux 4.6.
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
    ret = kstrtobool(buf, value);
#else
    switch (buf[0]) {
    case 'Y':
    case 'y':
    case '1':
        *value = true;
        ret = 0;
        break;
    case 'N':
    case 'n':
    case '0':
        *value = false;
        ret = 0;
    }
#endif
    if (ret < 0)
        return ret;

    return len;
}

/**
 * Helper function to get 16 bit unsigned integer value as string
 * from user space (write operation from user space).
 * @param user_buf User space buffer to read from.
 * @param size Maximum number of bytes to read.
 * @param f_pos Current buffer position.
 * @param value Place for value.
 * @return Number of bytes written or negative error code.
 */
static ssize_t deipce_debugfs_write_uint16(const char __user *user_buf,
                                           size_t size,
                                           loff_t *f_pos,
                                           uint16_t *value)
{
    char buf[32] = "";
    ssize_t len = -EFAULT;
    int ret = -ERANGE;

    len = simple_write_to_buffer(buf, sizeof(buf) - 1, f_pos, user_buf, size);
    if (len < 0)
        return len;

    buf[sizeof(buf) - 1] = '\0';

    ret = kstrtou16(buf, 0, value);
    if (ret < 0)
        return ret;

    return len;
}

/**
 * Helper function to get 32 bit unsigned integer value as string
 * from user space (write operation from user space).
 * @param user_buf User space buffer to read from.
 * @param size Maximum number of bytes to read.
 * @param f_pos Current buffer position.
 * @param value Place for value.
 * @return Number of bytes written or negative error code.
 */
static ssize_t deipce_debugfs_write_uint32(const char __user *user_buf,
                                           size_t size,
                                           loff_t *f_pos,
                                           uint32_t *value)
{
    char buf[32] = "";
    ssize_t len = -EFAULT;
    int ret = -ERANGE;

    len = simple_write_to_buffer(buf, sizeof(buf) - 1, f_pos, user_buf, size);
    if (len < 0)
        return len;

    buf[sizeof(buf) - 1] = '\0';

    ret = kstrtou32(buf, 0, value);
    if (ret < 0)
        return ret;

    return len;
}

// Cut-through (ct)

static bool deipce_debugfs_ct_supported(struct deipce_dev_priv *dp,
                                        struct deipce_port_priv *pp)
{
    return dp->features.ct_ports & (1u << pp->port_num);
}

// port file "rx_ct_enable"

static int deipce_debugfs_rx_ct_read(struct seq_file *m, void *v)
{
    struct deipce_port_priv *pp = m->private;
    uint16_t port_state = deipce_read_port_reg(pp, PORT_REG_STATE);

    seq_printf(m, "%u\n", !!(port_state & PORT_STATE_RX_CT));

    return 0;
}

static ssize_t deipce_debugfs_rx_ct_write(struct file *file,
                                          const char __user *user_buf,
                                          size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    struct deipce_netdev_priv *np = netdev_priv(pp->netdev);
    bool enable = false;
    uint16_t port_state = 0;
    ssize_t len = -EFAULT;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_bool(user_buf, size, f_pos, &enable);
    if (len < 0)
        return len;

    mutex_lock(&np->link_mode_lock);

    port_state = deipce_read_port_reg(pp, PORT_REG_STATE);
    if (enable)
        port_state |= PORT_STATE_RX_CT;
    else
        port_state &= ~PORT_STATE_RX_CT;
    deipce_write_port_reg(pp, PORT_REG_STATE, port_state);

    mutex_unlock(&np->link_mode_lock);

    *f_pos += len;
    return len;
}

static int deipce_debugfs_rx_ct_open(struct inode *inode,
                                     struct file *file)
{
    return single_open(file, &deipce_debugfs_rx_ct_read,
                       inode->i_private);
}

static const struct file_operations deipce_debugfs_rx_ct_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_rx_ct_open,
    .read = &seq_read,
    .write = &deipce_debugfs_rx_ct_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_ct_prio_queues"

static int deipce_debugfs_tx_ct_prio_read(struct seq_file *m, void *v)
{
    struct deipce_port_priv *pp = m->private;
    struct deipce_dev_priv *dp = pp->dp;
    uint16_t tx_ct = deipce_read_port_reg(pp, PORT_REG_TX_CT);

    seq_printf(m, "0x%x\n", tx_ct & ((1u << dp->features.prio_queues) - 1u));

    return 0;
}

static ssize_t deipce_debugfs_tx_ct_prio_write(struct file *file,
                                               const char __user *user_buf,
                                               size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    struct deipce_dev_priv *dp = pp->dp;
    struct deipce_netdev_priv *np = netdev_priv(pp->netdev);
    uint16_t tx_ct = false;
    ssize_t len = -EFAULT;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_uint16(user_buf, size, f_pos, &tx_ct);
    if (len < 0)
        return len;

    if (tx_ct & ~((1u << dp->features.prio_queues) - 1u))
        return -EINVAL;

    mutex_lock(&np->link_mode_lock);

    deipce_write_port_reg(pp, PORT_REG_TX_CT, tx_ct);

    mutex_unlock(&np->link_mode_lock);

    *f_pos += len;
    return len;
}

static int deipce_debugfs_tx_ct_prio_open(struct inode *inode,
                                          struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_ct_prio_read,
                       inode->i_private);
}

static const struct file_operations deipce_debugfs_tx_ct_prio_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_ct_prio_open,
    .read = &seq_read,
    .write = &deipce_debugfs_tx_ct_prio_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// Preemption

static bool deipce_debugfs_preempt_supported(struct deipce_dev_priv *dp,
                                             struct deipce_port_priv *pp)
{
    return dp->features.preempt_ports & (1u << pp->port_num);
}

// port file "tx_preempt_prio_queues"

static int deipce_debugfstx_preempt_prio_read(struct seq_file *m, void *v)
{
    struct deipce_port_priv *pp = m->private;
    unsigned int tx_preempt = deipce_preempt_get_enable(pp);

    seq_printf(m, "0x%x\n", tx_preempt);

    return 0;
}

static ssize_t deipce_debugfstx_preempt_prio_write(
        struct file *file,
        const char __user *user_buf,
        size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    struct deipce_dev_priv *dp = pp->dp;
    uint16_t tx_preempt = false;
    ssize_t len = -EFAULT;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_uint16(user_buf, size, f_pos, &tx_preempt);
    if (len < 0)
        return len;

    if (tx_preempt & ~((1u << dp->features.prio_queues) - 1u))
        return -EINVAL;

    deipce_preempt_set_enable(pp, tx_preempt);

    *f_pos += len;
    return len;
}

static int deipce_debugfstx_preempt_prio_open(struct inode *inode,
                                               struct file *file)
{
    return single_open(file, &deipce_debugfstx_preempt_prio_read,
                       inode->i_private);
}

static const struct file_operations deipce_debugfstx_preempt_prio_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfstx_preempt_prio_open,
    .read = &seq_read,
    .write = &deipce_debugfstx_preempt_prio_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_preempt_min_frag_size"

static int deipce_debugfs_tx_preempt_fragsize_read(struct seq_file *m, void *v)
{
    struct deipce_port_priv *pp = m->private;

    seq_printf(m, "%u\n", deipce_preempt_get_min_frag_size(pp));

    return 0;
}

static ssize_t deipce_debugfs_tx_preempt_fragsize_write(
        struct file *file,
        const char __user *user_buf,
        size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    uint16_t tx_fragsize = false;
    ssize_t len = -EFAULT;
    int ret = -EINVAL;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_uint16(user_buf, size, f_pos, &tx_fragsize);
    if (len < 0)
        return len;

    ret = deipce_preempt_set_min_frag_size(pp, tx_fragsize);
    if (ret)
        return ret;

    *f_pos += len;
    return len;
}

static int deipce_debugfs_tx_preempt_fragsize_open(struct inode *inode,
                                                   struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_preempt_fragsize_read,
                       inode->i_private);
}

static const struct file_operations deipce_debugfs_tx_preempt_fragsize_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_preempt_fragsize_open,
    .read = &seq_read,
    .write = &deipce_debugfs_tx_preempt_fragsize_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_preempt_verify_time"

static int deipce_debugfs_tx_preempt_verify_time_read(struct seq_file *m,
                                                      void *v)
{
    struct deipce_port_priv *pp = m->private;

    seq_printf(m, "%u\n", deipce_preempt_get_verify_time(pp));

    return 0;
}

static ssize_t deipce_debugfs_tx_preempt_verify_time_write(
        struct file *file,
        const char __user *user_buf,
        size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    uint32_t verify_time = false;
    ssize_t len = -EFAULT;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_uint32(user_buf, size, f_pos, &verify_time);
    if (len < 0)
        return len;

    deipce_preempt_set_verify_time(pp, verify_time);

    *f_pos += len;
    return len;
}

static int deipce_debugfs_tx_preempt_verify_time_open(struct inode *inode,
                                                      struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_preempt_verify_time_read,
                       inode->i_private);
}

static
const struct file_operations deipce_debugfs_tx_preempt_verify_time_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_preempt_verify_time_open,
    .read = &seq_read,
    .write = &deipce_debugfs_tx_preempt_verify_time_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_preempt_verify"

static int deipce_debugfs_tx_preempt_verify_read(struct seq_file *m, void *v)
{
    struct deipce_port_priv *pp = m->private;

    seq_printf(m, "%u\n", deipce_preempt_get_verify(pp) ? 1 : 0);

    return 0;
}

static ssize_t deipce_debugfs_tx_preempt_verify_write(
        struct file *file,
        const char __user *user_buf,
        size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    bool enable = false;
    ssize_t len = -EFAULT;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_bool(user_buf, size, f_pos, &enable);
    if (len < 0)
        return len;

    deipce_preempt_set_verify(pp, enable);

    *f_pos += len;
    return len;
}

static int deipce_debugfs_tx_preempt_verify_open(struct inode *inode,
                                                 struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_preempt_verify_read,
                       inode->i_private);
}

static const struct file_operations deipce_debugfs_tx_preempt_verify_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_preempt_verify_open,
    .read = &seq_read,
    .write = &deipce_debugfs_tx_preempt_verify_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_preempt_verify_status"

static int deipce_debugfs_tx_preempt_verify_status_read(struct seq_file *m,
                                                        void *v)
{
    struct deipce_port_priv *pp = m->private;
    const char *str = "unknown";

    switch (deipce_preempt_get_status(pp)) {
    case DEIPCE_PREEMPT_VERIFY_DISABLED:
        str = "disabled";
        break;
    case DEIPCE_PREEMPT_VERIFY_IDLE:
        str = "initial";
        break;
    case DEIPCE_PREEMPT_VERIFY_IN_PROGRESS:
        str = "verifying";
        break;
    case DEIPCE_PREEMPT_VERIFY_FAILED:
        str = "failed";
        break;
    case DEIPCE_PREEMPT_VERIFY_OK:
        str = "succeeded";
        break;
    }

    seq_printf(m, "%s\n", str);

    return 0;
}

static int deipce_debugfs_tx_preempt_verify_status_open(struct inode *inode,
                                                        struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_preempt_verify_status_read,
                       inode->i_private);
}

static
const struct file_operations deipce_debugfs_tx_preempt_verify_status_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_preempt_verify_status_open,
    .read = &seq_read,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_preempt_verify_limit"

static int deipce_debugfs_tx_preempt_verify_limit_read(struct seq_file *m,
                                                       void *v)
{
    struct deipce_port_priv *pp = m->private;

    seq_printf(m, "%u\n", deipce_preempt_get_verify_limit(pp));

    return 0;
}

static ssize_t deipce_debugfs_tx_preempt_verify_limit_write(
        struct file *file,
        const char __user *user_buf,
        size_t size, loff_t *f_pos)
{
    struct seq_file *m = file->private_data;
    struct deipce_port_priv *pp = m->private;
    uint32_t verify_limit = false;
    ssize_t len = -EFAULT;

    netdev_dbg(pp->netdev, "%s() size %zu\n", __func__, size);

    len = deipce_debugfs_write_uint32(user_buf, size, f_pos, &verify_limit);
    if (len < 0)
        return len;

    deipce_preempt_set_verify_limit(pp, verify_limit);

    *f_pos += len;
    return len;
}

static int deipce_debugfs_tx_preempt_verify_limit_open(struct inode *inode,
                                                       struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_preempt_verify_limit_read,
                       inode->i_private);
}

static
const struct file_operations deipce_debugfs_tx_preempt_verify_limit_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_preempt_verify_limit_open,
    .read = &seq_read,
    .write = &deipce_debugfs_tx_preempt_verify_limit_write,
    .llseek = &seq_lseek,
    .release = &single_release,
};

// port file "tx_preempt_verify_count"

static int deipce_debugfs_tx_preempt_verify_count_read(struct seq_file *m,
                                                       void *v)
{
    struct deipce_port_priv *pp = m->private;

    seq_printf(m, "%u\n", deipce_preempt_get_verify_count(pp));

    return 0;
}

static int deipce_debugfs_tx_preempt_verify_count_open(struct inode *inode,
                                                       struct file *file)
{
    return single_open(file, &deipce_debugfs_tx_preempt_verify_count_read,
                       inode->i_private);
}

static
const struct file_operations deipce_debugfs_tx_preempt_verify_count_fops = {
    .owner = THIS_MODULE,
    .open = &deipce_debugfs_tx_preempt_verify_count_open,
    .read = &seq_read,
    .llseek = &seq_lseek,
    .release = &single_release,
};

/**
 * Debugfs file creation information.
 */
struct deipce_debugfs_file {
    const char *name;                   ///< debugfs file name
    mode_t mode;                        ///< mode bits
    const struct file_operations *fops; ///< file operations
    /// function to determine whether supported or not, NULL means yes
    bool (*supported)(struct deipce_dev_priv *dp,
                      struct deipce_port_priv *pp);
};

/// Switch debugfs files.
static const struct deipce_debugfs_file deipce_debugfs_switch_files[] = {
    // None currently.
};

/// Port debugfs files.
static const struct deipce_debugfs_file deipce_debugfs_port_files[] = {
    {
        "rx_ct_enable", S_IRUGO | S_IWUSR,
        &deipce_debugfs_rx_ct_fops, &deipce_debugfs_ct_supported,
    },
    {
        "tx_ct_prio_queues", S_IRUGO | S_IWUSR,
        &deipce_debugfs_tx_ct_prio_fops, &deipce_debugfs_ct_supported,
    },
    {
        "tx_preempt_prio_queues", S_IRUGO | S_IWUSR,
        &deipce_debugfstx_preempt_prio_fops,
        &deipce_debugfs_preempt_supported,
    },
    {
        "tx_preempt_min_frag_size", S_IRUGO | S_IWUSR,
        &deipce_debugfs_tx_preempt_fragsize_fops,
        &deipce_debugfs_preempt_supported,
    },
    {
        "tx_preempt_verify_time", S_IRUGO | S_IWUSR,
        &deipce_debugfs_tx_preempt_verify_time_fops,
        &deipce_debugfs_preempt_supported,
    },
    {
        "tx_preempt_verify", S_IRUGO | S_IWUSR,
        &deipce_debugfs_tx_preempt_verify_fops,
        &deipce_debugfs_preempt_supported,
    },
    {
        "tx_preempt_verify_status", S_IRUGO,
        &deipce_debugfs_tx_preempt_verify_status_fops,
        &deipce_debugfs_preempt_supported,
    },
    {
        "tx_preempt_verify_limit", S_IRUGO | S_IWUSR,
        &deipce_debugfs_tx_preempt_verify_limit_fops,
        &deipce_debugfs_preempt_supported,
    },
    {
        "tx_preempt_verify_count", S_IRUGO,
        &deipce_debugfs_tx_preempt_verify_count_fops,
        &deipce_debugfs_preempt_supported,
    },
};

int deipce_debugfs_init_port(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    const struct deipce_debugfs_file *fileinfo = NULL;
    struct dentry *dentry = NULL;
    char name[128] = "";
    unsigned int i;

    netdev_dbg(pp->netdev, "Init debugfs for port %u\n", pp->port_num);

    sprintf(name, "port%u", pp->port_num);
    pp->debugfs_dir = debugfs_create_dir(name, dp->debugfs_dir);
    if (IS_ERR_OR_NULL(pp->debugfs_dir)) {
        netdev_err(pp->netdev, "Failed to create debugfs directory\n");
        pp->debugfs_dir = NULL;
        return -ENODEV;
    }

    for (i = 0; i < ARRAY_SIZE(deipce_debugfs_port_files); i++) {
        fileinfo = &deipce_debugfs_port_files[i];
        if (fileinfo->supported && !fileinfo->supported(dp, pp))
            continue;

        netdev_dbg(pp->netdev, "Create debugfs file %s for port %u\n",
                   fileinfo->name, pp->port_num);

        dentry = debugfs_create_file(fileinfo->name, fileinfo->mode,
                                     pp->debugfs_dir, pp,
                                     fileinfo->fops);
        if (IS_ERR_OR_NULL(dentry))
            return -ENODEV;
    }

    return 0;
}

int deipce_debugfs_init_device(struct deipce_dev_priv *dp)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_port_priv *pp = NULL;
    const struct deipce_debugfs_file *fileinfo = NULL;
    struct dentry *dentry = NULL;
    char name[sizeof(DRV_NAME) + 4];
    int ret = -ENODEV;
    unsigned int port_num;
    unsigned int i;

    dev_dbg(dp->this_dev, "Init debugfs for device\n");

    sprintf(name, "%s%02u", DRV_NAME, dp->dev_num);
    dp->debugfs_dir = debugfs_create_dir(name, drv->debugfs_dir);
    if (IS_ERR_OR_NULL(dp->debugfs_dir)) {
        dev_err(dp->this_dev, "Failed to create debugfs directory\n");
        dp->debugfs_dir = NULL;
        return -ENODEV;
    }

    for (i = 0; i < ARRAY_SIZE(deipce_debugfs_switch_files); i++) {
        fileinfo = &deipce_debugfs_switch_files[i];
        if (fileinfo->supported && !fileinfo->supported(dp, NULL))
            continue;

        dentry = debugfs_create_file(fileinfo->name, fileinfo->mode,
                                     dp->debugfs_dir, dp,
                                     fileinfo->fops);
        if (IS_ERR_OR_NULL(dentry))
            return -ENODEV;
    }

    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        pp = dp->port[port_num];
        if (!pp)
            continue;

        ret = deipce_debugfs_init_port(pp);
        if (ret)
            return ret;
    }

    return 0;
}

void deipce_debugfs_cleanup_device(struct deipce_dev_priv *dp)
{
    dev_dbg(dp->this_dev, "Cleanup debugfs for device\n");

    if (dp->debugfs_dir) {
        debugfs_remove_recursive(dp->debugfs_dir);
        dp->debugfs_dir = NULL;
    }

    return;
}

int __init deipce_debugfs_init_driver(struct deipce_drv_priv *drv)
{
    pr_debug(DRV_NAME ": Init debugfs for driver\n");

    drv->debugfs_dir = debugfs_create_dir(DRV_NAME, NULL);
    if (IS_ERR_OR_NULL(drv->debugfs_dir)) {
        pr_err(DRV_NAME ": Failed to create debugfs directory\n");
        drv->debugfs_dir = NULL;
        return -ENODEV;
    }

    return 0;
}

void deipce_debugfs_cleanup_driver(struct deipce_drv_priv *drv)
{
    pr_debug(DRV_NAME ": Cleanup debugfs for driver\n");

    debugfs_remove(drv->debugfs_dir);
    drv->debugfs_dir = NULL;

    return;
}

