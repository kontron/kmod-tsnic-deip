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
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include "deipce_main.h"
#include "deipce_mmio.h"

/**
 * Init MMIO register access.
 * @param dp Device private
 * @param pdev Platform device
 * @return 0 on success.
 */
int deipce_mmio_init_device(struct deipce_dev_priv *dp,
                            struct platform_device *pdev,
                            struct deipce_cfg *frs_cfg)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    int i;
    int ret = 0;

    dev_dbg(dp->this_dev, "Setup device for memory mapped register access\n");

    if (!res) {
        dev_err(dp->this_dev, "No I/O memory defined\n");
        ret = -ENXIO;
        goto out;
    }

    dp->ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->ioaddr) {
        dev_err(dp->this_dev, "ioremap failed for switch address 0x%llx\n",
                (unsigned long long int) res->start);
        ret = -ENXIO;
        goto out;
    }

    dev_printk(KERN_DEBUG, dp->this_dev,
               "Device uses memory mapped access: 0x%llx/0x%llx\n",
               (unsigned long long int) res->start,
               (unsigned long long int) resource_size(res));

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct deipce_port_priv *pp = dp->port[i];
        struct deipce_port_cfg *port_cfg = &frs_cfg->port[i];
        resource_size_t port_base;
        resource_size_t adapter_base;

        if (!pp)
            continue;

        port_base = port_cfg->baseaddr;
        if (!port_base) {
            dev_warn(dp->this_dev, "No base address for port %i\n",
                     pp->port_num);
            continue;
        }

        pp->ioaddr = ioremap_nocache(port_base, DEIPCE_PORT_IOSIZE);
        if (!pp->ioaddr) {
            dev_warn(dp->this_dev, "ioremap failed for port %i address 0x%llx\n",
                     pp->port_num, (unsigned long long int) port_base);
            // TODO: fail?
            continue;
        }

        dev_printk(KERN_DEBUG, dp->this_dev,
                   "Port %i uses memory mapped access: 0x%llx/0x%x\n",
                   pp->port_num, (unsigned long long int) port_base,
                   DEIPCE_PORT_IOSIZE);

        adapter_base = port_cfg->adapter_baseaddr;
        // Adapter is optional.
        if (!adapter_base)
            continue;

        pp->adapter.ioaddr =
            ioremap_nocache(adapter_base, DEIPCE_ADAPTER_IOSIZE);
        if (!pp->adapter.ioaddr) {
            dev_warn(dp->this_dev,
                     "ioremap failed for port %i adapter address 0x%llx\n",
                     pp->port_num, (unsigned long long int) adapter_base);
            // TODO: fail?
            continue;
        }

        dev_printk(KERN_DEBUG, dp->this_dev,
                   "Port %i adapter uses memory mapped access:"
                   " 0x%llx/0x%x\n",
                   pp->port_num, (unsigned long long int) adapter_base,
                   DEIPCE_ADAPTER_IOSIZE);
    }

out:
    return ret;
}

/**
 * Cleanup MMIO register access.
 * @param dp Device private
 */
void deipce_mmio_cleanup_device(struct deipce_dev_priv *dp)
{
    int i;

    dev_dbg(dp->this_dev, "Cleanup device memory mapped register access\n");

    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

    for (i = 0; i < ARRAY_SIZE(dp->port); i++) {
        struct deipce_port_priv *pp = dp->port[i];

        if (!pp)
            continue;

        if (pp->adapter.ioaddr) {
            iounmap(pp->adapter.ioaddr);
            pp->adapter.ioaddr = NULL;
        }

        if (pp->ioaddr) {
            iounmap(pp->ioaddr);
            pp->ioaddr = NULL;
        }
    }

    return;
}
