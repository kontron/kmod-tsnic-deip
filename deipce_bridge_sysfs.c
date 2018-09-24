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

#include "deipce_main.h"
#include "deipce_qbridge_sysfs.h"
#include "deipce_edgex_sysfs.h"
#include "deipce_mstp_sysfs.h"
#include "deipce_bridge_sysfs.h"

/**
 * Get switch privates by Linux bridge device.
 * @param dev Linux bridge device.
 * @return Switch privates or NULL.
 */
struct deipce_dev_priv *deipce_bridge_sysfs_get_priv(struct device *dev)
{
    struct net_device *master = to_net_dev(dev);
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_dev_priv *dp;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        dp = drv->dev_priv[i];
        if (dp && dp->switchdev.master == master)
            return dp;
    }

    return NULL;
}

/**
 * Initialize bridge sysfs files.
 * @param dp Switch privates.
 */
int deipce_bridge_sysfs_init(struct deipce_dev_priv *dp)
{
    int ret;

    ret = deipce_qbridge_sysfs_init(dp);
    if (ret)
        goto err_qbridge;

    ret = deipce_edgex_sysfs_init_switch(dp);
    if (ret)
        goto err_edgex;

    ret = deipce_mstp_sysfs_init_switch(dp);
    if (ret)
        goto err_mstp;

    return ret;

err_mstp:
    deipce_edgex_sysfs_cleanup_switch(dp);

err_edgex:
    deipce_qbridge_sysfs_cleanup(dp);

err_qbridge:
    return ret;
}

/**
 * Cleanup bridge sysfs files.
 * @param dp Switch privates.
 */
void deipce_bridge_sysfs_cleanup(struct deipce_dev_priv *dp)
{
    deipce_mstp_sysfs_cleanup_switch(dp);
    deipce_edgex_sysfs_cleanup_switch(dp);
    deipce_qbridge_sysfs_cleanup(dp);

    return;
}

