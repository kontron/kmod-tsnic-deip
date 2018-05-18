#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#include "deipce_mdio_main.h"

static void __iomem *mdio_iomem;
static spinlock_t mdio_lock;
static struct mii_bus *mdio_bus;

static int deipce_mdio_read(struct mii_bus *bus, int phy_id, int location)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&mdio_lock, flags);
    iowrite32(phy_id, mdio_iomem + 0x84);
    ret = ioread32(mdio_iomem + (location << 2)) & 0xffff;
    spin_unlock_irqrestore(&mdio_lock, flags);

    return ret;
}
static int deipce_mdio_write(struct mii_bus *bus, int phy_id, int location, u16 val)
{
    unsigned long flags;

    spin_lock_irqsave(&mdio_lock, flags);
    iowrite32(phy_id, mdio_iomem + 0x84);
    iowrite32(val, mdio_iomem + (location << 2));
    spin_unlock_irqrestore(&mdio_lock, flags);

    return 0;
}

static int deipce_mdio_probe(struct platform_device *pdev)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    int ret;

    if (!res) {
        dev_err(&pdev->dev, "No I/O memory defined\n");
        return -ENODEV;
    }

    mdio_iomem = ioremap_nocache(res->start, resource_size(res));
    if (!mdio_iomem) {
        dev_err(&pdev->dev, "ioremap failed\n");
        return -ENOMEM;
    }

    spin_lock_init(&mdio_lock);
    mdio_bus = mdiobus_alloc();
    if (!mdio_bus) {
        ret = -ENOMEM;
        goto out_free;
    }

    mdio_bus->name = "DEIPCE MII Bus";
    mdio_bus->read = &deipce_mdio_read;
    mdio_bus->write = &deipce_mdio_write;
    snprintf(mdio_bus->id, MII_BUS_ID_SIZE, "deipce-mdio");

    ret = mdiobus_register(mdio_bus);
    if (ret)
        goto out_free;

    return 0;

out_free:
    mdiobus_free(mdio_bus);

    return ret;
}

static int deipce_mdio_remove(struct platform_device *pdev)
{
    mdiobus_unregister(mdio_bus);
    mdiobus_free(mdio_bus);

    return 0;
}

static struct platform_driver deipce_mdio_driver = {
    .driver = {
        .name = "deipce-mdio",
        .owner = THIS_MODULE,
    },
    .probe = deipce_mdio_probe,
    .remove = deipce_mdio_remove,
};

int __init deipce_mdio_init_driver(void)
{
    return platform_driver_register(&deipce_mdio_driver);
}

void deipce_mdio_cleanup_driver(void)
{
    platform_driver_unregister(&deipce_mdio_driver);
}
