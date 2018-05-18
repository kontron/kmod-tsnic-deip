#ifndef DEIPCE_MDIO_MAIN_H
#define DEIPCE_MDIO_MAIN_H

#include <linux/init.h>

int __init deipce_mdio_init_driver(void);
void deipce_mdio_cleanup_driver(void);

#endif
