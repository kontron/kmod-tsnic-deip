# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)

ifneq ($(KERNELRELEASE),)
# call from kernel build system

ccflags-y += -Wall
obj-m += deipce.o
deipce-y := deipce_main.o \
	deipce_netdev.o deipce_netdevif.o \
	deipce_ioctl.o deipce_hw.o deipce_ethtool.o \
	deipce_adapter.o deipce_sfp.o deipce_preempt.o \
	deipce_clock_main.o deipce_clock_nco.o \
	deipce_fpts_main.o \
	deipce_fsc_main.o deipce_fsc_hw.o deipce_ibc_main.o \
	deipce_shaper.o deipce_time.o deipce_mstp.o

deipce-$(CONFIG_NET_SWITCHDEV) += deipce_switchdev.o
deipce-$(CONFIG_PTP_1588_CLOCK) += deipce_clock_ptp.o
deipce-$(CONFIG_SYSFS) += deipce_fsc_sysfs.o deipce_fqtss_sysfs.o \
	deipce_port_sysfs.o deipce_edgex_sysfs.o deipce_phys_sysfs.o \
	deipce_preempt_sysfs.o
ifdef CONFIG_NET_SWITCHDEV
deipce-$(CONFIG_SYSFS) += deipce_qbridge_sysfs.o \
	deipce_bridge_sysfs.o deipce_mstp_sysfs.o
endif

else

KDIR ?= /lib/modules/$(shell uname -r)/build

modules modules_install clean:
	$(MAKE) -C $(KDIR) M=$$PWD $@

endif

