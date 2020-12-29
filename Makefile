# Copied with only minor changes from apps/system/nsh

include $(APPDIR)/Make.defs

MAINSRC = main.c

PROGNAME = ETCetera
PRIORITY = $(CONFIG_INDUSTRY_ETCETERA_PRIORITY)
STACKSIZE = $(CONFIG_INDUSTRY_ETCETERA_STACKSIZE)
MODULE = $(CONFIG_INDUSTRY_ETCETERA)

include $(APPDIR)/Application.mk
