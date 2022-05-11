# Copied with only minor changes from apps/system/nsh

include $(APPDIR)/Make.defs

MAINSRC = main.c can_broadcast.c safing.c drs.c etb.c

PROGNAME = ETCetera can_broadcast safing drs etb
PRIORITY = $(CONFIG_INDUSTRY_ETCETERA_PRIORITY)
STACKSIZE = $(CONFIG_INDUSTRY_ETCETERA_STACKSIZE)
MODULE = $(CONFIG_INDUSTRY_ETCETERA)

include $(APPDIR)/Application.mk
