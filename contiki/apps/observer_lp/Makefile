DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = observer_lp

CONTIKI_NO_NET=1

TARGETDIRS += ../../platform

CFLAGS += -DCC2538_RF_CONF_CHANNEL=22
# CFLAGS += -DOBSERVER_ID

all: $(CONTIKI_PROJECT)
# ifndef OBSERVER_ID
# 	$(error OBSERVER_ID is not set)
# endif

CONTIKI = ../../../../contiki

include $(CONTIKI)/Makefile.include
