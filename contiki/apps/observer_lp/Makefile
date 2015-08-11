DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = observer_lp

CONTIKI_NO_NET=1

TARGETDIRS += ../../platform

CFLAGS += -DCC2538_RF_CONF_CHANNEL=22

all: $(CONTIKI_PROJECT)

CONTIKI = ../../../../contiki

include $(CONTIKI)/Makefile.include
