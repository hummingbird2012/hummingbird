ifndef TARGET
TARGET=avr-hummingbird
endif

CONTIKI_PROJECT = rf_tx

all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES = atcmd.c
CONTIKI_TARGET_SOURCEFILES += contiki-hummingbird-main.c

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

clean:		
	rm -rf symbols.c symbols.h $(CONTIKI_PROJECT).hex $(CONTIKI_PROJECT).eep
	rm -rf obj_avr-hummingbird
	rm -rf *.a *.map
