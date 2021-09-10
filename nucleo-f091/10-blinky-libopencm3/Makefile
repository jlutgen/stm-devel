include ../makefile-variables.mk

# Yes, that's a '=', not a '+='. We don't want any startup files
SRCS = 10-blinky-libopencm3.c

LIB_SRCS = 

HDRS = 

LDLIBS = -lopencm3_stm32f0
LDFLAGS += -L$(OPENCM3_DIR)/lib
LDFLAGS += -nostartfiles

# overwrite linkscript from makefile-variables.mk
LINKSCRIPT=$(COMMON_DIR)/stm32f091rc_opencm3.ld

include ../makefile-targets.mk
