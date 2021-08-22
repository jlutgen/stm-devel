include ../makefile-variables.mk

SRCS += 04-uart-ll.c

LIB_SRCS = $(LIB_DIR)/stm32f0xx_ll_rcc.c \
           $(LIB_DIR)/stm32f0xx_ll_utils.c

HDRS = 

# See ST UM1785 "Description of STM32F0 HAL and low-layer drivers"
CFLAGS += -DUSE_FULL_LL_DRIVER 

include ../makefile-targets.mk
