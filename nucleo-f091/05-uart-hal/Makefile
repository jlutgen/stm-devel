include ../makefile-variables.mk

SRCS += 05-uart-hal.c

# uart needs dma
LIB_SRCS := \
	$(LIB_DIR)/stm32f0xx_hal_tim.c \
	$(LIB_DIR)/stm32f0xx_hal_tim_ex.c \
	$(LIB_DIR)/stm32f0xx_hal_uart.c \
	$(LIB_DIR)/stm32f0xx_hal_uart_ex.c \
	$(LIB_DIR)/stm32f0xx_hal_rcc.c \
	$(LIB_DIR)/stm32f0xx_hal_rcc_ex.c \
	$(LIB_DIR)/stm32f0xx_hal.c \
	$(LIB_DIR)/stm32f0xx_hal_i2c.c \
	$(LIB_DIR)/stm32f0xx_hal_i2c_ex.c \
	$(LIB_DIR)/stm32f0xx_hal_gpio.c \
	$(LIB_DIR)/stm32f0xx_hal_dma.c \
	$(LIB_DIR)/stm32f0xx_hal_cortex.c \
	$(LIB_DIR)/stm32f0xx_hal_pwr.c \
	$(LIB_DIR)/stm32f0xx_hal_pwr_ex.c \
	$(LIB_DIR)/stm32f0xx_hal_flash.c \
	$(LIB_DIR)/stm32f0xx_hal_flash_ex.c

HDRS = \
	05-uart-hal.h \
	stm32f0xx_hal_conf.h

# See ST UM1785 "Description of STM32F0 HAL and low-layer drivers"
#CFLAGS += -DUSE_FULL_LL_DRIVER 

include ../makefile-targets.mk
