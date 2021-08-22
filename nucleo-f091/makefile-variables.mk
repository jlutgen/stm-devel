############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

# Processor type
# See stm32f0xx.h for a list of thes
PROCESSOR=STM32F091xC

ifdef WSL_DISTRO_NAME        # Windows Subsystem for Linux
	STM_COMMON=/mnt/c/Users/jlutg/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
    # TODO: PROG_PREFIX for WSL
else
	STM_COMMON=$(HOME)/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
	PROG_PREFIX=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin
endif

COMMON_DIR=../common
LIB_DIR=$(STM_COMMON)/Drivers/STM32F0xx_HAL_Driver/Src

CROSS=arm-none-eabi

# The C compiler
CC=$(CROSS)-gcc

# The bin creator
HX=$(CROSS)-objcopy

# The object dumper
OBJDMP=$(CROSS)-objdump

# The size-reporting tool
SIZE=$(CROSS)-size

# The utility for writing a .bin file to the MCU.
WRITE=$(PROG_PREFIX)/STM32_Programmer_CLI

# The output target $(TARGET).bin
TARGET=out

SRCS = \
	$(COMMON_DIR)/system_stm32f0xx.c \
	$(COMMON_DIR)/startup_stm32f091xc.s

INCLUDES = \
	-I. \
	-I$(STM_COMMON)/Drivers/CMSIS/Include \
	-I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
	-I$(STM_COMMON)/Drivers/STM32F0xx_HAL_Driver/Inc 

CFLAGS = \
	-ggdb3 -O0 -Wall \
	-std=gnu11 \
	-mthumb -mcpu=cortex-m0 \
	-mfloat-abi=soft \
	--specs=nosys.specs 

LINKFLAGS=-Wl,-Map=$(TARGET).map
LINKSCRIPT=$(COMMON_DIR)/STM32F091RCTX_FLASH.ld
