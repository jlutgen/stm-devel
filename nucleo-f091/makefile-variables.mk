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
FAMILY=STM32F0

ifdef WSL_DISTRO_NAME        # Windows Subsystem for Linux
	STM_COMMON=/mnt/c/Users/jlutg/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
    # TODO: PROG_PREFIX for WSL
else
	STM_COMMON=$(HOME)/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
	PROG_PREFIX=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin
endif

COMMON_DIR=../common
LIB_DIR=$(STM_COMMON)/Drivers/STM32F0xx_HAL_Driver/Src
OPENCM3_DIR=../../libopencm3

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

SRCS =

INCLUDES = \
	-I. \
	-I$(STM_COMMON)/Drivers/CMSIS/Include \
	-I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
	-I$(STM_COMMON)/Drivers/STM32F0xx_HAL_Driver/Inc \
	-I$(OPENCM3_DIR)/include

CFLAGS = \
	-ggdb3 -O0 -Wall \
	-std=gnu11 \
	-mthumb -mcpu=cortex-m0 \
	-mfloat-abi=soft \
	--specs=nosys.specs \
	-Wextra -Wshadow -Wno-unused-variable \
	-Wredundant-decls -Wstrict-prototypes

LDLIBS = -lopencm3_stm32f0
LDFLAGS  = -L$(OPENCM3_DIR)/lib
LDFLAGS += -T$(LINKSCRIPT)
LDFLAGS += -nostartfiles
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += -Wl,--gc-sections
LINKSCRIPT=$(COMMON_DIR)/stm32f091rc_opencm3.ld
