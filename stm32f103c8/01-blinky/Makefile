# Processor type
# See stm32f1xx.h for a list of these
PROCESSOR=STM32F103xB

STM_COMMON=/mnt/c/Users/jlutg/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.4
LINKSCRIPT=STM32F103XB_FLASH.ld

# Path to stlink folder for uploading code to board
#STLINK=~/Embedded/stlink

SRCS  = main.c 
SRCS += system_stm32f1xx.c
SRCS += startup_stm32f103xb.s

# Binaries will be generated with this name (.elf, .bin, .hex)
TARGET=test

# Compiler settings

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

# Compiler flags
CFLAGS  = -g3 -O0 -Wall 
CFLAGS += -T$(LINKSCRIPT)
#CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -D$(PROCESSOR)
CFLAGS += -std=gnu11
CFLAGS += -mthumb -mcpu=cortex-m3
CFLAGS += -mfloat-abi=soft
CFLAGS += --specs=nosys.specs
CFLAGS += -I.

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F1xx/Include 
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Include

#vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src \

.PHONY: all

all: $(TARGET).elf

$(TARGET).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

clean:
	rm -f *.o $(TARGET).elf $(TARGET).bin

# Flash the STM32
burn: proj
	$(STLINK)/st-flash write $(TARGET).bin 0x80000000