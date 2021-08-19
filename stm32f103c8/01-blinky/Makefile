# Processor type
# See stm32f1xx.h for a list of these
PROC_TYPE=STM32F103xB

STM_COMMON=/mnt/c/Users/jlutg/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.4
LD_FILE=STM32F103XB_FLASH.ld

# Path to stlink folder for uploading code to board
#STLINK=~/Embedded/stlink

SRCS  = main.c 
SRCS += system_stm32f1xx.c
SRCS += startup_stm32f103xb.s

# Binaries will be generated with this name (.elf, .bin, .hex)
PROJ_NAME=test

# Compiler settings

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

# Compiler flags
CFLAGS  = -g3 -O0 -Wall 
CFLAGS += -T$(LD_FILE)
#CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -D$(PROC_TYPE)
CFLAGS += -std=gnu11
CFLAGS += -mthumb -mcpu=cortex-m3
CFLAGS += -mfloat-abi=soft
CFLAGS += --specs=nosys.specs
CFLAGS += -I.

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F1xx/Include 
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Include

#vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src \

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32
burn: proj
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x80000000