############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

PREFIX=/Users/jlutgen/arm/gcc-arm-none-eabi/bin
PROG_PREFIX=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin


CROSS=arm-none-eabi
PREFIX=~/arm/gcc-$(CROSS)/bin
CC=$(PREFIX)/$(CROSS)-gcc

INCLUDES= \
-I. \
-I/Users/jlutgen/STM32Cube/Repository/STM32Cube_FW_F0_V1.10.1/Drivers/CMSIS/Include \
-I/Users/jlutgen/STM32Cube/Repository/STM32Cube_FW_F0_V1.10.1/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
-I/Users/jlutgen/STM32Cube/Repository/STM32Cube_FW_F0_V1.10.1/Drivers/STM32F0xx_HAL_Driver/Inc

HAL_DIR=~/STM32Cube/Repository/STM32Cube_FW_F0_V1.10.1/Drivers/STM32F0xx_HAL_Driver/Src

# The bin creator
HX=$(PREFIX)/$(CROSS)-objcopy

# The object dumper
OBJDMP=$(PREFIX)/$(CROSS)-objdump

# The size-reporting tool
SIZE=$(PREFIX)/$(CROSS)-size

# The utility for writing a .bin file to the MCU.
WRITE=$(PROG_PREFIX)/STM32_Programmer_CLI

# The output target $(TARGET).bin
TARGET=out

LINKSCRIPT=STM32F091RCTX_FLASH.ld

# Additional linker flags
LINKFLAGS=--specs=nano.specs -specs=nosys.specs -Wl,-Map=$(TARGET).map,--cref,--gc-sections

# if we have specified a linker script, add it
ifdef LINKSCRIPT
	LINKFLAGS:=$(LINKFLAGS)
endif

# List of object files needed to produce target.
OBJS := $(patsubst %.c, %.o, $(wildcard *.c)) $(patsubst %.s, %.o, $(wildcard *.s)) 

# uart needs dma
HAL_OBJS := \
stm32f0xx_hal_tim.o \
stm32f0xx_hal_tim_ex.o \
stm32f0xx_hal_uart.o \
stm32f0xx_hal_uart_ex.o \
stm32f0xx_hal_rcc.o \
stm32f0xx_hal_rcc_ex.o \
stm32f0xx_hal.o \
stm32f0xx_hal_i2c.o \
stm32f0xx_hal_i2c_ex.o \
stm32f0xx_hal_gpio.o \
stm32f0xx_hal_dma.o \
stm32f0xx_hal_cortex.o \
stm32f0xx_hal_pwr.o \
stm32f0xx_hal_pwr_ex.o \
stm32f0xx_hal_flash.o \
stm32f0xx_hal_flash_ex.o

HDRS := $(wildcard *.h)

PROCESSOR = STM32F091xC
C_DEFS = \
-DUSE_HAL_DRIVER \
-D$(PROCESSOR)

CFLAGS = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -std=gnu11 --specs=nano.specs \
         -specs=nosys.specs -ggdb3 -O0 -Wall -fdata-sections -ffunction-sections $(C_DEFS)

# if on Windows (but not MinGW), use a different RM
ifdef OS
    ifndef MINGW_PREFIX
	RM = del /Q
    endif
endif

# What to do for "make all"
.PHONY: all
all: $(TARGET).bin $(TARGET).dis size.stdout
# Turn the elf file into a bin file.
$(TARGET).bin: $(TARGET).elf
	@echo Creating bin file $@
	$(HX) -O binary $(TARGET).elf $@

.PHONY: size.stdout
size.stdout: $(TARGET).elf
	$(SIZE) $<

# Generate disassembly file.
$(TARGET).dis: $(TARGET).elf
	@echo Creating disassembly file $@
	$(OBJDMP) -h -S $< > $@

# Link all the object files and any local library code used by them into an elf file.
$(TARGET).elf: $(OBJS) $(HAL_OBJS)
	@echo Linking elf file $@
	$(CC) -mcpu=cortex-m0 -T$(LINKSCRIPT) -o $(TARGET).elf $(OBJS) $(HAL_OBJS) $(LINKFLAGS)

# Create an object file for each C file. Force recompile if *any* header has changed.
%.o: %.c $(HDRS)
	@echo Creating object file $@
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

# Create an object file for each assembly file.
%.o: %.s
	@echo Creating object file $@
	$(CC) $(CFLAGS) -c -o $@ $<

# Create object files for needed C files in STM32 HAL library
%.o: $(HAL_DIR)/%.c
	@echo Creating LL object file $@
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

.PHONY: clean
# Delete all bin, map, object, and elf files, and other assorted crud
clean:
	$(RM) *.bin *.map *.o *.a *.elf *.dep *.dis log.* *.xml* *~

.PHONY: write
write: $(TARGET).bin $(TARGET).dis
	$(WRITE) -c port=SWD -w $(TARGETDIR)$(TARGET).bin 0x8000000 -v -rst

.PHONY: termite
termite:
	"/c/Program Files (x86)/Termite/Termite.exe" &

.PHONY: putty
putty:
	"$(TERMEMU)" -serial -sercfg 230400,R "$(PORT)"

.PHONY: screen
screen:
	"$(TERMEMU)" "$(PORT)" 230400,crtscts
