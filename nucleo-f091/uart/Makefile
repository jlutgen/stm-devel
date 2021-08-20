############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

PROCESSOR=STM32F091xC
STM_COMMON=$(HOME)/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
PROG_PREFIX=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin

CROSS=arm-none-eabi
CC=$(CROSS)-gcc

INCLUDES=-I$(STM_COMMON)/Drivers/CMSIS/Include \
		 -I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
		 -I$(STM_COMMON)/Drivers/STM32F0xx_HAL_Driver/Inc

LL_DIR=$(STM_COMMON)/Drivers/STM32F0xx_HAL_Driver/Src

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

SRCS = main.c \
       system_stm32f0xx.c \
	   startup_stm32f091xc.s

LL_SRCS =

HDRS = 

LINKSCRIPT=STM32F091RCTX_FLASH.ld

CFLAGS = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -std=gnu11 --specs=nosys.specs -ggdb3 -O0 -Wall
LINKFLAGS=-Wl,-Map=$(TARGET).map

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
# Force recompile if *any* header has changed.
$(TARGET).elf: $(SRCS) $(LL_SRCS) $(HDRS)
	@echo Compiling all sources to elf file $@
	$(CC) $(CFLAGS) $(INCLUDES) -D$(PROCESSOR) -T$(LINKSCRIPT) $(LINKFLAGS) -o $(TARGET).elf $(SRCS) $(LL_SRCS)

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
	"$(TERMEMU)" "$(PORT)" 115200,crtscts
