############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

# Processor type
# See stm32f1xx.h for a list of these
PROCESSOR=STM32F103xB

ifdef WSL_DISTRO_NAME        # Windows Subsystem for Linux
	STM_COMMON=/mnt/c/Users/jlutg/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.4
	PROG_PREFIX=
else
	STM_COMMON=$(HOME)/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.4
	PROG_PREFIX=
endif

LL_DIR=$(STM_COMMON)/Drivers/STM32F1xx_HAL_Driver/Src

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
WRITE=$(PROG_PREFIX)/STM32_Programmer_CL

# The output target $(TARGET).bin, .elf, .hex
TARGET=out

SRCS  = main.c
SRCS += system_stm32f1xx.c
SRCS += startup_stm32f103xb.s

LL_SRCS =

HDRS = 

INCLUDES  =-I.
INCLUDES += -I$(STM_COMMON)/Drivers/CMSIS/Include
INCLUDES += -I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F1xx/Include
INCLUDES += -I$(STM_COMMON)/Drivers/STM32F1xx_HAL_Driver/Inc

CFLAGS  = -ggdb3 -O0 -Wall 
#CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -std=gnu11
CFLAGS += -mthumb -mcpu=cortex-m3
CFLAGS += -mfloat-abi=soft
CFLAGS += --specs=nosys.specs

LINKFLAGS=-Wl,-Map=$(TARGET).map
LINKSCRIPT=STM32F103XB_FLASH.ld

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

.PHONY: putty
putty:
	"$(TERMEMU)" -serial -sercfg 230400,R "$(PORT)"

.PHONY: screen
screen:
	"$(TERMEMU)" "$(PORT)" 115200,crtscts
