############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

ifdef WSL_DISTRO_NAME        # Windows Subsystem for Linux
	STM_COMMON=/mnt/c/Users/jlutg/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
    # TODO: PROG_PREFIX for WSL
else
	STM_COMMON=$(HOME)/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3
	PROG_PREFIX=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin
endif

COMMON_DIR=../common

CROSS=arm-none-eabi

# The assembler
AS=$(CROSS)-as

# The linker
LD=$(CROSS)-ld

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

SRCS = nothing.S

LDSCRIPT=stm32f091x.ld
LDFLAGS=-Map out.map -T $(LDSCRIPT)

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

# Assemble source file to create object file
$(TARGET).o: $(SRCS)
	@echo Assembling to $@
	$(AS) -g -o $(TARGET).o $(SRCS)

# Link all the object files into an elf file.
$(TARGET).elf: $(TARGET).o
	@echo Linking object file to create elf file $@
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(TARGET).o

.PHONY: clean
# Delete all bin, map, object, and elf files, and other assorted crud
clean:
	$(RM) *.bin *.map *.o *.a *.elf *.dep *.dis log.* *.xml* *~

.PHONY: write
write: $(TARGET).bin $(TARGET).dis
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "program out.elf verify reset exit"

.PHONY: putty
putty:
	"$(TERMEMU)" -serial -sercfg 230400,R "$(PORT)"

.PHONY: screen
screen:
	"$(TERMEMU)" "$(PORT)" 115200,crtscts
