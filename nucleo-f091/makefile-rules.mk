############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

# Define SRCS and HDRS in Makefile in project directory, then include this file

COMMON_DIR=../common
OPENCM3_DIR=../../libopencm3

# Processor type (used by linker script generator)
DEVICE=STM32F091RC

include $(OPENCM3_DIR)/mk/genlink-config.mk

ifdef WSL_DISTRO_NAME        # Windows Subsystem for Linux
    # TODO: PROG_PREFIX for WSL
else
	PROG_PREFIX=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin
endif

CROSS=arm-none-eabi

# The C compiler
CC=$(CROSS)-gcc

# The linker
LD=$(CROSS)-gcc

# The bin creator
HX=$(CROSS)-objcopy

# The object dumper
OBJDMP=$(CROSS)-objdump

# The size-reporting tool
SIZE=$(CROSS)-size

# The utility for writing a .bin file to the MCU.
WRITE=$(PROG_PREFIX)/STM32_Programmer_CLI

# The output target $(TARGET).elf
TARGET=out

OBJS = $(SRCS:%.c=%.o)

INCLUDES += -I.
INCLUDES += -I$(OPENCM3_DIR)/include

CFLAGS += -ggdb3 -O0 -Wall
CFLAGS += $(ARCH_FLAGS)
CFLAGS += -std=gnu11
CFLAGS += -Wextra -Wshadow -Wno-unused-variable
CFLAGS += -Wredundant-decls -Wstrict-prototypes
CFLAGS += -MMD

LDFLAGS += $(ARCH_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nosys.specs
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += -Wl,--cref
LDFLAGS += -Wl,--gc-sections

# What to do for "make all"
all: $(TARGET).elf $(TARGET).dis size.stdout

size.stdout: $(TARGET).elf
	@echo
	$(SIZE) $<

# Generate disassembly file.
$(TARGET).dis: $(TARGET).elf
	@echo
	@echo Creating disassembly file $@
	$(OBJDMP) -h -S $< > $@

# Compile each source file to an object file
%.o: %.c
	@echo
	@echo Compiling source file $< to object file $@
	$(CC) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $@ -c $<

# Link all the object files and any local library code used by them into an elf file.
$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo
	@echo Linking objects into elf file $@
	$(LD) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $(TARGET).elf 

# Delete map, object, and elf files, as well as other assorted crud
clean:
	$(RM) *.map *.o *.elf *.d *.dis generated.*.ld *~

write: $(TARGET).elf
	$(WRITE) -c port=SWD -w $(TARGETDIR)$(TARGET).elf -v -rst

include $(OPENCM3_DIR)/mk/genlink-rules.mk

.PHONY: all clean write size.stdout
-include $(OBJS:.o=.d)
