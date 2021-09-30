############################################################
#
# Makefile for Whittier College ARM Cortex projects
#
# Jeff Lutgen
#
#############################################################

# Define SRCS in Makefile in project directory, then include this file

COMMON_DIR=../common
OPENCM3_DIR=../../libopencm3

# Processor type (used by linker script generator)
DEVICE=stm32f091rc

# Configure variables for linker script generation,
# changing default "generated.$(DEVICE).ld"
include $(OPENCM3_DIR)/mk/genlink-config.mk
LDSCRIPT=$(COMMON_DIR)/$(DEVICE).ld

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

# The utility for writing a binary file to the MCU.
WRITE=$(PROG_PREFIX)/STM32_Programmer_CLI

# The output target $(TARGET).elf
TARGET=out

OBJS = $(SRCS:%.c=%.o)

INCLUDES += -I$(OPENCM3_DIR)/include

DEPFLAGS = -MMD -MP

CFLAGS += -ggdb3 -O0 -Wall
CFLAGS += $(ARCH_FLAGS) # Assigned by linkscript generator
CFLAGS += -std=gnu11
CFLAGS += -Wextra -Wshadow
CFLAGS += -Wredundant-decls -Wstrict-prototypes

LDFLAGS += $(ARCH_FLAGS) # Assigned by linkscript generator
LDFLAGS += -T$(LDSCRIPT) # Assigned by linkscript generator
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

# Compile a source file to an object file
%.o: %.c
	@echo
	@echo Compiling source file $< to object file $@
	$(CC) $(DEPFLAGS) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $@ -c $<

# Link all the object files into an elf file
$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo
	@echo Linking objects into elf file $@
	$(LD) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $(TARGET).elf 

# Delete map, object, and elf files, as well as other assorted crud
clean:
	$(RM) *.map *.o *.elf *.d *.dis *~

write: $(TARGET).elf
	openocd -f $(COMMON_DIR)/openocd.cfg -c "program $< verify reset exit"

include $(OPENCM3_DIR)/mk/genlink-rules.mk

.PHONY: all clean write size.stdout
-include $(OBJS:.o=.d)
