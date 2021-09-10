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
$(TARGET).elf: $(SRCS) $(LIB_SRCS) $(HDRS)
	@echo Compiling all sources to elf file $@
	$(CC) $(CFLAGS) $(INCLUDES) -D$(FAMILY) -D$(PROCESSOR) -T$(LINKSCRIPT) $(LINKFLAGS) $(SRCS) $(LIB_SRCS) $(LIBS) -o $(TARGET).elf 

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
