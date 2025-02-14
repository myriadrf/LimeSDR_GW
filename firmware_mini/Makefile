BUILD_DIR ?=../build/limesdr_mini_v2
LINKER    ?= linker_main_ram.ld

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Add debugging symbol flag
CFLAGS += -g

CFLAGS := $(filter-out -Wmissing-prototypes -Wstrict-prototypes, $(CFLAGS))

OBJECTS   = spiflash.o i2c0.o crt0.o main.o csr_access.o

all: firmware.bin firmware.hex

%.hex: %.bin
	srec_cat $< -binary -o $@ -Intel

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=../%)

firmware.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T $(LINKER) -N -o $@ \
		$(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-Wl,-Map,$@.map \
		$(LIBS:lib%=-l%)

ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

VPATH = $(BIOS_DIRECTORY):$(BIOS_DIRECTORY)/cmds:$(CPU_DIRECTORY)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) firmware.elf firmware.bin .*~ *~ *.map *.d

.PHONY: all clean
