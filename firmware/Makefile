BUILD_DIR?=../build/limesdr_xtrx_platform

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Add debugging symbol flag
CFLAGS += -g

OBJECTS   = i2c0.o i2c1.o crt0.o main.o regremap.o lms7002m.o Xil_clk_drp.o fpga_flash_qspi.o

all: firmware.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=../%)

firmware.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
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
	$(RM) $(OBJECTS) firmware.elf firmware.bin .*~ *~

.PHONY: all clean
