BUILD_DIR?=../build/limesdr_xtrx_platform
LINKER    ?= linker_main_ram.ld

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Add debugging symbol and zero optimization level flags
ifdef DEBUG
CFLAGS += -g -O0
endif

CFLAGS := $(filter-out -Wmissing-prototypes -Wstrict-prototypes, $(CFLAGS))

OBJECTS   = i2c0.o crt0.o main.o
OBJECTS  += lms7002m.o

ifeq ($(TARGET), LIMESDR_XTRX)
OBJECTS += i2c1.o regremap.o Xil_clk_drp.o fpga_flash_qspi.o
else
OBJECTS += csr_access.o spiflash.o
endif

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
	$(RM) *.o firmware.elf firmware.bin firmware.hex .*~ *~ *.map *.d

.PHONY: all clean
