# This defines
# BUILD_DIR, TARGET, LINKER, BSP_PROJECT_DIR
# And any other relevant variables
include env.mak

# Ensure BSP_PROJECT_DIR was defined by the fragment.
ifndef BSP_PROJECT_DIR
$(error BSP_PROJECT_DIR is not defined in env.mak)
endif
# This defines
# BSP_OBJS
include $(BSP_PROJECT_DIR)/bsp.mak

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

BSP_COMMON_DIR = bsp/common
DRIVERS_DIR = drivers
PERIPH_DIR = periph

# Add debugging symbol and zero optimization level flags
ifeq ($(DEBUG),1)
CFLAGS := $(filter-out -Os, $(CFLAGS)) -g -O0
endif

# Add the common BSP directory to the compiler's include path
CFLAGS += -I$(BSP_COMMON_DIR)
# Add the project BSP directory to the compiler's include path
CFLAGS += -I$(BSP_PROJECT_DIR)
# Add the drivers directory to the compiler's include path
CFLAGS += -I$(DRIVERS_DIR)
# Add the periph directory to the compiler's include path
CFLAGS += -I$(PERIPH_DIR)
# Add root directory
CFLAGS += -I.
CFLAGS := $(filter-out -Wmissing-prototypes -Wstrict-prototypes, $(CFLAGS))


OBJECTS = crt0.o main.o
# Following variables are retrieved from bsp.mak, which resides in BSP_PROJECT_DIR
OBJECTS += $(BSP_OBJS)
OBJECTS += $(BSP_COMMON_OBJS)
OBJECTS += $(DRIVERS_OBJS)
OBJECTS += $(PERIPH_OBJS)

ifeq ($(filter LIMESDR_MINI_V1 LIMESDR_MINI_V2, $(TARGET)), $(TARGET))
OBJECTS += csr_access.o spiflash.o
endif

ifeq ($(TARGET), LIMESDR_MINI_V1)
ifeq ($(GOLDEN), 0)
CFLAGS := $(filter-out -Os, $(CFLAGS)) -O0 -ggdb3 -g3
endif
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

VPATH = $(BIOS_DIRECTORY):$(BIOS_DIRECTORY)/cmds:$(CPU_DIRECTORY):$(BSP_PROJECT_DIR):$(BSP_COMMON_DIR):$(DRIVERS_DIR):$(PERIPH_DIR)

#DEBUG PRINTS
$(info Current OBJECTS is: $(OBJECTS))
$(info Current VPATH is: $(VPATH))


%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) *.o firmware.elf firmware.bin firmware.hex .*~ *~ *.map *.d

.PHONY: all clean
