BSP_OBJS = bsp.o regremap.o no_printf_stub.o
BSP_COMMON_OBJS = console_func.o lime_litex_helpers.o
DRIVERS_OBJS = litei2c.o spimaster.o
PERIPH_OBJS = LMS.o AD56xx.o LM75.o ADF4002.o i2c_eeprom.o

# Target-isolated compiler section and optimization flags
# DEPFLAGS += -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables
DEPFLAGS += -ffunction-sections
# LimeSDR-USB: also drop unused const data (CPU ROM size, M9K budget)
DEPFLAGS += -fdata-sections
# switch statements as compare chains: jump tables cost ~0.5 KB of ROM (readCSR/writeCSR/main)
DEPFLAGS += -fno-jump-tables

# No console on LimeSDR-USB: compile printf out to save CPU ROM (see no_printf.h)
DEPFLAGS += -include $(BSP_PROJECT_DIR)/no_printf.h
BSP_LDFLAGS += -Wl,--wrap=printf
