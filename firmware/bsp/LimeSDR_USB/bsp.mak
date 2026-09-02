BSP_OBJS = bsp.o regremap.o
BSP_COMMON_OBJS = console_func.o lime_litex_helpers.o
DRIVERS_OBJS = litei2c.o spimaster.o
PERIPH_OBJS = LMS.o AD56xx.o LM75.o ADF4002.o i2c_eeprom.o

# Target-isolated compiler section and optimization flags
# DEPFLAGS += -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables
DEPFLAGS += -ffunction-sections
