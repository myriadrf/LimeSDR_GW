/*
 * LimeSDR-USB has no UART/console in the SoC (with_uart = False), so printf output is
 * discarded. Compiling printf out removes vfprintf and the format strings from the CPU ROM
 * (~3 KB), which frees M9K blocks for the DDR features. Force-included via bsp.mak.
 */
#ifndef LIMESDR_USB_NO_PRINTF_H
#define LIMESDR_USB_NO_PRINTF_H
#ifndef __ASSEMBLER__
#include <stdio.h>
#define printf(...) ((void)0)
#endif
#endif
