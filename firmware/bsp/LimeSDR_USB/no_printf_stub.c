/*
 * LimeSDR-USB has no console: replace printf (also used by LiteX libbase irq/isr error
 * paths) with a stub so vfprintf and its tables are not linked into the CPU ROM.
 */
/* Linked with -Wl,--wrap=printf (bsp.mak BSP_LDFLAGS): every printf call, including the ones in
 * the prebuilt LiteX libbase, resolves here; libc's printf/vfprintf are then garbage-collected. */
/* Arguments are passed in registers (RISC-V calling convention) and ignored, so no varargs
 * prologue is needed. */
int __wrap_printf(void);
int __wrap_printf(void)
{
    return 0;
}
