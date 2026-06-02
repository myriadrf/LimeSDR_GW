/*
 * regremap.h
 */

#ifndef LimeSDR_USB_REGREMAP_H
#define LimeSDR_USB_REGREMAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void readCSR(uint8_t *address, uint8_t *regdata_array);
    void writeCSR(uint8_t *address, uint8_t *wrdata_array);

#ifdef __cplusplus
}
#endif

#endif /* LimeSDR_USB_REGREMAP_H */
