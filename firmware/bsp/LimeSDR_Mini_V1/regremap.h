/*
 * regremap.h
 */

#ifndef REGREMAP_H_
#define REGREMAP_H_

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

#endif /* REGREMAP_H_ */