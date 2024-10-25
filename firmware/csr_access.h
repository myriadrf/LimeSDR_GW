#ifndef __CSR_ACCESS_H
#define __CSR_ACCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void fpgacfg_write(uint16_t addr, uint8_t *wdata);
void fpgacfg_read(uint16_t addr, uint8_t *rdata);

void pllcfg_write(uint16_t addr, uint8_t *wdata);
void pllcfg_read(uint16_t addr, uint8_t *rdata);

void tstcfg_write(uint16_t addr, uint8_t *wdata);
void tstcfg_read(uint16_t addr, uint8_t *rdata);

void periphcfg_write(uint16_t addr, uint8_t *wdata);
void periphcfg_read(uint16_t addr, uint8_t *rdata);

#ifdef __cplusplus
}
#endif

#endif /* __CSR_ACCESS_H */
