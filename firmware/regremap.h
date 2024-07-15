/*
 * regremap.h
 *
 *  Created on: May 10, 2024
 *      Author: lab
 */

#ifndef REGREMAP_H_
#define REGREMAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void readCSR(uint8_t *address, uint8_t *regdata_array);
void writeCSR(uint8_t *address, uint8_t *regdata_array);




#endif /* REGREMAP_H_ */



