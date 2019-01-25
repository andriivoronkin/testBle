/*
 * Magnetometer.h
 *
 *  Created on:
 *      Author: Eugene
 */

#ifndef APPLICATION_CUSTOMDRIVERS_MAGNETOMETER_H_
#define APPLICATION_CUSTOMDRIVERS_MAGNETOMETER_H_

#include "stdint.h"

uint8_t Magnetometer_init(void);
uint8_t Magnetometer_getValues(int16_t *x, int16_t *y, int16_t *z);
uint8_t Magnetometer_getRelativeValues(volatile float *x, volatile float *y, volatile float *z);

#endif /* APPLICATION_CUSTOMDRIVERS_MAGNETOMETER_H_ */
