/*
 * Accelerometer.h
 *
 *  Created on:
 *      Author: Eugene
 */

#ifndef APPLICATION_CUSTOMDRIVERS_ACCELEROMETER_H_
#define APPLICATION_CUSTOMDRIVERS_ACCELEROMETER_H_

#include "stdint.h"

uint8_t Accelerometer_init(void);
uint8_t Accelerometer_getValues(int16_t *x, int16_t *y, int16_t *z);
uint8_t Accelerometer_getAccelerations(volatile float *x, volatile float *y, volatile float *z);

#endif /* APPLICATION_CUSTOMDRIVERS_ACCELEROMETER_H_ */
