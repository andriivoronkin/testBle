/*
 * I2cBus.h
 *
 *  Created on: 19 мая 2018 г.
 *      Author: Eugene
 */

#ifndef APPLICATION_CUSTOMDRIVERS_I2CDRIVER_H_
#define APPLICATION_CUSTOMDRIVERS_I2CDRIVER_H_

#include "stdint.h"

uint8_t I2cDriver_init(void);
uint8_t I2cDriver_writeReg(uint8_t addr, uint8_t *regAndData, uint8_t len);
uint8_t I2cDriver_readReg(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
uint8_t I2cDriver_writeSingleReg(uint8_t addr, uint8_t reg, uint8_t val);


#endif /* APPLICATION_CUSTOMDRIVERS_I2CDRIVER_H_ */
