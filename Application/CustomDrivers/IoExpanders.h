/*
 * IoExtenders.h
 *
 *  Created on: 21 мая 2018 г.
 *      Author: Eugene
 */

#ifndef APPLICATION_CUSTOMDRIVERS_IOEXPANDERS_H_
#define APPLICATION_CUSTOMDRIVERS_IOEXPANDERS_H_

#include "stdint.h"

enum IoExpandersEnumChip{
    kExpanderA,
    kExpanderB
};

uint8_t IoExpanders_init(void);
uint8_t IoExpanders_write(enum IoExpandersEnumChip chip, uint16_t values);
uint8_t IoExpanders_setMotor(enum IoExpandersEnumChip chip, uint8_t values);
uint8_t IoExpanders_setLeds(enum IoExpandersEnumChip chip, uint8_t values);


#endif /* APPLICATION_CUSTOMDRIVERS_IOEXPANDERS_H_ */
