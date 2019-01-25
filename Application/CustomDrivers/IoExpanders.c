/*
 * IoExtenders.c
 *
 *  Created on: 21 мая 2018 г.
 *      Author: Eugene
 */

#include "IoExpanders.h"
#include "I2cDriver.h"

#define I2C_EXPANDER_A_ADDR              (0x40 >> 1)
#define I2C_EXPANDER_B_ADDR              (0x42 >> 1)

#define PCAL6416_INPUT_PORT_LOW_REG     (0x00)
#define PCAL6416_INPUT_PORT_HIGH_REG    (0x01)
#define PCAL6416_OUTPUT_PORT_LOW_REG    (0x02)
#define PCAL6416_OUTPUT_PORT_HIGH_REG   (0x03)
#define PCAL6416_CONFIG_LOW_REG         (0x06)
#define PCAL6416_CONFIG_HIGH_REG        (0x07)

uint16_t s_valuesA = 0;
uint16_t s_valuesB = 0;

/*
 * @brief   Init expanders: all pins in output mode
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t IoExpanders_init(void)
{
    uint8_t l_writeData[] = {PCAL6416_CONFIG_LOW_REG, 0x00, 0x00};

    if (!I2cDriver_writeReg(I2C_EXPANDER_A_ADDR, l_writeData, 3)) {
//        while (1) ;
    }
    if (!I2cDriver_writeReg(I2C_EXPANDER_B_ADDR, l_writeData, 3)) {
//        while (1) ;
    }

    IoExpanders_write(kExpanderA, 0x00);
    IoExpanders_write(kExpanderB, 0x00);

    return 1;
}

/*
 * @brief   Write data to expander
 *
 * @param   chip    - chip number
 *          values  - pin states
 *
 * @return  1 - ok, 0 - fail.
 */
uint8_t IoExpanders_write(enum IoExpandersEnumChip chip, uint16_t values)
{
    uint8_t l_writeData[3];
    uint8_t l_addr;

    if (chip == kExpanderA) {
        l_addr = I2C_EXPANDER_A_ADDR;
        s_valuesA = values;
    } else {
        l_addr = I2C_EXPANDER_B_ADDR;
        s_valuesB = values;
    }

    l_writeData[0] = PCAL6416_OUTPUT_PORT_LOW_REG;
    l_writeData[1] = (uint8_t)values;
    l_writeData[2] = (uint8_t)(values >> 8);

    I2cDriver_writeReg(l_addr, l_writeData, 3);

    return 1;
}

/*
 * @brief   Write data to expander (motor driver control pins)
 *
 * @param   chip    - chip number
 *          values  - pin states
 *
 * @return  1 - ok, 0 - fail.
 */
uint8_t IoExpanders_setMotor(enum IoExpandersEnumChip chip, uint8_t values)
{
    uint8_t l_writeData[2];
    uint8_t l_addr;

    if (chip == kExpanderA) {
        l_addr = I2C_EXPANDER_A_ADDR;
        s_valuesA &= 0xFFF0;
        s_valuesA |= values;
        l_writeData[1] = (uint8_t)s_valuesA;
    } else {
        l_addr = I2C_EXPANDER_B_ADDR;
        s_valuesB &= 0xFFF0;
        s_valuesB |= values;
        l_writeData[1] = (uint8_t)s_valuesB;
    }

    l_writeData[0] = PCAL6416_OUTPUT_PORT_LOW_REG;

    I2cDriver_writeReg(l_addr, l_writeData, 2);

    return 1;
}

/*
 * @brief   Write data to expander (LEDs)
 *
 * @param   chip    - chip number
 *          values  - LED states
 *
 * @return  1 - ok, 0 - fail.
 */
uint8_t IoExpanders_setLeds(enum IoExpandersEnumChip chip, uint8_t values)
{
    uint8_t l_writeData[2];
    uint8_t l_addr;

    if (chip == kExpanderA) {
        l_addr = I2C_EXPANDER_A_ADDR;
        s_valuesA &= 0x00FF;
        s_valuesA |= (uint16_t)values << 8;
        l_writeData[1] = (uint8_t)(s_valuesA >> 8);
    } else {
        l_addr = I2C_EXPANDER_B_ADDR;
        s_valuesB &= 0x00FF;
        s_valuesB |= (uint16_t)values << 8;
        l_writeData[1] = (uint8_t)(s_valuesB >> 8);
    }

    l_writeData[0] = PCAL6416_OUTPUT_PORT_HIGH_REG;

    I2cDriver_writeReg(l_addr, l_writeData, 2);

    return 1;
}

