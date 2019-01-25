/*
 * Magnetometer.c
 *
 *  Created on:
 *      Author: Eugene
 */

#include "Magnetometer.h"
#include "I2cDriver.h"

#define MAGNETOMETER_ADDR               (0x10)

#define MAGNETOMETER_CHIP_ID_REG        (0x40)
#define MAGNETOMETER_DATA_X_LSB_REG     (0x42)
#define MAGNETOMETER_POWER_MODE_REG     (0x4B)
#define MAGNETOMETER_OP_MODE_REG        (0x4C)
#define MAGNETOMETER_XY_REPETITIONS_REG (0x51)
#define MAGNETOMETER_Z_REPETITIONS_REG  (0x52)

/*
 * @brief   Magnetometer init
 *
 *          Output frequency = 20 Hz
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t Magnetometer_init(void)
{
    // Go to suspended mode
    I2cDriver_writeSingleReg(MAGNETOMETER_ADDR, MAGNETOMETER_POWER_MODE_REG, 0x01);

    // Check if accelerometer is connected
    uint8_t l_chipId;

    I2cDriver_readReg(MAGNETOMETER_ADDR, MAGNETOMETER_CHIP_ID_REG, &l_chipId, 1);
    if (l_chipId != 0x32) {
        while(1);
    }
    /*
    do {
        I2cDriver_readReg(MAGNETOMETER_ADDR, MAGNETOMETER_CHIP_ID_REG, &l_chipId, 1);
        if (l_chipId == 0x32) {
            break;
           }
    } while (1);
    */

    // Normal mode, 20 Hz output data rate
    I2cDriver_writeSingleReg(MAGNETOMETER_ADDR, MAGNETOMETER_OP_MODE_REG, 0x28);
    // 15 repetitions for X and Y axes
    I2cDriver_writeSingleReg(MAGNETOMETER_ADDR, MAGNETOMETER_XY_REPETITIONS_REG, (15-1)/2);
    // 27 repetitions for Z axe
    I2cDriver_writeSingleReg(MAGNETOMETER_ADDR, MAGNETOMETER_Z_REPETITIONS_REG, 27-1);

    return 1;
}

/*
 * @brief   Get x, y, z data
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t Magnetometer_getValues(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t l_readData[6];

    I2cDriver_readReg(MAGNETOMETER_ADDR, MAGNETOMETER_DATA_X_LSB_REG, l_readData, 6);

    // X and Y resolution is 12 bit. Z resolution is 14 bit
    *x = (int16_t)( (l_readData[0] & 0xF8) | ((uint16_t)(l_readData[1]) << 8) );
    *y = (int16_t)( (l_readData[2] & 0xF8) | ((uint16_t)(l_readData[3]) << 8) );
    *z = (int16_t)( (l_readData[4] & 0xFE) | ((uint16_t)(l_readData[5]) << 8) );

    return 1;
}

/*
 * @brief   Get x, y, z relative values (-1.0 ... +1.0 range)
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t Magnetometer_getRelativeValues(volatile float *x, volatile float *y, volatile float *z)
{
    const float kMaxVal = 32768.0;
    int16_t l_x, l_y, l_z;

    Magnetometer_getValues(&l_x, &l_y, &l_z);

    *x = (1.0/kMaxVal)*l_x;
    *y = (1.0/kMaxVal)*l_y;
    *z = (1.0/kMaxVal)*l_z;

    return 1;
}
