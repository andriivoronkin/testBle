/*
 * Accelerometer.c
 *
 *  Created on:
 *      Author: Eugene
 */

#include "Accelerometer.h"
#include "I2cDriver.h"

#define I2C_ACCELEROMETER_ADDR          (0x18)

#define ACCELEROMETER_WHO_AM_I_REG      (0x0F)
#define ACCELEROMETER_STATUS_REG        (0x27)
#define ACCELEROMETER_REFERENCE_REG     (0x26)
#define ACCELEROMETER_INT1_CFG_REG      (0x30)
#define ACCELEROMETER_INT1_SRC_REG      (0x31)
#define ACCELEROMETER_INT1_THS_REG      (0x32)
#define ACCELEROMETER_OUT_X_L_REG       (0x28)
#define ACCELEROMETER_OUT_X_H_REG       (0x29)
#define ACCELEROMETER_OUT_Y_L_REG       (0x2A)
#define ACCELEROMETER_OUT_Y_H_REG       (0x2B)
#define ACCELEROMETER_OUT_Z_L_REG       (0x2C)
#define ACCELEROMETER_OUT_Z_H_REG       (0x2D)
#define ACCELEROMETER_CTRL_REG0_REG     (0x1E)
#define ACCELEROMETER_CTRL_REG1_REG     (0x20)
#define ACCELEROMETER_CTRL_REG2_REG     (0x21)
#define ACCELEROMETER_CTRL_REG3_REG     (0x22)
#define ACCELEROMETER_CTRL_REG4_REG     (0x23)
#define ACCELEROMETER_CTRL_REG5_REG     (0x24)
#define ACCELEROMETER_CTRL_REG6_REG     (0x25)

#define MAGNETOMETER_ADDR               (0x10)
#define MAGNETOMETER_CHIP_ID_REG        (0x40)

/*
 * @brief   Accelerometer init
 *
 *          Resolution: 10 bit
 *          Full scale: ±2 g (32767...-32768)
 *          Frequency: 100 Hz
 *          High-pass filter cutoff frequency: 2 Hz
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t Accelerometer_init(void)
{
//    uint8_t l_readData[1];
//    uint8_t l_writeData[2];
//
//    l_writeData[0] = 0x4B;
//    l_writeData[1] = 0x01;
//    I2cDriver_writeReg(MAGNETOMETER_ADDR, l_writeData, 2);
//    I2cDriver_readReg(MAGNETOMETER_ADDR, MAGNETOMETER_CHIP_ID_REG, l_readData, 1);

    uint8_t l_chipId;

    // Check if accelerometer is connected
    I2cDriver_readReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_WHO_AM_I_REG, &l_chipId, 1);
    if (l_chipId != 0x33) {
        while (1) ;
    }

    // Turn on the sensor, enable X, Y, and Z, ODR = 100 Hz
    I2cDriver_writeSingleReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG1_REG, 0x57);
    // High-pass filter enabled, cutoff frequency=2Hz
    I2cDriver_writeSingleReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG2_REG, 0x08);
    // Interrupts disabled
    I2cDriver_writeSingleReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG3_REG, 0x00);
    // FS = ±2 g
    I2cDriver_writeSingleReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG4_REG, 0x00);
    //
    I2cDriver_writeSingleReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG5_REG, 0x00);

    return 1;
}

/*
 * @brief   Get x, y, z accelerations
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t Accelerometer_getValues(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t l_readData[6];

    I2cDriver_readReg(I2C_ACCELEROMETER_ADDR, ACCELEROMETER_OUT_X_L_REG | (1<<7), l_readData, 6);

    *x = (int16_t)( l_readData[0] | ((uint16_t)(l_readData[1]) << 8) );
    *y = (int16_t)( l_readData[2] | ((uint16_t)(l_readData[3]) << 8) );
    *z = (int16_t)( l_readData[4] | ((uint16_t)(l_readData[5]) << 8) );

    return 1;
}

/*
 * @brief   Get x, y, z accelerations
 *
 * @param   None
 *
 * @return  None.
 */
uint8_t Accelerometer_getAccelerations(volatile float *x, volatile float *y, volatile float *z)
{
    const float kMaxAcc = 2.0;
    const float kMaxVal = 32768.0;
    int16_t l_x, l_y, l_z;

    Accelerometer_getValues(&l_x, &l_y, &l_z);

    *x = (kMaxAcc/kMaxVal)*l_x;
    *y = (kMaxAcc/kMaxVal)*l_y;
    *z = (kMaxAcc/kMaxVal)*l_z;

    return 1;
}
