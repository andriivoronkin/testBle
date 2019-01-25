/*
 * I2cBus.c
 *
 *  Created on: 19 мая 2018 г.
 *      Author: Eugene
 */

/* Example/Board Header files */
#include "board.h"
#include <ti/drivers/I2C.h>
#include <ti/sysbios/knl/Task.h>

I2C_Handle      s_i2c;

/*
 * @brief   Init I2C driver
 *
 * @param   none
 *
 * @return  1 - ok, 0 - fail
 */
uint8_t I2cDriver_init(void)
{
    I2C_Params      l_i2cParams;

    /* Call driver init functions */
    I2C_init();

    /* Create I2C for usage */
    I2C_Params_init(&l_i2cParams);
    l_i2cParams.bitRate = I2C_400kHz;
    s_i2c = I2C_open(Board_I2C_TMP, &l_i2cParams);

    if (s_i2c == NULL) {
        return 0;
    }

    return 1;
}

/*
 * @brief   Write registers
 *
 * @param   addr        - slave address (7 bit)
 * @param   regAndData  - array with data to be sent. First byte should contain
 *                        first register address
 * @param   len         - data length (with register address byte)
 *
 * @return  1 - ok, 0 - fail
 */
uint8_t I2cDriver_writeReg(uint8_t addr, uint8_t *regAndData, uint8_t len)
{
    I2C_Transaction l_i2cTransaction;

    l_i2cTransaction.slaveAddress = addr;
    l_i2cTransaction.writeBuf = regAndData;
    l_i2cTransaction.writeCount = len;
    l_i2cTransaction.readBuf = NULL;
    l_i2cTransaction.readCount = 0;

    if (!I2C_transfer(s_i2c, &l_i2cTransaction)) {
        return 0;
    }

    return 1;
}

/*
 * @brief   Read registers
 *
 * @param   addr        - slave address (7 bit)
 * @param   reg         - first register address
 * @param   data        - data
 * @param   len         - data length
 *
 * @return  1 - ok, 0 - fail
 */
uint8_t I2cDriver_readReg(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t l_writeBuf[1];
    I2C_Transaction l_i2cTransaction;

    l_writeBuf[0] = reg;

    l_i2cTransaction.slaveAddress = addr;
    l_i2cTransaction.writeBuf = l_writeBuf;
    l_i2cTransaction.writeCount = 1;
    l_i2cTransaction.readBuf = data;
    l_i2cTransaction.readCount = len;

    if (!I2C_transfer(s_i2c, &l_i2cTransaction)) {
        return 0;
    }

    return 1;
}
