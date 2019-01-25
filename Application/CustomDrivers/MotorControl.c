/*
 * MotorControl.c
 *
 *  Created on: 22 џэт. 2019 у.
 *      Author: Andrii
 */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/Error.h>
#include "CustomDrivers/MotorDriver.h"
#include "CustomDrivers/I2cDriver.h"
#include "CustomDrivers/IoExpanders.h"

static int32_t  s_steeringAngle = 0;
static int32_t  s_period = 0;
static uint8_t  s_motorsInitialized = 0;

static void SteeringMotor_taskFxn(UArg a0, UArg a1);
static void SpeedMotor_taskFxn(UArg a0, UArg a1);
static void testMotors_taskFxn(UArg a0, UArg a1);
static void MotorControl_mutexInit();

// Task configuration
#define I2C_BUS_TASK_PRIORITY         3   // 0 - idle, 1 - lowest, 15 - highest
#define I2C_BUS_TASK_STACK_SIZE       480 // multiples of 8 only

Task_Struct s_steeringMotorTask;
uint8_t s_steeringMotorTaskStack[I2C_BUS_TASK_STACK_SIZE];
Task_Struct s_speedMotorTask;
uint8_t s_speedMotorTaskStack[I2C_BUS_TASK_STACK_SIZE];
Task_Struct s_testMotorsTask;
uint8_t s_testMotorsTaskStack[I2C_BUS_TASK_STACK_SIZE];

Semaphore_Handle s_mutex;
Semaphore_Params s_mutexParameters;
//semParams.mode = Semaphore_Mode_BINARY;

Error_Block s_eb;

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void TestMotors_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_testMotorsTaskStack;
  l_taskParams.stackSize = I2C_BUS_TASK_STACK_SIZE;
  l_taskParams.priority = I2C_BUS_TASK_PRIORITY;

  Task_construct(&s_testMotorsTask, testMotors_taskFxn, &l_taskParams, NULL);
}

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void SteeringMotor_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_steeringMotorTaskStack;
  l_taskParams.stackSize = I2C_BUS_TASK_STACK_SIZE;
  l_taskParams.priority = I2C_BUS_TASK_PRIORITY;

  Task_construct(&s_steeringMotorTask, SteeringMotor_taskFxn, &l_taskParams, NULL);
}

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void SpeedMotor_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_speedMotorTaskStack;
  l_taskParams.stackSize = I2C_BUS_TASK_STACK_SIZE;
  l_taskParams.priority = I2C_BUS_TASK_PRIORITY;

  Task_construct(&s_speedMotorTask, SpeedMotor_taskFxn, &l_taskParams, NULL);
}


static void SteeringMotor_taskFxn(UArg a0, UArg a1)
{
    //UART_write(s_uart, "STRM\n", 5);
    I2cDriver_init();
    IoExpanders_init();
    MotorDriver_init();

    s_motorsInitialized = 1;

    for (;;) {

        //Semaphore_pend(s_mutex, BIOS_WAIT_FOREVER);
        if (s_steeringAngle > 0) {
            --s_steeringAngle;
            MotorDriver_stepRight(kSteeringMotor, kFullStep2WindingsOn);
        } else if (s_steeringAngle < 0) {
            ++s_steeringAngle;
            MotorDriver_stepLeft(kSteeringMotor, kFullStep2WindingsOn);
        } else {
            MotorDriver_off(kSteeringMotor);
        }
        //Semaphore_post(s_mutex);
        Task_sleep(10*100);
        MotorDriver_off(kSteeringMotor);
        Task_sleep(15*100);
    }
}

static void testMotors_taskFxn(UArg a0, UArg a1)
{
    while (!s_motorsInitialized) {
        Task_sleep(10*100);
    }

    while (1) {
        s_steeringAngle = -200;
        while (s_steeringAngle) {
            Task_sleep(100*100);
        }
        s_steeringAngle = +200;
        while (s_steeringAngle) {
            Task_sleep(100*100);
        }
    }
}

static void SpeedMotor_taskFxn(UArg a0, UArg a1)
{
    //UART_write(s_uart, "Init\n", 5);
    while (!s_motorsInitialized) {
        Task_sleep(10*100);
    }

    for (;;) {
        //Semaphore_pend(s_mutex, BIOS_WAIT_FOREVER);
        if ((s_period < 80*100) && (s_period > -80*100)) {
            if (s_period == 0) {
                MotorDriver_off(kTractionMotor);
                Task_sleep(10*100);
            } else if (s_period > 0) {
                MotorDriver_stepRight(kTractionMotor, kFullStepMode);
                if (s_period > 10*100) {
                    Task_sleep(s_period/2);
                    MotorDriver_off(kTractionMotor);
                    Task_sleep(s_period/2);
                } else {
                    Task_sleep(s_period);
                }
            } else {
                MotorDriver_stepLeft(kTractionMotor, kFullStepMode);
                if (s_period < -10*100) {
                    Task_sleep(-s_period/2);
                    MotorDriver_off(kTractionMotor);
                    Task_sleep(-s_period/2);
                } else {
                    Task_sleep(-s_period);
                }
            }
        } else {
            if (s_period == 0) {
                MotorDriver_off(kTractionMotor);
                Task_sleep(10*100);
            } else if (s_period > 0) {
                MotorDriver_stepRight(kTractionMotor, kHalfStepMode);
                if (s_period/2 > 10*100) {
                    Task_sleep(s_period/4);
                    MotorDriver_off(kTractionMotor);
                    Task_sleep(s_period/4);
                } else {
                    Task_sleep(s_period/2);
                }
            } else {
                MotorDriver_stepLeft(kTractionMotor, kHalfStepMode);
                if (s_period/2 < -10*100) {
                    Task_sleep(-s_period/4);
                    MotorDriver_off(kTractionMotor);
                    Task_sleep(-s_period/4);
                } else {
                    Task_sleep(-s_period/2);
                }
            }
        }
    //Semaphore_post(s_mutex);
    }
}

void MotorControl_init()
{
    SteeringMotor_createTask();
    SpeedMotor_createTask();
    //MotorControl_mutexInit();
    //TestMotors_createTask();
}

void MotorControl_setSpeed(int16_t l_pps)
{

    //Semaphore_pend(s_mutex, BIOS_WAIT_FOREVER);
    if (l_pps == 0)
    {
        s_period = 0;
    }
    else if ((l_pps <= 2000) && (l_pps >= -2000))
    {
        s_period = 100000L / l_pps;
    }
    //Semaphore_post(s_mutex);
}


void MotorControl_setSteeringAngle(int16_t angle)
{

    //Semaphore_pend(s_mutex, BIOS_WAIT_FOREVER);

    s_steeringAngle = angle;

    //Semaphore_post(s_mutex);
}

void MotorControl_mutexInit()
{
    s_mutexParameters.mode = Semaphore_Mode_BINARY;
    Semaphore_Params_init(&s_mutexParameters);
    s_mutex = Semaphore_create(0, &s_mutexParameters, &s_eb);
}



