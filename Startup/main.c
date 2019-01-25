/******************************************************************************

 @file       main.c

 @brief main entry of the BLE stack sample application.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2013-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_01_50_00_58
 Release Date: 2017-10-17 18:09:51
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include <xdc/runtime/Error.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/display/Display.h>

#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"
#include "peripheral.h"
#include "project_zero.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
#ifdef ICALL_JT
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#else  /* ! ICALL_JT */
bleUserCfg_t user0Cfg = BLE_USER_CFG;
#endif /* ICALL_JT */

#endif // USE_DEFAULT_USER_CFG

#ifdef USE_FPGA
#include <inc/hw_prcm.h>
#endif // USE_FPGA

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#if defined( USE_FPGA )
  #define RFC_MODE_BLE                 PRCM_RFCMODESEL_CURR_MODE1
  #define RFC_MODE_ANT                 PRCM_RFCMODESEL_CURR_MODE4
  #define RFC_MODE_EVERYTHING_BUT_ANT  PRCM_RFCMODESEL_CURR_MODE5
  #define RFC_MODE_EVERYTHING          PRCM_RFCMODESEL_CURR_MODE6
  //
  #define SET_RFC_BLE_MODE(mode) HWREG( PRCM_BASE + PRCM_O_RFCMODESEL ) = (mode)
#endif // USE_FPGA

/*******************************************************************************
 * TYPEDEFS
 */



/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
//uint8_t dataBluetooth[255];


#ifdef CC1350_LAUNCHXL
#ifdef POWER_SAVING
// Power Notify Object for wake-up callbacks
Power_NotifyObj rFSwitchPowerNotifyObj;
static uint8_t rFSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg,
                                uint32_t *clientArg);
#endif //POWER_SAVING

PIN_State  radCtrlState;
PIN_Config radCtrlCfg[] =
{
  Board_DIO1_RFSW   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* RF SW Switch defaults to 2.4GHz path*/
  Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Power to the RF Switch */
  PIN_TERMINATE
};
PIN_Handle radCtrlHandle;
#endif //CC1350_LAUNCHXL

/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle dispHandle;



#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>
#include "CustomDrivers/I2cDriver.h"
#include "CustomDrivers/IoExpanders.h"
#include "CustomDrivers/MotorDriver.h"
#include "CustomDrivers/Accelerometer.h"
#include "CustomDrivers/Magnetometer.h"
#include "Middleware/ControlLEDs.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

static void SteeringMotor_taskFxn(UArg a0, UArg a1);
static void SpeedMotor_taskFxn(UArg a0, UArg a1);

// Task configuration
#define I2C_BUS_TASK_PRIORITY         3   // 0 - idle, 1 - lowest, 15 - highest
#define I2C_BUS_TASK_STACK_SIZE       480 // multiples of 8 only


Task_Struct s_steeringMotorTask;
uint8_t s_steeringMotorTaskStack[I2C_BUS_TASK_STACK_SIZE];
Task_Struct s_speedMotorTask;
uint8_t s_speedMotorTaskStack[I2C_BUS_TASK_STACK_SIZE];
Task_Struct s_accelerometerTask;
uint8_t s_accelerometerTaskStack[I2C_BUS_TASK_STACK_SIZE];//*2
Task_Struct s_magnetometerTask;
uint8_t s_magnetometerTaskStack[I2C_BUS_TASK_STACK_SIZE];//*3
Task_Struct s_regulatorTask;
uint8_t s_regulatorTaskStack[I2C_BUS_TASK_STACK_SIZE];//*3


UART_Handle s_uart;
Watchdog_Handle s_watchdogHandle;

/*
 *  ======== watchdogCallback ========
 *  Watchdog interrupt callback function.
 */
void watchdogCallback(uintptr_t unused)
{
    /* Insert timeout handling code here. */
}

void watchdogInit(void)
{
    Watchdog_Params l_params;

    /* Call board init functions */
    Watchdog_init();

    /* Create and enable a Watchdog with resets disabled */
    Watchdog_Params_init(&l_params);
    l_params.callbackFxn = (Watchdog_Callback)watchdogCallback;
    l_params.resetMode = Watchdog_RESET_ON;
    s_watchdogHandle = Watchdog_open(CtrlCarBoard_WATCHDOG0, &l_params);
    if (s_watchdogHandle == NULL) {
        /* Error opening Watchdog */
        while (1);
    }

    Watchdog_clear(s_watchdogHandle);
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

int32_t s_steeringAngle = 0;
int32_t s_period = 0;
uint8_t s_motorsInitialized = 0;

/*
 * @brief   I2C bus management task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SteeringMotor_taskFxn(UArg a0, UArg a1)
{
    //UART_write(s_uart, "STRM\n", 5);

    I2cDriver_init();
    IoExpanders_init();
    MotorDriver_init();

    s_motorsInitialized = 1;

    for (;;) {
        if (s_steeringAngle > 0) {
            --s_steeringAngle;
            MotorDriver_stepRight(kSteeringMotor, kFullStep2WindingsOn);
        } else if (s_steeringAngle < 0) {
            ++s_steeringAngle;
            MotorDriver_stepLeft(kSteeringMotor, kFullStep2WindingsOn);
        } else {
            MotorDriver_off(kSteeringMotor);
        }
        Task_sleep(10*100);
        MotorDriver_off(kSteeringMotor);
        Task_sleep(15*100);
    }
}

/*
 * @brief   I2C bus management task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SpeedMotor_taskFxn(UArg a0, UArg a1)
{
    //UART_write(s_uart, "Init\n", 5);

    while (!s_motorsInitialized) {
        Task_sleep(10*100);
    }

    for (;;) {
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
    }
}

char s_log[100];

/*
 * @brief   Accelerometer management task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void Accelerometer_taskFxn(UArg a0, UArg a1)
{
    volatile float l_accX, l_accY, l_accZ;

    Task_sleep(100*100);

    I2cDriver_init();
    Task_sleep(100*100);
    Accelerometer_init();


    while (1) {

        Accelerometer_getAccelerations(&l_accX, &l_accY, &l_accZ);

    }
}

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void Accelerometer_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_accelerometerTaskStack;
  l_taskParams.stackSize = I2C_BUS_TASK_STACK_SIZE*2;
  l_taskParams.priority = I2C_BUS_TASK_PRIORITY;

  Task_construct(&s_accelerometerTask, Accelerometer_taskFxn, &l_taskParams, NULL);
}

/*
 * @brief   Magnetometer management task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
float s_mod, s_oY;

static void Magnetometer_taskFxn(UArg a0, UArg a1)
{
    volatile float l_magX, l_magY, l_magZ;
    float l_magXcount = 0, l_magYcount = 0, l_magZcount = 0;
    const float l_kEps = 0.0001;

    Task_sleep(100*100);

    I2cDriver_init();
    Task_sleep(100*100);
    Magnetometer_init();

    while (1) {

        const int countMeasurement = 20;
        for(int l_i = 0; l_i < countMeasurement; l_i++){
            Magnetometer_getRelativeValues(&l_magX, &l_magY, &l_magZ);
            l_magXcount += l_magX;
            l_magYcount += l_magY;
            l_magZcount += l_magZ;
            Task_sleep(1000*5);
        }

        l_magXcount /= countMeasurement;
        l_magYcount /= countMeasurement;
        l_magZcount /= countMeasurement;




        if( l_magZcount >= -1.0 + l_kEps && l_magYcount >= -1.0 + l_kEps )
        {
            //float l_alfa, oX;
            s_mod = sqrt(l_magYcount*l_magYcount + l_magZcount*l_magZcount);

            //oX = asin(l_magXcount / s_mod)*180.0/3.14;
            s_oY = asin(l_magYcount / s_mod)*180.0/3.14;

            //l_alfa = atan(l_magXcount/l_magZcount)*180.0/3.14;

            //sprintf(s_log, "X=%.2f Y=%.2f Z=%.2f mod=%.2f alfa=%.2f oX = %.2f oY = %.2f\r", l_magXcount, l_magYcount, l_magZcount, s_mod, l_alfa, oX, s_oY);
            //sprintf(s_log, "Y=%.2f Z=%.2f oY = %.2f\r",l_magYcount, l_magZcount, s_oY);
            //UART_write(s_uart, s_log, strlen(s_log));
            //Task_sleep(1000);
        }
        l_magXcount = 0;
        l_magYcount = 0;
        l_magZcount = 0;

    }
}

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void Magnetometer_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_magnetometerTaskStack;
  l_taskParams.stackSize = I2C_BUS_TASK_STACK_SIZE*2;
  l_taskParams.priority = I2C_BUS_TASK_PRIORITY;

  Task_construct(&s_magnetometerTask, Magnetometer_taskFxn, &l_taskParams, NULL);
}

void UartInit(void)
{
    /* Call driver init functions */
    UART_init();
    UART_Params l_uartParams;
    /* Initialize and open UART */
    UART_Params_init(&l_uartParams);

    l_uartParams.writeDataMode = UART_DATA_BINARY;
    l_uartParams.readDataMode = UART_DATA_BINARY;
    l_uartParams.readReturnMode = UART_RETURN_FULL;
    l_uartParams.readEcho = UART_ECHO_OFF;
    l_uartParams.baudRate = 115200;

    s_uart = UART_open(CtrlCarBoard_UART0, (UART_Params *) &l_uartParams);

    if (s_uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}



/*
 * @brief   Regulator management task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void Regulator_taskFxn(UArg a0, UArg a1)
{
    //Task_sleep(100*100);

    const int k_lCountStepMotor = 100; // max count step for Steering Motor
    const int k_lKpReg = 1; // Kp for P regulator
    const int k_lLimitsOY = 90; // limit for angle
    const float k_lBedModLevel = 0.08;



    while (1) {

        if(s_mod > k_lBedModLevel)
         {
             s_steeringAngle = floor(s_oY * k_lCountStepMotor * k_lKpReg / k_lLimitsOY);
         }else
         {
             s_steeringAngle = 0; // in future add processing this variant
         }



         if (s_steeringAngle < -25)
         {
             sprintf(s_log, "RIGHT\rmod=%.2f, angle=%.2f steeringAngle=%d\r", s_mod, s_oY, s_steeringAngle);
             UART_write(s_uart, s_log, strlen(s_log));

         } else if(s_steeringAngle > 25)
         {
             sprintf(s_log, "LEFT\rmod=%.2f, angle=%.2f steeringAngle=%d\r", s_mod, s_oY, s_steeringAngle);
             UART_write(s_uart, s_log, strlen(s_log));
         }else
         {
             sprintf(s_log, "FORWARD\rmod=%.2f, angle=%.2f steeringAngle=%d\r", s_mod, s_oY, s_steeringAngle);
             UART_write(s_uart, s_log, strlen(s_log));
         }

        Task_sleep(1000*100);
    }
}

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void Regulator_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_regulatorTaskStack;
  l_taskParams.stackSize = I2C_BUS_TASK_STACK_SIZE*2;
  l_taskParams.priority = I2C_BUS_TASK_PRIORITY;

  Task_construct(&s_regulatorTask, Regulator_taskFxn, &l_taskParams, NULL);

}




/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
int main()
{
#if defined( USE_FPGA )
  HWREG(PRCM_BASE + PRCM_O_PDCTL0) &= ~PRCM_PDCTL0_RFC_ON;
  HWREG(PRCM_BASE + PRCM_O_PDCTL1) &= ~PRCM_PDCTL1_RFC_ON;
#endif // USE_FPGA

  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  PIN_init(BoardGpioInitTable);

#ifdef CC1350_LAUNCHXL
  // Enable 2.4GHz Radio
  radCtrlHandle = PIN_open(&radCtrlState, radCtrlCfg);

#ifdef POWER_SAVING
  Power_registerNotify(&rFSwitchPowerNotifyObj,
                       PowerCC26XX_ENTERING_STANDBY | PowerCC26XX_AWAKE_STANDBY,
                       (Power_NotifyFxn) rFSwitchNotifyCb, NULL);
#endif //POWER_SAVING
#endif //CC1350_LAUNCHXL

#if defined( USE_FPGA )
  // set RFC mode to support BLE
  // Note: This must be done before the RF Core is released from reset!
  SET_RFC_BLE_MODE(RFC_MODE_BLE);
#endif // USE_FPGA

#ifdef CACHE_AS_RAM
  // retain cache during standby
  Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
  Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
#else
  // Enable iCache prefetching
  VIMSConfigure(VIMS_BASE, TRUE, TRUE);
  // Enable cache
  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
#endif //CACHE_AS_RAM

#if !defined( POWER_SAVING ) || defined( USE_FPGA )
  /* Set constraints for Standby, powerdown and idle mode */
  // PowerCC26XX_SB_DISALLOW may be redundant
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif // POWER_SAVING | USE_FPGA

#ifdef ICALL_JT
  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();
#endif  /* ICALL_JT */
  /* Initialize ICall module */
  ICall_init();

  /* Start tasks of external images - Priority 5 */
  ICall_createRemoteTasks();

  /* Kick off profile - Priority 3 */
  GAPRole_createTask();

  ProjectZero_createTask();

  UartInit();
  //watchdogInit();

  //SteeringMotor_createTask();
  //SpeedMotor_createTask();

  //Accelerometer_createTask();
  //Regulator_createTask();
  //Magnetometer_createTask();
  LedsControl_createTask();


  /* enable interrupts and start SYS/BIOS */
  BIOS_start();

  return 0;
}


/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
    UART_write(s_uart, "Assert\n", 7);

#if !defined(Display_DISABLE_ALL)
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_LCD, NULL);
  }

  Display_print0(dispHandle, 0, 0, ">>>STACK ASSERT");
#endif // ! Display_DISABLE_ALL

  // check the assert cause
  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> OUT OF MEMORY!");
#endif // ! Display_DISABLE_ALL
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
#if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL FW ERROR!");
#endif // ! Display_DISABLE_ALL
      }
      else
      {
#if !defined(Display_DISABLE_ALL)
        Display_print0(dispHandle, 0, 0, "***ERROR***");
        Display_print0(dispHandle, 2, 0, ">> INTERNAL ERROR!");
#endif // ! Display_DISABLE_ALL
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL ABORT!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> ICALL TIMEOUT!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> WRONG API CALL!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
      break;

  default:
#if !defined(Display_DISABLE_ALL)
      Display_print0(dispHandle, 0, 0, "***ERROR***");
      Display_print0(dispHandle, 2, 0, ">> DEFAULT SPINLOCK!");
#endif // ! Display_DISABLE_ALL
      HAL_ASSERT_SPINLOCK;
  }

  return;
}


/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * input parameters
 *
 * @param       eb - Pointer to Error Block.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

#if defined (CC1350_LAUNCHXL) && defined (POWER_SAVING)
/*******************************************************************************
 * @fn          rFSwitchNotifyCb
 *
 * @brief       Power driver callback to toggle RF switch on Power state
 *              transitions.
 *
 * input parameters
 *
 * @param   eventType - The state change.
 * @param   eventArg  - Not used.
 * @param   clientArg - Not used.
 *
 * @return  Power_NOTIFYDONE to indicate success.
 */
static uint8_t rFSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg,
                                uint32_t *clientArg)
{
  if (eventType == PowerCC26XX_ENTERING_STANDBY)
  {
    // Power down RF Switch
    PIN_setOutputValue(radCtrlHandle, Board_DIO30_SWPWR, 0);
  }
  else if (eventType == PowerCC26XX_AWAKE_STANDBY)
  {
    // Power up RF Switch
    PIN_setOutputValue(radCtrlHandle, Board_DIO30_SWPWR, 1);
  }

  // Notification handled successfully
  return Power_NOTIFYDONE;
}
#endif //CC1350_LAUNCHXL || POWER_SAVING


/*******************************************************************************
 */
