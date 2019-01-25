/*
 * MotorDriver.c
 *
 * Low level control of step motor
 */

#include "board.h"
#include <ti/drivers/PIN.h>
#include "MotorDriver.h"
#include "IoExpanders.h"

#define MAX_STEPS (8)

enum MotorDriverEnumDriverPins {
    kMotorDriverIn1 = 1<<0,
    kMotorDriverIn2 = 1<<1,
    kMotorDriverIn3 = 1<<2,
    kMotorDriverIn4 = 1<<3,
};

PIN_State s_motorPinStates; // For Sleep input control

// Output combinations for motor stepping control (half step mode)
const uint8_t s_kMotorStepsTable[MAX_STEPS] = {
                              kMotorDriverIn1,                      // Half, Full step
                              kMotorDriverIn1 | kMotorDriverIn3,    // Half
                              kMotorDriverIn3,                      // Half, Full
                              kMotorDriverIn2 | kMotorDriverIn3,    // Half
                              kMotorDriverIn2,                      // Half, Full
                              kMotorDriverIn2 | kMotorDriverIn4,    // Half
                              kMotorDriverIn4,                      // Half, Full
                              kMotorDriverIn1 | kMotorDriverIn4     // Half
};

// Rotor position points to s_kMotorStepsTable element
uint8_t s_rotorPositions[kNumberOfMotors] = {0};

/**
  * @brief Set states of driver control pins
  *
  * @param[in] motor      Motor number
  * @param[in] pinStates  Pin states (4 least significant bits)
  *
  * @return none
  */
void MotorDriver_setDriverPins(enum MotorDriverEnumMotor motor, uint8_t pinStates)
{
    if (motor == kSteeringMotor) {
        IoExpanders_setMotor(kExpanderA, pinStates);
    } else {
        IoExpanders_setMotor(kExpanderB, pinStates);
    }
}

/**
  * @brief Motor driver initialization
  *
  * @return none
  */
void MotorDriver_init(void)
{
    // Initialization of SLEEP input of motor drivers
    const PIN_Config l_kPinConfig[] = {
                                       CtrlCarBoard_HOLD | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                                       PIN_TERMINATE
    };

    // Get handle to this collection of pins
    if (!PIN_open(&s_motorPinStates, l_kPinConfig)) {
        // Handle allocation error
    }

    PIN_setOutputValue(&s_motorPinStates, CtrlCarBoard_HOLD, 1);
}

/**
  * @brief Do one step right
  *
  * @param[in] motor  Motor number
  * @param[in] mode   Full / Half step mode
  *
  * @return none
  */
void MotorDriver_stepRight(enum MotorDriverEnumMotor motor, enum MotorDriverEnumMode mode)
{
    uint8_t l_rotorPosition;

    l_rotorPosition= s_rotorPositions[motor];

    if (mode == kHalfStepMode) {
        l_rotorPosition += 1;
    } else if (mode == kFullStep2WindingsOn) {
        switch (l_rotorPosition) {
            case 1:
                l_rotorPosition = 3;
                break;
            case 3:
                l_rotorPosition = 5;
                break;
            case 5:
                l_rotorPosition = 7;
                break;
            case 7:
                l_rotorPosition = 1;
                break;
            default:
                l_rotorPosition = 1;
                break;
        }
    } else { // Full step mode
        // Adjusting to full step
        l_rotorPosition = l_rotorPosition/2;
        l_rotorPosition = l_rotorPosition*2;

        l_rotorPosition += 2;
    }

    if (l_rotorPosition >= MAX_STEPS) {
        l_rotorPosition = 0;
    }
    MotorDriver_setDriverPins(motor, s_kMotorStepsTable[l_rotorPosition]);

    s_rotorPositions[motor] = l_rotorPosition;
}

/**
  * @brief Do one step left
  *
  * @param[in] motor  Motor number
  * @param[in] mode   Full / Half step mode
  *
  * @return none
  */
void MotorDriver_stepLeft(enum MotorDriverEnumMotor motor, enum MotorDriverEnumMode mode)
{
    uint8_t l_rotorPosition;

    l_rotorPosition = s_rotorPositions[motor];

    if (mode == kHalfStepMode) {
        if (l_rotorPosition > 0) {
            l_rotorPosition -= 1;
        } else {
            l_rotorPosition = MAX_STEPS-1;
        }
    } else if (mode == kFullStep2WindingsOn) {
        switch (l_rotorPosition) {
            case 1:
                l_rotorPosition = 7;
                break;
            case 3:
                l_rotorPosition = 1;
                break;
            case 5:
                l_rotorPosition = 3;
                break;
            case 7:
                l_rotorPosition = 5;
                break;
            default:
                l_rotorPosition = 1;
                break;
        }
    } else { // Full step mode
        // Adjusting to full step
        l_rotorPosition++;
        l_rotorPosition = l_rotorPosition/2;
        l_rotorPosition = l_rotorPosition*2;

        if (l_rotorPosition >= 2) {
            l_rotorPosition -= 2;
        } else {
            l_rotorPosition = MAX_STEPS-2;
        }
    }

    MotorDriver_setDriverPins(motor, s_kMotorStepsTable[l_rotorPosition]);

    s_rotorPositions[motor] = l_rotorPosition;
}

/**
  * @brief Turn off motor driver (can be used for power saving)
  *
  * @param[in] motor  Motor number
  *
  * @return none
  */
void MotorDriver_off(enum MotorDriverEnumMotor motor)
{
    MotorDriver_setDriverPins(motor, 0);
}
