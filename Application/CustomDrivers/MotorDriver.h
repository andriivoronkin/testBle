/*
 * MotorDriver.h
 *
 * Low level control of step motor
 */

#ifndef APPLICATION_CUSTOMDRIVERS_MOTORDRIVER_H_
#define APPLICATION_CUSTOMDRIVERS_MOTORDRIVER_H_

enum MotorDriverEnumMode {
    kFullStepMode = 0,
    kHalfStepMode,
    kFullStep2WindingsOn
};

enum MotorDriverEnumMotor {
    kSteeringMotor = 0,
    kTractionMotor,
    kNumberOfMotors     // Total number of motors
};

void MotorDriver_init(void);
void MotorDriver_stepRight(enum MotorDriverEnumMotor motor, enum MotorDriverEnumMode mode);
void MotorDriver_stepLeft(enum MotorDriverEnumMotor motor, enum MotorDriverEnumMode mode);
void MotorDriver_off(enum MotorDriverEnumMotor motor);


#endif /* APPLICATION_CUSTOMDRIVERS_MOTORDRIVER_H_ */
