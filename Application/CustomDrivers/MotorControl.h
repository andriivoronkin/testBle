/*
 * MotorControl.h
 *
 *  Created on: 22 ���. 2019 �.
 *      Author: Andrii
 */

#ifndef APPLICATION_CUSTOMDRIVERS_MOTORCONTROL_H_
#define APPLICATION_CUSTOMDRIVERS_MOTORCONTROL_H_


void MotorControl_init();
void MotorControl_setSpeed(int16_t speed); //int32_t
void MotorControl_setSteeringAngle(int16_t angle); //int32_t

#endif /* APPLICATION_CUSTOMDRIVERS_MOTORCONTROL_H_ */
