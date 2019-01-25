/*
 * ControlLEDs.h
 *
 *  Created on:
 *      Author: Eugene
 */
#ifndef CONTROL_LEDS_H
#define CONTROL_LEDS_H

#include <ti/sysbios/knl/Task.h>


#define LED_CONTROL_TASK_STACK_SIZE   480 // multiples of 8 only
#define LED_CONTROL_TASK_PRIORITY         3   // 0 - idle, 1 - lowest, 15 - highest
#define TIME_SIZE 14
#define LED_SIZE 5
#define LEDSEQUENCE_SIZE 16
#define PACK_LED_SIZE 16
#define ON 1
#define OFF 0


typedef struct {

    uint8_t led[PACK_LED_SIZE]; // 1 - on, 0 - off

} LedStates_t;

typedef struct {

    uint8_t time[TIME_SIZE]; // 1..14

} Times_t;

typedef struct {

    uint8_t ledNumber;
    uint8_t sequenceNumber;
    uint8_t firstLedState;
    Times_t ts;

} LedSequenceParameters_t;

void LedsControl_createTask(void);
static void LedsControl_taskFxn(UArg a0, UArg a1);
void LedTurnON(uint8_t ledNumber, uint8_t *const l_portaValue, uint8_t *const l_portbValue);
void LedTurnOFF(uint8_t ledNumber, uint8_t *const l_portaValue, uint8_t *const l_portbValue);
void setLedSequence(const uint8_t *const l_pData);
void setLedsComand(const uint8_t *const l_pData);


#endif  /* CONTROL_LEDS_H */
