/*
 * BleParser.h
 *
 *  Created on: 23.01.2019
 *      Author: Andrii
 */

#ifndef APPLICATION_BLEPARSER_H_
#define APPLICATION_BLEPARSER_H_


#include "stdint.h"

#define COMMAND_SIZE 16
#define PACK_LED_SIZE 16 //40
//#define PACK_LED_SIZE 16
#define LED_SIZE 5
#define TIME_SIZE 14
#define MODE_SIZE 2
#define STATUS_SIZE 7
#define LEDSEQUENCE_SIZE 16
#define ACCELERATIONPART1_SIZE 16
#define ACCELERATIONPART2_SIZE 16
#define DECELERATIONPART1_SIZE 16
#define DECELERATIONPART2_SIZE 16

typedef struct {

    uint8_t data[COMMAND_SIZE];

} RowCommand;

typedef struct {

    uint8_t led[PACK_LED_SIZE]; // 1 - on, 0 - off

} RowLedStates;

typedef struct {

    uint8_t data[MODE_SIZE];

} RowMode;

typedef struct {

    uint8_t data[STATUS_SIZE];

} RowStatus;

typedef struct {

    uint8_t time[TIME_SIZE]; // 1..255

} TimesParameters;


typedef struct {

    uint8_t data[LEDSEQUENCE_SIZE];

} RowLedSequence;

typedef struct {

    uint8_t data[ACCELERATIONPART1_SIZE];

} RowAccelerationCurvePart1;

typedef struct {

    uint8_t data[ACCELERATIONPART2_SIZE];

} RowAccelerationCurvePart2;


typedef struct {

    uint8_t data[DECELERATIONPART1_SIZE];

} RowDecelerationCurvePart1;


typedef struct {

    uint8_t data[DECELERATIONPART1_SIZE];

} RowDecelerationCurvePart2;



typedef struct {

    uint8_t led[LED_SIZE]; // 1 - on, 0 - off

} LedStatesParameters;

typedef struct {

    int16_t speed;
    int16_t steering;
    int16_t motor3;
    int16_t motor4;
    int16_t motor5;
    int8_t reserved;

} CommandParameters;

typedef struct {

    uint8_t mode;
    uint8_t stopEngine;
    uint8_t reserved;

} ModeParameters;

typedef struct {

    int16_t speed;
    int16_t angle;
    uint8_t crashSensorSignal;
    uint8_t passageOfMagnet;
    uint8_t batteryLevel;

} StatusParameters;

typedef struct {

    uint8_t numberOfPoints;
    uint8_t reserved1;
    uint16_t time1;
    uint16_t speed1;
    uint16_t time2;
    uint16_t speed2;
    uint16_t time3;
    uint16_t speed3;
    uint16_t reserved2;

} AccelerationCurveParametersPart1;

typedef struct {

    uint16_t time4;
    uint16_t speed4;
    uint16_t time5;
    uint16_t speed5;
    uint16_t time6;
    uint16_t speed6;
    uint16_t time7;
    uint16_t speed7;

} AccelerationCurveParametersPart2;


typedef struct {

    uint8_t numberOfPoints;
    uint8_t reserved1;
    uint16_t time1;
    uint16_t speed1;
    uint16_t time2;
    uint16_t speed2;
    uint16_t time3;
    uint16_t speed3;
    uint16_t reserved2;

} DecelerationCurveParametersPart1;


typedef struct {

    uint16_t time4;
    uint16_t speed4;
    uint16_t time5;
    uint16_t speed5;
    uint16_t time6;
    uint16_t speed6;
    uint16_t time7;
    uint16_t speed7;

} DecelerationCurveParametersPart2;

typedef struct {

    uint8_t ledNumber;
    uint8_t sequenceNumber;
    uint8_t firstLedState;
    TimesParameters ts;

} LedSequenceParameters;

void getCommand(RowCommand *cmd, CommandParameters *cmdp, LedStatesParameters *getleds);
void getMode(RowMode *mod, ModeParameters *mp);
void getStatus(RowStatus *status, StatusParameters *sp);
void getLedSequence(RowLedSequence * ls, LedSequenceParameters *lsp);
//void getLedSequence(const uint8_t *const l_pData);
void getAccelerationCurvePart1(RowAccelerationCurvePart1 *acp1, AccelerationCurveParametersPart1 * acpp1);
void getAccelerationCurvePart2(RowAccelerationCurvePart2 *acp2, AccelerationCurveParametersPart2 * acpp2);
void getDecelerationCurvePart1(RowDecelerationCurvePart1 *dcp1, DecelerationCurveParametersPart1 * dcpp1);
void getDecelerationCurvePart2(RowDecelerationCurvePart2 *dcp2, DecelerationCurveParametersPart2 * dcpp2);
void processCommand (uint8_t *data);
void processMode (uint8_t *data);
void processStatus (uint8_t *data);
void processLedSequence (uint8_t *data);
void processAccelerationCurvePart1 (uint8_t *data);
void processAccelerationCurvePart2 (uint8_t *data);
void processDecelerationCurvePart1 (uint8_t *data);
void processDecelerationCurvePart2 (uint8_t *data);

#endif /* APPLICATION_BLEPARSER_H_ */
