/*
 * BleParser.c
 *
 *  Created on: 23.01.2019
 *      Author: Andrii
 */

#include "BleParser.h"
#include "CustomDrivers/MotorControl.h"
#include "CustomDrivers/IoExpanders.h"
#include "Middleware/ControlLEDs.h"

static float s_previousSpeed =0;



void getCommand(RowCommand *cmd, CommandParameters *cmdp, LedStatesParameters *getleds)
{

    cmdp->speed = (cmd->data[1] << 8) | cmd->data[0];
    cmdp->steering = (cmd->data[3] << 8) | cmd->data[2];

    int BitShift = 0;
    int ByteCounter = 0;
    int Counter = 4;

    for (int i = 0; i < PACK_LED_SIZE + 1; i++)
    {
        if ((cmd->data[Counter] >> BitShift) & 1u)
        {
            getleds->led[ByteCounter] |= 1 << BitShift;
        }

        BitShift++;

        if (BitShift == 8)
        {
            BitShift = 0;
            ByteCounter++;
            Counter ++;
        }
    }

    cmdp->motor3 = (cmd->data[10] << 8) | cmd->data[9];
    cmdp->motor4 = (cmd->data[12] << 8) | cmd->data[11];
    cmdp->motor5 = (cmd->data[14] << 8) | cmd->data[13];
    cmdp->reserved = cmd->data[15] ;
}

void getMode(RowMode *mod, ModeParameters *mp)
{
    mp->mode = mod->data[0];
    if ((mod->data[1] >> 7) & 1u)
    {
        mp->stopEngine = 1;
    }
    else
    {
        mp->stopEngine = 0;
    }

}

void getStatus(RowStatus *status, StatusParameters *sp)
{
    sp->speed = (status->data[1] << 8) | status->data[0];
    sp->angle = (status->data[3] << 8) | status->data[2];
    sp->crashSensorSignal = status->data[4];
    sp->passageOfMagnet = status->data[5];
    sp->batteryLevel = status->data[6];
}

void getLedSequence(RowLedSequence * ls, LedSequenceParameters *lsp)
{
    lsp->firstLedState = (ls->data[0] & 0b10000000) >> 7;
    lsp->ledNumber = ls->data[0];
    lsp->sequenceNumber = ls->data[1];
    int j = 0;

    for (int i = 2; i <LEDSEQUENCE_SIZE; i++)
    {
        lsp->ts.time[j] = ls->data[i];
        j++;
    }
}

/*
void getLedSequence(const uint8_t *const l_pData)
{
    uint8_t l_number = 0b01111111 & *l_pData;
    s_ledSequenceParam[l_number].ledNumber = l_number;
    s_ledSequenceParam[l_number].firstLedState = (l_pData[0] & 0b10000000) >> 7;
    s_ledSequenceParam[l_number].sequenceNumber = l_pData[1];
    for(uint8_t l_i = 0; l_i < l_pData[1] && l_i < TIME_SIZE; ++l_i)
    {
        s_ledSequenceParam[l_number].ts.time[l_i] = l_pData[2+l_i];
    }
}
*/

void getAccelerationCurvePart1(RowAccelerationCurvePart1 *acp1, AccelerationCurveParametersPart1 * acpp1)
{
    acpp1->numberOfPoints = acp1->data[0];
    acpp1->reserved1 = acp1->data[1];
    acpp1->time1 = (acp1->data[3] << 8) | acp1->data[2];
    acpp1->speed1 = (acp1->data[5] << 8) | acp1->data[4];
    acpp1->time2 = (acp1->data[7] << 8) | acp1->data[6];
    acpp1->speed2 = (acp1->data[9] << 8) | acp1->data[8];
    acpp1->time3 = (acp1->data[11] << 8) | acp1->data[10];
    acpp1->speed3 = (acp1->data[13] << 8) | acp1->data[12];
    acpp1->reserved2 = (acp1->data[15] << 8) | acp1->data[14];
}


void getAccelerationCurvePart2(RowAccelerationCurvePart2 *acp2, AccelerationCurveParametersPart2 * acpp2)
{
    acpp2->time4 = (acp2->data[1] << 8) | acp2->data[0];
    acpp2->speed4 = (acp2->data[3] << 8) | acp2->data[2];
    acpp2->time5 = (acp2->data[5] << 8) | acp2->data[4];
    acpp2->speed5 = (acp2->data[7] << 8) | acp2->data[6];
    acpp2->time6 = (acp2->data[9] << 8) | acp2->data[8];
    acpp2->speed6 = (acp2->data[11] << 8) | acp2->data[10];
    acpp2->time7 = (acp2->data[13] << 8) | acp2->data[12];
    acpp2->speed7 = (acp2->data[15] << 8) | acp2->data[14];
}


void getDecelerationCurvePart1(RowDecelerationCurvePart1 *dcp1, DecelerationCurveParametersPart1 * dcpp1)
{
    dcpp1->numberOfPoints = dcp1->data[0];
    dcpp1->reserved1 = dcp1->data[1];
    dcpp1->time1 = (dcp1->data[3] << 8) | dcp1->data[2];
    dcpp1->speed1 = (dcp1->data[5] << 8) | dcp1->data[4];
    dcpp1->time2 = (dcp1->data[7] << 8) | dcp1->data[6];
    dcpp1->speed2 = (dcp1->data[9] << 8) | dcp1->data[8];
    dcpp1->time3 = (dcp1->data[11] << 8) | dcp1->data[10];
    dcpp1->speed3 = (dcp1->data[13] << 8) | dcp1->data[12];
    dcpp1->reserved2 = (dcp1->data[15] << 8) | dcp1->data[14];
}

void getDecelerationCurvePart2(RowDecelerationCurvePart2 *dcp2, DecelerationCurveParametersPart2 * dcpp2)
{
    dcpp2->time4 = (dcp2->data[1] << 8) | dcp2->data[0];
    dcpp2->speed4 = (dcp2->data[3] << 8) | dcp2->data[2];
    dcpp2->time5 = (dcp2->data[5] << 8) | dcp2->data[4];
    dcpp2->speed5 = (dcp2->data[7] << 8) | dcp2->data[6];
    dcpp2->time6 = (dcp2->data[9] << 8) | dcp2->data[8];
    dcpp2->speed6 = (dcp2->data[11] << 8) | dcp2->data[10];
    dcpp2->time7 = (dcp2->data[13] << 8) | dcp2->data[12];
    dcpp2->speed7 = (dcp2->data[15] << 8) | dcp2->data[14];
}

void processCommand (uint8_t *data)
{
    CommandParameters cmdp;
    LedStatesParameters l_leds;


    enum IoExpandersEnumChip chip_a = kExpanderA;
    enum IoExpandersEnumChip chip_b = kExpanderB;
    IoExpanders_setLeds(chip_a, 255);

    getCommand((RowCommand*)data, &cmdp, &l_leds);
    //MotorControl_setSpeed((int16_t)cmdp.speed);
    //MotorControl_setSteeringAngle((int16_t)cmdp.steering);
    float speed = (float)cmdp.speed/100.0;

    if (speed > s_previousSpeed)
    {
        IoExpanders_setLeds(chip_b, 255);
    }
    else if (speed < s_previousSpeed)
    {
        IoExpanders_setLeds(chip_b, 254);
    }
    else if (speed = s_previousSpeed)
    {
        IoExpanders_setLeds(chip_b, 255);
    }

    s_previousSpeed = speed;

    float pps = 10*speed;

    MotorControl_setSpeed((int16_t)pps);
    MotorControl_setSteeringAngle((int16_t)cmdp.steering);
    //setLedsComand(&l_leds);

    //IoExpanders_setLeds(chip_a, l_leds.led[0]);
    //IoExpanders_setLeds(chip_b, l_leds.led[1]);

}

void processMode (uint8_t *data)
{
    ModeParameters mp;
    getMode((RowMode *)data, &mp);
}

void processStatus (uint8_t *data)
{
    ;
}
void processLedSequence (uint8_t *data)
{
    LedSequenceParameters lsp;
    getLedSequence((RowLedSequence *)data, &lsp);
    //setLedSequence (&lsp);

}
void processAccelerationCurvePart1 (uint8_t *data)
{
    AccelerationCurveParametersPart1 acpp1;
    getAccelerationCurvePart1((RowAccelerationCurvePart1 *)data, &acpp1);
}
void processAccelerationCurvePart2 (uint8_t *data)
{
   AccelerationCurveParametersPart2 acpp2;
   getAccelerationCurvePart2((RowAccelerationCurvePart2 *)data, &acpp2);
}
void processDecelerationCurvePart1 (uint8_t *data)
{
    DecelerationCurveParametersPart1 dcpp1;
    getDecelerationCurvePart1((RowDecelerationCurvePart1 *)data, &dcpp1);
}
void processDecelerationCurvePart2 (uint8_t *data)
{
    DecelerationCurveParametersPart2 dcpp2;
    getDecelerationCurvePart2((RowDecelerationCurvePart2 *)data, &dcpp2);
}
