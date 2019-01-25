/*
 * ControlLEDs.c
 *
 *  Created on: 5 вер. 2018 р.
 *      Author: Eugene
 */

#include "ControlLEDs.h"
#include "../CustomDrivers/IoExpanders.h"
#include "../CustomDrivers/I2cDriver.h"

LedStates_t s_ledsStates;
LedSequenceParameters_t s_ledSequenceParam[PACK_LED_SIZE];

Task_Struct s_ledsControlTask;
uint8_t s_ledsControlTaskStack[LED_CONTROL_TASK_STACK_SIZE*3];


/*
 * @brief   LEDs control management task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void LedsControl_taskFxn(UArg a0, UArg a1)
{
    //init leds driver
    I2cDriver_init();
    Task_sleep(100*100);
    IoExpanders_init();

    //init for test
//    s_ledsStates.led[2] = ON;
////    s_ledSequenceParam[2].ledNumber = 0;
////    s_ledSequenceParam[2].sequenceNumber = 4;//4
////    s_ledSequenceParam[2].firstLedState = 1;
////    s_ledSequenceParam[2].ts.time[0] = 50;
////    s_ledSequenceParam[2].ts.time[1] = 10;
////    s_ledSequenceParam[2].ts.time[2] = 50;
////    s_ledSequenceParam[2].ts.time[3] = 30;
    s_ledsStates.led[3] = ON;
    s_ledSequenceParam[3].ledNumber = 3;
    s_ledSequenceParam[3].sequenceNumber = 3;//3
    s_ledSequenceParam[3].firstLedState = 0;
    s_ledSequenceParam[3].ts.time[0] = 50;
    s_ledSequenceParam[3].ts.time[1] = 10;
    s_ledSequenceParam[3].ts.time[2] = 50;
    //end init for test


    uint8_t l_numberSequence[LEDSEQUENCE_SIZE] = {0};
    uint8_t l_countTimeSequence[LEDSEQUENCE_SIZE] = {0};
    uint8_t l_ledState[LEDSEQUENCE_SIZE] = {0};// on or off
    uint8_t l_ledFlag[LEDSEQUENCE_SIZE] = {0};// for on init
    uint8_t l_portaValue = 0;
    uint8_t l_portbValue = 0;
    while (1) {

        //work
        for(uint8_t l_numberLed = 0; l_numberLed < LEDSEQUENCE_SIZE; ++l_numberLed){
            if(s_ledsStates.led[l_numberLed] == ON)
            {
                if(s_ledSequenceParam[l_numberLed].sequenceNumber == 0) // this led did`t init
                {
                    continue;
                }

                if(l_ledFlag[l_numberLed] == OFF){
                    if(s_ledSequenceParam[l_numberLed].firstLedState == ON)// first state
                    {
                        //ON this diode
                        LedTurnON(l_numberLed, &l_portaValue, &l_portbValue);
                        l_ledState[l_numberLed] = ON;
                    }else
                    {
                        //OFF this diode
                        LedTurnOFF(l_numberLed, &l_portaValue, &l_portbValue);
                        l_ledState[l_numberLed] = OFF;
                    }
                    l_ledFlag[l_numberLed] = ON;
                }

                if(l_countTimeSequence[l_numberLed] < s_ledSequenceParam[l_numberLed].ts.time[l_numberSequence[l_numberLed]])
                {
                    ++l_countTimeSequence[l_numberLed];
                }else
                {
                    l_countTimeSequence[l_numberLed] = 0;
                    ++l_numberSequence[l_numberLed];

                    if(l_ledState[l_numberLed] == ON)
                    {
                        //OFF this diode
                        LedTurnOFF(l_numberLed, &l_portaValue, &l_portbValue);
                        l_ledState[l_numberLed] = OFF;
                    }else
                    {
                        //ON this diode
                        LedTurnON(l_numberLed, &l_portaValue, &l_portbValue);
                        l_ledState[l_numberLed] = ON;
                    }

                }
                if(s_ledSequenceParam[l_numberLed].sequenceNumber == l_numberSequence[l_numberLed])
                {
                    l_numberSequence[l_numberLed] = 0;
                    l_countTimeSequence[l_numberLed] = 0;
                    l_ledFlag[l_numberLed] = OFF;
                }


            }else
            {
                // if command led off then reset flags
                l_numberSequence[l_numberLed] = 0;
                l_countTimeSequence[l_numberLed] = 0;
                l_ledFlag[l_numberLed] = 0;
            }


        }

        //write state LEDs
        IoExpanders_setLeds(kExpanderA, ~l_portaValue);
        IoExpanders_setLeds(kExpanderB, ~l_portbValue);

        Task_sleep(20*100);//20 ms
    }
}

/*
 * @brief   Task creation function for .
 *
 * @param   none
 *
 * @return  none
 */
void LedsControl_createTask(void)
{
  Task_Params l_taskParams;

  // Configure task
  Task_Params_init(&l_taskParams);
  l_taskParams.stack = s_ledsControlTaskStack;
  l_taskParams.stackSize = LED_CONTROL_TASK_STACK_SIZE*3;
  l_taskParams.priority = LED_CONTROL_TASK_PRIORITY;

  Task_construct(&s_ledsControlTask, LedsControl_taskFxn, &l_taskParams, NULL);

}

void LedTurnON(uint8_t ledNumber, uint8_t *const l_portaValue, uint8_t *const l_portbValue)
{

    if(ledNumber < 8)
    {
        *l_portaValue |= 1<<ledNumber;
    }else
    {
        *l_portbValue |= 1<<(ledNumber-8);
    }

}



void LedTurnOFF(uint8_t ledNumber, uint8_t *const l_portaValue, uint8_t *const l_portbValue)
{
    if(ledNumber < 8)
    {
        *l_portaValue &= ~(1<<ledNumber);
    }else
    {
        *l_portbValue &= ~(1<<(ledNumber-8));
    }
}


/*
 * @brief   func creation function for set led sequence parameter in struct s_ledSequenceParam.
 * Fanc call in project_zero_bds.c c745
 *
 * @param   none
 *
 * @return  none
 */
void setLedSequence(const uint8_t *const l_pData)
{
    uint8_t l_number = 0b01111111 & *l_pData;
    s_ledSequenceParam[l_number].ledNumber = l_number;
    s_ledSequenceParam[l_number].firstLedState = (l_pData[0] & 0b10000000) >> 7;
    s_ledSequenceParam[l_number].sequenceNumber = l_pData[1];
    for(uint8_t l_i = 0; l_i < l_pData[1] && l_i < TIME_SIZE; ++l_i){
        s_ledSequenceParam[l_number].ts.time[l_i] = l_pData[2+l_i];
    }
}


/*
 * @brief   func creation function for set led sequence parameter in struct s_ledSequenceParam.
 * Fanc call in project_zero_bds.c c745
 *
 * @param   none
 *
 * @return  none
 */
void setLedsComand(const uint8_t *const l_pData)
{

    uint8_t l_bitCounter = 0;
    uint8_t l_byteCounter = 4;

    for(uint8_t l_i = 0; l_i <= PACK_LED_SIZE; l_i++)
    {
        if ( (*(l_pData+l_byteCounter) >> l_bitCounter) & 0b00000001)
        {
            s_ledsStates.led[l_i] = ON;
        }else
        {
            s_ledsStates.led[l_i] = OFF;
        }

        l_bitCounter++;

        if (l_bitCounter == 8)
        {
            l_bitCounter = 0;
            l_byteCounter++;
        }


    }

}
