/******************************************************************************
 * Filename:       Settings.c
 *
 * Description:    This file contains the implementation of the service.
 *
 *                 Generated by:
 *                 BDS version: 1.1.3139.0
 *                 Plugin:      Texas Instruments BLE SDK GATT Server plugin 1.0.9
 *                 Time:        Tue May 29 2018 02:48:00 GMT+02:00
 *

 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#ifdef UARTLOG_ENABLE
#  include "UartLog.h"
#endif

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "settings.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// Settings Service UUID
CONST uint8_t SettingsUUID[ATT_UUID_SIZE] =
{
  SETTINGS_SERV_UUID_BASE128(SETTINGS_SERV_UUID)
};

// LedSequence UUID
CONST uint8_t s_LedSequenceUUID[ATT_UUID_SIZE] =
{
  S_LEDSEQUENCE_UUID_BASE128(S_LEDSEQUENCE_UUID)
};

// AccCurve1 UUID
CONST uint8_t s_AccCurve1UUID[ATT_UUID_SIZE] =
{
  S_ACCCURVE1_UUID_BASE128(S_ACCCURVE1_UUID)
};

// AccCurve2 UUID
CONST uint8_t s_AccCurve2UUID[ATT_UUID_SIZE] =
{
  S_ACCCURVE2_UUID_BASE128(S_ACCCURVE2_UUID)
};

// DeccCurve1 UUID
CONST uint8_t s_DeccCurve1UUID[ATT_UUID_SIZE] =
{
  S_DECCCURVE1_UUID_BASE128(S_DECCCURVE1_UUID)
};

// DeccCurve2 UUID
CONST uint8_t s_DeccCurve2UUID[ATT_UUID_SIZE] =
{
  S_DECCCURVE2_UUID_BASE128(S_DECCCURVE2_UUID)
};


/*********************************************************************
 * LOCAL VARIABLES
 */

static SettingsCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t SettingsDecl = { ATT_UUID_SIZE, SettingsUUID };

// Characteristic "LedSequence" Properties (for declaration)
static uint8_t s_LedSequenceProps = GATT_PROP_WRITE;

// Characteristic "LedSequence" Value variable
static uint8_t s_LedSequenceVal[S_LEDSEQUENCE_LEN] = {0};

// Length of data in characteristic "LedSequence" Value variable, initialized to minimal size.
static uint16_t s_LedSequenceValLen = S_LEDSEQUENCE_LEN_MIN;



// Characteristic "AccCurve1" Properties (for declaration)
static uint8_t s_AccCurve1Props = GATT_PROP_WRITE;

// Characteristic "AccCurve1" Value variable
static uint8_t s_AccCurve1Val[S_ACCCURVE1_LEN] = {0};

// Length of data in characteristic "AccCurve1" Value variable, initialized to minimal size.
static uint16_t s_AccCurve1ValLen = S_ACCCURVE1_LEN_MIN;



// Characteristic "AccCurve2" Properties (for declaration)
static uint8_t s_AccCurve2Props = GATT_PROP_WRITE;

// Characteristic "AccCurve2" Value variable
static uint8_t s_AccCurve2Val[S_ACCCURVE2_LEN] = {0};

// Length of data in characteristic "AccCurve2" Value variable, initialized to minimal size.
static uint16_t s_AccCurve2ValLen = S_ACCCURVE2_LEN_MIN;



// Characteristic "DeccCurve1" Properties (for declaration)
static uint8_t s_DeccCurve1Props = GATT_PROP_WRITE;

// Characteristic "DeccCurve1" Value variable
static uint8_t s_DeccCurve1Val[S_DECCCURVE1_LEN] = {0};

// Length of data in characteristic "DeccCurve1" Value variable, initialized to minimal size.
static uint16_t s_DeccCurve1ValLen = S_DECCCURVE1_LEN_MIN;



// Characteristic "DeccCurve2" Properties (for declaration)
static uint8_t s_DeccCurve2Props = GATT_PROP_WRITE;

// Characteristic "DeccCurve2" Value variable
static uint8_t s_DeccCurve2Val[S_DECCCURVE2_LEN] = {0};

// Length of data in characteristic "DeccCurve2" Value variable, initialized to minimal size.
static uint16_t s_DeccCurve2ValLen = S_DECCCURVE2_LEN_MIN;



/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t SettingsAttrTbl[] =
{
  // Settings Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&SettingsDecl
  },
    // LedSequence Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &s_LedSequenceProps
    },
      // LedSequence Characteristic Value
      {
        { ATT_UUID_SIZE, s_LedSequenceUUID },
        GATT_PERMIT_WRITE,
        0,
        s_LedSequenceVal
      },
    // AccCurve1 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &s_AccCurve1Props
    },
      // AccCurve1 Characteristic Value
      {
        { ATT_UUID_SIZE, s_AccCurve1UUID },
        GATT_PERMIT_WRITE,
        0,
        s_AccCurve1Val
      },
    // AccCurve2 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &s_AccCurve2Props
    },
      // AccCurve2 Characteristic Value
      {
        { ATT_UUID_SIZE, s_AccCurve2UUID },
        GATT_PERMIT_WRITE,
        0,
        s_AccCurve2Val
      },
    // DeccCurve1 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &s_DeccCurve1Props
    },
      // DeccCurve1 Characteristic Value
      {
        { ATT_UUID_SIZE, s_DeccCurve1UUID },
        GATT_PERMIT_WRITE,
        0,
        s_DeccCurve1Val
      },
    // DeccCurve2 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &s_DeccCurve2Props
    },
      // DeccCurve2 Characteristic Value
      {
        { ATT_UUID_SIZE, s_DeccCurve2UUID },
        GATT_PERMIT_WRITE,
        0,
        s_DeccCurve2Val
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Settings_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t Settings_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t SettingsCBs =
{
  Settings_ReadAttrCB,  // Read callback function pointer
  Settings_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Settings_AddService- Initializes the Settings service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t Settings_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( SettingsAttrTbl,
                                        GATT_NUM_ATTRS( SettingsAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &SettingsCBs );
  Log_info1("Registered service, %d attributes", (IArg)GATT_NUM_ATTRS( SettingsAttrTbl ));
  return ( status );
}

/*
 * Settings_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Settings_RegisterAppCBs( SettingsCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;
    Log_info1("Registered callbacks to application. Struct %p", (IArg)appCallbacks);
    return ( SUCCESS );
  }
  else
  {
    Log_warning0("Null pointer given for app callbacks.");
    return ( FAILURE );
  }
}

/*
 * Settings_SetParameter - Set a Settings parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t Settings_SetParameter( uint8_t param, uint16_t len, void *value )
{

  switch ( param )
  {
    default:
      Log_error1("SetParameter: Parameter #%d not valid.", (IArg)param);
      return INVALIDPARAMETER;
  }
}


/*
 * Settings_GetParameter - Get a Settings parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t Settings_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case S_LEDSEQUENCE_ID:
      *len = MIN(*len, s_LedSequenceValLen);
      memcpy(value, s_LedSequenceVal, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"LedSequence", (IArg)*len);
      break;

    case S_ACCCURVE1_ID:
      *len = MIN(*len, s_AccCurve1ValLen);
      memcpy(value, s_AccCurve1Val, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"AccCurve1", (IArg)*len);
      break;

    case S_ACCCURVE2_ID:
      *len = MIN(*len, s_AccCurve2ValLen);
      memcpy(value, s_AccCurve2Val, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"AccCurve2", (IArg)*len);
      break;

    case S_DECCCURVE1_ID:
      *len = MIN(*len, s_DeccCurve1ValLen);
      memcpy(value, s_DeccCurve1Val, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"DeccCurve1", (IArg)*len);
      break;

    case S_DECCCURVE2_ID:
      *len = MIN(*len, s_DeccCurve2ValLen);
      memcpy(value, s_DeccCurve2Val, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"DeccCurve2", (IArg)*len);
      break;

    default:
      Log_error1("GetParameter: Parameter #%d not valid.", (IArg)param);
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}

/*********************************************************************
 * @internal
 * @fn          Settings_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref Settings.h) or 0xFF if not found.
 */
static uint8_t Settings_findCharParamId(gattAttribute_t *pAttr)
{
  // Is this a Client Characteristic Configuration Descriptor?
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
    return Settings_findCharParamId(pAttr - 1); // Assume the value attribute precedes CCCD and recurse

  // Is this attribute in "LedSequence"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, s_LedSequenceUUID, pAttr->type.len))
    return S_LEDSEQUENCE_ID;

  // Is this attribute in "AccCurve1"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, s_AccCurve1UUID, pAttr->type.len))
    return S_ACCCURVE1_ID;

  // Is this attribute in "AccCurve2"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, s_AccCurve2UUID, pAttr->type.len))
    return S_ACCCURVE2_ID;

  // Is this attribute in "DeccCurve1"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, s_DeccCurve1UUID, pAttr->type.len))
    return S_DECCCURVE1_ID;

  // Is this attribute in "DeccCurve2"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, s_DeccCurve2UUID, pAttr->type.len))
    return S_DECCCURVE2_ID;

  else
    return 0xFF; // Not found. Return invalid.
}

/*********************************************************************
 * @fn          Settings_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Settings_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  uint8_t paramID = 0xFF;

  // Find settings for the characteristic to be read.
  paramID = Settings_findCharParamId( pAttr );
  switch ( paramID )
  {
    default:
      Log_error0("Attribute was not found.");
      return ATT_ERR_ATTR_NOT_FOUND;
  }
}

/*********************************************************************
 * @fn      Settings_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t Settings_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;
  uint8_t   changeParamID = 0xFF;
  uint16_t writeLenMin;
  uint16_t writeLenMax;
  uint16_t *pValueLenVar;

  // Find settings for the characteristic to be written.
  paramID = Settings_findCharParamId( pAttr );
  switch ( paramID )
  {
    case S_LEDSEQUENCE_ID:
      writeLenMin  = S_LEDSEQUENCE_LEN_MIN;
      writeLenMax  = S_LEDSEQUENCE_LEN;
      pValueLenVar = &s_LedSequenceValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"LedSequence",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for LedSequence can be inserted here */
      break;

    case S_ACCCURVE1_ID:
      writeLenMin  = S_ACCCURVE1_LEN_MIN;
      writeLenMax  = S_ACCCURVE1_LEN;
      pValueLenVar = &s_AccCurve1ValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"AccCurve1",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for AccCurve1 can be inserted here */
      break;

    case S_ACCCURVE2_ID:
      writeLenMin  = S_ACCCURVE2_LEN_MIN;
      writeLenMax  = S_ACCCURVE2_LEN;
      pValueLenVar = &s_AccCurve2ValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"AccCurve2",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for AccCurve2 can be inserted here */
      break;

    case S_DECCCURVE1_ID:
      writeLenMin  = S_DECCCURVE1_LEN_MIN;
      writeLenMax  = S_DECCCURVE1_LEN;
      pValueLenVar = &s_DeccCurve1ValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"DeccCurve1",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for DeccCurve1 can be inserted here */
      break;

    case S_DECCCURVE2_ID:
      writeLenMin  = S_DECCCURVE2_LEN_MIN;
      writeLenMax  = S_DECCCURVE2_LEN;
      pValueLenVar = &s_DeccCurve2ValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"DeccCurve2",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for DeccCurve2 can be inserted here */
      break;

    default:
      Log_error0("Attribute was not found.");
      return ATT_ERR_ATTR_NOT_FOUND;
  }
  // Check whether the length is within bounds.
  if ( offset >= writeLenMax )
  {
    Log_error0("An invalid offset was requested.");
    status = ATT_ERR_INVALID_OFFSET;
  }
  else if ( offset + len > writeLenMax )
  {
    Log_error0("Invalid value length was received.");
    status = ATT_ERR_INVALID_VALUE_SIZE;
  }
  else if ( offset + len < writeLenMin && ( method == ATT_EXECUTE_WRITE_REQ || method == ATT_WRITE_REQ ) )
  {
    // Refuse writes that are lower than minimum.
    // Note: Cannot determine if a Reliable Write (to several chars) is finished, so those will
    //       only be refused if this attribute is the last in the queue (method is execute).
    //       Otherwise, reliable writes are accepted and parsed piecemeal.
    Log_error0("Invalid value length was received.");
    status = ATT_ERR_INVALID_VALUE_SIZE;
  }
  else
  {
    // Copy pValue into the variable we point to from the attribute table.
    memcpy(pAttr->pValue + offset, pValue, len);

    // Only notify application and update length if enough data is written.
    //
    // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
    //       the application will get a callback for every write with an offset + len larger than _LEN_MIN.
    // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
    //       because the write fragments are concatenated before being sent here.
    if ( offset + len >= writeLenMin )
    {
      changeParamID = paramID;
      *pValueLenVar = offset + len; // Update data length.
    }
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (changeParamID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( connHandle, SETTINGS_SERV_UUID, paramID, pValue, len+offset ); // Call app function from stack task context.

  return status;
}
