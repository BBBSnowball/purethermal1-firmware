/*******************************************************************************
**
**    File NAME: jova_I2C.c
**
**      AUTHOR:  Hart Thomson
**
**      CREATED: 9/25/2012
**  
**      DESCRIPTION: Lepton Device-Specific Driver for the JOVA
**                   Master I2C
**
**      HISTORY:  9/25/2012 HT - Initial Draft 
**
** Copyright 2010, 2011, 2012, 2013 FLIR Systems - Commercial Vision Systems
**
**  All rights reserved.
**
**  Redistribution and use in source and binary forms, with or without
**  modification, are permitted provided that the following conditions are met:
**
**  Redistributions of source code must retain the above copyright notice, this
**  list of conditions and the following disclaimer.
**
**  Redistributions in binary form must reproduce the above copyright notice,
**  this list of conditions and the following disclaimer in the documentation
**  and/or other materials provided with the distribution.
**
**  Neither the name of the Indigo Systems Corporation nor the names of its
**  contributors may be used to endorse or promote products derived from this
**  software without specific prior written permission.
**
**  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
**  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
**  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
**  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
**  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
**  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
**  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
**  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
**  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
**  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**  THE POSSIBILITY OF SUCH DAMAGE.
**
*******************************************************************************/
/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/

#include "LEPTON_Types.h"
#include "LEPTON_ErrorCodes.h"
#include "LEPTON_Macros.h"
#include "stm32_i2c.h"
#include "LEPTON_I2C_Reg.h"
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/******************************************************************************/
/** LOCAL DEFINES                                                            **/
/******************************************************************************/
#define ADDRESS_SIZE_BYTES          (2)
#define MAX_TXRX_SIZE_BYTES         (1024)
#define COMM_TIMEOUT_MS             (500)

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

/******************************************************************************/
/** LOCAL TYPE DEFINITIONS                                                   **/
/******************************************************************************/

/******************************************************************************/
/** PRIVATE DATA DECLARATIONS                                                **/
/******************************************************************************/

extern I2C_HandleTypeDef hi2c1;
LEP_UINT8 txrxdata[MAX_TXRX_SIZE_BYTES];

/******************************************************************************/
/** PRIVATE FUNCTION DECLARATIONS                                            **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/


/******************************************************************************/
/**
 * Performs I2C Master Initialization
 * 
 * @param portID     LEP_UINT16  User specified port ID tag.  Can be used to
 *                   select between multiple cameras
 * 
 * @param BaudRate   Clock speed in kHz. Typically this is 400.
 *                   The Device Specific Driver will try to match the desired
 *                   speed.  This parameter is updated to the actual speed the
 *                   driver can use.
 * 
 * @return LEP_RESULT  0 if all goes well, errno otherwise
 */
LEP_RESULT DEV_I2C_MasterInit(LEP_UINT16 portID, 
                              LEP_UINT16 *BaudRate)
{
    LEP_RESULT result = LEP_OK;

    /* Place Device-Specific Interface here
    */

    // port is already initialized, so nothing to do.

    return(result);
}

/**
 * Closes the I2C driver connection.
 * 
 * @return LEP_RESULT  0 if all goes well, errno otherwise.
 */
LEP_RESULT DEV_I2C_MasterClose()
{
    LEP_RESULT result = LEP_OK;

    /* Place Device-Specific Interface here
    */ 

    // meh

    return(result);
}

/**
 * Resets the I2C driver back to the READY state.
 * 
 * @return LEP_RESULT  0 if all goes well, errno otherwise.
 */
LEP_RESULT DEV_I2C_MasterReset(void )
{
    LEP_RESULT result = LEP_OK;

    /* Place Device-Specific Interface here
    */ 

    // meh

    return(result);
}

LEP_RESULT DEV_I2C_MasterReadData(LEP_UINT16  portID,               // User-defined port ID
                                  LEP_UINT8   deviceAddress,        // Lepton Camera I2C Device Address
                                  LEP_UINT16  regAddress,           // Lepton Register Address
                                  LEP_UINT16 *readDataPtr,          // Read DATA buffer pointer
                                  LEP_UINT16  wordsToRead,          // Number of 16-bit words to Read
                                  LEP_UINT16 *numWordsRead,         // Number of 16-bit words actually Read
                                  LEP_UINT16 *status                // Transaction Status
                                 )
{
    LEP_RESULT result = LEP_OK;

    /* Place Device-Specific Interface here
    */ 

    HAL_StatusTypeDef hal_status;
    LEP_UINT16 bytesToRead = wordsToRead << 1;
    LEP_UINT16 wordsRead = wordsToRead;
    LEP_UINT8 txdata[ADDRESS_SIZE_BYTES];
    LEP_UINT16 *dataPtr;
    LEP_UINT16 *writePtr;

    *(LEP_UINT16*)txdata = REVERSE_ENDIENESS_UINT16(regAddress);

    hal_status = HAL_I2C_Mem_Read(&hi2c1, deviceAddress << 1, regAddress, I2C_MEMADD_SIZE_16BIT, txrxdata, bytesToRead, COMM_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        DEBUG_PRINTF("DEV_I2C_MasterReadData::HAL_I2C_Mem_Read\r\n\t(deviceAddress=0x%02x, regAddress=0x%04x, bytesToRead=%d) failed: %d\r\n", deviceAddress, regAddress, bytesToRead, hal_status);
        goto finish_DEV_I2C_MasterReadData;
    }

    *numWordsRead = (LEP_UINT16)(wordsRead);

    dataPtr = (LEP_UINT16*)&txrxdata[0];
    writePtr = readDataPtr;
    while(wordsRead--){
        *writePtr++ = REVERSE_ENDIENESS_UINT16(*dataPtr);
        dataPtr++;
    }

finish_DEV_I2C_MasterReadData:

    if(hal_status != HAL_OK)
    {
        result = LEP_ERROR_I2C_FAIL;
    }
    else
    {
        result = LEP_OK;
    }
    return(result);
}

LEP_RESULT DEV_I2C_MasterWriteData(LEP_UINT16  portID,              // User-defined port ID
                                   LEP_UINT8   deviceAddress,       // Lepton Camera I2C Device Address
                                   LEP_UINT16  regAddress,          // Lepton Register Address
                                   LEP_UINT16 *writeDataPtr,        // Write DATA buffer pointer
                                   LEP_UINT16  wordsToWrite,        // Number of 16-bit words to Write
                                   LEP_UINT16 *numWordsWritten,     // Number of 16-bit words actually written
                                   LEP_UINT16 *status)              // Transaction Status
{
    LEP_RESULT result = LEP_OK;

    HAL_StatusTypeDef hal_status;

    LEP_INT16 bytesOfDataToWrite = (wordsToWrite << 1);
    LEP_INT16 bytesToWrite = bytesOfDataToWrite + ADDRESS_SIZE_BYTES;
    LEP_UINT16 *dataPtr;
    LEP_UINT16 *txPtr;

    *(LEP_UINT16*)txrxdata = REVERSE_ENDIENESS_UINT16(regAddress);
    dataPtr = (LEP_UINT16*)&writeDataPtr[0];
    txPtr = (LEP_UINT16*)&txrxdata[ADDRESS_SIZE_BYTES]; //Don't overwrite the address bytes
    while(wordsToWrite--){
        *txPtr++ = (LEP_UINT16)REVERSE_ENDIENESS_UINT16(*dataPtr);
        dataPtr++;
    }

    hal_status = HAL_I2C_Master_Transmit(&hi2c1, deviceAddress << 1, txrxdata, bytesToWrite, COMM_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        DEBUG_PRINTF("DEV_I2C_MasterWriteData::HAL_I2C_Master_Transmit failed: %d\r\n", hal_status);
        goto finish_DEV_I2C_MasterWriteData;
    }

    *numWordsWritten = (bytesToWrite >> 1);

finish_DEV_I2C_MasterWriteData:

    if(hal_status != HAL_OK)
    {
        result = LEP_ERROR;
    }
    return(result);
}

LEP_RESULT DEV_I2C_MasterReadRegister( LEP_UINT16 portID,
                                       LEP_UINT8  deviceAddress, 
                                       LEP_UINT16 regAddress,
                                       LEP_UINT16 *regValue,     // Number of 16-bit words actually written
                                       LEP_UINT16 *status
                                     )
{
    LEP_RESULT result = LEP_OK;

    LEP_UINT16 wordsActuallyRead;

    /* Place Device-Specific Interface here
    */

    result = DEV_I2C_MasterReadData(portID, deviceAddress, regAddress, regValue, 1 /*1 word*/, &wordsActuallyRead, status);

    return(result);
}

LEP_RESULT DEV_I2C_MasterWriteRegister( LEP_UINT16 portID,
                                        LEP_UINT8  deviceAddress, 
                                        LEP_UINT16 regAddress,
                                        LEP_UINT16 regValue,     // Number of 16-bit words actually written
                                        LEP_UINT16 *status
                                      )
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 wordsActuallyWritten;

    /* Place Device-Specific Interface here
    */

    result = DEV_I2C_MasterWriteData(portID, deviceAddress, regAddress, &regValue, 1, &wordsActuallyWritten, status);

    return(result);
}
LEP_RESULT DEV_I2C_MasterStatus(void )
{
    LEP_RESULT result = LEP_OK;

    /* Place Device-Specific Interface here
    */ 

    return(result);
}


LEP_RESULT DEV_I2C_MasterGenericWriteRead(LEP_UINT16  portID,               // User-defined port ID
                                          LEP_UINT8   deviceAddress,        // Lepton Camera I2C Device Address
                                          LEP_UINT8  *writeDataPtr,         // Write DATA buffer pointer
                                          LEP_INT16  bytesToWrite,          // Number of bytes to Write
                                          LEP_UINT8  *readDataPtr,          // Read DATA buffer pointer
                                          LEP_INT16  bytesToRead            // Number of bytes to Read
                                         )
{
    /* Place Device-Specific Interface here
    */ 

    HAL_StatusTypeDef hal_status;

    if ((bytesToWrite == 1 || bytesToWrite == 2) && bytesToRead >= 0)
    {
        // Special case: Use HAL_I2C_Mem_Read because that seems to be the only way to do it with I2C repeated-start,
        // which is necessary for Melexis MLX90614. Unfortunately, this will only work if the write has 1 or 2 bytes
        // so we fall back to separate accesses in the other cases.

        uint16_t writeData;
        uint16_t writeDataSize;
        if (bytesToWrite == 1)
        {
            writeData = writeDataPtr[0];
            writeDataSize = I2C_MEMADD_SIZE_8BIT;
        }
        else
        {
            writeData = (writeDataPtr[0] << 8) | writeDataPtr[1];
            writeDataSize = I2C_MEMADD_SIZE_16BIT;
        }
        hal_status = HAL_I2C_Mem_Read(&hi2c1, deviceAddress << 1, writeData, writeDataSize, readDataPtr, bytesToRead, COMM_TIMEOUT_MS);

        switch (hal_status)
        {
            case HAL_OK:
                break;
            case HAL_ERROR:
                // This could be NACK but we don't know. Unfortunately, the HAL doesn't tell us the details :-/
                return LEP_ERROR_I2C_NACK_RECEIVED;
            case HAL_BUSY:
                return LEP_ERROR_I2C_BUS_NOT_READY;
            case HAL_TIMEOUT:
                return LEP_TIMEOUT_ERROR;
            default:
                return LEP_ERROR_I2C_FAIL;
        }

        return LEP_OK;
    }

    if (bytesToWrite >= 0)
    {
        hal_status = HAL_I2C_Master_Transmit(&hi2c1, deviceAddress << 1, writeDataPtr, bytesToWrite, COMM_TIMEOUT_MS);

        switch (hal_status)
        {
            case HAL_OK:
                break;
            case HAL_ERROR:
                // This could be NACK but we don't know. Unfortunately, the HAL doesn't tell us the details :-/
                return LEP_ERROR_I2C_NACK_RECEIVED;
            case HAL_BUSY:
                return LEP_ERROR_I2C_BUS_NOT_READY;
            case HAL_TIMEOUT:
                return LEP_TIMEOUT_ERROR;
            default:
                return LEP_ERROR_I2C_FAIL;
        }
    }

    //FIXME I think that we cannot really read 0 bytes. Or maybe we can? We cannot NACK before the first byte but we could send STOP.
    if (bytesToRead >= 0)
    {
        hal_status = HAL_I2C_Master_Receive(&hi2c1, deviceAddress << 1, readDataPtr, bytesToRead, COMM_TIMEOUT_MS);

        switch (hal_status)
        {
            case HAL_OK:
                break;
            case HAL_ERROR:
                // This could be NACK but we don't know. Unfortunately, the HAL doesn't tell us the details :-/
                return LEP_ERROR_I2C_NACK_RECEIVED;
            case HAL_BUSY:
                return LEP_ERROR_I2C_BUS_NOT_READY;
            case HAL_TIMEOUT:
                return LEP_TIMEOUT_ERROR;
            default:
                return LEP_ERROR_I2C_FAIL;
        }
    }

    return LEP_OK;
}

/******************************************************************************/
/** PRIVATE MODULE FUNCTIONS                                                 **/
/******************************************************************************/

