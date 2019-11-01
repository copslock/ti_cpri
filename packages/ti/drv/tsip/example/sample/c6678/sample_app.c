/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/**
 *   @file  sample_app.c
 *
 *   @brief   
 *      This is the application init and callback of TSIP example code. 
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ============================================================================ 
 *  \par
 */

#include "sample.h"

/* Define transmit and receive buffers in application. Application gives data
   in appToTsipBuffer to TSIP, and receives TSIP data to tsipToAppBuffer */
#pragma DATA_SECTION(appToTsipBuffer, ".appData")
#pragma DATA_ALIGN(appToTsipBuffer, 8)
tsipData_t  appToTsipBuffer[BUFSIZE_APP];

#pragma DATA_SECTION(tsipToAppBuffer, ".appData")
#pragma DATA_ALIGN(tsipToAppBuffer, 8)
tsipData_t  tsipToAppBuffer[BUFSIZE_APP];

/* Define application data buffer descriptors */
#pragma DATA_SECTION(toTsip, ".appData")
#pragma DATA_ALIGN(toTsip, 8)
appBuf_t toTsip;
#pragma DATA_SECTION(fromTsip, ".appData")
#pragma DATA_ALIGN(fromTsip, 8)
appBuf_t fromTsip;

/* Event to trigger data verification */
extern Event_Handle   dataVerifyEvent;

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This functions advances buffer pointer by nelem
 *
 *  @retval
 *      Return the buffer pointer before the advance
 */
tsipData_t *appBufAdvancePointer (appBuf_t *buf, UInt16 nelem, UInt16* flag)
{
  UInt16 *index;    /* The index to modify */
  UInt16  startI;   /* The index on entry  */
  UInt16  lastI;    /* The last valid index  */
  tsipData_t *retIndex = NULL; /* The return value    */

  lastI = (buf->size/nelem -1) * nelem;
  index = &(buf->indexForTsip);

  /* Store the original value of the index */
  startI = *index;

  /* Advance the index. Check for buffer staying at the last valid index */
  if (*index >= lastI) {
    *index = lastI;
    *flag = 1;
  }
  else
    *index = *index + nelem;

  /* Return either a pointer to the original buffer */
  retIndex = &(buf->base[startI]);

  return(retIndex);
}

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This is the callback of TSIP ISR on the APP-TSIP direction
 *
 *  @retval
 *      None
 */
void appToTsip(void* dummy1, tsipData_t **toTsipData, tsipData_t **dummy2,
               uint32_t timestamp, uint16_t nSamples)
{
    Uint16 endFlag = 0;
    *toTsipData = appBufAdvancePointer(&toTsip, nSamples, &endFlag);
    if (endFlag) {
      Event_post(dataVerifyEvent, Event_Id_00);
    }
}

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This is the callback of TSIP ISR on the TSIP-APP direction
 *
 *  @retval
 *      None
 */
void tsipToApp(void* dummy1, tsipData_t **fromTsipData, tsipData_t **dummy2, 
               uint32_t timestamp, uint16_t nSamples)
{
    Uint16 endFlag = 0;
    *fromTsipData = appBufAdvancePointer(&fromTsip, nSamples, &endFlag);
    if (endFlag) {
      Event_post(dataVerifyEvent, Event_Id_01);
    }
}

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function initialize application buffers
 *
 *  @retval
 *      None
 */
void appBufInit()
{

  Int32 i;
  Uint8 data = 0x00;

  /* Initialize application buffers */
  /* Application gives a pattern to TSIP */
  for (i=0; i<BUFSIZE_APP; i++)
  {
    appToTsipBuffer[i] = data++;
  }
  /* Zero out application buffer which will receive data from TSIP*/
  memset(tsipToAppBuffer, 0x00, BUFSIZE_APP);

  /* Initialize buffer descriptors: To TSIP direction */
  toTsip.base = appToTsipBuffer;
  toTsip.size = BUFSIZE_APP;
  toTsip.indexForTsip = 0;

  /* Initialize buffer descriptors: from TSIP direction */
  fromTsip.base = tsipToAppBuffer;
  fromTsip.size = BUFSIZE_APP;
  fromTsip.indexForTsip = 0;

}

