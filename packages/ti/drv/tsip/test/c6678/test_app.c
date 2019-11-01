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
 *   @file  test_app.c
 *
 *   @brief   
 *      This is the application init and callback of TSIP test code. 
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ============================================================================ 
 *  \par
 */

#include "test.h"

/* One set of buffers per TSIP port */

/* Define transmit and receive buffers in application. Application gives data
   in appToTsipBuffer to TSIP, and receives TSIP data to tsipToAppBuffer */
#pragma DATA_SECTION(appToTsipBuffer0, ".appData")
#pragma DATA_ALIGN(appToTsipBuffer0, 8)
tsipData_t  appToTsipBuffer0[NUM_USED_TIME_SLOTS * BUFSIZE_APP];

#pragma DATA_SECTION(appToTsipBuffer1, ".appData")
#pragma DATA_ALIGN(appToTsipBuffer1, 8)
tsipData_t  appToTsipBuffer1[NUM_USED_TIME_SLOTS * BUFSIZE_APP];

#pragma DATA_SECTION(tsipToAppBuffer0, ".appData")
#pragma DATA_ALIGN(tsipToAppBuffer0, 8)
tsipData_t  tsipToAppBuffer0[NUM_USED_TIME_SLOTS * BUFSIZE_APP];

#pragma DATA_SECTION(tsipToAppBuffer1, ".appData")
#pragma DATA_ALIGN(tsipToAppBuffer1, 8)
tsipData_t  tsipToAppBuffer1[NUM_USED_TIME_SLOTS * BUFSIZE_APP];


/* Define application data buffer descriptors */
#pragma DATA_SECTION(toTsip0, ".appData")
#pragma DATA_ALIGN(toTsip0, 8)
appBuf_t toTsip0[NUM_USED_TIME_SLOTS];
#pragma DATA_SECTION(fromTsip0, ".appData")
#pragma DATA_ALIGN(fromTsip0, 8)
appBuf_t fromTsip0[NUM_USED_TIME_SLOTS];

#pragma DATA_SECTION(toTsip1, ".appData")
#pragma DATA_ALIGN(toTsip1, 8)
appBuf_t toTsip1[NUM_USED_TIME_SLOTS];
#pragma DATA_SECTION(fromTsip1, ".appData")
#pragma DATA_ALIGN(fromTsip1, 8)
appBuf_t fromTsip1[NUM_USED_TIME_SLOTS];

Uint16 endFlagA2T[NUM_USED_TSIP_PORTS * NUM_USED_TIME_SLOTS];
Uint16 endFlagT2A[NUM_USED_TSIP_PORTS * NUM_USED_TIME_SLOTS];

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
    *flag |= 1;
     //System_printf ("--Last Index = %d\n",lastI);
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
void appToTsip(void* context, tsipData_t **toTsipData, tsipData_t **dummy2,
               uint32_t timestamp, uint16_t nSamples)
{

    static Uint16 initFlag=0;
    Uint16 i, index;
    appBuf_t * pBufDesc = (appBuf_t *)context;
    
    if(!initFlag)
    {
      /*initialize endFlag Array*/
      initFlag=1;
      for(i=0;i<NUM_USED_TSIP_PORTS * NUM_USED_TIME_SLOTS; i++)
        endFlagA2T[i]=0;
    }

    /* computing index of the array for the specific (Tsip port, timeslot) combination */
    index = (pBufDesc->TsipPort)*NUM_USED_TIME_SLOTS +  pBufDesc->timeSlot;
    
    *toTsipData = appBufAdvancePointer(pBufDesc, nSamples, &endFlagA2T[index]);
    
    /*check if all time slots in all ports are done*/
    for(i=0;i<NUM_USED_TSIP_PORTS * NUM_USED_TIME_SLOTS; i++)
    {
      if(!endFlagA2T[i])
        return;
    }  
    
    Event_post(dataVerifyEvent, Event_Id_00);
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
void tsipToApp(void* context, tsipData_t **fromTsipData, tsipData_t **dummy2, 
               uint32_t timestamp, uint16_t nSamples)
{
    static Uint16 initFlag=0;
    Uint16 i, index;
    appBuf_t * pBufDesc = (appBuf_t *)context;

    if(!initFlag)
    {
      /*initialize endFlag Array*/
      initFlag=1;
      for(i=0;i<NUM_USED_TSIP_PORTS * NUM_USED_TIME_SLOTS; i++)
        endFlagT2A[i]=0;
    }

    /* computing index of the array for the specific (Tsip port, timeslot) combination */
    index = (pBufDesc->TsipPort)*NUM_USED_TIME_SLOTS +  pBufDesc->timeSlot;

    *fromTsipData = appBufAdvancePointer(pBufDesc, nSamples, &endFlagT2A[index]);

    /*check if all time slots in all ports are done*/
    for(i=0;i<NUM_USED_TSIP_PORTS * NUM_USED_TIME_SLOTS; i++)
    {
      if(!endFlagT2A[i])
        return;
    }  

    Event_post(dataVerifyEvent, Event_Id_01);
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

  Int32 i, j;
  Uint8 data;
  

  memset(tsipToAppBuffer0, 0, NUM_USED_TIME_SLOTS * BUFSIZE_APP);
  memset(tsipToAppBuffer1, 0, NUM_USED_TIME_SLOTS * BUFSIZE_APP);
 
  for(j=0; j<NUM_USED_TIME_SLOTS; j++)
  {
    data=0;
    /* Initialize application buffers */
    /* Application gives a pattern to TSIP */
    for (i=0; i<BUFSIZE_APP; i++)
    {
      appToTsipBuffer0[j*BUFSIZE_APP + i] = data;
      appToTsipBuffer1[j*BUFSIZE_APP + i] = data;
      data++;
      
      #if 0
      if(j==1)
      {
        appToTsipBuffer0[j*BUFSIZE_APP + i] = 0x66;
        appToTsipBuffer1[j*BUFSIZE_APP + i] = 0x57;
      }  
      #endif
    }
    /* Zero out application buffer which will receive data from TSIP*/

    /* Initialize buffer descriptors: To TSIP direction */
    toTsip0[j].base = &appToTsipBuffer0[j*BUFSIZE_APP];
    toTsip0[j].size = BUFSIZE_APP;
    toTsip0[j].indexForTsip = 0;
    toTsip0[j].timeSlot = j;
    toTsip0[j].TsipPort = 0;

    toTsip1[j].base = &appToTsipBuffer1[j*BUFSIZE_APP];
    toTsip1[j].size = BUFSIZE_APP;
    toTsip1[j].indexForTsip = 0;
    toTsip1[j].timeSlot = j;
    toTsip1[j].TsipPort = 1;

    /* Initialize buffer descriptors: from TSIP direction */
    fromTsip0[j].base = &tsipToAppBuffer0[j*BUFSIZE_APP];
    fromTsip0[j].size = BUFSIZE_APP;
    fromTsip0[j].indexForTsip = 0;
    fromTsip0[j].timeSlot =j;
    fromTsip0[j].TsipPort =0;

    fromTsip1[j].base = &tsipToAppBuffer1[j*BUFSIZE_APP];
    fromTsip1[j].size = BUFSIZE_APP;
    fromTsip1[j].indexForTsip = 0;
    fromTsip1[j].timeSlot =j;
    fromTsip1[j].TsipPort =1;
  }
}

