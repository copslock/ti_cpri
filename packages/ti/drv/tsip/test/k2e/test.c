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
 *   @file  test.c
 *
 *   @brief   
 *      This is the TSIP test code. TSIP runs in data loopback mode.
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

/************************ GLOBAL VARIABLES ********************/
/* Define buffers required by TSIP                            
   Size of buffers depends on the maximal number of timeslots 
   to be used. Example below is for 128 timeslots */

/* Buffer size is based only on the size of the instance*/
#define BUFSIZE_TSIP_INST_SHARED           40

/* Buffer size is function of TSIP_N_PORTS */
#define BUFSIZE_TSIP_HEAP_SHARED           200

/* Buffer size is a function of the number of time slots */
#define BUFSIZE_TSIP_HEAP_TIMESLOT         7168

/* Buffer size is a function of the number of time slots and TSIP_N_PORTS*/
#define BUFSIZE_TSIP_TX_DMA_BUFFER         2560

/* Buffer size is a function of the number of time slots and TSIP_N_PORTS*/
#define BUFSIZE_TSIP_RX_DMA_BUFFER         2560

/* Buffer size is based only on the size of the instance*/
#define BUFSIZE_TSIP_INST_SIZE_PORT        176

/* Buffers used for the Shared TSIP Driver instance */
#pragma DATA_SECTION(memTsipInstShared, ".tsipData")
#pragma DATA_ALIGN(memTsipInstShared, 2)
UInt8 memTsipInstShared[BUFSIZE_TSIP_INST_SHARED];

#pragma DATA_SECTION(memTsipHeapShared, ".tsipData")
#pragma DATA_ALIGN(memTsipHeapShared, 2)
UInt8 memTsipHeapShared[BUFSIZE_TSIP_HEAP_SHARED];

#pragma DATA_SECTION(memTsipHeapTimeslot, ".tsipData")
#pragma DATA_ALIGN(memTsipHeapTimeslot, 8)
UInt8 memTsipHeapTimeslot[BUFSIZE_TSIP_HEAP_TIMESLOT];

#pragma DATA_SECTION(memTsipTxDmaBuf, ".tsipData")
#pragma DATA_ALIGN(memTsipTxDmaBuf, 2)
UInt8 memTsipTxDmaBuf[BUFSIZE_TSIP_TX_DMA_BUFFER];

#pragma DATA_SECTION(memTsipRxDmaBuf, ".tsipData")
#pragma DATA_ALIGN(memTsipRxDmaBuf, 2)
UInt8 memTsipRxDmaBuf[BUFSIZE_TSIP_RX_DMA_BUFFER];


/* Buffer used for the individual TSIP port instances */
#pragma DATA_SECTION(memTsipInstPort, ".tsipData")
#pragma DATA_ALIGN(memTsipInstPort, 8)
UInt8 memTsipInstPort[NUM_USED_TSIP_PORTS * BUFSIZE_TSIP_INST_SIZE_PORT];

/* Define TSIP driver handle and port handle */
#pragma DATA_SECTION(tsipHandle, ".tsipData")
#pragma DATA_ALIGN(tsipHandle, 8)
Tsip_DrvHandle tsipHandle;

#pragma DATA_SECTION(portHandle, ".tsipData")
#pragma DATA_ALIGN(portHandle, 8)
Tsip_PortHandle portHandle[NUM_USED_TSIP_PORTS];

/* Define event which will be used to trigger data verification */
Event_Handle   dataVerifyEvent;

/* TSIP timeslot control structure */
tsipTsControl_t tsCtl;

/* Multicore Parameters */


/* Debug Variables */
Int printSetupProgress=0;
Int debugCheckBuff=0;


/* Defines from test_app.c which servers as the application communicating with TSIP driver */
extern tsipData_t  appToTsipBuffer0[];
extern tsipData_t  tsipToAppBuffer0[];
extern tsipData_t  appToTsipBuffer1[];
extern tsipData_t  tsipToAppBuffer1[];

extern appBuf_t toTsip0[];
extern appBuf_t fromTsip0[];
extern appBuf_t toTsip1[];
extern appBuf_t fromTsip1[];

extern void appBufInit();
extern tsipData_t *appBufAdvancePointer (appBuf_t *buf, UInt16 nelem, UInt16* flag);
extern void tsipToApp(void* dummy1, tsipData_t **fromTsipData, tsipData_t **dummy2, uint32_t timestamp, uint16_t nSamples);
extern void appToTsip(void* dummy1, tsipData_t **toTsipData, tsipData_t **dummy2, uint32_t timestamp, uint16_t nSamples);

extern cregister  volatile unsigned int DNUM;

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function initialize driver buffers
 *
 *  @retval
 *      None
 */
void drvBufInit()
{
  memset(memTsipInstShared,   0, sizeof(memTsipInstShared));
  memset(memTsipHeapShared,   0, sizeof(memTsipHeapShared));
  memset(memTsipHeapTimeslot, 0, sizeof(memTsipHeapTimeslot));
  memset(memTsipTxDmaBuf,     0, sizeof(memTsipTxDmaBuf));
  memset(memTsipTxDmaBuf,     0, sizeof(memTsipTxDmaBuf));
  memset(memTsipRxDmaBuf,     0, sizeof(memTsipRxDmaBuf));
  memset(memTsipInstPort,     0, sizeof(memTsipInstPort));
  memset(portHandle,          0, sizeof(portHandle));
  
}

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function sets up TSIP to transmit and receive the data.
 *
 *  @retval
 *      None
 */
void tsipConfig (tsipSizeInfo_t *sizeCfg, tsipConfig_t *cfg)
{

    /* Provide size information for TSIP */
    sizeCfg->maxChannels = TSIP_MAX_TIMESLOTS;
    sizeCfg->subFrameSize = 8;
    sizeCfg->wordSize = 8;
	sizeCfg->numPorts = NUM_USED_TSIP_PORTS;
	
    /* Global configuration */
    cfg->testMode = TRUE;
    cfg->testModeSelect = CSL_TSIP_TESTMODE_DATA_LOOPBACK;
    cfg->clkRedund = CSL_TSIP_CLKD_REDUN;
    cfg->endian = CSL_TSIP_ENDIAN_LITTLE;
    cfg->priority = CSL_TSIP_PRI_0;
    cfg->maxPriority = CSL_TSIP_PRI_0;

    cfg->sizeCfg = sizeCfg;
    cfg->maxPhase = 10;
    cfg->subFrameCallout=NULL;
    cfg->cxt=NULL; 

    /* Transmit configuration */
    cfg->tx.channel = deviceWhoAmI();
    cfg->tx.frameSize = CSL_TSIP_FRAMESIZE_128;
    cfg->tx.tsPerFrame = 256;
    cfg->tx.clkSrc = CSL_TSIP_CLKSRC_A;
    cfg->tx.dataDelay = 1;
    cfg->tx.bdxDelay = CSL_TSIP_DLY_CTRL_DISABLE;
    cfg->tx.idleDrive = CSL_TSIP_XMTDIS_HIGHIMP;
    cfg->tx.fsyncPol = CSL_TSIP_FSYNCP_ALOW;
    cfg->tx.fsyncClkPol = CSL_TSIP_CLKP_RISING;
    cfg->tx.clkPol = CSL_TSIP_CLKP_RISING;
    cfg->tx.dataRate = CSL_TSIP_DATARATE_16M;
    cfg->tx.clkMode = CSL_TSIP_CLKM_SGL;
    cfg->tx.superFrameInt = CSL_TSIP_INT_ACK;
    cfg->tx.frameInt = CSL_TSIP_INT_ACK;
    cfg->tx.frameIntDelay = 0;
    
    /* Receive configuration */
    cfg->rx.channel = deviceWhoAmI();
    cfg->rx.frameSize = CSL_TSIP_FRAMESIZE_128; 
    cfg->rx.tsPerFrame = 256;
    cfg->rx.clkSrc = CSL_TSIP_CLKSRC_A;
    cfg->rx.dataDelay = 1;
    cfg->rx.bdxDelay = CSL_TSIP_DLY_CTRL_DISABLE;
    cfg->rx.fsyncPol = CSL_TSIP_FSYNCP_ALOW;
    cfg->rx.fsyncClkPol = CSL_TSIP_CLKP_RISING;
    cfg->rx.clkPol = CSL_TSIP_CLKP_FALLING;
    cfg->rx.dataRate = CSL_TSIP_DATARATE_16M;
    cfg->rx.clkMode = CSL_TSIP_CLKM_SGL;
    cfg->rx.superFrameInt = CSL_TSIP_INT_ACK;
    cfg->rx.frameInt = CSL_TSIP_INT_ACK;
    cfg->rx.frameIntDelay = 0;
}


/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function creates.TSIP drvier instance
 *
 *  @retval
 *      return code   0:  init succeeded
 *                    -1: init failed
 */
Int initTsipShared (tsipSizeInfo_t *sizeCfg, tsipConfig_t *pTsipCfg)
{
  int sizes[tsip_N_BUFS_SHARED];
  int aligns[tsip_N_BUFS_SHARED];
  void* bases[tsip_N_BUFS_SHARED];
  tsipReturn_t ret;

  /* Get buffer requirements */
  ret = Tsip_getBufferReqShared (sizeCfg, sizes, aligns);

  if (ret != tsip_OK)  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared() return with error code %d\n", ret);
    return (-1);
  }

  /* Allocate space for the TSIP LLD shared buffers. */
  /*System_printf ("sizes [0]=%d [1]=%d [2]=%d [3]=%d [4]=%d  \n", sizes[0],sizes[1],sizes[2],sizes[3],sizes[4]);*/


  /* The first buffer is for the shared instance */
  if (sizeof(memTsipInstShared) < sizes[0])  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires size %d for instance shared buffer, have only %d\n", sizes[0], sizeof(memTsipInstShared));
    return (-1);
  }

  bases[0] = (Void *)memTsipInstShared;


  /* The second buffer is for shared heap */
  if (sizeof(memTsipHeapShared) <  sizes[1])  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d bytes for buffer 1, have only %d\n", sizes[1], sizeof(memTsipHeapShared));
    return (-1);
  }

  bases[1] = (Void *)memTsipHeapShared;


  /* The third buffer is for timeslot arrays */
  if ((Uint32)memTsipHeapTimeslot & (aligns[2] - 1))  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d alignment for buffer 2, but address is 0x%08x\n", aligns[2], (Uint32)memTsipHeapTimeslot);
    return (-1);
  }
  if (sizeof(memTsipHeapTimeslot) <  sizes[2])  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d bytes for buffer 2, have only %d\n", sizes[2], sizeof(memTsipHeapTimeslot));
    return (-1);
  }

  bases[2] = (Void *)memTsipHeapTimeslot;


  /* The fourth buffer is for transmit DMA */
  if ((Uint32)memTsipTxDmaBuf & (aligns[3] - 1))  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d alignment for buffer 3, but address is 0x%08x\n", aligns[3], (Uint32)memTsipTxDmaBuf);
    return (-1);
  }
  if (sizeof(memTsipTxDmaBuf) <  sizes[3])  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d bytes for buffer 3, have only %d\n", sizes[3], sizeof(memTsipTxDmaBuf));
    return (-1);
  }

  bases[3] = (Void *)memTsipTxDmaBuf;

  /* The fifth buffer is for receive DMA */
  if ((Uint32)memTsipRxDmaBuf & (aligns[4] - 1))  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d alignment for buffer 4, but address is 0x%08x\n", aligns[4], (Uint32)memTsipRxDmaBuf);
    return (-1);
  }
  if (sizeof(memTsipRxDmaBuf) <  sizes[4])  {
    System_printf ("initTsipShared: Tsip_getBufferReqShared requires %d bytes for buffer 4, have only %d\n", sizes[4], sizeof(memTsipRxDmaBuf));
    return (-1);
  }

  bases[4] = (Void *)memTsipRxDmaBuf;

  /* Create TSIP driver instance */
  ret = Tsip_createShared (pTsipCfg, bases, &tsipHandle);
  if (ret != tsip_OK)  {
    System_printf ("initTsipShared: Tsip_createShared returned with error code %d\n", ret);
    return (-1);
  }

  /* Init done. Return success. */
   return (0);
}

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function creates.TSIP port instance
 *
 *  @retval
 *      return code   0:  init succeeded
 *                    -1: init failed
 */

Int initTsipPort (tsipConfig_t *pTsipCfg)
{
  int sizes[tsip_N_BUFS_PORT];
  int aligns[tsip_N_BUFS_PORT];
  void* bases[tsip_N_BUFS_PORT];
  tsipReturn_t ret;
  int portIndex;
  int coreNum;
  
  coreNum=DNUM;

  /* Get buffer requirements */
  ret = Tsip_getBufferReqPort (NULL, sizes, aligns);
  if (ret != tsip_OK)  {
    System_printf ("initTsipPort: Tsip_getBufferReqPort() return with error code %d\n", ret);
    return (-1);
  }
#ifdef CSL_TSIP_0
  if(pTsipCfg->physPort == CSL_TSIP_0)
#else
  if(pTsipCfg->physPort == CSL_TSIP)
#endif
    portIndex = 0;
  else
    portIndex = 1;


  /* The first buffer is used as the port instance buffer */
  if ((Uint32) &memTsipInstPort[portIndex * BUFSIZE_TSIP_INST_SIZE_PORT] & (aligns[0] - 1))  {
    System_printf ("initTsipPort: Tsip_getBufferReqPort requires %d alignment for port instance buffer, but address is 0x%08x\n", aligns[0], (Uint32)memTsipInstPort);
    return (-1);
  }

  if ( (sizeof(memTsipInstPort)/NUM_USED_TSIP_PORTS) < sizes[0])  {
    System_printf ("initTsipPort: Tsip_getBufferReqPort requires size %d for port instance buffer, have only %d\n", sizes[0], sizeof(memTsipInstPort));
    return (-1);
  }

  
  bases[0] = (Void *) &memTsipInstPort[portIndex * BUFSIZE_TSIP_INST_SIZE_PORT];

  if(coreNum==0)
    pTsipCfg->masterCore=1;
  else
    pTsipCfg->masterCore=0;

  /* Create TSIP port instance */
  ret = Tsip_createPort (tsipHandle, pTsipCfg, bases, &portHandle[portIndex]);
  if (ret != tsip_OK)  {
    System_printf ("initTsipPort: Tsip_createPort returned with error code %d\n", ret);
    return (-1);
  }

  ret = Tsip_enablePortChannel (tsipHandle,
                                portHandle[portIndex],
                                coreNum,
                                coreNum);



  /* Init done. Return success. */
  return (0);

}


/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function configures TSIP Tx/Rx timeslot.
 *
 *  @retval
 *      None
 */
void timeslotConfig (tsipTsControl_t *ctl, UInt32 timeSlot, UInt32 tsipPort)
{
  UInt32 timeSlotLinkPort;

  /*combining time slot, link and TSIP port information in one word*/
  timeSlotLinkPort = timeSlot;
  timeSlotLinkPort = TSIP_HAL_SET_BITFIELD (timeSlotLinkPort, tsipPort, TSIP_PORT_MSB, TSIP_PORT_LSB);
  
  /* Companding and phase */
  ctl->compand = CSL_TSIP_TIMESLOT_LINEAR;
  ctl->phase = 0;

  /* Transmit direction */
  ctl->tx.enable = TRUE;
  ctl->tx.timeslot = timeSlotLinkPort;
  ctl->tx.frameSize = 40;
  ctl->tx.callout = appToTsip;
  if(tsipPort==0)
  {
    ctl->tx.context = (void *) &toTsip0[timeSlot];
    ctl->tx.buffer = (tsipData_t *) &appToTsipBuffer0[timeSlot * BUFSIZE_APP];
  }
  else
  {
    ctl->tx.context = (void *) &toTsip1[timeSlot];
    ctl->tx.buffer = (tsipData_t *) &appToTsipBuffer1[timeSlot * BUFSIZE_APP];
  }

  /* Receive direction */
  ctl->rx.enable = TRUE;
  ctl->rx.timeslot = timeSlotLinkPort;
  ctl->rx.frameSize = 40;
  ctl->rx.callout = tsipToApp;
  if(tsipPort==0)
  {
    ctl->rx.context = (void *) &fromTsip0[timeSlot];
    ctl->rx.buffer = (tsipData_t *) &tsipToAppBuffer0[timeSlot * BUFSIZE_APP];
  }
  else
  {
    ctl->rx.context = (void *) &fromTsip1[timeSlot];
    ctl->rx.buffer = (tsipData_t *) &tsipToAppBuffer1[timeSlot * BUFSIZE_APP];
  }
}


/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function power up.a TSIP port
 *
 *  @retval
 *      None
 */
void domainEnableTsip(UInt16 port)
{

  UInt32 done, pscID;

  /* Determine the event ID */
#ifdef CSL_TSIP_0
  if (port == CSL_TSIP_0)
    pscID = CSL_PSC_LPSC_TSIP0;
  else
    pscID = CSL_PSC_LPSC_TSIP1;
#else
#ifdef CSL_TSIP
  pscID = CSL_PSC_LPSC_TSIP;
#else
#error CSL Defines missing
#endif
#endif
  CSL_PSC_setModuleNextState(pscID, PSC_MODSTATE_ENABLE);
  CSL_PSC_startStateTransition(CSL_PSC_PD_ALWAYSON);
  do {
    done = CSL_PSC_isStateTransitionDone(CSL_PSC_PD_ALWAYSON);
  } while(!done);
}


/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n  
 *      This function register TSIP super frame interrupt
 *
 *  @retval
 *      None
 */
void registerTsipInterrupts(UInt16 port)
{
  static UInt32 cookie = 0;
  Int16  eventId;
  Int32  vectId;
  Int    portIndex;

  /* Disabling the global interrupts */
  cookie = Hwi_disable();
#ifdef CSL_TSIP_0
  /* Determine the event ID */
  if (port == CSL_TSIP_0)
  {
    eventId = CSL_GEM_TSIP0_RSFINT_N;
    portIndex=0;
    
    /* Pick a interrupt vector id to use */
    vectId = 7 + deviceWhoAmI();
  }
  else
  {
    eventId = CSL_GEM_TSIP1_RSFINT_N;
    portIndex=1;
    
    /* Pick a interrupt vector id to use */
    vectId = 8 + deviceWhoAmI();
  }
#else
#ifdef CSL_TSIP
  if (port == CSL_TSIP)
   {
     eventId = CSL_C66X_COREPAC_TSIP_RCV_SFINT0;
     portIndex=0;

     /* Pick a interrupt vector id to use */
     vectId = 7 + deviceWhoAmI();
   }
#else
#error CSL Definitions missing
#endif
#endif

  /* Register ISR handle for this event */
  EventCombiner_dispatchPlug (eventId, (EventCombiner_FuncPtr)Tsip_superFrameIsr, (UArg)portHandle[portIndex], TRUE);
  EventCombiner_enableEvent(eventId);

  /* Map the event id to hardware interrupt (# vectId). */
  Hwi_eventMap(vectId, eventId); 

  /* Enable interrupt (# vectId). */
  Hwi_enableInterrupt(vectId);

  /* Restore interrupts */
  Hwi_restore(cookie);
}

/*************************** FUNCTION ************************/
/**
 *  @b Description
 *  @n  
 *      Tsip sample application that sets up TSIP port and timeslot
 *
 *  @retval
 *      None
 */
Int tsipApp()
{
  Int ret;
  UInt32 i,j;
  
  /* TSIP configuration structures */
  tsipConfig_t tsipCfg;
  tsipSizeInfo_t tsipSizeCfg;

  System_printf ("**************************************************\n");
  System_printf ("********** TSIP Test Start ***********************\n");
  System_printf ("**************************************************\n");

  /* Initialize driver buffers */
  drvBufInit();

  /* Initialize application buffers to prepare for TSIP-APP transfer */
  if(printSetupProgress)System_printf ("Initialize application buffers.\n");
  appBufInit();

  /* Configure TSIP and create the TSIP driver instance */
  memset(&tsipCfg, 0, sizeof(tsipCfg));
  tsipConfig (&tsipSizeCfg, &tsipCfg);
  
  ret = initTsipShared(&tsipSizeCfg, &tsipCfg);
  if( ret != 0)
  {
    System_printf ("initTsipShared failed\n");
    return -1;
  } else
  {
    if(printSetupProgress)System_printf ("TSIP driver instance successfully initialized \n");
    if(printSetupProgress)System_printf ("-----------------------\n");
  }

  for(j=0; j<NUM_USED_TSIP_PORTS; j++)
  {
    /* Select the TSIP port and power it up */
    if(j==0)
#ifdef CSL_TSIP_0
      tsipCfg.physPort = CSL_TSIP_0;
#else
    tsipCfg.physPort = CSL_TSIP;
#endif
#ifdef CSL_TSIP_1
    else
      tsipCfg.physPort = CSL_TSIP_1;
#endif
    /* Core 0 powers up the TSIP ports */
    if(!deviceWhoAmI())
    {
      domainEnableTsip(tsipCfg.physPort);
      if(printSetupProgress) System_printf ("TSIP port %01x is powered up\n", tsipCfg.physPort);
    }

    /* Create the TSIP port instance */
    ret = initTsipPort(&tsipCfg);
    if( ret != 0)
    {
      System_printf ("initTsipPort failed\n");
      return -1;
    } else
    {
      if(printSetupProgress) System_printf ("TSIP port instance successfully initialized \n");
    }

    /* Register TSIP super frame interrupts */
    registerTsipInterrupts(tsipCfg.physPort);
    if(printSetupProgress) System_printf ("TSIP port %01x super frame interrupt is registered\n", tsipCfg.physPort);

    for(i=0; i<NUM_USED_TIME_SLOTS; i++)
    {
      /* Fill in timeslot configuration */
      memset(&tsCtl, 0, sizeof(tsCtl));
      timeslotConfig(&tsCtl, i, j);

      /* Advance the application buffers once as the the base has been passed to TSIP to use for the first frame */
      if(j==0)
      {
        appBufAdvancePointer((appBuf_t *) &fromTsip0[i], tsCtl.rx.frameSize, NULL);
        appBufAdvancePointer((appBuf_t *) &toTsip0[i],   tsCtl.tx.frameSize, NULL);
      }
      else
      {
        appBufAdvancePointer((appBuf_t *) &fromTsip1[i], tsCtl.rx.frameSize, NULL);
        appBufAdvancePointer((appBuf_t *) &toTsip1[i],   tsCtl.tx.frameSize, NULL);
      }
      
      if(printSetupProgress) System_printf ("Enabling TSIP timeslot TX 0x%01x RX 0x%01x... \n", tsCtl.tx.timeslot, tsCtl.rx.timeslot);

      /* Enable timeslot to process APP-TSIP-APP transfer*/ 
      ret = Tsip_timeslotConfig(tsipHandle, &tsCtl);
      if (ret != tsip_OK)
      {
        System_printf ("Tsip timeslot enable failed\n");
        return -1;
      } else
      {
        if(printSetupProgress) System_printf ("TSIP timeslot TX 0x%01x RX 0x%01x successfully enabled \n", tsCtl.tx.timeslot, tsCtl.rx.timeslot);
      }
    }
  
    if(printSetupProgress) System_printf ("TSIP Port %d configured\n",j);
    if(printSetupProgress) System_printf ("-----------------------\n");
  }
  
  if(printSetupProgress) System_printf ("TSIP data transfer...\n");
  return 0;
}

/*************************** FUNCTION ************************/
/**
 *  @b Description
 *  @n  
 *      Buffer Verification
 *
 *  @retval
 *      None
 */
Uint16 checkBufs(tsipData_t * appToTsipBuf, tsipData_t *tsipToAppBuf)
{
  Uint16 i, j, k, m, n;
  
  /*find first non-zero value*/
  for (j = 0; j<BUFSIZE_APP; j++)
  {
    if ((tsipToAppBuf[j] != 0) && (tsipToAppBuf[j] != 0xff))
	  break;
  }
  k = tsipToAppBuf[j];  /* first index in appToTsipBuffer whose value is equal to tsipToAppBuffer[j]*/

  if(j>=BUFSIZE_APP-1)
  {
    System_printf ("TSIP data transfer verfication failed. Array tsipToAppBuffer is empty. \n");
    return 1;
  }
  
  m=0;
  /* Compare data buffers */
  for (i = j; i<BUFSIZE_APP-tsCtl.rx.frameSize; i++) /* excluding the last frame since it is still used by ISR */
  {
     if (tsipToAppBuf[i] != appToTsipBuf[k++])
     {
       System_printf ("TSIP data transfer verfication failed. \n");
       System_printf ("Before failing, verified %d samples.\n", m);
       System_printf ("tsipToAppBuf[%d]=0x%x when it should have been 0x%x\n",i,tsipToAppBuf[i],appToTsipBuf[k-1]);
       if(debugCheckBuff)
       {
         for (n = 0; n<BUFSIZE_APP; n++)
         {
           System_printf ("tsipToAppBuf[%d]=%d != appToTsipBuf[%d]=%d\n",n,tsipToAppBuf[n],n,appToTsipBuf[n]);
         }
       }
       return 1;
     }
     m++;
  }

  System_printf ("verified %d samples.\n", m);
  return 0;
}

/*************************** FUNCTION ************************/
/**
 *  @b Description
 *  @n  
 *      Data Verification
 *
 *  @retval
 *      None
 */
void dataVerify()
{
  Uint16 events, timeSlot;
  Uint16 flag = TRUE;
  Uint16 testFail;

  while (flag) {
    /* Waiting for tranmission completed */
    events = Event_pend(dataVerifyEvent, Event_Id_00+Event_Id_01, 
                        Event_Id_NONE, BIOS_WAIT_FOREVER);
    if (events == Event_Id_00 + Event_Id_01) 
    {
      flag = FALSE;
      
      System_printf ("Verify TSIP Port 0 data transfer...\n");

      testFail=0;
      for(timeSlot=0; timeSlot < NUM_USED_TIME_SLOTS; timeSlot++)
      {
        System_printf ("-->Slot %d ", timeSlot);
        testFail |= checkBufs(&appToTsipBuffer0[timeSlot * BUFSIZE_APP], &tsipToAppBuffer0[timeSlot * BUFSIZE_APP]);
      }
      
      if(testFail)
      {
        System_printf ("TSIP Port 0 data transfer verfication FAILED \n");
        System_printf ("TSIP Test ended. Test Failed. \n");
        exit(1);
      }
      else
        System_printf ("TSIP Port 0 data transfer verfication PASSED \n");
      
      
      #if (NUM_USED_TSIP_PORTS > 1)  
      System_printf ("Verify TSIP Port 1 data transfer...\n");

      testFail=0;
      for(timeSlot=0; timeSlot < NUM_USED_TIME_SLOTS; timeSlot++)
      {
        System_printf ("-->Slot %d ", timeSlot);
        testFail |= checkBufs(&appToTsipBuffer1[timeSlot * BUFSIZE_APP], &tsipToAppBuffer1[timeSlot * BUFSIZE_APP]);
      }
      if(testFail)
      {
        System_printf ("TSIP Port 1 data transfer verfication FAILED \n");
        System_printf ("TSIP Test ended. Test Failed. \n");
        exit(1);
      }
      else
        System_printf ("TSIP Port 1 data transfer verfication PASSED \n");
      #endif
    }
  }
  
  System_printf ("TSIP Test ended. Test Passed. \n");
  exit(0);
}

/*************************** FUNCTION ************************/
/**
 *  @b Description
 *  @n  
 *      Entry point for sample application.
 *
 *  @retval
 *      None
 */
void main()
{
    Task_Params    tsipTaskParams;
    
    /* Initialize the task params */
    Task_Params_init(&tsipTaskParams);

    /* Create the task for TSIP sample application */
    Task_create((Task_FuncPtr)tsipApp, &tsipTaskParams, NULL);

    /* Create the task for data initialization and verification */
    Task_create((Task_FuncPtr)dataVerify, &tsipTaskParams, NULL);

    /* Create event which will be used to trigger data verify */
    dataVerifyEvent = Event_create(NULL, NULL);

    /* Start the BIOS Task scheduler */
    BIOS_start();
}

