/*
 * mcbspMasterDigLpbk.c
 *
 * This file contains the test / demo code to demonstrate the McBSP driver
 * master functionality using Digital Loopback setup. The file configures 
 * the EVM in master mode.
 *
 * Copyright (C) 2012 - 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */

#include <ti/sysbios/BIOS.h>
#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>
#include <string.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>

#include <xdc/cfg/global.h>
#include <board.h>
#include <MCBSP_log.h>

/* Include EDMA3 Driver */
#include <ti/sdo/edma3/drv/edma3_drv.h>

/* CSL Chip Functional Layer */
//#include <ti/csl/csl_chip.h>

/* CSL Cache Functional Layer */
//#include <ti/csl/csl_cacheAux.h>

/* CSL CPINTC Include Files. */
//#include<ti/csl/csl_cpIntc.h>

/* MCBSP Driver Include File. */
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/drv/mcbsp/mcbsp_osal.h>

/* PlatformLib Include File */
//#include <ti/platform/platform.h>


/* ========================================================================== */
/*                        EXTERNAL FUNCTIONS                                  */
/* ========================================================================== */

extern EDMA3_DRV_Handle edma3init(unsigned int edma3Id, EDMA3_DRV_Result *);

extern void McbspDevice_init(void);

extern int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);

extern Board_STATUS Board_init(Board_initCfg);

extern void McbspXmtInterrupt_init(void *mcbspTxChan);
extern void McbspRcvInterrupt_init(void *mcbspRxChan);

/* FPGA Configuration Misc-1 Register offset */
#define MCBSP_FPGA_MISC_REG_OFFSET (0x0C)

/* ========================================================================== */
/*                           MACRO DEFINTIONS                                 */
/* ========================================================================== */

#define NUM_BUFS                     2          /* Max of buffers used in each direction */
#define FRAMESIZE                   40          /* e.g 5 ms 8 KHz samples         */
#define NUM_OF_ENABLED_CHANNELS      8          /* Number of slots to be used     */
#define NUM_OF_MAX_CHANNELS	       128			/* Maximum number of time slots available based on clock settings   */
#define BUFSIZE                   (NUM_OF_ENABLED_CHANNELS * FRAMESIZE)        /* Total buffer size per frame */

/* Defines the core number responsible for system initialization. */
#define CORE_SYS_INIT         0
/* Number of iterations to execute test: Only applicable to INTERNAL CLOCK Loopback test */
#define NUM_OF_ITERATIONS 10

/* Number of MCBSP Frame structures used for submit channel */
#define NUM_OF_MCBSP_FRAMES 2

/* Number of buffers initially submitted to Mcbsp lld: Only applicable to
   External clock test ; Note: This should be set to 2*/
#define INIT_SUBMIT_Q_CNT 2

/*============================================================================*/
/*                            GLOBAL VARIABLES                                */
/*============================================================================*/

/* Shared Memory Variable to ensure synchronizing MCBSP initialization
 * with all the other cores. */
/* Created an array to pad the cache line with MCBSP_CACHE_LENGTH size */
#pragma DATA_ALIGN   (isMCBSPInitialized, MCBSP_MAX_CACHE_ALIGN)
#pragma DATA_SECTION (isMCBSPInitialized, ".mcbspSharedMem");
volatile Uint32 isMCBSPInitialized[(MCBSP_CACHE_LENGTH / sizeof(Uint32))] = { 0 };

/* Handle to the EDMA driver instance */
#pragma DATA_ALIGN   (hEdma, MCBSP_MAX_CACHE_ALIGN)
EDMA3_DRV_Handle hEdma[(MCBSP_CACHE_LENGTH / sizeof(EDMA3_DRV_Handle))] = { NULL };

/* Handle to MCBSP driver instance */
typedef void* Mcbsp_DevHandle;
Mcbsp_DevHandle  hMcbspDev;

/* Handle to MCBSP driver channel instance */
typedef void* Mcbsp_ChanHandle;
Mcbsp_ChanHandle  hMcbspTxChan;
Mcbsp_ChanHandle  hMcbspRxChan;

/* Core Number Identifier */
UInt32 coreNum = 0xFFFF;

/* Array to hold the pointer to the allocated buffers     */
void* bufRx[NUM_BUFS];
void* bufTx[NUM_BUFS];

#ifdef MCBSP_LOOP_PING_PONG
/* Ping pong buffers used to submit to Mcbsp lld, which will be used in a loop */
void* bufRxPingPong[INIT_SUBMIT_Q_CNT];
void* bufTxPingPong[INIT_SUBMIT_Q_CNT];
#endif
/* Global Error call back function prototype */
void mcbsp_GblErrCallback(uint32_t chanHandle,uint32_t spcr_read,uint32_t Arg3);

/* Variables to indicate status of EDMA TX and RX requests */
volatile uint32_t edmaTxDone = 0;
volatile uint32_t edmaRxDone = 0;
/* Global variables to track number of buffers submitted, number of iterations, error counts etc */
int rxSubmitCount=0, txSubmitCount=0;
uint32_t num_iterations=0;
uint32_t num_rx_Call_backs=0, num_tx_Call_backs=0, dummy_call_backs=0;
uint32_t rxunderflowcnt=0, txunderflowcnt=0;
uint32_t errBuffCount=0;
/* Debug variables */
volatile int debugVar=1;  /* This can be used to maintain a while loop, If set to 0 will exit loop */
volatile int debugCommand=1;  /* This can be used to set to value below to send command to restart channel  */

#define DEBUG_COMMAND_RESTART_CHANNELS 1
/**
 * \brief    Mcbsp Sample rate generator default parameters.
 *
 */
Mcbsp_srgConfig mcbspSrgCfg =
{
    FALSE,                     /* No gsync to be used as input is not CLKS    */
    Mcbsp_ClkSPol_RISING_EDGE, /* Dont care as input clock is not clks        */
    Mcbsp_SrgClk_CLKCPU,       /* McBSP internal clock to be used             */
    //166666667,                 /* Mcbsp internal clock frequency(PLL-SYSCLK6) */
    333333333, //sys_oscclk or cpu_clk/3
    0                          /* frame sync pulse width (val+1) is used      */
};

/**
 * \brief    Mcbsp device creation default parameters.
 *
 */
const Mcbsp_Params Mcbsp_PARAMS =
{
    Mcbsp_DevMode_McBSP,       /* Use the device as MCBSP                     */
    Mcbsp_OpMode_DMAINTERRUPT, /* Use DMA mode of operation                   */
    TRUE,                      /* cache coherency taken care of by driver     */
    Mcbsp_EmuMode_FREE,        /* Emulation mode free is to be enabled        */
#ifdef    MCBSP_EXTERNAL_CLOCK
    Mcbsp_Loopback_DISABLE,    /* Loop back mode disabled                     */
#else
    Mcbsp_Loopback_ENABLE,     /* Loop back mode enabled                      */
#endif
    &mcbspSrgCfg,              /* sample rate generator configuration         */
    NULL,                      /* TX pending buffer queue from application    */
    NULL,                      /* TX floating buffer queue in DMA             */
    NULL,                      /* RX pending buffer queue from application    */
    NULL                       /* RX floating buffer queue in DMA             */
};

#pragma DATA_ALIGN(loopTxJob, MCBSP_MAX_CACHE_ALIGN)
static Int32 loopTxJob[16] = {
    /* Filling with Mu law silence pattern : Can be any user defined pattern */
    0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f,
    0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f, 0x7f7f7f7f
};
#pragma DATA_ALIGN(loopRxJob, MCBSP_MAX_CACHE_ALIGN)
static Int32 loopRxJob[16] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

/**< settings to configure the TX or RX hardware sections                 */
Mcbsp_DataConfig mcbspChanConfigTx =
{
    Mcbsp_Phase_SINGLE,
    Mcbsp_WordLength_8,
    Mcbsp_WordLength_8,    /* Dont care for single phase*/
    NUM_OF_MAX_CHANNELS,
    NUM_OF_MAX_CHANNELS,      // Only used with dual phase
    Mcbsp_FrmSync_DETECT,
#ifdef MCBSP_EXTERNAL_CLOCK
    Mcbsp_DataDelay_0_BIT,
#else
    Mcbsp_DataDelay_1_BIT,
#endif
    Mcbsp_Compand_OFF_MSB_FIRST,
    Mcbsp_BitReversal_DISABLE,
    Mcbsp_IntMode_ON_SYNCERR,
    Mcbsp_RxJust_RZF,  /* Dont care for TX         */
    Mcbsp_DxEna_OFF
};

/**< settings to configure the TX or RX hardware sections                 */
Mcbsp_DataConfig mcbspChanConfigRx =
{
    Mcbsp_Phase_SINGLE,
    Mcbsp_WordLength_8,
    Mcbsp_WordLength_8,    /* Dont care for single phase*/
    NUM_OF_MAX_CHANNELS,
    NUM_OF_MAX_CHANNELS,      // Only used with dual phase
    Mcbsp_FrmSync_DETECT,
#ifdef MCBSP_EXTERNAL_CLOCK
    Mcbsp_DataDelay_0_BIT,
#else
    Mcbsp_DataDelay_1_BIT,
#endif
    Mcbsp_Compand_OFF_MSB_FIRST,
    Mcbsp_BitReversal_DISABLE,
    Mcbsp_IntMode_ON_SYNCERR,
    Mcbsp_RxJust_RZF,  /* Dont care for TX         */
    Mcbsp_DxEna_OFF
};
/**< clock setup for the TX section                     */
Mcbsp_ClkSetup mcbspClkConfigTx =
{
#ifdef MCBSP_EXTERNAL_CLOCK
    Mcbsp_FsClkMode_EXTERNAL,
    8000,                   /* 8KHz                   */
    Mcbsp_TxRxClkMode_EXTERNAL,
#else
    Mcbsp_FsClkMode_INTERNAL,
    8000,                   /* 8KHz                   */
    Mcbsp_TxRxClkMode_INTERNAL,
#endif
    Mcbsp_FsPol_ACTIVE_HIGH,
    Mcbsp_ClkPol_RISING_EDGE
};

/**< clock setup for the RX section                     */
Mcbsp_ClkSetup mcbspClkConfigRx =
{
#ifdef MCBSP_EXTERNAL_CLOCK
    Mcbsp_FsClkMode_EXTERNAL,
    8000,                   /* 8KHz                   */
    Mcbsp_TxRxClkMode_EXTERNAL,
#else
    Mcbsp_FsClkMode_INTERNAL,
    8000,                   /* 8KHz                   */
    Mcbsp_TxRxClkMode_INTERNAL,
#endif
    Mcbsp_FsPol_ACTIVE_HIGH,
    Mcbsp_ClkPol_FALLING_EDGE
};

/**< Multi channel setup                                                      */
Mcbsp_McrSetup mcbspMultiChanCtrl =
{
    Mcbsp_McmMode_ALL_CHAN_DISABLED_UNMASKED,
    Mcbsp_PartitionMode_CHAN_0_15,
    Mcbsp_PartitionMode_CHAN_16_31,
    Mcbsp_PartitionMode_2
};


Mcbsp_ChanParams mcbspChanparamTx =
{
    Mcbsp_WordLength_8,  /* wordlength configured    */
    &loopTxJob[0],          /* loop job buffer internal */
    8,                    /* user loopjob length      */
    mcbsp_GblErrCallback, /* global error callback    */
    NULL,                 /* edma Handle              */
    1,                    /* EDMA event queue         */
    8,                    /* hwi number               */
    Mcbsp_BufferFormat_MULTISLOT_NON_INTERLEAVED,
    TRUE,                 /* FIFO mode enabled        */
    &mcbspChanConfigTx,   /* channel configuration    */
    &mcbspClkConfigTx,    /* clock configuration      */
    &mcbspMultiChanCtrl,  /* multi channel control    */
    0x7531,  /* Enabled timeslots: 0, 4, 5, 8, 10, 12, 13, 14 */
    0x00,
    0x00,
    0x00,
    NUM_OF_ENABLED_CHANNELS    /* Total number of channels enabled*/
};

Mcbsp_ChanParams mcbspChanparamRx =
{
    Mcbsp_WordLength_8,  /* wordlength configured    */
    &loopRxJob[0],          /* loop job buffer internal */
    8,                    /* user loopjob length      */
    mcbsp_GblErrCallback, /* global error callback    */
    NULL,                 /* edma Handle              */
    1,                    /* EDMA event queue         */
    8,                    /* hwi number               */
    Mcbsp_BufferFormat_MULTISLOT_NON_INTERLEAVED,
    TRUE,                 /* FIFO mode enabled        */
    &mcbspChanConfigRx,   /* channel configuration    */
    &mcbspClkConfigRx,    /* clock configuration      */
    &mcbspMultiChanCtrl,  /* multi channel control    */
    0x7531,  /* Enabled timeslots: 0, 4, 5, 8, 10, 12, 13, 14 */
    0x00,
    0x00,
    0x00,
    NUM_OF_ENABLED_CHANNELS    /* Total number of channels enabled*/
};


/* ========================================================================== */
/*                           FUNCTION DEFINITIONS                             */
/* ========================================================================== */

/*
 *   This is the application's callback function. The driver will
 *   call this function whenever an EDMA I/O operation is over.
 *
 */
void mcbspAppCallback(void* arg, Mcbsp_IOBuf *ioBuf)
{
    int32_t mode;
    int32_t *pmode = (int32_t *)arg;

    mode = *pmode;
    if (mode == MCBSP_MODE_OUTPUT)
    {
    	if(ioBuf) {
            num_tx_Call_backs++;
            edmaTxDone = 1;
        }else
            txunderflowcnt++;
    }
    else if (mode == MCBSP_MODE_INPUT)
    {
        if(ioBuf) {
            num_rx_Call_backs++;
            edmaRxDone = 1;
        }else
            rxunderflowcnt++;
    }else
        dummy_call_backs++;
    return;
}

/*
 *   This is the application's Global error callback function 
 */
void mcbsp_GblErrCallback(uint32_t chanHandle,uint32_t spcr_read,uint32_t Arg3)
{
    MCBSP_log ("Debug(Core %d): ERROR callback called SPCR: %x", coreNum, spcr_read);
}

/*
 * \brief   This function demostrates the use of Mcbsp using digital loopback
 *          communication setup.
 *
 * \param   None
 *
 * \return  None
 */
void mcbspDigLpbkApp(UArg arg0, UArg arg1)
{
    /**< Mcbsp device params                                                  */
    Mcbsp_Params  mcbspParams;

    /**< Queue to hold the pending packets received from the application      */
    Queue_Struct  txQueuePendingList, rxQueuePendingList;
    /**< Queue to manage floating packets in DMA                              */
    Queue_Struct  txQueueFloatingList, rxQueueFloatingList;

    uint32_t count   = 0, tempCount = 0;
    int32_t  status  = 0, retval = 0;
    int32_t  txChanMode = MCBSP_MODE_OUTPUT;
    int32_t  rxChanMode = MCBSP_MODE_INPUT;
    uint32_t mcbspTxDone = 0, mcbspRxDone = 0;
    Mcbsp_IOBuf txFrame[NUM_OF_MCBSP_FRAMES], rxFrame[NUM_OF_MCBSP_FRAMES];
    int txFrameIndex=0, rxFrameIndex=0;
    int init_count=0;
#ifdef MCBSP_LOOP_PING_PONG
    int pingPongIndex=0;
#endif

    /* Initialize the OSAL */
    if (Osal_dataBufferInitMemory(BUFSIZE) < 0)
    {
        MCBSP_log ("Debug(Core %d): Error: Unable to initialize the OSAL. \n", coreNum);
        return;
    }

    /* update EDMA3 handle to channel parameters */
    mcbspChanparamTx.edmaHandle = hEdma[0];
    mcbspChanparamRx.edmaHandle = hEdma[0];

    /* create the pending and floating queue for the TX channel           */
    Queue_construct(&txQueuePendingList, NULL);
    Queue_construct(&txQueueFloatingList, NULL);

    /* create the pending and floating queue for the RX channel           */
    Queue_construct(&rxQueuePendingList, NULL);
    Queue_construct(&rxQueueFloatingList, NULL);


    mcbspParams                 = Mcbsp_PARAMS;
    mcbspParams.txQPendingList  = &txQueuePendingList;
    mcbspParams.txQFloatingList = &txQueueFloatingList;
    mcbspParams.rxQPendingList  = &rxQueuePendingList;
    mcbspParams.rxQFloatingList = &rxQueueFloatingList;


    /* Bind the driver instance with device instance */
    status = mcbspBindDev(&hMcbspDev, coreNum, &mcbspParams);

    if (status != MCBSP_STATUS_COMPLETED)
    {
        MCBSP_log ("Debug(Core %d): MCBSP LLD Bind Device Failed\n", coreNum);
        return;
    }

    /* If the user loopjob buffer is in local L2 memory: Convert into Global memory address space */
    if(mcbspChanparamRx.userLoopJobBuffer)
        mcbspChanparamRx.userLoopJobBuffer = (void *)Mcbsp_osalLocal2Global(mcbspChanparamRx.userLoopJobBuffer);

    /* Create a RX channel for receiving */
    status = mcbspCreateChan(&hMcbspRxChan, hMcbspDev, MCBSP_MODE_INPUT, &mcbspChanparamRx, mcbspAppCallback, &rxChanMode);
    if (MCBSP_STATUS_COMPLETED != status)
    {
        MCBSP_log ("Debug(Core %d): Error: Create Channel (RX) failed\n", coreNum);
        return;
    }

    /* If the user loopjob buffer is in local L2 memory: Convert into Global memory address space */
    if(mcbspChanparamTx.userLoopJobBuffer)
        mcbspChanparamTx.userLoopJobBuffer = Mcbsp_osalLocal2Global(mcbspChanparamTx.userLoopJobBuffer);

    /* Create a TX channel for the transmission */
    status = mcbspCreateChan(&hMcbspTxChan, hMcbspDev, MCBSP_MODE_OUTPUT, &mcbspChanparamTx, mcbspAppCallback, &txChanMode);
    if (MCBSP_STATUS_COMPLETED != status)
    {
        MCBSP_log ("Debug(Core %d): Error: Create Channel (TX) failed\n", coreNum);
        return;
    }

    /* Register mcbsp interrupts */
    //McbspRcvInterrupt_init(hMcbspRxChan);
    //McbspXmtInterrupt_init(hMcbspTxChan);

    /* create the buffers required for the TX and RX operations */
    for (count = 0; count < (NUM_BUFS); count++)
    {
        bufTx[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(BUFSIZE);
        bufRx[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(BUFSIZE);

        if (bufTx[count] == NULL)
        {
            MCBSP_log ("Debug(Core %d): Error: Tx Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
        if (bufRx[count] == NULL)
        {
            MCBSP_log ("Debug(Core %d): Error: Rx Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
    }
#ifdef MCBSP_LOOP_PING_PONG
    /* create the ping pong buffers required for the TX and RX operations */
    for (count = 0; count < (NUM_BUFS); count++)
    {
        bufRxPingPong[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(BUFSIZE);
        bufTxPingPong[count] = (uint8_t *)Osal_mcbspDataBufferMalloc(BUFSIZE);

        if (bufTxPingPong[count] == NULL)
        {
            MCBSP_log ("Debug(Core %d): Error: Tx Ping pong Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
        if (bufRxPingPong[count] == NULL)
        {
            MCBSP_log ("Debug(Core %d): Error: Rx Ping pong Buffer (%d) Memory Allocation Failed\n", coreNum, count);
            return;
        }
    }
#endif
    /* Fill the buffers with known data and transmit the same and 
       check if the same pattern is received */
    for (count = 0; count < (NUM_BUFS); count++)
    {
        memset((uint8_t *)bufTx[count], 0, BUFSIZE);
        for (tempCount = 0; tempCount < BUFSIZE; tempCount++)
        {
            ((uint8_t *)bufTx[count])[tempCount] = (tempCount % 0x100);
        }
    }
    MCBSP_log ("Debug(Core %d): If required to restart set debugCommand variable to %d\n", coreNum, DEBUG_COMMAND_RESTART_CHANNELS);

restart_mcbsp_point:
    txFrameIndex=0;
    rxFrameIndex=0;
    init_count=0;
#ifdef MCBSP_EXTERNAL_CLOCK
    /* Initial loading of ping ping buffers */
    for(init_count =0 ; init_count < INIT_SUBMIT_Q_CNT; init_count++)
    {
#ifdef MCBSP_LOOP_PING_PONG
        memset((uint8_t *)bufRxPingPong[init_count], 0, BUFSIZE);

        /* RX frame processing */
        rxFrame[rxFrameIndex].cmd = Mcbsp_IOBuf_Cmd_READ;
        rxFrame[rxFrameIndex].addr = (void*)bufRxPingPong[init_count];
#else
        memset((uint8_t *)bufRx[init_count], 0, BUFSIZE);


        /* RX frame processing */
        rxFrame[rxFrameIndex].cmd = Mcbsp_IOBuf_Cmd_READ;
        rxFrame[rxFrameIndex].addr = (void*)bufRx[init_count];
#endif

        rxFrame[rxFrameIndex].size = BUFSIZE;
        rxFrame[rxFrameIndex].arg = (uint32_t) hMcbspRxChan;
        rxFrame[rxFrameIndex].status = MCBSP_STATUS_COMPLETED;
        rxFrame[rxFrameIndex].misc = 1;   /* reserved - used in callback to indicate asynch packet */

        status = mcbspSubmitChan(hMcbspRxChan, (void *)&rxFrame[rxFrameIndex]);
        if (status != MCBSP_STATUS_PENDING)
        {
            MCBSP_log ("Debug(Core %d): Error: RX buffer #%d submission FAILED\n", coreNum, init_count);
            retval = 1;
        }
        else
        {
#ifdef MCBSP_APP_VERBOSE
            MCBSP_log ("Debug(Core %d): RX buffer #%d is submitted to MCBSP driver\n", coreNum, init_count);
#endif
        }
        rxFrameIndex++;
        rxFrameIndex = (rxFrameIndex >= (NUM_OF_MCBSP_FRAMES)) ? 0 : rxFrameIndex;

        rxSubmitCount++;

        /* TX frame processing */
        txFrame[txFrameIndex].cmd = Mcbsp_IOBuf_Cmd_WRITE;
#ifdef MCBSP_LOOP_PING_PONG
        txFrame[txFrameIndex].addr = (void*)bufTxPingPong[init_count];
#else
        txFrame[txFrameIndex].addr = (void*)bufTx[init_count];
#endif
        txFrame[txFrameIndex].size = BUFSIZE;
        txFrame[txFrameIndex].arg = (uint32_t)hMcbspTxChan;
        txFrame[txFrameIndex].status = MCBSP_STATUS_COMPLETED;
        txFrame[txFrameIndex].misc = 1;   /* reserved - used in callback to indicate asynch packet */

        status = mcbspSubmitChan(hMcbspTxChan, (void *)&txFrame[txFrameIndex]);
        if (status != MCBSP_STATUS_PENDING)
        {
            MCBSP_log ("Debug(Core %d): Error: TX buffer  #%d submission FAILED\n", coreNum, init_count);
            retval = 1;
        }
        else
        {
#ifdef MCBSP_APP_VERBOSE
            MCBSP_log ("Debug(Core %d): TX buffer  #%d submission is submitted to MCBSP driver\n", coreNum, init_count);
#endif
        }
        txFrameIndex++;
        txFrameIndex = (txFrameIndex >=(NUM_OF_MCBSP_FRAMES)) ? 0 : txFrameIndex;

        txSubmitCount++;
    }

    /* Wait for TX and RX processing to complete */
    while (1)
    {
        if (edmaTxDone == 1)
        {
#ifdef MCBSP_APP_VERBOSE
            MCBSP_log ("Debug(Core %d): EDMA -> Iteration-%d frames are transmitted to TX path\n", coreNum, count);
#endif
            edmaTxDone = 0; /* Reset for next iteration */
            mcbspTxDone = 1;
        }
        if (edmaRxDone == 1)
        {
#ifdef MCBSP_APP_VERBOSE
            MCBSP_log ("Debug(Core %d): EDMA -> Iteration-%d frames are received from RX path\n", coreNum, count);
#endif
            edmaRxDone = 0;  /* Reset for next iteration */
            mcbspRxDone = 1;
        }
        if ((mcbspTxDone == 1) && (mcbspRxDone == 1))
        {
            mcbspTxDone = 0;  /* Reset for next iteration */
            mcbspRxDone = 0;  /* Reset for next iteration */
            break;
        }
    }
#ifdef MCBSP_LOOP_PING_PONG
    /* Change ping Pong Index */
    init_count=0;
#endif
#endif /* MCBSP_EXTERNAL_CLOCK */

    /* Start main loop to iterate through frames */
    while(debugVar)
    {
        /* submit frames to the driver */
        for (count = init_count; count < NUM_BUFS; count++)
        {
#ifdef MCBSP_EXTERNAL_CLOCK
/*   With External clock the data coming from the RX side is copied to loop back to the Tx side */
            memcpy((uint8_t *)bufTx[count], (uint8_t *)bufRx[count], BUFSIZE);
#endif
#ifdef MCBSP_LOOP_PING_PONG
            memset((uint8_t *)bufRxPingPong[pingPongIndex], 0, BUFSIZE);
#else
            memset((uint8_t *)bufRx[count], 0, BUFSIZE);
            /* RX frame processing */
            rxFrame[rxFrameIndex].cmd = Mcbsp_IOBuf_Cmd_READ;
            rxFrame[rxFrameIndex].addr = (void*)getGlobalAddr(bufRx[count]);
            rxFrame[rxFrameIndex].size = BUFSIZE;
            rxFrame[rxFrameIndex].arg = (uint32_t) hMcbspRxChan;
            rxFrame[rxFrameIndex].status = MCBSP_STATUS_COMPLETED;
            rxFrame[rxFrameIndex].misc = 1;   /* reserved - used in callback to indicate asynch packet */

            status = mcbspSubmitChan(hMcbspRxChan, (void *)&rxFrame[rxFrameIndex]);
            if (status != MCBSP_STATUS_PENDING)
            {
                MCBSP_log ("Debug(Core %d): Error: RX buffer #%d submission FAILED\n", coreNum, count);
                retval = 1;
            }
            else
            {
#ifdef MCBSP_APP_VERBOSE
                MCBSP_log ("Debug(Core %d): RX buffer #%d is submitted to MCBSP driver\n", coreNum, count);
#endif
            }
            rxFrameIndex++;
            rxFrameIndex = (rxFrameIndex >= (NUM_OF_MCBSP_FRAMES)) ? 0 : rxFrameIndex;
#endif
            rxSubmitCount++;
#ifdef MCBSP_LOOP_PING_PONG
            /* Copy buffer from buffer to ping pong buffer*/
            memcpy((uint8_t *)bufTxPingPong[pingPongIndex], (uint8_t *)bufTx[count],BUFSIZE);
#else
            /* TX frame processing */
            txFrame[txFrameIndex].cmd = Mcbsp_IOBuf_Cmd_WRITE;
            txFrame[txFrameIndex].addr = (void*)getGlobalAddr(bufTx[count]);
            txFrame[txFrameIndex].size = BUFSIZE;
            txFrame[txFrameIndex].arg = (uint32_t)hMcbspTxChan;
            txFrame[txFrameIndex].status = MCBSP_STATUS_COMPLETED;
            txFrame[txFrameIndex].misc = 1;   /* reserved - used in callback to indicate asynch packet */

            status = mcbspSubmitChan(hMcbspTxChan, (void *)&txFrame[txFrameIndex]);
            if (status != MCBSP_STATUS_PENDING)
            {
                MCBSP_log ("Debug(Core %d): Error: TX buffer  #%d submission FAILED\n", coreNum, count);
                retval = 1;
            }
            else
            {
#ifdef MCBSP_APP_VERBOSE
                MCBSP_log ("Debug(Core %d): TX buffer  #%d is submitted to MCBSP driver\n", coreNum, count);
#endif
            }
            txFrameIndex++;
            txFrameIndex = (txFrameIndex >= (NUM_OF_MCBSP_FRAMES)) ? 0 : txFrameIndex;
#endif
            txSubmitCount++;

            /* Wait for TX and RX processing to complete */
            while (1)
            {
                if (edmaTxDone == 1)
                {
#ifdef MCBSP_APP_VERBOSE
                    MCBSP_log ("Debug(Core %d): EDMA -> Buffer #-%d is transmitted to TX path\n", coreNum, count);
#endif
                    edmaTxDone = 0; /* Reset for next iteration */
                    mcbspTxDone = 1;
                }
                if (edmaRxDone == 1)
                {
#ifdef MCBSP_APP_VERBOSE
                    MCBSP_log ("Debug(Core %d): EDMA -> Buffer #-%d is received from RX path\n", coreNum, count);
#endif
                    edmaRxDone = 0;  /* Reset for next iteration */
                    mcbspRxDone = 1;
                }
                if ((mcbspTxDone == 1) && (mcbspRxDone == 1))
                {
                    mcbspTxDone = 0;  /* Reset for next iteration */
				    mcbspRxDone = 0;  /* Reset for next iteration */
				    break;
                }
            }
#ifdef MCBSP_LOOP_PING_PONG
            /* Change ping Pong Index */
            pingPongIndex=(pingPongIndex)?0:1;

            /* Copy buffer from buffer to other buffer*/
            memcpy((void*)bufRx[count], (uint8_t *)bufRxPingPong[pingPongIndex],BUFSIZE);
#endif
            /* compare buffer contents */
            for (tempCount = 0; tempCount < BUFSIZE; tempCount++)
            {
                if (((char *)bufTx[count])[tempCount] != ((char *)bufRx[count])[tempCount])
                {
#ifdef MCBSP_APP_VERBOSE
                    MCBSP_log("Debug(Core %d): Error: TX and RX frame data DOES NOT match in Buffer #-%d\n",
                        coreNum, count);
                    MCBSP_log("Debug(Core %d): Error: Buffer index = %d, Tx data = %d, Rx data = %d\n", coreNum,
                        tempCount, ((char *)bufTx[count])[tempCount], ((char *)bufRx[count])[tempCount]);
#endif
                    errBuffCount++;
                    break;
                }
            }
            if (tempCount >= BUFSIZE)
            {
#ifdef MCBSP_APP_VERBOSE
                MCBSP_log("Debug(Core %d): TX and RX frames data match in iteration-%d !!!\n", coreNum, count);
#endif
            }
        }
        init_count=0;
        num_iterations++;
#ifndef   MCBSP_EXTERNAL_CLOCK
        if(num_iterations >= NUM_OF_ITERATIONS)
            break;
#endif
        switch(debugCommand)
        {
        case DEBUG_COMMAND_RESTART_CHANNELS:
            debugCommand=0;
            /* Close channel and reopen */
            /* Delete  RX channel */
            status = mcbspDeleteChan(hMcbspRxChan);
            if (MCBSP_STATUS_COMPLETED != status)
            {
                MCBSP_log ("Debug(Core %d): Error: Delete Channel (RX) failed\n", coreNum);
                return;
            }

            /* Delete TX channel*/
            status = mcbspDeleteChan(hMcbspTxChan);
            if (MCBSP_STATUS_COMPLETED != status)
            {
                MCBSP_log ("Debug(Core %d): Error: Delete Channel (TX) failed\n", coreNum);
                return;
            }
            /* Create a RX channel for receiving */
            status = mcbspCreateChan(&hMcbspRxChan, hMcbspDev, MCBSP_MODE_INPUT, &mcbspChanparamRx, mcbspAppCallback, &rxChanMode);
            if (MCBSP_STATUS_COMPLETED != status)
            {
                MCBSP_log ("Debug(Core %d): Error: Create Channel (RX) failed\n", coreNum);
                return;
            }

            /* Create a TX channel for the transmission */
            status = mcbspCreateChan(&hMcbspTxChan, hMcbspDev, MCBSP_MODE_OUTPUT, &mcbspChanparamTx, mcbspAppCallback, &txChanMode);
            if (MCBSP_STATUS_COMPLETED != status)
            {
                MCBSP_log ("Debug(Core %d): Error: Create Channel (TX) failed\n", coreNum);
                return;
            }
            edmaTxDone = 0;   /* Reset for next iteration */
            edmaRxDone = 0;   /* Reset for next iteration */
            mcbspTxDone = 0;  /* Reset for next iteration */
            mcbspRxDone = 0;  /* Reset for next iteration */
            goto restart_mcbsp_point;

        default:
            debugCommand=0;
            break;

        }

    }

    if ((errBuffCount == 0 ) & (retval ==0))
    {
        MCBSP_log("Debug(Core %d): MCBSP Digital Loopback Application completed successfully : "\
            "Num iterations %d  Num buffers per iteration %d!!!\n",
            coreNum, num_iterations, NUM_BUFS);
        MCBSP_log("\n All tests have passed \n");
    }
    else
    {
        MCBSP_log("Debug(Core %d): MCBSP Digital Loopback application FAILED :  "\
            "Num iterations %d  Num buffers per iteration %d Failed buffers %d!!!\n",
            coreNum, num_iterations, NUM_BUFS, errBuffCount);
    }

    /* Delete  RX channel */
    status = mcbspDeleteChan(hMcbspRxChan);
    if (MCBSP_STATUS_COMPLETED != status)
    {
        MCBSP_log ("Debug(Core %d): Error: Delete Channel (RX) failed\n", coreNum);
        return;
    }

    /* Delete TX channel*/
    status = mcbspDeleteChan(hMcbspTxChan);
    if (MCBSP_STATUS_COMPLETED != status)
    {
        MCBSP_log ("Debug(Core %d): Error: Delete Channel (TX) failed\n", coreNum);
        return;
    }
    return;
}

/*
 * \brief  Void main(Void)
 *
 *         Main function of the sample application. This function calls the
 *         function to configure the mcbsp instance.
 *
 * \param  None
 *
 * \return None
 */
#include "ti/sysbios/hal/Cache.h"

Void main(Void)
{
    Task_Params taskParams;
    EDMA3_DRV_Result edmaResult = 0;
#ifdef SIMULATOR_SUPPORT
    uint8_t uchValue, uchReadValue;
#endif

    Board_initCfg arg = BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_UART_STDIO;
    Board_init(arg);

    /* Get the core number. */
    coreNum = 0; //CSL_chipReadReg (CSL_CHIP_DNUM);

#ifdef SIMULATOR_SUPPORT
#warn MCBSP Digital Loopback example is not supported on SIMULATOR !!!
    MCBSP_log ("MCBSP Digital Loopback example is not supported on SIMULATOR. Exiting!\n");
    return;
#else
    MCBSP_log ("Debug(Core %d): Running MCBSP Digital Loopback example on the DEVICE\n", coreNum);
#endif

    /* Initialize the system only if the core was configured to do so. */
    if (coreNum == CORE_SYS_INIT)
    {
#if 0
        MCBSP_log ("Debug(Core %d): System Initialization for MCBSP\n", coreNum);
        /* Read FPGA register */
        if (0 != (platform_fpgaReadConfigReg(MCBSP_FPGA_MISC_REG_OFFSET, &uchReadValue)))
        {
            MCBSP_log ("Debug(Core %d): FPGA McBSP_AMC_EN# register READ failed \n", coreNum);
            return;
        }
        /* Clear field for configuration */
        uchValue = (uchReadValue & (~0x3));
#ifndef PLATFORM_FPGA_MCBSP_AMC_EN
        uchValue |=  0x3;
#endif
        /* Drive McBSP_AMC_EN# high. Output SLCLKs, TxCLKs, RxCLKs, FSTs, FSRs as Hi-Z. 
         * These clocks and syncs are tri-stated and McBSP is accessed over 80-pin header */
        if (0 != (platform_fpgaWriteConfigReg(MCBSP_FPGA_MISC_REG_OFFSET, (uchValue))))
        {
            MCBSP_log ("Debug(Core %d): FPGA McBSP_AMC_EN# register WRITE failed \n", coreNum);
            return;
        }

        /* DEBUG: Verify if FPGA register is configured correctly */
        if (0 != (platform_fpgaReadConfigReg(MCBSP_FPGA_MISC_REG_OFFSET, &uchReadValue)))
        {
            MCBSP_log ("Debug(Core %d): FPGA McBSP_AMC_EN# register READ failed \n", coreNum);
            return;
        }

        if (uchValue != uchReadValue)
        {
            MCBSP_log ("Debug(Core %d): FPGA McBSP_AMC_EN# register setting failed \n", coreNum);
            return;
        }
        else
        {
            MCBSP_log ("Debug(Core %d): FPGA McBSP_AMC_EN# register is set to %d \n", coreNum, uchValue);
        }
#endif
        /* MCBSP Driver Initialization: This should always be called before
         * invoking the MCBSP Driver. */
        mcbspInit();

        /* Device Specific MCBSP Initializations */
        McbspDevice_init();
        
        /* MCBSP Driver is operational at this time. */
        MCBSP_log ("Debug(Core %d): MCBSP Driver Initialization Done\n", coreNum);

        /* Write to the SHARED memory location at this point in time. The other cores cannot execute
         * till the MCBSP Driver is up and running. */
        isMCBSPInitialized[0] = 1;

        /* The MCBSP IP block has been initialized. We need to writeback the cache here because it will
         * ensure that the rest of the cores which are waiting for MCBSP to be initialized would now be
         * woken up. */
        //CACHE_wbL1d ((void *) &isMCBSPInitialized[0], MCBSP_MAX_CACHE_ALIGN, CACHE_FENCE_WAIT);
        Cache_wb ((void *) &isMCBSPInitialized[0], MCBSP_CACHE_LENGTH,0x7fff, 1);
    }
    else
    {
        /* All other cores need to wait for the MCBSP to be initialized before they proceed with the test. */ 
        MCBSP_log ("Debug(Core %d): Waiting for MCBSP to be initialized.\n", coreNum);

        /* All other cores loop around forever till the MCBSP is up and running. 
         * We need to invalidate the cache so that we always read this from the memory. */
        while (isMCBSPInitialized[0] == 0)
            //CACHE_invL1d ((void *) &isMCBSPInitialized[0], MCBSP_MAX_CACHE_ALIGN, CACHE_FENCE_WAIT);
        	Cache_inv ((void *) &isMCBSPInitialized[0], MCBSP_CACHE_LENGTH, 0x7fff, 1);


        MCBSP_log ("Debug(Core %d): MCBSP can now be used.\n", coreNum);
    }

    /* Initialize EDMA3 library */
    hEdma[0] = edma3init(0, &edmaResult);

    if (edmaResult != EDMA3_DRV_SOK)
    {
        /* Report EDMA Error */
        MCBSP_log("Debug(Core %d): EDMA Driver Initialization FAILED\n", coreNum);
    }
    else
    {
        MCBSP_log("Debug(Core %d): EDMA Driver Initialization Done\n", coreNum);
    }

    /* Create the Digital Loopback Application Task */
    Task_Params_init(&taskParams);
    Task_create(mcbspDigLpbkApp, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();

    return;
}

/* ========================================================================== */
/*                                END OF FILE                                 */
/* ========================================================================== */
