/**
 *   @file  cppi_test.c
 *
 *   @brief
 *      This is the CPPI Low Level Driver resources test file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
 *  \par
*/

#ifndef __LINUX_USER_SPACE
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <xdc/cfg/global.h>
#ifdef __ARM_ARCH_7A__
#include <ti/sysbios/family/arm/a15/Mmu.h>
#endif
#else
#include "fw_test.h"
#endif

#include <string.h>

#ifdef NSS_LITE
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#endif

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* RM include */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

#ifndef __LINUX_USER_SPACE
/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#ifdef __ARM_ARCH_7A__
#include <ti/csl/cslr_msmc.h>
#endif

/* OSAL includes */
#include <cppi_osal.h>
#include <qmss_osal.h>
#endif

/* Test specific includes */
#include <cppi_test.h>

#define RM 1

/* Global variables */
/* Host and monolithic descriptor queue handles */
Qmss_QueueHnd       freeHostQueHnd, freeMonoQueHnd;

/* Error counter */
uint32_t            errorCount = 0, coreNum;

#ifndef __LINUX_USER_SPACE
/************************ GLOBAL VARIABLES ********************/
#ifdef _TMS320C6X
/* linking RAM */
#pragma DATA_ALIGN (linkingRAM0, 16)
uint64_t            linkingRAM0[NUM_HOST_DESC + NUM_MONOLITHIC_DESC];

/* Descriptor pool [Size of descriptor * Number of descriptors] */
#pragma DATA_ALIGN (hostDesc, 16)
uint8_t             hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC];
#pragma DATA_ALIGN (monolithicDesc, 16)
uint8_t             monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC];
#pragma DATA_ALIGN (dataBuff, 16)
uint8_t             dataBuff[SIZE_DATA_BUFFER * NUM_DATA_BUFFER];
#else
/* linking RAM */
uint64_t linkingRAM0[NUM_HOST_DESC + NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));

/* Descriptor pool [Size of descriptor * Number of descriptors] */
uint8_t hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC] __attribute__ ((aligned (16)));
uint8_t monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));
uint8_t dataBuff[SIZE_DATA_BUFFER * NUM_DATA_BUFFER] __attribute__ ((aligned (16)));
#endif
#ifdef _TMS320C6X
/* Handle to CPPI heap */
IHeap_Handle        cppiHeap;
#endif
/* Rx channel handles */
Cppi_ChHnd          rxChHnd[NUM_RX_CHANNELS];
/* Tx channel handles */
Cppi_ChHnd          txChHnd[NUM_TX_CHANNELS];
/* Rx flow handles */
Cppi_FlowHnd        rxFlowHnd[NUM_RX_FLOWS];

#ifdef CONFIG_ACC
/* List address for accumulator - twice the number of entries for Ping and Pong page */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (hiPrioList, 16)
Uint32                          hiPrioList[(NUM_PACKETS + 1) * 2];
#else
Uint32 hiPrioList[(NUM_PACKETS + 1) * 2] __attribute__ ((aligned (16)));
#endif
#endif

/************************ EXTERN VARIABLES ********************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;
#if RM
/* RM test Global Resource List (GRL) */
extern const char rmGlobalResourceList[];
/* RM test Global Policy provided to RM Server */
extern const char rmDspOnlyPolicy[];
#endif

/************************** EXTERNS ***************************/
extern Void testHostDataTransfer (Void);
extern Void testMonoDataTransfer (Void);

#ifdef _TMS320C6X
/************************** FUNCTIONS *************************/
/**
 *  @b Description
 *  @n
 *      The function is used to get the handle to the CPPI memory heap.
 *      If the application is run on a multiple cores then place the CPPI global
 *      variables in shared memory.
 *
 *  @retval
 *      None
 */
static Void cppiHeapInit ()
{
    cppiHeap = HeapMem_Handle_upCast (cppiLocalHeap);
}
#endif
/**
 *  @b Description
 *  @n
 *      Utility function which converts a local GEM L2 memory address
 *      to global memory address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Computed L2 global Address
 */
uint32_t l2_global_address (uint32_t addr)
{
#ifdef _TMS320C6X
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
}
#endif

/*****************************************************************************
 * Divert with error check
 *****************************************************************************/
void queue_divert_and_check (Qmss_QueueHnd src, Qmss_QueueHnd dst)
{
    Qmss_Result result;

    if ((result = Qmss_queueDivert (src, dst, Qmss_Location_TAIL)) != QMSS_SOK)
    {
#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
        /* Get the core number. */
        coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
        coreNum = 0;
#endif
#else
        coreNum = 0;
#endif
        errorCount++;
        System_printf("core %d: queue divert from %d to %d failed: %d\n", coreNum, src, dst, result);
    }
}

void testCpdmaGlobalConfig (Cppi_Handle cppiHnd)
{
    Cppi_Result         result;
    uint32_t            origLoopback;

    System_printf ("~~~~~~~~~~~~~Core %d TESTING CPDMA Global CONFIGURATION ~~~~~~~~~~~~\n", coreNum);

    if ((result = Cppi_getCpdmaLoopback (cppiHnd)) < CPPI_SOK)
    {
        System_printf ("Error Core %d : Getting CPDMA loopback configuration on startup\n", coreNum);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : CPDMA Loopback on startup is %d\n", coreNum, result);
        origLoopback = result;
    }
    if ((result = Cppi_setCpdmaLoopback (cppiHnd, 0)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Disabling CPDMA loopback\n", coreNum);
        errorCount++;
    }
    else
    {
        if ((result = Cppi_getCpdmaLoopback (cppiHnd)) < CPPI_SOK)
        {
            System_printf ("Error Core %d : Getting CPDMA loopback configuration\n", coreNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : CPDMA loopback is %d\n", coreNum, result);
    }

    if ((result = Cppi_setCpdmaLoopback (cppiHnd, 1)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling CPDMA loopback\n", coreNum);
        errorCount++;
    }
    else
    {
        if ((result = Cppi_getCpdmaLoopback (cppiHnd)) < CPPI_SOK)
        {
            System_printf ("Error Core %d : Getting CPDMA loopback configuration\n", coreNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : CPDMA loopback is %d\n", coreNum, result);
    }

    if ((result = Cppi_setCpdmaLoopback (cppiHnd, origLoopback) != CPPI_SOK))
    {
        System_printf ("Error Core %d : Enabling CPDMA loopback\n", coreNum);
        errorCount++;
    }
    else
    {
        if ((result = Cppi_getCpdmaLoopback (cppiHnd)) < CPPI_SOK)
        {
            System_printf ("Error Core %d : Getting CPDMA loopback configuration\n", coreNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : CPDMA loopback is %d\n", coreNum, result);
    }
}

void testRxChannelConfig (Cppi_Handle cppiHnd)
{
    uint8_t             isAllocated;
    Cppi_ChHnd          ChHnd;
    Cppi_RxChInitCfg    ChCfg;
    uint32_t            ChNum;
    Cppi_Result         result;

    System_printf ("~~~~~~~~~~~~~Core %d TESTING RX CHANNEL CONFIGURATION ~~~~~~~~~~~~\n", coreNum);

    /* Don't specify channel number and let CPPI allocate the next available one */
    ChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    ChCfg.rxEnable = Cppi_ChState_CHANNEL_ENABLE;

    /* Open Channel */
    ChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &ChCfg, &isAllocated);
    if (ChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, ChCfg.channelNum);
        errorCount++;
    }
    else
    {
        ChNum = Cppi_getChannelNumber (ChHnd);
        System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, ChNum);
    }

    /* Disable channel */
    if (Cppi_channelDisable (ChHnd) != CPPI_SOK)
        System_printf ("Error Core %d : Disabling Rx channel : %d\n", coreNum, ChNum);
    else
        System_printf ("Core %d : Disabled Rx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_DISABLE)
        {
            System_printf ("Error Core %d : Incorrect Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel status : %d\n", coreNum, result);
    }

    /* Enable channel */
    if (Cppi_channelEnable (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling Rx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Enabled Rx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel status : %d\n", coreNum, result);
    }

    /* Pause channel */
    if (Cppi_channelPause (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Pausing Rx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Paused Rx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel status : %d\n", coreNum, result);
    }

    /* Disable channel */
    if (Cppi_channelDisable (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Disabling Rx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Disabled Rx channel : %d\n", coreNum, ChNum);

    /* Close Channel */
    if ((result = Cppi_channelClose (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n", coreNum, ChNum, result);

    /* Testing teardown */
    /* Open Channel */
    ChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &ChCfg, &isAllocated);
    if (ChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, ChCfg.channelNum);
        errorCount++;
    }
    else
    {
        ChNum = Cppi_getChannelNumber (ChHnd);
        System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, ChNum);
    }

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel status : %d\n", coreNum, result);
    }

    /* Test blocking teardown */
    if (Cppi_channelTeardown (ChHnd, Cppi_Wait_WAIT) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Tearing down Rx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Torn down Rx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result == Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel status : %d\n", coreNum, result);
    }

    /* Close Channel */
    if ((result = Cppi_channelClose (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n", coreNum, ChNum, result);

    /* Open Channel */
    ChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &ChCfg, &isAllocated);
    if (ChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, ChCfg.channelNum);
        errorCount++;
    }
    else
    {
        ChNum = Cppi_getChannelNumber (ChHnd);
        System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, ChNum);
    }

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel status : %d\n", coreNum, result);
    }

    /* Test non-blocking teardown */
    if (Cppi_channelTeardown (ChHnd, Cppi_Wait_NO_WAIT) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Tearing down Rx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Torn down Rx channel : %d\n", coreNum, ChNum);

    do{
        /* Get channel status */
        if ((result = Cppi_channelStatus (ChHnd)) < 0)
        {
            System_printf ("Error Core %d : Obtaining Rx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
    }while (result == Cppi_ChState_CHANNEL_ENABLE);

    /* Close Channel */
    if ((result = Cppi_channelClose (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n", coreNum, ChNum, result);

    System_printf ("~~~~~~~~~~~~~Core %d RX CHANNEL CONFIGURATION DONE~~~~~~~~~~~~~~~~\n", coreNum);
}

void testRxChannelAllocate (Cppi_Handle cppiHnd)
{
    uint8_t             isAllocated;
    uint32_t            i;
    Cppi_Result         result;
    /* Rx channel configuration */
    Cppi_RxChInitCfg    rxChCfg;

    /* Don't specify channel number and let CPPI allocate the next available one */

    System_printf ("*************Core %d TESTING RX CHANNEL ***************\n", coreNum);

    System_printf ("~~~~~~~~~~~~~Core %d RX CHANNEL LLD ALLOCATE~~~~~~~~~~~\n", coreNum);
    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_ENABLE;

    /* Open Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
        if (rxChHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));
    }

    System_printf ("~~~~~~~~~~~~~Core %d TESTING RX CHANNEL CLOSE~~~~~~~~~~\n", coreNum);
    /* Close Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (rxChHnd[i]);

        if ((result = Cppi_channelClose (rxChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d RX CHANNEL SPECIFY~~~~~~~~~~~~~~~~\n", coreNum);

    /* Open Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        rxChCfg.channelNum = i;
        rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
        if (rxChHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));

    }

    /* Open Rx Channel the 2nd time */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        rxChCfg.channelNum = i;
        rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
        if (rxChHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));

    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Channel close with multiple opens~~~~~~~~~~\n", coreNum);

    /* Close Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (rxChHnd[i]);

        if ((result = Cppi_channelClose (rxChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Channel close for last open~~~~~~~~~~\n", coreNum);

    /* Close Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (rxChHnd[i]);

        if ((result = Cppi_channelClose (rxChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Channel open for odd channels~~~~~~~~~~\n", coreNum);

    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        rxChCfg.channelNum = i;
        if (i%2)
        {
            rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
            if (rxChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));
        }
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Channel open for remaining channels~~~~~~~~~~\n", coreNum);

    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        if (!(i%2))
        {
            rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
            if (rxChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));
        }
    }
    /* Close Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (rxChHnd[i]);

        if ((result = Cppi_channelClose (rxChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Channel open for even channels~~~~~~~~~~\n", coreNum);

    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        rxChCfg.channelNum = i;
        if (!(i%2))
        {
            rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
            if (rxChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));
        }
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Channel open for remaining channels~~~~~~~~~~\n", coreNum);
    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        if (i%2)
        {
            rxChHnd[i] = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
            if (rxChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd[i]));
        }
    }
    /* Close Rx Channel */
    for (i = 0; i < NUM_RX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (rxChHnd[i]);

        if ((result = Cppi_channelClose (rxChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }
}

void testTxChannelConfig (Cppi_Handle cppiHnd)
{
    uint8_t             isAllocated;
    Cppi_ChHnd          ChHnd;
    Cppi_TxChInitCfg    ChCfg;
    uint32_t            ChNum;
    Cppi_Result         result;

    System_printf ("~~~~~~~~~~~~~Core %d TESTING TX CHANNEL CONFIGURATION ~~~~~~~~~~~~\n", coreNum);

    /* Don't specify channel number and let CPPI allocate the next available one */
    ChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    ChCfg.priority = 2;
    ChCfg.filterEPIB = 1;
    ChCfg.filterPS = 1;
    ChCfg.aifMonoMode = 1;
    ChCfg.txEnable = Cppi_ChState_CHANNEL_ENABLE;

    /* Open Channel */
    ChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &ChCfg, &isAllocated);
    if (ChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, ChCfg.channelNum);
        errorCount++;
    }
    else
    {
        ChNum = Cppi_getChannelNumber (ChHnd);
        System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, ChNum);
    }

    /* Disable channel */
    if (Cppi_channelDisable (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Disabling Tx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Disabled Tx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_DISABLE)
        {
            System_printf ("Error Core %d : Incorrect Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel status : %d\n", coreNum, result);
    }

    /* Enable channel */
    if (Cppi_channelEnable (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling Tx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Enabled Tx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel status : %d\n", coreNum, result);
    }

    /* Pause channel */
    if (Cppi_channelPause (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Pausing Tx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Paused Tx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel status : %d\n", coreNum, result);
    }

    /* Disable channel */
    if (Cppi_channelDisable (ChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Disabling Tx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Disabled Tx channel : %d\n", coreNum, ChNum);

    /* Close Channel */
    if ((result = Cppi_channelClose (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n", coreNum, ChNum, result);

    /* Testing teardown */
    /* Open Channel */
    ChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &ChCfg, &isAllocated);
    if (ChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, ChCfg.channelNum);
        errorCount++;
    }
    else
    {
        ChNum = Cppi_getChannelNumber (ChHnd);
        System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, ChNum);
    }

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel status : %d\n", coreNum, result);
    }

    /* Test blocking teardown */
    if (Cppi_channelTeardown (ChHnd, Cppi_Wait_WAIT) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Tearing down Tx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Torn down Tx channel : %d\n", coreNum, ChNum);

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result == Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel status : %d\n", coreNum, result);
    }

    /* Close Channel */
    if ((result = Cppi_channelClose (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n", coreNum, ChNum, result);

    /* Open Channel */
    ChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &ChCfg, &isAllocated);
    if (ChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, ChCfg.channelNum);
        errorCount++;
    }
    else
    {
        ChNum = Cppi_getChannelNumber (ChHnd);
        System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, ChNum);
    }

    /* Get channel status */
    if ((result = Cppi_channelStatus (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
    {
        if (result != Cppi_ChState_CHANNEL_ENABLE)
        {
            System_printf ("Error Core %d : Incorrect Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel status : %d\n", coreNum, result);
    }

    /* Test non-blocking teardown */
    if (Cppi_channelTeardown (ChHnd, Cppi_Wait_NO_WAIT) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Tearing down Tx channel : %d\n", coreNum, ChNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Torn down Tx channel : %d\n", coreNum, ChNum);

    do{
        /* Get channel status */
        if ((result = Cppi_channelStatus (ChHnd)) < 0)
        {
            System_printf ("Error Core %d : Obtaining Tx channel status: %d\n", coreNum, ChNum);
            errorCount++;
        }
    }while (result == Cppi_ChState_CHANNEL_ENABLE);

    /* Close Channel */
    if ((result = Cppi_channelClose (ChHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n", coreNum, ChNum, result);

    System_printf ("~~~~~~~~~~~~~Core %d TX CHANNEL CONFIGURATION DONE~~~~~~~~~~~~~~~~\n", coreNum);
}

void testTxChannelAllocate (Cppi_Handle cppiHnd)
{
    uint8_t             isAllocated;
    uint32_t            i;
    Cppi_Result         result;
    /* Tx channel configuration */
    Cppi_TxChInitCfg    txChCfg;

    /* Don't specify channel number and let CPPI allocate the next available one */

    System_printf ("*************Core %d TESTING Tx CHANNEL ***************\n", coreNum);

    System_printf ("~~~~~~~~~~~~~Core %d Tx CHANNEL LLD ALLOCATE~~~~~~~~~~~\n", coreNum);

    /* Don't specify channel number and let CPPI allocate the next available one */
    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    txChCfg.priority = 2;
    txChCfg.filterEPIB = 0;
    txChCfg.filterPS = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_ENABLE;

    /* Open Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
        if (txChHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));

    }

    System_printf ("~~~~~~~~~~~~~Core %d TESTING Tx CHANNEL CLOSE~~~~~~~~~~\n", coreNum);

    /* Close Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (txChHnd[i]);

        if ((result = Cppi_channelClose (txChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Tx CHANNEL SPECIFY~~~~~~~~~~~~~~~~\n", coreNum);

    /* Open Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        txChCfg.channelNum = i;
        txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
        if (txChHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));

    }

    /* Open Tx Channel the 2nd time */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        txChCfg.channelNum = i;
        txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
        if (txChHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));

    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Tx Channel close with multiple opens~~~~~~~~~~\n", coreNum);
    /* Close Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (txChHnd[i]);

        if ((result = Cppi_channelClose (txChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Tx Channel close for last open~~~~~~~~~~\n", coreNum);

    /* Close Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (txChHnd[i]);

        if ((result = Cppi_channelClose (txChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Tx Channel open for odd channels~~~~~~~~~~\n", coreNum);

    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        txChCfg.channelNum = i;
        if (i%2)
        {
            txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
            if (txChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));
        }
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Tx Channel open for remaining channels~~~~~~~~~~\n", coreNum);
    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        if (!(i%2))
        {
            txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
            if (txChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));
        }
    }
    /* Close Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (txChHnd[i]);

        if ((result = Cppi_channelClose (txChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Tx Channel open for even channels~~~~~~~~~~\n", coreNum);

    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        txChCfg.channelNum = i;
        if (!(i%2))
        {
            txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
            if (txChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));
        }
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Tx Channel open for remaining channels~~~~~~~~~~\n", coreNum);
    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        if (i%2)
        {
            txChHnd[i] = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
            if (txChHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd[i]));
        }
    }
    /* Close Tx Channel */
    for (i = 0; i < NUM_TX_CHANNELS; i++)
    {
        uint32_t temp =  Cppi_getChannelNumber (txChHnd[i]);

        if ((result = Cppi_channelClose (txChHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Tx channel error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Tx Channel %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }
}

void testRxFlowConfig (Cppi_Handle cppiHnd)
{
    uint8_t             isAllocated;
    Cppi_FlowHnd        rxFlowHnd;
    Cppi_RxFlowCfg      rxFlowCfg;
    uint32_t            temp;
    Cppi_Result         result;

    System_printf ("***********Core %d TESTING RX FLOW CONFIGURATION*************\n", coreNum);

    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Request any flow number */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
    rxFlowCfg.rx_dest_qnum = 100;
    rxFlowCfg.rx_dest_qmgr = 0;
    rxFlowCfg.rx_sop_offset= 16;
    rxFlowCfg.rx_ps_location = Cppi_PSLoc_PS_IN_SOP;
    rxFlowCfg.rx_desc_type = Cppi_DescType_HOST;
    rxFlowCfg.rx_error_handling = 1;
    rxFlowCfg.rx_psinfo_present = 1;
    rxFlowCfg.rx_einfo_present = 1;
    rxFlowCfg.rx_dest_tag_lo = 0xf1;
    rxFlowCfg.rx_dest_tag_hi = 0xf4;
    rxFlowCfg.rx_src_tag_lo = 0x60;
    rxFlowCfg.rx_src_tag_hi = 0x48;
    rxFlowCfg.rx_size_thresh0_en = 1;
    rxFlowCfg.rx_size_thresh1_en = 1;
    rxFlowCfg.rx_size_thresh2_en = 0;
    rxFlowCfg.rx_dest_tag_lo_sel = 2;
    rxFlowCfg.rx_dest_tag_hi_sel = 3;
    rxFlowCfg.rx_src_tag_lo_sel = 4;
    rxFlowCfg.rx_src_tag_hi_sel = 5;
    rxFlowCfg.rx_fdq1_qnum = 1000;
    rxFlowCfg.rx_fdq1_qmgr = 0;
    rxFlowCfg.rx_fdq0_sz0_qnum = 2687;
    rxFlowCfg.rx_fdq0_sz0_qmgr = 0;
    rxFlowCfg.rx_fdq3_qnum = 5000;
    rxFlowCfg.rx_fdq3_qmgr = 1;
    rxFlowCfg.rx_fdq2_qnum = 7800;
    rxFlowCfg.rx_fdq2_qmgr = 1;
    rxFlowCfg.rx_size_thresh1 = 512;
    rxFlowCfg.rx_size_thresh0 = 256;
    rxFlowCfg.rx_fdq0_sz1_qnum = 1;
    rxFlowCfg.rx_fdq0_sz1_qmgr = 0;
    rxFlowCfg.rx_size_thresh2 = 1024;
    rxFlowCfg.rx_fdq0_sz3_qnum = 721;
    rxFlowCfg.rx_fdq0_sz3_qmgr= 0;
    rxFlowCfg.rx_fdq0_sz2_qnum = 32;
    rxFlowCfg.rx_fdq0_sz2_qmgr = 0;

    /* Open Rx Flow */

    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
        errorCount++;
    }
    else
        System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd));

    /* Close Rx Flow */

    temp = Cppi_getFlowId (rxFlowHnd);

    if ((result = Cppi_closeRxFlow (rxFlowHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing Rx flow error code: %d\n", coreNum, result);
        errorCount++;
    }
    else
        System_printf ("Core %d : Rx Flow %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    System_printf ("***********Core %d TESTING RX FLOW CONFIGURATION DONE*************\n", coreNum);
}

void testRxFlowAllocate (Cppi_Handle cppiHnd)
{
    uint8_t             isAllocated;
    uint32_t            i;
    Cppi_Result         result;
    Cppi_RxFlowCfg      rxFlowCfg;
    /* Rx flow handles */
    Cppi_FlowHnd        rxFlowHnd[NUM_RX_FLOWS];

    System_printf ("*************Core %d TESTING RX FLOW ***************\n", coreNum);

    System_printf ("~~~~~~~~~~~~~Core %d RX FLOW LLD ALLOCATE~~~~~~~~~~~\n", coreNum);

    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Don't specify flow number and let CPPI allocate the next available one */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;

    /* Open Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
        if (rxFlowHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));

    }

    System_printf ("~~~~~~~~~~~~~Core %d TESTING RX FLOW CLOSE~~~~~~~~~~\n", coreNum);
    /* Close Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        uint32_t temp =  Cppi_getFlowId (rxFlowHnd[i]);

        if ((result = Cppi_closeRxFlow (rxFlowHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx flow error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Flow %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d RX FLOW SPECIFY~~~~~~~~~~~~~~~~\n", coreNum);

    /* Open Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        rxFlowCfg.flowIdNum = i;
        rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
        if (rxFlowHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));

    }

    /* Open Rx Flow the 2nd time */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        rxFlowCfg.flowIdNum = i;
        rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
        if (rxFlowHnd[i] == NULL)
        {
            System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
            errorCount++;
        }
        else
            System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));

    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Flow close with multiple opens~~~~~~~~~~\n", coreNum);

    /* Close Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        uint32_t temp =  Cppi_getFlowId (rxFlowHnd[i]);

        if ((result = Cppi_closeRxFlow (rxFlowHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx flow error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Flow %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Flow close for last open~~~~~~~~~~\n", coreNum);

    /* Close Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        uint32_t temp =  Cppi_getFlowId (rxFlowHnd[i]);

        if ((result = Cppi_closeRxFlow (rxFlowHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx flow error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Flow %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Flow open for odd flows~~~~~~~~~~\n", coreNum);

    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        rxFlowCfg.flowIdNum = i;
        if (i%2)
        {
            rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
            if (rxFlowHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));
        }
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Flow open for remaining flows~~~~~~~~~~\n", coreNum);

    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        if (!(i%2))
        {
            rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
            if (rxFlowHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));
        }
    }
    /* Close Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        uint32_t temp =  Cppi_getFlowId (rxFlowHnd[i]);

        if ((result = Cppi_closeRxFlow (rxFlowHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx flow error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Flow %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Flow open for even flows~~~~~~~~~~\n", coreNum);

    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        rxFlowCfg.flowIdNum = i;
        if (!(i%2))
        {
            rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
            if (rxFlowHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));
        }
    }

    System_printf ("~~~~~~~~~~~~~Core %d Testing Rx Flow open for remaining flows~~~~~~~~~~\n", coreNum);

    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        if (i%2)
        {
            rxFlowHnd[i] = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
            if (rxFlowHnd[i] == NULL)
            {
                System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
                errorCount++;
            }
            else
                System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId (rxFlowHnd[i]));
        }
    }
    /* Close Rx Flow */
    for (i = 0; i < NUM_RX_FLOWS; i++)
    {
        uint32_t temp =  Cppi_getFlowId (rxFlowHnd[i]);

        if ((result = Cppi_closeRxFlow (rxFlowHnd[i])) < 0)
        {
            System_printf ("Error Core %d : Closing Rx flow error code: %d\n", coreNum, result);
            errorCount++;
        }
        else
            System_printf ("Core %d : Rx Flow %d closed successfully. Ref count :%d\n",
                            coreNum, temp, result);
    }
}

void testHostDescConfig (void)
{
    Cppi_Result         result;
    uint32_t            i, destLen, hostPktSize = 0, size;
    Cppi_Desc           *hostDesc, *hostDesc1, *hostDesc2, *nextBD, *tempBD;
    Cppi_DescType       descType;
    Qmss_Queue          sQueue, gQueue;
    uint8_t             *dataBuffPtr;
    uint32_t            timestamp, refCount, newRefCount;
    Cppi_DescTag        txTag, rxTag;

    System_printf ("\n~~~~~~~~~~~~~Core %d Testing Host descriptor fields~~~~~~~~~~~\n", coreNum);

    hostDesc = (Cppi_Desc *) Qmss_queuePop (freeHostQueHnd);
    descType = Cppi_getDescType (hostDesc);
    if (descType != Cppi_DescType_HOST)
    {
        System_printf ("Error Core %d : Incorrect descriptor type : %d\n", coreNum, descType);
        errorCount++;
    }

    Cppi_setDescType (hostDesc, Cppi_DescType_MONOLITHIC);
    descType = Cppi_getDescType (hostDesc);
    if (descType != Cppi_DescType_MONOLITHIC)
    {
        System_printf ("Error Core %d : Incorrect descriptor type : %d\n", coreNum, descType);
        errorCount++;
    }

    Cppi_setDescType (hostDesc, Cppi_DescType_HOST);
    descType = Cppi_getDescType (hostDesc);
    if (descType != Cppi_DescType_HOST)
    {
        System_printf ("Error Core %d : Incorrect descriptor type : %d\n", coreNum, descType);
        errorCount++;
    }

    nextBD = (Cppi_Desc *) Qmss_queuePop (freeHostQueHnd);

    Cppi_linkNextBD (Cppi_DescType_HOST, hostDesc, nextBD);

    tempBD = Cppi_getNextBD (Cppi_DescType_HOST, hostDesc);

    if (nextBD != tempBD)
    {
        System_printf ("Error Core %d : Linking next BD is incorrect HostDesc : 0x%p Next BD linked: 0x%p, Next BD read: 0x%p\n",
                        coreNum, hostDesc, nextBD, tempBD);
        errorCount++;
    }

    Cppi_linkNextBD (Cppi_DescType_HOST, hostDesc, NULL);

    tempBD = Cppi_getNextBD (Cppi_DescType_HOST, hostDesc);

    if (tempBD != NULL)
    {
        System_printf ("Error Core %d : NULL Linking is incorrect HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++)
        dataBuff[i] = i;

    /* Add data buffer */
    Cppi_setData (Cppi_DescType_HOST, hostDesc, (uint8_t *) l2_global_address ((uint32_t) dataBuff), SIZE_DATA_BUFFER);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, hostDesc, (uint8_t *) l2_global_address ((uint32_t) dataBuff), SIZE_DATA_BUFFER);

    /* Set packet length */
    Cppi_setPacketLen (Cppi_DescType_HOST, hostDesc, SIZE_DATA_BUFFER);

    if ((result = Cppi_getPacketLen (Cppi_DescType_HOST, hostDesc)) != SIZE_DATA_BUFFER)
    {
        System_printf ("Error Core %d : Set packet len %d Get packet len %d HostDesc : 0x%p\n",
                        coreNum, SIZE_DATA_BUFFER, result, hostDesc);
        errorCount++;
    }

    /* Get data buffer */
    Cppi_getData (Cppi_DescType_HOST, hostDesc, &dataBuffPtr, &destLen);

    if (dataBuffPtr != ((uint8_t *) l2_global_address ((uint32_t) dataBuff)))
    {
        System_printf ("Error Core %d : Set data buff 0x%x Get data buff 0x%p HostDesc : 0x%p\n",
                        coreNum, l2_global_address ((uint32_t) dataBuff), dataBuffPtr, hostDesc);
        errorCount++;
    }

    if (destLen != SIZE_DATA_BUFFER)
    {
        System_printf ("Error Core %d : Set data buff Len %d Get data buff Len %d HostDesc : 0x%p\n",
                        coreNum, SIZE_DATA_BUFFER, destLen, hostDesc);
        errorCount++;
    }
    /* Get original buffer info */
    dataBuffPtr = 0;
    destLen = 0;
    Cppi_getOriginalBufInfo (Cppi_DescType_HOST, hostDesc, &dataBuffPtr, &destLen);

    if (dataBuffPtr != ((uint8_t *) l2_global_address ((uint32_t) dataBuff)))
    {
        System_printf ("Error Core %d : Set original data buff 0x%x Get data buff 0x%p HostDesc : 0x%p\n",
                        coreNum, l2_global_address ((uint32_t) dataBuff), dataBuffPtr, hostDesc);
        errorCount++;
    }

    if (destLen != SIZE_DATA_BUFFER)
    {
        System_printf ("Error Core %d : Set original data buff Len %d Get data buff Len %d HostDesc : 0x%p\n",
                        coreNum, SIZE_DATA_BUFFER, destLen, hostDesc);
        errorCount++;
    }

    /* Test optional fields */
    Cppi_setTimeStamp (Cppi_DescType_HOST, hostDesc, 0x7E3B0C8A);

    result = Cppi_getTimeStamp (Cppi_DescType_HOST, hostDesc, &timestamp);
    if (result != CPPI_SOK)
    {
        System_printf ("Error Core %d : Timestamp - EPIB not set HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Set software info */
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, hostDesc, dataBuff);

    dataBuffPtr = 0;
    Cppi_getSoftwareInfo (Cppi_DescType_HOST, hostDesc, &dataBuffPtr);
    if (result != CPPI_SOK)
    {
        System_printf ("Error Core %d : Software Info - EPIB not set HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    for (i=0; i < 12; i++)
    {
        if (dataBuff[i] != dataBuffPtr[i])
        {
            System_printf ("Error Core %d : In software info set: %02X - get: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
            errorCount++;
            break;
        }
    }

    /* Set software info 0 */
    Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, hostDesc, 0xF0);

    result  = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, hostDesc);
    if (result != 0xF0)
    {
        System_printf ("Error Core %d : Software Info0 - Value %d is incorrect in HostDesc : 0x%p\n", coreNum, result, hostDesc);
        errorCount++;
    }

    /* Set software info 1 */
    Cppi_setSoftwareInfo1 (Cppi_DescType_HOST, hostDesc, 0x83546);

    result  = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, hostDesc);
    if (result != 0x83546)
    {
        System_printf ("Error Core %d : Software Info1 - Value %d is incorrect in HostDesc : 0x%p\n", coreNum, result, hostDesc);
        errorCount++;
    }

    /* Set software info 2 */
    Cppi_setSoftwareInfo2 (Cppi_DescType_HOST, hostDesc, 0xFFF0FFFF);

    result  = Cppi_getSoftwareInfo2 (Cppi_DescType_HOST, hostDesc);
    if (result != 0xFFF0FFFF)
    {
        System_printf ("Error Core %d : Software Info2 - Value %d is incorrect in HostDesc : 0x%p\n", coreNum, result, hostDesc);
        errorCount++;
    }

    /* Test return policy */

    Cppi_setReturnPolicy (Cppi_DescType_HOST, hostDesc, Cppi_ReturnPolicy_RETURN_BUFFER);

    if (Cppi_getReturnPolicy (Cppi_DescType_HOST, hostDesc) != Cppi_ReturnPolicy_RETURN_BUFFER)
    {
        System_printf ("Error Core %d : Incorrect return policy HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    Cppi_setReturnPolicy (Cppi_DescType_HOST, hostDesc, Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET);

    if (Cppi_getReturnPolicy (Cppi_DescType_HOST, hostDesc) != Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET)
    {
        System_printf ("Error Core %d : Incorrect return policy HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Test return push policy */
    Cppi_setReturnPushPolicy (Cppi_DescType_HOST, hostDesc, Qmss_Location_HEAD);

    if (Cppi_getReturnPushPolicy (Cppi_DescType_HOST, hostDesc) != Qmss_Location_HEAD)
    {
        System_printf ("Error Core %d : Incorrect return push policy HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    Cppi_setReturnPushPolicy (Cppi_DescType_HOST, hostDesc, Qmss_Location_TAIL);

    if (Cppi_getReturnPushPolicy (Cppi_DescType_HOST, hostDesc) != Qmss_Location_TAIL)
    {
        System_printf ("Error Core %d : Incorrect return push policy HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Test return queue */
    sQueue.qMgr = 1;
    sQueue.qNum = 3648;
    Cppi_setReturnQueue (Cppi_DescType_HOST, hostDesc, sQueue);
    gQueue = Cppi_getReturnQueue (Cppi_DescType_HOST, hostDesc);

    if (sQueue.qMgr != gQueue.qMgr || sQueue.qNum != gQueue.qNum)
    {
        System_printf ("Error Core %d : Incorrect return queue HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Test original buffer pool index settings */
    Cppi_setOrigBufferpooIndex (Cppi_DescType_HOST, hostDesc, 5);
    if (Cppi_getOrigBufferpooIndex (Cppi_DescType_HOST, hostDesc) != 5)
    {
        System_printf ("Error Core %d : Incorrect original buffer pool index HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    Cppi_setOrigBufferpooIndex (Cppi_DescType_HOST, hostDesc, 0xF);
    if (Cppi_getOrigBufferpooIndex (Cppi_DescType_HOST, hostDesc) != 0xF)
    {
        System_printf ("Error Core %d : Incorrect original buffer pool index HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }


    /* Test reference count */
    refCount = Cppi_getRefCount (Cppi_DescType_HOST, hostDesc);

    System_printf ("Core %d : Original reference count HostDesc : 0x%p ref Count : %d\n", coreNum, hostDesc, refCount);

    Cppi_incrementRefCount (Cppi_DescType_HOST, hostDesc);

    newRefCount = Cppi_getRefCount (Cppi_DescType_HOST, hostDesc);
    if (newRefCount != refCount + 1)
    {
        System_printf ("Error Core %d : Incorrect reference count HostDesc : 0x%p ref Count : %d New ref Count : %d\n", coreNum,
                        hostDesc, refCount, newRefCount);
        errorCount++;
    }

    Cppi_decrementRefCount (Cppi_DescType_HOST, hostDesc);

    newRefCount = Cppi_getRefCount (Cppi_DescType_HOST, hostDesc);
    if (newRefCount != refCount)
    {
        System_printf ("Error Core %d : Incorrect reference count HostDesc : 0x%p ref Count : %d New ref Count : %d\n", coreNum,
                        hostDesc, refCount, newRefCount);
        errorCount++;
    }

    /* Test PS location */
    Cppi_setPSLocation (Cppi_DescType_HOST, hostDesc, Cppi_PSLoc_PS_IN_SOP);
    if (Cppi_getPSLocation (Cppi_DescType_HOST, hostDesc) != Cppi_PSLoc_PS_IN_SOP)
    {
        System_printf ("Error Core %d : Incorrect PS location HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    Cppi_setPSLocation (Cppi_DescType_HOST, hostDesc, Cppi_PSLoc_PS_IN_DESC);
    if (Cppi_getPSLocation (Cppi_DescType_HOST, hostDesc) != Cppi_PSLoc_PS_IN_DESC)
    {
        System_printf ("Error Core %d : Incorrect PS location HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }
    /* Test PS data with optional fields */
    /* Fill in some data */
    dataBuff[0] = 0xAB;
    dataBuff[1] = 0xCD;
    dataBuff[2] = 0xEF;
    dataBuff[3] = 0xDC;

    /* Add PS data */
    Cppi_setPSData (Cppi_DescType_HOST, hostDesc, (uint8_t *) dataBuff, 8);

    dataBuffPtr = 0;
    destLen = 0;
    /* Get PS data */
    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, hostDesc, &dataBuffPtr, &destLen) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Getting PS data HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    for (i=0; i < destLen; i++)
    {
        if (dataBuff[i] != dataBuffPtr[i])
        {
            System_printf ("Error Core %d : In PS data Tx: %02X - Rx: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
            errorCount++;
            break;
        }
    }
    if (Cppi_getPSLen (Cppi_DescType_HOST, hostDesc) != destLen)
    {
        System_printf ("Error Core %d : PS data Len HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Test PS data without optional fields */
    hostDesc = (Cppi_Desc *) Qmss_queuePop (freeHostQueHnd);

    Cppi_setPSLocation (Cppi_DescType_HOST, hostDesc, Cppi_PSLoc_PS_IN_DESC);
    if (Cppi_getPSLocation (Cppi_DescType_HOST, hostDesc) != Cppi_PSLoc_PS_IN_DESC)
    {
        System_printf ("Error Core %d : Incorrect PS location HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

        /* Fill in some data */
    dataBuff[0] = 0xAB;
    dataBuff[1] = 0xCD;
    dataBuff[2] = 0xEF;
    dataBuff[3] = 0xDC;

    /* Add PS data */
    Cppi_setPSData (Cppi_DescType_HOST, hostDesc, (uint8_t *) dataBuff, 8);

    dataBuffPtr = 0;
    destLen = 0;
    /* Get PS data */
    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, hostDesc, &dataBuffPtr, &destLen) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Getting PS data HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    for (i=0; i < destLen; i++)
    {
        if (dataBuff[i] != dataBuffPtr[i])
        {
            System_printf ("Error Core %d : In PS data Tx: %02X - Rx: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
            errorCount++;
            break;
        }
    }
    if (Cppi_getPSLen (Cppi_DescType_HOST, hostDesc) != destLen)
    {
        System_printf ("Error Core %d : PS data Len HostDesc : 0x%p\n", coreNum, hostDesc);
        errorCount++;
    }

    /* Set Tags */
    txTag.destTagLo = 128;
    txTag.destTagHi = 6;
    txTag.srcTagLo = 32;
    txTag.srcTagHi = 223;

    Cppi_setTag (Cppi_DescType_HOST, hostDesc, &txTag);

    /*Get Tag */
    rxTag = Cppi_getTag (Cppi_DescType_HOST, hostDesc);

    if (txTag.destTagLo != rxTag.destTagLo)
    {
        System_printf ("Error Core %d : Incorrect destination Low tag Tx %d Rx %d\n", coreNum, txTag.destTagLo, rxTag.destTagLo);
        errorCount++;
    }
    else
        System_printf ("Core %d : Correct destination Low tag\n", coreNum);

    if (txTag.destTagHi != rxTag.destTagHi)
    {
        System_printf ("Error Core %d : Incorrect destination High tag Tx %d Rx %d\n", coreNum, txTag.destTagHi, rxTag.destTagHi);
        errorCount++;
    }
    else
        System_printf ("Core %d : Correct destination High tag\n", coreNum);

    if (txTag.srcTagLo != rxTag.srcTagLo)
    {
        System_printf ("Error Core %d : Incorrect source Low tag Tx %d Rx %d\n", coreNum, txTag.srcTagLo, rxTag.srcTagLo);
        errorCount++;
    }
    else
        System_printf ("Core %d : Correct source Low tag\n", coreNum);

    if (txTag.srcTagHi != rxTag.srcTagHi)
    {
        System_printf ("Error Core %d : Incorrect source High tag Tx %d Rx %d\n", coreNum, txTag.srcTagHi, rxTag.srcTagHi);
        errorCount++;
    }
    else
        System_printf ("Core %d : Correct source High tag\n", coreNum);

#ifndef NSS_LITE /* NSS LITE qmss does not support this feature */

    /* Test pop address and size */
    hostDesc = (Cppi_Desc *) Qmss_queuePop (freeHostQueHnd);
    System_printf ("Core %d : Descriptor address : 0x%p\n", coreNum, hostDesc);

    Qmss_queuePush (freeHostQueHnd, hostDesc, SIZE_DATA_BUFFER, SIZE_HOST_DESC, Qmss_Location_HEAD);
    while (Qmss_getQueueEntryCount(freeHostQueHnd) == 0);
    Qmss_queuePopDescSize (freeHostQueHnd, (void *)&hostDesc1, &hostPktSize);

    if (hostPktSize != SIZE_DATA_BUFFER)
    {
        System_printf ("Error Core %d : Descriptor address : 0x%p Incorrect packet size %d\n",
                       coreNum, hostDesc1, hostPktSize);
        errorCount++;
    }
    else
        System_printf ("Core %d : Descriptor address : 0x%p of packet size : %d\n",
                        coreNum, hostDesc1, hostPktSize);

    Qmss_queuePush (freeHostQueHnd, hostDesc1, SIZE_DATA_BUFFER * 4, SIZE_HOST_DESC, Qmss_Location_HEAD);
    while (Qmss_getQueueEntryCount(freeHostQueHnd) == 0);
    Qmss_queuePopDescSize (freeHostQueHnd, (void *)&hostDesc2, &hostPktSize);

    if (hostPktSize != SIZE_DATA_BUFFER * 4)
    {
        System_printf ("Error Core %d : Descriptor address : 0x%p Incorrect packet size %d\n",
                       coreNum, hostDesc2, hostPktSize);
        errorCount++;
    }
    else
        System_printf ("Core %d : Descriptor address : 0x%p of packet size : %d\n",
                        coreNum, hostDesc2, hostPktSize);

    System_printf ("Core %d : Descriptor address : 0x%p of packet size : %d\n",
                        coreNum, hostDesc2, hostPktSize);
    size = QMSS_DESC_SIZE (hostDesc2);
    System_printf ("Core %d : Descriptor address : 0x%p of desc size : %d\n",
                        coreNum, hostDesc2, size);
    hostDesc2 = (Cppi_Desc *) QMSS_DESC_PTR (hostDesc2);
    System_printf ("Core %d : Descriptor address : 0x%p of packet size : %d\n",
                        coreNum, hostDesc2, hostPktSize);
                        
#endif                        
    return;
}

void testMonoDescConfig (void)
{
    Cppi_Result         result;
    uint32_t            i, destLen;
    Cppi_Desc           *monoDesc;
    Cppi_DescType       descType;
    Qmss_Queue          sQueue, gQueue;
    uint8_t             *dataBuffPtr;
    uint32_t            timestamp;

    System_printf ("\n~~~~~~~~~~Core %d Testing Monolithic descriptor fields~~~~~~~~\n", coreNum);

    monoDesc = (Cppi_Desc *) Qmss_queuePop (freeMonoQueHnd);
    descType = Cppi_getDescType (monoDesc);
    if (descType != Cppi_DescType_MONOLITHIC)
    {
        System_printf ("Error Core %d : Incorrect descriptor type : %d\n", coreNum, descType);
        errorCount++;
    }

    Cppi_setDescType (monoDesc, Cppi_DescType_HOST);
    descType = Cppi_getDescType (monoDesc);
    if (descType != Cppi_DescType_HOST)
    {
        System_printf ("Error Core %d : Incorrect descriptor type : %d\n", coreNum, descType);
        errorCount++;
    }

    Cppi_setDescType (monoDesc, Cppi_DescType_MONOLITHIC);
    descType = Cppi_getDescType (monoDesc);
    if (descType != Cppi_DescType_MONOLITHIC)
    {
        System_printf ("Error Core %d : Incorrect descriptor type : %d\n", coreNum, descType);
        errorCount++;
    }

    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++)
        dataBuff[i] = i;

    if (Cppi_getDataOffset (Cppi_DescType_MONOLITHIC, monoDesc) != 12)
    {
        System_printf ("Error Core %d : Incorrect data offset %d MonoDesc : 0x%p\n",
                        coreNum, SIZE_DATA_BUFFER, monoDesc);
        errorCount++;
    }

    Cppi_setDataOffset (Cppi_DescType_MONOLITHIC, monoDesc, DATA_OFFSET);

    /* Add data buffer */
    Cppi_setData (Cppi_DescType_MONOLITHIC, monoDesc, (uint8_t *) dataBuff, SIZE_DATA_BUFFER);

    /* Get packet length. Should already be set */
    if ((result = Cppi_getPacketLen (Cppi_DescType_MONOLITHIC, monoDesc)) != SIZE_DATA_BUFFER)
    {
        System_printf ("Error Core %d : Set packet len %d Get packet len %d MonoDesc : 0x%p\n",
                        coreNum, SIZE_DATA_BUFFER, result, monoDesc);
        errorCount++;
    }

    /* Get data buffer */
    Cppi_getData (Cppi_DescType_MONOLITHIC, monoDesc, &dataBuffPtr, &destLen);

    if (dataBuffPtr != ((uint8_t *) monoDesc + DATA_OFFSET))
    {
        System_printf ("Error Core %d : Set data buff 0x%x Get data buff 0x%p MonoDesc : 0x%p\n",
                        coreNum, l2_global_address ((uint32_t) dataBuff), dataBuffPtr, monoDesc);
        errorCount++;
    }

    if (destLen != SIZE_DATA_BUFFER)
    {
        System_printf ("Error Core %d : Set data buff Len %d Get data buff Len %d MonoDesc : 0x%p\n",
                        coreNum, SIZE_DATA_BUFFER, destLen, monoDesc);
        errorCount++;
    }

    /* Test optional fields */
    Cppi_setTimeStamp (Cppi_DescType_MONOLITHIC, monoDesc, 0x7E3B0C8A);

    result = Cppi_getTimeStamp (Cppi_DescType_MONOLITHIC, monoDesc, &timestamp);
    if (result != CPPI_SOK)
    {
        System_printf ("Error Core %d : Timestamp - EPIB not set MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    /* Set software info */
    Cppi_setSoftwareInfo (Cppi_DescType_MONOLITHIC, monoDesc, dataBuff);

    dataBuffPtr = 0;
    Cppi_getSoftwareInfo (Cppi_DescType_MONOLITHIC, monoDesc, &dataBuffPtr);
    if (result != CPPI_SOK)
    {
        System_printf ("Error Core %d : Software Info - EPIB not set MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    for (i=0; i < 12; i++)
    {
        if (dataBuff[i] != dataBuffPtr[i])
        {
            System_printf ("Error Core %d : In software info set: %02X - get: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
            errorCount++;
            break;
        }
    }

    /* Set software info 0 */
    Cppi_setSoftwareInfo0 (Cppi_DescType_MONOLITHIC, monoDesc, 0xFF);

    result  = Cppi_getSoftwareInfo0 (Cppi_DescType_MONOLITHIC, monoDesc);
    if (result != 0xFF)
    {
        System_printf ("Error Core %d : Software Info0 - Value %d is incorrect in MonoDesc : 0x%p\n", coreNum, result, monoDesc);
        errorCount++;
    }

    /* Set software info 1 */
    Cppi_setSoftwareInfo1 (Cppi_DescType_MONOLITHIC, monoDesc, 0x6389);

    result  = Cppi_getSoftwareInfo1 (Cppi_DescType_MONOLITHIC, monoDesc);
    if (result != 0x6389)
    {
        System_printf ("Error Core %d : Software Info1 - Value %d is incorrect in MonoDesc : 0x%p\n", coreNum, result, monoDesc);
        errorCount++;
    }

    /* Set software info 2 */
    Cppi_setSoftwareInfo2 (Cppi_DescType_MONOLITHIC, monoDesc, 0x1FFFFFFF);

    result  = Cppi_getSoftwareInfo2 (Cppi_DescType_MONOLITHIC, monoDesc);
    if (result != 0x1FFFFFFF)
    {
        System_printf ("Error Core %d : Software Info2 - Value %d is incorrect in MonoDesc : 0x%p\n", coreNum, result, monoDesc);
        errorCount++;
    }

    /* Test return push policy */
    Cppi_setReturnPushPolicy (Cppi_DescType_MONOLITHIC, monoDesc, Qmss_Location_HEAD);

    if (Cppi_getReturnPushPolicy (Cppi_DescType_MONOLITHIC, monoDesc) != Qmss_Location_HEAD)
    {
        System_printf ("Error Core %d : Incorrect return push policy monoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    Cppi_setReturnPushPolicy (Cppi_DescType_MONOLITHIC, monoDesc, Qmss_Location_TAIL);

    if (Cppi_getReturnPushPolicy (Cppi_DescType_MONOLITHIC, monoDesc) != Qmss_Location_TAIL)
    {
        System_printf ("Error Core %d : Incorrect return push policy monoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    /* Test return queue */
    sQueue.qMgr = 1;
    sQueue.qNum = 3648;
    Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, monoDesc, sQueue);
    gQueue = Cppi_getReturnQueue (Cppi_DescType_MONOLITHIC, monoDesc);

    if (sQueue.qMgr != gQueue.qMgr || sQueue.qNum != gQueue.qNum)
    {
        System_printf ("Error Core %d : Incorrect return queue MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    /* Test PS data with optional fields */
    /* Fill in some data */
    dataBuff[0] = 0xAB;
    dataBuff[1] = 0xCD;
    dataBuff[2] = 0xEF;
    dataBuff[3] = 0xDC;

    /* Add PS data */
    Cppi_setPSData (Cppi_DescType_MONOLITHIC, monoDesc, (uint8_t *) dataBuff, 8);

    dataBuffPtr = 0;
    destLen = 0;
    /* Get PS data */
    if (Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, monoDesc, &dataBuffPtr, &destLen) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Getting PS data MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    for (i=0; i < destLen; i++)
    {
        if (dataBuff[i] != dataBuffPtr[i])
        {
            System_printf ("Error Core %d : In PS data Tx: %02X - Rx: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
            errorCount++;
            break;
        }
    }
    if (Cppi_getPSLen (Cppi_DescType_MONOLITHIC, monoDesc) != destLen)
    {
        System_printf ("Error Core %d : PS data Len MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    /* Test PS data without optional fields */
    monoDesc = (Cppi_Desc *) Qmss_queuePop (freeMonoQueHnd);

    /* Fill in some data */
    dataBuff[0] = 0xAB;
    dataBuff[1] = 0xCD;
    dataBuff[2] = 0xEF;
    dataBuff[3] = 0xDC;

    /* Add PS data */
    Cppi_setPSData (Cppi_DescType_MONOLITHIC, monoDesc, (uint8_t *) dataBuff, 8);

    dataBuffPtr = 0;
    destLen = 0;
    /* Get PS data */
    if (Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, monoDesc, &dataBuffPtr, &destLen) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Getting PS data MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    for (i=0; i < destLen; i++)
    {
        if (dataBuff[i] != dataBuffPtr[i])
        {
            System_printf ("Error Core %d : In PS data Tx: %02X - Rx: %02X \n", coreNum, dataBuff[i], dataBuffPtr[i]);
            errorCount++;
            break;
        }
    }
    if (Cppi_getPSLen (Cppi_DescType_MONOLITHIC, monoDesc) != destLen)
    {
        System_printf ("Error Core %d : PS data Len MonoDesc : 0x%p\n", coreNum, monoDesc);
        errorCount++;
    }

    return;
}

void testDescFunctions (Qmss_Result *hostReg_p, Qmss_Result *monoReg_p)
{
    /* Memory region configuration information */
    Qmss_MemRegInfo     hostMemInfo, monoMemInfo;
    /* Memory region configuration status */
    Qmss_MemRegCfg      memRegStatus;
    /* Descriptor configuration */
    Cppi_DescCfg        descCfg;

    Cppi_Result         result;
    uint32_t            i, numAllocated;

    System_printf ("**********Core %d TESTING DESCRIPTOR FUNCTIONS ************\n", coreNum);

    System_printf ("~~~~~~~~~~~~~~~Core %d Creating memory regions~~~~~~~~~~~~~\n", coreNum);

    memset ((void *) hostDesc, 0, (SIZE_HOST_DESC * NUM_HOST_DESC));
    memset ((void *) monolithicDesc, 0, (SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC));

    /* Setup memory region for host descriptors */
    memset((void *)&hostMemInfo, 0, sizeof(hostMemInfo));
    hostMemInfo.descBase = (uint32_t *) l2_global_address ((uint32_t)hostDesc);
    hostMemInfo.descSize = SIZE_HOST_DESC;
    hostMemInfo.descNum = NUM_HOST_DESC;
    hostMemInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    hostMemInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    hostMemInfo.startIndex = 0;
    
    /* Setup memory region for monolithic descriptors */
    memset((void *)&monoMemInfo, 0, sizeof(monoMemInfo));
    monoMemInfo.descBase = (uint32_t *) l2_global_address ((uint32_t)monolithicDesc);
    monoMemInfo.descSize = SIZE_MONOLITHIC_DESC;
    monoMemInfo.descNum = NUM_MONOLITHIC_DESC;
    monoMemInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    monoMemInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    monoMemInfo.startIndex = 0;
    
    /* For devices that require ordering, insert lowest address first */
    if (hostMemInfo.descBase < monoMemInfo.descBase)
    {
       /* host starts on lower address */
       *hostReg_p = Qmss_insertMemoryRegion (&hostMemInfo);
       *monoReg_p = Qmss_insertMemoryRegion (&monoMemInfo);
    } 
    else
    {
       /* Mono starts on lower address */
       *monoReg_p = Qmss_insertMemoryRegion (&monoMemInfo);
       *hostReg_p = Qmss_insertMemoryRegion (&hostMemInfo);
    }
    
    if (*hostReg_p < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, hostMemInfo.memRegion, *hostReg_p);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Memory region %d inserted\n", coreNum, *hostReg_p);
    }

    if (*monoReg_p < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, monoMemInfo.memRegion, *monoReg_p);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Memory region %d inserted\n", coreNum, *monoReg_p);
    }

    System_printf ("~~~~~~~~~~Core %d Memory regions creation passed~~~~~~~~~~~\n", coreNum);

    System_printf ("\n~~~~~~~~~~Core %d Memory region Configuration~~~~~~~~~~~~\n", coreNum);
    result = Qmss_getMemoryRegionCfg (&memRegStatus);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Getting memory region configuration info error code : %d\n", coreNum, result);
        errorCount++;
    }

    System_printf ("Current Desc count  : %d\n", memRegStatus.currDescCnt);
    for (i = 0; i < QMSS_MAX_MEM_REGIONS; i++)
    {
        if (memRegStatus.memRegInfo[i].descBase != 0)
        {
            System_printf ("\nMemory Region Index : %d\n", memRegStatus.memRegInfo[i].memRegion);
            System_printf ("Start Index         : %d\n", memRegStatus.memRegInfo[i].startIndex);
            System_printf ("Descriptor Size     : %d\n", memRegStatus.memRegInfo[i].descSize);
            System_printf ("Descriptor Num      : %d\n", memRegStatus.memRegInfo[i].descNum);
            System_printf ("Descriptor Base     : 0x%p\n", memRegStatus.memRegInfo[i].descBase);
            System_printf ("Managed Descriptor  : %d\n", memRegStatus.memRegInfo[i].manageDescFlag);
        }
    }

    System_printf ("\n~~~~~~~~~~~~~~~Core %d Initializing descriptors~~~~~~~~~~~~~~~\n", coreNum);

    /* Setup the descriptors */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum = NUM_HOST_DESC;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_HOST;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;
    /* Descriptor should be recycled back to freeQue allocated since destQueueNum is < 0 */
    descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnPushPolicy = Qmss_Location_TAIL;
    descCfg.cfg.host.returnPolicy = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
    descCfg.cfg.host.psLocation = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors and push to host free Queue */
    if ((freeHostQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing host descriptor error code: %d \n", coreNum, freeHostQueHnd);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Number of host descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);

    /* Setup the descriptors for receive free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION1;
    descCfg.descNum = NUM_MONOLITHIC_DESC;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifndef NSS_LITE
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.cfg.mono.dataOffset = 12;

    /* Initialize the descriptors and push to monolithic free Queue */
    if ((freeMonoQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing monolithic descriptor error code: %d \n", coreNum, freeMonoQueHnd);
        errorCount++;
        return;
    }
    else
        System_printf ("Core %d : Number of monolithic descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);

    /* Test configuring host descriptor fields */
    testHostDescConfig ();

    /* Test configuring monolithic descriptor fields */
    testMonoDescConfig ();
    return;
}

/***************************************************************************************
 * FUNCTION PURPOSE: Power up PA subsystem
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void passPowerUp (void)
{

#ifdef NSS_LITE
    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any 
     * PASS device register access. This not required for the simulator. */

    /* Set NSS Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_NSS);

    /* Enable the clocks for NSS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_NSS, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_NSS);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_NSS));
#endif  
  
}

/**
 *  @b Description
 *  @n
 *      Entry point for the test code.
 *      This is unit test code to test the following
 *
 *      It performs the following
 *          - Initializes the Resource Manager
 *          - Initializes the Queue Manager low level driver.
 *          - Initializes the CPPI low level driver.
 *          - Opens the SRIO CPDMA
 *          - Test Rx channel allocation and freeing
 *          - Test Tx channel allocation and freeing
 *          - Test Rx flow allocation and freeing
 *          - Test descriptor manipulation functions
 *          - Closes CPPI CPDMA
 *          - Deinitializes CPPI low level driver
 *
 *  @retval
 *      Not Applicable.
 */
#ifndef __LINUX_USER_SPACE
Void main (Void)
#else
extern uint32_t totalErrorCount;
void test_main (Cppi_Handle cppiHnd)
#endif
{
    Qmss_Result         hostReg, monoReg;
    uint32_t            temp;
    Cppi_Result         result;
#ifndef __LINUX_USER_SPACE
#if RM
    /* RM configuration */
    Rm_InitCfg          rmInitCfg;
    char                rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
    Rm_Handle           rmHandle;
    Rm_ServiceHandle   *rmServiceHandle;
    int32_t             rmResult;
    Cppi_StartCfg       cppiStartCfg;
#endif
    /* QMSS configuration */
    Qmss_InitCfg        qmssInitConfig;
    /* CPPI CPDMA configuration */
    Cppi_CpDmaInitCfg   cpdmaCfg;

    Cppi_Handle         cppiHnd;
    uint32_t            totalErrorCount = 0;
#ifdef __ARM_ARCH_7A__
    /* Add MMU entries for MMR's required for PCIE example */
    Uint32 privid, index;
    CSL_MsmcRegs *msmc = (CSL_MsmcRegs *)CSL_MSMC_CFG_REGS;
    Mmu_DescriptorAttrs attrs;
    extern char ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;
    uint32_t addr = (uint32_t)&ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;

    Mmu_initDescAttrs(&attrs);

    attrs.type = Mmu_DescriptorType_TABLE;
    attrs.shareable = 0;            // non-shareable
    attrs.accPerm = 1;              // read/write at any privelege level
    attrs.attrIndx = 0;             // Use MAIR0 Register Byte 3 for
                                    // determining the memory attributes
                                    // for each MMU entry


    // Update the first level table's MMU entry for 0x80000000 with the
    // new attributes.
    Mmu_setFirstLevelDesc((Ptr)0x40000000, (UInt64)addr, &attrs);

    // Set up SES & SMS to make all masters coherent
    for (privid = 0; privid < 16; privid++)
    {
      for (index = 0; index < 8; index++)
      {
        uint32_t ses_mpaxh = msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH;
        uint32_t sms_mpaxh = msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH;
        if (CSL_FEXT (ses_mpaxh, MSMC_SES_MPAXH_0_SEGSZ) != 0)
        {
          // Clear the "US" bit to make coherent.  This is at 0x80.
          ses_mpaxh &= ~0x80;
          msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH = ses_mpaxh;
        }
        if (CSL_FEXT (sms_mpaxh, MSMC_SMS_MPAXH_0_SEGSZ) != 0)
        {
          // Clear the "US" bit to make coherent.  This is at 0x80.
          sms_mpaxh &= ~0x80;
          msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH = sms_mpaxh;
        }
      }
    }
#endif

System_printf ("**************************************************\n");
    System_printf ("***************** CPPI LLD Testing ***************\n");
    System_printf ("**************************************************\n");
    
    passPowerUp();

#ifdef _TMS320C6X
    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Initialize the heap in shared memory for CPPI data structures */
    cppiHeapInit ();
#else
    coreNum = 0;
#endif

    System_printf ("*******Test running on Core %d *******************\n", coreNum);
#if RM
    /* Create the Server instance */
    memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
    rmInitCfg.instName = rmServerName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
        return;
    }

    rmServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Creating RM service handle error code : %d\n", coreNum, rmResult);
        return;
    }
#endif

    memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));
    memset ((Void *) &linkingRAM0, 0, sizeof (linkingRAM0));

    /* Set up the linking RAM. Use the internal Linking RAM for host descriptors and external
     * linking RAM for monolithic descriptors.
     * LLD will configure the internal linking RAM address and default size if a value of zero is specified.
     *
     */

#if defined(INTERNAL_LINKING_RAM) || defined(NSS_LITE) 
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
#else
    qmssInitConfig.linkingRAM0Base = l2_global_address ((uint32_t) linkingRAM0);
    qmssInitConfig.linkingRAM0Size = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;
#endif

#ifndef NSS_LITE
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#endif
#endif

#if RM
    qmssGblCfgParams.qmRmServiceHandle = rmServiceHandle;
#endif

    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", coreNum, result);
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", coreNum, result);
    }

    /* Initialize CPPI LLD */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error Core %d : Initializing CPPI LLD error code : %d\n", coreNum, result);
        return;
    }

#if RM
    cppiStartCfg.rmServiceHandle = rmServiceHandle;
    Cppi_startCfg(&cppiStartCfg);
#endif

    /* Set up QMSS CPDMA configuration */
    memset ((Void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

#ifndef NSS_LITE
    cpdmaCfg.dmaNum = Cppi_CpDma_QMSS_CPDMA;
    cpdmaCfg.writeFifoDepth = 32;
    cpdmaCfg.timeoutCount = 0x7F;
    cpdmaCfg.qm0BaseAddress = 0; /* take from cppi_device.c */
#else
    cpdmaCfg.dmaNum = Cppi_CpDma_NETCP_CPDMA;
#endif    

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        System_printf ("Error Core %d : Initializing QMSS(or NetCP) CPPI CPDMA %d\n", coreNum, cpdmaCfg.dmaNum);
        return;
    }
#endif

    /* Test CPDMA global configuration */
    testCpdmaGlobalConfig (cppiHnd);

    /* Test Rx Channels configuration */
    testRxChannelConfig (cppiHnd);

    if (errorCount == 0)
        System_printf ("\nCore %d : Receive Channel Configuration tests Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Receive Channel Configuration tests failed %d\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;
#ifdef RX_CH_ALLOCATION
    /* Test Rx Channels Allocation and free */
    testRxChannelAllocate (cppiHnd);

    if (errorCount == 0)
        System_printf ("\nCore %d : Receive Channel Allocation tests Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Receive Channel Allocation %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;
#endif
    /* Test Tx Channels configuration */
    testTxChannelConfig (cppiHnd);

    if (errorCount == 0)
        System_printf ("\nCore %d : Transmit Channel Configuration tests Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Transmit Channel Configuration %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;
#ifdef TX_CH_ALLOCATION
    /* Test Tx Channels Allocation and free */
    testTxChannelAllocate (cppiHnd);

    if (errorCount == 0)
        System_printf ("\nCore %d : Transmit Channel Allocation tests Passed %d\n", coreNum);
    else
        System_printf ("Error Core %d : Transmit Channel Allocation tests failed %d\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;
#endif
    /* Test Rx flow configuration */
    testRxFlowConfig (cppiHnd);
    if (errorCount == 0)
        System_printf ("\nCore %d : Receive Flow Configuration tests Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Receive Flow Configuration %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;
#ifdef RX_FLOW_ALLOCATION
    /* Test Rx Flows Allocation and free */
    testRxFlowAllocate (cppiHnd);

    if (errorCount == 0)
        System_printf ("\nCore %d : Receive Flow Allocation tests Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Receive Flow Allocation %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;
#endif
    /* Test CPPI descriptor functions */
    testDescFunctions(&hostReg, &monoReg);

    if (errorCount == 0)
        System_printf ("\nCore %d : CPPI Descriptor Functions tests Passed\n", coreNum);
    else
        System_printf ("Error Core %d : CPPI Descriptor Functions %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;

    /* Test data transfer using Host descriptor */
    testHostDataTransfer();

#ifdef _TMS320C6X
    System_flush();
#endif
    if (errorCount == 0)
        System_printf ("\nCore %d : Data transfer tests using host descriptor Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Data transfer tests using host descriptor %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;

    /* Test data transfer using Monolithic descriptor*/
    testMonoDataTransfer();

    if (errorCount == 0)
        System_printf ("\nCore %d : Data transfer tests using monolithic descriptor Passed\n", coreNum);
    else
        System_printf ("Error Core %d : Data transfer tests using monolithic descriptor %d tests failed\n", coreNum, errorCount);

    totalErrorCount += errorCount;
    errorCount = 0;

    /* Empty the queues */
    Qmss_queueEmpty (freeHostQueHnd);
    Qmss_queueEmpty (freeMonoQueHnd);

    /* Close the Host descriptor free queue */
    temp = freeHostQueHnd;
    if ((result = Qmss_queueClose (freeHostQueHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing host descriptor free queue error code: %d\n", coreNum, result);
    }
    else
        System_printf ("Core %d : Host descriptor free queue %d closed successfully. Ref count :%d\n",
                        coreNum, temp, result);

    /* Close the monolithic descriptor free queue */
    temp = freeMonoQueHnd;
    if ((result = Qmss_queueClose (freeMonoQueHnd)) < 0)
    {
        System_printf ("Error Core %d : Closing monolithic descriptor free queue error code: %d\n", coreNum, result);
    }
    else
        System_printf ("Core %d : Monolithic descriptor free queue %d closed successfully. Ref count :%d\n",
                        coreNum, temp, result);

    /* Close CPPI CPDMA instance */
    if ((result = Cppi_close (cppiHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing CPPI CPDMA error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : CPPI CPDMA closed successfully\n", coreNum);
    }

    /* Deinitialize CPPI LLD */
    if ((result = Cppi_exit ()) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Exiting CPPI error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : CPPI exit successful\n", coreNum);
    }

    /* Free the memory regions */
    if ((result = Qmss_removeMemoryRegion (monoReg, 0)) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Remove mono region error code : %d\n", coreNum, result);
    }

    if ((result = Qmss_removeMemoryRegion (hostReg, 0)) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Remove host region error code : %d\n", coreNum, result);
    }

    System_printf ("Core %d : exit QM\n", coreNum);
    if ( (result = Qmss_exit()) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Core %d : Error exiting QM: %d \n", coreNum, result);
    }
#if defined(RM) && !defined(__LINUX_USER_SPACE)
    if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
    {
        System_printf ("Error Core %d : Number of unfreed resources : %d\n", coreNum, rmResult);
        errorCount++;
        /* Print the resources to see the leak */
        Rm_resourceStatus(rmHandle, TRUE);
    }
    else
        System_printf ("Core %d : All resources freed successfully\n", coreNum);
#endif

    totalErrorCount += errorCount;
#ifndef __LINUX_USER_SPACE

    System_printf ("*******************************************************\n");
    System_printf ("***************** CPPI LLD Testing DONE ***************\n");
    System_printf ("*******************************************************\n");

    if (totalErrorCount == 0)
        System_printf ("\nCPPI LLD ALL TESTS PASSED\n");
    else
        System_printf ("\nCPPI LLD %d TESTS FAILED\n", totalErrorCount);

#endif
}
