/*
 * sample_int_reg.c
 *
 * Platform specific interrupt registration and un-registration routines.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
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

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>

#include <xdc/runtime/System.h>

#include "sample.h"
#include "tcp3d_drv_sample.h"

extern unsigned int ccXferCompInt[][EDMA3_MAX_REGIONS];
extern unsigned int ccErrorInt[];
extern unsigned int tcErrorInt[][EDMA3_MAX_TC];
extern unsigned int numEdma3Tc[];

#define DEBUG_PRINTS            0
#define MAP_ONCE_ONLY           1
#define ISR_APPROACH_WHILE      1   // 0 would make the code to hang

#if EDMA_LOCAL_COMP_ISR
#include <ti/csl/soc.h>
#include <ti/csl/cslr_tpcc.h>

//extern CSL_TpccRegs *tpcc2Regs;
extern unsigned int tpccRegionUsedLoc;

tccCallbackParams edma3IntrParamsLoc[64];
unsigned int allocatedTCCsLoc[2u] = {0x0u, 0x0u};

static void edma3ComplHandlerLoc (unsigned int edma3Id);
#endif

void (*ptrEdma3TcIsrHandler[EDMA3_MAX_TC])(unsigned int arg) =
                                                {
                                                &lisrEdma3TC0ErrHandler0,
                                                &lisrEdma3TC1ErrHandler0,
                                                &lisrEdma3TC2ErrHandler0,
                                                &lisrEdma3TC3ErrHandler0,
                                                &lisrEdma3TC4ErrHandler0,
                                                &lisrEdma3TC5ErrHandler0,
                                                &lisrEdma3TC6ErrHandler0,
                                                &lisrEdma3TC7ErrHandler0,
                                                };

unsigned int hwiInterrupt = 8;

unsigned int gemEvents[2];

/* Host interrupts for transfer completion (per spec intc_1.3.4.12.xlsx) */
/* First 4 cores are connected from CP_INTC0 and last 4 cores are connected from CP_INTC1 */
//unsigned int ccXferHostInt[NUM_EDMA3_INSTANCES][NUM_DSPS] = {
									    /*    CP_INTC0      |     CP_INTC1      */
#ifndef SOC_C6657
unsigned int ccXferHostInt[5][8] = {
										{68u, 78u, 88u, 98u, 68u, 78u, 88u, 98u},
										{69u, 79u, 89u, 99u, 69u, 79u, 89u, 99u},
										{70u, 80u, 90u, 100u, 70u, 80u, 90u, 100u},
										{71u, 81u, 91u, 101u, 71u, 81u, 91u, 101u},
										{72u, 82u, 92u, 102u, 72u, 82u, 92u, 102u},
										};
unsigned int edma3ErrHostInt[5][8] = {
										{73u, 83u, 93u, 103u, 73u, 83u, 93u, 103u},
										{64u, 74u, 84u, 94u, 64u, 74u, 84u, 94u},
										{65u, 75u, 85u, 95u, 65u, 75u, 85u, 95u},
										{66u, 76u, 86u, 96u, 66u, 76u, 86u, 96u},
										{67u, 77u, 87u, 97u, 67u, 77u, 87u, 97u},
										};

#else /* SOC_C6657 */
									    /*    only CP_INTC0 for C6657 */

unsigned int ccXferHostInt[5][8] = {
										{0u, 20u, 255u, 255u, 255u, 255u, 255u, 255u},
										{1u, 21u, 255u, 255u, 255u, 255u, 255u, 255u},
										{2u, 22u, 255u, 255u, 255u, 255u, 255u, 255u},
										{3u, 23u, 255u, 255u, 255u, 255u, 255u, 255u},
										{4u, 24u, 255u, 255u, 255u, 255u, 255u, 255u},
};
unsigned int edma3ErrHostInt[5][8] = {
										{6u, 26u, 255u, 255u, 255u, 255u, 255u, 255u},
										{7u, 27u, 255u, 255u, 255u, 255u, 255u, 255u},
										{5u, 25u, 255u, 255u, 255u, 255u, 255u, 255u},
										{8u, 28u, 255u, 255u, 255u, 255u, 255u, 255u},
										{9u, 29u, 255u, 255u, 255u, 255u, 255u, 255u},
										};
#endif

//extern unsigned int dsp_num;
//extern unsigned int tpccRegionUsed;
#if USE_LOCAL_CPINTC_DISPATCH
extern Void CpIntc_dispatchLoc(UInt hostInt);
#endif

/**  To Register the ISRs with the underlying OS, if required */
void registerEdma3Interrupts (  unsigned int edma3Id,
                                unsigned int tpccRegionUsed,
                                unsigned int dsp_num)
    {
    static UInt32 cookie = 0;
    Int eventId = 0;	/* GEM event id */
	unsigned int numTc = 0;
#if MAP_ONCE_ONLY
    static UInt32 mapDone = 0;
#endif
    unsigned int cpIntcNum = WHICH_CPINTC_NUM(dsp_num);//(dsp_num > 3)? 1: 0;

    /* Disabling the global interrupts */
    cookie = Hwi_disable();

	/* Transfer completion ISR */
	CpIntc_dispatchPlug(ccXferCompInt[edma3Id][tpccRegionUsed],
#if EDMA_LOCAL_COMP_ISR
                        edma3ComplHandlerLoc,
#else
                        lisrEdma3ComplHandler0,
#endif
                        edma3Id,
						TRUE);
#if MAP_ONCE_ONLY
	if (!mapDone)
#endif
    CpIntc_mapSysIntToHostInt(cpIntcNum, ccXferCompInt[edma3Id][tpccRegionUsed],
								ccXferHostInt[edma3Id][dsp_num]);
	CpIntc_enableHostInt(cpIntcNum, ccXferHostInt[edma3Id][dsp_num]);
    eventId = CpIntc_getEventId(ccXferHostInt[edma3Id][dsp_num]);
    EventCombiner_dispatchPlug (eventId,
#if USE_LOCAL_CPINTC_DISPATCH
                                CpIntc_dispatchLoc,
#else
                                CpIntc_dispatch,
#endif
                                ccXferHostInt[edma3Id][dsp_num],
                                TRUE);
#if DEBUG_PRINTS
    System_printf("\t\t ccXferCompInt : %d \n", ccXferCompInt[edma3Id][tpccRegionUsed]);
    System_printf("\t\t ccXferHostInt : %d \n", ccXferHostInt[edma3Id][dsp_num]);
    System_printf("\t\t eventId : %d \n", eventId);
#endif
    gemEvents[0] = eventId;

	/* CC Error ISR */
	CpIntc_dispatchPlug(ccErrorInt[edma3Id], lisrEdma3CCErrHandler0,
						edma3Id, TRUE);
#if MAP_ONCE_ONLY
    if (!mapDone)
#endif
	CpIntc_mapSysIntToHostInt(cpIntcNum, ccErrorInt[edma3Id],
								edma3ErrHostInt[edma3Id][dsp_num]);
#if DEBUG_PRINTS
    System_printf("\t\t ccErrorInt : %d \n", ccErrorInt[edma3Id]);
    System_printf("\t\t edma3ErrHostInt : %d \n", edma3ErrHostInt[edma3Id][dsp_num]);
#endif

	/* TC Error ISR */
    while (numTc < numEdma3Tc[edma3Id])
	    {
		CpIntc_dispatchPlug(tcErrorInt[edma3Id][numTc],
							(CpIntc_FuncPtr )(ptrEdma3TcIsrHandler[numTc]),
							edma3Id, TRUE);
#if MAP_ONCE_ONLY
    if (!mapDone)
#endif
		CpIntc_mapSysIntToHostInt(cpIntcNum, tcErrorInt[edma3Id][numTc],
									edma3ErrHostInt[edma3Id][dsp_num]);
#if DEBUG_PRINTS
    System_printf("\t\t tcErrorInt : %d \n", tcErrorInt[edma3Id][numTc]);
    System_printf("\t\t edma3ErrHostInt : %d \n", edma3ErrHostInt[edma3Id][dsp_num]);
#endif
        numTc++;
    	}
	/* Enable the host interrupt which is common for both CC and TC error */
	CpIntc_enableHostInt(cpIntcNum, edma3ErrHostInt[edma3Id][dsp_num]);
    eventId = CpIntc_getEventId(edma3ErrHostInt[edma3Id][dsp_num]);
    EventCombiner_dispatchPlug (eventId,
#if USE_LOCAL_CPINTC_DISPATCH
                                CpIntc_dispatchLoc,
#else
                                CpIntc_dispatch,
#endif
                                edma3ErrHostInt[edma3Id][dsp_num],
                                TRUE);
#if DEBUG_PRINTS
    System_printf("\t\t eventId : %d \n", eventId);
#endif
    gemEvents[1] = eventId;

    //Hwi_enableInterrupt(hwiInterrupt);

    /* enable the 'global' switch */
    CpIntc_enableAllHostInts(cpIntcNum);

#if EDMA_LOCAL_COMP_ISR
    tpccRegionUsedLoc = tpccRegionUsed;
#endif

#if MAP_ONCE_ONLY
    mapDone = 1;
#endif

    /* Restore interrupts */
    Hwi_restore(cookie);
    }

/**  To Unregister the ISRs with the underlying OS, if previously registered. */
void unregisterEdma3Interrupts (unsigned int edma3Id, unsigned int dsp_num)
    {
    static UInt32 cookie = 0;
    Int eventId = 0;	/* GEM event id */
//    unsigned int numTc = 0;
    unsigned int cpIntcNum = WHICH_CPINTC_NUM(dsp_num);//(dsp_num > 3)? 1: 0;

    /* Disabling the global interrupts */
    cookie = Hwi_disable();

	/* Transfer completion ISR */
	CpIntc_disableHostInt(cpIntcNum, ccXferHostInt[edma3Id][dsp_num]);
    eventId = CpIntc_getEventId(ccXferHostInt[edma3Id][dsp_num]);
	EventCombiner_disableEvent(eventId);

	/* CC/TC Error ISR */
	CpIntc_disableHostInt(cpIntcNum, edma3ErrHostInt[edma3Id][dsp_num]);
    eventId = CpIntc_getEventId(edma3ErrHostInt[edma3Id][dsp_num]);
	EventCombiner_disableEvent(eventId);

    /**
     * Clear all system interrupt to host interrupt mapping.
     * - might not be needed
     * - doing to get clean numbers from cpintc dispatcher for debugging
     * - DID NOT HELP, so commenting for now
     */
//    CpIntc_mapSysIntToHostInt(cpIntcNum, ccXferCompInt[edma3Id][tpccRegionUsedLoc], 0);
//    CpIntc_mapSysIntToHostInt(cpIntcNum, ccErrorInt[edma3Id], 0);
//    while (numTc < numEdma3Tc[edma3Id])
//        {
//        CpIntc_mapSysIntToHostInt(cpIntcNum, tcErrorInt[edma3Id][numTc], 0);
//        numTc++;
//        }

    /* Restore interrupts */
    Hwi_restore(cookie);
    }

#if EDMA_LOCAL_COMP_ISR
/**
 * edma3ComplHandler
 * \brief   Interrupt handler for successful transfer completion.
 *
 * \note    This function first disables its own interrupt to make it non-
 *          entrant. Later, after calling all the callback functions, it
 *          re-enables its own interrupt.
 *
 * \return  None.
 */
UInt32 tpccIsrCntr = 0;
UInt32 tpccCbCntr = 0;
static void edma3ComplHandlerLoc (unsigned int edma3Id)
    {
#if !ISR_APPROACH_WHILE
    unsigned int Cnt;
#endif
    volatile CSL_TPCC_ShadowRegs *shadowRegs = NULL;
    volatile unsigned int pendingIrqs;
    unsigned int indexl;
    unsigned int indexh;
    CSL_TpccRegs *tpcc2Regs = (CSL_TpccRegs *) CSL_EDMACC_2_REGS;

    tpccIsrCntr++;

    if (tpcc2Regs != NULL)
        {
        shadowRegs = (volatile CSL_TPCC_ShadowRegs *)
                                    (&tpcc2Regs->SHADOW[tpccRegionUsedLoc]);
        }

#if !ISR_APPROACH_WHILE
    Cnt = 0u;
#endif
    pendingIrqs = 0u;
    indexl = 1u;
    indexh = 1u;

#if ISR_APPROACH_WHILE
    while((shadowRegs->TPCC_IPR !=0 ) || (shadowRegs->TPCC_IPRH !=0 ))
        {
        /* Loop for EDMA3_RM_COMPL_HANDLER_RETRY_COUNT number of time,
           breaks when no pending interrupt is found */
            indexl = 0u;
            pendingIrqs = shadowRegs->TPCC_IPR;

            /**
             * Choose interrupts coming from our allocated TCCs
             * and MASK remaining ones.
             */
            pendingIrqs = (pendingIrqs & allocatedTCCsLoc[0u]);

            while (pendingIrqs)
                {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == TRUE)
                    {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPR register will NOT be cleared.
                     */
                    if(edma3IntrParamsLoc[indexl].tccCb != NULL)
                        {
                         /* here write to ICR to clear the corresponding IPR bits*/
                        shadowRegs->TPCC_ICR = (1u << indexl);

                                tpccCbCntr++;
                                
                        edma3IntrParamsLoc[indexl].tccCb (indexl,
                                    EDMA3_RM_XFER_COMPLETE,
                                    edma3IntrParamsLoc[indexl].cbData);
                        }
                    }
                ++indexl;
                pendingIrqs >>= 1u;
                }

            indexh = 0u;
            pendingIrqs = shadowRegs->TPCC_IPRH;

            /**
             * Choose interrupts coming from our allocated TCCs
             * and MASK remaining ones.
             */
            pendingIrqs = (pendingIrqs & allocatedTCCsLoc[1u]);

            while (pendingIrqs)
                {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                    {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPRH register will NOT be cleared.
                     */
                    if(edma3IntrParamsLoc[32u+indexh].tccCb!=NULL)
                        {
                         /* here write to ICR to clear the corresponding IPR bits*/
                        shadowRegs->TPCC_ICRH = (1u << indexh);

                        edma3IntrParamsLoc[32u+indexh].tccCb(32u+indexh,
                                    EDMA3_RM_XFER_COMPLETE,
                                    edma3IntrParamsLoc[32u+indexh].cbData);
                        }
                    }
                ++indexh;
                pendingIrqs >>= 1u;
                }
        }
#else // ISR_APPROACH_WHILE
    if((shadowRegs->TPCC_IPR !=0 ) || (shadowRegs->TPCC_IPRH !=0 ))
        {
        /**
         * Since an interrupt has found, we have to make sure that this
         * interrupt (TCC) belongs to the TCCs allocated by us only.
         * It might happen that someone else, who is using EDMA3 also,
         * is the owner of this interrupt channel i.e. the TCC.
         * For this, use the allocatedTCCs[], to check which all interrupt
         * channels are owned by the EDMA3 RM Instances.
         */

        edma3OsProtectEntry (edma3Id,
                            EDMA3_OS_PROTECT_INTERRUPT_XFER_COMPLETION,
                            NULL);

        /* Loop for EDMA3_RM_COMPL_HANDLER_RETRY_COUNT number of time,
           breaks when no pending interrupt is found */
        while ((Cnt < 10u)
                    && ((indexl != 0u) || (indexh != 0u)))
            {
            indexl = 0u;
            pendingIrqs = shadowRegs->TPCC_IPR;

            /**
             * Choose interrupts coming from our allocated TCCs
             * and MASK remaining ones.
             */
            pendingIrqs = (pendingIrqs & allocatedTCCsLoc[0u]);

            while (pendingIrqs)
                {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == TRUE)
                    {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPR register will NOT be cleared.
                     */
                    if(edma3IntrParamsLoc[indexl].tccCb != NULL)
                        {
                         /* here write to ICR to clear the corresponding IPR bits*/
                        shadowRegs->TPCC_ICR = (1u << indexl);

                                tpccCbCntr++;
                                
                        edma3IntrParamsLoc[indexl].tccCb (indexl,
                                    EDMA3_RM_XFER_COMPLETE,
                                    edma3IntrParamsLoc[indexl].cbData);
                        }
                    }
                ++indexl;
                pendingIrqs >>= 1u;
                }

            indexh = 0u;
            pendingIrqs = shadowRegs->TPCC_IPRH;

            /**
             * Choose interrupts coming from our allocated TCCs
             * and MASK remaining ones.
             */
            pendingIrqs = (pendingIrqs & allocatedTCCsLoc[1u]);

            while (pendingIrqs)
                {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                    {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPRH register will NOT be cleared.
                     */
                    if(edma3IntrParamsLoc[32u+indexh].tccCb!=NULL)
                        {
                         /* here write to ICR to clear the corresponding IPR bits*/
                        shadowRegs->TPCC_ICRH = (1u << indexh);

                        edma3IntrParamsLoc[32u+indexh].tccCb(32u+indexh,
                                    EDMA3_RM_XFER_COMPLETE,
                                    edma3IntrParamsLoc[32u+indexh].cbData);
                        }
                    }
                ++indexh;
                pendingIrqs >>= 1u;
                }

            Cnt++;
            }

        indexl = (shadowRegs->TPCC_IPR & allocatedTCCsLoc[0u]);
        indexh = (shadowRegs->TPCC_IPRH & allocatedTCCsLoc[1u]);

        if((indexl !=0 ) || (indexh !=0 ))
            {
            shadowRegs->TPCC_IEVAL=0x1u;
            }

        edma3OsProtectExit (edma3Id,
                            EDMA3_OS_PROTECT_INTERRUPT_XFER_COMPLETION,
                            NULL);
        }
        /* for testing only */
        else
        {
            while(1);
        }
#endif // ISR_APPROACH_WHILE
    }
#endif // EDMA_LOCAL_COMP_ISR
