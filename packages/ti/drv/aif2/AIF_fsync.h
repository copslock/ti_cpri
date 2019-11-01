/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
                    
/** 
 * @file AIF_fsync.h
 *
 * @brief Header file for FSYNC setup hardware interrupts. These functions can be
 * used with or without OS.
 * 
*/


/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */


#ifndef __AIF_FSYNC_H
#define __AIF_FSYNC_H

//#include <csl.h>
//#include <csl_srio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b AIF_initFsync
 *
 *   @b Description
 *   @n This function initializes the Fsync event variables used by AIF2 driver software.
 *      It is called by AIF_startHw().
 *
 *   @b Arguments
 *   @verbatim
 *   None
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies    
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_initFsync(hAif);
     @endverbatim
 * 
 */

#ifndef __AIF_FSYNC_C
extern
#endif
void 
AIF_initFsync(
);

/** 
 *   @n@b AIF_fsync1Event7Count
 *
 *   @b Description
 *   @n This function counts occurrences of AT event 7. This is the 10ms tick.
 *      User application can then use aifFsyncEventCount[1] to pace its actions toward AIF2.
 *   @b Arguments
 *   @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies    
 *   @n  aifFsyncEventCount array
 *
 *   @b Example 1
 *   @verbatim
        //Caution: if BIOS interrupt dispatcher is used, you must remove the "interrupt" keyword before the ISR
		interrupt void
		Aif2_RadT_Sevt7_FSEVT1_ISR()
		{
			AIF2_fsync1Event7Count();
			fsevt1_userIsr();
		}
     @endverbatim
 *     
 *   @b Example 2
 *   @verbatim
        // for instance, wait 10ms for AIF RAM to be flushed
        while (aifFsyncEventCount[1] <= 2) {
              asm("    IDLE");
        }     
 * 
 */
#ifndef __AIF_FSYNC_C
extern
#endif
void 
AIF_fsync1Event7Count(
);

/**
 *   @n@b AIF_fsyncEvent2_7Count
 *
 *   @b Description
 *   @n This function counts occurrences of AT event 0 to 5.
 *      The ABT (one of the AIF2 lib mode) mechanism uses those events to trigger incoming EDMA channels
 *      This function, in that case, is only used for debug purpose to check 16-chip tick for each of the links
 *      User application can then use aifFsyncEventCount[2] to aifFsyncEventCount[7].
 *   @b Arguments
 *   @verbatim
        interruptFlags   Contains a copy of MEVTFLAG[combinedEvent]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n  aifFsyncEventCount array
 *
 *   @b Example
 *   @verbatim
        //Caution: if BIOS interrupt dispatcher is used, you must remove the "interrupt" keyword before the ISR
		interrupt void
		Aif2_RadT_Sevt0_5_FSEVT2_7_ISR()
		{
			volatile uint32_t intFlag= IntcRegs->MEVTFLAG[3];

			while(intFlag)
			{
				IntcRegs->EVTCLR[3]= intFlag;
				AIF_fsyncEvent2_7Count(intFlag);
				//make sure all pending events are handled before return
				intFlag= IntcRegs->MEVTFLAG[3];
			}
		}
     @endverbatim
 *
 */
#ifndef __AIF_FSYNC_C
extern
#endif
void 
AIF_fsyncEvent2_7Count(
    uint32_t  interruptFlag
);

/** 
 *   @n@b AIF2_dioEngineIsr
 *
 *   @b Description
 *   @n This function counts occurrences of AT event 6 for each of the links.
 *      User application can use aif2DioIntCount[] to check on enabled and CPU-visible AT events.
 *
 *   @b Arguments
 *   @verbatim
        hAif   Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies    
 *   @n  aif2DioIntCount array
 *
 *   @b Example
 *   @verbatim
        //Caution: if BIOS interrupt dispatcher is used, you must remove the "interrupt" keyword before the ISR
		// Used by ABT mechanism, for instance, to implement ping-pong on the TX direction
		interrupt void
		Aif2_RadT_Sevt6_ISR()
		{
		   AIF2_dioEngineIsr(hConfigAif);
		}
     @endverbatim
 * 
 */
#ifndef __AIF_FSYNC_C
extern
#endif
void 
AIF_dioEngineIsr(
    AIF_ConfigHandle    hAif
);

#ifdef __cplusplus
}
#endif


#endif //__AIF_FSYNC_H

/** @} */ // end of module additions
