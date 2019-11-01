/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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

/*  
 *  This file contains the interrupt service routines and the
 *  code to install them.
 */
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/drv/hyplnk/hyplnk.h>
#include <csl_intc.h>
#include <csl_intcAux.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hyplnkIsr.h"
#include "hyplnkPlatCfg.h"
#include <c6x.h>

#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(DEVICE_K2E) || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)
#include <ti/csl/csl_device_interrupt.h>
#endif /* DEVICE_K2K || DEVICE_K2H || DEVICE_K2E || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E) */

#ifdef CSL_INTC0_VUSR_INT_O
#define CSL_CIC0_HYPERLINK_0_INT    CSL_INTC0_VUSR_INT_O
#endif 

CSL_IntcEventHandlerRecord  hyplnkExampleEvtHdlrRecord[2];
CSL_IntcObj                 hyplnkExampleIntcObj;
CSL_IntcHandle              hyplnkExampleIntcHnd;
CSL_IntcContext             hyplnkExampleIntcContext;

volatile int hyplnkExampleFatalReason = -1;
/*****************************************************************************
 * Trap/infinite loop entered when a fatal error is detected in the ISR
 * printf() and exit() are not reliable from an ISR.
 ****************************************************************************/
void hyplnkExampleFatal (int reason) {
  hyplnkExampleFatalReason = reason;
  while(1);
}

/*****************************************************************************
 * This attempts to pretty print the fatal error.  No guarantees it will
 * work from ISR context.
 ****************************************************************************/
void hyplnkExampleCheckOneStat (hyplnkLocation_e  location,
                                const char       *name,
                                int               noWarn);

/*****************************************************************************
 * Hyperlink ISR handler
 ****************************************************************************/
void hyplnkExampleIsr (void *eventId)
{
  hyplnkRegisters_t       regs;
  hyplnkIntStatusClrReg_t intStatusClr;
  hyplnkIntPendSetReg_t   intPendSet;
  Hyplnk_Handle           handle = NULL;
  uint32_t                remainingInterrupts;
  CSL_CPINTC_Handle       cpintcHnd;

  cpintcHnd = CSL_CPINTC_open (0);
  if (! cpintcHnd) {
    hyplnkExampleFatal(0);
  }

  memset (&regs, 0, sizeof(regs));
  regs.intStatusClr = &intStatusClr;

  if (Hyplnk_open(0, &handle) != hyplnk_RET_OK) {
    hyplnkExampleFatal(1);
  }

  /* Figure out which interrupt it is */
  if (Hyplnk_readRegs (handle, hyplnk_LOCATION_LOCAL, &regs) != hyplnk_RET_OK) {
    hyplnkExampleFatal(2);
  }

  /* Process the fatal error interrupt */
  remainingInterrupts = intStatusClr.intClr;
  if (remainingInterrupts & (1 << hyplnk_EXAMPLE_ISRNUM_FATAL)) {
    remainingInterrupts &= ~(1 << hyplnk_EXAMPLE_ISRNUM_FATAL);
    printf ("Fatal error detected\n");
    hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "fatal isr", 1);
    hyplnkExampleFatal(3);
  }

  /* Placeholder to process other interrupts that happened at the same time */
  if (remainingInterrupts) {
    printf ("Unknown interrupts: 0x%08x\n", remainingInterrupts);
  }

  /* Clear the interrupt */
  if (Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &regs) != hyplnk_RET_OK) {
    hyplnkExampleFatal(4);
  }

  /* Acknowledge CorePac interrupt */
  CSL_intcHwControl(hyplnkExampleIntcHnd, CSL_INTC_CMD_EVTCLEAR, NULL);
  /* Acknowledge cp intc interrupt */
  CSL_CPINTC_clearSysInterrupt (cpintcHnd, CSL_CIC0_HYPERLINK_0_INT);

  /* Retrigger any remaining interrupts */
  intPendSet.intSet = 0;
  regs.intStatusClr = NULL;
  regs.intPendSet   = &intPendSet;
  if (Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &regs) != hyplnk_RET_OK) {
    hyplnkExampleFatal(4);
  }

  if (Hyplnk_close (&handle) != hyplnk_RET_OK) {
    hyplnkExampleFatal(5);
  }

}

/*****************************************************************************
 * Set up the INTC that is in the CorePac.  This works the same way on
 * any c64x or c66x
 ****************************************************************************/
static int hyplnkExampleInitCoreIntc (void)
{
  CSL_IntcGlobalEnableState   state;

  /* INTC module initialization */
  hyplnkExampleIntcContext.eventhandlerRecord = hyplnkExampleEvtHdlrRecord;
  hyplnkExampleIntcContext.numEvtEntries      = 2;
  if (CSL_intcInit (&hyplnkExampleIntcContext) != CSL_SOK) 
    return -1;

  /* Enable NMIs */
  if (CSL_intcGlobalNmiEnable () != CSL_SOK) 
    return -1;
 
  /* Enable global interrupts */
  if (CSL_intcGlobalEnable (&state) != CSL_SOK) 
    return -1;

  /* INTC has been initialized successfully. */
  return 0;
}

/*****************************************************************************
 * Set up the INTC that is at the chip level.  This varies over the various
 * c66x devices.
 ****************************************************************************/
static int hyplnkExampleInitChipIntc (void)
{
  CSL_CPINTC_Handle hnd;

  hnd = CSL_CPINTC_open (0);
  if (! hnd) {
    return -1;
  }
  /* Disable all host interrupts. */
  CSL_CPINTC_disableAllHostInterrupt(hnd);
    
  /* Configure no nesting support in the CPINTC Module. */
  CSL_CPINTC_setNestingMode (hnd, CPINTC_NO_NESTING);

  /* Clear CSL_INTC0_VUSR_INT_O */
  CSL_CPINTC_clearSysInterrupt (hnd, CSL_CIC0_HYPERLINK_0_INT);
  /* Enable it */
  CSL_CPINTC_enableSysInterrupt (hnd, CSL_CIC0_HYPERLINK_0_INT);
  CSL_CPINTC_mapSystemIntrToChannel (hnd, CSL_CIC0_HYPERLINK_0_INT, hyplnk_EXAMPLE_INTC_OUTPUT);
  /* Do not need to use CSL_CPINTC_mapChannelToHostInterrupt because the mapping is static
   * such that channel and host interrupt are the same 
   */

  /* Enable it */
  CSL_CPINTC_enableHostInterrupt (hnd, hyplnk_EXAMPLE_INTC_OUTPUT);

  CSL_CPINTC_enableAllHostInterrupt(hnd);

  return 0;
}

/*****************************************************************************
 * Install the interrupt service routine
 ****************************************************************************/
int hyplnkExampleInitVec() 
{
  CSL_IntcParam vectId = hyplnk_EXAMPLE_COREPAC_VEC;
  Int16         eventId = hyplnk_EXAMPLE_COREPAC_INT_INPUT;

  hyplnkExampleIntcHnd = CSL_intcOpen (&hyplnkExampleIntcObj, eventId, &vectId, NULL);
  if (! hyplnkExampleIntcHnd) {
    return 0;
  }

  hyplnkExampleEvtHdlrRecord[0].handler = hyplnkExampleIsr;
  hyplnkExampleEvtHdlrRecord[0].arg     = (void *)eventId;
  CSL_intcPlugEventHandler(hyplnkExampleIntcHnd, hyplnkExampleEvtHdlrRecord);
  CSL_intcHwControl(hyplnkExampleIntcHnd, CSL_INTC_CMD_EVTCLEAR, NULL);
  CSL_intcHwControl(hyplnkExampleIntcHnd, CSL_INTC_CMD_EVTENABLE, NULL);
  return 0;
}

/*****************************************************************************
 * Set up an ISR handler for the hyperlink error event
 ****************************************************************************/
void hyplnkExampleInstallIsr (void)
{
  /* Setup up the core-specific INTC */
  if (hyplnkExampleInitCoreIntc()) {
    printf ("Failed to set up CorePac INTC\n");
    exit(1);
  }

  /* Connect hyperlink's common interrupt to a CorePac event */
  if (hyplnkExampleInitChipIntc()) {
    printf ("Failed to set up Chip INTC\n");
    exit(1);
  }

  /* Connect the CorePac event to a vector */
  if (hyplnkExampleInitVec()) {
    printf ("Failed to set up interrupt vector\n");
    exit(1);
  }
}

/* Nothing past this point */

