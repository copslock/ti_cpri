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
 * @file AIF_hibernation.h
 *
 * @brief Header file for AIF routines to save and restore the state of Antenna
 *  Interface (AIF2) for hibernation purposes.  There is also a function to
 *  be called during initialization of Aif2 to keep a pointer to the initial
 *  Aif2Fl_Setup structure. This structure contains all parameters needed to
 *  restore Aif2 state.
 *
*/


/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */

#ifndef AIF_HIBERNATE_H_
#define AIF_HIBERNATE_H_

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>


/**
 * @brief This structure defines the AIF data items that are used by the application.
 * This tells the AIF save/restore functions where the Application main CSL initialization
 * structure is located.
 */
typedef struct
{
	Aif2Fl_Obj        *save_aif2_object_pointer;
	Aif2Fl_Setup      *save_aif2_struct_pointer;
} AIF_HwContextSaveInfo;

/**
 * @brief This structure defines the global (DDR) storage structure that will preserve the
 * state of AIF while the chip is asleep.
 */
typedef struct
{
	AIF_HwContextSaveInfo info;

} AIF_HwContextSaveData;


/**
 *   @n@b AIF_hwContextSaveInit
 *
 *   @b Description
 *   @n This function initializes the pointer to CSL structure required to save AIF configuration.
 *   Must be called prior to any prior to AIF_hwContextSave().
 *
 *   @b Arguments
 *   @verbatim
        hSaveInfo   Pointer to a AIF_HibernateSaveInfo instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n  None.
 *
 *   @b Modifies
 *   @n  H/W context save parameters
 *
 *   @b Example
 *   @verbatim
        AIF_HwContextSaveInfo saveInfo;
            AIF_hwContextSaveInit(&saveInfo);
     @endverbatim
 *
 */
#ifndef __AIF_HIBERNATE_C
extern
#endif
void
AIF_hwContextSaveInit(
		AIF_HwContextSaveInfo *hSaveInfo
);

/**
 *   @n@b AIF_hwContextSave
 *
 *   @b Description
 *   @n This function causes AIF to be set to bypass prior to hibernation.
 *   AIF2 Bypass just means disabling DIO for WCDMA and disabling PktDMA channel for LTE to
 *   cut real incoming frame data and re-transmit it to the next DSP in the chain
 *
 *   @b Arguments
 *   @verbatim
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  AIF_hwContextSaveInit() must have been called prior to this function
 *
 *   <b> Post Condition </b>
 *   @n  None.
 *
 *   @b Modifies
 *   @n  H/W set to bypass
 *
 *   @b Example
 *   @verbatim
        AIF_HwContextSaveInfo saveInfo;
            AIF_hwContextSaveInit(&saveInfo);
            AIF_hwContextSave();
     @endverbatim
 *
 */
#ifndef __AIF_HIBERNATE_C
extern
#endif
void AIF_hwContextSave(
);

/**
 *   @n@b AIF_hwContextRestore
 *
 *   @b Description
 *   @n This function causes the entire state of Aif2 to be restored following.
 *   hibernation. In non-reset isolation mode, AIF physical and radio timers
 *   (radt, ulradt) init frame values are being set based on currentBfn function parameter.
 *   This function supports the AIF2 reset isolation and non-reset isolation modes.
 *
 *   @b Arguments
 *   @verbatim
		currentBfn	Init value for radio timers (radt, ulradt) init frame values
		aif2_rsiso  Specify whether AIF2 is in reset isolation mode
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  AIF_hwContextSaveInit() must have been called prior to entering hibernation.
 *
 *   <b> Post Condition </b>
 *   @n  None.
 *
 *   @b Modifies
 *   @n  H/W restarted
 *
 *   @b Example
 *   @verbatim
        AIF_HwContextSaveInfo saveInfo;
            AIF_hwContextSaveInit(&saveInfo);
            AIF_hwContextSave();
            ... hibernate
            ... wake up
            ... get current Bfn
            AIF_hwContextRestore(currentBfn);
     @endverbatim
 *
 */
#ifndef __AIF_HIBERNATE_C
extern
#endif
void AIF_hwContextRestore(
		uint32_t currentBfn,
		uint32_t aif2_rsiso
);



#endif /* AIF_HIBERNATE_H_ */

/** @} */ // end of module additions

