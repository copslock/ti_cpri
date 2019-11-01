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
 * @file AIF_init.h
 *
 * @brief Header file for AIF H/W initialization
 * 
*/


/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */


#ifndef __AIF_INIT_H
#define __AIF_INIT_H

#include <ti/csl/csl_edma3.h>
//#include <csl.h>
//#include <csl_srio.h>

#ifdef __cplusplus
extern "C" {
#endif
/** 
 *   @n@b AIF_initDio
 *
 *   @b Description
 *   @n This function sets up the dioConfig[] structure given the user parameters.
 *
 *   @b Arguments
 *   @verbatim
		hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_calcParameters()
 *   
 *   <b> Post Condition </b>
 *   @n  AIF2_ConfigObj instance has valid parameters for DIO engines
 *
 *   @b Modifies    
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_initDio(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_INIT_C
extern
#endif
void 
AIF_initDio(
	AIF_ConfigHandle    hAif
);

/** 
 *   @n@b AIF_initPktDma
 *
 *   @b Description
 *   @n This function sets up the pktDmaConfig structure given the user parameters. Tx and Rx Hw (free) queues
 *   are initialized, as well as AIF2 PKTDMA channels. It should be noted that for LTE, there is an option to not
 *   use this function and configure queues, descriptors, pktdma channels and rx flows outside of AIF2LLD.
 *
 *   @b Arguments
 *   @verbatim
		hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_calcParameters()
 *   
 *   <b> Post Condition </b>
 *   @n  AIF2_ConfigObj instance has valid parameters for PFTDMA channels
 *
 *   @b Modifies    
 *   @n
 *
 *   @b Example
 *   @verbatim
            AIF_initPktDma(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_INIT_C
extern
#endif
int32_t
AIF_initPktDma(
	AIF_ConfigHandle    hAif
);

/** 
 *   @n@b AIF_initHw
 *
 *   @b Description
 *   @n This function sets up the CSL structures for the 6 AIF links.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_initHw(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_initHw(
	AIF_ConfigHandle hAif
);

/** 
 *   @n@b AIF_startHw
 *
 *   @b Description
 *   @n This function configures AIF2 HW registers for the 6 AIF link
 *      given user AIF2 configuration. HW is then started, waiting for
 *      the selected synchronization.
 *
 *   @b Arguments
 *   @verbatim
       hAif        Pointer to a AIF2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  The AIF links enabled by the user application are configured.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            AIF_ConfigHandle    hAif;
            AIF_startHw(hAif);
     @endverbatim
 * 
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_startHw(
	AIF_ConfigHandle hAif
);


/**
 *   @n@b AIF_configureAtEvent
 *
 *   @b Description
 *   @n This function is used prior to AIF_startHw to configure application specific AT events.
 *
 *   @b Arguments
 *   @verbatim
       hAtEvent        Pointer to a Aif2Fl_AtEvent instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AtEvent* hAtEvent
            AIF_configureAtEvent(hAtEvent);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_configureAtEvent(
		Aif2Fl_AtEvent* hAtEvent
);


/**
 *   @n@b AIF_enableAtEvent
 *
 *   @b Description
 *   @n This function is used to enable AT event.
 *
 *   @b Arguments
 *   @verbatim
       event        event of interest in Aif2Fl_AtEventIndex.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AtEventIndex event
            AIF_enableAtEvent(event);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_enableAtEvent(
	Aif2Fl_AtEventIndex event
);

/**
 *   @n@b AIF_disableAtEvent
 *
 *   @b Description
 *   @n This function is used to disable AT event.
 *
 *   @b Arguments
 *   @verbatim
       event        event of interest in Aif2Fl_AtEventIndex.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AtEventIndex event
            AIF_disbleAtEvent(event);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_disableAtEvent(
	Aif2Fl_AtEventIndex event
);


/**
 *   @n@b AIF_configureEgrDioEvent
 *
 *   @b Description
 *   @n This function is used prior to AIF_startHw to override the default parameter of a DIO engine egress event.
 *
 *   @b Arguments
 *   @verbatim
       hAtEvent        Pointer to a Aif2Fl_AtEvent instance.
       dioNum          Egress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AtEvent* hAtEvent
            uint32_t           dioNum
            AIF_configureEgrDioEvent(hAtEvent, dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_configureEgrDioEvent(
		Aif2Fl_AtEvent* hAtEvent,
		uint32_t           dioNum
);


/**
 *   @n@b AIF_enableEgrDioEvent
 *
 *   @b Description
 *   @n This function is used to enable a DIO egress engine event.
 *
 *   @b Arguments
 *   @verbatim
       dioNum        Egress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            uint32_t           dioNum
            AIF_enableEgrDioEvent(dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void AIF_enableEgrDioEvent(
		uint32_t           dioNum
);

/**
 *   @n@b AIF_disableEgrDioEvent
 *
 *   @b Description
 *   @n This function is used to disable a DIO egress engine event.
 *
 *   @b Arguments
 *   @verbatim
       dioNum        Egress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            uint32_t           dioNum
            AIF_disableEgrDioEvent(dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void AIF_disableEgrDioEvent(
		uint32_t           dioNum
);

/**
 *   @n@b AIF_configureIngrDioEvent
 *
 *   @b Description
 *   @n This function is used prior to AIF_startHw to override the default parameter of a DIO engine ingress event.
 *
 *   @b Arguments
 *   @verbatim
       hAtEvent        Pointer to a Aif2Fl_AtEvent instance.
       dioNum          Ingress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AtEvent* hAtEvent
            uint32_t           dioNum
            AIF_configureIngrDioEvent(hAtEvent, dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_configureIngrDioEvent(
		Aif2Fl_AtEvent* hAtEvent,
		uint32_t           dioNum
);


/**
 *   @n@b AIF_enableIngrDioEvent
 *
 *   @b Description
 *   @n This function is used to enable a DIO ingress engine event.
 *
 *   @b Arguments
 *   @verbatim
       dioNum        Ingress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            uint32_t           dioNum
            AIF_enableIngrDioEvent(dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void AIF_enableIngrDioEvent(
		uint32_t           dioNum
);

/**
 *   @n@b AIF_disableIngrDioEvent
 *
 *   @b Description
 *   @n This function is used to disable a DIO ingress engine event.
 *
 *   @b Arguments
 *   @verbatim
       dioNum        Ingress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            uint32_t           dioNum
            AIF_disableIngrDioEvent(dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void AIF_disableIngrDioEvent(
		uint32_t           dioNum
);

/**
 *   @n@b AIF_configureEgrDioEngine
 *
 *   @b Description
 *   @n This function is used to override the default configuration of a DIO egress engine.
 *
 *   @b Arguments
 *   @verbatim
       hEgrDioEngine Pointer to the parameters for this egress DIO engine
       dioNum        Egress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AdDioEngine egrDioEngine
            uint32_t           dioNum
            AIF_configureEgrDioEngine(&egrDioEngine,dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_configureEgrDioEngine(
		Aif2Fl_AdDioEngine* hEgrDioEngine,
		uint32_t               dioNum
);

/**
 *   @n@b AIF_configureIngrDioEngine
 *
 *   @b Description
 *   @n This function is used to override the default configuration of a DIO ingress engine.
 *
 *   @b Arguments
 *   @verbatim
       hIngrDioEngine Pointer to the parameters for this ingress DIO engine
       dioNum         Ingress DIO engine number.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw().
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            Aif2Fl_AdDioEngine ingrDioEngine
            uint32_t           dioNum
            AIF_configureIngrDioEngine(&ingrDioEngine,dioNum);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_configureIngrDioEngine(
		Aif2Fl_AdDioEngine* hIngrDioEngine,
		uint32_t               dioNum
);

/**
 *   @n@b AIF_setRadTimerTc
 *
 *   @b Description
 *   @n This function is used to configure the RadTimerTc in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		clockNum		AT event count clock num
		symbolNum		AT event count symbol num
		frameLsbNum		AT event count frame lsb num
		lutIndexNum		AT event count lut index num
		radClockCountTc	pointer to an array of clock terminal counts of size lutIndexNum
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setRadTimerTc(AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER,139,AIF2_FRAME_COUNT_TC_WCDMA_FDD,AIF2_LTE_SYMBOL_NUM - 1);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setRadTimerTc(
		uint32_t 	clockNum,
		uint8_t 	symbolNum,
		uint32_t 	frameLsbNum,
		uint8_t 	lutIndexNum,
		uint32_t *radClockCountTc
);

/**
 *   @n@b AIF_setPhyTimerInit
 *
 *   @b Description
 *   @n This function is used to configure the PhyTimerInit in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		clockNum		AT event count clock num
		frameLsbNum		AT event count frame lsb num
		frameMsbNum		AT event count frame msb num
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setPhyTimerInit(ClockNum, 0, 0);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setPhyTimerInit(
   uint32_t       ClockNum,   
   uint32_t       FrameLsbNum,   
   uint32_t       FrameMsbNum
);

/**
 *   @n@b AIF_setDlRadTimerInit
 *
 *   @b Description
 *   @n This function is used to configure the DlRadTimerInit in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		SymbolNum		AT event count symbol num
		ClockNum		AT event count clock num
		FrameLsbNum		AT event count frame lsb num
		FrameMsbNum		AT event count frame msb num
		FcbMinusOne		FCB - 1 flag
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setDlRadTimerInit(0, ClockNum, 0, 0, 0);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setDlRadTimerInit(
   uint32_t       SymbolNum,
   uint32_t       ClockNum,
   uint32_t       FrameLsbNum,
   uint32_t       FrameMsbNum,
   uint16_t         FcbMinusOne
);

/**
 *   @n@b AIF_setUlRadTimerInit
 *
 *   @b Description
 *   @n This function is used to configure the UlRadTimerInit in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		SymbolNum		AT event count symbol num
		ClockNum		AT event count clock num
		FrameLsbNum		AT event count frame lsb num
		FrameMsbNum		AT event count frame msb num
		FcbMinusOne		FCB - 1 flag
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setUlRadTimerInit(0, ClockNum, 0, 0, 0);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setUlRadTimerInit(
   uint32_t       SymbolNum,
   uint32_t       ClockNum,
   uint32_t       FrameLsbNum,
   uint32_t       FrameMsbNum,
   uint16_t         FcbMinusOne
);

//returns the egress groupId associated with the AxC for dynamic control for MBSFN in LTE
#ifndef __AIF_INIT_C
extern
#endif
uint32_t AIF_getEgressGroupId(
    uint32_t AxC
);

/**
 *   @n@b AIF_setRmLinkSetupParams
 *
 *   @b Description
 *   @n This function is used to configure the RmLinkSetup threshold parameters for a given link number in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		link					link to configure
		RmFifoThold				Rm fifo threshold word size for reading received data
		bEnableLcvControl		enable the Line Code Violation counter
		losDetThreshold			8b10b los detect threshold values in number of Line Code Violations
		SyncThreshold			Threshold value for consecutive valid blocks of bytes which result in state ST1
		FrameSyncThreshold		Threshold value for consecutive valid message groups which result in state ST3
		UnsyncThreshold			Threshold value for consecutive invalid blocks of bytes which result in state ST0
		FrameUnsyncThreshold	Threshold value for consecutive invalid message groups which result in state ST1
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setRmLinkSetupParams(AIF2FL_RM_FIFO_THOLD_16DUAL,false,16,75,75,5,5);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setRmLinkSetupParams (
   /** which link to configure */
   int32_t link,
	
   /** setup Rm fifo threshold word size for reading received data */
   Aif2Fl_RmFifoThold      RmFifoThold,
   
   /** Writing a 1 to the bit will enable the Line Code Violation counter. THis 16 bit counter will saturate when it reaches a value of 0xffff. 
   Writing a 0 to this bit will clear and disable the counter. The current counter value is available as status, lcv_cntr_value */
   uint16_t                        bEnableLcvControl,
   /** Sets 8b10b los detect threshold values in number of Line Code Violations received during a master frame, OBSAI, or during a Hyperframe, 
   CPRI. Writing to this location will automatically clear the num_los counter and num_los_det status bit. Range 0 to 65,535*/
   uint16_t                      losDetThreshold,

    /** Threshold value for consecutive valid blocks of bytes which result in state ST1. Range 0 to 65,535 */
   uint16_t                      SyncThreshold,

    /** Threshold value for consecutive valid message groups which result in state ST3. Range 0 to 65,535*/
   uint16_t                      FrameSyncThreshold,

     /** Threshold value for consecutive invalid blocks of bytes which result in state ST0. Range 0 to 65,535 */
   uint16_t                      UnsyncThreshold,

    /** Threshold value for consecutive invalid message groups which result in state ST1. Range 0 to 65,535 */
   uint16_t                      FrameUnsyncThreshold
);

/**
 *   @n@b AIF_setPeFrameTC
 *
 *   @b Description
 *   @n This function is used to configure PE frame and symbol terminal counting value to calculate sop and eop of packet for a given group in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		index		index to one of the six sets of PE count value in order to  support up to 6 simultaneous LTE configurations
		cfg			PE frame and symbol terminal counting value to calculate sop and eop of packet
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
		Aif2Fl_FrameCounter cfg;
		
		cfg.FrameIndexSc = 0;
		cfg.FrameIndexTc = 13; 		// subframe processing
		cfg.FrameSymbolTc = 13;		// subframe processing
		AIF_setPeFrameTC(0,&cfg);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setPeFrameTC (
   int32_t index,
   Aif2Fl_FrameCounter *cfg
   );

/**
 *   @n@b AIF_setLinkPiMax
 *
 *   @b Description
 *   @n This function is used to configure PE frame and symbol terminal counting value to calculate sop and eop of packet for a given group in AIF2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
		link					link to configure
		piMax			        PiMax value to apply to this link
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
   	   int32_t                 link;
   	   uint32_t                PiMax;
		AIF_setLinkPiMax(link,PiMax);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void
AIF_setLinkPiMax (
   int32_t                 link,
   uint32_t                piMax
);

/**
 *   @n@b AIF_setPeFrameMsgTc
 *
 *   @b Description
 *   @n This function is used to configure the PE Frame message terminal count in AIF2LLD configuration structure.      
 *
 *   @b Arguments
 *   @verbatim
		indx	AxC index
		val		PE Frame message terminal count
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setPeFrameMsgTc(0, AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPRENORMAL1_SIZE);
     @endverbatim
 *
 */   
#ifndef __AIF_INIT_C
extern
#endif
void AIF_setPeFrameMsgTc (
    int32_t indx,
	uint16_t val
);

/**
 *   @n@b AIF_setPdChDioOffset
 *
 *   @b Description
 *   @n This function is used to configure the Pd Dma Channel DIO DMA offset in AIF2LLD configuration structure.
 *
 *   @b Arguments
 *   @verbatim
		indx	AxC index
		val		dio offset value for this AxC
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to AIF_initHw(). AIF2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  AIF2 configuration structure
 *
 *   @b Example
 *   @verbatim
            AIF_setPdChDioOffset(channelIdx, offset);
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
void AIF_setPdChDioOffset (
    int32_t indx,
	uint8_t val
);

/**
 *   @n@b AIF2_getVersion
 *
 *   @b Description
 *   @n The function is used to get the version information of the AIF2 LLD.
 *
 *   @b Arguments
 *   @verbatim
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t (version ID)
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  
 *
 *   @b Example
 *   @verbatim
     @endverbatim
 *
 */   
#ifndef __AIF_INIT_C
extern
#endif
uint32_t
AIF2_getVersion (
);

/**
 *   @n@b AIF2_getVersionStr
 *
 *   @b Description
 *   @n The function is used to get the version string for the AIF2 LLD
 *
 *   @b Arguments
 *   @verbatim
     @endverbatim
 *
 *   <b> Return Value </b>  pointer to version string
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  
 *
 *   @b Example
 *   @verbatim
     @endverbatim
 *
 */
#ifndef __AIF_INIT_C
extern
#endif
const char*
AIF2_getVersionStr(
);


#ifdef __cplusplus
}
#endif


#endif //__AIF_INIT_H

/** @} */ // end of module additions
