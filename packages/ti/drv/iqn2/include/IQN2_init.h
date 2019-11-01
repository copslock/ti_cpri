/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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
 * @file IQN2_init.h
 *
 * @brief Header file for IQN2 H/W initialization
 * 
*/


/** @addtogroup IQN2_FUNCTION  IQN2 Functions
 *  @{
 */


#ifndef __IQN2_INIT_H
#define __IQN2_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 *   @n@b IQN2_getIngressRadioStandardId
 *
 *   @b Description
 *   @n This function returns the ingress radio standard ID associated with the AxC.
 *
 *   @b Arguments
 *   @verbatim
        AxC        Ingress AxC number.
     @endverbatim
 *
 *   <b> Return Value </b>  Radio standard ID
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t axcId;
            IQN2_getIngressRadioStandardId(axcId);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
uint32_t
IQN2_getIngressRadioStandardId(
        uint32_t AxC
);

/**
 *   @n@b IQN2_getEgressRadioStandardId
 *
 *   @b Description
 *   @n This function returns the egress radio standard ID associated with the AxC. Used for dynamic control of MBSFN in LTE.
 *
 *   @b Arguments
 *   @verbatim
         AxC        Egress AxC number.
     @endverbatim
 *
 *   <b> Return Value </b>  Radio standard ID
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t axcId;
            IQN2_getEgressRadioStandardId(axcId);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
uint32_t
IQN2_getEgressRadioStandardId(
        uint32_t AxC
);

/**
 *   @n@b IQN2_getIngressRadioTimerId
 *
 *   @b Description
 *   @n This function returns the ingress radio timer ID associated with the AxC number. Ingress radio timer id
 *    is equal to Ingress radio standard id + max number of populated radio standards. Application may need to have
 *    separate radio timers for downlink and uplink. Following call to IQN2_initHw(), with this function, the
 *    application can get a radio timer ID and pass user-specified parameters for radio timers in IQN2_ConfigObj.
 *
 *   @b Arguments
 *   @verbatim
        AxC        Ingress AxC number.
     @endverbatim
 *
 *   <b> Return Value </b>  Radio standard ID
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t axcId;
            IQN2_getIngressRadioTimerId(axcId);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
uint32_t
IQN2_getIngressRadioTimerId(
        uint32_t AxC
);

/**
 *   @n@b IQN2_getEgressRadioTimerId
 *
 *   @b Description
 *   @n This function returns the egress radio timer ID associated with the AxC number. Egress radio timer id
 *    is equal to Egress radio standard id. Application may need to have  separate radio timers for downlink and uplink.
 *     Following call to IQN2_initHw(), with this function, the application can get a radio timer ID and pass
 *     user-specified parameters for radio timers in IQN2_ConfigObj.
 *
 *   @b Arguments
 *   @verbatim
        AxC        Egress AxC number.
     @endverbatim
 *
 *   <b> Return Value </b>  Radio standard ID
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t axcId;
            IQN2_getIngressRadioTimerId(axcId);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
uint32_t
IQN2_getEgressRadioTimerId(
        uint32_t AxC
);

/**
 *   @n@b IQN2_getRadioTimerFrameStrobe
 *
 *   @b Description
 *   @n This function returns the frame strobe enumeration for this radio timer id.
 *
 *   @b Arguments
 *   @verbatim
        radioId        Radio Timer Id [0:7].
     @endverbatim
 *
 *   <b> Return Value </b>  Frame strobe enum
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t radioTimerId;
            IQN2_getRadioTimerFrameStrobe(radioTimerId);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
Iqn2Fl_AtEvtStrobe
IQN2_getRadioTimerFrameStrobe(
        uint32_t radioId
);

/**
 *   @n@b IQN2_getRadioTimerSymbolStrobe
 *
 *   @b Description
 *   @n This function returns the symbol strobe enumeration for this radio timer id.
 *
 *   @b Arguments
 *   @verbatim
        radioId        Radio Timer Id [0:7].
     @endverbatim
 *
 *   <b> Return Value </b>  Symbol strobe enum
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t radioTimerId;
            IQN2_getRadioTimerSymbolStrobe(radioTimerId);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
Iqn2Fl_AtEvtStrobe
IQN2_getRadioTimerSymbolStrobe(
        uint32_t radioId
);

/** 
 *   @n@b IQN2_initDio
 *
 *   @b Description
 *   @n This function sets up the dioConfig[] structure given the user parameters.
 *
 *   @b Arguments
 *   @verbatim
		hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_calcParameters()
 *   
 *   <b> Post Condition </b>
 *   @n  IQN2_ConfigObj instance has valid parameters for DIO engines
 *
 *   @b Modifies    
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_initDio(hIqn2);
     @endverbatim
 * 
 */
#ifndef __IQN2_INIT_C
extern
#endif
void 
IQN2_initDio(
	IQN2_ConfigHandle    hIqn2
);

/** 
 *   @n@b IQN2_initHw
 *
 *   @b Description
 *   @n This function sets up the IQN2 FL setup structures given the IQN2_ConfigObj instance.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       hIqn2BaseAddr Pointer to the IQN2 device specific configuration used to pass configuration register base address
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 not started yet.
 *   
 *   <b> Post Condition </b>
 *   @n  IQN2 FL setup structures are populated.
 *
 *   @b Modifies    
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_initHw(hIqn2);
     @endverbatim
 * 
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initHw(
	IQN2_ConfigHandle hIqn2,
    Iqn2Fl_InitCfg*   hIqn2BaseAddr
);

/**
 *   @n@b IQN2_initRadioTimer
 *
 *   @b Description
 *   @n This function sets up the IQN2 FL setup Radio Timer structures given the IQN2_ConfigObj instance, and
 *    the egress/ingress radio standards configured during IQN2_initHw().
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 not started yet. Call to IQN2_initHw() already occurred.
 *
 *   <b> Post Condition </b>
 *   @n  IQN2 FL setup structures are populated.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_initRadioTimer(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initRadioTimer(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_initAt2Event
 *
 *   @b Description
 *   @n This function sets up the IQN2 FL setup AT2 event structures given the IQN2_At2Eventobj instance.
 *   This instance is also registered in IQN2_ConfigObj instance. Events will need to be enabled post call
 *   to IQN2_startHw() by calling IQN2_enableAt2Event() function.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       hAt2Evt       Pointer to a IQN2_At2Eventobj instance
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 not started yet. Call to IQN2_initHw() already occurred.
 *
 *   <b> Post Condition </b>
 *   @n  IQN2 FL setup structures are populated.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_At2EventHandle  hAt2Evt;
            IQN2_initAt2Event(hIqn2,hAt2Evt);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initAt2Event(
        IQN2_ConfigHandle    hIqn2,
        IQN2_At2EventHandle  hAt2Evt
);

/**
 *   @n@b IQN2_initNanoSecsToByteClocks
 *
 *   @b Description
 *   @n This function is a utility to convert EventOffset and EventModulo of an AT2 event instance from
 *   nansecs to byte clocks.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       hAt2Evt       Pointer to a IQN2_At2Eventobj instance
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_At2EventHandle  hAt2Evt;
            IQN2_initNanoSecsToByteClocks(hIqn2,hAt2Evt);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initNanoSecsToByteClocks(
        IQN2_ConfigHandle    hIqn2,
        IQN2_At2EventHandle  hAt2Evt
);

/**
 *   @n@b IQN2_initWcdmaChipsToByteClocks
 *
 *   @b Description
 *   @n This function is a utility to convert EventOffset and EventModulo of an AT2 event instance from
 *   Wcdma chip time to byte clocks.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       hAt2Evt       Pointer to a IQN2_At2Eventobj instance
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_At2EventHandle  hAt2Evt;
            IQN2_initWcdmaChipsToByteClocks(hIqn2,hAt2Evt);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initWcdmaChipsToByteClocks(
        IQN2_ConfigHandle    hIqn2,
        IQN2_At2EventHandle  hAt2Evt
);


/**
 *   @n@b IQN2_initEgrDioEvent
 *
 *   @b Description
 *   @n This function is used prior to IQN2_startHw to override the default parameter of a DIO engine egress event
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       hEgrDioEvent  Pointer to a IQN2_DioEventobj instance
       dioNum        DIO2 engine number
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 not started yet. Call to IQN2_initHw() already occurred.
 *
 *   <b> Post Condition </b>
 *   @n  IQN2 FL setup structures are populated.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_DioEventHandle  hDio2Evt;
            uint32_t             dioNum;
            IQN2_initEgrDioEvent(hIqn2,hDio2Evt,dioNum);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initEgrDioEvent(
		IQN2_ConfigHandle    hIqn2,
		IQN2_DioEventHandle  hEgrDioEvent,
		uint32_t             dioNum
);

/**
 *   @n@b IQN2_initIngrDioEvent
 *
 *   @b Description
 *   @n This function is used prior to IQN2_startHw to override the default parameter of a DIO engine ingress event
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       hIngrDioEvent  Pointer to a IQN2_DioEventobj instance
       dioNum        DIO2 engine number
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 not started yet. Call to IQN2_initHw() already occurred.
 *
 *   <b> Post Condition </b>
 *   @n  IQN2 FL setup structures are populated.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_DioEventHandle  hDio2Evt;
            uint32_t             dioNum;
            IQN2_initIngrDioEvent(hIqn2,hDio2Evt,dioNum);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initIngrDioEvent(
		IQN2_ConfigHandle    hIqn2,
		IQN2_DioEventHandle  hIngrDioEvent,
		uint32_t             dioNum
);

/**
 *   @n@b IQN2_initIngrDioAxCOffset
 *
 *   @b Description
 *   @n This function is used prior to IQN2_startHw to override the default parameter of AxC DIO ingress offset
 *
 *   @b Arguments
 *   @verbatim
       hIqn2         Pointer to a IQN2_ConfigObj instance.
       dioNum        Ingress DIO2 engine number
       axcNum        AxC number relative to this DIO engine
       offset        DIO Ingress AxC offset within the circular buffer (4-sample increments)
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 not started yet. Call to IQN2_initHw() already occurred.
 *
 *   <b> Post Condition </b>
 *   @n  IQN2 FL setup structures are populated.
 *
 *   @b Modifies
 *   @n  S/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             dioNum;
		    uint32_t             axcNum;
		    uint8_t              offset;
            IQN2_initIngrDioAxCOffset(hIqn2,dioNum,axcNum,offset);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initIngrDioAxCOffset(
		IQN2_ConfigHandle    hIqn2,
		uint32_t             dioNum,
		uint32_t             axcNum,
		uint8_t              offset
);

/**
 *   @n@b IQN2_initRmAilSetupParams
 *
 *   @b Description
 *   @n This function is used to configure the ail_phy_rm_regs threshold parameters for an AIL link number in IQN2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
        ailNum                  ail link to configure
        bEnableLcvControl       enable the Line Code Violation counter
        losDetThreshold         8b10b los detect threshold values in number of Line Code Violations
        SyncThreshold           Threshold value for consecutive valid blocks of bytes which result in state ST1
        FrameSyncThreshold      Threshold value for consecutive valid message groups which result in state ST3
        UnsyncThreshold         Threshold value for consecutive invalid blocks of bytes which result in state ST0
        FrameUnsyncThreshold    Threshold value for consecutive invalid message groups which result in state ST1
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw(). IQN2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  IQN2 configuration structure
 *
 *   @b Example
 *   @verbatim
            IQN2_initRmAilSetupParams(FALSE,16,75,75,5,5);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initRmAilSetupParams (
   Iqn2Fl_AilInstance    ailNum,
   int32_t               bEnableLcvControl,
   uint16_t              losDetThreshold,
   uint16_t              SyncThreshold,
   uint16_t              FrameSyncThreshold,
   uint16_t              UnsyncThreshold,
   uint16_t              FrameUnsyncThreshold
);

/**
 *   @n@b IQN2_initAilPiMax
 *
 *   @b Description
 *   @n This function is used to configure the ailUatSetup piMax parameter for an AIL link number in IQN2LLD configuration structure
 *
 *   @b Arguments
 *   @verbatim
        hIqn2                   Pointer to a IQN2_ConfigObj instance.
        ailNum                  ail link to configure
        piMax                   pi max value for this ail link
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Call to IQN2_initHw(). IQN2 not started yet.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  IQN2 configuration structure
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2
		    Iqn2Fl_AilInstance   ailNum
		    uint32_t             piMax
            IQN2_initAilPiMax(hIqn2,ailNum,piMax);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_initAilPiMax(
		IQN2_ConfigHandle    hIqn2,
		Iqn2Fl_AilInstance   ailNum,
		uint32_t             piMax
);


/** 
 *   @n@b IQN2_startHw
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers
 *      given user IQN2 configuration. HW is then started, waiting for
 *      the selected synchronization.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  
 *   
 *   <b> Post Condition </b>
 *   @n  The IQN2 links enabled by the user application are configured.
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_startHw(hIqn2);
     @endverbatim
 * 
 */
#ifndef __IQN2_INIT_C
extern
#endif
void
IQN2_startHw(
	IQN2_ConfigHandle hIqn2
);


/**
 *   @n@b IQN2_runBcnTimer
 *
 *   @b Description
 *   @n This function starts the BCN Timer free running. SW writes are not
     precise so it is expected in real case system, the application SW will
     correct the timer value with an offset.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2_startHw()
 *
 *   <b> Post Condition </b>
 *   @n  BCN timer is free running.
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_runBcnTimer(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_INIT_C
extern
#endif
void IQN2_runBcnTimer(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_getVersion
 *
 *   @b Description
 *   @n The function is used to get the version information of the IQN2 LLD.
 *
 *   @b Arguments
 *   @verbatim
     @endverbatim
 *
 *   <b> Return Value </b>  Uint32 (version ID)
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
#ifndef __IQN2_INIT_C
extern
#endif
Uint32
IQN2_getVersion (
);

/**
 *   @n@b IQN2_getVersionStr
 *
 *   @b Description
 *   @n The function is used to get the version string for the IQN2 LLD
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
#ifndef __IQN2_INIT_C
extern
#endif
const char*
IQN2_getVersionStr(
);


#ifdef __cplusplus
}
#endif



#endif //__IQN2_INIT_H

/** @} */ // end of module additions
