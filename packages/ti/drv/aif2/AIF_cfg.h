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
 * @file AIF_cfg.h
 *
 * @brief Header file for AIF configuration initialization
 * 
*/


/** @addtogroup AIF_FUNCTION  AIF Functions
 *  @{
 */

#ifndef __AIF_CFG_H
#define __AIF_CFG_H

#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/AIF_defs.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef K2
#define AIF_CFG_MAX_CORE	8
#else
#define AIF_CFG_MAX_CORE	4
#endif

typedef struct AIF_CoreCfgObj {
					//Aif mode
					AIF_Mode 				mode;
					//AIF link enable or disable, 1 or 0
					uint32_t                   linkEnable[AIF_MAX_NUM_LINKS];
					//AIF link rate (1=1x; 2=2x; 4= 4x).
					uint32_t                   linkRate[AIF_MAX_NUM_LINKS];
					//LTE number of AxCs per core.
					uint32_t                   numAxC[AIF_MAX_NUM_LINKS];
					//First AxC per core
					uint32_t                   firstAxCIndex[AIF_MAX_NUM_LINKS];
					//AIF link data type for outbound burst traffic.
					Aif2Fl_LinkDataType     outboundDataType[AIF_MAX_NUM_LINKS];
					//AIF link data width for outbound burst traffic.
					Aif2Fl_DataWidth        outboundDataWidth[AIF_MAX_NUM_LINKS];
					//AIF link data type for inbound burst traffic.
					Aif2Fl_LinkDataType     inboundDataType[AIF_MAX_NUM_LINKS];
					//AIF link data width for inbound burst traffic.
					Aif2Fl_DataWidth        inboundDataWidth[AIF_MAX_NUM_LINKS];
					//AIF link DIO engine used
					Aif2Fl_DioEngineIndex   dioEngine[AIF_MAX_NUM_LINKS];
					//link mode, need to select LTE or WCDMA
					AIF2_LinkMode			 linkMode[AIF_MAX_NUM_LINKS];
					//Enable control words
					uint32_t 					 CWEnable;
					// PE AxC offset value
					uint32_t 					 peAxCOffset;
					// PD AxC offset value
					uint32_t 					 pdAxCOffset;
					// Egress Buffer Depth
					Aif2Fl_DbFifoDepth 	 egressBufDepth;
					// Ingress Buffer Depth
					Aif2Fl_DbFifoDepth		 ingressBufDepth;
			} AIF_CoreCfgObj, *AIF_CoreCfgHandle;

typedef struct AIF_CfgObj {
					// Test name
					const unsigned char 	name[32];
					//Protocol(CPRI or OBSAI)
					Aif2Fl_LinkProtocol 	protocol;
					//Aif mode
					AIF_Mode 				mode;
					//pktdma or dio
					Aif2Fl_CppiDio 		pktdmaOrDioEngine;
					//
					AIF_CoreCfgObj 			coreCfg[AIF_CFG_MAX_CORE];
		}AIF_CfgObj, *AIF_CfgHandle;
 
 
#ifndef __AIF_CFG_C
extern
#endif

AIF_CoreCfgObj*
AIF_getCfg(
	AIF_CfgHandle    ptrInitCfg,
	uint32_t		 maxCfgBlock,
	uint32_t		 core
);

#ifdef __cplusplus
}
#endif


#ifndef __AIF_CFG_C
extern
#endif
//AIF_ConfigHandle
void
AIF_populateAifObj(
		AIF_ConfigHandle	hAifObj,
		AIF_CfgHandle 	 	hAifCfgObj,
		uint32_t 				clockspeed,
		uint32_t 				swSync,
		uint32_t 				intLoopback,
		uint32_t 				lte_rate
);

#ifndef __AIF_CFG_C
extern
#endif
void
AIF_startCfg(
);


#endif //__AIF_INIT_H

/** @} */ // end of module additions
