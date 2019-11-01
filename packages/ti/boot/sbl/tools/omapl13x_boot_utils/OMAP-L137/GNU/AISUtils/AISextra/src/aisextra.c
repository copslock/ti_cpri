/*
 * aisextra.c
*/

/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
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
/* --------------------------------------------------------------------------
    FILE        : aisxtra.c 				                             	 	        
    PURPOSE     : AIS Extra commands for use in AIS Scripts
    PROJECT     : TI Boot and Flash Utilities
    AUTHOR      : Daniel Allred
 ----------------------------------------------------------------------------- */

/************************************************************
* Include Files                                             *
************************************************************/
 
#include "tistdtypes.h"
#include "device.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/
#if defined(__TMS320C6X__)
  
	#pragma DATA_SECTION(paramTable,".params")
	Uint32 paramTable[16];

#elif defined(__GNUC__)
	Uint32 paramTable[16] __attribute__((,section(".params")));
#endif

/************************************************************
* Global Function Definitions                               *
************************************************************/
#if defined(__TMS320C6X__)
  #pragma CODE_SECTION(setEmifA45Div,".text")
  void setEmifA45Div()
#elif defined(__GNUC__)
  void setEmifA45Div() __attribute__((,section(".text")));
  void setEmifA45Div()
#endif
{
  SYSTEM->CFGCHIP[3] = SYSTEM->CFGCHIP[3] | 0x6;
}

#if defined(__TMS320C6X__)
  #pragma CODE_SECTION(setEmifB45Div,".text")
  void setEmifB45Div()
#elif defined(__GNUC__)
  void setEmifB45Div() __attribute__((,section(".text")));
  void setEmifB45Div()
#endif
{
  SYSTEM->CFGCHIP[3] = SYSTEM->CFGCHIP[3] | 0x5;
}


/************************************************************
* End File                                                  *
************************************************************/
