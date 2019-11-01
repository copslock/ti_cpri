/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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



#ifndef CFG_PARAM_H
#define CFG_PARAM_H
#include "typedefs.h"


/** 
 *  \struct   _OUT_TCP3_REGS
 * 
 *  \brief   The structure holds configuration parameters 
 *           used by tcp3d driver test bench.
 * 
 *  \sa    
 *  
 */
typedef struct _OUT_TCP3_REGS
{
	int32_t mode_sel;                 //TCP3_MODE
	int32_t lte_crc_init_sel;
	int32_t NumInfoBits; //Not part of tcp3 registers
	int32_t SW0_length;  //Not part of tcp3 registers
	int32_t maxst_en;
	int32_t out_flag_en;
	int32_t out_order_sel;
	int32_t ext_scale_en;
	int32_t soft_out_flag_en;
	int32_t soft_out_fmt;
	int32_t min_itr;
	int32_t max_itr;
	int32_t snr_val;
	int32_t snr_rep;
	int32_t stop_sel;
	int32_t crc_iter_pass;
	int32_t crc_sel;
	int32_t maxst_thold;              //CFG3
	int32_t maxst_value;
	int32_t ext_scale_0;              //CFG8
	int32_t ext_scale_1;
	int32_t ext_scale_2;
	int32_t ext_scale_3;
	int32_t ext_scale_4;              //CFG9
	int32_t ext_scale_5;
	int32_t ext_scale_6;
	int32_t ext_scale_7;
	int32_t ext_scale_8;              //CFG10
	int32_t ext_scale_9;
	int32_t ext_scale_10;
	int32_t ext_scale_11;
	int32_t ext_scale_12;             //CFG11
	int32_t ext_scale_13;
	int32_t ext_scale_14;
	int32_t ext_scale_15;
} OUT_TCP3_REGS;




#endif