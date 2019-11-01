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

#ifndef EXTERNAL_SYNC
#define EXTERNAL_SYNC           0
#endif
#define EXTERNAL_SYNC_TIMER     0   // Used to trigger an external sync, Frame sync with one shot pulse - could be any timer

#if LTE_RATE == 20
#define NUM_AXCS_LTE20_AID     2
#define NUM_AXCS_LTE10_AID     0
#define NUM_AXCS_LTE5_AID      0
#define NUM_AXCS_LTE15_AID     0
#define NUM_AXCS_LTE40_AID     0
#define NUM_AXCS_LTE80_AID     0

#elif LTE_RATE == 10
#define NUM_AXCS_LTE20_AID     0
#define NUM_AXCS_LTE10_AID     4
#define NUM_AXCS_LTE5_AID      0
#define NUM_AXCS_LTE15_AID     0
#define NUM_AXCS_LTE40_AID     0
#define NUM_AXCS_LTE80_AID     0

#elif LTE_RATE == 5
#define NUM_AXCS_LTE20_AID     0
#define NUM_AXCS_LTE10_AID     0
#define NUM_AXCS_LTE5_AID      4
#define NUM_AXCS_LTE15_AID     0
#define NUM_AXCS_LTE40_AID     0
#define NUM_AXCS_LTE80_AID     0

#else
#define NUM_AXCS_LTE20_AID     0
#define NUM_AXCS_LTE10_AID     0
#define NUM_AXCS_LTE5_AID      8
#define NUM_AXCS_LTE15_AID     0
#define NUM_AXCS_LTE40_AID     0
#define NUM_AXCS_LTE80_AID     0
#endif

#define NUM_AXCS_LTE_AID       NUM_AXCS_LTE20_AID + NUM_AXCS_LTE10_AID + NUM_AXCS_LTE5_AID + NUM_AXCS_LTE15_AID + NUM_AXCS_LTE40_AID + NUM_AXCS_LTE80_AID
#define NUM_AXCS_WCDMA_AID     8
#define AID_RATE               IQN2FL_LINK_RATE_8x

#ifdef WCDMA_UL
#define DIO_NUM_BLOCK           32 //used large block only for test purpose
#define NUM_CHIP_PER_EVT        8
#define RSA_ON                  1
#else
#define DIO_NUM_BLOCK           64 //used large block only for test purpose
#define NUM_CHIP_PER_EVT        4
#define RSA_ON                  0
#endif

//////////////////////////

