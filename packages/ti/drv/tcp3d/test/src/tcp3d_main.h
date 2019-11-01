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
#ifndef _TCP3D_MAIN_H_
#define _TCP3D_MAIN_H_

/* XDC includes */
//#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/IHeap.h>

/* Driver includes */
#include <ti/drv/tcp3d/tcp3d_drv.h>
#include <ti/drv/tcp3d/src/tcp3d_utils.h>

/* Utility includes */
#include "tcp3d_codeBlkSeg.h"

/**********************************************************************
 ************************** Test Macros *******************************
 **********************************************************************/
/**
 * Address mapping is done based on the mapping shown below.
 * Check on the address done to ensure it be in L2SRAM space.
 * 
 * L2SRAM      : org = 0x00800000, len = 0x100000 (local)
 * GEM0_L2_MEM : org = 0x10800000, len = 0x100000 (global)
 * GEM1_L2_MEM : org = 0x11800000, len = 0x100000 (global)
 * GEM2_L2_MEM : org = 0x12800000, len = 0x100000 (global)
 * GEM3_L2_MEM : org = 0x13800000, len = 0x100000 (global)
 *
 * MSMCSRAM    : org = 0x0c000000, len = 0x200000 (global)
 */
#define L2GLBMAP(coreID, addr)   \
    ( ( ((UInt32)(addr) >= 0x00800000) && ((UInt32)(addr) < 0x00900000) ) ? \
      ( (UInt32)(addr) | (UInt32)((0x10 | (coreID & 0x3)) << 24) ) : \
      (UInt32)(addr) ) 

/**********************************************************************
 ************************** Test Compile Flags ************************
 **********************************************************************/
/**
 * Prepare all input config registers outside the Send loop (block size based, etc)
 * except for beta states since they are data dependent.
 */
#define TEST_PREPARE_ONLY_BETASTATE                     1

/**
 * Set this flag to use the fixed input configuration parameters preparation
 * optimization. This flag is useful only if TEST_PREPARE_ONLY_BETASTATE is
 * not used.
 */
#define TEST_PREPARE_ONLY_CODEBLOCK_PARAM_DEPENDENT     0

/**
 * Check the beta state values with the reference from the file generated 
 * with the test vectors.
 */
#define TEST_BETA_VALUE_CHECK                           0

/**********************************************************************
 ************************** Test Definitions **************************
 **********************************************************************/
/* TCP3D modes */
#define TEST_MODE_SINGLE        CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_3GPP   //0 (NOT SUPPORTED)
#define TEST_MODE_LTE           CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_LTE    //1
#define TEST_MODE_WIMAX         CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_WIMAX  //2
#define TEST_MODE_SPLIT         CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_HSUPA  //3

/* Test True/False flags */
#define TEST_FALSE              0
#define TEST_TRUE               1

/* Test Interrupt flags */
#define TEST_INTR_ENABLE        1
#define TEST_INTR_DISABLE       0

/* Enable the correct power domain for the device. */
#ifdef DEVICE_K2L
#define TEST_CSL_PSC_PD_TCP3D_0       CSL_PSC_PD_TCP3D_0
#define TEST_CSL_PSC_LPSC_TCP3D_0     CSL_PSC_LPSC_TCP3D_0
#define TEST_CSL_PSC_PD_TCP3D_1       CSL_PSC_PD_TCP3D_1
#define TEST_CSL_PSC_LPSC_TCP3D_1     CSL_PSC_LPSC_TCP3D_1
#else /* DEVICE_K2K, DEVICE_K2H */
#define TEST_CSL_PSC_PD_TCP3D_0       CSL_PSC_PD_TCP3D_01
#define TEST_CSL_PSC_LPSC_TCP3D_0     CSL_PSC_LPSC_TCP3D_0
#define TEST_CSL_PSC_PD_TCP3D_1       CSL_PSC_PD_TCP3D_01
#define TEST_CSL_PSC_LPSC_TCP3D_1     CSL_PSC_LPSC_TCP3D_1
#endif

/**********************************************************************
 ************************** Test Structures ***************************
 **********************************************************************/
/*
 * Structure that holds the configuration parameters. Used for storing the
 * configuration values read from the file blockXX_cfgreg.dat for a given
 * code block.
 */
typedef struct cbConfig
{
    /* Control */
    Int32 mode_sel;         //TCP3_MODE
    Int32 lte_crc_init_sel;

    /* Input */
    Int32 NumInfoBits;      //CFG0
    Int32 SW0_length;       //CFG1
    Int32 maxst_en;         //CFG2
    Int32 out_flag_en;
    Int32 out_order_sel;
    Int32 ext_scale_en;
    Int32 soft_out_flag_en;
    Int32 soft_out_fmt;
    Int32 min_itr;
    Int32 max_itr;
    Int32 snr_val;
    Int32 snr_rep;
    Int32 stop_sel;
    Int32 crc_iter_pass;
    Int32 crc_sel;
    Int32 maxst_thold;        //CFG3
    Int32 maxst_value;
    Int32 ext_scale_0;        //CFG8
    Int32 ext_scale_1;
    Int32 ext_scale_2;
    Int32 ext_scale_3;
    Int32 ext_scale_4;        //CFG9
    Int32 ext_scale_5;
    Int32 ext_scale_6;
    Int32 ext_scale_7;
    Int32 ext_scale_8;        //CFG10
    Int32 ext_scale_9;
    Int32 ext_scale_10;
    Int32 ext_scale_11;
    Int32 ext_scale_12;       //CFG11
    Int32 ext_scale_13;
    Int32 ext_scale_14;
    Int32 ext_scale_15;
} cbConfig;

/*
 * Structure for one code block description.
 */
typedef struct cbDataDesc
{
    UInt32      mode;
    UInt32      crcInitVal;
    Int8        tailBits[12];
    Tcp3d_InCfgParams   *inCfgParams;
    UInt8       sw0LengthUsed;

    UInt32  blockSize;
    UInt32  interFlag;
    UInt32  sdFlag;
    UInt32  stsFlag;
    UInt32  llrOffset;
    UInt32  sdOffset;

    UInt32  sizeCFG;
    UInt32  sizeINTER;
    UInt32  sizeLLR;
    UInt32  sizeHD;
    UInt32  sizeSD;
    UInt32  sizeSTS;

    UInt32  *inCfg;
    UInt16  *inInter;
    Int8    *inLLR; /* three arrays with offset in llrOffset */
    UInt32  *outHD;
    UInt32  *refHD;
    Int8    *outSD; /* three arrays with offset in sdOffset */
    Int8    *refSD; /* three arrays with offset in sdOffset */
    UInt32  *outSts;
    UInt32  *refSts;
} cbDataDesc;

/*
 * Structure for one test description. 
 */
typedef struct cbTestDesc
{
    cbDataDesc  **cbData;
    Int32       maxNumCB;
    Int32       mode;
    Int32       doubleBuffer;
    Int32       lteCrcSel;
} cbTestDesc;

/**********************************************************************
 ************************** Test Global Tables ************************
 **********************************************************************/
/* interleaver tables (used in tcp3d_inputCongigPrep.c file) */
extern Int16 TCP3_LteInterleaverTable[220][7];
extern Int16 TCP3_WimaxInterleaverTable[17][4];

/**********************************************************************
 *********************** Test Global Functions ************************
 **********************************************************************/
/* interleaver table index calculation function */
Int32 LTE_interleaver_index(Int32 K);
Int32 WIMAX_interleaver_index(Int32 K);

/* input config prepare function definitions (see tcp3d_inputCongigPrep.c) */
Void prepareBlockSizeDepICParams(cbDataDesc *cbPtr);
Void prepareBetaStateICParams(cbDataDesc *cbPtr, UInt8 mode);
Void prepareIC(cbDataDesc *cbPtr, UInt32 *tempIC, UInt8 copyFlag);
Void fillICParams(Tcp3d_InCfgParams *inCfgParams, cbConfig *cbCfg);
Void checkBetaValues (UInt32 inCfg[]);

/* Test vector functions (see tcp3d_testvector.c) */
Int getTestSetCB(IHeap_Handle dataHeap, cbTestDesc *cbTestSet, Char *testFolder);
Void freeTestSetCB(IHeap_Handle dataHeap, cbTestDesc *cbTestSet);

#endif  /* _TCP3D_MAIN_H_ */
