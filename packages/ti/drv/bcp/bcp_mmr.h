/** 
 *   @file  bcp_mmr.h
 *
 *   @brief  
 *      Header file with BCP Memory Mapped Register (MMR) access functions.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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

#ifndef _BCP_MMR_H_
#define _BCP_MMR_H_

#include <ti/drv/bcp/bcp_lld.h>

/** @addtogroup BCP_LLD_DATASTRUCT
 @{ */

/** 
 *  @brief  Bcp_PeripheralIdParams
 *          
 *          Configuration structure to hold BCP 
 *          Peripheral ID Register contents.
 */
typedef struct _Bcp_PeripheralIdParams
{
    /** Minor Version Number (Y) */
    uint8_t             minorNum;
    /** Custom Version Number */
    uint8_t             customNum;
    /** Major Version Number (X) */
    uint8_t             majorNum;
    /** RTL Version Number (R) */
    uint8_t             rtlVersion;
    /** Fixed module ID. */        
    uint16_t            function;
    /** Scheme. */        
    uint8_t             scheme;
} Bcp_PeripheralIdParams;

/** 
 *  @brief  Bcp_ModuleIdleStatus
 *          
 *          Configuration structure to hold the idle
 *          status of various BCP submodules.
 */
typedef struct _Bcp_ModuleIdleStatus
{
    uint8_t             tm_idle_status;        
    uint8_t             dio_idle_status;        
    uint8_t             crc_idle_status;        
    uint8_t             enc_idle_status;        
    uint8_t             rm_idle_status;        
    uint8_t             mod_idle_status;        
    uint8_t             int_idle_status;        
    uint8_t             cor_idle_status;        
    uint8_t             ssl_idle_status;        
    uint8_t             rd_idle_status;        
    uint8_t             dnt_idle_status;        
    uint8_t             all_idle_status;        
} Bcp_ModuleIdleStatus;

/** 
 *  @brief  Bcp_ModuleCfg
 *          
 *          Structure to hold the configuration
 *          parameters for various BCP submodules.
 */
typedef struct _Bcp_ModuleCfg
{
    uint8_t             tm;        
    uint8_t             dio;        
    uint8_t             crc;        
    uint8_t             enc;        
    uint8_t             rm;        
    uint8_t             mod;        
    uint8_t             intleaver;        
    uint8_t             cor;        
    uint8_t             ssl;        
    uint8_t             rd;        
    uint8_t             dnt;        
} Bcp_ModuleCfg;

/** 
 *  @brief  Bcp_TmIntCfg
 *          
 *          Structure to hold the configuration
 *          parameters/status for Traffic Manager (TM)
 *          Interrupts.
 */
typedef struct _Bcp_TmIntCfg
{
    uint8_t             ppb_error;        
    uint8_t             dlg_done_int;        
    uint8_t             eng_error;        
} Bcp_TmIntCfg;

/** 
 *  @brief  Bcp_TmFlowEntry
 *          
 *          Structure to hold the configuration
 *          parameters of a TM Flow table entry.
 */
typedef struct _Bcp_TmFlowEntry
{
    /** Input data endian format */
    Bcp_EndianFormat    endian_in;        

    /** Input data format */
    Bcp_DataFormat      format_in;        

    /** Packet type to put in Info Word 0 on Rx */
    uint8_t             pkt_type;

    /** Which DSP to interrupt */
    uint8_t             dsp_int_sel;

    /** Output data endian format */
    Bcp_EndianFormat    endian_out;        

    /** Output data format */
    Bcp_DataFormat      format_out;        

    /** Output QFIFO to use */
    uint8_t             qfifo_out;

    /** PS flags to configure in packet Info word 0 */
    uint8_t             ps_flags;
} Bcp_TmFlowEntry;

/** 
 *  @brief  Bcp_DlgCtrlMode
 *
 *          Enumeration for specifying the DLG control modes.
 */        
typedef enum   
{
    /** Idle Mode.
     *  
     *  Data logger does not capture any information and write pointer
     *  is reset to address 0.
     */
    Bcp_DlgCtrlMode_Idle                = 0,
    /** Errors only Mode.
     *  
     *  Data logger stores information only on errored packets. Write counter
     *  is allowed to wrap so writes continue indefinitely.
     */
    Bcp_DlgCtrlMode_ErrorsOnly          = 1,
    /** Start On Error Mode.
     *  
     *  Data logger starts capturing data on next error and captures info
     *  for all following packets until all memory locations have been written.
     */
    Bcp_DlgCtrlMode_StartOnError        = 2,
    /** Stop On Error Mode.
     *  
     *  Data logger stores data from all packets until an error occurs.
     *  When the error occurs it stores that packets data and stops 
     *  collecting data.
     */
    Bcp_DlgCtrlMode_StopOnError         = 3,
    /** Free Run Mode.
     *  
     *  Data logger stores data from all packets. Write counter is allowed
     *  to wrap so writes continue indefinitely.
     */
    Bcp_DlgCtrlMode_FreeRun             = 4,
    /** One Shot Mode.
     *  
     *  Data logger stores data from all packets starting at location 0
     *  and continues until all memory has been written and then stops.
     */
    Bcp_DlgCtrlMode_OneShot             = 5,
    /** Hold Mode.
     *  
     *  Data logger does not store any more information but does not clear
     *  the write pointer.
     */
    Bcp_DlgCtrlMode_Hold                = 7
} Bcp_DlgCtrlMode;

/** @brief
 * Maximum number of Data logger RAM entries in any BCP Submodule
 */        
#define     BCP_MAX_NUM_DLG_RAM_ENTRIES     (256)                 

/** 
 *  @brief  Bcp_DlgRamEntry
 *          
 *          Structure to hold the DLG RAM entry contents.
 */
typedef struct _Bcp_DlgRamEntry
{
    /** Global Header field contents */
    uint32_t            global_hdr_val;        

    /** Software Timestamp value */
    uint32_t            sw_timestamp_val;        

    /** Hardware Timestamp value */
    uint8_t             hw_timestamp_val;        

    /** Engine error field value */
    uint8_t             engine_error_val;        
} Bcp_DlgRamEntry;


/**
@}
*/


/** @addtogroup BCP_LLD_FUNCTION
 @{ 
 */

/****************** Encoder Engine MMR ******************/

/**
 * ============================================================================
 *  @n@b Bcp_getEncPolyCoef1Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of POLY_COEF1 register. The POLY_COEF1 
 *      register contains the convolutional encoder coefficient for 
 *      WCDMA Code Rate 1/2.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of POLY_COEF1 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncPolyCoef1Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->POLY_COEF1, BCP_ENC_POLY_COEF1_POLYCOEF1);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncPolyCoef1Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of POLY_COEF1 register. The POLY_COEF1 
 *      register contains the convolutional encoder coefficient for 
 *      WCDMA Code Rate 1/2.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyCoefVal         Value to configure in POLY_COEF1 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Encoder engine's POLY_COEF1 register successfully
 *                                  configured.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  POLY_COEF1 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncPolyCoef1Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyCoefVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->POLY_COEF1, BCP_ENC_POLY_COEF1_POLYCOEF1, polyCoefVal);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncPolyCoef2Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of POLY_COEF2 register. The POLY_COEF2 
 *      register contains the convolutional encoder coefficient for 
 *      WCDMA Code Rate 1/3.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of POLY_COEF2 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncPolyCoef2Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->POLY_COEF2, BCP_ENC_POLY_COEF2_POLYCOEF2);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncPolyCoef2Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of POLY_COEF2 register. The POLY_COEF2 
 *      register contains the convolutional encoder coefficient for 
 *      WCDMA Code Rate 1/3.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyCoefVal         Value to configure in POLY_COEF2 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  POLY_COEF2 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncPolyCoef2Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyCoefVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->POLY_COEF2, BCP_ENC_POLY_COEF2_POLYCOEF2, polyCoefVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncPolyCoef3Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of POLY_COEF3 register. The POLY_COEF3 
 *      register contains the convolutional encoder coefficient for 
 *      LTE Code Rate 1/3.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of POLY_COEF3 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncPolyCoef3Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);


    return CSL_FEXT (pEncRegs->POLY_COEF3, BCP_ENC_POLY_COEF3_POLYCOEF3);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncPolyCoef3Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of POLY_COEF3 register. The POLY_COEF3 
 *      register contains the convolutional encoder coefficient for 
 *      LTE Code Rate 1/3.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyCoefVal         Value to configure in POLY_COEF3 register.

 *  @return     void
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  POLY_COEF3 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncPolyCoef3Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyCoefVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->POLY_COEF3, BCP_ENC_POLY_COEF3_POLYCOEF3, polyCoefVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncScrInit0Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of SCR_INIT_0 register. The SCR_INIT_0 
 *      register contains the WiMax randomizer initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of SCR_INIT_0 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncScrInit0Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->SCR_INIT_0, BCP_ENC_SCR_INIT_0_SCRINIT);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncScrInit0Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of SCR_INIT_0 register. The SCR_INIT_0 
 *      register contains the WiMax randomizer initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        initVal             Value to configure in SCR_INIT_0 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  SCR_INIT_0 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncScrInit0Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                initVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->SCR_INIT_0, BCP_ENC_SCR_INIT_0_SCRINIT, initVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncScrPoly0Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of SCR_POLY_0 register. The SCR_POLY_0 
 *      register contains the WiMax randomizer polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of SCR_POLY_0 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncScrPoly0Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->SCR_POLY_0, BCP_ENC_SCR_POLY_0_SCRPOLY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncScrPoly0Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of SCR_POLY_0 register. The SCR_POLY_0 
 *      register contains the WiMax randomizer polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyVal             Value to configure in SCR_POLY_0 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  SCR_POLY_0 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncScrPoly0Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->SCR_POLY_0, BCP_ENC_SCR_POLY_0_SCRPOLY, polyVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncCrc24Init0Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of CRC24_INIT_0 register. The CRC24_INIT_0 
 *      register contains the CRC-24 initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of CRC24_INIT_0 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncCrc24Init0Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->CRC24_INIT_0, BCP_ENC_CRC24_INIT_0_CRC24INIT);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncCrc24Init0Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of CRC24_INIT_0 register. The CRC24_INIT_0 
 *      register contains the CRC-24 initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        initVal             Value to configure in CRC24_INIT_0 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  CRC24_INIT_0 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncCrc24Init0Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                initVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->CRC24_INIT_0, BCP_ENC_CRC24_INIT_0_CRC24INIT, initVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncCrc24Poly0Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of CRC24_POLY_0 register. The CRC24_POLY_0 
 *      register contains the CRC-24 polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of CRC24_POLY_0 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncCrc24Poly0Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->CRC24_POLY_0, BCP_ENC_CRC24_POLY_0_CRC24POLY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncCrc24Poly0Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of CRC24_POLY_0 register. The CRC24_POLY_0 
 *      register contains the CRC-24 polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyVal             Value to configure in CRC24_POLY_0 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  CRC24_POLY_0 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncCrc24Poly0Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->CRC24_POLY_0, BCP_ENC_CRC24_POLY_0_CRC24POLY, polyVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncScrInit1Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of SCR_INIT_1 register. The SCR_INIT_1 
 *      register contains the WiMax randomizer initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of SCR_INIT_1 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncScrInit1Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->SCR_INIT_1, BCP_ENC_SCR_INIT_1_SCRINIT);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncScrInit1Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of SCR_INIT_1 register. The SCR_INIT_1 
 *      register contains the WiMax randomizer initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        initVal             Value to configure in SCR_INIT_1 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  SCR_INIT_1 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncScrInit1Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                initVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->SCR_INIT_1, BCP_ENC_SCR_INIT_1_SCRINIT, initVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncScrPoly1Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of SCR_POLY_1 register. The SCR_POLY_1 
 *      register contains the WiMax randomizer polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of SCR_POLY_1 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncScrPoly1Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->SCR_POLY_1, BCP_ENC_SCR_POLY_1_SCRPOLY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncScrPoly1Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of SCR_POLY_1 register. The SCR_POLY_1 
 *      register contains the WiMax randomizer polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyVal             Value to configure in SCR_POLY_1 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  SCR_POLY_1 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncScrPoly1Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->SCR_POLY_1, BCP_ENC_SCR_POLY_1_SCRPOLY, polyVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncCrc16Init1Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of CRC16_INIT_1 register. The CRC16_INIT_1 
 *      register contains the CRC-16 initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of CRC16_INIT_1 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncCrc16Init1Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->CRC16_INIT_1, BCP_ENC_CRC16_INIT_1_CRC16INIT);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncCrc16Init1Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of CRC16_INIT_1 register. The CRC16_INIT_1 
 *      register contains the CRC-16 initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        initVal             Value to configure in CRC16_INIT_1 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  CRC16_INIT_1 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncCrc16Init1Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                initVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->CRC16_INIT_1, BCP_ENC_CRC16_INIT_1_CRC16INIT, initVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncCrc16Poly1Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of CRC16_POLY_1 register. The CRC16_POLY_1 
 *      register contains the CRC-16 polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of CRC16_POLY_1 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncCrc16Poly1Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->CRC16_POLY_1, BCP_ENC_CRC16_POLY_1_CRC16POLY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncCrc16Poly1Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of CRC16_POLY_1 register. The CRC16_POLY_1 
 *      register contains the CRC-16 polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyVal             Value to configure in CRC16_POLY_1 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  CRC16_POLY_1 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncCrc16Poly1Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->CRC16_POLY_1, BCP_ENC_CRC16_POLY_1_CRC16POLY, polyVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncScrInit2Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of SCR_INIT_2 register. The SCR_INIT_2 
 *      register contains the WiMax randomizer initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of SCR_INIT_2 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncScrInit2Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->SCR_INIT_2, BCP_ENC_SCR_INIT_2_SCRINIT);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncScrInit2Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of SCR_INIT_2 register. The SCR_INIT_2 
 *      register contains the WiMax randomizer initialization value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        initVal             Value to configure in SCR_INIT_2 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  SCR_INIT_2 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncScrInit2Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                initVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->SCR_INIT_2, BCP_ENC_SCR_INIT_2_SCRINIT, initVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEncScrPoly2Reg
 *
 *  @b  brief
 *  @n  This API returns the contents of SCR_POLY_2 register. The SCR_POLY_2 
 *      register contains the WiMax randomizer polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of SCR_POLY_2 register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEncScrPoly2Reg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    return CSL_FEXT (pEncRegs->SCR_POLY_2, BCP_ENC_SCR_POLY_2_SCRPOLY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setEncScrPoly2Reg
 *
 *  @b  brief
 *  @n  This API populates the contents of SCR_POLY_2 register. The SCR_POLY_2 
 *      register contains the WiMax randomizer polynomial value.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        polyVal             Value to configure in SCR_POLY_2 register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  SCR_POLY_2 register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setEncScrPoly2Reg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                polyVal
)
{
    CSL_Bcp_encRegs*        pEncRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pEncRegs    =   (CSL_Bcp_encRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_ENC]);

    CSL_FINS (pEncRegs->SCR_POLY_2, BCP_ENC_SCR_POLY_2_SCRPOLY, polyVal);

    return 0;
}

/****************** Correlation Engine MMR ******************/

/**
 * ============================================================================
 *  @n@b Bcp_getCorReedMullerTableColumn
 *
 *  @b  brief
 *  @n  This API returns the contents of M register. Each of the M register 
 *      corresponds to a Reed Muller Table column.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        colNum              Column number to be read.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of M register corresponding to the
 *                                  column number specified.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getCorReedMullerTableColumn
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                colNum
)
{
    CSL_Bcp_corRegs*        pCorRegs;
    int32_t                 retVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pCorRegs    =   (CSL_Bcp_corRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_COR]);

    switch(colNum)
    {
        case 0:
            retVal = CSL_FEXT (pCorRegs->M0, BCP_COR_M0_MOD_EN);
            break;
        case 1:
            retVal = CSL_FEXT (pCorRegs->M1, BCP_COR_M1_MOD_EN);
            break;
        case 2:
            retVal = CSL_FEXT (pCorRegs->M2, BCP_COR_M2_MOD_EN);
            break;
        case 3:
            retVal = CSL_FEXT (pCorRegs->M3, BCP_COR_M3_MOD_EN);
            break;
        case 4:
            retVal = CSL_FEXT (pCorRegs->M4, BCP_COR_M4_MOD_EN);
            break;
        case 5:
            retVal = CSL_FEXT (pCorRegs->M5, BCP_COR_M5_MOD_EN);
            break;
        case 6:
            retVal = CSL_FEXT (pCorRegs->M6, BCP_COR_M6_MOD_EN);
            break;
        case 7:
            retVal = CSL_FEXT (pCorRegs->M7, BCP_COR_M7_MOD_EN);
            break;
        case 8:
            retVal = CSL_FEXT (pCorRegs->M8, BCP_COR_M8_MOD_EN);
            break;
        case 9:
            retVal = CSL_FEXT (pCorRegs->M9, BCP_COR_M9_MOD_EN);
            break;
        case 10:
            retVal = CSL_FEXT (pCorRegs->M10, BCP_COR_M10_MOD_EN);
            break;
        case 11:
            retVal = CSL_FEXT (pCorRegs->M11, BCP_COR_M11_MOD_EN);
            break;
        case 12:
            retVal = CSL_FEXT (pCorRegs->M12, BCP_COR_M12_MOD_EN);
            break;
        default:
            retVal = 0;          
            break;
    }

    return retVal;
}

/**
 * ============================================================================
 *  @n@b Bcp_setCorReedMullerTableColumn
 *
 *  @b  brief
 *  @n  This API populates the contents of M register. Each of the M register 
 *      corresponds to a Reed Muller Table column.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        colNum              Column number to be updated.

 *  @param[in]    
        mVal                Value to configure in the register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                      0  -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  M register corresponding to column number specified configured.
 * ============================================================================
 */
static inline int32_t Bcp_setCorReedMullerTableColumn
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                colNum,
    uint32_t                mVal
)
{
    CSL_Bcp_corRegs*        pCorRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pCorRegs    =   (CSL_Bcp_corRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_COR]);

    switch(colNum)
    {
        case 0:
            CSL_FINS (pCorRegs->M0, BCP_COR_M0_MOD_EN, mVal);
            break;
        case 1:
            CSL_FINS (pCorRegs->M1, BCP_COR_M1_MOD_EN, mVal);
            break;
        case 2:
            CSL_FINS (pCorRegs->M2, BCP_COR_M2_MOD_EN, mVal);
            break;
        case 3:
            CSL_FINS (pCorRegs->M3, BCP_COR_M3_MOD_EN, mVal);
            break;
        case 4:
            CSL_FINS (pCorRegs->M4, BCP_COR_M4_MOD_EN, mVal);
            break;
        case 5:
            CSL_FINS (pCorRegs->M5, BCP_COR_M5_MOD_EN, mVal);
            break;
        case 6:
            CSL_FINS (pCorRegs->M6, BCP_COR_M6_MOD_EN, mVal);
            break;
        case 7:
            CSL_FINS (pCorRegs->M7, BCP_COR_M7_MOD_EN, mVal);
            break;
        case 8:
            CSL_FINS (pCorRegs->M8, BCP_COR_M8_MOD_EN, mVal);
            break;
        case 9:
            CSL_FINS (pCorRegs->M9, BCP_COR_M9_MOD_EN, mVal);
            break;
        case 10:
            CSL_FINS (pCorRegs->M10, BCP_COR_M10_MOD_EN, mVal);
            break;
        case 11:
            CSL_FINS (pCorRegs->M11, BCP_COR_M11_MOD_EN, mVal);
            break;
        case 12:
            CSL_FINS (pCorRegs->M12, BCP_COR_M12_MOD_EN, mVal);
            break;
        default:            
            break;
    }

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getCorControlReg
 *
 *  @b  brief
 *  @n  This API returns the contents of Correlation engine control register. 
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        pQpskMap00          Value read from QPSK Map 00 field.

 *  @param[out]    
        pQpskMap01          Value read from QPSK Map 01 field.

 *  @param[out]    
        pQpskMap10          Value read from QPSK Map 10 field.

 *  @param[out]    
        pQpskMap11          Value read from QPSK Map 11 field.

 *  @return     int32_t     
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success. Correlation engine control
 *                                  contents loaded into the output params passed.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getCorControlReg
(
    Bcp_LldObj*             pBcpLldObj,
    uint8_t*                pQpskMap00,
    uint8_t*                pQpskMap01,
    uint8_t*                pQpskMap10,
    uint8_t*                pQpskMap11
)
{
    CSL_Bcp_corRegs*        pCorRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pQpskMap00 || !pQpskMap01 || !pQpskMap10 || !pQpskMap11)
        return -1;

    pCorRegs    =   (CSL_Bcp_corRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_COR]);

    *pQpskMap00 =   CSL_FEXT (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K10_C2K0);
    *pQpskMap01 =   CSL_FEXT (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K10_C2K1);
    *pQpskMap10 =   CSL_FEXT (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K11_C2K0);
    *pQpskMap11 =   CSL_FEXT (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K11_C2K1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setCorControlReg
 *
 *  @b  brief
 *  @n  This API populates the contents of Correlation engine control register. 
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        qpskMap00           Value to be written to QPSK Map 00 field.

 *  @param[out]    
        qpskMap01           Value to be written to QPSK Map 01 field.

 *  @param[out]    
        qpskMap10           Value to be written to QPSK Map 10 field.

 *  @param[out]    
        qpskMap11           Value to be written to QPSK Map 11 field.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Correlation engine Control register 
 *                                  succesfully configured.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  Correlation engine control register CONTROL configured.
 * ============================================================================
 */
static inline int32_t Bcp_setCorControlReg
(
    Bcp_LldObj*             pBcpLldObj,
    uint8_t                 qpskMap00,
    uint8_t                 qpskMap01,
    uint8_t                 qpskMap10,
    uint8_t                 qpskMap11
)
{
    CSL_Bcp_corRegs*        pCorRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pCorRegs    =   (CSL_Bcp_corRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_COR]);

    CSL_FINS (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K10_C2K0, qpskMap00);
    CSL_FINS (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K10_C2K1, qpskMap01);
    CSL_FINS (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K11_C2K0, qpskMap10);
    CSL_FINS (pCorRegs->CONTROL_REGISTER, BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K11_C2K1, qpskMap11);

    return 0;
}

/****************** Traffic Manager MMR ******************/

/**
 * ============================================================================
 *  @n@b Bcp_getPIDReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP PID Register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        pPIDParams          BCP PID Params object handle.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of BCP PID register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getPIDReg 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_PeripheralIdParams* pPIDParams
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pPIDParams)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    pPIDParams->minorNum    =   CSL_FEXT (pTmRegs->BCP_PID, BCP_TM_BCP_PID_BCP_PID_MINOR);
    pPIDParams->customNum   =   CSL_FEXT (pTmRegs->BCP_PID, BCP_TM_BCP_PID_BCP_PID_CUSTOM);
    pPIDParams->majorNum    =   CSL_FEXT (pTmRegs->BCP_PID, BCP_TM_BCP_PID_BCP_PID_MAJOR);
    pPIDParams->rtlVersion  =   CSL_FEXT (pTmRegs->BCP_PID, BCP_TM_BCP_PID_BCP_PID_RTL);
    pPIDParams->function    =   CSL_FEXT (pTmRegs->BCP_PID, BCP_TM_BCP_PID_BCP_PID_FUNC);
    pPIDParams->scheme      =   CSL_FEXT (pTmRegs->BCP_PID, BCP_TM_BCP_PID_BCP_PID_SCHEME);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_doSoftwareReset
 *
 *  @b  brief
 *  @n  This API configures the BCP Soft Reset Register element "soft_rst" to
 *      perform a software reset on the entire BCP engine other than the
 *      CDMAHP and VBUSM interfaces.
 *     
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.
 *     
 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                      0  -   Software reset succesfully initiated.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API.
 *
 *  @post
 *  @n  BCP Soft Reset Register's 'software reset' bit configured
 * ============================================================================
 */
static inline int32_t Bcp_doSoftwareReset 
(
    Bcp_LldObj*                pBcpLldObj
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    CSL_FINS (pTmRegs->BCP_SOFT_RESET, BCP_TM_BCP_SOFT_RESET_SOFT_RESET, 1);          

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_doSoftResetRxTeardown
 *
 *  @b  brief
 *  @n  This API configures the BCP Soft Reset Register element "Rx_teardown"
 *      to stop processing any packets in Rx path. If a port had an active
 *      packet it will send an EOP word to the QFIFO to finish off the packet
 *      to the CDMAHP.
 *     
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.
 *     
 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                      0  -   Rx Teardown succesfully initiated.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API.
 *
 *  @post
 *  @n  BCP Soft Reset Register's bits configured as passed.
 * ============================================================================
 */
static inline int32_t Bcp_doSoftResetRxTeardown
(
    Bcp_LldObj*                pBcpLldObj
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    CSL_FINS (pTmRegs->BCP_SOFT_RESET, BCP_TM_BCP_SOFT_RESET_RX_TEARDOWN, 1);          

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_doSoftResetTxTeardown
 *
 *  @b  brief
 *  @n  This API configures the BCP Soft Reset Register element "Tx_teardown"
 *      to stop passing CDMAHP data to the QFIFOs in Tx path. Any CDMAHP Tx
 *      queue that has data will be read with the data dropped so the CDMAHP
 *      can process the full packet.
 *     
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.
 *     
 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                      0  -   Tx Teardown succesfully initiated.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API.
 *
 *  @post
 *  @n  BCP Soft Reset Register's bits configured as passed.
 * ============================================================================
 */
static inline int32_t Bcp_doSoftResetTxTeardown
(
    Bcp_LldObj*                pBcpLldObj
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    CSL_FINS (pTmRegs->BCP_SOFT_RESET, BCP_TM_BCP_SOFT_RESET_TX_TEARDOWN, 1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_doSoftResetClockGateDisable
 *
 *  @b  brief
 *  @n  This API configures the BCP Soft Reset Register element "clkgate_disable"
 *      to disable the clock gating done by PPBs when the engines are idle.
 *     
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.
 *     
 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                      0  -   Clock Gate disable succesfully initiated.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API.
 *
 *  @post
 *  @n  BCP Soft Reset Register's bits configured as passed.
 * ============================================================================
 */
static inline int32_t Bcp_doSoftResetClockGateDisable
(
    Bcp_LldObj*                pBcpLldObj
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    CSL_FINS (pTmRegs->BCP_SOFT_RESET, BCP_TM_BCP_SOFT_RESET_CLKGATE_DISABLE, 1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_clearTMHalt
 *
 *  @b  brief
 *  @n  Given a BCP Tx Queue Number, this API sets the "clear_halt" bit 
 *      corresponding to it in the TM_HALT_CTRL register. This in turn
 *      clears the halt for the corresponding queue.
 *     
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        txQNum              BCP Tx Queue Number for which the TM halt bit must
                            be cleared.
 *     
 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                      0  -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API.
 *
 *  @post
 *  @n  BCP TM Halt Control Register's 'clear_halt' bit set to initiate a clear operation.
 * ============================================================================
 */
static inline int32_t Bcp_clearTMHalt 
(
    Bcp_LldObj*                 pBcpLldObj,
    Bcp_QueueId                 txQNum
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    CSL_FINSR (pTmRegs->TM_HALT_CTRL, txQNum, txQNum, 1);          

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getTMHaltStatus
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP TM Halt Status register (TM_HALT_STATUS).
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of TM_HALT_STATUS register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getTMHaltStatus 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    return CSL_FEXT (pTmRegs->TM_HALT_STATUS, BCP_TM_TM_HALT_STATUS_HALT_STATUS);
}

/**
 * ============================================================================
 *  @n@b Bcp_getEmulationClockStopStatus
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP Emulation clock stop status register.
 *      The contents of register indicate the idle status of each of the BCP 
 *      sub-modules.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        pIdleStatus         Structure to be filled with the Idle status read 
                            from H/w.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEmulationClockStopStatus 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleIdleStatus*   pIdleStatus
)
{
    CSL_Bcp_tmRegs*         pTmRegs;
    uint32_t                word;

    if (!pBcpLldObj || !pIdleStatus || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    word    =   pTmRegs->BCP_EMU_CLKSTOP_STATUS;

    pIdleStatus->tm_idle_status     =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_TM_ENG_IDLE);
    pIdleStatus->dio_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_DIO_ENG_IDLE);
    pIdleStatus->crc_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_CRC_ENG_IDLE);
    pIdleStatus->enc_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_ENC_ENG_IDLE);
    pIdleStatus->rm_idle_status     =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_RM_ENG_IDLE);
    pIdleStatus->mod_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_MOD_ENG_IDLE);
    pIdleStatus->int_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_INT_ENG_IDLE);
    pIdleStatus->cor_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_COR_ENG_IDLE);
    pIdleStatus->ssl_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_SSL_ENG_IDLE);
    pIdleStatus->rd_idle_status     =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_RD_ENG_IDLE);
    pIdleStatus->dnt_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_DNT_ENG_IDLE);
    pIdleStatus->all_idle_status    =   CSL_FEXT (word, BCP_TM_BCP_EMU_CLKSTOP_STATUS_ALL_ENG_IDLE);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEmulationClockStopControlReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP Emulation clock control stop register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        pEmuRtSel           Holds emulation suspend signal selection bit.

 *  @param[out]    
        pEmuFreeRun         Indicates if the hardware responds to/ignores the emulation
                            suspend signal.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEmulationClockStopControlReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t*                        pEmuRtSel,
    uint8_t*                        pEmuFreeRun
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pEmuRtSel || !pEmuFreeRun)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    *pEmuRtSel  =   CSL_FEXT (pTmRegs->BCP_EMU_CLKSTOP_CTRL, BCP_TM_BCP_EMU_CLKSTOP_CTRL_BCP_EMU_RT_SEL);
    *pEmuFreeRun=   CSL_FEXT (pTmRegs->BCP_EMU_CLKSTOP_CTRL, BCP_TM_BCP_EMU_CLKSTOP_CTRL_BCP_EMU_FREERUN);


    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setEmulationClockStopControlReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of BCP Emulation clock control stop register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        emuRtSel            Emulation suspend signal selection to configure. Set to
                            0 for BCP to monitor "emu_dbgsusp" signal and 1 for 
                            BCP to monitor "emu_dbgsusp_rt" signal.

 *  @param[in]    
        emuFreeRun          Set to 0 for BCP to act upon the emulation suspend
                            signal and to 1 for BCP to ignore the signal and 
                            run normally.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_setEmulationClockStopControlReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         emuRtSel,
    uint8_t                         emuFreeRun
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->BCP_EMU_CLKSTOP_CTRL   =   CSL_FMK (BCP_TM_BCP_EMU_CLKSTOP_CTRL_BCP_EMU_RT_SEL, emuRtSel) |
                                        CSL_FMK (BCP_TM_BCP_EMU_CLKSTOP_CTRL_BCP_EMU_FREERUN, emuFreeRun);

    return 0;
}


/**
 * ============================================================================
 *  @n@b Bcp_getDlgSwTimeStampReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP Data logger Software time stamp
 *      register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of BCP_DLG_SW_TIMESTAMP register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  BCP_DLG_SW_TIMESTAMP register configured.
 * ============================================================================
 */
static inline int32_t Bcp_getDlgSwTimeStampReg 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    return CSL_FEXT (pTmRegs->BCP_DLG_SW_TIMESTAMP, BCP_TM_BCP_DLG_SW_TIMESTAMP_BCP_DLG_SW_TIMESTAMP);
}

/**
 * ============================================================================
 *  @n@b Bcp_setDlgSwTimeStampReg
 *
 *  @b  brief
 *  @n  This API populates the contents of BCP Data logger Software time stamp
 *      register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        tsVal               Software timestamp value to configure in BCP_DLG_SW_TIMESTAMP
                            register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_setDlgSwTimeStampReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint32_t                        tsVal
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->BCP_DLG_SW_TIMESTAMP, BCP_TM_BCP_DLG_SW_TIMESTAMP_BCP_DLG_SW_TIMESTAMP, tsVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getDlgHwTimeStampReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP Data logger Hardware time stamp
 *      register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of BCP_DLG_HW_TIMESTAMP register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getDlgHwTimeStampReg 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    return CSL_FEXT (pTmRegs->BCP_DLG_HW_TIMESTAMP, BCP_TM_BCP_DLG_HW_TIMESTAMP_BCP_DLG_HW_TIMESTAMP);
}

/**
 * ============================================================================
 *  @n@b Bcp_resetDlgHwTimeStampReg
 *
 *  @b  brief
 *  @n  This API initiates a reset on the Hardware timestamp control register by
 *      writing 1 to the BCP_DLG_HW_TIMESTAMP_CTRL register. 
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success. Hardware timestamp register contents
 *                                  reset to 0.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  Hardware timestamp register BCP_DLG_HW_TIMESTAMP reset to 0.
 * ============================================================================
 */
static inline int32_t Bcp_resetDlgHwTimeStampReg 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->BCP_DLG_HW_TIMESTAMP_CTRL, BCP_TM_BCP_DLG_HW_TIMESTAMP_CTRL_BCP_DLG_HW_TIMESTAMP_RUN, 1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setDlgHold
 *
 *  @b  brief
 *  @n  This API puts data logger in hold state (DLG_HOLD) for all the BCP 
 *      sub modules specified. It does so by writing 1 to the 'hold' bit in the 
 *      BCP_DLG_HOLD_RST register corresponding to each of the modules that need
 *      to be put in hold.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        pDlgHoldCfg         Hold bit configuration for all the modules. To put
                            a specific module in hold, set its corresponding bit
                            in the structure to 1 and set to 0 otherwise.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed/Hold
 *                                  configuration handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  'hold' bits set to 1 in BCP_DLG_HOLD_RST register according to 
 *      configuration specified.
 * ============================================================================
 */
static inline int32_t Bcp_setDlgHold 
(
    Bcp_LldObj*                     pBcpLldObj,
    Bcp_ModuleCfg*                  pDlgHoldCfg
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;
    uint16_t                        holdVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pDlgHoldCfg)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    holdVal =   CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_TM_DLG_HOLD, pDlgHoldCfg->tm) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_DIO_DLG_HOLD, pDlgHoldCfg->dio) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_CRC_DLG_HOLD, pDlgHoldCfg->crc) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_ENC_DLG_HOLD, pDlgHoldCfg->enc) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_RM_DLG_HOLD, pDlgHoldCfg->rm) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_MOD_DLG_HOLD, pDlgHoldCfg->mod) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_INT_DLG_HOLD, pDlgHoldCfg->intleaver) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_COR_DLG_HOLD, pDlgHoldCfg->cor) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_SSL_DLG_HOLD, pDlgHoldCfg->ssl) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_RD_DLG_HOLD, pDlgHoldCfg->rd) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_DNT_DLG_HOLD, pDlgHoldCfg->dnt);
        
    CSL_FINSR (pTmRegs->BCP_DLG_HOLD_RST, 26, 16, holdVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getDlgHoldStatus
 *
 *  @b  brief
 *  @n  This API returns the data logger hold status for all BCP submodules by 
 *      reading the 'hold' bits from the BCP_DLG_HOLD_RST register.
 *      
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        pDlgHoldStatus      Hold bit configuration read from H/w for BCP submodules. 
                            

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed/Invalid 
 *                                  DLG configuration handle passed.
 *                                  
 *  @li                     =0  -   Success. Data logger hold status passed back in 
 *                                  the handle specified.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getDlgHoldStatus 
(
    Bcp_LldObj*                     pBcpLldObj,
    Bcp_ModuleCfg*                  pDlgHoldStatus
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;
    uint16_t                        holdVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pDlgHoldStatus)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    holdVal =   CSL_FEXTR (pTmRegs->BCP_DLG_HOLD_RST, 26, 16);
            
    pDlgHoldStatus->tm          =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_TM_DLG_HOLD);
    pDlgHoldStatus->dio         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_DIO_DLG_HOLD);
    pDlgHoldStatus->crc         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_CRC_DLG_HOLD);
    pDlgHoldStatus->enc         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_ENC_DLG_HOLD);
    pDlgHoldStatus->rm          =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_RM_DLG_HOLD);
    pDlgHoldStatus->mod         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_MOD_DLG_HOLD);
    pDlgHoldStatus->intleaver   =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_INT_DLG_HOLD);
    pDlgHoldStatus->cor         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_COR_DLG_HOLD);
    pDlgHoldStatus->ssl         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_SSL_DLG_HOLD);
    pDlgHoldStatus->rd          =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_RD_DLG_HOLD);
    pDlgHoldStatus->dnt         =   CSL_FEXT (holdVal, BCP_TM_BCP_DLG_HOLD_RST_DNT_DLG_HOLD);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setDlgIdle
 *
 *  @b  brief
 *  @n  This API sets up data logger in idle state (DLG_IDLE) as per the Idle
 *      configuration specified for all BCP submodules. It does so by writing 1 
 *      to the 'reset' bit in the BCP_DLG_HOLD_RST register corresponding to 
 *      each of the modules that need to be put in idle state.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        pDlgIdleCfg         Idle bit configuration for all the modules. To put
                            a specific module in idle state, set its corresponding 
                            bit in the structure to 1 and set to 0 otherwise.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed/DLG Idle state
 *                                  configuration handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  Corresponding 'reset' bit set to 1 in BCP_DLG_HOLD_RST register.
 * ============================================================================
 */
static inline int32_t Bcp_setDlgIdle 
(
    Bcp_LldObj*                     pBcpLldObj,
    Bcp_ModuleCfg*                  pDlgIdleCfg
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;
    uint16_t                        idleVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pDlgIdleCfg)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    idleVal =   CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_TM_DLG_RST, pDlgIdleCfg->tm) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_DIO_DLG_RST, pDlgIdleCfg->dio) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_CRC_DLG_RST, pDlgIdleCfg->crc) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_ENC_DLG_RST, pDlgIdleCfg->enc) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_RM_DLG_RST, pDlgIdleCfg->rm) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_MOD_DLG_RST, pDlgIdleCfg->mod) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_INT_DLG_RST, pDlgIdleCfg->intleaver) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_COR_DLG_RST, pDlgIdleCfg->cor) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_SSL_DLG_RST, pDlgIdleCfg->ssl) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_RD_DLG_RST, pDlgIdleCfg->rd) |
                CSL_FMK (BCP_TM_BCP_DLG_HOLD_RST_DNT_DLG_RST, pDlgIdleCfg->dnt);
        
    CSL_FINSR (pTmRegs->BCP_DLG_HOLD_RST, 10, 0, idleVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getDlgIdleStatus
 *
 *  @b  brief
 *  @n  This API returns the data logger idle status for all BCP submodules by
 *      reading the 'reset' bit from the BCP_DLG_HOLD_RST register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        pDlgIdleStatus      Reset/Idle bit configuration read from H/w for all BCP 
                            submodules.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed/Invalid DLG
 *                                  Idle status handle passed.
 *  @li                     =0  -   Data logger idle status returned in the handle
 *                                  passed.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getDlgIdleStatus 
(
    Bcp_LldObj*                     pBcpLldObj,
    Bcp_ModuleCfg*                  pDlgIdleStatus
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;
    uint16_t                        idleVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pDlgIdleStatus)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    idleVal =   CSL_FEXTR (pTmRegs->BCP_DLG_HOLD_RST, 10, 0);
        
    pDlgIdleStatus->tm          =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_TM_DLG_RST);
    pDlgIdleStatus->dio         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_DIO_DLG_RST);
    pDlgIdleStatus->crc         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_CRC_DLG_RST);
    pDlgIdleStatus->enc         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_ENC_DLG_RST);
    pDlgIdleStatus->rm          =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_RM_DLG_RST);
    pDlgIdleStatus->mod         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_MOD_DLG_RST);
    pDlgIdleStatus->intleaver   =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_INT_DLG_RST);
    pDlgIdleStatus->cor         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_COR_DLG_RST);
    pDlgIdleStatus->ssl         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_SSL_DLG_RST);
    pDlgIdleStatus->rd          =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_RD_DLG_RST);
    pDlgIdleStatus->dnt         =   CSL_FEXT (idleVal, BCP_TM_BCP_DLG_HOLD_RST_DNT_DLG_RST);
        
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setDlgHoldResetReg
 *
 *  @b  brief
 *  @n  This API can be used to configure the hold and reset/idle settings in 
 *      BCP_DLG_HOLD_RST register for all BCP modules at once.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        holdResetVal        32-bit value to configure to the BCP Data logger Hold
                            Reset register.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  BCP_DLG_HOLD_RST register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_setDlgHoldResetReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint32_t                        holdResetVal
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->BCP_DLG_HOLD_RST    =   holdResetVal;

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getDlgHoldResetReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP_DLG_HOLD_RST register. This API
 *      can be used to retrieve the hold and idle status of all BCP modules
 *      at once.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of BCP_DLG_HOLD_RST register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getDlgHoldResetReg 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    return pTmRegs->BCP_DLG_HOLD_RST;
}

/**
 * ============================================================================
 *  @n@b Bcp_setCdmaHpSrcId
 *
 *  @b  brief
 *  @n  This API sets up the Source Id for CDMAHP Info word.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        srcId               Source Id to use for BCP CDMAHP.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_setCdmaHpSrcId 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         srcId
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_CDMAHP_SRC_ID, srcId);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getCdmaHpSrcId
 *
 *  @b  brief
 *  @n  This API returns the Source Id setup for BCP CDMAHP.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Source Id read from TM_CONTROL register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getCdmaHpSrcId 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    return CSL_FEXT (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_CDMAHP_SRC_ID);
}

/**
 * ============================================================================
 *  @n@b Bcp_enableCdmaHp
 *
 *  @b  brief
 *  @n  This API enables BCP CDMAHP.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed. BCP CDMAHP turned on.
 * ============================================================================
 */
static inline int32_t Bcp_enableCdmaHp 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_CDMAHP_DISABLE, 0);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_disableCdmaHp
 *
 *  @b  brief
 *  @n  This API disables BCP CDMAHP, i.e., all signals from CDMAHP are masked off.
 *      Suitable for using the BCP CDMAHP in loopback mode.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed. BCP CDMAHP turned off.
 * ============================================================================
 */
static inline int32_t Bcp_disableCdmaHp 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_CDMAHP_DISABLE, 1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_isCdmaHpEnabled
 *
 *  @b  brief
 *  @n  This API returns 1 if BCP CDMA HP is enabled, 0 otherwise.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0/1 -   Reads CDMAHP enable status from TM_CONTROL register.
 *                                  Returns 0 to indicate CDMAHP is disabled and 1
 *                                  to indicate it is enabled.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_isCdmaHpEnabled 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    uint8_t                         bIsCdmaHpDisabled;        
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    bIsCdmaHpDisabled = CSL_FEXT (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_CDMAHP_DISABLE);

    return (bIsCdmaHpDisabled == 0 ? 1 : 0);
}

/**
 * ============================================================================
 *  @n@b Bcp_enableForcePayloadEndian
 *
 *  @b  brief
 *  @n  This API enables forcing the payliod to follow the endian swapping
 *      settings regardless if it is in big or little endian.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_enableForcePayloadEndian 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_FRC_PAYLOAD_ENDIAN, 1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_disableForcePayloadEndian
 *
 *  @b  brief
 *  @n  This API clears forcing the payliod to follow the endian swapping
 *      settings regardless if it is in big or little endian.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_disableForcePayloadEndian 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_FRC_PAYLOAD_ENDIAN, 0);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_isForcePayloadEndianEnabled
 *
 *  @b  brief
 *  @n  This API returns the value read from the "frc_payload_enable" field in
 *      the TM_CONTROL registration.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0/1 -   Value read from register field.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_isForcePayloadEndianEnabled 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    return CSL_FEXT (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_FRC_PAYLOAD_ENDIAN);
}

/**
 * ============================================================================
 *  @n@b Bcp_enableTxCdmaHpReadArbPrio
 *
 *  @b  brief
 *  @n  This API enables raising priority of Tx CDMAHP paths with active packets.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_enableTxCdmaHpReadArbPrio 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_TX_CDMAHP_RD_ARB_HPRIORITY, 1);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_disableTxCdmaHpReadArbPrio
 *
 *  @b  brief
 *  @n  This API disables raising priority of Tx CDMAHP paths with active packets.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_disableTxCdmaHpReadArbPrio 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    CSL_FINS (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_TX_CDMAHP_RD_ARB_HPRIORITY, 0);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_isTxCdmaHpReadArbPrioEnabled
 *
 *  @b  brief
 *  @n  This API returns 1 if BCP Tx CDMAHP read arbitration priority is enabled.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0/1 -   Reads Tx CDMAHP read arbitration priority enable 
 *                                  status from TM_CONTROL register.
 *                                  Returns 0 to indicate priority raise disabled and 1
 *                                  to indicate it is enabled.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_isTxCdmaHpReadArbPrioEnabled 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    return CSL_FEXT (pTmRegs->TM_CONTROL, BCP_TM_TM_CONTROL_TX_CDMAHP_RD_ARB_HPRIORITY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setTMControlReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of TM_CONTROL register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        srcId               Source Id to use for BCP CDMAHP.

 *  @param[in]    
        bCdmaDisable        Set to 1 to disable BCP CDMAHP and 0 otherwise.

 *  @param[in]    
        bFrcPayloadEndianDisable  Set to 1 to enable raising priority of Rx QFIFO and 
                            0 otherwise.

 *  @param[in]    
        bTxRdArbPrioEnable  Set to 1 to enable raising priority of Tx CDMAHP paths and 
                            0 otherwise.


 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_CONTROL register configured with the value passed.
 * ============================================================================
 */
static inline int32_t Bcp_setTMControlReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         srcId,
    uint8_t                         bCdmaDisable,
    uint8_t                         bFrcPayloadEndianDisable,
    uint8_t                         bTxRdArbPrioEnable
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->TM_CONTROL     =   CSL_FMK (BCP_TM_TM_CONTROL_CDMAHP_SRC_ID, srcId) |
                                CSL_FMK (BCP_TM_TM_CONTROL_CDMAHP_DISABLE, bCdmaDisable) |
                                CSL_FMK (BCP_TM_TM_CONTROL_FRC_PAYLOAD_ENDIAN, bFrcPayloadEndianDisable) |
                                CSL_FMK (BCP_TM_TM_CONTROL_TX_CDMAHP_RD_ARB_HPRIORITY, bTxRdArbPrioEnable);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getTMControlReg
 *
 *  @b  brief
 *  @n  This API returns the contents of the TM_CONTROL register.
 *
 *  @param[in]    
        pBcpLldObj              BCP LLD instance object.

 *  @param[out]    
        pSrcId                  BCP CDMAHP Source Id.

 *  @param[out]    
        pIsCdmaDisabled         CDMAHP disable status. 0 returned when CDMAHP enabled
                                and 1 otherwise.

 *  @param[out]    
        pIsFrcPayloadEndianDisabled   Indicates if Rx priority raise is enabled.

 *  @param[out]    
        pIsTxRdArbPrioEnabled   Indicates if Tx priority raise is enabled.


 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Succesfully populated output params.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getTMControlReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t*                        pSrcId,
    uint8_t*                        pIsCdmaDisabled,
    uint8_t*                        pIsFrcPayloadEndianDisabled,
    uint8_t*                        pIsTxRdArbPrioEnabled
)
{
    uint32_t                        tmpWord;        
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pSrcId || !pIsCdmaDisabled || !pIsFrcPayloadEndianDisabled || !pIsTxRdArbPrioEnabled)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->TM_CONTROL;
    
    *pSrcId                 =   CSL_FEXT (tmpWord, BCP_TM_TM_CONTROL_CDMAHP_SRC_ID);
    *pIsCdmaDisabled        =   CSL_FEXT (tmpWord, BCP_TM_TM_CONTROL_CDMAHP_DISABLE);
    *pIsFrcPayloadEndianDisabled  =   CSL_FEXT (tmpWord, BCP_TM_TM_CONTROL_FRC_PAYLOAD_ENDIAN);
    *pIsTxRdArbPrioEnabled  =   CSL_FEXT (tmpWord, BCP_TM_TM_CONTROL_TX_CDMAHP_RD_ARB_HPRIORITY);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setTxCdmaHpReadPrioReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of TM_TX_CDMAHP_READ_PRIORITY register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        prioVal             Array of priority values to configure for reading CDMAHP for
                            QFIFOs 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_TX_CDMAHP_READ_PRIORITY register configured with the values passed.
 * ============================================================================
 */
static inline int32_t Bcp_setTxCdmaHpReadPrioReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         prioVal[8]
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->TM_TX_CDMAHP_READ_PRIORITY  =    
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_0, prioVal[0]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_1, prioVal[1]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_2, prioVal[2]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_3, prioVal[3]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_4, prioVal[4]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_5, prioVal[5]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_6, prioVal[6]) |
                    CSL_FMK (BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_7, prioVal[7]);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getTxCdmaHpReadPrioReg
 *
 *  @b  brief
 *  @n  This API returns the contents of TM_TX_CDMAHP_READ_PRIORITY register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        prioVal             CDMAHP read priorities for QFIFOs 0..7 returned from 
                            hardware.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getTxCdmaHpReadPrioReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         prioVal[8]
)
{
    uint32_t                        tmpWord;
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->TM_TX_CDMAHP_READ_PRIORITY;

    prioVal[0]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_0);
    prioVal[1]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_1);
    prioVal[2]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_2);
    prioVal[3]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_3);
    prioVal[4]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_4);
    prioVal[5]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_5);
    prioVal[6]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_6);
    prioVal[7]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_CDMAHP_READ_PRIORITY_TX_CDMAHP_RD_ARB_PRIORITY_7);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setTxQfifoReadDestSelReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of TM_TX_QFIFO_RD_DEST_SEL register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        destSel             Destination select values for QFIFOs 0..7

 *  @param[in]    
        prioVal             Priority values to configure for reading QFIFOs 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_TX_QFIFO_RD_DEST_SEL register configured with the values passed.
 * ============================================================================
 */
static inline int32_t Bcp_setTxQfifoReadDestSelReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         destSel[8],
    uint8_t                         prioVal[8]
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->TM_TX_QFIFO_RD_DEST_SEL  =    
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_0, destSel[0]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_1, destSel[1]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_2, destSel[2]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_3, destSel[3]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_4, destSel[4]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_5, destSel[5]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_6, destSel[6]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_7, destSel[7]) |

                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_0, prioVal[0]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_1, prioVal[1]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_2, prioVal[2]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_3, prioVal[3]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_4, prioVal[4]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_5, prioVal[5]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_6, prioVal[6]) |
                    CSL_FMK (BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_7, prioVal[7]);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getTxQfifoReadDestSelReg
 *
 *  @b  brief
 *  @n  This API returns the contents of TM_TX_QFIFO_RD_DEST_SEL register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        destSel             Destination select values for QFIFOs 0..7

 *  @param[out]    
        prioVal             Priority values for reading QFIFOs 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getTxQfifoReadDestSelReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         destSel[8],
    uint8_t                         prioVal[8]
)
{
    uint32_t                        tmpWord;
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->TM_TX_QFIFO_RD_DEST_SEL;

    destSel[0]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_0);
    destSel[1]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_1);
    destSel[2]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_2);
    destSel[3]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_3);
    destSel[4]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_4);
    destSel[5]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_5);
    destSel[6]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_6);
    destSel[7]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_7);

    prioVal[0]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_0);
    prioVal[1]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_1);
    prioVal[2]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_2);
    prioVal[3]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_3);
    prioVal[4]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_4);
    prioVal[5]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_5);
    prioVal[6]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_6);
    prioVal[7]  =   CSL_FEXT (tmpWord, BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_ARB_PRIORITY_7);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setRxQFifoWriteArbPrioReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of TM_RX_QFIFO_WR_ARB_PRIORITY_3_0 and
 *      TM_RX_QFIFO_WR_ARB_PRIORITY_7_4 registers. This API sets priority for 
 *      data from each PPB to the 8 BCP Tx queues.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        prioVal             Priority values for writing QFIFO 0...7 from PPB 0..3

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_RX_QFIFO_WR_ARB_PRIORITY_3_0 and TM_RX_QFIFO_WR_ARB_PRIORITY_7_4  
 *      registers configured with the values passed.
 * ============================================================================
 */
static inline int32_t Bcp_setRxQFifoWriteArbPrioReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         prioVal[4][8]
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->TM_RX_QFIFO_WR_ARB_PRIORITY_3_0  =    
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_0, prioVal[0][0]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_1, prioVal[1][0]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_2, prioVal[2][0]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_3, prioVal[3][0]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_0, prioVal[0][1]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_1, prioVal[1][1]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_2, prioVal[2][1]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_3, prioVal[3][1]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_0, prioVal[0][2]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_1, prioVal[1][2]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_2, prioVal[2][2]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_3, prioVal[3][2]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_0, prioVal[0][3]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_1, prioVal[1][3]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_2, prioVal[2][3]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_3, prioVal[3][3]);

    pTmRegs->TM_RX_QFIFO_WR_ARB_PRIORITY_7_4  =    
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_0, prioVal[0][4]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_1, prioVal[1][4]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_2, prioVal[2][4]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_3, prioVal[3][4]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_0, prioVal[0][5]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_1, prioVal[1][5]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_2, prioVal[2][5]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_3, prioVal[3][5]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_0, prioVal[0][6]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_1, prioVal[1][6]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_2, prioVal[2][6]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_3, prioVal[3][6]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_0, prioVal[0][7]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_1, prioVal[1][7]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_2, prioVal[2][7]) |
                    CSL_FMK (BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_3, prioVal[3][7]);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getRxQFifoWriteArbPrioReg
 *
 *  @b  brief
 *  @n  This API returns the contents of TM_RX_QFIFO_WR_ARB_PRIORITY_3_0 and
 *      TM_RX_QFIFO_WR_ARB_PRIORITY_7_4 registers. This API retrieves priorities 
 *      configured for data from each PPB to the 8 BCP Tx queues.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        prioVal             Priority values for writing QFIFO 0...7 from PPB 0..3

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getRxQFifoWriteArbPrioReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         prioVal[4][8]
)
{
    uint32_t                        tmpWord [2];        
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs         =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord [0]     =   pTmRegs->TM_RX_QFIFO_WR_ARB_PRIORITY_3_0;
    tmpWord [1]     =   pTmRegs->TM_RX_QFIFO_WR_ARB_PRIORITY_7_4;

    prioVal[0][0]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_0);
    prioVal[1][0]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_1);
    prioVal[2][0]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_2);
    prioVal[3][0]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_0_WR_ARB_PRIORITY_3);
    prioVal[0][1]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_0);
    prioVal[1][1]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_1);
    prioVal[2][1]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_2);
    prioVal[3][1]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_1_WR_ARB_PRIORITY_3);
    prioVal[0][2]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_0);
    prioVal[1][2]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_1);
    prioVal[2][2]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_2);
    prioVal[3][2]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_2_WR_ARB_PRIORITY_3);
    prioVal[0][3]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_0);
    prioVal[1][3]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_1);
    prioVal[2][3]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_2);
    prioVal[3][3]   =   CSL_FEXT (tmpWord [0], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_3_0_TX_QFIFO_3_WR_ARB_PRIORITY_3);

    prioVal[0][4]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_0);
    prioVal[1][4]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_1);
    prioVal[2][4]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_2);
    prioVal[3][4]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_4_WR_ARB_PRIORITY_3);
    prioVal[0][5]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_0);
    prioVal[1][5]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_1);
    prioVal[2][5]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_2);
    prioVal[3][5]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_5_WR_ARB_PRIORITY_3);
    prioVal[0][6]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_0);
    prioVal[1][6]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_1);
    prioVal[2][6]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_2);
    prioVal[3][6]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_6_WR_ARB_PRIORITY_3);
    prioVal[0][7]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_0);
    prioVal[1][7]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_1);
    prioVal[2][7]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_2);
    prioVal[3][7]   =   CSL_FEXT (tmpWord [1], BCP_TM_TM_RX_QFIFO_WR_ARB_PRIORITY_7_4_TX_QFIFO_7_WR_ARB_PRIORITY_3);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setRxCdmaWriteArbPrioReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of TM_RX_CDMAHP_WR_ARB_PRIORITY register.
 *      This API sets up priorities for cdmahp_wr_arb writes to the
 *      CDMAHPs.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        prioVal             Priority values to configure for writing CDMAHP for
                            BCP Tx queues 0..7

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  TM_RX_CDMAHP_WR_ARB_PRIORITY register configured with the values passed.
 * ============================================================================
 */
static inline int32_t Bcp_setRxCdmaWriteArbPrioReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         prioVal[8]
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->TM_RX_CDMAHP_WR_ARB_PRIORITY  =    
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_0, prioVal[0]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_1, prioVal[1]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_2, prioVal[2]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_3, prioVal[3]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_4, prioVal[4]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_5, prioVal[5]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_6, prioVal[6]) |
                    CSL_FMK (BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_7, prioVal[7]);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getRxCdmaWriteArbPrioReg
 *
 *  @b  brief
 *  @n  This API returns the contents of TM_RX_CDMAHP_WR_ARB_PRIORITY register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[out]    
        prioVal             Priority values for writing CDMAHP for Tx queues 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getRxCdmaWriteArbPrioReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         prioVal[8]
)
{
    uint32_t                        tmpWord;
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->TM_RX_CDMAHP_WR_ARB_PRIORITY;

    prioVal[0]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_0);
    prioVal[1]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_1);
    prioVal[2]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_2);
    prioVal[3]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_3);
    prioVal[4]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_4);
    prioVal[5]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_5);
    prioVal[6]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_6);
    prioVal[7]  =   CSL_FEXT (tmpWord, BCP_TM_TM_RX_CDMAHP_WR_ARBPRIORITY_RX_CDMAHP_WR_ARB_PRIORITY_7);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getCdmaDescStarveStatus
 *
 *  @b  brief
 *  @n  This API returns the contents of Starve_Status field from the register 
 *      CDMA_DESC_STARVE_STATUS.
 *
 *  @param[in]
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]
        rxChNum             Rx Channel number for which the descriptor
                            starvation status. Takes values from 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getCdmaDescStarveStatus
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         rxChNum
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;
    int32_t                         retVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || (rxChNum > 7))
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    switch(rxChNum)
    {
        case 0:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_0);
            break;
        case 1:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_1);
            break;
        case 2:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_2);
            break;
        case 3:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_3);
            break;
        case 4:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_4);
            break;
        case 5:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_5);
            break;
        case 6:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_6);
            break;
        case 7:
            retVal = CSL_FEXT (pTmRegs->CDMA_DESC_STARVE_STATUS, BCP_TM_CDMA_DESC_STARVE_STATUS_DESC_STARVE_7);
            break;
        default:
            retVal = 0;
            break;
    }

    return retVal;
}

/**
 * ============================================================================
 *  @n@b Bcp_setCdmaDescStarveClear
 *
 *  @b  brief
 *  @n  This API force clear the specified RX channel description starvation
        in "Starve_Status" field in the register CDMA_DESC_STARVE_STATUS.
 *
 *  @param[in]
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]
        rxChNum             Rx Channel number for which the descriptor
                            starvation status. Takes values from 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_setCdmaDescStarveClear
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         rxChNum
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || (rxChNum > 7))
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    switch(rxChNum)
    {
        case 0:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_0, 1);
            break;
        case 1:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_1, 1);
            break;
        case 2:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_2, 1);
            break;
        case 3:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_3, 1);
            break;
        case 4:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_4, 1);
            break;
        case 5:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_5, 1);
            break;
        case 6:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_6, 1);
            break;
        case 7:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_CLEAR, BCP_TM_CDMA_DESC_STARVE_CLEAR_DESC_STARVE_7, 1);
            break;
        default: 
            break;
    }

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setCdmaDescStarveSet
 *
 *  @b  brief
 *  @n  This API force set the specified RX channel description starvation
        in "Starve_Status" field in the register CDMA_DESC_STARVE_STATUS.
 *
 *  @param[in]
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]
        rxChNum             Rx Channel number for which the descriptor
                            starvation status. Takes values from 0..7.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_setCdmaDescStarveSet
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         rxChNum
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || (rxChNum > 7))
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    switch(rxChNum)
    {
        case 0:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_0, 1);
            break;
        case 1:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_1, 1);
            break;
        case 2:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_2, 1);
            break;            
        case 3:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_3, 1);
            break;
        case 4:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_4, 1);
            break;
        case 5:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_5, 1);
            break;
        case 6:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_6, 1);
            break;
        case 7:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_SET, BCP_TM_CDMA_DESC_STARVE_SET_DESC_STARVE_7, 1);
            break;
        default:
            break;
    }

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setCdmaDescStarveInterruptSelect
 *
 *  @b  brief
 *  @n  This API force set the specified RX channel description starvation
        in "Starve_Status" field in the register CDMA_DESC_STARVE_INTR_SEL.
 *
 *  @param[in]
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]
        rxChNum             Rx Channel number selection from 0..7.

 *  @param[in]
        dspNum              DSP number from 0..3.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_setCdmaDescStarveInterruptSelect
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         rxChNum,
    uint8_t                         dspNum
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || (rxChNum > 7) || (dspNum > 3))
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    switch(rxChNum)
    {
        case 0:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_0, 1 << dspNum);
            break;
        case 1:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_1, 1 << dspNum);
            break;
        case 2:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_2, 1 << dspNum);
            break;
        case 3:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_3, 1 << dspNum);
            break;
        case 4:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_4, 1 << dspNum);
            break;
        case 5:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_5, 1 << dspNum);
            break;
        case 6:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_6, 1 << dspNum);
            break;
        case 7:
            CSL_FINS (pTmRegs->CDMA_DESC_STARVE_INTR_SEL, BCP_TM_CDMA_DESC_STARVE_INTR_SEL_DESC_STARVE_7, 1 << dspNum);
            break;
        default: 
            break;
    }

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setHaltOnErrorReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of BCP_HALT_ON_ERROR register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        index               Indicates which of the 4 Halt on error registers
                            to configure.

 *  @param[in]    
        pHaltCfg            Halt on error settings for all BCP submodules

*  @param[in]    
        bForceHalt          Boolean flag that indicates if a forced halt must
                            be initiated.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  BCP_HALT_ON_ERROR register configured with the values passed.
 * ============================================================================
 */
static inline int32_t Bcp_setHaltOnErrorReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         index,
    Bcp_ModuleCfg*                  pHaltCfg,
    uint8_t                         bForceHalt
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0]|| !pHaltCfg)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->TM_SPECIAL_INTERRUPT_REGISTERS[index].BCP_HALT_ON_ERROR  =    
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_TM_HALT_ON_ERROR, pHaltCfg->tm) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_DIO_HALT_ON_ERROR, pHaltCfg->dio) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_CRC_HALT_ON_ERROR, pHaltCfg->crc) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_ENC_HALT_ON_ERROR, pHaltCfg->enc) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_RM_HALT_ON_ERROR, pHaltCfg->rm) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_MOD_HALT_ON_ERROR, pHaltCfg->mod) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_INT_HALT_ON_ERROR, pHaltCfg->intleaver) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_COR_HALT_ON_ERROR, pHaltCfg->cor) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_SSL_HALT_ON_ERROR, pHaltCfg->ssl) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_RD_HALT_ON_ERROR, pHaltCfg->rd) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_DNT_HALT_ON_ERROR, pHaltCfg->dnt) |
                    CSL_FMK (BCP_TM_BCP_HALT_ON_ERROR_FRC_HALT_ON_ERROR, bForceHalt);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getHaltOnErrorReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP_HALT_ON_ERROR register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        index               Indicates which of the 4 Halt on error registers
                            to read.

 *  @param[out]    
        pHaltCfg            Halt on error settings for all BCP submodules

*  @param[out]    
        pForceHaltEnabled   1 indicates Forced halt is enabled and 0 indicates
                            otherwise.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getHaltOnErrorReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         index,
    Bcp_ModuleCfg*                  pHaltCfg,
    uint8_t*                        pForceHaltEnabled
)
{
    uint32_t                        tmpWord;
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pHaltCfg || !pForceHaltEnabled)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->TM_SPECIAL_INTERRUPT_REGISTERS[index].BCP_HALT_ON_ERROR;

    pHaltCfg->tm        =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_TM_HALT_ON_ERROR);
    pHaltCfg->dio       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_DIO_HALT_ON_ERROR);
    pHaltCfg->crc       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_CRC_HALT_ON_ERROR);
    pHaltCfg->enc       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_ENC_HALT_ON_ERROR);
    pHaltCfg->rm        =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_RM_HALT_ON_ERROR);
    pHaltCfg->mod       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_MOD_HALT_ON_ERROR);
    pHaltCfg->intleaver =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_INT_HALT_ON_ERROR);
    pHaltCfg->cor       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_COR_HALT_ON_ERROR);
    pHaltCfg->ssl       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_SSL_HALT_ON_ERROR);
    pHaltCfg->rd        =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_RD_HALT_ON_ERROR);
    pHaltCfg->dnt       =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_DNT_HALT_ON_ERROR);
    *pForceHaltEnabled  =   CSL_FEXT (tmpWord, BCP_TM_BCP_HALT_ON_ERROR_FRC_HALT_ON_ERROR);

    return 0;
}


/**
 * ============================================================================
 *  @n@b Bcp_getInterruptStatusReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP_INTERRUPT_STATUS register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        index               Indicates which of the 4 Interrupt status registers
                            to read.

 *  @param[out]    
        pIntStatus          Interrupt status for all BCP submodules

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getInterruptStatusReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         index,
    Bcp_ModuleCfg*                  pIntStatus
)
{
    uint32_t                        tmpWord;
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pIntStatus)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->TM_SPECIAL_INTERRUPT_REGISTERS[index].BCP_INTERRUPT_STATUS;

    pIntStatus->tm          =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_TM_INTR_STATUS);
    pIntStatus->dio         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_DIO_INTR_STATUS);
    pIntStatus->crc         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_CRC_INTR_STATUS);
    pIntStatus->enc         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_ENC_INTR_STATUS);
    pIntStatus->rm          =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_RM_INTR_STATUS);
    pIntStatus->mod         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_MOD_INTR_STATUS);
    pIntStatus->intleaver   =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_INT_INTR_STATUS);
    pIntStatus->cor         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_COR_INTR_STATUS);
    pIntStatus->ssl         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_SSL_INTR_STATUS);
    pIntStatus->rd          =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_RD_INTR_STATUS);
    pIntStatus->dnt         =   CSL_FEXT (tmpWord, BCP_TM_BCP_INTERRUPT_STATUS_DNT_INTR_STATUS);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setEoiReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of BCP_EOI register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        eoiVal              Interrupt vector to acknowledge.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  BCP_EOI register configured with the values passed.
 * ============================================================================
 */
static inline int32_t Bcp_setEoiReg 
(
    Bcp_LldObj*                     pBcpLldObj,
    uint8_t                         eoiVal
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    CSL_FINS (pTmRegs->BCP_EOI, BCP_TM_BCP_EOI_BCP_EOI,eoiVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getEoiReg
 *
 *  @b  brief
 *  @n  This API returns the contents of BCP_EOI register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Success.BCP_EOI register contents returned.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getEoiReg 
(
    Bcp_LldObj*                     pBcpLldObj
)
{
    CSL_Bcp_tmRegs*                 pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);

    return CSL_FEXT (pTmRegs->BCP_EOI, BCP_TM_BCP_EOI_BCP_EOI);
}

/**
 * ============================================================================
 *  @n@b Bcp_getTmFlowEntry
 *
 *  @b  brief
 *  @n  Given a TM Flow table index, this API returns the contents of 
 *      Flow entry corresponding to it.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        flow_index          Flow table index to be read.

 *  @param[out]    
        pFlowEntry          Flow entry contents corresponding to index passed.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getTmFlowEntry 
(
    Bcp_LldObj*             pBcpLldObj,
    uint8_t                 flow_index,
    Bcp_TmFlowEntry*        pFlowEntry
)
{
    uint32_t                tmpWord;
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pFlowEntry)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
    tmpWord =   pTmRegs->FLOW_ID_TABLE_0 [flow_index];

    pFlowEntry->endian_in   =   (Bcp_EndianFormat) CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_ENDIAN_IN);
    pFlowEntry->format_in   =   (Bcp_DataFormat) CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_FORMAT_IN);
    pFlowEntry->pkt_type    =   CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_PKT_TYPE);
    pFlowEntry->dsp_int_sel =   CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_DSP_INT_SEL);
    pFlowEntry->endian_out  =   (Bcp_EndianFormat) CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_ENDIAN_OUT);
    pFlowEntry->format_out  =   (Bcp_DataFormat) CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_FORMAT_OUT);
    pFlowEntry->qfifo_out   =   CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_QFIFO_OUT);
    pFlowEntry->ps_flags    =   CSL_FEXT (tmpWord, BCP_TM_FLOW_ID_TABLE_0_PS_FLAGS);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setTmFlowEntry
 *
 *  @b  brief
 *  @n  Given a TM Flow table index, this API populates the contents of 
 *      Flow entry corresponding to it.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        flow_index          Flow table index to be read.

 *  @param[in]    
        pFlowEntry          Flow entry settings to be configured.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  FLOW_ID_TABLE register corresponding to flow_index specified configured.
 * ============================================================================
 */
static inline int32_t Bcp_setTmFlowEntry 
(
    Bcp_LldObj*             pBcpLldObj,
    uint8_t                 flow_index,
    Bcp_TmFlowEntry*        pFlowEntry
)
{
    CSL_Bcp_tmRegs*         pTmRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pFlowEntry || flow_index >= BCP_MAX_NUM_TM_FLOWS)
        return -1;

    pTmRegs =   (CSL_Bcp_tmRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_TM]);
        
    pTmRegs->FLOW_ID_TABLE_0 [flow_index]  =   
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_ENDIAN_IN, pFlowEntry->endian_in ) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_FORMAT_IN, pFlowEntry->format_in) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_PKT_TYPE, pFlowEntry->pkt_type) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_DSP_INT_SEL, pFlowEntry->dsp_int_sel) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_ENDIAN_OUT, pFlowEntry->endian_out) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_FORMAT_OUT, pFlowEntry->format_out) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_QFIFO_OUT, pFlowEntry->qfifo_out) |
                CSL_FMK (BCP_TM_FLOW_ID_TABLE_0_PS_FLAGS, pFlowEntry->ps_flags);

    return 0;
}

/****************** DIO MMRs ******************/

/**
 * ============================================================================
 *  @n@b Bcp_getDioVbusMPriorityReg
 *
 *  @b  brief
 *  @n  This API returns the contents of DIO VBUSM Priority register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of VBUSM_PRIORITY register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getDioVbusMPriorityReg 
(
    Bcp_LldObj*             pBcpLldObj
)
{
    CSL_Bcp_dioRegs*        pDioRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pDioRegs    =   (CSL_Bcp_dioRegs *) (pBcpLldObj->modDlgRegs[Bcp_ModuleId_DIO]);

    return CSL_FEXT (pDioRegs->DIO_VBUSM_PRIORITY, BCP_DIO_DIO_VBUSM_PRIORITY_DIO_VBUSM_PRIORITY);
}

/**
 * ============================================================================
 *  @n@b Bcp_setDioVbusMPriorityReg
 *
 *  @b  brief
 *  @n  This API sets up the contents of DIO VBUSM Priority register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        prioVal             Priority value to configure. Value 0 is highest 
                            priority and 7 is considered lowest.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     =0  -   Succesfully configured VBUSM_PRIORITY register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  VBUSM_PRIORITY register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setDioVbusMPriorityReg 
(
    Bcp_LldObj*             pBcpLldObj,
    uint32_t                prioVal
)
{
    CSL_Bcp_dioRegs*        pDioRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pDioRegs    =   (CSL_Bcp_dioRegs *) (pBcpLldObj->modDlgRegs[Bcp_ModuleId_DIO]);

    CSL_FINS (pDioRegs->DIO_VBUSM_PRIORITY, BCP_DIO_DIO_VBUSM_PRIORITY_DIO_VBUSM_PRIORITY, prioVal);

    return 0;
}

/****************** Common MMRs ******************/

/**
 * ============================================================================
 *  @n@b Bcp_getModuleIntRawStatus
 *
 *  @b  brief
 *  @n  This API returns the contents of Interrupt raw status register for a
 *      given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be read.

 *  @param[in]    
        index               Raw status register index to read.

 *  @param[out]    
        pEngErrStatus       Engine error raw status read.

 *  @param[out]    
        pDlgDoneStatus      Data logger raw status read.

 *  @param[out]    
        pPpbErrStatus       PPB error raw status read.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Contents of INTR_IRS register returned in
 *                                  output parameters.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleIntRawStatus 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t*                pEngErrStatus,
    uint8_t*                pDlgDoneStatus,
    uint8_t*                pPpbErrStatus
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;
    uint32_t                regVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pEngErrStatus || !pDlgDoneStatus || !pPpbErrStatus)
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);
    regVal      =   pModIntRegs->INTR_IRS;

    *pEngErrStatus  =   CSL_FEXT (regVal, BCP_INT_INT_INTR_IRS_ENG_ERROR);
    *pDlgDoneStatus =   CSL_FEXT (regVal, BCP_INT_INT_INTR_IRS_DLG_DONE_INT);
    *pPpbErrStatus  =   CSL_FEXT (regVal, BCP_INT_INT_INTR_IRS_PPB_ERR);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setModuleIntRawStatus
 *
 *  @b  brief
 *  @n  This API populates the contents of Interrupt Raw status set register for
 *      any given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be set.

 *  @param[in]    
        index               Raw status register index to set.

 *  @param[in]    
        engErrStatus        Engine error status value to configure

 *  @param[in]    
        dlgDoneStatus       DLG done status value to configure

 *  @param[in]    
        ppbErrStatus        PPB error status value to configure

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  INTR_IRS_SET register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setModuleIntRawStatus
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t                 engErrStatus,
    uint8_t                 dlgDoneStatus,
    uint8_t                 ppbErrStatus
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);

    pModIntRegs->INTR_IRS_SET   =   CSL_FMK (BCP_INT_INT_INTR_IRS_SET_ENG_ERROR, engErrStatus) |
                                    CSL_FMK (BCP_INT_INT_INTR_IRS_SET_DLG_DONE_INT, dlgDoneStatus) |
                                    CSL_FMK (BCP_INT_INT_INTR_IRS_SET_PPB_ERR, ppbErrStatus);
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_clearModuleIntRawStatus
 *
 *  @b  brief
 *  @n  This API clears the contents of Interrupt Raw status register for any 
 *      given BCP submodule by writing to the interrupt raw status clear register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be cleared.

 *  @param[in]    
        index               Raw status register index to clear.

 *  @param[in]    
        engErrClrMask       Engine error clear mask. 

 *  @param[in]    
        dlgDoneClrMask      DLG done clear bit.

 *  @param[in]    
        ppbErrClrMask       PPB error clear bit. 

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  INTR_IRS_CLR register configured. Corresponding INTR_IRS register cleared.
 * ============================================================================
 */
static inline int32_t Bcp_clearModuleIntRawStatus 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t                 engErrClrMask,
    uint8_t                 dlgDoneClrMask,
    uint8_t                 ppbErrClrMask
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);

    pModIntRegs->INTR_IRS_CLR   =   CSL_FMK (BCP_INT_INT_INTR_IRS_CLR_ENG_ERROR, engErrClrMask) | 
                                    CSL_FMK (BCP_INT_INT_INTR_IRS_CLR_DLG_DONE_INT, dlgDoneClrMask) |
                                    CSL_FMK (BCP_INT_INT_INTR_IRS_CLR_PPB_ERR, ppbErrClrMask);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getModuleIntEnable
 *
 *  @b  brief
 *  @n  This API returns the contents of Interrupt enable register for a given
 *      BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be read.

 *  @param[in]    
        index               Interrupt enable register index to read.

 *  @param[out]    
        pEngErrEnable       Engine error enable value read.

 *  @param[out]    
        pDlgDoneEnable      Data logger enable bit read.

 *  @param[out]    
        pPpbErrEnable       PPB error enable bit read.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     =0  -   Contents of INTR_EN register returned
 *                                  succesfully in the output params.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleIntEnable 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t*                pEngErrEnable,
    uint8_t*                pDlgDoneEnable,
    uint8_t*                pPpbErrEnable         
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;
    uint32_t                regVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pEngErrEnable || !pDlgDoneEnable || !pPpbErrEnable)
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);
    regVal      =   pModIntRegs->INTR_EN;

    *pEngErrEnable  =   CSL_FEXT (regVal, BCP_INT_INT_INTR_EN_ENG_ERROR);
    *pDlgDoneEnable =   CSL_FEXT (regVal, BCP_INT_INT_INTR_EN_DLG_DONE_INT);
    *pPpbErrEnable  =   CSL_FEXT (regVal, BCP_INT_INT_INTR_EN_PPB_ERR);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setModuleIntEnable
 *
 *  @b  brief
 *  @n  This API populates the contents of Interrupt Enable Set register for a
 *      given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be set.

 *  @param[in]    
        index               Interrupt enable register index.

 *  @param[in]    
        engErrEnable        Engine error enable value to configure

 *  @param[in]    
        dlgDoneEnable       DLG done enable bit value.

 *  @param[in]    
        ppbErrEnable        PPB error enable bit value.

 *  @return     int32_t 
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  INTR_EN_SET register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setModuleIntEnable 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t                 engErrEnable,
    uint8_t                 dlgDoneEnable,
    uint8_t                 ppbErrEnable
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);

    pModIntRegs->INTR_EN_SET    =   CSL_FMK (BCP_INT_INT_INTR_EN_SET_ENG_ERROR, engErrEnable) |
                                    CSL_FMK (BCP_INT_INT_INTR_EN_SET_DLG_DONE_INT, dlgDoneEnable) |
                                    CSL_FMK (BCP_INT_INT_INTR_EN_SET_PPB_ERR, ppbErrEnable);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_clearModuleIntEnable
 *
 *  @b  brief
 *  @n  This API clears the contents of Interrupt Enable register for a given 
 *      BCP submodule by writing to the interrupt enable clear register.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be cleared.

 *  @param[in]    
        index               Interrupt enable register index to clear.

 *  @param[in]    
        engErrClrMask       Engine error clear mask. 

 *  @param[in]    
        dlgDoneClrMask      DLG done clear bit.

 *  @param[in]    
        ppbErrClrMask       PPB error clear bit. 

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  INTR_EN_CLR register configured. Corresponding INTR_EN register cleared.
 * ============================================================================
 */
static inline int32_t Bcp_clearModuleIntEnable
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t                 engErrClrMask,
    uint8_t                 dlgDoneClrMask,
    uint8_t                 ppbErrClrMask
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);

    pModIntRegs->INTR_EN_CLR    =   CSL_FMK (BCP_INT_INT_INTR_EN_CLR_ENG_ERROR, engErrClrMask) |
                                    CSL_FMK (BCP_INT_INT_INTR_EN_CLR_DLG_DONE_INT, dlgDoneClrMask) |
                                    CSL_FMK (BCP_INT_INT_INTR_EN_CLR_PPB_ERR, ppbErrClrMask);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getModuleIntEnableStatus
 *
 *  @b  brief
 *  @n  This API returns the contents of Interrupt enable status register for 
 *      a given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP Submodule for which the register must be read.

 *  @param[in]    
        index               Interrupt enable status register index to read.

 *  @param[out]    
        pEngErrEnable       Engine error enable value read.

 *  @param[out]    
        pDlgDoneEnable      Data logger enable bit read.

 *  @param[out]    
        pPpbErrEnable       PPB error enable bit read.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     =0  -   Contents of INTR_EN_STS register succesfully
 *                                  returned in output params.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleIntEnableStatus
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint32_t                index,
    uint8_t*                pEngErrEnable,
    uint8_t*                pDlgDoneEnable,
    uint8_t*                pPpbErrEnable         
)
{
    CSL_Bcp_IntRegs*        pModIntRegs;
    uint32_t                regVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pEngErrEnable || !pDlgDoneEnable || !pPpbErrEnable)
        return -1;

    pModIntRegs =   (CSL_Bcp_IntRegs *) (pBcpLldObj->modIntRegs[moduleId] + sizeof (CSL_Bcp_IntRegs) * index);
    regVal      =   pModIntRegs->INTR_EN_STS;

    *pEngErrEnable  =   CSL_FEXT (regVal, BCP_INT_INT_INTR_EN_STS_ENG_ERROR);
    *pDlgDoneEnable =   CSL_FEXT (regVal, BCP_INT_INT_INTR_EN_STS_DLG_DONE_INT);
    *pPpbErrEnable  =   CSL_FEXT (regVal, BCP_INT_INT_INTR_EN_STS_PPB_ERR);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getModuleDlgControlReg
 *
 *  @b  brief
 *  @n  This API returns the contents of Data logger control register for any
 *      given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP submodule for which the DLG registers must be read.

 *  @param[out]    
        pDlgCtlVal          DLG control mode value read.

 *  @param[out]    
        pDlgErrMaskVal      DLG error mask value read.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     =0  -   Contents of DLG_CONTROL register returned in
 *                                  output parameters.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleDlgControlReg 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    Bcp_DlgCtrlMode*        pDlgCtlVal,
    uint16_t*               pDlgErrMaskVal
)
{
    CSL_Bcp_DataLoggerRegs* pModDlgRegs;
    uint32_t                regVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pDlgCtlVal || !pDlgErrMaskVal)
        return -1;

    pModDlgRegs =   (CSL_Bcp_DataLoggerRegs *) (pBcpLldObj->modDlgRegs[moduleId]);
    regVal      =   pModDlgRegs->DLG_CONTROL;

    *pDlgCtlVal     =   (Bcp_DlgCtrlMode) CSL_FEXT (regVal, BCP_INT_INT_DATA_LOGGER_CTL_DLG_CTL);
    *pDlgErrMaskVal =   CSL_FEXT (regVal, BCP_INT_INT_DATA_LOGGER_CTL_DLG_ERR_MASK);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_setModuleDlgControlReg
 *
 *  @b  brief
 *  @n  This API populates the contents of Data logger control register for any
 *      given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP submodule for which the DLG registers must be configured.

 *  @param[out]    
        dlgCtlVal           DLG control mode value to configure.

 *  @param[out]    
        dlgErrMaskVal       DLG error mask value to configure.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Succesfully configured the register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  DLG_CONTROL register configured.
 * ============================================================================
 */
static inline int32_t Bcp_setModuleDlgControlReg 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    Bcp_DlgCtrlMode         dlgCtlVal,
    uint16_t                dlgErrMaskVal
)
{
    CSL_Bcp_DataLoggerRegs* pModDlgRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pModDlgRegs =   (CSL_Bcp_DataLoggerRegs *) (pBcpLldObj->modDlgRegs[moduleId]);

    pModDlgRegs->DLG_CONTROL    =   CSL_FMK (BCP_INT_INT_DATA_LOGGER_CTL_DLG_CTL, dlgCtlVal) |
                                    CSL_FMK (BCP_INT_INT_DATA_LOGGER_CTL_DLG_ERR_MASK, dlgErrMaskVal);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getModuleDlgStatusReg
 *
 *  @b  brief
 *  @n  This API returns the contents of Data logger status register for any given
 *      BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP submodule for which the DLG register must be read.

 *  @param[out]    
        pDlgRunningStatus   Set while data is being captured by DLG. Cleared when the
                            last location of memory is written if selected mode stops
                            capturing when memory is full.

 *  @param[out]    
        pDlgMemErrorStatus  When set, indicates there is atleast one error currently
                            stored in DLG RAM.

 *  @param[out]    
        pDlgWrWrapCount     Number of times writing to memory wrapped to 0.

 *  @param[out]    
        pDlgWrPtr           Current RAM Write Pointer.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     =0  -   Contents of DLG_STATUS register returned 
 *                                  succesfully in the output params.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleDlgStatusReg 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint8_t*                pDlgRunningStatus,
    uint8_t*                pDlgMemErrorStatus,
    uint8_t*                pDlgWrWrapCount,
    uint8_t*                pDlgWrPtr
)
{
    CSL_Bcp_DataLoggerRegs* pModDlgRegs;
    uint32_t                regVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pDlgRunningStatus || !pDlgMemErrorStatus || !pDlgWrWrapCount || !pDlgWrPtr)
        return -1;

    pModDlgRegs =   (CSL_Bcp_DataLoggerRegs *) (pBcpLldObj->modDlgRegs[moduleId]);
    regVal      =   pModDlgRegs->DLG_STATUS;

    *pDlgRunningStatus  =   CSL_FEXT (regVal, BCP_INT_INT_DATA_LOGGER_STATUS_DLG_RUNNING);
    *pDlgMemErrorStatus =   CSL_FEXT (regVal, BCP_INT_INT_DATA_LOGGER_STATUS_DLG_MEM_HAS_ERROR);
    *pDlgWrWrapCount    =   CSL_FEXT (regVal, BCP_INT_INT_DATA_LOGGER_STATUS_DLG_WR_WRAP);
    *pDlgWrPtr          =   CSL_FEXT (regVal, BCP_INT_INT_DATA_LOGGER_STATUS_DLG_WR_PTR);

    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_getModuleDlgGlobalHeaderReg
 *
 *  @b  brief
 *  @n  This API returns the contents of Data logger global header register for
 *      any given BCP submodule.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP submodule for which the DLG register must be read.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     >=0 -   Contents of Data logger DLG_GLOBAL_HDR register.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleDlgGlobalHeaderReg 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId
)
{
    CSL_Bcp_DataLoggerRegs* pModDlgRegs;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pModDlgRegs =   (CSL_Bcp_DataLoggerRegs *) (pBcpLldObj->modDlgRegs[moduleId]);

    return CSL_FEXT (pModDlgRegs->DLG_GLOBAL_HDR, BCP_INT_INT_GLOBAL_HDR_GLOBAL_HDR);
}

/**
 * ============================================================================
 *  @n@b Bcp_getModuleDlgRAMEntry
 *
 *  @b  brief
 *  @n  Given a BCP Submodule Id and a Data Logger RAM entry index, this API 
 *      returns the contents of RAM entry corresponding to it from the submodule's
 *      Data logger RAM memory.
 *
 *  @param[in]    
        pBcpLldObj          BCP LLD instance object.

 *  @param[in]    
        moduleId            BCP submodule Id for which the RAM entry must be read.

 *  @param[in]    
        entry_index         RAM entry index to read.

 *  @param[out]    
        pRamEntry           RAM entry contents corresponding to index passed.

 *  @return     int32_t
 *  @li                     -1  -   Invalid BCP instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Bcp_lldOpen () must be called to obtain the register overlay handle for 
 *      BCP instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
static inline int32_t Bcp_getModuleDlgRAMEntry 
(
    Bcp_LldObj*             pBcpLldObj,
    Bcp_ModuleId            moduleId,
    uint8_t                 entry_index,
    Bcp_DlgRamEntry*        pRamEntry
)
{
    CSL_Bcp_DataLoggerRAM*  pDlgRamRegs;
    uint32_t                regVal;

    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0] || !pRamEntry || entry_index > BCP_MAX_NUM_DLG_RAM_ENTRIES)
        return -1;

    pDlgRamRegs =   (CSL_Bcp_DataLoggerRAM *) (pBcpLldObj->modDlgRamRegs[moduleId]);

    pRamEntry->global_hdr_val   =   CSL_FEXT (pDlgRamRegs->DLG_DATA_LOG [entry_index].DLG_GLOBAL_HDR, 
                                              BCP_INT_INT_DLG_GLOBAL_HDR_GLOBAL_HDR);

    pRamEntry->sw_timestamp_val =   CSL_FEXT (pDlgRamRegs->DLG_DATA_LOG [entry_index].DLG_TM_SW_TIMESTAMP, 
                                              BCP_INT_INT_DLG_TM_SW_TIMESTAMP_TM_SW_TIMESTAMP);

    regVal                      =   pDlgRamRegs->DLG_DATA_LOG [entry_index].DLG_HW_TIMESTAMP_ENG_ERR;
    pRamEntry->hw_timestamp_val =   CSL_FEXT (regVal, BCP_INT_INT_DLG_HW_TIMESTAMP_ENG_ERR_TM_HW_TIMESTAMP);
    pRamEntry->engine_error_val =   CSL_FEXT (regVal, BCP_INT_INT_DLG_HW_TIMESTAMP_ENG_ERR_ENG_ERR);

    return 0;
}

/**
@}
*/

#endif
