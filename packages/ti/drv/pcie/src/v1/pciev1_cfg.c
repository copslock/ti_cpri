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
 *  File Name: pciev1_cfg.c
 *
 *  Processing/configuration functions for the PCIe Configuration Registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v1/pcieloc.h>


/*****************************************************************************
 **********  PCIe CONFIG REGISTERS COMMON TO TYPE0 AND TYPE1  *****************
 ****************************************************************************/

/* pack/unpack for backwards compatibility */
#define PCIE_REV1_CLASSCODE_MASK ( \
           CSL_EPCFGDBICS_CLASSCODE_REVISIONID_PROG_IF_CODE_MASK | \
           CSL_EPCFGDBICS_CLASSCODE_REVISIONID_SUBCLS_CD_MASK | \
           CSL_EPCFGDBICS_CLASSCODE_REVISIONID_BASE_CLS_CD_MASK)
#define PCIE_REV1_CLASSCODE_SHIFT (CSL_EPCFGDBICS_CLASSCODE_REVISIONID_PROG_IF_CODE_SHIFT)

/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Vendor and Device Identification register
 ****************************************************************************/
pcieRet_e pciev1_read_vndDevId_reg
(
  volatile const Uint32 *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEVICE_VENDORID;

  pcie_getbits(val, CSL_EPCFGDBICS_DEVICE_VENDORID_VENDORID, swReg->vndId);
  pcie_getbits(val, CSL_EPCFGDBICS_DEVICE_VENDORID_DEVICEID, swReg->devId);

  return pcie_RET_OK;
} /* pciev1_read_vndDevId_reg */

/*****************************************************************************
 * Combine and write the Vendor and Device Identification register
 ****************************************************************************/
pcieRet_e pciev1_write_vndDevId_reg
(
  volatile Uint32 *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_DEVICE_VENDORID_VENDORID, swReg->vndId);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEVICE_VENDORID_DEVICEID, swReg->devId);

  *hwReg_DEVICE_VENDORID = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_vndDevId_reg */



/*****************************************************************************
 * Read and split up the Status and Command register
 ****************************************************************************/
pcieRet_e pciev1_read_statusCmd_reg
(
  volatile const Uint32 *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_STATUS_COMMAND_REGISTER;

  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_IO_SPACE_EN,       swReg->ioSp);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_MEM_SPACE_EN,      swReg->memSp);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_BUSMASTER_EN,      swReg->busMs);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SPEC_CYCLE_EN,     swReg->specCycleEn);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_MEMWR_INVA,        swReg->memWrInva);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_VGA_SNOOP,         swReg->vgaSnoop);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_PARITYERRRESP,     swReg->resp);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_IDSEL_CTRL,        swReg->idselCtrl);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SERR_EN,           swReg->serrEn);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_INTX_ASSER_DIS,    swReg->dis);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_INTX_STATUS,       swReg->stat);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_CAP_LIST,          swReg->capList);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_C66MHZ_CAP,        swReg->c66MhzCap);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_FAST_B2B,          swReg->fastB2B);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_MASTERDATA_PARERR, swReg->parError);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_DEVSEL_TIME,       swReg->devSelTime);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SIGNAL_TRGTABORT,  swReg->sigTgtAbort);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_RCVD_TRGTABORT,    swReg->tgtAbort);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_RCVD_MASTERABORT,  swReg->mstAbort);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SIGNAL_SYSERR,     swReg->sysError);
  pcie_getbits(val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_DETECT_PARERR,     swReg->parity);

  return pcie_RET_OK;
} /* pciev1_read_statusCmd_reg */

/*****************************************************************************
 * Combine and write the Status and Command register
 ****************************************************************************/
pcieRet_e pciev1_write_statusCmd_reg
(
  volatile Uint32 *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_IO_SPACE_EN,       swReg->ioSp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_MEM_SPACE_EN,      swReg->memSp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_BUSMASTER_EN,      swReg->busMs);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SPEC_CYCLE_EN,     swReg->specCycleEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_MEMWR_INVA,        swReg->memWrInva);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_VGA_SNOOP,         swReg->vgaSnoop);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_PARITYERRRESP,     swReg->resp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_IDSEL_CTRL,        swReg->idselCtrl);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SERR_EN,           swReg->serrEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_INTX_ASSER_DIS,    swReg->dis);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_INTX_STATUS,       swReg->stat);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_CAP_LIST,          swReg->capList);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_C66MHZ_CAP,        swReg->c66MhzCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_FAST_B2B,          swReg->fastB2B);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_MASTERDATA_PARERR, swReg->parError);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_DEVSEL_TIME,       swReg->devSelTime);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SIGNAL_TRGTABORT,  swReg->sigTgtAbort);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_RCVD_TRGTABORT,    swReg->tgtAbort);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_RCVD_MASTERABORT,  swReg->mstAbort);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_SIGNAL_SYSERR,     swReg->sysError);
  pcie_setbits(new_val, CSL_EPCFGDBICS_STATUS_COMMAND_REGISTER_DETECT_PARERR,     swReg->parity);

  *hwReg_STATUS_COMMAND_REGISTER = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_statusCmd_reg */

/*****************************************************************************
 * Read and split up the Class Code and Revision ID register
 ****************************************************************************/
pcieRet_e pciev1_read_revId_reg
(
  volatile const Uint32 *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_CLASSCODE_REVISIONID;

  pcie_getbits(val, PCIE_REV1_CLASSCODE,                       swReg->classCode);
  pcie_getbits(val, CSL_EPCFGDBICS_CLASSCODE_REVISIONID_REVID, swReg->revId);

  return pcie_RET_OK;
} /* pciev1_read_revId_reg */

/*****************************************************************************
 * Combine and write the Class Code and Revision ID register
 ****************************************************************************/
pcieRet_e pciev1_write_revId_reg
(
  volatile Uint32 *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, PCIE_REV1_CLASSCODE,                       swReg->classCode);
  pcie_setbits(new_val, CSL_EPCFGDBICS_CLASSCODE_REVISIONID_REVID, swReg->revId);

  *hwReg_CLASSCODE_REVISIONID = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_revId_reg */



/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS **********************
 ****************************************************************************/


/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************  
 * Read and split up the PCIE Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_read_pciesCap_reg
(
  volatile const Uint32 *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_PCIE_CAP;

  pcie_getbits(val, CSL_EPCFGDBICS_PCIE_CAP_CAP_ID,      swReg->capId);
  pcie_getbits(val, CSL_EPCFGDBICS_PCIE_CAP_PCIE_NX_PTR, swReg->nextCap);
  pcie_getbits(val, CSL_EPCFGDBICS_PCIE_CAP_PCIE_VER,    swReg->pcieCap);
  pcie_getbits(val, CSL_EPCFGDBICS_PCIE_CAP_DEV_TYPE,    swReg->dportType);
  pcie_getbits(val, CSL_EPCFGDBICS_PCIE_CAP_SLOT,        swReg->sltImplN);
  pcie_getbits(val, CSL_EPCFGDBICS_PCIE_CAP_IM_NUM,      swReg->intMsg);

  return pcie_RET_OK;
} /* pciev1_read_pciesCap_reg */


/*****************************************************************************  
 * Combine and write the PCIE Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_write_pciesCap_reg
(
  volatile Uint32 *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_PCIE_CAP_CAP_ID,      swReg->capId);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PCIE_CAP_PCIE_NX_PTR, swReg->nextCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PCIE_CAP_PCIE_VER,    swReg->pcieCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PCIE_CAP_DEV_TYPE,    swReg->dportType);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PCIE_CAP_SLOT,        swReg->sltImplN);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PCIE_CAP_IM_NUM,      swReg->intMsg);

  *hwReg_PCIE_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_pciesCap_reg */

/*****************************************************************************  
 * Read and split up the Device Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_read_deviceCap_reg
(
  volatile const Uint32 *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAP;

  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_MAX_PAYLOAD_SIZE,             swReg->maxPayldSz);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_PHANTOMFUNC,                  swReg->phantomFld);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_EXTTAGFIELD_SUPPORT,          swReg->extTagFld);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_DEFAULT_EP_L0S_ACCPT_LATENCY, swReg->l0Latency);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_DEFAULT_EP_L1_ACCPT_LATENCY,  swReg->l1Latency);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_ROLEBASED_ERRRPT,             swReg->errRpt);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_CAPT_SLOW_PWRLIMIT_VALUE,     swReg->pwrLimitValue);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_CAPT_SLOW_PWRLIMIT_SCALE,     swReg->pwrLimitScale);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_FLR_EN,                       swReg->flrEn);

  return pcie_RET_OK;
} /* pciev1_read_deviceCap_reg */


/*****************************************************************************  
 * Combine and write the Device Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_write_deviceCap_reg
(
  volatile Uint32 *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_MAX_PAYLOAD_SIZE,             swReg->maxPayldSz);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_PHANTOMFUNC,                  swReg->phantomFld);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_EXTTAGFIELD_SUPPORT,          swReg->extTagFld);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_DEFAULT_EP_L0S_ACCPT_LATENCY, swReg->l0Latency);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_DEFAULT_EP_L1_ACCPT_LATENCY,  swReg->l1Latency);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_ROLEBASED_ERRRPT,             swReg->errRpt);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_CAPT_SLOW_PWRLIMIT_VALUE,     swReg->pwrLimitValue);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_CAPT_SLOW_PWRLIMIT_SCALE,     swReg->pwrLimitScale);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_FLR_EN,                       swReg->flrEn);

  *hwReg_DEV_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_deviceCap_reg */

/*****************************************************************************
 * Read and split up the Device Status and Control register
 ****************************************************************************/
pcieRet_e pciev1_read_devStatCtrl_reg
(
  volatile const Uint32 *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAS;

  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_COR_RE,     swReg->corErRp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_NFT_RE,     swReg->nFatalErRp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_FT_RE,      swReg->fatalErRp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_UR_RE,      swReg->reqRp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_EN_RO,      swReg->relaxed);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_MPS,        swReg->maxPayld);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_EXTAG_EN,   swReg->xtagEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_PHFUN_EN,   swReg->phantomEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_AUXPM_EN,   swReg->auxPwrEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_NOSNP_EN,   swReg->noSnoop);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_MRRS,       swReg->maxSz);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_COR_DET,    swReg->corrEr);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_NFT_DET,    swReg->nFatalEr);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_FT_DET,     swReg->fatalEr);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_UR_DET,     swReg->rqDet);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_AUXP_DET,   swReg->auxPwr);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_TRANS_PEND, swReg->tpend);

  /* Note: INIT_FLR is ignored since its reserved */
  swReg->initFLR = 0u;

  return pcie_RET_OK;
} /* pciev1_read_devStatCtrl_reg */


/*****************************************************************************
 * Combine and write the Device Status and Control register
 ****************************************************************************/
pcieRet_e pciev1_write_devStatCtrl_reg
(
  volatile Uint32 *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_COR_RE,     swReg->corErRp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_NFT_RE,     swReg->nFatalErRp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_FT_RE,      swReg->fatalErRp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_UR_RE,      swReg->reqRp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_EN_RO,      swReg->relaxed);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_MPS,        swReg->maxPayld);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_EXTAG_EN,   swReg->xtagEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_PHFUN_EN,   swReg->phantomEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_AUXPM_EN,   swReg->auxPwrEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_NOSNP_EN,   swReg->noSnoop);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_MRRS,       swReg->maxSz);
  /* Note: INIT_FLR is ignored since its reserved */
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_COR_DET,    swReg->corrEr);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_NFT_DET,    swReg->nFatalEr);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_FT_DET,     swReg->fatalEr);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_UR_DET,     swReg->rqDet);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_AUXP_DET,   swReg->auxPwr);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_TRANS_PEND, swReg->tpend);

  *hwReg_DEV_CAS = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_devStatCtrl_reg */

/*****************************************************************************  
 * Read and split up the Link Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_read_linkCap_reg
(      
  volatile const Uint32 *hwReg_LNK_CAP,
  pcieLinkCapReg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAP;

  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_MAX_LINK_SPEEDS,    swReg->maxLinkSpeed);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_MAX_LINK_WIDTH,     swReg->maxLinkWidth);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_AS_LINK_PM_SUPPORT, swReg->asLinkPm);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_L0S_EXIT_LAT,       swReg->losExitLat);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_L1_EXIT_LAT,        swReg->l1ExitLat);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_CLK_PWR_MGMT,       swReg->clkPwrMgmt);
  /* Note: UNSUP ignored */
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_DLL_ACTRPT_CAP,     swReg->dllRepCap);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_LNK_BW_NOT_CAP,     swReg->bwNotifyCap);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_ASPM_OPT_COMP,      swReg->aspmOptComp);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_PORT_NUM,           swReg->portNum);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->downErrRepCap = 0u; /* Note: same bit on hw rev 0 as UNSUP on hw rev 1 */

  return pcie_RET_OK;
} /* pciev1_read_linkCap_reg */


/*****************************************************************************  
 * Combine and write the Link Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_write_linkCap_reg
(      
  volatile Uint32 *hwReg_LNK_CAP,
  pcieLinkCapReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_MAX_LINK_SPEEDS,    swReg->maxLinkSpeed);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_MAX_LINK_WIDTH,     swReg->maxLinkWidth);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_AS_LINK_PM_SUPPORT, swReg->asLinkPm);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_L0S_EXIT_LAT,       swReg->losExitLat);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_L1_EXIT_LAT,        swReg->l1ExitLat);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_CLK_PWR_MGMT,       swReg->clkPwrMgmt);
  /* Note: UNSUP ignored */
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_DLL_ACTRPT_CAP,     swReg->dllRepCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_LNK_BW_NOT_CAP,     swReg->bwNotifyCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_ASPM_OPT_COMP,      swReg->aspmOptComp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_PORT_NUM,           swReg->portNum);

  *hwReg_LNK_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_linkCap_reg */


/*****************************************************************************  
 * Read and split up the Link Status and Control register
 ****************************************************************************/  
pcieRet_e pciev1_read_linkStatCtrl_reg
(
  volatile const Uint32 *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAS;

  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_ASPM_CTRL,       swReg->activeLinkPm);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_RCB,             swReg->rcb);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LINK_DIS,        swReg->linkDisable);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_RETRAIN_LINK,    swReg->retrainLink);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_COM_CLK_CFG,     swReg->commonClkCfg);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_EXT_SYN,         swReg->extSync);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_EN_CPM,          swReg->clkPwrMgmtEn);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_HAWD,            swReg->hwAutoWidthDis);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LBMIE,           swReg->linkBwMgmtIntEn);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LABIE,           swReg->linkBwIntEn);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LINK_SPEED,      swReg->linkSpeed);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_NEG_LW,          swReg->negotiatedLinkWd);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_UNDEF,           swReg->undef);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LINK_TRAIN,      swReg->linkTraining);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_SLOT_CLK_CONFIG, swReg->slotClkCfg);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_DLL_ACT,         swReg->dllActive);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LBW_STATUS,      swReg->linkBwMgmtStatus);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_LAB_STATUS,      swReg->linkBwStatus);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->drsSigCtrl = 0u;

  return pcie_RET_OK;
} /* pciev1_read_linkStatCtrl_reg */


/*****************************************************************************  
 * Combine and write the Link Status and Control register
 ****************************************************************************/  
pcieRet_e pciev1_write_linkStatCtrl_reg
(
  volatile Uint32 *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_ASPM_CTRL,       swReg->activeLinkPm);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_RCB,             swReg->rcb);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LINK_DIS,        swReg->linkDisable);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_RETRAIN_LINK,    swReg->retrainLink);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_COM_CLK_CFG,     swReg->commonClkCfg);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_EXT_SYN,         swReg->extSync);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_EN_CPM,          swReg->clkPwrMgmtEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_HAWD,            swReg->hwAutoWidthDis);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LBMIE,           swReg->linkBwMgmtIntEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LABIE,           swReg->linkBwIntEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LINK_SPEED,      swReg->linkSpeed);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_NEG_LW,          swReg->negotiatedLinkWd);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_UNDEF,           swReg->undef);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LINK_TRAIN,      swReg->linkTraining);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_SLOT_CLK_CONFIG, swReg->slotClkCfg);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_DLL_ACT,         swReg->dllActive);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LBW_STATUS,      swReg->linkBwMgmtStatus);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_LAB_STATUS,      swReg->linkBwStatus);

  *hwReg_LNK_CAS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_linkStatCtrl_reg */


/*****************************************************************************  
 * Read and split up the Device Capabilities 2 register
 ****************************************************************************/  
pcieRet_e pciev1_read_devCap2_reg
(
  volatile const Uint32 *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAP_2;

  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_CPL_TIMEOUT_RNG_SUPPORTED, swReg->cmplToEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_CPL_TIMEOUT_DIS_SUPPORTED, swReg->cmplToDisSupp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_ARI_FWD_SP,                swReg->ariFwdSp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_AOR_SP,                    swReg->aorSp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_AOC32_SP,                  swReg->aoc32Sp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_AOC64_SP,                  swReg->aoc64Sp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_CASC128_SP,                swReg->casc128Sp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_NOROPR,                    swReg->noRoPR);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAP_2_TPHC_SP,                   swReg->tphcSp);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->ltrSupp          = 0u;
  swReg->lnSysCls         = 0u;
  swReg->tag10bitCompSupp = 0u;
  swReg->tag10bitReqSupp  = 0u;
  swReg->obffSupp         = 0u;

  return pcie_RET_OK;
} /* pciev1_read_devCap2_reg */


/*****************************************************************************  
 * Combine and write the Device Capabilities 2 register
 ****************************************************************************/  
pcieRet_e pciev1_write_devCap2_reg
(
  volatile Uint32 *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_CPL_TIMEOUT_RNG_SUPPORTED, swReg->cmplToEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_CPL_TIMEOUT_DIS_SUPPORTED, swReg->cmplToDisSupp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_ARI_FWD_SP,                swReg->ariFwdSp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_AOR_SP,                    swReg->aorSp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_AOC32_SP,                  swReg->aoc32Sp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_AOC64_SP,                  swReg->aoc64Sp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_CASC128_SP,                swReg->casc128Sp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_NOROPR,                    swReg->noRoPR);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAP_2_TPHC_SP,                   swReg->tphcSp);

  *hwReg_DEV_CAP_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_devCap2_reg */


/*****************************************************************************  
 * Read and split up the Device Status and Control Register 2 register
 ****************************************************************************/  
pcieRet_e pciev1_read_devStatCtrl2_reg
(
  volatile const Uint32 *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAS_2;

  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_CPL_TIMEOUT_VALUE, swReg->cmplTo);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_CPL_TIMEOUT_DIS,   swReg->cmplToDis);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_ARI_FWD_SP,        swReg->ariFwdSp);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_AOP_REQ_EN,        swReg->aopReqEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_AOP_EG_BLK,        swReg->aopEgBlk);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_IDO_REQ_EN,        swReg->idoReqEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_IDO_CPL_EN,        swReg->idoCplEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_LTR_EN,            swReg->ltrEn);
  pcie_getbits(val, CSL_EPCFGDBICS_DEV_CAS_2_OBFF_EN,           swReg->obffEn);

  return pcie_RET_OK;
} /* pciev1_read_devStatCtrl2_reg */


/*****************************************************************************  
 * Combine and write the Device Status and Control Register 2 register
 ****************************************************************************/  
pcieRet_e pciev1_write_devStatCtrl2_reg
(
  volatile Uint32 *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_CPL_TIMEOUT_VALUE, swReg->cmplTo);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_CPL_TIMEOUT_DIS,   swReg->cmplToDis);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_ARI_FWD_SP,        swReg->ariFwdSp);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_AOP_REQ_EN,        swReg->aopReqEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_AOP_EG_BLK,        swReg->aopEgBlk);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_IDO_REQ_EN,        swReg->idoReqEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_IDO_CPL_EN,        swReg->idoCplEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_LTR_EN,            swReg->ltrEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_DEV_CAS_2_OBFF_EN,           swReg->obffEn);

  *hwReg_DEV_CAS_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_devStatCtrl2_reg */

/*****************************************************************************  
 * Read and split up the Link Capabilities 2 register
 ****************************************************************************/  
pcieRet_e pciev1_read_linkCap2_reg
(
  volatile const Uint32 *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAP_2;

  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_2_SP_LS_VEC,    swReg->spLsVec);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAP_2_CROSSLINK_SP, swReg->crosslinkSp);

  return pcie_RET_OK;
} /* pciev1_read_linkCap2_reg */


/*****************************************************************************  
 * Combine and write the Link Capabilites 2 register
 ****************************************************************************/  
pcieRet_e pciev1_write_linkCap2_reg
(
  volatile Uint32 *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_2_SP_LS_VEC,    swReg->spLsVec);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAP_2_CROSSLINK_SP, swReg->crosslinkSp);

  *hwReg_LNK_CAP_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_linkCap2_reg */


/*****************************************************************************  
 * Read and split up the Link Control 2 register
 ****************************************************************************/  
pcieRet_e pciev1_read_linkCtrl2_reg
(
  volatile const Uint32 *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg 
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAS_2;

  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_TRGT_LINK_SPEED,  swReg->tgtSpeed);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_ENTR_COMPL,       swReg->entrCompl);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_HW_AUTO_SP_DIS,   swReg->hwAutoSpeedDis);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_SEL_DEEMP,        swReg->selDeemph);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_TX_MARGIN,        swReg->txMargin);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_ENT_MOD_COMPL,    swReg->entrModCompl);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_COMPL_SOS,        swReg->cmplSos);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_COMPL_PRST_DEEPH, swReg->complPrstDeemph);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_DEEMPH_LEVEL,     swReg->deEmph);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_COMPLETE,      swReg->eqComplete);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_PH1,           swReg->eqPh1);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_PH2,           swReg->eqPh2);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_PH3,           swReg->eqPh3);
  pcie_getbits(val, CSL_EPCFGDBICS_LNK_CAS_2_LINK_EQ_REQ,      swReg->linkEqReq);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->pollDeemph   = 0u; /* Note: low bit of complPrstDeeph */
  swReg->downCompPres = 0u;
  swReg->drsMsgRecv   = 0u;

  return pcie_RET_OK;
} /* pciev1_read_linkCtrl2_reg */


/*****************************************************************************  
 * Combine and write the Link Control 2 register
 ****************************************************************************/  
pcieRet_e pciev1_write_linkCtrl2_reg
(
  volatile Uint32 *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_TRGT_LINK_SPEED,  swReg->tgtSpeed);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_ENTR_COMPL,       swReg->entrCompl);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_HW_AUTO_SP_DIS,   swReg->hwAutoSpeedDis);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_SEL_DEEMP,        swReg->selDeemph);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_TX_MARGIN,        swReg->txMargin);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_ENT_MOD_COMPL,    swReg->entrModCompl);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_COMPL_SOS,        swReg->cmplSos);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_COMPL_PRST_DEEPH, swReg->complPrstDeemph);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_DEEMPH_LEVEL,     swReg->deEmph);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_COMPLETE,      swReg->eqComplete);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_PH1,           swReg->eqPh1);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_PH2,           swReg->eqPh2);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_EQ_PH3,           swReg->eqPh3);
  pcie_setbits(new_val, CSL_EPCFGDBICS_LNK_CAS_2_LINK_EQ_REQ,      swReg->linkEqReq);

  *hwReg_LNK_CAS_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_linkCtrl2_reg */


/*****************************************************************************
 **********  PCIe EXTENDED CAPABILITIES  REGISTERS **********************
 ****************************************************************************/
/* No extended capabilities are defined for rev 1 hw */

/* Nothing past this point */

