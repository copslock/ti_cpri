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
 *  File Name: hyplnk.c
 *
 *  Processing/configuration functions for the HyperLink driver.
 *
 */

#include "hyplnk.h"
#include "hyplnkloc.h"
#include <ti/csl/cslr_vusr.h>

/* Global Variable which describes the HYPLNK LLD Version Information */
const char HYPLNKLLDVersionStr[] = hyplnk_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

/*****************************************************************************
 * Forward reference prototypes (these writes are used with table reads)
 ****************************************************************************/
static hyplnkRet_e hyplnk_write_RXPrivIDIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXPrivIDIdxReg_t *reg
);
static hyplnkRet_e hyplnk_write_RXSegIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXSegIdxReg_t *reg
);
static hyplnkRet_e hyplnk_write_intCtrlIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntCtrlIdxReg_t *reg
);
static hyplnkRet_e hyplnk_write_intPtrIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntPtrIdxReg_t *reg
);

/*****************************************************************************
 * check if handle is valid
 ****************************************************************************/
int hyplnk_check_handle_fcn (Hyplnk_Handle handle)
{
  int i;

  if (handle) {
    for (i = 0; i < hyplnk_MAX_PERIPHS; i++) {
      if (handle == hyplnkLObj.cfg.dev.bases[i].cfgBase) {
        return 1;
      }
    }
  }
  return 0;
}
/*****************************************************************************
 * Read and split up the revision register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_rev_reg, ".text:Hyplnk:cfg:read_regs:rev_reg");
static hyplnkRet_e hyplnk_read_rev_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRevReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->REV;

  hyplnk_getbits(val, CSL_VUSR_REV_SCHEME,                 reg->scheme);
  hyplnk_getbits(val, CSL_VUSR_REV_BU,                     reg->bu);
  hyplnk_getbits(val, CSL_VUSR_REV_MODID,                  reg->func);
  hyplnk_getbits(val, CSL_VUSR_REV_RTL_VER,                reg->rtl);
  hyplnk_getbits(val, CSL_VUSR_REV_REVMAJ,                 reg->revMaj);
  hyplnk_getbits(val, CSL_VUSR_REV_CUSTOMER,               reg->cust);
  hyplnk_getbits(val, CSL_VUSR_REV_REVMIN,                 reg->revMin);

  return hyplnk_RET_OK;
} /* hyplnk_read_rev_reg */

/*****************************************************************************
 * Read and split up the control register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_control_reg, ".text:Hyplnk:cfg:read_regs:control_reg");
static hyplnkRet_e hyplnk_read_control_reg
(
  CSL_VusrRegs       *baseAddr, 
  hyplnkControlReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CTL;

  hyplnk_getbits(val, CSL_VUSR_CTL_INTLOCAL,               reg->intLocal);
  hyplnk_getbits(val, CSL_VUSR_CTL_INTENABLE,              reg->statusIntEnable);
  hyplnk_getbits(val, CSL_VUSR_CTL_INTVEC,                 reg->statusIntVec);
  hyplnk_getbits(val, CSL_VUSR_CTL_INT2CFG,                reg->int2cfg);
  hyplnk_getbits(val, CSL_VUSR_CTL_SERIAL_STOP,            reg->serialStop);
  hyplnk_getbits(val, CSL_VUSR_CTL_LOOPBACK,               reg->iLoop);
  hyplnk_getbits(val, CSL_VUSR_CTL_RESET,                  reg->reset);

  return hyplnk_RET_OK;
} /* hyplnk_read_control_reg */

/*****************************************************************************
 * Read and split up the status register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_status_reg, ".text:Hyplnk:cfg:read_regs:status_reg");
static hyplnkRet_e hyplnk_read_status_reg
(
  CSL_VusrRegs      *baseAddr, 
  hyplnkStatusReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->STS;

  hyplnk_getbits(val, CSL_VUSR_STS_SWIDTHIN,               reg->swidthin);
  hyplnk_getbits(val, CSL_VUSR_STS_SWIDTHOUT,              reg->swidthout);
  hyplnk_getbits(val, CSL_VUSR_STS_SERIAL_HALT,            reg->serialHalt);
  hyplnk_getbits(val, CSL_VUSR_STS_PLL_UNLOCK,             reg->pllUnlock);
  hyplnk_getbits(val, CSL_VUSR_STS_RPEND,                  reg->rPend);
  hyplnk_getbits(val, CSL_VUSR_STS_IFLOW,                  reg->iFlow);
  hyplnk_getbits(val, CSL_VUSR_STS_OFLOW,                  reg->oFlow);
  hyplnk_getbits(val, CSL_VUSR_STS_RERROR,                 reg->rError);
  hyplnk_getbits(val, CSL_VUSR_STS_LERROR,                 reg->lError);
  hyplnk_getbits(val, CSL_VUSR_STS_NFEMPTY3,               reg->nfEmpty3);
  hyplnk_getbits(val, CSL_VUSR_STS_NFEMPTY2,               reg->nfEmpty2);
  hyplnk_getbits(val, CSL_VUSR_STS_NFEMPTY1,               reg->nfEmpty1);
  hyplnk_getbits(val, CSL_VUSR_STS_NFEMPTY0,               reg->nfEmpty0);
  hyplnk_getbits(val, CSL_VUSR_STS_SPEND,                  reg->sPend);
  hyplnk_getbits(val, CSL_VUSR_STS_MPEND,                  reg->mPend);
  hyplnk_getbits(val, CSL_VUSR_STS_LINK,                   reg->link);

  return hyplnk_RET_OK;
} /* hyplnk_read_status_reg */

/*****************************************************************************
 * Read and split up the interrupt priority vector status/clear register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intPriVec_reg, ".text:Hyplnk:cfg:read_regs:intPriVec_reg");
static hyplnkRet_e hyplnk_read_intPriVec_reg
(
  CSL_VusrRegs         *baseAddr, 
  hyplnkIntPriVecReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->INT_PRI_VEC;

  hyplnk_getbits(val, CSL_VUSR_INT_PRI_VEC_NOINTPEND,      reg->noIntPend);
  hyplnk_getbits(val, CSL_VUSR_INT_PRI_VEC_INTSTAT,        reg->intStat);

  return hyplnk_RET_OK;
} /* hyplnk_read_intPriVec_reg */

/*****************************************************************************
 * Read and split up the interrupt status/clear register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intStatusClr_reg, ".text:Hyplnk:cfg:read_regs:intStatusClr_reg");
static hyplnkRet_e hyplnk_read_intStatusClr_reg
(
  CSL_VusrRegs            *baseAddr, 
  hyplnkIntStatusClrReg_t *reg
)
{
  reg->intClr = baseAddr->INT_CLR;

  return hyplnk_RET_OK;
} /* hyplnk_read_intStatusClr_reg */

/*****************************************************************************
 * Read and split up the interrupt pending/set register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intPendSet_reg, ".text:Hyplnk:cfg:read_regs:intPendSet_reg");
static hyplnkRet_e hyplnk_read_intPendSet_reg
(
  CSL_VusrRegs          *baseAddr, 
  hyplnkIntPendSetReg_t *reg
)
{
  reg->intSet = baseAddr->INT_SET;

  return hyplnk_RET_OK;
} /* hyplnk_read_intPendSet_reg */

/*****************************************************************************
 * Read and split up the generate software interrupt register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_genSoftInt_reg, ".text:Hyplnk:cfg:read_regs:genSoftInt_reg");
static hyplnkRet_e hyplnk_read_genSoftInt_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkGenSoftIntReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SW_INT;

  /* The following bit is present in CSL but not documentation (TBD) */
  /* hyplnk_getbits(val, CSL_VUSR_SW_INT_SWMSTID,     XXXXXXXXXXXX); */
  hyplnk_getbits(val, CSL_VUSR_SW_INT_EOI_FLAG,            reg->eoiFlag);
  hyplnk_getbits(val, CSL_VUSR_SW_INT_IVECTOR,             reg->iVector);

  return hyplnk_RET_OK;
} /* hyplnk_read_genSoftInt_reg */

/*****************************************************************************
 * Read and split up the TX address overlay register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_TXAddrOvly_reg, ".text:Hyplnk:cfg:read_regs:TXAddrOvly_reg");
static hyplnkRet_e hyplnk_read_TXAddrOvly_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkTXAddrOvlyReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->TX_SEL_CTL;

  hyplnk_getbits(val, CSL_VUSR_TX_SEL_CTL_TXSECOVL,        reg->txSecOvl);
  hyplnk_getbits(val, CSL_VUSR_TX_SEL_CTL_TXPRIVIDOVL,     reg->txPrivIDOvl);
  hyplnk_getbits(val, CSL_VUSR_TX_SEL_CTL_TXIGNMSK,        reg->txIgnMask);

  return hyplnk_RET_OK;
} /* hyplnk_read_TXAddrOvly_reg */

/*****************************************************************************
 * Read and split up the RX address selector register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXAddrSel_reg, ".text:Hyplnk:cfg:read_regs:RXAddrSel_reg");
static hyplnkRet_e hyplnk_read_RXAddrSel_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXAddrSelReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->RX_SEL_CTL;

  hyplnk_getbits(val, CSL_VUSR_RX_SEL_CTL_RXSECHI,         reg->rxSecHi);
  hyplnk_getbits(val, CSL_VUSR_RX_SEL_CTL_RXSECLO,         reg->rxSecLo);
  hyplnk_getbits(val, CSL_VUSR_RX_SEL_CTL_RXSECSEL,        reg->rxSecSel);
  hyplnk_getbits(val, CSL_VUSR_RX_SEL_CTL_RXPRIVIDSEL,     reg->rxPrivIDSel);
  hyplnk_getbits(val, CSL_VUSR_RX_SEL_CTL_RXSEGSEL,        reg->rxSegSel);

  return hyplnk_RET_OK;
} /* hyplnk_read_RXAddrSel_reg */

/*****************************************************************************
 * Read and split up the RX PrivID index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXPrivIDIdx_reg, ".text:Hyplnk:cfg:read_regs:RXPrivIDIdx_reg");
static hyplnkRet_e hyplnk_read_RXPrivIDIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXPrivIDIdxReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->RX_PRIV_IDX;

  hyplnk_getbits(val, CSL_VUSR_RX_PRIV_IDX_RXPRIVID_IDX,   reg->rxPrivIDIdx);

  return hyplnk_RET_OK;
} /* hyplnk_read_RXPrivIDIdx_reg */

/*****************************************************************************
 * Read and split up the RX PrivID value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXPrivIDVal_reg, ".text:Hyplnk:cfg:read_regs:RXPrivIDVal_reg");
static hyplnkRet_e hyplnk_read_RXPrivIDVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXPrivIDValReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->RX_PRIV_VAL;

  hyplnk_getbits(val, CSL_VUSR_RX_PRIV_VAL_RXPRIVID_VAL,   reg->rxPrivIDVal);

  return hyplnk_RET_OK;
} /* hyplnk_read_RXPrivIDVal_reg */

/*****************************************************************************
 * Read and split up the RX Segment Index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXSegIdx_reg, ".text:Hyplnk:cfg:read_regs:RXSegIdx_reg");
static hyplnkRet_e hyplnk_read_RXSegIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXSegIdxReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->RX_SEG_IDX;

  hyplnk_getbits(val, CSL_VUSR_RX_SEG_IDX_RXSEG_IDX,       reg->rxSegIdx);

  return hyplnk_RET_OK;
} /* hyplnk_read_RXSegIdx_reg */

/*****************************************************************************
 * Read and split up the RX Segment Value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXSegVal_reg, ".text:Hyplnk:cfg:read_regs:RXSegVal_reg");
static hyplnkRet_e hyplnk_read_RXSegVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXSegValReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->RX_SEG_VAL;

  hyplnk_getbits(val, CSL_VUSR_RX_SEG_VAL_RXSEG_VAL,       reg->rxSegVal);
  hyplnk_getbits(val, CSL_VUSR_RX_SEG_VAL_RXLEN_VAL,       reg->rxLenVal);

  return hyplnk_RET_OK;
} /* hyplnk_read_RXSegVal_reg */

/*****************************************************************************
 * Read and split up the chip version register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_chipVer_reg, ".text:Hyplnk:cfg:read_regs:chipVer_reg");
static hyplnkRet_e hyplnk_read_chipVer_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkChipVerReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CHIP_ID_VER;

  hyplnk_getbits(val, CSL_VUSR_CHIP_ID_VER_DEVREV,         reg->devRev);
  hyplnk_getbits(val, CSL_VUSR_CHIP_ID_VER_DEVID,          reg->devID);

  return hyplnk_RET_OK;
} /* hyplnk_read_chipVer_reg */

/*****************************************************************************
 * Read and split up the lane power management register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_lanePwrMgmt_reg, ".text:Hyplnk:cfg:read_regs:lanePwrMgmt_reg");
static hyplnkRet_e hyplnk_read_lanePwrMgmt_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkLanePwrMgmtReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PWR;

  hyplnk_getbits(val, CSL_VUSR_PWR_H2L,                    reg->H2L);
  hyplnk_getbits(val, CSL_VUSR_PWR_L2H,                    reg->L2H);
  hyplnk_getbits(val, CSL_VUSR_PWR_PWC,                    reg->PWC);
  hyplnk_getbits(val, CSL_VUSR_PWR_HIGHSPEED,              reg->highSpeed);
  hyplnk_getbits(val, CSL_VUSR_PWR_QUADLANE,               reg->quadLane);
  hyplnk_getbits(val, CSL_VUSR_PWR_SINGLELANE,             reg->singleLane);
  hyplnk_getbits(val, CSL_VUSR_PWR_ZEROLANE,               reg->zeroLane);

  return hyplnk_RET_OK;
} /* hyplnk_read_lanePwrMgmt_reg */

/*****************************************************************************
 * Read and split up the ECC error counters register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_ECCErrors_reg, ".text:Hyplnk:cfg:read_regs:ECCErrors_reg");
static hyplnkRet_e hyplnk_read_ECCErrors_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkECCErrorsReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->ECC_CNTR;

  hyplnk_getbits(val, CSL_VUSR_ECC_CNTR_SGL_ERR_COR,       reg->sglErrCor);
  hyplnk_getbits(val, CSL_VUSR_ECC_CNTR_DBL_ERR_DET,       reg->dblErrDet);

  return hyplnk_RET_OK;
} /* hyplnk_read_ECCErrors_reg */

/*****************************************************************************
 * Read and split up the link status register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_linkStatus_reg, ".text:Hyplnk:cfg:read_regs:linkStatus_reg");
static hyplnkRet_e hyplnk_read_linkStatus_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkLinkStatusReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->LINK_STS;

  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TXPLS_REQ,         reg->txPlsReq);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TXPLS_ACK,         reg->txPlsAck);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TXPM_REQ,          reg->txPmReq);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TX_RSYNC,          reg->txRSync);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TXPLSOK,           reg->txPlsOK);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TX_PHY_EN,         reg->txPhyEn);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_TXFLOW_STS,        reg->txFlowSts);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RXPLS_REQ,         reg->rxPlsReq);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RXPLS_ACK,         reg->rxPlsAck);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RXPM_REQ,          reg->rxPmReq);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RX_LSYNC,          reg->rxLSync);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RX_ONE_ID,         reg->rxOneID);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RX_PHY_EN,         reg->rxPhyEn);
  hyplnk_getbits(val, CSL_VUSR_LINK_STS_RX_PHY_POL,        reg->rxPhyPol);

  return hyplnk_RET_OK;
} /* hyplnk_read_linkStatus_reg */

/*****************************************************************************
 * Read and split up the interrupt control index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intCtrlIdx_reg, ".text:Hyplnk:cfg:read_regs:intCtrlIdx_reg");
static hyplnkRet_e hyplnk_read_intCtrlIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntCtrlIdxReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->INT_CTL_IDX;

  hyplnk_getbits(val, CSL_VUSR_INT_CTL_IDX_ICIDX,          reg->intCtrlIdx);

  return hyplnk_RET_OK;
} /* hyplnk_read_intCtrlIdx_reg */

/*****************************************************************************
 * Read and split up the interrupt control value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intCtrlVal_reg, ".text:Hyplnk:cfg:read_regs:intCtrlVal_reg");
static hyplnkRet_e hyplnk_read_intCtrlVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntCtrlValReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->INT_CTL_VAL;

  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_INTEN,          reg->intEn);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_INTTYPE,        reg->intType);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_INTPOL,         reg->intPol);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_ISEC,           reg->iSec);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_SIEN,           reg->SIEN);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_DNID,           reg->DNID);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_MPS,            reg->mps);
  hyplnk_getbits(val, CSL_VUSR_INT_CTL_VAL_VECTOR,         reg->vector);

  return hyplnk_RET_OK;
} /* hyplnk_read_intCtrlVal_reg */

/*****************************************************************************
 * Read and split up the interrupt pointer index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intPtrIdx_reg, ".text:Hyplnk:cfg:read_regs:intPtrIdx_reg");
static hyplnkRet_e hyplnk_read_intPtrIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntPtrIdxReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->INT_PTR_IDX;

  hyplnk_getbits(val, CSL_VUSR_INT_PTR_IDX_IPIDX,          reg->intPtrIdx);

  return hyplnk_RET_OK;
} /* hyplnk_read_intPtrIdx_reg */

/*****************************************************************************
 * Read and split up the interrupt pointer value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intPtrVal_reg, ".text:Hyplnk:cfg:read_regs:intPtrVal_reg");
static hyplnkRet_e hyplnk_read_intPtrVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntPtrValReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->INT_PTR_VAL;

  hyplnk_getbits(val, CSL_VUSR_INT_PTR_VAL_INTPTR,         reg->intPtr);

  return hyplnk_RET_OK;
} /* hyplnk_read_intPtrVal_reg */

/*****************************************************************************
 * Read and split up the SERDES control and status register #1
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_serdesControl1_reg, ".text:Hyplnk:cfg:read_regs:serdesControl1");
static hyplnkRet_e hyplnk_read_serdesControl1_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl1Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SERDES_CTL_STS1;

  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS1_SLEEP_CNT,   reg->sleepCnt);
  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS1_DISABLE_CNT, reg->disableCnt);

  return hyplnk_RET_OK;
} /* hyplnk_read_serdesControl1_reg */

/*****************************************************************************
 * Read and split up the SERDES control and status register #2
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_serdesControl2_reg, ".text:Hyplnk:cfg:read_regs:serdesControl2");
static hyplnkRet_e hyplnk_read_serdesControl2_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl2Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SERDES_CTL_STS2;

  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS2_S2CTL,      reg->s2Ctl);

  return hyplnk_RET_OK;
} /* hyplnk_read_serdesControl2_reg */

/*****************************************************************************
 * Read and split up the SERDES control and status register #3
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_serdesControl3_reg, ".text:Hyplnk:cfg:read_regs:serdesControl3");
static hyplnkRet_e hyplnk_read_serdesControl3_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl3Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SERDES_CTL_STS3;

  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS3_S3CTL,      reg->s3Ctl);

  return hyplnk_RET_OK;
} /* hyplnk_read_serdesControl3_reg */

/*****************************************************************************
 * Read and split up the SERDES control and status register #4
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_serdesControl4_reg, ".text:Hyplnk:cfg:read_regs:serdesControl4");
static hyplnkRet_e hyplnk_read_serdesControl4_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl4Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SERDES_CTL_STS4;

  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS4_DVQUICK,    reg->dvQuick);
  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS4_S4CTL,      reg->s4Ctl);
  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS4_TX_SPC,     reg->txSpc);
  hyplnk_getbits(val, CSL_VUSR_SERDES_CTL_STS4_RX_SPC,     reg->rxSpc);

  return hyplnk_RET_OK;
} /* hyplnk_read_serdesControl4_reg */

/*****************************************************************************
 * Read the entire RX PrivID table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXPrivIDTbl, ".text:Hyplnk:cfg:read_tbls:RXPrivIDTbl");
static hyplnkRet_e hyplnk_read_RXPrivIDTbl
(
  CSL_VusrRegs        *baseAddr, 
  hyplnkRXPrivIDTbl_t *tbl
)
{
  hyplnkRXPrivIDIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_RXPrivIDIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Peform the requested read */
  for (i = 0; i < hyplnk_RX_PRIVID_TBL_ENTS; i++) {
    newIdx.rxPrivIDIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_RXPrivIDIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_read_RXPrivIDVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_RXPrivIDIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_read_RXPrivIDTbl */

/*****************************************************************************
 * Read the entire RX segment table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_RXSegTbl, ".text:Hyplnk:cfg:read_tbls:RXSegTbl");
static hyplnkRet_e hyplnk_read_RXSegTbl
(
  CSL_VusrRegs     *baseAddr, 
  hyplnkRXSegTbl_t *tbl
)
{
  hyplnkRXSegIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_RXSegIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Check that hyplnk_RX_SEG_TBL_ENTS is correctly defined */
  for (i = 0; i < hyplnk_RX_SEG_TBL_ENTS; i++) {
    newIdx.rxSegIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_RXSegIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_read_RXSegVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_RXSegIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_read_RXSegTbl */

/*****************************************************************************
 * Read the entire interrupt control table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intCtrlTbl, ".text:Hyplnk:cfg:read_tbls:intCtrlTbl");
static hyplnkRet_e hyplnk_read_intCtrlTbl
(
  CSL_VusrRegs       *baseAddr, 
  hyplnkIntCtrlTbl_t *tbl
)
{
  hyplnkIntCtrlIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_intCtrlIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Peform the requested rite */
  for (i = 0; i < hyplnk_INT_CTRL_TBL_ENTS; i++) {
    newIdx.intCtrlIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_intCtrlIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_read_intCtrlVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_intCtrlIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_read_intCtrlTbl */

/*****************************************************************************
 * Read the entire interrupt pointer table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_read_intPtrTbl, ".text:Hyplnk:cfg:read_tbls:intPtrTbl");
static hyplnkRet_e hyplnk_read_intPtrTbl
(
  CSL_VusrRegs      *baseAddr, 
  hyplnkIntPtrTbl_t *tbl
)
{
  hyplnkIntPtrIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_intPtrIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Peform the requested rite */
  for (i = 0; i < hyplnk_INT_PTR_TBL_ENTS; i++) {
    newIdx.intPtrIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_intPtrIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_read_intPtrVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_intPtrIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_read_intPtrTbl */

#pragma CODE_SECTION (Hyplnk_readRegs, ".text:Hyplnk:cfg:readRegs");
/*********************************************************************
 * FUNCTION PURPOSE: Reads any/all registers/tables
 ********************************************************************/
hyplnkRet_e Hyplnk_readRegs 
(
  Hyplnk_Handle      handle,   /**< [in] The HYPLNK LLD instance identifier */
  hyplnkLocation_e   location, /**< [in] Local or remote peripheral */
  hyplnkRegisters_t *readRegs  /**< [in/out] List of registers to read */
)
{
  CSL_VusrRegs *baseAddr = (CSL_VusrRegs *)handle;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  if (! hyplnkLObjIsValid) {
    return hyplnk_RET_NO_INIT;
  }

  hyplnk_check_handle(handle);

  if (location == hyplnk_LOCATION_REMOTE) {
    baseAddr = (CSL_VusrRegs *)&baseAddr->REM_REGS;
  }

  /* Single register operations */
  if (readRegs->rev) {
    hyplnk_check_result(retVal, hyplnk_read_rev_reg (baseAddr, readRegs->rev));
  }
  if (readRegs->control) {
    hyplnk_check_result(retVal, hyplnk_read_control_reg (baseAddr, readRegs->control));
  }
  if (readRegs->status) {
    hyplnk_check_result(retVal, hyplnk_read_status_reg (baseAddr, readRegs->status));
  }
  if (readRegs->intPriVec) {
    hyplnk_check_result(retVal, hyplnk_read_intPriVec_reg (baseAddr, readRegs->intPriVec));
  }
  if (readRegs->intStatusClr) {
    hyplnk_check_result(retVal, hyplnk_read_intStatusClr_reg (baseAddr, readRegs->intStatusClr));
  }
  if (readRegs->intPendSet) {
    hyplnk_check_result(retVal, hyplnk_read_intPendSet_reg (baseAddr, readRegs->intPendSet));
  }
  if (readRegs->genSoftInt) {
    hyplnk_check_result(retVal, hyplnk_read_genSoftInt_reg (baseAddr, readRegs->genSoftInt));
  }
  if (readRegs->TXAddrOvly) {
    hyplnk_check_result(retVal, hyplnk_read_TXAddrOvly_reg (baseAddr, readRegs->TXAddrOvly));
  }
  if (readRegs->RXAddrSel) {
    hyplnk_check_result(retVal, hyplnk_read_RXAddrSel_reg (baseAddr, readRegs->RXAddrSel));
  }
  if (readRegs->RXPrivIDIdx) {
    hyplnk_check_result(retVal, hyplnk_read_RXPrivIDIdx_reg (baseAddr, readRegs->RXPrivIDIdx));
  }
  if (readRegs->RXPrivIDVal) {
    hyplnk_check_result(retVal, hyplnk_read_RXPrivIDVal_reg (baseAddr, readRegs->RXPrivIDVal));
  }
  if (readRegs->RXSegIdx) {
    hyplnk_check_result(retVal, hyplnk_read_RXSegIdx_reg (baseAddr, readRegs->RXSegIdx));
  }
  if (readRegs->RXSegVal) {
    hyplnk_check_result(retVal, hyplnk_read_RXSegVal_reg (baseAddr, readRegs->RXSegVal));
  }
  if (readRegs->chipVer) {
    hyplnk_check_result(retVal, hyplnk_read_chipVer_reg (baseAddr, readRegs->chipVer));
  }
  if (readRegs->lanePwrMgmt) {
    hyplnk_check_result(retVal, hyplnk_read_lanePwrMgmt_reg (baseAddr, readRegs->lanePwrMgmt));
  }
  if (readRegs->ECCErrors) {
    hyplnk_check_result(retVal, hyplnk_read_ECCErrors_reg (baseAddr, readRegs->ECCErrors));
  }
  if (readRegs->linkStatus) {
    hyplnk_check_result(retVal, hyplnk_read_linkStatus_reg (baseAddr, readRegs->linkStatus));
  }
  if (readRegs->intCtrlIdx) {
    hyplnk_check_result(retVal, hyplnk_read_intCtrlIdx_reg (baseAddr, readRegs->intCtrlIdx));
  }
  if (readRegs->intCtrlVal) {
    hyplnk_check_result(retVal, hyplnk_read_intCtrlVal_reg (baseAddr, readRegs->intCtrlVal));
  }
  if (readRegs->intPtrIdx) {
    hyplnk_check_result(retVal, hyplnk_read_intPtrIdx_reg (baseAddr, readRegs->intPtrIdx));
  }
  if (readRegs->intPtrVal) {
    hyplnk_check_result(retVal, hyplnk_read_intPtrVal_reg (baseAddr, readRegs->intPtrVal));
  }
  if (readRegs->serdesControl1) {
    hyplnk_check_result(retVal, hyplnk_read_serdesControl1_reg (baseAddr, readRegs->serdesControl1));
  }
  if (readRegs->serdesControl2) {
    hyplnk_check_result(retVal, hyplnk_read_serdesControl2_reg (baseAddr, readRegs->serdesControl2));
  }
  if (readRegs->serdesControl3) {
    hyplnk_check_result(retVal, hyplnk_read_serdesControl3_reg (baseAddr, readRegs->serdesControl3));
  }
  if (readRegs->serdesControl4) {
    hyplnk_check_result(retVal, hyplnk_read_serdesControl4_reg (baseAddr, readRegs->serdesControl4));
  }

  /* Whole table operations */
  if (readRegs->RXPrivIDTbl) {
    hyplnk_check_result(retVal, hyplnk_read_RXPrivIDTbl (baseAddr, readRegs->RXPrivIDTbl));
  }
  if (readRegs->RXSegTbl) {
    hyplnk_check_result(retVal, hyplnk_read_RXSegTbl (baseAddr, readRegs->RXSegTbl));
  }
  if (readRegs->intCtrlTbl) {
    hyplnk_check_result(retVal, hyplnk_read_intCtrlTbl (baseAddr, readRegs->intCtrlTbl));
  }
  if (readRegs->intPtrTbl) {
    hyplnk_check_result(retVal, hyplnk_read_intPtrTbl (baseAddr, readRegs->intPtrTbl));
  }

  return retVal;
} /* Hyplnk_readRegs */

/*********************************************************************
 * The following hyplnk_write_* functions are used internally
 * in this file.  If it becomes necessary to optimize code by
 * avoiding the combined Hyplnk_read_regs and Hyplnk_write_regs
 * APIs, then these can be moved to public APIs as inlines.
 *
 * Also, if these functions are inlined, then the CSL files will
 * be recursively included in the user's app.
 ********************************************************************/
/*****************************************************************************
 * Combine and write the revision register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_rev_reg, ".text:Hyplnk:cfg:write_regs:rev_reg");
static hyplnkRet_e hyplnk_write_rev_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRevReg_t *reg
)
{
  /* Can't write the revision register */
  return hyplnk_USELESS_WRITE;
} /* hyplnk_write_rev_reg */

/*****************************************************************************
 * Combine and write the control register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_control_reg, ".text:Hyplnk:cfg:write_regs:control_reg");
static hyplnkRet_e hyplnk_write_control_reg
(
  CSL_VusrRegs       *baseAddr, 
  hyplnkControlReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_CTL_INTLOCAL,           reg->intLocal);
  hyplnk_setbits(new_val, CSL_VUSR_CTL_INTENABLE,          reg->statusIntEnable);
  hyplnk_setbits(new_val, CSL_VUSR_CTL_INTVEC,             reg->statusIntVec);
  hyplnk_setbits(new_val, CSL_VUSR_CTL_INT2CFG,            reg->int2cfg);
  hyplnk_setbits(new_val, CSL_VUSR_CTL_SERIAL_STOP,        reg->serialStop);
  hyplnk_setbits(new_val, CSL_VUSR_CTL_LOOPBACK,           reg->iLoop);
  hyplnk_setbits(new_val, CSL_VUSR_CTL_RESET,              reg->reset);
  baseAddr->CTL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_control_reg */

/*****************************************************************************
 * Combine and write the status register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_status_reg, ".text:Hyplnk:cfg:write_regs:status_reg");
static hyplnkRet_e hyplnk_write_status_reg
(
  CSL_VusrRegs      *baseAddr, 
  hyplnkStatusReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_STS_SWIDTHIN,           reg->swidthin);
  hyplnk_setbits(new_val, CSL_VUSR_STS_SWIDTHOUT,          reg->swidthout);
  hyplnk_setbits(new_val, CSL_VUSR_STS_SERIAL_HALT,        reg->serialHalt);
  hyplnk_setbits(new_val, CSL_VUSR_STS_PLL_UNLOCK,         reg->pllUnlock);
  hyplnk_setbits(new_val, CSL_VUSR_STS_RPEND,              reg->rPend);
  hyplnk_setbits(new_val, CSL_VUSR_STS_IFLOW,              reg->iFlow);
  hyplnk_setbits(new_val, CSL_VUSR_STS_OFLOW,              reg->oFlow);
  hyplnk_setbits(new_val, CSL_VUSR_STS_RERROR,             reg->rError);
  hyplnk_setbits(new_val, CSL_VUSR_STS_LERROR,             reg->lError);
  hyplnk_setbits(new_val, CSL_VUSR_STS_NFEMPTY3,           reg->nfEmpty3);
  hyplnk_setbits(new_val, CSL_VUSR_STS_NFEMPTY2,           reg->nfEmpty2);
  hyplnk_setbits(new_val, CSL_VUSR_STS_NFEMPTY1,           reg->nfEmpty1);
  hyplnk_setbits(new_val, CSL_VUSR_STS_NFEMPTY0,           reg->nfEmpty0);
  hyplnk_setbits(new_val, CSL_VUSR_STS_SPEND,              reg->sPend);
  hyplnk_setbits(new_val, CSL_VUSR_STS_MPEND,              reg->mPend);
  hyplnk_setbits(new_val, CSL_VUSR_STS_LINK,               reg->link);

  baseAddr->STS = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_status_reg */

/*****************************************************************************
 * Combine and write the interrupt priority vector status/clear register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intPriVec_reg, ".text:Hyplnk:cfg:write_regs:intPriVec_reg");
static hyplnkRet_e hyplnk_write_intPriVec_reg
(
  CSL_VusrRegs         *baseAddr, 
  hyplnkIntPriVecReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_INT_PRI_VEC_NOINTPEND,  reg->noIntPend);
  hyplnk_setbits(new_val, CSL_VUSR_INT_PRI_VEC_INTSTAT,    reg->intStat);

  baseAddr->INT_PRI_VEC = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_intPriVec_reg */

/*****************************************************************************
 * Combine and write the interrupt status/clear register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intStatusClr_reg, ".text:Hyplnk:cfg:write_regs:intStatusClr_reg");
static hyplnkRet_e hyplnk_write_intStatusClr_reg
(
  CSL_VusrRegs            *baseAddr, 
  hyplnkIntStatusClrReg_t *reg
)
{
  baseAddr->INT_CLR = reg->raw = reg->intClr;

  return hyplnk_RET_OK;
} /* hyplnk_write_intStatusClr_reg */

/*****************************************************************************
 * Combine and write the interrupt pending/set register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intPendSet_reg, ".text:Hyplnk:cfg:write_regs:intPendSet_reg");
static hyplnkRet_e hyplnk_write_intPendSet_reg
(
  CSL_VusrRegs          *baseAddr, 
  hyplnkIntPendSetReg_t *reg
)
{
  baseAddr->INT_SET = reg->raw = reg->intSet;

  return hyplnk_RET_OK;
} /* hyplnk_write_intPendSet_reg */

/*****************************************************************************
 * Combine and write the generate software interrupt register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_genSoftInt_reg, ".text:Hyplnk:cfg:write_regs:genSoftInt_reg");
static hyplnkRet_e hyplnk_write_genSoftInt_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkGenSoftIntReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  /* The following bit is present in CSL but not documentation (TBD) */
  /* hyplnk_setbits(new_val, CSL_VUSR_SW_INT_SWMSTID,     XXXXXXXXXXXX); */
  hyplnk_setbits(new_val, CSL_VUSR_SW_INT_EOI_FLAG,        reg->eoiFlag);
  hyplnk_setbits(new_val, CSL_VUSR_SW_INT_IVECTOR,         reg->iVector);

  baseAddr->SW_INT = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_genSoftInt_reg */

/*****************************************************************************
 * Combine and write the TX address overlay register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_TXAddrOvly_reg, ".text:Hyplnk:cfg:write_regs:TXAddrOvly_reg");
static hyplnkRet_e hyplnk_write_TXAddrOvly_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkTXAddrOvlyReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_TX_SEL_CTL_TXSECOVL,    reg->txSecOvl);
  hyplnk_setbits(new_val, CSL_VUSR_TX_SEL_CTL_TXPRIVIDOVL, reg->txPrivIDOvl);
  hyplnk_setbits(new_val, CSL_VUSR_TX_SEL_CTL_TXIGNMSK,    reg->txIgnMask);

  baseAddr->TX_SEL_CTL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_TXAddrOvly_reg */

/*****************************************************************************
 * Combine and write the RX address selector register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXAddrSel_reg, ".text:Hyplnk:cfg:write_regs:RXAddrSel_reg");
static hyplnkRet_e hyplnk_write_RXAddrSel_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXAddrSelReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_RX_SEL_CTL_RXSECHI,     reg->rxSecHi);
  hyplnk_setbits(new_val, CSL_VUSR_RX_SEL_CTL_RXSECLO,     reg->rxSecLo);
  hyplnk_setbits(new_val, CSL_VUSR_RX_SEL_CTL_RXSECSEL,    reg->rxSecSel);
  hyplnk_setbits(new_val, CSL_VUSR_RX_SEL_CTL_RXPRIVIDSEL, reg->rxPrivIDSel);
  hyplnk_setbits(new_val, CSL_VUSR_RX_SEL_CTL_RXSEGSEL,    reg->rxSegSel);

  baseAddr->RX_SEL_CTL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_RXAddrSel_reg */

/*****************************************************************************
 * Combine and write the RX PrivID value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXPrivIDIdx_reg, ".text:Hyplnk:cfg:write_regs:RXPrivIDIdx_reg");
static hyplnkRet_e hyplnk_write_RXPrivIDIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXPrivIDIdxReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_RX_PRIV_IDX_RXPRIVID_IDX, reg->rxPrivIDIdx);

  baseAddr->RX_PRIV_IDX = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_RXPrivIDIdx_reg */

/*****************************************************************************
 * Combine and write the RX PrivID value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXPrivIDVal_reg, ".text:Hyplnk:cfg:write_regs:RXPrivIDVal_reg");
static hyplnkRet_e hyplnk_write_RXPrivIDVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXPrivIDValReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_RX_PRIV_VAL_RXPRIVID_VAL, reg->rxPrivIDVal);

  baseAddr->RX_PRIV_VAL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_RXPrivIDVal_reg */

/*****************************************************************************
 * Combine and write the RX Segment Index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXSegIdx_reg, ".text:Hyplnk:cfg:write_regs:RXSegIdx_reg");
static hyplnkRet_e hyplnk_write_RXSegIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXSegIdxReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_RX_SEG_IDX_RXSEG_IDX,   reg->rxSegIdx);

  baseAddr->RX_SEG_IDX = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_RXSegIdx_reg */

/*****************************************************************************
 * Combine and write the RX Segment Value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXSegVal_reg, ".text:Hyplnk:cfg:write_regs:RXSegVal_reg");
static hyplnkRet_e hyplnk_write_RXSegVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkRXSegValReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_RX_SEG_VAL_RXSEG_VAL,   reg->rxSegVal);
  hyplnk_setbits(new_val, CSL_VUSR_RX_SEG_VAL_RXLEN_VAL,   reg->rxLenVal);

  baseAddr->RX_SEG_VAL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_RXSegVal_reg */

/*****************************************************************************
 * Combine and write the chip version register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_chipVer_reg, ".text:Hyplnk:cfg:write_regs:chipVer_reg");
static hyplnkRet_e hyplnk_write_chipVer_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkChipVerReg_t *reg
)
{
  /* Can't write the chip version register */
  return hyplnk_USELESS_WRITE;
} /* hyplnk_write_chipVer_reg */

/*****************************************************************************
 * Combine and write the lane power management register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_lanePwrMgmt_reg, ".text:Hyplnk:cfg:write_regs:lanePwrMgmt_reg");
static hyplnkRet_e hyplnk_write_lanePwrMgmt_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkLanePwrMgmtReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_PWR_H2L,                reg->H2L);
  hyplnk_setbits(new_val, CSL_VUSR_PWR_L2H,                reg->L2H);
  hyplnk_setbits(new_val, CSL_VUSR_PWR_PWC,                reg->PWC);
  hyplnk_setbits(new_val, CSL_VUSR_PWR_HIGHSPEED,          reg->highSpeed);
  hyplnk_setbits(new_val, CSL_VUSR_PWR_QUADLANE,           reg->quadLane);
  hyplnk_setbits(new_val, CSL_VUSR_PWR_SINGLELANE,         reg->singleLane);
  hyplnk_setbits(new_val, CSL_VUSR_PWR_ZEROLANE,           reg->zeroLane);

  baseAddr->PWR = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_lanePwrMgmt_reg */

/*****************************************************************************
 * Combine and write the ECC error counters register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_ECCErrors_reg, ".text:Hyplnk:cfg:write_regs:ECCErrors_reg");
static hyplnkRet_e hyplnk_write_ECCErrors_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkECCErrorsReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_ECC_CNTR_SGL_ERR_COR,   reg->sglErrCor);
  hyplnk_setbits(new_val, CSL_VUSR_ECC_CNTR_DBL_ERR_DET,   reg->dblErrDet);

  baseAddr->ECC_CNTR = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_ECCErrors_reg */

/*****************************************************************************
 * Combine and write the link status register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_linkStatus_reg, ".text:Hyplnk:cfg:write_regs:linkStatus_reg");
static hyplnkRet_e hyplnk_write_linkStatus_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkLinkStatusReg_t *reg
)
{
  /* Can't write the link status register */
  return hyplnk_USELESS_WRITE;
} /* hyplnk_write_linkStatus_reg */

/*****************************************************************************
 * Combine and write the interrupt control index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intCtrlIdx_reg, ".text:Hyplnk:cfg:write_regs:intCtrlIdx_reg");
static hyplnkRet_e hyplnk_write_intCtrlIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntCtrlIdxReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_IDX_ICIDX,      reg->intCtrlIdx);

  baseAddr->INT_CTL_IDX = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_intCtrlIdx_reg */

/*****************************************************************************
 * Combine and write the interrupt control value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intCtrlVal_reg, ".text:Hyplnk:cfg:write_regs:intCtrlVal_reg");
static hyplnkRet_e hyplnk_write_intCtrlVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntCtrlValReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_INTEN,      reg->intEn);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_INTTYPE,    reg->intType);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_INTPOL,     reg->intPol);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_ISEC,       reg->iSec);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_SIEN,       reg->SIEN);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_DNID,       reg->DNID);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_MPS,        reg->mps);
  hyplnk_setbits(new_val, CSL_VUSR_INT_CTL_VAL_VECTOR,     reg->vector);

  baseAddr->INT_CTL_VAL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_intCtrlVal_reg */

/*****************************************************************************
 * Combine and write the interrupt pointer index register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intPtrIdx_reg, ".text:Hyplnk:cfg:write_regs:intPtrIdx_reg");
static hyplnkRet_e hyplnk_write_intPtrIdx_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntPtrIdxReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_INT_PTR_IDX_IPIDX,      reg->intPtrIdx);

  baseAddr->INT_PTR_IDX = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_intPtrIdx_reg */

/*****************************************************************************
 * Combine and write the interrupt pointer value register
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intPtrVal_reg, ".text:Hyplnk:cfg:write_regs:intPtrVal_reg");
static hyplnkRet_e hyplnk_write_intPtrVal_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkIntPtrValReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_INT_PTR_VAL_INTPTR,      reg->intPtr);

  baseAddr->INT_PTR_VAL = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_intPtrVal_reg */

/*****************************************************************************
 * Combine and write the SERDES control and status register #1
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_serdesControl1_reg, ".text:Hyplnk:cfg:write_regs:serdesControl1");
static hyplnkRet_e hyplnk_write_serdesControl1_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl1Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS1_SLEEP_CNT,   reg->sleepCnt);
  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS1_DISABLE_CNT, reg->disableCnt);

  baseAddr->SERDES_CTL_STS1 = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_serdesControl1_reg */

/*****************************************************************************
 * Combine and write the SERDES control and status register #2
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_serdesControl2_reg, ".text:Hyplnk:cfg:write_regs:serdesControl2");
static hyplnkRet_e hyplnk_write_serdesControl2_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl2Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS2_S2CTL,  reg->s2Ctl);

  baseAddr->SERDES_CTL_STS2 = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_serdesControl2_reg */

/*****************************************************************************
 * Combine and write the SERDES control and status register #3
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_serdesControl3_reg, ".text:Hyplnk:cfg:write_regs:serdesControl3");
static hyplnkRet_e hyplnk_write_serdesControl3_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl3Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS3_S3CTL,  reg->s3Ctl);

  baseAddr->SERDES_CTL_STS3 = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_serdesControl3_reg */

/*****************************************************************************
 * Combine and write the SERDES control and status register #4
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_serdesControl4_reg, ".text:Hyplnk:cfg:write_regs:serdesControl4");
static hyplnkRet_e hyplnk_write_serdesControl4_reg
(
  CSL_VusrRegs   *baseAddr, 
  hyplnkSERDESControl4Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  hyplnk_range_check_begin;

  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS4_DVQUICK, reg->dvQuick);
  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS4_S4CTL,   reg->s4Ctl);
  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS4_TX_SPC,  reg->txSpc);
  hyplnk_setbits(new_val, CSL_VUSR_SERDES_CTL_STS4_RX_SPC,  reg->rxSpc);

  baseAddr->SERDES_CTL_STS4 = reg->raw = new_val;

  return hyplnk_range_check_return;
} /* hyplnk_write_serdesControl4_reg */

/*****************************************************************************
 * Write the entire RX PrivID table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXPrivIDTbl, ".text:Hyplnk:cfg:write_tbls:RXPrivIDTbl");
static hyplnkRet_e hyplnk_write_RXPrivIDTbl
(
  CSL_VusrRegs        *baseAddr, 
  hyplnkRXPrivIDTbl_t *tbl
)
{
  hyplnkRXPrivIDIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_RXPrivIDIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Check that hyplnk_RX_PRIVID_TBL_ENTS is correctly defined */
#if (hyplnk_RX_PRIVID_TBL_ENTS != \
     1 + ((CSL_VUSR_RX_PRIV_IDX_RXPRIVID_IDX_MASK >> \
           CSL_VUSR_RX_PRIV_IDX_RXPRIVID_IDX_SHIFT)))
#error hyplnk_RX_PRIVID_TBL_ENTS mismatches device
#endif
  /* Peform the requested write */
  for (i = 0; i < hyplnk_RX_PRIVID_TBL_ENTS; i++) {
    newIdx.rxPrivIDIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_RXPrivIDIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_write_RXPrivIDVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_RXPrivIDIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_write_RXPrivIDTbl */

/*****************************************************************************
 * Write the entire RX segment table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_RXSegTbl, ".text:Hyplnk:cfg:write_tbls:RXSegTbl");
static hyplnkRet_e hyplnk_write_RXSegTbl
(
  CSL_VusrRegs     *baseAddr, 
  hyplnkRXSegTbl_t *tbl
)
{
  hyplnkRXSegIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_RXSegIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Check that hyplnk_RX_SEG_TBL_ENTS is correctly defined */
#if (hyplnk_RX_SEG_TBL_ENTS != \
     1 + ((CSL_VUSR_RX_SEG_IDX_RXSEG_IDX_MASK >> \
           CSL_VUSR_RX_SEG_IDX_RXSEG_IDX_SHIFT)))
#error hyplnk_RX_SEG_TBL_ENTS mismatches device
#endif
  /* Peform the requested write */
  for (i = 0; i < hyplnk_RX_SEG_TBL_ENTS; i++) {
    newIdx.rxSegIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_RXSegIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_write_RXSegVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_RXSegIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_write_RXSegTbl */

/*****************************************************************************
 * Write the entire interrupt control table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intCtrlTbl, ".text:Hyplnk:cfg:write_tbls:intCtrlTbl");
static hyplnkRet_e hyplnk_write_intCtrlTbl
(
  CSL_VusrRegs       *baseAddr, 
  hyplnkIntCtrlTbl_t *tbl
)
{
  hyplnkIntCtrlIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_intCtrlIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Check that hyplnk_RX_SEG_TBL_ENTS is correctly defined */
#if (hyplnk_INT_CTRL_TBL_ENTS != \
     1 + ((CSL_VUSR_INT_CTL_IDX_ICIDX_MASK >> \
           CSL_VUSR_INT_CTL_IDX_ICIDX_SHIFT)))
#error hyplnk_INT_CTRL_TBL_ENTS mismatches device
#endif
  /* Peform the requested write */
  for (i = 0; i < hyplnk_INT_CTRL_TBL_ENTS; i++) {
    newIdx.intCtrlIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_intCtrlIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_write_intCtrlVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_intCtrlIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_write_intCtrlTbl */

/*****************************************************************************
 * Write the entire interrupt pointer table
 ****************************************************************************/
#pragma CODE_SECTION (hyplnk_write_intPtrTbl, ".text:Hyplnk:cfg:write_tbls:intPtrTbl");
static hyplnkRet_e hyplnk_write_intPtrTbl
(
  CSL_VusrRegs      *baseAddr, 
  hyplnkIntPtrTbl_t *tbl
)
{
  hyplnkIntPtrIdxReg_t origIdx, newIdx;
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  /* read current index */
  hyplnk_check_result(retVal, hyplnk_read_intPtrIdx_reg (baseAddr, &origIdx));
  newIdx = origIdx;

  /* Check that hyplnk_RX_SEG_TBL_ENTS is correctly defined */
#if (hyplnk_INT_PTR_TBL_ENTS != \
     1 + ((CSL_VUSR_INT_PTR_IDX_IPIDX_MASK >> \
           CSL_VUSR_INT_PTR_IDX_IPIDX_SHIFT)))
#error hyplnk_INT_PTR_TBL_ENTS mismatches device
#endif
  /* Peform the requested write */
  for (i = 0; i < hyplnk_INT_PTR_TBL_ENTS; i++) {
    newIdx.intPtrIdx = i;
    hyplnk_check_result(retVal, hyplnk_write_intPtrIdx_reg (baseAddr, &newIdx));
    hyplnk_check_result(retVal, hyplnk_write_intPtrVal_reg (baseAddr, *tbl + i));
  }

  /* Restore the original index */
  hyplnk_check_result(retVal, hyplnk_write_intPtrIdx_reg (baseAddr, &origIdx));

  return retVal;
} /* hyplnk_write_intPtrTbl */

#pragma CODE_SECTION (Hyplnk_writeRegs, ".text:Hyplnk:cfg:writeRegs");
/*********************************************************************
 * FUNCTION PURPOSE: Writes any/all registers/tables
 ********************************************************************/
hyplnkRet_e Hyplnk_writeRegs 
(
  Hyplnk_Handle      handle,   /**< [in] The HYPLNK LLD instance identifier */
  hyplnkLocation_e   location, /**< [in] Local or remote peripheral */
  hyplnkRegisters_t *writeRegs /**< [in] List of registers to write */
)
{
  CSL_VusrRegs *baseAddr = (CSL_VusrRegs *)handle;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  if (! hyplnkLObjIsValid) {
    return hyplnk_RET_NO_INIT;
  }

  hyplnk_check_handle(handle);

  if (location == hyplnk_LOCATION_REMOTE) {
    baseAddr = (CSL_VusrRegs *)&baseAddr->REM_REGS;
  }

  /* Single register operations */
  if (writeRegs->rev) {
    hyplnk_check_result(retVal, hyplnk_write_rev_reg (baseAddr, writeRegs->rev));
  }
  if (writeRegs->control) {
    hyplnk_check_result(retVal, hyplnk_write_control_reg (baseAddr, writeRegs->control));
  }
  if (writeRegs->status) {
    hyplnk_check_result(retVal, hyplnk_write_status_reg (baseAddr, writeRegs->status));
  }
  if (writeRegs->intPriVec) {
    hyplnk_check_result(retVal, hyplnk_write_intPriVec_reg (baseAddr, writeRegs->intPriVec));
  }
  if (writeRegs->intStatusClr) {
    hyplnk_check_result(retVal, hyplnk_write_intStatusClr_reg (baseAddr, writeRegs->intStatusClr));
  }
  if (writeRegs->intPendSet) {
    hyplnk_check_result(retVal, hyplnk_write_intPendSet_reg (baseAddr, writeRegs->intPendSet));
  }
  if (writeRegs->genSoftInt) {
    hyplnk_check_result(retVal, hyplnk_write_genSoftInt_reg (baseAddr, writeRegs->genSoftInt));
  }
  if (writeRegs->TXAddrOvly) {
    hyplnk_check_result(retVal, hyplnk_write_TXAddrOvly_reg (baseAddr, writeRegs->TXAddrOvly));
  }
  if (writeRegs->RXAddrSel) {
    hyplnk_check_result(retVal, hyplnk_write_RXAddrSel_reg (baseAddr, writeRegs->RXAddrSel));
  }
  if (writeRegs->RXPrivIDIdx) {
    hyplnk_check_result(retVal, hyplnk_write_RXPrivIDIdx_reg (baseAddr, writeRegs->RXPrivIDIdx));
  }
  if (writeRegs->RXPrivIDVal) {
    hyplnk_check_result(retVal, hyplnk_write_RXPrivIDVal_reg (baseAddr, writeRegs->RXPrivIDVal));
  }
  if (writeRegs->RXSegIdx) {
    hyplnk_check_result(retVal, hyplnk_write_RXSegIdx_reg (baseAddr, writeRegs->RXSegIdx));
  }
  if (writeRegs->RXSegVal) {
    hyplnk_check_result(retVal, hyplnk_write_RXSegVal_reg (baseAddr, writeRegs->RXSegVal));
  }
  if (writeRegs->chipVer) {
    hyplnk_check_result(retVal, hyplnk_write_chipVer_reg (baseAddr, writeRegs->chipVer));
  }
  if (writeRegs->lanePwrMgmt) {
    hyplnk_check_result(retVal, hyplnk_write_lanePwrMgmt_reg (baseAddr, writeRegs->lanePwrMgmt));
  }
  if (writeRegs->ECCErrors) {
    hyplnk_check_result(retVal, hyplnk_write_ECCErrors_reg (baseAddr, writeRegs->ECCErrors));
  }
  if (writeRegs->linkStatus) {
    hyplnk_check_result(retVal, hyplnk_write_linkStatus_reg (baseAddr, writeRegs->linkStatus));
  }
  if (writeRegs->intCtrlIdx) {
    hyplnk_check_result(retVal, hyplnk_write_intCtrlIdx_reg (baseAddr, writeRegs->intCtrlIdx));
  }
  if (writeRegs->intCtrlVal) {
    hyplnk_check_result(retVal, hyplnk_write_intCtrlVal_reg (baseAddr, writeRegs->intCtrlVal));
  }
  if (writeRegs->intPtrIdx) {
    hyplnk_check_result(retVal, hyplnk_write_intPtrIdx_reg (baseAddr, writeRegs->intPtrIdx));
  }
  if (writeRegs->intPtrVal) {
    hyplnk_check_result(retVal, hyplnk_write_intPtrVal_reg (baseAddr, writeRegs->intPtrVal));
  }
  if (writeRegs->serdesControl1) {
    hyplnk_check_result(retVal, hyplnk_write_serdesControl1_reg (baseAddr, writeRegs->serdesControl1));
  }
  if (writeRegs->serdesControl2) {
    hyplnk_check_result(retVal, hyplnk_write_serdesControl2_reg (baseAddr, writeRegs->serdesControl2));
  }
  if (writeRegs->serdesControl3) {
    hyplnk_check_result(retVal, hyplnk_write_serdesControl3_reg (baseAddr, writeRegs->serdesControl3));
  }
  if (writeRegs->serdesControl4) {
    hyplnk_check_result(retVal, hyplnk_write_serdesControl4_reg (baseAddr, writeRegs->serdesControl4));
  }
  
  /* Whole table operations */
  if (writeRegs->RXPrivIDTbl) {
    hyplnk_check_result(retVal, hyplnk_write_RXPrivIDTbl (baseAddr, writeRegs->RXPrivIDTbl));
  }
  if (writeRegs->RXSegTbl) {
    hyplnk_check_result(retVal, hyplnk_write_RXSegTbl (baseAddr, writeRegs->RXSegTbl));
  }
  if (writeRegs->intCtrlTbl) {
    hyplnk_check_result(retVal, hyplnk_write_intCtrlTbl (baseAddr, writeRegs->intCtrlTbl));
  }
  if (writeRegs->intPtrTbl) {
    hyplnk_check_result(retVal, hyplnk_write_intPtrTbl (baseAddr, writeRegs->intPtrTbl));
  }

  return retVal;
} /* Hyplnk_writeRegs */

#pragma CODE_SECTION (Hyplnk_getWindow, ".text:Hyplnk:cfg:getWindow");
/*********************************************************************
 * FUNCTION PURPOSE: Returns the data memroy window base and size
 *********************************************************************/
hyplnkRet_e Hyplnk_getWindow 
(
  Hyplnk_Handle  handle,   /**< [in] The HYPLNK LLD instance identifier */
  void         **base,     /**< [out] The data base address */
  uint32_t      *size      /**< [out] Data window size */
)
{
  int i;
  hyplnkRet_e retVal = hyplnk_RET_OK;

  if (! hyplnkLObjIsValid) {
    return hyplnk_RET_NO_INIT;
  }

  hyplnk_check_handle(handle);

  if (base) {
    for (i = 0; i < hyplnk_MAX_PERIPHS; i++) {
      if (handle == hyplnkLObj.cfg.dev.bases[i].cfgBase) {
        *base = hyplnkLObj.cfg.dev.bases[i].dataBase;
	break;
      }
    }

    if (i == hyplnk_MAX_PERIPHS) {
      retVal = hyplnk_RET_INV_HANDLE;
    }
  }

  if (size) *size = 0x10000000; /* 256 MB */

  return retVal;
} /* Hyplnk_getWindow */

/*********************************************************************
 * FUNCTION PURPOSE: Returns version number
 ********************************************************************/
#pragma CODE_SECTION (Hyplnk_getVersion, ".text:Hyplnk:version:getVersion");
uint32_t Hyplnk_getVersion 
(
  void
)
{
  return hyplnk_LLD_VERSION_ID;
} /* Hyplnk_getVersion */

/*********************************************************************
 * FUNCTION PURPOSE: Returns version string
 ********************************************************************/
#pragma CODE_SECTION (Hyplnk_getVersionStr, ".text:Hyplnk:version:getVersionStr");
const char* Hyplnk_getVersionStr
(
  void
)
{
  return HYPLNKLLDVersionStr;
} /* Hyplnk_getVersionStr */

/* Nothing past this point */

