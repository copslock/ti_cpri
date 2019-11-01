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

#include <string.h>
#include <stdio.h>

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#include <ti/drv/aif2/aif2fl_getHwStatusAux.h>
#include <ti/csl/cslr_cppidma_global_config.h>
#include <ti/csl/cslr_cppidma_rx_channel_config.h>
#include <ti/csl/cslr_cppidma_tx_channel_config.h>
#include <ti/csl/soc.h>

#include <ti/drv/aif2/AIF_defs.h>
#include <ti/drv/aif2/AIF_init_dat.h>
#include <ti/drv/aif2/AIF_fsync.h>
#include <ti/drv/aif2/AIF_calcParam.h>
#include <ti/drv/aif2/aif2_osal.h>

#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#define __AIF_DEBUG_C
#include <ti/drv/aif2/AIF_debug.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_resetException, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enableException, ".text:aifDriver");
#pragma CODE_SECTION(AIF_getException, ".text:aifDriver");
#pragma CODE_SECTION(AIF_printException, ".text:aifDriver");
#pragma CODE_SECTION(AIF_captureException, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enableDataTrace, ".text:aifDriver");
#pragma CODE_SECTION(AIF_disableDataTrace, ".text:aifDriver");
#pragma CODE_SECTION(AIF_printStatus, ".text:aifDriver");

#pragma CODE_SECTION(reset_ExceptionStats, ".text:aifDriver");

extern unsigned int _disable_interrupts(void);

extern void _restore_interrupts(unsigned int key);

#endif

/* Reset exception stats */
static void reset_ExceptionStats(
		AIF_ConfigHandle  hAif
)
{
	memset(&hAif->aif2EeCount, (uint8_t)0x00, sizeof(AIF_EeCountObj));
}

void AIF_resetException(
		AIF_ConfigHandle  hAif
)
{
	reset_ExceptionStats(hAif);
}

/* Enable  AIF2 Errors and Alarms */
void AIF_enableException(
		AIF_ConfigHandle  hAif
)
{
 Aif2Fl_EeDbInt    eeDbInt;
 Aif2Fl_EeLinkAInt eeLinkAInt;
 Aif2Fl_EeLinkBInt eeLinkBInt;
 Aif2Fl_EeAdInt    eeAdInt;
 Aif2Fl_EeCdInt    eeCdInt;
 Aif2Fl_EeSdInt    eeSdInt;
 Aif2Fl_EeVcInt    eeVcInt;
 Aif2Fl_EeAtInt    eeAtInt;
 Aif2Fl_EePdInt    eePdInt;
 Aif2Fl_EePeInt    eePeInt;
 uint32_t i;
#ifdef _TMS320C6X
 uint32_t gie;

   gie = _disable_interrupts();
#endif
   reset_ExceptionStats(hAif);

   /* DB errors */
   eeDbInt.db_ee_i_trc_ram_ovfl_err = true;
   eeDbInt.db_ee_i_token_ovfl_err = true;
   eeDbInt.db_ee_i_fifo_ovfl_err = true;
   eeDbInt.db_ee_i_pd2db_full_err = true;
   eeDbInt.db_ee_e_ps_axc_err = true;
   eeDbInt.db_ee_e_cd_data_err = false; // info
   eeDbInt.db_ee_e_cd_data_type_err = true;
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_DB_INT, (void *)&eeDbInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_DB_INT, (void *)&eeDbInt);
   /* AD errors */
   eeAdInt.ad_ee_i_cd_data_err = false; // info
   eeAdInt.ad_ee_e_cd_sch_err = false; //info
   eeAdInt.ad_ee_i_dma_0_err = true;
   eeAdInt.ad_ee_i_dma_1_err = true;
   eeAdInt.ad_ee_i_dma_2_err = true;
   eeAdInt.ad_ee_e_dma_0_err = true;
   eeAdInt.ad_ee_e_dma_1_err = true;
   eeAdInt.ad_ee_e_dma_2_err = true;
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_AD_INT, (void *)&eeAdInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_AD_INT, (void *)&eeAdInt);
   /* CD errors */
   eeCdInt.cd_ee_sop_desc_starve_err = true;
   eeCdInt.cd_ee_mop_desc_starve_err = true;
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_CD_INT, (void *)&eeCdInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV0;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_CD_INT, (void *)&eeCdInt);
   /* SD errors */
   eeSdInt.sd_ee_stspll_b4_err = false; // info
   eeSdInt.sd_ee_stspll_b8_err = false; // info
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_SD_INT, (void *)&eeSdInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_SD_INT, (void *)&eeSdInt);
   /* VC errors */
   eeVcInt.vc_ee_vbus_err = true;
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_VC_INT, (void *)&eeVcInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_VC_INT, (void *)&eeVcInt);
   /* AT errors */
   eeAtInt.at_ee_rp1_type_sys_rcvd_err = true;
   eeAtInt.at_ee_rp1_type_rp3_rcvd_err = true;
   eeAtInt.at_ee_rp1_type_tod_rcvd_err = true;
   eeAtInt.at_ee_rp1_type_unsel_err = true;
   eeAtInt.at_ee_rp1_type_spare_err = true;
   eeAtInt.at_ee_rp1_type_rsvd_err = true;
   eeAtInt.at_ee_rp1_bit_width_err = true;
   eeAtInt.at_ee_rp1_crc_err = true;
   eeAtInt.at_ee_rp1_rp3_err = true;
   eeAtInt.at_ee_rp1_sys_err = true;
   eeAtInt.at_ee_pi0_err = true;
   eeAtInt.at_ee_pi1_err = true;
   eeAtInt.at_ee_pi2_err = true;
   eeAtInt.at_ee_pi3_err = true;
   eeAtInt.at_ee_pi4_err = true;
   eeAtInt.at_ee_pi5_err = true;
   eeAtInt.at_ee_phyt_sync_err = true;
   eeAtInt.at_ee_radt_sync_err = true;
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_AT_INT, (void *)&eeAtInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_AT_INT, (void *)&eeAtInt);

   eePdInt.pd_ee_ts_wdog_err = false; // GSM only
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_PD_INT, (void *)&eePdInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_PD_INT, (void *)&eePdInt);

   eePeInt.pe_ee_rd2db_err = false;//false; // info
   eePeInt.pe_ee_token_req_ovfl_err = false;
   eePeInt.pe_ee_token_wr_err = false;
   eePeInt.pe_ee_dat_req_ovfl_err = false;
   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_PE_INT, (void *)&eePeInt);
   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_PE_INT, (void *)&eePeInt);

	for(i= 0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
		   hAif->hFl->arg_link = (Aif2Fl_LinkIndex)i;
		   /* Link A errors */
		   eeLinkAInt.rm_ee_sync_status_change_err = true;
		   eeLinkAInt.rm_ee_num_los_det_err = true;
		   eeLinkAInt.rm_ee_lcv_det_err = true;
		   eeLinkAInt.rm_ee_frame_bndry_det_err = false;
		   eeLinkAInt.rm_ee_block_bndry_det_err = false;
		   eeLinkAInt.rm_ee_missing_k28p5_err = true;
		   if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
			   eeLinkAInt.rm_ee_missing_k28p7_err = true;
			   eeLinkAInt.rm_ee_k30p7_det_err = true;
		   } else {
			   eeLinkAInt.rm_ee_missing_k28p7_err = false;
			   eeLinkAInt.rm_ee_k30p7_det_err = false;
		   }
		   eeLinkAInt.rm_ee_loc_det_err = true;
		   eeLinkAInt.rm_ee_rx_fifo_ovf_err = true;
		   if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol){
			   eeLinkAInt.rm_ee_rcvd_los_err = true;
			   eeLinkAInt.rm_ee_rcvd_lof_err = true;
			   eeLinkAInt.rm_ee_rcvd_rai_err = true;
			   eeLinkAInt.rm_ee_rcvd_sdi_err = true;
			   eeLinkAInt.rm_ee_los_err = true;
			   eeLinkAInt.rm_ee_lof_err = true;
			   eeLinkAInt.rm_ee_hfnsync_state_err = true;
			   eeLinkAInt.rm_ee_lof_state_err = true;
		   } else {
			   eeLinkAInt.rm_ee_rcvd_los_err = false;
			   eeLinkAInt.rm_ee_rcvd_lof_err = false;
			   eeLinkAInt.rm_ee_rcvd_rai_err = false;
			   eeLinkAInt.rm_ee_rcvd_sdi_err = false;
			   eeLinkAInt.rm_ee_los_err = false;
			   eeLinkAInt.rm_ee_lof_err = false;
			   eeLinkAInt.rm_ee_hfnsync_state_err = false;
			   eeLinkAInt.rm_ee_lof_state_err = false;
		   }
		   eeLinkAInt.tm_ee_frm_misalign_err = true;
		   eeLinkAInt.tm_ee_fifo_starve_err = true;
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
		   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_LINKA_INT, (void *)&eeLinkAInt);
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
		   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_LINKA_INT, (void *)&eeLinkAInt);

		   /* Link B errors */
		   eeLinkBInt.pd_ee_eop_err = true;
		   eeLinkBInt.pd_ee_crc_err = true;
		   eeLinkBInt.pd_ee_cpri_frame_err = true;
		   eeLinkBInt.pd_ee_axc_fail_err = true;
		   eeLinkBInt.pd_ee_sop_err = true;
		   eeLinkBInt.pd_ee_obsai_frm_err = true;
		   eeLinkBInt.pd_ee_wr2db_err = false; // info
		   eeLinkBInt.pe_ee_modrule_err = true;
		   eeLinkBInt.pe_ee_sym_err = true;
		   eeLinkBInt.pe_ee_mf_fifo_overflow_err = true;
		   eeLinkBInt.pe_ee_mf_fifo_underflow_err = true;
		   eeLinkBInt.pe_ee_db_starve_err = true;
		   eeLinkBInt.pe_ee_rt_if_err = true;
		   eeLinkBInt.pe_ee_pkt_starve_err = true;
		   eeLinkBInt.rt_ee_frm_err = true;
		   eeLinkBInt.rt_ee_ovfl_err = true;
		   eeLinkBInt.rt_ee_unfl_err = true;
		   eeLinkBInt.rt_ee_em_err = true;
		   eeLinkBInt.rt_ee_hdr_err = true;
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR;
		   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_LINKB_INT, (void *)&eeLinkBInt);
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_SET_EV1;
		   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_LINKB_INT, (void *)&eeLinkBInt);
		}
	}
	 Aif2Fl_eeEoiSetup(hAif->hFl, 0);
#ifdef _TMS320C6X
	 _restore_interrupts(gie);
#endif

}


/* Get AIF2 Errors and Alarms status and clear */
void AIF_getException(
		AIF_ConfigHandle  hAif
)
{
	Aif2Fl_EeDbInt    eeDbInt;
	Aif2Fl_EeLinkAInt eeLinkAInt;
	Aif2Fl_EeLinkBInt eeLinkBInt;
	Aif2Fl_EeAdInt    eeAdInt;
	Aif2Fl_EeCdInt    eeCdInt;
	Aif2Fl_EeSdInt    eeSdInt;
	Aif2Fl_EeVcInt    eeVcInt;
	Aif2Fl_EeAtInt    eeAtInt;
	Aif2Fl_EePdInt    eePdInt;
	Aif2Fl_EePeInt    eePeInt;
	Aif2Fl_EeOrigin   aif2Origin;
	uint32_t i;

    Aif2Fl_getEeAif2Origination(hAif->hFl,&aif2Origin);

	/* DB errors */
    if (aif2Origin.db_en_sts) {
    	hAif->aif2EeCount.eeFlag = 1;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_DB_INT_STATUS, (void *)&eeDbInt);
		hAif->aif2EeCount.eeDbIntCnt.db_ee_i_trc_ram_ovfl_err += eeDbInt.db_ee_i_trc_ram_ovfl_err;
		hAif->aif2EeCount.eeDbIntCnt.db_ee_i_token_ovfl_err += eeDbInt.db_ee_i_token_ovfl_err;
		hAif->aif2EeCount.eeDbIntCnt.db_ee_i_fifo_ovfl_err += eeDbInt.db_ee_i_fifo_ovfl_err;
		hAif->aif2EeCount.eeDbIntCnt.db_ee_i_pd2db_full_err += eeDbInt.db_ee_i_pd2db_full_err;
		hAif->aif2EeCount.eeDbIntCnt.db_ee_e_ps_axc_err += eeDbInt.db_ee_e_ps_axc_err;
		hAif->aif2EeCount.eeDbIntCnt.db_ee_e_cd_data_err += eeDbInt.db_ee_e_cd_data_err;
		hAif->aif2EeCount.eeDbIntCnt.db_ee_e_cd_data_type_err += eeDbInt.db_ee_e_cd_data_type_err;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_DB_INT, (void *)&eeDbInt);
    }
	/* AD errors */
    if (aif2Origin.ad_en_sts) {
    	hAif->aif2EeCount.eeFlag = 1;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_AD_INT_STATUS, (void *)&eeAdInt);
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_cd_data_err += eeAdInt.ad_ee_i_cd_data_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_cd_sch_err += eeAdInt.ad_ee_e_cd_sch_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_0_err += eeAdInt.ad_ee_i_dma_0_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_1_err += eeAdInt.ad_ee_i_dma_1_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_2_err += eeAdInt.ad_ee_i_dma_2_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_0_err += eeAdInt.ad_ee_e_dma_0_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_1_err += eeAdInt.ad_ee_e_dma_1_err;
		hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_2_err += eeAdInt.ad_ee_e_dma_2_err;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_AD_INT, (void *)&eeAdInt);
    }
	/* CD errors */
    if (aif2Origin.cd_en_sts) {
    	hAif->aif2EeCount.eeFlag = 1;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV0;
		Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_CD_INT_STATUS, (void *)&eeCdInt);
		hAif->aif2EeCount.eeCdIntCnt.cd_ee_sop_desc_starve_err += eeCdInt.cd_ee_sop_desc_starve_err;
		hAif->aif2EeCount.eeCdIntCnt.cd_ee_mop_desc_starve_err += eeCdInt.cd_ee_mop_desc_starve_err;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_CD_INT, (void *)&eeCdInt);
    }
	/* SD errors */
    if (aif2Origin.sd_en_sts) {
    	hAif->aif2EeCount.eeFlag = 1;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_SD_INT_STATUS, (void *)&eeSdInt);
		hAif->aif2EeCount.eeSdIntCnt.sd_ee_stspll_b4_err += eeSdInt.sd_ee_stspll_b4_err;
		hAif->aif2EeCount.eeSdIntCnt.sd_ee_stspll_b8_err += eeSdInt.sd_ee_stspll_b8_err;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_SD_INT, (void *)&eeSdInt);
    }
	/* VC errors */
    if (aif2Origin.vc_en_sts) {
    	hAif->aif2EeCount.eeFlag = 1;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_VC_INT_STATUS, (void *)&eeVcInt);
		hAif->aif2EeCount.eeVcIntCnt.vc_ee_vbus_err += eeVcInt.vc_ee_vbus_err;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_VC_INT, (void *)&eeVcInt);
    }
	/* AT errors */
    if (aif2Origin.at_en_sts) {
    	hAif->aif2EeCount.eeFlag = 1;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_AT_INT_STATUS, (void *)&eeAtInt);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_sys_rcvd_err += eeAtInt.at_ee_rp1_type_sys_rcvd_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_rp3_rcvd_err += eeAtInt.at_ee_rp1_type_rp3_rcvd_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_tod_rcvd_err += eeAtInt.at_ee_rp1_type_tod_rcvd_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_unsel_err += eeAtInt.at_ee_rp1_type_unsel_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_spare_err += eeAtInt.at_ee_rp1_type_spare_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_rsvd_err += eeAtInt.at_ee_rp1_type_rsvd_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_bit_width_err += eeAtInt.at_ee_rp1_bit_width_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_crc_err += eeAtInt.at_ee_rp1_crc_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_rp3_err += eeAtInt.at_ee_rp1_rp3_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_sys_err += eeAtInt.at_ee_rp1_sys_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_pi0_err += eeAtInt.at_ee_pi0_err;
		if (eeAtInt.at_ee_pi0_err)
			hAif->aif2EeCount.piCaptured[0] = CSL_FEXT(hAif->hFl->regs->PI_DATA[0].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_pi1_err += eeAtInt.at_ee_pi1_err;
		if (eeAtInt.at_ee_pi1_err)
			hAif->aif2EeCount.piCaptured[1] = CSL_FEXT(hAif->hFl->regs->PI_DATA[1].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_pi2_err += eeAtInt.at_ee_pi2_err;
		if (eeAtInt.at_ee_pi2_err)
			hAif->aif2EeCount.piCaptured[2] = CSL_FEXT(hAif->hFl->regs->PI_DATA[2].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_pi3_err += eeAtInt.at_ee_pi3_err;
		if (eeAtInt.at_ee_pi3_err)
			hAif->aif2EeCount.piCaptured[3] = CSL_FEXT(hAif->hFl->regs->PI_DATA[3].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_pi4_err += eeAtInt.at_ee_pi4_err;
		if (eeAtInt.at_ee_pi4_err)
			hAif->aif2EeCount.piCaptured[4] = CSL_FEXT(hAif->hFl->regs->PI_DATA[4].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_pi5_err += eeAtInt.at_ee_pi5_err;
		if (eeAtInt.at_ee_pi5_err)
			hAif->aif2EeCount.piCaptured[5] = CSL_FEXT(hAif->hFl->regs->PI_DATA[5].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
		hAif->aif2EeCount.eeAtIntCnt.at_ee_phyt_sync_err += eeAtInt.at_ee_phyt_sync_err;
		hAif->aif2EeCount.eeAtIntCnt.at_ee_radt_sync_err += eeAtInt.at_ee_radt_sync_err;
		hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_AT_INT, (void *)&eeAtInt);
    }
	/* PD errors */
	hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
	Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_PD_INT_STATUS, (void *)&eePdInt);
	hAif->aif2EeCount.eePdIntCnt.pd_ee_ts_wdog_err += eePdInt.pd_ee_ts_wdog_err;
	if (eePdInt.pd_ee_ts_wdog_err) hAif->aif2EeCount.eeFlag = 1;
	hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_PD_INT, (void *)&eePdInt);
	/* PE errors */
	hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
	Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_PE_INT_STATUS, (void *)&eePeInt);
	hAif->aif2EeCount.eePeIntCnt.pe_ee_rd2db_err += eePeInt.pe_ee_rd2db_err;
	if (eePeInt.pe_ee_rd2db_err) hAif->aif2EeCount.eeFlag = 1;
	hAif->aif2EeCount.eePeIntCnt.pe_ee_token_req_ovfl_err += eePeInt.pe_ee_token_req_ovfl_err;
	if (eePeInt.pe_ee_token_req_ovfl_err) hAif->aif2EeCount.eeFlag = 1;
	hAif->aif2EeCount.eePeIntCnt.pe_ee_token_wr_err += eePeInt.pe_ee_token_wr_err;
	if (eePeInt.pe_ee_token_wr_err) hAif->aif2EeCount.eeFlag = 1;
	hAif->aif2EeCount.eePeIntCnt.pe_ee_dat_req_ovfl_err += eePeInt.pe_ee_dat_req_ovfl_err;
	if (eePeInt.pe_ee_dat_req_ovfl_err) hAif->aif2EeCount.eeFlag = 1;
	hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_PE_INT, (void *)&eePeInt);

	if (aif2Origin.lk_en_sts_a0 && (1==hAif->linkConfig[0].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_b0 && (1==hAif->linkConfig[0].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_a1 && (1==hAif->linkConfig[1].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_b1 && (1==hAif->linkConfig[1].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_a2 && (1==hAif->linkConfig[2].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_b2 && (1==hAif->linkConfig[2].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_a3 && (1==hAif->linkConfig[3].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_b3 && (1==hAif->linkConfig[3].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_a4 && (1==hAif->linkConfig[4].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_b4 && (1==hAif->linkConfig[4].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_a5 && (1==hAif->linkConfig[5].linkEnable)) hAif->aif2EeCount.eeFlag = 1;
	if (aif2Origin.lk_en_sts_b5 && (1==hAif->linkConfig[5].linkEnable)) hAif->aif2EeCount.eeFlag = 1;

	for(i= 0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
		   hAif->hFl->arg_link = (Aif2Fl_LinkIndex)i;
		   /* Link A errors */
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		   Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_LINKA_INT_STATUS, (void *)&eeLinkAInt);
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_sync_status_change_err += eeLinkAInt.rm_ee_sync_status_change_err;
		   hAif->aif2EeCount.eeHfnsync[i].syncStatusChange = eeLinkAInt.rm_ee_sync_status_change_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_num_los_det_err += eeLinkAInt.rm_ee_num_los_det_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lcv_det_err += eeLinkAInt.rm_ee_lcv_det_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_frame_bndry_det_err += eeLinkAInt.rm_ee_frame_bndry_det_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_block_bndry_det_err += eeLinkAInt.rm_ee_block_bndry_det_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_missing_k28p5_err += eeLinkAInt.rm_ee_missing_k28p5_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_missing_k28p7_err += eeLinkAInt.rm_ee_missing_k28p7_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_k30p7_det_err += eeLinkAInt.rm_ee_k30p7_det_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_loc_det_err += eeLinkAInt.rm_ee_loc_det_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rx_fifo_ovf_err += eeLinkAInt.rm_ee_rx_fifo_ovf_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_los_err += eeLinkAInt.rm_ee_rcvd_los_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_lof_err += eeLinkAInt.rm_ee_rcvd_lof_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_rai_err += eeLinkAInt.rm_ee_rcvd_rai_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_sdi_err += eeLinkAInt.rm_ee_rcvd_sdi_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_los_err += eeLinkAInt.rm_ee_los_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lof_err += eeLinkAInt.rm_ee_lof_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_hfnsync_state_err += eeLinkAInt.rm_ee_hfnsync_state_err;
		   hAif->aif2EeCount.eeHfnsync[i].hfnsyncState = eeLinkAInt.rm_ee_hfnsync_state_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lof_state_err += eeLinkAInt.rm_ee_lof_state_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].tm_ee_frm_misalign_err += eeLinkAInt.tm_ee_frm_misalign_err;
		   hAif->aif2EeCount.eeLinkAIntCnt[i].tm_ee_fifo_starve_err += eeLinkAInt.tm_ee_fifo_starve_err;
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_LINKA_INT, (void *)&eeLinkAInt);

		   /* Link B errors */
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_EN_STATUS_EV1;
		   Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_EE_LINKB_INT_STATUS, (void *)&eeLinkBInt);
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_eop_err += eeLinkBInt.pd_ee_eop_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_crc_err += eeLinkBInt.pd_ee_crc_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_cpri_frame_err += eeLinkBInt.pd_ee_cpri_frame_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_axc_fail_err += eeLinkBInt.pd_ee_axc_fail_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_sop_err += eeLinkBInt.pd_ee_sop_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_obsai_frm_err += eeLinkBInt.pd_ee_obsai_frm_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_wr2db_err += eeLinkBInt.pd_ee_wr2db_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_modrule_err += eeLinkBInt.pe_ee_modrule_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_sym_err += eeLinkBInt.pe_ee_sym_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_mf_fifo_overflow_err += eeLinkBInt.pe_ee_mf_fifo_overflow_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_mf_fifo_underflow_err += eeLinkBInt.pe_ee_mf_fifo_underflow_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_db_starve_err += eeLinkBInt.pe_ee_db_starve_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_rt_if_err += eeLinkBInt.pe_ee_rt_if_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_pkt_starve_err += eeLinkBInt.pe_ee_pkt_starve_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_frm_err += eeLinkBInt.rt_ee_frm_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_ovfl_err += eeLinkBInt.rt_ee_ovfl_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_unfl_err += eeLinkBInt.rt_ee_unfl_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_em_err += eeLinkBInt.rt_ee_em_err;
		   hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_hdr_err += eeLinkBInt.rt_ee_hdr_err;
		   hAif->hFl->ee_arg = AIF2FL_EE_INT_CLR ;
		   Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_EE_LINKB_INT, (void *)&eeLinkBInt);
		}
	}

}

/* Print AIF2 Errors and Alarms status and clear */
void AIF_printException(
		AIF_ConfigHandle  hAif
)
{
    uint32_t i;
#ifdef _TMS320C6X
    uint32_t gie;

   gie = _disable_interrupts();
#endif


	if (hAif->aif2EeCount.eeFlag == 1) printf("\n######### AIF2 ERRORS ###########\n");
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_i_trc_ram_ovfl_err)
		printf("DB:%d Data Trace RAM overflowed. This is not a fatal error because it only affects the Data Trace RAM\n",hAif->aif2EeCount.eeDbIntCnt.db_ee_i_trc_ram_ovfl_err);
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_i_token_ovfl_err)
		printf("DB:%d Ingress Packet Data Token FIFO overflowed. This is a catastrophic condition and software needs to effectively flush all of the data buffers in the Ingress DB RAM and re-start the PD\n", hAif->aif2EeCount.eeDbIntCnt.db_ee_i_token_ovfl_err);
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_i_pd2db_full_err)
		printf("DB:%d PD-to-DB Bridge full. This is a catastrophic condition and indicates that the VBUS clock rate is too slow. Although this bridge is 64-deep, the error event will first occur when the bridge depth is 56 as an early warning\n", hAif->aif2EeCount.eeDbIntCnt.db_ee_i_pd2db_full_err);
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_e_ps_axc_err)
		printf("DB:%d AxC in Multicore Navigator Protocol Specific data does not match Multicore Navigator thread id. This is an indication of an application error. This event occurs if the application put data for an AxC in the wrong buffer in external memory\n", hAif->aif2EeCount.eeDbIntCnt.db_ee_e_ps_axc_err);
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_e_cd_data_type_err)
		printf("DB:%d EDB received data from PKTDMA with undefined Multicore Navigator data type. This is an indication of an application error. This event occurs if the Multicore Navigator Data Type received from the PKTDMA is not one of the ones supported.\n",hAif->aif2EeCount.eeDbIntCnt.db_ee_e_cd_data_type_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_0_err)
		printf("AD:%d For each of three Ingress DMA channels, if a trigger request is received while a request for that channel is still pending, a DMA error event will occur\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_0_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_1_err)
		printf("AD:%d For each of three Ingress DMA channels, if a trigger request is received while a request for that channel is still pending, a DMA error event will occur\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_1_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_2_err)
		printf("AD:%d For each of three Ingress DMA channels, if a trigger request is received while a request for that channel is still pending, a DMA error event will occur\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_dma_2_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_0_err)
		printf("AD:%d For each of three Egress DMA channels, if a trigger request is received while a request for that channel is still pending, a DMA error event will occur\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_0_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_1_err)
		printf("AD:%d For each of three Egress DMA channels, if a trigger request is received while a request for that channel is still pending, a DMA error event will occur\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_1_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_2_err)
		printf("AD:%d For each of three Egress DMA channels, if a trigger request is received while a request for that channel is still pending, a DMA error event will occur\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_dma_2_err);
	if (hAif->aif2EeCount.eeCdIntCnt.cd_ee_sop_desc_starve_err)
		printf("CD:%d PKTDMA SOP Descriptor Starvation\n",hAif->aif2EeCount.eeCdIntCnt.cd_ee_sop_desc_starve_err);
	if (hAif->aif2EeCount.eeCdIntCnt.cd_ee_mop_desc_starve_err)
		printf("CD:%d PKTDMA MOP Descriptor Starvation\n",hAif->aif2EeCount.eeCdIntCnt.cd_ee_mop_desc_starve_err);
	if (hAif->aif2EeCount.eeVcIntCnt.vc_ee_vbus_err)
		printf("VC:%d shows VBUS data error\n",hAif->aif2EeCount.eeVcIntCnt.vc_ee_vbus_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_unsel_err)
		printf("AT:%d AT RP1 Type Unsel Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_unsel_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_spare_err)
		printf("AT:%d AT RP1 Type Spare Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_spare_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_rsvd_err)
		printf("AT:%d AT RP1 Type Reserved Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_rsvd_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_bit_width_err)
		printf("AT:%d AT RP1 Type Bit Width Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_bit_width_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_crc_err)
		printf("AT:%d AT RP1 Type CRC Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_crc_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_rp3_err)
		printf("AT:%d AT PHYT selected width Frame number does not match the received RP1 RP3 frame number Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_rp3_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_sys_err)
		printf("AT:%d AT RADT selected width Frame number does not match the received RP1 SYS frame number Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_sys_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_pi0_err)
		printf("AT:%d AT Link 0 Captured Pi is not within pi window (%d)\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_pi0_err,hAif->aif2EeCount.piCaptured[0]);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_pi1_err)
		printf("AT:%d AT Link 1 Captured Pi is not within pi window (%d)\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_pi1_err,hAif->aif2EeCount.piCaptured[1]);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_pi2_err)
		printf("AT:%d AT Link 2 Captured Pi is not within pi window (%d)\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_pi2_err,hAif->aif2EeCount.piCaptured[2]);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_pi3_err)
		printf("AT:%d AT Link 3 Captured Pi is not within pi window (%d)\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_pi3_err,hAif->aif2EeCount.piCaptured[3]);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_pi4_err)
		printf("AT:%d AT Link 4 Captured Pi is not within pi window (%d)\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_pi4_err,hAif->aif2EeCount.piCaptured[4]);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_pi5_err)
		printf("AT:%d AT Link 5 Captured Pi is not within pi window (%d)\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_pi5_err,hAif->aif2EeCount.piCaptured[5]);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_phyt_sync_err)
		printf("AT:%d AT PHYT sync input is not aligned to the PHYT counter frame boundary Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_phyt_sync_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_radt_sync_err)
		printf("AT:%d AT RADT sync input is not aligned to the RADT counter frame boundary Error.\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_radt_sync_err);
	if (hAif->aif2EeCount.eePdIntCnt.pd_ee_ts_wdog_err)
		printf("PD:%d Global, Watch dog timer, time out before reception of GSM Time Slot EOP\n",hAif->aif2EeCount.eePdIntCnt.pd_ee_ts_wdog_err);
	if (hAif->aif2EeCount.eePeIntCnt.pe_ee_token_req_ovfl_err)
		printf("PE:%d Token request FIFO overflow\n",hAif->aif2EeCount.eePeIntCnt.pe_ee_token_req_ovfl_err);
	if (hAif->aif2EeCount.eePeIntCnt.pe_ee_dat_req_ovfl_err)
		printf("PE:%d Data request FIFO overflow\n",hAif->aif2EeCount.eePeIntCnt.pe_ee_dat_req_ovfl_err);

	for(i= 0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_num_los_det_err)
			   printf("RM link%d:%d los detection error\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_num_los_det_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lcv_det_err)
			   printf("RM link%d:%d lcv detection error\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lcv_det_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_missing_k28p5_err)
			   printf("RM link%d:%d Indicates that a k28.5 character was not received when it was supposed to have been received.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_missing_k28p5_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_missing_k28p7_err)
			   printf("RM link%d:%d OBSAI Only, Indicates that a k28.7 character was not received when it was supposed to have been received.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_missing_k28p7_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_k30p7_det_err)
			   printf("RM link%d:%d OBSAI Only, Indicates that a k30.7 error character was received.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_k30p7_det_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_loc_det_err)
			   printf("RM link%d:%d Indicates that the loss of clock watch dog timer has timed out that is, SerDes clock was not detected within the specified window.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_loc_det_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rx_fifo_ovf_err)
			   printf("RM link%d:%d Indicates that the RX FIFO has overflowed.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rx_fifo_ovf_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_los_err)
			   printf("RM link%d:%d CPRI only, Indicates a received, L1 Inband Loss Of Signal event(Z.130.0, bit3 as defined by CPRI)\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_los_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_lof_err)
			   printf("RM link%d:%d CPRI only, Indicates a received, L1 Inband Loss Of Frame\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_lof_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_rai_err)
			   printf("RM link%d:%d CPRI only, Indicates RAI error.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_rai_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_sdi_err)
			   printf("RM link%d:%d CPRI only, Indicates SDI error.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_rcvd_sdi_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_los_err)
			   printf("RM link%d:%d CPRI only, Indicates that the RX FSM is in the Loss Of Signal state that is, ST0(as defined by CPRI)\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_los_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lof_err)
			   printf("RM link%d:%d CPRI only, Indicates that the RX FSM is in the Loss Of Frame state that is, ST0 or St1 (as defined by CPRI)\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lof_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_hfnsync_state_err)
			   printf("RM link%d:%d CPRI only, Indicates RX FSM in the hyperframe state that is, state ST3. (as defined by CPRI)\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_hfnsync_state_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lof_state_err)
			   printf("RM link%d:%d CPRI only, Indicates that the RX FSM is in the Loss Of Frame state that is, ST0(as defined by CPRI)\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_lof_state_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].tm_ee_frm_misalign_err)
			   printf("TM link%d:%d The TM state machine detected a mis-aligned data frame on link. This bit is active on any clock cycle that the frame indication for the TM from the AT block does not line up with the frame indication in the data path. A misalignment will also occur if a frame indication from the AT block happens but the TM Fifo is empty\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].tm_ee_frm_misalign_err);
		   if (hAif->aif2EeCount.eeLinkAIntCnt[i].tm_ee_fifo_starve_err)
			   printf("TM link%d:%d The TM FIFO for link does not have data to transmit. This bit is active when the TM FIFO does not have data in it\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].tm_ee_fifo_starve_err);

		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_eop_err)
			   printf("PD link%d:%d Received a second EOP without an SOP in between\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_eop_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_crc_err)
			   printf("PD link%d:%d CRC failure for any packet(with CRC checking) within the given link.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_crc_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_cpri_frame_err)
			   printf("PD link%d:%d Arriving traffic (and user configuration of DBM) is non-multiple of 4 \n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_cpri_frame_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_axc_fail_err)
			   printf("PD link%d:%d Unrecoverable OBSAI Timestamp error for at least 1 AxC of a given link\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_axc_fail_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_sop_err)
			   printf("PD link%d:%d Received a second SOP without an EOP in between\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_sop_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_obsai_frm_err)
			   printf("PD link%d:%d Wrap of the PD_Frame Counters did not predict a Radio Frame Boundary consistent with TS=0 falling within the reception timing window. (PD_Frame)\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_obsai_frm_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_modrule_err)
			   printf("PE link%d:%d More than one modulo rule fired in a single clock cycle\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_modrule_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_sym_err)
			   printf("PE link%d:%d Symbol index in Multicore Navigator protocol specific header did not match for one or more symbol (Multicore Navigator packet).\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_sym_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_mf_fifo_overflow_err)
			   printf("PE link%d:%d MF FIFO was full when a write occurred.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_mf_fifo_overflow_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_mf_fifo_underflow_err)
			   printf("PE link%d:%d MF FIFO was empty when a read occurred.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_mf_fifo_underflow_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_db_starve_err)
			   printf("PE link%d:%d DB did not have antenna data for a AxC channel. Likely to occur if DMA was late.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_db_starve_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_rt_if_err)
			   printf("PE link%d:%d RT Interface Error\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_rt_if_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_pkt_starve_err)
			   printf("PE link%d:%d DB did not have packet data for a pkt channel. This error is only for CPRI packets and occurs when an active packet runs out of data in the middle of a packet.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pe_ee_pkt_starve_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_frm_err)
			   printf("RT link%d:%d The Retransmit frame no longer matches the Transmit frame in Aggregate mode for Link. The RT checks the frame structure of the data from the RM. If the frame structure received is not consistent with the programmed frame type(OBSAI or CPRI, link_rate), a frame error condition will occur. When a frame error condition occurs, this bit is made active, the RT will flush the RT Fifo so that it is empty, and a state machine will require a new frame boundary from the RM to begin buffering data again\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_frm_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_ovfl_err)
			   printf("RT link%d:%d The RT FIFO has overflowed in Retransmit or Aggregate mode for Link. The RT fifo contains the data that is to be retransmitted from the RM block while the RT is in Retransmit or Aggregate mode. The RT fifo is emptied for the TM to transmit. If the TM does not empty the Fifo as fast as the RM fills it, the RT fifo will suffer an overflow condition. When an overflow condition occurs, this bit is made active, and a state machine will require a new frame boundary from the RM to begin buffering data again.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_ovfl_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_unfl_err)
			   printf("RT link%d:%d The RT FIFO has underflowed in Retransmit or Aggregate mode for Link. The RT fifo contains the data that is to be retransmitted from the RM block while the RT is in Retransmit or Aggregate mode. The RT fifo is emptied for the TM to transmit. If the TM expects data and there is not any data for the TM to transmit, this situation defines an underflow condition. When an underflow condition occurs, this bit is made active, and a state machine will require a new frame boundary from the RM to begin buffering data again.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_unfl_err);
		   if (hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_hdr_err)
			   printf("RT link%d:%d The Retransmit header does not match the transmit header in OBSAI Aggregate mode for Link. While in OBSAI aggregate mode, the RT block expects that the header will match between the transmitted message from the PE and the Retransmitted message received by the RM. If the header does not match, the PE header is assumed to be the one used to transmit out of the TM and this indication is made active.\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_hdr_err);
		}
	}

	if (hAif->aif2EeCount.eeFlag == 1) printf("\n######### AIF2 INFO ##############\n");
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_i_fifo_ovfl_err)
		printf("DB:%d Ingress FIFO buffer channel overflowed. This can happen if the AIF2 VBUSM interface is stalled by the system. This is not a fatal error. The AIF2 can gracefully recover from this condition\n",hAif->aif2EeCount.eeDbIntCnt.db_ee_i_fifo_ovfl_err);
	if (hAif->aif2EeCount.eeDbIntCnt.db_ee_e_cd_data_err)
		printf("DB:%d Activity monitor that is active each time the Egress DB receives data from the PKTDMA\n",hAif->aif2EeCount.eeDbIntCnt.db_ee_e_cd_data_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_cd_data_err)
		printf("AD:%d Activity monitor that is active whenever the Ingress AD sends data to the PKTDMA\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_i_cd_data_err);
	if (hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_cd_sch_err)
		printf("AD:%d Activity monitor that is active high each time the Egress AD Scheduler sends a request to read data to the PKTDMA.\n",hAif->aif2EeCount.eeAdIntCnt.ad_ee_e_cd_sch_err);
	if (hAif->aif2EeCount.eeSdIntCnt.sd_ee_stspll_b4_err)
		printf("SD:%d If B4 PLL is locked correctly, this field will be set\n",hAif->aif2EeCount.eeSdIntCnt.sd_ee_stspll_b4_err);
	if (hAif->aif2EeCount.eeSdIntCnt.sd_ee_stspll_b8_err)
		printf("SD:%d If B8 PLL is locked correctly, this field will be set\n",hAif->aif2EeCount.eeSdIntCnt.sd_ee_stspll_b8_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_sys_rcvd_err)
		printf("AT:%d AT RP1 Type SYS data Receive\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_sys_rcvd_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_rp3_rcvd_err)
		printf("AT:%d AT RP1 Type RP3 data Receive\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_rp3_rcvd_err);
	if (hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_tod_rcvd_err)
		printf("AT:%d AT RP1 Type TOD data Receive\n",hAif->aif2EeCount.eeAtIntCnt.at_ee_rp1_type_tod_rcvd_err);
	if (hAif->aif2EeCount.eePeIntCnt.pe_ee_rd2db_err)
		printf("PE:%d If any DB channel is read and DB returns data (ACK). Indicating PE is both processing traffic and DB is supplying data (for debug)\n",hAif->aif2EeCount.eePeIntCnt.pe_ee_rd2db_err);
	if (hAif->aif2EeCount.eePeIntCnt.pe_ee_token_wr_err)
		printf("PE:%d One or more DMA transfer tokens has been issued by PE to DB (for debug) \n",hAif->aif2EeCount.eePeIntCnt.pe_ee_token_wr_err);
	for(i= 0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
			   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_sync_status_change_err)
				   printf("RM link%d:%d Indicates that the RX state machine changed state\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_sync_status_change_err);
			   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_frame_bndry_det_err)
				   printf("RM link%d:%d Indicates that a frame boundary was detected.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_frame_bndry_det_err);
			   if (hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_block_bndry_det_err)
				   printf("RM link%d:%d Indicates that a block boundary was detected. CPRI: K28.5 character & HFN!= 149 OBSAI: K28.5 character.\n",i,hAif->aif2EeCount.eeLinkAIntCnt[i].rm_ee_block_bndry_det_err);
			   if (hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_wr2db_err)
				   printf("PD link%d:%d If any value is written to the DB. Used as debug indicating PD is processing traffic for a given link. \n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].pd_ee_wr2db_err);
			   if (hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_em_err)
				   printf("RT link%d:%d The RT block created an Empty message in OBSAI Aggregate mode for Link. The RT block will replace a message with an empty message in OBSAI Retransmit mode if it detects that a message header is corrupted as indicated by error indication bits supplied by the RM. When this occurs, this bit is made active\n",i,hAif->aif2EeCount.eeLinkBIntCnt[i].rt_ee_em_err);
		}
	}

	if (hAif->aif2EeCount.eeFlag == 1) printf("\n###############################\n");

#ifdef _TMS320C6X
	_restore_interrupts(gie);
#endif
}

void AIF_printStatus(
		AIF_ConfigHandle  hAif
)
{
	AIF_getException(hAif);
	AIF_printException(hAif);
}

void AIF_captureException (
		AIF_ConfigHandle  hAif,
		AIF_EeCountObj *capturePtr
)
{
	memcpy(capturePtr, &hAif->aif2EeCount, sizeof(AIF_EeCountObj));
}

/* Official procedure from factory apps
 ***************  Data trace feature setup **********************************************
	//Set Data Trace receive data base address and framing data base address
	DTaddress = (uint32_t)&data_buffer1[0];
	Aif2Fl_hwControl(hAif2, AIF2FL_CMD_AD_TRACE_DATA_BASE_ADDR, (void *)&DTaddress);
	DTaddress = (uint32_t)&framing_buffer[0];
	Aif2Fl_hwControl(hAif2, AIF2FL_CMD_AD_TRACE_FRAMING_DATA_BASE_ADDR, (void *)&DTaddress);
	//Set Data Trace DMA burst wrap value
	DTwrap = 199;//200 * 64 bytes data and 200 * 16 bytes framing data
	Aif2Fl_hwControl(hAif2, AIF2FL_CMD_AD_TRACE_CPPI_DMA_BURST_WRAP, (void *)&DTwrap);

    //Enable Ingress DB Data trace (data, framing data) for link 0
	Aif2Fl_hwControl(hAif2, AIF2FL_CMD_ENABLE_DISABLE_LINK_DATA_CAPTURE, (void *)&ctrlArg);
    //Enable Data trace synchronization with frame boundary
	Aif2Fl_hwControl(hAif2, AIF2FL_CMD_ENABLE_DISABLE_DATA_TRACE_SYNC, (void *)&ctrlArg);
	//Enable Ingress AD Data trace (data, framing data)
	Aif2Fl_hwControl(hAif2, AIF2FL_CMD_AD_TRACE_DATA_DMA_CHANNEL_ON_OFF, (void *)&ctrlArg);
 *****************************************************************************************
 */

void AIF_enableDataTrace(
		AIF_ConfigHandle  	hAif,
		AIF_DataTraceHandle	hDataTrace
)
{
	uint32_t tempReg;

	// Enable ingress DIO if not enabled
	if (CSL_FEXT(hAif->hFl->regs->AD_DIO_I_GLOBAL_EN_SET, AIF2_AD_DIO_I_GLOBAL_EN_SET_DONT_CARE) == 0) {
		CSL_FINS(hAif->hFl->regs->AD_DIO_I_GLOBAL_EN_SET, AIF2_AD_DIO_I_GLOBAL_EN_SET_DONT_CARE,0x1);
	}

   //Set Data Trace receive data base address and framing data base address
   if (hDataTrace->dataCaptureEnable) {
	   Aif2Fl_adDataTraceBaseAddr(hAif->hFl, (uint32_t)hDataTrace->dataAddress);
   }
   if (hDataTrace->framingDataCaptureEnable) {
	   Aif2Fl_adFramingDataBaseAddr(hAif->hFl, (uint32_t)hDataTrace->framingDataAddress);
   }
   //Set Data Trace DMA burst wrap value
   Aif2Fl_adDtDmaWrap(hAif->hFl, hDataTrace->dtWrap);
	//Enable Ingress DB Data trace (data, framing data) for this link
   CSL_FINS(hAif->hFl->regs->RM_CFG, AIF2_RM_CFG_RAW_DATA_SEL, hDataTrace->linkIndex);
   //Enable Disable Data trace for DTB and DTF
   tempReg = 	CSL_FMK(AIF2_DB_IDB_CFG_DTB_EN,  hDataTrace->dataCaptureEnable)
		      | CSL_FMK(AIF2_DB_IDB_CFG_DTF_EN,  hDataTrace->framingDataCaptureEnable)
		      | CSL_FMK(AIF2_DB_IDB_CFG_DT_EN,   true)
		      | CSL_FMK(AIF2_DB_IDB_CFG_DT_SYNC, hDataTrace->syncCaptureEnable);
   hAif->hFl->regs->DB_IDB_CFG = tempReg;
   //Enable Ingress AD Data trace (data, framing data)
   tempReg =    CSL_FMK(AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_RD_CH_EN, hDataTrace->dataCaptureEnable)
		      | CSL_FMK(AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_FM_CH_EN, hDataTrace->framingDataCaptureEnable);
   hAif->hFl->regs->AD_DIO_DT_DMA_CFG0 = tempReg;
}

void AIF_disableDataTrace(
		AIF_ConfigHandle  	hAif
)
{
   uint32_t tempReg;

   //Enable Disable Data trace for DTB and DTF
   tempReg = 	CSL_FMK(AIF2_DB_IDB_CFG_DTB_EN,  false)
			  | CSL_FMK(AIF2_DB_IDB_CFG_DTF_EN,  false)
			  | CSL_FMK(AIF2_DB_IDB_CFG_DT_EN,   false)
			  | CSL_FMK(AIF2_DB_IDB_CFG_DT_SYNC, false);
   hAif->hFl->regs->DB_IDB_CFG = tempReg;
   //Enable Ingress AD Data trace (data, framing data)
   tempReg =    CSL_FMK(AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_RD_CH_EN, false)
			  | CSL_FMK(AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_FM_CH_EN, false);
   hAif->hFl->regs->AD_DIO_DT_DMA_CFG0 = tempReg;

}
////////////////////

