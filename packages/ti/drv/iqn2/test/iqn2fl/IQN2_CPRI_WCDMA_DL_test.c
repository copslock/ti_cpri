/*****************************************************************************\
*           TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION           
*                                                                            
*  Property of Texas Instruments 
*  For  Unrestricted  Internal  Use  Only 
*  Unauthorized reproduction and/or distribution is strictly prohibited.  
*  This product is protected under copyright law and trade secret law 
*  as an unpublished work.  
*  Created 2004, (C) Copyright 2003 Texas Instruments.  All rights reserved.
*------------------------------------------------------------------------------
*  Filename       : IQN2_CPRI_WCDMA_DL_test.c
*  Date Created   : Sep. 10, 2013
*  Last Modified  : Sep. 10, 2013
*  Description    : 16x16 WCDMA DIO AxCs AIL loopback test (CPRI 4x or 8x)
*
*  History        : v0.1
*  Project        : Lamarr
*  Owner          : Albert Bae
*                 : 
\*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_chip.h>
#include "IQN2_config.h"
#include "psc_util.h"


#define _IQN2FL_DUMP			0

// test control
#define NUM_AXCS_WCDMA          16
#define DIO_NUM_BLOCK           128 //used large block only for test purpose
#define NUM_CHIP_PER_EVT        4

//Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
#ifdef _TMS320C6X
#pragma DATA_SECTION(wcdma_dio_data,".intData_sect")//use SL2 memory
#pragma DATA_ALIGN (wcdma_dio_data, 16)
#endif
uint32_t  wcdma_dio_data[NUM_AXCS_WCDMA * NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];

#ifdef _TMS320C6X
#pragma DATA_SECTION(wcdma_dio_result,".intData_sect")//use SL2 memory
#pragma DATA_ALIGN (wcdma_dio_result, 16)
#endif
uint32_t  wcdma_dio_result[NUM_AXCS_WCDMA * NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];


Iqn2Fl_Status iqn2Status; // IQN2FL status

/* Intc variable declarartion */
CSL_IntcObj    intcObj;
CSL_IntcHandle   hIntc;
CSL_IntcEventHandlerRecord  EventHandler[8];
CSL_IntcGlobalEnableState state;

/* IQN2 Global structures and variables  */
Iqn2Fl_Obj Iqn2Obj;// Iqn2 CSL object
Iqn2Fl_Handle hIqn2;// Iqn2 handle
uint32_t ctrlArg; // Ctrl Argument;

Iqn2Fl_Context Iqn2Context;//Iqn2 context
Iqn2Fl_InitCfg Iqn2InitCfg;

Iqn2Fl_Setup               iqn2Setup; // Iqn2 HW setup

Iqn2Fl_AilSetup            ail0Setup; // Setup for AIL0
Iqn2Fl_AilSetup            ail1Setup; // Setup for AIL0
Iqn2Fl_AilEgrSetup         ailEgrSetup; // Egress setup
Iqn2Fl_AilIgrSetup         ailIgrSetup; // Ingress setup
Iqn2Fl_AilPeSetup          ailPeSetup; // PE setup
Iqn2Fl_AilPdSetup          ailPdSetup; // PD setup
Iqn2Fl_AilUatSetup         ailUatSetup;//uAT setup
Iqn2Fl_AilPhySetup         ailPhySetup;//Phy setup

Iqn2Fl_TopSetup            topSetup; // Top registers setup
Iqn2Fl_Iqs2Setup           iqs2Setup; // IQS2 setup
Iqn2Fl_Dio2Setup           dio2Setup; // DIO2 setup
Iqn2Fl_At2Setup            at2Setup; // AT2 setup

extern void SB_config_4p9152 (Iqn2Fl_LinkRate LinkRate, uint32_t iqn2Base);
extern void SD_NES_Loopback();
extern void iqn2_config(void);
extern void iqn2_reset_all();
extern unsigned int enable_module(unsigned int pdctl, unsigned int mdctl);

volatile unsigned int int4_result = 0;

#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif

interrupt void int4_isr(){

   int4_result++;
}

void Intc_config(void)
{
   CSL_IntcParam    vectId;
   CSL_IntcContext  context;
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
   //! GEM0 Intc Configuration              !//
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
   /* Setup the global Interrupt */
   context.numEvtEntries = 8;
   context.eventhandlerRecord = EventHandler;
   CSL_intcInit(&context);
   /* Enable NMIs  */
   CSL_intcGlobalNmiEnable();
   /* Enable Global Interrupts  */
   CSL_intcGlobalEnable(&state);

   /* VectorID for the Global Edma Event  */
   vectId = CSL_INTC_VECTID_4;

   /* Opening a handle for the Fsync->EDMA Interrupt Event */
   hIntc   = CSL_intcOpen(&intcObj,
                           AIF2_EVENT0_INTSEL_MAP, // Event0
                           &vectId,
                           NULL);
   //Hook the ISRs
   CSL_intcHookIsr(vectId,  &int4_isr);
   // Clear the Interrupt
   CSL_intcHwControl(hIntc, CSL_INTC_CMD_EVTCLEAR,  NULL);
   //Enable the Event & the interrupt
   CSL_intcHwControl(hIntc, CSL_INTC_CMD_EVTENABLE,  NULL);
}

void iqn2_config()
{
    uint32_t i, src_addr, dst_addr;
    Iqn2Fl_UatCfg uat_cfg;

    src_addr = (uint32_t)wcdma_dio_data >> 4;
    dst_addr = (uint32_t)wcdma_dio_result >> 4;

	/************ Initialize IQN2 structures to avoid garbage insertion **************************/
	memset(&iqn2Setup, 0, sizeof(iqn2Setup));

	memset(&ail0Setup, 0, sizeof(ail0Setup));
	memset(&ail1Setup, 0, sizeof(ail1Setup));
	memset(&ailEgrSetup, 0, sizeof(ailEgrSetup));
	memset(&ailIgrSetup, 0, sizeof(ailIgrSetup));
	memset(&ailPeSetup, 0, sizeof(ailPeSetup));
	memset(&ailPdSetup, 0, sizeof(ailPdSetup));
	memset(&ailUatSetup, 0, sizeof(ailUatSetup));
    memset(&ailPhySetup, 0, sizeof(ailPhySetup));

	memset(&topSetup, 0, sizeof(topSetup));
	memset(&iqs2Setup, 0, sizeof(iqs2Setup));
	memset(&dio2Setup, 0, sizeof(dio2Setup));
	memset(&at2Setup, 0, sizeof(at2Setup));

	// Initialize CSL library, this step is required
	Iqn2Fl_init(&Iqn2Context);

	// Open Iqn2 and get handle
	Iqn2InitCfg.dev.bases[0].cfgBase = (void*)CSL_IQN_CFG_REGS;
	hIqn2 = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, &Iqn2InitCfg, &iqn2Status);

	if ((hIqn2 == NULL) || (iqn2Status != IQN2FL_SOK))
	{
	   printf ("\nError opening CSL_IQN");
	   exit(1);
	}

	/************** populating IQN2 major setup structures ***************************************************/
	iqn2Setup.ailSetup[IQN2FL_AIL_0] = &ail0Setup;
	iqn2Setup.ailSetup[IQN2FL_AIL_1] = &ail1Setup;//AIL1 lane is not used for this test
	iqn2Setup.topSetup = &topSetup;
	iqn2Setup.iqs2Setup = &iqs2Setup;
	iqn2Setup.dio2Setup = &dio2Setup;
	iqn2Setup.at2Setup = &at2Setup;
	iqn2Setup.aid2Setup = NULL; //AID is only used for DFE subsystem setup

	/************** IQN2 TOP area Setup  ***************************************************/
	topSetup.top_vc_sys_sts_cfg.sysclk_sel   = 0;//use SERDES lane0 clk is used for sys_clk
	topSetup.top_psr_cfg.bw_limit = 0; //use 1/16 VBUS BW for packet flushing

	/************** IQS Setup  ********************************************************************************/
	//SoC Egress
	for(i=0;i<NUM_AXCS_WCDMA;i++){
	  iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].chan = i;//DIO chan 0 ~ 15 is mapped to AIL0 chan 0 ~ 15
	  iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].dest = 0x2; //destination is AIL0 AxC
	}

	//SoC Ingress
	for(i=0;i<NUM_AXCS_WCDMA;i++){
	  iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i].chan = i;//AIL0 chan 0 ~ 15 is mapped to DIO chan 0 ~ 15
	  iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i].dest = 0x1;//destination is DIO
	}

	iqs2Setup.iqs2_ingress_cfg.pktdma_cfg_pb_sel = 1;//pb_at_3qtr_full as a default

	/**************************************************************************************/
    /************** DIO Setup  ***********************************************************/
	/**************************************************************************************/

	/********* DIO Core setup ***************************/
	dio2Setup.dio2_global_cfg_rsa_big_endian  = 0;//little endian order for UL RSA data
	dio2Setup.dio2_global_cfg_vbusm_priority = 0; //highest priority

	//DIO Egress core (engine 0)
	dio2Setup.dio2_core_egress.bcn_table_sel[0] = 0;//Table A
	dio2Setup.dio2_core_egress.dma_num_axc[0] = NUM_AXCS_WCDMA -1;//Set N-1
	dio2Setup.dio2_core_egress.dma_cfg0[0].dma_brst_ln = 2;//max 4 QWD per burst
	dio2Setup.dio2_core_egress.dma_cfg0[0].dma_num_qwd = 0;//1 QWD for DL
	dio2Setup.dio2_core_egress.dma_cfg0[0].rsa_cnvrt_en = 0;
	dio2Setup.dio2_core_egress.dma_cfg0[0].dma_eng_en = 1;
	dio2Setup.dio2_core_egress.dma_cfg0[0].dma_num_blks = DIO_NUM_BLOCK -1;//Set N-1
	dio2Setup.dio2_core_egress.dma_cfg1_dma_blk_addr_stride[0] = 16;//1qwd per AxC
	for(i=0;i<NUM_AXCS_WCDMA;i++)
	    dio2Setup.dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[i] = 0;//similar to aif2 dio offset
	for(i=0;i<NUM_AXCS_WCDMA;i++){
		dio2Setup.dio2_e_dbcntx_ram_mmr[0].dma_vbus_base_addr_axc[i] = src_addr + i;//DL has 1QWD offset for each AxC
		dio2Setup.dio2_e_dbcntx_ram_mmr[0].ch_id[i] = i;
		dio2Setup.dio2_e_dbcntx_ram_mmr[0].ch_en[i] = 1;
	}

	//DIO Ingress core (engine 0)
	dio2Setup.dio2_core_ingress.bcn_table_sel[0] = 0;//Table A
	dio2Setup.dio2_core_ingress.dma_num_axc[0] = NUM_AXCS_WCDMA -1;//Set N-1
	dio2Setup.dio2_core_ingress.dma_cfg0[0].dma_brst_ln = 2;//max 4 QWD per burst
	dio2Setup.dio2_core_ingress.dma_cfg0[0].dma_num_qwd = 0;//1 QWD for DL
	dio2Setup.dio2_core_ingress.dma_cfg0[0].rsa_cnvrt_en = 0;
	dio2Setup.dio2_core_ingress.dma_cfg0[0].dma_eng_en = 1;
	dio2Setup.dio2_core_ingress.dma_cfg0[0].dma_num_blks = DIO_NUM_BLOCK -1;//Set N-1
	dio2Setup.dio2_core_ingress.dma_cfg1_dma_blk_addr_stride[0] = 16;//1qwd per AxC
	for(i=0;i<NUM_AXCS_WCDMA;i++)
	    dio2Setup.dio2_i_axc_off_mmr_cfg_4samp_offset[i] = 0;//similar to aif2 dio offset
	for(i=0;i<NUM_AXCS_WCDMA;i++){
		dio2Setup.dio2_i_dbcntx_ram_mmr[0].dma_vbus_base_addr_axc[i] = dst_addr + i;//DL has 1QWD offset for each AxC
		dio2Setup.dio2_i_dbcntx_ram_mmr[0].ch_id[i] = i;
		dio2Setup.dio2_i_dbcntx_ram_mmr[0].ch_en[i] = 1;
	}

	/** DIO SI setup. Be careful! DIO SI has reversed order of Egress/Ingress when compared to SoC level order ***/

	//DIO SI ingress for SoC level egress operation (matched with AIL egress)
	for(i=0;i<NUM_AXCS_WCDMA;i++){
	dio2Setup.dio2_ife_chan_cfg_grp[i].chan_en = 1;
	dio2Setup.dio2_ife_chan_cfg_grp[i].chan_radio_sel = 0;//use radio standard 0 for WCDMA
	dio2Setup.dio2_ife_chan_cfg_grp[i].chan_axc_offset = 0;//fine AxC offset within QWD level. normally set to zero
	}
	dio2Setup.dio2_ife_radio_std_grp.ife_frm_tc_cfg[0].sym_tc = 0xe;//15 WCDMA slots. Set N-1
	dio2Setup.dio2_ife_radio_std_grp.ife_frm_tc_cfg[0].index_sc = 0;
	dio2Setup.dio2_ife_radio_std_grp.ife_frm_tc_cfg[0].index_tc = 0;
	dio2Setup.ife_frm_samp_tc_cfg_samp_tc[0] = 2559;//Set N-1

	dio2Setup.dio2_iq_idc_rate_ctl_cfg_rate = 0xF;//set idc rate control to max as a default

	//DIO SI egress for SoC level ingress operation (Matched with AIL ingress)
	for(i=0;i<NUM_AXCS_WCDMA;i++){
	dio2Setup.dio2_efe_cfg_grp.chan_en[i] = 1;
	dio2Setup.dio2_efe_cfg_grp.chan_radio_sel[i] = 0;//use radio standard 0 for WCDMA
	dio2Setup.efe_chan_axc_offset_cfg[i] = 0;//for CPRI,this should be matched with AIL PD AxC offset
	}
	dio2Setup.dio2_efe_radio_std_grp.efe_frm_tc_cfg[0].sym_tc = 0xe;//15 WCDMA slots. Set N-1
	dio2Setup.dio2_efe_radio_std_grp.efe_frm_tc_cfg[0].index_sc = 0;
	dio2Setup.dio2_efe_radio_std_grp.efe_frm_tc_cfg[0].index_tc = 0;
	dio2Setup.efe_frm_samp_tc_cfg[0] = 2559;//Set N-1

	for(i=0;i<NUM_AXCS_WCDMA;i++){
	dio2Setup.efe_chan_tdm_lut_cfg_chan_idx_cfg[i] = i;
	dio2Setup.efe_chan_tdm_lut_cfg_chan_idx_en_cfg[i] = 1;
	}
	dio2Setup.efe_rad_std_sch_cfg_tdm_start[0] = 0;
	dio2Setup.efe_rad_std_sch_cfg_tdm_len[0] = 16;
	dio2Setup.efe_rad_std_sch_cfg_tdm_en[0] = 1;

	/******* DIO uAT setup **********************************/
	//DIO Core Egress RADT0 with event16 (Used for Soc level Egress operation)
	dio2Setup.uat_dio_egr_radt_tc_cfg_val[0] = 2457599;//CPRI frame length with 245.76 MHz clock
	dio2Setup.uat_dio_egr_radt_offset_cfg_val[0] = 0;//not used for DIAG_SYNC test mode
	dio2Setup.uat_evt_radt_cmp_cfg_val[16] = 0;//No initial delay
	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[16] = 255;//4 chip event for DL

	//DIO SI Ingress RADT is not used (SoC level DIO egress doesn't require event)

	//DIO SI Egress RADT0 with event0 (Used for SoC level Ingress operation)
	dio2Setup.uat_egr_radt_tc_cfg_val[0] = 2457599;//CPRI frame length with 245.76 MHz clock
	dio2Setup.uat_egr_radt_offset_cfg_val[0] = 0;//not used for DIAG_SYNC test mode
	dio2Setup.uat_evt_radt_cmp_cfg_val[0] = 900;//400(pi)+ 500(ingress pipe delay) = 1000 clock delay for DIO SI
	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[0] = 255;//4 sample event

	//DIO Core Ingress RADT0 with event19 (Used for Soc level Ingress operation)
	dio2Setup.uat_dio_ing_radt_tc_cfg_val[0] = 2457599;//CPRI frame length with 245.76 MHz clock
	dio2Setup.uat_dio_ing_radt_offset_cfg_val[0] = 0;//not used for DIAG_SYNC test mode
	dio2Setup.uat_evt_radt_cmp_cfg_val[19] = 1400;//DIO SI delay + 500 = 1400 clock delay for final DMA
	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[19] = 255;//4 chip event for DL

    /**************************************************************************************/
	/************** AIL0 Setup  ***********************************************************/
	/**************************************************************************************/
	ail0Setup.ailInstNum   = IQN2FL_AIL_0;
	ail0Setup.pAilEgrSetup = &ailEgrSetup;
	ail0Setup.pAilIgrSetup = &ailIgrSetup;
	ail0Setup.pAilPeSetup  = &ailPeSetup;
	ail0Setup.pAilPdSetup  = &ailPdSetup;
	ail0Setup.pAilUatSetup = &ailUatSetup;
	ail0Setup.pAilPhySetup = &ailPhySetup;

	/*********** Phy configuration **************************/
    ailPhySetup.ail_phy_glb_cfg.obsai_cpri = 0; //CPRI
    ailPhySetup.ail_phy_glb_cfg.link_rate = IQN2FL_LINK_RATE_4x; //4x link rate

    ailPhySetup.ail_phy_rt_cfg.config = 2;//transmit mode
    ailPhySetup.ail_phy_rt_cfg.em_en = 0;//empty msg disable

    ailPhySetup.ail_phy_ci_co_lut_cfg.phy_ci_lut_cfg_sel = 0;//Select table A
    ailPhySetup.ail_phy_ci_co_lut_cfg.phy_co_lut_cfg_sel = 0;//Select table A

    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[0].smpl_count = 0xf;//sample count (n-1)
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[0].smpl_type = IQN2FL_DATA_WIDTH_15_BIT;//15 bit sample
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[0].smpl_last = 1;//sample last

    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[0].smpl_count = 0xf;//sample count (n-1)
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[0].smpl_type = IQN2FL_DATA_WIDTH_15_BIT;//15 bit sample
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[0].smpl_last = 1;//sample last

    ailPhySetup.ail_phy_tm_regs.phy_tm_cfg_en = 1;//TM enable
    ailPhySetup.ail_phy_tm_regs.phy_tm_cpri_scr_ctrl_cfg_seed_value = 0x0;//zero means scrambler disabled
    ailPhySetup.ail_phy_tm_regs.phy_tm_cpri_ptrp_cfg_ptr_p = 20;//Set CPRI pointer p
    ailPhySetup.ail_phy_tm_regs.phy_tm_cpri_version_cfg_prot_vers = 0x1;//Set CPRI version


    ailPhySetup.ail_phy_rm_regs.phy_rm_cfg_rx_en = 1;//RM enable
    ailPhySetup.ail_phy_rm_regs.phy_rm_dscr_ctrl_cfg_scr_en = 0;//SCR disable
    ailPhySetup.ail_phy_rm_regs.phy_rm_fsm_sync_cfg_sync_t = 2;
    ailPhySetup.ail_phy_rm_regs.phy_rm_fsm_sync_cfg_frame_sync_t = 2;
    ailPhySetup.ail_phy_rm_regs.phy_rm_fsm_unsync_cfg_unsync_t = 2;
    ailPhySetup.ail_phy_rm_regs.phy_rm_fsm_unsync_cfg_frame_unsync_t = 2;

	/************ PE, PD configuration ***************************************/

	//PE configuration (most PE related configuration is in SI Egress)
	for(i=0;i<NUM_AXCS_WCDMA;i++)
	    ailPeSetup.ail_pe_common.ail_pe_common_chan_cfg[i].rt_ctl = 1;//PE insert mode

	//PD configuration
	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	    ailPdSetup.ail_pd_common[i].rad_std = 0;
	    ailPdSetup.ail_pd_common[i].axc_offset = 0;//ingress AxC offset
	}

	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	    ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[i].cont_lut_grp = 0;
	    ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[i].cont_lut_en = 1;
	}

	ailPdSetup.ail_pd_cpri_axc_cfg.bub_fsm_cfg_knc[0] = 15;//set n-1

	ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[0].ncont = 15;//set n-1
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[0].start_lut = 0;
	//ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[0].en = 1;

	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd_cfg_en[0] = 1;
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_bfrm_offset[0] = 0;//CPRI basic frame offset
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_hfrm_offset[0] = 0;//CPRI hyper frame offset
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd2_cfg_bfrm_num[0] = 38399;//set n-1 basic frame number in RAD frame

	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	    ailPdSetup.ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_axc[i] = i;
	    ailPdSetup.ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_en[i] = 1;
	}

	/************ AIL0 SI configuration ***************************************/
    //SI Egress (include major PE configuration)
	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	ailEgrSetup.ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i].chan_radio_sel = IQN2FL_CHAN_RADIO_SEL_STD_0;
	ailEgrSetup.ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i].axc_fine_offset =0;//fine AxC offset. normally set zero
	ailEgrSetup.ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i].chan_en =1;
	}

	ailEgrSetup.ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[0].index_sc = 0;
	ailEgrSetup.ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[0].index_tc = 0;
	ailEgrSetup.ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[0].sym_tc = 14;//15 WCDMA slots
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[0] = 2559;//2560 samples in slot

	for(i=0;i<NUM_AXCS_WCDMA;i++){
	ailEgrSetup.ail_iq_efe_chan_axc_offset[i] = 0;//egress AxC offset
	}

	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	    ailEgrSetup.ail_iq_pe_axc_tdm_lut_cfg[i].axc = i;
	    ailEgrSetup.ail_iq_pe_axc_tdm_lut_cfg[i].en = 1;
	}

	ailEgrSetup.phy_en = 1;
	ailEgrSetup.ail_si_iq_e_sch_cpri.axc_cont_tc = 15;//(n-1)total number of containers in basic frame for AIL0

	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.knc[0] = 15;//set n-1
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.gap_int[0] = 0x3FFFF;//set max value if stuffing bit not required
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.ncont[0] = 15;//set n-1
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.lutstrt[0] = 0;//lut array start position

	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd_cfg_en[0]= 1;
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_bfrm_offset[0]= 0;//basic frame offset
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_hfrm_offset[0]= 0;//hyper frame offset
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd2_cfg_bfrm_num[0]= 38399;//38400 basic frames in radio frame

	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	   ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_cont_cfg.lut_grp[i] = IQN2FL_CHAN_RADIO_SEL_STD_0;
	   ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_cont_cfg.lut_en[i] = 1;
	}

	ailEgrSetup.ail_ectl_reg_grp.rate = 0xF;//ctl channel rate control to max as a default

    //SI Ingress
	for(i=0;i<NUM_AXCS_WCDMA;i++){//WCDMA
	    ailIgrSetup.ail_iq_ife_chan_config_group[i].chan_radio_sel = IQN2FL_CHAN_RADIO_SEL_STD_0;
	    ailIgrSetup.ail_iq_ife_chan_config_group[i].chan_en = 1;
	    ailIgrSetup.ail_iq_ife_chan_config_group[i].chan_tdd_frc_off = IQN2FL_CHAN_NO_FRC_OFF_SYM;
	}

	ailIgrSetup.ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[0].index_sc = 0;
	ailIgrSetup.ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[0].index_tc = 0;
	ailIgrSetup.ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[0].sym_tc = 14;//15 WCDMA slots
	ailIgrSetup.samp_tc[0] = 2559;//2560 samples in slot

	ailIgrSetup.rate_idc = 0xF;//rate control to max as a default
	ailIgrSetup.rate_ictl = 0xF;//ctl channel rate control to max as a default

	/************ AIL0 uAT configuration ***************************************/
    ailUatSetup.ail_uat_bcn_tc_cfg_val = 2457599;
    ailUatSetup.ail_uat_bcn_offset_cfg_val = 0;

    ailUatSetup.ail_uat_pe_fb_cfg_val = 300;//PE event
    ailUatSetup.ail_uat_rt_fb_cfg_val = 360;//RT event (between 310 ~ 360)
    ailUatSetup.ail_uat_tm_fb_cfg_val = 380;//Delta (80 clock from PE)
    ailUatSetup.ail_uat_pimin_cfg_val = 390;//PI MIN
    ailUatSetup.ail_uat_pimax_cfg_val = 430;//PI MAX

	/************** AT Setup  **************************/
    //BCN timer
    at2Setup.at2_bcn_offset_cfg_val = 2457344;//BCN starts 255 clock before frame boundary
    at2Setup.at2_bcn_clkcnt_tc_cfg_val = 2457599;
	at2Setup.at2_bcn_frm_init_lsbs_cfg_wr_val = 0;//BCN frame init value
	at2Setup.at2_bcn_frame_tc_lsb_cfg_val = 4095;//BCN frame terminal count

	//RAD timer 0 (LTE10MHz)
	at2Setup.at2_radt_0_cfg_clkcnt_tc[0] = 63;//64 clock count in one sample for CPRI
	at2Setup.at2_radt_0_cfg_lutindex_tc[0] = 0;
	at2Setup.at2_radt_0_cfg_symb_tc[0] = 14;//15 slots in frame
	at2Setup.at2_radt_1_cfg_frm_tc_lsb[0] = 4095;
	at2Setup.at2_radt_3_cfg_lut_index_strt[0] = 0;
	at2Setup.at2_radt_4_cfg_bcn_sync_cmp[0] = 0;//RADT0 has no additional initial delay when compared to BCN
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[0] = 2559;//2560 chips in slot

	//AT event 0 setup
	at2Setup.at2_events_24array.at2_events_24array_offset_cfg_val[0] = 0;
	at2Setup.at2_events_24array.at2_events_24array_offset_cfg_evt_strb_sel[0] = 8;//select RAD0 symbol strobe
	at2Setup.at2_events_24array.at2_events_24array_mod_tc_cfg_val[0] = 0x3fffff;//set max value which means no additional events between strobe
	at2Setup.at2_events_24array.at2_events_24array_mask_lsbs_cfg_val[0] = 0xffffffff;
	at2Setup.at2_events_24array.at2_events_24array_mask_msbs_cfg_val[0] = 0xffffffff;


#if _IQN2FL_DUMP == 1
	{
	   FILE *fout;
	   fout = fopen("iqn2fl_dump.txt","w");
	   if (fout)
	   {
		   dump_Iqn2Fl_Setup(fout,&iqn2Setup);
		   fclose(fout);
	   }

	}
#endif


	iqn2Status = Iqn2Fl_hwSetup(hIqn2, &iqn2Setup);

	if (iqn2Status != IQN2FL_SOK) printf ("\nIQN2 HW setup error 0x%x",iqn2Status);

	ctrlArg = 0xAAAAAAAA;
	hIqn2->arg_ail = IQN2FL_AIL_0; // Using AIL_0 instance

	//global enable of AIL0 (Egress, Ingress)
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);

	//global enable of DIO SI (Egress, Ingress)
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	//global enable of DIO engine Core (Egress, Ingress)
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET, (void *)&ctrlArg);

	ctrlArg = 0x1;//Enable AT event 0
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG, (void *)&ctrlArg);

	//Enable DIO uAT with DIAG_SYNC
	uat_cfg.diag_sync = 1;
	uat_cfg.uat_run = 1;
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG, (void *)&uat_cfg);

	//Enable AIL0 uAT with DIAG_SYNC
	uat_cfg.diag_sync = 1;
	uat_cfg.uat_run = 1;
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AIL_UAT_GEN_CTL_UAT_CFG_REG, (void *)&uat_cfg);

	ctrlArg = 0x1;//Enable RADT0 and Run BCN
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN, (void *)&ctrlArg);
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN, (void *)&ctrlArg);

}


void main(void)
{
    uint16_t  testpass;
    int idx, idx2;
    Iqn2Fl_TopVCSwResetStbSetup reset;

    printf("\nBeginning CPRI WCDMA DL 16AxCs loopback test\n");

    config_l1d_cache(0);//disable L1D cache to avoid MSMC memory cache invalidation

    enable_module(dfe_iqn_sys_pdctl, dfe_iqn_sys_mdctl);//AID, IQS, PSR, AT, DIO power on
    enable_module(iqn_ail_pdctl, iqn_ail_mdctl);// do not change this enable order
    enable_module(dfe_pd2_pdctl, dfe_pd2_mdctl);

    Intc_config();//GEM interrupt controller setup

    memset(wcdma_dio_result, 0xFF, sizeof(wcdma_dio_result));//clear dest buffer
    memset(wcdma_dio_data, 0x00, sizeof(wcdma_dio_data));//clear src buffer

  	for(idx =0; idx < DIO_NUM_BLOCK; idx++){
         for (idx2 = 0; idx2 < NUM_CHIP_PER_EVT*NUM_AXCS_WCDMA; idx2 ++) {
  		   wcdma_dio_data[(NUM_CHIP_PER_EVT*NUM_AXCS_WCDMA*idx) + idx2] = (idx << 24) + (idx2/4 << 16) + (NUM_AXCS_WCDMA*idx) + idx2/4;
         }
  	}

    //SERDES configuration for IQN2 sys_clk generation
//    SB_config_4p9152 (IQN2FL_LINK_RATE_4x);
    //SB_config_9p8304 ();//for CPRI 16x test

//    SD_NES_Loopback();

    //load IQN2 configuration
    iqn2_config();

    /****** wait for data transfer completion.***********************/
    while(1)
    {
       asm (" NOP 5 ");
       asm (" NOP 5 ");
       if(int4_result == 15 + 15 + 2)// Wait 17 slot time
       {
    	  reset.sw_reset =1;//IQN2 full reset
    	  Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB, (void *)&reset);
          break;
       }
    }

    testpass = 0;
    /* Compare the WCDMA DIO loopback data */
    testpass |= memcmp(&wcdma_dio_data[0], &wcdma_dio_result[0], NUM_AXCS_WCDMA*NUM_CHIP_PER_EVT*DIO_NUM_BLOCK);

    if (testpass == 0)
      printf(" \nDIO AxC Data Send/Recv: PASS\n");
    else
      printf(" \nDIO AxC Data Send/Recv: FAIL\n");

    printf("\nEnding CPRI WCDMA DL 16 AxCs  loopback test\n");
    
}

