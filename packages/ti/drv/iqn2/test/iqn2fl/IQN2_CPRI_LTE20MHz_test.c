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
*  Filename       : IQN2_CPRI_LTE20MHz_test.c
*  Date Created   : Sep 8, 2013
*  Last Modified  : Sep 8, 2013
*  Description    : 8x LTE20MHz 4 AxCs loopback test.
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
#define NUM_AXCS_LTE            4
#define NUM_PKTS_PER_AXC        7
#define NUM_PKTS_ALL_AXCS       (NUM_AXCS_LTE * NUM_PKTS_PER_AXC)


/* Define queues for common FDQs */
#define MONO_TX_COMPLETE_Q      2000
#define MONO_RX_FDQ             2001

/* These are for the AIF test */
#define MONO_RX_Q               3000
#define MONO_TX_Q               832

//Users should use 16 bytes aligned data for IQN2 and PktDMA test
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region,".intData_sect")//use MSMC memory for test
#pragma DATA_ALIGN (mono_region, 16)
#endif

uint8_t   mono_region[64 * 8848];//Normal cyclic prefix 20 MHz LTE

uint32_t tmp[NUM_AXCS_LTE][NUM_PKTS_PER_AXC];

Iqn2Fl_Status iqn2Status; // CSL status

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
Iqn2Fl_At2Setup            at2Setup; // AT2 setup

extern void SB_config_4p9152 (Iqn2Fl_LinkRate LinkRate);
extern void SB_config_9p8304 ();//for CPRI 16x
extern void SD_NES_Loopback();
extern unsigned int enable_module(unsigned int pdctl, unsigned int mdctl);
void iqn2_config(void);
void Enable_pktDMA_ch(void);
void Disable_pktDMA_ch(void);

volatile unsigned int int4_result=0;


#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif


interrupt void int4_isr(){
   int chan, idx;
   if(int4_result == 136){//push packets before second frame
	   for(chan = 0; chan < NUM_AXCS_LTE; chan++)
	   {
	       for(idx =0; idx < NUM_PKTS_PER_AXC; idx++)
	       qm_push_queue((MONO_TX_Q + chan), 1, 0, tmp[chan][idx]);
	   }
   }

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

void MNavigator_config(void)
{
	uint32_t  flow_a;
    uint32_t  flow_d;
    uint16_t  idx;
    uint16_t  qm_mgr[2];
    uint16_t  qm_map[2];
    MNAV_MonolithicPacketDescriptor *mono_pkt;

   /* Descriptor memory regions must be defined in address order! */

   /* Setup Memory Region 0 for 64 * 8848B Monolithic descriptors. Our
    * Mono descriptors will be 12 bytes plus 4 bytes protocol specific field, plus
    * 8832(8768) bytes of payload(symbol). so the total size is 8848 and it is dividable by 16
    * 64 descriptors. (dead space is possible) */
    // (8848 / 16) - 1 = 0x228
    // log2(64) - 5 = 1
    qm_set_memory_region(0, 0,(uint32_t) mono_region, 0, 0x02280001);//use qm 1 and region 0

   /*****************************************************************
    * Configure Linking RAM 0 to use the 16KB internal memory. */
    qm_set_link_ram(0, 0, QMSS_LRAM_REGION, 0x3fff);//3fff(16KB) for Lamarr, 7fff(256KB) for Kepler

    /* Initialize descriptor regions to zero */
    memset(mono_region, 0, sizeof(mono_region));
    
    
    /* Push Monolithic packets into Tx Completion Queue */
    for (idx = 0; idx < NUM_PKTS_ALL_AXCS; idx ++)
    {
      mono_pkt = (MNAV_MonolithicPacketDescriptor *)(mono_region + (idx * 8848));
      mono_pkt->type_id = MNAV_DESC_TYPE_MONO;
      mono_pkt->data_offset = 16;
      mono_pkt->pkt_return_qmgr = MONO_TX_COMPLETE_Q >> 12;
      mono_pkt->pkt_return_qnum = MONO_TX_COMPLETE_Q & 0x0FFF; 
      
      qm_push_queue(MONO_TX_COMPLETE_Q, 1, 0, (uint32_t)(mono_pkt));
    }

    /* Push Monolithic packets to Rx FDQ  */
    for (idx = 32; idx < 32 + NUM_PKTS_ALL_AXCS; idx ++)
    {
      mono_pkt = (MNAV_MonolithicPacketDescriptor *)(mono_region + (idx * 8848));
      mono_pkt->type_id = MNAV_DESC_TYPE_MONO;
      mono_pkt->data_offset = 16;
      
      qm_push_queue(MONO_RX_FDQ, 1, 0, (uint32_t)(mono_pkt));
    }

    /* Create other PktDMA QM configurations. */
    qm_mgr[0] = 0; qm_map[0] = 0;     //QM1 - one qm for Lamarr
    qm_mgr[1] = 0; qm_map[1] = 4096;  //QM1
    pktdma_config_qm(DFE_IQN2_PKTDMA_GBL_CFG_REGION, qm_mgr, qm_map);

    /* PktDMA loopback is enabled by default. this should be disabled for IQN2 test */
    pktdma_config_loopback(DFE_IQN2_PKTDMA_GBL_CFG_REGION, 0);


    /***** Configure Rx channel flows  ************************************/

    //Create flow configuration for the Monolithic packets
    for(idx =0; idx < NUM_AXCS_LTE; idx++)
    {
        flow_a = 0x28100000 | (MONO_RX_Q + idx);
        flow_d = MONO_RX_FDQ << 16;
        pktdma_config_rx_flow(DFE_IQN2_PKTDMA_RX_FLOW_REGION, idx,
                   flow_a, 0, 0, flow_d, 0, 0, 0, 0);
    }

    //pktDMA channel enable will be done in different place
	
}

void Enable_pktDMA_ch(void)
{
	int idx;
	 /********************** Enable Tx and Rx channels. ******************/
	 for(idx =0; idx < NUM_AXCS_LTE; idx++){
	     pktdma_config_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, idx, 0x01000000); //set AIF_MONO_MODE to 1 and set PS filter to zero
	     pktdma_enable_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, idx, 0x80000000);
	     pktdma_enable_rx_chan(DFE_IQN2_PKTDMA_RX_CHAN_REGION, idx, 0x80000000);
	 }

}

void Disable_pktDMA_ch(void)
{
	int idx;
	 /********************** Enable Tx and Rx channels. ******************/
	 for(idx =0; idx < NUM_AXCS_LTE; idx++){
	     pktdma_enable_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, idx, 0x00000000);
	     pktdma_enable_rx_chan(DFE_IQN2_PKTDMA_RX_CHAN_REGION, idx, 0x00000000);
	 }
}

void iqn2_config()
{
    int i;
    Iqn2Fl_UatCfg uat_cfg;

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
	iqn2Setup.dio2Setup = NULL;
	iqn2Setup.at2Setup = &at2Setup;
	iqn2Setup.aid2Setup = NULL; //AID is only used for DFE subsystem setup

	/************** IQN2 TOP area Setup  ***************************************************/
	topSetup.top_vc_sys_sts_cfg.sysclk_sel   = 0;//use SERDES lane0 clk is used for sys_clk
	topSetup.top_psr_cfg.bw_limit = 0; //use 1/16 VBUS BW for packet flushing
	for(i=0;i<NUM_AXCS_LTE;i++){
	  topSetup.top_psr_cfg.pack_ps_data[i] = 1;//pack ps data for pktDMA chan 0 ~ 3
	}

	/************** IQS Setup  ********************************************************************************/
	//SoC Egress
	for(i=0;i<NUM_AXCS_LTE;i++){
	  iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].chan = i;//pktDMA chan 0 ~ 3 is mapped to AIL0 chan 0 ~ 3
	  iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].dest = 0x2;//destination is AIL0 AxC
	}
	//SoC Ingress
	for(i=0;i<NUM_AXCS_LTE;i++){
	  iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i].chan = i;//AIL0 chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
	  iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i].dest = 0x0;//destination is PktDMA
	}

	iqs2Setup.iqs2_ingress_cfg.pktdma_cfg_pb_sel = 1;//pb_at_3qtr_full as a default

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
    ailPhySetup.ail_phy_glb_cfg.link_rate = IQN2FL_LINK_RATE_8x; //8x

    ailPhySetup.ail_phy_rt_cfg.config = 2;//transmit mode
    ailPhySetup.ail_phy_rt_cfg.em_en = 0;//empty msg disable

    ailPhySetup.ail_phy_ci_co_lut_cfg.phy_ci_lut_cfg_sel = 0;//Select table A
    ailPhySetup.ail_phy_ci_co_lut_cfg.phy_co_lut_cfg_sel = 0;//Select table A

    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[0].smpl_count = 31;//sample count (n-1)
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[0].smpl_type = IQN2FL_DATA_WIDTH_15_BIT;//15 bit sample
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[0].smpl_last = 1;//sample last

    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[0].smpl_count = 31;//sample count (n-1)
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[0].smpl_type = IQN2FL_DATA_WIDTH_15_BIT;//15 bit sample
    ailPhySetup.ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[0].smpl_last = 1;//sample last

    ailPhySetup.ail_phy_tm_regs.phy_tm_cfg_en = 1;//TM enable
    ailPhySetup.ail_phy_tm_regs.phy_tm_cpri_scr_ctrl_cfg_seed_value = 0x0;//should be Set to zero if scrambler is not used
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
	for(i=0;i<NUM_AXCS_LTE;i++)
		ailPeSetup.ail_pe_common.ail_pe_common_chan_cfg[i].rt_ctl = 1;//PE insert mode

	//PD configuration
	for(i=0;i<NUM_AXCS_LTE;i++){//LTE
		ailPdSetup.ail_pd_common[i].rad_std = IQN2FL_CHAN_RADIO_SEL_STD_0;
		ailPdSetup.ail_pd_common[i].axc_offset = 0;//ingress RAD AxC offset. normally set to zero
	}

	for(i=0;i<NUM_AXCS_LTE*8;i++){//LTE20MHz
		ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[i].cont_lut_grp = IQN2FL_CHAN_RADIO_SEL_STD_0;
		ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[i].cont_lut_en = 1;
	}
	ailPdSetup.ail_pd_cpri_axc_cfg.bub_fsm_cfg_knc[0] = 31;//set n-1
	ailPdSetup.ail_pd_cpri_axc_cfg.bub_fsm2_cfg_gap_int[0] = 0;//zero means no stuffing samples

	ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[0].ncont = 31;//set n-1
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[0].start_lut = 0;
	//ailPdSetup.ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[0].en = 1;

	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd_cfg_en[0] = 1;
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_bfrm_offset[0] = 0;//CPRI basic frame offset
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_hfrm_offset[0] = 0;//CPRI hyper frame offset
	ailPdSetup.ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd2_cfg_bfrm_num[0] = 38399;//set n-1 basic frame number in RAD frame

	for(i=0;i<NUM_AXCS_LTE*8;i++){//LTE 20MHz
		ailPdSetup.ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_axc[i] = (i/8);
		ailPdSetup.ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_en[i] = 1;
	}

	/************ AIL0 SI configuration ***************************************/
    //SI Egress (include major PE configuration)
	for(i=0;i<NUM_AXCS_LTE;i++){//LTE
	ailEgrSetup.ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i].chan_radio_sel = IQN2FL_CHAN_RADIO_SEL_STD_0;
	ailEgrSetup.ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i].axc_fine_offset =0;//fine AxC offset. normally set zero
	ailEgrSetup.ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i].chan_en =1;
	}

	ailEgrSetup.ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[0].index_sc = 1;
	ailEgrSetup.ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[0].index_tc = 7;
	ailEgrSetup.ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[0].sym_tc = 139;//140 LTE symbols
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[1] = 2207;//2208 samples for first 20MHz symbol
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[2] = 2191;//2192 samples for other six 20MHz symbols
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[3] = 2191;
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[4] = 2191;
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[5] = 2191;
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[6] = 2191;
	ailEgrSetup.ail_iq_efe_frm_samp_tc_mmr_ram[7] = 2191;

	for(i=0;i<NUM_AXCS_LTE;i++){
	ailEgrSetup.ail_iq_efe_chan_axc_offset[i] = 0;//egress AxC offset
	}

	for(i=0;i<NUM_AXCS_LTE*8;i++){
		ailEgrSetup.ail_iq_pe_axc_tdm_lut_cfg[i].axc = (i/8);
		ailEgrSetup.ail_iq_pe_axc_tdm_lut_cfg[i].en = 1;
	}

	ailEgrSetup.phy_en = 1;
	ailEgrSetup.ail_si_iq_e_sch_cpri.axc_cont_tc = 31;//total number of containers in basic frame for AIL0

	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.knc[0] = 31;//set n-1
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.gap_int[0] = 0x3FFFF;//set max value if stuffing bit not required

	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.ncont[0] = 31;//set n-1
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.lutstrt[0] = 0;//lut array start position

	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd_cfg_en[0]= 1;
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_bfrm_offset[0]= 0;//basic frame offset
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_hfrm_offset[0]= 0;//hyper frame offset
	ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd2_cfg_bfrm_num[0]= 38399;//38400 basic frames in radio frame

	for(i=0;i<NUM_AXCS_LTE*8;i++){//LTE
	   ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_cont_cfg.lut_grp[i] = IQN2FL_CHAN_RADIO_SEL_STD_0;
	   ailEgrSetup.ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_cont_cfg.lut_en[i] = 1;
	}

	ailEgrSetup.ail_ectl_reg_grp.rate = 0xF;//ctl channel rate control to max as a default

    //SI Ingress
	for(i=0;i<NUM_AXCS_LTE;i++){//LTE
		ailIgrSetup.ail_iq_ife_chan_config_group[i].chan_radio_sel = IQN2FL_CHAN_RADIO_SEL_STD_0;
		ailIgrSetup.ail_iq_ife_chan_config_group[i].chan_en = 1;
		ailIgrSetup.ail_iq_ife_chan_config_group[i].chan_tdd_frc_off = IQN2FL_CHAN_NO_FRC_OFF_SYM;
	}

	ailIgrSetup.ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[0].index_sc = 1;
	ailIgrSetup.ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[0].index_tc = 7;
	ailIgrSetup.ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[0].sym_tc = 139;//140 LTE symbols
	ailIgrSetup.samp_tc[1] = 2207;//2208 samples for first 20MHz symbol
	ailIgrSetup.samp_tc[2] = 2191;//2192 samples for other six 20MHz symbols
	ailIgrSetup.samp_tc[3] = 2191;
	ailIgrSetup.samp_tc[4] = 2191;
	ailIgrSetup.samp_tc[5] = 2191;
	ailIgrSetup.samp_tc[6] = 2191;
	ailIgrSetup.samp_tc[7] = 2191;

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

	//RAD timer 0
	at2Setup.at2_radt_0_cfg_clkcnt_tc[0] = 7;//8 clock count in one sample for LTE20MHz
	at2Setup.at2_radt_0_cfg_lutindex_tc[0] = 6;
	at2Setup.at2_radt_0_cfg_symb_tc[0] = 139;//140 symbols in frame
	at2Setup.at2_radt_1_cfg_frm_tc_lsb[0] = 4095;
	at2Setup.at2_radt_3_cfg_lut_index_strt[0] = 0;
	at2Setup.at2_radt_4_cfg_bcn_sync_cmp[0] = 0;//RADT0 has no additional initial delay when compared to BCN

	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[0] = 2207;
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[1] = 2191;
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[2] = 2191;
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[3] = 2191;
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[4] = 2191;
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[5] = 2191;
	at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[6] = 2191;

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

	ctrlArg = 0x1;//Enable AT event 0
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG, (void *)&ctrlArg);

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
	uint32_t  monoRxCount;
    uint16_t  testpass;
    uint32_t *temp;
    uint32_t  chan, idx, idx2, rx_count, value;
    MNAV_MonolithicPacketDescriptor *mono_pkt;
    Iqn2Fl_TopVCSwResetStbSetup reset;

    printf("\nBeginning LTE20MHz 4 AxCs loopback test\n");

    config_l1d_cache(0);//disable L1D cache to avoid MSMC region invalidation

    enable_module(dfe_iqn_sys_pdctl, dfe_iqn_sys_mdctl);//AID, IQS, PSR, AT, DIO power on
    enable_module(iqn_ail_pdctl, iqn_ail_mdctl);// do not change this enable order
    enable_module(dfe_pd2_pdctl, dfe_pd2_mdctl);

    Intc_config();//GEM interrupt controller setup

    MNavigator_config();//multicore navigator configuration for LTE10MHz

    // push AxC packets
    for(chan = 0; chan < NUM_AXCS_LTE; chan++)
    {    	
        for(idx =0; idx < NUM_PKTS_PER_AXC; idx++)
        {  
            //push 7 packets into Tx queue for NUM_AXCS_LTE channels
    	
            tmp[chan][idx] = qm_pop_queue(MONO_TX_COMPLETE_Q);
            tmp[chan][idx] &= 0xFFFFFFF0;//set DESC_SIZE field to zero
            
            mono_pkt = (MNAV_MonolithicPacketDescriptor *)tmp[chan][idx];
        
            //Create Mono packet (initialize non-zero fields)
            mono_pkt->type_id = MNAV_DESC_TYPE_MONO;
            mono_pkt->data_offset = MNAV_MONO_PACKET_SIZE + 4;//16
            if((idx%7) == 0)
                mono_pkt->packet_length = 8832;//first symbol
            else 
                mono_pkt->packet_length = 8768;//other six symbols
                
            mono_pkt->ps_flags = 1; 
            mono_pkt->epib = 0;
            mono_pkt->psv_word_count = 1; // 4 byte PS field length
            mono_pkt->pkt_return_qnum = MONO_TX_COMPLETE_Q;

    
            temp = (uint32_t *)(tmp[chan][idx] + 16);
            
            if((idx%7) == 0)
            {
                for (idx2 = 0; idx2 < 2208; idx2 ++) temp[idx2] = (chan << 24) + (idx << 16)+ idx2; //payload data setup(first symbol)
            }
            else 
            {
                for (idx2 = 0; idx2 < 2192; idx2 ++) temp[idx2] = (chan << 24) + (idx << 16)+ idx2; //payload data setup(other six symbols)
            }
        
            //Create PS data
            temp = (uint32_t *)(tmp[chan][idx] + MNAV_MONO_PACKET_SIZE);
            
            temp[0] = (uint32_t)(0x00008000 + chan + (idx << 7));//add symbol number into PS field
           
            tmp[chan][idx] |= 0x00000003;//set DESC_SIZE to 3 for AIF2 mono mode
            
            //qm_push_queue((MONO_TX_Q + chan), 1, 0, tmp[chan][idx]); // push will be done in ISR
              
        }
    
    }

    //SERDES configuration
    SB_config_4p9152 (IQN2FL_LINK_RATE_8x);
    //SB_config_9p8304();//CPRI 16x link rate

    SD_NES_Loopback();//enable SERDES digital loopback

    //load IQN2 configuration and timer start
    iqn2_config();

    Enable_pktDMA_ch();//enable pktDMA channel

    /****** wait for data transfer completion.***********************/
    while(1)
    {
        asm (" NOP 5 ");
        asm (" NOP 5 ");
        if(int4_result == (140 + 14))// Wait 11 ms
         {
        	Disable_pktDMA_ch();
        	reset.sw_reset =1;//IQN2 full reset
        	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB, (void *)&reset);
            break;
         }
    }


    monoRxCount = 0;  // Rx descriptor count for monolithic RX queue
    while (monoRxCount < 7)
    {
        // Get current descriptor count for monolithic RX queue
        monoRxCount = qm_get_descriptor_count(MONO_RX_Q);
        if (monoRxCount > 6)
            printf(" Number of monolithic packets received in RX queue: %d\n", monoRxCount);
    }


    /* Compare the Monolithic packet data for AxC channels */
    testpass = 1;
    for(chan =0; chan < NUM_AXCS_LTE; chan++)
    {
        rx_count = qm_get_descriptor_count((MONO_RX_Q + chan));
        if(rx_count == 0)testpass = 0;

        for (idx = 0; idx < rx_count; idx ++)
        {
            tmp[chan][idx] = qm_pop_queue((MONO_RX_Q + chan));
            tmp[chan][idx] &= 0xFFFFFFF0;// clean DESC_SIZE field
            temp = (uint32_t *)tmp[chan][idx];
            temp += 4; //skip pkt header and PS field (16 bytes)
    
            if((idx%7) == 0)
            {
                for (idx2 = 0; idx2 < 2208; idx2 ++)if (temp[idx2] != (chan << 24) + (idx << 16)+ idx2)
                {
             	    testpass = 0;
                }
            }
            else 
            {
                for (idx2 = 0; idx2 < 2192; idx2 ++)if (temp[idx2] != (chan << 24) + (idx << 16)+ idx2)
                {
             	    testpass = 0;
                }
            }
    
            qm_push_queue(MONO_RX_FDQ, 1, 0, tmp[chan][idx]);          
        }
    }

    if (testpass == 1)
      printf(" Test a) Monolithic LTE AxC Data Send/Recv: PASS\n");
    else
      printf(" Test a) Monolithic LTE AxC Data Send/Recv: FAIL\n");

    /* read the descriptor counts of the Monolithic queues. */
    value = qm_get_descriptor_count(MONO_TX_Q);
    if (value != 0)
        printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d FAIL\n",value);
    else
        printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d PASS\n",value);

    value = qm_get_descriptor_count(MONO_TX_COMPLETE_Q);
    if (value != NUM_PKTS_ALL_AXCS)
        printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
    else
        printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);

    value = qm_get_descriptor_count(MONO_RX_Q);
    if (value != 0)
        printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
    else
        printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);

    value = qm_get_descriptor_count(MONO_RX_FDQ);
    if (value != NUM_PKTS_ALL_AXCS)
        printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
    else
        printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);


    printf("\nEnding LTE20MHz 4 AxCs loopback test\n");

}


