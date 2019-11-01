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
*  Filename       : doTest.c
*  Date Created   : Dec 18, 2012
*  Last Modified  : Dec 18, 2012
*  Description    : 4x LTE AxCs loopback test
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

//#define DFE_CTL  //uncomment this line for CTL test

#define _IQN2FL_DUMP			1

// test control
#define NUM_AXCS                4
#define NUM_PKTS_PER_AXC        7
#define NUM_PKTS_ALL_AXCS       (NUM_AXCS * NUM_PKTS_PER_AXC)

/* Define queues for common FDQs */
#define MONO_TX_COMPLETE_Q      2000
#define MONO_RX_FDQ             2001
#define MONO_TX_COMPLETE_CTRL_Q 2002
#define MONO_RX_CTRL_FDQ		2003


/* These are for the AIF test */
#define MONO_RX_Q               3000
#define MONO_TX_Q               832
#define MONO_RX_CTRL_Q			3004
#define MONO_TX_CTRL_Q			836

//Users should use 16 bytes aligned data for IQN2 and PktDMA test
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region,".intData_sect")//use MSMC memory for test
#pragma DATA_ALIGN (mono_region, 16)
#endif
uint8_t   mono_region[64 * 8848];//payload size is 8.8K bytes for Normal cyclic prefix 20 MHz LTE

#ifdef DFE_CTL
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region_ctrl,".intData_sect")//use MSMC memory for test mode
#pragma DATA_ALIGN (mono_region_ctrl, 16)
#endif
uint8_t   mono_region_ctrl[32 * 720];//payload size is 704 bytes for DFE control channel
#endif

uint32_t tmp[NUM_AXCS][NUM_PKTS_PER_AXC];
uint32_t tmp_ctrl[NUM_PKTS_PER_AXC];

Iqn2Fl_Status iqn2Status; 

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

Iqn2Fl_Aid2Setup           aid2Setup; // AID setup
Iqn2Fl_AilSetup            ail0Setup; // Setup for AIL0
Iqn2Fl_AilSetup            ail1Setup; // Setup for AIL0
Iqn2Fl_TopSetup            topSetup; // Top registers setup
Iqn2Fl_Iqs2Setup           iqs2Setup; // IQS2 setup
Iqn2Fl_At2Setup            at2Setup; // AT setup

extern void SB_config_4p9152 (Iqn2Fl_LinkRate LinkRate, uint32_t iqn2Base);
extern void iqn2_cfg();
extern unsigned int enable_module(unsigned int pdctl, unsigned int mdctl);
extern void iqn2_bb_loopback_config();
extern void dfe_init();
volatile unsigned int int4_result = 0;

#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif

#define MAX_SLOT		200
volatile uint32_t iqn2SymbolEgressCount[MAX_SLOT];
volatile uint32_t iqn2SymbolIngressCount[MAX_SLOT];
volatile uint32_t getRadt0Frames[MAX_SLOT];
volatile uint32_t getBCNFrame[MAX_SLOT];

interrupt void int4_isr(){
   /*int chan, idx;
   if(int4_result == 10){//push at 10th symbol time to send packet at second frame time
	   for(chan = 0; chan < NUM_AXCS; chan++)
	   {
	       for(idx =0; idx < NUM_PKTS_PER_AXC; idx++)
	       qm_push_queue((MONO_TX_Q + chan), 1, 0, tmp[chan][idx]);
	   }

   }*/
	getRadt0Frames[int4_result]   = Iqn2Obj.regs->At2.AT2_RADT[0].AT2_RADT_1_STS;
	getBCNFrame[int4_result]      = Iqn2Obj.regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
	iqn2SymbolEgressCount[int4_result] = Iqn2Obj.regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_EOP_CNTR_STS;
	iqn2SymbolIngressCount[int4_result] = Iqn2Obj.regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_EOP_CNTR_STS;
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
	uint32_t  flow_a, flow_d;
	uint32_t  flow_a_ctrl, flow_d_ctrl;
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

#ifdef DFE_CTL
    qm_set_memory_region(0, 1,(uint32_t) mono_region_ctrl, 0, 0x002C0000);//use qm 1 and region 1 for control channel
#endif
   /*****************************************************************
    * Configure Linking RAM 0 to use the 16KB internal memory. */
    qm_set_link_ram(0, 0, QMSS_LRAM_REGION, 0x3fff);//3fff(16KB) for Lamarr, 7fff(256KB) for Kepler

    /* Initialize descriptor regions to zero */
    memset(mono_region, 0, 64 * 8848);
#ifdef DFE_CTL
    memset(mono_region_ctrl, 0, 32*720);
#endif

   /* IMPORTANT NOTE: Descriptors (and their attached buffers) must not be
    * linked to other descriptors when added to a Free Queue. Each entry in
    * the free queue is expected to be a single descriptor/buffer pair (for
    * Host mode).  The Rx DMA will ignore the next descriptor pointer and
    * will overwrite it during Rx processing. For Tx, the host should not
    * assume the next ptr is initialized to any value. */

   /* Note: The first several tests require the host buffers to be 256 bytes
    * each.  They are set to 256 here, though they are given space for 512.
    * they will be resized prior to the FFTC test. */
    
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
#ifdef DFE_CTL
    /* Push 7 Monolithic packets into Tx Completion Control Queue*/
    for (idx = 0; idx < 7; idx ++)
    {
       mono_pkt = (MNAV_MonolithicPacketDescriptor *)(mono_region_ctrl + (idx * 720));
       mono_pkt->type_id = MNAV_DESC_TYPE_MONO;
       mono_pkt->data_offset = 12;
       mono_pkt->pkt_return_qmgr = 0;
       mono_pkt->pkt_return_qnum = MONO_TX_COMPLETE_CTRL_Q;
       mono_pkt->packet_length = 700;
       mono_pkt->ps_flags = 0;
       mono_pkt->epib = 0;
       mono_pkt->psv_word_count = 0;

       qm_push_queue(MONO_TX_COMPLETE_CTRL_Q, 1, 0, (uint32_t)(mono_pkt));
     }

     /* Push 7 Monolithic packets to Rx FDQ for control channel */
     for (idx = 16; idx < 23; idx ++)
     {
        mono_pkt = (MNAV_MonolithicPacketDescriptor *)(mono_region_ctrl + (idx * 720));
        mono_pkt->type_id = MNAV_DESC_TYPE_MONO;
        mono_pkt->data_offset = 12;
        mono_pkt->pkt_return_qmgr = 0;
        mono_pkt->pkt_return_qnum = MONO_RX_CTRL_FDQ;

        qm_push_queue(MONO_RX_CTRL_FDQ, 1, 0, (uint32_t)(mono_pkt));
     }
#endif
    /* Create other PktDMA QM configurations. */
    qm_mgr[0] = 0; qm_map[0] = 0;     //QM1 - one qm for Lamarr
    qm_mgr[1] = 0; qm_map[1] = 4096;  //QM1
    pktdma_config_qm(DFE_IQN2_PKTDMA_GBL_CFG_REGION, qm_mgr, qm_map);

    /* PktDMA loopback is enabled by default. this should be disabled for IQN2 test */
    pktdma_config_loopback(DFE_IQN2_PKTDMA_GBL_CFG_REGION, 0);


    /***** Configure Rx channel flows  ************************************/

    //Create flow configuration for the Monolithic packets
    for(idx =0; idx < NUM_AXCS; idx++)
    {
        flow_a = 0x28100000 | (MONO_RX_Q + idx);
        flow_d = MONO_RX_FDQ << 16;
        pktdma_config_rx_flow(DFE_IQN2_PKTDMA_RX_FLOW_REGION, idx,
                   flow_a, 0, 0, flow_d, 0, 0, 0, 0);
    }
#ifdef DFE_CTL
    /*Create flow configuration 0 for the Monolithic packets*/
    flow_a_ctrl = 0x080C0000 | MONO_RX_CTRL_Q;
    flow_d_ctrl = MONO_RX_CTRL_FDQ << 16;
    pktdma_config_rx_flow(DFE_IQN2_PKTDMA_RX_FLOW_REGION, 4,
                  flow_a_ctrl, 0, 0, flow_d_ctrl, 0, 0, 0, 0);
#endif
    //pktDMA channel enable will be done in different place
	
}

void Enable_pktDMA_ch(void)
{
	int idx;
	 /********************** Enable Tx and Rx channels. ******************/
	 for(idx =0; idx < NUM_AXCS; idx++){
	     pktdma_config_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, idx, 0x01000000); //set AIF_MONO_MODE to 1 and set PS filter to zero
	     pktdma_enable_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, idx, 0x80000000);
	     pktdma_enable_rx_chan(DFE_IQN2_PKTDMA_RX_CHAN_REGION, idx, 0x80000000);
	 }
#ifdef DFE_CTL
	 pktdma_enable_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, 4, 0x80000000);// control channel
	 pktdma_enable_rx_chan(DFE_IQN2_PKTDMA_RX_CHAN_REGION, 4, 0x80000000);
#endif
}

void Disable_pktDMA_ch(void)
{
	int idx;
	 /********************** Enable Tx and Rx channels. ******************/
	 for(idx =0; idx < NUM_AXCS; idx++){
	     pktdma_enable_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, idx, 0x00000000);
	     pktdma_enable_rx_chan(DFE_IQN2_PKTDMA_RX_CHAN_REGION, idx, 0x00000000);
	 }
#ifdef DFE_CTL
	 pktdma_enable_tx_chan(DFE_IQN2_PKTDMA_TX_CHAN_REGION, 4, 0x00000000);//control channel
	 pktdma_enable_rx_chan(DFE_IQN2_PKTDMA_RX_CHAN_REGION, 4, 0x00000000);
#endif
}


void iqn2_cfg(void)
{
    uint32_t i;
    Iqn2Fl_UatCfg uat_cfg;

	/************ Initialize IQN2 structures to avoid garbage insertion **************************/
	memset(&iqn2Setup, 0, sizeof(iqn2Setup));
	memset(&aid2Setup, 0, sizeof(aid2Setup));
	memset(&ail0Setup, 0, sizeof(ail0Setup));
	memset(&ail1Setup, 0, sizeof(ail1Setup));
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
	iqn2Setup.aid2Setup = &aid2Setup;

	/************** IQN2 TOP area Setup  ***************************************************/
	topSetup.top_vc_sys_sts_cfg.at_dfe_clk_sel = 1;//use DFE PLL output clk for sub system sys_clk
	topSetup.top_psr_cfg.bw_limit = 0; //use 1/16 VBUS BW for packet flushing
	topSetup.top_psr_cfg.pack_ps_data[0] = 1;//pack ps data for pktDMA chan 0 ~ 3
	topSetup.top_psr_cfg.pack_ps_data[1] = 1;
	topSetup.top_psr_cfg.pack_ps_data[2] = 1;
	topSetup.top_psr_cfg.pack_ps_data[3] = 1;


	/************** IQS Setup  ********************************************************************************/
	//SoC Egress
	for(i=0;i<NUM_AXCS;i++){
	  iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].chan = i;//pktDMA chan 0 ~ 3 is mapped to AID chan 0 ~ 3
	  iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].dest = IQN2FL_AID2_AXC;//destination is AID AxC
	}
	iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[4].chan = 0;//pktDMA chan 4 is mapped to AID CTL chan 0
	iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[4].dest = IQN2FL_AID2_CTL;//destination is AID CTL

	//SoC Ingress
	for(i=0;i<NUM_AXCS;i++){
	  iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i].chan = i;//AID chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
	  iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i].dest = IQN2FL_PKTDMA;//destination is PktDMA
	}
	iqs2Setup.iqs2_ingress_chan_cfg.aid2_ctl_lut_cfg[0].chan = 4;//AID CTL chan 0 is mapped to PktDMA chan 4
    iqs2Setup.iqs2_ingress_chan_cfg.aid2_ctl_lut_cfg[0].dest = IQN2FL_PKTDMA;//destination is PktDMA

	/**************************************************************************************/
	/************** AID Setup  ***********************************************************/
    /**************************************************************************************/

	/** AID SI setup ***/
	//AID SI egress
	for(i=0;i<NUM_AXCS;i++){
	aid2Setup.aid2_iq_efe_cfg_grp.chan_en[i] = 1;
	aid2Setup.aid2_iq_efe_cfg_grp.chan_radio_sel[i] = IQN2FL_CHAN_RADIO_SEL_STD_0;//use radio standard 0 for LTE
	aid2Setup.efe_axc_offset[i] = 0;
	}
	aid2Setup.aid2_iq_efe_radio_std_grp.sym_tc[0] = 139;//140 LTE symbols Set n-1
	aid2Setup.aid2_iq_efe_radio_std_grp.index_sc[0] = 0;
	aid2Setup.aid2_iq_efe_radio_std_grp.index_tc[0] = 6;
	aid2Setup.efe_samp_tc[0] = 2207;//first bigger symbol.
	aid2Setup.efe_samp_tc[1] = 2191;
	aid2Setup.efe_samp_tc[2] = 2191;
	aid2Setup.efe_samp_tc[3] = 2191;
	aid2Setup.efe_samp_tc[4] = 2191;
	aid2Setup.efe_samp_tc[5] = 2191;
	aid2Setup.efe_samp_tc[6] = 2191;

	for(i=0;i<NUM_AXCS;i++){//EFE TDM look up table setup
	aid2Setup.efe_chan_index_cfg[i] = i;
	aid2Setup.efe_chan_index_en_cfg[i] = 1;
	}
	aid2Setup.efe_tdm_start[0] = 0;
	aid2Setup.efe_tdm_len[0] = 4;
	aid2Setup.efe_tdm_en[0] = 1;

	//AID SI ingress
	for(i=0;i<NUM_AXCS;i++){
	aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_en = 1;
	aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_radio_sel = IQN2FL_CHAN_RADIO_SEL_STD_0;//use radio standard 0 for LTE
	aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_axc_offset = 0;//fine AxC offset within QWD level. normally set to zero
	}
	aid2Setup.aid2_iq_ife_radio_std_grp.sym_tc[0] = 139;//140 LTE symbols Set N-1
	aid2Setup.aid2_iq_ife_radio_std_grp.index_sc[0] = 0;
	aid2Setup.aid2_iq_ife_radio_std_grp.index_tc[0] = 6;
	aid2Setup.ife_samp_tc[0] = 2207;//first bigger symbol.
	aid2Setup.ife_samp_tc[1] = 2191;
	aid2Setup.ife_samp_tc[2] = 2191;
	aid2Setup.ife_samp_tc[3] = 2191;
	aid2Setup.ife_samp_tc[4] = 2191;
	aid2Setup.ife_samp_tc[5] = 2191;
	aid2Setup.ife_samp_tc[6] = 2191;

	//AID CTL channel setup
	aid2Setup.ectl_chan_en[0] = 1;
	aid2Setup.ectl_channel[0] = 0;//ctl channel DB threshold. 0 means any data in the buffer
	aid2Setup.ictl_pkt_if_chan_en[0] = 1;
	aid2Setup.ectl_rate_ctl_cfg = 7;//suppress eCTLBW to 50 %
	aid2Setup.ictl_rate_ctl_cfg_rate = 7;//suppress iCTL BW to 50 %

	/******* AID uAT setup **********************************/
	//AID SI Egress RADT0 with event0
	aid2Setup.uat_egr_radt_tc_cfg_val[0] = 2457599;//CPRI frame length with 245.76 MHz clock
	aid2Setup.uat_egr_radt_offset_cfg_val[0] = 0;//not used for DIAG_SYNC test mode
	aid2Setup.uat_evt_radt_cmp_cfg_val[0] = 200;//200 clock delay for AID SI event start
	aid2Setup.uat_evt_clk_cnt_tc_cfg_val[0] = 31;//4 sample event (LTE20MHz)

	//AID SI Ingress RADT0 with event8
	aid2Setup.uat_ing_radt_tc_cfg_val[0] = 2457599;//CPRI frame length with 245.76 MHz clock
	aid2Setup.uat_ing_radt_offset_cfg_val[0] = 0;//not used for DIAG_SYNC test mode
	aid2Setup.uat_evt_radt_cmp_cfg_val[8] = 200;//200 clock delay for AID SI event start
	aid2Setup.uat_evt_clk_cnt_tc_cfg_val[8] = 31;//4 sample event (LTE20MHz)

	/************** AT Setup  **************************/
    //BCN timer
	at2Setup.at2_bcn_clkcnt_tc_cfg_val = 2457599;
	at2Setup.at2_bcn_frame_tc_lsb_cfg_val = 4095;

	//RAD timer 0 (LTE20MHz)
	at2Setup.at2_radt_0_cfg_clkcnt_tc[0] = 7;//8 clock count in one sample (LTE 20MHz)
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
	at2Setup.at2_events_24array.at2_events_24array_offset_cfg_evt_strb_sel[0] = IQN2FL_RADT0_SYMBOL;//select RAD0 symbol strobe
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

	if (iqn2Status != IQN2FL_SOK)printf ("\nIQN2 HW setup error 0x%x",iqn2Status);

	ctrlArg = 0x55555555;
	//global enable of AID (Egress, Ingress)
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
#ifdef DFE_CTL
	//global enable of AID CTL (Egress, Ingress)
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
#endif

	//Enable AID uAT with DIAG_SYNC
	uat_cfg.diag_sync = 1;
	uat_cfg.uat_run = 1;
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AID2_UAT_GEN_CTL_UAT_CFG, (void *)&uat_cfg);

	ctrlArg = 0x1;//Enable AT event 0
	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG, (void *)&ctrlArg);

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

    printf("\nBeginning LTE20MHz 4AxCs + 1 Control AID loopback test\n");

    config_l1d_cache(0);//disable L1D cache to avoid MSMC region invalidation

    memset((void*)&iqn2SymbolEgressCount[0], 0, sizeof(iqn2SymbolEgressCount));
    memset((void*)&iqn2SymbolIngressCount[0], 0, sizeof(iqn2SymbolEgressCount));

//    //DFE pll should be set correctly before turning on DFE power domains
    enable_module(dfe_iqn_sys_pdctl, dfe_iqn_sys_mdctl);//AID, IQS, PSR, AT, DIO power on
    enable_module(iqn_ail_pdctl, iqn_ail_mdctl);// do not change this enable order
    enable_module(dfe_pd1_pdctl, dfe_pd1_mdctl);
    enable_module(dfe_pd0_pdctl, dfe_pd0_mdctl);
    enable_module(dfe_pd2_pdctl, dfe_pd2_mdctl);

    Intc_config();//GEM interrupt controller setup


    /****** DFE setup for egress to ingress digital loopback (LTE use case) **************/
    iqn2_bb_loopback_config(); //enable DFE BB loopback for simple AID LTE loopback test

    dfe_init();//init basic DFE setup
    /*************************************************************************/

    MNavigator_config();//multicore navigator configuration for LTE

    // push AxC packets
    for(chan = 0; chan < NUM_AXCS; chan++)
    {    	
        for(idx =0; idx < NUM_PKTS_PER_AXC; idx++)
        {  
            //push 42 packets into Tx queue for NUM_AXCS channels 
    	
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
            
            qm_push_queue((MONO_TX_Q + chan), 1, 0, tmp[chan][idx]); // push also can be done in ISR
              
        }
    
    }
#ifdef DFE_CTL
    //push 7 DFE control packets
    for(idx = 0; idx<NUM_PKTS_PER_AXC; idx++){
      tmp_ctrl[idx] = qm_pop_queue(MONO_TX_COMPLETE_CTRL_Q);
      tmp_ctrl[idx] &= 0xFFFFFFF0;//clean DESC_SIZE field

      temp = (uint32_t *)(mono_region_ctrl+ (idx * 720) + 12);
      for (idx2 = 0; idx2< 176;idx2++)temp[idx2] = idx2;//fill sequential data

      qm_push_queue(MONO_TX_CTRL_Q, 1, 0, tmp_ctrl[idx]); // push also can be done in ISR
    }
#endif

    //SERDES configuration
    SB_config_4p9152 (IQN2FL_LINK_RATE_8x, (uint32_t)CSL_IQN_CFG_REGS);

    //load IQN2 configuration for DFE emulation
    iqn2_cfg();

    Enable_pktDMA_ch();//enable pktDMA channel

    /****** wait for data transfer completion.***********************/
    while(1)
    {
        asm (" NOP 5 ");
        asm (" NOP 5 ");
        if(int4_result == 14)// Wait 1 ms (no frame delay required for DFE)
         {
        	Disable_pktDMA_ch();
        	reset.sw_reset =1;//IQN2 full reset
        	Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB, (void *)&reset);
            break;
         }
    }


    monoRxCount = 0;  // Rx descriptor count for monolithic RX queue
    while (monoRxCount != 7)
    {
        // Get current descriptor count for monolithic RX queue
        monoRxCount = qm_get_descriptor_count(MONO_RX_Q);
        if (monoRxCount > 6)
            printf(" Number of monolithic packets received in RX queue: %d\n", monoRxCount);
#ifdef DFE_CTL
        monoRxCount = qm_get_descriptor_count(MONO_RX_CTRL_Q);
        if (monoRxCount > 6)
            printf(" Number of monolithic packets received in RX Control queue: %d\n", monoRxCount);
#endif
    }


    /* Compare the Monolithic packet data for AxC channels */
    testpass = 1;
    for(chan =0; chan < NUM_AXCS; chan++)
    {
        rx_count = qm_get_descriptor_count((MONO_RX_Q + chan));

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
      printf(" Test a) Monolithic AxC Data Send/Recv: PASS\n");
    else
      printf(" Test a) Monolithic AxC Data Send/Recv: FAIL\n");

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

#ifdef DFE_CTL
    /* Compare the Monolithic packet data for Control channel */
    testpass = 1;
    rx_count = qm_get_descriptor_count(MONO_RX_CTRL_Q);

    for (idx = 0; idx < rx_count; idx ++)
    {
         tmp_ctrl[idx] = qm_pop_queue(MONO_RX_CTRL_Q);
         tmp_ctrl[idx] &= 0xFFFFFFF0;// clean DESC_SIZE field
         temp = (uint32_t *)tmp_ctrl[idx];
         temp += 3; //skip pkt header

         for(idx2 = 0; idx2< 176; idx2++){
        	 if(temp[idx2] != idx2)testpass = 0;
         }

         qm_push_queue(MONO_RX_CTRL_FDQ, 1, 0, tmp_ctrl[idx]);
     }

    if (testpass == 1)
      printf(" Test c) Monolithic Control Data Send/Recv: PASS\n");
    else
      printf(" Test c) Monolithic Control Data Send/Recv: FAIL\n");

    /* read the descriptor counts of the Monolithic queues. */
    value = qm_get_descriptor_count(MONO_TX_CTRL_Q);
    if (value != 0) 
        printf(" Test d1) Monolithic Control Packet Tx Descriptor Counts:%d FAIL\n",value);
    else 
        printf(" Test d1) Monolithic Control Packet Tx Descriptor Counts:%d PASS\n",value);

    value = qm_get_descriptor_count(MONO_TX_COMPLETE_CTRL_Q);
    if (value != NUM_PKTS_PER_AXC)
        printf(" Test d2) Monolithic Control Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
    else 
        printf(" Test d2) Monolithic Control Packet Tx Complete Descriptor Counts:%d PASS\n",value);

    value = qm_get_descriptor_count(MONO_RX_CTRL_Q);
    if (value != 0) 
        printf(" Test d3) Monolithic Control Packet Rx Descriptor Counts:%d FAIL\n",value);
    else 
        printf(" Test d3) Monolithic Control Packet Rx Descriptor Counts:%d PASS\n",value);

    value = qm_get_descriptor_count(MONO_RX_CTRL_FDQ);
    if (value != NUM_PKTS_PER_AXC)
        printf(" Test d4) Monolithic Control Packet Rx Free Descriptor Counts:%d FAIL\n",value);
    else 
        printf(" Test d4) Monolithic Control Packet Rx Free Descriptor Counts:%d PASS\n",value);
#endif

    printf("\nEnding LTE20MHz 4AxCs + 1 Control AID loopback test\n");

}

