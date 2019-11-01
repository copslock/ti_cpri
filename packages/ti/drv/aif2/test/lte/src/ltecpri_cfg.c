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

#ifdef CFG

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/aif2/device/aif2_device.h>

#include <ti/drv/aif2/test/utils/cslUtils.h>

#define    _AIF2FL_DUMP		0
#define     DIO_0               AIF2FL_DIO_ENGINE_0 // DIO engines are not used in LTE mode - instead PKTDMA and HW queues are used to carry the antenna data from/to AIF2
#define     LTE_RATE            20
#define     SYSCLK_INPUT_KHZ    122880      // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#define     TEST_NUM_MAX        1
#define     NB_LINKS            1
#if LTE_RATE == 20
#define     NBCHANNELMAXPERLINK 2   // Max number of AxCs per link
#elif LTE_RATE == 10
#define     NBCHANNELMAXPERLINK 4   // Max number of AxCs per link
#elif LTE_RATE == 5
#define     NBCHANNELMAXPERLINK 8   // Max number of AxCs per link
#elif LTE_RATE == 40
#define     NBCHANNELMAXPERLINK 1   // Max number of AxCs per link
#endif
#if LTE_RATE == 40
#define     NBCHANNELPERLINK    1                   // Number of AxCs in use per link (fixed to 1)
#else
#define     NBCHANNELPERLINK    2                   // Number of AxCs in use per link (fixed to 2)
#endif

typedef struct {
        // Test name
        const unsigned char name[32];
        //AIF link enable or disable, 1 or 0
        uint32_t                   linkEnable[AIF_MAX_NUM_LINKS];
        //AIF link rate (1=1x; 2=2x; 4= 4x).
        uint32_t                   linkRate[AIF_MAX_NUM_LINKS];
        //AIF link data type for outbound burst traffic.
        Aif2Fl_LinkDataType     outboundDataType[AIF_MAX_NUM_LINKS];
        //AIF link data width for outbound burst traffic.
        Aif2Fl_DataWidth        outboundDataWidth[AIF_MAX_NUM_LINKS];
        //AIF link data type for inbound burst traffic.
        Aif2Fl_LinkDataType     inboundDataType[AIF_MAX_NUM_LINKS];
        //AIF link data width for inbound burst traffic.
        Aif2Fl_DataWidth        inboundDataWidth[AIF_MAX_NUM_LINKS];
        //AIF link DIO engine used
        Aif2Fl_DioEngineIndex   dioEngine[AIF_MAX_NUM_LINKS];
        } TestObj;

/* IQN2 Global structures and variables  */
AIF_ConfigObj	 			aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile TestObj            testObjTab[TEST_NUM_MAX] = {
    {//1st configuration - CPRI 4x - Link 3 is used on CPRI relay setup to connect TCI6614 EVM and SCBP via SFP
#if LTE_RATE == 20
      "LTE_20MHZ", // test name
#elif LTE_RATE == 10
      "LTE_10MHZ", // test name
#elif LTE_RATE == 5
      "LTE_5MHZ", // test name
#elif LTE_RATE == 40
      "LTE_40MHZ", // test name
#endif
     // link0          link1          link2          link3          link4          link5
      {1,             0,             0,             0,             0,             0            },  // link enable
      {4,             4,             4,             4,             4,             4            },  // link rate
      {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
      {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
      {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
      {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
      {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
    }
};

#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

int32_t main(void)
{
    uint32_t i, chan, nblink, ntest=0, activeLink, idx, ctrlArg;
#ifdef _TMS320C6X
        /* Make shared memory (MSM) non cacheable for the purpose of testing */
        CSL_XMC_invalidatePrefetchBuffer();
        CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
        CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

        CACHE_setL1DSize (CACHE_L1_32KCACHE);
        CACHE_setL1PSize (CACHE_L1_32KCACHE);
#endif

        printf("Software configuration: %s\n", testObjTab[0].name);
        memset(&aifObj, 0, sizeof(aifObj));

        aif2_mapDevice();

        // Take AIF out of power saver
        AIF_enable();
        AIF_pscDisableResetIso();

        // General parameters
        aifObj.aif2ClkSpeedKhz     = (uint32_t)SYSCLK_INPUT_KHZ;
        aifObj.protocol            = AIF2FL_LINK_PROTOCOL_CPRI;
        aifObj.pktdmaOrDioEngine   = AIF2FL_CPPI;
        aifObj.mode                = AIF_LTE_FDD_MODE;
        aifObj.superPacket         = false;
        aifObj.aif2TimerSyncSource = AIF2FL_SW_SYNC;
#ifndef _TMS320C6X
        aifObj.hAif2SerDesBaseAddr = &aif2InitCfg;
#endif


        chan    = 0;
        nblink  = 0;

        for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        {
            if (testObjTab[ntest].linkEnable[i]==1 && (aifObj.linkConfig[i].RtEnabled==0))
            {
                nblink++;
                // Enable next link if required
                if ((NB_LINKS == 2) && (i<(AIF_MAX_NUM_LINKS-1))) testObjTab[ntest].linkEnable[i+1]=1;
                // PktDma parameters
                for (idx=0 ; idx<NBCHANNELPERLINK ; idx++)
                {
                    chan++;
                }
            }
        }
        //nbchanneltotal = NBCHANNELPERLINK*nblink;

        //AIF_resetAif(&aifObj);

        {
            aifObj.radTimerConfig[0].userSpecified                      = true;
            aifObj.radTimerConfig[0].frameTerminalCount                 = 4096;
            aifObj.radTimerConfig[0].initClockNum                       = 0;
            aifObj.radTimerConfig[0].initSymbolNum                      = 0;
            aifObj.radTimerConfig[0].initFrameLsbNum                    = 0;
            aifObj.radTimerConfig[0].initFrameMsbNum                    = 0;
            aifObj.radTimerConfig[0].lte.cpType                         = AIF2_LTE_CPTYPE_NORMAL;
            aifObj.radTimerConfig[0].lte.numSymbolsForSymbolStrobe      = 1;
            aifObj.radTimerConfig[0].lte.numSymbolStrobesForFrameStrobe = 140;
        }

        for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        {

             aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
             if (testObjTab[ntest].linkEnable[i] == 1)
             {
                 aifObj.linkConfig[i].numPeAxC           = NBCHANNELPERLINK;
                 aifObj.linkConfig[i].numPdAxC           = NBCHANNELPERLINK;
             }
             aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
#if LTE_RATE == 20
             aifObj.linkConfig[i].sampleRate         = AIF_SRATE_30P72MHZ;
             aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_8b8;
#elif LTE_RATE == 10
             aifObj.linkConfig[i].sampleRate         = AIF_SRATE_15P36MHZ;
             aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_4b4;
#elif LTE_RATE == 5
             aifObj.linkConfig[i].sampleRate         = AIF_SRATE_7P68MHZ;
             aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_2b2;
#elif LTE_RATE == 40
             aifObj.linkConfig[i].sampleRate         = AIF_SRATE_61P44MHZ;
             aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
#endif
             aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
             aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
             aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
             aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
             aifObj.linkConfig[i].psMsgEnable        = 0;
             aifObj.linkConfig[i].comType            = AIF2_LOOPBACK;
             aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i]; //NA for pkDMA
        }

        for (i= 0 ; i<AIF_MAX_NUM_LINKS; i++)
        {
            if (aifObj.linkConfig[i].linkEnable)
            {
                activeLink = i;
                break;
            }
        }

        if ((NB_LINKS == 2) && (activeLink<(AIF_MAX_NUM_LINKS-1))) {
            aifObj.linkConfig[activeLink+1].nodeTx      = aifObj.linkConfig[activeLink].nodeTx;
            aifObj.linkConfig[activeLink+1].nodeRx      = aifObj.linkConfig[activeLink].nodeRx;
        }

        // Compute default AIF2 parameters given this user configuration
        AIF_calcParameters(&aifObj);

        if ((NB_LINKS == 2) && (activeLink<(AIF_MAX_NUM_LINKS-1))) {
            aifObj.linkConfig[activeLink+1].piMin      = aifObj.linkConfig[activeLink].piMin;
        }

        // initialization function for the AIF2 H/W CSL structure (can still be overridden afterwards)
        AIF_initHw(&aifObj);

        // Disabling this mode, so that, on Cpri relay setup, once AIF2 timers are triggered, they keep running based on the initial pulse. Note there isn't a periodic 10ms pulse generated from EVMTCI6614, just a one shot pulse.
        aifObj.hAif2Setup->commonSetup->pAtCommonSetup->AutoResyncMode = AIF2FL_NO_AUTO_RESYNC_MODE;

        if ((NB_LINKS == 2) && (activeLink<(AIF_MAX_NUM_LINKS-1))) {
            aifObj.hAif2Setup->linkSetup[activeLink+1]->pAtLinkSetup->PiMax      = aifObj.hAif2Setup->linkSetup[activeLink]->pAtLinkSetup->PiMax;
        }

#if _AIF2FL_DUMP == 1
        if (DSP_procId == 1)
        {
           FILE *fout;
           fout = fopen("aif2fl_dump.txt","w");
           if (fout)
           {
               dump_Aif2Fl_Setup(fout,aifObj.hAif2Setup);
               fclose(fout);
           }

        }
#endif
        // Now programming AIF2 registers
        AIF_startHw(&aifObj);

        ctrlArg = 1;
        if (aifObj.aif2TimerSyncSource== AIF2FL_SW_SYNC) {
            Aif2Fl_hwControl(aifObj.hFl, AIF2FL_CMD_AT_DEBUG_SYNC, (void *)&ctrlArg);
        }

        /* Wait for first radio frame with valid DL transmission*/
        while (aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS !=2) {
            UTILS_waitForHw(1000);
        }

        /* Wait for first lte symbols to be transmitted */
        while ( CSL_FEXT(aifObj.hFl->regs->AT_RADT_VALUE_LSBS,AIF2_AT_RADT_VALUE_LSBS_RADTSYMBOL_COUNT_VALUE) !=14) {
            UTILS_waitForHw(1000);
        }

        AIF_enableException(&aifObj);

        /* Wait for most lte symbols to be transmitted in this radio frame */
        while ( CSL_FEXT(aifObj.hFl->regs->AT_RADT_VALUE_LSBS,AIF2_AT_RADT_VALUE_LSBS_RADTSYMBOL_COUNT_VALUE) !=126) {
            UTILS_waitForHw(1000);
        }

        AIF_getException(&aifObj);

        printf("Ending AIF2 configuration test \n");
        return (0);
}

#endif

////////////////////////////////
