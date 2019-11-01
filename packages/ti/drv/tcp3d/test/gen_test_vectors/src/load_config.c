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



#include <stdio.h>
#include<string.h>
#include <math.h>

#include "sim_param.h"
#include "cfg_param.h"


void SetDefaultConfig(TCP3_SIM_PARMS *sparms);

//********************************************************************
//Loads parameters from the configuration file into sparms structure
//********************************************************************
void LoadConfig(char *cfgFileName, TCP3_SIM_PARMS *sparms)
{
FILE *cfgFile;
char buffer[300];
char *pb,*pt;
int32_t itemp;
float ftemp;

    //Set default values into sparms
    SetDefaultConfig(sparms);

    if( (cfgFile = fopen(cfgFileName,"r")) == NULL)
    {
       printf("Configuration file %s is not found!\n",cfgFileName);
       return;
    }
    while(fgets(buffer,300,cfgFile) != NULL) 
    {
      if(buffer[0] == '#')  continue;   /* skip the line if it begins with the comment */
      if((pb = strchr(buffer,'=')) != NULL) /* process the line if it contains '=' */
      { 
         pb = strtok(buffer, " \t=#");  /* position to parameter descriptor */
         pt = strtok(NULL, " \t=#");    /* position to its value */

         if(strcmp(pb,"Coding_standard") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->CodingStandard = itemp;
            continue;
         }
         if(strcmp(pb,"Frame_size_index") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->frameLenInd = itemp;
            continue;
         }
         if(strcmp(pb,"Minimum_number_of_FEC_blocks") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->MinNumFecBlocks = (uint32_t) itemp;
            continue;
         }
         if(strcmp(pb,"Maximum_number_of_FEC_blocks") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->MaxNumFecBlocks = (uint32_t) itemp;
            continue;
         }
         if(strcmp(pb,"Minimum_number_of_frame_errors") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->MinNumFerErrors = (uint32_t) itemp;
            continue;
         }

         if(strcmp(pb,"Frame_error_rate_limit") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->FrameErrorRateLimit = itemp;
            continue;
         }
         if(strcmp(pb,"Coded_error_processing") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->ErrorProcessingOption = (uint32_t) itemp;
            continue;
         }

         if(strcmp(pb,"Snr_init_value") == 0) {
            sscanf(pt, "%f", &ftemp);
            sparms->SnrInitValue = ftemp;
            sparms->noiseSigma = (float) (1.0 / pow(10.0, (double)sparms->SnrInitValue/20.0));
            continue;
         }
         if(strcmp(pb,"Snr_increment_step") == 0) {
            sscanf(pt, "%f", &ftemp);
            sparms->SnrIncrementStep = ftemp;
            continue;
         }
         if(strcmp(pb,"Snr_limit_value") == 0) {
            sscanf(pt, "%f", &ftemp);
            sparms->SnrLimitValue = ftemp;
            continue;
         }
         if(strcmp(pb,"Adaptive_snr_step") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->AdaptiveSnrStep = itemp;
            continue;
         }

         if(strcmp(pb,"Add_noise") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->NoiseSwitch = itemp;
            continue;
         }
         if(strcmp(pb,"c_model_seed") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->seed = itemp;
            if(itemp<512) itemp <<= 9;//Fix for 23 bit scrambler register. The register occupies upper 23 bits
            sparms->Scr2318ShiftReg = itemp;

            continue;
         }
         if(strcmp(pb,"Bit_width_of_integer_part") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->bitWidthInt = itemp;
            continue;
         }

         if(strcmp(pb,"Bit_width_of_fractional_part") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->bitWidthFrac = itemp;
            continue;
         }
         if(strcmp(pb,"Max_star_enable") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->maxStarEn = itemp;
            continue;
         }
         if(strcmp(pb,"Max_star_threshold") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->maxStarThreshold = itemp;
            continue;
         }
         if(strcmp(pb,"Max_star_value") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->maxStarValue = itemp;
            continue;
         }


         if(strcmp(pb,"Max_number_of_turbo_iterations") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->MaxNumTurboIterations = itemp;
            continue;
         }
         if(strcmp(pb,"Min_number_of_turbo_iterations") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->MinNumTurboIterations = itemp;
            continue;
         }

         if(strcmp(pb,"Extrinsic_scales") == 0) {
             {
                 int32_t i;
                 for(i=0; i<16; i++)
                 {
                    sscanf(pt, "%d", &itemp);
                    sparms->extrinsicScales[i] = itemp;
                    pt = strtok(NULL, " \t=#,");
                    if(pt == NULL) 
                        break;
                 }
             }
            continue;
         }

         if(strcmp(pb,"tcp3_SW0_length") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_SW0_length = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_stopSel") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_stopSel = itemp; //=0 Max iter, =1 CRC, =2 or 3  SNR 
            continue;
         }

         if(strcmp(pb,"tcp3_crcSel") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_crcSel = itemp; //=0 gCRC24B, =1 gCRC24A
            continue;
         }

         if(strcmp(pb,"tcp3_SNR_Report") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_SNR_Report = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_SNR_stopVal") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_SNR_stopVal = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_intlvGenEn") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_intlvGenEn = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_intlvLoadSel") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_intlvLoadSel = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_extrScaleEn") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_extrScaleEn = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_softOutBitFormat") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_softOutBitFormat = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_outBitOrderSel") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_outBitOrderSel = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_lteCrcInitSel") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_lteCrcInitSel = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_lteCrcIterPass") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_lteCrcIterPass = itemp;
            continue;
         }

         if(strcmp(pb,"tcp3_softOutBitsReadEn") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_softOutBitsReadEn = itemp;
            continue;
         }


         if(strcmp(pb,"tcp3_outStatusReadEn") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->tcp3_outStatusReadEn = itemp;
            continue;
         }


         if(strcmp(pb,"Enable_top_level_device_verification") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->enableTopLvlDeviceVerification = itemp;
            continue;
         }

         if(strcmp(pb,"Enable_device_verification") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->enableDeviceVerification = itemp;
            continue;
         }

         if(strcmp(pb,"Belief_propagation_disabled") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->disableBeliefPropagation = itemp;
            continue;
         }
     
         if(strcmp(pb,"Alpha_belief_propagation_disabled") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->disableAlphaBeliefPropagation = itemp;
            continue;
         }
     
         if(strcmp(pb,"Beta_belief_propagation_disabled") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->disableBetaBeliefPropagation = itemp;
            continue;
         }
     
         if(strcmp(pb,"Dev_ver_alpha_block_number") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->devVerAlphaBlockNumber = itemp;
            continue;
         }

         if(strcmp(pb,"Dev_ver_extr_block_number") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->devVerExtrBlockNumber = itemp;
            continue;
         }

         if(strcmp(pb,"Dev_ver_beta_block_number") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->devVerBetaBlockNumber = itemp;
            continue;
         }

         if(strcmp(pb,"Dev_ver_use_linear_interleaver") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->devVerUseLinearInterleaver = itemp;
            continue;
         }

         if(strcmp(pb,"Dev_ver_zero_apriori_in_hard_dec") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->devVerZeroAprioriInHardDec = itemp;
            continue;
         }



         if(strcmp(pb,"Send_intermedite_internal_memories") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->devVerSendIntermediteInternalMemories = itemp;
            continue;
         }


     
         if(strcmp(pb,"Save_intermediate_data") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->saveIntermediateData = itemp;
            continue;
         }

         if(strcmp(pb,"Initial_process_index") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->initialProcInd = itemp;
            continue;
         }

         if(strcmp(pb,"Alternate_process_index") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->alternateProcInd = itemp;
            continue;
         }

         if(strcmp(pb,"Belief_prop_within_turbo_iter_enabled") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->beliefPropWithinTurboIterEnabled = itemp;
            continue;
         }

         if(strcmp(pb,"Puncture_interval") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->punctureInterval = itemp;
            continue;
         }

         if(strcmp(pb,"Enable_rate_matching") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->enableRateMatching = itemp;
            continue;
         }

         if(strcmp(pb,"Redundancy_version_number") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->redundancyVersionNumber = itemp;
            continue;
         }



         if(strcmp(pb,"Coding_rate") == 0) {
            sscanf(pt, "%f", &ftemp);
            sparms->codingRate = ftemp;
            continue;
         }

         if(strcmp(pb,"Load_info_bits_from_file") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->loadInfoBitsFromFile = itemp;
            continue;
         }
         if(strcmp(pb,"Store_info_bits_to_file") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->storeInfoBitsToFile = itemp;
            continue;
         }
         if(strcmp(pb,"Info_bits_file_name") == 0) {
            sscanf(pt, "%s", sparms->infoBitsFileName);
            continue;
         }
         if(strcmp(pb,"Store_coded_bits_to_file") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->storeCodedBitsToFile = itemp;
            continue;
         }
         if(strcmp(pb,"Coded_bits_file_name") == 0) {
            sscanf(pt, "%s", sparms->codedBitsFileName);
            continue;
         }
         if(strcmp(pb,"Info_bits_file_includes_CRC") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->infoBitsFileIncludesCRC = itemp;
            continue;
         }
         if(strcmp(pb,"Use_tcp3_encoder_c_model") == 0) {
            sscanf(pt, "%d", &itemp);
            sparms->use_tcp3_encoder_c_model = itemp;
            continue;
         }
     

      }
    }
    fclose(cfgFile);
}

//********************************************************************
//********************************************************************
void SetDefaultConfig(TCP3_SIM_PARMS *sparms){
int32_t i;

	sparms->CodingStandard = 0;
    sparms->MaxNumTurboIterations = 8;
    sparms->MinNumTurboIterations = 1;
    sparms->frameLenInd = 0;
    sparms->mappingSign = 1;
    sparms->seed = 0x55555555;
    sparms->Scr2318ShiftReg = 0xffffffff;
    sparms->SnrInitValue = -2.;
    sparms->SnrIncrementStep = 0.25;
    sparms->AdaptiveSnrStep     = 0;
    sparms->SnrLimitValue = 10.;
    sparms->noiseSigma = (float) (1.0 / pow(10.0, (double)sparms->SnrInitValue/20.0));
    sparms->bitWidthInt = 4;
    sparms->bitWidthInt = 2;
    sparms->maxStarEn = 0; // =0 MaxLogMAP, =1 MAXStar 

    sparms->ErrorProcessingOption = 1;
    sparms->FrameErrorRateLimit = -1;
    sparms->MinNumFecBlocks = 100;
    sparms->MaxNumFecBlocks = 1000;
    sparms->MinNumFerErrors = 0;


    for(i=0; i<16; i++)
    {
        sparms->extrinsicScales[i] = 32;
    }
    sparms->tcp3_SW0_length = 64;
 
    sparms->tcp3_stopSel = 0; //=0 Max iter, =1 CRC, =2 or 3  SNR
    sparms->tcp3_crcSel = 0; //=0 gCRC24B, =1 gCRC24A
    sparms->tcp3_SNR_stopVal = 20;
    sparms->tcp3_SNR_Report = 0;
    sparms->tcp3_intlvGenEn = 0;
    sparms->tcp3_intlvLoadSel = 1;
    sparms->tcp3_extrScaleEn = 1;
    sparms->tcp3_softOutBitFormat = 1;
    sparms->tcp3_outBitOrderSel = 0;
    sparms->tcp3_lteCrcInitSel = 0;
    sparms->tcp3_lteCrcIterPass = 0;
    sparms->tcp3_softOutBitsReadEn = 0;
    sparms->tcp3_outStatusReadEn = 0;
    
    
    sparms->enableTopLvlDeviceVerification = 0;
    sparms->enableDeviceVerification = 0;
    sparms->disableBeliefPropagation = 0;
    sparms->disableAlphaBeliefPropagation = 0;
    sparms->disableBetaBeliefPropagation = 0;

    sparms->devVerAlphaBlockNumber = 0;
    sparms->devVerExtrBlockNumber = 0;
    sparms->devVerBetaBlockNumber = 0;
    sparms->devVerSendIntermediteInternalMemories = 0;
    sparms->devVerUseLinearInterleaver = 0;
    sparms->devVerZeroAprioriInHardDec = 0;


    sparms->maxStarThreshold = 4;
    sparms->maxStarValue = 2;
    sparms->saveIntermediateData = 0;

    sparms->alternateProcInd = 0;
    sparms->initialProcInd = 0;

    sparms->beliefPropWithinTurboIterEnabled = 0;

    sparms->punctureInterval = -1;
    sparms->enableRateMatching = 0;
    sparms->redundancyVersionNumber = 0;
    sparms->codingRate = 0;
    sparms->loadInfoBitsFromFile = 0;
    sparms->storeCodedBitsToFile = 0;
    sparms->storeInfoBitsToFile = 0;
    sparms->infoBitsFileIncludesCRC = 0;
    sparms->infoBitsFileName[0] = 0;
    sparms->codedBitsFileName[0] = 0;
    sparms->use_tcp3_encoder_c_model = 0;
}

