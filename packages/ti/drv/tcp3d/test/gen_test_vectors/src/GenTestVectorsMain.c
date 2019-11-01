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



#include <windows.h> 
#include <process.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include <sys/stat.h>
//#include <sys/types.h>
#include <direct.h>
#include "typedefs.h"
#include "sim_param.h"
#include "cfg_param.h"

char *folderName;
char *folderNameOut;
char *listFileName;
char fullName[300];
char **codeBlockList;
int32_t numCodeBlocks;
int32_t codeBlockCntr;
char *outFileType = {"BOTH"};

TCP3_SIM_PARMS sparms;
TCP3_REGS reg;
OUT_TCP3_REGS outReg;

int32_t readCodeBlockList(char * blockListName);
void freeCodeBlockList(void);
void SetReg(TCP3_SIM_PARMS *sparms, TCP3_REGS *reg);
void LoadConfig(char *cfgFileName, TCP3_SIM_PARMS *sparms);
int32_t get_code_block(char *cfgFileName, int32_t Cntr);
int32_t save_code_block_files(char *folderName, int32_t cntr);
void SetOutReg(TCP3_REGS *reg, OUT_TCP3_REGS *outReg);
void conv2bin(char *infile, char *outfile, int type);

//********************************************************************
//This program generates input test vectors for the TCP3d driver
//Folder structure:
//+---gen_test_vectors
//¦   +---include
//¦   +---msvc
//¦   ¦   +---Debug
//¦   +---simulator
//¦   ¦   +---debug
//¦   +---src
//¦   ¦   +---Debug
//¦   +---test1_split
//¦   +---test2_split
//¦   +---test3_split
//
// Program GenerateTestVectors.exe is executed from the folder gen_test_vectors.
// Usage form dos prompt:
//  1) Goto gen_test_vectors directory
//  2) Then ./msvc/debug/GenerateTestVectors.exe folder_name list_file_name
// Program arguments: 
//          folder_name    - Folder name with the configuration files in the gen_test_vectors.
//                           Example folders: test1_split, test2_split,...
//          list_file_name - List file with the configuration file names to be processed. Configuration
//                           files must be in the same folder. All configuration files in the folder 
//                           should be set for the same mode (lte, wimax or wcdma), and one code block
//                           per file. 
//                           The following configuration parmeters must be set as:
//                                    Save_intermediate_data    = 1
//                                    Minimum_number_of_FEC_blocks = 1
//                                    Maximum_number_of_FEC_blocks = 1
//                                    Snr_increment_step = 0
//                                    Minimum_number_of_frame_errors = 0
// Program opens the the list file, reads configuration file names and
// per each configuration file perfoms:
// 1. Copies configuration file to config.cfg in the simulator folder,
// 2. Runs the simulator executable from the simulator\debug folder,
// 3. Copies generated files form simulator\debug folder to the folder_name.
// 
//********************************************************************
void main(int argc, char * argv[])
{
    int32_t isFound;
    FILE *fid;

    if(argc < 3)
    {
        printf("Program %s must be called with minimum 2 arguments:\n", argv[0]);
        printf("  Syntax:\n\t%s folder_name list_file_name <dest_folder_name> <out_file_format>\n", argv[0]);
        return;
    }

    folderName =  argv[1];
    listFileName = argv[2];
    strcpy(fullName, folderName);
    strcat(fullName, "\\");
    strcat(fullName, listFileName);

    if (argc == 4)
    {
        folderNameOut =  argv[3];
    }
    else
    {
        folderNameOut =  argv[1];
    }

    if (argc == 5)
    {
        outFileType =  argv[4];
    }

    isFound = readCodeBlockList(fullName);
    if(isFound == 0)
    {
        printf("List file %s is not found!\n", fullName);
        return;
    }


    for(codeBlockCntr=0; codeBlockCntr<numCodeBlocks; codeBlockCntr++)
    {
        strcpy(fullName, folderName);
        strcat(fullName, "\\");
        strcat(fullName, codeBlockList[codeBlockCntr]);
        printf("Config file[%d] =  %s\n",codeBlockCntr, fullName);
        isFound = get_code_block(fullName, codeBlockCntr);
        if(isFound == 0)
        {
            printf("Config file not found!\n", codeBlockCntr);
            break;
        }
        save_code_block_files(folderNameOut, codeBlockCntr);
    }

    //Write number of blocks
    strcpy(fullName, folderName);
    strcat(fullName, "\\number_of_blocks.dat");
    fid = fopen(fullName,"w");
    fprintf(fid, "%d\n", numCodeBlocks);
    fclose(fid);

    freeCodeBlockList();
    
}

//********************************************************************
// Reads the code block list. The list contains the TCP3 C model
// configuratin file names. Allocates the memory for the list.
// After the use it has to be freed using freeCodeBlockList().
//    
//********************************************************************
int32_t readCodeBlockList(char * blockListName)
{
    //Read the list of blocks to be processed
    FILE *fid;
    char *pb;
    int32_t i;
    char buffer[300];


    if( (fid = fopen(blockListName, "r")) == NULL)
    {
        printf("Code block list file %s is not found!",blockListName);
        return 0;
    }

    printf("*********************************************************\n");
    printf("Test : %s \n", blockListName);
    printf("*********************************************************\n");



    //See how many config file names are in the list file
    numCodeBlocks = 0;
    while(fgets(buffer,300,fid) != NULL) 
    {
        if(buffer[0] != '#')
        {
            if((pb = strtok(buffer, " \t=#\n\r")) != NULL)
            {
                numCodeBlocks ++;
            }
        }
    }
    fclose(fid);

    codeBlockList = (char **) malloc(sizeof(char *) * numCodeBlocks);

    fid = fopen(blockListName, "r");
    i=0;
    while(fgets(buffer,300,fid) != NULL) 
    {
        if(buffer[0] != '#')
        {
            pb = strtok(buffer, " \t=#\n\r");
            if(pb != NULL)
            {
                codeBlockList[i] = (char *) malloc(sizeof(char) * (strlen(pb)+1));
                strcpy(codeBlockList[i], pb);
                i++;
            }
        }
    }
    fclose(fid);

    codeBlockCntr = 0;

    return 1;
}

//********************************************************************
//********************************************************************
void freeCodeBlockList(void)
{
    int32_t i;

    for(i=0; i<numCodeBlocks; i++)
    {
        free(codeBlockList[i]);
    }
    free(codeBlockList);

}




//********************************************************************
//********************************************************************
// 1. Read configuration file config.cfg into sparms structure
// 2. Fill reg structure based on sparms structure
// 3. Run simulator
//********************************************************************
//********************************************************************
int32_t get_code_block(char *cfgFileName, int32_t Cntr)
{
    char buffer[301];
    FILE *fidSrc;
    FILE *fidDst;
    char *pb,*pt;
    uint32_t itemp;

    if((fidSrc = fopen(cfgFileName, "r")) == NULL)
    {
        printf("\nConfig file %s named in list is not found!\n", cfgFileName);
        return 0;
    }
    if((fidDst = fopen("simulator\\config.cfg", "w")) == NULL)
    {
        printf("\nCan not open config.cfg file!\n");
        return 0;
    }

    //Copy config file from the list to config.cfg
    while(fgets(buffer,300,fidSrc) != NULL) 
    {
        if(strstr(buffer,"c_model_seed") != NULL) 
        {
            pb = strtok(buffer, " \t=#");  /* position to parameter descriptor */
            pt = strtok(NULL, " \t=#");    /* position to its value */
            sscanf(pt, "%u", &itemp);
            itemp += 10000*Cntr;
            sprintf(buffer, "c_model_seed = %u\n", itemp);
        }
        fprintf(fidDst,"%s",buffer);
    }
    fclose(fidSrc);
    fclose(fidDst);

    //Read config.cfg file
    LoadConfig("simulator\\config.cfg", &sparms);

    //Fill register structure based on sparmas
    SetReg(&sparms, &reg);
    //Copy from register structure to output structure only relevant data
    SetOutReg(&reg, &outReg);

    //Run Simulator according to config.cfg
    SetCurrentDirectory("simulator\\debug");
    system("Test_c_model.exe");
    SetCurrentDirectory("..\\..\\");

    return 1; 
}


//********************************************************************
//********************************************************************
// 1. Create output file block#_cfgreg.dat in the folderName with 
//    outReg parameters 
// 2. Copy llrs, tail llrs, hard decisions, soft decisions and status 
//    from simulator\debug floder to folderName.
//********************************************************************
//********************************************************************
int32_t save_code_block_files(char *folderName, int32_t cntr)
{
    FILE *fid;
    char fileName[300];
    char copyCmd[500];
    int32_t *regPtr;
    int32_t i;

    sprintf(fileName, "%s\\block%d_cfgreg.dat",folderName, cntr);

    fid = fopen(fileName, "w");
    regPtr = (int32_t *) &outReg;
    for(i=0; i<sizeof(outReg)/4; i++)
    {
        fprintf(fid,"%d\n", regPtr[i]);
    }
    fclose(fid);

    //Input LLRs 
    sprintf(fileName, "%s\\block%d_llrs.bin", folderName, cntr);
    conv2bin("simulator\\debug\\test_rx_llrs.dat", fileName, 0);

    //tail LLRs 
    sprintf(copyCmd, "copy simulator\\debug\\test_rx_tail_llrs.dat %s\\block%d_tail_llrs.dat", folderName, cntr);
    system(copyCmd);

    //Interleaver
    //sprintf(copyCmd, "copy simulator\\debug\\test_intlv_tx.dat %s\\block%d_intlv.dat", folderName, cntr);
    //system(copyCmd);

    //Hard decisions
    sprintf(fileName, "%s\\block%d_hard_dec.bin", folderName, cntr);
    conv2bin("simulator\\debug\\test_final_hard_out_info.dat", fileName, 1);

    //Load soft decisions decisions
    if(reg.soft_out_flag_en)
    {
        sprintf(fileName, "%s\\block%d_soft_dec.bin", folderName, cntr);
        conv2bin("simulator\\debug\\test_final_soft_out_info.dat", fileName, 0);
    }

    //Load status bits
    if(reg.out_flag_en)
    {
        sprintf(fileName, "%s\\block%d_status.bin", folderName, cntr);
        conv2bin("simulator\\debug\\test_out_status.dat", fileName, 1);
    }

    //printf("outFileType = %s\n",outFileType);
    //if ( outFileType == "BOTH" )
    if ( !strcmp(outFileType,"BOTH") )
    {
        //Input LLRs 
        sprintf(copyCmd, "copy simulator\\debug\\test_rx_llrs.dat %s\\block%d_llrs.dat", folderName, cntr);
        system(copyCmd);

        //Hard decisions
        sprintf(copyCmd, "copy simulator\\debug\\test_final_hard_out_info.dat %s\\block%d_hard_dec.dat", folderName, cntr);
        system(copyCmd);

        //Load soft decisions decisions
        if(reg.soft_out_flag_en)
        {
            sprintf(copyCmd, "copy simulator\\debug\\test_final_soft_out_info.dat %s\\block%d_soft_dec.dat", folderName, cntr);
            system(copyCmd);
        }

        //Load status bits
        if(reg.out_flag_en)
        {
            sprintf(copyCmd, "copy simulator\\debug\\test_out_status.dat %s\\block%d_status.dat", folderName, cntr);
            system(copyCmd);
        }
    }

    //Create reference folder
    sprintf(copyCmd, "%s\\reference", folderName);
    mkdir(copyCmd);

    //Get original encoded bits generated by tcp3 simulator 
    sprintf(copyCmd, "copy simulator\\debug\\test_inp_cfg.dat %s\\reference\\block%d_inp_cfg.dat", folderName, cntr);
    system(copyCmd);


    //Get original info bits generated by tcp3 simulator
    sprintf(copyCmd, "copy simulator\\infobits_file.txt %s\\reference\\block%d_info_bits.dat", folderName, cntr);
    system(copyCmd);
    
    //Get original encoded bits generated by tcp3 simulator
    sprintf(copyCmd, "copy simulator\\codedbits_file.txt %s\\reference\\block%d_coded_bits.dat", folderName, cntr);
    system(copyCmd);


    return 1; 
}






