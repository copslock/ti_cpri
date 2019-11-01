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
#include <string.h>

#include "tcp3d_main.h"

#define READ_BINARY_TEST_VECTORS    1 /* Read .bin test vector files */

UInt32                  morePrints = 0;
extern Char             *strMode[4];
extern Char             testvectFolderBase[];

/*******************************************************************************
 ******************************************************************************/
/**
 * @brief   Bit reverse in 32-bit word
 */
UInt32 bitr(UInt32 src)
{
    UInt32 a,c;
    UInt32 i;
    UInt32 sa;

    a = src;
    c = 0;
    for (i = 0,sa=31; i < 16; i++,sa-=2) {
      c |= (a & (0x1 << i)) << sa;
    }
    for (i = 16,sa=1; i < 32; i++,sa+=2) {
      c |= (a & (0x1 << i)) >> (sa);
    }
    return c;
}

/*******************************************************************************
 ******************************************************************************/
/**
 * @brief   Swap bytes in 32-bit word
 */
UInt32 swapBytes(UInt32 src)
{
    UInt32 a,c;

    a = src;
    c = (a & 0x000000ff) << 24;
    c |= (a & 0x0000ff00) << 8;
    c |= (a & 0x00ff0000) >> 8;
    c |= (a & 0xff000000) >> 24;
    return c;
}

/*******************************************************************************
 ******************************************************************************/
Int getTestSetCB(IHeap_Handle dataHeap, cbTestDesc *cbTestSet, Char *testFolder)
{
    cbDataDesc      *cbPtr;
    cbConfig        tempCbConfig;
    Int             i, cbCnt;
    Int32           numBytes;

    Char            fileName[300];
    FILE            *fid;
    Int             cbCfgStrSize=(sizeof(cbConfig)>>2);
    Int32           *tmp32 = (Int32 *) &tempCbConfig;
                            /* Note: Temporary pointer used for writing
                                into the tempCbConfig Structure memory */
    Int32           stmp[3];
#if !READ_BINARY_TEST_VECTORS
    Int8            *syspar;
    UInt32          inSize;
#endif
    UInt32          tmp;

    /* Get number of blocks */
    strcpy(fileName, testFolder);
    strcat(fileName, "\\number_of_blocks.dat");
    if(!(fid = fopen(fileName,"r")))
    {
        System_printf("\t!!! Error in Number of blocks file : %s !!!\n", fileName);
        System_printf("\n***********************************************************\n");
        System_printf(" It means the test vectors are not available in the folder: \n\t %s \n\n", testFolder);
        System_printf(" Do the following: \n");
        System_printf(" 1) Check the variable named \"testvectFolderBase\" in the corresponding main.c file. \n");
        System_printf("    a) It is set to \"%s\" \n", testvectFolderBase);
        System_printf("    b) Make sure the \"testvectFolderBase\" variable is set to correct folder for testvectors. \n");
        System_printf(" 2) For test project, make sure to run the batch file before running tests: \n");
        System_printf("     %s \n", "\\tcp3d\\test\\gen_test_vectors\\genTestVect.bat");
        System_printf("***********************************************************\n");
        System_exit(0);
    }
    cbCnt = 0;
    fscanf(fid, "%d", &cbTestSet->maxNumCB);
    fclose(fid);

    /* Allocate memory for code blocks */
    numBytes = cbTestSet->maxNumCB * sizeof(cbDataDesc *);
    cbTestSet->cbData = (cbDataDesc **) Memory_alloc(dataHeap, numBytes, 64, NULL);       

    /* Load code blocks */
    for(cbCnt=0; cbCnt < cbTestSet->maxNumCB; cbCnt++)
    {
        /* Allocate memory for code block set and update the local pointer */
        cbTestSet->cbData[cbCnt] = (cbDataDesc *) Memory_alloc ( dataHeap,
                                                sizeof(cbDataDesc),
                                                64,
                                                NULL);
        cbPtr = cbTestSet->cbData[cbCnt];
        if ( cbPtr == NULL )
        {
            System_printf("\tMemory allocation failed for Code Block Set %d!\n", cbCnt);
        }

        /* Open the block parameters file */
        sprintf(fileName, "%s\\block%d_cfgreg.dat", testFolder, cbCnt);
        if ( !(fid = fopen(fileName,"r")) )
        {
            System_printf("\tConfig file open failed : %s\n", fileName);
            System_exit(0);
        }
        /* Load block parameters */
        for(i=0;i<cbCfgStrSize;i++)
        {
            fscanf(fid, "%d", &stmp[0]);
            tmp32[i] = stmp[0]; /* tmp32 points to tempCbConfig address */ 
        }
        fclose(fid);

        /* Set the Code Block specific values from Config Read */
        cbPtr->blockSize     = tempCbConfig.NumInfoBits;
        cbPtr->mode          = tempCbConfig.mode_sel;
        cbPtr->crcInitVal    = tempCbConfig.lte_crc_init_sel;
        cbPtr->sw0LengthUsed = tempCbConfig.SW0_length;

        /* Set the code block set value with the first one block config */
        if (cbCnt == 0)
        {
            cbTestSet->mode       = cbPtr->mode;
            cbTestSet->lteCrcSel  = cbPtr->crcInitVal;
        }

        /* Check if the mode is different from the first one in the group */
        /* Force the mode value to be same as first one */
        if ( cbPtr->mode != cbTestSet->mode )
        {
            if ( !((cbPtr->mode+cbTestSet->mode) % 3) )
            {
                System_printf("\tBlock = %d, Mode changed from %s to %s\n", cbCnt,
                                                    strMode[cbPtr->mode],
                                                    strMode[cbTestSet->mode]);
                cbPtr->mode = cbTestSet->mode;
            }
            else
            {
                System_exit(0);
            }
        }

        /* Check for LTE CRC Init Value Change */
        if ( ( cbPtr->mode == TEST_MODE_LTE ) &&
             ( cbPtr->crcInitVal != cbTestSet->lteCrcSel ) )
        {
            System_printf("\tLTE CRC Initial Value is different\n");
            System_printf("\tSet Value is %d\n", cbTestSet->lteCrcSel);
            System_exit(0);
        }

        /* Allocate Memory for Input LLR data */
        if ( cbPtr->mode == TEST_MODE_SPLIT )
        {   /* 3GPP - Split Mode */
            cbPtr->llrOffset   = COMPUTE_KOUT(cbPtr->blockSize);
        }
        else
        {
            cbPtr->llrOffset   = cbPtr->blockSize;
        }
        cbPtr->sizeLLR = 3 * cbPtr->llrOffset; /* three streams (sys, par0, par1) */
        cbPtr->inLLR = (Int8 *) Memory_alloc(dataHeap, cbPtr->sizeLLR, 64, NULL);
        if(cbPtr->inLLR == NULL)
        {
            System_printf("\tMemory allocation failed !!! (LLR)\n");
            System_exit(0);
        }
    
        /* Prepare Input LLR streams */
        /* This mode value is set with the value read from cfgreg file */
        if ( cbPtr->mode == TEST_MODE_SPLIT ) /* 3GPP - split mode */
        {
#if READ_BINARY_TEST_VECTORS
            /* Fill syspar with data bits */
            sprintf(fileName, "%s\\block%d_llrs.bin", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"rb")) )
            {
                System_printf("\tLLR Data file read failed : %s\n", fileName);
                System_exit(0);
            }
            fread(cbPtr->inLLR, 1, cbPtr->llrOffset, fid);
            fread(&cbPtr->inLLR[cbPtr->llrOffset], 1, cbPtr->llrOffset, fid);
            fread(&cbPtr->inLLR[cbPtr->llrOffset<<1], 1, cbPtr->llrOffset, fid);
            fclose(fid);

            /* Fill syspar with tail bits */
            sprintf(fileName, "%s\\block%d_tail_llrs.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tTail Bits file read failed : %s\n", fileName);
                System_exit(0);
            }
            for (i = 0; i < 6; i++)
            {
                fscanf(fid, "%d\t%d", &stmp[0], &stmp[1]);
                cbPtr->tailBits[i] = (Int8) stmp[0];
                cbPtr->tailBits[6+i] = (Int8) stmp[1];
            }
            fclose(fid);
#else // READ_BINARY_TEST_VECTORS
            /* set loop count for LLR file read */
            inSize  = cbPtr->blockSize*3;

            /* Allocate memory for syspar temp Buffer for WCDMA case 
            syspar = (Int8 *) Memory_alloc(dataHeap, inSize+12, 0, NULL);       
            if(syspar == NULL)
            {
                System_printf("\tMemory allocation failed !!! (SysPar)\n");
                System_exit(0);
            }*/

            /* Fill syspar with data bits */
            sprintf(fileName, "%s\\block%d_llrs.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tLLR Data file read failed : %s\n", fileName);
                System_exit(0);
            }
            for(i=0;i<inSize;i+=3)
            {
                fscanf(fid, "%d\t%d\t%d", &stmp[0], &stmp[1],&stmp[2]);
                //syspar[i]   = (Int8) stmp[0];
                //syspar[i+1] = (Int8) stmp[1];
                //syspar[i+2] = (Int8) stmp[2];
                cbPtr->inLLR[i]                     = (Int8) stmp[0];
                cbPtr->inLLR[i+cbPtr->llrOffset]    = (Int8) stmp[1];
                cbPtr->inLLR[i+cbPtr->llrOffset<<1] = (Int8) stmp[2];
            }
            fclose(fid);
        
            /* Fill syspar with tail bits */
            sprintf(fileName, "%s\\block%d_tail_llrs.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tTail Bits file read failed : %s\n", fileName);
                System_exit(0);
            }
            for (i = 0; i < 6; i++)
            {
                fscanf(fid, "%d\t%d", &stmp[0], &stmp[1]);
                //syspar[inSize+i]    = (Int8) stmp[0];
                //syspar[inSize+6+i]  = (Int8) stmp[1];
                cbPtr->tailBits[i]      = (Int8) stmp[0];
                cbPtr->tailBits[6+i]    = (Int8) stmp[1];
            }
            fclose(fid);
        
            /* Prepare the sys, par1, par2 streams 
            TCP3D_WCDMA_dataPrep (  syspar,
                                    cbPtr->blockSize,
                                    NULL, // No sign change
                                    cbPtr->inLLR,
                                    cbPtr->inLLR+cbPtr->llrOffset,
                                    cbPtr->inLLR+2*cbPtr->llrOffset,
                                    betaMap0,
                                    betaMap1);*/
        
            /* Free syspar Memory 
            Memory_free(dataHeap, syspar, inSize+12);*/        
#endif // READ_BINARY_TEST_VECTORS
        }
        else if ( cbPtr->mode == TEST_MODE_LTE )  /* LTE mode */
        {
            /* Fill syspar with data bits */
#if READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_llrs.bin", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"rb")) )
            {
                System_printf("\tLLR Data file read failed : %s\n", fileName);
                System_exit(0);
            }
            fread(cbPtr->inLLR, 1, cbPtr->blockSize, fid);
            fread(&cbPtr->inLLR[cbPtr->llrOffset], 1, cbPtr->blockSize, fid);
            fread(&cbPtr->inLLR[cbPtr->llrOffset<<1], 1, cbPtr->blockSize, fid);
            fclose(fid);
#else // READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_llrs.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tLLR Data file read failed : %s\n", fileName);
                System_exit(0);
            }
            for(i=0;i<cbPtr->blockSize;i++)
            {
                fscanf(fid, "%d\t%d\t%d", &stmp[0], &stmp[1],&stmp[2]);
                cbPtr->inLLR[i]                      = (Int8) stmp[0];
                cbPtr->inLLR[i+cbPtr->llrOffset]  = (Int8) stmp[1];
                cbPtr->inLLR[i+2*cbPtr->llrOffset]= (Int8) stmp[2];
            }
            fclose(fid);
#endif // READ_BINARY_TEST_VECTORS
        
            /* Fill syspar with tail bits */
            sprintf(fileName, "%s\\block%d_tail_llrs.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tTail Bits file read failed : %s\n", fileName);
                System_exit(0);
            }
            for (i = 0; i < 6; i++)
            {
                fscanf(fid, "%d\t%d", &stmp[0], &stmp[1]);
                cbPtr->tailBits[i] = (Int8) stmp[0];
                cbPtr->tailBits[6+i] = (Int8) stmp[1];
            }
            fclose(fid);
        }
        else if ( cbPtr->mode == TEST_MODE_WIMAX )  /* WIMAX mode */
        {
            /* Fill syspar with data bits */
#if READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_llrs.bin", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"rb")) )
            {
                System_printf("\tLLR Data file read failed : %s\n", fileName);
                System_exit(0);
            }
            fread(cbPtr->inLLR, 1, cbPtr->blockSize, fid);
            fread(&cbPtr->inLLR[cbPtr->llrOffset], 1, cbPtr->blockSize, fid);
            fread(&cbPtr->inLLR[cbPtr->llrOffset<<1], 1, cbPtr->blockSize, fid);
            fclose(fid);
#else // READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_llrs.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tLLR Data file read failed : %s\n", fileName);
                System_exit(0);
            }
            for(i=0;i<cbPtr->blockSize;i++)
            {
                fscanf(fid, "%d\t%d\t%d", &stmp[0], &stmp[1],&stmp[2]);
                cbPtr->inLLR[i]                      = (Int8) stmp[0];
                cbPtr->inLLR[i+cbPtr->llrOffset]  = (Int8) stmp[1];
                cbPtr->inLLR[i+2*cbPtr->llrOffset]= (Int8) stmp[2];
            }
            fclose(fid);
#endif // READ_BINARY_TEST_VECTORS
        }

        /* Allocate Memory for Ouput & Reference Hard Decisions */
        cbPtr->sizeHD = COMPUTE_HD_BYTE_SIZE(cbPtr->blockSize);
        cbPtr->outHD = (UInt32 *) Memory_calloc(dataHeap,
                                                    cbPtr->sizeHD,
                                                    64,
                                                    NULL);
        if(cbPtr->outHD == NULL)
        {
            System_printf("\tMemory allocation failed !!! (Out HD)\n");
            System_exit(0);
        }
        cbPtr->refHD = (UInt32 *) Memory_calloc(dataHeap,
                                                    cbPtr->sizeHD,
                                                    64,
                                                    NULL);
        if(cbPtr->refHD == NULL)
        {
            System_printf("\tMemory allocation failed !!! (Ref HD)\n");
            System_exit(0);
        }
    
        /* Prepare Reference Hard Decisions */
#if READ_BINARY_TEST_VECTORS
        sprintf(fileName, "%s\\block%d_hard_dec.bin", testFolder, cbCnt);
        if ( !(fid = fopen(fileName,"rb")) )
        {
            System_printf("\tHard Decision File open failed : %s\n", fileName);
            System_exit(0);
        }
        fread(cbPtr->refHD, 4, (cbPtr->sizeHD>>2), fid);
        
        for(i=0;i<(cbPtr->sizeHD>>2);i++)
        {
#ifdef _BIG_ENDIAN
            tmp = swapBytes(cbPtr->refHD[i]);
            if (!tempCbConfig.out_order_sel)
            {
                cbPtr->refHD[i] = bitr(tmp);
            }
            else
            {
                cbPtr->refHD[i] = tmp;
            }
#else
            tmp = cbPtr->refHD[i];
            if (!tempCbConfig.out_order_sel)
            {
                cbPtr->refHD[i] = tmp;
            }
            else
            {
                cbPtr->refHD[i] = bitr(tmp);
            }
#endif
        }
        fclose(fid);
#else // READ_BINARY_TEST_VECTORS
        sprintf(fileName, "%s\\block%d_hard_dec.dat", testFolder, cbCnt);
        if ( !(fid = fopen(fileName,"r")) )
        {
            System_printf("\tHard Decision File open failed : %s\n", fileName);
            System_exit(0);
        }
        for(i=0;i<(cbPtr->sizeHD>>2);i++)
        {
            fscanf(fid, "%x", &tmp);
#ifdef _BIG_ENDIAN
            if (!tempCbConfig.out_order_sel)
            {
                cbPtr->refHD[i] = bitr(tmp);
            }
            else
            {
                cbPtr->refHD[i] = tmp;
            }
#else
            if (!tempCbConfig.out_order_sel)
            {
                cbPtr->refHD[i] = tmp;
            }
            else
            {
                cbPtr->refHD[i] = bitr(tmp);
            }
#endif
        }
        fclose(fid);
#endif // READ_BINARY_TEST_VECTORS

        /* Allocate Memory for Interleaver Table */
        /* Interleaver flag is cleared - no external interleaver table */
        cbPtr->interFlag = 0;
        cbPtr->sizeINTER = 0;
        cbPtr->inInter = NULL;

        /* Allocate Memory for Ouput & Reference Soft Decisions */
        cbPtr->sdFlag = 0;
        cbPtr->sizeSD = 0;
        cbPtr->sdOffset = 0;
        cbPtr->outSD = NULL;
        cbPtr->refSD = NULL;
//        tempCbConfig.soft_out_flag_en = 0;
        if ( tempCbConfig.soft_out_flag_en )
        {
            cbPtr->sdFlag = 1;

            if ( cbPtr->mode == TEST_MODE_SPLIT )  /* SPLIT MODE */
            {
                cbPtr->sizeSD      = cbPtr->blockSize;
                cbPtr->sdOffset    = NULL;
            }
            else
            {
                cbPtr->sizeSD      = 3 * cbPtr->blockSize;
                cbPtr->sdOffset    = cbPtr->blockSize;
            }
            cbPtr->outSD = (Int8 *) Memory_calloc(dataHeap, cbPtr->sizeSD, 64, NULL);
            if(cbPtr->outSD == NULL)
            {
                System_printf("\tMemory allocation failed !!! (Out SD)\n");
                System_exit(0);
            }
            cbPtr->refSD = (Int8 *) Memory_calloc(dataHeap, cbPtr->sizeSD, 64, NULL);
            if(cbPtr->refSD == NULL)
            {
                System_printf("\tMemory allocation failed !!! (Ref SD)\n");
                System_exit(0);
            }
        
            /* Prepare Reference Soft Decisions */
#if READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_soft_dec.bin", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"rb")) )
            {
                System_printf("\tSoft Decision File open failed : %s\n", fileName);
                System_exit(0);
            }
            fread(cbPtr->refSD, 1, cbPtr->blockSize, fid);
            if ( cbPtr->sdOffset )
            {
                fread(&cbPtr->refSD[cbPtr->sdOffset], 1, cbPtr->blockSize, fid);
                fread(&cbPtr->refSD[cbPtr->sdOffset<<1], 1, cbPtr->blockSize, fid);
            }
            fclose(fid);
#else // READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_soft_dec.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tSoft Decision File open failed : %s\n", fileName);
                System_exit(0);
            }
            for (i = 0; i < cbPtr->blockSize; i++)
            {
                fscanf(fid, "%d\t%d\t%d", &stmp[0], &stmp[1], &stmp[2]);
                cbPtr->refSD[i]          = (Int8) stmp[0];
                if (cbPtr->sdOffset)
                {
                    cbPtr->refSD[i+cbPtr->sdOffset]   = (Int8) stmp[1];
                    cbPtr->refSD[i+2*cbPtr->sdOffset] = (Int8) stmp[2];
                }
            }
            fclose(fid);
#endif // READ_BINARY_TEST_VECTORS
        }

        /* Allocate Memory for Ouput & Reference Status Registers */
        cbPtr->stsFlag = 0;
        cbPtr->sizeSTS = 0;
        cbPtr->outSts = NULL;
        cbPtr->refSts = NULL;
        if ( tempCbConfig.out_flag_en )
        {
            cbPtr->stsFlag = 1;
            /* allocate outSts memory */
            cbPtr->sizeSTS = 3 * sizeof(UInt32);
            cbPtr->outSts = (UInt32 *) Memory_calloc(dataHeap, cbPtr->sizeSTS, 64, NULL);
            if(cbPtr->outSts == NULL)
            {
                System_printf("\tMemory allocation failed !!! (Out STS)\n");
                System_exit(0);
            }
            cbPtr->refSts = (UInt32 *) Memory_calloc(dataHeap, cbPtr->sizeSTS, 64, NULL);
            if(cbPtr->refSts == NULL)
            {
                System_printf("\tMemory allocation failed !!! (Ref STS)\n");
                System_exit(0);
            }
    
            /* Prepare Reference Status Registers */
#if READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_status.bin", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"rb")) )
            {
                System_printf("\tStatus File open failed : %s\n", fileName);
                System_exit(0);
            }
            fread(cbPtr->refSts, sizeof(UInt32), 3, fid);
#ifdef _BIG_ENDIAN
            cbPtr->refSts[0] = swapBytes(cbPtr->refSts[0]);
            cbPtr->refSts[1] = swapBytes(cbPtr->refSts[1]);
            cbPtr->refSts[2] = swapBytes(cbPtr->refSts[2]);
#endif
            fclose(fid);
#else // READ_BINARY_TEST_VECTORS
            sprintf(fileName, "%s\\block%d_status.dat", testFolder, cbCnt);
            if ( !(fid = fopen(fileName,"r")) )
            {
                System_printf("\tStatus File open failed : %s\n", fileName);
                System_exit(0);
            }
            for(i=0;i<3;i++)
            {
                fscanf(fid, "%x", &tmp);
                cbPtr->refSts[i] = (UInt32) tmp;
            }
            fclose(fid);
#endif // READ_BINARY_TEST_VECTORS
        }
//        else
//            tempCbConfig.out_flag_en = 1;

        /* Allocate Memory for Input Config Registers */
        cbPtr->sizeCFG = 15 * sizeof(UInt32);
        cbPtr->inCfg = (UInt32 *) Memory_calloc(dataHeap, cbPtr->sizeCFG, 64, NULL);
        if(cbPtr->inCfg == NULL)
        {
            System_printf("\tMemory allocation failed !!! (CFG)\n");
            System_exit(0);
        }

        /* Allocate Memory for Tcp3d_InCfgParams structure */
        cbPtr->inCfgParams = (Tcp3d_InCfgParams *) Memory_alloc(dataHeap,
                                                sizeof(Tcp3d_InCfgParams),
                                                64,
                                                NULL);
        if (cbPtr->inCfgParams == NULL)
        {
            System_printf("Memory allocation failed !!! (inCfgParams)\n");
            System_exit(0);
        }
        /* Update the input config params structure */
        fillICParams(cbPtr->inCfgParams, &tempCbConfig);

        if ( (morePrints) && ((cbCnt+1) % 5) == 0 )
            System_printf("\tCode block prepared : %d \n", cbCnt+1);

    } /* for(cbCnt=0; cbCnt < cbTestSet->maxNumCB; cbCnt++) */

    /* Set the double buffer value based on mode value */
    if ( ( cbTestSet->mode == TEST_MODE_SINGLE ) || ( cbTestSet->mode == TEST_MODE_SPLIT ) ) 
        cbTestSet->doubleBuffer = 0; //CSL_TCP3D_CFG_TCP3_MODE_IN_MEM_DB_EN_DISABLE
    else
        cbTestSet->doubleBuffer = 1; //CSL_TCP3D_CFG_TCP3_MODE_IN_MEM_DB_EN_ENABLE

    return (cbCnt);
}

/*******************************************************************************
 ******************************************************************************/
Void freeTestSetCB(IHeap_Handle dataHeap, cbTestDesc *cbTestSet)
{
    Int32           i;
    
    /* Free memory allocated for Code Block sets */
    for(i=0; i< cbTestSet->maxNumCB; i++)
    {
        Memory_free(dataHeap, cbTestSet->cbData[i]->inCfg, cbTestSet->cbData[i]->sizeCFG);                
        Memory_free(dataHeap, cbTestSet->cbData[i]->inLLR, cbTestSet->cbData[i]->sizeLLR);                
        Memory_free(dataHeap, cbTestSet->cbData[i]->outHD, cbTestSet->cbData[i]->sizeHD);                
        Memory_free(dataHeap, cbTestSet->cbData[i]->refHD, cbTestSet->cbData[i]->sizeHD);                
        if ( cbTestSet->cbData[i]->sdFlag )
        {
            Memory_free(dataHeap, cbTestSet->cbData[i]->outSD, cbTestSet->cbData[i]->sizeSD);                
            Memory_free(dataHeap, cbTestSet->cbData[i]->refSD, cbTestSet->cbData[i]->sizeSD);
        }
        if ( cbTestSet->cbData[i]->stsFlag )
        {
            Memory_free(dataHeap, cbTestSet->cbData[i]->outSts, cbTestSet->cbData[i]->sizeSTS);                
            Memory_free(dataHeap, cbTestSet->cbData[i]->refSts, cbTestSet->cbData[i]->sizeSTS);
        }
        if ( cbTestSet->cbData[i]->interFlag )
        {
            Memory_free(dataHeap, cbTestSet->cbData[i]->inInter, cbTestSet->cbData[i]->sizeINTER);
        }
        Memory_free(dataHeap, cbTestSet->cbData[i]->inCfgParams, sizeof(Tcp3d_InCfgParams));        
        Memory_free(dataHeap, cbTestSet->cbData[i], sizeof(cbDataDesc));        
    }
    Memory_free(dataHeap, cbTestSet->cbData, cbTestSet->maxNumCB * sizeof(cbDataDesc *));
}

/* end of file */
