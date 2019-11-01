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
#include <stdlib.h>
#include <string.h>

//#define DEBUG_CONV2BIN  0 - define in the compile options for prints

#define DATA_LLRS  0
#define DATA_BITS  1

//Converts input formated data file to output binary file
//Two type of input data files:
// 1. LLR data file - file contains three columns of LLRs: systematic, parity0, parity1 LLRs ar signed 8-bit numbers
// 2. Bits file - file with bits packed in 32-bit words in hex format
//Output file is binary file
//LLR data are stored in 32-bit words in three LSB bytes as [Byte3, Byte2,Byte1, Byte0] = [0, Sys, Par0, Par1] 
void conv2bin(char *infile, char *outfile, int type)
{
    
    FILE *fidIn, *fidOut;
    char buffer[300];
    unsigned int val;
    int cnt, LLRLen;
    int sys, par0, par1;
    char *sysPtr, *par0Ptr, *par1Ptr;

    //Open input file, either bits in hex format, or 8-bit LLRs in 3 column integer format 
    if( (fidIn = fopen(infile,"r")) == NULL)
    {
       printf("File %s is not found!\n", infile);
       return;
    }

    //Get the LLRs size
    LLRLen = 0;
    if(type == DATA_LLRS)
    {
        while(fgets(buffer,300,fidIn) != NULL)
        {
            LLRLen++;
        }
    }
    fclose(fidIn);

    //Open input file
    fidIn = fopen(infile, "r");

    //Open output binary file
    fidOut = fopen(outfile, "wb");

    //Read from input and write to output file
    if(type == DATA_BITS)
    {
        cnt=0;
        while(fgets(buffer,300,fidIn) != NULL)     
        {
            sscanf(buffer, "%x", &val);
            fwrite(&val, sizeof(int), 1, fidOut);
            cnt++;
        }
#ifdef DEBUG_CONV2BIN
        printf("Info bits file, converted %d lines!\n", cnt);
#endif
    }
    else
    {
        cnt=0;
        sysPtr = malloc(LLRLen);        
        par0Ptr = malloc(LLRLen);        
        par1Ptr = malloc(LLRLen);        
        while(fgets(buffer,300,fidIn) != NULL)     
        {
            sscanf(buffer, "%d %d %d", &sys, &par0, &par1);
            sysPtr[cnt] = sys;
            par0Ptr[cnt] = par0;
            par1Ptr[cnt] = par1;
            cnt++;
        }
        fwrite(sysPtr, sizeof(char), LLRLen, fidOut);
        fwrite(par0Ptr, sizeof(char), LLRLen, fidOut);
        fwrite(par1Ptr, sizeof(char), LLRLen, fidOut);
#ifdef DEBUG_CONV2BIN
        printf("LLR data file, converted %d lines!\n", cnt);
#endif
    }
    fclose(fidIn);
    fclose(fidOut);
}