/*
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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
/**********************************************************************************
 * FILE PURPOSE: General purpose CCS utility functions
 **********************************************************************************
 * FILE NAME: ccsutil.c
 *
 * DESCRIPTION: Common functions which make use of ccs files
 *
 **********************************************************************************/
#include <stdio.h>
#include <malloc.h>
#include "ccsutil.h"


/**********************************************************************************
 * FUNCTION PURPOSE: Read a CCS data file
 **********************************************************************************
 * DESCRIPTION: An array is allocated and the CCS data file is read
 **********************************************************************************/
unsigned int *readCcsFile (FILE *str, int *nwords, unsigned int *addr)
{
    int a, b, c, d, e, i;
    unsigned int *cf;
    char iline[132];

    fgets (iline, 132, str);
    sscanf (iline, "%x %x %x %x %x", &a, &b, addr, &d, &e);

    cf = malloc (e * sizeof (unsigned int));
    if (cf == NULL)  {
        *nwords = -1;
        return (NULL);
    }

    for (i = 0; i < e; i++)  {
      fgets (iline, 132, str);
      sscanf (&iline[2], "%x", &cf[i]);
    }

    *nwords = e;
    return (cf);

}


/****************************************************************************************
 * FUNCTION PURPOSE: Write a CCS data file
 ****************************************************************************************
 * DESCRIPTION: Data in the the array is written out
 ****************************************************************************************/
int writeCcsFile (FILE *str, unsigned int *data, int nwords, unsigned int addr)
{
   int i;

   fprintf (str, "1651 1 %x 1 %x\n", addr, nwords);

   for (i = 0; i < nwords; i++)
    fprintf (str, "0x%08x\n", data[i]);

   return (0);

}
    

/****************************************************************************************
 * FUNCTION PURPOSE: Write a CCS data file in upper case
 ****************************************************************************************
 * DESCRIPTION: Data in the the array is written out in upper case
 ****************************************************************************************/
int writeCcsFileUp (FILE *str, unsigned int *data, int nwords, unsigned int addr)
{
   int i;

   fprintf (str, "1651 1 %X 1 %X\n", addr, nwords);

   for (i = 0; i < nwords; i++)
    fprintf (str, "0x%08X\n", data[i]);

   return (0);

}


/******************************************************************************************
 * FUNCTION PURPOSE: Endian swap a data array
 ******************************************************************************************
 * DESCRIPTION: All the values in the array are endian swapped
 ******************************************************************************************/
void ccsEndianSwap (unsigned int *data, int nwords)
{
    int i;
    unsigned int v;

    for (i = 0; i < nwords; i++)  {

        v = (((data[i] >> 24) & 0xff) <<  0)   |
            (((data[i] >> 16) & 0xff) <<  8)   |
            (((data[i] >>  8) & 0xff) << 16)   |
            (((data[i] >>  0) & 0xff) << 24)   ;

        data[i] = v;
    }

}

/******************************************************************************************
 * FUNCTION PURPOSE: Perform a 16 bit swap on data
 ******************************************************************************************
 * DESCRIPTION: Data is 16 bit swapped
 ******************************************************************************************/
void ccsSwap16 (unsigned int *data, int nwords)
{
    int i;
    unsigned int v;

    for (i = 0; i < nwords; i++)  {

        v = (((data[i] >> 24) & 0xff) <<  8)   |
            (((data[i] >> 16) & 0xff) <<  0)   |
            (((data[i] >>  8) & 0xff) << 24)   |
            (((data[i] >>  0) & 0xff) << 16)   ;

        data[i] = v;
    }

}

    


/******************************************************************************************
 * FUNCTION PURPOSE: Open a file, read the CCS data, close the file
 ******************************************************************************************
 * DESCRIPTION: Reads a CCS data file
 ******************************************************************************************/
unsigned int *openReadCloseCcsFile (char *fname, int *size, unsigned int *addr)
{
    unsigned int *dat;

    FILE *str;

    if (fname == NULL)
        str = stdin;
    else
        str = fopen (fname, "r");

    if (str == NULL)  {
        *size = -1;
        return (NULL);
    }


    dat = readCcsFile (str, size, addr);

    if (fname != NULL)
        fclose (str);

    return (dat);

}


/******************************************************************************************
 * FUNCTION PURPOSE: Open a file, write CCS data, close the file
 ******************************************************************************************
 * DESCRIPTION: Writes a CCS data file
 ******************************************************************************************/
int openWriteCloseCcsFile (char *fname, unsigned int *data, int nwords, unsigned int addr)
{
    FILE *str;

    if (fname == NULL)
        str = stdout;
    else
        str = fopen (fname, "w");

    if (str == NULL)
        return (-1);

    
    writeCcsFile (str, data, nwords, addr);

    if (fname != NULL)
        fclose (str);

    return (0);

}
        
    




