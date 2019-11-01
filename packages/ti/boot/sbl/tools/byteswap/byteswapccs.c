/*
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
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
/********************************************************************************************
 * FILE PURPOSE: Byte swap a CCS data file
 ********************************************************************************************
 * FILE NAME: byteswapccs.c
 *
 * DESCRIPTION: A CCS file is read in, the data is byte swapped, and a CCS file is written out
 *
 *  usage: byteswapccs infile outfile
 *
 ********************************************************************************************/
#include <stdio.h>
#include <malloc.h>


int main (int argc, char *argv[])
{
    FILE *fin, *fout;
    unsigned int v, b0, b1, b2, b3;
    int a, b, c, d, n;
    int i;
    char iline[132];


    if (argc != 3)  {
        fprintf (stderr, "usage: %s infile outfile\n", argv[0]);
        return (-1);
    }


    fin = fopen (argv[1], "r");
    if (fin == NULL)  {
        fprintf (stderr, "%s: Could not open input file %s\n", argv[1]);
        return (-1);
    }

    fout = fopen (argv[2], "w");
    if (fout == NULL)  {
        fprintf (stderr, "%s: Could not open output file %s\n", argv[2]);
        fclose (fin);
        return (-1);
    }


    /* Read the CCS data file header, write it out unchanged */
    fgets (iline, 132, fin);
    sscanf (iline, "%x %x %x %x %x", &a, &b, &c, &d, &n);
    fputs (iline, fout);

    for (i = 0; i < n; i++)  {
        fgets (iline, 132, fin);
        sscanf (&iline[2], "%x", &v);

        b0 = (v >> 24) & 0xff;
        b1 = (v >> 16) & 0xff;
        b2 = (v >>  8) & 0xff;
        b3 = (v >>  0) & 0xff;

        v = (b3 << 24) | (b2 << 16) | (b1 <<8) | b0;
        fprintf (fout, "0x%08x\n", v);
    }

    fclose (fout);
    fclose (fin);

    return (0);

}









    





