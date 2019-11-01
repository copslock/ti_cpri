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
/*****************************************************************************
 * FILE PURPOSE: Add a GP header to a CCS data file
 ******************************************************************************
 * FILE NAME: ccsAddGphdr.c
 *
 * Description: Adds the two word GP header to a ccs data file
 *
 *  usage: ccsAddGphdr [baseAddress] [-infile infile] [-outfile outfile]
 *
 ******************************************************************************/
#include <stdio.h>
#include <malloc.h>
#include "ccsutil.h"

#define		LE	0
#define		BE 	1

char *infile  = NULL;
char *outfile = NULL;
unsigned int baseAddress = 0xffffffff;
int headerEndianess = BE;


unsigned int readInt (char *c)
{
    unsigned int v;

    if ((c[0] == '0') && (c[1] == 'x'))  
        sscanf (&c[2], "%x", &v);

    else
        sscanf (c, "%d", &v);

    return (v);

}

int parseit (int ac, char *av[])
{
    int i;

    if (ac == 1)  {
        fprintf (stderr, "usage: %s [baseAddress] [-infile infile] [-outfile outfile] [-headerEndian 'BE'|'LE']\n", av[0]);
        return (-1);
    }

    for (i = 1; i < ac;  )  {

        if (!strcmp(av[i], "-infile"))  {
            infile = av[i+1];
            i = i + 2;

        }  else if (!strcmp(av[i], "-outfile"))  {
            outfile = av[i+1];
            i = i + 2;

        }  else if (!strcmp(av[i], "-headerEndian"))  {
            if(!strcmp(av[i+1], "LE"))	{
				headerEndianess = LE;
			} else	{
				headerEndianess = BE;
			}
			i = i + 2;

        }  else  {

            baseAddress = readInt (av[i]);
            i = i + 1;
        }

    }


    return (0);

}


int main (int argc, char *argv[])
{
    FILE *str;
    unsigned int *data1, *data2, addr, *tmp;
    int nwords;
    int i;

    if (parseit (argc, argv) != 0)
        return (-1);

    if (infile != NULL)  {
        str = fopen (infile, "r");
        if (str == NULL)  {
            fprintf (stderr, "%s error: failed to open file %s\n", argv[0], infile);
            return (-1);
        }
    }  else  {
        str = stdin;
    }

    data1 = readCcsFile (str, &nwords, &addr);

    if (baseAddress == 0xffffffff)
        baseAddress = addr;

    
    
    if (infile != NULL)
        fclose (str);

    if (data1 == NULL)  {
        fprintf (stderr, "%s error: read ccs file returned error\n", argv[0]);
        return (-1);
    }

    data2 = malloc ((nwords + 2) * sizeof(unsigned int));
    if (data2 == NULL)  {
        fprintf (stderr, "%s error: malloc failed on %d unsigned ints\n", argv[0], nwords + 2);
        free (data1);
        return (-1);
    }

	data2[0] = nwords * 4;
    data2[1] = baseAddress;
	
	if (headerEndianess = BE)	{
		ccsEndianSwap(data2, 2);
	}

    for (i = 0; i < nwords; i++)
        data2[i+2] = data1[i];

    free (data1);

    if (outfile != NULL)  {
        str = fopen (outfile, "w");
        if (str == NULL)  {
            fprintf (stderr, "%s error: failed to open file %s\n", argv[0], outfile);
            return (-1);
        }
    }  else  {
        str = stdout;
    }

    writeCcsFile (str, data2, nwords+2, baseAddress);

    free (data2);

    if (outfile != NULL)
        fclose (str);

    return (0);

}

    





