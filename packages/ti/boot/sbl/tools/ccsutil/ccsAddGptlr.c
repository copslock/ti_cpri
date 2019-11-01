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
/******************************************************************************
 * FILE PURPOSE: Add a GP Trailer to a CCS file
 ******************************************************************************
 * FILE NAME: ccsAddGptlr.c
 *
 * DESCRIPTION: Adds the 8 byte General Purpose Trailer
 *
 *  Usage: ccsAddGptlr [-h] [-infile infile] [-outfile outfile]
 *
 ******************************************************************************/
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include "ccsutil.h"

char *infile  = NULL;
char *outfile = NULL;

int parseit (int ac, char *av[])
{

    int i;

    for (i = 1; i < ac;  )  {

        if (!strcmp (av[i], "-infile"))  {
            infile = av[i+1];
            i = i + 2;
            continue;
        }

        if (!strcmp (av[i], "-outfile"))  {
            outfile = av[i+1];
            i = i + 2;
            continue;
        }

        if (!strcmp (av[i], "-h"))  {
            fprintf (stderr, "usage: %s [-h] [-infile infile] [-outfile outfile]\n", av[0]);
            return (-1);
        }

        fprintf (stderr, "%s: Unknown option %s\n", av[0], av[i]);
        return (-1);

    }

    return (0);

}


int main (int argc, char *argv[])
{
    FILE         *str;
    unsigned int *dat, addr;
    int           n;

    if (parseit (argc, argv))
        return (-1);


    if (infile != NULL)  {

        str = fopen (infile, "r");
        if (str == NULL)  {
            fprintf (stderr, "%s: Failed to open input file %s\n", argv[0], infile);
            return (-1);
        }

    }  else  {

        str = stdin;

    }

    dat = readCcsFile (str, &n, &addr);

    if (dat == NULL)  {

        fprintf (stderr, "%s: Error reading CCS data file\n", argv[0]);
        return (-1);

    }

    if (infile != NULL)
        fclose (str);

    
    dat = realloc (dat, (n+2) * sizeof(unsigned int));
    if (dat == NULL)  {
        fprintf (stderr, "%s: Realloc failed\n", argv[0]);
        return (-1);
    }

    dat[n+0] = 0;
    dat[n+1] = 0;
    n = n + 2;

    if (outfile != NULL)  {

        str = fopen (outfile, "w");
        if (str == NULL)  {
            fprintf (stderr, "%s: Failed to open output file %s\n", argv[0], outfile);
            free (dat);
            return (-1);
        }

    }  else  {

        str = stdout;

    }

    writeCcsFile (str, dat, n, addr);

    if (outfile != NULL)
        fclose (str);

    free (dat);

    return (0);

}
    

