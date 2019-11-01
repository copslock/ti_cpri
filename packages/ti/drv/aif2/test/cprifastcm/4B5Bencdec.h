/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/


#include<stdio.h>
#include<stdint.h>
#include<ti/csl/tistdtypes.h>

#define INPUT_LEN  (1500+8+6+6+2+4)
#define OUTPUT_LEN 	(((INPUT_LEN*10 )+17)/8)
#define OUT_PACKET_LEN (1500+6+6+2+4)	//Payload +Dest MAC+SRC MAC+ LENGTH+CRC

#define ssdPattern		0x238
#define esdPattern0 	0xED
#define esdPattern1 	0x4ED
#define esdPattern2 	0xCED
#define esdPattern3		0x1CED
#define esdPattern4		0x3CED
#define esdPattern5		0x7CED
#define esdPattern6 	0xFCED
#define esdPattern7 	0x1FCED

extern uint8_t inputData[INPUT_LEN];
extern uint8_t outputData[OUT_PACKET_LEN];

void fourBitToFiveBitEncoder(uint8_t *inputPtr, uint8_t *outputPtr, uint32_t inLength,uint32_t outLength);

void processRxCmReset(void);
int processRxCM(int rxCount,int *outputlength, unsigned char *outputPtr, Qmss_QueueHnd rxQ, Qmss_QueueHnd rxFq);
void init_crc32_table(void);
unsigned long crc32(unsigned char *data, unsigned long len);



//////////////////////////////////
