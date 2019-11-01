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


#include <stdio.h>
#include <stdint.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

#include "4B5Bencdec.h"

#ifdef _LITTLE_ENDIAN
#define _endian_swap(a) (a)
#else
#define _endian_swap(a) _packlh2(_swap4(a),_swap4(a))
#endif

unsigned char fiveBitEncoder[16]={
 0x1E, 0x09, 0x14, 0x15, 0x0A, 0x0B, 0x0E, 0x0F, 0X12, 0x13, 0x16, 0x17, 0X1A, 0X1B, 0X1C, 0X1D
};

unsigned char fourBitDecoder[32]={
0Xff, //Invalid Code
0XFF, //Invalid Code
0xff, //Invalid Code
0xff, //Invalid Code
0xFE, //Halt
0xFF, //Invalid Code
0xFF, //Invalid Code
0XFD, //ESD part2
0xFF, //Invalid Code
0x01,
0x04,
0x05,
0xFF, //Invalid code
0xFC, //ESD part1
0x06,
0x07,
0XFF,
0xFF,
0x08,
0x09,
0x02,
0x03,
0x0A,
0x0B,
0xFB, //SSD part1
0XFA, //SSD part2
0x0c,
0x0D,
0X0E,
0X0F,
0X00,
0xF9 //Idle
};

/*** 10Bit format of JK along with 7 bytes preamble***/
unsigned char preamble_10[10]={0x38,0xAE,0xB5,0xD6,0x5A,0x6B,0xAD,0xB5,0xD6,0XDA};
/*** CPRI ESD PART1 andESD PART2 ***/
unsigned char esdPart1 = 0x0D, esdPart2 = 0x07;
//unsigned char dst_mac[6] ={0x00,0x0D,0XEE,0X09,0x05,0x24};

void fourBitToFiveBitEncoder(uint8_t *input, uint8_t *output, uint32_t inLength,uint32_t outLength){

	unsigned int i,j,length;
	unsigned char  temp, temp1, temp2, temp3,temp4,temp5,temp6,temp7;
	length = inLength;
	for(i=0;i<10;i++){

		output[i]=preamble_10[i];
	}

	for(i=8,j=10;i<(length/4)*4;i=i+4,j=j+5){

		//For Output Byte 1
		temp = fiveBitEncoder[input[i] & 0x0F];//1
		temp1 = fiveBitEncoder[(input[i]>>4) & 0x0F];//2

		//For Output Byte2
		temp2 = fiveBitEncoder[input[i+1] & 0x0F];//3
		temp3 =  fiveBitEncoder[(input[i+1]>>4) & 0x0F];//4

		//For Output Byte 3
		temp4 = fiveBitEncoder[input[i+2] & 0x0F];//5

		//For Output Byte 4
		temp5 = fiveBitEncoder[(input[i+2]>>4) & 0x0F];//6
		temp6  = fiveBitEncoder[input[i+3] & 0x0F];//7

		temp7 = fiveBitEncoder[(input[i+3]>>4) & 0x0F];

		output[j]= ((temp1 <<5) | (temp));
		output[j+1]= ((temp3<<7 )| (temp2<<2) | (temp1>>3));
		output[j+2]= ((temp4<<4)| (temp3>>1));

		output[j+3] = ((temp6<<6)|(temp5<<1)|(temp4>>4));

		//For OutPut Byte 5
		output[j+4] = ((temp7<<3) | (temp6>>2));

	}
//To calculate the processing for N%4 = 1,2,3 and add ESD in each case
	if( 0== (length % 4)){
		output[j]=   ((esdPart2 <<5) | (esdPart1));
		output[j+1]= ((0XFC)| (esdPart2>>3));
	}

	else if( 1== (length % 4)){
		temp = fiveBitEncoder[input[i] & 0x0F];
		temp1 = fiveBitEncoder[(input[i]>>4) & 0x0F];

		output[j]=   ((temp1 <<5) | (temp));
		output[j+1]= ((esdPart2<<7) | (esdPart1<<2) | (temp1>>3));
		output[j+2]= ((0XF0)|(esdPart2>>1));
	}
	else if(2 == (length % 4)){

			//For byte1
		    temp = fiveBitEncoder[input[i] & 0x0F];
			temp1 = fiveBitEncoder[(input[i]>>4) & 0x0F];

			output[j]= ((temp1 <<5) | (temp));

			//For Output Byte2
			temp2 = fiveBitEncoder[input[i+1] & 0x0F];
			temp =  fiveBitEncoder[(input[i+1]>>4) & 0x0F];
			output[j+1]= ((temp<<7 )| (temp2<<2) | (temp1>>3));

			output[j+2]= ((esdPart1<<4)| (temp>>1));
			output[j+3]= ((0xC0)|(esdPart2<<1)|(esdPart1>>4));
	}
	else if(3 == (length % 4) ){

		//For Output Byte 1
				temp = fiveBitEncoder[input[i] & 0x0F];
				temp1 = fiveBitEncoder[(input[i]>>4) & 0x0F];

				output[j]= ((temp1 <<5) | (temp));

				//For Output Byte2
				temp2 = fiveBitEncoder[input[i+1] & 0x0F];
				temp =  fiveBitEncoder[(input[i+1]>>4) & 0x0F];
				output[j+1]= ((temp<<7 )| (temp2<<2) | (temp1>>3));

				//For Output Byte 3
				temp1 = fiveBitEncoder[input[i+2] & 0x0F];

				output[j+2]= ((temp1<<4)| (temp>>1));

				//For Output Byte 4
				temp2 = fiveBitEncoder[(input[i+2]>>4) & 0x0F];

				output[j+3] = ((esdPart1<<6)|(temp2<<1)|(temp1>>4));

				//For OutPut Byte 5
				output[j+4] = ((esdPart2<<3) | (esdPart1>>2));
	}
}

void fiveBitToFourBitDecoder(uint8_t *input,uint32_t inputLength, uint32_t ssdOffset0, uint32_t esdOffset0,uint8_t *output,uint32_t *outputLength){

	int i,j;
	unsigned int temp, temp1,temp2,temp3,temp4 ,temp5,temp6,temp7;

	*outputLength = (inputLength * 8 - ssdOffset0 - esdOffset0 -10)/10;

//The below code is for Payload starting from Destination MAC ..
	if(ssdOffset0 == 0){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp = (input[i] & 0x1f);
			temp1 =(((input[i+1]& 0x3)<<3 )| (input[i]>>5));

			temp2 = ((input[i+1]>>2 )& 0x1F);
			temp3= (((input[i+2]&0xF)<<1)|(input[i+1]>>7));

			temp4 = (((input[i+3]&1)<<4) | (input[i+2]>>4));
			temp5 = ((input[i+3]>>1) & 0x1f);

			temp6 = (((input[i+4] & 0x7)<<2)|(input[i+3]>>6));
			temp7 = (input[i+4]>>3);

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
		}
	}

	if(ssdOffset0 == 1){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp = ((input[i]>>1) & 0x1f);
			temp1 =(((input[i+1]& 0x7)<<2 )| (input[i]>>6));

			temp2 = (input[i+1]>>3);
			temp3= ((input[i+2]) & 0x1F);

			temp4 = (((input[i+3]& 0x3)<<3) | (input[i+2]>>5));
			temp5 = ((input[i+3]>>2) & 0x1f);

			temp6 = (((input[i+4] & 0xF)<<1)|(input[i+3]>>7));
			temp7 = (((input[i+5] & 0x1)<<4)|(input[i+4]>>4));

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
		}
	}

	if(ssdOffset0 == 2){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp = ((input[i]>>2) & 0x1f);
			temp1 =(((input[i+1]& 0xF)<<1 )| (input[i]>>7));

			temp2 = (((input[i+2] & 0x1) <<4 )|(input[i+1]>>4 ));
			temp3= ((input[i+2] >> 1) & 0x1F);

			temp4 = (((input[i+3]& 0x7)<<2) | (input[i+2]>>6));
			temp5 = ((input[i+3]>>3));

			temp6 = (input[i+4] & 0x1F);
			temp7 = (((input[i+5] & 0x3)<<3)|(input[i+4]>>5));

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
		}
	}

	if(ssdOffset0 == 3){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp = (input[i]>>3);
			temp1 =(input[i+1]& 0x1f);

			temp2 = (((input[i+2] & 0x3)<<3)|(input[i+1]>>5));
			temp3= ((input[i+2]>>2) & 0x1F);

			temp4 = (((input[i+3]& 0xf)<<1) | (input[i+2]>>7));
			temp5 =(((input[i+4] & 0x1)<<4)|(input[i+3]>>4));

			temp6 = ((input[i+4] >>1) & 0x1f);
			temp7 = (((input[i+5] & 0x7)<<2)|(input[i+4]>>6));

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
		}
	}

	if(ssdOffset0 == 4){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp  = (((input[i+1] & 0x1)<<4)|(input[i]>>4));
			temp1 = ((input[i+1]>>1)& 0x1f);

			temp2 = (((input[i+2] & 0x7)<<2)|(input[i+1]>>6 ));
			temp3 = ((input[i+2])>>3);

			temp4 = ((input[i+3] & 0x1f));
			temp5 = (((input[i+4] & 0x3)<<3)|(input[i+3]>>5));

			temp6 = ((input[i+4]>>2)& 0x1f);
			temp7 = (((input[i+5] & 0xF)<<1)|(input[i+4]>>7));

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));

		}
	}

	if(ssdOffset0 == 5){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp = (((input[i+1]&0x3)<<3)|(input[i]>>5));
			temp1 =((input[i+1]>>2) & 0x1F);

			temp2 = (((input[i+2]& 0xF)<<1)|(input[i+1]>>7));
			temp3= (((input[i+3] & 0x1)<<4)|((input[i+2])>>4));

			temp4 = ((input[i+3]>> 1) & 0x1f);
			temp5 = (((input[i+4] & 0x7)<<2)|(input[i+3]>>6) );

			temp6 = (input[i+4] >>3);
			temp7 = input[i+5] & 0x1F;;

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
		}
	}

	if(ssdOffset0 == 6){
		for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){

			temp = (((input[i+1] & 0x7)<<2)|(input[i]>>6));
			temp1 =(input[i+1]>>3);

			temp2 = ((input[i+2])& 0x1F);
			temp3 = (((input[i+3] & 0x3)<<3)|(input[i+2]>>5));

			temp4 =  ((input[i+3]>>2) & 0x1f);
			temp5 = (((input[i+4] & 0xf)<<1)|(input[i+3]>>7));

			temp6 = (((input[i+5] & 0x1)<<4)|(input[i+4]>>4));
			temp7 = ((input[i+5]>>1)& 0x1f);

			output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
			output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
			output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
			output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
		}
	}

	if(ssdOffset0 == 7){
			for(i=0,j=0;j<*outputLength;i=i+5,j=j+4){


				temp = (((input[i+1] & 0x0F)<< 1)| (input[i] >> 7));
				temp1 =(((input[i+2] & 0x1)<< 4 )| (input[i+1]>>4));

				temp2 = ((input[i+2]>>1 )& 0x1F);
				temp3 = (((input[i+3] & 0x7)<<2)|(input[i+2] >> 6));

				temp4 = (input[i+3]>>3);
				temp5 = ((input[i+4]) & 0x1f);

				temp6 = (((input[i+5] & 0x3)<<3)|(input[i+4]>> 5));
				temp7 = ((input[i+5]>>2) & 0x1f);

				output[j] = ((fourBitDecoder[temp1]<<4)|(fourBitDecoder[temp]));
				output[j+1] = ((fourBitDecoder[temp3]<<4)|(fourBitDecoder[temp2]));
				output[j+2] = ((fourBitDecoder[temp5]<<4)|(fourBitDecoder[temp4]));
				output[j+3] = ((fourBitDecoder[temp7]<<4)|(fourBitDecoder[temp6]));
			}
		}
	output[0]= 0x55;
}

static uint8_t   rxCtrlDataBuff[1920];
static uint32_t  packetlength = 0;
static uint32_t  ssdDetected=0 , esdDetected=0, ssdOffset, esdOffset, lenInBits = 0;
static uint8_t   *rxCtrlDataBuffptr = rxCtrlDataBuff;

void processRxCmReset()
{
	esdDetected = 0;
	ssdDetected = 0;
	lenInBits = 0;
	packetlength = 0;
	rxCtrlDataBuffptr = rxCtrlDataBuff;
}

int processRxCM(int rxCount,int *outputlength, unsigned char *outputPtr, Qmss_QueueHnd rxQ, Qmss_QueueHnd rxFq)
{
	uint32_t 				i, destLen, decoderOutPacketLen;
	Cppi_MonolithicDesc *rxPkt;
	uint8_t				*esdSearchPtr;
	uint32_t 				inputdata, esdData0, esdData1, esdData2;
	uint32_t   			*rxBuffPtr;

    /*Go through all received descriptos and search for SSD and ESD*/
    for(i =0; i<rxCount;i++)	{

    	rxPkt = ((Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(rxQ)));
    	CACHE_invL1d((void *)rxPkt, 12 , CACHE_WAIT);
    	Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)rxPkt, (uint8_t**)&rxBuffPtr, &destLen);
    	CACHE_invL1d((void *)rxBuffPtr, destLen , CACHE_WAIT);

    	if(ssdDetected == 0){

    		//SSD detection 0 to 1
    		 inputdata = _endian_swap((uint32_t )*&(rxBuffPtr[0]));

    		 if((inputdata & 0x3ff) == ssdPattern){
    			 ssdDetected =1;
    			 ssdOffset = 0;
    		 }
    		 else if(((inputdata>>1) & 0x3ff) == ssdPattern){
    			 ssdDetected =1;
    			 ssdOffset = 1;
    		 }
    		 else if(((inputdata>>2) & 0x3ff) == ssdPattern){
    			 ssdDetected =1;
    			 ssdOffset = 2;
    		 }
    		 else if(((inputdata>>3) & 0x3ff) == ssdPattern){
    	    	 ssdDetected =1;
    	    	 ssdOffset = 3;
    	     }
    		 else if(((inputdata>>4) & 0x3ff) == ssdPattern){
    		     ssdDetected =1;
    		     ssdOffset = 4;
    		 }
    		 else if(((inputdata>>5) & 0x3ff) == ssdPattern){
    		    ssdDetected =1;
    		    ssdOffset = 5;
    		 }
    		 else if(((inputdata>>6) & 0x3ff) == ssdPattern){
    	    	ssdDetected =1;
    	    	ssdOffset = 6;
    		 }
    		 else if(((inputdata>>7) & 0x3ff) == ssdPattern){
    	    	ssdDetected =1;
    	    	ssdOffset = 7;
    	    }

    	}

    	if(esdDetected == 0){

    		lenInBits += destLen * 8;
    		/*** Check if ESD is there in First packet , else concatenate the data and proceed for the second packet***/
    		//SSD detection 0 to 1
    		esdSearchPtr =(uint8_t *)rxBuffPtr;
    		esdData0 = esdSearchPtr[destLen-3];
    		esdData1 = esdSearchPtr[destLen-2];
    		esdData2 = esdSearchPtr[destLen-1];

    		inputdata = (esdData2 <<16)|(esdData1 <<8)|(esdData0);
//    		printf("0x%02x",(inputdata));
    		if((inputdata >> 14) == esdPattern0){

    			esdOffset = 0;
    			if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    				esdDetected =1;
    			}
    		}
    		else if((inputdata >>13) == esdPattern1){

    			esdOffset = 1;
    			if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    			    esdDetected =1;
    			}
    		}
    		else if((inputdata >> 12) == esdPattern2){

    	    	esdOffset = 2;
    	    	if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    	    	    esdDetected =1;
    	    	}
    	    }
    		else if((inputdata >> 11) == esdPattern3){

    			esdOffset = 3;
    			if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    			    esdDetected =1;
    			}

    		}
    		else if((inputdata >> 10) == esdPattern4){

    		    esdOffset = 4;
    		    if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    		        esdDetected =1;
    		    }
    		}
    		else if((inputdata >> 9) == esdPattern5){

    		    esdOffset = 5;
    		    if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    		        esdDetected =1;
    		    }
    		}
    		else if((inputdata >> 8) == esdPattern6){

    		    esdOffset = 6;
    		    if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    		        esdDetected =1;
    		    }
    		}
    		else if((inputdata >> 7) == esdPattern7){

    		    esdOffset = 7;
    		    if((lenInBits-esdOffset-ssdOffset)%10 == 0){
    		        esdDetected =1;
    		    }
    		}

    		if(esdDetected == 0){
    			memcpy(rxCtrlDataBuffptr, &rxBuffPtr[0],destLen);
    			rxCtrlDataBuffptr = rxCtrlDataBuffptr+destLen;
    			*rxCtrlDataBuffptr = 0xFF;
    			rxCtrlDataBuffptr++;
    			packetlength = packetlength+destLen+1;
    			lenInBits+=8;

    			/* Push descriptor back to free queue */
    			Qmss_queuePushDesc(rxFq, (uint32_t*)rxPkt);

    			//System_printf("\n ->Incomplete Data --  ");
    			//dump_packet((uint8_t*)rxCtrlDataBuffptr, packetlength);

    			//return 0;
    		}
    		else{
    			memcpy(rxCtrlDataBuffptr, &rxBuffPtr[0],destLen);
    			packetlength = packetlength+destLen;
    			/* Push descriptor back to free queue */
    			Qmss_queuePushDesc(rxFq, (uint32_t*)rxPkt);
    			fiveBitToFourBitDecoder(&(rxCtrlDataBuff[0]),packetlength,ssdOffset,esdOffset,outputPtr,&decoderOutPacketLen);
    			*outputlength = decoderOutPacketLen;
    			// reset all static variables for the next packet
    			processRxCmReset();

    			return 1;
   			}

    	}

    }
    return 0;
}

/********FOR CRC ****************/
#define  CRC_INIT_VAL  0xFFFFFFFF
#define  XOROUT        0xFFFFFFFF
#define  POLY          0x04C11DB7
#define  BITMASK(X)    (1L<<(X))
static unsigned long crctab[256];

/*------------------------CRC Calculation--------------------------*/
unsigned long reflect(unsigned long v, int b)
/* Returns the value v with the bottom b [0,32] bits reflected. */
/* Example: reflect(0x3e23L,3) == 0x3e26                        */
{
  int i;
  unsigned long t = v;
  for (i=0; i<b; i++)
  {
    if (t & 1L)
      v |=  BITMASK((b-1)-i);
    else
      v &= ~BITMASK((b-1)-i);
    t>>=1;
   }
  return v;
}

void init_crc32_table(void)
{
  int n,i;
  unsigned long r;

	for (n=0; n<256; n++)
	{
 		r = reflect(n,8) << 24;
	  for (i=0; i<8; i++)
	  {
      if (r & (unsigned long)(1L<<31))
        r = (r << 1) ^ POLY;
      else
        r <<= 1;
    }
    crctab[n] = reflect(r,32);
  }
  //return r;
}

unsigned long crc32(unsigned char *data, unsigned long len)
{
  unsigned long crc = CRC_INIT_VAL;

  // dump data for CRC calculation
//  System_printf(" \n packet before CRC calculation \n ");
//  dump_packet(data,len);

  while (len--) {
    crc = crctab[(crc ^ *data++) & 0xFFL] ^ (crc>>8);
  }
  return (crc ^ XOROUT);
}





