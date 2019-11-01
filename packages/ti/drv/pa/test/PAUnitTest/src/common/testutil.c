/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "../pautest.h"

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
extern cregister volatile unsigned int DNUM;
extern cregister volatile unsigned int TSCL;
#endif
#endif

#ifndef __LINUX_USER_SPACE
#if 0
#include <cpsw_mgmt.h>

#ifdef  SIMULATOR_SUPPORT
int cpswSimTest = 1;
int cpswLpbkMode = CPSW_LOOPBACK_INTERNAL;
#else
int cpswSimTest = 0;
#ifndef __LINUX_USER_SPACE
int cpswLpbkMode = CPSW_LOOPBACK_NONE;
#endif
#endif

extern Int32 Init_Cpsw (Void);

void CycleDelay (int count)
{
	utilCycleDelay (count);
}
#endif

int32_t utilInitCpsw(void)
{
	// Init_Cpsw();
	return (0);
}

void utilCycleDelay (int count)
{
#ifdef _TMS320C6X
  uint32_t TSCLin = TSCL;

  if (count <= 0)
    return;

  while ((TSCL - TSCLin) < (uint32_t)count);
#else
    uint32_t start, end, cycles;

    if (count <= 0)
        return;

    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(start));
    cycles = (uint32_t)count;
    if ((0x100000000 - start) < (uint32_t)cycles) {
        do {
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(end));
        } while (end > start);
        cycles -= 0x100000000 - start;
        start = 0;
    }
    do {
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(end));
    } while ((end - start) < cycles);
#endif
}
#else
void utilCycleDelay (int32_t count)
{
    uint32_t sat; 
    for (sat=0; sat<count; sat++);
}
#endif

#ifndef __LINUX_USER_SPACE
uint32_t utilgAddr(uint32_t x)
{
#ifdef _TMS320C6X
	if ((x >= 0x800000) && (x < 0x900000))
	  x = (1 << 28) | (DNUM << 24) | x;
#endif
  	return (x);
}
#else
uint32_t utilgAddr(uint32_t x)
{
    return x;
}
#endif

uint16_t utilOnesComplementAdd (uint16_t v1, uint16_t v2)
{
  uint32_t result;

  result = (uint32_t)v1 + (uint32_t)v2;
  result = (result >> 16) + (result & 0xffff);
  result = (result >> 16) + (result & 0xffff);

  return ((uint16_t)result);

}

uint16_t utilOnesCompChkSum (uint8_t *p, unsigned int nwords)
{
  uint16_t chksum = 0;
  uint16_t v;
  unsigned int i;
  unsigned int j;

  for (i = j = 0; i < nwords; i++, j+=2)  {
    v = (p[j] << 8) | p[j+1];
    chksum = utilOnesComplementAdd (chksum, v);
  }

  return (chksum);

} /* onesCompChkSum */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute ipv4 psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ipv4 psudo checksum
 **************************************************************************************/
uint16_t utilGetIpv4PsudoChkSum (uint8_t *data, uint16_t payloadLen, uint16_t proto)
{
  uint16_t psudo_chksum;

  psudo_chksum = utilOnesCompChkSum (&data[12], 4);
  psudo_chksum = utilOnesComplementAdd(psudo_chksum, proto);
  psudo_chksum = utilOnesComplementAdd(psudo_chksum, payloadLen);
  
  return (psudo_chksum);

} /* utilGetIpv4PsudoChkSum */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute ipv6 psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ipv6 psudo checksum
 **************************************************************************************/
uint16_t utilGetIpv6PsudoChkSum (uint8_t *data, uint16_t payloadLen, uint16_t proto)
{
  uint16_t psudo_chksum;

  psudo_chksum = utilOnesCompChkSum (&data[8], 16);
  psudo_chksum = utilOnesComplementAdd(psudo_chksum, proto);
  psudo_chksum = utilOnesComplementAdd(psudo_chksum, payloadLen);
  
  return (psudo_chksum);

} /* utilGetIpv6PsudoChkSum */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute ip psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ip psudo checksum
 **************************************************************************************/
uint16_t utilGetIpPsudoChkSum (uint8_t *ip, uint16_t payloadLen, uint16_t proto)
{
  uint16_t psudo_chksum;
  
  if ((ip[0] & 0xF0) == 0x40)
  {
      psudo_chksum = utilGetIpv4PsudoChkSum(ip, payloadLen, proto);
  }
  else
  {
      psudo_chksum = utilGetIpv6PsudoChkSum(ip, payloadLen, proto);
  }
  
  return (psudo_chksum);

} /* utilGetIpPsudoChkSum */

/* Compute UDP checksums */
uint16_t utilCompUdpChksums (pktTestInfo_t *pktinfo, Bool insert)
{
	uint8_t  *udp;
    uint8_t  *ip;
	uint16_t 	udpOffset;
    uint16_t  ipOffset;
	uint16_t  pseudo;
	uint16_t  sum;
	int32_t     pktLen;
	
	udpOffset = TF_GET_UDP_OFFSET(pktinfo->info);
	ipOffset  = TF_GET_IP_OFFSET(pktinfo->info);
	
	udp   = &(pktinfo->pkt[udpOffset]);
	ip    = &(pktinfo->pkt[ipOffset]);
    
	pktLen = ((((int32_t)udp[4] << 8) | (int32_t)udp[5]) + 1) & ~1; 
    
    if ((ip[0] & 0xF0) == 0x40)
    {
        pseudo = utilGetIpv4PsudoChkSum(ip, pktLen, 0x11);
    }
    else
    {
        pseudo = utilGetIpv6PsudoChkSum(ip, pktLen, 0x11);
    }
	
	if (insert == TRUE)  {	
		/* replace the checksum with the pseudo header checksum */
		udp[6] = pseudo >> 8;
		udp[7] = pseudo & 0xff;
	}
	
	sum = utilOnesCompChkSum (udp, pktLen>>1);
    
    if (pktLen & 1)
    {
        sum = utilOnesComplementAdd(sum, udp[pktLen-1] << 8);
    }
    
    sum = ~sum;
	
    if(sum == 0)
       sum = 0xFFFF;
    
	if (insert == TRUE)  {
		udp[6] = sum >> 8;
		udp[7] = sum & 0x00ff;
	}
	
	return (sum);
}

/* Compute and Insert IP checksums */
uint16_t utilCompIpChksums (pktTestInfo_t *pktinfo,  Bool insert)
{
	uint8_t  *ip;
	uint16_t 	offset;
	uint16_t  sum;
	
	offset = TF_GET_IP_OFFSET(pktinfo->info);
	
	ip   = &(pktinfo->pkt[offset]);
	
	/* reset the checksum field to zero */
	if (insert == TRUE)  {	
	    ip[10] = 0;
	    ip[11] = 0;
	}
	sum = ~utilOnesCompChkSum (ip, 10);
	
	if (insert == TRUE)  {	
	    ip[10] = sum >> 8;
	    ip[11] = sum & 0x00ff;
	}
	
	return (sum);
}

uint16_t utilUpdateIpChksums (uint8_t  *ip)
{
	uint16_t  sum;
	
	/* reset the checksum field to zero */
	ip[10] = 0;
	ip[11] = 0;
    
	sum = ~utilOnesCompChkSum (ip, 10);
	
	ip[10] = sum >> 8;
	ip[11] = sum & 0x00ff;
	
	return (sum);
}


/*******************************************************************************
 *  Function: Generate payload  
 *******************************************************************************
 *  DESCRIPTION:  Fill the data buffer with the specified payload data  
 *
 *  Note: It is up to the caller to allocate buffer
 ******************************************************************************/
void testGenPayload(pauPayloadType_e type, uint8_t initValue, uint16_t len, uint8_t* buf)
{
    uint8_t data = initValue;
    int i;
    
    switch (type)
    {
        case PAU_PAYLOAD_CONST:
            memset(buf, data, len);
            break;
            
        case PAU_PAYLOAD_INC8: 
            for(i = 0; i < len; i++) buf[i] = data++;       
            break;
            
        case PAU_PAYLOAD_DEC8: 
            for(i = 0; i < len; i++) buf[i] = data--;       
            break;
            
         case PAU_PAYLOAD_RANDOM:
            for(i = 0; i < len; i++) buf[i] = rand() & 0xFF;       
            break;
            
         default:
            System_printf("testGenPayload: invalid paylaod type (%d)\n", type);
            System_flush();
            break;   
    }
}

void check8kQueues(void)
{
   int i, cnt, total = 0;

   for (i = 0; i < 8000; i++) {
	   cnt = Qmss_getQueueEntryCount(i);
	   if ( cnt > 0)
		   System_printf ("Queue %d is not Empty, has %d descriptors \n", i, cnt);
	       System_flush();
	       total += cnt;
   }

   System_printf ("total number of descriptors on all queues is %d \n", total);
   System_flush();
}

/* Nothing past this point */
	
		
		
		


