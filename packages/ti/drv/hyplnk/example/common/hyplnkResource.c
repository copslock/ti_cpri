/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "hyplnkResource.h"
#ifdef _VIRTUAL_ADDR_SUPPORT
static int dev_mem_fd;
#endif

uint32_t hyplnk_memAllocInit()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
	if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
	{
		printf("Failed to open /dev/mem \n");
		return -1;
	}
	hyplnkInitCfg.dev.bases[hyplnk_EXAMPLE_PORT].cfgBase = (void *) hyplnk_mmap((uint32_t)(hyplnkInitCfg.dev.bases[hyplnk_EXAMPLE_PORT].cfgBase), 0x1000);
	hyplnkInitCfg.dev.bases[hyplnk_EXAMPLE_PORT].dataBase = (void *) hyplnk_mmap((uint32_t)(hyplnkInitCfg.dev.bases[hyplnk_EXAMPLE_PORT].dataBase), 0x10000000);
	return 0;
#else
	return 0;
#endif
}

uint32_t hyplnk_mmap(uint32_t addr, uint32_t size)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
        uint32_t virt_addr;
        uint32_t page_size;
        uint32_t remainder;
        uint32_t aligned_addr;
        page_size = sysconf(_SC_PAGE_SIZE);
        if (size%page_size)
        {
                printf("Size does not align with page size. Size given: %d\n", size);
                return 0;
        }
        remainder = addr % page_size;
        aligned_addr = addr / page_size;
        aligned_addr *= page_size;
        virt_addr = (uint32_t) mmap(0, size + remainder, (PROT_READ|PROT_WRITE), MAP_SHARED, dev_mem_fd, (off_t)aligned_addr);
        if (virt_addr == -1)
        {
                printf("mmap failed!\n");
                return 0;
        }
        return virt_addr + remainder;
#else
	return addr;
#endif
}

uint32_t hyplnk_memRelease(void * dataBufPtr, uint32_t size)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
	munmap((hyplnkInitCfg.dev.bases[hyplnk_EXAMPLE_PORT].cfgBase), 0x10000000);
	munmap((hyplnkInitCfg.dev.bases[hyplnk_EXAMPLE_PORT].dataBase), 0x1000);
	close(dev_mem_fd);
#else
	memset (dataBufPtr, 0, size);
	CACHE_invL1d(dataBufPtr, size, CACHE_WAIT);
#endif
	return 0;
}


void hyplnkPSCSetup(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
	uint32_t mem_base_PSC;
	mem_base_PSC = hyplnk_mmap(CSL_PSC_REGS, 0x10000);
	if (CSL_FEXT(((CSL_PscRegs *) mem_base_PSC)->PDSTAT[pwrDmnNum], PSC_PDSTAT_STATE) != PSC_PDSTATE_ON)
	{
		/* Enable the domain */
		CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, ON);
	}
	else
		printf ("Power domain is already enabled.  You probably re-ran without device reset (which is OK)\n");
	/* Enable MDCTL */
	CSL_FINS (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_NEXT, PSC_MODSTATE_ENABLE);
	/* Apply the domain */
	((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
	/* Wait for it to finish */
	while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
	munmap((void *) mem_base_PSC, 0x10000);
#else
  /* Turn on the Hyperlink power domain */
  if (CSL_PSC_getPowerDomainState (pwrDmnNum) != PSC_PDSTATE_ON) {
    /* Enable the domain */
    CSL_PSC_enablePowerDomain (pwrDmnNum);
  } else {
    System_printf ("Power domain is already enabled.  You probably re-ran without device reset (which is OK)\n");
  }
    /* Enable MDCTL */
    CSL_PSC_setModuleNextState (lpscNum, PSC_MODSTATE_ENABLE);
    /* Apply the domain */
    CSL_PSC_startStateTransition (pwrDmnNum);
    /* Wait for it to finish */
    while (! CSL_PSC_isStateTransitionDone (pwrDmnNum));
#endif
} /* hyplnkPSCSetup */

void hyplnkPSCDisable(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
	uint32_t mem_base_PSC;
	mem_base_PSC = hyplnk_mmap(CSL_PSC_REGS, 0x10000);
	/* Disable MDCTL */
	CSL_FINS (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_NEXT, PSC_MODSTATE_DISABLE);
	/* Apply the domain */
	((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
	/* Wait for it to finish */
	while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
	munmap((void *) mem_base_PSC, 0x10000);
#else
    /* Disable MDCTL */
    CSL_PSC_setModuleNextState (lpscNum, PSC_MODSTATE_DISABLE);
    /* Apply the domain */
    CSL_PSC_startStateTransition (pwrDmnNum);
    /* Wait for it to finish */
    while (! CSL_PSC_isStateTransitionDone (pwrDmnNum));

#endif
} /* hyplnkPSCDisable */

