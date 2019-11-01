/*
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *   @file  srio_status_decode.c
 *
 *   @brief
 *      Utilities to decode SRIO status register values.
 *
 */

/* Standard library and XDC Include Files. */
#include <stdio.h>
#include <xdc/runtime/System.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srioAuxPhyLayer.h>

/* Benchmarking Include Files. */
#include <benchmarking.h>
#ifndef CSL_SRIO_CONFIG_REGS
#define CSL_SRIO_CONFIG_REGS CSL_SRIO_CFG_REGS
#endif
CSL_SrioHandle      hSrio_Direct = (CSL_SrioHandle)CSL_SRIO_CONFIG_REGS;

/**
 *  @b Description
 *  @n
 *      The function is used to display a register's name, address and value.
 *
 *  @retval
 *      Control code text string translation.
 */
#pragma CODE_SECTION (displayRegisterAddrAndValue, ".text");
static Uint32 displayRegisterAddrAndValue (String registerName, Uint32 registerAddress)
{
	Uint32	regValue=0x1F;
	regValue = *(Uint32*)registerAddress;

	System_printf ("  %s: 0x%08x:0x%08x\n", registerName, registerAddress, regValue);

	return regValue;
}

/**
 *  @b Description
 *  @n
 *      The function is used to display multiple SRIO status registers
 *
 *  @retval
 *      Success - 0
 */
int32_t displayVariousRegisters (void)
{
	char indent[] = {"           "};

	System_printf ("Debug: Various Register Display:\n");
	              System_printf ("                          %s32211100\n", indent);
	              System_printf ("                          %s17395173\n", indent);
	              System_printf ("                          %s||||||||\n", indent);
	              System_printf ("                          %s22211000\n", indent);
	              System_printf ("                          %s84062840\n", indent);
	              System_printf ("                          %s--------\n", indent);
	displayRegisterAddrAndValue ("                 PCR", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x0004));
	displayRegisterAddrAndValue ("        PER_SET_CNTL", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x0014));
	displayRegisterAddrAndValue ("       PER_SET_CNTL1", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x0018));
	displayRegisterAddrAndValue ("   ERR_RST_EVNT_ICSR", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x01E0));
	displayRegisterAddrAndValue ("           LSUx_Reg6", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x0D18));
	displayRegisterAddrAndValue ("       LSU_STAT_REG0", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x0DE8));
	displayRegisterAddrAndValue ("           SP_RT_CTL", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xB124));
	displayRegisterAddrAndValue ("          SP_GEN_CTL", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xB13C));
	displayRegisterAddrAndValue ("Port n Control 2 CSR", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xB154));
	displayRegisterAddrAndValue ("        SPn_ERR_STAT", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xB158));
	displayRegisterAddrAndValue ("             SPn_CTL", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xB15C));
	displayRegisterAddrAndValue ("         SPx_ERR_DET", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xC040));
	displayRegisterAddrAndValue ("        SPx_ERR_RATE", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xC068));
	displayRegisterAddrAndValue ("         LANEn_STAT0", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xE010));
	displayRegisterAddrAndValue ("         LANEn_STAT1", (Uint32)(CSL_SRIO_CONFIG_REGS + 0xE014));
	displayRegisterAddrAndValue ("PLM Port(n)Implement", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x1B080));
	displayRegisterAddrAndValue ("    PLM_SP(n)_Status", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x1B090));
	displayRegisterAddrAndValue ("     EM_PW_PORT_STAT", (Uint32)(CSL_SRIO_CONFIG_REGS + 0x1B928));
	return 0;
}

void clearSrioStatusErrors (void)
{
	/* Clear Errors */
	CSL_SRIO_SetErrorDetectCSR (hSrio_Direct,0x0);
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode the Logical/Transport Layer Error Detect CSR
 *
 *  @retval
 *      Success - 0
 */
int32_t displaySrioLTLEDCSRErrorStatus (void)
{
	Uint8				bitPos;
	Uint32				errorStatus, errorMask;
	char* indentText;

	indentText = "  ";

	CSL_SRIO_GetErrorDetectCSR (hSrio_Direct,&errorStatus);
	clearSrioStatusErrors ();
	printf ("Debug: %sSRIO status: 0x%08X.\n", indentText, errorStatus);
    for (bitPos = 0; bitPos < 32; bitPos++)
    {
		errorMask = 0x1 << bitPos;
		switch (errorStatus & errorMask)
		{
			case 0x80000000:
				System_printf ("Debug:   %sIO error response\n", indentText);
				break;
			case 0x40000000:
				System_printf ("Debug:   %sMessage error response\n", indentText);
				break;
			case 0x20000000:
				System_printf ("Debug:   %sGSM error response\n", indentText);
				break;
			case 0x10000000:
				System_printf ("Debug:   %sMessage Format Error\n", indentText);
				break;
			case 0x08000000:
				System_printf ("Debug:   %sIllegal transaction decode\n", indentText);
				break;
			case 0x04000000:
				System_printf ("Debug:   %sIllegal transaction target error\n", indentText);
				break;
			case 0x02000000:
				System_printf ("Debug:   %sMessage Request Time-out\n", indentText);
				break;
			case 0x01000000:
				System_printf ("Debug:   %sPacket Response Time-out\n", indentText);
				break;
			case 0x00800000:
				System_printf ("Debug:   %sUnsolicited Response\n", indentText);
				break;
			case 0x00400000:
				System_printf ("Debug:   %sUnsupported Transaction\n", indentText);
				break;
			case 0x00000080:
				System_printf ("Debug:   %sRX CPPI Security Violation\n", indentText);
				break;
			case 0x00000040:
				System_printf ("Debug:   %sRX I/O DMA Access Error\n", indentText);
				break;
		}
    }
	return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to display the Port Response Timeout CSR value
 *
 *  @retval
 *      Success - 0
 */
int32_t displayPortResponseTimeout (void)
{
	Uint32	timeout;
	CSL_SRIO_GetPortResponseTimeoutCSR (hSrio_Direct,&timeout);
	System_printf ("Debug:   Port response timeout: 0x%x\n", timeout);
	return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to display the Port General CSR decodes
 *
 *  @retval
 *      Success - 0
 */
int32_t displayPortGeneralCSR (void)
{
    Uint8 hostDev;
    Uint8 masterEn;
    Uint8 disc;
	CSL_SRIO_GetPortGeneralCSR (hSrio_Direct,&hostDev,&masterEn,&disc);
	if (hostDev == 1)
		System_printf ("Debug:   SRIO is a Host Device.\n");
	else
		System_printf ("Debug:   SRIO is an Agent or Slave device.\n");

	if (masterEn == 1)
		System_printf ("Debug:   SRIO processing element can issue requests.\n");
	else
		System_printf ("Debug:   SRIO processing element cannot issue requests.\n");

	if (disc == 1)
		System_printf ("Debug:   SRIO device has been discovered by another processing element.\n");
	else
		System_printf ("Debug:   SRIO device has not been previously discovered.\n");

	return 0;

}

/**
 *  @b Description
 *  @n
 *      The function is used to return a text string translation of the control code specified.
 *
 *  @retval
 *      Pointer to the string containing the control code decode text
  */
char* controlDecodeText (uint32_t controlCode)
{
	static char		controlCodeString[20];
	switch (controlCode)
	{
		case CTRL_SUCCESS:
			sprintf (controlCodeString,"CTRL_SUCCESS");
			break;
		case CTRL_FAILURE:
			sprintf (controlCodeString,"CTRL_FAILURE");
			break;
		case CTRL_TIMEOUT:
			sprintf (controlCodeString,"CTRL_TIMEOUT");
			break;
		case CTRL_READY:
			sprintf (controlCodeString,"CTRL_READY");
			break;
		case CTRL_RESET:
			sprintf (controlCodeString,"CTRL_RESET");
			break;
		case CTRL_NEXT:
			sprintf (controlCodeString,"CTRL_NEXT");
			break;
		case CTRL_CONTINUE:
			sprintf (controlCodeString,"CTRL_CONTINUE");
			break;
		case CTRL_STOP:
			sprintf (controlCodeString,"CTRL_STOP");
			break;
		default:
			sprintf (controlCodeString,"0x%X",controlCode);
			break;
	}
	return (controlCodeString);
}
