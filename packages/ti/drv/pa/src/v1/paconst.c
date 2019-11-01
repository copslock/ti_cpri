/******************************************************************************
 * FILE PURPOSE:  Constant Register values for PASS
 ******************************************************************************
 * FILE NAME:   paconst.c  
 *
 * DESCRIPTION: Constant Registers for PA 
 *
 * FUNCTION               DESCRIPTION
 * ========               ===========
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2016
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

#include "pa.h"

#define PASS_NUM_PDSPS 	(15U)
/****************************************************************************
 * DATA PURPOSE: PDSP Constant Registers
 ****************************************************************************
 * DESCRIPTION: Specify the pre-defined PDSP constant registers (c0-c31)
 ****************************************************************************/
 const uint32_t pap_pdsp_const_reg_map[PASS_NUM_PDSPS][32] = 
 {
    /* Ingress0 PDSP0: Classify1 */
    {
        0xFFF84000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000000,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved */
        0xFFF80000,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: Time Accumulation Constants and EOAM Exception table */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80C00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81000,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF83F00,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF980000,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Ingress0 PDSP1: Classify1 */
    {
        0xFFF88000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000010,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF14000,         /* C6: CDE new packet input region */
        0xFFF10100,         /* C7: CDE new packet output region */
        0xFFF10200,         /* C8: CDE held packet region */
        0xFFF18800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF19000,         /* C10: LUT1 Registers */
        0xFFF19400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0xFFF80100,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: Time Accumulation Constants and EOAM Exception table */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80D00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81100,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF83F20,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF980040,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
   /* Ingress1 PDSP0: Classify1 */
    {
        0xFFF84000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000020,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0xFFF80000,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: IP Traffic Flow */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80C00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81000,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF83F00,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF980080,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Ingress1 PDSP1: Classify1 */
    {
        0xFFF88000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000030,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF14000,         /* C6: CDE new packet input region */
        0xFFF10100,         /* C7: CDE new packet output region */
        0xFFF10200,         /* C8: CDE held packet region */
        0xFFF18800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF19000,         /* C10: LUT1 Registers */
        0xFFF19400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0xFFF80100,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: IP Traffic Flow */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80D00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81100,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF83F20,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF9800C0,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
   /* Ingress2 PDSP0: Classify1 */
    {
        0xFFF82000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000040,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0xFFF80000,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: IP Traffic Flow */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80C00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81000,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF81F00,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF980100,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
   /* Ingress3 PDSP0: Classify1 */
    {
        0xFFF82000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000050,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0xFFF80000,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: IP Traffic Flow */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80C00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81000,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF81F00,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF980140,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Ingress4 PDSP0: Classify1 */
    {
        0xFFF84000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000060,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0xFFF80000,         /* C14: PDSP Context */
        0x00000000,         /* C15: Reserved*/
        0xFFF80400,         /* C16: IP Traffic Flow */
        0xFFF80800,         /* C17: IP Reassembly Control Block */
        0xFFF80C00,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81000,         /* C21: Classify1 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF83F00,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF980180,         /* C25: User Stats CB and FIFO  (Global address of Post Cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Ingress4 PDSP1: Classify2 */
    {
        0xFFF88000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000070,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF14000,         /* C6: CDE new packet input region */
        0xFFF10100,         /* C7: CDE new packet output region */
        0xFFF10200,         /* C8: CDE held packet region */
        0xFFF18800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF19000,         /* C10: LUT1 Registers */
        0xFFF19400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0x00000000,         /* C14: Reserved*/
        0x00000000,         /* C15: Reserved*/
        0xFFF81800,         /* C16: Custom2 Info */
        0x00000000,         /* C17: Reserved*/
        0x00000000,         /* C18: Reserved*/
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF81100,         /* C21: Classify2 Parsing Call Table */
        0x00000000,         /* C22: Reserved */
        0xFFF83F20,         /* C23: PDSP Info */
        0xFFF81400,         /* C24: Next Route Global address table */
        0xFF9801C0,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Post PDSP0: Modifier */
    {
        0xFFF80000,         /* C0: User Stats FIFO Base */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000080,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0x00000000,         /* C14: Reserved*/
        0xFFF80400,         /* C15: User Stats Control Block */
        0xFFF80800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Command Set Table */
        0xFFF81800,         /* C18: Multi-route table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF82800,         /* C21: CRC Verify FIFO */
        0xFFF82900,         /* C22: Split Context FIFO */
        0xFFF83F00,         /* C23: PDSP Info*/
        0x00000000,         /* C24: Reserved */
        0xFFF80200,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Post PDSP1: Modifier */
    {
        0xFFF80000,         /* C0: User Stats FIFO Base */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF000090,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF14000,         /* C6: CDE new packet input region */
        0xFFF10100,         /* C7: CDE new packet output region */
        0xFFF10200,         /* C8: CDE held packet region */
        0xFFF18800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF19000,         /* C10: LUT1 Registers */
        0xFFF19400,         /* C11: LUT2 Registers */
        0x00000000,         /* C12: Reserved */
        0x00000000,         /* C13: Reserved*/
        0x00000000,         /* C14: Reserved*/
        0xFFF80400,         /* C15: User Stats Control Block */
        0xFFF80800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Command Set Table */
        0xFFF82000,         /* C18: Multi-route table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF82800,         /* C21: CRC Verify FIFO */
        0xFFF82900,         /* C22: Split Context FIFO */
        0xFFF83F20,         /* C23: PDSP Info*/
        0x00000000,         /* C24: Reserved */
        0xFFF80240,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0x00000000,         /* C26: Reserved*/
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Egress0 PDSP0: Flow Cache */
    {
        0xFFF82000,         /* C0: LUT1 Info */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF0000A0,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0xFFF84000,         /* C12: Egress Flow Record0 */
        0xFFF88000,         /* C13: Egress Flow Record1 */
        0xFFF80000,         /* C14: PDSP Context */
        0xFF980400,         /* C15: User Stats Control Block */
        0xFF980800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Modify Context */
        0xFFF80200,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF80500,         /* C21: Parse table */
        0x00000000,         /* C22: Reserved */
        0xFFF81F00,         /* C23: PDSP Info */
        0xFFF80A00,         /* C24: Temporary Buffer */
        0xFF980280,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0xFF020400,         /* C26: Eflow Exception route */
        0x00000000,         /* C27: Reserved*/
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Egress0 PDSP1: Flow Cache */
    {
        0x00000000,         /* C0: Reserved */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF0000B0,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF14000,         /* C6: CDE new packet input region */
        0xFFF10100,         /* C7: CDE new packet output region */
        0xFFF10200,         /* C8: CDE held packet region */
        0xFFF18800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF19000,         /* C10: LUT1 Registers */
        0xFFF19400,         /* C11: LUT2 Registers */
        0xFFF84000,         /* C12: Egress Flow Record0 */
        0xFFF88000,         /* C13: Egress Flow Record1 */
        0x00000000,         /* C14: Reserved */
        0xFF980400,         /* C15: User Stats Control Block */
        0xFF980800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Modify Context */
        0xFFF80300,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF80600,         /* C21: Parse table */
        0x00000000,         /* C22: Reserved */
        0xFFF81F20,         /* C23: PDSP Info */
        0xFFF80A00,         /* C24: Temporary Buffer */
        0xFF9802C0,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0xFF020400,         /* C26: Eflow Exception route */
        0xFFF80800,         /* C27: Command Buffer */
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Egress0 PDSP2: Flow Cache */
    {
        0x00000000,         /* C0: Reserved */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF0000C0,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF24000,         /* C6: CDE new packet input region */
        0xFFF20100,         /* C7: CDE new packet output region */
        0xFFF20200,         /* C8: CDE held packet region */
        0xFFF28800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF29000,         /* C10: LUT1 Registers */
        0xFFF29400,         /* C11: LUT2 Registers */
        0xFFF84000,         /* C12: Egress Flow Record0 */
        0xFFF88000,         /* C13: Egress Flow Record1 */
        0x00000000,         /* C14: Reserved */
        0xFF980400,         /* C15: User Stats Control Block */
        0xFF980800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Modify Context */
        0xFFF80400,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Exception Routes */
        0xFFF80700,         /* C21: Parse table */
        0x00000000,         /* C22: Reserved */
        0xFFF81F40,         /* C23: PDSP Info */
        0xFFF80A00,         /* C24: Temporary Buffer */
        0xFF980300,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0xFF020400,         /* C26: Eflow Exception route */
        0xFFF80900,         /* C27: Command Buffer */
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
    
    /* Egress1 PDSP0: Flow Cache */
    {
        0x00000000,         /* C0: Reserved */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF0000D0,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0xFFF81000,         /* C12: Egress Flow Record2 */
        0x00000000,         /* C13: Reserved*/
        0xFFF80000,         /* C14: PDSP Context */
        0xFF980400,         /* C15: User Stats Control Block */
        0xFF980800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Modify Context */
        0xFFF80200,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Egress Exception Routes */
        0xFFF80500,         /* C21: Parse table */
        0x00000000,         /* C22: Reserved */
        0xFFF80F00,         /* C23: PDSP Info */
        0xFFF80A00,         /* C24: Temporary Buffer */
        0xFF980340,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0xFF020400,         /* C26: Eflow Exception route */
        0xFFF80800,         /* C27: Command Buffer */
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    },
  
    /* Egress2 PDSP0: Flow Cache */
    {
        0x00000000,         /* C0: Reserved */    
        0xFFF80000,         /* C1: Loacl SRAM */
        0xFF020000,         /* C2: Global SRAM */
        0xFF408800,         /* C3: System Timer (PDSP0 Timer) */
        0xFF0000E0,         /* C4: MailBox */
        0x00000000,         /* C5: Reserved*/
        0xFFF04000,         /* C6: CDE new packet input region */
        0xFFF00100,         /* C7: CDE new packet output region */
        0xFFF00200,         /* C8: CDE held packet region */
        0xFFF08800,         /* C9: PDSP Timer (PDSP specific) */
        0xFFF09000,         /* C10: LUT1 Registers */
        0xFFF09400,         /* C11: LUT2 Registers */
        0xFFF81000,         /* C12: Egress Flow Record3 */
        0x00000000,         /* C13: Reserved*/
        0xFFF80000,         /* C14: PDSP Context */
        0xFF980400,         /* C15: User Stats Control Block */
        0xFF980800,         /* C16: User Stats  */
        0xFFF81000,         /* C17: Modify Context */
        0xFFF80200,         /* C18: IP Protocol Table */
        0xFF020000,         /* C19: Custom LUT1 and global configuration */
        0xFF020200,         /* C20: Egress Exception Routes */
        0xFFF80500,         /* C21: Parse table */
        0x00000000,         /* C22: Reserved */
        0xFFF80F00,         /* C23: PDSP Info */
        0xFFF80A00,         /* C24: Temporary Buffer */
        0xFF980380,         /* C25: User Stats CB and FIFO  (Global address of Post cluster) */
        0xFF020400,         /* C26: Eflow Exception route */
        0xFFF80800,         /* C27: Command Buffer */
        0xFF020500,         /* C28: Port (Interface-based) configurations */
        0x00000000,         /* C29: Reserved*/
        0x00000000,         /* C30: Reserved*/
        0x00000000          /* C31: Reserved*/
    }
    
};
