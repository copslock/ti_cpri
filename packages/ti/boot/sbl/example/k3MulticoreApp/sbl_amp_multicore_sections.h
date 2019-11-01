/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if defined (SOC_AM65XX)
#define SBL_AMP_TEST_NUM_BOOT_CORES (6)
#endif

#if defined (SOC_J721E)
#define SBL_AMP_TEST_NUM_BOOT_CORES (8)
// Currently the amp test does not have an
// app for c66x_1, c66x_2 and c7x
//#define SBL_AMP_TEST_NUM_BOOT_CORES (11)
#endif

#define POKE_MEM_ADDR_MCU1_0 0x70011FFC
#define POKE_MEM_ADDR_MCU1_1 0x70013FFC
#define POKE_MEM_ADDR_MCU2_0 0x70015FFC
#define POKE_MEM_ADDR_MCU2_1 0x70017FFC
#define POKE_MEM_ADDR_MCU3_0 0x70019FFC
#define POKE_MEM_ADDR_MCU3_1 0x7001BFFC
#define POKE_MEM_ADDR_C66X_0 0x7001DFFC
#define POKE_MEM_ADDR_C66X_1 0x7001FFFC
#define POKE_MEM_ADDR_C7X_0  0x70021FFC
#define POKE_MEM_ADDR_C7X_1  0x70023FFC
#define POKE_MEM_ADDR_MPU1_0 0x70025FFC
#define POKE_MEM_ADDR_MPU1_1 0x70027FFC
#define POKE_MEM_ADDR_MPU2_0 0x70029FFC
#define POKE_MEM_ADDR_MPU2_1 0x7002BFFC

#ifdef BUILD_MCU1_0
    #define CORE_NAME "MCU1_0"
    #define BOOT_DELAY (0x2A0000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MCU1_0)
    #pragma SET_CODE_SECTION(".sbl_mcu_1_0_resetvector")
#endif

#ifdef BUILD_MCU1_1
    #define CORE_NAME "MCU1_1"
    #define BOOT_DELAY (0x80000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MCU1_1)
    #pragma SET_CODE_SECTION(".sbl_mcu_1_1_resetvector")
#endif

#ifdef BUILD_MCU2_0
    #define CORE_NAME "MCU2_0"
    #define BOOT_DELAY (0xA0000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MCU2_0)
    #pragma SET_CODE_SECTION(".sbl_mcu_2_0_resetvector")
#endif

#ifdef BUILD_MCU2_1
    #define CORE_NAME "MCU2_1"
    #define BOOT_DELAY (0xC0000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MCU2_1)
    #pragma SET_CODE_SECTION(".sbl_mcu_2_1_resetvector")
#endif

#ifdef BUILD_MCU3_0
    #define CORE_NAME "MCU3_0"
    #define BOOT_DELAY (0xE0000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MCU3_0)
    #pragma SET_CODE_SECTION(".sbl_mcu_3_0_resetvector")
#endif

#ifdef BUILD_MCU3_1
    #define CORE_NAME "MCU3_1"
    #define BOOT_DELAY (0x100000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MCU3_1)
    #pragma SET_CODE_SECTION(".sbl_mcu_3_1_resetvector")
#endif

#ifdef BUILD_C66X_1
    #define CORE_NAME "C66X_0"
    #define BOOT_DELAY (0x120000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_C66X_0)
    #pragma SET_CODE_SECTION(".sbl_c66x_0_resetvector")
#endif

#ifdef BUILD_C66X_2
    #define CORE_NAME "C66X_1"
    #define BOOT_DELAY (0x140000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_C66X_1)
    #pragma SET_CODE_SECTION(".sbl_c66x_1_resetvector")
#endif

#ifdef BUILD_C7X_1
    #define CORE_NAME "C7X_0"
    #define BOOT_DELAY (0x160000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_C7X_0)
    #pragma SET_CODE_SECTION(".sbl_c7x_0_resetvector")
#endif

#ifdef BUILD_C7X_2
    #define CORE_NAME "C7X_1"
    #define BOOT_DELAY (0x180000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_C7X_1)
    #pragma SET_CODE_SECTION(".sbl_c7x_1_resetvector")
#endif

#ifdef BUILD_MPU1_0
    #define CORE_NAME "MPU1_0"
    #define BOOT_DELAY (0x1)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MPU1_0)
    int sblTestmain(void)  __attribute__((section(".sbl_mpu_1_0_resetvector")));
#endif

#ifdef BUILD_MPU1_1
    #define CORE_NAME "MPU1_1"
    #define BOOT_DELAY (0x20000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MPU1_1)
    int sblTestmain(void)  __attribute__((section(".sbl_mpu_1_1_resetvector")));
#endif

#ifdef BUILD_MPU2_0
    #define CORE_NAME "MPU2_0"
    #define BOOT_DELAY (0x40000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MPU2_0)
    int sblTestmain(void)  __attribute__((section(".sbl_mpu_2_0_resetvector")));
#endif

#ifdef BUILD_MPU2_1
    #define CORE_NAME "MPU2_1"
    #define BOOT_DELAY (0x60000)
    #define POKE_MEM_ADDR (POKE_MEM_ADDR_MPU2_1)
    int sblTestmain(void)  __attribute__((section(".sbl_mpu_2_1_resetvector")));
#endif

void sbl_puts(char *str);


