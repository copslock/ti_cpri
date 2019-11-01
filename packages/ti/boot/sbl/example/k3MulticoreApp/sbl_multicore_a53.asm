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

#ifdef BUILD_MPU1_0
	.section   .sbl_mpu_1_0_resetvector, "ax"
#endif

#ifdef BUILD_MPU1_1
	.section   .sbl_mpu_1_1_resetvector, "ax"
#endif

#ifdef BUILD_MPU2_0
	.section   .sbl_mpu_2_0_resetvector, "ax"
#endif

#ifdef BUILD_MPU2_1
	.section   .sbl_mpu_2_1_resetvector, "ax"
#endif

	.global sblTestmain

	.global _sblTestResetVectors
	.func _sblTestResetVectors
_sblTestResetVectors:
    ldr     x1, =_sblTestStackBase
    and     x1, x1, #(~0xf)
    mov     sp, x1

    ldr     x1, =sblTestmain
    blr     x1
    wfi
    add     x0, x0, #0x1
    b       .

_sblTestStackTop:
	.skip 512
_sblTestStackBase:

	.endfunc

