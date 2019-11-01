;******************************************************************************
;*                                                                            *
;* Copyright (c) 2018-2019 Texas Instruments Incorporated                     *
;* http://www.ti.com/                                                         *
;*                                                                            *
;*  Redistribution and  use in source  and binary forms, with  or without     *
;*  modification,  are permitted provided  that the  following conditions     *
;*  are met:                                                                  *
;*                                                                            *
;*     Redistributions  of source  code must  retain the  above copyright     *
;*     notice, this list of conditions and the following disclaimer.          *
;*                                                                            *
;*     Redistributions in binary form  must reproduce the above copyright     *
;*     notice, this  list of conditions  and the following  disclaimer in     *
;*     the  documentation  and/or   other  materials  provided  with  the     *
;*     distribution.                                                          *
;*                                                                            *
;*     Neither the  name of Texas Instruments Incorporated  nor the names     *
;*     of its  contributors may  be used to  endorse or  promote products     *
;*     derived  from   this  software  without   specific  prior  written     *
;*     permission.                                                            *
;*                                                                            *
;*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS     *
;*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT     *
;*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR     *
;*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT     *
;*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
;*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT     *
;*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
;*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
;*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT     *
;*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
;*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
;*                                                                            *
;******************************************************************************
;****************************************************************************
; Setup Reset Vectors
;****************************************************************************
    .arm

    .sect   ".rstvectors"
    .global _sciclientTestResetVectors

_sciclientTestResetVectors:
    .asmfunc
    LDR pc, sciclientTestEntry ; Reset
    B _sciclientTestLoopForever ; Undefined Instruction
    B _sciclientTestLoopForever ; SVC call
    B _sciclientTestLoopForever ; Prefetch abort
    B _sciclientTestLoopForever ; Data abort
    B _sciclientTestLoopForever ; Hypervisor
    B _sciclientTestLoopForever ; IRQ
    B _sciclientTestLoopForever ; FIQ

sciclientTestEntry    .long _sciclientTestEntry
    .endasmfunc

;****************************************************************************
; sciclientTest Entry
;****************************************************************************
    .def    _sciclientTestEntry
    .ref    _c_int00

_c_int00_addr        .long _c_int00
_sciclientTestAtcmEnAddr        .long _sciclientTestAtcmEn

_sciclientTestEntry:
    .asmfunc

    LDR   r0, _atcm_start                 ; Start address of ATCM
    LDR   r1, _atcm_end                   ; End address of ATCM
    MOV   r2, #0
Loop:
    STR   r2, [r0], #4                   ; Clear one word in ATCM
    CMP   r0, r1
    BLE   Loop                           ; Clear till ATCM end

    LDR   r0, _btcm_start                 ; Start address of BTCM
    LDR   r1, _btcm_end                   ; End address of BTCM
    MOV   r2, #0
Loop2:
    STR   r2, [r0], #4                   ; Clear one word in BTCM
    CMP   r0, r1
    BLE   Loop2                           ; Clear till BTCM end

    ADR    r1, _sciclientTestTestStackBase
    BIC    r1, r1, #0xf
    MOV    sp, r1

    LDR    r1, _sciclientTestAtcmEnAddr
    BLX    r1

    LDR    r1, _c_int00_addr
    BLX    r1

_sciclientTestLoopForever:
    WFI
    B    _sciclientTestLoopForever

    .endasmfunc

_atcm_start:
    .word 0x0
_atcm_end:
    .word 0x7FFC
_btcm_start:
    .word 0x41010000
_btcm_end:
    .word 0x41017FFC

_sciclientTestTestStackTop:
    .space 32
_sciclientTestTestStackBase:

;****************************************************************************
; sciclientTest Read ATCM Region Register
;****************************************************************************
    .sect   ".text"
    .global _sciclientTestAtcmEn

_sciclientTestAtcmEn:
    .asmfunc
    MRC     p15, #0, r0, c9, c1, #1
    ORR     r0, r0, #0x1
    MCR     p15, #0, r0, c9, c1, #1
    BX      lr
    .endasmfunc
    

