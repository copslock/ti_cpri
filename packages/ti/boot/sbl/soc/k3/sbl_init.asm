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
	.global _sblResetVectors

_sblResetVectors:
	.asmfunc
	LDR pc, sblEntry ; Reset
	B _sblLoopForever ; Undefined Instruction
	B _sblLoopForever ; SVC call
	B _sblLoopForever ; Prefetch abort
	B _sblLoopForever ; Data abort
	B _sblLoopForever ; Hypervisor
	B _sblLoopForever ; IRQ
	B _sblLoopForever ; FIQ

sblEntry	.long _sblEntry
	.endasmfunc

;****************************************************************************
; SBL Entry
;****************************************************************************
	.def	_sblEntry
	.ref	_c_int00
	.ref 	SBL_init_profile
	.ref	_sblTcmEn

_c_int00_addr		.long _c_int00
SBL_init_profile_addr	.long SBL_init_profile
_sblTcmEnAddr		.long _sblTcmEn

_sblEntry:
	.asmfunc

	MRC	p15, #0, r1, c0, c0, #5
	BFC	r1, #8, #24
	CMP	r1, #0
	BNE	_sblLoopForever

	ADR	r1, _sblTestStackBase
	BIC	r1, r1, #0xf
	MOV	sp, r1

	LDR	r1, SBL_init_profile_addr
	BLX	r1

	LDR	r1, _sblTcmEnAddr
	BLX	r1

	LDR	r1, _c_int00_addr
	BLX	r1

_sblLoopForever:
	WFI
	B	_sblLoopForever

	.endasmfunc

_sblTestStackTop:
	.space 64
_sblTestStackBase:

