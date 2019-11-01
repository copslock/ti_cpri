;
;  TEXAS INSTRUMENTS TEXT FILE LICENSE
;
;   Copyright (c) 2018-2019 Texas Instruments Incorporated
;
;  All rights reserved not granted herein.
;
;  Limited License.
;
;  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
;  license under copyrights and patents it now or hereafter owns or controls to
;  make, have made, use, import, offer to sell and sell ("Utilize") this software
;  subject to the terms herein.  With respect to the foregoing patent license,
;  such license is granted  solely to the extent that any such patent is necessary
;  to Utilize the software alone.  The patent license shall not apply to any
;  combinations which include this software, other than combinations with devices
;  manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
;
;  Redistributions must preserve existing copyright notices and reproduce this license
;  (including the above copyright notice and the disclaimer and (if applicable) source
;  code license limitations below) in the documentation and/or other materials provided
;  with the distribution.
;
;  Redistribution and use in binary form, without modification, are permitted provided
;  that the following conditions are met:
; 	No reverse engineering, decompilation, or disassembly of this software is
;   permitted with respect to any software provided in binary form.
; 	Any redistribution and use are licensed by TI for use only with TI Devices.
; 	Nothing shall obligate TI to provide you with source code for the software
;   licensed and provided to you in object code.
;
;  If software source code is provided to you, modification and redistribution of the
;  source code are permitted provided that the following conditions are met:
; 	Any redistribution and use of the source code, including any resulting derivative
;   works, are licensed by TI for use only with TI Devices.
; 	Any redistribution and use of any object code compiled from the source code
;   and any resulting derivative works, are licensed by TI for use only with TI Devices.
;
;  Neither the name of Texas Instruments Incorporated nor the names of its suppliers
;  may be used to endorse or promote products derived from this software without
;  specific prior written permission.
;
;  DISCLAIMER.
;
;  THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED
;  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
;  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S
;  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
;  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

;---------------------
; File:psiloop.h
;PSI LOOPBACK ROUTINES
;  (diagnostics)
; PSI LOOPBACK runs on
; BG on PRU, but looks
;  like a tx task
;   (e.g. will use
;    tx task registers)
; if this runs, no TX Will run
; and no RX should run since this is
;  grabing PSI widget for its use
;   always uses unit 0
;---------------------

;'tx' init for psi loopback
PSILOOP_TX_INIT	.macro 	 r_d, runit
	flip_tx_r10_r23
	mov	TxRegs.ds_flags, r_d.w2 ;descriptor2(flags, etc)
	mov	GRegs.tx.b.len, r_d.w0  ;save length we are expecting to transmit
	ldi	GRegs.tx.b.state, TX_S_LOOP
	PSILOOP_TX_POLL	runit, nd_lab?, eop_lab?
nd_lab?:
eop_lab?:
	flip_tx_r10_r23
 .endm

;poll routine for bg.  If dma present, it will process it
PSILOOP_TX_POLL	.macro 	 runit, no_data_label,eop_label
	XFR2VBUS_POLL_READ runit
	qbbc	no_data_label, r18, 2

;have data ready
	qbge	tx_cont1a?, GRegs.tx.b.len, 64
;read the 64 bytes & write to PSI in 4 transactions
	XFR2VBUS_READ64_RESULT runit
	ldi32	r1, MD_DATA0
	PSI_WRITE
	LEBE2_5_swap_6_9
	PSI_WRITE
	LEBE2_9_swap_10_17
	LEBE2_5_swap_6_9
	PSI_WRITE
	LEBE2_5_swap_6_9
	PSI_WRITE
	sub	GRegs.tx.b.len, GRegs.tx.b.len, 64
	jmp	tx_done?

tx_cont1a?:   ;eop, (again always using unit a)
	XFR2VBUS_CANCEL_READ_AUTO_64_CMD (runit)
	XFR2VBUS_READ64_RESULT (runit)
	qble	mt16?, GRegs.tx.b.len, 16
;just have 16bytes or less
	ldi32	r1, MD_DATA1
	add	r0.b0, GRegs.tx.b.len, 4
	PSI_WRITE_N
	jmp	tx_done2?
mt16?:
	qble	mt32?, GRegs.tx.b.len, 32
;just have 32bytes or less
	ldi32	r0, MD_DATA0
	PSI_WRITE
	LEBE2_5_swap_6_9
	ldi32	r1, MD_DATA1
	sub	GRegs.tx.b.len, GRegs.tx.b.len,16
	add	r0.b0, GRegs.tx.b.len, 4
	PSI_WRITE_N
	jmp	tx_done2?
mt32?:
	qble	mt48?, GRegs.tx.b.len, 48
;just have 48bytes or less
	ldi32	r0, MD_DATA0
	PSI_WRITE
	LEBE2_5_swap_6_9
	PSI_WRITE
	LEBE2_9_swap_10_17
	LEBE2_5_swap_6_9
	ldi32	r1, MD_DATA1
	sub	GRegs.tx.b.len, GRegs.tx.b.len,32
	add	r0.b0, GRegs.tx.b.len, 4
	PSI_WRITE_N
	jmp	tx_done2?
mt48?:
;have more than 48
	ldi32	r0, MD_DATA0
	PSI_WRITE
	LEBE2_5_swap_6_9
	PSI_WRITE
	LEBE2_9_swap_10_17
	LEBE2_5_swap_6_9
	PSI_WRITE
	LEBE2_5_swap_6_9
	ldi32	r1, MD_DATA1
	sub	GRegs.tx.b.len, GRegs.tx.b.len,48
	add	r0.b0, GRegs.tx.b.len, 4
	PSI_WRITE_N
tx_done2?:
;say we are done
	SPIN_SET_LOCK_LOC PRU_RTU_EOD_P_FLAG
	SPIN_CLR_LOCK_LOC PRU_RTU_EOD_P_FLAG
;set tx state
	ldi	GRegs.tx.b.state, TX_S_IDLE
	jmp eop_label
tx_done?:
 .endm
