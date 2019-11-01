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

;Basic Ethernet rx/tx firmware==> TASK MANAGER + S&F MODE {{{1
;/ modified by DAL
;TXL2 version
; 'new' rxl2  mode, so eof handling is different
; defines for config
; RGMII: set up for RGMII mode
;  MII : set up for MII mode (RX ok, TX not tested)
; SLICE0 or SLICE1 must be defined  (but not both)
; WAIT_FOR_DEBUGGER:  wait for debugger to attach
; VLAN_ENABLED
; PSILOOP
DATA_ONLY	.set	1 ;control path moved to RTU

; sanity check ;{{{1
 .if $isdefed("SLICE0") & $isdefed("SLICE1")
	cant	have SLICE0 and SLICE1
 .endif
 .if !$isdefed("SLICE0") & !$isdefed("SLICE1")
	must	have slice0 or slice1 defined
 .endif

; includes {{{1
 .include "regs.h"
 .include "portq.h"
 .include "reg_alias.h"
 .include "smem.h"
 .include "bsram_pru.h"
 .include "spin.h"
 .include "xfr2vbus_widget.h"
 .include "xfr2psi_widget.h"
 .include "basicio.h"
 .include "tm.h"
 .include "rx.h"
 .include "tx.h"
 .include "filter.h"
 .include "lebe.h"
 .include "ipc.h"
 .include "psiloop.h"
 .include "iep.h"
 .include "psisandf.h"

loop_here	.macro
here?:	jmp	here?
 .endm

; slice0 vs slice1 {{{1
 .if $isdefed("SLICE0")
	.asg	MII_RXCFG0_ADDR, MII_RXCFGn_ADDR
	.asg	0x8, GPCFGn_REG
	.asg	FDB_XID_PORT0_RES, FDB_XID_PORTn_RES
	.asg	MII_PRE_CNT0, MII_PRE_CNTn
 .else
	.asg	MII_RXCFG1_ADDR, MII_RXCFGn_ADDR
	.asg	0xc, GPCFGn_REG
	.asg	FDB_XID_PORT1_RES, FDB_XID_PORTn_RES
	.asg	MII_PRE_CNT1, MII_PRE_CNTn
 .endif
;slice0 controls tx1 if switch, tx0 otherwise
;slice1 controls tx0 if switch, tx1 otherwise
 .if $isdefed("SLICE0")
	.asg	MII_TXCFG0_ADDR, MII_TXCFGn_ADDR
	.asg	MII_TXIPG0_ADDR, MII_TXIPGn_ADDR
 .else
	.asg	MII_TXCFG1_ADDR, MII_TXCFGn_ADDR
	.asg	MII_TXIPG1_ADDR, MII_TXIPGn_ADDR
 .endif

; Enable RX L2 pru0 & rx
enable_rx_l2	.macro
	ldi32	r28, MII_RXCFGn_ADDR
	lbbo	&r11, r28, 0, 4
	set	r11.t4 ;rx l2 enable
	set	r11.t0 ;rx enable
	set	r11.t9 ;rx_eof_sclr_dis0
	set	r11.t1 ;make tx eof visible in r31, bit16
	sbbo	&r11, r28, 0, 4
 .endm

long_preamble_firewall_disable	.macro
	ldi32	r28, MII_PRE_CNTn
	lbbo	&r11, r28, 0, 4
	and	r11.b0, r11.b0, 0x0f
	sbbo	&r11, r28, 0, 4
 .endm

gpcfg_reg_config	.macro
	ldi32	r28, PRUSS1_CFG_PRU_OFFSET
	lbbo	&r11, r28, GPCFGn_REG, 4
	set	r11.t1
	set	r11.t0
	set	r11.t27
	sbbo	&r11, r28, GPCFGn_REG, 4
 .endm

mii_tx_config	.macro
	ldi32	r28, MII_TXCFGn_ADDR
	lbbo	&r11, r28, 0, 4
	set	r11.t0	;enable txl1
	set	r11.t11	;tx_32_mode en
	set	r11.t1	;tx_auto_preamble
;set mux
 .if $isdefed("SLICE0")
	clr	r11.t8
 .else
	set	r11.t8
 .endif
	ldi	r11.b2, 0 ;set tx start delay to 0!!!
	clr	r11.t25
	clr	r11.t24
	sbbo	&r11, r28, 0, 4
 .endm

set_tx_ipg	.macro
;change ipg to 88 ns  (22  250mhz clocks)
	ldi32	r28, MII_TXIPGn_ADDR
	lbbo	&r11, r28, 0, 4
	ldi	r11, 0x16 ; min IPG for testing (CCLINK)
	sbbo	&r11, r28, 0, 4
 .endm

icss_g_config	.macro
; **for now let slice0 do txl2 & port mode config (ICSS_G register) for both sides {{{2
;TX Config TXl2 - icss_g register
	ldi32	r28, ICSS_G
	lbbo	&r11, r28, 0, 4
	set	r11.t1
	set	r11.t0
	set	r11.t2 ;rx_l2_g_en

 .if $isdefed("RGMII")
	set	r11.t3
	clr	r11.t4
	set	r11.t5
	clr	r11.t6
 .endif
 .if $isdefed("MII")
	clr	r11.t3
	clr	r11.t4
	clr	r11.t5
	clr	r11.t6
 .endif
	sbbo	&r11, r28, 0, 4
 .endm

wait4debugger	.macro
 .if $isdefed("WAIT_FOR_DEBUGGER")
	ldi32	r11, 0
$1:	qbeq	$1, r11, 0
 .endif
 .endm

; Code starts {{{1
 .retain     ; Required forbuilding	.out with assembly file
 .retainrefs ; Required forbuilding	.out with assembly file
 .sect    ".text:Start"
 .global  Start

Start:
	TM_DISABLE
	zero	&r0, 124
	P2P_IPC_ZAP	;zap IPC area
	xout	XFR2VBUS_XID_READ0, &r18, 4 ;disable xfr2vbus autoread mode
	xout	XFR2VBUS_XID_READ1, &r18, 4 ;

;Initialization:  set up RX & TX MII stuff.
	enable_rx_l2
	long_preamble_firewall_disable
	gpcfg_reg_config	; 0x0800_0003
	mii_tx_config
	set_tx_ipg
	icss_g_config

	wait4debugger

	set	r31, r31, 18 ;RX reset
;setup BSRAM
	BSRAM_ZERO_BANK	r1

;SETUP portQ for to-host traffic, in order to do sandf PSI
	PSIQ_CREATE

;set up PSI INFO, CONTROL, STATUS TEMPLATES
 .if $isdefed("SLICE0")
	PSI_SETUP_INFO	PSI_INFO_SLOT, 1
 .else
	PSI_SETUP_INFO	PSI_INFO_SLOT, 2
 .endif
	PSI_SETUP_STATUS	PSI_STATUS_SLOT

;VA we should start here
	ldi	GRegs.pkt_cnt.x, 0

;START RX/TX of 1 packet burst ; {{{1
RX:
	RX_TASK_INIT
;setup taskmanager
	TM_PRU_CONFIG	RX_SOF, RX_B0, RX_B1, RX_BN, RX_EOF, TX_NOP, TX_NOP, TX_FIFO, TX_EOF, RXTX_ERR
	TM_ENABLE
	set	r31.t18 ;RX reset
	ldi	r18, 0
	xout	40, &r18, 4 ;set tx fifo mode

;read in global state
	ldi	GRegs.pkt_cnt.x, 0
	ldi	GRegs.tx.b.state, 0
	ldi	GRegs.rx.x, 0
	ldi	GRegs.snf.b.wr_cur, MINPS
	ldi	GRegs.snf.b.rd_cur, MINPS
	ldi	r30, 1522  ;todo - make a parameter
	ldi32	r10, FW_CONFIG
	lbbo	&r2, r10, CFG_N_BURST, 4

; set pru ready status
	ldi32	r0, PRU_READY
	sbbo	&r0, r10, CFG_STATUS, 4
; wait rtu ready
	ldi32	r1, RTU_READY
wait_rtu_ready:
	lbbo	&r0, r10, CFG_RTU_STATUS, 4
	qbne	wait_rtu_ready, r0, r1
; let's go

;-------------------------------------------------------------------
;BG TASK:     r24-r29  are global ;{{{1
;-------------------------------------------------------------------------
	zero	&r18, 24
	mov	BgRegs.borg_limit, r2 ; GS_NBURST ;

;================================
; BG LOOP:  stay here until we see
;            GRegs.pkt_cnt.w.tx == BgRegs.borg_limit if in borg mode
;            until cmd cancel seen
;=================================
bg_loop:
	add	BgRegs.bg_cnt, BgRegs.bg_cnt, 1 ;loop count
;can we exit?
	qbeq	skip_chk, BgRegs.borg_limit, 0
	qbeq	bg_done, GRegs.pkt_cnt.w.tx, BgRegs.borg_limit ; done
skip_chk:
; if RTU started shutdown process - disable TM and loop forever
	ldi32	r0, FW_CONFIG
	ldi32	r1, RTU_STARTED_SHUTDOWN
	lbbo	&r9, r0, CFG_RTU_STATUS, 4
	qbne	skip_chk01, r9, r1
;disable xfr2vbus autoread mode
	ldi32	r18, 0
	xout	XFR2VBUS_XID_READ0, &r18, 4
	xout	XFR2VBUS_XID_READ1, &r18, 4

	PSI_ABORT
;	if we are here, we can place debug error code somewere in the SMEM
	TM_DISABLE
	ldi32	r1, PRU_STOPPED
	sbbo	&r1, r0, CFG_STATUS, 4
	loop_here

skip_chk01:
;-----------------------
;schedule TX2HOST?
;----------------------
; do nothing if widget is full!!
	xin	XID_PSI_S, &r1,8
	qbbc	scheduler, r2,TB_WRITE
	qbeq	th_schedule0, BgRegs.psi2h_active, 0
; active 2host
	PSISANDF_TX	bg_to_host2, scheduler
bg_to_host2:  ;go again
	xin	XID_PSI_S, &r1,8
	qbbc	scheduler, r2,TB_WRITE
	PSISANDF_TX	scheduler, scheduler
	jmp	scheduler  ;just in case
th_schedule0:
	qbeq scheduler, GRegs.psiq.b.num_elem, 0
; have new packet to send
	TM_DISABLE
	PSIQ_POP
	TM_ENABLE
;r2 = flow | len  r3 = starting read index
	PSISANDF_TX_INIT2	r2, r3

;-----------------------
;schedule TX2WIRE ?
;----------------------
scheduler:
	TM_DISABLE
	qbeq	bg_schedule0, GRegs.tx.b.state, TX_S_IDLE ;if tx state is idle, check IPC for new descriptor
	qbeq	bg_schedule1, GRegs.tx.b.state, TX_S_ERR ;if tx state is idle, check IPC for new descriptor
 .if $isdefed("PSILOOP")
	qbeq	bg_schedulePL, GRegs.tx.b.state, TX_S_LOOP ;if psi loopback
 .endif

;TX ACTIVE.
sched_done:
	TM_ENABLE
	jmp	bg_loop

bg_schedule0:
;non PSA case only check expected next dma
	qbbs	bg_chk1, GRegs.tx.b.flags, f_next_dma
;check dma0
	PRU_IPC_RX_CH0Q	sched_done, r2, XFR2VBUS_XID_READ0
; have new packet in r2= len-flags
 .if $isdefed("PSILOOP")
	PSILOOP_TX_INIT	r2, XFR2VBUS_XID_READ0
;don't pingpong with PSILOOP!
 .else
	TX_TASK_INIT2_shell	r2
	set	GRegs.tx.b.flags, GRegs.tx.b.flags, f_next_dma
 .endif
	TM_ENABLE
	jmp	bg_loop
bg_chk1:
;check 2nd dma
	PRU_IPC_RX_CH0Q	sched_done, r3, XFR2VBUS_XID_READ1
	TX_TASK_INIT2_shell	r3
	clr	GRegs.tx.b.flags, GRegs.tx.b.flags, f_next_dma
	TM_ENABLE
	jmp	bg_loop

bg_schedule1:
;error case (underflow)
;todo
	ldi	GRegs.tx.b.state, TX_S_IDLE
	TM_ENABLE
	jmp	bg_loop
 .if $isdefed("PSILOOP")
bg_schedulePL:
;psi loopback
	flip_tx_r10_r23
	PSILOOP_TX_POLL	XFR2VBUS_XID_READ0, psi_poll_done, psi_poll_done
psi_poll_done:
	flip_tx_r10_r23
	TM_ENABLE
	jmp	bg_loop
 .endif

;-------------------------------------
; done with packets.
;-------------------------------------
bg_done:

;save bg info:  bg loops, txstate, rxstate, (pkt counts)
	PAGE_RESTORE	BG_STATE, 32
	mov	r2, BgRegs.bg_cnt
	mov	r3, GRegs.tx.x 
	mov	r4, GRegs.rx.x
	mov	r5, GRegs.pkt_cnt.x
	PAGE_SAVEQ

;update result area (length)
	ldi32	r1, FW_CONFIG
	sbbo	&r2, r1, CFG_OUT, 20
	ldi32	r2, 0x10000001
	sbbo	&r2, r1, CFG_STATUS, 4
	loop_here

;-------------------------------------------------------------------------
;end BG task
;-------------------------------------------------------------------------

;---------------------------------------------------------------------
; TX_EOF  EvENT {{{1
;----------------------------------------------------------------------
TX_EOF:
	qbne	tx_underflow, GRegs.tx.b.state, TX_S_W_EOF
; TX TS processing
	flip_tx_r0_r23
	qbbc	no_tx_ts, TxRegs.ds_flags, 5 ; we don't need tx_ts
	GET_PKT_TX_TS	r2
	ldi32	r10, FW_CONFIG + TX_TS_BASE
	sbbo	&r2, r10, 0, 8
	SPIN_TOG_LOCK_LOC	PRU_RTU_TX_TS_READY
no_tx_ts:
	flip_tx_r0_r23

;-----------------------------
;Legit EOF. Restore registers
;-----------------------------
legit_tx_eof:
	flip_tx_r0_r23
	qbbs	teof_chk1, GRegs.tx.b.flags, f_next_dma
;check dma0
	PRU_IPC_RX_CH0Q	no_new_tx, r2, XFR2VBUS_XID_READ0
; have new packet in r2= len-flags
	TX_TASK_INIT2	r2
	set	GRegs.tx.b.flags, GRegs.tx.b.flags, f_next_dma 
	jmp	tx_eof_on_deck_done
teof_chk1:
;check 2nd dma
	PRU_IPC_RX_CH0Q	no_new_tx, r3, XFR2VBUS_XID_READ1
	TX_TASK_INIT2	r3
	clr	GRegs.tx.b.flags, GRegs.tx.b.flags, f_next_dma

;started next pkt, terminate task
tx_eof_on_deck_done:
	TM_YIELD
;these next 2 instructions are done in delayed branch fashion
	flip_tx_r0_r23
	add	GRegs.pkt_cnt.w.tx, GRegs.pkt_cnt.w.tx, 1
	loop_here

;---------------------------------------------
;eof w/ no new packet to TX
;---------------------------------------------
no_new_tx:
	ldi	GRegs.tx.b.state, TX_S_IDLE
	TM_YIELD
	add	GRegs.pkt_cnt.w.tx, GRegs.pkt_cnt.w.tx, 1
	flip_tx_r0_r23
	loop_here

;------exception cases------
;underflow case:
tx_underflow:
	ldi	GRegs.tx.b.state, TX_S_ERR
	TM_YIELD
	add	GRegs.pkt_cnt.w.tx, GRegs.pkt_cnt.w.tx, 1
	loop_here

;---------------------------------------------------------------------
;  ENd TX_EOF
;----------------------------------------------------------------------

;---------------------------------------------------------------------
; TX_FIFO  EVENT {{{1
;----------------------------------------------------------------------
TX_FIFO:
	qbeq	handle_portq, GRegs.tx.b.state, TX_S_ACTIVE
;ignore rest
	TM_YIELD
	loop_here

;----------------------
; FROM PORTQ CASE
;----------------------
handle_portq:
	flip_tx_r0_r23
;
;check to see if need to preempt!
; conditions:  preempt enabled on port and pkt is preemptible
;              and enuf bytes sent and  enuf bytes left and
;              (hold set or Express Frame waiting )
;if not preemptible pkt skip all
	qbbs	skip_preempt, TxRegs.ds_flags, 4 ; R_TX_D1.f_desc_express
	qbne	skip_preempt, TxRegs.pp_ppok, PPOK
	qbbs	do_preempt, GRegs.tx.b.flags, f_tx_hold
	qbbs	do_preempt, GRegs.tx.b.flags, f_tx_efq
	qbbc	skip_preempt, GRegs.tx.b.flags, f_tx_efqd
do_preempt:
	END_TX_MCRC	;send MCRC
	mov	TxRegs.stash_ds_flags, TxRegs.ds_flags
	mov	TxRegs.stash_tx_len, GRegs.tx.b.len

	set	GRegs.tx.b.flags, GRegs.tx.b.flags, f_tx_stash ;so we know
	ldi	GRegs.tx.b.state, TX_S_W_PEOF
	TM_YIELD
	flip_tx_r0_r23
	add	TxRegs.pp_count, TxRegs.pp_count, 1
	loop_here

skip_preempt:
;assume data is here
	TX_FILL_FIFO	XFR2VBUS_XID_READ0
	TM_YIELD
	flip_tx_r0_r23
	loop_here

;-------------------------------------------------------------------------
; End TX_FIFO  EVENT
;-------------------------------------------------------------------------

;-------------------------------------------------------------------------
;RXTX_ERR EVENT  ; {{{1
; assume rx issue.
; reset rxl2 fifo
; hopefully that cleans things up
;  need to see what else needs to be done
RXTX_ERR:
	flip_tx_r0_r23
	qbne	rx_err?, GRegs.tx.b.state, TX_S_ERR
	; wait DMA ir complete
	qbbs	$3, TxRegs.ds_flags, 4
$1:	xin	XFR2VBUS_XID_READ0, &r18, 4
	qbeq	$2, r18.w0, 0x5
	qbeq	$2, r18, 0
	qba	$1
$2:	nop
	XFR2VBUS_CANCEL_READ_AUTO_64_CMD XFR2VBUS_XID_READ0
	nop
	XFR2VBUS_READ64_RESULT XFR2VBUS_XID_READ0
	SPIN_SET_LOCK_LOC PRU_RTU_EOD_P_FLAG
	SPIN_CLR_LOCK_LOC PRU_RTU_EOD_P_FLAG
	jmp	$5
$3:	xin	XFR2VBUS_XID_READ1, &r18, 4
	qbeq	$4, r18.w0, 0x5
	qbeq	$4, r18, 0
	qba	$3
$4:	nop
	XFR2VBUS_CANCEL_READ_AUTO_64_CMD XFR2VBUS_XID_READ1
	nop
	XFR2VBUS_READ64_RESULT XFR2VBUS_XID_READ1
	SPIN_SET_LOCK_LOC PRU_RTU_EOD_E_FLAG
	SPIN_CLR_LOCK_LOC PRU_RTU_EOD_E_FLAG
$5:
	set	r31.t30; TX_RESET
	nop
	nop
	;set TX to TX_S_IDLE
	ldi	GRegs.tx.b.state, TX_S_IDLE
	qba	rxtx_err_exit
rx_err?
	ldi	r11.b3, 0x80
	xout	22, &r11, 4
	set	r31.t22  ; clear rx eof  (why isnt it bit 20?)
	ldi	r7, 1  ;indicate we drop
;for now assume it is the RXL1 overflowing for next frame
;todo: how to check to see if it is for this frame??
	ldi	GRegs.rx.b.state, RX_STATE_OVER0
rxtx_err_exit:
	TM_YIELD
	flip_tx_r0_r23
	loop_here

;-------------------------------------------------------------------------
; Dummy: TX_NOP {{{1
;-------------------------------------------------------------------------
TX_NOP:
	TM_YIELD
	loop_here
;----------------------------------------------------------------------
; Dummy: END TX_NOP
;-------------------------------------------------------------------------

;---------------------------------------------------------------
; RX TASK, State 1:  RX  SOF {{{1
;--------------------------------------------------------------
RX_SOF:
;turn off RX_SOF
	TM_YIELD
	loop_here
;---------------------------------------------------------------
; END RX TASK, State 1
;--------------------------------------------------------------

NBTR	.set	32
;---------------------------------------------------------------
; RX TASK, State 2: RX_B0 {{{1
;--------------------------------------------------------------
RX_B0:
	flip_rx_r0_r23

;BRING IN THE DATA
	xin	RXL2_BANK0, &r2, NBTR
	ldi	RxRegs.aux_flags, 1
	qbeq	rxb0_already_over, GRegs.rx.b.state, RX_STATE_OVER0

	ldi	GRegs.rx.x, 0x180 ;fresh state=1, (sof flag set)
	P_W32_S	rxb0_full  ;stash pkt bytes in bs slot but don't start

	jmp	rx_b0_done

;some error/exception cases handling here
rxb0_full:  ;psi fifo in bsram full
	P_W32_ABORT
	ldi	GRegs.rx.x, 0x7F80
	add	GRegs.snf.b.dbg_cnt, GRegs.snf.b.dbg_cnt, 1
	jmp	rx_b0_done

rxb0_already_over:
	ldi	GRegs.rx.x, 0x7F80
;todo: bump stats

rx_b0_done:
	TM_YIELD
	flip_rx_r0_r23
	add	GRegs.rx.b.pkt_len, GRegs.rx.b.pkt_len, 32 ;hopefully this gets executed!!
	loop_here
;---------------------------------------------------------------
; END RX TASK, State 2
;--------------------------------------------------------------

;---------------------------------------------------------------
; RX TASK, State 3: RX_B1 {{{1
;--------------------------------------------------------------
RX_B1:
	flip_rx_r0_r23
	qbeq	skip_b1, GRegs.rx.b.state, RX_STATE_DROP
;!!better be here!!
;r5.b0 = route info
;r5.b1 = fid
;r5.w2 = index
;r6 = buffer ptr for s&f
	ldi	r0.b1, 0
stall_loop:
	PRU_IPC_RX_CH1
	qbbs	got_ipc, r5.b0, f_rx_sof
	add	r0.b1, r0.b1, 1
	qbne	stall_loop, r0.b1, RB1_STALL_LIMIT
	ldi	GRegs.rx.b.state, RX_STATE_DROP ;stall limit reached
	jmp	rb1_ipc_done
got_ipc:
	mov	GRegs.rx.b.flags, r5.b0 ;forwarding info we got from RTU
	PRU_IPC_RX_CH1_CLRB7	;clear bit so we know we got it
	mov	RxRegs.pq_cur, r6 ;save buffer pointer (may not use)  ;=flow in MAC

rb1_ipc_done:
	xin	RXL2_BANK1, &r2, 32 ;bring in data (r2-r9)
	clr	RxRegs.aux_flags, RxRegs.aux_flags, f_fh
	set	RxRegs.aux_flags, RxRegs.aux_flags, f_b1_seen

	TM_DISABLE
	qbbc	b1_rx_path1a, GRegs.rx.b.flags, f_tohost
	P_W32	rxb1_full  ;stash pkt bytes in bs slot
	jmp	rx_b1_done
rxb1_full:
	ldi	GRegs.rx.x, 0x7F80
	add	GRegs.snf.b.dbg_cnt, GRegs.snf.b.dbg_cnt,1
b1_rx_path1a:
	P_W32_ABORT ;abort the stashing of pkt in BS (for PSI)
rx_b1_done:
	TM_ENABLE
b1_exit:TM_YIELD
	flip_rx_r0_r23
	add	GRegs.rx.b.pkt_len, GRegs.rx.b.pkt_len, 32
	loop_here

skip_b1:xin	RXL2_BANK1, &r2, 32  ;error case. just read in data and drop
	clr	RxRegs.aux_flags, RxRegs.aux_flags, f_fh
	jmp	b1_exit
;---------------------------------------------------------------
; END RX TASK, State 3
;--------------------------------------------------------------

;---------------------------------------------------------------
; RX TASK, State 4: RX_BN {{{1
;--------------------------------------------------------------
RX_BN:
	flip_rx_r0_r23
;check for overflow/drop or pkt too long
	qbne	pkt_ok, GRegs.rx.b.state, RX_STATE_DROP
;errors:
	ldi	GRegs.rx.x, 0x7f80  ;indicate we need to drop
	qbbs	rx_bnerr_sideB, RxRegs.aux_flags, f_fh
;side A
	xin	RXL2_BANK0, &r2, 32
	jmp	rx_bn_done
rx_bnerr_sideB:
;side B
	xin	RXL2_BANK1, &r2, 32
	jmp	rx_bn_done
rxbn_full:
	ldi	GRegs.rx.x, 0x7f80  ;indicate we need to drop
	P_W32_ABORT
	add	GRegs.snf.b.dbg_cnt, GRegs.snf.b.dbg_cnt,1
	jmp	rx_bn_done

;take pkt, all good
pkt_ok:	qbbs	rx_bn_sideB, RxRegs.aux_flags, f_fh
	xin	RXL2_BANK0, &r2, 32
	set	RxRegs.aux_flags, RxRegs.aux_flags, f_fh
	jmp	bn_cont1
rx_bn_sideB:
	xin	RXL2_BANK1, &r2, 32
	clr	RxRegs.aux_flags, RxRegs.aux_flags, f_fh
bn_cont1:
	qbbc	rx_bn_done, GRegs.rx.x, 0
	P_W32	rxbn_full
rx_bn_done:
	TM_YIELD
	flip_rx_r0_r23
	add	GRegs.rx.b.pkt_len, GRegs.rx.b.pkt_len, 32
	loop_here
;---------------------------------------------------------------
; END RX TASK, State 4
;--------------------------------------------------------------

;---------------------------------------------------------------
; RX TASK, State 5: RX_EOF {{{1
;--------------------------------------------------------------
RX_EOF:
	TM_DISABLE
	flip_rx_r0_r23
	qbeq	no_rx_sof, GRegs.rx.b.state, RX_STATE_DROP
	qbbc	no_rx_sof, GRegs.rx.b.flags, f_rx_sof
	ldi	r0.w0, 0x3310
	and	r0.w0, r31.w2, r0.w0
	qbne	beof_rx_err, r0.b0, 0x10 ;check for rx errors
	qbbs	rx_beof_sideB, RxRegs.aux_flags, f_fh

;side A, r18 and r0.b0  has length
	RX_EOF_RCV_BANK0
	ldi	r11.b3, 0x80
	xout	22, &r11, 4
	set	r31.t22 ; clear rx eof  (why isnt it bit 20?)
	mov	RxRegs.res1, r0.b0
	jmp	rx_beof_cont0

rx_beof_sideB:
;side B, r18 and r0.b0  has length
	RX_EOF_RCV_BANK1
	ldi	r11.b3, 0x80
	xout	22, &r11, 4
	set	r31.t22 ; clear rx eof  (why isnt it bit 20?)
	add	RxRegs.res1, r0.b0, 32

rx_beof_cont0:
	add	GRegs.rx.b.pkt_len, GRegs.rx.b.pkt_len, r0.b0
	qbbc	rx_eof_done, GRegs.rx.b.flags, f_tohost
;check for pkt to long
	qbge	th_pkt_ok, GRegs.rx.b.pkt_len, r30.w0
	add	r30.w2, r30.w2,1   ; max error (like crc)
	P_W32_ABORT
	jmp	rx_eof_done
th_pkt_ok:
; push 32 to bsram fifo (not all will be valid)
	P_W32	rxeof_noroom
; push 'descriptor' to local queue. will be popped by bg task
	mov	r2.w2, RxRegs.pq_cur       ;flow
	mov	r2.w0, GRegs.rx.b.pkt_len
	mov	r3, GRegs.snf.x      ;bsram fifo position (maybe not necessary)
;to do add rxtx timestamp
	PSIQ_PUSH	rxeof_qfull

rx_eof_done:
	; check for runt packet
	qble	rx_eof_done_b, GRegs.rx.b.pkt_len, 64
	set	r31.t18		;reset RX_fifo

rx_eof_done_b:
	TM_ENABLE
	flip_rx_r0_r23
	TM_YIELD
	ldi	GRegs.rx.s.fl_n_state, 0
	add	GRegs.pkt_cnt.w.rx, GRegs.pkt_cnt.w.rx, 1 ;also should tell bg #of pkts we have procesed
	loop_here

;----------------------------
; bad frame, error handling
;-----------------------------
rxeof_noroom:
rxeof_qfull:
	P_W32_ABORT
	add	GRegs.snf.b.dbg_cnt, GRegs.snf.b.dbg_cnt,1
	jmp	rx_eof_done

beof_rx_err:
no_rx_sof:
;saw no sof before eof or other such issues
	ldi	r11.b3, 0x80
	xout	22, &r11, 4
	set	r31, r31, 22 ; clear rx eof  (why isnt it bit 20?)
	jmp rx_eof_done
;---------------------------------------------------------------
; END RX TASK, State 5
;--------------------------------------------------------------

 .include "resource_table.h"
