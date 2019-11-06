; half duplex helpers

READ_RGMII_CFG	.macro rtmp, speed_flags
	TM_DISABLE
	lbco	&rtmp, c9, 0x4, 4
	and	rtmp.b0, rtmp.b0, f_mask_o ; takes only speed/duplex related bits
	and	speed_flags, speed_flags, f_mask_a ; clean old status
	or	speed_flags, speed_flags, rtmp.b0
	TM_ENABLE
	.endm

; most likely IPG and backoff is the same timer
; so, we will need only one macro 
if_ipg_not_expired	.macro then_go
	; TODO check IPG
	; if not expired jmp then_go
	ldi32	r2, FW_CONFIG
	lbbo	&r3, r2, TX_IPG, 4 ;
	lbco	&r4, c11, 0x0c, 4  ; read cycle counte
	qbgt	then_go, r4, r3	   ; not expired yet
	clr	GRegs.speed_f, GRegs.speed_f, f_wait_ipg
	.endm

; touch r2, r3, r4
reset_cycle_cnt .macro
	lbco	&r2, c11, 0x0, 4 ; disable cycle counter
	clr	r2.t3
	sbco	&r2, c11, 0x0, 4
	zero	&r3, 4		 ; reset cycle counter
	sbco	&r3, c11, 0xc, 4
	set	r2.t3		 ; enable cycle counter
	sbco	&r2, c11, 0x0, 4
	.endm

IPG_10MBPS	.set 2400 ; (12 * 8 * 100) nsec / 4
IPG_10MBPS7	.set 3800 ; ((12 + 7) * 8 * 100) nsec / 4
IPG_10MBPS_ADJ	.set 600  ; (3 * 8 * 100) / 4
IPG_100MBPS_ADJ	.set 60   ; (3 * 8 * 10) / 4

start_backoff_timer .macro num
	reset_cycle_cnt
	mov	r2, num
	qbge	$3, r2, 9
	ldi	r2, 9
$3:	ldi	r3, 1		; create mask
	not	r3, r3		;
	lsl	r3, r3, r2	;
	not	r3, r3		;
	GET_IEP_CNT	r4	;
	xor	r4.w0, r4.w0, r4.w2
	ldi32	r2, FW_CONFIG	   ; read value from SEED
	lbbo	&r5, r2, CFG_SEED, 4	; get seed
	xor	r4, r4, r5	; xor time with seed 
	and	r4, r4, r3	; mask with backoff interval
	qbbc	$1, GRegs.speed_f, f_100mbps
	; for 100 mbps multiply on 5.12 usec (r4 << 7) 
	lsl	r3, r4, 7
	ldi	r5, IPG_100MBPS_ADJ ;
	qba	$2
$1:	; for 10 mbps multiply on 51.2 usec (r4 << 10) + (r4 << 8)
	lsl	r3, r4, 10
	lsl	r4, r4, 8
	add	r3, r3, r4
	; for 10 MBPS we need to maintain IPG manually
	; so if r3 is 0 then set it to 9.6 usec
	qbne	$2, r3, 0
	ldi	r3, IPG_10MBPS
	ldi	r5, IPG_10MBPS_ADJ ;
$2:	add	r3, r3, r5
	sbbo	&r3, r2, TX_IPG, 4 ; store it to TX_IPG
	set	GRegs.speed_f, GRegs.speed_f, f_wait_ipg
	.endm

start_ipg_timer .macro
	reset_cycle_cnt
	ldi32	r2, FW_CONFIG	   ;
	ldi	r4, IPG_10MBPS7
	sbbo	&r4, r2, TX_IPG, 4 ; store it to TX_IPG
	set	GRegs.speed_f, GRegs.speed_f, f_wait_ipg
	.endm


; read 8bytes from smem offset to reg and reg+1
; we use r0
read_bd_from_smem .macro reg, offset
	ldi32	r0, FW_CONFIG + offset
	lbbo	&reg, r0, 0, 8
	.endm

write_bd_to_smem .macro reg, offset
	ldi32	r0, FW_CONFIG + offset
	sbbo	&reg, r0, 0, 8
	.endm

update_col_status .macro
 .if $defined("SLICE0")
	lbco	&r2, c27, 0x38, 4
 .else
	lbco	&r2, c27, 0x3c, 4
 .endif
	qbbc	$1, r2, 1
	set	GRegs.speed_f, GRegs.speed_f, f_col_detected
$1:
	.endm

read_col_status .macro	reg
 .if $defined("SLICE0")
	lbco	&reg, c27, 0x38, 4
 .else
	lbco	&reg, c27, 0x3c, 4
 .endif
	.endm

