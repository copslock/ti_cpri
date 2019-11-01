; Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;              file:	main.asm
;
;              brief:	PRU IO-Link master assembly


;main.asm
;PRU IO-Link Master
;ICSS0 PRU0
;v0.1

	.retain 			; Required for building .out with assembly file
 	.retainrefs 		; Required for building .out with assembly file

 	.include "./include/io_link_master/memory_map.inc"
 	.include "./include/io_link_master/register_map.inc"
 	.include "./include/io_link_master/defines.inc"

	.global	main
	.sect	".text:main"

;program entry point:

main:
	;configuration goes here
	zero &R0, 120 ;clear R0 to R29

	;load the configured maximum response time
	;NOTE: this will only be done once at PRU startup
	ldi gp_r0, maxResponseTime
	lbbo &taReg, gp_r0, 0, 1

	;set t2
	;NOTE: this value is hard coded (you can change it in the defines.inc file if needed)
	ldi t2Reg, t2_time

	;preload frequently used LUT addresses into registers
	ldi32 startbitfilter_adr, startbitfilter
	ldi32 averagingfilter_adr, averagingfilter
	ldi32 paritylut_adr, paritylut
	ldi32 sampleratelut_adr, sampleratelut

	;preload frequently used address offsets and constants into registers
	ldi ch0MemAdr, channel0mem
	ldi ch0RxBufferAdr, receivebfrCh0
	ldi ch0TxBufferAdr, transmitbfrCh0

	ldi gpioSetRegOfs, 0x194
	ldi gpioClrRegOfs, 0x190

	ldi globalCtrlRegAdr, globalCtrl

	;set all tx pins to high (DQ line will be low)
	ldi32 txr, 0xFFFFFFFF

;IO-Link main software loop:

run:

sample:
	;use the samplerate LUT to load one byte of data encoding the baud rate selective sample points (uses loopcounter)
	lbbo &gp_r1, sampleratelut_adr, loopcounter, 1
	;3 cycles

sampleCh0:
	;check if bit nr. baud0 in gp_r1 (result of samplerate LUT) is set.
	qbbc sampleCh0exit0, gp_r1.b0, baud0
	;bit nr. baud0 in gp_r1 (result of samplerate LUT) is set
	;shift sample0 to the left
	lsl sample0, sample0, 1
	;check if rx0 is high
	qbbc sampleCh0exit1, rxr, rx0_pin
	;rx0 is high
	;set the LSB of sample 0
	set sample0, sample0, 0
continue0:
	;count the sample count of channel 1 up
	ADD count0, count0, 0x1
	;check if the first 3 bit of count0 are 0 (check if count0 % 8 == 0)
	AND gp_r0, count0, 0x7
	qbeq setflag0, gp_r0, 0
	;count0 % 8 =/= 0
	;don't set a flag and continue sampling with channel 2
	jmp sampleCh1

sampleCh0exit0:
	;bit nr. baud0 in gp_r1 (result of samplerate LUT) is not set
	;dont take a sample and jump to sampleCh1
	nopx 6
	jmp sampleCh1

sampleCh0exit1:
	;rx0 is not high
	;dont set the LSB of sample0 and continue
	jmp continue0

setflag0:
	;count0 % 8 == 0
	;we have received a new octet of samples
	;set a flag and continue sampling with channel 2
	set nxtSymbolFlag, nxtSymbolFlag, 0
	;8 cycles

	;this repeats for the next 7 channels:

sampleCh1:
	qbbc sampleCh1exit0, gp_r1.b0, baud1
	lsl sample1, sample1, 1
	qbbc sampleCh1exit1, rxr, rx1_pin
	set sample1, sample1, 0
continue1:
	ADD count1, count1, 0x1
	AND gp_r0, count1, 0x7
	qbeq setflag1, gp_r0, 0
	jmp sampleCh2

sampleCh1exit0:
	nopx 6
	jmp sampleCh2

sampleCh1exit1:
	jmp continue1

setflag1:
	set nxtSymbolFlag, nxtSymbolFlag, 1
	;8 cycles

sampleCh2:
	qbbc sampleCh2exit0, gp_r1.b0, baud2
	lsl sample2, sample2, 1
	qbbc sampleCh2exit1, rxr, rx2_pin
	set sample2, sample2, 0
continue2:
	ADD count2, count2, 0x1
	AND gp_r0, count2, 0x7
	qbeq setflag2, gp_r0, 0
	jmp sampleCh3

sampleCh2exit0:
	nopx 6
	jmp sampleCh3

sampleCh2exit1:
	jmp continue2

setflag2:
	set nxtSymbolFlag, nxtSymbolFlag, 2
	;8 cycles

sampleCh3:
	qbbc sampleCh3exit0, gp_r1.b0, baud3
	lsl sample3, sample3, 1
	qbbc sampleCh3exit1, rxr, rx3_pin
	set sample3, sample3, 0
continue3:
	ADD count3, count3, 0x1
	AND gp_r0, count3, 0x7
	qbeq setflag3, gp_r0, 0
	jmp sampleCh4

sampleCh3exit0:
	nopx 6
	jmp sampleCh4

sampleCh3exit1:
	jmp continue3

setflag3:
	set nxtSymbolFlag, nxtSymbolFlag, 3
	;8 cycles

sampleCh4:
	qbbc sampleCh4exit0, gp_r1.b0, baud4
	lsl sample4, sample4, 1
	qbbc sampleCh4exit1, rxr, rx4_pin
	set sample4, sample4, 0
continue4:
	ADD count4, count4, 0x1
	AND gp_r0, count4, 0x7
	qbeq setflag4, gp_r0, 0
	jmp sampleCh5

sampleCh4exit0:
	nopx 6
	jmp sampleCh5

sampleCh4exit1:
	jmp continue4

setflag4:
	set nxtSymbolFlag, nxtSymbolFlag, 4
	;8 cycles

sampleCh5:
	qbbc sampleCh5exit0, gp_r1.b0, baud5
	lsl sample5, sample5, 1
	qbbc sampleCh5exit1, rxr, rx5_pin
	set sample5, sample5, 0
continue5:
	ADD count5, count5, 0x1
	AND gp_r0, count5, 0x7
	qbeq setflag5, gp_r0, 0
	jmp sampleCh6

sampleCh5exit0:
	nopx 6
	jmp sampleCh6

sampleCh5exit1:
	jmp continue5

setflag5:
	set nxtSymbolFlag, nxtSymbolFlag, 5
	;8 cycles

sampleCh6:
	qbbc sampleCh6exit0, gp_r1.b0, baud6
	lsl sample6, sample6, 1
	qbbc sampleCh6exit1, rxr, rx6_pin
	set sample6, sample6, 0
continue6:
	ADD count6, count6, 0x1
	AND gp_r0, count6, 0x7
	qbeq setflag6, gp_r0, 0
	jmp sampleCh7

sampleCh6exit0:
	nopx 6
	jmp sampleCh7

sampleCh6exit1:
	jmp continue6

setflag6:
	set nxtSymbolFlag, nxtSymbolFlag, 6
	;8 cycles

sampleCh7:
	qbbc sampleCh7exit0, gp_r1.b0, baud7
	lsl sample7, sample7, 1
	qbbc sampleCh7exit1, rxr, rx7_pin
	set sample7, sample7, 0
continue7:
	ADD count7, count7, 0x1
	AND gp_r0, count7, 0x7
	qbeq setflag7, gp_r0, 0
	jmp samplefinish

sampleCh7exit0:
	nopx 6
	jmp samplefinish

sampleCh7exit1:
	jmp continue7

sampleCountUpFinish:
	;we have not reached the wraparound threshold yet
	;continue with the uart state machines
	jmp uartmode

setflag7:
	set nxtSymbolFlag, nxtSymbolFlag, 7
	;8 cycles

samplefinish:
	;we have finished sampling
	;count the loopcounter up
	ADD loopcounter, loopcounter, 1
	;check if we have reached the wraparound threshold of loopcounter > 240
	qbgt sampleCountUpFinish, loopcounter, 240
	;we have reached the wraparound threshold
	;reset the loopcounter to 0
	ldi loopcounter, 0
	;3 cycles
; 3 + 64 + 3 = 70 cycles for variable baudrate sampling

;beginning of the UART state machines:
uartmode:

;state machine of channel 0
transceiver:
	lsl gp_r2, substate, 4 ;offset between channels is 16 bytes, calculate the memory offset for channel "substate"
	ADD activeChMemAdr, ch0MemAdr, gp_r2 ;calculate the absolute memory adress by adding channel0's memory offset
	lbbo &r12, activeChMemAdr, 0, 16 ;load the channels memory (6 cycles for 16 bytes, 2 cycles + 16/4 * 1 cycles)
	;check if the symbolFlag of channel 0 is set
	qbbc flagClear ,nxtSymbolFlag, substate
	;9 cycles
	;the flag is set, we should process data now (rx or tx)
	clr nxtSymbolFlag, nxtSymbolFlag, substate ;clear the flag
	;state decider:
	qbeq startbitdet, state, rx_idle ;21 cycles left for startbitdet
	qbeq samplesymbol, state, rx_get ;20
	qbeq channelIdle, state, idle ;19
	qbeq comFinal, state, final ;18
	qbeq startpulse, state, idle_startpulse ;17
	qbeq startpulseCount, state, idle_startpulseCount ;16
	qbeq txSetup, state, tx_setup ;15
	qbeq txStart, state, tx_start ;14
	qbeq txNextMsg, state, tx_nxtMsg ;13
	qbeq txStop, state, tx_complete ;12
	qbeq txTx, state, tx_transmit ;11
	qbeq startpulseClrTxEn, state, idle_startpulseFin ;10
	nopx 9 ;should never be reached
	jmp cyclefinish
flagClear:
	;the flag is not set
	;we will process idle and startpulse functionality only
	nopx 3
	qbeq channelIdle, state, idle
	qbeq comFinal, state, final
	qbeq startpulse, state, idle_startpulse
	qbeq startpulseCount, state, idle_startpulseCount
	nopx 2
	qbeq txStop, state, tx_complete
	nopx 2
	qbeq startpulseClrTxEn, state, idle_startpulseFin
	nopx 9
	jmp cyclefinish

receivefxn:

startbitdet:
	lsl gp_r1.b0, substate, 1 ;sample register is 2 bytes wide so we have to multiply by 2
	ADD gp_r1.b0, gp_r1.b0, &R4.b0 ;sample register file adr
	mviw r0.w0, *r1.b0 ;load sample
	;use the predefined LUT to detect a startbit edge in the sample stream
	;the LUT will also return the position of the start bit
	lbbo &gp_r2, startbitfilter_adr, gp_r0.b0, 1
	qbne startbitdet_0, gp_r2.b0, 0
	;we have not detected a start bit. shift the bitstream by 4 and retry
	lsr gp_r0, gp_r0, 4
	lbbo &gp_r2, startbitfilter_adr, gp_r0.b0, 1
	qbne startbitdet_1, gp_r2.b0, 0
	;there is no valid start bit edge in the bit stream
	;check if we have already received a byte
	qbne checkt2, bfr_ptr, 0
	;we have not received any bytes yet
	;check if we have reached the maximum response time
	ADD idle_count, idle_count, 1
	qblt rxFinish, idle_count, taReg
	;threshold not reached, continue with startbitdet
	nopx 5
	jmp cyclefinish
	;21 cycles for startbit detection

checkt2:
	;there are bytes in the buffer
	;we need to check the devices t2 time
	ADD idle_count, idle_count, 1
	qblt rxFinish, idle_count, t2Reg
	;threshold not reached, continue with startbitdet
	nopx 5
	jmp cyclefinish
	;8 cycles

startbitdet_0:
	;we have detected a valid startbit pattern in the first 8 bit of the sample buffer
	;the next instructions will reset the count register of the active channel to 0
	ADD gp_r1.b0, substate, &R8.b0
	ldi gp_r0.b0, 0
	mvib *r1.b0, gp_r0.b0
	;copy the samplepoint to the active channel's samplepoint register
	mov samplepoint, gp_r2.b0
	;load the next state for this channel
	ldi state, rx_get
	nopx 8
	jmp cyclefinish
	;14 cycles

startbitdet_1:
	;we have a detected valid startbit pattern located between bit 4 and 11 of the sample buffer
	;copy the samplepoint to the active channel's samplepoint register
	mov samplepoint, gp_r2.b0
	;the offset of 4 bit has to be added to the result
	ADD samplepoint, samplepoint, 4
	;load the next state for this channel
	ldi state, rx_get
	;the next instructions will try to move the samplepoint to the right re-calculation of the count register
	qble adjustSamplePoint, samplepoint, 8
	;the next instructions will reset the count register of the active channel to 0
	ADD gp_r1.b0, substate, &R8.b0
	ldi gp_r0.b0, 0
	mvib *r1.b0, gp_r0.b0
	NOP
	jmp cyclefinish
	;9 cycles

adjustSamplePoint:
	;try to move the samplepoint to the right re-calculation of the count register
	;the next instructions will set the count register of the active channel to 8
	ADD gp_r1.b0, substate, &R8.b0
	ldi gp_r0.b0, 8
	mvib *r1.b0, gp_r0.b0
	;decrease samplepoint by 8
	SUB samplepoint, samplepoint, 8 ;decrease samplepoint by 8
	jmp cyclefinish
	;5 cycles

rxFinish:
	;we have reached the idlecount limit
	;the device has finished transmitting data
	;write reveived nr of bytes to memory and go to comFinal state
	lsl gp_r3, substate, 7 ;multiply substate by 265 (offset between buffers)
	ADD gp_r2, ch0RxBufferAdr, gp_r3 ;add the offset to the entry address
	sbbo &bfr_ptr, gp_r2, 0, 1 ;write the rx length to the RAM
	ldi state, final ;load next state
	jmp cyclefinish
	;6 cycles

samplesymbol:
	lsl gp_r1.b0, substate, 1 ;sample register is 2 bytes wide so we have to multiply by 2
	ADD gp_r1.b0, gp_r1.b0, &R4.b0 ;sample register file adr
	mviw gp_r0.w0, *r1.b0 ;load sample

	;take the next 8 samples and filter them
	;this will decide if a '1' or '0' has been received
	lsr gp_r0, gp_r0.w0, samplepoint
	lbbo &gp_r0, averagingfilter_adr, gp_r0.b0, 1

	ADD gp_r1.b0, substate, &R8.b0 ;count register file adr
	mvib gp_r2.b0, *r1.b0 ;load count

	;check if we have received > 72 samples (9 symbols, position of parity bit)
	qblt paritychk, gp_r2.b0, 72

	;it's not a parity bit
	lsr databuffer, databuffer, 1 ;shift the databuffer to the right
	OR databuffer, databuffer, gp_r0.b0 ;add the new bit of data as msb (buffer will fill from left to right)

	qblt writeData, gp_r2.b0, 64 ;jump to writeData if we are done receiving data bits (8 x 8 samples)
	nopx 6
	jmp cyclefinish
	;20 cycles for symbol sampling

writeData:
	;we have received a new octet of data
	;write it to the RAM
	ADD bfr_ptr, bfr_ptr, 1
	lsl gp_r2, substate, 7
	ADD gp_r0, ch0RxBufferAdr, gp_r2
	sbbo &databuffer, gp_r0, bfr_ptr, 1
	NOP
	jmp cyclefinish
	;7 cycles

paritychk:
	;load the pre-calculated parity for the bit pattern in the databuffer
	lbbo &gp_r2, paritylut_adr, databuffer, 1
	;compare the received parity bit with the calculated bit
	XOR gp_r0, gp_r2.b0, gp_r0.b0
	OR error, error, gp_r0
	;reset the idle_count to 0, to make it available for the next startbit detection
	ldi idle_count, 0
	;load the next state for the active channel
	ldi state, rx_idle
	;check for rxlength error (too many bytes received)
	qblt rxlength_error, bfr_ptr, bfr_length
	NOP
	jmp cyclefinish
	;10 cycles for parity check

rxlength_error:
	set error, error, 0 ;set the rxlength error flag
	jmp cyclefinish
	;2 cycles

;tx functions
transmitter:

txSetup:
	;set txEnable pin
	ADD gp_r0 ,txEnAdr ,gpioSetRegOfs
	sbbo &txEnPin, gp_r0, 0, 4
	;reset bfr_ptr to 2 (0 and 1 are used for length information)
	ldi bfr_ptr, 2
	;load the bfr_length byte from the transmit buffer
	lsl gp_r2, substate, 8 ;shift the substate by 8 to calculate the memory offset of the channel
	ADD gp_r2, ch0TxBufferAdr, gp_r2 ;add ch0's txbuffer address
	lbbo &bfr_length, gp_r2, 0, 1 ;load the byte from DRAM
	;add 2 to bfr_length to compensate for the first 2 bytes
	ADD bfr_length, bfr_length, 2
	;set the active channels state to tx_start
	ldi state, tx_start
	;reset the channels error register
	ldi error, 0x00
	nopx 2
	jmp cyclefinish
	;15 cycles

txStart:
	;generate start bit
	;reset the channels count register to 0
	ADD gp_r1.b0, substate, &R8.b0
	ldi gp_r0.b0, 0
	mvib *r1.b0, gp_r0.b0 ;set count to 0
	;clr the tx pin to transmit the start bit
	clr txr, txr, txPin
	;set state to tx_transmit
	ldi state, tx_transmit
	;reset the idle count to 0 (will be used in the rx state machine later)
	ldi idle_count, 0
	;load the first byte of data from the txbuffer (RAM)
	lsl gp_r2, substate, 8
	ADD gp_r2, ch0TxBufferAdr, gp_r2
	;check if buffer0 or buffer1 is selected
	qbbc txStartUseBfr0, txBufferSel, substate
	ADD gp_r2, gp_r2, 0x80
	lbbo &databuffer, gp_r2, bfr_ptr, 1
	jmp cyclefinish
	;14 cycles

txStartUseBfr0:
	NOP
	lbbo &databuffer, gp_r2, bfr_ptr, 1
	jmp cyclefinish
	;5 cycles

txNextMsg:
	;reset count to 0
	ADD gp_r1.b0, substate, &R8.b0 ;count register file adr
	ldi gp_r0.b0, 0
	mvib *r1.b0, gp_r0.b0 ;set count to 0
	;set state to tx_transmit
	ldi state, tx_transmit
	;clr the tx pin to transmit the start bit
	clr txr, txr, txPin
	;load next byte from tx buffer (RAM)
	lsl gp_r2, substate, 8
	ADD gp_r2, ch0TxBufferAdr, gp_r2
	;tx buffer selection (0 or 1)
	qbbc txNextMsgUseBfr0, txBufferSel, substate
	ADD gp_r2, gp_r2, 0x80
	lbbo &databuffer, gp_r2, bfr_ptr, 1
	jmp cyclefinish
	;13 cycles

txNextMsgUseBfr0:
	NOP
	lbbo &databuffer, gp_r2, bfr_ptr, 1
	jmp cyclefinish
	;5 cycles

txTx:
	;load the count register
	ADD gp_r1.b0, substate, &R8.b0 ;count register file adr
	mvib gp_r0.b0, *r1.b0 ;load count
	;check if we need to send a stopbit (count0 > 9 x 8 samples)
	qblt generateStopbit, gp_r0.b0, 72
	;check if we need to send a paritybit (count0 > 8 x 8 samples)
	qblt generateParitybit, gp_r0.b0, 64
	;divide count0 by 8 and subtract 1 for the start bit
	lsr gp_r0, gp_r0.b0, 3
	SUB gp_r0, gp_r0, 1
	;use the result to determine which bit needs to be transmitted next
	;this bit will be set or cleared
	qbbs setTx, databuffer, gp_r0
	clr txr, txr, txPin
	nopx 2
	jmp cyclefinish
	;11 cycles
setTx:
	set txr, txr, txPin
	nopx 2
	jmp cyclefinish
	;4 cycles

generateParitybit:
	;calculate the parity using a LUT
	lbbo &gp_r0, paritylut_adr, databuffer, 1
	;transmit the parity bit
	qbbs setParity, gp_r0, 7
	clr txr, txr, txPin
	NOP
	jmp cyclefinish
	;7 cycles
setParity:
	set txr, txr, txPin
	NOP
	jmp cyclefinish
	;3 cycles

generateStopbit:
	;set the tx pin to transmit the stop bit
	set txr, txr, txPin
	;count bfr_ptr up
	ADD bfr_ptr, bfr_ptr, 1
	;check if we have transmitted all bytes in the tx buffer
	qble txComplete, bfr_ptr, bfr_length
	;we still need to send a byte
	;set the channels state to tx_nxtMsg
	ldi state, tx_nxtMsg
	nopx 3
	jmp cyclefinish
	;8 cycles

txComplete:
	;we have sent all bytes in the buffer
	;set the channels state to tx_complete
	ldi state, tx_complete
	nopx 3
	jmp cyclefinish
	;5 cycles

txStop:
	;we have transmitted all bytes in the buffer
	;clr the txen pin
	ADD gp_r0 ,txEnAdr ,gpioClrRegOfs
	sbbo &txEnPin, gp_r0, 0, 4
	ldi state, rx_idle ;set the state to rx_idle
	;load the rx bfr_length from RAM
	lsl gp_r2, substate, 8
	ADD gp_r2, ch0TxBufferAdr, gp_r2
	lbbo &bfr_length, gp_r2, 1, 1
	;add 1 to the length (first field in the rx buffer is the nr. of received bytes in the buffer)
	ADD bfr_length, bfr_length, 1
	;reset the bfr_ptr to 0
	ldi bfr_ptr, 0
	jmp cyclefinish
	;12 cycles

;ARM communication will be handled in the idle state:
channelIdle:
	;load configuration and baud rate from memory
	ldi gp_r0, ctrlCh0
	lsl gp_r2, substate, 3
	ADD gp_r0, gp_r0, gp_r2
	lbbo &gp_r2, gp_r0, 0, 2
	ADD gp_r1.b0, substate, &R10.b0
	mvib *R1.b0, gp_r2.b1
	;check global control
	lbbo &gp_r3, globalCtrlRegAdr, 0, 1
	AND gp_r3.b0, gp_r2.b0, gp_r3.b0
	;check if we need to start a commmunication cycle
	qbbs beginDataExchange, gp_r3.b0, 0
	;the bit is not set, so we don't need to do anything
	;the channel will remain in idle state
	nopx 5
	jmp cyclefinish
	;19 cycles

txBuffer0sel:
	;tx buffer 0 is selected (we dont need to set the bit in the txBufferSel register)
	jmp checkBit;

beginDataExchange:
	;reset the tx buffer select bit
	clr txBufferSel, txBufferSel, substate
	;check if tx buffer 0 or tx buffer 1 is selected
	qbbc txBuffer0sel, gp_r2.b0, 1
	;tx buffer 1 is selected (set the according bit in the txBufferSel register)
	set txBufferSel, txBufferSel, substate
checkBit:
	;check if the startpulse bit is set
	qbbs beginStartPulse ,gp_r2.b0, 2
	;there are > 0 bytes in the tx buffer
	ldi state, tx_setup ;set state to tx_setup
	jmp cyclefinish
	;6 cycles

;start pulse generator
;will be triggered by transmitting an empty (txlength = 0) buffer of tx bytes
beginStartPulse:
	;there are no bytes in the buffer
	ldi state, idle_startpulse ;set state to idle_startpulse (generate a startpulse)
	jmp cyclefinish
	;2 cycles

startpulse:
	;check the voltage level on the dq line
	lsl gp_r1.b0, substate, 1 ;sample register is 2 bytes wide so we have to multiply by 2
	ADD gp_r1.b0, gp_r1.b0, &R4.b0 ;sample register file adr
	mvib r0.b0, *r1.b0 ;load sample
	lbbo &gp_r0, averagingfilter_adr, r0.b0, 1
	;reset bfr_length to 0 (temp. used to measure the pulsewidth)
	ldi bfr_length, 0
	;set the txEn pin
	ADD gp_r2 ,txEnAdr ,gpioSetRegOfs
	sbbo &txEnPin, gp_r2, 0, 4
	;set state to idle_starpulseCount
	ldi state, idle_startpulseCount
	;transmit a signal of oppposite voltage level
	qbbc lowpulse, gp_r0, 7
	clr txr, txr, txPin
	nopx 3
	jmp cyclefinish
	;17 cycles

lowpulse:
	set txr, txr, txPin
	nopx 3
	jmp cyclefinish
	;5 cycles

startpulseCount:
	;measure the pulse width by using bfr_length as a counter
	qblt startpulseFinish, bfr_length, 17
	;count bfr_length up
	ADD bfr_length, bfr_length, 1
	nopx 13
	jmp cyclefinish
	;16 cycles

startpulseFinish:
	;bfr_length has reached it's threshold
	set txr, txr, txPin ;set the tx pin (this will set the dq line's voltage level to low)
	ldi state, idle ;set the state to idle
	;reset the channels control byte
	ldi gp_r0, ctrlCh0
	lsl gp_r2, substate, 3
	ADD gp_r0, gp_r0, gp_r2
	ldi gp_r2, 0x00
	sbbo &gp_r2, gp_r0, 0, 1
	ldi state, idle_startpulseFin ;set state to startpulse fin
	nopx 5
	jmp cyclefinish
	;15 cycles

startpulseClrTxEn:
	ldi state, idle ;set state to idle
	; clear the channels TxEn pin
	ADD gp_r0 ,txEnAdr ,gpioClrRegOfs
	sbbo &txEnPin, gp_r0, 0, 4
	nopx 5
	jmp cyclefinish
	;10 cycles


comFinal:
	;cleanup function for end of communication cycle
	ldi gp_r0, ctrlCh0
	lsl gp_r2, substate, 3
	ADD gp_r0, gp_r0, gp_r2
	ldi gp_r2, 0x00
	sbbo &gp_r2, gp_r0, 0, 1 ;reset the control byte in RAM
	ldi state, idle ;set state to idle
	ldi gp_r0, statusCh0
	lsl gp_r2, substate, 3
	ADD gp_r0, gp_r0, gp_r2
	ldi32 gp_r2, 0
	ldi gp_r2.b0, status_complete
	mov gp_r2.b2, error
	sbbo &gp_r2, gp_r0, 0, 4 ;set the channels status register to complete
	ldi R31.b0, ((1<<5) | 0x0) ;generate cycle complete interrupt
	jmp cyclefinish
	;18 cycles

	;32 cycles for symbol processing

cyclefinish:
	sbbo &r12, activeChMemAdr, 0, 8 ;write channels memory (3 cycles for 8 bytes)
	;channel selector (Channels will be processed in a row while its necessary to sample every loop cycle)
chselect:
	qblt wraparound, substate, 6
	ADD substate, substate, 1
	jmp run
wraparound:
	ldi substate, 0
	nopx 4 ;cycle compensation
	jmp run
	;6.5 cycles on average
