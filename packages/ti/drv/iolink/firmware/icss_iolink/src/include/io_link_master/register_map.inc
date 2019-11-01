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
;              file:    register_map.inc
;
;              brief:   PRU IO-Link master register map

;register_map.inc
;PRU IO-Link Master
;ICSS0 PRU0
;v0.1

    .asg "R0", gp_r0    ;general purpose register
    .asg "R1", gp_r1    ;general purpose register
    .asg "R2", gp_r2    ;general purpose register
    .asg "R3", gp_r3    ;general purpose register

    .asg "R4", sample_r0
    .asg "R5", sample_r1
    .asg "R6", sample_r2
    .asg "R7", sample_r3
    .asg "R4.w0", sample0   ;2 bytes of raw samples (CH0)
    .asg "R4.w2", sample1   ;2 bytes of raw samples (CH1)
    .asg "R5.w0", sample2   ;2 bytes of raw samples (CH2)
    .asg "R5.w2", sample3   ;2 bytes of raw samples (CH3)
    .asg "R6.w0", sample4   ;2 bytes of raw samples (CH4)
    .asg "R6.w2", sample5   ;2 bytes of raw samples (CH5)
    .asg "R7.w0", sample6   ;2 bytes of raw samples (CH6)
    .asg "R7.w2", sample7   ;2 bytes of raw samples (CH7)

    .asg "R8", count_r0
    .asg "R9", count_r1
    .asg "R8.b0", count0    ;count of received samples since start bit det. (CH0)
    .asg "R8.b1", count1    ;count of received samples since start bit det. (CH0)
    .asg "R8.b2", count2    ;count of received samples since start bit det. (CH0)
    .asg "R8.b3", count3    ;count of received samples since start bit det. (CH0)
    .asg "R9.b0", count4    ;count of received samples since start bit det. (CH0)
    .asg "R9.b1", count5    ;count of received samples since start bit det. (CH0)
    .asg "R9.b2", count6    ;count of received samples since start bit det. (CH0)
    .asg "R9.b3", count7    ;count of received samples since start bit det. (CH0)

    ;Baud rate setting encoding:
    ;COM3: 0x3
    ;COM2: 0x2
    ;COM1: 0x1
    ;other values will cause the channels sampler and state machine to halt

    .asg "R10", baud_r0
    .asg "R11", baud_r1
    .asg "R10.b0", baud0    ;baud rate setting register (CH0)
    .asg "R10.b1", baud1    ;baud rate setting register (CH0)
    .asg "R10.b2", baud2    ;baud rate setting register (CH0)
    .asg "R10.b3", baud3    ;baud rate setting register (CH0)
    .asg "R11.b0", baud4    ;baud rate setting register (CH0)
    .asg "R11.b1", baud5    ;baud rate setting register (CH0)
    .asg "R11.b2", baud6    ;baud rate setting register (CH0)
    .asg "R11.b3", baud7    ;baud rate setting register (CH0)

    .asg "R12.b0", state            ;state of CH0
    .asg "R12.b1", idle_count       ;count of Tbit waiting for a startbit (CH0)
    .asg "R12.b2", error            ;error register (CH0)
    .asg "R12.b3", samplepoint      ;samplepoint (within sample register) (CH0)
    .asg "R13.b0", databuffer       ;1 byte uart frame buffer (CH0)
    .asg "R13.b1", bfr_ptr          ;points to the next position in the tx or rx buffer (RAM) (CH0)
    .asg "R13.b2", bfr_length       ;length of valid data in buffer (tx or rx) (CH0)
    .asg "R13.b3", txPin            ;pru txPin nr.
    .asg "R14", txEnAdr             ;txEn gpio address of the active port
    .asg "R15", txEnPin             ;txEn Pin of the active port

    .asg "R16.b0", loopcounter      ;counts the nr. of loops (oversampling period) for variable baudrate sampling
    .asg "R16.b1", substate         ;used by the chsel state machine (encodes the active chanel)
    .asg "R16.b2", nxtSymbolFlag    ;8 symbol flags (1 bit per channel) to signal that another 8 samples (1 symbol) have been received

    .asg "R17", ch0MemAdr           ;channel 0 memory base adr
    .asg "R18.w0", ch0RxBufferAdr   ;channel 0 rx buffer base adr
    .asg "R18.w2", ch0TxBufferAdr   ;channel 0 tx buffer base adr
    .asg "R19", activeChMemAdr      ;active channel memory adr

    .asg "R20.w0", gpioSetRegOfs    ;register offset of the gpio set register
    .asg "R20.w2", gpioClrRegOfs    ;register offset of the gpio clr register

    .asg "R21.b0", taReg            ;holds ta and is loaded once at startup (this value can be set once at every PRU startup)
    .asg "R21.b1", t2Reg            ;holds t2 and is loaded once at startup (this value has to be set in code)
    .asg "R21.b2", txBufferSel      ;tx buffer offset register (selects txbuffer0 or txbuffer1)

    .asg "R22", sampleratelut_adr   ;stores a 32bit address for faster access with LBBO (used for LUT's)
    .asg "R23", startbitfilter_adr  ;stores a 32bit address for faster access with LBBO (used for LUT's)
    .asg "R24", averagingfilter_adr ;stores a 32bit address for faster access with LBBO (used for LUT's)
    .asg "R25", paritylut_adr       ;stores a 32bit address for faster access with LBBO (used for LUT's)

    .asg "R26", globalCtrlRegAdr    ;globalCtrlRegAdr

    .asg "R31", rxr                 ;receive pin register (r31 is mapped to the PRU's GPI pins)
    .asg "R30", txr                 ;transmit pin register (r30 is mapped to the PRU's GPO pins)
