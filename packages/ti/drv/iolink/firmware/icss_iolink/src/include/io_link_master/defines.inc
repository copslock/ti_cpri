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
;              file:    defines.inc
;
;              brief:   PRU IO-Link master defines

;defines.inc
;PRU IO-Link Master
;ICSS0 PRU0
;v0.1

    ;hard coded timings
    .asg "5", t2_time   ;hard coded t2 time (Tbit)

    ;status codes
    .asg "0x0", status_idle
    .asg "0x1", status_complete

    ;State machine encoding
    .asg "0x0", idle
    .asg "0x1", idle_startpulse
    .asg "0x2", idle_startpulseCount
    .asg "0x3", idle_startpulseFin
    .asg "0x4", rx_idle
    .asg "0x5", rx_get
    .asg "0x6", tx_setup
    .asg "0x7", tx_start
    .asg "0x8", tx_nxtMsg
    .asg "0x9", tx_transmit
    .asg "0xA", tx_complete
    .asg "0xB", final

    ;RX and TX pin definition
    ;GPI / GPO pins of the PRU (R30)
    ;these pins are much faster as GPIO pins and can be written in a single PRU cycle
    .asg "0", rx0_pin   ;N24 J16 Pin 31
    .asg "1", rx1_pin   ;N22 J16 Pin 33
    .asg "2", rx2_pin   ;H23 J16 Pin 35
    .asg "3", rx3_pin   ;M24 J16 Pin 32
    .asg "4", rx4_pin   ;L23 J16 Pin 34
    .asg "5", rx5_pin   ;K23 J16 Pin 36
    .asg "6", rx6_pin   ;M25 J16 Pin 52
    .asg "7", rx7_pin   ;L24 J16 Pin 54

    .asg "8", tx0_pin   ;B1 J16 Pin 56
    .asg "9", tx1_pin   ;B2 J16 Pin 37
    .asg "10", tx2_pin  ;C2 J16 Pin 38
    .asg "11", tx3_pin  ;C1 J16 Pin 58
    .asg "19", tx4_pin  ;bluewire Pin 5
    .asg "12", tx5_pin  ;bluewire Pin 53
    .asg "13", tx6_pin  ;bluewire Pin 55
    .asg "18", tx7_pin  ;bluewire Pin 57

;macro used to precisely wait for (P1 > 1) cycles
nopx    .macro P1
    loop endloop?, P1 - 1
    NOP
endloop?:
    .endm
