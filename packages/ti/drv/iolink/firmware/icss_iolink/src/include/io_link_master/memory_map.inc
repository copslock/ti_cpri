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
;              file:    memory_map.inc
;
;              brief:   PRU IO-Link master memory map

;memory_map.inc
;PRU IO-Link Master
;ICSS0 PRU0
;v0.1

;adresses of data sections in RAM

;instance specific memory
globalSts       .set 0x0000 ;reserved
globalCtrl      .set 0x0001 ;bit 0: needs to be set to start a new transmission on any port
maxResponseTime .set 0x0002 ;sets the maximum response time of all channels and needs to be set before PRU startup
firmwareRev     .set 0x0003 ;reserved

;channel specific control and indication registers
ctrlCh0     .set    0x0004  ;32 bit for control

;control register description:
;bit 0: set to start a new transmission cycle
;bit 1: transmit buffer selector (0 = transmit buffer 0; 1 = transmit buffer 1)
;bit 2: set this bit and bit 0 to transmit a start pulse

statusCh0   .set    0x0008  ;16 bit for status

;status register description:
;bit 0: indicates a completed transmission cycle (has to be reset by the driver)

errorCh0    .set    0x000A  ;16 bit for error

;error register description:
;bit 0: rx_length error
;bit 7: parity error

ctrlCh1     .set    0x000C
statusCh1   .set    0x0010
errorCh1    .set    0x0012

ctrlCh2     .set    0x0014
statusCh2   .set    0x0018
errorCh2    .set    0x001A

ctrlCh3     .set    0x001C
statusCh3   .set    0x0020
errorCh3    .set    0x0022

ctrlCh4     .set    0x0024
statusCh4   .set    0x0028
errorCh4    .set    0x002A

ctrlCh5     .set    0x002C
statusCh5   .set    0x0030
errorCh5    .set    0x0032

ctrlCh6     .set    0x0034
statusCh6   .set    0x0038
errorCh6    .set    0x003A

ctrlCh7     .set    0x003C
statusCh7   .set    0x0040
errorCh7    .set    0x0042

;transmit and receive buffers
;rx buffer length = 128 bytes

receivebfrCh0   .set 0x0100
receivebfrCh1   .set 0x0180
receivebfrCh2   .set 0x0200
receivebfrCh3   .set 0x0280
receivebfrCh4   .set 0x0300
receivebfrCh5   .set 0x0380
receivebfrCh6   .set 0x0400
receivebfrCh7   .set 0x0480

;the transmit buffer is twice as large as the receive buffer
;you can use it as a double send buffer by setting the buffer select bit
;tx buffer length = 256 bytes (or 128 bytes per sub-buffer)

transmitbfrCh0  .set 0x0500
transmitbfrCh1  .set 0x0600
transmitbfrCh2  .set 0x0700
transmitbfrCh3  .set 0x0800
transmitbfrCh4  .set 0x0900
transmitbfrCh5  .set 0x0A00
transmitbfrCh6  .set 0x0B00
transmitbfrCh7  .set 0x0C00

;channel specific RAM

channel0mem     .set 0x0D00
channel1mem     .set 0x0D10
channel2mem     .set 0x0D20
channel3mem     .set 0x0D30
channel4mem     .set 0x0D40
channel5mem     .set 0x0D50
channel6mem     .set 0x0D60
channel7mem     .set 0x0D70

;LUT's

sampleratelut   .set 0x0E00
startbitfilter  .set 0x0F00

;LUT's in the other PRU's memory
averagingfilter .set 0x2E00
paritylut       .set 0x2F00
