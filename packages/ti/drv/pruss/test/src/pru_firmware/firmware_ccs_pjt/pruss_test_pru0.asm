

    .include "pruss_test_pru.hp"
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/
; Assembler Directives Section
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;/
	.sect   ".text"
	.retain		; Required for building .out with assembly file
	.retainrefs ; Required for building .out with assembly file
	.global MAIN
MAIN:
	ZERO	&R0, 124
	
ARM_TO_PRU0_INTERRUPT:
POLL:
    QBBS      EVENT, R31, 30
    JMP       POLL
EVENT:

    LDI32   r0, 0
    LDI     r3.w0, 0x280
    SET     R0,R0.t16

    LDI     R31.b0, PRU0_ARM_EVENT+16
    SBCO    &r0, C0, r3.w0, 4
    JMP POLL

