.origin 0
.entrypoint ARM_TO_PRU0_INTERRUPT

#include "pruss_test_pru.hp"

ARM_TO_PRU0_INTERRUPT:
POLL:
    // Poll for receipt of interrupt on host 0
    QBBS      EVENT, R31, 30
    JMP       POLL

EVENT:
    //Clear system event in SECR1/2
    MOV32   r0, 0
    // Send notification to Host for program completion
#ifdef AM33XX
    MOV     r3.w0, 0x280 //SECR1
    SET     r0, ARM_PRU0_EVENT
    MOV     R31.b0, PRU0_ARM_EVENT+16
#else
    MOV     r3.w0, 0x284 //SECR2
    SET     r0, ARM_PRU0_EVENT-32
    MOV     R31.b0, PRU0_ARM_EVENT
#endif
    SBCO    r0, C0, r3.w0, 4 
    QBA POLL

