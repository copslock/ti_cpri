/*
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef BUILD_MCU1_0
    #pragma CODE_SECTION(sbl_putc, ".sbl_mcu_1_0_resetvector")
    #pragma CODE_SECTION(sbl_puts, ".sbl_mcu_1_0_resetvector")
    #pragma CODE_SECTION(sbl_putbyte, ".sbl_mcu_1_0_resetvector")
    #pragma CODE_SECTION(sbl_putui, ".sbl_mcu_1_0_resetvector")
    #pragma CODE_SECTION(sbl_putsui, ".sbl_mcu_1_0_resetvector")
#endif

#ifdef BUILD_MCU1_1
    #pragma CODE_SECTION(sbl_putc, ".sbl_mcu_1_1_resetvector")
    #pragma CODE_SECTION(sbl_puts, ".sbl_mcu_1_1_resetvector")
    #pragma CODE_SECTION(sbl_putbyte, ".sbl_mcu_1_1_resetvector")
    #pragma CODE_SECTION(sbl_putui, ".sbl_mcu_1_1_resetvector")
    #pragma CODE_SECTION(sbl_putsui, ".sbl_mcu_1_1_resetvector")
#endif

#ifdef BUILD_MCU2_0
    #pragma CODE_SECTION(sbl_putc, ".sbl_mcu_2_0_resetvector")
    #pragma CODE_SECTION(sbl_puts, ".sbl_mcu_2_0_resetvector")
    #pragma CODE_SECTION(sbl_putbyte, ".sbl_mcu_2_0_resetvector")
    #pragma CODE_SECTION(sbl_putui, ".sbl_mcu_2_0_resetvector")
    #pragma CODE_SECTION(sbl_putsui, ".sbl_mcu_2_0_resetvector")
#endif

#ifdef BUILD_MCU2_1
    #pragma CODE_SECTION(sbl_putc, ".sbl_mcu_2_1_resetvector")
    #pragma CODE_SECTION(sbl_puts, ".sbl_mcu_2_1_resetvector")
    #pragma CODE_SECTION(sbl_putbyte, ".sbl_mcu_2_1_resetvector")
    #pragma CODE_SECTION(sbl_putui, ".sbl_mcu_2_1_resetvector")
    #pragma CODE_SECTION(sbl_putsui, ".sbl_mcu_2_1_resetvector")
#endif

#ifdef BUILD_MCU3_0
    #pragma CODE_SECTION(sbl_putc, ".sbl_mcu_3_0_resetvector")
    #pragma CODE_SECTION(sbl_puts, ".sbl_mcu_3_0_resetvector")
    #pragma CODE_SECTION(sbl_putbyte, ".sbl_mcu_3_0_resetvector")
    #pragma CODE_SECTION(sbl_putui, ".sbl_mcu_3_0_resetvector")
    #pragma CODE_SECTION(sbl_putsui, ".sbl_mcu_3_0_resetvector")
#endif

#ifdef BUILD_MCU3_1
    #pragma CODE_SECTION(sbl_putc, ".sbl_mcu_3_1_resetvector")
    #pragma CODE_SECTION(sbl_puts, ".sbl_mcu_3_1_resetvector")
    #pragma CODE_SECTION(sbl_putbyte, ".sbl_mcu_3_1_resetvector")
    #pragma CODE_SECTION(sbl_putui, ".sbl_mcu_3_1_resetvector")
    #pragma CODE_SECTION(sbl_putsui, ".sbl_mcu_3_1_resetvector")
#endif

#ifdef BUILD_MPU1_0
    void sbl_putc(unsigned char c)  __attribute__((section(".sbl_mpu_1_0_resetvector")));
    void sbl_puts(char *str)  __attribute__((section(".sbl_mpu_1_0_resetvector")));
    void sbl_putbyte(unsigned char b)  __attribute__((section(".sbl_mpu_1_0_resetvector")));
    void sbl_putui(unsigned int ul)  __attribute__((section(".sbl_mpu_1_0_resetvector")));
    void sbl_putsui(char *s, unsigned int ui, int crlf)  __attribute__((section(".sbl_mpu_1_0_resetvector")));
#endif

#ifdef BUILD_MPU1_1
    void sbl_putc(unsigned char c)  __attribute__((section(".sbl_mpu_1_1_resetvector")));
    void sbl_puts(char *str)  __attribute__((section(".sbl_mpu_1_1_resetvector")));
    void sbl_putbyte(unsigned char b)  __attribute__((section(".sbl_mpu_1_1_resetvector")));
    void sbl_putui(unsigned int ul)  __attribute__((section(".sbl_mpu_1_1_resetvector")));
    void sbl_putsui(char *s, unsigned int ui, int crlf)  __attribute__((section(".sbl_mpu_1_1_resetvector")));
#endif

#ifdef BUILD_MPU2_0
    void sbl_putc(unsigned char c)  __attribute__((section(".sbl_mpu_2_0_resetvector")));
    void sbl_puts(char *str)  __attribute__((section(".sbl_mpu_2_0_resetvector")));
    void sbl_putbyte(unsigned char b)  __attribute__((section(".sbl_mpu_2_0_resetvector")));
    void sbl_putui(unsigned int ul)  __attribute__((section(".sbl_mpu_2_0_resetvector")));
    void sbl_putsui(char *s, unsigned int ui, int crlf)  __attribute__((section(".sbl_mpu_2_0_resetvector")));
#endif

#ifdef BUILD_MPU2_1
    void sbl_putc(unsigned char c)  __attribute__((section(".sbl_mpu_2_1_resetvector")));
    void sbl_puts(char *str)  __attribute__((section(".sbl_mpu_2_1_resetvector")));
    void sbl_putbyte(unsigned char b)  __attribute__((section(".sbl_mpu_2_1_resetvector")));
    void sbl_putui(unsigned int ul)  __attribute__((section(".sbl_mpu_2_1_resetvector")));
    void sbl_putsui(char *s, unsigned int ui, int crlf)  __attribute__((section(".sbl_mpu_2_1_resetvector")));
#endif

#define UART0_ADDR	((int)0x40A00000)
#define UART_RHR	((int)0x0U)
#define UART_LSR	((int)0x14U)

#define UART_RHR_REG_ADDR	((volatile unsigned int *)(UART0_ADDR + UART_RHR))
#define UART_LSR_REG_ADDR	((volatile unsigned int *)(UART0_ADDR + UART_LSR))

void sbl_putc(unsigned char c)
{
    while((*UART_LSR_REG_ADDR & 0x20) == 0);
    *UART_RHR_REG_ADDR = c;
}

void sbl_puts(char *str)
{
	for (; *str != '\0'; str++ )
		sbl_putc(*str);
}


void sbl_putbyte(unsigned char b)
{
	const char dig[] = "0123456789abcdef";

	sbl_putc(dig[(b >> 4) & 0xf]);
	sbl_putc(dig[b & 0xf]);
}

void sbl_putui(unsigned int ul)
{
	sbl_putbyte((ul >> 24) & 0xff);
	sbl_putbyte((ul >> 16) & 0xff);
	sbl_putbyte((ul >>  8) & 0xff);
	sbl_putbyte( ul        & 0xff);
}

void sbl_putsui(char *s, unsigned int ui, int crlf)
{
	sbl_puts(s);
	sbl_putui(ui);
	if (crlf)
		sbl_puts("\r\n");
}
