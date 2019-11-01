/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *              file:    ioLink_powerSwitchTask.h
 *
 *              brief:   PRU IO-Link master power switch driver
 */

//NOTE: This file is not final and should be used with caution.
//      There is a quick fix applied to this driver which deactivates its functionality and switches all outputs on.

#ifndef IO_LINK_IOLINK_POWERSWITCHTASK_H_
#define IO_LINK_IOLINK_POWERSWITCHTASK_H_

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

enum portStateenum{port_off=0, port_on=1};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern float IO_Link_get_current(int port);
extern int IO_Link_get_fault(int port);
extern float IO_Link_get_input_current(void);
extern void IO_Link_Power(int port, enum portStateenum state);
extern int IO_Link_getPower(int port);
extern void IOLink_powerSwitchTask(UArg arg0);

#endif /* IO_LINK_IOLINK_POWERSWITCHTASK_H_ */
