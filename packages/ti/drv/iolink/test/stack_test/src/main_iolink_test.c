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
 */

/*              file:    main_iolink_test.c
 *
 *              brief:   IO-Link master unit test code
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/starterware/include/hw/am437x.h>
#include <ti/starterware/include/am43xx/chipdb_defs.h>
#include <ti/starterware/include/soc_control.h>
#include <ti/starterware/include/prcm.h>
#include <ti/csl/src/ip/icss/V1/cslr_icss_cfg.h>

#include <ti/drv/pruss/pruicss.h>
#include <ti/drv/iolink/src/v0/IOLINK_v0.h>
#include <ti/drv/iolink/test/stack_test/src/ioLink_LEDTask.h>
#include <ti/drv/iolink/test/stack_test/src/ioLink_powerSwitchTask.h>
#include <ti/drv/iolink/test/stack_test/src/ioLink_printTask.h>
#include <ti/drv/iolink/test/stack_test/src/ioLink_autosenTask.h>

#include <ti/board/board.h>

IOLINK_Handle iolinkHandles[2] = {NULL, };

extern IOLINK_v0_Callbacks iolinkCallbacks;
extern void mst_main (void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void IO_Link_Master_Stack_Task(UArg arg0)
{
    IOLINK_Params params;
    uint32_t      instance = 0;

    /* Initialize the PRU IO-Link driver */
    IOLINK_init();

    /* Open IO-Link instance */
    IOLINK_Params_init(&params);
    iolinkHandles[instance] = IOLINK_open(instance, &params);

    /* run the IO-Link master stack */
    if (iolinkHandles[instance] != NULL)
    {
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_SET_CALLBACKS, (void *)(&iolinkCallbacks));
        mst_main();
    }

}

int32_t IOLINK_boardInit(void)
{
    Board_STATUS   boardStatus;
    uint32_t       timerInstance = 4;
    int32_t        retVal = 0;

    boardStatus = Board_init(BOARD_INIT_MODULE_CLOCK |
                             BOARD_INIT_ICSS_PINMUX  |
                             BOARD_INIT_UART_STDIO);

    if (boardStatus == BOARD_SOK)
    {
        PRCMModuleEnable(CHIPDB_MOD_ID_PWMSS, 0, 0);

        /* Clock source selection */
        SOCCtrlTimerClkSrcSelect(timerInstance,
                                 SOC_CTRL_DMTIMER_CLK_SRC_M_OSC_24M);
        PRCMModuleEnable(CHIPDB_MOD_ID_DMTIMER, timerInstance , FALSE);
        PRCMModuleEnable(CHIPDB_MOD_ID_DMTIMER, timerInstance + 2, FALSE);

        /*Pinmux for LatchO in AM437x IDK*/
        *((uint32_t *)0x44E10978) = 0x00050006;

        //Board_phyReset(2);

        /* Set ICSS1 in no standby mode */
        HW_WR_FIELD32(0x54426000U + CSL_ICSSCFG_SYSCFG,
                      CSL_ICSSCFG_SYSCFG_STANDBY_MODE , 1);
    }
    else
    {
        retVal = -1;
    }

    return (retVal);
}

int main()
{
    Task_Params taskParams;

    if (IOLINK_boardInit() == 0)
    {
        /* create a new task for the LED driver */
        Task_Params_init(&taskParams);
        taskParams.priority = 10;
        taskParams.stackSize = 2048*4;
        taskParams.instance->name = "ioLink LED Task";
        Task_create((Task_FuncPtr)IOLink_LEDTask, &taskParams, NULL);

        /* create a new task for the high side switch driver */
        Task_Params_init(&taskParams);
        taskParams.priority = 10;
        taskParams.stackSize = 2048*4;
        taskParams.instance->name = "ioLink Power Switch Task";
        Task_create((Task_FuncPtr)IOLink_powerSwitchTask, &taskParams, NULL);

        /* create the IO-Link Master Stack Task */
        Task_Params_init(&taskParams);
        taskParams.priority = 11;
        taskParams.stackSize = 2048;
        taskParams.instance->name = "ioLink Master Task";
        Task_create((Task_FuncPtr)IO_Link_Master_Stack_Task, &taskParams, NULL);

        /* create the IO-Link Master Autosen device controlTask */
        Task_Params_init(&taskParams);
        taskParams.priority = 9;  /* lower priority than the master stack task and other control tasks */
        taskParams.stackSize = 2048;
        taskParams.instance->name = "ioLink Master Autosen device control Task";
        Task_create((Task_FuncPtr)IOLink_autosenTask, &taskParams, NULL);

        /* create the IO-Link Stack Test Print Task */
        Task_Params_init(&taskParams);
        taskParams.priority = 8;  /* lowest priority */
        taskParams.stackSize = 2048;
        taskParams.instance->name = "ioLink Master Print Task";
        Task_create((Task_FuncPtr)IOLink_printTask, &taskParams, NULL);

        BIOS_start();
    }

    return 0;
}
