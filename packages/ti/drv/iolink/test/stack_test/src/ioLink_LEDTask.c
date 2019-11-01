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
 *              file:    ioLink_LEDTask.c
 *
 *              brief:   PRU IO-Link master LED driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/cfg/global.h>

#include <ti/drv/iolink/test/stack_test/src/ioLink_LEDTask.h>
#include <ti/drv/iolink/test/stack_test/src/ioLink_TLC59281.h>

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

#define PORTS           8
#define LEDSPERPORT     2

static enum ledenum leds[PORTS][LEDSPERPORT];
static int led_timer[PORTS][LEDSPERPORT];

struct led_mapping_struct{
    int port;
    int led;
};

struct LEDMSGstruct{
    int port;
    int led;
    enum ledenum state;
};

/*
 * defines how LEDs are mapped
 * index is the LED number on GPIO or LED driver
 * each entry defines the IO-Link port and LED number on this port
 */
 static const struct led_mapping_struct led_mapping[] =
{
 {3, 1}, /* 1st LED -> port 4 LED 1 */
 {3, 0}, /* 2ns LED -> port 4 LED 0 */
 {2, 0},
 {2, 1},
 {0, 0},
 {0, 1},
 {6, 1},
 {6, 0},
 {4, 0},
 {4, 1},
 {5, 1},
 {5, 0},
 {7, 1},
 {7, 0},
 {1, 1},
 {1, 0},
};

#define SLOWBLINKTIME   500 /* 500 ms */
#define FASTBLINKTIME   200 /* 200 ms */
#define TICKRATE        100 /* 100 ms */

struct led_cfg_struct{
    void(*initLEDs)();
    void(*deinitLEDs)();
    void(*updateLEDs)(uint32_t);
};

/* define which functions have to be used to control the LEDs */
static struct led_cfg_struct IOLink_leds =
{
     TLC59281_initLEDs,
     TLC59281_deInitLEDs,
     TLC59281_updateLEDs,
};

static Mailbox_Handle mBoxLEDs;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void IO_Link_LEDs(int port, int led, enum ledenum state){
    struct LEDMSGstruct msg;
    if(mBoxLEDs == 0) return;
    msg.port = port;
    msg.led = led;
    msg.state = state;
    Mailbox_post(mBoxLEDs, &msg, BIOS_WAIT_FOREVER);
}

void IOLink_LEDTask(UArg arg0){
    uint32_t led_state=0;
    int i;
    struct LEDMSGstruct msg;
    IOLink_leds.initLEDs();

    Mailbox_Params mboxParams;
    Error_Block eb;
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    mBoxLEDs = Mailbox_create(sizeof(msg), 5, &mboxParams, &eb);
    if (mBoxLEDs == NULL) {
        System_printf("taskFxn(): %s\n", Error_getMsg(&eb) );
        System_abort("Mailbox create failed");
    }

    while(1){

        while(Mailbox_pend(mBoxLEDs, &msg, 1) == TRUE){
            if(msg.port < PORTS && msg.led < LEDSPERPORT)
                leds[msg.port][msg.led] = msg.state;
        }

        for(i=0; i<sizeof(led_mapping)/sizeof(led_mapping[0]); i++){
            led_timer[led_mapping[i].port][led_mapping[i].led]++;

            switch(leds[led_mapping[i].port][led_mapping[i].led]){
                case led_off:
                    led_state &= ~(1<<i);
                    break;
                case led_on:
                    led_state |= 1<<i;
                    break;
                case led_blink_slow:
                    if(led_timer[led_mapping[i].port][led_mapping[i].led] > SLOWBLINKTIME/TICKRATE){
                        led_state ^= 1<<i;
                        led_timer[led_mapping[i].port][led_mapping[i].led] = 0;
                    }
                    break;
                case led_blink_fast:
                    if(led_timer[led_mapping[i].port][led_mapping[i].led] > FASTBLINKTIME/TICKRATE){
                        led_state ^= 1<<i;
                        led_timer[led_mapping[i].port][led_mapping[i].led] = 0;
                    }
                    break;
            }
        }

        IOLink_leds.updateLEDs(led_state);
        Task_sleep(200); /* tick time is 500us ?! */
    }
}
