/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 */

/**
 *  \file dss_colorbar_test.h
 *
 *  \brief DSS Colorbar Test Header file.
 */

#ifndef DSS_COLORBAR_TEST_H_
#define DSS_COLORBAR_TEST_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/dss/dss.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if defined (SIMULATOR)
#define OSAL_DELAY_COUNT                (10U)
#else
#define OSAL_DELAY_COUNT                (1000U)
#endif

#define DISP_APP_LCD_WIDTH              (1280U)
#define DISP_APP_LCD_HEIGHT             (800U)

/* Enable the below macro to have prints on the IO Console */
#undef CIO_CONSOLE

#ifndef CIO_CONSOLE
#define DSS_log                UART_printf
#else
#define DSS_log                printf
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Test application data structure.
 */
typedef struct
{
    Fvid2_Handle dctrlHandle;
    /**< DCTRL handle */
    Dss_InitParams initParams;
    /**< DSS Initialization Parameters */
    Dss_DctrlVpParams vpParams;
    /**< VP Params */
    Dss_DctrlOverlayParams overlayParams;
    /**< Overlay Params */
#if defined (SOC_AM65XX)
    Dss_DctrlOldiParams oldiParams;
    /**< OLDI Params */
#endif
} DispApp_Obj;

/* ========================================================================== */
/*                  Internal/Private Function Declarations                   */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void App_print(const char *format, ...);

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*      Internal Function Declarations (Needed for other static inlines)      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef DSS_COLORBAR_TEST_H_ */

/* @} */
