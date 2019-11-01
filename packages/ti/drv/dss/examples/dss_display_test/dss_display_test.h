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
 *  \file dss_display_test.h
 *
 *  \brief DSS display Test Header file.
 */

#ifndef DSS_DISPLAY_TEST_H_
#define DSS_DISPLAY_TEST_H_

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
#define DISP_APP_RUN_COUNT              (0x10U)
#else
#define DISP_APP_RUN_COUNT              (0x1000U)
#endif

#if defined (SOC_AM65XX)
#define DISP_APP_LCD_WIDTH              (1280U)
#define DISP_APP_LCD_HEIGHT             (800U)
#else
#define DISP_APP_LCD_WIDTH              (1920U)
#define DISP_APP_LCD_HEIGHT             (1080U)
#endif
/* Worst case frames per handle */
#define DISP_APP_MAX_FRAMES_PER_HANDLE    (2U)

/* Test Params */
#define DISP_APP_BGRA32_1                 (1U)
#define DISP_APP_BGRA32_2                 (2U)
#define DISP_APP_ARGB32                   (3U)
#define DISP_APP_RGB24                    (4U)
#define DISP_APP_BGR24                    (5U)
#define DISP_APP_YUV2                     (6U)
#define DISP_APP_UVVY                     (7U)
#define DISP_APP_YUV420                   (8U)
#define DISP_APP_YUV420_12                (9U)
#define DISP_APP_RGB565                   (10U)
#define DISP_APP_BGR565                   (11U)
#define DISP_APP_BGRA32                   (12U)

/* Test Params to be used. Possible values:
 * 1U: Test VID1 and VIDL1
 * 2U: Test VID2 and VIDL2 (only for J7)
 * 3U: Test VID1 ARGB32
 * 4U: Test VID1 RGB24
 * 5U: Test VID1 BGR24
 * 6U: Test VID1 YUV422I-YUYV (only for J7)
 * 7U: Test VID1 YUV422I-UYVY (only for J7)
 * 8U: Test VID1 YUV420 (only for J7)
 * 9U: Test VID1 YUV420 12 bit (Only for J7)
 * 10U: Test VID1 RGB565 (Only for J7)
 * 11U: Test VID1 BGR565 (Only for J7)
 * 12U: Test VID1 BGRA32 (Only for J7, eDP)
 */
#define DISP_APP_USE_TEST_PARAMS          (DISP_APP_BGRA32_1)

/* Load buffers runtime for RGB24 and YUV formats */
#define DISP_APP_LOAD_BUFFERS_RUNTIME     (0U)

#define DISP_APP_DDR_LOAD_ADDRESS         (0x82000000U)

#define DISP_APP_ENBALE_PIPE_CROP         (0U)

#define DISP_APP_ENABLE_COMMON1_REGION    (0U)

#define DISP_APP_TEST_OVERLAY_VP_4        (0U)

#define DISP_APP_ENABLE_FLIP              (0U)

#define DISP_APP_RAW_DATA_INPUT           (0U)

#define DISP_APP_TEST_EDP                 (0U)

#define DISP_APP_TEST_MULTISYNC           (0U)

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
 *  \brief Display application test parameters.
 *  The test case execution happens based on values of this structure
 */
typedef struct
{
    uint32_t numTestPipes;
    /**< Number of pipes in test params */
    uint32_t bpp;
    /**< Number of bytes per pixel */
    uint32_t instId[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Driver instance id */
    uint32_t pipeId[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Pipe id */
    uint32_t pipeNodeId[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Pipe Node id */
    uint32_t pipeType[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Video pipe type */
    uint32_t inDataFmt[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Data format */
    uint32_t inWidth[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Input buffer resolution width in pixels */
    uint32_t inHeight[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Input buffer resolution height in lines */
    uint32_t pitch[CSL_DSS_VID_PIPE_ID_MAX][FVID2_MAX_PLANES];
    /**< Pitch of input buffer */
    uint32_t inScanFmt[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Scan format */
    uint32_t outWidth[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Output buffer resolution width in pixels */
    uint32_t outHeight[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Output buffer resolution height in lines */
    uint32_t scEnable[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Scaler enable */
    uint32_t globalAlpha[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Global Alpha value */
    uint32_t preMultiplyAlpha[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Pre-multiply Alpha value */
    uint32_t posx[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Input buffer position x. */
    uint32_t posy[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Input buffer position y. */
    uint32_t invalidPipeId[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Pipe id */
} DispApp_TestParams;

/**
 *  \brief Driver instance information.
 */
typedef struct
{
    uint32_t instId;
    /**< Instance ID */
    Dss_DispCreateParams createParams;
    /**< Create time parameters */
    Dss_DispCreateStatus createStatus;
    /**< Create status returned by driver during Fvid2_create() */
    Dss_DispParams dispParams;
    /**< DSS display parameters */
    Dss_DispPipeMflagParams mflagParams;
    /**< DSS mflag parameters */
    Fvid2_Handle drvHandle;
    /**< FVID2 display driver handle */
    Fvid2_CbParams cbParams;
    /**< Callback parameters */
    Fvid2_Frame frames[DISP_APP_MAX_FRAMES_PER_HANDLE];
    /**< FVID2 Frames that will be used for display */
    SemaphoreP_Handle syncSem;
    /**< Semaphore for ISR */
} DispApp_InstObj;

/**
 *  \brief Test application data structure.
 */
typedef struct
{
    DispApp_InstObj instObj[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Display driver instance objects */
    Fvid2_Handle dctrlHandle;
    /**< DCTRL handle */
    Dss_InitParams initParams;
    /**< DSS Initialization Parameters */
    Dss_DctrlPathInfo dctrlPathInfo;
    /**< DSS Path Information */
    Dss_DctrlVpParams vpParams;
    /**< VP Params */
    Dss_DctrlVpParams syncVpParams;
    /**< VP Params for synchronised VP */
    Dss_DctrlOverlayParams overlayParams;
    /**< Overlay Params */
    Dss_DctrlOverlayLayerParams layerParams;
    /**< Layer Params */
    Dss_DctrlVpErrorStats errorStats;
    /**< Error Stats */
    Dss_DctrlAdvVpParams advVpParams;
    /**< Advance VP Params */
    Dss_DctrlAdvVpParams syncAdvVpParams;
    /**< Advance VP Params for Synchronised VP */
    Dss_DctrlGlobalDssParams globalDssParams;
    /**< Global DSS Params */
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

#if (DISP_APP_BGRA32 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    4U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_BGRA32_8888
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*4U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_BGR565 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    2U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_BGR16_565
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*2U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_RGB565 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    2U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_RGB16_565
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*2U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_YUV420_12 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp, not used in this case */
    3U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_YUV420SP_UV
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*2U, 1920U*2U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_YUV420 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp, not used in this case */
    3U/2U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_YUV420SP_UV
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U, 1920U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_UVVY == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    2U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_YUV422I_UYVY
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*2U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_YUV2 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    2U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_YUV422I_YUYV
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*2U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_BGR24 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    3U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_BGR24_888
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*3U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_RGB24 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    3U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_RGB24_888
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*3U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_ARGB32 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    1U,
    /* bpp */
    4U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID
    },
    /* Data format */
    {
        FVID2_DF_ARGB32_8888
    },
    /* Input buffer width */
    {
        1920U
    },
    /* Input buffer height */
    {
        1080U
    },
    /* Pitch */
    {
        {
            1920U*4U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        DISP_APP_LCD_WIDTH
    },
    /* Output buffer height */
    {
        DISP_APP_LCD_HEIGHT
    },
    /* Scaler enable */
    {
        TRUE
    },
    /* Global Alpha */
    {
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE
    },
    /* X Position */
    {
        0U
    },
    /* Y position */
    {
        0U
    },
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VIDL1,
#if defined (SOC_J721E)
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
#endif
    }
};
#elif (DISP_APP_BGRA32_2 == DISP_APP_USE_TEST_PARAMS)
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    2U,
    /* bpp */
    4U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID2,
        DSS_DISP_INST_VIDL2
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID2,
        DSS_DCTRL_NODE_VIDL2
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID,
        CSL_DSS_VID_PIPE_TYPE_VIDL
    },
    /* Data format */
    {
        FVID2_DF_BGRA32_8888,
        FVID2_DF_BGRA32_8888
    },
    /* Input buffer width */
    {
        480U,
        480U
    },
    /* Input buffer height */
    {
        360U,
        360U
    },
    /* Pitch */
    {
        {
            480U*4U, 0U, 0U, 0U, 0U, 0U
        },
        {
            480U*4U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE,
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        360U,
        480U
    },
    /* Output buffer height */
    {
        240U,
        360U
    },
    /* Scaler enable */
    {
        TRUE,
        FALSE
    },
    /* Global Alpha */
    {
        0xFFU,
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE,
        FALSE
    },
#if defined (SOC_AM65XX)
    /* X Position */
    {
        0U,
        800U
    },
    /* Y position */
    {
        0U,
        440U
    },
#else
    /* X Position */
    {
        0U,
        1440U
    },
    /* Y position */
    {
        0U,
        720U
    },
#endif
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1,
        CSL_DSS_VID_PIPE_ID_VIDL1
    }
};
#else
static const DispApp_TestParams gDispAppTestParams=
{
    /* Number of Pipes */
    2U,
    /* bpp */
    4U,
    /* Instance Id */
    {
        DSS_DISP_INST_VID1,
        DSS_DISP_INST_VIDL1
    },
    /* Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID1,
        CSL_DSS_VID_PIPE_ID_VIDL1
    },
    /* Pipe Node Id */
    {
        DSS_DCTRL_NODE_VID1,
        DSS_DCTRL_NODE_VIDL1
    },
    /* Pipe Type */
    {
        CSL_DSS_VID_PIPE_TYPE_VID,
        CSL_DSS_VID_PIPE_TYPE_VIDL
    },
    /* Data format */
    {
        FVID2_DF_BGRA32_8888,
        FVID2_DF_BGRA32_8888
    },
    /* Input buffer width */
    {
        480U,
        480U
    },
    /* Input buffer height */
    {
        360U,
        360U
    },
    /* Pitch */
    {
        {
            480U*4U, 0U, 0U, 0U, 0U, 0U
        },
        {
            480U*4U, 0U, 0U, 0U, 0U, 0U
        }
    },
    /* Scan format */
    {
        FVID2_SF_PROGRESSIVE,
        FVID2_SF_PROGRESSIVE
    },
    /* Output buffer width */
    {
        720U,
        480U
    },
    /* Output buffer height */
    {
        540U,
        360U
    },
    /* Scaler enable */
    {
        TRUE,
        FALSE
    },
    /* Global Alpha */
    {
        0xFFU,
        0xFFU
    },
    /* Pre-multiply alpha */
    {
        FALSE,
        FALSE
    },
#if defined (SOC_AM65XX)
    /* X Position */
    {
        0U,
        800U
    },
    /* Y position */
    {
        0U,
        440U
    },
#else
    /* X Position */
    {
        0U,
        1440U
    },
    /* Y position */
    {
        0U,
        720U
    },
#endif
#if defined (SOC_J721E)
    /* Invalid Pipe Id */
    {
        CSL_DSS_VID_PIPE_ID_VID2,
        CSL_DSS_VID_PIPE_ID_VIDL2
    }
#endif
};
#endif

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

#endif /* #ifndef DSS_DISPLAY_TEST_H_ */

/* @} */
