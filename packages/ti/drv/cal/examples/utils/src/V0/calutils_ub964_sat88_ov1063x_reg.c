/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file calutils_ub964_sat88_ov1063x.c
 *
 *  \brief Implements APIs to initialize, de-initialize UB964 EVM,
 *         With SAT0088 module and the OV1063x 
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/soc.h>
#include <calutils_ub964_sat88_ov1063x.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
const CalUtils_Ub960I2cParams gUb960Cfg_SAT0088_OV10635[] = {
    {0x01, 0x01, 0xFFF},    /* Digital Reset 0 */
    {0x1F, 0x00, 0x4FF},    /* UB960 = 0x5 and UB964 = 0x00, 800 MHz DDR */
    {0x1D, 0xFF, 0x4FF},    /* Frame Count limit high byte limit to 0xFF */
    {0x1E, 0xFF, 0x4FF},    /* Frame Count limit low byte limit to 0xFF */

    /* Port 0 */
    {0x4C, 0x01, 0x0},
    {0x58, 0x58, 0x0},
    {0x5D, ((UInt8) (SAT0088_OV10635_SER_ADDR << 1U)), 0x0},
    {0x65, ((UInt8) (SAT0088_OV10635_PORT_0_SER_ALIAS_ADDR << 1U)), 0x0},
    {0x5E, ((UInt8) (SAT0088_OV10635_SENSOR_ADDR << 1U)), 0x0},
    {0x66, ((UInt8) (SAT0088_OV10635_PORT_0_SENSOR_ALIAS_ADDR << 1U)), 0x0},
    {0x7C, 0x81, 0x0},  /* SAT0088+OV10635,frame-valid polarity inversion &
                            process 8 MS bits only */
    {0x6e, 0x98, 0x0},  /*0x99: GPIO0=1 GPIO1 = 1. 0x98 = GPIO0=0 GPIO1 = 1 */
    {0x70, 0x1E, 0x0},  /* VPS_ISS_CAL_CSI2_YUV422_8B */
    {0x71, 0x2C, 0x0},
    {0x6D, 0x7F, 0x0}, /* 0x7E RAW 12, 0x7F RAW 10 */

    /* Port 1 */
    {0x4C, 0x12, 0x0},
    {0x58, 0x58, 0x0},
    {0x5D, ((UInt8) (SAT0088_OV10635_SER_ADDR << 1U)), 0x0},
    {0x65, ((UInt8) (SAT0088_OV10635_PORT_1_SER_ALIAS_ADDR << 1U)), 0x0},
    {0x5E, ((UInt8) (SAT0088_OV10635_SENSOR_ADDR << 1U)), 0x0},
    {0x66, ((UInt8) (SAT0088_OV10635_PORT_1_SENSOR_ALIAS_ADDR << 1U)), 0x0},
    {0x7C, 0x81, 0x0},
    {0x6e, 0x98, 0x0},
    {0x70, 0x5E, 0x0},  /* VPS_ISS_CAL_CSI2_YUV422_8B */
    {0x71, 0x6C, 0x0},
    {0x6D, 0x7F, 0x0}, /* 0x7E RAW 12, 0x7F RAW 10 */

    /* Port 2 */
    {0x4C, 0x24, 0x0},
    {0x58, 0x58, 0x0},
    {0x5D, ((UInt8) (SAT0088_OV10635_SER_ADDR << 1U)), 0x0},
    {0x65, ((UInt8) (SAT0088_OV10635_PORT_2_SER_ALIAS_ADDR << 1U)), 0x0},
    {0x5E, ((UInt8) (SAT0088_OV10635_SENSOR_ADDR << 1U)), 0x0},
    {0x66, ((UInt8) (SAT0088_OV10635_PORT_2_SENSOR_ALIAS_ADDR << 1U)), 0x0},
    {0x7C, 0x81, 0x0},
    {0x6e, 0x98, 0x0},
    {0x70, 0x9E, 0x0},  /* VPS_ISS_CAL_CSI2_YUV422_8B */
    {0x71, 0xAC, 0x0},
    {0x6D, 0x7F, 0x0}, /* 0x7E RAW 12, 0x7F RAW 10 */

    /* Port 3 */
    {0x4C, 0x38, 0x0},
    {0x58, 0x58, 0x0},
    {0x5D, ((UInt8) (SAT0088_OV10635_SER_ADDR << 1U)), 0x0},
    {0x65, ((UInt8) (SAT0088_OV10635_PORT_3_SER_ALIAS_ADDR << 1U)), 0x0},
    {0x5E, ((UInt8) (SAT0088_OV10635_SENSOR_ADDR << 1U)), 0x0},
    {0x66, ((UInt8) (SAT0088_OV10635_PORT_3_SENSOR_ALIAS_ADDR << 1U)), 0x0},
    {0x7C, 0x81, 0x0},
    {0x6e, 0x98, 0x0},
    {0x70, 0xDE, 0x0},  /* VPS_ISS_CAL_CSI2_YUV422_8B */
    {0x71, 0xEC, 0x0},
    {0x6D, 0x7F, 0x0}, /* 0x7E RAW 12, 0x7F RAW 10 */

    {0xB0, 0x1C, 0xFFF},
    {0xB1, 0x13, 0xFFF},
    {0xB2, 0x1F, 0xFFF},

    {0x32, 0x01, 0x0},
#ifdef STREAM_ON_2_LANES
    {0x33, 0x21, 0x0},
#else
    {0x33, 0x01, 0x0},
#endif /* STREAM_ON_2_LANES */

    {0x20, 0x00, 0xFFF}
};

const CalUtils_SensorConfigParams gUb960Cfg_Ov1063xLvdsSensorsDefault[] =
{
/* Register configuration for full resolution : 1280x720 */
    {0x103, 0x1, 0x0},    /** Software Reset */
    {0x301b, 0xff, 0x0},  /** System Control Clock Reset #1 */
    {0x301c, 0xff, 0x0},  /** System Control Clock Reset #2 */
    {0x301a, 0xff, 0x0},  /** System Control Clock Reset #0 */
    {0x300c, 0x61, 0x0},  /** Serial Camera Control Bus ID */
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x300c, 0x61, 0x0},
    {0x3021, 0x3, 0x0},  /** System Control Misc */
    {0x3011, 0x2, 0x0},
    {0x6900, 0xc, 0x0},
    {0x6901, 0x1, 0x0},
    {0x3033, 0x8, 0x0},   /** System clock/4 */
    {0x3503, 0x10, 0x0},  /** AEC Delay enabled */
    {0x302d, 0x2f, 0x0},  /** Power Down Control */
    {0x3025, 0x3, 0x0},  /** Debug Control enabled */
    /*
     * System clock is computed as:
     * XVCLK       = 24MHz
     * preDivide   = {/1,/1.5,/2,/3,/4,/5,/6,/7}
     * decided based on (0x3004[6:4].
     * numerator   = (0x3003[5:0] * XVCLK)/preDivide)
     * denominator = (2 * (1+0x3004[2:0]))
     * System clock = numerator/denominator.
     */
    {0x3003, 0x20, 0x0},    /* fps = 36fps */
    {0x3004, 0x3, 0x0},
    {0x3005, 0x20, 0x0},
    {0x3006, 0x91, 0x0},
    {0x3600, 0x74, 0x0},
    {0x3601, 0x2b, 0x0},
    {0x3612, 0x0, 0x0},
    {0x3611, 0x67, 0x0},
    {0x3633, 0xba, 0x0},
    {0x3602, 0x2f, 0x0},
    {0x3603, 0x0, 0x0},
    {0x3630, 0xa8, 0x0},
    {0x3631, 0x16, 0x0},
    {0x3714, 0x10, 0x0},
    {0x371d, 0x1, 0x0},
    {0x4300, 0x3A, 0x0}, /* UYVY mode */
    {0x3007, 0x1, 0x0},
    /*
     * RAW mode and Pixel CLK selct.
     * 0x3024[0] = 1 :: System CLK (0x3003,0x3004)
     * 0x3024[0] = 0 :: secondary CLK (0x3005,0x3006)
     */
    {0x3024, 0x1, 0x0},
    {0x3020, 0xb, 0x0},
    {0x3702, 0xd, 0x0},
    {0x3703, 0x20, 0x0},
    {0x3704, 0x15, 0x0},
    {0x3709, 0x28, 0x0},
    {0x370d, 0x0, 0x0},
    {0x3712, 0x0, 0x0},
    {0x3713, 0x20, 0x0},
    {0x3715, 0x4, 0x0},
    {0x381d, 0x40, 0x0},
    {0x381c, 0x0, 0x0},
    {0x3824, 0x10, 0x0},
    {0x3815, 0x8c, 0x0},
    {0x3804, 0x5, 0x0},
    {0x3805, 0x1f, 0x0},
    {0x3800, 0x0, 0x0},
    {0x3801, 0x0, 0x0},
    {0x3806, 0x3, 0x0},
    {0x3807, 0x1, 0x0},
    {0x3802, 0x0, 0x0},
    {0x3803, 0x2e, 0x0},
    {0x3808, 0x5, 0x0},
    {0x3809, 0x0, 0x0},
    {0x380a, 0x2, 0x0},
    {0x380b, 0xd0, 0x0},
    {0x380c, 0x6, 0x0},
    {0x380d, 0xf6, 0x0}, /* 1280x720 */
    {0x380e, 0x3, 0x0},
    {0x380f, 0x80, 0x0}, /* 1280x720 */
    {0x3811, 0x8, 0x0},
    {0x381f, 0xc, 0x0},
    {0x3621, 0x63, 0x0},
    {0x5005, 0x8, 0x0},
    {0x56d5, 0x0, 0x0},
    {0x56d6, 0x80, 0x0},
    {0x56d7, 0x0, 0x0},
    {0x56d8, 0x0, 0x0},
    {0x56d9, 0x0, 0x0},
    {0x56da, 0x80, 0x0},
    {0x56db, 0x0, 0x0},
    {0x56dc, 0x0, 0x0},
    {0x56e8, 0x0, 0x0},
    {0x56e9, 0x7f, 0x0},
    {0x56ea, 0x0, 0x0},
    {0x56eb, 0x7f,0x0},
    {0x5100, 0x0, 0x0},
    {0x5101, 0x80, 0x0},
    {0x5102, 0x0, 0x0},
    {0x5103, 0x80, 0x0},
    {0x5104, 0x0, 0x0},
    {0x5105, 0x80, 0x0},
    {0x5106, 0x0, 0x0},
    {0x5107, 0x80, 0x0},
    {0x5108, 0x0, 0x0},
    {0x5109, 0x0, 0x0},
    {0x510a, 0x0, 0x0},
    {0x510b, 0x0, 0x0},
    {0x510c, 0x0, 0x0},
    {0x510d, 0x0, 0x0},
    {0x510e, 0x0, 0x0},
    {0x510f, 0x0, 0x0},
    {0x5110, 0x0, 0x0},
    {0x5111, 0x80, 0x0},
    {0x5112, 0x0, 0x0},
    {0x5113, 0x80, 0x0},
    {0x5114, 0x0, 0x0},
    {0x5115, 0x80, 0x0},
    {0x5116, 0x0, 0x0},
    {0x5117, 0x80, 0x0},
    {0x5118, 0x0, 0x0},
    {0x5119, 0x0, 0x0},
    {0x511a, 0x0, 0x0},
    {0x511b, 0x0, 0x0},
    {0x511c, 0x0, 0x0},
    {0x511d, 0x0, 0x0},
    {0x511e, 0x0, 0x0},
    {0x511f, 0x0, 0x0},
    {0x56d0, 0x0, 0x0},
    {0x5006, 0x24, 0x0},
    {0x5608, 0x0, 0x0},
    {0x52d7, 0x6, 0x0},
    {0x528d, 0x8, 0x0},
    {0x5293, 0x12, 0x0},
    {0x52d3, 0x12, 0x0},
    {0x5288, 0x6, 0x0},
    {0x5289, 0x20, 0x0},
    {0x52c8, 0x6, 0x0},
    {0x52c9, 0x20, 0x0},
    {0x52cd, 0x4, 0x0},
    {0x5381, 0x0, 0x0},
    {0x5382, 0xff, 0x0},
    {0x5589, 0x76, 0x0},
    {0x558a, 0x47, 0x0},
    {0x558b, 0xef, 0x0},
    {0x558c, 0xc9, 0x0},
    {0x558d, 0x49, 0x0},
    {0x558e, 0x30, 0x0},
    {0x558f, 0x67, 0x0},
    {0x5590, 0x3f, 0x0},
    {0x5591, 0xf0, 0x0},
    {0x5592, 0x10, 0x0},
    {0x55a2, 0x6d, 0x0},
    {0x55a3, 0x55, 0x0},
    {0x55a4, 0xc3, 0x0},
    {0x55a5, 0xb5, 0x0},
    {0x55a6, 0x43, 0x0},
    {0x55a7, 0x38, 0x0},
    {0x55a8, 0x5f, 0x0},
    {0x55a9, 0x4b, 0x0},
    {0x55aa, 0xf0, 0x0},
    {0x55ab, 0x10, 0x0},
    {0x5581, 0x52, 0x0},
    {0x5300, 0x1, 0x0},
    {0x5301, 0x0, 0x0},
    {0x5302, 0x0, 0x0},
    {0x5303, 0xe, 0x0},
    {0x5304, 0x0, 0x0},
    {0x5305, 0xe, 0x0},
    {0x5306, 0x0, 0x0},
    {0x5307, 0x36, 0x0},
    {0x5308, 0x0, 0x0},
    {0x5309, 0xd9, 0x0},
    {0x530a, 0x0, 0x0},
    {0x530b, 0xf, 0x0},
    {0x530c, 0x0, 0x0},
    {0x530d, 0x2c, 0x0},
    {0x530e, 0x0, 0x0},
    {0x530f, 0x59, 0x0},
    {0x5310, 0x0, 0x0},
    {0x5311, 0x7b, 0x0},
    {0x5312, 0x0, 0x0},
    {0x5313, 0x22, 0x0},
    {0x5314, 0x0, 0x0},
    {0x5315, 0xd5, 0x0},
    {0x5316, 0x0, 0x0},
    {0x5317, 0x13, 0x0},
    {0x5318, 0x0, 0x0},
    {0x5319, 0x18, 0x0},
    {0x531a, 0x0, 0x0},
    {0x531b, 0x26, 0x0},
    {0x531c, 0x0, 0x0},
    {0x531d, 0xdc, 0x0},
    {0x531e, 0x0, 0x0},
    {0x531f, 0x2, 0x0},
    {0x5320, 0x0, 0x0},
    {0x5321, 0x24, 0x0},
    {0x5322, 0x0, 0x0},
    {0x5323, 0x56, 0x0},
    {0x5324, 0x0, 0x0},
    {0x5325, 0x85, 0x0},
    {0x5326, 0x0, 0x0},
    {0x5327, 0x20, 0x0},
    {0x5609, 0x1, 0x0},
    {0x560a, 0x40, 0x0},
    {0x560b, 0x1, 0x0},
    {0x560c, 0x40, 0x0},
    {0x560d, 0x0, 0x0},
    {0x560e, 0xfa, 0x0},
    {0x560f, 0x0, 0x0},
    {0x5610, 0xfa, 0x0},
    {0x5611, 0x2, 0x0},
    {0x5612, 0x80, 0x0},
    {0x5613, 0x2, 0x0},
    {0x5614, 0x80, 0x0},
    {0x5615, 0x1, 0x0},
    {0x5616, 0x2c, 0x0},
    {0x5617, 0x1, 0x0},
    {0x5618, 0x2c, 0x0},
    {0x563b, 0x1, 0x0},
    {0x563c, 0x1, 0x0},
    {0x563d, 0x1, 0x0},
    {0x563e, 0x1, 0x0},
    {0x563f, 0x3, 0x0},
    {0x5640, 0x3, 0x0},
    {0x5641, 0x3, 0x0},
    {0x5642, 0x5, 0x0},
    {0x5643, 0x9, 0x0},
    {0x5644, 0x5, 0x0},
    {0x5645, 0x5, 0x0},
    {0x5646, 0x5, 0x0},
    {0x5647, 0x5, 0x0},
    {0x5651, 0x0, 0x0},
    {0x5652, 0x80, 0x0},
    {0x521a, 0x1, 0x0},
    {0x521b, 0x3, 0x0},
    {0x521c, 0x6, 0x0},
    {0x521d, 0xa, 0x0},
    {0x521e, 0xe, 0x0},
    {0x521f, 0x12, 0x0},
    {0x5220, 0x16, 0x0},
    {0x5223, 0x2, 0x0},
    {0x5225, 0x4, 0x0},
    {0x5227, 0x8, 0x0},
    {0x5229, 0xc, 0x0},
    {0x522b, 0x12, 0x0},
    {0x522d, 0x18, 0x0},
    {0x522f, 0x1e, 0x0},
    {0x5241, 0x4, 0x0},
    {0x5242, 0x1, 0x0},
    {0x5243, 0x3, 0x0},
    {0x5244, 0x6, 0x0},
    {0x5245, 0xa, 0x0},
    {0x5246, 0xe, 0x0},
    {0x5247, 0x12, 0x0},
    {0x5248, 0x16, 0x0},
    {0x524a, 0x3, 0x0},
    {0x524c, 0x4, 0x0},
    {0x524e, 0x8, 0x0},
    {0x5250, 0xc, 0x0},
    {0x5252, 0x12, 0x0},
    {0x5254, 0x18, 0x0},
    {0x5256, 0x1e, 0x0},
    {0x4605, 0x8, 0x0},  /* 8-bit YUV mode. */
    {0x4606, 0x7, 0x0},
    {0x4607, 0x71, 0x0},
    {0x460a, 0x2, 0x0},
    {0x460b, 0x70, 0x0},
    {0x460c, 0x0, 0x0},
    {0x4620, 0xe, 0x0},
    {0x4700, 0x4, 0x0},

    {0x4701, 0x01, 0x0},
    {0x4702, 0x00, 0x0},
    {0x4703, 0x00, 0x0},
    {0x4704, 0x00, 0x0},
    /* Non-overlapping HSYNC-VSYNC.
     * Therefore do not set the VSYNC delay registers. */
    {0x4705, 0x00, 0x0},    /* Vsync delay high byte */
    {0x4706, 0x00, 0x0},    /* Vsync delay middle byte */
    {0x4707, 0x00, 0x0},    /* Vsync delay low byte */
    {0x4004, 0x8, 0x0},
    {0x4005, 0x18, 0x0},
    {0x4001, 0x4, 0x0},
    {0x4050, 0x20, 0x0},
    {0x4051, 0x22, 0x0},
    {0x4057, 0x9c, 0x0},
    {0x405a, 0x0, 0x0},
    {0x4202, 0x2, 0x0},
    {0x3023, 0x10, 0x0},
    {0x100, 0x1, 0x0},
    {0x100, 0x1, 0x0},
    {0x6f0e, 0x0, 0x0},
    {0x6f0f, 0x0, 0x0},
    {0x460e, 0x8, 0x0},
    {0x460f, 0x1, 0x0},
    {0x4610, 0x0, 0x0},
    {0x4611, 0x1, 0x0},
    {0x4612, 0x0, 0x0},
    {0x4613, 0x1, 0x0},
    {0x4605, 0x8, 0x0},
    {0x4608, 0x0, 0x0},
    {0x4609, 0x8, 0x0},
    {0x6804, 0x0, 0x0},
    {0x6805, 0x6, 0x0},
    {0x6806, 0x0, 0x0},
    {0x5120, 0x0, 0x0},
    {0x3510, 0x0, 0x0},
    {0x3504, 0x0, 0x0},
    {0x6800, 0x0, 0x0},
    {0x6f0d, 0x0, 0x0},
    {0x5000, 0xff, 0x0},
    {0x5001, 0xbf, 0x0},
    {0x5002, 0xfe, 0x0},
    {0x503d, 0x0, 0x0},
    {0xc450, 0x1, 0x0},
    {0xc452, 0x4, 0x0},
    {0xc453, 0x0, 0x0},
    {0xc454, 0x0, 0x0},
    {0xc455, 0x0, 0x0},
    {0xc456, 0x0, 0x0},
    {0xc457, 0x0, 0x0},
    {0xc458, 0x0, 0x0},
    {0xc459, 0x0, 0x0},
    {0xc45b, 0x0, 0x0},
    {0xc45c, 0x0, 0x0},
    {0xc45d, 0x0, 0x0},
    {0xc45e, 0x0, 0x0},
    {0xc45f, 0x0, 0x0},
    {0xc460, 0x0, 0x0},
    {0xc461, 0x1, 0x0},
    {0xc462, 0x1, 0x0},
    {0xc464, 0x88, 0x0},
    {0xc465, 0x0, 0x0},
    {0xc466, 0x8a, 0x0},
    {0xc467, 0x0, 0x0},
    {0xc468, 0x86, 0x0},
    {0xc469, 0x0, 0x0},
    {0xc46a, 0x40, 0x0},
    {0xc46b, 0x50, 0x0},
    {0xc46c, 0x30, 0x0},
    {0xc46d, 0x28, 0x0},
    {0xc46e, 0x60, 0x0},
    {0xc46f, 0x40, 0x0},
    {0xc47c, 0x1, 0x0},
    {0xc47d, 0x38, 0x0},
    {0xc47e, 0x0, 0x0},
    {0xc47f, 0x0, 0x0},
    {0xc480, 0x0, 0x0},
    {0xc481, 0xff, 0x0},
    {0xc482, 0x0, 0x0},
    {0xc483, 0x40, 0x0},
    {0xc484, 0x0, 0x0},
    {0xc485, 0x18, 0x0},
    {0xc486, 0x0, 0x0},
    {0xc487, 0x18, 0x0},
    {0xc488, 0x2e, 0x0},
    {0xc489, 0x80, 0x0},
    {0xc48a, 0x2e, 0x0},
    {0xc48b, 0x80, 0x0},
    {0xc48c, 0x0, 0x0},
    {0xc48d, 0x4, 0x0},
    {0xc48e, 0x0, 0x0},
    {0xc48f, 0x4, 0x0},
    {0xc490, 0x7, 0x0},
    {0xc492, 0x20, 0x0},
    {0xc493, 0x8, 0x0},
    {0xc498, 0x2, 0x0},
    {0xc499, 0x0, 0x0},
    {0xc49a, 0x2, 0x0},
    {0xc49b, 0x0, 0x0},
    {0xc49c, 0x2, 0x0},
    {0xc49d, 0x0, 0x0},
    {0xc49e, 0x2, 0x0},
    {0xc49f, 0x60, 0x0},
    {0xc4a0, 0x4, 0x0},
    {0xc4a1, 0x0, 0x0},
    {0xc4a2, 0x6, 0x0},
    {0xc4a3, 0x0, 0x0},
    {0xc4a4, 0x0, 0x0},
    {0xc4a5, 0x10, 0x0},
    {0xc4a6, 0x0, 0x0},
    {0xc4a7, 0x40, 0x0},
    {0xc4a8, 0x0, 0x0},
    {0xc4a9, 0x80, 0x0},
    {0xc4aa, 0xd, 0x0},
    {0xc4ab, 0x0, 0x0},
    {0xc4ac, 0xf, 0x0},
    {0xc4ad, 0xc0, 0x0},
    {0xc4b4, 0x1, 0x0},
    {0xc4b5, 0x1, 0x0},
    {0xc4b6, 0x0, 0x0},
    {0xc4b7, 0x1, 0x0},
    {0xc4b8, 0x0, 0x0},
    {0xc4b9, 0x1, 0x0},
    {0xc4ba, 0x1, 0x0},
    {0xc4bb, 0x0, 0x0},
    {0xc4be, 0x2, 0x0},
    {0xc4bf, 0x33, 0x0},
    {0xc4c8, 0x3, 0x0},
    {0xc4c9, 0xd0, 0x0},
    {0xc4ca, 0xe, 0x0},
    {0xc4cb, 0x0, 0x0},
    {0xc4cc, 0xe, 0x0},
    {0xc4cd, 0x51, 0x0},
    {0xc4ce, 0xe, 0x0},
    {0xc4cf, 0x51, 0x0},
    {0xc4d0, 0x4, 0x0},
    {0xc4d1, 0x80, 0x0},
    {0xc4e0, 0x4, 0x0},
    {0xc4e1, 0x2, 0x0},
    {0xc4e2, 0x1, 0x0},
    {0xc4e4, 0x10, 0x0},
    {0xc4e5, 0x20, 0x0},
    {0xc4e6, 0x30, 0x0},
    {0xc4e7, 0x40, 0x0},
    {0xc4e8, 0x50, 0x0},
    {0xc4e9, 0x60, 0x0},
    {0xc4ea, 0x70, 0x0},
    {0xc4eb, 0x80, 0x0},
    {0xc4ec, 0x90, 0x0},
    {0xc4ed, 0xa0, 0x0},
    {0xc4ee, 0xb0, 0x0},
    {0xc4ef, 0xc0, 0x0},
    {0xc4f0, 0xd0, 0x0},
    {0xc4f1, 0xe0, 0x0},
    {0xc4f2, 0xf0, 0x0},
    {0xc4f3, 0x80, 0x0},
    {0xc4f4, 0x0, 0x0},
    {0xc4f5, 0x20, 0x0},
    {0xc4f6, 0x2, 0x0},
    {0xc4f7, 0x0, 0x0},
    {0xc4f8, 0x4, 0x0},
    {0xc4f9, 0xb, 0x0},
    {0xc4fa, 0x0, 0x0},
    {0xc4fb, 0x1, 0x0},
    {0xc4fc, 0x1, 0x0},
    {0xc4fd, 0x1, 0x0},
    {0xc4fe, 0x4, 0x0},
    {0xc4ff, 0x2, 0x0},
    {0xc500, 0x68, 0x0},
    {0xc501, 0x74, 0x0},
    {0xc502, 0x70, 0x0},
    {0xc503, 0x80, 0x0},
    {0xc504, 0x5, 0x0},
    {0xc505, 0x80, 0x0},
    {0xc506, 0x3, 0x0},
    {0xc507, 0x80, 0x0},
    {0xc508, 0x1, 0x0},
    {0xc509, 0xc0, 0x0},
    {0xc50a, 0x1, 0x0},
    {0xc50b, 0xa0, 0x0},
    {0xc50c, 0x1, 0x0},
    {0xc50d, 0x2c, 0x0},
    {0xc50e, 0x1, 0x0},
    {0xc50f, 0xa, 0x0},
    {0xc510, 0x0, 0x0},
    {0xc511, 0x0, 0x0},
    {0xc512, 0xe5, 0x0},
    {0xc513, 0x14, 0x0},
    {0xc514, 0x4, 0x0},
    {0xc515, 0x0, 0x0},
    {0xc518, 0x3, 0x0},
    {0xc519, 0x48, 0x0},
    {0xc51a, 0x7, 0x0},
    {0xc51b, 0x70, 0x0},
    {0xc2e0, 0x0, 0x0},
    {0xc2e1, 0x51, 0x0},
    {0xc2e2, 0x0, 0x0},
    {0xc2e3, 0xd6, 0x0},
    {0xc2e4, 0x1, 0x0},
    {0xc2e5, 0x5e, 0x0},
    {0xc2e9, 0x1, 0x0},
    {0xc2ea, 0x7a, 0x0},
    {0xc2eb, 0x90, 0x0},
    {0xc2ed, 0x1, 0x0},
    {0xc2ee, 0x7a, 0x0},
    {0xc2ef, 0x64, 0x0},
    {0xc308, 0x0, 0x0},
    {0xc309, 0x0, 0x0},
    {0xc30a, 0x0, 0x0},
    {0xc30c, 0x0, 0x0},
    {0xc30d, 0x1, 0x0},
    {0xc30e, 0x0, 0x0},
    {0xc30f, 0x0, 0x0},
    {0xc310, 0x1, 0x0},
    {0xc311, 0x60, 0x0},
    {0xc312, 0xff, 0x0},
    {0xc313, 0x8, 0x0},
    {0xc314, 0x1, 0x0},
    {0xc315, 0x7f, 0x0},
    {0xc316, 0xff, 0x0},
    {0xc317, 0xb, 0x0},
    {0xc318, 0x0, 0x0},
    {0xc319, 0xc, 0x0},
    {0xc31a, 0x0, 0x0},
    {0xc31b, 0xe0, 0x0},
    {0xc31c, 0x0, 0x0},
    {0xc31d, 0x14, 0x0},
    {0xc31e, 0x0, 0x0},
    {0xc31f, 0xc5, 0x0},
    {0xc320, 0xff, 0x0},
    {0xc321, 0x4b, 0x0},
    {0xc322, 0xff, 0x0},
    {0xc323, 0xf0, 0x0},
    {0xc324, 0xff, 0x0},
    {0xc325, 0xe8, 0x0},
    {0xc326, 0x0, 0x0},
    {0xc327, 0x46, 0x0},
    {0xc328, 0xff, 0x0},
    {0xc329, 0xd2, 0x0},
    {0xc32a, 0xff, 0x0},
    {0xc32b, 0xe4, 0x0},
    {0xc32c, 0xff, 0x0},
    {0xc32d, 0xbb, 0x0},
    {0xc32e, 0x0, 0x0},
    {0xc32f, 0x61, 0x0},
    {0xc330, 0xff, 0x0},
    {0xc331, 0xf9, 0x0},
    {0xc332, 0x0, 0x0},
    {0xc333, 0xd9, 0x0},
    {0xc334, 0x0, 0x0},
    {0xc335, 0x2e, 0x0},
    {0xc336, 0x0, 0x0},
    {0xc337, 0xb1, 0x0},
    {0xc338, 0xff, 0x0},
    {0xc339, 0x64, 0x0},
    {0xc33a, 0xff, 0x0},
    {0xc33b, 0xeb, 0x0},
    {0xc33c, 0xff, 0x0},
    {0xc33d, 0xe8, 0x0},
    {0xc33e, 0x0, 0x0},
    {0xc33f, 0x48, 0x0},
    {0xc340, 0xff, 0x0},
    {0xc341, 0xd0, 0x0},
    {0xc342, 0xff, 0x0},
    {0xc343, 0xed, 0x0},
    {0xc344, 0xff, 0x0},
    {0xc345, 0xad, 0x0},
    {0xc346, 0x0, 0x0},
    {0xc347, 0x66, 0x0},
    {0xc348, 0x1, 0x0},
    {0xc349, 0x0, 0x0},
    {0x6700, 0x4, 0x0},
    {0x6701, 0x7b, 0x0},
    {0x6702, 0xfd, 0x0},
    {0x6703, 0xf9, 0x0},
    {0x6704, 0x3d, 0x0},
    {0x6705, 0x71, 0x0},
    /*
     * 0x6706[3:0] :: XVCLK
     * 0x6706[3:0] :: 0 = 6MHz
     * 0x6706[3:0] :: 1 = 9MHz
     * 0x6706[3:0] :: 8 = 24MHz
     * 0x6706[3:0] :: 9 = 27MHz
     */
    {0x6706, 0x78, 0x0},
    {0x6708, 0x5, 0x0},
    {0x3822, 0x50, 0x0},
    {0x6f06, 0x6f, 0x0},
    {0x6f07, 0x0, 0x0},
    {0x6f0a, 0x6f, 0x0},
    {0x6f0b, 0x0, 0x0},
    {0x6f00, 0x3, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x3042, 0xf0, 0x0},
    {0x301b, 0xf0, 0x0},
    {0x301c, 0xf0, 0x0},
    {0x301a, 0xf0, 0x0},
    {0x0100, 0x00, 0x0},
    {0x4300, 0x38, 0x0},
    {0x0100, 0x01, 0x0},
};

uint32_t gUb960Cfg_SAT0088_OV10635_size = sizeof(gUb960Cfg_SAT0088_OV10635) / sizeof(gUb960Cfg_SAT0088_OV10635[0]);

uint32_t gUb960Cfg_Ov1063xLvdsSensorsDefault_size = sizeof(gUb960Cfg_Ov1063xLvdsSensorsDefault) / sizeof(gUb960Cfg_Ov1063xLvdsSensorsDefault[0]);

/* ========================================================================== */
/*                          Function Implementation                           */
/* ========================================================================== */
