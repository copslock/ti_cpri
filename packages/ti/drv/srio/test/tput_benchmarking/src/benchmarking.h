/*
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/**
 *   @file  benchmarking.h
 *
 *   @brief
 *      Header file which has common data structure and definitions used
 *      by the benchmarking application.
 *
 */

#ifndef __BENCHMARKING_H__
#define __BENCHMARKING_H__

/* CSL Includes */
#include <ti/csl/csl_qm_queue.h>

/* SRIO Driver Includes */
#include <ti/drv/srio/srio_drv.h>

/**************************************************************************\
* Test control enums. (Please do not change these enums)
\**************************************************************************/
/** These are the possible values Tests to Run */
typedef enum 
{
  srio_nwrite_tests = 0,                        /**< Run DIO NWRITE tests only */
  srio_nread_tests,                             /**< Run DIO NREAD tests only */
  srio_nwrite_nread_tests,                      /**< Run DIO NWRITE and NREAD tests only */
  srio_type11_tests,                            /**< Run Message Type-11 tests only */
  srio_all_available_tests                      /**< Run all available tests */
} srioTestsToRun_e;

/** These are the possible values for SRIO lane speed in Gbps */
typedef enum 
{
  srio_lane_rate_1p250Gbps = 0,                 /**< Rate of 1.250Gbps for SRIO lanes */
  srio_lane_rate_2p500Gbps,                     /**< Rate of 2.500Gbps for SRIO lanes */
  srio_lane_rate_3p125Gbps,                     /**< Rate of 3.125Gbps for SRIO lanes */
  srio_lane_rate_5p000Gbps                      /**< Rate of 5.000Gbps for SRIO lanes */
} srioLaneRateGbps_e;

/** These are the possible values for SRIO lane mode */
typedef enum 
{
  srio_lanes_form_four_1x_ports = 0,             /**< SRIO lanes form four 1x ports */
  srio_lanes_form_one_2x_port_and_two_1x_ports,  /**< SRIO lanes form one 2x port and two 1x ports */
  srio_lanes_form_two_1x_ports_and_one_2x_port,  /**< SRIO lanes form two 1x ports and one 2x port */
  srio_lanes_form_two_2x_ports,                  /**< SRIO lanes form two 2x ports */
  srio_lanes_form_one_4x_port                    /**< SRIO lanes form one 4x port */
} srioLanesMode_e;


/**************************************************************************\
*
* Test control default vaules. (User changeable values)
*
\**************************************************************************/

/* SRIO Lane Rate.(Please see
 * srioLaneRateGbps_e enum above
 * for valid values).
 */
#define SRIO_LANE_SPEED                   srio_lane_rate_5p000Gbps
//#define SRIO_LANE_SPEED                   srio_lane_rate_3p125Gbps

/* SRIO Port Width.(Please see
 * srioLanesMode_e enum above
 * for valid values).
 */
#define SRIO_PORT_WIDTH                   srio_lanes_form_one_4x_port

/* Tests to Run.(Please see
 * srioTestsToRun_e enum above
 * for valid values).
 */
#define TESTS_TO_RUN                      srio_all_available_tests

/* Number of seconds to
 * run each packet size.
 */
#define NUM_SECS_TO_RUN_EACH_PACKET_SIZE  5

/* Board to Board flag. This is used to
 * have the output data indicate that
 * the test is running board to board.
 */
#define IS_BOARD_TO_BOARD                 FALSE

/* External SRIO Switch flag. This is
 * used to have the output data
 * indicate that the test is running
 * over an external SRIO switch.
 */
#define IS_OVER_EXTERNAL_SRIO_SWITCH      FALSE

/* Loopback Mode Enable.
 */
#define USE_LOOPBACK_MODE                 TRUE

/* ID bit size selection.
 */
#define USE_SRIO_16_BIT_ID                FALSE

/* Enable running latency measurement.
 */
#define RUN_LATENCY_MEASUREMENT           TRUE

/* Message (Type-11) DATA Sizes:
 * (data size is multiplied by 2 to
 *  get the next data size to run, up
 *  to the maximum data size.)
 * Type-11 supports up to a 4096 byte
 * payload. Because of an added 12
 * byte software header the minimum
 * size should be 16 or greater.
 */
#define MESSAGE_INITIAL_DATA_SIZE         16
#define MESSAGE_MAX_DATA_SIZE             4096

/* DIO DATA Sizes:
 * (data size is multiplied by 2 to
 *  get the next data size to run, up
 *  to the maximum data size.)
 * DIO supports up to a 1megabyte
 * range, but since the buffers reside
 * in L2 memory the maximum value here
 * should not go above 8192 or memory
 * and heap problems will occur.
 */
#define DIO_INITIAL_DATA_SIZE             4
#define DIO_MAX_DATA_SIZE                 8192

/* Core Definitions on which the
 * Producer and Consumer will be
 * executed.
 */
#define CONSUMER_CORE                     0x0
#define PRODUCER_CORE                     0x1

/* sRIO Initialization CORE:
 * For core to core on the same EVM:
 *   This should be CONSUMER_CORE
 *.
 * For board to board:
 *   RX side EVM (Consumer):
 *     This should be CONSUMER_CORE
 *     .out file to be loaded on core 0.
 *   TX side EVM (Producer):
 *     This should be PRODUCER_CORE,
 *     .out file to be loaded on core 1.
 */
#define CORE_TO_INITIALIZE_SRIO           CONSUMER_CORE

/* Producer Device Identifiers.
 */
#define PRODUCER_16BIT_DEVICE_ID          0xDEAD 
#define PRODUCER_8BIT_DEVICE_ID           0xDE

/* Consumer Device Identifiers.
 */
#define CONSUMER_16BIT_DEVICE_ID          0xBEEF
#define CONSUMER_8BIT_DEVICE_ID           0xBE

/* Type11: Consumer Mailbox and
 *         Letter Configuration.
 */
#define CONSUMER_MBOX                     SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE
#define CONSUMER_LTR                      SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE
#define CONSUMER_CTRL_MBOX                0
#define CONSUMER_CTRL_LTR                 1

/* Type11: Producer Mailbox and
 *         Letter Configuration.
 */
#define PRODUCER_MBOX                     0
#define PRODUCER_LTR                      1
#define PRODUCER_CTRL_MBOX                0
#define PRODUCER_CTRL_LTR                 1

/* DIO: Consumer Doorbell Register
 *      and Bit Information.
 */
#define CONSUMER_DOORBELL_REG             0
#define CONSUMER_DOORBELL_BIT             1

/* DIO: Producer Doorbell Register
 * and Bit Information.
 */
#define PRODUCER_DOORBELL_REG             0
#define PRODUCER_DOORBELL_BIT             2

/* SRIO Port to be used.
 */
#define SRIO_PORT_NUM                     0

/* To display progress message during run.
 * If set to TRUE then the application
 * will display progress messages during
 * the measurement run. If set to FALSE
 * on progress message will be displayed.
 * In either cases the tab delimited display
 * will always be output. This will add
 * quite a bit of extra lines to the
 * console output. Really only for debug.
 */
#define DISPLAY_RUN_PROGRESS_MESSAGES     FALSE

/* Base number for the start of the
 * high priority accumulation and
 * interrupt support queues to use
 * for this application.
 */
#define HIGH_PRIORITY_ACC_INT_QUEUE_BASE  QMSS_HIGH_PRIORITY_QUEUE_BASE

/**************************************************************************\
*
* End of Test control default vaules. (End of user changeable values)
*
\**************************************************************************/



/**************************************************************************\
* Please do not change the values below this line
\**************************************************************************/

/* Control message definitions */
#define CTRL_SUCCESS                      0x99
#define CTRL_FAILURE                      0x11
#define CTRL_TIMEOUT                      0x22
#define CTRL_READY                        0x33
#define CTRL_RESET                        0xAA
#define CTRL_NEXT                         0xBB
#define CTRL_CONTINUE                     0xCC
#define CTRL_STOP                         0xEE
#define CTRL_LATENCY                      0xFA

#define CHECK_NON_BLOCKING                FALSE
#define CHECK_BLOCKING                    TRUE

typedef enum ProdCons_ControlPktType
{
    ProdCons_ControlPktType_REQUEST     = 1,
    ProdCons_ControlPktType_REPLY       = 2,
    ProdCons_ControlPktType_CTRL_MSG    = 3
}ProdCons_ControlPktType;

/* Control Packet Exchange between Producer and Consumer has the 
 * following Packet Format. */
typedef struct ProdCons_ControlPkt
{
    /* Type of Packet which is being exchanged. (4 bytes) */
    ProdCons_ControlPktType        pktType;

    /* Payload Length in the packet. (4 bytes) */
    uint32_t                       payloadLength;

    /* Packet Identifier. (4 bytes) */
    uint32_t                       id;

    /* Data Payload of the packet. Data Payload will be the packet size minus 12 bytes for the
     * packet type, payload length and packet identifier which are declared above.
     */
    uint8_t                        pktPayload[MESSAGE_MAX_DATA_SIZE - 12];
}ProdCons_ControlPkt;

#endif /* __BENCHMARKING_H__ */


