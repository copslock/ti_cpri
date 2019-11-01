/*
 *  Copyright (C) 2018 Texas Instruments Incorporated
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
 *  \file sciclient_fmwSecureProxyMap.c
 *
 *  \brief File containing the secure proxy map for all hosts.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/sciclient/src/sciclient_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

const Sciclient_MapStruct_t gSciclientMap[SCICLIENT_CONTEXT_MAX_NUM] =
{
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MCU_0_R5_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_MCU_R5FSS0_CORE0_INTR_MCU_NAVSS0_INTR_ROUTER_0_OUTL_INTR_1
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MCU_0_R5_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_MCU_R5FSS0_CORE0_INTR_MCU_NAVSS0_INTR_ROUTER_0_OUTL_INTR_3
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_2,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_2_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_2_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MCU_0_R5_2_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_2_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_2_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_MCU_R5FSS0_CORE1_INTR_MCU_NAVSS0_INTR_ROUTER_0_OUTL_INTR_33
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_3,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_3_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_3_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MCU_0_R5_3_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_3_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MCU_0_R5_3_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_MCU_R5FSS0_CORE1_INTR_MCU_NAVSS0_INTR_ROUTER_0_OUTL_INTR_35
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A72_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_A72_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_1
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A72_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_A72_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_3
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A72_2,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_2_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_2_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_A72_2_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_2_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_2_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_5
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A72_3,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_3_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_3_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_A72_3_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_3_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_3_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_7
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A72_4,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_4_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A72_4_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_A72_4_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_4_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_A72_4_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_9
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_C7X_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C7X_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C7X_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_C7X_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_C7X_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_C7X_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        SCICLIENT_C7X_SECURE_INTERRUPT_NUM
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_C7X_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C7X_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C7X_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_C7X_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_C7X_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_C7X_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        SCICLIENT_C7X_NON_SECURE_INTERRUPT_NUM
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_C6X_0_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_0_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_0_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_C6X_0_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_0_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_0_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        5U
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_C6X_0_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_0_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_0_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_C6X_0_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_0_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_0_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        7U
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_C6X_1_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_1_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_1_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_C6X_1_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_1_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_1_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        5U
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_C6X_1_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_1_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_C6X_1_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_C6X_1_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_1_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_C6X_1_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        7U
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_GPU_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_GPU_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_GPU_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_GPU_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        0
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_0_R5_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS0_CORE0_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_193
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_0_R5_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS0_CORE0_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_195
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_2,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_2_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_2_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_0_R5_2_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_2_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_2_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS0_CORE1_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_225
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_3,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_3_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_3_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_0_R5_3_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_3_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_3_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS0_CORE1_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_227
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_1_R5_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_1_R5_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS1_CORE0_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_257
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_1_R5_1,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_1_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_1_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_1_R5_1_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_1_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_1_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS1_CORE0_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_259
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_1_R5_2,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_2_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_2_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_1_R5_2_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_2_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_2_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS1_CORE1_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_289
    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_1_R5_3,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_3_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_3_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_MAIN_1_R5_3_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_3_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_1_R5_3_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        CSLR_R5FSS1_CORE1_INTR_NAVSS0_INTR_ROUTER_0_OUTL_INTR_291
    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,
        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_ICSSG_0,
        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_WRITE_HIGH_PRIORITY_THREAD_ID,
        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_WRITE_LOW_PRIORITY_THREAD_ID,
        /** Thread ID of the thread(write) for sending a notification to the firmware */
        TISCI_SEC_PROXY_ICSSG_0_WRITE_NOTIFY_RESP_THREAD_ID,
        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_READ_RESPONSE_THREAD_ID,
        /** Thread ID of the notification thread(read) available for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_READ_NOTIFY_THREAD_ID,
        /** Notification Interrupt Number.  */
        0
    }
};
