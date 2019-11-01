/*
 *  Copyright (C) 2018-2019 Texas Instruments Incorporated
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
 * \ingroup DRV_SCICLIENT_MODULE
 * \defgroup SCICLIENT_FMW_RM_IF Sciclient RM API Interface
 *
 * The DMSC firmware Resource Management (RM) (sub) system manages SoC shared
 * resources.  RM manages access and configuration of shared resources amongst
 * SoC processing entities.  RM provides a set of interfaces over which SoC
 * processing entities can allocate, configure, and free shared resources.
 *
 *
 * @{
 */
/**
 *  \file   sciclient_rm.h
 *
 *  \brief  This file contains the definition of all the message IDs, message
 *          formats to be able to interact with the System Controller firmware
 *          for resource management.
 */

#ifndef SCICLIENT_RM_H_
#define SCICLIENT_RM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/**
 *  \brief Retrieves a host's assigned range for a resource
 *
 *  Returns the range for a unique resource type assigned to the specified host,
 *  or secondary host.  The unique resource type is formed by combining the
 *  10 LSB of type and the 6 LSB of subtype.
 *
 *  Unique types which do not map to an SoC resource will not be NACK'd.  Instead
 *  the tisci_msg_rm_get_resource_range_resp range_start and range_num values are
 *  zeroed.  This provides a known response mechanism across varied SoCs.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_GET_RESOURCE_RANGE
 *  \n<b>Request</b>:    #tisci_msg_rm_get_resource_range_req
 *  \n<b>Response</b>:   #tisci_msg_rm_get_resource_range_resp
 *
 *  \param  req             Pointer to resource range get payload
 *
 *  \param  resp            Pointer to resource range response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmGetResourceRange(
                const struct tisci_msg_rm_get_resource_range_req *req,
                struct tisci_msg_rm_get_resource_range_resp *resp,
                uint32_t timeout);

/**
 *  \brief Configures a peripheral to processor IRQ
 *
 *  Configures an interrupt route between the peripheral and host processor
 *  specified within the #tisci_msg_rm_irq_set_req payload.  The interrupt destination
 *  is either the processor sending the request, or the secondary host if it's
 *  defined as a valid host.  The shortest route between the peripheral and the
 *  host processor is programmed.  Interrupts are not configured on the host
 *  processor.  Information received from the tisci_msg_rm_irq_set_resp
 *  message must be used by the host to complete hookup of the irq.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_IRQ_SET
 *  \n<b>Request</b>:    #tisci_msg_rm_irq_set_req
 *  \n<b>Response</b>:   #tisci_msg_rm_irq_set_resp
 *
 *  \param  req             Pointer to interrupt route set payload
 *
 *  \param  resp            Pointer to interrupt route set response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmIrqSet(const struct tisci_msg_rm_irq_set_req *req,
                           const struct tisci_msg_rm_irq_set_resp *resp,
                           uint32_t timeout);

/**
 *  \brief Releases a peripheral to processor IRQ
 *
 *  Releases a previously configured interrupt route between a peripheral and
 *  host processor.  The interrupt destination is either the processor sending
 *  the request, or the secondary host if it's defined as a valid host.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_IRQ_RELEASE
 *  \n<b>Request</b>:    #tisci_msg_rm_irq_release_req
 *
 *  \param  req             Pointer to interrupt route release payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmIrqRelease(const struct tisci_msg_rm_irq_release_req *req,
                               uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem ring
 *
 *  Configures the non-real-time registers of a Navigator Subsystem ring.
 *  The ring index must be assigned to the host defined in the TISCI header via
 *  the RM board configuration resource assignment range list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_RING_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_ring_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_ring_cfg_resp
 *
 *  \param  req             Pointer to Ring Accelerator configure payload
 *
 *  \param  resp            Pointer to Ring Accelerator configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmRingCfg(const struct tisci_msg_rm_ring_cfg_req *req,
                            const struct tisci_msg_rm_ring_cfg_resp *resp,
                            uint32_t timeout);

/**
 *  \brief Get Navigator Subsystem ring's non-real-time register configuration
 *
 *  Gets the configuration of the non-real-time register fields of a ring.  The
 *  host, or a supervisor of the host, who owns the ring must be the requesting
 *  host.  The values of the non-real-time registers are returned in
 *  #tisci_msg_rm_ring_get_cfg_resp.  The reset_cfg parameter is used to
 *  request either the existing non-real-time register values or the hardware
 *  reset values for the ring's register fields.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_RING_GET_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_ring_get_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_ring_get_cfg_resp
 *
 *  \param  req             Pointer to Ring Accelerator get config payload
 *
 *  \param  resp            Pointer to Ring Accelerator get config response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmRingGetCfg(const struct tisci_msg_rm_ring_get_cfg_req *req,
                               struct tisci_msg_rm_ring_get_cfg_resp *resp,
                               uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem ring monitor
 *
 *  Configures the non-real-time registers of a Navigator Subsystem ring
 *  monitor.
 *  The ring monitor index must be assigned to the host defined in the
 *  TISCI header via the RM board configuration resource assignment range list.
 *  Also, the ring being monitored must be assigned to the same host as
 *  the ring monitor
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_RING_MON_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_ring_mon_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_ring_mon_cfg_resp
 *
 *  \param  req             Pointer to Ring monitor configure payload
 *
 *  \param  resp            Pointer to Ring monitor configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmRingMonCfg(const struct tisci_msg_rm_ring_mon_cfg_req *req,
                               const struct tisci_msg_rm_ring_mon_cfg_resp *resp,
                               uint32_t timeout);

/**
 *  \brief Configures Navigator Subsystem UDMAP GCFG region
 *
 *  Configures a Navigator Subsystem UDMAP global configuration region.
 *  Configures the non-real-time registers of a Navigator Subsystem UDMAP
 *  global configuration region.  The register fields specified as valid for
 *  programming must be assigned to the host defined in the TISCI header via the
 *  RM board configuration resource assignment array.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_GCFG_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_gcfg_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_gcfg_cfg_resp
 *
 *  \param  req             Pointer to UDMAP GCFG configure payload
 *
 *  \param  resp            Pointer to UDMAP GCFG configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapGcfgCfg(
            const struct tisci_msg_rm_udmap_gcfg_cfg_req *req,
            const struct tisci_msg_rm_udmap_gcfg_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Get Navigator Subsystem UDMAP GCFG non-real-time
 *  register configuration
 *
 *  Get Navigator Subsystem UDMAP global configuration's non-real-time
 *  register configuration. Gets the configuration of the non-real-time register
 *  fields of a UDMAP's global configuration.  The host, or a supervisor of the
 *  host, who owns the gcfg region  must be the requesting host.  The values of
 *  the non-real-time registers are returned in
 *  \ref tisci_msg_rm_udmap_gcfg_get_cfg_resp.  The reset_cfg parameter is used
 *  to request either the existing non-real-time register values or the hardware
 *  reset values for the UDMAP's global configuration register fields.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_GCFG_GET_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_gcfg_get_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_gcfg_get_cfg_resp
 *
 *  \param  req             Pointer to UDMAP GCFG get config payload
 *
 *  \param  resp            Pointer to UDMAP GCFG get config response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapGcfgGetCfg(
            const struct tisci_msg_rm_udmap_gcfg_get_cfg_req *req,
            struct tisci_msg_rm_udmap_gcfg_get_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP transmit channel
 *
 *  Configures the non-real-time registers of a Navigator Subsystem UDMAP
 *  transmit channel.  The channel index must be assigned to the host defined
 *  in the TISCI header via the RM board configuration resource assignment
 *  range list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_TX_CH_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_tx_ch_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_tx_ch_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Tx channel configure payload
 *
 *  \param  resp            Pointer to UDMAP Tx channel configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapTxChCfg(
            const struct tisci_msg_rm_udmap_tx_ch_cfg_req *req,
            const struct tisci_msg_rm_udmap_tx_ch_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Get Navigator Subsystem UDMAP transmit channel's non-real-time
 *  register configuration
 *
 *  Gets the configuration of the non-real-time register fields of a UDMAP
 *  transmit channel.  The host, or a supervisor of the host, who owns the
 *  channel must be the requesting host.  The values of the non-real-time
 *  registers are returned in #tisci_msg_rm_udmap_tx_ch_get_cfg_resp.
 *  The reset_cfg parameter is used to request either the existing
 *  non-real-time register values or the hardware reset values for the UDMAP
 *  transmit channel's register fields.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_TX_CH_GET_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_tx_ch_get_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_tx_ch_get_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Tx channel get config payload
 *
 *  \param  resp            Pointer to UDMAP Tx channel get config response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapTxChGetCfg(
            const struct tisci_msg_rm_udmap_tx_ch_get_cfg_req *req,
            struct tisci_msg_rm_udmap_tx_ch_get_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP receive channel
 *
 *  Configures the non-real-time registers of a Navigator Subsystem UDMAP
 *  receive channel.  The channel index must be assigned to the host defined
 *  in the TISCI header via the RM board configuration resource assignment
 *  range list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_RX_CH_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_rx_ch_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_rx_ch_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx channel configure payload
 *
 *  \param  resp            Pointer to UDMAP Rx channel configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapRxChCfg(
            const struct tisci_msg_rm_udmap_rx_ch_cfg_req *req,
            const struct tisci_msg_rm_udmap_rx_ch_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Get Navigator Subsystem UDMAP receive channel's non-real-time
 *  register configuration
 *
 *  Gets the configuration of the non-real-time register fields of a UDMAP
 *  receive channel.  The host, or a supervisor of the host, who owns the
 *  channel must be the requesting host.  The values of the non-real-time
 *  registers are returned in #tisci_msg_rm_udmap_rx_ch_get_cfg_resp.
 *  The reset_cfg parameter is used to request either the existing
 *  non-real-time register values or the hardware reset values for the UDMAP
 *  receive channel's register fields.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_RX_CH_GET_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_rx_ch_get_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_rx_ch_get_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx channel get config payload
 *
 *  \param  resp            Pointer to UDMAP Rx channel get config response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapRxChGetCfg(
            const struct tisci_msg_rm_udmap_rx_ch_get_cfg_req *req,
            struct tisci_msg_rm_udmap_rx_ch_get_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP receive flow
 *
 *  Configures a Navigator Subsystem UDMAP receive flow's registers.
 *  Configuration does not include the flow registers which handle size-based
 *  free descriptor queue routing.  The
 *  #tisci_msg_rm_udmap_flow_size_thresh_cfg_req message is used to
 *  configure register fields related to size based free descriptor queues.
 *
 *  The flow index must be assigned to the host defined in the TISCI header via
 *  the RM board configuration resource assignment range list.
 *
 *  It's the user's responsibility to make sure any receive channels using the
 *  flow are disabled when changing the receive flow configuration.  Otherwise,
 *  unknown operation may occur.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_FLOW_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_flow_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_flow_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx flow configure payload
 *
 *  \param  resp            Pointer to UDMAP Rx flow configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapFlowCfg(
            const struct tisci_msg_rm_udmap_flow_cfg_req *req,
            const struct tisci_msg_rm_udmap_flow_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief UDMAP receive flow get configuration response message
 *
 *  Response received by host processor after RM has handled
 *  #tisci_msg_rm_udmap_flow_get_cfg_req.  The response contains
 *  the receive flow's register values.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_FLOW_GET_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_flow_get_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_flow_get_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx flow get config payload
 *
 *  \param  resp            Pointer to UDMAP Rx flow get config response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapFlowGetCfg(
            const struct tisci_msg_rm_udmap_flow_get_cfg_req *req,
            struct tisci_msg_rm_udmap_flow_get_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP receive flow's size threshold
 *         fields.
 *
 *  Configures a Navigator Subsystem UDMAP receive flow's size threshold fields
 *
 *  The flow index must be assigned to the host defined in the TISCI header via
 *  the RM board configuration resource assignment range list.
 *
 *  It's the user's responsibility to make sure any receive channels using the
 *  flow are disabled when changing the receive flow configuration.  Otherwise,
 *  unknown operation may occur.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_FLOW_SIZE_THRESH_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_flow_size_thresh_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_flow_size_thresh_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx flow size threshold based free
 *                          queue routing configure payload
 *
 *  \param  resp            Pointer to UDMAP Rx flow size threshold based free
 *                          queue routing configure response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapFlowSizeThreshCfg(
            const struct tisci_msg_rm_udmap_flow_size_thresh_cfg_req *req,
            const struct tisci_msg_rm_udmap_flow_size_thresh_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Get Navigator Subsystem UDMAP receive flow's non-real-time
 *  size threshold based queue routing register configuration
 *
 *  Gets the configuration of the non-real-time register fields of a UDMAP
 *  receive flow's size threshold routing registers.  The host, or a supervisor
 *  of the host, who owns the flow must be the requesting host.  The values of
 *  the non-real-time registers are returned in
 *  #tisci_msg_rm_udmap_flow_size_thresh_get_cfg_resp.
 *  The reset_cfg parameter is used to request either the existing
 *  non-real-time register values or the hardware reset values for the UDMAP
 *  flow's register fields.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_FLOW_SIZE_THRESH_GET_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_flow_size_thresh_get_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_flow_size_thresh_get_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx flow size threshold based free
 *                          queue routing get config payload
 *
 *  \param  resp            Pointer to UDMAP Rx flow size threshold based free
 *                          queue routing get config response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmUdmapFlowSizeThreshGetCfg(
            const struct tisci_msg_rm_udmap_flow_size_thresh_get_cfg_req *req,
            struct tisci_msg_rm_udmap_flow_size_thresh_get_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Pairs a PSI-L source thread and destination threads
 *
 *  Pairs a PSI-L source thread to a PSI-L destination thread.  The pairing
 *  occurs only if both threads are unpaired at the time of the pairing request.
 *  The source thread's width and credit count parameters are set to the
 *  destination thread's capabilities.  Both the source and destination threads
 *  are non-real-time enabled on successful pairing.
 *
 *  The PSI-L configuration proxy used to pair the source and destination
 *  threads is based on the Navigator Subsystem specified by #tisci_msg_rm_psil_pair_req::nav_id
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_PAIR
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_pair_req
 *
 *  \param  req             Pointer to PSI-L thread pair payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmPsilPair(const struct tisci_msg_rm_psil_pair_req *req,
                             uint32_t timeout);

/**
 *  \brief Unpairs a PSI-L source thread and destination thread
 *
 *  Unpairs a PSI-L source thread from a PSI-L destination thread.  The
 *  source thread's width and credit count parameters are cleared.  Both the
 *  source and destination threads are non-real-time disabled on successful
 *  unpairing.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_UNPAIR
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_unpair_req
 *
 *  \param  req             Pointer to PSI-L thread unpair payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmPsilUnpair(const struct tisci_msg_rm_psil_unpair_req *req,
                               uint32_t timeout);

/**
 *  \brief Reads a PSI-L thread real-time register
 *
 *  Reads the specified thread real-time configuration register from a
 *  specified PSI-L thread using the PSI-L configuration proxy.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_READ
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_read_req
 *
 *  \param  req             Pointer to PSI-L thread read payload
 *
 *  \param  resp            Pointer to PSI-L thread read response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmPsilRead(const struct tisci_msg_rm_psil_read_req *req,
                             struct tisci_msg_rm_psil_read_resp *resp,
                             uint32_t timeout);

/**
 *  \brief Writes a PSI-L thread real-time register
 *
 *  Writes the specified thread real-time configuration register to a
 *  specified PSI-L thread using the PSI-L configuration proxy.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_WRITE
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_write_req
 *
 *  \param  req             Pointer to PSI-L thread write payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref Sciclient_ServiceOperationTimeout.
 *
 *  \return CSL_PASS on success, else failure
 */
int32_t Sciclient_rmPsilWrite(const struct tisci_msg_rm_psil_write_req *req,
                              uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_RM_H_ */

/* @} */
