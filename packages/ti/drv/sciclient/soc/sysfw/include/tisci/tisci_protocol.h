/*
 *  Copyright (C) 2017-2019 Texas Instruments Incorporated
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
 * \ingroup TISCI
 * \defgroup tisci_protocol tisci_protocol
 *
 * DMSC controls the power management, security and resource management
 * of the device.
 *
 *
 * @{
 */
/**
 *
 *  \brief  This file contains:
 *
 *          WARNING!!: Autogenerated file from SYSFW. DO NOT MODIFY!!
 * System Firmware TISCI Messaging Core
 *
 * TISCI Protocol Definitions
 *
 */

#ifndef TISCI_PROTOCOL_H
#define TISCI_PROTOCOL_H


/**
 * This flag is reserved and not to be used.
 */
#define TISCI_MSG_FLAG_RESERVED0    TISCI_BIT(0)

/**
 * ACK on Processed: Send a response to a message after it has been processed
 * with TISCI_MSG_FLAG_ACK set if the processing succeeded, or a NAK otherwise.
 * This response contains the complete response to the message with the result
 * of the actual action that was requested.
 */
#define TISCI_MSG_FLAG_AOP    TISCI_BIT(1)

/** Indicate that this message is marked secure */
#define TISCI_MSG_FLAG_SEC    TISCI_BIT(2)

/**
 * Response flag for a message that indicates success. If this flag is NOT
 * set then that is to be interpreted as a NAK.
 */
#define TISCI_MSG_FLAG_ACK    TISCI_BIT(1)

/**
 * \brief Header that prefixes all TISCI messages.
 *
 * \param type Type of message identified by a TISCI_MSG_* ID
 * \param host Host of the message.
 * \param seq Message identifier indicating a transfer sequence.
 * \param flags TISCI_MSG_FLAG_* for the message.
 * \param payload Placeholder pointer that can be used to access the raw message body.
 */
struct tisci_header {
    uint16_t    type;
    uint8_t    host;
    uint8_t    seq;
    uint32_t    flags;
    uint8_t    payload[];
};

/*
 * Messages sent via secure transport need an additional field for integrity
 * check in addition to the fields of a TISCI message. The below data structure
 * is used to enhance the standard TISCI message header with a field for
 * integrity check and a reserved field for future use.
 *
 * The below defined header is placed before the \ref tisci_header when
 * transmitting a message via a secure transport.
 *
 * For more information, please refer to the TISCI documentation.
 */

/**
 * \brief Header that prefixes all TISCI messages sent via secure transport.
 *
 * \param integ_check This field should always be initialized to zero on GP
 * devices. The mechanism to calculate the integrity check for HS devices
 * is under definition.
 *
 * \param rsvd Reserved field for future use. Initialize to zero.
 *
 */
struct tisci_sec_header {
    uint16_t    integ_check;
    uint16_t    rsvd;
};

/* TISCI Message IDs */
#define TISCI_MSG_VERSION                       (0x0002U)
#define TISCI_MSG_BOOT_NOTIFICATION             (0x000AU)
#define TISCI_MSG_BOARD_CONFIG                  (0x000BU)
#define TISCI_MSG_BOARD_CONFIG_RM               (0x000CU)
#define TISCI_MSG_BOARD_CONFIG_SECURITY         (0x000DU)
#define TISCI_MSG_BOARD_CONFIG_PM               (0x000EU)

#define TISCI_MSG_ENABLE_WDT                    (0x0000U)
#define TISCI_MSG_WAKE_RESET                    (0x0001U)
#define TISCI_MSG_WAKE_REASON                   (0x0003U)
#define TISCI_MSG_GOODBYE                       (0x0004U)
#define TISCI_MSG_SYS_RESET                     (0x0005U)

#define TISCI_MSG_QUERY_MSMC                    (0x0020U)

#define TISCI_MSG_SET_CLOCK                     (0x0100U)
#define TISCI_MSG_GET_CLOCK                     (0x0101U)
#define TISCI_MSG_SET_CLOCK_PARENT              (0x0102U)
#define TISCI_MSG_GET_CLOCK_PARENT              (0x0103U)
#define TISCI_MSG_GET_NUM_CLOCK_PARENTS         (0x0104U)
#define TISCI_MSG_SET_FREQ                      (0x010cU)
#define TISCI_MSG_QUERY_FREQ                    (0x010dU)
#define TISCI_MSG_GET_FREQ                      (0x010eU)

#define TISCI_MSG_SET_DEVICE                    (0x0200U)
#define TISCI_MSG_GET_DEVICE                    (0x0201U)
#define TISCI_MSG_SET_DEVICE_RESETS             (0x0202U)
#define TISCI_MSG_DEVICE_DROP_POWERUP_REF       (0x0203U)

/** Message to set a firewall region configuration */
#define TISCI_MSG_SET_FWL_REGION                (0x9000U)
/** Message to get a firewall region configuration */
#define TISCI_MSG_GET_FWL_REGION                (0x9001U)
/** Message to change firewall region owner */
#define TISCI_MSG_CHANGE_FWL_OWNER              (0x9002U)
/** Message to derive a KEK and set SA2UL DKEK register */
#define TISCI_MSG_SA2UL_SET_DKEK                (0x9003U)
/** Message to erase the DKEK register */
#define TISCI_MSG_SA2UL_RELEASE_DKEK            (0x9004U)
/** Message to import a symmetric key to the keystore */
#define TISCI_MSG_KEYSTORE_IMPORT_SKEY          (0x9005U)
/** Message to erase a imported symmetric key in the keystore */
#define TISCI_MSG_KEYSTORE_ERASE_SKEY           (0x9006U)
/** Message to import a asymmetric key to the keystore */
#define TISCI_MSG_KEYSTORE_IMPORT_ASKEY         (0x9007U)
/** Message to erase a imported asymmetric key in the keystore */
#define TISCI_MSG_KEYSTORE_ERASE_ASKEY          (0x9008U)
/** Message to set a ISC region configuration */
#define TISCI_MSG_SET_ISC_REGION                (0x9009U)
/** Message to get a ISC region configuration */
#define TISCI_MSG_GET_ISC_REGION                (0x900AU)
/** Notification for firewall exception */
#define TISCI_MSG_FWL_EXCP_NOTIFICATION         (0x900BU)
/** Message to open debug firewalls using a certificate */
#define TISCI_MSG_OPEN_DEBUG_FWLS               (0x900CU)
/**
 * Message to write partitioning data and provisioned keys to the keystore
 * memory
 */
#define TISCI_MSG_KEYSTORE_WRITE                (0x900DU)
/**
 * Message to encrypt and export the full keystore contents for saving to
 * external storage
 */
#define TISCI_MSG_KEYSTORE_EXPORT_ALL           (0x900EU)
/** Message to import and decrypt a previously exported keystore blob */
#define TISCI_MSG_KEYSTORE_IMPORT_ALL           (0x900FU)
/** Message to perform the RSA signature primitive */
#define TISCI_MSG_SA2UL_PKA_RSA_SIGN_PRIM       (0x9010U)
/** Message to perform the RSA verification primitive */
#define TISCI_MSG_SA2UL_PKA_RSA_VERIFY_PRIM     (0x9011U)
/** Message to perform the RSA encryption primitive */
#define TISCI_MSG_SA2UL_PKA_RSA_ENCRYPT_PRIM    (0x9012U)
/** Message to perform the RSA decryption primitive */
#define TISCI_MSG_SA2UL_PKA_RSA_DECRYPT_PRIM    (0x9013U)
/** Message to sign a message hash with an EC private key */
#define TISCI_MSG_SA2UL_PKA_ECDSA_SIGN          (0x9014U)
/** Message to verify an EC signature */
#define TISCI_MSG_SA2UL_PKA_ECDSA_VERIFY        (0x9015U)

/** Message to generate a symmetric key via RNG and save it in the runtime keystore */
#define TISCI_MSG_KEYSTORE_GEN_SKEY_FROM_RNG    (0x9016U)


/** Message to get random numbers */
#define TISCI_MSG_GET_RANDOM                    (0x9020U)

/* Processor Control APIs */

/** Message to get a Processor */
#define TISCI_MSG_PROC_REQUEST          (0xC000U)
/** Message to release a Processor */
#define TISCI_MSG_PROC_RELEASE          (0xC001U)
/** Message to handover a Processor */
#define TISCI_MSG_PROC_HANDOVER         (0xC005U)

/** Message to Set the processor configuration */
#define TISCI_MSG_PROC_SET_CONFIG       (0xC100U)
/** Message to Set the processor control */
#define TISCI_MSG_PROC_SET_CONTROL      (0xC101U)

/** Message to Get the processor status */
#define TISCI_MSG_PROC_GET_STATUS       (0xC400U)

/** Message to Wait for processor status */
#define TISCI_MSG_PROC_WAIT_STATUS      (0xC401U)

/** Message to do authenticated boot configuration of a processor */
#define TISCI_MSG_PROC_AUTH_BOOT        (0xC120U)

/* RM IDs */

/**
 * RM TISCI message to request a resource range assignment for a host
 */
#define TISCI_MSG_RM_GET_RESOURCE_RANGE         (0x1500U)
/**
 * RM TISCI message to set an IRQ between a peripheral and host processor
 */
#define TISCI_MSG_RM_IRQ_SET                    (0x1000U)
/**
 * RM TISCI message to release a configured IRQ
 */
#define TISCI_MSG_RM_IRQ_RELEASE                (0x1001U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1100              (0x1100U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1101              (0x1101U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1102              (0x1102U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1103              (0x1103U)
/**
 * RM TISCI message to configure a Navigator Subsystem ring
 */
#define TISCI_MSG_RM_RING_CFG                   (0x1110U)
/**
 * RM TISCI message to get the register cfg for a Navigator Subsystem ring
 */
#define TISCI_MSG_RM_RING_GET_CFG               (0x1111U)
/**
 * RM TISCI message to configure a Navigator Subsystem ring monitor
 */
#define TISCI_MSG_RM_RING_MON_CFG               (0x1120U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1200              (0x1200U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1201              (0x1201U)
/**
 * RM TISCI message to configure a Navigator Subsystem UDMAP transmit channel
 */
#define TISCI_MSG_RM_UDMAP_TX_CH_CFG            (0x1205U)
/**
 * RM TISCI message to get the register cfg for a Navigator Subsystem UDMAP
 * transmit channel
 */
#define TISCI_MSG_RM_UDMAP_TX_CH_GET_CFG        (0x1206U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1210              (0x1210U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1211              (0x1211U)
/**
 * RM TISCI message to configure a Navigator Subsystem UDMAP receive channel
 */
#define TISCI_MSG_RM_UDMAP_RX_CH_CFG            (0x1215U)
/**
 * RM TISCI message to get the register cfg for a Navigator Subsystem UDMAP
 * receive channel
 */
#define TISCI_MSG_RM_UDMAP_RX_CH_GET_CFG        (0x1216U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1220              (0x1220U)
/* Reserved ID not for (re)use */
#define TISCI_MSG_RM_RESERVED_1221              (0x1221U)
/**
 * RM TISCI message to configure a Navigator Subsystem UDMAP receive flow
 */
#define TISCI_MSG_RM_UDMAP_FLOW_CFG             (0x1230U)
/**
 * RM TISCI message to configure a Navigator Subsystem UDMAP receive flow's
 * optional, size based free descriptor queue registers
 */
#define TISCI_MSG_RM_UDMAP_FLOW_SIZE_THRESH_CFG (0x1231U)
/**
 * RM TISCI message to get the register cfg for a Navigator Subsystem UDMAP
 * receive flow's standard registers
 */
#define TISCI_MSG_RM_UDMAP_FLOW_GET_CFG         (0x1232U)
/**
 * RM TISCI message to get the register cfg for a Navigator Subsystem UDMAP
 * receive flow's size based free descriptor queue registers
 */
#define TISCI_MSG_RM_UDMAP_FLOW_SIZE_THRESH_GET_CFG (0x1233U)
/**
 * RM TISCI message to configure a Navigator Subsystem UDMAP global
 * configuration
 */
#define TISCI_MSG_RM_UDMAP_GCFG_CFG             (0x1240U)
/**
 * RM TISCI message to get the register cfg for a Navigator Subsystem UDMAP
 * global configuration
 */
#define TISCI_MSG_RM_UDMAP_GCFG_GET_CFG         (0x1241U)
/**
 * RM TISCI message to pair PSI-L source and destination threads
 */
#define TISCI_MSG_RM_PSIL_PAIR                  (0x1280U)
/**
 * RM TISCI message to unpair PSI-L source and destination threads
 */
#define TISCI_MSG_RM_PSIL_UNPAIR                (0x1281U)
/**
 * RM TISCI message to read PSI-L thread RT registers via the PSI-L proxy
 */
#define TISCI_MSG_RM_PSIL_READ                  (0x1282U)
/**
 * RM TISCI message to write PSI-L thread RT registers via the PSI-L proxy
 */
#define TISCI_MSG_RM_PSIL_WRITE                 (0x1283U)
/**
 * RM TISCI message to setup a Navigator Subsystem proxy for
 * configuration
 */
#define TISCI_MSG_RM_PROXY_CFG                  (0x1300U)

#endif /* TISCI_PROTOCOL_H */

/* @} */