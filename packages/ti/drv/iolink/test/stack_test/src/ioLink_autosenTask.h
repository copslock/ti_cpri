/*
 *  Copyright (C) 2019 Texas Instruments Incorporated - http:;www.ti.com/
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
 *              file:    ioLink_autosenTask.h
 *
 *              brief:   IO-Link master stack Autosen devices control driver task
 */

#include <mst_appl.h>

#ifndef IO_LINK_IOLINK_AUTOSENTASK_H_
#define IO_LINK_IOLINK_AUTOSENTASK_H_

/* Device data object parameter indexes */
#define IOLINK_AUTOSEN_PARAMS_INDEX_VENDOR_NAME    (16U)
#define IOLINK_AUTOSEN_PARAMS_INDEX_PRODUCT_NAME   (18U)

/* Autosen device IDs */
#define IOLINK_AUTOSEN_DEVICE_NONE                 (0U)
#define IOLINK_AUTOSEN_DEVICE_AO001                (1U)
#define IOLINK_AUTOSEN_DEVICE_AD003                (2U)

/* Autosen product names */
#define IOLINK_AUTOSEN_PROD_NAME_LEN               (5U)
#define IOLINK_AUTOSEN_PROD_NAME_AO001             "AO001"
#define IOLINK_AUTOSEN_PROD_NAME_AD003             "AD003"

/* Autosen AD003 device display control parameters */
#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY           (552U)
#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_LEN       (2U)

#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_ON        (0U)
#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_OFF       (1U)
#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_SHIFT     (7U)

#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_RATE_FAST (1U)
#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_RATE_MED  (2U)
#define IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_RATE_SLOW (4U)

/*!
 *  @brief IO-Link autosen control data structure
 */
typedef struct IOLink_AutosenCtrl_s
{
    /*! device ID */
    uint32_t         devID;
    /*! port of paired device, if pairDevPort = MPL_PORTS_NUMBER, no paired device */
    uint8_t          pairDevPort;
    /*! Device vendor name                        */
    uint8_t          vendorName[IOLINK_MAX_APP_DATA_LEN];
    /*! Device product name                       */
    uint8_t          productName[IOLINK_MAX_APP_DATA_LEN];

    /*!
     * last sensor device status or actuator device control data
     * for sensor, 1: object detected, 0: object not detected
     * for actuator, 1: display off, 0: display on
     */
    uint8_t          status_ctrl;

    /*! vendor name updated flag, TRUE - updated, FALSE - not updated   */
    bool             vendorNameFlag;
    /*! product name updated flag, TRUE - updated, FALSE - not updated  */
    bool             productNameFlag;
    /*!
     * sensor status or actuator control flag
     * TRUE: status or control updated, FALSE: status or control not updated
     */
    bool             status_ctrlFlag;

} IOLink_AutosenCtrl;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
extern void IOLink_autosenTask(UArg arg0);

#endif /* IO_LINK_IOLINK_AUTOSENTASK_H_ */
