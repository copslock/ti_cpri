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
 */

/* file:  ioLink_autosenTask.c
 *
 * brief: IO-Link master stack Autosen devices control driver task
 *
 *        This driver is device and application specific, if customer wants
 *        to change the devices, this driver need to be updated or re-written
 *
 *        This application demo uses 4 Autosen sensor devices (AO001) and 4
 *        actuator devices (AD003). After the devices are waken up, the
 *        application sends ISDU read requests to each device to obtain the
 *        vendor name and product name name. The application then pairs one
 *        sensor device with one actuator device. When the sensor detect an
 *        object, the application will turn on the actuator display. When the
 *        sensor does not detect an object, the application will turn off
 *        the display
 *
 *        The device control data are saved in IOLink_autosenCtrl[port], the
 *        device control print data are saved in IOLink_printAutosenCtrl[port],
 *        the device status/control information is printed in the low priority
 *        print task.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/cfg/global.h>

#include <string.h>
#include <stdbool.h>
#include <IOLINK_log.h>
#include <ioLink_autosenTask.h>

#define AUTOSEN_OD_REQ_PRINT

/* ========================================================================== */
/*                          Global Variables                                   */
/* ========================================================================== */

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */
static IOLink_AutosenCtrl IOLink_autosenCtrl[MPL_PORTS_NUMBER] = {0, };
static IOLink_AutosenCtrl IOLink_printAutosenCtrl[MPL_PORTS_NUMBER] = {0, };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t IOLink_autosenGetDevID(uint8_t *prodName)
{
    uint32_t devID;

    if (strncmp((const char *)prodName,
                IOLINK_AUTOSEN_PROD_NAME_AO001,
                IOLINK_AUTOSEN_PROD_NAME_LEN) == 0)
    {
        devID = IOLINK_AUTOSEN_DEVICE_AO001;
    }
    else if (strncmp((const char *)prodName,
                     IOLINK_AUTOSEN_PROD_NAME_AD003,
                     IOLINK_AUTOSEN_PROD_NAME_LEN) == 0)
    {
        devID = IOLINK_AUTOSEN_DEVICE_AD003;
    }
    else
    {
        devID = IOLINK_AUTOSEN_DEVICE_NONE;
    }

    return (devID);
}

static uint8_t IOLink_autosenGetPairedDevPort(void)
{
    uint8_t port;

    for (port = 0; port < MPL_PORTS_NUMBER; port++)
    {
        if ((IOLink_autosenCtrl[port].devID == IOLINK_AUTOSEN_DEVICE_AD003) &&
            (IOLink_autosenCtrl[port].pairDevPort == MPL_PORTS_NUMBER))
        {
            break;
        }
    }

    return (port);
}

static void IOLink_autosenSetReadReq(uint8_t port, uint16_t index, uint8_t subIndex)
{
    IOLink_AppData         *pAppData;
    IOLink_AppDataFlags    *pAppFlag;
    IOLink_AppDataCounts   *pAppCnt;

    pAppData = &IOLink_appData[port];
    pAppFlag = &IOLink_appDataFlags[port];
    pAppCnt  = &IOLink_appDataCounts[port];

    memset(&pAppData->readReq, 0, sizeof(IOLink_ODreq));
    pAppData->readReq.index    = index;
    pAppData->readReq.subIndex = subIndex;

    pAppFlag->readRespFlag    = FALSE;
    pAppFlag->readRespErrFlag = FALSE;
    pAppFlag->readReqFlag     = TRUE;
    pAppCnt->readReqCnt++;
}

static void IOLink_autosenSetWriteReq(uint8_t port, uint16_t index, uint8_t subIndex, uint8_t *data, uint8_t dataLen)
{
    IOLink_AppData         *pAppData;
    IOLink_AppDataFlags    *pAppFlag;
    IOLink_AppDataCounts   *pAppCnt;
    uint8_t                 i;

    pAppData = &IOLink_appData[port];
    pAppFlag = &IOLink_appDataFlags[port];
    pAppCnt  = &IOLink_appDataCounts[port];

    memset(&pAppData->writeReq, 0, sizeof(IOLink_ODreq));
    pAppData->writeReq.index      = index;
    pAppData->writeReq.subIndex   = subIndex;
    pAppData->writeReq.dataLength = dataLen;
    for (i = 0; i < dataLen; i++)
    {
        pAppData->writeReq.data[i] = data[i];
    }

    pAppFlag->writeRespFlag    = FALSE;
    pAppFlag->writeRespErrFlag = FALSE;
    pAppFlag->writeReqFlag     = TRUE;
    pAppCnt->writeReqCnt++;
}

static void IOLink_printVendorProdName (uint32_t port)
{
    IOLink_AutosenCtrl     *pAutosen;
    IOLink_AutosenCtrl     *pPrtAsen;

    pAutosen = &IOLink_autosenCtrl[port];
    pPrtAsen = &IOLink_printAutosenCtrl[port];

    if (!pPrtAsen->productNameFlag)
    {
        /* print product name only once */
        IOLINK_log("\n%s device %s connected on port %d\n",
                   pAutosen->vendorName,
                   pAutosen->productName,
                   port);
#ifdef IO_CONSOLE
        System_flush();
#endif

        pPrtAsen->productNameFlag = TRUE;
    }
}

static void IOLink_printResetData (uint32_t port)
{
    IOLink_AutosenCtrl     *pPrtAsen;

    pPrtAsen = &IOLink_printAutosenCtrl[port];
    memset(pPrtAsen, 0, sizeof(IOLink_AutosenCtrl));
}

static void IOLink_printStatusCtrl (uint32_t port)
{
    bool                    printFlag = FALSE;
    IOLink_AutosenCtrl     *pAutosen;
    IOLink_AutosenCtrl     *pActuator;
    IOLink_AutosenCtrl     *pPrtAsen;

    pAutosen = &IOLink_autosenCtrl[port];
    pPrtAsen = &IOLink_printAutosenCtrl[port];

    /* print sensor device status or actuator device control data */
    if (pPrtAsen->status_ctrlFlag)
    {
        if (pPrtAsen->status_ctrl != pAutosen->status_ctrl)
        {
            /* print only when device status or control data changed */
            printFlag = TRUE;
        }
    }
    else
    {
        /* print sensor device status or actuator device control data first time */
        printFlag = TRUE;
    }

    if (printFlag)
    {
        if (pAutosen->devID == IOLINK_AUTOSEN_DEVICE_AO001)
        {
            /* print device status data */
            IOLINK_log("\n%s device %s on port %d ",
                       pAutosen->vendorName,
                       pAutosen->productName, port);

            if (pAutosen->status_ctrl)
            {
                IOLINK_log("detected an object");
            }
            else
            {
                IOLINK_log("did not detect an object");
            }

            if (pAutosen->pairDevPort < MPL_PORTS_NUMBER)
            {
                /* sensor and actuator paired */
                pActuator = &IOLink_autosenCtrl[pAutosen->pairDevPort];
                IOLINK_log(", paired device %s on port %d\n",
                           pActuator->productName,
                           pAutosen->pairDevPort);

                /* print actuator device control data */
                IOLINK_log("\n%s device %s on port %d ",
                           pActuator->vendorName,
                           pActuator->productName,
                           pAutosen->pairDevPort);
                if (pAutosen->status_ctrl)
                {
                    IOLINK_log("display turned on\n");
                }
                else
                {
                    IOLINK_log("display turned off\n");
                }
            }
            else
            {
                IOLINK_log("\n");
            }
        }
        else if ((pAutosen->devID == IOLINK_AUTOSEN_DEVICE_AD003) &&
                 (pAutosen->pairDevPort == MPL_PORTS_NUMBER))
        {
            /* print device status or control data */
            IOLINK_log("\n%s device %s on port %d ",
                       pAutosen->vendorName,
                       pAutosen->productName, port);

            /* Print unpaired actuator device control data */
            if (pAutosen->status_ctrl)
            {
                IOLINK_log("display turned off\n");
            }
            else
            {
                IOLINK_log("display turned on\n");
            }
        }

#ifdef IO_CONSOLE
        System_flush();
#endif
        /* save last print status/control data and flag */
        pPrtAsen->status_ctrl     = pAutosen->status_ctrl;
        pPrtAsen->status_ctrlFlag = pAutosen->status_ctrlFlag;

    }
}

void IOLink_autosenTask(UArg arg0)
{
    uint8_t                 port;
    IOLink_AppData         *pAppData;
    IOLink_AppDataFlags    *pAppFlag;
    IOLink_AutosenCtrl     *pAutosen;
    IOLink_AutosenCtrl     *pSensor;
    bool                    reqFlag;
    uint8_t                 displayCtrl;
    uint8_t                 displayUpdate;

    while(1)
    {
        /* send OD requests to read vendor/product name */
        for (port = 0; port < MPL_PORTS_NUMBER; port++)
        {
            pAppData = &IOLink_appData[port];
            pAppFlag = &IOLink_appDataFlags[port];
            pAutosen = &IOLink_autosenCtrl[port];
            reqFlag  = FALSE;

            if (pAppData->portState != CM_PORT_STATE_OPERATE)
            {
                /* reset port autosen control data if not in OPERATE state */
                memset (pAutosen, 0, sizeof(IOLink_AutosenCtrl));
                IOLink_printResetData(port);
                pAutosen->pairDevPort = MPL_PORTS_NUMBER;
                continue;
            }

            if (pAutosen->vendorNameFlag)
            {
                if (pAutosen->productNameFlag)
                {
                    /* vendor/prodcut name received */
                    pAutosen->devID = IOLink_autosenGetDevID(pAutosen->productName);
                    if (pAutosen->devID == IOLINK_AUTOSEN_DEVICE_NONE)
                    {
                        IOLink_printResetData(port);
                    }
                    else
                    {
                        IOLink_printVendorProdName (port);
                    }
                }
                else
                {
                    /* product name not saved */
                    if (pAppFlag->readRespFlag)
                    {
                        /* previous read request successful */
                        if ((pAppData->readReq.index == IOLINK_AUTOSEN_PARAMS_INDEX_PRODUCT_NAME) &&
                            (pAppData->readReq.dataLength != 0))
                        {
                            uint8_t len;

                            /* product name received */
                            len = pAppData->readReq.dataLength < IOLINK_MAX_APP_DATA_LEN ? \
                                  pAppData->readReq.dataLength:IOLINK_MAX_APP_DATA_LEN;
                            memcpy(pAutosen->productName, pAppData->readReq.data, len);
                            pAutosen->devID = IOLink_autosenGetDevID(pAutosen->productName);
                            if (pAutosen->devID != IOLINK_AUTOSEN_DEVICE_NONE)
                            {
                                pAutosen->productNameFlag = TRUE;

                                /* reset the read request/response/error flags */
                                pAppFlag->readRespFlag    = FALSE;
                                pAppFlag->readRespErrFlag = FALSE;
                                pAppFlag->readReqFlag     = FALSE;
                            }
                            else
                            {
                                /* if wrong product name received, request again */
                                reqFlag = TRUE;
                            }
                        }
                        else
                        {
                            /* wrong param index, request again */
                            reqFlag = TRUE;
                        }
                    }
                    else if (pAppFlag->readRespErrFlag)
                    {
                        /*
                         * previous read request not successful
                         * send the request again
                         */
                        reqFlag = TRUE;
                    }
                    else
                    {
                        /* No success or error response */
                        if (!pAppFlag->readReqFlag)
                        {
                            /* first time request */
                            reqFlag = TRUE;
                        }
                    }

                    if (reqFlag)
                    {
                        /* request to read the product name */
                        IOLink_autosenSetReadReq(port, IOLINK_AUTOSEN_PARAMS_INDEX_PRODUCT_NAME, 0);
                    }
                }
            }
            else
            {
                /* vendor name not saved */
                if (pAppFlag->readRespFlag)
                {
                    /* previous read request successful */
                    if ((pAppData->readReq.index == IOLINK_AUTOSEN_PARAMS_INDEX_VENDOR_NAME) &&
                        (pAppData->readReq.dataLength != 0))
                    {
                        uint8_t len;

                        len = pAppData->readReq.dataLength < IOLINK_MAX_APP_DATA_LEN ? \
                              pAppData->readReq.dataLength:IOLINK_MAX_APP_DATA_LEN;
                        memcpy(pAutosen->vendorName, pAppData->readReq.data, len);
                        pAutosen->vendorNameFlag = TRUE;
                        IOLink_autosenSetReadReq(port, IOLINK_AUTOSEN_PARAMS_INDEX_PRODUCT_NAME, 0);
                    }
                    else
                    {
                        /* wrong param index, request again */
                        reqFlag = TRUE;
                    }

                }
                else if (pAppFlag->readRespErrFlag)
                {
                    /*
                     * previous read request not successful
                     * send the request again
                     */
                    reqFlag = TRUE;
                }
                else
                {
                    /* No success or error response */
                    if (!pAppFlag->readReqFlag)
                    {
                        /* first time request */
                        reqFlag = TRUE;
                    }
                }

                if (reqFlag)
                {
                    /* request to read the vendor name */
                    IOLink_autosenSetReadReq(port, IOLINK_AUTOSEN_PARAMS_INDEX_VENDOR_NAME, 0);
                }
            }
        }

        /* pair the sensor device and actuator device */;
        for (port = 0; port < MPL_PORTS_NUMBER; port++)
        {
            pAutosen = &(IOLink_autosenCtrl[port]);
            if (pAutosen->devID == IOLINK_AUTOSEN_DEVICE_AO001)
            {
                if (pAutosen->pairDevPort == MPL_PORTS_NUMBER)
                {
                    /* sensor and actuator not paired */
                    pAutosen->pairDevPort = IOLink_autosenGetPairedDevPort();
                    if (pAutosen->pairDevPort < MPL_PORTS_NUMBER)
                    {
                        IOLink_autosenCtrl[pAutosen->pairDevPort].pairDevPort = port;
                        /* reset the status flag if a paired device found */
                        IOLink_printAutosenCtrl[port].status_ctrlFlag = FALSE;
                    }
                }
            }
        }

        /*
         * control the actuator device based on the detection of the sensor device
         * if the sensor detects an object, control the actutor to turn on the display
         * if the sensor does not detect an object, control the actuaor to turn off the display
         */
        for (port = 0; port < MPL_PORTS_NUMBER; port++)
        {
            displayUpdate = FALSE;
            pAppData = &IOLink_appData[port];
            pAutosen = &IOLink_autosenCtrl[port];
            pAppFlag = &IOLink_appDataFlags[port];
            if (pAutosen->devID == IOLINK_AUTOSEN_DEVICE_AO001)
            {
                /* sensor device, update the device status */
                if (pAppFlag->pdFlag)
                {
                    /* if PD indication received, update the device data status */
                    pAutosen->status_ctrl = pAppData->pd.inData[0];
                    pAutosen->status_ctrlFlag = TRUE;
                }
                else
                {
                    pAutosen->status_ctrlFlag = FALSE;
                }
            }

            if (pAutosen->devID == IOLINK_AUTOSEN_DEVICE_AD003)
            {
                /* AD003 acuator device */
                if (pAutosen->pairDevPort < MPL_PORTS_NUMBER)
                {
                    /* acuator device is paired with a sensor device */
                    pSensor = &IOLink_autosenCtrl[pAutosen->pairDevPort];

                    /* set the actuabor display control based on the sensor status */
                    displayCtrl = pSensor->status_ctrl ? \
                       IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_ON:IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_OFF;
                }
                else
                {
                    /* acuator device not paired with a sensor device, set display off */
                    displayCtrl = IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_OFF;
                }

                if (pAutosen->status_ctrlFlag)
                {
                    /* already set the display control to the actuator device */
                    if (pAppFlag->writeRespFlag)
                    {
                        /* previous write request successful */
                        if (pAutosen->status_ctrl != displayCtrl)
                        {
                            /*
                             * send request again only if display
                             * control data changed
                             */
                            displayUpdate = TRUE;
                        }
                    }
                    else if (pAppFlag->writeRespErrFlag)
                    {
                        /*
                         * previous write request not successful
                         * send the request again
                         */
                        displayUpdate = TRUE;
                    }
                    else
                    {
                        /*
                         * No success or error response,
                         * wait for write response or timeout??
                         */
                    }
                }
                else
                {
                    /* first time send display control to the actuator */
                    displayUpdate = TRUE;
                }

                if (displayUpdate)
                {
                    uint8_t data[IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_LEN];
                    data[0] = 0;
                    data[1] = (displayCtrl << IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_SHIFT) | \
                              IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_RATE_FAST;
                    IOLink_autosenSetWriteReq(port,
                                              IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY,
                                              0,
                                              data,
                                              IOLINK_AUTOSEN_PARAMS_IDX_DISPLAY_LEN);
                    pAutosen->status_ctrl = displayCtrl;

                    /* set the actuator device data update flag */
                    pAutosen->status_ctrlFlag = TRUE;
                }
            }

            if (pAutosen->status_ctrlFlag)
            {
                IOLink_printStatusCtrl (port);
            }
        }
        Task_sleep(200); /* in msec */
    }
}
