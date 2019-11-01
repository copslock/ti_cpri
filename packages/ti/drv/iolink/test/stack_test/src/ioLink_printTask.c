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
 *              file:    ioLink_printTask.c
 *
 *              brief:   IO-Link stack master application print task
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

#include <stdbool.h>
#include <IOLINK_log.h>
#include <mst_appl.h>
#include <ioLink_autosenTask.h>

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */
static IOLink_AppDataFlags IOLink_printDataFlags[MPL_PORTS_NUMBER]   = {0, };
static IOLink_AppData      IOLink_printData[MPL_PORTS_NUMBER]        = {0, };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void IOLink_printPortState(uint8_t port)
{
    bool                 printFlag = FALSE;
    IOLink_AppDataFlags *pAppFlags = &IOLink_appDataFlags[port];
    IOLink_AppDataFlags *pPrtFlags = &IOLink_printDataFlags[port];
    IOLink_AppData      *pApp = &IOLink_appData[port];
    IOLink_AppData      *pPrt = &IOLink_printData[port];

    if (pAppFlags->portStateFlag)
    {
        /* port state already updated */
        if (pPrtFlags->portStateFlag)
        {
            /*
             * if port state already printed to the console
             * only print again when the state changed
             */
            if (pPrt->portState != pApp->portState)
            {
                /* Port state changed */
                printFlag = TRUE;
            }
        }
        else
        {
            /* Print initial port state */
            pPrtFlags->portStateFlag = TRUE;
            printFlag = TRUE;
        }

        if (printFlag)
        {
            pPrt->portState = pApp->portState;
            IOLINK_log("\nport %d state changed to %d\n", port, pPrt->portState);
#ifdef IO_CONSOLE
            System_flush();
#endif
        }
    }
}

#ifdef IOLINK_DEBUG
static bool IOLink_checkODreq(IOLink_ODreq *reqApp, IOLink_ODreq *reqPrint)
{
    bool     match = TRUE;
    uint32_t i;
    uint8_t  dataLen;

    /* check if the last printed OD request matches with the current OD request */
    if (
        (reqApp->index == reqPrint->index)           &&
        (reqApp->subIndex == reqPrint->subIndex)     &&
        (reqApp->dataLength == reqPrint->dataLength)
       )
    {
        dataLen = (reqApp->dataLength<IOLINK_MAX_APP_DATA_LEN)?reqApp->dataLength:IOLINK_MAX_APP_DATA_LEN;
        for (i = 0; i < dataLen; i++)
        {
            if (reqApp->data[i] != reqPrint->data[i])
            {
                match = FALSE;
                break;
            }
        }
    }
    else
    {
        match = FALSE;
    }

    return (match);
}

static void IOLink_printODreq(uint8_t port, bool read)
{
    IOLink_ODreq        *pAppReq;
    IOLink_ODreq        *pPrtReq;
    uint8_t             *pPrtFlag;
    bool                 printFlag = FALSE;
    uint8_t              dataLen;
    uint8_t              i;

    if (read)
    {
        pPrtFlag = &(IOLink_printDataFlags[port].readRespFlag);
        pAppReq  = &(IOLink_appData[port].lastReadReq);
        pPrtReq  = &(IOLink_printData[port].lastReadReq);
    }
    else
    {
        pPrtFlag = &(IOLink_printDataFlags[port].writeRespFlag);
        pAppReq  = &(IOLink_appData[port].lastWriteReq);
        pPrtReq  = &(IOLink_printData[port].lastWriteReq);
    }

    if (pAppReq->dataLength != 0)
    {
        /* OD request data available */
        if (*pPrtFlag)
        {
            /*
             * OD request data already printed to the console
             * only print again if request data changed
             */
            if (!IOLink_checkODreq(pAppReq, pPrtReq))
            {
                /* OD request data changed */
                printFlag = TRUE;
            }
        }
        else
        {
            /* print the OD request data first time */
            printFlag = TRUE;
        }

        if (printFlag)
        {
            *pPrtReq = *pAppReq;
            IOLINK_log("\nOD ");
            if (read)
            {
                IOLINK_log("read request on ");
            }
            else
            {
                IOLINK_log("write request on ");
            }
            IOLINK_log("port %d: index %d, subIndex %d, dataLen %d\n",
                       port, pPrtReq->index, pPrtReq->subIndex, pPrtReq->dataLength);
            dataLen = (pPrtReq->dataLength<IOLINK_MAX_APP_DATA_LEN)?pPrtReq->dataLength:IOLINK_MAX_APP_DATA_LEN;
            IOLINK_log("data: ");
            for (i = 0; i < dataLen; i++)
            {
                IOLINK_log("0x%x ", pPrtReq->data[i]);
            }
            IOLINK_log("\n");
#ifdef IO_CONSOLE
            System_flush();
#endif
            *pPrtFlag = TRUE;
        }
    }
}

static void IOLink_printEvent(uint8_t port)
{
    bool                 printFlag = FALSE;
    IOLink_AppDataFlags *pAppFlags = &IOLink_appDataFlags[port];
    IOLink_AppDataFlags *pPrtFlags = &IOLink_printDataFlags[port];
    IOLink_Event        *pApp = &(IOLink_appData[port].event);
    IOLink_Event        *pPrt = &(IOLink_printData[port].event);

    if (pAppFlags->eventFlag)
    {
        /* event updated */
        if (pPrtFlags->eventFlag)
        {
            /*
             * event already printed to the console
             * only print again if event changed
             */
            if (
                (pPrt->instance != pApp->instance) ||
                (pPrt->type != pApp->type)         ||
                (pPrt->mode != pApp->mode)         ||
                (pPrt->source != pApp->source)     ||
                (pPrt->code != pApp->code)
               )
            {
                /* Event changed */
                printFlag = TRUE;
            }
        }
        else
        {
            /* print event to the console first time */
            pPrtFlags->eventFlag = TRUE;
            printFlag = TRUE;
        }

        if (printFlag)
        {
            *pPrt = *pApp;
            IOLINK_log("\nevent received on port %d: instance 0x%x, type 0x%x, mode 0x%x, source 0x%x, code 0x%x\n",
                       port, pPrt->instance, pPrt->type, pPrt->mode, pPrt->source, pPrt->code);
#ifdef IO_CONSOLE
            System_flush();
#endif
        }
    }
    else
    {
        /* event not updated */
        pPrtFlags->eventFlag = FALSE;
    }
}

static bool IOLink_checkPDind(IOLink_PD *pApp, IOLink_PD *pPrt)
{
    bool     match = TRUE;
    uint32_t i;
    uint8_t  dataLen;

    /* check if last printed PD data matches with the current PD data */
    if (
        (pApp->invalidFlag == pPrt->invalidFlag) &&
        (pApp->inLength == pPrt->inLength)       &&
        (pApp->outLength == pPrt->outLength)
       )
    {
        dataLen = (pApp->inLength<IOLINK_MAX_APP_DATA_LEN)?pApp->inLength:IOLINK_MAX_APP_DATA_LEN;
        for (i = 0; i < dataLen; i++)
        {
            if (pApp->inData[i] != pPrt->inData[i])
            {
                match = FALSE;
                break;
            }
        }
        if (match)
        {
            dataLen = (pApp->outLength<IOLINK_MAX_APP_DATA_LEN)?pApp->outLength:IOLINK_MAX_APP_DATA_LEN;
            for (i = 0; i < dataLen; i++)
            {
                if (pApp->outData[i] != pPrt->outData[i])
                {
                    match = FALSE;
                    break;
                }
            }
        }
    }
    else
    {
        match = FALSE;
    }

    return (match);
}

static void IOLink_printPDind(uint8_t port)
{
    bool                 printFlag = FALSE;
    IOLink_PD           *pAppPD;
    IOLink_PD           *pPrtPD;
    IOLink_AppDataFlags *pAppFlag;
    IOLink_AppDataFlags *pPrtFlag;
    uint8_t              dataLen;
    uint8_t              i;

    pAppFlag = &IOLink_appDataFlags[port];
    pPrtFlag = &IOLink_printDataFlags[port];
    pAppPD  = &(IOLink_appData[port].pd);
    pPrtPD  = &(IOLink_printData[port].pd);

    if (pAppFlag->pdFlag)
    {
        /*
         * PD data updated
         */
        if (pPrtFlag->pdFlag)
        {
            /*
             * PD data already printed to the console
             * only print again if PD data changed
             */
            if (!IOLink_checkPDind(pAppPD, pPrtPD))
            {
                /* PD indication changed */
                printFlag = TRUE;
            }
        }
        else
        {
            /* print PD indication first time */
            pPrtFlag->pdFlag = TRUE;
            printFlag = TRUE;
        }

        if (printFlag)
        {
            *pPrtPD = *pAppPD;
            IOLINK_log("\nPD indication received on port %d: invalid flag 0x%x, in len %d, out len %d\n",
                       port, pPrtPD->invalidFlag, pPrtPD->inLength, pPrtPD->outLength);
            dataLen = (pPrtPD->inLength<IOLINK_MAX_APP_DATA_LEN)?pPrtPD->inLength:IOLINK_MAX_APP_DATA_LEN;
            if (dataLen)
            {
                IOLINK_log("PD in data: ");
                for (i = 0; i < dataLen; i++)
                {
                    IOLINK_log("0x%x ", pPrtPD->inData[i]);
                }
            }
            dataLen = (pPrtPD->outLength<IOLINK_MAX_APP_DATA_LEN)?pPrtPD->outLength:IOLINK_MAX_APP_DATA_LEN;
            if (dataLen)
            {
                IOLINK_log("\nPD out data: ");
                for (i = 0; i < dataLen; i++)
                {
                    IOLINK_log("0x%x ", pPrtPD->outData[i]);
                }
            }
            IOLINK_log("\n");
#ifdef IO_CONSOLE
            System_flush();
#endif
        }
    }
    else
    {
        /* PD indication not updated */
        pPrtFlag->pdFlag = FALSE;
    }
}
#endif

static void IOLink_printODrespErr(uint8_t port, bool read)
{
    IOLink_ODrespErr    *pAppRespErr;
    IOLink_ODrespErr    *pPrtRespErr;
    IOLink_ODreq        *pAppReq;
    uint8_t             *pPrtFlag;
    bool                 printFlag = FALSE;

    if (read)
    {
        pPrtFlag    = &(IOLink_printDataFlags[port].readRespErrFlag);
        pAppRespErr = &(IOLink_appData[port].readRespErr);
        pPrtRespErr = &(IOLink_printData[port].readRespErr);
        pAppReq     = &(IOLink_appData[port].readReq);
    }
    else
    {
        pPrtFlag    = &(IOLink_printDataFlags[port].writeRespErrFlag);
        pAppRespErr = &(IOLink_appData[port].writeRespErr);
        pPrtRespErr = &(IOLink_printData[port].writeRespErr);
        pAppReq     = &(IOLink_appData[port].writeReq);
    }

    if (pAppRespErr->errorCode != 0)
    {
        /* OD response error data available */
        if (*pPrtFlag)
        {
            /*
             * OD response datat already printed to the console
             * only print again if response data changed
             */
            if (
                (pAppRespErr->errorCode != pPrtRespErr->errorCode)       ||
                (pAppRespErr->addErrorCode != pPrtRespErr->addErrorCode)
               )
            {
                /* OD request data changed */
                printFlag = TRUE;
            }
        }
        else
        {
            /* print OD response data first time */
            printFlag = TRUE;
        }

        if (printFlag)
        {
            *pPrtRespErr = *pAppRespErr;
            IOLINK_log("\nOD ");
            if (read)
            {
                IOLINK_log("read response error on ");
            }
            else
            {
                IOLINK_log("write response error on ");
            }
            IOLINK_log("port %d: err code 0x%x, additional err code 0x%x, index %d, sub index %d\n",
                       port,
                       pPrtRespErr->errorCode,
                       pPrtRespErr->addErrorCode,
                       pAppReq->index,
                       pAppReq->subIndex);
#ifdef IO_CONSOLE
            System_flush();
#endif
            *pPrtFlag = TRUE;
        }
    }
}

void IOLink_printTask(UArg arg0)
{
    uint8_t port;

    while(1)
    {
        for (port = 0; port < MPL_PORTS_NUMBER; port++)
        {
            /* Print port state */
            IOLink_printPortState(port);

#ifdef IOLINK_DEBUG
            /* Print read request data on a port */
            IOLink_printODreq(port, true);

            /* Print write request data on a port */
            IOLink_printODreq(port, false);

            /* Print event received on a port */
            IOLink_printEvent(port);

            /* Print PD indication received on a port */
            IOLink_printPDind(port);
#endif
            /* Print read response error on a port */
            IOLink_printODrespErr(port, true);

            /* Print write request response error on a port */
            IOLink_printODrespErr(port, false);
        }

        Task_sleep(200); /* in msec */
    }
}
