/********************************************************************
* Copyright (C) 2014 Texas Instruments Incorporated.
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
#ifndef _QMSS_SRIORTR_REGS_H_
#define _QMSS_SRIORTR_REGS_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <stdint.h>
#include <stdlib.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure enables
\**************************************************************************/
typedef struct {
    volatile uint32_t portEnabled;    /* status is seen here - 1 LSB per port*/
#define CSL_QMSS_SRIORTR_ENABLES_PORTENABLED_STAT_MASK                 (0x0000000f)
#define CSL_QMSS_SRIORTR_ENABLES_PORTENABLED_STAT_SHIFT                (0x00000000)
#define CSL_QMSS_SRIORTR_ENABLES_PORTENABLED_STAT_RESETVAL             (0x00000000)

    volatile uint32_t portEnables;    /* host writes here - 1 LSB per port */
#define CSL_QMSS_SRIORTR_ENABLES_PORTENABLES_SET_MASK                  (0x0000000f)
#define CSL_QMSS_SRIORTR_ENABLES_PORTENABLES_SET_SHIFT                 (0x00000000)
#define CSL_QMSS_SRIORTR_ENABLES_PORTENABLES_SET_RESETVAL              (0x00000000)
} qmss_srioRtr_Enables;

/**************************************************************************\
* Register Overlay Structure for queue map
\**************************************************************************/
typedef struct {
    /* queue base must be such that the 16 queues do not span a 32
     * queue boundary */
    volatile uint32_t SRIORTR_INPUT_QUEUEBASE;
#define CSL_QMSS_SRIORTR_QUEUECFG_INPUT_QUEUEBASE_QUEUE_MASK          (0x0000ffff)
#define CSL_QMSS_SRIORTR_QUEUECFG_INPUT_QUEUEBASE_QUEUE_SHIFT         (0x00000000)
#define CSL_QMSS_SRIORTR_QUEUECFG_INPUT_QUEUEBASE_QUEUE_RESETVAL      (0x00000000)

    /* credits assoiated with each credit grant message */
    volatile uint32_t CREDITS;
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDITS_MASK                        (0x0000ffff)
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDITS_SHIFT                       (0x00000000)
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDITS_RESETVAL                    (0x00000000)

    /* One queue per port to receive new packets from host */
#define QMSS_SRIORTR_INPUT_QUEUEOFFSET_HOST_QUEUE(port)     ( 0 + (port))
    /* One queue per prt to receive routing packets from SRIO */
#define QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_QUEUE(port)      ( 4 + (port))
    /* One queue per port to receive return packets from SRIO for forwarding */
#define QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_RET_QUEUE(port)  ( 8 + (port))
    /* One queue per port to receive credit grant request messages */
#define QMSS_SRIORTR_INPUT_QUEUEOFFSET_CREDIT_QUEUE(port)   (12 + (port))

    /* TX queue for each input port (5th is host, 6th is credit tx) and each output port (0..3) */
    volatile uint32_t SRIORTR_OUTPUT_QUEUES[6][4];
#define CSL_QMSS_SRIORTR_QUEUECFG_OUTPUT_QUEUES_QUEUE_MASK            (0x0000ffff)
#define CSL_QMSS_SRIORTR_QUEUECFG_OUTPUT_QUEUES_QUEUE_SHIFT           (0x00000000)
#define CSL_QMSS_SRIORTR_QUEUECFG_OUTPUT_QUEUES_QUEUE_RESETVAL        (0x00000000)

    /* Return queue for received credit messages - per port */
    volatile uint32_t SRIORTR_CREDIT_RETURN_QUEUES[4];
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDIT_RETURN_QUEUES_QUEUE_MASK     (0x0000ffff)
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDIT_RETURN_QUEUES_QUEUE_SHIFT    (0x00000000)
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDIT_RETURN_QUEUES_QUEUE_RESETVAL (0x00000000)

    /* Source queue for transmit credit messages - per port */
    volatile uint32_t SRIORTR_CREDIT_SOURCE_QUEUES[4];
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDIT_SOURCE_QUEUES_QUEUE_MASK     (0x0000ffff)
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDIT_SOURCE_QUEUES_QUEUE_SHIFT    (0x00000000)
#define CSL_QMSS_SRIORTR_QUEUECFG_CREDIT_SOURCE_QUEUES_QUEUE_RESETVAL (0x00000000)

    /* fowarding rx free pool (to return transmitted packets) */
    volatile uint32_t SRIORTR_FWD_RX_FREE_QUEUES[4];
#define CSL_QMSS_SRIORTR_QUEUECFG_FWD_RX_FREE_QUEUES_QUEUE_MASK       (0x0000ffff)
#define CSL_QMSS_SRIORTR_QUEUECFG_FWD_RX_FREE_QUEUES_QUEUE_SHIFT      (0x00000000)
#define CSL_QMSS_SRIORTR_QUEUECFG_FWD_RX_FREE_QUEUES_QUEUE_RESETVAL   (0x00000000)
} qmss_srioRtr_queueCfg;

/**************************************************************************\
* Register Overlay Structure for stats
\**************************************************************************/
typedef struct {
    volatile uint32_t PACKETS_RX[4];   /* One counter per port */
#define CSL_QMSS_SRIORTR_STATS_PACKETS_RX_MASK                        (0xffffffff)
#define CSL_QMSS_SRIORTR_STATS_PACKETS_RX_SHIFT                       (0x00000000)
#define CSL_QMSS_SRIORTR_STATS_PACKETS_RX_RESETVAL                    (0x00000000)

    volatile uint32_t PACKETS_TX[4];  
#define CSL_QMSS_SRIORTR_STATS_PACKETS_TX_MASK                        (0xffffffff)
#define CSL_QMSS_SRIORTR_STATS_PACKETS_TX_SHIFT                       (0x00000000)
#define CSL_QMSS_SRIORTR_STATS_PACKETS_TX_RESETVAL                    (0x00000000)

    volatile uint32_t CREDITS_TX[4];
#define CSL_QMSS_SRIORTR_STATS_CREDITS_TX_MASK                        (0xffffffff)
#define CSL_QMSS_SRIORTR_STATS_CREDITS_TX_SHIFT                       (0x00000000)
#define CSL_QMSS_SRIORTR_STATS_CREDITS_TX_RESETVAL                    (0x00000000)

    volatile uint32_t CREDITS_RX[4];
#define CSL_QMSS_SRIORTR_STATS_CREDITS_RX_MASK                        (0xffffffff)
#define CSL_QMSS_SRIORTR_STATS_CREDITS_RX_SHIFT                       (0x00000000)
#define CSL_QMSS_SRIORTR_STATS_CREDITS_RX_RESETVAL                    (0x00000000)

    volatile uint32_t HOST_RX[4];
#define CSL_QMSS_SRIORTR_STATS_HOST_RX_MASK                           (0xffffffff)
#define CSL_QMSS_SRIORTR_STATS_HOST_RX_SHIFT                          (0x00000000)
#define CSL_QMSS_SRIORTR_STATS_HOST_RX_RESETVAL                       (0x00000000)

    volatile uint32_t FWD_RETS[4];
#define CSL_QMSS_SRIORTR_STATS_FWD_RETS_MASK                          (0xffffffff)
#define CSL_QMSS_SRIORTR_STATS_FWD_RETS_SHIFT                         (0x00000000)
#define CSL_QMSS_SRIORTR_STATS_FWD_RETS_RESETVAL                      (0x00000000)
} qmss_srioRtr_stats;

/**************************************************************************\
* Register Overlay Structure for routing table entries
\**************************************************************************/
typedef struct {
#define CSL_QMSS_SRIORTR_ROUTE_TBL_8_DESTS_MASK                      (0xffffffff)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_8_DESTS_SHIFT                     (0x00000000)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_8_DESTS_RESETVAL                  (0x00000000)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_1_DEST_PORT_MASK                  (0x00000003)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_1_DEST_PORT_SHIFT                 (0x00000000)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_1_DEST_PORT_RESETVAL              (0x00000000)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_1_DEST_FINDEST_MASK               (0x00000004)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_1_DEST_FINDEST_SHIFT              (0x00000002)
#define CSL_QMSS_SRIORTR_ROUTE_TBL_1_DEST_FINDEST_RESETVAL           (0x00000000)
    volatile uint32_t ROUTE_TBL[32];
} qmss_srioRtr_routetbl;

/**************************************************************************\
* Register Overlay Structure for routing table entries
\**************************************************************************/
typedef struct {
#define CSL_QMSS_SRIORTR_DEST_ID_DEST_MASK                            (0xffffffff)
#define CSL_QMSS_SRIORTR_DEST_ID_DEST_SHIFT                           (0x00000000)
#define CSL_QMSS_SRIORTR_DEST_ID_DEST_RESETVAL                        (0x00000000)
    volatile uint32_t DEST_ID[4];
} qmss_srioRtr_destId;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    uint8_t  RSVD0[4];         /* Download protocol expects this to 0 */
    volatile uint32_t                   MAGIC;     /* offset = 0x004 */
#define CSL_QMSS_SRIORTR_MAGIC_ENDIAN_MASK                            (0x00000001u)
#define CSL_QMSS_SRIORTR_MAGIC_ENDIAN_SHIFT                           (0x00000000u)
#define CSL_QMSS_SRIORTR_MAGIC_ENDIAN_RESETVAL                        (0x00000000u)

#define QMSS_SRIORTR_MAGIC_ENDIAN_BIG                                 1
#define QMSS_SRIORTR_MAGIC_ENDIAN_LITTLE                              0

#define CSL_QMSS_SRIORTR_MAGIC_MAGIC_MASK                             (0xFFFF0000u)
#define CSL_QMSS_SRIORTR_MAGIC_MAGIC_SHIFT                            (0x00000010u)
#define CSL_QMSS_SRIORTR_MAGIC_MAGIC_RESETVAL                         (0x00000000u)

#define CSL_QMSS_SRIORTR_MAGIC_MASK                                   (0xFFFFFFFFu)
#define CSL_QMSS_SRIORTR_MAGIC_SHIFT                                  (0x00000000u)
#define CSL_QMSS_SRIORTR_MAGIC_RESETVAL                               (0x00000000u)

#define QMSS_SRIORTR_MAGIC                                            0x8100

    volatile uint32_t                   VERSION;   /* offset = 0x008 */
#define CSL_QMSS_SRIORTR_VERSION_MASK                                 (0xFFFFFFFFu)
#define CSL_QMSS_SRIORTR_VERSION_SHIFT                                (0x00000000u)
#define CSL_QMSS_SRIORTR_VERSION_RESETVAL                             (0x00000000u)

    uint8_t RSVD1[12];
    volatile uint32_t                   PDSPNUM;   /* offset = 0x018 */
    qmss_srioRtr_Enables                ENABLES;   /* offset = 0x01c, size = 0x08 */
    uint8_t RSVD2[8];
    qmss_srioRtr_queueCfg               QUEUE_CFG; /* offset = 0x02c, size = 0x98 */
    uint8_t RSVD3[12];
    qmss_srioRtr_stats                  STATS;     /* offset = 0x0D0, size = 0x60 */
    qmss_srioRtr_routetbl               ROUTETBL;  /* offset = 0x130, size = 0x80 */
    qmss_srioRtr_destId                 DESTID;    /* offset = 0x1B0, size = 0x10 */
} Qmss_SrioRtrRegs;


/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* COMMAND_BUFFER_WORD0 */

#endif
