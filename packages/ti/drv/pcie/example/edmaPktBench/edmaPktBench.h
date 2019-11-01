/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef PCIEEDMAPKTBENCH_H_
#define PCIEEDMAPKTBENCH_H_

#include <ti/osal/SemaphoreP.h>

#define EDMAPKT_NUM_TX_PKTS    (32)
#define EDMAPKT_NUM_TX_PKTS_PER_LINKAGE (1)
#define EDMAPKT_NUM_TX_PARAMS_PER_PKT (3)
#define EDMAPKT_NUM_TX_PARAMS_LINKAGE (1)
#define EDMAPKT_NUM_TX_PARAMS_TOT_LINKAGE ((EDMAPKT_NUM_TX_PKTS + EDMAPKT_NUM_TX_PARAMS_LINKAGE - 1) / \
                                           (EDMAPKT_NUM_TX_PKTS_PER_LINKAGE))
#define EDMAPKT_NUM_TX_PARAMS_TOT_PKTS (EDMAPKT_NUM_TX_PKTS * EDMAPKT_NUM_TX_PARAMS_PER_PKT)
#define EDMAPKT_NUM_TX_PARAMS  (EDMAPKT_NUM_TX_PARAMS_TOT_PKTS + EDMAPKT_NUM_TX_PARAMS_TOT_LINKAGE)
#define EDMAPKT_NUM_TX_PARAMS_PER_LINKAGE ((EDMAPKT_NUM_TX_PKTS_PER_LINKAGE * EDMAPKT_NUM_TX_PARAMS_PER_PKT) + \
                                            EDMAPKT_NUM_TX_PARAMS_LINKAGE)
#define EDMAPKT_TX_PKT_SIZE    (64)
#define EDMAPKT_TX_PKT_RAM_SIZE (EDMAPKT_TX_PKT_SIZE * EDMAPKT_NUM_TX_PKTS)

#define EDMAPKT_NUM_RX_PKTS    (1)
#define EDMAPKT_NUM_RX_PKTS_PER_LINKAGE (1)
#define EDMAPKT_NUM_RX_PARAMS_PER_PKT (3)
#define EDMAPKT_NUM_RX_PARAMS_LINKAGE (1)
#define EDMAPKT_NUM_RX_PARAMS_TOT_LINKAGE ((EDMAPKT_NUM_RX_PKTS + EDMAPKT_NUM_RX_PARAMS_LINKAGE - 1) / \
                                           (EDMAPKT_NUM_RX_PKTS_PER_LINKAGE))
#define EDMAPKT_NUM_RX_PARAMS_TOT_PKTS (EDMAPKT_NUM_RX_PKTS * EDMAPKT_NUM_RX_PARAMS_PER_PKT)
#define EDMAPKT_NUM_RX_PARAMS  (EDMAPKT_NUM_RX_PARAMS_TOT_PKTS + EDMAPKT_NUM_RX_PARAMS_TOT_LINKAGE)
#define EDMAPKT_NUM_RX_PARAMS_PER_LINKAGE ((EDMAPKT_NUM_RX_PKTS_PER_LINKAGE * EDMAPKT_NUM_RX_PARAMS_PER_PKT) + \
                                            EDMAPKT_NUM_RX_PARAMS_LINKAGE)
#define EDMAPKT_RX_PKT_SIZE    (64)
#define EDMAPKT_RX_PKT_RAM_SIZE (EDMAPKT_RX_PKT_SIZE * EDMAPKT_NUM_RX_PKTS)


#define EDMAPKT_NUM_PORTS 2

typedef struct edmaPktBenchTxBuf_s {
   uint8_t buf[EDMAPKT_TX_PKT_RAM_SIZE];
   volatile void *txSrcDesc[EDMAPKT_NUM_TX_PKTS]; /* "Descriptor" holding real source address */
} edmaPktBenchTxBuf_t;

typedef struct edmaPktBenchRxBuf_s {
   uint8_t buf[EDMAPKT_RX_PKT_RAM_SIZE];
} edmaPktBenchRxBuf_t;

typedef struct edmaPktBenchBuf_s {
  volatile uint32_t msiTracker[16];  /* *16 for align and pad to cache line */
  edmaPktBenchTxBuf_t tx[EDMAPKT_NUM_PORTS];
  edmaPktBenchRxBuf_t rx[EDMAPKT_NUM_PORTS];
} edmaPktBenchBuf_t;

int32_t PcieEdmaPktBench (volatile uint32_t *msi, edmaPktBenchBuf_t *buf, 
                          pcieMode_e mode, SemaphoreP_Handle *msiSem);

#endif /* PCIEEDMAPKTBENCH_H_ */

/* Nothing past this point */

