/****************************************************************************\
 *           (C) Copyright 2013, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#ifndef __TPR_17112_H__
#define __TPR_17112_H__

#define NUM_CHLS                16
#define NUM_PKTS_PER_CHL        8
#define NUM_PKTS_ALL_CHLS       (NUM_CHLS * NUM_PKTS_PER_CHL)

#define NUM_AXCS                4
#define NUM_PKTS_PER_AXC        7
#define NUM_PKTS_ALL_AXCS       (NUM_AXCS * NUM_PKTS_PER_AXC)

/* Define queues for common FDQs */
#define IQ_TX_COMPLETE_Q        2000
#define IQ_RX_FDQ               2001
#define CTL_TX_COMPLETE_Q       2002
#define CTL_RX_FDQ              2003

/* These are for the CTL test */
#define IQN2_QUE2CHL(q)			((q)-832)
#define IQN2_CHL2QUE(c)			((c)+832)
#define CTL_CHL_BASE		    IQN2_QUE2CHL(CTL_TX_Q)
#define IQ_RX_Q                 3000 // 3000 ~ 3003
#define IQ_TX_Q                 832  // 832 ~ 835
#define CTL_RX_Q                3004
#define CTL_TX_Q                864
#define PKTDMA_GBL_CFG_REGION   DFE_IQN2_PKTDMA_GBL_CFG_REGION
#define PKTDMA_TX_CHAN_REGION   DFE_IQN2_PKTDMA_TX_CHAN_REGION
#define PKTDMA_RX_CHAN_REGION   DFE_IQN2_PKTDMA_RX_CHAN_REGION
#define PKTDMA_TX_SCHD_REGION   DFE_IQN2_PKTDMA_TX_SCHD_REGION
#define PKTDMA_RX_FLOW_REGION   DFE_IQN2_PKTDMA_RX_FLOW_REGION

extern void waitloop(Uint32 loopCount);
#endif // __TPR_17112_H__

