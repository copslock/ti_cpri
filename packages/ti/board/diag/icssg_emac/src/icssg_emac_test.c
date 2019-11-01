/******************************************************************************
 * Copyright (c) 2018-2019 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/
/**
 *  \file   icssg_emac_test.c
 *
 *  \brief  icssg emac diagnostic test file
 *
 *  Targeted Functionality: Verification of basic functionality of icssg emac
 *  interface.
 *
 *  Operation: This is the port to port loopback test for verifying the
 *  icssg emac interface. Test is run with PRU port 0 and PRU port 1 of an ICSS
 *  instance connected with Ethernet cable. Packets are sent from PRU port 0
 *  and received on PRU port 1. Similarly packets are sent form PRU port 1 and
 *  received on PRU port 0. Test is run at 1000mpbs Ethernet speed.
 *
 *  Setup:
 *  AM65xx EVM
 *  Connect Ethernet cable between PRU2 ETH0 and ETH1 ports (J14) on CP board
 *
 *  AM65xx IDK
 *  Connect Ethernet cable between PRU0 ETH0 and ETH1 ports (J3) on IDK board
 *  Connect Ethernet cable between PRU1 ETH0 and ETH1 ports (J1) on IDK board
 *  Connect Ethernet cable between PRU2 ETH0 and ETH1 ports (J14) on CP board
 *
 *  Supported SoCs: AM65XX.
 *
 *  Supported Platforms: am65xx_evm am65xx_idk.
 *
 */


#include "icssg_emac_test.h"

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
uint32_t interposerCardPresent = 0;

Board_IDInfo_v2 gIcssEmacBoardInfo;

/*
 * UDMA driver objects
 */
struct Udma_DrvObj      gUdmaDrvObj;
struct Udma_ChObj       gUdmaTxChObj[EMAC_MAX_PORTS][4];
struct Udma_ChObj       gUdmaRxChObj[EMAC_MAX_PORTS][4];
struct Udma_EventObj    gUdmaRxCqEventObj[EMAC_MAX_PORTS][4];

struct Udma_ChObj       gUdmaRxCfgPsiChObj[EMAC_MAX_PORTS];
struct Udma_EventObj    gUdmaRxCfgPsiCqEventObj[EMAC_MAX_PORTS];

Udma_DrvHandle          gDrvHandle = NULL;


uint8_t  app_pkt_buffer[APP_TOTAL_PKTBUF_SIZE] __attribute__ ((aligned (128))) __attribute__ ((section (".data_buffer")));
/* TX/RX ring entries memory */
uint8_t  gTxRingMem[EMAC_MAX_PORTS][RING_TRSIZE * RING_TRCNT] __attribute__ ((aligned (CACHE_LINESZ)));
uint8_t  gTxCompRingMem[EMAC_MAX_PORTS][RING_TRSIZE * RING_TRCNT] __attribute__ ((aligned (CACHE_LINESZ)));
uint8_t  gRxRingMem[EMAC_MAX_PORTS][RING_TRSIZE * RING_TRCNT] __attribute__ ((aligned (CACHE_LINESZ)));
uint8_t  gRxCompRingMem[EMAC_MAX_PORTS][RING_TRSIZE * RING_TRCNT] __attribute__ ((aligned (CACHE_LINESZ)));
uint8_t gRxCfgPsiRingMem[EMAC_MAX_PORTS][RING_TRSIZE * RING_TRCNT] __attribute__ ((aligned(CACHE_LINESZ)));
uint8_t gRxCfgPsiCompRingMem[EMAC_MAX_PORTS][RING_TRSIZE * RING_TRCNT] __attribute__ ((aligned(CACHE_LINESZ)));



/* TX/RX ring CPPI descriptor memory */
uint8_t gUdmapDescRamTx[EMAC_MAX_PORTS][RING_TRCNT*UDMAP_DESC_SIZE] __attribute__ ((aligned (CACHE_LINESZ)))__attribute__ ((section (".data_buffer"))); 
uint8_t gUdmapDescRamRx[EMAC_MAX_PORTS][RING_TRCNT*UDMAP_DESC_SIZE] __attribute__ ((aligned (CACHE_LINESZ)))__attribute__ ((section (".data_buffer")));
uint8_t gUdmapDescRamRxCfgPsi[EMAC_MAX_PORTS][RING_TRCNT * UDMAP_DESC_SIZE] __attribute__ ((aligned(CACHE_LINESZ))) __attribute__ ((section (".data_buffer")));


static pru_rtu_fw_t firmware[2] = {
    { PDSPcode_0, sizeof(PDSPcode_0), PDSP2code_0, sizeof(PDSP2code_0) },
    { PDSP3code_0, sizeof(PDSP3code_0), PDSP4code_0, sizeof(PDSP4code_0) },
};


static uint32_t pkt_rcv_count = 0;
volatile uint32_t pkt_rcvd = 0;
uint32_t gTxPktCount     = 0;
uint32_t linkCheckTime   = 0;
uint32_t linkUp          = 0;
int      hs_index[EMAC_MAX_ICSS * EMAC_MAC_PORTS_PER_ICSS];//one per icss slice
uint8_t icss_tx_port_queue[3][100352] __attribute__ ((aligned (CACHE_LINESZ)));

PRUICSS_Handle prussHandle[EMAC_MAX_ICSS] = {NULL, NULL, NULL};
APP_EMAC_MCB_T   app_mcb;
EMAC_LINK_INFO_T link_info;
EMAC_MAC_ADDR_T  macTest;

EMAC_HwAttrs_V5         emac_cfg;
EMAC_CHAN_MAC_ADDR_T    chan_cfg[APP_EMAC_NUM_CHANS_PER_CORE];

pruicssgDiagMdioInfo  gPruicssTestMdioInfo[EMAC_MAX_PORTS_ICSS] = {{(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE+CSL_ICSS_MDIO_CFG_BASE), BOARD_ICSS0_EMAC_PHY0_ADDR, 0x6000, 0x0500},
                                                          {(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE+CSL_ICSS_MDIO_CFG_BASE), BOARD_ICSS0_EMAC_PHY1_ADDR, 0x6003, 0x0500},
                                                          {(CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE+CSL_ICSS_MDIO_CFG_BASE), BOARD_ICSS1_EMAC_PHY0_ADDR, 0x6000, 0x0500},
                                                          {(CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE+CSL_ICSS_MDIO_CFG_BASE), BOARD_ICSS1_EMAC_PHY1_ADDR, 0x6003, 0x0500},
                                                          {(CSL_PRU_ICSSG2_DRAM0_SLV_RAM_BASE+CSL_ICSS_MDIO_CFG_BASE), BOARD_ICSS2_EMAC_PHY0_ADDR, 0, 0x100},
                                                          {(CSL_PRU_ICSSG2_DRAM0_SLV_RAM_BASE+CSL_ICSS_MDIO_CFG_BASE), BOARD_ICSS2_EMAC_PHY1_ADDR, 3, 0x100},
                                                        };

/**********************************************************************
 ****************** Test Configuration Variables **********************
 **********************************************************************/

static const uint8_t test_pkt[TEST_PKT_SIZE] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* broadcast mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02,
    0x01,0x02,0x03,0x04,
    0x01,0x02,0x03,0x04,
    0x01,0x02,0x03,0x04,
    0x01,0x02,0x03,0x04,
    0x01,0x02,0x03,0x04,
    0xfe,0xfe
};





/**********************************************************************
 ************************ EMAC TEST FUNCTIONS *************************
 **********************************************************************/



/**
 * \brief  Detect presence of interposer card
 *
 *
 * \param   none
 *
 * \return  TRUE if present, FALSE if NOT present
 *
 */

bool BoardDiag_DetectInterposerCard(void)
{
    EMAC_HwAttrs_V5 emac_cfg;
    EMAC_socGetInitCfg(0, &emac_cfg);
    CSL_MdioRegs *pBaseAddr = (CSL_MdioRegs*) emac_cfg.portCfg[2].mdioRegsBaseAddr;
    if ((CSL_MDIO_isPhyAlive(pBaseAddr,emac_cfg.portCfg[2].phyAddr)) ||(CSL_MDIO_isPhyAlive(pBaseAddr,emac_cfg.portCfg[3].phyAddr)))
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}


/**
 * \brief  Application queue pop function
 *
 * Dequeues a packet descriptor from an app queue.
 *
 * \param   pq         [IN/OUT]  Packet queue
 *
 * \return  EMAC_Pkt popped from the queue
 *
 */
static EMAC_PKT_DESC_T* BoardDiag_appQueuePop(APP_PKT_QUEUE_T   *pq)
{
    EMAC_PKT_DESC_T *pPktHdr;

    if (!pq->Count)
    {
        return 0;
    }

    pPktHdr = pq->pHead;
    if( pPktHdr )
    {
        pq->pHead = pPktHdr->pNext;
        pq->Count--;
        pPktHdr->pPrev = pPktHdr->pNext = 0;
    }

    return( pPktHdr );
}

/**
 * \brief  Application queue push function
 *
 * Enqueues a packet in EMAC_Pkt queue.
 *
 * \param   pq          [OUT]       Packet queue
 *          pPktHdr     [IN]        Packet to push
 *
 */
static void BoardDiag_appQueuePush(APP_PKT_QUEUE_T *pq,
                                   EMAC_PKT_DESC_T *pPktHdr)
{
    pPktHdr->pNext = 0;

    if( !pq->pHead )
    {
        /* Queue is empty - Initialize it with this one packet */
        pq->pHead = pPktHdr;
        pq->pTail = pPktHdr;
    }
    else
    {
        /* Queue is not empty - Push onto end */
        pq->pTail->pNext = pPktHdr;
        pq->pTail        = pPktHdr;
    }

    pq->Count++;
}

/**
 * \brief  Allocate packet descriptor function
 *
 * This function gets packet from the free Queue and returns to the application
 *
 * \param   portNum   [IN]  EMAC port number
 *          pktSize   [IN]  size of the packet
 *
 * \return   pointer to the allocated packet descriptor.
 *
 */
EMAC_PKT_DESC_T* BoardDiag_AppAllocPkt(uint32_t  portNum, uint32_t pktSize)
{
    EMAC_PKT_DESC_T    *p_pkt_desc = NULL;

    if (pktSize <= APP_EMAC_MAX_PKT_SIZE)
    {
        /* Get a packet descriptor from the free queue */
        p_pkt_desc              = BoardDiag_appQueuePop(&app_mcb.emac_pcb[portNum].freeQueue);
        if(p_pkt_desc != NULL)
        {
            p_pkt_desc->AppPrivate  = (uint32_t)p_pkt_desc;
            p_pkt_desc->BufferLen   = APP_EMAC_MAX_PKT_SIZE;
            p_pkt_desc->DataOffset  = 0;
            p_pkt_desc->pPrev = NULL;
            p_pkt_desc->pNext = NULL;
        }
    }
    else
    {
        UART_printf ("BoardDiag_AppAllocPkt on port %d failed, packet size %d \
                      is big\n", portNum, pktSize);
        return NULL;
    }

    return p_pkt_desc;
}

/**
 * \brief  Free packet function
 *
 * This function pushes the free packet descriptor into the queue
 *
 * \param   portNum    [IN]  EMAC port number
 *          pPktDesc   [IN]  Packet descriptor
 *
 */
void BoardDiag_AppFreePkt(uint32_t  portNum,
                          EMAC_PKT_DESC_T *pPktDesc)
{
    /* Free a packet descriptor to the free queue */
    BoardDiag_appQueuePush(&app_mcb.emac_pcb[portNum].freeQueue,
                   (EMAC_PKT_DESC_T *)pPktDesc->AppPrivate);
}

/**
 * \brief  PHY register write function
 *
 * This function is used to Read a PHY register using MDIO.
 *
 * \param   baseAddr [IN]   MDIO base address
 *          phyAddr  [IN]   PHY Address
 *          regAddr  [IN]   Register offset to be written
 *          regData  [OUT]  Pointer where the read value shall be written
 *
 * \return  uint32_t
            TRUE     Read is successful.
 *          FALSE    Read is not acknowledged properly.
 */
static uint32_t BoardDiag_emacPhyRegRead(uint32_t baseAddr, uint32_t phyAddr,
                                         uint32_t regAddr, uint16_t *regData)
{
    uint32_t regVal = 0U;
    uint32_t retVal = 0U;

    /* Wait till transaction completion if any */
    while(HW_RD_FIELD32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U),
        CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO) == 1)
    {}
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO,1);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, 0);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regAddr);
    HW_WR_REG32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U), regVal);

    /* wait for command completion */
    while(HW_RD_FIELD32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U),
          CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO) == 1)
    {}

    /* Store the data if the read is acknowledged */
    if(HW_RD_FIELD32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U),
        CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK) == 1)
    {
        *regData = (uint16_t)(HW_RD_FIELD32(baseAddr + \
                    CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U),
                    CSL_MDIO_USER_GROUP_USER_ACCESS_REG_DATA));
        retVal = (uint32_t)TRUE;
    }
    else
    {
        retVal = (uint32_t)FALSE;
    }

    return(retVal);
}

/**
 * \brief  PHY register write function
 *
 * This function is used to writes a PHY register using MDIO.
 *
 * \param   baseAddr [IN]   MDIO base address
 *          phyAddr  [IN]   PHY Address
 *          regAddr  [IN]   Register offset to be written
 *          data     [IN]   Value to be written
 *
 */
static void BoardDiag_emacPhyRegWrite(uint32_t baseAddr, uint32_t phyAddr,
                                      uint32_t regAddr, uint16_t data)
{
    uint32_t regVal = 0U;

    /* Wait till transaction completion if any */
    while(HW_RD_FIELD32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U),
          CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO) == 1)
    {}

    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO, 1);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, 1);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regAddr);
    HW_SET_FIELD(regVal, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_DATA, data);
    HW_WR_REG32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U), regVal);

    /* wait for command completion */
    while(HW_RD_FIELD32(baseAddr + CSL_MDIO_USER_GROUP_USER_ACCESS_REG(0U),
          CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO) == 1)
    {}
}

/**
 * \brief  Function to write extended address registers of Ethernet PHY
 *
 * \param   baseAddr [IN]    MDIO base address
 *          phyAddr  [IN]    Ethernet PHY address
 *          regNum   [IN]    PHY Register address
 *          pData    [OUT]   Values read from register
 *
 */
static void BoardDiag_emacPhyExtendedRegRead (uint32_t baseAddr,
                                              uint32_t phyAddr, uint32_t regNum,
                                              uint16_t *pData)
{
    BoardDiag_emacPhyRegWrite(baseAddr, phyAddr, 0x0D, 0x001F);
	BoardDiag_emacPhyRegWrite(baseAddr, phyAddr, 0x0E, regNum);
	BoardDiag_emacPhyRegWrite(baseAddr, phyAddr, 0x0D, 0x401F);
	BoardDiag_emacPhyRegRead(baseAddr, phyAddr, 0x0E, pData);
}

/**
 * \brief  Application initialization function
 *
 * This function initialize the application control block and
 * free/rx packet queue.
 *
 */
static void BoardDiag_appInit(void)
{
    EMAC_PKT_DESC_T  *p_pkt_desc;
    uint32_t         index;
    uint32_t         count;
    uint8_t          *pktbuf_ptr;

    UART_printf ("\nEMAC loopback test application initialization\n");

    /* Reset application control block */
    memset(&app_mcb, 0, sizeof (APP_EMAC_MCB_T));

    app_mcb.core_num = 0;
    /* packet buffer stores in internal memory */
    pktbuf_ptr = (uint8_t *) ((uint32_t) app_pkt_buffer) ;

    /* Initialize the free packet queue */
    for (index = 0; index < EMAC_MAX_NUM_EMAC_PORTS; index++)
    {
        for (count = 0; count < APP_MAX_PKTS; count++)
        {
            p_pkt_desc               = &app_mcb.emac_pcb[index].pkt_desc[count];
            p_pkt_desc->pDataBuffer  = pktbuf_ptr;
            p_pkt_desc->BufferLen    = APP_EMAC_MAX_PKT_SIZE;
            BoardDiag_appQueuePush(&app_mcb.emac_pcb[index].freeQueue,
                                   p_pkt_desc );
            pktbuf_ptr += APP_EMAC_MAX_PKT_SIZE;
        }
    }
}

/**
 * \brief  Send packet function
 *
 * This function is used to send packets to the specified emac port.
 * Functional test will send all the packets and receive but stress test will
 * send one packet and receive one packet due to poll packet limitation.
 *
 * \param   portNum [IN]   EMAC port number
 *
 */
static void BoardDiag_appTestSendPkts(uint32_t portNum)
{
#ifdef DIAG_STRESS_TEST
    if(gTxPktCount < PKT_SEND_COUNT)
    {
        UART_printf("Sending Packet: %d\n", (gTxPktCount+1));
        EMAC_PKT_DESC_T *p_pkt_desc = BoardDiag_AppAllocPkt(portNum,
                                                            TEST_PKT_SIZE);
        if(p_pkt_desc!=NULL)
        {
          memcpy (p_pkt_desc->pDataBuffer, &test_pkt[0], TEST_PKT_SIZE);
          p_pkt_desc->AppPrivate   = (uint32_t)p_pkt_desc;
          p_pkt_desc->Flags        = EMAC_PKT_FLAG_SOP | EMAC_PKT_FLAG_EOP;
          p_pkt_desc->ValidLen     = TEST_PKT_SIZE;
          p_pkt_desc->DataOffset   = 0;
          p_pkt_desc->PktChannel   = 0;
          p_pkt_desc->PktLength    = TEST_PKT_SIZE;
          p_pkt_desc->BufferLen    = TEST_PKT_SIZE;
          p_pkt_desc->PktFrags     = 1;
          p_pkt_desc->pNext        = NULL;
          p_pkt_desc->pPrev        = NULL;
          

          emac_send(portNum, p_pkt_desc);
          gTxPktCount++;
        }  
    }
#else
    uint32_t index;
    uint32_t pkt_send_count =0;
    for (index = 0; index < PKT_SEND_COUNT; index++)
    {
        UART_printf("Sending Packet: %d\n", (index+1));
        EMAC_PKT_DESC_T *p_pkt_desc = BoardDiag_AppAllocPkt(portNum,
                                                            TEST_PKT_SIZE);
       if(p_pkt_desc!=NULL)
        {
          memcpy (p_pkt_desc->pDataBuffer, &test_pkt[0], TEST_PKT_SIZE);
          p_pkt_desc->AppPrivate   = (uint32_t)p_pkt_desc;
          p_pkt_desc->Flags        = EMAC_PKT_FLAG_SOP | EMAC_PKT_FLAG_EOP;
          p_pkt_desc->ValidLen     = TEST_PKT_SIZE;
          p_pkt_desc->DataOffset   = 0;
          p_pkt_desc->PktChannel   = 0;
          p_pkt_desc->PktLength    = TEST_PKT_SIZE;
          p_pkt_desc->BufferLen    = TEST_PKT_SIZE;
          p_pkt_desc->PktFrags     = 1;
          p_pkt_desc->pNext        = NULL;
          p_pkt_desc->pPrev        = NULL;

          emac_send(portNum, p_pkt_desc);
          pkt_send_count++;
	   }
    }
#endif
}

/**
 * \brief  Receive call back function
 *
 * This function is used to call back the network application when a
 * packet is received.
 * In this function emac stress test will trigger test packet send function to
 * send one packet each time.
 * After receiving all the packets it will call emac close function to exit
 * from the poll pkt function.
 *
 * \param   portNum  [IN]   EMAC port number
 *          pDesc    [IN]   Packet descriptor
 *
 */
void BoardDiag_AppTestRxPktCb(uint32_t portNum, EMAC_PKT_DESC_T *pDesc)
{
	/* Change buffer length to not compare checksum data */
	pDesc->BufferLen = TEST_PKT_SIZE;
    if (memcmp(pDesc->pDataBuffer, test_pkt, pDesc->BufferLen) == 0)
    {
        pkt_rcvd = 1;
        pkt_rcv_count++;
        UART_printf("Received Packet: %d\n", pkt_rcv_count);
    }
    else
    {
        UART_printf("packet match failed\n");
    }

    BoardDiag_AppFreePkt(portNum,  (EMAC_PKT_DESC_T*) pDesc->AppPrivate);
}

/**
 * \brief  Link detect function
 *
 * This function used to detect the PRU ICSSG EMAC link status
 *
 * \param   portNum  [IN]   EMAC port number
 *
 * \return  int8_t
 *              0  - in case of link detect
 *             -1  - in case of failure
 *
 */
static int8_t BoardDiag_icssgEmacLinkDetect(uint32_t portNum)
{
    linkCheckTime = 0;

    do
    {
        emac_poll(portNum, &link_info);
		BOARD_delay(10);
		linkCheckTime++;
		if(linkCheckTime > LINK_TIMOUT_COUNT)
		{
			return (-1);
		}
    } while(link_info.link_status == EMAC_LINKSTATUS_NOLINK);

    return (0);
}

/**
 * \brief  Function to initialize MDIO
 *
 * \param   baseAddr [IN]   MDIO base address
 *
 * \return  uint32_t
            TRUE     Read is successful.
 *          FALSE    Read is not acknowledged properly.
 */
static void BoardDiag_mdioInit(uint32_t baseAddr)
{
    HW_WR_REG32((baseAddr + 0x4), (CSL_FMKT(MDIO_CONTROL_REG_ENABLE, YES)  |
                CSL_FMK(MDIO_CONTROL_REG_CLKDIV,0xFF)));
}

/**
 * \brief  PHY register dump test function
 *
 * This function used to read Ethernet PHY status and strapping registers
 * for debug purpose
 *
  * \param   portNum  [IN]   EMAC port number
 *
 * \return  int8_t
 *              0  - in case of success
 *             -1  - in case of failure
 *
 */
static int8_t BoardDiag_getPhyRegDump(uint32_t portNum)
{
    uint16_t regAddr;
    uint16_t regVal = 0;
    int8_t   ret    = 0;
    uint32_t baseAddr = gPruicssTestMdioInfo[portNum].mdioBaseAddrs;
    uint8_t phyAddr = gPruicssTestMdioInfo[portNum].phyAddrs;

    /* Initialize MDIO module. MDIO will get initialized multiple times
       by keeping this function call here which should be OK. Will help
       while calling BoardDiag_getPhyRegDump for differe MDIO instances */
    BoardDiag_mdioInit(baseAddr);

    /* On some of the AM65xx boards it is observed that ICSSG EMAC test is
       failing due to master-slave handshake failure as both the PHYs
       are trying to be in master mode. Forcing one of the PHYs to be master
       and other one to be a slave by default. */
    if((portNum % 2) == 0)
    {
        BoardDiag_emacPhyRegWrite(baseAddr,phyAddr,0x0009,0x1B00);
        BoardDiag_emacPhyRegRead(baseAddr, phyAddr, 0, &regVal);
        regVal |= 0x200; /* Restart Auto negotiation */
    }
    else
    {
        BoardDiag_emacPhyRegWrite(baseAddr,phyAddr,0x0009,0x1300);
        BoardDiag_emacPhyRegRead(baseAddr, phyAddr, 0, &regVal);
        regVal |= 0x200; /* Restart Auto negotiation */
        BoardDiag_emacPhyRegWrite(baseAddr,phyAddr, 0, regVal);

        do
        {
            BoardDiag_emacPhyRegRead(baseAddr, phyAddr, 1, &regVal);
        } while ((regVal & 0x20) == 0);
    }

    UART_printf("\n\nRegister Dump for PHY Addr - 0x%04X\n", phyAddr);

    for (regAddr = 0; regAddr < BOARD_ICSS_EMAC_REG_DUMP_MAX; regAddr++)
    {
        BoardDiag_emacPhyRegRead(baseAddr, phyAddr, regAddr, &regVal);
        UART_printf("PHY Register 0x%04X - 0x%04X\n", regAddr, regVal);
    }

    BoardDiag_emacPhyRegRead(baseAddr, phyAddr, 0x31, &regVal);
    UART_printf("PHY Configuration Register(CFG4) 0x%04X - 0x%04X\n",
                0x31, regVal);

    BoardDiag_emacPhyExtendedRegRead(baseAddr, phyAddr,
                            BOARD_ICSS_EMAC_STRAP_STS1_ADDR, &regVal);
    if(regVal != gPruicssTestMdioInfo[portNum].strapst1)
    {
        UART_printf("Default PHY Register(STRAP1) Data mismatch\n");
        ret = -1;
    }

    UART_printf("PHY Register(STRAP1) 0x%04X - 0x%04X\n",
                BOARD_ICSS_EMAC_STRAP_STS1_ADDR, regVal);

    BoardDiag_emacPhyExtendedRegRead(baseAddr, phyAddr,
                            BOARD_ICSS_EMAC_STRAP_STS2_ADDR, &regVal);
    if(regVal != gPruicssTestMdioInfo[portNum].strapst2)
    {
        UART_printf("Default PHY Register(STRAP2) Data mismatch\n");
        ret = -1;
    }

    UART_printf("PHY Register(STRAP2) 0x%04X - 0x%04X\n",
                BOARD_ICSS_EMAC_STRAP_STS2_ADDR, regVal);

    BoardDiag_emacPhyExtendedRegRead(baseAddr, phyAddr, 0x32, &regVal);
    UART_printf("RGMII Control Register (RGMIICTL) Value - 0x%04X\n", regVal);
    UART_printf("  --- RGMII_RX_CLK_DELAY - 0x%04X\n", (regVal & 0x1));
    UART_printf("  --- RGMII_TX_CLK_DELAY - 0x%04X\n", ((regVal >> 1) & 0x1));

    BoardDiag_emacPhyExtendedRegRead(baseAddr, phyAddr, 0x86, &regVal);
    UART_printf("RGMII Delay Control Register (RGMIIDCTL) Value - 0x%04X\n", regVal);

    return ret;
}



/**
 * \brief  PRU disable test function
 *
 * This function used to disable the PRU subsystem
 *
 * \param   instance [IN]  PRU-ICSS port number
 *
 */

int32_t  BoardDiag_disablePruss(uint32_t portNum)
{
    PRUICSS_Handle prussDrvHandle;
    uint8_t pru_n, rtu_n, slice_n ;

    if (portNum > 5)
        return -1;

    prussDrvHandle =prussHandle[portNum >> 1];
    if (prussDrvHandle == NULL)
        return -1;

    slice_n = (portNum & 1);
    pru_n = (slice_n) ? PRUICCSS_PRU1 : PRUICCSS_PRU0;
    rtu_n = (slice_n) ? PRUICCSS_RTU1 : PRUICCSS_RTU0;

    if (PRUICSS_pruDisable(prussDrvHandle, pru_n) != 0)
        UART_printf("PRUICSS_pruDisable for PRUICCSS_PRU%d failed\n", slice_n);

    if (PRUICSS_pruDisable(prussDrvHandle, rtu_n) != 0)
        UART_printf("PRUICSS_pruDisable for PRUICCSS_RTU%d failed\n", slice_n);

    /* CLEAR SHARED MEM which is used for host/firmware handshake */
    PRUICSS_pruInitMemory(prussDrvHandle, PRU_ICSS_SHARED_RAM);


    return 0;
}

/**
 * \brief  PRU initialization test function
 *
 * This function used to initialize the PRU subsystem
 *
 * \param   instance [IN]  PRU-ICSS port number
 *
 */
static int32_t  BoardDiag_initPruss(uint32_t portNum)
{
    PRUICSS_Handle prussDrvHandle;
    uint8_t firmwareLoad_done = FALSE;
    uint8_t pru_n, rtu_n, slice_n ;

    if (portNum > 5)
        return -1;

    prussDrvHandle =prussHandle[portNum >> 1];
    if (prussDrvHandle == NULL)
        return -1;

    slice_n = (portNum & 1);
    pru_n = (slice_n) ? PRUICCSS_PRU1 : PRUICCSS_PRU0;
    rtu_n = (slice_n) ? PRUICCSS_RTU1 : PRUICCSS_RTU0;

    if (PRUICSS_pruWriteMemory(prussDrvHandle,PRU_ICSS_IRAM(slice_n), 0,
                               firmware[slice_n].pru, firmware[slice_n].pru_size)) {
        if (PRUICSS_pruWriteMemory(prussDrvHandle,PRU_ICSS_IRAM(slice_n + 2), 0,
                                   firmware[slice_n].rtu, firmware[slice_n].rtu_size))
            firmwareLoad_done = TRUE;
        else
            UART_printf("PRUICSS_pruWriteMemory for PRUICCSS_PRU%d failed\n", slice_n);
    }
    else
        UART_printf("PRUICSS_pruWriteMemory for PRUICCSS_RTU%d failed\n", slice_n);

    if( firmwareLoad_done)
    {
        if (PRUICSS_pruEnable(prussDrvHandle, pru_n) != 0)
            UART_printf("PRUICSS_pruEnable for PRUICCSS_PRU%d failed\n", slice_n);
        if (PRUICSS_pruEnable(prussDrvHandle, rtu_n) != 0)
            UART_printf("PRUICSS_pruEnable for PRUICCSS_RTU%d failed\n", slice_n);
    }

    return 0;

}


void BoardDiag_setupFwDualmac(uint32_t port_num, EMAC_HwAttrs_V5 *pEmacCfg)
{
    EMAC_FW_APP_CONFIG *pFwAppCfg;
    emacGetDualMacFwAppInitCfg(port_num, &pFwAppCfg);
    if ((port_num % 2) == 0)
    {
        pFwAppCfg->txPortQueueLowAddr = 0xFFFFFFFF & ((uint32_t) &icss_tx_port_queue[port_num >> 1][0]);
    }
    else
    {
        pFwAppCfg->txPortQueueLowAddr = 0xFFFFFFFF & ((uint32_t) &icss_tx_port_queue[port_num >> 1][TX_BUFF_POOL_TOTAL_DUAL_MAC]);
    }

    pFwAppCfg->txPortQueueHighAddr = 0;

    emacSetDualMacFwAppInitCfg(port_num, pFwAppCfg);

    /* Need to update the emac configuraiton with  function required by the driver to get the FW configuration to write to shared mem */
    pEmacCfg->portCfg[port_num].getFwCfg = &emacGetDualMacFwConfig;
}

/**
 * \brief  ICSSG emac loopback test function
 *
 * This test is used to verify the Ethernet interface by sending and receiving
 * the same number of packets using external loopback cable.
 *
 * \param   portNum [IN] EMAC port number
 *
 * \return  int8_t
 *              0  - in case of success
 *             -1  - in case of failure
 *
 */
static int8_t BoardDiag_icssgemacLoopbackTest(uint32_t portNum)
{
    EMAC_OPEN_CONFIG_INFO_T open_cfg;
    EMAC_CONFIG_INFO_T      cfg_info;
    EMAC_DRV_ERR_E          open_ret;
    int8_t   ret    = 0;
    int32_t chanNum = 0;
    int32_t subChanNum = 0;

    EMAC_socGetInitCfg(0, &emac_cfg);
    emac_cfg.portCfg[portNum].nTxChans = 1;
    emac_cfg.portCfg[portNum].rxChannel.nsubChan = 1;
    emac_cfg.portCfg[portNum].rxChannelCfgOverPSI.nsubChan = 1;
    emac_cfg.portCfg[portNum].rxChannel2CfgOverPSI.nsubChan = 0;

    emac_cfg.portCfg[portNum].txChannel[chanNum].chHandle = (void *)&gUdmaTxChObj[portNum];
    emac_cfg.portCfg[portNum].txChannel[chanNum].freeRingMem= (void*)&gTxRingMem[portNum][0];
    emac_cfg.portCfg[portNum].txChannel[chanNum].compRingMem= (void*)&gTxCompRingMem[portNum][0];
    emac_cfg.portCfg[portNum].txChannel[chanNum].hPdMem = (void*)&gUdmapDescRamTx[portNum][0];

    emac_cfg.portCfg[portNum].rxChannel.chHandle = (void *)&gUdmaRxChObj[portNum];
    emac_cfg.portCfg[portNum].rxChannel.subChan[subChanNum].freeRingMem[0] = (void*)&gRxRingMem[portNum][0];
    emac_cfg.portCfg[portNum].rxChannel.subChan[subChanNum].compRingMem= (void*)&gRxCompRingMem[portNum][0];
    emac_cfg.portCfg[portNum].rxChannel.subChan[subChanNum].hPdMem[0] = (void*)&gUdmapDescRamRx[portNum][0];
    emac_cfg.portCfg[portNum].rxChannel.subChan[subChanNum].eventHandle = (void *)&gUdmaRxCqEventObj[portNum];


    emac_cfg.portCfg[portNum].rxChannelCfgOverPSI.chHandle = (void *)&gUdmaRxCfgPsiChObj[portNum];
    emac_cfg.portCfg[portNum].rxChannelCfgOverPSI.subChan[subChanNum].freeRingMem[0] = (void*)&gRxCfgPsiRingMem[portNum][0];
    emac_cfg.portCfg[portNum].rxChannelCfgOverPSI.subChan[subChanNum].compRingMem= (void*)&gRxCfgPsiCompRingMem[portNum][0];
    emac_cfg.portCfg[portNum].rxChannelCfgOverPSI.subChan[subChanNum].hPdMem[0] = (void*)&gUdmapDescRamRxCfgPsi[portNum][0];
    emac_cfg.portCfg[portNum].rxChannelCfgOverPSI.subChan[subChanNum].eventHandle = (void *)&gUdmaRxCfgPsiCqEventObj[portNum];


    /* Firmware config */
    BoardDiag_setupFwDualmac(portNum, &emac_cfg);

    /* Now set the config*/
    EMAC_socSetInitCfg(0, &emac_cfg);

    BoardDiag_appInit();

    memset(&open_cfg, 0, sizeof(EMAC_OPEN_CONFIG_INFO_T));
    open_cfg.hwAttrs            = (void*)&emac_cfg;
    open_cfg.alloc_pkt_cb       = BoardDiag_AppAllocPkt;
    open_cfg.free_pkt_cb        = BoardDiag_AppFreePkt;
    open_cfg.loop_back          = 0;
    open_cfg.master_core_flag   = 1;
    open_cfg.max_pkt_size       = APP_EMAC_INIT_PKT_SIZE;
    open_cfg.mdio_flag          = 1;
    open_cfg.num_of_chans       = 1;
    open_cfg.num_of_rx_pkt_desc = EMAC_RX_PKT_DESC_COUNT;
    open_cfg.num_of_tx_pkt_desc = EMAC_RX_PKT_DESC_COUNT;
    open_cfg.phy_addr           = gPruicssTestMdioInfo[portNum].phyAddrs;
    open_cfg.p_chan_mac_addr    = &chan_cfg[0];
    open_cfg.rx_pkt_cb          = BoardDiag_AppTestRxPktCb;
    open_cfg.mode_of_operation  = EMAC_MODE_POLL;
	open_cfg.udmaHandle = (void*)gDrvHandle;

    /* Set the channel configuration */
    chan_cfg[0].chan_num         = 0;
    chan_cfg[0].num_of_mac_addrs = 1;

    macTest.addr[0] = 0x48;
    macTest.addr[1] = 0x93;
    macTest.addr[2] = 0xfe;
    macTest.addr[3] = 0xfa;
    macTest.addr[4] = 0x18;
    macTest.addr[5] = 0x44;
    chan_cfg[0].p_mac_addr = & macTest;




    open_ret = emac_open(portNum, &open_cfg);
    if (open_ret == EMAC_DRV_RESULT_OPEN_PORT_ERR)
    {
        UART_printf("main: emac_open failure: %d\n", open_ret);
        return (-1);
    }
    else
        UART_printf("main: emac_open success\n");


    BoardDiag_initPruss(portNum);

    cfg_info.mcast_cnt    = 0;
    cfg_info.p_mcast_list = NULL;
    cfg_info.rx_filter    = EMAC_PKTFLT_MULTICAST;;
    emac_config(portNum, &cfg_info);

    UART_printf("\n\nWaiting for LINK UP, Make sure to plugin loopback cable\n");
    if(BoardDiag_icssgEmacLinkDetect(portNum))
    {
        UART_printf("PRU_ICSS port %d LINK TIMEOUT!!\n",portNum);
    }
    else
    {
        UART_printf("PRU_ICSS port %d LINK IS UP!\n",portNum);
    }


#if !defined(ICSSG_PORT2PORT_TEST)
    BoardDiag_appTestSendPkts(portNum);
#ifdef  DIAG_STRESS_TEST
    emac_poll_pkt(portNum);
#else
    emac_poll_pkt(portNum);
    emac_close(portNum);
#if defined(SOC_AM65XX)
    Udma_deinit(&gUdmaDrvObj);
#endif
#endif

    if (pkt_rcv_count == PKT_SEND_COUNT)
    {
        UART_printf("\nPackets sent: %d, Packets received: %d\n",
                    PKT_SEND_COUNT, pkt_rcv_count);
        UART_printf("\nEthernet Loopback test passed\n");
        ret = 0;
    }
    else
    {
        UART_printf("\nEthernet Loopback test failed!!\n");
        ret = -1;
    }
    pkt_rcv_count = 0;

	UART_printf("All tests completed\n");
#endif  /* ICSSG_PORT2PORT_TEST */

    return (ret);
}

#if !defined(DIAG_STRESS_TEST) && !defined(ICSSG_PORT2PORT_TEST)
/**
 * \brief  emac cable connect and disconnect test function
 *
 * This function verifies the Ethernet interface by disconnecting and
 * reconnecting the Ethernet loopback cable and re-running the test.
 *
 * \param   portNum [IN] EMAC port number
 *
 * \return  int8_t
 *              0  - in case of success
 *             -1  - in case of failure
 *
 */
static int8_t BoardDiag_emacCableDisconTest(uint8_t portNum)
{
    UART_printf("Please disconnect the loopback cable \n");

    /* Waiting to disconnect the Ethernet cable
     * After disconnecting the cable shows the link status
     */
    while(1)
    {
        emac_poll(portNum, &link_info);

        if((link_info.link_status_change) && (link_info.link_status ==
                                              EMAC_LINKSTATUS_NOLINK))
        {
            UART_printf("Link is Down\n");
            break;
        }
    }

    UART_printf("Please reconnect the loopback cable \n");

    /* Waiting to reconnect the Ethernet cable
     * After connecting the cable shows the link status
     */
    while(1)
    {
        emac_poll(portNum, &link_info);

        if((link_info.link_status_change) && (link_info.link_status !=
                                                    EMAC_LINKSTATUS_NOLINK))
        {
            UART_printf("Link is UP\n");
            break;
        }
    }

    /* Re-run the loopback test to confirm the interface functionality is intact
       after cable disconnect and connect */
    return BoardDiag_icssgemacLoopbackTest(portNum);
}
#endif





/**
 * \brief  emac test function
 *
 * This function does UDMA init
 *
 * \param   void
 *
 * \return  void
 *              0  - in case of success
 *              1  - in case of failure
 *
 */
void BoardDiag_IcssgUdmaInit(void)
{
    int32_t         retVal = UDMA_SOK;
    Udma_InitPrms   initPrms;
    uint32_t        instId;
    /* UDMA driver init */
#if defined (__aarch64__)
    instId = UDMA_INST_ID_MAIN_0;
#else
    instId = UDMA_INST_ID_MCU_0;
#endif
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.rmInitPrms.numIrIntr = EMAC_MAX_PORTS;
    retVal = Udma_init(&gUdmaDrvObj, &initPrms);
    if(UDMA_SOK == retVal)
    {
        gDrvHandle = &gUdmaDrvObj;
    }
}

/**
 * \brief  emac test function
 *
 * This function executes emac diagnostic test on interposer
 *
 * \param   portNum [IN] EMAC port number
 *
 * \return  int8_t
 *              0  - in case of success
 *              1  - in case of failure
 *
 */
int8_t BoardDiag_IcssgEmacTestInterposer(void)
{
    int8_t  ret;
    uint32_t portNum;
    uint8_t startInstance = 0;
    uint32_t startPort;
    PRUICSS_Config  *prussCfg;

    UART_printf  ("***************************************\n");
    UART_printf  ("*           ICSSG EMAC TEST           *\n");
    UART_printf  ("***************************************\n");

    startInstance = startInstance;
#if defined (am65xx_idk)
    startInstance = PRUICCSS_INSTANCE_ONE;
    startPort     = 0;
#else
    startInstance = PRUICCSS_INSTANCE_THREE;
    startPort     = 4;
#endif

    UART_printf("\n\nPerforming UDMA driver init...\n");
    BoardDiag_IcssgUdmaInit();

    UART_printf("\n\nReading Ethernet PHY Register Dump...\n");
    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        if (1 == interposerCardPresent)
        {
            if((portNum == 2) || (portNum == 3))
                 continue;
        }
        ret = BoardDiag_getPhyRegDump(portNum);
        if(ret != 0)
        {
            UART_printf("Strapping Registers data mismatch\n");
        }
    }

    PRUICSS_socGetInitCfg(&prussCfg);
    prussHandle[0] =  PRUICSS_create((PRUICSS_Config*)prussCfg,PRUICCSS_INSTANCE_ONE);
    prussHandle[1] =  PRUICSS_create((PRUICSS_Config*)prussCfg,PRUICCSS_INSTANCE_TWO);
    prussHandle[2] =  PRUICSS_create((PRUICSS_Config*)prussCfg,PRUICCSS_INSTANCE_THREE);

    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        BoardDiag_disablePruss(portNum);
    }

    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        ret = BoardDiag_icssgemacLoopbackTest(portNum);
        if(ret != 0)
        {
            return ret;
        }
    }
#if defined(ICSSG_PORT2PORT_TEST)
    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS;portNum++)
    {
        /* with presence of interposer card, we dont send on ports 0 and 2 */
        if (1 == interposerCardPresent)
        {
            if((portNum == 0) || (portNum == 2))
                 continue;
        }
        UART_printf("\n\nSending Packets on Port - %d\n", portNum);
        BoardDiag_appTestSendPkts(portNum);
        /* Wait to allow packets to come back */
        BOARD_delay(1000);
        if (1== interposerCardPresent)
        {
            if (portNum == 1)
            {
                UART_printf("\nReceiving Packets on Port - %d\n", (portNum-1));
                emac_poll_pkt(portNum-1);
            }
            else if (portNum == 3)
            {
                UART_printf("\nReceiving Packets on Port - %d\n", (portNum-1));
                emac_poll_pkt(portNum-1);
            }
            else if (portNum == 4)
            {
                UART_printf("\nReceiving Packets on Port - %d\n", (portNum + 1));
                emac_poll_pkt(portNum + 1);
            }
            else if (portNum == 5)
            {
                UART_printf("\nReceiving Packets on Port - %d\n", (portNum -1));
                emac_poll_pkt(portNum - 1);
            }
        }
        else
        {
            UART_printf("\nReceiving Packets on Port - %d\n", (portNum + 1));
            emac_poll_pkt(portNum + 1);
        }
        if (pkt_rcv_count == PKT_SEND_COUNT)
        {
            UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                        PKT_SEND_COUNT, pkt_rcv_count);
            UART_printf("Port %d Send to Port %d Receive Test Passed!\n",
                        portNum, (portNum + 1));
            ret = 0;
        }
        else
        {
            UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                        PKT_SEND_COUNT, pkt_rcv_count);
            UART_printf("Port %d Send to Port %d Receive Test Failed!!\n",
                        portNum, (portNum + 1));
            ret = -1;
            break;
        }
        pkt_rcv_count = 0;
    }

    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        emac_close(portNum);
    }

#if defined(SOC_AM65XX)
    Udma_deinit(&gUdmaDrvObj);
#endif

    if(ret)
    {
        UART_printf("\n\nICSSG Ethernet Port to Port Test Failed!!\n");
    }
    else
    {
        UART_printf("\n\nICSSG Ethernet Port to Port Test Passed!\n");
    }

    UART_printf("All Tests Completed\n");
#endif /* ICSSG_PORT2PORT_TEST */

    return ret;
}


/**
 * \brief  emac test function
 *
 * This function executes emac diagnostic test
 *
 * \param   portNum [IN] EMAC port number
 *
 * \return  int8_t
 *              0  - in case of success
 *              1  - in case of failure
 *
 */
int8_t BoardDiag_IcssgEmacTest(void)
{
    int8_t  ret;
    uint32_t portNum;
    uint8_t startInstance = 0;
    uint32_t startPort;
    PRUICSS_Config  *prussCfg;
#if defined(DIAG_STRESS_TEST)
    uint32_t index;
    uint32_t timeout;
    char rdBuf = 'y';
#endif

    startInstance = startInstance;

#ifdef DIAG_STRESS_TEST
    UART_printf  ("**********************************************\n");
    UART_printf  ("*           ICSSG EMAC STRESS TEST           *\n");
    UART_printf  ("**********************************************\n");
#else
    UART_printf  ("***************************************\n");
    UART_printf  ("*           ICSSG EMAC TEST           *\n");
    UART_printf  ("***************************************\n");
#endif

#if defined (am65xx_idk)
    startInstance = PRUICCSS_INSTANCE_ONE;
    startPort     = 0;

    /* Check for the IDK board version
       If Board_getIDInfo_v2 function fails, it indicates
       intial version of boards withour board ID for which
       no need to change the strapping details */
    if(!Board_getIDInfo_v2(&gIcssEmacBoardInfo,
                           BOARD_ICSS_EMAC_APP_BOARDID_ADDR))
    {
        /* HW Strapping is different on E4 IDK.
           Update the expected default strap values */
        if(gIcssEmacBoardInfo.boardInfo.pcbRev[1] >= '4')
        {
            for(portNum = startPort; portNum < BOARD_ICSS_MAX_PORTS_IDK; portNum++)
            {
                gPruicssTestMdioInfo[portNum].strapst1 &= ~0x6000;
            }
        }
    }

#else
    startInstance = PRUICCSS_INSTANCE_THREE;
    startPort     = 4;
#endif

	UART_printf("\n\nPerforming UDMA driver init...\n");
	BoardDiag_IcssgUdmaInit();

    UART_printf("\n\nReading Ethernet PHY Register Dump...\n");
    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        if (1 == interposerCardPresent)
        {
            if((portNum == 0) || (portNum == 2))
                 continue;
        }
        ret = BoardDiag_getPhyRegDump(portNum);
        if(ret != 0)
        {
            UART_printf("Strapping Registers data mismatch\n");
        }
    }

    PRUICSS_socGetInitCfg(&prussCfg);
    prussHandle[0] =  PRUICSS_create((PRUICSS_Config*)prussCfg,PRUICCSS_INSTANCE_ONE);
    prussHandle[1] =  PRUICSS_create((PRUICSS_Config*)prussCfg,PRUICCSS_INSTANCE_TWO);
    prussHandle[2] =  PRUICSS_create((PRUICSS_Config*)prussCfg,PRUICCSS_INSTANCE_THREE);

    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        BoardDiag_disablePruss(portNum);
    }

    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        ret = BoardDiag_icssgemacLoopbackTest(portNum);
        if(ret != 0)
        {
            return ret;
        }
#if !defined(DIAG_STRESS_TEST) && !defined(ICSSG_PORT2PORT_TEST)
        else
        {
            Board_init(BOARD_INIT_ETH_PHY);
            ret = BoardDiag_emacCableDisconTest(portNum);
        }
#endif
    }

#if defined(ICSSG_PORT2PORT_TEST)
    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS;)
    {
    	/* with presence of interposer card, we dont send on ports 0 and 2 */
        if (1 == interposerCardPresent)
        {
            if((portNum == 0) || (portNum == 2))
                 continue;
        }
#if defined(DIAG_STRESS_TEST)
        UART_printf("\n\nSending Packets on Port - %d\n", portNum);
        UART_printf("Receiving Packets on Port - %d\n\n", (portNum + 1));

        for (index = 0; index < PKT_SEND_COUNT; index++)
        {
            pkt_rcvd = 0;
            timeout  = 0;

            BoardDiag_appTestSendPkts(portNum);

            while(pkt_rcvd == 0)
            {
                emac_poll_pkt(portNum + 1);
                /* Wait 1msec */
                BOARD_delay(1000);
                timeout++;
                if(timeout >= BOARD_DIAG_ICSSEMAC_TEST_TIMEOUT)
                {
                    UART_printf("\nReceive Timeout!\n");
                    ret = -1;
                    break;
                }
            }
            /* Check if there a input from console to break the test */
            rdBuf = (char)BoardDiag_getUserInput(BOARD_UART_INSTANCE);
            if((rdBuf == 'b') || (rdBuf == 'B'))
            {
                UART_printf("Received Test Termination... Exiting the Test\n\n");
                UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                            gTxPktCount, pkt_rcv_count);

                if (pkt_rcv_count != gTxPktCount)
                {
                    UART_printf("Port %d Send to Port %d Receive Test Failed!!\n",
                                portNum, (portNum + 1));
                    ret = -1;
                }
                else
                {
                    UART_printf("Port %d Send to Port %d Receive Test Passed!\n",
                                portNum, (portNum + 1));
                    ret = 0;
                }

                goto end_test;
            }
        }
        gTxPktCount = 0;
#else
        UART_printf("\n\nSending Packets on Port - %d\n", portNum);
        BoardDiag_appTestSendPkts(portNum);
        /* Wait to allow packets to come back */
        BOARD_delay(1000);
        UART_printf("\nReceiving Packets on Port - %d\n", (portNum + 1));
        emac_poll_pkt(portNum + 1);
#endif
        if (pkt_rcv_count == PKT_SEND_COUNT)
        {
            UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                        PKT_SEND_COUNT, pkt_rcv_count);
            UART_printf("Port %d Send to Port %d Receive Test Passed!\n",
                        portNum, (portNum + 1));
            ret = 0;
        }
        else
        {
            UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                        PKT_SEND_COUNT, pkt_rcv_count);
            UART_printf("Port %d Send to Port %d Receive Test Failed!!\n",
                        portNum, (portNum + 1));
            ret = -1;
            break;
        }
        pkt_rcv_count = 0;

#if defined(DIAG_STRESS_TEST)
        UART_printf("\n\nSending Packets on Port - %d\n", (portNum + 1));
        UART_printf("Receiving Packets on Port - %d\n", portNum);

        for (index = 0; index < PKT_SEND_COUNT; index++)
        {
            pkt_rcvd = 0;
            timeout  = 0;

            BoardDiag_appTestSendPkts(portNum + 1);

            while(pkt_rcvd == 0)
            {
                emac_poll_pkt(portNum);
                /* Wait 1msec */
                BOARD_delay(1000);
                timeout++;
                if(timeout >= BOARD_DIAG_ICSSEMAC_TEST_TIMEOUT)
                {
                    UART_printf("\nReceive Timeout!\n");
                    ret = -1;
                    break;
                }
            }

            /* Check if there a input from console to break the test */
            rdBuf = (char)BoardDiag_getUserInput(BOARD_UART_INSTANCE);
            if((rdBuf == 'b') || (rdBuf == 'B'))
            {
                UART_printf("Received Test Termination... Exiting the Test\n\n");
                UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                            gTxPktCount, pkt_rcv_count);

                if (pkt_rcv_count != gTxPktCount)
                {
                    UART_printf("Port %d Send to Port %d Receive Test Failed!\n",
                                (portNum + 1), portNum);
                    ret = -1;
                }
                else
                {
                    UART_printf("Port %d Send to Port %d Receive Test Passed!\n",
                                (portNum + 1), portNum);
                    ret = 0;
                }

                goto end_test;
            }
        }

        gTxPktCount = 0;
#else
        UART_printf("\n\nSending Packets on Port - %d\n", (portNum + 1));
        BoardDiag_appTestSendPkts(portNum + 1);
        /* Wait to allow packets to come back */
        BOARD_delay(1000);
        UART_printf("\nReceiving Packets on Port - %d\n", portNum);
        emac_poll_pkt(portNum);
#endif
        if (pkt_rcv_count == PKT_SEND_COUNT)
        {
            UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                        PKT_SEND_COUNT, pkt_rcv_count);
            UART_printf("Port %d Send to Port %d Receive Test Passed!\n",
                        (portNum + 1), portNum);
            ret = 0;
        }
        else
        {
            UART_printf("\nPackets Sent: %d, Packets Received: %d\n",
                        PKT_SEND_COUNT, pkt_rcv_count);
            UART_printf("Port %d Send to Port %d Receive Test Failed!!\n",
                        (portNum + 1), portNum);
            ret = -1;
            break;
        }

        pkt_rcv_count = 0;

        portNum += 2;
    }

#if defined(DIAG_STRESS_TEST)
end_test:
#endif
    for(portNum = startPort; portNum < EMAC_MAX_PORTS_ICSS; portNum++)
    {
        emac_close(portNum);
    }

#if defined(SOC_AM65XX)
    Udma_deinit(&gUdmaDrvObj);
#endif

    if(ret)
    {
        UART_printf("\n\nICSSG Ethernet Port to Port Test Failed!!\n");
    }
    else
    {
        UART_printf("\n\nICSSG Ethernet Port to Port Test Passed!\n");
    }

    UART_printf("All Tests Completed\n");
#endif /* ICSSG_PORT2PORT_TEST */

    return ret;
}

/**
 * \brief  main function
 *
 *  This function performs board initializations and calls emac test
 *
 * \return  int
 *              0  - in case of success
 *             -1  - in case of failure
 *
 */
int main(void)
{
    Board_initCfg boardCfg;

#ifdef PDK_RAW_BOOT
    boardCfg = BOARD_INIT_MODULE_CLOCK |
               BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_UART_STDIO |
               BOARD_INIT_ICSS_ETH_PHY;
#else
    boardCfg = BOARD_INIT_UART_STDIO |
               BOARD_INIT_ICSS_ETH_PHY | BOARD_INIT_PINMUX_CONFIG;
#endif

    Board_init(boardCfg);

    if (BoardDiag_DetectInterposerCard() == TRUE)
    {
        UART_printf("Interposer card is  present\n");
        interposerCardPresent = 1;
        return BoardDiag_IcssgEmacTestInterposer();
    }
    else
    {
        UART_printf("Interposer card is NOT present\n");
        return BoardDiag_IcssgEmacTest();
    }

}
