/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
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

/* ================================================================= */
/*  file  tsiploc.h
 *
 *  Internal module data structures and definitions
 *
 */
#ifndef _TSIPLOC_H
#define _TSIPLOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include "tsip.h"
#include "tsipcsl.h"

#define TSIP_MAX_N_PORTS 2
typedef struct tsipMemBuf_s {
    void*   base;
} tsipMemBuf_t;

/**
 * @brief Specification of Tsip_HandleShared 
 *
 * The Tsip_HandleShared is used to identify TSIP resource context common to all the ports
 */
typedef void*  Tsip_HandleShared;

/**
 * @brief Specification of Tsip_HandlePort 
 *
 * The Tsip_HandlePort is used to identify TSIP port instance for individual ports
 */
typedef void*  Tsip_HandlePort;

/** 
 * @brief 
 *  The structure describes the TSIP Driver Instance
 *
 * @details
 *  The TSIP driver contain one shared instance and may have multiple per port instances each of which is 
 *  correponding to a specific TSIP port 
 */
typedef struct tsipDrvInst_s
{
    tsipMemBuf_t tsipBufsShared[tsip_N_BUFS_SHARED];

    /**
     * @brief   The number of TSIP ports enabled
     */
    int16_t nTsipPorts;

    /**
     * @brief   Array of pointers (size nTsipPorts) to port instances
     */
    Tsip_HandlePort   tsipPort[TSIP_MAX_N_PORTS];

    /**
     * @brief   Shared TSIP resource context
     */    
    Tsip_HandleShared tsipShared;

} tsipDrvInst_t;


/* The tsip registers are all 32 bit values accessed through memory space */
typedef uint32_t tsipReg;

/* 32bit alignment requirements */
#define TSIP_DMA_ALIGN_REQ  2  /* 2 lsbs must be 0 */

/* TSIP Frame header/footer sizing */
#define TSIP_FRAME_HEADER_SIZE    sizeof(uint32_t)
#define TSIP_FRAME_FOOTER_SIZE    sizeof(uint32_t)
#define TSIP_FRAME_MIN_SIZE       TSIP_FRAME_HEADER_SIZE + TSIP_FRAME_FOOTER_SIZE
#define TSIP_FRAME_ROUND_SIZE(x)  (((x)+sizeof(uint32_t)-1) & ~(sizeof(uint32_t)-1))
#define TSIP_FRAME_SIZE_REMAIN(x)  ((x) & (sizeof(uint32_t)-1))
#define TSIP_REGSIZE              4  /* 4 byte aligment required */

/* The number of tulongs required to map the entire 1024 possible timeslots in a TSIP */
#define TSIP_PORT_TSIP_TIMESLOT_MAP_SIZE_TULONG 32


/*****************************************************************************************
 * Definition: The individual timeslot tracking definition. These are stored
 *             as a singly linked list.
 *****************************************************************************************/
typedef enum {
  TSIP_TS_STATE_DISABLED,
  TSIP_TS_STATE_ENABLED,
  TSIP_TS_STATE_PENDING_ENABLE
} tsipState_t;

typedef union {
  void (*tx) (void *, tsipData_t **, tsipData_t **, uint32_t, uint16_t);
  void (*rx) (void *, tsipData_t **, tsipData_t **, uint32_t, uint16_t);
} tsipAppCallout_t;

typedef union {
  /* If this is a to-tdm timeslot, this is a link to the corresponding 
   * from-tdm timeslot, if present 
   */
  struct tsipTsArray_s *totdmToFromtdmLink;
  /* If this is a from-tdm timeslot, this is the buffer containing the 
   * corresponding delay line * samples for ecan.  The offset is index, 
   * which is updated by normal from-tdm operation
   */
  tsipData_t *rinBase;
} tsipRinDesc_t;

typedef struct tsipTsArray_s
{
  uint16_t state_compand;  /* Timeslot stat (bits 15:8) and compand option (bits 7:0) */
  
  uint16_t port_Link_Ts;   /* Bits for Port, Link and timeslot coming from tsip_cfg.h */
  uint16_t dataOffset;     /* data offset to data for this timeslot                       */
  uint8_t  size;           /* Size of application buffer              */
  uint8_t  index;          /* Current application data pointer        */
  
  void *context;        /* Application channel context             */
  tsipData_t *base;      /* Base address of application data buffer */
  
  /* Function to call when app buffer needs service */
  tsipAppCallout_t callout;

  /* Link from tdm out to tdm in, or rin buffer */
  tsipRinDesc_t rinDesc;

  /* Pointer to next list element */
  struct tsipTsArray_s  *next;
} tsipTsArray_t;


/**************************************************************************************
 * Definition: The TSIP port change command stack. The stack is used to change the
 *             memory allocation and timeslot enables/disables.
 *
 *             The command word is 16 bits and has the following format:
 *
 *      /--------------------------------------------------------------------\
 *      |  15   | 14       12   |  11        8  |  7                       0 |
 *      | wait  |     port      |     command   |        parameter           |
 *      \--------------------------------------------------------------------/
 *
 *   Wait: Indicates that in the last interrupt a command was executed. At the next
 *         interrupt the wait is removed. The wait allows the second buffer half
 *         to complete operation before a map change.
 *
 *   Port: The physical port number, indexed from 0.
 *
 *   command/parameter:
 *         0xb - Delay. Parameter is a count, which is decremented each process.
 *               When the count reaches 0 the command is popped off the queue
 *         0xc - resize frame alloc smaller - keep base address the same
 *         0xd - resize frame alloc larger, base address decreases to keep buffer end constant
 *         0xe - shift base down. Parameter is the size of the shift, in units of the 
 *               current frame allocation. The entire shift is done at once
 *         0xf - shift base up. Parameter is the size of the shift, in units of the
 *               current frame allocation. The shift is done one frame allocation size
 *               per interrupt until the entire shift is complete.
 *
 *
 **************************************************************************************/
typedef uint32_t tsipCommand_t;

#define TSIP_COMMAND_DELAY                      0xb
#define TSIP_COMMAND_RESIZE_FRAME_SMALLER       0xc
#define TSIP_COMMAND_RESIZE_FRAME_LARGER        0xd
#define TSIP_COMMAND_SHIFT_BASE_LOWER           0xe
#define TSIP_COMMAND_SHIFT_BASE_HIGHER          0xf

#define TSIP_BIT_MASK(x,y)      (   (   (  (1U << ((x)-(y)+1U) ) - 1U )   )   <<  (y)   )
#define TSIP_READ_BITFIELD(z,x,y)   (((z) & TSIP_BIT_MASK(x,y)) >> (y))
#define TSIP_SET_BITFIELD(z,f,x,y)  ((z) & ~TSIP_BIT_MASK(x,y)) | ( ((f) << (y)) & TSIP_BIT_MASK(x,y) )

#define TSIP_READ_WAIT(x)    TSIP_READ_BITFIELD((x),31,31)
#define TSIP_READ_PORT(x)    TSIP_READ_BITFIELD((x),30,28)
#define TSIP_READ_COMMAND(x) TSIP_READ_BITFIELD((x),27,24)
#define TSIP_READ_PARAM(x)   TSIP_READ_BITFIELD((x),23,0)

#define TSIP_SET_PARAM(v,param)  TSIP_SET_BITFIELD((v),(param),23,0)

#define TSIP_FORM_COMMAND(v,wait,port,command,parameter)                \
                                (v)=TSIP_SET_BITFIELD((v),(wait),31,31);     \
                                (v)=TSIP_SET_BITFIELD((v),(port),30,28);     \
                                (v)=TSIP_SET_BITFIELD((v),(command),27,24);  \
                                (v)=TSIP_SET_BITFIELD((v),(parameter),23,0);
                                
/* The NULL command must have the entire command field 0, not just the command part */                                
#define TSIP_COMMAND_NULL   0                              

enum {
  TSIP_NO_MAP_CHANGE,
  TSIP_ENABLE_MAP_CHANGE
};


/***************************************************************************************
 * Definition: The TSIP change token. Only one of the directions (tx or rx) can 
 *             undergo a map change in one isr interval. This tracks the allowed
 *             direction to change.
 ***************************************************************************************/
#define TSIP_CHANGE_TOKEN_TX          (1 << 1)
#define TSIP_CHANGE_TOKEN_MASK        (1 << 1)
#define TSIP_CHANGE_TOKEN_ARITH_MASK  (~(1 << 2))
#define TSIP_CHANGE_TOKEN_ENABLE      (1 << 3)


/***************************************************************************************
 * Definition: The TSIP driver uses shared memory resources for all TSIP ports.
 *             The DMA memory buffers and the timeslot arrays are all shared.
 *             The pointer to this structure is stored in halContext.
 ***************************************************************************************/
typedef struct tsipDmaBuffer_s {

  uint16_t baseOffset;
  uint16_t frameAlloc; /* NOTE: The actual SIZE used by the buffer is frameCount * frameAlloc */
  
} tsipDmaBuffer_t;

enum {
  TSIP_SHARED_DIR_TX,
  TSIP_SHARED_DIR_RX
};
 
typedef struct tsipSharedDir_s {

  int16_t dirId;            /* Identifies this as tx or rx direction */

  uint32_t memBase;        /* The base of the shared DMA buffer */
  uint16_t  lengthBytes;    /* The length of the DMA buffer      */
  
  tsipDmaBuffer_t   *dmaBuffer;   /* Array of port buffers which segment the DMA mem */
  
  /* The command stack */
  int16_t stackp;                       /* Stack pointer. 0 indicates the stack is empty */
  tsipCommand_t  *commandStack;   /* The max number of elements is 3 plus the number of ports */
  
  /* New changes are added to the head of these next two linked lists,
   * changes are taken from the tail of the linked list. Deactivates
   * take priority over activates. */
  tsipTsArray_t *tsPendingActivate;   /* Timeslots waiting for enable */
  tsipTsArray_t *tsPendingDeactivate; /* Timeslots waiting for disable */
  
} tsipSharedDir_t;
 
 
typedef struct tsipShared_s {

  int16_t  allocated;  /* If 0 then the common memory has not been allocated          */
  uint16_t enableMap;  /* A bit is set to indicate a (physical) port has been started */
  
  tsipTsArray_t *tsArrayFree;       /* Timeslots unallocated to any port */
  
  tsipSharedDir_t sharedTx;
  tsipSharedDir_t sharedRx;
  
} tsipShared_t;

/****************************************************************************************
 * Definition: The TSIP DMA memory requirements. The TSIP buffers 2 * subFrame buffers.
 *             These values are used to feed the TSIP memory context.
 ****************************************************************************************/
typedef struct tsipDmaMem_s
{
  /* Four values comprise the TSIP channel context. 
   * It is critical to synchronize the switch over to occur when the DMA would 
   * have begun at the start of the buffer if no switch were flagged */
   
  tsipReg baseAddressLocal;   /* NOT the global address */
  tsipReg frameAlloc;
  tsipReg frameSize;
  
  /* The fourth element in the channel context is the frame count. This value
   * can never be changed during operation and is stored in each direction 
   * structure */

} tsipDmaMem_t;

/****************************************************************************************
 * Definition: The structure used to track an array of timeslots and the associated
 *             DMA memory.
 ****************************************************************************************/
typedef struct tsipDir_s  {
  
  int16_t channel;                    /* Tsip channel in use                     */
  
  tsipDmaMem_t   dma1;          /* First block of dma info                 */
  tsipDmaMem_t   dma2;          /* Second block of dma info                */
  tsipDmaMem_t  *dmap1;         /* Points to one of the dma blocks         */
  tsipDmaMem_t  *dmap2;         /* Points to one of the dma blocks         */
  
  
  /* frameCount is the fourth element of a TSIP channel context. The
   * other three values are in the halTsipDmaMem_t structures. The frameCount
   * value can never change during operation */
  tsipReg frameCount;
  
  /* The driver creates a double buffered memory for TSIP data. The offset value
   * keeps track of which half to use on entry to an ISR. This value is not
   * actually the word offset, but a 0 or a 1. 0 indicates the buffer base,
   * 1 = buffer mid point */
  int16_t offset;
  
  
  tsipReg        ExptdCnIDFrfc; /* Expected CnID, frfc                     */
  tsipTsArray_t *tsArray;       /* Active timeslots allocated to this port       */

} tsipDir_t;

/***************************************************************************************
 * Definition: Error tracking structure
 ***************************************************************************************/
typedef struct tsipErr_s {

  uint16_t txContextGetFail;   /* hal_tsip_tx_provide_new_context failure */
  uint16_t rxContextGetFail;   /* hal_tsip_rx_provide_new_context failure */
  
  uint16_t rxActiveContextGetFail; /* hal_tsip_update_rx_maps failure */
  
  uint16_t portEnabled;   /* core 0 tried to enable an already enabled port */
  
  uint16_t txChEnabled;  /* local core tried to enable an already enabled tx channel */
  uint16_t rxChEnabled;  /* local core tried to enable an already enabled rx channel */
  
  uint16_t changeMiss;   /* Count of interrupts that were late so no map change could occur */
  
  uint16_t txFrfcError;  /* Count of transmit FRFC check failures */
  uint16_t rxFrfcError;  /* Count of receive FRFC check failures  */
  
  uint16_t txResync;     /* Count of transmit resyncs performed   */
  uint16_t rxResync;     /* Count of receive resyncs performed    */
  
} tsipErr_t;


/*****************************************************************************************
 * Definition: Cross core staggering information
 *****************************************************************************************/
enum  {
  TSIP_STAGGER_STATE_FIND_RX_SYNC,
  TSIP_STAGGER_STATE_FIND_TX_SYNC,
  TSIP_STAGGER_STATE_VERIFY_SYNC
};

#define TSIP_MAP_STATE_CHANGE_TX_CHANGE  (1 << 0)
#define TSIP_MAP_STATE_CHANGE_RX_CHANGE  (1 << 1)
#define TSIP_MAP_STATE_CHANGE_ANY        (TSIP_MAP_STATE_CHANGE_TX_CHANGE | TSIP_MAP_STATE_CHANGE_RX_CHANGE)
#define TSIP_MAP_STATE_FORCE_ALLOW       (1 << 2)

enum  {
  TSIP_STAGGER_STAGGER_START_ISR,
  TSIP_STAGGER_MID_ISR
};

typedef struct tsipStagger_s  {
  int16_t  syncState;
  uint16_t mapChangeState; 
  
  uint32_t txChEn;        /* Transmit channel enable information */
  uint32_t rxChEn;        /* Receive channel enable information  */
  
  int16_t  isrChoice;     /* Run corresponding ISR functions depending on run-time status */

} tsipStagger_t;


/*****************************************************************************************
 * Definition: The structure containing the information required for software management
 *             of the TSIP. A pointer to this structure is provided to the hal level
 *             ISR handler.
 *****************************************************************************************/
typedef struct tsip_s   {

  int16_t port;          /* The physical port number, 0 indexed  */
  int16_t subFrame;      /* Number of frames in a super-frame    */
  int16_t tsPerLink;     /* The number of timeslots in each link */
                       
  /* Phase info */
  int16_t subPhase;      /* Current sub frame phase (sub frame counter) */
  int16_t maxSubPhase;   /* Maximum value for the the sub phase */
  
  /* Rx Remaps. This limits remaps to true map changes, not base relocations */
  unsigned short rxRemap;
  
  /* The change token. Only one side (tx or rx) can change in an interrupt interval.
   * The token is a bit mapped field:
   *
   *  Bits 0 and 2 are used for arithmetic around bit 1. Bit 1 indicates 
   *  which half (tx or rx) can be changed at this interrupt. It cannot
   *  toggle every interrupt because the changes are allowed only when
   *  the second half of the dma buffer is active.
   *
   *  The allow bit is used to determine if any changes are allowed. 
   *  Changes are denied if interrupt processing is delayed.
   *
   *        /-----------------------------------------------------\
   *        | 15     4 |   3    |      2    |  1    |    0        |
   *        |   unused | allow  |  overflow | Tx/Rx |  bit buffer | 
   *        \-----------------------------------------------------/
   */
  int16_t token;
  
  /* To TSIP */
  tsipDir_t tx;
  
  /* From TSIP */
  tsipDir_t rx;
  
  /* Function to call at every subframe interval, along with callout context */
  void (*subFrameCallout) (void *, uint16_t);
  void *cxt;
  
#ifdef TSIP_CORE_STAGGER
  tsipStagger_t stagger;
#endif
  
  
  /* Error tracking */
  tsipErr_t err;

} tsip_t;

/** 
 * @brief 
 *  The structure describes the TSIP Port Instance
 *
 * @details
 *  The TSIP port instance contains 
 *
 */
typedef struct tsipPortInst_s
{
    /**
     * @brief  Internal TSIP port structure
     */
    tsip_t         tsipPort;

    tsipMemBuf_t      tsipBufsPort[tsip_N_BUFS_PORT];
    /**
     * @brief   Pointer to the TSIP driver instance.
     */
    tsipDrvInst_t        *tsipDrvInst;

} tsipPortInst_t;

/*************************************************************************************
 * Definition: A structure used to parameterize timeslot enables now that
 *             asynchronous timeslot enables are allowed.
 *************************************************************************************/
typedef struct tsipTs_s
{
  unsigned short    enable;        /* TRUE for timeslot enable, FALSE for disable */
  int16_t           timeslot;      /* The link/timeslot number                    */
  uint16_t          compand;       /* Hardware companding                         */
  tsipData_t        *base;         /* Application buffer                          */
  uint16_t          appFrameSize;  /* Application frame                           */
  uint16_t          phase;         /* Desired sub Frame phase                     */
  int16_t           subPhase;      /* Current system sub-phase                    */
  int16_t           maxSubPhase;   /* System max sub-phase                        */
  int16_t           subFrame;      /* Number of samples in a super-frame          */
  
  /* Application call out and context */
  tsipAppCallout_t callout;
  void *context;                            

  /* Rx to Tx link or rxin buffer */
  tsipRinDesc_t rinDesc;
} tsipTs_t;  

/**************************************************************************************
 * Macros for accessing fields in the tsip structures 
 **************************************************************************************/
#define TSIP_GET_CNID(x)           TSIP_READ_BITFIELD((x),31,24)
#define TSIP_SET_CNID(x,y)         TSIP_SET_BITFIELD((x),(y),31,24)
#define TSIP_GET_FRFC(x)           TSIP_READ_BITFIELD((x),23,0)
#define TSIP_SET_FRFC(x,y)         TSIP_SET_BITFIELD((x),(y),23,0)
#define TSIP_FRFC_WRAP(x)		   (x) = ((x) & TSIP_BIT_MASK(23,0))

#define TSIP_GET_TSA_STATE(x)      TSIP_READ_BITFIELD(((x)->state_compand),15,8)
#define TSIP_GET_TSA_COMPAND(x)    TSIP_READ_BITFIELD(((x)->state_compand),7,0)
#define TSIP_SET_TSA_STATE(x,y)    (x)->state_compand = TSIP_SET_BITFIELD(((x)->state_compand),(y),15,8)
#define TSIP_SET_TSA_COMPAND(x,y)  (x)->state_compand = TSIP_SET_BITFIELD(((x)->state_compand),(y), 7,0)

#define TSIP_GET_TSA_PORT(x)       TSIP_READ_BITFIELD(((x)->port_Link_Ts), \
                                                     TSIP_PORT_MSB, TSIP_PORT_LSB)
#define TSIP_GET_TSA_LINK(x)       TSIP_READ_BITFIELD(((x)->port_Link_Ts), \
                                                     TSIP_LINK_MSB, TSIP_LINK_LSB)
#define TSIP_GET_TSA_TS(x)         TSIP_READ_BITFIELD(((x)->port_Link_Ts), \
                                                     TSIP_TS_MSB, TSIP_TS_LSB)
                                                        


/******************************************************************************
 * Definition: Local symbolic return values
 ******************************************************************************/
#define TSIP_SUCCESS    0
#define TSIP_FRFC_FAIL  1


/******************************************************************************
 * Definition: Initial CNID values. These are 8 bit values, with the lsb
 *             reserved to indicate which map (a or b) is in use. The MSB 
 *             is restricted by the TSIP so that 0 indicates the tx (to tsip)
 *             and 1 for and rx (from tsip).
 ******************************************************************************/
#define TSIP_INITIAL_TX_CNID    0x00U
#define TSIP_INITIAL_RX_CNID    0x80U
#define TSIP_CNID_INC_MASK      0xfeU

/*****************************************************************************
 * Definition: Updated map timeslot state control. Tx timeslots are enabled
 *             as soon as the context is provided. Rx are not enabled until
 *             the new cnid is received.
 *****************************************************************************/
#define TSIP_TS_REMAIN_PENDING      0
#define TSIP_TS_ENABLE_PENDING      1

/*****************************************************************************
 * Definition: Symbolic definitions to indicate if a new context will require
 *             a new base address.
 *****************************************************************************/
#define TSIP_SAME_CONTEXT_BASE      0
#define TSIP_NEW_CONTEXT_BASE       1


/*
 * Define TRUE and FALSE
 */
#undef TRUE
#undef FALSE
#define TRUE     (unsigned short)1
#define FALSE    (unsigned short)0


/******************************************************************************
 * Prototypes
 ******************************************************************************/
int16_t tsip_tx_provide_new_context (tsipShared_t *sTsip, tsip_t *tsip, unsigned short growDown, uint16_t baseRef, int16_t mapChangeEnable);
int16_t tsip_rx_provide_new_context (tsipShared_t *sTsip, tsip_t *tsip, unsigned short growDown, uint16_t baseRef, int16_t mapChangeEnable);
void tsip_update_rx_maps (tsipShared_t *sTsip, tsip_t *tsip);
void tsip_resync_rx (tsip_t *tsip);
void tsip_resync_tx (tsip_t *tsip);
tsip_t *tsipFromPort (tsipDrvInst_t *tsipDrvInst, int16_t port);
void tsipLoadDmaEnc (tsip_t *tsip, cslTsipChEnable_t *entx, cslTsipChEnable_t *enrx);
void tsipStartDma (tsipShared_t *sdir, tsip_t *tsip);
tsipCommand_t tsip_process_command_dis_port (tsipCommand_t command, int16_t port, tsipSharedDir_t *sdir);
tsipCommand_t tsip_process_command (tsipShared_t *sTsip, tsipCommand_t command, tsip_t *tsip, tsipSharedDir_t *sdir);
tsipReturn_t tsip_com_seq_activate (tsipDrvInst_t *tsipDrvInst, tsipSharedDir_t *sdir);
tsipReturn_t tsip_com_seq_deactivate (tsipDrvInst_t *tsipDrvInst, tsipSharedDir_t *sdir);
void tsip_ts_chain_move (tsipTsArray_t **source, tsipTsArray_t **dest, int16_t port, uint16_t state);

void tsipIsr (void *vtsip);
void tsipStaggerStartIsr (void *vtsip);

#ifdef __cplusplus
}
#endif

#endif  /* _TSIPLOC_H */




