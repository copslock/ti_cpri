#ifndef IQN2_CONFIG_H_
#define IQN2_CONFIG_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <ti/csl/csl_chip.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/tistdtypes.h>

#include <ti/drv/iqn2/iqn2fl.h>

//AT event interrupt mapping for KeystoneII
#define AIF2_EVENT0_INTSEL_MAP 87
#define AIF2_EVENT1_INTSEL_MAP 88
#define AIF2_EVENT2_INTSEL_MAP 89
#define AIF2_EVENT3_INTSEL_MAP 90
#define AIF2_EVENT4_INTSEL_MAP 91
#define AIF2_EVENT5_INTSEL_MAP 92
#define AIF2_EVENT6_INTSEL_MAP 93
#define AIF2_EVENT7_INTSEL_MAP 94


#define IQN2_BASE_TX_QUE_NUM      832  //832 ~ 879 (total 48 channels)

// 3GPP constants  
#define FRAME_COUNT_TC_LTE_FDD  4095
#define SYMBOL_COUNT_TC_LTE_FDD  9
#define CLOCK_COUNT_TC_LTE_FDD   307199

#define FRAME_COUNT_TC_WCDMA_FDD   4095
#define SLOT_COUNT_TC_WCDMA_FDD    14
#define CLOCK_COUNT_TC_WCDMA_FDD   204799

#define FRAME_COUNT_TC_PHY_TIMER  4095
#define CLOCK_COUNT_TC_PHY_TIMER  3071999

// 3GPP clock count TC for CPRI (2x, 4x, 8x)
#define CLOCK_COUNT_TC_LTE_FDD_CPRI     245759
#define CLOCK_COUNT_TC_WCDMA_FDD_CPRI   163839
#define CLOCK_COUNT_TC_PHY_TIMER_CPRI  2457599


// IQN setup constants
#define RM_LOS_DET_THOLD                  0xFFFF
#define RM_SYNC_THOLD                         1
#define RM_UNSYNC_THOLD                    5

#define OBSAI_TYPE_WCDMA_FDD          0x2
#define OBSAI_TYPE_LTE                0xE
#define OBSAI_TYPE_GSM                0x4
#define OBSAI_TYPE_GENERIC            0xF
#define OBSAI_TYPE_CONTROL            0x0

#define CRC8_POLY                     0xF
#define CRC8_SEED                     0xF
#define DB_PE_DELAY_OBSAI                 28
#define DB_PE_DELAY_CPRI                   0
#define AXC_OFFSET_WIN             300// +/- 4  WCDMA chip


/***************** Nysh common config for cache operation **********************/
#define L1_CACHE_0K                0
#define L1_CACHE_4K                1
#define L1_CACHE_8K                2
#define L1_CACHE_16K               3
#define L1_CACHE_MAX               7 /* 32K */

#define L2_CACHE_0K                0
#define L2_CACHE_32K               1
#define L2_CACHE_64K               2
#define L2_CACHE_128K              3
#define L2_CACHE_MAX               7 /* 256K */

/* MPU2 config to unprotect QMSS queue proxy */
#define MPU2_BASE_ADDRESS          0x02370000
#define MPU2_PROG1_MPPA_ADDR      (MPU2_BASE_ADDRESS + 0x0208)
#define MPU2_PROG1_MPPA_RESET_VAL  0x0003FCB6
#define MPU2_PROG1_MPPA_GRANT_ALL  0x03FFFCB6
#define MPU2_PROG1_MPPA_GRANT_QM   0x0013FCB6

/******************** PktDMA QMSS config****************************************/
#define QMSS_CFG_BASE                            (0x02a00000)  /* VBUSP */
#define QMSS_DATA_BASE                           (0x23a00000)  /* VBUSM */
#define CSL_QM_SS_LINKING_RAM                    (QMSS_CFG_BASE + 0x00100000)
#define CSL_QM_SS_CFG_QM1_CONFIG_REGS            (QMSS_CFG_BASE + 0x02000)
#define CSL_QM_SS_CFG_QM1_DESCRIPTOR_REGS        (QMSS_CFG_BASE + 0x03000)
#define CSL_QM_SS_CFG_QM1_STATUS_RAM             (QMSS_CFG_BASE + 0x06000)
#define CSL_QM_SS_CFG_QM1_QUE_PEEK_REGS          (QMSS_CFG_BASE + 0x40000)
#define CSL_QM_SS_CFG_QM1_QUEUE_MANAGEMENT_REGS  (QMSS_CFG_BASE + 0x80000)
#define CSL_QM_SS_CFG_QM1_QUEUE_PROXY_REGS       (QMSS_CFG_BASE + 0xc0000)

#define CSL_QM_SS_DATA_QM1_QUEUE_MANAGEMENT_REGS (QMSS_DATA_BASE + 0x80000)
#define CSL_QM_SS_DATA_QM1_QUEUE_PROXY_REGS      (QMSS_DATA_BASE + 0xc0000)


#define CSL_QM_SS_INTD_REG_OFFSET    (0x1000)
#define CSL_QM_SS_CFG_INTD1_REGS     (QMSS_CFG_BASE + 0x0c000)
#define CSL_QM_SS_CFG_INTD2_REGS     (CSL_QM_SS_CFG_INTD1_REGS + CSL_QM_SS_INTD_REG_OFFSET)
#define CSL_QM_SS_PDSP_REG_OFFSET    (0x0100)
#define CSL_QM_SS_CFG_PDSP1_REGS     (QMSS_CFG_BASE + 0x0f000)
#define CSL_QM_SS_CFG_PDSP2_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET))
#define CSL_QM_SS_CFG_PDSP3_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET*2))
#define CSL_QM_SS_CFG_PDSP4_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET*3))
#define CSL_QM_SS_CFG_PDSP5_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET*4))
#define CSL_QM_SS_CFG_PDSP6_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET*5))
#define CSL_QM_SS_CFG_PDSP7_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET*6))
#define CSL_QM_SS_CFG_PDSP8_REGS     (CSL_QM_SS_CFG_PDSP1_REGS + (CSL_QM_SS_PDSP_REG_OFFSET*7))
#define CSL_QM_SS_PDSP_IRAM_OFFSET   (0x1000)
#define CSL_QM_SS_CFG_PDSP1_IRAM     (QMSS_CFG_BASE + 0x10000)
#define CSL_QM_SS_CFG_PDSP2_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET))
#define CSL_QM_SS_CFG_PDSP3_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*2))
#define CSL_QM_SS_CFG_PDSP4_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*3))
#define CSL_QM_SS_CFG_PDSP5_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*4))
#define CSL_QM_SS_CFG_PDSP6_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*5))
#define CSL_QM_SS_CFG_PDSP7_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*6))
#define CSL_QM_SS_CFG_PDSP8_IRAM     (CSL_QM_SS_CFG_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*7))
#define CSL_QM_SS_DATA_PDSP1_IRAM    (QMSS_DATA_BASE + 0x10000)
#define CSL_QM_SS_DATA_PDSP2_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET))
#define CSL_QM_SS_DATA_PDSP3_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*2))
#define CSL_QM_SS_DATA_PDSP4_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*3))
#define CSL_QM_SS_DATA_PDSP5_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*4))
#define CSL_QM_SS_DATA_PDSP6_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*5))
#define CSL_QM_SS_DATA_PDSP7_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*6))
#define CSL_QM_SS_DATA_PDSP8_IRAM    (CSL_QM_SS_DATA_PDSP1_IRAM + (CSL_QM_SS_PDSP_IRAM_OFFSET*7))
#define CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET (0x4000)
#define CSL_QM_SS_CFG_SCRATCH_RAM1   (QMSS_CFG_BASE + 0x20000)
#define CSL_QM_SS_CFG_SCRATCH_RAM2   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET))
#define CSL_QM_SS_CFG_SCRATCH_RAM3   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET*2))
#define CSL_QM_SS_CFG_SCRATCH_RAM4   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET*3))
#define CSL_QM_SS_CFG_SCRATCH_RAM5   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET*4))
#define CSL_QM_SS_CFG_SCRATCH_RAM6   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET*5))
#define CSL_QM_SS_CFG_SCRATCH_RAM7   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET*6))
#define CSL_QM_SS_CFG_SCRATCH_RAM8   (CSL_QM_SS_CFG_SCRATCH_RAM1 + (CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET*7))

#define CSL_QM_SS_PKT_DMA1                           (QMSS_CFG_BASE + 0x8000)
#define CSL_QM_SS_CFG_PKT_DMA1_GLOBAL_CFG_REGS       (CSL_QM_SS_PKT_DMA1)
#define CSL_QM_SS_CFG_PKT_DMA1_TX_CFG_REGS           (CSL_QM_SS_PKT_DMA1 + 0x0400)
#define CSL_QM_SS_CFG_PKT_DMA1_RX_CFG_REGS           (CSL_QM_SS_PKT_DMA1 + 0x0800)
#define CSL_QM_SS_CFG_PKT_DMA1_TX_SCHEDULER_CFG_REGS (CSL_QM_SS_PKT_DMA1 + 0x0C00)
#define CSL_QM_SS_CFG_PKT_DMA1_RX_FLOW_CFG_REGS      (CSL_QM_SS_PKT_DMA1 + 0x1000)

/* Define QMSS Register block regions. */
#define QM1_CFG_REGION                  CSL_QM_SS_CFG_QM1_CONFIG_REGS
#define QM1_DESC_REGION                 CSL_QM_SS_CFG_QM1_DESCRIPTOR_REGS
#define QM1_QMAN_REGION                 CSL_QM_SS_CFG_QM1_QUEUE_MANAGEMENT_REGS
#define QM1_QMAN_VBUSM_REGION           CSL_QM_SS_DATA_QM1_QUEUE_MANAGEMENT_REGS
#define QM1_PEEK_REGION                 CSL_QM_SS_CFG_QM1_QUE_PEEK_REGS
#define QM1_PROXY_REGION                CSL_QM_SS_CFG_QM1_QUEUE_PROXY_REGS
#define QM1_STATUS_REGION               CSL_QM_SS_CFG_QM1_STATUS_RAM

#define QMSS_LRAM_REGION                (CSL_QM_SS_LINKING_RAM - QMSS_CFG_BASE)
#define QMSS_INTD1_REGION               CSL_QM_SS_CFG_INTD1_REGS
#define INTD_REG_OFFSET                 CSL_QM_SS_INTD_REG_OFFSET

#define PDSP1_CMD_REGION                CSL_QM_SS_CFG_SCRATCH_RAM1
#define PDSP1_REG_REGION                CSL_QM_SS_CFG_PDSP1_REGS
#define PDSP1_IRAM_REGION               CSL_QM_SS_CFG_PDSP1_IRAM
#define PDSP_CMD_OFFSET                 CSL_QM_SS_PDSP_SCRATCH_RAM_OFFSET
#define PDSP_REG_OFFSET                 CSL_QM_SS_PDSP_REG_OFFSET
#define PDSP_IRAM_OFFSET                CSL_QM_SS_PDSP_IRAM_OFFSET


/* Define IQN2 PKTDMA Register block regions. */
#define DFE_IQN2_CFG_BASE                   CSL_IQN_CFG_REGS
#define DFE_IQN2_PKTDMA_GBL_CFG_REGION      (DFE_IQN2_CFG_BASE + 0x104000)
#define DFE_IQN2_PKTDMA_TX_CHAN_REGION      (DFE_IQN2_CFG_BASE + 0x106000)
#define DFE_IQN2_PKTDMA_RX_CHAN_REGION      (DFE_IQN2_CFG_BASE + 0x108000)
#define DFE_IQN2_PKTDMA_TX_SCHD_REGION      (DFE_IQN2_CFG_BASE + 0x10A000)
#define DFE_IQN2_PKTDMA_RX_FLOW_REGION      (DFE_IQN2_CFG_BASE + 0x10C000)

/**********************************************************************
 * Define offsets to individual QM registers within an address region. */

/* Descriptor Memory Region */
/* Queue Manager Region */
#define QM_REG_QUE_REVISION       0x000
#define QM_REG_QUE_DIVERSION      0x008
#define QM_REG_LINKRAM_0_BASE     0x00c
#define QM_REG_LINKRAM_0_SIZE     0x010
#define QM_REG_LINKRAM_1_BASE     0x014
#define QM_REG_STARVATION_CNT     0x020

/* Descriptor Memory Region */
#define QM_REG_MEM_REGION_BASE    0x000
#define QM_REG_MEM_REGION_INDEX   0x004
#define QM_REG_MEM_REGION_SETUP   0x008

/* Queue Management Region */
#define QM_REG_QUE_REG_A          0x000
#define QM_REG_QUE_REG_B          0x004
#define QM_REG_QUE_REG_C          0x008
#define QM_REG_QUE_REG_D          0x00c

/* Queue Status Region */
#define QM_REG_QUE_STATUS_REG_A   0x000
#define QM_REG_QUE_STATUS_REG_B   0x004
#define QM_REG_QUE_STATUS_REG_C   0x008
#define QM_REG_QUE_STATUS_REG_D   0x00c

/* Interrupt Distributor (INTD) Region */
#define QM_REG_INTD_REVISION      0x000
#define QM_REG_INTD_EOI           0x010
#define QM_REG_INTD_STATUS        0x200
#define QM_REG_INTD_STATUS_CLEAR  0x280
#define QM_REG_INTD_INT_COUNT     0x300

/* PDSP(n) Register Region */
#define QM_REG_PDSP_CONTROL       0x000
#define QM_REG_PDSP_STATUS        0x004
#define QM_REG_PDSP_WAKEUP_ENABLE 0x008
#define QM_REG_PDSP_CYCLE_COUNT   0x00c
#define QM_REG_PDSP_STALL_COUNT   0x010
#define QM_REG_PDSP_CTBL_BLK_IDX0 0x020
#define QM_REG_PDSP_CTBL_BLK_IDX1 0x024
#define QM_REG_PDSP_CTBL_PRG_PTR0 0x028
#define QM_REG_PDSP_CTBL_PRG_PTR1 0x02c

/**********************************************************************
 * Define offsets to individual PKTDMA registers within an address region.
 */
/* Global Cfg Register Block */
#define PKTDMA_REG_REVISION         0x000
#define PKTDMA_REG_PERFORM_CTRL     0x004
#define PKTDMA_REG_EMULATION_CTRL   0x008
#define PKTDMA_REG_PRIORITY_CTRL    0x00c
#define PKTDMA_REG_QM0_BASE_ADDR    0x010
#define PKTDMA_REG_QM1_BASE_ADDR    0x014
#define PKTDMA_REG_QM2_BASE_ADDR    0x018
#define PKTDMA_REG_QM3_BASE_ADDR    0x01c

/* Tx Chan Cfg Register Block */
#define PKTDMA_REG_TX_CHAN_CFG_A    0x000
#define PKTDMA_REG_TX_CHAN_CFG_B    0x004

/* Rx Chan Cfg Register Block */
#define PKTDMA_REG_RX_CHAN_CFG_A    0x000

/* Rx Flow Cfg Register Block */
#define PKTDMA_REG_RX_FLOW_CFG_A    0x000
#define PKTDMA_REG_RX_FLOW_CFG_B    0x004
#define PKTDMA_REG_RX_FLOW_CFG_C    0x008
#define PKTDMA_REG_RX_FLOW_CFG_D    0x00c
#define PKTDMA_REG_RX_FLOW_CFG_E    0x010
#define PKTDMA_REG_RX_FLOW_CFG_F    0x014
#define PKTDMA_REG_RX_FLOW_CFG_G    0x018
#define PKTDMA_REG_RX_FLOW_CFG_H    0x01c

/* Tx Sched Cfg Register Block */
#define PKTDMA_REG_TX_SCHED_CHAN_CFG 0x000

/****** M Navigator application functions ***********************/
void qm_set_memory_region(uint16_t qm, uint16_t regn, uint32_t addr, uint32_t indx, uint32_t setup);
void qm_set_link_ram(uint16_t qm, uint16_t ram, uint32_t addr, uint32_t count);
void qm_push_queue(uint16_t qn, uint8_t mode, uint32_t c_val, uint32_t d_val);
uint32_t qm_pop_queue(uint16_t qn);
void qm_divert_queue(uint16_t qm, uint16_t src_qn, uint16_t dest_qn, uint8_t headtail);
void qm_empty_queue(uint16_t qn, uint32_t *list, uint32_t *listCount);
uint32_t qm_get_byte_count(uint16_t qn);
uint32_t qm_get_descriptor_count(uint16_t qn);
void qm_set_queue_threshold(uint16_t qn, uint32_t value);
uint8_t qm_get_queue_status(uint16_t qn);
void pktdma_config_qm(uint32_t base, uint16_t *physical_qmgr, uint16_t *physical_qnum);
void pktdma_config_loopback(uint32_t base, uint8_t enable);
void pktdma_config_retry_timeout(uint32_t base, uint16_t timeout);
void pktdma_config_tx_chan(uint32_t base, uint16_t chan, uint32_t reg_b);
void pktdma_config_tx_sched(uint32_t base, uint16_t chan, uint32_t priority);
void pktdma_config_rx_flow(uint32_t base, uint16_t flow,
                           uint32_t a, uint32_t b, uint32_t c, uint32_t d,
                           uint32_t e, uint32_t f, uint32_t g, uint32_t h);
void pktdma_enable_rx_chan(uint32_t base, uint16_t chan, uint32_t value);
void pktdma_enable_tx_chan(uint32_t base, uint16_t chan, uint32_t value);
uint32_t intd_read_status(uint16_t intd, uint16_t group, uint32_t chan);
void intd_set_status(uint16_t intd, uint16_t group, uint32_t chan);
void intd_clear_status(uint16_t intd, uint16_t group, uint32_t chan);
uint32_t intd_read_intcount(uint16_t intd, uint16_t intnum);
void intd_write_intcount(uint16_t intd, uint16_t intnum, uint32_t val);
void intd_write_eoi(uint16_t intd, uint32_t val);
void pdsp_control(uint16_t pdsp, uint32_t val);
void pdsp_enable(uint16_t pdsp);
void pdsp_disable(uint16_t pdsp);
uint8_t pdsp_running(uint16_t pdsp);
void pdsp_download_firmware(uint16_t pdsp, uint8_t *code, uint32_t size);


/* Nyquist Common and Cache application functions  */
uint32_t l2_global_address(uint32_t coreId, uint32_t addr);
uint32_t get_core_id(void);
void config_l1p_cache(uint16_t cache_size);
void config_l1d_cache(uint16_t cache_size);
void config_l2_cache(uint16_t cache_size);
void l1d_cache_wb_inv(uint8_t writeback, uint8_t invalidate);
void l2_cache_wb_inv(uint8_t writeback, uint8_t invalidate);
void config_cache_region(uint32_t address);
void qproxy_access(void);



/* Define descriptor constants  */
#define MNAV_DESC_TYPE_HOST      0
#define MNAV_DESC_TYPE_MONO      2
#define MNAV_DESC_TYPE_DEFAULT   3 /* for stream use, not flow config */

/*************** Define Accumulator Command constants. ************************/
/* Low Priority Accumulation Constants */
#define QMSS_ACC_LOPRI_CHANNELS    16
#define QMSS_ACC_LOPRI_1WORD_ENTRY 120
#define QMSS_ACC_LOPRI_2WORD_ENTRY 60
#define QMSS_ACC_LOPRI_4WORD_ENTRY 30

/* High Priority Accumulation Constants */
#define QMSS_ACC_HIPRI_CHANNELS    32
#define QMSS_ACC_HIPRI_1WORD_ENTRY 88
#define QMSS_ACC_HIPRI_2WORD_ENTRY 44
#define QMSS_ACC_HIPRI_4WORD_ENTRY 22

/* Define Accumulator Command constants. */
#define QMSS_ACC_CMD_TIMER         0x82
#define QMSS_ACC_CMD_ENABLE        0x81
#define QMSS_ACC_CMD_DISABLE       0x80
#define QMSS_ACC_CMD_RETURN_IDLE   0
#define QMSS_ACC_CMD_RETURN_OKAY   1
#define QMSS_ACC_CMD_RETURN_BAD_CMD 2
#define QMSS_ACC_CMD_RETURN_BAD_CHAN 3
#define QMSS_ACC_CMD_RETURN_CHAN_NOT_ACTIVE 4
#define QMSS_ACC_CMD_RETURN_CHAN_ACTIVE 5
#define QMSS_ACC_CMD_RETURN_BAD_QUEUE 6

/*******************************************************************/
/* Define the bit and word layouts for the Host Packet Descriptor. */
/* For a Host Packet, this is used for the first descriptor only.  */
/*******************************************************************/
#ifdef _BIG_ENDIAN
typedef struct
{
  /* word 0 */
  uint32_t type_id         : 2;  //always 0x0 (Host Packet ID)
  uint32_t packet_type     : 5;
  uint32_t reserved_w0     : 2;
  uint32_t ps_reg_loc      : 1;  //0=PS words in desc, 1=PS words in SOP buff
  uint32_t packet_length   : 22; //in bytes (4M - 1 max)

  /* word 1 */
  uint32_t src_tag_hi      : 8;
  uint32_t src_tag_lo      : 8;
  uint32_t dest_tag_hi     : 8;
  uint32_t dest_tag_lo     : 8;

  /* word 2 */
  uint32_t epib            : 1;  //1=extended packet info block is present
  uint32_t reserved_w2     : 1;
  uint32_t psv_word_count  : 6;  //number of 32-bit PS data words
  uint32_t err_flags       : 4;
  uint32_t ps_flags        : 4;
  uint32_t return_policy   : 1;  //0=linked packet goes to pkt_return_qnum,
                                 //1=each descriptor goes to pkt_return_qnum
  uint32_t ret_push_policy : 1;  //0=return to queue tail, 1=queue head
  uint32_t pkt_return_qmgr : 2;
  uint32_t pkt_return_qnum : 12;

  /* word 3 */
  uint32_t reserved_w3     : 10;
  uint32_t buffer_len      : 22;

  /* word 4 */
  uint32_t buffer_ptr;

  /* word 5 */
  uint32_t next_desc_ptr;

  /* word 6 */
  uint32_t orig_buff0_pool : 4;
  uint32_t orig_buff0_refc : 6;
  uint32_t orig_buff0_len  : 22;

  /* word 7 */
  uint32_t orig_buff0_ptr;

} MNAV_HostPacketDescriptor;
#else
typedef struct
{
  /* word 0 */
  uint32_t packet_length   : 22; //in bytes (4M - 1 max)
  uint32_t ps_reg_loc      : 1;  //0=PS words in desc, 1=PS words in SOP buff
  uint32_t reserved_w0     : 2;
  uint32_t packet_type     : 5;
  uint32_t type_id         : 2;  //always 0x0 (Host Packet ID)

  /* word 1 */
  uint32_t dest_tag_lo     : 8;
  uint32_t dest_tag_hi     : 8;
  uint32_t src_tag_lo      : 8;
  uint32_t src_tag_hi      : 8;

  /* word 2 */
  uint32_t pkt_return_qnum : 12;
  uint32_t pkt_return_qmgr : 2;
  uint32_t ret_push_policy : 1;  //0=return to queue tail, 1=queue head
  uint32_t return_policy   : 1;  //0=linked packet goes to pkt_return_qnum,
                                 //1=each descriptor goes to pkt_return_qnum
  uint32_t ps_flags        : 4;
  uint32_t err_flags       : 4;
  uint32_t psv_word_count  : 6;  //number of 32-bit PS data words
  uint32_t reserved_w2     : 1;
  uint32_t epib            : 1;  //1=extended packet info block is present


  /* word 3 */
  uint32_t buffer_len      : 22;
  uint32_t reserved_w3     : 10;

  /* word 4 */
  uint32_t buffer_ptr;

  /* word 5 */
  uint32_t next_desc_ptr;

  /* word 6 */
  uint32_t orig_buff0_len  : 22;
  uint32_t orig_buff0_refc : 6;
  uint32_t orig_buff0_pool : 4;

  /* word 7 */
  uint32_t orig_buff0_ptr;

} MNAV_HostPacketDescriptor;
#endif

#define MNAV_HOST_PACKET_SIZE  sizeof(MNAV_HostPacketDescriptor)


/*******************************************************************/
/* Define the bit and word layouts for the Host Buffer Descriptor. */
/* For a Host Packet, this will used for secondary descriptors.    */
/*******************************************************************/
#ifdef _BIG_ENDIAN
typedef struct
{
  /* word 0 */
  uint32_t reserved_w0;
  /* word 1 */
  uint32_t reserved_w1;

  /* word 2 */
  uint32_t reserved_w2     : 17;
  uint32_t ret_push_policy : 1;  //0=return to queue tail, 1=queue head
  uint32_t pkt_return_qmgr : 2;
  uint32_t pkt_return_qnum : 12;

  /* word 3 */
  uint32_t reserved_w3     : 10;
  uint32_t buffer_len      : 22;

  /* word 4 */
  uint32_t buffer_ptr;

  /* word 5 */
  uint32_t next_desc_ptr;

  /* word 6 */
  uint32_t orig_buff0_pool : 4;
  uint32_t orig_buff0_refc : 6;
  uint32_t orig_buff0_len  : 22;

  /* word 7 */
  uint32_t orig_buff0_ptr;

} MNAV_HostBufferDescriptor;
#else
typedef struct
{
  /* word 0 */
  uint32_t reserved_w0;
  /* word 1 */
  uint32_t reserved_w1;

  /* word 2 */
  uint32_t pkt_return_qnum : 12;
  uint32_t pkt_return_qmgr : 2;
  uint32_t ret_push_policy : 1;  //0=return to queue tail, 1=queue head
  uint32_t reserved_w2     : 17;

  /* word 3 */
  uint32_t buffer_len      : 22;
  uint32_t reserved_w3     : 10;

  /* word 4 */
  uint32_t buffer_ptr;

  /* word 5 */
  uint32_t next_desc_ptr;

  /* word 6 */
  uint32_t orig_buff0_len  : 22;
  uint32_t orig_buff0_refc : 6;
  uint32_t orig_buff0_pool : 4;

  /* word 7 */
  uint32_t orig_buff0_ptr;

} MNAV_HostBufferDescriptor;
#endif

// Host Buffer packet size is always the same as Host Packet size


/*********************************************************************/
/* Define the bit and word layouts for the Monolithic Pkt Descriptor.*/
/*********************************************************************/
#ifdef _BIG_ENDIAN
typedef struct
{
  /* word 0 */
  uint32_t type_id         : 2;  //always 0x2 (Monolithic Packet ID)
  uint32_t packet_type     : 5;
  uint32_t data_offset     : 9;
  uint32_t packet_length   : 16; //in bytes (65535 max)

  /* word 1 */
  uint32_t src_tag_hi      : 8;
  uint32_t src_tag_lo      : 8;
  uint32_t dest_tag_hi     : 8;
  uint32_t dest_tag_lo     : 8;

  /* word 2 */
  uint32_t epib            : 1;  //1=extended packet info block is present
  uint32_t reserved_w2     : 1;
  uint32_t psv_word_count  : 6;  //number of 32-bit PS data words
  uint32_t err_flags       : 4;
  uint32_t ps_flags        : 4;
  uint32_t reserved_w2b    : 1;
  uint32_t ret_push_policy : 1;  //0=return to queue tail, 1=queue head
  uint32_t pkt_return_qmgr : 2;
  uint32_t pkt_return_qnum : 12;

} MNAV_MonolithicPacketDescriptor;
#else
typedef struct
{
  /* word 0 */
  uint32_t packet_length   : 16; //in bytes (65535 max)
  uint32_t data_offset     : 9;
  uint32_t packet_type     : 5;
  uint32_t type_id         : 2;  //always 0x2 (Monolithic Packet ID)

  /* word 1 */
  uint32_t dest_tag_lo     : 8;
  uint32_t dest_tag_hi     : 8;
  uint32_t src_tag_lo      : 8;
  uint32_t src_tag_hi      : 8;

  /* word 2 */
  uint32_t pkt_return_qnum : 12;
  uint32_t pkt_return_qmgr : 2;
  uint32_t ret_push_policy : 1;  //0=return to queue tail, 1=queue head
  uint32_t reserved_w2b    : 1;
  uint32_t ps_flags        : 4;
  uint32_t err_flags       : 4;
  uint32_t psv_word_count  : 6;  //number of 32-bit PS data words
  uint32_t reserved_w2     : 1;
  uint32_t epib            : 1;  //1=extended packet info block is present

} MNAV_MonolithicPacketDescriptor;
#endif

#define MNAV_MONO_PACKET_SIZE  sizeof(MNAV_MonolithicPacketDescriptor)


/*********************************************************************/
/* Define the word layout of the Extended Packet Info Block.  It     */
/* is optional and may follow Host Packet and Monolithic descriptors.*/
/* For Monolithic descriptors, there is an extra NULL word. This null*/
/* is not copied or passed through the Streaming I/F.                */
/*********************************************************************/
typedef struct
{
  /* word 0 */
  uint32_t timestamp;

  /* word 1 */
  uint32_t sw_info0;

  /* word 2 */
  uint32_t sw_info1;

  /* word 3 */
  uint32_t sw_info2;

} MNAV_ExtendedPacketInfoBlock;

/****************************************************************************/
/************* Define the Accumulator Command Interface Structure ******************/
/****************************************************************************/
#ifdef _BIG_ENDIAN
typedef struct
{
  uint32_t  retrn_code:8;  //0=idle, 1=success, 2-6=error
  uint32_t  un1:8;
  uint32_t  command:8;     //0x80=disable, 0x81=enable, 0=firmware response
  uint32_t  channel:8;     //0 to 47 or 0 to 15
  uint32_t  queue_mask;    //(multi-mode only) bit 0=qm_index queue
  uint32_t  list_address;  //address of Host ping-pong buffer
  uint32_t  max_entries:16;//max entries per list
  uint32_t  qm_index:16;   //qnum to monitor (multiple of 32 for multimode)
  uint32_t  un2:8;
  uint32_t  cfg_un:2;
  uint32_t  cfg_multi_q:1; //0=single queue mode, 1=multi queue mode
  uint32_t  cfg_list_mode:1;//0=NULL terminate, 1=entry count mode
  uint32_t  cfg_list_size:2;//0="D" Reg, 1="C+D" regs, 2="A+B+C+D"
  uint32_t  cfg_int_delay:2;//0=none, 1=last int, 2=1st new, 3=last new
  uint32_t  timer_count:16;//number of 25us timer ticks to delay int
} Qmss_AccCmd;
#else
typedef struct
{
  uint32_t  channel:8;     //0 to 47 or 0 to 15
  uint32_t  command:8;     //0x80=disable, 0x81=enable, 0=firmware response
  uint32_t  un1:8;
  uint32_t  retrn_code:8;  //0=idle, 1=success, 2-6=error

  uint32_t  queue_mask;    //(multi-mode only) bit 0=qm_index queue

  uint32_t  list_address;  //address of Host ping-pong buffer

  uint32_t  qm_index:16;   //qnum to monitor (multiple of 32 for multimode)
  uint32_t  max_entries:16;//max entries per list

  uint32_t  timer_count:16;//number of 25us timer ticks to delay int
  uint32_t  cfg_int_delay:2;//0=none, 1=last int, 2=1st new, 3=last new
  uint32_t  cfg_list_size:2;//0="D" Reg, 1="C+D" regs, 2="A+B+C+D"
  uint32_t  cfg_list_mode:1;//0=NULL terminate, 1=entry count mode
  uint32_t  cfg_multi_q:1; //0=single queue mode, 1=multi queue mode
  uint32_t  cfg_un:2;
  uint32_t  un2:8;
} Qmss_AccCmd;
#endif

void pdsp_program_accumulator(uint16_t pdsp, Qmss_AccCmd *cmd);
void pdsp_disable_accumulator(uint16_t pdsp, uint16_t channel);
void pdsp_set_firmware_timer(uint16_t pdsp, uint16_t time);


#endif /*IQN2_CONFIG_H_*/
