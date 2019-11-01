/****************************************************************************\
 *           Copyright (C) 2009 Texas Instruments Incorporated.             *
 *                           All Rights Reserved                            *
 *                                                                          *
 * GENERAL DISCLAIMER                                                       *
 * ------------------                                                       *
 * All software and related documentation is provided "AS IS" and without   *
 * warranty or support of any kind and Texas Instruments expressly disclaims*
 * all other warranties, express or implied, including, but not limited to, *
 * the implied warranties of merchantability and fitness for a particular   *
 * purpose.  Under no circumstances shall Texas Instruments be liable for   *
 * any incidental, special or consequential damages that result from the    *
 * use or inability to use the software or related documentation, even if   *
 * Texas Instruments has been advised of the liability.                     *
 ****************************************************************************
 *                                                                          *
 * Written by :                                                             *
 *            Dave Woodall                                                  *
 *            Texas Instruments                                             *
 *            22 May, 2012                                                 *
 *                                                                          *
 * The purpose of this test is to validate the integration of the MNAV      *
 * model into the Kepler, Lamarr platform.                              *
 * This test should not be redistributed for any reason without             *
 * permission.                                                              *
 *                                                                          *
 ***************************************************************************/
#include "IQN2_config.h"
/* NOTE: These APIs only used for devices that has only one qm like Lamarr */

#define USE_VBUSM


/* This function programs a QM memory region. */
void qm_set_memory_region(uint16_t qm, uint16_t regn, uint32_t addr, uint32_t indx, uint32_t setup)
{
  uint32_t *reg;
  uint32_t  rgn;

  rgn = QM1_DESC_REGION;

  reg = (uint32_t *)(rgn + QM_REG_MEM_REGION_BASE + (regn * 16));
 *reg = addr;

  reg = (uint32_t *)(rgn + QM_REG_MEM_REGION_INDEX + (regn * 16));
 *reg = indx;

  /* See register description for programming values. */
  reg = (uint32_t *)(rgn + QM_REG_MEM_REGION_SETUP + (regn * 16));
 *reg = setup;
}


/* This function programs a QM Link RAM. */
void qm_set_link_ram(uint16_t qm, uint16_t ram, uint32_t addr, uint32_t count)
{
  uint32_t *reg;
  uint32_t  regn;

  regn = QM1_CFG_REGION;

  reg = (uint32_t *)(regn + QM_REG_LINKRAM_0_BASE + (ram * 8));
 *reg = addr;

  if (ram == 0)
  {
    reg = (uint32_t *)(regn + QM_REG_LINKRAM_0_SIZE);
   *reg = count;
  }
}


/* This function pushes descriptor info to a queue.
 * mode parameter:  1 = write reg D only.
 *                  2 = write regs C and D.
 *
 * It turns out the VBUSM is more efficient to push to than VBUSP, and due
 * to the problem with qproxy, also allows for atomic c+d pushes.
 */
void qm_push_queue(uint16_t qn, uint8_t mode, uint32_t c_val, uint32_t d_val)
{
#ifdef USE_VBUSM

  if (mode == 2)
  {
    uint64_t *reg;

    reg = (uint64_t *)(QM1_QMAN_VBUSM_REGION + QM_REG_QUE_REG_C + (qn * 16));

    #ifdef _BIG_ENDIAN
     *reg = ((uint64_t)c_val << 32) | d_val;
    #else
     *reg = ((uint64_t)d_val << 32) | c_val;
    #endif
  }
  else
  {
    uint32_t *reg;
    reg = (uint32_t *)(QM1_QMAN_VBUSM_REGION + QM_REG_QUE_REG_D + (qn * 16));
   *reg = d_val;
  }
#else
  uint32_t *reg;

  if (mode == 2)
  {
    reg = (uint32_t *)(QM1_QMAN_REGION + QM_REG_QUE_REG_C + (qn * 16));
   *reg = c_val;
  }

  reg = (uint32_t *)(QM1_QMAN_REGION + QM_REG_QUE_REG_D + (qn * 16));
 *reg = d_val;
#endif
}


/* This function pops a descriptor from a queue. Since it is more
 * efficient to pop from VBUSP, use it always. */
uint32_t qm_pop_queue(uint16_t qn)
{
  uint32_t *reg;
  uint32_t  value;

  reg = (uint32_t *)(QM1_QMAN_REGION + QM_REG_QUE_REG_D + (qn * 16));
  value = *reg;

  return(value);
}


/* This function moves a source queue to a destination queue.  If
 * headtail = 0, the source queue is appended to the tail of the
 * dest queue.  If 1, it is appended at the head. Both src_qn and
 * dest_qn must be from the same Queue Manager!!! */
void qm_divert_queue(uint16_t qm, uint16_t src_qn, uint16_t dest_qn, uint8_t headtail)
{
  uint32_t *reg;
  uint32_t  value;


  reg = (uint32_t *)(QM1_CFG_REGION + QM_REG_QUE_DIVERSION);


  value = (headtail << 31) + (dest_qn << 16) + src_qn;
 *reg = value;

  return;
}


/* This function pops a queue until it is empty. If *list is not NULL,
 * it will return the list of descriptor addresses and the count. */
void qm_empty_queue(uint16_t qn, uint32_t *list, uint32_t *listCount)
{
  uint32_t *reg;
  uint32_t  value;
  uint16_t  idx;
  uint32_t  count;

  reg = (uint32_t *)(QM1_PEEK_REGION + QM_REG_QUE_REG_A + (qn * 16));
  count = *reg; //read the descriptor count
 *listCount = count;

  reg = (uint32_t *)(QM1_QMAN_REGION + QM_REG_QUE_REG_D + (qn * 16));

  for (idx = 0; idx < count; idx ++)
  {
    value = *reg;
    if (list != NULL)
    {
      list[idx] = value;
    }
  }
}


/* This function returns the byte count of a queue. */
uint32_t qm_get_byte_count(uint16_t qn)
{
  uint32_t *reg;
  uint32_t  count;

  reg = (uint32_t *)(QM1_PEEK_REGION + QM_REG_QUE_REG_B + (qn * 16));
  count = *reg;

  return(count);
}


/* This function returns the descriptor count of a queue. */
uint32_t qm_get_descriptor_count(uint16_t qn)
{
  uint32_t *reg;
  uint32_t  count;

  reg = (uint32_t *)(QM1_PEEK_REGION + QM_REG_QUE_REG_A + (qn * 16));
  count = *reg;

  return(count);
}


/* This function sets a queue threshold for triggering PDSP firmware. */
void qm_set_queue_threshold(uint16_t qn, uint32_t value)
{
  uint32_t *reg;

  reg = (uint32_t *)(QM1_PEEK_REGION + QM_REG_QUE_STATUS_REG_D + (qn * 16));
 *reg = value;
}


/* This function returns 1 if the Status RAM for a queue is 1, else 0. */
uint8_t qm_get_queue_status(uint16_t qn)
{
  uint32_t *reg;
  uint32_t  word;
  uint32_t  bit;
  uint32_t  statmask;

  word = qn / 32;
  bit  = qn - (word * 32);
  reg  = (uint32_t *)(QM1_STATUS_REGION + (word * 4));
  statmask = *reg;

  return((uint8_t)(statmask >> bit) & 0x01);
}


/* This function programs base addresses for the four logical
 * queue managers that a PKTDMA may setup.  Use a value of 0xffff
 * if you don't want to set value into QM base addr reg. N. */
void pktdma_config_qm(uint32_t base, uint16_t *physical_qmgr, uint16_t *physical_qnum)
{
  uint16_t  idx;
  uint32_t  qm_base;
  uint32_t *reg;

  for (idx = 0; idx < 2; idx ++)
  {
    if (physical_qnum[idx] != 0xffff)
    {
      qm_base = QM1_QMAN_VBUSM_REGION + (physical_qnum[idx] * 16);

      reg = (uint32_t *)(base + PKTDMA_REG_QM0_BASE_ADDR + (idx * 4));
     *reg = qm_base;
    }
  }
}


/* This function enables/disables internal loopback mode for a pktDMA.
 * By default, it should be enabled for QMSS, disabled for all others. */
void pktdma_config_loopback(uint32_t base, uint8_t enable)
{
  uint32_t *reg;

  reg = (uint32_t *)(base + PKTDMA_REG_EMULATION_CTRL);

  if (enable)
   *reg = 0x80000000;
  else
   *reg = 0x0;
}


/* This function sets the packet retry timeout.
 * A value of 0 disables the retry feature. */
void pktdma_config_retry_timeout(uint32_t base, uint16_t timeout)
{
  uint32_t *reg;
  uint32_t  val;

  reg = (uint32_t *)(base + PKTDMA_REG_PERFORM_CTRL);

  val = *reg & 0xFFFF0000;

 *reg = val | timeout;
}


/* This function disables a TX PKTDMA channel, then configures Reg B. */
void pktdma_config_tx_chan(uint32_t base, uint16_t chan, uint32_t reg_b)
{
  uint32_t *reg;

  reg = (uint32_t *)(base + PKTDMA_REG_TX_CHAN_CFG_A + (chan * 32));
 *reg = 0; //disable the channel

  reg = (uint32_t *)(base + PKTDMA_REG_TX_CHAN_CFG_B + (chan * 32));
 *reg = reg_b;
}


/* This function configures priority a TX PKTDMA channel */
void pktdma_config_tx_sched(uint32_t base, uint16_t chan, uint32_t priority)
{
  uint32_t *reg;

  reg = (uint32_t *)(base + PKTDMA_REG_TX_SCHED_CHAN_CFG + (chan * 4));
 *reg = priority;
}


/* This function configures a RX PKTDMA channel flow. */
void pktdma_config_rx_flow(uint32_t base, uint16_t flow,
                           uint32_t a, uint32_t b, uint32_t c, uint32_t d,
                           uint32_t e, uint32_t f, uint32_t g, uint32_t h)
{
  uint32_t *reg;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_A + (flow * 32));
 *reg = a;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_B + (flow * 32));
 *reg = b;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_C + (flow * 32));
 *reg = c;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_D + (flow * 32));
 *reg = d;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_E + (flow * 32));
 *reg = e;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_F + (flow * 32));
 *reg = f;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_G + (flow * 32));
 *reg = g;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_FLOW_CFG_H + (flow * 32));
 *reg = h;
}


/* This function writes an RX PKTDMA channel's enable register. */
void pktdma_enable_rx_chan(uint32_t base, uint16_t chan, uint32_t value)
{
  uint32_t *reg;

  reg = (uint32_t *)(base + PKTDMA_REG_RX_CHAN_CFG_A + (chan * 32));
 *reg = value;
}


/* This function writes a TX PKTDMA channel's enable register. */
void pktdma_enable_tx_chan(uint32_t base, uint16_t chan, uint32_t value)
{
  uint32_t *reg;

  reg = (uint32_t *)(base + PKTDMA_REG_TX_CHAN_CFG_A + (chan * 32));
 *reg = value;
}


/* This function reads a QMSS INTD Status Register.
 * group parameter:  0 = high priority interrupts.
 *                   1 = low priority interrupts.
 *                   4 = PKTDMA starvation interrupts.
 *
 * If the chan parameter = 0xffffffff, the entire register contents
 * are returned.  Otherwise, chan is expected to be a channel number,
 * and the return value will be a 0 or 1 for that channel's status.
 */
uint32_t intd_read_status(uint16_t intd, uint16_t group, uint32_t chan)
{
  uint32_t *reg;
  uint32_t  value;
  uint32_t  mask;
  uint32_t  offset = (intd - 1) * INTD_REG_OFFSET + QM_REG_INTD_STATUS + (group * 4);

  reg = (uint32_t *)(QMSS_INTD1_REGION + offset);

  value = *reg;

  if (chan != 0xffffffff)
  {
    mask = 1 << (chan & 0x001f);
    if ((value & mask) == 0)
      value = 0;
    else
      value = 1;
  }

  return(value);
}


/* This function writes a QMSS INTD Status Register.
 * group parameter:  0 = high priority interrupts.
 *                   1 = low priority interrupts.
 *                   4 = PKTDMA starvation interrupts.
 */
void intd_set_status(uint16_t intd, uint16_t group, uint32_t chan)
{
  uint32_t *reg;
  uint32_t  value;
  uint32_t  mask;
  uint32_t  offset = (intd - 1) * INTD_REG_OFFSET + QM_REG_INTD_STATUS + (group * 4);

  reg = (uint32_t *)(QMSS_INTD1_REGION + offset);

  value = *reg;

  mask = 1 << (chan & 0x001f);
  value |= mask;
 *reg = value;
}


/* This function writes a QMSS INTD Status Clear Register.
 * group parameter:  0 = high priority interrupts.
 *                   1 = low priority interrupts.
 *                   4 = PKTDMA starvation interrupts.
 */
void intd_clear_status(uint16_t intd, uint16_t group, uint32_t chan)
{
  uint32_t *reg;
  uint32_t  value;
  uint32_t  mask;
  uint32_t  offset = (intd - 1) * INTD_REG_OFFSET + QM_REG_INTD_STATUS_CLEAR + (group * 4);

  reg = (uint32_t *)(QMSS_INTD1_REGION + offset);

  value = *reg;

  mask = 1 << (chan & 0x001f);
  value |= mask;
 *reg = value;
}


/* This function reads a QMSS INTD Int Count Register.
 * Reading has no effect on the register.
 * "intnum" is:  0..31 for High Pri interrupts
 *              32..47 for Low Pri interrupts
 *              48..49 for PKTDMA Starvation interrupts
 */
uint32_t intd_read_intcount(uint16_t intd, uint16_t intnum)
{
  uint32_t *reg;
  uint32_t  value;
  uint32_t  offset = (intd - 1) * INTD_REG_OFFSET + QM_REG_INTD_INT_COUNT + (intnum * 4);

  reg = (uint32_t *)(QMSS_INTD1_REGION + offset);

  value = *reg;

  return(value);
}


/* This function reads a QMSS INTD Int Count Register.
 * Writing will cause the written value to be subtracted from the register.
 * "intnum" is:  0..31 for High Pri interrupts
 *              32..47 for Low Pri interrupts
 *              48..49 for PKTDMA Starvation interrupts
 */
void intd_write_intcount(uint16_t intd, uint16_t intnum, uint32_t val)
{
  uint32_t *reg;
  uint32_t  offset = (intd - 1) * INTD_REG_OFFSET + QM_REG_INTD_INT_COUNT + (intnum * 4);

  reg = (uint32_t *)(QMSS_INTD1_REGION + offset);

 *reg = val;
}


/* This function writes a QMSS INTD EOI Register.  Values to write are:
     0 or  1: PKTDMA starvation interrupts,
     2 to 33: High Pri interrupts,
    34 to 49: Low Pri interrupts.
 * Writing one of these values will clear the corresponding interrupt.
 */
void intd_write_eoi(uint16_t intd, uint32_t val)
{
  uint32_t *reg;
  uint32_t  offset = (intd - 1) * INTD_REG_OFFSET + QM_REG_INTD_EOI;

  reg = (uint32_t *)(QMSS_INTD1_REGION + offset);

 *reg = val;
}


/* This function writes a QMSS PDSP Control Register. */
void pdsp_control(uint16_t pdsp, uint32_t val)
{
  uint32_t *reg;
  uint32_t  offset = (pdsp - 1) * PDSP_REG_OFFSET + QM_REG_PDSP_CONTROL;

  reg = (uint32_t *)(PDSP1_REG_REGION + offset);

 *reg = val;
}


/* This function enables QMSS PDSP n. */
void pdsp_enable(uint16_t pdsp)
{
  uint32_t *reg;
  uint32_t  tmp;
  uint32_t  offset = (pdsp - 1) * PDSP_REG_OFFSET + QM_REG_PDSP_CONTROL;

  reg = (uint32_t *)(PDSP1_REG_REGION + offset);

  tmp  = *reg;
  tmp |= 0x02;
 *reg  = tmp;
}


/* This function disables QMSS PDSP n. */
void pdsp_disable(uint16_t pdsp)
{
  uint32_t *reg;
  uint32_t  tmp;
  uint32_t  offset = (pdsp - 1) * PDSP_REG_OFFSET + QM_REG_PDSP_CONTROL;

  reg = (uint32_t *)(PDSP1_REG_REGION + offset);

  tmp  = *reg;
  tmp &= 0xfffffffd;
 *reg  = tmp;
}


/* This function returns true if QMSS PDSP n is running. */
uint8_t pdsp_running(uint16_t pdsp)
{
  uint32_t *reg;
  uint32_t  offset = (pdsp - 1) * PDSP_REG_OFFSET + QM_REG_PDSP_CONTROL;

  reg = (uint32_t *)(PDSP1_REG_REGION + offset);

  return(*reg & 0x00008000);
}


/* This function controls the PDSP to load firmware to it. */
void pdsp_download_firmware(uint16_t pdsp, uint8_t *code, uint32_t size)
{
  uint16_t  idx;
  uint32_t  value;
  uint32_t  offset;
  uint32_t *reg;

  /* Reset PDSP 1 */
  pdsp_disable(pdsp);

  /* Confirm PDSP has halted */
  do
  {
    value = pdsp_running(pdsp);
  } while (value == 1);

  offset = (pdsp - 1) * PDSP_IRAM_OFFSET;

  /* Download the firmware */
  memcpy ((void *)(PDSP1_IRAM_REGION + offset), code, size);

  /* Use the command register to sync the PDSP */
  offset = (pdsp - 1) * PDSP_CMD_OFFSET;
  reg = (uint32_t *)(PDSP1_CMD_REGION + offset);
 *reg = 0xffffffff;

  /* Wait to the memory write to land */
  for (idx = 0; idx < 20000; idx++)
  {
    value = *reg;
    if (value == 0xffffffff) break;
  }

  /* Reset program counter to zero, and clear Soft Reset bit. */
  offset = (pdsp - 1) * PDSP_REG_OFFSET + QM_REG_PDSP_CONTROL;
  reg = (uint32_t *)(PDSP1_REG_REGION + offset);

  value = *reg;
 *reg = value & 0x0000fffe; //PC reset is in upper 16 bits, soft reset in bit 0

  /* Enable the PDSP */
  pdsp_enable(pdsp);

  /* Wait for the command register to clear */
  offset = (pdsp - 1) * PDSP_CMD_OFFSET;
  reg = (uint32_t *)(PDSP1_CMD_REGION + offset);
  do
  {
    value = *reg;
  } while (value != 0);
}


/* This function programs a Hi or Lo Accumulator channel. */
void pdsp_program_accumulator(uint16_t pdsp, Qmss_AccCmd *cmd)
{
  uint16_t  idx;
  uint8_t   result;
  uint32_t  offset;
  uint32_t *tmplist;
  uint32_t *reg;

  offset = (pdsp - 1) * PDSP_CMD_OFFSET;
  reg = (uint32_t *)(PDSP1_CMD_REGION + offset + 4*4); //point to last word

  tmplist = ((uint32_t *) cmd) + 4; //write first word last

  for (idx = 0; idx < 5; idx ++)
  {
    *reg-- = *tmplist--;
  }

  /* wait for the command byte to clear */
  reg++;
  do
  {
    result = (*reg & 0x0000ff00) >> 8;
  } while (result != 0);
}


/* This function disables a Hi or Lo Accumulator program. */
void pdsp_disable_accumulator(uint16_t pdsp, uint16_t channel)
{
  uint16_t  idx;
  uint32_t  offset;
  uint32_t *tmplist;
  uint32_t *reg;
  Qmss_AccCmd cmd;

  memset(&cmd, 0, sizeof(Qmss_AccCmd));
  cmd.channel = channel;
  cmd.command = QMSS_ACC_CMD_DISABLE;

  offset = (pdsp - 1) * PDSP_CMD_OFFSET;
  reg = (uint32_t *)(PDSP1_CMD_REGION + offset + 4*4); //point to last word

  tmplist = ((uint32_t *) &cmd) + 4; //write first word last

  for (idx = 0; idx < 5; idx ++)
  {
    *reg-- = *tmplist--;
  }
}


/* This function writes a new value to a PDSP's firmware
 * Time is specified in usec, then converted to the hardware
 * expect value assuming a 350Mhz QMSS sub-system clock. */
void pdsp_set_firmware_timer(uint16_t pdsp, uint16_t time)
{
  uint32_t  offset;
  uint32_t *tmplist;
  uint32_t *reg;
  Qmss_AccCmd cmd;

  memset(&cmd, 0, sizeof(Qmss_AccCmd));
  cmd.queue_mask = (time * 175); //convert usec to hw val
  cmd.command = QMSS_ACC_CMD_TIMER;

  offset = (pdsp - 1) * PDSP_CMD_OFFSET;
  reg = (uint32_t *)(PDSP1_CMD_REGION + offset + 4); //point to 2nd word

  tmplist = ((uint32_t *) &cmd) + 1; //write 2nd word last

  *reg-- = *tmplist--;
  *reg   = *tmplist;
}
