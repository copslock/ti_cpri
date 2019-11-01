/****************************************************************************\
 *           Copyright (C) 2011 Texas Instruments Incorporated.             *
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
 *            February, 2011                                                *
 *                                                                          *
 * This file contains functions for board level configuration operations    *
 * such as cache configuration and maintenance.                             *
 *                                                                          *
 ***************************************************************************/
#include "IQN2_config.h"



/* This function converts a local address to a global address.
 * Global addresses are returned unchanged.
 */
uint32_t l2_global_address(uint32_t coreId, uint32_t addr)
{
  if ((addr >= 0x00800000) && (addr < 0x00F08000))
  {
    return (addr + (0x10000000 + (coreId * 0x01000000)));
  }

  return (addr);
}


/* This function returns a unique core (DSP) number for the chip.
 */
uint32_t get_core_id(void)
{
  return(CSL_chipReadReg(CSL_CHIP_DNUM));
}


/* This function configures the L1P cache size. Use the following
 * values for cache_size:
 *   0 = no cache
 *   1 = 4K cache
 *   2 = 8K cache
 *   3 = 16K cache
 *   7 = 32K cache
 * For cache sizes less than 32K, the cache is placed in the highest
 * addresses of the memory (the lowest addresses are available as
 * memory). 32KB L1P local memory is located at 0x00E0_0000.
 */
void config_l1p_cache(uint16_t cache_size)
{
  uint32_t *L1PCFG;
  uint32_t  oldCfg;

  L1PCFG = (uint32_t *)0x01840020;

  oldCfg = *L1PCFG;
 *L1PCFG = (oldCfg & 0xFFFFFFF8) | cache_size;
}


/* This function configures the L1D cache size. Use the following
 * values for cache_size:
 *   0 = no cache
 *   1 = 4K cache
 *   2 = 8K cache
 *   3 = 16K cache
 *   7 = 32K cache
 * For cache sizes less than 32K, the cache is placed in the highest
 * addresses of the memory (the lowest addresses are available as
 * memory). 32KB L1D local memory is located at 0x00F0_0000.
 */
void config_l1d_cache(uint16_t cache_size)
{
  uint32_t *L1DCFG;
  uint32_t  oldCfg;

  L1DCFG = (uint32_t *)0x01840040;

  oldCfg = *L1DCFG;
 *L1DCFG = (oldCfg & 0xFFFFFFF8) | cache_size;
}


/* This function configures the L2 cache size. Use the following
 * values for cache_size:
 *   0 = no cache
 *   1 = 32K cache
 *   2 = 64K cache
 *   3 = 128K cache
 *   7 = 256K cache
 * For all non-zero cache sizes, the cache is placed in the highest
 * addresses of the memory (the lowest addresses are available as
 * memory). 1MB L2 local memory is located at 0x0080_0000.
 */
void config_l2_cache(uint16_t cache_size)
{
  uint32_t *LL2CFG;
  uint32_t  oldCfg;

  LL2CFG = (uint32_t *)0x01840000;

  oldCfg = *LL2CFG;
 *LL2CFG = (oldCfg & 0xFFFFFFF8) | cache_size;
}


/* This function updates the global L1D cache state.
 *   writeback  = 1: causes all dirty L1D cache lines to be written back.
 *   invalidate = 1: causes all L1D cache lines to be invalidated.
 *
 */
void l1d_cache_wb_inv(uint8_t writeback, uint8_t invalidate)
{
  uint32_t *L1reg;

  if ((writeback == 1) && (invalidate == 1))
  {
    L1reg = (uint32_t *)0x01845044; //L1DWBINV
   *L1reg = 1;
  }
  else if (writeback == 1)
  {
    L1reg = (uint32_t *)0x01845040; //L1DWB
   *L1reg = 1;
  }
  else if (invalidate == 1)
  {
    L1reg = (uint32_t *)0x01845048; //L1DINV
   *L1reg = 1;
  }
}


/* This function updates the global L2 cache state.
 *   writeback  = 1: causes all dirty L2 cache lines to be written back.
 *   invalidate = 1: causes all L2 cache lines to be invalidated.
 *
 */
void l2_cache_wb_inv(uint8_t writeback, uint8_t invalidate)
{
  uint32_t *L2reg;

  if ((writeback == 1) && (invalidate == 1))
  {
    L2reg = (uint32_t *)0x01845004; //L2WBINV
   *L2reg = 1;
  }
  else if (writeback == 1)
  {
    L2reg = (uint32_t *)0x01845000; //L2WB
   *L2reg = 1;
  }
  else if (invalidate == 1)
  {
    L2reg = (uint32_t *)0x01845008; //L2INV
   *L2reg = 1;
  }
}


/* This function configures a region of global memory to be cached in L2
 * or L1D. Indicate the region to be cached using an address on a 16MB
 * boundary such as these:
 *   0x0c000000 = cache all 2MB of MSMC
 *   0x80000000 = cache 0x80000000 to 0x80FFFFFF of DDR
 *   0x83000000 = cache 0x83000000 to 0x83FFFFFF of DDR
 *   0x91000000 = cache 0x91000000 to 0x91FFFFFF of DDR
 */
void config_cache_region(uint32_t address)
{
  uint32_t *MAR;
  uint32_t  region = address >> 24;

  if (region == 0x0c) //MSMC
  {
    MAR = (uint32_t *)0x01848030; //MAR12
   *MAR = 1;
  }
  else //region is in DDR
  {
    MAR = (uint32_t *)0x01848200 + ((region - 0x80) * 4); //MAR128...
   *MAR = 1;
  }
}


/* This function grants access to the QMSS queue proxy region in case
 * it hasn't been enabled by boot.
 */
void qproxy_access(void)
{
  uint32_t *reg;

  //Grant access to the queue proxy region
  reg = (uint32_t *)MPU2_PROG1_MPPA_ADDR;
 *reg = MPU2_PROG1_MPPA_GRANT_ALL;
}
