/* --------------------------------------------------------------------------
  FILE        : device_sdmmc.h
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Provides device differentiation for the project files. This
                file MUST be modified to match the device specifics.
----------------------------------------------------------------------------- */

#ifndef _DEVICE_SDMMC_H_
#define _DEVICE_SDMMC_H_

#include "tistdtypes.h"
#include "sdmmc.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/

#define DEVICE_SDMMC_TIMEOUT            (10240)
#define DEVICE_SDMMCBOOT_PERIPHNUM          (0)

// Define the size of the FIFO for high and low levels
#define DEVICE_SDMMC_FIFO_LEVEL_LOW_COUNT     (32)
#define DEVICE_SDMMC_FIFO_LEVEL_LOW_SHIFT     (5)
#define DEVICE_SDMMC_FIFO_LEVEL_HIGH_COUNT    (64)
#define DEVICE_SDMMC_FIFO_LEVEL_HIGH_SHIFT    (6)


/***********************************************************
* Global Typedef Declarations                              *
***********************************************************/


/***********************************************************
* Global Variable Declarations                             *
***********************************************************/

extern __FAR__ SDMMC_ConfigHandle const hDEVICE_SDMMC_config;


/***********************************************************
* Global Function Declarations                             *
***********************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_SDMMC_H_

