/* --------------------------------------------------------------------------
  FILE        : device_spi.h
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Provides device differentiation for the project files. This
                file MUST be modified to match the device specifics.
----------------------------------------------------------------------------- */

#ifndef _DEVICE_SPI_H_
#define _DEVICE_SPI_H_

#include "tistdtypes.h"
#include "spi_mem.h"
#include "spi.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/

#define DEVICE_SPI_TIMEOUT            (10240)
#define DEVICE_SPI_DATOFFSET          (1)

#define DEVICE_SPIBOOT_PERIPHNUM          (1)
#define DEVICE_SPIBOOT_CSNUM              (0)

#define DEVICE_SPI_UBL_HDR_OFFSET     (0*1024)
#define DEVICE_SPI_APP_HDR_OFFSET     (64*1024)


/***********************************************************
* Global Typedef Declarations                              *
***********************************************************/


/***********************************************************
* Global Variable Declarations                             *
***********************************************************/

extern __FAR__ SPI_ConfigHandle const hDEVICE_SPI_config;
extern __FAR__ SPI_MEM_ParamsHandle const hDEVICE_SPI_MEM_params;


/***********************************************************
* Global Function Declarations                             *
***********************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_SPI_H_

