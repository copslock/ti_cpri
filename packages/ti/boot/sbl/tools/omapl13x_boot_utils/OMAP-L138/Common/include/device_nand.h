/* --------------------------------------------------------------------------
  FILE        : device_nand.h
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Provides device differentiation for the project files. This
                file MUST be modified to match the device specifics.
----------------------------------------------------------------------------- */

#ifndef _DEVICE_NAND_H_
#define _DEVICE_NAND_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/

#define DEVICE_NAND_DATA_OFFSET        (0x00000000u)
#define DEVICE_NAND_ALE_OFFSET         (0x00000008u)
#define DEVICE_NAND_CLE_OFFSET         (0x00000010u)
#define DEVICE_NAND_TIMEOUT            (10240)

#define DEVICE_NAND_MAX_BYTES_PER_OP       (512)   // Max Bytes per operation (EMIF IP constrained)
#define DEVICE_NAND_MAX_SPAREBYTES_PER_OP  (16)    // Max Spare Bytes per operation
#define DEVICE_NAND_MIN_SPAREBYTES_PER_OP  (10)    // Min Spare Bytes per operation (ECC operation constrained)

// Defines which NAND blocks the RBL will search in for a UBL image
#define DEVICE_NAND_RBL_SEARCH_START_BLOCK     (1)
#define DEVICE_NAND_RBL_SEARCH_END_BLOCK       (5)
#define DEVICE_NAND_DSPUBL_SEARCH_START_BLOCK  (1)
#define DEVICE_NAND_DSPUBL_SEARCH_END_BLOCK    (24)
#define DEVICE_NAND_ARMUBL_SEARCH_START_BLOCK  (2)
#define DEVICE_NAND_ARMUBL_SEARCH_END_BLOCK    (24)
#define DEVICE_NAND_UBOOT_SEARCH_START_BLOCK   (4)
#define DEVICE_NAND_UBOOT_SEARCH_END_BLOCK     (24)

// Defines which NAND blocks are valid for writing the APP data
#define DEVICE_NAND_UBL_SEARCH_START_BLOCK     (6)
#define DEVICE_NAND_UBL_SEARCH_END_BLOCK       (50)

// Defines EMIF type used 
#define EMIF_TYPE_NAND	(0)
#define DEVICE_EMIF_CS_REGION (3)
#define NAND_BASE_ADDR 0x62000000u

/******************************************************
* Global Typedef declarations                         *
******************************************************/


/***********************************************************
* Global Function Declarations                             *
***********************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_NAND_H_

