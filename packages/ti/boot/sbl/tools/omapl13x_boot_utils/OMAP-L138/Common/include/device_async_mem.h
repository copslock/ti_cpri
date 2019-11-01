/* --------------------------------------------------------------------------
  FILE        : device_async_mem.h
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Provides device differentiation for the project files. This
                file MUST be modified to match the device/platform specifics.
----------------------------------------------------------------------------- */

#ifndef _DEVICE_ASYNC_MEM_H_
#define _DEVICE_ASYNC_MEM_H_

#include "tistdtypes.h"
#include "async_mem.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/

#define DEVICE_ASYNC_MEM_INTERFACE_CNT  (1)
#define DEVICE_ASYNC_MEM0_REGION_CNT    (4)

#define DEVICE_ASYNC_MEM_NORBOOT_INTERFACE    (0)
#define DEVICE_ASYNC_MEM_NORBOOT_REGION       (0)

#define DEVICE_ASYNC_MEM_NANDBOOT_INTERFACE   (0)
#define DEVICE_ASYNC_MEM_NANDBOOT_REGION      (1)

#if defined(OMAPL138_LCDK)
  #define DEVICE_ASYNC_MEM_NANDBOOT_BUSWIDTH    (DEVICE_BUSWIDTH_16BIT)
#elif defined(C6748_LCDK)
  #define DEVICE_ASYNC_MEM_NANDBOOT_BUSWIDTH    (DEVICE_BUSWIDTH_16BIT)
#else
  #define DEVICE_ASYNC_MEM_NANDBOOT_BUSWIDTH    (DEVICE_BUSWIDTH_8BIT)
#endif

#define DEVICE_ASYNC_MEM_ONENAND_INTERFACE    (0)
#define DEVICE_ASYNC_MEM_ONENAND_REGION       (0)


/***********************************************************
* Global Typedef Declarations                              *
***********************************************************/


/***********************************************************
* Global Variable Declarations                             *
***********************************************************/

extern __FAR__ const ASYNC_MEM_DEVICE_InterfaceObj DEVICE_ASYNC_MEM_interfaces[];
extern __FAR__ const ASYNC_MEM_DEVICE_InfoObj DEVICE_ASYNC_MEM_info;


/***********************************************************
* Global Function Declarations                             *
***********************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_ASYNC_MEM_H_

