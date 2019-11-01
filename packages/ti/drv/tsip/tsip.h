/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2013
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

#ifndef _TSIP_H
#define _TSIP_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>
#include <ti/csl/csl_tsip.h>
#include "tsip_cfg.h"
#include "tsipver.h"
/* ============================================================= */
/**
 *   @file  tsip.h
 *
 *   path  ti/drv/tsip/tsip.h
 *
 *   @brief  Telecom Serial Interface Port (TSIP) sub-system API and Data Definitions
 *
 */

/**  @mainpage Telecom Serial Interface Port Low Level Driver
 *
 *
 *   @section intro  Introduction

 *   The TSIP is a multi-link serial interface consisting of a maximum of eight transmit data signals (or links),
 *   eight receive data signals (or links), two frame sync input signals, and two serial clock inputs. Internally the
 *   TSIP offers multiple channels of timeslot data management and multi-channel DMA capability that allow
 *   individual timeslots to be selectively processed.
 *
 *   The module can be configured to use the frame sync signals and the serial clocks as redundant sources
 *   for all transmit and receive data signals, or one frame sync and serial clock for transmit and the second
 *   frame sync and clock for receive. The standard serial data rate for each TSIP transmit and receive data
 *   signal is 8.192 Mbps. The standard frame sync is a one or more bit wide pulse that occurs once every 125
 *   µs or a minimum of one serial clock period every 1024 serial clocks.
 *
 *   At the standard rate and default configuration there are eight transmit and eight receive links that are
 *   active. Each serial interface link supports up to 128 8-bit timeslots. This corresponds to an H-MVIP or
 *   H.110 serial data rate interface. The serial interface clock frequency can be either 16.384 MHz (default) or
 *   8.192 MHz. The data rate for the serial interface links can also be set to 16.384 Mbps or 32.768 Mbps.
 *   The maximum number of active serial links is reduced to four and two, respectively, in these
 *   configurations. The serial interface clock frequency can be either 32.768 MHz or 16.384 MHz for 16.384
 *   Mbps serial links, and 65.536 MHz or 32.768 MHz for 32.768 Mbps serial links. Maximum occupation of
 *   the serial interface links for the entire TSIP is 1024 transmit and receive timeslots in all configurations.
 *
 */

/* Define TSIPLLD Module as a master group in Doxygen format and add all TSIP LLD API 
   definitions to this group. */
/** @defgroup tsiplld_module TSIP LLD Module API
 *  @{
 */
/** @} */
   
/** @defgroup tsiplld_api_functions TSIP LLD Functions
 *  @ingroup tsiplld_module
 */
 
/** @defgroup tsiplld_api_macros TSIP LLD Macros
 *  @ingroup tsiplld_module
 */

/** @defgroup tsiplld_api_structures TSIP LLD Data Structures
 *  @ingroup tsiplld_module
 */

/** @defgroup tsiplld_api_constants TSIP LLD Constants (enum's and define's)
 *  @ingroup tsiplld_module
 */

/**
 *  @defgroup  ReturnValues  Function Return Values
 *  @ingroup tsiplld_api_constants
 *  @{
 *
 *  @name TSIPLLD Function Return Codes
 *
 *  Error codes returned by TSIPLLD API functions.
 */
/*@{*/

/**
 *  @def  tsip_OK
 *        TSIP return code -- Function executed successfully
 */
#define tsip_OK                                    0

/**
 *  @def  tsip_FAIL
 *        TSIP return code -- Function failed
 */
#define tsip_FAIL                                -10

/**
 *  @def  tsip_ERR_CONFIG
 *        Invalid configuration provided to TSIP
 */
#define tsip_ERR_CONFIG                          -11

/**
 *  @def  tsip_NO_PORTS_AVAILABLE
 *        No TSIP ports are available
 */
#define tsip_NO_PORTS_AVAILABLE                  -12

/**
 *  @def  tsip_INVALID_TSIP_PORT_ID
 *        Invalid ID to a TSIP port
 */
#define tsip_INVALID_TSIP_PORT_ID                -13

/**
 *  @def  tsip_TX_RX_ALREADY_ENABLED
 *        TSIP receive or transmit has been enabled before
 */
#define tsip_TX_RX_ALREADY_ENABLED               -14

/**
 *  @def  tsip_TDMU_DMATCU_ALREADY_ENABLED
 *        TSIP TDMU and DMATCU have been enabled before
 */
#define tsip_TDMU_DMATCU_ALREADY_ENABLED         -15

/**
 *  @def  tsip_TIMESLOT_ALREADY_ENABLED
 *        TSIP timeslot has been enabled before
 */
#define tsip_TIMESLOT_ALREADY_ENABLED            -16

/**
 *  @def  tsip_NO_FREE_TIMESLOT
 *        There is no free timeslot available
 */
#define tsip_NO_FREE_TIMESLOT                    -17

/*@}*/  
/**
 *  @defgroup  Valid param: for Size info paramters
 *  @ingroup tsiplld_api_size_info_valid_params
 *  @{
 *
 *  @name TSIPLLD Size info valid parameters
 *
 */
/*@{*/
/**
 *  @def  TSIP_SIZE_INFO_VALID_PARAMS_NUM_PORTS
 *        Number of port parameter is valid
 */
#define TSIP_SIZE_INFO_VALIDPARAMS_NUM_PORTS      0x1
/*@}*/  
/** @} */

/* TSIP memory buffer requirements for an instance of TSIP common to all the TSIP ports */
#define TSIP_BUFFNUM_INSTANCE_DRIVER      0
#define TSIP_BUFFNUM_HEAP_SHARED          1
#define TSIP_BUFFNUM_HEAP_TIMESLOT        2
#define TSIP_BUFFNUM_TX_DMA_BUFFER        3
#define TSIP_BUFFNUM_RX_DMA_BUFFER        4

/* TSIP memory buffer requirements for an instance of TSIP specific to individual TSIP ports */
#define TSIP_BUFFNUM_INSTANCE_PORT        0


/**
 *  @ingroup tsiplld_api_structures
 *  @brief  The return type for module functions
 *
 *  @details  Function calls to this module return values used to determine if the command was successful or
 *            the reason for failure (see @ref ReturnValues).
 */
typedef int tsipReturn_t;

/**
 * @ingroup tsiplld_api_structures
 * @brief Specification of Tsip_DrvHandle 
 *
 * The Tsip_DrvHandle is used to identify a TSIP Driver instance
 */
typedef void*  Tsip_DrvHandle;

/**
 * @ingroup tsiplld_api_structures
 * @brief Specification of Tsip_PortHandle 
 *
 * The Tsip_PortHandle is used to identify a TSIP Port instance
 */
typedef void*  Tsip_PortHandle;


/**
 * @ingroup tsiplld_api_structures
 * @brief  TSIP Size Configuration Structure
 *
 * @details The parameters in this structure are used to provide size information 
 *          for calculating the memory requirements for the TSIP driver
 */
typedef struct  {
  uint16_t validParams; /**< valid parameter bits for optional parameters */
  int16_t maxChannels;   /**< Maximum number of channels supported : Mandatory */
  int16_t subFrameSize;  /**< Number of frames per sub frame : Mandatory */
  int16_t wordSize;      /**< Size of one sample in bits : Mandatory  */
  uint16_t numPorts;     /**<Number of ports */
} tsipSizeInfo_t;


/**
 * @ingroup tsiplld_api_structures
 * @brief TSIP Initialization config structure specific to Tx/Rx directions
 *
 * @details The parameters in this structure are used to do initial setup
 *          of the TSIP driver specific to Tx/Rx directions.
 */

typedef struct  {

  int16_t channel;         /**< The DMA channel number                                    */

  int16_t frameSize;       /**< Frame size                                                */
  int16_t tsPerFrame;      /**< Number of timeslots per frame                             */

  int16_t clkSrc;          /**< Clock source                                              */
  int16_t dataDelay;       /**< Amount of delay between frame sync and the first data bit */
  int16_t bdxDelay;        /**< Tx output delay: enabled (1), disabled(0)                 */
  int16_t idleDrive;       /**< Value to drive on an idle timeslot (tx only)              */
  int16_t fsyncPol;        /**< Frame sync Polarity   (1 to invert)                       */
  int16_t fsyncClkPol;     /**< Frame sync clock polarity                                 */
  int16_t clkPol;          /**< Clock polarity                                            */
  int16_t dataRate;        /**< Data rate                                                 */
  int16_t clkMode;         /**< Clock mode                                                */

  int16_t superFrameInt;   /**< Super frame interrupt selection                           */
  int16_t frameInt;        /**< Frame interrupt selection                                 */
  int16_t frameIntDelay;   /**< Frame interrupt delay                                     */

} tsipConfigDir_t;


/**
 * @ingroup tsiplld_api_structures
 * @brief TSIP Initialization configuration structure
 *
 * @details The parameters in this structure are used to do initial setup
 *          of the driver.
 */
typedef struct  {

  int16_t physPort;        /**< Physical port number and it is 0 indexed */
  uint32_t hwId;           /**< Returned hardware ID in PID register     */
  uint16_t masterCore;     /**< Master core (True/False). Only the master core configures TSIP port registers*/

  int16_t testMode;        /**< Test mode enable/disable                 */
  int16_t testModeSelect;  /**< Test mode selection                      */

  int16_t clkRedund;       /**< Clock redundancy flag                    */
  int16_t endian;          /**< Endian mode selection                    */
  int16_t priority;        /**< Transfer priority                        */
  int16_t maxPriority;     /**< Maximum transfer priority                */

  tsipSizeInfo_t* sizeCfg;                  /**< Pointer to the size configuration information */
  int16_t  maxPhase;                           /**< Maximum number of phases tracked */
  void (*subFrameCallout) (void *, uint16_t);  /**< Periodic function to call at every subframe */
  void *cxt;                                /**< Callout context for the periodic function */
  tsipConfigDir_t tx;  /**< Configuration parameters specific to the transmit direction */
  tsipConfigDir_t rx;  /**< Configuration parameters specific to the receive direction */


} tsipConfig_t;

/**
 *  @ingroup tsiplld_api_structures
 *  @brief  The input/output data type of TSIP
 *
 *  @details  The input/output data type of TSIP.
 *  
 */
#ifdef TSIP_APPBUF_PACKED
typedef uint8_t tsipData_t;
#else
typedef int16_t tsipData_t;
#endif

/**
 * @ingroup tsiplld_api_structures
 * @brief TSIP timeslot run-time configuration structure specific to Tx/Rx directions
 *
 * @details The parameters in this structure are used to run-time configure Tx/Rx directions of
 *          individual timeslots
 */

typedef struct  {

  unsigned short enable;             /**< Enable or disable the timeslot  */
  int16_t        timeslot;           /**< Timeslot number                 
                                          Bits  TSIP_TS_LSB -  TSIP_TS_MSB: Timeslot ID
                                          Bits  TSIP_LINK_LSB - TSIP_LINK_MSB: Link ID
                                          Bits  TSIP_PORT_LSB - TSIP_PORT_MSB: Port ID */
  uint16_t       frameSize;          /**< Frame size and it must be multiple of subFrameRate */
  void           (*callout) (void *, tsipData_t **, tsipData_t **, uint32_t, uint16_t); /**< Callout function to transmit data between TSIP and application */
  void           *context;           /**< First argument to pass back in the above callout function */
  tsipData_t     *buffer;            /**< Initial data to send out for transmit; Place to store initial data for receive */

} tsipTsControlDir_t;


/**
 * @ingroup tsiplld_api_structures
 * @brief TSIP timeslot run-time configuration structure
 *
 * @details The parameters in this structure are used to run-time configure individual timeslots
 *          of the TSIP driver.
 */
typedef struct  {

  uint16_t           compand;       /**< Companding                               */
  uint16_t           phase;         /**< Phase of channel staggering              */
  void               *txTsContext;  /**< Returned timeslot context pointer        */
  tsipTsControlDir_t tx;     /**< Timeslot configuration specific to the transmit direction */
  tsipTsControlDir_t rx;     /**< Timeslot configuration specific to the receive direction  */

} tsipTsControl_t;


/**
 *  @brief   Define the maximum number of buffers the module can request common to all the TSIP ports
 *
 */
#define tsip_N_BUFS_SHARED           5

/**
 *  @brief   Define the maximum number of buffers the module can request for individual TSIP ports
 *
 */
#define tsip_N_BUFS_PORT             1 

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_getBufferReqShared returns the memory requirements for the TSIP driver which is shared
 *         by all the TSIP ports
 *
 *  @details This function returns the memory buffer requirements common to all the TSIP ports,
 *           in terms of the size and alignment array.
 *
 *  @param[in]   sizeCfg     Size configuration information
 *  @param[out]  sizes       Array of size requirements
 *  @param[out]  aligns      Array of alignment requirements
 *  @retval                  Value @ref ReturnValues
 */
tsipReturn_t Tsip_getBufferReqShared (tsipSizeInfo_t *sizeCfg, int sizes[], int aligns[]);


/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_getBufferReqPort returns the memory requirements for the TSIP driver which is specific
 *         to individual TSIP ports
 *
 *  @details This function returns the per port memory buffer requirements in terms
 *           of the size and alignment array.
 *
 *  @param[in]   sizeCfg     Size configuration information
 *  @param[out]  sizes       Array of size requirements
 *  @param[out]  aligns      Array of alignment requirements
 *  @retval                  Value @ref ReturnValues
 */
tsipReturn_t Tsip_getBufferReqPort   (tsipSizeInfo_t *sizeCfg, int sizes[], int aligns[]);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief  Tsip_createShared creates the TSIP driver instance and performs common initialization
 *
 *  @details This function initializes the TSIP driver common to all the TSIP ports
 *           based on user configuration
 *
 *  @param[in]  cfg     Configuration information
 *  @param[in]  bases   Array of the memory buffer base addresses 
 *  @param[out] pHandle Instance handle. This is a pointer to an initialized
 *                      instance structure. 
 *  @retval             Value @ref ReturnValues
 */
tsipReturn_t Tsip_createShared (tsipConfig_t *cfg, void* bases[], Tsip_DrvHandle *pHandle);


/**
 *  @ingroup tsiplld_api_functions
 *  @brief  Tsip_createPort initializes individual ports of the TSIP driver
 *
 *  @details This function initializes the TSIP driver specific to individual TSIP ports
 *           based on user configuration
 *
 *  @param[in]  handle      The TSIP LLD instance identifier
 *  @param[in,out]  cfg     Configuration information: out for cfg->hwId, and in for others
 *  @param[in]  bases       Array of the memory buffer base addresses 
 *  @param[out] portHandle  Instance handle for the TSIP port. This is a pointer to an initialized
 *                          instance structure. 
 *  @retval                 Value @ref ReturnValues
 *  @pre                    A driver instance must be created via calling Tsip_createShared()
 */
tsipReturn_t Tsip_createPort (Tsip_DrvHandle handle, tsipConfig_t *cfg, void* bases[], Tsip_PortHandle *portHandle);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief  Tsip_enablePortChannel Enables Transmit and Receive channels on a TSIP port
 *
 *  @details TSIP transmit and receive channel is enabled for a TSIP port
 *
 *  @param[in]  handle      The TSIP LLD instance identifier
 *  @param[in]  portHandle  Instance handle for the TSIP port 
 *  @param[in]  txCh        Transmit DMA channel number 
 *  @param[in]  rxCh        Receive  DMA channel number 
 *  @retval                 Value @ref ReturnValues
 *  @pre                    A driver and port instance must be created
 */
tsipReturn_t Tsip_enablePortChannel (Tsip_DrvHandle handle, Tsip_PortHandle portHandle,int16_t txCh, int16_t rxCh);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_closeShared decativates the shared portion of the TSIP driver instance
 *
 *  @details This function deactivates the TSIP driver instance common to all the TSIP ports, 
 *           all the aoociated memory buffers can be freed after this call. 
 *
 *  @param[in]  handle  The TSIP LLD instance identifier
 *  @param[out] bases   Array of the memory buffer base addresses 
 *  @retval             Value @ref ReturnValues
 */
tsipReturn_t Tsip_closeShared (Tsip_DrvHandle handle, void* bases[]);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_closePort decativates the per port portion of the TSIP driver instance
 *
 *  @details This function deactivates the TSIP driver instance specific to individual TSIP ports, 
 *           all the aoociated memory buffers can be freed after this call. 
 *
 *  @param[in]  portHandle  Handle for the TSIP port
 *  @param[out] bases       Array of the memory buffer base addresses 
 *  @retval                 Value @ref ReturnValues
 */
tsipReturn_t Tsip_closePort (Tsip_PortHandle portHandle, void* bases[]);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_timeslotConfig performs run-time configuration change of timeslots
 *  @details  This function is used to run-time configure a TSIP timeslot. 
 *            New configuration parameters are passed in through the control structure.
 *
 *  @param[in]     handle     The driver instance handle
 *  @param[in, out]   ctl     The run-time control structure: out for ctl->txTsContext, in for others
 *  @retval                   Value @ref ReturnValues
 *  @pre                      A driver and port instance must be created
 */
tsipReturn_t Tsip_timeslotConfig(Tsip_DrvHandle handle, tsipTsControl_t *ctl);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_getCallout queries the TSIP timeslot callout along with the context
 *  @details  This function is used to get the current TSIP timeslot callout function along with the context
 *            used by the callout
 *
 *  @param[in]     handle     The driver instance handle
 *  @param[in,out]    ctl     The run-time control structure
 *  @retval                   Value @ref ReturnValues
 *  @pre                      A driver and port instance must be created, timeslot configuration must be done
 */
tsipReturn_t Tsip_getCallout(Tsip_DrvHandle handle, tsipTsControl_t *ctl);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_setCallout changes the TSIP timeslot callout and its context
 *  @details  This function is used to change the TSIP timeslot callout function as well as the context
 *            used by the callout
 *
 *  @param[in]     handle     The driver instance handle
 *  @param[in]     ctl        The run-time control structure
 *  @retval                   Value @ref ReturnValues
 *  @pre                      A driver and port instance must be created, timeslot configuration must be done
 */
tsipReturn_t Tsip_setCallout(Tsip_DrvHandle handle, tsipTsControl_t *ctl);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_configPhase configures the desired stagger phase of a TSIP timeslot
 *  @details  This function is used to configure the desired stagger phase of a TSIP timeslot
 *
 *  @param[in]     handle     The driver instance handle
 *  @param[in]     ctl        The run-time control structure
 *  @retval                   Value @ref ReturnValues
 *  @pre                      A driver and port instance must be created, timeslot configuration must be done
 */
tsipReturn_t Tsip_configPhase(Tsip_DrvHandle handle, tsipTsControl_t *ctl);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief TSIP_getTxDmaPos gets Tx DMA position
 *  @details  This function is used to get the position of transmit DMA position, 
              i.e., the index of the last consumed sample in DMA buffer
 *
 *  @param[in]     cxt        Timeslot context pointer which is returned after calling Tsip_timeslotConfig()
 *  @retval                   The index of the last consumed sample in the DMA buffer
 *  @pre                      A driver and port instance must be created, timeslot configuration must be done
 */
int16_t Tsip_getTxDmaPos(void *cxt);

/**
 *  @ingroup tsiplld_api_functions
 *  @brief Tsip_superFrameIsr is the ISR function for super-frame interrupts
 *  @details  This function is used as the ISR function for super-frame interrupts
 *
 *  @param[in]     portHandle The TSIP port instance handle
 *  @retval                   Value @ref ReturnValues
 *  @pre                      A driver instance must be created
 */
void Tsip_superFrameIsr (Tsip_PortHandle portHandle);


/**
 *  @ingroup tsiplld_api_functions
 *  @brief  Tsip_getVersion returns the TSIP LLD version information
 *
 *  @details This function is used to get the version information of the TSIP LLD in 0xAABBCCDD format.
 *           where Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 *
 *  @retval                32-bit version information
 */
uint32_t Tsip_getVersion (void);


/**
 *  @ingroup tsiplld_api_functions
 *  @brief  Tsip_getVersionStr returns the TSIP LLD version string
 *
 *  @details This function is used to get the version string of the TSIP LLD.
 *
 *  @retval                Version string
 */
const char* Tsip_getVersionStr (void);

#ifdef __cplusplus
}
#endif
  

#endif  /* _TSIP_H */
