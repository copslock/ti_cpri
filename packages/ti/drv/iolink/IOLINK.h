/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** ============================================================================
 *  @file       IOLINK.h
 *
 *  @brief      IOLINK driver interface
 *
 *  The IOLINK header file should be included in an application as follows:
 *  @code
 *  #include <ti/drv/iolink/IOLINK.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef IOLINK_H
#define IOLINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** ===========================================================================
 *
 * @defgroup IOLINK_API IOLINK API
 * @ingroup IOLINK_API
 *
 * ============================================================================
 */
/**
@defgroup IOLINK_DATASTRUCT  IOLINK Data Structures
@ingroup IOLINK_API
*/
/**
@defgroup IOLINK_FUNCTION  IOLINK Functions
@ingroup IOLINK_API
*/
/**
@defgroup IOLINK_ENUM IOLINK Enumerated Data Types
@ingroup IOLINK_API
*/

/** ===========================================================================
 *  @addtogroup IOLINK_ENUM
    @{
 * ============================================================================
 */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible interrupt types. Each source
 *        interrupt is either an active high level or active high pulse.
 *
 * ----------------------------------------------------------------------------
 */
typedef int32_t IOLINK_STATUS;
#define IOLINK_STATUS_SUCCESS ((int32_t)(0))   /** Successful status code returned by IOLINK APIs */
#define IOLINK_STATUS_ERROR   ((int32_t)(-1))  /** Error status code returned by IOLINK APIs */

/* @} */

/**
 *  \addtogroup IOLINK_DATASTRUCT
 *  @{
 */

/*!
 *  @brief    Basic IOLINK Parameters
 */
typedef struct IOLINK_Params_s
{
    /*! Delay in Tbit between the end of the stop bit of last UART frame
     *  being received and the beginning of the start bit of the
     *  first UART frame being sent. */
    uint32_t       tA;

} IOLINK_Params;


/*!
 *  @brief      A handle that is returned from a IOLINK_open() call.
 */
typedef struct IOLINK_Config_s *IOLINK_Handle;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              IOLINK_init().
 */
typedef void (*IOLINK_InitFxn)(IOLINK_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              IOLINK_OpenFxn().
 */
typedef IOLINK_Handle (*IOLINK_OpenFxn)(IOLINK_Handle        handle,
                                        const IOLINK_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              IOLINK_CloseFxn().
 */
typedef IOLINK_STATUS (*IOLINK_CloseFxn) (IOLINK_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              IOLINK_ControlFxn().
 */
typedef IOLINK_STATUS (*IOLINK_ControlFxn)(IOLINK_Handle handle,
                                           uint32_t      cmd,
                                           void         *arg);

/*!
 *  @brief      The definition of a IOLINK function table that contains the
 *              required set of functions to control a specific IOLINK driver
 *              implementation.
 */
typedef struct IOLINK_FxnTable_s {
    /*! Function to initialize the given data object */
    IOLINK_InitFxn         initFxn;

    /*! Function to open the specified instance */
    IOLINK_OpenFxn         openFxn;

    /*! Function to close the specified instance */
    IOLINK_CloseFxn        closeFxn;

    /*! Function to implementation specific control function */
    IOLINK_ControlFxn      controlFxn;

} IOLINK_FxnTable;

/*!
 *  @brief  IOLINK Global configuration
 *
 *  The IOLINK_Config structure contains a set of pointers used to characterize
 *  the IOLINK driver implementation.
 *
 *  This structure needs to be defined before calling IOLINK_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     IOLINK_init()
 */
typedef struct IOLINK_Config_s {
    /*! Pointer to a table of driver-specific implementations of IOLINK APIs */
    IOLINK_FxnTable const *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    void                  *object;

    /*! Pointer to a driver specific SW or HW IP attributes structure */
    void const            *ipAttrs;
} IOLINK_Config;

#define IOLINK_MAX_CONFIG_CNT (2U)
typedef IOLINK_Config IOLINK_config_list[IOLINK_MAX_CONFIG_CNT];


/* @} */


/**
 *  \addtogroup IOLINK_FUNCTION
 *  @{
 */

/*!
 *  @brief  Function to initializes the IOLINK module
 *
 *  @pre    The IOLINK_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other IOLINK driver APIs.
 *
 *  @return none.
 */
extern void IOLINK_init(void);

/*!
 *  @brief  Function to initialize a given IOLINK instance specified by the
 *          particular index value. The parameter specifies which mode the IOLINK
 *          will operate.
 *
 *  @pre    IOLINK controller has been initialized
 *
 *  @param  index         Logical instance number for the IOLINK instance indexed
 *                        into the IOLINK_config table
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A IOLINK_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     IOLINK_init()
 *  @sa     IOLINK_close()
 */
extern IOLINK_Handle IOLINK_open(uint32_t index, IOLINK_Params *params);

/*!
 *  @brief  Function to close an IOLINK instance specified by the IOLINK handle
 *
 *  @pre    IOLINK_open() had to be called first.
 *
 *  @param  handle  A IOLINK_Handle returned from IOLINK_open
 *
 *  @return IOLINK_STATUS_SUCCESS on success or IOLINK_STATUS_ERROR on error.
 *
 *  @sa     IOLINK_open()
 */
extern IOLINK_STATUS IOLINK_close(IOLINK_Handle handle);

/*!
 *  @brief  Function performs specific controls on a given IOLINK_Handle.
 *
 *  @pre    IOLINK_open() has to be called first.
 *
 *  @param  handle      A IOLINK handle returned from IOLINK_open()
 *
 *  @param  cmd         A control command value defined by the driver specific
 *                      implementation
 *
 *  @param  arg         An optional R/W (read/write) argument that is
 *                      accompanied with cmd
 *
 *  @return IOLINK_STATUS_SUCCESS on success or IOLINK_STATUS_ERROR on error.
 *
 *  @sa     IOLINK_open()
 */
extern IOLINK_STATUS IOLINK_control(IOLINK_Handle handle, uint32_t cmd, void *arg);

/*!
 *  @brief  Function to initialize the IOLINK_Params struct to its defaults
 *
 *  @param  params      An pointer to IOLINK_Params structure for
 *                      initialization
 *
 *  @return none
 */
extern void IOLINK_Params_init(IOLINK_Params *params);

/* @} */

#ifdef __cplusplus
}
#endif

#endif /* _IOLINK_H_ */
