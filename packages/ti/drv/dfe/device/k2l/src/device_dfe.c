/**
 *   @file  device_dfe.c
 *
 *   @brief   
 *      The DFE Device specific code. The DFE LLD calls out
 *      this code to initialize the DFE IP block. The file is provided as 
 *      a sample configuration and should be modified by customers for 
 *      their own platforms and configurations.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012-2013 Texas Instruments, Inc.
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
 *  \par
 */

/* DFE LLD Includes. */
#include <ti/drv/dfe/dfe_types.h>
#include <ti/drv/dfe/dfe_drv.h>

/* CSL DFE Functional Layer */
#include <ti/csl/csl_dfe.h>

/**********************************************************************
 ************************* LOCAL Definitions **************************
 **********************************************************************/


/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/


/**********************************************************************
 *********************** DEVICE DFE FUNCTIONS ***********************
 **********************************************************************/

/** @addtogroup DFE_DEVICE_API
 @{ */

/**
 *  @b Description
 *  @n  
 *      The function provides the initialization sequence for the DFE IP
 *      block. This can be modified by customers for their application and
 *      configuration.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
#pragma CODE_SECTION(Iqn2Device_init, ".text:Iqn2Device_init");
int32_t Iqn2Device_init (void)
{
    CSL_Iqn2Handle      hIqn2;

    /* Get the CSL DFE Handle. */
    hIqn2 = CSL_dfeOpen (0);
    if (hIqn2 == NULL)
        return -1;

    /* Initialization has been completed. */
    return 0;
}

/**
@}
*/
