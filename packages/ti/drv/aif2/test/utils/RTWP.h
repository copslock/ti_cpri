/*
 * RTWP.h
 *
 * API for reading RTWP measurements in the SmallCell WCDMA stack.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef RTWP_H
#define RTWP_H

#include <inttypes.h>

#ifdef __cplusplus 
/* C++ compatibility */
extern "C" {
#endif

/*
* uint32_t RTWP_open(uint32_t cpri_link_nbr, float rf_rx_gain1, float rf_rx_gain2);
*
* DESCRIPTION
*
* The PHY layer must call this function when it is configured/started. This function
* responsible to perform all one time initializations and resource allocations before
* the RTWP_read() API can be called safely.
*
* PARAMETERS
*	* cpri_link_nbr
*	   Used cpri link number. 
*
*   * rf_rx_gain1, rf_rx_gain2 
*      These gain parameters are used by RTWP_read() API calls to convert the raw readings
*      of RTWP measurements into mW, numbers refer to antenna number.
*
* RETURN VALUE
*
* The return value is an error code:
*
*   * return value of 0 means the API call was successful.
*   * return value > 0 means an error occurred.
*
*/
 uint32_t RTWP_open(uint32_t cpri_link_nbr, float rf_rx_gain1, float rf_rx_gain2); 

/*
* uint32_t RTWP_read( uint32_t link_nbr_rtwp_address, float *result_mW, float *result_raw);
*
* DESCRIPTION
*
* This API call returns the RTWP measurement for one antenna carrier. 
 *
* PARAMETERS
*
*   * link_nbr_rtwp_address
*      This API parameter is retrieved by the PHY layer from the content
*      of the array provided in the BOWcdmaCell attribute:
*
*         arrRtwpFpgaAddr[0..1].
*
*	   and combined with CPRI link number in 4 MSB
*
* 	   ie. RTWP_read( (cpri_link_nbr<<28 | cellPtr->rxAntConfig[j].rtwpAddr), &rtwpMW, &rtwpRaw)
* 
*      Please refer to the "B-OAM WCDMA Object Model", authored by Nash,
*      for more details about these parameters.
*
*   * result_mW
*
*     When no error occurs the RTWP power measurement (expressed in mW) is
*     written at the memory location pointed by the result_mW parameter.
*
*     The result_mW is derived from the result_raw value by applying a formula
*     similar to the following:
*
*       result_mW = (some value derived from result_raw) * rf_rx_gain;
*
*     Nothing is written to this memory location in case of error.
*
*   * result_raw
*
*     When no error occurs the raw value of the measurement, as read from the
*     hardware/FPGA registers, is written at the memory location pointed by
*     this pointer.
*
*     Nothing is written to this memory location in case of error.
*
* RETURN VALUE
*
* The return value is an error code:
*
*   * return value of 0 means the API call was successful.
*   * return value > 0 means an error occurred.
*   * return value of 1 means that the rtwp_address is invalid.
*
* SPECIAL NOTE
*
* It is the responsibility of this API to detect and prevent the use
* of invalid rtwp_address values. This is needed to prevent accidental
* or intentional misuse of this parameter and will be helpful in
* troubleshooting OAM configuration errors.
*
* The implementation of this API being board/hardware specific it
* should restrict the range of acceptable addresses to those actually
* supported by the hardware platform.
*
* The code invoking this API should explicitely check for a return code of 1
* and report it as a configuration error.
*
*/
uint32_t RTWP_read( uint32_t link_nbr_rtwp_address, float *result_mW, float *result_raw);

/*
* uint32_t RTWP_close();
*
* DESCRIPTION
*
* This API must be called by the PHY when the PHY layer stopped. It is responsible to free
* the resources used for RTWP measurement reads, and reset the configuration in order to
* allow a possible future restart of the RTWP open/read+/close cycle.
*
* PARAMETERS
*
*   None
*
* RETURN VALUE
*
* The return value is an error code:
*
*   * return value of 0 means the API call was successful.
*   * return value > 0 means an error occurred.
*
*/
uint32_t RTWP_close();

#ifdef __cplusplus 
}
#endif

#endif /* RTWP_H */
