/******************************************************************************
 * FILE PURPOSE:  Header file for static Resource Manager used by NWAL test
 *                for ARM and DSP
 ******************************************************************************
 * FILE NAME:   fw_rm.h
 *
 * DESCRIPTION: Header file for unit test package
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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

#ifndef _FW_RM_H
#define _FW_RM_H
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>



/* UDP Ports being used by Test programs */
#define TEST_NWAL_BASE_UDP_PORT             0x0555  /* Base UDP port for a group
                                                     * of cores sharing same NWAL
                                                     * instance. Each core will 
                                                     * configure base + proc #
                                                     */
#define TEST_NWAL_BASE_REM_UDP_PORT         0x0655  /* Base UDP port configured
                                                     * for remote cores
                                                     * by master core. All cores 
                                                     * share same NWAL
                                                     * instance
                                                     */
#define TEST_NWAL_BASE_AH_UDP_PORT          0x0755  /* Base IPSEC AH UDP port 
                                                     * for a group
                                                     * of cores sharing same NWAL
                                                     * instance. Each core will 
                                                     * configure base + proc #
                                                     */
#define TEST_NWAL_BASE_IPV6_ESP_UDP_PORT    0x0855  /* Base IPv6/IPSEC ESP UDP port 
                                                     * for a group
                                                     * of cores sharing same NWAL
                                                     * instance. Each core will 
                                                     * configure base + proc #
                                                     */
#define TEST_NWAL_BASE_IPV6_AH_UDP_PORT     0x0955  /* Base IPv6/IPSEC AH UDP port 
                                                     * for a group
                                                     * of cores sharing same NWAL
                                                     * instance. Each core will 
                                                     * configure base + proc #
                                                     */
#define TEST_NWAL_BASE_ESP_TRANS_UDP_PORT   0x0A55  /* Base ESP Transport UDP port 
                                                     * for a group
                                                     * of cores sharing same NWAL
                                                     * instance. Each core will 
                                                     * configure base + proc #
                                                     */
#define TEST_NWAL_BASE_AH_TRANS_UDP_PORT    0x0B55  /* Base AH Transport UDP port 
                                                     * for a group
                                                     * of cores sharing same NWAL
                                                     * instance. Each core will 
                                                     * configure base + proc #
                                                     */

/* Number of remote Fast Path C66x cores terminating packets and
 * looping back
 */
#define CPU_NUM_REM_FAST_PATH_CORES         8
#define TEST_NWAL_BASE_REM_FP_UDP_PORT      0x0C55  /* Base UDP port 
                                                     * for a group
                                                     * of remote Fast Path cores 
                                                     * The remote cores will be 
                                                     * just terminating the packets
                                                     */
#define TEST_NWAL_BASE_REM_FP_RX_PKT_QUEUE  2000    /* Base Queue for first remote
                                                     * FP core terminating 
                                                     * packets
                                                     */
                                                    

/* APP ID being used by NWAL Tests */
#define TEST_NWAL_APP_ID_DEFAULT            0xABCDEFAB
#define TEST_NWAL_APP_ID_MAC                0xFFFFFFFF
#define TEST_NWAL_APP_ID_IP                 0xFFFFFFFE
#define TEST_NWAL_APP_ID_DM_ENC             0xFFFFFFFD
#define TEST_NWAL_APP_ID_DM_DEC             0xFFFFFFFC
#define TEST_NWAL_APP_ID_EXCEPTION          0xEEEEEEEE
#define TEST_NWAL_CTX_ID_DM_ENC             0xFFFFFFFD
#define TEST_NWAL_CTX_ID_DM_DEC             0xFFFFFFFC

#endif /* _FW_RM_H */



