/*
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/**
 *   @file  srio_tput.h
 *
 *   @brief
 *      Header file for the tput utility functions
 *
*/

#ifndef __SRIO_TPUT_H__
#define __SRIO_TPUT_H__

#include <ti/drv/srio/srio_drv.h>
#include <ti/drv/srio/srio_types.h>

/* Application Include Files */
#include <srio_laneconfig.h>
#include <benchmarking.h>

/* This is the Number of host descriptors which are available & configured
 * in the memory region for this test. */
#define NUM_HOST_DESC                                  128

/* This is the size of each descriptor. */
#define SIZE_HOST_DESC                                 64

/* This is the number of RX descriptors to use */
#define NUM_RX_DESC                                    32
#define NUM_RX_DESC_ON_TX_SIDE                         1

/* This is the number of TX descriptors to use */
#define NUM_TX_DESC                                    32
#define NUM_TX_DESC_ON_RX_SIDE                         1

/* Round trip or one way designators for
 * latency statistics */
#define ROUND_TRIP_TIME                                TRUE
#define ONE_WAY_TIME                                   FALSE

/* Maximum attempts to try in order to find
 * a good pacing value before terminating */
#define MAX_TX_FAIL_ATTEMPTS                           100

/* Minimum number of packets to send in order
 * to correctly calculate the number of packets
 * to use for a given amount of seconds. */
#define MIN_NUM_PACKETS_TO_SEND                        200000

/* Our software header byte count for Type-11.
 * This will be part of the payload. */
#define SOFTWARE_HEADER_BYTES                          12

/* Bytes of overhead for specific packet types */
#define TYPE_11_MESSAGE_OVERHEAD_BYTES_PER_256B        24
#define DIO_NWRITE_OVERHEAD_BYTES_PER_256B             16
#define DIO_NWRITER_OVERHEAD_BYTES_PER_256B            28
#define DIO_NREAD_OVERHEAD_BYTES_PER_256B              28

typedef int bool;

/**************************************************************************\
* Test Control Variables Structure
\**************************************************************************/
typedef struct {
	srioLaneRateGbps_e	srio_laneSpeedGbps;
	srioLanesMode_e		srio_lanesMode;
	srioTestsToRun_e	srio_testsToRun;
	uint32_t			srio_testTimeInSeconds;
	int32_t				srio_initCorenum;
	bool				srio_isBoardToBoard;
	bool				srio_isExternalSrioSwitch;
	bool				srio_isLoopbackMode;
	bool				srio_isDeviceID16Bit;
	bool				srio_isLatencyTest;
	bool				srio_isRunProgress;
	int32_t				srio_payloadSize;
	int32_t				srio_payloadEndSize;
	int32_t				srio_dioPayloadSize;
	int32_t				srio_dioPayloadEndSize;
} struct_testcontrol;

/**************************************************************************\
* Throughput Calculations Parameter Pass Structure
\**************************************************************************/
typedef struct  {
    uint64_t	numberOfPackets;
    uint32_t	packetSizeBytes;
    uint32_t	overheadBytes;
    uint64_t	tsLoopStart;
    uint64_t	tsLoopEnd;
    uint64_t	totalLldCycles;
    uint64_t	totalProcCycles;
    uint64_t	totalIdleCycles;
    uint32_t	pacingCycles;
    uint32_t	minLatCycles;
    uint32_t	maxLatCycles;
    uint32_t	fType;
    uint32_t	tType;
    bool	isShowHeader;
    bool	isTestInfoOnly;
} Tput_Parameters;

/**************************************************************************\
* Binary Search Structure
\**************************************************************************/
typedef struct  {
    uint32_t	currMin;
    uint32_t	currMax;
    uint32_t	curr;
    uint32_t	knownGood;
    uint32_t	unstableCount;
    bool	isDelayWorking;
} Tput_BinSearch;

/**********************************************************************
 ******************** srio_type11_tput EXPORTED APIs ********************
 **********************************************************************/
extern Int32 consumerType11LatencyTest (void);
extern Int32 producerType11LatencyTest (void);
extern Int32 consumerType11Throughput (void);
extern Int32 producerType11Throughput (void);

/**********************************************************************
 ********************* srio_dio_tput EXPORTED APIs **********************
 **********************************************************************/
extern void myDIOIsr(UArg argument);
extern Srio_SockHandle createMgmtSocket(void);
extern Int32 consumerDIONReadLatencyTest(void);
extern Int32 producerDIONReadLatencyTest(void);
extern Int32 consumerDIONReadThroughput(void);
extern Int32 producerDIONReadThroughput(void);
extern Int32 consumerDIOLatencyTest(void);
extern Int32 producerDIOLatencyTest(void);
extern Int32 consumerDIONWriteThroughput(void);
extern Int32 producerDIONWriteThroughput(void);

/**********************************************************************
 ********************** tput_utils EXPORTED APIs **********************
 **********************************************************************/
extern void clearSrioStatsOutputBuffer (void);
extern void displayStatsOutputBuffer (void);
extern uint32_t getIterationCountUsingSeconds (uint64_t tsLoopStart, uint64_t tsLoopEnd, uint32_t timeInSeconds);
extern void displayTputStatistics (Tput_Parameters* tparams);
extern void displayLatencyStatistics (Tput_Parameters* tparams, bool isRoundTripTime);
extern char* uint64ToString (uint64_t longLongUnsigned);
extern void debugPrintf (char* string);
extern void debugPrintf_d (char* string, Uint32 numVal1);
extern void debugPrintf_dd (char* string, Uint32 numVal1, Uint32 numVal2);
extern void debugPrintf_ddd (char* string, Uint32 numVal1, Uint32 numVal2, Uint32 numVal3);
extern void debugPrintf_p (char* string, void* numVal1);
extern void debugPrintf_pp (char* string, void* numVal1, void* numVal2);
extern void debugPrintf_sdd (char* string, char* stringVal1, Uint32 numVal1, Uint32 numVal2);
extern void initPacingSearchValues (Tput_BinSearch* ptrPacing, uint32_t maxCycles);
extern void getPacingBisect (Tput_BinSearch* ptrPacing);
extern int32_t displaySrioStatus (void);
extern void cycleDelay (uint64_t cyclesToDelay);
extern int32_t setStartParameters (int argc, char* argv[], uint32_t coreNum);
extern int32_t setBiosCpuFrequencyIndicatorIfNeeded (void);
/**********************************************************************
 ****************** srio_status_decode EXPORTED APIs ******************
 **********************************************************************/
extern int32_t displaySrioLTLEDCSRErrorStatus (void);
extern int32_t displayPortResponseTimeout (void);
extern int32_t displayPortGeneralCSR (void);
extern int32_t displayVariousRegisters (void);
extern char* controlDecodeText (uint32_t controlCode);
extern char* testsToRunDecodeText (uint32_t testToRunCode);
extern void clearSrioStatusErrors (void);

#endif /* __SRIO_TPUT_H__ */

