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
 *   @file  tput_utils.c
 *
 *   @brief
 *      Functions and support functions to calculate and display throughput and latency statistics.
 *
 */

/* Standard includes */
#include "stdio.h"
#include "string.h"
#include "math.h"

/* SRIO Driver Includes. */
#include <ti/drv/srio/srio_drv.h>

/* Application Include Files */
#include <srio_laneconfig.h>
#include <srio_tput.h>

/* Runtime and Sysbios includes */
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h> //new for calculations
#include <xdc/runtime/Types.h> //new for calculations
#include <ti/sysbios/BIOS.h>

/* CSL Includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/cslr_pllc.h>

/* Producer-Consumer Include Files. */
#include <benchmarking.h>

/* Time stamp includes */
#include <ti/csl/csl_tsc.h>

/* Print in space Screen Field Size (SFSIZE) */
#define	SFSIZE_1	4
#define	SFSIZE_2	6
#define	SFSIZE_3	12
#define	SFSIZE_4	12
#define	SFSIZE_5	12
#define	SFSIZE_6	14
#define	SFSIZE_7	9
#define	SFSIZE_8	10
#define	SFSIZE_9	10
#define	SFSIZE_10	10

/* String Output defines */
#define MAX_SHORT_LEN 25
#define MAX_MSG_LEN 50
#define OF_LF00 "%0.0Lf"
#define OF_LF02 "%0.2Lf"

/* The number of status buffers needed. Should be one for each core. If core 0 and 6 are the
 * cores being used then this number must be 7 to cover the buffer indexing for cores 0 thru 6.
 */
#define NUM_SHARED_STATS_BUFFERS	2

/* Status Output defines */
#define STAT_LINE_LEN 256
#define NUM_PKT_SIZES 12
#define LANE_SPEEDS   1
#define LANE_CONFIGS  1

/* Convert cycles to seconds */
#define CYCLES_TO_SECS(CYCLES, CPUFREQ) \
    ((float)((((Double)CYCLES * 1))/(Double)CPUFREQ))

/* Convert cycles to milliseconds */
#define CYCLES_TO_MS(CYCLES, CPUFREQ) \
    ((UInt32)((((Double)CYCLES * 1000))/(Double)CPUFREQ))

/* Convert cycles to microseconds */
#define CYCLES_TO_US(CYCLES, CPUFREQ) \
    ((UInt32)((((Double)CYCLES * 1000000))/(Double)CPUFREQ))

/* Convert cycles to nanoseconds */
#define CYCLES_TO_NS(CYCLES, CPUFREQ) \
    ((UInt32)((((Double)CYCLES * 1000000000))/(Double)CPUFREQ))

#pragma DATA_SECTION (srioStatsOutputBuffer, ".srioSharedMem");
static char	srioStatsOutputBuffer[NUM_SHARED_STATS_BUFFERS][STAT_LINE_LEN * NUM_PKT_SIZES * LANE_SPEEDS * LANE_CONFIGS];

bool isDisplayTxNow = FALSE;

/**********************************************************************
 ************************ External Definitions ************************
 **********************************************************************/

/* Variables to control test run. */
extern struct_testcontrol testControl;


/* Defines for debug use only */
//#define TPUT_DEBUG
//#define DEBUG_PRINT
//#define DEBUG_PRINT_SP

/**
 *  @b Description
 *  @n
 *      The function is used display a string only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *
 *  @retval
 *      None.
 */
void debugPrintf (char* string)
{
#ifdef DEBUG_PRINT
	System_printf (string);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used display a string with a Uint32 value only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *  @param[in]  numVal1
 *      Unsigned 32-bit number to print.
 *
 *  @retval
 *      None.
 */
void debugPrintf_d (char* string, Uint32 numVal1)
{
#ifdef DEBUG_PRINT
	System_printf (string, numVal1);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used display a string with a pointer value only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *  @param[in]  numVal1
 *      Pointer to print.
 *
 *  @retval
 *      None.
 */
void debugPrintf_p (char* string, void* numVal1)
{
#ifdef DEBUG_PRINT
	System_printf (string, numVal1);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used display a string with a pointer value only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *  @param[in]  numVal1
 *      Pointer to print.
 *  @param[in]  numVal2
 *      Pointer to print.
 *
 *  @retval
 *      None.
 */
void debugPrintf_pp (char* string, void* numVal1, void* numVal2)
{
#ifdef DEBUG_PRINT_SP
	System_printf (string, numVal1, numVal2);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used display a string with two Uint32 values only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *  @param[in]  numVal1
 *      Unsigned 32-bit number to print.
 *  @param[in]  numVal2
 *      Unsigned 32-bit number to print.
 *
 *  @retval
 *      None.
 */
void debugPrintf_dd (char* string, Uint32 numVal1, Uint32 numVal2)
{
#ifdef DEBUG_PRINT
	System_printf (string, numVal1, numVal2);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used display a string with two Uint32 values only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *  @param[in]  numVal1
 *      Unsigned 32-bit number to print.
 *  @param[in]  numVal2
 *      Unsigned 32-bit number to print.
 *  @param[in]  numVal3
 *      Unsigned 32-bit number to print.
 *
 *  @retval
 *      None.
 */
void debugPrintf_ddd (char* string, Uint32 numVal1, Uint32 numVal2, Uint32 numVal3)
{
#ifdef DEBUG_PRINT_SP
	System_printf (string, numVal1, numVal2, numVal3);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used display a string with a string and two Uint32 values only when DEBUG_PRINT is defined.
 *
 *  @param[in]  string
 *      String containing the text to be displayed.
 *  @param[in]  stringVal1
 *      Substring containing the text to be displayed.
 *  @param[in]  numVal1
 *      Unsigned 32-bit number to print.
 *  @param[in]  numVal2
 *      Unsigned 32-bit number to print.
 *  @retval
 *      None.
 */
void debugPrintf_sdd (char* string, char* stringVal1, Uint32 numVal1, Uint32 numVal2)
{
#ifdef DEBUG_PRINT
	System_printf (string, stringVal1, numVal1, numVal2);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to convert a float to a string. Needed because
 *      System_printf () does not support the floating point format.
 *
 *  @param[in]  floatNum
 *      Number to convert
 *  @param[in]  outFormat
 *      Floating point format
 *
 *  @retval
 *      Pointer to the string containing the converted number
 */
static char* floatToString (float floatNum, char* outFormat)
{
	static char		textBuffer[40];
	sprintf (textBuffer, outFormat, floatNum);
	return textBuffer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to convert a long long uint to a string.
 *      Needed because System_printf () does not support displaying a
 *      long long uint.
 *
 *  @param[in]  longLongUnsigned
 *      Number to convert
 *
 *  @retval
 *      Pointer to the string containing the converted number
 */
char* uint64ToString (uint64_t longLongUnsigned)
{
	static char		textBuffer[40];
	sprintf (textBuffer, "%llu", longLongUnsigned);
	return textBuffer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to clear the srio stats output buffer.
 *
 *  @retval
 *      None.
 */
void clearSrioStatsOutputBuffer (void)
{
	uint32_t		coreNumber;

	/* Get the core number. */
    coreNumber = CSL_chipReadReg (CSL_CHIP_DNUM);

	strcpy (srioStatsOutputBuffer[coreNumber], "");
}

/**
 *  @b Description
 *  @n
 *      The function determines save state need depending on the run progress flag.
 *
 *  @param[in]  cyclesToDelay
 *      Number of cycles to wait.
 *
 *  @retval
 *      Not Applicable.
 */
static bool getDisplayState(Bool isDisplayRunProgress)
{
	bool isDisplayNow = FALSE;
	uint32_t		coreNumber;

	/* Get the core number. */
    coreNumber = CSL_chipReadReg (CSL_CHIP_DNUM);

    if (!isDisplayRunProgress)
    {
    	if (testControl.srio_isBoardToBoard)
    	{
    		/* If is board to board and progress messages are off then displays statistics on both RX
    		 * and TX instead of saving them for displaying later. */
    		isDisplayNow = TRUE;
    	}
    	else
    	{
    		/* if is core to core and progress messages are off then display statistics on the RX side
    		 * instead of saving them for displaying later. TX side will be saved and displayed after.
    		 */
    		if (coreNumber == CONSUMER_CORE)
    			isDisplayNow = TRUE;
    		else
    		{
    	    	/* Always display when displaying latency, since this is a TX side only measurement */
    	    	if (isDisplayTxNow)
    				isDisplayNow = TRUE;
    		}
    	}
    }

    return isDisplayNow;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a string to the srio stats output buffer.
 *
 *  @param[in]  textString
 *      String containing the text to be added.
 *  @param[in]  separator
 *      String containing the separator to be added.
 *
 *  @retval
 *      None.
 */
static void addStringToStatsOutputBuffer (char* textString, char* separator)
{
	uint32_t		coreNumber;
	bool			is_DisplayNow = getDisplayState(testControl.srio_isRunProgress);

	/* Get the core number. */
    coreNumber = CSL_chipReadReg (CSL_CHIP_DNUM);

	if (!is_DisplayNow)
    {
		strcat (srioStatsOutputBuffer[coreNumber], textString);
		strcat (srioStatsOutputBuffer[coreNumber], separator);
    }
	else
		System_printf("%s%s", textString, separator);
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a decimal number to the srio stats output buffer.
 *
 *  @param[in]  number
 *      32-bit number to be added.
 *  @param[in]  separator
 *      String containing the separator to be added.
 *
 *  @retval
 *      None.
 */
static void addDecimalToStatsOutputBuffer (Uint32 number, char* separator)
{
	static char	numToText[MAX_SHORT_LEN];

	sprintf (numToText,"%d",number);
	addStringToStatsOutputBuffer (numToText, separator);
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a decimal number to the srio stats output buffer.
 *
 *  @param[in]  number
 *      32-bit number to be added.
 *  @param[in]  separator
 *      String containing the separator to be added.
 *
 *  @retval
 *      None.
 */
static void addLongToStatsOutputBuffer (Uint32 number, char* separator)
{
	static char	numToText[40];

	sprintf (numToText,"%lu",number);
	addStringToStatsOutputBuffer (numToText, separator);
}

/**
 *  @b Description
 *  @n
 *      The function is used to print text within a specified number of spaces by padding with spaces.
 *
 *  @param[in]  textString
 *      String containing the text to be printed.
 *  @param[in]  numSpaces
 *      Number of spaces it should take up.
 *
 *  @retval
 *      None.
 */
static void printInSpaceText (char* textString, Int8 numSpaces)
{
	Int8	spacesToPrint;
	Int8	spacesCount;

	spacesToPrint = numSpaces - strlen(textString);
	if (spacesToPrint < 0)
		spacesToPrint = 1;
	System_printf ("%s",textString);
    for (spacesCount = 0; spacesCount < spacesToPrint; spacesCount++)
    	System_printf (" ");
}

/**
 *  @b Description
 *  @n
 *      The function is used to print a decimal within a specified number of spaces by padding with spaces.
 *
 *  @param[in]  number
 *      Number to be printed.
 *  @param[in]  numSpaces
 *      Number of spaces the it should take up.
 *
 *  @retval
 *      None.
 */
static void printInSpaceDecimal (Uint32 number, Int8 numSpaces)
{
	static char	numToText[MAX_SHORT_LEN];

	sprintf (numToText,"%d",number);
	printInSpaceText (numToText, numSpaces);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the per 256 byte overhead.
 *
 *  @param[in]  packetSize
 *      Size of packet.
 *  @param[in]  packetOHbytes
 *      Srio overhead byte count.
 *
 *  @retval
 *      None.
 */
static Uint32 get256ByteOverHead (Uint32 packetSize, Uint32 packetOHbytes)
{
	Uint32	divisor = ((packetSize - 1) / 256) + 1;

	return (Uint32)(packetOHbytes / divisor);
}

/**
 *  @b Description
 *  @n
 *      The function is used to print out the entire srio stats output buffer.
 *
 *  @retval
 *      None.
 */
void displayStatsOutputBuffer (void)
{
	uint32_t		coreNumber;

	/* Get the core number. */
    coreNumber = CSL_chipReadReg (CSL_CHIP_DNUM);

	/* Add more separation between statistics on the busy display */
    if (testControl.srio_isRunProgress)
		System_printf ("\n");

    /* Display only saved output */
    if (strlen(srioStatsOutputBuffer[coreNumber]) != 0)
		System_printf ("%s\n", srioStatsOutputBuffer[coreNumber]);
    else
		System_printf ("\n");
}

/**
 *  @b Description
 *  @n
 *      The function is used to determine the connection type using
 *      various global variables.
 *
 *  @retval
 *      Pointer to the string containing the determined connection type string
 */
static char* getConnectionType (void)
{
	static char 	connType[10];
	if (testControl.srio_isLoopbackMode)
		sprintf (connType, "C-I-C");
	else
	{
		if (!testControl.srio_isBoardToBoard)
		{
			if (testControl.srio_isExternalSrioSwitch)
				sprintf (connType, "C-S-C");
			else
				sprintf (connType, "C-E-C");
		}
		else
		{
			if (testControl.srio_isExternalSrioSwitch)
				sprintf (connType, "B-S-B");
			else
				sprintf (connType, "B-E-B");
		}
	}
	return connType;
}

/**
 *  @b Description
 *  @n
 *      The function is used to calculate the number of iterations that are
 *      needed to run the test for the amount of seconds specified.
 *
 *  @param[in]  tsLoopStart
 *      Iteration start time-stamp
 *  @param[in]  tsLoopEnd
 *      Iteration end time-stamp
 *  @param[in]  timeInSeconds
 *      Length of time in seconds
 *
 *  @retval
 *      Pointer to the string containing the converted number
 */
uint32_t getIterationCountUsingSeconds (uint64_t tsLoopStart, uint64_t tsLoopEnd, uint32_t timeInSeconds)
{
	uint32_t		newIterationCount;
	uint64_t		elapsedIterationCycles;
	float			cpuTimerFreqRatio, elapsedIterationSeconds;
    Types_FreqHz	timerFreq, cpuFreq;

    Timestamp_getFreq (&timerFreq);
    BIOS_getCpuFreq (&cpuFreq);
    cpuTimerFreqRatio = (float)(cpuFreq.lo / timerFreq.lo);

    elapsedIterationCycles = tsLoopEnd - tsLoopStart;
    elapsedIterationCycles *= cpuTimerFreqRatio;
    elapsedIterationSeconds = CYCLES_TO_SECS(elapsedIterationCycles, cpuFreq.lo);

    /* Determine elapsed seconds based on a reduced precision of 2 decimal places to the right */
    newIterationCount = elapsedIterationSeconds * 100000;
    elapsedIterationSeconds = ceil((float)newIterationCount / (float)1000) / (float)100;

	newIterationCount = (uint32_t)ceil((float)timeInSeconds / (float)elapsedIterationSeconds);
	if (newIterationCount <= 0)
		newIterationCount = 1;

    //System_printf(" Calculated iteration count: %lu\n", newIterationCount);

	return newIterationCount;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode the srio lane mode.
 *
 *  @param[in]  laneMode
 *      Mode number to determine which ports to check for operational status.
 *
 *  @retval
 *      Pointer to the string containing the decoded value
 */
static char* srioLaneModeToString (srioLanesMode_e laneMode)
{
	static char		textBuffer[15];
	switch (laneMode)
	{
		case srio_lanes_form_four_1x_ports:
			sprintf (textBuffer, "%s", "1");
			break;
		case srio_lanes_form_one_2x_port_and_two_1x_ports:
			sprintf (textBuffer, "%s", "2");
			break;
		case srio_lanes_form_two_1x_ports_and_one_2x_port:
			sprintf (textBuffer, "%s", "1");
			break;
		case srio_lanes_form_two_2x_ports:
			sprintf (textBuffer, "%s", "2");
			break;
		case srio_lanes_form_one_4x_port:
			sprintf (textBuffer, "%s", "4");
			break;
		default:
			sprintf (textBuffer, "%s", "Unknown");
			break;
	}
    return textBuffer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode the srio lane speed.
 *
 *  @param[in]  laneSpeedGbps
 *      Speed number to determine lane speed.
 *
 *  @retval
 *      Pointer to the string containing the decoded value
 */
static char* srioLaneSpeedToString (srioLaneRateGbps_e laneSpeedGbps)
{
	static char		textBuffer[15];
	switch (laneSpeedGbps)
	{
		case srio_lane_rate_1p250Gbps:
			sprintf (textBuffer, "%s", "1.250");
			break;
		case srio_lane_rate_2p500Gbps:
			sprintf (textBuffer, "%s", "2.500");
			break;
		case srio_lane_rate_3p125Gbps:
			sprintf (textBuffer, "%s", "3.125");
			break;
		case srio_lane_rate_5p000Gbps:
			sprintf (textBuffer, "%s", "5.000");
			break;
		default:
			sprintf (textBuffer, "%s", "Unknown");
			break;
	}
    return textBuffer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode the srio tType.
 *
 *  @param[in]  tType
 *      Ftype to decode.
 *  @param[in]  fType
 *      Ftype to help decode tType.
 *
 *  @retval
 *      Pointer to the string containing the decoded value
 */
static char* srioTtypeToString (uint32_t tType, uint32_t fType)
{
	static char		textBuffer[20];
	switch (tType)
	{
		case Srio_Ttype_Request_NREAD:
		//case Srio_Ttype_Write_NWRITE:
			sprintf (textBuffer, "%s", ((fType == Srio_Ftype_REQUEST)? "NR" : "NW"));
			break;
		case Srio_Ttype_Write_NWRITE_R:
			sprintf (textBuffer, "%s", "NWR");
			break;
		case Srio_Ttype_Request_ATOMIC_INC:
			sprintf (textBuffer, "%s", "ATI");
			break;
		case Srio_Ttype_Request_ATOMIC_DEC:
			sprintf (textBuffer, "%s", "ATD");
			break;
		case Srio_Ttype_Request_ATOMIC_SET:
		//case Srio_Ttype_Write_ATOMIC_TEST_SET:
			sprintf (textBuffer, "%s", ((fType == Srio_Ftype_REQUEST)? "ATT" : "ATS"));
			break;
		case Srio_Ttype_Request_ATOMIC_CLR:
			sprintf (textBuffer, "%s", "ATC");
			break;
		default:
			sprintf (textBuffer, "%s", "Unknown");
			break;
	}
    return textBuffer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode the srio ftype.
 *
 *  @param[in]  fType
 *      Ftype to decode.
 *
 *  @retval
 *      Pointer to the string containing the decoded value
 */
static char* srioFtypeToString (uint32_t fType, uint32_t tType)
{
	static char		textBuffer[15];
	switch (fType)
	{
		case Srio_Ftype_REQUEST:
			//sprintf (textBuffer, "%s_%s", "Type-2", srioTtypeToString (tType, fType));
			sprintf (textBuffer, "%s_%s", "DIO", srioTtypeToString (tType, fType));
			break;
		case Srio_Ftype_WRITE:
			sprintf (textBuffer, "%s_%s", "DIO", srioTtypeToString (tType, fType));
			break;
		case Srio_Ftype_SWRITE:
			sprintf (textBuffer, "%s_%s", "DIO", srioTtypeToString (tType, fType));
			break;
		case Srio_Ftype_CONGESTION:
			sprintf (textBuffer, "%s", "Type-7");
			break;
		case Srio_Ftype_MAINTENANCE:
			sprintf (textBuffer, "%s", "Type-8");
			break;
		case Srio_Ftype_DATA_STREAMING:
			sprintf (textBuffer, "%s", "Type-9");
			break;
		case Srio_Ftype_DOORBELL:
			sprintf (textBuffer, "%s", "DOORBELL");
			break;
		case Srio_Ftype_MESSAGE:
			sprintf (textBuffer, "%s", "Type-11");
			break;
		case Srio_Ftype_RESPONSE:
			sprintf (textBuffer, "%s", "Type-13");
			break;
		default:
			sprintf (textBuffer, "%s", "Unknown");
			break;
	}
    return textBuffer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to return a text string translation of the test_to_run code specified.
 *
 *  @retval
 *      Pointer to the string containing the control code decode text
  */
char* testsToRunDecodeText (uint32_t testToRunCode)
{
	static char		testToRunCodeString[25];
	switch (testToRunCode)
	{
		case srio_nwrite_tests:
			sprintf (testToRunCodeString,"NWRITE tests");
			break;
		case srio_nread_tests:
			sprintf (testToRunCodeString,"NREAD tests");
			break;
		case srio_nwrite_nread_tests:
			sprintf (testToRunCodeString,"NWRITE and NREAD tests");
			break;
		case srio_type11_tests:
			sprintf (testToRunCodeString,"Type-11 tests");
			break;
		case srio_all_available_tests:
			sprintf (testToRunCodeString,"All tests");
			break;
		default:
			sprintf (testToRunCodeString,"Unknown: %d",testToRunCode);
			break;
	}
	return (testToRunCodeString);
}

/**
 *  @b Description
 *  @n
 *      The function initializes the pacing search structure.
 *
 *  @param[in]  pacing
 *      Pointer to the pacing search structure.
 *
 *  @retval
 *      Not Applicable.
 */
void initPacingSearchValues (Tput_BinSearch* ptrPacing, uint32_t maxCycles)
{
    ptrPacing->unstableCount = 0;
    ptrPacing->currMin = 0;
    ptrPacing->currMax = maxCycles;
    ptrPacing->curr = 0;
    ptrPacing->knownGood = ptrPacing->currMax;
    ptrPacing->isDelayWorking = FALSE;
}

/**
 *  @b Description
 *  @n
 *      The function sets ptrPacing->curr to the mid value based on the min/max values in the
 *      pacing structure. It also keeps track of where it currently is to provide the next value
 *      in the sequence when requested.
 *
 *  @param[in]  ptrPacing
 *      Pointer of the pacing search structure.
 *
 *  @retval
 *      Not Applicable.
 */
void getPacingBisect (Tput_BinSearch* ptrPacing)
{
	uint32_t		bisectValue=0;

	if (ptrPacing->isDelayWorking)
	{
		if (ptrPacing->curr <= ptrPacing->currMin)
			bisectValue = ptrPacing->currMin;
		else
		{
			/* Need less delay. Go towards min */
			if (ptrPacing->curr < ptrPacing->currMax)
				ptrPacing->currMax = ptrPacing->curr;
			bisectValue =  (uint32_t)(ptrPacing->curr - (abs(ptrPacing->currMax - ptrPacing->currMin) / 2));
		}
		ptrPacing->knownGood = ptrPacing->curr;
	}
	else
	{
		if (ptrPacing->curr >= ptrPacing->currMax)
			bisectValue = ptrPacing->currMax;
		else
		{
			/* Need more delay. Go towards max */
			ptrPacing->currMin = ptrPacing->curr;
			bisectValue = (uint32_t)(ptrPacing->curr + (abs(ptrPacing->currMax - ptrPacing->currMin) / 2));
		}
		if (ptrPacing->curr == ptrPacing->knownGood)
		{
			/* Increment the unstable throughput count that is occurring even though value has already been determined */
			ptrPacing->unstableCount = ptrPacing->unstableCount + 1;

			/* Keep the delay value going up. If we are here the throughput is still not stable */
			ptrPacing->knownGood += (uint32_t)(2 * ((ptrPacing->unstableCount / (uint32_t)5) + (uint32_t)1));
		}
	}
	if ((abs(ptrPacing->currMax - ptrPacing->currMin) / 2) < 2)
		bisectValue = ptrPacing->knownGood;

	ptrPacing->curr = bisectValue;
}

/**
 *  @b Description
 *  @n
 *      The function displays various SRIO register values.
 *
 *  @retval
 *      Always 0
 */
int32_t displaySrioStatus (void)
{
	displaySrioLTLEDCSRErrorStatus ();
	displayPortResponseTimeout ();
	displayPortGeneralCSR ();
	displayVariousRegisters ();
	return 0;
}

/**
 *  @b Description
 *  @n
 *      Read TSCL+TSCH and return as a 64 bit unsigned integer.
 *
 *  @retval
 *      TSCL+TSCH as 64bit unsigned integer
 */
static inline uint64_t tputExampleReadTime ()
{
  uint32_t low = TSCL;
  uint32_t high = TSCH;
  return _itoll(high,low);
}

/**
 *  @b Description
 *  @n
 *      The function waits for a number of cycles to pass.
 *
 *  @param[in]  cyclesToDelay
 *      Number of cycles to wait.
 *
 *  @retval
 *      Not Applicable.
 */
void cycleDelay (uint64_t cyclesToDelay)
{
	uint64_t	now = tputExampleReadTime ();
	while((tputExampleReadTime() - now) < cyclesToDelay);
}

/**
 *  @b Description
 *  @n
 *      The function is used to calculate throughput statistics based on
 *      packet count, packet size, starting timestamp, and ending timestamp
 *      using the timer frequency and clock frequency.
 *
 *  @param[in]  tparams->numberOfPackets
 *      Number of packets to calculate statistics on
 *  @param[in]  tparams->packetSizeBytes
 *      Packet size in bytes to calculate statistics on
 *  @param[in]  tparams->overheadBytes
 *      Bytes of overhead each packet has.
 *  @param[in]  tparams->tsLoopStart
 *      Start timestamp
 *  @param[in]  tparams->tsLoopEnd
 *      End timestamp
 *  @param[in]  tparams->totalLldCycles
 *      Total cycles that the LLD used
 *  @param[in]  tparams->totalProcCycles
 *      Total cycles for other processing
 *  @param[in]  tparams->pacingCycles
 *      Number of cycles used to pace each transmitted packet.
 *  @param[in]  tparams->isShowHeader
 *      Variable to control the display of the information header.
 *
 *  @retval
 *      Not applicable
 */
void displayTputStatistics (Tput_Parameters* tparams)
{
	uint64_t		elapsedCycles;
	uint32_t		averageCycles=0, averageLldCycles=0, averageProcCycles=0, averageIdleCycles=0, coreNumber;
    float			cpuTimerFreqRatio, tpsCalc, bitsPerSecondNoOh, bitsPerSecondOh, elapsedSeconds;
    Types_FreqHz	timerFreq, cpuFreq;

    static char		bpsText[MAX_SHORT_LEN];
    static char		bpsOHText[MAX_SHORT_LEN];
    static char		ppsText[MAX_SHORT_LEN];
    static char		numPktsText[MAX_MSG_LEN];
    static char		eSecsText[MAX_SHORT_LEN];
    static char		laneConfigText[MAX_SHORT_LEN];
    static char		laneSpeedText[MAX_SHORT_LEN];
    static char		connectionText[MAX_SHORT_LEN];
    static char		packetTypeText[MAX_SHORT_LEN];

    isDisplayTxNow = ((tparams->fType == (uint32_t)Srio_Ftype_REQUEST) ? TRUE : FALSE);

	/* Get the core number. */
    coreNumber = CSL_chipReadReg (CSL_CHIP_DNUM);

    Timestamp_getFreq (&timerFreq);
    BIOS_getCpuFreq (&cpuFreq);
    cpuTimerFreqRatio = (float)(cpuFreq.lo / timerFreq.lo);

	elapsedCycles = tparams->tsLoopEnd - tparams->tsLoopStart;
    elapsedCycles *= cpuTimerFreqRatio;

    averageCycles = (uint32_t)(elapsedCycles/tparams->numberOfPackets);
    averageLldCycles = (uint32_t)(tparams->totalLldCycles/tparams->numberOfPackets);
    averageProcCycles = (uint32_t)(tparams->totalProcCycles/tparams->numberOfPackets);
    averageIdleCycles = (uint32_t)(averageCycles - averageLldCycles - averageProcCycles);

    tpsCalc = (float)(cpuFreq.lo / (float)(elapsedCycles/tparams->numberOfPackets));

    bitsPerSecondNoOh = ((float)((((Double)cpuFreq.lo))/(Double)averageCycles));
    bitsPerSecondOh = bitsPerSecondNoOh;
    bitsPerSecondNoOh *= (float)tparams->packetSizeBytes * (float)8;
    bitsPerSecondOh *= ((float)tparams->packetSizeBytes + (float)tparams->overheadBytes)*(float)8;
    bitsPerSecondNoOh /= (float)1000000;
    bitsPerSecondOh /= (float)1000000;

    elapsedSeconds = CYCLES_TO_SECS(elapsedCycles, cpuFreq.lo);

    sprintf (bpsText,"%s", floatToString(bitsPerSecondNoOh, OF_LF02));
    sprintf (bpsOHText,"%s", floatToString(bitsPerSecondOh, OF_LF02));
    sprintf (ppsText,"%s", floatToString(tpsCalc, OF_LF02));
    sprintf (numPktsText,"%s", uint64ToString(tparams->numberOfPackets));
    sprintf (eSecsText,"%s", floatToString(elapsedSeconds, OF_LF02));

    sprintf (laneConfigText,"%s", srioLaneModeToString(testControl.srio_lanesMode));
    sprintf (laneSpeedText,"%s", srioLaneSpeedToString(testControl.srio_laneSpeedGbps));
    sprintf (connectionText,"%s", getConnectionType());
    sprintf (packetTypeText,"%s", srioFtypeToString(tparams->fType, tparams->tType));

	if (tparams->isTestInfoOnly)
	{
		/* Output test information header */
		addStringToStatsOutputBuffer ("Throughput: (","");
		addStringToStatsOutputBuffer (((coreNumber % 2 == 0) ? "RX side" : "TX side"),", ");
		addStringToStatsOutputBuffer (packetTypeText,", ");				/* MsgType */
		addStringToStatsOutputBuffer (laneSpeedText,"");				/* Speed */
    	addStringToStatsOutputBuffer ("GBaud",", ");
    	addStringToStatsOutputBuffer (laneConfigText,"");				/* Lanes */
    	addStringToStatsOutputBuffer ("X",", ");
    	addStringToStatsOutputBuffer ("tab delimited)","\n");

    	if (testControl.srio_isRunProgress)
    	{
			if (testControl.srio_isBoardToBoard || (coreNumber % 2 == 1) )
				System_printf ("\n(%s, %sGbaud, %sX)\n",packetTypeText, laneSpeedText, laneConfigText);
    	}
	}

	if (!tparams->isTestInfoOnly)
	{
		if (tparams->isShowHeader)
		{
			/* Output column headers */
			addStringToStatsOutputBuffer ("Core","\t");
			addStringToStatsOutputBuffer ("Lanes","\t");
			addStringToStatsOutputBuffer ("Speed","\t");
			addStringToStatsOutputBuffer ("Conn","\t");
			addStringToStatsOutputBuffer ("MsgType","\t");
			addStringToStatsOutputBuffer ("OHBytes","\t");
			addStringToStatsOutputBuffer ("PktSize","\t");
			addStringToStatsOutputBuffer ("Pacing","\t");
			(strlen(bpsText) < 8 ? addStringToStatsOutputBuffer("Thruput","\t") : addStringToStatsOutputBuffer("Thruput ","\t"));
			addStringToStatsOutputBuffer ("PktsSec.","\t");
			(strlen(numPktsText) < 8 ? addStringToStatsOutputBuffer("NumPkts","\t") : addStringToStatsOutputBuffer("NumPkts ","\t"));
			addStringToStatsOutputBuffer ("PktLoss","\t");
			addStringToStatsOutputBuffer ("AgPCycs","\t");
			addStringToStatsOutputBuffer ("AgLCycs","\t");
			addStringToStatsOutputBuffer ("AgICycs","\t");
			addStringToStatsOutputBuffer ("AgOCycs","\t");
			addStringToStatsOutputBuffer ("Seconds","\n");
		}
		addDecimalToStatsOutputBuffer (coreNumber,"\t");				/* Core */
		addStringToStatsOutputBuffer (laneConfigText,"\t");				/* Lanes */
		addStringToStatsOutputBuffer (laneSpeedText,"\t");				/* Speed */
		addStringToStatsOutputBuffer (connectionText,"\t");				/* Conn */
		addStringToStatsOutputBuffer (packetTypeText,"\t");				/* MsgType */
		addDecimalToStatsOutputBuffer (get256ByteOverHead(tparams->packetSizeBytes, tparams->overheadBytes),"\t");	/* OHBytes */
		addDecimalToStatsOutputBuffer (tparams->packetSizeBytes,"\t");	/* PktSize */
		addDecimalToStatsOutputBuffer (tparams->pacingCycles,"\t");		/* Pacing */
		addStringToStatsOutputBuffer (bpsText,"\t");					/* Thruput */
		addStringToStatsOutputBuffer (ppsText,"\t");					/* PktsSec */
		addStringToStatsOutputBuffer (numPktsText,"\t");				/* NumPkts */
		addStringToStatsOutputBuffer ("No","\t");						/* PktLoss */
		addLongToStatsOutputBuffer (averageCycles,"\t");				/* AgPCycs */
		addLongToStatsOutputBuffer (averageLldCycles,"\t");				/* AgLCycs */
		addLongToStatsOutputBuffer (averageIdleCycles,"\t");			/* AgICycs */
		addLongToStatsOutputBuffer (averageProcCycles,"\t");			/* AgOCycs */
		addStringToStatsOutputBuffer (eSecsText,"\n");					/* Duration */
	}

	if (testControl.srio_isRunProgress)
	{
		if (!tparams->isTestInfoOnly)
		{
			if (tparams->isShowHeader)
			{
				//System_printf ("\n(%s, %sGbaud, %sX)\n",packetTypeText, laneSpeedText, laneConfigText);
				printInSpaceText ("Cr",SFSIZE_1);
				printInSpaceText ("Size",SFSIZE_2);
				printInSpaceText ("MBps(NoOh)",SFSIZE_3);
				printInSpaceText ("MBps(Oh)",SFSIZE_4);
				printInSpaceText ("Packts/sec",SFSIZE_5);
				printInSpaceText ("NumPackets",SFSIZE_6);
				printInSpaceText ("Elapsed",SFSIZE_7);
				printInSpaceText ("Pkt/cycs",SFSIZE_8);
				printInSpaceText ("LLD/cycs",SFSIZE_9);
				printInSpaceText ("Idl/cycs",SFSIZE_9);
				printInSpaceText ("Oth/cycs",SFSIZE_10);
				System_printf ("\n");

			}
			printInSpaceDecimal (coreNumber,SFSIZE_1);
			printInSpaceDecimal (tparams->packetSizeBytes,SFSIZE_2);
			printInSpaceText (bpsText,SFSIZE_3);
			printInSpaceText (bpsOHText,SFSIZE_4);
			printInSpaceText (ppsText,SFSIZE_5);
			printInSpaceText (numPktsText,SFSIZE_6);
			printInSpaceText (eSecsText,SFSIZE_7);
			printInSpaceDecimal (averageCycles,SFSIZE_8);
			printInSpaceDecimal (averageLldCycles,SFSIZE_9);
			printInSpaceDecimal (averageIdleCycles,SFSIZE_9);
			printInSpaceDecimal (averageProcCycles,SFSIZE_10);
			System_printf ("\n");
		}
	}
	isDisplayTxNow = FALSE;
#ifdef TPUT_DEBUG
    System_printf ("     CPU frequency             : %lu\n",cpuFreq.lo);
    System_printf ("     Timer frequency           : %lu\n",timerFreq.lo);
    System_printf ("     CPU/Timer ratio           : %s\n", floatToString(cpuTimerFreqRatio, OF_LF02));
    System_printf ("     Start timestamp           : %s\n", uint64ToString(tparams->tsLoopStart));
    System_printf ("     End timestamp             : %s\n", uint64ToString(tparams->tsLoopEnd));
    System_printf ("     Packet overhead (bytes)   : %d \n", tparams->overheadBytes);
    System_printf ("     Average packet time (uS)  : %d\n", CYCLES_TO_US(averageCycles, cpuFreq.lo));
    System_printf ("     Average packet cycles     : %lu\n", averageCycles);
    System_printf ("     Total elapsed cycles      : %s\n", uint64ToString(elapsedCycles));
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to calculate throughput statistics based on
 *      packet count, packet size, starting timestamp, and ending timestamp
 *      using the timer frequency and clock frequency.
 *
 *  @param[in]  tparams->numberOfPackets
 *      Number of packets to calculate statistics on
 *  @param[in]  tparams->packetSizeBytes
 *      Packet size in bytes to calculate statistics on
 *  @param[in]  tparams->overheadBytes
 *      Bytes of overhead each packet has.
 *  @param[in]  tparams->tsLoopStart
 *      Start timestamp
 *  @param[in]  tparams->tsLoopEnd
 *      End timestamp
 *  @param[in]  tparams->totalLldCycles
 *      Total cycles that the LLD used
 *  @param[in]  tparams->totalProcCycles
 *      Total cycles for other processing
 *  @param[in]  tparams->pacingCycles
 *      Number of cycles used to pace each transmitted packet.
 *  @param[in]  tparams->isShowHeader
 *      Variable to control the display of the information header.
 *
 *  @retval
 *      Not applicable
 */
void displayLatencyStatistics (Tput_Parameters* tparams, bool isRoundTripTime)
{
	uint32_t		averageLldCycles=0, averageEndToEndCycles, minEndToEndCycles=0, maxEndToEndCycles=0;
	uint32_t		coreNumber, divisor;
    float			cpuTimerFreqRatio;
    Types_FreqHz	timerFreq, cpuFreq;

    static char		numPktsText[MAX_MSG_LEN];
    static char		laneConfigText[MAX_SHORT_LEN];
    static char		laneSpeedText[MAX_SHORT_LEN];
    static char		connectionText[MAX_SHORT_LEN];
    static char		packetTypeText[MAX_SHORT_LEN];

    isDisplayTxNow = TRUE;

	/* Get the core number. */
    coreNumber = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Set divisor */
    divisor = (isRoundTripTime ? (uint32_t)2 : (uint32_t)1);

    Timestamp_getFreq (&timerFreq);
    BIOS_getCpuFreq (&cpuFreq);
    cpuTimerFreqRatio = (float)(cpuFreq.lo / timerFreq.lo);

    averageLldCycles = (uint32_t)(tparams->totalLldCycles/tparams->numberOfPackets);
    averageLldCycles *= cpuTimerFreqRatio;
    averageEndToEndCycles = averageLldCycles / divisor;

    minEndToEndCycles = tparams->minLatCycles / divisor;
    maxEndToEndCycles = tparams->maxLatCycles / divisor;

    sprintf (numPktsText,"%s", uint64ToString(tparams->numberOfPackets));

    sprintf (laneConfigText,"%s", srioLaneModeToString(testControl.srio_lanesMode));
    sprintf (laneSpeedText,"%s", srioLaneSpeedToString(testControl.srio_laneSpeedGbps));
    sprintf (connectionText,"%s", getConnectionType());
    sprintf (packetTypeText,"%s", srioFtypeToString(tparams->fType, tparams->tType));

	if (tparams->isTestInfoOnly)
	{
		/* Output test information header */
		addStringToStatsOutputBuffer ("Latency: (","");
		addStringToStatsOutputBuffer (packetTypeText,", ");				/* MsgType */
		addStringToStatsOutputBuffer (laneSpeedText,"");				/* Speed */
		addStringToStatsOutputBuffer ("GBaud",", ");
		addStringToStatsOutputBuffer (laneConfigText,"");				/* Lanes */
		addStringToStatsOutputBuffer ("X",", ");
		addStringToStatsOutputBuffer ("tab delimited)","\n");

		if (testControl.srio_isRunProgress)
			System_printf ("\n(%s, %sGbaud, %sX)\n",packetTypeText, laneSpeedText, laneConfigText);
	}

	if (!tparams->isTestInfoOnly)
	{
		if (tparams->isShowHeader)
		{
			/* Output column headers */
			addStringToStatsOutputBuffer ("Core","\t");
			addStringToStatsOutputBuffer ("Lanes","\t");
			addStringToStatsOutputBuffer ("Speed","\t");
			addStringToStatsOutputBuffer ("Conn","\t");
			addStringToStatsOutputBuffer ("MsgType","\t");
			addStringToStatsOutputBuffer ("PktSize","\t");
			(strlen(numPktsText) < 8 ? addStringToStatsOutputBuffer("NumPkts","\t") : addStringToStatsOutputBuffer("NumPkts ","\t"));
			addStringToStatsOutputBuffer ("MnLCycs","\t");
			addStringToStatsOutputBuffer ("AgLCycs","\t");
			addStringToStatsOutputBuffer ("MxLCycs","\n");
		}
		addDecimalToStatsOutputBuffer (coreNumber,"\t");				/* Core */
		addStringToStatsOutputBuffer (laneConfigText,"\t");				/* Lanes */
		addStringToStatsOutputBuffer (laneSpeedText,"\t");				/* Speed */
		addStringToStatsOutputBuffer (connectionText,"\t");				/* Conn */
		addStringToStatsOutputBuffer (packetTypeText,"\t");				/* MsgType */
		addDecimalToStatsOutputBuffer (tparams->packetSizeBytes,"\t");	/* PktSize */
		addStringToStatsOutputBuffer (numPktsText,"\t");				/* NumPkts */
		addLongToStatsOutputBuffer (minEndToEndCycles,"\t");		/* MnLCycs */
		addLongToStatsOutputBuffer (averageEndToEndCycles,"\t");				/* AgLCycs */
		addLongToStatsOutputBuffer (maxEndToEndCycles,"\n");		/* AgICycs */
	}

	if (testControl.srio_isRunProgress)
	{
		if (!tparams->isTestInfoOnly)
		{
			if (tparams->isShowHeader)
			{
				printInSpaceText ("Cr",SFSIZE_1);
				printInSpaceText ("Size",SFSIZE_2);
				printInSpaceText ("NumPackets",SFSIZE_6);
				printInSpaceText ("Min/cycs",SFSIZE_9);
				printInSpaceText ("Avg/cycs",SFSIZE_9);
				printInSpaceText ("Max/cycs",SFSIZE_9);
				System_printf ("\n");
			}
			printInSpaceDecimal (coreNumber,SFSIZE_1);
			printInSpaceDecimal (tparams->packetSizeBytes,SFSIZE_2);
			printInSpaceText (numPktsText,SFSIZE_6);
			printInSpaceDecimal (minEndToEndCycles,SFSIZE_9);
			printInSpaceDecimal (averageEndToEndCycles,SFSIZE_9);
			printInSpaceDecimal (maxEndToEndCycles,SFSIZE_9);
			System_printf ("\n");
		}
	}
	isDisplayTxNow = FALSE;
#ifdef TPUT_DEBUG
    System_printf ("     CPU frequency             : %lu\n",cpuFreq.lo);
    System_printf ("     Timer frequency           : %lu\n",timerFreq.lo);
    System_printf ("     CPU/Timer ratio           : %s\n", floatToString(cpuTimerFreqRatio, OF_LF02));
    System_printf ("     Start timestamp           : %s\n", uint64ToString(tparams->tsLoopStart));
    System_printf ("     End timestamp             : %s\n", uint64ToString(tparams->tsLoopEnd));
    System_printf ("     Packet overhead (bytes)   : %d \n", tparams->overheadBytes);
    System_printf ("     Average packet time (uS)  : %d\n", CYCLES_TO_US(averageEndToEndCycles, cpuFreq.lo));
    System_printf ("     Average packet cycles     : %lu\n", averageEndToEndCycles);
    //System_printf ("     Total elapsed cycles      : %s\n", uint64ToString(elapsedCycles));
#endif
}

/**
 *  @b Description
 *  @n
 *      Utility function that will set a global variable based on the index and value passed.
 *
 *  @param[in]  paramIndex
 *      Index for the parameter that is to be set.
 *  @param[in]  paramValue
 *      Value to which to set the parameter.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t setOneStartParameter (int paramIndex, int32_t paramValue)
{
	int32_t	returnStatus = 0;

	switch (paramIndex)
	{
		case 0:	/* Parameter is always zero and therefore is not of use */
			break;
		case 1: /* Set port lane configuration */
			switch (paramValue)
			{
				case 1:
					testControl.srio_lanesMode = srio_lanes_form_four_1x_ports;
					System_printf ("Four 1X ports");
					break;
				case 2:
					testControl.srio_lanesMode = srio_lanes_form_two_2x_ports;
					System_printf ("Two 2X ports");
					break;
				case 4:
					testControl.srio_lanesMode = srio_lanes_form_one_4x_port;
					System_printf ("One 4X port");
					break;
				default:
					System_printf ("\n**Port configuration value");
					returnStatus = -1;
					break;
			}
			break;
		case 2:	/* Set lane rate */
			switch (paramValue)
			{
				case 1:
					testControl.srio_laneSpeedGbps = srio_lane_rate_1p250Gbps;
					System_printf ("1.25Gbaud lane rate");
					break;
				case 2:
					testControl.srio_laneSpeedGbps = srio_lane_rate_2p500Gbps;
					System_printf ("2.5Gbaud lane rate");
					break;
				case 3:
					testControl.srio_laneSpeedGbps = srio_lane_rate_3p125Gbps;
					System_printf ("3.125Gbaud lane rate");
					break;
				case 5:
					testControl.srio_laneSpeedGbps = srio_lane_rate_5p000Gbps;
					System_printf ("5.0Gbaud lane rate");
					break;
				default:
					System_printf ("\n**Lane rate");
					returnStatus = -1;
					break;
			}
			break;
		case 3:	/* Set the number of seconds to run the test per packet size */
			if ((paramValue >= 1) && (paramValue <= 65535))
			{
				testControl.srio_testTimeInSeconds =  paramValue;
				System_printf ("%d seconds test time per packet size", testControl.srio_testTimeInSeconds);
			}
			else
			{
				System_printf ("\n**Test seconds");
				returnStatus = -1;
			}
			break;
		case 4:	/* Set test mode for which tests to run */
			if ((paramValue >= 0) && (paramValue <= srio_all_available_tests))
			{
				testControl.srio_testsToRun =  (srioTestsToRun_e)paramValue;
				System_printf ("%s will be run", testsToRunDecodeText(testControl.srio_testsToRun));
			}
			else
			{
				System_printf ("\n**Tests to run selection number");
				returnStatus = -1;
			}
			break;
		case 5:	/* Set packet size to start with */
			if ((paramValue >= 4) && (paramValue <= 8192))
			{
				/* Set dio payload start size */
				testControl.srio_dioPayloadSize = paramValue;

				/* Set type11 payload start size */
				if (paramValue < 16)
					testControl.srio_payloadSize = 16 - SOFTWARE_HEADER_BYTES;
				else if (paramValue > 4096)
					testControl.srio_payloadSize = 4096 - SOFTWARE_HEADER_BYTES;
				else
					testControl.srio_payloadSize = paramValue - SOFTWARE_HEADER_BYTES;

				System_printf ("%d bytes start packet size", paramValue);
			}
			else
			{
				System_printf ("\n**Packet start size");
				returnStatus = -1;
			}
			break;
		case 6:	/* Set packet size to end with */
			if ((paramValue >= 4) && (paramValue <= 8192))
			{
				/* Set dio payload end size */
				testControl.srio_dioPayloadEndSize = paramValue;

				/* Set type11 payload end size */
				if (paramValue < 16)
					testControl.srio_payloadEndSize = 16 - SOFTWARE_HEADER_BYTES;
				else if (paramValue > 4096)
					testControl.srio_payloadEndSize = 4096 - SOFTWARE_HEADER_BYTES;
				else
					testControl.srio_payloadEndSize = paramValue - SOFTWARE_HEADER_BYTES;

				System_printf ("%d bytes end packet size", paramValue);
			}
			else
			{
				System_printf ("\n**Packet end size");
				returnStatus = -1;
			}
			break;
		case 7:	/* Set core that will initialize sRIO IP */
			if ((paramValue >= 0) && (paramValue <= 1))
			{
				testControl.srio_initCorenum = paramValue;
				System_printf ("Core %d will initialize the sRIO IP", paramValue);
			}
			else
			{
				System_printf ("\n**sRIO core init value");
				returnStatus = -1;
			}
			break;
		case 8:	/* Set whether test is board to board */
			if ((paramValue >= 0) && (paramValue <= 1))
			{
				testControl.srio_isBoardToBoard = ((paramValue == 1) ? TRUE : FALSE);
				System_printf ("Testing %s", ((testControl.srio_isBoardToBoard) ? "board to board" : "core to core on same EVM"));
			}
			else
			{
				System_printf ("\n**Board to board value");
				returnStatus = -1;
			}
			break;
		case 9:	/* Set whether external SRIO switch is being used for test. */
			if ((paramValue >= 0) && (paramValue <= 1))
			{
				testControl.srio_isExternalSrioSwitch = ((paramValue == 1) ? TRUE : FALSE);
				System_printf ("%s for test", ((testControl.srio_isExternalSrioSwitch) ? "Using external SRIO switch" : "Not using external SRIO switch"));
			}
			else
			{
				System_printf ("\n**External SRIO switch being used value");
				returnStatus = -1;
			}
			break;
	}
	if (returnStatus != 0)
	{
		System_printf (" of %d is invalid, please try again.\n", paramValue);
		System_printf ("   Options available: [port_confg] [lane_rate] [test_secs] [tests_to_run]\n");
		System_printf ("                      [packet_start_size] [packet_end_size] [srio_init_core]\n");
		System_printf ("                      [board_to_board] [srio_switch]\n");
		System_printf ("       Option ranges:\n");
		System_printf (" -------------------------------------------------------------\n");
		System_printf ("        [port_confg]: (1X, 2X, 4X)              default: 4X\n");
		System_printf ("         [lane_rate]: (1.25, 2.50, 3.125, 5.0)  default: 5.0\n");
		System_printf ("         [test_secs]: (1 to 65535)              default: 60\n");
		System_printf ("      [tests_to_run]: (0 to %d)                  default: %d\n", srio_all_available_tests, testControl.srio_testsToRun);
		System_printf ("                       %d for NWRITE, %d for NREAD, %d for NWRITE & NREAD,.\n", srio_nwrite_tests, srio_nread_tests, srio_nwrite_nread_tests);
		System_printf ("                       %d for Type11, %d for all tests.\n", srio_type11_tests, srio_all_available_tests);
		System_printf (" [packet_start_size]: (%d to %d)              default: %d\n", SOFTWARE_HEADER_BYTES, MESSAGE_MAX_DATA_SIZE, MESSAGE_INITIAL_DATA_SIZE);
		System_printf ("   [packet_end_size]: (%d to %d)              default: %d\n", SOFTWARE_HEADER_BYTES, MESSAGE_MAX_DATA_SIZE, MESSAGE_MAX_DATA_SIZE);
		System_printf ("    [srio_init_core]: (%d to %d)                  default: %d\n", 0, 1, 0);
		System_printf ("                       For core to core testing:\n");
		System_printf ("                         0 for RX and TX side.\n");
		System_printf ("                       For board to board testing:\n");
		System_printf ("                         0 for RX side (Consumer).\n");
		System_printf ("                         1 for TX side (Producer).\n");
		System_printf ("    [board_to_board]: (%d to %d)                  default: %d\n", 0, 1, 0);
		System_printf ("                       0 for core to core testing.\n");
		System_printf ("                       1 for board to board testing.\n");
		System_printf ("     [srio_switch]  : (%d to %d)                  default: %d\n", 0, 1, 0);
		System_printf ("                       0 when not using an external SRIO switch.\n");
		System_printf ("                       1 when using an external SRIO switch.\n");
		System_printf ("  Note:All options shown are optional, but they must be specified in the order\n");
		System_printf ("       shown above. For example, in order to use the {test_secs} parameter the\n");
		System_printf ("       {port_config} and {lane_rate} parameters must be specified before it.\n\n");
	}
	return returnStatus;
}

/**
 *  @b Description
 *  @n
 *      Utility function that will set the global variables that determine how the test will be run,
 *      if user passes values to the test.
 *
 *  @param[in]  argc
 *      Count of parameters passed to this application.
 *  @param[in]  argv
 *      Array of parameters passed to this application.
 *  @param[in]  coreNum
 *      Core for which these parameters are being set.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t setStartParameters (int argc, char* argv[], uint32_t coreNum)
{
	int					index;

	if (argc > 0)
		System_printf ("Setting core %d test parameters to:\n", coreNum);

	/* Process all arguments */
	for (index=1; index < argc; index++)
	{
		System_printf("  ");
		if (setOneStartParameter(index, atol(argv[index])) < 0)
			return -1;
		System_printf ("\n");
	}
	if (argc > 0)
		System_printf ("\n");
	return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to set the BIOS cpu frequency indicator value based on
 *      the PLL base frequency, PLL multiplier and PLL divider register values.
 *      This function will only run for 6638 K2H and K2K.
 *
 *  @retval
 *      Always returns 0
 */
#define MAIN_PLLD_MASK              0xFFFFFFC0
int32_t setBiosCpuFrequencyIndicatorIfNeeded (void)
{
#if defined(__CSL_RL_K2H_H__) || defined(__CSL_RL_K2K_H__)
	Types_FreqHz		freq;
	uint32_t			pllBaseFreq, pllMultiplier;
	CSL_PllcRegs		*pllMultiplierREGs;

	pllBaseFreq = 122880000;
	freq.hi = 0;
	pllMultiplierREGs = (CSL_PllcRegs*)(CSL_PLLC_REGS);
	pllMultiplier = (pllMultiplierREGs->PLLM + 1) / ((pllMultiplierREGs->PLLCTL&&MAIN_PLLD_MASK) + 1);
	freq.lo = pllBaseFreq * pllMultiplier;
	BIOS_setCpuFreq(&freq);
	System_printf ("\n");
	System_printf (" CPU Frequency       : %u Hz\n", freq.lo);
	System_printf ("   PLL base frequency: %u Hz\n", pllBaseFreq);
	System_printf ("   PLL multiplier    : %u\n\n", pllMultiplier);
#endif
	return 0;
}
