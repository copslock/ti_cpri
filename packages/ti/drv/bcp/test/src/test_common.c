/** 
 *   @file  test_common.c
 *
 *   @brief  
 *      Utility functions used commonly by the test application.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
#include "bcp_test.h"

/** ============================================================================
 *   @n@b read_data_from_file
 *
 *   @b Description
 *   @n Utility function to read data from a given file.
 *
 *   @param[in]  
 *   @n fp                  File handle from which data needs to be read from.
 *
 *   @param[out]  
 *   @n pDataBuffer         Data buffer handle to read into.
 *
 *   @param[out]  
 *   @n dataBufferLen       Number of bytes read from the file.
 *
 *   @return        
 *   @n None. 
 * =============================================================================
 */
Void read_data_from_file (FILE* fp, UInt8* pDataBuffer, UInt32* pDataBufferLen)
{
	Int32   temp = 0;
	
	while (fscanf(fp, "0x%x\n", &temp) != EOF)
	{
        *(UInt32 *)pDataBuffer  =   temp;
        *pDataBufferLen 		+=  4;
        pDataBuffer				+=	4;
	}
}

/** ============================================================================
 *   @n@b write_data_to_file
 *
 *   @b Description
 *   @n Utility function to write data to a given file.
 *
 *   @param[in]  
 *   @n pDataBuffer         Data buffer to save in the file.
 *
 *   @param[in]  
 *   @n dataBufferLen       Number of bytes to save.
 *
 *   @param[out]  
 *   @n fpOut               File handle to which data needs to be written to.
 *
 *   @return        
 *   @n None. 
 * =============================================================================
 */
Void write_data_to_file (UInt8* pDataBuffer, UInt32 dataBufferLen, FILE* fpOut)
{
	UInt32	i;
	
	i = 0;	
	while (dataBufferLen >= 4 && pDataBuffer)
	{
        fprintf (fpOut, "0x%08x\n", *(UInt32 *)pDataBuffer);
        pDataBuffer         +=  4;
        dataBufferLen   	-=  4;
        i++;
	}
    if (pDataBuffer && dataBufferLen)
    {
        fprintf(fpOut, "0x%08x\n", *(UInt32 *)pDataBuffer);
		i++;
    }
	
	return;    
}

#pragma DATA_SECTION (rowTbl, ".testData");
UInt32 rowTbl [71] = {24,  48,  72,  96,  120,
                      144, 192, 216, 240, 288,
					  360, 384, 432, 480, 576,
					  600, 648, 720, 768, 864,
					  960, 1080,1152,1200,1296,
					  1440,1536,1728,1800,1920,
					  1944,2160,2304,2400,2592,
					  2880,3072,3240,3456,3600,
					  3840,3888,4320,4608,4800,
					  5184,5400,5760,5832,6144,
					  6480,6912,7200,7680,7776,
					  8640,9216,9600,10368,10800,
					  11520,11664,12288,12960,13824,
					  14400,15360,15552,17280,18432,
					  19200};

/** ============================================================================
 *   @n@b gind_row_index
 *
 *   @b Description
 *   @n Utility function to calculate modulation engine's B table index.
 *
 *   @param[in]  
 *   @n input               Row table index.
 * 
 *   @return        
 *   @n >0                  B table index.
 * =============================================================================
 */
UInt8 gind_row_index (UInt32 input)
{
	UInt8 ii;
	
	for (ii = 0; ii < 71; ii++)
	{
		if (rowTbl[ii] == input)
			return ii;
	}

    return 0;
}

/** ============================================================================
 *   @n@b validate_rxdata
 *
 *   @b Description
 *   @n Utility API used to validate data received from BCP against output 
 *      reference data.
 *
 *   @param[in]  
 *   @n pRefDataBuffer      Reference data buffer handle.
 *
 *   @param[in]  
 *   @n refDataBufferLen    Total length of reference data.
 *
 *   @param[in]  
 *   @n pRxDataBuffer       Handle to data buffer received from BCP that needs
 *                          to be validated against reference data.
 *
 *   @param[in]  
 *   @n rxDataBufferLen     Number of bytes to compare.
 *
 *   @param[in]  
 *   @n dataOffset          Reference data offset. The data buffer will be compared
 *                          to reference data starting at this offset.
 *
 *   @return        
 *   @n 0           -       Data buffer matches output reference data.
 *   @n -1          -       Validation failed. Data buffer doesnt match output reference data. 
 * =============================================================================
 */
Int32 validate_rxdata 
(
    UInt8*              pRefDataBuffer,
    UInt32              refDataBufferLen,
    UInt8*              pRxDataBuffer, 
    UInt32              rxDataBufferLen, 
    UInt32              dataOffset
)
{
    /* Do some basic validation */            
    if (!pRefDataBuffer || !pRxDataBuffer || (refDataBufferLen < rxDataBufferLen + dataOffset))
            return -1;
            
    if (memcmp (pRefDataBuffer + dataOffset, pRxDataBuffer, rxDataBufferLen) != 0)
        return -1;            
    else
        return 0;   /* Data validation succeeded */
}

/* Macro to convert a 32 bit number into a byte by byte stream */
#ifdef  xdc_target__bigEndian                   
#define read32(a)  ((((a)>>24)&0xff) + (((a)>>8)&0xff00) + (((a)<<8)&0xff0000) + (((a)<<24)&0xff000000))
#else
#define read32(a)  (a)
#endif

/** ============================================================================
 *   @n@b read_harq_data_from_file
 *
 *   @b Description
 *   @n Utility function to read HARQ input data from a given file. HARQ I/p and
 *      O/p inside RD module is expected always to be fed in as 8 bit byte data.
 *      This function reads the HARQ input from a file as 8 bit byte data.
 *
 *   @param[in]  
 *   @n fp                  File handle from which data needs to be read from.
 *
 *   @param[out]  
 *   @n pDataBuffer         HARQ I/p buffer handle to read into.
 *
 *   @param[out]  
 *   @n dataBufferLen       Number of bytes read from the file.
 *
 *   @return        
 *   @n None. 
 * =============================================================================
 */
Void read_harq_data_from_file (FILE* fp, UInt8* pDataBuffer, UInt32* pDataBufferLen)
{
	Int32   temp = 0;
	
	while (fscanf(fp, "0x%x\n", &temp) != EOF)
	{
        /* Read HARQ I/p as 32 bit word from file and convert it into native 
         * 8 bit byte format and save it in memory.
         */
        *(UInt32 *)pDataBuffer  =   read32(temp);
        *pDataBufferLen 		+=  4;
        pDataBuffer				+=	4;
	}
}

/** ============================================================================
 *   @n@b validate_harqoutput
 *
 *   @b Description
 *   @n Utility API used to validate HARQ output data received from BCP against 
 *      HARQ output reference data. HARQ data is output as 8 bit byte data. The
 *      reference HARQ output on the other hand in application is stored as 
 *      32 bit word data. This API converts the reference data (32 bit) to 
 *      stream of 8 bit byte data and then compares against BCP output
 *      data to eliminate any endian dependent data ordering issues.
 *
 *   @param[in]  
 *   @n pRefDataBuffer      HARQ Output Reference data buffer handle.
 *
 *   @param[in]  
 *   @n refDataBufferLen    Total length of reference data.
 *
 *   @param[in]  
 *   @n pRxDataBuffer       Handle to harq output data buffer received from BCP 
 *                          that needs to be validated against reference data.
 *
 *   @param[in]  
 *   @n rxDataBufferLen     Number of bytes to compare.
 *
 *   @param[in]  
 *   @n dataOffset          Reference data offset. The data buffer will be compared
 *                          to reference data starting at this offset.
 *
 *   @return        
 *   @n 0           -       HARQ O/p Data buffer matches HARQ output reference data.
 *   @n -1          -       Validation failed. HARQ O/p Data buffer doesnt match 
 *                          HARQ output reference data. 
 * =============================================================================
 */
Int32 validate_harqoutput 
(
    UInt32*             pRefDataBuffer,
    UInt32              refDataBufferLen,
    UInt32*             pRxDataBuffer, 
    UInt32              rxDataBufferLen, 
    UInt32              dataOffset
)
{
	UInt32		        i;

    /* Do some basic validation */            
    if (!pRefDataBuffer || !pRxDataBuffer || (refDataBufferLen < rxDataBufferLen + dataOffset))
            return -1;

    for (i = 0; i < rxDataBufferLen/4; i ++, pRefDataBuffer++, pRxDataBuffer++)
    {
    	if (read32(*pRefDataBuffer)  != *pRxDataBuffer)
    		break;
    }
    if (i != rxDataBufferLen/4)
    	return -1;
    else
    	return 0;
}

/** ============================================================================
 *   @n@b deallocate_fdq
 *
 *   @b Description
 *   @n This API cleans up the Tx/Rx FDQ passed. It restores the free descriptors 
 *      from Tx/Rx FDQ to the application's global FDQ and closes the FDQ. If a 
 *      valid History queue handle is passed too, the API closes it also.
 *
 *   @param[in]  
 *   @n hFDQ            Tx/Rx FDQ handle obtained earlier from @a allocate_fdq () 
 *                      API.
 *
 *   @param[in]  
 *   @n hGlblFDQ        Global FDQ to which to restore the descriptors from the
 *                      Tx/Rx FDQ being de-allocated.
 *
 *   @param[in]  
 *   @n numDesc         Number of descriptors setup on this Rx/Tx FDQ.
 * 
 *   @param[in]  
 *   @n buffSize        Size of pre-allocated buffers linked to the descriptor.
 * 
 *   @param[in]  
 *   @n hHistQ          History queue handle obtained earlier from @a allocate_fdq () 
 *                      API.
 *
 *   @return        
 *   @n None.
 * =============================================================================
 */
Void deallocate_fdq 
(
    Qmss_QueueHnd       hFDQ, 
    Qmss_QueueHnd       hGlblFDQ, 
    UInt32              numDesc, 
    UInt32              buffSize,
    Qmss_QueueHnd       hHistQ
)
{
    UInt8               i;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    UInt32              dataBufferLen;

    for (i = 0; i < numDesc && hFDQ; i ++)
    {
        if ((pCppiDesc = Qmss_queuePop (hFDQ)) == NULL)
        {
            break;                
        }

        /* The descriptor address returned from the hardware has the 
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last 
         * 4 bits of the address.
         */
	    pCppiDesc = (Void*) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, buffSize);
        Bcp_osalFree (pDataBuffer, buffSize, TRUE);

        //Cppi_linkNextBD (Cppi_DescType_HOST, pCppiDesc, NULL);

        /* Push descriptor back to free queue */
        Qmss_queuePushDescSize (hGlblFDQ, pCppiDesc, 256);           
    }
	Qmss_queueClose (hFDQ);

    /* If a valid History queue handle's passed, close that too */
    if (hHistQ)
    	Qmss_queueClose (hHistQ);
	
    return;        
}

/** ============================================================================
 *   @n@b allocate_fdq
 *
 *   @b Description
 *   @n This API opens a Tx/Rx Free Descriptor queue (FDQ) and sets it up with
 *      'numDesc' number of free descriptors populated with pre-allocated buffers.
 *      If 'bAllocHistQ' flag is set to 1, it opens up a History queue too for
 *      the application and sets it up as the Tx completion queue for all Tx FDs.
 *
 *   @param[in]  
 *   @n hGlblFDQ        Global FDQ set up for the test application. This APi
 *                      uses the required number of FDs from this queue.
 *
 *   @param[in]  
 *   @n numDesc         Number of descriptors to setup for this Rx/Tx FDQ.
 * 
 *   @param[in]  
 *   @n buffSize        Size of data buffer to allocate. The data buffer is 
 *                      linked to descriptor at FDQ allocation time.
 * 
 *   @param[in]  
 *   @n bAllocHistQ     Boolean flag to indicate whether a History queue must
 *                      be opened for the application.
 *
 *   @param[in]  
 *   @n phFDQ           Pointer to store the Tx FDQ handle in.
 *
 *   @param[in]  
 *   @n phHistQ         Pointer to store the History queue handle in.
 *
 *   @return        Qmss_QueueHnd
 *   @n -1      -       Error setting up Tx/Rx FDQ with descriptors.
 *   @n 0       -       Succesfully set up a Tx/Rx FDQ with 'numDesc' descriptors.
 *                      All descriptors are linked with pre-allocated buffers of
 *                      size 'buffSize'.
 * =============================================================================
 */
Int32 allocate_fdq
(
    Qmss_QueueHnd       hGlblFDQ, 
    UInt32              numDesc, 
    UInt32              buffSize,
    UInt8               bAllocHistQ,
    Qmss_QueueHnd*      phFDQ,
    Qmss_QueueHnd*      phHistQ
)
{
    Qmss_QueueHnd       hFDQ;
    Qmss_QueueHnd       hHistQ = NULL;
    UInt8               isAllocated, i;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    Qmss_Queue          txComplnQInfo;

    if ((hFDQ = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	{
        Bcp_osalLog ("Error opening FDQ \n");
		return -1;
	}    

    if (bAllocHistQ)
    {
        /* Setup a History queue and use it as the Tx completion queue */            
        if ((hHistQ = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	    {
            Bcp_osalLog ("Error opening History Queue \n");
		    return -1;
	    }    
        txComplnQInfo = Qmss_getQueueNumber (hHistQ);
    }
    else
    {
        /* NO History queue requested. Use Tx FDQ itself as the Tx completion queue */            
        txComplnQInfo = Qmss_getQueueNumber (hFDQ);
    }

    for (i = 0; i < numDesc; i ++)
    {
        if ((pCppiDesc = Qmss_queuePop (hGlblFDQ)) == NULL)
        {
            break;                
        }

        /* The descriptor address returned from the hardware has the 
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last 
         * 4 bits of the address.
         */
	    pCppiDesc = (Void*) (QMSS_DESC_PTR (pCppiDesc));    
        if ((pDataBuffer =   (uint8_t *) Bcp_osalMalloc (buffSize, TRUE)) == NULL)
        {
        	Bcp_osalLog ("OOM error while allocating buffers\n");
            break;                
        }

        Cppi_setData (Cppi_DescType_HOST, pCppiDesc, pDataBuffer, buffSize);

        /* Save original buffer information */
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, pCppiDesc, pDataBuffer, buffSize);

        /* Setup the completion queue for the descriptor */
        Cppi_setReturnQueue (Cppi_DescType_HOST, pCppiDesc, txComplnQInfo);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, buffSize);

        Cppi_linkNextBD (Cppi_DescType_HOST, pCppiDesc, NULL);

        /* Push descriptor back to free queue */
        Qmss_queuePushDescSize (hFDQ, pCppiDesc, 256);           
    }
    if (i !=  numDesc)
	{
        deallocate_fdq (hFDQ, hGlblFDQ, i, buffSize, hHistQ);
        return -1;
	}            
    else
    {
        *phFDQ      =   hFDQ;
        *phHistQ    =   hHistQ;
        return 0;            
    }
}

/** ============================================================================
 *   @n@b prepare_crchdr_cfg
 *
 *   @b Description
 *   @n Sets up the CRC engine header for the test. 
 *
 *   @param[out]  
 *   @n pCrcHdrCfg      CRC Header configuration thus populated for the test.
 * 
 *   @param[in]  
 *   @n radioStd        Radio standard to use.
 * 
 *   @param[in]  
 *   @n tbSize          Transport block size.
 * 
 *   @param[in]  
 *   @n numFillerBits   Number of filler bits.
 * 
 *   @param[in]  
 *   @n method2Id       Method2 Id.
 * 
 *   @param[in]  
 *   @n dtxFormat       DTX format.
 * 
 *   @param[in]  
 *   @n scrambFlag      Enable/Disable Scrambler.
 * 
 *   @param[in]  
 *   @n crcSize         CRC size.
 * 
 *   @param[in]  
 *   @n numTb           number of transport blocks
 * 
 *   @param[in]  
 *   @n dataFormatIn    Input data format
 * 
 *   @param[in]
 *   @n numTrCh         Number of transport channels
 * 
 *   @param[in]
 *   @n trChLen         Transport channel length array
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
Void prepare_crchdr_cfg 
(
    Bcp_CrcHdrCfg*      pCrcHdrCfg, 
    Bcp_RadioStd        radioStd,
    UInt32              tbSize,
    UInt32              numFillerBits,
    UInt32              method2Id,
    UInt32              dtxFormat,
    UInt8               scrambFlag,
    UInt32              crcSize, 
    UInt32              numTb, 
    UInt32              dataFormatIn, 
    UInt32              numTrCh, 
    UInt32*             trChLen
)
{
    memset (pCrcHdrCfg, 0, sizeof (Bcp_CrcHdrCfg));

    /* Setup the CRC header configuration as per the  inputs passed. */
    if (scrambFlag == 1)
        pCrcHdrCfg->num_scramble_sys=   0x1000000;
    else
        pCrcHdrCfg->num_scramble_sys=   0;

    pCrcHdrCfg->filler_bits         =   numFillerBits;

    if (radioStd == Bcp_RadioStd_LTE)
        pCrcHdrCfg->bit_order       =   1;  //LTE bit order
    else
        pCrcHdrCfg->bit_order       =   0;  //WCDMA bit order

    pCrcHdrCfg->dtx_format          =   dtxFormat;
    pCrcHdrCfg->method2_id          =   method2Id;

    /* Initialize header length to 2 words. we'll increment it as 
     * and how we add more config.
     */
    pCrcHdrCfg->local_hdr_len       =   2; 
        
    if (dataFormatIn == 2)//transport channel concatenation  
	{
		if (numTrCh >= 1)
		{
            pCrcHdrCfg->va_blk_len  =   trChLen[0]; 
            pCrcHdrCfg->va_blks     =   1;
            pCrcHdrCfg->va_crc          =   Bcp_CrcFormat_Crc0;
        
            pCrcHdrCfg->local_hdr_len ++; 
		}

		if (numTrCh >= 2)
		{
            pCrcHdrCfg->vb_blk_len  =   trChLen[1]; 
            pCrcHdrCfg->vb_blks     =   1;
            pCrcHdrCfg->vb_crc          =   Bcp_CrcFormat_Crc0;
        
            pCrcHdrCfg->local_hdr_len ++; 
		}

		if (numTrCh >= 3)
		{
            pCrcHdrCfg->vc_blk_len  =   trChLen[2]; 
            pCrcHdrCfg->vc_blks     =   1;
            pCrcHdrCfg->vc_crc          =   Bcp_CrcFormat_Crc0;

            pCrcHdrCfg->local_hdr_len ++; 
        }

		if (numTrCh >= 4)
		{
            pCrcHdrCfg->d1_blk_len  =   trChLen[3];                
            pCrcHdrCfg->d1_blks     =   1;                
            pCrcHdrCfg->d1_crc          =   Bcp_CrcFormat_Crc0;                

            pCrcHdrCfg->local_hdr_len ++; 
        }

		if (numTrCh >= 5)
		{
            pCrcHdrCfg->d2_blk_len  =   trChLen[4];                
            pCrcHdrCfg->d2_blks     =   1;                
            pCrcHdrCfg->d2_crc          =   Bcp_CrcFormat_Crc0;                
        
            pCrcHdrCfg->local_hdr_len ++; 
        }

		if (numTrCh >= 6)
		{
            pCrcHdrCfg->dc_blk_len  =   trChLen[5];                
            pCrcHdrCfg->dc_blks     =   1;                
            pCrcHdrCfg->dc_crc          =   Bcp_CrcFormat_Crc0;                

            pCrcHdrCfg->local_hdr_len ++; 
        }
	}
	else//CRC
	{
        pCrcHdrCfg->va_blk_len      =   tbSize;

        if (radioStd == Bcp_RadioStd_WCDMA_R99)
            pCrcHdrCfg->va_blks     =   numTb;
        else
            pCrcHdrCfg->va_blks     =   1;

        if (radioStd == Bcp_RadioStd_LTE)
            pCrcHdrCfg->va_crc      =   Bcp_CrcFormat_Crc24a;   
        else if (radioStd == Bcp_RadioStd_WIMAX_802_16E)
            pCrcHdrCfg->va_crc      =   Bcp_CrcFormat_Crc16;   
        else if (radioStd == Bcp_RadioStd_WCDMA_R99)
        {
            if (crcSize == 0)
                pCrcHdrCfg->va_crc  =   Bcp_CrcFormat_Crc0;   
            else if (crcSize == 8)
                pCrcHdrCfg->va_crc  =   Bcp_CrcFormat_Crc8;   
            else if (crcSize == 12)
                pCrcHdrCfg->va_crc  =   Bcp_CrcFormat_Crc12;   
            else if (crcSize == 16)
                pCrcHdrCfg->va_crc  =   Bcp_CrcFormat_Crc16;   
            else if (crcSize == 24)
                pCrcHdrCfg->va_crc  =   Bcp_CrcFormat_Crc24b;   
            else
            {
                Bcp_osalLog ("Wrong CRC size \n");                    
                return;                    
            }
        }
        else
            pCrcHdrCfg->va_crc      =   Bcp_CrcFormat_Crc16;   
        pCrcHdrCfg->local_hdr_len ++; 
    }
            
    return;        
}

