/*
 * edma_simpleTest.h
 *
 *  Created on: Apr 13, 2012
 *      Author: x0177609
 */

#ifndef EDMALLD_H_
#define EDMALLD_H_

#ifndef edma_object_t

#define nbrChan	4		//nbr of channels in the edma object


typedef struct channel_object
{
	uint32_t chanNum;		//Channel number to open
	uint32_t tccCh;							//Type of channel (DMA, QDMA,...)
	EDMA3_DRV_TrigMode triggerType;			//type of trigger mode used for transfer
	EDMA3_DRV_SyncType syncType;			// type of synchronization used for a count b count c count
	uint32_t aCnt;							//size of a 1D table to transfer	(1st Dimension) in bytes
	uint32_t bCnt;							//number of 1D table to transfer 	(2nd Dimension)
	uint32_t cCnt;							//number of 2D table to transfer	(3rd Dimension)
	uint16_t srcBoffset;						//table B offset (if table source address is 0x0001, next one is 0x0001+srcBoffset)
	uint16_t srcCoffset;						//table C offset (if table source address is 0x0001, next one is 0x0001+srcCoffset)
	uint16_t dstBoffset;						//table B offset (if table destination address is 0x0001, next one is 0x0001+dstBoffset)
	uint16_t dstCoffset;						//table C offset (if table destination address is 0x0001, next one is 0x0001+dstCoffset)
	uint32_t srcAddr;					//source table to be transfered
	uint32_t dstAddr;					//destination table to receive data
	EDMA3_RM_EventQueue queue;				//queue for the event
	int chainChan;
	EDMA3_DRV_ChainOptions	opt;
} channel_object;


typedef struct edma_object
{
	EDMA3_DRV_Handle hEdma;
	uint16_t edma_nbr;						// EDMA number (0 to 2 because 3 EDMA are on the board)
	channel_object	chan[nbrChan];
	EDMA3_DRV_Result isError;				//state of the object
} edma_object;



#endif

#define BUFFSIZE 32
//#define coreNum	0
//#define instNum	1
//#define data	0xAB

//	void edma_config(edma_object_t *EdmaObj);
//	void edma_createChan(edma_object_t *EdmaObj, int16_t *Src, int16_t *Dst);
//	void edma_start(edma_object_t EdmaObj);

	void init_buffers(void);
	EDMA3_DRV_Handle edmalldinit (unsigned int edma3Id, EDMA3_DRV_Result *errorCode, unsigned int region);
	void edmaOpenChannels(EDMA3_DRV_Handle* hEdma, channel_object* chanObj);
	void edma_start(EDMA3_DRV_Handle* hEdma,uint32_t chanID, EDMA3_DRV_TrigMode triggerType);
	void edma_wait(EDMA3_DRV_Handle* hEdma, uint32_t* tccCh);
	unsigned short edma_check(EDMA3_DRV_Handle* hEdma, uint32_t* tccCh);
	void edma_release(EDMA3_DRV_Handle* hEdma, unsigned int edma3Id);
	void EDMA_intr_init(EDMA3_DRV_Handle* hEdma, uint32_t eventChannel);


#endif /* EDMALLD_H_ */

