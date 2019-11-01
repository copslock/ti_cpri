/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "../pautest.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif

/* Add/Delete MAC and PDSP routing test
 * This test tests the LLD Pa_addMac and Pa_delHandle functions, as well as the
 * PDSP firmware for routing mac packets with or without VLAN and MPLS. 
 * This test also verifies the MAC router scenario in which the MAC header of 
 * ingress packet is replaced with pre-defined MAC header.
 * This test has the following sub-tests
 *   - Test the LLD for the ability to determine if a new entry invalidates a previous entry
 *   - Test the LLD to enter increasingly strict match requirements (dst, src, vlan, ethertype, mpls)
 *   - Test the LLD when entering more entries than configured for
 *   - Test the LLD and firmware for the ability to configure command set
 *   - Test the LLD and firmware for various PASS global configuration
 *   - Test the firmware for the ability to take a burst of Pa_addMac commands
 *   - Test the firmware for the routing of matches and next step fail routes
 *   - Test the firmware for the ability to detect LLC/SNAP errors
 *   - Test the firmware for the ability to handle inner and outer VLAN tags
 *   - Test the firmware for the ability to break out of too many nested VLAN tags
 *   - Test the firmware for the ability to handle MPLS packets with a single tag
 *   - Test the firmware for the ability to handle MPLS packets with multiple tags
 *   - Test the firmware for the ability to parse unsupported protocol such as ARP
 *   - Test the firmware for the ability to take a burst of data packets
 * 	 - Test the firmware for correct header offset calculation
 *   - Test the LLD/firmware for the ability to delete entries
 *   - Test the LLD/firmware for the ability to detect and deliver the 802.1ag packets
 *   - Test the firmware for the ability to execute command set with blind patch command
 *   - Test the LLD/firmware for Ingress default route operation
 *   - Test the LLD/firmware for Enhanced QoS routing in Ingress direction
 */

#define TEST_ARP
 
static char *tfName = "paTestL2Routing";
 
/* General purpose queue usage */
#define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
#define Q_CMD_REPLY  		  1		/* Replies from PA routed here */
#define Q_DPKT_RECYCLE		  2		/* Data packet recycle queue */
#define Q_MATCH		  	      3		/* Packets of interfcae 0 from PA which match a lookup criteria */
#define Q_MATCH2		  	  4		/* Packets of interface 1 from PA which match a lookup criteria */
#define Q_MATCH3		  	  5		/* Packets of interface 2 from PA which match a lookup criteria */
#define Q_MATCH4		  	  6		/* Packets of interface 3 from PA which match a lookup criteria */
#define Q_NFAIL		          7		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
#define Q_PARSE_ERR		      8		/* Packets which resulted in a parse error */
#define Q_802_1AG             9     /* 802.1ag queue */
#define Q_QoS_BASE            10    /* First QoS queue */
#define Q_IF_BASE             3     /* First EMAC Interfcae queue */
#define Q_DROUTE              20    /* Default Route queue */
#define Q_BOUNCE_DDR          21    /* Queue Bounce DDR queue */
#define Q_BOUNCE_MSMC         22    /* Queue Bounce MSMC queue */

#define T2_NUM_QoS_QUEUES     8
#define T2_NUM_IF_QUEUES      4
#define T2_NUM_EQoS_QUEUES    16
 
/* 802_1ag packet index */
#define  T2_802_1AG_PKT_INDEX           20
#define  T2_802_1AG_FAIL_PKT_INDEX      21

/* Default route packet index */
#define  T2_DR_MC_PKT_INDEX             30
#define  T2_DR_BC_PKT_INDEX             31
#define  T2_DR_UC_PKT_INDEX             32
#define  T2_DR_PC_MC_PKT_INDEX          33
#define  T2_DR_PC_BC_PKT_INDEX          34
 
/* The number of PA handles maintained by this test */
#define T2_NUM_LOCAL_HANDLES	64

/* The total number of buffers with linked descriptors */
#define TOTAL_BUFS   (TF_LINKED_BUF_Q1_NBUFS + TF_LINKED_BUF_Q2_NBUFS + TF_LINKED_BUF_Q3_NBUFS)
 
 /* Commands to the PA are verified through the value in swinfo0.
  * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T2_CMD_SWINFO0_TYPE_MASK        0xffff0000
#define T2_CMD_SWINFO0_ID_MASK          0x0000ffff
#define T2_CMD_SWINFO0_ADD_ID  		    0x11110000  /* Identifies add mac command */
#define T2_CMD_SWINFO0_DEL_ID  		    0x22220000  /* Identifies del mac command */
#define T2_CMD_SWINFO0_STATS_REQ_ID	    0x33330000	/* Identifies the req stats command */
#define T2_CMD_SWINFO0_CMDSET_CFG_ID    0x44440000  /* Identifies the cmd set command */
#define T2_CMD_SWINFO0_PKT_ID		    0x55550000  /* Identifies the packet as a data packet */
#define T2_CMD_SWINFO0_EROUTE_CFG_ID   	0x66660000  /* Exception Routes configuration */
#define T2_CMD_SWINFO0_802_1AG_CFG_ID   0x77770000  /* 802.1ag detector configuration */
#define T2_CMD_SWINFO0_DROUTE_CFG_ID   	0x88880000  /* Default Routes configuration */
#define T2_CMD_SWINFO0_DR_GLOB_CFG_ID   0x88890000  /* Default Route Global configuration */
#define T2_CMD_SWINFO0_GLOB_CFG_ID      0x888A0000  /* Global configuration */
#define T2_CMD_SWINFO0_EQoS_CFG_ID   	0x99990000  /* EQoS Mode configuration */
 
#define T2_NUM_PACKET_ITERATIONS        1    /* 3 */

//extern int testCommonSetGblConfig(	paCtrlInfo_t   ctrlInfo, uint32_t replyId);

#include "test2pkts.h"
 
paSysStats_t paTestL2ExpectedStats;  /* Expected stats results */
 
 /* 32 L2 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T2_L2_HANDLE_UNCONFIGURED = 0,
	T2_L2_HANDLE_PENDING_ACK,
	T2_L2_HANDLE_ACTIVE,
	T2_L2_HANDLE_DISABLED
};

typedef struct t2Handles_s  {

 	paHandleL2L3_t  paHandle;     /* The handle returned by the PA LLD */

	unsigned int	state;        /* T2_L2_HANDLE_UNCONFIGURED = handle not configured
								   * T2_L2_HANDLE_PENDING_ACK = handle configured and sent to pa
								   * T2_L2_HANDLE_ACTIVE = handle creation acknowledged by pa */
	
} t2Handles_t;

/* Static test configuration - routing of matching packets to the same queue and 
 * distinguished by swinfo0 */
typedef struct t2EthAndRoute_s  {
	
	paEthInfo_t  eth;
	uint32_t       swinfo0;

} t2EthAndRoute_t;
 
const t2EthAndRoute_t  t2EthAndSwinfo[] =  {
	
	{ {  {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  	/* Entry 0: Route on dest mac only */
		 {  0x11, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 0 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID },
		
	{ {  {  0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },  	/* Entry1: Route on dest/src mac */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 1 */
		   0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID+1 },
		
	{ {  {  0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },	/* Entry 2: route on dest/src mac, vlan */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 2 */
		   0x888, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID+2 },
		
	{ {  {  0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },	/* Entry 3: route on dest/src mac, vlan */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },   /* and ethertype = MPLS unicast */
		   0x888, 0x8847, 0, 0}, T2_CMD_SWINFO0_PKT_ID+3 },		/* PA entry 3 */
		
	{ {  {  0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },	/* Entry 4: route on dest/src mac, vlan */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },   /* and ethertype = MPLS multicast */
		   0x888, 0x8848, 0, 0}, T2_CMD_SWINFO0_PKT_ID+4 },		/* PA entry 4 */
		
	{ {  { 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },	/* Entry 5: route on dest/src mac, vlan */
		 { 0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },    /* ethertype = MPLS unicast, and MPLS tag */
		   0x888, 0x8847, 0x012345, 0}, T2_CMD_SWINFO0_PKT_ID+5 },	/* PA entry 5 */
		   
	{ {  { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44 },    /* Entry 6: more specific entry then 7 */
		 { 0x30, 0x33, 0x33, 0x33, 0x33, 0x33 },	/* PA entry 6 */
		   0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID+6 },
		 	 
	{ {  { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 },    /* Entry 7 route on source mac only */
		 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 7 */
		   0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID+7 },
		 
	{ {  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* Entry 8 route on vlan only */
		 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 8 */
		   0x999, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID+8 },
		 
	{ {  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* Entry 9 route on ethertype only */
		 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 9 */
		   0, 0x999, 0, 0}, T2_CMD_SWINFO0_PKT_ID+9 },
		 
         
#ifndef TEST_ARP         
	{ {  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    /* Entry 10 route on mpls tag only */
		 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 10 */
		   0, 0, 0x11111, 0}, T2_CMD_SWINFO0_PKT_ID+10 },
#else           
	{ {  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    /* Entry 10 route on ARP only */
		 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 10 */
		   0x0, 0x806, 0, 0}, T2_CMD_SWINFO0_PKT_ID+10 },
#endif     
	{ {  {  0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },	/* Entry 11: route on dest/src mac, vlan, ethtype = 0x8864 */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 11 */
		    0x888, 0x8864, 0, 0}, T2_CMD_SWINFO0_PKT_ID+11 },
    /* Pa_addMac2 Entries for QoS */         
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x04 },  	/* Entry 12: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 12 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 12 },
		
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x05 },  	/* Entry 13: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 13 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 13 },
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x06 },  	/* Entry 14: Route on src mac and vlan ID */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 14 */
		    0x777, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 14 },
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x06 },  	/* Entry 15: Route on src and dest mac and VLAN ID */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x77 },	/* PA entry 15 */
		    0x888, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 15 },
            
    /* Pa_addMac2 Entries for Interface-based Routing */         
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x07 },  	/* Entry 16: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 16 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 16 },
		
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x08 },  	/* Entry 17: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 17 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 17 },
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x09 },  	/* Entry 18: Route on src mac and VLAN ID */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 18 */
		    0x888, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 18 },
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x0a },  	/* Entry 19: Route on src and dest mac and etherType */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x88 },	/* PA entry 19 */
		    0x00, 0x800, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 19 },
            
    /* Pa_addMac2 Entries for EQoS */         
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x10 },  	/* Entry 20: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 20 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 20 },
		
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x11 },  	/* Entry 21: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 21 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 21 },
            
	{ {  {  0x0e, 0xa0, 0x01, 0x02, 0x03, 0x12 },  	/* Entry 22: Route on src mac only */
		 {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	/* PA entry 22 */
		    0, 0, 0, 0}, T2_CMD_SWINFO0_PKT_ID + 22 },
      
 };
 
/*
 *  Use both pre-determined and system-allocated LUT1 entry index 
 *  Note: The pre-determined LUT1 entry index should be consistent with the system-allocated one.
 *        It is not recommended to mix those two modes in the application.
 */ 
#ifndef __LINUX_USER_SPACE 
const int  t2EthIndex[] =  {
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 0 */
                62,                             /* entry 1 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 2 */
                60,                             /* entry 3 */
                59,                             /* entry 4 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 5 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 6 */
                56,                             /* entry 7 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 8 */
                54,                             /* entry 9 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 10 */
                40,                             /* entry 11 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 12 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 13 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 14 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 15 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 16 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 17 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 18 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 19 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 20 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 21 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 22 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 23 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 24 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 25 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 26 */
};      
#else
const int  t2EthIndex[] =  {
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 0 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 1 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 2 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 3 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 4 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 5 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 6 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 7 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 8 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 9 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 10 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 11 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 12 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 13 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 14 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 15 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 16 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 17 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 18 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 19 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 20 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 21 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 22 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 23 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 24 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 25 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 26 */
};      
#endif
/* 802.1ag Detector Configuration */
#define T2_802_1AG_CFG_INDEX_DISABLE     0
#define T2_802_1AG_CFG_INDEX_STANDARD    1
#define T2_802_1AG_CFG_INDEX_DRAFT       2
#define T2_802_1AG_CFG_NUM               (T2_802_1AG_CFG_INDEX_DRAFT + 1)

static  pa802p1agDetConfig_t  t2E802p1agDetCfg[T2_802_1AG_CFG_NUM] = 
        {
            /* Config 0: Disable */
            {
                0                               /* ctrlBitMap */
            },
            
            /* Config 1: Enable, standard */
            {
                pa_802_1ag_DETECT_ENABLE |      /* ctrlBitMap */
                pa_802_1ag_DETECT_STANDARD
            },
            
            /* Config 1: Enable, draft */
            {
                pa_802_1ag_DETECT_ENABLE        /* ctrlBitMap */
            }
            
        };
        
/* Exception Route configurations */ 
#define T2_NUM_EXCEPTION_ROUTES     1
static int t2ErouteTypes[] = {
    pa_EROUTE_802_1ag
};
 
static paRouteInfo_t t2Eroutes[] = {
 
    /* 802.1ag Packet Route */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_802_1AG + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T2_802_1AG_PKT_INDEX + T2_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    }
};             


/* Default Route configurations */ 
static paDefRouteConfig_t  t2DefRouteCfg[] =
{
    /* Entry 0 */
    {
        pa_EMAC_IF_DEFAULT_ROUTE_MC_ENABLE |    /* ctrlBitMap */
        pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE |
        pa_EMAC_IF_DEFAULT_ROUTE_UC_ENABLE, 
          
        pa_EMAC_PORT_1,                         /* EMAC Port number */
        {
            /* routeInfo */
            /* Multicast Default Route */
            {   pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,   /* Valid Bit map */
 	            pa_DEST_HOST,		                /* Dest */
 	            0,					                /* Flow ID */
 	            Q_DROUTE + TF_FIRST_GEN_QUEUE,      /* queue */
 	            -1,					                /* Multi route */
 	            T2_CMD_SWINFO0_PKT_ID +             /* sw Info 0 */
                T2_DR_MC_PKT_INDEX,
                0,                                  /* sw Info 1 */       
                0,                                  /* customType : not used */         
                0,                                  /* customIndex: not used */     
                pa_EMAC_PORT_1,                     /* pkyType_emacCtrl */    
                (paCmdInfo_t *) NULL,               /* No commands */
                0,                                  /* No Prioritytype */
                NULL                                /* No eflow */
            },
            /* Broadcast Default Route */
            {   pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,   /* Valid Bit map */
                pa_DEST_HOST,                       /* Dest */
                0,                                  /* Flow ID */
 	            Q_DROUTE + TF_FIRST_GEN_QUEUE,      /* queue */
                -1,                                 /* Multi route */
 	            T2_CMD_SWINFO0_PKT_ID +             /* sw Info 0 */
                T2_DR_BC_PKT_INDEX,
                0,                                  /* sw Info 1 */       
                0,                                  /* customType : not used */         
                0,                                  /* customIndex: not used */     
                pa_EMAC_PORT_1,                     /* pkyType_emacCtrl */    
                (paCmdInfo_t *) NULL,               /* No commands */
                0,                                  /* No Prioritytype */
                NULL                                /* No eflow */
            },
            /* No match Unicast Default Route */
            {  
                pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,   /* Valid Bit map */
                pa_DEST_HOST,                       /* Dest */
                0,                                  /* Flow ID */
 	            Q_DROUTE + TF_FIRST_GEN_QUEUE,      /* queue */
                -1,                                 /* Multi route */
 	            T2_CMD_SWINFO0_PKT_ID +             /* sw Info 0 */
                T2_DR_UC_PKT_INDEX,
                0,                                  /* sw Info 1 */       
                0,                                  /* customType : not used */         
                0,                                  /* customIndex: not used */     
                pa_EMAC_PORT_1,                     /* pkyType_emacCtrl */    
                (paCmdInfo_t *) NULL,               /* No commands */
                0,                                  /* No Prioritytype */
                NULL                                /* No eflow */
            }
        }
    },
    
    
    /* Entry 1 */
    {
        pa_EMAC_IF_DEFAULT_ROUTE_MC_ENABLE |    /* ctrlBitMap */
        pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE |
        pa_EMAC_IF_DEFAULT_ROUTE_MC_PRE_CLASSIFY_ENABLE |
        pa_EMAC_IF_DEFAULT_ROUTE_BC_PRE_CLASSIFY_ENABLE,
          
        pa_EMAC_PORT_3,                             /* EMAC Port number */
        {
            /* routeInfo */
            /* Multicast Default Route */
            {   pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,   /* Valid Bit map */
 	            pa_DEST_HOST,		                /* Dest */
 	            0,					                /* Flow ID */
 	            Q_DROUTE + TF_FIRST_GEN_QUEUE,      /* queue */
 	            -1,					                /* Multi route */
 	            T2_CMD_SWINFO0_PKT_ID +             /* sw Info 0 */
                T2_DR_PC_MC_PKT_INDEX,
                0,                                  /* sw Info 1 */       
                0,                                  /* customType : not used */         
                0,                                  /* customIndex: not used */     
                pa_EMAC_PORT_3,                     /* pkyType_emacCtrl */    
                (paCmdInfo_t *) NULL,               /* No commands */
                0,                                  /* No Prioritytype */
                NULL                                /* No eflow */
            },
            /* Broadcast Default Route */
            {   pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,   /* Valid Bit map */
                pa_DEST_HOST,                       /* Dest */
                0,                                  /* Flow ID */
 	            Q_DROUTE + TF_FIRST_GEN_QUEUE,      /* queue */
                -1,                                 /* Multi route */
 	            T2_CMD_SWINFO0_PKT_ID +             /* sw Info 0 */
                T2_DR_PC_BC_PKT_INDEX,
                0,                                  /* sw Info 1 */       
                0,                                  /* customType : not used */         
                0,                                  /* customIndex: not used */     
                pa_EMAC_PORT_3,                     /* pkyType_emacCtrl */    
                (paCmdInfo_t *) NULL,               /* No commands */
                0,                                  /* No Prioritytype */
                NULL                                /* No eflow */
            },
            /* No match Unicast Default Route */
            {  
                0,                                  /* Valid Bit map */
                pa_DEST_DISCARD,                    /* Dest */
                0,                                  /* Flow ID */
                0,                                  /* queue */
                -1,                                 /* Multi route */
 	            0,                                  /* sw Info 0 */
                0,                                  /* sw Info 1 */       
                0,                                  /* customType : not used */         
                0,                                  /* customIndex: not used */     
                0,                                  /* pkyType_emacCtrl */    
                (paCmdInfo_t *) NULL,               /* No commands */
                0,                                  /* No Prioritytype */
                NULL                                /* No eflow */
            }
        }
    }
};          

#define T2_NUM_DR_PORTS         sizeof(t2DefRouteCfg)/sizeof(paDefRouteConfig_t)

/* EQoS Mode configurations (Ingress only) */ 
static paEQosModeConfig_t  t2EQoSModeCfg[] =
{
    /* Entry 0 */
    /* Default Priority only */
    {
        
        0,                                  /* ctrlBitMap */
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* pbitMap */
         {0, 0}, {0, 0}, {0, 0}, {0, 0}},          
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* dscpMap */
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0}},
        pa_EMAC_PORT_0,                      /* port number */
        0,                                  /* ingressDefPri */
        0,                                  /* vlanId (not used) */
        0,                                  /* flowBase (not used) */
        0                                   /* queueBase (not used) */
    },
    
    /* Entry 1 */
    /* P-Bit mode */
    {
        
        pa_IF_EQoS_ROUTE_DP_BIT_MODE,       /* ctrlBitMap */
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* pbitMap */
         {0, 1}, {0, 1}, {0, 2}, {0, 2}},          
        {{0, 8},  {0, 8},  {0, 8},  {0, 8}, /* dscpMap */
         {0, 8},  {0, 8},  {0, 8},  {0, 8}, 
         {0, 9},  {0, 9},  {0, 9},  {0, 9}, 
         {0, 9},  {0, 9},  {0, 9},  {0, 9}, 
         {0, 10}, {0, 10}, {0, 10}, {0, 10}, 
         {0, 10}, {0, 10}, {0, 10}, {0, 10}, 
         {0, 11}, {0, 11}, {0, 11}, {0, 11}, 
         {0, 11}, {0, 11}, {0, 11}, {0, 11}, 
         {0, 12}, {0, 12}, {0, 12}, {0, 12}, 
         {0, 12}, {0, 12}, {0, 12}, {0, 12}, 
         {0, 13}, {0, 13}, {0, 13}, {0, 13}, 
         {0, 13}, {0, 13}, {0, 13}, {0, 13}, 
         {0, 14}, {0, 14}, {0, 14}, {0, 14}, 
         {0, 14}, {0, 14}, {0, 14}, {0, 14}, 
         {0, 15}, {0, 15}, {0, 15}, {0, 15}, 
         {0, 15}, {0, 15}, {0, 15}, {0, 15}},
        pa_EMAC_PORT_1,                     /* port number */
        7,                                  /* ingressDefPri */
        0,                                  /* vlanId (not used) */
        0,                                  /* flowBase (not used) */
        0                                   /* queueBase (not used) */
    },
    
    /* Entry 2 */
    /* P-Bit mode with priority override */
    {
        
        pa_IF_EQoS_ROUTE_DP_BIT_MODE |      /* ctrlBitMap */
        pa_IF_EQoS_PRIORITY_OVERRIDE_ENABLE,
        {{0, 3}, {0, 3}, {0, 3}, {0, 3},    /* pbitMap */
         {0, 4}, {0, 4}, {0, 5}, {0, 5}},          
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* dscpMap */
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0}},
        pa_EMAC_PORT_2,                     /* port number */
        2,                                  /* ingressDefPri */
        0,                                  /* vlanId (not used) */
        0,                                  /* flowBase (not used) */
        0                                   /* queueBase (not used) */
    },
    
    /* Entry 3 */
    /* DSCP mode */
    {
        0,                                  /* ctrlBitMap */
        {{0, 6}, {0, 6}, {0, 6}, {0, 6},    /* pbitMap */
         {0, 7}, {0, 7}, {0, 7}, {0, 7}},          
        {{0, 15}, {0, 14}, {0, 13}, {0, 12},/* dscpMap */
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}, 
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}}, 
        pa_EMAC_PORT_3,                     /* port number */
        3,                                  /* ingressDefPri */
        0,                                  /* vlanId (not used) */
        0,                                  /* flowBase (not used) */
        0                                   /* queueBase (not used) */
    }
};    
    
#define T2_NUM_EQoS_PORTS         sizeof(t2EQoSModeCfg)/sizeof(paEQosModeConfig_t)
    
const t2EthAndRoute_t  t2EthAndSwinfoFail = {
		 { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },			/* This would steal matches from entry 6 if allowed */
		   { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33 },	  		/* This entry is not passed to PA */
		     0, 0, 0, 0}, 0xDEADDEAD };						/* !!! No PA Entry !!!! */
            
             
/* A single non-const entry is used to test handling entering too many table elements
 * as well as delete and add */
t2EthAndRoute_t  t2VarEthAndRoute =  { {  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
									      { 0x45, 0x45, 0x45, 0x45, 0x45, 0x00 },
                        			        0, 0, 0, 0},  0 };
/*
 * Command Set to test MAC router
 *
 */         
#define T2_CMDSET_INDEX         20
#define T2_CMDSET_NUM_CMDS      1
 
static uint8_t t2MacRouteHdr[] = 
{
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
    0x08, 0x00
}; 
                                    
static  paPatchInfo_t t2PatchCmd = 
        {
            pa_PATCH_OP_MAC_HDR,    /* ctrlBitfield */  
            sizeof(t2MacRouteHdr),  /* nPatchBytes */
            sizeof(t2MacRouteHdr),  /* totalPatchSize */
            0,                      /* offset */
            t2MacRouteHdr           /* Pointer to the patch data */
        };
      
static paCmdInfo_t t2CmdSet20[] =
{
    
    /* Command 0: Insert Bytes */
    {
        pa_CMD_PATCH_DATA,
        {
            {
                0,                            /* ctrlBitfield */
                0,                            /* nPatchBytes */
                0,                            /* totalPatchSize */
                0,                            /* offset */  
                0                             /* Pointer to the patch data */
            }
        }
    }
    
}; 

static paCmdInfo_t t2CmdSetCmd =
    {
        pa_CMD_CMDSET,
        {
             
            {
                T2_CMDSET_INDEX               /* Command set index */
            }                   
        }    
    };
    
#define T2_CMDSET2_INDEX         30
#define T2_CMDSET2_NUM_CMDS      1
 
static  paCmdNextRoute_t t2NextRouteCmd = 
        {
            0,                      /* ctrlBitfield */  
            pa_DEST_EMAC,           /* dest */
            pa_EMAC_PORT_0,         /* pktType_emacCtrl */
            0,                      /* flowId */
            0,                      /* queue */
            0,                      /* swInfo0 */
            0,                      /* swInfo1 */
            0                       /* multiRouteIndex */
        };
      
static paCmdInfo_t t2CmdSet30[] =
{
    
    /* Command 0: Next Route Command */
    {
        pa_CMD_NEXT_ROUTE,
        {
            {
                0,                            /* ctrlBitfield */
                0,                            /* nPatchBytes */
                0,                            /* totalPatchSize */
                0,                            /* offset */  
                0                             /* Pointer to the patch data */
            }
        }
    }
    
}; 

static paCmdInfo_t t2CmdSetCmd2 =
    {
        pa_CMD_CMDSET,
        {
             
            {
                T2_CMDSET2_INDEX              /* Command set index */
            }                   
        }    
    };
    
static	paQueueBounceConfig_t t2QueueBounceCfg =
	        {
	            1,      /* Enable */
                Q_BOUNCE_DDR  + TF_FIRST_GEN_QUEUE,             /* ddrQueueId */
                Q_BOUNCE_MSMC + TF_FIRST_GEN_QUEUE,             /* msmcQueueId */
                TF_PA_TX_QUEUE_BASE,                            /* hwQueueBegin */
                TF_PA_TX_QUEUE_BASE + NSS_NUM_TX_QUEUES - 1,    /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_MSMC,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_DDR,     /* QoS mode */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_MSMC     /* All traffics */
                }
	        };

static  paSysConfig_t  t2GlobalCfg =
        {
            NULL,                   /* pProtoLimit */
            NULL,                   /* pOutIpReassmConfig */
            NULL,                   /* pInIpReassmConfig */
            NULL,                   /* pCmdSetConfig */
            NULL,                   /* pUsrStatsConfig */
            NULL,                   /* pQueueDivertConfig */
            NULL,                   /* pPktControl */
            &t2QueueBounceCfg       /* pQueueBounceConfig */
        };

/* Prototype required due to circular function calling */									        
paTestStatus_t t2CheckStats (tFramework_t *tf, paTest_t *pat, Bool clear, t2Handles_t *l2Handles);			


static Cppi_HostDesc *formDataPacket (tFramework_t *tf, paTest_t *pat, int pktIdx, uint8_t *expectedPktCount)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	
	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}
	
	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QfreeDesc;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
  	
  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t2PktTestInfo[pktIdx].pkt)), t2PktTestInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t2PktTestInfo[pktIdx].pktLen);
  	
  	return (hd);
}
  
								        
/* Recycle delete commands and command recycles  */
int t2StateDel (tFramework_t *tf, paTest_t *pat, t2Handles_t *l2Handles)
{
	Cppi_HostDesc    *hd;
	paEntryHandle_t   reth;
	paReturn_t        paret;
	int				  htype;
	int  			  cmdDest;
	int 			  i;
		
	/* Don't send the command until half of the rx buffers are available. The command replies will always be
	 * sourced from QLinkedBuf1 and arrive in Q_CMD_REPLY. Delay the command until there are enough buffers available
	 * to send and leave a 50% overhead.*/
	for (i = 0; (i < 100) && (Qmss_getQueueEntryCount (tf->QLinkedBuf1) < (TF_LINKED_BUF_Q1_NBUFS >> 1)); i++)  {		
		if ((Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) + Qmss_getQueueEntryCount(tf->QLinkedBuf1)) >= TF_LINKED_BUF_Q1_NBUFS)
		  break;
	}
			
	utilCycleDelay (1000);	
    testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, NULL);
	while (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)  {
				
		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
		if ((hd->softwareInfo0 & 0xffff0000u) != T2_CMD_SWINFO0_DEL_ID)  {
			System_printf ("%s (%s:%d): Found packet in PA command reply queue without delete ID (found 0x%08x)", tfName, __FILE__, __LINE__, hd->softwareInfo0);
			return (-1);
		}
				
		/* Send the reply back to PA to let the driver know the command has been accepted */
		paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
				
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): paForwardResult returned error %d in response to pa_DelHandle reply from PA for handle #%d\n", tfName, __FILE__, __LINE__, hd->softwareInfo0 & 0xffff);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			return (-1);
		}
			
		l2Handles[hd->softwareInfo0 & 0xffff].state = T2_L2_HANDLE_DISABLED;
				
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			return (-1);
		}
			
	}
		
	/* The command recycle descriptor/buffer */
	while (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_RECYCLE]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			return (-1);
		}
	}
		
	
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Timeout waiting for free descriptor/buffer queue to fill to halfway point\n", tfName, __FILE__, __LINE__);
		return (-1);
	}
	
	return (0);
		
}
	
paTestStatus_t paTestL2Delete (tFramework_t *tf, paTest_t *pat, t2Handles_t *l2Handles, uint32_t numHandles, uint32_t startId)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
 	uint16_t		cmdSize;
	int 			i;
	//volatile int    mdebugWait = 0;
	paTestStatus_t status = PA_TEST_PASSED;
	
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
	cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];

	
	/* Delete the handles as needed */
	for (i = startId; i < numHandles; i++)  {
		/* do not attempt to delete if handle is null */
		if ( (l2Handles[i].state == T2_L2_HANDLE_UNCONFIGURED) ||
			 (l2Handles[i].state == T2_L2_HANDLE_DISABLED) )
			continue;

		cmdReply.replyId = T2_CMD_SWINFO0_DEL_ID + i;
		hd = testCommonDelHandle (tf, &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, &cmdReply, &cmdDest, &cmdSize, &paret);
		
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): PA LLD returned error code %d on handle deletion\n", tfName, __FILE__, __LINE__,paret);
			status = PA_TEST_FAILED;
			break;
		}
		
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): No descriptor available for del handle command\n", tfName, __FILE__, __LINE__);
			status = PA_TEST_FAILED;
			break;
		}
		
        /* mdebugHaltPdsp (0); */
		/* Send the command to the PA */
		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
		paTestL2ExpectedStats.classify1.nPackets += 1;

		/* Wait to send the command until half of the rx buffers are available */
		if (t2StateDel (tf, pat, l2Handles))  {
			status = PA_TEST_FAILED;
			break;
		}
		
		//while (mdebugWait);
	}
	
	return (status);

}

void paTestL2RecoverAndExit (tFramework_t *tf, paTest_t *pat, t2Handles_t *l2Handles, paTestStatus_t status, Bool doStats)
{
    int i, j, m;
	/* Delete all the handles */
	status = paTestL2Delete(tf, pat, l2Handles, T2_NUM_LOCAL_HANDLES, 0);

	/* Verify the status is as expected */
	if (status == PA_TEST_FAILED) {
  	   /* Test result */
   	   pat->testStatus = status;
	   Task_exit();
    }

	/* Give some time for remaining commands to complete */
	for (i = 0; i < 100; i++)  {
		
		for (j = m = 0; j < T2_NUM_LOCAL_HANDLES; j++) {
            /* don't get the state accumulation for unconfigured L2 Handles*/
			if (l2Handles[j].state == T2_L2_HANDLE_UNCONFIGURED)
				continue;
			
			if (l2Handles[j].state != T2_L2_HANDLE_DISABLED)
				m += 1;
	    }
				
		if (m)  {
			if (t2StateDel (tf, pat, l2Handles))  {
				status = PA_TEST_FAILED;
				break;
			}
			utilCycleDelay (100);
		} else
			break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Unable to delete all L2 handles. %d remain undeleted\n", tfName, __FILE__, __LINE__, m);
		status = PA_TEST_FAILED;
	}

	/* Verify the stats are as expected */
	if (t2CheckStats (tf, pat, TRUE, l2Handles) == PA_TEST_FAILED)
	  status = PA_TEST_FAILED;
	
	/* Test result */
	pat->testStatus = status;
	
	/* Return */
	Task_exit();
}		
				
/* Look for command replies from PA */					     
void t2L2CmdRep (tFramework_t *tf, paTest_t *pat, t2Handles_t *localHandles, int *cmdReply)
{
	Cppi_HostDesc  *hd;
	uint32_t         *swInfo0;
	uint32_t 			swInfoCmd;
	unsigned int			lid;
	paReturn_t      paret;
	paEntryHandle_t reth;
	int				htype;
	int				cmdDest;

    testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, NULL);
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_REPLY]) > 0)  {
		
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_REPLY]))) & ~15);
		if (Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **) &swInfo0) == CPPI_EPIB_NOT_PRESENT)  {
			System_printf ("%s (%s:%d): Found descriptor in PA command reply queue without EIPB present, failing\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		swInfoCmd = (*swInfo0 & 0xffff0000u); 
        
        /* Add General Command Processing */
        /* Command Set Command */
        if((swInfoCmd == T2_CMD_SWINFO0_CMDSET_CFG_ID)   || 
           (swInfoCmd == T2_CMD_SWINFO0_EROUTE_CFG_ID)   ||
           (swInfoCmd == T2_CMD_SWINFO0_DROUTE_CFG_ID)   ||
           (swInfoCmd == T2_CMD_SWINFO0_DR_GLOB_CFG_ID)  ||
           (swInfoCmd == T2_CMD_SWINFO0_GLOB_CFG_ID)     ||
           (swInfoCmd == T2_CMD_SWINFO0_EQoS_CFG_ID)     ||
           (swInfoCmd == T2_CMD_SWINFO0_802_1AG_CFG_ID))
        {
            
		    /* Recycle the descriptor and buffer */
		    if (testCommonRecycleLBDesc (tf, hd))  {
			    System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			    paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		    }
            *cmdReply = TRUE;
            break;
        }
        
		/* Verify expected value in swinfo0 16 msbs */
		if ( (swInfoCmd != T2_CMD_SWINFO0_ADD_ID) && (swInfoCmd != T2_CMD_SWINFO0_DEL_ID) ) {
			System_printf ("%s (%s:%d): Found descriptor in PA command reply queue without command reply swinfo0\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Extract the local instance value */
		lid = *swInfo0 & 0xffffu;
		if (lid >= T2_NUM_LOCAL_HANDLES)  {
			System_printf ("%s (%s:%d): Received PA command reply for out of range local handle %d (max %d)\n", tfName, __FILE__, __LINE__, lid, T2_NUM_LOCAL_HANDLES);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Send the reply back to PA to let the driver know the command has been accepted */
		paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): paForwardResult returned error %d in response to paAddMac reply from PA\n", tfName, __FILE__, __LINE__, paret);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Make sure the handle returned by PA matches the local copy */
		if (localHandles[lid].paHandle != reth.l2l3Handle)  {
			System_printf ("%s (%s:%d): paForwardResult returned handle (0x%08x) that did match internal table value (0x%08x)\n", tfName, __FILE__, __LINE__, (uint32_t)(localHandles[lid].paHandle), (uint32_t) reth.l2l3Handle);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Recycle the descriptor and buffer */
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		if (swInfoCmd == T2_CMD_SWINFO0_ADD_ID)
			localHandles[lid].state = T2_L2_HANDLE_ACTIVE;
		else
			localHandles[lid].state = T2_L2_HANDLE_UNCONFIGURED;
	}
	
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_RECYCLE]) > 0)  {
		
		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_RECYCLE]))) & ~15);
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
	}
			
		
}
								     
									     
								     
									     
void paL2HandleError (tFramework_t *tf, paTest_t *pat, t2Handles_t *l2Handles, paReturn_t paret, Cppi_HostDesc *hd)
{	 	  
    /* Check paret before the descriptor. If paret indicates failure the descriptor will be NULL */                    
 	if (paret != pa_OK)  {
 		System_printf ("%s (%s:%d): testCommonAddMac failed, PA LLD error code = %d\n", tfName, __FILE__, __LINE__, paret);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
 	
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): testCommonAddMac failed due to unavailable free linked buffer descriptor\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
}


/* Check the stats */

paTestStatus_t t2CheckStats (tFramework_t *tf, paTest_t *pat, Bool clear, t2Handles_t *l2Handles)
{ 	
 	paSysStats_t   paStats1;
    paSysStats_t  *paStats = &paStats1; 
	paTestStatus_t  status;
    
    #ifndef NSS_GEN2
	Cppi_HostDesc  *hd;
	uint8_t		   *bp;
 	unsigned int	blen;
	int             i;

	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
 	cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];
 	
 	/* Check the PA stats to make sure they are set as expected */
 	cmdReply.replyId = T2_CMD_SWINFO0_STATS_REQ_ID;
 	if (testCommonRequestPaStats (tfName, tf, clear, tf->QLinkedBuf1, tf->QGen[Q_CMD_RECYCLE],  &cmdReply))  {
 		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
 	}
    
 	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, NULL);
		if (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	
	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
 		
 	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
	paStats = (paSysStats_t *)Pa_formatStatsReply (tf->passHandle, (paCmd_t)bp);
    
    #else
	  
	if (testCommonQueryPaStats (tfName, tf, TRUE, paStats))  {
		System_printf ("%s (%s:%d): testCommonQueryPaStats f\ailed\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
    #endif 
     
    if (testCommonCompareStats (tfName, (paSysStats_t *)&paTestL2ExpectedStats, paStats))
    	status = PA_TEST_FAILED;
    else
    	status = PA_TEST_PASSED;   

    #ifndef  NSS_GEN2      
     /* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tfName, __FILE__, __LINE__);
		status = PA_TEST_FAILED;
	}
    #endif
	
	return (status);
}

static paTestStatus_t t2ExceptionRoutes (tFramework_t *tf, paTest_t *pat)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    int             fCmdReply;
    
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
    
    
    /* Issue the exception route command */
    cmdReply.replyId  = T2_CMD_SWINFO0_EROUTE_CFG_ID;
	cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tf->tfFlowNum[0];
    for(i = 0; i < T2_NUM_EXCEPTION_ROUTES; i++)
        t2Eroutes[i].flowId = tf->tfFlowNum[0];    
    
 	hd = testCommonConfigExceptionRoute (tf, T2_NUM_EXCEPTION_ROUTES, t2ErouteTypes, t2Eroutes,  
 	                                     tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                                     &cmdReply, &cmdDest, &cmdSize, &paret, FALSE);
                                         
   if (hd == NULL)  {
   			
   	 System_printf ("%s: (%s:%d): Failure in ConfigExceptionRoute command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);
   	
   }								 
    
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, NULL, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
    
	if (i == 100)  {
		System_printf ("%s: (%s:%d): pa_ConfigExceptionRoute commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t2ConfigDefaultRoutes(tFramework_t *tf, paTest_t *pat, int clear)
{
	int 			i, j;
	Cppi_HostDesc  *hd = NULL;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    int             fCmdReply;
    
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
    
    /* Issue the default route configuration command */
    cmdReply.replyId  = T2_CMD_SWINFO0_DROUTE_CFG_ID;
	cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tf->tfFlowNum[0];
    if (!clear)
    {
 	    hd = testCommonConfigDefaultRoute (tf, T2_NUM_DR_PORTS, t2DefRouteCfg,  
 	                                       tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                                       &cmdReply, &cmdDest, &cmdSize, &paret);
    }
    else
    {
        paDefRouteConfig_t  defRouteCfg[T2_NUM_DR_PORTS];
        memset(defRouteCfg, 0, sizeof(defRouteCfg));
        for (i = 0; i < T2_NUM_DR_PORTS; i++)
        {
            defRouteCfg[i].port = t2DefRouteCfg[i].port;       
            for (j = 0; j < pa_DROUTE_MAX; j++)
            {
                defRouteCfg[i].dRouteInfo[j].dest = pa_DEST_DISCARD;
            }
        }
        
 	    hd = testCommonConfigDefaultRoute (tf, T2_NUM_DR_PORTS, defRouteCfg,  
 	                                       tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                                       &cmdReply, &cmdDest, &cmdSize, &paret);
    
    }        
                                         
    if (hd == NULL)  {
   			
   	  System_printf ("%s: (%s:%d): Failure in ConfigDefaultRoute command with error code = %d\n", tfName, __FILE__, __LINE__, paret);
   	  return (PA_TEST_FAILED);
   	
    }								 
    
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, NULL, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
    
	if (i == 100)  {
		System_printf ("%s: (%s:%d): ConfigDefaultRoutes commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t2DefaultRouteGlobalControl (tFramework_t *tf, paTest_t *pat, int enable)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paPacketControl2Config_t  pPktControl2;
	paSysConfig_t  paSysCfg;
    paCtrlInfo_t   ctrlInfo;
    int             fCmdReply;
    
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */

    memset(&paSysCfg, 0, sizeof(paSysConfig_t));  
    memset(&pPktControl2, 0,   sizeof(paPacketControl2Config_t)); 
    memset (&ctrlInfo, 0, sizeof (paCtrlInfo_t));
  
    pPktControl2.ctrlBitMap     = (enable)?pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE:0;
    pPktControl2.validBitMap    = pa_PKT_CTRL2_VALID_EMAC_IF_INGRESS_DEFAULT_ROUTE;        
    paSysCfg.pPktControl2 = &pPktControl2;

  	/* set System Global default configuration */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = paSysCfg;
    cmdReply.replyId  = T2_CMD_SWINFO0_DR_GLOB_CFG_ID;
	cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tf->tfFlowNum[0];
    
 	hd = testCommonGlobalConfig (tf, &ctrlInfo,  
 	                             tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);
                                         
    if (hd == NULL)  {
        System_printf ("%s: (%s:%d): Failure in Default Route Global Control command with error code = %d\n", tfName, __FILE__, __LINE__, paret);
   	    return (PA_TEST_FAILED);
    }								 
    
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
	/* All the packets should have been acked */
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, NULL, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
    		
	if (i == 100)  {
		System_printf ("%s: (%s:%d): Pa_control (Deafult Route Global Control) commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
    
    
	return (PA_TEST_PASSED);
}

static paTestStatus_t t2GlobalConfiguration (tFramework_t *tf, paTest_t *pat, paSysConfig_t *pCfg)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paCtrlInfo_t    ctrlInfo;
    int             fCmdReply;

    paCmdReply_t cmdReply = {  pa_DEST_HOST,            /* Dest */
                               0,                       /* Reply ID (returned in swinfo0) */
                               0,                       /* Queue */
                               0 };                     /* Flow ID */

    memset (&ctrlInfo, 0, sizeof (paCtrlInfo_t));

    /* set System Global default configuration */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = *pCfg;
    cmdReply.replyId  = T2_CMD_SWINFO0_GLOB_CFG_ID;
    cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
    cmdReply.flowId   = tf->tfFlowNum[0];

    hd = testCommonGlobalConfig (tf, &ctrlInfo,
                                 tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3,
                                 &cmdReply, &cmdDest, &cmdSize, &paret);

    if (hd == NULL)  {
        System_printf ("%s: (%s:%d): Failure in Global Configuration command with error code = %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }

    /* Send command */
    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    /* All the packets should have been acked */
    /* Wait for a PA reply */
    fCmdReply = FALSE;
    for (i = 0; i < 100; i++)  {
        utilCycleDelay (1000);
        t2L2CmdRep (tf, pat, NULL, &fCmdReply);
        if (fCmdReply)
            break;
    }

    if (i == 100)  {
        System_printf ("%s: (%s:%d): Pa_control (Global Configuration) commands was not acked\n", tfName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    return (PA_TEST_PASSED);
}


static paEQosModeConfig_t  defEQoSModeCfg[T2_NUM_EQoS_PORTS];

static paTestStatus_t t2ConfigEQoSModes(tFramework_t *tf, paTest_t *pat, int clear)
{
	int 			i;
	Cppi_HostDesc  *hd = NULL;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    int             fCmdReply;
    
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
    
    /* Issue EQoS mode configuration command */
    cmdReply.replyId  = T2_CMD_SWINFO0_EQoS_CFG_ID;
	cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tf->tfFlowNum[0];
    if (!clear)
    {
 	    hd = testCommonConfigEQoSMode (tf, T2_NUM_EQoS_PORTS, t2EQoSModeCfg,  
 	                                   tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                                   &cmdReply, &cmdDest, &cmdSize, &paret);
    }
    else
    {
        memset(defEQoSModeCfg, 0, sizeof(defEQoSModeCfg));
        for (i = 0; i < T2_NUM_EQoS_PORTS; i++)
        {
            defEQoSModeCfg[i].port = t2EQoSModeCfg[i].port;       
        }
        
 	    hd = testCommonConfigEQoSMode (tf, T2_NUM_EQoS_PORTS, defEQoSModeCfg,  
 	                                   tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                                   &cmdReply, &cmdDest, &cmdSize, &paret);
    
    }        
                                         
    if (hd == NULL)  {
   			
   	  System_printf ("%s: (%s:%d): Failure in ConfigEQoSMode command with error code = %d\n", tfName, __FILE__, __LINE__, paret);
   	  return (PA_TEST_FAILED);
   	
    }								 
    
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, NULL, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
    
	if (i == 100)  {
		System_printf ("%s: (%s:%d): ConfigDefaultRoutes commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}


static paTestStatus_t t2E802p1agConfiguration (tFramework_t *tf, paTest_t *pat, int cfgIndex)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paCtrlInfo_t    ctrlInfo;
    int             fCmdReply;
    
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
    
    
    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_802_1ag_CONFIG;
    ctrlInfo.params.pa802p1agDetCfg = t2E802p1agDetCfg[cfgIndex];
    cmdReply.replyId  = T2_CMD_SWINFO0_802_1AG_CFG_ID;
	cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tf->tfFlowNum[0];
    
 	hd = testCommonGlobalConfig (tf, &ctrlInfo,  
 	                             tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);
                                         
   if (hd == NULL)  {
   			
   	 System_printf ("%s: (%s:%d): Failure in 802.1ag DEtector Config command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);
   }								 
    
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
    /* Command goes to PDSP 0 */
 	paTestL2ExpectedStats.classify1.nPackets += 1;
    
	/* All the packets should have been acked */
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, NULL, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
    		
	if (i == 100)  {
		System_printf ("%s: (%s:%d): Pa_control (802.1ag Detector) commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
    
    
	return (PA_TEST_PASSED);
}

static Cppi_HostDesc *t2GetDataPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
    for (index = Q_MATCH; index <= Q_MATCH; index++)
    {
    
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t2GetDataPk: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }
        
            return (hd);
	    }	
    }
    
   return (NULL); 
	
}

	
/* Search the receive data packet queue for received data packets. Remain in 
 * this function until all buffers are restored to their respective queues */
int t2ReceiveDataPkts (tFramework_t *tf, paTest_t *pat, uint8_t *actualPktCount, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		 *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		  infoLen;
	int               i, j;
	unsigned int	  chan;
    uint8_t           psFlags;
    int               count = 0;
    int               msmcBounceCount=0;
#ifdef NSS_GEN2    
    Cppi_DescTag      tag;
#endif       
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }
	
	for (i = 0; i < 100; i++)  {
		
        /* Wait for packets to arrive */
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
		while ((hd = t2GetDataPkt (tf)) != NULL) {
			
			/* Verify PS flags (= pa_EMAC_PORT_0) */
            #ifndef NSS_GEN2
            psFlags = Cppi_getPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)hd);
            #else
            tag = Cppi_getTag(Cppi_DescType_HOST, (Cppi_Desc *)hd); 
            psFlags = tag.destTagLo;
            #endif
            
            if (psFlags != pa_EMAC_PORT_0)
            {
                System_printf ("%s (%s:%d): Packet received with unexpected PS flags = 0x%02x\n", tfName, __FILE__, __LINE__, psFlags);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
            }
            
            
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & 0xffff0000) != T2_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & 0xffff;
			if (chan < T2_NUM_LOCAL_HANDLES)
            {
			  actualPktCount[chan] += 1;
              count++;
            }
            else
            {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
            }
              
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t2PktTestInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t2PktTestInfo[j].idx == chan)  {
					tinfo = &t2PktTestInfo[j];
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
				
			  
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
            
            /* Verify the replaced MAC header */
            if(memcmp((void*)hd->buffPtr, t2MacRouteHdr, sizeof(t2MacRouteHdr)))
            {
				System_printf ("%s (%s:%d): MAC header does not match!\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
            }
			
			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);								
			
		}
		
        if(count >= expCount)
	        break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all descriptors with associated buffers\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}
	
    if(msmcBounceCount != count)
        System_printf("t2ReceiveDataPkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

	return (0);

}

static Cppi_HostDesc *t2FormQoSPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	
	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}
    
	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QfreeDesc;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
  	
  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t2QoSPktTestInfo[pktIdx].pkt)), t2QoSPktTestInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t2QoSPktTestInfo[pktIdx].pktLen);
    
  	return (hd);
}


static Cppi_HostDesc *t2GetQoSPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
    for (index = Q_QoS_BASE; index <= (T2_NUM_QoS_QUEUES +  Q_QoS_BASE); index++)
    {
    
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t2GetQoSPk: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }
        
            return (hd);
	    }	
    }
    
   return (NULL); 
	
}

/* Search the QoS queue for received packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t2ReceiveQoSPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               count = 0;
    int               ddrBounceCount=0;
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);
		while ((hd = t2GetQoSPkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T2_CMD_SWINFO0_TYPE_MASK) != T2_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T2_CMD_SWINFO0_ID_MASK;
            count++;
              
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t2QoSPktTestInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t2QoSPktTestInfo[j].idx == chan)  {
					tinfo = &t2QoSPktTestInfo[j];
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			  
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
            
            /* push packet to Tx commands quuee for further processing to simulate QoS output processing */
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            
            utilCycleDelay (1000);
            
	        /* This packet will be recycled to the default recycle queue */
	        while (Qmss_getQueueEntryCount(tf->QDefRet) > 0)  {
		
		        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QDefRet)) & ~15);
		        if (hd == NULL)  {
			        System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			        return (-1);
		        }
		
		        testCommonRecycleLBDesc (tf, hd);
	        }
			
		}
        
        if(count >= expCount)
            break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): QoS Queue Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}
	
    if(ddrBounceCount != count)
        System_printf("t2ReceiveQoSPkts: receives %d queue bounce packets and %d final packets\n", ddrBounceCount, count);

    return (0);

}


static Cppi_HostDesc *t2FormIfPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
    Cppi_DescTag   tag;
    
    memset(&tag, 0, sizeof(tag));
	
	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}
    
	/* Setup the return for the descriptor */
    /* Need to reset the source tag */
  	q.qMgr = 0;
  	q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
    
    /* Set Input interface number */
    tag.srcTagHi = pktIdx + 1;
    Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)hd, &tag);
    
#ifdef NSS_GEN2
    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0x8);
#endif    
  	
  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t2IfPktTestInfo[pktIdx].pkt)), t2IfPktTestInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t2IfPktTestInfo[pktIdx].pktLen);
    
  	return (hd);
}


static Cppi_HostDesc *t2GetIfPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
    for (index = Q_IF_BASE; index <= (T2_NUM_IF_QUEUES +  Q_IF_BASE); index++)
    {
    
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t2GetIfPkt: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }
        
            return (hd);
	    }	
    }
    
   return (NULL); 
	
}

/* Search the interfcae queues for received packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t2ReceiveIfPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t	     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t	      infoLen;
	int               i, j;
	unsigned int      chan;
    int               count = 0;
    int               msmcBounceCount=0;
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
		while ((hd = t2GetIfPkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T2_CMD_SWINFO0_TYPE_MASK) != T2_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T2_CMD_SWINFO0_ID_MASK;
            count++;
              
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t2IfPktTestInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t2IfPktTestInfo[j].idx == chan)  {
					tinfo = &t2IfPktTestInfo[j];
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			  
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
            
			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);	
            
            /* Clear and recycle the free descriptor */
  	        while (Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 		        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
                hd->tagInfo = 0;
                #ifdef NSS_GEN2
                    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
                #endif    
 		        Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        }
            							
		}
        
        if(count >= expCount)
            break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): If Queue Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}
	
    if(msmcBounceCount != count)
        System_printf("t2ReceiveIfPkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

	return (0);

}


static Cppi_HostDesc *t2Form802p1agPacket (tFramework_t *tf, paTest_t *pat, int group, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	
	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}
	
	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QfreeDesc;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
  	
  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t2E8021agPktTestInfo[group][pktIdx].pkt)), t2E8021agPktTestInfo[group][pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t2E8021agPktTestInfo[group][pktIdx].pktLen);
  	
  	return (hd);
}


/* Search the receive data packet queue for received data packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t2Receive802p1agPkts (tFramework_t *tf, paTest_t *pat, uint8_t *actualPktCount, int group, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
    uint8_t            *pkt;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               count = 0;
    int               msmcBounceCount=0;
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (10000 * expCount);
    }
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
		while ((hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_802_1AG])) & ~15)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T2_CMD_SWINFO0_TYPE_MASK) != T2_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T2_CMD_SWINFO0_ID_MASK;
            count++;
			if (chan <= T2_802_1AG_PKT_INDEX)
			  actualPktCount[chan] += 1;
              
            pkt = (uint8_t *)hd->buffPtr;  
			  
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t2E8021agPktTestInfo[group]) / sizeof(pktTestInfo_t); j++)  {
				if (t2E8021agPktTestInfo[group][j].idx == chan)  {
					tinfo = &t2E8021agPktTestInfo[group][j + pkt[6]];
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			  
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);								
			
		}
        
        if(count >= expCount)
            break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): 802.1ag Packet Processing Error - unable to recover all packets (count = %d, expCount = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}
	
    if(msmcBounceCount != count)
        System_printf("t2Receive802p1agPkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

    return (0);

}

static void t2Test802p1agPkts (tFramework_t *tf, paTest_t *pat, t2Handles_t* l2Handles, int numEntries, uint8_t *actualPktCount, int group)
{
	Cppi_HostDesc    *hd;
	pktTestInfo_t    *pPktInfo;
	int               i,j;
    int               count;


    /* Send 802.3 packets with detector disabled: all packets should be dropped due to LUT2 match failure*/
	/* Run packets through the system. the complete set of packets is run through three times. */
    pPktInfo = &t2E8021agPktTestInfo[group][0];
    
	for (j = 0; j < T2_NUM_PACKET_ITERATIONS; j++)   {
        count = 0;
		for (i = 0; i < numEntries; i++ )  {
		
			hd = t2Form802p1agPacket (tf, pat, 0, i);
			
			if (hd == NULL)  {
			    System_printf ("%s (%s:%d): T2 802.1ag packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
                paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
                break;
			}
			
			/* Increment any expected stats */
			testCommonIncStats (pPktInfo[i].statsMap, &paTestL2ExpectedStats);
            if (pPktInfo[i].idx == T2_802_1AG_FAIL_PKT_INDEX)
            {	
 		        paTestL2ExpectedStats.classify1.nSilentDiscard += 1;
                paTestL2ExpectedStats.classify1.nNoTableMatch  += 1;
            }    
            else
            {
                count++;
            }
            
            /* send packet */
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            
		}
			
		if (t2Receive802p1agPkts (tf, pat, actualPktCount, group, count))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t2Receive802p1agPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
            paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
			break;
        }    
	}
    
}

static Cppi_HostDesc *t2FormDRpacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
    Cppi_DescTag   tag;
    
    memset(&tag, 0, sizeof(tag));
	
	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}
    
	/* Setup the return for the descriptor */
    /* Need to reset the source tag */
  	q.qMgr = 0;
  	q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
    
    /* Set Input interface number */
    tag.srcTagHi = t2DRpktTestInfo[pktIdx].emacPort;
    Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)hd, &tag);

#ifdef NSS_GEN2
    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0x8);
#endif    
  	
  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t2DRpktTestInfo[pktIdx].pktInfo.pkt)), t2DRpktTestInfo[pktIdx].pktInfo.pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t2DRpktTestInfo[pktIdx].pktInfo.pktLen);
    
  	return (hd);
}


static Cppi_HostDesc *t2GetDRpkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
    for (index = Q_DROUTE; index <= Q_DROUTE; index++)
    {
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t2GetDRpkt: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }
        
            return (hd);
	    }	
    }
    
   return (NULL); 
	
}

/* Search the default route queues for received packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t2ReceiveDRpkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t	     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t	      infoLen;
	int               i, j;
	unsigned int      chan;
    int               count = 0;
    int               msmcBounceCount=0;
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
		while ((hd = t2GetDRpkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T2_CMD_SWINFO0_TYPE_MASK) != T2_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T2_CMD_SWINFO0_ID_MASK;
            count++;
              
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t2DRpktTestInfo) / sizeof(ifPktTestInfo_t); j++)  {
				if (t2DRpktTestInfo[j].pktInfo.idx == chan)  {
					tinfo = &t2DRpktTestInfo[j].pktInfo;
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			  
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
            
			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);	
            
            /* Clear and recycle the free descriptor */
  	        while (Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 		        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
                hd->tagInfo = 0;
                #ifdef NSS_GEN2
                    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
                #endif    
 		        Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        }
            							
		}
        
        if(count >= expCount)
            break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Default Route Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}
	
    if(msmcBounceCount != count)
        System_printf("t2ReceiveDRpkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

	return (0);

}

static Cppi_HostDesc *t2FormEQoSPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
    Cppi_DescTag   tag;
    
    memset(&tag, 0, sizeof(tag));
	
	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}
    
	/* Setup the return for the descriptor */
    /* Need to reset the source tag */
  	q.qMgr = 0;
  	q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
    
    /* Set Input interface number */
    tag.srcTagHi = t2eQoSPktTestInfo[pktIdx].emacPort;
    Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)hd, &tag);
    
#ifdef NSS_GEN2
    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0x8);
#endif    
  	
  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t2eQoSPktTestInfo[pktIdx].pktInfo.pkt)), t2eQoSPktTestInfo[pktIdx].pktInfo.pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t2eQoSPktTestInfo[pktIdx].pktInfo.pktLen);
    
  	return (hd);
}


static Cppi_HostDesc *t2GetEQoSPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
    for (index = Q_QoS_BASE; index <= (T2_NUM_EQoS_QUEUES +  Q_QoS_BASE); index++)
    {
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t2GetEQoSPkt: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }
        
            return (hd);
	    }	
    }
    
   return (NULL); 
	
}

/* Search the default route queues for received packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t2ReceiveEQoSPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t	     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
    uint8_t          *pkt;
	uint32_t	      infoLen;
	int               i, j;
	unsigned int      chan;
    int               count = 0;
    int               ddrBounceCount=0;
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);

		while ((hd = t2GetEQoSPkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T2_CMD_SWINFO0_TYPE_MASK) != T2_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T2_CMD_SWINFO0_ID_MASK;
            count++;
            pkt = (uint8_t *)hd->buffPtr;  
            
              
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t2eQoSPktTestInfo) / sizeof(ifPktTestInfo_t); j++)  {
				if (t2eQoSPktTestInfo[j].pktInfo.idx == chan)  {
					tinfo = &t2eQoSPktTestInfo[j + pkt[5]].pktInfo;
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			  
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
            
			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);	
            
            /* Clear and recycle the free descriptor */
  	        while (Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 		        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
                hd->tagInfo = 0;
                #ifdef NSS_GEN2
                    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
                #endif    
 		        Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        }
            							
		}
        
        if(count >= expCount)
            break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): EQoS Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}

    if(ddrBounceCount != count)
        System_printf("t2ReceiveEQoSPkts: receives %d queue bounce packets and %d final packets\n", ddrBounceCount, count);

	return (0);
}

#ifdef __LINUX_USER_SPACE
void* paTestL2Routing (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
void paTestL2Routing (UArg a0, UArg a1)
{
 	tFramework_t   *tf  = (tFramework_t *)a0;
 	paTest_t       *pat = (paTest_t *)a1;
#endif
 	Cppi_HostDesc  *hd[8];
 	paReturn_t      paret;
 	t2Handles_t     l2Handles[T2_NUM_LOCAL_HANDLES];
    #ifdef PASS_LUT_LIMIT_TEST 
 	paHandleL2L3_t  fHandle;
    #endif
 	int32_t				i, j, k, l;
 	int32_t				state;
 	int32_t  			cmdDest;
 	Bool			halt;
 	uint16_t			cmdSize;
 	paTestStatus_t  testStatus = PA_TEST_PASSED;
 	uint8_t			expectedPktCount[T2_NUM_LOCAL_HANDLES];
 	uint8_t			actualPktCount[T2_NUM_LOCAL_HANDLES];
    int             fCmdReply;
    int             num802p1agPkts;
    int             numPkts, expCount; 
 	
 	//volatile int mdebugWait = 1;
 	
    /* Normal operation routing */
 	paRouteInfo_t   matchRoute = {  pa_DEST_HOST,		/* Dest */
 								    0,					/* Flow ID */
 								    0,					/* queue */
 								    -1,					/* Multi route */
 								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used */         
                                    0,                  /* customIndex: not used */     
                                    pa_EMAC_PORT_0,     /* pkyType(SRIO)/EMAC port number (ETH/HOST)*/    
                                    &t2CmdSetCmd};      /* command set command */
                                    
                                    
 								    
 	paRouteInfo_t   nfailRoute = {  pa_DEST_HOST,		/* Dest */
 									0,					/* Flow ID */
 									0,					/* queue */
 									-1,					/* Multi route */
 									0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used */         
                                    0,                  /* customIndex: not used */     
                                    0,                  /* pkyType: for SRIO only */    
                                    NULL};              /* No commands */
                                    
    paRouteInfo2_t matchRoute2 =  { 
                                    pa_ROUTE_INFO_VALID_PRIORITY_TYPE | /* priority type and command */
                                    pa_ROUTE_INFO_VALID_PKTTYPE_EMAC  |
                                    pa_ROUTE_INFO_VALID_PCMD,            
                                    pa_DEST_HOST,		    /* Dest */
						       		0,					    /* Flow ID */
						       		0,					    /* queue */
						       	    0,					    /* Multi route */
						       		0,              	    /* sw Info 0 */
                                    0,                      /* sw Info 1 */       
                                    0,                      /* customType : not used  */         
                                    0,                      /* customIndex: not used  */     
                                    pa_EMAC_PORT_1,         /* pkyType: for SRIO only */    
                                    &t2CmdSetCmd2,          /* Command set: Next Route */
                                    pa_ROUTE_PRIORITY_VLAN, /* Priority Type          */
                                    NULL                    /* efOpInfo */
                                  };
                              
    paRouteInfo2_t nfailRoute2 =  { 
                                    0,                   /* validBitMap            */   
                                    pa_DEST_DISCARD,	/* Dest */
								    0,					/* Flow ID */
								    0,					/* queue */
								    -1,					/* Multi route */
								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used  */         
                                    0,                  /* customIndex: not used  */     
                                    0,                  /* pkyType: for SRIO only */    
                                    NULL,               /* No commands            */
                                    0,                  /* Priority Type          */
                                    NULL                /* efOpInfo */
						       	   };
    paRouteInfo2_t matchRoute3[4] = {
                                        { 
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE,  /* priority type and command */
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        0,                      /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_INTF,          /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        },
                                        
                                        { 
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE,  /* priority type and command */
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        0,                      /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_INTF_W_FLOW,   /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        },
                                        
                                        { 
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE,  /* priority type and command */
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        0,                      /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_INTF,          /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        },
                                        
                                        { 
                                        0,                      /* priority type and command */
                                        pa_DEST_CONTINUE_PARSE_LUT1,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        0,                      /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        0,                      /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        }
                                    };
                                    
                                    
    paRouteInfo2_t nfailRoute3 =  { 
                                    pa_ROUTE_INFO_VALID_PRIORITY_TYPE,   /* validBitMap            */  
                                     
                                    pa_DEST_HOST,    	/* Dest */
								    0,					/* Flow ID */
								    0,					/* queue */
								    -1,					/* Multi route */
								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used  */         
                                    0,                  /* customIndex: not used  */     
                                    0,                  /* pkyType: for SRIO only */    
                                    NULL,               /* No commands            */
                                    pa_ROUTE_INTF_W_FLOW,/* Priority Type          */
                                    NULL                /* efOpInfo */
						       	   };
                                   
                                   
    /* Match Route for EQoS test packet */                                   
    paRouteInfo2_t matchRoute4[3] = {
                                        { 
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE |  /* priority type and command */
                                        pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        pa_EMAC_PORT_1,         /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_EQoS_MODE,     /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        },
                                        
                                        { 
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE |  /* priority type and command */
                                        pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        pa_EMAC_PORT_2,         /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_EQoS_MODE,     /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        },

                                        { 
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE |  /* priority type and command */
                                        pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
						       		    0,              	    /* sw Info 0 */
                                        0,                      /* sw Info 1 */       
                                        0,                      /* customType : not used  */         
                                        0,                      /* customIndex: not used  */     
                                        pa_EMAC_PORT_3,         /* pkyType: for SRIO only */    
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_EQoS_MODE,     /* Priority Type          */
                                        NULL                    /* efOpInfo */
                                        },
                                        
                                    };
                                    
paRouteInfo2_t nfailRoute4 = { 
                                 0,                             /* priority type and command */
                                 pa_DEST_DISCARD,	            /* Dest */
						       	 0,					            /* Flow ID */
						       	 0,					            /* queue */
						       	 0,					            /* Multi route */
						       	 0,              	            /* sw Info 0 */
                                 0,                             /* sw Info 1 */       
                                 0,                             /* customType : not used  */         
                                 0,                             /* customIndex: not used  */     
                                 0,                             /* pkyType: for SRIO only */    
                                 NULL,                          /* Command set: Next Route */
                                 0,                             /* Priority Type          */
                                 NULL                           /* efOpInfo */
                             };
                                   
                                     
                             
   /*
    * Special test case to demonstrate the nextFail route usage with unsupported Ethernet type 
    * Apply to the first entry only 
    */                                 
 	paRouteInfo_t   matchRoute1 = { pa_DEST_CONTINUE_PARSE_LUT1, /* Dest */
 								    0,					/* Flow ID */
 								    0,					/* queue */
 								    -1,					/* Multi route */
 								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used */         
                                    0,                  /* customIndex: not used */     
                                    0,                  /* pkyType(SRIO)/EMAC port number (ETH/HOST)*/    
                                    NULL};              /* command set command */
                                    
                                    
 	paRouteInfo_t   nfailRoute1 = { pa_DEST_HOST,		/* Dest */
 								    0,					/* Flow ID */
 								    0,					/* queue */
 								    -1,					/* Multi route */
 								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used */         
                                    0,                  /* customIndex: not used */     
                                    pa_EMAC_PORT_0,     /* pkyType(SRIO)/EMAC port number (ETH/HOST)*/    
                                    &t2CmdSetCmd};      /* command set command */
                                    
 									
 	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
    i = setupPktTestInfo(t2PktTestInfo, (sizeof(t2PktTestInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }		
	
    i = setupPktTestInfo(t2QoSPktTestInfo, (sizeof(t2QoSPktTestInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }	
    
    i = setupPktTestInfo(t2IfPktTestInfo, (sizeof(t2IfPktTestInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }	
    
	for (j = 0; j < 3; j++)  {
		i = setupPktTestInfo(t2E8021agPktTestInfo[j], 2, tfName );
		if (i < 0) {
			System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
			pat->testStatus = PA_TEST_FAILED;
			Task_exit();
		}			
    }
    
    i = setupIfPktTestInfo(t2DRpktTestInfo, (sizeof(t2DRpktTestInfo) / sizeof(ifPktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupIfPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }	
    
    i = setupIfPktTestInfo(t2eQoSPktTestInfo, (sizeof(t2eQoSPktTestInfo) / sizeof(ifPktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupIfPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }	

    
    /* Runtime initial values */
    matchRoute.queue = (uint16_t) tf->QGen[Q_MATCH];
    matchRoute.flowId = tf->tfFlowNum[0];
    nfailRoute.queue = (uint16_t) tf->QGen[Q_NFAIL];
    nfailRoute.flowId = tf->tfFlowNum[0];
    nfailRoute1.queue = (uint16_t) tf->QGen[Q_MATCH];   
    nfailRoute1.flowId = tf->tfFlowNum[0];
    cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];
    
    matchRoute2.queue = (uint16_t) tf->QGen[Q_QoS_BASE];
    matchRoute2.flowId = tf->tfFlowNum[0];
    
    matchRoute3[0].queue = 
    matchRoute3[1].queue =
    matchRoute3[2].queue = (uint16_t) tf->QGen[Q_IF_BASE];
    matchRoute3[0].flowId = 
    matchRoute3[1].flowId =
    matchRoute3[2].flowId = tf->tfFlowNum[0];
    
    matchRoute4[0].queue = 
    matchRoute4[1].queue =
    matchRoute4[2].queue = (uint16_t) tf->QGen[Q_QoS_BASE];
    matchRoute4[0].flowId = 
    matchRoute4[1].flowId =
    matchRoute4[2].flowId = tf->tfFlowNum[0];
    
    
    nfailRoute3.queue = (uint16_t) tf->QGen[Q_IF_BASE];  
    nfailRoute3.flowId = tf->tfFlowNum[0];
    
    for (i = 0; i < T2_NUM_DR_PORTS; i++)
    {
        for (j = 0; j < pa_DROUTE_MAX; j++)
        {
            t2DefRouteCfg[i].dRouteInfo[j].flowId = tf->tfFlowNum[0];
        }
    }
    
    t2CmdSet20[0].params.patch  = t2PatchCmd;  
    t2CmdSetCmd.params.cmdSet.index = T2_CMDSET_INDEX;  
    t2CmdSet30[0].params.route  = t2NextRouteCmd;  
    t2CmdSetCmd2.params.cmdSet.index = T2_CMDSET2_INDEX;  
    
    /* Zero out the l2Handle array and packet counts */
    memset (l2Handles, 0, sizeof(l2Handles));
    memset (expectedPktCount, 0, sizeof(expectedPktCount));
    memset (actualPktCount, 0, sizeof(actualPktCount));
    
    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL2ExpectedStats, 0, sizeof(paTestL2ExpectedStats));
    
    /* Global Configuration */
    testStatus = t2GlobalConfiguration(tf, pat, &t2GlobalCfg);
	if (testStatus == PA_TEST_FAILED)
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */

    /* Issue the command set command */
    cmdReply.replyId  = T2_CMD_SWINFO0_CMDSET_CFG_ID;
 	hd[0] = testCommonConfigCmdSet (tf, T2_CMDSET_INDEX, T2_CMDSET_NUM_CMDS, t2CmdSet20,  
 	                                tf->QGen[Q_CMD_RECYCLE],tf->QLinkedBuf2, 
 	                                &cmdReply, &cmdDest, &cmdSize, &paret);
                                    
    /* Check paret before the descriptor. If paret indicates failure the descriptor will be NULL */                    
 	if (paret != pa_OK)  {
 		System_printf ("%s (%s:%d): testCommonConfigCmdSet failed, PA LLD error code = %d\n", tfName, __FILE__, __LINE__, paret);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
 	
 	if (hd[0] == NULL)  {
 		System_printf ("%s (%s:%d): testCommonConfigCmdSet failed due to unavailable free linked buffer descriptor\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
                                         
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
 	
 	if (i == 100)  {
 		System_printf ("%s (%s:%d): Reply to Pa_configCmdSet not found\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
 	}
    
    /* Issue the second command set command */
    cmdReply.replyId  = T2_CMD_SWINFO0_CMDSET_CFG_ID;
 	hd[0] = testCommonConfigCmdSet (tf, T2_CMDSET2_INDEX, T2_CMDSET2_NUM_CMDS, t2CmdSet30,  
 	                                tf->QGen[Q_CMD_RECYCLE],tf->QLinkedBuf2, 
 	                                &cmdReply, &cmdDest, &cmdSize, &paret);
                                    
    /* Check paret before the descriptor. If paret indicates failure the descriptor will be NULL */                    
 	if (paret != pa_OK)  {
 		System_printf ("%s (%s:%d): testCommonConfigCmdSet failed, PA LLD error code = %d\n", tfName, __FILE__, __LINE__, paret);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
 	
 	if (hd[0] == NULL)  {
 		System_printf ("%s (%s:%d): testCommonConfigCmdSet failed due to unavailable free linked buffer descriptor\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
                                         
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
 	/* Wait for a PA reply */
    fCmdReply = FALSE;
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, &fCmdReply);
 		if (fCmdReply)
 			break;
 	}
 	
 	if (i == 100)  {
 		System_printf ("%s (%s:%d): Reply to Pa_configCmdSet not found\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
 	}
    
    /* Exception Route Configurations */
	testStatus = t2ExceptionRoutes (tf, pat);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */

  
    /* 802.1ag Detector Configuration  */
	testStatus = t2E802p1agConfiguration (tf, pat, T2_802_1AG_CFG_INDEX_DISABLE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */

 	/* Initialize the first entry in the table */
 	cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + 0;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 	cmdReply.queue = tf->QGen[Q_CMD_REPLY];
 	matchRoute1.swInfo0 = nfailRoute1.swInfo0 = t2EthAndSwinfo[0].swinfo0;
 	hd[0] = testCommonAddMac (tf, t2EthIndex[0], (paEthInfo_t *)&t2EthAndSwinfo[0].eth, &matchRoute1, &nfailRoute1,
 	                        &l2Handles[0].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	                        
 	paL2HandleError (tf, pat, l2Handles, paret, hd[0]);  /* Will not return on error */
 	
 	/* Send the command to PA. This will result in 1 packet in classify1 */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	l2Handles[0].state = T2_L2_HANDLE_PENDING_ACK;
 	paTestL2ExpectedStats.classify1.nPackets += 1;
 	 	
 	/* Wait for a PA reply */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, NULL);
 		if (l2Handles[0].state == T2_L2_HANDLE_ACTIVE)
 			break;
 	}
 	
 	if (i == 100)  {
 		System_printf ("%s (%s:%d): Reply to paAddMac not found\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
 	}
 	
 	
 	/* Add the next 6 entries in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. These are entries 1-6 in the local handle table */
 	for (i = 0; i < 6; i++)  {
 		cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + i + 1;
 		matchRoute.swInfo0 = nfailRoute.swInfo0 = t2EthAndSwinfo[i+1].swinfo0;
 		hd[i] = testCommonAddMac (tf, t2EthIndex[i+1], (paEthInfo_t *)&t2EthAndSwinfo[i+1].eth, &matchRoute, &nfailRoute, 
 	                        &l2Handles[i+1].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 0; i < 6; i++)  {
 		/* if (mdebugWait) mdebugHaltPdsp(0); */ 
 		
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		
 		/* while (mdebugWait); */
 		
 		
 		l2Handles[i+1].state = T2_L2_HANDLE_PENDING_ACK;
 		paTestL2ExpectedStats.classify1.nPackets += 1;
 		
 	}
 	
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, NULL);
 		
 		state = 1;
 		for (j = 1; j < 7; j++)  {
 			if (l2Handles[j].state == T2_L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	if ((i == 100) && (state == 0))  {
 		System_printf ("%s: (%s:%d): Burst of 5 addMac commands did not result in 5 acks from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);
 	}


#if 0
 	
 	/* The next entry should result in the PA generating an error */
 	cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + 7;  /* T2_CMD_SWINFO0_ADD_ID identfies command, 16 LS bits identify the local handle number */
 	matchRoute.swInfo0 = nfailRoute.swInfo0 = t2EthAndSwinfoFail.swinfo0;
 	hd[0] = testCommonAddMac (tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t2EthAndSwinfoFail.eth, &matchRoute, &nfailRoute,
 	                        &l2Handles[7].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	                        
 	                         	/* If the descriptor came back non-null, restore it and return it */
 	if (hd[0] != NULL)  {
 		hd[0]->buffLen = hd[0]->origBufferLen;
 		Qmss_queuePush (tf->QLinkedBuf2, (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}
 	                  
 	/* The test won't exit on this failure since no commands have been sent to PA */                  
 	if (paret != pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT)  {
 		System_printf ("%s (%s:%d): function paAddMac did not detect an invalid entry order\n", tfName, __FILE__, __LINE__);
 		testStatus = PA_TEST_FAILED;
 	}
    
#endif    
 		
    /* Rapid fire the next 5 entries from the const table to the PA */
    /* Add the next 5 entries in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. These are entries 1-6 in the local handle table */
 	for (i = 0; i < 5; i++)  {
 		cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + i + 7;
 		matchRoute.swInfo0 = nfailRoute.swInfo0 = t2EthAndSwinfo[i+7].swinfo0;
 		hd[i] = testCommonAddMac (tf, t2EthIndex[i+7], (paEthInfo_t *)&t2EthAndSwinfo[i+7].eth, &matchRoute, &nfailRoute, 
 	                        &l2Handles[i+7].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 0; i < 5; i++)  {
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		l2Handles[i+7].state = T2_L2_HANDLE_PENDING_ACK;
 		paTestL2ExpectedStats.classify1.nPackets += 1;
 	}
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, NULL);
 		
 		state = 1;
 		for (j = 7; j < 12; j++)  {
 			if (l2Handles[j].state == T2_L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	 if (i == 100)  {
 		System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
 	}
    
#ifndef __LINUX_USER_SPACE
    
    /* Rapid fire the next 4 entries from the const table to the PA */
    /* Add the next 4 entries with Pa_addMac2 in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. These are entries 1-6 in the local handle table */
 	for (i = 0; i < 4; i++)  {
 		cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + i + 12;
 		matchRoute2.swInfo0 = nfailRoute.swInfo0 = t2EthAndSwinfo[i+12].swinfo0;
 		hd[i] = testCommonAddMac2 (tf, t2EthIndex[i+12], (paEthInfo_t *)&t2EthAndSwinfo[i+12].eth, &matchRoute2, &nfailRoute2, 
 	                               NULL, &l2Handles[i+12].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                               &cmdReply, &cmdDest, &cmdSize, 0, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 0; i < 4; i++)  {
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		l2Handles[i+12].state = T2_L2_HANDLE_PENDING_ACK;
 		paTestL2ExpectedStats.classify1.nPackets += 1;
 	}
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, NULL);
 		
 		state = 1;
 		for (j = 12; j < 16; j++)  {
 			if (l2Handles[i].state == T2_L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	 if (i == 100)  {
 		System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
 	}
    
 	/* 15 Entries into the table have been made. Make an additional 48 entries in 12 batches of 4.
 	 * These entries do not test any particular routing, simply table addition and overflow */
 	for (i = k = 0; i < 12; i++, k += 4)  {
 		
 		for (j = 0; j < 4; j++)  {
 			cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + 16 + k + j;
 			t2VarEthAndRoute.eth.dst[5] = k + j;
 			matchRoute.swInfo0 = nfailRoute.swInfo0 = 0x5555000b + k + j + 16;
 			hd[j] = testCommonAddMac (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t2VarEthAndRoute.eth, &matchRoute, &nfailRoute, 
 	                       		 	  &l2Handles[k+j+16].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                        		  &cmdReply, &cmdDest, &cmdSize, &paret);
			
 	    	paL2HandleError (tf, pat, l2Handles, paret, hd[j]);  /* Will not return on error */
 		}
 		
 		/* Send all the commands at once to test the ability to handle back to back commands */
 		for (j = 0; j < 4; j++)  {
 			Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[j], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 			l2Handles[k+j+16].state = T2_L2_HANDLE_PENDING_ACK;
 			paTestL2ExpectedStats.classify1.nPackets += 1;
 		}
 
 		/* Wait for the PA to generate all the responses */
 		for (l = 0; l < 100; l++)  {
 			utilCycleDelay (1000);
 			t2L2CmdRep (tf, pat, l2Handles, NULL);
 		
 			state = 1;
 			for (j = 0; j < 4; j++)  {
 				if (l2Handles[16 + k + j].state == T2_L2_HANDLE_PENDING_ACK)
 					state = 0;
 			}
 		
 			if (state == 1)
 				break;
 		}
 		
 		if (l == 100)  {
 			System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 			paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
 		}
 	}
	
    #ifdef PASS_LUT_LIMIT_TEST 
 	/* There is no place left in the table. Try to make one more entries and it should be rejected by the PA lld */
 	t2VarEthAndRoute.eth.dst[5] = k;
 	matchRoute.swInfo0 = nfailRoute.swInfo0 = T2_CMD_SWINFO0_ADD_ID + k;
 	hd[0] = testCommonAddMac (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t2VarEthAndRoute.eth, &matchRoute, &nfailRoute, 
 	                    	  &fHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                          &cmdReply, &cmdDest, &cmdSize, &paret);
 		                          
 	/* If the descriptor came back non-null, restore it and return it */
 	if (hd[0] != NULL)  {
 		hd[0]->buffLen = hd[0]->origBufferLen;
 		Qmss_queuePush (tf->QLinkedBuf2, (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}
 	
 	/* The test will continue on failure since the PA sub-system has not been changed */
 	if (paret != pa_HANDLE_TABLE_FULL)  {
 		System_printf ("%s (%s:%d): function paAddMac did not detect a full handle table\n", tfName, __FILE__, __LINE__);
 		testStatus = PA_TEST_FAILED;
 	}
    #endif  

#endif
    
 	/* Check and clear the stats */
 	testStatus = t2CheckStats (tf, pat, TRUE, l2Handles);
	memset (&paTestL2ExpectedStats, 0, sizeof(paTestL2ExpectedStats));
	
	if (testStatus != PA_TEST_PASSED)
		paTestL2RecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
		
	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0, halt = FALSE; (j < T2_NUM_PACKET_ITERATIONS) && (halt == FALSE); j++)   {
		
		for (i = 0, expCount =0; (i < sizeof(t2PktTestInfo) / sizeof(pktTestInfo_t)) && (halt == FALSE);  )  {
			
			/* Form up to 8 data packets to send */
			for (k = 0; ((k < 8) && (i < sizeof(t2PktTestInfo) / sizeof(pktTestInfo_t))); k++)  {
				hd[k] = formDataPacket (tf, pat, i, expectedPktCount);
				
				if (hd[k] == NULL)  {
					halt = TRUE;
					break;
				}
				
				/* Inc the count if the packet is passed back to the host */
  				if (t2PktTestInfo[i].idx >= 0) {
	  				expectedPktCount[t2PktTestInfo[i].idx] += 1;
					expCount++;
  			    }
		
				/* Increment any expected stats */
				testCommonIncStats (t2PktTestInfo[i].statsMap, &paTestL2ExpectedStats);	
				
				/* Proceed to the next packet */
				i += 1;  				

			}
			
            //mdebugWait = 1;
 		    //if (mdebugWait) mdebugHaltPdsp(0);  
				
			for (l = 0; l < k; l++)
				Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[l], hd[l]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
                
	        //while (mdebugWait);
				
		}
		
		if (t2ReceiveDataPkts (tf, pat, actualPktCount, expCount))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t2ReceiveDataPkts timeout %d\n", tfName,
						   __FILE__, __LINE__);
            System_flush();               
			break;
        }    
	}
	
	/* Verify that the expected and actual received packet counts match */
	for (i = 0; i < T2_NUM_LOCAL_HANDLES; i++)  {
		if (expectedPktCount[i] != actualPktCount[i])  {
			System_printf ("%s (%s:%d): Packet count mismatch for entry %d - expected %d, found %d\n", tfName,
						   __FILE__, __LINE__, i, expectedPktCount[i], actualPktCount[i]);
            System_flush();               
			testStatus = PA_TEST_FAILED;
		}
	}
    
	/* Due to limited MAC entries available for the user space, free up the previous entries, so that
	 * the next test gets them 
	 */    
	if ((paTestL2Delete(tf, pat, l2Handles, T2_NUM_LOCAL_HANDLES, 0)) == PA_TEST_FAILED) {
	 System_printf ("%s (%s:%d): Failed to delete used L2 handles\n", tfName, __FILE__, __LINE__);
	 paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
	}
     
    /* QoS packet testing */
	/* Rapid fire the next 4 entries from the const table to the PA */
    /* Add the next 4 entries with Pa_addMac2 in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. These are entries 1-6 in the local handle table */
 	for (i = 0; i < 4; i++)  {
 		cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + i;
 		matchRoute2.swInfo0 = nfailRoute.swInfo0 = t2EthAndSwinfo[i+12].swinfo0;
 		hd[i] = testCommonAddMac2 (tf, t2EthIndex[i+12], (paEthInfo_t *)&t2EthAndSwinfo[i+12].eth, &matchRoute2, &nfailRoute2, 
 	                               NULL, &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                               &cmdReply, &cmdDest, &cmdSize, 0, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 0; i < 4; i++)  {
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		l2Handles[i].state = T2_L2_HANDLE_PENDING_ACK;
 		paTestL2ExpectedStats.classify1.nPackets += 1;
 	}
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t2L2CmdRep (tf, pat, l2Handles, NULL);
 		
 		state = 1;
 		for (j = 0; j < 4; j++)  {
   	    if (l2Handles[j].state == T2_L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	if (i == 100)  {
 		System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
 	}
    
#ifndef NSS_GEN2
 	/* Note: To run this test for NSS_GEN2, need to enable the cpsw_switch */
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	numPkts = sizeof (t2QoSPktTestInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T2_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < numPkts; i++ )  {
		
			hd[0] = t2FormQoSPacket (tf, pat, i);
			
			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T2 QoS packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
 		        paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
                break;
			}
			
			/* Increment any expected stats */
			testCommonIncStats (t2QoSPktTestInfo[i].statsMap, &paTestL2ExpectedStats);
            
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            
		}
			
		if (t2ReceiveQoSPkts (tf, pat, numPkts))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t2ReceiveQoSPktstimeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
 		    paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
			break;
        }    
	}
#endif

    /* Intf packet testing */
	
    /* Rapid fire the next 4 entries from the const table to the PA */
    /* Add the next 4 entries with Pa_addMac2 in a burst. Take the memory from linked buffer descriptor area two
    * since the responses will come from area one. These are entries 1-6 in the local handle table */
    for (i = 0; i < 4; i++)	{
       cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + i + 4;
       matchRoute3[i].swInfo0 = nfailRoute3.swInfo0 = t2EthAndSwinfo[i+16].swinfo0;
       hd[i] = testCommonAddMac2 (tf, t2EthIndex[i+16], (paEthInfo_t *)&t2EthAndSwinfo[i+16].eth, &matchRoute3[i], &nfailRoute3, 
   							   NULL, &l2Handles[i+4].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
   							   &cmdReply, &cmdDest, &cmdSize, 0, &paret);
       paL2HandleError (tf, pat, l2Handles, paret, hd[i]);	/* Will not return on error */
    }
    
    /* Send all the commands at once to test the ability to handle back to back commands */
    for (i = 0; i < 4; i++)	{
       Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
       l2Handles[i+4].state = T2_L2_HANDLE_PENDING_ACK;
       paTestL2ExpectedStats.classify1.nPackets += 1;
    }
   
    /* Wait for the PA to generate all the responses */
    for (i = 0; i < 100; i++)  {
       utilCycleDelay (1000);
       t2L2CmdRep (tf, pat, l2Handles, NULL);
      
       state = 1;
       for (j = 4; j < 8; j++)  {
   	    if (l2Handles[j].state == T2_L2_HANDLE_PENDING_ACK)
   		    state = 0;
       }
      
       if (state == 1)
   	    break;
    }
   
    if (i == 100)  {
       System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
       paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
    }
	
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	numPkts = sizeof (t2IfPktTestInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T2_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < numPkts; i++ )  {
		
			hd[0] = t2FormIfPacket (tf, pat, i);
			
			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T2 QoS packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
 		        paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
                break;
			}
			
			/* Increment any expected stats */
			testCommonIncStats (t2IfPktTestInfo[i].statsMap, &paTestL2ExpectedStats);
            
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            
		}
			
		if (t2ReceiveIfPkts (tf, pat, numPkts))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t2ReceiveIfPktstimeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
 		    paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
			break;
        }    
	}
    
    /* 802.1ag Packets test */
    
    num802p1agPkts =  sizeof(t2E8021agPktTestInfo)/(sizeof(pktTestInfo_t)*3);
    
    /* Send 802.3 packets with detector disabled: all packets should be dropped due to LUT1 match failure*/
	/* Run packets through the system. the complete set of packets is run through three times. */
    t2Test802p1agPkts (tf, pat, l2Handles, num802p1agPkts, actualPktCount, T2_802_1AG_CFG_INDEX_DISABLE);
    
    /* Send 802.3 packets with detector set to standard */
	/* Run packets through the system. the complete set of packets is run through three times. */
	testStatus = t2E802p1agConfiguration (tf, pat, T2_802_1AG_CFG_INDEX_STANDARD);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
    
    t2Test802p1agPkts (tf, pat, l2Handles, num802p1agPkts, actualPktCount, T2_802_1AG_CFG_INDEX_STANDARD);
    
    /* Send 802.3 packets with detector set to draft */
	/* Run packets through the system. the complete set of packets is run through three times. */
	testStatus = t2E802p1agConfiguration (tf, pat, T2_802_1AG_CFG_INDEX_DRAFT);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
    
    t2Test802p1agPkts (tf, pat, l2Handles, num802p1agPkts, actualPktCount, T2_802_1AG_CFG_INDEX_DRAFT);
    
    /*  Restore the standard setting */
	testStatus = t2E802p1agConfiguration (tf, pat, T2_802_1AG_CFG_INDEX_DISABLE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
        
    /* Ingress Default Route test */    
        
    /* Set Default Route Configurations */
	testStatus = t2ConfigDefaultRoutes (tf, pat, FALSE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */

    /* Enable the default route feature */
    testStatus = t2DefaultRouteGlobalControl(tf, pat, TRUE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
        
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	numPkts = sizeof (t2DRpktTestInfo) / sizeof (ifPktTestInfo_t);
	for (j = 0; j < T2_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < numPkts; i++ )  {
		
			hd[0] = t2FormDRpacket (tf, pat, i);
			
			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T2 Default Route test packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
 		        paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
                break;
			}
			
			/* Increment any expected stats */
			testCommonIncStats (t2DRpktTestInfo[i].pktInfo.statsMap, &paTestL2ExpectedStats);
            
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            
		}
			
		if (t2ReceiveDRpkts (tf, pat, numPkts))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t2ReceiveDRpkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
 		    paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
			break;
        }    
	}

    /* Disable the default route feature */
    testStatus = t2DefaultRouteGlobalControl(tf, pat, FALSE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
        
    /* Clear Default Route Configurations */
	testStatus = t2ConfigDefaultRoutes (tf, pat, TRUE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
        
    /* EQoS packet testing */
	
    /* Set EQoS Mode Configurations */
	testStatus = t2ConfigEQoSModes (tf, pat, FALSE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
        
    /* Rapid fire the next 3 entries from the const table to the PA */
    /* Add the next 3 entries with Pa_addMac2 in a burst. Take the memory from linked buffer descriptor area two
    * since the responses will come from area one. */
    for (i = 0; i < 3; i++)	{
       cmdReply.replyId = T2_CMD_SWINFO0_ADD_ID + i + 8;
       matchRoute4[i].swInfo0 = t2EthAndSwinfo[i+20].swinfo0;
       hd[i] = testCommonAddMac2 (tf, t2EthIndex[i+20], (paEthInfo_t *)&t2EthAndSwinfo[i+20].eth, &matchRoute4[i], &nfailRoute4, 
   							     NULL, &l2Handles[i+8].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
   							     &cmdReply, &cmdDest, &cmdSize, 0, &paret);
       paL2HandleError (tf, pat, l2Handles, paret, hd[i]);	/* Will not return on error */
    }
       
    /* Send all the commands at once to test the ability to handle back to back commands */
    for (i = 0; i < 3; i++)	{
       Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
       l2Handles[i+8].state = T2_L2_HANDLE_PENDING_ACK;
       paTestL2ExpectedStats.classify1.nPackets += 1;
    }
   
    /* Wait for the PA to generate all the responses */
    for (i = 0; i < 100; i++)  {
       utilCycleDelay (1000);
       t2L2CmdRep (tf, pat, l2Handles, NULL);
      
       state = 1;
       for (j = 8; j < 11; j++)  {
   	    if (l2Handles[j].state == T2_L2_HANDLE_PENDING_ACK)
   		    state = 0;
       }
      
       if (state == 1)
   	    break;
    }
   
    if (i == 100)  {
       System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
       paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
    }

	/* Run packets through the system. the complete set of packets is run through multiple times. */
	numPkts = sizeof (t2eQoSPktTestInfo) / sizeof (ifPktTestInfo_t);
	for (j = 0; j < T2_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < numPkts; i++ )  {
		
			hd[0] = t2FormEQoSPacket (tf, pat, i);
			
			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T2 EQoS test packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
 		        paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
                break;
			}
			
			/* Increment any expected stats */
			testCommonIncStats (t2eQoSPktTestInfo[i].pktInfo.statsMap, &paTestL2ExpectedStats);
            
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            
		}
			
		if (t2ReceiveEQoSPkts (tf, pat, numPkts))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t2ReceiveEQoSPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
 		    paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
			break;
        }    
	}
        
    /* Clear EQoS Mode Configurations */
	testStatus = t2ConfigEQoSModes (tf, pat, TRUE);
	if (testStatus == PA_TEST_FAILED)
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */

	/* Clean up and return */
 	paTestL2RecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
 	
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
}
 		
 
 		 
 
 	
 	


