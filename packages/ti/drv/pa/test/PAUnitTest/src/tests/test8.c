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
#include "test8pkts.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif

/* Broadcast, Multicast and Multi-route test
 * This test tests the LLD exception route and multi-route configuration functions, as well as the
 * PDSP firmware for routing broadcast and multicast packets. 
 * This test invokes the following LLD APIs
 *  - Pa_configMultiRoute
 *  - Pa_configExceptionRoute
 *  - Pa_addMac
 *  - Pa_delHandle
 *  - Pa_forwardResult
 *  - Pa_requestStats
 *  - Pa_formatStatsReply
 *
 * This test has the following sub-tests
 *   - Test the ability to configure multiple multi-route sets
 *   - Test the ability to configure exception routes for MAC/IP broadcast and multicast packets
 *   - Test the LLD for the ability to detect configuration error related to the multi-route set
 *   - Test the firmware for the ability to detect and forward MAC broadcast and multicast packets
 *   - Test the firmware for the ability to detect and forward IP broadcast and multicast packets
 *   - Test the firmware for the ability to forward packets to multiple destinations as specified
 *   - Test the firmware for the ability to deliver the packet descriptor without the packet to the specified destinations
 */

 static char *tfName = "paTestMultiRouting";
 
 /* General purpose queue usage */
 #define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
 #define Q_CMD_REPLY  		  1		/* Replies from PA routed here */
 #define Q_MATCH		  	  2		/* Packets from PA which match a lookup criteria */
 #define Q_NFAIL		      3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
 #define Q_PARSE_ERR		  4		/* Packets which resulted in a parse error */
 /* Multi route group 1 */
 #define Q_MULTI_0			  5		/* Multi route queue 0 */
 #define Q_MULTI_1			  6	    /* Multi route queue 1 */
 #define Q_MULTI_2			  7		/* Multi route queue 2 */
 #define Q_MULTI_3			  8		/* Multi route queue 3 */
 #define Q_MULTI_4			  9		/* Multi route queue 4 */
 #define Q_MULTI_5			 10		/* Multi route queue 5 */
 #define Q_MULTI_6           11		/* Multi route queue 6 */
 #define Q_MULTI_7			 12		/* Multi route queue 7 */
 /* Multi route group 2 */
 #define Q_MULTI_10			 13		/* Multi route queue 0 */
 #define Q_MULTI_11			 14	    /* Multi route queue 1 */
 #define Q_MULTI_12			 15		/* Multi route queue 2 */
 #define Q_MULTI_13			 16		/* Multi route queue 3 */
 #define Q_MULTI_14			 17		/* Multi route queue 4 */
 #define Q_MULTI_15			 18		/* Multi route queue 5 */
 #define Q_MULTI_16          19		/* Multi route queue 6 */
 #define Q_MULTI_17          20		/* Multi route queue 7 */

#define Q_BOUNCE_DDR         21     /* Queue Bounce DDR queue */
#define Q_BOUNCE_MSMC        22     /* Queue Bounce MSMC queue */
 
#define  Q_MULTI_START  Q_MULTI_0
#define  Q_MULTI_END    Q_MULTI_17
 
/* The number of PA handles maintained by this test */
#define T8_NUM_LOCAL_HANDLES	sizeof(t8EthAndSwinfo)/sizeof(t8EthAndRoute_t)

/* The total number of buffers with linked descriptors */
#define TOTAL_BUFS   (TF_LINKED_BUF_Q1_NBUFS + TF_LINKED_BUF_Q2_NBUFS + TF_LINKED_BUF_Q3_NBUFS)
 
 /* Commands to the PA are verified through the value in swinfo0.
  * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T8_CMD_SWINFO0_ADD_ID  		    0x11110000  /* Identifies add mac command */
#define T8_CMD_SWINFO0_DEL_ID  		    0x22220000  /* Identifies del mac command */
#define T8_CMD_SWINFO0_STATS_REQ_ID	    0x33330000	/* Identifies the req stats command */
#define T8_CMD_SWINFO0_EROUTE_CFG_ID    0x44440000  /* Identifies Eroute configuration command */
#define T8_CMD_SWINFO0_MROUTE_CFG_ID    0x55550000  /* Identifies Multi-route configuration command */
#define T8_CMD_SWINFO0_GLOB_CFG_ID      0x55560000  /* Identifies Global configuration command */
#define T8_CMD_SWINFO0_PKT_ID		    0x66660000  /* Identifies the packet as a data packet */

#define T8_MAX_CHAN                     T8_IP_MULTICAST_PKT_INDEX

#define T8_MAX_CMDS_PER_BURST            8

 
static paSysStats_t paTestExpectedStats;  /* Expected stats results */
 
 /* 32 L2 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	L2_HANDLE_UNCONFIGURED = 0,
	L2_HANDLE_PENDING_ACK,
	L2_HANDLE_ACTIVE,
	L2_HANDLE_DISABLED
};

 typedef struct t8Handles_s  {
 
  	paHandleL2L3_t  paHandle;     /* The handle returned by the PA LLD */

 	unsigned int			state;		  /* L2_HANDLE_UNCONFIGURED = handle not configured
 								   * L2_HANDLE_PENDING_ACK = handle configured and sent to pa
 								   * L2_HANDLE_ACTIVE = handle creation acknowledged by pa */
 	
 } t8Handles_t;
 
 
 /* Static test configuration - routing of matching packets to the same queue and 
  * distinguished by swinfo0 */
 typedef struct t8EthAndRoute_s  {
 	
 	paEthInfo_t  eth;
    paRouteInfo_t match;
    paRouteInfo_t nfail;
 } t8EthAndRoute_t;
 
 
 typedef struct pauMultiRoute_s {
    uint16_t                index;      /* Multi-route set index */
    uint16_t                nRoute;     /* number of routes */
    paMultiRouteEntry_t     routeEntry[pa_MAX_MULTI_ROUTE_ENTRIES];
 
 }  pauMultiRoute_t; 
 
/* use the first 10 packet ID */ 
static t8EthAndRoute_t  t8EthAndSwinfo[] =  {
	
	{ {  {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  	/* Entry 0: Route on dest mac only */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 0 */
		    0, 0, 0, 0}, 
 	  {   pa_DEST_HOST,		                        /* Dest */
 		  0,					                    /* Flow ID */
 		  TF_FIRST_GEN_QUEUE + Q_MATCH,             /* queue */
 		  -1,					                    /* Multi route */
 		  T8_CMD_SWINFO0_PKT_ID,                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL},                                    /* No commands */
                                                    
 	  {   pa_DEST_DISCARD,		                    /* Dest */
 		  0,					                    /* Flow ID */
 		  0,					                    /* queue */
 		  -1,					                    /* Multi route */
 		  0,					                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL}                                     /* No commands */
    },                                
		
	{ {  {  0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },  	/* Entry 1: Route on dest/src mac */
		 {  0x10, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 1 */
		    0, 0, 0, 0},  
 	  {   pa_DEST_HOST,		                        /* Dest */
 		  0,					                    /* Flow ID */
 		  TF_FIRST_GEN_QUEUE + Q_MATCH,             /* queue */
 		  -1,					                    /* Multi route */
 		  T8_CMD_SWINFO0_PKT_ID+1,                  /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL},                                    /* No commands */
                                                    
 	  {   pa_DEST_DISCARD,		                    /* Dest */
 		  0,					                    /* Flow ID */
 		  0,					                    /* queue */
 		  -1,					                    /* Multi route */
 		  0,					                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL}                                     /* No commands */
    },                                
            
           
	{ {  {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  	/* Entry 2: Multi-cast dest mac */
		 {  0x11, 0x22, 0x33, 0x44, 0x55, 0x66 },	/* PA entry 2 */
		    0, 0, 0, 0},  
 	  {   pa_DEST_HOST,		                        /* Dest */
 		  0,					                    /* Flow ID */
 		  TF_FIRST_GEN_QUEUE + Q_MATCH,             /* queue */
 		  5,					                    /* Multi route */
#ifndef SIMULATOR_SUPPORT          
 		  0xbabeface,                               /* sw Info 0: It will be replaced by multi-route operation */
#else
 		  T8_CMD_SWINFO0_PKT_ID + 2,                /* sw Info 0: It will be replaced by multi-route operation */
                                                    /* Note: re-enable after the simulator bug is fixed */
#endif          
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL},                                    /* No commands */
                                                    
 	  {   pa_DEST_DISCARD,		                        /* Dest */
 		  0,					                    /* Flow ID */
 		  0,					                    /* queue */
 		  -1,					                    /* Multi route */
 		  0,					                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL}                                     /* No commands */
    },
    
    
	{ {  {  0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f },  	/* Entry 3: Broadcast with source MAC  */
		 {  0xff, 0xff, 0xff, 0xff, 0xff, 0xff },	/* PA entry 3 */
		    0, 0, 0, 0},  
 	  {   pa_DEST_CONTINUE_PARSE_LUT1,              /* Dest */
 		  0,					                    /* Flow ID */
 		  0,                                        /* queue */
 		  -1,					                    /* Multi route */
 		  0,                                        /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL},                                    /* No commands */
                                                    
 	  {   pa_DEST_DISCARD,		                        /* Dest */
 		  0,					                    /* Flow ID */
 		  0,					                    /* queue */
 		  -1,					                    /* Multi route */
 		  0,					                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL}                                     /* No commands */
    },
    
    
	{ {  {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  	/* Entry 4: Multi-cast dest mac */
		 {  0x01, 0x00, 0x1e, 0x14, 0x15, 0x16 },	/* PA entry 4 */
		    0, 0, 0, 0},  
 	  {   pa_DEST_CONTINUE_PARSE_LUT1,              /* Dest */
 		  0,					                    /* Flow ID */
 		  0,                                        /* queue */
 		  -1,					                    /* Multi route */
 		  0,                                        /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL},                                    /* No commands */
                                                    
 	  {   pa_DEST_DISCARD,		                        /* Dest */
 		  0,					                    /* Flow ID */
 		  0,					                    /* queue */
 		  -1,					                    /* Multi route */
 		  0,					                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL}                                     /* No commands */
    },
    
    
	{ {  {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  	/* Entry 5: Route on dest mac only */
		 {  0x10, 0x11, 0x12, 0x13, 0x14, 0x15 },	/* PA entry 5 */
		    0, 0, 0, 0}, 
 	  {   pa_DEST_CONTINUE_PARSE_LUT1,              /* Dest */
 		  0,					                    /* Flow ID */
 		  0,                                        /* queue */
 		  -1,					                    /* Multi route */
 		  0,                                        /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          0,                                        /* pkyType: for SRIO only */    
          NULL},                                    /* No commands */
                                                    
 	  {   
          #if 0 /* Disable for QT test */
          pa_DEST_EMAC, 		                    /* Dest */
          #else
          pa_DEST_DISCARD, 	                        /* Dest */
          #endif
 		  0,					                    /* Flow ID */
 		  0,					                    /* queue */
 		  -1,					                    /* Multi route */
 		  0,					                    /* sw Info 0 */
          0,                                        /* sw Info 1 */       
          0,                                        /* customType : not used */         
          0,                                        /* customIndex: not used */     
          pa_EMAC_PORT_0,                           /* emacCtrl: for EMAC only */    
          NULL}                                     /* No commands */
    }                                
                                    
            
 };
 
 /* Define multi-route tables */
 static  pauMultiRoute_t pauMultiRouteInfo[] = {
    {   /* Entry 0 (MAC/IP Broadcast)  */
        0,               /* index */
        8,               /* nRoute */
        { 
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_0,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_1,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_2,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_3,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_4,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_5,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_6,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_7,
                0
            }
        }            
    },
    
    {   /* Entry 1 (MAC Entry 2) */
        5,               /* index */
        4,               /* nRoute */
        { 
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_10,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_11,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_12,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_13,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_14,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_15,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_16,
                T8_CMD_SWINFO0_PKT_ID + 2
            },
            {
                pa_MULTI_ROUTE_REPLACE_SWINFO,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_17,
                T8_CMD_SWINFO0_PKT_ID + 2
            }
        }            
    },
    
    {   /* Entry 2 (MAC/IP Multicast) */
        10,              /* index */
        4,               /* nRoute */
        { 
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_10,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_11,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_12,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_13,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_14,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_15,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_16,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_17,
                0
            }
        }            
    },
    {   /* Entry 3 (IP Broadcast) (not used) */
        20,              /* index */
        6,               /* nRoute */
        { 
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_0,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_1,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_2,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_3,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_4,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_5,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_6,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_7,
                0
            }
        }            
    },
    
    {   /* Entry 4 (IP Multicast) (not used) */
        31,              /* index */
        3,               /* nRoute */
        { 
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_10,
                0
            },
            {
                pa_MULTI_ROUTE_DESCRIPTOR_ONLY,        
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_11,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_12,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_13,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_14,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_15,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_16,
                0
            },
            {
                0,       /* ctrlBitfield */
                0,       /* flow Id */
                TF_FIRST_GEN_QUEUE + Q_MULTI_17,
                0
            }
        }            
    }
 
 };
 
 uint8_t t8CmdAck[T8_MAX_CMDS_PER_BURST];
 
 
 #define T8_NUM_EXCEPTION_ROUTES    5
 static int t8ErouteTypes[] = {
    pa_EROUTE_L2L3_FAIL,
    pa_EROUTE_MAC_BROADCAST,
    pa_EROUTE_MAC_MULTICAST,
    pa_EROUTE_IP_BROADCAST,
    pa_EROUTE_IP_MULTICAST   
 };
 
 static paRouteInfo_t t8Eroutes[] = {
 
 #if 0
    /* TBD: This mode does not work at QT */
    /* LUT1 Failure */
 	{  pa_DEST_EMAC,    	/* Dest */
 	   0,					/* Flow ID */
 	   0,					/* queue */
 	   0,					/* Multi route */
 	   0,	                /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       pa_EMAC_CTRL_CRC_DISABLE | pa_EMAC_PORT_1,  /* emacCtrl: for EMAC only */    
       NULL                 /* No commands */
    },
#else
 	{  pa_DEST_DISCARD,    	/* Dest */
 	   0,					/* Flow ID */
 	   0,					/* queue */
 	   0,					/* Multi route */
 	   0,	                /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       pa_EMAC_CTRL_CRC_DISABLE | pa_EMAC_PORT_1,  /* emacCtrl: for EMAC only */    
       NULL                 /* No commands */
    },
#endif

    /* MAC Broadcast (not used at NSS_GEN2) */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   0,					/* queue */
 	   0,					/* Multi route */
 	   T8_MAC_BROADCAST_PKT_INDEX + T8_CMD_SWINFO0_PKT_ID,	/* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* MAC Multicast (not used at NSS_GEN2) */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   0,					/* queue */
 	   10,					/* Multi route */
 	   T8_MAC_MULTICAST_PKT_INDEX + T8_CMD_SWINFO0_PKT_ID,	/* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* IP Broadcast */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   0,					/* queue */
 	   0,					/* Multi route */
 	   T8_IP_BROADCAST_PKT_INDEX + T8_CMD_SWINFO0_PKT_ID,	/* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* IP Multicast */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   0,					/* queue */
 	   10,					/* Multi route */
 	   T8_IP_MULTICAST_PKT_INDEX + T8_CMD_SWINFO0_PKT_ID,	/* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    }
    
 };

static	paQueueBounceConfig_t t8QueueBounceCfg =
	        {
	            1,      /* Enable */
                Q_BOUNCE_DDR  + TF_FIRST_GEN_QUEUE,             /* ddrQueueId */
                Q_BOUNCE_MSMC + TF_FIRST_GEN_QUEUE,             /* msmcQueueId */
                TF_PA_TX_QUEUE_BASE,                            /* hwQueueBegin */
                TF_PA_TX_QUEUE_BASE + NSS_NUM_TX_QUEUES - 1,    /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* EQoS mode */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_DDR      /* All traffics */
                }
	        };

static  paSysConfig_t  t8GlobalCfg =
        {
            NULL,                   /* pProtoLimit */
            NULL,                   /* pOutIpReassmConfig */
            NULL,                   /* pInIpReassmConfig */
            NULL,                   /* pCmdSetConfig */
            NULL,                   /* pUsrStatsConfig */
            NULL,                   /* pQueueDivertConfig */
            NULL,                   /* pPktControl */
            &t8QueueBounceCfg       /* pQueueBounceConfig */
        };
									        
/* Prototype required due to circular function calling */									        
static paTestStatus_t testCheckStats (tFramework_t *tf, paTest_t *pat, Bool clear, t8Handles_t *l2Handles);			


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
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t8PktTestInfo[pktIdx].pkt)), t8PktTestInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t8PktTestInfo[pktIdx].pktLen);
  	
  	return (hd);
}
  
								        
/* Recycle delete commands and command recycles  */
int t8StateDel (tFramework_t *tf, paTest_t *pat, t8Handles_t *l2Handles)
{
	Cppi_HostDesc  *hd;
	paEntryHandle_t reth;
	paReturn_t      paret;
	int				htype;
	int  			cmdDest;
    
    utilCycleDelay (1000);
			
	while (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)  {
				
		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
		if ((hd->softwareInfo0 & 0xffff0000u) != T8_CMD_SWINFO0_DEL_ID)  {
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
			
		l2Handles[hd->softwareInfo0 & 0xffff].state = L2_HANDLE_DISABLED;
				
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
		
	return (0);
		
}
														        									        									
	
static void paTestRecoverAndExit (tFramework_t *tf, paTest_t *pat, t8Handles_t *l2Handles, paTestStatus_t status, Bool doStats)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
 	uint16_t			cmdSize;
	int 			i, j, m;
	//volatile int    mdebugWait = 0;
	
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
	cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];

	
	/* Delete all the handles */
	for (i = 0; i < T8_NUM_LOCAL_HANDLES; i++)  {
		
		cmdReply.replyId = T8_CMD_SWINFO0_DEL_ID + i;
		hd = testCommonDelHandle (tf, &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, &cmdReply, &cmdDest, &cmdSize, &paret);
		
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): PA LLD returned error code %d on handle deletion\n", tfName, __FILE__, __LINE__, paret);
			status = PA_TEST_FAILED;
			break;
		}
		
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): No descriptor available for del handle command\n", tfName, __FILE__, __LINE__);
			status = PA_TEST_FAILED;
			break;
		}
		
		// mdebugHaltPdsp (0); 
		/* Send the command to the PA */
		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
		paTestExpectedStats.classify1.nPackets += 1;
		
		//while (mdebugWait);
        
		/* Wait to send the command until half of the rx buffers are available */
		if (t8StateDel (tf, pat, l2Handles))  {
			status = PA_TEST_FAILED;
			break;
		}
        
	}
	
	/* Give some time for remaining commands to complete */
	for (i = 0; i < 100; i++)  {
		
		for (j = m = 0; j < T8_NUM_LOCAL_HANDLES; j++)
			if (l2Handles[j].state != L2_HANDLE_DISABLED)
				m += 1;
				
		if (m)  {
			if (t8StateDel (tf, pat, l2Handles))  {
				status = PA_TEST_FAILED;
				break;
			}
			utilCycleDelay (1000);
		} else
			break;
		
	}
	
	if (i >= 100)  {
		System_printf ("%s (%s:%d): Unable to delete all L2 handles. %d remain undeleted\n", tfName, __FILE__, __LINE__, m);
		status = PA_TEST_FAILED;
	}
	
	/* Verify the stats are as expected */
	if (testCheckStats (tf, pat, TRUE, l2Handles) == PA_TEST_FAILED)
	  status = PA_TEST_FAILED;
	
	/* Test result */
	pat->testStatus = status;
	
	/* Return */
	Task_exit();
}		
				
/* Look for command replies from PA */					     
void testL2CmdRep (tFramework_t *tf, paTest_t *pat, t8Handles_t *localHandles)
{
	Cppi_HostDesc  *hd;
	uint32_t         *swInfo0;
	uint32_t 			swInfoCmd;
	unsigned int			lid;
	paReturn_t      paret;
	paEntryHandle_t reth;
	int				htype;
	int				cmdDest;
	
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_REPLY]) > 0)  {
		
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_REPLY]))) & ~15);
		if (Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **) &swInfo0) == CPPI_EPIB_NOT_PRESENT)  {
			System_printf ("%s (%s:%d): Found descriptor in PA command reply queue without EIPB present, failing\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		swInfoCmd = (*swInfo0 & 0xffff0000u); 
		lid = *swInfo0 & 0xffffu;
        
        
		/* Verify expected value in swinfo0 16 msbs */
        switch (swInfoCmd)
        {
            case T8_CMD_SWINFO0_ADD_ID:
            case T8_CMD_SWINFO0_DEL_ID:
		        /* Extract the local instance value */
		        if (lid >= T8_NUM_LOCAL_HANDLES)  {
			        System_printf ("%s (%s:%d): Received PA command reply for out of range local handle %d (max %d)\n", tfName, __FILE__, __LINE__, lid, T8_NUM_LOCAL_HANDLES);
			        testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			        paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		        }
		
		        /* Send the reply back to PA to let the driver know the command has been accepted */
		        paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
		        if (paret != pa_OK)  {
			        System_printf ("%s (%s:%d): paForwardResult returned error %d in response to paAddMac reply from PA\n", tfName, __FILE__, __LINE__);
			        testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			        paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		        }
		
		        /* Make sure the handle returned by PA matches the local copy */
		        if (localHandles[lid].paHandle != reth.l2l3Handle)  {
			        System_printf ("%s (%s:%d): paForwardResult returned handle (0x%08x) that did match internal table value (0x%08x)\n", tfName, __FILE__, __LINE__, (uint32_t)(localHandles[lid].paHandle), (uint32_t) reth.l2l3Handle);
			        testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			        paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		        }
		
		        if (swInfoCmd == T8_CMD_SWINFO0_ADD_ID)
			        localHandles[lid].state = L2_HANDLE_ACTIVE;
		        else
			        localHandles[lid].state = L2_HANDLE_UNCONFIGURED;
                    
                break;
                
             case T8_CMD_SWINFO0_EROUTE_CFG_ID:
             case T8_CMD_SWINFO0_MROUTE_CFG_ID:
             case T8_CMD_SWINFO0_GLOB_CFG_ID:
		        /* Extract the local instance value */
		        if (lid >= T8_MAX_CMDS_PER_BURST)  {
			        System_printf ("%s (%s:%d): Received PA command reply for out of range command id %d (max %d)\n", tfName, __FILE__, __LINE__, lid, T8_MAX_CMDS_PER_BURST);
			        testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			        paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		        }
             
		        /* Send the reply back to PA to let the driver know the command has been accepted */
		        paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
		        if (paret != pa_OK)  {
                    if (swInfoCmd == T8_CMD_SWINFO0_EROUTE_CFG_ID)
			            System_printf ("%s (%s:%d): paForwardResult returned error %d in response to Pa_configExceptionRoute reply from PA\n", tfName, __FILE__, __LINE__);
                    else if (swInfoCmd == T8_CMD_SWINFO0_MROUTE_CFG_ID)
			            System_printf ("%s (%s:%d): paForwardResult returned error %d in response to Pa_configMultiRouter reply from PA\n", tfName, __FILE__, __LINE__);
                    else
			            System_printf ("%s (%s:%d): paForwardResult returned error %d in response to Pa_control(Global Configuration) reply from PA\n", tfName, __FILE__, __LINE__);

			        testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			        paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		        }
                
                t8CmdAck[lid] = TRUE;
                
                break;
             
        
        
             default:
			    System_printf ("%s (%s:%d): Found descriptor in PA command reply queue without command reply swinfo0\n", tfName, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			    paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
                break;
        
        }
        
		/* Recycle the descriptor and buffer */
		if (testCommonRecycleLBDesc (tf, hd))  {
		    System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
		    paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
	}
	
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_RECYCLE]) > 0)  {
		
		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_RECYCLE]))) & ~15);
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			paTestRecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
	}
		
}
								     
static void paL2HandleError (tFramework_t *tf, paTest_t *pat, t8Handles_t *l2Handles, paReturn_t paret, Cppi_HostDesc *hd)
{	 	  
    /* Check paret before the descriptor. If paret indicates failure the descriptor will be NULL */                    
 	if (paret != pa_OK)  {
 		System_printf ("%s (%s:%d): PA LLD API call failed with error code = %d\n", tfName, __FILE__, __LINE__, paret);
 		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
 	
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): PA LLD API call failed due to unavailable free linked buffer descriptor\n", tfName, __FILE__, __LINE__);
 		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
}

static paTestStatus_t t8GlobalConfiguration (tFramework_t *tf, paTest_t *pat, paSysConfig_t *pCfg, t8Handles_t *l2Handles)
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
    cmdReply.replyId  = T8_CMD_SWINFO0_GLOB_CFG_ID;
    cmdReply.queue    = tf->QGen[Q_CMD_REPLY];
    cmdReply.flowId   = tf->tfFlowNum[0];

    hd = testCommonGlobalConfig (tf, &ctrlInfo,
                                 tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3,
                                 &cmdReply, &cmdDest, &cmdSize, &paret);

    paL2HandleError (tf, pat, l2Handles, paret, hd);  /* Will not return on error */

    /* Send command */
    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    /* All the packets should have been acked */
    /* Wait for a PA reply */
    t8CmdAck[0] = FALSE;
    for (i = 0; i < 100; i++)  {
        utilCycleDelay (1000);
        testL2CmdRep (tf, pat, l2Handles);
        if (t8CmdAck[0])
            break;
    }

    if (i == 100)  {
        System_printf ("%s: (%s:%d): Pa_control (Global Configuration) commands was not acked\n", tfName, __FILE__, __LINE__);
    return (PA_TEST_FAILED);
    }

    return (PA_TEST_PASSED);
}

/* Check the stats */

static paTestStatus_t testCheckStats (tFramework_t *tf, paTest_t *pat, Bool clear, t8Handles_t *l2Handles)
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
 	cmdReply.replyId = T8_CMD_SWINFO0_STATS_REQ_ID;
 	if (testCommonRequestPaStats (tfName, tf, clear, tf->QLinkedBuf1, tf->QGen[Q_CMD_RECYCLE],  &cmdReply))  {
 		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tfName, __FILE__, __LINE__);
 		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
 	}
 	
 	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tfName, __FILE__, __LINE__);
		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	
	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tfName, __FILE__, __LINE__);
		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tfName, __FILE__, __LINE__);
		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
 		
 	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
	paStats = (paSysStats_t *)Pa_formatStatsReply (tf->passHandle, (paCmd_t)bp);
    #else

	if (testCommonQueryPaStats (tfName, tf, TRUE, paStats))  {
		System_printf ("%s (%s:%d): testCommonQueryPaStats f\ailed\n", tfName, __FILE__, __LINE__);
		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	#endif
      
    if (testCommonCompareStats (tfName, (paSysStats_t *)&paTestExpectedStats, paStats))
    	status = PA_TEST_FAILED;
    else
    	status = PA_TEST_PASSED;   
    	  
    #ifndef NSS_GEN2      
     /* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tfName, __FILE__, __LINE__);
		status = PA_TEST_FAILED;
	}
    #endif
	
	return (status);
}


static Cppi_HostDesc *t8GetRxPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
	if (Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_MATCH])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
        
        return (hd);
	}	
		
	if (Qmss_getQueueEntryCount(tf->QGen[Q_NFAIL]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_NFAIL])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
        
        return (hd);
	}
    
    for (index = Q_MULTI_START; index <= Q_MULTI_END; index++)
    {
    
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }
        
            return (hd);
	    }	
    
    }
    
   return (NULL); 
	
}

	
	
/* Search the receive data packet queue for received data packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t8ReceiveDataPkts (tFramework_t *tf, paTest_t *pat, uint8_t *actualPktCount, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTest8Info_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               count = 0;
    int               ddrBounceCount=0;
	
	for (i = 0; i < 200; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);
		while ((hd = t8GetRxPkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & 0xffff0000) != T8_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & 0xffff;
            count++;
			if (chan <= T8_MAX_CHAN)
			  actualPktCount[chan] += 1;
			  
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t8PktTestInfo) / sizeof(pktTest8Info_t); j++)  {
				if (t8PktTestInfo[j].idx == chan)  {
                    /*
                     * Both IP and MAC boradcast and multicast packets share the same IDs. Use pkt[19] to determine which type it uis.
                     */
                    #if T8_MAC_BROADCAST_PKT_INDEX == T8_IP_BROADCAST_PKT_INDEX
                    uint8_t* pkt = (uint8_t *)hd->buffPtr; 
                    if ((chan == T8_IP_BROADCAST_PKT_INDEX) || (chan == T8_IP_MULTICAST_PKT_INDEX)) 
                    {
                        j += (pkt[19]?2:0);
                    }
                    #endif
                    
					tinfo = &t8PktTestInfo[j];
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
	
	if (i >= 200)  {
		System_printf ("%s (%s:%d): Timeout -  receive only %d out of (%d) packets\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}
	
    if(ddrBounceCount != count)
        System_printf("t8ReceiveDataPkts: receives %d queue bounce packets and %d final packets\n", ddrBounceCount, count);

    return (0);

}

/* setup the Test Pkt for pktdma */
int t8SetupPktTestInfo(pktTest8Info_t* testInfoPkt, int count, char* tfname)
{
	int returnVal = 0;
#ifdef __LINUX_USER_SPACE
	int i,pktSz;
	uint8_t *pkt;
	for (i = 0; i < count; i++)  {
		/* Store the original information */
        pkt   = testInfoPkt[i].pkt;
		pktSz = testInfoPkt[i].pktLen;
        /* Allocate memory for the Packet buffers from Framework, to control phy2Virt and Virt2Phy */
        testInfoPkt[i].pkt = (uint8_t *)fw_memAlloc(pktSz, CACHE_LINESZ);
        if(testInfoPkt[i].pkt == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfname, i);
  	        returnVal = -1;
        }
		/* Restore the original pkt information in the allocated memory */
        memcpy(testInfoPkt[i].pkt, pkt, pktSz);
    }
#endif

	return (returnVal);
}	
 
#ifdef __LINUX_USER_SPACE
void* paTestMultiRouting (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
void paTestMultiRouting (UArg a0, UArg a1)
{
 	tFramework_t   *tf  = (tFramework_t *)a0;
 	paTest_t       *pat = (paTest_t *)a1;
#endif
 	Cppi_HostDesc  *hd[8];
 	paReturn_t      paret;
 	t8Handles_t     l2Handles[T8_NUM_LOCAL_HANDLES];
 	int				n, i, j, l;
 	int				state;
 	int				count;
 	int  			cmdDest;
 	Bool			halt;
 	uint16_t			cmdSize;
 	paTestStatus_t  testStatus = PA_TEST_PASSED;
 	uint8_t			expectedPktCount[T8_MAX_CHAN+1];
 	uint8_t			actualPktCount[T8_MAX_CHAN+1];

 	
 	//volatile int mdebugWait = 1;
 	
 	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 		
    i = t8SetupPktTestInfo(t8PktTestInfo, (sizeof(t8PktTestInfo) / sizeof(pktTest8Info_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    /* Runtime initial values */
    cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];
    
    /* Zero out the l2Handle array and packet counts */
    memset (l2Handles, 0, sizeof(l2Handles));
    memset (expectedPktCount, 0, sizeof(expectedPktCount));
    memset (actualPktCount, 0, sizeof(actualPktCount));
    
    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestExpectedStats, 0, sizeof(paTestExpectedStats));

    testStatus = t8GlobalConfiguration (tf, pat, &t8GlobalCfg, l2Handles);
    if (testStatus != PA_TEST_PASSED)
        paTestRecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
 	
 	/* Add the multiple MAC entries in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. */
    n = sizeof(t8EthAndSwinfo)/sizeof(t8EthAndRoute_t); 
 	for (i = 0; i < n; i++)  {
        t8EthAndSwinfo[i].match.flowId = tf->tfFlowNum[0];
 		cmdReply.replyId = T8_CMD_SWINFO0_ADD_ID + i;
 		hd[i] = testCommonAddMac (tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t8EthAndSwinfo[i].eth, &t8EthAndSwinfo[i].match, 
                            &t8EthAndSwinfo[i].nfail, 
 	                        &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 0; i < n; i++)  {
 		//if (mdebugWait) mdebugHaltPdsp(0);  
 		
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		
 		//while (mdebugWait); 
 		
 		l2Handles[i].state = L2_HANDLE_PENDING_ACK;
 		paTestExpectedStats.classify1.nPackets += 1;
 		
 	}
 	
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
		testL2CmdRep (tf, pat, l2Handles);
 		
 		state = 1;
 		for (j = 0; j < n; j++)  {
 			if (l2Handles[j].state == L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	if ((i == 100) && (state == 0))  {
 		System_printf ("%s: (%s:%d): Burst of addMac commands did not result in all acks from PA\n", tfName, __FILE__, __LINE__);
 		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);
 	}
    
    
 	/* Add the multi-route entries in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one.  */
    n = sizeof(pauMultiRouteInfo)/sizeof(pauMultiRoute_t); 
 	for (i = 0; i < n; i++)  {
        int j;
        for(j = 0; j < pauMultiRouteInfo[i].nRoute; j++)
            pauMultiRouteInfo[i].routeEntry[j].flowId = tf->tfFlowNum[0];    
 		cmdReply.replyId = T8_CMD_SWINFO0_MROUTE_CFG_ID + i;
 		hd[i] = testCommonConfigMultiRoute (tf, pa_MULTI_ROUTE_MODE_CONFIG, pauMultiRouteInfo[i].index, pauMultiRouteInfo[i].nRoute,  
 	                                        pauMultiRouteInfo[i].routeEntry, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 0; i < n; i++)  {
        //mdebugWait = 1;
 		//if (mdebugWait) mdebugHaltPdsp(4);  
 		
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		
 		//while (mdebugWait); 
 		
 		t8CmdAck[i] = FALSE;
 		
 	}
 	
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
		testL2CmdRep (tf, pat, l2Handles);
 		
 		state = 1;
 		for (j = 0; j < n; j++)  {
 			if (t8CmdAck[i] == FALSE)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	if ((i == 100) && (state == 0))  {
 		System_printf ("%s: (%s:%d): Burst of Pa_configMultiRoute commands did not result in all acks from PA\n", tfName, __FILE__, __LINE__);
 		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);
 	}
    
    
    /* Issue the exception route command */
    cmdReply.replyId = T8_CMD_SWINFO0_EROUTE_CFG_ID;
    for(i = 0; i < T8_NUM_EXCEPTION_ROUTES; i++)
        t8Eroutes[i].flowId = tf->tfFlowNum[0];    
 	hd[0] = testCommonConfigExceptionRoute (tf, T8_NUM_EXCEPTION_ROUTES, t8ErouteTypes, t8Eroutes,  
 	                                        tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3, 
 	                                        &cmdReply, &cmdDest, &cmdSize, &paret, FALSE);
 	paL2HandleError (tf, pat, l2Handles, paret, hd[0]);  /* Will not return on error */
    
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    t8CmdAck[0] = FALSE;
    
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
		testL2CmdRep (tf, pat, l2Handles);
 		if(t8CmdAck[0])break;
 	}
 	
 	if ((i == 100) && (state == 0))  {
 		System_printf ("%s: (%s:%d): Pa_configExceptionRoute commands did not result in all acks from PA\n", tfName, __FILE__, __LINE__);
 		paTestRecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);
 	}
    
 	
 	/* Check and clear the stats */
 	testStatus = testCheckStats (tf, pat, TRUE, l2Handles);
	memset (&paTestExpectedStats, 0, sizeof(paTestExpectedStats));
	
	if (testStatus != PA_TEST_PASSED)
		paTestRecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
		
	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0, halt = FALSE; (j < 1) && (halt == FALSE); j++)   {
        count = 0;
		for (i = 0; (i < sizeof(t8PktTestInfo) / sizeof(pktTest8Info_t)) && (halt == FALSE);  )  {
		
			hd[i] = formDataPacket (tf, pat, i, expectedPktCount);
			
			if (hd[i] == NULL)  {
				halt = TRUE;
				break;
			}
			
			/* Inc the count if the packet is passed back to the host */
  			if (t8PktTestInfo[i].idx >= 0)
            {
	  			expectedPktCount[t8PktTestInfo[i].idx] += t8PktTestInfo[i].multiCount;
                count += t8PktTestInfo[i].multiCount;
            }
			/* Increment any expected stats */
			testCommonIncStats (t8PktTestInfo[i].statsMap, &paTestExpectedStats);	
			
			/* Proceed to the next packet */
			i += 1;  				

		}
			
        //mdebugWait = 1;
 		//if (mdebugWait) mdebugHaltPdsp(4);  
			
		for (l = 0; l < sizeof(t8PktTestInfo) / sizeof(pktTest8Info_t); l++)
        {
            //mdebugHaltPdsp(4);
			Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[l], hd[l]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 		    utilCycleDelay (10000);
            //while (mdebugWait);
        }    
            
	    //while (mdebugWait);
        				
		if (t8ReceiveDataPkts (tf, pat, actualPktCount, count))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t8ReceiveDataPkts failed\n", tfName,
						   __FILE__, __LINE__);
            System_flush();               
			break;
        }    
	}
	
	/* Verify that the expected and actual received packet counts match */
	for (i = 0; i <= T8_MAX_CHAN; i++)  {
		if (expectedPktCount[i] != actualPktCount[i])  {
			System_printf ("%s (%s:%d): Packet count mismatch for entry %d - expected %d, found %d\n", tfName,
						   __FILE__, __LINE__, i, expectedPktCount[i], actualPktCount[i]);
            System_flush();               
			testStatus = PA_TEST_FAILED;
		}
	}
    
	/* Clean up and return */
 	paTestRecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
 	
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
}
 		
 
 		 
 
 	
 	


