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



#ifndef TEST7PKTS_H_
#define TEST7PKTS_H_



/* Subtest A - custom level 4 configuration.
 *  One mac, two IPs, two UDPs, custom L4 setup with link, 3 custom L4s */



/* SWINFO 0 identifiers, always in the 16 MSBs */
#define T7_SWINFO0_MAC_NROUTE_FAIL	0x07010000
#define T7_SWINFO0_IP_NROUTE_FAIL   0x07020000
#define T7_SWINFO0_UDP_MATCH		0x07030000
#define T7_SWINFO0_CUSTOM_C1_MATCH  0x07040000
#define T7_SWINFO0_CUSTOM_C2_MATCH  0x07050000
#define T7_SWINFO0_C1_NROUTE_FAIL   0x07060000
#define T7_CMD_SWINFO0_CRC_CFG_ID   0x07070000
#define T7_CMD_SWINFO0_GLOBAL_CFG_ID 0x07080000

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSetCmd4, ".testPkts")
#endif
static paCmdInfo_t t7CmdSetCmd4 =
    {
        pa_CMD_CMDSET,
        {

            {
                4                        /* Command set index */
            }
        }
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSetCmd11, ".testPkts")
#endif
static paCmdInfo_t t7CmdSetCmd11 =
    {
        pa_CMD_CMDSET,
        {

            {
                11                        /* Command set index */
            }
        }
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSetCmd22, ".testPkts")
#endif
static paCmdInfo_t t7CmdSetCmd22 =
    {
        pa_CMD_CMDSET,
        {

            {
                22                        /* Command set index */
            }
        }
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSetCmd44, ".testPkts")
#endif
static paCmdInfo_t t7CmdSetCmd44 =
    {
        pa_CMD_CMDSET,
        {

            {
                44                        /* Command set index */
            }
        }
    };


#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSetCmd55, ".testPkts")
#endif
static paCmdInfo_t t7CmdSetCmd55 =
    {
        pa_CMD_CMDSET,
        {

            {
                55                        /* Command set index */
            }
        }
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7AMacSetup, ".testPkts")
#endif
static pauTestMacSetup_t t7AMacSetup[] =  {

	{  /* entry 0 */
		{	/* paEthInfo */
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 		{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
    		0,      /* Vlan      */
    		0,      /* ethertype */
    	    0,		/* mpls tag  */
            0       /* input EMAC port */
    	},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT1,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
            0,                          /* customIndex: not used  */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_MAC_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

   	 	pa_OK,		/* Expected PA return value */
   	 	0,			/* returned PA handle */
    	TRUE,		/* Wait for the return value */

    	TF_LINKED_BUF_Q2,							/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT			/* Status */
	},

	{   /* entry 1: */

		{	/* paEthInfo */
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 		{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x11 },      /* Dest mac */
    		0,      /* Vlan      */
    		0x800,  /* ethertype */
    	    0,		/* mpls tag  */
            0       /* input EMAC port */
    	},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT1,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            pa_CUSTOM_TYPE_LUT1,        /* customType */
            1,                          /* customIndex */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_MAC_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

   	 	pa_OK,		/* Expected PA return value */
   	 	0,			/* returned PA handle */
    	TRUE,		/* Wait for the return value */

    	TF_LINKED_BUF_Q2,							/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT			/* Status */
	},

	{   /* entry 2: */

		{	/* paEthInfo */
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 		{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x12 },      /* Dest mac */
    		0,      /* Vlan      */
    		0x800,  /* ethertype */
    	    0,		/* mpls tag  */
            0       /* input EMAC port */
    	},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT1,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            pa_CUSTOM_TYPE_LUT1,        /* customType  */
            2,                          /* customIndex */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_MAC_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

   	 	pa_OK,		/* Expected PA return value */
   	 	0,			/* returned PA handle */
    	TRUE,		/* Wait for the return value */

    	TF_LINKED_BUF_Q2,							/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT			/* Status */
	},

	{   /* entry 3: */

		{	/* paEthInfo */
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 		{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x13 },      /* Dest mac */
    		0,      /* Vlan      */
    		0x800,  /* ethertype */
    	    0,		/* mpls tag  */
            0       /* input EMAC port */
    	},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT1,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            pa_CUSTOM_TYPE_LUT1,        /* customType  */
            4,                          /* customIndex */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_MAC_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

   	 	pa_ERR_CONFIG,	/* Expected PA return value */
   	 	0,			    /* returned PA handle */
    	TRUE,		    /* Wait for the return value */

    	TF_LINKED_BUF_Q2,							/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT			/* Status */
	}
};



#ifdef _TMS320C6X
#pragma DATA_SECTION(t7AIpSetup, ".testPkts")
#endif
static pauTestIpSetup_t t7AIpSetup[] =  {

	/* Entry 0 */
	{
			/* paIpInfo */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  			/* IP Source address */
			{ 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 	/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT2,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            pa_CUSTOM_TYPE_LUT2,        /* customType  */
            0,                          /* customIndex */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_IP_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

		&t7AMacSetup[0],    	/* Link to MAC address */
		NULL,				    /* Link to IP address */
		pa_OK,				    /* expected PA return value */
		0,					    /* returned PA handle */
		FALSE,				    /* Don't wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 1 */
	{
			/* paIpInfo */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  			/* IP Source address */
			{ 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 	/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT2,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            pa_CUSTOM_TYPE_LUT2,        /* customType  */
            15,                         /* customIndex */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_IP_NROUTE_FAIL + 1,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},


		&t7AMacSetup[0],	/* Link to MAC address */
		NULL,				/* Link to IP address */
		pa_OK,				/* expected PA return value */
		0,					/* returned PA handle */
		TRUE,				/* Wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 2 - This one is not linked */
	{
			/* paIpInfo */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  			/* IP Source address */
			{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 	/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT2,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
            0,                          /* customIndex: not used  */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_IP_NROUTE_FAIL + 1,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},


		&t7AMacSetup[0],	/* Link to MAC address */
		NULL,				/* Link to IP address */
		pa_OK,				/* expected PA return value */
		0,					/* returned PA handle */
		TRUE,				/* Wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 3 */
	{
			/* paIpInfo */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  			/* IP Source address */
			{ 200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 	/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},

    	{  /* Match Routing */
    		pa_DEST_CONTINUE_PARSE_LUT2,/* Destination */
    		0,							/* Flow ID */
    		0,							/* queue */
    		-1,							/* Multi route index */
    		0,							/* software info 0 */
            0,                          /* software info 1 */
            pa_CUSTOM_TYPE_LUT2,        /* customType  */
            10,                         /* customIndex */
            0,                          /* pkyType: for SRIO only */
            NULL                        /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_IP_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

		&t7AMacSetup[0],    	/* Link to MAC address */
		NULL,				    /* Link to IP address */
		pa_OK,				    /* expected PA return value */
		0,					    /* returned PA handle */
		FALSE,				    /* Don't wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},


};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7AL4Setup, ".testPkts")
#endif
static pauTestL4Setup_t t7AL4Setup[] = {

	{	/* Entry 0 */
		2144,				/* Dest Port = not GTPU port */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_UDP_MATCH,	/* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_UDP_MATCH + 0,			/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            NULL                                /* No commands            */
    	},

		&t7AIpSetup[0], 	/* IP Link Link */
		pa_OK,				/* Expected return value */
		{ 0, 0 },			/* returned L4 handle */
		FALSE,				/* Dont wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	{	/* Entry 1 */
		2144,				/* Dest Port = not GTPU port */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_UDP_MATCH,	/* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_UDP_MATCH + 1,			/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            &t7CmdSetCmd11                      /* Command set            */
    	},

		&t7AIpSetup[2], 	/* IP Link Link */
		pa_OK,				/* Expected return value */
		{ 0, 0 },			/* returned L4 handle */
		TRUE,				/* Wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	}
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (t7ACl4Config, ".testPkts")
#endif
static pauTestCl4Config_t  t7ACl4Config[] = {

	/* Entry 0 should fail because of invalid byte offsets */
	{
		TRUE,						/* Link to previous IP */
		0,						    /* Custom Index */
        0,                          /* Custom Header size */
		{  1, 2, 4, 3},  			/* Byte offsets */
		{ 0xff, 0xff, 0xff, 0xff},	/* Byte masks */
		pa_ERR_CONFIG,				/* Expected PA return value */
		FALSE,						/* Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 1 is valid */
	{
		TRUE,						/* Link to previous IP */
		0,						    /* Custom Index */
        0,                          /* Custom Header size */
		{  10, 12, 14, 15},			/* Byte offsets */
		{ 0x0f, 0xf0, 0x3c, 0x00},	/* Byte masks */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 2 is valid */
	{
		TRUE,						/* Link to previous IP */
		15,						    /* Custom Index */
        0,                          /* Custom Header size */
		{  10, 12, 14, 15},			/* Byte offsets */
		{ 0x0f, 0xf0, 0x3c, 0x00},	/* Byte masks */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 3 is valid */
	{
		TRUE,						/* Link to previous IP */
		10,						    /* Custom Index */
        8,                          /* Custom Header size */
		{   0, 1, 2, 3},			    /* Byte offsets */
		{ 0xff, 0xff, 0x00, 0x00},	/* Byte masks */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 4 should fail because of invalid index */
	{
		TRUE,						/* Link to previous IP */
		16,						    /* Custom Index */
        0,                          /* Custom Header size */
		{  1, 2, 4, 3},  			/* Byte offsets */
		{ 0xff, 0xff, 0xff, 0xff},	/* Byte masks */
		pa_ERR_CONFIG,				/* Expected PA return value */
		FALSE,						/* Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSet4, ".testPkts")
#endif

static  paCmdCopy_t t7CopyHdr =
        {
            0,              /* ctrlBitfield */
            0,              /* source offset */
            20,             /* destOffset */
            4               /* number of bytes */
        };

static uint8_t  t7PatchData[4] = {0xba, 0xbe, 0xfa, 0xce};

static  paPatchInfo_t t7Patch =         /* For Packet 0 */
        {
            pa_PATCH_OP_INSERT,         /* ctrlBitfield */
            4,                          /* nPatchBytes */
            4,                          /* totalPatchSize */
            39,                         /* offset */
            t7PatchData                 /* patchData */
        };

static  paPatchInfo_t t7Patch_2 =      /* For Packet 2 */
        {
            pa_PATCH_OP_INSERT,         /* ctrlBitfield */
            4,                          /* nPatchBytes */
            4,                          /* totalPatchSize */
            22,                         /* offset */
            t7PatchData                 /* patchData */
        };

static  paCmdSplitOp_t t7SplitOP =
        {
            pa_SPLIT_OP_FRAME_TYPE,     /* ctrlBitfield */
            20,                         /* startOffset */
            pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2,       /* frameType */
            TF_FIRST_GEN_QUEUE + Q_PAYLOAD_MATCH,            /* Destination queue */
            0                           /* flow Id */
        };


static  paCmdCrcOp_t t7CrcOP =
        {
            pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
            pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
            pa_CRC_OP_CRC_FRAME_TYPE            |
            pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
            20,                   /* startOffset */
            0,                    /* length */
            8,                    /* lengthOffset */
            0x0fff,               /* lenMask */
            22,                   /* lenAdjust */
            0,                    /* crcOffset */
            2,                    /* crcSize */
            pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2,      /* frameType */
            0                     /* Init value */
        };

static  paCmdCopy_t t7CopyTail =
        {
            pa_COPY_OP_FROM_END,  /* ctrlBitfield */
            4,                    /* source offset */
            28,                   /* destOffset */
            4                     /* number of bytes */
        };

static paCmdInfo_t t7CmdSet4[] =                           /* packet 0 */
{
    /* Command 0: Copy from header (place hold) */
    {
        pa_CMD_COPY_DATA_TO_PSINFO,
        {
            {
                0,              /* ctrlBitfield */
                0,              /* source offset */
                24,             /* destOffset */
                4               /* number of bytes */
            }
        }
    },

    /* Command 1: Split (place hold) */
    {
        pa_CMD_SPLIT,
        {
            {
                0,              /* ctrlBitfield */
                0,              /* source offset */
                24,             /* destOffset */
                4               /* number of bytes */
            }
        }
    },


    /* Command 2: Insert prior to CRC payload  (place hold) */
    {
        pa_CMD_PATCH_DATA,
        {
            {
                pa_PATCH_OP_INSERT,         /* ctrlBitfield */
                4,                          /* nPatchBytes */
                4,                          /* totalPatchSize */
                39,                         /* offset */
                0                           /* patchData */
            }
        }
    },


    /* Command 3: CRC Operation (place hold) */
    {
        pa_CMD_CRC_OP,
        {
            {
                pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
                pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
                pa_CRC_OP_CRC_FRAME_TYPE            |
                pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
                20,                   /* startOffset */
                0,                    /* length */
                8,                    /* lengthOffset */
                0x0fff,               /* lenMask */
                22,                   /* lenAdjust */
                0,                    /* crcOffset */
                pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2       /* frameType */
            }
        }
    },


    /* Command 4: Copy from Tail (place hold) */
    {
        pa_CMD_COPY_DATA_TO_PSINFO,
        {
            {
                pa_COPY_OP_FROM_END,  /* ctrlBitfield */
                4,                    /* source offset */
                28,                   /* destOffset */
                4                     /* number of bytes */
            }
        }
    }

};

static  paCmdSplitOp_t t7SplitOP_2 =
        {
            0,                          /* ctrlBitfield */
            22,                         /* startOffset */
            0,                          /* frameType */
            TF_FIRST_GEN_QUEUE + Q_PAYLOAD_MATCH,            /* Destination queue */
            0                           /* flow Id */
        };


static  paCmdCrcOp_t t7CrcOP_1 =                            /* packet 1 */
        {
            pa_CRC_OP_CRC_VALIDATE                          |           /* ctrlBitfield */
            pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER              |
            pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE     |
            pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
            14,                   /* startOffset */
            0,                    /* length */
            4,                    /* lengthOffset */
            0x0fff,               /* lenMask */
            25,                   /* lenAdjust */
            0,                    /* crcOffset */
            2,                    /* crcSize */
            0,                    /* frameType */
            0                     /* Init value */
        };

static  paCmdCrcOp_t t7CrcOP_2 =                            /* packet 2 */
        {
            pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
            pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
            pa_CRC_OP_CRC_FRAME_TYPE            |
            pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
            8,                    /* startOffset */
            0,                    /* length */
            4,                    /* lengthOffset */
            0x0fff,               /* lenMask */
            10,                   /* lenAdjust */
            0,                    /* crcOffset */
            2,                    /* crcSize */
            pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3,      /* frameType */
            0                     /* Init value */
        };

static  paCmdCrcOp_t t7CrcOP_4 =                            /* packet 4 */
        {
            pa_CRC_OP_CRC_VALIDATE                          |           /* ctrlBitfield */
            pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
            22,                   /* startOffset */
            83,                   /* length */
            0,                    /* lengthOffset */
            0,                    /* lenMask */
            0,                    /* lenAdjust */
            0,                    /* crcOffset */
            2,                    /* crcSize */
            0,                    /* frameType */
            0                     /* Init value */
        };

static  paCmdCrcOp_t t7CrcOP_5 =                            /* packet 5 */
        {
            pa_CRC_OP_CRC_VALIDATE,                          /* ctrlBitfield */
            22,                   /* startOffset */
            83,                   /* length */
            0,                    /* lengthOffset */
            0,                    /* lenMask */
            0,                    /* lenAdjust */
            105,                  /* crcOffset */
            2,                    /* crcSize */
            0,                    /* frameType */
            0                     /* Init value */
        };

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSet11, ".testPkts")
#endif


static paCmdInfo_t t7CmdSet11[] =                           /* packet 1 */
{

    /* Command 0: CRC Operation (place hold) */
    {
        pa_CMD_CRC_OP,
        {
            {
                pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
                pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
                pa_CRC_OP_CRC_FRAME_TYPE            |
                pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
                20,                   /* startOffset */
                0,                    /* length */
                8,                    /* lengthOffset */
                0x0fff,               /* lenMask */
                22,                   /* lenAdjust */
                0,                    /* crcOffset */
                pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2       /* frameType */
            }
        }
    }

};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSet22, ".testPkts")
#endif


static paCmdInfo_t t7CmdSet22[] =                           /* packet 2 */
{


    /* Command 0: Split (place hold) */
    {
        pa_CMD_SPLIT,
        {
            {
                0,              /* ctrlBitfield */
                0,              /* source offset */
                24,             /* destOffset */
                4               /* number of bytes */
            }
        }
    },

    /* Command 1: Insert prior to CRC payload  (place hold) */
    {
        pa_CMD_PATCH_DATA,
        {
            {
                pa_PATCH_OP_INSERT,         /* ctrlBitfield */
                4,                          /* nPatchBytes */
                4,                          /* totalPatchSize */
                39,                         /* offset */
                0                           /* patchData */
            }
        }
    },

    /* Command 2: CRC Operation (place hold) */
    {
        pa_CMD_CRC_OP,
        {
            {
                pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
                pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
                pa_CRC_OP_CRC_FRAME_TYPE            |
                pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
                20,                   /* startOffset */
                0,                    /* length */
                8,                    /* lengthOffset */
                0x0fff,               /* lenMask */
                22,                   /* lenAdjust */
                0,                    /* crcOffset */
                pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2       /* frameType */
            }
        }
    }

};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSet44, ".testPkts")
#endif


static paCmdInfo_t t7CmdSet44[] =                              /* packet 4 */
{

    /* Command 0: CRC Operation (place hold) */
    {
        pa_CMD_CRC_OP,
        {
            {
                pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
                pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
                pa_CRC_OP_CRC_FRAME_TYPE            |
                pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
                20,                   /* startOffset */
                0,                    /* length */
                8,                    /* lengthOffset */
                0x0fff,               /* lenMask */
                22,                   /* lenAdjust */
                0,                    /* crcOffset */
                pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2       /* frameType */
            }
        }
    }

};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7CmdSet55, ".testPkts")
#endif


static paCmdInfo_t t7CmdSet55[] =                              /* packet 5 */
{

    /* Command 0: CRC Operation (place hold) */
    {
        pa_CMD_CRC_OP,
        {
            {
                pa_CRC_OP_CRC_VALIDATE              |           /* ctrlBitfield */
                pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  |
                pa_CRC_OP_CRC_FRAME_TYPE            |
                pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD,
                20,                   /* startOffset */
                0,                    /* length */
                8,                    /* lengthOffset */
                0x0fff,               /* lenMask */
                22,                   /* lenAdjust */
                0,                    /* crcOffset */
                pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2       /* frameType */
            }
        }
    }

};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t7ACmdSetSetup, ".testPkts")
#endif

static pauTestCmdSetSetup_t t7ACmdSetSetup[] = {
    {   /* Entry 0 */
        4,                          /* Command Set Index */
        5,                          /* number of commands (no need for next route command) */
        &t7CmdSet4[0],              /* command array */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Dont wait for the command to complete */
    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
    },

    {   /* Entry 1 */
        11,                         /* Command Set Index */
        1,                          /* number of commands (no need for next route command) */
        &t7CmdSet11[0],             /* command array */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Dont wait for the command to complete */
    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
    },

    {   /* Entry 2 */
        22,                         /* Command Set Index */
        3,                          /* number of commands (no need for next route command) */
        &t7CmdSet22[0],             /* command array */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Dont wait for the command to complete */
    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
    },

    {   /* Entry 3 */
        44,                         /* Command Set Index */
        1,                          /* number of commands (no need for next route command) */
        &t7CmdSet44[0],              /* command array */
		pa_OK,						/* Expected PA return value */
		FALSE,						/* Dont wait for the command to complete */
    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
    },

    {   /* Entry 4 */
        55,                         /* Command Set Index */
        1,                          /* number of commands (no need for next route command) */
        &t7CmdSet55[0],              /* command array */
		pa_OK,						/* Expected PA return value */
		TRUE,						/* Wait for the command to complete */
    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
    }
};


#ifdef _TMS320C6X
#pragma DATA_SECTION(t7ACl4Setup, ".testPkts")
#endif
static pauTestCl4Setup_t t7ACl4Setup[] = {

	{   /* Entry 0 */
        0,                              /* Custom Index */
		{ 0x0a, 0xa0, 0x38, 0x00 },		/* Table match values */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,	    /* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C2_MATCH + 0,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            &t7CmdSetCmd4                       /* Command set            */
    	},

		&t7AIpSetup[0],					/* Link to IP handle */
		pa_OK,							/* Expected PA return value */
		{ 0, 0},						/* Returned handle */
		FALSE,							/* Dont wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	{  /* Entry 1 */
        15,                             /* Custom Index */
		{ 0x0a, 0xa0, 0x38, 0x00 },		/* Table match values */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,		/* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C2_MATCH + 1,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            &t7CmdSetCmd22                      /* Command set            */
    	},

		&t7AIpSetup[1],					/* Link to IP handle */
		pa_OK,							/* Expected PA return value */
		{ 0, 0},						/* Returned handle */
		FALSE,							/* Dont wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	}
    ,
	{  /* Entry 2 */
        15,                             /* Custom Index */
		{ 0x0f, 0xf0, 0x3c, 0x00 },		/* Table match values */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,		/* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C2_MATCH + 2,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            NULL                                /* No commands            */
    	},

		&t7AIpSetup[2],					/* Link to IP handle */
		pa_OK,							/* Expected PA return value */
		{ 0, 0},						/* Returned handle */
		TRUE,							/* Dont wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	{  /* Entry 3 */
        10,                             /* Custom Index */
		{ 0xaa, 0xbb, 0x00, 0x00 },		/* Table match values */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,		/* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C2_MATCH + 3,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            NULL                                /* No commands            */
    	},

		&t7AIpSetup[3],					/* Link to IP handle */
		pa_OK,							/* Expected PA return value */
		{ 0, 0},						/* Returned handle */
		TRUE,							/* wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	{  /* Entry 3 */
        10,                             /* Custom Index */
		{ 0xaa, 0xbb, 0x00, 0x00 },		/* Table match values */

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,		/* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C2_MATCH + 3,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            NULL                                /* No commands            */
    	},

		&t7AIpSetup[3],					/* Link to IP handle */
		pa_OK,							/* Expected PA return value */
		{ 0, 0},						/* Returned handle */
		TRUE,							/* wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	}

};

#ifdef _TMS320C6X
#pragma DATA_SECTION (t7ACl3Config, ".testPkts")
#endif
static pauTestCl3Config_t  t7ACl3Config[] = {

	/* Entry 0 should fail because of invalid byte offsets */
	{
		4,						    /* Custom Index */
        0,                          /* Byte offset */
        pa_HDR_TYPE_UDP,            /* Next header */
        20,                         /* Offset to the next header */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* Byte masks */
          0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		pa_ERR_CONFIG,				/* Expected PA return value */
		FALSE,						/* Don't Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 1 valid */
	{
		1,						    /* Custom Index */
        0,                          /* Byte offset */
        pa_HDR_TYPE_UDP,            /* Next header */
        20,                         /* Offset to the next header */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* Byte masks */
          0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		pa_OK,				        /* Expected PA return value */
		FALSE,						/* Don't Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	/* Entry 2 valid */
	{
		2,						    /* Custom Index */
        0,                          /* Byte offset */
        pa_HDR_TYPE_UDP,            /* Next header */
        20,                         /* Offset to the next header */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* Byte masks */
          0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		pa_OK,				        /* Expected PA return value */
		TRUE,						/* Wait for the command to complete */

    	TF_LINKED_BUF_Q3,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	}

};


#ifdef _TMS320C6X
#pragma DATA_SECTION(t7ACl3Setup, ".testPkts")
#endif
static pauTestCl3Setup_t t7ACl3Setup[] = {

	{   /* Entry 0 */
		1,						    /* Custom Index */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	          /* Byte masks */
          200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,	    /* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C1_MATCH + 0,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            &t7CmdSetCmd44                      /* Command set            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_C1_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},

		&t7AMacSetup[1],	/* Link to MAC address */
		NULL,				/* Link to IP address */
		pa_OK,				/* Expected PA return value */
   	 	0,			        /* returned PA handle */
		TRUE,				/* wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	{   /* Entry 1 */
		2,						    /* Custom Index */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* Byte masks */
          200, 201, 202, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,	    /* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C1_MATCH + 1,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            &t7CmdSetCmd55                      /* Command set            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_C1_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},


		&t7AMacSetup[2],	/* Link to MAC address */
		NULL,				/* Link to IP address */
		pa_OK,				/* Expected PA return value */
   	 	0,			        /* returned PA handle */
		TRUE,				/* wait for the command to complete */

    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	},

	{   /* Entry 2 */
		1,						    /* Custom Index */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* Byte masks */
          200, 201, 202, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

		{  /* Match Routing */
    		pa_DEST_HOST,						/* Destination */
    		0,									/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_MATCH,	    /* queue */
    		-1,									/* Multi route index */
    		T7_SWINFO0_CUSTOM_C1_MATCH + 2,		/* software info 0 */
            0,                                  /* software info 1 */
            0,                                  /* customType : not used  */
            0,                                  /* customIndex: not used  */
            0,                                  /* pkyType: for SRIO only */
            NULL                                /* No commands            */
    	},

    	{  /* Next fail routing */
    		pa_DEST_HOST,					/* Destination */
    		0,								/* Flow ID */
    		TF_FIRST_GEN_QUEUE + Q_NFAIL,	/* queue */
    		-1,								/* Multi route index */
    		T7_SWINFO0_C1_NROUTE_FAIL + 0,	/* software info 0 */
            0,                              /* software info 1 */
            0,                              /* customType : not used  */
            0,                              /* customIndex: not used  */
            0,                              /* pkyType: for SRIO only */
            NULL                            /* No commands            */
    	},


		&t7AMacSetup[1],	/* Link to MAC address */
		NULL,				/* Link to IP address */
		pa_OK,				/* Expected PA return value */
   	 	0,			        /* returned PA handle */
		TRUE,				/* wait for the command to complete */
    	TF_LINKED_BUF_Q2,						/* Free buffer queue */
    	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT		/* Command status */
	}

};


#ifdef _TMS320C6X
#pragma DATA_SECTION(t7ATestSetup, ".testPkts")
#endif
pauTestSetup_t t7ATestSetup =
{

		sizeof(t7AMacSetup) / sizeof(pauTestMacSetup_t),
		t7AMacSetup,

		sizeof(t7AIpSetup) / sizeof(pauTestIpSetup_t),
		t7AIpSetup,

		sizeof(t7AL4Setup) / sizeof(pauTestL4Setup_t),
		t7AL4Setup,

		sizeof(t7ACl3Config) / sizeof(pauTestCl3Config_t),
		t7ACl3Config,

		sizeof(t7ACl3Setup) / sizeof(pauTestCl3Setup_t),
		t7ACl3Setup,

		sizeof(t7ACl4Config) / sizeof(pauTestCl4Config_t),
		t7ACl4Config,

		sizeof(t7ACl4Setup) / sizeof(pauTestCl4Setup_t),
		t7ACl4Setup,

		sizeof(t7ACmdSetSetup) / sizeof(pauTestCmdSetSetup_t),
		t7ACmdSetSetup

};


/* Packet 0 should match custom entry 0. This packet tests MAC/IP to custom switch/custom L4 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(166,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 166, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_CUSTOM),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f6d)	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (IP Payload) */
	TF_FORM_PKT_INFO_WORD1(166,0,1,0,0,0),  /* end offset = 166, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_CUSTOM),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f6d)	/* Pseudo header checksum */
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x94, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x7e, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,  /* IP dest = 200.201.202.100 */
	0xca, 0x64, 0xba, 0xbe, 0xfa, 0xce, 0x00, 0x80,
	0x00, 0x00, 0x00, 0x7c, 0x1a, 0x00, 0xa1, 0x00,  /* bytes 10, 12, 14 are used for custom lookup */
	0xf8, 0x6b, 0x6c, 0x6d, 0x00, 0x6f, 0x70, 0x71,	 /* UDP payload byte 10 contains the packet index */
	0x2d, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,  /* 20-byte Properitery header = UDP Header + 12 byte */
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,  /* Payload = WCDMA FP HS-DSCH type 2 frame with 5 block (payload offset = 19) */
	0x82,                                            /* Mesage length at beginning of UDP payload (offset = 8) */
    0xC0, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,  /* standard test vector: size 83 (0x53) CRC = 0xDA26) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20,  /* Message length = 20 + 19 + 83 + 2 = 124 (0x7C) */
    0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0,  /* UDP Length =  124 + 4 (trail) = 128 (0x80) */
    0xB0, 0xC0, 0xD0, 0xE0, 0xF1, 0x01, 0x11, 0x21,
    0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1,
    0xB1, 0xC1, 0xD1, 0xE1, 0xF2, 0x02, 0x12, 0x22,
    0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92, 0xA2,
    0xB2, 0xC2, 0xD2, 0xE2, 0xF3, 0x03, 0x13, 0x23,
    0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3,
    0xB3, 0xC3, 0xD3, 0xE3, 0xF4, 0x04, 0x14, 0x24,
    0x34, 0x44, 0x50, 0xDA, 0x26, 0xde, 0xad, 0xbe,
    0xef  };

/* Packet 1 should not match a custom entry since IP2 does not trigger custom lookup, UDP match
 * This packet tests MAC/IP/UDP with no switch */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(1538,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f79)	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(1538,0,1,0,0,0),  /* end offset = 142, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f79)	/* Pseudo header checksum */
};
#endif

/* Mac index 0, IP index 2, UDP dest = 2144, no custom lookup triggered */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x05, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x72, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,         /* IP dest = 200.201.202.102 */
	0xca, 0x66, 0xaa, 0xbb, 0x08, 0x60, 0x05, 0xE0,
	0x00, 0x00, 0x32, 0x33, 0xfa, 0x35, 0xaf, 0x37,         /* Matches on data bytes 2,4 and 6 */
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,         /* CRC payload offset = 14 from UDP payload (start offset) */
                                                            /* Message length offset = 4 (negative) */
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,   /* standard test vector: size 1479 (0x5C7) CRC = 0xCF18) */
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,   /* Message(UDP) length = 8 + 14 + 1479 + 2 + 1= 1504 (0x5E0) */
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,   /* message length adjust = 14 + 8 + 2 + 1 = 25 */
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
    0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
    0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
    0x01, 0x52, 0x35, 0xCF, 0x18, 0x00
};


/* Packet 2 should match custom entry 1. This packet tests MAC/IP to custom switch/custom L4 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(145,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 141, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_CUSTOM),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f82)	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,34),		/* cmd len = 20, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(145,0,1,0,0,0),  /* end offset = 145, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_CUSTOM),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f82)	/* Pseudo header checksum */
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x69, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,  /* IP dest = 200.201.202.101 */
	0xca, 0x65, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6b,
	0x00, 0x00, 0x3c, 0x3d, 0xfa, 0x27, 0xaf, 0x41,  /* Custom match info */
	0xf8, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,

    0xC0, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, /* standard test vector: size 83 (0x53) CRC = 0xDA26) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, /* Payload = WCDMA FP HS-DSCH type 3 frame with 4 block (payload offset = 14) */
    0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, /* Message(UDP) length = 8 + 14 + 83 + 2 = 107 (0x6B) */
    0xB0, 0xC0, 0xD0, 0xE0, 0xF1, 0x01, 0x11, 0x21, /* message length adjust = 8 + 2 = 10 */
    0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1, /* CRC start offset (header plus payload) = 8 from UDP header (start offset) */
    0xB1, 0xC1, 0xD1, 0xE1, 0xF2, 0x02, 0x12, 0x22, /* Message length offset = 4 from UDP header (start offset) */
    0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92, 0xA2,
    0xB2, 0xC2, 0xD2, 0xE2, 0xF3, 0x03, 0x13, 0x23,
    0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3,
    0xB3, 0xC3, 0xD3, 0xE3, 0xF4, 0x04, 0x14, 0x24,
    0x34, 0x44, 0x50, 0xDA, 0x26};


/* Packet 3 matches the MAC/IP for custom lookup entry 1, but will fail
 * the custom lookup. This tests the nfail routing for custom lookup */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(136,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_CUSTOM),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f8c)	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,34),		/* cmd len = 20, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(136,0,1,0,0,0),  /* end offset = 136, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_CUSTOM),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f8c)	/* Pseudo header checksum */
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt3, ".testPkts")
#endif
static uint8_t pkt3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5f, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x65, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x66, /* IP dest = 200.201.202.101 */
	0x00, 0x00, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
	0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
	0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
	0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,};

/* Packet 4 should match custom entry 0. This packet tests MAC to custom switch/custom LUT1 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(141,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_CUSTOM),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f6d)	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,34),		/* cmd len = 20, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(141,0,1,0,0,0),  /* end offset = 141, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_CUSTOM),PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f6d)	/* Pseudo header checksum */
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4, ".testPkts")
#endif
static uint8_t pkt4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x11, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x7e, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,  /* IP dest = 200.201.202.103 */
	0xca, 0x67, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6B,
	0x00, 0x00, 0x64, 0x65, 0x1a, 0x00, 0xa1, 0x00,  /* bytes 2, 4, 6 are used for custom lookup */
	0xf8, 0x6b, 0x6c, 0x6d, 0x04, 0x6f, 0x70, 0x71,	 /* UDP payload byte 10 contains the packet index */

    0xC0, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, /* standard test vector: size 83 (0x53) CRC = 0xDA26) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, /* Message(UDP) length = 8 + 14 + 83 + 2 = 107 (0x6B) */
    0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0,
    0xB0, 0xC0, 0xD0, 0xE0, 0xF1, 0x01, 0x11, 0x21, /* CRC payload offset = 22 from UDP header (start offset) */
    0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1, /* CRC payload size = 83 */
    0xB1, 0xC1, 0xD1, 0xE1, 0xF2, 0x02, 0x12, 0x22,
    0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92, 0xA2,
    0xB2, 0xC2, 0xD2, 0xE2, 0xF3, 0x03, 0x13, 0x23,
    0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3,
    0xB3, 0xC3, 0xD3, 0xE3, 0xF4, 0x04, 0x14, 0x24,
    0x34, 0x44, 0x50, 0xDA, 0x26};


/* Packet 5 should match custom entry 1. This packet tests MAC to custom switch/custom LUT1 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(141,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_CUSTOM),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f6d)	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,34),		/* cmd len = 20, start offset = 34 (Payload) */
	TF_FORM_PKT_INFO_WORD1(141,0,1,0,0,0),  /* end offset = 141, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_CUSTOM),PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f6d)	/* Pseudo header checksum */
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5, ".testPkts")
#endif
static uint8_t pkt5[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x12, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x7e, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,  /* IP dest = 200.201.202.104 */
	0xca, 0x68, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6b,
	0x00, 0x00, 0x64, 0x65, 0x1a, 0x00, 0xa1, 0x00,  /* bytes 2, 4, 6 are used for custom lookup */
	0xf8, 0x6b, 0x6c, 0x6d, 0x05, 0x6f, 0x70, 0x71,	 /* UDP payload byte 10 contains the packet index */

    0xC0, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, /* standard test vector: size 83 (0x53) CRC = 0xDA26) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, /* Message(UDP) length = 8 + 14 + 83 + 2 = 107 (0x6B) */
    0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0,
    0xB0, 0xC0, 0xD0, 0xE0, 0xF1, 0x01, 0x11, 0x21, /* CRC payload offset = 22 from UDP header (start offset) */
    0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1, /* CRC payload size = 83 */
    0xB1, 0xC1, 0xD1, 0xE1, 0xF2, 0x02, 0x12, 0x22, /* CRC offset = 105 from UDP header (start offset) */
    0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92, 0xA2,
    0xB2, 0xC2, 0xD2, 0xE2, 0xF3, 0x03, 0x13, 0x23,
    0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3,
    0xB3, 0xC3, 0xD3, 0xE3, 0xF4, 0x04, 0x14, 0x24,
    0x34, 0x44, 0x50, 0xDA, 0x26};

/* Packet 6 should match MAC0/IP3. This packet tests MAC/IP to custom switch/custom LUT2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt6Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(141,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC |  PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_CUSTOM),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5((unsigned int)0x9f6d)	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(141,0,1,0,0,0),  /* end offset = 141, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_CUSTOM),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4((unsigned int)0x9f82)	/* Pseudo header checksum */
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6, ".testPkts")
#endif
static uint8_t pkt6[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x69, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,  /* IP dest = 200.201.202.103 */
	0xca, 0x67, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6b,
	0x00, 0x00, 0x3c, 0x3d, 0xfa, 0x3f, 0xaf, 0x41,  /* Custom match info */
	0xf8, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,

    0xC0, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20,
    0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0,
    0xB0, 0xC0, 0xD0, 0xE0, 0xF1, 0x01, 0x11, 0x21,
    0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1,
    0xB1, 0xC1, 0xD1, 0xE1, 0xF2, 0x02, 0x12, 0x22,
    0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92, 0xA2,
    0xB2, 0xC2, 0xD2, 0xE2, 0xF3, 0x03, 0x13, 0x23,
    0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3,
    0xB3, 0xC3, 0xD3, 0xE3, 0xF4, 0x04, 0x14, 0x24,
    0x34, 0x44, 0x50, 0xDA, 0x26};


#ifdef _TMS320C6X
#pragma DATA_SECTION (t7PktInfo, ".testPkts")
#endif
static pktTestInfo_t t7PktInfo[] =  {
	/* Packet 0 */
	{
		(uint8_t *)pkt0,
		(pasahoLongInfo_t *)&pkt0Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_CUSTOM) }, 								  	/* C2 match */
		T7_SWINFO0_CUSTOM_C2_MATCH + 0		/* Expected swinfo0 to find with the packet */
	},

	/* Packet 1 */
	{
		(uint8_t *)pkt1,
		(pasahoLongInfo_t *)&pkt1Info,
		sizeof(pkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T7_SWINFO0_UDP_MATCH + 1		/* Expected swinfo0 to find with the packet */
	},

	/* Packet 2 */
	{
		(uint8_t *)pkt2,
		(pasahoLongInfo_t *)&pkt2Info,
		sizeof(pkt2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_CUSTOM) }, 								  	/* C2 match */
		T7_SWINFO0_CUSTOM_C2_MATCH + 1		/* Expected swinfo0 to find with the packet */
	},


	/* Packet 3 */
	{
		(uint8_t *)pkt3,
		(pasahoLongInfo_t *)&pkt3Info,
		sizeof(pkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_CUSTOM) },								  	/* C2 lookup */	                            /* Fail Match */
		T7_SWINFO0_IP_NROUTE_FAIL + 1		/* Expected swinfo0 to find with the packet */
	},

	/* Packet 4 */
	{
		(uint8_t *)pkt4,
		(pasahoLongInfo_t *)&pkt4Info,
		sizeof(pkt4),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									    /* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_CUSTOM),  	/* C1 match */
		  0 },																	  		                                /* None */
		T7_SWINFO0_CUSTOM_C1_MATCH + 0		/* Expected swinfo0 to find with the packet */
	},

	/* Packet 5 */
	{
		(uint8_t *)pkt5,
		(pasahoLongInfo_t *)&pkt5Info,
		sizeof(pkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									    /* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_CUSTOM),  	/* C1 Match */
		  0 },																	  	                                    /* None */
		T7_SWINFO0_CUSTOM_C1_MATCH + 1		/* Expected swinfo0 to find with the packet */
	},

	/* Packet 6 */
	{
		(uint8_t *)pkt6,
		(pasahoLongInfo_t *)&pkt6Info,
		sizeof(pkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_CUSTOM) },								  	/* C2 lookup */	                            /* Fail Match */
		T7_SWINFO0_CUSTOM_C2_MATCH + 3		/* Expected swinfo0 to find with the packet */
	}

};




#endif /*TEST7PKTS_H_*/
