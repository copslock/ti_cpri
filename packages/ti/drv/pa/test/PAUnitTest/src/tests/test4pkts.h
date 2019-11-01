/*
 *
 * Copyright (C) 2010-2014 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef TEST4PKTS_H_
#define TEST4PKTS_H_

/* Valid rx MAC addresses used during the test */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t4EthInfo, ".testPkts")
#endif
static paEthInfo_t t4EthInfo[] =  {

    {
    	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
    },

    {
    	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09 },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
    },

    /* Pa_addMac2() entries */
    {
    	{ 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x04 },      /* Source mac */
   	 	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Dest mac is dont care */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
    },

};



/* Outer IP configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t4OuterIpInfo, ".testPkts")
#endif
static t4IpPaSetup_t t4OuterIpInfo[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		0,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		1,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 1 ----------- */
	{
		1,			/* Sequential ID */
		1,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 2 ----------- */
		/* Route on Dest IP only */
	{
		2,			/* Sequential ID */
		2,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		1,			/* Route Index - route to LUT2 (used to check nextFail route for unsupported protocol */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},	/* IP Destination address */
			0,	        /* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 3 ----------- */
		/* Route based on source GRE protocol only */
	{
        3,		/* Sequential ID */
		3,			/* Local Handle Index */
        pa_LUT1_INST_1,          /* LUT1 Instance (LUT1_1) */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0x0800,		/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 4 ----------- */
		/* Route on source and dest IP (more specific then Entry 2) */
	{
		4,			/* Sequential ID */
		4,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0x44440000,	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0x32,		/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 5 ----------- */
		/* Route on source IP, dest IP, protocol = UDP(more specific then Entry 2, 3) */
	{
		5,			/* Sequential ID */
		5,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		2,			/* Route Index - continue L3 (LUT1) parse */
		{
			{{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			17,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 6 ----------- */
		/* Route on source IP, dest IP, protocol = TCP(more specific then Entry 2, 3) */
	{
		6,			/* Sequential ID */
		6,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 7 ----------- */
		/* Route on source IP, dest IP, protocol = UDP, TOS (more specific than Entry 2, 3, 5) */
	{
		7,			/* Sequential ID */
		7,			/* Local Handle Index */
        pa_LUT1_INST_1,          /* LUT1 Instance (LUT1_1) */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			17,			/* Protocol */
			0x20,		/* TOS */
			1,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

#ifdef PASS_LUT1_ORDER_MATTER   /* No longer true for NSS_GEN2 device */

		/* ------- Entry 8 ----------- */
		/* Route on source IP, more general than Entry 3, 5, should be rejected */
	{
		8,							/* Sequential ID */
		T4_NUM_LOCAL_L3_HANDLES,	/* Local Handle Index - essentially a discard */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT,		/* PA LLD return value expected */
		FALSE												/* Set to true when the command is acked */
	},


			/* ------- Entry 9 ----------- */
		/* Same as entry 7, should be replaced or rejected */
        /* Note: Use the original Sequential ID since error is no longer expected */
	{
		7,									/* Sequential ID */
		7,			                        /* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,          /* LUT1 Instance */
		0,									/* L2 Handle Index */
		0,									/* Route Index - route to host */
		{
			{{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			17,			/* Protocol */
			0x20,		/* TOS */
			1,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_DUP_ENTRY,		/* PA LLD return value expected */
		FALSE				/* Set to true when the command is acked */
	},

#endif

		/* ------- Entry 10 ----------- */
		/* Route based on source IP only */
	{
		10,			/* Sequential ID */
		8,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		2,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 81, 82, 83, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 11 ----------- */
            /* The same IP address as Entry 1, but with GRE Protocol */
      {
		11,			/* Sequential ID */
        9,      /* Local Handle Index */
            pa_LUT1_INST_1, /* LUT1 Instance */
        0,      /* L2 Handle Index */
        0,      /* Route Index - route to host */
        {
          {{ 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
          {{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
          0,      /* SPI */
          0,      /* Flow */
          pa_IPV4,  /* IP Type */
          0x0800,   /* GRE Protocol */
          0,      /* Protocol */
          0,      /* TOS */
          0,      /* TOS Care flag */
                0           /* SCTP port */
        },
        pa_OK,    /* PA LLD return value expected */
        FALSE,    /* Set to true when the command is acked */
        FALSE   /* Set to true when tunnel using virtual link */
      },


				/* ------- Entry 12 ----------- */
		/* Route based on SPI only */
	{
		12,			/* Sequential ID */
		10,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		2,			/* L2 Handle Index */
		6,			/* Route Index - route to host */
		{
			{{ 81, 82, 83, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0      ,	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},


#ifndef NSS_GEN2
				/* ------- Entry 13 ----------- */
		/* IPv6 route on dest only */
	{
		13,			/* Sequential ID */
		11,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 14 ----------- */
		/* IPv6 route on dest, source only */
	{
		14,			/* Sequential ID */
		12,			/* Local Handle Index */
        pa_LUT1_INST_1,          /* LUT1 Instance (LUT1_1) */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},


#else

		/* ------- Entry 13 ----------- */
		/* IPv4 catchup rule */
	{
		13,			/* Sequential ID */
		11,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		-1,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 14 ----------- */
		/* IPv6 catchup rule */
	{
		14,			/* Sequential ID */
		12,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,    /* LUT1 Instance (LUT1_1) */
		-1,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

#endif
				/* ------- Entry 15 ----------- */
		/* IPv6 route on dest, source, flow */
	{
		15,			/* Sequential ID */
		13,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,			/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 16 ----------- */
		/* IPv6 route on dest, source, flow, protocol */
	{
		16,			/* Sequential ID */
		14,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,			/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			17,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 17 ----------- */
		/* IPv6 route on dest, source, flow, protocol */
	{
		17,			/* Sequential ID */
		15,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,			/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol = IPv4 */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 18 ----------- */
		/* IPv6 route on dest, source, flow, protocol, gre protocol */
	{
		18,			/* Sequential ID */
		16,			/* Local Handle Index */
        pa_LUT1_INST_1,          /* LUT1 Instance (LUT1_1) */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,			/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0x0800,		/* GRE Protocol = IPv4 */
			47,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 19 ----------- */
		/* IPv6 route on dest, source, flow, protocol, spi (0x44445555) */
	{
		19,			/* Sequential ID */
		17,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,		    /* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			50,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 20 ----------- */
		/* IPv6 route on dest, source, flow, protocol, spi (0x44445555)*/
	{
		20,			/* Sequential ID */
		18,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,   		/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			51,			/* Protocol AH */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

#ifdef PASS_LUT1_ORDER_MATTER   /* No longer true */

				/* ------- Entry 21 ----------- */
		/* More general IPv6 then entry 20, should be rejected */
	{
		21,							/* Sequential ID */
		T4_NUM_LOCAL_L3_HANDLES,    /* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,  /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0x44445555,		/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT,		/* PA LLD return value expected */
		FALSE												/* Set to true when the command is acked */
	},


				/* ------- Entry 22 ----------- */
		/* Identical to entry 20, should be rejected or replaced SPI = 0x44445555 */
        /* Note: Use the original Sequential ID since error is no longer expected */
	{
		20,							/* Sequential ID */
		18,	                        /* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,  /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 }},		/* IP Source address */
			{{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 }},		/* IP Destination address */
			0,  		/* SPI */
			0x98765,	/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			51,			/* Protocol AH */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_DUP_ENTRY,		/* PA LLD return value expected */
		FALSE				/* Set to true when the command is acked */
	},

#endif


				/* ------- Entry 23 ----------- */
		/* Route on Destination IP only. On match continue parse (used for nested IP) */
	{
		23,		/* Sequential ID */
		19,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		2,		/* Route Index - continue L3 parse */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

				/* ------- Entry 24 ----------- */
		/* Route on Destination IP only. On match continue parse (used for nested IP) */
	{
		24,		/* Sequential ID */
		20,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		2,		/* Route Index - continue L3 parse */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 25 ----------- */
		/* Route on Destination IP only */
	{
		25,		/* Sequential ID */
		21,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		2,		/* L2 Handle Index */
		1,		/* Route Index - Continue LUT2 */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},	/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 26 ----------- */
		/* Route on Destination IP only */
	{
		26,		/* Sequential ID */
		22,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 27 ----------- */
		/* Route on Destination IP only */
	{
		27,		/* Sequential ID */
		23,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 81, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 28 ----------- */
		/* Route on Destination IP only */
	{
		28,		/* Sequential ID */
		24,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		2,		/* L2 Handle Index */
		4,		/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 29 ----------- */
		/* Route on Destination IP only */
	{
		29,		/* Sequential ID */
		25,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		2,		/* L2 Handle Index */
		1,		/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 76, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 30 ----------- */
		/* Route on Destination IP only */
	{
		30,		/* Sequential ID */
		26,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 31 ----------- */
		/* Route on Destination IP only */
	{
		31,		/* Sequential ID */
		27,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 32 ----------- */
		/* Route on Destination IP only */
	{
		32,		/* Sequential ID */
		28,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 33 ----------- */
		/* Route on Destination IP only */
	{
		33,		/* Sequential ID */
		29,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 34 ----------- */
		/* Route on Destination IP only */
	{
		34,		/* Sequential ID */
		30,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 35 ----------- */
		/* Route on Destination IP only */
	{
		35,		/* Sequential ID */
		31,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 82, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 36 ----------- */
		/* Route on Destination IP only */
	{
		36,		/* Sequential ID */
		32,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 83, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 37 ----------- */
		/* Route on Destination IP only */
	{
		37,		/* Sequential ID */
		33,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 38 ----------- */
		/* Route on Destination IP only */
	{
		38,		/* Sequential ID */
		34,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},


					/* ------- Entry 39 ----------- */
		/* Route on Destination IP only */
	{
		39,		/* Sequential ID */
		35,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 86, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 40 ----------- */
		/* Route on Destination IP only */
	{
		40,		/* Sequential ID */
		36,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 87, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 41 ----------- */
		/* Route on Destination IP only */
	{
		41,		/* Sequential ID */
		37,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 88, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 42 ----------- */
		/* Route on Destination IP only */
	{
		42,		/* Sequential ID */
		38,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 43 ----------- */
		/* Route on Destination IP only */
	{
		43,		/* Sequential ID */
		39,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 44 ----------- */
		/* Route on Destination IP only */
	{
		44,		/* Sequential ID */
		40,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 91, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 45 ----------- */
		/* Route on Destination IP only */
	{
		45,		/* Sequential ID */
		41,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 92, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 46 ----------- */
		/* Route on Destination IP only */
	{
		46,		/* Sequential ID */
		42,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 47 ----------- */
		/* Route on Destination IP only */
	{
		47,		/* Sequential ID */
		43,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 48 ----------- */
		/* Route on Destination IP only */
	{
		48,		/* Sequential ID */
		44,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 95, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 49 ----------- */
		/* Route on Destination IP only */
	{
		49,		/* Sequential ID */
		45,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 50 ----------- */
		/* Route on Destination IP only */
	{
		50,		/* Sequential ID */
		46,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 51 ----------- */
		/* Route on Destination IP only */
	{
		51,		/* Sequential ID */
		47,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 98, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 52 ----------- */
		/* Route on Destination IP only */
	{
		52,		/* Sequential ID */
		48,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 99, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 53 ----------- */
		/* Route on Destination IP only */
	{
		53,		/* Sequential ID */
		49,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 54 ----------- */
		/* Route on Destination IP only */
	{
		54,		/* Sequential ID */
		50,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 55 ----------- */
		/* Route on Destination IP only */
	{
		55,		/* Sequential ID */
		51,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 56 ----------- */
		/* Route on Destination IP only */
	{
		56,		/* Sequential ID */
		52,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 57 ----------- */
		/* Route on Destination IP only */
	{
		57,		/* Sequential ID */
		53,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 58 ----------- */
		/* Route on Destination IP only */
	{
		58,		/* Sequential ID */
		54,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 59 ----------- */
		/* Route on Destination IP only */
	{
		59,		/* Sequential ID */
		55,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 60 ----------- */
		/* Route on Destination IP only */
	{
		60,		/* Sequential ID */
		56,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 61 ----------- */
		/* Route on Destination IP only */
	{
		61,		/* Sequential ID */
		57,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

#ifndef __LINUX_USER_SPACE

					/* ------- Entry 62 ----------- */
		/* Route on Destination IP only */
	{
		62,		/* Sequential ID */
		58,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 63 ----------- */
		/* Route on Destination IP only */
	{
		63,		/* Sequential ID */
		59,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

		/* ------- Entry 64 ----------- */
		/* Route on Destination IP only */
	{
		64,		/* Sequential ID */
		60,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

					/* ------- Entry 65 ----------- */
		/* Route on Destination IP only */
	{
		65,		/* Sequential ID */
		61,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 112, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},


					/* ------- Entry 66 ----------- */
		/* Route on Destination IP only */
	{
		66,		/* Sequential ID */
		62,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		2,		/* Route Index - continue L3 parse */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		TRUE		/* Set to true when tunnel using virtual link (0)*/
	},

#endif

					/* ------- Entry 67 ----------- */
		/* Route on Destination IP only */
	{
		67,		/* Sequential ID */
		63,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		2,		/* Route Index - continue L3 parse */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		TRUE		/* Set to true when tunnel using virtual link (0) */
	},


#ifdef PASS_LUT_LIMIT_TEST

		/* ------- Entry 68 ----------- */
		/* Route on Destination IP only - the hardware table is full */
	{
		68,							/* Sequential ID */
		T4_NUM_LOCAL_L3_HANDLES,	/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		/* IP Source address */
			{{ 71, 72, 73, 115, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_LUT_ENTRY_FAILED,		/* PA LLD return value expected */
		FALSE						/* Set to true when the command is acked */
	},

				/* ------- Entry 69 ----------- */
		/* Route on Destination IP only - Invalid LUT1 Instance */
	{
		69,							/* Sequential ID */
		T4_NUM_LOCAL_L3_HANDLES,	/* Local Handle Index */
        3,                          /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},		    /* IP Source address */
			{{ 71, 72, 73, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},		/* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol ESP */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_LUT_INST,		/* PA LLD return value expected */
		FALSE						/* Set to true when the command is acked */
	}

#endif

};

/* Test is applicable only for Gen 1 */
#ifndef NSS_GEN2
/* The following configurations link to an IP header (nested inner IP) */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t4ReplaceOuterIpInfo, ".testPkts")
#endif
static t4RepIpPaSetup_t t4ReplaceOuterIpInfo[] = {
   /* -------- Entry 0 --------------
    * replace ip entry that does not exist */
    {
      25, /* Reference IpSetup entry index */
      pa_INVALID_INPUT_HANDLE,
      T4_CMD_REPLACE_WITH_ERR_INDEX
    },
   /* -------- Entry 1 --------------
    * replace ip entry that does not exist */
    {
      28, /* Reference IpSetup entry index */
      pa_OK,
      T4_CMD_REPLACE_WITH_INDEX
    },
   /* -------- Entry 2 --------------
    * replace ip entry that does not exist */
    {
      29, /* Reference IpSetup entry index */
      pa_OK,
      T4_CMD_REPLACE_WITH_NO_INDEX
    }
};
#endif

/* The following configurations link to an IP header (nested inner IP) */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t4InnerIpInfo, ".testPkts")
#endif
static t4IpPaSetup_t t4InnerIpInfo[] =  {

	/* ------- Entry 0 -----------
	 * match IP dest only  */
	{
		0,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 0,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		19,												/* L3 Outer IP Handle Index */
                                                        /* Note: It is entry 23 */
		0,												/* Route Index - route to host */
		{
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  	 /* IP Source address */
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI  */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 1 -----------
	 * match IP dest and source */
	{
		1,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 1,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		19,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 2 -----------
	 * match IP dest, srouce, protocol, and sctp Port */
	{
		2,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 2,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		19,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			132, 		/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0xd75d      /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 3 -----------
	 * match IP dest, srouce, protocol, TOS */
	{
		3,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 3,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		19,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			17,			/* Protocol */
			0x20,		/* TOS */
			1,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 4 -----------
	 * match IP dest only, outer IP index = 24 */
	{
		4,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 4,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 20, 21, 22, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 5 -----------
	 * match IP dest, source , outer IP index = 24 */
	{
		5,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 5,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  20,  21,  22,  23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 6 -----------
	 * match SPI only, outer IP index = 5 */
	{
		6,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 6,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		5,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0x55550000,	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 7 -----------
	 * match IP dest, source, protocol , outer IP index = 24 */
	{
		7,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 7,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  20,  21,  22,  23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

    #if 0

	/* ------- Entry 8 -----------
	 * match source, more general then entry 5-7, should be rejected*/
	{
		8,									/* Sequential ID */
		T4_NUM_LOCAL_L3_HANDLES,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,          /* LUT1 Instance */
		24,									/* L3 Outer IP Handle Index */
		0,									/* Route Index - route to host */
		{
			{{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{   0,   0,   0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT,		/* PA LLD return value expected */
		FALSE												/* Set to true when the command is acked */
	},


	/* ------- Entry 9 -----------
	 * identical to entry 7, should be rejected or replaced */
    /* Note: Use the original Sequential ID since error is no longer expected */
	{
		7,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 7,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  20,  21,  22,  23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_INVALID_DUP_ENTRY,		/* PA LLD return value expected */
		FALSE				/* Set to true when the command is acked */
	},

    #endif


	/* ------- Entry 10 -----------
	 * match IP dest outer IP index = 23 */
	{
		10,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 8,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 11 -----------
	 * match IP dest outer IP index = 23 */
	{
		11,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 9,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 12 -----------
	 * match IP dest outer IP index = 23 */
	{
		12,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 10,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 13 -----------
	 * match IP dest outer IP index = 23 */
	{
		13,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 11,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 14 -----------
	 * match IP dest outer IP index = 23 */
	{
		14,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 12,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 15 -----------
	 * match IP dest outer IP index = 23 */
	{
		15,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 13,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 16 -----------
	 * match IP dest outer IP index = 23 */
	{
		16,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 14,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 17 -----------
	 * match IP dest outer IP index = 23 */
	{
		17,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 15,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 18 -----------
	 * match IP dest outer IP index = 23 */
	{
		18,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 16,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 19 -----------
	 * match IP dest outer IP index = 23 */
	{
		19,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 17,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 20 -----------
	 * match IP dest outer IP index = 23 */
	{
		20,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 18,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 21 -----------
	 * match IP dest outer IP index = 23 */
	{
		21,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 19,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 22 -----------
	 * match IP dest outer IP index = 23 */
	{
		22,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 20,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 23 -----------
	 * match IP dest outer IP index = 23 */
	{
		23,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 21,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 24 -----------
	 * match IP dest outer IP index = 23 */
	{
		24,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 22,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 25 -----------
	 * match IP dest outer IP index = 23 */
	{
		25,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 23,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 26 -----------
	 * match IP dest outer IP index = 23 */
	{
		26,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 24,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 27 -----------
	 * match IP dest outer IP index = 23 */
	{
		27,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 25,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 28 -----------
	 * match IP dest outer IP index = 23 */
	{
		28,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 26,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 29 -----------
	 * match IP dest outer IP index = 23 */
	{
		29,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 27,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 30 -----------
	 * match IP dest outer IP index = 23 */
	{
		30,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 28,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 31 -----------
	 * match IP dest outer IP index = 23 */
	{
		31,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 29,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 41, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 32 -----------
	 * match IP dest outer IP index = 23 */
	{
		32,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 30,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 33 -----------
	 * match IP dest outer IP index = 23 */
	{
		33,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 31,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 34 -----------
	 * match IP dest outer IP index = 23 */
	{
		34,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 32,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 44, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 35 -----------
	 * match IP dest outer IP index = 23 */
	{
		35,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 33,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 36 -----------
	 * match IP dest outer IP index = 23 */
	{
		36,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 34,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 46, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 37 -----------
	 * match IP dest outer IP index = 23 */
	{
		37,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 35,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 47, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 38 -----------
	 * match IP dest outer IP index = 23 */
	{
		38,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 36,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 39 -----------
	 * match IP dest outer IP index = 23 */
	{
		39,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 37,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 49, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 40 -----------
	 * match IP dest outer IP index = 23 */
	{
		40,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 38,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 41 -----------
	 * match IP dest outer IP index = 23 */
	{
		41,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 39,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 42 -----------
	 * match IP dest outer IP index = 23 */
	{
		42,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 40,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 43 -----------
	 * match IP dest outer IP index = 23 */
	{
		43,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 41,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 53, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 44 -----------
	 * match IP dest outer IP index = 23 */
	{
		44,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 42,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 54, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 45 -----------
	 * match IP dest outer IP index = 23 */
	{
		45,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 43,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 46 -----------
	 * match IP dest outer IP index = 23 */
	{
		46,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 44,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 47 -----------
	 * match IP dest outer IP index = 23 */
	{
		47,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 45,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 48 -----------
	 * match IP dest outer IP index = 23 */
	{
		48,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 46,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 58, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 49 -----------
	 * match IP dest outer IP index = 23 */
	{
		49,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 47,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 59, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 50 -----------
	 * match IP dest outer IP index = 23 */
	{
		50,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 48,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},


	/* ------- Entry 51 -----------
	 * match IP dest outer IP index = 23 */
	{
		51,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 49,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 61, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 52 -----------
	 * match IP dest outer IP index = 23 */
	{
		52,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 50,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 53 -----------
	 * match IP dest outer IP index = 23 */
	{
		53,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 51,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 54 -----------
	 * match IP dest outer IP index = 23 */
	{
		54,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 52,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 55 -----------
	 * match IP dest outer IP index = 23 */
	{
		55,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 53,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 56 -----------
	 * match IP dest outer IP index = 23 */
	{
		56,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 54,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 57 -----------
	 * match IP dest outer IP index = 23 */
	{
		57,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 55,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 67, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 58 -----------
	 * match IP dest outer IP index = 23 */
	{
		58,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 56,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 59 -----------
	 * match IP dest outer IP index = 23 */
	{
		59,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 57,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 60 -----------
	 * match IP dest outer IP index = 23 */
	{
		60,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 58,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 61 -----------
	 * match IP dest outer IP index = 23 */
	{
		61,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 59,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			6,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 62 -----------  */
    /* link to virtual link 0        */
	{
		62,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 60,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance */
		0,												/* Virtual Link Handle Index */
		5,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		TRUE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 63 -----------  */
    /* link to virtual link 0        */
	{
		63,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + 61,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		0,												/* Virtual Link Handle Index */
		0,												/* Route Index - route to host */
		{
			{{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{  25, 26, 27, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			17,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		TRUE		/* Set to true when tunnel using virtual link */
	}



};


/* The following configurations link to an IP header which is also nested (innermost of 3 IP headers) */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t4InnerInnerIpInfo, ".testPkts")
#endif
static t4IpPaSetup_t t4InnerInnerIpInfo[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_AND_INNER1_IP_HANDLE + 0,	/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		0,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  	 /* IP Source address */
			{{ 40, 41, 42, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	},

	/* ------- Entry 1 ----------- */
	{
		1,												/* Sequential ID */
		T4_NUM_LOCAL_L3_OUTER_AND_INNER1_IP_HANDLE + 1,	/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		0,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{{ 40, 41, 42, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 40, 41, 42, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,		/* Set to true when the command is acked */
		FALSE		/* Set to true when tunnel using virtual link */
	}
};


/* packet 0
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif

/* packet 1
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 2
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info 23)
 * inner ip dest = 10.11.12.13 (inner IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match inner IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x46, 0x45, 0x00, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x93, 0x85, 0x9e, 0xda,
	0x6d, 0x0a, 0x0a, 0x0b, 0x0c, 0x0d, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0xeb, 0xc7, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt2Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,54),		/* cmd len = 20, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,1,0,0,0),  /* end offset = 142 */
	TF_FORM_PKT_INFO_WORD2(34,0,0,0),		/* L3 offset = 34, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 2, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 3
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info 23)
 * inner ip dest = 10.11.12.13 (inner IP info 2)
 * inner ip src  = 10.11.12.13
 * inner ip protocol = 132 (0x84) (SCTP)
 * Designed to match inner IP configuration 2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3, ".testPkts")
#endif
static uint8_t pkt3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x23, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x46, 0x45, 0x00, 0x00, 0x50, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x84, 0x88, 0xfb, 0x0a, 0x0b,
	0x0c, 0x0d, 0x0a, 0x0b, 0x0c, 0x0d,
    0x8e, 0x3d, 0xd7, 0x5d, 0x45, 0x23, 0x05, 0x4f, /* SCTP source port, dest port and tag */
    0x5f, 0x12, 0x9f, 0xa6, 0x05, 0x00, 0x00, 0x30, /* CRC-32C reference 0x5f129fa6 */
    0x00, 0x01, 0x00, 0x2c, 0x02, 0x00, 0x8e, 0x3d,
    0xc0, 0xa8, 0x01, 0x0d, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x40, 0x43, 0x22, 0xdf, 0x56, 0x01, 0x00, 0x00,
    0x40, 0x69, 0x57, 0x00, 0xb6, 0x51, 0xe7, 0xc2,
    0xcc, 0x82, 0x90, 0xf9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,66),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 66 (SCTP Header) */
	TF_FORM_PKT_INFO_WORD1(114,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 114, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,54,0,0),    	            /* L3 offset = 34, l4Offset = 54, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_SCTP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t pkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,66),		/* cmd len = 20, start offset = 66 (SCTP payload) */
	TF_FORM_PKT_INFO_WORD1(114,0,1,0,0,0),  /* end offset = 114 */
	TF_FORM_PKT_INFO_WORD2(34,54,0,0),		/* L3 offset = 34, l4Offset = 54, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 2, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UNKNOWN,0,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 4
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * PPPoE session header (verLen = 0x11, code = 0,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0021 (IPv4)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4, ".testPkts")
#endif
static uint8_t pkt4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
    0x12, 0x34, 0x00, 0x6e, 0x00, 0x21, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UDP),     /* end offset = 114, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,42,0,0),    	            /* L3 offset = 34, l4Offset = 42, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_PPPoE),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(130,0,1,0,0,0),  /* end offset = 130 */
	TF_FORM_PKT_INFO_WORD2(22,0,0,0),		/* L3 offset = 22, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,PASAHO_HDR_BITMASK2_PPPoE,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 5
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 2)
 * out ip dest = 71.72.73.74
 * fake ICMP packet
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5, ".testPkts")
#endif
static uint8_t pkt5[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0x47, 0x48,
	0x49, 0x4a, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (ICMP) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (ICMP) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 6
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 4)
 * out ip dest = 71.72.73.74
 * fake IPSEC ESP packet
 * Designed to match outer IP configuration 4 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6, ".testPkts")
#endif
static uint8_t pkt6[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x32,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0x47, 0x48,
	0x49, 0x4a, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00,
	0x00, 0x01, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (ESP) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,34),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_ESP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (ESP) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset */
	TF_FORM_PKT_INFO_WORD2(14,0,0,34),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 2, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_ESP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 7
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.114 (out IP info 67)
 * inner ip dest = 25.26.27.73 (inner IP info 63)
 * UDP (destination port = 0x0555)
 * Designed to match virtual link inner IP configuration 63
 * data[18] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7, ".testPkts")
#endif
static uint8_t pkt7[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x80,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x72, 0x45, 0x40, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x93, 0x85, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x49, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0xeb, 0xc7, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(34,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,54),		/* cmd len = 24, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,1,0,0,0),  /* end offset = 142 */
	TF_FORM_PKT_INFO_WORD2(34,0,0,0),		/* L3 offset = 34, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 2, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


/* packet 8
 * mac src = 0e:a0:01:02:03:04  (MAC Info 2)
 * out ip src = 81.82.83.1      (out IP info 10)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 8
 * data[18] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8, ".testPkts")
#endif
static uint8_t pkt8[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x51, 0x52, 0x53, 0x01, 0xe8, 0xe9,
	0xea, 0xeb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 122, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 9
 * mac src = 0e:a0:01:02:03:04  (MAC Info 2)
 * out ip src = 81.82.83.1      (out IP info 10)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 8
 * IP fragments
 * data[18] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9, ".testPkts")
#endif
static uint8_t pkt9[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x30,
	0x00, 0x6c, 0x02, 0x00, 0x20, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x51, 0x52, 0x53, 0x01, 0xe8, 0xe9,
	0xea, 0xeb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 122, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t pkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 10
 * Purpose: Interface-based routing
 * mac src = 0e:a0:01:02:03:04  (MAC Info 2)
 * out ip src = 71.72.73.75     (out IP info 28)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 24
 * data[18] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10, ".testPkts")
#endif
static uint8_t pkt10[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x02, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0xde, 0xad, 0xbe, 0xef, 0x47, 0x48,
	0x49, 0x4b, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 11
 * Purpose: Interface-based routing at LUT2 per nextFail route
 * mac src = 0e:a0:01:02:03:04  (MAC Info 2)
 * out ip src = 71.72.73.76     (out IP info 29)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 25
 * data[18] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11, ".testPkts")
#endif
static uint8_t pkt11[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x02, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0xba, 0xbe, 0xfa, 0xce, 0x47, 0x48,
	0x49, 0x4c, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* IP Fragments
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt12, ".testPkts")
#endif
static uint8_t pkt12[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x20, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt12Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,1,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FRAG,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,00,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FRAG,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 13
 * EQoS DP-bit without override, tagged IP over IP packet
 * VLAN Pbit = 0x6 ==> offset = 2
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.114 (out IP info 67)
 * inner ip dest = 25.26.27.72 (inner IP info 62)
 * UDP (destination port = 0x0555)
 * Designed to match virtual link inner IP configuration 62
 * data[last-1] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt13, ".testPkts")
#endif
static uint8_t pkt13[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x00, 0x81, 0x00, 0xC9, 0x99,
    0x08, 0x00, 0x45, 0x80,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x72, 0x45, 0x40, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x93, 0x85, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x48, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0xeb, 0xc7, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x00,
    0xba, 0xbe, 0xfa, 0xce,                             /* fake CRC to be removed */
    };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt13Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,58),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(146,0,0,PASAHO_HDR_UDP),     /* end offset = 146, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(38,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,58),		/* cmd len = 24, start offset = 58 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(146,0,1,0,0,0),  /* end offset*/
	TF_FORM_PKT_INFO_WORD2(38,0,0,0),		/* L3 offset,  l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 1, ip count = 2, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,1,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 14
 * EQoS DP-bit without override, untagged IP over IP packet
 * Inner IP DSCP = 0x22 ==> offset = 12
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.114 (out IP info 67)
 * inner ip dest = 25.26.27.72 (inner IP info 62)
 * UDP (destination port = 0x0555)
 * Designed to match virtual link inner IP configuration 62
 * data[last-1] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt14, ".testPkts")
#endif
static uint8_t pkt14[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x01, 0x08, 0x00, 0x45, 0x80,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x72, 0x45, 0x88, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x93, 0x85, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x48, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0xeb, 0xc7, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x01,
    0xba, 0xbe, 0xfa, 0xce,                             /* fake CRC to be removed */
    };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt14Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt14Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(34,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt14Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,54),		/* cmd len = 24, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,1,0,0,0),  /* end offset = 142 */
	TF_FORM_PKT_INFO_WORD2(34,0,0,0),		/* L3 offset = 34, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 2, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 15
 * EQoS DP-bit with override set, untagged IP packet
 * Default route of EMAC 1 2 ==> offset = 3
 * mac src = 0e:a0:01:02:03:04  (MAC Info 2)
 * out ip src = 81.82.83.2      (out IP info 12)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 10
 * data[last-1] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt15, ".testPkts")
#endif
static uint8_t pkt15[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x51, 0x52, 0x53, 0x02, 0xe8, 0xe9,
	0xea, 0xeb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0x00,
    0xba, 0xbe, 0xfa, 0xce,                             /* fake CRC to be removed */
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt15Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt15Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 122, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt15Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 16
 * EQoS DSCP mode, tagged IP packet
 * DSCP 0x04 ==> offset = 11
 * mac src = 0e:a0:01:02:03:04  (MAC Info 2)
 * out ip dst = 71.72.73.72     (out IP info 25)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 21
 * data[last-1] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt16, ".testPkts")
#endif
static uint8_t pkt16[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x81, 0x00, 0x0a, 0xaa,
    0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x17, 0x18, 0x19, 0x80, 0x47, 0x48,
	0x49, 0x48, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0x00,
    0xba, 0xbe, 0xfa, 0xce,                             /* fake CRC to be removed */
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt16Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt16Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,46),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Payload) */
	TF_FORM_PKT_INFO_WORD1(126,0,0,PASAHO_HDR_UNKNOWN), /* end offset errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(18,38,46,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 1),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt16Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,46),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(126,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),		/* L3 offset, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,1,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 17
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 3)
 * out ip dest = 200.201.202.203
 * GRE
 * inner ip src  = 110.111.112.113 (out IP info 3)
 * inner ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt17, ".testPkts")
#endif
static uint8_t pkt17[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x05, 0x2f,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0xB0, 0x00, 0x08, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x45, 0x00, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11,	0x43, 0x0c, 0x6e, 0x6f,
	0x70, 0x71, 0xc8, 0xc9,	0xca, 0xcb, 0x12, 0x34,
	0x05, 0x55, 0x00, 0x58,	0xc0, 0x0b, 0x2d, 0xcf,
	0x46, 0x29, 0x04, 0xb4,	0x78, 0xd8, 0x68, 0xa7,
	0xff, 0x3f, 0x2b, 0xf1,	0xfc, 0xd9, 0x7a, 0x96,
	0x09, 0x2c, 0xa5, 0x57,	0x74, 0x64, 0xc4, 0xaf,
	0x15, 0x28, 0xa4, 0xe9,	0x57, 0xdb, 0x5e, 0x20,
	0xfb, 0x38, 0xa8, 0x4e,	0xa6, 0x14, 0x93, 0x25,
	0x56, 0x24, 0x44, 0xdf,	0x59, 0x8d, 0x43, 0x7b,
	0xbe, 0x90, 0x16, 0x89,	0x9d, 0x7e, 0x77, 0xc6,
	0x2f, 0x26, 0x98, 0x88,	0xf5, 0xb4, 0x30, 0xd4,
	0x34, 0x9d, 0x3a, 0x0d,	0x0f, 0xbd, 0x2f, 0xa1,
	0xf7, 0x0f, 0xd9, 0x68,	0xf4, 0xd9
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt17Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt17Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 50 (Inner IP Header) */
	TF_FORM_PKT_INFO_WORD1(146,0,0,PASAHO_HDR_IPv4),     /* end offset = 106, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 1, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt17Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (Inner IP Header) */
	TF_FORM_PKT_INFO_WORD1(146,0,1,0,0,0),  /* end offset = 146 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,1,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


#ifdef NSS_GEN2
/* packet 20
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 5)
 * out ip dest = 71.72.73.74
 * fake IPSEC ESP NAT-T packet
 * Designed to match inner IP configuration (SPI) 6 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt20, ".testPkts")
#endif
static uint8_t pkt20[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x74, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0x47, 0x48,
	0x49, 0x4a, 0xaa, 0xbb, 0x11, 0x94, 0x00, 0x60,
    0x00, 0x00, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00,
	0x00, 0x01, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt20Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt20Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 50 (ESP) */
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 130, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,42),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

#endif

#ifdef NSS_GEN2

/* packet 30
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 1.1.1.2 (Handle 11: all matches)
 * UDP (destination port = 0xabcd)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt30, ".testPkts")
#endif
static uint8_t pkt30[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x01, 0x01, 0x02, 0x01, 0x01,
	0x01, 0x02, 0xab, 0xcd, 0xab, 0xcd, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt30Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt30Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt30Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif

/* packet 31
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest (Handle 11: all matches)
 * UDP (destination port = 0xabcd)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt31, ".testPkts")
#endif
static uint8_t pkt31[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x86, 0xdd,
	0x60, 0x01, 0x01, 0x01,             // Iv6 header
	0x00, 0x58, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,
	0xab, 0xcd, 0xab, 0xcd, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt31Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt31Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt31Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,54),		/* cmd len = 24, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,1,0,0,0),  /* end offset */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif
#endif

/* packet 40
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * Illegal IPv4 packet with protocol = 0
 * out ip dest = 200.201.202.203 (out IP info 0)
 * Designed to verify illegal packet processing */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt40, ".testPkts")
#endif
static uint8_t pkt40[526] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00,
	0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
	0x07, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt40Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt40Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(526,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 000, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t pkt40Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(526,0,1,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif

/* Bits 14 and 15 are used to tell where the packet is expected to wind up */
#define T4_PACKET_L3_MATCH_VALID	(0 << 14) 		/* Packet will match (handle index in bits 13:0) */
#define T4_PACKET_NFAIL			    (1 << 14)     	/* Packet will arrive in the next fail queue  */
#define T4_PACKET_DISCARD			(2 << 14)		/* Packet is discarded by PA */
#define T4_PACKET_DEST_MASK		    (3 << 14)
#define T4_PACKET_INDEX_MASK		0x3fff

#ifdef _TMS320C6X
#pragma DATA_SECTION (t4PktInfo, ".testPkts")
#endif
static pktTestInfo_t t4PktInfo[] =  {
	/* Packet 40 */
	{
		(uint8_t *)pkt40,
		(pasahoLongInfo_t *)&pkt40Info,
		sizeof(pkt40),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
          #ifndef NSS_GEN2
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_SILENT_DISCARD) | (1 << TF_STATS_BM_C1_NUM_IPV4), /* IP match */
          #else
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_SILENT_DISCARD),                                 /* IP match */
          #endif
		  0 },																	  									/* no other match */
		T4_PACKET_DISCARD		            /* Packet will be discard */
	},
	/* Packet 0 */
	{
		(uint8_t *)pkt0,
		(pasahoLongInfo_t *)&pkt0Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  0 },																	  									/* no other match */
		T4_PACKET_L3_MATCH_VALID | 0		/* Packet will be matched by handle index 0 */
	},

#ifdef NSS_GEN2
    /* catchup rule tests */
	/* Packet 30 */
	{
		(uint8_t *)pkt30,
		(pasahoLongInfo_t *)&pkt30Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  0 },																	  									/* no other match */
		T4_PACKET_L3_MATCH_VALID | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 31 */
	{
		(uint8_t *)pkt31,
		(pasahoLongInfo_t *)&pkt31Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		  0 },																	  									/* no other match */
		T4_PACKET_L3_MATCH_VALID | 0		/* Packet will be matched by handle index 0 */
	},
#endif

	/* Packet 1 */
	{
		(uint8_t *)pkt1,
		(pasahoLongInfo_t *)&pkt1Info,
		sizeof(pkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
		  0 },
		 T4_PACKET_L3_MATCH_VALID | 1		/* Packet will be matched by handle index 1 */
	},

	/* Packet 17 */
	{
		(uint8_t *)pkt17,
		(pasahoLongInfo_t *)&pkt17Info,
		sizeof(pkt17),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		  0 },
		 T4_PACKET_L3_MATCH_VALID | 1		/* Packet will be matched by handle index 1 */
	},

	/* Packet 4 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt4,
		(pasahoLongInfo_t *)&pkt4Info,
		sizeof(pkt4),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
		  0 },
		 T4_PACKET_L3_MATCH_VALID | 1		/* Packet will be matched by handle index 1 */
	},

	/* Packet 5 */
    /* fake ICMP packet which shall fail at the next stage after Outer IP */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt5,
		(pasahoLongInfo_t *)&pkt5Info,
		sizeof(pkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
          #ifdef NSS_GEN2
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH)
          #else
          (1 << TF_STATS_BM_C2_NUM_PACKETS)
          #endif
          },
		 T4_PACKET_L3_MATCH_VALID | 2		/* Packet will be matched by handle index 2 */
	},

	/* Packet 6 */
    /* SPI packet: NSS_GEN2: Two matches
                   PASS_GEN1: Single match */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt6,
		(pasahoLongInfo_t *)&pkt6Info,
		sizeof(pkt6),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
          #if NSS_GEN2
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH)
          #endif
          },                                   /* SPI match */
		 T4_PACKET_L3_MATCH_VALID | 4		/* Packet will be matched by handle index 4 */
	},

    #ifdef NSS_GEN2
	/* Packet 20 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt20,
		(pasahoLongInfo_t *)&pkt20Info,
		sizeof(pkt20),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH)},                                   /* SPI match */
		  (T4_PACKET_L3_MATCH_VALID | T4_NUM_LOCAL_L3_OUTER_IP_HANDLES) + 6		/* Packet will match inner IP entry 5 */
	},
    #endif

	/* Packet 2 */
	{
		(uint8_t *)pkt2,
		(pasahoLongInfo_t *)&pkt2Info,
		sizeof(pkt2),
		{ ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),										/* MAC match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* Outer IP match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) },  	/* inner IP match */
		  (T4_PACKET_L3_MATCH_VALID | T4_NUM_LOCAL_L3_OUTER_IP_HANDLES) + 0  /* Packet will match inner IP entry 0 */
	},

	/* Packet 3 */
	{
		(uint8_t *)pkt3,
		(pasahoLongInfo_t *)&pkt3Info,
		sizeof(pkt3),
		{ ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),										/* MAC match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* Outer IP match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) },  	/* inner IP match */
		  ( T4_PACKET_L3_MATCH_VALID | T4_NUM_LOCAL_L3_OUTER_IP_HANDLES ) + 2    /* Packet will match inner IP entry 0 */
	},


};

#ifdef _TMS320C6X
#pragma DATA_SECTION (t4QoSPktInfo, ".testPkts")
#endif
static pktTestInfo_t t4QoSPktInfo[] =  {

	/* Packet 7 */
	{
		(uint8_t *)pkt7,
		(pasahoLongInfo_t *)&pkt7Info,
		sizeof(pkt7),
		{ ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),										/* MAC match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* Outer IP match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) },  	/* inner IP match */
		T4_QoS_PKT_INDEX
	},

	/* Packet 8 */
	{
		(uint8_t *)pkt8,
		(pasahoLongInfo_t *)&pkt8Info,
		sizeof(pkt8),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  0 },																	  									/* no other match */
		T4_QoS_PKT_INDEX
	},

	/* Packet 9 */
	{
		(uint8_t *)pkt9,
		(pasahoLongInfo_t *)&pkt9Info,
		sizeof(pkt9),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) |  /* IP match */
          (1 << TF_STATS_BM_C1_IP_FRAG),
		  0 },
		T4_QoS_PKT_INDEX
	}

};

#ifdef _TMS320C6X
#pragma DATA_SECTION (t4IfPktInfo, ".testPkts")
#endif
static pktTestInfo_t t4IfPktInfo[] =  {

	/* Packet 10 */
	{
		(uint8_t *)pkt10,
		(pasahoLongInfo_t *)&pkt10Info,
		sizeof(pkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  0 },																	  									/* no other match */
		T4_IF_PKT_INDEX
	},

	/* Packet 11 */
	{
		(uint8_t *)pkt11,
		(pasahoLongInfo_t *)&pkt11Info,
		sizeof(pkt11),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },
		T4_IF_PKT_INDEX + 1
	},

	/* Packet 12 */
	{
		(uint8_t *)pkt12,
		(pasahoLongInfo_t *)&pkt12Info,
		sizeof(pkt12),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) |  /* IP match */
          (1 << TF_STATS_BM_C1_IP_FRAG),
		   0 },																	  									/* no other match */
		T4_IF_PKT_INDEX + 2
	},
};

ifPktTestInfo_t  t4eQoSPktTestInfo[] =  {

    {
	    /* Packet 13 */
	    {
		    (uint8_t *)pkt13,
		    (pasahoLongInfo_t *)&pkt13Info,
		    sizeof(pkt13),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* outer IP match */
		      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) }, /* inner IP match */
		    T4_EQoS_PKT_INDEX
	    },

        pa_EMAC_PORT_0,
        { 0, 0, 0 }
    },


    {
	    /* Packet 14 */
	    {
		    (uint8_t *)pkt14,
		    (pasahoLongInfo_t *)&pkt14Info,
		    sizeof(pkt14),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* outer IP match */
		      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) }, /* inner IP match */
		    T4_EQoS_PKT_INDEX
	    },

        pa_EMAC_PORT_0,
        { 0, 0, 0 }
    },

    {
	    /* Packet 15 */
	    {
		    (uint8_t *)pkt15,
		    (pasahoLongInfo_t *)&pkt15Info,
		    sizeof(pkt15),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		      0 },
		    T4_EQoS_PKT_INDEX + 1
	    },

        pa_EMAC_PORT_1,
        { 0, 0, 0}
    },

    {
	    /* Packet 16 */
	    {
		    (uint8_t *)pkt16,
		    (pasahoLongInfo_t *)&pkt16Info,
		    sizeof(pkt16),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		      (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },                                      /* UDP */                                                                /* inner IP match */
		    T4_EQoS_PKT_INDEX + 2
	    },

        pa_EMAC_PORT_2,
        { 0, 0, 0 }
    },
};


/* IP Error packet 0
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match Exception IP header Error (IP version)
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt0, ".testPkts")
#endif
static uint8_t epkt0[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x75, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,PASAHO_HDR_IPv4),     /* end offset = 142, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t epkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_IPv4,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* IP Error packet 1
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match Exception IP header Error (IP version length)
 * pkt index (pkt[6]): 1
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt1, ".testPkts")
#endif
static uint8_t epkt1[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x44, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,PASAHO_HDR_IPv4),     /* end offset = 142, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t epkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_IPv4,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* IP Error packet 2
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match Exception IP header Error (IP total length)
 * pkt index (pkt[6]): 2
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt2, ".testPkts")
#endif
static uint8_t epkt2[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x02, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,PASAHO_HDR_IPv4),     /* end offset = 142, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t epkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_IPv4,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* IP Error packet 3
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match Exception IP header Error (IP TTL=0)
 * pkt index (pkt[6]): 3
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt3, ".testPkts")
#endif
static uint8_t epkt3[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x03, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,PASAHO_HDR_IPv4),     /* end offset = 142, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t epkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_IPv4,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* IP Error packet 4
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match Exception IP header Error (IP source address =broadcast)
 * pkt index (pkt[6]): 4
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt4, ".testPkts")
#endif
static uint8_t epkt4[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x04, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,PASAHO_HDR_IPv4),     /* end offset = 142, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t epkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_IPv4,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* IP Error packet 5
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match Exception IP header Error (IP destination address = 0)
 * pkt index (pkt[6]): 5
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt5, ".testPkts")
#endif
static uint8_t epkt5[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x05, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00,
	0x00, 0x00, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,PASAHO_HDR_IPv4),     /* end offset = 142, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t epkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FAIL,0,0,0,0),  /* end offset = 122 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_IPv4,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* PPPoE error packet 1 (Incorrect Code)
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * PPPoE session header (verLen = 0x11, code = 0x22,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0021 (IPv4)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt6, ".testPkts")
#endif
static uint8_t epkt6[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x22,
    0x12, 0x34, 0x00, 0x6e, 0x00, 0x21, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt6Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(130,pa_EROUTE_PPPoE_FAIL,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t epkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (PPPoE) */
	TF_FORM_PKT_INFO_WORD1(130,pa_EROUTE_PPPoE_FAIL,0,0,0,0),  /* end offset = 130 */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 22, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* PPPoE error packet 2 (Incorrect version)
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * PPPoE session header (verLen = 0x01, code = 0,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0021 (IPv4)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0
 * pkt index (pkt[6]): 1
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt7, ".testPkts")
#endif
static uint8_t epkt7[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x01, 0x00,
    0x12, 0x34, 0x00, 0x6e, 0x00, 0x21, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt7Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (IP header) */
	TF_FORM_PKT_INFO_WORD1(130,pa_EROUTE_PPPoE_FAIL,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t epkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (PPPoE) */
	TF_FORM_PKT_INFO_WORD1(130,pa_EROUTE_PPPoE_FAIL,0,0,0,0),  /* end offset = 130 */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 22, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* PPPoE control packet
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * PPPoE session header (verLen = 0x11, code = 0,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0033
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt8, ".testPkts")
#endif
static uint8_t epkt8[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
    0x12, 0x34, 0x00, 0x6e, 0x00, 0x33, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt8Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 20 (PPP Header) */
	TF_FORM_PKT_INFO_WORD1(130,pa_EROUTE_PPPoE_CTRL,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_PPPoE),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t epkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,20),		/* cmd len = 24, start offset = 20 (PPP Header) */
	TF_FORM_PKT_INFO_WORD1(130,pa_EROUTE_PPPoE_CTRL,0,0,0,0),  /* end offset = 130 */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 0, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),PASAHO_HDR_UNKNOWN,0,0,0,0,PASAHO_HDR_BITMASK2_PPPoE,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


/* 802.3 short packet with padding error (too much paading)
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt9, ".testPkts")
#endif
static uint8_t epkt9[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x00, 0x28, 0xaa, 0xaa,
    0x03, 0x00, 0x00, 0x01, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x0c,
	0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xde, 0xad, 0xbe, 0xef
	};

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt9Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,22),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 22 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(68,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_PPPoE),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t epkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,22),		/* cmd len = 24, start offset = 22 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(68,0,0,0,0,0),   /* end offset = 68 */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 0, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),PASAHO_HDR_UNKNOWN,0,0,0,0,PASAHO_HDR_BITMASK2_802_3,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


/* IP Fragments
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt10, ".testPkts")
#endif
static uint8_t epkt10[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x20, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt10Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t epkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,1,34),		        /* cmd len = 24, pmatch = 1, frag = 1, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FRAG,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static pasahoLongInfo_t epkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 22 (IP Header) */
	TF_FORM_PKT_INFO_WORD1(122,pa_EROUTE_IP_FRAG,0,0,0,0),   /* end offset = 68 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 0, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = UDP, vlan count = 0, ip count = 0, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef NSS_GEN2

/* packet 11
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 5)
 * out ip dest = 71.72.73.74
 * Designed to match NAT-T CTRL
 * Src port = 0xaabb, dest Port = 4500 (0x1194)
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt11, ".testPkts")
#endif
static uint8_t epkt11[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0x47, 0x48,
	0x49, 0x4a, 0xaa, 0xbb, 0x11, 0x94, 0x00, 0x4e,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x6c, 0x6d, 0x01, 0xc0, 0x01, 0xfa,
	0xce, 0x00, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt11Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(112,pa_EROUTE_NAT_T_CTRL,0,PASAHO_HDR_UNKNOWN), /* end offset = 108, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,00,00,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 12
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 5)
 * out ip dest = 71.72.73.74
 * Designed to match NAT-T Keep Alive
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt12, ".testPkts")
#endif
static uint8_t epkt12[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0x47, 0x48,
	0x49, 0x4a, 0x80, 0x00, 0x11, 0x94, 0x00, 0x09,
	0x00, 0x00, 0xFF
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt12Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(43,pa_EROUTE_NAT_T_KEEPALIVE,0,PASAHO_HDR_UNKNOWN), /* end offset = 43, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,00,00,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 13
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 5)
 * out ip dest = 71.72.73.74
 * Designed to match NAT-T Keep Alive (Fail)
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt index (pkt[6]): 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt13, ".testPkts")
#endif
static uint8_t epkt13[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0x47, 0x48,
	0x49, 0x4a, 0x80, 0x00, 0x11, 0x94, 0x00, 0x09,
	0x00, 0x00, 0xcc
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt13Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(43,pa_EROUTE_NAT_T_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 43, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,00,00,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 14
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 5)
 * out ip dest = 71.72.73.74
 * Designed to match NAT-T Parsing Error
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt index (pkt[6]): 1
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt14, ".testPkts")
#endif
static uint8_t epkt14[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0x47, 0x48,
	0x49, 0x4a, 0x80, 0x00, 0x11, 0x94, 0x00, 0x0a,
	0x00, 0x00, 0xff, 0x00
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt14Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt14Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(44,pa_EROUTE_NAT_T_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 44, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,00,00,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION (t4ePktInfo, ".testPkts")
#endif
static pktTestInfo_t t4ePktInfo[] =  {

	/* Packet 0 */
	{
		(uint8_t *)epkt0,
		(pasahoLongInfo_t *)&epkt0Info,
		sizeof(epkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  			/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4),               /* IP match */
		  0 },																	  			/* no other match */
		T4_IP_FAIL_PKT_INDEX
	},

	/* Packet 1 */
	{
		(uint8_t *)epkt1,
		(pasahoLongInfo_t *)&epkt1Info,
		sizeof(epkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  			/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	            /* IP match */
		  0 },																	  			/* no other match */
		T4_IP_FAIL_PKT_INDEX
	},

	/* Packet 2 */
	{
		(uint8_t *)epkt2,
		(pasahoLongInfo_t *)&epkt2Info,
		sizeof(epkt2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  			/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	            /* IP match */
		  0 },																	  			/* no other match */
		T4_IP_FAIL_PKT_INDEX
	},

	/* Packet 3 */
	{
		(uint8_t *)epkt3,
		(pasahoLongInfo_t *)&epkt3Info,
		sizeof(epkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  			/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	            /* IP match */
		  0 },																	  			/* no other match */
		T4_IP_FAIL_PKT_INDEX
	},

	/* Packet 4 */
	{
		(uint8_t *)epkt4,
		(pasahoLongInfo_t *)&epkt4Info,
		sizeof(epkt4),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  			/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	            /* IP match */
		  0 },																	  			/* no other match */
		T4_IP_FAIL_PKT_INDEX
	},

	/* Packet 5 */
	{
		(uint8_t *)epkt5,
		(pasahoLongInfo_t *)&epkt5Info,
		sizeof(epkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  			/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	            /* IP match */
		  0 },																	  			/* no other match */
		T4_IP_FAIL_PKT_INDEX
	},

	/* Packet 6 */
	{
		(uint8_t *)epkt6,
		(pasahoLongInfo_t *)&epkt6Info,
		sizeof(epkt6),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS),  									/* MAC packet */
		  0,                                                                    /* No other match */
		  0 },
		 T4_PPPoE_FAIL_PKT_INDEX
	},

	/* Packet 7 */
	{
		(uint8_t *)epkt7,
		(pasahoLongInfo_t *)&epkt7Info,
		sizeof(epkt7),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS),  									/* MAC packet */
		  0,                                                                    /* No other match */
		  0 },
		 T4_PPPoE_FAIL_PKT_INDEX
	},

	/* Packet 8 */
	{
		(uint8_t *)epkt8,
		(pasahoLongInfo_t *)&epkt8Info,
		sizeof(epkt8),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS),  									/* MAC packet */
		  0,                                                                    /* No other match */
		  0 },
		 T4_PPPoE_CTRL_PKT_INDEX
	},


    /* Packet 9 */
	{
		(uint8_t *)epkt9,
		(pasahoLongInfo_t *)&epkt9Info,
		sizeof(epkt9),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_SILENT_DISCARD), /* MAC packet */
		  0,                                                                        /* No other match */
		  0 },
		 T4_DROP_PKT_INDEX
	},

   #if 0

   // This packet will be stuck at RA if it is enabled
   // For reference only:the IP Frag exception route has been verified with packet12 within If-based routing group

	/* Packet 10 */
	{
		(uint8_t *)epkt10,
		(pasahoLongInfo_t *)&epkt10Info,
		sizeof(epkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) |  /* IP match */
          (1 << TF_STATS_BM_C1_IP_FRAG),
		   0 },																	  									/* no other match */
		T4_IP_FRAG_PKT_INDEX
	},

   #endif


   #ifdef NSS_GEN2

	/* Packet 11 */
	{
		(uint8_t *)epkt11,
		(pasahoLongInfo_t *)&epkt11Info,
		sizeof(epkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		   0 },																	  									/* no other match */
		T4_NAT_T_CTRL_PKT_INDEX
	},

	/* Packet 12 */
	{
		(uint8_t *)epkt12,
		(pasahoLongInfo_t *)&epkt12Info,
		sizeof(epkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		   0 },																	  									/* no other match */
		T4_NAT_T_KEEPALIVE_PKT_INDEX
	},

	/* Packet 13 */
	{
		(uint8_t *)epkt13,
		(pasahoLongInfo_t *)&epkt13Info,
		sizeof(epkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		   0 },																	  									/* no other match */
		T4_NAT_T_FAIL_PKT_INDEX
	},

	/* Packet 14 */
	{
		(uint8_t *)epkt14,
		(pasahoLongInfo_t *)&epkt14Info,
		sizeof(epkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		   0 },																	  									/* no other match */
		T4_NAT_T_FAIL_PKT_INDEX
	},

   #endif

};

/* Egress EQoS packets */

/* packet 0
 * EQoS DP-bit without override, tagged IP/UDP packet
 * VLAN Pbit = 0x6 ==> offset = 2
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt0, ".testPkts")
#endif
static uint8_t eQoSpkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x00, 0x81, 0x00, 0xC9, 0x99,
    0x08, 0x00, 0x45, 0x40, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x00, 0x00, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x48, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0x00, 0x00, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    };


#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count0, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 1
 * EQoS DP-bit without override, untagged IP packet
 * IP DSCP = 0x22 ==> offset = 12
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt1, ".testPkts")
#endif
static uint8_t eQoSpkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x01, 0x08, 0x00, 0x45, 0x88,
    0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x00, 0x00, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x48, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0x00, 0x00, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(122,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(122,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 2
 * EQoS DP-bit without override, tagged 802.3 IP/UDP packet
 * VLAN Pbit = 0x5 ==> offset = 1
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt2, ".testPkts")
#endif
static uint8_t eQoSpkt2[] = {
	0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x00, 0x64, 0xaa, 0xaa,
    0x03, 0x11, 0x22, 0x33, 0x81, 0x00, 0xA5, 0x55,
    0x08, 0x00, 0x45, 0x40, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x00, 0x00, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x48, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0x00, 0x00, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    };


#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(26,46,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(26,46,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count0, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


/* packet 3
 * EQoS DP-bit without override, untagged IP packet with MPLS
 * IP DSCP = 4 ==> offset = 8
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt3, ".testPkts")
#endif
static uint8_t eQoSpkt3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
	0xa6, 0x66, 0x57, 0x01, 0x88, 0x47, 0x12, 0x34,
    0x51, 0x64, 0x45, 0x10, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x00, 0x00, 0x9e, 0xda,
	0x6d, 0x19, 0x19, 0x1a, 0x1b, 0x48, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0x00, 0x00, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 4:
 * DP-Bit mode: untagged, non-IP packet to EMAC PORT 1:  defPri = 2 ==> offset = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt4, ".testPkts")
#endif
static uint8_t eQoSpkt4[] = {
	0x10, 0x22, 0x33, 0x44, 0x88, 0x02, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x10,
	0x08, 0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
	0x00, 0x00, 0x05, 0x11, 0x75, 0x4e, 0x0a, 0x0b,
	0x0c, 0x0d, 0x14, 0x15, 0x16, 0x17, 0x11, 0x11,
	0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
	0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
	0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
	0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
	0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
	0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
	0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 5
 * EQoS DP-bit with override set, untagged IP packet
 * Default priority 2 ==> offset = 3
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt5, ".testPkts")
#endif
static uint8_t eQoSpkt5[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x51, 0x52, 0x53, 0x02, 0xe8, 0xe9,
	0xea, 0xeb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x00, 0x00, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0x00,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(122,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(122,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


/* packet 6
 * EQoS DSCP mode, tagged IP packet
 * DSCP 0x04 ==> offset = 11
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt6, ".testPkts")
#endif
static uint8_t eQoSpkt6[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x81, 0x00, 0x0a, 0xaa,
    0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x17, 0x18, 0x19, 0x80, 0x47, 0x48,
	0x49, 0x48, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x00, 0x00, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0x00,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt6Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		         /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0,0,0),       /* end offset */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),		     /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count0, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 7:
 * DSCP mode: tagged, non-IP packet to EMAC PORT 3:  defPri = 2 ==> offset = 6
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt7, ".testPkts")
#endif
static uint8_t eQoSpkt7[] = {
	0x10, 0x22, 0x33, 0x44, 0x88, 0x02, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x10, 0x81, 0x00, 0xC9, 0x99,
	0x08, 0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
	0x00, 0x00, 0x05, 0x11, 0x75, 0x4e, 0x0a, 0x0b,
	0x0c, 0x0d, 0x14, 0x15, 0x16, 0x17, 0x11, 0x11,
	0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
	0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
	0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
	0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
	0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
	0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
	0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt7Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		        /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(110,0,0,0,0,0),      /* end offset */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		    /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 8
 * EQoS DSCP mode, PPPoE IP packet
 * DSCP 0x30 ==> offset = 15
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt8, ".testPkts")
#endif
static uint8_t eQoSpkt8[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x88, 0x64, 0x11, 0x00,
    0x12, 0x34, 0x00, 0x00, 0x00, 0x21, 0x45, 0xC0,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x17, 0x18, 0x19, 0x80, 0x47, 0x48,
	0x49, 0x48, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x00, 0x00, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0x00,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt8Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag, start offset */
	TF_FORM_PKT_INFO_WORD1(130,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,42,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		         /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(130,0,0,0,0,0),       /* end offset */
	TF_FORM_PKT_INFO_WORD2(22,42,0,0),		     /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count0, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 9
 * EQoS DSCP mode, tagged IP packet
 * DSCP 0x6 ==> offset = 9
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt9, ".testPkts")
#endif
static uint8_t eQoSpkt9[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x81, 0x00, 0x0a, 0xaa,
    0x08, 0x00, 0x45, 0x18,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x17, 0x18, 0x19, 0x80, 0x47, 0x48,
	0x49, 0x48, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x00, 0x00, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xee,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (eQoSpkt9Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t eQoSpkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		     /* cmd len, pmatch, frag, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0),           /* end offset, errIdx, portNum, nextHdr */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),    	     /* L3 offset */
	TF_FORM_PKT_INFO_WORD3(0,0,0),               /* bitmap, pdspNum, liIndex */
    TF_FORM_PKT_INFO_WORD4(0,0,0,0,0),           /* VC, pri, vlan Count, greCount, ipCount */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static pasahoLongInfo_t eQoSpkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		         /* cmd len, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,0,0,0),       /* end offset */
	TF_FORM_PKT_INFO_WORD2(18,38,0,0),		     /* L3 offset, l4Offset, l5Offset, ahEspOffset */

	/* bitmap, next header = UDP, vlan count, ip count, gre count0, frag, ip route options, multi route */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif




ifPktTestInfo_t  t4eQoSEgressPktTestInfo[] =  {

    {
	    /* Packet 0 */
	    {
		    (uint8_t *)eQoSpkt0,
		    (pasahoLongInfo_t *)&eQoSpkt0Info,
		    sizeof(eQoSpkt0),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_1,
        { 0, 0, 0 }
    },

    {
	    /* Packet 1 */
	    {
		    (uint8_t *)eQoSpkt1,
		    (pasahoLongInfo_t *)&eQoSpkt1Info,
		    sizeof(eQoSpkt1),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_1,
        { 0, 0, 0 }
    },

    {
	    /* Packet 2 */
	    {
		    (uint8_t *)eQoSpkt2,
		    (pasahoLongInfo_t *)&eQoSpkt2Info,
		    sizeof(eQoSpkt2),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_1,
        { 0, 0, 0 }
    },

    {
	    /* Packet 3 */
	    {
		    (uint8_t *)eQoSpkt3,
		    (pasahoLongInfo_t *)&eQoSpkt3Info,
		    sizeof(eQoSpkt3),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_1,
        { 0, 0, 0 }
    },

    {
	    /* Packet 4 */
	    {
		    (uint8_t *)eQoSpkt4,
		    (pasahoLongInfo_t *)&eQoSpkt4Info,
		    sizeof(eQoSpkt4),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_1,
        { 0, 0, 0 }
    },

    {
	    /* Packet 5 */
	    {
		    (uint8_t *)eQoSpkt5,
		    (pasahoLongInfo_t *)&eQoSpkt5Info,
		    sizeof(eQoSpkt5),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_2,
        { 0, 0, 0 }
    },

    {
	    /* Packet 6 */
	    {
		    (uint8_t *)eQoSpkt6,
		    (pasahoLongInfo_t *)&eQoSpkt6Info,
		    sizeof(eQoSpkt6),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_3,
        { 0, 0, 0 }
    },

    {
	    /* Packet 7 */
	    {
		    (uint8_t *)eQoSpkt7,
		    (pasahoLongInfo_t *)&eQoSpkt7Info,
		    sizeof(eQoSpkt7),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_3,
       { 0, 0, 0 }
    },

    {
	    /* Packet 8 */
	    {
		    (uint8_t *)eQoSpkt8,
		    (pasahoLongInfo_t *)&eQoSpkt8Info,
		    sizeof(eQoSpkt8),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_3,
        { 0, 0, 0 }
    },

    {
	    /* Packet 9 */
	    {
		    (uint8_t *)eQoSpkt9,
		    (pasahoLongInfo_t *)&eQoSpkt9Info,
		    sizeof(eQoSpkt9),
		    {0, 0, 0},
		    0
	    },

        pa_EMAC_PORT_3,
        { 60,               /* MTU size */
          0, 0 }
    },


};

#endif /*TEST4PKTS_H_*/
