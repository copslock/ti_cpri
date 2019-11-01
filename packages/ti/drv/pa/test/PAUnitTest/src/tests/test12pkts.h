/*
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef TEST12PKTS_H_
#define TEST12PKTS_H_

/* Valid rx MAC addresses used during the test */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t12EthInfo, ".testPkts")
#endif
static paEthInfo_t t12EthInfo[] =  {

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
    }


};


/* Outer IP configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t12OuterIpInfo, ".testPkts")
#endif
static t12IpPaSetup_t t12OuterIpInfo[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		0,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		1,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 1 ----------- */
	{
		1,			/* Sequential ID */
		1,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

		/* ------- Entry 3 ----------- */
		/* The same IP address as Entry 2, but with a different assocated mac */
	{
			3,		/* Sequential ID */
			3,		/* Local Handle Index */
            pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
			1,		/* L2 Handle Index */
			0,			/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

		/* ------- Entry 5 ----------- */
		/* Route on source IP, dest IP, protocol = UDP(more specific then Entry 2, 3) */
	{
		5,			/* Sequential ID */
		5,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

				/* ------- Entry 10 ----------- */
		/* Route based on source IP only */
	{
		10,			/* Sequential ID */
		8,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 81, 82, 83, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

				/* ------- Entry 11 ----------- */
		/* Route based on source GRE protocol only */
	{
		11,			/* Sequential ID */
		9,			/* Local Handle Index */
        pa_LUT1_INST_1,          /* LUT1 Instance (LUT1_1) */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

				/* ------- Entry 12 ----------- */
		/* Route based on SPI only */
        /* TBD: Disable SPI (0x12345678)  */
	{
		12,			/* Sequential ID */
		10,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 81, 82, 83, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

				/* ------- Entry 13 ----------- */
		/* IPv6 route on dest only */
	{
		13,			/* Sequential ID */
		11,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

				/* ------- Entry 15 ----------- */
		/* IPv6 route on dest, source, flow */
	{
		15,			/* Sequential ID */
		13,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66 },		/* IP Source address */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

				/* ------- Entry 23 ----------- */
		/* Route on Destination IP only. On match continue parse (used for nested IP) */
	{
		23,		/* Sequential ID */
		19,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		2,		/* Route Index - continue L3 parse */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

					/* ------- Entry 25 ----------- */
		/* Route on Destination IP only */
	{
		25,		/* Sequential ID */
		21,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 81, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

					/* ------- Entry 28 ----------- */
		/* Route on Destination IP only */
	{
		28,		/* Sequential ID */
		24,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

					/* ------- Entry 29 ----------- */
		/* Route on Destination IP only */
	{
		29,		/* Sequential ID */
		25,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 76, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 82, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 83, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 86, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 87, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 88, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 91, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 92, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 95, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 98, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 99, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

					/* ------- Entry 62 ----------- */
		/* Route on Destination IP only */
	{
		62,		/* Sequential ID */
		58,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
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
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 112, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},


					/* ------- Entry 66 ----------- */
		/* Route on Destination IP only */
	{
		66,		/* Sequential ID */
		62,		/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

					/* ------- Entry 67 ----------- */
		/* Route on Destination IP only */
	{
		67,		/* Sequential ID */
		63,		/* Local Handle Index */
        pa_LUT1_INST_1,      /* LUT1 Instance (LUT1_1) */
		0,		/* L2 Handle Index */
		0,		/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

#ifdef PASS_LUT_LIMIT_TEST

				/* ------- Entry 68 ----------- */
		/* Route on Destination IP only - the hardware table is full */
	{
		68,							/* Sequential ID */
		T12_NUM_LOCAL_L3_HANDLES,	/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		/* IP Source address */
			{ 71, 72, 73, 115, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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
		T12_NUM_LOCAL_L3_HANDLES,	/* Local Handle Index */
        3,                          /* LUT1 Instance */
		0,							/* L2 Handle Index */
		0,							/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },		    /* IP Source address */
			{ 71, 72, 73, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},		/* IP Destination address */
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


/* The following configurations link to an IP header (nested inner IP) */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t12InnerIpInfo, ".testPkts")
#endif
static t12IpPaSetup_t t12InnerIpInfo[] =  {

	/* ------- Entry 0 -----------
	 * match IP dest only  */
	{
		0,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 0,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		19,												/* L3 Outer IP Handle Index */
                                                        /* Note: It is entry 23 */
		0,												/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  	 /* IP Source address */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 1 -----------
	 * match IP dest and source */
	{
		1,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 1,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		19,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 2 -----------
	 * match IP dest, srouce, protocol, and sctp Port */
	{
		2,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 2,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		19,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 3 -----------
	 * match IP dest, srouce, protocol, TOS */
	{
		3,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 3,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		19,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 4 -----------
	 * match IP dest only, outer IP index = 24 */
	{
		4,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 4,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 20, 21, 22, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 5 -----------
	 * match IP dest, source , outer IP index = 24 */
	{
		5,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 5,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  20,  21,  22,  23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 6 -----------
	 * match IP dest, source, protocol , outer IP index = 24 */
	{
		6,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 6,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  20,  21,  22,  23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 7 -----------
	 * match IP dest, source, protocol , outer IP index = 24 */
	{
		7,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 7,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		24,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{ 120, 121, 122, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  20,  21,  22,  23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 10 -----------
	 * match IP dest outer IP index = 23 */
	{
		10,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 8,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 11 -----------
	 * match IP dest outer IP index = 23 */
	{
		11,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 9,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 12 -----------
	 * match IP dest outer IP index = 23 */
	{
		12,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 10,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 13 -----------
	 * match IP dest outer IP index = 23 */
	{
		13,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 11,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 14 -----------
	 * match IP dest outer IP index = 23 */
	{
		14,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 12,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 15 -----------
	 * match IP dest outer IP index = 23 */
	{
		15,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 13,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 16 -----------
	 * match IP dest outer IP index = 23 */
	{
		16,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 14,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 17 -----------
	 * match IP dest outer IP index = 23 */
	{
		17,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 15,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 18 -----------
	 * match IP dest outer IP index = 23 */
	{
		18,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 16,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 19 -----------
	 * match IP dest outer IP index = 23 */
	{
		19,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 17,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 20 -----------
	 * match IP dest outer IP index = 23 */
	{
		20,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 18,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 21 -----------
	 * match IP dest outer IP index = 23 */
	{
		21,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 19,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 22 -----------
	 * match IP dest outer IP index = 23 */
	{
		22,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 20,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 23 -----------
	 * match IP dest outer IP index = 23 */
	{
		23,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 21,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 24 -----------
	 * match IP dest outer IP index = 23 */
	{
		24,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 22,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 25 -----------
	 * match IP dest outer IP index = 23 */
	{
		25,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 23,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 26 -----------
	 * match IP dest outer IP index = 23 */
	{
		26,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 24,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 27 -----------
	 * match IP dest outer IP index = 23 */
	{
		27,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 25,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 28 -----------
	 * match IP dest outer IP index = 23 */
	{
		28,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 26,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 29 -----------
	 * match IP dest outer IP index = 23 */
	{
		29,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 27,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 30 -----------
	 * match IP dest outer IP index = 23 */
	{
		30,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 28,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 31 -----------
	 * match IP dest outer IP index = 23 */
	{
		31,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 29,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 41, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 32 -----------
	 * match IP dest outer IP index = 23 */
	{
		32,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 30,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 33 -----------
	 * match IP dest outer IP index = 23 */
	{
		33,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 31,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 34 -----------
	 * match IP dest outer IP index = 23 */
	{
		34,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 32,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 44, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 35 -----------
	 * match IP dest outer IP index = 23 */
	{
		35,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 33,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 36 -----------
	 * match IP dest outer IP index = 23 */
	{
		36,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 34,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 46, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 37 -----------
	 * match IP dest outer IP index = 23 */
	{
		37,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 35,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 47, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 38 -----------
	 * match IP dest outer IP index = 23 */
	{
		38,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 36,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 39 -----------
	 * match IP dest outer IP index = 23 */
	{
		39,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 37,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 49, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 40 -----------
	 * match IP dest outer IP index = 23 */
	{
		40,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 38,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 41 -----------
	 * match IP dest outer IP index = 23 */
	{
		41,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 39,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 42 -----------
	 * match IP dest outer IP index = 23 */
	{
		42,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 40,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 43 -----------
	 * match IP dest outer IP index = 23 */
	{
		43,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 41,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 53, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 44 -----------
	 * match IP dest outer IP index = 23 */
	{
		44,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 42,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 54, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 45 -----------
	 * match IP dest outer IP index = 23 */
	{
		45,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 43,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 46 -----------
	 * match IP dest outer IP index = 23 */
	{
		46,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 44,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 47 -----------
	 * match IP dest outer IP index = 23 */
	{
		47,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 45,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 48 -----------
	 * match IP dest outer IP index = 23 */
	{
		48,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 46,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 58, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 49 -----------
	 * match IP dest outer IP index = 23 */
	{
		49,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 47,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 59, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 50 -----------
	 * match IP dest outer IP index = 23 */
	{
		50,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 48,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},


	/* ------- Entry 51 -----------
	 * match IP dest outer IP index = 23 */
	{
		51,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 49,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 61, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 52 -----------
	 * match IP dest outer IP index = 23 */
	{
		52,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 50,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 53 -----------
	 * match IP dest outer IP index = 23 */
	{
		53,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 51,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 54 -----------
	 * match IP dest outer IP index = 23 */
	{
		54,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 52,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 55 -----------
	 * match IP dest outer IP index = 23 */
	{
		55,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 53,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 56 -----------
	 * match IP dest outer IP index = 23 */
	{
		56,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 54,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 57 -----------
	 * match IP dest outer IP index = 23 */
	{
		57,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 55,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 67, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 58 -----------
	 * match IP dest outer IP index = 23 */
	{
		58,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 56,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 59 -----------
	 * match IP dest outer IP index = 23 */
	{
		59,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 57,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 60 -----------
	 * match IP dest outer IP index = 23 */
	{
		60,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 58,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 61 -----------
	 * match IP dest outer IP index = 23 */
	{
		61,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 59,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 62 -----------
	 * match IP dest outer IP index = 23 */
	{
		62,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 60,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 63 -----------
	 * match IP dest outer IP index = 23 */
	{
		63,												/* Sequential ID */
		T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 61,			/* Local Handle Index */
        pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
		23,												/* L3 Outer IP Handle Index */
		0,												/* Route Index - route to host */
		{
			{   0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{  25, 26, 27, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		FALSE		/* Set to true when the command is acked */
	}

};

/* Outer ACL configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t12OuterAclInfo, ".testPkts")
#endif
static t12AclPaSetup_t t12OuterAclInfo[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		0,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP |
            pa_ACL_INFO_VALID_DSCP,                               /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			4,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 1 ----------- */
	{
		1,			/* Sequential ID */
		1,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		0,			/* L2 Handle Index */
        -1,          /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_SRC_IP |
            pa_ACL_INFO_VALID_DST_IP,                            /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 2 ----------- */
	{
		2,			/* Sequential ID */
		2,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		0,			/* L2 Handle Index */
        1,          /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_PORT  |
            pa_ACL_INFO_VALID_SRC_IP    |
            pa_ACL_INFO_VALID_DST_IP  ,                          /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0555,     /* destPortbegin */
            0x0555,     /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},


	/* ------- Entry 3 ----------- */
	{
		3,			/* Sequential ID */
		3,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 4 ----------- */
	{
		4,			/* Sequential ID */
		4,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        3,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_PROTO |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			1,			/* Protocol (ICMP) */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 5 ----------- */
	{
		5,			/* Sequential ID */
		5,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        3,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_SRC_IP |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 81, 82, 83, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol (ICMP) */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},


	/* ------- Entry 6 ----------- */
	{
		6,			/* Sequential ID */
		6,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 7 ----------- */
	{
		7,			/* Sequential ID */
		7,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        3,         /* next ACL handle index */
		pa_ACL_ACTION_DENY,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_SRC_IP        |
            pa_ACL_INFO_VALID_SRC_IP_MASK   |
            pa_ACL_INFO_VALID_DST_IP_MASK   |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 81, 82, 82, 0x50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0xFF, 0xFF, 0xFF, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol (ICMP) */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 8 ----------- */
	{
		8,			/* Sequential ID */
		8,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        6,         /* next ACL handle index */
		pa_ACL_ACTION_DENY,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_CTRL_FLAG |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            pa_ACL_INFO_CONTROL_FLAG_FRAG,                        /* ctrlFlag */
            pa_ACL_INFO_CONTROL_FLAG_FRAG,                        /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 9 ----------- */
	{
		9,			/* Sequential ID */
		9,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP_MASK |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 0, 0, 0, 0},		/* IP Destination address */
			{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 10 ----------- */
	{
        /* same as entry 6 */
		10,			/* Sequential ID */
		63,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_INVALID_DUP_ACL_ENTRY,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},


	/* ------- Entry 11 ----------- */
	{
		11,			/* Sequential ID */
		63,			/* Local Handle Index */
        pa_LUT1_INST_0_0, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_INVALID_LUT_INST,	/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 12 ----------- */
	{
        /* same as entry 6 */
		12,			/* Sequential ID */
		63,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		-1,			/* L2 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_HOST,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 71, 72, 73, 79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_INVALID_ACL_ACTION,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 13 ----------- */
	{
		13,			/* Sequential ID */
		13,			/* Local Handle Index */
        pa_ACL_INST_OUTER_IP, /* ACL LUT Instance */
		0,			/* L2 Handle Index */
        1,          /* next ACL handle index */
		pa_ACL_ACTION_DENY,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_PORT  |
            pa_ACL_INFO_VALID_PROTO     |
            pa_ACL_INFO_VALID_SRC_IP    |
            pa_ACL_INFO_VALID_DST_IP  ,                          /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			132,		/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0700,     /* destPortbegin */
            0x0800,     /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

};



/* Inner ACL configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t12InnerAclInfo, ".testPkts")
#endif
static t12AclPaSetup_t t12InnerAclInfo[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		T12_NUM_LOCAL_OUTER_ACL_HANDLES + 0,			/* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
		19,			/* L3 Handle Index (note: It is entry 23) */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 1 ----------- */
	{
		1,			/* Sequential ID */
		T12_NUM_LOCAL_OUTER_ACL_HANDLES + 1,			/* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
		19,			/* L2 Handle Index */
        T12_NUM_LOCAL_OUTER_ACL_HANDLES + 0,            /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_SRC_IP |
            pa_ACL_INFO_VALID_DST_IP  ,                          /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 2 ----------- */
	{
		2,			/* Sequential ID */
		T12_NUM_LOCAL_OUTER_ACL_HANDLES + 2,			/* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
		19,			/* L3 Handle Index (note: It is entry 23) */
        T12_NUM_LOCAL_OUTER_ACL_HANDLES + 0,            /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_PORT |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0400,     /* destPortbegin */
            0x0600,     /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 3 ----------- */
	{
		3,			/* Sequential ID */
		T12_NUM_LOCAL_OUTER_ACL_HANDLES + 3,			/* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
		19,			/* L2 Handle Index */
        T12_NUM_LOCAL_OUTER_ACL_HANDLES + 1,            /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			                /* ACL actions */
		{
            pa_ACL_INFO_VALID_PROTO  |
            pa_ACL_INFO_VALID_SRC_IP |
            pa_ACL_INFO_VALID_DST_IP  ,                          /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			132,    	/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 4 ----------- */
	{
		4,			/* Sequential ID */
		T12_NUM_LOCAL_OUTER_ACL_HANDLES + 4,			/* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
		19,			/* L3 Handle Index (note: It is entry 23) */
        T12_NUM_LOCAL_OUTER_ACL_HANDLES + 2,            /* next ACL handle index */
		pa_ACL_ACTION_DENY,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_SRC_PORT |
            pa_ACL_INFO_VALID_DST_PORT |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xAAC0,     /* srcPortbegin */
            0xAAF0,     /* srcPort end */
            0x0400,     /* destPortbegin */
            0x0600,     /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 5 ----------- */
	{
		2,			/* Sequential ID */
		127, 		/* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
		19,			/* L3 Handle Index (note: It is entry 23) */
        T12_NUM_LOCAL_OUTER_ACL_HANDLES + 0,            /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_PORT |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0600,     /* destPortbegin */
            0x0400,     /* destPort end */
		},
		pa_ERR_CONFIG,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

};

/* packet 0
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.203 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Permitted by outer ACL Rule 0
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x10,
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
static pasahoLongInfo_t pkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 1
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Permitted by outer ACL Rule 2
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x10,
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
static pasahoLongInfo_t pkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 2
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info 23)
 * inner ip dest = 10.11.12.13 (inner IP info 0)
 * UDP (destination port = 0x0555)
 * Permitted by outer ACL Rule 6 and inner ACL Rule 2
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
static pasahoLongInfo_t pkt2Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 3
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info 23)
 * inner ip dest = 10.11.12.13 (inner IP info 2)
 * inner ip src  = 10.11.12.13
 * inner ip protocol = 132 (0x84) (SCTP)
 * Permitted by outer ACL Rule 6 and inner ACL Rule 3
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
static pasahoLongInfo_t pkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,66),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 66 (SCTP Header) */
	TF_FORM_PKT_INFO_WORD1(114,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 114, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,54,0,0),    	            /* L3 offset = 34, l4Offset = 54, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_SCTP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 4
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * PPPoE session header (verLen = 0x11, code = 0,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0021 (IPv4)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * UDP (destination port = 0x0555)
 * Permitted by outer ACL Rule 2
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
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UDP),     /* end offset = 114, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,42,0,0),    	            /* L3 offset = 34, l4Offset = 42, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_PPPoE),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 5
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 2)
 * out ip dest = 71.72.73.74
 * fake ICMP packet
 * Permitted by outer ACL Rule 4
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
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (ICMP) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};


/* packet 6
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84 (out IP info 4)
 * out ip dest = 71.72.73.74
 * fake IPSEC ESP packet
 * Permitted by outer ACL Rule 5
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
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (ESP) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,34),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_ESP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};


/* packet 8
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.82.85 (rejected by ACL rule 81.82.82.8x)
 * out ip dest = 71.72.73.74
 * UDP (destination port = 0x0555)
 * rejected by ACL rule 7
 * Designed to match outer IP configuration 2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8, ".testPkts")
#endif
static uint8_t pkt8[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x52, 0x55, 0x47, 0x48,
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
#pragma DATA_SECTION (pkt8Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (ICMP) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 9
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info 23)
 * UDP (destination port = 0x0555)
 * Fragment: rejected by ACL rule 8
 * Designed to match inner IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9, ".testPkts")
#endif
static uint8_t pkt9[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x20, 0x00, 0x05, 0x11,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x46, 0xaa, 0xbb, 0x05, 0x55, 0x00, 0x58,
    0xeb, 0xc7, 0x14, 0x15,
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
#pragma DATA_SECTION (pkt9Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt9Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 122, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 10
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info 23)
 * inner ip dest = 10.11.12.13 (inner IP info 0)
 * UDP (destination port = 0x0555)
 *     (source port = 0xaae0) rejected by ACL range rule (0xaac0, 0xaaf0)
 * Permitted by outer ACL rule 6
 * Rejected by inner ACL rule 4
 * Designed to match inner IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10, ".testPkts")
#endif
static uint8_t pkt10[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
	0x49, 0x46, 0x45, 0x00, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x93, 0x85, 0x9e, 0xda,
	0x6d, 0x0a, 0x0a, 0x0b, 0x0c, 0x0d, 0xaa, 0xe0,
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
#pragma DATA_SECTION (pkt10Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt10Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 11
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 110.111.112.113 (out IP info 1)
 * out ip dest = 200.201.202.203
 * SCTP (destination port = 0x0780)
 * Permitted by outer ACL Rule 13
 * Rejected to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11, ".testPkts")
#endif
static uint8_t pkt11[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x84,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
    0xca, 0xcb, 0x12, 0x34, 0x07, 0x80, 0x12, 0x34,  /* SCTP source port, dest port and tag */
	0x56, 0x78, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
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
#pragma DATA_SECTION (pkt11Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt11Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (SCTP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_SCTP),    /* end offset, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* Bits 14 and 15 are used to tell where the packet is expected to wind up */
#define T12_PACKET_L3_MATCH_VALID	(0 << 14) 		/* Packet will match (handle index in bits 13:0) */
#define T12_PACKET_NFAIL			(1 << 14)     	/* Packet will arrive in the next fail queue  */
#define T12_PACKET_DISCARD			(2 << 14)		/* Packet is discarded by PA */
#define T12_PACKET_DEST_MASK		(3 << 14)
#define T12_PACKET_INDEX_MASK		0x3fff

#ifdef _TMS320C6X
#pragma DATA_SECTION (t12PktInfo, ".testPkts")
#endif
static pktTestInfo_t t12PktInfo[] =  {

	/* Packet 0 */
	{
		(uint8_t *)pkt0,
		(pasahoLongInfo_t *)&pkt0Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  0 },																	  									/* no other match */
		T12_PACKET_L3_MATCH_VALID | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 1 */
	{
		(uint8_t *)pkt1,
		(pasahoLongInfo_t *)&pkt1Info,
		sizeof(pkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
		  0 },
		 T12_PACKET_L3_MATCH_VALID | 1		/* Packet will be matched by handle index 1 */
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
		 T12_PACKET_L3_MATCH_VALID | 1		/* Packet will be matched by handle index 1 */
	},

	/* Packet 5 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt5,
		(pasahoLongInfo_t *)&pkt5Info,
		sizeof(pkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH)},
		 T12_PACKET_L3_MATCH_VALID | 2		/* Packet will be matched by handle index 1 */
	},

	/* Packet 6 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt6,
		(pasahoLongInfo_t *)&pkt6Info,
		sizeof(pkt6),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH)},                                   /* SPI match */
		 T12_PACKET_L3_MATCH_VALID | 4		/* Packet will be matched by handle index 4 */
	},

	/* Packet 2 */
	{
		(uint8_t *)pkt2,
		(pasahoLongInfo_t *)&pkt2Info,
		sizeof(pkt2),
		{ ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),										/* MAC match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* Outer IP match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) },  	/* inner IP match */
		  T12_PACKET_L3_MATCH_VALID | T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 0  /* Packet will match inner IP entry 0 */
	},

	/* Packet 3 */
	{
		(uint8_t *)pkt3,
		(pasahoLongInfo_t *)&pkt3Info,
		sizeof(pkt3),
		{ ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),										/* MAC match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* Outer IP match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) },  	/* inner IP match */
		  T12_PACKET_L3_MATCH_VALID | T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + 2  /* Packet will match inner IP entry 0 */
	},


    /* packet rejected by ACL rule */
	/* Packet 8 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt8,
		(pasahoLongInfo_t *)&pkt8Info,
		sizeof(pkt8),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS),                                                                        /* IP match */
		  0},                                                                                                       /* No Match */
		T12_PACKET_DISCARD  		/* Packet will be rejected by ACL rule */
	},


	/* Packet 9 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt9,
		(pasahoLongInfo_t *)&pkt9Info,
		sizeof(pkt9),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_IP_FRAG),                                                                        /* IP match */
		  0},                                                                                                       /* No Match */
		T12_PACKET_DISCARD  		/* Packet will be rejected by ACL rule */
	},

	/* Packet 10 */
	{
		(uint8_t *)pkt10,
		(pasahoLongInfo_t *)&pkt10Info,
		sizeof(pkt10),
		{ ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),										/* MAC match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* Outer IP match */
		  ( 1 << TF_STATS_BM_C1_NUM_PACKETS)  },  	                                                                    /* inner IP match */
		T12_PACKET_DISCARD    /* Packet will be rejected by ACL rule */
	},

    /* packet rejected by ACL rule */
	/* Packet 11 */
    /* Move here since packet order should be maintained */
	{
		(uint8_t *)pkt11,
		(pasahoLongInfo_t *)&pkt11Info,
		sizeof(pkt8),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS),                                                                        /* IP match */
		  0},                                                                                                       /* No Match */
		T12_PACKET_DISCARD  		/* Packet will be rejected by ACL rule */
	},

};


#endif /*TEST12PKTS_H_*/
