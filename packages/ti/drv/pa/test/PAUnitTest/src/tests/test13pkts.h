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

#ifndef TEST13PKTS_H_
#define TEST13PKTS_H_


/* Valid rx MAC addresses used during the test */
typedef struct t13EthSetup  {
	paEthInfo_t  ethInfo;		/* PA Ethernet configuration structure */
	Bool	     acked;			/* Set to TRUE when the reply to the command is received */
} t13EthSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION (t13EthSetup, ".testPkts")
#endif
static t13EthSetup_t t13EthSetup[] =  {

    {
      {	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
      },
      FALSE  },

    {
      {	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xcc },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
      },
      FALSE  },
};

/* Valid rx MAC addresses used during the test */
typedef struct t13EthSetup2  {
	paEthInfo_t   ethInfo;		/* PA Ethernet configuration structure */
    int           routeIdx;     /* Routing Index */
	Bool	      acked;		/* Set to TRUE when the reply to the command is received */
} t13EthSetup2_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION (t13EthSetup2, ".testPkts")
#endif
static t13EthSetup2_t t13EthSetup2[] =  {

    {
      {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xbb },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
      },
      0,
      FALSE  }
};

#define T13_IP_NEXT_ROUTE_HOST      0
#define T13_IP_NEXT_ROUTE_LUT1      1
#define T13_IP_NEXT_ROUTE_LUT2      2
#define T13_IP_NEXT_ROUTE_HOST_ESP  3         /* simulate SA ESP offset fixup operation */
#define T13_IP_NEXT_ROUTE_IPSEC2    4

typedef struct t13IpSetup_s  {
    Bool       innerIp;     /* Inner IP link to MAC */
    int        nextRoute;   /* Next route */
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */

} t13IpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13IpSetup, ".testPkts")
#endif
static t13IpSetup_t  t13IpSetup[] = {
	/* IP Entry 0 */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
		0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 1 */
	{
        0,     /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
        0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 2 */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        1,	    /* Linked to outer IP  index 1 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 3 */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        0,	    /* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x38 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 4 */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
        0,	    /* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x39 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 5 */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        4,	    /* Linked to outer IP index 4 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x3a },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 6 */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
		0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0x44440000,	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0x33,		/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 7 */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route Host (ESP)*/
        0,	    /* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x38 },  /* IP Destination address */
			0x55550000,	/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0x32,   	/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 8 (IP/ESP) */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route: Host (ESP)*/
        0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0x55550001, /* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0x32,   	/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 9 (IP/NAT-T) */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route: Host */
        0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,          /* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			17,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 10 (Iink to IP/ESP */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        8,	    /* Linked to outer IP  index 8 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,      	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 11 (ESP link to IP/NAT-T)*/
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route: Host (ESP)*/
        9,	    /* Linked to outer IP  index 9 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0x55550003,	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 12 (Iink to IP/ESP */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        11,	    /* Linked to outer IP  index 8 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 201, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,       	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},


	/* IP Entry 13 (IP/AH) */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
        0,	    /* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x39 },  /* IP Destination address */
			0x44440001,	/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0x33,		/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 14 (IP/AH/ESP) */
	{
        FALSE,  /* Outer IP */
        T13_IP_NEXT_ROUTE_IPSEC2,  /* Next Route: next IPSEC stage  */
        0,	    /* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x39 },  /* IP Destination address */
			0x44440002,	/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0x33,		/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},


	/* IP Entry 15 (Link to IP/AH (0x44440001) */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        13,	    /* Linked to outer IP index 4 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x3a },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 16 (ESP:Link to IP/AH (0x44440002) */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route HOST (ESP) */
        14,	    /* Linked to outer IP index 4 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0x55550002,	/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 17 (ESP:Link to IP/AH/ESP */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        16,	    /* Linked to outer IP index 4 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x3a },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV6,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 18 (IP forwarding 1) */
	{
        0,     /* Outer IP */
        T13_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
        1,	/* Linked to dest mac index 1 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 117, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 19 (IP forwarding 2) */
	{
        TRUE,   /* Inner IP */
        T13_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
        18,	/* Linked to outer IP  index 1 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 149, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},
};


typedef struct t13IpSetup2_s  {
    Bool        innerIp;     /* Inner IP link to MAC */
    int         routeIdx;    /* routing info index */
	int		    lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	paIpInfo_t  ipInfo;		 /* PA IP configuration structure */
	Bool	    acked;		 /* Set to TRUE when the reply to the command is received */

} t13IpSetup2_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13IpSetup2, ".testPkts")
#endif
static t13IpSetup2_t  t13IpSetup2[] = {
	/* IP Entry 20 */
	{
        FALSE,  /* Outer IP */
        1,      /* Route Info Index */
		1,	    /* Linked to dest mac index 1 */
		{
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },          /* IP Source address */
			{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
            0,          /* TOSCare */
			0,			/* TOS */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 21 */
	{
        TRUE,   /* Inner IP */
        2,      /* Route Info Index */
		18,	    /* Linked to dest IP index 18 */
		{
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },          /* IP Source address */
			{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
            0,          /* TOSCare */
			0,			/* TOS */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 22 */
	{
        TRUE,   /* Inner IP */
        4,      /* Route Info Index */
		18,	    /* Linked to dest IP index 18 */
		{
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },          /* IP Source address */
			{ 200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
            0,          /* TOSCare */
			0,			/* TOS */
            0           /* SCTP port */
		},
		FALSE
	}
};

typedef struct t13UdpSetup_s  {
	int		   lHandleIdx;  /* Linked handle (to previous L3 layer) */
	uint16_t   port;		/* destination port number */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */
} t13UdpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13UdpSetup, ".testPkts")
#endif
static t13UdpSetup_t  t13UdpSetup[] = {
	/* UDP Entry 0 */
	{
		0,	    /* Linked to dest ip index 0 */
        0x8000, /* destination port number */
		FALSE
	},

	/* UDP Entry 1 */
	{
		0,	    /* Linked to dest ip index 0 */
        0x8002, /* destination port number */
		FALSE
	},

	/* UDP Entry 2 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9000, /* destination port number */
		FALSE
	},

	/* UDP Entry 3 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9002, /* destination port number */
		FALSE
	},

	/* UDP Entry 4 */
	{
		3,	    /* Linked to dest ip index 3 */
        0x8100, /* destination port number */
		FALSE
	},

	/* UDP Entry 5 */
	{
		3,	    /* Linked to dest ip index 3 */
        0x8102, /* destination port number */
		FALSE
	},

	/* UDP Entry 6 */
	{
		5,	    /* Linked to dest ip index 5 */
        0x9100, /* destination port number */
		FALSE
	},

	/* UDP Entry 7 */
	{
		5,	    /* Linked to dest ip index 5 */
        0x9102, /* destination port number */
		FALSE
	},

	/* UDP Entry 8 */
	{
		6,	    /* Linked to dest ip index 6 */
        0xa000, /* destination port number */
		FALSE
	},

	/* UDP Entry 9 */
	{
		7,	    /* Linked to dest ip index 7 */
        0xa002, /* destination port number */
		FALSE
	},

	/* UDP Entry 10 */
	{
		10,	    /* Linked to dest ip index 10 */
        0xb000, /* destination port number */
		FALSE
	},

	/* UDP Entry 11 */
	{
		15,	    /* Linked to dest ip index 15 */
        0xb002, /* destination port number */
		FALSE
	},

	/* UDP Entry 12 */
	{
		12,	    /* Linked to dest ip index 12 */
        0xc000, /* destination port number */
		FALSE
	},

	/* UDP Entry 13 */
	{
		17,	    /* Linked to dest ip index 17 */
        0xc002, /* destination port number */
		FALSE
	},

	/* UDP Entry 14 */
	{
		0,	    /* Linked to dest ip index 0 */
        0x8004, /* destination port number */
		FALSE
	},


	/* UDP Entry 15 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9004, /* destination port number */
		FALSE
	},

	/* UDP Entry 16 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9006, /* destination port number */
		FALSE
	},

	/* UDP Entry 17 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9008, /* destination port number */
		FALSE
	},

	/* UDP Entry 18 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x900a, /* destination port number */
		FALSE
	},
};

typedef struct t13UdpSetup2_s  {
	int		   lHandleIdx;  /* Linked handle (to previous L3 layer) */
    int        routeIdx;    /* Routing Info index */
	uint16_t   port;		/* destination port number */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */
} t13UdpSetup2_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13UdpSetup2, ".testPkts")
#endif
static t13UdpSetup2_t  t13UdpSetup2[] = {
	/* UDP Entry 0 */
	{
		19,	    /* Linked to dest ip index 19 */
        3,      /* routing info index */
        0x9008, /* destination port number */
		FALSE
	}
};


#define T13_MAX_EF_RECORDS          100
#ifdef _TMS320C6X
#pragma DATA_SECTION(t13EfRecs, ".testPkts")
#endif
paEfRec_t t13EfRecs[T13_MAX_EF_RECORDS];

typedef struct t13EfRec1Setup_s  {
    int             index;
	paEfRecLevel1_t rec;		/* Level 1 record */
} t13EfRec1Setup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13EfRec1Setup, ".testPkts")
#endif

t13EfRec1Setup_t  t13EfRec1Setup[] = {
    /* Entry 0 */
    {
        0,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,

            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x0b,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},               /* dstIp */
            0,                                                               /* flow */
            1,                                                               /* tos */
            38,                                                              /* mtu */
            0xbbaa,                                                          /* srcPort */
            0                                                                /* dstPort */
        }

    },

    /* Entry 1 */
    {
        10,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_DST     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_DST_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |           /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM
            ,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0xc8, 0xc9, 0xca, 0x64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* dstIp */
            0,                                                               /* flow */
            4,                                                               /* tos */
            200,                                                             /* mtu */
            0xbbaa,                                                          /* srcPort */
            0x8002                                                           /* dstPort */
        }

    },

    /* Entry 2 */
    {
        240,       /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x22,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},               /* dstIp */
            0,                                                               /* flow */
            2,                                                               /* tos */
            300,                                                             /* mtu */
            0xccdd,                                                          /* srcPort */
            0                                                                /* dstPort */
        }

    },

    /* Entry 3 */
    {
        254,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_DST     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_DST_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |         /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_TCP_CTRL     |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_OPTION    |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT  |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE
            ,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0xc8, 0xc9, 0xca, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* dstIp */
            0,                                                               /* flow */
            3,                                                               /* tos */
            512,                                                             /* mtu */
            0xbbaa,                                                          /* srcPort */
            0x9002                                                           /* dstPort */
        }
    },

    /* Entry 4 */
    {
        20,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_FLOW_LABEL |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,

            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV6,
            { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10},  /* srcIp */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},               /* dstIp */
            0x00012345,                                                      /* flow */
            5,                                                               /* tos */
            80,                                                              /* mtu */
            0xbeef,                                                          /* srcPort */
            0                                                                /* dstPort */
        }

    },

    /* Entry 5 */
    {
        50,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_DST     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_FLOW_LABEL |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_DST_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM            /* ctrlBitMap */
            ,
            pa_IPV6,
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x38 },  /* IP Destination address */
            0x00020202,                                                      /* flow */
            2,                                                               /* tos */
            200,                                                             /* mtu */
            0xbbaa,                                                          /* srcPort */
            0x8102                                                           /* dstPort */
        }

    },

    /* Entry 6 */
    {
        100,       /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV6,
            { 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
              0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20},  /* srcIp */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},               /* dstIp */
            0,                                                               /* flow */
            0x11,                                                            /* tos */
            300,                                                             /* mtu */
            0xccdd,                                                          /* srcPort */
            0                                                                /* dstPort */
        }

    },

    /* Entry 7 */
    {
        150,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_DST     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_IP_MTU     |
            pa_EF_LVL1_RECORD_VALID_DST_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM         |  /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_TCP_CTRL     |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_OPTION    |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT  |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE
            ,
            pa_IPV6,
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            {
	            0x00, 0x02, 0x04, 0x08,
	            0x10, 0x12, 0x14, 0x18,
	            0x20, 0x22, 0x24, 0x28,
	            0x30, 0x32, 0x34, 0x3a
            },  /* dstIp */
            0,                                                               /* flow */
            0x33,                                                            /* tos */
            512,                                                             /* mtu */
            0xbbaa,                                                          /* srcPort */
            0x9102                                                           /* dstPort */
        }
    },

    /* Entry 8 */
    {
        88,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,

            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x88,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},               /* dstIp */
            0,                                                               /* flow */
            0x08,                                                            /* tos */
            0,                                                               /* mtu */
            0xabab,                                                          /* srcPort */
            0                                                                /* dstPort */
        }

    },

    /* Entry 9 */
    {
        29,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_FLOW_LABEL |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,

            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV6,
            { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x18},  /* srcIp */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},               /* dstIp */
            0x00054321,                                                      /* flow */
            0x0f,                                                            /* tos */
            0,                                                               /* mtu */
            0xbabe,                                                          /* srcPort */
            0                                                                /* dstPort */
        }
    },

    /* Entry 10 */
    {
        250,       /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_IP_DST     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x22,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0xc8, 0xc9, 0xca, 0x66,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* dstIp */
            0,                                                               /* flow */
            3,                                                               /* tos */
            0,                                                               /* mtu */
            0xeeff,                                                          /* srcPort */
            0                                                                /* dstPort */
        }
    },

    /* Entry 11 */
    {
        70,       /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_SRC     |
            pa_EF_LVL1_RECORD_VALID_TOS_CLASS  |
            pa_EF_LVL1_RECORD_VALID_SRC_PORT,


            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV4,
            { 0x9e, 0xda, 0x6d, 0x22,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0xc8, 0xc9, 0xca, 0x66,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* dstIp */
            0,                                                               /* flow */
            7,                                                               /* tos */
            0,                                                               /* mtu */
            0xbeef,                                                          /* srcPort */
            0                                                                /* dstPort */
        }
    },

    /* Entry 12 */
    {
        75,         /* Record Index */
        {
            pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL1_RECORD_VALID_IP_DST,

            pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM |          /* ctrlBitMap */
            pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM |
            pa_EF_LVL1_RECORD_CONTROL_REMOVE_OUTER_IP_HDR_TRAIL |
            pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC,
            pa_IPV4,
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* srcIp */
            { 0xc8, 0xc9, 0xca, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* dstIp */
            0,                                                               /* flow */
            0,                                                               /* tos */
            0,                                                               /* mtu */
            0,                                                               /* srcPort */
            0                                                                /* dstPort */
        }

    },


};

uint8_t t13ipHdr1[] = {
    0x45, 0x10, 0x00, 0x62, 0x00, 0x00, 0x00, 0x00,
    0x05, 0x04, 0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20,
    0xc8, 0xc9, 0xca, 0x65 };

uint8_t t13ipHdr2[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39
    };

uint8_t t13ipHdr3[] = {
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00, 0x00, 0x00,
    0x05, 0x04, 0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20,
    0xc8, 0xc9, 0xca, 0xc8 };


typedef struct t13EfRec2Setup_s  {
    int             index;
	paEfRecLevel2_t rec;		/* Level 2 record */
} t13EfRec2Setup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13EfRec2Setup, ".testPkts")
#endif
t13EfRec2Setup_t  t13EfRec2Setup[] = {

    /* Entry 0 */
    {
        20,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS,                /* validBitMap */


            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC,            /* ctrlBitMap */
            0,                                                 /* mtu */
            sizeof(t13ipHdr1),                                 /* ipHdrSize */
            t13ipHdr1,                                         /* ipHdr */
            {                                                  /* ipSec */
                0,                                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                0,                                  /* macSize */
                0,                                  /* flowId */
                0,                                  /* queueId */
                0,                                  /* spi */
                0,                                  /* saInfo0 */
                0                                   /* saInfo1 */
            }
        }
    },

    /* Entry 1 */
    {
        40,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS                 /* validBitMap */
            ,

            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC        |    /* ctrlBitMap */
            pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_OPTION  |
            pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT|
            pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE
            ,
            0,                                                 /* mtu */
            0,                                                 /* ipHdrSize */
            0,                                                 /* ipHdr */
            {                                                  /* ipSec */
                0,                                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                0,                                  /* macSize */
                0,                                  /* flowId */
                0,                                  /* queueId */
                0,                                  /* spi */
                0,                                  /* saInfo0 */
                0                                   /* saInfo1 */
            }
        }
    },

    /* Entry 2 */
    {
        80,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS                 /* validBitMap */
            ,


            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC,            /* ctrlBitMap */
            0,                                                 /* mtu */
            sizeof(t13ipHdr2),                                 /* ipHdrSize */
            t13ipHdr2,                                         /* ipHdr */
            {                                                  /* ipSec */
                0,                                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                0,                                  /* macSize */
                0,                                  /* flowId */
                0,                                  /* queueId */
                0,                                  /* spi */
                0,                                  /* saInfo0 */
                0                                   /* saInfo1 */
            }
        }
    },

    /* Entry 3 */
    {
        160,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS                 /* validBitMap */
            ,

            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC        |    /* ctrlBitMap */
            pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_OPTION  |
            pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT|
            pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE
            ,
            0,                                                 /* mtu */
            0,                                                 /* ipHdrSize */
            0,                                                 /* ipHdr */
            {                                                  /* ipSec */
                0,                                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                0,                                  /* macSize */
                0,                                  /* flowId */
                0,                                  /* queueId */
                0,                                  /* spi */
                0,                                  /* saInfo0 */
                0                                   /* saInfo1 */
            }
        }
    },

    /* Entry 4 */
    {
        38,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL2_RECORD_VALID_IP_MTU     |
            pa_EF_LVL2_RECORD_VALID_IPSEC,


            pa_EF_LVL2_RECORD_CONTROL_SINGLE_IP    |
            pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
            ,            /* ctrlBitMap */
            400,                                               /* mtu */
            0,                                                 /* ipHdrSize */
            0,                                                 /* ipHdr */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_AH,                   /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                12,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS2,                /* queueId */
                0x44440000,                         /* spi */
                0x66660008,                         /* saInfo0 */
                0x0c000000                          /* saInfo1 */
            }
        }
    },

    /* Entry 5 */
    {
        49,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL2_RECORD_VALID_IP_MTU     |
            pa_EF_LVL2_RECORD_VALID_IPSEC,


            pa_EF_LVL2_RECORD_CONTROL_SINGLE_IP    |
            pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
            ,            /* ctrlBitMap */
            500,                                               /* mtu */
            0,                                                 /* ipHdrSize */
            0,                                                 /* ipHdr */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_ESP,                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                4,                                  /* encBlkSize */
                8,                                  /* ivSize */
                12,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS2,                /* queueId */
                0x55550000,                         /* spi */
                0x66660009,                         /* saInfo0 */
                0x0c000100                          /* saInfo1 */
            }
        }
    },

    /* Entry 6 */
    {
        100,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL2_RECORD_VALID_IP_MTU     |
            pa_EF_LVL2_RECORD_VALID_IPSEC,


            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC |
            pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
            ,            /* ctrlBitMap */
            256,                                               /* mtu */
            sizeof(t13ipHdr1),                                 /* ipHdrSize */
            t13ipHdr1,                                         /* ipHdr */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_ESP,                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                4,                                  /* encBlkSize */
                8,                                  /* ivSize */
                12,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS2,                /* queueId */
                0x55550001,                         /* spi */
                0x6666000a,                         /* saInfo0 */
                0x0c000200                          /* saInfo1 */
            }
        }
    },

    /* Entry 7 */
    {
        110,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL2_RECORD_VALID_IP_MTU     |
            pa_EF_LVL2_RECORD_VALID_IPSEC,


            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC |
            pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
            ,            /* ctrlBitMap */
            256,                                               /* mtu */
            sizeof(t13ipHdr3),                                 /* ipHdrSize */
            t13ipHdr3,                                         /* ipHdr */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_ESP,                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                16,                                 /* encBlkSize */
                8,                                  /* ivSize */
                12,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS1,                /* queueId */
                0x55550003,                         /* spi */
                0x6666000c,                         /* saInfo0 */
                0x0c000300                          /* saInfo1 */
            }
        }
    },

    /* Entry 8 */
    {
        120,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL2_RECORD_VALID_IP_MTU     |
            pa_EF_LVL2_RECORD_VALID_IPSEC,


            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC |
            pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
            ,            /* ctrlBitMap */
            256,                                               /* mtu */
            sizeof(t13ipHdr2),                                 /* ipHdrSize */
            t13ipHdr2,                                         /* ipHdr */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_AH,                   /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                16,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS2,                /* queueId */
                0x44440001,                         /* spi */
                0x6666000b,                         /* saInfo0 */
                0x0c000400                          /* saInfo1 */
            }
        }
    },

    /* Entry 9 */
    {
        130,         /* Record Index */
        {
            pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL2_RECORD_VALID_IP_MTU     |
            pa_EF_LVL2_RECORD_VALID_IPSEC,


            pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC |
            pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
            ,            /* ctrlBitMap */
            256,                                                 /* mtu */
            sizeof(t13ipHdr2),                                 /* ipHdrSize */
            t13ipHdr2,                                         /* ipHdr */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_ESP,                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                4,                                  /* encBlkSize */
                8,                                  /* ivSize */
                12,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS1,                /* queueId */
                0x55550002,                         /* spi */
                0x6666000d,                         /* saInfo0 */
                0x0c000500                          /* saInfo1 */
            }
        }
    }

};

typedef struct t13EfRec3Setup_s  {
    int             index;
	paEfRecLevel3_t rec;		/* Level 3 record */
} t13EfRec3Setup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13EfRec3Setup, ".testPkts")
#endif
t13EfRec3Setup_t  t13EfRec3Setup[] = {

    /* Entry 0 (NAT-T) */
    {
        33,         /* Record Index */
        {
            pa_EF_LVL3_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL3_RECORD_VALID_IP_MTU,
            0,                                               /* ctrlBitMap */
            256,                                              /* mtu */
             500,                                              /* srcPort */
            4500,                                              /* dstPort */
            {                                                  /* ipSec */
                0,                                  /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                0,                                  /* macSize */
                0,                                  /* flowId */
                0,                                  /* queueId */
                0,                                  /* spi */
                0,                                  /* saInfo0 */
                0                                   /* saInfo1 */
            }
        }
    },

    /* Entry 1 */
    {
        66,         /* Record Index */
        {
            pa_EF_LVL3_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL3_RECORD_VALID_IPSEC      |
            pa_EF_LVL3_RECORD_VALID_IP_MTU,
            0,                                                 /* ctrlBitMap */
            256,                                               /* mtu */
            0,                                                 /* srcPort */
            0,                                                 /* dstPort */
            {                                                  /* ipSec */
                pa_IPSEC_PROTO_AH,                   /*ipsecMode */
                0,                                  /* ctrlBitMap */
                0,                                  /* encBlkSize */
                0,                                  /* ivSize */
                12,                                 /* macSize */
                0,                                  /* flowId */
                TF_PA_QUEUE_EGRESS2,                /* queueId */
                0x44440002,                         /* spi */
                0x6666000d,                         /* saInfo0 */
                0x0c001000                          /* saInfo1 */
            }
        }
    }

};

uint8_t t13l2Hdr1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x88, 0x64, 0x11, 0x00, 0x12, 0x34,
    0x00, 0x00, 0x00, 0x21
    };

uint8_t t13l2Hdr2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x08, 0x00
    };

uint8_t t13l2Hdr3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x88, 0x64, 0x11, 0x00, 0x12, 0x34,
    0x00, 0x00, 0x00, 0x57,
    };

uint8_t t13l2Hdr4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
	0x86, 0xdd,
    };

typedef struct t13EfRec4Setup_s  {
    int             index;
	paEfRecLevel4_t rec;		/* Level 4 record */
} t13EfRec4Setup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13EfRec4Setup, ".testPkts")
#endif
t13EfRec4Setup_t  t13EfRec4Setup[] = {

    /* Entry 0 */
    {
        1,         /* Record Index */
        {
            pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL4_RECORD_VALID_PPPoE      |
            pa_EF_LVL4_RECORD_VALID_MIN_PKTSIZE,
            0,                                                 /* ctrlBitMap */
            14,                                                /* lenOffsetPPPoE */
            0,                                                 /* lenOffset802p3 */
            0,                                                 /* vlan1 */
            0,                                                 /* vlan2 */
            sizeof(t13l2Hdr1),                                 /* l2HdrSize */
            t13l2Hdr1,                                         /* l2Hdr */
            60,                                                /* minPktSize */
#ifndef T13_TEST_EFLOW_DEST_EMAC
            pa_DEST_HOST,                                      /* dest */
            0,                                                 /* flowId */
            pa_EMAC_PORT_0,                                    /* pktType_emacCtrl */
            0x12345678,                                        /* swInfo0 */
            0,                                                 /* swInfo1 */
            TF_PA_QUEUE_INPUT,                                 /* queueId */
#else
            pa_DEST_EMAC,                                      /* dest */
            0,                                                 /* flowId */
            pa_EMAC_PORT_0,                                    /* pktType_emacCtrl */
            0x12345678,                                        /* swInfo0 */
            0,                                                 /* swInfo1 */
            0,                                                 /* queueId */
#endif
            0                                                  /* priorityType */
        }
    },

    /* Entry 1 */
    {
        11,         /* Record Index */
        {
            pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL4_RECORD_VALID_MIN_PKTSIZE,
            0,                                                 /* ctrlBitMap */
            0,                                                 /* lenOffsetPPPoE */
            0,                                                 /* lenOffset802p3 */
            0,                                                 /* vlan1 */
            0,                                                 /* vlan2 */
            sizeof(t13l2Hdr2),                                 /* l2HdrSize */
            t13l2Hdr2,                                         /* l2Hdr */
            60,                                                /* minPktSize */
#ifndef T13_TEST_EFLOW_DEST_EMAC
            pa_DEST_HOST,                                      /* dest */
            0,                                                 /* flowId */
            pa_EMAC_PORT_1,                                    /* pktType_emacCtrl */
            0x87654321,                                        /* swInfo0 */
            0,                                                 /* swInfo1 */
            TF_PA_QUEUE_INPUT,                                 /* queueId */
#else
            pa_DEST_EMAC,                                      /* dest */
            0,                                                 /* flowId */
            pa_EMAC_PORT_1,                                    /* pktType_emacCtrl */
            0x12345678,                                        /* swInfo0 */
            0,                                                 /* swInfo1 */
            0,                                                 /* queueId */
#endif
            0                                                  /* priorityType */
        }
    },
    /* Entry 2 */
    {
        101,        /* Record Index */
        {
            pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS |               /* validBitMap */
            pa_EF_LVL4_RECORD_VALID_PPPoE,
            0,                                                 /* ctrlBitMap */
            14,                                                /* lenOffsetPPPoE */
            0,                                                 /* lenOffset802p3 */
            0,                                                 /* vlan1 */
            0,                                                 /* vlan2 */
            sizeof(t13l2Hdr3),                                 /* l2HdrSize */
            t13l2Hdr3,                                         /* l2Hdr */
            0,                                                 /* minPktSize */
            pa_DEST_HOST,                                      /* dest */
            0,                                                 /* flowId */
            0,                                                 /* pktType_emacCtrl */
            0xbabeface,                                        /* swInfo0 */
            0,                                                 /* swInfo1 */
            TF_PA_QUEUE_INPUT,                                 /* queueId */
            0                                                  /* priorityType */
        }
    },

    /* Entry 3 */
    {
        111,        /* Record Index */
        {
            pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS,                /* validBitMap */
            0,                                                 /* ctrlBitMap */
            0,                                                /* lenOffsetPPPoE */
            0,                                                 /* lenOffset802p3 */
            0,                                                 /* vlan1 */
            0,                                                 /* vlan2 */
            sizeof(t13l2Hdr4),                                 /* l2HdrSize */
            t13l2Hdr4,                                         /* l2Hdr */
            0,                                                 /* minPktSize */
            pa_DEST_HOST,                                      /* dest */
            0,                                                 /* flowId */
            0,                                                 /* pktType_emacCtrl */
            0xfacebabe,                                        /* swInfo0 */
            0,                                                 /* swInfo1 */
            TF_PA_QUEUE_INPUT,                                 /* queueId */
            0                                                  /* priorityType */
        }
    },

    /* Entry 4 */
    {
        201,     /* Record Index */
        {
            pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS |              /* validBitMap */
            pa_REF_LVL4_RECORD_VALID_ROUTE_PRIORITY_TYPE,
            0,                                                /* ctrlBitMap */
            0,                                                /* lenOffsetPPPoE */
            0,                                                /* lenOffset802p3 */
            0,                                                /* vlan1 */
            0,                                                /* vlan2 */
            0,                                                /* l2HdrSize */
            NULL,                                             /* l2Hdr */
            0,                                                /* minPktSize */
            pa_DEST_HOST,                                     /* dest */
            0,                                                /* flowId */
            0,                                                /* pktType_emacCtrl */
            T13_SWINFO0_QOS_PKT_ID,                           /* swInfo0 */
            0,                                                /* swInfo1 */
            Q_QoS_BASE + TF_FIRST_GEN_QUEUE,                  /* queueId */
            pa_ROUTE_PRIORITY_DSCP                            /* priorityType */
        }
    },

    /* Entry 5 */
    {
        211,     /* Record Index */
        {
            pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS |              /* validBitMap */
            pa_REF_LVL4_RECORD_VALID_ROUTE_PRIORITY_TYPE,
            0,                                                /* ctrlBitMap */
            0,                                                /* lenOffsetPPPoE */
            0,                                                /* lenOffset802p3 */
            0,                                                /* vlan1 */
            0,                                                /* vlan2 */
            0,                                                /* l2HdrSize */
            NULL,                                             /* l2Hdr */
            0,                                                /* minPktSize */
            pa_DEST_HOST,                                     /* dest */
            0,                                                /* flowId */
            0,                                                /* pktType_emacCtrl */
            T13_SWINFO0_QOS_PKT_ID + 1,                       /* swInfo0 */
            0,                                                /* swInfo1 */
            Q_QoS_BASE + TF_FIRST_GEN_QUEUE,                  /* queueId */
            pa_ROUTE_PRIORITY_VLAN                            /* priorityType */
        }
    }
};

typedef struct t13FcSetup_s  {
	int             seqId;		/* Sequential enumeration to identify */
	int		        handleIdx;	/* Local handle index. Specifies which local handle corresponds to this entry */
    int             index;      /* Lut index */
	paFcInfo_t      fcInfo;	    /* PA FC configuration structure */
    paEfOpInfo_t    efOpInfo;   /* FC routing info */
	paReturn_t      ret;		/* Expected return code from pa_addIp command */
	Bool	        acked;		/* Set to TRUE when the reply to the command is received */
} t13FcSetup_t;

/* FC configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t13FcSetup, ".testPkts")
#endif
static t13FcSetup_t t13FcSetup[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		0,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_PROTO      |
            pa_FC_INFO_VALID_DSCP       |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x0b, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			17,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x8000,     /* destPortbegin */
            0x8000,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL4,
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            1                                   /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 1 ----------- */
	{
		1,			/* Sequential ID */
		1,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_PROTO      |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			17,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x8002,     /* destPortbegin */
            0x8002,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL4,
            10,                                 /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 2 ----------- */
	{
		2,			/* Sequential ID */
		2,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x9000,     /* destPortbegin */
            0x9000,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            240,                                /* lvl1Index */
            20,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 3 ----------- */
	{
		3,			/* Sequential ID */
		3,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x9002,     /* destPortbegin */
            0x9002,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 4 ----------- */
	{
		4,			/* Sequential ID */
		4,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			    /* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_PROTO      |
            pa_FC_INFO_VALID_DSCP       |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x38
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			17,			/* Protocol */
			0,			/* DSCP/Class */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x8100,     /* destPortbegin */
            0x8100,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL4,
            20,                                 /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            101                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 5 ----------- */
	{
		5,			/* Sequential ID */
		5,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_PROTO      |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x01
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{
	          0x00, 0x02, 0x04, 0x08,
	          0x10, 0x12, 0x14, 0x18,
	          0x20, 0x22, 0x24, 0x28,
	          0x30, 0x32, 0x34, 0x38
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			17,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x8102,     /* destPortbegin */
            0x8102,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL4,
            50,                                 /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 6 ----------- */
	{
		6,			/* Sequential ID */
		6,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x03
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{
	          0x00, 0x02, 0x04, 0x08,
	          0x10, 0x12, 0x14, 0x18,
	          0x20, 0x22, 0x24, 0x28,
	          0x30, 0x32, 0x34, 0x3a
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x9100,     /* destPortbegin */
            0x9100,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            100,                                /* lvl1Index */
            80,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 7 ----------- */
	{
		7,			/* Sequential ID */
		7,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x05
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{
	          0x00, 0x02, 0x04, 0x08,
	          0x10, 0x12, 0x14, 0x18,
	          0x20, 0x22, 0x24, 0x28,
	          0x30, 0x32, 0x34, 0x3a
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x9102,     /* destPortbegin */
            0x9102,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 8----------- */
	{
		8,			/* Sequential ID */
		8,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_PROTO      |
            pa_FC_INFO_VALID_DSCP       |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x0b, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			17,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0xa000,     /* destPortbegin */
            0xa000,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            88,                                 /* lvl1Index */
            38,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            1                                   /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 9 ----------- */
	{
		9,			/* Sequential ID */
		9,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			    /* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_PROTO      |
            pa_FC_INFO_VALID_DSCP       |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0x00, 0x02, 0x04, 0x08,
			  0x10, 0x12, 0x14, 0x18,
			  0x20, 0x22, 0x24, 0x28,
			  0x30, 0x32, 0x34, 0x38
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			17,			/* Protocol */
			0,			/* DSCP/Class */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0xa002,     /* destPortbegin */
            0xa002,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            29,                                 /* lvl1Index */
            49,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            101                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 10 ----------- */
	{
		10,			/* Sequential ID */
		10,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0xb000,     /* destPortbegin */
            0xb000,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            240,                                /* lvl1Index */
            100,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 11 ----------- */
	{
		11,			/* Sequential ID */
		11,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x03
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{
	          0x00, 0x02, 0x04, 0x08,
	          0x10, 0x12, 0x14, 0x18,
	          0x20, 0x22, 0x24, 0x28,
	          0x30, 0x32, 0x34, 0x3a
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0xb002,     /* destPortbegin */
            0xb002,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            100,                                /* lvl1Index */
            120,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 12 ----------- */
	{
		12,			/* Sequential ID */
		12,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0xc9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0xc000,     /* destPortbegin */
            0xc000,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL3   |
            pa_EF_OP_INFO_VALID_LVL4,
            240,                                /* lvl1Index */
            110,                                /* lvl2Index */
            33,                                 /* lvl3Index */
            11                                  /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},


	/* ------- Entry 13 ----------- */
	{
		13,			/* Sequential ID */
		13,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV6,	/* IP Type */
			{
	          0x20, 0x02, 0x9e, 0xda,
	          0x6d, 0x30, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x00,
	          0x00, 0x00, 0x00, 0x03
            },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{
	          0x00, 0x02, 0x04, 0x08,
	          0x10, 0x12, 0x14, 0x18,
	          0x20, 0x22, 0x24, 0x28,
	          0x30, 0x32, 0x34, 0x3a
            },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0xc002,     /* destPortbegin */
            0xc002,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL3   |
            pa_EF_OP_INFO_VALID_LVL4,
            100,                                /* lvl1Index */
            130,                                /* lvl2Index */
            66,                                 /* lvl3Index */
            111                                 /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 14 ----------- */
	{
		14,			/* Sequential ID */
		14,			/* Local Handle Index */
		pa_LUT1_INDEX_NOT_SPECIFIED,			/* Lut1 Index */
		{
            pa_FC_INFO_VALID_SRC_IP     |                         /* validbitMap */
            pa_FC_INFO_VALID_DST_IP     |
            pa_FC_INFO_VALID_SRC_PORT   |
            pa_FC_INFO_VALID_DST_PORT,
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0x9e, 0xda, 0x6d, 0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0xc8, 0xc9, 0xca, 0x95, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0xaabb,     /* srcPortbegin */
            0xaabb,     /* srcPort end */
            0x9008,     /* destPortbegin */
            0x9008,     /* destPort end */
		},
        {
            0,                                  /* ctrlFlags */
            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
            pa_EF_OP_INFO_VALID_LVL2   |
            pa_EF_OP_INFO_VALID_LVL4,
            250,                                /* lvl1Index */
            20,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	}
};


/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * PPPoE header for IPv4
 * ip src  = 158.218.109.11
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
    0x12, 0x34, 0x00, 0x00, 0x00, 0x21,
    #endif
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x00, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,42,50,0),    	            /* L3 offset = 22, l4Offset = 42, l5Offset = 50, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4| PASAHO_HDR_BITMASK_PPPoE | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 1
 * mac dest = 00:01:02:03:04:aa
 * ip src  = 158.218.109.12
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0c, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x02, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 2
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    #endif
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x00, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 3
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3, ".testPkts")
#endif
static uint8_t pkt3[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};

/* packet 4
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x00
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x38
 * Designed to match IP configuration 3 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4, ".testPkts")
#endif
static uint8_t pkt4[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x88, 0x64, 0x11, 0x00, 0x12, 0x34,
    0x00, 0x00, 0x00, 0x57,
    #endif
	0x60, 0x01, 0x01, 0x01,                 /* IPv6 hedaer */
	0x00, 0x00, 0x00, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x38,

    0x11, 0x01, 0x04, 0x08,                /* Hop-by-hop header */
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x38,

	0xaa, 0xbb, 0x81, 0x00,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,86),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,78,86,0),    	            /* L3 offset = 22, l4Offset = 78, l5Offset = 86, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_PPPoE | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 5
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x01
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x38
 * Designed to match IP configuration 3 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5, ".testPkts")
#endif
static uint8_t pkt5[] = {
#if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
	0x86, 0xdd,
#endif
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x2b, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x01,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x38,

    0x11, 0x01, 0x01, 0x00,      // Extension Route header
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x38,

	0xaa, 0xbb, 0x81, 0x02,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,78),		        /* cmd len = 14, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,70,78,0),    	            /* L3 offset = 14, l4Offset = 70, l5Offset = 78, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 6
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x02
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x03
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 * Designed to match IP configuration 5 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6, ".testPkts")
#endif
static  uint8_t pkt6[] = {
#if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
	0x86, 0xdd,
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x02,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,
#endif

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x03,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x00,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,102),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,94,102,0),    	        /* L3 offset = 14, l4Offset = 94, l5Offset = 102, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 7
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 * Designed to match IP configuration 5 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7, ".testPkts")
#endif
static uint8_t pkt7[] = {
#if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
	0x86, 0xdd,
#endif
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,102),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,94,102,0),    	        /* L3 offset = 14, l4Offset = 94, l5Offset = 102, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 8
 * mac dest = 00:01:02:03:04:aa
 * PPPoE header for IPv4
 * ip src  = 158.218.109.11
 * ip dest = 200.201.202.100
 * AH:0x44440000, icvSize = 12
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8, ".testPkts")
#endif
static uint8_t pkt8[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
    0x12, 0x34, 0x00, 0x00, 0x00, 0x21,
    #endif
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0xa0, 0x00, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt8Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,74),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,66,74,0),    	            /* L3 offset = 22, l4Offset = 66, l5Offset = 74, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4| PASAHO_HDR_BITMASK_PPPoE | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_AH),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 9
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x00
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x38
 * ESP: 0x55550000, UDP:a002
 * Designed to match IP configuration 3 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9, ".testPkts")
#endif
static uint8_t pkt9[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x88, 0x64, 0x11, 0x00, 0x12, 0x34,
    0x00, 0x00, 0x00, 0x57,
    #endif
	0x60, 0x01, 0x01, 0x01,                 /* IPv6 hedaer */
	0x00, 0x00, 0x00, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x38,

    0x11, 0x01, 0x04, 0x08,                /* Hop-by-hop header */
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x38,

	0xaa, 0xbb, 0xA0, 0x02,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,102),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,94,102,0),    	        /* L3 offset = 22, l4Offset = 78, l5Offset = 86, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_PPPoE | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_ESP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 10
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 * SPI:0x55550001, ESP, UDP 0xb000
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10, ".testPkts")
#endif
static uint8_t pkt10[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    #endif
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0xb0, 0x00, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,78),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,70,78,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_ESP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 11
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x02
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x03
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 * SPI (AH):0x44440001, UDP: 0xb002
 * Designed to match IP configuration 5 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11, ".testPkts")
#endif
static  uint8_t pkt11[] = {
#if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
	0x86, 0xdd,
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x02,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,
#endif

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x03,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0xb0, 0x02,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,134),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,126,134,0),    	        /* L3 offset = 14, l4Offset = 94, l5Offset = 102, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_AH),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 12
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 * IPSEC/NAT-T/ESP
 * SPI:0x55550003, ESP, UDP 0xc000
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt12, ".testPkts")
#endif
static uint8_t pkt12[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    #endif
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0xc9, 0xaa, 0xbb,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt12Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,86),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,78,86,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 13
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x02
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x03
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 * IPSEC?AH/ESP
 * SPI (AH):0x44440002, ESP:0x55550002, UDP: 0xc002
 * Designed to match IP configuration 5 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt13, ".testPkts")
#endif
static  uint8_t pkt13[] = {
#if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
	0x86, 0xdd,
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x02,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,
#endif

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x00, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x03,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0xc0, 0x02,
	0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt13Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,142),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,134,142,0),    	        /* L3 offset = 14, l4Offset = 94, l5Offset = 102, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_AH | PASAHO_HDR_BITMASK_ESP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

typedef struct oipFragInfo_s
{
    Bool    singleIp;
    int     mtuSize;      /* MTU: 0, disable */
    uint8_t hdrSize;
    uint8_t numExtraBytes; /* contain IPSEC header and trail */
}  oipFragInfo_t;

typedef struct pktTest13Info_s
{
    pktTestInfo_t   pktInfo;
    paCmdEfOp_t     cmdInfo;
    int             numIpsecHdr;
    int             l4Offset;
    Bool            innerIp;
    Bool            ipv6;
    Bool            natT;
    oipFragInfo_t   oipInfo;
} pktTest13Info_t;


#ifdef _TMS320C6X
#pragma DATA_SECTION (t13PktInfo, ".testPkts")
#endif
static pktTest13Info_t t13PktInfo[] =  {

	/* Packet 0 */
    {
	    {
		    (uint8_t *)pkt0,
		    (pasahoLongInfo_t *)&pkt0Info,
		    sizeof(pkt0),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 0		/* Packet will be matched by packet index 0 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt0),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        0,
        20,
        FALSE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 1 */
    {
	    {
		    (uint8_t *)pkt1,
		    (pasahoLongInfo_t *)&pkt1Info,
		    sizeof(pkt1),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 1		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt1),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        0,
        20,
        FALSE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 2 */
    {
	    {
		    (uint8_t *)pkt2,
		    (pasahoLongInfo_t *)&pkt2Info,
		    sizeof(pkt2),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 2		/* Packet will be matched by packet index 2 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt2),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        0,
        20,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 3 */
    {
	    {
		    (uint8_t *)pkt3,
		    (pasahoLongInfo_t *)&pkt3Info,
		    sizeof(pkt3),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 3		/* Packet will be matched by packet index 3 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt3),                       /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        40,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 4 */
    {
	    {
		    (uint8_t *)pkt4,
		    (pasahoLongInfo_t *)&pkt4Info,
		    sizeof(pkt4),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 4		/* Packet will be matched by packet index 4 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt4),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        0,
        56,
        FALSE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 5 */
    {
	    {
		    (uint8_t *)pkt5,
		    (pasahoLongInfo_t *)&pkt5Info,
		    sizeof(pkt5),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 5		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt5),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        0,
        56,
        FALSE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
	},

	/* Packet 6 */
    {
	    {
		    (uint8_t *)pkt6,
		    (pasahoLongInfo_t *)&pkt6Info,
		    sizeof(pkt6),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)  | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6)},	/* IP/UDP match */															  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 6		/* Packet will be matched by packet index 2 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt6),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        0,
        40,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 7 */
    {
	    {
		    (uint8_t *)pkt7,
		    (pasahoLongInfo_t *)&pkt7Info,
		    sizeof(pkt7),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6)},	/* IP/UDP match */															  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 3		/* Packet will be matched by packet index 3 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            40,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt7),                       /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        80,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 8 */
    {
	    {
		    (uint8_t *)pkt8,
		    (pasahoLongInfo_t *)&pkt8Info,
		    sizeof(pkt8),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 8		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt8),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        1,
        20,
        FALSE,
        FALSE,
        FALSE,
        {
            TRUE,
            400,
            20,
            24
        }
	},

	/* Packet 9 */
    {
	    {
		    (uint8_t *)pkt9,
		    (pasahoLongInfo_t *)&pkt9Info,
		    sizeof(pkt9),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 9		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt9),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        1,
        56,
        FALSE,
        TRUE,
        FALSE,
        {
            TRUE,
            500,
            56,
            32
        }
	},

	/* Packet 10 */
    {
	    {
		    (uint8_t *)pkt10,
		    (pasahoLongInfo_t *)&pkt10Info,
		    sizeof(pkt10),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)}, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 10		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt10),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        1,
        20,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            256,
            20,
            32
        }
	},

	/* Packet 11 */
    {
	    {
		    (uint8_t *)pkt11,
		    (pasahoLongInfo_t *)&pkt11Info,
		    sizeof(pkt11),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 11		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt11),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        1,
        40,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            256,
            40,
            32
        }
	},

	/* Packet 12 */
    {
	    {
		    (uint8_t *)pkt12,
		    (pasahoLongInfo_t *)&pkt12Info,
		    sizeof(pkt12),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C2_NUM_PACKETS),  	/* IP match (NAT-T)*/
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 12		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt12),                       /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        1,
        20,
        TRUE,
        FALSE,
        TRUE,
        {
            0,
            256,
            20,
            48
        }
	},

	/* Packet 13 */
    {
	    {
		    (uint8_t *)pkt13,
		    (pasahoLongInfo_t *)&pkt13Info,
		    sizeof(pkt13),
		    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),  	/* IP match */
		    (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6) }, 								  		/* UDP match */
		    T13_SWINFO0_PKT_ID | 13		/* Packet will be matched by packet index 1 */
	    },

        {
            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlBitfield */
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            0,                                  /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(pkt13),                      /* endOffset */
            0,                                  /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            0                                   /* lvl4Index */
        },
        2,
        40,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            256,
            40,
            56
        }
	},

};

/* IP Forwarding test case
 * Step 1: The ingess packet matches IP forwarding rule
 * Step 2: Flow cache and/or Egress operation per IP forwarding rule
 * Step 3: Perform host-terminated classification on the re-entry packets
 */

/* Ingress packet 1
 * Ingress L2 forwarding with L2 replacement
 * mac dest = 00:01:02:03:04:aa
 * ip src  = 158.218.109.12
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt1, ".testPkts")
#endif
static uint8_t inpkt1[] = {
    #if 1
    // Ingress L2 Header (first input)
    0x00, 0x01, 0x02, 0x03, 0x04, 0xbb, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x05, 0x08, 0x00,
    #else
    // Egress L2 header (final input)
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0c, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x04, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt1Info, ".testPkts")
#endif
static pasahoLongInfo_t inpkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 2
 * Ingess outer IP forwarding with L2 and outer IP insertion
 * mac dest = 00:01:02:03:04:aa
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt2, ".testPkts")
#endif
static uint8_t inpkt2[] = {
    #if 1
    //Ingress L2/L3 Header (first input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xcc, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #else
    // Egress L2/L3 header (final input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    #endif
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x04, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt2Info, ".testPkts")
#endif
static pasahoLongInfo_t inpkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 3
 * Ingess inner IP forwarding with L2 and outer IP replacements
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt3, ".testPkts")
#endif
static uint8_t inpkt3[] = {
    #if 1
    // Ingress L2/L3 Header (first input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xcc, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x06, 0x08, 0x00,
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x75,
    #else
    // Egress L2/L3 header (final input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65,
    #endif
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x06, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt3Info, ".testPkts")
#endif
static pasahoLongInfo_t inpkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};


/* packet 4
 * Ingess L4 forwarding with Flow cache, inner IP updates, L2 and outer IP replacement
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt4, ".testPkts")
#endif
static uint8_t inpkt4[] = {
    #if 1
    //Ingress L2/L3 Header (first input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xcc, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x75,
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x95,
    #else
    // Egress L2/L3 header (final input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66,
    #endif
    0xaa, 0xbb, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt4Info, ".testPkts")
#endif
static pasahoLongInfo_t inpkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 5
 * Ingess outer IP forwarding with L2 and outer IP removal at EFLOW1 and insert at EFLOW2
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt5, ".testPkts")
#endif
static uint8_t inpkt5[] = {
    #if 1
    // Ingress L2/L3 Header (first input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xcc, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x06, 0x08, 0x00,
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x75,
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x67,
    #else
    // Egress L2/L3 header (final input)
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65,
    0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66,
    #endif
    0xaa, 0xbb,
    0x90, 0x0a, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (inpkt5Info, ".testPkts")
#endif
static pasahoLongInfo_t inpkt5Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

#define T13_FIRST_IP_FWD_ENTRY                  14

typedef struct pktTest13Info2_s
{
    pktTestInfo_t   pktInfo;
	paStatsBmap_t   statsMap2[3];  /* StatsMap for the first pass where IP-forwarding operation occurs */
} pktTest13Info2_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION (t13PktInfo2, ".testPkts")
#endif
static pktTest13Info2_t t13PktInfo2[] =  {

	/* Packet 1 */
    {
	    {
	        (uint8_t *)inpkt1,
	        (pasahoLongInfo_t *)&inpkt1Info,
	        sizeof(inpkt1),
	        { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
	        T13_SWINFO0_PKT_ID | (T13_FIRST_IP_FWD_ENTRY + 0)
	    },

	    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	      0,                                                                                                      	/* IP match */
	      0                                                                                                         /* UDP match */
        }
    },

	/* Packet 2 */
    {
	    {
	        (uint8_t *)inpkt2,
	        (pasahoLongInfo_t *)&inpkt2Info,
	        sizeof(inpkt2),
	        { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
	        T13_SWINFO0_PKT_ID | (T13_FIRST_IP_FWD_ENTRY + 1)
	    },

	    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	      0	                                                                                                       /* IP/UDP match */
        }
    },

	/* Packet 3 */
    {
	    {
	        (uint8_t *)inpkt3,
	        (pasahoLongInfo_t *)&inpkt3Info,
	        sizeof(inpkt3),
	        { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
	        T13_SWINFO0_PKT_ID | (T13_FIRST_IP_FWD_ENTRY + 2)
	    },

	    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)	/* Inner IP match */															  		/* UDP match */
        }
    },

	/* Packet 4 */
    {
	    {
	        (uint8_t *)inpkt4,
	        (pasahoLongInfo_t *)&inpkt4Info,
	        sizeof(inpkt4),
	        { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
	        T13_SWINFO0_PKT_ID | (T13_FIRST_IP_FWD_ENTRY + 3)
	    },

	    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)	/* IP/UDP match */															  		/* UDP match */
        }
    },

	/* Packet 5 */
    {
	    {
	        (uint8_t *)inpkt5,
	        (pasahoLongInfo_t *)&inpkt5Info,
	        sizeof(inpkt5),
	        { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
	        T13_SWINFO0_PKT_ID | (T13_FIRST_IP_FWD_ENTRY + 4)
	    },

	    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
	      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)	/* Inner IP match */															  		/* UDP match */
        }
    },
};

/* Egress Flow exception packets */

/* packet 0: Inner IP fragments
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt0, ".testPkts")
#endif
static uint8_t epkt0[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x20, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt0Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,40,48,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 1:Inner IP options
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt1, ".testPkts")
#endif
static uint8_t epkt1[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x54, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x46, 0x00, 0x00, 0x40, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xba, 0xbe,
    0xbe, 0xef,
    0xaa, 0xbb, 0x90, 0x02, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt1Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,44,52,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 2: Inner IP timeout
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt2, ".testPkts")
#endif
static uint8_t epkt2[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt2Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt2Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,40,48,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 3: TCP control packet
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt3, ".testPkts")
#endif
static uint8_t epkt3[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x48, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x06, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x02, 0x00, 0x64, 0xbe, 0xef,
    0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt3Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,40,60,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 4: Outer IP fragment
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt4, ".testPkts")
#endif
static uint8_t epkt4[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x50, 0x00, 0x01, 0x00, 0x20, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt4Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt4Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,40,48,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 5: Outer IP with IPv4 options
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt5, ".testPkts")
#endif
static uint8_t epkt5[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x47, 0x00,
	0x00, 0x58, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt5Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt5Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,28),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,48,56,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 6: Outer IP timeout
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt6, ".testPkts")
#endif
static uint8_t epkt6[] = {
    #if 0
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    #endif
    0x45, 0x00,
	0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt6Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt6Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,20),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,40,48,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 7: Inner IP Frags
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt7, ".testPkts")
#endif
static uint8_t epkt7[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x48, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x20, 0x2c, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

    0x11, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,

	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt7Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,80,88,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 8: Inner IP Options
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt8, ".testPkts")
#endif
static uint8_t epkt8[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x50, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x28, 0x00, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

    0x11, 0x01, 0x04, 0x08,                /* Hop-by-hop header */
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x38,


	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt8Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,96,104,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 9: Inner IP Timeout
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt9, ".testPkts")
#endif
static uint8_t epkt9[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x40, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x18, 0x11, 0x00,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt9Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,80,88,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 10: TCP Control
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt10, ".testPkts")
#endif
static uint8_t epkt10[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x4C, 0x29, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x24, 0x06, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x02,      // TCP Control Header
    0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x64,
    0xbe, 0xef, 0x00, 0x00,

    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt10Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,80,100,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 11: Outer IP Frags
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt11, ".testPkts")
#endif
static uint8_t epkt11[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x48, 0x2c, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

    0x29, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x18, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt11Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,88,96,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 12: Outer IP Options
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt12, ".testPkts")
#endif
static uint8_t epkt12[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x50, 0x00, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

    0x29, 0x01, 0x04, 0x08,     // Hop-by-hop header
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x38,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x18, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt12Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,96,104,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 13: Outer IP timeout
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x04
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Inner ip src  = .... 0x00, 0x00, 0x00, 0x05
 * Inner ip dest = .... 0x30, 0x32, 0x34, 0x3a
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt13, ".testPkts")
#endif
static uint8_t epkt13[] = {
	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x40, 0x29, 0x00,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x39,

	0x60, 0x01, 0x01, 0x01,      // Iv6 header
	0x00, 0x18, 0x11, 0xFF,
	0x20, 0x02,	0x9e, 0xda,
	0x6d, 0x30,	0x00, 0x00,
	0x00, 0x00,	0x00, 0x00,
	0x00, 0x00, 0x00, 0x05,
	0x00, 0x02, 0x04, 0x08,
	0x10, 0x12, 0x14, 0x18,
	0x20, 0x22, 0x24, 0x28,
	0x30, 0x32, 0x34, 0x3a,

	0xaa, 0xbb, 0x91, 0x02,
	0x00, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (epkt13Info, ".testPkts")
#endif
static pasahoLongInfo_t epkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),	     	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(0,80,88,0),    	            /* L3 offset = 0, l4Offset = 80, l5Offset = 88, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (t13expPktInfo, ".testPkts")
#endif
static pktTest13Info_t t13expPktInfo[] =  {

	/* Packet 0 */
    {
	    {
		    (uint8_t *)epkt0,
		    (pasahoLongInfo_t *)&epkt0Info,
		    sizeof(epkt0),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 0		/* Packet will be matched by exception packet index 0 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt0),                       /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        40,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 1 */
    {
	    {
		    (uint8_t *)epkt1,
		    (pasahoLongInfo_t *)&epkt1Info,
		    sizeof(epkt1),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 1		/* Packet will be matched by exception packet index 1 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt1),                      /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        44,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 2 */
    {
	    {
		    (uint8_t *)epkt2,
		    (pasahoLongInfo_t *)&epkt2Info,
		    sizeof(epkt2),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 2		/* Packet will be matched by exception packet index 2 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt2),                       /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        40,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 3 */
    {
	    {
		    (uint8_t *)epkt3,
		    (pasahoLongInfo_t *)&epkt3Info,
		    sizeof(epkt3),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 3		/* Packet will be matched by exception packet index 3 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt3),                      /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        40,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 4 */
    {
	    {
		    (uint8_t *)epkt4,
		    (pasahoLongInfo_t *)&epkt4Info,
		    sizeof(epkt4),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 0		/* Packet will be matched by exception packet index 0 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt4),                       /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        40,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 5 */
    {
	    {
		    (uint8_t *)epkt5,
		    (pasahoLongInfo_t *)&epkt5Info,
		    sizeof(epkt5),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 1		/* Packet will be matched by exception packet index 1 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            28,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt5),                       /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        48,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 6 */
    {
	    {
		    (uint8_t *)epkt6,
		    (pasahoLongInfo_t *)&epkt6Info,
		    sizeof(epkt6),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 2		/* Packet will be matched by exception packet index 2 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            20,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt6),                      /* endOffset */
            254,                                /* lvl1Index */
            40,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            11                                  /* lvl4Index */
        },
        0,
        40,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 7 */
    {
	    {
		    (uint8_t *)epkt7,
		    (pasahoLongInfo_t *)&epkt7Info,
		    sizeof(epkt7),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 0		   /* Packet will be matched by exception packet index 0 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            40,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt7),                      /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        88,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },


	/* Packet 8 */
    {
	    {
		    (uint8_t *)epkt8,
		    (pasahoLongInfo_t *)&epkt8Info,
		    sizeof(epkt8),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 1		   /* Packet will be matched by exception packet index 1 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            40,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt8),                      /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        96,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 9 */
    {
	    {
		    (uint8_t *)epkt9,
		    (pasahoLongInfo_t *)&epkt9Info,
		    sizeof(epkt9),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 2		   /* Packet will be matched by exception packet index 2 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            40,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt9),                      /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        80,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 10 */
    {
	    {
		    (uint8_t *)epkt10,
		    (pasahoLongInfo_t *)&epkt10Info,
		    sizeof(epkt10),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 3		   /* Packet will be matched by exception packet index 3 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            40,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt10),                      /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        80,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 11 */
    {
	    {
		    (uint8_t *)epkt11,
		    (pasahoLongInfo_t *)&epkt11Info,
		    sizeof(epkt11),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 0		   /* Packet will be matched by exception packet index 0 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            48,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt11),                     /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        88,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 12 */
    {
	    {
		    (uint8_t *)epkt12,
		    (pasahoLongInfo_t *)&epkt12Info,
		    sizeof(epkt12),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 1		   /* Packet will be matched by exception packet index 1 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            56,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt12),                     /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        96,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 13 */
    {
	    {
		    (uint8_t *)epkt13,
		    (pasahoLongInfo_t *)&epkt13Info,
		    sizeof(epkt13),
            { 0, 0, 0 },
		    T13_SWINFO0_EXP_PKT_ID | 2		   /* Packet will be matched by exception packet index 2 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            0,                                  /* l3Offset */
            40,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(epkt13),                     /* endOffset */
            150,                                /* lvl1Index */
            160,                                /* lvl2Index */
            0,                                  /* lvl3Index */
            111                                 /* lvl4Index */
        },
        0,
        80,
        TRUE,
        TRUE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    }

};


/* QoS packets */
/* packet 1: Single IP DSCP priority (TOS will be updated to 4 (DSCP = 1))
 * mac dest = 00:01:02:03:04:aa
 * ip src  = 158.218.109.12
 * ip dest = 200.201.202.100
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt1, ".testPkts")
#endif
static uint8_t qospkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    0x45, 0x00,
	0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0c, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x02, 0x00, 0x28,
	0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt1Info, ".testPkts")
#endif
static pasahoLongInfo_t qospkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		            /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset, l4Offset, l5Offset, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 2: DSCP Priority based on replaced outer IP TOS (0x10) ==> DSCP = 4
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt2, ".testPkts")
#endif
static uint8_t qospkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x00, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt2Info, ".testPkts")
#endif
static pasahoLongInfo_t qospkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		            /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset, l4Offset, l5Offset, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 3: VLAN Priority = 3
 * mac dest = 00:01:02:03:04:aa
 * ip src  = 158.218.109.12
 * ip dest = 200.201.202.100
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt3, ".testPkts")
#endif
static uint8_t qospkt3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x81, 0x00, 0x65, 0x55,
    0x08, 0x00,
    0x45, 0x00,
	0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0c, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x02, 0x00, 0x28,
	0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt3Info, ".testPkts")
#endif
static pasahoLongInfo_t qospkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		            /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(18,38,46,0),    	            /* L3 offset, l4Offset, l5Offset, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

/* packet 4: Inner VLAN priority in QinQ packet, Priority = 7
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt4, ".testPkts")
#endif
static uint8_t qospkt4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0xa8, 0x07, 0x77,
    0x81, 0x00, 0xE5, 0x55, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65,
    0x45, 0x00, 0x00, 0x3c, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x16, 0x78, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x00, 0x00, 0x28, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (qospkt4Info, ".testPkts")
#endif
static pasahoLongInfo_t qospkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,0),		            /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,62,70,0),    	            /* L3 offset, l4Offset, l5Offset,, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0, 0, 0),                    /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (t13qosPktInfo, ".testPkts")
#endif
static pktTest13Info_t t13qosPktInfo[] =  {

	/* Packet 1 */
    {
	    {
		    (uint8_t *)qospkt1,
		    (pasahoLongInfo_t *)&qospkt1Info,
		    sizeof(qospkt1),
            { 0, 0, 0 },
		    T13_SWINFO0_QOS_PKT_ID | 0		   /* Packet will be matched by exception packet index 0 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            14,                                 /* l3Offset */
            14,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(qospkt1),                      /* endOffset */
            10,                                 /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            201                                 /* lvl4Index */
        },
        0,
        34,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 2 */
    {
	    {
		    (uint8_t *)qospkt2,
		    (pasahoLongInfo_t *)&qospkt2Info,
		    sizeof(qospkt2),
            { 0, 0, 0 },
		    T13_SWINFO0_QOS_PKT_ID | 0		   /* Packet will be matched by exception packet index 0 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            14,                                 /* l3Offset */
            34,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(qospkt2),                    /* endOffset */
            240,                                /* lvl1Index */
            20,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            201                                 /* lvl4Index */
        },
        0,
        54,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 3 */
    {
	    {
		    (uint8_t *)qospkt3,
		    (pasahoLongInfo_t *)&qospkt3Info,
		    sizeof(qospkt3),
            { 0, 0, 0 },
		    T13_SWINFO0_QOS_PKT_ID | 1		   /* Packet will be matched by exception packet index 1 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            18,                                 /* l3Offset */
            18,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(qospkt3),                      /* endOffset */
            10,                                 /* lvl1Index */
            0,                                  /* lvl2Index */
            0,                                  /* lvl3Index */
            211                                 /* lvl4Index */
        },
        0,
        38,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    },

	/* Packet 4 */
    {
	    {
		    (uint8_t *)qospkt4,
		    (pasahoLongInfo_t *)&qospkt4Info,
		    sizeof(qospkt4),
            { 0, 0, 0 },
		    T13_SWINFO0_QOS_PKT_ID | 1		   /* Packet will be matched by exception packet index 1 */
	    },

        {
            pa_EF_OP_CMD_VALID_LVL1 |           /* ctrlBitfield */
            pa_EF_OP_CMD_VALID_LVL2 |
            pa_EF_OP_CMD_VALID_LVL4,
            0,                                  /* l2Offset */
            22,                                 /* l3Offset */
            42,                                 /* l3Offset2 */
            0,                                  /* ipsecOffset */
            sizeof(qospkt2),                    /* endOffset */
            240,                                /* lvl1Index */
            20,                                 /* lvl2Index */
            0,                                  /* lvl3Index */
            211                                 /* lvl4Index */
        },
        0,
        62,
        TRUE,
        FALSE,
        FALSE,
        {
            0,
            0,
            0,
            0
        }
    }
};

#endif /*TEST13PKTS_H_*/
