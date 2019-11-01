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



#ifndef TEST5PKTS_H_
#define TEST5PKTS_H_



/* Valid rx MAC addresses used during the test */
typedef struct t5EthSetup  {
	paEthInfo_t  ethInfo;		/* PA Ethernet configuration structure */
	Bool	     acked;			/* Set to TRUE when the reply to the command is received */
} t5EthSetup_t;


#ifdef _TMS320C6X
#pragma DATA_SECTION (t5EthSetup, ".testPkts")
#endif
static t5EthSetup_t t5EthSetup[] =  {

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
   	 	{ 0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09 },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
      },
      FALSE  }


};

typedef struct t5SpiIpSetup_s  {
	int		   lHandleIdx;  /* Linked handle to previous L3 (Ip), -1 indicates no link to IP */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	Bool	   acked;		    /* Set to TRUE when the reply to the command is received */
} t5SpiIpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t5SpiIpSetup, ".testPkts")
#endif
static t5SpiIpSetup_t  t5SpiIpSetup[] = {
	{
		/* SPI IP Entry 0 */
		0,		/* Linked to IP index 0 */
		{	{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },  /* IP Source address */
			{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },  /* IP Destination address */
			0x34ff003e,			/* SPI */
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

    {
      /* SPI IP Entry 0 */
      0,    /* Linked to IP index 0 */
      { {{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
        {{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
        0x34ff003f,     /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

#ifndef NSS_GEN2
	{
		/* SPI IP Entry 1 */
		-1,		/* SPI entry not linked */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
			0x44440001,	/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	}
#endif
};


typedef struct t5IpSetup_s  {
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */

} t5IpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t5IpSetup, ".testPkts")
#endif
static t5IpSetup_t  t5IpSetup[] = {

	{
		/* IP Entry 0 */
		0,		/* Linked to dest mac index 0 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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
	{	1,		/* Linked to dest mac index 1 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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
	{	0,		/* Linked to dest mac index 0 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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
	{	1,		/* Linked to dest mac index 1 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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

	/* IP Entry 4 */
	{	0,		/* Linked to dest mac index 0 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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

		/* IP Entry 5 */
	{	1,		/* Linked to dest mac index 1 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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

		/* IP Entry 6 */
	{	0,		/* Linked to dest mac index 0 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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

		/* IP Entry 7 */
	{	1,		/* Linked to dest mac index 1 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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

		/* IP Entry 8 */
	{	0,		/* Linked to dest mac index 0 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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


		/* IP Entry 9 */
	{	1,		/* Linked to dest mac index 1 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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


		/* IP Entry 10 */
	{	0,		/* Linked to dest mac index 0 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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

		/* IP Entry 11 */
	{	1,		/* Linked to dest mac index 1 */
		{	{{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Source address */
			{{ 200, 201, 202, 111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }},  /* IP Destination address */
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
	}

};

/*
 * User-definded Statitics Map
 *
 * Group 1: counter 0x00 - 0x0b, byte counter, linking to 0x40 - 0x4b respectively
 * Group 2: counter 0x40 - 0x4b, packet counter, linking to 0x80-0x84
 * Group 3: counter 0x80 - 0x85, packet counter, no link
 * Group 4: counter 0xb0, packet counter (MAC 0) link to 0x20
 *          counter 0xb1, packet counter (MAC 1) link to 0x21
 *          counter 0xb2, packet counter (NAT-T error) link to 0x22
 *          counter 0xb3, packet counter (NAT-T Keep Alive) link to 0x23
 * Group 5: counter 0x20 - 0x22, byte counter, no link
 * Group 6: counter 0x40, packet counter, link to 0x00
 *          This group should be rejected since it forms a close loop with entry 0 at Group 1
 * Group 7: counter 0x60, packet counter, link to 0x61
 *          counter 0x61, packet counter, link to 0x62
 *          counter 0x62, packet counter, link to 0x60
 *          This group should be rejected since it forms a close loop (0x60==>0x61==>0x62==>0x60)
 */
static uint16_t t5L3UsrStats[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};
static uint16_t t5L2UsrStats[2]  = {0xB0, 0xB1};
static uint16_t t5NatTErrUsrStats = 0xB2;
static uint16_t t5NatTKeepAliveUsrStats = 0xB3;

#ifdef _TMS320C6X
#pragma DATA_SECTION (usrStatsGroup1, ".testPkts")
#pragma DATA_SECTION (usrStatsGroup2, ".testPkts")
#pragma DATA_SECTION (usrStatsGroup3, ".testPkts")
#pragma DATA_SECTION (usrStatsGroup4, ".testPkts")
#pragma DATA_SECTION (usrStatsGroup5, ".testPkts")
#pragma DATA_SECTION (usrStatsGroup6, ".testPkts")
#pragma DATA_SECTION (usrStatsGroup7, ".testPkts")
#endif

static paUsrStatsCounterEntryConfig_t usrStatsGroup1[ ] =
{
    {0x00, 0x40, pa_USR_STATS_TYPE_BYTE},
    {0x01, 0x41, pa_USR_STATS_TYPE_BYTE},
    {0x02, 0x42, pa_USR_STATS_TYPE_BYTE},
    {0x03, 0x43, pa_USR_STATS_TYPE_BYTE},
    {0x04, 0x44, pa_USR_STATS_TYPE_BYTE},
    {0x05, 0x45, pa_USR_STATS_TYPE_BYTE},
    {0x06, 0x46, pa_USR_STATS_TYPE_BYTE},
    {0x07, 0x47, pa_USR_STATS_TYPE_BYTE},
    {0x08, 0x48, pa_USR_STATS_TYPE_BYTE},
    {0x09, 0x49, pa_USR_STATS_TYPE_BYTE},
    {0x0a, 0x4a, pa_USR_STATS_TYPE_BYTE},
    {0x0b, 0x4b, pa_USR_STATS_TYPE_BYTE}

};

static paUsrStatsCounterEntryConfig_t usrStatsGroup2[ ] =
{
    {0x40, 0x80, pa_USR_STATS_TYPE_PACKET},
    {0x41, 0x81, pa_USR_STATS_TYPE_PACKET},
    {0x42, 0x81, pa_USR_STATS_TYPE_PACKET},
    {0x43, 0x82, pa_USR_STATS_TYPE_PACKET},
    {0x44, 0x82, pa_USR_STATS_TYPE_PACKET},
    {0x45, 0x82, pa_USR_STATS_TYPE_PACKET},
    {0x46, 0x83, pa_USR_STATS_TYPE_PACKET},
    {0x47, 0x83, pa_USR_STATS_TYPE_PACKET},
    {0x48, 0x83, pa_USR_STATS_TYPE_PACKET},
    {0x49, 0x83, pa_USR_STATS_TYPE_PACKET},
    {0x4a, 0x84, pa_USR_STATS_TYPE_PACKET},
    {0x4b, 0x84, pa_USR_STATS_TYPE_PACKET}
};

static paUsrStatsCounterEntryConfig_t usrStatsGroup3[ ] =
{
    {0x80, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_PACKET},
    {0x81, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_PACKET},
    {0x82, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_PACKET},
    {0x83, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_PACKET},
    {0x84, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_PACKET}
};

static paUsrStatsCounterEntryConfig_t usrStatsGroup4[ ] =
{
    {0xb0, 0x20, pa_USR_STATS_TYPE_PACKET},
    {0xb1, 0x21, pa_USR_STATS_TYPE_PACKET},
    {0xb2, 0x22, pa_USR_STATS_TYPE_PACKET},
    {0xb3, 0x23, pa_USR_STATS_TYPE_PACKET},

};

static paUsrStatsCounterEntryConfig_t usrStatsGroup5[ ] =
{
    {0x20, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_BYTE},
    {0x21, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_BYTE},
    {0x22, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_BYTE},
    {0x23, pa_USR_STATS_LNK_END, pa_USR_STATS_TYPE_BYTE}
};


static paUsrStatsCounterEntryConfig_t usrStatsGroup6[ ] =
{
    {0x40, 0x00, pa_USR_STATS_TYPE_PACKET}
};

static paUsrStatsCounterEntryConfig_t usrStatsGroup7[ ] =
{
    {0x60, 0x61, pa_USR_STATS_TYPE_PACKET},
    {0x61, 0x62, pa_USR_STATS_TYPE_PACKET},
    {0x62, 0x60, pa_USR_STATS_TYPE_PACKET}
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t5IpSetup, ".testPkts")
#endif
static pauUsrStatsSetup_t  t5UsrStatsSetup[] = {
    /* entry 0 */
    {
        sizeof(usrStatsGroup1)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup1,                                                     /* counter Info table */
        pa_OK                                                               /* Expected return value */
    },

    /* entry 1 */
    {
        sizeof(usrStatsGroup2)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup2,                                                     /* counter Info table */
        pa_OK                                                               /* Expected return value */
    },

    /* entry 2 */
    {
        sizeof(usrStatsGroup3)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup3,                                                     /* counter Info table */
        pa_OK                                                               /* Expected return value */
    },

    /* entry 3 */
    {
        sizeof(usrStatsGroup4)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup4,                                                     /* counter Info table */
        pa_OK                                                               /* Expected return value */
    },

    /* entry 4 */
    {
        sizeof(usrStatsGroup5)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup5,                                                     /* counter Info table */
        pa_OK                                                               /* Expected return value */
    },

    /* entry 5 */
    {
        sizeof(usrStatsGroup6)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup6,                                                     /* counter Info table */
        pa_ERR_CONFIG                                                       /* Expected return value */
    },

    /* entry 6 */
    {
        sizeof(usrStatsGroup7)/sizeof(paUsrStatsCounterEntryConfig_t),      /* number of entries */
        usrStatsGroup7,                                                     /* counter Info table */
        pa_ERR_CONFIG                                                       /* Expected return value */
    }
};

/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x4e,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(112,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(112,0,1,0,0,0),  /* end offset = 112, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 1
 * mac dest = 00:e0:a6:66:57:09
 * ip dest = 200.201.202.101
 * Designed to match IP configuration 1 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x76, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x63, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x65, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x62,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb, 0xbc, 0xbd
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt1Info = {


	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(132,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 132, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(132,0,1,0,0,0),  /* end offset = 132, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


/* Packet 2
 * mac dest - 00:01:02:03:04:aa
 * ip dest = 200.201.202.102
 * Designed to match IP configuration 2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x86, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x66, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x3e,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(96,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 96, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(96,0,1,0,0,0),  /* end offset = 96, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif
/* Packet 3
 * mac dest = 00:e0:a6:66:57:09
 * ip dest = 200.201.202.103
 * Designed to match IP configuration 3 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3, ".testPkts")
#endif
static uint8_t pkt3[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x7b, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x67, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x48,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 106, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 4
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.104
 * Designed to match IP configuration 4 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4, ".testPkts")
#endif
static uint8_t pkt4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5e, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x68, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x64,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 134, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(134,0,1,0,0,0),  /* end offset = 134, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 5
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.105
 * Designed to match IP configuration 5 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5, ".testPkts")
#endif
static uint8_t pkt5[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x74, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x61, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x69, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x60,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 130, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(130,0,1,0,0,0),  /* end offset = 130, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 6
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.106
 * Designed to match IP configuration 6 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6, ".testPkts")
#endif
static uint8_t pkt6[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x64, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6a, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x5c,
	0x00, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 126, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(126,0,1,0,0,0),  /* end offset = 126, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 7
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 00.201.202.107
 * Designed to match IP configuration 7 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7, ".testPkts")
#endif
static uint8_t pkt7[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5b, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6b, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x64,
	0x00, 0x00, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
	0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21,
	0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
	0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
	0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
	0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
	0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 134, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(134,0,1,0,0,0),  /* end offset = 134, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 8
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.108
 * Designed to match IP configuration 8 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8, ".testPkts")
#endif
static uint8_t pkt8[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5a, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6c, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x64,
	0x00, 0x00, 0xa6, 0xe7, 0x94, 0x3d, 0x32, 0x83,
	0x00, 0x39, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 134, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(134,0,1,0,0,0),  /* end offset = 134, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 9
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9, ".testPkts")
#endif
static uint8_t pkt9[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x00, 0x00, 0x00, 0x6a,
	0x00, 0x00, 0xa6, 0xe7, 0x94, 0x3d, 0x32, 0x83,
	0x00, 0x39, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 140, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,42),		/* cmd len = 20, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(140,0,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 10
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.110
 * Designed to match IP configuration 10
 * TCP */

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10, ".testPkts")
#endif
static uint8_t pkt10[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x72, 0x00, 0x00, 0x00, 0x00, 0x05, 0x06,
	0x16, 0x69, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x12, 0x34,
	0x56, 0x78, 0x11, 0x22, 0x33, 0x44, 0x61, 0x6a,
	0x33, 0x33, 0xa9, 0x9a, 0x44, 0x44, 0x55, 0x66,
	0x77, 0x00, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9  };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,58),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(128,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_TCP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,58),		/* cmd len = 20, start offset = 58 (Payload) */
	TF_FORM_PKT_INFO_WORD1(128,0,1,0,0,0),  /* end offset = 128, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_TCP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 11
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.111
 * Designed to match IP configuration 10
 * TCP */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11, ".testPkts")
#endif
static uint8_t pkt11[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x88, 0x00, 0x00, 0x00, 0x00, 0x05, 0x06,
	0x16, 0x52, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x12, 0x34,
	0x56, 0x78, 0x11, 0x22, 0x33, 0x44, 0x51, 0x6a,
	0x33, 0x33, 0x3e, 0x96, 0x44, 0x44, 0x64, 0x65,
	0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d,
	0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75,
	0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d,
	0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85,
	0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d,
	0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95,
	0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d,
	0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad,
	0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5,
	0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd,
	0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(150,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 150, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,00,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_TCP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,20,54),		/* cmd len = 20, start offset = 54 (Payload) */
	TF_FORM_PKT_INFO_WORD1(150,0,1,0,0,0),  /* end offset = 150, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 0, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_TCP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#define T5_PKTINFO_IDX_MATCH_FLAG	0x40000000
#define T5_PKTINFO_IDX_MASK			0x30000000

#ifdef _TMS320C6X
#pragma DATA_SECTION (t5PktInfo, ".testPkts")
#endif
static pktTestInfo_t t5PktInfo[] =  {

	/* Packet 0 */
	{
		(uint8_t *)pkt0,
		(pasahoLongInfo_t *)&pkt0Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },                       		  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 1 */
	{
		(uint8_t *)pkt1,
		(pasahoLongInfo_t *)&pkt1Info,
		sizeof(pkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 2 */
	{
		(uint8_t *)pkt2,
		(pasahoLongInfo_t *)&pkt2Info,
		sizeof(pkt2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 3 */
	{
		(uint8_t *)pkt3,
		(pasahoLongInfo_t *)&pkt3Info,
		sizeof(pkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 4 */
	{
		(uint8_t *)pkt4,
		(pasahoLongInfo_t *)&pkt4Info,
		sizeof(pkt4),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 5 */
	{
		(uint8_t *)pkt5,
		(pasahoLongInfo_t *)&pkt5Info,
		sizeof(pkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 6 */
	{
		(uint8_t *)pkt6,
		(pasahoLongInfo_t *)&pkt6Info,
		sizeof(pkt6),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},


	/* Packet 7 */
	{
		(uint8_t *)pkt7,
		(pasahoLongInfo_t *)&pkt7Info,
		sizeof(pkt7),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 8 */
	{
		(uint8_t *)pkt8,
		(pasahoLongInfo_t *)&pkt8Info,
		sizeof(pkt8),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 9 */
	{
		(uint8_t *)pkt9,
		(pasahoLongInfo_t *)&pkt9Info,
		sizeof(pkt9),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 10 */
	{
		(uint8_t *)pkt10,
		(pasahoLongInfo_t *)&pkt10Info,
		sizeof(pkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_TCP) },	 							  		/* TCP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	},

	/* Packet 11 */
	{
		(uint8_t *)pkt11,
		(pasahoLongInfo_t *)&pkt11Info,
		sizeof(pkt11),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_TCP) }, 								  		/* TCP match */
		T5_PKTINFO_IDX_MATCH_FLAG | 0		/* Packet will be matched by handle index 0 */
	}


};

/* The pseudo header checksums used for UDP checksum generation */
static unsigned int t5PseudoChksums[] = {
	0x9f73,		/* Packet 0 */
	0x9f88,		/* Packet 1 */
	0x9f65,		/* Packet 2 */
	0x9f70,		/* Packet 3 */
	0x9f8d,		/* Packet 4 */
	0x9f8a,		/* Packet 5 */
	0x9f87,		/* Packet 6 */
	0x9f90,		/* Packet 7 */
	0x9f91,		/* Packet 8 */
	0x9f98,		/* Packet 9 */
	0x9f82,		/* Packet 10 */
	0x9f99      /* Packet 11 */
};

/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0
 * TEID = 0x01000000
 * Insert the PDU number (0xface) at the packet descriptor offset 20
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt0, ".testPkts")
#endif
static uint8_t gtpuPkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x4e,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x3e, 0x01, 0x00,
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
#pragma DATA_SECTION (gtpuPkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,58),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(112,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,58),		/* cmd len = 24, start offset = 58 (GTPU-Payload) */
	TF_FORM_PKT_INFO_WORD1(112,0,1,0,0,0),  /* end offset = 112, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 1
 * mac dest = 00:e0:a6:66:57:09
 * ip dest = 200.201.202.101
 * Designed to match IP configuration 5
 * TEID = 0x01000005
 * Commands: Remove header/Insert 0/Remove Tail
 * Insert the PDU number (0xbeef) at the packet descriptor offset 20
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt1, ".testPkts")
#endif
static uint8_t gtpuPkt1[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x76, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x63, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x69, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x62,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x52, 0x01, 0x00,
	0x00, 0x05, 0xba, 0xbe, 0x01, 0xc0, 0x01, 0xbe,
	0xef, 0x00, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb, 0xbc, 0xbd, 0x01, 0x02, 0x03, 0x04
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0),  		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(75,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		    /* cmd len = 24, start offset = 0 (Payload) */
	TF_FORM_PKT_INFO_WORD1(75,0,1,0,0,0),   /* end offset = 75, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 2
 * mac dest - 00:01:02:03:04:aa
 * ip dest = 200.201.202.102
 * Designed to match IP configuration 10
 * TEID = 0x0100000A
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt2, ".testPkts")
#endif
static uint8_t gtpuPkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x86, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6e, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x3e,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x2e, 0x01, 0x00,
	0x00, 0x0a, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(96,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 96, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (Payload) */
	TF_FORM_PKT_INFO_WORD1(96,0,1,0,0,0),  /* end offset = 96, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 3
 * mac dest = 00:e0:a6:66:57:09
 * ip dest = 200.201.202.103
 * Designed to match IP configuration 9 (21 % 12)
 * TEID = 0x01000015
 * Commands: Remove header/Insert 0/Remove Tail
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt3, ".testPkts")
#endif
static uint8_t gtpuPkt3[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x7b, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x48,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x38, 0x01, 0x00,
	0x00, 0x15, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0),	    	        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(57,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 57, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),  		/* cmd len = 24, start offset = 0 (Payload) */
	TF_FORM_PKT_INFO_WORD1(57,0,1,0,0,0),   /* end offset = 57, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 4
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.104
 * Designed to match IP configuration 2 (26 % 12)
 * TEID = 0x0100001A
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt4, ".testPkts")
#endif
static uint8_t gtpuPkt4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5e, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x66, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x64,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x54, 0x01, 0x00,
	0x00, 0x1a, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (Payload) */
	TF_FORM_PKT_INFO_WORD1(134,0,1,0,0,0),  /* end offset = 134, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 5
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.105
 * Designed to match IP configuration 5 (37 % 12)
 * TEID = 0x01000025
 * Commands: Remove header/Insert 0/Remove Tail
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt5, ".testPkts")
#endif
static uint8_t gtpuPkt5[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x74, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x61, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x65, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x60,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x50, 0x01, 0x00,
	0x00, 0x25, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0), 		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(81,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 81, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		    /* cmd len = 24, start offset = 0 (Payload) */
	TF_FORM_PKT_INFO_WORD1(81,0,1,0,0,0),   /* end offset = 81, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 6
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.106
 * Designed to match IP configuration 6 (42 % 12)
 * TEID = 0x0100002A
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt6, ".testPkts")
#endif
static uint8_t gtpuPkt6[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x64, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6a, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x5c,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x4c, 0x01, 0x00,
	0x00, 0x2a, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt6Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(126,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 126, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (Payload) */
	TF_FORM_PKT_INFO_WORD1(126,0,1,0,0,0),  /* end offset = 126, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 7
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 00.201.202.107
 * Designed to match IP configuration 5 (53 % 12)
 * TEID = 0x01000035
 * Commands: Remove header/Insert 0/Remove Tail
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt7, ".testPkts")
#endif
static uint8_t gtpuPkt7[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5b, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x69, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x64,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x54, 0x01, 0x00,
	0x00, 0x35, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
	0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21,
	0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
	0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
	0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
	0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
	0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt7Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0), 		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(85,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 85, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt7Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),	   	    /* cmd len = 24, start offset = 0 (Payload) */
	TF_FORM_PKT_INFO_WORD1(85,0,1,0,0,0),   /* end offset = 85, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 8
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.108
 * Designed to match IP configuration 10  (58 % 12)
 * TEID = 0x0100003A
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt8, ".testPkts")
#endif
static uint8_t gtpuPkt8[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5a, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6e, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x64,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x54, 0x01, 0x00,
	0x00, 0x3a, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt8Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(134,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 134, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (Payload) */
	TF_FORM_PKT_INFO_WORD1(134,0,1,0,0,0),  /* end offset = 134, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 9
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 3  (99 % 12)
 * TEID = 0x01000063
 * Commands: Remove header/Insert 0/Remove Tail
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt9, ".testPkts")
#endif
static uint8_t gtpuPkt9[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x67, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt9Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0),		            /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(91,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt9Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,0),		    /* cmd len = 24, start offset = 0 (Payload) */
	TF_FORM_PKT_INFO_WORD1(91,0,1,0,0,0),   /* end offset = 91, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 10
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * GTPU message type 1
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt10, ".testPkts")
#endif
static uint8_t gtpuPkt10[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0x01, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt10Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_1,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t gtpuPkt10Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_1,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 11
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * GTP-U Message Type 2
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt11, ".testPkts")
#endif
static uint8_t gtpuPkt11[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0x02, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt11Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_2,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt11Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_2,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 12
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * GTP-U Message Type 26
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt12, ".testPkts")
#endif
static uint8_t gtpuPkt12[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0x1a, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt12Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_26,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_26,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 13
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * Message Type 31
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt13, ".testPkts")
#endif
static uint8_t gtpuPkt13[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0x1f, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt13Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_31,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt13Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_31,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 14
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * GTP-U Message Type 254
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt14, ".testPkts")
#endif
static uint8_t gtpuPkt14[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0xfe, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt14Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt14Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_254,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt14Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_MESSAGE_TYPE_254,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 15
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * Message type 0x80 ==> nusupported
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt15, ".testPkts")
#endif
static uint8_t gtpuPkt15[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0x80, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt15Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt15Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt15Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 16
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * Incorrect Version Number
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt16, ".testPkts")
#endif
static uint8_t gtpuPkt16[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x50, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt16Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt16Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt16Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 17
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * Incorrect PT
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt17, ".testPkts")
#endif
static uint8_t gtpuPkt17[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x20, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt17Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt17Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt17Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 18
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * E = 1, but bad Type
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt18, ".testPkts")
#endif
static uint8_t gtpuPkt18[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x00, 0x01, 0x02, 0xaa, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt18Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt18Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt18Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 19
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * E = 1, NextHdr = 0xc0, next Hdr = 0xc0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt19, ".testPkts")
#endif
static uint8_t gtpuPkt19[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x00, 0x01, 0x02, 0xc0, 0x01, 0x00,
	0x00, 0xc0, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt19Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt19Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt19Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_GTPU_FAIL,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 20
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * IP Fragmentation
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt20, ".testPkts")
#endif
static uint8_t gtpuPkt20[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x20, 0x80, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x00, 0x63, 0x00, 0x01, 0x02, 0xc0, 0x01, 0x00,
	0x00, 0xc0, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt20Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt20Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_IP_FRAG,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 00, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt20Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(140,pa_EROUTE_IP_FRAG,1,0,0,0),  /* end offset = 140, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,0,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* Packet 21
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * TEID = 0x01000263
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt21, ".testPkts")
#endif
static uint8_t gtpuPkt21[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x5a, 0x01, 0x00,
	0x02, 0x63, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt21Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t gtpuPkt21Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0, 50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(140, pa_EROUTE_GTPU_MATCH_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t gtpuPkt21Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		    /* cmd len = 24, start offset = 0 (Payload) */
	TF_FORM_PKT_INFO_WORD1(140, pa_EROUTE_GTPU_MATCH_FAIL,1,0,0,0),   /* end offset = 91, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_GTPU),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 0_3
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0
 * TEID = 0x01000000
 * Insert the PDU number (0xface) at the packet descriptor offset 20
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt0_3, ".testPkts")
#endif
static uint8_t gtpuPkt0_3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x4e,
	0x00, 0x00, 0x34, 0xfe, 0x00, 0x3e, 0x01, 0x00,
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
#pragma DATA_SECTION (t5GTPUPktInfo1, ".testPkts")
#endif
static pktTestInfo_t t5GTPUPktInfo1[] = {
	/* Packet 0_3, modified packet 0 to test treating msg254 same as msg255 */
	{
		(uint8_t *)gtpuPkt0_3,
		(pasahoLongInfo_t *)&gtpuPkt0Info,
		sizeof(gtpuPkt0_3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX
	},
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (t5GTPUPktInfo, ".testPkts")
#endif
static pktTestInfo_t t5GTPUPktInfo[] =  {

	/* Packet 0 */
	{
		(uint8_t *)gtpuPkt0,
		(pasahoLongInfo_t *)&gtpuPkt0Info,
		sizeof(gtpuPkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX
	},

	/* Packet 1 */
	{
		(uint8_t *)gtpuPkt1,
		(pasahoLongInfo_t *)&gtpuPkt1Info,
		sizeof(gtpuPkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x05
	},

	/* Packet 2 */
	{
		(uint8_t *)gtpuPkt2,
		(pasahoLongInfo_t *)&gtpuPkt2Info,
		sizeof(gtpuPkt2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x0a
	},

	/* Packet 3 */
	{
		(uint8_t *)gtpuPkt3,
		(pasahoLongInfo_t *)&gtpuPkt3Info,
		sizeof(gtpuPkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x15
	},

	/* Packet 4 */
	{
		(uint8_t *)gtpuPkt4,
		(pasahoLongInfo_t *)&gtpuPkt4Info,
		sizeof(gtpuPkt4),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x1a
	},

	/* Packet 5 */
	{
		(uint8_t *)gtpuPkt5,
		(pasahoLongInfo_t *)&gtpuPkt5Info,
		sizeof(gtpuPkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x25
	},

	/* Packet 6 */
	{
		(uint8_t *)gtpuPkt6,
		(pasahoLongInfo_t *)&gtpuPkt6Info,
		sizeof(gtpuPkt6),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x2a
	},


	/* Packet 7 */
	{
		(uint8_t *)gtpuPkt7,
		(pasahoLongInfo_t *)&gtpuPkt7Info,
		sizeof(gtpuPkt7),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x35
	},

	/* Packet 8 */
	{
		(uint8_t *)gtpuPkt8,
		(pasahoLongInfo_t *)&gtpuPkt8Info,
		sizeof(gtpuPkt8),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x3a
	},

	/* Packet 9 */
	{
		(uint8_t *)gtpuPkt9,
		(pasahoLongInfo_t *)&gtpuPkt9Info,
		sizeof(gtpuPkt9),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x63
	},

	/* Packet 10 */
	{
		(uint8_t *)gtpuPkt10,
		(pasahoLongInfo_t *)&gtpuPkt10Info,
		sizeof(gtpuPkt10),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_PING_REQ_PKT_INDEX
	},

	/* Packet 11 */
	{
		(uint8_t *)gtpuPkt11,
		(pasahoLongInfo_t *)&gtpuPkt11Info,
		sizeof(gtpuPkt11),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_PING_RESP_PKT_INDEX
	},

	/* Packet 12 */
	{
		(uint8_t *)gtpuPkt12,
		(pasahoLongInfo_t *)&gtpuPkt12Info,
		sizeof(gtpuPkt12),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_ERR_IND_PKT_INDEX
	},

	/* Packet 13 */
	{
		(uint8_t *)gtpuPkt13,
		(pasahoLongInfo_t *)&gtpuPkt13Info,
		sizeof(gtpuPkt13),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_HDR_NOTIFY_PKT_INDEX
	},

	/* Packet 14 */
	{
		(uint8_t *)gtpuPkt14,
		(pasahoLongInfo_t *)&gtpuPkt14Info,
		sizeof(gtpuPkt14),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_END_MARKER_PKT_INDEX
	},

    /*
     * Packet 15-19 test different types of GTP-U Parsing Error
     * The SwInfo0 will contain the same index (T5_GTPU_FAIL_PKT_INDEX)
     * Therefore, they are all received as Packet 15
     * The packetInfo of packet 15-19 should be identical
     *
     */
	/* Packet 15 */
	{
		(uint8_t *)gtpuPkt15,
		(pasahoLongInfo_t *)&gtpuPkt15Info,
		sizeof(gtpuPkt15),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 							  		/* UDP match */
		T5_GTPU_FAIL_PKT_INDEX
	},

	/* Packet 16 */
	{
		(uint8_t *)gtpuPkt16,
		(pasahoLongInfo_t *)&gtpuPkt16Info,
		sizeof(gtpuPkt16),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_GTPU_FAIL_PKT_INDEX
	},

	/* Packet 17 */
	{
		(uint8_t *)gtpuPkt17,
		(pasahoLongInfo_t *)&gtpuPkt17Info,
		sizeof(gtpuPkt17),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FAIL_PKT_INDEX
	},

	/* Packet 18 */
	{
		(uint8_t *)gtpuPkt18,
		(pasahoLongInfo_t *)&gtpuPkt18Info,
		sizeof(gtpuPkt18),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FAIL_PKT_INDEX
	},

	/* Packet 19 */
	{
		(uint8_t *)gtpuPkt19,
		(pasahoLongInfo_t *)&gtpuPkt19Info,
		sizeof(gtpuPkt19),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FAIL_PKT_INDEX
	},

    #if 0

    // Temporary disable since fragments packet will be stuck at the RA engine.

	/* Packet 20 */
	{
		(uint8_t *)gtpuPkt20,
		(pasahoLongInfo_t *)&gtpuPkt20Info,
		sizeof(gtpuPkt20),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)|  	/* IP match */
          (1 << TF_STATS_BM_C1_IP_FRAG),
		  0 },                          																	  		/* UDP match */
		T5_IP_FRAG_PKT_INDEX
	},

    #endif

	{
		(uint8_t *)gtpuPkt21,
		(pasahoLongInfo_t *)&gtpuPkt21Info,
		sizeof(gtpuPkt21),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_MATCH_FAIL_PKT_INDEX
	}

};

/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0
 * TEID = 0x010000
 * Insert the PDU number (0xface) at the packet descriptor offset 20
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt0_2, ".testPkts")
#endif
static uint8_t gtpuPkt0_2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x4e,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x3e, 0x00, 0x01,
	0x00, 0x00, 0x6c, 0x6d, 0x01, 0xc0, 0x01, 0xfa,
	0xce, 0x00, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9
};


/* packet 1
 * mac dest = 00:e0:a6:66:57:09
 * ip dest = 200.201.202.101
 * Designed to match IP configuration 1
 * TEID = 0x01000d
 * Commands: Remove header/Insert 0/Remove Tail
 * Insert the PDU number (0xbeef) at the packet descriptor offset 20
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt1_2, ".testPkts")
#endif
static uint8_t gtpuPkt1_2[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x76, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x63, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x65, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x62,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x52, 0x00, 0x01,
	0x00, 0x0d, 0xba, 0xbe, 0x01, 0xc0, 0x01, 0xbe,
	0xef, 0x00, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb, 0xbc, 0xbd, 0x01, 0x02, 0x03, 0x04
};

/* Packet 2
 * mac dest - 00:01:02:03:04:aa
 * ip dest = 200.201.202.102
 * Designed to match IP configuration 2
 * TEID = 0x01001A
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt2_2, ".testPkts")
#endif
static uint8_t gtpuPkt2_2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x86, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x66, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x3e,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x2e, 0x00, 0x01,
	0x00, 0x1a, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99
};

/* Packet 3
 * mac dest = 00:e0:a6:66:57:09
 * ip dest = 200.201.202.103
 * Designed to match IP configuration 3
 * TEID = 0x01001B
 * Commands: Remove header/Insert 0/Remove Tail
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt3_2, ".testPkts")
#endif
static uint8_t gtpuPkt3_2[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x7b, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x67, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x48,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x38, 0x00, 0x01,
	0x00, 0x1b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3
};


/* Packet 4
 * mac dest = 00:01:02:03:04:aa
 * ip dest = 200.201.202.104
 * Designed to match IP configuration 4
 * TEID = 0x010028
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt4_2, ".testPkts")
#endif
static uint8_t gtpuPkt4_2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5e, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x68, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x64,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x54, 0x00, 0x01,
	0x00, 0x28, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf
};

/* Packet 5
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.105
 * Designed to match IP configuration 5
 * TEID = 0x010035
 * Commands: Remove header/Insert 0/Remove Tail
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt5_2, ".testPkts")
#endif
static uint8_t gtpuPkt5_2[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x74, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x61, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x69, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x60,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x50, 0x00, 0x01,
	0x00, 0x35, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9,
	0xba, 0xbb
};

/* Packet 6
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.106
 * Designed to match IP configuration 6
 * TEID = 0x010036
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt6_2, ".testPkts")
#endif
static uint8_t gtpuPkt6_2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x64, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6a, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x5c,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x4c, 0x00, 0x01,
	0x00, 0x36, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
	0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1,
	0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7
};

/* Packet 7
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 00.201.202.107
 * Designed to match IP configuration 7
 * TEID = 0x010043
 * Commands: Remove header/Insert 0/Remove Tail
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt7_2, ".testPkts")
#endif
static uint8_t gtpuPkt7_2[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5b, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6b, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x64,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x54, 0x00, 0x01,
	0x00, 0x43, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
	0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21,
	0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
	0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
	0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
	0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
	0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f
};

/* Packet 8
 * mac dest = 00:01:02:03:04:aa
 * IP dest = 200.201.202.108
 * Designed to match IP configuration 8
 * TEID = 0x010050
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt8_2, ".testPkts")
#endif
static uint8_t gtpuPkt8_2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x5a, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6c, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x64,
	0x00, 0x00, 0x30, 0xff, 0x00, 0x54, 0x00, 0x01,
	0x00, 0x50, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25
};

/* Packet 9
 * mac dest = 00:e0:a6:66:57:09
 * IP dest = 200.201.202.109
 * Designed to match IP configuration 9
 * TEID = 0x01005d
 * Commands: Remove header/Insert 0/Remove Tail
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (gtpuPkt9_2, ".testPkts")
#endif
static uint8_t gtpuPkt9_2[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x53, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x6d, 0xaa, 0xbb, 0x08, 0x68, 0x00, 0x6a,
	0x00, 0x5d, 0x30, 0xff, 0x00, 0x5a, 0x00, 0x01,
	0x00, 0x5d, 0x7e, 0xdf, 0x2c, 0xf5, 0x8a, 0xfb,
	0x18, 0x71, 0x56, 0xd7, 0xc4, 0xad, 0xe2, 0x73,
	0x30, 0xa9, 0x2e, 0xcf, 0x5c, 0x65, 0x3a, 0xeb,
	0x48, 0xe1, 0x06, 0xc7, 0xf4, 0x1d, 0x92, 0x63,
	0x60, 0x19, 0xde, 0xbf, 0x8c, 0xd5, 0xea, 0xdb,
	0x78, 0x51, 0xb6, 0xb7, 0x24, 0x8d, 0x42, 0x53,
	0x90, 0x89, 0x8e, 0xaf, 0xbc, 0x45, 0x9a, 0xcb,
	0xa8, 0xc1, 0x66, 0xa7, 0x54, 0xfd, 0xf2, 0x43,
	0xc0, 0xf9, 0x3e, 0x9f, 0xec, 0xb5, 0x4a, 0xbb,
	0xd8, 0x31, 0x16, 0x97, 0x84, 0x6d, 0xa2, 0x33,
	0xf0, 0x69, 0xee, 0x8f, 0x1c, 0x25, 0xfa, 0xab,
	0x08, 0xa1, 0xc6, 0x87
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (t5GTPUPktInfo2, ".testPkts")
#endif
static pktTestInfo_t t5GTPUPktInfo2[] =  {

	/* Packet 0 */
	{
		(uint8_t *)gtpuPkt0_2,
		(pasahoLongInfo_t *)&gtpuPkt0Info,
		sizeof(gtpuPkt0_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX
	},

	/* Packet 1 */
	{
		(uint8_t *)gtpuPkt1_2,
		(pasahoLongInfo_t *)&gtpuPkt1Info,
		sizeof(gtpuPkt1_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x0d
	},

	/* Packet 2 */
	{
		(uint8_t *)gtpuPkt2_2,
		(pasahoLongInfo_t *)&gtpuPkt2Info,
		sizeof(gtpuPkt2_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x1a
	},

	/* Packet 3 */
	{
		(uint8_t *)gtpuPkt3_2,
		(pasahoLongInfo_t *)&gtpuPkt3Info,
		sizeof(gtpuPkt3_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x1b
	},

	/* Packet 4 */
	{
		(uint8_t *)gtpuPkt4_2,
		(pasahoLongInfo_t *)&gtpuPkt4Info,
		sizeof(gtpuPkt4_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x28
	},

	/* Packet 5 */
	{
		(uint8_t *)gtpuPkt5_2,
		(pasahoLongInfo_t *)&gtpuPkt5Info,
		sizeof(gtpuPkt5_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x35
	},

	/* Packet 6 */
	{
		(uint8_t *)gtpuPkt6_2,
		(pasahoLongInfo_t *)&gtpuPkt6Info,
		sizeof(gtpuPkt6_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x36
	},


	/* Packet 7 */
	{
		(uint8_t *)gtpuPkt7_2,
		(pasahoLongInfo_t *)&gtpuPkt7Info,
		sizeof(gtpuPkt7_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x43
	},

	/* Packet 8 */
	{
		(uint8_t *)gtpuPkt8_2,
		(pasahoLongInfo_t *)&gtpuPkt8Info,
		sizeof(gtpuPkt8_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x50
	},

	/* Packet 9 */
	{
		(uint8_t *)gtpuPkt9_2,
		(pasahoLongInfo_t *)&gtpuPkt9Info,
		sizeof(gtpuPkt9_2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_GTPU_FIRST_PKT_INDEX + 0x5d
	},

};

/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * ip dest =  200.201.202.100
 * Designed to match NAT-T IPSEC 0
 * Src port = 4500 (0x1194), dest Port = 0x8000
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt0, ".testPkts")
#endif
static uint8_t natTPkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0x11, 0x94, 0x80, 0x00, 0x00, 0x4e,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x3e, 0x01, 0x00,
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
#pragma DATA_SECTION (natTPkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t natTPkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(112,0,0,PASAHO_HDR_ESP), /* end offset = 112, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t natTPkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 42 (UDP-Payload) */
	TF_FORM_PKT_INFO_WORD1(112,0,1,0,0,0),  /* end offset = 112, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_ESP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 1
 * mac dest = 00:01:02:03:04:aa
 * ip dest =  200.201.202.100
 * Designed to match NAT-T IPSEC
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt1, ".testPkts")
#endif
static uint8_t natTPkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0x80, 0x00, 0x11, 0x94, 0x00, 0x4e,
	0x00, 0x00, 0x34, 0xff, 0x00, 0x3f, 0x01, 0x00,
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
#pragma DATA_SECTION (natTPkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t natTPkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(112,0,0,PASAHO_HDR_ESP), /* end offset = 130, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t natTPkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 42 (UDP-Payload) */
	TF_FORM_PKT_INFO_WORD1(112,0,1,0,0,0),  /* end offset = 112, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_ESP,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 2
 * mac dest = 00:01:02:03:04:aa
 * ip dest =  200.201.202.100
 * Designed to match NAT-T CTRL
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt2, ".testPkts")
#endif
static uint8_t natTPkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0x80, 0x00, 0x11, 0x94, 0x00, 0x4e,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x6c, 0x6d, 0x01, 0xc0, 0x01, 0xfa,
	0xce, 0x00, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
	0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1,
	0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
	0xba, 0xbe, 0xfa, 0xce
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t natTPkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(108,pa_EROUTE_NAT_T_CTRL,0,PASAHO_HDR_UNKNOWN), /* end offset = 108, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t natTPkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP-Payload) */
	TF_FORM_PKT_INFO_WORD1(108,pa_EROUTE_NAT_T_CTRL,1,0,0,0),  /* end offset = 111(112), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif
/* packet 3
 * mac dest = 00:01:02:03:04:aa
 * ip dest =  200.201.202.100
 * Designed to match NAT-T Keep Alive
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt3, ".testPkts")
#endif
static uint8_t natTPkt3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0x80, 0x00, 0x11, 0x94, 0x00, 0x09,
	0x00, 0x00, 0xFF
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t natTPkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(43,pa_EROUTE_NAT_T_KEEPALIVE,0,PASAHO_HDR_UNKNOWN), /* end offset = 43, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t natTPkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP-Payload) */
	TF_FORM_PKT_INFO_WORD1(43,pa_EROUTE_NAT_T_KEEPALIVE,1,0,0,0),  /* end offset = 43, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 4
 * mac dest = 00:01:02:03:04:aa
 * ip dest =  200.201.202.100
 * Designed to match NAT-T Keep Alive (Fail)
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt4, ".testPkts")
#endif
static uint8_t natTPkt4[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0x80, 0x00, 0x11, 0x94, 0x00, 0x09,
	0x00, 0x00, 0xcc
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt4Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t natTPkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(43,pa_EROUTE_NAT_T_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 43, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t natTPkt4Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP-Payload) */
	TF_FORM_PKT_INFO_WORD1(43,pa_EROUTE_NAT_T_FAIL,1,0,0,0),  /* end offset = 43, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 5
 * mac dest = 00:01:02:03:04:aa
 * ip dest =  200.201.202.100
 * Designed to match NAT-T Parsing Error
 * Src port = 0x8000, dest Port = 4500 (0x1194)
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt5, ".testPkts")
#endif
static uint8_t natTPkt5[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x01,
	0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x16, 0x78, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0x80, 0x00, 0x11, 0x94, 0x00, 0x0a,
	0x00, 0x00, 0xff, 0x00
};


#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt5Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t natTPkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(44,pa_EROUTE_NAT_T_FAIL,0,PASAHO_HDR_UNKNOWN), /* end offset = 44, errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t natTPkt5Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP-Payload) */
	TF_FORM_PKT_INFO_WORD1(44,pa_EROUTE_NAT_T_FAIL,1,0,0,0),   /* end offset = 44, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifndef NSS_GEN2
/* packet 6
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.83.84
 * out ip dest = 200.201.202.100 (out IP info 0)
 * fake IPSEC ESP NAT-T packet
 * Designed to match IP configuration (SPI = 0x44440001) Entry 1
 * Src port = 0xaabb, dest Port = 4500 (0x1194)
 * pkt Index: pkt[15] = 0
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (natTPkt6, ".testPkts")
#endif
static uint8_t natTPkt6[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x08, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x74, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x11, 0x94, 0x00, 0x60,
    0x00, 0x00, 0x44, 0x44, 0x00, 0x01, 0x00, 0x00,
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
#pragma DATA_SECTION (natTPkt6Info, ".testPkts")
#endif
static pasahoLongInfo_t natTPkt6Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (ESP) */
	TF_FORM_PKT_INFO_WORD1(130,0,1,0,0,0),   /* end offset = 130, errIdx = 0 (no longer an error as it is routed to PDSP1 for SPI processing), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_ESP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

typedef struct pktTestInfo2_s {
  pktTestInfo_t info;
	paStatsBmap_t statsMap[3];  /* Bit map of which stats to increment */
}pktTestInfo2_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION (t5NatTPktInfo, ".testPkts")
#endif
static pktTestInfo2_t t5NatTPktInfo[] =  {
	/* Packet 0 */
	{
   { 
		(uint8_t *)natTPkt0,
		(pasahoLongInfo_t *)&natTPkt0Info,
		sizeof(natTPkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_NAT_T_DATA_PKT_INDEX + 0,
   },
	 {  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                    /*  SPI match */
		   0,
		   0 
	 }
	},

	/* Packet 1 */
	{
   { 
		(uint8_t *)natTPkt1,
		(pasahoLongInfo_t *)&natTPkt1Info,
		sizeof(natTPkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_NAT_T_DATA_PKT_INDEX + 1,
   },
		{  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                    /*  SPI match */
		   0,
		   0 },
	},

	/* Packet 2 */
	{
   { 
		(uint8_t *)natTPkt2,
		(pasahoLongInfo_t *)&natTPkt2Info,
		sizeof(natTPkt2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_NAT_T_CTRL_PKT_INDEX,
   },
		{  0,
		   0,
		   0 },
	},

	/* Packet 3 */
	{
   { 
		(uint8_t *)natTPkt3,
		(pasahoLongInfo_t *)&natTPkt3Info,
		sizeof(natTPkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_NAT_T_KEEPALIVE_PKT_INDEX,
   },
		{  0,
		   0,
		   0 },
	},

	/* Packet 4 */
	{
   { 
		(uint8_t *)natTPkt4,
		(pasahoLongInfo_t *)&natTPkt4Info,
		sizeof(natTPkt4),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) },								  		/* UDP match */
		T5_NAT_T_FAIL_PKT_INDEX,
    },
		{  0,
		   0,
		   0 },
	},

	/* Packet 5 */
	{
   { 
		(uint8_t *)natTPkt5,
		(pasahoLongInfo_t *)&natTPkt5Info,
		sizeof(natTPkt5),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_NAT_T_FAIL_PKT_INDEX,
   },
		{  0,
		   0,
		   0 },
	},



#ifndef NSS_GEN2

	/* Packet 6 */
	{
   { 
		(uint8_t *)natTPkt6,
		(pasahoLongInfo_t *)&natTPkt6Info,
		sizeof(natTPkt6),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T5_NAT_T_DATA_PKT_INDEX + 2,
    },
		{  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                    /*  SPI match */
		    0,
		    0
		},
  },
#endif
};

#endif /*TEST5PKTS_H_*/
