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



#ifndef TEST16PKTS_H_
#define TEST16PKTS_H_


#ifdef NSS_GEN2
#define T16_SOC_MAX_LUT2_SIZE (16384)
#else
#define T16_SOC_MAX_LUT2_SIZE (8192)
#endif

/* Valid rx MAC addresses used during the test */
typedef struct t16EthSetup  {
	paEthInfo_t  ethInfo;		/* PA Ethernet configuration structure */
	Bool	     acked;			/* Set to TRUE when the reply to the command is received */
} t16EthSetup_t;


#ifdef _TMS320C6X
#pragma DATA_SECTION (t16EthSetup, ".testPkts")
#endif
static t16EthSetup_t t16EthSetup[] =  {

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

typedef struct t16IpSetup_s  {
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */

} t16IpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t16IpSetup, ".testPkts")
#endif
static t16IpSetup_t  t16IpSetup[] = {

	{
		/* IP Entry 0 */
		0,		/* Linked to dest mac index 0 */
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
	{	1,		/* Linked to dest mac index 1 */
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
	{	0,		/* Linked to dest mac index 0 */
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
	{	1,		/* Linked to dest mac index 1 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
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

#endif /*TEST16PKTS_H_*/
