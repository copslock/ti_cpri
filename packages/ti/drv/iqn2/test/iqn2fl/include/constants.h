
#ifndef CONST_H
#define CONST_H

#ifndef PASS
#define PASS		(1)
#endif

#ifndef FAIL
#define FAIL		(0)
#endif

#ifndef TRUE
#define TRUE		1
#endif

#ifndef FALSE
#define FALSE		0
#endif

#ifndef NULL
#define NULL  		0
#endif

// bit 2 - wave:
// 0 -> Keystone 1
// 1 -> Keystone 2
// bit 1 - master die:
// 1 -> Kepler Core
// 2 -> Napier Core
// bit 0 - SoC/device:

// KeyStone I SOC's. They are here to prevent reuse of constants. Tests
// That use these will fail to compile when these are commented out.

// KeyStone II Cores

#define KEPLER_CORE 0xFFFF0110 //KPC
#define lamarr_core 0xFFFF0110 //LMC (same value as KPC for bringup)
#define NAPIER_CORE 0xFFFF0120 //NPC

// KeyStone II SOC based on kepler_core (0xFFFF0110)

#define KEPLER          0xFFFF0111 //KP
#define KEPLER2         0xFFFF0112 //K2
#define HAWKING         0xFFFF0113 //HK

// KeyStone II SOC based on napier_core (0xFFFF0120)

#define SHANNON28       0xFFFF0121 //S2
#define NAPIERLITE      0xFFFF0122 //NL
#define EDISON          0xFFFF0123 //ED
#define EDISONLITE      0xFFFF0124 //EL

// KeyStone II GEM/ARM count

#define KPC_GEM_COUNT 	8
#define NPC_GEM_COUNT 	8
#define LMC_GEM_COUNT 	4
#define KPC_ARM_COUNT 	4
#define NPC_ARM_COUNT 	4
#define LMC_ARM_COUNT 	2

#define MAX_GEMS     	LMC_GEM_COUNT
#define MAX_ARMS     	LMC_ARM_COUNT

#define ACCESS_VALID	0
#define ACCESS_RSV	1

#define STAGE_CNT        4
#define WRONLY          8
#define RDONLY          9
#define RDWR            10
#define ALL             11

#define RW_TYPE_CNT      7
#define LMB	  	32
#define LB	  	33
#define RB	  	34
#define RMB	  	35
#define LHW	  	36
#define RHW	  	37
#define FW	  	38

#endif

