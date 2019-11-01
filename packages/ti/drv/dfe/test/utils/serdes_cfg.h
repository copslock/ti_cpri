#ifndef __SERDES_CFG_H__
#define __SERDES_CFG_H__
//
#define  DFE_REFCLK122P88MHz_6P144Gbps  0
#define  DFE_REFCLK122P88MHz_3P072Gbps  1
#define  DFE_REFCLK122P88MHz_7P3728Gbps 2
#define  DFE_REFCLK122P88MHz_3P6864Gbps 3	// P1_2, P1_4
#define  DFE_REFCLK122P88MHz_9P8304Gbps 4
#define  DFE_REFCLK122P88MHz_4P9152Gbps 5	// P1_3, P1_5
#define  DFE_REFCLK122P88MHz_2P4576Gbps 6 	// p0_2, P1_1
#define  DFE_REFCLK153P60MHz_6P144Gbps  7
#define  DFE_REFCLK153P60MHz_3P072Gbps  8
#define  IQN_REFCLK122P88MHz_4P9152Gbps 9
#define  IQN_REFCLK122P88MHz_6P144Gbps  10
#define  IQN_REFCLK122P88MHz_9P8304Gbps 11

//void serdes_cfg(unsigned int serdes_setup);
//void serdes_write_byte(unsigned int addr, unsigned int data);
void serdes_cfg_csl(unsigned int serdes_setup);

#endif // __SERDES_CFG_H__
