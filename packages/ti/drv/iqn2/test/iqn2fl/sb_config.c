
/* Snowbush SERDES configuration for IQN2   */
#include <ti/drv/iqn2/iqn2fl.h>
#include "IQN2_config.h"

// Register/Memory Read/Write Functions
void reg_write_32(Uint32 addr,Uint32 write_data)
{
  *(Uint32 *)(addr) = write_data;
}
// Register/Memory Read/Write Functions
void reg_write_16(Uint32 addr,Uint16 write_data)
{
  *(Uint16 *)(addr) = write_data;
}
// Register/Memory Read/Write Functions
void reg_write_8(Uint32 addr,Uint8 write_data)
{
  *(Uint8 *)(addr) = write_data;
}
// RAPID Register/Memory Read/Write Functions
void reg_write(Uint32 wdth, Uint32 addrs, Uint32 wr_data) 
{
  if(wdth == 16) {
    reg_write_16(addrs, wr_data);
  } else if(wdth == 8) {
    reg_write_8(addrs, wr_data);
  } else {
    reg_write_32(addrs, wr_data);
  }
}

void write_32_mask(Uint32 addr,Uint32 mask,Uint32 write_data)
{
	Uint32 read_data, data;

	read_data = (*(Uint32 *) (addr));
	data = (write_data & ~mask ) | (read_data & mask);
	(*(Uint32 *) (addr)) = data;
}

/* if reg_write_8() doesn't work correct with SERDES, this should be the safe access
because the Lamarr SERDES WIZ might cause wrong operation when try to access 8 bit level */
void serdes_write_byte(Uint32 addr, Uint32 data)
{
      Uint32 write_value;
      write_value  = *(volatile Uint32 *)(addr & 0xFFFFFFFC);
     write_value &= ~(0x000000FF << ((addr & 0x3) * 8));
      write_value |= (data << ((addr & 0x3) * 8));
   *(volatile Uint32 *)(addr & 0xFFFFFFFC) = write_value;
}

void reg_poll(Uint32 len, Uint32 addr,Uint32 read_mask, Uint32 read_data)
{
	while ( (*(Uint32*)(addr) & read_mask) != read_data)
	{
		asm("NOP");
	}
}

void reg_read_check (int a, int b, int c, int d){}


void SB_config_6p144 (Uint8 LinkRate)//used for OBSAI 2x,4x,8x and CPRI 5x, 10x
{
    reg_write(32,0x27000004,0x00000003);
    reg_poll(32,0x27000004,0x00000003,0x00000003);//reg write function ability test

    //SB core setup for IQN2 for reference clock 122.88 MHz

    //PhyA CMU subsystem setup
    serdes_write_byte((0x02324000 + 0x0003),0x00);
    serdes_write_byte((0x02324000 + 0x0014),0x5e);
    serdes_write_byte((0x02324000 + 0x0015),0x5e);
    serdes_write_byte((0x02324000 + 0x0060),0x5c);
    serdes_write_byte((0x02324000 + 0x0061),0xb8);
    serdes_write_byte((0x02324000 + 0x0062),0x13);
    serdes_write_byte((0x02324000 + 0x0065),0xc7);
    serdes_write_byte((0x02324000 + 0x0066),0xc3);
    serdes_write_byte((0x02324000 + 0x0079),0xc0);

    //PhyA Lane (lane 0) subsystem setup  (fixed for Lamarr)
    serdes_write_byte((0x02324000 + 0x0204),0x80);
    serdes_write_byte((0x02324000 + 0x0207),0x7e);
    serdes_write_byte((0x02324000 + 0x0208),0x24);
    serdes_write_byte((0x02324000 + 0x020f),0x02);
    serdes_write_byte((0x02324000 + 0x0213),0x1b);
    serdes_write_byte((0x02324000 + 0x0214),0x94);
    serdes_write_byte((0x02324000 + 0x0215),0x6f);
    serdes_write_byte((0x02324000 + 0x0218),0x84);
    serdes_write_byte((0x02324000 + 0x021a),0x80);
    serdes_write_byte((0x02324000 + 0x021b),0x75);
    serdes_write_byte((0x02324000 + 0x022e),0x20);
    serdes_write_byte((0x02324000 + 0x0280),0x54);
    serdes_write_byte((0x02324000 + 0x0282),0x54);
    serdes_write_byte((0x02324000 + 0x0284),0x85);
    serdes_write_byte((0x02324000 + 0x0286),0x0f);
    serdes_write_byte((0x02324000 + 0x0287),0x1d);
    serdes_write_byte((0x02324000 + 0x028d),0x3b);

    //PhyA Lane (lane 1) subsystem setup (fixed for Lamarr)
    serdes_write_byte((0x02324000 + 0x0404),0x80);
    serdes_write_byte((0x02324000 + 0x0407),0x7e);
    serdes_write_byte((0x02324000 + 0x0408),0x24);
    serdes_write_byte((0x02324000 + 0x040f),0x02);
    serdes_write_byte((0x02324000 + 0x0413),0x1b);
    serdes_write_byte((0x02324000 + 0x0414),0x94);
    serdes_write_byte((0x02324000 + 0x0415),0x6f);
    serdes_write_byte((0x02324000 + 0x0418),0x84);
    serdes_write_byte((0x02324000 + 0x041a),0x80);
    serdes_write_byte((0x02324000 + 0x041b),0x75);
    serdes_write_byte((0x02324000 + 0x042e),0x20);
    serdes_write_byte((0x02324000 + 0x0480),0x54);
    serdes_write_byte((0x02324000 + 0x0482),0x54);
    serdes_write_byte((0x02324000 + 0x0484),0x85);
    serdes_write_byte((0x02324000 + 0x0486),0x0f);
    serdes_write_byte((0x02324000 + 0x0487),0x1d);
    serdes_write_byte((0x02324000 + 0x048d),0x3b);

    //PhyA common lane subsystem (fixed for Lamarr)
    serdes_write_byte((0x02324000 + 0x0a01),0x08);
    serdes_write_byte((0x02324000 + 0x0a0a),0xb2);
    serdes_write_byte((0x02324000 + 0x0a0b),0x35);
    serdes_write_byte((0x02324000 + 0x0a31),0x5b);
    serdes_write_byte((0x02324000 + 0x0a32),0x5b);
    serdes_write_byte((0x02324000 + 0x0a85),0x06);
    serdes_write_byte((0x02324000 + 0x0a97),0x10);
    serdes_write_byte((0x02324000 + 0x0aa3),0x81);
    serdes_write_byte((0x02324000 + 0x0abf),0xff);
    serdes_write_byte((0x02324000 + 0x0ac0),0x8b);
    serdes_write_byte((0x02324000 + 0x0b07),0x02);
    serdes_write_byte((0x02324000 + 0x0b0a),0x3f);
    serdes_write_byte((0x02324000 + 0x0b0b),0x56);
    serdes_write_byte((0x02324000 + 0x0b0c),0x4e);
    serdes_write_byte((0x02324000 + 0x0000),0x03);//Master reset for CMU and setup a pll control option
    serdes_write_byte((0x02324000 + 0x0a00),0x5f);//Reset all lanes to make it start

    serdes_write_byte(0x02324000 + 0x0000022b, 0x0);
    serdes_write_byte(0x02324000 + 0x0000042b, 0x0);

    /* lane control and status (Rate setup ==> bit 27:26 for Tx and 11:10 for Rx. full : 0, Half : 1, Quad: 2)*/
    if(LinkRate == 0){//Rx full rate. Tx is always full rate
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F000 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F000 );//lane 1
    }
    else if(LinkRate == 1){//Rx half rate. Tx is always full rate
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F400 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F400 );//lane 1
    }
    else {//Rx Quad rate. Tx is always full rate
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F800 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F800 );//lane 1
    }

    write_32_mask(0x02324000+0x1ff4, 0x0FFFFFFF, 0xE0000000 );
    reg_poll(32,0x02324000 + 0x1ff4, 0xF0000300, 0xF0000300 ); // PLL on and wait for pll lock and lane status is ok

}

void SB_config_4p9152 (Iqn2Fl_LinkRate LinkRate)//used for CPRI 2x,4x,8x
{
	reg_write(32,0x27000004,0x00000003);
	reg_poll(32,0x27000004,0x00000003,0x00000003);//reg write function ability test

	//SB core setup for IQN2 for reference clock 122.88 MHz

	//PhyA CMU subsystem setup
    serdes_write_byte((0x02324000 + 0x0003),0x00);
    serdes_write_byte((0x02324000 + 0x0014),0x82);
    serdes_write_byte((0x02324000 + 0x0015),0x82);
    serdes_write_byte((0x02324000 + 0x0060),0x48);
    serdes_write_byte((0x02324000 + 0x0061),0x2c);
    serdes_write_byte((0x02324000 + 0x0062),0x13);
    serdes_write_byte((0x02324000 + 0x0065),0xc7);
    serdes_write_byte((0x02324000 + 0x0066),0xc3);
    serdes_write_byte((0x02324000 + 0x0079),0xc0);

    //PhyA Lane (lane 0) subsystem setup (updated for Lamarr)
    serdes_write_byte((0x02324000 + 0x0204),0x80);
    serdes_write_byte((0x02324000 + 0x0207),0x7e);
    serdes_write_byte((0x02324000 + 0x0208),0x24);
    serdes_write_byte((0x02324000 + 0x020f),0x02);
    serdes_write_byte((0x02324000 + 0x0213),0x1b);
    serdes_write_byte((0x02324000 + 0x0214),0x90);
    serdes_write_byte((0x02324000 + 0x0215),0x6f);
    serdes_write_byte((0x02324000 + 0x0218),0xe4);
    serdes_write_byte((0x02324000 + 0x021a),0x80);
    serdes_write_byte((0x02324000 + 0x021b),0x75);
    serdes_write_byte((0x02324000 + 0x022e),0x20);
    serdes_write_byte((0x02324000 + 0x0280),0x69);
    serdes_write_byte((0x02324000 + 0x0282),0x69);
    serdes_write_byte((0x02324000 + 0x0284),0x85);
    serdes_write_byte((0x02324000 + 0x0286),0x0f);
    serdes_write_byte((0x02324000 + 0x0287),0x1d);
    serdes_write_byte((0x02324000 + 0x028d),0x2c);

    //PhyA Lane (lane 1) subsystem setup (updated for Lamarr)
    serdes_write_byte((0x02324000 + 0x0404),0x80);
    serdes_write_byte((0x02324000 + 0x0407),0x7e);
    serdes_write_byte((0x02324000 + 0x0408),0x24);
    serdes_write_byte((0x02324000 + 0x040f),0x02);
    serdes_write_byte((0x02324000 + 0x0413),0x1b);
    serdes_write_byte((0x02324000 + 0x0414),0x90);
    serdes_write_byte((0x02324000 + 0x0415),0x6f);
    serdes_write_byte((0x02324000 + 0x0418),0xe4);
    serdes_write_byte((0x02324000 + 0x041a),0x80);
    serdes_write_byte((0x02324000 + 0x041b),0x75);
    serdes_write_byte((0x02324000 + 0x042e),0x20);
    serdes_write_byte((0x02324000 + 0x0480),0x69);
    serdes_write_byte((0x02324000 + 0x0482),0x69);
    serdes_write_byte((0x02324000 + 0x0484),0x85);
    serdes_write_byte((0x02324000 + 0x0486),0x0f);
    serdes_write_byte((0x02324000 + 0x0487),0x1d);
    serdes_write_byte((0x02324000 + 0x048d),0x2c);

    //PhyA common lane subsystem (updated for Lamarr)
    serdes_write_byte((0x02324000 + 0x0a01),0x08);
    serdes_write_byte((0x02324000 + 0x0a0a),0x02);
    serdes_write_byte((0x02324000 + 0x0a0b),0x37);
    serdes_write_byte((0x02324000 + 0x0a31),0x70);
    serdes_write_byte((0x02324000 + 0x0a32),0x70);
    serdes_write_byte((0x02324000 + 0x0a85),0x06);
    serdes_write_byte((0x02324000 + 0x0a97),0x10);
    serdes_write_byte((0x02324000 + 0x0aa3),0x81);
    serdes_write_byte((0x02324000 + 0x0abf),0xff);
    serdes_write_byte((0x02324000 + 0x0ac0),0x8b);
    serdes_write_byte((0x02324000 + 0x0b07),0x02);
    serdes_write_byte((0x02324000 + 0x0b0a),0x3f);
    serdes_write_byte((0x02324000 + 0x0b0b),0x56);
    serdes_write_byte((0x02324000 + 0x0b0c),0x4e);
    serdes_write_byte((0x02324000 + 0x0000),0x03);//Master reset for CMU and setup a pll control option
    serdes_write_byte((0x02324000 + 0x0a00),0x5f);//Reset all lanes to make it start

    serdes_write_byte(0x02324000 + 0x0000022b, 0x0);
    serdes_write_byte(0x02324000 + 0x0000042b, 0x0);

    /* lane control and status (Rate setup ==> bit 27:26 for Tx and 11:10 for Rx. full : 0, Half : 1, Quad: 2)*/
    if(LinkRate == IQN2FL_LINK_RATE_8x){//Rx full rate. Tx is always full rate
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F000 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F000 );//lane 1
    }
    else if(LinkRate == IQN2FL_LINK_RATE_4x){//Rx half rate. Tx is always full rate
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F400 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F400 );//lane 1
    }
    else {//Rx Quad rate. Tx is always full rate
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F800 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F800 );//lane 1
    }

    write_32_mask(0x02324000+0x1ff4, 0x0FFFFFFF, 0xE0000000 );
    reg_poll(32,0x02324000 + 0x1ff4, 0xF0000300, 0xF0000300 ); // PLL on and wait for pll lock and lane status is ok

}


void SB_config_9p8304 ()//used for CPRI 16x
{

//SB core setup for IQN2 for reference clock 122.88 MHz

serdes_write_byte(0x02324000 + 0x00000003, 0x00); 
serdes_write_byte(0x02324000 + 0x00000014, 0x3d); 
serdes_write_byte(0x02324000 + 0x00000015, 0x3d); 
serdes_write_byte(0x02324000 + 0x00000060, 0x98); 
serdes_write_byte(0x02324000 + 0x00000061, 0xb8); 
serdes_write_byte(0x02324000 + 0x00000062, 0x13);
serdes_write_byte(0x02324000 + 0x00000065, 0xc7); 
serdes_write_byte(0x02324000 + 0x00000066, 0xc3); 
serdes_write_byte(0x02324000 + 0x00000079, 0xc0); 

//link setup has fix for Lamarr
serdes_write_byte(0x02324000 + 0x00000201, 0x80);
serdes_write_byte(0x02324000 + 0x00000204, 0x80); 
serdes_write_byte(0x02324000 + 0x00000207, 0x7e); 
serdes_write_byte(0x02324000 + 0x00000208, 0x24); 
serdes_write_byte(0x02324000 + 0x0000020f, 0x02); 
serdes_write_byte(0x02324000 + 0x00000213, 0x1b); 
serdes_write_byte(0x02324000 + 0x00000214, 0x8c); 
serdes_write_byte(0x02324000 + 0x00000215, 0x6d); 
serdes_write_byte(0x02324000 + 0x00000218, 0x84); 
serdes_write_byte(0x02324000 + 0x0000021a, 0x80); 
serdes_write_byte(0x02324000 + 0x0000021b, 0x75); 
serdes_write_byte(0x02324000 + 0x00000280, 0x3b); 
serdes_write_byte(0x02324000 + 0x00000282, 0x3b); 
serdes_write_byte(0x02324000 + 0x00000284, 0x95);
serdes_write_byte(0x02324000 + 0x00000286, 0x0f); 
serdes_write_byte(0x02324000 + 0x00000287, 0x1d); 
serdes_write_byte(0x02324000 + 0x0000028d, 0x3b); 
//link setup has fix for Lamarr
serdes_write_byte(0x02324000 + 0x00000401, 0x80); 
serdes_write_byte(0x02324000 + 0x00000404, 0x80); 
serdes_write_byte(0x02324000 + 0x00000407, 0x7e); 
serdes_write_byte(0x02324000 + 0x00000408, 0x24); 
serdes_write_byte(0x02324000 + 0x0000040f, 0x02); 
serdes_write_byte(0x02324000 + 0x00000413, 0x1b); 
serdes_write_byte(0x02324000 + 0x00000414, 0x8c); 
serdes_write_byte(0x02324000 + 0x00000415, 0x6d);
serdes_write_byte(0x02324000 + 0x00000418, 0x84); 
serdes_write_byte(0x02324000 + 0x0000041a, 0x80); 
serdes_write_byte(0x02324000 + 0x0000041b, 0x75); 
serdes_write_byte(0x02324000 + 0x00000480, 0x3b); 
serdes_write_byte(0x02324000 + 0x00000482, 0x3b); 
serdes_write_byte(0x02324000 + 0x00000484, 0x95);
serdes_write_byte(0x02324000 + 0x00000486, 0x0f); 
serdes_write_byte(0x02324000 + 0x00000487, 0x1d); 
serdes_write_byte(0x02324000 + 0x0000048d, 0x3b); 

serdes_write_byte(0x02324000 + 0x00000a01, 0x08); 
serdes_write_byte(0x02324000 + 0x00000a0a, 0x22); 
serdes_write_byte(0x02324000 + 0x00000a0b, 0x34); 
serdes_write_byte(0x02324000 + 0x00000a31, 0x42); 
serdes_write_byte(0x02324000 + 0x00000a32, 0x77); 
serdes_write_byte(0x02324000 + 0x00000a85, 0x06); 
serdes_write_byte(0x02324000 + 0x00000a97, 0x10); 
serdes_write_byte(0x02324000 + 0x00000aa3, 0x81); 
serdes_write_byte(0x02324000 + 0x00000abf, 0xff); 
serdes_write_byte(0x02324000 + 0x00000ac0, 0x8b); 
serdes_write_byte(0x02324000 + 0x00000b07, 0x02); 
serdes_write_byte(0x02324000 + 0x00000b0a, 0x3f); 
serdes_write_byte(0x02324000 + 0x00000b0b, 0x56); 
serdes_write_byte(0x02324000 + 0x00000b0c, 0x4e); 
serdes_write_byte(0x02324000 + 0x00000000, 0x03); 
serdes_write_byte(0x02324000 + 0x00000a00, 0x5f);  //Reset all lanes to make it start

serdes_write_byte(0x02324000 + 0x0000022b, 0x0);
serdes_write_byte(0x02324000 + 0x0000042b, 0x0);

/* when SERDES set to 9.8G, the lane is used for CPRI 16x with full rate setup for both Tx and Rx
 * if user wants to use 8x or 4x speed with this configuration, Tx rate setup should be half and Rx rate setup can be half for 8x and quad for 4x
 */
    /* lane control and status (Rate setup ==> bit 27:26 for Tx and 11:10 for Rx. full : 0, Half : 1, Quad: 2)*/
    write_32_mask(0x02324000+0x1fe0, 0x0000000F, 0xF000F000 );//lane 0
    write_32_mask(0x02324000+0x1fe4, 0x0000000F, 0xF000F000 );//lane 1

    write_32_mask(0x02324000+0x1ff4, 0x0FFFFFFF, 0xE0000000 );
    reg_poll(32,0x02324000 + 0x1ff4, 0xF0000300, 0xF0000300 ); // PLL on and wait for pll lock and lane status is ok

}


void SD_NES_Loopback()
{
   //AIL lane 0 and 1
   //setup links for SERDES digital loopback mode
   serdes_write_byte(0x02324000+0x203,0x40); // Snowbush B4 lane 0 register  NES loopback enable
   serdes_write_byte(0x02324000+0x403,0x40); // Snowbush B4 lane 1 register  NES loopback enable

}
