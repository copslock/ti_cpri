//**************************************************************************************
//*  Serdes Cfg reg file : dfe_config_wiz8b8sb_cfg_setup_refclk122p88MHz_20bit_7p3728Gbps_2l1c.reg
//*  Label @ SS level    : DFESS_SERDESVSEQ_20130430_2
//*  Created by          : Manjunath Krishnegowda x0103755
//*  Date                : 2013-05-01 15:34:39
//**************************************************************************************
void serdes_setup_7p3728(UINT serdes_inst)
{
   UINT base;
  if(serdes_inst==0)
    base=csisc2_serdes0_cfg_base;
  else  
    base=csisc2_serdes1_cfg_base;

serdes_write_byte(base + 0x00000003, 0x00);
serdes_write_byte(base + 0x00000014, 0x42);
serdes_write_byte(base + 0x00000015, 0x42);
serdes_write_byte(base + 0x00000060, 0x70);
serdes_write_byte(base + 0x00000061, 0x80);
serdes_write_byte(base + 0x00000062, 0x14);
serdes_write_byte(base + 0x00000065, 0xc7);
serdes_write_byte(base + 0x00000066, 0xc3);
serdes_write_byte(base + 0x00000079, 0xc0);
serdes_write_byte(base + 0x00000204, 0x80);
serdes_write_byte(base + 0x00000207, 0x7e);
serdes_write_byte(base + 0x00000208, 0x24);
serdes_write_byte(base + 0x0000020f, 0x02);
serdes_write_byte(base + 0x00000213, 0x1b);
serdes_write_byte(base + 0x00000214, 0x0c);
serdes_write_byte(base + 0x00000215, 0x70);
serdes_write_byte(base + 0x00000218, 0x84);
serdes_write_byte(base + 0x0000021a, 0x80);
serdes_write_byte(base + 0x0000021b, 0x75);
serdes_write_byte(base + 0x0000022e, 0x10);
serdes_write_byte(base + 0x00000280, 0x44);
serdes_write_byte(base + 0x00000282, 0x44);
serdes_write_byte(base + 0x00000284, 0x85);
serdes_write_byte(base + 0x00000286, 0x0f);
serdes_write_byte(base + 0x00000287, 0x1d);
serdes_write_byte(base + 0x0000028d, 0x3b);
serdes_write_byte(base + 0x00000404, 0x80);
serdes_write_byte(base + 0x00000407, 0x7e);
serdes_write_byte(base + 0x00000408, 0x24);
serdes_write_byte(base + 0x0000040f, 0x02);
serdes_write_byte(base + 0x00000413, 0x1b);
serdes_write_byte(base + 0x00000414, 0x0c);
serdes_write_byte(base + 0x00000415, 0x70);
serdes_write_byte(base + 0x00000418, 0x84);
serdes_write_byte(base + 0x0000041a, 0x80);
serdes_write_byte(base + 0x0000041b, 0x75);
serdes_write_byte(base + 0x0000042e, 0x10);
serdes_write_byte(base + 0x00000480, 0x44);
serdes_write_byte(base + 0x00000482, 0x44);
serdes_write_byte(base + 0x00000484, 0x85);
serdes_write_byte(base + 0x00000486, 0x0f);
serdes_write_byte(base + 0x00000487, 0x1d);
serdes_write_byte(base + 0x0000048d, 0x3b);
serdes_write_byte(base + 0x00000a01, 0x08);
serdes_write_byte(base + 0x00000a0a, 0xb2);
serdes_write_byte(base + 0x00000a0b, 0x34);
serdes_write_byte(base + 0x00000a31, 0x4b);
serdes_write_byte(base + 0x00000a32, 0x4b);
serdes_write_byte(base + 0x00000a85, 0x06);
serdes_write_byte(base + 0x00000a97, 0x10);
serdes_write_byte(base + 0x00000aa3, 0x81);
serdes_write_byte(base + 0x00000abf, 0xff);
serdes_write_byte(base + 0x00000ac0, 0x8b);
serdes_write_byte(base + 0x00000b07, 0x02);
serdes_write_byte(base + 0x00000b0a, 0x3f);
serdes_write_byte(base + 0x00000b0b, 0x56);
serdes_write_byte(base + 0x00000b0c, 0x4e);
serdes_write_byte(base + 0x00000000, 0x03);
serdes_write_byte(base + 0x00000a00, 0x5f);
} //End of function
