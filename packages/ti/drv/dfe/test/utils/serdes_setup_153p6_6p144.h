void serdes_setup_153p6_6p144(UINT serdes_inst)
{
   UINT base;
  if(serdes_inst==0)
    base=csisc2_serdes0_cfg_base;
  else  
    base=csisc2_serdes1_cfg_base;

serdes_write_byte(base + 0x00000001, 0x18);
serdes_write_byte(base + 0x00000003, 0x00);
serdes_write_byte(base + 0x00000014, 0x3d);
serdes_write_byte(base + 0x00000015, 0x3d);
serdes_write_byte(base + 0x00000060, 0x48);
serdes_write_byte(base + 0x00000061, 0xb8);
serdes_write_byte(base + 0x00000062, 0x13);
serdes_write_byte(base + 0x00000065, 0xc7);
serdes_write_byte(base + 0x00000066, 0xc3);
serdes_write_byte(base + 0x00000079, 0xc0);
serdes_write_byte(base + 0x00000404, 0x80);
serdes_write_byte(base + 0x00000407, 0x7e);
serdes_write_byte(base + 0x00000408, 0x24);
serdes_write_byte(base + 0x0000040f, 0x02);
serdes_write_byte(base + 0x00000413, 0x1b);
serdes_write_byte(base + 0x00000414, 0x94);
serdes_write_byte(base + 0x00000415, 0x6f);
serdes_write_byte(base + 0x00000418, 0x84);
serdes_write_byte(base + 0x0000041a, 0x80);
serdes_write_byte(base + 0x0000041b, 0x75);
serdes_write_byte(base + 0x0000042e, 0x10);
serdes_write_byte(base + 0x00000480, 0x69);
serdes_write_byte(base + 0x00000482, 0x69);
serdes_write_byte(base + 0x00000484, 0x85);
serdes_write_byte(base + 0x00000486, 0x0f);
serdes_write_byte(base + 0x00000487, 0x1d);
serdes_write_byte(base + 0x0000048d, 0x3b);
serdes_write_byte(base + 0x00000604, 0x80);
serdes_write_byte(base + 0x00000607, 0x7e);
serdes_write_byte(base + 0x00000608, 0x24);
serdes_write_byte(base + 0x0000060f, 0x02);
serdes_write_byte(base + 0x00000613, 0x1b);
serdes_write_byte(base + 0x00000614, 0x94);
serdes_write_byte(base + 0x00000615, 0x6f);
serdes_write_byte(base + 0x00000618, 0x84);
serdes_write_byte(base + 0x0000061a, 0x80);
serdes_write_byte(base + 0x0000061b, 0x75);
serdes_write_byte(base + 0x0000062e, 0x10);
serdes_write_byte(base + 0x00000680, 0x69);
serdes_write_byte(base + 0x00000682, 0x69);
serdes_write_byte(base + 0x00000684, 0x85);
serdes_write_byte(base + 0x00000686, 0x0f);
serdes_write_byte(base + 0x00000687, 0x1d);
serdes_write_byte(base + 0x0000068d, 0x3b);
serdes_write_byte(base + 0x00000a01, 0x08);
serdes_write_byte(base + 0x00000a0a, 0x02);
serdes_write_byte(base + 0x00000a0b, 0x37);
serdes_write_byte(base + 0x00000a31, 0x70);
serdes_write_byte(base + 0x00000a32, 0x70);
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
