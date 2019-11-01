#include <ti/csl/cslr_device.h>

#ifndef PASS
#define PASS		(1)
#endif

#ifndef FAIL
#define FAIL		(0)
#endif

#define SUCCESS		PASS
#define FAILURE		FAIL


#define PSC_PRE_RESV_START	 0x02340000 // 0x02a48400  //  John Herrington - Ported from Curie to Nyquist
#define PSC_PRE_RESV_END	 0x0234FFFC // 0x02abfffc
#define PSC_START_ADDRESS	 CSL_PSC_REGS //0x02350000 // 0x02ac0000
#define PSC_POST_RESV_START	 0x02351000 // 0x02ac1000
#define PSC_POST_RESV_END	 0x0235FFFC // 0x02ac3ffc


#define PSC_MDCTL00           0xA00
#define PSC_MDSTAT00          0x800
#define PSC_PDCTL00           0x300
#define PSC_PDSTAT00          0x200

#define PSC_MDCTL_BASE        PSC_MDCTL00
#define PSC_MDSTAT_BASE       PSC_MDSTAT00
#define PSC_PDCTL_BASE        PSC_PDCTL00
#define PSC_PDSTAT_BASE       PSC_PDSTAT00

extern const unsigned int alwayson_pdctl;
extern const unsigned int alwayson_mdctl;
extern const unsigned int dfe_iqn_sys_pdctl;
extern const unsigned int dfe_iqn_sys_mdctl;
extern const unsigned int usb_pdctl;
extern const unsigned int usb_mdctl;
extern const unsigned int aemif_spi_pdctl;
extern const unsigned int aemif_spi_mdctl;
extern const unsigned int rsvd01_pdctl;
extern const unsigned int rsvd01_mdctl;
extern const unsigned int debugss_pdctl;
extern const unsigned int debugss_mdctl;
extern const unsigned int tetb_pdctl;
extern const unsigned int tetb_mdctl;
extern const unsigned int pass_pdctl;
extern const unsigned int pass_mdctl;
extern const unsigned int sgmii_pdctl;
extern const unsigned int sgmii_mdctl;
extern const unsigned int crypto_pdctl;
extern const unsigned int crypto_mdctl;
extern const unsigned int pcie0_pdctl;
extern const unsigned int pcie0_mdctl;
extern const unsigned int pcie1_pdctl;
extern const unsigned int pcie1_mdctl;
extern const unsigned int dfe_pd2_pdctl;
extern const unsigned int dfe_pd2_mdctl;
extern const unsigned int dfe_pd0_pdctl;
extern const unsigned int dfe_pd0_mdctl;
extern const unsigned int srss0_pdctl;
extern const unsigned int srss0_mdctl;
extern const unsigned int msmcram_pdctl;
extern const unsigned int msmcram_mdctl;
extern const unsigned int gem0_pdctl;
extern const unsigned int gem0_mdctl;
extern const unsigned int gem1_pdctl;
extern const unsigned int gem1_mdctl;
extern const unsigned int gem2_pdctl;
extern const unsigned int gem2_mdctl;
extern const unsigned int gem3_pdctl;
extern const unsigned int gem3_mdctl;
extern const unsigned int rsvd03_pdctl;
extern const unsigned int rsvd03_mdctl;
extern const unsigned int rsvd04_pdctl;
extern const unsigned int rsvd04_mdctl;
extern const unsigned int rsvd05_pdctl;
extern const unsigned int rsvd05_mdctl;
extern const unsigned int rsvd06_pdctl;
extern const unsigned int rsvd06_mdctl;
extern const unsigned int ddr3a_pdctl;
extern const unsigned int ddr3a_mdctl;
extern const unsigned int rsvd07_pdctl;
extern const unsigned int rsvd07_mdctl;
extern const unsigned int tac_pdctl;
extern const unsigned int tac_mdctl;
extern const unsigned int rac01_pdctl;
extern const unsigned int rac01_mdctl;
extern const unsigned int rsvd08_pdctl;
extern const unsigned int rsvd08_mdctl;
extern const unsigned int fftc0_pdctl;
extern const unsigned int fftc0_mdctl;
extern const unsigned int rsvd09_pdctl;
extern const unsigned int rsvd09_mdctl;
extern const unsigned int rsvd10_pdctl;
extern const unsigned int rsvd10_mdctl;
extern const unsigned int rsvd11_pdctl;
extern const unsigned int rsvd11_mdctl;
extern const unsigned int rsvd12_pdctl;
extern const unsigned int rsvd12_mdctl;
extern const unsigned int rsvd13_pdctl;
extern const unsigned int rsvd13_mdctl;
extern const unsigned int osr_pdctl;
extern const unsigned int osr_mdctl;
extern const unsigned int tcp3d0_pdctl;
extern const unsigned int tcp3d0_mdctl;
extern const unsigned int rsvd15_pdctl;
extern const unsigned int rsvd15_mdctl;
extern const unsigned int tcp3d1_pdctl;
extern const unsigned int tcp3d1_mdctl;
extern const unsigned int rsvd16_pdctl;
extern const unsigned int rsvd16_mdctl;
extern const unsigned int vcp0_pdctl;
extern const unsigned int vcp0_mdctl;
extern const unsigned int vcp1_pdctl;
extern const unsigned int vcp1_mdctl;
extern const unsigned int vcp2_pdctl;
extern const unsigned int vcp2_mdctl;
extern const unsigned int vcp3_pdctl;
extern const unsigned int vcp3_mdctl;
extern const unsigned int rsvd17_pdctl;
extern const unsigned int rsvd17_mdctl;
extern const unsigned int rsvd18_pdctl;
extern const unsigned int rsvd18_mdctl;
extern const unsigned int rsvd19_pdctl;
extern const unsigned int rsvd19_mdctl;
extern const unsigned int rsvd20_pdctl;
extern const unsigned int rsvd20_mdctl;
extern const unsigned int bcp_pdctl;
extern const unsigned int bcp_mdctl;
extern const unsigned int dfe_pd1_pdctl;
extern const unsigned int dfe_pd1_mdctl;
extern const unsigned int fftc1_pdctl;
extern const unsigned int fftc1_mdctl;
extern const unsigned int iqn_ail_pdctl;
extern const unsigned int iqn_ail_mdctl;
extern const unsigned int rsvd21_pdctl;
extern const unsigned int rsvd21_mdctl;
extern const unsigned int tetris_pdctl;
extern const unsigned int tetris_mdctl;


extern const unsigned int max_pd;
extern const unsigned int max_md;
extern const unsigned int md2pd[];

 

/*
//Following definitions to be updated with PSC register offsets

#define PSC_PERIPHERAL_ID  PSC_START_ADDRESS + 0x00000000
#define PSC_RESV_000       PSC_START_ADDRESS + 0x00000004
#define PSC_RESV_001       PSC_START_ADDRESS + 0x0000000F
	
#define PSC_GBLCTL         PSC_START_ADDRESS + 0x00000010
#define PSC_GBLSTAT        PSC_START_ADDRESS + 0x00000014
#define PSC_INTEVAL        PSC_START_ADDRESS + 0x00000018
#define PSC_IPWKCNT        PSC_START_ADDRESS + 0x0000001c
#define PSC_RESV_010       PSC_START_ADDRESS + 0x00000020
#define PSC_RESV_011       PSC_START_ADDRESS + 0x0000003F
	
#define PSC_MERRPR0	   PSC_START_ADDRESS + 0x00000040
#define PSC_MERRPR1	   PSC_START_ADDRESS + 0x00000044
#define PSC_UNUSED_REG0    PSC_START_ADDRESS + 0x048
#define PSC_UNUSED_REG1    PSC_START_ADDRESS + 0x04C
#define PSC_MERRCR0        PSC_START_ADDRESS + 0x050
#define PSC_MERRCR1        PSC_START_ADDRESS + 0x054
#define PSC_UNUSED_REG2    PSC_START_ADDRESS + 0x058
#define PSC_UNUSED_REG3    PSC_START_ADDRESS + 0x05C
#define PSC_PERRPR         PSC_START_ADDRESS + 0x060
#define PSC_RESV_020       PSC_START_ADDRESS + 0x00000064
#define PSC_RESV_021       PSC_START_ADDRESS + 0x00000067


#define PSC_PERRCR	   PSC_START_ADDRESS + 0x00000068
#define PSC_RESV_030   PSC_START_ADDRESS + 0x0000006C
#define PSC_RESV_031   PSC_START_ADDRESS + 0x0000006F

#define PSC_EPCPR	   PSC_START_ADDRESS + 0x00000070
#define PSC_RESV_040   PSC_START_ADDRESS + 0x00000074
#define PSC_RESV_041   PSC_START_ADDRESS + 0x00000077

#define PSC_EPCCR	   PSC_START_ADDRESS + 0x00000078
#define PSC_RESV_050   PSC_START_ADDRESS + 0x0000007C
#define PSC_RESV_051   PSC_START_ADDRESS + 0x000000FC
				



#define PSC_RAILSTAT        PSC_START_ADDRESS + 0x00000100
#define PSC_RAILCTL         PSC_START_ADDRESS + 0x00000104
#define PSC_RAILSEL         PSC_START_ADDRESS + 0x00000108
#define PSC_RESV_060        PSC_START_ADDRESS + 0x0000010c
#define PSC_RESV_061        PSC_START_ADDRESS + 0x0000011F
#define PSC_PTCMD           PSC_START_ADDRESS + 0x00000120
#define PSC_RESV_070        PSC_START_ADDRESS + 0x00000124
#define PSC_RESV_071        PSC_START_ADDRESS + 0x00000124
#define PSC_PTSTAT          PSC_START_ADDRESS + 0x00000128
#define PSC_RESV_080        PSC_START_ADDRESS + 0x0000012c
#define PSC_RESV_081        PSC_START_ADDRESS + 0x000001fc
                   
#define PSC_PDSTAT0         PSC_START_ADDRESS + 0x00000200
#define PSC_PDSTAT1         PSC_START_ADDRESS + 0x00000204
#define PSC_PDSTAT2         PSC_START_ADDRESS + 0x00000208
#define PSC_PDSTAT3         PSC_START_ADDRESS + 0x0000020c
#define PSC_PDSTAT4         PSC_START_ADDRESS + 0x00000210
#define PSC_PDSTAT5         PSC_START_ADDRESS + 0x00000214
#define PSC_PDSTAT6         PSC_START_ADDRESS + 0x00000218
#define PSC_PDSTAT7         PSC_START_ADDRESS + 0x0000021C
#define PSC_PDSTAT8         PSC_START_ADDRESS + 0x00000220
#define PSC_PDSTAT9         PSC_START_ADDRESS + 0x00000224
#define PSC_PDSTAT10         PSC_START_ADDRESS + 0x00000228
#define PSC_PDSTAT11        PSC_START_ADDRESS + 0x0000022C
#define PSC_PDSTAT12         PSC_START_ADDRESS + 0x00000230
#define PSC_PDSTAT13         PSC_START_ADDRESS + 0x00000234
#define PSC_PDSTAT14         PSC_START_ADDRESS + 0x00000238
#define PSC_PDSTAT15         PSC_START_ADDRESS + 0x0000023C
#define PSC_PDSTAT16         PSC_START_ADDRESS + 0x00000240
#define PSC_PDSTAT17         PSC_START_ADDRESS + 0x00000244
#define PSC_PDSTAT18         PSC_START_ADDRESS + 0x00000248
#define PSC_RESV_090        PSC_START_ADDRESS + 0x0000024C
#define PSC_RESV_091        PSC_START_ADDRESS + 0x000002FF
	               
#define PSC_PDCTL0         PSC_START_ADDRESS + 0x00000300
#define PSC_PDCTL1         PSC_START_ADDRESS + 0x00000304
#define PSC_PDCTL2         PSC_START_ADDRESS + 0x00000308
#define PSC_PDCTL3         PSC_START_ADDRESS + 0x0000030c
#define PSC_PDCTL4         PSC_START_ADDRESS + 0x00000310
#define PSC_PDCTL5         PSC_START_ADDRESS + 0x00000314
#define PSC_PDCTL6         PSC_START_ADDRESS + 0x00000318
#define PSC_PDCTL7         PSC_START_ADDRESS + 0x0000031C
#define PSC_PDCTL8         PSC_START_ADDRESS + 0x00000320
#define PSC_PDCTL9         PSC_START_ADDRESS + 0x00000324
#define PSC_PDCTL10         PSC_START_ADDRESS + 0x00000328
#define PSC_PDCTL11        PSC_START_ADDRESS + 0x0000032C
#define PSC_PDCTL12         PSC_START_ADDRESS + 0x00000330
#define PSC_PDCTL13         PSC_START_ADDRESS + 0x00000334
#define PSC_PDCTL14         PSC_START_ADDRESS + 0x00000338
#define PSC_PDCTL15         PSC_START_ADDRESS + 0x0000033C
#define PSC_PDCTL16         PSC_START_ADDRESS + 0x00000340
#define PSC_PDCTL17         PSC_START_ADDRESS + 0x00000344
#define PSC_PDCTL18         PSC_START_ADDRESS + 0x00000348
#define PSC_RESV_100        PSC_START_ADDRESS + 0x0000034C
#define PSC_RESV_111        PSC_START_ADDRESS + 0x000003FF
	               
#define PSC_PDCFG0         PSC_START_ADDRESS + 0x00000400
#define PSC_PDCFG1         PSC_START_ADDRESS + 0x00000404
#define PSC_PDCFG2         PSC_START_ADDRESS + 0x00000408
#define PSC_PDCFG3         PSC_START_ADDRESS + 0x0000040c
#define PSC_PDCFG4         PSC_START_ADDRESS + 0x00000410
#define PSC_PDCFG5         PSC_START_ADDRESS + 0x00000414
#define PSC_PDCFG6         PSC_START_ADDRESS + 0x00000418
#define PSC_PDCFG7         PSC_START_ADDRESS + 0x0000041C
#define PSC_PDCFG8         PSC_START_ADDRESS + 0x00000420
#define PSC_PDCFG9         PSC_START_ADDRESS + 0x00000424
#define PSC_PDCFG10         PSC_START_ADDRESS + 0x00000428
#define PSC_PDCFG11        PSC_START_ADDRESS + 0x0000042C
#define PSC_PDCFG12         PSC_START_ADDRESS + 0x00000430
#define PSC_PDCFG13         PSC_START_ADDRESS + 0x00000434
#define PSC_PDCFG14         PSC_START_ADDRESS + 0x00000438
#define PSC_PDCFG15         PSC_START_ADDRESS + 0x0000043C
#define PSC_PDCFG16         PSC_START_ADDRESS + 0x00000440
#define PSC_PDCFG17         PSC_START_ADDRESS + 0x00000444
#define PSC_PDCFG18         PSC_START_ADDRESS + 0x00000448
#define PSC_RESV_120        PSC_START_ADDRESS + 0x0000044C
#define PSC_RESV_121        PSC_START_ADDRESS + 0x000004FF


#define PSC_MDCFG7         PSC_START_ADDRESS + 0x0000061C
#define PSC_MDCFG8         PSC_START_ADDRESS + 0x00000620
#define PSC_MDCFG9         PSC_START_ADDRESS + 0x00000624
#define PSC_MDCFG10         PSC_START_ADDRESS + 0x00000628
#define PSC_MDCFG11        PSC_START_ADDRESS + 0x0000062C
#define PSC_MDCFG12         PSC_START_ADDRESS + 0x00000630
#define PSC_MDCFG13         PSC_START_ADDRESS + 0x00000634
#define PSC_MDCFG14         PSC_START_ADDRESS + 0x00000638
#define PSC_MDCFG15         PSC_START_ADDRESS + 0x0000063C
#define PSC_MDCFG16         PSC_START_ADDRESS + 0x00000640
#define PSC_MDCFG17         PSC_START_ADDRESS + 0x00000644
#define PSC_MDCFG18         PSC_START_ADDRESS + 0x00000648
#define PSC_MDCFG19         PSC_START_ADDRESS + 0x0000064C
#define PSC_MDCFG20         PSC_START_ADDRESS + 0x00000650
#define PSC_MDCFG21         PSC_START_ADDRESS + 0x00000654
#define PSC_MDCFG22         PSC_START_ADDRESS + 0x00000658
#define PSC_MDCFG23         PSC_START_ADDRESS + 0x0000065c
#define PSC_MDCFG24         PSC_START_ADDRESS + 0x00000660
#define PSC_MDCFG25         PSC_START_ADDRESS + 0x00000664
#define PSC_MDCFG26         PSC_START_ADDRESS + 0x00000668
#define PSC_MDCFG27         PSC_START_ADDRESS + 0x0000066C
#define PSC_MDCFG28         PSC_START_ADDRESS + 0x00000670
#define PSC_MDCFG29         PSC_START_ADDRESS + 0x00000674
#define PSC_MDCFG30         PSC_START_ADDRESS + 0x00000678
#define PSC_RESV_130        PSC_START_ADDRESS + 0x0000067C
#define PSC_RESV_131        PSC_START_ADDRESS + 0x000007FF
	               
#define PSC_MDSTAT0         PSC_START_ADDRESS + 0x00000800
#define PSC_MDSTAT1         PSC_START_ADDRESS + 0x00000804
#define PSC_MDSTAT2         PSC_START_ADDRESS + 0x00000808
#define PSC_MDSTAT3         PSC_START_ADDRESS + 0x0000080c
#define PSC_MDSTAT4         PSC_START_ADDRESS + 0x00000810
#define PSC_MDSTAT5         PSC_START_ADDRESS + 0x00000814
#define PSC_MDSTAT6         PSC_START_ADDRESS + 0x00000818
#define PSC_MDSTAT7         PSC_START_ADDRESS + 0x0000081C
#define PSC_MDSTAT8         PSC_START_ADDRESS + 0x00000820
#define PSC_MDSTAT9         PSC_START_ADDRESS + 0x00000824
#define PSC_MDSTAT10         PSC_START_ADDRESS + 0x00000828
#define PSC_MDSTAT11        PSC_START_ADDRESS + 0x0000082C
#define PSC_MDSTAT12         PSC_START_ADDRESS + 0x00000830
#define PSC_MDSTAT13         PSC_START_ADDRESS + 0x00000834
#define PSC_MDSTAT14         PSC_START_ADDRESS + 0x00000838
#define PSC_MDSTAT15         PSC_START_ADDRESS + 0x0000083C
#define PSC_MDSTAT16         PSC_START_ADDRESS + 0x00000840
#define PSC_MDSTAT17         PSC_START_ADDRESS + 0x00000844
#define PSC_MDSTAT18         PSC_START_ADDRESS + 0x00000848
#define PSC_MDSTAT19         PSC_START_ADDRESS + 0x0000084C
#define PSC_MDSTAT20         PSC_START_ADDRESS + 0x00000850
#define PSC_MDSTAT21         PSC_START_ADDRESS + 0x00000854
#define PSC_MDSTAT22         PSC_START_ADDRESS + 0x00000858
#define PSC_MDSTAT23         PSC_START_ADDRESS + 0x0000085c
#define PSC_MDSTAT24         PSC_START_ADDRESS + 0x00000860
#define PSC_MDSTAT25         PSC_START_ADDRESS + 0x00000864
#define PSC_MDSTAT26         PSC_START_ADDRESS + 0x00000868
#define PSC_MDSTAT27         PSC_START_ADDRESS + 0x0000086C
#define PSC_MDSTAT28         PSC_START_ADDRESS + 0x00000870
#define PSC_MDSTAT29         PSC_START_ADDRESS + 0x00000874
#define PSC_MDSTAT30         PSC_START_ADDRESS + 0x00000878
#define PSC_RESV_140        PSC_START_ADDRESS + 0x0000087C
#define PSC_RESV_141        PSC_START_ADDRESS + 0x000009FF

	               
#define PSC_MDCTL0         PSC_START_ADDRESS + 0x00000A00
#define PSC_MDCTL1         PSC_START_ADDRESS + 0x00000A04
#define PSC_MDCTL2         PSC_START_ADDRESS + 0x00000A08
#define PSC_MDCTL3         PSC_START_ADDRESS + 0x00000A0c
#define PSC_MDCTL4         PSC_START_ADDRESS + 0x00000A10
#define PSC_MDCTL5         PSC_START_ADDRESS + 0x00000A14
#define PSC_MDCTL6         PSC_START_ADDRESS + 0x00000A18
#define PSC_MDCTL7         PSC_START_ADDRESS + 0x00000A1C
#define PSC_MDCTL8          PSC_START_ADDRESS + 0x00000A20
#define PSC_MDCTL9          PSC_START_ADDRESS + 0x00000A24
#define PSC_MDCTL10         PSC_START_ADDRESS + 0x00000A28
#define PSC_MDCTL11         PSC_START_ADDRESS + 0x00000A2C
#define PSC_MDCTL12         PSC_START_ADDRESS + 0x00000A30
#define PSC_MDCTL13         PSC_START_ADDRESS + 0x00000A34
#define PSC_MDCTL14         PSC_START_ADDRESS + 0x00000A38
#define PSC_MDCTL15         PSC_START_ADDRESS + 0x00000A3C
#define PSC_MDCTL16         PSC_START_ADDRESS + 0x00000A40
#define PSC_MDCTL17         PSC_START_ADDRESS + 0x00000A44
#define PSC_MDCTL18         PSC_START_ADDRESS + 0x00000A48
#define PSC_MDCTL19         PSC_START_ADDRESS + 0x00000A4C
#define PSC_MDCTL20         PSC_START_ADDRESS + 0x00000A50
#define PSC_MDCTL21         PSC_START_ADDRESS + 0x00000A54
#define PSC_MDCTL22         PSC_START_ADDRESS + 0x00000A58
#define PSC_MDCTL23         PSC_START_ADDRESS + 0x00000A5c
#define PSC_MDCTL24         PSC_START_ADDRESS + 0x00000A60
#define PSC_MDCTL25         PSC_START_ADDRESS + 0x00000A64
#define PSC_MDCTL26         PSC_START_ADDRESS + 0x00000A68
#define PSC_MDCTL27         PSC_START_ADDRESS + 0x00000A6C
#define PSC_MDCTL28         PSC_START_ADDRESS + 0x00000A70
#define PSC_MDCTL29         PSC_START_ADDRESS + 0x00000A74
#define PSC_MDCTL30         PSC_START_ADDRESS + 0x00000A78
#define PSC_RESV_120        PSC_START_ADDRESS + 0x00000A7C
#define PSC_RESV_121        PSC_START_ADDRESS + 0x00000FFF

*/


//  Register reset values
#define PSC_PERIPHERAL_ID_RSTVAL		0x44820200
#define PSC_GBLCTL_RSTVAL			0x00000000
#define PSC_GBLSTAT_RSTVAL			0x0FFF0000
#define PSC_MERRPR0_RSTVAL			0x00000000
#define PSC_MERRPR1_RSTVAL			0x00000000
#define PSC_MERRCR0_RSTVAL			0x00000000
#define PSC_MERRCR1_RSTVAL			0x00000000
#define PSC_PERRPR_RSTVAL			0x00000000
#define PSC_PERRCR_RSTVAL			0x00000000
#define PSC_EPCPR_RSTVAL			0x00000000
#define PSC_EPCCR_RSTVAL			0x00000000
#define PSC_RAILSTAT_RSTVAL			0x00000000
#define PSC_RAILCTL_RSTVAL			0x00000000
#define PSC_RAILSEL_RSTVAL			0x00000000
#define PSC_PTCMD_RSTVAL			0x00000000
#define PSC_PTSTAT_RSTVAL			0x00000000

#define PSC_PDSTAT0_RSTVAL			0x00000301
#define PSC_PDSTAT1_RSTVAL			0x00000200
#define PSC_PDSTAT2_RSTVAL			0x00000200
#define PSC_PDSTAT3_RSTVAL			0x00000200
#define PSC_PDSTAT4_RSTVAL			0x00000200

#define PSC_PDCTL0_RSTVAL			0x00220001
#define PSC_PDCTL1_RSTVAL			0x00220000
#define PSC_PDCTL2_RSTVAL			0x00220000
#define PSC_PDCTL3_RSTVAL			0x00220000
#define PSC_PDCTL4_RSTVAL			0x00220000

#define PSC_PDCFG0_RSTVAL			0x0000000D
#define PSC_PDCFG1_RSTVAL			0x00000002
#define PSC_PDCFG2_RSTVAL			0x00000002
#define PSC_PDCFG3_RSTVAL			0x00000002
#define PSC_PDCFG4_RSTVAL			0x00000002

#define PSC_MDCFG0_RSTVAL			0x000000DB
#define PSC_MDCFG1_RSTVAL			0x000004DB
#define PSC_MDCFG2_RSTVAL			0x000001DB
#define PSC_MDCFG3_RSTVAL			0x000000DB
#define PSC_MDCFG4_RSTVAL			0x000000DB
#define PSC_MDCFG5_RSTVAL			0x000000DB
#define PSC_MDCFG6_RSTVAL			0x000004DB
#define PSC_MDCFG7_RSTVAL			0x000100DB
#define PSC_MDCFG8_RSTVAL			0x000200DB
#define PSC_MDCFG9_RSTVAL			0x000300DB
#define PSC_MDCFG10_RSTVAL			0x000400DB

#define PSC_MDSTAT0_RSTVAL			0x00001f03
#define PSC_MDSTAT1_RSTVAL			0x00001f03
#define PSC_MDSTAT2_RSTVAL			0x00001f03
#define PSC_MDSTAT3_RSTVAL			0x00001f03
#define PSC_MDSTAT4_RSTVAL			0x00001f03
#define PSC_MDSTAT5_RSTVAL			0x00001f03
#define PSC_MDSTAT6_RSTVAL			0x00001f03
#define PSC_MDSTAT7_RSTVAL			0x00001f03
#define PSC_MDSTAT8_RSTVAL			0x00001f03
#define PSC_MDSTAT9_RSTVAL			0x00001f03
#define PSC_MDSTAT10_RSTVAL			0x00001f03

#define PSC_MDCTL0_RSTVAL			0x00000103
#define PSC_MDCTL1_RSTVAL			0x00000103
#define PSC_MDCTL2_RSTVAL			0x00000103
#define PSC_MDCTL3_RSTVAL			0x00000103
#define PSC_MDCTL4_RSTVAL			0x00000103
#define PSC_MDCTL5_RSTVAL			0x00000103
#define PSC_MDCTL6_RSTVAL			0x00000103
#define PSC_MDCTL7_RSTVAL			0x00000103
#define PSC_MDCTL8_RSTVAL			0x00000103
#define PSC_MDCTL9_RSTVAL			0x00000103
#define PSC_MDCTL10_RSTVAL			0x00000103

// Register rw masks. Tells which bits are ok to do write.
#define PSC_REG_RDMASK				0xFFFFFFFF	
#define PSC_REG_WRMASK				0xFFFFFFFF	
/*
	;; Register writability mask. Masked bits are writable but
	;; write has no effect. The retain prewrite values. Unmasked bits
	;; Should show written value.
*/

#define PSC_STAT_VALMASK		 0xFFFFFFFF
#define PSC_CFG_VALMASK			 0xFFFFFFFF
#define PSC_CMD_VALMASK			 0xFFFFFFFF

#define PSC_PERIPHERAL_ID_VALMASK	0xFFFFFFFF
#define PSC_GBLCTL_VALMASK		0xFFFF00FC
	
//PSC_MERRCR0_VALMASK		 0x00000000
//PSC_MERRCR1_VALMASK		 0x00000000
//PSC_PERRCR_VALMASK		 0x00000000
//PSC_EPCCR_VALMASK		 0x00000000

#define PSC_RAILCTL_VALMASK		 0xFFFF0000
#define PSC_RAILSEL_VALMASK		 0xFFFFFFE0 // only 5 power domains others rsvd

#define PSC_PDCTL_VALMASK		 0x4F008CFE
#define PSC_MDCTL_VALMASK		 0x7FFFF0E0
	
