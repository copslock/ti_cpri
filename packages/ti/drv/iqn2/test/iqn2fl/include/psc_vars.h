#ifndef PSC_VARS_H
#define PSC_VARS_H

const unsigned int alwayson_pdctl       = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int alwayson_mdctl       = PSC_MDCTL_BASE + ( 0 * 4);
/*  Dummy 1 LPSC in Kepler is used for DFE_IQN_SYS in Lamarr --- erwinh
const unsigned int dummy1_pdctl         = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int dummy1_mdctl         = PSC_MDCTL_BASE + ( 1 * 4);
*/
const unsigned int dfe_iqn_sys_pdctl    = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int dfe_iqn_sys_mdctl    = PSC_MDCTL_BASE + ( 1 * 4);
const unsigned int usb_pdctl            = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int usb_mdctl            = PSC_MDCTL_BASE + ( 2 * 4);
const unsigned int aemif_spi_pdctl      = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int aemif_spi_mdctl      = PSC_MDCTL_BASE + ( 3 * 4);
/*  There is no TSIP.  This is now a reserved space in Lamarr --- erwinh
const unsigned int tsip01_pdctl         = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int tsip01_mdctl         = PSC_MDCTL_BASE + ( 4 * 4);
*/
const unsigned int rsvd01_pdctl         = PSC_PDCTL_BASE + ( 0 * 4);
const unsigned int rsvd01_mdctl         = PSC_MDCTL_BASE + ( 4 * 4);
const unsigned int debugss_pdctl        = PSC_PDCTL_BASE + ( 1 * 4);
const unsigned int debugss_mdctl        = PSC_MDCTL_BASE + ( 5 * 4);
const unsigned int tetb_pdctl           = PSC_PDCTL_BASE + ( 1 * 4);
const unsigned int tetb_mdctl           = PSC_MDCTL_BASE + ( 6 * 4);
const unsigned int pass_pdctl           = PSC_PDCTL_BASE + ( 2 * 4);
const unsigned int pass_mdctl           = PSC_MDCTL_BASE + ( 7 * 4);
const unsigned int sgmii_pdctl          = PSC_PDCTL_BASE + ( 2 * 4);
const unsigned int sgmii_mdctl          = PSC_MDCTL_BASE + ( 8 * 4);
const unsigned int crypto_pdctl         = PSC_PDCTL_BASE + ( 2 * 4);
const unsigned int crypto_mdctl         = PSC_MDCTL_BASE + ( 9 * 4);
/*  There is no PCIE in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int pcie_pdctl           = PSC_PDCTL_BASE + ( 3 * 4);
const unsigned int pcie_mdctl           = PSC_MDCTL_BASE + ( 10 * 4);
*/
const unsigned int rsvd02_pdctl	        = PSC_PDCTL_BASE + ( 3 * 4);
const unsigned int rsvd02_mdctl	        = PSC_MDCTL_BASE + ( 10 * 4);
/*  There is no SRIO in Lamarr.  Kepler SRIO LPSC/GPSC is used for DFE_TXRX_JESD (DFE_PD2) in Lamarr --- erwinh
const unsigned int srio_pdctl           = PSC_PDCTL_BASE + ( 4 * 4);
const unsigned int srio_mdctl           = PSC_MDCTL_BASE + ( 11 * 4);
*/
const unsigned int rsvd08_pdctl	        = PSC_PDCTL_BASE + ( 4 * 4);
const unsigned int rsvd08_mdctl	        = PSC_MDCTL_BASE + ( 11 * 4);
const unsigned int dfe_pd2_pdctl        = PSC_PDCTL_BASE + ( 5 * 4);
const unsigned int dfe_pd2_mdctl        = PSC_MDCTL_BASE + ( 12 * 4);
/*  There is no SRIO in Lamarr.  Kepler SRIO LPSC/GPSC is used for DFE_DDUC_CFR_BB (DFE_PD0) in Lamarr --- erwinh
const unsigned int vusr0_pdctl          = PSC_PDCTL_BASE + ( 5 * 4);
const unsigned int vusr0_mdctl          = PSC_MDCTL_BASE + ( 12 * 4);
*/

const unsigned int srss0_pdctl          = PSC_PDCTL_BASE + ( 6 * 4);
const unsigned int srss0_mdctl          = PSC_MDCTL_BASE + ( 13 * 4);
const unsigned int msmcram_pdctl        = PSC_PDCTL_BASE + ( 7 * 4);
const unsigned int msmcram_mdctl        = PSC_MDCTL_BASE + ( 14 * 4);
const unsigned int gem0_pdctl           = PSC_PDCTL_BASE + ( 8 * 4);
const unsigned int gem0_mdctl           = PSC_MDCTL_BASE + ( 15 * 4);
const unsigned int gem1_pdctl           = PSC_PDCTL_BASE + ( 9 * 4);
const unsigned int gem1_mdctl           = PSC_MDCTL_BASE + ( 16 * 4);
const unsigned int gem2_pdctl           = PSC_PDCTL_BASE + ( 10 * 4);
const unsigned int gem2_mdctl           = PSC_MDCTL_BASE + ( 17 * 4);
const unsigned int gem3_pdctl           = PSC_PDCTL_BASE + ( 11 * 4);
const unsigned int gem3_mdctl           = PSC_MDCTL_BASE + ( 18 * 4);
/*  There is no GEM4-7 in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int gem4_pdctl           = PSC_PDCTL_BASE + ( 12 * 4);
const unsigned int gem4_mdctl           = PSC_MDCTL_BASE + ( 19 * 4);
const unsigned int gem5_pdctl           = PSC_PDCTL_BASE + ( 13 * 4);
const unsigned int gem5_mdctl           = PSC_MDCTL_BASE + ( 20 * 4);
const unsigned int gem6_pdctl           = PSC_PDCTL_BASE + ( 14 * 4);
const unsigned int gem6_mdctl           = PSC_MDCTL_BASE + ( 21 * 4);
const unsigned int gem7_pdctl           = PSC_PDCTL_BASE + ( 15 * 4);
const unsigned int gem7_mdctl           = PSC_MDCTL_BASE + ( 22 * 4);
*/
const unsigned int rsvd03_pdctl         = PSC_PDCTL_BASE + ( 12 * 4);
const unsigned int rsvd03_mdctl         = PSC_MDCTL_BASE + ( 19 * 4);
const unsigned int rsvd04_pdctl         = PSC_PDCTL_BASE + ( 13 * 4);
const unsigned int rsvd04_mdctl         = PSC_MDCTL_BASE + ( 20 * 4);
const unsigned int rsvd05_pdctl         = PSC_PDCTL_BASE + ( 14 * 4);
const unsigned int rsvd05_mdctl         = PSC_MDCTL_BASE + ( 21 * 4);
const unsigned int rsvd06_pdctl         = PSC_PDCTL_BASE + ( 15 * 4);
const unsigned int rsvd06_mdctl         = PSC_MDCTL_BASE + ( 22 * 4);

const unsigned int ddr3a_pdctl          = PSC_PDCTL_BASE + ( 16 * 4);
const unsigned int ddr3a_mdctl          = PSC_MDCTL_BASE + ( 23 * 4);
/*  There is no DDR3B in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int ddr3b_pdctl          = PSC_PDCTL_BASE + ( 16 * 4);
const unsigned int ddr3b_mdctl          = PSC_MDCTL_BASE + ( 24 * 4);
*/
const unsigned int rsvd07_pdctl	        = PSC_PDCTL_BASE + ( 16 * 4);
const unsigned int rsvd07_mdctl	        = PSC_MDCTL_BASE + ( 24 * 4);
const unsigned int tac_pdctl            = PSC_PDCTL_BASE + ( 17 * 4);
const unsigned int tac_mdctl            = PSC_MDCTL_BASE + ( 25 * 4);
const unsigned int rac01_pdctl          = PSC_PDCTL_BASE + ( 17 * 4);
const unsigned int rac01_mdctl          = PSC_MDCTL_BASE + ( 26 * 4);
/*  There is no RAC 2/3 in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int rac23_pdctl          = PSC_PDCTL_BASE + ( 18 * 4);
const unsigned int rac23_mdctl          = PSC_MDCTL_BASE + ( 27 * 4);
*/
const unsigned int dfe_pd0_pdctl        = PSC_PDCTL_BASE + ( 18 * 4);
const unsigned int dfe_pd0_mdctl        = PSC_MDCTL_BASE + ( 27 * 4);
const unsigned int fftc0_pdctl          = PSC_PDCTL_BASE + ( 19 * 4);
const unsigned int fftc0_mdctl          = PSC_MDCTL_BASE + ( 28 * 4);
/*  FFTC1 is relocated in Lamarr.  There is no FFTC2-5 in Lamarr.  These are now reserved spaces in Lamarr --- erwinh
const unsigned int fftc1_pdctl          = PSC_PDCTL_BASE + ( 19 * 4);
const unsigned int fftc1_mdctl          = PSC_MDCTL_BASE + ( 29 * 4);
const unsigned int fftc2_pdctl          = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int fftc2_mdctl          = PSC_MDCTL_BASE + ( 30 * 4);
const unsigned int fftc3_pdctl          = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int fftc3_mdctl          = PSC_MDCTL_BASE + ( 31 * 4);
const unsigned int fftc4_pdctl          = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int fftc4_mdctl          = PSC_MDCTL_BASE + ( 32 * 4);
const unsigned int fftc5_pdctl          = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int fftc5_mdctl          = PSC_MDCTL_BASE + ( 33 * 4);
*/
const unsigned int rsvd09_pdctl	   = PSC_PDCTL_BASE + ( 19 * 4);
const unsigned int rsvd09_mdctl	   = PSC_MDCTL_BASE + ( 29 * 4);
const unsigned int rsvd10_pdctl	   = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int rsvd10_mdctl	   = PSC_MDCTL_BASE + ( 30 * 4);
const unsigned int rsvd11_pdctl	   = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int rsvd11_mdctl	   = PSC_MDCTL_BASE + ( 31 * 4);
const unsigned int rsvd12_pdctl	   = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int rsvd12_mdctl	   = PSC_MDCTL_BASE + ( 32 * 4);
const unsigned int rsvd13_pdctl	   = PSC_PDCTL_BASE + ( 20 * 4);
const unsigned int rsvd13_mdctl	   = PSC_MDCTL_BASE + ( 33 * 4);
/*  There is no AIF in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int aif_pdctl            = PSC_PDCTL_BASE + ( 21 * 4);
const unsigned int aif_mdctl            = PSC_MDCTL_BASE + ( 34 * 4);
*/
const unsigned int rsvd14_pdctl            = PSC_PDCTL_BASE + ( 21 * 4);
const unsigned int rsvd14_mdctl            = PSC_MDCTL_BASE + ( 34 * 4);

const unsigned int tcp3d0_pdctl         = PSC_PDCTL_BASE + ( 22 * 4);
const unsigned int tcp3d0_mdctl         = PSC_MDCTL_BASE + ( 35 * 4);
/*  TCP3D1 is relocated in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int tcp3d1_pdctl         = PSC_PDCTL_BASE + ( 22 * 4);
const unsigned int tcp3d1_mdctl         = PSC_MDCTL_BASE + ( 36 * 4);
*/
const unsigned int rsvd15_pdctl         = PSC_PDCTL_BASE + ( 22 * 4);
const unsigned int rsvd15_mdctl         = PSC_MDCTL_BASE + ( 36 * 4);
/*  There is no TCP3D2 in Lamarr.  TCP3D1 is relocated to TCP3D2 location in Lamarr --- erwinh
const unsigned int tcp3d2_pdctl         = PSC_PDCTL_BASE + ( 23 * 4);
const unsigned int tcp3d2_mdctl         = PSC_MDCTL_BASE + ( 37 * 4);
*/
const unsigned int tcp3d1_pdctl         = PSC_PDCTL_BASE + ( 23 * 4);
const unsigned int tcp3d1_mdctl         = PSC_MDCTL_BASE + ( 37 * 4);
/*  There is no TCP3D3 in Lamarr.  This is now a reserved space in Lamarr --- erwinh
const unsigned int tcp3d3_pdctl         = PSC_PDCTL_BASE + ( 23 * 4);
const unsigned int tcp3d3_mdctl         = PSC_MDCTL_BASE + ( 38 * 4);
*/
const unsigned int rsvd16_pdctl         = PSC_PDCTL_BASE + ( 23 * 4);
const unsigned int rsvd16_mdctl         = PSC_MDCTL_BASE + ( 38 * 4);
const unsigned int vcp0_pdctl           = PSC_PDCTL_BASE + ( 24 * 4);
const unsigned int vcp0_mdctl           = PSC_MDCTL_BASE + ( 39 * 4);
const unsigned int vcp1_pdctl           = PSC_PDCTL_BASE + ( 24 * 4);
const unsigned int vcp1_mdctl           = PSC_MDCTL_BASE + ( 40 * 4);
const unsigned int vcp2_pdctl           = PSC_PDCTL_BASE + ( 24 * 4);
const unsigned int vcp2_mdctl           = PSC_MDCTL_BASE + ( 41 * 4);
const unsigned int vcp3_pdctl           = PSC_PDCTL_BASE + ( 24 * 4);
const unsigned int vcp3_mdctl           = PSC_MDCTL_BASE + ( 42 * 4);
/*  There is no VCP4-7 in Lamarr.  These are now reserved spaces in Lamarr --- erwinh
const unsigned int vcp4_pdctl           = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int vcp4_mdctl           = PSC_MDCTL_BASE + ( 43 * 4);
const unsigned int vcp5_pdctl           = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int vcp5_mdctl           = PSC_MDCTL_BASE + ( 44 * 4);
const unsigned int vcp6_pdctl           = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int vcp6_mdctl           = PSC_MDCTL_BASE + ( 45 * 4);
const unsigned int vcp7_pdctl           = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int vcp7_mdctl           = PSC_MDCTL_BASE + ( 46 * 4);
*/
const unsigned int rsvd17_pdctl	    = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int rsvd17_mdctl	    = PSC_MDCTL_BASE + ( 43 * 4);
const unsigned int rsvd18_pdctl	    = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int rsvd18_mdctl	    = PSC_MDCTL_BASE + ( 44 * 4);
const unsigned int rsvd19_pdctl	    = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int rsvd19_mdctl	    = PSC_MDCTL_BASE + ( 45 * 4);
const unsigned int rsvd20_pdctl	    = PSC_PDCTL_BASE + ( 25 * 4);
const unsigned int rsvd20_mdctl	    = PSC_MDCTL_BASE + ( 46 * 4);
const unsigned int bcp_pdctl            = PSC_PDCTL_BASE + ( 26 * 4);
const unsigned int bcp_mdctl            = PSC_MDCTL_BASE + ( 47 * 4);
/*There are no DXB, VUSR, and XGE in Lamarr.  These spaces are reassigned --- erwinh
const unsigned int dxb_pdctl            = PSC_PDCTL_BASE + ( 27 * 4);
const unsigned int dxb_mdctl            = PSC_MDCTL_BASE + ( 48 * 4);
const unsigned int vusr1_pdctl          = PSC_PDCTL_BASE + ( 28 * 4);
const unsigned int vusr1_mdctl          = PSC_MDCTL_BASE + ( 49 * 4);
const unsigned int xge_pdctl            = PSC_PDCTL_BASE + ( 29 * 4);
const unsigned int xge_mdctl            = PSC_MDCTL_BASE + ( 50 * 4);
*/
const unsigned int dfe_pd1_pdctl            = PSC_PDCTL_BASE + ( 27 * 4);
const unsigned int dfe_pd1_mdctl            = PSC_MDCTL_BASE + ( 48 * 4);
const unsigned int fftc1_pdctl          = PSC_PDCTL_BASE + ( 28 * 4);
const unsigned int fftc1_mdctl          = PSC_MDCTL_BASE + ( 49 * 4);
const unsigned int iqn_ail_pdctl            = PSC_PDCTL_BASE + ( 29 * 4);
const unsigned int iqn_ail_mdctl            = PSC_MDCTL_BASE + ( 50 * 4);
/*There is no srss1 in Lamarr.  This space is a reserved space in Lamarr --- erwinh
const unsigned int srss1_pdctl          = PSC_PDCTL_BASE + ( 30 * 4);
const unsigned int srss1_mdctl          = PSC_MDCTL_BASE + ( 51 * 4);
*/
const unsigned int rsvd21_pdctl	    = PSC_PDCTL_BASE + ( 30 * 4);
const unsigned int rsvd21_mdctl	    = PSC_MDCTL_BASE + ( 51 * 4);
const unsigned int tetris_pdctl         = PSC_PDCTL_BASE + ( 31 * 4);
const unsigned int tetris_mdctl         = PSC_MDCTL_BASE + ( 52 * 4);

#define MAX_PD 32
//const unsigned int max_pd = 32;
const unsigned int max_md = 53;
const unsigned int md2pd[] = {
                    0, 0, 0, 0, 0, 1, 1, 2, 2, 2,
                    3, 4, 5, 6, 7, 8, 9,10,11,12,
                   13,14,15,16,16,17,17,18,19,19,
                   20,20,20,20,21,22,22,23,23,24,
                   24,24,24,25,25,25,25,26,27,28,
                   29,30,31
};

#endif
