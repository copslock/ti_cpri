/* --------------------------------------------------------------------------
    FILE        : aisxtra.c 				                             	 	        
    PURPOSE     : AIS Extra commands for use in AIS Scripts
    PROJECT     : TI Boot and Flash Utilities
    AUTHOR      : Daniel Allred
 ----------------------------------------------------------------------------- */

/************************************************************
* Include Files                                             *
************************************************************/
 
#include "tistdtypes.h"
#include "device.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern __FAR__ Uint32 EXTERNAL_RAM_START;


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/
#if defined(__TMS320C6X__)
  
	#pragma DATA_SECTION(paramTable,".params")
	Uint32 paramTable[16];

#elif defined(__GNUC__)
	Uint32 paramTable[16] __attribute__((,section(".params")));
#endif

/************************************************************
* Global Function Definitions                               *
************************************************************/

#if defined(__TMS320C6X__)
  #pragma CODE_SECTION(PatchDDRConfig,".text")
  void PatchDDRConfig()  {
#elif defined(__GNUC__)
  void PatchDDRConfig() __attribute__((,section(".text")));
  void PatchDDRConfig() {
#endif
 
  VUint32 data_tmp;
  VUint32 *DDRStart = (VUint32 *) ((Uint32 *) EXTERNAL_RAM_START);
  Uint32 ddrphycr = paramTable[0];
  Uint32 sdcr = paramTable[1];
  Uint32 sdtimr = paramTable[2];
  Uint32 sdtimr2 = paramTable[3];
  Uint32 sdrcr = paramTable[4];
  Uint32 sdcr2 = paramTable[5];
  
  // Set the DDR2 to enable
  {  
    // Wait for any outstanding transition to complete
    while ( (PSC1->PTSTAT) & (0x00000001 << PD0) );
    
    // If we are already in that state, just return
    if (((PSC1->MDSTAT[LPSC_EMIFB]) & 0x1F) == PSC_ENABLE) return;
      
    // Perform transition
    PSC1->MDCTL[LPSC_EMIFB] = ((PSC1->MDCTL[LPSC_EMIFB]) & (0xFFFFFFE0)) | (PSC_ENABLE);
    PSC1->PTCMD |= (0x00000001 << PD0);

    // Wait for transition to complete
    while ( (PSC1->PTSTAT) & (0x00000001 << PD0) );
    
    // Wait and verify the state
    while (((PSC1->MDSTAT[LPSC_EMIFB]) & 0x1F) != PSC_ENABLE);
  }

  // Begin VTP Calibration
  // If VTP claiberation enabled, then skip the VTP calibration
  if (SUBCHIPCFG->VTPIO_CTL & 0x00000040)
  {
    SUBCHIPCFG->VTPIO_CTL |= DEVICE_VTPIO_CTL_IOPWRDN_MASK;     // Set IOPWRDN bit to enable input buffer powerdown enable mode
    SUBCHIPCFG->VTPIO_CTL &= ~(DEVICE_VTPIO_CTL_POWERDN_MASK);  // Clear POWERDN bit (enable VTP)
    
    SUBCHIPCFG->VTPIO_CTL &= ~(DEVICE_VTPIO_CTL_LOCK_MASK);     // Clear LOCK bit

    // Pulse (low) CLRZ to initiate VTP IO Calibration
    SUBCHIPCFG->VTPIO_CTL |= DEVICE_VTPIO_CTL_CLKRZ_MASK;       // Set CLRZ bit 
    SUBCHIPCFG->VTPIO_CTL &= ~(DEVICE_VTPIO_CTL_CLKRZ_MASK);    // Clear CLRZ bit (CLRZ should be low for at least 2ns)
    SUBCHIPCFG->VTPIO_CTL |= DEVICE_VTPIO_CTL_CLKRZ_MASK;       // Set CLRZ bit 

    // Polling READY bit to see when VTP calibration is done
    while(!((SUBCHIPCFG->VTPIO_CTL & DEVICE_VTPIO_CTL_READY_MASK)>>DEVICE_VTPIO_CTL_READY_SHIFT));

    SUBCHIPCFG->VTPIO_CTL |= DEVICE_VTPIO_CTL_LOCK_MASK;        // Set LOCK bit for static mode
    SUBCHIPCFG->VTPIO_CTL |= DEVICE_VTPIO_CTL_PWRSAVE_MASK;     // Set PWRSAVE bit to save power
  }
  // End VTP Calibration
  
  // Config DDR timings
  EMIF3A->DDRPHYC1R   = ddrphycr;

  // Clear the unlock bits (in case user accidentally set them)
  sdcr = sdcr & (~DEVICE_SDCR_BOOTUNLOCK_MASK) & (~DEVICE_SDCR_TIMUNLOCK_MASK);
  
  // Set SDCR with BOOTUNLOCK Set and TIMUNLOCK cleared
  EMIF3A->SDCR        = sdcr | ((0x1 << DEVICE_SDCR_BOOTUNLOCK_SHIFT) & DEVICE_SDCR_BOOTUNLOCK_MASK);
  
  // Set SDCR with BOOTUNLOCK cleared and TIMUNLOCK set
  EMIF3A->SDCR        = sdcr | ((0x1 << DEVICE_SDCR_TIMUNLOCK_SHIFT) & DEVICE_SDCR_TIMUNLOCK_MASK);
  
  // Check if this init is for mDDR
  if (sdcr & DEVICE_SDCR_MSDRAMEN_MASK)
  {
    // If it is, set SDCR2 with PASR and ROWSIZE PARAMS
    EMIF3A->SDCR2     = sdcr2;
  }
                        
  EMIF3A->SDTIMR      = sdtimr;
  EMIF3A->SDTIMR2     = sdtimr2;

  // Clear TIMUNLOCK bit
  EMIF3A->SDCR        &= (~DEVICE_SDCR_TIMUNLOCK_MASK); 
  EMIF3A->SDRCR       = sdrcr;

  // Set the DDR2 to syncreset
  {
    EMIF3A->SDRCR |= 0xC0000000;  // Set to self-refresh, enable mclkstop
    // Wait for any outstanding transition to complete
    while ( (PSC1->PTSTAT) & (0x00000001 << PD0) );
    
    // If we are already in that state, just return
    if (((PSC1->MDSTAT[LPSC_EMIFB]) & 0x1F) == PSC_SYNCRESET) return;
      
    // Perform transition
    PSC1->MDCTL[LPSC_EMIFB] = ((PSC1->MDCTL[LPSC_EMIFB]) & (0xFFFFFFE0)) | (PSC_SYNCRESET);
    PSC1->PTCMD |= (0x00000001 << PD0);

    // Wait for transition to complete
    while ( (PSC1->PTSTAT) & (0x00000001 << PD0) );
    
    // Wait and verify the state
    while (((PSC1->MDSTAT[LPSC_EMIFB]) & 0x1F) != PSC_SYNCRESET);
  }

  // Set the DDR2 to enable
  {
    // Wait for any outstanding transition to complete
    while ( (PSC1->PTSTAT) & (0x00000001 << PD0) );
    
    // If we are already in that state, just return
    if (((PSC1->MDSTAT[LPSC_EMIFB]) & 0x1F) == PSC_ENABLE) return;
      
    // Perform transition
    PSC1->MDCTL[LPSC_EMIFB] = ((PSC1->MDCTL[LPSC_EMIFB]) & (0xFFFFFFE0)) | (PSC_ENABLE);
    PSC1->PTCMD |= (0x00000001 << PD0);

    // Wait for transition to complete
    while ( (PSC1->PTSTAT) & (0x00000001 << PD0) );
    
    // Wait and verify the state
    while (((PSC1->MDSTAT[LPSC_EMIFB]) & 0x1F) != PSC_ENABLE);
    EMIF3A->SDRCR &= ~(0xC0000000);  // disable self-refresh
  }
  
  // Dummy write/read to apply timing settings
  *DDRStart = 0xA55AA55A;               // write
  data_tmp = *DDRStart;                 // read
  *DDRStart = data_tmp;
}

