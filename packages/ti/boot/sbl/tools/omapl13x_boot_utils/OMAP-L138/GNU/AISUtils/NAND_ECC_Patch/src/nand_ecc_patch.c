/* --------------------------------------------------------------------------
    FILE        : nand_ecc_patch.c 				                             	 	        
    PROJECT     : Catalog boot and flash utils
    AUTHOR      : Daniel Allred
    DESC        : Patch to correct ECC correction functionality for ROM boot
                  loader
-------------------------------------------------------------------------- */ 


/************************************************************
* Include Files                                             *
************************************************************/

#include <stdint.h>


/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Typedef/Enum Definitions                            *
************************************************************/

// NAND ECC function typedefs
typedef uint32_t (*NAND_ECC_Correct)(void *unused, uint8_t *data, uint8_t *readECC);

// NAND_INFO structure - holds pertinent info for open driver instance
typedef struct _NAND_INFO_
{
  uint32_t    flashBase;          // Base address of CS memory space where NAND is connected
  uint8_t     busWidth;           // NAND bus width
  uint8_t     manfID;             // NAND manufacturer ID (just for informational purposes)
  uint8_t     devID;              // NAND_TABLE_Devices index (device ID)
  uint32_t    numBlocks;          // block count per device
  uint16_t    pagesPerBlock;      // page count per block
  uint16_t    pagesPerBlockPower2;     // power of 2 shift for page count per block
  uint16_t    dataBytesPerPage;   // Number of bytes in a page
  uint16_t    dataBytesPerPagePower2;  // power of 2 shift for number of bytes in a page
  uint16_t    spareBytesPerPage;  // Number of spare bytes in a page  
  uint16_t    dataBytesPerOp;     // Number of bytes per operation
  uint16_t    spareBytesPerOp;    // Number of spare bytes per operation
  uint8_t     numOpsPerPage;      // Number of operations to complete a page read/write  
  uint8_t     numColAddrBytes;    // Number of Column address cycles
  uint8_t     numRowAddrBytes;    // Number of Row address cycles
  uint8_t     CSOffset;           // 0 for CS2 space, 1 for CS3 space, 2 for CS4 space, 3 for CS5 space  
  uint16_t    isLargePage;        // TRUE = Big block device, FALSE = small block device
  uint16_t    isONFI;             // TRUE = ONFI-compatible device, FALSE = non-ONFI device
  int32_t     currBlock;          // current Block in use
  uint16_t    isBlockGood;        // TRUE=current block is good, FALSE=block is bad
  void        *hPageLayout;
  struct _NAND_ECC_INFO_ *hEccInfo;
  void        *hBbInfo;
  void        *hChipInfo;
} 
NAND_InfoObj, *NAND_InfoHandle;

typedef struct _NAND_ECC_INFO_
{
  volatile uint16_t ECCEnable;                 // Error correction enable (should be on by default)  
  volatile uint16_t calcECCByteCnt;            // Count of ECC bytes per op as read from HW registers
  volatile uint16_t storedECCByteCnt;          // Count of ECC bytes per op as stored in the NAND page
  volatile uint16_t unused;                    // Filler
  volatile uint32_t pFxnCalculate;
  volatile uint32_t pFxnStore;
  volatile uint32_t pFxnEnable;
  volatile uint32_t pFxnDisable;
  volatile uint32_t pFxnRead;
  volatile NAND_ECC_Correct pFxnCorrect;
}
NAND_ECC_InfoObj;

// AEMIF Register structure - From EMIF 2.5 Spec
typedef struct _DEVICE_EMIF25_REGS_
{
  volatile uint32_t ERCSR;              // 0x00
  volatile uint32_t AWCCR;              // 0x04
  volatile uint32_t SDBCR;              // 0x08
  volatile uint32_t SDRCR;              // 0x0C

  volatile uint32_t A1CR;               // 0x10
  volatile uint32_t A2CR;               // 0x14
  volatile uint32_t A3CR;               // 0x18
  volatile uint32_t A4CR;               // 0x1C

  volatile uint32_t SDTIMR;             // 0x20
  volatile uint32_t SDRSTAT;            // 0x24
  volatile uint32_t DDRPHYCR;           // 0x28
  volatile uint32_t DDRPHYSR;           // 0x2C

  volatile uint32_t SDRACCR;            // 0x30
  volatile uint32_t SDRACT;             // 0x34
  volatile uint32_t DDRPHYREV;          // 0x38
  volatile uint32_t SDRSRPDEXIT;        // 0x3C

  volatile uint32_t EIRR;               // 0x40
  volatile uint32_t EIMR;               // 0x44
  volatile uint32_t EIMSR;              // 0x48
  volatile uint32_t EIMCR;              // 0x4C

  volatile uint32_t IOCR;               // 0x50
  volatile uint32_t IOSR;               // 0x54
  volatile uint8_t RSVD0[4];            // 0x58
  volatile uint32_t ONENANDCTL;         // 0x5C  

  volatile uint32_t NANDFCR;            // 0x60
  volatile uint32_t NANDFSR;            // 0x64
  volatile uint32_t PMCR;               // 0x68
  volatile uint8_t RSVD1[4];            // 0x6C

  volatile uint32_t NANDF1ECC;          // 0x70
  volatile uint32_t NANDF2ECC;          // 0x74
  volatile uint32_t NANDF3ECC;          // 0x78
  volatile uint32_t NANDF4ECC;          // 0x7C

  volatile uint8_t RSVD2[4];            // 0x80
  volatile uint32_t IODFTEXECNT;        // 0x84
  volatile uint32_t IODFTGBLCTRL;       // 0x88
  volatile uint8_t RSVD3[4];            // 0x8C

  volatile uint32_t IODFTMISRLSB;       // 0x90
  volatile uint32_t IODFTMISRMID;       // 0x94
  volatile uint32_t IODFTMISRMSB;       // 0x98
  volatile uint8_t RSVD4[20];           // 0x9C

  volatile uint32_t MODRELNUM;          // 0xB0
  volatile uint8_t RSVD5[8];            // 0xB4
  volatile uint32_t NAND4BITECCLOAD;    // 0xBC

  volatile uint32_t NAND4BITECC1;       // 0xC0
  volatile uint32_t NAND4BITECC2;       // 0xC4
  volatile uint32_t NAND4BITECC3;       // 0xC8
  volatile uint32_t NAND4BITECC4;       // 0xCC

  volatile uint32_t NANDERRADD1;        // 0xD0
  volatile uint32_t NANDERRADD2;        // 0xD4
  volatile uint32_t NANDERRVAL1;        // 0xD8
  volatile uint32_t NANDERRVAL2;        // 0xDC
}
DEVICE_Emif25Regs;

// D800K008 boot loader struct definition (for patching abort function)

// Dummy declaration
struct _BOOTLOADER_CONFIG_;

typedef void (*voidFxnPtr) (void);
typedef uint32_t (*blProtocolImplementFn) (struct _BOOTLOADER_CONFIG_ *bl1Handle, uint32_t state);
typedef uint32_t (*blOpenFn)              (struct _BOOTLOADER_CONFIG_ *bl1Handle);
typedef uint32_t (*blSyncWordExchangeFn)  (struct _BOOTLOADER_CONFIG_ *bl1Handle, uint32_t* syncWord);
typedef uint32_t (*blReadFn)              (struct _BOOTLOADER_CONFIG_ *bl1Handle, uint8_t *dest, uint32_t byteCnt);
typedef uint32_t (*blWriteFn)             (struct _BOOTLOADER_CONFIG_ *bl1Handle, uint8_t *src, uint32_t byteCnt);
typedef uint32_t (*blCloseFn)             (struct _BOOTLOADER_CONFIG_ *bl1Handle);
typedef uint32_t (*blPatchFn)             (struct _BOOTLOADER_CONFIG_ *bl1Handle);
typedef void     (*blAbortFn)             (struct _BOOTLOADER_CONFIG_ *bl1Handle, uint32_t statusValue);

typedef struct _BOOTLOADER_CONFIG_
{
    // 0x00
    uint8_t                 state;
    uint8_t                 errorCode;
    uint16_t                isMaster;    //unsigned short

    //0x04
    uint8_t                 crcEnabled;
    uint8_t                 bootPeripheral;
    uint8_t                 devNumber;
    uint8_t                 bitsPerShot;

    //0x08
    uint8_t                 seqRead;
    uint8_t                 crcErrorCount;
    uint16_t                isSecure; //unsigned short

    //0x0C
    uint8_t                 unused0C;
    uint8_t                 keyInstalled;
    uint16_t                bootPins;
  
    //0x10
    uint16_t                bootMode;
    uint8_t                 unused12[2];
    uint32_t                secureType;

    //0x18
    uint32_t                exitType;
    uint16_t                skStarted;  // unsigned short
    //0x1E
    uint8_t                 unused1E[2];

    //0x20
    uint32_t                bitsPerShotShift;
    uint32_t                signatureByteCnt;

    //0x28
    uint16_t                addressBits;
    uint16_t                unused2A;
    uint16_t                unused2C;
    uint16_t                unused2E;

    //0x30
    uint32_t                addr;                    // hpiJumpAddr
    uint32_t                data;
    uint32_t                opcode;
    uint32_t                crc;

    //0x40
    voidFxnPtr              entryPoint;
    uint32_t                seqRdAddr;
    uint32_t                bootSearchOffset;
    volatile uint32_t       *bootSearchOffsetAddr;

    //0x50
    uint32_t                crcReturnAddress;
    uint32_t                robustMode;
    blSyncWordExchangeFn    syncWordExchange;
    void *REGS;

    //0x60
    blOpenFn                openPeripheral;
    blReadFn                readBytes;
    blWriteFn               writeBytes;
    blCloseFn               closePeripheral;

    //0x70
    blPatchFn               patch;
    blProtocolImplementFn   protocolImplement;
    voidFxnPtr              finalFxn;
    blAbortFn               abortFxn;

    //0x80 - Used by NAND and SDMMC driver
    void                    *nandInfo;
    void                    *sdmmcMemInfo;
    uint32_t                nandCurrPage;
    uint32_t                nandCurrBlock;
  
    //0x90
    uint8_t                 *memBuffer;
    uint32_t                memCurrAddr;
    uint32_t                memBufferPtr;
    uint32_t                bytesLeftInBuff;  
}
BL_CfgObject, *BL_CfgHandle;

// NAND function types
typedef uint32_t (* NAND_readPage) (struct _NAND_INFO_ *nandInfo, uint32_t block, uint32_t page, uint8_t *dest);
typedef uint32_t (* NAND_badBlockCheck) (struct _NAND_INFO_ *nandInfo, uint32_t block);

// Secure boot function types
typedef uint32_t (* SBL1_init) (struct  _BOOTLOADER_CONFIG_ *bl1Handle);

/************************************************************
* Local Macro Declarations                                  *
************************************************************/

#define AEMIF ((DEVICE_Emif25Regs*) 0x68000000u)
#define BL_MST_KEYWORD_CHECK        (1)
#define BL_ERROR_NONE               (0)
#define BL_ERROR_NAND_READ_FAIL     (26)
#define NAND_MAX_UBL_SEARCH_BLOCK   (32)
#define E_PASS (0x00000000u)

#define TRUE    ((unsigned short) 1)

/************************************************************
* Local Function Declarations                               *
************************************************************/

static uint32_t NAND_ECC_correct(void *unused, uint8_t *data, uint8_t *readECC);


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

#pragma DATA_SECTION(NAND_ECC_info,".data:nand_ecc_patch")
NAND_ECC_InfoObj NAND_ECC_info;


/************************************************************
* Global Function Definitions                               *
************************************************************/

#pragma CODE_SECTION (BL_NAND_abortBoot, ".text:bl1_nand_abort")
void BL_NAND_abortBoot(BL_CfgHandle bl1Handle, uint32_t statusValue)
{
 #if defined(__TMS320C6X__)
    NAND_readPage pNAND_rp = (NAND_readPage) 0x00714aac;
    NAND_badBlockCheck pNAND_BBCheck = (NAND_badBlockCheck) 0x0071494c;
    blAbortFn  pBL1_abortBoot = (blAbortFn) 0x00712100;
 #else
    NAND_readPage pNAND_rp = (NAND_readPage) 0xfffd0534;
    NAND_badBlockCheck pNAND_BBCheck = (NAND_badBlockCheck) 0xfffd039c;
    blAbortFn  pBL1_abortBoot = (blAbortFn) 0xfffd45b0;
 #endif
 
    bl1Handle->errorCode = statusValue;
 
    while ( (BL_ERROR_NAND_READ_FAIL == bl1Handle->errorCode) &&
            (bl1Handle->nandCurrBlock < NAND_MAX_UBL_SEARCH_BLOCK) )
    {
        // Previous block failed. Move to start of the next.
        bl1Handle->nandCurrBlock +=1;
        bl1Handle->nandCurrPage = 0;

        // Check if the block is good, if not let's skip
        if ((*pNAND_BBCheck)(bl1Handle->nandInfo,bl1Handle->nandCurrBlock) != E_PASS)
        {
            continue;
        }

        // Try to buffer up the first page
        if ((*pNAND_rp)(bl1Handle->nandInfo, bl1Handle->nandCurrBlock, bl1Handle->nandCurrPage, bl1Handle->memBuffer) != E_PASS) 
        {
            bl1Handle->errorCode = BL_ERROR_NAND_READ_FAIL;
            continue;
        }

#if defined(__TMS320C6X__)
	if  ( bl1Handle->isSecure == TRUE)
	{
        SBL1_init pSbl_init= (SBL1_init)0x007fc600;

        // Secure boot loader init (needed for booting both types of secure devices)
        (*pSbl_init)(bl1Handle);
       }
#endif

        // Get back into the AIS parsing routine
        bl1Handle->signatureByteCnt = 0;
        bl1Handle->addr = 0;
        bl1Handle->memCurrAddr = 0;
        bl1Handle->memBufferPtr = 0;
        bl1Handle->bytesLeftInBuff = ((NAND_InfoHandle)bl1Handle->nandInfo)->dataBytesPerPage;
        
        bl1Handle->errorCode = (*(bl1Handle->protocolImplement)) (bl1Handle, BL_MST_KEYWORD_CHECK);
        if (BL_ERROR_NONE == bl1Handle->errorCode)
        {
            // Boot image was completely parsed, so we can exit this patched abort function and finish the boot.
            // We should be returning to the BL1_startBoot function in bl1.c
            return;
        }
    }
        
    // For all other errors, or if we run out of blocks, it's crash and burn time in the original ROM abort fxn
    pBL1_abortBoot(bl1Handle, statusValue);
}

#pragma CODE_SECTION(NAND_ECC_patchApply,".text:nand_ecc_patch")
void NAND_ECC_patchApply( void )
{
#if defined(__TMS320C6X__)
    uint8_t *rev = (uint8_t *) 0x0070000F;
	BL_CfgHandle bl1Handle = (BL_CfgHandle) 0x00F00700;
#else
    uint8_t *rev = (uint8_t *) 0xFFFD000F;
	BL_CfgHandle bl1Handle = (BL_CfgHandle) 0xFFFF0700;
#endif
    NAND_ECC_info.ECCEnable         = 1;
    NAND_ECC_info.calcECCByteCnt    = 16;
    NAND_ECC_info.storedECCByteCnt  = 10;
    NAND_ECC_info.pFxnCorrect       = &NAND_ECC_correct;
    
    if ((*rev) == 0x36)
    {
        /* ROM Version D800K006 */
#if defined(__TMS320C6X__)
        /* DSP ROM addresses */
        NAND_ECC_info.pFxnCalculate   = 0x00715920;   // &(DEVICE_NAND_ECC_calculate)
        NAND_ECC_info.pFxnStore       = 0x007159b0;   // &(DEVICE_NAND_ECC_store)
        NAND_ECC_info.pFxnEnable      = 0x00715af8;   // &(DEVICE_NAND_ECC_enable)
        NAND_ECC_info.pFxnDisable     = 0x00715b60;   // &(DEVICE_NAND_ECC_disable)
        NAND_ECC_info.pFxnRead        = 0x00715bc0;   // &(DEVICE_NAND_ECC_read)
        
        /* Replace the NAND_ECC_Info pointer in the NAND_Info structure in RAM */
        *((NAND_ECC_InfoObj **) 0x00F007C4) = &NAND_ECC_info;        
#else
        /* ARM ROM addresses */
        NAND_ECC_info.pFxnCalculate   = 0xFFFD4358;   // &(DEVICE_NAND_ECC_calculate),
        NAND_ECC_info.pFxnStore       = 0xFFFD43D8;   // &(DEVICE_NAND_ECC_store),
        NAND_ECC_info.pFxnEnable      = 0xFFFD4540;   // &(DEVICE_NAND_ECC_enable),
        NAND_ECC_info.pFxnDisable     = 0xFFFD459C;   // &(DEVICE_NAND_ECC_disable),
        NAND_ECC_info.pFxnRead        = 0xFFFD45F8;   // &(DEVICE_NAND_ECC_read),
        
        /* Replace the NAND_ECC_Info pointer in the NAND_Info structure in RAM */
        *((NAND_ECC_InfoObj **) 0xFFFF07C4) = &NAND_ECC_info;
		
#endif
    }
    else if ((*rev) == 0x38)
    {
        /* ROM Version D800K008 */
		bl1Handle->abortFxn = &BL_NAND_abortBoot;
#if defined(__TMS320C6X__)
        /* DSP ROM addresses */
        NAND_ECC_info.pFxnCalculate   = 0x00716260;   // &(DEVICE_NAND_ECC_calculate)
        NAND_ECC_info.pFxnStore       = 0x007162e4;   // &(DEVICE_NAND_ECC_store)
        NAND_ECC_info.pFxnEnable      = 0x00716410;   // &(DEVICE_NAND_ECC_enable)
        NAND_ECC_info.pFxnDisable     = 0x00716468;   // &(DEVICE_NAND_ECC_disable)
        NAND_ECC_info.pFxnRead        = 0x007164b8;   // &(DEVICE_NAND_ECC_read)
        
        /* Replace the NAND_ECC_Info pointer in the NAND_Info structure in RAM */
        *((NAND_ECC_InfoObj **) 0x00F007D0) = &NAND_ECC_info;
#else
        /* ARM ROM addresses */
        NAND_ECC_info.pFxnCalculate   = 0xFFFD5B00;   // &(DEVICE_NAND_ECC_calculate)
        NAND_ECC_info.pFxnStore       = 0xFFFD5B80;   // &(DEVICE_NAND_ECC_store)
        NAND_ECC_info.pFxnEnable      = 0xFFFD5CE8;   // &(DEVICE_NAND_ECC_enable)
        NAND_ECC_info.pFxnDisable     = 0xFFFD5D44;   // &(DEVICE_NAND_ECC_disable)
        NAND_ECC_info.pFxnRead        = 0xFFFD5DA0;   // &(DEVICE_NAND_ECC_read)
        
        /* Replace the NAND_ECC_Info pointer in the NAND_Info structure in RAM */
        *((NAND_ECC_InfoObj **) 0xFFFF07D0) = &NAND_ECC_info;
#endif
    }
}



/************************************************************
* Local Function Definitions                                *
************************************************************/

#pragma CODE_SECTION(NAND_ECC_correct,".text:nand_ecc_patch")
static uint32_t NAND_ECC_correct(void *unused, uint8_t *data, uint8_t *readECC)
{
  volatile uint32_t temp, corrState, numE;
  int i;
  uint16_t addOffset, corrValue;
  uint16_t* syndrome10 = (uint16_t *)readECC;

  // Clear bit13 of NANDFCR
  temp = AEMIF->NANDERRADD1;
  
  // Load the syndrome10 (from 7 to 0) values
  for(i=8;i>0;i--)
  {
    AEMIF->NAND4BITECCLOAD = (syndrome10[i-1] & 0x000003FF);
  }
  
  // Read the EMIF status and version (dummy call) 
  temp = AEMIF->ERCSR;
  
  // Check if error is detected
  temp = (AEMIF->NAND4BITECC1 & 0x03FF03FF) | (AEMIF->NAND4BITECC2 & 0x03FF03FF) |
         (AEMIF->NAND4BITECC3 & 0x03FF03FF) | (AEMIF->NAND4BITECC4 & 0x03FF03FF);
  if(temp == 0)
  {
    return 0;
  }

  // Start calcuating the correction addresses and values
  AEMIF->NANDFCR |= (0x1U << 13);

  // Read the EMIF status and version (dummy call to allow previous write to complete) 
  temp = AEMIF->ERCSR;
  
  // Loop until timeout or the ECC calculations are complete (bit 11:10 == 00b)
  i = 1000;
  do
  {
    temp = (AEMIF->NANDFSR & 0x00000F00)>>10;
    i--;
  }
  while((i>0) && (temp != 0x0));

  // Read final correction state (should be 0x0, 0x1, 0x2, or 0x3)
  corrState = (AEMIF->NANDFSR & 0x00000F00) >> 8;

  if ((corrState == 1) || (corrState > 3))
  {
    temp = AEMIF->NANDERRVAL1;
    return 1;
  }
  else if (corrState == 0)
  {
    return 0;
  }
  else
  {
    // Error detected and address calculated
    // Number of errors corrected 17:16
    numE = (AEMIF->NANDFSR & 0x00030000) >> 16;

    switch( numE )
    {
      case 3:     // Four errors
        addOffset = 519 - ( (AEMIF->NANDERRADD2 & (0x03FF0000))>>16 );
        if (addOffset < 512)
        {
            // Error in the user data region can be corrected
            corrValue = (AEMIF->NANDERRVAL2 & (0x03FF0000))>>16;
            data[addOffset] ^= (uint8_t)corrValue;
        }
        // Fall through to case 2
      case 2:     // Three errors
        addOffset = 519 - (AEMIF->NANDERRADD2 & (0x000003FF));
        if (addOffset < 512)
        {
            // Error in the user data region can be corrected
            corrValue = AEMIF->NANDERRVAL2 & (0x000003FF);
            data[addOffset] ^= (uint8_t)corrValue;
        }
        // Fall through to case 1
      case 1:     // Two errors
        addOffset = 519 - ( (AEMIF->NANDERRADD1 & (0x03FF0000))>>16 );
        if (addOffset < 512)
        {
            // Error in the user data region can be corrected
            corrValue = (AEMIF->NANDERRVAL1 & (0x03FF0000))>>16;
            data[addOffset] ^= (uint8_t)corrValue;
        }
        // Fall through to case 0
      case 0:     // One error
        addOffset = 519 - (AEMIF->NANDERRADD1 & (0x000003FF));
        if (addOffset < 512)
        {
            // Error in the user data region can be corrected
            corrValue = AEMIF->NANDERRVAL1 & (0x3FF);
            data[addOffset] ^= (uint8_t)corrValue;
        }
        break;
    }
    return 0;
  }
}



/***********************************************************
* End file                                                 *
***********************************************************/


