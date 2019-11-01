#include <ti/drv/dss/src/csl/dp/csl_dp.h>
#include <ti/drv/dss/src/csl/dp_sd0801/csl_dp_sd0801.h>
#include <ti/drv/dss/dss.h>

#define ADDR_AFE                   (0x05050000)
#define DSSEDP0_CFGR1_BASE         (0x0A000000)
#define DSSEDP0_CFGR2_BASE         (0x04F48000)
#define APB_BASE                   DSSEDP0_CFGR1_BASE
#define SAPB_BASE                  DSSEDP0_CFGR2_BASE
#define PHAPB_BASE                 ADDR_AFE
#define APB_SIZE                   (0x00100000)
#define SAPB_SIZE                  (0x00100000)
#define PHAPB_SIZE                 (0x00100000)

/*
 * CDNS PHY memory map is based on 16b halfword in 32-bit word address
 * TI wiz wrapper collapsed them into 16b word addresses.
 * But, kept the same section addressing [17:14]
 * cdn_phapb_read/write converts CDNS PHY access to TI access
 * by mapping addr[17:14] in 15:12 and addr[12:2] in 11:1
 */
static uint32_t cdn_phapb_xlate(uint32_t addr)
{
    return (((addr & 0x3C000) >> 2) | ((addr & 0x1FFC) >> 1));
}

static uint32_t cdn_phapb_read(uint32_t addr)
{
    return CSL_REG16_RD(ADDR_AFE + cdn_phapb_xlate(addr));
}

static void cdn_phapb_write(uint32_t addr, uint32_t data)
{
    CSL_REG16_WR(ADDR_AFE + cdn_phapb_xlate(addr), (0xFFFF & data));
}

static uint32_t cdn_apb_read(uint32_t addr)
{
    return CSL_REG32_RD(DSSEDP0_CFGR1_BASE + addr);
}

static void cdn_apb_write(uint32_t addr, uint32_t data)
{
    CSL_REG32_WR(DSSEDP0_CFGR1_BASE + addr, data);
}

static uint32_t cdn_sapb_read(uint32_t addr)
{
    return CSL_REG32_RD(DSSEDP0_CFGR2_BASE + addr);
}

static void cdn_sapb_write(uint32_t addr, uint32_t data)
{
    CSL_REG32_WR(DSSEDP0_CFGR2_BASE + addr, data);
}

void CPS_BufferCopy(volatile uint8_t *dst, volatile const uint8_t *src, uint32_t size)
{
    memcpy((void *)dst, (void *)src, size);
}

void CPS_DelayNs(uint32_t ns)
{
    uint32_t us = ns / 1000;
    if(0U == us)
    {
        /* If less than 1 us sleep requested, fall back to 1 us */
        us = 1;
    }

    Osal_delay(us);
}

void CPS_ExtPhyReset(bool reset)
{
    uint32_t regVal;

    regVal = CSL_REG32_RD(ADDR_AFE + 0x40C);
    CSL_FINSR(regVal, 31, 31, reset ? 0 : 1);
    CSL_REG32_WR(ADDR_AFE + 0x40C, regVal);
}

uint32_t CPS_ReadReg32(volatile uint32_t* address)
{
    uint32_t data;
    uintptr_t address_int = (uintptr_t)address;

    if ((address_int >= APB_BASE) &&
    (address_int < APB_BASE + APB_SIZE))
    {
        data = cdn_apb_read(address_int - APB_BASE);
    }
    else if ((address_int >= SAPB_BASE) &&
    (address_int < SAPB_BASE + SAPB_SIZE))
    {
        data = cdn_sapb_read(address_int - SAPB_BASE);
    }
    else if((address_int >= PHAPB_BASE) &&
    (address_int < PHAPB_BASE + PHAPB_SIZE))
    {
        data = cdn_phapb_read(address_int - PHAPB_BASE);
    }
    else
    {
        GT_1trace(DssTrace,
                  GT_ERR,
                  "Address %d (read) doesn't map to any DP bus",
                  address);
        data = 0u;
    }

    return data;
}

void CPS_WriteReg32(volatile uint32_t* address, uint32_t value)
{
    uintptr_t address_int = (uintptr_t)address;

    if((address_int >= APB_BASE) &&
    (address_int < APB_BASE + APB_SIZE))
    {
        cdn_apb_write(address_int - APB_BASE, value);
    }
    else if ((address_int >= SAPB_BASE) &&
    (address_int < SAPB_BASE + SAPB_SIZE))
    {
        cdn_sapb_write(address_int - SAPB_BASE, value);
    }
    else if((address_int >= PHAPB_BASE) &&
    (address_int < PHAPB_BASE + PHAPB_SIZE))
    {
        cdn_phapb_write(address_int - PHAPB_BASE, value);
    }
    else
    {
        GT_1trace(DssTrace,
                  GT_ERR,
                  "Address %d (write) doesn't map to any DP bus",
                  address);
    }
}
