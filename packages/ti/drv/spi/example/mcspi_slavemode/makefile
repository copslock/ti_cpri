# Makefile for SPI unit test app
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

ifeq ($(IS_BAREMETAL),yes)
BUILD_OS_TYPE = baremetal
CFLAGS_OS_DEFINES =
LNKFLAGS_LOCAL_mpu1_0  += --entry Entry
COMP_LIST_COMMON =  csl_init osal_nonos
EXTERNAL_INTERFACES =
XDC_CFG_FILE_mpu1_0 =
XDC_CFG_FILE_mcu1_0 =
else
BUILD_OS_TYPE = tirtos
CFLAGS_OS_DEFINES = -DUSE_BIOS
EXTERNAL_INTERFACES = bios xdc
COMP_LIST_COMMON    = osal_tirtos

ifeq ($(SOC),$(filter $(SOC), am65xx))
XDC_CFG_FILE_mpu1_0 = ./am65xx/mcspiMasterSlaveTest_a53.cfg
XDC_CFG_FILE_mcu1_0 = ./am65xx/mcspiMasterSlaveTest_r5.cfg
else
ifeq ($(SOC),$(filter $(SOC), j721e))
XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/build/$(SOC)/sysbios_$(ISA).cfg
endif
endif

endif

SRCDIR = . src
INCDIR = . src

ifeq ($(BUILD_OS_TYPE), tirtos)
MCSPI_OS_TESTPREFIX=
else
MCSPI_OS_TESTPREFIX=_Baremetal
endif


# Common source files across all platforms and cores
SRCS_COMMON += main_mcspi_slave_mode.c SPI_log.c

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk $(EXTERNAL_INTERFACES)

# DMA enabled build
ifeq ($(DMA), enable)

#Name of the directory created under packages/ti/binary/
APP_NAME = MCSPI$(MCSPI_OS_TESTPREFIX)_MasterSlave_Dma_TestApp
# Name of the binary if different from the default (APP_NAME)_$(BOARD_$(CORE)_<build_profile>
LOCAL_APP_NAME =  MCSPI$(MCSPI_OS_TESTPREFIX)_MasterSlave_Dma_$(BOARD)_$(CORE)TestApp

COMP_LIST_COMMON += spi_dma udma sciclient i2c
CFLAGS_SPI_DMA    = -DSPI_DMA_ENABLE
else

#Name of the directory created under packages/ti/binary/
APP_NAME = MCSPI$(MCSPI_OS_TESTPREFIX)_MasterSlave_TestApp

# Name of the binary if different fmake clrom the default (APP_NAME)_$(BOARD_$(CORE)_<build_profile>
LOCAL_APP_NAME =  MCSPI$(MCSPI_OS_TESTPREFIX)_MasterSlave_$(BOARD)_$(CORE)TestApp


COMP_LIST_COMMON += spi sciclient i2c
CFLAGS_SPI_DMA    =

endif

# Master/Slave build
ifeq ($(SLAVE), 1)
CFLAGS_SPI_MS     = -DMCSPI_SLAVE_TASK
else
CFLAGS_SPI_MS     = -DMCSPI_MASTER_TASK
endif

# List all the components required by the application
COMP_LIST_COMMON  += csl board uart
# am65xx build
ifeq ($(BOARD),$(filter $(BOARD), am65xx_evm am65xx_idk j721e_sim j721e_evm))
CFLAGS_SPI_UT    = -DMCSPI_MULT_CHANNEL
endif

PACKAGE_SRCS_COMMON = .
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(CFLAGS_SPI_UT) $(CFLAGS_SPI_DMA) $(CFLAGS_SPI_MS) $(CFLAGS_OS_DEFINES)

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<core/SoC/platform-name> =
#   CFLAGS_LOCAL_<core/SoC/platform-name> =

# Include common make files
ifeq ($(MAKERULEDIR), )
#Makerule path not defined, define this and assume relative path from ROOTDIR
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif
include $(MAKERULEDIR)/common.mk

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
