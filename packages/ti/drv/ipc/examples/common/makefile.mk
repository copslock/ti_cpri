#
# This file is the makefile for building IPC example app for TI RTOS
#
SRCDIR += . ../common/src
INCDIR =

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# List all the components required by the application
COMP_LIST_COMMON = csl ipc sciclient
COMP_LIST_COMMON += board i2c uart
ifeq ($(BUILD_OS_TYPE), baremetal)
  COMP_LIST_COMMON += csl_init osal_nonos
  SRCS_COMMON = main_baremetal.c
  ifeq ($(ISA),$(filter $(ISA), a53, a72))
    LNKFLAGS_LOCAL_$(CORE) += --entry Entry
  endif
else
  INCLUDE_EXTERNAL_INTERFACES += xdc bios
  COMP_LIST_COMMON += osal_tirtos
  SRCS_COMMON += main_tirtos.c
  # Enable XDC build for application by providing XDC CFG File per core
  XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/build/$(SOC)/sysbios_$(ISA).cfg
ifeq ($(SOC),$(filter $(SOC), j721e))
  XDC_CFG_UPDATE_$(CORE) = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/j721e/ipc_override.cfg
  ifeq ($(ECHO_TEST_BTCM), 1)
    ifeq ($(ISA), r5f)
	  XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/j721e/sysbios_$(ISA).cfg
	  EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/j721e/linker_$(ISA)_$(CORE)_btcm_sysbios.lds
    else
	  EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/j721e/linker_$(ISA)_$(CORE)_sysbios.lds
    endif
  else
    ifeq ($(ISA), r5f)
	  XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/j721e/sysbios_$(ISA).cfg
    endif
	EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/j721e/linker_$(ISA)_$(CORE)_sysbios.lds
  endif
endif
ifeq ($(SOC),$(filter $(SOC), am65xx))
  XDC_CFG_UPDATE_$(CORE) = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/am65xx/ipc_override.cfg
  ifeq ($(ISA), r5f)
    XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/am65xx/sysbios_$(ISA).cfg
  endif
  EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/am65xx/linker_$(ISA)_$(CORE)_sysbios.lds
endif
endif

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON = . ../common ../../common
SRCS_COMMON += ipc_utils.c ipc_testsetup.c

CFLAGS_LOCAL_COMMON += $(PDK_CFLAGS)

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
