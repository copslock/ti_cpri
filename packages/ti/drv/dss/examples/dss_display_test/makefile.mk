#
# This file is the makefile for building DSS display test app for both TI RTOS
# and baremetal
#

SRCDIR = .
INCDIR = .

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# List all the components required by the application
COMP_LIST_COMMON = csl fvid2 dss
COMP_LIST_COMMON += board uart sciclient pm_lib i2c

ifeq ($(BUILD_OS_TYPE), baremetal)
  COMP_LIST_COMMON += osal_nonos csl_init dss_app_utils
  SRCS_COMMON = main_baremetal.c
  CFLAGS_LOCAL_COMMON += -DDSS_TESTAPP_BAREMETAL
  ifeq ($(ISA),$(filter $(ISA), a53))
    LNKFLAGS_LOCAL_$(CORE) += --entry Entry
  endif
else
  INCLUDE_EXTERNAL_INTERFACES += xdc bios
  COMP_LIST_COMMON += osal_tirtos dss_app_utils_sysbios
  SRCS_COMMON = main_tirtos.c
  # Enable XDC build for application by providing XDC CFG File per core
  XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/build/$(SOC)/sysbios_$(ISA).cfg
  XDC_CFG_UPDATE_$(CORE) = dss_display_test_prf.cfg
endif

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON = .
SRCS_COMMON += dss_display_test.c
CFLAGS_LOCAL_COMMON += $(PDK_CFLAGS) $(DSS_CFLAGS)

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
