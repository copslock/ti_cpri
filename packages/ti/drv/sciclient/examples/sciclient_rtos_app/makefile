#
# This file is the makefile for building Sciclient firmware boot test application.
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = sciclient_rtos_app
BUILD_OS_TYPE=tirtos

SRCDIR = . $(PDK_INSTALL_PATH)/ti/drv/sciclient/examples/common
INCDIR = . $(PDK_INSTALL_PATH)/ti/drv/sciclient/examples/common

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk sciclient osal_tirtos board uart bios xdc

# List all the components required by the application
COMP_LIST_COMMON  = sciclient csl osal_tirtos board uart bios xdc i2c

ifeq ($(ARCH),c66x)
  COMP_LIST_COMMON += csl_intc
else
  ifneq ($(ARCH),c71)
    COMP_LIST_COMMON += csl_init
  endif
endif

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON = sciclient_rtos_main.c \
			  sciclient_appCommon.c

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS)

PACKAGE_SRCS_COMMON = . ../common

# Enable XDC build for application by providing XDC CFG File per core
XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/build/$(SOC)/sysbios_$(ISA).cfg


# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<coqre/SoC/platform-name> =
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
