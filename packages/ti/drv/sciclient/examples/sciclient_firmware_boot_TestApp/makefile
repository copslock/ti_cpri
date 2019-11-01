#
# This file is the makefile for building Sciclient firmware boot test application.
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = sciclient_firmware_boot_TestApp
BUILD_OS_TYPE=baremetal

SRCDIR = . ../common
INCDIR = .

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk sciclient osal_nonos board uart i2c

# List all the components required by the application
COMP_LIST_COMMON  = sciclient csl osal_nonos board uart i2c
ifeq ($(ARCH),c66x)
  COMP_LIST_COMMON += csl_intc
else
  ifneq ($(ARCH),c71)
    COMP_LIST_COMMON += csl_init
  endif
endif

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON = sciclient_firmware_boot_main.c \
              sciclient_appCommon.c

ifeq ($(CORE), mcu1_0)
	COMP_LIST_COMMON += mmcsd fatfs_indp
	SRCS_ASM_COMMON += boot_init.asm
endif

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) -DBARE_METAL
PACKAGE_SRCS_COMMON = . ../common

CONFIG_BLD_LNK_r5f = $(PDK_INSTALL_PATH)/ti/drv/sciclient/examples/$(APP_NAME)/linker_r5.lds

ifeq ($(CORE),mpu1_0)
    LNKFLAGS_LOCAL_mpu1_0 += --entry Entry
endif
ifeq ($(CORE), mcu1_0)
    LNKFLAGS_LOCAL_mcu1_0 += --entry_point=_sciclientTestResetVectors
endif

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
