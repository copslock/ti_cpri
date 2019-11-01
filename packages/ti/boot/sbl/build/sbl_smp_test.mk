#
# This file is the makefile for building images used for SBL testing.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

APP_NAME = sbl_smp_test
BUILD_OS_TYPE = baremetal
LOCAL_APP_NAME = sbl_$(BUILD_OS_TYPE)_smp_test_$(BOARD)_$(CORE)TestApp

SBL_SRC_DIR =  $(PDK_INSTALL_PATH)/ti/boot/sbl

SRCDIR      += $(PDK_SBL_COMP_PATH)/example/k3MulticoreApp

INCDIR      += $(PDK_SBL_COMP_PATH)
INCDIR      += $(PDK_SBL_COMP_PATH)/soc
INCDIR      += $(PDK_SBL_COMP_PATH)/soc/k3
INCDIR      += $(PDK_SBL_COMP_PATH)/example/k3MulticoreApp

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(SBL_CFLAGS)
PACKAGE_SRCS_COMMON = .

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# List all the components required by the application
COMP_LIST_COMMON = csl_init csl uart osal_nonos

SRCS_COMMON += sbl_smp_multicore.c

ifeq ($(CORE), mcu1_0)
  COMP_LIST_COMMON += board sciclient i2c
endif

# asm files and linker scripts change due to different tool chains for R5 and A53
ifeq ($(CORE),$(filter $(CORE), mcu1_0 mcu2_0 mcu3_0))
  SRCS_ASM_COMMON = sbl_smp_r5.asm
  EXTERNAL_LNKCMD_FILE_LOCAL =  $(PDK_SBL_COMP_PATH)/example/k3MulticoreApp/$(CORE)_LockStepLinker.lds
  COMP_LIST_COMMON += sbl_lib_$(BOOTMODE) mmcsd fatfs_indp spi
endif

ifeq ($(CORE),$(filter $(CORE), mpu1_0 mpu2_0))
  LNKCMD_FILE = $(PDK_SBL_COMP_PATH)/example/k3MulticoreApp/$(CORE)_Smplinker.lds
  LNKFLAGS_LOCAL_$(CORE) += --entry Entry
endif

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

