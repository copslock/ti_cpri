#
# This file is the makefile for building a testcase
# to check multicore boot using SBL.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

APP_NAME = sbl_multicore_smp
BUILD_OS_TYPE = baremetal

SRCDIR      += $(PDK_SBL_COMP_PATH)/example/k3MulticoreApp

# Must exactly match values defined in enum cpu_core_id
# in sbl_slave_core_boot.h
SBL_CORE_ID_mpu1_0_smp = 14
SBL_CORE_ID_mpu2_0_smp = 15
SBL_CORE_ID_mpu_smp = 16
SBL_CORE_ID_mcu1_0_smp = 17
SBL_CORE_ID_mcu2_0_smp = 18
SBL_CORE_ID_mcu3_0_smp = 19
SBL_CORE_ID_only_load = 20

  
# Local name of SBL test app
RPRC_PREFIX = sbl_$(BUILD_OS_TYPE)_smp_test_$(BOARD)

MULTICORE_IMG_PARAMS = $(foreach SOC_CORE_ID, $(sbl_smp_test_$(SOC)_CORELIST), $(SBL_CORE_ID_$(SOC_CORE_ID)_smp) $(BINDIR)/$(RPRC_PREFIX)_$(SOC_CORE_ID)TestApp_$(BUILD_PROFILE_$(CORE)).rprc)

# To build an image that test SBL_CORE_ID_mpu_smp
# specify only one mpu in am65xx_smp_CORELIST in
# sbl_component.mk and make sure it is the last
# core in the list. Then uncomment the below lines.
# also change $(SBL_IMAGE_GEN) below to use
# SMP_MULTICORE_IMG_PARAMS instead of MULTICORE_IMG_PARAMS
#SMP_CORE_ID = $(lastword $(sbl_smp_test_$(SOC)_CORELIST))
#SMP_MULTICORE_IMG_PARAMS = $(subst  $(SBL_CORE_ID_$(SMP_CORE_ID)) ,$(SBL_CORE_ID_mpu_smp) , $(MULTICORE_IMG_PARAMS))

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS)
PACKAGE_SRCS_COMMON = .

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES =

# List all the components required by the application
COMP_LIST_COMMON =

SRCS_COMMON = force_multi_core_img_gen.c

force_multi_core_img_gen.c:
	$(ECHO) "# Combining RPRC images to generate multicore image...."
	$(SBL_IMAGE_GEN) LE $(SBL_DEV_ID) $(BINDIR)/$(RPRC_PREFIX)_all_coresTestApp_$(BUILD_PROFILE_$(CORE)).appimage $(MULTICORE_IMG_PARAMS)
	$(ECHO) "#"
	$(ECHO) "# Multicore SMP App image $(BINDIR)/$(RPRC_PREFIX)_all_coresTestApp_$(BUILD_PROFILE_$(CORE)).appimage created."
	$(ECHO) "#"
	$(ECHO) "# Signing the multicore image...."
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(SBL_CERT_GEN)
endif
	$(SBL_CERT_GEN) -b $(BINDIR)/$(RPRC_PREFIX)_all_coresTestApp_$(BUILD_PROFILE_$(CORE)).appimage -o $(BINDIR)/$(RPRC_PREFIX)_all_coresTestApp_$(BUILD_PROFILE_$(CORE)).appimage.signed -c R5 -l $(SBL_RUN_ADDRESS) -k $(SBL_CERT_KEY)

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

