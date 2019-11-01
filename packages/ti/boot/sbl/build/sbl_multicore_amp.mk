#
# This file is the makefile for building a testcase
# to check multicore boot using SBL.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

APP_NAME = sbl_multicore_amp
BUILD_OS_TYPE = baremetal

SRCDIR      += $(PDK_SBL_COMP_PATH)/example/k3MulticoreApp

# Local name of SBL test app
RPRC_PREFIX = sbl_$(BUILD_OS_TYPE)_boot_test_$(BOARD)

MULTICORE_IMG_PARAMS = $(foreach SOC_CORE_ID, $(sbl_boot_test_$(SOC)_CORELIST), $(SBL_CORE_ID_$(SOC_CORE_ID)) $(BINDIR)/$(RPRC_PREFIX)_$(SOC_CORE_ID)TestApp_$(BUILD_PROFILE_$(CORE)).rprc)

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
	$(ECHO) "# Multicore SBL App image $(BINDIR)/$(RPRC_PREFIX)_all_coresTestApp_$(BUILD_PROFILE_$(CORE)).appimage created."
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

