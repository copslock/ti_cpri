#
# This file is the makefile for building UDMA unit test app for TI RTOS
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

APP_NAME = udma_unit_testapp
ifeq ($(UDMA_UT_MANUAL_ENTRY), yes)
APP_NAME = udma_user_input_unit_testapp
endif
ifeq ($(UDMA_UT_BAREMETAL), yes)
APP_NAME = udma_baremetal_unit_testapp
BUILD_OS_TYPE=baremetal
endif
ifeq ($(UDMA_UT_DYNAMIC_ANALYSIS), yes)
APP_NAME = udma_dynamic_unit_testapp
BUILD_OS_TYPE=baremetal
endif

#
# This file is common makefile for building UDMA unit test app for both TI-RTOS/baremetal
#
SRCDIR = . ./src ./src/soc/$(SOC) ./rtos ./baremetal
INCDIR = . ./src ./src/soc/$(SOC) ./rtos ./baremetal

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# List all the components required by the application
COMP_LIST_COMMON = csl udma sciclient fvid2 udma_apputils
COMP_LIST_COMMON += board uart i2c
ifeq ($(BUILD_OS_TYPE), baremetal)
  COMP_LIST_COMMON += osal_nonos
  ifeq ($(ARCH),c66x)
    COMP_LIST_COMMON += csl_intc
  else
    ifneq ($(ARCH),c71)
      COMP_LIST_COMMON += csl_init
    endif
  endif
  SRCS_COMMON = main_baremetal.c
  ifeq ($(ISA),$(filter $(ISA), a53 a72))
    LNKFLAGS_LOCAL_$(CORE) += --entry Entry
  endif
else
  INCLUDE_EXTERNAL_INTERFACES += xdc bios
  COMP_LIST_COMMON += osal_tirtos
  SRCS_COMMON = main_tirtos.c
  XDC_CFG_FILE_$(CORE) = $(PDK_INSTALL_PATH)/ti/build/$(SOC)/sysbios_$(ISA).cfg
endif

XDC_CFG_UPDATE_$(CORE) = udma_ut.cfg

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON = . ./src ../udma_ut_component.mk
SRCS_COMMON += main_common.c
SRCS_COMMON += udma_test_parser.c udma_test_blkcpy.c udma_test_common.c
SRCS_COMMON += udma_test_ring.c udma_test_proxy.c udma_test_ring_monitor.c
SRCS_COMMON += udma_test_event.c udma_test_flow.c
SRCS_COMMON += udma_test_bug.c udma_test_misc.c
SRCS_COMMON += udma_test_soc.c
ifeq ($(BUILD_OS_TYPE), baremetal)
SRCS_COMMON += utils_prf_baremetal.c utils_mem_baremetal.c
else
SRCS_COMMON += utils_prf.c utils_mem.c
endif

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(UDMA_CFLAGS) $(FVID2_CFLAGS)
ifeq ($(UDMA_UT_MANUAL_ENTRY), yes)
  CFLAGS_LOCAL_COMMON += -DUDMA_UT_ENABLE_MANUAL_ENTRY
endif
ifeq ($(UDMA_UT_BAREMETAL), yes)
  CFLAGS_LOCAL_COMMON += -DUDMA_UT_BAREMETAL
endif
ifeq ($(UDMA_UT_DYNAMIC_ANALYSIS), yes)
  CFLAGS_LOCAL_COMMON += -DUDMA_UT_DYNAMIC_ANALYSIS -DUDMA_UT_BAREMETAL
endif
ifeq ($(BUILD_OS_TYPE), baremetal)
  ifeq ($(ISA), c66)
    EXTERNAL_LNKCMD_FILE_LOCAL = baremetal/$(SOC)/linker_$(ISA).cmd
  else
    EXTERNAL_LNKCMD_FILE_LOCAL = baremetal/$(SOC)/linker_$(ISA).lds
  endif
else
EXTERNAL_LNKCMD_FILE_LOCAL = rtos/$(SOC)/linker_$(CORE).lds
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
