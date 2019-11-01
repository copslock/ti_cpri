#
# This file is common makefile for building UDMA memcpy test app for both TI-RTOS/baremetal
#
SRCDIR = .
INCDIR =

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# List all the components required by the application
COMP_LIST_COMMON = csl udma sciclient udma_apputils
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

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON = .
SRCS_COMMON += udma_memcpy_test.c

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS)

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
