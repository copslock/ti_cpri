#
# This file is the makefile for building sbl library.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

MODULE_NAME = sbl_lib_$(BOOTMODE)

INCDIR	+= $(PDK_INSTALL_PATH)
INCDIR	+= $(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(SOC)

INCDIR	+= $(PDK_SBL_COMP_PATH)
INCDIR	+= $(PDK_SBL_COMP_PATH)/soc
INCDIR	+= $(PDK_SBL_COMP_PATH)/board/src
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/rprc
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/ospi
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/hyperflash
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/mmcsd
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/uart
INCDIR	+= $(PDK_SBL_COMP_PATH)/soc/k3

SRCDIR	+= $(PDK_SBL_COMP_PATH)/board/src
SRCDIR	+= $(PDK_SBL_COMP_PATH)/src/rprc
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/ospi
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/hyperflash
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/mmcsd
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/uart
SRCDIR	+= $(PDK_SBL_COMP_PATH)/soc/k3
SRCDIR	+= $(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(SOC)

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES =
INCLUDE_INTERNAL_INTERFACES =

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(SBL_CFLAGS)
PACKAGE_SRCS_COMMON  = ./build ./src ./tools
PACKAGE_SRCS_COMMON += ./soc/sbl_soc.h ./soc/k3
PACKAGE_SRCS_COMMON += ./.gitignore ./sbl_component.mk ./makefile ./sbl_ver.h

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON = sbl_soc.c
SRCS_COMMON += sbl_rprc.c
SRCS_COMMON += sbl_slave_core_boot.c
SRCS_COMMON += UART_soc.c

SRCS_COMMON += sbl_sci_client.c
SRCS_COMMON += sbl_vid_map.c
SRCS_ASM_COMMON += sbl_misc.asm
SRCS_ASM_COMMON += sbl_init.asm

# scratch memory given to the SBL
# for image load and parsing
SBL_CFLAGS += -DSBL_SCRATCH_MEM_START=0xB8000000
SBL_CFLAGS += -DSBL_SCRATCH_MEM_SIZE=0x4000000

# Check for custom flags
ifeq ($(BOOTMODE), cust)
  SBL_CFLAGS = $(CUST_SBL_FLAGS)
endif # ifeq ($(BOOTMODE), cust)

# BOOTMODE specific CFLAGS
ifeq ($(BOOTMODE), mmcsd)
  SBL_CFLAGS+= -DBOOT_MMCSD
endif # ifeq ($(BOOTMODE), mmcsd)

ifeq ($(BOOTMODE), ospi)
  SBL_CFLAGS += -DBOOT_OSPI
endif # ifeq ($(BOOTMODE), ospi)

ifeq ($(BOOTMODE), hyperflash)
  SBL_CFLAGS += -DBOOT_HYPERFLASH
endif # ifeq ($(BOOTMODE), hyperflash)

ifeq ($(BOOTMODE), uart)
  SBL_CFLAGS += -DBOOT_UART
endif # ifeq ($(BOOTMODE), uart)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_MMCSD), -DBOOT_MMCSD)
  SRCS_COMMON += sbl_mmcsd.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_MMCSD), -DBOOT_MMCSD)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_OSPI), -DBOOT_OSPI)
  SRCS_COMMON += sbl_ospi.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_OSPI), -DBOOT_OSPI)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_HYPERFLASH), -DBOOT_HYPERFLASH)
  SRCS_COMMON += sbl_hyperflash.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_HYPERFLASH), -DBOOT_HYPERFLASH)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_UART), -DBOOT_UART)
  SRCS_COMMON += sbl_uart.c sbl_xmodem.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_UART), -DBOOT_UART)

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
