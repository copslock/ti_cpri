# Makefile for PCIE sample app
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

ifeq ($(SMP), enable)
    #Name of the directory created under packages/ti/binary/
    APP_NAME = PCIE_sample_SMP_ExampleProject
    # Name of the binary if different from the default (APP_NAME)_$(BOARD_$(CORE)_<build_profile>
    LOCAL_APP_NAME =  PCIE_sample__$(BOARD)_$(CORE)_SMP_Example_Project

    CFLAGS_PCIE_SMP    = -DPCIE_SMP_ENABLE
else
    #Name of the directory created under packages/ti/binary/
    APP_NAME = PCIE_sample_ExampleProject
    # Name of the binary if different from the default (APP_NAME)_$(BOARD_$(CORE)_<build_profile>
    LOCAL_APP_NAME =  PCIE_sample__$(BOARD)_$(CORE)Example_Project
endif


ifeq ($(SOC),$(filter $(SOC), am65xx))
SRCDIR = . src am65xx/src udma
INCDIR = . src am65xx/src udma
# Common source files across all platforms and cores
SRCS_COMMON += pcie_sample.c pcie_sample_board.c pcie_udma.c
endif

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = bios xdc pdk

# List all the components required by the application
COMP_LIST_COMMON = pcie uart osal_tirtos csl board udma sciclient

ifeq ($(CORE),$(filter $(CORE), mpu1_0))
# Enable XDC build for application by providing XDC CFG File per core
XDC_CFG_FILE_$(CORE) = ./am65xx/pciesample_a53.cfg
endif

ifeq ($(CORE),$(filter $(CORE), mcu1_0))
# Enable XDC build for application by providing XDC CFG File per core
XDC_CFG_FILE_$(CORE) = ./am65xx/pciesample_r5.cfg
endif
PACKAGE_SRCS_COMMON = .
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(CFLAGS_PCIE_SMP)

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
