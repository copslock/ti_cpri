# Makefile for IOLINK unit test app
include $(PDK_INSTALL_PATH)/ti/build/Rules.make


ifeq ($(IS_BAREMETAL),yes)
BUILD_OS_TYPE = baremetal
CFLAGS_OS_DEFINES =
LNKFLAGS_LOCAL_mpu1_0  += --entry Entry
COMP_LIST_COMMON =  osal_nonos
ifeq ($(ARCH),c66x)
  COMP_LIST_COMMON += csl_intc
else
  COMP_LIST_COMMON += csl_init
endif
EXTERNAL_INTERFACES =
XDC_CFG_FILE_a9host =
else
BUILD_OS_TYPE = tirtos
CFLAGS_OS_DEFINES = -DUSE_BIOS
EXTERNAL_INTERFACES = bios xdc
COMP_LIST_COMMON    = osal_tirtos
XDC_CFG_FILE_a9host = ./$(SOC)/armv7/bios/iolink_arm_idkam437x.cfg
endif

SRCDIR = . src $(IOLINK_STACK_INSTALL_PATH) $(IOLINK_STACK_INSTALL_PATH)/al $(IOLINK_STACK_INSTALL_PATH)/dl $(IOLINK_STACK_INSTALL_PATH)/pl
INCDIR = . src $(IOLINK_STACK_INSTALL_PATH) $(IOLINK_STACK_INSTALL_PATH)/al $(IOLINK_STACK_INSTALL_PATH)/dl $(IOLINK_STACK_INSTALL_PATH)/pl
INCDIR += $(PDK_INSTALL_PATH)/ti/starterware/include/am43xx/ $(PDK_INSTALL_PATH)/ti/starterware/include/hw/

# Common source files across all platforms and cores
SRCS_COMMON +=  main_iolink_test.c ioLink_LEDTask.c ioLink_powerSwitchTask.c ioLink_TLC59281.c stack_api.c ioLink_printTask.c ioLink_autosenTask.c tsc_adc_ss.c board_gpioLed.c
SRCS_COMMON +=  mst_appl.c mst_al.c mst_cm.c  mst_ds.c mst_iol.c mst_sm.c mst_dl_eh.c mst_dl_meh.c mst_dl_mh.c mst_dl_oh.c mst_dl_sh.c mst_pl.c

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk $(EXTERNAL_INTERFACES)


ifeq ($(BUILD_OS_TYPE), tirtos)
IOLINK_OS_TYPE=
else
IOLINK_OS_TYPE=_Baremetal
endif

#Name of the directory created under packages/ti/binary/
APP_NAME = IOLINK$(IOLINK_OS_TYPE)_Stack_TestApp
# Name of the binary if different from the default (APP_NAME)_$(BOARD_$(CORE)_<build_profile>
LOCAL_APP_NAME =  IOLINK$(IOLINK_OS_TYPE)_Stack_$(BOARD)_$(CORE)TestApp

COMP_LIST_COMMON   += iolink pruss uart gpio spi

# List all the components required by the application
COMP_LIST_COMMON  += csl board 

PACKAGE_SRCS_COMMON = .
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(CFLAGS_IOLINK_DMA) $(CFLAGS_OS_DEFINES)

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
