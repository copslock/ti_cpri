# File: sciclient_component.mk
#       This file is component include make file.
# List of variables set in this file and their purpose:
# <mod>_RELPATH        - This is the relative path of the module, typically from
#                        top-level directory of the package
# <mod>_PATH           - This is the absolute path of the module. It derives from
#                        absolute path of the top-level directory (set in env.mk)
#                        and relative path set above
# <mod>_INCLUDE        - This is the path that has interface header files of the
#                        module. This can be multiple directories (space separated)
# <mod>_PKG_LIST       - Names of the modules (and sub-modules) that are a part
#                        part of this module, including itself.
# <mod>_BOARD_DEPENDENCY - "yes": means the code for this module depends on
#                             platform and the compiled obj/lib has to be kept
#                             under <platform> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no platform dependent code and hence
#                             the obj/libs are not kept under <platform> dir.
# <mod>_CORE_DEPENDENCY     - "yes": means the code for this module depends on
#                             core and the compiled obj/lib has to be kept
#                             under <core> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no core dependent code and hence
#                             the obj/libs are not kept under <core> dir.
# <mod>_APP_STAGE_FILES     - List of source files that belongs to the module
#                             <mod>, but that needs to be compiled at application
#                             build stage (in the context of the app). This is
#                             primarily for link time configurations or if the
#                             source file is dependent on options/defines that are
#                             application dependent. This can be left blank or
#                             not defined at all, in which case, it means there
#                             no source files in the module <mod> that are required
#                             to be compiled in the application build stage.
#
ifeq ($(sciclient_component_make_include), )

############################
# sciclient package
# List of components included under sciclient
# The components included here are built and will be part of sciclient lib
############################
sciclient_LIB_LIST = sciclient

drvsciclient_BOARDLIST = am65xx_evm am65xx_idk j721e_sim j721e_evm j7200_evm
drvsciclient_SOCLIST = am65xx j721e j7200
drvsciclient_am65xx_CORELIST = mcu1_0 mcu1_1 mpu1_0
drvsciclient_j721e_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x_1 c7x-hostemu
drvsciclient_j7200_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1
drvsciclient_DISABLE_PARALLEL_MAKE = yes

sciclient_COMP_LIST = sciclient
sciclient_RELPATH = ti/drv/sciclient
sciclient_PATH = $(PDK_SCICLIENT_COMP_PATH)
sciclient_LIBNAME = sciclient
sciclient_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
sciclient_MAKEFILE = -fsrc/makefile

export sciclient_MAKEFILE
export sciclient_LIBNAME
export sciclient_LIBPATH
# Simulator versus Silicon has a different Firmware Image.
sciclient_BOARD_DEPENDENCY = no
ifeq ($(BOARD),$(filter $(BOARD), j721e_ccqt j721e_loki j721e_hostemu))
sciclient_BOARD_DEPENDENCY = yes
endif
sciclient_CORE_DEPENDENCY = yes
export sciclient_COMP_LIST
export sciclient_BOARD_DEPENDENCY
export sciclient_CORE_DEPENDENCY
sciclient_PKG_LIST = sciclient
sciclient_INCLUDE = $(sciclient_PATH)
sciclient_SOCLIST = $(drvsciclient_SOCLIST)
export sciclient_SOCLIST
sciclient_BOARDLIST = $(drvsciclient_BOARDLIST)
export sciclient_BOARDLIST
sciclient_$(SOC)_CORELIST = $(drvsciclient_$(SOC)_CORELIST)
export sciclient_$(SOC)_CORELIST

############################
# sciclient examples
# List of examples under sciclient (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
sciclient_EXAMPLE_LIST =

# SCICLIENT Firmware Boot Application
sciclient_firmware_boot_TestApp_COMP_LIST = sciclient_firmware_boot_TestApp
sciclient_firmware_boot_TestApp_RELPATH = ti/drv/sciclient/examples/sciclient_firmware_boot_TestApp
sciclient_firmware_boot_TestApp_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_firmware_boot_TestApp
sciclient_firmware_boot_TestApp_BOARD_DEPENDENCY = no
sciclient_firmware_boot_TestApp_CORE_DEPENDENCY = yes
export sciclient_firmware_boot_TestApp_COMP_LIST
export sciclient_firmware_boot_TestApp_BOARD_DEPENDENCY
export sciclient_firmware_boot_TestApp_CORE_DEPENDENCY
sciclient_firmware_boot_TestApp_PKG_LIST = sciclient_firmware_boot_TestApp
sciclient_firmware_boot_TestApp_INCLUDE = $(sciclient_firmware_boot_TestApp_PATH)
sciclient_firmware_boot_TestApp_BOARDLIST = am65xx_evm
export sciclient_firmware_boot_TestApp_BOARDLIST
sciclient_firmware_boot_TestApp_$(SOC)_CORELIST = mcu1_0
export sciclient_firmware_boot_TestApp_$(SOC)_CORELIST
sciclient_firmware_boot_TestApp_SBL_APPIMAGEGEN = no
export sciclient_firmware_boot_TestApp_SBL_APPIMAGEGEN
ifeq ($(CORE),mcu1_0)
sciclient_firmware_boot_TestApp_SBL_IMAGEGEN = yes
else
sciclient_firmware_boot_TestApp_SBL_IMAGEGEN = no
endif
export sciclient_firmware_boot_TestApp_SBL_IMAGEGEN
sciclient_EXAMPLE_LIST += sciclient_firmware_boot_TestApp

# SCICLIENT CCS Init Application
sciclient_ccs_init_COMP_LIST = sciclient_ccs_init
sciclient_ccs_init_RELPATH = ti/drv/sciclient/examples/sciclient_ccs_init
sciclient_ccs_init_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_ccs_init
sciclient_ccs_init_BOARD_DEPENDENCY = no
sciclient_ccs_init_CORE_DEPENDENCY = yes
export sciclient_ccs_init_COMP_LIST
export sciclient_ccs_init_BOARD_DEPENDENCY
export sciclient_ccs_init_CORE_DEPENDENCY
sciclient_ccs_init_PKG_LIST = sciclient_ccs_init
sciclient_ccs_init_INCLUDE = $(sciclient_ccs_init_PATH)
sciclient_ccs_init_BOARDLIST = am65xx_evm j721e_sim j721e_evm
export sciclient_ccs_init_BOARDLIST
# This application is only for mcu1_0
sciclient_ccs_init_$(SOC)_CORELIST = mcu1_0
export sciclient_ccs_init_$(SOC)_CORELIST
sciclient_ccs_init_SBL_APPIMAGEGEN = no
export sciclient_ccs_init_SBL_APPIMAGEGEN
sciclient_ccs_init_SBL_IMAGEGEN = yes
export sciclient_ccs_init_SBL_IMAGEGEN
sciclient_EXAMPLE_LIST += sciclient_ccs_init

# SCICLIENT RTOS Application
sciclient_rtos_app_COMP_LIST = sciclient_rtos_app
sciclient_rtos_app_RELPATH = ti/drv/sciclient/examples/sciclient_rtos_app
sciclient_rtos_app_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_rtos_app
sciclient_rtos_app_BOARD_DEPENDENCY = no
sciclient_rtos_app_CORE_DEPENDENCY = yes
sciclient_rtos_app_XDC_CONFIGURO = yes
export sciclient_rtos_app_COMP_LIST
export sciclient_rtos_app_BOARD_DEPENDENCY
export sciclient_rtos_app_CORE_DEPENDENCY
export sciclient_rtos_app_XDC_CONFIGURO
sciclient_rtos_app_PKG_LIST = sciclient_rtos_app
sciclient_rtos_app_INCLUDE = $(sciclient_rtos_app_PATH)
sciclient_rtos_app_BOARDLIST = am65xx_evm j721e_sim j721e_evm
export sciclient_rtos_app_BOARDLIST
sciclient_rtos_app_$(SOC)_CORELIST = $(drvsciclient_$(SOC)_CORELIST)
export sciclient_rtos_app_$(SOC)_CORELIST
sciclient_rtos_app_SBL_APPIMAGEGEN = no
export sciclient_rtos_app_SBL_APPIMAGEGEN
sciclient_rtos_app_SBL_IMAGEGEN = no
export sciclient_rtos_app_SBL_IMAGEGEN
ifeq ($(BUILD_OS_TYPE),tirtos)
sciclient_EXAMPLE_LIST += sciclient_rtos_app
endif

# SCICLIENT UT
sciclient_unit_testapp_COMP_LIST = sciclient_unit_testapp
sciclient_unit_testapp_RELPATH = ti/drv/sciclient/examples/sciclient_unit_testapp
sciclient_unit_testapp_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_unit_testapp
sciclient_unit_testapp_BOARD_DEPENDENCY = no
sciclient_unit_testapp_CORE_DEPENDENCY = yes
sciclient_unit_testapp_XDC_CONFIGURO = yes
export sciclient_unit_testapp_COMP_LIST
export sciclient_unit_testapp_BOARD_DEPENDENCY
export sciclient_unit_testapp_CORE_DEPENDENCY
export sciclient_unit_testapp_XDC_CONFIGURO
sciclient_unit_testapp_PKG_LIST = sciclient_unit_testapp
sciclient_unit_testapp_INCLUDE = $(sciclient_unit_testapp_PATH)
sciclient_unit_testapp_BOARDLIST = am65xx_evm j721e_sim j721e_evm
export sciclient_unit_testapp_BOARDLIST
sciclient_unit_testapp_$(SOC)_CORELIST = $(drvsciclient_$(SOC)_CORELIST)
export sciclient_unit_testapp_$(SOC)_CORELIST
sciclient_unit_testapp_SBL_APPIMAGEGEN = no
export sciclient_unit_testapp_SBL_APPIMAGEGEN
sciclient_unit_testapp_SBL_IMAGEGEN = no
export sciclient_unit_testapp_SBL_IMAGEGEN
ifeq ($(BUILD_OS_TYPE),tirtos)
sciclient_EXAMPLE_LIST += sciclient_unit_testapp
endif

export sciclient_LIB_LIST
export sciclient_EXAMPLE_LIST
export drvsciclient_LIB_LIST = $(sciclient_LIB_LIST)
export drvsciclient_EXAMPLE_LIST  = $(sciclient_EXAMPLE_LIST)

sciclient_component_make_include := 1
endif
