# File: udma_ut_component.mk
#       This file is component include make file of UDMA unit test.
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
ifeq ($(udma_ut_component_make_include), )

############################
# udma_ut package
# List of components included under udma_ut
# The components included here are built and will be part of udma_ut lib
############################
udma_ut_LIB_LIST =
udma_ut_EXAMPLE_LIST =

# udma unit test app
export udma_unit_testapp_COMP_LIST = udma_unit_testapp
udma_unit_testapp_RELPATH = ti/drv/udma/unit_test/udma_ut
udma_unit_testapp_PATH = $(PDK_UDMA_COMP_PATH)/unit_test/udma_ut
export udma_unit_testapp_MAKEFILE = -fmakefile
export udma_unit_testapp_BOARD_DEPENDENCY = yes
export udma_unit_testapp_CORE_DEPENDENCY = yes
export udma_unit_testapp_XDC_CONFIGURO = yes
udma_unit_testapp_PKG_LIST = udma_unit_testapp
udma_unit_testapp_INCLUDE = $(udma_unit_testapp_PATH)
export udma_unit_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_unit_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x_1
else
export udma_unit_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0
endif
export udma_unit_testapp_SBL_APPIMAGEGEN = yes
udma_ut_EXAMPLE_LIST += udma_unit_testapp

# udma unit test app - with user input
export udma_user_input_unit_testapp_COMP_LIST = udma_user_input_unit_testapp
udma_user_input_unit_testapp_RELPATH = ti/drv/udma/unit_test/udma_ut
udma_user_input_unit_testapp_PATH = $(PDK_UDMA_COMP_PATH)/unit_test/udma_ut
export udma_user_input_unit_testapp_MAKEFILE = -fmakefile UDMA_UT_MANUAL_ENTRY=yes
export udma_user_input_unit_testapp_BOARD_DEPENDENCY = yes
export udma_user_input_unit_testapp_CORE_DEPENDENCY = yes
export udma_user_input_unit_testapp_XDC_CONFIGURO = yes
udma_user_input_unit_testapp_PKG_LIST = udma_user_input_unit_testapp
udma_user_input_unit_testapp_INCLUDE = $(udma_user_input_unit_testapp_PATH)
export udma_user_input_unit_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_user_input_unit_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x
else
export udma_user_input_unit_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0
endif
export udma_user_input_unit_testapp_SBL_APPIMAGEGEN = yes
udma_ut_EXAMPLE_LIST += udma_user_input_unit_testapp

# udma unit test app
export udma_baremetal_unit_testapp_COMP_LIST = udma_baremetal_unit_testapp
udma_baremetal_unit_testapp_RELPATH = ti/drv/udma/unit_test/udma_ut
udma_baremetal_unit_testapp_PATH = $(PDK_UDMA_COMP_PATH)/unit_test/udma_ut
export udma_baremetal_unit_testapp_MAKEFILE = -fmakefile UDMA_UT_BAREMETAL=yes
export udma_baremetal_unit_testapp_BOARD_DEPENDENCY = yes
export udma_baremetal_unit_testapp_CORE_DEPENDENCY = yes
export udma_baremetal_unit_testapp_XDC_CONFIGURO = no
udma_baremetal_unit_testapp_PKG_LIST = udma_baremetal_unit_testapp
udma_baremetal_unit_testapp_INCLUDE = $(udma_baremetal_unit_testapp_PATH)
export udma_baremetal_unit_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_baremetal_unit_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x_1
else
export udma_baremetal_unit_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0
endif
export udma_baremetal_unit_testapp_SBL_APPIMAGEGEN = yes
udma_ut_EXAMPLE_LIST += udma_baremetal_unit_testapp

# udma unit test app - for dynamic analysis
export udma_dynamic_unit_testapp_COMP_LIST = udma_dynamic_unit_testapp
udma_dynamic_unit_testapp_RELPATH = ti/drv/udma/unit_test/udma_ut
udma_dynamic_unit_testapp_PATH = $(PDK_UDMA_COMP_PATH)/unit_test/udma_ut
export udma_dynamic_unit_testapp_MAKEFILE = -fmakefile UDMA_UT_DYNAMIC_ANALYSIS=yes
export udma_dynamic_unit_testapp_BOARD_DEPENDENCY = yes
export udma_dynamic_unit_testapp_CORE_DEPENDENCY = yes
export udma_dynamic_unit_testapp_XDC_CONFIGURO = no
udma_dynamic_unit_testapp_PKG_LIST = udma_dynamic_unit_testapp
udma_dynamic_unit_testapp_INCLUDE = $(udma_dynamic_unit_testapp_PATH)
export udma_dynamic_unit_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_dynamic_unit_testapp_$(SOC)_CORELIST = mcu1_0 mcu2_1
else
export udma_dynamic_unit_testapp_$(SOC)_CORELIST = mcu1_0
endif
export udma_dynamic_unit_testapp_SBL_APPIMAGEGEN = yes
udma_ut_EXAMPLE_LIST += udma_dynamic_unit_testapp

export udma_ut_LIB_LIST
export udma_ut_EXAMPLE_LIST

udma_ut_component_make_include := 1
endif
