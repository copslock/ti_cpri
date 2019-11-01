# File: cal_component.mk
#       This file is component include make file of CAL driver library.
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
ifeq ($(cal_component_make_include), )

drvcal_SOCLIST          = am65xx
drvcal_BOARDLIST        = am65xx_evm
drvcal_am65xx_CORELIST  = mpu1_0

############################
# cal package
# List of components included under cal lib
# The components included here are built and will be part of cal lib
############################
cal_LIB_LIST = cal

############################
# cal examples
# List of examples under cal (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
cal_EXAMPLE_LIST =

#
# CAL Modules
#

# CAL library
cal_COMP_LIST = cal
cal_RELPATH = ti/drv/cal
cal_PATH = $(PDK_CAL_COMP_PATH)
cal_LIBNAME = cal
cal_LIBPATH = $(PDK_CAL_COMP_PATH)/lib
cal_MAKEFILE = -fsrc/makefile
export cal_MAKEFILE
export cal_LIBNAME
export cal_LIBPATH
cal_BOARD_DEPENDENCY = no
cal_CORE_DEPENDENCY = yes
export cal_COMP_LIST
export cal_BOARD_DEPENDENCY
export cal_CORE_DEPENDENCY
cal_PKG_LIST = cal
cal_INCLUDE = $(cal_PATH)
cal_SOCLIST = $(drvcal_SOCLIST)
export cal_SOCLIST
cal_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_$(SOC)_CORELIST

cal_app_utils_COMP_LIST = cal_app_utils
cal_app_utils_RELPATH = ti/drv/cal/examples/utils
cal_app_utils_PATH = $(PDK_CAL_COMP_PATH)/examples/utils
cal_app_utils_LIBNAME = cal_app_utils
cal_app_utils_LIBPATH = $(PDK_CAL_COMP_PATH)/lib
cal_app_utils_MAKEFILE = -fmakefile
export cal_app_utils_LIBNAME
export cal_app_utils_LIBPATH
export cal_app_utils_MAKEFILE
cal_app_utils_OBJPATH = $(cal_app_utils_RELPATH)
export cal_app_utils_OBJPATH
cal_app_utils_BOARD_DEPENDENCY = yes
cal_app_utils_CORE_DEPENDENCY = yes
export cal_app_utils_COMP_LIST
export cal_app_utils_BOARD_DEPENDENCY
export cal_app_utils_CORE_DEPENDENCY
cal_app_utils_PKG_LIST = cal_app_utils
cal_app_utils_INCLUDE = $(cal_app_utils_PATH)
cal_app_utils_SOCLIST = $(drvcal_SOCLIST)
export cal_app_utils_SOCLIST
cal_app_utils_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_app_utils_$(SOC)_CORELIST
cal_LIB_LIST += cal_app_utils

cal_app_utils_baremetal_COMP_LIST = cal_app_utils_baremetal
cal_app_utils_baremetal_RELPATH = ti/drv/cal/examples/utils
cal_app_utils_baremetal_PATH = $(PDK_CAL_COMP_PATH)/examples/utils
cal_app_utils_baremetal_LIBNAME = cal_app_utils_baremetal
cal_app_utils_baremetal_LIBPATH = $(PDK_CAL_COMP_PATH)/lib
cal_app_utils_baremetal_MAKEFILE = -fmakefile_baremetal
export cal_app_utils_baremetal_LIBNAME
export cal_app_utils_baremetal_LIBPATH
export cal_app_utils_baremetal_MAKEFILE
cal_app_utils_baremetal_OBJPATH = $(cal_app_utils_baremetal_RELPATH)_baremetal
export cal_app_utils_baremetal_OBJPATH
cal_app_utils_baremetal_BOARD_DEPENDENCY = yes
cal_app_utils_baremetal_CORE_DEPENDENCY = yes
export cal_app_utils_baremetal_COMP_LIST
export cal_app_utils_baremetal_BOARD_DEPENDENCY
export cal_app_utils_baremetal_CORE_DEPENDENCY
cal_app_utils_baremetal_PKG_LIST = cal_app_utils_baremetal
cal_app_utils_baremetal_INCLUDE = $(cal_app_utils_baremetal_PATH)
cal_app_utils_baremetal_SOCLIST = $(drvcal_SOCLIST)
export cal_app_utils_baremetal_SOCLIST
cal_app_utils_baremetal_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_app_utils_baremetal_$(SOC)_CORELIST
cal_LIB_LIST += cal_app_utils_baremetal

#
# CAL Examples
#

cal_capture_testapp_COMP_LIST = cal_capture_testapp
cal_capture_testapp_RELPATH = ti/drv/cal/examples/cal_capture_test
cal_capture_testapp_PATH = $(PDK_CAL_COMP_PATH)/examples/cal_capture_test
cal_capture_testapp_MAKEFILE = -fmakefile
cal_capture_testapp_BOARD_DEPENDENCY = yes
cal_capture_testapp_CORE_DEPENDENCY = yes
cal_capture_testapp_XDC_CONFIGURO = yes
export cal_capture_testapp_COMP_LIST
export cal_capture_testapp_BOARD_DEPENDENCY
export cal_capture_testapp_CORE_DEPENDENCY
export cal_capture_testapp_XDC_CONFIGURO
cal_capture_testapp_PKG_LIST = cal_capture_testapp
cal_capture_testapp_INCLUDE = $(cal_capture_testapp_PATH)
cal_capture_testapp_BOARDLIST = am65xx_evm
export cal_capture_testapp_BOARDLIST
cal_capture_testapp_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_capture_testapp_$(SOC)_CORELIST
cal_EXAMPLE_LIST += cal_capture_testapp

cal_baremetal_capture_testapp_COMP_LIST = cal_baremetal_capture_testapp
cal_baremetal_capture_testapp_RELPATH = ti/drv/cal/examples/cal_capture_test
cal_baremetal_capture_testapp_PATH = $(PDK_CAL_COMP_PATH)/examples/cal_capture_test
cal_baremetal_capture_testapp_MAKEFILE = -fmakefile_baremetal
cal_baremetal_capture_testapp_BOARD_DEPENDENCY = yes
cal_baremetal_capture_testapp_CORE_DEPENDENCY = yes
export cal_baremetal_capture_testapp_COMP_LIST
export cal_baremetal_capture_testapp_BOARD_DEPENDENCY
export cal_baremetal_capture_testapp_CORE_DEPENDENCY
cal_baremetal_capture_testapp_PKG_LIST = cal_baremetal_capture_testapp
cal_baremetal_capture_testapp_INCLUDE = $(cal_baremetal_capture_testapp_PATH)
cal_baremetal_capture_testapp_BOARDLIST = am65xx_evm
export cal_baremetal_capture_testapp_BOARDLIST
cal_baremetal_capture_testapp_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_baremetal_capture_testapp_$(SOC)_CORELIST
cal_EXAMPLE_LIST += cal_baremetal_capture_testapp

cal_loopback_testapp_COMP_LIST = cal_loopback_testapp
cal_loopback_testapp_RELPATH = ti/drv/cal/examples/cal_loopback_test
cal_loopback_testapp_PATH = $(PDK_CAL_COMP_PATH)/examples/cal_loopback_test
cal_loopback_testapp_MAKEFILE = -fmakefile
cal_loopback_testapp_BOARD_DEPENDENCY = yes
cal_loopback_testapp_CORE_DEPENDENCY = yes
cal_loopback_testapp_XDC_CONFIGURO = yes
export cal_loopback_testapp_COMP_LIST
export cal_loopback_testapp_BOARD_DEPENDENCY
export cal_loopback_testapp_CORE_DEPENDENCY
export cal_loopback_testapp_XDC_CONFIGURO
cal_loopback_testapp_PKG_LIST = cal_loopback_testapp
cal_loopback_testapp_INCLUDE = $(cal_loopback_testapp_PATH)
cal_loopback_testapp_BOARDLIST = $(drvcal_BOARDLIST)
export cal_loopback_testapp_BOARDLIST
cal_loopback_testapp_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_loopback_testapp_$(SOC)_CORELIST
cal_EXAMPLE_LIST += cal_loopback_testapp

cal_baremetal_loopback_testapp_COMP_LIST = cal_baremetal_loopback_testapp
cal_baremetal_loopback_testapp_RELPATH = ti/drv/cal/examples/cal_loopback_test
cal_baremetal_loopback_testapp_PATH = $(PDK_CAL_COMP_PATH)/examples/cal_loopback_test
cal_baremetal_loopback_testapp_MAKEFILE = -fmakefile_baremetal
cal_baremetal_loopback_testapp_BOARD_DEPENDENCY = yes
cal_baremetal_loopback_testapp_CORE_DEPENDENCY = yes
export cal_baremetal_loopback_testapp_COMP_LIST
export cal_baremetal_loopback_testapp_BOARD_DEPENDENCY
export cal_baremetal_loopback_testapp_CORE_DEPENDENCY
cal_baremetal_loopback_testapp_PKG_LIST = cal_baremetal_loopback_testapp
cal_baremetal_loopback_testapp_INCLUDE = $(cal_baremetal_loopback_testapp_PATH)
cal_baremetal_loopback_testapp_BOARDLIST = $(drvcal_BOARDLIST)
export cal_baremetal_loopback_testapp_BOARDLIST
cal_baremetal_loopback_testapp_$(SOC)_CORELIST = $(drvcal_$(SOC)_CORELIST)
export cal_baremetal_loopback_testapp_$(SOC)_CORELIST
cal_EXAMPLE_LIST += cal_baremetal_loopback_testapp

export cal_LIB_LIST
export cal_EXAMPLE_LIST
export drvcal_LIB_LIST = $(cal_LIB_LIST)
export drvcal_EXAMPLE_LIST = $(cal_EXAMPLE_LIST)

CAL_CFLAGS =
CAL_CFLAGS += $(FVID2_CFLAGS)

export CAL_CFLAGS

cal_component_make_include := 1
endif
