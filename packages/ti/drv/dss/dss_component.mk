# File: dss_component.mk
#       This file is component include make file of DSS driver library.
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
ifeq ($(dss_component_make_include), )

drvdss_SOCLIST         = am65xx j721e
drvdss_BOARDLIST       = am65xx_evm j721e_evm
drvdss_am65xx_CORELIST = mpu1_0
drvdss_j721e_CORELIST  = mcu2_1

############################
# DSS package
# List of components included under DSS lib
# The components included here are built and will be part of DSS lib
############################
dss_LIB_LIST =

############################
# DSS app lib package
# List of components included under DSS app lib
# The components included here are built and will be part of DSS app lib
############################
dss_APP_LIB_LIST =

############################
# DSS examples
# List of examples under DSS (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
dss_EXAMPLE_LIST =

#
# DSS Modules
#

# DSS library
dss_COMP_LIST = dss
dss_RELPATH = ti/drv/dss
dss_PATH = $(PDK_DSS_COMP_PATH)
dss_LIBNAME = dss
dss_LIBPATH = $(PDK_DSS_COMP_PATH)/lib
dss_MAKEFILE = -fsrc/makefile
export dss_MAKEFILE
export dss_LIBNAME
export dss_LIBPATH
dss_BOARD_DEPENDENCY = no
dss_CORE_DEPENDENCY = yes
export dss_COMP_LIST
export dss_BOARD_DEPENDENCY
export dss_CORE_DEPENDENCY
dss_PKG_LIST = dss
dss_INCLUDE = $(dss_PATH)
dss_SOCLIST = $(drvdss_SOCLIST)
export dss_SOCLIST
dss_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_$(SOC)_CORELIST
dss_LIB_LIST += dss

dss_app_utils_COMP_LIST = dss_app_utils
dss_app_utils_RELPATH = ti/drv/dss/examples/utils
dss_app_utils_PATH = $(PDK_DSS_COMP_PATH)/examples/utils
dss_app_utils_LIBNAME = dss_app_utils
dss_app_utils_LIBPATH = $(PDK_DSS_COMP_PATH)/lib
dss_app_utils_OBJPATH = $(dss_app_utils_RELPATH)/app_utils_nonos
dss_app_utils_MAKEFILE = -fmakefile
export dss_app_utils_LIBNAME
export dss_app_utils_LIBPATH
export dss_app_utils_MAKEFILE
export dss_app_utils_OBJPATH
dss_app_utils_BOARD_DEPENDENCY = yes
dss_app_utils_CORE_DEPENDENCY = yes
export dss_app_utils_COMP_LIST
export dss_app_utils_BOARD_DEPENDENCY
export dss_app_utils_CORE_DEPENDENCY
dss_app_utils_PKG_LIST = dss_app_utils
dss_app_utils_INCLUDE = $(dss_app_utils_PATH)
dss_app_utils_SOCLIST = $(drvdss_SOCLIST)
export dss_app_utils_SOCLIST
dss_app_utils_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_app_utils_$(SOC)_CORELIST

dss_app_utils_sysbios_COMP_LIST = dss_app_utils_sysbios
dss_app_utils_sysbios_RELPATH = ti/drv/dss/examples/utils
dss_app_utils_sysbios_PATH = $(PDK_DSS_COMP_PATH)/examples/utils
dss_app_utils_sysbios_LIBNAME = dss_app_utils_sysbios
dss_app_utils_sysbios_LIBPATH = $(PDK_DSS_COMP_PATH)/lib
dss_app_utils_sysbios_OBJPATH = $(dss_app_utils_sysbios_RELPATH)/app_utils_sysbios
dss_app_utils_sysbios_MAKEFILE = -fmakefile_sysbios
export dss_app_utils_sysbios_LIBNAME
export dss_app_utils_sysbios_LIBPATH
export dss_app_utils_sysbios_MAKEFILE
export dss_app_utils_sysbios_OBJPATH
dss_app_utils_sysbios_BOARD_DEPENDENCY = yes
dss_app_utils_sysbios_CORE_DEPENDENCY = yes
export dss_app_utils_sysbios_COMP_LIST
export dss_app_utils_sysbios_BOARD_DEPENDENCY
export dss_app_utils_sysbios_CORE_DEPENDENCY
dss_app_utils_sysbios_PKG_LIST = dss_app_utils_sysbios
dss_app_utils_sysbios_INCLUDE = $(dss_app_utils_sysbios_PATH)
dss_app_utils_sysbios_SOCLIST = $(drvdss_SOCLIST)
export dss_app_utils_sysbios_SOCLIST
dss_app_utils_sysbios_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_app_utils_sysbios_$(SOC)_CORELIST

dss_APP_LIB_LIST += dss_app_utils dss_app_utils_sysbios

#
# DSS Examples
#

# DSS colorbar test app
dss_colorbar_testapp_COMP_LIST = dss_colorbar_testapp
dss_colorbar_testapp_RELPATH = ti/drv/dss/examples/dss_colorbar_test
dss_colorbar_testapp_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_colorbar_test
dss_colorbar_testapp_BOARD_DEPENDENCY = yes
dss_colorbar_testapp_CORE_DEPENDENCY = yes
dss_colorbar_testapp_XDC_CONFIGURO = yes
export dss_colorbar_testapp_COMP_LIST
export dss_colorbar_testapp_BOARD_DEPENDENCY
export dss_colorbar_testapp_CORE_DEPENDENCY
export dss_colorbar_testapp_XDC_CONFIGURO
dss_colorbar_testapp_PKG_LIST = dss_colorbar_testapp
dss_colorbar_testapp_INCLUDE = $(dss_colorbar_testapp_PATH)
dss_colorbar_testapp_BOARDLIST = $(drvdss_BOARDLIST)
export dss_colorbar_testapp_BOARDLIST
dss_colorbar_testapp_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_colorbar_testapp_$(SOC)_CORELIST
dss_EXAMPLE_LIST += dss_colorbar_testapp
ifeq ($(SOC),$(filter $(SOC), am65xx j721e))
dss_colorbar_testapp_SBL_APPIMAGEGEN = yes
export dss_colorbar_testapp_SBL_APPIMAGEGEN
endif

# DSS display test app
dss_display_testapp_COMP_LIST = dss_display_testapp
dss_display_testapp_RELPATH = ti/drv/dss/examples/dss_display_test
dss_display_testapp_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_display_test
dss_display_testapp_BOARD_DEPENDENCY = yes
dss_display_testapp_CORE_DEPENDENCY = yes
dss_display_testapp_XDC_CONFIGURO = yes
export dss_display_testapp_COMP_LIST
export dss_display_testapp_BOARD_DEPENDENCY
export dss_display_testapp_CORE_DEPENDENCY
export dss_display_testapp_XDC_CONFIGURO
dss_display_testapp_PKG_LIST = dss_display_testapp
dss_display_testapp_INCLUDE = $(dss_display_testapp_PATH)
dss_display_testapp_BOARDLIST = $(drvdss_BOARDLIST)
export dss_display_testapp_BOARDLIST
dss_display_testapp_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_display_testapp_$(SOC)_CORELIST
dss_EXAMPLE_LIST += dss_display_testapp
ifeq ($(SOC),$(filter $(SOC), am65xx j721e))
dss_display_testapp_SBL_APPIMAGEGEN = yes
export dss_display_testapp_SBL_APPIMAGEGEN
endif

# DSS display baremetal test app
dss_baremetal_display_testapp_COMP_LIST = dss_baremetal_display_testapp
dss_baremetal_display_testapp_RELPATH = ti/drv/dss/examples/dss_display_test
dss_baremetal_display_testapp_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_display_test
dss_baremetal_display_testapp_MAKEFILE = -fmakefile_baremetal
export dss_baremetal_display_testapp_MAKEFILE
dss_baremetal_display_testapp_BOARD_DEPENDENCY = yes
dss_baremetal_display_testapp_CORE_DEPENDENCY = yes
export dss_baremetal_display_testapp_COMP_LIST
export dss_baremetal_display_testapp_BOARD_DEPENDENCY
export dss_baremetal_display_testapp_CORE_DEPENDENCY
dss_baremetal_display_testapp_PKG_LIST = dss_baremetal_display_testapp
dss_baremetal_display_testapp_INCLUDE = $(dss_baremetal_display_testapp_PATH)
dss_baremetal_display_testapp_BOARDLIST = $(drvdss_BOARDLIST)
export dss_baremetal_display_testapp_BOARDLIST
dss_baremetal_display_testapp_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_baremetal_display_testapp_$(SOC)_CORELIST
dss_EXAMPLE_LIST += dss_baremetal_display_testapp
ifeq ($(SOC),$(filter $(SOC), am65xx j721e))
dss_baremetal_display_testapp_SBL_APPIMAGEGEN = yes
export dss_baremetal_display_testapp_SBL_APPIMAGEGEN
endif

export dss_LIB_LIST
export dss_APP_LIB_LIST
export dss_EXAMPLE_LIST
export drvdss_LIB_LIST = $(dss_LIB_LIST)
export drvdss_APP_LIB_LIST = $(dss_APP_LIB_LIST)
export drvdss_EXAMPLE_LIST = $(dss_EXAMPLE_LIST)

DSS_CFLAGS = $(FVID2_CFLAGS)

export DSS_CFLAGS

dss_component_make_include := 1
endif
