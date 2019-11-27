# File: udma_component.mk
#       This file is component include make file of UDMA driver library.
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
ifeq ($(udma_component_make_include), )

drvudma_SOCLIST         = am65xx j721e j7200
drvudma_BOARDLIST       = am65xx_evm am65xx_idk j721e_sim j721e_evm
drvudma_dru_BOARDLIST   = am65xx_evm am65xx_idk j721e_evm
drvudma_am65xx_CORELIST = mpu1_0 mcu1_0 mcu1_1
drvudma_j721e_CORELIST  = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x_1 c7x-hostemu
drvudma_j7200_CORELIST  = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1

############################
# udma package
# List of components included under udma lib
# The components included here are built and will be part of udma lib
############################
udma_LIB_LIST =

############################
# udma examples
# List of examples under udma (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
udma_EXAMPLE_LIST =

#
# UDMA Modules
#

# UDMA library
udma_COMP_LIST = udma
udma_RELPATH = ti/drv/udma
udma_PATH = $(PDK_UDMA_COMP_PATH)
export udma_LIBNAME = udma
export udma_LIBPATH = $(PDK_UDMA_COMP_PATH)/lib
export udma_MAKEFILE = -fsrc/makefile
export udma_BOARD_DEPENDENCY = no
ifeq ($(BOARD),$(filter $(BOARD), j721e_ccqt j721e_loki j721e_hostemu))
export udma_BOARD_DEPENDENCY = yes
endif
export udma_CORE_DEPENDENCY = yes
udma_PKG_LIST = udma
udma_INCLUDE = $(udma_PATH)
export udma_SOCLIST = $(drvudma_SOCLIST)
export udma_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
udma_LIB_LIST += udma

#
# DMA Utils
#
# DMA Utils library
export dmautils_COMP_LIST = dmautils
dmautils_RELPATH = ti/drv/udma/dmautils
dmautils_PATH = $(PDK_UDMA_COMP_PATH)/dmautils
export dmautils_LIBNAME = dmautils
export dmautils_LIBPATH = $(PDK_UDMA_COMP_PATH)/lib
export dmautils_MAKEFILE = -fmakefile
export dmautils_BOARD_DEPENDENCY = no
ifeq ($(BOARD),$(filter $(BOARD), j721e_ccqt j721e_loki j721e_hostemu))
export dmautils_BOARD_DEPENDENCY = yes
endif
export dmautils_CORE_DEPENDENCY = yes
dmautils_PKG_LIST = dmautils
dmautils_INCLUDE = $(dmautils_PATH)
export dmautils_SOCLIST = $(drvudma_SOCLIST)
export dmautils_$(SOC)_CORELIST = c7x_1 c7x-hostemu
udma_LIB_LIST += dmautils

# UDMA example library
export udma_apputils_COMP_LIST = udma_apputils
udma_apputils_RELPATH = ti/drv/udma/examples/udma_apputils
udma_apputils_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_apputils
export udma_apputils_LIBNAME = udma_apputils
export udma_apputils_LIBPATH = $(PDK_UDMA_COMP_PATH)/lib
export udma_apputils_MAKEFILE = -fmakefile
export udma_apputils_BOARD_DEPENDENCY = no
export udma_apputils_CORE_DEPENDENCY = yes
udma_apputils_PKG_LIST = udma_apputils
udma_apputils_INCLUDE = $(udma_apputils_PATH)
export udma_apputils_SOCLIST = $(drvudma_SOCLIST)
export udma_apputils_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
udma_LIB_LIST += udma_apputils

#
# UDMA Examples
#

#
# DMA Utils tests
#

# DMA Utils test app
export dmautils_baremetal_autoincrement_testapp_COMP_LIST = dmautils_baremetal_autoincrement_testapp
dmautils_baremetal_autoincrement_testapp_RELPATH = ti/drv/udma/dmautils/test/dmautils_autoincrement_test
dmautils_baremetal_autoincrement_testapp_PATH = $(PDK_UDMA_COMP_PATH)/dmautils/test/dmautils_autoincrement_test
export dmautils_baremetal_autoincrement_testapp_BOARD_DEPENDENCY = yes
export dmautils_baremetal_autoincrement_testapp_CORE_DEPENDENCY = yes
dmautils_baremetal_autoincrement_testapp_PKG_LIST = dmautils_baremetal_autoincrement_testapp
dmautils_baremetal_autoincrement_testapp_INCLUDE = $(dmautils_baremetal_autoincrement_testapp_PATH)
export dmautils_baremetal_autoincrement_testapp_BOARDLIST = j721e_hostemu j721e_sim j721e_loki j721e_ccqt j721e_evm
export dmautils_baremetal_autoincrement_testapp_$(SOC)_CORELIST = c7x_1 c7x-hostemu
udma_EXAMPLE_LIST += dmautils_baremetal_autoincrement_testapp

export dmautils_baremetal_autoinc_1d2d3d_testapp_COMP_LIST = dmautils_baremetal_autoinc_1d2d3d_testapp
dmautils_baremetal_autoinc_1d2d3d_testapp_RELPATH = ti/drv/udma/dmautils/test/dmautils_autoinc_1d2d3d_test
dmautils_baremetal_autoinc_1d2d3d_testapp_PATH = $(PDK_UDMA_COMP_PATH)/dmautils/test/dmautils_autoinc_1d2d3d_test
export dmautils_baremetal_autoinc_1d2d3d_testapp_BOARD_DEPENDENCY = yes
export dmautils_baremetal_autoinc_1d2d3d_testapp_CORE_DEPENDENCY = yes
dmautils_baremetal_autoinc_1d2d3d_testapp_PKG_LIST = dmautils_baremetal_autoinc_1d2d3d_testapp
dmautils_baremetal_autoinc_1d2d3d_testapp_INCLUDE = $(dmautils_baremetal_autoinc_1d2d3d_testapp_PATH)
export dmautils_baremetal_autoinc_1d2d3d_testapp_BOARDLIST = j721e_hostemu j721e_sim j721e_loki j721e_ccqt j721e_evm
export dmautils_baremetal_autoinc_1d2d3d_testapp_$(SOC)_CORELIST = c7x_1 c7x-hostemu
udma_EXAMPLE_LIST += dmautils_baremetal_autoinc_1d2d3d_testapp

export dmautils_baremetal_autoinc_circular_testapp_COMP_LIST = dmautils_baremetal_autoinc_circular_testapp
dmautils_baremetal_autoinc_circular_testapp_RELPATH = ti/drv/udma/dmautils/test/dmautils_autoinc_circular_test
dmautils_baremetal_autoinc_circular_testapp_PATH = $(PDK_UDMA_COMP_PATH)/dmautils/test/dmautils_autoinc_circular_test
export dmautils_baremetal_autoinc_circular_testapp_BOARD_DEPENDENCY = yes
export dmautils_baremetal_autoinc_circular_testapp_CORE_DEPENDENCY = yes
dmautils_baremetal_autoinc_circular_testapp_PKG_LIST = dmautils_baremetal_autoinc_circular_testapp
dmautils_baremetal_autoinc_circular_testapp_INCLUDE = $(dmautils_baremetal_autoinc_circular_testapp_PATH)
export dmautils_baremetal_autoinc_circular_testapp_BOARDLIST = j721e_hostemu j721e_sim j721e_loki j721e_ccqt j721e_evm
export dmautils_baremetal_autoinc_circular_testapp_$(SOC)_CORELIST = c7x_1 c7x-hostemu
udma_EXAMPLE_LIST += dmautils_baremetal_autoinc_circular_testapp

# UDMA memcpy test app
export udma_memcpy_testapp_COMP_LIST = udma_memcpy_testapp
udma_memcpy_testapp_RELPATH = ti/drv/udma/examples/udma_memcpy_test
udma_memcpy_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_memcpy_test
export udma_memcpy_testapp_BOARD_DEPENDENCY = yes
export udma_memcpy_testapp_CORE_DEPENDENCY = yes
export udma_memcpy_testapp_XDC_CONFIGURO = yes
udma_memcpy_testapp_PKG_LIST = udma_memcpy_testapp
udma_memcpy_testapp_INCLUDE = $(udma_memcpy_testapp_PATH)
export udma_memcpy_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_memcpy_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x_1
else
export udma_memcpy_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_memcpy_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_memcpy_testapp

# UDMA memcpy test app
export udma_memcpy_smp_testapp_COMP_LIST = udma_memcpy_smp_testapp
udma_memcpy_smp_testapp_RELPATH = ti/drv/udma/examples/udma_memcpy_test
udma_memcpy_smp_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_memcpy_test
udma_memcpy_smp_testapp_MAKEFILE = -f makefile SMP=enable
export udma_memcpy_smp_testapp_BOARD_DEPENDENCY = yes
export udma_memcpy_smp_testapp_CORE_DEPENDENCY = yes
export udma_memcpy_smp_testapp_XDC_CONFIGURO = yes
udma_memcpy_smp_testapp_PKG_LIST = udma_memcpy_smp_testapp
udma_memcpy_smp_testapp_INCLUDE = $(udma_memcpy_smp_testapp_PATH)
export udma_memcpy_smp_testapp_BOARDLIST = am65xx_idk
export udma_memcpy_smp_testapp_$(SOC)_CORELIST = mpu1_0
export udma_memcpy_smp_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_memcpy_smp_testapp

# UDMA memcpy baremetal test app
export udma_baremetal_memcpy_testapp_COMP_LIST = udma_baremetal_memcpy_testapp
udma_baremetal_memcpy_testapp_RELPATH = ti/drv/udma/examples/udma_memcpy_test
udma_baremetal_memcpy_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_memcpy_test
export udma_baremetal_memcpy_testapp_MAKEFILE = -fmakefile_baremetal
export udma_baremetal_memcpy_testapp_BOARD_DEPENDENCY = yes
export udma_baremetal_memcpy_testapp_CORE_DEPENDENCY = yes
udma_baremetal_memcpy_testapp_PKG_LIST = udma_baremetal_memcpy_testapp
udma_baremetal_memcpy_testapp_INCLUDE = $(udma_baremetal_memcpy_testapp_PATH)
export udma_baremetal_memcpy_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_baremetal_memcpy_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0 mcu2_0
else
export udma_baremetal_memcpy_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_baremetal_memcpy_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_baremetal_memcpy_testapp

# UDMA chaining test app
export udma_chaining_testapp_COMP_LIST = udma_chaining_testapp
udma_chaining_testapp_RELPATH = ti/drv/udma/examples/udma_chaining_test
udma_chaining_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_chaining_test
export udma_chaining_testapp_BOARD_DEPENDENCY = yes
export udma_chaining_testapp_CORE_DEPENDENCY = yes
export udma_chaining_testapp_XDC_CONFIGURO = yes
udma_chaining_testapp_PKG_LIST = udma_chaining_testapp
udma_chaining_testapp_INCLUDE = $(udma_chaining_testapp_PATH)
export udma_chaining_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_chaining_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0
else
export udma_chaining_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_chaining_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_chaining_testapp

# UDMA DRU test app
export udma_dru_testapp_COMP_LIST = udma_dru_testapp
udma_dru_testapp_RELPATH = ti/drv/udma/examples/udma_dru_test
udma_dru_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_dru_test
export udma_dru_testapp_BOARD_DEPENDENCY = yes
export udma_dru_testapp_CORE_DEPENDENCY = yes
export udma_dru_testapp_XDC_CONFIGURO = yes
udma_dru_testapp_PKG_LIST = udma_dru_testapp
udma_dru_testapp_INCLUDE = $(udma_dru_testapp_PATH)
export udma_dru_testapp_BOARDLIST = $(drvudma_dru_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_dru_testapp_$(SOC)_CORELIST = mcu2_1 c66xdsp_1 c66xdsp_2 c7x_1
else
export udma_dru_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_dru_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_dru_testapp

# UDMA DRU Direct TR test app
export udma_dru_direct_tr_testapp_COMP_LIST = udma_dru_direct_tr_testapp
udma_dru_direct_tr_testapp_RELPATH = ti/drv/udma/examples/udma_dru_direct_tr_test
udma_dru_direct_tr_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_dru_direct_tr_test
export udma_dru_direct_tr_testapp_BOARD_DEPENDENCY = yes
export udma_dru_direct_tr_testapp_CORE_DEPENDENCY = yes
export udma_dru_direct_tr_testapp_XDC_CONFIGURO = yes
udma_dru_direct_tr_testapp_PKG_LIST = udma_dru_direct_tr_testapp
udma_dru_direct_tr_testapp_INCLUDE = $(udma_dru_direct_tr_testapp_PATH)
export udma_dru_direct_tr_testapp_BOARDLIST = $(drvudma_dru_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_dru_direct_tr_testapp_$(SOC)_CORELIST = mcu2_1 c66xdsp_1 c7x_1
else
export udma_dru_direct_tr_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_dru_direct_tr_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_dru_direct_tr_testapp

# UDMA CRC test app
export udma_crc_testapp_COMP_LIST = udma_crc_testapp
udma_crc_testapp_RELPATH = ti/drv/udma/examples/udma_crc_test
udma_crc_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_crc_test
export udma_crc_testapp_BOARD_DEPENDENCY = yes
export udma_crc_testapp_CORE_DEPENDENCY = yes
export udma_crc_testapp_XDC_CONFIGURO = yes
udma_crc_testapp_PKG_LIST = udma_crc_testapp
udma_crc_testapp_INCLUDE = $(udma_crc_testapp_PATH)
export udma_crc_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_crc_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0
else
export udma_crc_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_crc_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_crc_testapp

# UDMA ADC test app
export udma_adc_testapp_COMP_LIST = udma_adc_testapp
udma_adc_testapp_RELPATH = ti/drv/udma/examples/udma_adc_test
udma_adc_testapp_PATH = $(PDK_UDMA_COMP_PATH)/examples/udma_adc_test
export udma_adc_testapp_BOARD_DEPENDENCY = yes
export udma_adc_testapp_CORE_DEPENDENCY = yes
export udma_adc_testapp_XDC_CONFIGURO = yes
udma_adc_testapp_PKG_LIST = udma_adc_testapp
udma_adc_testapp_INCLUDE = $(udma_adc_testapp_PATH)
export udma_adc_testapp_BOARDLIST = $(drvudma_BOARDLIST)
ifeq ($(SOC),$(filter $(SOC), j721e))
export udma_adc_testapp_$(SOC)_CORELIST = mpu1_0 mcu1_0
else
export udma_adc_testapp_$(SOC)_CORELIST = $(drvudma_$(SOC)_CORELIST)
endif
export udma_adc_testapp_SBL_APPIMAGEGEN = yes
udma_EXAMPLE_LIST += udma_adc_testapp

-include $(PDK_UDMA_COMP_PATH)/unit_test/udma_ut_component.mk
ifneq ($(udma_ut_LIB_LIST),)
  udma_LIB_LIST += $(udma_ut_LIB_LIST)
endif
ifneq ($(udma_ut_EXAMPLE_LIST),)
  udma_EXAMPLE_LIST += $(udma_ut_EXAMPLE_LIST)
endif

export udma_LIB_LIST
export udma_EXAMPLE_LIST
export drvudma_LIB_LIST = $(udma_LIB_LIST)
export drvudma_EXAMPLE_LIST = $(udma_EXAMPLE_LIST)

UDMA_CFLAGS =

# Enable asserts and prints
UDMA_CFLAGS += -DUDMA_CFG_ASSERT_ENABLE
UDMA_CFLAGS += -DUDMA_CFG_PRINT_ENABLE

export UDMA_CFLAGS

udma_component_make_include := 1
endif
