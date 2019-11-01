#*******************************************************************************
#* FILE PURPOSE: Top level makefile for Creating Component Libraries for ARM
#* architectures
#*******************************************************************************
#* FILE NAME: makefile
#*
#* DESCRIPTION: Defines Compiler tools paths, libraries , Build Options 
#*
#*
#*******************************************************************************
#*
# (Mandatory) Specify where various tools are installed.

ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

#Default Do not use the shared object libraries for the test/example applications
export USEDYNAMIC_LIB ?= "no"

#export SOC ?= AM572x
export SOC
export BOOTMODE
export OPPMODE
export SECUREMODE

export LLD_NAME=sbl

# ROOT Directory
export ROOTDIR := ../..

#Check to identify between windows or linux environment
ifeq ($(OS), )
  OS := linux
endif

# INCLUDE Directory
export INCDIR := ../..;$(PDK_INSTALL_PATH);$(ROOTDIR);$(SBL_INC_DIR)

# Common Macros used in make

ifndef RM
export RM = rm -f
endif

ifndef CP
export CP = cp -p
endif

export MKDIR = mkdir -p

ifndef RMDIR
export RMDIR = rm -rf
endif

ifndef SED
export SED = sed
endif

ifndef MAKE
export MAKE = make
endif

# PHONY Targets
.PHONY: all clean spi_flashwriter spi_flashwriter_clean flashwriter flashwriter_clean mmcsd_flashwriter mmcsd_flashwriter_clean example example_clean tools sbl_lib sbl_lib_clean

# all rule
all:
	@$(MAKE) -f ./board/$(BOARD)/build/makefile all

spi_flashwriter:
	@$(MAKE) -f ./tools/flashWriter/spi/src/makefile $@

flashwriter:
	@$(MAKE) -f ./tools/flashWriter/qspi/src/makefile $@

mmcsd_flashwriter:
	@$(MAKE) -f ./tools/flashWriter/mmcsd/src/makefile $@

example:
	@$(MAKE) -f ./example/mpuMulticoreApp/makefile $@
	@$(MAKE) -f ./example/dsp1MulticoreApp/makefile $@
	@$(MAKE) -f ./example/ipu1MulticoreApp/makefile $@
ifeq ($(SOC),$(filter $(SOC), AM572x AM574x))
	@$(MAKE) -f ./example/mpu1MulticoreApp/makefile $@
	@$(MAKE) -f ./example/dsp2MulticoreApp/makefile $@
	@$(MAKE) -f ./example/ipu2MulticoreApp/makefile $@
endif

eveLoader:
ifeq ($(SOC),$(filter $(SOC), AM572x AM574x))
	@$(MAKE) -C  ./example/mpuM4EveLoaderApp $@
endif

tools:
	@$(MAKE) -C tools/btoccs
	@$(MAKE) -C tools/byteswap
	@$(MAKE) -C tools/ccsutil
	@$(MAKE) -C tools/tiImageGen

sbl_lib:
	@$(MAKE) -f ./src/sbl_eve/sbl_lib/src/makefile $@


# Rule to clean sbl binary
clean:
	@$(MAKE) -f ./board/$(BOARD)/build/makefile $@
	@$(RMDIR) $(PDK_INSTALL_PATH)/ti/boot/$(LLD_NAME)/binary/$(BOARD)

spi_flashwriter_clean:
	@$(MAKE) -f ./tools/flashWriter/spi/src/makefile $@

flashwriter_clean:
	@$(MAKE) -f ./tools/flashWriter/qspi/src/makefile $@
	@$(RMDIR) $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/flashWriter/qspi/bin/$(BOARD)

mmcsd_flashwriter_clean:
	@$(MAKE) -f ./tools/flashWriter/mmcsd/src/makefile $@

example_clean:
	@$(MAKE) -f ./example/mpuMulticoreApp/makefile $@
	@$(MAKE) -f ./example/dsp1MulticoreApp/makefile $@
	@$(MAKE) -f ./example/ipu1MulticoreApp/makefile $@
ifeq ($(SOC),$(filter $(SOC), AM572x AM574x))
	@$(MAKE) -f ./example/dsp2MulticoreApp/makefile $@
	@$(MAKE) -f ./example/ipu2MulticoreApp/makefile $@
endif
	@$(RMDIR) $(PDK_INSTALL_PATH)/ti/boot/sbl/binary/sbl_multicore_app/$(BOARD)

sbl_lib_clean:
	@$(MAKE) -f ./src/sbl_eve/sbl_lib/src/makefile $@

eveLoader_clean:
ifeq ($(SOC),$(filter $(SOC), AM572x AM574x))
	@$(MAKE) -C ./example/mpuM4EveLoaderApp $@
endif