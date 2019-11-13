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
.PHONY: all clean spi_flashwriter spi_flashwriter_clean flashwriter flashwriter_clean mmcsd_flashwriter mmcsd_flashwriter_clean example example_clean tools sbl_lib sbl_lib_clean all_sbl_images all_sbl_images_clean

# all rule
all:
	@$(MAKE) -f ./board/$(BOARD)/build/makefile all

# This target is used to build all configurations for a particular SBL_PLATFORM
all_sbl_images:
ifeq ($(SBL_PLATFORM),$(filter $(SBL_PLATFORM), am65xx j721e))
	# All keystone 3
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl/build all
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl/build all
else
ifeq ($(SBL_PLATFORM), am57xx)
	# am57xx
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=idkAM572x SOC=AM572x BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=idkAM572x SOC=AM572x BOOTMODE=qspi
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=idkAM574x SOC=AM574x BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=idkAM574x SOC=AM574x BOOTMODE=qspi
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=idkAM571x SOC=AM571x BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=idkAM571x SOC=AM571x BOOTMODE=qspi
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=evmAM572x SOC=AM572x BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=evmAM572x SOC=AM572x BOOTMODE=emmc
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter BOARD=idkAM572x SOC=AM572x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter BOARD=idkAM574x SOC=AM574x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter BOARD=idkAM571x SOC=AM571x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl sbl_lib BOARD=idkAM572x SOC=AM572x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl sbl_lib BOARD=idkAM574x SOC=AM574x
else ifeq ($(SBL_PLATFORM), k2g)
	# k2g
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=evmK2G SOC=K2G BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=evmK2G SOC=K2G BOOTMODE=qspi
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter BOARD=evmK2G SOC=K2G
else ifeq ($(SBL_PLATFORM), omapl137)
	# omapl137
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=evmOMAPL137 SOC=OMAPL137 BOOTMODE=spi
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl spi_flashwriter BOARD=evmOMAPL137 SOC=OMAPL137
else ifeq ($(SBL_PLATFORM), omapl138)
	# omapl138
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=lcdkOMAPL138 SOC=OMAPL138 BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl all BOARD=lcdkC6748 SOC=C6748 BOOTMODE=mmcsd
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl mmcsd_flashwriter BOARD=lcdkOMAPL138 SOC=OMAPL138
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl mmcsd_flashwriter BOARD=lcdkC6748 SOC=C6748
endif
endif


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

# This target is used to clean all images for a particular SBL_PLATFORM
all_sbl_images_clean:
ifeq ($(SBL_PLATFORM), am57xx)
	# am57xx
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=idkAM572x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=idkAM574x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=idkAM571x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=evmAM572x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter_clean BOARD=idkAM572x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter_clean BOARD=idkAM574x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter_clean BOARD=idkAM571x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl sbl_lib_clean BOARD=idkAM572x
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl sbl_lib_clean BOARD=idkAM574x
else ifeq ($(SBL_PLATFORM), k2g)
	# k2g
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=evmK2G
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl flashwriter_clean BOARD=evmK2G
else ifeq ($(SBL_PLATFORM), omapl137)
	# omapl137
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=evmOMAPL137
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl spi_flashwriter_clean BOARD=evmOMAPL137
else ifeq ($(SBL_PLATFORM), omapl138)
	# omapl138
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=lcdkOMAPL138
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl clean BOARD=lcdkC6748
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl mmcsd_flashwriter_clean BOARD=lcdkOMAPL138
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl mmcsd_flashwriter_clean BOARD=lcdkC6748
else ifeq ($(SBL_PLATFORM),$(filter $(SBL_PLATFORM), am65xx j721e))
	# All keystone3
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl/build clean
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/boot/sbl/build clean
endif
