# Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
#
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Macro definitions referenced below
#

SBL_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl
BM_DIR ?= $(TI_SECURE_DEV_PKG)/boot-monitor
SK_DIR ?= $(TI_SECURE_DEV_PKG)/ram_secure_kernel
SECBIN ?= $(TI_SECURE_DEV_PKG)/scripts/secimagecreate.sh
KEYPATH ?= $(TI_SECURE_DEV_PKG)/sec/ks2keys/dev
DEVICE ?= $(SOC)
SBLOBJDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/$(BOOTMODE)/obj
SBLBINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/$(BOOTMODE)/bin
ARMV7LIBDIR ?= ./lib
BINFLAGS = -O binary

#Cross tools
ifdef TOOLCHAIN_PATH_A15
# Support backwards compatibility with KeyStone1 approach
 CC = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)gcc
 AC = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)as
 AR = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)ar
 LD = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)gcc
 BIN = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)objcopy
endif

# INCLUDE Directories
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl
BOARD_DIR = $(PDK_INSTALL_PATH)/ti/board
SBL_INC_DIR = $(SBL_SRC_DIR)/board/src
SBL_LIB_INC_DIR = $(SBL_SRC_DIR)/src/rprc
SBL_SOC_DIR = $(SBL_SRC_DIR)/soc

# Libraries
BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/a15/release/ti.board.aa15fg"
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/k2g/a15/release/ti.drv.uart.aa15fg"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2g/a15/release/ti.drv.i2c.aa15fg"
MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/k2g/a15/release/ti.drv.mmcsd.aa15fg"
FATFS_LIB = "$(PDK_INSTALL_PATH)/ti/fs/fatfs/lib/a15/release/ti.fs.fatfs.aa15fg"
QSPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/a15/release/ti.drv.spi.aa15fg"
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2g/a15/release/ti.csl.aa15fg"
CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2g/a15/release/ti.csl.init.aa15fg"
OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2g/a15/release/ti.osal.aa15fg"

# BOOTMODE specific CFLAGS
ifeq ($(BOOTMODE), mmcsd)
  INTERNALDEFS += -DBOOT_MMCSD
  SBL_MMCSD_DIR = $(SBL_SRC_DIR)/src/mmcsd
  INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_INC_DIR);$(SBL_LIB_INC_DIR);$(SBL_MMCSD_DIR);$(SBL_SOC_DIR);$(SBL_SOC_DIR)/k2g;$(SBL_SOC_DIR)/a15
  PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(MMCSD_LIB) $(FATFS_LIB) $(OSAL_LIB)
endif # ifeq ($(BOOTMODE), mmcsd)

ifeq ($(BOOTMODE), qspi)
  INTERNALDEFS += -DBOOT_QSPI
  SBL_QSPI_DIR = $(SBL_SRC_DIR)/src/qspi
  INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_INC_DIR);$(SBL_LIB_INC_DIR);$(SBL_QSPI_DIR);$(SBL_SOC_DIR);$(SBL_SOC_DIR)/k2g;$(SBL_SOC_DIR)/a15
  PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(QSPI_LIB) $(OSAL_LIB)
endif # ifeq ($(BOOTMODE), qspi)

# Compiler options
INTERNALDEFS += -g -Wall -D__ARMv7 -DSOC_K2G -mcpu=cortex-a15 -mtune=cortex-a15 -marm -mfloat-abi=hard -mfpu=neon -D$(SOC)_BUILD -D$(BOARD)

# Linker options
INTERNALLINKDEFS = -g -gdwarf-3 -gstrict-dwarf -Wall  -mfloat-abi=hard -Wl,--undefined,__aeabi_uidiv -Wl,--undefined,__aeabi_idiv --entry Entry -nostartfiles -static -Wl,--gc-sections -Wl,-T $(SBL_SOC_DIR)/k2g/linker.cmd -Wl,--start-group -l"c" -lrdimon -l"m" -l"nosys" $(BOARD_LIB) $(PERIPHERAL_LIB) -Wl,--end-group $(LDFLAGS)

# Executable using device independent library and device object file
EXE=sbl.out

OBJEXT = o
ASMOBJEXT = ao
SRCDIR = $(SBL_SRC_DIR)/board

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(SRCDIR)/$(BOARD):$(SBL_SRC_DIR)/board/src:$(SBL_SRC_DIR)/soc/k2g:$(SBL_SRC_DIR)/soc/a15:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/k2g:$(SBL_SRC_DIR)/src/$(BOOTMODE):$(SBL_SRC_DIR)/src/rprc:$(PDK_INSTALL_PATH)/ti/drv/mmcsd/soc/k2g:$(PDK_INSTALL_PATH)/ti/drv/spi/soc/k2g

#List the Source Files
SRC_C = \
	sbl_main.c \
	sbl_rprc.c \
	sbl_slave_boot.c \
	sbl_a15.c

SRC_DRV = \
    sbl_soc.c


ifeq ($(BOOTMODE), qspi)
    SRC_C += sbl_qspi.c
    SRC_DRV += SPI_soc.c
endif

ifeq ($(BOOTMODE), mmcsd)
    SRC_C += sbl_mmcsd.c
    SRC_DRV += MMCSD_soc.c
endif


# enable JTAG by default now. Can force the build to lock JTAG with ENABLE_JTAG=no. 
# We actually enable JTAG only when the signing key matches the development/dummy keys
# to prevent accidentally unlocking JTAG. The check is done in Boot Monitor makefile
ENABLE_JTAG ?= yes

export ENABLE_JTAG

ifeq ($(SECUREMODE), yes)
  SRC_C += sbl_sec.c
  INTERNALDEFS += -DSECURE_BOOT
  BM_SEC_DEP = signed_sec_bm.h
  SK_SEC_DEP = bootMainGem.h
  DSP_SEC_SRV_DEP = secserver_self.h
  MLO_NAME = mlo_signed.bin
else
  BM_SEC_DEP = 
  SK_SEC_DEP =
  DSP_SEC_SRV_DEP =
  MLO_NAME = MLO
endif

#Common entry object
ENTRY_SRC = sbl_init.S

# BOOTMODE specific CFLAGS
ifeq ($(BOOTMODE), mmcsd)
  INTERNALDEFS += -DBOOT_MMCSD
  SBL_MMCSD_DIR =  $(SBL_SRC_DIR)/src/mmcsd
  INCS += -I$(SBL_MMCSD_DIR)
endif # ifeq ($(BOOTMODE), mmcsd)

ifeq ($(BOOTMODE), qspi)
  INTERNALDEFS += -DBOOT_QSPI
  SBL_QSPI_DIR =  $(SBL_SRC_DIR)/src/qspi
  INCS += -I$(SBL_QSPI_DIR)
endif # ifeq ($(BOOTMODE), qspi)

# FLAGS for the SourceFiles
CFLAGS += -Wall
SRC_CFLAGS = -I. $(CFLAGS) -g -gdwarf-3 -gstrict-dwarf -Wall 

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.S, $(SBLOBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_DRV))

all:$(SBLBINDIR)/$(EXE) $(MLO_NAME)

MLO: $(SBLBINDIR)/$(EXE)
	@$(TOOLCHAIN_PATH_M4)/bin/armhex $(SBLBINDIR)/$(EXE) $(SRCDIR)/$(BOARD)/build/sbl.rmd
	@mv sbl.hex $(SBLBINDIR)
	@$(SBL_SRC_DIR)/tools/btoccs/b2ccs $(SBLBINDIR)/sbl.hex $(SBLBINDIR)/sbl.ccs
	@$(SBL_SRC_DIR)/tools/ccsutil/ccsAddGphdr -infile $(SBLBINDIR)/sbl.ccs -outfile $(SBLBINDIR)/sbl.gphdr -headerEndian BE
	@$(SBL_SRC_DIR)/tools/ccsutil/ccsAddGptlr -infile $(SBLBINDIR)/sbl.gphdr -outfile $(SBLBINDIR)/sbl.gptlr
	@$(SBL_SRC_DIR)/tools/byteswap/byteswapccs $(SBLBINDIR)/sbl.gptlr $(SBLBINDIR)/MLO.ccs
	@$(SBL_SRC_DIR)/tools/btoccs/ccs2bin -swap $(SBLBINDIR)/MLO.ccs $(SBLBINDIR)/MLO

mlo_signed.bin:
	@$(TOOLCHAIN_PATH_M4)/bin/armhex $(SBLBINDIR)/$(EXE) $(SRCDIR)/$(BOARD)/build/sbl_hs.rmd
	@mv sbl_hs.hex $(SBLBINDIR)
	@$(SBL_SRC_DIR)/tools/btoccs/b2ccs $(SBLBINDIR)/sbl_hs.hex $(SBLBINDIR)/sbl_hs.ccs
	$(SBL_SRC_DIR)/tools/btoccs/ccs2bin $(SBLBINDIR)/sbl_hs.ccs $(SBLBINDIR)/sbl.bin
	$(SECBIN) -i  $(SBLBINDIR)/sbl.bin -r $(KEYPATH)/privkey.pem -a $(KEYPATH)/aeskey.txt -d $(SOC) -o  $(SBLBINDIR)/$@ -l 0x0c0b0000
	@cp  $(SBLBINDIR)/$@ $(SBLBINDIR)/MLO


$(SBLBINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(INTERNALLINKDEFS) -Wl,-Map=$(SBLBINDIR)/sbl.map -o $@

$(SBLOBJDIR)/%.$(OBJEXT): %.c $(SBLOBJDIR)/.created  $(BM_SEC_DEP) $(SK_SEC_DEP) $(DSP_SEC_SRV_DEP)
	@echo compiling $< ...
	$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(SBLOBJDIR)/%.$(ASMOBJEXT): %.S $(SBLOBJDIR)/.created
	@echo compiling $< ...
	$(CC) -c $(INTERNALDEFS) $(INCS)  $< -o $@

#make sure BM has been compiled and BM's build environment variables have been setup
# particularly:  $SECBIN, $BM_DIR, $KEY_PATH, $SOC 
$(BM_SEC_DEP): $(BM_DIR)/Docs/signed_sec_bm.h
	cp $< $(SBL_SOC_DIR)/signed_sec_bm.h

$(SK_SEC_DEP): $(SK_DIR)/runSk/bootMainGem.h
	cp $< $(SBL_SOC_DIR)/$@

$(DSP_SEC_SRV_DEP): $(TI_SECURE_DEV_PKG)/secure_server/src/secserver_self.h
	cp $< $(SBL_SOC_DIR)/$@

$(BM_DIR)/Docs/signed_sec_bm.h: .FORCE

.PHONY: .FORCE

.FORCE:
	$(BM_DIR)/../scripts/build_secdev.sh

$(SBLOBJDIR)/.created:
	@mkdir -p $(SBLOBJDIR)
	@touch $(SBLOBJDIR)/.created

$(SBLBINDIR)/.created:
	@mkdir -p $(SBLBINDIR)
	@touch $(SBLBINDIR)/.created

clean:
	@rm -f $(SBLBINDIR)/$(EXE)
	@rm -f $(SBL_SOC_DIR)/signed_sec_bm.h $(SBL_SOC_DIR)/bootMainGem.h $(SBL_SOC_DIR)/secserver_self.h
	@rm -f $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created

