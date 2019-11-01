# Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
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
SBLOBJDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/$(BOOTMODE)/obj
SBLBINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/$(BOOTMODE)/bin
ARMV7LIBDIR ?= ./lib
BINFLAGS = -O binary

#Cross tools
CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x

# INCLUDE Directories
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl
BOARD_DIR = $(PDK_INSTALL_PATH)/ti/board
SBL_INC_DIR = $(SBL_SRC_DIR)/board/src
SBL_LIB_INC_DIR = $(SBL_SRC_DIR)/src/rprc
SBL_SOC_DIR = $(SBL_SRC_DIR)/soc

# Libraries
BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/lcdkOMAPL138/c674/release/ti.board.ae674"
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/omapl138/c674/release/ti.drv.uart.ae674"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/omapl138/c674/release/ti.drv.i2c.ae674"
SPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/omapl138/c674/release/ti.drv.spi.ae674"
MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/omapl138/c674/release/ti.drv.mmcsd.ae674"
FATFS_LIB = "$(PDK_INSTALL_PATH)/ti/fs/fatfs/lib/c674/release/ti.fs.fatfs.ae674"
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl138/c674/release/ti.csl.ae674"
CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl138/c674/release/ti.csl.intc.ae674"
OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/omapl138/c674/release/ti.osal.ae674"
RTSLIB_PATH = "$(C6X_GEN_INSTALL_PATH)/lib/libc.a"

ifeq ($(BOOTMODE), mmcsd)
  INTERNALDEFS += -DBOOT_MMCSD
  SBL_MMCSD_DIR = $(SBL_SRC_DIR)/src/mmcsd
  INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_INC_DIR);$(SBL_LIB_INC_DIR);$(SBL_MMCSD_DIR);$(SBL_SOC_DIR);$(SBL_SOC_DIR)/c6748;$(C6X_GEN_INSTALL_PATH)/include
  PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(FATFS_LIB) $(MMCSD_LIB) $(CSL_LIB) $(SPI_LIB) $(BOARD_LIB) $(OSAL_LIB) $(CSL_INTC_LIB)
endif

# Compiler options
INTERNALDEFS += -mv64+ --abi=eabi -g --display_error_number --diag_warning=225 -g -DSOC_OMAPL138 -DSOC_C6748 -DEVM_$(SOC) -D$(SOC)_BUILD

# Linker options
INTERNALLINKDEFS = -mv64+ -g --diag_wrap=off --diag_warning=225 --display_error_number -z $(SBL_SOC_DIR)/c6748/linker.cmd

# Executable using device independent library and device object file
EXE=sbl.out

OBJEXT = o
ASMOBJEXT = ao
SRCDIR = $(SBL_SRC_DIR)/board

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(SRCDIR)/$(BOARD):$(SBL_SRC_DIR)/board/src:$(SBL_SRC_DIR)/soc/c6748:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/omapl138:$(SBL_SRC_DIR)/src/$(BOOTMODE):$(SBL_SRC_DIR)/src/rprc:$(PDK_INSTALL_PATH)/ti/drv/spi/soc/omapl138

#List the Source Files
SRC_C = \
	sbl_main.c \
    sbl_rprc.c \
    sbl_mmcsd.c

SRC_DRV = \
    sbl_soc.c \
    sbl_slave_boot.c

#Common entry object
ENTRY_SRC =

# FLAGS for the SourceFiles
CFLAGS += 
SRC_CFLAGS = -I. $(CFLAGS) -eo.$(OBJEXT) -ea.$(ASMOBJEXT) -g

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.asm, $(SBLOBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_DRV))

all:$(SBLBINDIR)/$(EXE) MLO

MLO: $(SBLBINDIR)/$(EXE)
ifeq ($(OS),Windows_NT)
	$(SBL_SRC_DIR)/tools/omapl13x_boot_utils/OMAP-L138/GNU/AISUtils/HexAIS_OMAP-L138.exe -o $(SBLBINDIR)/MLO -ini $(SBL_SOC_DIR)/c6748/c6748.ini $(SBLBINDIR)/sbl.out;
else
	mono $(SBL_SRC_DIR)/tools/omapl13x_boot_utils/OMAP-L138/GNU/AISUtils/HexAIS_OMAP-L138.exe -o $(SBLBINDIR)/MLO -ini $(SBL_SOC_DIR)/c6748/c6748.ini $(SBLBINDIR)/sbl.out;
endif

$(SBLBINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(INTERNALLINKDEFS) -m"$(SBLBINDIR)/sbl.map" --reread_libs --rom_model -o $@ $(PERIPHERAL_LIB) $(RTSLIB_PATH)

$(SBLOBJDIR)/%.$(OBJEXT): %.c $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS) -fr=$(SBLOBJDIR) -fs=$(SBLOBJDIR) -fc $<

$(SBLOBJDIR)/%.$(ASMOBJEXT): %.asm $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(INTERNALDEFS) $(INCS)  $< -o $@

$(SBLOBJDIR)/.created:
	@mkdir -p $(SBLOBJDIR)
	@touch $(SBLOBJDIR)/.created

$(SBLBINDIR)/.created:
	@mkdir -p $(SBLBINDIR)
	@touch $(SBLBINDIR)/.created

clean:
	@rm -f $(SBLBINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created

