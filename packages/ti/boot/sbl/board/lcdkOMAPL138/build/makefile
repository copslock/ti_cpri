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

ifdef TOOLCHAIN_PATH_ARM9
TOOLCHAIN_PATH = $(TOOLCHAIN_PATH_ARM9)
else
TOOLCHAIN_PATH = $(TOOLCHAIN_PATH_Arm9)
endif

#Cross tools
CC = $(TOOLCHAIN_PATH)/bin/armcl
AR = $(TOOLCHAIN_PATH)/bin/armar
LNK = $(TOOLCHAIN_PATH)/bin/armlnk
SIZE = $(TOOLCHAIN_PATH)/bin/armofd

# INCLUDE Directories
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl
BOARD_DIR = $(PDK_INSTALL_PATH)/ti/board
SBL_INC_DIR = $(SBL_SRC_DIR)/board/src
SBL_LIB_INC_DIR = $(SBL_SRC_DIR)/src/rprc
SBL_SOC_DIR = $(SBL_SRC_DIR)/soc

# Libraries
BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/arm9/release/ti.board.ae9"
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/omapl138/arm9/release/ti.drv.uart.ae9"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/omapl138/arm9/release/ti.drv.i2c.ae9"
SPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/omapl138/arm9/release/ti.drv.spi.ae9"
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl138/arm9/release/ti.csl.ae9"
CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl138/arm9/release/ti.csl.init.ae9"
RTSLIB_PATH = "$(TOOLCHAIN_PATH)/lib/libc.a"
MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/omapl138/arm9/release/ti.drv.mmcsd.ae9"
FATFS_LIB = "$(PDK_INSTALL_PATH)/ti/fs/fatfs/lib/arm9/release/ti.fs.fatfs.ae9"
OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/omapl138/arm9/release/ti.osal.ae9"

ifeq ($(BOOTMODE), mmcsd)
  INTERNALDEFS += -DBOOT_MMCSD
  SBL_MMCSD_DIR = $(SBL_SRC_DIR)/src/mmcsd
  INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_INC_DIR);$(SBL_LIB_INC_DIR);$(SBL_MMCSD_DIR);$(SBL_SOC_DIR);$(SBL_SOC_DIR)/omapl138;$(TOOLCHAIN_PATH)/include
  PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(FATFS_LIB) $(MMCSD_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(SPI_LIB) $(BOARD_LIB) $(OSAL_LIB) $(RTSLIB_PATH)
endif

# Compiler options
INTERNALDEFS += -g $(DEBUG_FLAG) -mv5e --code_state=32 -me -g $(DEFFLAGS) --diag_wrap=off --diag_warning=225 --display_error_number  -DSOC_$(SOC) -D$(SOC)_BUILD -DlcdkOMAPL138

# Linker options
INTERNALLINKDEFS = -mv5e --code_state=32 -me -g $(DEVFLAGS) --diag_wrap=off --diag_warning=225 --display_error_number -z $(SBL_SOC_DIR)/omapl138/linker.cmd

# Executable using device independent library and device object file
EXE=sbl.out

OBJEXT = o
ASMOBJEXT = ao
SRCDIR = $(SBL_SRC_DIR)/board

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(SRCDIR)/$(BOARD):$(SBL_SRC_DIR)/board/src:$(SBL_SRC_DIR)/soc/omapl138:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/omapl138:$(SBL_SRC_DIR)/src/$(BOOTMODE):$(SBL_SRC_DIR)/src/rprc:$(PDK_INSTALL_PATH)/ti/drv/spi/soc/omapl138

#List the Source Files
SRC_C = \
	sbl_main.c \
    sbl_rprc.c \
    sbl_mmcsd.c

SRC_DRV = \
    sbl_soc.c \
    sbl_soc.c \
    sbl_slave_boot.c

#Common entry object
ENTRY_SRC = 

# FLAGS for the SourceFiles
CFLAGS += -ms -oe --inline_recursion_limit=20
SRC_CFLAGS = -I. $(CFLAGS) -eo.$(OBJEXT) -ea.$(ASMOBJEXT) -g

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.asm, $(SBLOBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_DRV))

all:$(SBLBINDIR)/$(EXE) MLO

MLO: $(SBLBINDIR)/$(EXE)
ifeq ($(OS),Windows_NT)
	$(SBL_SRC_DIR)/tools/omapl13x_boot_utils/OMAP-L138/GNU/AISUtils/HexAIS_OMAP-L138.exe -o $(SBLBINDIR)/MLO -ini $(SBL_SOC_DIR)/omapl138/omapl138.ini $(SBLBINDIR)/sbl.out;
else
	mono $(SBL_SRC_DIR)/tools/omapl13x_boot_utils/OMAP-L138/GNU/AISUtils/HexAIS_OMAP-L138.exe -o $(SBLBINDIR)/MLO -ini $(SBL_SOC_DIR)/omapl138/omapl138.ini $(SBLBINDIR)/sbl.out;
endif

$(SBLBINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(INTERNALLINKDEFS) -m"$(SBLBINDIR)/sbl.map" --reread_libs --rom_model -o $@ $(PERIPHERAL_LIB)

$(SBLOBJDIR)/%.$(OBJEXT): %.c $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS) -fr=$(SBLOBJDIR) -fs=$(SBLOBJDIR) -fc $<

$(SBLOBJDIR)/%.$(ASMOBJEXT): %.asm $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(INTERNALDEFS) $(INCS)  $< --output_file=$@

$(SBLOBJDIR)/.created:
	@mkdir -p $(SBLOBJDIR)
	@touch $(SBLOBJDIR)/.created

$(SBLBINDIR)/.created:
	@mkdir -p $(SBLBINDIR)
	@touch $(SBLBINDIR)/.created

clean:
	@rm -f $(SBLBINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created

