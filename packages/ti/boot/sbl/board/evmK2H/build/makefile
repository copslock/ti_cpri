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
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/k2h/a15/release/ti.drv.uart.aa15fg"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2h/a15/release/ti.drv.i2c.aa15fg"
SPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/a15/release/ti.drv.spi.aa15fg"
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2h/a15/release/ti.csl.aa15fg"
CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2h/a15/release/ti.csl.init.aa15fg"
OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2h/a15/release/ti.osal.aa15fg"

ifeq ($(BOOTMODE), spi)
  INTERNALDEFS += -DBOOT_SPI
  SBL_SPI_DIR = $(SBL_SRC_DIR)/src/spi
  INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_INC_DIR);$(SBL_LIB_INC_DIR);$(SBL_SPI_DIR);$(SBL_SOC_DIR);$(SBL_SOC_DIR)/k2h;$(SBL_SOC_DIR)/a15
  PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(SPI_LIB) $(OSAL_LIB)
endif

# Compiler options
INTERNALDEFS += -g -Wall -D__ARMv7 -DA15_CORE -DSOC_K2H -mcpu=cortex-a15 -mtune=cortex-a15 -marm -mfloat-abi=hard -mfpu=neon -D$(SOC)_BUILD

# Linker options
INTERNALLINKDEFS = -g -gdwarf-3 -gstrict-dwarf -Wall  -mfloat-abi=hard -Wl,--undefined,__aeabi_uidiv -Wl,--undefined,__aeabi_idiv --entry Entry -nostartfiles -static -Wl,--gc-sections -Wl,-T $(SBL_SOC_DIR)/k2h/linker.cmd -Wl,--start-group -l"c" -lrdimon -l"m" -l"nosys" $(BOARD_LIB) $(PERIPHERAL_LIB) -Wl,--end-group $(LDFLAGS)

# Executable using device independent library and device object file
EXE=sbl.out

OBJEXT = o
ASMOBJEXT = ao
SRCDIR = $(SBL_SRC_DIR)/board

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(SRCDIR)/$(BOARD):$(SBL_SRC_DIR)/board/src:$(SBL_SRC_DIR)/soc/k2h:$(SBL_SRC_DIR)/soc/a15:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/k2h:$(SBL_SRC_DIR)/src/$(BOOTMODE):$(SBL_SRC_DIR)/src/rprc:$(PDK_INSTALL_PATH)/ti/drv/spi/soc/k2h

#List the Source Files
SRC_C = \
	sbl_main.c \
	sbl_rprc.c \
	sbl_spi.c \
	sbl_slave_boot.c \
	sbl_a15.c

SRC_DRV = \
    sbl_soc.c \
    SPI_soc.c

#Common entry object
ENTRY_SRC = sbl_init.S

# FLAGS for the SourceFiles
CFLAGS += -Wall -O3
SRC_CFLAGS = -I. $(CFLAGS) -g -gdwarf-3 -gstrict-dwarf -Wall 

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.S, $(SBLOBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_DRV))

ifeq ($(OS),Windows_NT)
  TIIMAGE_EXE = tiimage.exe
else
  TIIMAGE_EXE = tiimage
endif

all:$(SBLBINDIR)/$(EXE) MLO

MLO: $(SBLBINDIR)/$(EXE)
	$(BIN) -O binary $(SBLBINDIR)/sbl.out $(SBLBINDIR)/sbl.bin
	$(SBL_SRC_DIR)/tools/tiImageGen/bin/$(TIIMAGE_EXE) 0x0C0B0000 SPI $(SBLBINDIR)/sbl.bin $(SBLBINDIR)/MLO;

$(SBLBINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(INTERNALLINKDEFS) -Wl,-Map=$(SBLBINDIR)/sbl.map -o $@

$(SBLOBJDIR)/%.$(OBJEXT): %.c $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(SBLOBJDIR)/%.$(ASMOBJEXT): %.S $(SBLOBJDIR)/.created
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

