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

MMCSD_WRTR_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl

ifeq ($(SOC), OMAPL138)
MMCSD_WRTR_BINDIR = $(MMCSD_WRTR_SRC_DIR)/tools/flashWriter/mmcsd/bin/$(BOARD)
MMCSD_WRTR_OBJDIR = $(MMCSD_WRTR_SRC_DIR)/tools/flashWriter/mmcsd/bin/$(BOARD)/obj
MMCSD_WRTR_SOC_DIR = $(MMCSD_WRTR_SRC_DIR)/soc/omapl138
MMCSD_WRTR_BOARD = $(BOARD)
MMCSD_WRTR_SOC = $(SOC)
else
MMCSD_WRTR_BINDIR = $(MMCSD_WRTR_SRC_DIR)/tools/flashWriter/mmcsd/bin/lcdkC6748
MMCSD_WRTR_OBJDIR = $(MMCSD_WRTR_SRC_DIR)/tools/flashWriter/mmcsd/bin/lcdkC6748/obj
MMCSD_WRTR_SOC_DIR = $(MMCSD_WRTR_SRC_DIR)/soc/c6748
MMCSD_WRTR_BOARD = lcdkOMAPL138
MMCSD_WRTR_SOC = OMAPL138
endif

CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x

# INCLUDE Directories
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl
BOARD_DIR = $(PDK_INSTALL_PATH)/ti/board

# Libraries
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/c674/release/ti.drv.uart.ae674"
MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/c674/release/ti.drv.mmcsd.ae674"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/c674/release/ti.drv.i2c.ae674"

BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(MMCSD_WRTR_BOARD)/c674/release/ti.board.ae674"
BOARD_INC_DIR = $(BOARD_DIR)/src/$(MMCSD_WRTR_BOARD)/include
MMCSD_WRTR_SOC_FILE_DIR = omapl138
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl138/c674/release/ti.csl.ae674"
CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl138/c674/release/ti.csl.intc.ae674"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/omapl138/c674/release/ti.drv.i2c.ae674"
OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/omapl138/c674/release/ti.osal.ae674"
RTSLIB_PATH = "$(C6X_GEN_INSTALL_PATH)/lib/libc.a"

PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(MMCSD_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INTC_LIB) $(BOARD_LIB) $(RTSLIB_PATH)

INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(BOARD_INC_DIR);$(MMCSD_WRTR_SRC_DIR)/soc;$(C6X_GEN_INSTALL_PATH)/include

LIBDIR :=

# Compiler options
INTERNALDEFS += -mv64+ --abi=eabi -g --display_error_number --diag_warning=225 -g -DSOC_$(MMCSD_WRTR_SOC) -DEVM_$(MMCSD_WRTR_SOC) -DBOOT_MMCSD -D$(SOC)_BUILD

# Linker options
INTERNALLINKDEFS = -mfloat-abi=hard -Wl,--undefined,__aeabi_uidiv -Wl,--undefined,__aeabi_idiv --entry Entry -nostartfiles -static -Wl,--gc-sections -Wl,-T $(MMCSD_WRTR_SOC_DIR)/linker.cmd -Wl,--start-group -lgcc -lc -lrdimon $(BOARD_LIB) $(PERIPHERAL_LIB) $(TMP) -Wl,--end-group $(LDFLAGS)

# Executable using device independent library and device object file
EXE=mmcsd_flash_writer.out

OBJEXT = obj
ASMOBJEXT = ao
SRCDIR = $(MMCSD_WRTR_SRC_DIR)/tools/flashWriter/mmcsd/src

INCS = --include_path=$(strip $(subst ;, --include_path=,$(INCDIR)))

VPATH=$(SRCDIR):$(MMCSD_WRTR_SRC_DIR)/board:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(MMCSD_WRTR_SOC_FILE_DIR):$(PDK_INSTALL_PATH)/ti/drv/mmcsd/soc/$(MMCSD_WRTR_SOC_FILE_DIR):$(PDK_INSTALL_PATH)/ti/drv/mmcsd/soc/$(MMCSD_WRTR_SOC_FILE_DIR):$(MMCSD_WRTR_SOC_DIR)

LINK_CMD = $(SRCDIR)/c67x.cmd

#List the Source Files
SRC_C = \
	sbl_mmcsd_writer.c

SRC_DRV = \
    UART_soc.c \
    MMCSD_soc.c \
    sbl_soc.c

# FLAGS for the SourceFiles
CFLAGS +=
SRC_CFLAGS = -I. $(CFLAGS) -eo.$(OBJEXT) -ea.$(ASMOBJEXT) -g

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(MMCSD_WRTR_OBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.asm, $(MMCSD_WRTR_OBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(MMCSD_WRTR_OBJDIR)/%.$(OBJEXT), $(SRC_DRV))

mmcsd_flashwriter:$(MMCSD_WRTR_BINDIR)/$(EXE)

$(MMCSD_WRTR_BINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(MMCSD_WRTR_BINDIR)/.created $(MMCSD_WRTR_OBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(INTERNALDEFS) -z -m"$(MMCSD_WRTR_BINDIR)/mmcsd_flash_writer.map" -i"$(C6X_GEN_INSTALL_PATH)/lib" --reread_libs --warn_sections --rom_model -o $@ $(LINK_CMD) $(SRC_OBJS) $(SRC_DRV_OBJS) $(PERIPHERAL_LIB)

$(MMCSD_WRTR_OBJDIR)/%.$(OBJEXT): %.c $(MMCSD_WRTR_OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS) -fr=$(MMCSD_WRTR_OBJDIR) -fs=$(MMCSD_WRTR_OBJDIR) -fc $<

$(MMCSD_WRTR_OBJDIR)/%.$(ASMOBJEXT): %.asm $(MMCSD_WRTR_OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(MMCSD_WRTR_OBJDIR)/.created:
	@mkdir -p $(MMCSD_WRTR_OBJDIR)
	@touch $(MMCSD_WRTR_OBJDIR)/.created

$(MMCSD_WRTR_BINDIR)/.created:
	@mkdir -p $(MMCSD_WRTR_BINDIR)
	@touch $(MMCSD_WRTR_BINDIR)/.created

mmcsd_flashwriter_clean:
	@rm -f $(MMCSD_WRTR_BINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SRC_DEV_OBJS) $(ENTRY_OBJ) $(MMCSD_WRTR_BINDIR)/.created $(MMCSD_WRTR_OBJDIR)/.created
