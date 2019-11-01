# Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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
SBLOBJDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/a15/mpuc0/obj
SBLBINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/a15/mpuc0/bin
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
SBL_SOC_DIR = $(SBL_SRC_DIR)/soc/am57xx
SRCDIR = $(SBL_SRC_DIR)/example/mpuMulticoreApp
INCDIR = $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_SOC_DIR)

# Libraries
BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/a15/release/ti.board.aa15fg"
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/am572x/a15/release/ti.drv.uart.aa15fg"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/am572x/a15/release/ti.drv.i2c.aa15fg"
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am572x/a15/release/ti.csl.aa15fg"
CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am572x/a15/release/ti.csl.init.aa15fg"
OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/am572x/a15/release/ti.osal.aa15fg"

LIBDIR :=

# Compiler options
INTERNALDEFS += -g -gdwarf-3 -gstrict-dwarf -Wall $(DEBUG_FLAG) -D__ARMv7 -DSOC_$(SOC) -mtune=cortex-a15 -march=armv7-a -marm -mfloat-abi=hard -mfpu=neon -D$(SOC)_BUILD

# Linker options
INTERNALLINKDEFS = -mfloat-abi=hard -Wl,--undefined,__aeabi_uidiv -Wl,--undefined,__aeabi_idiv --entry Entry -nostartfiles -static -Wl,--gc-sections -Wl,-T $(SRCDIR)/lnk_mpu.cmd -Wl,--start-group -lgcc -lc -lrdimon $(BOARD_LIB) $(UART_LIB) $(I2C_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(OSAL_LIB) -Wl,--end-group $(LDFLAGS)

# Executable using device independent library and device object file
EXE=sbl_app.out

OBJEXT = o
ASMOBJEXT = ao

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(PDK_INSTALL_PATH)/ti/drv/uart/soc/am572x:$(SBL_SRC_DIR)/board:$(SBL_SRC_DIR)/soc/am57xx

#List the Source Files
SRC_C = \
	sbl_multicore_mpu.c \
	mailbox.c \
   	sbl_startup.c

SRC_DRV = \
	UART_soc.c

#Common entry object
ENTRY_SRC = sbl_init.S

# FLAGS for the SourceFiles
CFLAGS += -Wall -O2
SRC_CFLAGS = -I. $(CFLAGS) -g -gdwarf-3 -gstrict-dwarf -Wall

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.S, $(SBLOBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(SBLOBJDIR)/%.$(OBJEXT), $(SRC_DRV))

example:$(SBLBINDIR)/$(EXE)

$(SBLBINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) into $@ ...
	@$(CC) $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(INTERNALLINKDEFS) -Wl,-Map=$(SBLBINDIR)/sbl_app.map -o $@

$(SBLOBJDIR)/%.$(OBJEXT): %.c $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(SBLOBJDIR)/%.$(ASMOBJEXT): %.S $(SBLOBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(SBLOBJDIR)/.created:
	@mkdir -p $(SBLOBJDIR)
	@touch $(SBLOBJDIR)/.created

$(SBLBINDIR)/.created:
	@mkdir -p $(SBLBINDIR)
	@touch $(SBLBINDIR)/.created

example_clean:
	@rm -f $(SBLBINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SBLBINDIR)/.created $(SBLOBJDIR)/.created
