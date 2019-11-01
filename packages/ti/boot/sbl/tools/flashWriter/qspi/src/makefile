# Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
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
QSPI_WRTR_BINDIR = $(SBL_SRC_DIR)/tools/flashWriter/qspi/bin/$(BOARD)
QSPI_WRTR_OBJDIR = $(SBL_SRC_DIR)/tools/flashWriter/qspi/bin/$(BOARD)/obj
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

# Libraries
BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/a15/release/ti.board.aa15fg"
FATFS_LIB = "$(PDK_INSTALL_PATH)/ti/fs/fatfs/lib/a15/release/ti.fs.fatfs.aa15fg"

INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(SBL_SRC_DIR)/soc;$(SBL_SRC_DIR)/soc/a15

# Board Specific Flags
ifeq ($(SOC), AM572x)
	QSPI_WRTR_SOC = am572x
	QSPI_WRTR_MMCSD_SOC = am57xx
	QSPI_WRTR_SOC_DIR = $(SBL_SRC_DIR)/soc/am57xx
	QSPI_LINKER = linker.cmd
	UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/am572x/a15/release/ti.drv.uart.aa15fg"
	QSPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/am572x/a15/release/ti.drv.spi.aa15fg"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/am572x/a15/release/ti.drv.i2c.aa15fg"
	MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/am572x/a15/release/ti.drv.mmcsd.aa15fg"
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am572x/a15/release/ti.csl.aa15fg"
	CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am572x/a15/release/ti.csl.init.aa15fg"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/am572x/a15/release/ti.osal.aa15fg"
	PERIPHERAL_LIB = $(UART_LIB) $(QSPI_LIB) $(I2C_LIB) $(MMCSD_LIB) $(FATFS_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(OSAL_LIB)
endif # ifeq ($(SOC), AM572x)

ifeq ($(SOC), AM574x)
	QSPI_WRTR_SOC = am574x
	QSPI_WRTR_MMCSD_SOC = am57xx
	QSPI_WRTR_SOC_DIR = $(SBL_SRC_DIR)/soc/am57xx
	QSPI_LINKER = linker.cmd
	UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/am574x/a15/release/ti.drv.uart.aa15fg"
	QSPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/am574x/a15/release/ti.drv.spi.aa15fg"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/am574x/a15/release/ti.drv.i2c.aa15fg"
	MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/am574x/a15/release/ti.drv.mmcsd.aa15fg"
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am574x/a15/release/ti.csl.aa15fg"
	CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am574x/a15/release/ti.csl.init.aa15fg"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/am574x/a15/release/ti.osal.aa15fg"
	PERIPHERAL_LIB = $(UART_LIB) $(QSPI_LIB) $(I2C_LIB) $(MMCSD_LIB) $(FATFS_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(OSAL_LIB)
endif # ifeq ($(SOC), AM574x)

ifeq ($(SOC), AM571x)
	QSPI_WRTR_SOC = am571x
	QSPI_WRTR_MMCSD_SOC = am57xx
	QSPI_WRTR_SOC_DIR = $(SBL_SRC_DIR)/soc/am57xx
	QSPI_LINKER = linker.cmd
	UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/am571x/a15/release/ti.drv.uart.aa15fg"
	QSPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/am571x/a15/release/ti.drv.spi.aa15fg"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/am571x/a15/release/ti.drv.i2c.aa15fg"
	MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/am571x/a15/release/ti.drv.mmcsd.aa15fg"
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am571x/a15/release/ti.csl.aa15fg"
	CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/am571x/a15/release/ti.csl.init.aa15fg"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/am571x/a15/release/ti.osal.aa15fg"
	PERIPHERAL_LIB = $(UART_LIB) $(QSPI_LIB) $(I2C_LIB) $(MMCSD_LIB) $(FATFS_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(OSAL_LIB)
endif # ifeq ($(SOC), AM571x)

ifeq ($(SOC), K2G)
	QSPI_WRTR_SOC = k2g
	QSPI_WRTR_MMCSD_SOC = k2g
	QSPI_WRTR_SOC_DIR = $(SBL_SRC_DIR)/soc/k2g
	QSPI_LINKER = writer_linker.cmd
	UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/k2g/a15/release/ti.drv.uart.aa15fg"
	QSPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/k2g/a15/release/ti.drv.spi.aa15fg"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2g/a15/release/ti.drv.i2c.aa15fg"
	MMCSD_LIB = "$(PDK_INSTALL_PATH)/ti/drv/mmcsd/lib/k2g/a15/release/ti.drv.mmcsd.aa15fg"
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2g/a15/release/ti.csl.aa15fg"
	CSL_INIT_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2g/a15/release/ti.csl.init.aa15fg"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2g/a15/release/ti.drv.i2c.aa15fg"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2g/a15/release/ti.osal.aa15fg"
	PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(QSPI_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INIT_LIB) $(MMCSD_LIB) $(FATFS_LIB)
endif

LIBDIR :=

# Compiler options
INTERNALDEFS += -g -gdwarf-3 -gstrict-dwarf -Wall $(DEBUG_FLAG) -D__ARMv7 -DSOC_$(SOC) -D$(BOARD) -DBOOT_QSPI -mtune=cortex-a15 -march=armv7-a -marm -mfloat-abi=hard -mfpu=neon -D$(SOC)_BUILD

# Linker options
INTERNALLINKDEFS = -mfloat-abi=hard --specs=nano.specs -Wl,--undefined,__aeabi_uidiv -Wl,--undefined,__aeabi_idiv --entry Entry -nostartfiles -static -Wl,--gc-sections -Wl,-T $(QSPI_WRTR_SOC_DIR)/$(QSPI_LINKER) -Wl,--start-group -lgcc -lc -lrdimon $(BOARD_LIB) $(PERIPHERAL_LIB) -Wl,--end-group $(LDFLAGS)

# Executable using device independent library and device object file
EXE=qspi_flash_writer.out

OBJEXT = o
ASMOBJEXT = ao
SRCDIR = $(SBL_SRC_DIR)/tools/flashWriter/qspi/src

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(SBL_SRC_DIR)/board:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(QSPI_WRTR_SOC):$(PDK_INSTALL_PATH)/ti/drv/mmcsd/soc/$(QSPI_WRTR_MMCSD_SOC):$(PDK_INSTALL_PATH)/ti/drv/spi/soc/$(QSPI_WRTR_SOC):$(QSPI_WRTR_SOC_DIR):$(SBL_SRC_DIR)/soc/a15

#List the Source Files
SRC_C = \
	sbl_qspi_writer.c

SRC_DRV = \
    UART_soc.c \
    SPI_soc.c \
    MMCSD_soc.c \
    sbl_soc.c \
    sbl_a15.c

#Common entry object
ENTRY_SRC = sbl_qspi_writer.S

# FLAGS for the SourceFiles
CFLAGS += -Wall -O2
SRC_CFLAGS = -I. $(CFLAGS) -g -gdwarf-3 -gstrict-dwarf -Wall

ifeq ($(SECUREMODE), yes)
  SRC_CFLAGS += -DSECURE_BOOT
  SIGNED_WRITER = $(QSPI_WRTR_BINDIR)/flash_writer_signed.bin
else
  SIGNED_WRITER =
endif

out2rprc=$(SBL_SRC_DIR)/tools/out2rprc/bin/out2rprc.exe
multigen=$(SBL_SRC_DIR)/tools/multicoreImageGen/bin/MulticoreImageGen

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(QSPI_WRTR_OBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.S, $(QSPI_WRTR_OBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(QSPI_WRTR_OBJDIR)/%.$(OBJEXT), $(SRC_DRV))

flashwriter:$(SIGNED_WRITER) $(QSPI_WRTR_BINDIR)/$(EXE) 

$(SIGNED_WRITER): $(QSPI_WRTR_BINDIR)/$(EXE)
	mono $(out2rprc) $< $(QSPI_WRTR_BINDIR)/tmp.rprc
	$(multigen) LE 55 $(QSPI_WRTR_BINDIR)/writer.bin 0 $(QSPI_WRTR_BINDIR)/tmp.rprc
	$(SECBIN) -i  $(QSPI_WRTR_BINDIR)/writer.bin -r $(KEYPATH)/privkey.pem -a $(KEYPATH)/aeskey.txt -d $(SOC) -o  $@


$(QSPI_WRTR_BINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(QSPI_WRTR_BINDIR)/.created $(QSPI_WRTR_OBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(INTERNALLINKDEFS) -Wl,-Map=$(QSPI_WRTR_BINDIR)/qspi_flash_writer.map -o $@

$(QSPI_WRTR_OBJDIR)/%.$(OBJEXT): %.c $(QSPI_WRTR_OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(QSPI_WRTR_OBJDIR)/%.$(ASMOBJEXT): %.S $(QSPI_WRTR_OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(QSPI_WRTR_OBJDIR)/.created:
	@mkdir -p $(QSPI_WRTR_OBJDIR)
	@touch $(QSPI_WRTR_OBJDIR)/.created

$(QSPI_WRTR_BINDIR)/.created:
	@mkdir -p $(QSPI_WRTR_BINDIR)
	@touch $(QSPI_WRTR_BINDIR)/.created

flashwriter_clean:
	@rm -f $(QSPI_WRTR_BINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SRC_DEV_OBJS) $(ENTRY_OBJ) $(QSPI_WRTR_BINDIR)/.created $(QSPI_WRTR_OBJDIR)/.created
