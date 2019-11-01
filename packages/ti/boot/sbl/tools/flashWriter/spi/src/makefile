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

SPI_WRTR_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl
SPI_WRTR_BINDIR = $(SPI_WRTR_SRC_DIR)/tools/flashWriter/spi/bin/$(BOARD)
SPI_WRTR_OBJDIR = $(SPI_WRTR_SRC_DIR)/tools/flashWriter/spi/bin/$(BOARD)/obj


CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x

# INCLUDE Directories
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl
BOARD_DIR = $(PDK_INSTALL_PATH)/ti/board

# Libraries
ifeq ($(SOC), OMAPL137)
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/c674/release/ti.drv.uart.ae674"
SPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/c674/release/ti.drv.spi.ae674"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/c674/release/ti.drv.i2c.ae674"
else
UART_LIB = "$(PDK_INSTALL_PATH)/ti/drv/uart/lib/c66/release/ti.drv.uart.ae66"
SPI_LIB = "$(PDK_INSTALL_PATH)/ti/drv/spi/lib/c66/release/ti.drv.spi.ae66"
I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/c66/release/ti.drv.i2c.ae66"
endif

ifeq ($(SOC), OMAPL137)
	BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/c674/release/ti.board.ae674"
	BOARD_INC_DIR = $(BOARD_DIR)/src/$(BOARD)/include
	SPI_WRTR_SOC = omapl137
	SPI_WRTR_SOC_DIR = $(SPI_WRTR_SRC_DIR)/soc/omapl137
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl137/c674/release/ti.csl.ae674"
	CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/omapl137/c674/release/ti.csl.intc.ae674"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/omapl137/c674/release/ti.drv.i2c.ae674"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/omapl137/c674/release/ti.osal.ae674"
	PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(SPI_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INTC_LIB) $(BOARD_LIB)
endif

ifeq ($(SOC), K2H)
	BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/c66/release/ti.board.ae66"
	BOARD_INC_DIR = $(BOARD_DIR)/src/$(BOARD)/include
	SPI_WRTR_SOC = k2h
	SPI_WRTR_SOC_DIR = $(SPI_WRTR_SRC_DIR)/soc/k2h
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2h/c66/release/ti.csl.ae66"
	CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2h/c66/release/ti.csl.intc.ae66"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2h/c66/release/ti.drv.i2c.ae66"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2h/c66/release/ti.osal.ae66"
	PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(SPI_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INTC_LIB) $(BOARD_LIB)
endif

ifeq ($(SOC), K2E)
	BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/c66/release/ti.board.ae66"
	BOARD_INC_DIR = $(BOARD_DIR)/src/$(BOARD)/include
	SPI_WRTR_SOC = k2e
	SPI_WRTR_SOC_DIR = $(SPI_WRTR_SRC_DIR)/soc/k2e
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2e/c66/release/ti.csl.ae66"
	CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2e/c66/release/ti.csl.intc.ae66"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2e/c66/release/ti.drv.i2c.ae66"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2e/c66/release/ti.osal.ae66"
	PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(SPI_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INTC_LIB) $(BOARD_LIB)
endif

ifeq ($(SOC), K2L)
	BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/$(BOARD)/c66/release/ti.board.ae66"
	BOARD_INC_DIR = $(BOARD_DIR)/src/$(BOARD)/include
	SPI_WRTR_SOC = k2l
	SPI_WRTR_SOC_DIR = $(SPI_WRTR_SRC_DIR)/soc/k2l
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2l/c66/release/ti.csl.ae66"
	CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2l/c66/release/ti.csl.intc.ae66"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2l/c66/release/ti.drv.i2c.ae66"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2l/c66/release/ti.osal.ae66"
	PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(SPI_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INTC_LIB) $(BOARD_LIB)
endif

ifeq ($(SOC), K2K)
	BOARD_LIB = "$(PDK_INSTALL_PATH)/ti/board/lib/evmK2H/c66/release/ti.board.ae66"
	BOARD_INC_DIR = $(BOARD_DIR)/src/evmK2H/include
	SPI_WRTR_SOC = k2h
	SPI_WRTR_SOC_DIR = $(SPI_WRTR_SRC_DIR)/soc/k2k
	CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2k/c66/release/ti.csl.ae66"
	CSL_INTC_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/k2k/c66/release/ti.csl.intc.ae66"
	I2C_LIB = "$(PDK_INSTALL_PATH)/ti/drv/i2c/lib/k2k/c66/release/ti.drv.i2c.ae66"
	OSAL_LIB = "$(PDK_INSTALL_PATH)/ti/osal/lib/nonos/k2k/c66/release/ti.osal.ae66"
	PERIPHERAL_LIB = $(UART_LIB) $(I2C_LIB) $(SPI_LIB) $(OSAL_LIB) $(CSL_LIB) $(CSL_INTC_LIB) $(BOARD_LIB)
endif

INCDIR := $(CSL_DIR);$(PDK_INSTALL_PATH);$(BOARD_DIR);$(BOARD_INC_DIR);$(SPI_WRTR_SRC_DIR)/soc

LIBDIR :=

# Compiler options
ifeq ($(SOC), OMAPL137)
INTERNALDEFS += -mv64+ --abi=eabi -g --display_error_number --diag_warning=225 -g -DSOC_$(SOC) -DEVM_$(SOC) -DBOOT_SPI -D$(SOC)_BUILD
else
INTERNALDEFS += -mv6600 --abi=eabi -g --display_error_number --diag_warning=225 -g -DSOC_$(SOC) -DEVM_$(SOC) -DBOOT_SPI -D$(SOC)_BUILD
endif

# Linker options
INTERNALLINKDEFS = -mfloat-abi=hard -Wl,--undefined,__aeabi_uidiv -Wl,--undefined,__aeabi_idiv --entry Entry -nostartfiles -static -Wl,--gc-sections -Wl,-T $(SPI_WRTR_SOC_DIR)/linker.cmd -Wl,--start-group -lgcc -lc -lrdimon $(BOARD_LIB) $(PERIPHERAL_LIB) $(TMP) -Wl,--end-group $(LDFLAGS)

# Executable using device independent library and device object file
EXE=spi_flash_writer.out

OBJEXT = obj
ASMOBJEXT = ao
SRCDIR = $(SPI_WRTR_SRC_DIR)/tools/flashWriter/spi/src

INCS = --include_path=$(strip $(subst ;, --include_path=,$(INCDIR)))

VPATH=$(SRCDIR):$(SPI_WRTR_SRC_DIR)/board:$(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(SPI_WRTR_SOC):$(PDK_INSTALL_PATH)/ti/drv/mmcsd/soc/$(SPI_WRTR_SOC):$(PDK_INSTALL_PATH)/ti/drv/spi/soc/$(SPI_WRTR_SOC):$(SPI_WRTR_SOC_DIR)

ifeq ($(SOC), OMAPL137)
 LINK_CMD = $(SRCDIR)/c67x.cmd
else
 LINK_CMD = $(SRCDIR)/c6x.cmd
endif 
#List the Source Files
SRC_C = \
	sbl_spi_writer.c

SRC_DRV = \
    UART_soc.c \
    SPI_soc.c \
    sbl_soc.c

# FLAGS for the SourceFiles
CFLAGS += -Wall -O3
SRC_CFLAGS = -I. $(CFLAGS) -g -gdwarf-3 -gstrict-dwarf -Wall

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SPI_WRTR_OBJDIR)/%.$(OBJEXT), $(SRC_C))
ENTRY_OBJ = $(patsubst %.S, $(SPI_WRTR_OBJDIR)/%.$(ASMOBJEXT), $(ENTRY_SRC))
SRC_DRV_OBJS = $(patsubst %.c, $(SPI_WRTR_OBJDIR)/%.$(OBJEXT), $(SRC_DRV))

spi_flashwriter:$(SPI_WRTR_BINDIR)/$(EXE)

$(SPI_WRTR_BINDIR)/$(EXE): $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) $(SPI_WRTR_BINDIR)/.created $(SPI_WRTR_OBJDIR)/.created
	@echo linking $(SRC_OBJS) $(SRC_DRV_OBJS) $(ENTRY_OBJ) into $@ ...
	@$(CC) $(INTERNALDEFS) -z -m"$(SPI_WRTR_BINDIR)/spi_flash_writer.map" -i"$(C6X_GEN_INSTALL_PATH)/lib" --reread_libs --warn_sections --rom_model -o $@ $(LINK_CMD) $(SRC_OBJS) $(SRC_DRV_OBJS) $(PERIPHERAL_LIB)

$(SPI_WRTR_OBJDIR)/%.$(OBJEXT): %.c $(SPI_WRTR_OBJDIR)/.created
	@echo compiling $< ...
ifeq ($(SOC), OMAPL137)
	@$(CC) --include_path="$(C6X_GEN_INSTALL_PATH)/include" $(INCS) $(INTERNALDEFS) -fr=$(SPI_WRTR_OBJDIR) -fs=$(SPI_WRTR_OBJDIR) -fc $<
else
	@$(CC) --include_path="$(C6X_GEN_INSTALL_PATH)/include" $(INCS) $< $(INTERNALDEFS) --output_file=$@
endif

$(SPI_WRTR_OBJDIR)/%.$(ASMOBJEXT): %.S $(SPI_WRTR_OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(SPI_WRTR_OBJDIR)/.created:
	@mkdir -p $(SPI_WRTR_OBJDIR)
	@touch $(SPI_WRTR_OBJDIR)/.created

$(SPI_WRTR_BINDIR)/.created:
	@mkdir -p $(SPI_WRTR_BINDIR)
	@touch $(SPI_WRTR_BINDIR)/.created

spi_flashwriter_clean:
	@rm -f $(SPI_WRTR_BINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SRC_DEV_OBJS) $(ENTRY_OBJ) $(SPI_WRTR_BINDIR)/.created $(SPI_WRTR_OBJDIR)/.created
