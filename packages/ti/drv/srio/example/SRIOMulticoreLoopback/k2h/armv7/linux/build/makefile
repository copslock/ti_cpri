#
# Macro definitions referenced below
#

empty =
space =$(empty) $(empty)

ARMV7OBJDIR ?= ./obj
ARMV7BINDIR ?= ./bin
ARMV7LIBDIR ?= ./lib

ARMV7OBJDIR := $(ARMV7OBJDIR)/srio/example/SRIOMulticoreLoopback
ARMV7BINDIR := $(ARMV7BINDIR)/srio/example

#Cross tools
ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

# INCLUDE Directories
QMSS_INC_DIR = $(PDK_INSTALL_PATH)/ti/drv/qmss
CPPI_INC_DIR = $(PDK_INSTALL_PATH)/ti/drv/cppi
SRIO_INC_DIR = $(PDK_INSTALL_PATH)/ti/drv/srio

SRIO_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/srio
SERDES_SRC_DIR = $(PDK_INSTALL_PATH)/ti/csl/src/ip/serdes_sb/V0
SRIO_SAMPLE_DIR = $(SRIO_SRC_DIR)/example/SRIOMulticoreLoopback
# must put DEVICE_SAMPLE_DIR first in search paths
SRIO_ARM_LIN_DEVICE_SAMPLE_DIR = $(SRIO_SAMPLE_DIR)/$(DEVICE)/armv7/linux
# must put COMMON_SAMPLE_DIR second in search paths
SRIO_ARM_LIN_COMMON_SAMPLE_DIR = $(SRIO_SAMPLE_DIR)/src/armv7/linux

INCDIR := $(PDK_INSTALL_PATH);$(CPPI_INC_DIR);$(QMSS_INC_DIR);$(SRIO_INC_DIR);$(SRIO_SAMPLE_DIR)/src;$(SRIO_ARM_LIN_DEVICE_SAMPLE_DIR);$(SRIO_ARM_LIN_COMMON_SAMPLE_DIR)

# Libraries
QMSS_LIB = -lqmss
CPPI_LIB = -lcppi
SRIO_LIB = -lsrio
RM_LIB = -lrm
CMEM_LIB = -lticmem

ifeq ($(USEDYNAMIC_LIB), yes)
#presuming ARM executable would depend on dynamic library dependency
EXE_EXTN = _so
LIBS     = $(QMSS_LIB) $(CPPI_LIB) $(SRIO_LIB) $(RM_LIB) $(CMEM_LIB)
else
#forcing ARM executable to depend on static LLD libraries
EXE_EXTN =
LIBS     = -Wl,-static $(QMSS_LIB) $(CPPI_LIB) $(SRIO_LIB) $(RM_LIB) $(CMEM_LIB) -Wl,-Bdynamic
endif

# Compiler options
INTERNALDEFS = $(DEBUG_FLAG) -D_FILE_OFFSET_BITS=64 -D__ARMv7 -DDEVICE_K2H -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD -DINTERNAL_LINKING_RAM -D_GNU_SOURCE

# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L $(ARMV7LIBDIR) $(LIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)

EXE=srioMulticoreLoopback$(EXE_EXTN).out
OBJEXT = o 
SRCDIR = $(SRIO_SAMPLE_DIR)/src:$(SRIO_ARM_LIN_DEVICE_SAMPLE_DIR):$(SRIO_ARM_LIN_COMMON_SAMPLE_DIR):

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(SERDES_SRC_DIR)

#List the Source Files
TEST_SRC = \
    multicoreLoopback.c \
    linuxutil.c \
    fw_main.c \
    fw_init.c \
    fw_osal.c \
    fw_mem_allocator.c \
    sockutils.c

SERDES_SRC = \
    csl_wiz8_sb_refclk312p5MHz_20bit_6p25Gbps.c \
    csl_wiz8_sb_refclk156p25MHz_20bit_6p25Gbps.c \
    csl_wiz8_sb_refclk125MHz_20bit_6p25Gbps.c \
    csl_wiz8_sb_refclk156p25MHz_20bit_5Gbps.c \
    csl_wiz8_sb_refclk125MHz_20bit_5Gbps.c 

# FLAGS for the SourceFiles
SRC_CFLAGS = -I. $(CFLAGS)

# Make Rule for the SRC Files
TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(TEST_SRC) $(SERDES_SRC))

all:$(ARMV7BINDIR)/$(EXE)

$(ARMV7BINDIR)/$(EXE): $(TEST_SRC_OBJS) $(ARMV7BINDIR)/.created $(ARMV7OBJDIR)/.created
	@echo linking $(TEST_SRC_OBJS) into $@ ...
	@$(CC) $(TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7BINDIR)/.created:
	@mkdir -p $(ARMV7BINDIR)
	@touch $(ARMV7BINDIR)/.created

clean:
	@rm -fr $(ARMV7OBJDIR)
	@rm -fr $(ARMV7BINDIR)

