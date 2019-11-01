#
# Macro definitions referenced below
#

empty =
space =$(empty) $(empty)

ARMV7OBJDIR ?= ./obj/$(DEVICE)
ARMV7BINDIR ?= ./bin/$(DEVICE)
ARMV7LIBDIR ?= ./lib

ARMV7OBJDIR := $(ARMV7OBJDIR)/qmss/test
ARMV7BINDIR := $(ARMV7BINDIR)/qmss/test

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

QMSS_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/qmss
QMSS_TEST_DIR = $(QMSS_SRC_DIR)/test
QMSS_ARM_LIN_TEST_DIR = $(QMSS_TEST_DIR)/$(DEVICE)/armv7/linux

INCDIR := $(PDK_INSTALL_PATH);$(CPPI_INC_DIR);$(QMSS_INC_DIR);$(QMSS_TEST_DIR)/src;$(QMSS_ARM_LIN_TEST_DIR)

# Libraries
QMSS_LIB = -lqmss
CPPI_LIB = -lcppi
RM_LIB = -lrm

ifeq ($(USEDYNAMIC_LIB), yes)
#presuming ARM executable would depend on dynamic library dependency
EXE_EXTN = _so
LIBS     = $(QMSS_LIB) $(CPPI_LIB) $(RM_LIB)
else
#forcing ARM executable to depend on static LLD libraries
EXE_EXTN =
LIBS     = -Wl,-Bstatic $(QMSS_LIB) $(CPPI_LIB) $(RM_LIB) -Wl,-Bdynamic
endif

# Compiler options
INTERNALDEFS = $(DEBUG_FLAG) -D__ARMv7 -DSOC_K2E -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD -D_GNU_SOURCE

# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L $(ARMV7LIBDIR) $(LIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)

DCFG_TEST_EXE=qmDcfgTest$(EXE_EXTN).out
DESC_TEST_EXE=qmDescTest$(EXE_EXTN).out
INS_REG_TEST_EXE=qmInsRegionTest$(EXE_EXTN).out
QALLOC_TEST_EXE=qmQAllocTest$(EXE_EXTN).out
SCFG_TEST_EXE=qmScfgTest$(EXE_EXTN).out

OBJEXT = o 

SRCDIR = $(QMSS_TEST_DIR)/src:$(QMSS_ARM_LIN_TEST_DIR):

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR)

#List the Source Files
FW_TEST_SRC = \
    fw_main.c \
    fw_init.c \
    fw_osal.c \
    fw_mem_allocator.c \
    sockutils.c

DCFG_TEST_SRC = \
    test_dynamicCfg.c

DESC_TEST_SRC = \
    test_descAlloc.c

INS_REG_TEST_SRC = \
    test_insRegion.c

QALLOC_TEST_SRC = \
    queueAlloc.c

SCFG_TEST_SRC = \
    test_staticCfg.c

# FLAGS for the SourceFiles
SRC_CFLAGS = -I. $(CFLAGS)

# Make Rule for the SRC Files
FW_TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(FW_TEST_SRC))
DCFG_TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(DCFG_TEST_SRC))
SCFG_TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(SCFG_TEST_SRC))
DESC_TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(DESC_TEST_SRC))
INS_REG_TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(INS_REG_TEST_SRC))
QALLOC_TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(QALLOC_TEST_SRC))

all:$(ARMV7BINDIR)/$(DCFG_TEST_EXE) $(ARMV7BINDIR)/$(DESC_TEST_EXE) $(ARMV7BINDIR)/$(INS_REG_TEST_EXE) $(ARMV7BINDIR)/$(QALLOC_TEST_EXE) $(ARMV7BINDIR)/$(SCFG_TEST_EXE)

$(ARMV7BINDIR)/$(DCFG_TEST_EXE): $(DCFG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(ARMV7BINDIR)/.created
	@echo linking $(DCFG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) into $@ ...
	@$(CC) $(DCFG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7BINDIR)/$(DESC_TEST_EXE): $(DESC_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(ARMV7BINDIR)/.created
	@echo linking $(DESC_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) into $@ ...
	@$(CC) $(DESC_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7BINDIR)/$(SCFG_TEST_EXE): $(SCFG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(ARMV7BINDIR)/.created
	@echo linking $(SCFG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) into $@ ...
	@$(CC) $(SCFG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7BINDIR)/$(INS_REG_TEST_EXE): $(INS_REG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(ARMV7BINDIR)/.created
	@echo linking $(INS_REG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) into $@ ...
	@$(CC) $(INS_REG_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7BINDIR)/$(QALLOC_TEST_EXE): $(QALLOC_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(ARMV7BINDIR)/.created
	@echo linking $(QALLOC_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) into $@ ...
	@$(CC) $(QALLOC_TEST_SRC_OBJS) $(FW_TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

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

