#
# Macro definitions referenced below
#

empty =
space =$(empty) $(empty)

export ARMV7OBJDIR ?= ./obj
export ARMV7BINDIR ?= ./bin
export ARMV7LIBDIR ?= ./lib
ARMV7LIBDIRLOCAL ?= ../../../../lib
PDK_ARMV7LIBDIR ?= .
ARMV7HPLIBDIR ?= .
ARMV7SALIBDIR ?= .


ifeq ($(DEVICE), k2h)
CSL_DEVICE = -DDEVICE_K2H
PA_LIB   = -lpa
endif
ifeq  ($(DEVICE), k2k)
CSL_DEVICE = -DDEVICE_K2K
PA_LIB   = -lpa
endif
ifeq ($(DEVICE), k2l)
CSL_DEVICE = -DDEVICE_K2L -DNSS_GEN2
PA_LIB   = -lpa2
endif
ifeq ($(DEVICE), k2e)
CSL_DEVICE = -DDEVICE_K2E -DNSS_GEN2
PA_LIB   = -lpa2
endif

#default if DEVICE not provided
CSL_DEVICE ?= -DDEVICE_K2L -DNSS_GEN2
PA_LIB ?=-lpa2


#Cross tools
ifdef CROSS_TOOL_INSTALL_PATH
## Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar ../../../../lib
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

# INCLUDE Directories
NWAL_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/nwal
QMSS_INC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/qmss
HPLIB_INSTALL_PATH ?= $(TRANS_SDK_INSTALL_PATH)
NWAL_INC_DIR ?= $(TRANS_SDK_INSTALL_PATH)
SA_INSTALL_PATH ?= $(SALLD_INSTALL_PATH)/ti/drv/sa
CPPI_INC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/cppi
PKTLIB_INSTALL_PATH ?= $(PDK_INSTALL_PATH)/ti/runtime/pktlib
RM_INC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/rm
NWAL_TEST_INC_DIR ?= $(NWAL_SRC_DIR)/test/unitTest/armv7/linux;$(NWAL_SRC_DIR)/test
NWAL_TEST_COMMON_SRC_DIR ?= $(NWAL_SRC_DIR)/test/unitTest:$(NWAL_SRC_DIR)/test/unitTest/armv7/linux:$(NWAL_SRC_DIR)/test/common/armv7/linux:$(NWAL_SRC_DIR)/test/common/armv7/linux/pdk$(PDK_VER)

INCDIR := $(PDK_INSTALL_PATH);$(SA_INSTALL_PATH);$(NWAL_TEST_INC_DIR);$(QMSS_INC_DIR);$(CPPI_INC_DIR);$(PKTLIB_INSTALL_PATH);$(HPLIB_INSTALL_PATH);$(NWAL_INC_DIR);$(RM_INC_DIR)

# Libraries
QMSS_LIB = -lqmss
CPPI_LIB = -lcppi
PKTLIB_LIB = -lpktlib
RM_LIB = -lrm
HPLIB_LIB = -lhplib_$(DEVICE)
SA_LIB = -lsa
# Compiler options
INTERNALDEFS = $(DEBUG_FLAG) $(CSL_DEVICE) -D__ARMv7 -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD -D_GNU_SOURCE

ifeq ($(ENABLE_SA),1)
EXTERNALDEFS=-DNWAL_ENABLE_SA
NWAL_LIB = -lnwalsa_$(DEVICE)

ifeq ($(USEDYNAMIC_LIB), yes)
#presuming ARM executable would depend on dynamic library dependency
EXE_EXTN = _so
LIBS     = $(NWAL_LIB) $(QMSS_LIB) $(CPPI_LIB) $(PA_LIB) $(PKTLIB_LIB) $(HPLIB_LIB) $(SA_LIB) $(RM_LIB)
else
#forcing ARM executable to depend on static LLD libraries
LIBS = -Wl,-Bstatic $(NWAL_LIB) $(QMSS_LIB) $(CPPI_LIB) $(PA_LIB) $(PKTLIB_LIB) $(HPLIB_LIB) $(SA_LIB) $(RM_LIB) -Wl,-Bdynamic
EXE_EXTN =
endif
else
EXTERNALDEFS=
NWAL_LIB = -lnwal_$(DEVICE)
ifeq ($(USEDYNAMIC_LIB), yes)
#presuming ARM executable would depend on dynamic library dependency
EXE_EXTN = _so
LIBS     = $(NWAL_LIB) $(QMSS_LIB) $(CPPI_LIB) $(PA_LIB) $(PKTLIB_LIB) $(HPLIB_LIB) $(SA_LIB $(RM_LIB))
else
#forcing ARM executable to depend on static LLD libraries
LIBS = -static $(NWAL_LIB) $(QMSS_LIB) $(CPPI_LIB) $(PA_LIB) $(PKTLIB_LIB) $(HPLIB_LIB) $(SA_LIB) $(RM_LIB) -Wl,-Bdynamic
EXE_EXTN =
endif
SA_LIB=
AES_LIB=
SHA1_LIB=
PKTUTL_LIB=
endif

EXE = nwalTest_$(DEVICE)$(EXE_EXTN).out


# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L$(ARMV7LIBDIR) -L$(PDK_ARMV7LIBDIR) -L$(ARMV7LIBDIRLOCAL) -L$(ARMV7SALIBDIR) -L$(ARMV7HPLIBDIR) $(LIBS) -Wl,--end-group -lrt -pthread
OBJEXT = o 
SRCDIR = ./

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR):$(NWAL_TEST_COMMON_SRC_DIR)

#List the COMMONSRC Files
COMMONSRCC = \
    fw_init.c \
    fw_osal.c \
    fw_test.c \
    fw_pdk.c \
    fw_main.c \
    sockutils.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = $(CFLAGS) -I. -I../../

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/nwal/test/%.$(OBJEXT), $(COMMONSRCC))

all: tests

tests: $(ARMV7BINDIR)/nwal/test/.created $(ARMV7BINDIR)/nwal/test/$(EXE)

$(ARMV7BINDIR)/nwal/test/.created:
	@mkdir -p $(ARMV7BINDIR)/nwal/test/


$(ARMV7BINDIR)/nwal/test/$(EXE): $(COMMONSRCCOBJS)
	mkdir -p $(ARMV7BINDIR)/nwal/test/
	@echo linking $(COMMONSRCCOBJS) into $@ ...
	$(CC) $(COMMONSRCCOBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7OBJDIR)/nwal/test/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/nwal/test/.created
	@echo compiling $< ...
	$(CC) -c $(INTERNALDEFS) $(COMMONSRCCFLAGS) $(EXTERNALDEFS) $(INCS)  $< -o $@

$(ARMV7OBJDIR)/nwal/test/.created:
	mkdir -p $(ARMV7OBJDIR)/nwal/test/

clean:
	rm -rf $(ARMV7OBJDIR)/nwal/test/
	rm -rf $(ARMV7BINDIR)/nwal/test/

