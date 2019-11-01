#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: ./build/am572x/armv7/ti.transport.ndk.nimu.am572x.aa15fg.mk
#*
#* DESCRIPTION: Defines Source Files, Compilers flags and build rules
#*
#*
#*******************************************************************************
#

#
# Macro definitions referenced below
#
empty =
space =$(empty) $(empty)
CC = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)gcc
AC = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)as
ARIN = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)ar
LD = $(TOOLCHAIN_PATH_A15)/bin/$(CROSS_TOOL_PRFX)gcc
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR)))) -I$(TOOLCHAIN_PATH_A15)/include
OBJEXT = oa15fg
AOBJEXT = sa15fg
CFLAGS_INTERNAL = -mno-unaligned-access -c -mtune=cortex-a15 -marm -DDRA7xx -gstrict-dwarf -Wall -D__ARMv7 -D_LITTLE_ENDIAN=1 -mcpu=cortex-a15 -mfpu=neon -mfloat-abi=hard -mabi=aapcs -g
ASFLAGS_INTERNAL =  -mcpu=cortex-a15 -mfpu=neon -mfloat-abi=hard
ARFLAGS_INTERNAL = cr
LNKFLAGS_INTERNAL = 
INTERNALDEFS = -MD -MF $@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)/am572x/armv7/obj/soc

#List the COMMONSRC Files
COMMONSRCC= \
    src/v1/cpsw_ethdriver.c\
    src/v1/cpsw_impl.c\
    src/v1/cpsw_nimu_eth.c\
    soc/am572x/NIMU_soc.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -DSOC_AM572x  -I./src -I. 

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo cla15fg $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(CFLAGS_INTERNAL) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) $< -o $@

#Create Empty rule for dependency
$(COMMONSRCCOBJS):.\build\am572x\armv7\ti.transport.ndk.nimu.am572x.aa15fg.mk
.\build\am572x\armv7\ti.transport.ndk.nimu.am572x.aa15fg.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/am572x/armv7/ti.transport.ndk.nimu.am572x.aa15fg : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/am572x/armv7 ]; then $(MKDIR) $(LIBDIR)/am572x/armv7 ; fi;
	$(ARIN) $(ARFLAGS_INTERNAL) $@ $?
