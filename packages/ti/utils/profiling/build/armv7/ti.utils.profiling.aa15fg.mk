#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: ./build/armv7/ti.utils.profiling.aa15fg.mk
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
CFLAGS_INTERNAL = -mno-unaligned-access -c -mtune=cortex-a15 -marm -DDRA7xx -g -gdwarf-3 -gstrict-dwarf -Wall -D__ARMv7 -D_LITTLE_ENDIAN=1 -finstrument-functions -gdwarf-3 -g -D_ENABLE_BM -mcpu=cortex-a15 -mfpu=neon -mfloat-abi=hard -mabi=aapcs -g
ASFLAGS_INTERNAL =  -mcpu=cortex-a15 -mfpu=neon -mfloat-abi=hard
ARFLAGS_INTERNAL = cr
LNKFLAGS_INTERNAL = 
INTERNALDEFS = -MD -MF $@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)/armv7/obj

#List the COMMONSRC Files
COMMONSRCC= \
    src/profilingHooksA15.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -I./src -I. 

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo cla15fg $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(CFLAGS_INTERNAL) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) $< -o $@

#Create Empty rule for dependency
$(COMMONSRCCOBJS):.\build\armv7\ti.utils.profiling.aa15fg.mk
.\build\armv7\ti.utils.profiling.aa15fg.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/armv7/ti.utils.profiling.aa15fg : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/armv7 ]; then $(MKDIR) $(LIBDIR)/armv7 ; fi;
	$(ARIN) $(ARFLAGS_INTERNAL) $@ $?
