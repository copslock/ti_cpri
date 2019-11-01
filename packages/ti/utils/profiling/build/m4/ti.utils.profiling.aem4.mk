#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: ./build/m4/ti.utils.profiling.aem4.mk
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
CC = $(TOOLCHAIN_PATH_M4)/bin/armcl -c
AC = $(TOOLCHAIN_PATH_M4)/bin/armcl -c
ARIN = $(TOOLCHAIN_PATH_M4)/bin/armar
LD = $(TOOLCHAIN_PATH_M4)/bin/armlnk
RTSLIB = -l $(TOOLCHAIN_PATH_M4)/lib/undefined
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR)))) -I$(TOOLCHAIN_PATH_M4)/include
OBJEXT = oem4
AOBJEXT = sem4
CFLAGS_INTERNAL = -qq -pdsw255 -DMAKEFILE_BUILD --entry_parm=address --exit_hook=ti_utils_exit --exit_parm=address --entry_hook=ti_utils_entry -g -D_ENABLE_BM --endian=little -mv7M4 --float_support=vfplib --abi=eabi
ASFLAGS_INTERNAL = -qq --endian=little -mv7M4 --float_support=vfplib --abi=eabi
ARFLAGS_INTERNAL = rq
LNKFLAGS_INTERNAL = --silicon_version=7M4 --strict_compatibility=on
INTERNALDEFS = -Dti_targets_arm_elf_M4  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)/m4/obj

#List the COMMONSRC Files
COMMONSRCC= \
    src/profilingHooksM4.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -I./src -I. 

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo clem4 $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(CFLAGS_INTERNAL) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) -fc $< 
	-@$(CP) $@.dep $@.pp; \
         $(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
             -e '/^$$/ d' -e 's/$$/ :/' < $@.pp >> $@.dep; \
         $(RM) $@.pp 

#Create Empty rule for dependency
$(COMMONSRCCOBJS):.\build\m4\ti.utils.profiling.aem4.mk
.\build\m4\ti.utils.profiling.aem4.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/m4/ti.utils.profiling.aem4 : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/m4 ]; then $(MKDIR) $(LIBDIR)/m4 ; fi;
	$(ARIN) $(ARFLAGS_INTERNAL) $@ $?
