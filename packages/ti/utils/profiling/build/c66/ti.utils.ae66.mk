#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: ./build/c66/ti.utils.ae66.mk
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
CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c
AC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c
ARIN = $(C6X_GEN_INSTALL_PATH)/bin/ar6x
LD = $(C6X_GEN_INSTALL_PATH)/bin/lnk6x
RTSLIB = -l $(C6X_GEN_INSTALL_PATH)/lib/undefined
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR)))) -I$(C6X_GEN_INSTALL_PATH)/include
OBJEXT = oe66
AOBJEXT = se66
CFLAGS_INTERNAL = -mo -q -k -eo.o --entry_parm=address --exit_hook=ti_utils_exit --exit_parm=address --entry_hook=ti_utils_entry -g -D_ENABLE_BM -mv6600 --abi=eabi
ASFLAGS_INTERNAL = -qq -mv6600 --abi=eabi
ARFLAGS_INTERNAL = rq
LNKFLAGS_INTERNAL = --abi=eabi
INTERNALDEFS = -Dti_targets_elf_C66  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)/c66/obj

#List the COMMONSRC Files
COMMONSRCC= \
    src/profilingHooksC66.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -I./src -I. 

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo cle66 $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(CFLAGS_INTERNAL) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) -fc $< 
	-@$(CP) $@.dep $@.pp; \
         $(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
             -e '/^$$/ d' -e 's/$$/ :/' < $@.pp >> $@.dep; \
         $(RM) $@.pp 

#Create Empty rule for dependency
$(COMMONSRCCOBJS):.\build\c66\ti.utils.ae66.mk
.\build\c66\ti.utils.ae66.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/c66/ti.utils.ae66 : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/c66 ]; then $(MKDIR) $(LIBDIR)/c66 ; fi;
	$(ARIN) $(ARFLAGS_INTERNAL) $@ $?
