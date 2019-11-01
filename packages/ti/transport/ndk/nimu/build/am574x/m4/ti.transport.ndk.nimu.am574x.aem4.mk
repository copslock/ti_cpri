#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: ./build/am574x/m4/ti.transport.ndk.nimu.am574x.aem4.mk
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
CFLAGS_INTERNAL = -o4 -qq -pdsw255 -DMAKEFILE_BUILD --endian=little -mv7M4 --float_support=vfplib --abi=eabi
ASFLAGS_INTERNAL = -qq --endian=little -mv7M4 --float_support=vfplib --abi=eabi
ARFLAGS_INTERNAL = rq
LNKFLAGS_INTERNAL = --silicon_version=7M4 --strict_compatibility=on
INTERNALDEFS = -Dti_targets_arm_elf_M4  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)/am574x/m4/obj/soc

#List the COMMONSRC Files
COMMONSRCC= \
    src/v1/cpsw_ethdriver.c\
    src/v1/cpsw_impl.c\
    src/v1/cpsw_nimu_eth.c\
    soc/am574x/NIMU_soc.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -DSOC_AM574x  -I./src -I. 

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
$(COMMONSRCCOBJS):.\build\am574x\m4\ti.transport.ndk.nimu.am574x.aem4.mk
.\build\am574x\m4\ti.transport.ndk.nimu.am574x.aem4.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/am574x/m4/ti.transport.ndk.nimu.am574x.aem4 : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/am574x/m4 ]; then $(MKDIR) $(LIBDIR)/am574x/m4 ; fi;
	$(ARIN) $(ARFLAGS_INTERNAL) $@ $?
