#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: ./build/c66/ti.drv.dfe.ae66.mk
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
CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -mo -g -mn -o1 -ms2 -k -eo.o --verbose_diagnostics --display_error_number --diag_error=225 --diag_error=9 --diag_warning=179 --diag_remark=880 --diag_remark=188 --mem_model:data=far -mv6600 --abi=eabi
AC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c -qq -mv6600 --abi=eabi
ARIN = $(C6X_GEN_INSTALL_PATH)/bin/ar6x rq
LD = $(C6X_GEN_INSTALL_PATH)/bin/lnk6x --abi=eabi
RTSLIB = -l $(C6X_GEN_INSTALL_PATH)/lib/undefined
INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))
OBJEXT = oe66
AOBJEXT = se66
INTERNALDEFS = -Dti_targets_elf_C66  -DMAKEFILE_BUILD -eo.$(OBJEXT) -ea.$(AOBJEXT) -fr=$(@D) -fs=$(@D) -ppa -ppd=$@.dep
INTERNALLINKDEFS = -o $@ -m $@.map
OBJDIR =  $(LIBDIR)/obj

#List the COMMONSRC Files
COMMONSRCC= \
    src/dfefl/dfe_fl_Close.c\
    src/dfefl/dfe_fl_Init.c\
    src/dfefl/dfe_fl_Open.c\
    src/dfefl/dfe_fl_autocpClose.c\
    src/dfefl/dfe_fl_autocppGetHwStatus.c\
    src/dfefl/dfe_fl_autocppHwControl.c\
    src/dfefl/dfe_fl_autocppOpen.c\
    src/dfefl/dfe_fl_bbClose.c\
    src/dfefl/dfe_fl_bbGetHwStatus.c\
    src/dfefl/dfe_fl_bbHwControl.c\
    src/dfefl/dfe_fl_bbOpen.c\
    src/dfefl/dfe_fl_cbClose.c\
    src/dfefl/dfe_fl_cbGetHwStatus.c\
    src/dfefl/dfe_fl_cbHwControl.c\
    src/dfefl/dfe_fl_cbOpen.c\
    src/dfefl/dfe_fl_cdfrClose.c\
    src/dfefl/dfe_fl_cdfrGetHwStatus.c\
    src/dfefl/dfe_fl_cdfrHwControl.c\
    src/dfefl/dfe_fl_cdfrOpen.c\
    src/dfefl/dfe_fl_cfrClose.c\
    src/dfefl/dfe_fl_cfrGetHwStatus.c\
    src/dfefl/dfe_fl_cfrHwControl.c\
    src/dfefl/dfe_fl_cfrOpen.c\
    src/dfefl/dfe_fl_cppDescripClose.c\
    src/dfefl/dfe_fl_cppDescripControl.c\
    src/dfefl/dfe_fl_cppDescripOpen.c\
    src/dfefl/dfe_fl_cppDescripStatus.c\
    src/dfefl/dfe_fl_cppDmaClose.c\
    src/dfefl/dfe_fl_cppDmaControl.c\
    src/dfefl/dfe_fl_cppDmaOpen.c\
    src/dfefl/dfe_fl_cppDmaStatus.c\
    src/dfefl/dfe_fl_dducClose.c\
    src/dfefl/dfe_fl_dducGetHwStatus.c\
    src/dfefl/dfe_fl_dducHwControl.c\
    src/dfefl/dfe_fl_dducOpen.c\
    src/dfefl/dfe_fl_dpdClose.c\
    src/dfefl/dfe_fl_dpdGetHwStatus.c\
    src/dfefl/dfe_fl_dpdHwControl.c\
    src/dfefl/dfe_fl_dpdOpen.c\
    src/dfefl/dfe_fl_dpdaClose.c\
    src/dfefl/dfe_fl_dpdaGetHwStatus.c\
    src/dfefl/dfe_fl_dpdaHwControl.c\
    src/dfefl/dfe_fl_dpdaOpen.c\
    src/dfefl/dfe_fl_fbClose.c\
    src/dfefl/dfe_fl_fbGetHwStatus.c\
    src/dfefl/dfe_fl_fbHwControl.c\
    src/dfefl/dfe_fl_fbOpen.c\
    src/dfefl/dfe_fl_jesdClose.c\
    src/dfefl/dfe_fl_jesdGetHwStatus.c\
    src/dfefl/dfe_fl_jesdHwControl.c\
    src/dfefl/dfe_fl_jesdOpen.c\
    src/dfefl/dfe_fl_miscClose.c\
    src/dfefl/dfe_fl_miscGetHwStatus.c\
    src/dfefl/dfe_fl_miscHwControl.c\
    src/dfefl/dfe_fl_miscOpen.c\
    src/dfefl/dfe_fl_rxClose.c\
    src/dfefl/dfe_fl_rxGetHwStatus.c\
    src/dfefl/dfe_fl_rxHwControl.c\
    src/dfefl/dfe_fl_rxOpen.c\
    src/dfefl/dfe_fl_summerClose.c\
    src/dfefl/dfe_fl_summerGetHwStatus.c\
    src/dfefl/dfe_fl_summerHwControl.c\
    src/dfefl/dfe_fl_summerOpen.c\
    src/dfefl/dfe_fl_txClose.c\
    src/dfefl/dfe_fl_txGetHwStatus.c\
    src/dfefl/dfe_fl_txHwControl.c\
    src/dfefl/dfe_fl_txOpen.c\
    src/dfelld/DFE_bb.c\
    src/dfelld/DFE_cb.c\
    src/dfelld/DFE_cfr.c\
    src/dfelld/DFE_dduc.c\
    src/dfelld/DFE_dpd.c\
    src/dfelld/DFE_dpda.c\
    src/dfelld/DFE_device.c\
    src/dfelld/DFE_excep.c\
    src/dfelld/DFE_fb.c\
    src/dfelld/DFE_jesd.c\
    src/dfelld/DFE_misc.c\
    src/dfelld/DFE_open.c\
    src/dfelld/DFE_rx.c\
    src/dfelld/DFE_summer.c\
    src/dfelld/DFE_sync.c\
    src/dfelld/DFE_tx.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS =   -DDEVICE_K2L   

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(OBJDIR)/%.$(OBJEXT): %.c
	-@echo cle66 $< ...
	if [ ! -d $(@D) ]; then $(MKDIR) $(@D) ; fi;
	$(RM) $@.dep
	$(CC) $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS) -fc $< 
	-@$(CP) $@.dep $@.pp; \
         $(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
             -e '/^$$/ d' -e 's/$$/ :/' < $@.pp >> $@.dep; \
         $(RM) $@.pp 

#Create Empty rule for dependency
$(COMMONSRCCOBJS):./build/c66/ti.drv.dfe.ae66.mk
./build/c66/ti.drv.dfe.ae66.mk:

#Include Depedency for COMMONSRC Files
ifneq (clean,$(MAKECMDGOALS))
 -include $(COMMONSRCCOBJS:%.$(OBJEXT)=%.$(OBJEXT).dep)
endif


$(LIBDIR)/c66/ti.drv.dfe.ae66 : $(COMMONSRCCOBJS)
	@echo archiving $? into $@ ...
	if [ ! -d $(LIBDIR)/c66 ]; then $(MKDIR) $(LIBDIR)/c66 ; fi;
	$(ARIN) $@ $?
