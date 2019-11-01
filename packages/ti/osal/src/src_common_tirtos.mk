
SRCDIR = . src/tirtos arch/core
INCDIR = . src/tirtos

SRCS_COMMON += Core_utils.c SemaphoreP_tirtos.c HwiP_tirtos.c SwiP_tirtos.c Utils_tirtos.c CacheP_tirtos.c RegisterIntr_tirtos.c EventCombinerP_tirtos.c Queue_tirtos.c EventP_tirtos.c TaskP_tirtos.c

ifeq ($(SOC),$(filter $(SOC), am574x am572x am571x k2g k2l k2e k2h k2k c6678 c6657 am437x am335x omapl137 omapl138 c6747 am65xx j721e))
SRCS_COMMON += TimerP_tirtos.c
endif

ifeq ($(ISA),$(filter $(ISA), a53 a72 c7x))
  SRCDIR += soc/$(SOC)
  SRCS_COMMON += bios_mmu.c
endif

PACKAGE_SRCS_COMMON = makefile HwiP.h SwiP.h MuxIntcP.h osal.h osal_component.mk SemaphoreP.h \
                      arch/core src/tirtos src/src_common_nonos.mk  src/src_common_tirtos.mk \
                      docs/OSAL_SoftwareManifest.html docs/ReleaseNotes_OSAL.pdf \
                      build/makefile_nonos_indp.mk build/makefile_nonos.mk build/makefile_tirtos_indp.mk build/makefile_tirtos.mk

