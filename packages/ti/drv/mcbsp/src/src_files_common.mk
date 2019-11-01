
SRCDIR = . src
INCDIR = . src
# Common source files across all platforms and cores
SRCS_COMMON += mcbsp_drv.c mcbsp_ioctl.c mcbsp_edma.c

PACKAGE_SRCS_COMMON = makefile mcbsp_component.mk mcbsp_osal.h \
                      docs/ReleaseNotes_MCBSP_LLD.pdf \
                      mcbsp_drv.h include/mcbsp_pvt.h \
		      mcbsp_types.h \
                      build/makefile.mk src/src_files_common.mk

