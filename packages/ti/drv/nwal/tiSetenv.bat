@REM ******************************************************************************
@REM * FILE PURPOSE: Wrapper for setting PDK setup environment
@REM ******************************************************************************
@REM * FILE NAME: tiSetenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment for NWAL based on internal TI 
@REM *  tool chain locations
@REM *
@REM * Copyright (C) 2011 Texas Instruments, Inc.
@REM *****************************************************************************
@echo off
REM This is the base location for the various tools. 

@REM CODE GEN TOOL LOCATION. TO BE PROVIDED BY USER
REM set C6X_GEN_INSTALL_PATH="C:/Program Files/Texas Instruments/C6000 Code Generation Tools 7.2.4"
set C6X_GEN_INSTALL_PATH=t:\c6xx\cgen7_2_04\c6000\cgtools

@REM XDC Tools location: Optional only needed for gmake. For msys or cygwin build not needed
set XDC_INSTALL_PATH=t:/gen/xdc/xdctools_3_22_01_21


REM  In the case of Linux based build only two steps required
REM export PDK_INSTALL_PATH=$PWD
REM export C6X_GEN_INSTALL_PATH="c:/Program Files/Texas Instruments/ccsv5/C6000 Code Generation Tools 7.2.4"

@REM Environment Variable which control STATIC Analysis of the code
set STATIC_ANALYZE_PATH=T:\gen\coverity\prevent-mingw-3.8.0

set XDC_ECLIPSE_PLUGIN_INSTALL_PATH=T:/gen/xdc/xdc_eclipse_plugin_gen/20091203

set CG_XML_BIN_INSTALL_PATH=T:\SDOApps\cg_xml\cg_xml_v2_20_00\bin
set DOXYGEN_INSTALL_PATH=T:\Doxygen\doxygen\1.5.1-p1\bin
set HTML_HELP_WORKSHOP_INSTALL_PATH=T:\Doxygen\HTML_Help_Workshop\10-01-2007
set INSTALL_JAMMER_DIR=T:\gen\InstallJammer\v1_2_05
set SA_LLD_INSTALL_PATH=c:/ti/salld_01_00_05_04/packages
REM set PDK_INSTALL_PATH="c:/Program Files/Texas Instruments/pdk_C6678_1_0_0_17/packages"
REM set PDK_INSTALL_PATH="C:/Program Files/Texas Instruments/pdk_TCI6614_1_0_0_1/packages"
set PDK_INSTALL_PATH="c:/ti/pdk_C6678_2_1_3_7/packages"
set PKTLIB_INSTALL_PATH=c:/ti/pdk_C6678_2_1_3_7/packages

:SUCCESS
@echo on
@echo NWAL Environment Configured for TI Shared Server
@echo -----------------------------------------------
@echo off
goto DONE

:ERROR
@echo on
@echo Error configuring NWAL Environment  
@echo -----------------------------------------------

:DONE
