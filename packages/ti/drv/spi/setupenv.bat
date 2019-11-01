@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building LLD
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment
@REM *
@REM * Copyright (C) 2015, Texas Instruments, Inc.
@REM *****************************************************************************

@echo ---------------------------------------
@echo Configuring LLD Build Environment
@echo off

IF DEFINED PARTNO GOTO partno_defined
@REM Configure the Part Number
set PARTNO=am57xx
:partno_Defined

IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
@REM set PDK_INSTALL_PATH=c:/ti/pdk_am57xx_8_00_00_00/packages
set PDK_INSTALL_PATH=c:/ti/pdk_am57xx_8_00_00_00/packages
:pdk_defined

@REM This is the base location for the various tools. 
set C6X_GEN_INSTALL_PATH=T:/c6xx/cgen7_3_02/c6000/cgtools
set TOOLCHAIN_PATH_M4=c:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2
set TOOLCHAIN_PATH_A15=c:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_8-2014q3
set CROSS_TOOL_PRFX=arm-none-eabi-

REM *******************************************************************************
REM *************************** XDC PATH Configuration ****************************
REM *******************************************************************************
REM SPI LLD depends upon the following packages:-
REM     - CSL Package for the CSL SPI Functional Layer
REM These packages should be installed before trying to build the driver else 
REM compilations will fail.
REM PDK Package: CSL is a part of the PDK package.

@REM Specify the XDC Tool Path
@REM set XDC_INSTALL_PATH=T:/gen/xdc/xdctools_3_22_04_46
set XDC_INSTALL_PATH=C:/ti/xdctools_3_30_06_67
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Configure the XDCPATH
@REM set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;%C6X_GEN_INSTALL_PATH%/include;
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;

@REM Eclipse Help Plugin (Not required by customers)
set XDC_ECLIPSE_PLUGIN_INSTALL_PATH=T:/gen/xdc/xdc_eclipse_plugin_gen/20091203
set XDC_FILTER_INSTALL_PATH=T:/gen/xdc/xdcFilter/20100428
set XDCPATH=%XDCPATH%;%XDC_ECLIPSE_PLUGIN_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%XDC_FILTER_INSTALL_PATH%

@REM Configure the paths to ensure that the XDC is available.
set PATH=%XDC_INSTALL_PATH%;%XDC_INSTALL_PATH%\bin;T:\Doxygen\doxygen\1.5.1-p1\bin;

@REM Third Party Tools: Install-Jammer (Not required by customers)
set PATH=%PATH%;T:\gen\InstallJammer\v1_2_05

set PATH=%PATH%;T:\gen\gnu\99-11-01\cygwin-b20\H-i586-cygwin32\bin
set PATH=%PATH%;T:\SDOApps\cg_xml\cg_xml_v2_30_00\bin
set PATH=%PATH%;%C6X_GEN_INSTALL_PATH%\bin
set PATH=%PATH%;%TOOLCHAIN_PATH_M4%\bin
set PATH=%PATH%;%TOOLCHAIN_PATH_A15%\bin

@REM Third Party Tools: HTML Help compiler.
set PATH=%PATH%;T:\Doxygen\HTML_Help_Workshop\10-01-2007

@REM Environment Variable which control STATIC Analysis of the code
set STATIC_ANALYZE_PATH=T:\gen\coverity\prevent-mingw-3.4.0
set PATH=%PATH%;%STATIC_ANALYZE_PATH%\bin

@REM Set the Title Window appropriately.
Title LLD Build Environment

@echo LLD Build Environment Configured 
@echo -----------------------------------------------

