@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building MCBSP LLD
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment
@REM *
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM *****************************************************************************

@echo ---------------------------------------
@echo Configuring MCBSP LLD Build Environment
@echo off

IF DEFINED PARTNO GOTO partno_defined
@REM Configure the Part Number
set PARTNO=K2G
:partno_Defined

IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
set PDK_INSTALL_PATH=C:\ti3\pdk_keystone2_3_02_00_00\packages
:pdk_defined

@REM ---------------------------------
@REM Enabling MINI PACKAGE to be Built
@REM ---------------------------------
set MINI_PACKAGE=OFF
@echo MINI PACKAGE is set to %MINI_PACKAGE%

@REM This is the base location for the various tools. 
set XDCCGROOT=T:\c6xx\cgen7_4_02\c6000\cgtools
set C6X_GEN_INSTALL_PATH=T:\c6xx\cgen7_4_02\c6000\cgtools
set EDMA3_INSTALL_PATH=C:\ti\edma3_lld_02_11_09_08\packages

@REM EDMA3 LLD installation path variables
@REM set EDMA3LLD_BIOS6_INSTALLDIR=C:/ti/edma3_lld_02_11_05_02

REM *******************************************************************************
REM *************************** XDC PATH Configuration ****************************
REM *******************************************************************************
REM MCBSP LLD depends upon the following packages:-
REM     - CSL Package for the CSL Register Layer
REM     - EDMA3 LLD
REM     - RM LLD
REM     - BIOS Package
REM These packages should be installed before trying to build the driver else 
REM compilations will fail.
REM CSL/LLDs Package: CSL, MCBSP and RM are a part of the CSL/LLDs package.

@REM Specify the XDC Tool Path
set XDC_INSTALL_PATH=T:/gen/xdc/xdctools_3_24_06_63
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Configure the XDCPATH
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;%C6X_GEN_INSTALL_PATH%/include;%EDMA3_INSTALL_PATH%

@REM Eclipse Help Plugin (Not required by customers)
set XDC_ECLIPSE_PLUGIN_INSTALL_PATH=T:/gen/xdc/xdc_eclipse_plugin_gen/20091203
set XDC_FILTER_INSTALL_PATH=T:/gen/xdc/xdcFilter/20100428
set XDCPATH=%XDCPATH%;%XDC_ECLIPSE_PLUGIN_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%XDC_FILTER_INSTALL_PATH%

@REM Configure the paths to ensure that the XDC is available.
set PATH=%PATH%;%XDC_INSTALL_PATH%;%XDC_INSTALL_PATH%\bin;T:\Doxygen\doxygen\1.5.1-p1\bin

@REM Third Party Tools: Install-Jammer (Not required by customers)
set PATH=%PATH%;T:\gen\InstallJammer\v1_3_0

set PATH=%PATH%;%XDCCGROOT%\bin;T:\gen\gnu\99-11-01\cygwin-b20\H-i586-cygwin32\bin
set PATH=%PATH%;T:\SDOApps\cg_xml\cg_xml_v2_30_00\bin

@REM Third Party Tools: HTML Help compiler.
set PATH=%PATH%;T:\Doxygen\HTML_Help_Workshop\10-01-2007

@REM Environment Variable which control STATIC Analysis of the code
set STATIC_ANALYZE_PATH=T:\gen\coverity\prevent-mingw-3.4.0
set PATH=%PATH%;%STATIC_ANALYZE_PATH%\bin

@REM Set the Title Window appropriately.
Title MCBSP LLD Build Environment

@echo MCBSP LLD Build Environment Configured 
@echo ---------------------------------------
