@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building CPPI LLD
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment
@REM *
@REM * Copyright (C) 2011, Texas Instruments, Inc.
@REM *****************************************************************************

@echo ---------------------------------------
@echo Configuring CPPI LLD Build Environment
@echo off

IF DEFINED PARTNO GOTO partno_defined
@REM Configure the Part Number
set PARTNO=keystone2
:partno_Defined

IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
set PDK_INSTALL_PATH=../../..
:pdk_defined

@REM ---------------------------------
@REM Enabling MINI PACKAGE to be Built
@REM ---------------------------------
set MINI_PACKAGE=OFF
@echo MINI PACKAGE is set to %MINI_PACKAGE%

@REM ---------------------------------
@REM Enabling debug flags
@REM ---------------------------------
set EXTDBGFLAGS=-mn -g

@REM This is the base location for the various tools. 
set XDCCGROOT=T:/c6xx/cgen7_3_02/c6000/cgtools
set C6X_GEN_INSTALL_PATH=T:\c6xx\cgen7_3_02\c6000\cgtools

REM *******************************************************************************
REM *************************** XDC PATH Configuration ****************************
REM *******************************************************************************
REM CPPI LLD depends upon the following packages:-
REM     - CSL Package for the CSL Register Layer
REM     - QMSS LLD
REM     - BIOS Package
REM These packages should be installed before trying to build the driver else 
REM compilations will fail.
REM PDK Package: CSL, CPPI and QMSS are a part of the PDK package.

@REM Specify the XDC Tool Path
set XDC_INSTALL_PATH=T:/gen/xdc/xdctools_3_22_04_46
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Configure the XDCPATH
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;%C6X_GEN_INSTALL_PATH%/include

@REM Eclipse Help Plugin (Not required by customers)
set XDC_ECLIPSE_PLUGIN_INSTALL_PATH=T:/gen/xdc/xdc_eclipse_plugin_gen/20091203
set XDC_FILTER_INSTALL_PATH=T:/gen/xdc/xdcFilter/20100428
set XDCPATH=%XDCPATH%;%XDC_ECLIPSE_PLUGIN_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%XDC_FILTER_INSTALL_PATH%

@REM Configure the paths to ensure that the XDC is available.
set PATH=%XDC_INSTALL_PATH%;%XDC_INSTALL_PATH%\bin;T:\Doxygen\doxygen\1.5.1-p1\bin

@REM Third Party Tools: Install-Jammer (Not required by customers)
set PATH=%PATH%;T:\gen\InstallJammer\v1_2_05

set PATH=%PATH%;%XDCCGROOT%\bin;T:\gen\gnu\99-11-01\cygwin-b20\H-i586-cygwin32\bin
set PATH=%PATH%;T:\SDOApps\cg_xml\cg_xml_v2_30_00\bin
set PATH=%PATH%;T:\ti_pdsp_cgen\20091120

@REM Third Party Tools: HTML Help compiler.
set PATH=%PATH%;T:\Doxygen\HTML_Help_Workshop\10-01-2007

@REM Environment Variable which control STATIC Analysis of the code
set STATIC_ANALYZE_PATH=T:\gen\coverity\prevent-mingw-3.4.0
set PATH=%PATH%;%STATIC_ANALYZE_PATH%\bin

@REM Set the Title Window appropriately.
Title CPPI LLD Build Environment

@echo CPPI LLD Build Environment Configured 
@echo ---------------------------------------

