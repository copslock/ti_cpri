@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building FFTC Driver
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment
@REM *
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM *****************************************************************************

@echo ------------------------------------------------
@echo Configuring FFTC Driver Build Environment
@echo off

IF DEFINED PARTNO GOTO partno_defined
@REM Configure the Part Number
set PARTNO=keystone2
:partno_Defined

IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
rem set PDK_INSTALL_PATH="c:\ti\pdk_keystone2_3_01_00_00\packages"
set PDK_INSTALL_PATH="c:/ti/pdk_keystone2_3_01_00_01/packages"
rem set PDK_INSTALL_PATH=../../..
:pdk_defined

@REM ---------------------------------
@REM Enabling MINI PACKAGE to be Built
@REM ---------------------------------
set MINI_PACKAGE=OFF
@echo MINI PACKAGE is set to %MINI_PACKAGE%

@REM This is the base location for the various tools. 
REM set XDCCGROOT=c:/ti/ccsv5/tools/compiler/c6000
REM set C6X_GEN_INSTALL_PATH=%XDCCGROOT:/=\%
set C6X_GEN_INSTALL_PATH="c:/ti/ccsv5/tools/compiler/c6000_7.4.4"
set XDCCGROOT=%C6X_GEN_INSTALL_PATH%

REM *******************************************************************************
REM *************************** XDC PATH Configuration ****************************
REM *******************************************************************************
REM FFTC Driver depends upon the following packages:-
REM     - CSL Package for the FFTC CSL Register Layer
REM     - CPPI & QMSS LLD
REM These packages should be installed before trying to build the driver else 
REM compilations will fail.
REM PDK Package: CSL, CPPI and QMSS are a part of the PDK package.

@REM Specify the XDC Tool Path
set XDC_INSTALL_PATH="c:/ti/xdctools_3_25_05_94"
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Configure the XDCPATH
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;%C6X_GEN_INSTALL_PATH%/include

@REM Eclipse Help Plugin (Not required by customers)
set XDC_ECLIPSE_PLUGIN_INSTALL_PATH=T:/gen/xdc/xdc_eclipse_plugin_gen/20091203
set XDC_FILTER_INSTALL_PATH=T:/gen/xdc/xdcFilter/20100428
set XDCPATH=%XDCPATH%;%XDC_ECLIPSE_PLUGIN_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%XDC_FILTER_INSTALL_PATH%

REM Ensure following build tools are available in the path:
REM     -   XDC tools 
REM     -   Compiler toolchain (Code Generation Tools)
REM     -   Doxygen to generate documentation from code
REM     -   HTML Help workshop (required by Doxygen for documentation)
REM     -   InstallJammer for building an executable 
REM     -   CG-XML for generating size etc meta-info for builds
set PATH=%XDC_INSTALL_PATH%;%XDC_INSTALL_PATH%\bin
set PATH=%PATH%;%XDCCGROOT%\bin;C:\Windows\system32
set PATH=%PATH%;T:\Doxygen\doxygen\1.5.1-p1\bin
set PATH=%PATH%;T:\Doxygen\HTML_Help_Workshop\10-01-2007
set PATH=%PATH%;T:\gen\InstallJammer\v1_2_05
set PATH=%PATH%;T:\SDOApps\cg_xml\cg_xml_v2_30_00\bin

REM Set the Coverity Path where the Coverity Tools are located.
REM Required only if static analysis needs to be run.
set STATIC_ANALYZE_PATH=T:\gen\coverity\prevent-mingw-3.4.0
set PATH=%PATH%;%STATIC_ANALYZE_PATH%\bin

REM By default, turn off the Static Analysis.
REM Set to ON for running static analysis when needed.
set STATIC_ANALYZE=OFF

@REM Set the Title Window appropriately.
Title FFTC Driver Build Environment

@echo FFTC Driver Build Environment Configured 
@echo -----------------------------------------------
