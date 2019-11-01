@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building BCP Driver
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment
@REM *
@REM * Copyright (C) 2010, Texas Instruments, Inc.
@REM *****************************************************************************

@echo ------------------------------------------------
@echo Configuring BCP Driver Build Environment
@echo off

REM set the PDK install path
IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
set PDK_INSTALL_PATH="c:\ti\mcsdk_3_1_3_6\pdk_keystone2_3_01_03_06\packages"
rem set PDK_INSTALL_PATH=../../..
:pdk_defined

REM Setup the part number for the driver build
set PARTNO=keystone2

REM This is the base location for the various tools. 
REM set XDCCGROOT=c:/ti/ccsv5/tools/compiler/c6000
REM set C6X_GEN_INSTALL_PATH=%XDCCGROOT:/=\%
REM set C6X_GEN_INSTALL_PATH="C:\ti\ccsv5\tools\compiler\c6000_7.4.4"
set C6X_GEN_INSTALL_PATH="C:\Program Files (x86)\Texas Instruments\C6000 Code Generation Tools 7.4.8"
set XDCCGROOT=%C6X_GEN_INSTALL_PATH%

REM BCP Driver depends upon the following packages:-
REM     - CSL Package for the BCP CSL Register Layer
REM     - CPPI & QMSS LLD
REM These packages should be installed before trying to build the driver else 
REM compilations will fail.

@REM Specify the XDC Tool Path
set XDC_INSTALL_PATH="C:/ti/mcsdk_3_1_3_6/xdctools_3_30_05_60"
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Configure the XDCPATH
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;%C6X_GEN_INSTALL_PATH%/include

REM Eclipse Help Plugin (Not required by customers)
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

REM Set the Title Window appropiately.
Title BCP Driver Build Environment

@echo BCP Driver Build Environment Configured 
@echo -----------------------------------------------

