@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building Transport BMET Ethernet Library
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment for Trace Framework BMET Lib
@REM *
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM *****************************************************************************

@echo ---------------------------------------
@echo Configuring Trace Framework Bmet Lib Build Environment
@echo off

IF DEFINED PARTNO GOTO partno_defined
@REM Configure the Part Number
set PARTNO=TCI6614
:partno_Defined

IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
set PDK_INSTALL_PATH=C:\ti\pdk_tci6614_1_00_00_09\packages
:pdk_defined

IF DEFINED BIOS_INSTALL_PATH GOTO bios_defined
set BIOS_INSTALL_PATH=C:\ti\bios_6_33_04_39\packages
:bios_defined

@REM This is the base location for the various tools. 
set C6X_GEN_INSTALL_PATH=C:/ti/ccsv5/tools/compiler/c6000
set XDCCGROOT=%C6X_GEN_INSTALL_PATH%
:codegen_defined

@REM This is the base location for the UIA . 
set UIA_INSTALL_PATH=C:/ti/uia_1_00_04_35
:uia_defined

@REM This is the base location for the UIA PLUS . 
set UIAPLUS_INSTALL_PATH=C:\ti\uiaplus_1_00_00_02_eng
:uiaplus defined


REM *******************************************************************************
REM *************************** XDC PATH Configuration ****************************
REM *******************************************************************************
REM Trace Framework BMET library depends upon the following packages:-
REM     - None

@REM Specify the XDC Tool Path
set XDC_INSTALL_PATH=C:\ti\xdctools_3_23_02_47
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Configure the XDCPATH
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%;%BIOS_INSTALL_PATH%;%C6X_GEN_INSTALL_PATH%/include;%UIA_INSTALL_PATH%/packages;%UIAPLUS_INSTALL_PATH%/packages;

@REM Configure the paths to ensure that the XDC is available (Setting Doxygen path for internal use only).
set PATH=%XDC_INSTALL_PATH%;%XDC_INSTALL_PATH%\bin;T:\Doxygen\doxygen\1.5.1-p1\bin

set PATH=%PATH%;%XDCCGROOT%\bin; (Setting cgxml path for internal use only)
set PATH=%PATH%;T:\SDOApps\cg_xml\cg_xml_v2_20_00\bin

@REM Third Party Tools: HTML Help compiler.(Setting HTML Help Workshop path for internal use only)
set PATH=%PATH%;T:\Doxygen\HTML_Help_Workshop\10-01-2007

@REM Set the Title Window appropriately.
Title Trace Framework BMET Library Build Environment

@echo Trace Framework BMET Lib Build Environment Configured 
@echo ---------------------------------------
@echo Summary:
@echo PDK PATH     = %PDK_INSTALL_PATH%
@echo BIOS PATH    = %BIOS_INSTALL_PATH%
@echo XDC PATH     = %XDC_INSTALL_PATH%
@echo CGTOOLS Path = %C6X_GEN_INSTALL_PATH%
@echo UIA_INSTALL_PATH = %UIA_INSTALL_PATH%
@echo UIAPLUS_INSTALL_PATH =%UIAPLUS_INSTALL_PATH%
@echo Note: Please set the paths accordingly and re run this batch file if the paths are not correct
@echo ---------------------------------------

