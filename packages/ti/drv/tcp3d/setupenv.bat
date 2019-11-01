@REM ***************************************************************************
@REM * FILE PURPOSE: Environment Setup for building TCP3D driver
@REM ***************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment
@REM *
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM ***************************************************************************

@echo ------------------------------------------------
@echo Configuring TCP3D Driver Build Environment
@echo off

REM Set CCS Installation Root directory
REM ============================================================================
if exist "c:\ti" (
set CCS_ROOT="c:/ti"
) else (
echo.   ********************************************
echo.   CCS_ROOT is not defined, check the script
echo.   ********************************************
)
REM Print message about the CCS base path detected
echo.   ********************************************
echo.   Detected CCS_ROOT is set to %CCS_ROOT%
echo.   ********************************************
REM ============================================================================
REM Get XDC utililty path and set to path to use 'path2dos'
for /f "tokens=1* delims=" %%a in ('dir /b %CCS_ROOT:/=\%\MCSDK_3A7\xdctools_3_24*') do (
set XDC_UTIL_PATH=%CCS_ROOT:/=\%\%%a\packages\xdc\services\io\release
)
set PATH=%PATH%;%XDC_UTIL_PATH%
set XDC_UTIL_PATH=

REM Covert variables for short path
for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %CCS_ROOT%') do (set CCS_ROOT=%%a)
REM ============================================================================


REM set the PDK install path
IF DEFINED PDK_INSTALL_PATH GOTO pdk_defined
set PDK_INSTALL_PATH="%CCS_ROOT%/pdk_keystone2_3_01_00_01/packages"
:pdk_defined

@REM EDMA3 LLD installation path variables
set EDMA3LLD_BIOS6_INSTALLDIR=%CCS_ROOT%/edma3_lld_02_11_11_15

@REM set the Code Gen tools
set C6X_GEN_INSTALL_PATH=%CCS_ROOT%/ccsv5/tools/compiler/c6000_7.4.4
set XDCCGROOT=%C6X_GEN_INSTALL_PATH%

@REM Specify the XDC Tool Path
set XDC_INSTALL_PATH="C:/ti/xdctools_3_25_05_94"
set XDCPATH=../../..;%XDC_INSTALL_PATH%/packages

@REM Third Party Tools: Coverity 
set STATIC_ANALYZE_PATH=T:\gen\coverity\prevent-mingw-3.8.0
set COVERITY_INSTALL_PATH=%STATIC_ANALYZE_PATH:\=/%

@REM Eclipse Help Plugin (Not required by customers)
set XDC_ECLIPSE_PLUGIN_INSTALL_PATH=T:/gen/xdc/xdc_eclipse_plugin_gen/20091203

@REM XDC filete for creating simple makefile
set XDC_FILTER_INSTALL_PATH=T:/gen/xdc/xdcFilter/20100428
set XDCPATH=%XDCPATH%;%XDC_ECLIPSE_PLUGIN_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%XDC_FILTER_INSTALL_PATH%


@REM set the CG XML path
set CG_XML_BIN_INSTALL_PATH=%CCS_ROOT%/cg_xml/bin

@REM Third Party Tools: Doxygen
set DOXYGEN_INSTALL_PATH=T:\Doxygen\doxygen\1.5.1-p1\bin

@REM Third Party Tools: Install-Jammer (Not required by customers)
set INSTALL_JAMMER_DIR=T:\gen\InstallJammer\v1_2_05

@REM Third Party Tools: HTML Help compiler.
set HTML_HELP_WORKSHOP_INSTALL_PATH=T:\Doxygen\HTML_Help_Workshop\10-01-2007

@REM PDK Environment Setup:
@call %PDK_INSTALL_PATH%\pdksetupenv.bat

@REM set XDCPATH with necessary packages
set XDCPATH=%XDCPATH%;%COVERITY_INSTALL_PATH%

@echo TCP3D Driver Build Environment Configured 
@echo ------------------------------------------------
