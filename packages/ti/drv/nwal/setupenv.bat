@REM ******************************************************************************
@REM * FILE PURPOSE: Environment Setup for building PDK
@REM ******************************************************************************
@REM * FILE NAME: setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment for PDK. 

@REM *  The batch file expects an optional argument:PDK_INSTALL_PATH: Location of the PDK package.
@REM *   If the argument is not specified the  batch file assumes that the PDK is installed in the same location 
@REM *   where the batch file is located and is being executed.
@REM *
@REM * USAGE:
@REM *  setupenv.bat "C:/Program Files/Texas Instruments/pdk_C6678_1_0_0_17/packages" 
@REM *      --- OR ---
@REM *  setupenv.bat
@REM *
@REM * Copyright (C) 2011, Texas Instruments, Inc.
@REM *****************************************************************************
@echo off
@REM *******************************************************************************
@REM ********************** GET PARAMETERS PASSED THROUGH ARGUMENT   ***************
@REM *******************************************************************************
@REM Parameter Validation: Check if the argument was passed to the batch file and
@REM if so we use that else we default to the working directory where the batch 
@REM file was invoked from

IF DEFINED PDK_INSTALL_PATH  GOTO endparameter
set tempVar=%1
IF NOT DEFINED tempVar GOTO noparameter
set PDK_INSTALL_PATH=%~fs1
goto done1
:noparameter
set PDK_INSTALL_PATH=%~sdp0
:done1

:endparameter

@REM *******************************************************************************
@REM ********************** CHECK REQUIRED ENVIRONMENT DEFINES BEGIN ***************
@REM *******************************************************************************
if not defined C6X_GEN_INSTALL_PATH      goto HLP_C6X_GEN_INSTALL_PATH
set C6X_GEN_INSTALL_PATH=%C6X_GEN_INSTALL_PATH:\=/%

if not defined PKTLIB_INSTALL_PATH      goto HLP_PKTLIB_INSTALL_PATH

if not defined XDC_INSTALL_PATH  @echo "XDC_INSTALL_PATH: XDC NOT CONFIGURED!!!!. REQUIRED FOR XDC BUILD"
if not defined SA_LLD_INSTALL_PATH   @echo "SA_LLD_INSTALL_PATH:  NOT CONFIGURED!!!!. REQUIRED FOR NWAL Library with SA Support"


@REM *******************************************************************************
@REM ********************** CHECK REQUIRED ENVIRONMENT DEFINES END ***************
@REM *******************************************************************************


@REM *******************************************************************************
@REM *************************** OPTIONAL ENVIRONMENT DEFINES **********************
@REM *************************** NOT REQUIRED FOR BUILDING THE PDK *****************
@REM *******************************************************************************
@REM COVERITY_INSTALL_PATH: Environment Variable for tool to do STATIC Analysis 
@REM                      of the code
@REM 
@REM DOXYGEN_INSTALL_PATH: DOXYGEN Version 1.7.3 [Only needed for generating Doxygen]
@REM 
@REM HTML_HELP_WORKSHOP_INSTALL_PATH: HTML Help compiler.[Only needed for generating Doxygen]
@REM *******************************************************************************
@REM *******************************************************************************
@REM *******************************************************************************

@REM PDK PARTNO
set PARTNO=TCI6608

set CGTOOLS=%C6X_GEN_INSTALL_PATH%

@REM *******************************************************************************
@REM ** Convert C6X_GEN_INSTALL_PATH and PDK_INSTALL_PATH to short name and to UNIX STYLE PATH  for XDC build **
@REM *******************************************************************************
if not defined XDC_INSTALL_PATH goto END_SHORT_CONVERSION
set PATH=%PATH%;%XDC_INSTALL_PATH%\packages\xdc\services\io\release
@REM for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %PDK_INSTALL_PATH%') do set PDK_INSTALL_PATH=%%a 
@REM for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %C6X_GEN_INSTALL_PATH%') do set C6X_GEN_INSTALL_PATH=%%a 

for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %PDK_INSTALL_PATH%') do set PDK_INSTALL_PATH=%%a

for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %C6X_GEN_INSTALL_PATH%') do set C6X_GEN_INSTALL_PATH=%%a

:END_SHORT_CONVERSION
if not defined C6X_GEN_INSTALL_PATH   set C6X_GEN_INSTALL_PATH=%C6X_GEN_INSTALL_PATH:\=/%

set XDCCGROOT=%C6X_GEN_INSTALL_PATH%
echo CGTOOL INSTALL Directory %C6X_GEN_INSTALL_PATH%
echo PDK Directory %PDK_INSTALL_PATH%
echo SA LLD Directory %SA_LLD_INSTALL_PATH%
echo XDC Directory %XDC_INSTALL_PATH%
echo DEVICE=%DEVICE%
@echo off
@REM *******************************************************************************
@REM *************************** XDC PATH Configuration ****************************
@REM *******************************************************************************

@REM Ensure that all the components inside PDK are a part of the XDC Path; such that
@REM one component can use another.
set XDCPATH=../../..;%PDK_INSTALL_PATH%;%PKTLIB_INSTALL_PATH%;%XDC_INSTALL_PATH%/packages;%C6X_GEN_INSTALL_PATH%/include

@REM Eclipse Help Plugin (Not required by customers)
if defined XDC_ECLIPSE_PLUGIN_INSTALL_PATH set XDCPATH=%XDCPATH%;%XDC_ECLIPSE_PLUGIN_INSTALL_PATH%

if defined XDC_FILTER_INSTALL_PATH set XDCPATH=%XDCPATH%;%XDC_FILTER_INSTALL_PATH%

if defined SA_LLD_INSTALL_PATH set XDCPATH=%XDCPATH%;%SA_LLD_INSTALL_PATH%

set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%
@REM *******************************************************************************
@REM ************************** Build Tools Configuration **************************
@REM *******************************************************************************

@REM Windows Path
set PATH=C:\Windows\System32

@REM XDC Tools location:
set PATH=%PATH%;%XDC_INSTALL_PATH%;%XDC_INSTALL_PATH%\bin

@REM Compiler Tools: 
set PATH=%PATH%;%C6X_GEN_INSTALL_PATH%\bin

@REM CG-XML Package:
set PATH=%PATH%;%CG_XML_BIN_INSTALL_PATH%

@REM Third Party Tools: Doxygen
if defined DOXYGEN_INSTALL_PATH  set PATH=%PATH%;%DOXYGEN_INSTALL_PATH%

@REM Third Party Tools: HTML Help compiler.
if defined HTML_HELP_WORKSHOP_INSTALL_PATH  set PATH=%PATH%;%HTML_HELP_WORKSHOP_INSTALL_PATH%;

@REM Third Party Tools: Install-Jammer (Not required by customers)
if defined INSTALL_JAMMER_DIR set PATH=%PATH%;%INSTALL_JAMMER_DIR%

@REM Third Party Tools: Coverity 
if defined COVERITY_INSTALL_PATH set PATH=%PATH%;%COVERITY_INSTALL_PATH%\bin

@REM Set the Title Window appropiately.
Title NWAL Build Environment

goto SUCCESS

:HLP_C6X_GEN_INSTALL_PATH
@echo "ENVIRONMENT VARIABLE C6X_GEN_INSTALL_PATH: Code Generation Tool NOT CONFIGURED!!!!"
@echo Example [NOTE ""]:set C6X_GEN_INSTALL_PATH="c:/Program Files/Texas Instruments/C6000 Code Generation Tools 7.2.2"
goto ERROR

:HLP_PKTLIB_INSTALL_PATH
@echo "ENVIRONMENT VARIABLE PKTLIB_INSTALL_PATH: PacketLib NOT CONFIGURED!!!!"
@echo Example [NOTE ""]:set PKTLIB_INSTALL_PATH="c:/Program Files/Texas Instruments"
goto ERROR

:HLP_XDC_INSTALL_PATH
@echo "ENVIRONMENT VARIABLE XDC_INSTALL_PATH: XDC NOT CONFIGURED!!!!"
@echo XDC_INSTALL_PATH EXAMPLE [DEFAULT LOCATION]:
@echo  set XDC_INSTALL_PATH=C:\Program Files\Texas Instruments\xdctools_3_20_07_86

:HLP_CG_XML_BIN_INSTALL_PATH
@echo "ENVIRONMENT VARIABLE CG_XML_BIN_INSTALL_PATH: Code Generation Tools XML package NOT CONFIGURED!!!!"
@echo CG_XML_BIN_INSTALL_PATH EXAMPLE [DEFAULT LOCATION]:
@echo  set CG_XML_BIN_INSTALL_PATH=C:\Program Files\Texas Instruments\cg_xml\bin
goto ERROR

:SUCCESS
@echo NWAL BUILD ENVIRONMENT CONFIGURED
@echo *******************************************************************************
goto DONE

:ERROR
@echo ERROR CONFIGURING NWAL BUILD ENVIRONMENT
@echo *******************************************************************************

:DONE
