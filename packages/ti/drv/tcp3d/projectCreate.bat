@REM ******************************************************************************
@REM * FILE PURPOSE: Unit Test and Example Project Creator
@REM ******************************************************************************
@REM * FILE NAME: projectCreate.bat
@REM *
@REM * DESCRIPTION: 
@REM *  The script file is used to create the test and example projects of TCP3D.
@REM *  These projects are available in the specified workspace.
@REM *
@REM * Syntax:
@REM *  projectCreate.bat [deviceName] [endian]
@REM *
@REM * Description:     (first option is default)
@REM *  deviceName  -   k2k / k2h / k2l
@REM *  endian      -   little / big
@REM *
@REM * Example:
@REM *  a) projectCreate.bat
@REM *              - Creates all module projects for k2h device for little endian
@REM *  b) projectCreate.bat k2h
@REM *              - Creates all module projects for k2h device for little endian
@REM *  c) projectCreate.bat k2k big
@REM *              - Creates all module projects for k2k device for big endian
@REM *
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM *****************************************************************************
@echo OFF

REM Parameter Validation: Check if the argument was passed to the batch file.
REM *****************************************************************************
REM Argument [deviceName] is used to set DEVICE_NAME variable.
REM Valid values are 'k2k', 'k2h', 'k2l'. Defaults to 'k2h'.
set tempVar1=%1
if not defined tempVar1 goto nodevice
set DEVICE_NAME=%tempVar1%
goto devicedone
:nodevice
set DEVICE_NAME=k2h
:devicedone

REM Argument [endian] is used to set ENDIAN variable.
REM This is Endianess of the Projects being created.
REM Valid Values are 'little' and 'big'. Defaults to 'little'.
set tempVar2=%2
if not defined tempVar2 goto littleendian
if %tempVar2% == big (
set ENDIAN=big
goto endiandone
) 
:littleendian
set ENDIAN=little
:endiandone

REM This is specific module for which Projects are created.
set MODULE=tcp3d

REM Batch file execution location
set WORKDIR_SHORT=%~sdp0

REM *****************************************************************************

echo =========================================================================
echo.   DEVICE_NAME     :   %DEVICE_NAME%
echo.   ENDIAN          :   %ENDIAN%
echo.   MODULE          :   %MODULE%
echo.   WORKDIR_SHORT   :   %WORKDIR_SHORT%
echo =========================================================================

REM *****************************************************************************
REM * Version Information of the various tools etc required to build the test
REM * projects. Customers are free to modify these to meet their requirements.
REM *****************************************************************************

REM This is to control the CCS version specific project create command
REM Set to 'no' when using CCSv5 or set to 'yes' when using CCSv4
set IS_CCS_VERSION_4=no

REM Set to 'no' when using QT, EVM, VDB, or other hardware. Set to 'yes' only when using the simulator.
set IS_SIMULATOR_SUPPORT_NEEDED=yes

REM Install Location for CCS
set CCS_INSTALL_PATH="C:\ti\ccsv5"

REM Workspace where the projects will be created in "example" and "test" folders.
set MY_WORKSPACE=.

REM macros.ini location
set MACROS_FILE=%WORKDIR_SHORT%test\macros.ini

REM This is the format of the executable being created
REM Valid Values are 'ELF' and 'COFF'
set OUTPUT_FORMAT=ELF

REM Version of CG-Tools
set CGT_VERSION=7.4.4

REM Version of XDC
set XDC_VERSION=3.25.05.94

REM Version of BIOS
set BIOS_VERSION=6.37.00.20

REM Version of the IPC
set IPC_VERSION=3.22.00.04

REM EDMA3 Version
set EDMA_VERSION=02.11.11.15

REM Version of the PDK
set PDK_VERSION=3.01.00.01

REM PDK Part Number & Platform name
if %DEVICE_NAME% == k2k (
set PDK_PARTNO=TCI6638
set RTSC_PLATFORM_NAME=ti.platforms.evmTCI6638K2K
) else if  %DEVICE_NAME% == k2h (
set PDK_PARTNO=TCI6636
set RTSC_PLATFORM_NAME=ti.platforms.evmTCI6636K2H
) else if  %DEVICE_NAME% == k2l (
set PDK_PARTNO=TCI6630
set RTSC_PLATFORM_NAME=ti.platforms.evmTCI6630K2L
REM Temporarily using simKepler 
REM set RTSC_PLATFORM_NAME=ti.platforms.simKepler
) else (
set PDK_PARTNO=TCI6634
set RTSC_PLATFORM_NAME=ti.platforms.simKepler
)

REM RTSC Target 
REM - Please ensure that you select this taking into account the
REM   OUTPUT_FORMAT and the RTSC_PLATFORM_NAME 
if "%ENDIAN%" == "big" (
set RTSC_TARGET=ti.targets.elf.C66_big_endian
) else ( 
set RTSC_TARGET=ti.targets.elf.C66
)

REM *****************************************************************************
REM *****************************************************************************
REM                 Please do NOT change anything below this
REM *****************************************************************************
REM *****************************************************************************

REM Set auto create command by default for use with CCSv5
set AUTO_CREATE_COMMAND=eclipse\eclipsec -noSplash 

REM If is CCS version 4 then set auto create command for use with CCSv4
If .%IS_CCS_VERSION_4% == .yes set AUTO_CREATE_COMMAND=eclipse\jre\bin\java -jar %CCS_INSTALL_PATH%\eclipse\startup.jar

REM Set project for Silicon or QT by default
set SIMULATOR_SUPPORT_DEFINE=

REM If simulator support is needed then set the define
If .%IS_SIMULATOR_SUPPORT_NEEDED% == .yes set SIMULATOR_SUPPORT_DEFINE=-ccs.setCompilerOptions "--define SIMULATOR_SUPPORT"

echo *****************************************************************************
echo Detecting UnitTest Projects in TCP3D and importing them in the workspace %MY_WORKSPACE%

REM Search for all the test Project Files in the TCP3D.
for /F %%I IN ('dir /b /s *%DEVICE_NAME%*testproject.txt') do (
echo Detected Test Project: %%~nI

REM Goto each directory where the test project file is located and create the projects.
pushd test

REM Execute the command to create the project using the parameters specified above.
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI_%ENDIAN% -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.IPC:%IPC_VERSION%;com.ti.rtsc.SYSBIOS:%BIOS_VERSION%;ti.pdk:%PDK_VERSION%;com.ti.sdo.edma3:%EDMA_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a -ccs.args %%I %SIMULATOR_SUPPORT_DEFINE% -ccs.overwrite full

REM copy the macros.ini to project location
echo Copying macro.ini
copy %MACROS_FILE% %MY_WORKSPACE%\%%~nI_%ENDIAN%\macros.ini

popd
)

echo *****************************************************************************
echo Detecting Example Projects in TCP3D and importing them in the workspace %MY_WORKSPACE%

REM Search for all the Example Project Files in the TCP3D.
for /F %%I IN ('dir /b /s *%DEVICE_NAME%*exampleproject.txt') do (
echo Detected Example Project: %%~nI

REM Goto each directory where the example project file is located and create the projects.
pushd example

REM Execute the command to create the project using the parameters specified above.
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI_%ENDIAN% -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.IPC:%IPC_VERSION%;com.ti.rtsc.SYSBIOS:%BIOS_VERSION%;ti.pdk:%PDK_VERSION%;com.ti.sdo.edma3:%EDMA_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a -ccs.args %%I %SIMULATOR_SUPPORT_DEFINE% -ccs.overwrite full

REM copy the macros.ini to project location
echo Copying macro.ini
copy %MACROS_FILE% %MY_WORKSPACE%\%%~nI_%ENDIAN%\macros.ini

popd
)
