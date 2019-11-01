@REM ******************************************************************************
@REM * FILE PURPOSE: NWAL Unit Test and Example Project Creator
@REM ******************************************************************************
@REM * FILE NAME: nwalProjectCreate.bat
@REM *
@REM * DESCRIPTION: 
@REM *  The script file is used to create the test and example projects of all
@REM *  components under NWAL. These projects are available in the specified 
@REM *  workspace.
@REM *
@REM * USAGE:
@REM *  nwalProjectCreate.bat 
@REM *      --- OR ---
@REM * Description:     (first option is default)
@REM *  socFamily  -   k1/k2k / k2h
@REM *  endian      -   little / big
@REM *
@REM * Example:
@REM *  a) nwalProjectCreate.bat
@REM *              - Creates all projects for Keystone-1 devices for little endian
@REM *  b) nwalProjectCreate.bat k2h
@REM *              - Creates all module projects for k2h device for little endian
@REM *
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM *****************************************************************************

@echo OFF

REM Argument [pdkDir] is used to set PDK_SHORT_NAME. This is PDK directory
REM where project description files are located. If there is no value passed,
REM it defaults to the file location directory. Then convert the variable to 
REM short name to avoid issues using batch file commands.
set tempVar4=%4
if not defined tempVar4 goto noparameter
set NWAL_SHORT_NAME=%tempVar4%
goto done
:noparameter
set NWAL_SHORT_NAME=%~sdp0
:done
REM *****************************************************************************

REM Parameter Validation: Check if the argument was passed to the batch file and
REM *****************************************************************************
REM Argument [socFamily] is used to set DEVICE_NAME variable.
REM Valid values are  'k2h'. Defaults to 'none Keystone-1'.
set tempVar1=%1
if not defined tempVar1 goto nodevice
set DEVICE_NAME=%tempVar1%

goto devicedone
:nodevice
set DEVICE_NAME=k1

:devicedone

if %DEVICE_NAME% == k1 (
set MACROS_FILE=%NWAL_SHORT_NAME%\macros.ini
REM RTSC Platform Name
set RTSC_PLATFORM_NAME=ti.platforms.evm6678
REM PDK Part Number
set PDK_PARTNO=C6678L
set PDK_VER=v2
REM Version of the PDK
set PDK_VERSION=1.0.0.21
REM Version of the SALLD
set SA_VERSION=1.0.5.2
) else ( 
set MACROS_FILE=%NWAL_SHORT_NAME%\macros_k2.ini
set PDK_PARTNO=TCI6634
set RTSC_PLATFORM_NAME=ti.platforms.simKepler
set PDK_VER=v3
REM Version of the PDK
set PDK_VERSION=1.0.0.11
REM Version of the SALLD
set SA_VERSION=2.0.0.4
)

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


echo =========================================================================
echo.   DEVICE_NAME     :   %DEVICE_NAME%
echo.   ENDIAN          :   %ENDIAN%
echo =========================================================================

REM *****************************************************************************
REM * Version Information of the various tools etc required to build the test
REM * projects. Customers are free to modify these to meet their requirements.
REM *****************************************************************************

REM This is to control the CCS version specific project create command
REM Set to 'no' when using CCSv5 or set to 'yes' when using CCSv4
set IS_CCS_VERSION_4=no

REM Set to 'no' when using QT, EVM, VDB, or other hardware. Set to 'yes' only when using the simulator.
set IS_SIMULATOR_SUPPORT_NEEDED=no

REM Install Location for CCS
set CCS_INSTALL_PATH="C:\ti\ccsv5"

REM Workspace where the NWAL projects will be created.
REM set MY_WORKSPACE="c:\MyNWALWorkspaceC6678-k1-LE"
set MY_WORKSPACE="c:\MyNWALWorkspace-k1-LE"
REM macros.ini location

REM This is the format of the executable being created
REM Valid Values are 'ELF' and 'COFF'
set OUTPUT_FORMAT=ELF

REM Version of CG-Tools
set CGT_VERSION=7.4.2

REM Version of XDC
set XDC_VERSION=3.23.03.53

REM Version of BIOS
set BIOS_VERSION=6.34.02.18

REM Version of the IPC
set IPC_VERSION=1.22.03.23



REM RTSC Target 
REM - Please ensure that you select this taking into account the
REM   OUTPUT_FORMAT and the RTSC_PLATFORM_NAME 
if %ENDIAN% == big (
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

REM Goto the NWAL Installation Path.
pushd %NWAL_SHORT_NAME%

echo *****************************************************************************
echo Detecting UnitTest Projects in NWAL and importing them in the workspace %MY_WORKSPACE%

REM Search for all the test Project Files in the NWAL.
REM for /F %%I IN ('dir /b /s *%SOC_FAMILY%*testproject.txt') do (
for /F %%I IN ('dir /b /s *%DEVICE_NAME%*testproject.txt') do (

echo Detected Test Project: %%~nI

REM Goto each directory where the test project file is located and create the projects.
pushd %%~dI%%~pI

REM Execute the command to create the project using the parameters specified above.
if %DEVICE_NAME% == k1 (
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.IPC:%IPC_VERSION%;com.ti.rtsc.SYSBIOS:%BIOS_VERSION%;com.ti.biosmcsdk.pdk.%PDK_PARTNO%:%PDK_VERSION%;ti.drv.sa:%SA_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a -ccs.args %%~nI%%~xI %SIMULATOR_SUPPORT_DEFINE% -ccs.setBuildOption com.ti.rtsc.*.XDC_PATH "${PKTLIB_INSTALL_PATH}" -ccs.setBuildOption com.ti.rtsc.*.XDC_PATH "${NWAL_INSTALL_PATH}" -ccs.setBuildOption com.ti.rtsc.*.XDC_PATH "${SA_INSTALL_PATH}"
) else ( 
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.IPC:%IPC_VERSION%;com.ti.rtsc.SYSBIOS:%BIOS_VERSION%;ti.pdk:%PDK_VERSION%;ti.drv.sa:%SA_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a -ccs.args %%~nI%%~xI %SIMULATOR_SUPPORT_DEFINE% -ccs.setBuildOption com.ti.rtsc.*.XDC_PATH "${PKTLIB_INSTALL_PATH}" -ccs.setBuildOption com.ti.rtsc.*.XDC_PATH "${NWAL_INSTALL_PATH}" -ccs.setBuildOption com.ti.rtsc.*.XDC_PATH "${SA_INSTALL_PATH}"
)

echo Copying macro.ini
copy %MACROS_FILE% %MY_WORKSPACE%\%%~nI\macros.ini

popd
)

popd

