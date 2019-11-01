@REM ******************************************************************************
@REM * FILE PURPOSE: SA Unit Test and Example Project Creator
@REM ******************************************************************************
@REM * FILE NAME: uartProjectCreate.bat
@REM *
@REM * DESCRIPTION:
@REM *  The script file is used to create the test and example projects of all
@REM *  components under SA. These projects are available in the specified
@REM *  workspace.
@REM *
@REM * Syntax:
@REM *  uartProjectCreate.bat [deviceName] [endian] [module] [pdkDir]
@REM *
@REM * Description:     (first option is default)
@REM *  deviceName  -   /k2l /k2e /k2k /k2h/am57x
@REM *  endian      -   little / big
@REM *  module      -   uart
@REM *  pdkDir      -   THIS FILE LOCATION / "C:\ti\pdk_keystone2_3_01_00_00\packages"
@REM *
@REM * Example:
@REM *  a) uartProjectCreate.bat
@REM *              - Creates uart module projects for k2k device for little endian
@REM *  b) uartProjectCreate.bat k2l
@REM *              - Creates uart module projects for k2l device for little endian
@REM *  c) uartProjectCreate.bat k2k big
@REM *              - Creates uart module projects for k2k device for big endian
@REM *
@REM * Copyright (C) 2012-2013, Texas Instruments, Inc.
@REM *****************************************************************************
rem @echo OFF

REM Parameter Validation: Check if the argument was passed to the batch file.
REM *****************************************************************************
REM Argument [deviceName] is used to set DEVICE_NAME variable.
REM Valid values are 'k2k', 'k2h' or 'k2l'. Defaults to 'am57x'.
set tempVar1=%1
if not defined tempVar1 goto nodevice
set DEVICE_NAME=%tempVar1%
goto devicedone
:nodevice
set DEVICE_NAME=am57x
:devicedone

set EDMA3LLD_BIOS6_INSTALLDIR="C:/ti/edma3_lld_02_12_00_20"
set PDK_INSTALL_PATH=C:/ti/pdk_keystone2_3_01_00_03/packages
set BIOS_INSTALL_PATH=C:/ti/bios_6_37_03_30
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

REM Argument [module] is used to set MODULE variable.
REM This is specific module for which Projects are created.
REM Valid Values are LLD name. Defaults to all LLD's.
set tempVar3=%3
if not defined tempVar3 goto nomodule
set MODULE=%tempVar3%
goto moduledone
:nomodule
set MODULE=uart
:moduledone

REM Argument [pdkDir] is used to set PDK_SHORT_NAME. This is PDK directory
REM where project description files are located. If there is no value passed,
REM it defaults to the file location directory. Then convert the variable to
REM short name to avoid issues using batch file commands.
set tempVar4=%4
if not defined tempVar4 goto noparameter
set PDK_SHORT_NAME=%tempVar4%
goto done
:noparameter
set PDK_SHORT_NAME=%~sdp0
:done

REM *****************************************************************************

echo =========================================================================
echo.   DEVICE_NAME     :   %DEVICE_NAME%
echo.   ENDIAN          :   %ENDIAN%
echo.   MODULE          :   %MODULE%
echo.   PDK_SHORT_NAME  :   %PDK_SHORT_NAME%
echo =========================================================================


REM *****************************************************************************
REM *****************************************************************************
REM                         Customer Modifiable Section
REM *****************************************************************************
REM *****************************************************************************

REM macros.ini location
set MACROS_FILE=%PDK_SHORT_NAME%\macros.ini

REM This is to control the CCS version specific project create command
REM Set to 'no' when using CCSv5 or set to 'yes' when using CCSv4
set IS_CCS_VERSION_4=no

REM Set to 'no' when using QT, EVM, VDB, or other hardware. Set to 'yes' only when using the simulator.
set IS_SIMULATOR_SUPPORT_NEEDED=no

REM Install Location for CCSv5.4. Ensure the PATH here is in compliance with the 'IS_CCS_VERSION_4' variable
REM defined above.
set CCS_INSTALL_PATH="C:\ti_6_0\ccsv6"

REM Workspace where the PDK projects will be created.
set MY_WORKSPACE="workspace_pcie"

REM macros.ini location
set MACROS_FILE=%PDK_SHORT_NAME%\macros.ini

REM This is the format of the executable being created
REM Valid Values are 'ELF' and 'COFF'
set OUTPUT_FORMAT=ELF

REM Version of CG-Tools
set CGT_VERSION=7.4.7

REM Version of XDC
set XDC_VERSION=3.25.06.96

REM Version of BIOS
set BIOS_VERSION=6.37.03.30


REM Version of the PDK
set PDK_VERSION=3.01.00.01

REM PDK Part Number
set PDK_PARTNO=TCI6634

REM Version of the SALLD
set UART_VERSION=3.0.0.4

REM RTSC Platform Name
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
) else if  %DEVICE_NAME% == k2e (
set PDK_PARTNO=66AK2E05
set RTSC_PLATFORM_NAME=ti.platforms.evmC66AK2E
REM  Temporarily using simKepler
REM set RTSC_PLATFORM_NAME=ti.platforms.simKepler
)else if  %DEVICE_NAME% == am57x (
set PDK_PARTNO=AM57X
set RTSC_PLATFORM_NAME=ti.platforms.evmDRA7XX
)else (
set PDK_PARTNO=TCI6634
set RTSC_PLATFORM_NAME=ti.platforms.simKepler
)

REM RTSC Target
REM - Please ensure that you select this taking into account the
REM   OUTPUT_FORMAT and the RTSC_PLATFORM_NAME
if %ENDIAN% == big (
set RTSC_TARGET=gnu.targets.arm.A15F
) else (
set RTSC_TARGET=gnu.targets.arm.A15F
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
set SIMULATOR_SUPPORT_DEFINE=yes

REM If simulator support is needed then set the define
If .%IS_SIMULATOR_SUPPORT_NEEDED% == .yes set SIMULATOR_SUPPORT_DEFINE=-ccs.setCompilerOptions "--define SIMULATOR_SUPPORT"

REM Goto the PDK Installation Path.
pushd %PDK_SHORT_NAME%

echo *****************************************************************************
echo Detecting UnitTest Projects in UART LLD package and importing them in the workspace %MY_WORKSPACE%

REM Search for all the test Project Files in UART.
for /F %%I IN ('dir /b /s *%DEVICE_NAME%*armtestproject.txt') do (
echo Detected Test Project: %%~nI

REM Goto each directory where the test project file is located and create the projects.
pushd %%~dI%%~pI

REM Execute the command to create the project using the parameters specified above.

%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device "Cortex A.DRA75x_DRA74x" -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion "GNU_4.7.4:Linaro" -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.SYSBIOS:%BIOS_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a libgcc.a libm.a libnosys.a -ccs.args %%~nI%%~xI %SIMULATOR_SUPPORT_DEFINE% -ccs.setCompilerOptions
echo Copying macro.ini
copy %MACROS_FILE% %MY_WORKSPACE%\%%~nI\macros.ini

popd
)

echo *****************************************************************************
echo Detecting Example Projects in UART LLD package and importing them in the workspace %MY_WORKSPACE%

REM Search for all the Example Project Files in the PDK.
for /F %%I IN ('dir /b /s *%DEVICE_NAME%*armexampleproject.txt') do (
echo Detected Example Project: %%~nI

REM Goto each directory where the example project file is located and create the projects.
pushd %%~dI%%~pI

REM Execute the command to create the project using the parameters specified above.
echo on
echo
echo foo
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device "Cortex A.DRA75x_DRA74x" -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion "GNU_4.7.4:Linaro" -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.SYSBIOS:%BIOS_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a libgcc.a libm.a libnosys.a -ccs.args %%~nI%%~xI %SIMULATOR_SUPPORT_DEFINE% -ccs.setCompilerOptions

echo Copying macro.ini
copy %MACROS_FILE% %MY_WORKSPACE%\%%~nI\macros.ini

popd
)

popd



