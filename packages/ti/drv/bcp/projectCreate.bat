@REM ******************************************************************************
@REM * FILE PURPOSE: Unit Test and Example Project Creator
@REM ******************************************************************************
@REM * FILE NAME: projectCreate.bat
@REM *
@REM * DESCRIPTION: 
@REM *  The script file is used to create the test and example projects for
@REM *  TCP3D. These projects are available in the specified workspace.
@REM *
@REM * USAGE:
@REM *  projectCreate.bat big
@REM *      --- OR ---
@REM *  projectCreate.bat
@REM *
@REM * DEPENDENCIES:
@REM *  "C:\Program Files\Texas Instruments\pdk_C6657_1_0_0_1001\packages"
@REM *
@REM * Copyright (C) 2011, Texas Instruments, Inc.
@REM *****************************************************************************
@echo OFF

REM skip checking as done for PDK
@call setupenv.bat

REM *****************************************************************************
REM *****************************************************************************
REM                         Customer Modifiable Section
REM *****************************************************************************
REM *****************************************************************************

REM This is to control the CCS version specific project create command
REM Set to 'no' when using CCSv5 or set to 'yes' when using CCSv4
set IS_CCS_VERSION_4=no

REM Set to 'no' when using QT, EVM, VDB, or other hardware. Set to 'yes' only when using the simulator.
set IS_SIMULATOR_SUPPORT_NEEDED=yes

REM Install Location for CCS. Ensure the PATH here is in compliance with the 'IS_CCS_VERSION_4' variable
REM defined above.
set CCS_INSTALL_PATH="c:\ti\ccsv5"

REM Workspace where the projects will be created in "example" and "test" folders.
set MY_WORKSPACE=.

REM This is Endianess of the Projects being created.
REM Valid Values are 'little' and 'big'
if "%1" == "big" (
set ENDIAN=big
) else (
set ENDIAN=little
)

REM This is the format of the executable being created
REM Valid Values are 'ELF' and 'COFF'
set OUTPUT_FORMAT=ELF

REM Version of CG-Tools
set CGT_VERSION=7.4.8

REM Version of XDC
set XDC_VERSION=3.30.05.60

REM Version of BIOS
set BIOS_VERSION=6.40.04.47

REM Version of the IPC
set IPC_VERSION=3.30.01.12

REM EDMA3 Version
set EDMA_VERSION=02.11.13.17

REM Version of the PDK
set PDK_VERSION=3.01.03.06

REM PDK Part Number
set PDK_PARTNO=TCI6634

REM RTSC Platform Name
set RTSC_PLATFORM_NAME=ti.platforms.simKepler

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

REM Batch file execution location
set WORKDIR_SHORT=%~sdp0

REM Set auto create command by default for use with CCSv5
set AUTO_CREATE_COMMAND=eclipse\eclipsec -noSplash 

REM If is CCS version 4 then set auto create command for use with CCSv4
If .%IS_CCS_VERSION_4% == .yes set AUTO_CREATE_COMMAND=eclipse\jre\bin\java -jar %CCS_INSTALL_PATH%\eclipse\startup.jar

REM Set project for Silicon or QT by default
set SIMULATOR_SUPPORT_DEFINE=

REM If simulator support is needed then set the define
If .%IS_SIMULATOR_SUPPORT_NEEDED% == .yes set SIMULATOR_SUPPORT_DEFINE=-ccs.setCompilerOptions "--define SIMULATOR_SUPPORT"

echo *****************************************************************************
echo Detecting UnitTest Projects in PDK and importing them in the workspace %MY_WORKSPACE%

set listFile=testpjtlist.txt
dir /b /s *testproject.txt | findstr "bcp" > %listFile%
REM Search for all the test Project Files in the PDK.
for /F %%I IN (%listFile%) do (
echo Detected Test Project: %%~nI

REM Goto each directory where the test project file is located and create the projects.
pushd test

REM Execute the command to create the project using the parameters specified above.
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI_%ENDIAN% -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.IPC:%IPC_VERSION%;com.ti.rtsc.SYSBIOS:%BIOS_VERSION%;ti.pdk:%PDK_VERSION%;com.ti.sdo.edma3:%EDMA_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a -ccs.args %%I %SIMULATOR_SUPPORT_DEFINE% -ccs.overwrite full

REM copy the macros.ini to project location
copy %WORKDIR_SHORT%test\macros.ini %MY_WORKSPACE%\%%~nI_%ENDIAN%\macros.ini

popd
)
@del /q %listFile%

echo *****************************************************************************
echo Detecting Example Projects in PDK and importing them in the workspace %MY_WORKSPACE%

set listFile=examplepjtlist.txt
dir /b /s *exampleproject.txt | findstr "bcp" > %listFile%
REM Search for all the Example Project Files in the PDK.
for /F %%I IN (%listFile%) do (
echo Detected Example Project: %%~nI

REM Goto each directory where the example project file is located and create the projects.
pushd example

REM Execute the command to create the project using the parameters specified above.
%CCS_INSTALL_PATH%\%AUTO_CREATE_COMMAND% -data %MY_WORKSPACE% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI_%ENDIAN% -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.products "com.ti.rtsc.IPC:%IPC_VERSION%;com.ti.rtsc.SYSBIOS:%BIOS_VERSION%;ti.pdk:%PDK_VERSION%;com.ti.sdo.edma3:%EDMA_VERSION%" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -ccs.rts libc.a -ccs.args %%I %SIMULATOR_SUPPORT_DEFINE% -ccs.overwrite full

REM copy the macros.ini to project location
copy %WORKDIR_SHORT%test\macros.ini %MY_WORKSPACE%\%%~nI_%ENDIAN%\macros.ini

popd
)
@del /q %listFile%
