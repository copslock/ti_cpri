@REM ******************************************************************************
@REM * FILE PURPOSE: Create Bootable image from binaries
@REM ******************************************************************************
@REM * FILE NAME: K2GImageGen.bat
@REM *
@REM * DESCRIPTION:
@REM *  Creates Bootable image from binaries pointed by env variables
@REM *
@REM * USAGE:
@REM *  K2GImageGen.bat
@REM *
@REM * Copyright (C) 2017, Texas Instruments, Inc.
@REM *
@REM * Redistribution and use in source and binary forms, with or without
@REM * modification, are permitted provided that the following conditions
@REM * are met:
@REM *
@REM *   Redistributions of source code must retain the above copyright
@REM *   notice, this list of conditions and the following disclaimer.
@REM *
@REM *   Redistributions in binary form must reproduce the above copyright
@REM *   notice, this list of conditions and the following disclaimer in the
@REM *   documentation and/or other materials provided with the
@REM *   distribution.
@REM *
@REM *   Neither the name of Texas Instruments Incorporated nor the names of
@REM *   its contributors may be used to endorse or promote products derived
@REM *   from this software without specific prior written permission.
@REM *
@REM * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
@REM * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
@REM * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
@REM * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
@REM * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
@REM * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
@REM * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
@REM * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
@REM * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@REM * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@REM * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@REM ***************************************************************************


REM @echo off
REM Device Id Hard coded to  55
REM Device ID & CPU ID should be in sync with SBL. Refer SBL user guide for values
set Dev_ID=55
set MPU_CPU0_ID=0
set DSP0_ID=5

REM Define Output file path

if not defined TOOLS_PATH (
set TOOLS_PATH=%PDK_INSTALL_PATH%\ti\boot\sbl\tools
)

IF NOT EXIST %BIN_PATH%\ mkdir %BIN_PATH%

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_MPU_CPU0 (
set APP_MPU_CPU0=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_MPU_CPU0% (
set MPU_CPU0=%MPU_CPU0_ID%
set image_gen=1
set APP_MPU_CPU0_RPRC=%APP_MPU_CPU0%.rprc
)
if defined APP_MPU_CPU0_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_MPU_CPU0% %APP_MPU_CPU0_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_DSP0 (
set APP_DSP0=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_DSP0% (
set DSP0_CPU=%DSP0_ID%
set image_gen=1
set APP_DSP0_RPRC=%APP_DSP0%.rprc
)
if defined APP_DSP0_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_DSP0% %APP_DSP0_RPRC% )

REM ImageGen
if defined image_gen (
REM Generating MulticoreImage Gen
"%TOOLS_PATH%\multicoreImageGen\bin\MulticoreImageGen.exe" LE %Dev_ID% %BIN_PATH%\app %MPU_CPU0% %APP_MPU_CPU0_RPRC% %DSP0_CPU% %APP_DSP0_RPRC% )

if not defined image_gen (
echo. Error Application .out missing!! )

REM Clearing the image gen flag
set image_gen=
