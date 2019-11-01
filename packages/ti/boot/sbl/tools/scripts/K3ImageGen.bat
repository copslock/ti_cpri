@echo off
REM Define Device Id for K3 - 55
REM Device ID & CPU ID should be in sync with SBL. Refer SBL user guide for values
set Dev_ID=55

REM MPU1_CPU0_ID = 0
REM MPU1_CPU1_ID = 1
REM MPU2_CPU0_ID = 2
REM MPU2_CPU1_ID = 3
REM MCU1_CPU0_ID = 4
REM MCU1_CPU1_ID = 5
REM MCU2_CPU0_ID = 6
REM MCU2_CPU1_ID = 7
REM MCU3_CPU0_ID = 8
REM MCU3_CPU1_ID = 9
REM DSP1_C66X_ID = 10
REM DSP2_C66X_ID = 11
REM DSP1_C7X_ID = 12
REM DSP2_C7X_ID = 13
REM MPU1_SMP_ID = 14
REM MPU2_SMP_ID = 15
REM MPU_SMP_ID = 16
REM MCU1_SMP_ID = 17
REM MCU2_SMP_ID = 18
REM MCU3_SMP_ID = 19
REM ONLY_LOAD_ID = 20

if not defined TOOLS_PATH (
set TOOLS_PATH=%PDK_INSTALL_PATH%\ti\boot\sbl\tools
)

REM checking arguments
if -%1-==-- goto noArgProvided
if -%2-==-- goto noArgProvided

REM we now have enough 2 parameters
set APP_OUT_FILE=%2
set CORE_ID=%1
set APP_OUT_NAME=%~n2
set APP_OUT_DIR=%~dp0
set APP_OUT_DIR=%APP_OUT_DIR:~0,-1%

REM Define Output file path
if not defined BIN_PATH (
echo BIN_PATH not defined.
set BIN_PATH=%APP_OUT_DIR%
)

IF NOT EXIST %BIN_PATH%\ mkdir %BIN_PATH%

echo BIN_PATH set to %BIN_PATH%

IF EXIST %APP_OUT_FILE% (
set image_gen=1
set APP_RPRC=%BIN_PATH%\%APP_OUT_NAME%.rprc
)


if defined APP_RPRC (
echo converting %APP_OUT_FILE% to RPRC format for core #%CORE_ID%
echo TOOLS_PATH=%TOOLS_PATH%
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_OUT_FILE% %APP_RPRC% )


REM ImageGen
if defined image_gen (
echo Generating MulticoreImage 
"%TOOLS_PATH%\multicoreImageGen\bin\MulticoreImageGen.exe" LE %Dev_ID% %BIN_PATH%\%APP_OUT_NAME%.appimage %CORE_ID% %APP_RPRC%
"powershell -executionpolicy unrestricted -command %PDK_INSTALL_PATH%\ti\build\makerules\x509CertificateGen.ps1" -b %BIN_PATH%\%APP_OUT_NAME%.appimage -o %BIN_PATH%\%APP_OUT_NAME%.appimage.signed -c R5 -l 0x0 -k %PDK_INSTALL_PATH%\ti\build\makerules\k3_dev_mpk.pem )

if not defined image_gen (
echo. Error Application .out missing!! )

REM Clearing the image gen flag
set image_gen=
goto done


:noArgProvided
echo Invalid number of parameters.
echo Syntax:
echo "%~nx0 <CoreID> <.out> [<CoreID2> <.out2>] ...."

:done

