REM @echo off
REM Define Device Id Vayu - 55 - choosen random value will be updated
REM Device ID & CPU ID should be in sync with SBL. Refer SBL user guide for values
set Dev_ID=55
set MPU_CPU0_ID=0
set MPU_CPU1_ID=1
set	IPU1_CPU0_ID=2
set IPU1_CPU1_ID=3
set IPU1_CPU_SMP_ID=4
set	IPU2_CPU0_ID=5
set	IPU2_CPU1_ID=6
set IPU2_CPU_SMP_ID=7
set DSP1_ID=8
set	DSP2_ID=9

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
if not defined APP_MPU_CPU1 (
set APP_MPU_CPU1=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_MPU_CPU1% (
set MPU_CPU1=%MPU_CPU1_ID%
set image_gen=1
set APP_MPU_CPU1_RPRC=%APP_MPU_CPU1%.rprc
)
if defined APP_MPU_CPU1_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_MPU_CPU1% %APP_MPU_CPU1_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_IPU1_CPU0 (
set APP_IPU1_CPU0=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_IPU1_CPU0% (
set IPU1_CPU0=%IPU1_CPU0_ID%
set image_gen=1
set APP_IPU1_CPU0_RPRC=%APP_IPU1_CPU0%.rprc
)
if defined APP_IPU1_CPU0_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_IPU1_CPU0% %APP_IPU1_CPU0_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_IPU1_CPU1 (
set APP_IPU1_CPU1=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_IPU1_CPU1% (
set IPU1_CPU1=%IPU1_CPU1_ID%
set image_gen=1
set APP_IPU1_CPU1_RPRC=%APP_IPU1_CPU1%.rprc
)
if defined APP_IPU1_CPU1_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_IPU1_CPU1% %APP_IPU1_CPU1_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_IPU2_CPU0 (
set APP_IPU2_CPU0=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_IPU2_CPU0% (
set IPU2_CPU0=%IPU2_CPU0_ID%
set image_gen=1
set APP_IPU2_CPU0_RPRC=%APP_IPU2_CPU0%.rprc
)
if defined APP_IPU2_CPU0_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_IPU2_CPU0% %APP_IPU2_CPU0_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_IPU2_CPU1 (
set APP_IPU2_CPU1=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_IPU2_CPU1% (
set IPU2_CPU1=%IPU2_CPU1_ID%
set image_gen=1
set APP_IPU2_CPU1_RPRC=%APP_IPU2_CPU1%.rprc
)
if defined APP_IPU2_CPU1_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_IPU2_CPU1% %APP_IPU2_CPU1_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_DSP1 (
set APP_DSP1=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_DSP1% (
set DSP1_CPU=%DSP1_ID%
set image_gen=1
set APP_DSP1_RPRC=%APP_DSP1%.rprc
)
if defined APP_DSP1_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_DSP1% %APP_DSP1_RPRC% )

REM assigning variable to a dummy application to avoid creating a dummy app image.
if not defined APP_DSP2 (
set APP_DSP2=%PDK_INSTALL_PATH%\ti\boot\sbl\binary\TestApp
)
IF EXIST %APP_DSP2% (
set DSP2_CPU=%DSP2_ID%
set image_gen=1
set APP_DSP2_RPRC=%APP_DSP2%.rprc
)
if defined APP_DSP2_RPRC (
"%TOOLS_PATH%\out2rprc\bin\out2rprc.exe" %APP_DSP2% %APP_DSP2_RPRC% )

REM ImageGen
if defined image_gen (
REM Generating MulticoreImage Gen
"%TOOLS_PATH%\multicoreImageGen\bin\MulticoreImageGen.exe" LE %Dev_ID% %BIN_PATH%\app %MPU_CPU0% %APP_MPU_CPU0_RPRC%  %MPU_CPU1% %APP_MPU_CPU1_RPRC% %IPU1_CPU0% %APP_IPU1_CPU0_RPRC% %IPU1_CPU1% %APP_IPU1_CPU1_RPRC% %IPU2_CPU0% %APP_IPU2_CPU0_RPRC% %IPU2_CPU1% %APP_IPU2_CPU1_RPRC% %DSP1_CPU% %APP_DSP1_RPRC% %DSP2_CPU% %APP_DSP2_RPRC% )

if not defined image_gen (
echo. Error Application .out missing!! )

REM Clearing the image gen flag
set image_gen=
