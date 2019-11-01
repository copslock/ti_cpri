@echo off
rem ***************************************************************************
rem 
rem     The main purpose of this script is to apply label for release.
rem 
rem SYNTAX:
rem     applyFFTCLabel.bat <View driver> <Lable to Apply>
rem 
rem Note: There is no check on the label type is done.
rem 
rem ***************************************************************************

if %0%1==%0  goto noparameter
if %0%1%2==%0%1  goto noparameter

REM Clearcase Directory where LLD component is located.
set CLEARCASE_DIRECTORY=%1
IF NOT DEFINED CLEARCASE_DIRECTORY GOTO noparameter

REM Label to be applied
set LABEL=%2
IF NOT DEFINED LABEL GOTO noparameter

REM Create Label Type
echo Creating label type %LABEL%@%CLEARCASE_DIRECTORY%\gtcsl_projects
cleartool mklbtype -nc -global %LABEL%@%CLEARCASE_DIRECTORY%\gtcsl_projects

REM Apply label recursively
echo Applying label %LABEL% on %CLEARCASE_DIRECTORY%\gtcsl_ip\fftc_drv
cleartool mklabel -replace -recurse %LABEL% %CLEARCASE_DIRECTORY%\gtcsl_ip\fftc_drv

echo Applying label %LABEL% on %CLEARCASE_DIRECTORY%\gtcsl_platform\Appleton\ti\drv\fftc
cleartool mklabel -replace -recurse %LABEL% %CLEARCASE_DIRECTORY%\gtcsl_platform\Appleton\ti\drv\fftc

echo Applying label %LABEL% on %CLEARCASE_DIRECTORY%\gtcsl_platform\Nyquist\ti\drv\fftc
cleartool mklabel -replace -recurse %LABEL% %CLEARCASE_DIRECTORY%\gtcsl_platform\Nyquist\ti\drv\fftc

echo Applying label %LABEL% on %CLEARCASE_DIRECTORY%\gtcsl_platform\TurboNyquist\ti\drv\fftc
cleartool mklabel -replace -recurse %LABEL% %CLEARCASE_DIRECTORY%\gtcsl_platform\TurboNyquist\ti\drv\fftc

echo Applying label %LABEL% on %CLEARCASE_DIRECTORY%\gtcsl_platform\KeyStone2\ti\drv\fftc
cleartool mklabel -replace -recurse %LABEL% %CLEARCASE_DIRECTORY%\gtcsl_platform\KeyStone2\ti\drv\fftc

GOTO :EOF

:noparameter
echo.  *************************************************************************
echo.  "    Usage:- %0 <drive:>  <LABEL> 
echo.  "
echo.  "    Ex:- (1) %0 K: FUL_GT_CHK.KEYSTONE2.FFTC_2012_03_22
echo.  "         (2) %0 C:\snapviews\gt_kystn2_drv_rsk_snap FUL_GT_CHK.KEYSTONE2.FFTC_2012_03_22
echo.  *************************************************************************
