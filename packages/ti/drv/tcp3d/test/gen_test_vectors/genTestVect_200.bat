@echo off

set MSDEV_PATH="C:\Program Files\Microsoft Visual Studio\COMMON\MSDev98\Bin"
if exist %MSDEV_PATH% (
set path=%path%;%MSDEV_PATH%
) else (
echo !!! MSDEV path not set. Not found at %MSDEV_PATH%. !!!
goto end
)

REM Get the current directory
@set CUR_DIR=%~sdp0

REM build the executable first
@msdev %CUR_DIR%\msvc\GenTestVectors.dsp /make "all - Win32 Debug"
REM Set your Executable file path
@set EXE_NAME=%CUR_DIR%\msvc\Debug\GenTestVectors.exe
REM Set your Source list folder path
@set SRC_PATH=%CUR_DIR%
REM Set your Destination test vector folder path
@set DST_PATH=%CUR_DIR%

REM ~~~~~~~~~~~~~~~~~~~~~~ GENERATE TEST VECTORS ~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
REM GenTestVectors.exe <Source Folder> <List file name> [Destination Folder] [Out File Type]
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

@%EXE_NAME% %SRC_PATH%\WCDMA_200        config_list.cfg %DST_PATH%\WCDMA_200         %1
@%EXE_NAME% %SRC_PATH%\LTE_200          config_list.cfg %DST_PATH%\LTE_200           %1
@%EXE_NAME% %SRC_PATH%\WIMAX_200        config_list.cfg %DST_PATH%\WIMAX_200         %1

:end
