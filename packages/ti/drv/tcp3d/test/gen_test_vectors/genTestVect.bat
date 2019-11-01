@echo off

REM set MSDEV_PATH="C:\Program Files\Microsoft Visual Studio\COMMON\MSDev98\Bin"
REM if exist %MSDEV_PATH% (
REM set path=%path%;%MSDEV_PATH%
REM ) else (
REM echo !!! MSDEV path not set. Not found at %MSDEV_PATH%. !!!
REM goto end
REM )

REM Get the current directory
@set CUR_DIR=%~sdp0

REM build the executable first
REM @msdev %CUR_DIR%\msvc\GenTestVectors.dsp /make "all - Win32 Debug"
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

REM ~~~~~~~~~~~~~~~ LTE TEST ~~~~~~~~~~~~~~~
@%EXE_NAME% %SRC_PATH%\test0_lte        config_list.cfg %DST_PATH%\test0_lte         %1
@%EXE_NAME% %SRC_PATH%\test1_lte        test1_list.cfg  %DST_PATH%\test1_lte         %1
@%EXE_NAME% %SRC_PATH%\test2_lte        test2_list.cfg  %DST_PATH%\test2_lte         %1
@%EXE_NAME% %SRC_PATH%\test3_lte        test3_list.cfg  %DST_PATH%\test3_lte         %1
@%EXE_NAME% %SRC_PATH%\test4_lte        test4_list.cfg  %DST_PATH%\test4_lte         %1
@%EXE_NAME% %SRC_PATH%\sim_config\LTE   lte_list.cfg    %DST_PATH%\sim_config\LTE    %1
@%EXE_NAME% %SRC_PATH%\LTE              config_list.cfg %DST_PATH%\LTE               %1
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

REM ~~~~~~~~~~~~~~~ WCDMA TEST ~~~~~~~~~~~~~~~
@%EXE_NAME% %SRC_PATH%\test0_wcdma      config_list.cfg %DST_PATH%\test0_wcdma       %1
@%EXE_NAME% %SRC_PATH%\test1_wcdma      test1_list.cfg  %DST_PATH%\test1_wcdma       %1
@%EXE_NAME% %SRC_PATH%\test2_wcdma      test2_list.cfg  %DST_PATH%\test2_wcdma       %1
@%EXE_NAME% %SRC_PATH%\test3_wcdma      test3_list.cfg  %DST_PATH%\test3_wcdma       %1
@%EXE_NAME% %SRC_PATH%\sim_config\WCDMA wcdma_list.cfg  %DST_PATH%\sim_config\WCDMA  %1
@%EXE_NAME% %SRC_PATH%\WCDMA            config_list.cfg %DST_PATH%\WCDMA             %1
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

REM ~~~~~~~~~~~~~~~ WIMAX TEST ~~~~~~~~~~~~~~~
@%EXE_NAME% %SRC_PATH%\test0_wimax      config_list.cfg %DST_PATH%\test0_wimax       %1
@%EXE_NAME% %SRC_PATH%\test1_wimax      test1_list.cfg  %DST_PATH%\test1_wimax       %1
@%EXE_NAME% %SRC_PATH%\test2_wimax      test2_list.cfg  %DST_PATH%\test2_wimax       %1
@%EXE_NAME% %SRC_PATH%\test3_wimax      test3_list.cfg  %DST_PATH%\test3_wimax       %1
@%EXE_NAME% %SRC_PATH%\sim_config\WIMAX wimax_list.cfg  %DST_PATH%\sim_config\WIMAX  %1
@%EXE_NAME% %SRC_PATH%\WIMAX            config_list.cfg %DST_PATH%\WIMAX             %1
REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

REM @%EXE_NAME% %SRC_PATH%\WCDMA_200        config_list.cfg %DST_PATH%\WCDMA_200         %1
REM @%EXE_NAME% %SRC_PATH%\LTE_200          config_list.cfg %DST_PATH%\LTE_200           %1
REM @%EXE_NAME% %SRC_PATH%\WIMAX_200        config_list.cfg %DST_PATH%\WIMAX_200         %1

:end
