*************************************************************************************
 * FILE PURPOSE: Readme File on for Eve Loader application and How to build the app
 * and use the same.
 ************************************************************************************
 * FILE NAME: README.txt
 * Copyright (C) 2018, Texas Instruments, Inc.
*************************************************************************************

Introduction
************
This document provides the details of the EvE loader application and how to build the
application and execute the same on AM572x/4x

Software Legacy Notes
---------------------
The purpose of this application is to provide a application demo that uses the sbl library 
for loading the application into EVE Core of AM572x/4x. This example is limited only to
support EVE loader through M4 and verifying that the EVE is loaded successfully through
MPU core. 

NOTE:
*****
Currently the application is supported only in Linux platform.
It can be extended to Windows but need to follow some additional steps to generate the
firmware.

Building EVE Loader Application
-------------------------------
The application can only be built inside the sbl folder with a single command as follows.
For Linux:
make eveLoader SOC=AM572x BOARD=idkAM572x
make eveLoader SOC=AM574x BOARD=iskAM574x

The build library will be located in the following location.
<pdk_install_path_am57xx>/packages/ti/boot/sbl/lib/<SOC>/m4/release/sbl_lib.aem4
SOC= AM572x or AM574x

Breif on EVE Loader Application
*******************************
eve1MulticoreApp:
----------------
The eve1Multicore app is a mail box application which waits on to receive a message from 
the Master core and send back the message to the Master core. This way the application 
verifies that the EVE is loaded successfully.
This application is build and the binary is generated through the Multicoregen sbl tool.
The binary is converted into a header file with hex value. (eve_firmware.h)

ipu1EveLoaderApp:
----------------
The ipu1EveLoader app will use the firmware header file (eve_firmware.h) as an array and 
parses the firmware image. The application will determine the number of EVE core firmware
is available and reset the EVE core(s). The firmware is loaded into the respective core(s)
which will be determined by the firmware header with the core ID.

MpuMulticoreApp:
---------------
The mpuMulticoreApp is a mailbox application which is a master core. The application sends
the message to the slave core(s) and waits to receive the message from them.

The mpuM4EveLoader will create a multicore app by combining the mpuMulticoreApp and 
ipu1EveLoaderApp into a single app. This app is generated in the following location.
<pdk_install_path_am57xx>/packages/ti/boot/sbl/binary/<Board>/example/mpuM4EVELoaderApp/bin/app
<Board>= idkAM572x or idkAM574x
The app shall be copied into the SD card along with the MLO. The SD card is then inserted into
the IDK board and power on the board. Ensure the SD boot mode is set.
