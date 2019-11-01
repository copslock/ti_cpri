*************************************************************************************
 * FILE PURPOSE: Readme File for building the SBL Library for Eve Loader
 ************************************************************************************
 * FILE NAME: README.txt
 * Copyright (C) 2018, Texas Instruments, Inc.
*************************************************************************************

Introduction
************
This document provides the details of the SBL library that comprises.

Software Legacy Notes
---------------------
The purpose of this software is to provide a library for parsing the arp32 image and 
loading into EVE Core of AM572x/4x. This library is limited only with APIs that will 
be supported to run on IPU core. 

Building SBL Library
--------------------
The SBL library can be built from the top level with a simple make command as follows.
For Windows:
gmake sbl SOC=AM572x BOARD=idkAM572x
gmake sbl SOC=AM574x BOARD=iskAM574x
For Linux:
make sbl SOC=AM572x BOARD=idkAM572x
make sbl SOC=AM574x BOARD=iskAM574x

The build library will be located in the following location.
<pdk_install_path_am57xx>/packages/ti/boot/sbl/lib/<SOC>/m4/release/sbl_lib.aem4
SOC= AM572x or AM574x

Breif on SBL Library
--------------------
 The SBL EVE Loader library supports APIs to parse the image of EVE that is been
 generated with Multicoregen application available in the sbl tools.
 The image will be parsed to identify the number of EVE instance images are been
 combined. 
 Based on the number of EVE instance the EVE cores are put to reset. The firmware
 is extracted for the images of EVE core and it will be loaded into the respective 
 EVE core based on the ID.
 Once the image(s) are loaded in the core(s) the EVE core(s) is pulled out of reset.
 Now the EVE firmware starts running .
 
Unlike the native SBL this eve supported SBL library uses PM driver for configuring 
the EVE core(s).
  