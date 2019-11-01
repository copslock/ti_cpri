 ******************************************************************************
 * FILE PURPOSE: Readme File for the TSIP Example Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2010, Texas Instruments, Inc.
 *****************************************************************************

The example demonstrates the use of APIs provided in TSIP LLD. TSIP is running in
data loopback mode. The example is provided for both Big and Little Endian. 

Check the release notes for pre-requisites, tools and version information.

-------------------------
Steps to run the example
-------------------------


1. Note that TSIP peripheral is not modeled by the c6608 simulator. 
Therefore, this example can only run in a Shannon/c6608 EVM.


2. Build the CCS project

Use CCS V4 4.2.0.09000 (M9) or later version
Import the CCS RTSC projects (BE or LE) contained in this folder.
Set the following variables (CSL_INSTALL_PATH, TSIP_INSTALL_PATH) based on where
the LLD and the CSL were installed.
                           
For instance:
CSL_INSTALL_PATH  = C:\Program Files\Texas Instruments\pdk_c6608_1_0_0_3\packages
TSIP_INSTALL_PATH = C:\Program Files\Texas Instruments\pdk_c6608_1_0_0_3\packages


3. Execute

To execute the project ensure the following is done:
    a) Reset core 0
    b) Load the program into core 0 
    c) Start execution

4. Result

The core will output to the console its status progress and the pass/fail status.



