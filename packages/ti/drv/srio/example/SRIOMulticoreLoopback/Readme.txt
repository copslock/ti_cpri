 ******************************************************************************
 * FILE PURPOSE: Readme File for the Multicore Loopback Example Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************

The example is a demonstration of the SRIO driver while operating in a Multicore
Environment by running the SRIO IP Block in loopback mode. The test case here
showcases the SRIO Driver API being multicore safe and using the SRIO IP 
peripheral to exchange messages between different cores running on the same device.

In this test case each core performs the role of a producer and consumer. The test
starts with Core 1 sending data to Core 2. Core 2 receives and validates the data 
and sends another data message to Core3; which receives and validates the data and
sends another different data message to Core 0. Core 0 then sends a message to 
Core 1 which is received and validated. 

----------------
Prerequisites
----------------

The example project depends on the following packages:-

1. CPPI LLD 
2. QMSS LLD
3. CSL 
4. SRIO Driver

Please refer to the SRIO Driver release notes for more information on the component
version(s) required and detailed instructions on how to load & execute the projects.

