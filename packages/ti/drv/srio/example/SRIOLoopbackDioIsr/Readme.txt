 ******************************************************************************
 * FILE PURPOSE: Readme File for the Loopback DIO ISR Example Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2011, Texas Instruments, Inc.
 *****************************************************************************

The example is a demonstration of the SRIO driver running the SRIO IP Block 
in loopback mode. The example showcases the use of SRIO DIO sockets using 
LSU interrupts to indicate the completion of packet transfer. 

It is shown how multiple sockets with different Source IDs can post 
transactions and process the pending interrupt raised by SRIO device.

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

