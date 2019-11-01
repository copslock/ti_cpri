 ******************************************************************************
 * FILE PURPOSE: Readme File for sRIO Benchmarking Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2009-2011, Texas Instruments, Inc.
 *****************************************************************************

The example demonstrates the use of sRIO LLD APIs to configure and use the 
sRIO sub-system on C66xx devices.  This example can use sRIO
either in loopback mode on a single board, or peer to peer with a second
device or board.  The example performs Type-11, NWRITE and NREAD transfers
across sRIO.

For this example core 0 is used for the consumer and core 1 is used for
the producer. This holds true even when using the example across two
boards. Please see the top of the benchmarking.c file for the variables
used to control the test.

To run core to core in loopback mode on the same EVM with port width at
4X and lanes at 5.0Gbaud no changes are needed. This runs on two cores so
the same .out file will need to be loaded and run simultaneously on core 0
and core 1.

To test board to board at same port width and baud rate shown above, the following
changes to the global variables will be needed: (Boards will need to be connected
through break-out cards are using a chassis with an sRIO switch.
 On the Consumer EVM (RX side, load .out file on core 0):
   1) testControl.srio_isLoopbackMode = FALSE;
   2) Start consumer application before starting the producer application.
 On the Producer EVM (TX side, load .out file on core 1):
   1) testControl.srio_isLoopbackMode = FALSE;
   2) testControl.srio_initCorenum = 1;
   3) Start the producer after the consumer

Please see the benchmarking.h file for the current default test control values.

Please see the benchmarking.c and device_srio_tput.c files for the global
variables that are set by the control values defined in the benchmarking.h file.

Check the release notes for prerequisites, version information and steps on 
how to run examples

