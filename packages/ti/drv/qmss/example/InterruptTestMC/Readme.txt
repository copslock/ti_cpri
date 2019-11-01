 ******************************************************************************
 * FILE PURPOSE: Readme File for the QMSS Interrupt test example project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2015, Texas Instruments, Inc.
 *****************************************************************************

The example demonstrates QMSS interrupts from multiple cores to single receive core.
Also it include benchmark code to profile the interrupt latency.

This example works only on A15.

On the A15, the example should be started once from a Linux shell.  It auto-forks 4 separate tasks, to run
on 4 separate cores.  On A15, the example requires rmServer.out to be started first using the global resource
list (global-resource-list.dtb) and policy (policy_dsp_arm.dtb) for your device.

Check the release notes for prerequisites, version information and steps on how to run examples

The tests can be run using the following syntax

./testexecutable [<num_packets> <delay_between_packets_in_us> <test_mode:0,1,2> [core_number]]
Note: To measure latency, use test mode 1 and  the packets need to be spaced enough to allow for receive/processing time
(e.g)
./qmInterruptTestMultiprocess_k2h.out 1000000 100 1 

Here are the different target builds
        ├── qmInterruptTestMultiprocess_<device>.out
        -->Uses simple Q push to send packets, uses static qmss library
        ├── qmInterruptTestMultiprocess_so_k2h.out
        --> Same as above using shared qmss library
        ├── qmInterruptTestMultiprocessInfraDMA_<device>.out
        --> Uses infra DMA to send packets, Uses static qmss library
        ├── qmInterruptTestMultiprocessInfraDMA_so_k2h.out
        --> Same as above using shared qmss library
        ├── qmInterruptTestMultiprocessInfraDMASharedmem_so_k2e.out
        --> Uses shared memory to profile multiple sub components. Uses Shared qmss library