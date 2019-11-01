 ******************************************************************************
 * FILE PURPOSE: Readme File for the QMSS Infrastructure Multicore Mode Example Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2009-2014, Texas Instruments, Inc.
 *****************************************************************************

The example demonstrates data transfer and synchronization between multiple cores. 

This example works on both the c66x as well as the A15.

On the C66, the example has to be loaded on all 4 cores. It uses accumulation to trigger interrupts to the host. 
The example is provided with projects for both Big and Little Endian.  Since K2E does not have multicore c66x,
this example is not supported on K2E.

On the A15, the example should be started once from a Linux shell.  It auto-forks 4 separate tasks, to run
on 4 separate cores.  On A15, the example requires rmServer.out to be started first using the global resource
list (global-resource-list.dtb) and policy (policy_dsp_arm.dtb) for your device.

Check the release notes for prerequisites, version information and steps on how to run examples

