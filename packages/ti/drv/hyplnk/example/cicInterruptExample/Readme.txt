 ******************************************************************************
 * FILE PURPOSE: Readme File for the simple HYPLNK Example Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2015 Texas Instruments, Inc.
 *****************************************************************************

The example demonstrates the use of HYPLNK LLD APIs to generate interrupts
 through the HyperLink sub-system to ARM and DSP cores. Also supports receiving
 interrupts on the DSP core.

This example can use HyperLink
either in loopback mode on a single board, or peer to peer with a second
device or board.  The example performs simple memory accesses across hyperlink
using the CPU.

There are several configuration #define's in the example that allow
configuration of parameters like defining the reference clock, the serial
data rate, the number of lanes, and loopback.  These defines are located
in ti/drv/hyplnk/example/common/hyplnkLLDCfg.h.

No changes to the defines are required to run 6.25 GBaud across two
devices or boards.

Check the release notes for prerequisites, version information and steps on
how to run examples

Configuration of Hyperlink Port in the case of devices supporting multiple
 ports:
	- Check for hyplnk_EXAMPLE_PORT. Default will be port 0

Configuration for  Reference clock
	- Configure hyplnk_EXAMPLE_HYPLNK_REF_KHZ for the EVM in hyplnkPlatCfg.h file

Configuration of Link Speed:
	- Note limited link speed and rate are currently supported in example code for
	  SERDES configuration function hyplnkExampleDefSerdesSetup().
	  Refer the function for supported config and modify in the case of
	  platforms requiring  alternate configuration

To Run the test follow the steps below

1. On the ARM first run the cicInterruptExample.out
   This will power up the hyperlink interface and wait for prompt to send
   interrupts

	./cicInterruptExample.out

2. Load and Run the DSP binary hyplnk_K2HC66BiosCicInterruptExampleProject.out
   on the DSP cores.
   This will now wait for the interrupts. wiht prompt:
 Ready to send interrupts: Press any key to continue...

3. Also can run the uio-test on a telnet window to wait for interrupts on the
   ARM side.

4. Run enter on the ARM cicInterruptExample.out to send the interrupts.

5. Check DSP application finish successfully.

6. Check uio-test to confirm interrupt received on ARM

