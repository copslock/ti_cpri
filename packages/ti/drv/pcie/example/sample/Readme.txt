 ******************************************************************************
 * FILE PURPOSE: Readme File for the PCIE Example Project
 ******************************************************************************
 * FILE NAME: Readme.txt
 * Copyright (C) 2011, Texas Instruments, Inc.
 *****************************************************************************

The example demonstrates the use of APIs provided in PCIE LLD. This example does NOT work
in the Shannon Simulator.
Check the release notes for pre-requisites, tools and version information.

------------------
Example Overview
------------------

In the PCIe sample example two Shannon EVMs are used to test the PCIe driver. As described in the following figure, Shannon 1 is configured as a Root Complex and Shannon 2 is configured as End Point.

       Shannon 1                                       Shannon 2
   ------------------                             ------------------
   |                |                             |                |
   |   Root         |          PCIe Link          |  End Point     |
   |   Complex      | <-------------------------->|                |
   |                |                             |                |
   ------------------                             ------------------
  
At startup, each EVM configures its PCIe subsystem:
• Serdes, clock, PLL
• PCIe Mode and Power domain
• Inbound/Outbound address translation and BAR registers
• Link training is triggered

Once the PCIe link is established, the following sequence of events will happen:
• Shannon 1 sends data to Shannon 2
• Shannon 2 waits to receive all the data
• Shannon 2 sends the data back to Shannon 1
• Shannon 1 waits to receive all the data
• Shannon 1 verifies if the received data matches the sent data and declares test pass or fail.

-------------------------
Steps to run the example
-------------------------
1. Build the example
2. Do a System Reset in both EVMs
3. Load exampleProject.out in core zero in both EVMs
4. In Shannon 1, use CCS watch window to modify the value of global
   variable PcieModeGbl. In this example, Shannon 1 is a Root Complex,
   therefore set PcieModeGbl to pcie_RC_MODE (this can be done through a drop down
   menu in the watch window).
5. Click the "Run" button in CCS for both EVMs (it is okay to have a few seconds between
   the "Run" buttons are clicked in both sides).

-------------------------
Expected result
-------------------------
1. In Shannon 1 CCS console the status of the test will be updated. At the end, the message 
   "Test passed" is expected.
2. In Shannon 2 CCS console the status of the test will be updated. At the end, the message 
   "End of test" is expected.



    

