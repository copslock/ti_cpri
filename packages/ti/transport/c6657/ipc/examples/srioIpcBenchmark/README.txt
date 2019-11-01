IPC over SRIO Benchmark Example

A benchmark example demonstrating how to use the IPC over SRIO transport as well 
as its performance.

Steps to build the benchmark example:

1. Import the srioIpcBenchmark_c66xx CCS project from transport\ipc\examples\srioIpcBenchmark directory. (in CCSv5, 
   Project->Import Existing CCS/CCE Eclipse Projects)

2. Clean the srioIpcBenchmark_c66xx project, delete the Debug and Release directories, and re-build the project. 
   After the build is complete, srioIpcBenchmark_c66xx.out and srioIpcBenchmark_c66xx.map will be generated under 
   transport\ipc\examples\srioIpcBenchmark\Debug (or \Release depending on the build configuration) directory.

Steps to run srioIpcBenchmark_c66xx in CCSv5:

1. Be sure to set the boot mode dip switch to no boot/EMIF16 boot mode on the EVM.

2. Connect the physical SRIO lanes in a loopback configuration.  This can be done with a breakout board connected in the following
    manner:
    lane 8 TX+ to RX+, TX- to RX-
    lane 9 TX+ to RX+, TX- to RX-
    lane 10 TX+ to RX+, TX- to RX-
    lane 11 TX+ to RX+, TX- to RX-

    If no breakout board is available the SRIO device can be configured for internal loopback.  
    Search the transport\ipc\examples\srioIpcBenchmark\device_srio.c file for all instance of "loopback mode".  Uncomment
    all code relating to loopback mode and comment all code relating to non-loopback or normal mode.  Clean then rebuild
    the project.

3. Group Core 0 and Core 1 in CCS.

4. Connect to both cores via the group.

5. Load the evmc66xxl.gel to initialize the DDR.  The GEL can be found in the 
    "CCS install dir"\ccsv5\ccs_base_x.x.x.xxxxx\emulation\boards\evmc66xxl\gel directory.  Once loaded execute
   the default setup script on each core.  In the CCS menu go to Scripts->EVMC6657L Init Functions->Global_Default_Setup.

6. Highlighting the Group in the CCS Debug window, load 
    transport\ipc\examples\srioIpcBenchmark\Debug\srioIpcBenchmark_c66xx.out on each core

7. Highlighting the Group in CCS Debug window, run the program in CCS on both cores simultaneously, srioIpcBenchmark_c66xx         will send messageQ messages between the cores via the SRIO transport.  The messages will be used to measure the       transport's performance.  The test will be complete after the throughput (msg/s) has been calculated.
