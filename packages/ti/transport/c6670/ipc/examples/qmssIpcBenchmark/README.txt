IPC over QMSS Benchmark Example

A benchmark example demonstrating how to use the IPC over QMSS transport as well 
as its performance.

Steps to build the benchmark example:

1. Import the qmssIpcBenchmark_c66xx CCS project from transport\ipc\examples\qmssIpcBenchmark directory. (in CCSv5, 
   Project->Import Existing CCS/CCE Eclipse Projects)

2. Clean the qmssIpcBenchmark_c66xx project, delete the Debug and Release directories, and re-build the project. 
   After the build is complete, qmssIpcBenchmark_c66xx.out and qmssIpcBenchmark_c66xx.map will be generated under 
   transport\ipc\examples\qmssIpcBenchmark\Debug (or \Release depending on the build configuration) directory.

Steps to run qmssIpcBenchmark_c66xx in CCSv5:

1. Be sure to set the boot mode dip switch to no boot/EMIF16 boot mode on the EVM.

2. Group Core 0 and Core 1 in CCS.

3. Connect to both cores via the group.

4. Load the evmc66xxl.gel to initialize the DDR.  The GEL can be found in the 
    "CCS install dir"\ccsv5\ccs_base_x.x.x.xxxxx\emulation\boards\evmc66xxl\gel directory.  Once loaded execute
   the default setup script on each core.  In the CCS menu go to Scripts->CPSW Functions->Global_Default_Setup.

5. Highlighting the Group in the CCS Debug window, load 
    transport\ipc\examples\qmssIpcBenchmark\Debug\qmssIpcBenchmark_c66xx.out on each core

6. Highlighting the Group in CCS Debug window, run the program in CCS on both cores simultaneously, qmssIpcBenchmark_c66xx       will send messageQ messages between the cores via the QMSS transport.  The messages will be used to measure the    transport's performance.  The test will be complete after the throughput (msg/s) has been calculated.

Please refer to C6678L/C6670L EVM boot mode dip switch settings:
http://processors.wiki.ti.com/index.php/TMDXEVM6678L_EVM_Hardware_Setup#Boot_Mode_Dip_Switch_Settings
http://processors.wiki.ti.com/index.php/TMDXEVM6670L_EVM_Hardware_Setup#Boot_Mode_Dip_Switch_Settings
