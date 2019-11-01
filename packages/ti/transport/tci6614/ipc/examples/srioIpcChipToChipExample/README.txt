SRIO Chip-to-Chip Example

An example demonstrating how to use the IPC over SRIO transport to transfer MessageQ messages between cores on
different chips

Steps to build the example:

1. Import the srioChipToChipProducer_c66xx CCS project from transport\ipc\examples\srioIpcChipToChipExample\producer
   directory. (in CCSv5, Project->Import Existing CCS/CCE Eclipse Projects)

2. Clean the srioChipToChipProducer_c66xx project, delete the Debug and Release directories, and re-build the project. 
   After the build is complete, srioChipToChipProducer_c66xx.out and srioChipToChipProducer_c66xx.map will be generated under 
   transport\ipc\examples\srioIpcChipToChipExample\producer\Debug (or \Release depending on the build configuration) directory.

3. Import the srioChipToChipConsumer_c66xx CCS project from transport\ipc\examples\srioIpcChipToChipExample\consumer
   directory. (in CCSv5, Project->Import Existing CCS/CCE Eclipse Projects)

4. Clean the srioChipToChipConsumer_c66xx project, delete the Debug and Release directories, and re-build the project. 
   After the build is complete, srioChipToChipConsumer_c66xx.out and srioChipToChipConsumer_c66xx.map will be generated under 
   transport\ipc\examples\srioIpcChipToChipExample\consumer\Debug (or \Release depending on the build configuration) directory.

Steps to run SRIO Chip-to-Chip example in CCSv5:

1. Two EVMs, connected to one another over all four SRIO lanes via breakout boards is required for this example.

2. Be sure to set the boot mode dip switch to no boot/EMIF16 boot mode on each EVM.

3. Open two instances of CCSv5 to connect to both boards.

4. On each board group Core 0 and Core 1 in CCS.

4. Connect to both cores via the group.

5. Load the evmc66xxl.gel to initialize the DDR.  The GEL can be found in the 
    "CCS install dir"\ccsv5\ccs_base_x.x.x.xxxxx\emulation\boards\evmc66xxl\gel directory.  Once loaded execute
   the default setup script on each core.  In the CCS menu go to Scripts->CPSW Functions->Global_Default_Setup.

6. Highlighting the Group in the CCS Debug window, load 
    transport\ipc\examples\srioIpcChipToChipExample\producer\Debug\srioChipToChipProducer_c66xx.out on each core of board 1 simultaneously

7. Highlighting the Group in the CCS Debug window, load 
    transport\ipc\examples\srioIpcChipToChipExample\consumer\Debug\srioChipToChipConsumer_c66xx.out on each core of board 2 simultaneously

8. Highlighting the Group in CCS Debug window for board 1, run the program in CCS on both cores simultaneously, srioChipToChipProducer_c66xx will run until the SRIO hardware tries to sync up with board 2.  At this point it will wait until the consumer images on board 2 are executed.

9. Highlighting the Group in CCS Debug window for board 2, run the program in CCS on both cores simultaneously, srioChipToChipConsumer_c66xx will run until the SRIO hardware tries to sync up with board 1.  

10. After both images have been started and the SRIO hardware on each board sync's with one another the programs on each
     core will continue execution.  The Producer core's will send four messageQ messages each to the consumer cores. Producer 
     core 0 will send four messages to Consumer core 1.  Producer core 1 will send four messages to Consumer core 0.  When 
     the test completes the consumer cores should print out that they successfully received four messages.

Please refer to C6678L/C6670L EVM boot mode dip switch settings:
http://processors.wiki.ti.com/index.php/TMDXEVM6678L_EVM_Hardware_Setup#Boot_Mode_Dip_Switch_Settings
http://processors.wiki.ti.com/index.php/TMDXEVM6670L_EVM_Hardware_Setup#Boot_Mode_Dip_Switch_Settings
