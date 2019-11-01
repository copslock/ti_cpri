Running the RM ARM Linux + DSP Test:

Please see the RM User Guide for instructions on how to run this test

http://processors.wiki.ti.com/index.php/MCSDK_UG_Chapter_Developing_System_Mgmt#Running_RM_Test_Projects



Running the RM ARM Linux multi-process test:



To run rmServer.out:



Copy the following files from rm/test/dts_files to the install directory containing rmServer.out:

global-resources.dtb

server-policy.dtb

linux-evm.dtb [Optional]



The data contained in the latter DTB files is not required to run the RM Server.  A new GRL and policy
can be written and supplied instead of the latter files.



For information on how to create new GRLs and Policies please see:

http://processors.wiki.ti.com/index.php/MCSDK_UG_Chapter_Developing_System_Mgmt#Resource_Manager



To run the Server:

$ ./rmServer.out global-resources.dtb server-policy.dtb -l linux-evm.dtb



The Server will wait for Client socket connections and service any requests received via those sockets.





To run the rmLinuxClientTest.out:



Copy the following files from rm/test/dts_files to the install directory containing rmLinuxClientTest.out:

static-policy.dtb



To execute the Client test:

$ ./rmLinuxClientTest.out static-policy.dtb



The Client test will establish a socket connection with the Server, request resources and then free all resources requested.


To execute the Multi-threaded Client test, rmLinuxMtClientTest.out:

$ ./rmLinuxMtClientTest.out

The Multi-threaded Client test will establish a socket connection with the Server, request resources from two pthreads using the same RM instance and then free all resources requested.
