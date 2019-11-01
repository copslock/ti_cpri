*************************************************************************
*	PASS Multi Core Example 
*	---------------------------------
*
*	(C) Copyright 2009-2013, Texas Instruments, Inc.
*
*	Use of this software is controlled by the terms and conditions found 
*	in the license agreement under which this software has been supplied.
*************************************************************************

The PASS multi core example demonstrates the use of CPP/QMSS/PA/CSL APIs to 
send/receive data through the ethernet switch using Host descriptors 
in a multi core scenario.

By default the project runs on 4 cores (0-3). All cores share the same MAC and IP addresses.
Each core has a distinct UDP port associated to it. Each core sends packets towards the RGMII
through the QMSS/CPPI/PASS modules. The packet is looped back at the RGMII and routed to a 
specific RX queue depending on the UDP port number. Each core monitors one RX queue.
Each core verifies that it received all of its packets back and declares pass or fail.

The example is designed to run in loopback modes, i.e. it expects to receive the transmitted packet.
The following two loopback modes are supported:
    CPSW_LOOPBACK_INTERNAL(default): The transmitted packet is loopbacked at the SGMII through SGMII internal loopback
    CPSW_LOOPBACK_EXTERNAL: The transmitted packet should bo loopbacked by application outsite the SoC
    
This example also demonstrates the normal CPSW configuration as CPSW_LOOPBACK_NONE mode. In this configuration, it is up to the user
to modify PASS configuration to receive desired ingress packets.

-------------------------------------
Steps to run the example on DSP
-------------------------------------
1. Simulator configuration

In order for the example to work in external loopback mode successfully, i.e. to be able to send/receive data packets from the wire, 
Please refer to http://ap-fpdsp-swapps.dal.design.ti.com/index.php/66AK2H12_Device_Simulator_User_Guide#Configurable_parameters_for_simulation
to configure the following EMAC config params in CCSEup
    a) EMAC Adaptor Enable: ON
    b) EMAC Network Adaptor: Broadcom (NIC card using in PC)
    c) EMAC MAC Adress Port 0: other
       EMAC_MAC_ADDR_PORT0: 10-11-12-13-14-15
    
2. Build the CCS project

By default this example is set to run in 4 cores, this way it can run on multiple targets.
In order to change the number of cores, the following should be modified:
#define NUM_CORES <#cores>
This can be found in the multicore_example.h file.

Once the number of cores is set, select the project in CCS, clean it (Project->clean) and build it (Project->build).

3. Execute

To execute the project ensure the following is done:
    a) Reset the cores (applicable for EVM under NO_BOOT mode only)
    b) Run EVM specific GEL scripts (applicable for EVM under NO_BOOT mode only)
    c) Load the program into the cores (0 - NUM_CORES) before running any core. 
    d) Run the cores (0 - NUM_CORES).

NOTE:
If EVM is *NOT* running in NO_BOOT mode, the below assumptions are made
3a. The PDSPs are downloaded for QMSS and PA outside the example
3b. CPSW Switch is configured outside the example
3c. To disable the auto detect logic, please set 'autodetectLogic' variable to '0' using CCS. When this is done, the all the steps as mentioned in 'Execute' section are mandatory.

4. Result

Each core will output to the console its status progress and the number of sent/received packets.
Core 0 will declare pass/fail.


