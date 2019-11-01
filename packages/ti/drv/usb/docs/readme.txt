
USB LLD Currently supports: 
+ USB Host and device MSC on AM437x, AM335x, AM57xx, DRA7xx, OMAP137, OMAP138, K2G, and AM65xx
+ USB Audio Class device mode on AM335x
+ Generic USB bulk device mode on all (AM437x, AM335x, AM57xx, K2G, OMAP13x, DRA7xx, AM65xx)

1. Building the demos:

    1.a. Bare metal examples:
        + Host and Device MSC example currently are built as two simple makefile projects. They are at
            ti/drv/usb/examples/usb_host/msc/build/am437x and
            ti/drv/usb/examples/usb_dev/msc/build/am437x respectively

        + These examples can be built with make / gmake  after PDK build environment has been setup.


        + Re-build the USB library (assuming the PDK is installed at c:\ti\pdk_am437x_1_0_0)
            cd c:\ti\pdk_am437x_1_0_0\packages
            pdksetupenv.bat
            cd ti\drv\usb
            make

            # the USB library will be then located at: ti\drv\usb\lib\armv7


    1.b. RTOS examples: Use the following commands to create CCS USB RTOS examples 
        
        cd c:\ti\pdk_am437x_1_0_0\packages
        pdksetupenv.bat

        run pdkProjectCreate.bat to create the example RTOS projects. Please refer to PDK document for command syntax.
        
        Examples CCS projects will be created under MyExampleProjects under pdk_am437x_1_0_0\packages         
        Import these CCS projects to build the examples
                   

2. Example demos:

    2.a. USB Host MSC:
        + Load the .out built from step mentioned above by CCS. The program counter should be now at main().

        + The AM437 USB host MSC demo is built to use the USB port 1 as a host port (customer can configure the example code to use the OTG port #0 as host if needed). 

        + Connect a USB Memory stick to the USB host port of the AM437x EVM.

        + Connect a serial cable between your PC and the COM port on the AM437 EVM. This serial port is used as a shell in which the example code showing the content of the file system in the said USB memory stick. Run a terminal program of your choice and connected to the serial port of the EVM. The baudrate should be 115200 with 8 bit, No parity, 1 stop bit, no flow control.

        + Run the USB host demo program from CCS. 

        + The UART console should print the demo message and a command prompt. The supported commands are: "ls", "cat", "cd", "help". Cat with direction operation can be used to copy files in the memory stick. Run "help" for more command options. 
R

    2.b. USB Device MSC:
        + Load the .out built from step mentioned above with CCS.
        + Run the program.
        + The Device mode MSC demo is using AM437EVM USB port #0 for device mode. Connect this USB#0 port with a PC's USB port
        + The PC connected to this USB device will notify a USB drive plugged in, and it will ask user to format the drive. Just need to format it before files can be copied and read out. 


3. Limitations / known issues:
    + Only Bulk Only Transport MSC is supported.
    + Only one USB thumb drive plugged directly to the host port is supported. No hub supported (and thus USB host on AM572x GP EVM (Beagle X15)  is not yet supported)
    + The shell "cd" command in the host example doesn't understand ".." option.
    + MSC host example codes for AM437x/AM57xx don't support unplug/replug of the USB device at the moment.

4. Change log:
    Release 1.0.0.19:
        + Add support for J721e SOC. Add BIOS examples for USB Dev and Host MSC and Dev Bulk. 
        + Support USB3.0 host mode and USB2.0 host and device mode on J721e
        + Add USB device mode CDC for AM335x.

    Release 1.0.0.18:
        + Add SMP examples for AM65xx
        + Update example memory map with new DMSC layout. Change makefile to use USB local linker files

    Release 1.0.0.17:
        + Add USB bulk performance tool. USB bulk demo has been changed to supported the performance tool
        + Add USB3.0 host support for AM65xx. USB3.0 host testings are still in progress. Not claiming operational USB3.0 at the moment
        + Add USB bulk example for AM572 GP EVM
        + Remove debug_printf in debug build of the USB LLD.
        + Fix race condition involving the bulk state and the bulk semaphores
        + Update MSC device examples to soft-reset USB core and restart USB stack for AM335x device to fix bogus bulk-in transaction that causes failed MSC device re-enumeration
        + Fix audio lag problem during setting USB audio volume on AM335x. 

    Release 1.0.0.16:
        + Fix MSC drive corruption in device mode when cable is unplugged/replugged on AM3
        + Fix USB device MSC drive re-enumeration on Windows
        + Add throughput benchmark tools for AM65x USB dev and host MSC.
            . For host MSC, use command "bm" on the host application
            . For device MSC, use included script running on a host Linux PC
        + Add USB-eMMC device mode example for BeagleBoneBlack
        + Removed obsoleted USB_EPC registers in MUSB register map
        

    Release 1.0.0.15:
        + Add support for AM65xx and add USB MSC (host, and device) and USB bulk device examples
        + Add USB generic bulk class for AM65xx
        + Add USB Dev MSC EMMC example for AM572x EVM

    Release 1.0.0.14:
        + Enable generic bulk application on all supported platfrom (example projects added for K2G,AM4,AM5,AM3)
        + Fix spurious interrupt on K2G device mode.
        + Support custom rules.mak

    Release 1.0.0.13:
        + Add support for DRA7xx: A15 and M4 USB MSC host example projects; M4 USB MSC device mode example
        + Organize all .txt project templates and RTSC cfg files to their appropriate sub directories
        + Fix USB audio crash in AM335x
        + Fix compilation error with BUILD_PROFILE=debug

    Release 1.0.0.12:
        + Added USB MSC Baremetal example for OMAPL13x.
        + Added support for USB audio functionality for OMAPL13x DSP.
        + Added support for AM574x SoC.

    Release 1.0.0.11:
        + Add USB Dev MSC to MMCSD example for AM335x (USB_DevMsc_mmcsd_evmAM335x_armExampleProject).
          Need MMCSD commit ID to work: 0b130debe6e0b2f2c73df21ecb0f3ee142f0a0a3
          Example has been tested with 32GB/64GB card formatted with FAT32 filesystem. 

        + Allow handling CPPI DMA interrupt from user application. This would 
          allow handling of USB events in a task or a main loop outside of 
          IRQ context
 
    Release 1.0.0.10:
        + Added support for OMAPL137 & OMAPL138/C6748
        + Update Examples project names

    Release 1.0.0.9:
        + Format the 16MB drive in USB device example with FAT16.
        + Fix USB host problem with AM571x
        + Fix wrong scratchpad buffer used in xHCI driver. This problem could cause random crashes
        + Fix warnings with GCCv.6
        + Initialize XHCI/DWC data so that examples still run when APP_UNCACHED_DATA_BLK3_MEM is 
          in a NO_LOAD memory section (as in AM437x SBL loadable example)

    Release 1.0.0.8:
        + Fix critical, warning, and error MISRAC issues for AM57xx
 
    Release 1.0.0.7:
        + Enable DMA for AM335x USB. DMA mode is enabled by default for AM3 for improve performace
          Disabling DMA_MODE flag to go back to FIFO mode.
        + Make USB work cache on AM437x, AM57xx, K2G. 
          All application provided buffers (the buffers that go into USBHostRead() / USBHostWrite() 
          need to be cache-size aligned.
          Also, the global structure: g_usb_global need to be in an uncached memory. In the provided examples,
          this memory is located at APP_UNCACHED_DATA_BLK3_MEM. The MMU attribute for this memory block is 
          set through the RTSC .cfg file (for RTOS example) or by the main C file (in bare metal examples)
        + Using BUILD_PROFILE environment to enable or disable debug.
        + Allowing backspace for editing shell command in USB host example
        + Allowing UART redirection to a file on USB device in USB host example
        + Fix wrong maxPacketSize information in XHCI driver
        + Fix compiler errors for AM3/AM4 bare-metal examples in previous release
        + Migrate AM3/AM4 bare-metal example interrupt, timer, and UART APIs to those of OSAL and UART LLD.

    Release 1.0.0.6:
        + Intentionally skipped.

    Release 1.0.0.5:
        + Fix some USB2.0 hung enumeration on K2G USB host
        + Fix Am57x USB wrapper initialization which could cause exception.

    Release 1.0.0.4: 
        + Support USB3.0 host mode on AM57xx IDK
        + Fix crashes in host example when typing command without parameters
        + Support K2G
        + Support make infrastructure.
        + Allow both USB instances to work in same USB mode (both host or both devices)


    Release 1.0.0.3:
        + Add support for AM57xx (USB2.0 only)
        + Add support for AM335x 


5. Performance number:
    a. AM335
        Performance increase from using DMA: Tested by connecting the AM335 running USB device MSC example to a 
        Linux PC and writing and reading a large file (about 13MB) to and from the AM335

        FIFO mode
        + Write from USB host: 13864664 bytes: from first write(0) command to the last Status: 
            5.051 (s.ms) => 2.62 MB/s
        + Read from host: 13864664 bytes: from the first Read(0) command to the ACK of the last Read(0) command: 
            4.086 (s.ms) => 3.24 MB/s

        DMA mode:
        + Write from host: 13864664 bytes: from first write(0) command to the last Status: 
            0.686 (s.ms) => 19.27 MB/s
        + Read from host: 13864664 bytes: from the first Read(0) command to the ACK of the last Read(0) command: 
            1.202 (s.ms) => 11MB/s


