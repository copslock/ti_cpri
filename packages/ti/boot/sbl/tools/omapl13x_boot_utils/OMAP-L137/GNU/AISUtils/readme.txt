 /***************************************************************
 *  TI OMAP-L137 AIS Utilities                                  *
 *  (C) 2009-2017 Texas Instruments, Inc.                       *
 *                                                              *
 ***************************************************************/
 
This directory contains the source and build files for various utilities 
associated with booting flashing the OMAP-L137 SoC device. These utilities 
include 
  HexAIS          - Command-line AIS generation utility for non-secure OMAP-L137 devices

Running
=======

These applications are built on the Microsoft's .Net framework.  To use them, you 
must run them on a system that has the Microsoft .Net framework v3.5 installed.  
Alternatively, you can run them using the open-source Mono framework v2.4 or later.
This may allow the applications to be run under Linux, since Mono exists and runs on 
that platform.

NOTE: On windows, using the mono framework requires you to use the mono executable.  With
      Mono in your path, the usage is as follows:
        mono <ApplicationName> <ApplicationParams>
      If the mono command is left off, the Microsoft framework will be used instead.
      

HexAIS tool
===========
The HexAIS and tool relies on its input parameters to be provided in INI files, 
example of which are included in the package.  The ini files are processed in order and are 
primarily used to specify what AIS Function Execute commands you want to use to initialize the 
system. The comments in the INI file themselves provide a rough documentation.

The tools have a number of options that can be specified on the command line.  You can run the 
program with the -h option to see the help screen for each program.

Command Syntax
==============

HexAIS_OMAP-L137[Options] [Input File Names]
        <Option> can be any of the following:
                -h                      Show this help screen.
                -entrypoint <Addr>      Force specified entry point for AIS boot image.
                -ini <INI file name>    Specify the ini file (default is OMAP-L137.ini).
                -otype <Output Type>    Specify type of output file, from following:.
                     binary                  Create a binary AIS file (.bin), default.
                     carray                  Create a text file with AIS data in a C array (.c).
                     srecord@addr            Create a Motorola S-record format (.srec) at addr.
                     text                    Create a text file with AIS data as ASCII text (.txt).
                -o <Output File Name>   Explicitly specify the output filename.
                                        Default is input file name with extension based on
                                        output type.
                                        
Input files (ELF or COFF object files OR Binary files) can be supplied on the command line 
or via the INI file.  Binary files must be supplied with a load address, so they are specified
in the following manner:
  <Binary File Name>@<Hex Load Address>

It is recommended to use the INI file syntax for placing and including files since 
that provides greater flexibility (see below).


Complete INI Syntax
===================

The INI file consists of a collection of INI sections.  Except for the [General] sections,
which describe configuration information for the AIS file, the INI sections are processed
in order and result in contents being added to the generated AIS output.  Each INI section
contains a collection of fields which are described below:

  Configuration
  -------------
  [GENERAL]
  ; Can be 8 or 16 - used in emifa (NOR boot mode)
  busWidth=8            

  ; EMIFA,NONE
  BootMode=Specify boot mode (typically use NONE).  Some boot modes can require 
           certain extra information beyond the actual AIS data. The tools
           may support generating this information depending on the boot mode selection.

  ; TRUE/ON or FALSE/OFF
  seqReadEn=Specify that sequential read should be used for I2C or SPI master modes. This
            is equivalent to using the [AIS_SeqReadEnable] section (see below).

  Include a file
  --------------  
  The INPUTFILE section allows including a binary or object file into the AIS boot
  image.  The result of this INI section will be one or more AIS SectionLoad or 
  EncryptedSectionLoad commands inserted into the AIS output.  Note that input files
  can be specified on the command line, but using individual [INPUTFILE] sections
  offer more control over placement and content.
  

  [INPUTFILE]
  FILENAME=Filename of the binary or object file to include in the AIS iamge
  LOADADDRESS=Address to which file contents are loaded. If specified, input file is treated as a
              binary file. For object files, the load address for loadable sections is embedded
              in the object file itself.
  ENTRYPOINTADDRESS=Allows specifying the entry point for this module. This is typically used
                    with binary files since object files embed the entry point.
  USEENTRYPOINT=Set to YES or TRUE to ensure that the entrypoint is taken from this input file
                for the JUMP_CLOSE command

                
  Insert ROM Function Calls (AIS Function Execute)
  ------------------------------------------------
  These INI sections are used to insert specific AIS Function Execute
  commands into the AIS output.
  
  ; This section allows setting the PLL system clock with a  
  ; specified multiplier and divider as shown. The clock source
  ; can also be chosen for internal or external.
  ;           |------24|------16|-------8|-------0|
  ; PLLCFG0:  |    PLLM| POSTDIV| PLLDIV3| PLLDIV5|
  ; PLLCFG1:  | CLKMODE| PLLDIV7|PLL_LOCK_TIME_CNT|
  [PLLCONFIG]
  PLLCFG0 =
  PLLCFG1 =

  ; This section lets us configure the peripheral interface
  ; of the current booting peripheral (I2C, SPI, or UART).
  ; Use with caution. The format of the PERIPHCLKCFG field 
  ; is as follows:
  ; SPI:        |------24|------16|-------8|-------0|
  ;             |           RSVD           |PRESCALE|
  ;
  ; I2C:        |------24|------16|-------8|-------0|
  ;             |  RSVD  |PRESCALE|  CLKL  |  CLKH  |
  ;
  ; UART:       |------24|------16|-------8|-------0|
  ;             | RSVD   |  OSR   |  DLH   |  DLL   |  
  [PERIPHCLKCFG]
  PERIPHCLKCFG=

  ; This section can be used to configure the PLL1 and the EMIF3a registers
  ; for starting the DDR2 interface. 
  ; See PLL1CONFIG section for the format of the PLL1CFG fields.
  ;            |------24|------16|-------8|-------0|
  ; SDCR:      |              SDCR                 |
  ; SDTIMR:    |              SDTIMR               |
  ; SDTIMR2:   |              SDTIMR2              |
  ; SDRCR:     |              SDRCR                |
  [EMIF3SDRAM]
  SDCR=
  SDTIMR=
  SDTIMR2=
  SDRCR=
  
  ; This section can be used to configure the EMIFA to use 
  ; CS0 as an SDRAM interface.  The fields required to do this
  ; are given below.
  ;                     |------24|------16|-------8|-------0|
  ; SDBCR:              |               SDBCR               |
  ; SDTIMR:             |               SDTIMR              |
  ; SDRSRPDEXIT:        |             SDRSRPDEXIT           |
  ; SDRCR:              |               SDRCR               |
  [EMIF25SDRAM]
  SDBCR=
  SDTIMR=
  SDRSRPDEXIT=
  SDRCR=

  ; This section can be used to configure the async chip selects
  ; of the EMIFA (CS2-CS5).  The fields required to do this
  ; are given below.
  ;           |------24|------16|-------8|-------0|
  ; A1CR:     |                A1CR               |
  ; A2CR:     |                A2CR               |
  ; A3CR:     |                A3CR               |
  ; A4CR:     |                A4CR               |
  [EMIF25ASYNC]
  A1CR=
  A2CR=
  A3CR=
  A4CR=
  
  ; This section should be used in place of PLL0CONFIG when
  ; the I2C, SPI, or UART modes are being used.  This ensures that 
  ; the system PLL and the peripheral's clocks are changed together.
  ; See PLLCONFIG section for the format of the PLLCFG fields.
  ; See PERIPHCLKCFG section for the format of the CLKCFG field.
  ;               |------24|------16|-------8|-------0|
  ; PLLCFG0:      |              PLLCFG               |
  ; PLLCFG1:      |              PLLCFG               |
  ; PERIPHCLKCFG: |              CLKCFG               |
  [PLLANDCLOCKCONFIG]
  PLLCFG0=
  PLLCFG1=
  PERIPHCLKCFG=

  ; This section should be used to setup the power state of modules
  ; of the two PSCs.  This section can be included multiple times to
  ; allow the configuration of any or all of the device modules.
  ;           |------24|------16|-------8|-------0|
  ; LPSCCFG:  | PSCNUM | MODULE |   PD   | STATE  |  
  [PSCCONFIG]
  LPSCCFG=

  ; This section allows setting of a single PINMUX register.
  ; This section can be included multiple times to allow setting
  ; as many PINMUX registers as needed.
  ;         |------24|------16|-------8|-------0|
  ; REGNUM: |              regNum               |
  ; MASK:   |               mask                |
  ; VALUE:  |              value                |  
  [PINMUX]
  REGNUM=
  MASK=
  VALUE=
  
  Insert AIS commands
  -------------------
  
  ; Requires no fields, simply inserts AIS Enable CRC command
  [AIS_EnableCRC]
  
  ; Requires no fields, simply inserts AIS Disable CRC command
  [AIS_DisableCRC] 
  
  ; Requires no fields, simply inserts AIS Disable CRC command
  [AIS_RequestCRC] 
  CRCVALUE=CRC value for comparison for the Request CRC command
  SEEKVALUE=Seek value for the the Request CRC command
  
  [AIS_Jump]
  LOCATION=Raw address or symbol to which to jump
  
  [AIS_JumpClose]              
  ENTRYPOINT=Raw address or symbol to which to jump upon close boot.  
  
  NOTE: The AIS JumpClose command is automatically inserted at the end of the boot
  with the currently specified entry point.  Manually inserting this command will 
  cause all commands following it to be ignored during boot, though the AIS file 
  generated will still include all those extra commands.
  
  [AIS_Set]
  TYPE=Type field for AIS SET/BOOT_TABLE command
  ADDRESS=Address field for AIS SET/BOOT_TABLE command
  DATA=Data field for AIS SET/BOOT_TABLE command
  SLEEP=Sleep field for AIS SET/BOOT_TABLE command
  
  [AIS_SectionFill]
  ADDRESS=Address field for AIS SECTION_FILL command
  SIZE=Size field for AIS SECTION_FILL command
  TYPE=Type field for AIS SECTION_FILL command
  PATTERN=Pattern field for AIS SECTION_FILL command
  
  [AIS_ReadWait]
  ADDRESS=Address field for READWAIT command
  MASK=Mask field for READWAIT command
  DATA=Data field for READWAIT command
  
  ; Requires no fields, simply inserts AIS Sequential Read command
  [AIS_SeqReadEnable] 
  
  [AIS_FinalFunctionReg]
  FINALFXNSYMBOLNAME=Symbol name of function that should be called after boot loading
                     completes but before the ROM boot loader exits.



Source
======

The source for each app will be in the application directory and in the top-level 
Common directory (in Common/AIS and Common/UtilLib).  The makefiles in each directory
give the names of all dependent source files.

Compiling
=========

- Under Windows - 
The currently supported method is to use the Cygwin enviroment and put the 
C sharp compiler's (the csc.exe executable) install location in the user's
path.  This compiler comes with the .NET Framework installation and can
usually be found in C:\WINDOWS\Microsoft.NET\Framework\<version number>.

In addition, the location of the C6000 and ARM Codegen tools should be in the 
path.

Then go to the top level directory of the package and run:
	
	make
