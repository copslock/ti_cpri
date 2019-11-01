/*
 * AIS.cs
*/

/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*/

/* 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/****************************************************************
 *  TI AIS Namespace Definitions
 *  (C) 2007-2009, Texas Instruments, Inc.                           
 *                                                              
 * Author:  Daniel Allred   
 *                                                              
 ****************************************************************/

using System;

namespace TI.AISLib
{

  /// <summary>
  /// Generic return type enumeration
  /// </summary>
  public enum retType : uint
  {
    SUCCESS = 0,
    FAIL    = 1,
    TIMEOUT = 2
  }

  /// <summary>
  /// Enum of AIS values that can be used in AIS scripts
  /// </summary>
  public enum AisOps : uint
  {
    MagicNumber     = 0x41504954,
    Section_Load    = 0x58535901,
    RequestCRC      = 0x58535902,
    EnableCRC       = 0x58535903,
    DisableCRC      = 0x58535904,
    Jump            = 0x58535905,
    Jump_Close      = 0x58535906,
    Set             = 0x58535907,
    Start_Over      = 0x58535908,
    CmpSection_Load = 0x58535909,
    Section_Fill    = 0x5853590A,
    Ping            = 0x5853590B,
    Get             = 0x5853590C,
    FunctionExec    = 0x5853590D,
    FastBoot        = 0x58535913,
    ReadWait        = 0x58535914,
    FinalFxnReg     = 0x58535915,
    SecureKeyLoad   = 0x58535920,
    EncSection_Load = 0x58535921,
    SecSection_Load = 0x58535922,
    SetSecExitMode  = 0x58535923,
    SetDelegateKey  = 0x58535924,
    RemDelegateKey  = 0x58535925,
    SeqReadEnable   = 0x58535963,
    XmtStartWord    = 0x58535441,
    RcvStartWord    = 0x52535454
  };

  public enum AisSetType : uint
  {
    BYTE    = 0x00,
    SHORT   = 0x01,
    INT     = 0x02,
    FIELD   = 0x03,
    BITS    = 0x04,
  }
  
  public enum AisSecureType:int
  {
    NONE    = 0x0,
    CUSTOM  = 0x1,
    GENERIC = 0x2
  }
  
  public enum AisHostType : uint
  {
    I2C   = 0,
    SPI   = 1,
    UART  = 2
  }
  
  public enum AisStatus : int
  {
    ERROR         = -1,
    IN_PROGRESS   =  0,
    COMPLETE      =  1
  }

  /// <summary>
  /// Enum of memory types
  /// </summary>
  public enum AisMemType : uint
  {
    EightBit = 0x1,
    SixteenBit = 0x2,
    ThirtyTwoBit = 0x4,
    SixtyFourBit = 0x8
  };
  
  public enum AisMachineType : uint
  {
    NONE,
    ARM,
    DSP
  }

  /// <summary>
  /// Enum of possible boot modes
  /// </summary>
  public enum AisBootModes : uint
  {
    NONE = 0x0,
    SPIMASTER,
    I2CMASTER,
    EMIFA,
    NAND,
    EMAC,
    UART,
    PCI,
    HPI,
    USB,
    MMC_SD,
    VLYNQ,
    RAW,
    LEGACY
  };

  /// <summary>
  /// CRC check type for the AIS stream
  /// </summary>
  public enum AisCRCCheckType : uint
  {
    NO_CRC = 0,
    SECTION_CRC = 1,
    SINGLE_CRC = 2
  };
  
} //end of TI.AIS namespace

