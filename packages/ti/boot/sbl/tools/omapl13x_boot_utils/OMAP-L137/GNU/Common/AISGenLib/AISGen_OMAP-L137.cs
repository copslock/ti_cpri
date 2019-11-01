/*
 * AISGen_OMAP-L137.cs
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
 /* --------------------------------------------------------------------------
    FILE        : AISGen_OMAP-L137.cs
    PROJECT     : TI Booting and Flashing Utilities
    AUTHOR      : Daniel Allred
    DESC        : Concrete AISGen class implemenatation for OMAP-L137
 ----------------------------------------------------------------------------- */

using System;
using System.Text;
using System.Text.RegularExpressions;
using System.IO;
using System.IO.Ports;
using System.Reflection;
using System.Threading;
using System.Globalization;
using System.Collections;
using TI.UtilLib.IO;
using TI.UtilLib.CRC;

namespace TI.AISLib
{
  /// <summary>
  /// AISGen class that is specific to the device (inherits from abtract base class AISGen)
  /// </summary>
  public class AISGen_OMAP_L137:AISGen
  {
    /// <summary>
    /// String definitions for built-in ROM functions
    /// </summary>
    public struct ROMFunctionNames
    {
      public const String PLLConfig           = "PLLConfig";
      public const String PeriphClockConfig   = "PeriphClockConfig";
      public const String EMIF3CConfigSDRAM   = "EMIF3CConfigSDRAM";
      public const String EMIF25ConfigSDRAM   = "EMIF25ConfigSDRAM";
      public const String EMIF25ConfigAsync   = "EMIF25ConfigAsync";
      public const String PLLandClockConfig   = "PLLandClockConfig";
      public const String PSCConfig           = "PSCConfig";
      public const String PINMUXConfig        = "PinMuxConfig";
      public const String FastBoot            = "FastBoot";
    }
    
    /// <summary>
    /// The public constructor for the OMAP_L137 device AIS generator.
    /// The constructor is where the device differentiation is defined.
    /// </summary>
    public AISGen_OMAP_L137() : base()
    {
      // Define the device name - used for default file names
      devNameShort = "OMAP-L137";
      devNameLong = "OMAPL137";
            
      // Define OMAP-L138 ROM boot loader functions
      ROMFunc = new AisROMFunction[9];
      
      ROMFunc[0].funcName = ROMFunctionNames.PLLConfig;
      ROMFunc[0].iniSectionName = "PLLCONFIG";
      ROMFunc[0].numParams = 2;
      ROMFunc[0].paramNames = new String[2] { "PLLCFG0", "PLLCFG1" };
      
      ROMFunc[1].funcName = ROMFunctionNames.PeriphClockConfig;
      ROMFunc[1].iniSectionName = "PERIPHCLKCFG";
      ROMFunc[1].numParams = 1;
      ROMFunc[1].paramNames = new String[1] { "PERIPHCLKCFG" };

      ROMFunc[2].funcName = ROMFunctionNames.EMIF3CConfigSDRAM;
      ROMFunc[2].iniSectionName = "EMIF3SDRAM";
      ROMFunc[2].numParams = 4;
      ROMFunc[2].paramNames = new String[4] { "SDCR", "SDTIMR", "SDTIMR2", "SDRCR" };

      ROMFunc[3].funcName = ROMFunctionNames.EMIF25ConfigSDRAM;
      ROMFunc[3].iniSectionName = "EMIF25SDRAM";
      ROMFunc[3].numParams = 4;
      ROMFunc[3].paramNames = new String[4] { "SDBCR", "SDTIMR", "SDRSRPDEXIT", "SDRCR"};
      
      ROMFunc[4].funcName = ROMFunctionNames.EMIF25ConfigAsync;
      ROMFunc[4].iniSectionName = "EMIF25ASYNC";
      ROMFunc[4].numParams = 4;
      ROMFunc[4].paramNames = new String[4] { "A1CR", "A2CR", "A3CR", "A4CR" };
      
      ROMFunc[5].funcName = ROMFunctionNames.PLLandClockConfig;
      ROMFunc[5].iniSectionName = "PLLANDCLOCKCONFIG";
      ROMFunc[5].numParams = 3;
      ROMFunc[5].paramNames = new String[3] { "PLLCFG0", "PLLCFG1", "PERIPHCLKCFG" };
      
      ROMFunc[6].funcName = ROMFunctionNames.PSCConfig;
      ROMFunc[6].iniSectionName = "PSCCONFIG";
      ROMFunc[6].numParams = 1;
      ROMFunc[6].paramNames = new String[1] { "LPSCCFG" };

      ROMFunc[7].funcName = ROMFunctionNames.PINMUXConfig;
      ROMFunc[7].iniSectionName = "PINMUX";
      ROMFunc[7].numParams = 3;
      ROMFunc[7].paramNames = new String[3] { "REGNUM", "MASK", "VALUE" };
      
      ROMFunc[8].funcName = ROMFunctionNames.FastBoot;
      ROMFunc[8].iniSectionName = "FASTBOOT";
      ROMFunc[8].numParams = 0;
      ROMFunc[8].paramNames = null;
      
      AISExtraFunc = new AisExtraFunction[4];
      AISExtraFunc[0].funcName = "setEmifB45Div";
      AISExtraFunc[0].aisExtraFileName = "DSP_AISExtra_"+devNameShort+".out";
      AISExtraFunc[0].iniSectionName = "DSP_SET_EMIFB_45DIV";
      AISExtraFunc[0].numParams = 0;      
      AISExtraFunc[0].paramNames = null;
      AISExtraFunc[0].isInitFunc = false;
      
      AISExtraFunc[1].funcName = "setEmifB45Div";
      AISExtraFunc[1].aisExtraFileName = "ARM_AISExtra_"+devNameShort+".out";
      AISExtraFunc[1].iniSectionName = "ARM_SET_EMIFB_45DIV";
      AISExtraFunc[1].numParams = 0;      
      AISExtraFunc[1].paramNames = null;
      AISExtraFunc[1].isInitFunc = false;
      
      AISExtraFunc[2].funcName = "setEmifA45Div";
      AISExtraFunc[2].aisExtraFileName = "DSP_AISExtra_"+devNameShort+".out";
      AISExtraFunc[2].iniSectionName = "DSP_SET_EMIFA_45DIV";
      AISExtraFunc[2].numParams = 0;      
      AISExtraFunc[2].paramNames = null;
      AISExtraFunc[2].isInitFunc = false;
      
      AISExtraFunc[3].funcName = "setEmifA45Div";
      AISExtraFunc[3].aisExtraFileName = "ARM_AISExtra_"+devNameShort+".out";
      AISExtraFunc[3].iniSectionName = "ARM_SET_EMIFA_45DIV";
      AISExtraFunc[3].numParams = 0;      
      AISExtraFunc[3].paramNames = null;
      AISExtraFunc[3].isInitFunc = false;      

      // OMAP-L137 is little endian
      devEndian = Endian.LittleEndian;
      
      // OMAP-L137 AIS data is little endian;
      devAISEndian = Endian.LittleEndian;

      // Create default CRC object for this device
      devCRC = new CRC32(0x04C11DB7, 0x00000000, 0x00000000, false, 1, UtilLib.CRC.CRCType.INCREMENTAL, UtilLib.CRC.CRCCalcMethod.BITWISE);
    } 
    
    public override retType InsertAISPreamble()
    {
      EndianBinaryWriter ebw = new EndianBinaryWriter( this.devAISStream, this.devEndian);
    
      switch (this.bootMode)
      {
        case AisBootModes.EMIFA:
        {
          if (this.busWidth == 16)
          {
            this.writer.Write((UInt32)(0x1 << 0)|(0x2 << 4));
          }
          else
          {
            this.writer.Write((UInt32)(0x0 << 0)|(0x2 << 4));
          }
          this.writer.Write((UInt32)AisOps.MagicNumber);
          break;
        }
        default:
        {
          this.writer.Write((UInt32)AisOps.MagicNumber);
          break;
        }
      }
      
      // Add the AIS magic number to signature buffer if a secure boot image
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.MagicNumber);
      }
      
      return retType.SUCCESS;
    }
  }
} //end of AISGenLib namespace
