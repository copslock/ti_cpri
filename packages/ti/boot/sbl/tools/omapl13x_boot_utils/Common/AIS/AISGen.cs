/*
 * AISGen.cs
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
 *  TI Abstract AISGen Class
 *  (C) 2007-2009, Texas Instruments, Inc.
 *                                                              
 * Author:  Daniel Allred
 *                                     
 ****************************************************************/

using System;
using System.Text;
using System.Text.RegularExpressions;
using System.IO;
using System.IO.Ports;
using System.Reflection;
using System.Threading;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using TI.AISLib;
using TI.UtilLib;
using TI.UtilLib.IO;
using TI.UtilLib.Ini;
using TI.UtilLib.CRC;
using TI.UtilLib.ObjectFile;

namespace TI.AISLib
{
#region Public enums and structs for AIS creation
  
  /// <summary>
  /// AIS ROM Function struct
  /// </summary>
  public struct AisROMFunction
  {
    public String funcName;
    public String iniSectionName;
    public UInt16 numParams;
    public String[] paramNames;
  }

  /// <summary>
  /// AIS Extra Function Struct
  /// </summary>
  public struct AisExtraFunction
  {
    public String funcName;
    public String iniSectionName;
    public String aisExtraFileName;
    public UInt16 numParams;
    public String[] paramNames;
    public UInt32 paramAddr;
    public UInt32 funcAddr;
    public Boolean isInitFunc;
  }

  public class InputFile
  {
    public String   fileName;
    public UInt32   loadAddr;
    public UInt32   entryPointAddr;
    public Boolean  useEntryPoint;      
  }

  public struct MemoryRange
  {
    public UInt32 startAddr;
    public UInt32 endAddr;
    
    public MemoryRange(UInt32 start, UInt32 end)
    {
      startAddr = start;  endAddr = end;
    }      
  }
#endregion

  /// <summary>
  /// Public abstract class (with static parts) to handle generic activities
  /// for device specific AISGen objects.
  /// </summary>
  public abstract partial class AISGen 
  {
  
  #region Protected Variables
    /// <summary>
    /// Short name of device 
    /// </summary>
    protected String devNameShort;

    /// <summary>
    /// Long name of device
    /// </summary>
    protected String devNameLong;

    /// <summary>
    /// The bootmode currently selected for the AIS generator.
    /// </summary>
    protected AisBootModes bootMode;

    /// <summary>
    /// An array of bytes for holding the generated AIS data.
    /// </summary>
    protected Byte[] AISData;

    /// <summary>
    /// An array of AisROMFunction objects for holding info about the device's callable AIS ROM functions. 
    /// </summary>
    protected AisROMFunction[] ROMFunc;
    
    /// <summary>
    /// An array of AisExtraFunction objects that correspond to the special functions offered in extra Object Files.
    /// </summary>
    protected AisExtraFunction[] AISExtraFunc;

    /// <summary>
    /// The current type of CRC checking that the device generator is using for AIS section load commands.
    /// This can be modified via the public property.
    /// </summary>
    protected AisCRCCheckType aisCRCType;

    /// <summary>
    /// The endianness of the device.
    /// </summary>
    protected Endian devEndian;
    
    /// <summary>
    /// The endianness of the device AIS data.
    /// </summary>
    protected Endian devAISEndian;

    protected CRC32 devCRC;
    protected UInt32 CRCStartPosition;
    protected UInt32 CRCEndPosition;
    protected Boolean crcIsEnabled;    
    
    protected UInt32 busWidth;
    protected UInt32 addrWidth;
    protected UInt32 entryPoint;

    protected AisMachineType machineType;
    
    protected Stream devAISStream;
    protected EndianBinaryWriter writer;
    
    protected Stream sigStream;
    protected EndianBinaryWriter sigWriter;
    
    // Private security type variable
    protected AisSecureType AisSecureTypeValue;
  #endregion
    
  #region Private Variables
    // List to keep track of memory regions being used
    // (used to warn about overlaps)
    private List<MemoryRange> sectionMemory;
    // List of ObjectFile objects that are being given to the 
    // AIS generating tool
    private List<ObjectFile> objectFiles;
  #endregion
  
  #region Public Properties
    /// <summary>
    /// Read or Set the type of CRC checking used in section loading.
    /// </summary>
    public AisCRCCheckType AISCRCType
    {
      get { return aisCRCType; }
      set
      {
          aisCRCType = value;
      }
    }

    /// <summary>
    /// Get or set the bootmode for the AIS generator.
    /// </summary>
    public AisBootModes BootMode
    {
      get { return bootMode; }
      set { bootMode = value; }
    }
    
    /// <summary>
    /// The AIS data.
    /// </summary>
    public Byte[] Data
    {
      get { return AISData; }
    }

    public String DeviceNameShort
    {
      get { return devNameShort; }
    }

    public String DeviceNameLong
    {
      get { return devNameLong; }
    }
    
    public UInt32 EntryPoint
    {
      get { return entryPoint; }
    }
    
    public AisMachineType MachineType
    {
      get{ return machineType; }
      set{ machineType = value; }
    }
    
    public AisSecureType SecureType
    {
      get { return AisSecureTypeValue; }
      set { AisSecureTypeValue = value; }
    }
  #endregion
    
  #region Public Contructor
    public AISGen()
    {
      // Set defaults for AISGen object
      bootMode        = AisBootModes.NONE;
      busWidth        = 8;
      addrWidth       = 16;
      entryPoint      = 0xFFFFFFFF;
      crcIsEnabled    = false;
      aisCRCType      = AisCRCCheckType.NO_CRC;
      ROMFunc         = null;
      AISExtraFunc    = null;
      writer          = null;
      devAISStream    = null;
      sigStream       = null;
      SecureType      = AisSecureType.NONE;
    }
  #endregion
  
  #region Public Virtual Methods
    public virtual retType InsertAISPreamble()
    {
      Debug.DebugMSG("Writing AIS Preamble");
      writer.Write((UInt32)AisOps.MagicNumber);
      
      // Add the AIS magic number to signature buffer if a secure boot image
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.MagicNumber);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISObjectFile( String fileName )
    {
      Debug.DebugMSG("Inserting Object File, fileName = " + fileName);
      // Since no load address is provided, we can assume ObjectFile is ElfFile or CoffFile
      if (File.Exists(fileName))
      {
        // Parse the object file
        ObjectFile file;
        if (ElfFile.IsElfFile(fileName))
        {
          file = new ElfFile(fileName);
        }
        else if (CoffFile.IsCoffFile(fileName))
        {
          file = new CoffFile(fileName);
        }
        else
        {
          Console.WriteLine("ERROR: Not a valid object file.");
          return retType.FAIL;
        }
          
        if (file != null)
        {
          // Load the object file contents
          AISObjectFileLoad(this,file);
          file.Close();
        }
        else
        {
          Console.WriteLine("ERROR: Parsing the input file {0} failed!",fileName);
        }
      }
      else
      {
        Console.WriteLine("WARNING: File {0} does not exist. Ignoring insert command.",fileName);
      }
      
      return retType.SUCCESS;
    }
    
    public virtual retType InsertAISObjectFile( String fileName, Boolean useEntryPoint )
    {
      // Since no load address is provided, we can assume ObjectFile is ElfFile or CoffFile
      if (InsertAISObjectFile( fileName ) != retType.SUCCESS)
      {
        return retType.FAIL;
      }
      
      // Set AIS entry point
      if (useEntryPoint)
      {
        ObjectFile file = FindObjectFile( this, fileName );
      
        this.entryPoint = (UInt32) file.EntryPoint;          
      }
      
      return retType.SUCCESS;
    }
    
    public virtual retType InsertAISObjectFile( String fileName, UInt32 loadAddr )
    {
      Debug.DebugMSG("Inserting Object File, fileName = " + fileName);
      // Since a load address is provided, we can assume ObjectFile is BinaryFile
      if (File.Exists(fileName))
      {
        // Parse the object file
        ObjectFile file = new BinaryFile(fileName, loadAddr);
          
        if (file != null)
        {
          // Load the object file contents
          AISObjectFileLoad(this, file);
          file.Close();
        }
        else
        {
          Console.WriteLine("ERROR: Parsing the input file {0} failed!",fileName);
          return retType.FAIL;          
        }
      }
      else
      {
        Console.WriteLine("ERROR: File {0} does not exist! ",fileName);
        return retType.FAIL;
      }
      
      return retType.SUCCESS;
    }
    
    public virtual retType InsertAISObjectFile( String fileName, UInt32 loadAddr, UInt32 entryPoint )
    {
      // Since a load address is provided, we can assume ObjectFile is BinaryFile
      if (InsertAISObjectFile( fileName, loadAddr) != retType.SUCCESS)
      {
        return retType.FAIL;
      }
      
      if (entryPoint != 0xFFFFFFFF)
      {
        this.entryPoint = entryPoint;
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISEnableCRC()
    {
      Debug.DebugMSG("Enable CRC command called.");
      if ( this.SecureType == AisSecureType.NONE )
      {
        if (this.aisCRCType != AisCRCCheckType.NO_CRC)
        {
          if (!crcIsEnabled)
          {
            Debug.DebugMSG("Inserting Enable CRC command.");
            writer.Write((UInt32)AisOps.EnableCRC);
            this.CRCStartPosition = (UInt32) this.devAISStream.Position;
            crcIsEnabled = true;
          }
        }
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISDisableCRC()
    {
      Debug.DebugMSG("Disable CRC command called.");
      if ( this.SecureType == AisSecureType.NONE )
      {
        if (this.aisCRCType != AisCRCCheckType.NO_CRC)
        {
          if (crcIsEnabled)
          {
            Debug.DebugMSG("Inserting Disable CRC command.");
            writer.Write((UInt32)AisOps.DisableCRC);
            crcIsEnabled = false;
          }
        }
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISRequestCRC(UInt32 crcValue, Int32 seekValue)
    {
      // For secure boot, silently discard CRC commands
      if ( this.SecureType == AisSecureType.NONE )
      {
        Debug.DebugMSG("Request CRC command called.");
        if (this.aisCRCType != AisCRCCheckType.NO_CRC)
        {
          if (crcIsEnabled)
          {
            Debug.DebugMSG("Inserting Request CRC command.");
            writer.Write((UInt32)AisOps.RequestCRC);
            writer.Write( crcValue );
            writer.Write( seekValue );
          }
        }
      }
      
      return retType.SUCCESS;
    }
    
    public virtual retType InsertAISRequestCRC(Int32 seekValue)
    {
      return InsertAISRequestCRC(this.devCRC.CurrentCRC, seekValue);
    }
    
    public virtual retType InsertAISRequestCRC()
    {
      Int32 seekValue;
      
      this.CRCEndPosition = (UInt32) this.devAISStream.Position;
      seekValue = (Int32)(-1) * (Int32)(12 + this.CRCEndPosition - this.CRCStartPosition);
      
      return InsertAISRequestCRC(this.devCRC.CurrentCRC, seekValue);
    }
  
    public virtual retType InsertAISFunctionExecute(UInt16 fxnNum, UInt16 argCnt, UInt32[] args)
    {
      Debug.DebugMSG("Inserting Function Execute command.");
      writer.Write((UInt32)AisOps.FunctionExec);
      writer.Write( (((UInt32)argCnt) << 16) | (((UInt32)fxnNum) << 0) );
      writer.Write(args);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.FunctionExec);
        sigWriter.Write( (((UInt32)argCnt) << 16) | (((UInt32)fxnNum) << 0) );
        sigWriter.Write(args);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISSectionLoad(UInt32 address, UInt32 size, Byte[] data)
    {
      Debug.DebugMSG("Inserting AIS Section Load command.");
      writer.Write((UInt32)AisOps.Section_Load);
      writer.Write(address);
      writer.Write(size);
      writer.Write(data);
     
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.Section_Load);
        sigWriter.Write(address);
        sigWriter.Write(size);
        sigWriter.Write(data);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISJump(String symbolName)
    {
      ObjectSymbol sym = FindSymbol(this, symbolName);
      
      if (sym != null)
      {
        return InsertAISJump((UInt32)sym.value);
      }
      
      return retType.FAIL;
    }
    
    public virtual retType InsertAISJump(UInt32 address)
    {
      Debug.DebugMSG("Inserting AIS Jump command.");
    
      writer.Write((UInt32)AisOps.Jump);
      writer.Write(address);
      
      // Add the data to signature buffer if a secure boot image
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.Jump);
        sigWriter.Write(address);
      }
      
      return retType.SUCCESS;
    }

    public virtual retType InsertAISJumpClose(String symbolName)
    {
      ObjectSymbol sym = FindSymbol(this, symbolName);
      
      if (sym != null)
      {
        return InsertAISJumpClose((UInt32)sym.value);
      }
      
      return retType.FAIL;  
    }

    public virtual retType InsertAISJumpClose(UInt32 address)
    {
      Debug.DebugMSG("Inserting AIS JumpClose command, jumping to 0x" + address.ToString("X8"));
    
      writer.Write((UInt32)AisOps.Jump_Close);
      writer.Write(address);
      
      // Add the data to signature buffer if a secure boot image
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.Jump_Close);
        sigWriter.Write(address);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISSet(UInt32 type, UInt32 address, UInt32 data, UInt32 sleep)
    {
      Debug.DebugMSG("Inserting AIS Set command, writing to 0x" + address.ToString("X8"));
    
      // Write SET command
      writer.Write((UInt32)AisOps.Set);
      
      // Write type field (32-bit only)
      writer.Write(type);
      
      // Write appropriate parameter address
      writer.Write(address);
      
      // Write data to write
      writer.Write(data);
      
      // Write Sleep value 
      writer.Write(sleep);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        // Write SET command
        sigWriter.Write((UInt32)AisOps.Set);
      
        // Write type field (32-bit only)
        sigWriter.Write(type);
        
        // Write appropriate parameter address
        sigWriter.Write(address);
        
        // Write data to write
        sigWriter.Write(data);
        
        // Write Sleep value 
        sigWriter.Write(sleep);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISSectionFill(UInt32 address, UInt32 size, UInt32 type, UInt32 pattern )
    {
      Debug.DebugMSG("Inserting AIS Section Fill command, writing to 0x" + address.ToString("X8"));
      
      // Write SECTION FILL command
      writer.Write((UInt32)AisOps.Section_Fill);
      
      // Write section start address
      writer.Write(address);
      
      // Write section size
      writer.Write(size);
      
      // Write memory access type
      writer.Write(type);
      
      // Write data pattern
      writer.Write(pattern);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        // Write SECTION FILL command
        sigWriter.Write((UInt32)AisOps.Section_Fill);
        
        // Write section start address
        sigWriter.Write(address);
        
        // Write section size
        sigWriter.Write(size);
        
        // Write memory access type
        sigWriter.Write(type);
        
        // Write data pattern
        sigWriter.Write(pattern);
      }
            
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISReadWait(UInt32 address, UInt32 mask, UInt32 data )
    {
      Debug.DebugMSG("Inserting AIS ReadWait command, reading from 0x" + address.ToString("X8"));
    
      writer.Write((UInt32)AisOps.ReadWait);
      writer.Write(address);
      writer.Write(mask);
      writer.Write(data);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.ReadWait);
        sigWriter.Write(address);
        sigWriter.Write(mask);
        sigWriter.Write(data);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISFastBoot()
    {
      Debug.DebugMSG("Inserting AIS FastBoot command");
    
      writer.Write((UInt32)AisOps.FastBoot);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        // Write SET command
        sigWriter.Write((UInt32)AisOps.FastBoot);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISFinalFxnReg( String symbolName)
    {
      Debug.DebugMSG("Inserting AIS FinalFxnReg command, fxnName = " + symbolName);
    
      ObjectSymbol sym = FindSymbol(this, symbolName);
      
      if (sym != null)
      {
        InsertAISFinalFxnReg((UInt32)sym.value);
        return retType.SUCCESS;
      }
      
      return retType.FAIL;  
    }
  
    public virtual retType InsertAISFinalFxnReg(UInt32 address)
    {
      Debug.DebugMSG("Inserting AIS FinalFxnReg command, fxnAddress = " + address.ToString("X8"));
    
      writer.Write((UInt32)AisOps.FinalFxnReg);
      writer.Write(address);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.FinalFxnReg);
        sigWriter.Write(address);
      }
      
      return retType.SUCCESS;
    }
  
    public virtual retType InsertAISSeqReadEnable()
    {
      Debug.DebugMSG("Inserting AIS SeqReadEnable command");
    
      writer.Write((UInt32)AisOps.SeqReadEnable);
      
      if ( this.SecureType != AisSecureType.NONE )
      {
        sigWriter.Write((UInt32)AisOps.SeqReadEnable);
      }
      
      return retType.SUCCESS;
    } 
  #endregion

  #region Public Static Methods
    /// <summary>
    /// genAIS command.  Always use section-by-section CRC checks
    /// </summary>
    /// <param name="mainObjectFileName"></param>
    /// <param name="bootMode"></param>
    /// <returns>Bytes of the binary or text AIS command</returns>
    public static Byte[] GenAIS( AISGen devAISGen, 
                                 List<String> inputFileNames,
                                 IniFile iniFile )
    {
      UInt32 numWords;
      
      // Setup the binary writer to generate the temp AIS file
      devAISGen.devAISStream = new MemoryStream();
      using ( devAISGen.writer = new EndianBinaryWriter( devAISGen.devAISStream, devAISGen.devEndian) )
      {
        // List to keep track of loadable sections and their occupied memory ranges
        devAISGen.sectionMemory = new List<MemoryRange>();  
        // Initiate list to keep track of the input files
        devAISGen.objectFiles   = new List<ObjectFile>();
        
        // Get data from the GENERAL INI Section
        GeneralIniSectionParse(devAISGen, iniFile);
        
        #region Main AIS Generation      
        // ---------------------------------------------------------
        // ****************** BEGIN AIS GENERATION *****************
        // ---------------------------------------------------------
        Console.WriteLine("Begining the AIS file generation.");
        
        // Diaplay currently selected boot mode
        Console.WriteLine("AIS file being generated for bootmode: {0}.",Enum.GetName(typeof(AisBootModes),devAISGen.bootMode));
        
        // Write the premilinary header and fields (everything before first AIS command)
        devAISGen.InsertAISPreamble();
        Debug.DebugMSG("Preamble Complete");
        
        // Parse the INI sections in order, inserting needed AIS commands
        if (iniFile != null)
        {
          foreach(IniSection sec in iniFile.Sections)
          {
            InsertAISCommandViaINI(devAISGen, sec);
          }
          Debug.DebugMSG("INI parsing complete");
        }
        
        // Insert the object file passed in on the top-level (if it exists)
        if (inputFileNames != null)
        {
          foreach (String fn in inputFileNames)
          {
            String[] nameAndAddr = fn.Split('@');
            
            Debug.DebugMSG("Inserting file " + nameAndAddr[0]);
            
            if (!File.Exists(nameAndAddr[0]))
            {
              Console.WriteLine("ERROR: {0} does not exist. Aborting...", nameAndAddr[0]);
              return null;
            }
            
            if (nameAndAddr.Length == 2)
            {
              UInt32 loadAddr;
              
              nameAndAddr[1] = nameAndAddr[1].ToLower();
              
              if (nameAndAddr[1].StartsWith("0x"))
              {
                if (!UInt32.TryParse(nameAndAddr[1].Replace("0x", ""), NumberStyles.HexNumber, null, out loadAddr))
                {
                  Console.WriteLine("WARNING: Invalid address format, {0}. Ignoring...", nameAndAddr[1]);
                }
                else
                {
                  devAISGen.InsertAISObjectFile(nameAndAddr[0], loadAddr);
                }
              }
              else if (UInt32.TryParse(nameAndAddr[1], out loadAddr))
              {
                devAISGen.InsertAISObjectFile(nameAndAddr[0], loadAddr);
              }
              else
              {
                Console.WriteLine("WARNING: Invalid address format, {0}. Ignoring...", nameAndAddr[1]);
              }
            }
            else if (nameAndAddr.Length == 1)
            {
              // If we still have not had a valid entry point set, then use entry point from 
              // first encountered non-binary file in the inputFileNames list
              if (devAISGen.entryPoint == 0xFFFFFFFF)
              {
                devAISGen.InsertAISObjectFile(nameAndAddr[0], true);
              }
              else
              {
                devAISGen.InsertAISObjectFile(nameAndAddr[0], false);
              }
            }
            else
            {
              Console.WriteLine("WARNING: Invalid filename format, {0}. Ignoring...", fn);
            }
          }
          
          Debug.DebugMSG("Main input file insertion complete.");
        }

        // If CRC type is for single CRC, send Request CRC now
        if (devAISGen.aisCRCType == AisCRCCheckType.SINGLE_CRC)
        {
          devAISGen.InsertAISRequestCRC();
        }
        
        // Insert closing JumpClose AIS command (issue warning)
        if (devAISGen.entryPoint == 0xFFFFFFFF)
        {
          // No valid entry point was ever set (issue warning)
          Console.WriteLine("WARNING: Entry point set to null pointer!");
          devAISGen.InsertAISJumpClose(0x00000000);
        }
        else
        {
          devAISGen.InsertAISJumpClose(devAISGen.entryPoint);
        }
        
        
        // Flush the data and then return to start
        devAISGen.devAISStream.Flush();
        devAISGen.devAISStream.Seek(0,SeekOrigin.Begin);
        
        Console.WriteLine("AIS file generation was successful.");
        // ---------------------------------------------------------
        // ******************* END AIS GENERATION ******************
        // ---------------------------------------------------------

      #endregion
      
        // Now create return Byte array based on tempAIS file and the bootmode
        EndianBinaryReader tempAIS_br = new EndianBinaryReader(devAISGen.devAISStream, Endian.LittleEndian);
      
        // Setup the binary reader object
        numWords = ((UInt32)tempAIS_br.BaseStream.Length) >> 2;
        devAISGen.AISData = new Byte[numWords << 2];   //Each word converts to 4 binary bytes
        
        Debug.DebugMSG("Number of words in the AIS output is {0}", numWords);

        // Copy the data to the output Byte array
        for (UInt32 i = 0; i < numWords; i++)
        {
          BitConverter.GetBytes(tempAIS_br.ReadUInt32()).CopyTo(devAISGen.AISData, i * 4);
        }

        // Close the binary reader
        tempAIS_br.Close();  
      }
      
      // Dispose of all object files
      foreach (ObjectFile file in devAISGen.objectFiles)
      {
        try
        {
          file.Dispose();
        }
        catch (Exception e)
        {
          Console.WriteLine(e.Message);
        }
      }

      // Clean up any embedded file resources that may have been extracted
      EmbeddedFileIO.CleanUpEmbeddedFiles();

      // Return Byte Array
      return devAISGen.AISData;
    }

          
    /// <summary>
    /// genAIS command.  Always use section-by-section CRC checks
    /// </summary>
    /// <param name="mainObjectFileName"></param>
    /// <param name="bootMode"></param>
    /// <returns>Bytes of the binary or text AIS command</returns>
    public static Byte[] GenAIS( AISGen devAISGen,
                                 List<String> inputFileNames,
                                 String iniData )
    {
      return GenAIS(devAISGen, inputFileNames, new IniFile(iniData));
    }
    
    /// <summary>
    /// Secondary genAIS that calls the first
    /// </summary>
    /// <param name="mainObjectFileName">File name of .out file</param>
    /// <param name="bootmode">String containing desired boot mode</param>
    /// <returns>an Array of bytes to write to create an AIS file</returns>
    public static Byte[] GenAIS( AISGen devAISGen,
                                 List<String> inputFileNames,
                                 String bootmode,
                                 IniFile iniFile )
    {
      devAISGen.bootMode = (AisBootModes)Enum.Parse(typeof(AisBootModes), bootmode, true);
      Console.WriteLine("Chosen bootmode is {0}.", devAISGen.bootMode.ToString());
      return GenAIS(devAISGen, inputFileNames, iniFile);
    }
    
    /// <summary>
    /// Secondary genAIS that calls the first
    /// </summary>
    /// <param name="mainObjectFileName">File name of .out file</param>
    /// <param name="bootmode">String containing desired boot mode</param>
    /// <returns>an Array of bytes to write to create an AIS file</returns>
    public static Byte[] GenAIS( AISGen devAISGen,
                                 List<String> inputFileNames,
                                 String bootmode,
                                 String iniData )
    {
      devAISGen.bootMode = (AisBootModes)Enum.Parse(typeof(AisBootModes), bootmode, true);
      Console.WriteLine("Chosen bootmode is {0}.", devAISGen.bootMode.ToString());
      return GenAIS(devAISGen, inputFileNames, iniData);
    }
    
    /// <summary>
    /// Secondary genAIS that calls the first
    /// </summary>
    /// <param name="mainObjectFileName">File name of .out file</param>
    /// <param name="bootmode">AISGen.AisBootModes Enum value containing desired boot mode</param>
    /// <returns>an Array of bytes to write to create an AIS file</returns>
    public static Byte[] GenAIS( AISGen devAISGen,
                                 List<String> inputFileNames,
                                 AisBootModes bootmode,
                                 IniFile iniFile )
    {
      devAISGen.bootMode = bootmode;
      Console.WriteLine("Chosen bootmode is {0}.", devAISGen.bootMode.ToString());
      return GenAIS(devAISGen, inputFileNames, iniFile);
    }
    
    /// <summary>
    /// Secondary genAIS that calls the first
    /// </summary>
    /// <param name="mainObjectFileName">File name of .out file</param>
    /// <param name="bootmode">AISGen.AisBootModes Enum value containing desired boot mode</param>
    /// <returns>an Array of bytes to write to create an AIS file</returns>
    public static Byte[] GenAIS( AISGen devAISGen,
                                 List<String> inputFileNames,
                                 AisBootModes bootmode,
                                 String iniData )
    {
      devAISGen.bootMode = bootmode;
      Console.WriteLine("Chosen bootmode is {0}.", devAISGen.bootMode.ToString());
      return GenAIS(devAISGen, inputFileNames, iniData);
    }

    
    public static retType InsertAISCommandViaINI(AISGen devAISGen, IniSection sec)
    {
      #region Handle Input Binary and Object Files
      if (sec.sectionName.Equals("INPUTFILE", StringComparison.OrdinalIgnoreCase))
      {
        String fileName = null;
        Boolean useEntryPoint = false;
        UInt32 loadAddr = 0xFFFFFFFF;
        UInt32 entryPointAddr = 0xFFFFFFFF;
        
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          // File name for binary section data
          if (((String)de.Key).Equals("FILENAME", StringComparison.OrdinalIgnoreCase))
          {
            fileName = (String) sec.sectionValues["FILENAME"];
          }
          
          // Binary section's load address in the memory map
          if (((String)de.Key).Equals("LOADADDRESS", StringComparison.OrdinalIgnoreCase))
          {
            loadAddr = (UInt32) sec.sectionValues["LOADADDRESS"];
          }
          
          // Binary section's entry point address in the memory map
          if (((String)de.Key).Equals("ENTRYPOINTADDRESS", StringComparison.OrdinalIgnoreCase))
          {
            entryPointAddr = (UInt32) sec.sectionValues["ENTRYPOINTADDRESS"];
          }
          
          // Option to specify that this entry point should be used for AIS
          if (((String)de.Key).Equals("USEENTRYPOINT", StringComparison.OrdinalIgnoreCase))
          {
            if (((String)sec.sectionValues["USEENTRYPOINT"]).Equals("YES", StringComparison.OrdinalIgnoreCase))
            {
              useEntryPoint = true;
            }
            else if (((String)sec.sectionValues["USEENTRYPOINT"]).Equals("TRUE", StringComparison.OrdinalIgnoreCase))
            {
              useEntryPoint = true;
            }
          }
        }
        
        if (fileName == null)
        {
          Console.WriteLine("ERROR: File name must be provided in INPUTFILE section.");
          return retType.FAIL;
        }
        
        // Insert the file into the AIS image
        if ( loadAddr != 0xFFFFFFFF )
        {
          // binary image
          if ( entryPointAddr != 0xFFFFFFFF )
          {
            devAISGen.InsertAISObjectFile(fileName, loadAddr, entryPointAddr);
          }
          else
          {
            devAISGen.InsertAISObjectFile(fileName, loadAddr);
          }
        }
        else
        {
          if ( entryPointAddr != 0xFFFFFFFF )
          {
            devAISGen.InsertAISObjectFile(fileName,true);
          }
          else
          {
            devAISGen.InsertAISObjectFile(fileName,useEntryPoint);
          }
        }
        return retType.SUCCESS;
      }
      #endregion
      
      #region Handle ROM and AIS Extra Functions  
      // Handle ROM functions
      if (devAISGen.ROMFunc != null)
      {
        for (UInt32 j = 0; j < devAISGen.ROMFunc.Length; j++)
        {
          if (sec.sectionName.Equals(devAISGen.ROMFunc[j].iniSectionName, StringComparison.OrdinalIgnoreCase))
          {
            UInt32 funcIndex = j;

            UInt32[] args = new UInt32[ (UInt32)devAISGen.ROMFunc[funcIndex].numParams ];
            
            for (Int32 k = 0; k < devAISGen.ROMFunc[funcIndex].numParams; k++)
            {
              Debug.DebugMSG("\tParam name: {0}, Param num: {1}, Value: {2}\n",
                devAISGen.ROMFunc[funcIndex].paramNames[k],
                k, 
                sec.sectionValues[devAISGen.ROMFunc[funcIndex].paramNames[k].ToUpper()]);
              try
              {              
                args[k] = (UInt32) sec.sectionValues[devAISGen.ROMFunc[funcIndex].paramNames[k].ToUpper()];
              }
              catch
              {
                Console.WriteLine("WARNING: INI Section {0} is malformed - {1} parameter not provided. Ignoring section contens.",sec.sectionName, devAISGen.ROMFunc[funcIndex].paramNames[k].ToUpper());
                return retType.SUCCESS;
              }
            }

            devAISGen.InsertAISFunctionExecute((UInt16) funcIndex, (UInt16) devAISGen.ROMFunc[funcIndex].numParams, args);
            
            return retType.SUCCESS;
          }
        }
      }
      
      // Handle AISExtras functions
      if (devAISGen.AISExtraFunc != null)
      {
        for (UInt32 j = 0; j < devAISGen.AISExtraFunc.Length; j++)
        {
          if (sec.sectionName.Equals(devAISGen.AISExtraFunc[j].iniSectionName, StringComparison.OrdinalIgnoreCase))
          {
            UInt32 funcIndex = j;
            
            UInt32[] args = new UInt32[ (UInt32)devAISGen.AISExtraFunc[j].numParams ];
            
            // Load the AIS extras file if needed
            {
              IniSection tempSec = new IniSection();
              tempSec.sectionName = "INPUTFILE";
              tempSec.sectionValues = new Hashtable();
              tempSec.sectionValues["FILENAME"] = devAISGen.AISExtraFunc[funcIndex].aisExtraFileName;
              
              EmbeddedFileIO.ExtractFile(Assembly.GetExecutingAssembly(), devAISGen.AISExtraFunc[funcIndex].aisExtraFileName, true);
                    
              InsertAISCommandViaINI(devAISGen, tempSec);
              
              Debug.DebugMSG("AISExtras file loaded.\n");
              
              // Use symbols to get address for AISExtra functions and parameters
              for (Int32 k = 0; k < devAISGen.AISExtraFunc.Length; k++)
              {
                ObjectFile tempFile = FindFileWithSymbol(devAISGen, devAISGen.AISExtraFunc[funcIndex].funcName);
                if (tempFile == null)
                {
                  // Try looking for underscore version
                  tempFile = FindFileWithSymbol(devAISGen, "_" + devAISGen.AISExtraFunc[funcIndex].funcName);
                }
                
                if (tempFile != null)
                {
                  ObjectSymbol tempSym = tempFile.symFind(devAISGen.AISExtraFunc[funcIndex].funcName);
                  if (tempSym == null)
                  {
                    // Try looking for underscore version
                    tempSym = tempFile.symFind("_"+devAISGen.AISExtraFunc[funcIndex].funcName);
                  }
                  
                  if (tempSym != null)
                  {
                    devAISGen.AISExtraFunc[funcIndex].funcAddr = (UInt32) tempSym.value;
                    ObjectSection tempObjSec = tempFile.secFind(".params");
                    if (tempObjSec == null)
                    {
                      Console.WriteLine(".params section not found in file {0}.", 
                                        devAISGen.AISExtraFunc[funcIndex].aisExtraFileName);
                      return retType.FAIL;
                    }
                    else
                    {
                      devAISGen.AISExtraFunc[funcIndex].paramAddr = (UInt32) tempObjSec.runAddr;
                    }
                  }
                  else
                  {
                    Console.WriteLine("AIS extra function, {0}, not found in file {1}.", 
                                      devAISGen.AISExtraFunc[funcIndex].funcName, 
                                      devAISGen.AISExtraFunc[funcIndex].aisExtraFileName);
                    return retType.FAIL;
                  }
                }
                else
                {
                  // The function name was not found - that's a big problem with our 
                  // device specific AISGen class.
                  Console.WriteLine("AIS extra function, {0}, not found in file {1}.", 
                                    devAISGen.AISExtraFunc[funcIndex].funcName, 
                                    devAISGen.AISExtraFunc[funcIndex].aisExtraFileName);
                  return retType.FAIL;
                }
              }
            }
            
            Debug.DebugMSG("Found required sections and symbols in AISExtras file.\n");
            
            // Validate input parameters
            for (Int32 k = 0; k < devAISGen.AISExtraFunc[funcIndex].numParams; k++)
            {
              try
              {              
                args[k] = (UInt32) sec.sectionValues[devAISGen.AISExtraFunc[funcIndex].paramNames[k].ToUpper()];
              }
              catch
              {
                Console.WriteLine("WARNING: INI Section {0} is malformed - {1} parameter not provided. Ignoring section contens.",sec.sectionName, devAISGen.ROMFunc[funcIndex].paramNames[k].ToUpper());
                return retType.SUCCESS;
              }
            }
            
            Debug.DebugMSG("Input parameter validation for AISExtras function is complete.\n");
            
            // Write SET command for each input parameter
            for (Int32 k = 0; k < devAISGen.AISExtraFunc[funcIndex].numParams; k++)
            {
              devAISGen.InsertAISSet(
                (UInt32)AisSetType.INT,    // Write type field (32-bit only)
                (UInt32) (devAISGen.AISExtraFunc[funcIndex].paramAddr + (k * 4)), 
                args[k],
                (UInt32)0x0 );  // Write Sleep value (should always be zero)
            }

            // Now that params are set, Jump to function
            devAISGen.InsertAISJump(devAISGen.AISExtraFunc[funcIndex].funcAddr);
            
            return retType.SUCCESS;
          }
        }
      }
      #endregion
      
      #region Handle AIS Command Sections
      
      if (sec.sectionName.Equals("AIS_EnableCRC", StringComparison.OrdinalIgnoreCase))
      {
        devAISGen.InsertAISEnableCRC();
      }
      
      else if (sec.sectionName.Equals("AIS_DisableCRC", StringComparison.OrdinalIgnoreCase))
      {
        devAISGen.InsertAISDisableCRC();
      }
      
      else if (sec.sectionName.Equals("AIS_RequestCRC", StringComparison.OrdinalIgnoreCase))
      {
        UInt32 crcValue = 0x00000000;
        Int32 seekValue = -12;
        
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (((String)de.Key).Equals("CRCValue", StringComparison.OrdinalIgnoreCase))
          {
            crcValue = (UInt32)sec.sectionValues["CRCVALUE"];
          }
          if (((String)de.Key).Equals("SEEKValue", StringComparison.OrdinalIgnoreCase))
          {
            seekValue = (Int32)sec.sectionValues["SEEKVALUE"];
          }
        }
        if (devAISGen.InsertAISRequestCRC(crcValue, seekValue) != retType.SUCCESS)
        {
          Console.WriteLine("WARNING: Final function register AIS command failed.");
        }
      }
      
      else if (sec.sectionName.Equals("AIS_Jump", StringComparison.OrdinalIgnoreCase))
      {
        String symbolName = "";
        UInt32 address = 0x00000000;
      
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (((String)de.Key).Equals("LOCATION", StringComparison.OrdinalIgnoreCase))
          {
            symbolName = sec.sectionValues["LOCATION"].ToString();
          }
        }
        // See if string is number (address)
        if (UInt32.TryParse(symbolName, out address))
        {
          if (devAISGen.InsertAISJump(address) != retType.SUCCESS)
          {
            Console.WriteLine("WARNING: AIS Jump to {0} was not inserted.",symbolName);
          }
        }
        else
        {
          if (devAISGen.InsertAISJump(symbolName) != retType.SUCCESS)
          {
            Console.WriteLine("WARNING: AIS Jump to {0} was not inserted.",symbolName);
          }
        }
      }
      
      else if (sec.sectionName.Equals("AIS_JumpClose", StringComparison.OrdinalIgnoreCase))
      {
        String symbolName = "";
        UInt32 address = 0x00000000;
      
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (((String)de.Key).Equals("ENTRYPOINT", StringComparison.OrdinalIgnoreCase))
          {
            symbolName = (String)sec.sectionValues["ENTRYPOINT"];
          }
        }
        
        if (symbolName == "")
        {
          devAISGen.InsertAISJumpClose(devAISGen.entryPoint);
        }
        else
        {
          // See if string is number (address)
          if (UInt32.TryParse(symbolName, out address))
          {
            if (devAISGen.InsertAISJumpClose(address) != retType.SUCCESS)
            {
              Console.WriteLine("WARNING: AIS Jump to {0} was not inserted.",symbolName);
            }
          }
          else
          {
            if (devAISGen.InsertAISJumpClose(symbolName) != retType.SUCCESS)
            {
              Console.WriteLine("WARNING: AIS Jump to {0} was not inserted.",symbolName);
            }
          }
        }
      }
      
      else if (sec.sectionName.Equals("AIS_Set", StringComparison.OrdinalIgnoreCase))
      {
        UInt32 type     = 0x00000000;
        UInt32 address  = 0x00000000;
        UInt32 data     = 0x00000000;
        UInt32 sleep    = 0x00000000;
      
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (sec.sectionValues["TYPE"].GetType() == typeof(String))
          {          
            if (((String)de.Key).Equals("TYPE", StringComparison.OrdinalIgnoreCase))
            {
              if (! UInt32.TryParse((String)sec.sectionValues["TYPE"], out type))
              {
                try
                {
                  type = (UInt32)Enum.Parse(typeof(AisSetType),(String)sec.sectionValues["TYPE"]);
                }
                catch (ArgumentException e)
                {
                  Console.WriteLine((String)sec.sectionValues["TYPE"] + " is not allowed specifier for SET type.");
                  Console.WriteLine(e.Message);
                  return retType.FAIL;
                }
              }
            }
          }
          else
          {
            type = (UInt32)sec.sectionValues["TYPE"];
          }
          if (((String)de.Key).Equals("ADDRESS", StringComparison.OrdinalIgnoreCase))
          {
            address = (UInt32)sec.sectionValues["ADDRESS"];
          }
          if (((String)de.Key).Equals("DATA", StringComparison.OrdinalIgnoreCase))
          {
            data = (UInt32)sec.sectionValues["DATA"];
          }
          if (((String)de.Key).Equals("SLEEP", StringComparison.OrdinalIgnoreCase))
          {
            sleep = (UInt32)sec.sectionValues["SLEEP"];
          }
          
        }
        devAISGen.InsertAISSet(type, address, data, sleep);
      }
      
      else if (sec.sectionName.Equals("AIS_SectionFill", StringComparison.OrdinalIgnoreCase))
      {
        UInt32 address  = 0x00000000;
        UInt32 size     = 0x00000000;
        UInt32 type     = 0x00000000;
        UInt32 pattern  = 0x00000000;
      
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (((String)de.Key).Equals("ADDRESS", StringComparison.OrdinalIgnoreCase))
          {
            address = (UInt32)sec.sectionValues["ADDRESS"];
          }
          if (((String)de.Key).Equals("SIZE", StringComparison.OrdinalIgnoreCase))
          {
            size = (UInt32)sec.sectionValues["SIZE"];
          }
          if (((String)de.Key).Equals("TYPE", StringComparison.OrdinalIgnoreCase))
          {
            type = (UInt32)sec.sectionValues["TYPE"];
          }
          if (((String)de.Key).Equals("PATTERN", StringComparison.OrdinalIgnoreCase))
          {
            pattern = (UInt32)sec.sectionValues["PATTERN"];
          }
        }
        devAISGen.InsertAISSectionFill( address, size, type, pattern);
      }
      
      else if (sec.sectionName.Equals("AIS_FastBoot", StringComparison.OrdinalIgnoreCase))
      {
        devAISGen.InsertAISFastBoot();
      }
      
      else if (sec.sectionName.Equals("AIS_ReadWait", StringComparison.OrdinalIgnoreCase))
      {
        UInt32 address  = 0x00000000;
        UInt32 mask     = 0xFFFFFFFF;
        UInt32 data     = 0xFFFFFFFF;
      
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (((String)de.Key).Equals("ADDRESS", StringComparison.OrdinalIgnoreCase))
          {
            address = (UInt32)sec.sectionValues["ADDRESS"];
          }
          if (((String)de.Key).Equals("MASK", StringComparison.OrdinalIgnoreCase))
          {
            mask = (UInt32)sec.sectionValues["MASK"];
          }
          if (((String)de.Key).Equals("DATA", StringComparison.OrdinalIgnoreCase))
          {
            data = (UInt32)sec.sectionValues["DATA"];
          }
        }
        devAISGen.InsertAISReadWait(address, mask, data);
      }
      
      else if (sec.sectionName.Equals("AIS_SeqReadEnable", StringComparison.OrdinalIgnoreCase))
      {
        devAISGen.InsertAISSeqReadEnable();
      }
      
      else if (sec.sectionName.Equals("AIS_FinalFunctionReg", StringComparison.OrdinalIgnoreCase))
      {
        String finalFxnName = "";
        
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          if (((String)de.Key).Equals("FINALFXNSYMBOLNAME", StringComparison.OrdinalIgnoreCase))
          {
            finalFxnName = (String)sec.sectionValues["FINALFXNSYMBOLNAME"];
          }
        }
        if (devAISGen.InsertAISFinalFxnReg(finalFxnName) != retType.SUCCESS)
        {
          Console.WriteLine("WARNING: Final function register AIS command failed.");
        }
      }
      
      else if ( (sec.sectionName.Equals("GENERAL", StringComparison.OrdinalIgnoreCase)) ||
                (sec.sectionName.Equals("SECURITY", StringComparison.OrdinalIgnoreCase)) )
      {
        // Ignore General/Security section here since it should have already been processed
      }
      
      else
      {
        // Any other sections names should be ignored with warning
        Console.WriteLine("WARNING: Unrecognized INI section, {0}. Ignoring...", sec.sectionName );
      }
      
      #endregion  
    
      return retType.SUCCESS;
    }
    
    public static ObjectFile FindFileWithSymbol(AISGen devAISGen, String symbolName)
    {
      foreach (ObjectFile file in devAISGen.objectFiles)
      {
        if (file.symFind(symbolName) != null)
        {
          return file;
        }
      }
      return null;
    }
    
    public static ObjectSymbol FindSymbol(AISGen devAISGen, String symbolName)
    {
      ObjectSymbol sym = null;
      foreach (ObjectFile file in devAISGen.objectFiles)
      {
        sym = file.symFind(symbolName);
        if (sym != null) break;
      }
      return sym;
    }
    
    public static ObjectFile FindObjectFile(AISGen devAISGen, String fileName)
    {
      foreach (ObjectFile file in devAISGen.objectFiles)
      {
        if (file.FileName.Equals(fileName))
        {
          return file;
        }
      }
      return null;
    }
  #endregion
    
  #region Private Static Methods
    private static void GeneralIniSectionParse(AISGen devAISGen, IniFile iniFile)
    {
      // Get data from the GENERAL INI Section
      IniSection genSec = iniFile.GetSectionByName("General");
      
      foreach (DictionaryEntry de in genSec.sectionValues)
      {
        // Read buswidth
        if (((String)de.Key).Equals("BUSWIDTH", StringComparison.OrdinalIgnoreCase))
          devAISGen.busWidth = (UInt32)genSec.sectionValues["BUSWIDTH"];
        
        // Read BootMode (unless already set)
        if ((((String)de.Key).Equals("BOOTMODE", StringComparison.OrdinalIgnoreCase)) && (devAISGen.bootMode == AisBootModes.NONE))
          devAISGen.bootMode = (AisBootModes) Enum.Parse(typeof(AisBootModes), (String)genSec.sectionValues["BOOTMODE"], true);
        
        // Read Addr width (for I2C/SPI)
        if (((String)de.Key).Equals("ADDRWIDTH", StringComparison.OrdinalIgnoreCase))
        {
          devAISGen.addrWidth = (UInt32)genSec.sectionValues["ADDRWIDTH"];
        }
        
        // CRC Type override
        if (((String)de.Key).Equals("CRCCheckType", StringComparison.OrdinalIgnoreCase))
        {
          devAISGen.AISCRCType = (AisCRCCheckType) Enum.Parse(typeof(AisCRCCheckType), (String)genSec.sectionValues["CRCCHECKTYPE"], true);
        }
        
        // Read global entry point
        if (((String)de.Key).Equals("ENTRYPOINT", StringComparison.OrdinalIgnoreCase))
        {
          devAISGen.entryPoint = (UInt32)genSec.sectionValues["ENTRYPOINT"];
        }
      }
    }
  
    private static retType AISObjectFileLoad( AISGen devAISGen, ObjectFile file )
    {
      UInt32 loadedSectionCount = 0;
    
      // Check if object file already loaded
      if (FindObjectFile(devAISGen,file.FileName) != null)
      {
        return retType.FAIL;
      }
      
      // If this is a new file, let's add it to our list
      devAISGen.objectFiles.Add(file);
    
      if (!devAISGen.devEndian.ToString().Equals(file.Endianness))
      {
        Console.WriteLine("Endianness mismatch. Device is {0} endian, Object file is {1} endian",
            devAISGen.devEndian.ToString(),
            file.Endianness);
        return retType.FAIL;
      }

      // Make sure the .TIBoot section is first (if it exists)
      ObjectSection firstSection = file.LoadableSections[0];
      for (Int32 i = 1; i < file.LoadableSectionCount; i++)
      {
        if ((file.LoadableSections[i].name).Equals(".TIBoot"))
        {
          file.LoadableSections[0] = file.LoadableSections[i];
          file.LoadableSections[i] = firstSection;
          break;
        }
      }

      // Enable CRC if needed
      devAISGen.InsertAISEnableCRC();
      
      // Do all SECTION_LOAD commands
      for (Int32 i = 0; i < file.LoadableSectionCount; i++)
      {
        if (AISSectionLoad(devAISGen, file, file.LoadableSections[i]) != retType.SUCCESS)
        {
          return retType.FAIL;
        }
        
        // Check for need to do TIBoot initialization
        if (loadedSectionCount == 0)
        {
          devAISGen.InsertAISJump("_TIBootSetup");
        }
        
        loadedSectionCount++;
      } 
      // End of SECTION_LOAD commands
      
      // Now that we are done with file contents, we can close it
      file.Close();
      
      return retType.SUCCESS;
    }
    
    private static retType AISSectionLoad( AISGen devAISGen, ObjectFile file, ObjectSection section)
    {
      Byte[] secData = file.secRead(section);
      Byte[] srcCRCData = new Byte[section.size + 8];
      
      Debug.DebugMSG("AISSectionLoad for section " + section.name + " from file " + file.FileName + ".");
      
      // If we are doing section-by-section CRC, then zero out the CRC value
      if (devAISGen.aisCRCType == AisCRCCheckType.SECTION_CRC)
      {
        devAISGen.devCRC.ResetCRC();
      }
      
      // Add section load to the output
      devAISGen.InsertAISSectionLoad((UInt32) section.loadAddr, (UInt32) section.size, secData);

      // Copy bytes to CRC byte array for future CRC calculation
      if (devAISGen.aisCRCType != AisCRCCheckType.NO_CRC)
      {
        if (devAISGen.devEndian != devAISGen.devAISEndian)
        {
          Endian.swapEndian(BitConverter.GetBytes(section.loadAddr)).CopyTo(srcCRCData, 0);
          Endian.swapEndian(BitConverter.GetBytes(section.size)).CopyTo(srcCRCData, 4);
        }
        else
        {
          BitConverter.GetBytes(section.loadAddr).CopyTo(srcCRCData, 0);
          BitConverter.GetBytes(section.size).CopyTo(srcCRCData, 4);
        }
        
      }

      // Now write contents to CRC array
      for (UInt32 k = 0; k < section.size; k+=4)
      {
        // Copy bytes to array for future CRC calculation
        if (devAISGen.aisCRCType != AisCRCCheckType.NO_CRC)
        {
          Byte[] temp = new Byte[4];
          Array.Copy(secData,k,temp,0,4);
          if (devAISGen.devEndian != devAISGen.devAISEndian)
          {
            Endian.swapEndian(temp).CopyTo(srcCRCData, (8 + k));
          }
          else
          {
            temp.CopyTo(srcCRCData, (8 + k));
          }
        }
      }
        
      // Add this section's memory range, checking for overlap
      AddMemoryRange(devAISGen, (UInt32) section.loadAddr, (UInt32) (section.loadAddr+section.size-1));

      // Perform CRC calculation of the section's contents
      if (devAISGen.aisCRCType != AisCRCCheckType.NO_CRC)
      {
        devAISGen.devCRC.CalculateCRC(srcCRCData);
        if (devAISGen.aisCRCType == AisCRCCheckType.SECTION_CRC)
        {
          // Write CRC request command, value, and jump value to temp AIS file
          devAISGen.InsertAISRequestCRC(((Int32)(-1) * (Int32)(section.size + 12 + 12)));
        }
      }

      return retType.SUCCESS;
    }
    
    private static void AddMemoryRange(AISGen devAISGen, UInt32 startAddr, UInt32 endAddr)
    {
      // Cycle through entire list looking for overlaps
      foreach (MemoryRange m in devAISGen.sectionMemory)
      {
        // Three issues:
        //   1) Both fall in occupied memory        
        //   2) Input startAddr falls in occuppied memory, input endAddr does not
        //   3) Input endAddr falls in occuppied memory
        if ( ( (startAddr >= m.startAddr) && (startAddr <= m.endAddr) ) && ( (endAddr >= m.startAddr) && (endAddr <= m.endAddr) ) )
        {
          Console.WriteLine("WARNING: Memory overlap from 0x{0:X8} to 0x{1:X8}.",startAddr, endAddr);
          continue;
        }
        
        if ( (startAddr >= m.startAddr) && (startAddr <= m.endAddr) && (endAddr > m.endAddr) )
        {
          Console.WriteLine("WARNING: Memory overlap from 0x{0:X8} to 0x{1:X8}.",startAddr, m.endAddr);
          continue;
        }
        
        if ( (startAddr < m.startAddr) && (endAddr >= m.startAddr) && (endAddr <= m.endAddr) )
        {
          Console.WriteLine("WARNING: Memory overlap from 0x{0:X8} to 0x{1:X8}.",m.startAddr, endAddr);
          continue;
        }
      }
      
      // Add the MemoryRange for this section to the list
      devAISGen.sectionMemory.Add(new MemoryRange(startAddr,endAddr));
    }
  #endregion
  
  }                    
} //end of AISGenLib namespace

