/*
 * ObjectFile.cs
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
// Abstract ObjectFile class

using System;
using System.Text;
using System.IO;
using System.IO.Ports;
using System.Reflection;
using System.Threading;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using TI.UtilLib;
using TI.UtilLib.IO;

namespace TI.UtilLib.ObjectFile
{ 

  public enum ObjectMachineType
  {
    ARM,
    C6000,
    C2400,
    C5400,
    C5500,
    C2000,
    MSP430,
    C5500p    
  }

  public enum ObjectFileType
  {
    COFF,
    ELF,
    PE, 
    BINARY
  }
  
  public class ObjectSection : IComparable<ObjectSection>
  {
    public String name;
    public UInt64 size;
    public UInt64 runAddr;
    public UInt64 loadAddr;
    public Boolean isLoadable;
    public UInt64 binFileAddr;
    
    public int CompareTo(ObjectSection otherSection)
    {
      // The ObjectSection comparison depends on the comparison of the
      // the underlying loadAddr values. Because the CompareTo method is
      // strongly typed, it is not necessary to test for the correct
      // object type.
      return this.loadAddr.CompareTo(otherSection.loadAddr);
    }
  }
  
  public class ObjectSymbol : IComparable<ObjectSymbol>, IEquatable<ObjectSymbol>
  {
    public String name;
    public UInt64 value;
    
    public int CompareTo(ObjectSymbol otherSymbol)
    {
      // The ObjectSymbol comparison depends on the comparison of the
      // the underlying value values. Because the CompareTo method is
      // strongly typed, it is not necessary to test for the correct
      // object type.
      return this.value.CompareTo(otherSymbol.value);
    }
    
    public bool Equals(ObjectSymbol otherSymbol)
    {
      return (this.value.Equals(otherSymbol.value) && 
              this.name.Equals(otherSymbol.name)); 
    }
    
    
  }

  /// <summary>
  /// Generic Public class for Object Files
  /// </summary>
  public class ObjectFile : IDisposable
  {
    #region Private internal members
    protected String            fileName;
    protected Stream            binFile;
    protected Endian            endian;
    protected UInt32            sectionCount;
    protected UInt32            loadableSectionCount;
    protected UInt32            symbolCount;
    protected UInt32            currSectionIndex;
    protected UInt32            currSymbolIndex;
    protected UInt64            entryPoint;
    protected ObjectMachineType machineType;
    protected ObjectFileType    fileType;
        
    protected ObjectSection[]   sections;
    protected ObjectSection[]   loadableSections;
    protected ObjectSymbol[]    symbols;
    
    private Boolean disposed = false;
    #endregion
    
    #region Class constructors
    public ObjectFile()
    {
      fileName    = "";
      binFile     = null;
      
      sectionCount = 0;
      symbolCount = 0;
      loadableSectionCount = 0;
      
      currSectionIndex = 0;
      currSymbolIndex = 0;
      entryPoint = 0;
      
      sections    = null;
      symbols     = null;
    }
    
    public ObjectFile(String filename)
    {
      
      // Get COFF file details
      FileInfo fi = new FileInfo(filename);
      try
      {
        if (fi.Exists)
          binFile = new FileStream(filename, FileMode.Open, FileAccess.Read);
      }
      catch (Exception e)
      {
          Console.Write(e.Message);
      }
                  
      // Init parsing info
      currSectionIndex = 0;
      currSymbolIndex = 0;

      this.fileName = filename;

    }
    #endregion
    
    #region Public properties and indexers
    public String FileName
    {
      get {return fileName; }
    }
    
    public UInt64 EntryPoint
    {
      get { return entryPoint; }
    }
    
    public ObjectMachineType MachineType
    {
      get { return machineType; }
    }
    
    public ObjectFileType FileType
    {
      get { return fileType; }
    }
    
    public String Endianness
    {
      get { return endian.ToString(); }
    }
    
    public UInt32 SectionCount
    {
      get { return sectionCount; }
    }
    
    public UInt32 LoadableSectionCount
    {
      get { return loadableSectionCount; }
    }
    
    public UInt32 SymbolCount
    {
      get { return symbolCount; }
    }
    
    /// <summary>
    /// Indexer to return section with specified index
    /// </summary>
    /// <param name="index">Index of section in the COFF file</param>
    /// <returns>Array of section's data</returns>
    public Byte[] this[UInt32 index]
    {
      get
      {
        return secRead(index);
      }
    }

    /// <summary>
    /// Indexer to return section with specified name
    /// </summary>
    /// <param name="name">Section name we wish to get</param>
    /// <returns>Array of section's data </returns>
    public Byte[] this[String name]
    {
      get
      {
        return secRead(name);
      }
    }

    public ObjectSection[] Sections
    {
      get { return sections; }
    }
    
    public ObjectSection[] LoadableSections
    {
      get { return loadableSections; }
    }

    public ObjectSymbol[] Symbols
    {
      get { return symbols; }
    }

    #endregion
    
    #region Public Class Methods
    public void Dispose()
    {
      Dispose(true);

      // This object will be cleaned up by the Dispose method.
      // Therefore, you should call GC.SupressFinalize to
      // take this object off the finalization queue
      // and prevent finalization code for this object
      // from executing a second time.
      GC.SuppressFinalize(this);    
    }
    
    public void dumpSymbolTable()
    {
      symRewind();
      Console.WriteLine("\tValue     \tName");
      Console.WriteLine("\t==========\t========");
      foreach (ObjectSymbol sym in Symbols)
      {
        if (!( (sym.name).Equals("") || (sym.name == null) ))
        {
          Console.WriteLine("\t0x{0:X8}\t {1}", ((UInt32)sym.value), sym.name);
        }
      }
    }
    
    public new String ToString()
    {
      StringBuilder strBuilder = new StringBuilder(512);
      FieldInfo[] myFieldInfo;
      Type myType;
      
      // Get the type and fields of ObjectSection.
      myType = typeof(ObjectSection);
      myFieldInfo = myType.GetFields();

      for (int i = 0; i < sectionCount; i++)
      {
        for (int j = 0; j<myFieldInfo.Length; j++)
        {
          strBuilder.Append("Sections["+i+"]." + myFieldInfo[j].Name + " = " + myFieldInfo[j].GetValue(sections[i]) + "\n");  
        }
      }
      
      myType = typeof(ObjectSymbol);
      myFieldInfo = myType.GetFields();

      for (int i = 0; i < symbolCount; i++)
      {
        for (int j = 0; j<myFieldInfo.Length; j++)
        {
          strBuilder.Append("Symbols["+i+"]." + myFieldInfo[j].Name + " = " + myFieldInfo[j].GetValue(symbols[i]) + "\n");
        }
      }

      return strBuilder.ToString();
    }

    #region Section seek and access commands
    
    public ObjectSection secRewind()
    {
      currSectionIndex = 0;
      return Sections[currSectionIndex];
    }

    public ObjectSection secSeek(UInt32 index)
    {
      if ( (index >= 0) && (index < sectionCount) )
      {
        currSectionIndex = index;
        return Sections[currSectionIndex];
      }
      return null;
    }

    public ObjectSection secFind(String secName)
    {
      for (UInt32 i = 0; i < sectionCount; i++)
      {
        if ( (Sections[i].name).Equals(secName))
        {
          currSectionIndex = i;
          return Sections[currSectionIndex];
        }
      }
      return null;
    }

    public ObjectSection secEnum()
    {
      if ( (currSectionIndex >= 0) && ( currSectionIndex < sectionCount ) )
      {
        currSectionIndex++;
        return Sections[currSectionIndex-1];
      }
      
      return null;
    }
    
    public Byte[] secRead(ObjectSection section)
    {
      Byte[] dataArr          = new Byte[section.size];
      EndianBinaryReader ebr  = new EndianBinaryReader(this.binFile,this.endian);
      
      // Go to start of section in the binary file
      binFile.Seek((Int64) section.binFileAddr, SeekOrigin.Begin);
      
      // Read section bytes
      dataArr = ebr.ReadBytes((Int32)section.size);
      
      // Check to see if we need to pad the output array
      // Should only be needed for binary files
      if (dataArr.Length < (Int32) section.size)
      {
        Array.Resize(ref dataArr, (Int32) section.size);
      }

      return dataArr;
    }
    
    public Byte[] secRead()
    {
      ObjectSection section   = Sections[currSectionIndex];
      return secRead(section);
    }    

    public Byte[] secRead(UInt32 secNum)
    {
        secSeek(secNum);
        return secRead();
    }

    public Byte[] secRead(String secName)
    {
        secFind(secName);
        return secRead();
    }
    
    #endregion

    #region Symbol seek and access commands
    public ObjectSymbol symRewind()
    {
      currSymbolIndex = 0;
      return Symbols[currSymbolIndex];
    }

    public ObjectSymbol symSeek( UInt32 index)
    {
      if ((index>=0) && (index < symbolCount))
      {
        currSymbolIndex = index;
        return Symbols[currSymbolIndex];
      }
      return null;
    }

    public ObjectSymbol symFind( String symName)
    {
      for(UInt32 i = 0; i < symbolCount; i++)
      {
        if ( (Symbols[i].name).Equals(symName))
        {
          currSymbolIndex = i;
          return Symbols[currSymbolIndex];
        }
      }
      return null;
    }

    public ObjectSymbol symEnum()
    {
      if ( (currSymbolIndex >=0) && (currSymbolIndex <= symbolCount) )
      {
        currSymbolIndex++;
        return Symbols[currSymbolIndex-1];
      }
     
      return null;
    }
    #endregion

    /// <summary>
    /// Function to close the Object file associated with this object
    /// </summary>
    public void Close()
    {
      binFile.Close();
    }

    #endregion

    #region Public Static Class Methods
    public static Boolean IsElfFile(ObjectFile file)
    {
      return (file.FileType == ObjectFileType.ELF);
    }
    
    public static Boolean IsCoffFile(ObjectFile file)
    {
      return (file.FileType == ObjectFileType.COFF);
    }
    #endregion
    
    #region Private class Methods
    private void Dispose(Boolean disposing)
    {
      // Check to see if Dispose has already been called.
      if(!this.disposed)
      {
        // If disposing equals true, dispose all managed
        // and unmanaged resources.
        if(disposing)
        {
          // Dispose managed resources.
          binFile.Dispose();
        }

        // Dispose any unmanaged resource here
        
        // Note disposing has been done.
        disposed = true;
      }
    }
    #endregion
  }

} //end of namespace
