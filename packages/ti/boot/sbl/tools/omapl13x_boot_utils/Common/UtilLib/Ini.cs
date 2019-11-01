/*
 * Ini.cs
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
using System;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;

namespace TI.UtilLib.Ini
{

  /**
   * This struct defines a section of an INI file/stream
   */
  public class IniSection
  {
    /**
     * INI Section Name
     */
    public String sectionName;
    
    /**
     * HashTable of the section values
     */
    public Hashtable sectionValues;
    
    /**
     * Default Constructor
     */
    public IniSection()
    {
      sectionName = null;
      sectionValues = new Hashtable();
    }
    
    public IniSection(String name)
    {
      sectionName = name;
      sectionValues = new Hashtable();
    }
    
    public IniSection(String name, Hashtable values)
    {
      sectionName = name;
      sectionValues = values;
    }
    
    public Object GetValue(String valueName)
    {
      return sectionValues[valueName.ToUpper()];
    }
    
    public void InsertValue(String valueName, String value)
    {
      if (sectionValues == null)
      {
        sectionValues = new Hashtable();
      }
      TryInsertValue(this, valueName, value);
    }
    
    public override String ToString()
    {
      return IniSection.ToString(this);
    }
    
    /**
     * Static function to insert key/value pair into specified IniSection
     */
    public static Boolean TryInsertValue(IniSection section, String valueName, String value)
    {
      if ((section == null) || (section.sectionValues == null))
      {
        return false;
      }
      else
      {
        UInt32 valueNum;
        valueName = valueName.ToUpper();
        
        // Hex values must be prefixed by "0x"
        value = value.ToLower();
        if (value.StartsWith("0x"))
        {
          if (!UInt32.TryParse(value.Replace("0x", ""), NumberStyles.HexNumber, null, out valueNum) )
          {
            return false;
          }
          else
          {
            section.sectionValues[valueName] = valueNum;
          }
        }
        else if (UInt32.TryParse(value, out valueNum))
        {
          section.sectionValues[valueName] = valueNum;
        }
        else
        {
          section.sectionValues[valueName] = value;
        }
      }
      return true;
    }
    
    public static String ToString(IniSection section)
    {
      StringWriter sw = new StringWriter(new StringBuilder());
      sw.WriteLine("[{0}]",section.sectionName);
        
      foreach (DictionaryEntry de in section.sectionValues)
      {
        sw.WriteLine("{0} = {1}",de.Key,de.Value);
      }
      return sw.GetStringBuilder().ToString();
    }
  }
  
  public class IniFile
  {
    private List<IniSection> sections;
    private String fileName;
    
    public IniFile()
    {
      fileName = null;
      sections = null;
    }
    
    public List<IniSection> Sections
    {
      get { return sections; }
    }
  
    public IniFile(String fileName)
    {
      try
      {
        this.sections = (IniFile.Parse(fileName)).sections;
      }
      catch (Exception e)
      {
        //Fixme
        Console.WriteLine(e.Message);
        throw new ArgumentException();
      }
        
      this.fileName = fileName;
    }
    
    public IniFile(Stream fileStream, String fileName)
    {
      try
      {
        this.sections = (IniFile.Parse(fileStream)).sections;
      }
      catch (Exception e)
      {
        //Fixme
        Console.WriteLine(e.Message);
        throw new ArgumentException();
      }
      this.fileName = fileName;
    }
    
    public IniFile(String iniString, String fileName)
    {
      try
      {
        this.sections = (IniFile.Parse(iniString)).sections;
      }
      catch (Exception e)
      {
        //Fixme
        Console.WriteLine(e.Message);
        throw new ArgumentException();
      }
      this.fileName = fileName;
    }
    
    public IniFile(Byte[] iniFileData, String fileName)
    {
      try
      {
        this.sections = (IniFile.Parse(iniFileData)).sections;
      }
      catch (Exception e)
      {
        //Fixme
        Console.WriteLine(e.Message);
        throw new ArgumentException();
      }
      this.fileName = fileName;
    }
    
    public IniFile(List<IniSection> sections, String fileName)
    {
      this.sections = sections;
      this.fileName = fileName;
    }
    
    public IniSection GetSectionByName(String name)
    {
      foreach (IniSection sec in sections)
      {
        if (sec.sectionName.Equals(name, StringComparison.OrdinalIgnoreCase))
        {
          return sec;
        }
      }
      return null;
    }
    
    public Object GetValueByName(String sectionName, String valueName)
    {
      IniSection section = GetSectionByName(sectionName);
      if (section != null)
      {
        return section.GetValue(valueName);
      }
      else
      {
        return null;
      }
    }
    
    public void InsertSection(String sectionName)
    {
      InsertSection(new IniSection(sectionName));
    }
    
    public void InsertSection(IniSection section)
    {
      IniSection sec = GetSectionByName(section.sectionName);
      if (sec == null)
      {
        // Section by this name does not exist
        sections.Add(sec);
      }
      else 
      {
        // Section by this name exists, simply insert values
        foreach (DictionaryEntry de in section.sectionValues)
        {
          sec.InsertValue((String)de.Key, (String)de.Value);
        }
      }
    }
    
    public void InsertValue(String sectionName, String valueName, String value)
    {
      IniSection sec = GetSectionByName(sectionName);
      if (sec == null)
      {
        // Section doesn't yet exist, so insert it
        InsertSection(sectionName);
      }
      sec = GetSectionByName(sectionName);
      sec.InsertValue(valueName, value);
    }
    
    /**
     * Function to Parse an input data stream containing INI data
     *
     * @param iniStream is an input Stream object
     */
    public static IniFile Parse(Stream iniStream)
    {
      IniFile myFile = new IniFile();
      List<String> streamLines = new List<String>();
      List<IniSection> streamSections = new List<IniSection>();
      StreamReader iniSR;
      
      IniSection currSec = new IniSection();
      Boolean inASection = false;
      Regex iniSecHdr = new Regex("\\[[A-Za-z0-9_]*\\]");

      try
      {
        iniSR = new StreamReader(iniStream);
      }
      catch (Exception e)
      {
        Console.WriteLine(e.Message);
        throw e;
      }
     
      // Get lines of data from the stream
      while (!iniSR.EndOfStream)
      {
        streamLines.Add(iniSR.ReadLine());
      }

      // Parse actually line contents
      for (int i=0; i<streamLines.Count; i++)
      {
        // Get current line from the streamLines List
        String currLine = (streamLines[i]).Trim();
        
        // Ignore comment and empty lines
        if ( (currLine.StartsWith(";")) || (currLine.Equals("")) )
        {
          continue;
        }                                  
          
        // If we find a section header, begin a new section
        Match m = iniSecHdr.Match(currLine);
        if (m.Success)
        {
          if (inASection)
          {
            streamSections.Add(currSec);
          }
          inASection = true;
          currSec = new IniSection();
          currSec.sectionName = m.Value.ToUpper().Trim('[', ']');
          currSec.sectionValues = new Hashtable();
          Debug.DebugMSG("INI Section: {0}", currSec.sectionName);
          continue;
        }

        // If we find key/value paramter pairs, parse the value and key
        if (currLine.Contains("="))
        {
          // Split the name at the '=' sign
          String[] paramAndValue = currLine.Split(new char[1] { '=' }, StringSplitOptions.RemoveEmptyEntries);

          // Trim the param name and value
          paramAndValue[0] = paramAndValue[0].Trim().ToUpper();
          paramAndValue[1] = paramAndValue[1].Trim();

          // Hex values must be prefixed by "0x"
          if (paramAndValue[1].StartsWith("0x") || paramAndValue[1].StartsWith("0X"))
          {
            currSec.sectionValues[paramAndValue[0]]
              = UInt32.Parse(paramAndValue[1].Replace("0x", ""), NumberStyles.AllowHexSpecifier);
          }
          else
          {
            UInt32 value;
            if (UInt32.TryParse(paramAndValue[1], out value))
            {
              currSec.sectionValues[paramAndValue[0]] = value;
            }
            else
            {
              currSec.sectionValues[paramAndValue[0]] = paramAndValue[1];
            }
          }
          Debug.DebugMSG("\t{0} = {1}", paramAndValue[0], currSec.sectionValues[paramAndValue[0]]);
          continue;
        }
        
        // Any other lines throw an error
        throw new Exception(String.Format("Bad INI data at line {0}: {1}.", i, currLine));
      } // End of parsing INI
      
      // Add last section to return value
      if (inASection)
      {
        streamSections.Add(currSec);
      }

      // Return parsed ini sections    
      myFile.sections = streamSections;
      return myFile;
    }
    
    /**
     * Function to parse an input data stream containing INI data
     *
     * @param iniData is a String containing the INI data
     */    
    public static IniFile Parse(String iniData)
    {
      ASCIIEncoding ae = new ASCIIEncoding();
      return IniFile.Parse(ae.GetBytes(iniData));
    }
    
    /**
     * Function to parse an input data stream containing INI data
     *
     * @param iniData is an array of bytes containing the INI data
     */
    public static IniFile Parse(Byte[] iniData)
    {
      return IniFile.Parse(new MemoryStream(iniData, false));
    }
    
    /**
     * Static function to build an INI data string
     *
     * @param sections is an array of IniSection objects
     */
    public static String ToString(IniSection[] sections)
    {
      StringWriter sw = new StringWriter(new StringBuilder());
      
      foreach(IniSection currSection in sections)
      {
        sw.WriteLine(currSection.ToString());
        sw.WriteLine();
      }
      
      return sw.GetStringBuilder().ToString();
    }
    
    /**
     * Static function to build an INI data string
     *
     * @param sections is a List of IniSection objects
     */
    public static String ToString(List<IniSection> sections)
    {
      return IniFile.ToString(sections.ToArray());
    }
    
    /**
     * Static function to build an INI data string
     *
     * @param sections is a List of IniSection objects
     */
    public static String ToString(IniFile file)
    {
      return IniFile.ToString(file.sections);
    }
  }
  
}