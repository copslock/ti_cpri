/*
 * FileIO.cs
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
using System.Reflection;
using System.Collections.Generic;
using System.Text;

namespace TI.UtilLib.IO
{
  public class FileIO
  {
    /// <summary>
    /// Function to find and read a file's data
    /// </summary>
    /// <param name="filename">The name of the file to load</param>
    /// <returns></returns>
    public static Byte[] GetFileData(String filename)
    {
      Byte[] data;

      if (!File.Exists(filename))
      {
        throw new FileNotFoundException("File " + filename + " is not present.");
      }

      // Open file and read data
      try
      {
        data = File.ReadAllBytes(filename);
      }
      catch (Exception e)
      {
        Console.WriteLine("Error: "+e.Message);
        throw e;
      }

      return data;
    }
    
    public static String GetFileText(String filename)
    {
      String text;

      if (!File.Exists(filename))
      {
        throw new FileNotFoundException("File " + filename + " is not present.");
      }

      // Open file and read data
      try
      {
        text = File.ReadAllText(filename);
      }
      catch (Exception e)
      {
        Console.WriteLine("Error: "+e.Message);
        throw e;
      }

      return text;
    }

    public static void SetFileData(String filename, Byte[] data, Boolean overwrite)
    {
      // Open file and read data
      try
      {
        if (!File.Exists(filename))
        {
          using (FileStream fs = File.Create(filename)){}
        }
        else if (overwrite)
        {
          using (FileStream fs = File.Create(filename)){}
        }
        
        File.WriteAllBytes(filename,data);
      }
      catch (Exception e)
      {
        Console.WriteLine("Error: "+e.Message);
        throw e;
      }
    }
    
    public static void SetFileText(String filename, String data, Boolean overwrite)
    {
      // Open file and read data
      try
      {
        if (!File.Exists(filename))
        {
          using (FileStream fs = File.Create(filename)){}
        }
        else if (overwrite)
        {
          using (FileStream fs = File.Create(filename)){}
        }

        File.WriteAllText(filename,data);
      }
      catch (Exception e)
      {
        Console.WriteLine("Error: "+e.Message);
        throw e;
      }
    }
    

  }
}
