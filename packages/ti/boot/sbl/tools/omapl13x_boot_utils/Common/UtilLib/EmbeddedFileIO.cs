/*
 * EmbeddedFileIO.cs
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
  public class EmbeddedFileIO
  {
    private static List<String> resources = new List<String>(10);
    private static List<String> files = new List<String>(10);

    public static void ExtractFile(Assembly thisAssm, String fileName, Boolean overWrite)
    {
      Stream dataStream;
      String myResourceName = "";

      dataStream = GetEmbeddedStream(thisAssm, fileName);

      if ((File.Exists(fileName)) && overWrite)
      {
        // If the file is already there and we are supposed to overwrite,
        // but there is no embedded version, do nothing
        if (dataStream == null)
        {
          return;
        }
        // If the file is already there and we are supposed to overwrite,
        // but we already extracted it from here, do nothing
        else if (resources.Contains(myResourceName))
          return;
      }
      else if (!File.Exists(fileName))
      {
        if (dataStream == null)
          throw new FileNotFoundException("File " + fileName + " was not found in the assembly or in the current directory.");
      }
      else
      {
        return;
      }

      FileStream newFile = File.Open(fileName, FileMode.Create, FileAccess.Write);
      Byte[] buffer = new Byte[dataStream.Length];
      dataStream.Seek(0, SeekOrigin.Begin);
      dataStream.Read(buffer, 0, (int)dataStream.Length);
      newFile.Write(buffer, 0, (int)buffer.Length);
      newFile.Close();
      files.Add(fileName);
      resources.Add(myResourceName);
    }
    
    public static Byte[] ExtractFileBytes(Assembly thisAssm, String fileName)
    {
      Stream dataStream = GetEmbeddedStream(thisAssm, fileName);
      Byte[] buffer;
      
      if (dataStream == null)
      {
        throw new FileNotFoundException("File " + fileName + " was not found in the assembly or in the current directory.");
      }
      
      buffer = new Byte[dataStream.Length];
      
      // Read data to byte array
      dataStream.Seek(0, SeekOrigin.Begin);
      dataStream.Read(buffer, 0, (int)dataStream.Length);
      
      return buffer;
    }

    public static void CleanUpEmbeddedFiles()
    {
      foreach (String s in files)
      {
        if (File.Exists(s))
        {
          try 
          {
            File.Delete(s);
          }
          catch (Exception e)
          {
            Console.WriteLine(e.Message);
          }
        }
      }
      files.Clear();
    }

    private static Stream GetEmbeddedStream(Assembly thisAssm, String fileName)
    {
      Stream myStream = null;
      String myResourceName = "";

      foreach (String s in thisAssm.GetManifestResourceNames())
      {
        if (s.Contains(fileName))
        {
          myResourceName = s;
          break;
        }
      }
      if (myResourceName.Equals(""))
        return null;

      try
      {
        myStream = thisAssm.GetManifestResourceStream(myResourceName);
      }
      catch (Exception e)
      {
        if (e is FileNotFoundException)
          return null;
        else
          throw e;
      }

      return myStream;
    }

    
  }
}
