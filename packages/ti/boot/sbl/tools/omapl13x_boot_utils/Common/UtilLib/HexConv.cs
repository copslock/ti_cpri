/*
 * HexConv.cs
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
using System.Text;
using System.IO;
using System.IO.Ports;
using System.Reflection;
using System.Threading;
using System.Globalization;

namespace TI.UtilLib.HexConv
{
  class SRecord
  {
      /// <summary>
      /// Function to convert the input filestream into an byte array in S-record format.
      /// </summary>
      /// <param name="inputFileStream">The input filestream that encapsulates the
      /// input binary file.</param>
      /// <param name="startAddr">The starting address of the RAM location where the binary data
      /// encapsulated by the S-record will be stored.</param>
      /// <returns>A byte array of the file data.</returns>
      public static Byte[] bin2srec(Stream inputStream, UInt32 startAddr)
      {
          Int64 totalLen;
          BinaryReader fileBR = new BinaryReader(inputStream);
          StringBuilder fileSB;
          String fileName;
          String shortFileName;
          Byte[] currChar = new Byte[1];
          Byte[] currDataRecord;
          Int32 i, checksum8 = 0;
          Int32 recordSize = 16;
          UInt32 memAddr = startAddr;

          // Set the actual length
          totalLen = fileBR.BaseStream.Length;
          fileSB = new StringBuilder(4 * (int)totalLen);

          // Set S-record filename (real name or fake)
          if (inputStream is FileStream)
              fileName = ((FileStream)inputStream).Name;
          else
              fileName = "ublDaVinci.bin";

          // Make sure we are at the right place in the stream
          fileBR.BaseStream.Seek(0x0, SeekOrigin.Begin);

          // Get filename (this is S-record module name)
          if (Path.HasExtension(fileName))
              shortFileName = Path.GetFileNameWithoutExtension(fileName) + ".hex";
          else
              shortFileName = Path.GetFileName(fileName) + ".hex";

          // Make sure S-record module name fits in 20 byte field
          if (shortFileName.Length > 20)
              shortFileName = shortFileName.Substring(0, 20);

          // Create first s-record (S0 record)
          fileSB.Append("S0");
          // Write length field
          fileSB.AppendFormat("{0:X2}", shortFileName.Length + 3);
          checksum8 += (Byte)(shortFileName.Length + 3);
          // Write address field
          fileSB.Append("0000");
          // Write name field
          for (i = 0; i < shortFileName.Length; i++)
          {
              currChar = (new ASCIIEncoding()).GetBytes(shortFileName.Substring(i, 1));
              checksum8 += currChar[0];
              fileSB.AppendFormat("{0:X2}", currChar[0]);
          }
          // Write Checksum field
          fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

          // Write collection of S3 records (actual binary data)
          i = (Int32)totalLen;

          while (i >= recordSize)
          {
              checksum8 = 0;
              // Write S3 record label
              fileSB.Append("S3");
              // Write length field (4 address bytes + 16 data bytes + 1 checksum byte)
              fileSB.AppendFormat("{0:X2}", recordSize + 5);
              checksum8 += (recordSize + 5);

              // Write address field and update it
              fileSB.AppendFormat("{0:X8}", memAddr);
              currDataRecord = System.BitConverter.GetBytes(memAddr);
              for (int j = 0; j < 4; j++)
              {
                  checksum8 += currDataRecord[j];
              }

              // Write out the bytes of data
              currDataRecord = fileBR.ReadBytes(recordSize);
              for (int j = 0; j < recordSize; j++)
              {
                  fileSB.AppendFormat("{0:X2}", currDataRecord[j]);
                  checksum8 += currDataRecord[j];
              }
              //Write out checksum and linefeed character
              fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

              memAddr += (UInt32)recordSize; i -= recordSize;
          }

          // Finish out the record if anything is left over
          if (i > 0)
          {
              checksum8 = 0;
              // Write S3 record label
              fileSB.Append("S3");
              // Write length field (4 address bytes + 16 data bytes + 1 checksum byte)
              fileSB.AppendFormat("{0:X2}", i + 5);
              checksum8 += (i + 5);

              // Write address field and update it
              fileSB.AppendFormat("{0:X8}", memAddr);
              currDataRecord = System.BitConverter.GetBytes(memAddr);
              for (int j = 0; j < 4; j++)
              {
                  checksum8 += currDataRecord[j];
              }

              // Write out the bytes of data
              currDataRecord = fileBR.ReadBytes(i);
              for (int j = 0; j < i; j++)
              {
                  fileSB.AppendFormat("{0:X2}", currDataRecord[j]);
                  checksum8 += currDataRecord[j];
              }
              //Write out checksum and linefeed character
              fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

              memAddr += (UInt32)i; i = 0;

          }

          // Write out the final record (S7 record)
          checksum8 = 0;
          // Write S7 record label
          fileSB.Append("S7");
          // Write length field (4 address bytes + 1 checksum byte)
          fileSB.AppendFormat("{0:X2}", 5);
          checksum8 += 5;

          // Write execution start address field and update it
          fileSB.AppendFormat("{0:X8}", startAddr);
          currDataRecord = System.BitConverter.GetBytes(startAddr);
          for (int j = 0; j < 4; j++)
          {
              checksum8 += currDataRecord[j];
          }
          //Write out checksum and linefeed character
          fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

          return (new ASCIIEncoding()).GetBytes(fileSB.ToString());

      }
      
      /// <summary>
      /// Function to convert the input binary byte array into an byte array in S-record format.
      /// </summary>
      /// <param name="inputFileStream">The input filestream that encapsulates the
      /// input binary file.</param>
      /// <param name="startAddr">The starting address of the RAM location where the binary data
      /// encapsulated by the S-record will be stored.</param>
      /// <param name="recordSize">The size in bytes of each S-record (line) of the output.</param>
      /// <returns>A byte array of the file data.</returns>
      public static Byte[] bin2srec(Byte[] inputData, UInt32 startAddr, Int32 recordSize)
      {
          Int64 totalLen;
          StringBuilder fileSB;
          String fileName;
          String shortFileName;
          Byte[] currChar = new Byte[1];
          Byte[] currDataRecord;
          Int32 i, checksum8 = 0;
          UInt32 memAddr = startAddr;

          // Set the actual length
          totalLen = inputData.Length;
          fileSB = new StringBuilder(4 * (int)totalLen);

          // Set S-record filename (real name or fake)
          //if (inputStream is FileStream)
          //    fileName = ((FileStream)inputStream).Name;
          //else
              fileName = "ublDaVinci.bin";

          // Make sure we are at the right place in the stream
          //fileBR.BaseStream.Seek(0x0, SeekOrigin.Begin);

          // Get filename (this is S-record module name)
          if (Path.HasExtension(fileName))
              shortFileName = Path.GetFileNameWithoutExtension(fileName) + ".hex";
          else
              shortFileName = Path.GetFileName(fileName) + ".hex";

          // Make sure S-record module name fits in 20 byte field
          if (shortFileName.Length > 20)
              shortFileName = shortFileName.Substring(0, 20);

          // Create first s-record (S0 record)
          fileSB.Append("S0");
          // Write length field
          fileSB.AppendFormat("{0:X2}", shortFileName.Length + 3);
          checksum8 += (Byte)(shortFileName.Length + 3);
          // Write address field
          fileSB.Append("0000");
          // Write name field
          for (i = 0; i < shortFileName.Length; i++)
          {
              currChar = (new ASCIIEncoding()).GetBytes(shortFileName.Substring(i, 1));
              checksum8 += currChar[0];
              fileSB.AppendFormat("{0:X2}", currChar[0]);
          }
          // Write Checksum field
          fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

          // Write collection of S3 records (actual binary data)
          i = (Int32)totalLen;

          while (i >= recordSize)
          {
              checksum8 = 0;
              // Write S3 record label
              fileSB.Append("S3");
              // Write length field (4 address bytes + 16 data bytes + 1 checksum byte)
              fileSB.AppendFormat("{0:X2}", recordSize + 5);
              checksum8 += (recordSize + 5);

              // Write address field and update it
              fileSB.AppendFormat("{0:X8}", memAddr);
              currDataRecord = System.BitConverter.GetBytes(memAddr);
              for (int j = 0; j < 4; j++)
              {
                  checksum8 += currDataRecord[j];
              }

              // Write out the bytes of data
              //currDataRecord = fileBR.ReadBytes(recordSize);
              
              for (int j = 0; j < recordSize; j++)
              {
                  //fileSB.AppendFormat("{0:X2}", currDataRecord[j]);
                  fileSB.AppendFormat("{0:X2}", inputData[j + (memAddr - startAddr)]);
                  //checksum8 += currDataRecord[j];
                  checksum8 += inputData[j + (memAddr - startAddr)];
              }
              //Write out checksum and linefeed character
              fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

              memAddr += (UInt32)recordSize; i -= recordSize;
          }

          // Finish out the record if anything is left over
          if (i > 0)
          {
              checksum8 = 0;
              // Write S3 record label
              fileSB.Append("S3");
              // Write length field (4 address bytes + 16 data bytes + 1 checksum byte)
              fileSB.AppendFormat("{0:X2}", i + 5);
              checksum8 += (i + 5);

              // Write address field and update it
              fileSB.AppendFormat("{0:X8}", memAddr);
              currDataRecord = System.BitConverter.GetBytes(memAddr);
              for (int j = 0; j < 4; j++)
              {
                  checksum8 += currDataRecord[j];
              }

              // Write out the bytes of data
              //currDataRecord = fileBR.ReadBytes(i);
              for (int j = 0; j < i; j++)
              {
                  //fileSB.AppendFormat("{0:X2}", currDataRecord[j]);
                  fileSB.AppendFormat("{0:X2}", inputData[j + (memAddr - startAddr)]);
                  //checksum8 += currDataRecord[j];
                  checksum8 += inputData[j + (memAddr - startAddr)];
              }
              //Write out checksum and linefeed character
              fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

              memAddr += (UInt32)i; i = 0;

          }

          // Write out the final record (S7 record)
          checksum8 = 0;
          // Write S7 record label
          fileSB.Append("S7");
          // Write length field (4 address bytes + 1 checksum byte)
          fileSB.AppendFormat("{0:X2}", 5);
          checksum8 += 5;

          // Write execution start address field and update it
          fileSB.AppendFormat("{0:X8}", startAddr);
          currDataRecord = System.BitConverter.GetBytes(startAddr);
          for (int j = 0; j < 4; j++)
          {
              checksum8 += currDataRecord[j];
          }
          //Write out checksum and linefeed character
          fileSB.AppendFormat("{0:X2}\x0A", ((checksum8 & 0xFF) ^ 0xFF));

          return (new ASCIIEncoding()).GetBytes(fileSB.ToString());
      }

  }

  class CArray
  {
    public static Byte[] bin2CArray(Byte[] inputData, UInt32 memberSizeInBytes)
    {
      return CArray.bin2CArray("data_array", inputData, memberSizeInBytes);
    }
    
    public static Byte[] bin2CArray(String arrayName, Byte[] inputData, UInt32 memberSizeInBytes)
    {
      StringBuilder cArraySB;
      UInt32 sizeSB = (((UInt32)inputData.Length)/memberSizeInBytes +1) * (memberSizeInBytes*2 + 4) + 30;
    
      cArraySB = new StringBuilder((Int32)sizeSB);
      
      switch(memberSizeInBytes)
      {
        case 1:
          bin2charArray(arrayName,inputData,cArraySB);
          break;
        case 2:
          bin2ushortArray(arrayName,inputData,cArraySB);
          break;
        case 4:
          bin2uintArray(arrayName,inputData,cArraySB);
          break;
        case 8:
          break;
        default:
          throw new ArgumentException("Must be 1, 2, 4, or 8","memberSizeInBytes");
      } 
    
      return (new ASCIIEncoding()).GetBytes(cArraySB.ToString());    
    }
    
    private static void bin2charArray(String arrayName, Byte[] inputData, StringBuilder sb)
    {
      sb.Append("const unsigned char "+ arrayName + "[] = {");
      
      // Output data in hex format
      for (int i = 0; i<inputData.Length; i++)
      {
        sb.AppendFormat("\n0x{0:X2},",inputData[i]);
      }
  
      // Delete comma from last one 
      sb.Remove(sb.Length - 1, 1);
      sb.Append("\n};\n");
    }
    
    private static void bin2ushortArray(String arrayName, Byte[] inputData, StringBuilder sb)
    {
      sb.Append("const unsigned short "+ arrayName + "[] = {");
      
      // Output data in hex format
      for (int i = 0; i<inputData.Length/2; i++)
      {
        UInt32 data = System.BitConverter.ToUInt32(inputData, i*2);
        sb.AppendFormat("\n0x{0:X4},",data);
      }
      
      if ((inputData.Length % 2) != 0)
      {
        Byte[] temp = new Byte[] {0x00,0x00};
        System.Array.Copy(inputData,inputData.Length - (inputData.Length % 2), temp, 0, (inputData.Length % 2));
        sb.AppendFormat("\n0x{0:X4}",System.BitConverter.ToUInt32(temp,0));
      }
      else
      {
        // delete comma from last one 
        sb.Remove(sb.Length - 1, 1);
      }
      sb.Append("\n};\n");
    }
    
    private static void bin2uintArray(String arrayName, Byte[] inputData, StringBuilder sb)
    {
      sb.Append("const unsigned int "+ arrayName + "[] = {");
      
      // Output data in hex format
      for (int i = 0; i<inputData.Length/4; i++)
      {
        UInt32 data = System.BitConverter.ToUInt32(inputData, i*4);
        sb.AppendFormat("\n0x{0:X8},",data);
      }
      
      if ((inputData.Length % 4) != 0)
      {
        Byte[] temp = new Byte[] {0x00,0x00,0x00,0x00};
        System.Array.Copy(inputData,inputData.Length - (inputData.Length % 4), temp, 0, (inputData.Length % 4));
        sb.AppendFormat("\n0x{0:X8}",System.BitConverter.ToUInt32(temp,0));
      }
      else
      {
        // delete comma from last one 
        sb.Remove(sb.Length - 1, 1);
      }
      sb.Append("\n};\n");
    }
  }

  /*  
  class Text
  {
  
  }
  */

}
