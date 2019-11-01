/*
 * SerialIO.cs
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
using System.IO.Ports;
using System.Reflection;
using System.Collections.Generic;
using System.Text;

namespace TI.UtilLib.IO
{
  public class SerialIO
  {
        /// <summary>
    /// Waitforsequence with option for verbosity
    /// </summary>
    /// <param name="str">String to look for</param>
    /// <param name="altStr">String to look for but don't want</param>
    /// <param name="sp">SerialPort object.</param>
    /// <param name="verbose">Boolean to indicate verbosity.</param>
    /// <returns>Boolean to indicate if str or altStr was found.</returns>
    public static Boolean waitForSequence(String str, String altStr, SerialPort sp, Boolean verbose)
    {
        Boolean strFound = false, altStrFound = false;
        Byte[] input = new Byte[256];
        String inputStr;
        Int32 i;
		sp.ReadTimeout=300000;
        while ((!strFound) && (!altStrFound))
        {
            
            i = 0;
            do
            {
                try
                {
                    input[i++] = (Byte)sp.ReadByte();
                }
                catch
                {
					//Console.Write(".");
                }
                //Console.Write(input[i - 1] + " ");
            } while ( (input[i - 1] != 0) &&
                      (i < (input.Length - 1)) &&
                      (input[i - 1] != 0x0A) &&
                      (input[i - 1] != 0x0D) );

            // Convert to string for comparison
            if ((input[i-1] == 0x0A) || (input[i-1] == 0x0D))
                inputStr = (new ASCIIEncoding()).GetString(input, 0, i-1);
            else
                inputStr = (new ASCIIEncoding()).GetString(input, 0, i);

            if (inputStr.Length == 0)
            {
                continue;
            }

            // Compare Strings to see what came back
            if (verbose)
                Console.WriteLine("\tTarget:\t{0}", inputStr);
            if (inputStr.Contains(altStr))
            {
                altStrFound = true;
                if (String.Equals(str, altStr))
                {
                    strFound = altStrFound;
                }
            }
            else if (inputStr.Contains(str))
            {
                strFound = true;
            }
            else
            {
                strFound = false;
            }
        }
        return strFound;
    }
    public static string readSequence(SerialPort sp, Boolean verbose)
    {
        Boolean strFound = false, altStrFound = false;
        Byte[] input = new Byte[256];
        String inputStr;
        Int32 i;
		sp.ReadTimeout=300000;
        while ((!strFound) && (!altStrFound))
        {
            
            i = 0;
            do
            {
                try
                {
                    input[i++] = (Byte)sp.ReadByte();
                }
                catch
                {
					//Console.Write(".");
                }
                //Console.Write(input[i - 1] + " ");
            } while ( (input[i - 1] != 0) &&
                      (i < (input.Length - 1)) &&
                      (input[i - 1] != 0x0A) &&
                      (input[i - 1] != 0x0D) );

            // Convert to string for comparison
            if ((input[i-1] == 0x0A) || (input[i-1] == 0x0D))
                inputStr = (new ASCIIEncoding()).GetString(input, 0, i-1);
            else
                inputStr = (new ASCIIEncoding()).GetString(input, 0, i);

            if (inputStr.Length == 0)
            {
                continue;
            }

            // Compare Strings to see what came back
            if (verbose)
                Console.WriteLine("\tTarget:\t{0}", inputStr);
						return inputStr;
           }
		return "FAIL";
    }
  }
}
