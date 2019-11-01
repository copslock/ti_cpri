/*
 * Debug.cs
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

namespace TI.UtilLib
{
    public class Debug
    {
        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(String value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }
        
        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(Boolean value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(Char value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="buffer">A unicode character array.</param>
        public static void DebugMSG(char[] buffer)
        {
#if DEBUG
            Console.WriteLine(buffer);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(int value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(double value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(float value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(decimal value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(short value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(long value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(object value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(uint value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }       

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(ulong value)
        {
#if DEBUG
            Console.WriteLine(value);
#endif
        }

        /// <summary>
        /// Writes the text representation of the specified array of objects, followed by the line terminator, to the standard
        /// output, but only if the current build is a DEBUG build.  
        /// </summary>
        /// <param name="value">The value to write.</param>
        public static void DebugMSG(String format, params Object[] arg)
        {
#if DEBUG
            Console.WriteLine(format, arg);
#endif
        }

    }
}
