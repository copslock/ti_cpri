/*
 * Endian.cs
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
using System.Collections;

namespace TI.UtilLib.IO
{

    /// <summary>
    /// Public class to express big and Little endianness.
    /// Only creatable via BigEndian and LittleEndian public property.
    /// </summary>
    public class Endian
    {
        #region Private members and methods
        // Private internal members and constants
        private UInt32 endianness;
        private const UInt32 Big = 0x0;
        private const UInt32 Little = 0x1;

        /// <summary>
        /// Private hidden constructor
        /// </summary>
        private Endian()
        {
        }

        /// <summary>
        /// Private hidden constructor - called from public properties
        /// </summary>
        /// <param name="endian">UInt32 indicating endianness - provided from private constants</param>
        private Endian(UInt32 endian)
        {
            endianness = endian;
        }
        #endregion

        #region Public properties and methods
        /// <summary>
        /// Public static property that returns an Endian object of Big type
        /// </summary>
        public static Endian BigEndian
        {
            get
            {
                return new Endian(Big);
            }
        }

        /// <summary>
        /// Public static property that returns an Endian object of the Little type
        /// </summary>
        public static Endian LittleEndian
        {
            get
            {
                return new Endian(Little);
            }
        }

        /// <summary>
        /// Public static function to reverse endianness of the byte array
        /// </summary>
        /// <param name="bytes">Input byte array</param>
        /// <returns></returns>
        public static byte[] swapEndian(byte[] bytes)
        {
            Int32 length = bytes.Length;
            Int32 halfLen = bytes.Length / 2;
            Byte temp;
            for (int i = 0; i < halfLen; i++)
            {
                temp = bytes[length - i - 1];
                bytes[length - i - 1] = bytes[i];
                bytes[i] = temp;
            }
            return bytes;
        }

        /// <summary>
        /// Check if this object is LittleEndian
        /// </summary>
        /// <returns>Boolean True if object is little endian</returns>
        public Boolean isLittleEndian()
        {
            return (endianness == Little);
        }

        /// <summary>
        /// Check if this object if BigEndian
        /// </summary>
        /// <returns>Boolean True if oject if big endian</returns>
        public Boolean isBigEndian()
        {
            return (endianness == Big);
        }

        public new string ToString()
        {
            if (endianness == Big)
                return "big";
            else
                return "little";
        }

        #endregion
        
        # region Public operator overloading
        
        public static Boolean operator== (Endian endian1, Endian endian2)
        {
          return endian1.Equals(endian2);
        }
        
        public static Boolean operator!= (Endian endian1, Endian endian2)
        {
          return !endian1.Equals(endian2);
        }
        
        // Override of Equals from object class
        public override bool Equals( object endian )
        {
          if (endian is Endian)
          {
            return Equals( (Endian) endian );
          }
          else 
          {
            return false;
          }
        }
        
        // Equals operator for this class
        public bool Equals (Endian endian)
        {
          return (bool)( this.endianness == endian.endianness);
        }
        
        // Must override this if overriding Equals from object class
        public override int GetHashCode()
        {
          return (int) this.endianness;
        }
        
        #endregion

    }

    /// <summary>
    /// Wrapper class to add endianness to the BinaryReader class
    /// </summary>
    public class EndianBinaryReader : BinaryReader
    {
        private Endian myEndian;

        public Endian endianness
        {
            get { return myEndian; }
        }

        /// <summary>
        /// Public EndianBinaryReader constructor
        /// </summary>
        /// <param name="str">Stream</param>
        /// <param name="endian">Endian object</param>
        public EndianBinaryReader(Stream str, Endian endian)
            : base(str)
        {
            myEndian = endian;
        }

        /// <summary>
        /// Public EndianBinaryReader constructor
        /// </summary>
        /// <param name="str">Base Stream object</param>
        /// <param name="enc">Encoding object</param>
        /// <param name="endian">Endian object</param>
        public EndianBinaryReader(Stream str, Encoding enc, Endian endian)
            : base(str, enc)
        {
            myEndian = endian;
        }

        public new short ReadInt16()
        {
            if (myEndian.isBigEndian())
            {
                return BitConverter.ToInt16(Endian.swapEndian(BitConverter.GetBytes(base.ReadInt16())), 0);
            }
            else
            {
                return base.ReadInt16();
            }
        }

        public new int ReadInt32()
        {
            if (myEndian.isBigEndian())
            {
                return BitConverter.ToInt32(Endian.swapEndian(BitConverter.GetBytes(base.ReadInt32())), 0);
            }
            else
            {
                return base.ReadInt32();
            }
        }

        public new long ReadInt64()
        {
            if (myEndian.isBigEndian())
            {
                return BitConverter.ToInt64(Endian.swapEndian(BitConverter.GetBytes(base.ReadInt64())), 0);
            }
            else
            {
                return base.ReadInt64();
            }
        }

        public new ushort ReadUInt16()
        {
            if (myEndian.isBigEndian())
            {
                return BitConverter.ToUInt16(Endian.swapEndian(BitConverter.GetBytes(base.ReadUInt16())), 0);
            }
            else
            {
                return base.ReadUInt16();
            }
        }

        public new uint ReadUInt32()
        {
            if (myEndian.isBigEndian())
            {
                return BitConverter.ToUInt32(Endian.swapEndian(BitConverter.GetBytes(base.ReadUInt32())), 0);
            }
            else
            {
                return base.ReadUInt32();
            }
        }

        public new ulong ReadUInt64()
        {
            if (myEndian.isBigEndian())
            {
                return BitConverter.ToUInt64(Endian.swapEndian(BitConverter.GetBytes(base.ReadUInt64())), 0);
            }
            else
            {
                return base.ReadUInt64();
            }
        }
    }

    /// <summary>
    /// Wrapper class to add endianness to the BinaryWriter class
    /// </summary>
    public class EndianBinaryWriter : BinaryWriter
    {
        private Endian myEndian;

        public Endian endianness
        {
            get { return myEndian; }
        }

        /// <summary>
        /// Public EndianBinaryWriter constructor
        /// </summary>
        /// <param name="str">Stream</param>
        /// <param name="endian">Endian object</param>
        public EndianBinaryWriter(Stream str, Endian endian)
            : base(str)
        {
            myEndian = endian;
        }

        /// <summary>
        /// Public EndianBinaryWriter constructor
        /// </summary>
        /// <param name="str">Base Stream object</param>
        /// <param name="enc">Encoding object</param>
        /// <param name="endian">Endian object</param>
        public EndianBinaryWriter(Stream str, Encoding enc, Endian endian)
            : base(str, enc)
        {
            myEndian = endian;
        }

        public new void Write(short value)
        {
            if (myEndian.isBigEndian())
            {
                base.Write(BitConverter.ToInt16(Endian.swapEndian(BitConverter.GetBytes(value)), 0));
            }
            else
            {
                base.Write(value);
            }
        }
        
        public void Write(short[] value)
        {
          if (value != null)
          {
            foreach(short val in value)
            {
              this.Write(val);
            }
          }
        }

        public new void Write(int value)
        {
          if (myEndian.isBigEndian())
          {
            base.Write(BitConverter.ToInt32(Endian.swapEndian(BitConverter.GetBytes(value)), 0));
          }
          else
          {
            base.Write(value);
          }
        }
        
        public void Write(int[] value)
        {
          if (value != null)
          {
            foreach(int val in value)
            {
              this.Write(val);
            }
          }
        }

        public new void Write(long value)
        {
          if (myEndian.isBigEndian())
          {
            base.Write(BitConverter.ToInt64(Endian.swapEndian(BitConverter.GetBytes(value)), 0));
          }
          else
          {
            base.Write(value);
          }
        }
        
        public void Write(long[] value)
        {
          if (value != null)
          {
            foreach(long val in value)
            {
              this.Write(val);
            }
          }
        }

        public new void Write(ushort value)
        {
          if (myEndian.isBigEndian())
          {
            base.Write(BitConverter.ToUInt16(Endian.swapEndian(BitConverter.GetBytes(value)), 0));
          }
          else
          {
            base.Write(value);
          }
        }
        
        public void Write(ushort[] value)
        {
          if (value != null)
          {
            foreach(ushort val in value)
            {
              this.Write(val);
            }
          }
        }

        public new void Write(uint value)
        {
          if (myEndian.isBigEndian())
          {
            base.Write(BitConverter.ToUInt32(Endian.swapEndian(BitConverter.GetBytes(value)), 0));
          }
          else
          {
            base.Write(value);
          }
        }
        
        public void Write(uint[] value)
        {
          if (value != null)
          {
            foreach(uint val in value)
            {
              this.Write(val);
            }
          }
        }

        public new void Write(ulong value)
        {
          if (myEndian.isBigEndian())
          {
            base.Write(BitConverter.ToUInt64(Endian.swapEndian(BitConverter.GetBytes(value)), 0));
          }
          else
          {
            base.Write(value);
          }
        }
        
        public void Write(ulong[] value)
        {
          if (value != null)
          {
            foreach(ulong val in value)
            {
              this.Write(val);
            }
          }
        }
    }
}