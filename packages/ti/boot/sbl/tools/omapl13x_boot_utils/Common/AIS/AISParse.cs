/*
 * AISParse.cs
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
// ======================================================================= 
//  TEXAS INSTRUMENTS, INC.                                                
// ----------------------------------------------------------------------- 
//                                                                         
// AIS_Util_SEC.cs -- Implements AIS_Parser class for secure boot (C#)
//                                                                         
// Rev 0.0.1                                                               
//                                                                         
//  USAGE                                                                  
//      Include AIS_Util namespace in your project and use the following
//      constructor:
//                                                                         
//          AIS_Parser parser = new AIS_Parser
//                                  (
//                                      AIS_Parser.AIS_<bootPeripheral>,
//                                      FxnDelegate_msg_log,
//                                      FxnDelegate_<bootPeripheral>_read,
//                                      FxnDelegate_<bootPeripheral>_write
//                                  );
//
//      Call parsing function using the contents of a binary AIS file
//      stored in a Byte array:
//
//          parser.boot(ais_file_contents);
//                                                                         
//  DESCRIPTION                                                            
//      Parses a binary AIS file passed in as a Byte array.  Uses external
//      functions (passed as delegates) for I/O read and write and message
//      logging.  Performs all host operations as described in "Using the
//      D800K001 Bootloader" application note for I2C slave, SPI slave, and
//      UART boot modes.
//                                                                         
// ----------------------------------------------------------------------- 
//            Copyright (c) 2009 Texas Instruments, Incorporated.          
//                           All Rights Reserved.                          
// ======================================================================= 

// Security mode (default: custom secure)
//#define AIS_GENERIC_SECURE

/****************************************************************
 *  TI AIS Parsing Class
 *  (C) 2007-2009, Texas Instruments, Inc.                           
 *                                                              
 * Author:  Joseph Coombs (minor updates by Daniel Allred)
 *                                                              
 ****************************************************************/

using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using TI.AISLib;

namespace TI.AISLib
{
  // host function delegates (rx, tx, log)
  delegate AisStatus ReadFxnDelegate(Byte[] rcvBuf, Int32 index, Int32 rcvSize, Int32 timeout);
  delegate AisStatus WriteFxnDelegate(Byte[] xmtBuf, Int32 index, Int32 xmtSize, Int32 timeout);
  delegate void LogFxnDelegate(String line);
 
  class AIS_Parser
  {
    // Public data members
    public Int32 posN, ioBits, ioDelay, opcodeDelay, ioTimeout;
    public bool waitBOOTME;

    // Private members
    private AisSecureType secureType;
    private AisHostType hostDevice;
    private ReadFxnDelegate readFxn;
    private WriteFxnDelegate writeFxn;
    private LogFxnDelegate logFxn;
    private Int32 sigSize;

    // Public Constructor
    public AIS_Parser(AisHostType hostType, AisSecureType secType, LogFxnDelegate hostLogFxn, ReadFxnDelegate hostReadFxn, WriteFxnDelegate hostWriteFxn)
    {
      // apply specified params
      this.hostDevice = hostType;
      this.readFxn = hostReadFxn;
      this.writeFxn = hostWriteFxn;
      this.logFxn = hostLogFxn;
      this.secureType = secType;

      // use defaults for others
      posN = 2;
      ioBits = 8;
      ioDelay = 0;
      opcodeDelay = 5;
      ioTimeout = 5000; // 5s
      waitBOOTME = true;
      
      if (secureType == AisSecureType.GENERIC)
      {
        sigSize = 32;
      }
      else if (secureType == AisSecureType.CUSTOM)
      {
        sigSize = 128; // default to RSA1024 to start
      }
      else
      {
        sigSize = 0;
      }
    }

    // Private utility functions (were C macros)
    private UInt32 AIS_XMT_START(Int32 bits)
    {
      // return (bits) MSBs or AIS start word
      return ((UInt32)AisOps.XmtStartWord) >> (32 - bits);
    }

    private UInt32 AIS_RCV_START(Int32 bits)
    {
      // return (bits) MSBs or AIS start word ACK
      return ((UInt32)AisOps.RcvStartWord) >> (32 - bits);
    }

    private UInt32 AIS_opcode2ack(UInt32 opcode)
    {
      // return opcode ACK
      return (opcode & 0xF0FFFFFF) | 0x02000000;
    }

    private Int32 LOCAL_roundUpTo(Int32 num, Int32 mod)
    {
      // round (num) up to nearest multiple of (mod)
      return ( (num + (mod - 1)) / mod * mod);
    }

    private UInt32 LOCAL_b2UInt32(Byte[] ba)
    {
      // convert Byte array to UInt32 with little endian order
      return (UInt32)ba[0] + ((UInt32)ba[1] << 8) + ((UInt32)ba[2] << 16) + ((UInt32)ba[3] << 24);
    }

    private Byte[] LOCAL_UInt322b(UInt32 ui)
    {
      // convert UInt32 to Byte array with little endian order
      Byte[] ba = new Byte[4];
      ba[0] = (Byte)(ui & 0xFF);
      ba[1] = (Byte)((ui >> 8) & 0xFF);
      ba[2] = (Byte)((ui >> 16) & 0xFF);
      ba[3] = (Byte)((ui >> 24) & 0xFF);
      return ba;
    }

    private void LOCAL_delay(Int32 N)
    {
      Thread.Sleep(N);
    }
      
    // Start Word Sync Function
    private AisStatus AIS_SWS()
    {
      UInt32 rcvWord = 0;
      Byte[] rcvWordB = new Byte[4];
      UInt32 xmtWord = AIS_XMT_START(ioBits);
      AisStatus status = AisStatus.IN_PROGRESS;

      while (true)
      {
        // send xmt start
        status |= LOCAL_bufWrite(LOCAL_UInt322b(xmtWord), 0, ioBits / 8, ioTimeout);
        if (status < 0)
        {
          status = 0;
          LOCAL_delay(opcodeDelay);
          continue;
        }

        // receive word
        status |= LOCAL_bufRead(rcvWordB, 0, ioBits / 8, ioTimeout);
        rcvWord = LOCAL_b2UInt32(rcvWordB);

        // fail on IO error
        if (status < 0)
        {
          return AisStatus.ERROR;
        }

        // break if word is rcv start
        if (rcvWord == AIS_RCV_START(ioBits))
        {
                break;
        }
      }

      return AisStatus.IN_PROGRESS;
    }
    
    private AisStatus AIS_POS(UInt32 command)
    {
        UInt32 xmtWord = command;
        UInt32 rcvWord;
        Byte[] rcvWordB = new Byte[4];
        AisStatus status;
        Int32  i;

        // 1. send ping
        status = LOCAL_bufWrite(LOCAL_UInt322b(xmtWord), 0, 4, ioTimeout);
        // receive pong
        status |= LOCAL_bufRead(rcvWordB, 0, 4, ioTimeout);
        rcvWord = LOCAL_b2UInt32(rcvWordB);

        // fail on improper response or IO error
        if (rcvWord != AIS_opcode2ack(xmtWord) || status < 0)
            return AisStatus.ERROR;

        LOCAL_delay(opcodeDelay);

        // 2. send N
        xmtWord = (UInt32)posN;
        // send ping
        status |= LOCAL_bufWrite(LOCAL_UInt322b(xmtWord), 0, 4, ioTimeout);
        // receive pong
        status |= LOCAL_bufRead(rcvWordB, 0, 4, ioTimeout);
        rcvWord = LOCAL_b2UInt32(rcvWordB);

        // fail on improper response or IO error
        if (rcvWord != posN || status < 0)
        {
          return AisStatus.ERROR;
        }

        // 3. send/receive numerical sequence
        for (i = 1; i <= posN; i++)
        {
          LOCAL_delay(opcodeDelay);

          xmtWord = (UInt32)i;
          status |= LOCAL_bufWrite(LOCAL_UInt322b(xmtWord), 0, 4, ioTimeout);
          status |= LOCAL_bufRead(rcvWordB, 0, 4, ioTimeout);
          rcvWord = LOCAL_b2UInt32(rcvWordB);

          // fail on improper response or IO error
          if (rcvWord != xmtWord || status < 0)
          {
            return AisStatus.ERROR;
          }
        }

        return AisStatus.IN_PROGRESS;
    }

    private AisStatus AIS_OS(UInt32 command)
    {
      UInt32 xmtWord = command;
      UInt32 rcvWord = 0;
      Byte[] rcvWordB = new Byte[4];
      AisStatus status = AisStatus.IN_PROGRESS;
      Int32 retryCnt = 0;
      Int32 retryCntMax = 10;

      while (true)
      {
        // send ping
        status |= LOCAL_bufWrite(LOCAL_UInt322b(xmtWord), 0, 4, ioTimeout);
        // receive pong
        if (status >= 0)
        {
          status |= LOCAL_bufRead(rcvWordB, 0, 4, ioTimeout);
          rcvWord = LOCAL_b2UInt32(rcvWordB);
        }

        // fail on IO error
        if (status < 0)
        {
          LOCAL_delay(opcodeDelay);
          if (retryCnt++ >= retryCntMax)
          {
            logFxn(String.Format("(AIS Parse): Opcode Sync failed after {0} consecutive I/O failures.", retryCnt));
            return AisStatus.ERROR;
          }

          // send zero word (32 bits) to clear potentially corrupted state on target
          status |= LOCAL_bufWrite(LOCAL_UInt322b(0), 0, 4, ioTimeout);

          status = 0;
          continue;
        }

        // pass on proper response
        if (rcvWord == AIS_opcode2ack(xmtWord))
        {
          if (retryCnt > 0)
          {
            logFxn( String.Format("(AIS Parse): Opcode Sync passed after {0} consecutive I/O failures.", retryCnt) );
          }
          return AisStatus.IN_PROGRESS;
        }
      }
    }

    // Read Int32 (4 Bytes) and advance cursor
    private UInt32 LOCAL_parseInt(Byte[] ais, ref Int32 cursor)
    {
      UInt32 token = 0;

      token = BitConverter.ToUInt32(ais, cursor);
      cursor += 4;
      
      return token;
    }

    // Advance cursor arbitrary number of Bytes (ignore data)
    private void LOCAL_parseSkip(ref Int32 cursor, Int32 n)
    {
      cursor += n;
    }

    private AisStatus LOCAL_bufRead(Byte[] buffer, Int32 index, Int32 Bytes, Int32 timeout)
    {
      Int32 rcvSize = ioBits / 8;
      AisStatus status = AisStatus.IN_PROGRESS;

        // check that we can read specified Byte count cleanly
        if (Bytes % rcvSize != 0)
        {
            logFxn( String.Format("(AIS Parse): Cannot read {0} Bytes in chunks of {1}!", Bytes, rcvSize) );
            return AisStatus.ERROR;
        }

      // perform IO transaction in N-bit "bites"
      for (Int32 i = 0; i < Bytes / rcvSize; i++)
      {
        status |= readFxn(buffer, index + i * rcvSize, rcvSize, timeout);
        LOCAL_delay(ioDelay);

        if (status < 0)
        {
          logFxn("(AIS Parse): I/O Error in read!");
          break;
        }
      }

      return status;
    }

    private AisStatus LOCAL_bufWrite(Byte[] buffer, Int32 index, Int32 Bytes, Int32 timeout)
    {
      Int32 xmtSize = ioBits / 8;
      AisStatus status = AisStatus.IN_PROGRESS;

      // check that we can write specified Byte count cleanly
      if (Bytes % xmtSize != 0)
      {
        logFxn( String.Format("(AIS Parse): Cannot write {0} Bytes in chunks of {1}!", Bytes, xmtSize) );
        return AisStatus.ERROR;
      }
      
      // perform IO transaction in N-bit "bites"
      for (Int32 i = 0; i < Bytes / xmtSize; i++)
      {
        status |= writeFxn(buffer, index + i * xmtSize, xmtSize, timeout);
        LOCAL_delay(ioDelay);

        if (status < 0)
        {
          logFxn("(AIS Parse): I/O Error in write!");
          break;
        }
      }

      return status;
    }

    // Public boot loading function
    public AisStatus boot(Byte[] AIS_Contents)
    {
      Int32 AIS_Cursor = 0;
      UInt32 command, addr, size, type,
           sleep, data, crc, crcGuess,
           seek, exp, mask, val,
           args;
      Byte[] crcGuessB = new Byte[4];
      Int32 i;
      Int32 BytesLeft;
      AisStatus status;
      Int32 opsRead = 0;
      bool secureMode = false;

      // check for magic word first
      command = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
      status = (command == (UInt32) AisOps.MagicNumber) ? AisStatus.IN_PROGRESS : AisStatus.ERROR;
      logFxn(String.Format("(AIS Parse): Read magic word 0x{0:X8}.", command));

      // UART only: read "BOOTME "
      if ((status == AisStatus.IN_PROGRESS) && (hostDevice == AisHostType.UART) && waitBOOTME)
      {
        Byte[] rcvInit = new Byte[1];
        Byte[] corInit = { (Byte)'B', (Byte)'O', (Byte)'O', (Byte)'T',
                                 (Byte)'M', (Byte)'E', (Byte)' ', (Byte)0 };

        logFxn("(AIS Parse): Waiting for BOOTME... (power on or reset target now)");
        //status = readFxn(rcvInit, 0, 8, -1);

        BytesLeft=7;
        while(true) 
        {
            status = readFxn(rcvInit, 0, 1, -1);

            //Allow for leading NULL character
            if (BytesLeft == 7 && rcvInit[0] == (Byte)0) 
            {
                continue;
            }        

            //Exit after trailing NULL character received
            if (BytesLeft == 0 && rcvInit[0] == (Byte)0)
            {
                break;
            }

            //Error if read something other than 'BOOTME'
            else if (rcvInit[0] != corInit[7 - BytesLeft])
            {
                logFxn("(AIS Parse): Read invalid BOOTME string.");
                status = AisStatus.ERROR;
                break;
            }
            else
            {
                BytesLeft--;
            }
        }
        // report if correct BOOTME is received
        if (status == AisStatus.IN_PROGRESS)
            logFxn("(AIS Parse): BOOTME received!");
      }
      
      while (status == AisStatus.IN_PROGRESS)
      {
        // perform synchronization on first pass
        if (opsRead == 0)
        {
          // perform SWS
          logFxn("(AIS Parse): Performing Start-Word Sync...");
          status = AIS_SWS();
          if (status == AisStatus.ERROR)
          {
            break;    // fail if SWS fails
          }

          // perform POS
          logFxn("(AIS Parse): Performing Ping Opcode Sync...");
          status = AIS_POS((UInt32)AisOps.Ping);
          if (status == AisStatus.ERROR)
          {
            continue;  // retry SWS if POS fails
          }
        }

        // delay; give bootloader a chance to process previous command
        LOCAL_delay(opcodeDelay);

        // read a command
        command = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
        logFxn( String.Format("(AIS Parse): Processing command {0}: 0x{1:X8}.", opsRead, command) );
        opsRead++;

        // perform Op-code sync
        logFxn("(AIS Parse): Performing Opcode Sync...");
        status = AIS_OS(command);

        //
        if (status == AisStatus.ERROR)
          break;    // fail if OS fails

        switch((AisOps)command)
        {
          case AisOps.Set:
            logFxn("(AIS Parse): Loading boot table...");
            // read: type, addr, data, sleep
            type = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            data = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            sleep = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            // send: type, addr, data, sleep
            status |= LOCAL_bufWrite(LOCAL_UInt322b(type), 0, 4, ioTimeout);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(data), 0, 4, ioTimeout);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(sleep), 0, 4, ioTimeout);
            if (secureMode)
            {
              // secure mode: send key (custom secure: 128 Bytes; generic secure: 32 Bytes)
              status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize, ioTimeout);
              LOCAL_parseSkip(ref AIS_Cursor, sigSize);
              logFxn("(AIS Parse): Secure mode; sending signature.");
            }
            break;

          case AisOps.SeqReadEnable:
            // no extra IO required
            logFxn("(AIS Parse): No slave memory present; Sequential Read Enable has no effect.");
            break;

          case AisOps.Section_Load:
          case AisOps.CmpSection_Load:
            logFxn("(AIS Parse): Loading section...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            // send size
            size = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(size), 0, 4, ioTimeout);
            // send data
            status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, LOCAL_roundUpTo((Int32)size, 4), ioTimeout);
                    LOCAL_parseSkip(ref AIS_Cursor, LOCAL_roundUpTo((Int32)size, 4));
            logFxn( String.Format("(AIS Parse): Loaded {0}-Byte section to address 0x{1:X8}.", size, addr) );
                    break;

          case AisOps.Section_Fill:
            logFxn("(AIS Parse): Filling section...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            // send size
            size = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(size), 0, 4, ioTimeout);
            // send type
            type = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(type), 0, 4, ioTimeout);
            // send pattern (data)
            data = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(data), 0, 4, ioTimeout);
            logFxn( String.Format("(AIS Parse): Filled {0}-Byte section with pattern 0x{1:X8}.", size, data) );
            break;

          case AisOps.SecureKeyLoad:
            logFxn("(AIS Parse): Secure key loading, entering secure mode.");

            // If custom secure device, get sigSize from key data
            if (secureType == AisSecureType.CUSTOM)
            {
              // Send exponent
              exp = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
              status |= LOCAL_bufWrite(LOCAL_UInt322b(exp), 0, 4, ioTimeout);
              
              // Send modulus size
              size = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
              status |= LOCAL_bufWrite(LOCAL_UInt322b(size), 0, 4, ioTimeout);
              // NOTE: use only upper half; value listed in 2-Byte chunks (convert to Bytes)
              size = (size >> 16) * 2;
              
              // Update signature size based on modulus size
              sigSize = (Int32) size;
              
            }
            else if (secureType == AisSecureType.GENERIC)
            {
              size = (UInt32) sigSize;
            }
            else
            {
              size = 0;
            }


            // Send key data (modulus for Custom, key header for Generic)
            status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, (Int32)size, ioTimeout);
            LOCAL_parseSkip(ref AIS_Cursor, (Int32)size);
            // enable secure mode
            secureMode = true;
            break;

          case AisOps.SecSection_Load:
            logFxn("(AIS Parse): Loading secure section...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            // send size
            size = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(size), 0, 4, ioTimeout);
            // send reserved word
            val = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(val), 0, 4, ioTimeout);
            // send data
                    status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, LOCAL_roundUpTo((Int32)size, 4), ioTimeout);
                    LOCAL_parseSkip(ref AIS_Cursor, LOCAL_roundUpTo((Int32)size, 4));
            logFxn( String.Format("(AIS Parse): Loaded {0}-Byte section to address 0x{1:X8}.", size, addr) );
            break;

          case AisOps.EncSection_Load:
            if (!secureMode)
            {
              logFxn( String.Format("Secure opcode (0x{0:X8}), not allowed.", command) );
              status = AisStatus.ERROR;
              break;
            }
          
            logFxn("(AIS Parse): Loading encoded section...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            // send size
            size = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(size), 0, 4, ioTimeout);
            

#if AIS_D800K001
            // send reserved word
            val = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(val), 0, 4, ioTimeout);
            // send data
                    status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, (Int32)LOCAL_roundUpTo((Int32)size, 16), ioTimeout);
                    LOCAL_parseSkip(ref AIS_Cursor, LOCAL_roundUpTo((Int32)size, 16));
#else
            // AIS contains at least 16 Bytes
            if (size < 16) size = 16;
            // send data
            status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, (Int32)size, ioTimeout);
            LOCAL_parseSkip(ref AIS_Cursor, (Int32)size);
#endif
            logFxn( String.Format("(AIS Parse): Loaded {0}-Byte section to address 0x{1:X8}.", size, addr) );
            break;

          case AisOps.SetSecExitMode:
            if (!secureMode)
            {
              logFxn( String.Format("Secure opcode (0x{0:X8}), not allowed.", command) );
              status = AisStatus.ERROR;
              break;
            }
            logFxn("(AIS Parse): Setting boot exit mode...");
            // send mode value (32-bit)
            val = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(val), 0, 4, ioTimeout);
            logFxn( String.Format("(AIS Parse): Set exit mode to 0x{0:X8}.", val) );
            break;

          case AisOps.DisableCRC:
            // no extra IO required
            logFxn("(AIS Parse): CRC disabled.");
            break;

          case AisOps.EnableCRC:
            // no extra IO required
            logFxn("(AIS Parse): CRC enabled.");
            break;

          case AisOps.RequestCRC:
            logFxn("(AIS Parse): Requesting CRC...");
            // read computed CRC
            status |= LOCAL_bufRead(crcGuessB, 0, 4, ioTimeout);
                    crcGuess = LOCAL_b2UInt32(crcGuessB);
            crc = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            if (crcGuess == crc)
            {
              // CRC succeeded.  Skip seek value to reach next opcode
              logFxn("(AIS Parse): CRC passed!");
              LOCAL_parseSkip(ref AIS_Cursor, 4);
            }
            else
            {
              // CRC error; send startover opcode and seek AIS
              logFxn("(AIS Parse): CRC failed!  Sending STARTOVER...");
              status |= AIS_OS((UInt32)AisOps.Start_Over);
              // seek AIS
              seek = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
              logFxn( String.Format("(AIS Parse): {0}-Byte seek applied.", seek) );
                        LOCAL_parseSkip(ref AIS_Cursor, (Int32)seek);
            }
            break;
            
          case AisOps.ReadWait:
            logFxn("(AIS Parse): Performing read-wait...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            // send mask
            mask = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(mask), 0, 4, ioTimeout);
            // send value
            val = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(val), 0, 4, ioTimeout);
            if (secureMode)
            {
              // secure mode: send key
              status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize, ioTimeout);
              LOCAL_parseSkip(ref AIS_Cursor, sigSize);
              logFxn("(AIS Parse): Secure mode; sending signature.");
            }
            break;

          case AisOps.FunctionExec:
            logFxn("(AIS Parse): Executing function...");
            // send function number and number of arguments
            args = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(args), 0, 4, ioTimeout);
            args = (args & 0xFFFF0000) >> 16;
            for (i = 0; i < args; i++)
            {
              // send arg i
              val = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
              status |= LOCAL_bufWrite(LOCAL_UInt322b(val), 0, 4, ioTimeout);
            }
            if (secureMode)
            {
              // secure mode: send key
              status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize, ioTimeout);
              LOCAL_parseSkip(ref AIS_Cursor, sigSize);
              logFxn("(AIS Parse): Secure mode; sending signature.");
            }
            break;

          case AisOps.Jump:
            logFxn("(AIS Parse): Performing jump...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            if (secureMode)
            {
              // secure mode: send key
              status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize, ioTimeout);
              LOCAL_parseSkip(ref AIS_Cursor, sigSize);
              logFxn("(AIS Parse): Secure mode; sending signature.");
            }
            // TODO:  wait?
            logFxn( String.Format("(AIS Parse): Jump to address 0x{0:X8}.", addr) );
            break;

          case AisOps.Jump_Close:
            logFxn("(AIS Parse): Performing jump and close...");
            // send address
            addr = LOCAL_parseInt(AIS_Contents, ref AIS_Cursor);
            status |= LOCAL_bufWrite(LOCAL_UInt322b(addr), 0, 4, ioTimeout);
            if (secureMode)
            {
              // secure mode: send signature
              status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize, ioTimeout);
              LOCAL_parseSkip(ref AIS_Cursor, sigSize);
              logFxn("(AIS Parse): Secure mode; sending signature.");
            }
            // parsing complete
            status = AisStatus.COMPLETE;
            logFxn( String.Format("(AIS Parse): AIS complete. Jump to address 0x{0:X8}.", addr) );
            break;

          case AisOps.Start_Over:
            // control should never pass here; opcode is not present in AIS files
            break;
          
          case AisOps.SetDelegateKey:
            if (!secureMode)
            {
              logFxn( String.Format("Secure opcode (0x{0:X8}), not allowed.", command) );
              status = AisStatus.ERROR;
              break;
            }
            
            logFxn("(AIS Parse): Setting Delegate Key...");
            
            // secure mode: send key Certificate
            status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize + 48, ioTimeout);
            LOCAL_parseSkip(ref AIS_Cursor, sigSize + 48);
            logFxn("(AIS Parse): Secure mode - sending delegate key certificate .");
            
            // secure mode: send signature
            status |= LOCAL_bufWrite(AIS_Contents, AIS_Cursor, sigSize, ioTimeout);
            LOCAL_parseSkip(ref AIS_Cursor, sigSize);
            logFxn("(AIS Parse): Secure mode - sending signature.");
            
            break;
            
          case AisOps.RemDelegateKey:
            if (!secureMode)
            {
              logFxn( String.Format("Secure opcode (0x{0:X8}), not allowed.", command) );
              status = AisStatus.ERROR;
              break;
            }
            logFxn("(AIS Parse): Removing Delegate Key.");
            break;

          // Unrecognized opcode
          default:
            logFxn( String.Format("(AIS Parse): Unhandled opcode (0x{0:X8}).", command) );
            status = AisStatus.ERROR;
                    break;
        }
      }

      // UART only: read "   DONE"
      if ((status == AisStatus.COMPLETE) && (hostDevice == AisHostType.UART))
      {
        Byte[] rcvEnd = new Byte[8];
        Byte[] corEnd = { (Byte)' ', (Byte)' ', (Byte)' ', (Byte)'D',
                              (Byte)'O', (Byte)'N', (Byte)'E', (Byte)0 };

        logFxn("(AIS Parse): Waiting for DONE...");
        status = readFxn(rcvEnd, 0, 8, -1);

        // fail on incorrect sequence or IO error
        for (i = 0; i < 7; i++)
        {
          if (rcvEnd[i] != corEnd[i])
          {
            logFxn("(AIS Parse): Read invalid DONE string.");
            status = AisStatus.ERROR;
            break;
          }
        }
        
        if (status != AisStatus.ERROR)
        {
          // success
          status = AisStatus.COMPLETE;
        }
      }            

      if (status == AisStatus.COMPLETE)
      {
        logFxn("(AIS Parse): Boot completed successfully.");
      }
      else
      {
        logFxn("(AIS Parse): Boot aborted.");
      }
      return status;
    }


  }
}
