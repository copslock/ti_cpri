/* --------------------------------------------------------------------------
    FILE        : HexAIS.cs
    PURPOSE     : TI Booting and Flashing Utilities
    DESC        : Non-secure AIS generator
 ----------------------------------------------------------------------------- */

using System;
using System.Text;
using System.IO;
using System.Globalization;
using System.Reflection;
using System.Collections;
using System.Collections.Generic;

using TI.AISLib;
using TI.UtilLib;
using TI.UtilLib.IO;
using TI.UtilLib.Ini;
using TI.UtilLib.HexConv;

namespace TIBootAndFlash
{
  partial class Program
  {
    private enum ConvType: uint
    {
      Exec2Bin    = 0,
      Exec2CArray = 1,
      Exec2Srec   = 2,
      Exec2Text   = 3
    };

    private struct ProgramCmdParams
    {
      // Program conversion type
      public ConvType convType;

      public List<String> inputFileName;

      public String iniFileName;

      public String outFileName;
      
      public String cArrayName;
      
      public UInt64 srecAddr;
      
      public UInt64 entryPoint; 

      public Boolean valid;
    }
      
    /// <summary>
    /// Function to display help in case command-line is invalid
    /// </summary>
    static void DispHelp()
    {
      Console.Write("Usage:\n\n");
      Console.Write("HexAIS_"+devString+"[Options] [Input File Names]\n");
      Console.Write("\t" + "<Option> can be any of the following:\n");
      Console.Write("\t\t" + "-h                   \tShow this help screen.\n");
      Console.Write("\t\t" + "-entrypoint <Addr>   \tForce specified entry point for AIS boot image.\n");        
      Console.Write("\t\t" + "-ini <INI file name> \tSpecify the ini file (default is " + devString + ".ini).\n");            
      Console.Write("\t\t" + "-otype <Output Type> \tSpecify type of output file, from following:.\n");
      Console.Write("\t\t" + "     binary          \t     Create a binary AIS file (.bin), default.\n");
      Console.Write("\t\t" + "     carray[:name]   \t     Create a text file with AIS data in a C array (with optional name).\n");
      Console.Write("\t\t" + "     srecord@addr    \t     Create a Motorola S-record format (.srec) at addr.\n");
      Console.Write("\t\t" + "     text            \t     Create a text file with AIS data as ASCII text (.txt).\n");
      Console.Write("\t\t" + "-o <Output File Name>\tExplicitly specify the output filename.\n");
      Console.Write("\t\t" + "                     \tDefault is input file name with extension based on\n");
      Console.Write("\t\t" + "                     \toutput type.\n");
      Console.Write("\n");
    }

    /// <summary>
    /// Function to parse the command line
    /// </summary>
    /// <param name="args">Array of command-line arguments</param>
    /// <returns>Struct of the filled in program arguments</returns>
    static ProgramCmdParams ParseCmdLine(String[] args)
    {
      ProgramCmdParams myCmdParams = new ProgramCmdParams();
      Boolean[] argsHandled = new Boolean[args.Length];

      Int32 numUnhandledArgs, numHandledArgs = 0;
      String defaultExtension;

      // Set Defaults
      myCmdParams.valid = true;
      myCmdParams.outFileName = null;
      myCmdParams.iniFileName = null;
      myCmdParams.inputFileName = null;
      myCmdParams.srecAddr = 0xFFFFFFFF;
      myCmdParams.entryPoint = 0xFFFFFFFF;
      myCmdParams.convType = ConvType.Exec2Bin;
      defaultExtension = ".bin";

      // Initialize array of handled argument booleans to false
      for (int i = 0; i < argsHandled.Length; i++)
      {
        argsHandled[i] = false;
      }
      
      // For loop to check for all dash options
      for (int i = 0; i < args.Length; i++)
      {
        if (args[i].StartsWith("-"))
        {
          switch (args[i].Substring(1).ToLower())
          {
            case "entrypoint":
            {
              UInt32 temp;
              if (args[i + 1].StartsWith("0x") || args[i + 1].StartsWith("0X"))
              {
                if (!UInt32.TryParse(args[i + 1].Replace("0x", ""), NumberStyles.HexNumber, null, out temp))
                {
                  Console.WriteLine("WARNING: Invalid entrypoint address, {0}. Ignoring...", args[i + 1]);
                }
                else
                {
                  myCmdParams.entryPoint = temp;
                }
              }
              else if (UInt32.TryParse(args[i + 1], out temp))
              {
                myCmdParams.entryPoint = temp;
              }
              else
              {
                Console.WriteLine("WARNING: Invalid entrypoint address, {0}. Ignoring...", args[i + 1]);
              }
              
              argsHandled[i + 1] = true;
              numHandledArgs++;
              break;
            }
            case "otype":
            {
              // Handle possible srecord@addr, carray:name case
              String[] otype = args[i + 1].Split(new Char [] {'@',':'});
              
              if (otype[0].ToLower().Equals("binary"))
              {
                myCmdParams.convType = ConvType.Exec2Bin;
                defaultExtension = ".bin";
              }
              else if (otype[0].ToLower().Equals("carray"))
              {
                myCmdParams.convType = ConvType.Exec2CArray;
                defaultExtension = ".c";
                if (otype.Length == 1)
                {
                  myCmdParams.cArrayName = "data_array";
                }
                else if (otype.Length == 2)
                {
                  myCmdParams.cArrayName = otype[1];
                }
                else
                {
                  myCmdParams.valid = false;
                }
              }
              else if (otype[0].ToLower().Equals("text"))
              {
                myCmdParams.convType = ConvType.Exec2Text;
                defaultExtension = ".txt";
              }
              else if (otype[0].ToLower().Equals("srecord"))
              {
                if (otype.Length == 2)
                {
                  UInt32 temp;
                  otype[1] = otype[1].ToLower();
                  if (otype[1].StartsWith("0x"))
                  {
                    if (!UInt32.TryParse(otype[1].Replace("0x", ""), NumberStyles.HexNumber, null, out temp))
                    {
                      Console.WriteLine("ERROR: Invalid S-record address, {0}. Aborting...", otype[1]);
                      myCmdParams.valid = false;
                    }
                    else
                    {
                      myCmdParams.srecAddr = temp;
                    }
                  }
                  else if (UInt32.TryParse(otype[1], out temp))
                  {
                    myCmdParams.srecAddr = temp;
                  }
                  else
                  {
                    Console.WriteLine("ERROR: Invalid S-record address, {0}. Aborting...", otype[1]);
                    myCmdParams.valid = false;
                  }
                  myCmdParams.convType = ConvType.Exec2Srec;
                  defaultExtension = ".srec";
                }
                else
                {
                  myCmdParams.valid = false;
                }
              }
              else
              {
                myCmdParams.valid = false;
              }
              argsHandled[i + 1] = true;
              numHandledArgs++;
              break;
            }
            case "ini":
            {
              myCmdParams.iniFileName = args[i + 1];
              argsHandled[i + 1] = true;
              numHandledArgs++;
              break;
            }
            case "o":
            {
              myCmdParams.outFileName = args[i + 1];
              argsHandled[i + 1] = true;
              numHandledArgs++;
              break;
            }
            default:
            {
              myCmdParams.valid = false;
              break;
            }
          }
          argsHandled[i] = true;
          numHandledArgs++;
          
          // If we've seen any error, bug out
          if (!myCmdParams.valid)
          {
            break;
          }
        }
      }
      numUnhandledArgs = args.Length - numHandledArgs;
      
      // Check to make sure we are still valid
      if (!myCmdParams.valid)
      {
        return myCmdParams;
      }
       
      // Get optional input files in order (all unhandled args, if any, are input files)
      if (numUnhandledArgs != 0)
      {
        myCmdParams.inputFileName = new List<String>(numUnhandledArgs);
      }
      for (int i=numUnhandledArgs; i>0; i--)
      {
        String[] file = args[args.Length-i].Split('@');
        myCmdParams.inputFileName.Add(args[args.Length-i]);
      }
      
      // Set output filename to match input filename, if needed
      if (myCmdParams.outFileName == null)
      {
        if  (myCmdParams.inputFileName != null)
        {
          String lastFileName = ((String) myCmdParams.inputFileName[myCmdParams.inputFileName.Count - 1]).Split('@')[0];
          myCmdParams.outFileName = Path.GetFileNameWithoutExtension(lastFileName) + defaultExtension;
        }
        else
        {
          myCmdParams.outFileName = "ais_output" + defaultExtension;
        }
      }
  
      return myCmdParams;
    }
      
    /// <summary>
    /// Main program.
    /// </summary>
    /// <param name="args">Input commandline arguments</param>
    /// <returns>Return code</returns>
    static Int32 Main(String[] args)
    {          
      IniFile myIniFile;
    
      // From Common/AIS/HexAIS_version.cs
      System.Version v = GetVersion();
      
      // From Common/AIS/HexAIS_version.cs
      Int32 buildYear = GetBuildYear();
      
      // Begin main code
      Console.WriteLine("-----------------------------------------------------");
      Console.WriteLine("   TI AIS Hex File Generator for " + devString   );
      Console.WriteLine("   (C) "+buildYear+", Texas Instruments, Inc."        );
      Console.WriteLine("   Ver. "+v.Major+"."+v.Minor.ToString("D2")          );
      Console.WriteLine("-----------------------------------------------------");
      Console.Write("\n\n");            
      

      // Parse the input command line parameters
      ProgramCmdParams cmdParams = ParseCmdLine(args);
      if (!cmdParams.valid)
      {
        DispHelp();
        return -1;
      }
      
      // Now proceed with main program
      FileStream tempAIS_fs = null;
      Byte[] AISData, convertedData;
        
      AISGen_OMAP_L138 generator = new AISGen_OMAP_L138();
       
      // Update the default INI file name to the one supplied on the command line
      if (cmdParams.iniFileName == null)
      {
        Console.WriteLine("No ini file provided. Using default, {0}", generator.DeviceNameShort + ".ini");
        cmdParams.iniFileName = generator.DeviceNameShort + ".ini";
      }
      
      // Read the INI data from file
      if (File.Exists(cmdParams.iniFileName))
      {
        myIniFile = new IniFile(new FileStream(cmdParams.iniFileName, FileMode.Open, FileAccess.Read), cmdParams.iniFileName);
      }
      else
      {
        Console.WriteLine("File {0} not found.",cmdParams.iniFileName);
        return -1;
      }
      
      // Put entryPoint in General Ini section (may be overridden by INI InputFile sections)
      myIniFile.InsertValue("General","EntryPoint","0x"+cmdParams.entryPoint.ToString("X8"));
      
      // Force section-by-section CRC checks (may be overridden in INI file)
      generator.AISCRCType = AisCRCCheckType.SECTION_CRC;

      // Do the AIS generation
      try
      {
        AISData = AISGen.GenAIS(generator, cmdParams.inputFileName, myIniFile);
      }
      catch (Exception e)
      {
        System.Diagnostics.StackTrace trace = new System.Diagnostics.StackTrace(e, true);
      
        Console.WriteLine(e.StackTrace);
        Console.WriteLine(e.Message);
        Console.WriteLine("Unhandled Exception!!! Application will now exit.");
        return -1;
      }
      
      // Check if SecureAISGen completed successfully
      if (AISData == null)
      {
        Console.WriteLine("AIS generation failed.");
        return -1;
      }
      
      using (tempAIS_fs = new FileStream(cmdParams.outFileName, FileMode.Create, FileAccess.Write))
      {
      
        // Convert the AIS data to the correct output format
        switch ( cmdParams.convType )
        {
          case ConvType.Exec2Bin:
            tempAIS_fs.Write(AISData, 0, (int)AISData.Length);
            Console.WriteLine("Wrote {0} bytes to file {1}.",AISData.Length,cmdParams.outFileName);
            break;
          case ConvType.Exec2CArray:
            convertedData = CArray.bin2CArray(cmdParams.cArrayName, AISData, 4);
            tempAIS_fs.Write(convertedData, 0, (int)convertedData.Length);
            Console.WriteLine("Wrote {0} bytes to file {1}.",convertedData.Length,cmdParams.outFileName);
            break;
          case ConvType.Exec2Srec:
            convertedData = SRecord.bin2srec(AISData, (UInt32)cmdParams.srecAddr, 32);
            tempAIS_fs.Write(convertedData, 0, (int)convertedData.Length);
            Console.WriteLine("Wrote {0} bytes to file {1}.",convertedData.Length,cmdParams.outFileName);
            break;
          case ConvType.Exec2Text:
            Console.WriteLine("Mode Not supported.");
            break;            
        }
      }
              
      Console.WriteLine("Conversion is complete.");
      return 0;
    }
  }
}
