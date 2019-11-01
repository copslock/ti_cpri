/* --------------------------------------------------------------------------
    FILE        : device_name.cs
    PROJECT     : TI Booting and Flashing Utilities
    AUTHOR      : Daniel Allred
    DESC        : Name adaptor
 ----------------------------------------------------------------------------- */

using System;
using System.Text;
 
namespace TIBootAndFlash
{
  partial class Program
  {
    public static String devString = "OMAP-L138";
    
    public static UInt32 externalRAMStart = 0xC0000000;
    
    public static String[] deviceTypes = { "OMAPL138", "OMAPL138_LCDK", "AM1808", "AM1810", "C6748", "C6746", "C6748_LCDK" };
    
    public static String[] flashTypes = { "SPI_MEM", "NAND" , "NOR"};
  }
}