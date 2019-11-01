/*
 * CryptoLoadMod.cs
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
using System.Collections;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;

using TI.UtilLib;
using TI.UtilLib.IO;
using TI.UtilLib.Ini;
using TI.AISLib;

namespace TI.UtilLib.Crypto
{ 
  public enum SHA_Algorithm : int
  {
    SHA1 = 0x0,
    SHA256 = 0x1,
    SHA384 = 0x2,
    SHA512 = 0x3
  }
  
  public enum SecureLoadMagic : uint
  {
    // Load module magic ID
    LOADMOD_MAGIC   = 0x70ADC0DE,
    // Key certificate magic ID
    CERT_MAGIC      = 0x70ADCE87,
    // Generic user key magic ID
    GENKEY_MAGIC    = 0xBE40C0DE
  }
 
  public class LoadModule
  {
    public SecureLoadMagic MagicNum;
    
    public UInt32 bufSize;
    
    // Private security type variable
    public AisSecureType SecureType;
    
    // Encryption Key
    public Byte[] customerEncryptionKey;
    public Byte[] certEncryptionKey;
    public Byte[] CEKInitialValue;
    public Byte[] keyEncryptionKey;

    // RSA Object
    public RSACryptoServiceProvider rsaObject;
    public RSACryptoServiceProvider certRsaObject;
    
    // Variable for current selected Hash algorihtm
    public SHA_Algorithm currHashAlgorithmValue;
    public HashAlgorithm currHashAlgorithm;
    
    public Boolean encryptData;
    
    public LoadModule()
    {
      SecureType = AisSecureType.NONE;
      MagicNum = SecureLoadMagic.LOADMOD_MAGIC;
      certRsaObject = null;
      rsaObject = null;
      certEncryptionKey = null;
      customerEncryptionKey = null;
    }
    
    public LoadModule(AisSecureType secureType, SecureLoadMagic magic): this()
    {
      SecureType = secureType;
      MagicNum = magic;
    }
    
    public LoadModule(AisSecureType secureType, SecureLoadMagic magic, String encKey, String rsaKeyFileName) : this()
    {
      String currHashAlgorithmString = "SHA1";  // Default hash algorithm
    
      SecureType = secureType;
      MagicNum = magic;
    
      // Get the encryption key
      customerEncryptionKey = new Byte[16];
      CEKInitialValue = new Byte[16];
      
      if (encKey.Length != 32)
      {
        throw new ArgumentException("AES Encryption Key is wrong length!");
      }
      for (int j=0; j<encKey.Length; j+=2)
      {
        customerEncryptionKey[(j>>1)] = Convert.ToByte(encKey.Substring(j,2),16);
      }
      
      // Generate IV as encrypted version of AES Key
      using (MemoryStream ms = new MemoryStream(CEKInitialValue))
      {
        Aes myAES = new AesManaged();
        myAES.KeySize = 128;
        myAES.Mode = CipherMode.ECB;
        myAES.Padding = PaddingMode.None;
        ICryptoTransform encryptor = myAES.CreateEncryptor(customerEncryptionKey, new Byte[16]);
        CryptoStream cs = new CryptoStream(ms,encryptor,CryptoStreamMode.Write);
        cs.Write(customerEncryptionKey,0,customerEncryptionKey.Length);
      }
      
      // Get RSA key
      rsaObject = RSAKey.LoadFromFile(rsaKeyFileName);
      if (rsaObject == null)
      {
        throw new ArgumentException("RSA key loading failed!");
      }
      
      // Update the hash algo string if RSA key size is 2048 bits
      if (rsaObject.KeySize == 2048)
      {
        currHashAlgorithmString = "SHA256";
        currHashAlgorithmValue = SHA_Algorithm.SHA256;
      }
      
      try
      {
        currHashAlgorithm = HashAlgorithm.Create(currHashAlgorithmString);
      }
      catch (Exception e)
      {
        Console.WriteLine("Invalid Hash Algorithm Selected. Exception message: {0}.",e.Message);
        throw e;
      }
    }
    
    public LoadModule(IniFile iniFile) : this()
    {
      if (SecurityIniSectionParse(this, iniFile) != retType.SUCCESS)
      {
        throw new ArgumentException();
      }
    }
    
    public static Byte[] GenerateCertData(LoadModule loadMod)
    {
      Byte[] delegateKeyData = null;
     
      if (loadMod.SecureType == AisSecureType.CUSTOM)
      {
        // Create Delegate Key Certificate Format
        //Key cert: 32 + (128 or 256) bytes
        //  decryptionKey: 16 byte (128 bits) AES key

        //  rsaPublicKey: 8 bytes + 128/256 for modulus data (actual modulus data may only be 128 bytes for RSA1024)
        //    keyExponent: 4 bytes
        //    keyPad: 2 bytes
        //    modLength: 2 bytes
        //    modData: 128 or 256 bytes
        //  keyFlags: 8 bytes
      
        delegateKeyData = new Byte[32 + (loadMod.rsaObject.KeySize >> 3)];
      
        // Fill with random data
        (new Random()).NextBytes(delegateKeyData);
        
        // Insert decryptionKey at offset 0
        loadMod.certEncryptionKey.CopyTo(delegateKeyData, 0);
        
        // Insert rsaPublicKey at offset 16
        RSAKey.CreateCustomSecureKeyVerifyStruct(loadMod.certRsaObject).CopyTo(delegateKeyData, 16);
        
        // Insert flags data
        BitConverter.GetBytes((UInt32)0x00000000).CopyTo(delegateKeyData, 24 + (loadMod.rsaObject.KeySize >> 3) );
        BitConverter.GetBytes((UInt32)0x00000000).CopyTo(delegateKeyData, 28 + (loadMod.rsaObject.KeySize >> 3) );
        
      }
      else if (loadMod.SecureType == AisSecureType.GENERIC)
      {
        delegateKeyData = new Byte[32];
      
        // Fill with random data
        (new Random()).NextBytes(delegateKeyData);
        
        // Insert decryptionKey at offset 0
        loadMod.certEncryptionKey.CopyTo(delegateKeyData, 0);
        
        // Insert flags data
        BitConverter.GetBytes((UInt32)0x00000000).CopyTo(delegateKeyData, 16 );
        BitConverter.GetBytes((UInt32)0x00000000).CopyTo(delegateKeyData, 20 );
        BitConverter.GetBytes((UInt32)0x00000000).CopyTo(delegateKeyData, 24 );
        BitConverter.GetBytes((UInt32)0x00000000).CopyTo(delegateKeyData, 28 );        
      }
      
      return delegateKeyData;
    }
    
    public static Byte[] GenerateLoadModule(LoadModule loadMod, Byte[] loadData)
    {
      Byte[] outputData, sigData;
      Byte[] outputDataSigOrder, encOutputData;
      Int32 sigSize = 0;
      
      if (loadMod.SecureType == AisSecureType.CUSTOM)
      {
        sigSize = (loadMod.rsaObject.KeySize >> 3);
      }
      else if (loadMod.SecureType == AisSecureType.GENERIC)
      {
        sigSize = 32;
      }
      
      if ((loadData.Length % 16) != 0)
      {
        Int32 len = ((loadData.Length + 15) / 16) * 16;
        Byte[] temp = new Byte[len];
        loadData.CopyTo(temp,0);
        loadData = temp;
      }
      
      outputData = new Byte[loadData.Length + 16 + sigSize];
    
      // Fill with random data
      (new Random()).NextBytes(outputData);
      
      // Insert load module header data
      BitConverter.GetBytes((UInt32)loadMod.MagicNum).CopyTo(outputData, 0 );
      BitConverter.GetBytes((UInt32)outputData.Length).CopyTo(outputData, 4 );
      
      // FIXME - use random data in practice
      BitConverter.GetBytes((UInt32)0x00000000).CopyTo(outputData, 8 );
      BitConverter.GetBytes((UInt32)0x00000000).CopyTo(outputData, 12 );
      
      
      // Insert load module payload data at offset 16
      loadData.CopyTo(outputData, 16);
      
      // Create copy of load module in output/signature order
      outputDataSigOrder = new Byte[outputData.Length - sigSize];
      Array.Copy(outputData, 0 , outputDataSigOrder, loadData.Length, 16);
      loadData.CopyTo(outputDataSigOrder, 0);
      
      // Encrypt the data (load header first, then payload)
      if (loadMod.encryptData)
      {
        encOutputData = AesManagedUtil.AesCBCEncrypt(outputData, loadMod.customerEncryptionKey, loadMod.CEKInitialValue);
        Array.Copy(encOutputData, 0 , outputData, loadData.Length, 16);
        Array.Copy(encOutputData, 16, outputData,  0, loadData.Length);
      }
      else
      {
        outputDataSigOrder.CopyTo(outputData, 0);
      }
      
      // Generate and Insert Signature
      sigData = GenerateSecureSignature(loadMod, outputDataSigOrder);
      
      // Append signature to the end of output data
      sigData.CopyTo(outputData, (loadData.Length + 16));
      
      return outputData;
    }
    
    private static Byte[] GenerateSecureSignature(LoadModule loadMod, Byte[] dataForSigGen)
    {
      Byte[] signatureData = null;
      
      // Calculate hash of data
      Byte[] hash = loadMod.currHashAlgorithm.ComputeHash(dataForSigGen);
      
      // Generate signature via encryption
      if ( loadMod.SecureType == AisSecureType.GENERIC )
      {
        signatureData = new Byte[32];
               
        // Fill signature data buffer with random bytes
        (new Random()).NextBytes(signatureData);
        
        // Copy calculated SHA hash into signature data buffer
        hash.CopyTo(signatureData,0);

        using (MemoryStream ms = new MemoryStream())
        {
          Aes myAES = new AesManaged();
          myAES.KeySize = 128;
          myAES.Mode = CipherMode.CBC;
          myAES.Padding = PaddingMode.None;
          ICryptoTransform encryptor = myAES.CreateEncryptor(loadMod.customerEncryptionKey, loadMod.CEKInitialValue);
          CryptoStream cs = new CryptoStream(ms,encryptor,CryptoStreamMode.Write);
          cs.Write(signatureData,0,signatureData.Length);
          cs.FlushFinalBlock();
          ms.ToArray().CopyTo(signatureData,0);
        }
      }
      else if ( loadMod.SecureType == AisSecureType.CUSTOM )
      {

        RSAPKCS1SignatureFormatter rsaFormatter = new RSAPKCS1SignatureFormatter(loadMod.rsaObject);

        // Create a signature for HashValue and return it.
        signatureData = rsaFormatter.CreateSignature(loadMod.currHashAlgorithm);
        
        // Signature info needs to be revered to work with RSA functionality in ROM
        Array.Reverse(signatureData);
      }

      return signatureData;
    }
    
    private static retType SecurityIniSectionParse(LoadModule loadMod, IniFile iniFile)
    {
      String currHashAlgorithmString = "SHA1";  // Default hash algorithm
      
      // Get data from the GENERAL INI Section
      IniSection sec = iniFile.GetSectionByName("Security");
    
    #region INI Section parsing
      if (sec != null)
      {
        foreach (DictionaryEntry de in sec.sectionValues)
        {
          // Security Type
          if (((String)de.Key).Equals("SECURITYTYPE", StringComparison.OrdinalIgnoreCase))
          {
            loadMod.SecureType = (AisSecureType) Enum.Parse(typeof(AisSecureType), (String)sec.sectionValues["SECURITYTYPE"], true);
          }
           
          if (((String)de.Key).Equals("ENCRYPT", StringComparison.OrdinalIgnoreCase))
          {
            if (((String)sec.sectionValues["ENCRYPT"]).Equals("YES", StringComparison.OrdinalIgnoreCase))
              loadMod.encryptData = true;
            if (((String)sec.sectionValues["ENCRYPT"]).Equals("TRUE", StringComparison.OrdinalIgnoreCase))
              loadMod.encryptData = true;
          }
          
          // AES Encryption Key (CEK)
          if (((String)de.Key).Equals("MAGICNUM", StringComparison.OrdinalIgnoreCase))
          {
            loadMod.MagicNum = (SecureLoadMagic) Enum.Parse(typeof(SecureLoadMagic), (String)sec.sectionValues["MAGICNUM"], true);
          }

          // AES Encryption Key (CEK)
          if (((String)de.Key).Equals("ENCRYPTIONKEY", StringComparison.OrdinalIgnoreCase))
          {
            loadMod.customerEncryptionKey = new Byte[16];
            loadMod.CEKInitialValue = new Byte[16];
            
            String keyString = (String)sec.sectionValues["ENCRYPTIONKEY"];
            if (keyString.Length != 32)
            {
              Console.WriteLine("AES Encryption Key is wrong length!");
              return retType.FAIL;
            }
            for (int j=0; j<keyString.Length; j+=2)
            {
              loadMod.customerEncryptionKey[(j>>1)] = Convert.ToByte(keyString.Substring(j,2),16);
            }
            
            // Generate IV as encrypted version of AES Key
            using (MemoryStream ms = new MemoryStream(loadMod.CEKInitialValue))
            {
              Aes myAES = new AesManaged();
              myAES.KeySize = 128;
              myAES.Mode = CipherMode.ECB;
              myAES.Padding = PaddingMode.None;
              ICryptoTransform encryptor = myAES.CreateEncryptor(loadMod.customerEncryptionKey, new Byte[16]);
              CryptoStream cs = new CryptoStream(ms,encryptor,CryptoStreamMode.Write);
              cs.Write(loadMod.customerEncryptionKey,0,loadMod.customerEncryptionKey.Length);
            }  
          }
          
          // Key Encryption Key (not normally known, here for debug/testing purposes)
          if (((String)de.Key).Equals("KEYENCRYPTIONKEY", StringComparison.OrdinalIgnoreCase))
          {
            loadMod.keyEncryptionKey = new Byte[16];
            String keyString = (String)sec.sectionValues["KEYENCRYPTIONKEY"];
            if (keyString.Length != 32)
            {
              Console.WriteLine("Key Encryption Key is wrong length!");
              return retType.FAIL;
            }
            for (int j=0; j<keyString.Length; j+=2)
            {
              loadMod.keyEncryptionKey[(j>>1)] = Convert.ToByte(keyString.Substring(j,2),16);
            }
          }
          
          // Custom Secure RSA Key File
          if (((String)de.Key).Equals("RSAKEYFILENAME", StringComparison.OrdinalIgnoreCase))
          {
            String rsaKeyFileName = (String)sec.sectionValues["RSAKEYFILENAME"];
            loadMod.rsaObject = RSAKey.LoadFromFile(rsaKeyFileName);
            
            if (loadMod.rsaObject == null)
            {
              Console.WriteLine("RSA key loading failed!");
              return retType.FAIL;
            }
            
            // Update the hash algo string if RSA key size is 2048 bits
            if (loadMod.rsaObject.KeySize == 2048)
            {
              currHashAlgorithmString = "SHA256";
              loadMod.currHashAlgorithmValue = SHA_Algorithm.SHA256;
            }
          }
          
          // Custom Secure RSA Key File
          if (((String)de.Key).Equals("CERTRSAKEYFILENAME", StringComparison.OrdinalIgnoreCase))
          {
            String rsaKeyFileName = (String)sec.sectionValues["CERTRSAKEYFILENAME"];
            loadMod.certRsaObject = RSAKey.LoadFromFile(rsaKeyFileName);
            
            if (loadMod.certRsaObject == null)
            {
              Console.WriteLine("Certificate RSA key loading failed!");
              return retType.FAIL;
            }
          }
          
          // AES Encryption Key (CEK)
          if (((String)de.Key).Equals("CERTENCRYPTIONKEY", StringComparison.OrdinalIgnoreCase))
          {
            loadMod.certEncryptionKey = new Byte[16];
            
            String keyString = (String)sec.sectionValues["CERTENCRYPTIONKEY"];
            if (keyString.Length != 32)
            {
              Console.WriteLine("AES Encryption Key is wrong length!");
              return retType.FAIL;
            }
            for (int j=0; j<keyString.Length; j+=2)
            {
              loadMod.certEncryptionKey[(j>>1)] = Convert.ToByte(keyString.Substring(j,2),16);
            }
          }
          
          // Generic Secure Option to select the hash algorithm for signatures
          if (((String)de.Key).Equals("GENERICSHASELECTION", StringComparison.OrdinalIgnoreCase))
          {
            currHashAlgorithmString = (String)sec.sectionValues["GENERICSHASELECTION"];
            loadMod.currHashAlgorithmValue = (SHA_Algorithm) Enum.Parse(typeof(SHA_Algorithm), currHashAlgorithmString, true);
          }
        }
      }
    #endregion
      
    #region Security INI Input Validation
      // 2) Make sure a secure type has been specified
      if (loadMod.SecureType == AisSecureType.NONE)
      {
        Console.WriteLine("ERROR: The security type was not specified!");
        return retType.FAIL;
      }
      else
      {
        Console.WriteLine("Generating load modulefor a {0} secure device.",loadMod.SecureType.ToString().ToLower());
        // 3) Make sure we have a CEK and IV
        if (loadMod.customerEncryptionKey == null)
        {
          Console.WriteLine("ERROR: No encryption key was specified!");
          return retType.FAIL;
        }
        
        // 4) If custom secure, make sure we have an rsaObject
        if ((loadMod.SecureType == AisSecureType.CUSTOM) && (loadMod.rsaObject == null))
        {
          Console.WriteLine("ERROR: No RSA key file was specified!");
          return retType.FAIL;
        }
        
        // 5) Make sure RSA key size is supported
        if ((loadMod.SecureType == AisSecureType.CUSTOM) && (loadMod.rsaObject != null))
        {
          if ( (loadMod.rsaObject.KeySize != 1024) && (loadMod.rsaObject.KeySize != 2048) )
          {
            Console.WriteLine("ERROR: No RSA key size is invalid!");
            return retType.FAIL;
          }
          else
          {
            Console.WriteLine("INFO: RSA key is {0} bits.",loadMod.rsaObject.KeySize);
          }
        }
      }

      // 6) Make sure valid hash algorithm was selected      
      try
      {
        loadMod.currHashAlgorithm = HashAlgorithm.Create(currHashAlgorithmString);
        Console.WriteLine("INFO: Current SHA algorithm is {0}.",loadMod.currHashAlgorithmValue);
      }
      catch (Exception e)
      {
        Console.WriteLine("ERROR: Invalid Hash Algorithm Selected. Exception message: {0}.",e.Message);
        return retType.FAIL;
      }
      
      // 7) If custom CERT_MAGIC module, verify certRsaObject and certEncryptionKey are set
      if ((loadMod.MagicNum == SecureLoadMagic.CERT_MAGIC) && (loadMod.SecureType == AisSecureType.CUSTOM))
      {
        if (loadMod.certRsaObject == null)
        {
          Console.WriteLine("ERROR: Delegate Key Certificate load module for custom devices requires CERTRSAKEYFILENAME value.");
          return retType.FAIL;
        }
        if (loadMod.certEncryptionKey == null)
        {
          Console.WriteLine("ERROR:Delegate Key Certificate load module for custom devices requires CERTENCRYPTIONKEY value.");
          return retType.FAIL;
        }
        
        // 9) Make sure RSA key sizes match  
        if (loadMod.certRsaObject.KeySize != loadMod.rsaObject.KeySize)
        {
          Console.WriteLine("ERROR: Certificate RSA key size does not match current RSA key size.");
          return retType.FAIL;
        }
      }
      
      // 8) If custom CERT_MAGIC module, verify certRsaObject and certEncryptionKey are set
      if ((loadMod.MagicNum == SecureLoadMagic.CERT_MAGIC) && (loadMod.SecureType == AisSecureType.GENERIC))
      {
        if (loadMod.certEncryptionKey == null)
        {
          Console.WriteLine("ERROR: Delegate Key Certificate load module for generic devices requires CERTENCRYPTIONKEY value.");
          return retType.FAIL;
        }
      }
      
      if ((loadMod.MagicNum == SecureLoadMagic.CERT_MAGIC) && (!loadMod.encryptData))
      {
        Console.WriteLine("INFO: For CERT_MAGIC, encryption is required - forcing load module payload to be encrypted.");
        loadMod.encryptData = true;
      }
      
    #endregion
      return retType.SUCCESS;
    }
    
    
  }
}