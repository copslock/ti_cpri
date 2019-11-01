/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//
//File Name: load_cores.js
//Description:
//   Launch firmware on different cores.

// Import the DSS packages into our namespace to save on typing
importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)
importPackage(java.io);
importPackage(java.lang);

var debugServer;
var script;

//PDK path. Edit this
pdkPath = "C:/work/Repo/pdk/";
pdkbinPath = pdkPath+"packages/ti/binary/ipc_echo_test/bin/j721e_evm/"

var binFiles = [
    'ipc_echo_test_mpu1_0_debug.xa72fg',
    'ipc_echo_test_c7x_debug.xe71',
    'ipc_echo_test_mcu2_0_debug.xer5f',
    'ipc_echo_test_mcu2_1_debug.xer5f',
    'ipc_echo_test_mcu3_0_debug.xer5f',
    'ipc_echo_test_mcu3_1_debug.xer5f',
    'ipc_echo_test_c66xdsp_1_debug.xe66',
    'ipc_echo_test_c66xdsp_2_debug.xe66',
    'ipc_echo_test_mcu1_0_debug.xer5f'
//    'ipc_echo_test_mcu1_1_debug.xer5f',
];

var debugServerNames = [
    '.*CortexA72_0_0',
    '.*C71X_0',
    '.*MAIN_Cortex_R5_0_0',
    '.*MAIN_Cortex_R5_0_1',
    '.*MAIN_Cortex_R5_1_0',
    '.*MAIN_Cortex_R5_1_1',
    '.*C66xx_0',
    '.*C66xx_1',
    '.*MCU_Cortex_R5_0'
    //'.*MCU_Cortex_R5_1'
];

function loadBinary(filePath, dbgSrvrName, index)
{
	dbgSession = debugServer.openSession( dbgSrvrName );
	
	// Connect the MCU R5F
    dbgSession.target.connect();
    
    // Reset the MCU
	if(index > 7)
	{
		dbgSession.target.reset();
		//sysResetVar=dbgSession.target.getResetType(1);
		//sysResetVar.issueReset();
	}    
    
    // Load the binary file.
    dbgSession.memory.loadProgram(filePath);
    
    // Run the target now.
    dbgSession.target.runAsynch();
}

function loadBinaries()
{
	binFiles.forEach(function(item, index, array) 
	{
		var binFilePath=pdkbinPath+item;
		print("Loading binary file "+binFilePath);
		
		var dbgSrvrName = debugServerNames[index];
		print("Debug Server Name - "+dbgSrvrName);
		
		loadBinary(binFilePath, dbgSrvrName, index);
	});
}

function setEnvVariable()
{
	// Check to see if running from within CCSv4 Scripting Console
	var withinCCS = (ds !== undefined);
	
	// Create scripting environment and get debug server if running standalone
	if (!withinCCS)
	{
		// Import the DSS packages into our namespace to save on typing
		importPackage(Packages.com.ti.debug.engine.scripting);
		importPackage(Packages.com.ti.ccstudio.scripting.environment);
		importPackage(Packages.java.lang);

		// Create our scripting environment object - which is the main entry point into any script and
		// the factory for creating other Scriptable ervers and Sessions
		script = ScriptingEnvironment.instance();

		// Get the Debug Server and start a Debug Session
		debugServer = script.getServer("DebugServer.1");
	}
	else // otherwise leverage existing scripting environment and debug server
	{
		debugServer = ds;
		script = env;
	}	
}

function doEverything()
{
	setEnvVariable();
	
	loadBinaries();
}



doEverything();

