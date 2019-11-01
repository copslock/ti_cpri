/*
 *  ======== package.xs ========
 *
 */

/*
 *  ======== Package.getLibs ========
 *  This function is called when a program's configuration files are
 *  being generated and it returns the name of a library appropriate
 *  for the program's configuration.
 */

function getLibs(prog)
{
    var nwalSettings = this.Settings;
    
    var suffix = prog.build.target.suffix;
    if (nwalSettings.useNwalSaLib == true)
    {
        var name = this.$name + ".sa" + ".a" + suffix; 
    }
    else
        var name = this.$name + ".a" + suffix; 
    
    /* Read LIBDIR variable */
    var lib = java.lang.System.getenv("LIBDIR");
    
    /* If NULL, default to "lib" folder */
    if (lib == null)
    {
        lib = "./lib";
    } else {
        print ("\tSystem environment LIBDIR variable defined : " + lib);
    }

    /* Device types supported */
    var deviceTypes = [
                        'k2l',
                        'k2e',
                        'k2h',
                        'k2k',
                      ];

    /* Search for the supported devices (defined in config.bld) */
    for each(var device in deviceTypes)
    {
        if (this.Settings.deviceType.equals(device))
        {
            lib = lib + "/" + device;
            break;
        }
    }

    /* Get target folder, if applicable */
    if ( java.lang.String(suffix).contains('66') )
        lib = lib + "/c66";



    /* Get library name with path */
    lib = lib + "/" + name;
    if (java.io.File(this.packageBase + lib).exists()) {
       print ("\tLinking with library " + this.$name + ":" + lib);
       return lib;
    }
    
    /* could not find any library, throw exception */
    throw Error("Library not found: " + lib);
}

/*
 *  ======== package.close ========
 */
function close()
{    
    if (xdc.om.$name != 'cfg') {
        return;
    }
}
