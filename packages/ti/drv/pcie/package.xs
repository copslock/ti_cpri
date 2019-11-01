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
    var suffix = prog.build.target.suffix;
    var name = "";
    var socType = this.Settings.socType;
    var devType = this.Settings.deviceType;
    var profilingTag = "";
    if ((! socType) && devType)
    {
        /* Backward compatibilty with keystone .cfg */
        socType = devType;
    }

    socType = socType.toLowerCase();
    /* Replace the last charecter in SoC am#### to am###x */
    if (socType.substring(0, 2) == "am")
    {
        socType = socType.substring(0, socType.length - 1);
        socType = socType.concat("x");
    }

    if (this.Settings.enableProfiling == true)
    {
        profilingTag = ".profiling"
    }
    name = this.$name + profilingTag + ".a" + suffix;
    
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
    var socTypes = [
                       'am65xx',
                       'k2k',
                       'k2h',
                       'k2l',
                       'k2e',
                       'k2g',
                       'c6657',
                       'c6678',
                       'am571x',
                       'am574x',
                       'am572x'
                   ];

    /* Search for the supported socs (defined in config.bld) */
    for each(var soc in socTypes)
    {
        if (socType.equals(soc))
        {
            lib = lib + "/" + soc;
            name = this.$name + profilingTag + ".a" + suffix;	 
            break;
        }
    }

    /* Get target folder, if applicable */
    if ( java.lang.String(suffix).contains('66') )
        lib = lib + "/c66";
    else if (java.lang.String(suffix).contains('a15') )
        lib = lib + "/a15";
    else if (java.lang.String(suffix).contains('a53') )
        lib = lib + "/a53";
    else if (java.lang.String(suffix).contains('m4') )
        lib = lib + "/m4";
    else if (java.lang.String(suffix).contains('r5') )
        lib = lib + "/r5f";
    else
        throw new Error("\tUnknown target for: " + this.packageBase + lib);

    var libProfiles = ["debug", "release"];
    /* get the configured library profile */
    for each(var profile in libProfiles)
    {
        if (this.Settings.libProfile.equals(profile))
        {
            lib = lib + "/" + profile;
            break;
        }
    }	

    /* Get library name with path */
    lib = lib + "/" + name;
    if (java.io.File(this.packageBase + lib).exists()) {
       print ("\tLinking with library " + this.$name + ":" + lib);
       return lib;
    }

    /* Could not find any library, throw exception */
    throw new Error("\tLibrary not found: " + this.packageBase + lib);
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
