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
    var lib = "";

    if (prog.build.target.suffix == "e66" || prog.build.target.suffix == "lib" )
    {
        if (this.profile.match(/debug/))
        {
            lib = "nimu/lib/debug/ti.transport.ndk.nimu.ae66";
        } else
        {
            lib = "nimu/lib/release/ti.transport.ndk.nimu.ae66";
        }
    }
    else if (prog.build.target.suffix == "e66e")
    {
        if (this.profile.match(/debug/))
        {
            lib = "nimu/lib/debug/ti.transport.ndk.nimu.ae66e";
        } else
        {
            lib = "nimu/lib/release/ti.transport.ndk.nimu.ae66e";
        }
    }

    if (java.io.File(this.packageBase + lib).exists()) {
        return lib;
    }

    /* could not find any library, throw exception */
    throw Error("Library not found: " + this.packageBase + lib);
}

/*
 *  ======== package.init ========
 */
function init() {
xdc.loadPackage("ti.osal");
xdc.loadPackage("ti.csl");
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

