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
    var suffix;

    /* find a compatible suffix */
    if ("findSuffix" in prog.build.target) {
        suffix = prog.build.target.findSuffix(this);
    }
    else {
        suffix = prog.build.target.suffix;
    }

    var name = this.$name + ".a" + suffix;
    var lib = "";

    lib = "lib/" + name;
    if (java.io.File(this.packageBase + lib).exists()) {
        return lib;
    }

    /* could not find any library, throw exception */
    throw Error("Library not found: " + name);
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

