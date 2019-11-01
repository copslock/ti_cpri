To run this benchmarking project using the loadti that is included with CCS you must first copy these
enhanced files (main.js and getArgs.js) to the following directories:

 For CCSv5 copy .js files to the following directory: (please replace the x.x.xxxx with your CCSv5 version)
   "C:\Program Files\Texas Instruments\ccsv5\ccs_base_5.x.x.xxxx\scripting\examples\loadti"

 For CCSv4 copy .js files to the following directory:
   "C:\Program Files\Texas Instruments\ccsv4\scripting\examples\loadti" directory.

Please be sure to make a copy of the original files in the directories shown above. This will allow you to
go back to the original files just in case you have problems with these new files.

If your CCS install directory is different, just copy the files to the "scripting\examples\loadti" directory
of your CCS install location.

The new loadti main.js and getArgs.js files add the ability to load and run the same .out file on multiple cores.
These files are written so they will work as they did originally, or if the "-ctr {number_of_cores_to_test}" is
specified will allow loading and running on multiple cores. In other words, the new files should work with your
previous calls to loadti as well as with the calls used for multicore loading and running. These .js files only
enable the user to load and run the same .out file on multiple cores.
