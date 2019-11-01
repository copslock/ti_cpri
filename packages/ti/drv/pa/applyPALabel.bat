
REM Clearcase Directory where LLD component is located.
set CLEARCASE_DIRECTORY=%1
IF NOT DEFINED CLEARCASE_DIRECTORY GOTO noparameter

REM Label to be applied
set LABEL=%2
IF NOT DEFINED LABEL GOTO noparameter

REM Create Label Type
echo Creating label type %LABEL%@%CLEARCASE_DIRECTORY%\gtcsl_projects
cleartool mklbtype -nc -global %LABEL%@%CLEARCASE_DIRECTORY%\gtcsl_projects

REM Apply label recursively
echo Applying label %LABEL% on %CLEARCASE_DIRECTORY%\gtcsl_ip\pa_lld
cleartool mklabel -replace -recurse %LABEL% %CLEARCASE_DIRECTORY%\gtcsl_ip\pa_lld

GOTO done

:noparameter
echo usage applyPALabel <drive:>  <LABEL> 

:done