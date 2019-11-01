call setupenv.bat

REM set SOC_FAMILY=KeyStone2
set XDCOPTIONS=-v

XDC clean		
IF "%1"=="release" GOTO release
:build
XDC
IF "%1"=="" GOTO done
:release
XDC release
:done
		