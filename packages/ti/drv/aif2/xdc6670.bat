call aif2setupenv.bat

set SOC_FAMILY=KeyStone1
set XDCPATH=%CslDir6616%;%XDC_INSTALL_PATH%/packages;%PDK_INSTALL_PATH%/.;%PDK_INSTALL_PATH%/../../..;%C6X_GEN_INSTALL_PATH%/include;%EDMA3_LLD_INSTALL_PATH%/packages;
set XDCOPTIONS=-v

copy 	.\lib\c66\ti.drv.aif2.6614.ae66        .\ti.drv.aif2.6614.tmp.ae66
copy 	.\lib\c66\ti.drv.aif2.6614.ae66e       .\ti.drv.aif2.6614.tmp.ae66e		
copy 	.\lib\c66\ti.drv.aif2.K2.ae66        .\ti.drv.aif2.K2.tmp.ae66
copy 	.\lib\c66\ti.drv.aif2.K2.ae66e       .\ti.drv.aif2.K2.tmp.ae66e		

XDC clean		
IF "%1"=="release" GOTO release
:build
XDC
IF "%1"=="" GOTO done
:release
XDC release
:done

copy 	.\ti.drv.aif2.6614.tmp.ae66    .\lib\c66\ti.drv.aif2.6614.ae66
copy 	.\ti.drv.aif2.6614.tmp.ae66e   .\lib\c66\ti.drv.aif2.6614.ae66e		

copy 	.\ti.drv.aif2.K2.tmp.ae66    .\lib\c66\ti.drv.aif2.K2.ae66
copy 	.\ti.drv.aif2.K2.tmp.ae66e   .\lib\c66\ti.drv.aif2.K2.ae66e		
		
del 	.\ti.drv.aif2.6614.tmp.ae66
del 	.\ti.drv.aif2.6614.tmp.ae66e

del 	.\ti.drv.aif2.K2.tmp.ae66
del 	.\ti.drv.aif2.K2.tmp.ae66e