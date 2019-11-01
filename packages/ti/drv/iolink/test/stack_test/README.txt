This test application requires the IQ2 master stack which is not free, please contact IQ2 Devemlopment to get a license
and the source code.

To build the IO-Link IQ2 master stack application:

1. Get a copy of the IQ2 stack source
2. export IOLINK_STACK_INSTALL_PATH=<path where IQ2 stack source is installed>
3. Under $(PDK_INSTALL_PATH)/ti/driver/iolink, run "make apps LIMIT_SOCS=am437x LIMIT_BOARDS=idkAM437x"
