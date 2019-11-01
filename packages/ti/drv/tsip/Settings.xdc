

module Settings
{
    config string tsiplldVersionString = "01.00.00.06";
    /*! This variable is to control the device type selection.
     * By default this variable is set to NULL.
     * 
     * To use TSIP for the selected device, add the following lines to config
     * file and set the deviceType correctly:
     * 
     *      var tsip = xdc.useModule ('ti.drv.tsip.Settings');
     *      tsip.deviceType = "k2e";
     */
    metaonly config string deviceType = "k2e";
}

