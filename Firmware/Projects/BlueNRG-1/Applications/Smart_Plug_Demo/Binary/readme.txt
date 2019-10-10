In these folders, compiled images of the firmware are provided, in order to allow the user to test the application in a quick and easy way.

- Install ST-LINK drivers: http://www.st.com/web/en/catalog/tools/FM146/CL1984/SC720/SS1450/PF251168   


The binaries are provided in .bin format and can be flashed into the SoC/micro-controller flash memory using one of the procedures described below. 

- Procedure 1 (.bin only) -

 1 - Plug the Eval Board to the host PC using a micro-USB cable. The board will be recognized as an external memory device called "BlUENRG-1".
 2 - Drag and drop or copy the binary file into the "BlUENRG-1" device you see in Computer.
 3 - Wait until flashing is complete.

- Procedure 2 (.hex and .bin) -

 1 - Install BlueNRG-1 ST-Link Utility http://www.st.com/en/embedded-software/stsw-bnrg1stlink.html
 2 - Plug the Eval Board to the host PC using a micro USB cable.
 3 - Open the BlueNRG-1 ST-Link utility.
 4 - Connect to the board selecting "Target -> Connect" from the menu or pressing the corresponding button.
 5 - Open the binary file selecting "File -> Open File..." and choose the one you want to flash.
 6 - From the menu choose: "Target -> Program"
 7 - Click Start.
 8 - Wait until flashing is complete.

Description of Binary files
---------------------------
1. BlueNRG1_Lighting_Demo: This binary uses static random address of the controller to generate
                            MAC address, Other options to generate MAC address are as below
				 
                           
2.BlueNRG1_Lighting_Demo_Ext_MAC : Use this binary to use configure the MAC addresses externally. 
                                   		
Note: When using external MAC address, it is recommended to firstly program the BlueNRG1_Lighting_Demo_Ext_MAC.bin file and program the MAC addresses from Utilities\BlueNRG1_MAC later

3.BlueNRG1_Lighting_Demo_UniqueSno: This binary uses internal unique serial number of the controller to generate  MAC address. 

4. Different Binary files can be generated on the basis of their features like friendship,low-power,proxy and relay by modifying mesh_cfg.h file  
 
Note: a. Uncomment EXTERNAL_MAC_ADDR_MGMT in mesh_cfg.h file for external MAC address.
                               Some examples of MAC address are available from Utilities\BlueNRG1_MAC 
                            b. Uncomment INTERNAL_UNIQUE_NUMBER_MAC in mesh_cfg.h file for Internal Unique Number.