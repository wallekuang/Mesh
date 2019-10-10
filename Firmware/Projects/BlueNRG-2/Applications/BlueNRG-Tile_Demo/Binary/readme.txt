In these folders, compiled images of the firmware are provided, in order to allow the user to test the application in a quick and easy way.

- Install ST-LINK drivers: http://www.st.com/web/en/catalog/tools/FM146/CL1984/SC720/SS1450/PF251168   


The binary is provided in .bin format and can be flashed into the SoC/micro-controller flash memory using one of the procedures described below. 

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

Description of Binary file :
---------------------------
Flashed "STEVAL-BCN002V1.bin" file in STEVAL-BCN002V1 (BlueNRG-Tile) board.
Firmware supports following features :
 1. Sensor Model
 2. Lighting HSL model
 3. Single tap / double tap
 4. Free fall detection


						   
