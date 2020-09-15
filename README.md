# RaceBoy-G35
Arduino code that sniffs the CAN messages sent throughout a cars bus. The CAN id's listed in here are specific to the 03 Infiniti G35 (YMMV with other model years)

More to come....

### Prerequisites

This has been built with a MEGA. This has not been tested on an UNO.
The MEGA's faster processor speeds are prefered due to the heavy "poor constructed" logic in the main loop.


### Installing

Make sure you use the libraries included. 
It should build correctly.

Look to object initializations and preprocessor macros for pin connections to screen/button/etc.



## Customizing

If you can work out the logic to get value loaded into a varible in the arduino, you can easily extend that to be a logger object. This is useful if wanting to change the code to use different CAN id or if you are implementing the ELM327

Look at the updateLoggers() function. That contains the data for reading from canbus buffers. 

```
CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
			rxId = CAN0.getCanId();                    // Get message ID
      
			if (rxId == 0x792 && !recvA)
			{//Brake Light and individual wheel speeds
				BRK = ((rxBuf[2]*100)/255);
				recvA = true;
			}
```

For items that dont need to be sniffed/trapped they can be specified in the main loop. Look for the two switch statements (one for sd logging on and one for just LCD display only). 

Make sure to update both the LCD and Logger switch statements otherwise you might not log the data to sd care

```
Logger Code:
case 0:
					// Code for TPS
					lgLcdList.set(loggerIndex[i], formatString(loggerTexts[i], TPS, 0));
					SDLog.print(TPS);
          //Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						SDLog.println();
					}
					else
					{
						SDLog.print(",");
					}
					break;
          
Plain Ol' LCD Code:

switch (i) {
				case 0:
					// Code for TPS
					
					lgLcdList.set(loggerIndex[i], formatString(loggerTexts[i], TPS, 0));
					break;
```


## Built With

* [Ardunio Mega](https://store.arduino.cc/usa/arduino-mega-2560-rev3/) - The core microprocessor for the project
* [SeeedStudio CAN Sheild](https://www.seeedstudio.com/CAN-BUS-Shield-V1.2-p-2256.html) - CAN Interface
* [NewHaven 4x20 OLED](http://www.newhavendisplay.com/nhd0420dzway5-p-4218.html) - Prefered to the Standard LCD displays; them black levels are out of this whorl.


## Authors

* **John Daley** - *Initial work* 

See also the list of [contributors](https://github.com/john159753/RaceBoy-G35/contributors) who participated in this project.

## Acknowledgments

* Thanks to coryjfowler for his great MCPCan Library! [Source](https://github.com/coryjfowler/MCP_CAN_lib)
* Thank you to Rupert Hirst for his work on correcting the OLED Library! [BlogPost](http://runawaybrainz.blogspot.com/2014/01/midas-2004-oled-character-display.html)
* Thank you to Brad Wentz and all his work on decoding the Nissan 370 CAN Bus Messages! Although not apples->apples with a 2003 G35. His work helped add alot of understanding of the Nissan CAN Bus system. [Blog](https://projectbytes.wordpress.com/2013/08/25/nissan-370z-can-hacking/)
