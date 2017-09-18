# RaceBoy-G35
Arduino code that sniffs the CAN messages sent throughout a cars bus. The CAN id's listed in here are specific to the 03 Infiniti G35 (YMMV with other model years)

More to come....

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

### Installing

A step by step series of examples that tell you have to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system


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
