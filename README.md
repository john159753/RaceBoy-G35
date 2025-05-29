# RaceBoy-G35

Arduino-based racing telemetry system that interfaces with a vehicle's CAN bus to provide real-time data logging, lap timing, and visual feedback. Originally designed for the 2003 Infiniti G35, this system has evolved into a comprehensive racing data acquisition platform.

## Features

- **Real-time CAN Bus Data Logging**: Monitors throttle position, brake pressure, RPM, speed, steering angle, engine temperature, and more
- **GPS Lap Timing**: Automatic lap detection with zone-based timing using configurable track layouts
- **Visual RPM Indicator**: LED bar display with progressive lighting and redline warning
- **Tire Temperature Monitoring**: Integration with external Arduino via serial communication
- **Data Export**: Logs all telemetry data to CSV files with GPS coordinates and lap timing
- **External Display Support**: Sends formatted data to external devices for dashboard display
- **Smart Power Management**: Automatic screen dimming and external device sleep control
- **Interactive Menu System**: OLED-based menu for logger configuration and track selection

## System Architecture

This is the core telemetry unit. The complete racing system consists of:

- **RaceBoy-G35** (this project): Main telemetry unit with CAN interface, GPS, and data logging
- **TireTempProxy** (separate project): Dedicated Arduino for tire temperature sensors (needed due to I2C distance limitations in automotive environment)
- **Racing Dashboard** (separate project): Raspberry Pi-based real-time dashboard display
- **Track Builder** (separate project): Desktop application for creating track zone layouts

## Prerequisites

**Hardware Requirements:**
- Arduino MEGA 2560 (required for memory and processing power)
- MCP2515 CAN Bus shield/module
- 20x4 Character OLED display
- GPS module (NMEA compatible)
- Adafruit TLC5947 LED driver (for RPM bar)
- 3-axis accelerometer (for G-force measurement)
- SD card module
- 4 tactile buttons (navigation)
- Optional: TireTempProxy Arduino system for tire temperature monitoring

**Software Requirements:**
- Arduino IDE
- SdFat library v2+ (supports FAT16/32/exFAT)
- NMEAGPS library
- Adafruit TLC5947 library
- MCP_CAN library
- Custom libraries included in project

## Installation

1. Install required libraries through Arduino Library Manager or download from their respective repositories
2. Use the included custom libraries (MenuMaster, Stopwatch, etc.)
3. Configure pin assignments in the code header (see preprocessor definitions)
4. Create a "Tracks" folder on your SD card with track layout files (use Track Builder application)
5. Upload to Arduino MEGA

**Pin Configuration:**
- Refer to object initializations and #define statements in the code for specific pin assignments
- CAN module typically uses SPI pins + pin 9 for CS
- GPS on Serial1, external display on Serial2, tire temps on Serial3

## Track Setup

### Using GPSTrapCreator (Recommended)
https://github.com/john159753/GPSTrapCreator
Use the GPSTrapCreator to visually create track zones by clicking waypoints on a map. This generates the properly formatted CSV files automatically.

### Manual Track File Format
If creating tracks manually, place CSV files in the SD card's "/Tracks" folder:
```
ZoneName,Lat1,Lat2,Lat3,Lat4,Lon1,Lon2,Lon3,Lon4,TotalZones
Start/Finish,123456789,123456789,123456789,123456789,-123456789,-123456789,-123456789,-123456789,4
Turn1,123456789,123456789,123456789,123456789,-123456789,-123456789,-123456789,-123456789,4
```

Each zone is defined by a 4-point polygon using GPS coordinates (multiplied by 10^7 for precision).

## System Integration

### TireTempProxy Integration
The main unit communicates with the TireTempProxy Arduino via Serial3. The proxy handles:
- Multiple tire temperature sensors via I2C
- Data formatting and transmission to main unit
- Isolated power management for sensor reliability

**Data Format:** `<FL_temp,FR_temp,RL_temp,RR_temp>`

### Dashboard Integration  
Real-time data is sent via Serial2 to external devices (typically Raspberry Pi) running the dashboard application:

**Command Formats:**
- `1,PDL,BRK,RPM,SPD` - Primary sensor data
- `2,ECT,FuelLevel,SplitTime,LapCount` - Secondary data and lap info
- `3,FL_temp,FR_temp,RL_temp,RR_temp` - Tire temperatures
- `4,command,,,` - Timer control (start/stop/reset)
- `5,,left,right,` - Turn signal status

## Customizing

### Adding New CAN Messages

1. **Update CAN Processing**: Add new message IDs to `processCanMessage()` function
2. **Add Data Structure**: Create new `CanData` structure for your parameter
3. **Update Global Variables**: Add corresponding global variable and update logic in `updateGlobalCanVariables()`

Example:
```cpp
case 0xXXX: // Your CAN ID
    if (len >= required_bytes) {
        canYourParam.value = process_your_data(rxBuf);
        canYourParam.timestamp = timestamp;
        canYourParam.valid = true;
    }
    break;
```

### Adding New Logger Parameters

1. **Update Logger Arrays**: Add entry to `loggerTexts[]` array
2. **Update Menu System**: Modify MenuMaster class to include new parameter
3. **Add Switch Cases**: Update both display and SD logging switch statements in main loop
4. **Update EEPROM**: Ensure array sizes accommodate new parameters

### CAN ID Reference (2003 G35)

- `0x23D`: TPS/RPM data (high priority)
- `0x792`: Brake pressure (high priority)
- `0x2D1`: Speed/Fuel level  
- `0x002`: Steering angle
- `0x551`: Engine coolant temperature
- `0x625`: Vehicle status/lighting
- `0x60D`: Turn signals

## Hardware Notes

- **MEGA Required**: The extensive feature set requires the MEGA's additional memory and processing power
- **SD Card**: Use Class 10 or better for reliable high-speed data logging
- **GPS**: 10Hz update rate recommended for accurate lap timing
- **Tire Temp Sensors**: Use separate TireTempProxy project due to I2C distance limitations in automotive installations
- **Power**: Consider automotive power filtering for stable operation

## Built With

* [Arduino Mega 2560](https://store.arduino.cc/usa/arduino-mega-2560-rev3/) - Main microcontroller
* [MCP2515 CAN Module](https://www.seeedstudio.com/CAN-BUS-Shield-V1.2-p-2256.html) - CAN Bus interface
* [20x4 OLED Display](http://www.newhavendisplay.com/nhd0420dzway5-p-4218.html) - Primary display
* [Adafruit TLC5947](https://www.adafruit.com/product/1429) - LED driver for RPM bar
* GPS Module (NMEA compatible) - Lap timing and position logging

## Related Projects

- **TireTempProxy**: Dedicated Arduino system for tire temperature sensor management
- **Racing Dashboard**: Raspberry Pi-based real-time telemetry dashboard
- **GPSTrapCreator**: Desktop application for creating GPS-based track zone layouts

## Authors

* **John Daley** - *Initial work and ongoing development*


## Acknowledgments

* **coryjfowler** - Excellent MCP_CAN library [Source](https://github.com/coryjfowler/MCP_CAN_lib)
* **Rupert Hirst** - OLED library corrections [BlogPost](http://runawaybrainz.blogspot.com/2014/01/midas-2004-oled-character-display.html)
* **Brad Wentz** - Nissan CAN bus research and documentation [Blog](https://projectbytes.wordpress.com/2013/08/25/nissan-370z-can-hacking/)
* **SlashDevin** - NMEAGPS library for reliable GPS parsing
* **Bill Greiman** - SdFat library for robust SD card operations