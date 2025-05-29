/*
 Name:    RaceBoy-G35.ino
 Created: 1/31/2016 5:23:27 PM
 Updated: 5/20/2025
 Author:  John Daley

 Most block comments are due to lab testing and no canbus to watch data
 */

// include the library code:
#include <EEPROM.h>                 // For non-volatile storage of settings
#include <Adafruit_TLC5947.h>       // For controlling the LED RPM bar
#include <Adafruit_CharacterOLED.h> // For the OLED display
#include <avr/pgmspace.h>           // For storing strings in program memory (flash) to save RAM
#include <Button.h>                 // Custom library for handling button debouncing and long presses
#include <mcp_can.h>                // For MCP2515 CAN bus controller
#include <SPI.h>                    // For SPI communication (used by CAN and SD card)
#include <LinkedList.h>             // For dynamic list of loggers to display on LCD
#include "SdFat.h"
#include "sdios.h"
#include "MenuMaster.h" // Custom menu system class
#include "Stopwatch.h"  // Custom stopwatch class for timing laps and logging
// GPS Libs
#include <NMEAGPS.h> // For parsing NMEA GPS sentences
#include <GPSport.h> // For GPS serial port communication
#include <Wire.h>    // For I2C communication (might be used by OLED or other sensors)

#define DISABLE_FS_H_WARNING // Disable warning for type File not defined.
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

const uint8_t SD_CS_PIN = 4;
#define SPI_CLOCK SD_SCK_MHZ(50)

/* new defines for interrupt based approach*/
volatile bool canMessageAvailable = false;
unsigned long lastCanActivity = 0;
const unsigned long CAN_TIMEOUT_MS = 3000; // 3 seconds timeout for CAN silence detection

// Structure to hold latest CAN data with timestamps
struct CanData
{
    int value;
    unsigned long timestamp;
    bool valid;
};

// Data structure for each CAN parameter
CanData canTPS = {0, 0, false};
CanData canPDL = {0, 0, false};
CanData canBRK = {0, 0, false};
CanData canSPD = {0, 0, false};
CanData canRPM = {0, 0, false};
CanData canSAS = {0, 0, false};
CanData canECT = {0, 0, false};
CanData canFuelLevel = {0, 0, false};

void canISR()
{
    canMessageAvailable = true;
}

// Improved CAN initialization with proper filters/masks

// Create CANBUS reader object: Instance of the MCP_CAN class for CAN bus communication.
MCP_CAN CAN0(9);
// Declare CANBUS vars: Variables for storing incoming CAN message data.
long unsigned int rxId; // Variable for holding the message ID (PID) of the incoming CAN frame
unsigned char len = 0;  // Variable for showing the frame length in bytes
unsigned char rxBuf[8]; // Array for the incoming CAN frame buffer
byte retry;             // Retry counter (e.g., for SD card initialization)

void initCAN()
{
    // Initialize CAN bus controller
    if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    {
        Serial.println("MCP2515 Initialized Successfully!");
    }
    else
    {
        Serial.println("Error Initializing MCP2515...");
        return;
    }

    // RACING DATA LOGGING PRIORITY:
    // FASTEST CHANGING (need immediate response): 0x23D (TPS/RPM), 0x792 (brake)
    // MODERATE SPEED: 0x002 (steering), 0x2D1 (speed/fuel), 0x60D (turn signals)
    // SLOW CHANGING: 0x551 (coolant temp), 0x625 (vehicle status)

    // RXB0 - Priority buffer for FASTEST CHANGING racing parameters
    // These change rapidly during racing and need immediate logging
    CAN0.init_Mask(0, 0, 0x7FF0000); // Exact match mask for standard ID

    // Filters for RXB0 - Fastest changing racing data gets priority
    CAN0.init_Filt(0, 0, 0x23D0000); // TPS/RPM (0x23D) - FASTEST: throttle position critical for racing
    CAN0.init_Filt(1, 0, 0x7920000); // Brake pressure (0x792) - FASTEST: brake input critical for racing

    // RXB1 - Secondary buffer for moderate/slow changing parameters
    // Use a strategic mask to catch the remaining IDs we care about
    // Looking at remaining IDs: 0x002, 0x2D1, 0x60D, 0x551, 0x625

    // Try a mask that can catch multiple of our target IDs
    // Let's use a mask that allows our specific IDs through more efficiently
    // We'll use a partial mask that reduces unwanted traffic but allows our IDs
    CAN0.init_Mask(1, 0, 0x7000000); // Mask upper bits to reduce some unwanted traffic

    // Filters for RXB1 - Remaining racing parameters
    CAN0.init_Filt(2, 0, 0x0020000); // Steering angle (0x002) - moderate speed changes
    CAN0.init_Filt(3, 0, 0x2D10000); // Speed/Fuel (0x2D1) - speed changes moderately
    CAN0.init_Filt(4, 0, 0x60D0000); // Turn signals (0x60D) - moderate frequency
    CAN0.init_Filt(5, 0, 0x5510000); // Coolant temp (0x551) - slow changes

    // Note: Vehicle status (0x625) will need software filtering as we're out of filters
    // But it's the least critical for racing data logging

    CAN0.setMode(MCP_NORMAL);

    // Enable interrupts for both receive buffers
    pinMode(2, INPUT_PULLUP); // MCP2515 interrupt pin
    attachInterrupt(digitalPinToInterrupt(2), canISR, FALLING);
}
/*end interrupt based can*/

//------------------------------------------------------------------------------

// STUFF FOR SOFTRESET
void (*resetFunc)(void) = 0; // declare reset function @ address 0: A function pointer to force a software reset of the Arduino.
// MenuMaster Defines: Constants defining the types of menu items.
#define BLOCK -1   // Menu item is blocked/inaccessible (e.g., end of a list, non-existent path)
#define NOTUSED 0  // Placeholder, not actively used as a menu type in logic
#define UPDOWN 1   // Menu item allows cycling through options or navigating deeper/shallower
#define ONOFF 2    // Menu item is a toggle (on/off)
#define CALLBACK 3 // Menu item triggers a specific function call
#define LIST 4     // Menu item displays a scrollable list of options (e.g., track names)

// Define Button Debounce Time: Prevents multiple readings from a single button press.
#define DEBOUNCE_MS 20
// Define Long Press Duration: Time (in milliseconds) for a button to be considered "long pressed".
#define LONG_PRESS 1000
// LCD Refresh Rate: Intervals for updating the OLED display.
#define lcdInterval 130     // Interval at which to refresh LCD for logger screens (milliseconds)
#define loggerInterval 1    // Interval at which to refresh LCD (milliseconds) - seems unused directly for LCD refresh
#define lapSplashTime 12000 // Duration (in milliseconds) for the lap completed splash screen

// Calibrations and vars for Accelerometer (ADXL335 or similar)
#define zero_G_x 510 // Analog read value for 0G on X-axis
#define zero_G_y 507 // Analog read value for 0G on Y-axis
#define zero_G_z 520 // Analog read value for 0G on Z-axis
#define scale_x 18   // Scaling factor for X-axis G-force calculation
#define scale_y 19   // Scaling factor for Y-axis G-force calculation
#define scale_z 20   // Scaling factor for Z-axis G-force calculation

// Accel Pin Declare
#define xpin A10           // Analog pin for X-axis accelerometer input
#define ypin A11           // Analog pin for Y-axis accelerometer input
#define zpin A12           // Analog pin for Z-axis accelerometer input
#define screenPwmPin 7     // PWM pin for controlling screen brightness
#define screenDimLvl 100   // PWM value for dimmed screen brightness
#define arduinoSleepPin 12 // Digital pin to control power/sleep for an external Arduino board

// Define for GPS: NMEA sentences for configuring the GPS module.
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"                                      // Set GPS update rate to 10Hz
#define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F"                               // Set GPS fix control to 5Hz
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" // Output RMC sentences only

// Define for LED RPM Bar: Configuration for the Adafruit TLC5947 LED driver.
#define NUM_TLC5974 1 // Number of TLC5947 chips used
#define gate1 4500    // RPM threshold for first LED segment
#define gate2 5000    // RPM threshold for second LED segment
#define gate3 5300    // RPM threshold for third LED segment
#define gate4 5500    // RPM threshold for fourth LED segment
#define gate5 5700    // RPM threshold for fifth LED segment
#define gate6 6000    // RPM threshold for sixth LED segment (and blinking)
#define ledBarData 8  // Data pin for TLC5947
#define ledBarClock 5 // Clock pin for TLC5947
#define ledBarLatch 6 // Latch pin for TLC5947

// Used for setting pausing without halting thread: Generic timestamp for non-blocking delays.
unsigned long oldMillis = 0;

// initialize the OLED object: Adafruit Character OLED display instance.
Adafruit_CharacterOLED Oled(OLED_V2, 33, 32, 34, 35, 36, 37, 38); // Constructor with pin assignments
bool splashInterrupt = false;                                     // Flag to indicate if a splash screen (e.g., lap completed) is active

// initialize the Button Objects: Instances of the custom Button class for each physical button.
Button UpButton = Button(31, true, true, DEBOUNCE_MS);    // Up button, pin 30, pullup enabled, active low, debounce
Button DownButton = Button(30, true, true, DEBOUNCE_MS);  // Down button, pin 31
Button LeftButton = Button(28, true, true, DEBOUNCE_MS);  // Left button, pin 28
Button RightButton = Button(29, true, true, DEBOUNCE_MS); // Right button, pin 29

// Encoder Vars / Screen Scroller: Variables for managing logger screen scrolling.
bool loggerIndexChanged = false; // Flag to indicate if the logger display order has changed
int lastLoggerIndexPosition = 0; // Stores the previous scroll position
int newLoggerIndexPosition = 0;  // Stores the new desired scroll position
int loggerCount;                 // Number of active loggers, used for scrollbar drawing

// Init Menu Position Vars: Variables tracking the current position within the multi-level menu.
int menu1Index = 0; // Index for the top-level menu
int menu2Index = 0; // Index for the second-level menu
int menu3Index = 0; // Index for the third-level menu
int menuDepth = 0;  // Current menu depth (0 for top, -1 for logger screen)
// Init for LED RPM Bar: Variables for controlling the RPM LED bar animation.
byte ledBarCurrentGate = -1;       // Current RPM gate (0-6)
bool ledBarBlinkState = false;     // State for blinking (on/off)
unsigned long ledBarInterval = 25; // Blinking interval (milliseconds)
long ledBarPreviousMillis = 0;     // Last time the LED bar state was updated

// Adafruit_TLC5947 instance for the RPM bar.
Adafruit_TLC5947 rpmBar = Adafruit_TLC5947(NUM_TLC5974, ledBarClock, ledBarData, ledBarLatch);

// Tire temperature variables (Front Left, Front Right, Rear Left, Rear Right).
int flTemp = 0;
int frTemp = 0;
int rlTemp = 0;
int rrTemp = 0;

// Init the menu object: Instance of the MenuMaster class.
MenuMaster menu(menuDepth, menu1Index, menu2Index, menu3Index);
int *menuSize = menu.GetMenuSize(); // Pointer to an array holding the maximum size of each menu dimension.
// init linked list: For dynamic storage of logger strings to display on the OLED.
LinkedList<String> lgLcdList = LinkedList<String>();
// init accel values: Variables to store calculated G-force values.
float xAxisGValue = 449;
float yAxisGValue = 4410;
// Declare custom LCD Characters for Scrolling LCD: Custom characters for OLED display (arrows, pipe, log icon).
byte upArrow[8] = { // Custom character for an upward arrow
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b10001,
    0b00100,
    0b00100,
    0b00100};
byte dnArrow[8] = { // Custom character for a downward arrow
    0b00100,
    0b00100,
    0b00100,
    0b10001,
    0b10001,
    0b01010,
    0b01010,
    0b00100};
byte pipe[8] = { // Custom character for a vertical pipe (used in scrollbar)
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100};
byte logIcon[8] = { // Custom character for a logging icon
    0b11000,
    0b10100,
    0b10100,
    0b11000,
    0b00100,
    0b00100,
    0b00100,
    0b00111};

bool longpressUp = false; // Flag to track if the Up button was long pressed.

// Declare temp data holders: Variables for storing decoded sensor values from CAN bus.
int TPS = 441;            // Throttle Position Sensor
int PDL = 442;            // Pedal Position
int RPM = 443;            // Engine Revolutions Per Minute
int SPD = 444;            // Speed
int BRK = 445;            // Brake Pressure
int SAS = 446;            // Steering Angle Sensor
int ECT = 447;            // Engine Coolant Temperature
int FuelLevel = 448;      // Fuel Level
bool canSilent = false;   // Flag: true if no activity on CAN bus (for screen dimming/sleep)
bool screenAwake = true;  // Flag: true if the screen is currently awake
bool screenDim = false;   // Flag: true if the screen is currently dimmed
bool leftSignal = false;  // Flag: true if left turn signal is active
bool rightSignal = false; // Flag: true if right turn signal is active
// Flags to track if specific CAN messages have been received in the current loop iteration.
bool recvA = false;
bool recvB = false;
bool recvC = false;
bool recvD = false;
bool recvE = false;
bool recvF = false;
int tmpA;               // Temporary variable for CAN data processing
int tmpB;               // Temporary variable for CAN data processing
int loggerListSize = 0; // Stores the current size of the logger display list

// Define **IN SAME ORDER AS MENU** what should be displayed on logger screen - Must be 10 char EXACTLY
//  Array of strings for logger names, displayed on the logger screen.
const char *loggerTexts[10] = {"Throttle  ", "Gas Pedal ", "Brake     ", "Speed     ", "Engine RPM", "Turn Angle", "ECT       ", "Fuel      ", "Lateral-G ", "Long-G    "};
// Array to map logger index to its position in the display list (-1 means not active/displayed).
char loggerIndex[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
char menuModifier; // Used for scrolling the logger display.

// Inits for SDCard Logging: Variables for SD card operations.

#if SD_FAT_TYPE == 0
SdFat sd;       // SdFat file system object
File sdLog;     // SdFile object for the main log file
File trackFile; // SdFile object for track trap files
File tracksDir; // An SdFile object to represent the /tracks directory
#elif SD_FAT_TYPE == 1
SdFat32 sd;       // SdFat file system object
File32 sdLog;     // SdFile object for the main log file
File32 trackFile; // SdFile object for track trap files
File32 tracksDir; // An SdFile object to represent the /tracks directory
#elif SD_FAT_TYPE == 2
SdExFat sd;       // SdFat file system object
ExFile sdLog;     // SdFile object for the main log file
ExFile trackFile; // SdFile object for track trap files
ExFile tracksDir; // An SdFile object to represent the /tracks directory
#elif SD_FAT_TYPE == 3
SdFs sd;          // SdFat file system object
FsFile sdLog;     // SdFile object for the main log file
FsFile trackFile; // SdFile object for track trap files
FsFile tracksDir; // An SdFile object to represent the /tracks directory
#endif // SD_FAT_TYPE

bool loggerRunning;  // Flag: true if data logging to SD card is active
int file = 0;        // File counter for log file naming
char fileString[12]; // Buffer for log file name (e.g., "01.csv")

StopWatch logTime;    // Stopwatch for overall logging duration
StopWatch lapTime;    // Stopwatch for individual lap times
bool sdReady = false; // Flag: true if SD card is successfully initialized
// Maximum line length plus space for zero byte.
const size_t LINE_DIM = 100; // Max buffer size for reading lines from files
char line[LINE_DIM];         // Buffer for reading lines from track files
size_t n;                    // Number of bytes read by fgets

// GPSVariable Definitions: Variables for GPS data processing.
static NMEAGPS gps;      // NMEAGPS parser object
static gps_fix fix;      // Structure to hold parsed GPS fix data
bool newGpsData = false; // Flag: true if new GPS data is available

// LapTimer Variable Definitions: Variables for lap timing and track zone detection.
int zone = 0;                      // Current track zone (0 for start/finish line)
int maxZones = 0;                  // Total number of zones for the current track
int laps = 0;                      // Current lap count
unsigned long previousLapTime = 0; // Time of the previous completed lap
unsigned long LapTime = 0;         // Time of the current lap
long latitudeTraps[4];             // Array to store latitudes of the current zone's polygon vertices
long longitudeTraps[4];            // Array to store longitudes of the current zone's polygon vertices
String zoneName;                   // Name of the current track zone
bool trapsLoaded = false;          // Flag: true if track trap data has been loaded
bool onTrack = false;              // Flag: true if the vehicle is currently on the track
char splitTime[10] = "";           // Buffer for displaying lap time differences

void setup()
{
    Serial.begin(115200); // Initialize Serial communication for debugging output
    Serial2.begin(9600);  // Initialize Serial2 for communication with an external display/device
    Serial3.begin(9600);  // Initialize Serial3 for receiving tire temps from another Arduino

    // Setup SD Card
    // pinMode(SD_CS_PIN, OUTPUT); // Configure SD card Chip Select pin

    // Attempt to initialize the SD card.
    if (!sd.begin(SD_CS_PIN, SPI_CLOCK))
    {
        retry = 0;
        while (retry < 2) // Retry initialization up to 2 times
        {
            retry++;
            if (sd.begin(SD_CS_PIN, SPI_CLOCK))
            {
                Serial.println(F("Card Ready"));
                sdReady = true; // Set flag if successful
                break;
            }
            // this will get this on all iterations of loop that dont work
            sd.initErrorPrint(&Serial);
        }
        if (!sdReady) // If still not ready after retries
        {
            sdReady = false;
            sd.initErrorPrint(&Serial);
            Serial.println(F("Card Failure"));
        }
    }
    else // If SD card initialized successfully on first try
    {
        sdReady = true;
        Serial.println(F("Card Ready"));
    }

    initCAN();         // Initialize the CAN bus controller with filters and masks
    Oled.begin(20, 4); // Initialize the 20x4 character OLED display

    // Begin the RPM LED bar driver
    rpmBar.begin(); // Initialize the TLC5947 LED driver

    // Set pin for other arduino sleep
    pinMode(arduinoSleepPin, OUTPUT);    // Configure pin for controlling external Arduino's power
    digitalWrite(arduinoSleepPin, HIGH); // Turn on the external Arduino initially

    // Setup Screen Brightness PWM
    pinMode(screenPwmPin, OUTPUT);  // Configure pin for PWM control of screen brightness
    analogWrite(screenPwmPin, 255); // Set screen to full brightness initially

    // Setup Accel: Configure pins for accelerometer input.
    pinMode(xpin, INPUT);
    pinMode(ypin, INPUT);
    pinMode(zpin, INPUT);
    analogReference(EXTERNAL); // Look to ARef pin for analog reference voltage (for accelerometer)

    // create custom lcd objects: Create custom characters on the OLED display.
    Oled.createChar(0, upArrow);
    Oled.createChar(1, dnArrow);
    Oled.createChar(2, pipe);
    Oled.createChar(3, logIcon);
    delay(100); // Short delay after creating custom characters

    // Read from EEPROM and set vars up
    ReadFromEEPROM(); // Load saved logger settings and positions from EEPROM
    // Turn LCD to full Bright
    Oled.command(0x17); // Command to set OLED brightness (0x17 for full, 0x13 for dim)

////////////////////Setting GPS
// Conditional compilation checks for NMEAGPS library configuration.
#ifndef NMEAGPS_RECOGNIZE_ALL
#error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#if !defined(NMEAGPS_PARSE_GGA) & !defined(NMEAGPS_PARSE_GLL) & \
    !defined(NMEAGPS_PARSE_GSA) & !defined(NMEAGPS_PARSE_GSV) & \
    !defined(NMEAGPS_PARSE_RMC) & !defined(NMEAGPS_PARSE_VTG) & \
    !defined(NMEAGPS_PARSE_ZDA) & !defined(NMEAGPS_PARSE_GST)

    DEBUG_PORT.println(F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed."));

#else
    if (gps.merging == NMEAGPS::NO_MERGING)
    {
        DEBUG_PORT.print(F("\nWARNING: displaying data from "));
        DEBUG_PORT.print(gps.string_for(LAST_SENTENCE_IN_INTERVAL));
        DEBUG_PORT.print(F(" sentences ONLY, and only if "));
        DEBUG_PORT.print(gps.string_for(LAST_SENTENCE_IN_INTERVAL));
        DEBUG_PORT.println(F(" is enabled.\n"
                             "  Other sentences may be parsed, but their data will not be displayed."));
    }
#endif

    DEBUG_PORT.print(F("\nGPS quiet time is assumed to begin after a "));
    DEBUG_PORT.print(gps.string_for(LAST_SENTENCE_IN_INTERVAL));
    DEBUG_PORT.println(F(" sentence is received.\n"
                         "  You should confirm this with NMEAorder.ino\n"));

    //  trace_header( DEBUG_PORT ); // Commented out debug trace header
    DEBUG_PORT.flush(); // Ensure all debug output is sent

    gpsPort.begin(9600);
    gpsPort.println(PMTK_SET_NMEA_OUTPUT_RMCONLY); // Configure GPS to output RMC sentences only
    delay(10);
    gpsPort.println(PMTK_API_SET_FIX_CTL_5HZ); // Configure GPS fix control
    delay(10);
    gpsPort.println(PMTK_SET_NMEA_UPDATE_10HZ); // Configure GPS update rate
    delay(10);
}

//------------------------------------------------------------------------------
// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t *date, uint16_t *time, uint8_t *ms10)
{
    // Always initialize pointers to default values in case GPS data is not valid.
    // This prevents SdFat from getting uninitialized or garbage values.
    // Using a fixed known date/time (e.g., 2000-01-01 00:00:00) is a good practice.
    *date = FS_DATE(2000, 1, 1); // Default: Jan 1, 2000
    *time = FS_TIME(0, 0, 0);    // Default: 00:00:00
    *ms10 = 0;

    if (fix.valid.time && fix.valid.date)
    {
        // Serial.print("Current UTC Time: ");
        // Serial.print(fix.dateTime.hours - 6);
        // Serial.print(":");
        // Serial.print(fix.dateTime.minutes);
        // Serial.print(":");
        // Serial.print(fix.dateTime.seconds);
        // Serial.print(".");
        // Serial.print(fix.dateTime_cs);
        // Serial.print(" on ");
        // Serial.print(fix.dateTime.month);
        // Serial.print("/");
        // Serial.print(fix.dateTime.day);
        // Serial.print("/");
        // Serial.println(fix.dateTime.year);

        *date = FS_DATE(fix.dateTime.year + 2000, fix.dateTime.month, fix.dateTime.day);

        // Return time using FS_TIME macro to format fields.
        *time = FS_TIME(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);

        // Return low time bits in units of 10 ms, 0 <= ms10 <= 199.
        *ms10 = fix.dateTime_cs;
    }
}

// Go Home Function - Resets menu position to the top level.
void GoHome()
{
    Oled.clear();   // Clear the OLED display
    menuDepth = 0;  // Reset menu depth to top level
    menu1Index = 0; // Reset all menu indices to 0
    menu2Index = 0;
    menu3Index = 0;
}

// button up code: Handles actions when the Up button is released.
void CheckUpButton()
{
    if (UpButton.wasReleased() && !longpressUp) // Check if button was released and not a long press
    {
        Oled.clear();            // Clear display on button press
        if (menu.IsSelectable()) // Check if current menu item is a "Yes/No" (selectable) type
        {
            menu.SelectUp(); // Call MenuMaster to handle selection (e.g., toggle Yes/No)
        }
        else if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST) // Check if current menu item is a "LIST" type
        {
            menu.SelectUp();                           // Call MenuMaster to scroll up in the list
            menu.LoadTracks(sd, tracksDir, trackFile); // Reload track names for display after scrolling
        }
        else if (!menu.IsSelectable()) // If not a selectable (Yes/No) menu
        {
            switch (menuDepth)
            {                             // Navigate based on current menu depth
            case -1:                      // Logger screen (menuDepth == -1)
                newLoggerIndexPosition++; // Increment scroll position for logger list
                break;
            case 0: // Top-level menu
                // Check if moving to the next item would go out of bounds or hit a BLOCK
                if (menu1Index >= (*(menuSize + 0) - 1) || menu.GetMenuType((menu1Index + 1), menu2Index, menu3Index) == BLOCK)
                {
                }
                else // If within bounds, increment menu1Index to move down the list
                    menu1Index++;
                break;
            case 1: // Second-level menu
                // Check bounds/BLOCK for menu2Index
                if (menu2Index >= *(menuSize + 1) || menu.GetMenuType(menu1Index, (menu2Index + 1), menu3Index) == BLOCK)
                {
                }
                else // Increment menu2Index
                    menu2Index++;
                break;
            case 2: // Third-level menu
                // Check bounds/BLOCK for menu3Index
                if (menu3Index >= *(menuSize + 2) || menu.GetMenuType(menu1Index, menu2Index, (menu3Index + 1)) == BLOCK)
                {
                }
                else // Increment menu3Index
                    menu3Index++;
                break;
            case 3:
                // Code for depth 3 (currently empty)
                break;
            default:
                // Default case (currently empty)
                break;
            }
        }
    }
    longpressUp = false; // Reset long press flag

    // button up longpress: Handles actions for a long press of the Up button.
    if (UpButton.pressedFor(LONG_PRESS))
    {
        GoHome(); // Go back to the main menu (depth 0, index 0)
        // menu.SetLoggersTrue(); // Enable all loggers
        longpressUp = true; // Set long press flag to prevent immediate release action
    }
}
// button down code: Handles actions when the Down button is released.
void CheckDownButton()
{
    if (DownButton.wasReleased()) // Check if button was released
    {
        Oled.clear();            // Clear display on button press
        if (menu.IsSelectable()) // Check if current menu item is a "Yes/No" (selectable) type
        {
            menu.SelectDown(); // Call MenuMaster to handle selection (e.g., toggle Yes/No)
        }
        else if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST) // Check if current menu item is a "LIST" type
        {
            menu.SelectDown();                         // Call MenuMaster to scroll down in the list
            menu.LoadTracks(sd, tracksDir, trackFile); // Reload track names for display after scrolling
        }
        else if (!menu.IsSelectable()) // If not a selectable (Yes/No) menu
        {
            switch (menuDepth)
            {                             // Navigate based on current menu depth
            case -1:                      // Logger screen
                newLoggerIndexPosition--; // Decrement scroll position for logger list
                break;
            case 0: // Top-level menu
                if (menu1Index == 0)
                {
                } // Prevent going below the first item
                else
                { // If within bounds, decrement menu1Index to move up the list
                    menu1Index--;
                }
                break;
            case 1: // Second-level menu
                if (menu2Index == 1)
                {
                } // Specific boundary for menu2Index (might be related to menu structure)
                else
                { // Decrement menu2Index
                    menu2Index--;
                }
                break;
            case 2: // Third-level menu
                if (menu3Index == 1)
                {
                } // Specific boundary for menu3Index
                else
                { // Decrement menu3Index
                    menu3Index--;
                }
                break;
            case 3:
                // Code for depth 3 (currently empty)
                break;
            default:
                // Default case (currently empty)
                break;
            }
        }
    }
}

// button left code: Handles actions when the Left button is released.
void CheckLeftButton()
{
    if (LeftButton.wasReleased()) // Check if button was released
    {
        Oled.clear();            // Clear display on button press
        if (menu.IsSelectable()) // If in a "Yes/No" (selectable) menu
        {
            // Reset the index of the current depth to 0 when moving back,
            // ensuring the first item is selected when re-entering a sub-menu.
            switch (menuDepth)
            {
            case 0:
                menu1Index = 0;
                break;
            case 1:
                menu2Index = 0;
                break;
            case 2:
                menu3Index = 0;
                break;
            case 3:
                // Code for depth 3 (currently empty)
                break;
            default:
                // Default case (currently empty)
                break;
            }
            if (menuDepth <= -1) // If at logger screen or lower (cannot go back further)
            {
                // DO NOTHING BECAUSE AINT SHIT HERE
            }
            else // Otherwise, go back one menu depth
            {
                menuDepth--;
            }
        }
        else if (!menu.IsSelectable()) // If not a selectable (Yes/No) menu
        {
            // Reset the index of the current depth to 0 when moving back,
            // ensuring the first item is selected when re-entering a sub-menu.
            switch (menuDepth)
            {
            case 0:
                // No index reset needed for depth 0 when going back (already at top)
                break;
            case 1:
                menu2Index = 0; // Go to index zero to selected menu
                break;
            case 2:
                menu3Index = 0; // Go to index zero to selected menu
                break;
            case 3:
                // Code for depth 3 (currently empty)
                break;
            default:
                // Default case (currently empty)
                break;
            }
            if (menuDepth <= -1) // If at logger screen or lower (cannot go back further)
            {
                // DO NOTHING BECUZ SHIT CANT EXIST LOWER
            }
            else // Otherwise, go back one menu depth
            {
                menuDepth--; // else to back a menu
            }
        }
    }
}
// button right code: Handles actions when the Right button is released.
void CheckRightButton()
{
    if (RightButton.wasReleased())
    {
        Oled.clear();            // Clear display on button press
        if (menu.IsSelectable()) // If in a "Yes/No" (selectable) menu (e.g., confirming a setting)
        {
            menu3Index = 0; // Reset index (might be specific to the Yes/No menu structure)
            menuDepth--;    // Go back one menu level after selection
        }
        else if (!menu.IsSelectable()) // If not a selectable (Yes/No) menu
        {
            // Check if the current menu item is a CALLBACK type
            if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == CALLBACK)
            {
                switch (menuDepth)
                {
                case -1:
                    break;               // No callback from logger screen
                case 0:                  // Top-level menu callbacks
                    if (menu1Index == 1) // Corresponds to "Data Logger" (TL2)
                    {
                        // This block is empty, suggesting no direct callback from TL2 itself,
                        // but rather from its sub-items.
                    }
                    else if (menu1Index == 3) // Corresponds to "Soft Reset" (TL4)
                    {
                        // Do The Soft reset
                        resetFunc(); // Execute the software reset function
                    }
                    GoHome(); // Go back to the main menu after callback
                    break;
                case 1:                  // Second-level menu callbacks
                    if (menu2Index == 1) // Specific callback (e.g., "Reset Dash Serial")
                    {
                        //  Serial.write("Resetting Dash Serial"); // Commented out debug
                        //  Serial2.end(); // Commented out serial reset
                        //  delay(300);
                        //  Serial2.begin(9600);
                    }
                    GoHome(); // Go back to the main menu after callback
                    break;
                case 2:
                    GoHome(); // Go back to the main menu after callback
                    break;
                case 3:
                    GoHome(); // Go back to the main menu after callback
                    break;
                default:
                    GoHome(); // Go back to the main menu after callback
                    break;
                }
            }
            // Check if the current menu item is a LIST type (e.g., selecting a track from the list)
            else if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST) // FIRST INSTANCE OF LIST HANDLING (Selecting a track)
            {
                switch (menuDepth)
                {
                case -1:
                    break; // No list selection from logger screen
                case 0:
                    break;               // No list selection at depth 0
                case 1:                  // Second-level menu (where "Track Select" is located)
                    if (menu1Index == 2) // If the top-level menu is "Track Select" (TL3)
                    {
                        // If "None" is selected (index 0 in the track list)
                        if ((menu.trackSelectIndex + menu.trackReadIndex) == 0)
                        {
                            trackFile.close(); // Close any open track file
                            GoHome();          // Go back to main menu
                            break;
                        }
                        // Do the List Logic: Open and select the chosen track file.
                        byte k = 0;
                        if (!tracksDir.open("/Tracks", O_READ))
                        {
                            Serial.println("Error: Could not open /Tracks directory.");
                            Serial.println("Make sure the /Tracks directory exists on your SD card.");
                            sd.errorPrint(&Serial); // Uncomment for more detailed error info if needed
                        }

                        Serial.print("Selecting: ");                                   // Debug print
                        Serial.println((menu.trackSelectIndex + menu.trackReadIndex)); // Debug print
                        // Iterate through files to find the selected track.
                        while (trackFile.openNext(&tracksDir, O_READ))
                        {
                            k++;
                            if (k == (menu.trackSelectIndex + menu.trackReadIndex)) // If current file matches selection
                            {
                                Serial.print("Selected Track: "); // Debug print
                                trackFile.printName(&Serial);     // Print track name to serial
                                Serial.println();
                                break; // Found and selected, exit loop
                            }
                            trackFile.close(); // Close file if not the selected one
                        }
                        tracksDir.rewindDirectory();

                        InitializeTraps(); // Load GPS trap data for the newly selected track
                        menuDepth--;       // Go back one menu level
                        menu2Index = 0;    // Reset index for the previous menu level
                    }
                    else if (menu1Index == 3) // This condition likely won't be met for a LIST type
                    {
                        // Wont be called
                        GoHome();
                    }
                    else
                    {
                        GoHome();
                    }
                    break;
                case 2:
                    if (menu1Index == 2) // If top-level is "Track Select"
                    {
                        GoHome(); // Go home after selection
                    }
                    else if (menu1Index == 3) // This condition likely won't be met
                    {
                        // Wont be called
                        GoHome();
                    }
                    else
                    {
                        GoHome();
                    }
                    break;
                case 3:
                    GoHome(); // Go home after selection
                    break;
                default:
                    GoHome(); // Go home after selection
                    break;
                }
            }
            // If not a CALLBACK or LIST type, it's a regular menu item to enter a sub-menu.
            else // SECOND INSTANCE OF LIST HANDLING (Entering the track selection menu)
            {
                if (menuDepth > 2)
                {
                } // Limit depth to prevent going too deep (max depth 2 for sub-menus)
                else             // If within limits, progress to next menu depth
                    menuDepth++; // Progress to next menu depth

                switch (menuDepth)
                { // Handle actions based on the new menu depth
                case -1:
                    break; // Cannot enter logger screen from a menu
                case 0:
                    menu1Index = 0; // Reset index for depth 0 (shouldn't happen when progressing depth)
                    break;
                case 1:             // Entering a second-level menu
                    menu2Index = 1; // Set default index for this depth (might be specific to menu structure)

                    if (menu1Index == 2) // If the top-level menu is "Track Select" (TL3)
                    {
                        if (sdReady) // Check if SD card is ready to load tracks
                        {
                            trackFile.close(); // Close any previously open track file
                            // Do the List Logic: This section populates the track list when entering the menu.
                            byte k = 0;
                            // if (!sd.chdir("Tracks"))
                            // {                                                              // Change directory to "Tracks" folder
                            //     Serial.println(F("chdir failed for track traps folder.")); // Debug print
                            // }
                            if (!tracksDir.open("/Tracks", O_READ))
                            {
                                Serial.println("Error: Could not open /Tracks directory.");
                                Serial.println("Make sure the /Tracks directory exists on your SD card.");
                                sd.errorPrint(&Serial); // Uncomment for more detailed error info if needed
                            }

                            // Iterate through files in the "Tracks" directory to count them and print names to serial.
                            while (trackFile.openNext(&tracksDir, O_READ))
                            {
                                if (trackFile.isFile())
                                {
                                    k++;                  // Increment track count
                                    Serial.print(k, DEC); // Debug print track number
                                    Serial.print(". ");
                                    trackFile.printName(&Serial); // Print track name to serial
                                    Serial.println();
                                }
                                trackFile.close(); // Close file
                            }
                            tracksDir.rewindDirectory();
                            menu.trackCount = k;                       // Set total number of tracks found
                            menu.trackSelectIndex = 0;                 // Reset selected track index
                            menu.trackReadIndex = 0;                   // Reset read index for display
                            menu.LoadTracks(sd, tracksDir, trackFile); // Load initial track names into MenuMaster's buffer
                        }
                        else // If SD Not Ready
                        {
                            Oled.clear();                           // Clear display
                            PrintLcd('l', 0, 0, F("SD Not Ready")); // Display error message
                            PrintLcd('l', 0, 1, F("Check Card"));
                            PrintLcd('l', 0, 3, F("Then Soft Reset"));
                            delay(3500);  // Display for a few seconds
                            Oled.clear(); // Clear display
                            GoHome();     // Go back to main menu
                        }
                    }
                    break;
                case 2:             // Entering a third-level menu
                    menu3Index = 1; // Set default index for this depth
                    break;
                case 3:
                    // Code for depth 3 (currently empty)
                    break;
                default:
                    // Default case (currently empty)
                    break;
                }
            }
        }
    }
}

// Clear a row of logger screen: Clears a specific row on the OLED display.
void ClearRow(int rowNum)
{
    Oled.setCursor(0, rowNum);            // Set cursor to the beginning of the specified row
    Oled.print(F("                   ")); // Print spaces to clear the row (20 spaces for 20-char display)
}

// Code to print to lcd screens: Overloaded function to print String messages to the OLED.
void PrintLcd(char screen, int col, int row, String message)
{
    switch (screen)
    {
    case 's': // Small LCD (deprecated in this version)
        Oled.print(F("No sm LCD in this version"));
        break;
    case 'l':                     // Large LCD (OLED)
        Oled.setCursor(col, row); // Set cursor position
        Oled.print(message);      // Print the message
        break;
    default:
        Oled.print(F("Some Fatal Error")); // Error message for invalid screen type
    }
}
// Overload for long: Overloaded function to print long integer messages to the OLED.
void PrintLcd(char screen, int col, int row, long message)
{
    switch (screen)
    {
    case 's': // Small LCD (deprecated)
        Oled.print(F("No sm LCD in this version"));
        break;
    case 'l':                     // Large LCD (OLED)
        Oled.setCursor(col, row); // Set cursor position
        Oled.print(message);      // Print the long integer
        break;
    default:
        Oled.print(F("Some Fatal Error")); // Error message
    }
}

// Returns Free Ram of Arduino - Used for debugging: Calculates and returns the amount of free RAM.
int FreeRam()
{
    extern int __heap_start, *__brkval;                                    // Pointers to heap start and break
    int v;                                                                 // Local variable to get stack pointer
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval); // Calculate free RAM
}

// Updates Large LCD position based on menu modifier number: Manages scrolling logic for the logger display.
void RefreshLoggerList()
{
    // Large LCD Scrolling Code to set menu modifier
    if (newLoggerIndexPosition != lastLoggerIndexPosition)
    {                                                     // If desired scroll position has changed
        lastLoggerIndexPosition = newLoggerIndexPosition; // Update last position
        // Check if new position is within bounds (allowing for 4 lines of display)
        if ((newLoggerIndexPosition < (lgLcdList.size() - 3)) && newLoggerIndexPosition >= 0)
        {
            menuModifier = newLoggerIndexPosition; // Set the modifier for display offset
            loggerIndexChanged = true;             // Flag that display needs refreshing
        }
        else if ((newLoggerIndexPosition >= (lgLcdList.size() - 3))) // If trying to scroll too far down
        {
            newLoggerIndexPosition = (lgLcdList.size() - 4); // Cap at the lowest possible scroll position
            loggerIndexChanged = true;                       // Flag for refresh
        }
        else // If trying to scroll too far up
        {
            newLoggerIndexPosition = 0; // Cap at the top (index 0)
        }
    }
}

// Overloaded for custom linked list sizes (not really used): Overloaded version for custom list sizes (unused).
void RefreshLoggerList(byte size)
{
    // Large LCD Scrolling Code to set menu modifier
    if (newLoggerIndexPosition != lastLoggerIndexPosition)
    {
        lastLoggerIndexPosition = newLoggerIndexPosition;
        if ((newLoggerIndexPosition < (size - 3)) && newLoggerIndexPosition >= 0)
        {
            menuModifier = newLoggerIndexPosition;
            loggerIndexChanged = true;
        }
        else if ((newLoggerIndexPosition >= (size - 3)))
        {
            newLoggerIndexPosition = (lgLcdList.size() - 4);
            loggerIndexChanged = true;
        }
        else
        {
            newLoggerIndexPosition = 0;
        }
    }
}

// Draw Arrow for more than 4 loggers: Draws scrollbar arrows and logging icon on the OLED.
void UpdateScrollbar()
{
    // Check if there are more loggers below the current view.
    if (loggerCount > (4 + menuModifier))
    {
        if (!splashInterrupt)
        {                          // Only draw if no splash screen is active
            Oled.setCursor(19, 2); // Position for pipe character
            Oled.write(byte(2));   // Draw pipe
            Oled.setCursor(19, 3); // Position for down arrow
            Oled.write(byte(1));   // Draw down arrow
        }
    }
    else // No more loggers below
    {
        if (!splashInterrupt)
        {
            Oled.setCursor(19, 2);
            Oled.print(" "); // Clear pipe
            Oled.setCursor(19, 3);
            Oled.print(" "); // Clear down arrow
        }
    }
    // Check if there are more loggers above the current view.
    if (loggerCount > 4 && menuModifier > 0)
    {
        if (!splashInterrupt)
        {
            Oled.setCursor(19, 0); // Position for top icon/arrow
            if (loggerRunning)     // If logging is active, show log icon
            {
                Oled.write(byte(3)); // Draw log icon
            }
            else // Otherwise, show up arrow
            {
                Oled.write(byte(0)); // Draw up arrow
            }
            Oled.setCursor(19, 1); // Position for pipe character
            Oled.write(byte(2));   // Draw pipe
        }
    }
    else // No more loggers above or not enough loggers to scroll
    {
        if (!splashInterrupt)
        {
            Oled.setCursor(19, 0);
            if (loggerRunning) // Still show log icon if logging, even if no scroll
            {
                Oled.write(byte(3));
            }
            else
            {
                Oled.print(" "); // Clear top icon/arrow
            }
            Oled.setCursor(19, 1);
            Oled.print(" "); // Clear pipe
        }
    }
}

// Returns digit length: Calculates the number of digits in an integer.
int GetIntLength(int number)
{
    int digits = 0;
    if (number < 0)
        digits = 1; // Account for '-' sign if it counts as a digit
    if (number != 0)
    {
        while (number)
        {
            number /= 10;
            digits++;
        }
    }
    else
    {
        return 1; // Special case for number 0
    }
    return digits;
}

// Formats The Large LCD String: Formats sensor values into a fixed-width string for display.
String FormatString(String text, float value, byte pres)
{
    String output;
    byte strLEN = text.length(); // Length of the logger text
    int roundVal;

    if (pres == 0) // If precision is 0 (for integer values)
    {
        roundVal = round(value);              // Round the float to nearest integer
        char valLEN = GetIntLength(roundVal); // Get length of the integer value

        String endBuf; // Buffer for padding spaces
        if (valLEN == 5)
            endBuf = F("");
        else if (valLEN == 4)
            endBuf = F(" ");
        else if (valLEN == 3)
            endBuf = F("   ");
        else if (valLEN == 2)
            endBuf = F("    ");
        else if (valLEN == 1)
            endBuf = F("     ");

        output = text + F("  ") + roundVal + endBuf; // Concatenate text, spaces, value, and padding
    }
    else // If precision is not 0 (for float values, e.g., G-forces)
    {
        String endBuf;
        if (GetIntLength((int)value) == 1) // If integer part is single digit
        {
            if (value < 0)
            {
                endBuf = F(" "); // Pad for negative single digit
            }
            else
            {
                endBuf = F("  "); // Pad for positive single digit
            }
        }
        else // If integer part is multiple digits
        {
            if (value < 0)
            {
                endBuf = F(""); // No padding for negative multi-digit
            }
            else
            {
                endBuf = F(" "); // Single space padding for positive multi-digit
            }
        }
        value = (int)(value * 100);               // Multiply by 100 to shift two decimal places
        value = (float)(value / 100);             // Divide by 100 to get original value with two decimal places
        output = text + F("  ") + value + endBuf; // Concatenate text, spaces, value, and padding
    }
    return output;
}

// Non-blocking CAN message processing
void processCanMessages()
{
    unsigned long currentTime = millis();

    // // Check for CAN timeout (vehicle off)
    // if (currentTime - lastCanActivity > CAN_TIMEOUT_MS) {
    //     if (!canSilent) {
    //         canSilent = true;
    //         Serial.println("CAN bus silent - vehicle off");
    //         // Turn off screen and external Arduino when going silent
    //         if (screenAwake) {
    //             digitalWrite(arduinoSleepPin, LOW); // Turn off the external Arduino board
    //             Oled.noDisplay();                   // Turn off OLED display
    //             analogWrite(screenPwmPin, 0);       // Set screen PWM to 0 (off)
    //             screenAwake = false;                // Set screen awake flag to false
    //         }
    //     }
    // }

    // Process available CAN messages
    while (canMessageAvailable || CAN0.checkReceive() == CAN_MSGAVAIL)
    {
        canMessageAvailable = false;
        lastCanActivity = currentTime;

        // // Wake up from sleep if we were silent
        // if (canSilent) {
        //     canSilent = false;
        //     Serial.println("CAN bus active - vehicle on");
        //     // Turn screen and external Arduino back on
        //     if (!screenAwake) {
        //         Oled.display(); // Turn on OLED display
        //         if (screenDim) {
        //             analogWrite(screenPwmPin, screenDimLvl);
        //         } else {
        //             analogWrite(screenPwmPin, 255);
        //         }
        //         digitalWrite(arduinoSleepPin, HIGH); // Turn on the external Arduino board
        //         screenAwake = true;
        //         ledBarCurrentGate = -1; // Reset RPM bar state to force reinitialization

        //     }
        // }

        long unsigned int rxId;
        unsigned char len;
        unsigned char rxBuf[8];

        // Read the message
        if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK)
        {
            processCanMessage(rxId, len, rxBuf, currentTime);
        }
    }

    // Update global variables with latest valid data
    updateGlobalCanVariables();
}

// Process individual CAN message based on ID
void processCanMessage(long unsigned int rxId, unsigned char len, unsigned char rxBuf[], unsigned long timestamp)
{
    // Since we're using a permissive mask for some IDs, we need to filter the ones we want
    switch (rxId)
    {
    case 0x792: // Brake Light and wheel speeds
        if (len >= 3)
        {
            canBRK.value = (rxBuf[2] * 100) / 255;
            canBRK.timestamp = timestamp;
            canBRK.valid = true;
        }
        break;

    case 0x2D1: // Speed and Fuel Level
        if (len >= 7)
        {
            int tmpA = rxBuf[2] * 256;
            int tmpB = rxBuf[1];
            canSPD.value = (tmpA + tmpB) / 16;
            canSPD.timestamp = timestamp;
            canSPD.valid = true;

            canFuelLevel.value = 3 - rxBuf[6];
            canFuelLevel.timestamp = timestamp;
            canFuelLevel.valid = true;
        }
        break;

    case 0x23D: // TPS and RPM
        if (len >= 5)
        {
            int tmpA = rxBuf[4] * 256;
            int tmpB = rxBuf[3];
            canRPM.value = (tmpA + tmpB) * 3.15;
            canRPM.timestamp = timestamp;
            canRPM.valid = true;

            canTPS.value = (rxBuf[2] * 100) / 255;
            canTPS.timestamp = timestamp;
            canTPS.valid = true;

            canPDL.value = (rxBuf[1] * 100) / 255;
            canPDL.timestamp = timestamp;
            canPDL.valid = true;
        }
        break;

    case 0x002: // Steering Angle Sensor
        if (len >= 2)
        {
            int tmpB = rxBuf[1] * 256;
            int tmpA = rxBuf[0];
            tmpA = tmpA + tmpB;
            if (tmpA > 32767)
            {
                canSAS.value = -(65535 - tmpA / 10);
            }
            else
            {
                canSAS.value = tmpA / 10;
            }
            canSAS.timestamp = timestamp;
            canSAS.valid = true;
        }
        break;

    case 0x551: // ECM data (Engine Coolant Temp)
        if (len >= 1)
        {
            int tmpA = rxBuf[0] - 39;
            tmpA = tmpA * 9 / 5;
            canECT.value = tmpA + 32;
            canECT.timestamp = timestamp;
            canECT.valid = true;
        }
        break;

    case 0x625: // Vehicle status
        // Handle screen dimming and turn signals
        handleVehicleStatus(rxBuf, len, timestamp);
        break;

    case 0x60D: // Turn signals
        if (len >= 2)
        {
            handleTurnSignals(rxBuf[1], timestamp);
        }
        break;

    default:
        // Ignore unwanted CAN IDs (since we're using permissive filtering)
        break;
    }
}

// Handle vehicle status messages (screen dimming, etc.)
void handleVehicleStatus(unsigned char rxBuf[], unsigned char len, unsigned long timestamp)
{
    if (len >= 4)
    {
        // Check if vehicle is on/off
        if (rxBuf[3] != 0x90 && rxBuf[3] != 0x10)
        {
            canSilent = true;       // Set flag for CAN silence
            if (menu.enableLogging) // If logging was active, stop it
            {
                sdLog.close();                 // Close SD log file
                Serial.println(F("SDCLOSED")); // Debug print
                menu.UnlockLoggers();          // Unlock loggers (allow user to change settings)
                logTime.stop();                // Stop overall log timer
                logTime.reset();               // Reset overall log timer
                lapTime.stop();                // Stop lap timer
                lapTime.reset();               // Reset lap timer
                Serial2.println("4,stop,,,");  // Send command to external display to stop timer
                previousLapTime = 0;           // Reset lap timing variables
                laps = 0;
                zone = 0;
                onTrack = false;
                trackFile.seekSet(0);  // Rewind track file pointer
                loggerRunning = false; // Set logging flag to false
            }
        }
    }

    if (len >= 2)
    {
        // Handle screen brightness
        if ((rxBuf[1] & 0x70) == 0x0)
        {
            if (screenDim)
            {
                Oled.command(0x17);
                analogWrite(screenPwmPin, 255);
                screenDim = false;
            }
        }
        else
        {
            if (!screenDim)
            {
                analogWrite(screenPwmPin, screenDimLvl);
                Oled.command(0x13);
                screenDim = true;
            }
        }
    }
}

// Handle turn signal status
void handleTurnSignals(unsigned char statusByte, unsigned long timestamp)
{
    // Left Signal
    if ((statusByte & 0x20) == 0x0)
    {
        if (leftSignal)
        {
            Serial2.println("5,,0,,");
            leftSignal = false;
        }
    }
    else
    {
        if (!leftSignal)
        {
            Serial2.println("5,,1,,");
            leftSignal = true;
        }
    }

    // Right Signal
    if ((statusByte & 0x40) == 0x0)
    {
        if (rightSignal)
        {
            Serial2.println("5,,,0,");
            rightSignal = false;
        }
    }
    else
    {
        if (!rightSignal)
        {
            Serial2.println("5,,,1,");
            rightSignal = true;
        }
    }
}

// Update global variables with latest CAN data
void updateGlobalCanVariables()
{
    unsigned long currentTime = millis();
    const unsigned long DATA_TIMEOUT_MS = 1000; // 1 second timeout for stale data

    // Only update if data is valid and recent
    if (canTPS.valid && (currentTime - canTPS.timestamp < DATA_TIMEOUT_MS))
    {
        TPS = canTPS.value;
    }
    if (canPDL.valid && (currentTime - canPDL.timestamp < DATA_TIMEOUT_MS))
    {
        PDL = canPDL.value;
    }
    if (canBRK.valid && (currentTime - canBRK.timestamp < DATA_TIMEOUT_MS))
    {
        BRK = canBRK.value;
    }
    if (canSPD.valid && (currentTime - canSPD.timestamp < DATA_TIMEOUT_MS))
    {
        SPD = canSPD.value;
    }
    if (canRPM.valid && (currentTime - canRPM.timestamp < DATA_TIMEOUT_MS))
    {
        RPM = canRPM.value;
    }
    if (canSAS.valid && (currentTime - canSAS.timestamp < DATA_TIMEOUT_MS))
    {
        SAS = canSAS.value;
    }
    if (canECT.valid && (currentTime - canECT.timestamp < DATA_TIMEOUT_MS))
    {
        ECT = canECT.value;
    }
    if (canFuelLevel.valid && (currentTime - canFuelLevel.timestamp < DATA_TIMEOUT_MS))
    {
        FuelLevel = canFuelLevel.value;
    }

    // Send data to Serial2 (external display)
    sendDataToDisplay();
}

// Send current data to external display
void sendDataToDisplay()
{
    Serial2.print("1,");
    Serial2.print(PDL);
    Serial2.print(",");
    Serial2.print(BRK);
    Serial2.print(",");
    Serial2.print(RPM);
    Serial2.print(",");
    Serial2.print(SPD);
    Serial2.println();

    Serial2.print("2,");
    Serial2.print(ECT);
    Serial2.print(",");
    Serial2.print(FuelLevel);
    Serial2.print(",");
    Serial2.print(splitTime);
    Serial2.print(",");
    Serial2.print(laps);
    Serial2.println();
}
// Updates Logger Variables with current CANBUS data: Reads CAN bus messages and updates sensor variables.
void UpdateLoggers()
{
    processCanMessages(); // Non-blocking CAN processing

    // Reset flags for received CAN messages for the current loop iteration.
    // recvA = false;
    // recvB = false;
    // recvC = false;
    // recvD = false;
    // recvE = false;
    // recvF = false;
    // do
    // {                        // Loop to read all available CAN messages until all expected types are received or no more messages.
    //     if (!digitalRead(2)) // If pin 2 (MCP2515 interrupt) is low, new message is available
    //     {
    //         if (!screenAwake) // If screen was off, turn it back on
    //         {
    //             Oled.display(); // Turn on OLED display
    //             if (screenDim)  // If screen was dimmed, restore dimmed brightness
    //             {
    //                 analogWrite(screenPwmPin, screenDimLvl);
    //             }
    //             else
    //             { // Otherwise, restore full brightness
    //                 analogWrite(screenPwmPin, 255);
    //             }
    //             digitalWrite(arduinoSleepPin, HIGH); // Turn on the external Arduino board
    //             screenAwake = true;                  // Set screen awake flag
    //         }
    //         CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data from CAN buffer
    //         // OLD Depricated rxID method rxId = CAN0.getCanId(); // Old method for getting ID

    //         if (rxId == 0x792 && !recvA)        // CAN ID for Brake Light and wheel speeds
    //         {                                   // Brake Light and individual wheel speeds
    //             BRK = ((rxBuf[2] * 100) / 255); // Calculate brake pressure from byte 2
    //             recvA = true;                   // Mark as received
    //         }
    //         if (rxId == 0x2D1 && !recvB) // CAN ID for Speed and Fuel Level
    //         {
    //             tmpA = 0;
    //             tmpB = 0;
    //             tmpA = rxBuf[2] * 256;      // High byte of speed
    //             tmpB = rxBuf[1];            // Low byte of speed
    //             SPD = ((tmpA + tmpB) / 16); // Calculate speed
    //             FuelLevel = 3 - rxBuf[6];   // Calculate fuel level (assuming 3 is max)
    //             recvB = true;
    //         }
    //         if (rxId == 0x23D && !recvC) // CAN ID for TPS and RPM
    //         {                            // Two TPS and two byte RPM
    //             tmpA = 0;
    //             tmpB = 0;
    //             tmpA = rxBuf[4] * 256;          // High byte of RPM
    //             tmpB = rxBuf[3];                // Low byte of RPM
    //             RPM = ((tmpA + tmpB) * 3.15);   // Calculate RPM
    //             TPS = ((rxBuf[2] * 100) / 255); // Calculate TPS from byte 2
    //             PDL = ((rxBuf[1] * 100) / 255); // Calculate Pedal Position from byte 1
    //             recvC = true;
    //         }
    //         if (rxId == 0x2 && !recvD) // CAN ID for Steering Angle Sensor
    //         {                          // Steering Angle Sensor and velocity
    //             tmpA = 0;
    //             tmpB = 0;
    //             tmpB = rxBuf[1] * 256; // High byte of steering angle
    //             tmpA = rxBuf[0];       // Low byte of steering angle
    //             tmpA = (tmpA + tmpB);
    //             if (tmpA > 32767)               // Handle signed 16-bit value
    //                 SAS = -(65535 - tmpA / 10); // Calculate negative steering angle
    //             else
    //                 SAS = tmpA / 10; // Calculate positive steering angle
    //             recvD = true;
    //         }
    //         if (rxId == 0x551 && !recvE) // CAN ID for ECM data (Engine Coolant Temp)
    //         {
    //             // Contains Some ECM data
    //             tmpA = 0;
    //             tmpB = 0;
    //             tmpA = rxBuf[0] - 39; // Raw temp value adjustment
    //             tmpA = tmpA * 9;      // Convert to Fahrenheit (part 1)
    //             tmpA = tmpA / 5;      // Convert to Fahrenheit (part 2)
    //             ECT = tmpA + 32;      // Convert to Fahrenheit (part 3)
    //             recvE = true;
    //         }
    //         if (rxId == 0x625) // CAN ID for vehicle status (e.g., ignition, door status)
    //         {
    //             // Check if CAN bus is silent (e.g., engine off, no activity)
    //             if (rxBuf[3] != 0x90 && rxBuf[3] != 0x10) // Specific byte values indicating activity
    //             {
    //                 canSilent = true;       // Set flag for CAN silence
    //                 if (menu.enableLogging) // If logging was active, stop it
    //                 {
    //                     sdLog.close();                 // Close SD log file
    //                     Serial.println(F("SDCLOSED")); // Debug print
    //                     menu.UnlockLoggers();          // Unlock loggers (allow user to change settings)
    //                     logTime.stop();                // Stop overall log timer
    //                     logTime.reset();               // Reset overall log timer
    //                     lapTime.stop();                // Stop lap timer
    //                     lapTime.reset();               // Reset lap timer
    //                     Serial2.println("4,stop,,,");  // Send command to external display to stop timer
    //                     previousLapTime = 0;           // Reset lap timing variables
    //                     laps = 0;
    //                     zone = 0;
    //                     onTrack = false;
    //                     trackFile.seekSet(0);  // Rewind track file pointer
    //                     loggerRunning = false; // Set logging flag to false
    //                 }
    //             }
    //         }
    //         if (rxId == 0x625) // Another check for the same CAN ID (for screen dimming)
    //         {
    //             // Reading with bit AND operator for masking the whole byte to just the three bits we care about for this rxId
    //             if ((rxBuf[1] & 0x70) == 0x0) // Check specific bits in byte 1 for screen brightness control
    //             {
    //                 if (screenDim) // If screen was dimmed, restore full brightness
    //                 {
    //                     Oled.command(0x17);             // OLED command for full brightness
    //                     analogWrite(screenPwmPin, 255); // PWM for full brightness
    //                     screenDim = false;              // Set screen dim flag to false
    //                 }
    //             }
    //             else // If bits indicate dimming
    //             {
    //                 if (!screenDim) // If screen was not already dimmed, dim it
    //                 {
    //                     analogWrite(screenPwmPin, screenDimLvl); // PWM for dimmed brightness
    //                     Oled.command(0x13);                      // OLED command for dim brightness
    //                     screenDim = true;                        // Set screen dim flag to true
    //                 }
    //             }
    //         }
    //         if (rxId == 0x60D && !recvF) // CAN ID for turn signals
    //         {
    //             // Left Signal: Check specific bits in byte 1 for left signal status
    //             if ((rxBuf[1] & 0x20) == 0x0) // If left signal is off
    //             {
    //                 if (leftSignal) // If it was previously on
    //                 {
    //                     Serial2.println("5,,0,,"); // Send command to external display to turn off left signal
    //                     leftSignal = false;        // Set flag to false
    //                 }
    //             }
    //             else // If left signal is on
    //             {
    //                 if (!leftSignal) // If it was previously off
    //                 {
    //                     Serial2.println("5,,1,,"); // Send command to external display to turn on left signal
    //                     leftSignal = true;         // Set flag to true
    //                 }
    //             }
    //             // Right Signal: Check specific bits in byte 1 for right signal status
    //             if ((rxBuf[1] & 0x40) == 0x0) // If right signal is off
    //             {
    //                 if (rightSignal) // If it was previously on
    //                 {
    //                     Serial2.println("5,,,0,"); // Send command to external display to turn off right signal
    //                     rightSignal = false;       // Set flag to false
    //                 }
    //             }
    //             else // If right signal is on
    //             {
    //                 if (!rightSignal) // If it was previously off
    //                 {
    //                     Serial2.println("5,,,1,"); // Send command to external display to turn on right signal
    //                     rightSignal = true;        // Set flag to true
    //                 }
    //             }
    //             recvF = true; // Mark as received
    //         }
    //     }
    // } while (!recvA && !recvB && !recvC && !recvD && !recvE && !recvF); // Continue loop until all expected messages are received

    /*
    Test data generation (commented out)
    BRK = random(10000);
    SPD = random(10000);
    ECT = random(10000);
    TPS = random(10000);
    RPM = random(10000);
    PDL = random(10000);
    SAS = random(10000);
  */

    /*
    // Debug prints to Serial (commented out)
    Serial.print(TPS);
    Serial.print(" , ");
    Serial.print(PDL);
    Serial.print(" , ");
    Serial.print(BRK);
    Serial.print(" , ");
    Serial.print(SPD);
    Serial.print(" , ");
    Serial.print(RPM);
    Serial.print(" , ");
    Serial.print(SAS);
    Serial.println();
    */

    // // Send sensor data to Serial2 (external display/device)
    // Serial2.print("1,"); // Command prefix
    // Serial2.print(PDL);
    // Serial2.print(",");
    // Serial2.print(BRK);
    // Serial2.print(",");
    // Serial2.print(RPM);
    // Serial2.print(",");
    // Serial2.print(SPD);
    // Serial2.println(); // End of command

    // Serial2.print("2,"); // Command prefix
    // Serial2.print(ECT);
    // Serial2.print(",");
    // Serial2.print(FuelLevel);
    // Serial2.print(",");
    // Serial2.print(splitTime); // Lap time difference
    // Serial2.print(",");
    // Serial2.print(laps); // Lap count
    // Serial2.println();   // End of command
}

// Save logger states and indexes to EEPROM: Saves the enabled state and display order of loggers to EEPROM.
void SaveToEEPROM()
{
    // Get Array Sizes for EEPROM write loops
    int menuLoggersSize = sizeof(menu.loggers) / sizeof(bool); // Size of the boolean array for logger states
    int loggerIndexSize = sizeof(loggerIndex) / sizeof(char);  // Size of the char array for logger display order
    // For Each item in the T/F logger state; write to eeprom
    for (int i = 0; i < menuLoggersSize; i++)
        EEPROM.write(i, menu.loggers[i]); // Write boolean state (0 or 1) to EEPROM
    // Continuing the EEPROM addresses from above, write the logger index values
    for (int i = menuLoggersSize; i < (loggerIndexSize + menuLoggersSize); i++)
        EEPROM.write(i, loggerIndex[i - menuLoggersSize]); // Write logger display order
}

// Read logger states and positions from EEPROM: Reads saved logger settings from EEPROM.
void ReadFromEEPROM()
{
    // Not sure if i really need this, but it helps it from reading trash if nothing is loaded into eeprom
    // (Good place to do data validation if i get around to it)
    if (EEPROM.read(0) != 0xff) // Check if EEPROM address 0 is not empty (0xff is default empty value)
    {
        // Get Array Sizes for EEPROM write loops
        int menuLoggersSize = sizeof(menu.loggers) / sizeof(bool);
        int loggerIndexSize = sizeof(loggerIndex) / sizeof(char);
        for (int i = 0; i < menuLoggersSize; i++)
            menu.loggers[i] = EEPROM.read(i); // Read boolean states
        for (int i = menuLoggersSize; i < (loggerIndexSize + menuLoggersSize); i++)
            loggerIndex[i - menuLoggersSize] = EEPROM.read(i); // Read logger display order
    }
    // Add loggers in on read. Unlike the loop add/remove logic, these already have predefined indexes and arent -1.
    // They just needed to be added to the lcd linked list
    for (int i = 0; i < 10; i++) // Iterate through all possible loggers
    {
        // Check Menu class to see if logger is enabled for this index/loop
        if (menu.loggers[i]) // If logger is enabled in the menu settings
        {
            // If it is greater than -1; because we are reading the last state from EEPROM
            if (loggerIndex[i] >= 0) // If this logger has a valid display index (meaning it was active)
            {
                // Add the logger text to the LCDList and increase the listAmt (for arrow code)
                lgLcdList.add(loggerIndex[i], loggerTexts[i]); // Add logger text to the display list at its saved position
                loggerCount++;                                 // Increment count of active loggers
            }
        }
    }
}

// InitializeTraps: Loads track zone (trap) data from a CSV file on the SD card.
void InitializeTraps()
{
    int ln = 0;
    trackFile.seekSet(0); // Rewind track file to the beginning
    while ((n = trackFile.fgets(line, sizeof(line))) > 0)
    { // Read line by line from the file
        // We are loading the first zone, so get to line NOT including header info
        if (ln == 0) // Assuming the first line contains the first zone's data
        {
            zoneName = strtok(line, ","); // Parse zone name
            for (int a = 0; a < 4; a = a + 1)
            { // Parse 4 latitude trap points
                latitudeTraps[a] = atol(strtok(NULL, ","));
            }
            for (int a = 0; a < 4; a = a + 1)
            { // Parse 4 longitude trap points
                longitudeTraps[a] = atol(strtok(NULL, ","));
            }
            maxZones = atoi(strtok(NULL, ",")); // Parse total number of zones
            break;                              // Stop after reading the first zone
        }
        ln++; // Increment line counter
    }
    trapsLoaded = true; // Set flag indicating trap data is loaded
}

// CollectLapTimerStats: Manages lap timing and track zone transitions based on GPS data.
void CollectLapTimerStats()
{
    if (newGpsData) // Only proceed if new GPS data is available
    {
        // Serial.print(F(" Current Lat/Lng:"));
        // Serial.print(fix.latitudeL());
        // Serial.print(F(","));
        // Serial.println(fix.longitudeL());
        // if (fix.valid.time && fix.valid.date)
        // {
        //     Serial.print("Current UTC Time: ");
        //     Serial.print(fix.dateTime.hours);
        //     Serial.print(":");
        //     Serial.print(fix.dateTime.minutes);
        //     Serial.print(":");
        //     Serial.print(fix.dateTime.seconds);
        //     Serial.print(" on ");
        //     Serial.print(fix.dateTime.month);
        //     Serial.print("/");
        //     Serial.print(fix.dateTime.day);
        //     Serial.print("/");
        //     Serial.println(fix.dateTime.year);
        // }
        if (trapsLoaded) // Only proceed if track trap data is loaded
        {

            // Debug prints for lap timing and GPS data (commented out)
            Serial.print(F("In Zone: "));
            Serial.print(zoneName);
            Serial.print(F(". Lap Time: "));
            Serial.print(LapTime);
            Serial.print(F(". Last Lap Time: "));
            Serial.print(lapTime.value());
            Serial.print(F(". LapCount: "));
            Serial.print(laps);

            // Check if current GPS position intersects with the current zone's polygon.
            if (PolyIntersect(4, latitudeTraps, longitudeTraps, fix.latitudeL(), fix.longitudeL()))
            {
                if (onTrack) // If already on track
                {
                    // This means a lap was completed, since we were already on track and we are in zone zero
                    if (zone == 0) // If re-entering the start/finish zone (zone 0)
                    {
                        laps++;                        // Increment lap count
                        previousLapTime = LapTime;     // Store current lap time as previous
                        LapTime = lapTime.value();     // Get current lap time from stopwatch
                        lapTime.reset();               // Reset lap stopwatch
                        lapTime.start();               // Start lap stopwatch for the new lap
                        Serial2.println("4,reset,,,"); // Command to external display to reset timer
                        Oled.clear();                  // Clear display
                        splashInterrupt = true;        // Activate lap completed splash screen
                    }

                    // We are at the last zone in the csv, reset to zero. lap will be calculated when in zone 0 again
                    if (maxZones == (zone + 1)) // If entering the last zone
                    {
                        zone = 0; // Reset zone to 0 (for next lap's start)
                        int ln = 0;
                        trackFile.seekSet(0); // Rewind track file
                        while ((n = trackFile.fgets(line, sizeof(line))) > 0)
                        {                   // Read file to load data for the new zone
                            if (ln == zone) // Load data for zone 0
                            {
                                zoneName = strtok(line, ",");
                                for (int a = 0; a < 4; a = a + 1)
                                {
                                    latitudeTraps[a] = atol(strtok(NULL, ","));
                                }
                                for (int a = 0; a < 4; a = a + 1)
                                {
                                    longitudeTraps[a] = atol(strtok(NULL, ","));
                                }
                                break;
                            }
                            else
                            {
                                ln++;
                            }
                        }
                    }
                    else // If entering a new intermediate zone
                    {
                        zone++; // Increment zone
                        int ln = 0;
                        trackFile.seekSet(0); // Rewind track file
                        while ((n = trackFile.fgets(line, sizeof(line))) > 0)
                        {                   // Read file to load data for the new zone
                            if (ln == zone) // Load data for the current zone
                            {
                                zoneName = strtok(line, ",");
                                for (int a = 0; a < 4; a = a + 1)
                                {
                                    latitudeTraps[a] = atol(strtok(NULL, ","));
                                }
                                for (int a = 0; a < 4; a = a + 1)
                                {
                                    longitudeTraps[a] = atol(strtok(NULL, ","));
                                }
                                break;
                            }
                            else
                            {
                                ln++;
                            }
                        }
                    }
                }
                else // We are getting on track now (first time entering zone 0)
                {
                    if (zone == 0) // You did your first lap and passed zone zero, time to start racing!
                    {
                        Oled.clear();                  // Clear display
                        splashInterrupt = true;        // Activate splash screen
                        lapTime.start();               // Start lap timer
                        Serial2.println("4,reset,,,"); // Command to external display to reset timer
                        Serial2.println("4,start,,,"); // Command to external display to start timer
                        zone++;                        // Move to next zone
                        int ln = 0;
                        trackFile.seekSet(0); // Rewind track file
                        while ((n = trackFile.fgets(line, sizeof(line))) > 0)
                        {                   // Read file to load data for the new zone
                            if (ln == zone) // Load data for the current zone
                            {
                                zoneName = strtok(line, ",");
                                for (int a = 0; a < 4; a = a + 1)
                                {
                                    latitudeTraps[a] = atol(strtok(NULL, ","));
                                }
                                for (int a = 0; a < 4; a = a + 1)
                                {
                                    longitudeTraps[a] = atol(strtok(NULL, ","));
                                }
                                break;
                            }
                            else
                            {
                                ln++;
                            }
                        }
                        onTrack = true; // Set flag indicating vehicle is on track
                    }
                }
            }
        }
    }

    // Log GPS and lap timing data to SD card.
    sdLog.print(F(","));
    sdLog.print(zoneName);
    sdLog.print(F(","));
    sdLog.print(lapTime.value());
    sdLog.print(F(","));
    sdLog.print(LapTime);
    sdLog.print(F(","));
    sdLog.print(laps);
    sdLog.print(F(","));
    sdLog.print(fix.latitudeL());
    sdLog.print(F(","));
    sdLog.print(fix.longitudeL());
    sdLog.println();    // New line for the next log entry
    newGpsData = false; // Reset flag for new GPS data
}
// PolyIntersect: Checks if a given point (testx, testy) is inside a polygon defined by vertx and verty.
// nvert: Number of vertices in the polygon.
// vertx, verty: Arrays of x and y coordinates of the polygon's vertices.
// testx, testy: Coordinates of the point to test.
bool PolyIntersect(int nvert, long *vertx, long *verty, long testx, long testy)
{
    int i, j, c = 0;                               // c is a counter for intersections
    for (i = 0, j = nvert - 1; i < nvert; j = i++) // Loop through each edge of the polygon
    {
        // Check if the ray from test point intersects the current edge
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c; // Toggle intersection counter
    }
    return c; // Returns true if point is inside (odd number of intersections), false otherwise
}

// FormatMilliseconds: Converts milliseconds into a formatted time string (MM:SS.mmm or SS.mmm).
String FormatMilliseconds(unsigned long milliseconds)
{
    int millis;
    int totalSeconds;
    byte seconds;
    byte minutes;

    millis = milliseconds % 1000;       // Get milliseconds part
    totalSeconds = milliseconds / 1000; // Get total seconds
    minutes = totalSeconds / 60;        // Get minutes part
    seconds = totalSeconds % 60;        // Get seconds part (remainder after minutes)

    if (minutes == 0) // If less than a minute
    {
        char buf[8];                                // Buffer for "SS.mmm"
        sprintf(buf, "%02d.%03d", seconds, millis); // Format as SS.mmm (e.g., "05.123")
        return buf;
    }
    else // If one minute or more
    {
        char buf[11];                                             // Buffer for "MM:SS.mmm"
        sprintf(buf, "%02d:%02d.%03d", minutes, seconds, millis); // Format as MM:SS.mmm (e.g., "01:05.123")
        return buf;
    }
}

// GetTireTemps: Sends tire temperature data to Serial2 (external display/device).
void GetTireTemps()
{
    Serial2.print("3,"); // Command prefix
    Serial2.print(flTemp);
    Serial2.print(",");
    Serial2.print(frTemp);
    Serial2.print(",");
    Serial2.print(rlTemp);
    Serial2.print(",");
    Serial2.print(rrTemp);
    Serial2.println(); // End of command
}

// SetRPMBar: Controls the LED RPM bar based on current RPM.
bool SetRPMBar(int rpm)
{
    if (rpm < gate1) // RPM below first gate
    {
        if (ledBarCurrentGate != 0) // If not already in this state
        {
            ledBarCurrentGate = 0;     // Set current gate
            rpmBar.setLED(0, 0, 0, 0); // Turn off all LEDs (assuming 5 LEDs, 0-4)
            rpmBar.setLED(1, 0, 0, 0);
            rpmBar.setLED(2, 0, 0, 0);
            rpmBar.setLED(3, 0, 0, 0);
            rpmBar.setLED(4, 0, 0, 0);
            rpmBar.write(); // Update LED bar
        }
    }
    if (rpm >= gate1 && rpm < gate2) // RPM between gate1 and gate2
    {
        if (ledBarCurrentGate != 1)
        {
            ledBarCurrentGate = 1;
            rpmBar.setLED(0, 0, 0, 4095); // Turn on first LED (blue, full brightness)
            rpmBar.setLED(1, 0, 0, 0);
            rpmBar.setLED(2, 0, 0, 0);
            rpmBar.setLED(3, 0, 0, 0);
            rpmBar.setLED(4, 0, 0, 0);
            rpmBar.write();
        }
    }
    if (rpm >= gate2 && rpm < gate3) // RPM between gate2 and gate3
    {
        if (ledBarCurrentGate != 2)
        {
            ledBarCurrentGate = 2;
            rpmBar.setLED(0, 0, 0, 4095); // First two LEDs on
            rpmBar.setLED(1, 0, 0, 4095);
            rpmBar.setLED(2, 0, 0, 0);
            rpmBar.setLED(3, 0, 0, 0);
            rpmBar.setLED(4, 0, 0, 0);
            rpmBar.write();
        }
    }
    if (rpm >= gate3 && rpm < gate4) // RPM between gate3 and gate4
    {
        if (ledBarCurrentGate != 3)
        {
            ledBarCurrentGate = 3;
            rpmBar.setLED(0, 0, 0, 4095); // First three LEDs on
            rpmBar.setLED(1, 0, 0, 4095);
            rpmBar.setLED(2, 0, 0, 4095);
            rpmBar.setLED(3, 0, 0, 0);
            rpmBar.setLED(4, 0, 0, 0);
            rpmBar.write();
        }
    }
    if (rpm >= gate4 && rpm < gate5) // RPM between gate4 and gate5
    {
        if (ledBarCurrentGate != 4)
        {
            ledBarCurrentGate = 4;
            rpmBar.setLED(0, 0, 0, 4095); // First three blue, next one red
            rpmBar.setLED(1, 0, 0, 4095);
            rpmBar.setLED(2, 0, 0, 4095);
            rpmBar.setLED(3, 4095, 0, 0); // Red LED
            rpmBar.setLED(4, 0, 0, 0);
            rpmBar.write();
        }
    }
    if (rpm >= gate5 && rpm < gate6) // RPM between gate5 and gate6
    {
        if (ledBarCurrentGate != 5)
        {
            ledBarCurrentGate = 5;
            rpmBar.setLED(0, 0, 0, 4095); // First three blue, next two red
            rpmBar.setLED(1, 0, 0, 4095);
            rpmBar.setLED(2, 0, 0, 4095);
            rpmBar.setLED(3, 4095, 0, 0);
            rpmBar.setLED(4, 4095, 0, 0);
            rpmBar.write();
        }
    }
    if (rpm >= gate6) // RPM at or above gate6 (redline, blinking)
    {
        unsigned long currentMillis = millis();
        if (currentMillis - ledBarPreviousMillis > ledBarInterval)
        {                                         // Check for blinking interval
            ledBarPreviousMillis = currentMillis; // Update last blink time

            if (ledBarBlinkState == LOW) // Toggle blink state
            {
                ledBarBlinkState = HIGH;
                rpmBar.setLED(0, 0, 0, 4095); // All LEDs on (blue/red)
                rpmBar.setLED(1, 0, 0, 4095);
                rpmBar.setLED(2, 0, 0, 4095);
                rpmBar.setLED(3, 4095, 0, 0);
                rpmBar.setLED(4, 4095, 0, 0);
                rpmBar.write();
            }
            else
            {
                ledBarBlinkState = LOW;
                rpmBar.setLED(0, 0, 0, 0); // All LEDs off
                rpmBar.setLED(1, 0, 0, 0);
                rpmBar.setLED(2, 0, 0, 0);
                rpmBar.setLED(3, 0, 0, 0);
                rpmBar.setLED(4, 0, 0, 0);
                rpmBar.write();
            }
        }
        if (ledBarCurrentGate != 6) // Set current gate to 6 if not already
        {
            ledBarCurrentGate = 6;
        }
    }
    return true; // Function returns a boolean, though its value isn't used.
}

// Function to copy 'len' elements from 'src' to 'dst': Copies a specified number of characters from source to destination.
void arrCpy(char *src, char *dst, int len)
{
    memcpy(dst, src, sizeof(src[0]) * len); // Uses memcpy for efficient byte copying
}

// trying shit: Variables for serial communication with external tire temp Arduino.
const byte numChars = 32;     // Maximum number of characters in a received message
char receivedChars[numChars]; // Buffer to store received characters
char tempChars[numChars];     // Temporary buffer for parsing received data

boolean newData = false; // Flag to indicate if new data has been received via serial
// recvWithStartEndMarkers: Reads serial data until start '<' and end '>' markers are found.
void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false; // Static flag to track reception progress
    static byte ndx = 0;                   // Static index for receivedChars buffer
    char startMarker = '<';
    char endMarker = '>';
    char rc; // Received character

    while (Serial3.available() > 0 && newData == false)
    {                        // While data is available and new data flag is false
        rc = Serial3.read(); // Read a character

        if (recvInProgress == true)
        { // If reception is in progress
            if (rc != endMarker)
            { // If not end marker, store character
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                { // Prevent buffer overflow
                    ndx = numChars - 1;
                }
            }
            else
            {                              // End marker received
                receivedChars[ndx] = '\0'; // Null-terminate the string
                recvInProgress = false;    // End reception
                ndx = 0;                   // Reset index
                newData = true;            // Set new data flag
            }
        }
        else if (rc == startMarker)
        { // If start marker received, begin reception
            recvInProgress = true;
        }
    }
}

//============

// parseData: Parses the received serial data string into tire temperature variables.
void parseData()
{                     // split the data into its parts
    char *strtokIndx; // This is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // Get the first part - the string (FL temp)
    flTemp = atoi(strtokIndx);           // Convert this part to an integer

    strtokIndx = strtok(NULL, ","); // This continues where the previous call left off (FR temp)
    frTemp = atoi(strtokIndx);      // Convert this part to an integer

    strtokIndx = strtok(NULL, ","); // This continues where the previous call left off (RL temp)
    rlTemp = atoi(strtokIndx);      // Convert this part to an integer

    strtokIndx = strtok(NULL, ","); // This continues where the previous call left off (RR temp)
    rrTemp = atoi(strtokIndx);      // Convert this part to an integer
}

//============

// Main Loop
void loop()
{
    // Read tire temperatures from external Arduino via Serial3.
    recvWithStartEndMarkers(); // Check for incoming serial data
    if (newData == true)
    {                                     // If new data is available
        strcpy(tempChars, receivedChars); // Copy received data to a temporary buffer
                                          // This temporary copy is necessary to protect the original data
                                          //   because strtok() used in parseData() replaces the commas with \0
        parseData();                      // Parse the data into tire temperature variables
        newData = false;                  // Reset new data flag
    }

    // Get new CAN info: Update sensor variables from CAN bus.
    UpdateLoggers();

    if (loggerRunning) // If data logging is currently active
    {
        // Get GPS info: Check for new GPS data if logging.
        if (gps.available(gpsPort) && !newGpsData)
        {
            // If GPS data is available on its serial port and not already processed
            fix = gps.read();  // Read the GPS fix data
            newGpsData = true; // Set flag indicating new GPS data
        }
    }

    // SubLoop if arduino is put into screen off state from logger update function above
// SubLoop if arduino is put into screen off state
while (canSilent) {
    if (screenAwake) { // Turn off screen first time entering sleep
        digitalWrite(arduinoSleepPin, LOW);
        Oled.noDisplay();
        analogWrite(screenPwmPin, 0);
        screenAwake = false;
    }
    
    // Use the interrupt-driven CAN processing instead of manual pin checking
    if (canMessageAvailable || CAN0.checkReceive() == CAN_MSGAVAIL) {
        canMessageAvailable = false;
        
        if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
            if (rxId == 0x625) {
                if (rxBuf[3] == 0x90 || rxBuf[3] == 0x10) {
                    canSilent = false;
                    ledBarCurrentGate = -1; // Reset RPM bar
                    // Turn screen back on
                    Oled.display();
                    if (screenDim) {
                        analogWrite(screenPwmPin, screenDimLvl);
                    } else {
                        analogWrite(screenPwmPin, 255);
                    }
                    digitalWrite(arduinoSleepPin, HIGH);
                    screenAwake = true;
                }
            }
        }
    }
}

    // read buttons: Update the state of all physical buttons.
    UpButton.read();
    DownButton.read();
    LeftButton.read();
    RightButton.read();
    CheckUpButton();    // Handle Up button actions
    CheckDownButton();  // Handle Down button actions
    CheckLeftButton();  // Handle Left button actions
    CheckRightButton(); // Handle Right button actions

    // Draw the Arrows based off of elements and position of LgLcd
    if (menuDepth == -1)   // If currently on the logger screen (not in menus)
        UpdateScrollbar(); // Draw scrollbar arrows and logging icon

    // SD Card control code
    if (menu.enableLogging) // If logging is enabled in the menu settings
    {
        if (sdReady) // Check if SD card is initialized and ready
        {
            if (!loggerRunning) // If logger is not currently running, start it
            {
                // start sd card code and print headers
                // Check for next available file: Find the next available log file name (e.g., "01.csv", "02.csv")
                sprintf(fileString, "%02d.csv", file);
                do
                {
                    file = file + 1;
                    sprintf(fileString, "%02d.csv", file);
                } while (sd.exists(fileString)); // Loop until a non-existent file name is found

                FsDateTime::setCallback(dateTime);
                sdLog.open(fileString, FILE_WRITE); // Open the new log file for writing

                Serial.println(F("SDOPEN")); // Debug print
                // Deprecated: sdLog.print(F("Time,")); // Old header format
                sdLog.print(F("Time,")); // Print "Time" header
                // Print headers for all enabled loggers in their display order.
                for (int i = 0; i < lgLcdList.size(); i++)
                {
                    for (int ii = 0; ii < 10; ii++)
                    {
                        if (loggerIndex[ii] == i) // Find the logger corresponding to the current display order index
                        {
                            sdLog.print(loggerTexts[ii]); // Print logger name
                            sdLog.print(F(","));          // Add comma separator
                            break;
                        }
                    }
                }
                if (trapsLoaded) // If track trap data is loaded, add GPS/lap timing headers
                {
                    sdLog.print(F("Current Zone, Current Laptime, Previous Laptime, Lap Count, Latitude, Longitude"));
                }

                sdLog.println(); // New line after headers
                if (!splashInterrupt)
                { // If no splash screen, update logging icon on OLED
                    Oled.setCursor(19, 0);
                    Oled.write(byte(3)); // Draw logging icon
                }
                menu.LockLoggers();   // Lock logger settings (prevent changes while logging)
                loggerRunning = true; // Set logging flag to true
                logTime.start();      // Start overall logging timer
            }
            sdLog.print(logTime.value());
            sdLog.print(",");
        }
        else // If SD Not Ready: Handle case where SD card is not ready
        {
            Oled.clear();                           // Clear display
            PrintLcd('l', 0, 0, F("SD Not Ready")); // Display error messages
            PrintLcd('l', 0, 1, F("Check Card"));
            PrintLcd('l', 0, 3, F("Then Soft Reset"));
            menu.enableLogging = false; // Disable logging
            delay(3500);                // Display message for a few seconds
            Oled.clear();               // Clear display
        }
    }
    else // If logging is disabled in the menu settings
    {
        // Condition if logger is running but should be stopped
        if (loggerRunning) // If logger was running but is now disabled, stop it
        {
            // close sd card code
            sdLog.close();                 // Close SD log file
            Serial.println(F("SDCLOSED")); // Debug print
            menu.UnlockLoggers();          // Unlock loggers (allow user to change settings)
            logTime.stop();                // Stop overall log timer
            logTime.reset();               // Reset overall log timer
            lapTime.stop();                // Stop lap timer
            lapTime.reset();               // Reset lap timer
            Serial2.println("4,stop,,,");  // Send command to external display to stop timer
            // TODO: Stop and Reset Timer Serial
            previousLapTime = 0; // Reset lap timing variables
            laps = 0;
            zone = 0;
            onTrack = false;
            trackFile.seekSet(0);  // Rewind track file pointer
            loggerRunning = false; // Set logging flag to false
            if (!splashInterrupt)
            { // If no splash screen, clear logging icon
                Oled.setCursor(19, 0);
                Oled.print(F(" "));
            }
        }
    }

    // Start Loop to add loggers if needed and write to screeen as well as file
    for (int i = 0; i < 10; i++) // Iterate through all possible loggers (0-9)
    {
        // Check Menu class to see if logger is enabled for this index/loop
        if (menu.loggers[i]) // If this logger is enabled in the MenuMaster settings
        {
            // If it made it this far, it is enabled in the menu, if its set to -1, it not added to the lgLcdList; nor being logged. SO ADD IT
            if (loggerIndex[i] < 0) // If this logger is enabled but not yet added to the display list
            {
                // Add the logger text to the LCDList with the text in the array above (AGAIN ORDER REALLY FUCKING MATTTERS HERE)
                lgLcdList.add(loggerTexts[i]); // Add the logger's text to the display linked list
                // Set the logger index to the bottom of the logger stack
                loggerIndex[i] = (lgLcdList.size() - 1); // Assign its position in the display list
                // Increase the counter of how many loggers we have
                loggerCount++;  // Increment count of active loggers
                SaveToEEPROM(); // Save updated logger settings to EEPROM
            }

            switch (i)
            {
            case 0:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], TPS, 0));
                break;
            case 1:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], PDL, 0));
                break;
            case 2:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], BRK, 0));
                break;
            case 3:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], SPD, 0));
                break;
            case 4:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], RPM, 0));
                break;
            case 5:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], SAS, 0));
                break;
            case 6:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], ECT, 0));
                break;
            case 7:
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], FuelLevel, 0));
                break;
            case 8:
                xAxisGValue = ((float)analogRead(xpin) - zero_G_x) / scale_x;
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], xAxisGValue, 1));
                break;
            case 9:
                yAxisGValue = ((float)analogRead(ypin) - zero_G_y) / scale_y;
                lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], yAxisGValue, 1));
                break;
            default:
                break;
            }
        }
        // Goes all the way back up to the top, I'll Remind you.. This is if the logger wasnt enabled in the menu class
        else // If this logger is NOT enabled in the MenuMaster settings
        {
            // If we are in here, we should make sure that this interations logger isnt actally.. well.. logging..
            if (loggerIndex[i] >= 0) // If this logger was previously active (has a valid display index)
            {
                // WTF why are you logging GTFO
                // Revmoe the logger from the LCD list (via its index stored in loggerIndex
                lgLcdList.remove(loggerIndex[i]); // Remove the logger from the display list

                // Clean up the lists index/order; we might have just taken a chunk out of the middle of it.
                // Loop through the size of the loggerindex array (probably 10, unless i made a code change and didnt change this comment)
                for (int j = 0; j < sizeof(loggerIndex); j++) // Iterate through loggerIndex array
                {
                    // If the logger in this sub iteration is greater than the removed loggers position, move that logger (and all other loggers in this loop) down 1 spot;
                    if (loggerIndex[j] > loggerIndex[i]) // If a logger's display index is greater than the removed one
                    {
                        loggerIndex[j] -= 1; // Decrement its display index to shift it up
                    }
                }
                // Remove the item from the list (good for arrows)
                loggerCount--; // Decrement count of active loggers
                // Set the index to -1 AKA unused
                loggerIndex[i] = -1; // Mark this logger as inactive
                SaveToEEPROM();      // Save updated logger settings to EEPROM
            }
        }
    }

    // do sdlogs. pulling this out of main logger loop for better readability and thefact thecode needed to be refactored
    //  SD Loggging On and SDCard open
    if (menu.enableLogging && sdLog) // If SD logging is enabled and the log file is open
    {
        // Case for SDLogging
        if (!splashInterrupt)
        { // If no splash screen, redraw logging icon
            Oled.setCursor(19, 0);
            Oled.write(byte(3)); // redraw datalogging icon
        }
        for (int i = 0; i < lgLcdList.size(); i++)
        {
            for (int ii = 0; ii < 10; ii++)
            {
                if (loggerIndex[ii] == i) // Find the logger corresponding to the current display order index
                {
                    // sdLog.print(loggerTexts[ii]); // Print logger name
                    // sdLog.print(F(","));          // Add comma separator

                    // case statement for each of the loggers. case 0-9
                    // instead of the above swtich statement, this is only doing the sd card logging. order matters which is why i had to bust this out. writing in the correct order to the sd card is required, while the lcd logic has an index paramter.

                    switch (ii)
                    {
                    case 0:               // TPS (Throttle Position Sensor)
                        sdLog.print(TPS); // Log value to SD
                        break;
                    case 1:               // PDL (Pedal Position)
                        sdLog.print(PDL); // Log value to SD
                        break;
                    case 2:               // BRK (Brake Pressure)
                        sdLog.print(BRK); // Log value to SD
                        break;
                    case 3:               // Speed
                        sdLog.print(SPD); // Log value to SD
                        break;
                    case 4:               // RPM (Engine Revolutions Per Minute)
                        sdLog.print(RPM); // Log value to SD
                        break;
                    case 5:               // SAS (Steering Angle Sensor)
                        sdLog.print(SAS); // Log value to SD
                        break;
                    case 6:               // ECT (Engine Coolant Temperature)
                        sdLog.print(ECT); // Log value to SD
                        break;
                    case 7:                     // Fuel Level
                        sdLog.print(FuelLevel); // Log value to SD
                        break;
                    case 8:                                                           // X Axis G-Force
                        xAxisGValue = ((float)analogRead(xpin) - zero_G_x) / scale_x; // Calculate X-axis G-force
                        sdLog.print(xAxisGValue, 1);                                  // Log value to SD with 1 decimal place
                        break;
                    case 9:                                                           // Y Axis G-Force
                        yAxisGValue = ((float)analogRead(ypin) - zero_G_y) / scale_y; // Calculate Y-axis G-force
                        sdLog.print(yAxisGValue, 1);                                  // Log value to SD with 1 decimal place
                        break;
                    default:
                        sdLog.print(F("Unknown Logger")); // If an unknown logger is encountered, log a placeholder
                        break;
                    }

                    // Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
                    if (loggerIndex[ii] == (lgLcdList.size() - 1)) // If this is the last logger in the display list
                    {
                        // Adding Conditional - If no traps are loaded then dont check gps.
                        if (trapsLoaded) // If track traps are loaded, call GPS/lap timing collector
                        {
                            CollectLapTimerStats();
                        }
                        else // Otherwise, just print a new line for the next log entry
                        {
                            sdLog.println();
                        }
                    }
                    else // If not the last logger, add a comma separator
                    {
                        sdLog.print(",");
                    }
                    break;
                }
            }
        }
    }

    // clear left over open row if exists
    if (lgLcdList.size() != loggerListSize) // If the number of items in the display list has changed
    {
        ClearRow((lgLcdList.size()));      // Clear the row where the last item used to be (if list shrunk)
        loggerListSize = lgLcdList.size(); // Update the stored list size
    }

    RefreshLoggerList(); // Update the scroll position for the logger display

    // Assign menu position to menu class
    menu.SetValues(menuDepth, menu1Index, menu2Index, menu3Index); // Pass current menu state to MenuMaster
    if (splashInterrupt)
    {                  // If a splash screen is active (e.g., lap completed)
        if (laps == 0) // Special splash screen for when lap timer just started
        {
            PrintLcd('l', 0, 0, "LapTimer Started");
            PrintLcd('l', 0, 1, "Go!");
            PrintLcd('l', 0, 2, "      Haul Some Ass");
            PrintLcd('l', 0, 3, "   Dont Crash!");
        }
        else // Splash screen for completed laps
        {
            PrintLcd('l', 0, 0, "   Lap Completed!   ");
            PrintLcd('l', 0, 1, "Lap: ");
            PrintLcd('l', 0, 2, "Time: ");

            char buf[10]; // Buffer for lap number
            sprintf(buf, "%d", laps);
            PrintLcd('l', 5, 1, buf); // Print lap number
            buf[0] = {0};             // Clear buffer

            PrintLcd('l', 6, 2, FormatMilliseconds(LapTime)); // Print current lap time

            if (previousLapTime != 0) // If there was a previous lap time, calculate and display difference
            {
                long diffTime = LapTime - previousLapTime; // Calculate time difference
                char sign;
                if (diffTime < 0) // If current lap was faster
                {
                    diffTime = -diffTime; // Make difference positive
                    sign = '-';           // Set sign to negative
                }
                else // If current lap was slower or same
                {
                    sign = '+'; // Set sign to positive
                }

                // Format difference with sign and milliseconds
                sprintf(buf, "%c%s", sign, FormatMilliseconds(diffTime).c_str());

                PrintLcd('l', 9, 3, buf);   // Print lap time difference
                arrCpy(buf, splitTime, 10); // Copy difference to splitTime buffer (for Serial2)
            }
        }
        unsigned int currentMillis = millis();
        if (currentMillis - oldMillis >= lapSplashTime) // Check if splash screen duration has passed
        {
            Oled.clear();              // Clear display
            oldMillis = currentMillis; // Reset timer
            splashInterrupt = false;   // Deactivate splash screen
        }
    }
    else
    {                 // If no splash screen is active, display either menu or logger data
                      // Setting RPM Bar - doing it in here to speed up code
        if (RPM == 0) // If RPM is 0, use pedal position to estimate RPM for bar display
        {
            SetRPMBar(map(PDL, 0, 100, 4000, 6500)); // Map pedal position (0-100) to RPM range (4000-6500)
        }
        else // Otherwise, use actual RPM
        {
            SetRPMBar(RPM);
        }

        // Read the tire temps
        GetTireTemps(); // Send tire temperature data to Serial2

        if (menuDepth != -1) // If currently in a menu (not logger screen)
        {
            if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST) // If current menu is a LIST type
            {
                PrintLcd('l', 0, 0, menu.GetLine1Text()); // Print all four lines of the list
                PrintLcd('l', 0, 1, menu.GetLine2Text());
                PrintLcd('l', 0, 2, menu.GetLine3Text());
                PrintLcd('l', 0, 3, menu.GetLine4Text());
            }
            else
            { // For other menu types (UPDOWN, ONOFF, CALLBACK)
                // Print line 1 centered, and line 2
                PrintLcd('l', (((20 / 2) - ((menu.GetLine1Text().length()) / 2))), 0, menu.GetLine1Text());
                PrintLcd('l', 0, 1, menu.GetLine2Text());
            }
        }
        else // If currently on the logger screen (menuDepth == -1)
        {
            // Delay for LCD Screen Readability: Introduce a delay for readability of logger data.
            unsigned int currentMillis = millis();
            if (currentMillis - oldMillis >= lcdInterval) // Check if refresh interval has passed
            {
                oldMillis = currentMillis; // Reset timer
                // Print the current 4 lines of logger data from the linked list, adjusted by scroll modifier
                PrintLcd('l', 0, 0, lgLcdList.get(0 + menuModifier));
                PrintLcd('l', 0, 1, lgLcdList.get(1 + menuModifier));
                PrintLcd('l', 0, 2, lgLcdList.get(2 + menuModifier));
                PrintLcd('l', 0, 3, lgLcdList.get(3 + menuModifier));
            }
        }
    }
    // Serial.println(FreeRam()); // Debug print for free RAM (commented out)
}
