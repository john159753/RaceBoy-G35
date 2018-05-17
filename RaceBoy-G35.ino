/*
 Name:    RaceBoy-G35.ino
 Created: 1/31/2016 5:23:27 PM
 Updated: 9/6/2017
 Author:  John Daley

 Most block comments are due to lab testing and no canbus to watch data
 */

 // include the library code:
#include <EEPROM.h>
#include <Adafruit_CharacterOLED.h>
#include <avr/pgmspace.h>
#include <Button.h>
#include <mcp_can.h>
#include <SPI.h>
#include <LinkedList.h>
#include <SdFat.h>
#include "MenuMaster.h"
#include "Stopwatch.h"
//GPS Libs
#include <NMEAGPS.h>
#include <GPSport.h>


// STUFF FOR SOFTRESET
void(*resetFunc) (void) = 0; //declare reset function @ address 0

//MenuMaster Defines
#define BLOCK -1
#define NOTUSED 0
#define UPDOWN 1
#define ONOFF 2
#define CALLBACK 3
#define LIST 4

//Define Button Debounce Time
#define DEBOUNCE_MS 20
//Define Long Press Duration
#define LONG_PRESS 1000
//LCD Refresh Rate
#define lcdInterval 130       // interval at which to refresh LCD for logger screens
#define loggerInterval 1         // interval at which to refresh LCD (milliseconds)
//Calibrations and vars for Accl
#define zero_G_x 510
#define zero_G_y 507
#define zero_G_z 520
#define scale_x 18
#define scale_y 19
#define scale_z 20 
//Accel Pin Declare
#define xpin A10                  // x-axis of the accelerometer
#define ypin A11                  // y-axis
#define zpin A12                  // z-axis 
//Define SPI speed for SDCard Class
#define SPI_SPEED SD_SCK_MHZ(50)
// Define for GPS
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

//Used for setting pausing without halting thread
unsigned long oldMillis = 0;

// initialize the OLED object
Adafruit_CharacterOLED Oled(OLED_V2, 33, 32, 34, 35, 36, 37, 38);

// initialize the Button Objects
Button UpButton = Button(31, true, true, DEBOUNCE_MS);
Button DownButton = Button(29, true, true, DEBOUNCE_MS);
Button LeftButton = Button(28, true, true, DEBOUNCE_MS);
Button RightButton = Button(30, true, true, DEBOUNCE_MS);

//Encoder Vars / Screen Scroller
bool loggerIndexChanged = false;
byte lastLoggerIndexPosition = 0;
byte newLoggerIndexPosition = 0;
int loggerCount; //For Arrow Drawing code
//Init Menu Position Vars
int menu1Index = 0;
int menu2Index = 0;
int menu3Index = 0;
int menuDepth = 0;

//Init the menu object
MenuMaster menu(menuDepth, menu1Index, menu2Index, menu3Index);
int *menuSize = menu.GetMenuSize(); //pointer for array of menu size to calculate menu limits
//init linked list
LinkedList<String> lgLcdList = LinkedList<String>();
//init accel values
float xAxisGValue;
float yAxisGValue;
//Declare custom LCD Characters for Scrolling LCD
byte upArrow[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b10001,
  0b00100,
  0b00100,
  0b00100
};
byte dnArrow[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b10001,
  0b10001,
  0b01010,
  0b01010,
  0b00100
};
byte pipe[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};
byte logIcon[8] = {
  0b11000,
  0b10100,
  0b10100,
  0b11000,
  0b00100,
  0b00100,
  0b00100,
  0b00111
};

bool longpressUp = false; //long press bool
// Create CANBUS reader object
MCP_CAN CAN0(9);
//Declare CANBUS vars
long unsigned int rxId; //Var for holding PID of incoming frame
unsigned char len = 0; //Var for showing frame length in bytes
unsigned char rxBuf[8]; //Array for incoming frame buffer
byte retry;
//Declare temp data holders
int TPS = 0;
int PDL = 0;
int RPM = 0;
int SPD = 0;
int BRK = 0;
int SAS = 0;
int MAP = 0;
bool canSilent = false; //Should be true is no activity on canbus for 100 loop cycles
bool screenAwake = true;
bool screenDim = false;
bool recvA = false;
bool recvB = false;
bool recvC = false;
bool recvD = false;
int tmpA;
int tmpB;
int loggerListSize = 0;



//const char* const PROGMEM menuTable[9] = {(pgm_read_word(&(MAP))),(pgm_read_word(&(TPS))),(pgm_read_word(&(RPM))),(pgm_read_word(&(speed))),(pgm_read_word(&(IAT))),(pgm_read_word(&(ECT))),(pgm_read_word(&(advance))),(pgm_read_word(&(brake))),"Accel"};
//Define **IN SAME ORDER AS MENU** what should be displayed on logger screen - Must be 10 char EXACTLY - ??? means i dont know what the value actually means, but it moves with engine
const char* loggerTexts[10] = { "Throttle  ", "Gas Pedal ", "Brake     ", "Speed     ", "Engine RPM", "Turn Angle", "???       ", "Empty     ", "Lateral-G ", "Long-G    " };
char loggerIndex[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
char menuModifier;

//Inits for SDCard Logging
//OLD LIB: File sdLog;
SdFat sd;
SdFile sdLog;
SdFile trackFile;
bool loggerRunning;
int file = 0;
char fileString[12];
byte sd_cs_pin = 4;
StopWatch logTime;
StopWatch lapTime;
bool sdReady = false;
// Maximum line length plus space for zero byte.
const size_t LINE_DIM = 100;
char line[LINE_DIM];
size_t n;

//GPSVariable Definitions
static NMEAGPS  gps;
static gps_fix  fix;

//LapTimer Variable Definitions
int zone = 0;
int maxZones = 0;
int laps = 0;
long latitudeTraps[4];
long longitudeTraps[4];
String zoneName;
bool trapsLoaded = false;
bool onTrack = false;


void setup() {
	Serial.begin(115200);         //Begin Serial Out
	// OLD Deprecated Start Function CAN0.begin(CAN_500KBPS);      // Begin can bus : baudrate = 500k
	if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
		Serial.println("MCP2515 Initialized Successfully!");
	else
		Serial.println("Error Initializing MCP2515...");
	CAN0.setMode(MCP_NORMAL);
	pinMode(2, INPUT);            //Setting pin 2 for /INT input

	// set up the LCDs number of columns and rows
	//lcdSm.begin(16, 2);
	Oled.begin(20, 4);

	//Setup Accel
	pinMode(xpin, INPUT);
	pinMode(ypin, INPUT);
	pinMode(zpin, INPUT);
	analogReference(EXTERNAL); //Look to ARef pin for analog reference voltage

	//create custom lcd objects
	Oled.createChar(0, upArrow);
	Oled.createChar(1, dnArrow);
	Oled.createChar(2, pipe);
	Oled.createChar(3, logIcon);
	delay(100);


	CAN0.init_Mask(0, 0, 0x189C0000);
	CAN0.init_Filt(0, 0, 0x18940000);
	CAN0.init_Filt(1, 0, 0x00080000);

	CAN0.init_Mask(1, 0, 0x1FFC0000);
	CAN0.init_Filt(2, 0, 0x08F40000);
	CAN0.init_Filt(3, 0, 0x1E480000);
	CAN0.init_Filt(4, 0, 0x0B440000);
	CAN0.init_Filt(5, 0, 0x0B440000);


	//Setup SD Card
	pinMode(sd_cs_pin, OUTPUT);

	if (!sd.begin(sd_cs_pin, SPI_SPEED))
	{
		retry = 0;
		while (retry < 2)
		{
			retry++;
			if (sd.begin(sd_cs_pin, SPI_SPEED))
			{
				Serial.println(F("Card Ready"));
				sdReady = true;
				break;
			}
		}
		if (!sdReady)
		{
			sdReady = false;
			Serial.println(F("Card Failure"));
		}
	}
	else
	{
		sdReady = true;
		Serial.println(F("Card Ready"));
	}


	//Read from EEPROM and set vars up
	ReadFromEEPROM();
	//Turn LCD to full Bright
	Oled.command(0x17);

}

//Go Home Function - Resets menu position
void GoHome()
{

	Oled.clear();

	menuDepth = 0;
	menu1Index = 0;
	menu2Index = 0;
	menu3Index = 0;


}
//button up code
void CheckUpButton()
{
	if (UpButton.wasReleased() && !longpressUp)
	{

		Oled.clear();
		if (menu.IsSelectable()) // Check if YesNo Menu
		{
			menu.SelectUp();
		}
		else if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST) // Check if YesNo Menu
		{
			menu.SelectUp();
			menu.LoadTracks(sd, trackFile);
		}
		else if (!menu.IsSelectable())  // Check if YesNo Menu
		{
			switch (menuDepth) {
			case -1:
				//Code for loggerup
				newLoggerIndexPosition--;
			case 0:
				if (menu1Index >= (*(menuSize + 0) - 1) || menu.GetMenuType((menu1Index + 1), menu2Index, menu3Index) == BLOCK) {} //The Index +1 is a look ahead to see if menu item can be accessed
				else //Sets Limits for menu bounds
					menu1Index++;
				break;
			case 1:
				if (menu2Index >= *(menuSize + 1) || menu.GetMenuType(menu1Index, (menu2Index + 1), menu3Index) == BLOCK) {} //The Index +1 is a look ahead to see if menu item can be accessed
				else //Sets Limits for menu bounds
					menu2Index++;
				break;
			case 2:
				if (menu3Index >= *(menuSize + 2) || menu.GetMenuType(menu1Index, menu2Index, (menu3Index + 1)) == BLOCK) {} //The Index +1 is a look ahead to see if menu item can be accessed
				else //Sets Limits for menu bounds
					menu3Index++;
				break;
			case 3:
				//code
				break;
			default:
				//code
				break;
			}
		}
	}
	longpressUp = false;

	//button up longpress
	if (UpButton.pressedFor(LONG_PRESS)) {
		GoHome();
		menu.SetLoggersTrue();
		longpressUp = true;

	}

}
//button down code
void CheckDownButton()
{
	if (DownButton.wasReleased())
	{
		Oled.clear();
		if (menu.IsSelectable()) // Check if YesNo Menu
		{
			menu.SelectDown();
		}
		else if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST) // Check if YesNo Menu
		{
			menu.SelectDown();
			menu.LoadTracks(sd, trackFile);
		}
		else if (!menu.IsSelectable()) // Check if YesNo Menu
		{
			switch (menuDepth) {
			case -1:
				//Code for loggerdown
				newLoggerIndexPosition++;
			case 0:
				if (menu1Index == 0) {}
				else { //Sets Limits for menu bounds
					menu1Index--;
				}
				break;
			case 1:
				if (menu2Index == 1) {}
				else { //Sets Limits for menu bounds
					menu2Index--;
				}
				break;
			case 2:
				if (menu3Index == 1) {}
				else { //Sets Limits for menu bounds
					menu3Index--;
				}
				break;
			case 3:
				//code
				break;
			default:
				//code
				break;
			}
		}
	}
}
//button left code
void CheckLeftButton()
{
	if (LeftButton.wasReleased())
	{
		Oled.clear();
		if (menu.IsSelectable()) // Check if YesNo Menu
		{
			switch (menuDepth) {
			case 0:
				menu1Index = 0;
				break;
			case 1:
				menu2Index = 0; //Go to index zero to selected menu
				break;
			case 2:
				menu3Index = 0; //Go to index zero to selected menu
				break;
			case 3:
				//code
				break;
			default:
				//code
				break;
			}
			if (menuDepth <= -1) // If at level zero menu
			{
				//DO NOTHING BECAUSE AINT SHIT HERE
			}
			else
			{
				menuDepth--;
			}

		}
		else if (!menu.IsSelectable())
		{
			switch (menuDepth) {
			case 0:

				break;
			case 1:
				menu2Index = 0; //Go to index zero to selected menu
				break;
			case 2:
				menu3Index = 0;//Go to index zero to selected menu
				break;
			case 3:
				//code
				break;
			default:
				//code
				break;
			}
			if (menuDepth <= -1) // If at level zero menu
			{
				//DO NOTHING BECUZ SHIT CANT EXIST LOWER
			}
			else
			{
				menuDepth--; //else to back a menu
			}
		}
	}
}
//button right code
void CheckRightButton()
{
	if (RightButton.wasReleased()) {
		Oled.clear();
		if (menu.IsSelectable()) // Check if YesNo (selectable) Menu
		{
			menu3Index = 0;
			menuDepth--; // Go back a menu
		}
		else if (!menu.IsSelectable()) // Check if YesNo (selectable) Menu
		{
			if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == CALLBACK)// Check if the menu calls back to a subroutine
			{
				switch (menuDepth) {
				case -1:
					break;
				case 0:
					if (menu1Index == 2)
					{
						//This shouldnt be called as menu1index2 isnt a callback
					}
					else if (menu1Index == 3)
					{

						//Do The Soft reset
						resetFunc();
					}
					GoHome();
					//code
					break;
				case 1:
					GoHome();
					break;
				case 2:
					GoHome();
					//code
					break;
				case 3:
					GoHome();
					//code
					break;
				default:
					GoHome();
					//code
					break;
				}
			}
			else if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST)
			{
				switch (menuDepth) {
				case -1:
					break;
				case 0:
					break;
				case 1:
					if (menu1Index == 2)
					{
						if ((menu.trackSelectIndex + menu.trackReadIndex) == 0)
						{
							GoHome();
							break;
						}
						//Do the List Logic
						byte k = 0;
						if (!sd.chdir("Tracks")) {
							//error("chdir failed for track traps folder.\n");
						}

						Serial.print("Selecting: ");
						Serial.println((menu.trackSelectIndex + menu.trackReadIndex));
						while (trackFile.openNext(sd.vwd(), O_READ)) {
							k++;
							if (k == (menu.trackSelectIndex + menu.trackReadIndex))
							{
								Serial.print("Selected Track: ");
								trackFile.printName(&Serial);

								Serial.println();

								break;
							}
							trackFile.close();
						}
						sd.chdir("/");
						sd.vwd()->rewind();

						menuDepth--;
						menu2Index = 0;

					}
					else if (menu1Index == 3)
					{
						//Wont be called
						GoHome();
					}
					else
					{
						GoHome();
					}

					//code
					break;
				case 2:
					if (menu1Index == 2)
					{

						GoHome();

					}
					else if (menu1Index == 3)
					{
						//Wont be called
						GoHome();
					}
					else
					{
						GoHome();
					}
					break;
				case 3:
					GoHome();
					//code
					break;

				default:
					GoHome();
					//code
					break;
				}
			}
			else
			{
				if (menuDepth > 2) {}
				else // limit to menu size
					menuDepth++; //Progress to next menu depth

				switch (menuDepth) {
				case -1:
					break;
				case 0:
					menu1Index = 0;
					//code
					break;
				case 1:
					menu2Index = 1;

					if (menu1Index == 2)
					{
						trackFile.close();
						//Do the List Logic
						byte k = 0;
						if (!sd.chdir("Tracks")) {
							Serial.println(F("chdir failed for track traps folder."));
						}

						while (trackFile.openNext(sd.vwd(), O_READ)) {
							k++;
							Serial.print(k, DEC);
							Serial.print(". ");
							trackFile.printName(&Serial);

							Serial.println();
							trackFile.close();
						}
						sd.chdir("/");
						sd.vwd()->rewind();

						menu.trackCount = k;
						menu.trackSelectIndex = 0;
						menu.trackReadIndex = 0;
						menu.LoadTracks(sd, trackFile);
						
					}

					break;
				case 2:
					menu3Index = 1;
					break;
				case 3:
					//code
					break;
				default:
					//code
					break;
				}
			}
		}
	}

}
//Clear a row of logger screen
void ClearRow(int rowNum)
{
	Oled.setCursor(0, rowNum);
	Oled.print(F("                   "));
}
//Code to print to lcd screens
void PrintLcd(char screen, int col, int row, String message)
{

	switch (screen)
	{
	case 's':
		Oled.print(F("No sm LCD in this version"));
		break;
	case 'l':
		Oled.setCursor(col, row);
		Oled.print(message);
		break;
	default:
		Oled.print(F("Some Fatal Error"));
	}
}
// Returns Free Ram of Arduino - Used for debugging
int FreeRam()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
//Updates Large LCD position based on menu modifier number
void RefreshLoggerList()
{
	//Large LCD Scrolling Code to set menu modifier
	if (newLoggerIndexPosition != lastLoggerIndexPosition) {
		lastLoggerIndexPosition = newLoggerIndexPosition;
		if ((newLoggerIndexPosition < (lgLcdList.size() - 3)) && newLoggerIndexPosition >= 0)
		{
			menuModifier = newLoggerIndexPosition;
			loggerIndexChanged = true;
		}
		else if ((newLoggerIndexPosition >= (lgLcdList.size() - 3)))
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
//Overloaded for custom linked list sizes (not really used)
void RefreshLoggerList(byte size)
{
	//Large LCD Scrolling Code to set menu modifier
	if (newLoggerIndexPosition != lastLoggerIndexPosition) {
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

//Draw Arrow for more than 4 loggers
void UpdateScrollbar()
{

	if (loggerCount > (4 + menuModifier))
	{

		Oled.setCursor(19, 2);
		Oled.write(byte(2));
		Oled.setCursor(19, 3);
		Oled.write(byte(1));
	}
	else
	{
		Oled.setCursor(19, 2);
		Oled.print(" ");
		Oled.setCursor(19, 3);
		Oled.print(" ");
	}
	if (loggerCount > 4 && menuModifier > 0)
	{

		Oled.setCursor(19, 0);
		if (loggerRunning)
		{
			Oled.write(byte(3));
		}
		else
		{
			Oled.write(byte(0));
		}
		Oled.setCursor(19, 1);
		Oled.write(byte(2));
	}
	else
	{
		Oled.setCursor(19, 0);
		if (loggerRunning)
		{

			Oled.write(byte(3));
		}
		else
		{
			Oled.print(" ");
		}
		Oled.setCursor(19, 1);
		Oled.print(" ");
	}
}
//Returns digit length
int GetIntLength(int number)
{
	int digits = 0;
	if (number < 0) digits = 1; // remove this line if '-' counts as a digit
	if (number != 0)
	{
		while (number) {

			number /= 10;

			digits++;
		}
	}
	else
	{
		return 1;
	}
	return digits;
}
//Formats The Large LCD String
String FormatString(String text, float value, byte pres)
{
	String output;
	byte strLEN = text.length();
	int roundVal;
	if (pres == 0)
	{
		roundVal = round(value);
		char valLEN = GetIntLength(roundVal);

		String endBuf;
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


		output = text + F("  ") + roundVal + endBuf;
	}
	else
	{

		String endBuf;
		if (GetIntLength((int)value) == 1)
		{
			if (value < 0)
			{
				endBuf = F(" ");
			}
			else
			{
				endBuf = F("  ");
			}
		}
		//aint no way im hitting 100 g's so this works fine
		else
		{
			if (value < 0)
			{
				endBuf = F("");
			}
			else
			{
				endBuf = F(" ");
			}
		}
		value = (int)(value * 100);
		value = (float)(value / 100);
		output = text + F("  ") + value + endBuf;


	}
	return output;
}

//Updates Logger Variables with current CANBUS data
void UpdateLoggers()
{

	recvA = false;
	recvB = false;
	recvC = false;
	recvD = false;
	do {


		if (!digitalRead(2))                         // If pin 2 is low, read receive buffer
		{

			if (!screenAwake)
			{
				Oled.display();
				screenAwake = true;
			}
			CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
			//OLD Depricated rxID method rxId = CAN0.getCanId();                    // Get message ID

			if (rxId == 0x792 && !recvA)
			{//Brake Light and individual wheel speeds
				BRK = ((rxBuf[2] * 100) / 255);
				recvA = true;
			}
			if (rxId == 0x2D1 && !recvB)
			{//Contains maybe MAP?
				tmpA = 0;
				tmpB = 0;
				tmpA = rxBuf[2] * 256;
				tmpB = rxBuf[1];
				SPD = ((tmpA + tmpB) / 16);
				MAP = rxBuf[5];
				recvB = true;
			}
			if (rxId == 0x23D && !recvC)
			{//Two TPS and two byte RPM
				tmpA = 0;
				tmpB = 0;
				tmpA = rxBuf[4] * 256;
				tmpB = rxBuf[3];
				RPM = ((tmpA + tmpB) * 3.15);
				TPS = ((rxBuf[2] * 100) / 255);
				PDL = ((rxBuf[1] * 100) / 255);
				recvC = true;
			}
			if (rxId == 0x2 && !recvD)
			{//Steering Angle Sensor and velocity
				tmpA = 0;
				tmpB = 0;
				tmpB = rxBuf[1] * 256;
				tmpA = rxBuf[0];
				tmpA = (tmpA + tmpB);
				if (tmpA > 32767)
					SAS = -(65535 - tmpA / 10);
				else
					SAS = tmpA / 10;
				recvD = true;
			}
			if (rxId == 0x625)
			{
				if (rxBuf[3] != 0x90 && rxBuf[3] != 0x10)
				{
					canSilent = true;
				}
			}
			if (rxId == 0x625)
			{
				// Reading with bit AND operator for masking the whole byte to just the three bits we care about for this rxId 
				if ((rxBuf[1] & 0x70) == 0x0)
				{
					if (screenDim)
					{
						Oled.command(0x17);


						screenDim = false;
					}

				}
				else
				{
					if (!screenDim)
					{

						Oled.command(0x13);
						screenDim = true;
					}

				}
			}
		}
	} while (!recvA && !recvB && !recvC && !recvD);
	/*

	BRK = random(10000);
	SPD = random(10000);
	MAP = random(10000);
	TPS = random(10000);
	RPM = random(10000);
	PDL = random(10000);
	SAS = random(10000);

  */

  /*
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

}

//Save logger states and indexes to EEPROM
void SaveToEEPROM()
{
	// Get Array Sizes for EEPROM write loops
	int menuLoggersSize = sizeof(menu.loggers) / sizeof(bool);
	int loggerIndexSize = sizeof(loggerIndex) / sizeof(char);
	//For Each item in the T/F logger state; write to eeprom
	for (int i = 0; i < menuLoggersSize; i++)
		EEPROM.write(i, menu.loggers[i]);
	// Continuing the EEPROM addresses from above, write the logger index values
	for (int i = menuLoggersSize; i < (loggerIndexSize + menuLoggersSize); i++)
		EEPROM.write(i, loggerIndex[i - menuLoggersSize]);
}
// Read logger states and positions from EEPROM
void ReadFromEEPROM()
{
	//Not sure if i really need this, but it helps it from reading trash if nothing is loaded into eeprom (Good place to do data validation if i get around to it)
	if (EEPROM.read(0) != 0xff)
	{
		// Get Array Sizes for EEPROM write loops
		int menuLoggersSize = sizeof(menu.loggers) / sizeof(bool);
		int loggerIndexSize = sizeof(loggerIndex) / sizeof(char);
		for (int i = 0; i < menuLoggersSize; i++)
			menu.loggers[i] = EEPROM.read(i);
		for (int i = menuLoggersSize; i < (loggerIndexSize + menuLoggersSize); i++)
			loggerIndex[i - menuLoggersSize] = EEPROM.read(i);
	}
	// Add loggers in on read.Unlike the loop add/remove logic, these already have predefined indexes and arent -1. They just needed to be added to the lcd linked list 
	for (int i = 0; i < 10; i++)
	{
		//Check Menu class to see if logger is enabled for this index/loop
		if (menu.loggers[i])
		{
			// If it is greater than -1; because we are reading the last state from EEPROM
			if (loggerIndex[i] >= 0)
			{
				//Add the logger text to the LCDList and increase the listAmt (for arrow code)
				lgLcdList.add(loggerIndex[i], loggerTexts[i]);
				loggerCount++;

			}
		}
	}

}

// Main Loop
void loop() {



	UpdateLoggers();

	//SubLoop if arduino is put into screen off state from logger update function above
	while (canSilent)
	{

		if (screenAwake)
		{
			Oled.noDisplay();
			screenAwake = false;
		}
		if (!digitalRead(2))                         // If pin 2 is low, read receive buffer
		{
			CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
			//OLD Depricated rxID method rxId = CAN0.getCanId();                    // Get message ID
			if (rxId == 0x625)
			{
				if (rxBuf[3] == 0x90 || rxBuf[3] == 0x10)
				{
					canSilent = false;
				}
			}

		}

	}

	//read buttons
	UpButton.read();
	DownButton.read();
	LeftButton.read();
	RightButton.read();
	CheckUpButton();
	CheckDownButton();
	CheckLeftButton();
	CheckRightButton();

	//Draw the Arrows based off of elements and position of LgLcd
	if (menuDepth == -1)//Not in menus
		UpdateScrollbar();

	//SD Card control code
	if (menu.enableLogging)
	{
		if (sdReady) //checks to see if SD is ready
		{
			if (!loggerRunning)
			{
				//start sd card code and print headers
				//Check for next available file
				sprintf(fileString, "%02d.csv", file);
				do
				{
					file = file + 1;
					sprintf(fileString, "%02d.csv", file);
				} while (sd.exists(fileString));
				sdLog.open(fileString, FILE_WRITE);

				Serial.println(F("SDOPEN"));
				//Deprecated: sdLog.print(F("Time,"));
				sdLog.print(F("Time,"));
				for (int i = 0; i < lgLcdList.size(); i++)
				{
					for (int ii = 0; ii < 10; ii++)
					{
						if (loggerIndex[ii] == i)
						{
							sdLog.print(loggerTexts[ii]);
							sdLog.print(F(","));
							break;
						}

					}

				}
				sdLog.println();
				Oled.setCursor(19, 0);
				Oled.write(byte(3));
				menu.LockLoggers();
				loggerRunning = true;
				logTime.start();
			}
			sdLog.print(logTime.value());
			sdLog.print(",");
		}
		else //If SD Not Ready
		{
			Oled.clear();
			PrintLcd('l', 0, 0, F("SD Not Ready"));
			PrintLcd('l', 0, 1, F("Check Card"));
			PrintLcd('l', 0, 3, F("Then Soft Reset"));
			menu.enableLogging = false;
			delay(3500);
			Oled.clear();
		}
	}
	else
	{
		//Condition if logger is running but should be stopped
		if (loggerRunning)
		{
			//close sd card code
			sdLog.close();
			Serial.println(F("SDCLOSED"));
			menu.UnlockLoggers();
			logTime.stop();
			logTime.reset();
			loggerRunning = false;
			Oled.setCursor(19, 0);
			Oled.print(F(" "));
		}
	}

	//Start Loop to add loggers if needed and write to screeen as well as file
	for (int i = 0; i < 10; i++)
	{
		//Check Menu class to see if logger is enabled for this index/loop
		if (menu.loggers[i])
		{
			//If it made it this far, it is enabled in the menu, if its set to -1, it not added to the lgLcdList; nor being logged. SO ADD IT
			if (loggerIndex[i] < 0)
			{
				//Add the logger text to the LCDList with the text in the array above (AGAIN ORDER REALLY FUCKING MATTTERS HERE)
				lgLcdList.add(loggerTexts[i]);
				//Set the logger index to the bottom of the logger stack
				loggerIndex[i] = (lgLcdList.size() - 1);
				//Increase the counter of how many loggers we have
				loggerCount++;
				SaveToEEPROM();
			}
			//SD Loggging On and SDCard open
			if (menu.enableLogging && sdLog)
			{
				//Case for SDLogging

				Oled.setCursor(19, 0);
				Oled.write(byte(3)); //redraw datalogging icon

				//Switch statement for each of the 10 possible loggers in this loop - NOTE YOU CANT GET TO THIS SPOT OF CODE UNLESS THE LOGGER WAS ENABLED IN THE MENU CLASS
				switch (i) {
				case 0:
					// Code for TPS
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], TPS, 0));
					sdLog.print(TPS);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 1:
					// Code for PDL
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], PDL, 0));
					sdLog.print(PDL);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 2:
					// Code for BRK
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], BRK, 0));
					sdLog.print(BRK);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 3:
					// Code for Speed
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], SPD, 0));
					sdLog.print(SPD);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 4:
					// Code for RPM
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], RPM, 0));
					sdLog.print(RPM);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 5:
					// Code for SAS
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], SAS, 0));
					sdLog.print(SAS);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 6:
					// Code for Timing Advance
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], MAP, 0));
					sdLog.print(MAP);
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 7:
					// Code for NULL ( nothing yet)
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], 0, 0));
					sdLog.print("Off");
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 8:
					// Code for Lat G
					xAxisGValue = ((float)analogRead(xpin) - zero_G_x) / scale_x;
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], xAxisGValue, 1));
					sdLog.print(xAxisGValue);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				case 9:
					// Code for Long G
					yAxisGValue = ((float)analogRead(ypin) - zero_G_y) / scale_y;
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], yAxisGValue, 1));
					sdLog.print(yAxisGValue);
					//Logic to check if this is the last object in the loop (since the order is based off the LoggerIndex (this is the same logic in how the csv headers are generated)
					if (loggerIndex[i] == (lgLcdList.size() - 1))
					{
						sdLog.println();
					}
					else
					{
						sdLog.print(",");
					}
					break;
				default:
					// Code
					break;
				}
			}
			// SD Logging Off
			else
			{
				switch (i) {
				case 0:
					// Code for TPS

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], TPS, 0));
					break;
				case 1:
					// Code for PDL

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], PDL, 0));
					break;
				case 2:
					// Code for BRK

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], BRK, 0));
					break;
				case 3:
					// Code for SPD

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], SPD, 0));
					break;
				case 4:
					// Code for RPM

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], RPM, 0));
					break;
				case 5:
					// Code for SAS

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], SAS, 0));
					break;
				case 6:
					// Code for MAP

					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], MAP, 0));
					break;
				case 7:
					// Code for NONE
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], 0, 0));
					break;
				case 8:
					// Code for Lat G
					xAxisGValue = ((float)analogRead(xpin) - zero_G_x) / scale_x;
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], xAxisGValue, 1));
					break;
				case 9:
					// Code for Long G
					yAxisGValue = ((float)analogRead(ypin) - zero_G_y) / scale_y;
					lgLcdList.set(loggerIndex[i], FormatString(loggerTexts[i], yAxisGValue, 1));

					break;
				default:
					// Code
					break;
				}
			}
		}
		// Goes all the way back up to the top, I'll Remind you.. This is if the logger wasnt enabled in the menu class
		else
		{
			// If we are in here, we should make sure that this interations logger isnt actally.. well.. logging..
			if (loggerIndex[i] >= 0)
			{
				// WTF why are you logging GTFO
				//Revmoe the logger from the LCD list (via its index stored in loggerIndex
				lgLcdList.remove(loggerIndex[i]);

				//Clean up the lists index/order; we might have just taken a chunk out of the middle of it.
				//Loop through the size of the loggerindex array (probably 10, unless i made a code change and didnt change this comment)
				for (int j = 0; j < sizeof(loggerIndex); j++)
				{
					//If the logger in this sub iteration is greater than the removed loggers position, move that logger (and all other loggers in this loop) down 1 spot; 
					if (loggerIndex[j] > loggerIndex[i])
					{
						loggerIndex[j] -= 1;
					}
				}
				// Remove the item from the list (good for arrows)
				loggerCount--;
				// Set the index to -1 AKA unused
				loggerIndex[i] = -1;
				SaveToEEPROM();
			}
		}
	}

	//clear left over open row if exists
	if (lgLcdList.size() != loggerListSize)
	{
		ClearRow((lgLcdList.size()));
		loggerListSize = lgLcdList.size();

	}


	RefreshLoggerList();



	//Assign menu position to menu class
	menu.SetValues(menuDepth, menu1Index, menu2Index, menu3Index);

	if (menuDepth != -1)
	{
		if (menu.GetMenuType(menu1Index, menu2Index, menu3Index) == LIST)
		{
			PrintLcd('l', 0, 0, menu.GetLine1Text());
			PrintLcd('l', 0, 1, menu.GetLine2Text());
			PrintLcd('l', 0, 2, menu.GetLine3Text());
			PrintLcd('l', 0, 3, menu.GetLine4Text());
		}
		else {
			PrintLcd('l', (((20 / 2) - ((menu.GetLine1Text().length()) / 2))), 0, menu.GetLine1Text());
			PrintLcd('l', 0, 1, menu.GetLine2Text());
		}
	}

	else
	{
		//Delay for LCD Screen Readability
		unsigned int currentMillis = millis();
		if (currentMillis - oldMillis >= lcdInterval)
		{
			oldMillis = currentMillis;
			PrintLcd('l', 0, 0, lgLcdList.get(0 + menuModifier));
			PrintLcd('l', 0, 1, lgLcdList.get(1 + menuModifier));
			PrintLcd('l', 0, 2, lgLcdList.get(2 + menuModifier));
			PrintLcd('l', 0, 3, lgLcdList.get(3 + menuModifier));
		}
	}
	//Serial.println(FreeRam());


}

