#include "MenuMaster.h"
#include <WString.h>
//#include <avr/pgmspace.h>

#define UPDOWN 1
#define ONOFF 2
#define CALLBACK 3

using namespace std;
/*
 * Define Menu texts below - max size is 20
 * This is not the same as the logger screen (Defined in main file)
 */
const char MenuMaster::TL1[] = { "Parameters" };
const char MenuMaster::TL1S1[] = { "TPS" };
const char MenuMaster::TL1S2[] = { "Accel Position" };
const char MenuMaster::TL1S3SS1[] = { "Brake" };
const char MenuMaster::TL1S3SS2[] = { "Brake Pressure" };
const char MenuMaster::TL1S4[] = { "Speed" };
const char MenuMaster::TL1S5[] = { "RPM" };
const char MenuMaster::TL1S6[] = { "Steering Angle" };
const char MenuMaster::TL1S7[] = { "Unknown Monitor" };
const char MenuMaster::TL1S8[] = { "Empty" };
const char MenuMaster::TL1S9[] = { "G-Force Lat" };
const char MenuMaster::TL1S10[] = { "G-Force Long" };
const char MenuMaster::TL2[] = { "Data Logger" };
const char MenuMaster::TL2S1[] = { "Start/Stop" };
const char MenuMaster::TL3[] = { "??? Mystery Item! ??" };
const char MenuMaster::TL4[] = { "Soft Reset" };
/* Below is the Text Array for the menu structure
 * This is confusing AS ALL HELL, but once you get it, you get it.
 * Think of it as a flattened menustructure (see menutype array from header file)
 * 
 * To further save memory, the text for all the menu items are stored in progmem as well
 * 
 * ALL DEM SPACES NEED TO BE IN THERE TO KEEP ALIGNMENT
 */
const char* const MenuMaster::menuTable[] = { TL1, "", TL1S1, TL1S1, TL1S2, TL1S2, TL1S3SS1, TL1S3SS2, TL1S4, TL1S4, TL1S5, TL1S5, TL1S6, TL1S6, TL1S7, TL1S7, TL1S8, TL1S8, TL1S9, TL1S9, TL1S10, TL1S10, TL2, "", TL2S1, TL2, "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", TL3, "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", TL4, "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" };

MenuMaster::MenuMaster(int depth, int index1, int index2, int index3)
{
	mD = depth;
	m1 = index1;
	m2 = index2;
	m3 = index3;
}
void MenuMaster::setValues(int depth, int index1, int index2, int index3)
{
	mD = depth;
	m1 = index1;
	m2 = index2;
	m3 = index3;
}
//Converts multi-dimensional array location into single dimensional array location
short MenuMaster::calcIndex(short index1, short index2, short index3)
{
	short index;
	short calc1 = index1 * dim2Size * dim3Size;
	short calc2 = index2 * dim3Size;
	index = calc1 + calc2 + index3;
	return index;
}

int* MenuMaster::getMenuSize()
{
	/* This had to be altered since moving menu items to progmem
	static int menuSize[3];
	menuSize[0] = sizeof(menuText) / sizeof(*menuText);
	menuSize[1] = (sizeof(*menuText) / sizeof(**menuText) - 1);
	menuSize[2] = (sizeof(**menuText) / sizeof(***menuText) - 1);
	return menuSize;
	*/
	static int menuSize[3];
	menuSize[0] = dim1Size;
	menuSize[1] = dim2Size - 1;
	menuSize[2] = dim3Size - 1;
	return menuSize;
}
//Returns Top Text of smallLCD based off menu indexes
String MenuMaster::getTopText()
{
	String top;
	if (menuType[m1][m2][m3] == UPDOWN)
	{	
		if (mD == -1)
			top = "";
		else if (mD == 0)
			top = "Main Menu";
		else if (mD == 1){
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, 0, 0)]))); // Necessary casts and dereferencing,
			top = buffer;
		}

		else if (mD == 2)
		{
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, 0)]))); // Necessary casts and dereferencing,
			top = buffer;
		}

		else{
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, m3)]))); // Necessary casts and dereferencing,
			top = buffer;
		}
	}
	else if (menuType[m1][m2][m3] == CALLBACK)
	{
		if (mD == 0)
			top = "Main Menu";
		else if (mD == 1){
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, 0, 0)]))); // Necessary casts and dereferencing,
			top = buffer;
		}

		else if (mD == 2)
		{
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, 0)]))); // Necessary casts and dereferencing,
			top = buffer;
		}

		else{
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, m3)]))); // Necessary casts and dereferencing,
			top = buffer;
		}
	}
	else if (menuType[m1][m2][m3] == ONOFF)
	{
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, m3)]))); // Necessary casts and dereferencing,
		top = buffer;
	}
	return top;
}
//Returns Bottom Text of smallLCD based off menu indexes
String MenuMaster::getBottomText()
{
	String bottom;
	if (menuType[m1][m2][m3] == UPDOWN)
	{
		selectable = false;
		callBack = false;
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, m3)]))); // Necessary casts and dereferencing,
		bottom = buffer;
	}
	else if (menuType[m1][m2][m3] == CALLBACK)
	{
		selectable = false;
		callBack = true;
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuTable[calcIndex(m1, m2, m3)]))); // Necessary casts and dereferencing,
		bottom = buffer;

	}
	else if (menuType[m1][m2][m3] == ONOFF)
	{
		selectable = true;
		callBack = false;
		if (m1 == 0) {
			loggerSelected = loggers[m2 - 1];
		}
		else if (m1 == 1) {
			loggerSelected = enableLogging;
		}

		if (loggerSelected)
			bottom = "   [Yes]  No";
		else
			bottom = "    Yes  [No]";

	}

	return bottom;
}
//Returns Bool if yesno menu
bool MenuMaster::getSelectable()
{
	return selectable;
}
//Returns Bool if callback menu
bool MenuMaster::getCallback()
{
	return callBack;
}
//Return Menu Type for a specified menu - used for menu blocking commands currently
short MenuMaster::getMenuType(short index1, short index2, short index3)
{
  return menuType[index1][index2][index3];
}
//Select up in menu lists
void MenuMaster::selectUp()
{
	if (m1 == 0) {
		if (!locked)
		{
      //This part sets a logger object (aligned with the menu text and text in master *hopefully*) to either true or false
			if (loggerSelected)
				loggers[m2 - 1] = false;
			else
				loggers[m2 - 1] = true;
		}
	}
	else if (m1 == 1) {
		if (loggerSelected)
			enableLogging = false;
		else
			enableLogging = true;
	}
}
//Select down in menu lists
void MenuMaster::selectDown()
{
	if (m1 == 0) {
		if (!locked)
		{
			if (loggerSelected)
				loggers[m2 - 1] = false;
			else
				loggers[m2 - 1] = true;
		}
	}
	else if (m1 == 1) {
		if (loggerSelected)
			enableLogging = false;
		else
			enableLogging = true;
	}
}
//Sets all loggers on
void MenuMaster::setLoggersTrue()
{
	for (int i = 0; i < (dim2Size - 1); i++)
	{
		loggers[i] = true;
	}
}
//Locks Logger State
void MenuMaster::lockLoggers()
{
	locked = true;
}
//Unlocks Logger State
void MenuMaster::unlockLoggers()
{
	locked = false;
}
