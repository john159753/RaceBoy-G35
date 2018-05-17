#ifndef MENUMASTER_H
#define MENUMASTER_H
#include <avr/pgmspace.h>
#include <Wstring.H>
#include <SdFat.h>

#define BLOCK -1
#define NOTUSED 0
#define UPDOWN 1
#define ONOFF 2
#define CALLBACK 3
#define LIST 4

class MenuMaster
{
private:
	// Data Members - FYI THESE ARE ALL PROGMEM = <3 on my arduino memory
	static const char PROGMEM TL1[];
	static const char PROGMEM TL1S1[];
	static const char PROGMEM TL1S2[];
	static const char PROGMEM TL1S3SS1[];
	static const char PROGMEM TL1S3SS2[];
	static const char PROGMEM TL1S4[];
	static const char PROGMEM TL1S5[];
	static const char PROGMEM TL1S6[];
	static const char PROGMEM TL1S7[];
	static const char PROGMEM TL1S8[];
	static const char PROGMEM TL1S9[];
	static const char PROGMEM TL1S10[];
	static const char PROGMEM TL2[];
	static const char PROGMEM TL2S1[];
	static const char PROGMEM TL3[];
	static const char PROGMEM TL4[];
	static const char* const PROGMEM menuTable[88];
 /*
  * This is confusing, but hear me out.
  * 1st dimension (x4) is how many top menus exist
  * 2nd dimension (x11 at max (has to include top menu) is how many sub menus exist
  * 3rd dimension (x2) is the action of the menu you are on vs what happens when you progress to the next menu
  * 
  * UpDown - Cycle through menu item array
  * OnOff - Toggles a menu item on or off
  * Callback - Performs no action on the menu object itself, calls back to main for a method
  * Notused - Because of design, there needs to be a place holder here for when you move onto a submenu
  * BLOCK - Used to block access to further (non-existent) elements of the array
  */
	const char menuType[4][11][2] = {
		{ { UPDOWN, NOTUSED }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF }, { UPDOWN, ONOFF } },
		{ { UPDOWN, NOTUSED }, { UPDOWN, ONOFF }, { BLOCK, NOTUSED } },
		{ { UPDOWN, NOTUSED }, { LIST, LIST }, { BLOCK, NOTUSED } },
		{ { CALLBACK, NOTUSED } }
	};
  //Buffer to copy menuitems from progmem to dynamic memory
	char buffer[20];
	short menuModifier = 0;
	short mD;
	short m1;
	short m2;
	short m3;
	short calcIndex(short index1, short index2, short index3);
	bool loggerSelected;
	bool locked;
  //Define menu sizes manually since the menu text array had been collapsed to work with progmem
	const static short dim1Size = 4; // TL is 4 deep
	const static short dim2Size = 11; // Sub is 10 deep (remeber - have to include the top levels)
	const static short dim3Size = 2; // Choice menu is flat
	//Track Selection Strings
	char trackNames[4][20];


public:
	// Data Members
	bool enableLogging;
	bool loggers[(dim2Size - 1)];
	bool selectable;
	bool callBack;
	// Constructors
	MenuMaster(int depth, int index1, int index2, int index3);

	// Methods
	void setLoggersTrue();
	void setValues(int depth, int index1, int index2, int index3);
	int* getMenuSize();
	String getTopText();
	String getBottomText();
	String getLine3Text();
	String getLine4Text();
	bool getSelectable();
	bool getCallback();
	short getMenuType(short index1, short index2, short index3);
	int selectUp();
	int selectDown();
	void lockLoggers();
	void unlockLoggers();
	void loadTracks(SdFat &sd, SdFile &file);
	byte trackCount;
	byte trackSelectIndex = 0;
	byte trackReadIndex = 0;
};
#endif
