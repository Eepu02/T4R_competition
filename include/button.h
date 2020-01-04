/*++++++++++++++++++++++++++++++++++++++++++| INFOMATION |++++++++++++++++++++++++++++++++++++++++++++
Welcome to the ACCESS_OS. Here, you can configure settings for your robot and display text
Functions that are very useful:

GUI_print(string text, int row, int col) --> Replacement for Screen.print and Screen.setCursor
GUI_printMSG(string text) --> Prints text on the first line
GUI_clearLine(int row) --> Replacement for Screen.clearLine without clearing the selector
GUI_clearScreen() --> Replacement for Screen.clearScreen without clearing the selector
GUI_selector(int row) --> Shows cursor on passed row
GUI_displayMenu(int currRow, int configuration[]) --> Display the menu options for selected row
ACCESS_config() --> Easy command to run ACCESS OS's GUI + options

I, robonxt, has provided a basic templete, but you'll need to modify the code to fit your robot
------------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------------
[GLOBAL] Includes > Include needed external libraries and files
------------------------------------------------------------------------------------------------------*/
//#include "config.h" //Remember to update this file if we add new sensors!
#include "voidit.h"
#include "pros/misc.h"
//create object Win of the Competition class because we are going to win. Period.
<<<<<<< Updated upstream
=======
vex::competition Win;
>>>>>>> Stashed changes

/*------------------------------------------------------------------------------------------------------
[GLOBAL] Variables for GUI > Constant vars that won't change during program run
------------------------------------------------------------------------------------------------------*/
//Define buttons
#define btnNONE			0
#define btnUP				1
#define btnDOWN			2
#define btnLEFT			3
#define btnRIGHT		4
#define btnA				5
#define btnB				6
#define btnX				7
#define btnY				8
#define btnR1				9
#define btnR2				10
#define btnL1				11
#define btnL2				12

//Define delays
<<<<<<< Updated upstream
#define bounceDelay				20 //Time to wait between reads to check for key bounce
#define refreshDelay			200 //Time to wait between refreshing screens
=======
#define bounceDelay		20 //Time to wait between reads to check for key bounce
#define refreshDelay	200 //Time to wait between refreshing screens
>>>>>>> Stashed changes

//Define usable screen space
#define screenTextWidth		16 //Max characters you can print on one line
#define screenTextHeight	3 //Max lines of text that can show on the controller

//Array
<<<<<<< Updated upstream
#define maxMenus					3 //Make this the same # as items in maxMenusIndex
#define maxOptions				3 //Make this the same # as items in 2D of menuDisplayIndex

//Robot status
#define modeDisabled			0
#define modeDriver				1
#define modeAuton					2
#define modeError					3
=======
#define maxMenus			3 //Make this the same # as items in maxMenusIndex
#define maxOptions		3 //Make this the same # as items in 2D of menuDisplayIndex

//Robot status
#define modeDisabled	0
#define modeDriver		1
#define modeAuton		2
#define modeError		3
>>>>>>> Stashed changes

/*------------------------------------------------------------------------------------------------------
[GLOBAL] Arrays for GUI > GUI needed stuff
------------------------------------------------------------------------------------------------------*/
//1D array for displaying robot status
std::string robotStatus[4] =
{
	"!Robot Disabled!",
	"Driver Control",
	"Auton Control",
	"!!!MODE ERROR!!!"
};

//1D Array for max options in menu
int maxMenusIndex[maxMenus] =
{
	2,
	2,
<<<<<<< Updated upstream
 	2
=======
	2
>>>>>>> Stashed changes
};

//1D Array for selected options
int configuration[maxMenus] =
{
	0,
	0,
	0
};

//1D array for displaying the menu types
std::string menuTypes[maxMenus] =
{
	"Color: ",
	"Row: ",
	"Mode: "
};

//2D array for displaying the menu options
std::string menuOptions[maxMenus][maxOptions] =
{
	{ "Blue", "Red", "" },
	{ "Front", "Back", "" },
	{ "RC", "TANK", "" }
};

/*------------------------------------------------------------------------------------------------------
[STATUS - SYSTEM] rumble > User feedback through controller's rumble motors
------------------------------------------------------------------------------------------------------*/
void rumble()	{
	Controller1.rumble(".");
}

/*------------------------------------------------------------------------------------------------------
[STATUS - SYSTEM] delay > Use as replacement for the system's vex::task::sleep
------------------------------------------------------------------------------------------------------*/
void delay(int d)	{
	sleep(d);
}

/*------------------------------------------------------------------------------------------------------
[STATUS - SYSTEM] keyPressedRaw > Returns raw values for button pressed
------------------------------------------------------------------------------------------------------*/
int keyPressedRaw()	{

	if (Controller1.get_digital(DIGITAL_UP) == true)			return btnUP;

	if (Controller1.get_digital(DIGITAL_DOWN) == true)		return btnDOWN;

	if (Controller1.get_digital(DIGITAL_LEFT) == true)		return btnLEFT;

	if (Controller1.get_digital(DIGITAL_RIGHT) == true)		return btnRIGHT;

	if (Controller1.get_digital(DIGITAL_A) == true)				return btnA;

	if (Controller1.get_digital(DIGITAL_B) == true)				return btnB;

	if (Controller1.get_digital(DIGITAL_X) == true)				return btnX;

	if (Controller1.get_digital(DIGITAL_Y) == true)				return btnY;

	if (Controller1.get_digital(DIGITAL_R1) == true)			return btnR1;

	if (Controller1.get_digital(DIGITAL_R2) == true)			return btnR2;

	if (Controller1.get_digital(DIGITAL_L1) == true)			return btnL1;

	if (Controller1.get_digital(DIGITAL_L2) == true)			return btnL2;

	else																									return btnNONE;
}

/*------------------------------------------------------------------------------------------------------
[STATUS - SYSTEM] keyPressed > Returns clean values (no bounce) for button pressed
------------------------------------------------------------------------------------------------------*/
int keyPressed()
{
	int noBounceKey = keyPressedRaw();
	delay(bounceDelay);
	if (noBounceKey == keyPressedRaw())
	{
		return noBounceKey;
	}
	else
	{
		return btnNONE;
	}
}

/*------------------------------------------------------------------------------------------------------
[STATUS - SYSTEM] currStatus > Returns 0 (Disabled), 1 (Auton), or 2 (Driver)
------------------------------------------------------------------------------------------------------*/
int currStatus()
{
	int status;
<<<<<<< Updated upstream
	if (eneble == true) //If robot is enabled, check what mode/period the match is in
	{
		if (win == true) {
			status = modeAuton; //Robot is in Autonomous mode
		}
		else if (win == false) {
=======
	if (Win.isEnabled()) //If robot is enabled, check what mode/period the match is in
	{
		if (Win.isAutonomous()) {
			status = modeAuton; //Robot is in Autonomous mode
		}
		else if (Win.isDriverControl()) {
>>>>>>> Stashed changes
			status = modeDriver; //Robot is in Driver Control mode
		}
		else
		{
			status = modeError; //ERROR MODE
		}
	}
	else
	{
		status = modeDisabled; //Robot is Disabled
	}
	return status;
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] GUI_clearLine > GUI replacement for the system's Controller.Screen.clearLine
------------------------------------------------------------------------------------------------------*/
void GUI_clearLine(int l_row) //select what row to clear (not selector)
{

	Controller1.clear_line(l_row);//Screen.setCursor(l_row + 1, 2); //Prevent overwrite of selector GUI
	//Controller1.Screen.print("               "); //15 spaces (1 less than screenTextWidth)
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] GUI_clearScreen > GUI replacement for the system's Controller.Screen.clearScreen
------------------------------------------------------------------------------------------------------*/
void GUI_clearScreen() //clears everything but the selector
{
	for (int row = 0; row < screenTextHeight; row++) //loops rows
	{
		GUI_clearLine(row);
	}
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] GUI_print > GUI replacement for the system's Controller.Screen.print
------------------------------------------------------------------------------------------------------*/
void GUI_print(std::string text, int row, int col) //pass text, row, and col
{
	//NOTE: Does not call GUI_clearScreen for you, only clears current line
	GUI_clearLine(row);
<<<<<<< Updated upstream
	Controller1.set_text(row + 1, col + 2, text.c_str());
	//Controller1.Screen.setCursor(row + 1, col + 2); //Col + 2: Prevent overwrite of selector GUI
	//Controller1.Screen.print(text.c_str()); //Future GUI will scroll text
=======
	controller_print(CONTROLLER_MASTER, row+1, col+2);
	Controller1.Screen.setCursor(row + 1, col + 2); //Col + 2: Prevent overwrite of selector GUI
	Controller1.Screen.print(text.c_str()); //Future GUI will scroll text
>>>>>>> Stashed changes
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] GUI_selector > pass 0, 1, or 2 to pick lines. (if more than 2, it jumps back to 0)
------------------------------------------------------------------------------------------------------*/
void GUI_selector(int row) //pass which row you want the selector to be
{
	for (int i = 1; i <= screenTextHeight; i++) //clears just the selector GUI
	{
<<<<<<< Updated upstream
		Controller1.set_text(i, 1, "|");
		//Controller.Screen.setCursor(i, 1);
		//Controller.Screen.print("|");
=======
		Controller.Screen.setCursor(i, 1);
		Controller.Screen.print("|");
>>>>>>> Stashed changes
	}

	//reminder of row == row mod / 3
	int showCursor = (row % screenTextHeight) + 1;
<<<<<<< Updated upstream
	Controller1.set_text(showCursor, 1, ">");
	//Controller.Screen.setCursor(showCursor, 1);
	//Controller.Screen.print(">");
=======
	Controller.Screen.setCursor(showCursor, 1);
	Controller.Screen.print(">");
>>>>>>> Stashed changes
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] GUI_displayMenu > Displays the actual menu (only works up to three menus)
------------------------------------------------------------------------------------------------------*/
void GUI_displayMenu(int currRow, int configuration[]) //pass which row you want the selector to be
{
	std::string temp;
	GUI_selector(currRow); //Display selector
	for (int i = 0; i < screenTextHeight; i++)
	{
		temp = menuTypes[i] + menuOptions[i][configuration[i]];
		GUI_print(temp, i, 0); //Display menu line i
	}
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] GUI_status > Displays the status of the robot
------------------------------------------------------------------------------------------------------*/
void GUI_status(int mode) //pass 0 (Disabled), 1 (Auton), or 2 (????)
{
	std::string temp;
	//Display Selected Options
	temp = menuOptions[0][configuration[0]] + " " + menuOptions[1][configuration[1]] + " " + menuOptions[2][configuration[2]]; //Color + space + Row + space + Drive
	GUI_print(robotStatus[mode], 1, 0); //Show robot mode on line 2
	GUI_print(temp, 2, 0); //Show Color + Row on line 3
}

/*------------------------------------------------------------------------------------------------------
[STATUS - SYSTEM] GUI_printMSG > Replacement for Controller.Screen.print (Prints on top line)
------------------------------------------------------------------------------------------------------*/
void GUI_printMSG(std::string str)
{
	//Clears screen then print GUI
<<<<<<< Updated upstream
	Controller1.set_text(1, 1,"");
	//Controller.Screen.setCursor(1, 1);
	//Controller.Screen.clearScreen();
	Controller1.clear();
=======
	Controller.Screen.setCursor(1, 1);
	Controller.Screen.clearScreen();
>>>>>>> Stashed changes
	GUI_status(currStatus());

	GUI_print(str, 0, 0);
}

/*------------------------------------------------------------------------------------------------------
[STATUS - GUI] ACCESS_config > Select config GUI (ONLY WORKS IF ROBOT IS NOT DIABLED)
------------------------------------------------------------------------------------------------------*/
void ACCESS_config()
{
	int currCursorMenu = 0; //Goes up and down of array
	int currCursorOptions = 0; //Goes left and right of array

	bool isAutonSelectScreen = true;
	while (isAutonSelectScreen == true)
	{
		GUI_displayMenu(currCursorMenu, configuration);

		bool isValidButton = false;
		while (isValidButton == false) //Only updates screen if button was pressed
		{
			switch (keyPressed())
			{
			case(btnUP):	//Go up the array
				isValidButton = true;
				currCursorMenu--;

				//Get new values from configuration
				currCursorMenu = (currCursorMenu + maxMenus) % maxMenus; //currCursorMenu % the amount of items in 1D array
				currCursorOptions = configuration[currCursorMenu];
				break;
			case(btnDOWN):	//Go down the array
				isValidButton = true;
				currCursorMenu++;

				//Get new values from configuration
				currCursorMenu = (currCursorMenu + maxMenus) % maxMenus; //currCursorMenu % the amount of items in 1D array
				currCursorOptions = configuration[currCursorMenu];
				break;
			case(btnLEFT):	//Shift left of array
				isValidButton = true;
				currCursorOptions--;

				//Update values back into configuration
				currCursorOptions = (currCursorOptions + maxMenusIndex[currCursorMenu]) % maxMenusIndex[currCursorMenu]; //currCursorOptions % the amount of options in 1D array
				configuration[currCursorMenu] = currCursorOptions; //Assign current menu(row) option(col) to configuration array
				break;
			case(btnRIGHT):	//Shift right of array
				isValidButton = true;
				currCursorOptions++;

				//Update values back into configuration
				currCursorOptions = (currCursorOptions + maxMenusIndex[currCursorMenu]) % maxMenusIndex[currCursorMenu]; //currCursorOptions % the amount of options in 1D array
				configuration[currCursorMenu] = currCursorOptions; //Assign current menu(row) option(col) to configuration array
				break;
			case(btnA):		//Select auton and exit
				isValidButton = true;
				isAutonSelectScreen = false;
				break;
			default:
				break;
			}
		}
	}
}

/*------------------------------------------------------------------------------------------------------
[STATUS] autoMoveTile > Moves forward or backwards by tile counts AUTON ONLY
------------------------------------------------------------------------------------------------------*/
void autoMoveTile(float tiles, int speed)
{
	//Code for moving the robot forward/backwards here
}

/*------------------------------------------------------------------------------------------------------
[STATUS] autoTurnTile > Turns left or right AUTON ONLY DUE TO THE BLUE/RED TILE SWAP
------------------------------------------------------------------------------------------------------*/
void autoTurnTile(float x, int speed)
{
	if (configuration[0] == 1) //If color is 1 (RED), swap values
	{
		x = -x;
	}
	//Code for moving the robot left/right here
}

/*------------------------------------------------------------------------------------------------------
[STATUS] tankDrive > Tank drive control system
------------------------------------------------------------------------------------------------------*/
void tankDrive()
{
	//Add TANK-style driver code here
}

/*------------------------------------------------------------------------------------------------------
[STATUS] RCDrive > RC-style drive control system
------------------------------------------------------------------------------------------------------*/
void RCDrive()
{
	//Add RC-style driver code here
}

/*------------------------------------------------------------------------------------------------------
[STATUS] pre_auton > Set up any initialization stuff here before auton
------------------------------------------------------------------------------------------------------*/
void pre_auton(void)
{
	ACCESS_config();
	GUI_printMSG("Settings Done");
}

/*------------------------------------------------------------------------------------------------------
[STATUS] autonFrontRow > Front Auton (PROGRAM FOR BLUE SIDE, IT WILL ADAPT FOR RED SIDE)
------------------------------------------------------------------------------------------------------*/
void autonFrontRow()
{
	//Code for Auton Front Row here
}

/*------------------------------------------------------------------------------------------------------
[STATUS] autonBackRow > Back Auton (PROGRAM FOR BLUE SIDE, IT WILL ADAPT FOR RED SIDE)
------------------------------------------------------------------------------------------------------*/
void autonBackRow()
{
	//Code for Auton Back Row here
}

/*------------------------------------------------------------------------------------------------------
[STATUS] autonomous > The heart and soul of your robot
------------------------------------------------------------------------------------------------------*/
void autonomous(void)
{
	//If configuration[0] is 0 (BLUE) or 1 (RED), display color on Brain
	if (configuration[0] == 0) //If Blue
	{
<<<<<<< Updated upstream
		lv_color16_t(blue);
		//Brain.Screen.clearScreen(vex::color::blue);
	}
	else if (configuration[0] == 1) //If Red
	{
		lv_color16_t(red);
		//Brain.Screen.clearScreen(vex::color::red);
=======
		Brain.Screen.clearScreen(vex::color::blue);
	}
	else if (configuration[0] == 1) //If Red
	{
		Brain.Screen.clearScreen(vex::color::red);
>>>>>>> Stashed changes
	}

	//If configuration[1] is 0 (Front row) or 1 (Back row), run correct auton
	if (configuration[1] == 0)
	{
		autonFrontRow();
	}
	else if (configuration[1] == 1)
	{
		autonBackRow();
	}

	//Auton end. Prints auton end on the controller + vibrate controller
	GUI_printMSG("Auton: END");
	rumble();
}

/*------------------------------------------------------------------------------------------------------
[STATUS] usercontrol > Driver control stuff here, your robot is in your hands
------------------------------------------------------------------------------------------------------*/
void usercontrol(void)
{
	//User control code here, inside the loop
	while (1)
	{
<<<<<<< Updated upstream
		Controller1.clear();
		lv_color16_t(green);
		//___int16_t_definedBrain.Screen.clearScreen(vex::color::green); //Get the driver control ready with visual cues
=======
		Brain.Screen.clearScreen(vex::color::green); //Get the driver control ready with visual cues
>>>>>>> Stashed changes

		//If configuration[2] is 0 (RC) or 1 (TANK), run correct mode
		while (configuration[2] == 0)
		{
			RCDrive();
			//Add your scoring code here
		}
		while (configuration[2] == 1)
		{
			tankDrive();
			//Add your scoring code here
		}
	}
}

/*------------------------------------------------------------------------------------------------------
[STATUS] main > System required loop, (Can't?) add stuff here
------------------------------------------------------------------------------------------------------*/
int main()
{
	int tempStatus = 0;
	//Run the pre-autonomous function.
	pre_auton();

	//Set up callbacks for autonomous and driver control periods.
<<<<<<< Updated upstream
	/*Win.autonomous(autonomous);									 // muista
	Win.drivercontrol(usercontrol);               // laita main.cpp callback!!!!
*/
=======
	Win.autonomous(autonomous);
	Win.drivercontrol(usercontrol);

>>>>>>> Stashed changes
	//Prevent main from exiting with an infinite loop.
	while (1)
	{
		if (tempStatus != currStatus()) //If currStatus was updated, refresh screen
		{
			GUI_status(tempStatus);
			tempStatus = currStatus();
		}
		sleep(100); //Sleep the task for a short amount of time to prevent wasted resources.
	}
}
