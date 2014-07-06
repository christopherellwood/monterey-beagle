#include "monterey-beagle.h"
//#include <linux/delay.h>

#define INPUTPORT 51000   //The port on which to listen for incoming data
#define OUTPUTPORT 50000   //The port on which to send return data

int main()
{
	//Declare a UDP Connection for commands and telemetry
	UDP_Connection UDP(INPUTPORT, OUTPUTPORT);

	//Create the ROV object that interprets the commands and controls the ROV
	ROV_Manager ROV(&UDP);

	//Pointers to UDP and ROV so they can be passed to different threads
	Pointer_Set Pointers;
	//Set the values in the pointer structure
	Pointers.connection = &UDP;
	Pointers.manager = &ROV;

	//Start the ROV communications module
	ROV.Start_Comms(&Pointers);

	//Start the ROV sampling sensors
	ROV.Start_Sensors(&Pointers);

	//Get user input, pretty much just wait for a q command
	char a;
	while (a != 'q')
	{
		cin >> a;

	}
	return 0;
}
