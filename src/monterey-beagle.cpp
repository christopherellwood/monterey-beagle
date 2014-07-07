#include "monterey-beagle.h"


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
//	int i = 0;
//	double x, y;
//	sleep(1);
//	while(1)
//	{
//		i++;
//		if (i == 100) i = 0;
//
//		x = atan2((double)ROV.DirectionSensor.M.x, (double)(ROV.DirectionSensor.M.y))  * 180 / 3.14159;
//		if (x < 0) x+=360;
//		printf("%4.2f\r", x);
//		//printf("%d: %.0f, %.0f, %.0f     \r", i, ROV.DirectionSensor.M.x, ROV.DirectionSensor.M.y, ROV.DirectionSensor.M.z);
//		usleep(3000);
//	}
	//Get user input, pretty much just wait for a q command
	char a;
	while (a != 'q')
	{
		cin >> a;

	}
	return 0;
}
