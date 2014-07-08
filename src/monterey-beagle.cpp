#include "monterey-beagle.h"


#define INPUTPORT 51000  //The port on which to listen for incoming data
#define OUTPUTPORT 50000 //The port on which to send return data
#define LOOP_PRD_MSEC 50 //Timed loop callback interval

//Declare a UDP Connection for commands and telemetry
UDP_Connection UDP(INPUTPORT, OUTPUTPORT);

//Create the ROV object that interprets the commands and controls the ROV
ROV_Manager ROV(&UDP);

//Pointers to UDP and ROV so they can be passed to the comms thread
Pointer_Set Pointers;

void timed_loop()
{
	//Tell the ROV manager to sample the sensors
	ROV.Sample_Sensors();
}

int main()
{
	//Set the values in the pointer structure
	Pointers.connection = &UDP;
	Pointers.manager = &ROV;

	//Start an even driven communications thread
	ROV.Start_Comms(&Pointers);

	//Start a timer so the program runs in a controlled fashion
	if(start_timer(LOOP_PRD_MSEC, &timed_loop))
	{
		printf("Timer error, exiting!\n");
	}

	//Idle while not sampling the sensors
	while(1)
	{
		sleep(1);
	}
	stop_timer();
	return 0;
}
