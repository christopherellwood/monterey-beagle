#include "monterey-beagle.h"

#define INPUTPORT 51000   //The port on which to listen for incoming data
#define OUTPUTPORT 50000   //The port on which to send return data


int main()
{

	//Declare a UDP Connection
	UDP_Connection UDP(INPUTPORT, OUTPUTPORT);

	//Create an ROV object
	ROV_Manager ROV(&UDP);

	//Structure of pointers to UDP and ROV so they can be passed together
	Pointer_Set Pointers;

	//Set the values in the pointer structure
	Pointers.connection = &UDP;
	Pointers.manager = &ROV;

	//Start ROV communications server, this branches a thread and loops
	ROV.Start_Comms(&Pointers);

	//Start sampling sensors, this just samples the sensors and loops
	ROV.Sample_Sensors();

	pthread_exit(NULL);
	return 0;
}
