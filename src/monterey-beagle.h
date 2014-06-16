#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include <cstdlib>
#include <pthread.h>
#include <iostream>
#include <fstream>

#define BUFFERLEN 64  //Length of receive buffer
#define NUM_MOTORS 3  //Number of motors
#define NUM_RELAYS 3  //Number of relays
#define NUM_SERVOS 1  //Number of servos

using namespace std;

class IO_Pin
{
private:
	char * direction_;
	char * directionfile;
	char * valuefile;
	char * ID_;
	static const int kfilelength = 35;
	static const int knumAM335xGPIO = 128;
	char value_[2];
	FILE * fp_;

public:
	IO_Pin(char * ID, char * direction, int value)
	{
	//char direction must be "in" or "out"
	//note that if it is an input, the value argument has no effect
		if ((atoi(ID) < knumAM335xGPIO) &&
			(atoi(ID) > 0 ) &&
			((strcmp(direction,"in") == 0) ||
			 (strcmp(direction, "out") == 0)) &&
			((value == 0) ||
			 (value == 1)))
		{
			valuefile = new char[kfilelength];
			directionfile = new char[kfilelength];
			bzero((char *) directionfile, kfilelength);
			bzero((char *) valuefile, kfilelength);
			ID_ = ID;
			direction_ = direction;

			sprintf(value_, "%d", value);
			sprintf(directionfile, "/sys/class/gpio/gpio%s/direction", ID_);
			sprintf(valuefile, "/sys/class/gpio/gpio%s/value", ID_);

			fp_ = fopen("/sys/class/gpio/export", "ab");
			rewind(fp_);
			fwrite(ID_, sizeof(char), 4, fp_);
			fclose(fp_);

			fp_ = fopen(directionfile, "rb+");
			rewind(fp_);
			fwrite(direction_, sizeof(char), 4, fp_);
			fclose(fp_);

			if (strcmp(direction, "out") == 0)
			{
				fp_ = fopen(valuefile, "rb+");
				rewind(fp_);
				fwrite(value_, sizeof(char), 1, fp_);
				fclose(fp_);
			}
		}
		else
		{
			printf("Error: GPIO ID number out of bounds\n");
			printf("Error direction %s\n", direction);
		}
	}
	~IO_Pin()
	{
		fp_ = fopen("/sys/class/gpio/unexport", "ab");
		rewind(fp_);
		fwrite(ID_, sizeof(char), 4, fp_);
		fclose(fp_);
	}
	void Set(int value)
	{
		// Set the value of a pin (0 or 1)
		sprintf(value_, "%d", value);
		fp_ = fopen(valuefile, "rb+");
		rewind(fp_);
		fwrite(value_, sizeof(char), 1, fp_);
		fclose(fp_);
	}
	int Read()
	{
		fp_ = fopen(valuefile, "rb+");
		rewind(fp_);
		fread(value_, sizeof(char), 1, fp_);
		return atoi(value_);
	}
};

class PWM_Pin
{

private:
	char * period_;
	char * periodfile;
	char * duty_;
	char * dutyfile;
	char * polarityfile;
	char * ID_;
	static const int kfilelength = 50;
	FILE * fp_;
public:
	PWM_Pin(char * ID, char * period, char * duty)
	{
		periodfile = new char[kfilelength];
		dutyfile = new char[kfilelength];
		polarityfile = new char[kfilelength];
		bzero((char *) periodfile, kfilelength);
		bzero((char *) dutyfile, kfilelength);
		bzero((char *) polarityfile, kfilelength);
		ID_ = ID;
		period_ = period;
		duty_ = duty;

		sprintf(periodfile, "/sys/devices/ocp.2/%s/period", ID_);
		sprintf(dutyfile, "/sys/devices/ocp.2/%s/duty", ID_);
		sprintf(polarityfile, "/sys/devices/ocp.2/%s/polarity", ID_);

		fp_ = fopen(periodfile, "rb+");
		rewind(fp_);
		fwrite(period_, sizeof(char), 12, fp_);
		fclose(fp_);

		fp_ = fopen(dutyfile, "rb+");
		rewind(fp_);
		fwrite(duty_, sizeof(char), 12, fp_);
		fclose(fp_);

		fp_ = fopen(polarityfile, "rb+");
		rewind(fp_);
		fwrite("0", sizeof(char), 1, fp_);
		fclose(fp_);
	}
	~PWM_Pin()
	{

	}
	void Set_Period(char * period)
	{
		period_ = period;
		fp_ = fopen(periodfile, "rb+");
		rewind(fp_);
		fwrite(period_, sizeof(char), 12, fp_);
		fclose(fp_);
	}
	void Set_Duty(char * duty)
	{
		duty_ = duty;
		fp_ = fopen(dutyfile, "rb+");
		rewind(fp_);
		fwrite(duty_, sizeof(char), 12, fp_);
		fclose(fp_);
	}
};

class GPIO_Control
{
private:
public:
	IO_Pin * Pin[4];
	PWM_Pin * MotorPWM[3];
	PWM_Pin * ServoPWM[3];
	GPIO_Control()
	{
		Pin[0] = new IO_Pin("68", "out", 0);
		Pin[1] = new IO_Pin("45", "out", 0);
		Pin[2] = new IO_Pin("44", "out", 0);
		Pin[3] = new IO_Pin("26", "out", 0);
		MotorPWM[0] = new PWM_Pin("pwm_test_P8_13.11", "20000000", "1000000");
		MotorPWM[1] = new PWM_Pin("pwm_test_P8_19.12", "20000000", "1000000");
		MotorPWM[2] = new PWM_Pin("pwm_test_P9_14.13", "20000000", "1000000");
		ServoPWM[0] = new PWM_Pin("pwm_test_P9_16.14", "20000000", "1000000");
		ServoPWM[1] = new PWM_Pin("pwm_test_P9_21.15", "20000000", "1000000");
		ServoPWM[2] = new PWM_Pin("pwm_test_P9_22.16", "20000000", "1000000");
	}
	~GPIO_Control()
	{
	}
};

struct Pointer_Set {
	/* This class exists because pthread_create can only pass one void argument
	 * to the subroutine it links to, and because that function must be static
	 * when it is a member of a class. Static functions are global, so they
	 * cannot see or act on data members of instances.
	 */
	void * manager;
	void * connection;
};

class UDP_Connection
{
    /* The UDP Connection class manages a connection between the ROV controller
     * program and the embedded ROV manager application. It is designed to
     * receive broadcast control messages, then respond with sensor data to the
     * client that sent the control message. The control data is stored in
     * public arrays for the calling function to retrieve and apply to the ROV
     * motors, relays, lights, etc.
     *
     * Outgoing data is stored in a txbuffer string, and sent by the SendData()
     * command. This allows the client to sample telemetry from the robot at
     * whatever rate is comfortable. The txbuffer variable is public so it can
     * be set by the sensor sampling process.
     *
     * The inPort and outPort arguments are needed to set the input and output
     * ports the connection should use. The SensorData file descriptor is the
     * source of the reply data string. The file should be space separated
     * floating point numbers in character format.
     */
private:
    unsigned int inportnum_, outportnum_, socketnum_;
    struct sockaddr_in server_addr_, client_addr_;
    unsigned int client_addr_len_;
    int transmit_;

public:
    int receive_successful;
    char rxbuffer[BUFFERLEN];

    char * txbuffer;
    UDP_Connection(unsigned int inPort, unsigned int outPort)
    {
        /*
         * This is a server side function to open a socket to receive incoming
         * socket datagram (UDP) connections. The arguments are the port number
         * for incoming and outgoing data.
         */
        // Initialize variables
        char s[BUFFERLEN];
        bzero((char *) &s, BUFFERLEN);
        txbuffer = s;
        inportnum_ = inPort;
        outportnum_ = outPort;
        socketnum_ = -1;
        //recv_len_ = -1;
        transmit_ = -1;
        receive_successful = 0;
        client_addr_len_ = sizeof(client_addr_);

        //Create a new socket, check for error.
        socketnum_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketnum_ < 0)
            printf("ERROR opening socket");

        //Zeroize the memory allocated to the socket address structure
        bzero((char *) &server_addr_, sizeof(server_addr_));

        //Populate socket address structure with AF_INET, server address, and
        //portnum
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr_.sin_port = htons(inportnum_);

        //Link the new socket referred to by socketnum, with the server address
        //structure. Warn if there is an error.
        if (bind(socketnum_, (struct sockaddr *) &server_addr_,
            sizeof(server_addr_)) < 0)
            printf("ERROR on binding");

    }
    ~UDP_Connection()
    {

    }
    void ReceiveData()
    {
        /* This function receives a packet of characters from the topside
         * controller and converts it into a set of integers for controlling
         * motors, relays, and servos.
         */
    	receive_successful = -1;
        bzero((char *) &rxbuffer, BUFFERLEN);
        receive_successful = recvfrom(socketnum_, rxbuffer, BUFFERLEN, 0,
                            (struct sockaddr *) &client_addr_,
                            &client_addr_len_);
    }
    void SendData()
    {
        /* This function returns sensordata to the topside controller for
         * operator feedback.
         */
        client_addr_.sin_port = htons(outportnum_);
        transmit_ = sendto(socketnum_, txbuffer, strlen(txbuffer), 0,
                            (struct sockaddr*) &client_addr_,
                            client_addr_len_);
        if (transmit_ <= 0)
        {
            printf("Transmit Error!\n");
        }
    }

}; //End UDP Connection Class

class ROV_Manager
{
    /* The ROV Manager class controls an ROV. It spawns a listening thread to
     * receive operator commands, and interprets the command strings. After a
     * command is received, it transmits the latest stored telemetry back to
     * the operator. The received command is then converted from a text string
     * to an array of integers representing motor speed, relay state, or servo
     * angle.
     *
     * When not communicating with the operator, the manager polls the sensor
     * collection.
     */
private:
    char sensor_string_[BUFFERLEN];
    UDP_Connection * connection_;
    int motor[NUM_MOTORS];
    int relay[NUM_RELAYS];
    int servo[NUM_SERVOS];
    static void *Communications_Handler(void *ptr)
    {
        /* This function is run as a separate thread to wait for incoming
         * control packets from the surface control station. When a control
         * packet arrives, the function receives it, responds with a packet
         * of the most recently sampled telemetry, then applies the control
         * message to the ROV actuators.
         */

    	Pointer_Set * p;
    	p = (Pointer_Set *)ptr;

        UDP_Connection *c;
        c = (UDP_Connection *)p->connection;

        ROV_Manager *m;
        m = (ROV_Manager *)p->manager;
        while (1)
        {
            c->ReceiveData();
            c->SendData();
            if ((c->receive_successful) >= 0)
                {
                    m->Parse_Command(c->rxbuffer);
                    m->Apply_Commands();
                }
        }
        pthread_exit(NULL);
        return 0;
    }
    void Parse_Command(char * rxdata)
    {
    	/* This function converts the received message string in the UDP object
    	 * into a set of integers for the ROV controls. The message packet is a
    	 * space separated character string of the form:
    	 * intMotor1 intMotorN intRelay1 intRelayN intServo1 intServoN
    	 *
    	 * The maximum number N of each type of device is set in the # defines.
    	 * The data is put into integer arrays to represent the actuation of
    	 * each actuator. The arrays are: motor[], relay[], servo[].
    	 */

        int i=0;
        int j=0;
        int k=0;
        int intRxdata[NUM_MOTORS + NUM_RELAYS + NUM_SERVOS];
        std::istringstream iss(rxdata);
        while((iss >> intRxdata[i]) &&
              (i < (NUM_MOTORS + NUM_RELAYS + NUM_SERVOS - 1)))
        {
            if (i < NUM_MOTORS)
            {
            	motor[i] = intRxdata[i];
            }
            else if (i < (NUM_MOTORS + NUM_RELAYS))
            {
            	relay[j] = intRxdata[i];
                j++;
            }
            else if (i < (NUM_MOTORS + NUM_RELAYS + NUM_SERVOS))
            {
            	servo[k] = intRxdata[i];
                k++;
            }
            i++;
        }
    /*    printf("\nMotors: ");
        for(i=0;i<NUM_MOTORS; i++)
        {
    			printf("%d ", mgr->->motor[i]);
    		}
    			printf("\nRelays: ");
    		for(i=0;i<NUM_RELAYS; i++)
    		{
    			printf("%d ", mgr->->relay[i]);
    		}
    			printf("\nServos: ");
    		for(i=0;i<NUM_SERVOS; i++)
    		{
    			printf("%d ", mgr->->servo[i]);
    		}
        }*/
        return;
    }
    void Apply_Commands()
    {
    	int i;
    	for (i=0; i<NUM_MOTORS; i++)
    	{
    		char * a = new char[12];
    		sprintf(a, "%i", motor[i]*1000);
    		Setpin.MotorPWM[i]->Set_Duty(a);
    	}
    	for (i=0; i<NUM_RELAYS; i++)
    	{
    		Setpin.Pin[i]->Set(relay[i]);
    	}
    	for (i=0; i<NUM_SERVOS; i++)
    	{
    		char * a = new char[12];
    		sprintf(a, "%i", motor[i]*1000);
    		Setpin.ServoPWM[i]->Set_Duty(a);
    	}
    }
public:
    GPIO_Control Setpin;
    ROV_Manager(void *UDP_Connection_Ptr)
	{
        pthread_t comms_thread;
		bzero((char *) &sensor_string_, BUFFERLEN);
		connection_ = (UDP_Connection *)UDP_Connection_Ptr;
		Setpin = GPIO_Control();
	}
    ~ROV_Manager()
    {
    }
    void Start_Comms(void * ptr)
    {
    /*
     * This Function initiates a new thread to listen for communications
     */
        int comms_thread_id;
        pthread_t comms_thread;
        comms_thread_id = pthread_create(&comms_thread, NULL,
                Communications_Handler, (void *) ptr);
    }
    void Sample_Sensors()
    {
        int i=0;
        while(1)
        {
            sprintf(sensor_string_, "%d %d %d %d %d %d", i, i+1, i+2, i+3, i+4, i+5);
            connection_->txbuffer = sensor_string_;
            i++;
            if (i > 5) i = 0;
            sleep(1);
        }
        return;
    }

}; //End ROV Manager Class




