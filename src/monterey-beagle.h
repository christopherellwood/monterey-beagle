#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
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
#include "i2c-dev.h"
#include <sys/ioctl.h>
#include "timer.c"
#include <math.h>

#define BUFFERLEN 64  //Length of receive buffer
#define NUM_MOTORS 3  //Number of motors
#define NUM_RELAYS 3  //Number of relays
#define NUM_SERVOS 1  //Number of servos

using namespace std;

class Vector
{
private:

public:
	float x, y ,z;
	float magnitude;
	Vector()
	{
		x = 0;
		y = 0;
		z = 0;
		magnitude = 0;
	}
	~Vector()
	{
	}
	void Set(float x_, float y_, float z_)
	{
		x = x_;
		y = y_;
		z = z_;
		Mag();
	}
	static Vector Cross(Vector a, Vector b)
	{
		Vector result;
		result.x = (a.y * b.z) - (a.z * b.y);
		result.y = (a.z * b.x) - (a.x * b.z);
		result.z = (a.x * b.y) - (a.y * b.x);
		result.Mag();
		return result;
	}
	static float Dot(Vector a, Vector b)
	{
		float result;
		result = a.x * b.x + a.y * b.y + a.z+ b.z;
		return result;
	}
	void Mag()
	{
		magnitude = sqrt(x * x  + y * y + z * z);
	}
	void Norm()
	{
		x = (x / magnitude);
		y = (y / magnitude);
		z = (z / magnitude);
		magnitude = 1;
	}
};

class LSM303DLHC
{
private:
	static const int ki2cBusFileLength = 12;

	//Magnetometer Constants
	static const int knumMagRegisters = 7;
	static const int kRunMag15Hz = 0x14;
	static const int kMagControlReg = 0x00;
	static const int kMagFirstDataReg = 0x02;

	//Accelerometer Constants
	static const int kRunAccel50Hz = 0x2700;
	static const int kRunAccelFullScale = 0x4000;
	static const int kAccelControlReg1 = 0x20;
	static const int kAccelControlReg4 = 0x23;
	static const int kAccelFirstDataReg = 0x28;
	static const int knumAccelRegisters = 6;
	char i2cBusFileName[ki2cBusFileLength];
	int MagAddress_;
	int AccelAddress_;
	char RawAccel[knumAccelRegisters];
	char RawMag[knumMagRegisters];


public:
	Vector M, A;
	LSM303DLHC()
	{
		MagAddress_ = 0;
		AccelAddress_ = 0;
		memset(RawMag, 0, knumMagRegisters);
		memset(RawAccel, 0, knumAccelRegisters);
		memset(i2cBusFileName, NULL, ki2cBusFileLength);
		memset(&A, 0, sizeof(A));
		memset(&M, 0, sizeof(M));
	}
	~LSM303DLHC()
	{

	}
	void Configure(int i2cBus, int MagAddress, int AccelAddress)
	{
		MagAddress_ = MagAddress;
		AccelAddress_ = AccelAddress;
		sprintf(i2cBusFileName, "/dev/i2c-%d", i2cBus);
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, MagAddress_) >= 0)
			{
				printf("LSM303DLHC Magnetometer enabled on address %x\n", MagAddress_);
				i2c_smbus_write_word_data(fp, kMagControlReg, kRunMag15Hz);
				i2c_smbus_write_word_data(fp, kMagFirstDataReg, 0x00);
			}
			else
			{
				printf("Error communicating with LSM303DLHC Magnetometer\n");
			}
			if (ioctl(fp, I2C_SLAVE, AccelAddress_) >= 0)
			{
				printf("LSM303DLHC Accelerometer enabled on address %x\n", AccelAddress_);
				i2c_smbus_write_word_data(fp, kAccelControlReg1, kRunAccel50Hz);
				i2c_smbus_write_word_data(fp, kAccelControlReg4, kRunAccelFullScale);
			}
			else
			{
				printf("Error communicating with LSM303DLHC Accelerometer");
			}
			close(fp);
		}
	}
	void ReadAccelRawData()
	{
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, AccelAddress_) >= 0)
			{
				int i;
				for (i = 0; i < knumAccelRegisters; i++)
				{
					RawAccel[i] = i2c_smbus_read_word_data(fp, kAccelFirstDataReg + i);
				}
				A.Set((float)((int16_t)((RawAccel[0] << 8) + RawAccel[1])),
					  (float)((int16_t)((RawAccel[2] << 8) + RawAccel[3])),
					  (float)((int16_t)((RawAccel[4] << 8) + RawAccel[5]))
					  );
				//printf("%.6f, %.6f, %.6f              \r",  A.x, A.y, A.z);
				close(fp);
			}
		}
	}
	void ReadMagRawData()
	{
		int fp;
		fp = open(i2cBusFileName, O_RDWR);

		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, MagAddress_) >= 0)
			{
				int i;
				for (i = 0; i < knumMagRegisters; i++)
				{
					RawMag[i] = i2c_smbus_read_word_data(fp, kMagFirstDataReg + i);
				}
				M.Set((float)((int16_t)((RawMag[1] << 8) + RawMag[2])),
					  (float)((int16_t)((RawMag[3] << 8) + RawMag[4])),
					  (float)((int16_t)((RawMag[5] << 8) + RawMag[6]))
					  );
				//printf("%.0f, %.0f, %.0f              \r",  M.x, M.y, M.z);
				close(fp);
			}
		}
	}
	int GetHeading()
	{
		/* This algorithm is taken from ryantm at:
		 * https://github.com/ryantm/LSM303DLH/blob/master/LSM303DLH/LSM303DLH.cpp
		 */
		int heading;
		Vector temp = A;
		temp.Norm();

		Vector::Cross(A, M);


		return heading;
	}
};

class IO_Pin
{
private:
	static const int kdirectionlength = 4; //Lengths need to be 1 longer then
	static const int kvaluelength = 2;     //the string to accommodate a NULL
	static const int kIDlength = 4;        //character for string operations
	static const int kDiscretePinfilelength = 64;
	static const int knumAM335xGPIO = 128;
	char direction_[kdirectionlength];
	char directionfile[kDiscretePinfilelength];
	char valuefile[kDiscretePinfilelength];
	char ID_[kIDlength];
	char value_[kvaluelength];

public:
	IO_Pin()
	{
	}
	~IO_Pin()
	{
		FILE * fp;
		fp = fopen("/sys/class/gpio/unexport", "ab");
		rewind(fp);
		fwrite(ID_, sizeof(char), kIDlength, fp);
		fclose(fp);
	}
	void Configure(char * ID, char * direction, int value)
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
			//initialize and zero-ize variables
			FILE * fp;
			bzero(directionfile, kDiscretePinfilelength);
			bzero(valuefile, kDiscretePinfilelength);
			bzero(direction_, kdirectionlength);
			bzero(ID_, kIDlength);
			bzero(value_, kvaluelength);
			strcpy(ID_, ID);
			strcpy(direction_, direction);

			//set access filenames
			sprintf(value_, "%d", value);
			sprintf(directionfile, "/sys/class/gpio/gpio%s/direction", ID_);
			sprintf(valuefile, "/sys/class/gpio/gpio%s/value", ID_);

			//create files in filesystem to write to
			fp = fopen("/sys/class/gpio/export", "ab");
			rewind(fp);
			fwrite(ID_, sizeof(char), kIDlength, fp);
			fclose(fp);

			//set direction
			fp = fopen(directionfile, "rb+");
			rewind(fp);
			fwrite(direction_, sizeof(char), kdirectionlength, fp);
			fclose(fp);

			//set initial value if it is an output
			if (strcmp(direction, "out") == 0)
			{
				fp = fopen(valuefile, "rb+");
				rewind(fp);
				fwrite(value_, sizeof(char), kvaluelength, fp);
				fclose(fp);
			}

		//print setup information to the user
			printf("IO %s set as %sput\n", ID_, direction_);
		}
		else
		{
			printf("Error: GPIO ID number out of bounds\n");
			printf("Error direction %s\n", direction);
		}
	}
	void Set(int value)
	{
		// Set the value of a pin (0 or 1)
		FILE * fp;
		sprintf(value_, "%d", value);
		fp = fopen(valuefile, "rb+");
		rewind(fp);
		fwrite(value_, sizeof(char), 1, fp);
		fclose(fp);
	}
	int Read()
	{
		FILE * fp;
		fp = fopen(valuefile, "rb+");
		rewind(fp);
		fread(value_, sizeof(char), 1, fp);
		return atoi(value_);
	}
};

class PWM_Pin
{

private:
	static const int kPWMfilelength = 64;
	static const int kvaluelength = 24;
	char period_[kvaluelength];
	char ID_[kvaluelength];
	char duty_[kvaluelength];
	char periodfile[kPWMfilelength];
	char dutyfile[kPWMfilelength];
	char polarityfile[kPWMfilelength];

public:
	PWM_Pin()
	{
	}
	~PWM_Pin()
	{
	}
	void Configure(char * ID, char * period, char * duty)
	{
		bzero(periodfile, kPWMfilelength);
		bzero(dutyfile, kPWMfilelength);
		bzero(polarityfile, kPWMfilelength);
		strcpy(ID_, ID);
		strcpy(period_, period);
		strcpy(duty_, duty);

		sprintf(periodfile, "/sys/devices/ocp.2/%s/period", ID_);
		sprintf(dutyfile, "/sys/devices/ocp.2/%s/duty", ID_);
		sprintf(polarityfile, "/sys/devices/ocp.2/%s/polarity", ID_);

		FILE * fp;
		fp = fopen(periodfile, "rb+");
		rewind(fp);
		fwrite(period_, sizeof(char), kvaluelength, fp);
		fclose(fp);

		fp = fopen(dutyfile, "rb+");
		rewind(fp);
		fwrite(duty, sizeof(char), kvaluelength, fp);
		fclose(fp);

		fp = fopen(polarityfile, "rb+");
		rewind(fp);
		fwrite("0", sizeof(char), 1, fp);
		fclose(fp);

		printf("PWM %s period %sns, duty %sns\n", ID_, period_, duty_);
	}
	void Set_Period(char * period)
	{
		FILE * fp;
		strcpy(period_, period);
		fp = fopen(periodfile, "rb+");
		rewind(fp);
		fwrite(period_, sizeof(char), 12, fp);
		fclose(fp);
	}
	void Set_Duty(char * duty)
	{
		FILE * fp;
		strcpy(duty_, duty);
		fp = fopen(dutyfile, "rb+");
		rewind(fp);
		fwrite(duty, sizeof(char), 12, fp);
		fclose(fp);
	}
};

class GPIO_Pins
{
public:
	IO_Pin Pin[4];
	PWM_Pin MotorPWM[3];
	PWM_Pin ServoPWM[3];
	GPIO_Pins()
	{
	}
	~GPIO_Pins()
	{
	}
	void ReadHardwareConfig()
	{
		Pin[0].Configure("68", "out", 0); //P8_10
		Pin[1].Configure("45", "out", 0); //P8_11
		Pin[2].Configure("44", "out", 0); //P8_12
		Pin[3].Configure("26", "out", 0); //P8_14

		MotorPWM[0].Configure("pwm_test_P8_13.11", "20000000", "1000000");
		MotorPWM[1].Configure("pwm_test_P8_19.12", "20000000", "1000000");
		MotorPWM[2].Configure("pwm_test_P9_14.13", "20000000", "1000000");
		ServoPWM[0].Configure("pwm_test_P9_16.14", "20000000", "1000000");
		ServoPWM[1].Configure("pwm_test_P9_21.15", "20000000", "1000000");
		ServoPWM[2].Configure("pwm_test_P9_22.16", "20000000", "1000000");
	}
};

struct Pointer_Set {
	/* This class exists because pthread_create can only pass one void argument
	 * to the subroutine it links to, and because that function must be static
	 * when it is a member of a class. Static functions are global, so they
	 * cannot see or act on data members of instances.
	 */
	void * connection;
	void * manager;
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
     * When not communicating with the operator, the manager has a 'sample
     * sensor' function to collect sensor data for telemetry.
     */
private:
    GPIO_Pins HardwarePinArray;
    LSM303DLHC DirectionSensor;
    UDP_Connection *connection_;
    int comms_thread_id;
    static const int kSensorWait = 20000; //uSeconds

    char sensor_string_[BUFFERLEN];

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
    		HardwarePinArray.MotorPWM[i].Set_Duty(a);
    	}
    	for (i=0; i<NUM_RELAYS; i++)
    	{
    		HardwarePinArray.Pin[i].Set(relay[i]);
    	}
    	for (i=0; i<NUM_SERVOS; i++)
    	{
    		char * a = new char[12];
    		sprintf(a, "%i", motor[i]*1000);
    		HardwarePinArray.ServoPWM[i].Set_Duty(a);
    	}
    }

public:
    ROV_Manager(UDP_Connection * UDP_Ptr)
	{
    	DirectionSensor.Configure(2, 0x1e, 0x19);
    	HardwarePinArray.ReadHardwareConfig();
		bzero((char *) &sensor_string_, BUFFERLEN);
		connection_ = UDP_Ptr;
	    comms_thread_id = 0;
	}
    ~ROV_Manager()
    {
    }
    void Start_Comms(void * ptr)
    {
    /*
     * This function initiates a new thread to listen for communications
     */
        pthread_t comms_thread;
        comms_thread_id = pthread_create(&comms_thread, NULL,
                Communications_Handler, (void *) ptr);
    }
    void Sample_Sensors()
    {
    	//while(1)
    	//{
    		DirectionSensor.ReadAccelRawData();
    		DirectionSensor.ReadMagRawData();
        	//usleep(kSensorWait);
        	//Hardware pin to get loop timing
    		HardwarePinArray.Pin[0].Set(1); //P8_10
        	HardwarePinArray.Pin[0].Set(0); //P8_10
    	//}
    }
}; //End ROV Manager Class

