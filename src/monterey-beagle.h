#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <cstdlib>
#include <pthread.h>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include "i2c-dev.h"
#include "timer.c"


#define BUFFERLEN 64  //Length of receive buffer
#define NUM_MOTORS 3  //Number of motors
#define NUM_RELAYS 3  //Number of relays
#define NUM_SERVOS 3  //Number of servos

using namespace std;

class SensorData
{
private:
	static const int kMaxArraySize = 255;
	float sum_;
	int arraysize_;
	int count_;
	float data_[kMaxArraySize];
	float conversionfactor_;

public:

	float average;
	SensorData()
	{
		conversionfactor_ = 1;
		arraysize_ = kMaxArraySize;
		average = 0;
		sum_ = 0;
		count_ = 0;
		memset(data_, 0, kMaxArraySize);
	}
	~SensorData()
	{
	}
	void Setup(int SampleArraySize, float ConversionFactor)
	{
		conversionfactor_ = ConversionFactor;
		arraysize_ = SampleArraySize;
	}
	void Update(float value)
	{
		value = value * conversionfactor_;
		sum_ -= data_[count_];
		data_[count_] = value;
		sum_ += value;
		average = sum_ / arraysize_;
		count_++;
		if (count_ == arraysize_)
		{
			count_ = 0;
		}
	}
};

class ADS1115
{
private:
	static const char kConversionReg = 0x00;
	static const char kConfigReg = 0x01;
	static const char kLowThresholdReg = 0x02;
	static const char kHighThresholdReg = 0x03;
	static const int kmax16bitvalue = 65536;

	//Converter Configuration Constants
	static const int kDoSingleConversion = 0x8000;
	static const int kMuxControlMask     = 0xFF8F;
	static const int kRateControlMask     = 0x1FFF;
	//Multiplexer setting
	static const int kDiff_Ain0_1 = 0x0000;
	static const int kDiff_Ain0_3 = 0x0010;
	static const int kDiff_Ain1_3 = 0x0020;
	static const int kDiff_Ain2_3 = 0x0030;
	static const int kSE_Ain0     = 0x0040;
	static const int kSE_Ain1     = 0x0050;
	static const int kSE_Ain2     = 0x0060;
	static const int kSE_Ain3     = 0x0070;

	//Programmable gain amplifier
	static const int kMax6v144 = 0x0000;
	static const int kMax4v096 = 0x0002;
	static const int kMax2v048 = 0x0004;
	static const int kMax1v024 = 0x0006;
	static const int kMax0v512 = 0x0008;
	static const int kMax0v256 = 0x000A;

	//Mode
	static const int kContinuousMode = 0x0000;
	static const int kPowerDownMode  = 0x0001;

	//Sample Rate
	static const int kRate8Hz    = 0x0000;
	static const int kRate16Hz   = 0x2000;
	static const int kRate32Hz   = 0x4000;
	static const int kRate64Hz   = 0x6000;
	static const int kRate128Hz  = 0x8000;
	static const int kRate250Hz  = 0xA000;
	static const int kRate475Hz  = 0xC000;
	static const int kRate860Hz  = 0xE000;

	//Comparator Settings
	static const int kCompNormal = 0x0000;
	static const int kCompWindow = 0x1000;
	static const int kCompActHigh= 0x0800;
	static const int kCompActLow = 0x0000;
	static const int kCompLatch  = 0x0400;
	static const int kCompNoLatch= 0x0000;
	static const int kCompFilter1= 0x0000;
	static const int kCompFilter2= 0x0100;
	static const int kCompFilter4= 0x0200;
	static const int kCompDisable= 0x0300;

	enum
	{
		ReadChannel0 = 0,
		ReadChannel1,
		ReadChannel2,
		ReadChannel3
	};
	enum
	{
		Rate8Hz = 0,
		Rate16Hz,
		Rate32Hz,
		Rate64Hz,
		Rate128Hz,
		Rate250Hz,
		Rate475Hz,
		Rate860Hz,
	};

	static const int kDataLength = 2;

	int configuration_;
	int Address_;
	static const int ki2cBusFileLength = 12;
	char i2cBusFileName[ki2cBusFileLength];
	unsigned char data_[kDataLength];
	int channel_;
	int channel0offset;
	int channel1offset;
	int channel2offset;
	int channel3offset;

public:
	int channel0;
	int channel1;
	int channel2;
	int channel3;

	ADS1115()
	{
		memset(i2cBusFileName, NULL, ki2cBusFileLength);
		memset(data_, NULL, kDataLength);
		Address_ = 0;
		channel0 = 0;
		channel1 = 0;
		channel2 = 0;
		channel3 = 0;
		channel0offset = 0;
		channel1offset = 0;
		channel2offset = 0;
		channel3offset = 0;
		channel_ = 0;
		configuration_ = 0;
	}
	~ADS1115()
	{
	}
	void Configure(int i2cBus, int Address)
	{
		Address_ = Address;
		sprintf(i2cBusFileName, "/dev/i2c-%d", i2cBus);
		int fp;
		//default configuration
		configuration_ = kSE_Ain0|kMax2v048|kContinuousMode|kRate128Hz|kCompDisable;

		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
			{
				i2c_smbus_write_word_data(fp, kConfigReg, configuration_);
				printf("ADS1115 ADC enabled on address %x\n", Address_);
			}
			else
			{
				printf("Error communicating with ADS1115 ADC\n");
			}
			close(fp);
		}
	}
	int ReadConfig()
	{
		int config = 0;
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
			{
				i2c_smbus_read_i2c_block_data(fp,
										kConfigReg,
										kDataLength,
										data_);
				config = (int)((data_[1]) | data_[0] << 8);
				close(fp);
			}
		}
		return config;
	}
	int ReadData()
	{
		int fp;
		int * channelptr;
		int * offsetptr;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
			{
				i2c_smbus_read_i2c_block_data(fp,
						kConversionReg,
										kDataLength,
										data_);
				switch (channel_)
				{
				case ReadChannel0:
					channelptr = &channel0;
					offsetptr = &channel0offset;
					break;
				case ReadChannel1:
					channelptr = &channel1;
					offsetptr = &channel1offset;
					break;
				case ReadChannel2:
					channelptr = &channel2;
					offsetptr = &channel2offset;
					break;
				case ReadChannel3:
					channelptr = &channel3;
					offsetptr = &channel3offset;
					break;
				default:
					break;
				}
				*channelptr = (int)((data_[1]) | data_[0] << 8);
				*channelptr += *offsetptr;
				close(fp);
			}
		}
		return *channelptr;
	}
	int GetMuxChannel()
	{
		return channel_;
	}
	void SetMuxChannel(int channel0_to_3)
	{
		if ((channel0_to_3 >= 0) || (channel0_to_3 <= 3))
		{
			//Reset mux control bits
			configuration_ &= kMuxControlMask;
			int fp;
			fp = open(i2cBusFileName, O_RDWR);
			if (fp >= 0)
			{
				if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
				{

					switch(channel0_to_3)
					{
						case ReadChannel0:
							configuration_ |= kSE_Ain0;
							break;
						case ReadChannel1:
							configuration_ |= kSE_Ain1;
							break;
						case ReadChannel2:
							configuration_ |= kSE_Ain2;
							break;
						case ReadChannel3:
							configuration_ |= kSE_Ain3;
							break;
						default:
							configuration_ |= kSE_Ain0;
							break;
					}
					i2c_smbus_write_word_data(fp, kConfigReg, configuration_);
					channel_ = channel0_to_3;
					close(fp);
				}
			}
		}
	}
	void SetConversionRate(int rate0_to_7)
	{
		if ((rate0_to_7 >= 0) || (rate0_to_7 <= 7))
		{
			//Reset rate control bits
			configuration_ &= kRateControlMask;
			int fp;
			fp = open(i2cBusFileName, O_RDWR);
			if (fp >= 0)
			{
				if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
				{
					switch(rate0_to_7)
					{
						case Rate8Hz:
							configuration_ |= kRate8Hz;
							break;
						case Rate16Hz:
							configuration_ |= kRate16Hz;
							break;
						case Rate32Hz:
							configuration_ |= kRate32Hz;
							break;
						case Rate64Hz:
							configuration_ |= kRate64Hz;
							break;
						case Rate128Hz:
							configuration_ |= kRate128Hz;
							break;
						case Rate250Hz:
							configuration_ |= kRate250Hz;
							break;
						case Rate475Hz:
							configuration_ |= kRate475Hz;
							break;
						case Rate860Hz:
							configuration_ |= kRate860Hz;
							break;
						default:
							configuration_ |= kRate128Hz;
							break;
					}
					i2c_smbus_write_word_data(fp, kConfigReg, configuration_);
					close(fp);
				}
			}
		}
	}
};

class Peripheral
{
private:
	volatile unsigned int *baseaddress;
public:
	Peripheral()
	{
		baseaddress = 0;
	}
	~Peripheral()
	{
	}
	volatile unsigned int * open_peripheral(__off_t baseAddress, size_t size)
	{
		int mem_fd;
		char *peripheral_map;
		volatile unsigned *peripheral;
	    /* open /dev/mem */
	    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
	            printf("can't open /dev/mem \n");
	            exit (-1);
	    }

	    /* mmap GPIO */
	    peripheral_map = (char *)mmap(
									0,
									size,
									PROT_READ|PROT_WRITE,
									MAP_SHARED,
									mem_fd,
									baseAddress
	    							);

	    if (peripheral_map == MAP_FAILED) {
	            printf("mmap error %d\n", (int)peripheral_map);
	            exit (-1);
	    }

	    // Always use the volatile pointer!
	    peripheral = (volatile unsigned int *)peripheral_map;
	    return peripheral;
	}
	volatile unsigned int readByOffset(volatile unsigned int * peripheral, unsigned int offset)
	{
		volatile unsigned int x;
		x = *(peripheral + (offset/4));
		return x;
	}
	void writeByOffset(volatile unsigned int * peripheral, unsigned int offset, unsigned int value)
	{
		*(peripheral + (offset/4)) = value;
	}
};

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
		result = a.x * b.x + a.y * b.y + a.z * b.z;
		return result;
	}
	void Mag()
	{
		magnitude = sqrt(x * x  + y * y + z * z);
	}
	void Normalize()
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
	static const char kReadbit = 0x80;

	//Magnetometer Constants
	static const char kMagControlReg = 0x00;
	static const char kMagGainReg = 0x01;
	static const char kMagConversionReg = 0x02;
	static const char kMagFirstDataReg = 0x03;
	static const char kMagTempRegH = 0x31;
	static const char kMagTempRegL = 0x32;
	static const int kEnableMagTemp = 0x8000;
	static const int kMagDataRate15Hz = 0x1000;
	static const int kMagDataRate30Hz = 0x1400;
	static const int kMagDataRate75Hz = 0x1800;
	static const int kMagDataRate220Hz = 0x1c00;
	static const int kMagGain1g3 = 0x2000;
	static const int kMagGain1g9 = 0x4000;
	static const int kMagGain2g5 = 0x6000;
	static const int kMagGain4g0 = 0x8000;
	static const int kMagGain4g7 = 0xa000;
	static const int kMagGain5g6 = 0xc000;
	static const int kMagGain8g1 = 0xe000;
	static const int kMagContinuous = 0x0000;
	static const int kMagSingle = 0x0100;
	static const int knumMagRegisters = 7;


	//Accelerometer Constants
	static const int kAccel200Hz = 0x6000;
	static const int kAccel100Hz = 0x5000;
	static const int kAccel50Hz = 0x4000;
	static const int kAccelEnableZ = 0x0400;
	static const int kAccelEnableY = 0x0200;
	static const int kAccelEnableX = 0x0100;
	static const int kAccelEnableAxes = 0x0700;
	static const int kAccelRange2g = 0x0000;
	static const int kAccelRange4g = 0x1000;
	static const int kAccelRange8g = 0x2000;
	static const int kAccelHighResMode = 0x0800;
	static const char kAccelControlReg1 = 0x20;
	static const char kAccelControlReg2 = 0x21;
	static const char kAccelControlReg3 = 0x22;
	static const char kAccelControlReg4 = 0x23;
	static const char kAccelFirstDataReg = 0x28;
	static const int knumAccelRegisters = 6;

	char i2cBusFileName[ki2cBusFileLength];
	int MagAddress_;
	int AccelAddress_;
	unsigned char temp_l;
	unsigned char temp_h;
	unsigned char RawAccel[knumAccelRegisters];
	unsigned char RawMag[knumMagRegisters];
	Vector Xaxis, Yaxis, Xhorizontal, Yhorizontal;

public:
	int16_t Temperature;
	float Roll, Pitch, Heading;
	Vector M, A;
	LSM303DLHC()
	{
		MagAddress_ = 0;
		AccelAddress_ = 0;
		temp_h = 0;
		temp_l = 0;
		Temperature = 0;
		Roll = 0;
		Pitch = 0;
		Heading = 0;
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
		Xaxis.Set(1,0,0);
		Yaxis.Set(0,1,0);
		MagAddress_ = MagAddress;
		AccelAddress_ = AccelAddress;
		sprintf(i2cBusFileName, "/dev/i2c-%d", i2cBus);
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, MagAddress_) >= 0)
			{

				i2c_smbus_write_word_data(fp, kMagControlReg,
											kEnableMagTemp |
											kMagDataRate75Hz);
				i2c_smbus_write_word_data(fp, kMagGainReg, kMagGain1g3);
				i2c_smbus_write_word_data(fp, kMagConversionReg, kMagContinuous);
				printf("LSM303DLHC Magnetometer enabled on address %x\n", MagAddress_);
			}
			else
			{
				printf("Error communicating with LSM303DLHC Magnetometer\n");
			}
			if (ioctl(fp, I2C_SLAVE, AccelAddress_) >= 0)
			{

				i2c_smbus_write_word_data(fp, kAccelControlReg1,
											kAccel100Hz |
											kAccelEnableAxes);
				i2c_smbus_write_word_data(fp, kAccelControlReg2, 0x0000);
				i2c_smbus_write_word_data(fp, kAccelControlReg3, 0x0000);
				i2c_smbus_write_word_data(fp, kAccelControlReg4,
											kAccelHighResMode |
											kAccelRange2g);
				printf("LSM303DLHC Accelerometer enabled on address %x\n", AccelAddress_);
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
		/* Or the Readbit (0x80 or 0b1000_0000) to the address of the first
		 * data register because Page 20 of the LSM303DLHC datasheet says:
		 * "In order to read multiple bytes, it is necessary to assert the most
		 * significant bit of the subaddress field. In other words, SUB(7) must
		 * be equal to 1 while SUB(6-0) represents the address of the first
		 * register to be read."
		 */
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, AccelAddress_) >= 0)
			{
				if (i2c_smbus_read_i2c_block_data(fp,
												kReadbit | kAccelFirstDataReg,
												knumAccelRegisters,
												RawAccel) < 0)
				{
					printf("Error reading Accelerometer\n");
				}
				A.Set((float)((int16_t)(RawAccel[0] | (RawAccel[1] << 8)) >>4),
					  (float)((int16_t)(RawAccel[2] | (RawAccel[3] << 8)) >>4),
					  (float)((int16_t)(RawAccel[4] | (RawAccel[5] << 8)) >>4)
					  );
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
				if (i2c_smbus_read_i2c_block_data(fp,
												kMagConversionReg,
												knumMagRegisters,
												RawMag) < 0)
				{
					printf("Error reading magnetometer\n");
				}

//				temp_h = i2c_smbus_read_word_data(fp, kMagTempRegH);
//				temp_l = i2c_smbus_read_word_data(fp, kMagTempRegL);
//				Temperature = (int16_t)((temp_h << 8) | temp_l);

				M.Set((float)( (int16_t)((RawMag[1] << 8) | RawMag[2]) ),
					  (float)( (int16_t)((RawMag[3] << 8) | RawMag[4]) ),
					  (float)( (int16_t)((RawMag[5] << 8) | RawMag[6]) )
					  );
				//printf("%.0f, %.0f, %.0f              \r",  M.x, M.y, M.z);
				close(fp);
			}
		}
	}
	float GetHeading()
	{
		/* Algorithm
		 * 1) Project the north vector into the horizonatal plane:
		 * As long as the platform is still, the acceleration vector is the
		 * Down Vector.
		 * Cross Magnetic Vector with Down Vector to get West Vector.
		 * Cross Down and West to get the North Vector in the horizontal plane.
		 *
		 * 2) Project the coordinate axes into the horizontal plane:
		 * Cross Yaxis with Down Vector to get X axis in horizontal plane
		 * Cross Xaxis with down to get Y axis in horizontal plane
		 *
		 * 3) Get scalar values for the % facing vs perpendicular to North:
		 * Dot X horizontal with North Vector to get the %facing to North
		 * Dot Y horizontal with North Vector to get the %perpendicular
		 * to North.
		 *
		 * 4) Calculate heading with trig function, and scale to degrees:
		 * Arctan(perpendicular/facing) * 180/pi = heading in degrees
		 * Arctan function gives answers +-180, so to get a heading ranging
		 * from 0 to 360, do (heading<0) +=360
		 */

		//1)
		Vector Down = A;
		Down.Normalize();
		M.Normalize();
		Vector W = Vector::Cross(M, Down);
		W.Normalize();
		Vector N = Vector::Cross(Down, W);
		N.Normalize();
		//2)
		Xhorizontal = Vector::Cross(Yaxis, Down);
		Xhorizontal.Normalize();
		Yhorizontal = Vector::Cross(Down, Xaxis);
		Yhorizontal.Normalize();
		//3
		float facingnorth;
		float perpendicularnorth;
		facingnorth = Vector::Dot(Xhorizontal, N);
		perpendicularnorth = Vector::Dot(Yhorizontal, N);
		//4
		Heading = atan2(perpendicularnorth, facingnorth) * 180 / 3.14159;
		if (Heading < 0)
		{
			Heading +=360;
		}
		return Heading;
	}
	float GetRoll()
	{
		/* Algorithm
		 * The arccos of two unit vectors is equal to the angle between them
		 * http://en.wikipedia.org/wiki/Vector_projection
		 */
		float Roll;
		Vector Down = A;
		Down.Normalize();
		Roll = acos(Vector::Dot(Yaxis, Down)) * -180 / 3.14159 + 90;
		return Roll;
	}
	float GetPitch()
	{
		/* Algorithm
		 * The arccos of two unit vectors is equal to the angle between them
		 * http://en.wikipedia.org/wiki/Vector_projection
		 */
		float Pitch;
		Vector Down = A;
		Down.Normalize();
		Pitch = acos(Vector::Dot(Xaxis, Down)) * -180 / 3.14159 + 90;
		return Pitch;
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
        //printf("%s \r", rxbuffer);
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
    UDP_Connection *connection_;
    LSM303DLHC DirectionSensor;
    ADS1115 ADC;
    int comms_thread_id;
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
              (i < (NUM_MOTORS + NUM_RELAYS + NUM_SERVOS)))
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
//		printf("\nMotors: ");
//		for(i=0;i<NUM_MOTORS; i++)
//		{
//			printf("%d ", motor[i]);
//		}
//			printf("\nRelays: ");
//		for(i=0;i<NUM_RELAYS; i++)
//		{
//			printf("%d ", relay[i]);
//		}
//			printf("\nServos: ");
//		for(i=0;i<NUM_SERVOS; i++)
//		{
//			printf("%d ", servo[i]);
//		}

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
    		int servopwm = 0;
    		//Monterey sends the servo deflection in degrees
    		//This line converts to microseconds
    		servopwm = 900 + (int)round(((float)servo[i]/180) * 1200);
    		//The pwm in the beaglebone needs it in nanoseconds
    		sprintf(a, "%i", servopwm * 1000);
    		//printf("%s \r", a);
    		HardwarePinArray.ServoPWM[i].Set_Duty(a);
    	}
    }

public:
    SensorData Heading;
    SensorData Roll;
    SensorData Pitch;
    SensorData Voltage;

    ROV_Manager(UDP_Connection * UDP_Ptr)
	{
    	/*Hardcoded Values*/
    	DirectionSensor.Configure(2, 0x1e, 0x19);
    	ADC.Configure(2, 0x48);
    	Heading.Setup(10, 1);
    	Roll.Setup(5, 1);
    	Pitch.Setup(5, 1);
    	Voltage.Setup(10, .0006875);
    	/*There are more hardcoded values in the hardware config*/
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
//		DirectionSensor.ReadMagRawData();
//		DirectionSensor.ReadAccelRawData();
    	Voltage.Update((float)ADC.ReadData());
    	//ADC.SetMuxChannel(0);

//		sprintf(sensor_string_, "1 1 %4.0f 12 %4.0f %4.0f",
//				DirectionSensor.GetHeading(),
//				DirectionSensor.GetRoll(),
//				DirectionSensor.GetPitch());

		sprintf(sensor_string_, "1 1 360 %7.2f 90 90",
				Voltage.average);
    	connection_->txbuffer = sensor_string_;

    }

}; //End ROV Manager Class

