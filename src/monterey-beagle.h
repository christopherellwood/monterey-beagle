#include <iostream> //needed?
#include <stdio.h> //needed?
#include <unistd.h>
#include <stdlib.h> //needed?
#include <fcntl.h>
#include <stdint.h> //needed?
#include <string.h>
#include <cstdlib>
#include <pthread.h> //needed?
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

using namespace std;
//TODO: Break up into header files
class PCA9685
{
	/*The PCA9685 shows up as two addresses. One is an individual address based
	 *on the device hardware straps, and ranges from 0x40 to 0x7f. The other is
	 *an all call, 0x70, which addresses all PCA9685's on the bus.*/
private:
	static const int ki2cBusFileLength 	= 12;
	static const float 	kClkFreq = 25000000;	//PCA9685 internal oscillator
	static const int 	kOutputFreqMin 	= 40;	//given in datasheet
	static const int 	kOutputFreqMax 	= 1000;	//given in datasheet
	static const float 	kServoMaxSteps 	= 246;	//Output is at 100% throttle
	static const short 	kServoMinSteps 	= 184;	//Output is at 0% throttle
	static const short 	kServoDelay		= 0;	//Delay before rising edge
	static const int 	kServoInitSteps = 0x0133;//Initial Throttle Setting
	static const int 	kStepsPerCycle 	= 4096; //Total steps for 12bit counter
	static const int 	kNumServos 		= 16;	//Number of outputs on PCA9685

	/*Register Addresses*/
	static const short kMode1Reg 	 = 0x00;//Starts and stops outputs
	static const short kMode2Reg 	 = 0x01;//Not used
	static const short kSub1Reg 	 = 0x02;//
	static const short kSub2Reg 	 = 0x03;//
	static const short kSub3Reg 	 = 0x04;//
	static const short kAllCallReg	 = 0x05;//
	static const short kPrescaleReg	 = 0xFE;//Prescale sets output frequency
	static const short kOn_Ch0_L_Reg = 0x06;//When ch0 turns on, bits 0:7
	static const short kOn_Ch0_H_Reg = 0x07;//When ch0 turns on, bits 8:12
	static const short kOff_Ch0_L_Reg= 0x08;//When ch0 turns off, bits 0:7
	static const short kOff_Ch0_H_Reg= 0x09;//When ch0 turns off, bits 8:12
											//The rest of the channels controls
	static const short kAddressOffset= 0x04;//are each offset by 4*n from ch0

	/*Commands*/
	static const int kZero			= 0x0000;
	static const int kStartOsc 		= 0x0100;
	static const int kStopOsc 		= 0x1100;
	static const int kMode2Default	= 0x0000; //0x0400 for totem pole output
	static const int kServoFreq		= 46;	//Tune to get 50Hz out
	static const int kAllCallDef	= 0xe000;
	static const int kFullOnOff		= 0x1000;

	short servo[kNumServos]; //There are 16 pwm outputs

	int Address_;
	float frequency_;
	char i2cBusFileName[ki2cBusFileLength];

	void WriteOutput(short addr, short data)
	{
		//printf("%02x, %04x\n", addr, data);
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
			{
				i2c_smbus_write_word_data(fp, addr, data);
				close(fp);
			}
		}
	}
	short OutputOffLAddr(int ServoNum)
	{
		short v;
		v = (short)((short)ServoNum * kAddressOffset + kOff_Ch0_L_Reg);
		return v;
	}
	short OutputOffHAddr(int ServoNum)
	{
		short v;
		v = (short)((short)ServoNum * kAddressOffset + kOff_Ch0_H_Reg);
		return  v;
	}
	short OutputOnLAddr(int ServoNum)
	{
		short v;
		v = (short)((short)ServoNum * kAddressOffset + kOn_Ch0_L_Reg);
		return  v;
	}
	short OutputOnHAddr(int ServoNum)
	{
		short v;
		v = (short)((short)ServoNum * kAddressOffset + kOn_Ch0_H_Reg);
		return  v;
	}
	void SetPWM(int PWMNum, short OnVal, short OffVal)
	{
		WriteOutput(OutputOnLAddr(PWMNum),  (short)(OnVal  << 8));
		WriteOutput(OutputOnHAddr(PWMNum),  (short) OnVal		);
		WriteOutput(OutputOffLAddr(PWMNum), (short)(OffVal << 8));
		WriteOutput(OutputOffHAddr(PWMNum), (short) OffVal		);
	}
	void SetOFF(int PWMNum, short OffVal)
	{
		WriteOutput(OutputOffLAddr(PWMNum), (short)(OffVal << 8));
		WriteOutput(OutputOffHAddr(PWMNum), (short) OffVal		);
	}
	void SetON(int PWMNum, short OnVal)
	{
		WriteOutput(OutputOnLAddr(PWMNum),  (short)(OnVal  << 8));
		WriteOutput(OutputOnHAddr(PWMNum),  (short) OnVal		);
	}
public:
	PCA9685()
	{
		Address_ = 0;
		memset(i2cBusFileName, NULL, ki2cBusFileLength);
		memset(servo, NULL, kNumServos);
		frequency_ = (float)kServoFreq;
	}
	~PCA9685()
	{
	}
	void RC_PWM_Setup(int i2cBus, int Address)
	{
		Address_ = Address;
		short prescale = 0;
		prescale = (short)round(kClkFreq / (kStepsPerCycle * kServoFreq));
		prescale = prescale << 8;

		sprintf(i2cBusFileName, "/dev/i2c-%d", i2cBus);
		int fp;
		fp = open(i2cBusFileName, O_RDWR);
		if (fp >= 0)
		{
			if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
			{
				i2c_smbus_write_word_data(fp, kMode1Reg, kStopOsc);
				i2c_smbus_write_word_data(fp, kMode2Reg, kMode2Default);
				i2c_smbus_write_word_data(fp, kSub1Reg, kZero);
				i2c_smbus_write_word_data(fp, kSub2Reg, kZero);
				i2c_smbus_write_word_data(fp, kSub3Reg, kZero);
				i2c_smbus_write_word_data(fp, kPrescaleReg, prescale);
				i2c_smbus_write_word_data(fp, kMode1Reg, kStartOsc);
				close (fp);
				int i;
				for (i = 0; i < kNumServos; i++)
				{
					//Init motors/servos to initialize at 1500us
					SetPWM(i, kServoDelay, kServoInitSteps);
				}
				printf("PCA9685 PWM Controller enabled on address %x\n", Address_);
			}
			else
			{
				printf("Error communicating with PCA9685 PWM Controller\n");
			}
		}
	}
	void SetFrequency(float frequency)
	{
		//Datasheet says frequency min/max is 40/1000
		if ((frequency >= kOutputFreqMin) && (frequency <= kOutputFreqMax))
		{
			short prescale = 0;
			prescale = (short)round(kClkFreq / (kStepsPerCycle * frequency));
			frequency_ = frequency;
			int fp;
			fp = open(i2cBusFileName, O_RDWR);
			if (fp >= 0)
			{
				if (ioctl(fp, I2C_SLAVE, Address_) >= 0)
				{
					i2c_smbus_write_word_data(fp, kMode1Reg, kStopOsc);
					i2c_smbus_write_word_data(fp, kPrescaleReg, prescale);
					i2c_smbus_write_word_data(fp, kMode1Reg, kStartOsc);
					close(fp);
				}
			}
		}
		else
		{
			printf("Error! Frequency must be between 40 and 1000Hz.\n");
		}
	}
	void Set_RC_Throttle(int ServoNum, float Percent)
	{
		if ((ServoNum >= 0) && (ServoNum <=15))
		{
			servo[ServoNum] = (short)round((Percent / 100) * kServoMaxSteps);
			servo[ServoNum] = kServoDelay + kServoMinSteps + servo[ServoNum];
			SetOFF(ServoNum, servo[ServoNum]);
		}
		else
		{
			printf("Error! Servo number must be between 0 and 15.\n");
		}
	}
	void SetThrottle_us(int ServoNum, int onTime)
	{
		short counts;
		counts = (float)onTime * frequency_ * (float)kStepsPerCycle / 1000000;
		SetPWM(ServoNum, kServoDelay, counts);
	}
	void Set_Discrete(int ServoNum, int value)
	{
		switch (value)
		{
		case 0:
			WriteOutput(OutputOnHAddr(ServoNum), 0x0000);
			WriteOutput(OutputOffHAddr(ServoNum), 0x1000);
			break;
		case 1:
			WriteOutput(OutputOffHAddr(ServoNum), 0x0000);
			WriteOutput(OutputOnHAddr(ServoNum), 0x1000);
			break;
		}
	}
};

class SensorData
{
private:
	static const int kMaxArraySize = 255;
	float sum_;
	int arraysize_;
	int count_;
	float data_[kMaxArraySize];
	float conversionfactor_;
	float offset_;

public:

	float average;
	SensorData()
	{
		offset_ = 0;
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
	void Setup(int SampleArraySize, float ConversionFactor, float offset)
	{
		conversionfactor_ = ConversionFactor;
		arraysize_ = SampleArraySize;
		offset_ = offset;
	}
	void Update(float value)
	{
		value = value * conversionfactor_ + offset_;
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
	static const int knum_channels = 4;
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
		/*********************************************************************/
		/* These vectors define the X and Y axes of the vehicle relative to the
		 * LSM303DLHC. This is necessary to determine the angle between the
		 * vehicle forward axis and magnetic north.
		 *
		 * Calibrate the X axis to the vehicle front. look at the labeling on
		 * the top of LSM303DLHC, and put a unit vector in the direction of the
		 * vehicle front relative to the LSM303DLHC. For example, if the front
		 * of the vehicle is in the negative X direction from the X axis on the
		 * LSM303DLHC, then X axis is (-1,0,0).
		 *
		 * Calibrate the Y axis to the left side of the vehicle, using the same
		 * technique. For example, if the left side of the vehicle is the
		 * toward -Z on the LSM303DLHC, then Y axis is (0,0,-1)
		 *
		 * TODO: Make the following X and Y axis transforms not hard coded, or
		 * put them in the magic numbers section
		 */
		Xaxis.Set(-1,0,0); //X-axis for the vehicle relative to the LSM303DLHC
		Yaxis.Set(0,0,-1); //Y-axis for the vehicle relative to the LSM303DLHC
		/********************************************************************/

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
					  (float)( (int16_t)((RawMag[5] << 8) | RawMag[6]) ),
					  (float)( (int16_t)((RawMag[3] << 8) | RawMag[4]) )
					  );
				//printf("%04.2f, %04.2f, %04.2f\n", M.x, M.y, M.z);
				close(fp);
			}
		}
	}
	float GetHeading()
	{
		/* Algorithm
		 * 1) Project the north vector into the horizontal plane:
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
	static const int internal = 0; 			//denotes PWM pins on AM3359 chip
	static const int external = 1; 			//Output on PCA9685 I2C expansion
	static const int kdirectionlength = 4; 	//Lengths need to be 1 longer then
	static const int kvaluelength = 2;     	//the string to accommodate a NULL
	static const int kIDlength = 4;        	//character for string operations
	static const int kDiscretePinfilelength = 64;
	static const int knumAM335xGPIO = 128;
	static const int knumParameters = 6;	//# of config parameters
	int hardwaretype_;
	int invert_;
	int intValue_;
	int id_;
	char direction_[kdirectionlength];
	char directionfile[kDiscretePinfilelength];
	char valuefile[kDiscretePinfilelength];
	char ID_[kIDlength];
	char value_[kvaluelength];
	PCA9685 *extPin;

	int CheckInvert(int val)
	{
		if (invert_)
		{
			switch (val)
			{
			case 0:
				val = 1;
				break;
			default:
				val = 0;
				break;
			}
		}
		return val;
	}
	void ConfigureInternal()
	{
		//initialize and zero-ize variables
		FILE * fp;
		bzero(directionfile, kDiscretePinfilelength);
		bzero(valuefile, kDiscretePinfilelength);

		//set access filenames
		sprintf(value_, "%s", CheckInvert(intValue_));
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
		if (strcmp(direction_, "out") == 0)
		{
			fp = fopen(valuefile, "rb+");
			rewind(fp);
			fwrite(value_, sizeof(char), kvaluelength, fp);
			fclose(fp);
		}

		//print setup information to the user
		//printf("IO %s set as %sput\n", ID_, direction_);
	}
	void ConfigurePCA9685(void *ptr)
	{
		id_ = (int)strtol(ID_, NULL, 10);
		extPin = (PCA9685*)ptr;
		extPin->Set_Discrete(id_, CheckInvert(intValue_));
	}
public:
	int C_Index;
	IO_Pin()
	{
		intValue_ = 0;
		invert_ = 0;
		hardwaretype_ = 0;
		C_Index = -1;
		id_ = 0;
		extPin = NULL;
	}
	~IO_Pin()
	{
		FILE * fp;
		fp = fopen("/sys/class/gpio/unexport", "ab");
		rewind(fp);
		fwrite(ID_, sizeof(char), kIDlength, fp);
		fclose(fp);
	}
	void Config_Pin(char *fileread, int length, void * ptr)
	{
		int period = 0;
		int ontime = 0;
		char token[knumParameters][kIDlength];
		char line[length];
		strcpy(line, fileread);
		line[strcspn(line, "\n")] = 0; //Set the \n character to a NULL

		int i = 0;
		int j;
		int param = 0;
		for (param = 0; param < knumParameters; param++)
		{
			bzero(token[param], kIDlength);
			j = 0;
			while ((line[i] != ' ') && (line[i] != NULL) && (i < length))
			{
				token[param][j] = line[i];
				j++;
				i++;
			}
			while ((line[i] == ' ') && (line[i] != NULL) && (i < length))
			{
				i++;
			}
		}
		strcpy(ID_, token[0]);
		strcpy(direction_, token[1]);
		intValue_ = (int)strtol(token[2], NULL, 10);
		hardwaretype_ = (int)strtol(token[3], NULL, 10);
		invert_ = (int)strtod(token[4], NULL);
		C_Index = (int)strtol(token[5], NULL, 10);

		switch (hardwaretype_)
		{
		case internal:
			ConfigureInternal();
			break;
		case external:
			ConfigurePCA9685(ptr);
			break;
		default:
			break;
		}
	}
	void Set(int value)
	{
		intValue_ = value;
		switch (hardwaretype_)
		{
		case 0:
			// Set the value of a pin (0 or 1)
			FILE * fp;
			sprintf(value_, "%d", CheckInvert(intValue_));
			fp = fopen(valuefile, "rb+");
			rewind(fp);
			fwrite(value_, sizeof(char), 1, fp);
			fclose(fp);
			break;
		case 1:
			extPin->Set_Discrete(id_, CheckInvert(intValue_));
			break;
		default:
			break;
		}
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
	static const int RC_Min_us 		= 900;	//RC PWM min ontime in us
	static const int RC_Max_us 		= 2100;	//RC PWM max ontime in us
	static const int kTrimNeutral 	= 50;	//trim is "neutral"
	static const int internal 		= 0;	//denotes PWM pins on AM3359 chip
	static const int external 		= 1;	//PWM pin on PCA9685 I2C expansion
	static const int kPWMfilelength = 64;	//long enough to hold a filename
	static const int kvaluelength 	= 24;	//long enough to hold name values

	static const int knumParameters = 7;	//# of configuration parameters

	PCA9685 *extPWM;
	char period_[kvaluelength];
	char ID_[kvaluelength];
	char duty_[kvaluelength];
	char periodfile[kPWMfilelength];
	char dutyfile[kPWMfilelength];
	char polarityfile[kPWMfilelength];
	int iPeriod;
	int id;
	int hardwaretype_;
	float T_coef;

	void ConfigurePCA9685(char * ID, void * ptr, int initPeriod_us , int initOntime_us)
	{
		id = (int)strtol(ID, NULL, 10);
		extPWM = (PCA9685*)ptr;
		extPWM->SetThrottle_us(id, initOntime_us);
	}
	void ConfigureInternal(char * ID, int initPeriod_us , int initOntime_us)
	{

		FILE * fp;
		bzero(periodfile, kPWMfilelength);
		bzero(dutyfile, kPWMfilelength);
		bzero(polarityfile, kPWMfilelength);
		strcpy(ID_, ID);

		//The drivers add a .## to the end of the pwm control folders. The OS
		//can change these numbers on a whim, so this code searches for the
		//file with a wildcard and returns whatever the current number is.
		sprintf(periodfile, "echo /sys/devices/ocp*/%s*/period", ID_);
		sprintf(dutyfile, "echo /sys/devices/ocp*/%s*/duty", ID_);
		sprintf(polarityfile, "echo /sys/devices/ocp*/%s*/polarity", ID_);

		fp = popen(periodfile, "r");
		fgets(periodfile, kPWMfilelength, fp);
		periodfile[strcspn(periodfile, "\n")] = 0;
		pclose(fp);

		fp = popen(dutyfile, "r");
		fgets(dutyfile, kPWMfilelength, fp);
		dutyfile[strcspn(dutyfile, "\n")] = 0;
		pclose(fp);

		fp = popen(polarityfile, "r");
		fgets(polarityfile, kPWMfilelength, fp);
		polarityfile[strcspn(polarityfile, "\n")] = 0;
		pclose(fp);
		//With the drivers found, we can go ahead and set the initial values

		fp = fopen(polarityfile, "rb+");
		rewind(fp);
		fwrite("0", sizeof(char), 1, fp);
		fclose(fp);

		Set_Period_us(initPeriod_us);
		Set_OnTime_us(initOntime_us);

		//printf("PWM %s period %sns, duty %sns\n", ID_, period_, duty_);
	}
public:
	int C_Index;
	int T_Index;
	PWM_Pin()
	{
		iPeriod = 0;
		id = 0;
		extPWM = NULL;
		C_Index = -1;
		T_Index = -1;
		T_coef = 1;
		hardwaretype_ = 0;
	}
	~PWM_Pin()
	{
	}
	void Config_Pin(char *fileread, int length, void * ptr)
	{
		int period = 0;
		int ontime = 0;
		char token[knumParameters][kvaluelength];
		char line[length];
		strcpy(line, fileread);
		line[strcspn(line, "\n")] = 0; //Set the \n character to a NULL

		int i = 0;
		int j;
		int param = 0;
		for (param = 0; param < knumParameters; param++)
		{
			bzero(token[param], kvaluelength);
			j = 0;
			while ((line[i] != ' ') && (line[i] != NULL) && (i < length))
			{
				token[param][j] = line[i];
				j++;
				i++;
			}
			while ((line[i] == ' ') && (line[i] != NULL) && (i < length))
			{
				i++;
			}
		}
		strcpy(ID_, token[0]);
		hardwaretype_ = (int)strtol(token[1], NULL, 10);
		C_Index = (int)strtol(token[2], NULL, 10);
		T_Index = (int)strtol(token[3], NULL, 10);
		T_coef = (float)strtod(token[4], NULL);
		period = (int)strtol(token[5], NULL, 10);
		ontime = (int)strtol(token[6], NULL, 10);

		switch (hardwaretype_)
		{
		case internal:
			ConfigureInternal(ID_, period , ontime);
			break;
		case external:
			ConfigurePCA9685(ID_, ptr, (int)period , (int)ontime);
			break;
		default:
			break;
		}
		//printf("%15s period %sns, duty %sns\n", ID_, period_, duty_);
	}
	void Set_Period_us(int Period)
	{
		switch (hardwaretype_)
		{
		case internal:
			iPeriod = Period;
			Period = Period * 1000; //convert us to ns
			sprintf(period_, "%d", Period);
			FILE * fp;
			fp = fopen(periodfile, "rb+");
			rewind(fp);
			fwrite(period_, sizeof(char), kvaluelength, fp);
			fclose(fp);
			break;
		case external:
			float f;
			f = 1 / ((float)Period);
			extPWM->SetFrequency(f);
			break;
		default:
			break;
		}
	}
	void Set_OnTime_us(int onTime)
	{
		switch (hardwaretype_)
		{
		case internal:
			onTime = onTime * 1000; //convert us to ns
			sprintf(duty_, "%d", onTime);
			FILE * fp;
			fp = fopen(dutyfile, "rb+");
			rewind(fp);
			fwrite(duty_, sizeof(char), kvaluelength, fp);
			fclose(fp);
			break;
		case external:
			extPWM->SetThrottle_us(id, onTime);
			break;
		default:
			break;
		}
	}
	void Set_RC_Throttle(float percent)
	{
		switch (hardwaretype_)
		{
		case internal:
			int onTime;
			/* HACK WARNING!
			 * Hardware change means that all internal PWM pins are now basic
			 * PWM or 0-100% of iPeriod. No RC. The easiest way to change the
			 * software is by changing the following line of code.
			 * TODO: Fix this throttle control chain starting at the ROV
			 * controls object all the way down to this line to make this
			 * change clear so the code doesn't seem hacked.
			 */
			//onTime = (int)round(percent*(RC_Max_us-RC_Min_us)/100 + RC_Min_us);
			onTime = (int)round(percent*(iPeriod)/100);

			onTime = onTime * 1000; //convert us to ns
			//printf("Period is: %d * 1000, On Time is: %d\n", iPeriod, onTime);
			sprintf(duty_, "%d", onTime);
			FILE * fp;
			fp = fopen(dutyfile, "rb+");
			rewind(fp);
			fwrite(duty_, sizeof(char), kvaluelength, fp);
			fclose(fp);
			break;
		case external:
			extPWM->Set_RC_Throttle(id, percent);
			break;
		default:
			break;
		}
	}
	void Set_RC_Throttle_with_Trim(float command, float trim)
	{
		command = command + T_coef * (trim - kTrimNeutral);
		if (command < 0) command = 0;
		if (command > 100) command = 100;
		Set_RC_Throttle(command);
	}
};

class ROV_Controls
{
/* This class creates an array of pwm and discrete pins (they control the ROV).
 * It reads a configuration file and sends the lines to the PWM and discrete
 * pin objects to be parsed and stored. It also provides an interface to apply
 * the command array from the ROV object to those pin objects.
 *
 */
private:
	static const int numPWM = 22;		//6 internal, 15 external PWM
	static const int numDiscrete = 22;	//15 internal, 6 external discrete
	static const int knameLength = 16;	//longer than longest pwm name
	static const int klineLength = 80;	//length of config file lines
	PCA9685 IOExpansion;				//16 Channel I2C IO expander

public:
	IO_Pin Pin[numDiscrete];
	PWM_Pin PWM[numPWM];

	ROV_Controls()
	{
	}
	~ROV_Controls()
	{
	}
	void SetupOutputs(int extPWMbus, int extPWMaddr)
	{
	/*This function opens the Disc_Pins.cfg and PWM_Pins.cfg files and reads
	 * them line by line. Each line is fed in turn to a PWM or Discrete pin
	 * control object, where it is parsed and used to initialize that pin.
	 */
		IOExpansion.RC_PWM_Setup(extPWMbus, extPWMaddr);

		FILE *fp;
		char line[klineLength];
		fp = fopen("PWM_Pins.cfg", "r");

		int i;
		for (i = 0; i < numPWM; i++)
		{
			bzero(line, klineLength);
			fgets(line, klineLength, fp);
			PWM[i].Config_Pin(line, klineLength, &IOExpansion);
		}
		fclose(fp);

		fp = fopen("Disc_Pins.cfg", "r");
		for (i = 0; i < numDiscrete; i++)
		{
			fgets(line, klineLength, fp);
			Pin[i].Config_Pin(line, klineLength, &IOExpansion);
		}
		fclose(fp);
	}
	void ApplyCommands(int *CommandList)
	{
	/* Commands are sent as an integer array. Each pin has a command array
	 * index which tells it what command array position carries its
	 * instruction. The config files allow the pins to be reorganized to
	 * respond to different commands.
	 */
		int i;
		for (i = 0; i < numPWM; i++)
		{
			if (PWM[i].C_Index > -1)
			{
				float command = (float)CommandList[PWM[i].C_Index];
				if (PWM[i].T_Index > -1)
				{
					float trim = (float)CommandList[PWM[i].T_Index];
					PWM[i].Set_RC_Throttle_with_Trim(command, trim);
				}
				else
				{
					PWM[i].Set_RC_Throttle(command);
				}
			}
		}
		for (i = 0; i < numDiscrete; i++)
		{
			if (Pin[i].C_Index > -1)
			{
				int command = CommandList[Pin[i].C_Index];
				Pin[i].Set(command);
			}
		}
	}
};

struct Pointer_Set
{
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
        //printf("received: %s \n", rxbuffer);
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
            //printf("Transmit Error!\n");
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
	/*TODO: Change name from Deep One to Tashtego, or to a generic name*/
	ROV_Controls Controls;		//Interface to PWM and discrete outputs
    UDP_Connection *connection_;	//Pointer to the comms handler object
    LSM303DLHC DirectionSensor; 	//Compass + 3 axis accelerometer
    ADS1115 ADC0;					//16 bit, 4 channel ADC's.
    ADS1115 ADC1;					//
    int comms_thread_id;			//ID for the second thread
    int channel_;					//Read one ADC channel per ADC, per loop
    int commands[BUFFERLEN];		//Command list from the surface
    char sensor_string_[BUFFERLEN];	//Telemetry string to send to the surface

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
    	 * space separated character string
    	 */

        int i=0;
        std::istringstream iss(rxdata);
        while((iss >> commands[i]) && (i < BUFFERLEN))
        {
        	//printf("%d ", commands[i]);
            i++;
        }
        //printf("\n");
        return;
    }
    void Apply_Commands()
    {
    	Controls.ApplyCommands(commands);
    }

public:
    /* SensorData's are data structures for applying a conversion constant and
     * offset for raw data from a sensor, and updating a moving average to
     * smooth out noise.*/
    SensorData Heading;			//LSM303DLHC Magnetic heading in degrees
    SensorData Roll;			//LSM303DLHC Roll, based on gravity vector
    SensorData Pitch;			//LSM303DLHC Pitch, based on gravity vector
    SensorData Voltage;			//ADC0 ch2 Bus Voltage
    SensorData BusCurrent;		//ADC0 ch0 Bus Current
    SensorData BatteryCurrent;	//ADC0 ch1 Battery Current
    SensorData Depth;			//ADC0 ch3 Depth by pressure
    SensorData Temperature;		//ADC1 ch3
    SensorData Sensor0;			//ADC1 ch0
    SensorData Sensor1;			//ADC1 ch1
    SensorData Sensor2;			//ADC1 ch2

    ROV_Manager(UDP_Connection * UDP_Ptr)
	{
    	/*Initialize memory*/
		bzero((char *) &sensor_string_, BUFFERLEN);
		bzero((int *) &commands, BUFFERLEN);
		connection_ = UDP_Ptr;
	    comms_thread_id = 0;
	    channel_ = 0;
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
    void Setup_Hardware_with_Magic_Numbers()
    {
    	/*Hardcoded Values for sensor addresses and things like that*/
    	DirectionSensor.Configure(2, 0x1e, 0x19);
    	ADC0.Configure(2, 0x48);
    	ADC0.SetMuxChannel(channel_);
    	ADC1.Configure(2, 0x49);
    	ADC1.SetMuxChannel(channel_);

    	/*Configure sensor data structures*/
    	/*LSM303DLHC*/
    	Heading.Setup(10, 1, 0);
    	Roll.Setup(5, 1, 0);
    	Pitch.Setup(5, 1, 0);

    	/*ADC0*/
    	Voltage.Setup(10, .0004817, 0);
    	BusCurrent.Setup(10, .0025, -1.7);
    	BatteryCurrent.Setup(10, .00125, -1.7);
    	Depth.Setup(10, .0001, 0);

    	/*ADC1*/
    	Sensor0.Setup(10, .0001, 0);
    	Sensor1.Setup(10, .0001, 0);
    	Sensor2.Setup(10, .0001, 0);
    	Temperature.Setup(10, .00625, -50);

    	/*Hardware interface*/
    	Controls.SetupOutputs(2, 0x40);

    }
    void Sample_Sensors()
    {
    	/*Start by clearing out any old data*/
    	bzero((char *) &sensor_string_, BUFFERLEN);

    	/*Read LSM303DLHC*/
    	DirectionSensor.ReadMagRawData();
		DirectionSensor.ReadAccelRawData();
		Roll.Update(DirectionSensor.GetRoll());
		Pitch.Update(DirectionSensor.GetPitch());
		Heading.Update(DirectionSensor.GetHeading());

		/* Read ADC's. The I2C bus has a lot of sensors on it, so only sample
		 * what is needed per ADC per program loop. The "channel" variable is
		 * just a counter. The ADCs can be set to any channel and read in any
		 * order.*/
		switch (channel_)
		{
		case 0:
			BusCurrent.Update((float)ADC0.ReadData());
			break;
		case 1:
			BatteryCurrent.Update((float)ADC0.ReadData());
			break;
		case 2:
			Voltage.Update((float)ADC0.ReadData());
			break;
		case 3:
			Depth.Update((float)ADC0.ReadData());
			Temperature.Update((float)ADC1.ReadData());
			break;
		default:
			break;
		}
		channel_++;
		if (channel_ == ADC0.knum_channels)
		{
			channel_ = 0;
		}
		ADC0.SetMuxChannel(channel_);
		ADC1.SetMuxChannel(channel_);

		/*Print sensor data to console to help with debug*/
//    	printf("H:%.0fdeg R:%.0fdeg P:%.0fdeg "
//    			"T:%.1fC "
//    			"VBus:%.1fV IBus:%.2fA IBatt:%.2fA D:%.0fm\n",
//    			Heading.average, Roll.average, Pitch.average,
//				Temperature.average,
//				Voltage.average, BusCurrent.average,
//				BatteryCurrent.average, Depth.average);

    	/*Convert floating point sensor data to a string for transmission*/
    	sprintf(sensor_string_, "1.0 %.0f %.1f %.2f %.0f 1",
				Depth.average, Voltage.average, BusCurrent.average,
				Heading.average);
//		Test data generator
//    	i++;
//    	if (i>10) i = 0;
//		sprintf(sensor_string_, "%d %d %d %d %d %d ", i, i+1, i+2, i+3, i+4, i+5);

    	/*Load the data transmit buffer with the sensor string*/
    	connection_->txbuffer = sensor_string_;
    }

}; //End ROV Manager Class

