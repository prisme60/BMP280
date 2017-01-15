/*  (Dev)   -  (Pi)
    SDA     -  SDA
    SCL     -  SCL
    GND     -  GND
    VCC     -  3.3V
    Note: Check your pin out
    Note: Make sure you connect the PI's 3.3 V line to the BMP280 boards Vcc 'IN' line not the 3.3v 'OUT' 
    
    How to compile, @ command line type
    
        gcc -Wall -o BMP280 ./BMP280.c
    
    for constants such as O_RWRD or I2C_M_RD checkout i2c.h & i2c-dev.h
    this also contains the definition of 'struct i2c_msg' so if you want to see what is
    possible check it out. 
    also have a look at
    
>>>>>  https://www.kernel.org/doc/Documentation/i2c/i2c-protocol <<<<<<<<<< NB! read it 
    
    There are no examples of the 'Write User Register' available, so back to basics and RTFM to
    create it from the above documentation.
           
    Use as you see fit.
    
    Christian Foucher 2017-01-15
*/
     
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#define sleepms(ms)  usleep((ms)*1000)   
#define I2CBus             "/dev/i2c-1"      //New Pi's 
//#define I2CBus             "/dev/i2c-0"    //Old, but not stale Pi's

//power mode
typedef enum {
	POWER_MODE_SLEEP=0,
	POWER_MODE_FORCED1=1,
	POWER_MODE_FORCED2=2,
	POWER_MODE_NORMAL=3
} tPowerMode ;

//temperature resolution
typedef enum {
	OSRS_TEMPERATURE_SKIPPED = 0,
	OSRS_TEMPERATURE_16BITS = 1,
	OSRS_TEMPERATURE_17BITS = 2,
	OSRS_TEMPERATURE_18BITS = 3,
	OSRS_TEMPERATURE_19BITS = 4,
	OSRS_TEMPERATURE_20BITS = 5,
} tOsrsTemperature;

// Pressure resolution
typedef enum {
	OSRS_PRESSURE_SKIPPED = 0, // pressure measurement skipped
	OSRS_PRESSURE_16BITS = 1,   // 16 Bits ultra low power
	OSRS_PRESSURE_17BITS = 2,   // 17 Bits low power
	OSRS_PRESSURE_18BITS = 3,   // 18 Bits standard resolution
	OSRS_PRESSURE_19BITS = 4,   // 19 Bits high resolution
	OSRS_PRESSURE_20BITS = 5,   // 20 Bits ultra high resolution
} tOsrsPressure;

// filter settings
typedef enum {
	FILTER0 = 0,
	FILTER1 = 1,
	FILTER2 = 2,
	FILTER3 = 3,
	FILTER4 = 4, // default
	FILTER5 = 5,
	FILTER6 = 6,
	FILTER7 = 7,
} tFilter;

// standby settings
typedef enum {
	STANDBY_0000_5_MS = 0, //    0.5 ms
	STANDBY_0062_5_MS = 1, //   62.5 ms
	STANDBY_0125_0_MS = 2, //  125   ms
	STANDBY_0250_0_MS = 3, //  250   ms
	STANDBY_0500_0_MS = 4, //  500   ms
	STANDBY_1000_0_MS = 5, // 1000   ms
	STANDBY_2000_0_MS = 6, // 2000   ms
	STANDBY_4000_0_MS = 7, // 4000   ms
} tStandby;

// BMP280 registers
typedef enum {
	BMP280_REGISTER_DIG_T1 = 0x88,
	BMP280_REGISTER_DIG_T2 = 0x8A,
	BMP280_REGISTER_DIG_T3 = 0x8C,
	BMP280_REGISTER_DIG_P1 = 0x8E,
	BMP280_REGISTER_DIG_P2 = 0x90,
	BMP280_REGISTER_DIG_P3 = 0x92,
	BMP280_REGISTER_DIG_P4 = 0x94,
	BMP280_REGISTER_DIG_P5 = 0x96,
	BMP280_REGISTER_DIG_P6 = 0x98,
	BMP280_REGISTER_DIG_P7 = 0x9A,
	BMP280_REGISTER_DIG_P8 = 0x9C,
	BMP280_REGISTER_DIG_P9 = 0x9E,
	BMP280_REGISTER_CHIPID = 0xD0,
	BMP280_REGISTER_VERSION = 0xD1,
	BMP280_REGISTER_SOFTRESET = 0xE0,
	BMP280_REGISTER_CONTROL = 0xF4,
	BMP280_REGISTER_CONFIG  = 0xF5,
	BMP280_REGISTER_STATUS = 0xF3,
	BMP280_REGISTER_PRESSDATA_MSB = 0xF7,
	BMP280_REGISTER_PRESSDATA_LSB = 0xF8,
	BMP280_REGISTER_PRESSDATA_XLSB = 0xF9,
	BMP280_REGISTER_TEMPDATA_MSB = 0xFA,
	BMP280_REGISTER_TEMPDATA_LSB = 0xFB,
	BMP280_REGISTER_TEMPDATA_XLSB = 0xFC
} tBmp280Register;

typedef enum bool {false=0, true} bool;
typedef unsigned char U8;
typedef short S16;
typedef unsigned short U16;
typedef int S32;
typedef unsigned int U32;
typedef long long S64;

static U8 BMP280_DEVICE_ADDRESS = 0x76;
static U8 BMP280_CHIPID = 0x58;
static U8 BMP280_RESET = 0xB6;

// Combine bits for config
U8 config(tStandby standby, tFilter filter)
{
	return (standby << 5) | (filter << 2);
}

// Combine bits for ctrl_meas
U8 ctrlMeas(tOsrsTemperature temperatureResolution, tOsrsPressure pressureResolution, tPowerMode powerMode)
{
	return (temperatureResolution << 5) | (pressureResolution << 2) | powerMode;
}

// Combine MSB LSB XLSB
S32 rawValFromMsbLsbXslb(U8 msb, U8 lsb, U8 xlsb)
{
	return (msb << 12) | (lsb << 4) | (xlsb >> 4);
}

U16 U16LE(U8 data[])
{
	return (U16) (((U16)data[0]) | (((U16)data [1]) << 8));
}

S16 S16LE(U8 data[])
{
	return (S16) (((S16)data[0]) | (((S16)data [1]) << 8));
}

// Formula for temperature from datasheet
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global variable
S32 t_fine;
S32 bmp280_compensate_Temperature(S32 adc_T, U16 dig_T1, S16 dig_T2, S16 dig_T3)
{
	S32 var1  = ((((adc_T >> 3) - ((S32)dig_T1 << 1))) * ((S32)dig_T2)) >> 11;
	S32 var2  = (((((adc_T>>4) - ((S32)dig_T1)) * ((adc_T>>4) - ((S32)dig_T1))) >> 12) * ((S32)dig_T3)) >> 14;
	t_fine = var1 + var2;
	S32 T  = (t_fine * 5 + 128) >> 8;
	return T;
}

//~ double bmp280_compensate_TemperatureDouble(S32 adc_T, U16 dig_T1, S16 dig_T2, S16 dig_T3)
//~ {
	//~ double var1 = (adc_T / 16384.0d - dig_T1 / 1024.0d) * dig_T2;
	//~ double var2 = (adc_T / 131072.0d - dig_T1 / 8192.0d) * (adc_T / 131072.0d - dig_T1 / 8192.0d) * dig_T3;
	//~ double temp = (var1 + var2) / 5120.0d;
	//~ t_fine = (var1 + var2);
	//~ return temp;
//~ }

// Pure 64-bit int compute
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867”represents 24674867/256 = 96386.2 Pa = 963.862 hPa
U32 bmp280_compensate_Pressure64(S32 adc_P, U16 dig_P1, S16 dig_P2, S16 dig_P3, S16 dig_P4, S16 dig_P5, S16 dig_P6, S16 dig_P7, S16 dig_P8, S16 dig_P9) {
	S64 p;
	S64 var1 = ((S64)t_fine) - 128000;
	S64 var2 = var1 * var1 * (S64)dig_P6;
	var2 = var2 + ((var1*(S64)dig_P5)<<17);
	var2 = var2 + (((S64)dig_P4)<<35);
	var1 = ((var1 * var1 * (S64)dig_P3)>>8) + ((var1 * (S64)dig_P2)<<12);
	var1 = (((((S64)1)<<47)+var1))*((S64)dig_P1)>>33;
	if(var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125)/var1;
	var1 = (((S64)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((S64)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((S64)dig_P7)<<4);
	return (U32) p;
}

// Pure 32-bit int compute
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
U32 bmp280_compensate_Pressure32(S32 adc_P, U16 dig_P1, S16 dig_P2, S16 dig_P3, S16 dig_P4, S16 dig_P5, S16 dig_P6, S16 dig_P7, S16 dig_P8, S16 dig_P9) {
	S32 var1, var2;
	U32 p;
	var1 = (((S32)t_fine)>>1) - (S32)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((S32)dig_P6);
	var2 = var2 + ((var1*((S32)dig_P5))<<1);
	var2 = (var2>>2)+(((S32)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((S32)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((S32)dig_P1))>>15);
	if(var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = (((U32)(((S32)1048576) - adc_P) - (var2>>12)))*3125;
	if(p < 0x80000000) {
		p = (p << 1) / ((U32)var1);
	} else {
		p = (p / (U32)var1) * 2;
	}
	var1 = (((S32)dig_P9) * ((S32)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((S32)(p>>2)) * ((S32)dig_P8))>>13;
	p = (U32)((S32)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}

// Double compute
double bmp280_compensate_PressureDouble(S32 adc_P, U16 dig_P1, S16 dig_P2, S16 dig_P3, S16 dig_P4, S16 dig_P5, S16 dig_P6, S16 dig_P7, S16 dig_P8, S16 dig_P9) {
	double press;
	double var1 = t_fine / 2.0d - 64000.0d;
	double var2 = var1 * var1 * dig_P6 / 32768.0d;
	var2 += var1 * dig_P5 * 2.0d;
	var2 = var2 / 4.0d + dig_P4 * 65536.0d; 
	var1 = (dig_P3 * var1 * var1/524288.0d + dig_P2 * var1) / 524288.0d; 
	var1 = (1.0d + var1 / 32768.0d ) * dig_P1;
	press = 1048576.0d - adc_P; 
	press = (press - var2 / 4096.0d ) * 6250.0d / var1; 
	var1 = dig_P9 * press * press / 2147483648.0d;
	var2 = adc_P * dig_P8 / 32768.0d; 
	press += (var1 + var2 + dig_P7) / 16.0d;
	return press;
}	


// Returns a file id for the port/bus
int i2c_Open(char *I2CBusName){
  int fd;
  //Open port for reading and writing
  if ((fd = open(I2CBusName, O_RDWR)) < 0){
    printf ("\n");
    printf ("%s : Failed to open the i2c bus, error : %d\n",__func__,errno);
    printf ("Check to see if you have a bus: %s\n",I2CBusName); 
    printf ("This is not a slave device problem, I can not find the bus/port with which to talk to the device\n"); 
    printf ("\n");
    // Only one of the following lines should be used
    // the second line allows you to retry another bus, PS disable all the printf 's 
    exit(1);      //Use this line if the function must terminate on failure
    //return fd;  //Use this line if it must return to the caller for processing
    }
    else{
     return fd;
    }
} 

int bmp280_ReadReg(int fd, uint8_t deviceAddress, uint8_t registerAddress, U8 outputDataRead[], int nbDataToRead){
	struct i2c_msg read_user_reg[2]={
		{deviceAddress,0,1,&registerAddress},
		{deviceAddress,I2C_M_RD,nbDataToRead,outputDataRead}
	};
	struct i2c_rdwr_ioctl_data messagebuffer = {
		.nmsgs = 2,                  //Two message/action
		.msgs = read_user_reg        //load the 'read_user_reg' message into the buffer
	};
	int rc = ioctl(fd, I2C_RDWR, &messagebuffer);  //Send the buffer to the bus and returns a send status
	if (rc < 0 ){
		printf("\n");
		printf("%s :htu21df User Reg Read command failed with error :%d\n",__func__,errno);
		printf("This means that device with address :0x%0x failed to receive this command\n", deviceAddress);
		printf("This command was preceded by a reset if that worked\n");
		printf("and this failed, then possible causes are Delay timing to short (overclock stuffing timing up)\n");
		printf("or bus unstable ,wire length,power supply unstable, terminating resistors.\n");
		printf("\n");
		return rc;
	}
	return 0;
}

int bmp280_WriteReg(int fd, uint8_t deviceAddress, U8 pairRegisterDataToWrite[], int nbDataToWrite){
	//Unlike the read commands the write requires the command and data to be sent in one message  
	//Build a user register write  command
	struct i2c_msg write_reg = {deviceAddress, 0, nbDataToWrite, pairRegisterDataToWrite};
	struct i2c_rdwr_ioctl_data messagebuffer = {
		.nmsgs = 1,                   //One message/action
		.msgs = &write_reg            //load the 'writ_reg' message into the buffer
	};
	int rc = ioctl(fd, I2C_RDWR, &messagebuffer);  //Send the buffer to the bus and returns a send status
	if (rc < 0 ) {
		return -1;
	}
	return 0;
}

int bmp280_init(int fd, bool softReset, tPowerMode powerMode, tStandby standby, tFilter filter, tOsrsTemperature temperatureResolution, tOsrsPressure pressureResolution){
	U8 data[4];
	// Check sensor id 0x58=BMP280_CHIPID
	if(bmp280_ReadReg(fd, BMP280_DEVICE_ADDRESS, BMP280_REGISTER_CHIPID, data, 1) == 0 && data[0] == BMP280_CHIPID) {
		if(softReset) {
			// Reset sensor
			data[0] = BMP280_REGISTER_SOFTRESET;
			data[1] = BMP280_RESET;
			if(bmp280_WriteReg(fd, BMP280_DEVICE_ADDRESS, data, 2) ==0) {
				//sleepms(200);
			}
		}
		data[0] = BMP280_REGISTER_CONTROL;
		data[1] = ctrlMeas(temperatureResolution, pressureResolution, powerMode);
		data[2] = BMP280_REGISTER_CONFIG;
		data[3] = config(standby, filter);
		if(bmp280_WriteReg(fd, BMP280_DEVICE_ADDRESS, data, 4)==0) {
			sleepms(100);
			return 0;
		}
	}
	return -1;
}

int main(int argc, char* argv[]){
	int rc;
	int fd = i2c_Open(I2CBus); //program will terminate within function if bus not present.
	bool softReset = true;

	for(int i = 0; i < argc; i++) {
		if(strcasecmp("--noReset", argv[i]) == 0) {
			fprintf(stderr, "\nOption : --noReset : No soft reset of the device\n");
			softReset = false;
		}
	}

	int bmp280rc = bmp280_init(fd, softReset, POWER_MODE_NORMAL, STANDBY_0500_0_MS, FILTER4, OSRS_TEMPERATURE_20BITS, OSRS_PRESSURE_20BITS);
	fprintf(stderr, "\nDevice accessed via File descriptor:%0i\t at Address:0x%0x\t has returned:%0i\n", fd, BMP280_DEVICE_ADDRESS, bmp280rc);

	if (bmp280rc == 0) {
		U8 data[24];
		if(bmp280_ReadReg(fd, BMP280_DEVICE_ADDRESS, BMP280_REGISTER_DIG_T1, data, 24) == 0) {
			U16 dig_T1 = U16LE(&data[BMP280_REGISTER_DIG_T1 - BMP280_REGISTER_DIG_T1]);
			S16 dig_T2 = S16LE(&data[BMP280_REGISTER_DIG_T2 - BMP280_REGISTER_DIG_T1]);
			S16 dig_T3 = S16LE(&data[BMP280_REGISTER_DIG_T3 - BMP280_REGISTER_DIG_T1]);
			U16 dig_P1 = U16LE(&data[BMP280_REGISTER_DIG_P1 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P2 = S16LE(&data[BMP280_REGISTER_DIG_P2 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P3 = S16LE(&data[BMP280_REGISTER_DIG_P3 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P4 = S16LE(&data[BMP280_REGISTER_DIG_P4 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P5 = S16LE(&data[BMP280_REGISTER_DIG_P5 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P6 = S16LE(&data[BMP280_REGISTER_DIG_P6 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P7 = S16LE(&data[BMP280_REGISTER_DIG_P7 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P8 = S16LE(&data[BMP280_REGISTER_DIG_P8 - BMP280_REGISTER_DIG_T1]);
			S16 dig_P9 = S16LE(&data[BMP280_REGISTER_DIG_P9 - BMP280_REGISTER_DIG_T1]);

			fprintf(stderr, "[0]=%hhX\t[1]=%hhX\t[2]=%hhX\t[3]=%hhX\t[4]=%hhX\t[5]=%hhX\t\n", data[0], data[1], data[2], data[3], data[4], data[5]);

			fprintf(stderr, "T1=%hu\tT2=%hd\tT3=%hd\n", dig_T1, dig_T2, dig_T3);
			fprintf(stderr, "P1=%hu\tP2=%hd\tP3=%hd\tP4=%hd\tP5=%hd\tP6=%hd\tP7=%hd\tP8=%hd\tP9=%hd\n",
			dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);

			for(int i=0;i<20;i++) {
				if(bmp280_ReadReg(fd, BMP280_DEVICE_ADDRESS, BMP280_REGISTER_PRESSDATA_MSB, data, 6) == 0) {
					S32 adc_Pressure = rawValFromMsbLsbXslb(data[0], data[1], data[2]);
					S32 adc_Temperature = rawValFromMsbLsbXslb(data[3], data[4], data[5]);
					fprintf(stderr, "adc_Temperature=%d\tadc_Pressure=%d\t\n", adc_Temperature, adc_Pressure);

					S32 temperature = bmp280_compensate_Temperature(adc_Temperature, dig_T1, dig_T2, dig_T3);
					U32 pressure64 = bmp280_compensate_Pressure64(adc_Pressure, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
					U32 pressure32 = bmp280_compensate_Pressure32(adc_Pressure, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
					double pressureDouble = bmp280_compensate_Pressure64(adc_Pressure, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
					printf("{\"temperature\":%2.2lf,\"Pressure64\":%4.1lf,\"Pressure32\":%4.1lf,\"PressureDouble\":%4.1lf}\n",
						temperature/100.0d, pressure64 / 25600.0d, pressure32 / 100.0d, pressureDouble/25600);
					sleepms(500);
				}
			}
		}    
		rc = 0;
	}
	else
	{
		rc = -1;
	}
	close (fd);   
	return rc;
}
