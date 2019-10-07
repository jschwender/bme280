/*
  Raspberry pi programm to get data from Bosch Sensor BME 280
  Temperature, Pressure, Humidity and calculated pressure at NN when 
  measurement heigth is given.
  2019-10-04 J. Schwender
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "BME280.h"
//#define DEBUG
char  *DeviceFile = "/dev/i2c-1"; // I2C device file 1
char      I2CADDR = 0x76;         // default device i2c address, can be changed by command line
char     data[34] = {0};          // 33 raw calibration byte register 0x88…0xE7
char    config[2] = {0};          // config register values
char device_id[2] = {0};
int            h0 = 0;            // default altitude, override by command line
int         units = 0,endless=0,header;  // flag to display units with the values, to loop endless
int devicehandle;                 // devicehandle
enum tformat { none, seconds, iso };
enum tformat format=iso;
enum sep { comma, semi, space };
enum sep csep=space;
time_t timestamp;                 // system time variable
struct tm *local;                 // convert timestamp to time structure
          double temperature, pressure, pressure_nn, humidity;           // final temperature
  unsigned short dig_T1;          // datatypes according to data sheet!
    signed short dig_T2, dig_T3;
   unsigned char dig_H1, dig_H3;
    signed short dig_H2, dig_H4, dig_H5;
     signed char dig_H6;
  unsigned short dig_P1;
    signed short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
            long adc_p,adc_t,adc_h;

void my_help(void) {
    printf("command line options:\n  -H   print data header\n  -h <nnn>  (optional) height in m above NN, defaults to 0 m\n");
    printf("  -d <device>   (optional) i2c device defaults to /dev/i2c-1\n  -2    (optional) use alternative i2c-address 0x77 instead of default 0x76\n");
    printf("  -f [seconds|iso|none]   timestamps in either seconds, ISO 8660 format (default) or none\n");
    printf("  -t [space|comma|semi]   output values with given separator (default is space)\n");
    printf("  -u   output unit after each value (default is no unit)\n");
    printf("  -c   loop endless (default one time only)\n");
}

void my_debug(void) {
    printf("     altitude: %i m above NN (default value)\n",h0);
    printf("     altitude: %i m above NN\n",h0);
    printf("   time format %i\n", format);
    printf("     separator %i\n", csep);
    printf("     use units %i\n",units);
    printf("    continuous %i\n",endless);
    printf("   i2c device: %s\n", DeviceFile);
    printf("address in use 0x%x\n",I2CADDR);
}

void print_header(void) {
    switch (format) {
      case seconds:   printf("time"); break;
      case iso:       printf("time"); break;
      case none: break;
    }
    if ( format == none ) {
    switch (csep) {        // print different column separators between values, depending on given command line option
      case comma:  printf("temperature,pressure,humidity,pressureNN"); break;
      case semi:   printf("temperature;pressure;humidity;pressureNN"); break;
      case space:  printf("temperature pressure humidity pressureNN"); break;
      }
    } else {
    switch (csep) {        // print different column separators between values, depending on given command line option
      case comma:  printf(",temperature,pressure,humidity,pressureNN"); break;
      case semi:   printf(";temperature;pressure;humidity;pressureNN"); break;
      case space:  printf(" temperature pressure humidity pressureNN"); break;
      }
    }
    printf("\n");
}

void print_sensor_data(void)  {    //-------- output with units and comma separator  ---------
    switch (format) {
      case seconds:   printf("%li", timestamp);break;
      case iso:       printf("%04d-%02d-%02dT%02d:%02d:%02d", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);break;
      case none:      break;
      }
    if ( format == none ) {
        if ( units == 1 ) {
	        switch (csep) {
		case comma: printf("%0.2lf °C,%0.2lf hPa,%0.2lf %%,%0.2lf hPa(NN)\n", temperature, pressure, humidity, pressure_nn); break;
		case semi:  printf("%0.2lf °C;%0.2lf hPa;%0.2lf %%;%0.2lf hPa(NN)\n", temperature, pressure, humidity, pressure_nn); break;
		case space: printf("%0.2lf °C %0.2lf hPa %0.2lf %% %0.2lf hPa(NN)\n", temperature, pressure, humidity, pressure_nn); break;
		}
	} else {
	    switch (csep) {
		case comma: printf("%0.2lf,%0.2lf,%0.2lf,%0.2lf\n", temperature, pressure, humidity, pressure_nn); break;
		case semi:  printf("%0.2lf;%0.2lf;%0.2lf;%0.2lf\n", temperature, pressure, humidity, pressure_nn); break;
		case space: printf("%0.2lf %0.2lf %0.2lf %0.2lf\n", temperature, pressure, humidity, pressure_nn); break;
		}
        }
    } else {
        if ( units == 1 ) {
	        switch (csep) {
		case comma: printf(",%0.2lf °C,%0.2lf hPa,%0.2lf %%,%0.2lf hPa(NN)\n", temperature, pressure, humidity, pressure_nn); break;
		case semi:  printf(";%0.2lf °C;%0.2lf hPa;%0.2lf %%;%0.2lf hPa(NN)\n", temperature, pressure, humidity, pressure_nn); break;
		case space: printf(" %0.2lf °C %0.2lf hPa %0.2lf %% %0.2lf hPa(NN)\n", temperature, pressure, humidity, pressure_nn); break;
		}
	} else {
	    switch (csep) {
		case comma: printf(",%0.2lf,%0.2lf,%0.2lf,%0.2lf\n", temperature, pressure, humidity, pressure_nn); break;
		case semi:  printf(";%0.2lf;%0.2lf;%0.2lf;%0.2lf\n", temperature, pressure, humidity, pressure_nn); break;
		case space: printf(" %0.2lf %0.2lf %0.2lf %0.2lf\n", temperature, pressure, humidity, pressure_nn); break;
		}
        }
    }
}

void conf_config(void) {              // config register 0xF5 values
    config[0] = 0xF5;                 // select config register(0xF5)
    config[1] = ts_125m + filter_off; // stand_by time is only relevant in standard mode, not in forced mode.
    write(devicehandle, config, 2);
}

void measurement(void) {              // both control registers 0xF2 & 0xF4
    config[0] = 0xF2;                 // Select humid control measurement register(0xF2)
    config[1] = hum_oversampling_16;
    write(devicehandle, config, 2);   // changes here are only effective after a write operation to 0xF4 register!!
    config[0] = 0xF4;                 // Select control measurement register(0xF4)
    config[1] = mode_forced + pressure_oversampling_16 + temp_oversampling_16;
    write(devicehandle, config, 2);   // this triggers the start of measurement
}

double compensate_pressure(double raw_pressure,double temperature) {     // pressure offset calculations
    double press1, press2, press3, result, p1quadrat;
    press1 = temperature*2560 - 64000.0;  // das hier sollte das gleiche sein
    p1quadrat = press1*press1;
    press2 = p1quadrat*((double)dig_P6)/32768.0;
    press2 = press2 + press1*((double)dig_P5)*2.0;
    press2 = (press2/4.0) + (((double)dig_P4)*65536.0);
    press1 = (((double)dig_P3)*p1quadrat/524288.0 + ((double)dig_P2)*press1)/524288.0;
    press1 = (1.0 + press1/32768.0)*((double)dig_P1);
    press3 = 1048576.0 - (double)raw_pressure;
    if (press1 != 0.0)                           // avoid error: division by 0
	{
	press3 = (press3 - press2/4096.0)*6250.0/press1;
	press1 = ((double)dig_P9)*press3*press3/2147483648.0;
	press2 = press3 * ((double)dig_P8)/32768.0;
	result = (press3 + (press1 + press2 + ((double)dig_P7))/16.0)/100;
	}
    else
	{ result = 0.0; }
    return result;
}

double compensate_humidity(long raw_humidity,double temperature) {     // compensate humidity
    double var1, var2, var3, var4, var5, var6, result;

    var1 = temperature - 76800.0;
    var2 = (((double)dig_H4) * 64.0 + (((double)dig_H5) / 16384.0) * var1);
    var3 = (double)raw_humidity - var2;
    var4 = ((double)dig_H2) / 65536.0;
    var5 = (1.0 + (((double)dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)dig_H6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    result = var6 * (1.0 - ((double)dig_H1) * var6 / 524288.0);
#ifdef DEBUG
    printf("no-limit H %f %%    ",result);
#endif
    if (result > 100.0)    {   result = 100.0;  }
    else if (result < 0.0) {   result = 0.0;    }
#ifdef DEBUG
    printf("mit limit: H %f %%\n",result);
#endif
    return result;
}

double compensate_temperature(long raw_temp) {  // temperature offset calculations
    double temp1, temp2, temp3;
    temp1 = (((double)raw_temp)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
    temp3 = ((double)raw_temp)/131072.0 - ((double)dig_T1)/8192.0;
    temp2 = temp3 * temp3 * ((double)dig_T3);
    return (temp1 + temp2)/5120.0;
}

//########################################################################
int main(int argc, char* argv[])
{
	int i;                          // loop counter
	char reg[1] = {0};              // reg[], for I2C I/O

  for(i=1; i < argc; ++i)
  {
     if( !strcmp( argv[i], "--help" ) ) { my_help(); exit(0); }
     if( !strcmp( argv[i], "-help" ) )  { my_help(); exit(0); }
     if( !strcmp( argv[i], "-h" ) ) {
            if ( (i+1) >= argc ) { my_help(); exit(1); }  // avoid overflow if incomplete option provided
	    h0 = atoi(argv[i+1]);
     }
     if( !strcmp( argv[i], "-d" ) ) {
            if ( (i+1) >= argc ) { my_help(); exit(1); }
	    DeviceFile = argv[i+1];   // alternative device file
     }
     if( !strcmp( argv[i], "-2" ) ) {
            I2CADDR = 0x77;   // alternative address
     }
     if( !strcmp( argv[i], "-f" ) ) {
            if ( (i+1) >= argc ) { my_help(); exit(1);}
	    if ( !strcmp(argv[i+1],"none") )    { format = none;    }
	    if ( !strcmp(argv[i+1],"iso") )     { format = iso;     }
	    if ( !strcmp(argv[i+1],"seconds") ) { format = seconds; }
     }
     if( !strcmp( argv[i], "-t" ) ) {
            if ( (i+1) >= argc ) { my_help(); exit(1);}
	    if ( !strcmp(argv[i+1],"comma") )   { csep = comma; }
	    if ( !strcmp(argv[i+1],"semi") )    { csep = semi;  }
	    if ( !strcmp(argv[i+1],"space") )   { csep = space; }
	}
     if( !strcmp( argv[i], "-u" ) ) { units = 1;   } // display units
     if( !strcmp( argv[i], "-c" ) ) { endless = 1; } // don't exit the measurement & output loop
     if( !strcmp( argv[i], "-H" ) ) { header = 1;  } // must be last, depends on other options
  }

  if((devicehandle = open(DeviceFile, O_RDWR)) < 0) {
	printf("Failed to open the i2c device file. \n");
	my_help();
	exit(1);
  }

  ioctl(devicehandle, I2C_SLAVE, I2CADDR);   // set the I2C device to slave mode, we are master here

  reg[0] = 0xD0;                 // id register should always contain chip ID
  write(devicehandle, reg, 1);
  if(read(devicehandle, device_id, 1) != 1) {
	printf("Failed to find a BMP 280 device on register %x\n",reg[0]);
	exit(1);
  }
#ifdef DEBUG
  printf("device ID on register 0xD0: %x", device_id[0]);
#endif
  // read 33 bytes of calibration data from address(0x88)
  reg[0] = 0x88;
  write(devicehandle, reg, 1);
  if(read(devicehandle, data, 33) != 33) {
	printf("Unable to read data from i2c device file\n");
	exit(1);
  }
#ifdef DEBUG
  my_debug();
#endif
// temp coefficents
  dig_T1 = (data[1] << 8) + data[0];        // unsigned short
  dig_T2 = (data[3] << 8) + data[2];        // signed short
  dig_T3 = (data[5] << 8) + data[4];        // signed short
#ifdef DEBUG
  printf("T raw coeff %i %i    %i %i    %i %i\n", data[0], data[1], data[2], data[3], data[4], data[5]);
  printf("T coeff     %i  %i  %i\n", T[0], T[1], T[2]);
  printf("T new coeff %i  %i  %i\n", dig_T1, dig_T2, dig_T3);
#endif

// pressure coefficents
  dig_P1 =  data[6] + (data[7]  << 8);      // unsigned short
  dig_P2 =  data[8] + (data[9]  << 8);      // singned short
  dig_P3 = data[10] + (data[11] << 8);
  dig_P4 = data[12] + (data[13] << 8);
  dig_P5 = data[14] + (data[15] << 8);
  dig_P6 = data[16] + (data[17] << 8);
  dig_P7 = data[18] + (data[19] << 8);
  dig_P8 = data[20] + (data[21] << 8);
  dig_P9 = data[22] + (data[23] << 8);
#ifdef DEBUG
  printf("P raw coeff %3i %3i    %3i %3i   %3i %3i   %3i %3i   %3i %3i   %3i %3i ...\n", data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17]);
  printf("P coeff     %i  %i  %i  %i  %i  %i  %i  %i  %i\n", P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8]);
  printf("P alt coeff %i  %i  %i  %i  %i  %i  %i  %i  %i\n", dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9 );
#endif

// Humidity coeffs
  dig_H1 = data[25];                           // unsigned char (reg 0xA1)
  dig_H2 = data[26] + (data[27] << 8);         // signed short
  dig_H3 = data[28];                           // unsigned char
  dig_H4 = (data[29] << 4) + (data[30] & 0xf); // signed 12-bit
  dig_H5 = (data[31] << 4) + (data[32] >> 4);  // signed short
  dig_H6 = data[33];                           // signed char 
#ifdef DEBUG
  printf("H raw coeff %i  %i  %i  %i  %i  %i  %i  %i  %i\n", data[25], data[26], data[27], data[28],data[29],data[30],data[31],data[32],data[33]);
  printf("H coeff  %i  %i  %i  %i  %i  %i\n", dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);
#endif
  if ( header == 1 ) { print_header(); }
  while (1) {
	conf_config();       // configure standby, filter and SPI
	measurement();       // configure oversampling and mode, trigger measurement
	time(&timestamp);    // get the timestamp
	local = localtime(&timestamp);  // convert timestamp to time structure
	usleep(121000);      // at 16× oversampling for t,p,h 121 ms are max required for conversion
  
	reg[0] = 0xF7;
	write(devicehandle, reg, 1);
	if(read(devicehandle, data, 8) != 8)  // 8 bytes = pressure + temp + hum raw data
		{
		printf("Unable to read data from i2c device file\n");
		exit(1);
		}

	// Convert pressure and temperature data to 19-bits
	adc_p = (((long)data[0] << 12) + ((long)data[1] << 4) + (long)(data[2] >> 4));  // long variable
	adc_t = (((long)data[3] << 12) + ((long)data[4] << 4) + (long)(data[5] >> 4));
	adc_h = (((long)data[6] << 8)  + (long)data[7]); //++
#ifdef DEBUG
	printf("%li   %li   %li  p t h raw data\n", adc_p, adc_t, adc_h);
#endif
	//    temperature = compensate_temperature_alt(adc_t);
	temperature = compensate_temperature(adc_t);
	//    pressure = compensate_pressure_alt(adc_p,temperature);
	pressure = compensate_pressure(adc_p,temperature);
	pressure_nn = pressure/pow(1.0 - h0/44330.0, 5.255); // this gives the pressure at NN, calculated by given measurement height
	humidity = compensate_humidity(adc_h,temperature);
	print_sensor_data();
	if ( endless == 0 ) { return 0; }     // loop only if endless is not set to 0
  }
  return 0;
}



