/*
	ADXL345 Library
	
	This libary contains functions to interact with the ADXL345 Triple Axis Digital Accelerometer from Analog Devices written for the ATmega168
	
	created 20 Aug 2009
	by Ryan Owens
	http://www.sparkfun.com
	
 
*/

#include "ADXL345.h"

I2C_HandleTypeDef*	adxlI2cHandle;

char status=0;

//Constructor defines the I2C address
ADXL345::ADXL345(I2C_HandleTypeDef* I2cHandle)
{
	adxlI2cHandle = I2cHandle;
	
}

//Initialize the I2C communication and put the accelerometer in Measure mode
char ADXL345::begin(void)
{
	//Put the accelerometer in MEASURE mode
	write(POWER_CTL, MEASURE);
	//Set the Range to +/- 4G
	return write(DATA_FORMAT, RANGE_0);
	
	//default ADXL345 rate is 100 Hz. Perfect!
}

void ADXL345::powerDown(void)
{

}

//Read a register value from the ADXL345
//pre: register_addr is the register address to read
//	   value is a pointer to an integer
//post: value contains the value of the register that was read
//returns: 1-Success
//		   TWSR-Failure (Check out twi.h for TWSR error codes)
//usage: status = accelerometer.read(DEVID, &value); //value is created as an 'int' in main.cpp
uint8_t ADXL345::read(uint8_t register_addr, uint8_t * value){

	uint8_t rxdata_buf;
	HAL_I2C_Mem_Read(adxlI2cHandle, ADXL_ADDR, register_addr, I2C_MEMADD_SIZE_8BIT, value, 1, 10);
	//value = &rxdata_buf;
	//return rxdata_buf;
}

//Write a value to a register
//pre: register_addre is the register to write to
//	   value is the value to place in the register
//returns: 1-Success
//		   TWSR- Failure
//usage status=accelerometer.write(register_addr, value);
uint8_t ADXL345::write(uint8_t register_addr, uint8_t value){

	HAL_I2C_Mem_Write(adxlI2cHandle, ADXL_ADDR, register_addr, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);

}

//Reads the x,y and z registers and stores the contents into x,y and z variables
//returns 1
//usage: accelerometer.update();
//Note: update must be called before using the getX, getY or getZ functions in order
//      to obtain the most recent values from the accelerometer
char ADXL345::update(void)
{
	uint8_t temp=0;
	read(DATAX0, &value);
	temp=value;
	read(DATAX1, &value);
	x = (value<<8)|temp;
	
	read(DATAY0, &value);
	temp=value;
	read(DATAY1, &value);
	y = (value<<8)|temp;

	read(DATAZ0, &value);
	temp=value;
	read(DATAZ1, &value);
	z = (value<<8)|temp;	

	return 1;
}

/*
get functions return the g value of the specified axis
The conversion is based on a +/-4G range.
If range is changed, make sure to update the scaling in the get functions
usage: printf("Xg = %1.3fg", (double)accelerometer.getX()
*/
float ADXL345::getX(void)
{
	xg=(float)x*0.0078;	
	return xg;
}

float ADXL345::getY(void)
{
	yg=(float)y*0.0078;
	return yg;
}

float ADXL345::getZ(void)
{
	zg=(float)z*0.0078;
	return zg;
}

bool ADXL345::isonline(){

	if(HAL_I2C_IsDeviceReady(adxlI2cHandle, ADXL_ADDR, 100, 100) != HAL_OK)return false;
	else return true;

}
