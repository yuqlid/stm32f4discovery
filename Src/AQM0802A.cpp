/*
 * AQM0802A.cpp
 *
 *  Created on: 2016/10/12
 *      Author: Yuki
 */

#include "AQM0802A.h"

I2C_HandleTypeDef*	lcdI2cHandle;
GPIO_TypeDef* 		AQM0802A_RESET_GPIO;
uint16_t 			AQM0802A_RESET_PIN;

AQM0802A::AQM0802A(I2C_HandleTypeDef* I2cHandle){

	lcdI2cHandle = I2cHandle;
}

AQM0802A::AQM0802A(I2C_HandleTypeDef* I2cHandle, GPIO_TypeDef* RESET_GPIO, uint16_t RESET_PIN){

	lcdI2cHandle = I2cHandle;
	AQM0802A_RESET_GPIO = RESET_GPIO;
	AQM0802A_RESET_PIN = RESET_PIN;

}

void AQM0802A::cmd(uint8_t data){

	uint8_t txdata_buf[2] = {0x00, data};	// CO = 0,RS = 0
	HAL_I2C_Master_Transmit(lcdI2cHandle, AQCM0802_ADD, txdata_buf, 2, 10);
}

void AQM0802A::contdata(uint8_t data){

	uint8_t txdata_buf[2] = {0xC0, data};	//0b11000000 CO = 1, RS = 1
	HAL_I2C_Master_Transmit(lcdI2cHandle, AQCM0802_ADD, txdata_buf, 2, 10);
}

void AQM0802A::lastdata(uint8_t data){

	uint8_t txdata_buf[2] = {0x40, data};	//0b11000000 CO = 0, RS = 1
	HAL_I2C_Master_Transmit(lcdI2cHandle, AQCM0802_ADD, txdata_buf, 2, 10);
}

void AQM0802A::printStr(const char *s){

	while(*s) {
		if(*(s + 1)) {
			contdata(*s);
			} else {
			lastdata(*s);
		}
		s++;
	}
}

void AQM0802A::printHex(uint8_t num){

	contdata(num);
}

void AQM0802A::init(){

	HAL_Delay(40);
	// LCD initialize
	cmd(0x38); // function set
	cmd(0x39); // function set
	cmd(0x04); // EntryModeSet
	cmd(0x14); // interval osc
	cmd(0x70 | (contrast & 0xF)); // contrast Low
	cmd(0x5C | ((contrast >> 4) & 0x3)); // contast High/icon/power
	cmd(0x6C); // follower control
	HAL_Delay(200);
	cmd(0x38); // function set
	cmd(0x0C); // Display On
	cmd(0x01); // Clear Display
	HAL_Delay(200); // need additional wait to Clear Display
}

void AQM0802A::setCursor(uint8_t x, uint8_t y){

	cmd(0x80 | (y * 0x40 + x));
}

uint8_t cg[13 * 8] = {
/*
  0b00001111,0b00010000,0b00010000,0b00001110,0b00000001,0b00000001,0b00011110,0b00000000,
  0b00010001,0b00010001,0b00010001,0b00010101,0b00010101,0b00010101,0b00001010,0b00000000,
  0b00001110,0b00000100,0b00000100,0b00000100,0b00000100,0b00000100,0b00001110,0b00000000,
  0b00011111,0b00000100,0b00000100,0b00000100,0b00000100,0b00000100,0b00000100,0b00000000,
  0b00001110,0b00010001,0b00010000,0b00010000,0b00010000,0b00010001,0b00001110,0b00000000,
  0b00010001,0b00010001,0b00010001,0b00011111,0b00010001,0b00010001,0b00010001,0b00000000,
*/
  0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E,0x00, // S
  0x11,0x11,0x11,0x15,0x15,0x15,0x0A,0x00, // W
  0x0E,0x04,0x04,0x04,0x04,0x04,0x0E,0x00, // I
  0x1F,0x04,0x04,0x04,0x04,0x04,0x04,0x00, // T
  0x0E,0x11,0x10,0x10,0x10,0x11,0x0E,0x00, // C
  0x11,0x11,0x11,0x1F,0x11,0x11,0x11,0x00, // H
/*
  0b00001111,0b00010000,0b00010000,0b00001110,0b00000001,0b00000001,0b00011110,0b00000000,
  0b00001110,0b00010001,0b00010000,0b00010000,0b00010000,0b00010001,0b00001110,0b00000000,
  0b00001110,0b00000100,0b00000100,0b00000100,0b00000100,0b00000100,0b00001110,0b00000000,
  0b00011111,0b00010000,0b00010000,0b00011110,0b00010000,0b00010000,0b00011111,0b00000000,
  0b00010001,0b00010001,0b00011001,0b00010101,0b00010011,0b00010001,0b00010001,0b00000000,
  0b00001110,0b00010001,0b00010000,0b00010000,0b00010000,0b00010001,0b00001110,0b00000000,
  0b00011111,0b00010000,0b00010000,0b00011110,0b00010000,0b00010000,0b00011111,0b00000000,
*/
  0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E,0x00, // S
  0x0E,0x11,0x10,0x10,0x10,0x11,0x0E,0x00, // C
  0x0E,0x04,0x04,0x04,0x04,0x04,0x0E,0x00, // I
  0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F,0x00, // E
  0x11,0x11,0x19,0x15,0x13,0x11,0x11,0x00, // N
  0x0E,0x11,0x10,0x10,0x10,0x11,0x0E,0x00, // C
  0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F,0x00  // E
};

void AQM0802A::setCG(int src,int dst,int len){

	cmd(0x38);
	cmd(0x40 + dst);
	if (src >= 0) {
		for (int i = 0;i < len;i++) printHex(cg[src + i]);
	}else{
		for (int i = 0;i < len;i++) printHex(0);
	}
}

void AQM0802A::setContrast(uint8_t c) {
	cmd(0x39);
	cmd(0x70 | (c & 0x0f)); // contrast Low
	cmd(0x5C | ((c >> 4) & 0x03)); // contast High/icon/power
	cmd(0x38);
}

bool AQM0802A::isonline(){

		if(HAL_I2C_IsDeviceReady(lcdI2cHandle, AQCM0802_ADD, 100, 100) != HAL_OK)return false;
		else return true;
}

void AQM0802A::inactive() {

	HAL_GPIO_WritePin(AQM0802A_RESET_GPIO, AQM0802A_RESET_PIN, GPIO_PIN_RESET);
}

void AQM0802A::active() {

	HAL_GPIO_WritePin(AQM0802A_RESET_GPIO, AQM0802A_RESET_PIN, GPIO_PIN_RESET);
}
