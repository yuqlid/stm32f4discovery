/*
 * AQM0802A.h
 *
 *  Created on: 2016/10/12
 *      Author: Yuki
 */

#ifndef AQM0802A_H_
#define AQM0802A_H_

#include "gpio.h"

#pragma once

class AQM0802A{

	public:

		AQM0802A(I2C_HandleTypeDef* I2cHandle);
		AQM0802A(I2C_HandleTypeDef* I2cHandle, GPIO_TypeDef* NCS_GPIO, uint16_t NCS_PIN);
		void cmd(uint8_t x);
		void contdata(uint8_t x);
		void lastdata(uint8_t x);
		void printStr(const char *s);
		void printHex(uint8_t num);
		void init();
		void setCursor(uint8_t x, uint8_t y);
		void setCG(int src,int dst,int len);
		void setContrast(uint8_t c);
		bool isonline();
		void active();
		void inactive();

		uint8_t mode = 0;
		uint8_t contrast = 35; // 0-63
		uint8_t contrastFlag = false;
		int CGcounter;
		int FADEcounter;
};

const uint8_t	AQCM0802_ADD = 0x7C;

#endif /* AQM0802A_H_ */
