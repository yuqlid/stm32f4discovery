#ifndef ADXL345_h
#define ADXL345_h

#include "i2c.h"

class ADXL345
{
	public:
		char updated;
	
		ADXL345(I2C_HandleTypeDef* I2cHandle);
		char begin(void);
		bool isonline();
		uint8_t read(uint8_t register_addr, uint8_t * value);
		uint8_t write(uint8_t register_addr, uint8_t value);
		void powerDown(void);
		char update(void);
		
		float getX(void);
		float getY(void);
		float getZ(void);
		
//	private:
		char _i2c_address;
		int16_t x;
		int16_t y;
		int16_t z;
		float xg;
		float yg;
		float zg;
		uint8_t value;
};

//**********************************************************
//
//                  	Macros
//
//**********************************************************


//**********************************************************
//
//                  Pin Definitions
//
//**********************************************************
#define ADXL_ADDR	0xA6

/**
 * ADXL Register Map
 */
enum{
    DEVID          = 0x00,	//Device ID Register
    THRESH_TAP     = 0x1D,	//Tap Threshold
    OFSX           = 0x1E,	//X-axis offset
    OFSY           = 0x1F,	//Y-axis offset
    OFSZ           = 0x20,	//Z-axis offset
    DUR            = 0x21,	//Tap Duration
    Latent         = 0x22,	//Tap latency
    Window         = 0x23,	//Tap window
    THRESH_ACT     = 0x24,	//Activity Threshold
    THRESH_INACT   = 0x25,	//Inactivity Threshold
    TIME_INACT     = 0x26,	//Inactivity Time
    ACT_INACT_CTL  = 0x27,	//Axis enable control for activity and inactivity detection
    THRESH_FF      = 0x28,	//free-fall threshold
    TIME_FF        = 0x29,	//Free-Fall Time
    TAP_AXES       = 0x2A,	//Axis control for tap/double tap
    ACT_TAP_STATUS = 0x2B,	//Source of tap/double tap
    BW_RATE        = 0x2C,	//Data rate and power mode control
    POWER_CTL      = 0x2D,	//Power Control Register
    INT_ENABLE     = 0x2E,	//Interrupt Enable Control
    INT_MAP        = 0x2F,	//Interrupt Mapping Control
    INT_SOURCE     = 0x30,	//Source of interrupts
    DATA_FORMAT    = 0x31,	//Data format control
    DATAX0         = 0x32,	//X-Axis Data 0
    DATAX1         = 0x33,	//X-Axis Data 1
    DATAY0         = 0x34,	//Y-Axis Data 0
    DATAY1         = 0x35,	//Y-Axis Data 1
    DATAZ0         = 0x36,	//Z-Axis Data 0
    DATAZ1         = 0x37,	//Z-Axis Data 1
    FIFO_CTL       = 0x38,	//FIFO control
    FIFO_STATUS    = 0x39	//FIFO status
};

//Power Control Register Bits
#define WU_0		(1<<0)	//Wake Up Mode - Bit 0
#define	WU_1		(1<<1)	//Wake Up mode - Bit 1
#define SLEEP		(1<<2)	//Sleep Mode
#define	MEASURE		(1<<3)	//Measurement Mode
#define AUTO_SLP	(1<<4)	//Auto Sleep Mode bit
#define LINK		(1<<5)	//Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define	OVERRUN		(1<<0)
#define	WATERMARK	(1<<1)
#define FREE_FALL	(1<<2)
#define	INACTIVITY	(1<<3)
#define	ACTIVITY	(1<<4)
#define DOUBLE_TAP	(1<<5)
#define	SINGLE_TAP	(1<<6)
#define	DATA_READY	(1<<7)

//Data Format Bits
#define RANGE_0		(1<<0)
#define	RANGE_1		(1<<1)
#define JUSTIFY		(1<<2)
#define	FULL_RES	(1<<3)

#define	INT_INVERT	(1<<5)
#define	SPI			(1<<6)
#define	SELF_TEST	(1<<7)

#endif
