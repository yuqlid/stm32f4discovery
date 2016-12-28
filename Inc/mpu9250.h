/*
 * mpu9250.h
 *
 *  Created on: 2016/04/11
 *      Author: Yuki
 */


#ifndef MPU9250_H_
#define MPU9250_H_

#include "gpio.h"

#pragma once

class mpu9250{

  public:

#ifdef HAL_I2C_MODULE_ENABLED
	mpu9250(I2C_HandleTypeDef* I2cHandle);
    mpu9250(I2C_HandleTypeDef* I2cHandle, uint8_t slaveaddress);
#endif /* HAL_I2C_MODULE_ENABLED */
#ifdef HAL_SPI_MODULE_ENABLED
    mpu9250(SPI_HandleTypeDef* SpiHandle, GPIO_TypeDef* NCS_GPIO, uint16_t NCS_PIN);
#endif /* HAL_SPI_MODULE_ENABLED */
#ifdef HAL_I2C_MODULE_ENABLED
    bool isonline();//! Effective only I2C
#endif /* HAL_I2C_MODULE_ENABLED */
    bool init(int sample_rate_div,int low_pass_filter);
    void read_temp();
    void read_acc();
    void read_rot();
    unsigned int set_gyro_scale(int scale);
    unsigned int set_acc_scale(int scale);
    void calib_acc();
    void AK8963_calib_Magnetometer();

    unsigned int whoami();
    uint8_t  AK8963_whoami();
    void AK8963_read_Magnetometer();
    void read_all();

    float acc_divider;
    float gyro_divider;

    int calib_data[3];
    float Magnetometer_ASA[3];

    float accelerometer_data[3];
    float Temperature;
    float gyroscope_data[3];
    float Magnetometer[3];

    int16_t accelerometer_data_raw[3];
    int16_t gyroscope_data_raw[3];
    int16_t Magnetometer_raw[3];
    uint16_t Temperature_raw;
  private:
    unsigned int WriteReg( uint8_t WriteAddr, uint8_t WriteData );
    unsigned int ReadReg( uint8_t WriteAddr, uint8_t WriteData );
    void ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes );
#ifdef HAL_SPI_MODULE_ENABLED
    void select();  //!Effective only SPI
    void deselect();//!Effective only SPI
#endif /* HAL_SPI_MODULE_ENABLED */

};

// mpu9250 registers

#define MPUREG_XG_OFFS_TC          0x00
#define MPUREG_YG_OFFS_TC          0x01
#define MPUREG_ZG_OFFS_TC          0x02
#define MPUREG_X_FINE_GAIN         0x03
#define MPUREG_Y_FINE_GAIN         0x04
#define MPUREG_Z_FINE_GAIN         0x05
#define MPUREG_XA_OFFS_H           0x06
#define MPUREG_XA_OFFS_L           0x07
#define MPUREG_YA_OFFS_H           0x08
#define MPUREG_YA_OFFS_L           0x09
#define MPUREG_ZA_OFFS_H           0x0A
#define MPUREG_ZA_OFFS_L           0x0B
#define MPUREG_PRODUCT_ID          0x0C
#define MPUREG_SELF_TEST_X         0x0D
#define MPUREG_SELF_TEST_Y         0x0E
#define MPUREG_SELF_TEST_Z         0x0F
#define MPUREG_SELF_TEST_A         0x10
#define MPUREG_XG_OFFS_USRH        0x13
#define MPUREG_XG_OFFS_USRL        0x14
#define MPUREG_YG_OFFS_USRH        0x15
#define MPUREG_YG_OFFS_USRL        0x16
#define MPUREG_ZG_OFFS_USRH        0x17
#define MPUREG_ZG_OFFS_USRL        0x18
#define MPUREG_SMPLRT_DIV          0x19
#define MPUREG_CONFIG              0x1A
#define MPUREG_GYRO_CONFIG         0x1B
#define MPUREG_ACCEL_CONFIG        0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG         0x37
#define MPUREG_INT_ENABLE          0x38
#define DMP_INT_STATUS             0x39  // Check DMP interrupt
#define INT_STATUS                 0x3A
#define MPUREG_ACCEL_XOUT_H        0x3B
#define MPUREG_ACCEL_XOUT_L        0x3C
#define MPUREG_ACCEL_YOUT_H        0x3D
#define MPUREG_ACCEL_YOUT_L        0x3E
#define MPUREG_ACCEL_ZOUT_H        0x3F
#define MPUREG_ACCEL_ZOUT_L        0x40
#define MPUREG_TEMP_OUT_H          0x41
#define MPUREG_TEMP_OUT_L          0x42
#define MPUREG_GYRO_XOUT_H         0x43
#define MPUREG_GYRO_XOUT_L         0x44
#define MPUREG_GYRO_YOUT_H         0x45
#define MPUREG_GYRO_YOUT_L         0x46
#define MPUREG_GYRO_ZOUT_H         0x47
#define MPUREG_GYRO_ZOUT_L         0x48
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_EXT_SENS_DATA_01    0x4A
#define MPUREG_EXT_SENS_DATA_02    0x4B
#define MPUREG_EXT_SENS_DATA_03    0x4C
#define MPUREG_EXT_SENS_DATA_04    0x4D
#define MPUREG_EXT_SENS_DATA_05    0x4E
#define MPUREG_EXT_SENS_DATA_06    0x4F
#define MPUREG_EXT_SENS_DATA_07    0x50
#define MPUREG_EXT_SENS_DATA_08    0x51
#define MPUREG_EXT_SENS_DATA_09    0x52
#define MPUREG_EXT_SENS_DATA_10    0x53
#define MPUREG_EXT_SENS_DATA_11    0x54
#define MPUREG_EXT_SENS_DATA_12    0x55
#define MPUREG_EXT_SENS_DATA_13    0x56
#define MPUREG_EXT_SENS_DATA_14    0x57
#define MPUREG_EXT_SENS_DATA_15    0x58
#define MPUREG_EXT_SENS_DATA_16    0x59
#define MPUREG_EXT_SENS_DATA_17    0x5A
#define MPUREG_EXT_SENS_DATA_18    0x5B
#define MPUREG_EXT_SENS_DATA_19    0x5C
#define MPUREG_EXT_SENS_DATA_20    0x5D
#define MPUREG_EXT_SENS_DATA_21    0x5E
#define MPUREG_EXT_SENS_DATA_22    0x5F
#define MPUREG_EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS          0x61
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL           0x6A
#define MPUREG_PWR_MGMT_1          0x6B
#define MPUREG_PWR_MGMT_2          0x6C
#define MPUREG_BANK_SEL            0x6D
#define MPUREG_MEM_START_ADDR      0x6E
#define MPUREG_MEM_R_W             0x6F
#define MPUREG_DMP_CFG_1           0x70
#define MPUREG_DMP_CFG_2           0x71
#define MPUREG_FIFO_COUNTH         0x72
#define MPUREG_FIFO_COUNTL         0x73
#define MPUREG_FIFO_R_W            0x74
#define MPUREG_WHOAMI              0x75 // Should return 0x71
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E

/**
 * AK8963 Reg In MPU9250
 */
enum{
	AK8963_I2C_ADDR  = 0x0C,                                       //!< AK8963_I2C_ADDR
	AK8963_Device_ID = 0x48,                                       //!< AK8963_Device_ID
// Read-only Reg
	AK8963_WIA       = 0x00,/*Device ID*/                          //!< AK8963_WIA
	AK8963_INFO      = 0x01,/*Information*/                        //!< AK8963_INFO
	AK8963_ST1       = 0x02,/*Status 1*/                           //!< AK8963_ST1
	AK8963_HXL       = 0x03,/*Measurement data*/                   //!< AK8963_HXL
	AK8963_HXH       = 0x04,                                       //!< AK8963_HXH
	AK8963_HYL       = 0x05,                                       //!< AK8963_HYL
	AK8963_HYH       = 0x06,                                       //!< AK8963_HYH
	AK8963_HZL       = 0x07,                                       //!< AK8963_HZL
	AK8963_HZH       = 0x08,                                       //!< AK8963_HZH
	AK8963_ST2       = 0x09,/*Status 2*/                           //!< AK8963_ST2
// Write/Read Reg
	AK8963_CNTL1     = 0x0A,/*Control 1*/                          //!< AK8963_CNTL1
	AK8963_CNTL2     = 0x0B,/*Control 2*/                          //!< AK8963_CNTL2
	AK8963_ASTC      = 0x0C,/*Reserved*/                           //!< AK8963_ASTC
	AK8963_TS1       = 0x0D,/*Test 1*/                             //!< AK8963_TS1
	AK8963_TS2       = 0x0E,/*Test 2*/                             //!< AK8963_TS2
	AK8963_I2CDIS    = 0x0F,/*I2C disable*/                        //!< AK8963_I2CDIS
// Read-only Reg ( ROM )
	AK8963_ASAX      = 0x10,/*X-axis sensitivity adjustment value*///!< AK8963_ASAX
	AK8963_ASAY      = 0x11,/*Y-axis sensitivity adjustment value*///!< AK8963_ASAY
	AK8963_ASAZ      = 0x12 /*Z-axis sensitivity adjustment value*///!< AK8963_ASAZ
};

// Configuration bits mpu9250
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02

enum{
    BITS_FS_250DPS  = 0x00,
    BITS_FS_500DPS  = 0x08,
    BITS_FS_1000DPS = 0x10,
    BITS_FS_2000DPS = 0x18
};

enum{
    BITS_FS_2G   = 0x00,
    BITS_FS_4G   = 0x08,
    BITS_FS_8G   = 0x10,
    BITS_FS_16G  = 0x18,
    BITS_FS_MASK = 0x18
};

enum{
    BITS_DLPF_CFG_256HZ_NOLPF2 = 0x00,
    BITS_DLPF_CFG_188HZ        = 0x01,
    BITS_DLPF_CFG_98HZ         = 0x02,
    BITS_DLPF_CFG_42HZ         = 0x03,
    BITS_DLPF_CFG_20HZ         = 0x04,
    BITS_DLPF_CFG_10HZ         = 0x05,
    BITS_DLPF_CFG_5HZ          = 0x06,
    BITS_DLPF_CFG_2100HZ_NOLPF = 0x07,
    BITS_DLPF_CFG_MASK         = 0x07
};

#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG   0x80

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB

#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)
/**
 * MPU9250 I2C Address
 */
enum{
	MPU9250_I2C_ADD0 = 0x68,//!< MPU9250_I2C_ADD0 AD0 = L
	MPU9250_I2C_ADD1 = 0x69 //!< MPU9250_I2C_ADD1 AD0 = H
};

/**
 * axis
 */
enum{
	x = 0,//!< x
	y = 1,//!< y
	z = 2 //!< z
};

#define	   MPU9250_COMMS_I2C	0x01
#define	   MPU9250_COMMS_SPI	0x00

#endif /* MPU9250_H_ */
