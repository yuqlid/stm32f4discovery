

#include "mpu9250.h"

#ifdef HAL_SPI_MODULE_ENABLED
SPI_HandleTypeDef*	 mpuSpiHandle;
GPIO_TypeDef* 		mpu9250_NCS_GPIO;
uint16_t 			mpu9250_NCS_PIN;
#endif /* HAL_SPI_MODULE_ENABLED */
#ifdef HAL_I2C_MODULE_ENABLED
I2C_HandleTypeDef*	mpuI2cHandle;
uint8_t				mpuaddress;
#endif /* HAL_I2C_MODULE_ENABLED */
uint8_t				commtype;
#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @name mpu9250
 * @brief mpu9250 constructor (I2C)
 * @param I2cHandle
 * @note  default I2C address = 0x68
 */
mpu9250::mpu9250(I2C_HandleTypeDef* I2cHandle){

	mpuI2cHandle = I2cHandle;
	mpuaddress = MPU9250_I2C_ADD0;
	commtype = MPU9250_COMMS_I2C;
}

/**
 * @name mpu9250
 * @brief mpu9250 constructor (I2C)
 * @param I2cHandle
 * @param address ( 0x68 or 0x69 )
 */
mpu9250::mpu9250(I2C_HandleTypeDef* I2cHandle, uint8_t address){

	mpuI2cHandle = I2cHandle;
	//if(address != MPU9250_I2C_ADD0 && address != MPU9250_I2C_ADD1 )Error_Handler();
	mpuaddress = address;
	commtype = MPU9250_COMMS_I2C;
}
#endif /* HAL_I2C_MODULE_ENABLED */
#ifdef HAL_SPI_MODULE_ENABLED
/**
 * @name  mpu9250
 * @brief mpu9250 constructor (SPI)
 * @param SpiHandle
 * @param NCS_GPIO
 * @param NCS_PIN
 */
mpu9250::mpu9250(SPI_HandleTypeDef* SpiHandle, GPIO_TypeDef* NCS_GPIO, uint16_t NCS_PIN){

	mpuSpiHandle = SpiHandle;
	mpu9250_NCS_GPIO = NCS_GPIO;
	mpu9250_NCS_PIN = NCS_PIN;
	deselect();
	commtype = MPU9250_COMMS_SPI;
}
#endif /* HAL_SPI_MODULE_ENABLED */
unsigned int mpu9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
	if(commtype == MPU9250_COMMS_SPI){
#ifdef HAL_SPI_MODULE_ENABLED
		uint8_t txdata_buf[2] = {WriteAddr, WriteData};
		uint8_t rxdata_buf[2];
		select();
		HAL_SPI_TransmitReceive(mpuSpiHandle, (uint8_t*)txdata_buf, (uint8_t*)rxdata_buf, 2 , 1);
		deselect();
#endif /* HAL_SPI_MODULE_ENABLED */
	}else if(commtype == MPU9250_COMMS_I2C){
#ifdef HAL_I2C_MODULE_ENABLED
		HAL_I2C_Mem_Write(mpuI2cHandle, mpuaddress << 1, WriteAddr, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 10);
#endif /* HAL_I2C_MODULE_ENABLED */
	}
}

unsigned int  mpu9250::ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
	uint8_t rxdata_buf[2];

	if(commtype == MPU9250_COMMS_SPI){
#ifdef HAL_SPI_MODULE_ENABLED
		uint8_t txdata_buf[2] = {WriteAddr | READ_FLAG, WriteData};
		select();
		HAL_SPI_TransmitReceive(mpuSpiHandle, (uint8_t*)txdata_buf, (uint8_t*)rxdata_buf, 2 , 1);
		deselect();
#endif /* HAL_SPI_MODULE_ENABLED */
	}else if(commtype == MPU9250_COMMS_I2C){
#ifdef HAL_I2C_MODULE_ENABLED
		HAL_I2C_Mem_Read(mpuI2cHandle, mpuaddress << 1, WriteAddr| READ_FLAG, I2C_MEMADD_SIZE_8BIT, &rxdata_buf[1], 1, 10);
#endif /* HAL_I2C_MODULE_ENABLED */
	}
    return rxdata_buf[1];
}

void mpu9250::ReadRegs( uint8_t ReadAddr, uint8_t* ReadBuf, unsigned int Bytes )
{
	uint8_t txdata_buf= ReadAddr | READ_FLAG;

	if(commtype == MPU9250_COMMS_SPI){
#ifdef HAL_SPI_MODULE_ENABLED
		select();
		HAL_SPI_Transmit(mpuSpiHandle,&txdata_buf, 1, 1);
		HAL_SPI_Receive(mpuSpiHandle,(uint8_t*)ReadBuf, Bytes, 1);
		deselect();
#endif /* HAL_SPI_MODULE_ENABLED */
	}else if(commtype == MPU9250_COMMS_I2C){
#ifdef HAL_I2C_MODULE_ENABLED
		HAL_I2C_Mem_Read(mpuI2cHandle, mpuaddress << 1 , txdata_buf, I2C_MEMADD_SIZE_8BIT, (uint8_t*)ReadBuf, Bytes, 10);
#endif /* HAL_I2C_MODULE_ENABLED */
	}
}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ 
BITS_DLPF_CFG_5HZ 
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/
#define MPU_InitRegNum 17

bool mpu9250::init(int sample_rate_div,int low_pass_filter){
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {0x80, MPUREG_PWR_MGMT_1},     // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
        {0x00, MPUREG_ACCEL_CONFIG},   // +-2G
        {0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, MPUREG_INT_PIN_CFG},    //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        
    };

    for(i=0; i<MPU_InitRegNum; i++) {
		WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		HAL_Delay(10);  //I2C must slow down the write speed, otherwise it won't work
    }

    set_acc_scale(2);
    set_gyro_scale(250);
    
    //AK8963_calib_Magnetometer();  //Can't load this function here , strange problem?
    return 0;
}
/**
 * @name   set_acc_scale
 * @brief  Set the right range for the accelerometers.
 * @param  scale ( accelerometer range ) :
 *          BITS_FS_2G
 *          BITS_FS_4G
 *          BITS_FS_8G
 *          BITS_FS_16G
 * @return the range set (2,4,8 or 16)
 */
unsigned int mpu9250::set_acc_scale(int scale){
    unsigned int temp_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_2G:
            acc_divider = 16384;
        break;
        case BITS_FS_4G:
            acc_divider = 8192;
        break;
        case BITS_FS_8G:
            acc_divider = 4096;
        break;
        case BITS_FS_16G:
            acc_divider = 2048;
        break;   
    }
    temp_scale = ReadReg(MPUREG_ACCEL_CONFIG, 0x00);
    
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale = 2;
        break;
        case BITS_FS_4G:
            temp_scale = 4;
        break;
        case BITS_FS_8G:
            temp_scale = 8;
        break;
        case BITS_FS_16G:
            temp_scale = 16;
        break;   
    }
    return temp_scale;
}

/**
 * @name   set_gyro_scale
 * @brief  Set the right range for the gyroscopes.
 * @param  scale ( gyroscopes range ) :
 *          BITS_FS_250DPS
 *          BITS_FS_500DPS
 *          BITS_FS_1000DPS
 *          BITS_FS_2000DPS
 * @return the range set (250,500,1000 or 2000)
 */
unsigned int mpu9250::set_gyro_scale(int scale){
    unsigned int temp_scale;
    WriteReg(MPUREG_GYRO_CONFIG, scale);
    switch (scale){
        case BITS_FS_250DPS:
            gyro_divider=131;
        break;
        case BITS_FS_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_divider=16.4;
        break;   
    }
    temp_scale=ReadReg(MPUREG_GYRO_CONFIG, 0x00);
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu9250 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/

/**
 * @name   whoami
 * @brief  check mpu9250
 * @return 0x71
 */
unsigned int mpu9250::whoami(){
	uint8_t response;

	response = ReadReg(MPUREG_WHOAMI, 0x00);
	return response;
}


/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void mpu9250::read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        accelerometer_data_raw[i] = bit_data;
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
    
}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void mpu9250::read_rot()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        gyroscope_data_raw[i] = bit_data;
        data=(float)bit_data;
        gyroscope_data[i]=data/gyro_divider;
    }
}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data. 
returns the value in °C
-----------------------------------------------------------------------------------------------*/
void mpu9250::read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    ReadRegs(MPUREG_TEMP_OUT_H,response,2);

    bit_data=((int16_t)response[0]<<8)|response[1];
    Temperature_raw = bit_data;
    data=(float)bit_data;
    Temperature=(data/340)+36.53;
    //deselect();
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
void mpu9250::calib_acc()
{
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=ReadReg(MPUREG_ACCEL_CONFIG, 0x00);
    set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[x]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    calib_data[y]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    calib_data[z]=((response[2]&11100000)>>3)|((response[3]&00000011));

    set_acc_scale(temp_scale);
}
uint8_t mpu9250::AK8963_whoami(){
    uint8_t response;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
 
    HAL_Delay(10);
    response=ReadReg(MPUREG_EXT_SENS_DATA_00, 0x00);    //Read I2C

    return response;
}
void mpu9250::AK8963_calib_Magnetometer(){
    uint8_t response[3];
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    //HAL_Delay(1);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    for(i=0; i<3; i++) {
        data=response[i];
        Magnetometer_ASA[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}
void mpu9250::AK8963_read_Magnetometer(){
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

//    delay_ms(1);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        Magnetometer_raw[i] = bit_data;
        data=(float)bit_data;
        Magnetometer[i]=data*Magnetometer_ASA[i];
    }
}
void mpu9250::read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

    //Send I2C command at first
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    //Get accelerometer value
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        accelerometer_data_raw[i] = bit_data;
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
    //Get temperature
    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
    data=(float)bit_data;
    Temperature_raw = bit_data;
    Temperature=((data-21)/333.87)+21;
    //Get gyroscop value
    for(i=4; i<7; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        gyroscope_data_raw[i-4] = bit_data;
        data=(float)bit_data;
        gyroscope_data[i-4]=data/gyro_divider;
    }
    //Get Magnetometer value
    for(i=7; i<10; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        Magnetometer_raw[i-7] = bit_data;
        data=(float)bit_data;
        Magnetometer[i-7]=data*Magnetometer_ASA[i-7];
    }
}
#ifdef HAL_I2C_MODULE_ENABLED
/**
 * @name isonline
 * @brief check mpu9250 is connected
 * @return true or false
 */
bool mpu9250::isonline(){
	if(commtype == MPU9250_COMMS_I2C){
		if(HAL_I2C_IsDeviceReady(mpuI2cHandle, mpuaddress << 1, 100, 100) != HAL_OK)return false;
		else return true;
	}else{
		return true;
	}
}
#endif /* HAL_I2C_MODULE_ENABLED */
#ifdef HAL_SPI_MODULE_ENABLED
/*-----------------------------------------------------------------------------------------------
                                SPI SELECT AND DESELECT
usage: enable and disable mpu9250 communication bus
-----------------------------------------------------------------------------------------------*/
void mpu9250::select() {

	if(commtype == MPU9250_COMMS_SPI)HAL_GPIO_WritePin(mpu9250_NCS_GPIO, mpu9250_NCS_PIN, GPIO_PIN_RESET);
}
void mpu9250::deselect() {

	if(commtype == MPU9250_COMMS_SPI)HAL_GPIO_WritePin(mpu9250_NCS_GPIO, mpu9250_NCS_PIN, GPIO_PIN_SET);
}
#endif /* HAL_SPI_MODULE_ENABLED */
