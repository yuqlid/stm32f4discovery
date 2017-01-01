#include "stm32f4xx_empl_drv.h"
#include "i2c.h"

#include <string.h>

static unsigned char stm32f4xx_empl_i2c_wb[MAX_WRITE_SIZE];

int stm32f4xx_i2c_write (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char const *data)
{	
	if (length + 1 > MAX_WRITE_SIZE)
		return -1;
	
	stm32f4xx_empl_i2c_wb[0] = reg_addr;
	memcpy (stm32f4xx_empl_i2c_wb + 1, data, length);
	/*
    select();
    if(HAL_SPI_Transmit(&hspi1, (uint8_t*)stm32f4xx_empl_i2c_wb, length + 1, 100) != HAL_OK)Error_Handler();
    deselect();
    */
	while (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) == SET) ;

	if(HAL_I2C_Mem_Write (&hi2c2, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length , 1000) != HAL_OK)Error_Handler();
//	if(HAL_I2C_Master_Transmit (&hi2c1, slave_addr << 1, stm32f4xx_empl_i2c_wb, length + 1, 1000) != HAL_OK)Error_Handler();

	return 0;
}

													
int stm32f4xx_i2c_read  (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char *data)
{
    /*
    uint8_t txdata_buf= reg_addr | 0x80;    //0x80 = READ_FLAG
    select();
    if(HAL_SPI_Transmit(&hspi1,&txdata_buf, 1, 100) != HAL_OK)Error_Handler();
    if(HAL_SPI_Receive(&hspi1,(uint8_t*)data, length, 100) != HAL_OK)Error_Handler();
    deselect();
    */
    while (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) == SET) ;

    if(HAL_I2C_Mem_Read (&hi2c2, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length , 1000) != HAL_OK)Error_Handler();
	//if(HAL_I2C_Master_Transmit(&hi2c1, slave_addr << 1, &reg_addr, 1, 1000) != HAL_OK)Error_Handler();
	//if(HAL_I2C_Master_Receive (&hi2c1, slave_addr << 1, data, length, 1000) != HAL_OK)Error_Handler();

	return 0;
}


void stm32f4xx_delay_ms  (unsigned long num_ms)
{
	HAL_Delay(num_ms);
}


void stm32f4xx_get_ms    (unsigned long *count)
{
	(*count) = HAL_GetTick();
}


void stm32f4xx_log_i (char const *s, ...)
{
	return;
}


void stm32f4xx_log_e (char const *s, ...)
{
	return;
}
/*
void select(void) {

	HAL_GPIO_WritePin(MPU9250_NCS_GPIO_PORT, MPU9250_NCS_PIN, GPIO_PIN_RESET);
}
void deselect(void) {

	HAL_GPIO_WritePin(MPU9250_NCS_GPIO_PORT, MPU9250_NCS_PIN, GPIO_PIN_SET);
}
*/
