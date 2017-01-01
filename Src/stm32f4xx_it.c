/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "led.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    uint32_t IntStat = huart2.Instance->SR;
    if(IntStat & USART_SR_RXNE)
    {
        led_toggle(LED6);
        /* Advance buffer head. */
        unsigned int tempRX_Head = ((&USARTx_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

        /* Check for overflow. */
        unsigned int tempRX_Tail = (&USARTx_Buf)->RX_Tail;
        uint8_t data =  huart2.Instance->DR;

        if (tempRX_Head == tempRX_Tail) {
            /* Overflow MAX size Situation */
            /* Disable the UART Receive interrupt */
            huart2.Instance->CR1 &= ~(USART_CR1_RXNEIE);
        }else{
            (&USARTx_Buf)->RX[(&USARTx_Buf)->RX_Head] = data;
            (&USARTx_Buf)->RX_Head = tempRX_Head;
        }
    }

    if(IntStat & USART_SR_TXE)
    {
        led_toggle(LED3);
        /* Check if all data is transmitted. */
        unsigned int tempTX_Tail = (&USARTx_Buf)->TX_Tail;
        if ((&USARTx_Buf)->TX_Head == tempTX_Tail){
            /* Overflow MAX size Situation */
            /* Disable the UART Transmit interrupt */
            huart2.Instance->CR1 &= ~(USART_CR1_TXEIE);
        }else{
            /* Start transmitting. */
            uint8_t data = (&USARTx_Buf)->TX[(&USARTx_Buf)->TX_Tail];
            huart2.Instance->DR = data;

            /* Advance buffer tail. */
            (&USARTx_Buf)->TX_Tail = ((&USARTx_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
        }
    }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
