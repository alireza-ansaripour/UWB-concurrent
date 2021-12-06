/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal.h"
#include "port.h"
#include "instance.h"
#include "ring_buffer.h"

/* Tick timer count. */
volatile unsigned long time32_incr;

void SysTick_Handler(void)
{
    time32_incr++;
}

/**
  * @brief  This function handles external lines 10 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(DECAIRQ);

    do
    {
        port_deca_isr();
    } while (port_CheckEXT_IRQ() == 1);

    /* Clear EXTI Line 5 Pending Bit */
//    EXTI_ClearITPendingBit(DECAIRQ_EXTI);

//    instance_tx();
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data transmission
  */
void USARTx_IRQHandler(void)
{
	HAL_UART_IRQHandler(&USARTx_Handle);
}



///**
//  * @brief  Tx Transfer completed callback
//  * @param  UartHandle: UART handle.
//  * @note   This example shows a simple way to report end of IT Tx transfer, and
//  *         you can add your own implementation.
//  * @retval None
//  */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//	UNUSED(UartHandle);
//	/* Set transmission flag: transfer complete */
//	USARTx_Ready = SET;
//}

uint16_t c1 = 0;
uint16_t c2 = 0;
/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data reception
  */
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	ring_buffer_free_chunk();
}

void USARTx_TX_DMA_IRQHandler(void)
{
	uint32_t flag_it = USARTx_Handle.hdmatx->DmaBaseAddress->ISR;
	uint32_t source_it = USARTx_Handle.hdmatx->Instance->CCR;
	uint8_t * data_ptr;
	uint16_t data_len;

	c1++;

	HAL_DMA_IRQHandler(USARTx_Handle.hdmatx);
	//if ((RESET != (flag_it & (DMA_FLAG_HT1 << USARTx_Handle.hdmatx->ChannelIndex))) && (RESET != (source_it & DMA_IT_HT)))
	if ((RESET != (flag_it & (DMA_FLAG_TC1 << USARTx_Handle.hdmatx->ChannelIndex))) && (RESET != (source_it & DMA_IT_TC)))
	{
		c2++;
		HAL_UART_GetState(&USARTx_Handle);

//		if (HAL_UART_GetState(&USARTx_Handle) == HAL_BUSY){
//			led_on(LED_ALL);
//		}

//		data_len = ring_buffer_get_chunk(&data_ptr);
//		if (data_len)
//		{
//			HAL_UART_Transmit_DMA(&USARTx_Handle, data_ptr, data_len);
//		}
	}

}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data reception
  */
void USARTx_RX_DMA_IRQHandler(void)
{
//	uint32_t flag_it = USARTx_Handle.hdmarx->DmaBaseAddress->ISR;
//	uint32_t source_it = USARTx_Handle.hdmarx->Instance->CCR;
	HAL_DMA_IRQHandler(USARTx_Handle.hdmarx);
////	sleep_ms(1);
//	//if ((RESET != (flag_it & (DMA_FLAG_TC1 << USARTx_Handle.hdmarx->ChannelIndex))) && (RESET != (source_it & DMA_IT_TC)))
//	if ((RESET != (flag_it & (DMA_FLAG_HT1 << USARTx_Handle.hdmarx->ChannelIndex))) && (RESET != (source_it & DMA_IT_HT)))
//	{
//		HAL_UART_Receive_DMA(&USARTx_Handle, USARTx_RX_BUFFER, 16);
//	}
}


/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TIMx_Handle);
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
