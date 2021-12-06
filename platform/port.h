/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l433xx.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_gpio_ex.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <string.h>

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond
 * tick timer. */
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1000

#define STM32_UUID ((uint32_t *)0x1FFF7590)

#define UINT40_MAX	0x10000000000UL
#define UINT32_HALF	0x80000000UL

#define CONCAT2(a, b) a ## b
#define CONCAT(a, b) CONCAT2(a, b)

typedef enum
{
    LED_RX,
    LED_TX,
    LED_L,
    LED_ALL
} led_t;

#define LEDRX						GPIO_PIN_15
#define LEDRX_GPIO					GPIOA
#define LEDTX						GPIO_PIN_8
#define LEDTX_GPIO					GPIOB
#define LEDL						GPIO_PIN_1
#define LEDL_GPIO					GPIOB


typedef enum
{
	DEBUG_RX,
	DEBUG_TX,
	DEBUG_MAIN
} gpio_pin_t;

#define DEBUGRX						GPIO_PIN_5
#define DEBUGRX_GPIO				GPIOB
#define DEBUGTX						GPIO_PIN_4
#define DEBUGTX_GPIO				GPIOB
#define DEBUGMAIN					GPIO_PIN_0
#define DEBUGMAIN_GPIO				GPIOB


#define SPIx_PRESCALER				SPI_BAUDRATEPRESCALER_4
#define SPIx						SPI2
#define SPIx_CS						GPIO_PIN_8
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_PIN_13
#define SPIx_SCK_AF					GPIO_AF5_SPI2
#define SPIx_MISO					GPIO_PIN_14
#define SPIx_MISO_AF                GPIO_AF5_SPI2
#define SPIx_MOSI					GPIO_PIN_15
#define SPIx_MOSI_AF                GPIO_AF5_SPI2
#define SPIx_GPIO					GPIOB


#define DW1000_RSTn					GPIO_PIN_5
#define DW1000_RSTn_GPIO			GPIOA

#define DECAIRQ                     GPIO_PIN_11
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI_IRQn           EXTI15_10_IRQn

#define DECAWAKEUP                  GPIO_PIN_10
#define DECAWAKEUP_GPIO             GPIOB


//#define USARTx						USART1
//#define USARTx_CLK_ENABLE()			__HAL_RCC_USART1_CLK_ENABLE()
//#define USARTx_TX					GPIO_PIN_9
//#define USARTx_TX_GPIO				GPIOA
//#define USARTx_TX_AF				GPIO_AF7_USART1
//#define USARTx_RX					GPIO_PIN_10
//#define USARTx_RX_GPIO				GPIOA
//#define USARTx_RX_AF				GPIO_AF7_USART1
//#define USARTx_IRQn					USART1_IRQn
//#define USARTx_IRQHandler			USART1_IRQHandler

#define USARTx						USART2
#define USARTx_CLK_ENABLE()			__HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_TX					GPIO_PIN_2
#define USARTx_TX_GPIO				GPIOA
#define USARTx_TX_AF				GPIO_AF7_USART2
#define USARTx_RX					GPIO_PIN_3
#define USARTx_RX_GPIO				GPIOA
#define USARTx_RX_AF				GPIO_AF7_USART2
#define USARTx_IRQn					USART2_IRQn
#define USARTx_IRQHandler			USART2_IRQHandler

#define USARTx_TX_DMA_CLK_ENABLE()	__HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_TX_DMA_CHANNEL		DMA1_Channel7
#define USARTx_TX_DMA_REQUEST		DMA_REQUEST_2
#define USARTx_TX_DMA_IRQn			DMA1_Channel7_IRQn
#define USARTx_TX_DMA_IRQHandler	DMA1_Channel7_IRQHandler

#define USARTx_RX_DMA_CLK_ENABLE()	__HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_DMA_CHANNEL		DMA1_Channel6
#define USARTx_RX_DMA_REQUEST		DMA_REQUEST_2
#define USARTx_RX_DMA_IRQn			DMA1_Channel6_IRQn
#define USARTx_RX_DMA_IRQHandler	DMA1_Channel6_IRQHandler


/* Definition for TIMx clock resources */
#define TIMx                             TIM2
#define TIMx_CLK_ENABLE                  __HAL_RCC_TIM2_CLK_ENABLE
#define TIMx_FORCE_RESET()               __HAL_RCC_USART2_FORCE_RESET()
#define TIMx_RELEASE_RESET()             __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                        TIM2_IRQn
#define TIMx_IRQHandler                  TIM2_IRQHandler

#define port_USARTx_busy_sending()	(USART_GetFlagStatus((USARTx),(USART_FLAG_TXE))==(RESET))
#define port_USARTx_no_data()		(USART_GetFlagStatus((USARTx),(USART_FLAG_RXNE))==(RESET))
#define port_USARTx_send_data(x)	USART_SendData((USARTx),(uint8_t)(x))
#define port_USARTx_receive_data()	USART_ReceiveData(USARTx)

#define port_GET_stack_pointer()		__get_MSP()

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 HAL_GPIO_ReadPin(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

void led_on(led_t led);
void led_off(led_t led);

void gpio_toggle(gpio_pin_t pin, GPIO_PinState state);
void gpio_high(gpio_pin_t pin);
void gpio_low(gpio_pin_t pin);

/* DW1000 IRQ (EXTI9_5_IRQ) handler type. */
typedef void (*port_deca_isr_t)(void);

/* DW1000 IRQ handler declaration. */
extern port_deca_isr_t port_deca_isr;

extern UART_HandleTypeDef USARTx_Handle;
extern TIM_HandleTypeDef TIMx_Handle;
extern RNG_HandleTypeDef RNG_Handle;
extern PCD_HandleTypeDef PCD_Handle;
extern __IO ITStatus USARTx_Ready;
extern uint8_t USARTx_RX_BUFFER[3000];
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr);

void SPI_ChangeRate(uint16_t scalingfactor);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void);

unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()

void reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);

void Error_Handler();

uint32_t portGetRandom32(void);
uint32_t portGetRandom(uint32_t min, uint32_t max);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
