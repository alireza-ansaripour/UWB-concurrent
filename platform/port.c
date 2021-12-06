/*! ----------------------------------------------------------------------------
 * @file	port.c
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
#include "sleep.h"
#include "port.h"
#include "stm32l4xx_hal.h"
#include "stm32_hal_legacy.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"

#define rcc_init(x)					RCC_Configuration(x)
#define systick_init(x)				SysTick_Configuration(x)
#define interrupt_init(x)			NVIC_Configuration(x)
#define usart_init(x)				USART_Configuration(x)
#define spi_init(x)					SPI_Configuration(x)
#define gpio_init(x)				GPIO_Configuration(x)
#define usb_init(x)					USB_Configuration(x)

SPI_HandleTypeDef SPIx_Handle;
UART_HandleTypeDef USARTx_Handle;
UART_HandleTypeDef USARTy_Handle;
TIM_HandleTypeDef  TIMx_Handle;
RNG_HandleTypeDef RNG_Handle;
__IO ITStatus USARTx_Ready = SET;
uint8_t USARTx_RX_BUFFER[3000];
uint8_t USARTx_TX_BUFFER[3000];

/* DW1000 IRQ handler definition. */
port_deca_isr_t port_deca_isr = NULL;

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

int No_Configuration(void)
{
	return -1;
}

unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

int SysTick_Configuration(void)
{
	if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);

	return 0;
}

//
//int NVIC_DisableDECAIRQ(void)
//{
//	EXTI_InitTypeDef EXTI_InitStructure;
//
//	/* Configure EXTI line */
//	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
//	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_NOIRQ;
//	EXTI_Init(&EXTI_InitStructure);
//
//	return 0;
//}


int NVIC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;

	// Enable GPIO used as DECA IRQ for interrupt
	GPIO_InitStructure.Pin = DECAIRQ;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN; //IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
	HAL_GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);
//	/* Connect EXTI Line to GPIO Pin */
//	GPIO_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);
//
//	/* Configure EXTI line */
//	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
//	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
//	EXTI_Init(&EXTI_InitStructure);

	/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* Enable and set EXTI Interrupt to the lowest priority */
//	HAL_NVIC_SetPriority(DECAIRQ_EXTI_IRQn, 15, 0);
//	HAL_NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);

	/* NVIC for DECAWAVE */
	HAL_NVIC_SetPriority(DECAIRQ_EXTI_IRQn, 0, 15);
	HAL_NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);

	/* NVIC for USART */
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 15);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);

	/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
	HAL_NVIC_SetPriority(USARTx_TX_DMA_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_TX_DMA_IRQn);

	/* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
	HAL_NVIC_SetPriority(USARTx_RX_DMA_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(USARTx_RX_DMA_IRQn);

	/* Set Interrupt Group Priority */
	HAL_NVIC_SetPriority(TIMx_IRQn, 2, 0);

	/* Enable the TIMx global Interrupt */
	HAL_NVIC_EnableIRQ(TIMx_IRQn);

	return 0;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));

  // TODO: The following line only checks IM0 to IM31 and does not check IM32 to IM39. Add IMR2 to support them too.
  enablestatus =  EXTI->IMR1 & EXTI_Line;
  if (enablestatus != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

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
void port_set_deca_isr(port_deca_isr_t deca_isr)
{
    /* Check DW1000 IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
    if (en)
    {
        port_DisableEXT_IRQ();
    }
    port_deca_isr = deca_isr;
    if (en)
    {
        port_EnableEXT_IRQ();
    }
}

int RCC_Configuration(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_4);
	HAL_RCC_DeInit();

	/* HSE is enabled after System reset, activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;	// 24 MHz
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1; // 24 / 1 = 24 MHz
	RCC_OscInitStruct.PLL.PLLN = 10; // 24 * 10 = 240 MHz
//	RCC_OscInitStruct.PLL.PLLR = 3; // 240 / 3 = 80 MHz
	RCC_OscInitStruct.PLL.PLLR = 4; // 240 / 4 = 60 MHz
	RCC_OscInitStruct.PLL.PLLP = 10; // 240 / 10 = 24 MHz
	RCC_OscInitStruct.PLL.PLLQ = 5; // 240 / 5 = 48 MHz
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

//	/*Select PLL 48 MHz output as USB clock source */
//	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
//	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLCLK;
//	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/*Select PLLQ output as RNG clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG;
	PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_FLASH_CLK_ENABLE();

	/* Enable SPI2 clock */
	__HAL_RCC_SPI2_CLK_ENABLE();

	/* Enable GPIOs clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* Enable USART Clock */
	USARTx_CLK_ENABLE();

	/* Enable DMA clock */
	USARTx_TX_DMA_CLK_ENABLE();
//	USARTx_RX_DMA_CLK_ENABLE();

//	/* Enable USB power on Pwrctrl CR2 register */
//	HAL_PWREx_EnableVddUSB();

	/* Enable TIM peripherals Clock */
	TIMx_CLK_ENABLE();

	/* Enable RNG controller clock */
	__HAL_RCC_RNG_CLK_ENABLE();

	return 0;
}

int USART_Configuration(void)
{
	static DMA_HandleTypeDef hdma_tx;
	static DMA_HandleTypeDef hdma_rx;
	GPIO_InitTypeDef GPIO_InitStructure;

	// USARTx setup
	USARTx_Handle.Instance        = USARTx;
	USARTx_Handle.Init.BaudRate   = 600000;
	USARTx_Handle.Init.WordLength = UART_WORDLENGTH_8B;
	USARTx_Handle.Init.StopBits   = UART_STOPBITS_1;
	USARTx_Handle.Init.Parity     = UART_PARITY_NONE;
	USARTx_Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	USARTx_Handle.Init.Mode       = UART_MODE_TX_RX;
	HAL_UART_Init(&USARTx_Handle);

	// USARTx TX pin setup
	GPIO_InitStructure.Pin = USARTx_TX;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = USARTx_TX_AF;
	HAL_GPIO_Init(USARTx_TX_GPIO, &GPIO_InitStructure);

	// USARTx RX pin setup
	GPIO_InitStructure.Pin = USARTx_RX;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = USARTx_RX_AF;
	HAL_GPIO_Init(USARTx_RX_GPIO, &GPIO_InitStructure);

	/* Configure the DMA handler for Transmission process */
	hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
	hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode                = DMA_NORMAL;
	hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_tx.Init.Request             = USARTx_TX_DMA_REQUEST;

	HAL_DMA_Init(&hdma_tx);

	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(&USARTx_Handle, hdmatx, hdma_tx);

//	/* Disable the half transfer interrupt */
//	__HAL_DMA_DISABLE_IT(&hdma_tx, DMA_IT_HT);

//	DMA_HandleTypeDef *hdma,
//	HAL_DMA_CallbackIDTypeDef CallbackID,
//	void (* pCallback)( DMA_HandleTypeDef * _hdma)
//	HAL_DMA_RegisterCallback(&hdma_tx, HAL_DMA_XFER_CPLT_CB_ID, &USARTx_DMA_TX_Complete_Callback);

	/* Configure the DMA handler for Reception process */
	hdma_rx.Instance                 = USARTx_RX_DMA_CHANNEL;
	hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_rx.Init.Mode                = DMA_NORMAL;
	hdma_rx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_rx.Init.Request             = USARTx_RX_DMA_REQUEST;

//	/* Disable the half transfer interrupt */
//	__HAL_DMA_DISABLE_IT(&hdma_rx, DMA_IT_HT);

	HAL_DMA_Init(&hdma_rx);

	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(&USARTx_Handle, hdmarx, hdma_rx);



    return 0;
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void)
{
    SPI_ChangeRate(SPI_BAUDRATEPRESCALER_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void)
{
    SPI_ChangeRate(SPI_BAUDRATEPRESCALER_4);
}

int SPI_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	HAL_SPI_DeInit(&SPIx_Handle);

	// SPIx Mode setup
	SPIx_Handle.Instance = SPIx;
	SPIx_Handle.Init.Direction = SPI_DIRECTION_2LINES;
	SPIx_Handle.Init.Mode = SPI_MODE_MASTER;
	SPIx_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPIx_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPIx_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPIx_Handle.Init.NSS = SPI_NSS_SOFT;
	SPIx_Handle.Init.BaudRatePrescaler = SPIx_PRESCALER;
	SPIx_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPIx_Handle.Init.CRCPolynomial = 7;
	SPIx_Handle.Init.CRCLength = SPI_CRC_LENGTH_8BIT;
	SPIx_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPIx_Handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	SPIx_Handle.Init.TIMode = SPI_TIMODE_DISABLE;

	HAL_SPI_Init(&SPIx_Handle);

	// SPIx SCK pin setup
	GPIO_InitStructure.Pin = SPIx_SCK;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = SPIx_SCK_AF;

	HAL_GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	// SPIx MOSI pin setup
	GPIO_InitStructure.Pin = SPIx_MOSI;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = SPIx_MOSI_AF;

	HAL_GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	// SPIx MISO pin setup
	GPIO_InitStructure.Pin = SPIx_MISO;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = SPIx_MISO_AF;

	HAL_GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	// SPIx CS pin setup
	GPIO_InitStructure.Pin = SPIx_CS;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);
//
//	// Disable SPIx SS Output
//	SPI_SSOutputCmd(SPIx, DISABLE);
//
//	// Enable SPIx
//	SPI_Cmd(SPIx, ENABLE);
//
//	// Set CS high
//	GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS);

	__HAL_SPI_ENABLE(&SPIx_Handle);

    return 0;
}

int GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure all unused GPIO port pins in Analog Input mode (floating input
	* trigger OFF), this will reduce the power consumption and increase the device
	* immunity against EMI/EMC */

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Enable GPIO used for LEDs
	GPIO_InitStructure.Pin = LEDRX;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LEDRX_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = LEDTX;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LEDTX_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = LEDL;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LEDL_GPIO, &GPIO_InitStructure);




	GPIO_InitStructure.Pin = DEBUGRX;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DEBUGRX_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = DEBUGTX;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DEBUGTX_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = DEBUGMAIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DEBUGMAIN_GPIO, &GPIO_InitStructure);

//	GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE);

    return 0;
}

//int USB_Configuration(void)
//{
//	/* Init Device Library */
//	USBD_Init(&USBD_Handle, &VCP_Desc, 0);
//
//	/* Register the CDC class */
//	USBD_RegisterClass(&USBD_Handle, &USBD_CDC);
//
//	/* Add CDC Interface Class */
//	USBD_CDC_RegisterInterface(&USBD_Handle, &USBD_CDC_fops);
//
//	/* Start Device Process */
//	USBD_Start(&USBD_Handle);
//}

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.Pin = DW1000_RSTn;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW1000_RSTn_GPIO, DW1000_RSTn, GPIO_PIN_RESET);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.Pin = DW1000_RSTn;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

    sleep_ms(2);
}

void led_off (led_t led)
{
	switch (led)
	{
	case LED_RX:
		HAL_GPIO_WritePin(LEDRX_GPIO, LEDRX, GPIO_PIN_RESET);
		break;
	case LED_TX:
		HAL_GPIO_WritePin(LEDTX_GPIO, LEDTX, GPIO_PIN_RESET);
		break;
	case LED_L:
		HAL_GPIO_WritePin(LEDL_GPIO, LEDL, GPIO_PIN_RESET);
		break;
	case LED_ALL:
		HAL_GPIO_WritePin(LEDRX_GPIO, LEDRX, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDTX_GPIO, LEDTX, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDL_GPIO, LEDL, GPIO_PIN_RESET);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

void led_on (led_t led)
{
	switch (led)
	{
	case LED_RX:
		HAL_GPIO_WritePin(LEDRX_GPIO, LEDRX, GPIO_PIN_SET);
		break;
	case LED_TX:
		HAL_GPIO_WritePin(LEDTX_GPIO, LEDTX, GPIO_PIN_SET);
		break;
	case LED_L:
		HAL_GPIO_WritePin(LEDL_GPIO, LEDL, GPIO_PIN_SET);
		break;
	case LED_ALL:
		HAL_GPIO_WritePin(LEDRX_GPIO, LEDRX, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDTX_GPIO, LEDTX, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDL_GPIO, LEDL, GPIO_PIN_SET);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}


void gpio_toggle(gpio_pin_t pin, GPIO_PinState state)
{
	switch (pin) {
		case DEBUG_RX:
			HAL_GPIO_WritePin(DEBUGRX_GPIO, DEBUGRX, state);
			break;
		case DEBUG_TX:
			HAL_GPIO_WritePin(DEBUGTX_GPIO, DEBUGTX, state);
			break;
		case DEBUG_MAIN:
			HAL_GPIO_WritePin(DEBUGMAIN_GPIO, DEBUGMAIN, state);
			break;
		default:
			break;
	}
}

void gpio_high(gpio_pin_t pin)
{
	gpio_toggle(pin, GPIO_PIN_SET);
}

void gpio_low(gpio_pin_t pin)
{
	gpio_toggle(pin, GPIO_PIN_RESET);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void)
{
	HAL_Init();

	rcc_init();
	gpio_init();
	interrupt_init();
	systick_init();
	spi_init();
    usart_init();
//    usb_init();
}

void Error_Handler()
{

}

uint32_t portGetRandom32(void)
{
	uint32_t ret;
	HAL_RNG_GenerateRandomNumber(&RNG_Handle, &ret);
	return ret;
}

uint32_t portGetRandom(uint32_t min, uint32_t max)
{
	uint32_t ret = portGetRandom32();
	uint32_t mod = max - min - 1;
	ret %= mod;
	ret += min;
	return ret;
}
