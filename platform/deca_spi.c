/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_spi.h"

extern SPI_HandleTypeDef SPIx_Handle;

///*! ------------------------------------------------------------------------------------------------------------------
// * Function: openspi()
// *
// * Low level abstract function to open and initialise access to the SPI device.
// * returns 0 for success, or -1 for error
// */
//int openspi(/*SPI_TypeDef* SPIx*/)
//{
//	// done by port.c, default SPI used is SPI1
//
//	return 0;
//
//} // end openspi()
//
///*! ------------------------------------------------------------------------------------------------------------------
// * Function: closespi()
// *
// * Low level abstract function to close the the SPI device.
// * returns 0 for success, or -1 for error
// */
//int closespi(void)
//{
//	while (port_SPIx_busy_sending()); //wait for tx buffer to empty
//
//	port_SPIx_disable();
//
//	return 0;
//
//} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize ("O3")
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{

	uint16 i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

//    HAL_SPI_Transmit(&SPI_Handle, headerBuffer, headerLength, 10);
//    HAL_SPI_Transmit(&SPI_Handle, bodyBuffer, bodylength, 10);

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	*((volatile uint8_t*)&SPIx->DR) = headerBuffer[i];
    	while ((SPIx->SR & SPI_FLAG_RXNE) == (uint16_t)RESET);

    	SPIx->DR ;
    }

    for(i=0; i<bodylength; i++)
    {
    	*((volatile uint8_t*)&SPIx->DR) = bodyBuffer[i];

    	while((SPIx->SR & SPI_FLAG_RXNE) == (uint16_t)RESET);

		SPIx->DR ;
	}

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{

	uint16 i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

//    HAL_SPI_Transmit(&SPI_Handle, headerBuffer, headerLength, 100);
//    HAL_SPI_Receive(&SPI_Handle, readBuffer, readlength, 100);

    /* Wait for SPIx Tx buffer empty */
    //while (port_SPIx_busy_sending());

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	*((volatile uint8_t*)&SPIx->DR) = headerBuffer[i];

     	while((SPIx->SR & SPI_FLAG_RXNE) == (uint16_t)RESET);

     	readBuffer[0] = (uint8_t)(SPIx->DR) ; // Dummy read as we write the header
    }

    for(i=0; i<readlength; i++)
    {
    	*((volatile uint8_t*)&SPIx->DR) = 0;  // Dummy write as we read the message body

    	while((SPIx->SR & SPI_FLAG_RXNE) == (uint16_t)RESET);

	   	readBuffer[i] = (uint8_t)(SPIx->DR) ;//port_SPIx_receive_data(); //this clears RXNE bit
    }

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;

    return 0;
} // end readfromspi()
