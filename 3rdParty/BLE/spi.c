/**
 * @description SPI support library for stm32f4 boards derived
 *				from g4lvanix STM32F4-workarea and examples.
 */

#include <stm32f4xx.h>
#include <stm32f4xx_hal_uart.h>
#include "spi.h"
#include <stm32f4xx_ll_gpio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;
extern void Error_Handler(void);



/* prescaler to be sent is SPI_BAUDRATEPRESCALER_64 defined in  stm32f4xx_hal_spi.h */
error_type init_spi2(SPI_TypeDef* SPIxh, unsigned int prescaler)
{

	/*
	 * 1. Init SPI PINs done in hal_msp.c
	 * 2. Set the SPI clock, mode...etc.
	 * 3. Init RST, RDYN(input),REQN(output) PINs
	 * */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	error_type ReturnValue = E_SUCCESS;


//	/* GPIO Ports Clock Enable */
//	__HAL_RCC_GPIOE_CLK_ENABLE();
//	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOH_CLK_ENABLE();
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pins : RESET_PIN_Pin
	 * TODO: To be checked what will happen if the Output pin is set to be Pullup- conclusion for now, it is not necessary */
	/*Configure GPIO pins : REQN_PIN_Pin
	 * Pullup must be activated, pin must be in HIGH state, not to be left floating. Datasheet pg15.*/

	/* SPI2 parameter configuration*/
	  hspi2.Instance = SPIxh;
	  hspi2.Init.Mode = SPI_MODE_MASTER;
	  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi2.Init.NSS = SPI_NSS_SOFT;
	  hspi2.Init.BaudRatePrescaler = prescaler;
	  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi2.Init.CRCPolynomial = 10;



	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		//Error_Handler();
		ReturnValue = E_GENERAL_FAIL;
	}

	if(E_SUCCESS == ReturnValue)
	{

	HAL_GPIO_WritePin(REQN_GPIO_PORT, REQN_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = REQN_PIN ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(REQN_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = RESET_PIN ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(RESET_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = RDYN_PIN;
	GPIO_InitStruct.Mode = GPIO_PULLUP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(RDYN_GPIO_PORT, &GPIO_InitStruct);

	}
	else
	{
		// Do nothing
	}

	/* SPI_Cmd not used ? */




	return ReturnValue;
}

error_type send_byte_SPI(SPI_TypeDef* SPIx, uint8_t data)
{

	error_type ReturnValue;
	volatile uint32_t temp;

	temp = 0;
	SPIx->DR = data;
	while( !(SPIx->SR & SPI_FLAG_TXE) );
	while( !(SPIx->SR & SPI_FLAG_RXNE) );
	while( SPIx->SR & SPI_FLAG_BSY );

	temp = SPIx->DR;
	if(temp != 0)
	{
		ReturnValue = E_SUCCESS;
	}
    return ReturnValue;
}

error_type recv_byte_SPI(SPI_TypeDef* SPIx, uint8_t *data)
{
	error_type ReturnValue;

	SPIx->DR = 0xFF;
	while( !(SPIx->SR & SPI_FLAG_TXE) );
	while( !(SPIx->SR & SPI_FLAG_RXNE) );
	while( SPIx->SR & SPI_FLAG_BSY );
	*data = SPIx->DR;

	ReturnValue = E_SUCCESS;

    return ReturnValue;
}

error_type send_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length)
{
	error_type ReturnValue;

	while (length--)
	{
		send_byte_SPI(SPIx, *data);
		data++;
	}

	ReturnValue = E_SUCCESS;
    return ReturnValue;
}

error_type recv_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length)
{

	error_type ReturnValue;
	while (length--)
	{
		recv_byte_SPI(SPIx, data);
		data++;
	}

	ReturnValue = E_SUCCESS;

    return ReturnValue;
}

error_type transmit_SPI(SPI_TypeDef* SPIx, uint8_t *txbuf, uint8_t *rxbuf, uint16_t length)
{

	error_type ReturnValue = E_SUCCESS;

	ReturnValue = HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, length, 100);

//	while (length--)
//	{
//		SPIx->DR = *txbuf; /* write data to be transmitted to the SPI data register */
//		while( !(SPIx->SR & SPI_FLAG_TXE) ); /* wait until transmit complete */
//		while( !(SPIx->SR & SPI_FLAG_RXNE) ); /* wait until receive complete */
//		while( SPIx->SR & SPI_FLAG_BSY); /* wait until SPI is not busy anymore */
//		*rxbuf = SPIx->DR; /* return received data from SPI data register */
//		txbuf++;
//		rxbuf++;
//	}



    return ReturnValue;
}
