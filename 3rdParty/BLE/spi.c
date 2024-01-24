/**
 * @description SPI support library for stm32f4 boards derived
 *				from g4lvanix STM32F4-workarea and examples.
 */

#include <stm32f4xx.h>
#include <stm32f4xx_hal_uart.h>
#include "spi.h"
#include <stm32f4xx_ll_gpio.h>


extern SPI_HandleTypeDef hspi2;
extern void Error_Handler(void);



/* prescaler to be sent is SPI_BAUDRATEPRESCALER_64 defined in  stm32f4xx_hal_spi.h */
error_type init_spi2(SPI_TypeDef* SPIx, unsigned int prescaler)
{

	/*
	 * 1. Init SPI PINs done in hal_msp.c
	 * 2. Set the SPI clock, mode...etc.
	 * 3. Init RST, RDYN(input),REQN(output) PINs
	 * */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	 SPI_HandleTypeDef l_hspi2;
	error_type ReturnValue;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pins : RESET_PIN_Pin
	 * TODO: To be checked what will happen if the Output pin is set to be Pullup- conclusion for now, it is not necessary */
	GPIO_InitStruct.Pin = RESET_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : REQN_PIN_Pin
	 * Pullup must be activated, pin must be in HIGH state, not to be left floating. Datasheet pg15.*/

	GPIO_InitStruct.Pin = REQN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/*Configure GPIO pins : BOOT1_Pin RDYN_PIN_Pin
	 * TODO: To be checked if there is any special reason why RDYN pin should be BOOT1_Pin */
	GPIO_InitStruct.Pin = RDYN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



	// initial pin state high for MOSI and REQN pins
	LL_GPIO_SetOutputPin(REQN_GPIO_PORT, REQN_PIN);

	LL_GPIO_ResetOutputPin(SPI_COMMON_PORT, MISO_PIN);
	LL_GPIO_ResetOutputPin(SPI_COMMON_PORT, MOSI_PIN);
	LL_GPIO_ResetOutputPin(SPI_COMMON_PORT, SCLK_PIN);



	/* SPI2 parameter configuration*/
	l_hspi2.Instance = SPIx;
	l_hspi2.Init.Mode = SPI_MODE_MASTER;
	l_hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	l_hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	l_hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	l_hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	l_hspi2.Init.NSS = SPI_NSS_SOFT;
	l_hspi2.Init.BaudRatePrescaler = prescaler;
	l_hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
	l_hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	l_hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	l_hspi2.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&l_hspi2) != HAL_OK)
	{
		Error_Handler();
	}

	ReturnValue = E_SUCCESS;

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
	error_type ReturnValue;
	while (length--)
	{
		SPIx->DR = *txbuf; /* write data to be transmitted to the SPI data register */
		while( !(SPIx->SR & SPI_FLAG_TXE) ); /* wait until transmit complete */
		while( !(SPIx->SR & SPI_FLAG_RXNE) ); /* wait until receive complete */
		while( SPIx->SR & SPI_FLAG_BSY); /* wait until SPI is not busy anymore */
		*rxbuf = SPIx->DR; /* return received data from SPI data register */
		txbuf++;
		rxbuf++;
	}
	ReturnValue = E_SUCCESS;

    return ReturnValue;
}
