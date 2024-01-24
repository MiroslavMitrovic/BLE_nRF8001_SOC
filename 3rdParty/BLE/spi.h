/**
 * @description SPI support library for stm32f4 boards
 */

#ifndef SPI_H
#define SPI_H

#include <stm32f4xx.h>
#include "globals.h"


/**
 * CONSTANTS
 */

#define NRF8001_SPI		SPI2

#define SPI_COMMON_PORT GPIOB
#define MOSI_PIN        GPIO_PIN_15
#define MISO_PIN        GPIO_PIN_14
#define SCLK_PIN        GPIO_PIN_13

#define RESET_GPIO_PORT GPIOD
#define RESET_PIN		GPIO_PIN_8

#define RDYN_GPIO_PORT 	GPIOB
#define RDYN_PIN		GPIO_PIN_10

#define REQN_GPIO_PORT	GPIOB
#define REQN_PIN 		GPIO_PIN_11

/*TODO: To be tested if LL can be used in this case */
#define RST_LOW         LL_GPIO_ResetOutputPin(RESET_GPIO_PORT, RESET_PIN)
#define RST_HIGH        LL_GPIO_SetOutputPin(RESET_GPIO_PORT, RESET_PIN)

#define REQN_LOW     	LL_GPIO_ResetOutputPin(REQN_GPIO_PORT, REQN_PIN)
#define REQN_HIGH    	LL_GPIO_SetOutputPin(REQN_GPIO_PORT, REQN_PIN)

/**
 * PROTOTYPES
 */

error_type init_spi2(SPI_TypeDef* SPIx, unsigned int prescaler);
error_type send_byte_SPI(SPI_TypeDef* SPIx, uint8_t byte);
error_type recv_byte_SPI(SPI_TypeDef* SPIx, uint8_t *byte);
error_type send_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length);
error_type recv_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length);
error_type transmit_SPI(SPI_TypeDef* SPIx, uint8_t *txbuf, uint8_t *rxbuf, uint16_t length);

#endif
