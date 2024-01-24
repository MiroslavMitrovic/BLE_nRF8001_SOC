/**
 * @filename usart.h
 * @description USART support library for stm32f4 boards
 * @author Nicholas Shrake, <shraken@gmail.com>
 */

#ifndef _USART_H
#define _USART_H

/**
 * CONSTANTS
 */

/**
 * PROTOTYPES
 */

void init_usart2(uint32_t baudrate);
void puts_usart(USART_TypeDef* USARTx, volatile char *s);

void USART_SendData(USART_TypeDef* USART,  volatile char ch);
uint32_t USART_GetTCFlagStatus(USART_TypeDef* USART);
#endif
