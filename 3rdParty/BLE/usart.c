/**
 * @description UART support library for stm32f4 boards
 */

#include <stm32f4xx.h>
#include <stm32f4xx_hal_uart.h>
#include <usart.h>
#include <stm32f4xx_ll_usart.h>

extern UART_HandleTypeDef huart2;

extern void Error_Handler(void);

void init_usart2(uint32_t baudrate) {


/*	1. Set the USART pins as interrupts Done in MSP, call from HAL_UART_Init function
 * 	2. Init the USART  done as above.
 * 	3. Set USART as Interrupt Done in MSP, call from HAL_UART_Init function
 */

   /* Init Uart */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = baudrate;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
    	Error_Handler();
    }

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE(&huart2);

}
/*sends data byte by byte */
void puts_usart(USART_TypeDef* USARTx, volatile char *s) {
    while(*s){
		  // wait until data register is empty
		  while( !(USARTx->SR & 0x00000040) );
		  HAL_UART_Transmit(&huart2, (uint8_t*)s, 1, 10);
		  *s++;
    }
}

void USART_SendData(USART_TypeDef* USART,  volatile char ch)
{
	 LL_USART_TransmitData8(USART,(uint8_t)ch);
}

uint32_t USART_GetTCFlagStatus(USART_TypeDef* USART)
{
	return LL_USART_IsActiveFlag_TC(USART);
}

