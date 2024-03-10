/**
 * @description Debug support library for stm32f4 boards
 */

#include <stm32f4xx.h>
#include <stm32f4xx_hal_uart.h>
#include <debug.h>
#include <usart.h>
//#define __PRINTF_OVER_UART__

extern UART_HandleTypeDef huart2;

 #ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetTCFlagStatus(USART2) == RESET)
  {}

  return ch;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
#ifndef __PRINTF_OVER_UART__
int _write(int file, char *ptr, int len) {
    int n;

    for (n = 0; n < len; n++) {
        __io_putchar(*ptr++);
    }

    return len;
}
#endif
#if defined(__PRINTF_OVER_UART__)
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO


int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   // arbitrary timeout 1000
   HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}
#endif
