/**
 * @description IO support library for stm32f4 and nrf8001.  Defines
 	middleware functions which forward Arduino SDK GPIO, SPI, and time
 	delay to equivalent STM32F4 routines.
 */

#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal_uart.h>
#include <millis.h>
#include <IO_Support.h>
#include <spi.h>
#include <stm32f4xx_ll_gpio.h>
void __ble_assert(const char *file, uint16_t line)
{
    log_info("ERROR: %s, line %d\r\n", file, line);

	/*
  	  Serial.print("ERROR ");
  	  Serial.print(file);
  	  Serial.print(": ");
  	  Serial.print(line);
  	  Serial.print("\n");
  	  while(1);
	*/
}

void delay(uint16_t delay)
{
    uint32_t old_time = millis();

    while ((millis() - old_time) < delay) {}
}

uint8_t digitalRead(uint8_t pin)
{
    uint8_t value = 0;

    if (pin == RDYN_PIN) {
        if (LL_GPIO_ReadInputPort(RDYN_GPIO_PORT) & pin)
        {
            value = 1;
        }
        else
        {
            value = 0;
        }
 	  }

    //log_info("digitalRead(%d) = %d\r\n", pin, value);
    return value;
}

uint8_t digitalWrite(uint8_t pin, uint8_t value)
{
 	  // only do for REQN pin
 	  if ((pin == REQN_PIN) || (pin == RESET_PIN)) {
 	  	  if (value)
 	  	  {
 	  		LL_GPIO_SetOutputPin(REQN_GPIO_PORT, pin);
 	  	  }
 	  	  else
 	  	  {
 	  		LL_GPIO_SetOutputPin(REQN_GPIO_PORT, pin);
 	  	  }
 	  }

    //log_info("digitalWrite(%d) = %d\r\n", pin, value);
 	  return 0;
}

void pinMode(uint8_t pin, uint8_t mode)
{
 	  GPIO_InitTypeDef gpio_init;

 	  gpio_init.Pin = pin;
      gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

 	  switch (mode) {
 	      case OUTPUT:
 	          gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
 	          gpio_init.Pull = GPIO_PULLUP;
 	      	  break;

 	       case INPUT:
 	       case INPUT_PULLUP:
    		  gpio_init.Mode = GPIO_MODE_INPUT;
    		  gpio_init.Pull = GPIO_PULLUP;
 	          break;

 	  	   default:
 	  	       break;
 	  }

 	  if (pin == RDYN_PIN) {
 		 HAL_GPIO_Init(RDYN_GPIO_PORT, &gpio_init);
 	  } else if (pin == REQN_PIN) {
 		 HAL_GPIO_Init(REQN_GPIO_PORT, &gpio_init);
 	  }

}

/*TODO : See if deatachIntterupt is needed- rdyn pin not set as interrupt */
void detachInterrupt(uint8_t	interrupt_number)
{

	switch(interrupt_number)
	{
	case 1:
		break;
	default:
		break;
	}
}
	/*TODO : See if attachInterrupt is needed- rdyn pin not set as interrupt */
void	attachInterrupt(uint8_t	interrupt_number, void *fp_m_aci_isr, uint8_t pinStatus)
{

}




