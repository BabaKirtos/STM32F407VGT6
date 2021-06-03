/*
 * 001LedToggle.c
 *
 *  Created on: 22-Feb-2021
 *      Author: Abhishek Roy
 */

#include <stdint.h>
#include "stm32f407xx.h"

void Delay(void);

int main(void)
{
	GPIO_Handle_t Gpio_Led;

	Gpio_Led.pGPIOBaseAddress = GPIOD;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_15;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OPTYPE_PP;
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&Gpio_Led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUMBER_15);
		Delay();
	}

	return 0;
}

void Delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}
