/*
 * 002LedButton.c
 *
 *  Created on: 22-Feb-2021
 *      Author: Abhishek Roy
 */

#include <stdint.h>
#include "stm32f407xx.h"

void delay(void);

int main (void)
{
	// Create 2 variable handles for LED and Button
	GPIO_Handle_t GpioLED, GpioButton;

	// Initialize Base addresses for the 2 variables
	GpioLED.pGPIOBaseAddress = GPIOD;
	GpioButton.pGPIOBaseAddress = GPIOA;

	// Setup LED in output push pull configuration
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLED.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OPTYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// Setup Button in input configuration
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Enable clocks for port A and port D
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	// Call Init function
	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioButton);

	while(1)
	{
		//Set condition for LED on when button is pressed
		if((GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NUMBER_0))==1)
		{
			delay();
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUMBER_12, GPIO_PIN_SET);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUMBER_12, GPIO_PIN_RESET);
		}
	}
}

void delay(void)
{
	for(uint32_t i=0; i < 50000; i++);
}

