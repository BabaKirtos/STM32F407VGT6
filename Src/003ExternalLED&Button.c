/*
 * 003ExternalLED&Button.c
 *
 *  Created on: 23-Feb-2021
 *      Author: Abhishek Roy
 */

#include <stdint.h>
#include "stm32f407xx.h"

void delay(void);

int main(void)
{
	GPIO_Handle_t GpioLED, GpioButton;

	// Configuring base addresses to the variables
	GpioLED.pGPIOBaseAddress = GPIOA;
	GpioButton.pGPIOBaseAddress = GPIOB;

	// Configuring external Button at PB12 with internal Pull Up
	GpioButton.PinConfig.PinNumber = PIN_NUMBER_12;
	GpioButton.PinConfig.PinMode = MODE_INPUT;
	GpioButton.PinConfig.PinPuPdControl = PULL_UP;
	GpioButton.PinConfig.PinSpeed = SPEED_HIGH;

	// Configuring external LED at PA14 with PUSH PULL
	GpioLED.PinConfig.PinNumber = PIN_NUMBER_8;
	GpioLED.PinConfig.PinMode = MODE_OUTPUT;
	GpioLED.PinConfig.PinOutputType = OPTYPE_PUPD;
	GpioLED.PinConfig.PinSpeed = SPEED_HIGH;
	GpioLED.PinConfig.PinPuPdControl = NO_PUPD;

	// Turning ON the clocks for port A and B
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);

	// Calling Init function
	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioButton);

	while(1)
	{
		if(!(GPIO_ReadFromInputPin(GPIOB, PIN_NUMBER_12)))
		{
			delay();
			GPIO_WriteToOutputPin(GPIOA, PIN_NUMBER_8, SET);
		}
		else
		{
			delay();
			GPIO_WriteToOutputPin(GPIOA, PIN_NUMBER_8, RESET);
		}
	}
}

void delay(void)
{
	for(uint32_t i = 0; i < 50000; i++);
}
