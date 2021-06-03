/*
 * 004InterruptButtonLED.c
 *
 *  Created on: 24-Feb-2021
 *      Author: Abhishek Roy
 */

#include <stdint.h>
#include <string.h>

#include "stm32f407xx.h"

void EXTI0_IRQHandler(void);
void delay(void);



int main(void)
{
	// Create 2 variable handles for LED and Button
	GPIO_Handle_t GpioLEDGreen, GpioButton;

	//Set all member elements of the structure as 0
//	memset(&GpioLEDGreen,0,sizeof(GpioLEDGreen));
//	memset(&GpioButton,0,sizeof(GpioButton));

	// Initialize Base addresses for the 2 variables
	GpioLEDGreen.pGPIOBaseAddress = GPIOD;
	GpioButton.pGPIOBaseAddress = GPIOA;

	// Setup Green LED in output push pull configuration
	GpioLEDGreen.PinConfig.PinNumber = PIN_NUMBER_12;
	GpioLEDGreen.PinConfig.PinMode = MODE_OUTPUT;
	GpioLEDGreen.PinConfig.PinOutputType = OPTYPE_PUSHPULL;
	GpioLEDGreen.PinConfig.PinPuPdControl = NO_PUPD;
	GpioLEDGreen.PinConfig.PinSpeed = SPEED_HIGH;

	// Setup Button in input configuration
	GpioButton.PinConfig.PinNumber = PIN_NUMBER_0;
	GpioButton.PinConfig.PinMode = MODE_INTR_RT;
	GpioButton.PinConfig.PinSpeed = SPEED_HIGH;
	GpioButton.PinConfig.PinPuPdControl = NO_PUPD;

	// Enable clocks for port A and port D
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	// Call Init function
	GPIO_Init(&GpioLEDGreen);
	GPIO_Init(&GpioButton);

	//IRQ Configuration for PA0
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI_0, IRQ_PRIORITY_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI_0, ENABLE);

	while(1);
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(PIN_NUMBER_0);
	GPIO_ToggleOutputPin(GPIOD, PIN_NUMBER_12);
}

void delay(void)
{
	for(uint32_t i=0; i < 250000; i++); //200ms delay for 16MHz frequency
}

