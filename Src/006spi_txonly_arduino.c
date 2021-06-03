/*
 * 		007spi_txonly_arduino.c
 *
 *  	Created on : 16-Mar-2021
 *     	    Author : Abhishek Roy
 */

/*
 * 	AF5 ALL:-
 * 	PB12 --> SPI2_NSS
 * 	PB13 --> SPI2_SCLK
 * 	PB14 --> SPI2_MISO
 * 	PB15 --> SPI2_MOSI
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

void GPIO_Button_Init(void);
void SPI2_Inits(void);
void EXTI0_IRQHandler(void);
void delay(void);

int main(void)
{

	//GPIO button Configuration
	GPIO_Button_Init();

	//SPI initialization
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_12);	//nss
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_13);	//sclk
	//SPI_GPIO_Init(&Gpio_spi2_miso, PIN_NUMBER_14);
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_15);	//mosi

	SPI2_Inits();

	//enable SSI bit
	//SPI_SSIControl(SPI2, ENABLE);

	//enable SSOE bit in CR2 reg
	SPI_SSOEControl(SPI2, ENABLE);

	//enable the SPE in CR1 -> Bit 6 SPE: SPI enable
	//SPI_PeripheralControl(SPI2, ENABLE); // NSS is pulled to low automatically when SPE is enabled(when SSOE is enabled in CR2)

	while(1);

}

void GPIO_Button_Init(void)
{
	// Create a variable to handle Button
	GPIO_Handle_t GpioButton;

	//Initialize the base address of button gpio
	GpioButton.pGPIOBaseAddress = GPIOA;

	// Setup Button in input configuration
	GpioButton.PinConfig.PinNumber = PIN_NUMBER_0;
	GpioButton.PinConfig.PinMode = MODE_INTR_RT;
	GpioButton.PinConfig.PinSpeed = SPEED_HIGH;
	GpioButton.PinConfig.PinPuPdControl = NO_PUPD;

	// Enable clocks for port A
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Call Init function
	GPIO_Init(&GpioButton);


	//IRQ Configuration for PA0
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI_0, IRQ_PRIORITY_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI_0, ENABLE);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV8;
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8_BITS;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_IDLE_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_DISABLE;

	SPI_Init(&SPI2_Handle);

}

void EXTI0_IRQHandler(void)
{
	delay();

	GPIO_IRQHandling(PIN_NUMBER_0);

	char data[] = "Hello World";

	SPI_PeripheralControl(SPI2, ENABLE);	// NSS is pulled to low when enabled (only in STM)(when SSOE is enabled in CR2 reg)

	//send number of bytes to receive information to arduino before sending data
	uint8_t dataLen = strlen(data);

	SPI_SendData(SPI2, &dataLen, 1);

	SPI_SendData(SPI2, (uint8_t *)data, strlen(data));

	while(!(SPI_GetFlagStatus(SPI2, SPI_SR_BSY)));

	SPI_PeripheralControl(SPI2, DISABLE);	// NSS is pulled to high when disabled
}

void delay(void)
{
	for(uint32_t i=0; i < 250000; i++); //200ms delay for 16MHz frequency
}
