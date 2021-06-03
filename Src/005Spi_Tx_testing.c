/*
 * 005Spi_Tx_testing.c
 *
 *  Created on: 03-Mar-2021
 *      Author: Abhishek Roy
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

void SPI2_Inits(void);

int main(void)
{
	char data[] = "Hello World";

	//SPI_GPIO_Init(&Gpio_spi2_nss, PIN_NUMBER_12);
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_13);
	//SPI_GPIO_Init(&Gpio_spi2_miso, PIN_NUMBER_14);
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_15);

	SPI2_Inits();

	//enable SSI bit
	SPI_SSIControl(SPI2, ENABLE);

	//enable the SPE in CR1 -> Bit 6 SPE: SPI enable
	SPI_PeripheralControl(SPI2, ENABLE);

	while(1)
		SPI_SendData(SPI2, (uint8_t *)data, strlen(data));

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV2;
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8_BITS;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_IDLE_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLE;

	SPI_Init(&SPI2_Handle);

}


