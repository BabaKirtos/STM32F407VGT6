/*
 *		 007spi_cmd_handling.c
 *
 *  Created on: 19-Mar-2021
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
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

//Command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

#define LED_PIN					9

//arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

uint8_t ID[10];

void GPIO_Button_Init(void);
void SPI2_Inits(void);
//void EXTI0_IRQHandler(void);
uint8_t SPI_VerifyResponse(uint8_t AckByte);
void SPI_LED_control(uint8_t LED_on_or_off);
uint8_t SPI_Sensor_read(uint8_t PinNumber);
uint8_t SPI_LED_Read(void);
void SPI_Print(char * data);
uint8_t* SPI_ID_Read(void);
void delay(void);

int main(void)
{

	//GPIO button Configuration
	GPIO_Button_Init();

	//SPI initialization
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_12);	//nss
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_13);	//sclk
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_14);	//miso
	SPI_GPIO_Init(GPIOB, PIN_NUMBER_15);	//mosi

	SPI2_Inits();

	//enable SSI bit
	//SPI_SSIControl(SPI2, ENABLE);

	//enable SSOE bit in CR2 reg
	SPI_SSOEControl(SPI2, ENABLE);


	while(1)
	{
		char data[] = "Hi, Abhishek";

		while(! GPIO_ReadFromInputPin(GPIOA, PIN_NUMBER_0));

		delay();

		SPI_LED_control(LED_ON);

		while(! GPIO_ReadFromInputPin(GPIOA, PIN_NUMBER_0));

		delay();

		SPI_LED_control(LED_OFF);

		while(! GPIO_ReadFromInputPin(GPIOA, PIN_NUMBER_0));

		delay();

		uint8_t Analog_Value = SPI_Sensor_read(ANALOG_PIN0);

		printf("Analog Value is %d\n",Analog_Value);

		while(! GPIO_ReadFromInputPin(GPIOA, PIN_NUMBER_0));

		delay();

		uint8_t LED_Vaue = SPI_LED_Read();

		printf("LED Value is %d\n",LED_Vaue);

		while(! GPIO_ReadFromInputPin(GPIOA, PIN_NUMBER_0));

		delay();

		SPI_Print(data);

		while(! GPIO_ReadFromInputPin(GPIOA, PIN_NUMBER_0));

		delay();

		uint8_t *pDeviceID = (uint8_t *)SPI_ID_Read();

		printf("%s\n",pDeviceID);

		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

		SPI_PeripheralControl(SPI2, DISABLE);	// NSS is pulled to high when disabled

	}
}

void GPIO_Button_Init(void)
{
	// Create a variable to handle Button
	GPIO_Handle_t GpioButton;

	//Initialize the base address of button gpio
	GpioButton.pGPIOBaseAddress = GPIOA;

	// Setup Button in input configuration
	GpioButton.PinConfig.PinNumber = PIN_NUMBER_0;
	GpioButton.PinConfig.PinMode = MODE_INPUT;
	GpioButton.PinConfig.PinSpeed = SPEED_HIGH;
	GpioButton.PinConfig.PinPuPdControl = NO_PUPD;

	// Enable clocks for port A
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Call Init function
	GPIO_Init(&GpioButton);


//	//IRQ Configuration for PA0
//	GPIO_IRQPriorityConfig(IRQ_NO_EXTI_0, IRQ_PRIORITY_15);
//	GPIO_IRQInterruptConfig(IRQ_NO_EXTI_0, ENABLE);

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

//void EXTI0_IRQHandler(void)
//{
//	delay();
//
//	GPIO_IRQHandling(PIN_NUMBER_0);
//}

uint8_t SPI_VerifyResponse(uint8_t AckByte)
{
	if(AckByte == 0xF5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void SPI_LED_control(uint8_t LED_on_or_off)
{
	SPI_PeripheralControl(SPI2, ENABLE);	// NSS is pulled to low when enabled (only in STM)(when SSOE is enabled in CR2 reg)

	uint8_t commandCode = COMMAND_LED_CTRL;
	uint8_t AckByte = 0;
	uint8_t Args[2];
	uint8_t dummyData = 0xFF;
	uint8_t dummyRead = 0;

	SPI_SendData(SPI2, &commandCode, 1); // this will also fill up the receive buffer so rxne would be set

	//do a dummy read to clear rxne flag
	SPI_ReceiveData(SPI2, &dummyRead, 1);

	//to receive the ACK or NACK message from slave we need to send it some dummy data to initiate the communication

	SPI_SendData(SPI2, &dummyData, 1); //  the buffer must have response from slave after this api returns

	SPI_ReceiveData(SPI2, &AckByte, 1);

	//check if ACK or NACK was received

	if(SPI_VerifyResponse(AckByte))
	{
		//send arguments
		Args[0] = LED_PIN;
		Args[1] = LED_on_or_off;

		SPI_SendData(SPI2, Args, 2);
	}
}

uint8_t SPI_Sensor_read(uint8_t PinNumber)
{
	SPI_PeripheralControl(SPI2, ENABLE);	// NSS is pulled to low when enabled (only in STM)(when SSOE is enabled in CR2 reg)

	uint8_t commandCode = COMMAND_SENSOR_READ;
	uint8_t AckByte = 0;
	uint8_t Args = PinNumber;
	uint8_t dummyData = 0xFF;
	uint8_t dummyRead = 0;
	uint8_t SensorRead = 10;

	SPI_SendData(SPI2, &commandCode, 1); // this will also fill up the receive buffer so rxne would be set

	//do a dummy read to clear rxne flag
	SPI_ReceiveData(SPI2, &dummyRead, 1);

	//to receive the ACK or NACK message from slave we need to send it some dummy data to initiate the communication

	SPI_SendData(SPI2, &dummyData, 1); //  the buffer must have response from slave after this api returns

	SPI_ReceiveData(SPI2, &AckByte, 1);

	//check if ACK or NACK was received

	if(SPI_VerifyResponse(AckByte))
	{
		//send arguments
		SPI_SendData(SPI2, &Args, 1);

		SPI_ReceiveData(SPI2, &dummyRead, 1);

		delay();

		SPI_SendData(SPI2, &dummyData, 1);

		SPI_ReceiveData(SPI2, &SensorRead, 1);
	}
	return SensorRead;
}

uint8_t SPI_LED_Read(void)
{
	SPI_PeripheralControl(SPI2, ENABLE);	// NSS is pulled to low when enabled (only in STM)(when SSOE is enabled in CR2 reg)

	uint8_t commandCode = COMMAND_LED_READ;
	uint8_t AckByte = 0;
	uint8_t Args = LED_PIN;
	uint8_t dummyData = 0xFF;
	uint8_t dummyRead = 0;
	uint8_t LED_value = 10;

	SPI_SendData(SPI2, &commandCode, 1); // this will also fill up the receive buffer so rxne would be set

	//do a dummy read to clear rxne flag
	SPI_ReceiveData(SPI2, &dummyRead, 1);

	//to receive the ACK or NACK message from slave we need to send it some dummy data to initiate the communication

	SPI_SendData(SPI2, &dummyData, 1); //  the buffer must have response from slave after this api returns

	SPI_ReceiveData(SPI2, &AckByte, 1);

	//check if ACK or NACK was received

	if(SPI_VerifyResponse(AckByte))
	{
		//send arguments
		SPI_SendData(SPI2, &Args, 1);

		SPI_ReceiveData(SPI2, &dummyRead, 1);

		delay();

		SPI_SendData(SPI2, &dummyData, 1);

		SPI_ReceiveData(SPI2, &LED_value, 1);
	}
	return LED_value;
}

void SPI_Print(char * data)
{
	SPI_PeripheralControl(SPI2, ENABLE);	// NSS is pulled to low when enabled (only in STM)(when SSOE is enabled in CR2 reg)

	uint8_t commandCode = COMMAND_PRINT;
	uint8_t AckByte = 0;
	uint8_t dataLen = strlen(data);
	uint8_t dummyData = 0xFF;
	uint8_t dummyRead = 0;

	SPI_SendData(SPI2, &commandCode, 1); // this will also fill up the receive buffer so rxne would be set

	//do a dummy read to clear rxne flag
	SPI_ReceiveData(SPI2, &dummyRead, 1);

	//to receive the ACK or NACK message from slave we need to send it some dummy data to initiate the communication

	SPI_SendData(SPI2, &dummyData, 1); //  the buffer must have response from slave after this api returns

	SPI_ReceiveData(SPI2, &AckByte, 1);

	//check if ACK or NACK was received

	if(SPI_VerifyResponse(AckByte))
	{
		//send arguments
		SPI_SendData(SPI2, &dataLen, 1);

		SPI_SendData(SPI2, (uint8_t *)data, strlen(data));
	}
}

uint8_t* SPI_ID_Read(void)
{
	SPI_PeripheralControl(SPI2, ENABLE);	// NSS is pulled to low when enabled (only in STM)(when SSOE is enabled in CR2 reg)

	uint8_t commandCode = COMMAND_PRINT;
	uint8_t AckByte = 0;
	uint8_t dummyData = 0xFF;
	uint8_t dummyRead = 0;

	SPI_SendData(SPI2, &commandCode, 1); // this will also fill up the receive buffer so rxne would be set

	//do a dummy read to clear rxne flag
	SPI_ReceiveData(SPI2, &dummyRead, 1);

	//to receive the ACK or NACK message from slave we need to send it some dummy data to initiate the communication

	SPI_SendData(SPI2, &dummyData, 1); //  the buffer must have response from slave after this api returns

	SPI_ReceiveData(SPI2, &AckByte, 1);

	//check if ACK or NACK was received

	if(SPI_VerifyResponse(AckByte))
	{
		//activate clock to slave by writing dummy data
		SPI_SendData(SPI2, &dummyData, 1);

		//read 10 bytes of data from arduino
		for(uint8_t i = 0; i < 10; i++)
		{
			SPI_ReceiveData(SPI2, &ID[i], 1);
		}
		ID[11] = '\0';
	}
	return ID;
}


void delay(void)
{
	for(uint32_t i=0; i < 500000; i++); //200ms delay for 16MHz frequency
}
