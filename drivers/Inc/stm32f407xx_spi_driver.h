/*
 * 		stm32f407xx_spi_driver.h
 *
 *  Created on: 01-Mar-2021
 *      Author: Abhishek Roy
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * 	Configuration structure for SPIx Peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;							// Use macros defined at @SPI_DeviceMode
	uint8_t SPI_BusConfig;							// Use macros defined at @SPI_BusConfig
	uint8_t SPI_SclkSpeed;							// Use macros defined at @SPI_SclkSpeed
	uint8_t SPI_DFF;								// Use macros defined at @SPI_DFF
	uint8_t SPI_CPOL;								// Use macros defined at @SPI_CPOL
	uint8_t SPI_CPHA;								// Use macros defined at @SPI_CPHA
	uint8_t SPI_SSM;								// Use macros defined at @SPI_SSM

}SPI_Config_t;

/*
 * 	Handle structure for SPIx Peripheral
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;							// This holds the base address of SPIx(1,2..) peripheral
	SPI_Config_t SPIConfig;							// This hold configuration info about the SPIx peripheral
	uint8_t 	 *pTxBuffer;						// To store the application Tx buffer address
	uint8_t		 *pRxBuffer;						// To store applications Rx buffer address
	uint32_t	 TxLen;								// Tx length information
	uint32_t	 RxLen;								// Rx length information
	uint8_t 	 TxState;							// Stores the Tx state
	uint8_t  	 RxState;							// Stores the Rx state

}SPI_Handle_t;

/*
 * 	@SPI_DeviceMode
 */

#define SPI_MODE_SLAVE						0		// macro to configure MSTR as slave
#define SPI_MODE_MASTER 					1		// macro to configure MSTR as master

/*
 * 	@SPI_BusConfig
 */

#define SPI_BUS_FULL_DUPLEX					0		// macro to configure SPI as Full duplex
#define SPI_BUS_HALF_DUPLEX					1		// macro to configure SPI as Half duplex
#define SPI_BUS_SIMPLEX_RX_ONLY				2		// macro to configure SPI as Simplex Rx only

/*
 * 	@SPI_SclkSpeed
 */

#define SPI_SCLK_DIV2						0		// macro to configure baud rate as fpclk/2
#define SPI_SCLK_DIV4						1		// macro to configure baud rate as fpclk/4
#define SPI_SCLK_DIV8						2		// macro to configure baud rate as fpclk/8
#define SPI_SCLK_DIV16						3		// macro to configure baud rate as fpclk/16
#define SPI_SCLK_DIV32						4		// macro to configure baud rate as fpclk/32
#define SPI_SCLK_DIV64						5		// macro to configure baud rate as fpclk/64
#define SPI_SCLK_DIV128						6		// macro to configure baud rate as fpclk/128
#define SPI_SCLK_DIV256						7		// macro to configure baud rate as fpclk/256

/*
 * 	@SPI_DFF
 */

#define SPI_DFF_8_BITS						0		// macro to configure DFF as 8 bits
#define SPI_DFF_16_BITS						1		// macro to configure DFF as 16 bits

/*
 * 	@SPI_CPOL
 */

#define SPI_CPOL_IDLE_LOW					0		// macro to configure CPOL as low level (0) when clk is idle
#define SPI_CPOL_IDLE_HIGH					1		// macro to configure CPOL as high level (1) when clk is idle

/*
 * 	@SPI_CPHA
 */

#define SPI_CPHA_FIRST_EDGE					0		// macro to configure CPHA as first clock transition is the first data capture edge
#define SPI_CPHA_SECOND_EDGE				1		// macro to configure CPHA as second clock transition is the first data capture edge

/*
 * 	@SPI_SSM
 */

#define SPI_SSM_DISABLE						0		// Software slave management disabled
#define SPI_SSM_ENABLE						1		// Software slave management enabled

/*
 * 	@SPI_State
 */

#define SPI_READY							0		// SPI is ready to transmit or receive
#define SPI_BUSY_IN_RX						1		// SPI is busy receiving
#define SPI_BUSY_IN_TX						2		// SPI is busy transmitting

/*
 *  Possible SPI application events
 */

#define SPI_EVENT_TX_COMPLETE				0
#define SPI_EVENT_RX_COMPLETE				1
#define SPI_EVENT_OVR_ERR					2
#define SPI_EVENT_CRC_ERR					3


/**********************************************************************************************************************
 * 												APIs supported by this driver
 * 											For more information, check function definitions
 **********************************************************************************************************************/

/*
 *  Peripheral Clock control
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable);


/*
 *  Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * 	Check flag status in SPIx SR register
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint16_t FlagName);

/*
 *  Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  Data Send and Receive using interrupts
 */

uint8_t SPI_SendData_INTR(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveData_INTR(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 *	Other Peripheral control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable);
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable);
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * 	Application Event Callback (weak implementation, needs to be implemented by the application layer)
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
