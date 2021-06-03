/*
 *		stm32f407xx_spi_driver.c
 *
 *  Created on: 01-Mar-2021
 *      Author: Abhishek Roy
 */


#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"


static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle);


/*
 *  Peripheral Clock control
 */
/******************************************************************************************************
 * @fn			-	SPI_PeriClockControl
 *
 * @brief		-	This function enables or disables clock for the given SPIx peripheral
 *
 * @param[in]	-	SPIx base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if(pSPIx == SPI4)
			SPI4_PCLK_EN();
		else if(pSPIx == SPI5)
			SPI5_PCLK_EN();
		else if(pSPIx == SPI6)
			SPI6_PCLK_EN();
	}
	else
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if(pSPIx == SPI4)
			SPI4_PCLK_DI();
		else if(pSPIx == SPI5)
			SPI5_PCLK_DI();
		else if(pSPIx == SPI6)
			SPI6_PCLK_DI();
	}
}

/*
 *  Init and De-Init
 */
/******************************************************************************************************
 * @fn			-	SPI_Init
 *
 * @brief		-	This function initializes the SPIx
 *
 * @param[in]	-	SPIx base address
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable peripheral clock to SPIx
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// first configure CR1 register of SPIx

	uint32_t temp = 0;

	// 1. Configure the device mode

	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2. Configure the Bus type

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FULL_DUPLEX)
	{
		// BIDI Mode should be in 2 line ie reset (0)
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp &= ~(1 << SPI_CR1_RXONLY);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HALF_DUPLEX)
	{
		// BIDI Mode should be in 1 line ie set (1)
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX_ONLY)
	{
		//BIDI Mode should be in 2 line and RXONLY should be set for receive only (simplex)
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SCLK of SPIx

	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. Configure the DFF of SPIx

	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 5. Configure the CPHA of SPIx

	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 6. Configure the CPOL of SPIx

	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 7. Configure the SSM bit

	temp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	// 8. Assign the CR1 register of SPIx with temp

	pSPIHandle->pSPIx->CR1 = 0;

	pSPIHandle->pSPIx->CR1 = temp;

}

/******************************************************************************************************
 * @fn			-	SPI_DeInit
 *
 * @brief		-	This function resets all SPIx peripherals
 *
 * @param[in]	-	SPIx base address
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET;
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET;
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET;
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET;
	}
	else if(pSPIx == SPI5)
	{
		SPI5_REG_RESET;
	}
	else if(pSPIx == SPI6)
	{
		SPI6_REG_RESET;
	}
}

/*
 * 	Get Flag status function
 */
/******************************************************************************************************
 * @fn			-	SPI_GetFlagStatus
 *
 * @brief		-	This function gets the status of a flag from spi sr register
 *
 * @param[in]	-	SPIx base address
 *
 * @param[in]	-	Name of the flag
 *
 * @return		-	flag status
 *
 * @note		-	none
 *
 ********************************************************************************************************/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint16_t FlagName)
{
	uint8_t status = ((pSPIx->SR) & (1 << FlagName));
	return status;
}

/*
 *  Data Send and Receive
 */
/******************************************************************************************************
 * @fn			-	SPI_SendData
 *
 * @brief		-	This function sends data to SPIx peripheral
 *
 * @param[in]	-	SPIx base address
 *
 * @param[in]	-	Pointer to transfer buffer data
 *
 * @param[in]	-	Length of buffer in bytes
 *
 * @return		-	none
 *
 * @note		-	This is a blocking call (polling)
 *
 ********************************************************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

	while(Len > 0)
	{

		// Wait till TXE is set.
		while(!SPI_GetFlagStatus(pSPIx, SPI_SR_TXE));

		if((pSPIx->CR1) & (1 << SPI_CR1_DFF))
		{
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			(uint16_t *)pTxBuffer++;
			Len-=2;
		}
		else
		{
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
	}
}

/******************************************************************************************************
 * @fn			-	SPI_ReceiveData
 *
 * @brief		-	This function sends data to SPIx peripheral
 *
 * @param[in]	-	SPIx Handle
 *
 * @param[in]	-	Pointer to receive buffer data
 *
 * @param[in]	-	Length of buffer in bytes
 *
 * @return		-	none
 *
 * @note		-	This is a blocking call (polling)
 *
 ********************************************************************************************************/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

	while(Len > 0)
	{

		// Wait till RXNE is reset.
		while(!SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE));

		if((pSPIx->CR1) & (1 << SPI_CR1_DFF))
		{
			*((uint16_t *)pRxBuffer) = (uint16_t)pSPIx->DR;
			(uint16_t *)pRxBuffer++;
			Len-=2;
		}
		else
		{
			*pRxBuffer = (uint8_t)pSPIx->DR;
			pRxBuffer++;
			Len--;
		}
	}
}

/*
 *  Data Send and Receive using Interrupts
 */
/******************************************************************************************************
 * @fn			-	SPI_SendData_INTR
 *
 * @brief		-	This function sends data to SPIx peripheral based on interrupts generated
 *
 * @param[in]	-	SPIx Handle
 *
 * @param[in]	-	Pointer to transfer buffer data
 *
 * @param[in]	-	Length of buffer in bytes
 *
 * @return		-	returns the state
 *
 * @note		-	This is a non - blocking call
 *
 ********************************************************************************************************/

uint8_t SPI_SendData_INTR(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX )
	{
		// Save the Tx buffer address and length information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// Mark the SPI state as busy in Transmission so that no other peripheral can take hold of the SPI until
		// transmission is completed
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable the TXEIE bit in CR2 to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// Data transmission will be handled by the ISR code
	}

	return state;
}

/******************************************************************************************************
 * @fn			-	SPI_ReceiveData_INTR
 *
 * @brief		-	This function receives data from SPIx peripheral based on interrupts generated
 *
 * @param[in]	-	SPIx base address
 *
 * @param[in]	-	Pointer to receive buffer data
 *
 * @param[in]	-	Length of buffer in bytes
 *
 * @return		-	returns the state
 *
 * @note		-	This is a non - blocking call
 *
 ********************************************************************************************************/

uint8_t SPI_ReceiveData_INTR(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX )
	{
		// Save the Rx buffer address and length information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// Mark the SPI state as busy in Transmission so that no other peripheral can take hold of the SPI until
		// transmission is completed
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable the RXNEIE bit in CR2 to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data transmission will be handled by the ISR code
	}

	return state;
}

/*
 *  IRQ Configuration and ISR handling
 */
/******************************************************************************************************
 * @fn			-	SPI_IRQInterruptConfig
 *
 * @brief		-	This function configures the interrupt on processor side
 *
 * @param[in]	-	IRQ number
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/******************************************************************************************************
 * @fn			-	SPI_IRQPriorityConfig
 *
 * @brief		-	This function sets the IRQ priority for the interrupt
 *
 * @param[in]	-	IRQNumber
 *
 * @param[in]	-	IRQ Priority
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out IPR register to touch
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (NUM_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDRESS + iprx) |= (IRQPriority << (shift_amount));
}

/*************************************************************************************************************
 * @fn			-	SPI_IRQHandling
 *
 * @brief		-	Handles ISR request from the SPI interrupt
 *
 * @param[in]	-	Pin Number
 *
 * @return		-	Null
 *
 * @note		-
 *
 **************************************************************************************************************/

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1 = (pSPIHandle->pSPIx->SR) & (1 << SPI_SR_TXE);		//check TXE flag status in SR
	uint8_t temp2 = (pSPIHandle->pSPIx->CR2) & (1 << SPI_CR2_TXEIE);	//check TXEIE flag status in CR2


	if(temp1 && temp2)
	{
		spi_txe_interrupt_handler(pSPIHandle);
	}

	temp1 = (pSPIHandle->pSPIx->SR) & (1 << SPI_SR_RXNE);		//check RXNE flag status in SR
	temp2 = (pSPIHandle->pSPIx->CR2) & (1 << SPI_CR2_RXNEIE);	//check RXNEIE flag status in CR2

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handler(pSPIHandle);
	}

	temp1 = (pSPIHandle->pSPIx->SR) & (1 << SPI_SR_OVR);		//check OVR flag status in SR
	temp2 = (pSPIHandle->pSPIx->CR2) & (1 << SPI_CR2_ERRIE);	//check ERRIE flag status in CR2

	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handler(pSPIHandle);
	}

}

/******************************************************************************************************
 * @fn			-	SPI_PeripheralControl
 *
 * @brief		-	This function enables or disables SPIx peripheral using CR1->SPE bit
 *
 * @param[in]	-	SPI base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

/******************************************************************************************************
 * @fn			-	SPI_SSIControl
 *
 * @brief		-	This function enables or disables SSI bit using CR1->SSI bit
 *
 * @param[in]	-	SPI base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	Only valid when CR1->SSM is enabled
 *
 ********************************************************************************************************/

void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

/******************************************************************************************************
 * @fn			-	SPI_SSOEControl
 *
 * @brief		-	This function enables or disables SSOE bit using CR2->SSOE bit
 *
 * @param[in]	-	SPI base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	Only valid when CR1->SSM is disabled
 *
 ********************************************************************************************************/

void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= (uint32_t)~(1 << SPI_CR2_SSOE);
}

/******************************************************************************************************
 * @fn			-	SPI_ClearOVRFlag
 *
 * @brief		-	This function clears the overrun error flag from SR
 *
 * @param[in]	-	SPI base address
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	//1. Clear overrun flag
	uint8_t temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/******************************************************************************************************
 * @fn			-	SPI_CloseTransmission
 *
 * @brief		-	This function closes the transmission when length becomes 0 and informs app
 *
 * @param[in]	-	SPI handle function
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//Tx Len <= 0, inform application and close the transmission
	//Mask TXEIE bit from CR2, this prevents further interrupts for Tx
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

/******************************************************************************************************
 * @fn			-	SPI_CloseReception
 *
 * @brief		-	This function closes the reception when length becomes 0 and informs app
 *
 * @param[in]	-	SPI handle function
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//Rx Len <= 0, inform application and close the transmission
	//Mask RXNEIE bit from CR2, this prevents further interrupts for Rx
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

/******************************************************************************************************
 * @fn			-	SPI_ApplicationEventCallback
 *
 * @brief		-	This function is a weak implementation, needs to be implemented by the application
 *
 * @param[in]	-	SPI handle function
 *
 * @param[in]	-	AppEvent
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{

}

/**********************************************************************************************************

 									Private helper function implementations

 **********************************************************************************************************/

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
	{
		(pSPIHandle->pSPIx->DR) = *((uint16_t *)pSPIHandle->pTxBuffer);
		(uint16_t *)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen -= 2;
	}
	else
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}
	if (pSPIHandle->TxLen <= 0)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_COMPLETE);
	}
}
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
	{
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		(uint16_t *)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen-=2;
	}
	else
	{
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}
	if (pSPIHandle->RxLen <= 0)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_COMPLETE);
	}

}
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle)
{

	//Clear flag only when the SPI is not busy
	//Also clearing the flag by reading the data in DR is not right, the application might require that data
	//So we can just inform the application of the overrun error and provide additional APIs to deal with it

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	//Inform application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


