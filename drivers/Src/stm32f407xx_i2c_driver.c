/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 17-Apr-2021
 *      Author: Abhishek Roy
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

/*
 *  Peripheral Clock control
 */
/******************************************************************************************************
 * @fn			-	I2C_PeriClockControl
 *
 * @brief		-	This function enables or disables clock for the given I2Cx peripheral
 *
 * @param[in]	-	I2Cx base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}

/*
 *  Init and De-Init
 */

/******************************************************************************************************
 * @fn			-	I2C_DeInit
 *
 * @brief		-	This function resets all I2Cx peripherals
 *
 * @param[in]	-	I2Cx base address
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET;
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET;
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET;
	}
}

/*
 * 	Get Flag status function
 */
/******************************************************************************************************
 * @fn			-	I2C_GetFlagStatus
 *
 * @brief		-	This function gets the status of a flag from I2C sr register
 *
 * @param[in]	-	I2Cx base address
 *
 * @param[in]	-	Name of the flag
 *
 * @return		-	flag status
 *
 * @note		-	none
 *
 ********************************************************************************************************/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint16_t FlagName)
{
	// Need to check which SR to touch
	uint8_t status = ((pI2Cx->SR1) & (1 << FlagName));
	return status;
}

/*
 *  IRQ Configuration and ISR handling
 */
/******************************************************************************************************
 * @fn			-	I2C_IRQInterruptConfig
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

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable)
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
 * @fn			-	I2C_IRQPriorityConfig
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out IPR register to touch
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (NUM_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDRESS + iprx) |= (IRQPriority << (shift_amount));
}

/*
 * 	I2C peripheral control
 */
/******************************************************************************************************
 * @fn			-	I2C_PeripheralControl
 *
 * @brief		-	This function enables or disables I2Cx peripheral using CR1->SPE bit
 *
 * @param[in]	-	I2C base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}

/*
 * 	I2C app event callback
 */
/******************************************************************************************************
 * @fn			-	I2C_ApplicationEventCallback
 *
 * @brief		-	This function is a weak implementation, needs to be implemented by the application
 *
 * @param[in]	-	I2C handle function
 *
 * @param[in]	-	AppEvent
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{

}
