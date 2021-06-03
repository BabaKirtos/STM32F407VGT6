/*
 * 		stm32_f407xx_gpio_driver.c
 *
 * 		Created on: 19-Feb-2021
 *      Author: Abhishek Roy
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx_gpio_driver.h"


/*
 *  Peripheral Clock control
 */

/******************************************************************************************************
 * @fn			-	GPIO_PeriClockControl
 *
 * @brief		-	This function enables or disables clock for the given GPIO port
 *
 * @param[in]	-	GPIO base address
 *
 * @param[in]	-	Enable or Disable macros
 *
 * @return		-	none
 *
 * @note		-	none
 *
 ********************************************************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable_or_Disable)
{
	if(Enable_or_Disable)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN;
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN;
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN;
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN;
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN;
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN;
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN;
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN;
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN;
		else if(pGPIOx == GPIOJ)
			GPIOJ_PCLK_EN;
		else if(pGPIOx == GPIOK)
			GPIOK_PCLK_EN;
	}
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
		else if(pGPIOx == GPIOJ)
			GPIOJ_PCLK_DI();
		else if(pGPIOx == GPIOK)
			GPIOK_PCLK_DI();
	}
}

/*
 *  Init and De-Init control
 */

/******************************************************************************************************
 * @fn			-	GPIO_Init
 *
 * @brief		-	This function initializes the GPIO
 *
 * @param[in]	-	GPIO handle pointer
 *
 * @return		-	Null
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOBaseAddress, ENABLE);

	// 1. Configure the mode of GIPO pin
	if(pGPIOHandle->PinConfig.PinMode <= MODE_ANALOG)
	{
		temp = (pGPIOHandle->PinConfig.PinMode << (2 * pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOBaseAddress->MODER &= ~(3 << (2 * pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOBaseAddress->MODER |= temp;

	}
	else
	{
		// This will be coded later, for interrupt mode.

		// Making mode as input
		temp = MODE_INPUT << (2 * pGPIOHandle->PinConfig.PinNumber);
		pGPIOHandle->pGPIOBaseAddress->MODER &= ~(3 << (2 * pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOBaseAddress->MODER |= temp;

		if(pGPIOHandle->PinConfig.PinMode == MODE_INTR_FT)
		{
			// Configure Falling edge trigger
			EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);

		}
		else if(pGPIOHandle->PinConfig.PinMode == MODE_INTR_RT)
		{
			// Configure Rising edge trigger
			EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
		}
		else if(pGPIOHandle->PinConfig.PinMode == MODE_INTR_RFT)
		{
			// Configure both Falling and rising trigger
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
		}
		// Configure the GPIO port selection in SYSCGF_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t temp1 = (uint8_t)((pGPIOHandle->PinConfig.PinNumber) / 4);// To determine which EXTI register array to choose
		uint8_t temp2 = (4 * (uint8_t)((pGPIOHandle->PinConfig.PinNumber) % 4));// To determine which bit positions to set
		uint8_t portcode = GPIO_BASE_ADDRESS_TO_CODE (pGPIOHandle->pGPIOBaseAddress);

		SYSCFG->EXTICR[temp1] |= (portcode << temp2);


		// Enable EXTI interrupt delivery using IMR(Interrupt Mask Register in NVIC)
		EXTI->IMR |= (1 << pGPIOHandle->PinConfig.PinNumber);

	}

	temp = 0;

	// 2. Configure the speed of GPIO pin

	temp = pGPIOHandle->PinConfig.PinSpeed << (2 * pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddress->OSPEEDR &= ~(3 << (2 * pGPIOHandle->PinConfig.PinNumber));
	pGPIOHandle->pGPIOBaseAddress->OSPEEDR |= temp;

	temp = 0;

	// 3. Configure PuPd register

	temp = pGPIOHandle->PinConfig.PinPuPdControl << (2 * pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOBaseAddress->PUPDR &= ~(3 << (2 * pGPIOHandle->PinConfig.PinNumber));
	pGPIOHandle->pGPIOBaseAddress->PUPDR |= temp;

	temp = 0;

	// 4. Configure the output type
	if(pGPIOHandle->PinConfig.PinMode == MODE_OUTPUT)// to avoid bugs if output type is not declared when mode is input
	{
		temp = pGPIOHandle->PinConfig.PinOutputType << pGPIOHandle->PinConfig.PinNumber;
		pGPIOHandle->pGPIOBaseAddress->OTYPER &= ~(1 << (pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOBaseAddress->OTYPER |= temp;
	}
	else if(((pGPIOHandle->PinConfig.PinMode == MODE_INPUT) || (pGPIOHandle->PinConfig.PinMode > MODE_ANALOG )) && (pGPIOHandle->PinConfig.PinOutputType == OPTYPE_OD)) // setting output type as Pull up and pull down which is reset value (0)
	{
		temp = pGPIOHandle->PinConfig.PinOutputType << pGPIOHandle->PinConfig.PinNumber;
		pGPIOHandle->pGPIOBaseAddress->OTYPER &= ~(1 << (pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOBaseAddress->OTYPER |= temp;
	}
	else
		pGPIOHandle->pGPIOBaseAddress->OTYPER &= (OPTYPE_PUSHPULL << (pGPIOHandle->PinConfig.PinNumber));


	temp = 0;

	//5. Configure alternate functionality

	if(pGPIOHandle->PinConfig.PinMode == MODE_ALT_FUN) // Configure only when mode is alternate fun.
	{
		if(pGPIOHandle->PinConfig.PinNumber <= 7)
		{
			temp = pGPIOHandle->PinConfig.PinAltFunMode << (4 * pGPIOHandle->PinConfig.PinNumber);
			pGPIOHandle->pGPIOBaseAddress->AFRL &= ~(15 << (4 * pGPIOHandle->PinConfig.PinNumber));
			pGPIOHandle->pGPIOBaseAddress->AFRL |= temp;
		}
		else
		{
			uint8_t temp2 = (pGPIOHandle->PinConfig.PinNumber) - 8;
			temp = pGPIOHandle->PinConfig.PinAltFunMode << (4 * temp2);
			pGPIOHandle->pGPIOBaseAddress->OSPEEDR &= ~(15 << (4 * temp2));
			pGPIOHandle->pGPIOBaseAddress->AFRH |= temp;
		}
	}
}
/******************************************************************************************************
 * @fn			-	GPIO_De_Init
 *
 * @brief		-	This function resets all GPIOx peripherals
 *
 * @param[in]	-	Pointer to the GPIO reg def
 *
 * @return		-	NULL
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 0); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 0);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 2);
		//RCC->AHB1RSTR &= ~(1 << 2);
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}
	else if(pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET;
		//RCC->AHB1RSTR |= (1 << 1); // Setting bit 0 of AHB1RSTR
		//RCC->AHB1RSTR &= ~(1 << 1);	// Resetting bit 0 so that the port is not always in reset
	}

}

/*
 *  Data Read and Write
 */

/******************************************************************************************************
 * @fn			-	GPIO_ReadFromInputPin
 *
 * @brief		-	This function enables user to read from input pin of a gpio port
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @param[in]	-	Pin Number of that GPIO
 *
 * @return		-	Return the read value from the GPIO pin (0 or 1)
 *
 * @note		-
 *
 ********************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber & 0x00000001); // Right shifting the IDR to the bit 0 and masking other bits.
	return value;
}
/*********************************************************************************************************
* @fn			-	GPIO_ReadFromInputPort
 *
 * @brief		-	This function enables user to read from input of a gpio port
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @return		-	Return the read value from the GPIO port (0 or 1 of 16 pins)
 *
 * @note		-
 *
 ********************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}
/******************************************************************************************************
 * @fn			-	GPIO_WriteToOutputPin
 *
 * @brief		-	This function writes the given value to the GPIOx pin
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @param[in]	-	Pin Number
 *
 * @param[in]	-	Value to be written (0 o 1)
 *
 * @return		-	NULL
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}
/*********************************************************************************************************
  * @fn			-	GPIO_WriteToOutputPort
 *
 * @brief		-	This function writes the given value to the GPIOx port
 *
 * @param[in]	-	Pointer to GPIOx
 *
 * @param[in]	-	Value to be written (0 o 1)
 *
 * @return		-	NULL
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/*********************************************************************************************************
 * @fn			-	GPIO_ToggleOutputPin
 *
 * @brief		-	This function toggles the output pin of GPIOx
 *
 * @param[in]	-	Pointer to the GPIO
 *
 * @param[in]	-	Pin Number
 *
 * @return		-	Null
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 *  IRQ Configuration and ISR handling
 */

/******************************************************************************************************
 * @fn			-	GPIO_IRQInterruptConfig
 *
 * @brief		-	This function is used to configure the IRQ (Interrupt Request)
 *
 * @param[in]	-	IRQNumber (defined at @IRQNumber macro)
 *
 * @param[in]	-	Enable or Disable
 *
 * @return		-	Null
 *
 * @note		-
 *
 ********************************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable)
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
/***********************************************************************************************************
 * @fn			-	GPIO_IRQPriorityConfig
 *
 * @brief		-	This function is used to configure the IRQ Priority (Interrupt Request)
 *
 * @param[in]	-	IRQPriority (defined at  macro)
 *
 * @param[in]	-	IRQNumber (defined at @IRQNumber macro)
 *
 * @return		-	Null
 *
 * @note		-
 *
 ***********************************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out IPR register to touch
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (NUM_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDRESS + iprx) |= (IRQPriority << (shift_amount));
}

/*************************************************************************************************************
 * @fn			-	GPIO_IRQHandling
 *
 * @brief		-	Clears the PR flag of the EXTI register
 *
 * @param[in]	-	Pin Number
 *
 * @return		-	Null
 *
 * @note		-
 *
 **************************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear EXTI PR Register
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}

}
/*
 * 	SPIx GPIO Init Function
 */
/******************************************************************************************************
 * @fn			-	SPI_GPIO_Init
 *
 * @brief		-	This function initializes the SPIx GPIO pin to AltFunc Mode
 *
 * @param[in]	-	Pointer to spi handle
 *
 * @param[in]	-	Pointer to base address on which the spi is hanging
 *
 * @param[in]	-	Pin number
 *
 * @return		-	none
 *
 * @note		-	Only for Alternate Function 5 Enable
 *
 ********************************************************************************************************/
void SPI_GPIO_Init(GPIO_RegDef_t* pGPIOx, uint8_t pin_number)
{
	GPIO_Handle_t Gpio_spi_pin;

	// Configure the pin as AF5

	Gpio_spi_pin.pGPIOBaseAddress = pGPIOx;
	Gpio_spi_pin.PinConfig.PinNumber = pin_number;
	Gpio_spi_pin.PinConfig.PinMode = MODE_ALT_FUN;
	Gpio_spi_pin.PinConfig.PinAltFunMode = ALT_FUNC_5;
	Gpio_spi_pin.PinConfig.PinOutputType = OPTYPE_PUSHPULL;
	Gpio_spi_pin.PinConfig.PinPuPdControl = NO_PUPD;

	GPIO_Init(&Gpio_spi_pin);

}


