/*
 * 		stm32f407xx_gpio_driver.h
 *
 *  Created on: 19-Feb-2021
 *      Author: Abhishek Roy
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * 	GPIOx configuration structure
 */

typedef struct
{
	uint8_t PinNumber;						/* One of the possible values from @GPIOPinNumbers */
	uint8_t PinMode;						/* One of the possible values from @GPIOModes */
	uint8_t PinSpeed;						/* One of the possible values from @GPIOSpeed */
	uint8_t PinPuPdControl;					/* One of the possible values from @GPIOPushPull */
	uint8_t PinOutputType;					/* One of the possible values from @GPIOOutputTypes */
	uint8_t PinAltFunMode;					/* One of the possible values from @GPIOAltFuncMode */

}GPIO_Pin_Config_t;

/*
 *  Defining a handle structure for GPIOx
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOBaseAddress;		/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_Pin_Config_t PinConfig;			/* This holds the GPIO pin configuration setting */

}GPIO_Handle_t;

/*
 *  @GPIOPinNumbers
 *  GPIO pin possible mode macros
 */

#define PIN_NUMBER_0			0
#define PIN_NUMBER_1			1
#define PIN_NUMBER_2			2
#define PIN_NUMBER_3			3
#define PIN_NUMBER_4			4
#define PIN_NUMBER_5			5
#define PIN_NUMBER_6			6
#define PIN_NUMBER_7			7
#define PIN_NUMBER_8			8
#define PIN_NUMBER_9			9
#define PIN_NUMBER_10			10
#define PIN_NUMBER_11			11
#define PIN_NUMBER_12			12
#define PIN_NUMBER_13			13
#define PIN_NUMBER_14			14
#define PIN_NUMBER_15			15

/*
 *  @GPIOModes
 *  GPIO pin possible mode macros
 */

#define MODE_INPUT				0
#define MODE_OUTPUT				1
#define MODE_ALT_FUN 			2
#define MODE_ANALOG				3
#define MODE_INTR_FT			4			/* Interrupt for falling edge trigger 			  */
#define MODE_INTR_RT			5			/* Interrupt for rising edge trigger  			  */
#define MODE_INTR_RFT			6			/* Interrupt for rising and falling edge trigger  */

/*
 *  @GPIOOutputType
 *  GPIO pin possible output type macros
 */

#define OPTYPE_PUSHPULL			0
#define OPTYPE_OD				1

/*
 *  @GPIOSpeed
 *  GPIO pin possible speed macros
 */

#define SPEED_LOW				0
#define SPEED_MEDIUM			1
#define SPEED_HIGH				2
#define SPEED_VERY_HIGH			3

/*
 *  @GPIOPushPull
 *  GPIO pin possible PuPd macros
 */

#define NO_PUPD					0
#define PULL_UP					1
#define PULL_DOWN				2

/*
 *  @EXTIPortSelection
 *  EXTI port selection macros
 */

#define EXTI_PA					0
#define EXTI_PB					1
#define EXTI_PC					2
#define EXTI_PD					3
#define EXTI_PE					4
#define EXTI_PF					5
#define EXTI_PG					6
#define EXTI_PH					7
#define EXTI_PI					8
#define EXTI_PJ					9
#define EXTI_PK					10

/*
 * 	@GPIOAltFuncMode
 * 	Alternate Function Mode macros
 */

#define ALT_FUNC_0				0
#define ALT_FUNC_1				1
#define ALT_FUNC_2				2
#define ALT_FUNC_3				3
#define ALT_FUNC_4				4
#define ALT_FUNC_5				5
#define ALT_FUNC_6				6
#define ALT_FUNC_7				7
#define ALT_FUNC_8				8
#define ALT_FUNC_9				9
#define ALT_FUNC_10				10
#define ALT_FUNC_11				11
#define ALT_FUNC_12				12
#define ALT_FUNC_13				13
#define ALT_FUNC_14				14
#define ALT_FUNC_15				15

/**********************************************************************************************************************
 * 												APIs supported by this driver										  *
 * 											For more information, check function definitions						  *
 **********************************************************************************************************************/

/*
 *  Peripheral Clock control
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable_or_Disable);

/*
 *  Init and De-Init control
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *  Data Read and Write
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void 	 GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void 	 GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void 	 GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *  IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * 	SPIx GPIO Init function
 */

void SPI_GPIO_Init(GPIO_RegDef_t* pGPIOx, uint8_t pin_number);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
