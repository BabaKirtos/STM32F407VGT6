/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 17-Apr-2021
 *      Author: Abhishek Roy
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * 	I2Cx configuration structure
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;						/* One of the possible values from @I2C_SCLSpeed */
	uint8_t I2C_DeviceAddress;					/* User need to input the slave Device Address */
	uint8_t I2C_ACKControl;						/* One of the possible values from @I2C_ACKControl */
	uint8_t I2C_FMDutyCycle;					/* One of the possible values from @I2C_FMDutyCycle */

}I2C_Config_t;

/*
 * 	I2Cx Handle structure
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;						/* Pointer to RegDef of I2C */
	I2C_Config_t I2C_Config;					/* I2C configuration structure variable */

}I2C_Handle_t;


/*
 * 	@I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM			100000		/* Standard mode with clock speed as 100KHz */
#define I2C_SCL_SPEED_FM4K			400000		/* Fast mode with clock speed as 400KHz */
#define I2C_SCL_SPEED_FM2K			200000		/* Fast mode with clock speed as 200KHz (>100 KHz = FM) */

/*
 * 	@I2C_ACKControl
 */

#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0			/* Default it is disabled */

/*
 * 	@I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_CYCLE_2			0
#define I2C_FM_DUTY_CYCLE_16_9		1

/**********************************************************************************************************************
 * 												APIs supported by this driver										  *
 * 											For more information, check function definitions						  *
 **********************************************************************************************************************/

/*
 *  Peripheral Clock control
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_or_Disable);

/*
 *  Init and De-Init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * 	Check flag status in I2Cx SR register
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint16_t FlagName);

/*
 *  Data Send and Receive
 */


/*
 *  Data Send and Receive using interrupts
 */


/*
 *  IRQ Configuration and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable_or_Disable);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 *	Other Peripheral control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_or_Disable);


/*
 * 	Application Event Callback (weak implementation, needs to be implemented by the application layer)
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);



#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
