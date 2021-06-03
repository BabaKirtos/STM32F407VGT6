/*
 * 		stm32f407xx.h
 *
 * 		Created on: Feb 18, 2021
 *      Author: Abhishek Roy
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>
#include <stddef.h>


/********************************** START: Processor Specific Details ***********************************/
 /*
 *	ARM Cortex Mx Processor NVIC ISERx register Addresses (Interrupt Set Enable Register)
 */

#define NVIC_ISER0 					((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1 					((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2 					((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3 					((volatile uint32_t *)0xE000E10C)

/*
 *	ARM Cortex Mx Processor NVIC ICERx register Addresses (Interrupt Clear Enable Register)
 */

#define NVIC_ICER0 					((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1 					((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2 					((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3 					((volatile uint32_t *)0xE000E18C)

/*
 *  ARM Cortex Mx Processor NVIC IPRx register Addresses (Interrupt Priority Register)
 */

#define NVIC_IPR_BASE_ADDRESS		((volatile uint32_t *)0xE000E400)

/*
 *  ARM Cortex Mx Processor NVIC IPRx register bits implemented
 */

#define NUM_PR_BITS_IMPLEMENTED		(4)

/*********************************************** END *******************************************************/

/*
 * 	Base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDRESS			(0x08000000UL)
#define SRAM1_BASE_ADDRESS			(0x20000000UL)								//112KB
#define SRAM2_BASE_ADDRESS			(0x2001C000UL)								//16KB
#define ROM							(0x1FFF0000UL)
#define OTP_BASE_ADDRESS			(0x1FFF7800UL)
#define SRAM 						(SRAM1_BASE_ADDRESS)

/*
 * AHBx and APBx peripheral base addresses
 */

#define PERIPHERAL_BASE_ADDRESS		(0x40000000UL)
#define APB1_BASE_ADDRESS			(PERIPHERAL_BASE_ADDRESS)
#define APB2_BASE_ADDRESS			((PERIPHERAL_BASE_ADDRESS) + 0x10000)
#define AHB1_BASE_ADDRESS			((PERIPHERAL_BASE_ADDRESS) + 0x20000)
#define AHB2_BASE_ADDRESS			((PERIPHERAL_BASE_ADDRESS) + 0x10000000)
#define AHB3_BASE_ADDRESS			((PERIPHERAL_BASE_ADDRESS) + 0x60000000) 	// FSMC control register

/*
 *  Base address of all peripherals on APB1 bus
 */

#define TIM2_BASE_ADDRESS			(APB1_BASE_ADDRESS)
#define TIM3_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x400)
#define TIM4_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x800)
#define TIM5_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0xC00)
#define TIM6_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x1000)
#define TIM7_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x1400)
#define TIM12_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x1800)
#define TIM13_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x1C00)
#define TIM14_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x2000)
#define RTC_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x2800)
#define WWDG_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x2C00)
#define IWDG_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x3000)
#define I2S2EXT_BASE_ADDRESS		((APB1_BASE_ADDRESS) + 0x3400)
#define SPI2_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x3800)
#define SPI3_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x3C00)
#define I2S3EXT_BASE_ADDRESS		((APB1_BASE_ADDRESS) + 0x4000)
#define USART2_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x4400)
#define USART3_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x4800)
#define UART4_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x4C00)
#define UART5_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x5000)
#define I2C1_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x5400)
#define I2C2_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x5800)
#define I2C3_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x5C00)
#define CAN1_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x6400)
#define CAN2_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x6800)
#define PWR_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x7000)
#define DAC_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x7400)
#define UART7_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x7800)
#define UART8_BASE_ADDRESS			((APB1_BASE_ADDRESS) + 0x7C00)

/*
 *  Base address of all peripherals on APB2 bus
 */

#define TIM1_BASE_ADDRESS			(APB2_BASE_ADDRESS)
#define TIM8_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x400)
#define USART1_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x1000)
#define USART6_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x1400)
#define ADC_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x2000)
#define SDIO_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x2C00)
#define SPI1_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x3000)
#define SPI4_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x3400)
#define SYSCFG_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x3800)
#define EXTI_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x3C00)
#define TIM9_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x4000)
#define TIM10_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x4400)
#define TIM11_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x4800)
#define SPI5_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x5000)
#define SPI6_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x5400)
#define SAI1_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x5800)
#define LCD_BASE_ADDRESS			((APB2_BASE_ADDRESS) + 0x6800)

/*
 *  Base address of all peripherals on AHB1 bus
 */

#define GPIOA_BASE_ADDRESS			(AHB1_BASE_ADDRESS)
#define GPIOB_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x400)
#define GPIOC_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x800)
#define GPIOD_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0xC00)
#define GPIOE_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x1000)
#define GPIOF_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x1400)
#define GPIOG_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x1800)
#define GPIOH_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x1C00)
#define GPIOI_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x2000)
#define GPIOJ_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x2400)
#define GPIOK_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x2800)
#define CRC_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x3000)
#define RCC_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x3800)
#define FLASH_INTER_BASE_ADDRESS	((AHB1_BASE_ADDRESS) + 0x3C00)
#define BKPSRAM_BASE_ADDRESS		((AHB1_BASE_ADDRESS) + 0x4000)
#define DMA1_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x6000)
#define DMA2_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0x6400)
#define ETHERNET_BASE_ADDRESS		((AHB1_BASE_ADDRESS) + 0x8000)
#define DMA2D_BASE_ADDRESS			((AHB1_BASE_ADDRESS) + 0xB000)
#define USB_OTG_HS_BASE_ADDRESS		((AHB1_BASE_ADDRESS) + 0x20000)

/*
 *  Base address of all peripherals on AHB2 bus
 */

#define USB_OTG_FS_BASE_ADDRESS		(AHB2_BASE_ADDRESS)
#define DCMI_BASE_ADDRESS			((AHB2_BASE_ADDRESS) + 0x50000)
#define CRYP_BASE_ADDRESS			((AHB2_BASE_ADDRESS) + 0x60000)
#define HASH_BASE_ADDRESS			((AHB2_BASE_ADDRESS) + 0x60400)
#define RNG_BASE_ADDRESS			((AHB2_BASE_ADDRESS) + 0x60800)

/*
 *  Base address of AHB3 -> FSMC Control register
 */

#define FSMC_CR_BASE_ADDRESS		(AHB3_BASE_ADDRESS)


/************************************* Register Definition Structures **********************************************/

/*
 *  GPIO Definition
 */

typedef struct
{
	uint32_t volatile MODER;		/* GPIO port mode register,										Address offset: 0x00*/
	uint32_t volatile OTYPER;		/* GPIO port output type register,								Address offset: 0x04*/
	uint32_t volatile OSPEEDR;		/* GPIO port output speed register,								Address offset: 0x08*/
	uint32_t volatile PUPDR;		/* GPIO port pull-up/pull-down register,						Address offset: 0x0C*/
	uint32_t volatile IDR;			/* GPIO port input data register,								Address offset: 0x10*/
	uint32_t volatile ODR;			/* GPIO port output data register,								Address offset: 0x14*/
	uint32_t volatile BSRR;			/* GPIO port bit set/reset register,							Address offset: 0x18*/
	uint32_t volatile LCKR;			/* GPIO port configuration lock register,						Address offset: 0x1C*/
	uint32_t volatile AFRL;			/* GPIO alternate function low register,						Address offset: 0x20*/
	uint32_t volatile AFRH;			/* GPIO alternate function high register,						Address offset: 0x24*/
}GPIO_RegDef_t;


/*
 *  RCC Definition
 */

typedef struct
{
	uint32_t volatile CR;			/* RCC clock control register,									Address offset: 0x00*/
	uint32_t volatile PLLCFGR;		/* RCC PLL configuration register,								Address offset: 0x04*/
	uint32_t volatile CFGR;			/* RCC clock configuration register,							Address offset: 0x08*/
	uint32_t volatile CIR;			/* RCC clock interrupt register,								Address offset: 0x0C*/
	uint32_t volatile AHB1RSTR;		/* RCC AHB1 peripheral reset register,							Address offset: 0x10*/
	uint32_t volatile AHB2RSTR;		/* RCC AHB2 peripheral reset register,							Address offset: 0x14*/
	uint32_t volatile AHB3RSTR;		/* RCC AHB3 peripheral reset register,							Address offset: 0x18*/
	uint32_t RESERVED1;				/* RESERVED,													Address offset: 0x1C*/
	uint32_t volatile APB1RSTR;		/* RCC APB1 peripheral reset register,							Address offset: 0x20*/
	uint32_t volatile APB2RSTR;		/* RCC APB2 peripheral reset register,							Address offset: 0x24*/
	uint32_t RESERVED2;				/* RESERVED,													Address offset: 0x28*/
	uint32_t RESERVED3;				/* RESERVED,													Address offset: 0x2C*/
	uint32_t volatile AHB1ENR;		/* RCC AHB1 peripheral clock register,							Address offset: 0x30*/
	uint32_t volatile AHB2ENR;		/* RCC AHB2 peripheral clock enable register,					Address offset: 0x34*/
	uint32_t volatile AHB3ENR;		/* RCC AHB3 peripheral clock enable register,					Address offset: 0x38*/
	uint32_t RESERVED4;				/* RESERVED,													Address offset: 0x3C*/
	uint32_t volatile APB1ENR;		/* RCC APB1 peripheral clock enable register,					Address offset: 0x40*/
	uint32_t volatile APB2ENR;		/* RCC APB2 peripheral clock enable register,					Address offset: 0x44*/
	uint32_t RESERVED5;				/* RESERVED,													Address offset: 0x48*/
	uint32_t RESERVED6;				/* RESERVED,													Address offset: 0x4C*/
	uint32_t volatile AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode,			Address offset: 0x50*/
	uint32_t volatile AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode,			Address offset: 0x54*/
	uint32_t volatile AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode,			Address offset: 0x58*/
	uint32_t RESERVED7;				/* RESERVED,													Address offset: 0x5C*/
	uint32_t volatile APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode,			Address offset: 0x60*/
	uint32_t volatile APB2LPENR;	/* RCC APB1 peripheral clock enable in low power mode,			Address offset: 0x64*/
	uint32_t RESERVED8;				/* RESERVED,													Address offset: 0x68*/
	uint32_t RESERVED9;				/* RESERVED,													Address offset: 0x6C*/
	uint32_t volatile BDCR;			/* RCC Backup domain control register,							Address offset: 0x70*/
	uint32_t volatile CSR;			/* RCC clock control & status register,							Address offset: 0x74*/
	uint32_t RESERVED10;			/* RESERVED,													Address offset: 0x78*/
	uint32_t RESERVED11;			/* RESERVED,													Address offset: 0x7C*/
	uint32_t volatile SSCGR;		/* RCC spread spectrum clock generation register,				Address offset: 0x80*/
	uint32_t volatile PLLI2SCSGR;	/* RCC PLLI2S configuration register,							Address offset: 0x84*/
	uint32_t volatile PLLSAICFGR;	/* RCC PLL configuration register,								Address offset: 0x88*/
	uint32_t volatile DCKCFGR;		/* RCC Dedicated Clock Configuration Register,					Address offset: 0x8C*/
}RCC_RegDef_t;

/*
 *  EXTI Definition
 */

typedef struct
{
	uint32_t volatile IMR;			/* Interrupt mask register,										Address offset: 0x00*/
	uint32_t volatile EMR;			/* Event mask register,											Address offset: 0x04*/
	uint32_t volatile RTSR;			/* Rising trigger selection register,							Address offset: 0x08*/
	uint32_t volatile FTSR;			/* Falling trigger selection register,							Address offset: 0x0C*/
	uint32_t volatile SWIER;		/* Software interrupt event register,							Address offset: 0x10*/
	uint32_t volatile PR;			/* Pending register,											Address offset: 0x14*/
}EXTI_RegDef_t;

/*
 * 	SPIx Definition
 */

typedef struct
{
	uint32_t CR1;					/* SPI control register 1 (SPI_CR1) (not used in I2S mode)			Address offset: 0x00*/
	uint32_t CR2;					/* SPI control register 2 (SPI_CR2)									Address offset: 0x04*/
	uint32_t SR;					/* SPI status register (SPI_SR)										Address offset: 0x08*/
	uint32_t DR;					/* SPI data register (SPI_DR)										Address offset: 0x0C*/
	uint32_t CRCPR;					/* SPI CRC polynomial register (SPI_CRCPR) (not used in I2S mode)	Address offset: 0x10*/
	uint32_t RXCRCR;				/* SPI RX CRC register (SPI_RXCRCR) (not used in I2S mode)			Address offset: 0x14*/
	uint32_t TXCRCR;				/* SPI TX CRC register (SPI_TXCRCR) (not used in I2S mode)			Address offset: 0x18*/
	uint32_t I2SCFGR;				/* SPI_I2S configuration register (SPI_I2SCFGR)						Address offset: 0x1C*/
	uint32_t I2SPR;					/* SPI_I2S pre-scaler register (SPI_I2SPR)							Address offset: 0x20*/

}SPI_RegDef_t;

/*
 * 	I2Cx Definition
 */

typedef struct
{
	uint32_t CR1;					/* I2C Control register 1 (I2C_CR1)									Address offset: 0x00*/
	uint32_t CR2;					/* I2C Control register 2 (I2C_CR2)									Address offset: 0x04*/
	uint32_t OAR1;					/* I2C Own address register 1 (I2C_OAR1)							Address offset: 0x08*/
	uint32_t OAR2;					/* I2C Own address register 1 (I2C_OAR1)							Address offset: 0x0C*/
	uint32_t DR;					/* I2C Data register (I2C_DR)										Address offset: 0x10*/
	uint32_t SR1;					/* I2C Status register 1 (I2C_SR1)									Address offset: 0x14*/
	uint32_t SR2;					/* I2C Status register 2 (I2C_SR2)									Address offset: 0x18*/
	uint32_t CCR;					/* I2C Clock control register (I2C_CCR)								Address offset: 0x1C*/
	uint32_t TRISE;					/* I2C TRISE register (I2C_TRISE)									Address offset: 0x20*/
	uint32_t FLTR;					/* I2C FLTR register (I2C_FLTR)(NOTE - Not in F407)					Address offset: 0x24*/

}I2C_RegDef_t;

/*
 *  SYSCFG Definition
 */

typedef struct
{
	uint32_t volatile MEMRMP;		/* SYSCFG memory re-map register,									Address offset: 0x00*/
	uint32_t volatile PMC;			/* SYSCFG peripheral mode configuration register,					Address offset: 0x04*/
	uint32_t volatile EXTICR[4];	/* SYSCFG external interrupt configuration register 1,				Address offset: 0x08- 0x14*/
	uint32_t RESERVED;				/* Reserved,														Address offset: 0x1C*/
	uint32_t volatile CMPCR;		/* Compensation cell control register,								Address offset: 0x20*/

}SYSCFG_RegDef_t;

/********************* Peripheral Definition Macros (peripheral base address type casted to X_RegDef_t) ************************/

/*
 * 	GPIO Macros
 */

#define GPIOA						((GPIO_RegDef_t *) GPIOA_BASE_ADDRESS)
#define GPIOB						((GPIO_RegDef_t *) GPIOB_BASE_ADDRESS)
#define GPIOC						((GPIO_RegDef_t *) GPIOC_BASE_ADDRESS)
#define GPIOD						((GPIO_RegDef_t *) GPIOD_BASE_ADDRESS)
#define GPIOE						((GPIO_RegDef_t *) GPIOE_BASE_ADDRESS)
#define GPIOF						((GPIO_RegDef_t *) GPIOF_BASE_ADDRESS)
#define GPIOG						((GPIO_RegDef_t *) GPIOG_BASE_ADDRESS)
#define GPIOH						((GPIO_RegDef_t *) GPIOH_BASE_ADDRESS)
#define GPIOI						((GPIO_RegDef_t *) GPIOI_BASE_ADDRESS)
#define GPIOJ						((GPIO_RegDef_t *) GPIOJ_BASE_ADDRESS)
#define GPIOK						((GPIO_RegDef_t *) GPIOK_BASE_ADDRESS)

/*
 * 	SPI Macros
 */

#define SPI1						((SPI_RegDef_t *) SPI1_BASE_ADDRESS)
#define SPI2						((SPI_RegDef_t *) SPI2_BASE_ADDRESS)
#define SPI3						((SPI_RegDef_t *) SPI3_BASE_ADDRESS)
#define SPI4						((SPI_RegDef_t *) SPI4_BASE_ADDRESS)
#define SPI5						((SPI_RegDef_t *) SPI5_BASE_ADDRESS)
#define SPI6						((SPI_RegDef_t *) SPI6_BASE_ADDRESS)

/*
 * 	I2C Macros
 */

#define I2C1						((I2C_RegDef_t *) I2C1_BASE_ADDRESS)
#define I2C2						((I2C_RegDef_t *) I2C2_BASE_ADDRESS)
#define I2C3						((I2C_RegDef_t *) I2C3_BASE_ADDRESS)


/*
 * 	RCC Macro
 */

#define RCC							((RCC_RegDef_t *) RCC_BASE_ADDRESS)

/*
 * ETXI Macro
 */

#define EXTI						((EXTI_RegDef_t *) EXTI_BASE_ADDRESS)

/*
 * 	SYSCFG Macro
 */

#define SYSCFG						((SYSCFG_RegDef_t *) SYSCFG_BASE_ADDRESS)

/******************************************************************************************************************************/

/*
 *  Clock Enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN				(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN				(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN				(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN				(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN				(RCC -> AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN				(RCC -> AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN				(RCC -> AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN				(RCC -> AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN				(RCC -> AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN				(RCC -> AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN				(RCC -> AHB1ENR |= (1 << 10))

/*
 *  Clock Enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()				(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC -> APB1ENR |= (1 << 23))

/*
 *  Clock Enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()				(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC -> APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()				(RCC -> APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()				(RCC -> APB2ENR |= (1 << 21))

/*
 *  Clock Enable macros for USARTx/UARTx peripherals
 */

#define USART1_PCLK_EN()			(RCC -> APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC -> APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC -> APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC -> APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC -> APB2ENR |= (1 << 5))

/*
 *  Clock Enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()			(RCC -> APB2ENR |= (1 << 14))

/*
 *  Clock Disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()				(RCC -> AHB1ENR &= ~(1 << 10))

/*
 *  Clock Disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 23))

/*
 *  Clock Disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()				(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC -> APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()				(RCC -> APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()				(RCC -> APB2ENR &= ~(1 << 21))

/*
 *  Clock Disable macros for USARTx/UARTx peripherals
 */

#define USART1_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC -> APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 5))

/*
 *  Clock Disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 14))

/*
 *  GPIOx reset macros for GPIOx peripherals
 *  (using do while condition zero technique, which can enables us to execute multiple C statements in a single macro)
 */

#define GPIOA_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)
#define GPIOJ_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9));}while(0)
#define GPIOK_REG_RESET				do{(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10));}while(0)

/*
 *  GPIO Base address to code for port code implementation, takes base address as argument
 */

#define GPIO_BASE_ADDRESS_TO_CODE(x)			( (x == GPIOA)? 0:\
												  (x == GPIOB)? 1:\
												  (x == GPIOC)? 2:\
												  (x == GPIOD)? 3:\
												  (x == GPIOE)? 4:\
												  (x == GPIOF)? 5:\
												  (x == GPIOG)? 6:\
												  (x == GPIOH)? 7:\
												  (x == GPIOI)? 8:\
												  (x == GPIOJ)? 9:\
												  (x == GPIOK)?10:0  )

/*
 *  SPIx reset macros for SPIx peripherals
 *  (using do while condition zero technique, which can enables us to execute multiple C statements in a single macro)
 */

#define SPI1_REG_RESET				do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET				do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET				do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET				do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)
#define SPI5_REG_RESET				do{(RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20));}while(0)
#define SPI6_REG_RESET				do{(RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21));}while(0)

/*
 *  I2Cx reset macros for I2Cx peripherals
 *  (using do while condition zero technique, which can enables us to execute multiple C statements in a single macro)
 */

#define I2C1_REG_RESET				do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET				do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET				do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)

/*
 *  IRQ Number definition macros
 *  @IRQNumber
 */

#define IRQ_NO_WWDG					0		/* Window Watchdog interrupt,											Address: 0x00000040 */
#define IRQ_NO_PVD					1		/* PVD through EXTI line detection interrupt,							Address: 0x00000044 */
#define IRQ_NO_TAMP_STAMP			2		/* Tamper and TimeStamp interrupts through the EXTI line,				Address: 0x00000048 */
#define IRQ_NO_RTC_WKUP				3		/* RTC Wakeup interrupt through the EXTI line,							Address: 0x0000004C */
#define IRQ_NO_FLASH				4		/* Flash global interrupt,												Address: 0x00000050 */
#define IRQ_NO_RCC					5		/* RCC global interrupt,												Address: 0x00000054 */
#define IRQ_NO_EXTI_0				6		/* EXTI Line0 interrupt,												Address: 0x00000058 */
#define IRQ_NO_EXTI_1				7		/* EXTI Line1 interrupt,												Address: 0x0000005C */
#define IRQ_NO_EXTI_2				8		/* EXTI Line2 interrupt,												Address: 0x00000060 */
#define IRQ_NO_EXTI_3				9		/* EXTI Line3 interrupt,												Address: 0x00000064 */
#define IRQ_NO_EXTI_4				10		/* EXTI Line4 interrupt,												Address: 0x00000068 */
#define IRQ_NO_DMA1_STREAM0			11		/* DMA1 Stream0 global interrupt,										Address: 0x0000006C */
#define IRQ_NO_DMA1_STREAM1			12		/* DMA1 Stream1 global interrupt,										Address: 0x00000070 */
#define IRQ_NO_DMA1_STREAM2			13		/* DMA1 Stream2 global interrupt,										Address: 0x00000074 */
#define IRQ_NO_DMA1_STREAM3			14		/* DMA1 Stream3 global interrupt,										Address: 0x00000078 */
#define IRQ_NO_DMA1_STREAM4			15		/* DMA1 Stream4 global interrupt,										Address: 0x0000007C */
#define IRQ_NO_DMA1_STREAM5			16		/* DMA1 Stream5 global interrupt,										Address: 0x00000080 */
#define IRQ_NO_DMA1_STREAM6			17		/* DMA1 Stream6 global interrupt,										Address: 0x00000084 */
#define IRQ_NO_ADC					18		/* ADC1, ADC2 and ADC3 global interrupts,								Address: 0x00000088 */
#define IRQ_NO_CAN1_TX				19		/* CAN1 TX interrupts,													Address: 0x0000008C */
#define IRQ_NO_CAN1_RX0				20		/* CAN1 RX0 interrupts,													Address: 0x00000090 */
#define IRQ_NO_CAN1_RX1				21		/* CAN1 RX1 interrupt,													Address: 0x00000094 */
#define IRQ_NO_CAN1_SCE				22		/* CAN1 SCE interrupt,													Address: 0x00000098 */
#define IRQ_NO_EXTI_5_9				23		/* EXTI Line[9:5] interrupts,											Address: 0x0000009C */
#define IRQ_NO_TIM1_BRK_TIM9		24		/* TIM1 Break interrupt and TIM9 global interrupt,						Address: 0x000000A0 */
#define IRQ_NO_TIM1_UP_TIM10		25		/* TIM1 Update interrupt and TIM10 global interrupt,					Address: 0x000000A4 */
#define IRQ_NO_TIM1_TRG_COM_TIM11	26		/* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt,	Address: 0x000000A8 */
#define IRQ_NO_TIM1_CC				27		/* TIM1 Capture Compare interrupt,										Address: 0x000000AC */
#define IRQ_NO_TIM2					28		/* TIM2 global interrupt,												Address: 0x000000B0 */
#define IRQ_NO_TIM3					29		/* TIM3 global interrupt,												Address: 0x000000B4 */
#define IRQ_NO_TIM4					30		/* TIM4 global interrupt,												Address: 0x000000B8 */
#define IRQ_NO_I2C1_EV				31		/* I2C1 event interrupt,												Address: 0x000000BC */
#define IRQ_NO_I2C1_ER				32		/* I2C1 error interrupt,												Address: 0x000000C0 */
#define IRQ_NO_I2C2_EV				33		/* I2C2 event interrupt,												Address: 0x000000C4 */
#define IRQ_NO_I2C2_ER				34		/* I2C2 error interrupt,												Address: 0x000000C8 */
#define IRQ_NO_SPI1					35		/* SPI1 global interrupt,												Address: 0x000000CC */
#define IRQ_NO_SPI2					36		/* SPI2 global interrupt,												Address: 0x000000D0 */
#define IRQ_NO_USART1				37		/* USART1 global interrupt,												Address: 0x000000D4 */
#define IRQ_NO_USART2				38		/* USART2 global interrupt,												Address: 0x000000D8 */
#define IRQ_NO_USART3				39		/* USART3 global interrupt,												Address: 0x000000DC */
#define IRQ_NO_EXTI_10_15			40		/* EXTI Line[15:10] interrupts,											Address: 0x000000E0 */
#define IRQ_NO_RTC_ALARM			41		/* RTC Alarms (A and B) through EXTI line interrupt,					Address: 0x000000E4 */
#define IRQ_NO_OTG_FS_WKUP			42		/* USB On-The-Go FS Wake up through EXTI line interrupt,				Address: 0x000000E8 */
#define IRQ_NO_TIM8_BRK_TIM12		43		/* TIM8 Break interrupt and TIM12 global interrupt,						Address: 0x000000EC */
#define IRQ_NO_TIM8_UP_TIM13		44		/* TIM8 Update interrupt and TIM13 global interrupt,					Address: 0x000000F0 */
#define IRQ_NO_TIM8_TRG_COM_TIM14	45		/* TIM8 Trigger and Commutation interrupts and TIM14 global interrupt,	Address: 0x000000F4 */
#define IRQ_NO_TIM8_CC				46		/* TIM8 Capture Compare interrupt,										Address: 0x000000F8 */
#define IRQ_NO_DMA1_STREAM7			47		/* DMA1 Stream7 global interrupt,										Address: 0x000000FC */
#define IRQ_NO_FSMC					48		/* FSMC global interrupt,												Address: 0x00000100 */
#define IRQ_NO_SDIO					49		/* SDIO global interrupt,												Address: 0x00000104 */
#define IRQ_NO_TIM5					50		/* TIM5 global interrupt,												Address: 0x00000108 */
#define IRQ_NO_SPI3					51		/* SPI3 global interrupt,												Address: 0x0000010C */
#define IRQ_NO_UART4				52		/* UART4 global interrupt,												Address: 0x00000110 */
#define IRQ_NO_UART5				53		/* UART5 global interrupt,												Address: 0x00000114 */
#define IRQ_NO_TIM6_DAC				54		/* TIM6 global interrupt, DAC1 and DAC2 under run error interrupts,		Address: 0x00000118 */
#define IRQ_NO_TIM7					55		/* TIM7 global interrupt,												Address: 0x0000011C */
#define IRQ_DMA2_STREAM0			56		/* DMA2 Stream0 global interrupt,										Address: 0x00000120 */
#define IRQ_DMA2_STREAM1			57		/* DMA2 Stream1 global interrupt,										Address: 0x00000124 */
#define IRQ_DMA2_STREAM2			58		/* DMA2 Stream2 global interrupt,										Address: 0x00000128 */
#define IRQ_DMA2_STREAM3			59		/* DMA2 Stream3 global interrupt,										Address: 0x0000012C */
#define IRQ_DMA2_STREAM4			60		/* DMA2 Stream4 global interrupt,										Address: 0x00000130 */
#define IRQ_NO_ETH					61		/* Ethernet global interrupt,											Address: 0x00000134 */
#define IRQ_NO_ETH_WKUP				62		/* Ethernet Wake up through EXTI line interrupt,						Address: 0x00000138 */
#define IRQ_NO_CAN2_TX				63		/* CAN2 TX interrupts,													Address: 0x0000013C */
#define IRQ_NO_CAN2_RX0				64		/* CAN2 RX0 interrupts,													Address: 0x00000140 */
#define IRQ_NO_CAN2_RX1				65		/* CAN2 RX1 interrupt,													Address: 0x00000144 */
#define IRQ_NO_CAN2_SCE				66		/* CAN2 SCE interrupt,													Address: 0x00000148 */
#define IRQ_NO_OTG_FS				67		/* USB On The Go FS global interrupt,									Address: 0x0000014C */
#define IRQ_NO_DMA2_STREAM5			68		/* DMA2 Stream5 global interrupt,										Address: 0x00000150 */
#define IRQ_NO_DMA2_STREAM6			69		/* DMA2 Stream6 global interrupt,										Address: 0x00000154 */
#define IRQ_NO_DMA2_STREAM7			70		/* DMA2 Stream7 global interrupt,										Address: 0x00000158 */
#define IRQ_NO_USART6				71		/* USART6 global interrupt,												Address: 0x0000015C */
#define IRQ_NO_I2C3_EV				72		/* I2C3 event interrupt,												Address: 0x00000160 */
#define IRQ_NO_I2C3_ER				73		/* I2C3 error interrupt,												Address: 0x00000160 */
#define IRQ_NO_OTG_HS_EP1_OUT		74		/* USB On The Go HS End Point 1 Out global interrupt,					Address: 0x00000160 */
#define IRQ_NO_OTG_HS_EP1_IN		75		/* USB On The Go HS End Point 1 In global interrupt,					Address: 0x00000160 */
#define IRQ_NO_OTG_HS_WKUP			76		/* USB On The Go HS Wake up through EXTI interrupt,						Address: 0x00000170 */
#define IRQ_NO_OTG_HS				77		/* USB On The Go HS global interrupt,									Address: 0x00000174 */
#define IRQ_NO_DCMI					78		/* DCMI global interrupt,												Address: 0x00000178 */
#define IRQ_NO_CRYP					79		/* CRYP crypto global interrupt,										Address: 0x0000017C */
#define IRQ_NO_HASH_RNG				80		/* Hash and Rng global interrupt,										Address: 0x00000180 */
#define IRQ_NO_FPU					81		/* FPU global interrupt,												Address: 0x00000184 */

/*
 *  IRQ Priority number macros
 *  @IRQPriority
 */

#define IRQ_PRIORITY_0				0
#define IRQ_PRIORITY_1				1
#define IRQ_PRIORITY_2				2
#define IRQ_PRIORITY_3				3
#define IRQ_PRIORITY_4				4
#define IRQ_PRIORITY_5				5
#define IRQ_PRIORITY_6				6
#define IRQ_PRIORITY_7				7
#define IRQ_PRIORITY_8				8
#define IRQ_PRIORITY_9				9
#define IRQ_PRIORITY_10				10
#define IRQ_PRIORITY_11				11
#define IRQ_PRIORITY_12				12
#define IRQ_PRIORITY_13				13
#define IRQ_PRIORITY_14				14
#define IRQ_PRIORITY_15				15

/* Some generic Macros */

#define ENABLE						1
#define DISABLE 					0
#define SET							ENABLE
#define RESET						DISABLE
#define PIN_SET						SET
#define PIN_RESET					RESET

/***********************************************************************************************************************
 * 										SPI Register Bit field definition Macros									   *
 ***********************************************************************************************************************/

/*
 * Bit definition macro for SPI_CR1
 */

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

/*
 * Bit definition macro for SPI_CR2
 */

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

/*
 * Bit definition macro for SPI_SR
 */

#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/***********************************************************************************************************************
 * 										I2C Register Bit field definition Macros									   *
 ***********************************************************************************************************************/

/*
 * 	Bit definition for CR1
 */

#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

/*
 * 	Bit definition for CR2
 */

#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

/*
 * 	Bit definition for OAR1
 */

#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD1				1
#define I2C_OAR1_ADD8				8
#define I2C_OAR1_ADDMODE			15

/*
 * 	Bit definition for OAR2
 */

#define I2C_OAR2_ENDUAL				0
#define I2C_OAR2_ADD1				1

/*
 * 	Bit definition for SR1
 */

#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RxNE				6
#define I2C_SR1_TxE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

/*
 * 	Bit definition for SR2
 */

#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

/*
 * 	Bit definition for CCR
 */

#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15



/*********************************************************************************************************************************/


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F407XX_H_ */



