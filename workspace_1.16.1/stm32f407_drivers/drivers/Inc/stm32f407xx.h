/*
 * stm32f407xx.h
 *
 *  Created on: Jun 7, 2025
 *      Author: root
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

// Flash aka Main memory
#define FLASH_BASEADDR 				0x08000000U

#define SRAM1_BASEADDR 				0x20000000U // 112Kbs
#define SRAM2_BASEADDR 				0x2001C000U
// ROM aka System Memory
#define ROM							0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR



#define PERI_BASE					0x40000000U
#define APB1_PERI_BASE              PERI_BASE
#define APB2_PERI_BASE				0x40010000U
#define AHB1_PERI_BASE              0x40020000U
#define AHB2_PERI_BASE              0x50000000U


/* Base addresses of peripherals hanging on AHB1 */

//#define USB_OTG_BASEADDR			0x4004 0000U // - 0x4007 FFFF USB OTG HS
//#define DMAD1_BASEADDR				0x4002 B000U // - 0x4002 BBFF DMA2D
//#define ETHERNET_MAS_BASEADDR		0x4002 8000U // - 0x4002 93FF ETHERNET MAC
//#define DMA2_BASEADDR				0x4002 6400U // - 0x4002 67FF DMA2
//#define DMA1_BASEADDR				0x4002 6000U // - 0x4002 63FF DMA1
//#define BKPSRAM_BASEADDR			0x4002 4000U // - 0x4002 4FFF BKPSRAM -
//#define FLASH_IFACE_BASEADDR		0x4002 3C00U // - 0x4002 3FFF Flash interface
#define RCC_BASEADDR 			(AHB1_PERI_BASE + 0x3800U) // - 0x4002 3BFF RCC
//#define CRC_BASEADDR			0x4002 3000U // - 0x4002 33FF CRC
#define GPIOK_BASEADDR			(AHB1_PERI_BASE + 0x2800U) //  - 0x4002 2BFF GPIOK
#define GPIOJ_BASEADDR 			(AHB1_PERI_BASE + 0x2400U) // - 0x4002 27FF GPIOJ
#define GPIOI_BASEADDR 			(AHB1_PERI_BASE + 0x2000U) // - 0x4002 23FF GPIOI
#define GPIOH_BASEADDR 			(AHB1_PERI_BASE + 0x1C00U) // - 0x4002 1FFF GPIOH
#define GPIOG_BASEADDR 			(AHB1_PERI_BASE + 0x1800U) // - 0x4002 1BFF GPIOG
#define GPIOF_BASEADDR 			(AHB1_PERI_BASE + 0x1400U) // - 0x4002 17FF GPIOF
#define GPIOE_BASEADDR 			(AHB1_PERI_BASE + 0x1000U) // - 0x4002 13FF GPIOE
#define GPIOD_BASEADDR 			(AHB1_PERI_BASE + 0x0C00U) // - 0x4002 0FFF GPIOD
#define GPIOC_BASEADDR 			(AHB1_PERI_BASE + 0x0800U) // - 0x4002 0BFF GPIOC
#define GPIOB_BASEADDR 			(AHB1_PERI_BASE + 0x0400U) // - 0x4002 07FF GPIOB
#define GPIOA_BASEADDR 			(AHB1_PERI_BASE + 0x0000U) // - 0x4002 03FF GPIOA


/* APB1 Peripherals */

#define I2C1_BASEADDR			(APB1_PERI_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1_PERI_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1_PERI_BASE + 0x5C00)

#define SPI2_BASEADDR			(APB1_PERI_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1_PERI_BASE + 0x3C00)

#define USART2_BASEADDR			(APB1_PERI_BASE + 0x4400)
#define USART3_BASEADDR			(APB1_PERI_BASE + 0x4800)
#define UART4_BASEADDR			(APB1_PERI_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1_PERI_BASE + 0x5000)

/* APB2 Peripherals */
#define EXTI_BASEADDR			(APB2_PERI_BASE + 0x3C00)
#define SPI1_BASEADDR			(APB2_PERI_BASE + 0x3000)
#define SYSCFG_BASEADDR			(APB2_PERI_BASE + 0x3800)
#define USART1_BASEADDR			(APB2_PERI_BASE + 0x1000)
#define USART6_BASEADDR			(APB2_PERI_BASE + 0x1400)

/* SPI1 Registers */
#define SPI1_CR1_ADDR			(SPI1_BASEADDR + 0x00) // Control Register 1
#define SPI1_CR2_ADDR			(SPI1_BASEADDR + 0x04) // Control Register 2
#define SPI1_SR_ADDR			(SPI1_BASEADDR + 0x08) // Status Register
#define SPI1_DR_ADDR			(SPI1_BASEADDR + 0x0C) // Data Register
#define SPI1_CRCPR_ADDR			(SPI1_BASEADDR + 0x10) // CRC Polynomial Register
#define SPI1_RXCRCR_ADDR		(SPI1_BASEADDR + 0x14) // RX CRC Register
#define SPI1_TXCRCR_ADDR		(SPI1_BASEADDR + 0x18) // TX CRC Register
#define SPI1_I2SCFGR_ADDR		(SPI1_BASEADDR + 0x1C) // configuration register
#define SPI1_I2SPR_ADDR			(SPI1_BASEADDR + 0x20) // prescaler register

/* Peripheral register definition structures for STM32F407xx */

// GPIO Register definition structure
typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSSR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2]; // low[0] and high[1] alternate function registers
} GPIO_RegDef_t;



// RCC

typedef struct {
	__vo uint32_t CR;			/* 0x00 */
	__vo uint32_t PLLCFGR;		/* 0x04 */
	__vo uint32_t CFGR;			/* 0x08 */
	__vo uint32_t CIR ;			/* 0x0C */
	__vo uint32_t AHB1RSTR;		/* 0x10 */
	__vo uint32_t AHB2RSTR ;	/* 0x14 */
	__vo uint32_t AHB3RSTR;		/* 0x18 */
	uint32_t RESERVED_1;		/* 0x1C */
	__vo uint32_t APB1RSTR;		/* 0x20 */
	__vo uint32_t APB2RSTR;		/* 0x24 */
	uint32_t RESERVED_2[2];		/* 0x28 */
	__vo uint32_t AHB1ENR;		/* 0x30 */
	__vo uint32_t AHB2ENR ;		/* 0x34 */
	__vo uint32_t AHB3ENR ;		/* 0x38 */
	uint32_t RESERVED_3;		/* 0x3C */
	__vo uint32_t APB1ENR ;		/* 0x40 */
	__vo uint32_t APB2ENR ;		/* 0x44 */
	uint32_t RESERVED_4[2];		/* 0x48 */
	__vo uint32_t AHB1LPENR ;	/* 0x50 */
	__vo uint32_t AHB2LPENR ;	/* 0x54 */
	__vo uint32_t AHB3LPENR ;	/* 0x58 */
	uint32_t RESERVED_5;		/* 0x5C */
	__vo uint32_t APB1LPENR ;	/* 0x60 */
	__vo uint32_t APB2LPENR;	/* 0x64 */
	uint32_t RESERVED_6[2];		/* 0x68 */
	__vo uint32_t BDCR;			/* 0x70 */
	__vo uint32_t CSR;			/* 0x74 */
	uint32_t RESERVED_7[2];		/* 0x78 */
	__vo uint32_t SSCGR;		/* 0x80 */
	__vo uint32_t PLLI2SCFGR;	/* 0x84 */
	__vo uint32_t PLLSAICFGR;	/* 0x88 */
	__vo uint32_t DCKCFGR ;		/* 0x8C */
}RCC_RegDef_t;


/* Peripheral definitions */
#define GPIOA 				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 				((GPIO_RegDef_t*) GPIOG_BASEADDR)

#define RCC					((RCC_RegDef_t*) RCC_BASEADDR)


/* Clock enable macros */

/* GPIOX */
#define GPIOA_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN() 		( RCC->AHB1ENR |= (1 << 6) )

/* I2Cx */
#define I2C1_PCLK_EN() 			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN() 			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN() 			( RCC->APB1ENR |= (1 << 23) )

/* SPIx */
#define SPI1_PCLK_EN() 			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() 			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN() 			( RCC->APB1ENR |= (1 << 15) )

/* USARTx / UARTx */
#define USART1_PCLK_EN() 		( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN() 		( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN() 		( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN() 		( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN() 		( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN() 		( RCC->APB2ENR |= (1 << 5) )

/* SYSCFG */
#define SYSCFG_PCLK_EN() 		( RCC->APB2ENR |= (1 << 14) )

/* Clock disable macros */

/* GPIOX */
#define NEG_BIT(val, x)			( val &= ~((uint32_t)(1U << x)) )
#define GPIOA_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 0)
#define GPIOB_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 2)
#define GPIOC_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 2)
#define GPIOD_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 3)
#define GPIOE_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 4)
#define GPIOF_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 5)
#define GPIOG_PCLK_DI() 		NEG_BIT(RCC->AHB1ENR, 6)

/* I2Cx */
#define I2C1_PCLK_DI() 			NEG_BIT(RCC->APB1ENR, 21)
#define I2C2_PCLK_DI() 			NEG_BIT(RCC->APB1ENR, 22)
#define I2C3_PCLK_DI() 			NEG_BIT(RCC->APB1ENR, 23)

/* SPIx */
#define SPI1_PCLK_DI() 			NEG_BIT(RCC->APB2ENR, 12)
#define SPI2_PCLK_DI() 			NEG_BIT(RCC->APB1ENR, 14)
#define SPI3_PCLK_DI() 			NEG_BIT(RCC->APB1ENR, 15)

/* USARTx / UARTx */
#define USART1_PCLK_DI() 		NEG_BIT(RCC->APB2ENR, 4)
#define USART2_PCLK_DI() 		NEG_BIT(RCC->APB1ENR, 17)
#define USART3_PCLK_DI() 		NEG_BIT(RCC->APB1ENR, 18)
#define UART4_PCLK_DI() 		NEG_BIT(RCC->APB1ENR, 19)
#define UART5_PCLK_DI() 		NEG_BIT(RCC->APB1ENR, 20)
#define USART6_PCLK_DI() 		NEG_BIT(RCC->APB2ENR, 5)

/* SYSCFG */
#define SYSCFG_PCLK_DI() 		NEG_BIT(RCC->APB2ENR, 14)

/* Generic macros */

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET

#endif /* INC_STM32F407XX_H_ */
