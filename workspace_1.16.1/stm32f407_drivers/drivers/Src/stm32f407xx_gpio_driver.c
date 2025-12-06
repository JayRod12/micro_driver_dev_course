/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Jun 8, 2025
 *      Author: root
 */
#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx_gpio_driver.h"
/**
 * Peripheral Clock enable or disable. Takes a GPIO port base address and enable (1) or disable (0)
 *
 * @param pGpioX base address of gpio peripheral
 * @param enOrDi ENABLE or DISABLE macros
 * */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi) {
	if (enOrDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else {
			printf(
					"Error: Unknown GPIO base address in GPIO_PeriClockControl(En): %p\n",
					pGPIOx);
		}
	} else {
		assert(enOrDi == DISABLE);
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else {
			printf(
					"Error: Unknown GPIO base address in GPIO_PeriClockControl(Di): %p\n",
					pGPIOx);
		}
	}
}

/* Init and DeInit */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	// Init clock
//	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	const uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// MODER
	{
		// configure the mode of the pin
		const uint8_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
		if (mode <= GPIO_MODE_ALTFN) {
			// clear 2 bits
			// each port uses 2 bits
			// build a mask with all 1s except a 00 in the port's location
			uint32_t clearMask = ~(0b11 << (2 * pinNumber));
			uint32_t modeMask = ((uint32_t)mode) << (2 * pinNumber);
			pGPIOHandle->pGPIOx->MODER &= clearMask;
			pGPIOHandle->pGPIOx->MODER |= modeMask;
		} else {
			// TODO: interrupt modes not implemented yet
			return;
		}
	}

	// speed
	{
		const uint8_t speed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
		const uint32_t clearMask = ~(0b11 << (2 * pinNumber));
		const uint32_t speedMask = ((uint32_t)speed) << (2 * pinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR &= clearMask;
		pGPIOHandle->pGPIOx->OSPEEDR |= speedMask;
	}

	// pupd
	{
		const uint8_t pupd = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;
		const uint32_t clearMask = ~(0b11 << (2 * pinNumber));
		const uint32_t pupdMask = ((uint32_t)pupd) << (2 * pinNumber);
		pGPIOHandle->pGPIOx->PUPDR &= clearMask;
		pGPIOHandle->pGPIOx->PUPDR |= pupdMask;
	}

	// optype
	{
		const uint8_t outputType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
		const uint32_t clearMask = ~(0b11 << pinNumber);
		const uint32_t outputTypeMask = ((uint32_t)outputType) << pinNumber;
		pGPIOHandle->pGPIOx->OTYPER &= clearMask;
		pGPIOHandle->pGPIOx->OTYPER |= outputTypeMask;
	}
	// alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		const uint32_t altfun = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
		// 0-7 in low register
		// 8-15 in high register
		const uint8_t regIndex = (uint8_t)(pinNumber / 8);
		const uint8_t pinNumberMod8 = pinNumber % 8;
		const uint32_t clearMask = ~(0b1111 << (4 * pinNumberMod8));
		const uint32_t setMask = altfun << (4 * pinNumberMod8);
		// Low register
		pGPIOHandle->pGPIOx->AFR[regIndex] &= clearMask;
		pGPIOHandle->pGPIOx->AFR[regIndex] |= setMask;
	}


}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	// Deinit clock
	// TODO: I think this is delegated to outside the API
	//	GPIO_PeriClockControl(pGPIOx, DISABLE);
	// RCC reset
	if (pGPIOx == GPIOA) {
		GPIOA_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_RESET();
	} else {
		printf(
				"Error: Unknown GPIO base address in GPIO_PeriClockControl(En): %p\n",
				pGPIOx);
	}

}

/* Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	return (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
}

// There are 16 pins in a port, so we need 16 bits of output
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) pGPIOx->IDR;
}
// Value written can be 0 or 1
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value) {
	// not read-modify-write safe. if desired, use BSSR
	if (value == GPIO_PIN_RESET) {
		// 0
		pGPIOx->ODR &= ~(1 << pinNumber); // clear
	} else {
		// 1
		pGPIOx->ODR |= (1 << pinNumber); // set
	}


}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1 << pinNumber);
	// 0 ^ 1 = 1
	// 1 ^ 1 = 0
}

/* IRQ Configuration and ISR handling */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPrio, uint8_t enableOrDisable) {

}
void GPIO_IRQHandling(uint8_t pinNumber) {

}
