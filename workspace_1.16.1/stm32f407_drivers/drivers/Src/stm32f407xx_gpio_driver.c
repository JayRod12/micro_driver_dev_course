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
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	// MODER

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	// Deinit clock
	GPIO_PeriClockControl(pGPIOx, DISABLE);
	// RCC reset
}

/* Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

}

// There are 16 pins in a port, so we need 16 bits of output
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

}
// Value written can be 0 or 1
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value) {

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){

}
/* IRQ Configuration and ISR handling */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPrio, uint8_t enableOrDisable) {

}
void GPIO_IRQHandling(uint8_t pinNumber) {

}
