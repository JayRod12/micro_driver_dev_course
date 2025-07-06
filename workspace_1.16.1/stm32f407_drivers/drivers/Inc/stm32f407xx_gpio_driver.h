/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jun 8, 2025
 *      Author: root
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx; /* Base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig; /* Pin configuration settings */

} GPIO_Handle_t;


/**
 * APIs supported by the driver
 */

/* Peripheral Clock enable or disable. Takes a GPIO port base address and enable (1) or disable (0) */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enableOrDisable);

/* Init and DeInit */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
// There are 16 pins in a port, so we need 16 bits of output
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
// Value written can be 0 or 1
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
/* IRQ Configuration and ISR handling */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPrio, uint8_t enableOrDisable);
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
