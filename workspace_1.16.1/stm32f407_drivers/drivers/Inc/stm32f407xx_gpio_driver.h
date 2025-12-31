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
  uint8_t GPIO_PinMode;        /*!< possible values from @GPIO_PIN_MODES >*/
  uint8_t GPIO_PinSpeed;       /*!< possible values from @GPIO_PIN_SPEED >*/
  uint8_t GPIO_PinPuPdControl; /*!< possible values from @GPIO_PIN_PUPD >*/
  uint8_t GPIO_PinOPType;      /*!< possible values from @GPIO_PIN_OP_TYPE >*/
  uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
  GPIO_RegDef_t
      *pGPIOx; /* Base address of the GPIO port to which the pin belongs */
  GPIO_PinConfig_t GPIO_PinConfig; /* Pin configuration settings */
} GPIO_Handle_t;

/* @GPIO_PIN_NO */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 **/
#define GPIO_MODE_IN 0     // input (reset)
#define GPIO_MODE_OUT 1    // general purpose output
#define GPIO_MODE_ALTFN 2  // alternate function
#define GPIO_MODE_ANALOG 3 // analog
// custom interrupt modes (not in spec). gpio can be configured to deliver
// interrupts to the microcontroller when a falling edge or rising edge on the
// gpio pin is detected
#define GPIO_MODE_IT_FT 4  // falling edge trigger
#define GPIO_MODE_IT_RT 5  // rising edge trigger
#define GPIO_MODE_IT_RFT 6 // rising edge, falling edge trigger

/* @GPIO_PIN_OP_TYPE GPIO TYPER - output type register*/
#define GPIO_OP_TYPE_PP 0 // output type push pull
#define GPIO_OP_TYPE_OD 1 // output type open drain
#define GPIO_OP_TYPE_MAX GPIO_OP_TYPE_OD

/* @GPIO_PIN_SPEED GPIO Output speed */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MED 1
#define GPIO_SPEED_HIGH 2
#define GPIO_SPEED_VERY_HIGH 3

/* @GPIO_PIN_PUPD GPIO Pull-Up Pull-Down settings */
#define GPIO_PIN_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

///* GPIO register macros */
// #define GPIOA_SET_MODE(mode, port) GPIOA->MODER |= (mode << port); // port
// 0->15, mode 0-3

/**
 * APIs supported by the driver
 */

/* Peripheral Clock enable or disable. Takes a GPIO port base address and enable
 * (1) or disable (0) */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enableOrDisable);

/* Init and DeInit */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
// There are 16 pins in a port, so we need 16 bits of output
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
// Value written can be 0 or 1
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
                           uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * Interrupt terminology.
 *
 * IRQ (Interrupt Request): The signal/number identifying which interrupt
 * occurred (e.g., IRQ #6 for EXTI0).
 *
 * ISR (Interrupt Service Routine): The handler function that runs in response
 * to an IRQ (e.g., EXTI0_IRQHandler).
 */

/*
 * Configure a specific IRQ number with priority (only applicable when enabling)
 */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPrio,
                    uint8_t enableOrDisable);
/*
 * Notify driver that an ISR has been handled by the user.
 *
 * Must be called from user code when the ISR is called. This takes care of
 * reading and clearing the pending bit. Otherwise future interrupts won't be
 * correctly delivered.
 */
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
