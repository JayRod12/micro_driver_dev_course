/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Jun 8, 2025
 *      Author: root
 */
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>
#include <stdio.h>
static uint8_t gpio_port_to_code(GPIO_RegDef_t *pGPIOx) {
  if (pGPIOx == GPIOA)
    return 0;
  if (pGPIOx == GPIOB)
    return 1;
  if (pGPIOx == GPIOC)
    return 2;
  if (pGPIOx == GPIOD)
    return 3;
  if (pGPIOx == GPIOE)
    return 4;
  if (pGPIOx == GPIOF)
    return 5;
  if (pGPIOx == GPIOG)
    return 6;
  return 0;
}

/**
 * Peripheral Clock enable or disable. Takes a GPIO port base address and enable
 * (1) or disable (0)
 *
 * @param pGpioX base address of gpio peripheral
 * @param enOrDi ENABLE or DISABLE macros
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi) {
  uint8_t portCode = gpio_port_to_code(pGPIOx);
  if (enOrDi == ENABLE) {
    RCC->AHB1ENR |= (1 << portCode);
  } else {
    RCC->AHB1ENR &= ~(1 << portCode);
  }
}

static void configure_exticr(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber) {
  SYSCFG_PCLK_EN();
  uint8_t portCode = gpio_port_to_code(pGPIOx);
  uint8_t regIndex = pinNumber / 4;
  uint8_t fieldOffset = (pinNumber % 4) * 4;
  SYSCFG->EXTICR[regIndex] &= ~(0b1111 << fieldOffset);
  SYSCFG->EXTICR[regIndex] |= (portCode << fieldOffset);
}

static void apply_moder(__vo uint32_t *moder, uint8_t mode,
                        uint32_t pinNumber) {
  uint32_t clearMask = ~(0b11 << (2 * pinNumber));
  uint32_t modeMask = ((uint32_t)mode) << (2 * pinNumber);
  *moder &= clearMask;
  *moder |= modeMask;
}

static void set_irq_priority(uint8_t irqNumber, uint8_t irqPrio) {
  const size_t kIrqsPerIpr = 4;
  const size_t kBitsPerPrio = 8;
  size_t index = irqNumber / kIrqsPerIpr;
  size_t offset = kBitsPerPrio * (irqNumber % kIrqsPerIpr);
  NVIC_IPR->reg[index] &= ~(0xFF << offset);
  // skip unimplemented bits
  NVIC_IPR->reg[index] |=
      ((irqPrio << (kBitsPerPrio - NVIC_PRIO_BITS)) << offset);
}

/* Init and DeInit */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
  // Init clock
  //	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

  const uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
  GPIO_RegDef_t *pGPIOx = pGPIOHandle->pGPIOx;

  // MODER
  {
    // configure the mode of the pin
    const uint8_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    if (mode <= GPIO_MODE_ALTFN) {
      apply_moder(&pGPIOx->MODER, mode, pinNumber);
    } else {
      // Interrupt modes - configure as input in MODER
      apply_moder(&pGPIOx->MODER, GPIO_MODE_IN, pinNumber);
      // Configure edge trigger
      if (mode == GPIO_MODE_IT_FT) {
        // Falling edge trigger
        EXTI->FTSR |= (1 << pinNumber);
        EXTI->RTSR &= ~(1 << pinNumber);
      } else if (mode == GPIO_MODE_IT_RT) {
        // Rising edge trigger
        EXTI->RTSR |= (1 << pinNumber);
        EXTI->FTSR &= ~(1 << pinNumber);
      } else if (mode == GPIO_MODE_IT_RFT) {
        // Rising and falling edge trigger
        EXTI->RTSR |= (1 << pinNumber);
        EXTI->FTSR |= (1 << pinNumber);
      }
      // Configure SYSCFG to map GPIO port to EXTI line
      configure_exticr(pGPIOx, pinNumber);
      // Enable EXTI interrupt delivery
      EXTI->IMR |= (1 << pinNumber);
    }
  }

  // speed
  {
    const uint8_t speed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
    if (speed > 3) {
      printf("Error! GPIO Pin speed must be between 0-3, but found %d", speed);
    } else {
      const uint32_t clearMask = ~(0b11 << (2 * pinNumber));
      const uint32_t speedMask = ((uint32_t)speed) << (2 * pinNumber);
      pGPIOHandle->pGPIOx->OSPEEDR &= clearMask;
      pGPIOHandle->pGPIOx->OSPEEDR |= speedMask;
    }
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
  return (uint16_t)pGPIOx->IDR;
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

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
  pGPIOx->ODR ^= (1 << pinNumber);
  // 0 ^ 1 = 1
  // 1 ^ 1 = 0
}

/* IRQ Configuration and ISR handling */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPrio,
                    uint8_t enableOrDisable) {
  if (irqNumber > NVIC_IRQ_MAX) {
    printf("Error(%s): irqNumber %d exceeds STM32F407 max (%d)\n", __func__,
           irqNumber, NVIC_IRQ_MAX);
    return;
  }
  // ISER0 = 0-31, ISER1 = 32-63, etc.
  size_t index = irqNumber / 32;
  size_t offset = irqNumber % 32;
  if (enableOrDisable == ENABLE) {
    set_irq_priority(irqNumber, irqPrio);
    NVIC_ISER->reg[index] |= (1 << offset);
  } else if (enableOrDisable == DISABLE) {
    NVIC_ICER->reg[index] |= (1 << offset);
  } else {
    printf("Error(%s): unknown enableOrDisable value = %d\n", __func__,
           enableOrDisable);
  }
}

// Example (in reality this is done in user code)
// Overwrites weak symbol in startup_stm32f407vgtx.s
// void EXTI0_IRQHandler(void) {
//   GPIO_IRQHandling(0);
// }

void GPIO_IRQHandling(uint8_t pinNumber) {
  if (EXTI->PR & (1 << pinNumber)) {
    printf("IRQ received on pin %d\n", pinNumber);
    EXTI->PR |= (1 << pinNumber); // clear pending bit by writing 1
  }
}
