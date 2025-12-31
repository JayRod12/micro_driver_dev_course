/**
 ******************************************************************************
 * @file           : main_gpio_irq.c
 * @brief          : GPIO interrupt test - button triggers LED toggle
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU."
#endif

int main(void) {
  // Configure LED on PD12
  {
    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Handle_t led;
    led.pGPIOx = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
    GPIO_Init(&led);
  }

  // Opt 1. Configure board button on PA0 with falling edge interrupt
  // {
  //   GPIO_PeriClockControl(GPIOA, ENABLE);
  //   GPIO_Handle_t button;
  //   button.pGPIOx = GPIOA;
  //   button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
  //   button.GPIO_PinConfig.GPIO_PinMode =
  //       GPIO_MODE_IT_FT; // Falling edge trigger
  //   button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
  //   button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
  //   GPIO_Init(&button);
  //
  //   // Enable IRQ in NVIC
  //   GPIO_IRQConfig(IRQ_NO_EXTI0, 15, ENABLE);
  // }

  // Opt 2. Configure external button on PD5 with falling edge interrupt
  {
    // gpiod peri control done for led already
    GPIO_Handle_t pd5;
    pd5.pGPIOx = GPIOD;
    pd5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    pd5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    pd5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    pd5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    pd5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GPIO_Init(&pd5);
    GPIO_IRQConfig(IRQ_NO_EXTI9_5, 15, ENABLE);
  }

  // Loop forever - interrupts will handle button presses
  for (;;) {
  }
}

// ISR for EXTI0 (PA0 button)
void EXTI0_IRQHandler(void) {
  GPIO_IRQHandling(0); // Clear pending bit
  GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

// ISR for EXTI5 (PD5 button)
void EXTI9_5_IRQHandler(void) {
  GPIO_IRQHandling(7); // Clear pending bit
  GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
