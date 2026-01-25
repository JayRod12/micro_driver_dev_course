//! STM32F407 Drivers - Bare metal driver development from scratch
//!
//! Project structure mirrors the C course:
//! - stm32f407xx.rs  → stm32f407xx.h (MCU header)
//! - gpio.rs         → stm32f407xx_gpio_driver.h/.c
//! - spi.rs          → stm32f407xx_spi_driver.h/.c
//! - i2c.rs          → stm32f407xx_i2c_driver.h/.c
//! - usart.rs        → stm32f407xx_usart_driver.h/.c

#![no_std]

pub mod stm32f407xx;
pub mod gpio;
pub mod spi;
// pub mod i2c;
// pub mod usart;
