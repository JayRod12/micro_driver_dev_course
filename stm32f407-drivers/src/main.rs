//! STM32F407 Driver Examples
//!
//! This file contains example applications using the bare-metal drivers.
//! Equivalent to the Src/*.c files in the C project.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

// Import your driver library
use stm32f407_drivers::stm32f407xx::*;
use stm32f407_drivers::gpio::*;
// use stm32f407_drivers::spi::*;

#[entry]
fn main() -> ! {
    // ==========================================================
    // Example: LED Blink (PD12 - Green LED on Discovery board)
    // ==========================================================
    //
    // TODO: Once you implement the drivers, this is how you'd use them:
    //
    // unsafe {
    //     // 1. Enable GPIOD clock
    //     gpio_peri_clock_control(GPIOD, ENABLE);
    //
    //     // 2. Configure PD12 as output
    //     let handle = GpioHandle {
    //         gpio: GPIOD,
    //         config: GpioPinConfig {
    //             pin_number: GPIO_PIN_NO_12,
    //             pin_mode: GPIO_MODE_OUT,
    //             pin_speed: GPIO_SPEED_LOW,
    //             pin_pupd_control: GPIO_PIN_NO_PUPD,
    //             pin_op_type: GPIO_OP_TYPE_PP,
    //             pin_alt_fun_mode: 0,
    //         },
    //     };
    //     gpio_init(&handle);
    //
    //     // 3. Blink LED
    //     loop {
    //         gpio_toggle_output_pin(GPIOD, GPIO_PIN_NO_12);
    //         delay(500_000);
    //     }
    // }

    // Temporary: simple loop until drivers are implemented
    loop {
        cortex_m::asm::nop();
    }
}

/// Simple delay using NOP instructions
fn delay(count: u32) {
    for _ in 0..count {
        cortex_m::asm::nop();
    }
}

// =============================================================================
// Additional Examples (uncomment as you implement drivers)
// =============================================================================

// Example: GPIO Interrupt (button press)
// fn button_interrupt_example() { ... }

// Example: SPI Master TX
// fn spi_master_tx_example() { ... }

// Example: SPI Full Duplex
// fn spi_full_duplex_example() { ... }
