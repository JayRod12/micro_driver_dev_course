//! STM32F407xx GPIO Driver
//!
//! Equivalent to: stm32f407xx_gpio_driver.h / stm32f407xx_gpio_driver.c
//!
//! Contains:
//! - GPIO pin configuration structure
//! - GPIO handle structure
//! - GPIO pin modes, output types, speeds, pull-up/pull-down settings
//! - Driver API functions

#![allow(dead_code)]
#![allow(clippy::missing_safety_doc)]

use core::{char::MAX, pin};

use crate::stm32f407xx::*;

#[derive(Debug, Clone, Copy)]
pub enum GpioError {
    InvalidPin,
    InvalidMode,
    InvalidValueForPin,
    InvalidIrqNumber,
    InvalidIrqPrio,
    UnimplementedMode,
    LogicalError,
    InvalidGpioAddress,
}

pub const MAX_PIN: u8 = 15;

// =============================================================================
// GPIO Configuration Structure
// =============================================================================
//
// Equivalent to GPIO_PinConfig_t in C

// TODO: Define GpioPinConfig struct with fields:
// - pin_number: u8
// - pin_mode: u8        (see GPIO_PIN_MODES)
// - pin_speed: u8       (see GPIO_PIN_SPEED)
// - pin_pupd_control: u8 (see GPIO_PIN_PUPD)
// - pin_op_type: u8     (see GPIO_PIN_OP_TYPE)
// - pin_alt_fun_mode: u8
pub struct GpioPinConfig {
    pub pin_number: u8,
    pub pin_mode: GpioMode,
    pub pin_speed: u8,
    pub pin_pupd_control: u8,
    pub pin_op_type: u8,
    pub pin_alt_fun_mode: u8,
}

// GPIO Handle Structure. Equivalent to GPIO_Handle_t in C
pub struct GpioHandle {
    pub p_gpiox: *mut GpioRegDef,
    pub gpio_pin_config: GpioPinConfig,
}

// =============================================================================
// @GPIO_PIN_MODES
// =============================================================================

#[derive(Copy, Clone)]
pub enum GpioMode {
    In,
    Out,
    AltFn,
    Analog,
    InterruptFallingEdge,
    InterruptRisingEdge,
    InterruptFallingRisingEdge,
}

fn check_pin_number(pin_number: u8) -> Result<(), GpioError> {
    if pin_number > MAX_PIN {
        return Err(GpioError::InvalidPin);
    }
    Ok(())
}

fn is_interrupt_mode(mode: GpioMode) -> bool {
    matches!(
        mode,
        GpioMode::InterruptFallingEdge
            | GpioMode::InterruptRisingEdge
            | GpioMode::InterruptFallingRisingEdge,
    )
}

// TODO: GPIO_MODE_IN (0)      - Input mode
// TODO: GPIO_MODE_OUT (1)     - General purpose output
// TODO: GPIO_MODE_ALTFN (2)   - Alternate function
// TODO: GPIO_MODE_ANALOG (3)  - Analog mode
// TODO: GPIO_MODE_IT_FT (4)   - Interrupt falling edge trigger
// TODO: GPIO_MODE_IT_RT (5)   - Interrupt rising edge trigger
// TODO: GPIO_MODE_IT_RFT (6)  - Interrupt rising/falling edge trigger

// =============================================================================
// @GPIO_PIN_OP_TYPE - Output Type
// =============================================================================

// TODO: GPIO_OP_TYPE_PP (0)  - Push-pull
// TODO: GPIO_OP_TYPE_OD (1)  - Open drain

// =============================================================================
// @GPIO_PIN_SPEED - Output Speed
// =============================================================================

// TODO: GPIO_SPEED_LOW (0)
// TODO: GPIO_SPEED_MED (1)
// TODO: GPIO_SPEED_HIGH (2)
// TODO: GPIO_SPEED_VERY_HIGH (3)

// =============================================================================
// @GPIO_PIN_PUPD - Pull-up / Pull-down
// =============================================================================

// TODO: GPIO_PIN_NO_PUPD (0)
// TODO: GPIO_PIN_PU (1)
// TODO: GPIO_PIN_PD (2)

// =============================================================================
// Driver API - Peripheral Clock Control
// =============================================================================

/// Enable or disable peripheral clock for a GPIO port
///
/// # Arguments
/// * `gpio` - Base address of the GPIO port
/// * `enable_or_disable` - ENABLE or DISABLE
///
/// # Safety
/// Caller must ensure `gpio` points to a valid GPIO peripheral
pub unsafe fn gpio_peri_clock_control(gpio: *mut GpioRegDef, enable_or_disable: u8) {
    // TODO: Implement based on which GPIO port is passed
    // Hint: Compare gpio pointer against GPIOA, GPIOB, etc.
    // Then set/clear the appropriate bit in RCC->AHB1ENR
    todo!()
}

// =============================================================================
// Driver API - Init and DeInit
// =============================================================================

unsafe fn apply_moder(reg_addr: *mut u32, mode: GpioMode, pin_number: u8) -> Result<(), GpioError> {
    check_pin_number(pin_number)?;
    let mode_as_val: Result<u32, GpioError> = match mode {
        GpioMode::Analog
        | GpioMode::AltFn
        | GpioMode::InterruptFallingEdge
        | GpioMode::InterruptRisingEdge
        | GpioMode::InterruptFallingRisingEdge => Err(GpioError::InvalidMode),
        GpioMode::In => Ok(0),
        GpioMode::Out => Ok(1),
    };
    let clear_mask: u32 = !(0b11 << (2 * pin_number));
    let mode_mask: u32 = mode_as_val? << (2 * pin_number);
    let val = core::ptr::read_volatile(reg_addr);
    core::ptr::write_volatile(reg_addr, (val & clear_mask) | mode_mask);
    Ok(())
}

fn gpio_port_to_code(p_gpiox: *mut GpioRegDef) -> Result<u32, GpioError> {
    match p_gpiox {
        GPIOA => Ok(0),
        GPIOB => Ok(1),
        GPIOC => Ok(2),
        GPIOD => Ok(3),
        GPIOE => Ok(4),
        GPIOF => Ok(5),
        GPIOG => Ok(6),
        _ => Err(GpioError::InvalidGpioAddress),
    }
}

unsafe fn configure_exticr(p_gpiox: *mut GpioRegDef, pin_number: u8) -> Result<(), GpioError> {
    syscfg_pclk_en();
    check_pin_number(pin_number)?;
    let port_code: u32 = gpio_port_to_code(p_gpiox)?;
    let reg_index = (pin_number / 4) as usize;
    let field_offset = (pin_number % 4) * 4;
    let val = core::ptr::read_volatile(&(*SYSCFG).exticr[reg_index]);
    core::ptr::write_volatile(
        &mut (*SYSCFG).exticr[reg_index],
        (val & !(0b1111 << field_offset)) | (port_code << field_offset),
    );
    Ok(())
}

/// Initialize a GPIO pin according to the handle configuration
///
/// # Safety
/// Caller must ensure handle contains valid GPIO pointer and configuration
pub unsafe fn gpio_init(handle: &GpioHandle) -> Result<(), GpioError> {
    // TODO: Configure the GPIO pin:
    // 2. Configure speed (OSPEEDR register)
    // 3. Configure pull-up/pull-down (PUPDR register)
    // 4. Configure output type (OTYPER register)
    // 5. Configure alternate function if needed (AFR registers)
    // 6. For interrupt modes: configure EXTI, SYSCFG

    // TODO
    let pin_number = handle.gpio_pin_config.pin_number;
    let mode = handle.gpio_pin_config.pin_mode;
    match mode {
        GpioMode::Out => apply_moder(&mut (*handle.p_gpiox).moder, GpioMode::Out, pin_number),
        GpioMode::In
        | GpioMode::InterruptFallingEdge
        | GpioMode::InterruptRisingEdge
        | GpioMode::InterruptFallingRisingEdge => {
            apply_moder(&mut (*handle.p_gpiox).moder, GpioMode::In, pin_number)
        }
        _ => Err(GpioError::UnimplementedMode),
    }?;

    if is_interrupt_mode(mode) {
        let exti = &mut *EXTI;
        let ftsr = core::ptr::read_volatile(&exti.ftsr);
        let rtsr = core::ptr::read_volatile(&exti.rtsr);
        match mode {
            GpioMode::InterruptFallingEdge => {
                core::ptr::write_volatile(&mut exti.ftsr, ftsr | (1 << pin_number));
                core::ptr::write_volatile(&mut exti.rtsr, rtsr & !(1 << pin_number));
            }
            GpioMode::InterruptRisingEdge => {
                core::ptr::write_volatile(&mut exti.ftsr, ftsr & !(1 << pin_number));
                core::ptr::write_volatile(&mut exti.rtsr, rtsr | (1 << pin_number));
            }
            GpioMode::InterruptFallingRisingEdge => {
                core::ptr::write_volatile(&mut exti.ftsr, ftsr | (1 << pin_number));
                core::ptr::write_volatile(&mut exti.rtsr, rtsr | (1 << pin_number));
            }
            _ => (),
        }
        // implement exticr
        configure_exticr(handle.p_gpiox, pin_number)?;
        let imr = core::ptr::read_volatile(&exti.imr);
        core::ptr::write_volatile(&mut exti.imr, imr | (1 << pin_number));
    };
    todo!("speed, pupd, optype, alt functionality")
}

/// Reset a GPIO port to its default state
///
/// # Safety
/// Caller must ensure `gpio` points to a valid GPIO peripheral
pub unsafe fn gpio_deinit(gpio: *mut GpioRegDef) {
    // TODO: Reset the GPIO port using RCC reset registers
    // Hint: Set then clear the appropriate bit in RCC->AHB1RSTR
    todo!()
}

// =============================================================================
// Driver API - Read and Write
// =============================================================================

/// Read from a specific GPIO input pin
///
/// # Returns
/// 0 or 1
pub unsafe fn gpio_read_from_input_pin(
    gpio: *mut GpioRegDef,
    pin_number: u8,
) -> Result<u8, GpioError> {
    if pin_number > MAX_PIN {
        return Err(GpioError::InvalidPin);
    }
    check_pin_number(pin_number)?;
    Ok(((core::ptr::read_volatile(&(*gpio).idr) >> pin_number) & 1) as u8)
}

/// Read the entire GPIO input port
///
/// # Returns
/// 16-bit value representing all pins
pub unsafe fn gpio_read_from_input_port(gpio: *mut GpioRegDef) -> Result<u16, GpioError> {
    Ok(core::ptr::read_volatile(&(*gpio).idr) as u16)
}

/// Write to a specific GPIO output pin
/// Note: This operation is not atomic read-modify-write. If desired use BSRR.
pub unsafe fn gpio_write_to_output_pin(
    gpio: *mut GpioRegDef,
    pin_number: u8,
    value: u8,
) -> Result<(), GpioError> {
    if pin_number > MAX_PIN {
        return Err(GpioError::InvalidPin);
    }
    let current: u32 = core::ptr::read_volatile(&(*gpio).odr) as u32;
    match value {
        0 => {
            core::ptr::write_volatile(&mut (*gpio).odr, current & !(1 << pin_number));
            Ok(())
        }
        1 => {
            core::ptr::write_volatile(&mut (*gpio).odr, current | (1 << pin_number));
            Ok(())
        }
        _ => Err(GpioError::InvalidValueForPin),
    }
}

/// Write to the entire GPIO output port
pub unsafe fn gpio_write_to_output_port(
    gpio: *mut GpioRegDef,
    value: u16,
) -> Result<(), GpioError> {
    // TODO: Write to ODR register
    core::ptr::write_volatile(&mut (*gpio).odr, value as u32);
    Ok(())
}

/// Toggle a GPIO output pin
/// Note: Operation is not an atomic read-modify-write
pub unsafe fn gpio_toggle_output_pin(
    gpio: *mut GpioRegDef,
    pin_number: u8,
) -> Result<(), GpioError> {
    if pin_number > MAX_PIN {
        return Err(GpioError::InvalidPin);
    }
    // TODO: XOR the appropriate bit in ODR register
    let current: u32 = core::ptr::read_volatile(&(*gpio).odr);
    core::ptr::write_volatile(&mut (*gpio).odr, current ^ (1 << pin_number));
    Ok(())
}

// =============================================================================
// Driver API - Interrupt Handling
// =============================================================================

unsafe fn set_irq_priority(irq_number: u8, irq_priority: u8) -> Result<(), GpioError> {
    let irqs_per_ipr = 4;
    let bits_per_prio = 8;
    let index = (irq_number / irqs_per_ipr) as usize;
    let offset = bits_per_prio * (irq_number % irqs_per_ipr);
    if irq_priority >= (1 << NVIC_PRIO_BITS) {
        return Err(GpioError::InvalidIrqPrio);
    }
    let mut val = core::ptr::read_volatile(&(*NVIC_IPR).reg[index]);
    // clear bits for irq number
    val &= !(0xFF << offset);
    // set, skipping unimplemented bits
    val |= ((irq_priority << (bits_per_prio - NVIC_PRIO_BITS)) << offset) as u32;
    core::ptr::write_volatile(&mut (*NVIC_IPR).reg[index], val);
    Ok(())
}

/// Configure IRQ for a GPIO pin
///
/// # Arguments
/// * `irq_number` - IRQ number (see IRQ_NO_* constants)
/// * `irq_priority` - Priority value (0-15 for STM32F407)
/// * `command` - GpioCommand
pub unsafe fn gpio_irq_config(
    irq_number: u8,
    irq_priority: u8,
    command: GpioCommand,
) -> Result<(), GpioError> {
    // TODO: Configure NVIC:
    // 1. Enable/disable IRQ in NVIC_ISER/NVIC_ICER
    // 2. Set priority in NVIC_IPR
    if irq_number > NVIC_IRQ_MAX {
        return Err(GpioError::InvalidIrqNumber);
    }
    // ISER0 = 0-31, ISER1 = 32-63, etc.
    let index: usize = (irq_number / 32) as usize;
    let offset: u8 = irq_number % 32;
    match command {
        GpioCommand::Enable => {
            // ISER
            set_irq_priority(irq_number, irq_priority)?;
            let iser_reg_val = core::ptr::read_volatile(&(*NVIC_ISER).reg[index]);
            core::ptr::write_volatile(&mut (*NVIC_ISER).reg[index], iser_reg_val | (1 << offset));
            Ok(())
        }
        GpioCommand::Disable => {
            // ICER
            let icer_reg_val = core::ptr::read_volatile(&(*NVIC_ICER).reg[index]);
            core::ptr::write_volatile(&mut (*NVIC_ICER).reg[index], icer_reg_val | (1 << offset));
            Ok(())
        }
    }
}

/// Handle GPIO interrupt - clear pending bit
///
/// Must be called from the ISR to acknowledge the interrupt
pub unsafe fn gpio_irq_handling(pin_number: u8) {
    // TODO: Clear the pending bit in EXTI->PR
    todo!()
}
