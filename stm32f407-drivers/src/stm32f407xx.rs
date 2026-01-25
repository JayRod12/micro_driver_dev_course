//! STM32F407xx MCU Header
//!
//! Equivalent to: stm32f407xx.h
//!
//! Contains:
//! - Processor specific definitions (NVIC)
//! - Memory base addresses (Flash, SRAM, ROM)
//! - Peripheral base addresses (AHB1, AHB2, APB1, APB2)
//! - Peripheral register definition structures
//! - Peripheral pointer definitions
//! - Clock enable/disable macros
//! - IRQ numbers
//! - Generic macros

#![allow(dead_code)]
#![allow(clippy::missing_safety_doc)]

// =============================================================================
// Processor Specific Definitions (Cortex-M4)
// =============================================================================

pub const NVIC_ISER_BASEADDR: u32 = 0xE000_E100; // -> E11C (0-7)
pub const NVIC_ICER_BASEADDR: u32 = 0xE000_E180; // -> E19C (0-7)
pub const NVIC_IPR_BASEADDR: u32 = 0xE000_E400; // -> E4EF (0-59)

// Only the NVIC_PRIO_BITS most significant bits are implemented in STM32F407
pub const NVIC_PRIO_BITS: u8 = 4;
// STM32F407 supports 82 IRQs (0-81)
pub const NVIC_IRQ_MAX: u8 = 81;

/// NVIC Interrupt Set/Clear Enable Registers (8 registers, 32 IRQs each)
#[repr(C)]
pub struct NvicRegDef {
    pub reg: [u32; 8],
}

/// NVIC Interrupt Priority Registers (60 registers, 4 IRQs each)
#[repr(C)]
pub struct NvicIprRegDef {
    pub reg: [u32; 60],
}

// NVIC peripheral pointers
pub const NVIC_ISER: *mut NvicRegDef = NVIC_ISER_BASEADDR as *mut NvicRegDef;
pub const NVIC_ICER: *mut NvicRegDef = NVIC_ICER_BASEADDR as *mut NvicRegDef;
pub const NVIC_IPR: *mut NvicIprRegDef = NVIC_IPR_BASEADDR as *mut NvicIprRegDef;

// =============================================================================
// Memory Base Addresses
// =============================================================================

// Flash aka Main memory
pub const FLASH_BASEADDR: u32 = 0x0800_0000;

pub const SRAM1_BASEADDR: u32 = 0x2000_0000; // 112Kbs
pub const SRAM2_BASEADDR: u32 = 0x2001_C000;
// ROM aka System Memory
pub const ROM: u32 = 0x1FFF_0000;
pub const SRAM: u32 = SRAM1_BASEADDR;

// =============================================================================
// Peripheral Bus Base Addresses
// =============================================================================

pub const PERI_BASE: u32 = 0x4000_0000;
pub const APB1_PERI_BASE: u32 = PERI_BASE; // 0x4000_0000
pub const APB2_PERI_BASE: u32 = 0x4001_0000;
pub const AHB1_PERI_BASE: u32 = 0x4002_0000;
pub const AHB2_PERI_BASE: u32 = 0x5000_0000;

// =============================================================================
// AHB1 Peripheral Base Addresses
// =============================================================================
pub const RCC_BASEADDR: u32 = AHB1_PERI_BASE + 0x3800;
pub const GPIOK_BASEADDR: u32 = AHB1_PERI_BASE + 0x2800;
pub const GPIOJ_BASEADDR: u32 = AHB1_PERI_BASE + 0x2400;
pub const GPIOI_BASEADDR: u32 = AHB1_PERI_BASE + 0x2000;
pub const GPIOH_BASEADDR: u32 = AHB1_PERI_BASE + 0x1C00;
pub const GPIOG_BASEADDR: u32 = AHB1_PERI_BASE + 0x1800;
pub const GPIOF_BASEADDR: u32 = AHB1_PERI_BASE + 0x1400;
pub const GPIOE_BASEADDR: u32 = AHB1_PERI_BASE + 0x1000;
pub const GPIOD_BASEADDR: u32 = AHB1_PERI_BASE + 0x0C00;
pub const GPIOC_BASEADDR: u32 = AHB1_PERI_BASE + 0x0800;
pub const GPIOB_BASEADDR: u32 = AHB1_PERI_BASE + 0x0400;
pub const GPIOA_BASEADDR: u32 = AHB1_PERI_BASE;

// =============================================================================
// APB1 Peripheral Base Addresses
// =============================================================================

pub const I2C1_BASEADDR: u32 = APB1_PERI_BASE + 0x5400;
pub const I2C2_BASEADDR: u32 = APB1_PERI_BASE + 0x5800;
pub const I2C3_BASEADDR: u32 = APB1_PERI_BASE + 0x5C00;

pub const SPI2_BASEADDR: u32 = APB1_PERI_BASE + 0x3800;
pub const SPI3_BASEADDR: u32 = APB1_PERI_BASE + 0x3C00;

pub const USART2_BASEADDR: u32 = APB1_PERI_BASE + 0x4400;
pub const USART3_BASEADDR: u32 = APB1_PERI_BASE + 0x4800;
pub const UART4_BASEADDR: u32 = APB1_PERI_BASE + 0x4C00;
pub const UART5_BASEADDR: u32 = APB1_PERI_BASE + 0x5000;

// =============================================================================
// APB2 Peripheral Base Addresses
// =============================================================================

pub const EXTI_BASEADDR: u32 = APB2_PERI_BASE + 0x3C00;
pub const SPI1_BASEADDR: u32 = APB2_PERI_BASE + 0x3000;
pub const SYSCFG_BASEADDR: u32 = APB2_PERI_BASE + 0x3800;
pub const USART1_BASEADDR: u32 = APB2_PERI_BASE + 0x1000;
pub const USART6_BASEADDR: u32 = APB2_PERI_BASE + 0x1400;

// =============================================================================
// Peripheral Register Definition Structures
// =============================================================================
//
// IMPORTANT: Use #[repr(C)] to prevent Rust from reordering fields!

/// GPIO Register Definition (Reference Manual section 8.4)
#[repr(C)]
pub struct GpioRegDef {
    pub moder: u32,
    pub otyper: u32,
    pub ospeedr: u32,
    pub pupdr: u32,
    pub idr: u32,
    pub odr: u32,
    pub bsrr: u32, // Note: BSRR not BSSR (bit set/reset register)
    pub lckr: u32,
    pub afr: [u32; 2],
}

/// RCC Register Definition (Reference Manual section 7.3)
/// Reserved fields maintain correct memory offsets - don't remove them!
#[repr(C)]
pub struct RccRegDef {
    pub cr: u32,          // 0x00
    pub pllcfgr: u32,     // 0x04
    pub cfgr: u32,        // 0x08
    pub cir: u32,         // 0x0C
    pub ahb1rstr: u32,    // 0x10
    pub ahb2rstr: u32,    // 0x14
    pub ahb3rstr: u32,    // 0x18
    _reserved1: u32,      // 0x1C
    pub apb1rstr: u32,    // 0x20
    pub apb2rstr: u32,    // 0x24
    _reserved2: [u32; 2], // 0x28-0x2C
    pub ahb1enr: u32,     // 0x30 <- clock enable registers
    pub ahb2enr: u32,     // 0x34
    pub ahb3enr: u32,     // 0x38
    _reserved3: u32,      // 0x3C
    pub apb1enr: u32,     // 0x40
    pub apb2enr: u32,     // 0x44
    _reserved4: [u32; 2], // 0x48-0x4C
    pub ahb1lpenr: u32,   // 0x50
    pub ahb2lpenr: u32,   // 0x54
    pub ahb3lpenr: u32,   // 0x58
    _reserved5: u32,      // 0x5C
    pub apb1lpenr: u32,   // 0x60
    pub apb2lpenr: u32,   // 0x64
    _reserved6: [u32; 2], // 0x68-0x6C
    pub bdcr: u32,        // 0x70
    pub csr: u32,         // 0x74
    _reserved7: [u32; 2], // 0x78-0x7C
    pub sscgr: u32,       // 0x80
    pub plli2scfgr: u32,  // 0x84
    pub pllsaicfgr: u32,  // 0x88
    pub dckcfgr: u32,     // 0x8C
}

/// EXTI Register Definition (Reference Manual section 12.3)
#[repr(C)]
pub struct ExtiRegDef {
    pub imr: u32,   // Interrupt mask register
    pub emr: u32,   // Event mask register
    pub rtsr: u32,  // Rising trigger selection
    pub ftsr: u32,  // Falling trigger selection
    pub swier: u32, // Software interrupt event register
    pub pr: u32,    // Pending register
}

/// SYSCFG Register Definition (Reference Manual section 9.2)
/// exticr[0]=EXTI0-3, exticr[1]=EXTI4-7, exticr[2]=EXTI8-11, exticr[3]=EXTI12-15
/// Each entry uses 4 bits: 0000-0111 = PA,PB,...PH
#[repr(C)]
pub struct SyscfgRegDef {
    pub memrmp: u32,
    pub pmc: u32,
    pub exticr: [u32; 4],
    _reserved: [u32; 2], // 0x18-0x1C gap before CMPCR at 0x20
    pub cmpcr: u32,
}

/// SPI Register Definition (Reference Manual section 28.5)
#[repr(C)]
pub struct SpiRegDef {
    pub cr1: u32,     // 0x00 Control register 1
    pub cr2: u32,     // 0x04 Control register 2
    pub sr: u32,      // 0x08 Status register
    pub dr: u32,      // 0x0C Data register
    pub crcpr: u32,   // 0x10 CRC polynomial register
    pub rxcrcr: u32,  // 0x14 RX CRC register
    pub txcrcr: u32,  // 0x18 TX CRC register
    pub i2scfgr: u32, // 0x1C I2S configuration register
    pub i2spr: u32,   // 0x20 I2S prescaler register
}

// =============================================================================
// Peripheral Pointer Definitions
// =============================================================================

pub const GPIOA: *mut GpioRegDef = GPIOA_BASEADDR as *mut GpioRegDef;
pub const GPIOB: *mut GpioRegDef = GPIOB_BASEADDR as *mut GpioRegDef;
pub const GPIOC: *mut GpioRegDef = GPIOC_BASEADDR as *mut GpioRegDef;
pub const GPIOD: *mut GpioRegDef = GPIOD_BASEADDR as *mut GpioRegDef;
pub const GPIOE: *mut GpioRegDef = GPIOE_BASEADDR as *mut GpioRegDef;
pub const GPIOF: *mut GpioRegDef = GPIOF_BASEADDR as *mut GpioRegDef;
pub const GPIOG: *mut GpioRegDef = GPIOG_BASEADDR as *mut GpioRegDef;

pub const RCC: *mut RccRegDef = RCC_BASEADDR as *mut RccRegDef;
pub const EXTI: *mut ExtiRegDef = EXTI_BASEADDR as *mut ExtiRegDef;
pub const SYSCFG: *mut SyscfgRegDef = SYSCFG_BASEADDR as *mut SyscfgRegDef;
// TODO: SPI1, SPI2, SPI3 pointers

// =============================================================================
// Clock Enable Functions/Macros
// =============================================================================

#[inline(always)]
pub unsafe fn syscfg_pclk_en() {
    let rcc = &mut *RCC;
    let val = core::ptr::read_volatile(&rcc.apb2enr);
    core::ptr::write_volatile(&mut rcc.apb2enr, val | (1 << 14));
}

// TODO: GPIO clock enable (GPIOA_PCLK_EN through GPIOG_PCLK_EN)
// TODO: I2C clock enable (I2C1_PCLK_EN, I2C2_PCLK_EN, I2C3_PCLK_EN)
// TODO: SPI clock enable (SPI1_PCLK_EN, SPI2_PCLK_EN, SPI3_PCLK_EN)
// TODO: USART/UART clock enable
// TODO: SYSCFG clock enable

// =============================================================================
// Clock Disable Functions/Macros
// =============================================================================

// TODO: GPIO clock disable
// TODO: I2C clock disable
// TODO: SPI clock disable
// TODO: USART/UART clock disable
// TODO: SYSCFG clock disable

// =============================================================================
// Generic Constants
// =============================================================================

pub enum GpioCommand {
    Enable,
    Disable,
}

pub const ENABLE: u8 = 1;
pub const DISABLE: u8 = 0;
pub const SET: u8 = ENABLE;
pub const RESET: u8 = DISABLE;
pub const GPIO_PIN_SET: u8 = SET;
pub const GPIO_PIN_RESET: u8 = RESET;

// =============================================================================
// IRQ Numbers (Interrupt Request) for STM32F407xx
// =============================================================================

pub const IRQ_NO_EXTI0: u8 = 6;
pub const IRQ_NO_EXTI1: u8 = 7;
pub const IRQ_NO_EXTI2: u8 = 8;
pub const IRQ_NO_EXTI3: u8 = 9;
pub const IRQ_NO_EXTI4: u8 = 10;
pub const IRQ_NO_EXTI9_5: u8 = 23;
pub const IRQ_NO_EXTI15_10: u8 = 40;

// SPI IRQ numbers (add when implementing SPI interrupts)
pub const IRQ_NO_SPI1: u8 = 35;
pub const IRQ_NO_SPI2: u8 = 36;
pub const IRQ_NO_SPI3: u8 = 51;
