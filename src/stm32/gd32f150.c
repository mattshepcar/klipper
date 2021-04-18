// Code to setup clocks and gpio on gd32f150
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // armcm_main
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main
#include "board/gpio.h" // gpio_out_setup

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 2)

// Enable a peripheral clock
void
enable_pclock(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        RCC->APB1ENR |= 1 << pos;
        RCC->APB1ENR;
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        RCC->APB2ENR |= 1 << pos;
        RCC->APB2ENR;
    } else {
        uint32_t pos = (periph_base - AHB2PERIPH_BASE) / 0x400;
        RCC->AHBENR |= 1 << (pos + 17);
        RCC->AHBENR;
    }
}

// Check if a peripheral clock has been enabled
int
is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        return RCC->APB1ENR & (1 << pos);
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        return RCC->APB2ENR & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHB2PERIPH_BASE) / 0x400;
        return RCC->AHBENR & (1 << (pos + 17));
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - AHB2PERIPH_BASE) / 0x400;
    RCC->AHBENR |= 1 << (rcc_pos + 17);
    RCC->AHBENR;
}

// Set the mode and extended function of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf, func = (mode >> 4) & 0xf, od = mode >> 8;
    uint32_t pup = pullup ? (pullup > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16, af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2, m_msk = 0x03 << m_shift;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
    regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
    regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
    regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);
    regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (0x02 << m_shift);
}

//#define USB_BOOT_FLAG_ADDR (CONFIG_RAM_START + CONFIG_RAM_SIZE - 1024)
//#define USB_BOOT_FLAG 0x55534220424f4f54 // "USB BOOT"

// Handle USB reboot requests
void
usb_request_bootloader(void)
{
    irq_disable();
    //*(uint64_t*)USB_BOOT_FLAG_ADDR = USB_BOOT_FLAG;
    NVIC_SystemReset();
}

// Copy vector table and remap ram so new vector table is used
static void
enable_ram_vectortable(void)
{
    // Symbols created by armcm_link.lds.S linker script
    extern uint32_t _ram_vectortable_start, _ram_vectortable_end;
    extern uint32_t _text_vectortable_start;

    uint32_t count = (&_ram_vectortable_end - &_ram_vectortable_start) * 4;
    __builtin_memcpy(&_ram_vectortable_start, &_text_vectortable_start, count);
    barrier();

    enable_pclock(SYSCFG_BASE);
    SYSCFG->CFGR1 |= 3 << SYSCFG_CFGR1_MEM_MODE_Pos;
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PF0,PF1");
#endif

// Configure and enable the PLL as clock source
static void
pll_setup(void)
{
    uint32_t cfgr;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL) {
        // Configure 72Mhz PLL from external crystal (HSE)
        uint32_t div = CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ;
        RCC->CR |= RCC_CR_HSEON;
        cfgr = RCC_CFGR_PLLSRC_HSE_PREDIV | ((div - 2) << RCC_CFGR_PLLMUL_Pos);
    } else {
        // Configure 72Mhz PLL from internal 8Mhz oscillator (HSI)
        uint32_t div2 = (CONFIG_CLOCK_FREQ / 8000000) * 2;
        cfgr = RCC_CFGR_PLLSRC_HSI_DIV2 | ((div2 - 2) << RCC_CFGR_PLLMUL_Pos);
    }
    RCC->CFGR = cfgr;
    RCC->CR |= RCC_CR_PLLON;

    // Wait for PLL lock
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Switch system clock to PLL
    RCC->CFGR = cfgr | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
        ;

    // Setup CFGR3 register
    uint32_t cfgr3 = RCC_CFGR3_I2C1SW;
#if CONFIG_USBSERIAL
    // Select PLL as source for USB clock
    cfgr3 |= RCC_CFGR3_USBSW;
#endif
    RCC->CFGR3 = cfgr3;
}


// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    //GPIOA->ODR ^= 2;

    // if (CONFIG_USBSERIAL && 
    //     && *(uint64_t*)USB_BOOT_FLAG_ADDR == USB_BOOT_FLAG) {
    //     *(uint64_t*)USB_BOOT_FLAG_ADDR = 0;
    //     uint32_t *sysbase = (uint32_t*)0x1fffc400;
    //     asm volatile("mov sp, %0\n bx %1"
    //                  : : "r"(sysbase[0]), "r"(sysbase[1]));
    // }

    //SystemInit();
    RCC->CSR |= RCC_CSR_RMVF;
//   /* Reset the RCC clock configuration to the default reset state ------------*/
//   /* Set HSION bit */
   RCC->CR |= (uint32_t)0x00000001U;
//   /* Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE, MCOSEL[2:0], MCOPRE[2:0] and PLLNODIV bits */
   RCC->CFGR &= (uint32_t)0x08FFB80CU;
//   /* Reset HSEON, CSSON and PLLON bits */
   RCC->CR &= (uint32_t)0xFEF6FFFFU;
//   /* Reset HSEBYP bit */
   RCC->CR &= (uint32_t)0xFFFBFFFFU;
//   /* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
   RCC->CFGR &= (uint32_t)0xFFC0FFFFU;
//   /* Reset PREDIV[3:0] bits */
   RCC->CFGR2 &= (uint32_t)0xFFFFFFF0U;
//   /* Reset HSI14 bit */
   RCC->CR2 &= (uint32_t)0xFFFFFFFEU;
//   /* Disable all interrupts */
   RCC->CIR = 0x00000000U;

    if (CONFIG_ARMCM_RAM_VECTORTABLE)
        enable_ram_vectortable();

    // Set flash latency
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | (2 << FLASH_ACR_LATENCY_Pos);

    // Configure main clock
    pll_setup();

    sched_main();
}

