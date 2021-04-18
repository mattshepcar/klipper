#ifndef GD32_F150_h
#define GD32_F150_h

#ifdef __cplusplus
 extern "C" {
#endif 

#define __CM3_REV                  0x0200U  /*!< Core Revision r2p0                           */
 #define __MPU_PRESENT             0U       /*!< Other STM32 devices does not provide an MPU  */
#define __NVIC_PRIO_BITS           4U       /*!< STM32 uses 4 Bits for the Priority Levels    */
#define __Vendor_SysTickConfig     0U       /*!< Set to 1 if different SysTick Config is used */


 /*!< Interrupt Number Definition */
typedef enum
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M3 Hard Fault Interrupt                     */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

/******  STM32F0 specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                               */
  // pvd
  RTC_IRQn                    = 2,      /*!< RTC Interrupt through EXTI Lines 17, 19 and 20                  */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                          */
  RCC_IRQn                    = 4,      /*!< RCC global Interrupt                                            */
  EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupt                                     */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupt                                     */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupt                                     */
  // tsc
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                        */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupt                          */
  DMA1_Channel4_5_IRQn        = 11,     /*!< DMA1 Channel 4 and Channel 5 Interrupt                          */
  ADC1_IRQn                   = 12,     /*!< ADC1 Interrupt                                                  */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupt           */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                  */
  TIM2_IRQn                   = 15,     /*!< TIM2 global Interrupt                                           */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                           */
  //tim6dac
  //reserved0
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                                          */
  //tim15
  TIM16_IRQn                  = 21,     /*!< TIM16 global Interrupt                                          */
  TIM17_IRQn                  = 22,     /*!< TIM17 global Interrupt                                          */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)      */
  //i2c2
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                           */
  SPI2_IRQn                   = 26,     /*!< SPI1 global Interrupt                                           */
  //spi2
  USART1_IRQn                 = 27,      /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup) */
	USB_LP_IRQn = 37,
 
} IRQn_Type;

#include "core_cm3.h"            /* Cortex-M0 processor and core peripherals */

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function low register,  Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */
} GPIO_TypeDef;

typedef struct
{
  __IO uint32_t CR1;          /*!< I2C Control register 1,                      Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< I2C Control register 2,                      Address offset: 0x04 */
  __IO uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI Control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI Status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
} SPI_TypeDef;

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                   Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __IO uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  __IO uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __IO uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */
} RCC_TypeDef;

typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR1;        /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t CFGR2;        /*!< ADC configuration register 2,                  Address offset: 0x10 */
  __IO uint32_t SMPR;         /*!< ADC sampling time register,                    Address offset: 0x14 */
       uint32_t RESERVED1;    /*!< Reserved,                                                      0x18 */
       uint32_t RESERVED2;    /*!< Reserved,                                                      0x1C */
  __IO uint32_t TR;           /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
       uint32_t RESERVED3;    /*!< Reserved,                                                      0x24 */
  __IO uint32_t CHSELR;       /*!< ADC group regular sequencer register,          Address offset: 0x28 */
       uint32_t RESERVED4[5]; /*!< Reserved,                                                      0x2C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CCR;          /*!< ADC common configuration register,             Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

typedef struct
{
  __IO uint16_t EP0R;            /*!< USB Endpoint 0 register,                Address offset: 0x00 */ 
  __IO uint16_t RESERVED0;       /*!< Reserved */     
  __IO uint16_t EP1R;            /*!< USB Endpoint 1 register,                Address offset: 0x04 */
  __IO uint16_t RESERVED1;       /*!< Reserved */       
  __IO uint16_t EP2R;            /*!< USB Endpoint 2 register,                Address offset: 0x08 */
  __IO uint16_t RESERVED2;       /*!< Reserved */       
  __IO uint16_t EP3R;            /*!< USB Endpoint 3 register,                Address offset: 0x0C */ 
  __IO uint16_t RESERVED3;       /*!< Reserved */       
  __IO uint16_t EP4R;            /*!< USB Endpoint 4 register,                Address offset: 0x10 */
  __IO uint16_t RESERVED4;       /*!< Reserved */       
  __IO uint16_t EP5R;            /*!< USB Endpoint 5 register,                Address offset: 0x14 */
  __IO uint16_t RESERVED5;       /*!< Reserved */       
  __IO uint16_t EP6R;            /*!< USB Endpoint 6 register,                Address offset: 0x18 */
  __IO uint16_t RESERVED6;       /*!< Reserved */       
  __IO uint16_t EP7R;            /*!< USB Endpoint 7 register,                Address offset: 0x1C */
  __IO uint16_t RESERVED7[17];   /*!< Reserved */     
  __IO uint16_t CNTR;            /*!< Control register,                       Address offset: 0x40 */
  __IO uint16_t RESERVED8;       /*!< Reserved */       
  __IO uint16_t ISTR;            /*!< Interrupt status register,              Address offset: 0x44 */
  __IO uint16_t RESERVED9;       /*!< Reserved */       
  __IO uint16_t FNR;             /*!< Frame number register,                  Address offset: 0x48 */
  __IO uint16_t RESERVEDA;       /*!< Reserved */       
  __IO uint16_t DADDR;           /*!< Device address register,                Address offset: 0x4C */
  __IO uint16_t RESERVEDB;       /*!< Reserved */       
  __IO uint16_t BTABLE;          /*!< Buffer Table address register,          Address offset: 0x50 */
  __IO uint16_t RESERVEDC;       /*!< Reserved */       
  __IO uint16_t LPMCSR;          /*!< LPM Control and Status register,        Address offset: 0x54 */
  __IO uint16_t RESERVEDD;       /*!< Reserved */       
  __IO uint16_t BCDR;            /*!< Battery Charging detector register,     Address offset: 0x58 */
  __IO uint16_t RESERVEDE;       /*!< Reserved */       
} USB_TypeDef;

typedef struct
{
  __IO uint32_t CR1;          /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;         /*!< TIM slave Mode Control register,     Address offset: 0x08 */
  __IO uint32_t DIER;         /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;           /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;          /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;        /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;        /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;         /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;          /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;          /*!< TIM prescaler register,              Address offset: 0x28 */
  __IO uint32_t ARR;          /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
  __IO uint32_t CCR1;         /*!< TIM capture/compare register 1,      Address offset: 0x34 */    
  __IO uint32_t CCR2;         /*!< TIM capture/compare register 2,      Address offset: 0x38 */    
  __IO uint32_t CCR3;         /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;         /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
  __IO uint32_t DCR;          /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
  __IO uint32_t OR;           /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

typedef struct
{
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                           Address offset: 0x00 */
       uint32_t RESERVED;    /*!< Reserved,                                                                  0x04 */
  __IO uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration register,     Address offset: 0x14-0x08 */
  __IO uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                           Address offset: 0x18 */
} SYSCFG_TypeDef;

typedef struct
{
  __IO uint32_t ACR;          /*!<FLASH access control register,                 Address offset: 0x00 */
  __IO uint32_t KEYR;         /*!<FLASH key register,                            Address offset: 0x04 */
  __IO uint32_t OPTKEYR;      /*!<FLASH OPT key register,                        Address offset: 0x08 */
  __IO uint32_t SR;           /*!<FLASH status register,                         Address offset: 0x0C */
  __IO uint32_t CR;           /*!<FLASH control register,                        Address offset: 0x10 */
  __IO uint32_t AR;           /*!<FLASH address register,                        Address offset: 0x14 */
  __IO uint32_t RESERVED;     /*!< Reserved,                                                     0x18 */
  __IO uint32_t OBR;          /*!<FLASH option bytes register,                   Address offset: 0x1C */
  __IO uint32_t WRPR;         /*!<FLASH option bytes register,                   Address offset: 0x20 */
} FLASH_TypeDef;

#define PERIPH_BASE			    (0x40000000U)
#define APBPERIPH_BASE      PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE     (PERIPH_BASE + 0x08000000)

#define TIM2_BASE			      (APBPERIPH_BASE + 0x0000)
#define TIM3_BASE			      (APBPERIPH_BASE + 0x0400)
#define IWDG_BASE			      (APBPERIPH_BASE + 0x3000)
#define SPI2_BASE			      (APBPERIPH_BASE + 0x3800)
#define I2C1_BASE			      (APBPERIPH_BASE + 0x5400)
#define USB_BASE            (APBPERIPH_BASE + 0x5C00)
#define USB_PMAADDR 			  (APBPERIPH_BASE + 0x6000)

#define SYSCFG_BASE		      (APB2PERIPH_BASE + 0x0000)
#define ADC1_BASE		    	  (APB2PERIPH_BASE + 0x2400)
#define ADC_BASE            (APB2PERIPH_BASE + 0x2708)
#define SPI1_BASE			      (APB2PERIPH_BASE + 0x3000)

#define RCC_BASE			      (AHBPERIPH_BASE + 0x01000)
#define FLASH_R_BASE        (AHBPERIPH_BASE + 0x02000) 

#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000800)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400)

#define UID_BASE		        (0x1ffff7acU)

#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define USB                 ((USB_TypeDef *) USB_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)

#define SYSCFG_CFGR1_MEM_MODE_Pos            (0U)                              

#define RCC_CR_HSEON_Pos                         (16U)                         
#define RCC_CR_HSEON_Msk                         (0x1U << RCC_CR_HSEON_Pos)    /*!< 0x00010000 */
#define RCC_CR_HSEON                             RCC_CR_HSEON_Msk              /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos                        (17U)                         
#define RCC_CR_HSERDY_Msk                        (0x1U << RCC_CR_HSERDY_Pos)   /*!< 0x00020000 */
#define RCC_CR_HSERDY                            RCC_CR_HSERDY_Msk             /*!< External High Speed clock ready flag */
#define RCC_CR_PLLON_Pos                         (24U)                         
#define RCC_CR_PLLON_Msk                         (0x1U << RCC_CR_PLLON_Pos)    /*!< 0x01000000 */
#define RCC_CR_PLLON                             RCC_CR_PLLON_Msk              /*!< PLL enable */
#define RCC_CR_PLLRDY_Pos                        (25U)                         
#define RCC_CR_PLLRDY_Msk                        (0x1U << RCC_CR_PLLRDY_Pos)   /*!< 0x02000000 */
#define RCC_CR_PLLRDY                            RCC_CR_PLLRDY_Msk             /*!< PLL clock ready flag */

#define RCC_CSR_RMVF_Pos                         (24U)                         
#define RCC_CSR_RMVF_Msk                         (0x1U << RCC_CSR_RMVF_Pos)    /*!< 0x01000000 */
#define RCC_CSR_RMVF                             RCC_CSR_RMVF_Msk              /*!< Remove reset flag */

// #define RCC_CFGR_PLLSRC_Pos                      (16U)                         
// #define RCC_CFGR_PLLSRC_Msk                      (0x1U << RCC_CFGR_PLLSRC_Pos) /*!< 0x00010000 */
// #define RCC_CFGR_PLLSRC                          RCC_CFGR_PLLSRC_Msk           /*!< PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSI_DIV2                 (0x00000000U)                 /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSE_PREDIV               (0x00010000U)                 /*!< HSE/PREDIV clock selected as PLL entry clock source */

// #define RCC_CFGR_PLLXTPRE_Pos                    (17U)                         
// #define RCC_CFGR_PLLXTPRE_Msk                    (0x1U << RCC_CFGR_PLLXTPRE_Pos) /*!< 0x00020000 */
// #define RCC_CFGR_PLLXTPRE                        RCC_CFGR_PLLXTPRE_Msk         /*!< HSE divider for PLL entry */
// #define RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1        (0x00000000U)                 /*!< HSE/PREDIV clock not divided for PLL entry */
// #define RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2        (0x00020000U)                 /*!< HSE/PREDIV clock divided by 2 for PLL entry */

#define RCC_CFGR_PLLMUL_Pos                      (18U)                         
#define RCC_CFGR_PLLMUL_Msk                      (0xFU << RCC_CFGR_PLLMUL_Pos) /*!< 0x003C0000 */
#define RCC_CFGR_PLLMUL                          RCC_CFGR_PLLMUL_Msk           /*!< PLLMUL[3:0] bits (PLL multiplication factor) */

#define RCC_CFGR_SWS_Pos                         (2U)                          
#define RCC_CFGR_SWS_Msk                         (0x3U << RCC_CFGR_SWS_Pos)    /*!< 0x0000000C */
#define RCC_CFGR_SWS                             RCC_CFGR_SWS_Msk              /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_HSI                         (0x00000000U)                 /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                         (0x00000004U)                 /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                         (0x00000008U)                 /*!< PLL used as system clock */

#define RCC_CFGR_SW_HSI                          (0x00000000U)                 /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                          (0x00000001U)                 /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                          (0x00000002U)                 /*!< PLL selected as system clock */

#define RCC_CFGR3_I2C1SW_Pos                     (4U)                          
#define RCC_CFGR3_I2C1SW_Msk                     (0x1U << RCC_CFGR3_I2C1SW_Pos) /*!< 0x00000010 */
#define RCC_CFGR3_I2C1SW                         RCC_CFGR3_I2C1SW_Msk          /*!< I2C1SW bits */ 
#define RCC_CFGR3_USBSW_Pos                      (7U)                          
#define RCC_CFGR3_USBSW_Msk                      (0x1U << RCC_CFGR3_USBSW_Pos) /*!< 0x00000080 */
#define RCC_CFGR3_USBSW                          RCC_CFGR3_USBSW_Msk           /*!< USBSW bits */

#define USB_EPADDR_FIELD                     ((uint16_t)0x000FU)               /*!<  EndPoint ADDRess FIELD */
#define USB_CNTR_FRES                        ((uint16_t)0x0001U)               /*!< Force USB RESet */
#define USB_CNTR_RESETM                      ((uint16_t)0x0400U)               /*!< RESET Mask   */
#define USB_CNTR_CTRM                        ((uint16_t)0x8000U)               /*!< Correct TRansfer Mask */
#define USB_ISTR_RESET                       ((uint16_t)0x0400U)               /*!< RESET (clear-only bit) */
#define USB_ISTR_EP_ID                       ((uint16_t)0x000FU)               /*!< EndPoint IDentifier (read-only bit)  */
#define USB_ISTR_CTR                         ((uint16_t)0x8000U)               /*!< Correct TRansfer (clear-only bit) */
#define USB_EPTX_STAT                        ((uint16_t)0x0030U)               /*!<  EndPoint TX STATus bit field */
#define USB_EPRX_STAT                        ((uint16_t)0x3000U)               /*!<  EndPoint RX STATus bit field */
#define USB_EP_CTR_TX                        ((uint16_t)0x0080U)               /*!<  EndPoint Correct TRansfer TX */
#define USB_EP_KIND                          ((uint16_t)0x0100U)               /*!<  EndPoint KIND */
#define USB_EP_CTR_RX                        ((uint16_t)0x8000U)               /*!<  EndPoint Correct TRansfer RX */
#define USB_EP_TX_STALL                      ((uint16_t)0x0010U)               /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK                        ((uint16_t)0x0020U)               /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID                      ((uint16_t)0x0030U)               /*!< EndPoint TX VALID */
#define USB_EP_RX_STALL                      ((uint16_t)0x1000U)               /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK                        ((uint16_t)0x2000U)               /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID                      ((uint16_t)0x3000U)               /*!< EndPoint RX VALID */
#define USB_EP_INTERRUPT                     ((uint16_t)0x0600U)               /*!< EndPoint INTERRUPT */

#define USB_EP_TYPE_MASK                     ((uint16_t)0x0600U)               /*!< EndPoint TYPE Mask */
#define USB_EP_BULK                          ((uint16_t)0x0000U)               /*!< EndPoint BULK */
#define USB_EP_CONTROL                       ((uint16_t)0x0200U)               /*!< EndPoint CONTROL */
#define USB_DADDR_EF                         ((uint8_t)0x80U)                  /*!< USB device address Enable Function */


#define TIM_DIER_CC1IE_Pos        (1U)                                         
#define TIM_DIER_CC1IE_Msk        (0x1U << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable */
#define TIM_CCER_CC1E_Pos         (0U)                                         
#define TIM_CCER_CC1E_Msk         (0x1U << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable */
#define TIM_CR1_CEN_Pos           (0U)                                         
#define TIM_CR1_CEN_Msk           (0x1U << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable */


#define FLASH_ACR_LATENCY_Pos             (0U)                                 
#define FLASH_ACR_LATENCY_Msk             (0x3U << FLASH_ACR_LATENCY_Pos)      /*!< 0x00000001 */
#define FLASH_ACR_LATENCY                 FLASH_ACR_LATENCY_Msk                /*!< LATENCY bit (Latency) */


/*****************************************************************************/
/*                                                                           */
/*                        Serial Peripheral Interface (SPI)                  */
/*                                                                           */
/*****************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F0 serie)
 */
/* Note: No specific macro feature on this device */

/*******************  Bit definition for SPI_CR1 register  *******************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA_Msk            (0x1U << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!< Clock Phase */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL_Msk            (0x1U << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!< Clock Polarity */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR_Msk            (0x1U << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!< Master Selection */
#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR_Msk              (0x7U << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1U << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2U << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4U << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */
#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE_Msk             (0x1U << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!< SPI Enable */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST_Msk        (0x1U << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!< Frame Format */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI_Msk             (0x1U << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!< Internal slave select */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM_Msk             (0x1U << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!< Software slave management */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY_Msk          (0x1U << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!< Receive only */
#define SPI_CR1_CRCL_Pos            (11U)                                      
#define SPI_CR1_CRCL_Msk            (0x1U << SPI_CR1_CRCL_Pos)                 /*!< 0x00000800 */
#define SPI_CR1_CRCL                SPI_CR1_CRCL_Msk                           /*!< CRC Length */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT_Msk         (0x1U << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!< Transmit CRC next */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN_Msk           (0x1U << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE_Msk          (0x1U << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE_Msk        (0x1U << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  *******************/
#define SPI_CR2_RXDMAEN_Pos         (0U)                                       
#define SPI_CR2_RXDMAEN_Msk         (0x1U << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN_Pos         (1U)                                       
#define SPI_CR2_TXDMAEN_Msk         (0x1U << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE_Msk            (0x1U << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!< SS Output Enable */
#define SPI_CR2_NSSP_Pos            (3U)                                       
#define SPI_CR2_NSSP_Msk            (0x1U << SPI_CR2_NSSP_Pos)                 /*!< 0x00000008 */
#define SPI_CR2_NSSP                SPI_CR2_NSSP_Msk                           /*!< NSS pulse management Enable */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF_Msk             (0x1U << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE_Msk           (0x1U << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE_Msk          (0x1U << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE_Msk           (0x1U << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!< Tx buffer Empty Interrupt Enable */
#define SPI_CR2_DS_Pos              (8U)                                       
#define SPI_CR2_DS_Msk              (0xFU << SPI_CR2_DS_Pos)                   /*!< 0x00000F00 */
#define SPI_CR2_DS                  SPI_CR2_DS_Msk                             /*!< DS[3:0] Data Size */
#define SPI_CR2_DS_0                (0x1U << SPI_CR2_DS_Pos)                   /*!< 0x00000100 */
#define SPI_CR2_DS_1                (0x2U << SPI_CR2_DS_Pos)                   /*!< 0x00000200 */
#define SPI_CR2_DS_2                (0x4U << SPI_CR2_DS_Pos)                   /*!< 0x00000400 */
#define SPI_CR2_DS_3                (0x8U << SPI_CR2_DS_Pos)                   /*!< 0x00000800 */
#define SPI_CR2_FRXTH_Pos           (12U)                                      
#define SPI_CR2_FRXTH_Msk           (0x1U << SPI_CR2_FRXTH_Pos)                /*!< 0x00001000 */
#define SPI_CR2_FRXTH               SPI_CR2_FRXTH_Msk                          /*!< FIFO reception Threshold */
#define SPI_CR2_LDMARX_Pos          (13U)                                      
#define SPI_CR2_LDMARX_Msk          (0x1U << SPI_CR2_LDMARX_Pos)               /*!< 0x00002000 */
#define SPI_CR2_LDMARX              SPI_CR2_LDMARX_Msk                         /*!< Last DMA transfer for reception */
#define SPI_CR2_LDMATX_Pos          (14U)                                      
#define SPI_CR2_LDMATX_Msk          (0x1U << SPI_CR2_LDMATX_Pos)               /*!< 0x00004000 */
#define SPI_CR2_LDMATX              SPI_CR2_LDMATX_Msk                         /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  *******************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE_Msk             (0x1U << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE_Msk              (0x1U << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!< Transmit buffer Empty */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR_Msk           (0x1U << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!< CRC Error flag */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF_Msk             (0x1U << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!< Mode fault */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR_Msk              (0x1U << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!< Overrun flag */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY_Msk              (0x1U << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!< Busy flag */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE_Msk              (0x1U << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!< TI frame format error */
#define SPI_SR_FRLVL_Pos            (9U)                                       
#define SPI_SR_FRLVL_Msk            (0x3U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000600 */
#define SPI_SR_FRLVL                SPI_SR_FRLVL_Msk                           /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0              (0x1U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000200 */
#define SPI_SR_FRLVL_1              (0x2U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000400 */
#define SPI_SR_FTLVL_Pos            (11U)                                      
#define SPI_SR_FTLVL_Msk            (0x3U << SPI_SR_FTLVL_Pos)                 /*!< 0x00001800 */
#define SPI_SR_FTLVL                SPI_SR_FTLVL_Msk                           /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0              (0x1U << SPI_SR_FTLVL_Pos)                 /*!< 0x00000800 */
#define SPI_SR_FTLVL_1              (0x2U << SPI_SR_FTLVL_Pos)                 /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  *******************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR_Msk               (0xFFFFFFFFU << SPI_DR_DR_Pos)             /*!< 0xFFFFFFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  *****************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFFFFFU << SPI_CRCPR_CRCPOLY_Pos)     /*!< 0xFFFFFFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  *****************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFFFFFU << SPI_RXCRCR_RXCRC_Pos)      /*!< 0xFFFFFFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  *****************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFFFFFU << SPI_TXCRCR_TXCRC_Pos)      /*!< 0xFFFFFFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  ****************/
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)                                      
#define SPI_I2SCFGR_I2SMOD_Msk      (0x1U << SPI_I2SCFGR_I2SMOD_Pos)           /*!< 0x00000800 */
#define SPI_I2SCFGR_I2SMOD          SPI_I2SCFGR_I2SMOD_Msk                     /*!< Keep for compatibility */


/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                      Analog to Digital Converter (ADC)                     */
/*                                                                            */
/******************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F0 serie)
 */
/* Note: No specific macro feature on this device */

/********************  Bits definition for ADC_ISR register  ******************/
#define ADC_ISR_ADRDY_Pos         (0U)                                         
#define ADC_ISR_ADRDY_Msk         (0x1U << ADC_ISR_ADRDY_Pos)                  /*!< 0x00000001 */
#define ADC_ISR_ADRDY             ADC_ISR_ADRDY_Msk                            /*!< ADC ready flag */
#define ADC_ISR_EOSMP_Pos         (1U)                                         
#define ADC_ISR_EOSMP_Msk         (0x1U << ADC_ISR_EOSMP_Pos)                  /*!< 0x00000002 */
#define ADC_ISR_EOSMP             ADC_ISR_EOSMP_Msk                            /*!< ADC group regular end of sampling flag */
#define ADC_ISR_EOC_Pos           (2U)                                         
#define ADC_ISR_EOC_Msk           (0x1U << ADC_ISR_EOC_Pos)                    /*!< 0x00000004 */
#define ADC_ISR_EOC               ADC_ISR_EOC_Msk                              /*!< ADC group regular end of unitary conversion flag */
#define ADC_ISR_EOS_Pos           (3U)                                         
#define ADC_ISR_EOS_Msk           (0x1U << ADC_ISR_EOS_Pos)                    /*!< 0x00000008 */
#define ADC_ISR_EOS               ADC_ISR_EOS_Msk                              /*!< ADC group regular end of sequence conversions flag */
#define ADC_ISR_OVR_Pos           (4U)                                         
#define ADC_ISR_OVR_Msk           (0x1U << ADC_ISR_OVR_Pos)                    /*!< 0x00000010 */
#define ADC_ISR_OVR               ADC_ISR_OVR_Msk                              /*!< ADC group regular overrun flag */
#define ADC_ISR_AWD1_Pos          (7U)                                         
#define ADC_ISR_AWD1_Msk          (0x1U << ADC_ISR_AWD1_Pos)                   /*!< 0x00000080 */
#define ADC_ISR_AWD1              ADC_ISR_AWD1_Msk                             /*!< ADC analog watchdog 1 flag */

/* Legacy defines */
#define ADC_ISR_AWD             (ADC_ISR_AWD1)
#define ADC_ISR_EOSEQ           (ADC_ISR_EOS)

/********************  Bits definition for ADC_IER register  ******************/
#define ADC_IER_ADRDYIE_Pos       (0U)                                         
#define ADC_IER_ADRDYIE_Msk       (0x1U << ADC_IER_ADRDYIE_Pos)                /*!< 0x00000001 */
#define ADC_IER_ADRDYIE           ADC_IER_ADRDYIE_Msk                          /*!< ADC ready interrupt */
#define ADC_IER_EOSMPIE_Pos       (1U)                                         
#define ADC_IER_EOSMPIE_Msk       (0x1U << ADC_IER_EOSMPIE_Pos)                /*!< 0x00000002 */
#define ADC_IER_EOSMPIE           ADC_IER_EOSMPIE_Msk                          /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOCIE_Pos         (2U)                                         
#define ADC_IER_EOCIE_Msk         (0x1U << ADC_IER_EOCIE_Pos)                  /*!< 0x00000004 */
#define ADC_IER_EOCIE             ADC_IER_EOCIE_Msk                            /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EOSIE_Pos         (3U)                                         
#define ADC_IER_EOSIE_Msk         (0x1U << ADC_IER_EOSIE_Pos)                  /*!< 0x00000008 */
#define ADC_IER_EOSIE             ADC_IER_EOSIE_Msk                            /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_OVRIE_Pos         (4U)                                         
#define ADC_IER_OVRIE_Msk         (0x1U << ADC_IER_OVRIE_Pos)                  /*!< 0x00000010 */
#define ADC_IER_OVRIE             ADC_IER_OVRIE_Msk                            /*!< ADC group regular overrun interrupt */
#define ADC_IER_AWD1IE_Pos        (7U)                                         
#define ADC_IER_AWD1IE_Msk        (0x1U << ADC_IER_AWD1IE_Pos)                 /*!< 0x00000080 */
#define ADC_IER_AWD1IE            ADC_IER_AWD1IE_Msk                           /*!< ADC analog watchdog 1 interrupt */

/* Legacy defines */
#define ADC_IER_AWDIE           (ADC_IER_AWD1IE)
#define ADC_IER_EOSEQIE         (ADC_IER_EOSIE)

/********************  Bits definition for ADC_CR register  *******************/
#define ADC_CR_ADEN_Pos           (0U)                                         
#define ADC_CR_ADEN_Msk           (0x1U << ADC_CR_ADEN_Pos)                    /*!< 0x00000001 */
#define ADC_CR_ADEN               ADC_CR_ADEN_Msk                              /*!< ADC enable */
#define ADC_CR_ADDIS_Pos          (1U)                                         
#define ADC_CR_ADDIS_Msk          (0x1U << ADC_CR_ADDIS_Pos)                   /*!< 0x00000002 */
#define ADC_CR_ADDIS              ADC_CR_ADDIS_Msk                             /*!< ADC disable */
#define ADC_CR_ADSTART_Pos        (2U)                                         
#define ADC_CR_ADSTART_Msk        (0x1U << ADC_CR_ADSTART_Pos)                 /*!< 0x00000004 */
#define ADC_CR_ADSTART            ADC_CR_ADSTART_Msk                           /*!< ADC group regular conversion start */
#define ADC_CR_ADSTP_Pos          (4U)                                         
#define ADC_CR_ADSTP_Msk          (0x1U << ADC_CR_ADSTP_Pos)                   /*!< 0x00000010 */
#define ADC_CR_ADSTP              ADC_CR_ADSTP_Msk                             /*!< ADC group regular conversion stop */
#define ADC_CR_ADCAL_Pos          (31U)                                        
#define ADC_CR_ADCAL_Msk          (0x1U << ADC_CR_ADCAL_Pos)                   /*!< 0x80000000 */
#define ADC_CR_ADCAL              ADC_CR_ADCAL_Msk                             /*!< ADC calibration */

/*******************  Bits definition for ADC_CFGR1 register  *****************/
#define ADC_CFGR1_DMAEN_Pos       (0U)                                         
#define ADC_CFGR1_DMAEN_Msk       (0x1U << ADC_CFGR1_DMAEN_Pos)                /*!< 0x00000001 */
#define ADC_CFGR1_DMAEN           ADC_CFGR1_DMAEN_Msk                          /*!< ADC DMA transfer enable */
#define ADC_CFGR1_DMACFG_Pos      (1U)                                         
#define ADC_CFGR1_DMACFG_Msk      (0x1U << ADC_CFGR1_DMACFG_Pos)               /*!< 0x00000002 */
#define ADC_CFGR1_DMACFG          ADC_CFGR1_DMACFG_Msk                         /*!< ADC DMA transfer configuration */
#define ADC_CFGR1_SCANDIR_Pos     (2U)                                         
#define ADC_CFGR1_SCANDIR_Msk     (0x1U << ADC_CFGR1_SCANDIR_Pos)              /*!< 0x00000004 */
#define ADC_CFGR1_SCANDIR         ADC_CFGR1_SCANDIR_Msk                        /*!< ADC group regular sequencer scan direction */

#define ADC_CFGR1_RES_Pos         (3U)                                         
#define ADC_CFGR1_RES_Msk         (0x3U << ADC_CFGR1_RES_Pos)                  /*!< 0x00000018 */
#define ADC_CFGR1_RES             ADC_CFGR1_RES_Msk                            /*!< ADC data resolution */
#define ADC_CFGR1_RES_0           (0x1U << ADC_CFGR1_RES_Pos)                  /*!< 0x00000008 */
#define ADC_CFGR1_RES_1           (0x2U << ADC_CFGR1_RES_Pos)                  /*!< 0x00000010 */

#define ADC_CFGR1_ALIGN_Pos       (5U)                                         
#define ADC_CFGR1_ALIGN_Msk       (0x1U << ADC_CFGR1_ALIGN_Pos)                /*!< 0x00000020 */
#define ADC_CFGR1_ALIGN           ADC_CFGR1_ALIGN_Msk                          /*!< ADC data alignement */

#define ADC_CFGR1_EXTSEL_Pos      (6U)                                         
#define ADC_CFGR1_EXTSEL_Msk      (0x7U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x000001C0 */
#define ADC_CFGR1_EXTSEL          ADC_CFGR1_EXTSEL_Msk                         /*!< ADC group regular external trigger source */
#define ADC_CFGR1_EXTSEL_0        (0x1U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000040 */
#define ADC_CFGR1_EXTSEL_1        (0x2U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000080 */
#define ADC_CFGR1_EXTSEL_2        (0x4U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000100 */

#define ADC_CFGR1_EXTEN_Pos       (10U)                                        
#define ADC_CFGR1_EXTEN_Msk       (0x3U << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000C00 */
#define ADC_CFGR1_EXTEN           ADC_CFGR1_EXTEN_Msk                          /*!< ADC group regular external trigger polarity */
#define ADC_CFGR1_EXTEN_0         (0x1U << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000400 */
#define ADC_CFGR1_EXTEN_1         (0x2U << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000800 */

#define ADC_CFGR1_OVRMOD_Pos      (12U)                                        
#define ADC_CFGR1_OVRMOD_Msk      (0x1U << ADC_CFGR1_OVRMOD_Pos)               /*!< 0x00001000 */
#define ADC_CFGR1_OVRMOD          ADC_CFGR1_OVRMOD_Msk                         /*!< ADC group regular overrun configuration */
#define ADC_CFGR1_CONT_Pos        (13U)                                        
#define ADC_CFGR1_CONT_Msk        (0x1U << ADC_CFGR1_CONT_Pos)                 /*!< 0x00002000 */
#define ADC_CFGR1_CONT            ADC_CFGR1_CONT_Msk                           /*!< ADC group regular continuous conversion mode */
#define ADC_CFGR1_WAIT_Pos        (14U)                                        
#define ADC_CFGR1_WAIT_Msk        (0x1U << ADC_CFGR1_WAIT_Pos)                 /*!< 0x00004000 */
#define ADC_CFGR1_WAIT            ADC_CFGR1_WAIT_Msk                           /*!< ADC low power auto wait */
#define ADC_CFGR1_AUTOFF_Pos      (15U)                                        
#define ADC_CFGR1_AUTOFF_Msk      (0x1U << ADC_CFGR1_AUTOFF_Pos)               /*!< 0x00008000 */
#define ADC_CFGR1_AUTOFF          ADC_CFGR1_AUTOFF_Msk                         /*!< ADC low power auto power off */
#define ADC_CFGR1_DISCEN_Pos      (16U)                                        
#define ADC_CFGR1_DISCEN_Msk      (0x1U << ADC_CFGR1_DISCEN_Pos)               /*!< 0x00010000 */
#define ADC_CFGR1_DISCEN          ADC_CFGR1_DISCEN_Msk                         /*!< ADC group regular sequencer discontinuous mode */

#define ADC_CFGR1_AWD1SGL_Pos     (22U)                                        
#define ADC_CFGR1_AWD1SGL_Msk     (0x1U << ADC_CFGR1_AWD1SGL_Pos)              /*!< 0x00400000 */
#define ADC_CFGR1_AWD1SGL         ADC_CFGR1_AWD1SGL_Msk                        /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CFGR1_AWD1EN_Pos      (23U)                                        
#define ADC_CFGR1_AWD1EN_Msk      (0x1U << ADC_CFGR1_AWD1EN_Pos)               /*!< 0x00800000 */
#define ADC_CFGR1_AWD1EN          ADC_CFGR1_AWD1EN_Msk                         /*!< ADC analog watchdog 1 enable on scope ADC group regular */

#define ADC_CFGR1_AWD1CH_Pos      (26U)                                        
#define ADC_CFGR1_AWD1CH_Msk      (0x1FU << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x7C000000 */
#define ADC_CFGR1_AWD1CH          ADC_CFGR1_AWD1CH_Msk                         /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR1_AWD1CH_0        (0x01U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x04000000 */
#define ADC_CFGR1_AWD1CH_1        (0x02U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x08000000 */
#define ADC_CFGR1_AWD1CH_2        (0x04U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x10000000 */
#define ADC_CFGR1_AWD1CH_3        (0x08U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x20000000 */
#define ADC_CFGR1_AWD1CH_4        (0x10U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x40000000 */

/* Legacy defines */
#define ADC_CFGR1_AUTDLY        (ADC_CFGR1_WAIT)
#define ADC_CFGR1_AWDSGL        (ADC_CFGR1_AWD1SGL)
#define ADC_CFGR1_AWDEN         (ADC_CFGR1_AWD1EN)
#define ADC_CFGR1_AWDCH         (ADC_CFGR1_AWD1CH)
#define ADC_CFGR1_AWDCH_0       (ADC_CFGR1_AWD1CH_0)
#define ADC_CFGR1_AWDCH_1       (ADC_CFGR1_AWD1CH_1)
#define ADC_CFGR1_AWDCH_2       (ADC_CFGR1_AWD1CH_2)
#define ADC_CFGR1_AWDCH_3       (ADC_CFGR1_AWD1CH_3)
#define ADC_CFGR1_AWDCH_4       (ADC_CFGR1_AWD1CH_4)

/*******************  Bits definition for ADC_CFGR2 register  *****************/
#define ADC_CFGR2_CKMODE_Pos      (30U)                                        
#define ADC_CFGR2_CKMODE_Msk      (0x3U << ADC_CFGR2_CKMODE_Pos)               /*!< 0xC0000000 */
#define ADC_CFGR2_CKMODE          ADC_CFGR2_CKMODE_Msk                         /*!< ADC clock source and prescaler (prescaler only for clock source synchronous) */
#define ADC_CFGR2_CKMODE_1        (0x2U << ADC_CFGR2_CKMODE_Pos)               /*!< 0x80000000 */
#define ADC_CFGR2_CKMODE_0        (0x1U << ADC_CFGR2_CKMODE_Pos)               /*!< 0x40000000 */

/* Legacy defines */
#define  ADC_CFGR2_JITOFFDIV4   (ADC_CFGR2_CKMODE_1)   /*!< ADC clocked by PCLK div4 */
#define  ADC_CFGR2_JITOFFDIV2   (ADC_CFGR2_CKMODE_0)   /*!< ADC clocked by PCLK div2 */

/******************  Bit definition for ADC_SMPR register  ********************/
#define ADC_SMPR_SMP_Pos          (0U)                                         
#define ADC_SMPR_SMP_Msk          (0x7U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000007 */
#define ADC_SMPR_SMP              ADC_SMPR_SMP_Msk                             /*!< ADC group of channels sampling time 2 */
#define ADC_SMPR_SMP_0            (0x1U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000001 */
#define ADC_SMPR_SMP_1            (0x2U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000002 */
#define ADC_SMPR_SMP_2            (0x4U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000004 */

/* Legacy defines */
#define  ADC_SMPR1_SMPR         (ADC_SMPR_SMP)         /*!< SMP[2:0] bits (Sampling time selection) */
#define  ADC_SMPR1_SMPR_0       (ADC_SMPR_SMP_0)       /*!< bit 0 */
#define  ADC_SMPR1_SMPR_1       (ADC_SMPR_SMP_1)       /*!< bit 1 */
#define  ADC_SMPR1_SMPR_2       (ADC_SMPR_SMP_2)       /*!< bit 2 */

/*******************  Bit definition for ADC_TR register  ********************/
#define ADC_TR1_LT1_Pos           (0U)                                         
#define ADC_TR1_LT1_Msk           (0xFFFU << ADC_TR1_LT1_Pos)                  /*!< 0x00000FFF */
#define ADC_TR1_LT1               ADC_TR1_LT1_Msk                              /*!< ADC analog watchdog 1 threshold low */
#define ADC_TR1_LT1_0             (0x001U << ADC_TR1_LT1_Pos)                  /*!< 0x00000001 */
#define ADC_TR1_LT1_1             (0x002U << ADC_TR1_LT1_Pos)                  /*!< 0x00000002 */
#define ADC_TR1_LT1_2             (0x004U << ADC_TR1_LT1_Pos)                  /*!< 0x00000004 */
#define ADC_TR1_LT1_3             (0x008U << ADC_TR1_LT1_Pos)                  /*!< 0x00000008 */
#define ADC_TR1_LT1_4             (0x010U << ADC_TR1_LT1_Pos)                  /*!< 0x00000010 */
#define ADC_TR1_LT1_5             (0x020U << ADC_TR1_LT1_Pos)                  /*!< 0x00000020 */
#define ADC_TR1_LT1_6             (0x040U << ADC_TR1_LT1_Pos)                  /*!< 0x00000040 */
#define ADC_TR1_LT1_7             (0x080U << ADC_TR1_LT1_Pos)                  /*!< 0x00000080 */
#define ADC_TR1_LT1_8             (0x100U << ADC_TR1_LT1_Pos)                  /*!< 0x00000100 */
#define ADC_TR1_LT1_9             (0x200U << ADC_TR1_LT1_Pos)                  /*!< 0x00000200 */
#define ADC_TR1_LT1_10            (0x400U << ADC_TR1_LT1_Pos)                  /*!< 0x00000400 */
#define ADC_TR1_LT1_11            (0x800U << ADC_TR1_LT1_Pos)                  /*!< 0x00000800 */

#define ADC_TR1_HT1_Pos           (16U)                                        
#define ADC_TR1_HT1_Msk           (0xFFFU << ADC_TR1_HT1_Pos)                  /*!< 0x0FFF0000 */
#define ADC_TR1_HT1               ADC_TR1_HT1_Msk                              /*!< ADC Analog watchdog 1 threshold high */
#define ADC_TR1_HT1_0             (0x001U << ADC_TR1_HT1_Pos)                  /*!< 0x00010000 */
#define ADC_TR1_HT1_1             (0x002U << ADC_TR1_HT1_Pos)                  /*!< 0x00020000 */
#define ADC_TR1_HT1_2             (0x004U << ADC_TR1_HT1_Pos)                  /*!< 0x00040000 */
#define ADC_TR1_HT1_3             (0x008U << ADC_TR1_HT1_Pos)                  /*!< 0x00080000 */
#define ADC_TR1_HT1_4             (0x010U << ADC_TR1_HT1_Pos)                  /*!< 0x00100000 */
#define ADC_TR1_HT1_5             (0x020U << ADC_TR1_HT1_Pos)                  /*!< 0x00200000 */
#define ADC_TR1_HT1_6             (0x040U << ADC_TR1_HT1_Pos)                  /*!< 0x00400000 */
#define ADC_TR1_HT1_7             (0x080U << ADC_TR1_HT1_Pos)                  /*!< 0x00800000 */
#define ADC_TR1_HT1_8             (0x100U << ADC_TR1_HT1_Pos)                  /*!< 0x01000000 */
#define ADC_TR1_HT1_9             (0x200U << ADC_TR1_HT1_Pos)                  /*!< 0x02000000 */
#define ADC_TR1_HT1_10            (0x400U << ADC_TR1_HT1_Pos)                  /*!< 0x04000000 */
#define ADC_TR1_HT1_11            (0x800U << ADC_TR1_HT1_Pos)                  /*!< 0x08000000 */

/* Legacy defines */
#define  ADC_TR_HT              (ADC_TR1_HT1)
#define  ADC_TR_LT              (ADC_TR1_LT1)
#define  ADC_HTR_HT             (ADC_TR1_HT1)
#define  ADC_LTR_LT             (ADC_TR1_LT1)

/******************  Bit definition for ADC_CHSELR register  ******************/
#define ADC_CHSELR_CHSEL_Pos      (0U)                                         
#define ADC_CHSELR_CHSEL_Msk      (0x7FFFFU << ADC_CHSELR_CHSEL_Pos)           /*!< 0x0007FFFF */
#define ADC_CHSELR_CHSEL          ADC_CHSELR_CHSEL_Msk                         /*!< ADC group regular sequencer channels, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL18_Pos    (18U)                                        
#define ADC_CHSELR_CHSEL18_Msk    (0x1U << ADC_CHSELR_CHSEL18_Pos)             /*!< 0x00040000 */
#define ADC_CHSELR_CHSEL18        ADC_CHSELR_CHSEL18_Msk                       /*!< ADC group regular sequencer channel 18, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL17_Pos    (17U)                                        
#define ADC_CHSELR_CHSEL17_Msk    (0x1U << ADC_CHSELR_CHSEL17_Pos)             /*!< 0x00020000 */
#define ADC_CHSELR_CHSEL17        ADC_CHSELR_CHSEL17_Msk                       /*!< ADC group regular sequencer channel 17, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL16_Pos    (16U)                                        
#define ADC_CHSELR_CHSEL16_Msk    (0x1U << ADC_CHSELR_CHSEL16_Pos)             /*!< 0x00010000 */
#define ADC_CHSELR_CHSEL16        ADC_CHSELR_CHSEL16_Msk                       /*!< ADC group regular sequencer channel 16, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL15_Pos    (15U)                                        
#define ADC_CHSELR_CHSEL15_Msk    (0x1U << ADC_CHSELR_CHSEL15_Pos)             /*!< 0x00008000 */
#define ADC_CHSELR_CHSEL15        ADC_CHSELR_CHSEL15_Msk                       /*!< ADC group regular sequencer channel 15, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL14_Pos    (14U)                                        
#define ADC_CHSELR_CHSEL14_Msk    (0x1U << ADC_CHSELR_CHSEL14_Pos)             /*!< 0x00004000 */
#define ADC_CHSELR_CHSEL14        ADC_CHSELR_CHSEL14_Msk                       /*!< ADC group regular sequencer channel 14, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL13_Pos    (13U)                                        
#define ADC_CHSELR_CHSEL13_Msk    (0x1U << ADC_CHSELR_CHSEL13_Pos)             /*!< 0x00002000 */
#define ADC_CHSELR_CHSEL13        ADC_CHSELR_CHSEL13_Msk                       /*!< ADC group regular sequencer channel 13, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL12_Pos    (12U)                                        
#define ADC_CHSELR_CHSEL12_Msk    (0x1U << ADC_CHSELR_CHSEL12_Pos)             /*!< 0x00001000 */
#define ADC_CHSELR_CHSEL12        ADC_CHSELR_CHSEL12_Msk                       /*!< ADC group regular sequencer channel 12, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL11_Pos    (11U)                                        
#define ADC_CHSELR_CHSEL11_Msk    (0x1U << ADC_CHSELR_CHSEL11_Pos)             /*!< 0x00000800 */
#define ADC_CHSELR_CHSEL11        ADC_CHSELR_CHSEL11_Msk                       /*!< ADC group regular sequencer channel 11, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL10_Pos    (10U)                                        
#define ADC_CHSELR_CHSEL10_Msk    (0x1U << ADC_CHSELR_CHSEL10_Pos)             /*!< 0x00000400 */
#define ADC_CHSELR_CHSEL10        ADC_CHSELR_CHSEL10_Msk                       /*!< ADC group regular sequencer channel 10, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL9_Pos     (9U)                                         
#define ADC_CHSELR_CHSEL9_Msk     (0x1U << ADC_CHSELR_CHSEL9_Pos)              /*!< 0x00000200 */
#define ADC_CHSELR_CHSEL9         ADC_CHSELR_CHSEL9_Msk                        /*!< ADC group regular sequencer channel 9, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL8_Pos     (8U)                                         
#define ADC_CHSELR_CHSEL8_Msk     (0x1U << ADC_CHSELR_CHSEL8_Pos)              /*!< 0x00000100 */
#define ADC_CHSELR_CHSEL8         ADC_CHSELR_CHSEL8_Msk                        /*!< ADC group regular sequencer channel 8, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL7_Pos     (7U)                                         
#define ADC_CHSELR_CHSEL7_Msk     (0x1U << ADC_CHSELR_CHSEL7_Pos)              /*!< 0x00000080 */
#define ADC_CHSELR_CHSEL7         ADC_CHSELR_CHSEL7_Msk                        /*!< ADC group regular sequencer channel 7, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL6_Pos     (6U)                                         
#define ADC_CHSELR_CHSEL6_Msk     (0x1U << ADC_CHSELR_CHSEL6_Pos)              /*!< 0x00000040 */
#define ADC_CHSELR_CHSEL6         ADC_CHSELR_CHSEL6_Msk                        /*!< ADC group regular sequencer channel 6, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL5_Pos     (5U)                                         
#define ADC_CHSELR_CHSEL5_Msk     (0x1U << ADC_CHSELR_CHSEL5_Pos)              /*!< 0x00000020 */
#define ADC_CHSELR_CHSEL5         ADC_CHSELR_CHSEL5_Msk                        /*!< ADC group regular sequencer channel 5, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL4_Pos     (4U)                                         
#define ADC_CHSELR_CHSEL4_Msk     (0x1U << ADC_CHSELR_CHSEL4_Pos)              /*!< 0x00000010 */
#define ADC_CHSELR_CHSEL4         ADC_CHSELR_CHSEL4_Msk                        /*!< ADC group regular sequencer channel 4, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL3_Pos     (3U)                                         
#define ADC_CHSELR_CHSEL3_Msk     (0x1U << ADC_CHSELR_CHSEL3_Pos)              /*!< 0x00000008 */
#define ADC_CHSELR_CHSEL3         ADC_CHSELR_CHSEL3_Msk                        /*!< ADC group regular sequencer channel 3, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL2_Pos     (2U)                                         
#define ADC_CHSELR_CHSEL2_Msk     (0x1U << ADC_CHSELR_CHSEL2_Pos)              /*!< 0x00000004 */
#define ADC_CHSELR_CHSEL2         ADC_CHSELR_CHSEL2_Msk                        /*!< ADC group regular sequencer channel 2, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL1_Pos     (1U)                                         
#define ADC_CHSELR_CHSEL1_Msk     (0x1U << ADC_CHSELR_CHSEL1_Pos)              /*!< 0x00000002 */
#define ADC_CHSELR_CHSEL1         ADC_CHSELR_CHSEL1_Msk                        /*!< ADC group regular sequencer channel 1, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL0_Pos     (0U)                                         
#define ADC_CHSELR_CHSEL0_Msk     (0x1U << ADC_CHSELR_CHSEL0_Pos)              /*!< 0x00000001 */
#define ADC_CHSELR_CHSEL0         ADC_CHSELR_CHSEL0_Msk                        /*!< ADC group regular sequencer channel 0, available when ADC_CFGR1_CHSELRMOD is reset */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)                                         
#define ADC_DR_DATA_Msk           (0xFFFFU << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_DATA               ADC_DR_DATA_Msk                              /*!< ADC group regular conversion data */
#define ADC_DR_DATA_0             (0x0001U << ADC_DR_DATA_Pos)                 /*!< 0x00000001 */
#define ADC_DR_DATA_1             (0x0002U << ADC_DR_DATA_Pos)                 /*!< 0x00000002 */
#define ADC_DR_DATA_2             (0x0004U << ADC_DR_DATA_Pos)                 /*!< 0x00000004 */
#define ADC_DR_DATA_3             (0x0008U << ADC_DR_DATA_Pos)                 /*!< 0x00000008 */
#define ADC_DR_DATA_4             (0x0010U << ADC_DR_DATA_Pos)                 /*!< 0x00000010 */
#define ADC_DR_DATA_5             (0x0020U << ADC_DR_DATA_Pos)                 /*!< 0x00000020 */
#define ADC_DR_DATA_6             (0x0040U << ADC_DR_DATA_Pos)                 /*!< 0x00000040 */
#define ADC_DR_DATA_7             (0x0080U << ADC_DR_DATA_Pos)                 /*!< 0x00000080 */
#define ADC_DR_DATA_8             (0x0100U << ADC_DR_DATA_Pos)                 /*!< 0x00000100 */
#define ADC_DR_DATA_9             (0x0200U << ADC_DR_DATA_Pos)                 /*!< 0x00000200 */
#define ADC_DR_DATA_10            (0x0400U << ADC_DR_DATA_Pos)                 /*!< 0x00000400 */
#define ADC_DR_DATA_11            (0x0800U << ADC_DR_DATA_Pos)                 /*!< 0x00000800 */
#define ADC_DR_DATA_12            (0x1000U << ADC_DR_DATA_Pos)                 /*!< 0x00001000 */
#define ADC_DR_DATA_13            (0x2000U << ADC_DR_DATA_Pos)                 /*!< 0x00002000 */
#define ADC_DR_DATA_14            (0x4000U << ADC_DR_DATA_Pos)                 /*!< 0x00004000 */
#define ADC_DR_DATA_15            (0x8000U << ADC_DR_DATA_Pos)                 /*!< 0x00008000 */

/*************************  ADC Common registers  *****************************/
/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_VREFEN_Pos        (22U)                                        
#define ADC_CCR_VREFEN_Msk        (0x1U << ADC_CCR_VREFEN_Pos)                 /*!< 0x00400000 */
#define ADC_CCR_VREFEN            ADC_CCR_VREFEN_Msk                           /*!< ADC internal path to VrefInt enable */
#define ADC_CCR_TSEN_Pos          (23U)                                        
#define ADC_CCR_TSEN_Msk          (0x1U << ADC_CCR_TSEN_Pos)                   /*!< 0x00800000 */
#define ADC_CCR_TSEN              ADC_CCR_TSEN_Msk                             /*!< ADC internal path to temperature sensor enable */

/******************************************************************************/
/*                                                                            */
/*                   Inter-integrated Circuit Interface (I2C)                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  *******************/
#define I2C_CR1_PE_Pos               (0U)                                      
#define I2C_CR1_PE_Msk               (0x1U << I2C_CR1_PE_Pos)                  /*!< 0x00000001 */
#define I2C_CR1_PE                   I2C_CR1_PE_Msk                            /*!< Peripheral enable */
#define I2C_CR1_TXIE_Pos             (1U)                                      
#define I2C_CR1_TXIE_Msk             (0x1U << I2C_CR1_TXIE_Pos)                /*!< 0x00000002 */
#define I2C_CR1_TXIE                 I2C_CR1_TXIE_Msk                          /*!< TX interrupt enable */
#define I2C_CR1_RXIE_Pos             (2U)                                      
#define I2C_CR1_RXIE_Msk             (0x1U << I2C_CR1_RXIE_Pos)                /*!< 0x00000004 */
#define I2C_CR1_RXIE                 I2C_CR1_RXIE_Msk                          /*!< RX interrupt enable */
#define I2C_CR1_ADDRIE_Pos           (3U)                                      
#define I2C_CR1_ADDRIE_Msk           (0x1U << I2C_CR1_ADDRIE_Pos)              /*!< 0x00000008 */
#define I2C_CR1_ADDRIE               I2C_CR1_ADDRIE_Msk                        /*!< Address match interrupt enable */
#define I2C_CR1_NACKIE_Pos           (4U)                                      
#define I2C_CR1_NACKIE_Msk           (0x1U << I2C_CR1_NACKIE_Pos)              /*!< 0x00000010 */
#define I2C_CR1_NACKIE               I2C_CR1_NACKIE_Msk                        /*!< NACK received interrupt enable */
#define I2C_CR1_STOPIE_Pos           (5U)                                      
#define I2C_CR1_STOPIE_Msk           (0x1U << I2C_CR1_STOPIE_Pos)              /*!< 0x00000020 */
#define I2C_CR1_STOPIE               I2C_CR1_STOPIE_Msk                        /*!< STOP detection interrupt enable */
#define I2C_CR1_TCIE_Pos             (6U)                                      
#define I2C_CR1_TCIE_Msk             (0x1U << I2C_CR1_TCIE_Pos)                /*!< 0x00000040 */
#define I2C_CR1_TCIE                 I2C_CR1_TCIE_Msk                          /*!< Transfer complete interrupt enable */
#define I2C_CR1_ERRIE_Pos            (7U)                                      
#define I2C_CR1_ERRIE_Msk            (0x1U << I2C_CR1_ERRIE_Pos)               /*!< 0x00000080 */
#define I2C_CR1_ERRIE                I2C_CR1_ERRIE_Msk                         /*!< Errors interrupt enable */
#define I2C_CR1_DNF_Pos              (8U)                                      
#define I2C_CR1_DNF_Msk              (0xFU << I2C_CR1_DNF_Pos)                 /*!< 0x00000F00 */
#define I2C_CR1_DNF                  I2C_CR1_DNF_Msk                           /*!< Digital noise filter */
#define I2C_CR1_ANFOFF_Pos           (12U)                                     
#define I2C_CR1_ANFOFF_Msk           (0x1U << I2C_CR1_ANFOFF_Pos)              /*!< 0x00001000 */
#define I2C_CR1_ANFOFF               I2C_CR1_ANFOFF_Msk                        /*!< Analog noise filter OFF */
#define I2C_CR1_SWRST_Pos            (13U)                                     
#define I2C_CR1_SWRST_Msk            (0x1U << I2C_CR1_SWRST_Pos)               /*!< 0x00002000 */
#define I2C_CR1_SWRST                I2C_CR1_SWRST_Msk                         /*!< Software reset */
#define I2C_CR1_TXDMAEN_Pos          (14U)                                     
#define I2C_CR1_TXDMAEN_Msk          (0x1U << I2C_CR1_TXDMAEN_Pos)             /*!< 0x00004000 */
#define I2C_CR1_TXDMAEN              I2C_CR1_TXDMAEN_Msk                       /*!< DMA transmission requests enable */
#define I2C_CR1_RXDMAEN_Pos          (15U)                                     
#define I2C_CR1_RXDMAEN_Msk          (0x1U << I2C_CR1_RXDMAEN_Pos)             /*!< 0x00008000 */
#define I2C_CR1_RXDMAEN              I2C_CR1_RXDMAEN_Msk                       /*!< DMA reception requests enable */
#define I2C_CR1_SBC_Pos              (16U)                                     
#define I2C_CR1_SBC_Msk              (0x1U << I2C_CR1_SBC_Pos)                 /*!< 0x00010000 */
#define I2C_CR1_SBC                  I2C_CR1_SBC_Msk                           /*!< Slave byte control */
#define I2C_CR1_NOSTRETCH_Pos        (17U)                                     
#define I2C_CR1_NOSTRETCH_Msk        (0x1U << I2C_CR1_NOSTRETCH_Pos)           /*!< 0x00020000 */
#define I2C_CR1_NOSTRETCH            I2C_CR1_NOSTRETCH_Msk                     /*!< Clock stretching disable */
#define I2C_CR1_GCEN_Pos             (19U)                                     
#define I2C_CR1_GCEN_Msk             (0x1U << I2C_CR1_GCEN_Pos)                /*!< 0x00080000 */
#define I2C_CR1_GCEN                 I2C_CR1_GCEN_Msk                          /*!< General call enable */
#define I2C_CR1_SMBHEN_Pos           (20U)                                     
#define I2C_CR1_SMBHEN_Msk           (0x1U << I2C_CR1_SMBHEN_Pos)              /*!< 0x00100000 */
#define I2C_CR1_SMBHEN               I2C_CR1_SMBHEN_Msk                        /*!< SMBus host address enable */
#define I2C_CR1_SMBDEN_Pos           (21U)                                     
#define I2C_CR1_SMBDEN_Msk           (0x1U << I2C_CR1_SMBDEN_Pos)              /*!< 0x00200000 */
#define I2C_CR1_SMBDEN               I2C_CR1_SMBDEN_Msk                        /*!< SMBus device default address enable */
#define I2C_CR1_ALERTEN_Pos          (22U)                                     
#define I2C_CR1_ALERTEN_Msk          (0x1U << I2C_CR1_ALERTEN_Pos)             /*!< 0x00400000 */
#define I2C_CR1_ALERTEN              I2C_CR1_ALERTEN_Msk                       /*!< SMBus alert enable */
#define I2C_CR1_PECEN_Pos            (23U)                                     
#define I2C_CR1_PECEN_Msk            (0x1U << I2C_CR1_PECEN_Pos)               /*!< 0x00800000 */
#define I2C_CR1_PECEN                I2C_CR1_PECEN_Msk                         /*!< PEC enable */

/******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_SADD_Pos             (0U)                                      
#define I2C_CR2_SADD_Msk             (0x3FFU << I2C_CR2_SADD_Pos)              /*!< 0x000003FF */
#define I2C_CR2_SADD                 I2C_CR2_SADD_Msk                          /*!< Slave address (master mode) */
#define I2C_CR2_RD_WRN_Pos           (10U)                                     
#define I2C_CR2_RD_WRN_Msk           (0x1U << I2C_CR2_RD_WRN_Pos)              /*!< 0x00000400 */
#define I2C_CR2_RD_WRN               I2C_CR2_RD_WRN_Msk                        /*!< Transfer direction (master mode) */
#define I2C_CR2_ADD10_Pos            (11U)                                     
#define I2C_CR2_ADD10_Msk            (0x1U << I2C_CR2_ADD10_Pos)               /*!< 0x00000800 */
#define I2C_CR2_ADD10                I2C_CR2_ADD10_Msk                         /*!< 10-bit addressing mode (master mode) */
#define I2C_CR2_HEAD10R_Pos          (12U)                                     
#define I2C_CR2_HEAD10R_Msk          (0x1U << I2C_CR2_HEAD10R_Pos)             /*!< 0x00001000 */
#define I2C_CR2_HEAD10R              I2C_CR2_HEAD10R_Msk                       /*!< 10-bit address header only read direction (master mode) */
#define I2C_CR2_START_Pos            (13U)                                     
#define I2C_CR2_START_Msk            (0x1U << I2C_CR2_START_Pos)               /*!< 0x00002000 */
#define I2C_CR2_START                I2C_CR2_START_Msk                         /*!< START generation */
#define I2C_CR2_STOP_Pos             (14U)                                     
#define I2C_CR2_STOP_Msk             (0x1U << I2C_CR2_STOP_Pos)                /*!< 0x00004000 */
#define I2C_CR2_STOP                 I2C_CR2_STOP_Msk                          /*!< STOP generation (master mode) */
#define I2C_CR2_NACK_Pos             (15U)                                     
#define I2C_CR2_NACK_Msk             (0x1U << I2C_CR2_NACK_Pos)                /*!< 0x00008000 */
#define I2C_CR2_NACK                 I2C_CR2_NACK_Msk                          /*!< NACK generation (slave mode) */
#define I2C_CR2_NBYTES_Pos           (16U)                                     
#define I2C_CR2_NBYTES_Msk           (0xFFU << I2C_CR2_NBYTES_Pos)             /*!< 0x00FF0000 */
#define I2C_CR2_NBYTES               I2C_CR2_NBYTES_Msk                        /*!< Number of bytes */
#define I2C_CR2_RELOAD_Pos           (24U)                                     
#define I2C_CR2_RELOAD_Msk           (0x1U << I2C_CR2_RELOAD_Pos)              /*!< 0x01000000 */
#define I2C_CR2_RELOAD               I2C_CR2_RELOAD_Msk                        /*!< NBYTES reload mode */
#define I2C_CR2_AUTOEND_Pos          (25U)                                     
#define I2C_CR2_AUTOEND_Msk          (0x1U << I2C_CR2_AUTOEND_Pos)             /*!< 0x02000000 */
#define I2C_CR2_AUTOEND              I2C_CR2_AUTOEND_Msk                       /*!< Automatic end mode (master mode) */
#define I2C_CR2_PECBYTE_Pos          (26U)                                     
#define I2C_CR2_PECBYTE_Msk          (0x1U << I2C_CR2_PECBYTE_Pos)             /*!< 0x04000000 */
#define I2C_CR2_PECBYTE              I2C_CR2_PECBYTE_Msk                       /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define I2C_OAR1_OA1_Pos             (0U)                                      
#define I2C_OAR1_OA1_Msk             (0x3FFU << I2C_OAR1_OA1_Pos)              /*!< 0x000003FF */
#define I2C_OAR1_OA1                 I2C_OAR1_OA1_Msk                          /*!< Interface own address 1 */
#define I2C_OAR1_OA1MODE_Pos         (10U)                                     
#define I2C_OAR1_OA1MODE_Msk         (0x1U << I2C_OAR1_OA1MODE_Pos)            /*!< 0x00000400 */
#define I2C_OAR1_OA1MODE             I2C_OAR1_OA1MODE_Msk                      /*!< Own address 1 10-bit mode */
#define I2C_OAR1_OA1EN_Pos           (15U)                                     
#define I2C_OAR1_OA1EN_Msk           (0x1U << I2C_OAR1_OA1EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR1_OA1EN               I2C_OAR1_OA1EN_Msk                        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define I2C_OAR2_OA2_Pos             (1U)                                      
#define I2C_OAR2_OA2_Msk             (0x7FU << I2C_OAR2_OA2_Pos)               /*!< 0x000000FE */
#define I2C_OAR2_OA2                 I2C_OAR2_OA2_Msk                          /*!< Interface own address 2 */
#define I2C_OAR2_OA2MSK_Pos          (8U)                                      
#define I2C_OAR2_OA2MSK_Msk          (0x7U << I2C_OAR2_OA2MSK_Pos)             /*!< 0x00000700 */
#define I2C_OAR2_OA2MSK              I2C_OAR2_OA2MSK_Msk                       /*!< Own address 2 masks */
#define I2C_OAR2_OA2NOMASK           (0x00000000U)                             /*!< No mask                                        */
#define I2C_OAR2_OA2MASK01_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK01_Msk       (0x1U << I2C_OAR2_OA2MASK01_Pos)          /*!< 0x00000100 */
#define I2C_OAR2_OA2MASK01           I2C_OAR2_OA2MASK01_Msk                    /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define I2C_OAR2_OA2MASK02_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK02_Msk       (0x1U << I2C_OAR2_OA2MASK02_Pos)          /*!< 0x00000200 */
#define I2C_OAR2_OA2MASK02           I2C_OAR2_OA2MASK02_Msk                    /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define I2C_OAR2_OA2MASK03_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK03_Msk       (0x3U << I2C_OAR2_OA2MASK03_Pos)          /*!< 0x00000300 */
#define I2C_OAR2_OA2MASK03           I2C_OAR2_OA2MASK03_Msk                    /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define I2C_OAR2_OA2MASK04_Pos       (10U)                                     
#define I2C_OAR2_OA2MASK04_Msk       (0x1U << I2C_OAR2_OA2MASK04_Pos)          /*!< 0x00000400 */
#define I2C_OAR2_OA2MASK04           I2C_OAR2_OA2MASK04_Msk                    /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define I2C_OAR2_OA2MASK05_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK05_Msk       (0x5U << I2C_OAR2_OA2MASK05_Pos)          /*!< 0x00000500 */
#define I2C_OAR2_OA2MASK05           I2C_OAR2_OA2MASK05_Msk                    /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define I2C_OAR2_OA2MASK06_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK06_Msk       (0x3U << I2C_OAR2_OA2MASK06_Pos)          /*!< 0x00000600 */
#define I2C_OAR2_OA2MASK06           I2C_OAR2_OA2MASK06_Msk                    /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define I2C_OAR2_OA2MASK07_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK07_Msk       (0x7U << I2C_OAR2_OA2MASK07_Pos)          /*!< 0x00000700 */
#define I2C_OAR2_OA2MASK07           I2C_OAR2_OA2MASK07_Msk                    /*!< OA2[7:1] is masked, No comparison is done      */
#define I2C_OAR2_OA2EN_Pos           (15U)                                     
#define I2C_OAR2_OA2EN_Msk           (0x1U << I2C_OAR2_OA2EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR2_OA2EN               I2C_OAR2_OA2EN_Msk                        /*!< Own address 2 enable */

/*******************  Bit definition for I2C_TIMINGR register ****************/
#define I2C_TIMINGR_SCLL_Pos         (0U)                                      
#define I2C_TIMINGR_SCLL_Msk         (0xFFU << I2C_TIMINGR_SCLL_Pos)           /*!< 0x000000FF */
#define I2C_TIMINGR_SCLL             I2C_TIMINGR_SCLL_Msk                      /*!< SCL low period (master mode) */
#define I2C_TIMINGR_SCLH_Pos         (8U)                                      
#define I2C_TIMINGR_SCLH_Msk         (0xFFU << I2C_TIMINGR_SCLH_Pos)           /*!< 0x0000FF00 */
#define I2C_TIMINGR_SCLH             I2C_TIMINGR_SCLH_Msk                      /*!< SCL high period (master mode) */
#define I2C_TIMINGR_SDADEL_Pos       (16U)                                     
#define I2C_TIMINGR_SDADEL_Msk       (0xFU << I2C_TIMINGR_SDADEL_Pos)          /*!< 0x000F0000 */
#define I2C_TIMINGR_SDADEL           I2C_TIMINGR_SDADEL_Msk                    /*!< Data hold time */
#define I2C_TIMINGR_SCLDEL_Pos       (20U)                                     
#define I2C_TIMINGR_SCLDEL_Msk       (0xFU << I2C_TIMINGR_SCLDEL_Pos)          /*!< 0x00F00000 */
#define I2C_TIMINGR_SCLDEL           I2C_TIMINGR_SCLDEL_Msk                    /*!< Data setup time */
#define I2C_TIMINGR_PRESC_Pos        (28U)                                     
#define I2C_TIMINGR_PRESC_Msk        (0xFU << I2C_TIMINGR_PRESC_Pos)           /*!< 0xF0000000 */
#define I2C_TIMINGR_PRESC            I2C_TIMINGR_PRESC_Msk                     /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register ****************/
#define I2C_TIMEOUTR_TIMEOUTA_Pos    (0U)                                      
#define I2C_TIMEOUTR_TIMEOUTA_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTA_Pos)     /*!< 0x00000FFF */
#define I2C_TIMEOUTR_TIMEOUTA        I2C_TIMEOUTR_TIMEOUTA_Msk                 /*!< Bus timeout A */
#define I2C_TIMEOUTR_TIDLE_Pos       (12U)                                     
#define I2C_TIMEOUTR_TIDLE_Msk       (0x1U << I2C_TIMEOUTR_TIDLE_Pos)          /*!< 0x00001000 */
#define I2C_TIMEOUTR_TIDLE           I2C_TIMEOUTR_TIDLE_Msk                    /*!< Idle clock timeout detection */
#define I2C_TIMEOUTR_TIMOUTEN_Pos    (15U)                                     
#define I2C_TIMEOUTR_TIMOUTEN_Msk    (0x1U << I2C_TIMEOUTR_TIMOUTEN_Pos)       /*!< 0x00008000 */
#define I2C_TIMEOUTR_TIMOUTEN        I2C_TIMEOUTR_TIMOUTEN_Msk                 /*!< Clock timeout enable */
#define I2C_TIMEOUTR_TIMEOUTB_Pos    (16U)                                     
#define I2C_TIMEOUTR_TIMEOUTB_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTB_Pos)     /*!< 0x0FFF0000 */
#define I2C_TIMEOUTR_TIMEOUTB        I2C_TIMEOUTR_TIMEOUTB_Msk                 /*!< Bus timeout B*/
#define I2C_TIMEOUTR_TEXTEN_Pos      (31U)                                     
#define I2C_TIMEOUTR_TEXTEN_Msk      (0x1U << I2C_TIMEOUTR_TEXTEN_Pos)         /*!< 0x80000000 */
#define I2C_TIMEOUTR_TEXTEN          I2C_TIMEOUTR_TEXTEN_Msk                   /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  ********************/
#define I2C_ISR_TXE_Pos              (0U)                                      
#define I2C_ISR_TXE_Msk              (0x1U << I2C_ISR_TXE_Pos)                 /*!< 0x00000001 */
#define I2C_ISR_TXE                  I2C_ISR_TXE_Msk                           /*!< Transmit data register empty */
#define I2C_ISR_TXIS_Pos             (1U)                                      
#define I2C_ISR_TXIS_Msk             (0x1U << I2C_ISR_TXIS_Pos)                /*!< 0x00000002 */
#define I2C_ISR_TXIS                 I2C_ISR_TXIS_Msk                          /*!< Transmit interrupt status */
#define I2C_ISR_RXNE_Pos             (2U)                                      
#define I2C_ISR_RXNE_Msk             (0x1U << I2C_ISR_RXNE_Pos)                /*!< 0x00000004 */
#define I2C_ISR_RXNE                 I2C_ISR_RXNE_Msk                          /*!< Receive data register not empty */
#define I2C_ISR_ADDR_Pos             (3U)                                      
#define I2C_ISR_ADDR_Msk             (0x1U << I2C_ISR_ADDR_Pos)                /*!< 0x00000008 */
#define I2C_ISR_ADDR                 I2C_ISR_ADDR_Msk                          /*!< Address matched (slave mode)*/
#define I2C_ISR_NACKF_Pos            (4U)                                      
#define I2C_ISR_NACKF_Msk            (0x1U << I2C_ISR_NACKF_Pos)               /*!< 0x00000010 */
#define I2C_ISR_NACKF                I2C_ISR_NACKF_Msk                         /*!< NACK received flag */
#define I2C_ISR_STOPF_Pos            (5U)                                      
#define I2C_ISR_STOPF_Msk            (0x1U << I2C_ISR_STOPF_Pos)               /*!< 0x00000020 */
#define I2C_ISR_STOPF                I2C_ISR_STOPF_Msk                         /*!< STOP detection flag */
#define I2C_ISR_TC_Pos               (6U)                                      
#define I2C_ISR_TC_Msk               (0x1U << I2C_ISR_TC_Pos)                  /*!< 0x00000040 */
#define I2C_ISR_TC                   I2C_ISR_TC_Msk                            /*!< Transfer complete (master mode) */
#define I2C_ISR_TCR_Pos              (7U)                                      
#define I2C_ISR_TCR_Msk              (0x1U << I2C_ISR_TCR_Pos)                 /*!< 0x00000080 */
#define I2C_ISR_TCR                  I2C_ISR_TCR_Msk                           /*!< Transfer complete reload */
#define I2C_ISR_BERR_Pos             (8U)                                      
#define I2C_ISR_BERR_Msk             (0x1U << I2C_ISR_BERR_Pos)                /*!< 0x00000100 */
#define I2C_ISR_BERR                 I2C_ISR_BERR_Msk                          /*!< Bus error */
#define I2C_ISR_ARLO_Pos             (9U)                                      
#define I2C_ISR_ARLO_Msk             (0x1U << I2C_ISR_ARLO_Pos)                /*!< 0x00000200 */
#define I2C_ISR_ARLO                 I2C_ISR_ARLO_Msk                          /*!< Arbitration lost */
#define I2C_ISR_OVR_Pos              (10U)                                     
#define I2C_ISR_OVR_Msk              (0x1U << I2C_ISR_OVR_Pos)                 /*!< 0x00000400 */
#define I2C_ISR_OVR                  I2C_ISR_OVR_Msk                           /*!< Overrun/Underrun */
#define I2C_ISR_PECERR_Pos           (11U)                                     
#define I2C_ISR_PECERR_Msk           (0x1U << I2C_ISR_PECERR_Pos)              /*!< 0x00000800 */
#define I2C_ISR_PECERR               I2C_ISR_PECERR_Msk                        /*!< PEC error in reception */
#define I2C_ISR_TIMEOUT_Pos          (12U)                                     
#define I2C_ISR_TIMEOUT_Msk          (0x1U << I2C_ISR_TIMEOUT_Pos)             /*!< 0x00001000 */
#define I2C_ISR_TIMEOUT              I2C_ISR_TIMEOUT_Msk                       /*!< Timeout or Tlow detection flag */
#define I2C_ISR_ALERT_Pos            (13U)                                     
#define I2C_ISR_ALERT_Msk            (0x1U << I2C_ISR_ALERT_Pos)               /*!< 0x00002000 */
#define I2C_ISR_ALERT                I2C_ISR_ALERT_Msk                         /*!< SMBus alert */
#define I2C_ISR_BUSY_Pos             (15U)                                     
#define I2C_ISR_BUSY_Msk             (0x1U << I2C_ISR_BUSY_Pos)                /*!< 0x00008000 */
#define I2C_ISR_BUSY                 I2C_ISR_BUSY_Msk                          /*!< Bus busy */
#define I2C_ISR_DIR_Pos              (16U)                                     
#define I2C_ISR_DIR_Msk              (0x1U << I2C_ISR_DIR_Pos)                 /*!< 0x00010000 */
#define I2C_ISR_DIR                  I2C_ISR_DIR_Msk                           /*!< Transfer direction (slave mode) */
#define I2C_ISR_ADDCODE_Pos          (17U)                                     
#define I2C_ISR_ADDCODE_Msk          (0x7FU << I2C_ISR_ADDCODE_Pos)            /*!< 0x00FE0000 */
#define I2C_ISR_ADDCODE              I2C_ISR_ADDCODE_Msk                       /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  ********************/
#define I2C_ICR_ADDRCF_Pos           (3U)                                      
#define I2C_ICR_ADDRCF_Msk           (0x1U << I2C_ICR_ADDRCF_Pos)              /*!< 0x00000008 */
#define I2C_ICR_ADDRCF               I2C_ICR_ADDRCF_Msk                        /*!< Address matched clear flag */
#define I2C_ICR_NACKCF_Pos           (4U)                                      
#define I2C_ICR_NACKCF_Msk           (0x1U << I2C_ICR_NACKCF_Pos)              /*!< 0x00000010 */
#define I2C_ICR_NACKCF               I2C_ICR_NACKCF_Msk                        /*!< NACK clear flag */
#define I2C_ICR_STOPCF_Pos           (5U)                                      
#define I2C_ICR_STOPCF_Msk           (0x1U << I2C_ICR_STOPCF_Pos)              /*!< 0x00000020 */
#define I2C_ICR_STOPCF               I2C_ICR_STOPCF_Msk                        /*!< STOP detection clear flag */
#define I2C_ICR_BERRCF_Pos           (8U)                                      
#define I2C_ICR_BERRCF_Msk           (0x1U << I2C_ICR_BERRCF_Pos)              /*!< 0x00000100 */
#define I2C_ICR_BERRCF               I2C_ICR_BERRCF_Msk                        /*!< Bus error clear flag */
#define I2C_ICR_ARLOCF_Pos           (9U)                                      
#define I2C_ICR_ARLOCF_Msk           (0x1U << I2C_ICR_ARLOCF_Pos)              /*!< 0x00000200 */
#define I2C_ICR_ARLOCF               I2C_ICR_ARLOCF_Msk                        /*!< Arbitration lost clear flag */
#define I2C_ICR_OVRCF_Pos            (10U)                                     
#define I2C_ICR_OVRCF_Msk            (0x1U << I2C_ICR_OVRCF_Pos)               /*!< 0x00000400 */
#define I2C_ICR_OVRCF                I2C_ICR_OVRCF_Msk                         /*!< Overrun/Underrun clear flag */
#define I2C_ICR_PECCF_Pos            (11U)                                     
#define I2C_ICR_PECCF_Msk            (0x1U << I2C_ICR_PECCF_Pos)               /*!< 0x00000800 */
#define I2C_ICR_PECCF                I2C_ICR_PECCF_Msk                         /*!< PAC error clear flag */
#define I2C_ICR_TIMOUTCF_Pos         (12U)                                     
#define I2C_ICR_TIMOUTCF_Msk         (0x1U << I2C_ICR_TIMOUTCF_Pos)            /*!< 0x00001000 */
#define I2C_ICR_TIMOUTCF             I2C_ICR_TIMOUTCF_Msk                      /*!< Timeout clear flag */
#define I2C_ICR_ALERTCF_Pos          (13U)                                     
#define I2C_ICR_ALERTCF_Msk          (0x1U << I2C_ICR_ALERTCF_Pos)             /*!< 0x00002000 */
#define I2C_ICR_ALERTCF              I2C_ICR_ALERTCF_Msk                       /*!< Alert clear flag */

/******************  Bit definition for I2C_PECR register  *******************/
#define I2C_PECR_PEC_Pos             (0U)                                      
#define I2C_PECR_PEC_Msk             (0xFFU << I2C_PECR_PEC_Pos)               /*!< 0x000000FF */
#define I2C_PECR_PEC                 I2C_PECR_PEC_Msk                          /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define I2C_RXDR_RXDATA_Pos          (0U)                                      
#define I2C_RXDR_RXDATA_Msk          (0xFFU << I2C_RXDR_RXDATA_Pos)            /*!< 0x000000FF */
#define I2C_RXDR_RXDATA              I2C_RXDR_RXDATA_Msk                       /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *******************/
#define I2C_TXDR_TXDATA_Pos          (0U)                                      
#define I2C_TXDR_TXDATA_Msk          (0xFFU << I2C_TXDR_TXDATA_Pos)            /*!< 0x000000FF */
#define I2C_TXDR_TXDATA              I2C_TXDR_TXDATA_Msk                       /*!< 8-bit transmit data */

#ifdef __cplusplus
 }
#endif 

#endif // GD32_F150_h
