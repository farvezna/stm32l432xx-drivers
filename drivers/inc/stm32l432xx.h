/*
 * stm32l432xx.h
 *
 *  Created on: May 6, 2020
 *      Author: farvez
 */

#ifndef INC_STM32L432XX_H_
#define INC_STM32L432XX_H_

#include <stdint.h>

#define __vo volatile

/********************START: Processor Specific Details**************************************
 *
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0                  ( (__vo uint32_t*) 0xE00E100 )
#define NVIC_ISER1                  ( (__vo uint32_t*) 0xE00E104 )
#define NVIC_ISER2                  ( (__vo uint32_t*) 0xE00E108 )
#define NVIC_ISER3                  ( (__vo uint32_t*) 0xE00E10C )

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0                  ( (__vo uint32_t*) 0xE00E180 )
#define NVIC_ICER1                  ( (__vo uint32_t*) 0xE00E184 )
#define NVIC_ICER2                  ( (__vo uint32_t*) 0xE00E188 )
#define NVIC_ICER3                  ( (__vo uint32_t*) 0xE00E18C )

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR           ( (__vo uint32_t*) 0xE00E400)


#define NO_PR_BITS_IMPLEMENTED     4
/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR              0x08000000U         //base address of FLASH memory
#define SRAM1_BASEADDR				0x20000000U         //base address of SRAM1
#define SRAM2_BASEADDR				0x10000000U         //base address of SRAM2
#define ROM							0x1FFF0000U			//system memory in ref manual
#define SRAM 						SRAM1_BASEADDR      //main SRAM is SRAM1

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR             0x40000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR         0x40010000U
#define AHB1PERIPH_BASEADDR         0x40020000U
#define AHB2PERIPH_BASEADDR         0x48000000U

/*
 * Base addresses of peripherals hanging on AHB1 bus
 */

#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x1000)

/*
 * Base addresses of peripherals hanging on AHB2 bus
 */

#define GPIOA_BASEADDR              (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOH_BASEADDR              (AHB2PERIPH_BASEADDR + 0x1C00)

/*
 * Base addresses of peripherals hanging on APB1 bus
 */

#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x4400)
#define LPUART1_BASEADDR            (APB1PERIPH_BASEADDR + 0x8000)

/*
 * Base addresses of peripherals hanging on APB2 bus
 */

#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)

#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x0400)  //external interrupt
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x0000)

/***********************************peripheral register definition structures************************************/

/*
 * peripheral register definition struct for GPIO
 */
typedef struct {
    __vo uint32_t MODER;             //GPIO port mode register           address offset 0x00 bytes
    __vo uint32_t OTYPER;            //GPIO output type register         address offset 0x04
    __vo uint32_t OSPEEDR;           //GPIO output speed register        address offset 0x08 etc.
    __vo uint32_t PUPDR;             //GPIO port pull-up/pull-down register
    __vo uint32_t IDR;               //GPIO port input data register
    __vo uint32_t ODR;               //GPIO port output data register
    __vo uint32_t BSRR;              //GPIO port bit set/reset register
    __vo uint32_t LCKR;              //GPIO port configuration lock register
    __vo uint32_t AFR[2];            //GPIO alternate function register; for both L[0] and H[1] registers AFRL/AFRH
    __vo uint32_t BRR;               //GPIO port bit reset register?--what do
} GPIO_RegDef_t;

/*
 * peripheral register definition struct for RCC
 */
typedef struct {
    __vo uint32_t CR;               //clock control register
    __vo uint32_t ICSCR;            //internal clock sources calibration register
    __vo uint32_t CFGR;             //clock configuration register
    __vo uint32_t PLLCFGR;          //PLL configuration register
    __vo uint64_t PLLSAI1CFGR;      //PLLSAI1 configuration register
    __vo uint32_t CIER;             //clock interrupt enable register
    __vo uint32_t CIFR;             //clock interrupt flag register
    __vo uint64_t CICR;             //clock interrupt clear register
    __vo uint32_t AHB1RSTR;         //AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;         //AHB2 peripheral reset register
    __vo uint64_t AHB3RSTR;         //AHB3 peripheral reset register
    __vo uint32_t APB1RSTR1;        //APB1 peripheral reset register 1
    __vo uint32_t APB1RSTR2;        //APB1 peripheral reset register 2
    __vo uint64_t APB2RSTR;         //APB2 peripheral reset register
    __vo uint32_t AHB1ENR;          //AHB1 peripheral clock enable register
    __vo uint32_t AHB2ENR;          //AHB2 peripheral clock enable register
    __vo uint64_t AHB3ENR;          //AHB3 peripheral clock enable register
    __vo uint32_t APB1ENR1;         //APB1 peripheral clock enable register 1
    __vo uint32_t APB1ENR2;         //APB1 peripheral clock enable register 2
    __vo uint64_t APB2ENR;          //APB2 peripheral clock enable register
    __vo uint32_t AHB1SMENR;        //AHB1 peripheral clocks enable in Sleep and Stop modes register
    __vo uint32_t AHB2SMENR;        //AHB2 peripheral clocks enable in Sleep and Stop modes register
    __vo uint64_t AHB3SMENR;        //AHB3 peripheral clocks enable in Sleep and Stop modes register
    __vo uint32_t APB1SMENR1;       //APB1 peripheral clocks enable in Sleep and Stop modes register 1
    __vo uint32_t APB1SMENR2;       //APB1 peripheral clocks enable in Sleep and Stop modes register 2
    __vo uint64_t APB2SMENR;        //APB2 peripheral clocks enable in Sleep and Stop modes register
    __vo uint64_t CCIPR;            //Peripherals independent clock configuration register
    __vo uint32_t BDCR;             //Backup domain control register
    __vo uint32_t CSR;              //Control/status register
    __vo uint32_t CRRCR;            //Clock recovery RC register
    __vo uint32_t CCIPR2;           //Peripherals independent clock configuration register (2?)
} RCC_RegDef_t;

/*
 * peripheral register definition struct for EXTI
 */
typedef struct {
    __vo uint32_t IMR;             //address offset 0x00 bytes
    __vo uint32_t EMR;             //address offset 0x04
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;
} EXTI_RegDef_t;   //TODO: stm32L4 (vs F4) has IMR2, EMR2....etc. configure later pls

/*
 * peripheral register definition struct for SYSCFG
 */
typedef struct {
    __vo uint32_t MEMRMP;           //memory remap register      address offset 0x00 bytes
    __vo uint32_t CFGR1;            //configuration register 1    address offset 0x04
    __vo uint32_t EXTICR[4];          //external interrupt configuration registers [1-4] ...note: only one of interest for EXTI use
    __vo uint32_t SCSR;             //SRAM2 control and status register
    __vo uint32_t CFGR2;            //configuration register 2
    __vo uint32_t SWPR;             //SRAM2 write protection register
    __vo uint32_t SKR;              //SRAM2 key register
} SYSCFG_RegDef_t;

/*
 * Peripheral definitions (base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOH       ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC         ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI        ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()         (RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB2ENR |= (1 << 2))
#define GPIOH_PCLK_EN()         (RCC->AHB2ENR |= (1 << 7))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR1 |= (1 << 21))
#define I2C3_PCLK_EN()          (RCC->APB1ENR1 |= (1 << 23))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR  |= (1 << 12))
#define SPI3_PCLK_EN()          (RCC->APB1ENR1 |= (1 << 15))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR  |= (1 << 14))
#define USART2_PCLK_EN()        (RCC->APB1ENR1 |= (1 << 17))
#define LPUART1_PCLK_EN()       (RCC->APB1ENR2 |= (1 <<  0))

/*
 * Clock enable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 0))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()         (RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()         (RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB2ENR &= ~(1 << 2))
#define GPIOH_PCLK_DI()         (RCC->AHB2ENR &= ~(1 << 7))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR1 &= ~(1 << 21))
#define I2C3_PCLK_DI()          (RCC->APB1ENR1 &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR  &= ~(1 << 12))
#define SPI3_PCLK_DI()          (RCC->APB1ENR1 &= ~(1 << 15))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()        (RCC->APB2ENR  &= ~(1 << 14))
#define USART2_PCLK_DI()        (RCC->APB1ENR1 &= ~(1 << 17))
#define LPUART1_PCLK_DI()       (RCC->APB1ENR2 &= ~(1 <<  0))

/*
 * Clock disable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 0))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()       do{ (RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0));} while(0) //reset at 1, cannot keep holding so put back to 0
#define GPIOB_REG_RESET()       do{ (RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()       do{ (RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2));} while(0)
#define GPIOH_REG_RESET()       do{ (RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7));} while(0)

#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
                                    (x == GPIOB) ? 1 :\
                                    (x == GPIOC) ? 2 :\
                                    (x == GPIOH) ? 15:0 )

//IRQ numbers STM32L43xxx (interrupt request) position
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10

//generic macros
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET

#include "stm32l432xx_gpio_driver.h"

#endif /* INC_STM32L432XX_H_ */
