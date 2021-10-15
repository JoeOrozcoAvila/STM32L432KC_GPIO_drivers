/*
 * stm32l432kc.h
 *
 *	MCU specific header file
 *
 *  Created on: Oct 14, 2021
 *      Author: avila
 */

#ifndef INC_STM32L432KC_H_
#define INC_STM32L432KC_H_

#include <stdint.h>
#include "stm32l432kc_gpio_driver.h"

/*
 * Base address of Flash and SRAM memories
 * */
#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U		//size of 48Kb
#define SRAM2_BASEADDR					0x2000C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM 							SRAM1_BASEADDR

/*
 * AHBx and APBx Bus peripheral base addresses
 * */
#define PERIPH_BASE						0x40000000U
#define APB1_BASEADDR					PERIPH_BASE
#define APB2_BASEADDR					0x40010000U
#define AHB1_BASEADDR					0x40020000U
#define AHB2_BASEADDR					0x48000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * */
#define DMA1_BASEADDR					(AHB1_BASEADDR + 0x0000)
#define DMA2_BASEADDR					(AHB1_BASEADDR + 0x0400)
#define RCC_BASEADDR					(AHB1_BASEADDR + 0x1000)
#define FLASHR_BASEADDR					(AHB1_BASEADDR + 0x2000)
#define CRC_BASEADDR					(AHB1_BASEADDR + 0x3000)
#define TSC_BASEADDR					(AHB1_BASEADDR + 0x4000)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 * */
#define GPIOA_BASEADDR					(AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB2_BASEADDR + 0x0800)
#define GPIOH_BASEADDR					(AHB2_BASEADDR + 0x1C00)
#define ADC_BASEADDR					(AHB2_BASEADDR + 0x08040000)
#define AES_BASEADDR					(AHB2_BASEADDR + 0x08060000)
#define RNG_BASEADDR					(AHB2_BASEADDR + 0x08060800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * */
#define TIM2_BASEADDR					(APB1_BASEADDR + 0x0000)
#define TIM6_BASEADDR					(APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR					(APB1_BASEADDR + 0x1400)
#define LCD_BASEADDR					(APB1_BASEADDR + 0x2400)
#define RTC_BASEADDR					(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR					(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR					(APB1_BASEADDR + 0x3000)
#define SPI2_BASEADDR					(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR					(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1_BASEADDR + 0x4800)
#define I2C1_BASEADDR					(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1_BASEADDR + 0x5C00)
#define CRS_BASEADDR					(APB1_BASEADDR + 0x6000)
#define CAN1_BASEADDR					(APB1_BASEADDR + 0x6400)
#define USBFS_BASEADDR					(APB1_BASEADDR + 0x6800)
#define USBSRAM_BASEADDR				(APB1_BASEADDR + 0x6C00)
#define PWR_BASEADDR					(APB1_BASEADDR + 0x7000)
#define DAC1_BASEADDR					(APB1_BASEADDR + 0x7400)
#define OPAMP_BASEADDR					(APB1_BASEADDR + 0x7800)
#define LPTIM1_BASEADDR					(APB1_BASEADDR + 0x7C00)
#define LPUART1_BASEADDR				(APB1_BASEADDR + 0x8000)
#define I2C4_BASEADDR					(APB1_BASEADDR + 0x8400)
#define SWPMI1_BASEADDR					(APB1_BASEADDR + 0x8800)
#define LPTIM2_BASEADDR					(APB1_BASEADDR + 0x9400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * */
#define SYSCFG_BASEADDR					(APB2_BASEADDR + 0x0000)
#define VREFBUF_BASEADDR				(APB2_BASEADDR + 0x0030)
#define COMP_BASEADDR					(APB2_BASEADDR + 0x0200)
#define EXTI_BASEADDR					(APB2_BASEADDR + 0x0400)
#define FIREWALL_BASEADDR				(APB2_BASEADDR + 0x1C00)
#define TIM1_BASEADDR					(APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR					(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR					(APB2_BASEADDR + 0x3800)
#define TIM15_BASEADDR					(APB2_BASEADDR + 0x4000)
#define TIM16_BASEADDR					(APB2_BASEADDR + 0x4400)
#define SAI1_BASEADDR					(APB2_BASEADDR + 0x5400)
#define DFSDM1_BASEADDR					(APB2_BASEADDR + 0x6000)

/*******************************************************************************************************************
 * *********   Peripheral Register Definition Structures   *********************************************************
 * *****************************************************************************************************************/
typedef struct
{
	volatile uint32_t MODER;				//OFFSET 0x00
	volatile uint32_t OTYPER;				//OFFSET 0x04
	volatile uint32_t OSPEEDR;				//OFFSET 0x08
	volatile uint32_t PUPDR;				//OFFSET 0x0C
	volatile uint32_t IDR;					//OFFSET 0x10
	volatile uint32_t ODR;					//OFFSET 0x14
	volatile uint32_t BSRR;					//OFFSET 0x18
	volatile uint32_t LCKR;					//OFFSET 0x1C
	volatile uint32_t AFRL;					//OFFSET 0x20
	volatile uint32_t AFRH;					//OFFSET 0x24
	volatile uint32_t BRR;					//OFFSET 0x28
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;					//OFFSET 0x00
	volatile uint32_t ICSCR;				//OFFSET 0x04
	volatile uint32_t CFGR;					//OFFSET 0x08
	volatile uint32_t PLLCFGR;				//OFFSET 0x0C
	volatile uint32_t PLLSAI1CFGR;			//OFFSET 0x10
	volatile uint32_t CIER;					//OFFSET 0x18
	volatile uint32_t CIFR;					//OFFSET 0x1C
	volatile uint32_t CICR;					//OFFSET 0x20
	volatile uint32_t AHB1RSTR;				//OFFSET 0x28
	volatile uint32_t AHB2RSTR;				//OFFSET 0x2C
	volatile uint32_t AHB3RSTR;				//OFFSET 0x30
	volatile uint32_t APB1RSTR1;			//OFFSET 0x38
	volatile uint32_t APB1RSTR2;			//OFFSET 0x3C
	volatile uint32_t APB2RSTR;				//OFFSET 0x40
	volatile uint32_t AHB1ENR;				//OFFSET 0x48
	volatile uint32_t AHB2ENR;				//OFFSET 0x4C
	volatile uint32_t AHB3ENR;				//OFFSET 0x50
	volatile uint32_t APB1ENR1;				//OFFSET 0x58
	volatile uint32_t APB1ENR2;				//OFFSET 0x5C
	volatile uint32_t APB2ENR;				//OFFSET 0x60
	volatile uint32_t AHB1SMENR;			//OFFSET 0x68
	volatile uint32_t AHB2SMENR;			//OFFSET 0x6C
	volatile uint32_t AHB3SMENR;			//OFFSET 0x70
	volatile uint32_t APB1SMENR1;			//OFFSET 0x78
	volatile uint32_t APB1SMENR2;			//OFFSET 0x7C
	volatile uint32_t APB2SMENR;			//OFFSET 0x80
	volatile uint32_t CCIPR;				//OFFSET 0x88
	volatile uint32_t BDCR;					//OFFSET 0x90
	volatile uint32_t CSR;					//OFFSET 0x94
	volatile uint32_t CRRCR;				//OFFSET 0x98
	volatile uint32_t CCIPR2;				//OFFSET 0x9C
}RCC_RegDef_t;

/*
 * Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t pointer structure type
 * */
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)
/*
 * Clock enable macros for GPIOx Peripherals
 * */
#define GPIOA_PCLK_EN()					( RCC->AHB2ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()					( RCC->AHB2ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()					( RCC->AHB2ENR |= ( 1 << 2 ) )
#define GPIOH_PCLK_EN()					( RCC->AHB2ENR |= ( 1 << 7 ) )

/*
 * Clock disable macros for GPIOx Peripherals
 * */
#define GPIOA_PCLK_DIS()				( RCC->AHB2ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DIS()				( RCC->AHB2ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DIS()				( RCC->AHB2ENR &= ~( 1 << 2 ) )
#define GPIOH_PCLK_DIS()				( RCC->AHB2ENR &= ~( 1 << 7 ) )

/*Macros to reset GPIOx peripherals*/
#define GPIOA_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<0)); (RCC->AHB2RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<1)); (RCC->AHB2RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<2)); (RCC->AHB2RSTR &= ~(1<<2)); }while(0)
#define GPIOH_REG_RESET()				do{ (RCC->AHB2RSTR |= (1<<7)); (RCC->AHB2RSTR &= ~(1<<7)); }while(0)

/*Generic macros*/
#define ENABLE							1
#define DISABLE 						0
#define SET 							ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET



#endif /* INC_STM32L432KC_H_ */
