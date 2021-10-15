/*
 * stm32l432kc_gpio_driver.c
 *
 *  Created on: Oct 14, 2021
 *      Author: avila
 */


#include "stm32l432kc.h"
//#include "stm32l432kc_gpio_driver.h"
/*
 * Function that enables or disables peripheral clock for the given GPIO port
 * Param1:	base address of the gpio peripheral
 * Param2:  ENABLE or DISABLE macros
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DIS();
			}
		else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DIS();
			}
		else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DIS();
			}
		else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DIS();
			}
	}
}
/*Function that initialize given GPIO port and GPIO pin
 * Param1:	base address of the gpio peripheral
 * Param2:  ENABLE or DISABLE macros*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		//                 pin mode: 1                  <<  2 *  (pin#4) to set pin 4 as output
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //store the value of temp into the actual register that is MODER
		temp = 0;
	}
	else
	{
		//interrupt mode
	}
	//configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
		{	//pin 0 to 7 low register
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFRL &= (0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp));
		}
		else
		{	//pin 8 to 15 high register
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFRH &= (0xF << (4 * temp));//clearing
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp));
		}
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) //use the RCC peripheral reset register, just take the base address of the peripheral
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


uint8_t  GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}
