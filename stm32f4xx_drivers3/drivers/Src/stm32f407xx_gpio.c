/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Jul 23, 2025
 *      Author: yig88
 */
#include "stm32f407xx_gpio.h"
/**
	* @brief  Init the GPIO
	* @param  *pGPIOHandle : pointer to the GPIO handle struct
	* @param
	* @param
	* @return None
	*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
		//1 configure the mode of GPIO pin
	uint32_t temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else{
		//TODO Interrupt mode
	}

		//2 configure the speed of GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

		//3 configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
		//4 configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
		//5 configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~((0xF << (4 * temp2)));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}
/**
	* @brief  Deinitlizes the GPIO
	* @param  *pGPIOx : GPIO Base address
	* @param
	* @param
	* @return None
	*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}

}
/**
	* @brief  Enables or disables the peripheral clock for a given GPIO port
	* @param  *pGPIOx : GPIO peripheral base address
	* @param  EnorDi  : ENABLE(1) or DISABLE(0) macros
	* @param
	* @return None
	*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi== ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}

}
/**
	* @brief  Reads the data from a given GPIO pin.
	* @param  *pGPIOx : GPIO peripheral base address
	* @param  PinNum  : pin number that you want to read from
	* @param
	* @return : uint8_t data from GPIO pin (0 if low, 1 if high)
	*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNum) & 0x00000001);
	return value;

}
/**
 * @brief  Read the value of the entire GPIO input data register.
 * @param  pGPIOx : pointer to GPIO peripheral base address
 * @return uint16_t : 16‑bit value representing all 16 input pins
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
		uint16_t value;
		value = (uint16_t)(pGPIOx->IDR);
		return value;
}

/**
 * @brief  Write a value to a specific GPIO output pin.
 * @param  pGPIOx : pointer to GPIO peripheral base address
 * @param  PinNum : pin number to write to (0–15)
 * @param  val    : SET to set the pin(1), RESET to clear the pin(0)
 * @return None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t val){
	if(val == GPIO_PIN_SET){
		pGPIOx->ODR |= 1 << PinNum;
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNum);
	}

}
/**
 * @brief  Write a 16‑bit value to the GPIO output data register.
 * @param  pGPIOx : pointer to GPIO peripheral base address
 * @param  val    : 16‑bit value to write to all output pins at once
 * @return None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val){
	pGPIOx->ODR = val;
}

/**
 * @brief  Toggle the state of a specific GPIO output pin.
 * @param  pGPIOx : pointer to GPIO peripheral base address
 * @param  PinNum : pin number to toggle (0–15)
 * @return None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum){
	pGPIOx->ODR ^= (1 << PinNum);
}
/**
 * @brief  Configure and enable/disable an IRQ in the NVIC for a given GPIO line.
 * @param  IRQNum  : IRQ number corresponding to EXTI line
 * @param  IRQPrio : priority to assign to the IRQ (0 = highest)
 * @param  EnorDi  : ENABLE to enable the interrupt; DISABLE to disable it
 * @return None
 */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t IRQPrio, uint8_t EnorDi){


}

/**
 * @brief  Handle the pending EXTI interrupt for a specific GPIO pin.
 *         Clears the interrupt pending bit.
 * @param  PinNum : GPIO pin number for which to clear the EXTI flag
 * @return None
 */
void GPIO_IRQHandling(uint8_t PinNum){

}

