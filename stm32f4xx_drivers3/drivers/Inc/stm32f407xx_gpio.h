/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Jul 23, 2025
 *      Author: yig88
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_
#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber; //values from @GPIO_PIN
	uint8_t GPIO_PinMode; // values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; //values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl; //values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType; //values from @GPIO_PIN_OUT
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;
typedef struct
{
	GPIO_RegDef_t *pGPIOx; //holds the base address
	GPIO_PinConfig_t GPIO_PinConfig; //holds GPIO pin configuration settings

}GPIO_Handle_t;

/* @GPIO_PIN
 * GPIO pin numbers
 */
#define GPIO_PIN_No_0   0
#define GPIO_PIN_No_1   1
#define GPIO_PIN_No_2   2
#define GPIO_PIN_No_3   3
#define GPIO_PIN_No_4   4
#define GPIO_PIN_No_5   5
#define GPIO_PIN_No_6   6
#define GPIO_PIN_No_7   7
#define GPIO_PIN_No_8   8
#define GPIO_PIN_No_9   9
#define GPIO_PIN_No_10  10
#define GPIO_PIN_No_11  11
#define GPIO_PIN_No_12  12
#define GPIO_PIN_No_13  13
#define GPIO_PIN_No_14  14
#define GPIO_PIN_No_15  15

/* @GPIO_PIN_MODES
 * possible mode of a pin
 */
#define GPIO_MODE_IN  		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT 	6
/* @GPIO_PIN_OUT
 * GPIO output mode
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD 	1

/*@GPIO_PIN_SPEED
 * GPIO output speeds
 */
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3
/*  @GPIO_PIN_PUPD
 * GPIO pin pull up pull down macros
 */
#define GPIO_NO_PUPD 		0
#define GPIO_PU 			1
#define GPIO_PD				2


void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNum);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);

#endif /* INC_STM32F407XX_GPIO_H_ */
