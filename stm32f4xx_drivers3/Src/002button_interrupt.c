/*
 *
 *
 *  Created on: Jul 26, 2025
 *      Author: yig88
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include<string.h>
void delay(void){
	for(uint32_t i = 0; i<500000/2; i++);
}

int main(void){
	GPIO_Handle_t GpioLed, GPIOBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOBtn);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_No_12, GPIO_PIN_RESET);
	//IRQ config
	GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay();//200ms
	GPIO_IRQHandling(GPIO_PIN_No_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_No_12);
}
