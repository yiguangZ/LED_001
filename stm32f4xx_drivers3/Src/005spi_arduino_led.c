/*
 * 005spi_arduino_led.c
 *
 *  Created on: Aug 3, 2025
 *      Author: yig88
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi_driver.h"
#include<stdio.h>
#include<string.h>

extern void initialise_monitor_handles();


#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0x51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0x53
#define COMMAND_ID_READ		0x54

#define LED_ON 				1
#define LED_OFF				0

#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

#define LED_PIN				9




void SPI2_GPIOInit(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_15;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_14;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_12;
	GPIO_Init(&SPIPins);

}
void SPI2_Init(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;//Hardware
	SPI_Init(&SPI2handle);
}
void GPIO_ButtonInit(void){
	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIOButton);
}
void delay(void){
	for(uint32_t i = 0; i<500000/2; i++);
}
uint8_t SPI_verifyresponse(uint8_t ackbyte){
	if (ackbyte == (uint8_t)0xF5){
		//ack
		return 1;
	}
	else{
		return 0;
	}
}

int main(void){
	initialise_monitor_handles();
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	printf("Application is running\n");
	GPIO_ButtonInit();
	SPI2_GPIOInit();
	SPI2_Init();
	printf("SPI2 Init done\n");
	//enable SPI2 peripheral
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_No_0) );
		delay();
		SPI_PeripheralControl(SPI2, ENABLE);
		//first send command 1 CMD_LED_CTRL
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		SPI_SendData(SPI2, &commandcode, 1);
		//do dummy read since everytime master sends one byte it gets one byte back
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//send some dummy bits (1byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_verifyresponse(ackbyte)){
			//send arguments(pin num,val)
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			SPI_ReceiveData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL executed\n");

		}
		//Command sensor read
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_No_0) );
		delay();
		commandcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//send some dummy bits (1byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_verifyresponse(ackbyte)){
			//send arguments(pin num,val)
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			delay();
			SPI_SendData(SPI2, &dummy_write, 1);
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("CMD_SENSOR_READ %d\n", analog_read);

		}
		//COMMAND_LED_READ
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_No_0) );
		delay();
		commandcode = COMMAND_LED_READ;
		SPI_SendData(SPI2, &commandcode, 1);
		//do dummy read since everytime master sends one byte it gets one byte back
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//send some dummy bits (1byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_verifyresponse(ackbyte)){
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			delay();
			SPI_SendData(SPI2, &dummy_write, 1);
			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("CMD_LED_READ %d\n", led_status);

		}
		//COMMAND_PRINT
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_No_0) );
		delay();
		commandcode = COMMAND_PRINT;
		SPI_SendData(SPI2, &commandcode, 1);
		//do dummy read since everytime master sends one byte it gets one byte back
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//send some dummy bits (1byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t message[] = "Hello ! How are you??";
		if(SPI_verifyresponse(ackbyte)){
			args[0] = strlen((char*)message);
			SPI_SendData(SPI2, args, 1);
			SPI_SendData(SPI2, message, args[0]);
			printf("CMD_PRINT done\n");

		}
		//COMMAND_ID_READ
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_No_0) );
		delay();
		commandcode = COMMAND_ID_READ;
		SPI_SendData(SPI2, &commandcode, 1);
		//do dummy read since everytime master sends one byte it gets one byte back
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//send some dummy bits (1byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[10];
		uint32_t i = 0;
		if(SPI_verifyresponse(ackbyte)){
			//read 10 bytes of id
			for(i=0;i<10;i++){
				SPI_SendData(SPI2, &dummy_read, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}
			id[11] = '\0';
			printf("COMMAND_ID: %s \n", id);
		}
		//have to check if SPI is busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		//disbale
		SPI_PeripheralControl(SPI2, DISABLE);
		printf("SPI Communication closed");
	}
	return 0;
}



