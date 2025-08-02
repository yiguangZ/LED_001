/*
 * 003SPI_TX.c
 *
 *  Created on: Jul 30, 2025
 *      Author: yig88
 */
//PB15 as MOSI
//PB14 as MISO
//PB13 as SCLK
//PB12 as NSS
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi_driver.h"
#include<string.h>
void SPI2_GPIOInit(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_15;
	GPIO_Init(&SPIPins);
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_14;
	//GPIO_Init(&SPIPins);
	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_No_12;
	//GPIO_Init(&SPIPins);

}
void SPI2_Init(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2handle);
}
int main(void){
	char user_data[] = "Hello world";
	SPI2_GPIOInit();
	SPI2_Init();
	SPI_SSIConfig(SPI2, ENABLE);
	//enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	//have to check if SPI is busy
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
	//disbale
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);
	return 0;
}
