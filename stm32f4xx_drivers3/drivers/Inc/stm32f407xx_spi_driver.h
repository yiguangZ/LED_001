/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 30, 2025
 *      Author: yig88
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;
/*
 * SPI Device mode
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0
/*
 * SPI Bus config
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3
/*
 * SPI clock speeds
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7
/*
 * SPI DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1
/*
 * CPOL and CPHA
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0
/*
 * SSM
 */
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0
/*
 * SPI Flag
 */
#define SPI_TXE_FLAG		(1<<1)
#define SPI_RXNE_FLAG		(1<<0)
#define SPI_BSY_FLAG		(1<<7)


/******************************************************************************************************************
 * APIs supported by this SPI driver
 *****************************************************************************************************************/
/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
/*
 * SPI init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
/*
 * SPI Deinit
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ config and ISR handling
 */
void SPI_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
/*
 * other
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

