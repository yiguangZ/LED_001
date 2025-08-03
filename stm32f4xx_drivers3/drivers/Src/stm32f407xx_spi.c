/*
 * stm32f407xx_spi.c
 *
 *  Created on: Jul 30, 2025
 *      Author: yig88
 */
#include "stm32f407xx_spi_driver.h"

/**
	* @brief  sets up the peripheral clock for SPI
	* @param  *pSPIx : pointer to the SPI base address
	* @param  EnorDi : takes either ENABLE(1) or DISABLE(0)to enable or disable clock
	* @param
	* @return None
	*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi== ENABLE){
			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3){
				SPI3_PCLK_EN();
			}
		}
		else{
			if(pSPIx == SPI1){
							SPI1_PCLK_DI();
						}
					else if(pSPIx == SPI2){
							SPI2_PCLK_DI();
						}
						else if (pSPIx == SPI3){
							SPI3_PCLK_DI();
						}
		}

}
/**
	* @brief  Init the SPI
	* @param  *pSPIHandle : pointer to the GPIO handle struct
	* @param
	* @param
	* @return None
	*/
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//enable SPI clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//configure SPI_CR1 reg
	uint32_t temp = 0;
	//1. configure device mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR;
	//2. configure bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		temp &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		temp |= (1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//BIDI mode should be cleared and Rx_only bit must be set
		temp &= ~(1<<SPI_CR1_BIDIMODE);
		temp |= (1<<SPI_CR1_RXONLY);
	}
	//configure clk speed
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed <<SPI_CR1_BR;
	//configure DFF
	temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//configure CPOl
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//config CPHA
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = temp;

}
/**
	* @brief  Deinitlizes the SPI
	* @param  *pSPIx : pointer to the SPI base address
	* @param
	* @param
	* @return None
	*/
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
			SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
			SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3){
			SPI3_REG_RESET();
	}

}
/*
 * gets the flag stat
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	else{
		return FLAG_RESET;
	}
}
/**
	* @brief  Send data from tx buffer to slave
	* @param  *pSPIx : pointer to the SPI base address
	* @param  *pTxBuffer : pointer to the Tx buffer where data is held
	* @param  Len : length of data
	* @return None
	*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len>0){
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == RESET);//blocking and polling
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else {
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
/**
	* @brief  Receive data from rx buffer
	* @param  *pSPIx : pointer to the SPI base address
	* @param  *pRxBuffer : pointer to the Rx buffer where data is held
	* @param  Len : length of data
	* @return None
	*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/**
 * @brief   enable/disable an IRQ in the NVIC for a given SPI line.
 * @param  IRQNum  : IRQ number corresponding to EXTI line
 * @param  EnorDi  : ENABLE to enable the interrupt; DISABLE to disable it
 * @return None
 */
void SPI_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi);
/**
 * @brief  Handle the pending EXTI interrupt for a specific SPI handle.
 *         Clears the interrupt pending bit.
 * @param  *pHandle : pointer to the SPI handle struct
 * @return None
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/**
 * @brief  Sets the priority of the interrupt
 * @param  IRQNum  : IRQ number corresponding to EXTI line
 * @param  IRQPriority : prioity of the interrupt
 * @return None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			pSPIx->CR1 |= (1<<SPI_CR1_SSI);
		}
		else{
			pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
		}
}
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR2 |= (1<<2);
	}
	else{
		pSPIx->CR2 &= ~(1<<2);
	}
}
