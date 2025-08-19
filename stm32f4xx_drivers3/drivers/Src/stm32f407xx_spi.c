/*
 * stm32f407xx_spi.c
 *
 *  Created on: Jul 30, 2025
 *      Author: yig88
 */
#include "stm32f407xx_spi_driver.h"
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len>0){
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)RESET);//blocking and polling
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else {
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}
/**
 * @brief   enable/disable an IRQ in the NVIC for a given SPI line.
 * @param  IRQNum  : IRQ number
 * @param  EnorDi  : ENABLE to enable the interrupt; DISABLE to disable it
 * @return None
 */
void SPI_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNum <=31){
			*NVIC_ISER0 |= (1<<IRQNum);

		}
		else if(IRQNum >31 && IRQNum<64){
			*NVIC_ISER1 |= (1<<(IRQNum%32));

		}
		else if(IRQNum>=64 && IRQNum<96){
			*NVIC_ISER3 |= (1<<(IRQNum%64));

		}
	}
	else{
		if(IRQNum <=31){
			*NVIC_ICER0 |= (1<<IRQNum);

		}
		else if(IRQNum >31 && IRQNum<64){
			*NVIC_ICER1 |= (1<<(IRQNum%32));

		}
		else if(IRQNum>=64 && IRQNum<96){
			*NVIC_ICER3 |= (1<<(IRQNum%64));

		}

	}

}
/**
 * @brief  Handle the pending EXTI interrupt for a specific SPI handle.
 *         Clears the interrupt pending bit.
 * @param  *pHandle : pointer to the SPI handle struct
 * @return None
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1,temp2;
	temp1 = pHandle->pSPIx->SR & (1<<1); //TXE flag
	temp2 = pHandle->pSPIx->CR2 & (1<<7); //TXEIE flag
	if (temp1 && temp2){
		//handle txe
		spi_txe_interrupt_handle(pHandle);
	}
	temp1 = pHandle->pSPIx->SR & (1<<0); //RXNE flag
	temp2 = pHandle->pSPIx->CR2 & (1<<6); //RXNEIE flag
	if (temp1 && temp2){
		//handle rxe
		spi_rxne_interrupt_handle(pHandle);
	}
	temp1 = pHandle->pSPIx->SR & (1<<6); //OVR flag
	temp2 = pHandle->pSPIx->CR2 & (1<<5); //ERRIE
	if (temp1 && temp2){
		//handle rxe
		spi_ovr_err_interrupt_handle(pHandle);
	}
	//check
}
/**
 * @brief  Sets the priority of the interrupt
 * @param  IRQNum  : IRQ number corresponding to EXTI line
 * @param  IRQPriority : prioity of the interrupt
 * @return None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum %4;
	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= IRQPriority << shift_amount;
}

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
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t status = pSPIHandle->TxState;
	if(status != SPI_BUSY_IN_TX){

		//1. save the Tx buffer addr and Len info in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. mark SPI state as busy so no other code can take over same SPI peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3 enable TXEIE control bit to ge interrupt whenver TXE flag is set in Status Reg
		pSPIHandle->pSPIx->CR2 |= (1<<7);
	}
	return status;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t status = pSPIHandle->RxState;
	if(status != SPI_BUSY_IN_RX){

		//1. save the Tx buffer addr and Len info in global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. mark SPI state as busy so no other code can take over same SPI peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3 enable TXEIE control bit to ge interrupt whenver TXE flag is set in Status Reg
		pSPIHandle->pSPIx->CR2 |= (1<<6);
	}
	return status;
}
//helper functions
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
		if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)){
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}
		else {
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if(!pSPIHandle->TxLen){
			//TxLen is 0, close SPI communication
			//Tx is over so clears TXEIE
			SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		}
}
void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)){
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else {
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen){
		//RxLen is 0, close SPI communication
		//Rx is over so clears RXNEIE
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//clear OVR flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pHandle){
	pHandle->pSPIx->CR2 &= ~(1<<7);
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pHandle){
	pHandle->pSPIx->CR2 &= ~(1<<6);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t event){
	//weak implementation application may override this function
}

