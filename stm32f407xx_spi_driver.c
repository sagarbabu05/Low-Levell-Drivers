/*
 * stm32f407xx_spi_driver.c
 *
 *      Author: banal
 */
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else if(EnorDi == DISABLE)
		{
			if(pSPIx == SPI1)
					{
				         SPI1_PCLK_DI();
					}else if(pSPIx == SPI2)
					{
						SPI2_PCLK_DI();;
					}else if(pSPIx == SPI3)
					{
						SPI3_PCLK_DI();
					}else if(pSPIx == SPI4)
					{
						SPI4_PCLK_DI();
					}
		}
}


/*
 * .........SPI Init........
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1 register
	uint32_t tempreg = 0;
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE)
	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. config the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//MODE SHOULD BE CLEAR
		tempreg &= ~( 1<< SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//MODE SHOULD BE SET
		tempreg |= (1 << SPI_CR1_BIDIMODE );
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be clear
		tempreg &= (1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY );
	}
	//3. config the spi serial clock speed (baud rate
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. config the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*
 * .......Dinit....
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG RESET;
}

/*
 * ....Send Data.....
 */
void SPI_SendData(SPI_RegDef_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1, wait until TXE IS SET
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. CHECK THE DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit dff
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			Len--;
			pTxBuffer++;
		}

	}
}

/*
 * ...SPI_ReceiveData.....
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1, wait until RXE IS SET
			while(SPI_GetFlagStatus(pSPIx,SPI_RXE_FLAG) == FLAG_RESET);

			//2. CHECK THE DFF bit in CR1
			if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
			{
				//16 bit dff
				//1. load the data FROM DR to RxBuffer
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}

		}
}

/*
 * ...Send Data Interrupt API.....
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->Txstate;

	if(state != SPI_BUSY_IN_TX)
	{

	 //1. Save the Tx buffer address and Len information in some global variables
	 pSPIHandle->pTxBuffer = pTxBuffer;
	 pSPIHandle->TxLen = Len;

	 //2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral util transmission is over
	 pSPIHandle->TxState = SPI_BUSY_IN_TX;

	 //3. Enable the TXEIE control bit to get interrupt whenever txe flag is =set in SR
	  pSPIHandle->TxState = SPI_BUSY_IN_TX;

	 //4. dATA tRANSMISSION WILL BE HANBLED BY THE isr CODE (WILL IMPLEMENT LATER)
	  pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;

}


/*
 * ...Receive Data Interrupt API.....
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint32_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->Rxstate;

		if(state != SPI_BUSY_IN_RX)
		{

		 //1. Save the Tx buffer address and Len information in some global variables
		 pSPIHandle->pRxBuffer = pRxBuffer;
		 pSPIHandle->RxLen = Len;

		 //2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral util transmission is over
		 pSPIHandle->RxState = SPI_BUSY_IN_RX;

		 //3. Enable the TXEIE control bit to get interrupt whenever txe flag is =set in SR
		  pSPIHandle->RxState = SPI_BUSY_IN_RX;

		 //4. dATA tRANSMISSION WILL BE HANBLED BY THE isr CODE (WILL IMPLEMENT LATER)
		  pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXnEIE);
		}

		return state;

}

/*
 *.....SPI Interrupt Handler.....
 *..
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
			{
				//16 bit dff
				//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}else
			{
				//8 bit DFF
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->Len--;
				pSPIHandle->pTxBuffer++;
			}
	if(!pSPIHandle-> TxLen)
	{
		//this prevents interrupts from setting up of txe flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
				{
					//16 bit dff
					//1. load the data in to the DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->prxBuffer);
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
					(uint16_t*)pSPIHandle->pRxBuffer++;
				}else
				{
					//8 bit DFF
					pSPIHandle->pSPIx->DR = *pSPIHandle->pRxBuffer;
					pSPIHandle->Len--;
					pSPIHandle->pRxBuffer++;
				}
		if(!pSPIHandle-> RxLen)
		{
			//this prevents interrupts from setting up of txe flag
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//1. clear the ovr flag
	if(pSPIHandle->TxState !=SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp
	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t*pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t*pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNIE);
				pSPIHandle->pRxBuffer = NULL;
				pSPIHandle->RxLen = 0;
				pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t*pSPIHandle)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp= pSPIX->SR;
	(void)temp;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle();
	}
	//lets check for RXE
		temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
		temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

		if(temp1 && temp2)
			{
				//handle RXE
				spi_rxne_interrupt_handle();
			}

    // check for over flag
		temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
		temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

		if(temp1 && temp2)
		{
			//handle ovr error
			spi_ovr_err_interrupt_handle();
		}
}


/*
 * ....SPI_peripheralControl......
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);

	}
}
/*
 * ...SPI_SSOEConfig....
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv )
{

}
