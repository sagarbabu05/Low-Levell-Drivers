/*
 * stm32f407xx_spi_driver.h
 *
 *      Author: banal
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t Txstate;
	uint8_t RxState;

}SPI_Handle_t;

/*
 * ....SPI application states
 */
#define SPI_READY      0
#define SPI_BUSY_IN_RX 1
#define spi_busy_in_tx 2

/*
 * Posssible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3
#define SPI_EVENT_CRC_ERR  4
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);



/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint32_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint32_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * other Peripheral Control APIs
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_CloseTransmission(SPI_Handle_t*pSPIHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t*pSPIHandle);
void SPI_CloseReception(SPI_Handle_t*pSPIHandle);

/*
 * Application EventCallback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv );
/*
 * @BIDIMODE
 */
#define SPI_BIDI_D     1
#define SPI_BIDI_UN_D  0

/*
 * @BIDIOE output enable in bidireational mode
 */

#define SPI_BIDIOE_0   0
#define SPI_BIDIOE_1   1

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH  1
#define SPI_CPHA_LOW   0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN  1
#define SPI_SSM_DI  0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG   (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG  (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG  (1 << SPI_SR_BSY)

/*
 * Posiiblle SPI Application States
 */

#define SPI_READY       0
#define SPI_BUSY_IN_RX  1
#define SPI_BUSY_IN_TX  2




#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
