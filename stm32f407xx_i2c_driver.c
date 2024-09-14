/*
 * stm32f407xx_i2c_driver.c
 *      Author: banal
 */
#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

unit16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
unit8_t APB1_PreScaler[4] = {2,4,8,16,};
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
/*
 * peripheral control of I2C
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//p12BaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

/*
 * peripheral clock control of I2C
 */
void I2C_PeriClockControl(I2C_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		///
	}
}

uint32_t RCC_GetPCLKaValue(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, ahb1p;

	clksrc = ((RCC->CFGR >> 2) & 0X3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//ahb1

	temp = ((RCC->CFGR >> 10 ) & 0x7);

		if(temp < 8)
		{
			ahb1p = 1;
		}else
		{
			ahb1p = APB1_PreScaler[temp-4];
		}

		pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/*
 * ..............@I2C Init...............
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg |= pI2CHandle->I2C_Config_I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);

	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
		}else
		{
			//mode is fast mode
			tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
		}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);


}



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirmation that start generation is completed by checking the SB flag in the SR1
	// Note: untill SB is cleared SCL will be stretched
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. send the address of the slave with r /nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. confirm that address phase is completed by checking the ADDR flag in the SR1
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//note. Util ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(p2CHandle->pI2Cx);

	//6. send the data untill len becomes 0

	while (Len > 0)
	{
		while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //wait till TXE ie set
		pI2CHandle->pI2Cx-DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//Note: TXE=1, BTF=1, means that both SR and DR are empty and next trandmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)

	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop cindition.
	// Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);   // slaveAddr is Slave address + r/nw bit = 0
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;   // slaveAddr is Slave address + r/nw bit = 1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	uint32_t dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

/*
 *   ......Receive Data Funtion............
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//Note: Untill SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait untill address phase is completed by checking the ADR flag in teh SR1
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave

	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(p2CHandle->pI2Cx);

		//wait until RXNE becomes 1
		while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read dta in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx-DR;

		return;
	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		// clear the ADDR flag
		I2C_ClearADDRFlag(p2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait untill RXNE becomes 1
			while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

			if(i==2) //if last 2 byted are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx-DR;

			//increment the buffer addresss
			*pRxBuffer++;
		}
	}

	//re-enable ACKING
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
	   I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR2 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * ........Interrupt Priority of I2C ........
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1 first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx = IRQNumber % 4;

	uint_t shift_amount = (8 * iprx+section) + ( 8 - NO_PR_BITS_IMPLEMENTED) * (NIVIC_PR_BASE_ADDR +iprx ) |= ( IRQPriority << shift_amount );

}

/*
 * ..........Interrupt funtion when sending data...................
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	uint8_t busystate = pI2CHandle->TxRxState
			;
			if(busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)
			{
				pI2CHandle->pTxBuffer = pTxBuffer;
				pI2CHandle->TxLen     = Len;
				pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
				pI2CHandle->DevAddr   = SlaveAddr;
				pI2CHandle->Sr        = Sr;

				//Implement code to Generate START Condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);;

				//Implent the code to enable ITBUFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

				//Implent the code to enable ITEVFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

				//Implent the code to enable ITERREN Control Bit
			    pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
			}
			return busystate;

}

/*
 * ..........Interrupt funtion when receving data...................
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	uint8_t busystate = pI2CHandle->

				if(busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)
				{
					pI2CHandle->pRxBuffer = ;
					pI2CHandle->TxLen     = ;
					pI2CHandle->TxRxState = ;
					pI2CHandle->RxSize    = Len;
					pI2CHandle->DevAddr   = ;
					pI2CHandle->Sr        = sr;

					//Implement code to Generate START Condition

					//Implent the code to enable ITBUFEN Control Bit
					pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

					//Implent the code to enable ITEVFEN Control Bit
				}
}









