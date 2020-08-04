/**
  ******************************************************************************
  * @file    rtl8721dhp_usi_ssi.c
  * @author
  * @version V1.0.0
  * @date    2017-11-27
  * @brief   This file contains all the functions prototypes for USI-SPI:
  *		- Initialization 
  *		- Clock polarity and phase setting
  *		- SPI data frame size setting
  *		- SPI baud rate setting
  *		- Receive/Send data interface
  *		- Get TRx FIFO valid entries
  *		- check SPI device busy status
  *		- DMA transfers management
  *		- Interrupts and management 
  *
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
  * Copyright(c) 2020, Seeed Technology Co.,Ltd. All rights reserved.
  ****************************************************************************** 
  */

#include "ameba_soc.h"

/** @addtogroup USI_SPI_Exported_Functions
*@verbatim
  *      
  *          ===================================================================
  *                                 	How to use this driver
  *          ===================================================================
  *			1. Enable peripheral clock using the following functions 
  *
  *
  *			2. Configure the SPIx pinmux.
  *
  *
  *			3. Disable the SPI using the USI_SSI_Cmd() function 
  *
  *			4. Program the Polarity,Phase,Transfer Mode,Baud Rate Prescaler,DataFrameSize,
  *				Interrupt TRx Threshold level,DMA TRx Threshold level and other parameters using
  *				the USI_SSI_Init() function.
  *
  *			5. Enable the USI-SPI using the USI_SSI_Cmd() function 
  *
  *			6. When using poll:
  *				-Using USI_SSI_Writeable() function make sure that the transmit FIFO is not full,
  *				then using USI_SSI_WriteData() function to send data
  *			     
  *				-Using USI_SSI_Readable() function make sure that the receive FIFO is not empty,
  *				then using USI_SSI_ReadData() function to receive data
  *
  *			7. Enable corresponding interrupt using the function  USI_SSI_INTConfig() if you need to 
  *				use interrupt mode. 
  *
  *			8. When using the DMA mode 
  *				- Configure & Initialize the DMA 
  *				- Active the DMA Tx or Rx using USI_SSI_SetDmaEnable() function
  *
  * @endverbatim
  */
  
/**
  * @brief  Fills each USI_SSI_InitStruct member with its default value.
  * @param  USI_SSI_InitStruct: pointer to a USI_SSI_InitTypeDef structure which will be 
  *         initialized.
  * @retval None
  */
void
USI_SSI_StructInitEx(USI_SSI_InitTypeDef* USI_SSI_InitStruct)
{
	USI_SSI_InitStruct->USI_SPI_RxThresholdLevel  = 0;   // if number of entries in the RX FIFO  > RxThresholdLevel, RX interrupt asserted
	USI_SSI_InitStruct->USI_SPI_TxThresholdLevel  = 30;  // if number of entries in the TX FIFO <= TxThresholdLevel, TX interrupt asserted
	USI_SSI_InitStruct->USI_SPI_DmaRxDataLevel    = 3;
	USI_SSI_InitStruct->USI_SPI_DmaTxDataLevel    = 56;
	USI_SSI_InitStruct->USI_SPI_ClockDivider      = 6;
	USI_SSI_InitStruct->USI_SPI_DataFrameNumber   = 0;
	USI_SSI_InitStruct->USI_SPI_DataFrameSize     = USI_SPI_DFS_8_BITS;
	USI_SSI_InitStruct->USI_SPI_InterruptMask     = 0x0;
	USI_SSI_InitStruct->USI_SPI_SclkPhase         = USI_SPI_SCPH_TOGGLES_IN_MIDDLE;
	USI_SSI_InitStruct->USI_SPI_SclkPolarity      = USI_SPI_SCPOL_INACTIVE_IS_LOW;
	USI_SSI_InitStruct->USI_SPI_TransferMode      = USI_SPI_TMOD_TR;

	USI_SSI_InitStruct->USI_SPI_RxSampleDelay     = 0;
	USI_SSI_InitStruct->USI_SPI_SSTogglePhase     = 0;
}


/**
  * @brief    Initializes the USI-SPI registers according to the specified parameters 
  *         in USI_SSI_InitStruct.
  * @param  usi_dev: where spi_dev can be USI0_DEV.
  * @param  USI_SSI_InitStruct: pointer to a USI_SSI_InitTypeDef structure that contains 
  *         the configuration information for the USI-SPI peripheral.
  * @retval None
  */

void USI_SSI_InitEx(USI_TypeDef *usi_dev, USI_SSI_InitTypeDef *USI_SSI_InitStruct)
{
	u32 TempValue1  = 0, TempValue2 = 0;

	assert_param(IS_USI_SPI_RxThresholdLevel(USI_SSI_InitStruct->USI_SPI_RxThresholdLevel));
	assert_param(IS_USI_SPI_TxThresholdLevel(USI_SSI_InitStruct->USI_SPI_TxThresholdLevel));

	/* Set USI to SPI mode */
	TempValue1 = usi_dev->USI_MODE_CTRL;
	TempValue1 &= ~USI_SERIAL_MODE;
	TempValue1 |= USI_SERIAL_SPI_MODE;
	usi_dev->USI_MODE_CTRL = TempValue1;

	/* Disable SPI and Tx/Rx Path, for some bits in SPI_CTRL are writeable only when Tx/Rx path are both disable.*/	
	USI_SSI_Reset(usi_dev, ENABLE);
	USI_SSI_TRxPath_Cmd(usi_dev, USI_SPI_RX_ENABLE | USI_SPI_TX_ENABLE, DISABLE);

	/* Set SPI Control Register */
	TempValue1 = 0;
	TempValue1 |= USI_SSI_InitStruct->USI_SPI_DataFrameSize;
	TempValue1 |= (USI_SSI_InitStruct->USI_SPI_SclkPhase << 6);
	TempValue1 |= (USI_SSI_InitStruct->USI_SPI_SclkPolarity << 7);

	/* Master Only */
	if (USI_SSI_InitStruct->USI_SPI_Role & USI_SPI_MASTER) {
		TempValue1 |= (USI_SSI_InitStruct->USI_SPI_ClockDivider) << 16;
		TempValue1 |= (USI_SSI_InitStruct->USI_SPI_RxSampleDelay) << 8;
		TempValue1 |= (USI_SSI_InitStruct->USI_SPI_SSTogglePhase) << 5;
		TempValue1 |= USI_SPI_MASTER_MODE;

		TempValue2 |= (USI_SSI_InitStruct->USI_SPI_DataFrameNumber) << 16;
	}
	usi_dev->SPI_CTRL = TempValue1;

	/* Set Tx/Rx FIFO Threshold Level*/
	usi_dev->TX_FIFO_CTRL = USI_SPI_TX_FIFO_DEPTH - USI_SSI_InitStruct->USI_SPI_TxThresholdLevel;
	/*
	 * TX interrupt when (empty-entries >= 64 - USI_SPI_TxThresholdLevel),
	 * (USI_SPI_TxThresholdLevel >= 64 - empty-entries = valid-entries)
	 * (valid-entries <= USI_SPI_TxThresholdLevel)
	 */
	usi_dev->RX_FIFO_CTRL = USI_SSI_InitStruct->USI_SPI_RxThresholdLevel + 1;

	/* Set interrupt */
	usi_dev->INTERRUPT_ENABLE = USI_SSI_InitStruct->USI_SPI_InterruptMask & USI_SPI_INTERRUPT_MASK;

	/*DMA level set */
	USI_SSI_SetDmaLevel(usi_dev, USI_SSI_InitStruct->USI_SPI_DmaTxDataLevel, USI_SSI_InitStruct->USI_SPI_DmaRxDataLevel);

	/* Set Tx/Rx Path enable */
	switch(USI_SSI_InitStruct->USI_SPI_TransferMode){
	case USI_SPI_TMOD_TO:
		TempValue2 |= USI_SPI_TX_ENABLE;
		break;
	case USI_SPI_TMOD_RO:
		TempValue2 |= USI_SPI_RX_ENABLE;
		break;
	case USI_SPI_TMOD_TR:
		TempValue2 |= (USI_SPI_TX_ENABLE | USI_SPI_RX_ENABLE);
		break;
	default:
		break;
	}
	usi_dev->SPI_TRANS_EN  = TempValue2;

	/* Enable SPI. SPI can work normally when Tx/Rx Path and all logic are released.*/
	USI_SSI_Reset(usi_dev, DISABLE);
}

/**
  * @brief  Enables or disables USI-SPI peripheral.
  * @param  usi_dev: where usi_dev can be USI0_DEV.
  * @param  NewStatus: This parameter can be one of the following values:
  *            @arg ENABLE
  *            @arg DISABLE
  * @retval None
  */

void USI_SSI_CmdEx(USI_TypeDef *usi_dev, u32 NewStatus)
{
	USI_SSI_TRxPath_Cmd(usi_dev, USI_SPI_RX_ENABLE | USI_SPI_TX_ENABLE, NewStatus);
}

/**
  * @brief  Reset or release USI-SPI peripheral.
  * @param  usi_dev: where usi_dev can be USI0_DEV.
  * @param  NewStatus: This parameter can be one of the following values:
  *            @arg ENABLE reset the device
  *            @arg DISABLE
  * @retval None
  */
void USI_SSI_Reset(USI_TypeDef *usi_dev, u32 NewStatus)
{
	if (NewStatus == DISABLE) {
		usi_dev->SW_RESET |= USI_SW_RESET_RSTB | USI_SW_RESET_RXFIFO_RSTB |
			USI_SW_RESET_TXFIFO_RSTB | USI_SW_RESET_RX_RSTB | USI_SW_RESET_TX_RSTB;
	} else {
		usi_dev->SW_RESET &= ~USI_SW_RESET_RSTB;
	}
}

/**
  * @brief  Enables or disables slave .
  * @note  Valid only when the device is configured as a master.
  * @param  spi_dev: where spi_dev can be USI0_DEV.
  * @param  SlaveIndex: the index of slave to be selected.
  * @retval None
  */
void USI_SSI_SetSlaveEnable(USI_TypeDef *spi_dev, u32 SlaveIndex)
{
	(void) spi_dev;
	(void) SlaveIndex;
	/* empty implementation */
	return;
}

/******************* (C) COPYRIGHT 2016 Realtek Semiconductor *****END OF FILE****/

