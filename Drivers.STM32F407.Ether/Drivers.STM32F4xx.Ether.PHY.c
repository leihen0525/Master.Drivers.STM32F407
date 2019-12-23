/*
 * Drivers.STM32F4xx.Ether.PHY.c
 *
 *  Created on: 2019年5月21日
 *      Author: Master.HE
 */
#include "Error.h"

#include "__Sys.API.h"

#include "STM32F4xx.h"



#include "Drivers.STM32F4xx.Ether.PHY.h"

void Drivers_STM32F4xx_Ether_PHY_IO_Reset(void)
{
	GPIO_ResetBits(GPIOD , GPIO_Pin_3);					//硬件复位LAN8720
	__Sys_Scheduling_Sleep_Task(50);
	GPIO_SetBits(GPIOD , GPIO_Pin_3);				 	//复位结束
}
//int Drivers_STM32F4xx_Ether_PHY_Reset_Register(void)
//{
//	int Err;
//	if((Err=Drivers_STM32F4xx_Ether_PHY_Write(ETHERNET_PHY_ADDRESS, PHY_BCR, PHY_Reset))!=Error_OK)
//	{
//		return Err;
//	}
//
//	/* Delay to assure PHY reset */
//	__Sys_Scheduling_Sleep_Task(50);
//
//	return Error_OK;
//}

int Drivers_STM32F4xx_Ether_PHY_SET_Clock(void)
{
	RCC_ClocksTypeDef  rcc_clocks;

	RCC_GetClocksFreq(&rcc_clocks);


	/* Set CR bits depending on hclk value */
	if((rcc_clocks.HCLK_Frequency >= 20000000)&&(rcc_clocks.HCLK_Frequency < 35000000))
	{
	/* CSR Clock Range between 20-35 MHz */
	//tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div16;
		ETH->MACMIIAR_BIT.CR=2;
	}
	else if((rcc_clocks.HCLK_Frequency >= 35000000)&&(rcc_clocks.HCLK_Frequency < 60000000))
	{
	/* CSR Clock Range between 35-60 MHz */
	//tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div26;
		ETH->MACMIIAR_BIT.CR=3;
	}
	else if((rcc_clocks.HCLK_Frequency >= 60000000)&&(rcc_clocks.HCLK_Frequency < 100000000))
	{
	/* CSR Clock Range between 60-100 MHz */
	//tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;
		ETH->MACMIIAR_BIT.CR=0;
	}
	else if((rcc_clocks.HCLK_Frequency >= 100000000)&&(rcc_clocks.HCLK_Frequency < 150000000))
	{
	/* CSR Clock Range between 100-150 MHz */
	//tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div62;
		ETH->MACMIIAR_BIT.CR=1;
	}
	else /* ((hclk >= 150000000)&&(hclk <= 168000000)) */
	{
	/* CSR Clock Range between 150-168 MHz */
	//tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div102;
		ETH->MACMIIAR_BIT.CR=4;
	}

	return Error_OK;
}

int Drivers_STM32F4xx_Ether_PHY_Write(uint8_t PHYAddress, uint8_t PHYReg, uint16_t PHYValue)
{
	if(PHYAddress>0x1F || PHYReg>0x1F)
	{
		return Error_Invalid_Parameter;
	}

	if(ETH->MACMIIAR_BIT.MB==1)
	{
		return Error_Busy;
	}

	ETH->MACMIIDR = PHYValue;

	ETH->MACMIIAR_BIT.PA=PHYAddress;
	ETH->MACMIIAR_BIT.MR=PHYReg;
	ETH->MACMIIAR_BIT.MW=1;

	ETH->MACMIIAR_BIT.MB=1;


	__IO uint32_t timeout = 0;


	/* Check for the Busy flag */
	do
	{
		timeout++;

	} while ((ETH->MACMIIAR_BIT.MB==1) && (timeout < (uint32_t)PHY_WRITE_TO));
	/* Return ERROR in case of timeout */
	if(timeout == PHY_WRITE_TO)
	{
		return Error_Time_Out;
	}

	/* Return SUCCESS */
	return Error_OK;
}

int Drivers_STM32F4xx_Ether_PHY_Read(uint8_t PHYAddress, uint8_t PHYReg,uint16_t *PHYValue)
{
	if(PHYAddress>0x1F || PHYReg>0x1F || PHYValue==Null)
	{
		return Error_Invalid_Parameter;
	}

	if(ETH->MACMIIAR_BIT.MB==1)
	{
		return Error_Busy;
	}



	ETH->MACMIIAR_BIT.PA=PHYAddress;
	ETH->MACMIIAR_BIT.MR=PHYReg;
	ETH->MACMIIAR_BIT.MW=0;

	ETH->MACMIIAR_BIT.MB=1;


	__IO uint32_t timeout = 0;


	/* Check for the Busy flag */
	do
	{
		timeout++;

	} while ((ETH->MACMIIAR_BIT.MB==1) && (timeout < (uint32_t)PHY_READ_TO));
	/* Return ERROR in case of timeout */
	if(timeout == PHY_READ_TO)
	{
		return Error_Time_Out;
	}

	*PHYValue=(uint16_t)ETH->MACMIIDR;

	/* Return SUCCESS */
	return Error_OK;
}
