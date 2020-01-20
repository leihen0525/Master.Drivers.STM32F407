/*
 * Drivers.STM32F4xx.Ether.c
 *
 *  Created on: 2019Äê5ÔÂ21ÈÕ
 *      Author: Master.HE
 */
#include <string.h>

#include "Define.h"
#include "Error.h"

//#include "__Sys.Device.Enum.h"
#include "Device\__Sys.Device.h"

#include "Module\Module.h"

#include "__Sys.API.h"
#include "STM32F4xx.h"

#include "Drivers.STM32F4xx.Ether.h"

#include "Drivers.STM32F4xx.Ether.MAC.h"
#include "Drivers.STM32F4xx.Ether.PHY.h"

#include "Drivers.STM32F4xx.Ether.Enum.h"

#include "Drivers.STM32F4xx.Ether.Struct.h"

Ether_DATA_Type Ether_DATA;

const __Sys_Device_OPS_Type __Device_OPS_Ether=
{
	.Device_Name="Ether",
	.Device_Args=Null,
	.Open=Null,//Drivers_STM32F4xx_Ether_Open,
	.Close=Null,
	.Read=Drivers_STM32F4xx_Ether_Read,
	.Write=Drivers_STM32F4xx_Ether_Write,
	.Control=Drivers_STM32F4xx_Ether_Control,

	.Info="MCU:STM32F4xx Module:Ether Version:0.0.1 "__DATE__" "__TIME__,
};

int Drivers_STM32F4xx_Ether_Setup(void)
{

	__Sys_Device_Register_Drivers(&__Device_OPS_Ether);

	Drivers_STM32F4xx_Ether_MAC_GPIO_Init();

	Drivers_STM32F4xx_Ether_MAC_Init();

	Ether_DATA.Handle_Rx_Event_Flag=__Sys_Event_Flag_Create(Null,false);
	Ether_DATA.Handle_Tx_Event_Flag=__Sys_Event_Flag_Create(Null,false);

	__Sys_IRQ_Register_Hook(ETH_IRQn,Drivers_STM32F4xx_Ether_IRQ,Null);
	//__Sys_IRQ_Set_Priority(ETH_IRQn,IRQ_Priority);
	__Sys_IRQ_Enable(ETH_IRQn);

	Ether_DATA.Rx_Index=0;
	Ether_DATA.Tx_Index=0;

	return Error_OK;
}

int Drivers_STM32F4xx_Ether_Open(void *Device_Args,int Mode)
{
	return Error_OK;
}
int Drivers_STM32F4xx_Ether_Read(void *Device_Args,long OffSet_Pos, void *Buffer, unsigned long Size,int TimeOut)
{
	if(Buffer==Null || Size==0 || Size>ETH_MAX_PACKET_SIZE)
	{
		return Error_Invalid_Parameter;
	}
	uint8_t Index=Ether_DATA.Rx_Index;
	ETH_DMADESCTypeDef *P_DATA;
	while(1)
	{
		P_DATA=&DMARxDscrTab[Ether_DATA.Rx_Index++];

		if(P_DATA->DES0_BIT.Rx.OWN==0)
		{
			memcpy(Buffer,(void*)P_DATA->Buffer1Addr,Size);

			P_DATA->DES0=ETH_DMARxDesc_OWN;

			if(ETH->DMASR_BIT.RBUS==1)
			{
				ETH->DMASR=ETH_DMASR_RBUS;
				ETH->DMARPDR = 0;
			}

			return Error_OK;
		}

		if(Ether_DATA.Rx_Index==Index)
		{
			__Sys_Event_Flag_Clear(Ether_DATA.Handle_Rx_Event_Flag);

			int Err=__Sys_Event_Flag_Wait(Ether_DATA.Handle_Rx_Event_Flag,TimeOut);
			if(Err!=Error_OK)
			{
				return Err;
			}
			//break;
		}

	}



	return Error_Undefined;
}
int Drivers_STM32F4xx_Ether_Write(void *Device_Args,long OffSet_Pos, const void *Buffer, unsigned long Size,int TimeOut)
{
	if(Buffer==Null || Size==0 || Size>ETH_MAX_PACKET_SIZE)
	{
		return Error_Invalid_Parameter;
	}

	uint8_t Index=Ether_DATA.Tx_Index;
	ETH_DMADESCTypeDef *P_DATA;
	while(1)
	{
		P_DATA=&DMATxDscrTab[Ether_DATA.Tx_Index++];

		if(P_DATA->DES0_BIT.Tx.OWN==0)
		{
			P_DATA->DES0_BIT.Tx.FS=1;
			P_DATA->DES0_BIT.Tx.LS=1;

			if(OffSet_Pos==1)
			{
				Ether_TX_DATA_Type *P_TX_DATA=(Ether_TX_DATA_Type*)Buffer;
				uint32_t Size_DATA=0;
				uint8_t *P_DATA_Buff=(uint8_t *)P_DATA->Buffer1Addr;
				for(int i=0;i<Size;i++)
				{

					memcpy((void*)&P_DATA_Buff[Size_DATA],P_TX_DATA[i].DATA,P_TX_DATA[i].Size);

					Size_DATA=Size_DATA+P_TX_DATA[i].Size;
				}

				P_DATA->DES1_BIT.Tx.TBS1=Size_DATA;
			}
			else
			{
				memcpy((void*)P_DATA->Buffer1Addr,Buffer,Size);

				P_DATA->DES1_BIT.Tx.TBS1=Size;
				//P_DATA->DES0=ETH_DMARxDesc_OWN;
			}
			P_DATA->DES0_BIT.Tx.OWN=1;

			if(ETH->DMASR_BIT.TBUS==1)
			{
				ETH->DMASR=ETH_DMASR_TBUS;
				ETH->DMATPDR = 0;
			}

			return Error_OK;
		}

		if(Ether_DATA.Tx_Index==Index)
		{
			__Sys_Event_Flag_Clear(Ether_DATA.Handle_Tx_Event_Flag);

			int Err=__Sys_Event_Flag_Wait(Ether_DATA.Handle_Tx_Event_Flag,TimeOut);
			if(Err!=Error_OK)
			{
				return Err;
			}
			//break;
		}

	}

	return Error_Undefined;
}
int Drivers_STM32F4xx_Ether_Control(void *Device_Args,int Cmd,unsigned long Args)
{
	switch (Cmd)
	{
		case Ether_Control_PHY_IO_Reset:
		{
			Drivers_STM32F4xx_Ether_PHY_IO_Reset();
			return Error_OK;
		}break;

		case Ether_Control_SET_MAC_Address:
		{
			if((uint8_t *)Args==Null)
			{
				return Error_Invalid_Parameter;
			}
			//uint64_t *P_DATA=Args;
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_Address((uint8_t *)Args);

			return Error_OK;
		}break;

		case Ether_Control_SET_MAC_Start:
		{
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_Start();
			return Error_OK;
		}break;

		case Ether_Control_SET_MAC_Stop:
		{
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_Stop();
			return Error_OK;
		}break;

		case Ether_Control_SET_MAC_10M_HalfDuplex:
		{
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_10M_HalfDuplex();
			return Error_OK;
		}break;
		case Ether_Control_SET_MAC_100M_HalfDuplex:
		{
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_100M_HalfDuplex();
			return Error_OK;
		}break;
		case Ether_Control_SET_MAC_10M_FullDuplex:
		{
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_10M_FullDuplex();
			return Error_OK;
		}break;
		case Ether_Control_SET_MAC_100M_FullDuplex:
		{
			Drivers_STM32F4xx_Ether_MAC_SET_MAC_100M_FullDuplex();
			return Error_OK;
		}break;
		case Ether_Control_PHY_Register_Write:
		{
			if((Ether_Control_PHY_DATA_Type *)Args==Null)
			{
				return Error_Invalid_Parameter;
			}

			Ether_Control_PHY_DATA_Type *P_DATA=(Ether_Control_PHY_DATA_Type *)Args;

			return Drivers_STM32F4xx_Ether_PHY_Write(P_DATA->PHY_Address,P_DATA->MII_Register,P_DATA->DATA);

		}break;
		case Ether_Control_PHY_Register_Read:
		{
			if((Ether_Control_PHY_DATA_Type *)Args==Null)
			{
				return Error_Invalid_Parameter;
			}

			Ether_Control_PHY_DATA_Type *P_DATA=(Ether_Control_PHY_DATA_Type *)Args;

			return Drivers_STM32F4xx_Ether_PHY_Read(P_DATA->PHY_Address,P_DATA->MII_Register,&P_DATA->DATA);
		}break;

		default:
		{
			return Error_Undefined;
		}break;
	}


	return Error_Undefined;
}

void Drivers_STM32F4xx_Ether_IRQ(void *Args,int IRQ_Index)
{
	if(ETH->DMASR_BIT.NIS==1)
	{
		if(ETH->DMASR_BIT.RS==1)
		{
			__Sys_Event_Flag_Set(Ether_DATA.Handle_Rx_Event_Flag);

			ETH->DMASR=ETH_DMASR_RS;
		}
		if(ETH->DMASR_BIT.ERS==1)
		{
			__Sys_Event_Flag_Set(Ether_DATA.Handle_Rx_Event_Flag);

			ETH->DMASR=ETH_DMASR_ERS;
		}
		if(ETH->DMASR_BIT.TS==1)
		{
			__Sys_Event_Flag_Set(Ether_DATA.Handle_Tx_Event_Flag);

			ETH->DMASR=ETH_DMASR_TS;
		}
		if(ETH->DMASR_BIT.ETS==1)
		{
			__Sys_Event_Flag_Set(Ether_DATA.Handle_Tx_Event_Flag);
			ETH->DMASR=ETH_DMASR_ETS;
		}

		ETH->DMASR=ETH_DMASR_NIS;
	}
}

__Sys_Device_Module_Init_Export(Drivers_STM32F4xx_Ether_Setup);

