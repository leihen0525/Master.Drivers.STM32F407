/*
 * Drivers.STM32F4xx.Ether.Struct.h
 *
 *  Created on: 2019Äê5ÔÂ24ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_STRUCT_H_
#define DRIVERS_STM32F4XX_ETHER_STRUCT_H_

#include "Master.Stdint.h"


typedef struct
{
	int Handle_Rx_Event_Flag;
	int Handle_Tx_Event_Flag;

	uint8_t	Rx_Index			:3;
	uint8_t Tx_Index			:3;

}Ether_DATA_Type;

typedef struct
{
	uint8_t *DATA;
	uint32_t Size;

}Ether_TX_DATA_Type;


//----------------------------------------

typedef struct
{
	uint16_t PHY_Address				:5;
	uint16_t 							:3;
	uint16_t MII_Register				:5;
	uint16_t 							:3;

	uint16_t DATA;

}Ether_Control_PHY_DATA_Type;




#endif /* DRIVERS_STM32F4XX_ETHER_STRUCT_H_ */
