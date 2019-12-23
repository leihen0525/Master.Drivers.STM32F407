/*
 * Drivers.STM32F4xx.Ether.Enum.h
 *
 *  Created on: 2019Äê5ÔÂ24ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_ENUM_H_
#define DRIVERS_STM32F4XX_ETHER_ENUM_H_

typedef enum
{
	Ether_Control_PHY_IO_Reset							=0,

	Ether_Control_SET_MAC_Address,

	Ether_Control_SET_MAC_Start,
	Ether_Control_SET_MAC_Stop,

	Ether_Control_SET_MAC_10M_HalfDuplex,
	Ether_Control_SET_MAC_100M_HalfDuplex,
	Ether_Control_SET_MAC_10M_FullDuplex,
	Ether_Control_SET_MAC_100M_FullDuplex,

	Ether_Control_PHY_Register_Write,
	Ether_Control_PHY_Register_Read,
}Ether_Control_Type;

#endif /* DRIVERS_STM32F4XX_ETHER_ENUM_H_ */
