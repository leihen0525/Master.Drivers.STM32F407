/*
 * Drivers.STM32F4xx.Ether.PHY.h
 *
 *  Created on: 2019Äê5ÔÂ21ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_PHY_H_
#define DRIVERS_STM32F4XX_ETHER_PHY_H_

#include "Master.Stdint.h"

#include "Drivers.STM32F4xx.Ether.PHY.Define.h"

void Drivers_STM32F4xx_Ether_PHY_IO_Reset(void);
//int Drivers_STM32F4xx_Ether_PHY_Reset_Register(void);


int Drivers_STM32F4xx_Ether_PHY_SET_Clock(void);

int Drivers_STM32F4xx_Ether_PHY_Write(uint8_t PHYAddress, uint8_t PHYReg, uint16_t PHYValue);
int Drivers_STM32F4xx_Ether_PHY_Read(uint8_t PHYAddress, uint8_t PHYReg,uint16_t *PHYValue);


#endif /* DRIVERS_STM32F4XX_ETHER_PHY_H_ */
