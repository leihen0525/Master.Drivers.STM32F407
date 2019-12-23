/*
 * Drivers.STM32F4xx.Ether.h
 *
 *  Created on: 2019Äê5ÔÂ21ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_H_
#define DRIVERS_STM32F4XX_ETHER_H_

int Drivers_STM32F4xx_Ether_Init(void);
int Drivers_STM32F4xx_Ether_Open(void *Device_Args,int Mode);
int Drivers_STM32F4xx_Ether_Read(void *Device_Args,long OffSet_Pos, void *Buffer, unsigned long Size,int TimeOut);
int Drivers_STM32F4xx_Ether_Write(void *Device_Args,long OffSet_Pos, const void *Buffer, unsigned long Size,int TimeOut);
int Drivers_STM32F4xx_Ether_Control(void *Device_Args,int Cmd,unsigned long Args);

void Drivers_STM32F4xx_Ether_IRQ(void *Args,int IRQ_Index);

#endif /* DRIVERS_STM32F4XX_ETHER_H_ */
