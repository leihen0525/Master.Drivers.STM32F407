/*
 * Drivers.STM32F4xx.Ether.MAC.h
 *
 *  Created on: 2019Äê5ÔÂ21ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_MAC_H_
#define DRIVERS_STM32F4XX_ETHER_MAC_H_

#include "Drivers.STM32F4xx.Ether.MAC.Struct.h"

void Drivers_STM32F4xx_Ether_MAC_GPIO_Init(void);
int Drivers_STM32F4xx_Ether_MAC_Init(void);
void Drivers_STM32F4xx_Ether_MAC_Software_Reset(void);
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_Address(uint8_t *MAC_Address);
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_Start(void);
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_Stop(void);

void Drivers_STM32F4xx_Ether_MAC_SET_MAC_10M_HalfDuplex(void);
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_100M_HalfDuplex(void);
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_10M_FullDuplex(void);
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_100M_FullDuplex(void);

void ETH_DMATxDescChainInit(void);
void ETH_DMARxDescChainInit(void);


extern ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
extern ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */

#endif /* DRIVERS_STM32F4XX_ETHER_MAC_H_ */
