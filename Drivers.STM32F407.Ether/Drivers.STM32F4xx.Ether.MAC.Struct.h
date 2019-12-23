/*
 * Drivers.STM32F4xx.Ether.MAC.Struct.h
 *
 *  Created on: 2019Äê5ÔÂ22ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_MAC_STRUCT_H_
#define DRIVERS_STM32F4XX_ETHER_MAC_STRUCT_H_

#include "Master.Stdint.h"

#include "Drivers.STM32F4xx.Ether.MAC.Define.h"

typedef struct
{
	union
	{
		volatile uint32_t DES0;                /*!< Status */
		union
		{
			struct
			{
				volatile uint32_t DB					:1;
				volatile uint32_t UF					:1;
				volatile uint32_t ED					:1;
				volatile uint32_t CC					:4;
				volatile uint32_t VF					:1;

				volatile uint32_t EC					:1;
				volatile uint32_t LCO					:1;
				volatile uint32_t NC					:1;
				volatile uint32_t LCA					:1;
				volatile uint32_t IPE					:1;
				volatile uint32_t FF					:1;
				volatile uint32_t JT					:1;
				volatile uint32_t ES					:1;

				volatile uint32_t IHE					:1;
				volatile uint32_t TTSS					:1;
				volatile uint32_t 						:2;
				volatile uint32_t TCH					:1;
				volatile uint32_t TER					:1;
				volatile uint32_t CIC					:2;

				volatile uint32_t						:1;
				volatile uint32_t TTSE					:1;
				volatile uint32_t DP					:1;
				volatile uint32_t DC					:1;
				volatile uint32_t FS					:1;
				volatile uint32_t LS					:1;
				volatile uint32_t IC					:1;
				volatile uint32_t OWN					:1;

			}Tx;
			struct
			{
				volatile uint32_t PCE_ESA				:1;
				volatile uint32_t CE					:1;
				volatile uint32_t DBE					:1;
				volatile uint32_t RE					:1;
				volatile uint32_t RWT					:1;
				volatile uint32_t FT					:1;
				volatile uint32_t LCO					:1;
				volatile uint32_t IPHCE_TSV			:1;

				volatile uint32_t LS					:1;
				volatile uint32_t FS					:1;
				volatile uint32_t VLAN					:1;
				volatile uint32_t OE					:1;
				volatile uint32_t LE					:1;
				volatile uint32_t SAF					:1;
				volatile uint32_t DE					:1;
				volatile uint32_t ES					:1;

				volatile uint32_t FL					:14;
				volatile uint32_t AFM					:1;
				volatile uint32_t OWN					:1;
			}Rx;
		}DES0_BIT;
	};
	union
	{
		volatile uint32_t DES1;     /*!< Control and Buffer1, Buffer2 lengths */
		union
		{
			struct
			{
				volatile uint32_t TBS1					:13;
				volatile uint32_t 						:3;
				volatile uint32_t TBS2					:13;
				volatile uint32_t 						:3;
			}Tx;
			struct
			{
				volatile uint32_t RBS1					:13;
				volatile uint32_t 						:1;
				volatile uint32_t RCH					:1;
				volatile uint32_t RER					:1;

				volatile uint32_t RBS2					:13;
				volatile uint32_t 						:2;
				volatile uint32_t DIC					:1;
			}Rx;
		}DES1_BIT;
	};
	volatile uint32_t Buffer1Addr;           /*!< Buffer1 address pointer */
	volatile uint32_t Buffer2NextDescAddr;   /*!< Buffer2 or next descriptor address pointer */
/* Enhanced ETHERNET DMA PTP Descriptors */

//#ifdef USE_ENHANCED_DMA_DESCRIPTORS
	union
	{
		volatile uint32_t DES4;        /* Extended status for PTP receive descriptor */
		union
		{
			struct
			{
				volatile uint32_t IPPT					:3;
				volatile uint32_t IPHE					:1;
				volatile uint32_t IPPE					:1;
				volatile uint32_t IPCB					:1;
				volatile uint32_t IPV4PR				:1;
				volatile uint32_t IPV6PR				:1;

				volatile uint32_t PMT					:4;
				volatile uint32_t PFT					:1;
				volatile uint32_t PV					:1;
				volatile uint32_t 						:18;
			}Rx;
		}DES4_BIT;
	};
	volatile uint32_t Reserved1;             /* Reserved */
	volatile uint32_t TimeStampLow;          /* Time Stamp Low value for transmit and receive */
	volatile uint32_t TimeStampHigh;         /* Time Stamp High value for transmit and receive */
//#endif /* USE_ENHANCED_DMA_DESCRIPTORS */
} ETH_DMADESCTypeDef;





#endif /* DRIVERS_STM32F4XX_ETHER_MAC_STRUCT_H_ */
