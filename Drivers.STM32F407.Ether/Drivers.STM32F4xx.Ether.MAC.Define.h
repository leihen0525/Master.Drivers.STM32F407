/*
 * Drivers.STM32F4xx.Ether.MAC.Define.h
 *
 *  Created on: 2019Äê5ÔÂ21ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_MAC_DEFINE_H_
#define DRIVERS_STM32F4XX_ETHER_MAC_DEFINE_H_

/*
	ETH_MDIO -------------------------> PA2
	ETH_MDC --------------------------> PC1
	ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
	ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
	ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
	ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
	ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PG11
	ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
	ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
																						*/
/* ETH_MDIO */
#define ETH_MDIO_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define ETH_MDIO_PORT                   GPIOA
#define ETH_MDIO_PIN                    GPIO_Pin_2
#define ETH_MDIO_AF                     GPIO_AF_ETH
#define ETH_MDIO_SOURCE                 GPIO_PinSource2

/* ETH_MDC */
#define ETH_MDC_GPIO_CLK                RCC_AHB1Periph_GPIOC
#define ETH_MDC_PORT                    GPIOC
#define ETH_MDC_PIN                     GPIO_Pin_1
#define ETH_MDC_AF                      GPIO_AF_ETH
#define ETH_MDC_SOURCE                  GPIO_PinSource1

/* ETH_RMII_REF_CLK */
#define ETH_RMII_REF_CLK_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define ETH_RMII_REF_CLK_PORT           GPIOA
#define ETH_RMII_REF_CLK_PIN            GPIO_Pin_1
#define ETH_RMII_REF_CLK_AF             GPIO_AF_ETH
#define ETH_RMII_REF_CLK_SOURCE         GPIO_PinSource1

/* ETH_RMII_CRS_DV */
#define ETH_RMII_CRS_DV_GPIO_CLK        RCC_AHB1Periph_GPIOA
#define ETH_RMII_CRS_DV_PORT            GPIOA
#define ETH_RMII_CRS_DV_PIN             GPIO_Pin_7
#define ETH_RMII_CRS_DV_AF              GPIO_AF_ETH
#define ETH_RMII_CRS_DV_SOURCE          GPIO_PinSource7

/* ETH_RMII_RXD0 */
#define ETH_RMII_RXD0_GPIO_CLK          RCC_AHB1Periph_GPIOC
#define ETH_RMII_RXD0_PORT              GPIOC
#define ETH_RMII_RXD0_PIN               GPIO_Pin_4
#define ETH_RMII_RXD0_AF                GPIO_AF_ETH
#define ETH_RMII_RXD0_SOURCE            GPIO_PinSource4

/* ETH_RMII_RXD1 */
#define ETH_RMII_RXD1_GPIO_CLK          RCC_AHB1Periph_GPIOC
#define ETH_RMII_RXD1_PORT              GPIOC
#define ETH_RMII_RXD1_PIN               GPIO_Pin_5
#define ETH_RMII_RXD1_AF                GPIO_AF_ETH
#define ETH_RMII_RXD1_SOURCE            GPIO_PinSource5

/* ETH_RMII_TX_EN */
#define ETH_RMII_TX_EN_GPIO_CLK         RCC_AHB1Periph_GPIOG
#define ETH_RMII_TX_EN_PORT             GPIOG
#define ETH_RMII_TX_EN_PIN              GPIO_Pin_11
#define ETH_RMII_TX_EN_AF               GPIO_AF_ETH
#define ETH_RMII_TX_EN_SOURCE           GPIO_PinSource11

/* ETH_RMII_TXD0 */
#define ETH_RMII_TXD0_GPIO_CLK          RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXD0_PORT              GPIOG
#define ETH_RMII_TXD0_PIN               GPIO_Pin_13
#define ETH_RMII_TXD0_AF                GPIO_AF_ETH
#define ETH_RMII_TXD0_SOURCE            GPIO_PinSource13

/* ETH_RMII_TXD1 */
#define ETH_RMII_TXD1_GPIO_CLK          RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXD1_PORT              GPIOG
#define ETH_RMII_TXD1_PIN               GPIO_Pin_14
#define ETH_RMII_TXD1_AF                GPIO_AF_ETH
#define ETH_RMII_TXD1_SOURCE            GPIO_PinSource14

/* MII and RMII mode selection, for STM324xG-EVAL Board(MB786) RevB ***********/
#define RMII_MODE  // User have to provide the 50 MHz clock by soldering a 50 MHz
                     // oscillator (ref SM7745HEV-50.0M or equivalent) on the U3
                     // footprint located under CN3 and also removing jumper on JP5.
                     // This oscillator is not provided with the board.
                     // For more details, please refer to STM3240G-EVAL evaluation
                     // board User manual (UM1461).

//#define MII_MODE

/* Uncomment the define below to clock the PHY from external 25MHz crystal (only for MII mode) */
#ifdef 	MII_MODE
 #define PHY_CLOCK_MCO
#endif


#define ETH_MAX_PACKET_SIZE    1524    /*!< ETH_HEADER + ETH_EXTRA + VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC */
#define ETH_HEADER               14    /*!< 6 byte Dest addr, 6 byte Src addr, 2 byte length/type */
#define ETH_CRC                   4    /*!< Ethernet CRC */
#define ETH_EXTRA                 2    /*!< Extra bytes in some cases */
#define VLAN_TAG                  4    /*!< optional 802.1q VLAN Tag */
#define MIN_ETH_PAYLOAD          46    /*!< Minimum Ethernet payload size */
#define MAX_ETH_PAYLOAD        1500    /*!< Maximum Ethernet payload size */
#define JUMBO_FRAME_PAYLOAD    9000    /*!< Jumbo frame payload size */

#ifndef ETH_RXBUFNB
 #define ETH_RXBUFNB             8     /*  5 Rx buffers of size ETH_RX_BUF_SIZE */
#endif

#ifndef ETH_RX_BUF_SIZE
 #define ETH_RX_BUF_SIZE         ETH_MAX_PACKET_SIZE
#endif

#ifndef ETH_TXBUFNB
 #define ETH_TXBUFNB             8      /* 5  Tx buffers of size ETH_TX_BUF_SIZE */
#endif

#ifndef ETH_TX_BUF_SIZE
 #define ETH_TX_BUF_SIZE         ETH_MAX_PACKET_SIZE
#endif

#define ETH_DMATxDesc_OWN                     ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMATxDesc_IC                      ((uint32_t)0x40000000)  /*!< Interrupt on Completion */
#define ETH_DMATxDesc_LS                      ((uint32_t)0x20000000)  /*!< Last Segment */
#define ETH_DMATxDesc_FS                      ((uint32_t)0x10000000)  /*!< First Segment */
#define ETH_DMATxDesc_DC                      ((uint32_t)0x08000000)  /*!< Disable CRC */
#define ETH_DMATxDesc_DP                      ((uint32_t)0x04000000)  /*!< Disable Padding */
#define ETH_DMATxDesc_TTSE                    ((uint32_t)0x02000000)  /*!< Transmit Time Stamp Enable */
#define ETH_DMATxDesc_CIC                     ((uint32_t)0x00C00000)  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATxDesc_CIC_ByPass              ((uint32_t)0x00000000)  /*!< Do Nothing: Checksum Engine is bypassed */
#define ETH_DMATxDesc_CIC_IPV4Header          ((uint32_t)0x00400000)  /*!< IPV4 header Checksum Insertion */
#define ETH_DMATxDesc_CIC_TCPUDPICMP_Segment  ((uint32_t)0x00800000)  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */
#define ETH_DMATxDesc_CIC_TCPUDPICMP_Full     ((uint32_t)0x00C00000)  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */
#define ETH_DMATxDesc_TER                     ((uint32_t)0x00200000)  /*!< Transmit End of Ring */
#define ETH_DMATxDesc_TCH                     ((uint32_t)0x00100000)  /*!< Second Address Chained */
#define ETH_DMATxDesc_TTSS                    ((uint32_t)0x00020000)  /*!< Tx Time Stamp Status */
#define ETH_DMATxDesc_IHE                     ((uint32_t)0x00010000)  /*!< IP Header Error */
#define ETH_DMATxDesc_ES                      ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATxDesc_JT                      ((uint32_t)0x00004000)  /*!< Jabber Timeout */
#define ETH_DMATxDesc_FF                      ((uint32_t)0x00002000)  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATxDesc_PCE                     ((uint32_t)0x00001000)  /*!< Payload Checksum Error */
#define ETH_DMATxDesc_LCA                     ((uint32_t)0x00000800)  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATxDesc_NC                      ((uint32_t)0x00000400)  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATxDesc_LCO                     ((uint32_t)0x00000200)  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATxDesc_EC                      ((uint32_t)0x00000100)  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATxDesc_VF                      ((uint32_t)0x00000080)  /*!< VLAN Frame */
#define ETH_DMATxDesc_CC                      ((uint32_t)0x00000078)  /*!< Collision Count */
#define ETH_DMATxDesc_ED                      ((uint32_t)0x00000004)  /*!< Excessive Deferral */
#define ETH_DMATxDesc_UF                      ((uint32_t)0x00000002)  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATxDesc_DB                      ((uint32_t)0x00000001)  /*!< Deferred Bit */


#define ETH_DMARxDesc_OWN         ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARxDesc_AFM         ((uint32_t)0x40000000)  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARxDesc_FL          ((uint32_t)0x3FFF0000)  /*!< Receive descriptor frame length  */
#define ETH_DMARxDesc_ES          ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARxDesc_DE          ((uint32_t)0x00004000)  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARxDesc_SAF         ((uint32_t)0x00002000)  /*!< SA Filter Fail for the received frame */
#define ETH_DMARxDesc_LE          ((uint32_t)0x00001000)  /*!< Frame size not matching with length field */
#define ETH_DMARxDesc_OE          ((uint32_t)0x00000800)  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARxDesc_VLAN        ((uint32_t)0x00000400)  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARxDesc_FS          ((uint32_t)0x00000200)  /*!< First descriptor of the frame  */
#define ETH_DMARxDesc_LS          ((uint32_t)0x00000100)  /*!< Last descriptor of the frame  */
#define ETH_DMARxDesc_IPV4HCE     ((uint32_t)0x00000080)  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */
#define ETH_DMARxDesc_LC          ((uint32_t)0x00000040)  /*!< Late collision occurred during reception   */
#define ETH_DMARxDesc_FT          ((uint32_t)0x00000020)  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARxDesc_RWT         ((uint32_t)0x00000010)  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARxDesc_RE          ((uint32_t)0x00000008)  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARxDesc_DBE         ((uint32_t)0x00000004)  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARxDesc_CE          ((uint32_t)0x00000002)  /*!< CRC error */
#define ETH_DMARxDesc_MAMPCE      ((uint32_t)0x00000001)  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

#define ETH_DMARxDesc_DIC   ((uint32_t)0x80000000)  /*!< Disable Interrupt on Completion */
#define ETH_DMARxDesc_RBS2  ((uint32_t)0x1FFF0000)  /*!< Receive Buffer2 Size */
#define ETH_DMARxDesc_RER   ((uint32_t)0x00008000)  /*!< Receive End of Ring */
#define ETH_DMARxDesc_RCH   ((uint32_t)0x00004000)  /*!< Second Address Chained */
#define ETH_DMARxDesc_RBS1  ((uint32_t)0x00001FFF)  /*!< Receive Buffer1 Size */

#define ETH_DMATxDesc_ChecksumByPass             ((uint32_t)0x00000000)   /*!< Checksum engine bypass */
#define ETH_DMATxDesc_ChecksumIPV4Header         ((uint32_t)0x00400000)   /*!< IPv4 header checksum insertion  */
#define ETH_DMATxDesc_ChecksumTCPUDPICMPSegment  ((uint32_t)0x00800000)   /*!< TCP/UDP/ICMP checksum insertion. Pseudo header checksum is assumed to be present */
#define ETH_DMATxDesc_ChecksumTCPUDPICMPFull     ((uint32_t)0x00C00000)   /*!< TCP/UDP/ICMP checksum fully in hardware including pseudo header */

#endif /* DRIVERS_STM32F4XX_ETHER_MAC_DEFINE_H_ */
