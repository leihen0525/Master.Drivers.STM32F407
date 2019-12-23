/*
 * Drivers.STM32F4xx.Ether.MAC.c
 *
 *  Created on: 2019年5月21日
 *      Author: Master.HE
 */
#include "Error.h"
#include "STM32F4xx.h"
#include "Drivers.STM32F4xx.Ether.MAC.Define.h"
#include "Drivers.STM32F4xx.Ether.MAC.h"
#include "Drivers.STM32F4xx.Ether.PHY.h"


#pragma data_alignment=4
ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
#pragma data_alignment=4
ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
#pragma data_alignment=4
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */
#pragma data_alignment=4
uint8_t Tx_Buff[ETH_TXBUFNB][ETH_MAX_PACKET_SIZE]; /* Ethernet Transmit Buffer */



void Drivers_STM32F4xx_Ether_MAC_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(
			 ETH_MDIO_GPIO_CLK
			|ETH_MDC_GPIO_CLK
			|ETH_RMII_REF_CLK_GPIO_CLK
			|ETH_RMII_CRS_DV_GPIO_CLK
			|ETH_RMII_RXD0_GPIO_CLK
			|ETH_RMII_RXD1_GPIO_CLK
			|ETH_RMII_TX_EN_GPIO_CLK
			|ETH_RMII_TXD0_GPIO_CLK
			|ETH_RMII_TXD1_GPIO_CLK
			|RCC_AHB1Periph_GPIOD,
			ENABLE);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	/* MII/RMII Media interface selection --------------------------------------*/
#ifdef MII_MODE /* Mode MII with STM324xG-EVAL  */
#ifdef PHY_CLOCK_MCO

	/* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
	RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);

#endif /* PHY_CLOCK_MCO */

	SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII);

#elif defined RMII_MODE  /* Mode RMII with STM324xG-EVAL */

	SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);

#endif

	/* Ethernet pins configuration ************************************************/
	/*
	ETH_MDIO -------------------------> PA2
	ETH_MDC --------------------------> PC1
	ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
	ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
	ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
	ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
	ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
	ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
	ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
	ETH_NRST -------------------------> PI1
	  */

	/* Configure ETH_MDIO */
	GPIO_InitStructure.GPIO_Pin = ETH_MDIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ETH_MDIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_MDIO_PORT, ETH_MDIO_SOURCE, ETH_MDIO_AF);

	/* Configure ETH_MDC */
	GPIO_InitStructure.GPIO_Pin = ETH_MDC_PIN;
	GPIO_Init(ETH_MDC_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_MDC_PORT, ETH_MDC_SOURCE, ETH_MDC_AF);

	/* Configure ETH_RMII_REF_CLK */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_REF_CLK_PIN;
	GPIO_Init(ETH_RMII_REF_CLK_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_REF_CLK_PORT, ETH_RMII_REF_CLK_SOURCE, ETH_RMII_REF_CLK_AF);

	/* Configure ETH_RMII_CRS_DV */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_CRS_DV_PIN;
	GPIO_Init(ETH_RMII_CRS_DV_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_CRS_DV_PORT, ETH_RMII_CRS_DV_SOURCE, ETH_RMII_CRS_DV_AF);

	/* Configure ETH_RMII_RXD0 */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_RXD0_PIN;
	GPIO_Init(ETH_RMII_RXD0_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_RXD0_PORT, ETH_RMII_RXD0_SOURCE, ETH_RMII_RXD0_AF);

	/* Configure ETH_RMII_RXD1 */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_RXD1_PIN;
	GPIO_Init(ETH_RMII_RXD1_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_RXD1_PORT, ETH_RMII_RXD1_SOURCE, ETH_RMII_RXD1_AF);

	/* Configure ETH_RMII_TX_EN */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_TX_EN_PIN;
	GPIO_Init(ETH_RMII_TX_EN_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_TX_EN_PORT, ETH_RMII_TX_EN_SOURCE, ETH_RMII_TX_EN_AF);

	/* Configure ETH_RMII_TXD0 */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_TXD0_PIN;
	GPIO_Init(ETH_RMII_TXD0_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_TXD0_PORT, ETH_RMII_TXD0_SOURCE, ETH_RMII_TXD0_AF);

	/* Configure ETH_RMII_TXD1 */
	GPIO_InitStructure.GPIO_Pin = ETH_RMII_TXD1_PIN;
	GPIO_Init(ETH_RMII_TXD1_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(ETH_RMII_TXD1_PORT, ETH_RMII_TXD1_SOURCE, ETH_RMII_TXD1_AF);

	//配置PD3为推完输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推完输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);




}
int Drivers_STM32F4xx_Ether_MAC_Init(void)
{
	/* Enable ETHERNET clock  */
	RCC_AHB1PeriphClockCmd(
			 RCC_AHB1Periph_ETH_MAC
			|RCC_AHB1Periph_ETH_MAC_Tx
			|RCC_AHB1Periph_ETH_MAC_Rx,
			ENABLE);

	/* Reset ETHERNET on AHB Bus */
	RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, ENABLE);
	RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, DISABLE);

	Drivers_STM32F4xx_Ether_MAC_Software_Reset();

	Drivers_STM32F4xx_Ether_PHY_SET_Clock();


	//------------------------------------
	ETH->MACCR_BIT.WD=1;//0;/* MAC watchdog enabled: cuts off long frame */
	ETH->MACCR_BIT.JD=1;//0;/* MAC Jabber enabled in Half-duplex mode */
	ETH->MACCR_BIT.IFG=0;/* Ethernet interframe gap set to 96 bits */
	ETH->MACCR_BIT.CSD=0;/* Carrier Sense Enabled in Half-Duplex mode */
	ETH->MACCR_BIT.FES=1;/* PHY speed configured to 100Mbit/s */
	ETH->MACCR_BIT.ROD=0;/* Receive own Frames in Half-Duplex mode enabled */


	ETH->MACCR_BIT.LM=0;/* 关闭反馈 */

	ETH->MACCR_BIT.DM=1;/* Full-Duplex mode selected */

	ETH->MACCR_BIT.IPCO=1;/* 开启ipv4和TCP/UDP/ICMP的帧校验和卸载   */

	ETH->MACCR_BIT.RD=1;/* 关闭重传功能 */

	ETH->MACCR_BIT.APCS=0;/* 关闭自动去除PDA/CRC功能  */

	ETH->MACCR_BIT.BL=0;/* half-duplex mode retransmission Backoff time_limit = 10 slot times*/

	ETH->MACCR_BIT.DC=0;/* half-duplex mode Deferral check disabled */


	//---------------------------------------
	ETH->MACFFR_BIT.RA=0;/* 关闭接收所有的帧 */

	ETH->MACFFR_BIT.SAF=0;/* Source address filtering (on the optional MAC addresses) disabled */
	ETH->MACFFR_BIT.SAIF=0;

	ETH->MACFFR_BIT.PCF=1;/* Do not forward control frames that do not pass the address filtering */

	ETH->MACFFR_BIT.BFD=0;/* 允许接收所有广播帧 */

	ETH->MACFFR_BIT.DAIF=0;/* Normal Destination address filtering (not reverse addressing) */

	ETH->MACFFR_BIT.HM=0;/* 对于组播地址使用完美地址过滤    */
	ETH->MACFFR_BIT.PAM=0;
	ETH->MACFFR_BIT.HPF=0;

	ETH->MACFFR_BIT.HU=0;/* 对单播地址使用完美地址过滤  */
	ETH->MACFFR_BIT.HPF=0;

	ETH->MACFFR_BIT.PM=0;/* 关闭混合模式的地址过滤  */


	//----------------------------------------------
	ETH->MACHTHR = 0;
	ETH->MACHTLR = 0;

	//----------------------------------------------
	ETH->MACFCR_BIT.PT=0;/* Flow control config (flow control disabled)*/
	ETH->MACFCR_BIT.ZQPD=1;
	ETH->MACFCR_BIT.PLT=0;
	ETH->MACFCR_BIT.UPFD=0;
	ETH->MACFCR_BIT.RFCE=0;
	ETH->MACFCR_BIT.TFCE=0;


	//----------------------------------------------
	ETH->MACVLANTR_BIT.VLANTC=0;
	ETH->MACVLANTR_BIT.VLANTI=0;


	//----------------------------------------------
	ETH->DMAOMR_BIT.DTCEFD=0;/* 开启丢弃TCP/IP错误帧 */
	ETH->DMAOMR_BIT.RSF=1;/* 开启接收数据的存储转发模式  */
	ETH->DMAOMR_BIT.DFRF=0;/* Flush received frame that created FIFO overflow */
	ETH->DMAOMR_BIT.TSF=1;/* 开启发送数据的存储转发模式   */
	ETH->DMAOMR_BIT.TTC=0;/* Threshold TXFIFO level set to 64 bytes (used when threshold mode is enabled) */
	ETH->DMAOMR_BIT.FEF=0;/* 禁止转发错误帧 */
	ETH->DMAOMR_BIT.FUGF=0;/* 不转发过小的好帧 */
	ETH->DMAOMR_BIT.RTC1=0;
	ETH->DMAOMR_BIT.OSF=1;/* 打开处理第二帧功能 */

	//-----------------------------------------------
	ETH->DMABMR_BIT.AAB=1;/* 开启DMA传输的地址对齐功能 */
	ETH->DMABMR_BIT.FB=1;/* 开启固定突发功能 */
	ETH->DMABMR_BIT.RDP=32;/*DMA接收的最大突发长度为32个节拍 */
	ETH->DMABMR_BIT.PBL=32;/* DMA发送的最大突发长度为32个节拍 */
	ETH->DMABMR_BIT.DSL=0;/* DMA Ring mode skip length = 0 */

	ETH->DMABMR_BIT.PM=1;
	ETH->DMABMR_BIT.DA=0;


	//-------------------------------------------------
	ETH->DMABMR_BIT.EDFE=1;//开启扩展标识符


	//-------------------------------------------------

	ETH_DMATxDescChainInit();
	ETH_DMARxDescChainInit();

	/* Enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
	for(int i=0; i<ETH_TXBUFNB; i++)
	{
		//DMATxDscrTab[i].DES0|=ETH_DMATxDesc_ChecksumTCPUDPICMPFull;
	}


	//IRQ
	ETH->DMAIER_BIT.NISE=1;
	ETH->DMAIER_BIT.RIE=1;
	ETH->DMAIER_BIT.TIE=1;
	ETH->DMAIER_BIT.ETIE=1;
	ETH->DMAIER_BIT.ERIE=1;

	return Error_OK;

#if 0
//
	  /*-------------------- PHY initialization and configuration ----------------*/
	int Err=Target_STM32F4xx_Ether_PHY_Reset_Register();
	uint16_t PHYValue=0;
	__IO uint32_t timeout = 0;


	if(Err!=Error_OK)
	{
		goto error;
	}

	Target_STM32F4xx_Ether_PHY_Read(ETHERNET_PHY_ADDRESS, PHY_BCR,&PHYValue);
	//Target_STM32F4xx_Ether_PHY_Write(ETHERNET_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation);


	if(1)/* 开启网络自适应功能 */
	{
		/* We wait for linked status...*/
		do
		{
			timeout++;
			if((Err=Target_STM32F4xx_Ether_PHY_Read(ETHERNET_PHY_ADDRESS, PHY_BSR,&PHYValue))!=Error_OK)
			{
				goto error;
			}
		} while (1 && (timeout < PHY_READ_TO));

		/* Return ERROR in case of timeout */
		if(timeout == PHY_READ_TO)
		{
			Err = Error_Time_Out;
			goto error;
		}

		/* Reset Timeout counter */
		timeout = 0;
		/* Enable Auto-Negotiation */
		if((Err=Target_STM32F4xx_Ether_PHY_Write(ETHERNET_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation))!=Error_OK)
		{
		/* Return ERROR in case of write timeout */
		//err = ETH_ERROR;
		}

		/* Wait until the auto-negotiation will be completed */
		do
		{
			timeout++;
			if((Err=Target_STM32F4xx_Ether_PHY_Read(ETHERNET_PHY_ADDRESS, PHY_BSR,&PHYValue))!=Error_OK)
			{
				goto error;
			}
		} while (!(PHYValue & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));

		/* Return ERROR in case of timeout */
		if(timeout == PHY_READ_TO)
		{
			Err = Error_Time_Out;
			goto error;
		}

		/* Reset Timeout counter */
		timeout = 0;
		/* Read the result of the auto-negotiation */
		Target_STM32F4xx_Ether_PHY_Read(ETHERNET_PHY_ADDRESS, PHY_SR,&PHYValue);


		/* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
		if((PHYValue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
		{
			/* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
			//ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
		}
		else
		{
			/* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
			//ETH_InitStruct->ETH_Mode = ETH_Mode_HalfDuplex;
		}
		/* Configure the MAC with the speed fixed by the auto-negotiation process */
		if(PHYValue & PHY_SPEED_STATUS)
		{
			/* Set Ethernet speed to 10M following the auto-negotiation */
			//ETH_InitStruct->ETH_Speed = ETH_Speed_10M;
		}
		else
		{
			/* Set Ethernet speed to 100M following the auto-negotiation */
			//ETH_InitStruct->ETH_Speed = ETH_Speed_100M;
		}
	}
	else
	{
//		if(!ETH_WritePHYRegister(ETHERNET_PHY_ADDRESS, PHY_BCR, ((uint16_t)(ETH_InitStruct->ETH_Mode >> 3) |
//										   (uint16_t)(ETH_InitStruct->ETH_Speed >> 1))))
//		{
//			/* Return ERROR in case of write timeout */
//			err = ETH_ERROR;
//		}
//		/* Delay to assure PHY configuration */
//		_eth_delay_(PHY_CONFIG_DELAY);
	}


error:
	if (Err != Error_OK) /* Auto-negotiation failed */
	{
		/* Set Ethernet duplex mode to Full-duplex */
		//ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;

		/* Set Ethernet speed to 100M */
		//ETH_InitStruct->ETH_Speed = ETH_Speed_100M;
	}



	/*------------------------ ETHERNET MACCR Configuration --------------------*/
	/* Get the ETHERNET MACCR value */
//	tmpreg = ETH->MACCR;
	/* Clear WD, PCE, PS, TE and RE bits */
//	tmpreg &= MACCR_CLEAR_MASK;
	/* Set the WD bit according to ETH_Watchdog value */
	/* Set the JD: bit according to ETH_Jabber value */
	/* Set the IFG bit according to ETH_InterFrameGap value */
	/* Set the DCRS bit according to ETH_CarrierSense value */
	/* Set the FES bit according to ETH_Speed value */
	/* Set the DO bit according to ETH_ReceiveOwn value */
	/* Set the LM bit according to ETH_LoopbackMode value */
	/* Set the DM bit according to ETH_Mode value */
	/* Set the IPCO bit according to ETH_ChecksumOffload value */
	/* Set the DR bit according to ETH_RetryTransmission value */
	/* Set the ACS bit according to ETH_AutomaticPadCRCStrip value */
	/* Set the BL bit according to ETH_BackOffLimit value */
	/* Set the DC bit according to ETH_DeferralCheck value */




//	tmpreg |= (uint32_t)(ETH_InitStruct->ETH_Watchdog |
//						ETH_InitStruct->ETH_Jabber |
//						ETH_InitStruct->ETH_InterFrameGap |
//						ETH_InitStruct->ETH_CarrierSense |
//						ETH_InitStruct->ETH_Speed |
//						ETH_InitStruct->ETH_ReceiveOwn |
//						ETH_InitStruct->ETH_LoopbackMode |
//						ETH_InitStruct->ETH_Mode |
//						ETH_InitStruct->ETH_ChecksumOffload |
//						ETH_InitStruct->ETH_RetryTransmission |
//						ETH_InitStruct->ETH_AutomaticPadCRCStrip |
//						ETH_InitStruct->ETH_BackOffLimit |
//						ETH_InitStruct->ETH_DeferralCheck);
	/* Write to ETHERNET MACCR */
//	ETH->MACCR = (uint32_t)tmpreg;

	/* Wait until the write operation will be taken into account :
	at least four TX_CLK/RX_CLK clock cycles */
//	tmpreg = ETH->MACCR;
//	_eth_delay_(ETH_REG_WRITE_DELAY);
//	ETH->MACCR = tmpreg;

	/*----------------------- ETHERNET MACFFR Configuration --------------------*/
	/* Set the RA bit according to ETH_ReceiveAll value */
	/* Set the SAF and SAIF bits according to ETH_SourceAddrFilter value */
	/* Set the PCF bit according to ETH_PassControlFrames value */
	/* Set the DBF bit according to ETH_BroadcastFramesReception value */
	/* Set the DAIF bit according to ETH_DestinationAddrFilter value */
	/* Set the PR bit according to ETH_PromiscuousMode value */
	/* Set the PM, HMC and HPF bits according to ETH_MulticastFramesFilter value */
	/* Set the HUC and HPF bits according to ETH_UnicastFramesFilter value */
	/* Write to ETHERNET MACFFR */



//	ETH->MACFFR = (uint32_t)(ETH_InitStruct->ETH_ReceiveAll |
//							ETH_InitStruct->ETH_SourceAddrFilter |
//							ETH_InitStruct->ETH_PassControlFrames |
//							ETH_InitStruct->ETH_BroadcastFramesReception |
//							ETH_InitStruct->ETH_DestinationAddrFilter |
//							ETH_InitStruct->ETH_PromiscuousMode |
//							ETH_InitStruct->ETH_MulticastFramesFilter |
//							ETH_InitStruct->ETH_UnicastFramesFilter);

	/* Wait until the write operation will be taken into account :
	at least four TX_CLK/RX_CLK clock cycles */
//	tmpreg = ETH->MACFFR;
//	_eth_delay_(ETH_REG_WRITE_DELAY);
//	ETH->MACFFR = tmpreg;


	/*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
	/* Write to ETHERNET MACHTHR */


	/* Write to ETHERNET MACHTLR */


	/*----------------------- ETHERNET MACFCR Configuration --------------------*/

	/* Get the ETHERNET MACFCR value */
//	tmpreg = ETH->MACFCR;
	/* Clear xx bits */
//	tmpreg &= MACFCR_CLEAR_MASK;

	/* Set the PT bit according to ETH_PauseTime value */
	/* Set the DZPQ bit according to ETH_ZeroQuantaPause value */
	/* Set the PLT bit according to ETH_PauseLowThreshold value */
	/* Set the UP bit according to ETH_UnicastPauseFrameDetect value */
	/* Set the RFE bit according to ETH_ReceiveFlowControl value */
	/* Set the TFE bit according to ETH_TransmitFlowControl value */



//	tmpreg |= (uint32_t)((ETH_InitStruct->ETH_PauseTime << 16) |
//						ETH_InitStruct->ETH_ZeroQuantaPause |
//						ETH_InitStruct->ETH_PauseLowThreshold |
//						ETH_InitStruct->ETH_UnicastPauseFrameDetect |
//						ETH_InitStruct->ETH_ReceiveFlowControl |
//						ETH_InitStruct->ETH_TransmitFlowControl);
	/* Write to ETHERNET MACFCR */
//	ETH->MACFCR = (uint32_t)tmpreg;

	/* Wait until the write operation will be taken into account :
	at least four TX_CLK/RX_CLK clock cycles */
//	tmpreg = ETH->MACFCR;
//	_eth_delay_(ETH_REG_WRITE_DELAY);
//	ETH->MACFCR = tmpreg;

	/*----------------------- ETHERNET MACVLANTR Configuration -----------------*/
	/* Set the ETV bit according to ETH_VLANTagComparison value */
	/* Set the VL bit according to ETH_VLANTagIdentifier value */

//	ETH->MACVLANTR = (uint32_t)(ETH_InitStruct->ETH_VLANTagComparison |
//							  ETH_InitStruct->ETH_VLANTagIdentifier);

	/* Wait until the write operation will be taken into account :
	at least four TX_CLK/RX_CLK clock cycles */
//	tmpreg = ETH->MACVLANTR;
//	_eth_delay_(ETH_REG_WRITE_DELAY);
//	ETH->MACVLANTR = tmpreg;

	/*-------------------------------- DMA Config ------------------------------*/
	/*----------------------- ETHERNET DMAOMR Configuration --------------------*/

	/* Get the ETHERNET DMAOMR value */
//	tmpreg = ETH->DMAOMR;
	/* Clear xx bits */
//	tmpreg &= DMAOMR_CLEAR_MASK;

	/* Set the DT bit according to ETH_DropTCPIPChecksumErrorFrame value */
	/* Set the RSF bit according to ETH_ReceiveStoreForward value */
	/* Set the DFF bit according to ETH_FlushReceivedFrame value */
	/* Set the TSF bit according to ETH_TransmitStoreForward value */
	/* Set the TTC bit according to ETH_TransmitThresholdControl value */
	/* Set the FEF bit according to ETH_ForwardErrorFrames value */
	/* Set the FUF bit according to ETH_ForwardUndersizedGoodFrames value */
	/* Set the RTC bit according to ETH_ReceiveThresholdControl value */
	/* Set the OSF bit according to ETH_SecondFrameOperate value */



//	tmpreg |= (uint32_t)(ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame |
//								ETH_InitStruct->ETH_ReceiveStoreForward |
//								ETH_InitStruct->ETH_FlushReceivedFrame |
//								ETH_InitStruct->ETH_TransmitStoreForward |
//								ETH_InitStruct->ETH_TransmitThresholdControl |
//								ETH_InitStruct->ETH_ForwardErrorFrames |
//								ETH_InitStruct->ETH_ForwardUndersizedGoodFrames |
//								ETH_InitStruct->ETH_ReceiveThresholdControl |
//								ETH_InitStruct->ETH_SecondFrameOperate);
	/* Write to ETHERNET DMAOMR */
//	ETH->DMAOMR = (uint32_t)tmpreg;

	/* Wait until the write operation will be taken into account :
	at least four TX_CLK/RX_CLK clock cycles */
//	tmpreg = ETH->DMAOMR;
//	_eth_delay_(ETH_REG_WRITE_DELAY);
//	ETH->DMAOMR = tmpreg;

	/*----------------------- ETHERNET DMABMR Configuration --------------------*/
	/* Set the AAL bit according to ETH_AddressAlignedBeats value */
	/* Set the FB bit according to ETH_FixedBurst value */
	/* Set the RPBL and 4*PBL bits according to ETH_RxDMABurstLength value */
	/* Set the PBL and 4*PBL bits according to ETH_TxDMABurstLength value */
	/* Set the DSL bit according to ETH_DesciptorSkipLength value */
	/* Set the PR and DA bits according to ETH_DMAArbitration value */



//	ETH->DMABMR = (uint32_t)(ETH_InitStruct->ETH_AddressAlignedBeats |
//						ETH_InitStruct->ETH_FixedBurst |
//						ETH_InitStruct->ETH_RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
//						ETH_InitStruct->ETH_TxDMABurstLength |
//						(ETH_InitStruct->ETH_DescriptorSkipLength << 2) |
//						ETH_InitStruct->ETH_DMAArbitration |
//						ETH_DMABMR_USP); /* Enable use of separate PBL for Rx and Tx */

	/* Wait until the write operation will be taken into account :
	at least four TX_CLK/RX_CLK clock cycles */
//	tmpreg = ETH->DMABMR;
//	_eth_delay_(ETH_REG_WRITE_DELAY);
//	ETH->DMABMR = tmpreg;



	#ifdef USE_ENHANCED_DMA_DESCRIPTORS
	  /* Enable the Enhanced DMA descriptors */
	  ETH->DMABMR |= ETH_DMABMR_EDE;

	  /* Wait until the write operation will be taken into account :
	   at least four TX_CLK/RX_CLK clock cycles */
	  tmpreg = ETH->DMABMR;
	  _eth_delay_(ETH_REG_WRITE_DELAY);
	  ETH->DMABMR = tmpreg;
	#endif /* USE_ENHANCED_DMA_DESCRIPTORS */

//	/* Return Ethernet configuration success */
//	if(err == ETH_SUCCESS)
//	{
//		/* Return Ethernet configuration success */
//		return ETH_SUCCESS;
//	}
//	else /* Auto-negotiation failed */
//	{
//		/* Return Ethernet error */
//		return ETH_ERROR;
//	}

#endif


}


void Drivers_STM32F4xx_Ether_MAC_Software_Reset(void)
{
	/* Software reset */
	ETH->DMABMR_BIT.SR=1;

	/* Wait for software reset */
	while(ETH->DMABMR_BIT.SR!=0);
}

void Drivers_STM32F4xx_Ether_MAC_SET_MAC_Address(uint8_t *MAC_Address)
{
	ETH->MACA0LR_BIT.MAC_ADDR0=MAC_Address[0];
	ETH->MACA0LR_BIT.MAC_ADDR1=MAC_Address[1];
	ETH->MACA0LR_BIT.MAC_ADDR2=MAC_Address[2];
	ETH->MACA0LR_BIT.MAC_ADDR3=MAC_Address[3];

	ETH->MACA0HR_BIT.MAC_ADDR4=MAC_Address[4];
	ETH->MACA0HR_BIT.MAC_ADDR5=MAC_Address[5];
}

void Drivers_STM32F4xx_Ether_MAC_SET_MAC_Start(void)
{
	ETH->MACCR_BIT.TE=1;
	ETH->MACCR_BIT.RE=1;

	ETH->DMAOMR_BIT.FTF=1;

	ETH->DMAOMR_BIT.ST=1;

	ETH->DMAOMR_BIT.SR=1;
}
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_Stop(void)
{
	ETH->DMAOMR_BIT.ST=0;
	ETH->DMAOMR_BIT.SR=0;
	ETH->MACCR_BIT.RE=0;
	ETH->DMAOMR_BIT.FTF=1;
	ETH->MACCR_BIT.TE=0;
}
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_10M_HalfDuplex(void)
{
	ETH->MACCR_BIT.FES=0;
	ETH->MACCR_BIT.DM=0;
}
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_100M_HalfDuplex(void)
{
	ETH->MACCR_BIT.FES=1;
	ETH->MACCR_BIT.DM=0;
}
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_10M_FullDuplex(void)
{
	ETH->MACCR_BIT.FES=0;
	ETH->MACCR_BIT.DM=1;
}
void Drivers_STM32F4xx_Ether_MAC_SET_MAC_100M_FullDuplex(void)
{
	ETH->MACCR_BIT.FES=1;
	ETH->MACCR_BIT.DM=1;
}


void ETH_DMATxDescChainInit(void)
{
	uint32_t i = 0;
	ETH_DMADESCTypeDef *DMATxDesc;

	/* Set the DMATxDescToSet pointer with the first one of the DMATxDescTab list */
	//DMATxDescToSet = DMATxDscrTab;
	/* Fill each DMATxDesc descriptor with the right values */
	for(i=0; i < ETH_TXBUFNB; i++)
	{
		/* Get the pointer on the ith member of the Tx Desc list */
		DMATxDesc = DMATxDscrTab + i;
		/* Set Second Address Chained bit */
		DMATxDesc->DES0 = ETH_DMATxDesc_TCH;

		/* Set Buffer1 address pointer */
		DMATxDesc->Buffer1Addr = (uint32_t)(&Tx_Buff[i][0]);

		/* Initialize the next descriptor with the Next Descriptor Polling Enable */
		if(i < (ETH_TXBUFNB-1))
		{
			/* Set next descriptor address register with next descriptor base address */
			DMATxDesc->Buffer2NextDescAddr = (uint32_t)(DMATxDscrTab+i+1);
		}
		else
		{
			/* For last descriptor, set next descriptor address register equal to the first descriptor base address */
			DMATxDesc->Buffer2NextDescAddr = (uint32_t) DMATxDscrTab;
		}
	}

	/* Set Transmit Desciptor List Address Register */
	ETH->DMATDLAR = (uint32_t) DMATxDscrTab;
}

void ETH_DMARxDescChainInit(void)
{
	uint32_t i = 0;
	ETH_DMADESCTypeDef *DMARxDesc;

	/* Set the DMARxDescToGet pointer with the first one of the DMARxDescTab list */
	//DMARxDescToGet = DMARxDescTab;
	/* Fill each DMARxDesc descriptor with the right values */
	for(i=0; i < ETH_RXBUFNB; i++)
	{
		/* Get the pointer on the ith member of the Rx Desc list */
		DMARxDesc = DMARxDscrTab+i;
		/* Set Own bit of the Rx descriptor Status */
		DMARxDesc->DES0 = ETH_DMARxDesc_OWN;

		/* Set Buffer1 size and Second Address Chained bit */
		//DMARxDesc->ControlBufferSize = ETH_DMARxDesc_RCH | (uint32_t)ETH_RX_BUF_SIZE;
		DMARxDesc->DES1_BIT.Rx.RBS1=(uint32_t)ETH_RX_BUF_SIZE;
		DMARxDesc->DES1_BIT.Rx.RCH=1;

		/* Set Buffer1 address pointer */
		DMARxDesc->Buffer1Addr = (uint32_t)(&Rx_Buff[i][0]);

		/* Initialize the next descriptor with the Next Descriptor Polling Enable */
		if(i < (ETH_RXBUFNB-1))
		{
			/* Set next descriptor address register with next descriptor base address */
			DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDscrTab+i+1);
		}
		else
		{
			/* For last descriptor, set next descriptor address register equal to the first descriptor base address */
			DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDscrTab);
		}
	}

	/* Set Receive Descriptor List Address Register */
	ETH->DMARDLAR = (uint32_t) DMARxDscrTab;

	//DMA_RX_FRAME_infos = &RX_Frame_Descriptor;

}

