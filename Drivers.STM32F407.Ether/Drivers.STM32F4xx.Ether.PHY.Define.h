/*
 * Drivers.STM32F4xx.Ether.PHY.Define.h
 *
 *  Created on: 2019Äê5ÔÂ21ÈÕ
 *      Author: Master.HE
 */

#ifndef DRIVERS_STM32F4XX_ETHER_PHY_DEFINE_H_
#define DRIVERS_STM32F4XX_ETHER_PHY_DEFINE_H_

#define ETHERNET_PHY_ADDRESS       0x00

#define PHY_READ_TO                     ((uint32_t)0x0004FFFF)
#define PHY_WRITE_TO                    ((uint32_t)0x0004FFFF)


/* The LAN8742A PHY status register  */
#define PHY_SR                 ((uint16_t)0x001F) /* PHY status register Offset */
#define PHY_SPEED_STATUS       ((uint16_t)0x0004) /* PHY Speed mask  1:10Mb/s       0:100Mb/s*/
#define PHY_DUPLEX_STATUS      ((uint16_t)0x0010) /* PHY Duplex mask 1:Full duplex  0:Half duplex*/


/** @defgroup PHY_Register_address
  * @{
  */
#define PHY_BCR                          0          /*!< Transceiver Basic Control Register */
#define PHY_BSR                          1          /*!< Transceiver Basic Status Register */

#define IS_ETH_PHY_ADDRESS(ADDRESS) ((ADDRESS) <= 0x20)
#define IS_ETH_PHY_REG(REG) (((REG) == PHY_BCR) || \
                             ((REG) == PHY_BSR) || \
                             ((REG) == PHY_SR))
/**
  * @}
  */



/**
  * @}
  */




#endif /* DRIVERS_STM32F4XX_ETHER_PHY_DEFINE_H_ */
