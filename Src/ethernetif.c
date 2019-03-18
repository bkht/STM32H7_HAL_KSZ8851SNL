/**
  ******************************************************************************
  * File Name          : ethernetif.c
  * Description        : This file provides code for the configuration
  *                      of the ethernetif.c MiddleWare.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
// Based on
// https://github.com/avrxml/asf/blob/master/thirdparty/lwip/lwip-port-1.4.1/sam/netif/sam0_spi_ksz8851snl.c

/* Includes ------------------------------------------------------------------*/
#include <dmc_print.h>
#include <KSZ8851SNL.h>
#include <KSZ8851SNL_reg.h>
#include "main.h"
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include "lan8742.h"
#include <string.h>

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/

/* Network interface name */
#define IFNAME0 'e'
#define IFNAME1 'n'

/** Maximum transfer unit. */
#define NET_MTU								1500

/** Network link speed. */
#define NET_LINK_SPEED						100000000

/**
 * ksz8851snl driver structure.
 */
struct ksz8851snl_device {
	/** Set to 1 when owner is software (ready to read), 0 for Micrel. */
	uint32_t rx_desc[NETIF_RX_BUFFERS];
	/** Set to 1 when owner is Micrel, 0 for software. */
	uint32_t tx_desc[NETIF_TX_BUFFERS];
	/** RX pbuf pointer list */
	struct pbuf *rx_pbuf[NETIF_RX_BUFFERS];
	/** TX pbuf pointer list */
	struct pbuf *tx_pbuf[NETIF_TX_BUFFERS];
	struct pbuf *tx_cur_pbuf;

	/** Circular buffer head pointer for packet received. */
	uint32_t us_rx_head;
	/** Circular buffer tail pointer for packet to be read. */
	uint32_t us_rx_tail;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_head;
	/** Circular buffer tail pointer incremented by handlers (buffer sent). */
	uint32_t us_tx_tail;

	/** Reference to lwIP netif structure. */
	struct netif *netif;

#if NO_SYS == 0
	/** RX task notification semaphore. */
	sys_sem_t sync_sem;
#endif
};

/**
 * ksz8851snl driver instance.
 */
static struct ksz8851snl_device gs_ksz8851snl_dev;

static uint16_t pending_frame = 0;

/**
 * MAC address to use.
 */
static uint8_t gs_uc_mac_address[] =
{
	ETHERNET_CONF_ETHADDR0,
	ETHERNET_CONF_ETHADDR1,
	ETHERNET_CONF_ETHADDR2,
	ETHERNET_CONF_ETHADDR3,
	ETHERNET_CONF_ETHADDR4,
	ETHERNET_CONF_ETHADDR5
};

#if LWIP_STATS
/** Used to compute lwIP bandwidth. */
uint32_t lwip_tx_count = 0;
uint32_t lwip_rx_count = 0;
uint32_t lwip_tx_rate = 0;
uint32_t lwip_rx_rate = 0;
#endif

volatile uint32_t g_intn_flag = 0;
volatile uint32_t g_spi_pdc_flag = 0;
//extern Pdc *g_p_spi_pdc;

#define SPI_PDC_IDLE		0
#define SPI_PDC_RX_START	1
#define SPI_PDC_RX_COMPLETE	2
#define SPI_PDC_TX_START	3
#define SPI_PDC_TX_COMPLETE	4

extern uint8_t tmpbuf[];


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
/* 
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers are allocated statically and passed directly to the LwIP stack
          they will return back to DMA after been processed by the stack.
        - Tx Buffers will be allocated from LwIP stack memory heap, 
          then passed to ETH HAL driver.

@Notes: 
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_RX_DESC_CNT in ETH GUI (Rx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_TX_DESC_CNT in ETH GUI (Tx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_MAX_PACKET_SIZE, this value must
       passed to ETH DMA in the init field (heth.Init.RxBuffLen)
*/

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

uint32_t current_pbuf_idx = 0;

/* Global Ethernet handle */
ETH_HandleTypeDef heth;
ETH_TxPacketConfig TxConfig;
struct pbuf_custom rx_pbuf[ETH_RX_DESC_CNT];

/* Private function prototypes -----------------------------------------------*/
int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);

lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                                  ETH_PHY_IO_DeInit,
                                  ETH_PHY_IO_WriteReg,
                                  ETH_PHY_IO_ReadReg,
                                  ETH_PHY_IO_GetTick};

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/
void pbuf_free_custom(struct pbuf *p);
void Error_Handler(void);

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
  
    /**ETH GPIO Configuration    
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB13     ------> ETH_TXD1
    PG11     ------> ETH_TX_EN
    PG13     ------> ETH_TXD0 
    */
    GPIO_InitStruct.Pin = RMII_MDC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(RMII_MDC_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* Must be GPIO_SPEED_FREQ_HIGH */
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_RXD0_Pin|RMII_RXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* Must be GPIO_SPEED_FREQ_HIGH */
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* Must be GPIO_SPEED_FREQ_HIGH */
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* Must be GPIO_SPEED_FREQ_HIGH */
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
    HAL_NVIC_SetPriority(ETH_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ETH_WKUP_IRQn);
  /* USER CODE BEGIN ETH_MspInit 1 */

  /* USER CODE END ETH_MspInit 1 */
  }
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Disable Peripheral clock */
    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1RX_CLK_DISABLE();
  
    /**ETH GPIO Configuration    
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB13     ------> ETH_TXD1
    PG11     ------> ETH_TX_EN
    PG13     ------> ETH_TXD0 
    */
    HAL_GPIO_DeInit(GPIOC, RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin);

    HAL_GPIO_DeInit(GPIOA, RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin);

    HAL_GPIO_DeInit(RMII_TXD1_GPIO_Port, RMII_TXD1_Pin);

    HAL_GPIO_DeInit(GPIOG, RMII_TX_EN_Pin|RMII_TXD0_Pin);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(ETH_IRQn);

    HAL_NVIC_DisableIRQ(ETH_WKUP_IRQn);

  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH) 
*******************************************************************************/
/**
 * @brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{ 


	/* Set MAC hardware address length. */
	netif->hwaddr_len = sizeof(gs_uc_mac_address);
	/* Set MAC hardware address. */
	netif->hwaddr[0] = gs_uc_mac_address[0];
	netif->hwaddr[1] = gs_uc_mac_address[1];
	netif->hwaddr[2] = gs_uc_mac_address[2];
	netif->hwaddr[3] = gs_uc_mac_address[3];
	netif->hwaddr[4] = gs_uc_mac_address[4];
	netif->hwaddr[5] = gs_uc_mac_address[5];

	/* Set maximum transfer unit. */
	netif->mtu = NET_MTU;

	/* Device capabilities. */
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP
#if defined(DHCP_USED)
			| NETIF_FLAG_DHCP
#endif
	;

	ksz8851snl_rx_init(&gs_ksz8851snl_dev);
	ksz8851snl_tx_init(&gs_ksz8851snl_dev);

	/* Enable NVIC interrupts. */
//	NVIC_SetPriority(KSZ8851SNL_SPI_IRQn, INT_PRIORITY_SPI);
//	NVIC_EnableIRQ(KSZ8851SNL_SPI_IRQn);

	/* Initialize SPI link. */
	if (0 != ksz8851_init()) {
		LWIP_DEBUGF(NETIF_DEBUG,
				("ksz8851snl_low_level_init: failed to initialize the Micrel driver!\n"));
		LWIP_ASSERT("SPI communication issue", 1);
	}

	/* Initialize interrupt line INTN. */
//	configure_intn(INTN_Handler);


//  uint32_t idx = 0;
//  HAL_StatusTypeDef hal_eth_init_status;
//
//	dmc_puts("low_level_init\n");
//
//  /* Init ETH */
//
//  uint8_t MACAddr[6] ;
//  heth.Instance = ETH;
//  MACAddr[0] = gs_uc_mac_address[0];
//  MACAddr[1] = gs_uc_mac_address[1];
//  MACAddr[2] = gs_uc_mac_address[2];
//  MACAddr[3] = gs_uc_mac_address[3];
//  MACAddr[4] = gs_uc_mac_address[4];
//  MACAddr[5] = gs_uc_mac_address[5];
////  MACAddr[0] = 0xba;
////  MACAddr[1] = 0xbe;
////  MACAddr[2] = 0xde;
////  MACAddr[3] = 0xaf;
////  MACAddr[4] = 0xbe;
////  MACAddr[5] = 0xef;
//  heth.Init.MACAddr = &MACAddr[0];
//  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
//  heth.Init.TxDesc = DMATxDscrTab;
//  heth.Init.RxDesc = DMARxDscrTab;
//  heth.Init.RxBuffLen = 1524;
//
//  /* USER CODE BEGIN MACADDRESS */
//
//  /* USER CODE END MACADDRESS */
//
//  // If left out
//  // Assertion "pc->custom_free_function != NULL" failed at line 761 in ..\..\..\Middlewares\Third_Party\LwIP\src\core\pbuf.c
//  hal_eth_init_status = HAL_ETH_Init(&heth);
//
//  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
//  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
//  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
//  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
//
////  for(idx = 0; idx < ETH_RX_DESC_CNT; idx ++)
////  {
////    HAL_ETH_DescAssignMemory(&heth, idx, Rx_Buff[idx], NULL);
////
////    /* Set Custom pbuf free function */
////    rx_pbuf[idx].custom_free_function = pbuf_free_custom;
////  }
//
//
//#if LWIP_ARP || LWIP_ETHERNET
//
//  /* set MAC hardware address length */
//  netif->hwaddr_len = ETH_HWADDR_LEN;
//
//  /* set MAC hardware address */
//  netif->hwaddr[0] =  heth.Init.MACAddr[0];
//  netif->hwaddr[1] =  heth.Init.MACAddr[1];
//  netif->hwaddr[2] =  heth.Init.MACAddr[2];
//  netif->hwaddr[3] =  heth.Init.MACAddr[3];
//  netif->hwaddr[4] =  heth.Init.MACAddr[4];
//  netif->hwaddr[5] =  heth.Init.MACAddr[5];
//
//  /* maximum transfer unit */
//  netif->mtu = NET_MTU;
//
//  /* Accept broadcast address and ARP traffic */
//  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
//  #if LWIP_ARP
//    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
//  #else
//    netif->flags |= NETIF_FLAG_BROADCAST;
//  #endif /* LWIP_ARP */
//
//
///* USER CODE BEGIN PHY_PRE_CONFIG */
//
///* USER CODE END PHY_PRE_CONFIG */
//
//    ksz8851_init();
////    ksz8851_init(heth.Init.MACAddr);
//
//
//
//  /* Set PHY IO functions */
//  // lan8742.c, 81
////  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
//
//  /* Initialize the LAN8742 ETH PHY */
//  // lan8742.c, 106
////  LAN8742_Init(&LAN8742);
//
////  if (hal_eth_init_status == HAL_OK)
////  {
////    netif_set_up(netif);
////    netif_set_link_up(netif);
////    HAL_ETH_Start(&heth);
////  }
////  else
////  {
////    Error_Handler();
////  }
//
///* USER CODE BEGIN PHY_POST_CONFIG */
//
///* USER CODE END PHY_POST_CONFIG */
//
//#endif /* LWIP_ARP || LWIP_ETHERNET */



/* USER CODE BEGIN LOW_LEVEL_INIT */ 
    
/* USER CODE END LOW_LEVEL_INIT */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = netif->state;

	dmc_puts("low_level_output\n");

	/* Make sure the next descriptor is free. */
	if (ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_head]) {
#if NO_SYS
//		LWIP_DEBUGF(NETIF_DEBUG,
//				("ksz8851snl_low_level_output: out of free descriptor! [tail=%u head=%u]\n",
//				ps_ksz8851snl_dev->us_tx_tail, ps_ksz8851snl_dev->us_tx_head));
#endif
		return ERR_IF;
	}

	/* Ensure lwIP won't free this pbuf before the Micrel actually sends it. */
	pbuf_ref(p);

	/* Mark descriptor has owned by Micrel. Enqueue pbuf packet. */
	ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_head] = 1;
	ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_head] = p;

#if NO_SYS
//	LWIP_DEBUGF(NETIF_DEBUG,
//			("ksz8851snl_low_level_output: DMA buffer 0x%p sent, size=%u [tail=%u head=%u]\n",
//			p->payload, p->tot_len, ps_ksz8851snl_dev->us_tx_tail, ps_ksz8851snl_dev->us_tx_head));
#endif

	ps_ksz8851snl_dev->us_tx_head = (ps_ksz8851snl_dev->us_tx_head + 1) % NETIF_TX_BUFFERS;

#if LWIP_STATS
	lwip_tx_count += p->tot_len;
#endif
	LINK_STATS_INC(link.xmit);

#if NO_SYS == 0
	/* Release KSZ task to perform packet transfer. */
	xSemaphoreGive(ps_ksz8851snl_dev->sync_sem);
#endif

	return ERR_OK;
//   uint32_t i=0, framelen = 0;
//  struct pbuf *q;
//  err_t errval = ERR_OK;
//  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
//
//	dmc_puts("low_level_output\n");
//
//  for(q = p; q != NULL; q = q->next)
//  {
//    if(i >= ETH_TX_DESC_CNT)
//      return ERR_IF;
//
//    Txbuffer[i].buffer = q->payload;
//    Txbuffer[i].len = q->len;
//    framelen += q->len;
//
//    if(i>0)
//    {
//      Txbuffer[i-1].next = &Txbuffer[i];
//    }
//
//    if(q->next == NULL)
//    {
//      Txbuffer[i].next = NULL;
//    }
//
//    i++;
//  }
//
//  TxConfig.Length = framelen;
//  TxConfig.TxBuffer = Txbuffer;
//
//  /* Clean and Invalidate data cache */
//  SCB_CleanInvalidateDCache();
//
////  HAL_ETH_Transmit(&heth, &TxConfig, 0);
//  ksz8851_send_packet(TxConfig.TxBuffer, TxConfig.Length); // uint16_t pTXLength, uint8_t *pTXData
//
//  return errval;
}

/**
 * \brief Populate the RX descriptor ring buffers with pbufs.
 *
 * \param p_ksz8851snl_dev Pointer to driver data structure.
 */
static void ksz8851snl_rx_populate_queue(struct ksz8851snl_device *p_ksz8851snl_dev)
{
	uint32_t ul_index = 0;
	struct pbuf *p = 0;

		dmc_puts("low_level_input\n");

	/* Set up the RX descriptors */
	for (ul_index = 0; ul_index < NETIF_RX_BUFFERS; ul_index++) {
		if (p_ksz8851snl_dev->rx_pbuf[ul_index] == 0) {

			/* Allocate a new pbuf with the maximum size. */
			p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
			if (0 == p) {
				LWIP_DEBUGF(NETIF_DEBUG, ("ksz8851snl_rx_populate_queue: pbuf allocation failure\n"));
			}

			/* Make sure lwIP is well configured so one pbuf can contain the maximum packet size. */
			LWIP_ASSERT("ksz8851snl_rx_populate_queue: pbuf size too small!", pbuf_clen(p) <= 1);

			/* Set owner as Micrel. */
			p_ksz8851snl_dev->rx_desc[ul_index] = 0;

			/* Save pbuf pointer to be sent to lwIP upper layer. */
			p_ksz8851snl_dev->rx_pbuf[ul_index] = p;

			LWIP_DEBUGF(NETIF_DEBUG,
					("ksz8851snl_rx_populate_queue: new pbuf allocated with size %d: 0x%p [pos=%u]\n",
					PBUF_POOL_BUFSIZE, p, ul_index));
		}
	}
}



/**
 * \brief Update Micrel state machine and perform required actions.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void ksz8851snl_update(struct netif *netif)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = netif->state;
	uint16_t status = 0;
	uint16_t len = 0;
	uint16_t txmir = 0;

	//	dmc_puts("ksz8851snl_update\n");

	/* Check for free PDC. */
	if (SPI_PDC_IDLE == g_spi_pdc_flag)
	{

		/* Handle TX. */
		/* Fetch next packet marked as owned by Micrel. */
		if (ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_tail] && (pending_frame == 0))
		{
			len = ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail]->tot_len;

			/* TX step1: check if TXQ memory size is available for transmit. */
			txmir = ksz8851_reg_read(REG_TX_MEM_INFO) & TX_MEM_AVAILABLE_MASK;
			if (txmir < len + 8)
			{
				LWIP_DEBUGF(NETIF_DEBUG,
						("ksz8851snl_update: TX not enough memory in queue: %d required %d\n",
								txmir, len + 8));
				return;
			}

			/* TX step2: disable all interrupts. */
			ksz8851_reg_write_0(REG_INT_MASK, 0);

			LWIP_DEBUGF(NETIF_DEBUG,
					("ksz8851snl_update: TX start packet transmit len=%d [tail=%u head=%u]\n",
							len,
							ps_ksz8851snl_dev->us_tx_tail, ps_ksz8851snl_dev->us_tx_head));

			/* TX step3: enable TXQ write access. */
			ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);

			/* TX step4-8: perform FIFO write operation. */
			g_spi_pdc_flag = SPI_PDC_TX_START;
			ps_ksz8851snl_dev->tx_cur_pbuf = ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail];
	//			gpio_set_pin_low(KSZ8851SNL_CSN_GPIO);
	//			ksz8851_fifo_write(ps_ksz8851snl_dev->tx_cur_pbuf->payload,
	//					ps_ksz8851snl_dev->tx_cur_pbuf->tot_len,
	//					ps_ksz8851snl_dev->tx_cur_pbuf->len);
			ksz8851_fifo_write(ps_ksz8851snl_dev->tx_cur_pbuf->payload,
					ps_ksz8851snl_dev->tx_cur_pbuf->tot_len);
		}

		/* Handle RX. */
		else if (g_intn_flag || pending_frame > 0)
		{
			g_intn_flag = 0;

			if (0 == pending_frame)
			{
				/* RX step1: read interrupt status for INT_RX flag. */
				status = ksz8851_reg_read(REG_INT_STATUS);
				if (!(status & INT_RX))
				{
					return;
				}

				/* RX step2: disable all interrupts. */
				ksz8851_reg_write_0(REG_INT_MASK, 0);

				/* RX step3: clear INT_RX flag. */
				ksz8851_reg_setbits(REG_INT_STATUS, INT_RX);

				/* RX step4-5: check for received frames. */
				pending_frame = ksz8851_reg_read(REG_RX_FRAME_CNT_THRES) >> 8;
				if (0 == pending_frame)
				{
					/* RX step24: enable INT_RX flag. */
					ksz8851_reg_write_0(REG_INT_MASK, INT_RX);
					return;
				}
			}

			/* Don't break Micrel state machine, wait for a free descriptor first! */
			if (ps_ksz8851snl_dev->rx_desc[ps_ksz8851snl_dev->us_rx_head])
			{
				LWIP_DEBUGF(NETIF_DEBUG,
						("ksz8851snl_update: out of free descriptor! [tail=%u head=%u]\n",
								ps_ksz8851snl_dev->us_rx_tail, ps_ksz8851snl_dev->us_rx_head));
				return;
			}
			if (0 == ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head])
			{
				ksz8851snl_rx_populate_queue(ps_ksz8851snl_dev);
				LWIP_DEBUGF(NETIF_DEBUG,
						("ksz8851snl_update: descriptor with NULL pbuf! [head=%u]\n",
								ps_ksz8851snl_dev->us_rx_head));
				return;
			}

			/* RX step6: get RX packet status. */
			status = ksz8851_reg_read(REG_RX_FHR_STATUS);
			if (((status & RX_VALID) == 0) || (status & RX_ERRORS))
			{
				ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);
				LWIP_DEBUGF(NETIF_DEBUG, ("ksz8851snl_update: RX packet error!\n"));
			}
			else
			{
				/* RX step7: read frame length. */
				len = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT) & RX_BYTE_CNT_MASK;

				/* RX step8: Drop packet if len is invalid or no descriptor available. */
				if (0 == len)
				{
					ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);
					LWIP_DEBUGF(NETIF_DEBUG, ("ksz8851snl_update: RX bad len!\n"));
				}
				else
				{
					LWIP_DEBUGF(NETIF_DEBUG,
							("ksz8851snl_update: RX start packet receive len=%d [tail=%u head=%u]\n",
									len,
									ps_ksz8851snl_dev->us_rx_tail, ps_ksz8851snl_dev->us_rx_head));

					/* RX step9: reset RX frame pointer. */
					ksz8851_reg_clrbits(REG_RX_ADDR_PTR, ADDR_PTR_MASK);

					/* RX step10: start RXQ read access. */
					ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);

					/* RX step11-17: start asynchronous FIFO read operation. */
					g_spi_pdc_flag = SPI_PDC_RX_START;
	//					gpio_set_pin_low(KSZ8851SNL_CSN_GPIO);
					ksz8851_fifo_read(ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->payload, len);

					/* Remove CRC and update pbuf length. */
					len -= 4;
					ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->len = len;
					ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->tot_len = len;
				}
			}
		}
	}
	else if (SPI_PDC_RX_COMPLETE == g_spi_pdc_flag)
	{
		/* RX step18-19: pad with dummy data to keep dword alignment. */
		len = ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->tot_len & 3;
		if (len)
		{
	//			ksz8851_fifo_dummy(4 - len);
		}

		/* RX step20: end RX transfer. */
	//		gpio_set_pin_high(KSZ8851SNL_CSN_GPIO);
		/* Disable asynchronous transfer mode. */
		g_spi_pdc_flag = SPI_PDC_IDLE;

		/* RX step21: end RXQ read access. */
		ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);

		/* RX step22-23: update frame count to be read. */
		pending_frame -= 1;

		/* RX step24: enable INT_RX flag if transfer complete. */
		if (0 == pending_frame)
		{
			ksz8851_reg_write_0(REG_INT_MASK, INT_RX);
		}

		/* Mark descriptor ready to be read. */
		ps_ksz8851snl_dev->rx_desc[ps_ksz8851snl_dev->us_rx_head] = 1;
		ps_ksz8851snl_dev->us_rx_head = (ps_ksz8851snl_dev->us_rx_head + 1) % NETIF_RX_BUFFERS;
	}
	else if (SPI_PDC_TX_COMPLETE == g_spi_pdc_flag)
	{
		/* TX step9-10: pad with dummy data to keep dword alignment. */
		len = ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail]->tot_len & 3;
		if (len)
		{
	//			ksz8851_fifo_dummy(4 - len);
		}

		/* TX step11: end TX transfer. */
	//		gpio_set_pin_high(KSZ8851SNL_CSN_GPIO);
		/* Disable asynchronous transfer mode. */
		g_spi_pdc_flag = SPI_PDC_IDLE;

		/* TX step12: disable TXQ write access. */
		ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);

		/* TX step12.1: enqueue frame in TXQ. */
		ksz8851_reg_setbits(REG_TXQ_CMD, TXQ_ENQUEUE);

		/* RX step13: enable INT_RX flag. */
		ksz8851_reg_write_0(REG_INT_MASK, INT_RX);

		/* Buffer sent, free the corresponding buffer and mark descriptor as owned by software. */
		pbuf_free(ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail]);
		ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail] = NULL;
		ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_tail] = 0;
		ps_ksz8851snl_dev->us_tx_tail = (ps_ksz8851snl_dev->us_tx_tail + 1) % NETIF_TX_BUFFERS;
	}
}


/**
 * \brief Set up the RX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for RX packets.
 *
 * \param ps_ksz8851snl_dev Pointer to driver data structure.
 */
void ksz8851snl_rx_init(struct ksz8851snl_device *ps_ksz8851snl_dev)
{
	uint32_t ul_index = 0;

	dmc_puts("ksz8851snl_rx_init\n");

	/* Init pointer index. */
	ps_ksz8851snl_dev->us_rx_head = 0;
	ps_ksz8851snl_dev->us_rx_tail = 0;

	/* Set up the RX descriptors. */
	for (ul_index = 0; ul_index < NETIF_RX_BUFFERS; ul_index++) {
		ps_ksz8851snl_dev->rx_pbuf[ul_index] = 0;
		ps_ksz8851snl_dev->rx_desc[ul_index] = 0;
	}

	/* Build RX buffer and descriptors. */
	ksz8851snl_rx_populate_queue(ps_ksz8851snl_dev);
}

/**
 * \brief Set up the TX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for TX packets.
 *
 * \param ps_ksz8851snl_dev Pointer to driver data structure.
 */
void ksz8851snl_tx_init(struct ksz8851snl_device *ps_ksz8851snl_dev)
{
	uint32_t ul_index = 0;

	dmc_puts("ksz8851snl_tx_init\n");

	/* Init TX index pointer. */
	ps_ksz8851snl_dev->us_tx_head = 0;
	ps_ksz8851snl_dev->us_tx_tail = 0;

	/* Set up the TX descriptors */
	for (ul_index = 0; ul_index < NETIF_TX_BUFFERS; ul_index++) {
		ps_ksz8851snl_dev->tx_desc[ul_index] = 0;
	}
}


/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
   */
static struct pbuf * low_level_input(struct netif *netif)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = netif->state;
	struct pbuf *p = 0;

//	dmc_puts("low_level_input\n");

	/* Check that descriptor is owned by software (ie packet received). */
	if (ps_ksz8851snl_dev->rx_desc[ps_ksz8851snl_dev->us_rx_tail]) {

		/* Fetch pre-allocated pbuf. */
		p = ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_tail];

		/* Remove this pbuf from its descriptor. */
		ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_tail] = 0;

		LWIP_DEBUGF(NETIF_DEBUG,
				("ksz8851snl_low_level_input: DMA buffer 0x%p received, size=%u [tail=%u head=%u]\n",
				p->payload, p->tot_len, ps_ksz8851snl_dev->us_rx_tail, ps_ksz8851snl_dev->us_rx_head));

		/* Set pbuf total packet size. */
		LINK_STATS_INC(link.recv);

		/* Fill empty descriptors with new pbufs. */
		ksz8851snl_rx_populate_queue(ps_ksz8851snl_dev);

		ps_ksz8851snl_dev->us_rx_tail = (ps_ksz8851snl_dev->us_rx_tail + 1) % NETIF_RX_BUFFERS;

#if LWIP_STATS
		lwip_rx_count += p->tot_len;
#endif
	}

	return p;

//  struct pbuf *p = NULL;
//  ETH_BufferTypeDef RxBuff;
//  uint32_t framelength = 0;
//
//
//  if (HAL_ETH_IsRxDataAvailable(&heth))
//  {
//    /* Clean and Invalidate data cache */
//    SCB_CleanInvalidateDCache();
//
//// Jack
////    HAL_ETH_GetRxDataBuffer(&heth, &RxBuff);
////    HAL_ETH_GetRxDataLength(&heth, &framelength);
//    framelength = ksz8851_read_packet(&RxBuff, MAX_FRAMELEN);	// uint8_t *pRXData, uint16_t pRXLength
//
//    p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_POOL, &rx_pbuf[current_pbuf_idx], RxBuff.buffer, framelength);
//
//    if(current_pbuf_idx < (ETH_RX_DESC_CNT -1))
//    {
//      current_pbuf_idx++;
//    }
//    else
//    {
//      current_pbuf_idx = 0;
//    }
//
//    return p;
//  }
//  else
//  {
//    return NULL;
//  }
}


#if NO_SYS == 0
/**
 * \brief This function waits for the notification
 * semaphore from the interrupt, processes the incoming packet and then
 * passes it to the lwIP stack.
 *
 * \param pvParameters A pointer to the ksz8851snl_device instance.
 */
static void ksz8851snl_task(void *pvParameters)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = pvParameters;

	while (1) {
		/* Block if no transfer pending. */
		if ((SPI_PDC_IDLE == g_spi_pdc_flag) &&
				(0 == pending_frame) &&
				(0 == ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_tail])) {
			/* Wait for the RX notification semaphore. */
			sys_arch_sem_wait(&ps_ksz8851snl_dev->sync_sem, 0);
		}

		/* Process the incoming packet. */
		ethernetif_input(ps_ksz8851snl_dev->netif);
	}
}
#endif


/**
 * \brief Handler for INTN falling edge interrupt.
 */
void INTN_Handler()
{
	dmc_puts("INTN_Handler\n");
#if NO_SYS == 0
	portBASE_TYPE xKSZTaskWoken = pdFALSE;
#endif

	/* Enable INTN flag. */
	g_intn_flag = 1;

#if NO_SYS == 0
	xSemaphoreGiveFromISR(gs_ksz8851snl_dev.sync_sem, &xKSZTaskWoken);
	portEND_SWITCHING_ISR(xKSZTaskWoken);
#endif
}



/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(struct netif *netif)
{
	struct eth_hdr *ethhdr;
	struct pbuf *p;

//		dmc_puts("ethernetif_input\n");

	/* Update driver state machine. */
	ksz8851snl_update(netif);

	/* Move received packet into a new pbuf. */
	p = low_level_input(netif);
	if (p == NULL)
		return;

	/* Points to packet payload, which starts with an Ethernet header. */
	ethhdr = p->payload;

	switch (htons(ethhdr->type)) {
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
#if PPPOE_SUPPORT
		case ETHTYPE_PPPOEDISC:
		case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
			/* Send packet to lwIP for processing. */
			if (netif->input(p, netif) != ERR_OK) {
//				LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
				/* Free buffer. */
				pbuf_free(p);
			}
			break;

		default:
			/* Free buffer. */
			pbuf_free(p);
			break;
	}
//  err_t err;
//  struct pbuf *p;
//
//  /* move received packet into a new pbuf */
//  p = low_level_input(netif);
//
//  /* no packet could be read, silently ignore this */
//  if (p == NULL) return;
//
//  /* entry point to the LwIP stack */
//  err = netif->input(p, netif);
//
//  if (err != ERR_OK)
//  {
//    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
//    pbuf_free(p);
//    p = NULL;
//  }
//  HAL_ETH_BuildRxDescriptors(&heth);
  
}


/* USER CODE BEGIN 8 */
/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file
            when the callback is needed,
  */

}
/* USER CODE END 8 */
//#endif /* LWIP_NETIF_LINK_CALLBACK */


#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{  
  err_t errval;
  errval = ERR_OK;
    
/* USER CODE BEGIN 5 */ 
    
/* USER CODE END 5 */  
    
  return errval;
  
}

#endif /* LWIP_ARP */ 

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
	LWIP_ASSERT("netif != NULL", (netif != NULL));

	gs_ksz8851snl_dev.netif = netif;

	dmc_puts("ethernetif_init\n");

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "ksz8851snldev";
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
#if LWIP_SNMP
	NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, NET_LINK_SPEED);
#endif /* LWIP_SNMP */

	netif->state = &gs_ksz8851snl_dev;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;

	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;
	/* Initialize the hardware. */
	low_level_init(netif);

#if NO_SYS == 0
	err_t err;
	sys_thread_t id;

	/* Incoming packet notification semaphore. */
	err = sys_sem_new(&gs_ksz8851snl_dev.sync_sem, 0);
	LWIP_ASSERT("ethernetif_init: ksz8851snl RX semaphore allocation ERROR!\n",
			(err == ERR_OK));
	if (err == ERR_MEM)
		return ERR_MEM;

	id = sys_thread_new("ksz8851", ksz8851snl_task, &gs_ksz8851snl_dev,
			netifINTERFACE_TASK_STACK_SIZE, netifINTERFACE_TASK_PRIORITY);
	LWIP_ASSERT("ethernetif_init: ksz8851snl Task allocation ERROR!\n",
			(id != 0));
	if (id == 0)
		return ERR_MEM;
#endif

	return ERR_OK;
//  LWIP_ASSERT("netif != NULL", (netif != NULL));
//
//	dmc_puts("ethernetif_init\n");
//
//#if LWIP_NETIF_HOSTNAME
//  /* Initialize interface hostname */
//  netif->hostname = "lwip";
//#endif /* LWIP_NETIF_HOSTNAME */
//
//  netif->name[0] = IFNAME0;
//  netif->name[1] = IFNAME1;
//  /* We directly use etharp_output() here to save a function call.
//   * You can instead declare your own function an call etharp_output()
//   * from it if you have to do some checks before sending (e.g. if link
//   * is available...) */
//
//#if LWIP_IPV4
//#if LWIP_ARP || LWIP_ETHERNET
//#if LWIP_ARP
//  netif->output = etharp_output;
//#else
//  /* The user should write ist own code in low_level_output_arp_off function */
//  netif->output = low_level_output_arp_off;
//#endif /* LWIP_ARP */
//#endif /* LWIP_ARP || LWIP_ETHERNET */
//#endif /* LWIP_IPV4 */
//
//#if LWIP_IPV6
//  netif->output_ip6 = ethip6_output;
//#endif /* LWIP_IPV6 */
//
//  netif->linkoutput = low_level_output;
//
//  /* initialize the hardware */
//  low_level_init(netif);
//
//  return ERR_OK;
}

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  if(p!=NULL)
  {
    p->flags = 0;
    p->next = NULL;
    p->len = p->tot_len = 0;
    p->ref = 0;
    p->payload = NULL;
  }
}

/* USER CODE BEGIN 6 */

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Current Time value
*/
u32_t sys_jiffies(void)
{
  return HAL_GetTick();
}

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Current Time value
*/
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/* USER CODE END 6 */

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{  
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here 
  */
  
  /* Configure the MDIO Clock */
//  HAL_ETH_SetMDIOClockRange(&heth);
  
  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit(void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
	*pRegVal = (uint32_t) ksz8851_reg_read((uint16_t) RegAddr);
//  if(HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
//  {
//    return -1;
//  }
  
  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
	ksz8851_reg_write_0((uint16_t) RegAddr, (uint16_t) RegVal);
//  if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
//  {
//    return -1;
//  }
  
  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

/* USER CODE BEGIN 8 */

/* USER CODE END 8 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



/**
  * @brief
  * @retval None
  */
void ethernet_link_check_state(struct netif *netif)
{
//  ETH_MACConfigTypeDef MACConf;
//  uint32_t PHYLinkState;
//  uint32_t linkchanged = 0, speed = 0, duplex =0;
//
//  PHYLinkState = LAN8742_GetLinkState(&LAN8742);
//
//  if(netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN))
//  {
//    HAL_ETH_Stop(&heth);
//    netif_set_down(netif);
//    netif_set_link_down(netif);
//  }
//  else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
//  {
//    switch (PHYLinkState)
//    {
//    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
//      duplex = ETH_FULLDUPLEX_MODE;
//      speed = ETH_SPEED_100M;
//      linkchanged = 1;
//      break;
//    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
//      duplex = ETH_HALFDUPLEX_MODE;
//      speed = ETH_SPEED_100M;
//      linkchanged = 1;
//      break;
//    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
//      duplex = ETH_FULLDUPLEX_MODE;
//      speed = ETH_SPEED_10M;
//      linkchanged = 1;
//      break;
//    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
//      duplex = ETH_HALFDUPLEX_MODE;
//      speed = ETH_SPEED_10M;
//      linkchanged = 1;
//      break;
//    default:
//      break;
//    }
//
//    if(linkchanged)
//    {
//      /* Get MAC Config MAC */
//      HAL_ETH_GetMACConfig(&heth, &MACConf);
//      MACConf.DuplexMode = duplex;
//      MACConf.Speed = speed;
//      HAL_ETH_SetMACConfig(&heth, &MACConf);
//      HAL_ETH_Start(&heth);
//      netif_set_up(netif);
//      netif_set_link_up(netif);
//    }
//  }
}

