/*
 * The MIT License (MIT)
 *
 *
 * https://github.com/fabiobaltieri/avr-nrf/blob/master/firmware/ksz8851snl.c
 * https://github.com/marcorussi/iot_eth_server
 * https://siliconlabs.github.io/Gecko_SDK_Doc/efr32mg1/html/ksz8851snl_8c_source.html
 * https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
 * https://github.com/avrxml/asf/blob/master/sam/components/ethernet_phy/ksz8851snl/ksz8851snl.c
 * https://www.oryx-embedded.com/doc/ksz8851_8c_source.html
 * https://code.zoetrope.io/bfo/nrf52-freertos/blob/edf7bd8206281b3e5d9c71e39c9c8ed2b0710ce6/lib/FreeRTOS-Plus-TCP/portable/NetworkInterface/ksz8851snl/ksz8851snl.c
 * TODO Check below code
 * https://github.com/atx/avr-uip-2/blob/master/drivers/ksz8851/ksz8851.c
 * https://github.com/Velleman/VM204-Firmware/blob/master/cyclone_tcp/drivers/ksz8851.c
 * https://github.com/RevolutionPi/piControl/blob/master/ksz8851.c
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* ------------------- Local inclusions -------------------- */
#include <dmc_print.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpio.h>
#include <dmc_terminal.h>
#include "stm32h7xx_hal.h"
#include "conf_eth.h"
#include "ksz8851snl.h"
#include "ksz8851snl_reg.h"

/** Network interface identifier. */
#define IFNAME0								'e'
#define IFNAME1								'n'

/** Maximum transfer unit. */
#define NET_MTU								1500

/** Network link speed. */
#define NET_LINK_SPEED						100000000

#define MAX_FRAMES_IN_RXQ	16

/* ------------------- Local variables -------------------- */

/**
 * ksz8851snl driver structure.
 */
struct ksz8851snl_device
{
	/** Set to 1 when owner is software (ready to read), 0 for Micrel. */
	uint32_t rx_desc[NETIF_RX_BUFFERS];
	/** Set to 1 when owner is Micrel, 0 for software. */
	uint32_t tx_desc[NETIF_TX_BUFFERS];
	/** RX pbuf pointer list */
	struct pbuf *rx_pbuf[NETIF_RX_BUFFERS];
	/** TX pbuf pointer list */
	struct pbuf *tx_pbuf[NETIF_TX_BUFFERS];

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
//	sys_sem_t sync_sem;
#endif
};

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
ETHERNET_CONF_ETHADDR5 };

typedef struct KSZ8851SLN_mib_s
{
	uint32_t RxByteCnt;
	uint32_t RxUndersizePktCnt;
	uint32_t RxFragmentsCnt;
	uint32_t RxOversizeCnt;
	uint32_t RxJabbersCnt;
	uint32_t RxSymbolErrorCnt;
	uint32_t RxCRCErrorCnt;
	uint32_t RxPausePktsCnt;
	uint32_t RxBroadcastCnt;
	uint32_t RxMulticastCnt;
	uint32_t RxUnicastCnt;
	uint32_t TxByteCnt;
	uint32_t TxPausePktsCnt;
	uint32_t TxBroadcastPktsCnt;
	uint32_t TxMulticastPktsCnt;
	uint32_t TxUnicastPktsCnt;
	uint32_t TxDeferredCnt;
	uint32_t TxTotalCollisionCnt;
} KSZ8851SLN_mib_t;

typedef struct FR_HEADER_INFO_s
{
	uint16_t rxStatus;
	uint16_t rxLength;
} FR_HEADER_INFO_t;

static struct ksz8851snl_device gs_ksz8851snl_dev;

static uint16_t pending_frame = 0;

static SPI_HandleTypeDef SpiHandle;

static uint16_t rxqcr;
static uint8_t fid;

static uint16_t frameId = 0;
static uint8_t macAddress[6];
static uint16_t rxFrameCount;
static uint32_t interruptCnt = 0;
static KSZ8851SLN_mib_t mibCounters;


static uint16_t isr_old = 0;
volatile uint16_t isr_reg = 0;
volatile uint8_t isr_ocurred = 0;

volatile uint8_t dma_ended = 0;

/* ------------------- Local functions prototypes -------------------- */
uint8_t ksz8851_interface_init(void);
void ksz8851_hard_reset(void);
void ksz8851_soft_reset(uint8_t queue_only);

/* ------------------- Local functions -------------------- */
// Below functions are implemented in stm32h7xx_it.c
//// SPI2_RX
//void DMA1_Stream3_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(SpiHandle.hdmarx);
//}
//
//// SPI2_TX
//void DMA1_Stream4_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(SpiHandle.hdmatx);
//}
//
//void SPI2_IRQHandler(void)
//{
//	HAL_SPI_IRQHandler(&SpiHandle);
//}


/**
 * \brief Initialize SPI bus
 * \param None
 * \return Error code
 */
uint8_t ksz8851_interface_init(void)
{
	bool success = true;

	dmc_puts("ksz8851_interface_init\n");

	/* Configure the SPI peripheral */
	/* Set the SPI parameters */
	SpiHandle.Instance = SPI2;
	SpiHandle.Init.Mode = SPI_MODE_MASTER;
	SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
	SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
	SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SpiHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;	// SPI_NSS_SOFT
	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;	// KSZ8851SNL can handle up to 50 MHz.
	SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
	SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial = 7;
	SpiHandle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	SpiHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	SpiHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	SpiHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	SpiHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	SpiHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	SpiHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	SpiHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	SpiHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	SpiHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&SpiHandle) != HAL_OK)
	{
		success = false;
	}

	return success;
}

/**
 * \brief Perform KSZ8851 hard reset
 * \param None
 * \return None
 */
void ksz8851_hard_reset(void)
{
	/* Perform hardware reset */
	HAL_GPIO_WritePin(KSZ8851_RST_GPIO_Port, KSZ8851_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(KSZ8851_RST_GPIO_Port, KSZ8851_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

/**
 * \brief Perform KSZ8851 soft reset
 * \param[in] Software reset QMU or GLOBAL
 * \return None
 */
void ksz8851_soft_reset(uint8_t queue_only)
{
	if (queue_only)
	{
		/* Reset QMU Modules(flush out TXQ and RXQ) */
		ksz8851_reg_setbits(REG_RESET_CTRL, QMU_SOFTWARE_RESET);
		HAL_Delay(1);
		ksz8851_reg_clrbits(REG_RESET_CTRL, QMU_SOFTWARE_RESET);
	}
	else
	{
		/* Perform Global Soft Reset (clear registers of PHY, MAC, QMU, DMA) */
		ksz8851_reg_setbits(REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
		HAL_Delay(1);
		ksz8851_reg_clrbits(REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
	}
//	if (queue_only)
//		ksz8851_reg_write(REG_RESET_CTRL, QMU_SOFTWARE_RESET);
//	else
//		ksz8851_reg_write(REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
//	HAL_Delay(1);
//	ksz8851_reg_write(REG_RESET_CTRL, 0x00);
	HAL_Delay(1);
}

/**
 * \brief Read KSZ8851 register
 * \param[in] address Register address
 * \return Register value
 */
uint16_t ksz8851_reg_read(uint16_t reg)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	uint16_t cmd = 0;
	uint16_t rddata = 0;

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	/* Move register address to cmd bits 9-2 */
	cmd = (reg << 2) & REG_ADDR_MASK;	// & 0x3F0

	/* Add byte enables to cmd */
	/* Last 2 bits still under "don't care bits" handled with byte enable. */
	/* Select byte enable for command. */
	if (reg & 2)
	{
		/* Odd word address writes bytes 2 and 3 */
		cmd |= (0xc << 10);
	}
	else
	{
		/* Even word address write bytes 0 and 1 */
		cmd |= (0x3 << 10);
	}

	/* Add opcode to cmd */
	/* Add command read code. */
	cmd |= CMD_READ;
	outbuf[0] = cmd >> 8;
	outbuf[1] = cmd & 0xff;
	outbuf[2] = 0xff;
	outbuf[3] = 0xff;

//  dmc_puts("outbuf: ");
//  dmc_puthex2(outbuf[0]);
//  dmc_puthex2(outbuf[1]);
//  dmc_puthex2(outbuf[2]);
//  dmc_puthex2(outbuf[3]);
//  dmc_putcr();

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) outbuf, (uint8_t *) inbuf, 4, 2000);

//	SET_SPI_CS_PIN();
//	HAL_Delay(2);
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

//  dmc_puts("inbuf : ");
//  dmc_puthex2(inbuf[0]);
//  dmc_puthex2(inbuf[1]);
//  dmc_puthex2(inbuf[2]);
//  dmc_puthex2(inbuf[3]);
//  dmc_putcr();

	rddata = (inbuf[3] << 8) | inbuf[2];
	return rddata;
}

/**
 * \brief Write KSZ8851 register
 * \param[in] address Register address
 * \param[in] data Register value
 */
void ksz8851_reg_write(uint16_t reg, uint16_t wrdata)
{
	uint8_t outbuf[4];
	uint16_t cmd = 0;

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	/* Move register address to cmd bits 9-2 */
	cmd = (reg << 2) & REG_ADDR_MASK;
	// 0x90 << 2 = 0x240 & 0x3F0 = 0x240


	/* Add byte enables to cmd */
	/* Last 2 bits still under "don't care bits" handled with byte enable. */
	/* Select byte enable for command. */
	if (reg & 2)
	{
		/* Odd word address writes bytes 2 and 3 */
		// 0x3000 0011 0000 0000 0000
		cmd |= (0xc << 10);
	}
	else
	{
		/* Even word address write bytes 0 and 1 */
		// 0x0C00 0000 1100 0000 0000
		cmd |= (0x3 << 10);
	}

	/* Add opcode to cmd */
	/* Add command write code. */
	cmd |= CMD_WRITE;
	outbuf[0] = cmd >> 8;
	outbuf[1] = cmd & 0xff;
	outbuf[2] = wrdata & 0xff;
	outbuf[3] = wrdata >> 8;

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_Transmit(&SpiHandle, (uint8_t*) outbuf, 4, 2000);

//	SET_SPI_CS_PIN();
//	HAL_Delay(2);
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

/**
 * \brief Read register content, set bitmask and write back to register.
 * \param reg the register address to modify.
 * \param bits_to_set bitmask to apply.
 */
void ksz8851_reg_setbits(uint16_t reg, uint16_t bits_to_set)
{
	uint16_t temp;

	// In order to set a bit in a register, such as during initialization,
	// read the register first, modify the target bit only and write it back.
	temp = ksz8851_reg_read(reg);
	temp |= bits_to_set;
	ksz8851_reg_write(reg, temp);
}

/**
 * \brief Read register content, clear bitmask and write back to register.
 * \param reg the register address to modify.
 * \param bits_to_set bitmask to apply.
 */
void ksz8851_reg_clrbits(uint16_t reg, uint16_t bits_to_clr)
{
	uint16_t temp;

	// In order to set a bit in a register, such as during initialization,
	// read the register first, modify the target bit only and write it back.
	temp = ksz8851_reg_read(reg);
	temp &= ~bits_to_clr;
	ksz8851_reg_write(reg, temp);
}

/**
 * \brief KSZ8851 controller initialization
 * \param None
 * \return Error code
 */
uint8_t ksz8851_init(void)
{
	uint32_t count = 0;
	uint16_t dev_id = 0;
	bool success = true;
	uint8_t MACaddress[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00 };

	dmc_puts("ksz8851_init\n");

	/* Configure the SPI peripheral */
	if (ksz8851_interface_init() == false)
	{
		success = false;
		dmc_puts("FAILED: ksz8851_interface_init\n");
		return success;
	}

	// Init step1: read chip ID.
	/* Reset the Micrel in a proper state. */
	uint32_t TickResetMicrel = HAL_GetTick();
	do
	{
		/* Perform hardware reset */
		ksz8851_hard_reset();

//		/* Perform Global Soft Reset (clear registers of PHY, MAC, QMU, DMA) */
//		ksz8851_reg_setbits(REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
//		HAL_Delay(1);
//		ksz8851_reg_clrbits(REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
//
//		/* Reset QMU Modules(flush out TXQ and RXQ) */
//		ksz8851_reg_setbits(REG_RESET_CTRL, QMU_SOFTWARE_RESET);
//		HAL_Delay(1);
//		ksz8851_reg_clrbits(REG_RESET_CTRL, QMU_SOFTWARE_RESET);

		// Read the device chip ID
		uint32_t TickReadID = HAL_GetTick();
		do
		{
			// Read the device chip ID, make sure it is correct ID 0x8872;
			// otherwise there are some errors on the host bus interface.
			// dev_id = 0x8872
			// Bit 15-8 = Chip family ID, 0x88
			// Bit 7-4 = Chip ID, 0x7 is assigned to KSZ8851SNL
			// Bit 3-1 = Revision ID, 0x2 >> 1 = 1
			// Bit 0 = Reserved
			dev_id = ksz8851_reg_read(REG_CHIP_ID); /* CIDER */
      dmc_puts(TERMINAL_YELLOW);
      dmc_puts("Chip ID: ");
      dmc_puthex4cr(dev_id);
      dmc_puts(TERMINAL_DEFAULT);

//			if ((dev_id & 0xFFF0) != CHIP_ID_8851_16)
//			{
//				dmc_puts("Expected Device ID 0x");
//				dmc_puthex4(CHIP_ID_8851_16);
//				dmc_puts(", got 0x");
//				dmc_puthex4cr(dev_id);
//			}
//			else
//			{
//				dmc_puts("Device ID: 0x");
//				dmc_puthex4cr(dev_id);
//			}
			// Try for 10mS maximum
			if (HAL_GetTick() - TickReadID > 10)
			{
				break;
			}
		}
		while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);
//		while ((dev_id & 0xFFF0) != 0x1234);

		// Try for 100mS maximum
		if (HAL_GetTick() - TickResetMicrel > 100)
		{
      dmc_puts(TERMINAL_LIGHT_RED);
      dmc_puts("FAILED: wrong Chip ID: ");
      dmc_puthex4cr(dev_id);
      dmc_puts(TERMINAL_DEFAULT);
//			return 1;
		}
	}
	while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);
//	while ((dev_id & 0xFFF0) != 0x1234);
  dmc_puts(TERMINAL_LIGHT_GREEN);
  dmc_puts("Device ID: ");
  dmc_puthex4cr(dev_id);
  dmc_puts(TERMINAL_DEFAULT);


//		dmc_puts("MAC_ADDR: ");
//		dmc_puthex2(MACaddress[0]);
//		dmc_puts(":");
//		dmc_puthex2(MACaddress[1]);
//		dmc_puts(":");
//		dmc_puthex2(MACaddress[2]);
//		dmc_puts(":");
//		dmc_puthex2(MACaddress[3]);
//		dmc_puts(":");
//		dmc_puthex2(MACaddress[4]);
//		dmc_puts(":");
//		dmc_puthex2cr(MACaddress[5]);

	/* Init step2-4: Write QMU MAC address (low, middle then high). */
	// Write QMU MAC address (low)
	ksz8851_reg_write(REG_MAC_ADDR_0, (MACaddress[4] << 8) | MACaddress[5]); /* MARL */
	// Write QMU MAC address (Medium)
	ksz8851_reg_write(REG_MAC_ADDR_2, (MACaddress[2] << 8) | MACaddress[3]); /* MARM */
	// Write QMU MAC address (High)
	ksz8851_reg_write(REG_MAC_ADDR_4, (MACaddress[0] << 8) | MACaddress[1]); /* MARH */

	// Just checking...
	uint16_t addr_0 = ksz8851_reg_read(REG_MAC_ADDR_0);
	uint16_t addr_2 = ksz8851_reg_read(REG_MAC_ADDR_2);
	uint16_t addr_4 = ksz8851_reg_read(REG_MAC_ADDR_4);
	dmc_puts("MAC_ADDR: ");
	dmc_puthex2(addr_4 >> 8);
	dmc_puts(":");
	dmc_puthex2(addr_4 & 0xff);
	dmc_puts(":");
	dmc_puthex2(addr_2 >> 8);
	dmc_puts(":");
	dmc_puthex2(addr_2 & 0xff);
	dmc_puts(":");
	dmc_puthex2(addr_0 >> 8);
	dmc_puts(":");
	dmc_puthex2cr(addr_0 & 0xff);

	/* Init step5: Enable QMU Transmit Frame Data Pointer Auto Increment. */
	ksz8851_reg_write(REG_TX_ADDR_PTR,	/* TXFDPR */
			ADDR_PTR_AUTO_INC);			/* Enable Frame data pointer increments automatically */
//		uint16_t TX_ADDR_PTR = ksz8851_reg_read(REG_TX_ADDR_PTR);
//		dmc_puts("TX_ADDR_PTR: ");
//		dmc_puthex4cr(TX_ADDR_PTR);

	/* Enable QMU TxQ Auto-Enqueue frame */
//	ksz8851_reg_write(REG_TXQ_CMD, TXQ_AUTO_ENQUEUE);

	/* Init step6: Configure QMU transmit control register. */
	/* Enable QMU Transmit:
	 * Flow control,
	 * Transmit padding,
	 * Transmit CRC,
	 * and IP/TCP/UDP/ICMP checksum generation.
	 */
	// Packets shorter than 64 bytes are padded and the CRC is automatically generated
	ksz8851_reg_write(REG_TX_CTRL,		/* TXCR */
			TX_CTRL_ICMP_CHECKSUM | 	/* Enable ICMP frame checksum generation */
			TX_CTRL_UDP_CHECKSUM |		/* Enable UDP frame checksum generation */
			TX_CTRL_TCP_CHECKSUM |   	/* Enable TCP frame checksum generation */
			TX_CTRL_IP_CHECKSUM |   	/* Enable IP frame checksum generation */
			TX_CTRL_FLOW_ENABLE |   	/* Enable transmit flow control */
			TX_CTRL_PAD_ENABLE |   		/* Eanble adding a padding to a packet shorter than 64 bytes */
			TX_CTRL_CRC_ENABLE);   		/* Enable adding a CRC to the end of transmit frame */

	/* Init step7: Enable QMU Receive Frame Data Pointer Auto Increment. */
	ksz8851_reg_write(REG_RX_ADDR_PTR,	/* RXFDPR */
			ADDR_PTR_AUTO_INC); 		/* Enable Frame data pointer increments automatically */

	/* Init step8: Configure QMU Receive Frame Threshold for one frame. */
	ksz8851_reg_write(REG_RX_FRAME_CNT_THRES, 1); /* RXFCTFC */

	/* Init step9: Configure QMU receive control register1. */
	/* Enable QMU Receive:
	 * Flow control,
	 * Receive all broadcast frames,
	 * receive unicast frames, and
	 * IP/TCP/UDP checksum verification.
	 */
	ksz8851_reg_write(REG_RX_CTRL1,		/* RXCR1 */
			RX_CTRL_UDP_CHECKSUM |		/* Enable UDP frame checksum verification */
			RX_CTRL_TCP_CHECKSUM |		/* Enable TCP frame checksum verification */
			RX_CTRL_IP_CHECKSUM |		/* Enable IP frame checksum verification */
			RX_CTRL_MAC_FILTER |		/* Receive with address that pass MAC address filtering */
			RX_CTRL_FLOW_ENABLE |		/* Enable receive flow control */
			RX_CTRL_BROADCAST |			/* Receive all the broadcast frames */
			RX_CTRL_ALL_MULTICAST |		/* Receive all the multicast frames (including broadcast frames) */
			RX_CTRL_UNICAST |			/* Receive unicast frames that match the device MAC address */
			RX_CTRL_PROMISCUOUS);		/* Receive all incoming frames, regardless of frame's DA */

	/* Init step10: Configure QMU receive control register2. */
	/* Enable QMU Receive:
	 * ICMP frame checksum verification,
	 * UDP Lite frame checksum verification/generation,
	 * IPv4/IPv6 UDP fragment frame pass
	 * IPv4/IPv6 UDP UDP checksum field is zero pass,
	 * single frame data burst if SPI master controller could read a single frame data in one SPI CSN activate,
	 * and IPv6 UDP fragment frame pass.
	 */
	ksz8851_reg_write(REG_RX_CTRL2,			/* RXCR2 */
			RX_CTRL_IPV6_UDP_NOCHECKSUM |	/* No checksum generation and verification if IPv6 UDP is fragment */
			RX_CTRL_UDP_LITE_CHECKSUM |		/* Enable UDP Lite frame checksum generation and verification */
			RX_CTRL_ICMP_CHECKSUM |			/* Enable ICMP frame checksum verification */
			RX_CTRL_BURST_LEN_FRAME);

	/* Init step11: Configure QMU receive queue: trigger INT and auto-dequeue frame. */
	/* Enable QMU Receive:
	 * IP Header Two-Byte Offset,
	 * Receive Frame Count Threshold,
	 * and RXQ Auto-Dequeue frame.
	 */
	ksz8851_reg_write(REG_RXQ_CMD,			/* RXQCR */
			RXQ_EN_ON_FRAME_CNT_INT |		/* Enable RX interrupt by frame count threshold */
			RXQ_TWOBYTE_OFFSET |			/* Enable adding 2-bytes offset before IP frame header */
			RXQ_AUTO_DEQUEUE);				/* Enable Auto Dequeue RXQ Frame */

	/* Init step12: adjust SPI data output delay. */
	ksz8851_reg_write(REG_BUS_CLOCK_CTRL,	/* OBCR */
			BUS_CLOCK_166 |					/* 166 MHz on-chip bus clock (defaul is 125MHz) */
			BUS_CLOCK_DIVIDEDBY_1);			/* Bus clock devided by 1 */

	/* Init step13: Restart Port 1 auto-negotiation. */
	ksz8851_reg_setbits(REG_PORT_CTRL,		/* P1CR */
			PORT_AUTO_NEG_RESTART);			/* Restart auto-negotiation */

	/* Init step13.1: Force link in half duplex if auto-negotiation failed. */
	if ((ksz8851_reg_read(REG_PORT_CTRL) & PORT_AUTO_NEG_RESTART) != PORT_AUTO_NEG_RESTART)	/* P1CR */
	{
		ksz8851_reg_clrbits(REG_PORT_CTRL,	/* P1CR */
				PORT_FORCE_FULL_DUPLEX);	/* Force PHY in full duplex mode */
//		dmc_puts("Full Duplex\n");
	}

	/* Init step14: Clear interrupt status. */
	ksz8851_reg_write(REG_INT_STATUS, 0xFFFF);	/* ISR */

	/* Init step14.1: Configure Low Watermark to 6KByte available buffer space out of 12KByte. */
	ksz8851_reg_write(REG_RX_LOW_WATERMARK,		/* FCLWR */
			WATERMARK_6KB);						/* 6KByte Watermark */

	/* Init step14.2: Configure High Watermark to 4KByte available buffer space out of 12KByte. */
	ksz8851_reg_write(REG_RX_HIGH_WATERMARK,	/* FCHWR */
			WATERMARK_4KB);						/* 4KByte Watermark */

	/* Init step15: Set interrupt mask. */
	/*
	 * Enable Link Change\Transmit\Receive if your host processor can handle the interrupt,
	 * otherwise do not need to do this step.
	 */
//		ksz8851_reg_write(REG_INT_ENABLE, 0x0000);
//		ksz8851_reg_write(REG_INT_ENABLE, INT_RX);
//		ksz8851_reg_write(REG_INT_ENABLE, INT_RX | INT_PHY);
	ksz8851_reg_write(REG_INT_ENABLE,			/* IER */
			KSZ8851SNL_INT_RX_DONE |		/** Enable transmit done interrupt */
			KSZ8851SNL_INT_RX_STOPPED |		/** Enable receive process stopped interrupt */
			KSZ8851SNL_INT_TX_STOPPED |		/** Enable transmit process stopped interrupt */
			KSZ8851SNL_INT_LINK_CHANGE |	/** Enable link change interrupt */
			KSZ8851SNL_INT_RX_SPI_ERROR);	/** Enable receive SPI bus error interrupt */
//			KSZ8851SNL_INT_ENABLE_MASK);

	/* Init step16: Enable QMU Transmit. */
	ksz8851_reg_setbits(REG_TX_CTRL,	/* TXCR */
			TX_CTRL_ENABLE);			/* Enable tranmsit */

	/* Init step17: Enable QMU Receive. */
	ksz8851_reg_setbits(REG_RX_CTRL1,	/* RXCR1 */
			RX_CTRL_ENABLE);			/* Enable receive */


#if MULTI_FRAME
	/* Configure Receive Frame Threshold for 4 frames */
	ksz8851_reg_write(REG_RX_FRAME_CNT_THRES, 4);

	/* Configure Receive Duration Threshold for 1 ms */
	ksz8851_reg_write(REG_RX_TIME_THRES, 0x03e8);

	/* Enable
	 * QMU Recieve IP Header Two-Byte Offset,
	 * Receive Frame Count Threshold,
	 * Receive Duration Timer Threshold, and
	 * RXQ Auto-Dequeue frame.
	 */
	ksz8851_reg_write(REG_RXQ_CMD,
	              RXQ_TWOBYTE_OFFSET |
	              RXQ_TIME_INT | RXQ_FRAME_CNT_INT | RXQ_AUTO_DEQUEUE);
#endif

	return success;
}

void ksz8851_set_pm(uint8_t mode)
{
	uint8_t pmecr;

	pmecr = ksz8851_reg_read(REG_POWER_CNTL);
	pmecr &= ~POWER_STATE_MASK;
	pmecr |= mode;
	ksz8851_reg_write(REG_POWER_CNTL, pmecr);
}

uint8_t ksz8851_has_data(void)
{
	uint16_t isr = ksz8851_reg_read(REG_INT_STATUS);

	ksz8851_reg_write(REG_INT_STATUS, isr);
	if (isr & IRQ_RXI)
	{
		return 1;
	}
	return 0;
}

/* ------------------- Exported functions -------------------- */
uint8_t ksz8851_get_spi_state(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_StateTypeDef spiState;
	uint8_t spiIntCode;

	spiState = HAL_SPI_GetState(hspi);

	switch (spiState)
	{
	case HAL_SPI_STATE_READY:
	{
		spiIntCode = INT_SPI_READY;
		break;
	}
	case HAL_SPI_STATE_BUSY:
	case HAL_SPI_STATE_BUSY_TX:
	case HAL_SPI_STATE_BUSY_RX:
	case HAL_SPI_STATE_BUSY_TX_RX:
	{
		spiIntCode = INT_SPI_BUSY;
		break;
	}
	case HAL_SPI_STATE_ERROR:
	default:
	{
		//TODO: get and understand the error code
		//error_code = HAL_SPI_GetError(hspi);
		spiIntCode = INT_SPI_ERROR;
	}
	}

	return spiIntCode;
}

// Required
void ksz8851_fifo_read(uint8_t *buf, uint16_t len)
{
	uint8_t inbuf[9];
	uint8_t pad_bytes;

	//TODO: check len value

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	/* calculate number of dummy pad bytes to read a 32-bits aligned buffer */
	pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

	inbuf[0] = FIFO_READ;

	/* Perform blocking SPI transfer. */
	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) inbuf, (uint8_t*) inbuf, 9, 2000);

	/* update length to a 32-bits aligned value */
	len += pad_bytes;

	/* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len);
	/* an interrupt will occur */

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
//	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len, 1000);
}

// Required
void ksz8851_fifo_write(uint8_t *buf, uint16_t len)
{
	uint8_t outbuf[5];
	uint8_t pad_bytes;
	static uint8_t frameID = 0;

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	//TODO: check len value

	/* length is 11 bits long */
	len &= 0x07FF;
	/* calculate number of dummy pad bytes to send a 32-bits aligned buffer */
	pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

	/* Prepare control word and byte count. */
	outbuf[0] = FIFO_WRITE;
	outbuf[1] = (frameID++ & FRAME_ID_MASK);
	outbuf[2] = 0;
	/* Write txPacketLength to the "byte count" of the frame header. */
	outbuf[3] = len & LSB_MASK;
	outbuf[4] = len >> MSB_POS;

	/* Perform blocking SPI transfer. */
	(void) HAL_SPI_Transmit(&SpiHandle, (uint8_t*) outbuf, 5, 2000);

	/* update length to a 32-bits aligned value */
	len += pad_bytes;

	/* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
	/* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len);
	/* an interrupt will occur */

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
//	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len, 1000);
}

uint32_t ksz8851_MIBCountersRead(uint16_t offset)
{
	uint16_t data;
	uint32_t counter;

	data = MIB_MASK | offset;
	ksz8851_reg_write(REG_IND_IACR, data);
	counter = ksz8851_reg_read(REG_IND_DATA_LOW);
	counter |= ksz8851_reg_read(REG_IND_DATA_HIGH) << 16;

	return counter;
}

/***************************************************************************//**
 * @brief Dumps the Management Information Base Counters
 * @note  Support method used for debugging.
 *
 * @param param
 *     the string representing the moment of the register dump
 *
 *****************************************************************************/
void KSZ8851SNL_ReadMIBCounters(char* param)
{
	EFM_ASSERT(param != NULL);
	uint16_t data;
	uint32_t counter;
	dmc_puts("Dumping MIB Counters values @");
	dmc_puts(param);
	dmc_puts("\n");
	int i;
	for (i = 0; i < 0x20; i++)
	{
		data = MIB_MASK | i;
		ksz8851_reg_write(REG_IND_IACR, data);
		counter = ksz8851_reg_read(REG_IND_DATA_LOW);
		counter |= ksz8851_reg_read(REG_IND_DATA_HIGH) << 16;
		dmc_puts("MIB_REG[");
		dmc_puthex2(i);
		dmc_puts("] contains ");
		dmc_puthex4(counter);
		dmc_puts("\n");
	}
}

void ksz8851_AllRegistersDump(void)
{
	dmc_puts("###################### ALL REGISTER DUMP ########################\n");
	int i;
	for (i = 0x00; i < 0xFF; i += 0x02)
	{
		if ((i % 8 == 0) && (i > 0))
		{
			dmc_puts("\n");
		}
		dmc_puts("REG[0x");
		dmc_puthex2(i);
		dmc_puts("]=0x");
		dmc_puthex4cr(ksz8851_reg_read(i));
	}
	dmc_puts("\n");
	dmc_puts("#################################################################\n");
}

void ksz8851_RegistersDump(void)
{
	dmc_puts("##################### SPECIAL REGISTER DUMP ######################\n");
//	printf("MARL  [0x%02X]=0x%04X\n", REG_MAC_ADDR_0, ksz8851_reg_read(REG_MAC_ADDR_0));
	dmc_puts("MARL  [0x");
	dmc_puthex2(REG_MAC_ADDR_0);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_MAC_ADDR_0));
//	printf("MARM  [0x%02X]=0x%04X\n", REG_MAC_ADDR_2, ksz8851_reg_read(REG_MAC_ADDR_2));
	dmc_puts("MARM  [0x");
	dmc_puthex2(REG_MAC_ADDR_2);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_MAC_ADDR_2));
//	printf("MARH  [0x%02X]=0x%04X\n", REG_MAC_ADDR_4, ksz8851_reg_read(REG_MAC_ADDR_4));
	dmc_puts("MARH  [0x");
	dmc_puthex2(REG_MAC_ADDR_4);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_MAC_ADDR_4));
//	printf("OBCR  [0x%02X]=0x%04X\n", REG_BUS_CLOCK_CTRL, ksz8851_reg_read(REG_BUS_CLOCK_CTRL));
	dmc_puts("OBCR  [0x");
	dmc_puthex2(REG_BUS_CLOCK_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_BUS_CLOCK_CTRL));
//	printf("GRR   [0x%02X]=0x%04X\n", REG_RESET_CTRL, ksz8851_reg_read(REG_RESET_CTRL));
	dmc_puts("GRR   [0x");
	dmc_puthex2(REG_RESET_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RESET_CTRL));
//	printf("TXCR  [0x%02X]=0x%04X\n", REG_TX_CTRL, ksz8851_reg_read(REG_TX_CTRL));
	dmc_puts("TXCR  [0x");
	dmc_puthex2(REG_TX_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_CTRL));
//	printf("RXCR1 [0x%02X]=0x%04X\n", REG_RX_CTRL1, ksz8851_reg_read(REG_RX_CTRL1));
	dmc_puts("RXCR1 [0x");
	dmc_puthex2(REG_RX_CTRL1);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_CTRL1));
//	printf("RXCR2 [0x%02X]=0x%04X\n", REG_RX_CTRL2, ksz8851_reg_read(REG_RX_CTRL2));
	dmc_puts("RXCR2 [0x");
	dmc_puthex2(REG_RX_CTRL2);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_CTRL2));
//	printf("TXMIR [0x%02X]=0x%04X\n", REG_TX_MEM_INFO, ksz8851_reg_read(REG_TX_MEM_INFO));
	dmc_puts("TXMIR [0x");
	dmc_puthex2(REG_TX_MEM_INFO);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_MEM_INFO));
#if (READ_UNSAFE_REGISTERS)
//	printf("RXFHSR[0x%02X]=0x%04X\n", REG_RX_FHR_STATUS, ksz8851_reg_read(REG_RX_FHR_STATUS));
	dmc_puts("RXFHSR[0x");
	dmc_puthex2(REG_RX_FHR_STATUS);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_FHR_STATUS));
#endif
//	printf("TXQCR [0x%02X]=0x%04X\n", REG_TXQ_CMD, ksz8851_reg_read(REG_TXQ_CMD));
	dmc_puts("TXQCR [0x");
	dmc_puthex2(REG_TXQ_CMD);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TXQ_CMD));
//	printf("RXQCR [0x%02X]=0x%04X\n", REG_RXQ_CMD, ksz8851_reg_read(REG_RXQ_CMD));
	dmc_puts("RXQCR [0x");
	dmc_puthex2(REG_RXQ_CMD);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RXQ_CMD));
//	printf("TXFDPR[0x%02X]=0x%04X\n", REG_TX_ADDR_PTR, ksz8851_reg_read(REG_TX_ADDR_PTR));
	dmc_puts("TXFDPR[0x");
	dmc_puthex2(REG_TX_ADDR_PTR);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_ADDR_PTR));
//	printf("RXFDPR[0x%02X]=0x%04X\n", REG_RX_ADDR_PTR, ksz8851_reg_read(REG_RX_ADDR_PTR));
	dmc_puts("RXFDPR[0x");
	dmc_puthex2(REG_RX_ADDR_PTR);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_ADDR_PTR));
//	printf("IER   [0x%02X]=0x%04X\n", REG_INT_ENABLE, ksz8851_reg_read(REG_INT_ENABLE));
	dmc_puts("IER   [0x");
	dmc_puthex2(REG_INT_ENABLE);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_INT_ENABLE));
//	printf("ISR   [0x%02X]=0x%04X\n", REG_INT_STATUS, ksz8851_reg_read(REG_INT_STATUS));
	dmc_puts("ISR   [0x");
	dmc_puthex2(REG_INT_STATUS);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_INT_STATUS));
//	printf("RXFCTR[0x%02X]=0x%04X\n", KS_RXFCTR, ksz8851_reg_read(KS_RXFCTR));
	dmc_puts("RXFCTR[0x");
	dmc_puthex2(KS_RXFCTR);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(KS_RXFCTR));
#if (READ_UNSAFE_REGISTERS)
//	printf("TXNTFSR[0x%02X]=0x%04X\n", TXNTFSR, ksz8851_reg_read(TXNTFSR));
	dmc_puts("TXNTFSR[0x");
	dmc_puthex2(REG_TX_MEM_INFO);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_MEM_INFO));
#endif
	printf("CIDER [0x%02X]=0x%04X\n", REG_CHIP_ID, ksz8851_reg_read(REG_CHIP_ID));
	dmc_puts("CIDER [0x");
	dmc_puthex2(REG_CHIP_ID);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_CHIP_ID));
//	printf("PHYRR [0x%02X]=0x%04X\n", REG_PHY_RESET, ksz8851_reg_read(REG_PHY_RESET));
	dmc_puts("PHYRR [0x");
	dmc_puthex2(REG_PHY_RESET);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_PHY_RESET));
//	printf("P1MBCR[0x%02X]=0x%04X\n", REG_PHY_CNTL, ksz8851_reg_read(REG_PHY_CNTL));
	dmc_puts("P1MBCR[0x");
	dmc_puthex2(REG_PHY_CNTL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_PHY_CNTL));
//	printf("P1CR  [0x%02X]=0x%04X\n", REG_PORT_CTRL, ksz8851_reg_read(REG_PORT_CTRL));
	dmc_puts("P1CR  [0x");
	dmc_puthex2(REG_PORT_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_PORT_CTRL));
	dmc_puts("#################################################################\n");
}

void ksz8851_IntEnable(void)
{
//	if (interruptCnt)
//	{
//		interruptCnt--;
//	}
//	if (interruptCnt == 0)
//	{
	/* Enable interrupts */
  ksz8851_reg_write(REG_INT_ENABLE,			/* IER */
			KSZ8851SNL_INT_RX_DONE |		/** Enable transmit done interrupt */
			KSZ8851SNL_INT_RX_STOPPED |		/** Enable receive process stopped interrupt */
			KSZ8851SNL_INT_TX_STOPPED |		/** Enable transmit process stopped interrupt */
			KSZ8851SNL_INT_LINK_CHANGE |	/** Enable link change interrupt */
			KSZ8851SNL_INT_RX_SPI_ERROR);	/** Enable receive SPI bus error interrupt */
//    }
}

void ksz8851_IntDisable(void)
{
  ksz8851_reg_write(REG_INT_ENABLE, INT_NO_INT);
//	interruptCnt++;
}

void ksz8851_IntClear(uint16_t flags)
{
  ksz8851_reg_write(REG_INT_STATUS, flags);
}

uint16_t ksz8851_IntGet(void)
{
	return ksz8851_reg_read(REG_INT_STATUS);
}

void ksz8851_PMECRStatusClear(uint16_t flags)
{
	uint16_t status = ksz8851_reg_read(REG_POWER_CNTL) | flags;
	ksz8851_reg_write(REG_POWER_CNTL, status);
}

uint16_t ksz8851_RXQCRGet(void)
{
	return ksz8851_reg_read(REG_RXQ_CMD);
}

uint16_t ksz8851_FrameCounterSet(void)
{
	/* Read the frame count and threshold register */
	uint16_t rxftr = ksz8851_reg_read(KS_RXFCTR);
	/* Extract the actual number of frames from RX_FRAME_THRES_REG */
	rxFrameCount = rxftr >> MSB_POS;
	return rxftr;
}

// Required
uint32_t ksz8851snl_reset_rx(void)
{
	uint16_t usValue;

	usValue = ksz8851_reg_read(REG_RX_CTRL1);
	usValue &= ~( ( uint16_t ) (RX_CTRL_ENABLE | RX_CTRL_FLUSH_QUEUE) );
	ksz8851_reg_write(REG_RX_CTRL1, usValue );
//	HAL_Delay(2);
	ksz8851_reg_write(REG_RX_CTRL1, usValue | RX_CTRL_FLUSH_QUEUE );
//	HAL_Delay(1);
	ksz8851_reg_write(REG_RX_CTRL1, usValue );
//	HAL_Delay(1);
	ksz8851_reg_write(REG_RX_CTRL1, usValue | RX_CTRL_ENABLE );
//	HAL_Delay(1);

	return ( uint32_t )usValue;
}

// Required
uint32_t ksz8851snl_reset_tx(void)
{
	uint16_t usValue;

	usValue = ksz8851_reg_read( REG_TX_CTRL );
	usValue &= ~( ( uint16_t ) (TX_CTRL_ENABLE | TX_CTRL_FLUSH_QUEUE) );
	ksz8851_reg_write(REG_TX_CTRL, usValue );
//	HAL_Delay(2);
	ksz8851_reg_write(REG_TX_CTRL, usValue | TX_CTRL_FLUSH_QUEUE );
//	HAL_Delay(1);
	ksz8851_reg_write(REG_TX_CTRL, usValue );
//	HAL_Delay(1);
	ksz8851_reg_write(REG_TX_CTRL, usValue | TX_CTRL_ENABLE );
//	HAL_Delay(1);

	return ( uint32_t )usValue;
}

uint16_t ksz8851_FrameCounterGet(void)
{
	return rxFrameCount;
}

void ksz8851_Enable(void)
{
	uint16_t data;

	ksz8851_reg_write(REG_INT_ENABLE, KSZ8851SNL_INT_ENABLE_MASK);

	/* Enable QMU Transmit */
	data = ksz8851_reg_read(REG_TX_CTRL);
	data |= TX_FLOW_CTRL_ENABLE;
	ksz8851_reg_write(REG_TX_CTRL, data);

	/* Enable QMU Receive */
	data = ksz8851_reg_read(REG_RX_CTRL1);
	data |= RX_FLOW_CTRL_RX_ENABLE;
	ksz8851_reg_write(REG_RX_CTRL1, data);
}

void ksz8851_ReleaseIncosistentFrame(void)
{
//	uint16_t data;
	/* Issue the Release error frame command */
//	data = ksz8851_reg_read(REG_RXQ_CMD);	/* RXQCR */
//	data |= RXQ_RELEASE_ERROR_FRAME;
//	ksz8851_reg_write(REG_RXQ_CMD, data);	/* RXQCR */
	ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_RELEASE_ERROR_FRAME);	/* RXQCR */
	/* Wait for PHY to clear the command/flag */
	while (ksz8851_reg_read(REG_RXQ_CMD) & RXQ_RELEASE_ERROR_FRAME)
	{
		;
	}
}

// Jack, KSZ8851SNL Transmit Steps
/***************************************************************************//**
 * @brief Performs the actual transmit of a raw frame over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 * @param pTXData
 *     the data of the transmitted frame
 *****************************************************************************/
void ksz8851_Send(uint8_t *pTXData, uint16_t pTXLength)
{
//  EFM_ASSERT(pTXData != NULL);
	// Page 22
	uint16_t data;
	uint16_t txmir;
	uint16_t txPacketLength;
	uint8_t outbuf[4096];

	/* Step 1 */
	/* Check if TXQ has enough memory for the transmission of the package */
	/* Read value from TXMIR to check if QMU TXQ has enough amount of memory
	 * for the Ethernet packet data plus 4-byte frame header, plus 4-byte for
	 * DWORD alignment. Compare the read value with (txPacketLength+4+4),
	 * if less than (txPacketLength+4+4), Exit. */

	data = ksz8851_reg_read(REG_TX_MEM_INFO);	/* TXMIR */
	txmir = data & TX_MEM_AVAIL_MASK;			/* 0x1FFF */
	/* plus 4-byte frame header, plus 4-byte for DWORD alignment: EXTRA_SIZE = 8 bytes */
	txPacketLength = pTXLength + EXTRA_SIZE;
	if (txmir < txPacketLength)
	{
		// QMU TXQ does not have enough space
//		dmc_puts(TERMINAL_YELLOW);
//		dmc_puts("Wait until QMU TXQ space is available... ");

//		KSZ8851SNL_ExceptionHandler(INFO, "I will wait until mem is available\n");

		/* Enable TX memory available monitor */
	  ksz8851_reg_write(REG_TX_TOTAL_FRAME_SIZE, txPacketLength);	/* TXNTFSR */
//		data = ksz8851_reg_read(REG_TXQ_CMD);					/* TXQCR */
//		data |= TXQ_MEM_AVAILABLE_INT;
//		ksz8851_reg_write(REG_TXQ_CMD, data);					/* TXQCR */
		ksz8851_reg_setbits(REG_TXQ_CMD, TXQ_MEM_AVAILABLE_INT);	/* TXQCR */

		/* Wait until enough space is available */
		while (1)
		{
			data = ksz8851_reg_read(REG_INT_STATUS);	/* ISR */
			if (data & INT_TX_SPACE)
			{
//				dmc_puts(".");
				break;
			}
		}
//		KSZ8851SNL_ExceptionHandler(INFO, "Done\n");
//		dmc_puts("Done\n");
//		dmc_puts(TERMINAL_DEFAULT);

		/* Clear flag */
		// Acknowledge (clear) INT_TX_SPACE Interrupt bit.
		// Note we have to set the bit in ISR to clear it!
//		data = ksz8851_reg_read(REG_INT_STATUS);	/* ISR */
//		data |= INT_TX_SPACE;
//		ksz8851_reg_write(REG_INT_STATUS, data);	/* ISR */
		ksz8851_reg_setbits(REG_INT_STATUS, INT_TX_SPACE);	/* ISR */
	}

	/* Step 2 */
	/* Disable all interrupts on KSZ8851SNL */
	ksz8851_reg_write(REG_INT_ENABLE, NO_INT);	/* IER */

	/* Step 3 */
	/* Enable TXQ write access */
//	data = ksz8851_reg_read(REG_RXQ_CMD);	/* RXQCR */
//	data |= RXQ_START;						/* bit 3 or 0x0008 */
//	ksz8851_reg_write(REG_RXQ_CMD, data);	/* RXQCR */
	ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);	/* RXQCR */

	// For debugging probe
	HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, PF13_Test_Pin, GPIO_PIN_RESET);

	//	dmc_puts("Perform blocking SPI transfer\n");

//	// 5 bytes
//	outbuf[0] = FIFO_WRITE;
//	outbuf[1] = (frameId++ & FRAME_ID_MASK);
//	outbuf[2] = 0;
//	outbuf[3] = pTXLength & LSB_MASK;
//	outbuf[4] = pTXLength >> MSB_POS;
//	/* Perform blocking SPI transfer. */
//	(void) HAL_SPI_Transmit(&SpiHandle, (uint8_t*) outbuf, 5, 2000);

//	// 4 bytes
//	outbuf[0] = (frameId++ & FRAME_ID_MASK) | TX_INT_on_COMPLETION >> MSB_POS;
//	outbuf[1] = 0;
//	outbuf[2] = pTXLength & LSB_MASK;
//	outbuf[3] = pTXLength >> MSB_POS;
//	for (uint16_t i = 0; i < pTXLength; i++)
//	{
//		pTXData2[i + 4] = pTXData[i];
//	}
//	pTXLength += 4;

	// Step 6
	// Write TXIC to the "control word" of the frame header. (0x8000)
	// Number of SCLK for register access as following:
	// Write frame to TXQ: CMD(8bits) + Frame Header(32bits) + Frame Data(8bits*N)
	// Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
	pTXData[0] = FIFO_WRITE;
	pTXData[1] = (frameId++ & FRAME_ID_MASK);
//	pTXData[1] = (frameId++ & FRAME_ID_MASK) | TX_INT_on_COMPLETION >> MSB_POS;
	pTXData[2] = 0;
	// Step 7
	// Write txPacketLength to the "byte count" of the frame header.
	uint16_t TXLen = pTXLength - 5;
	pTXData[3] = TXLen & LSB_MASK;
	pTXData[4] = TXLen >> MSB_POS;

//	dmc_puts("Perform non-blocking/blocking SPI transfer\n");


	// Write frame data pointer by pTxData to the QMU TXQ in BYTE until
	// finished the full packet length (txPacketLength) in DWORD alignment "bytesToWrite"
	/* Round to DWORD boundary, for example, the driver has to write up
	 * to 68 bytes if transmit frame size is 65 bytes. */
	uint16_t bytesToWrite = 4 * ((TXLen + 3) >> 2) + 5;
//	int lengthInDWord=((pTXLength+3)>>2);
//	int bytesToWrite=(lengthInDWord *4);
//	dmc_putintcr(lengthInByte);

	// https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
//	__DSB();

	// DMA is not working on STM32H7 devices
	// https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
	/* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
	/* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
//	(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) pTXData, (uint8_t*) pRXData, lengthInByte);
	/* an DMA1_Stream4_IRQHandler and DMA1_Stream3_IRQHandler interrupt will occur */
//	(void) HAL_SPI_Transmit_DMA(&SpiHandle, (uint8_t*) pTXData, lengthInByte);
	/* an DMA1_Stream4_IRQHandler interrupt will occur */

//	clr_dma_ended();

	/* Perform blocking SPI transfer. */
	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) pTXData, (uint8_t *) outbuf, bytesToWrite, 2000);

	/* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
//	(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) pTXData, (uint8_t*) outbuf, bytesToWrite);

//	while (dma_ended == 0);

	/* Start the SPI Transfer */
//	ETHSPI_StartWriteFIFO();
	/* Send the frame header info */
//	ETHSPI_WriteFifoContinue(4, outbuf);

	/* Send the actual data */
//	ETHSPI_WriteFifoContinue(pTXLength, pTXData);

	/* Send dummy bytes to align data to DWORD */
//	ETHSPI_WriteFifoContinue(KSZ8851SNL_DwordAllignDiff(pTXLength), pTXData);

	/* Stop the SPI Transfer */
//	ETHSPI_StopFIFO();

	/* Step 12 */
	/* Disable TXQ write access */
//	data = ksz8851_reg_read(REG_RXQ_CMD);	/* RXQCR */
//	data &= ~RXQ_START;						/* bit 3 or 0x0008 */
//	ksz8851_reg_write(REG_RXQ_CMD, data);	/* RXQCR */
	ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);	/* RXQCR */

	/* Step 12.1 */
	/* Start TXQ Manual Enqueue */
//	data = ksz8851_reg_read(REG_TXQ_CMD);	/* RXQCR */
//	data |= TXQ_ENQUEUE;					/* bit 0 or 0x0001 */
//	ksz8851_reg_write(REG_TXQ_CMD, data);	/* RXQCR */
	ksz8851_reg_setbits(REG_TXQ_CMD, TXQ_ENQUEUE);	/* RXQCR */

	/* Wait until transmit command clears */
	while (1)
	{
		data = ksz8851_reg_read(REG_TXQ_CMD);	/* RXQCR */
		if (!(data & TXQ_ENQUEUE))				/* bit 0 or 0x0001 */
		{
			break;
		}
	}

	/* Step 13 */
	/* Enable interrupts */
//	ksz8851_reg_write(REG_INT_ENABLE, INT_MASK_EXAMPLE);
	/* Enable interrupts */
	ksz8851_reg_write(REG_INT_ENABLE,			/* IER */
			KSZ8851SNL_INT_RX_DONE |		/** Enable transmit done interrupt */
			KSZ8851SNL_INT_RX_STOPPED |		/** Enable receive process stopped interrupt */
			KSZ8851SNL_INT_TX_STOPPED |		/** Enable transmit process stopped interrupt */
			KSZ8851SNL_INT_LINK_CHANGE |	/** Enable link change interrupt */
			KSZ8851SNL_INT_RX_SPI_ERROR);	/** Enable receive SPI bus error interrupt */
}

void clr_dma_ended(void)
{
	dma_ended = 0;
}

void set_dma_ended(void)
{
	dma_ended = 1;
}
// Jack, KSZ8851SNL Receive Steps
/***************************************************************************//**
 * @brief Performs the actual receive of a raw frame over the network.
 *
 * @param pRXLength
 *     the length of the received frame
 * @param pRXData
 *     the data of the received frame
 *
 * @return
 *     received packet length, 0 in case of failure
 *****************************************************************************/
uint16_t ksz8851_Receive(uint8_t *pRXData, uint16_t pRXLength)
{
	uint16_t data;
	uint16_t rxFrameCount;
	uint16_t rxStatus;
	uint16_t rxPacketLength;
	uint16_t frameLength;
	uint16_t bytesToRead;
	uint8_t outbuf[4096];

	//	dmc_puts("ksz8851_Receive\n");
//	FR_HEADER_INFO_t rxFrameHeader[MAX_FRAMES_IN_RXQ];

//	EFM_ASSERT(buffer != NULL);
//	EFM_ASSERT(pRXData != NULL);
//	EFM_ASSERT(pRXLength != NULL);

	// Step 1
	// Read value from ISR to check if RXIS ‘Receive Interrupt’ is set. If not set, Exit.
	data = ksz8851_reg_read(REG_INT_STATUS);	/* ISR */
	if (!(data & IRQ_RXI))	/* INT_RX_DONE */
	{
		return 0;
	}
//	dmc_puts("OK\n");
//	dmc_puts(TERMINAL_CYAN);
//	dmc_puts("Handle IRQ_RXI\n");
//	dmc_puts(TERMINAL_DEFAULT);

	// Step 2
	// Disable all the device interrupts generation.
	ksz8851_reg_write(REG_INT_ENABLE, NO_INT);	/* IER */

	// Step 3
	/* Clear flag */
	// Acknowledge (clear) RXIS Receive Interrupt bit.
	// Note we have to set the bit in ISR to clear it!
//	data = ksz8851_reg_read(REG_INT_STATUS);	/* ISR */
//	data |= IRQ_RXI;							/* RXIS */
//	ksz8851_reg_write(REG_INT_STATUS, data);	/* ISR */
	ksz8851_reg_setbits(REG_INT_STATUS, IRQ_RXI);

//	intStatus = ksz8851_reg_read(REG_INT_STATUS);	/* ISR */
//	dmc_puts("REG_INT_STATUS ");
//	dmc_puthex4cr(intStatus);
	// When receive interrupt occurred and software driver writes "1" to clear
	// the RX interrupt in ISR register;
	// the KSZ8851 will update Receive Frame Counter (RXFCTR) Register for this interrupt.

	// Step4
	// Read current total amount of received frame count from RXFCTR, and save in ‘rxFrameCount’.
	/* Read the frame count and threshold register */
	data = ksz8851_reg_read(REG_RX_FRAME_CNT_THRES);	/* RXFCTR */
	/* Extract the actual number of frames from RX_FRAME_THRES_REG*/
	rxFrameCount = data >> MSB_POS;
	// When software driver reads back Receive Frame Count (RXFCTR) Register;
	// the KSZ8851 will update both Receive Frame Header Status
	// and Byte Count Registers (RXFHSR/RXFHBCR).

//	dmc_puts(TERMINAL_MAGENTA);
//	dmc_puts("rxFrameCount: ");
//	dmc_putintcr(rxFrameCount);
//	dmc_puts(TERMINAL_DEFAULT);

	// Step 5
	// Repeatedly reading all frames from RXQ.
	// If rxFrameCount <= 0, goto step 24
	while (rxFrameCount > 0)
	{
		// Step 23
		// Jack: we do this first, so we can use the "continue" statement
		// to skip the rest of the iteration, and continue to next frame
		rxFrameCount--;

		// When software driver reads back both Receive Frame Header Status and Byte Count Registers (RXFHSR/RXFHBCR);
		// the KSZ8851 will update next receive frame header status and byte count registers (RXFHSR/RXFHBCR).

		// Step 6
		// Read received frame status from RXFHSR to check if this is a good frame.
		// This register (read only) indicates the current received frame header status information.
		/* Read the received frame status */
		rxStatus = ksz8851_reg_read(REG_RX_FHR_STATUS);		/* RXFHSR */
//		dmc_puts("rxStatus: ");
//		dmc_puthex4cr(rxStatus);

		// Step 7
		// Read received frame byte size from RXFHBCR to get this received frame length
		// (4-byte CRC, and extra 2-byte due to "IP Header Two-Byte Offset Enable" are included),
		// and store into rxPacketLength variable.
//		data = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT);	/* RXFHBCR */
//		rxPacketLength = data & RX_BYTE_CNT_MASK;		/* 0x0FFF */
////		rxPacketLength -= 11; // 11 preceeding bytes
//		dmc_puts("rxPacketLength: ");
//		dmc_putintcr(rxPacketLength);

		/* Check the consistency of the frame */
		// if rxStatus’s bit_15 is 0, or
		// if rxStatus’s bit_0, bit_1, bit_2, bit_4, bit_10, bit_11, bit_12, bit_13 are 1,
		// received an error frame, goto step 8,
		// else received a good frame, goto step 9.
		// if rxPacketLength <= 0, goto step 8;
		// else goto step 9;
		// CHECKSUM_VALID_FRAME_MASK = 0011 1100 0001 0111 = 0x3C17 (bits 13,12,11,10,4,2,1,0)
		if ((!(rxStatus & VALID_FRAME_MASK)) || (rxStatus & CHECKSUM_VALID_FRAME_MASK))
		{
			/* Issue the Release error frame command */
			ksz8851_ReleaseIncosistentFrame();
			/* continue to next frame */
			continue;
		}
		else
		{
//			dmc_puts(TERMINAL_MAGENTA);
//			dmc_puts("rxFrameCount: ");
//			dmc_putintcr(rxFrameCount);
//			dmc_puts(TERMINAL_DEFAULT);

			// Step 7
			// Read received frame byte size from RXFHBCR to get this received frame length
			// (4-byte CRC, and extra 2-byte due to ‘IP Header Two-Byte Offset Enable’are included),
			// and store into rxPacketLength variable.
			/* Read the byte size of the received frame */
			/* Get current frame byte size */
			/* Frame length includes 4 byte CRC and extra 2 byte IP Header */
			rxPacketLength = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT) & RX_BYTE_CNT_MASK;	/* RXFHBCR */
//			dmc_puts("rxPacketLength: ");
//			dmc_putintcr(rxPacketLength);

			// Step 17
			// Read frame data to system memory pointer by pRxData from the QMU RXQ in BYTE until
			// finished the full packet length (rxPacketLength) in DWORD alignment ‘lengthInByte.
			/* Round to DWORD boundary
			 * For example, the driver has to read up to 68 bytes if received frame size is 65 bytes. */
			// Jack: read a bit more because of missing bytes
			// Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
			bytesToRead = 4 * ((rxPacketLength + 3) >> 2);
//			dmc_puts("bytesToRead: ");
//			dmc_putintcr(bytesToRead);


			if ((bytesToRead > pRXLength) || (rxPacketLength <= 4))
			{
				/* Issue the Release error frame command */
				ksz8851_ReleaseIncosistentFrame();
				/* continue to next frame */
				continue;
			}

			// Step 9
			// Reset QMU RXQ frame pointer to zero with auto increment.
//			ksz8851_reg_write(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);
			/* Set QMU RXQ frame pointer to start of packet data. Note
			 * that we skip the status word and byte count since we
			 * already know this from RXFHSR and RXFHBCR.
			 */
			// This pointer value must reset to 0x000 before each DMA operation from
			// the host CPU to read RXQ frame buffer.
			// When this bit is set, the RXQ Address register increments automatically
			// on accesses to the data register. The increment is by one for every byte
			// access, by two for every word access, and by four for every double word
			// access.
			// When this bit is reset, the RX frame data pointer is manually controlled
			// by user to access the RX frame location.
			ksz8851_reg_write(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC | 0x0004);	/* RXFDPR */
//			ksz8851_reg_write(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC | 0x0000);	/* RXFDPR */

			// Step 10
			// Start QMU DMA transfer operation to read frame data from the RXQ to host CPU.
			// When this bit is written as 1, the KSZ8851SNL allows a DMA operation
			// from the host CPU to access either read RXQ frame buffer or write TXQ
			// frame buffer with SPI command operation for RXQ/TXQ FIFO read/
			// write. All registers access are disabled except this register during this
			// DMA operation.
			// This bit must be set to 0 when DMA operation is finished in order to
			// access the rest of registers.
			/* Set bit 3 of RXQCR to start QMU DMA transfer operation */
			/* Must not access other registers once starting QMU DMA transfer operation */
//			data = ksz8851_reg_read(REG_RXQ_CMD);
//			data |= RXQ_START;
//			ksz8851_reg_write(REG_RXQ_CMD, data);
			ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);	/* RXQCR */
//			ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START | RXQ_AUTO_DEQUEUE);

//			/* Read the whole ethernet frame */
//			ksz8851_fifo_read(pRXData, bytesToRead);

			// Step 13
			// Must read out dummy 4-byte.
//			dmc_puts("Step 13\n");
//			uint8_t Dummy[4];
//			(void) HAL_SPI_Receive(&SpiHandle, (uint8_t*) Dummy, 4, 2000);

//			// Step 14
//			// Read out 2-byte ‘Status Word’ of frame header from the QMU RXQ.
//			dmc_puts("Step 14\n");
//			uint8_t StatusWord[2];
//			(void) HAL_SPI_Receive(&SpiHandle, (uint8_t*) StatusWord, 2, 2000);
//
//			// Step 15
//			// Read out 2-byte ‘Byte Count’ of frame header from the QMU RXQ.
//			dmc_puts("Step 15\n");
//			uint8_t ByteCount[2];
//			(void) HAL_SPI_Receive(&SpiHandle, (uint8_t*) ByteCount, 2, 2000);
//			dmc_puts("ByteCount: ");
//			dmc_puthex2(ByteCount[1]);
//			dmc_puthex2cr(ByteCount[0]);

			// Step 18
			// Read 1-byte of frame data to system memory pointer by pRxData from
			// the QMU RXQ. Increase pRxData pointer by 1.
			/* Perform blocking SPI transfer. */
//			dmc_puts("Perform blocking SPI transfer.\n");
//			(void) HAL_SPI_Receive(&SpiHandle, (uint8_t*) pRXData, bytesToRead, 2000);
			// Number of SCLK for register access as following:
			// Write frame to TXQ: CMD(8bits) + Frame Header(32bits) + Frame Data(8bits*N)
			// Read frame from RXQ: CMD(8bits) + Dummy(32bits) + Frame Status(32bits) + Frame Data(8bits*N)
			/* Write RXQ command phase */
			memset(outbuf, 0, bytesToRead);
			outbuf[0] = FIFO_READ;
			/* Dummy 4 byte, status word 2 byte, byte count 2 byte, IP header 2 byte */
			/* Buffer allocation = number of bytes to receive */
			bytesToRead += 8;

//dmc_putintcr(bytesToRead);
//			memset(pRXData, 0x55, bytesToRead);

			(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) outbuf, (uint8_t *) pRXData, bytesToRead, 2000);

//			(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) outbuf, (uint8_t *) pRXData, bytesToRead);

//			(void) HAL_SPI_Receive_DMA(&SpiHandle, (uint8_t*) pRXData, bytesToRead);
//			dmc_puts("Done.\n");

			// Step 21
			// Stop QMU DMA transfer operation.
//			data = ksz8851_reg_read(REG_RXQ_CMD);
//			data &= ~RXQ_START;
//			ksz8851_reg_write(REG_RXQ_CMD, data);
			ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);

			// Step 22
			// Pass this received frame to the upper layer protocol stack.
			// Jack: may print it if it makes sense...
			// 192.168.25.xxx
//			uint8_t found = 0;
////			for (uint8_t i = 0; i < 30; i++)
////			{
////				if ((pRXData[i] == 0xC0) && (pRXData[i+1] == 0xA8) && (pRXData[i+2] == 0xA8))
////				{
////					found = 1;
////				}
////			}
//
//			uint16_t offset1 = 0;
//			uint16_t offset2 = 0;
//
//			for (uint16_t i = 30; i < 64; i++)
//			{
//				if ((pRXData[i] == 0xC0) && (pRXData[i+1] == 0xA8) && (pRXData[i+2] == 0x19))
//				{
//					if (offset1 == 0)
//					{
//						offset1 = i;
//					}
//					else
//					{
//						offset2 = i;
//						found = 1;
//						break;
//					}
//				}
//			}
//
////			found = 1;
//			if (found)
//			{
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(' ');
//				dmc_putint(pRXData[offset1++]);
//				dmc_putc('.');
//				dmc_putint(pRXData[offset1++]);
//				dmc_putc('.');
//				dmc_putint(pRXData[offset1++]);
//				dmc_putc('.');
//				uint8_t ipa4 = pRXData[offset1++];
//				dmc_putint(ipa4);
//				dmc_putc('\n');
//
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(':');
////				dmc_puthex2(pRXData[offset++]);
////				dmc_putc(' ');
//				dmc_putint(pRXData[offset2++]);
//				dmc_putc('.');
//				dmc_putint(pRXData[offset2++]);
//				dmc_putc('.');
//				dmc_putint(pRXData[offset2++]);
//				dmc_putc('.');
//				uint8_t ipb4 = pRXData[offset2++];
//				dmc_putint(ipb4);
//				dmc_putc('\n');
//
//				uint8_t ip4 = 238;
//				if ((ipa4 == ip4) || (ipb4 == ip4))
//				{
//					dmc_puts(TERMINAL_YELLOW);
//				}
//				else
//				{
//					dmc_puts(TERMINAL_CYAN);
//				}
//				for (uint16_t i = 0; i < bytesToRead; i++)
//				{
//					dmc_puthex2(pRXData[i]);
//					dmc_putc(' ');
//				}
//				dmc_putc('\n');
//				uint8_t pcr = 0;
//				for (uint16_t i = 0; i < bytesToRead; i++)
//				{
//					if ((pRXData[i] >= 0x20) && (pRXData[i] < 0x7F))
//					{
//						dmc_putc(pRXData[i]);
//						pcr = 1;
//					}
//				}
//				if (pcr)
//				{
//					dmc_putc('\n');
//				}
//				dmc_puts(TERMINAL_DEFAULT);
//			}
			break;
		}
	}


//	data = ksz8851_reg_read(REG_INT_ENABLE);	/* ISR */
//	dmc_puts("REG_INT_ENABLE ");
//	dmc_puthex4cr(data);

	// Step 24
	/* Enable interrupts */
	ksz8851_reg_write(REG_INT_ENABLE,			/* IER */
			KSZ8851SNL_INT_RX_DONE |		/** Enable transmit done interrupt */
			KSZ8851SNL_INT_RX_STOPPED |		/** Enable receive process stopped interrupt */
			KSZ8851SNL_INT_TX_STOPPED |		/** Enable transmit process stopped interrupt */
			KSZ8851SNL_INT_LINK_CHANGE |	/** Enable link change interrupt */
			KSZ8851SNL_INT_RX_SPI_ERROR);	/** Enable receive SPI bus error interrupt */

	// After enabling interrupts, an interrupt is generated immediately

//	data = ksz8851_reg_read(REG_INT_ENABLE);	/* ISR */
//	dmc_puts("REG_INT_ENABLE ");
//	dmc_puthex4cr(data);

//	return bytesToRead;

	/* Return frame length without CRC */
	rxPacketLength -= 4;
	rxPacketLength -= 2;	// Not sure why
//	rxPacketLength -= 11;	// Strip 11 characters
//	frameLength = rxPacketLength - 2;
//	frameLength = rxPacketLength + 2;
	return rxPacketLength;




//	/* Read the frame count and threshold register */
//	rxftr = ksz8851_reg_read(REG_RX_FRAME_CNT_THRES);	/* RXFCTFC */
//	/* Extract the actual number of frames from RX_FRAME_THRES_REG*/
//	rxFrameCount = rxftr >> MSB_POS;
//
//	while (rxFrameCount > 0)
//	{
//		rxFrameCount--;
//		/* Read the received frame status */
//		rxStatus = ksz8851_reg_read(REG_RX_FHR_STATUS);
//
//		/* Check the consistency of the frame */
//		if ((!(rxStatus & VALID_FRAME_MASK)) || (rxStatus & CHECKSUM_VALID_FRAME_MASK))
//		{
//			/* Issue the Release error frame command */
//			ksz8851_ReleaseIncosistentFrame();
//			/* continue to next frame */
//			continue;
//		}
//		else
//		{
//			/* Read the byte size of the received frame */
//			rxPacketLength = ksz8851_reg_read(RXFHBCR) & RX_BYTE_CNT_MASK;
//
//			/* round to dword boundary */
//			bytesToRead = 4 * ((rxPacketLength + 3) >> 2);
////			LWIP_DEBUGF(NETIF_DEBUG,
////					("KSZ8851SNL_Receive: rxPacketLength=%u, bytesToRead=%u \n", rxPacketLength, bytesToRead));
//			if ((bytesToRead > pRXLength) || (rxPacketLength <= 4))
//			{
//				/* Issue the Release error frame command */
//				ksz8851_ReleaseIncosistentFrame();
//				/* continue to next frame */
//				continue;
//			}
//
//			/* Set QMU RXQ frame pointer to start of packet data. Note
//			 * that we skip the status word and byte count since we
//			 * already know this from RXFHSR and RXFHBCR.
//			 */
//			ksz8851_reg_write(REG_RX_ADDR_PTR, 0x0004 | FD_PTR_AUTO_INC);
//
//			/* Start QMU DMA transfer */
//			data = ksz8851_reg_read(REG_RXQ_CMD);
//			data |= RXQ_START_DMA;
//			ksz8851_reg_write(REG_RXQ_CMD, data);
//
//			/* Read the whole ethernet frame */
//			ksz8851_fifo_read(pRXData, bytesToRead);
//
//			/* Stop QMU DMA transfer */
//			data = ksz8851_reg_read(REG_RXQ_CMD);
//			data &= ~RXQ_START_DMA;
//			ksz8851_reg_write(REG_RXQ_CMD, data);
//
//			/* Enable interrupts */
//			data = INT_MASK_EXAMPLE;
//			ksz8851_reg_write(REG_INT_ENABLE, data);
//
//			/* Return frame length without CRC */
//			frameLength = rxPacketLength - 4;
//			return frameLength;
//		}
//		/* Enable interrupts */
//		data = INT_MASK_EXAMPLE;
//		ksz8851_reg_write(REG_INT_ENABLE, data);
//	}
	return 0;
}

/***************************************************************************//**
 * @brief Get the MAC address of the current board.
 * @note  Support method used for minimizing the code size.
 * @param[out] macAddress
 *     data buffer to store the macAddress
 *****************************************************************************/
void ksz8851_GetMacAddress(uint8_t *macAddress)
{
	/* TODO:  Get MAC based on actual MAC and not on the CMU unique ID. */

//	int i;
//	EFM_ASSERT(macAddress != NULL);
	/* set the first 3 bytes given by the EM MAC Address space */
//	macAddress[0] = HIGH_QMU_MAC_H;
//	macAddress[1] = HIGH_QMU_MAC_L;
//	macAddress[2] = MID_QMU_MAC_H;
	/* set the next 3 bytes given by the CMU unique ID */

//	for (i = 0; i < 3; i++)
//	{
//		macAddress[5 - i] = (DEVINFO->UNIQUEL & (BYTE_MASK << i * BYTE_SIZE)) >> i * BYTE_SIZE;
//	}
}

uint16_t ksz8851_PHYStatusGet(void)
{
	return ksz8851_reg_read(P1SR);
}

void ksz8851_SetDigitalLoopbackMode(void)
{
	uint16_t data;
	/* Reset PHY. */
	data = PHY_RESET;
	ksz8851_reg_write(REG_PHY_RESET, data);
	/* Disable Auto-negotiation.1. Reset PHY. */
	/* Set Speed to either 100Base-TX or 10Base-T. */
	/* Set Duplex to full-duplex. */
	/* Set PHY register 0.14 to Â‘1Â’ to enable Local Loop-back. */
	data = DIGITAL_LOOPBACK | FORCE_FULL_DUPLEX | FORCE_100;
	data &= ~AUTO_NEG;
	ksz8851_reg_write(REG_PHY_CNTL, data);

//  KSZ8851SNL_ExceptionHandler(INFO, "Loopback mode initiated");
}

/**************************************************************************//**
 * @brief enables the chip interrupts
 *****************************************************************************/
void ksz8851_EnableInterrupts(void)
{
	/* Enable interrupts */
  ksz8851_reg_write(INT_ENABLE_REG, INT_MASK_EXAMPLE);
}

// https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
/***************************************************************************//**
 * @brief Checks for any interrrupts and if found, clears their status
 *        and prepair for interrupt handler routines
 * @note  Programmer needs to re-enable the KSZ8851SNL interrupts after
 *        calling this function
 *
 * @return
 *     found interrupts
 *
 *****************************************************************************/
uint16_t ksz8851_CheckIrqStat(void)
{
	uint16_t data, ISR_stat, found_INT;
	found_INT = 0;
	ISR_stat = ksz8851_reg_read(REG_INT_STATUS);

	/* Disable interrupts on KSZ8851SNL */
	data = NO_INT;
	ksz8851_reg_write(REG_INT_ENABLE, data);

	/* Resolve the RX completion interrupt */
	if (ISR_stat & INT_RX_DONE)
	{
		dmc_puts("INT_RX_DONE\n");
		/* Clear RX Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_DONE;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_RX_DONE;
	}
	/* Resolve the Link change interrupt */
	if (ISR_stat & INT_LINK_CHANGE)
	{
//		dmc_puts("INT_LINK_CHANGE\n");
		/* Clear Link change Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_LINK_CHANGE;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_LINK_CHANGE;
	}
	/* Resolve the RX overrun interrupt */
	if (ISR_stat & INT_RX_OVERRUN)
	{
//		dmc_puts("INT_RX_OVERRUN\n");
		/* Clear RX overrun Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_OVERRUN;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_RX_OVERRUN;
	}
	/* Resolve the TX stopped interrupt */
	if (ISR_stat & INT_TX_STOPPED)
	{
//		dmc_puts("INT_TX_STOPPED\n");
		/* Clear TX stopped Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_TX_STOPPED;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_TX_STOPPED;
	}
	/* Resolve the RX stopped interrupt */
	if (ISR_stat & INT_RX_STOPPED)
	{
//		dmc_puts("INT_RX_STOPPED\n");
		/* Clear RX stopped Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_STOPPED;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_RX_STOPPED;
	}
	/* Resolve the RX of a WakeOnLan frame interrupt */
	if (ISR_stat & INT_RX_WOL_FRAME)
	{
//		dmc_puts("INT_RX_WOL_FRAME\n");
		/* Clear RX of a WakeOnLan Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_WOL_FRAME;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_RX_WOL_FRAME;
	}
	/* Resolve the RX of a magic frame interrupt */
	if (ISR_stat & INT_MAGIC)
	{
//		dmc_puts("INT_MAGIC\n");
		/* Clear RX of a magic frame Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_MAGIC;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_MAGIC;
	}
	/* Resolve the RX of a LINKUP interrupt */
	if (ISR_stat & INT_LINKUP)
	{
//		dmc_puts("INT_LINKUP\n");
		/* Clear RX of a LINKUP Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_LINKUP;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_LINKUP;
	}
	/* Resolve the RX of a Energy interrupt */
	if (ISR_stat & INT_ENERGY)
	{
//		dmc_puts("INT_ENERGY\n");
		/* Clear RX of a Energy Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_ENERGY;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_ENERGY;
	}
	/* Resolve the SPI Error interrupt */
	if (ISR_stat & INT_SPI_ERROR)
	{
//		dmc_puts("INT_SPI_ERROR\n");
		/* Clear SPI Error Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_SPI_ERROR;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_SPI_ERROR;
	}
	/* Resolve the TX space interrupt */
	if (ISR_stat & INT_TX_SPACE)
	{
//		dmc_puts("INT_TX_SPACE\n");
		/* Clear TX space Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_TX_SPACE;
		ksz8851_reg_write(REG_INT_STATUS, data);
		found_INT |= INT_TX_SPACE;
	}
	return found_INT;
}

// https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
/***************************************************************************//**
 * @brief Returns the size of the currently received frame
 *
 * @return
 *     the printed string
 *
 *****************************************************************************/
uint16_t ksz8851_CurrFrameSize(void)
{
	uint16_t data;

	/* Read the byte size of the received frame */
	data = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT);

	data &= RX_BYTE_CNT_MASK;

	return data;
}



/***************************************************************************//**
 * @brief Returns the difference in bytes to be DWORD aligned
 *
 * @param val
 *     value that needs to be aligned
 * @return
 *     the number of bytes needed to be added so that the value is aligned
 *
 *****************************************************************************/
uint8_t ksz8851_DwordAllignDiff(uint8_t val)
{
	if (val % 4 == 0)
	{
		return 0;
	}
	else
	{
		return val % 4;
	}
}

void ksz8851_irq(void)
{
	uint16_t isr = ksz8851_reg_read(REG_INT_STATUS);

	if (isr == 0)
	{
		return;
	}
//	if (isr_old == isr)
//	{
//		return;
//	}
//	dmc_puts("ksz8851_isr: ");
//	dmc_puthex4(isr);
//	dmc_puts("\n");

	if (isr & IRQ_LCI)
	{
		// LCIS Link Change Interrupt Status
		// When this bit is set, it indicates that the link status has changed from link
		// up to link down, or link down to link up.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		dmc_puts("LCI ");
		if (isr & IRQ_LDI)
		{
			// LDIS Linkup Detect Interrupt Status
			// When this bit is set, it indicates that wake-up from linkup detect status
			// has occurred. Write “0010” to PMECR[5:2] to clear this bit.
			dmc_puts("LDI (Link up)\n");
	    	uint16_t PHYStatusGet = ksz8851_PHYStatusGet();
//	    	if (PHYStatusGet & (1 < 15))
//	    		dmc_puts("HP Auto MDI-X mode.\n");
	    	if (PHYStatusGet & (1 < 10))
	    		dmc_puts("100 Mbps\n");
	    	if (PHYStatusGet & (1 < 9))
	    		dmc_puts("Full Duplex\n");
	    	dmc_puthex4cr(PHYStatusGet);
		}
		else
		{
		  ksz8851_reg_write(REG_INT_STATUS, isr);
			dmc_puts("(Link down)\n");
		}
//		ksz8851_reg_write(REG_INT_STATUS, isr);
	}
	else
	if (isr & IRQ_LDI)
	{
		// LDIS Linkup Detect Interrupt Status
		// When this bit is set, it indicates that wake-up from linkup detect status
		// has occurred. Write “0010” to PMECR[5:2] to clear this bit.
//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("LDI (Link up)\n");
	}
	if (isr & IRQ_TXI)
	{
		// TXIS Transmit Interrupt Status
		// When this bit is set, it indicates that the TXQ MAC has transmitted at
		// least a frame on the MAC interface and the QMU TXQ is ready for new
		// frames from the host.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("TXI\n");
//		led_net_tx();
	}
	if (isr & IRQ_RXI)
	{
		// When this bit is set, it indicates that the QMU RXQ has received at least
		// a frame from the MAC interface and the frame is ready for the host CPU
		// to process.
		dmc_puts("RXI\n");
    	dmc_puts("CurrFrameSize: ");
		uint16_t CurrFrameSize = ksz8851_CurrFrameSize();
		dmc_putintcr(CurrFrameSize);

		ksz8851snl_reset_rx();

//		led_net_rx();
	}
	if (isr & IRQ_TXPSI)
	{
		// TXPSIS Transmit Process Stopped Interrupt Status
		// When this bit is set, it indicates that the Transmit Process has stopped.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("IRQ_TXPSI\n");
	}
	if (isr & IRQ_RXPSI)
	{
		// RXPSIS Receive Process Stopped Interrupt Status
		// When this bit is set, it indicates that the Receive Process has stopped.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		dmc_puts("IRQ_RXPSI\n");
	}
	if (isr & IRQ_SPIBEI)
	{
		// SPIBEIS SPI Bus Error Interrupt Status
		// When this bit is set, it indicates that SPI bus error status has occurred.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("SPIBEI\n");
	}

//	ksz8851_reg_write(REG_INT_STATUS, isr);

	// Write 1 (0xFFFF) to clear all interrupt status bits after an interrupt
	// occurred in Interrupt Status Register.

	ksz8851snl_reset_rx();

	ksz8851_reg_write(REG_INT_STATUS, 0xFFFF);
	ksz8851_reg_write(REG_POWER_CNTL, 0x003C);

//	  uint8_t MACaddress[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00 };
//	  ksz8851_init(MACaddress);
	isr_old = isr;
}

void ksz8851_clr_irq(void)
{
	uint16_t isr = ksz8851_reg_read(REG_INT_STATUS);

	ksz8851_reg_write(REG_INT_STATUS, isr);
//	ksz8851_reg_write(REG_INT_STATUS, 0xFFFF);
//	ksz8851_reg_write(REG_POWER_CNTL, 0x003C);
}

void ksz8851_ClearRxInterrupt(void)
{
	// Clear RXIS Receive Interrupt Status
	ksz8851_reg_setbits(INT_STATUS_REG, INT_RX);
}

void ksz8851_EnableRxInterrupt(void)
{
	// Clear RXIS Receive Interrupt Status
	ksz8851_reg_setbits(INT_STATUS_REG, INT_RX);
	// Set RXIE Receive Interrupt Enable
	ksz8851_reg_setbits(INT_ENABLE_REG, INT_RX);
}

void ksz8851_DisableRxInterrupt(void)
{
	// Clr RXIE Receive Interrupt Enable
	ksz8851_reg_clrbits(INT_ENABLE_REG, INT_RX);
}

uint16_t ksz8851_ReadRxFrameCount(void)
{
	uint16_t rxfctr;
	// Bit 15-8
	// RXFC RX Frame Count
	// To indicate the total received frames in RXQ frame buffer when receive
	// interrupt (bit13=1 in ISR) occurred and write “1” to clear this bit 13 in
	// ISR. The host CPU can start to read the updated receive frame header
	// information in RXFHSR/RXFHBCR registers after read this RX frame
	// count register.
	rxfctr = ksz8851_reg_read(KS_RXFCTR);
	rxfctr = rxfctr >> 8;
	return rxfctr;
}

uint16_t ksz8851_ReadRxByteCount(void)
{
	uint16_t rxfhbcr;
	rxfhbcr = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT);
	rxfhbcr &= 0x0fff;
	return rxfhbcr;
}

uint16_t ksz8851_ReadRxHeaderStatus(void)
{
	uint16_t rxfhsr;
	rxfhsr = ksz8851_reg_read(REG_RX_FHR_STATUS);
	rxfhsr &= 0xBCFF;	// Mask out reserved bits 1011 1100 1111 1111
	// Bit 15 RXFV Receive Frame Valid
	// Bit 14 Reserved
	// Bit 13 RXICMPFCS Receive ICMP Frame Checksum Status
	// Bit 12 RXIPFCS Receive IP Frame Checksum Status
	// Bit 11 RXTCPFCS Receive TCP Frame Checksum Status
	// Bit 10 RXUDPFCS Receive UDP Frame Checksum Status
	// Bit  9 Reserved
	// Bit  8 Reserved
	// Bit  7 RXBF Receive Broadcast Frame
	// Bit  6 RXMF Receive Multicast Frame
	// Bit  5 RXUF Receive Unicast Frame
	// Bit  4 RXMR Receive MII Error
	// Bit  3 RXFT Receive Frame Type
	// Bit  2 RXFTL Receive Frame Too Long
	// Bit  1 RXRF Receive Runt Frame
	// Bit  0 RXCE Receive CRC Error
	return rxfhsr;
}

void ksz8851_ClearRxFramePointer(void)
{
	// The value of this register determines the address to be accessed within the RXQ frame buffer. When the Auto Increment
	// is set, it will automatically increment the RXQ Pointer on read accesses to the data register.
	// The counter is incremented is by one for every byte access, by two for every word access, and by four for every double
	// word access.
	uint16_t rxfdpr;
	rxfdpr = ksz8851_reg_read(RX_FD_PTR_REG);
	rxfdpr &= ADDR_PTR_AUTO_INC;	// Save ADDR_PTR_AUTO_INC bit, clear 0x07ff = pointer mask
	ksz8851_reg_write(RX_FD_PTR_REG, rxfdpr);
}

void ksz8851_SetRxFramePointerAutoIncrement(void)
{
	uint16_t rxfdpr;
	rxfdpr = ksz8851_reg_read(RX_FD_PTR_REG);
	rxfdpr |= ADDR_PTR_AUTO_INC;
	ksz8851_reg_write(RX_FD_PTR_REG, rxfdpr);
}

void ksz8851_ClrRxFramePointerAutoIncrement(void)
{
	uint16_t rxfdpr;
	rxfdpr = ksz8851_reg_read(RX_FD_PTR_REG);
	rxfdpr &= ~ADDR_PTR_AUTO_INC;
	ksz8851_reg_write(RX_FD_PTR_REG, rxfdpr);
}

void ksz8851_EnableRXQReadAccess(void)
{
	// Clear RXIS Receive Interrupt Status
	ksz8851_reg_setbits(RXQ_CMD_REG, RXQ_START);
}

void ksz8851_DisableRXQReadAccess(void)
{
	// Clear RXIS Receive Interrupt Status
	ksz8851_reg_clrbits(RXQ_CMD_REG, RXQ_START);
}

uint16_t ksz8851_ReadRxInterruptSource(void)
{
	uint16_t data;

	/* Read Rx interrupt source */
	data = ksz8851_reg_read(RXQ_CMD_REG);
	// Mask bits 10-12
	// Bit 10
	// RXFCTS RX Frame Count Threshold Status
	// When this bit is set, it indicates that RX interrupt is due to the number of
	// received frames in RXQ buffer exceeds the threshold set in
	// RX Frame Count Threshold Register (0x9C, RXFCT).
	// This bit will be updated when write 1 to bit 13 in ISR register.
	// Bit 11
	// RXDBCTS RX Data Byte Count Threshold Status
	// When this bit is set, it indicates that RX interrupt is due to the number of
	// received bytes in RXQ buffer exceeds the threshold set in
	// RX Data Byte Count Threshold Register (0x8E, RXDBCT).
	// This bit will be updated when write 1 to bit 13 in ISR register.
	// Bit 12
	// RXDTTS RX Duration Timer Threshold Status
	// When this bit is set, it indicates that RX interrupt is due to the time start
	// at first received frame in RXQ buffer exceeds the threshold set in
	// RX Duration Timer Threshold Register (0x8C, RXDTT).
	// This bit will be updated when write 1 to bit 13 in ISR register.
	data &= 0x1c00;
//	if (data & 0x0400)
//	{
//		dmc_puts("RXFCTS RX Frame Count Threshold Status\n");
//	}
//	if (data & 0x0800)
//	{
//		dmc_puts("RXDBCTS RX Data Byte Count Threshold Status\n");
//	}
//	if (data & 0x1000)
//	{
//		dmc_puts("RXDTTS RX Duration Timer Threshold Status\n");
//	}

	return data;
}

// Required
uint8_t ksz8851_read_id(void)
{
	uint16_t dev_id = ksz8851_reg_read(REG_CHIP_ID);
	dmc_puts("dev_id: ");
	dmc_puthex4(dev_id);
}


// https://www.oryx-embedded.com/doc/ksz8851_8c_source.html
// https://www.oryx-embedded.com/doc/ksz8851_8h_source.html
// https://www.oryx-embedded.com/doc/ethernet_8h_source.html

///**
// * @brief KSZ8851 interrupt service routine
// * @param[in] interface Underlying network interface
// * @return TRUE if a higher priority task must be woken. Else FALSE is returned
// **/
//
//uint8_t ksz8851IrqHandler(NetInterface *interface)
//{
//	uint8_t flag;
//   uint16_t ier;
//   uint16_t isr;
//
//   //This flag will be set if a higher priority task must be woken
//   flag = FALSE;
//
//   //Save IER register value
//   ier = ksz8851_reg_read(INT_ENABLE_REG);
//   //Disable interrupts to release the interrupt line
//   ksz8851_reg_write(INT_ENABLE_REG, 0);
//
//   //Read interrupt status register
//   isr = ksz8851_reg_read(INT_STATUS_REG);
//
//   //Link status change?
//   if(isr & INT_LINK_CHANGE)
//   {
//      //Disable LCIE interrupt
//      ier &= ~INT_LINK_CHANGE;
//
////      //Set event flag
////      interface->nicEvent = TRUE;
////      //Notify the TCP/IP stack of the event
////      flag |= osSetEventFromIsr(&netEvent);
//   }
//
//   //Packet transmission complete?
//   if(isr & INT_TX_DONE)
//   {
//      //Clear interrupt flag
//      ksz8851_reg_write(INT_STATUS_REG, INT_TX_DONE);
//
////      //Notify the TCP/IP stack that the transmitter is ready to send
////      flag |= osSetEventFromIsr(&interface->nicTxEvent);
//   }
//
//   //Packet received?
//   if(isr & INT_RX_DONE)
//   {
//      //Disable RXIE interrupt
//      ier &= ~INT_RX_DONE;
//
////      //Set event flag
////      interface->nicEvent = TRUE;
////      //Notify the TCP/IP stack of the event
////      flag |= osSetEventFromIsr(&netEvent);
//   }
//
//   //Re-enable interrupts once the interrupt has been serviced
//   ksz8851_reg_write(INT_ENABLE_REG, ier);
//
//   //A higher priority task must be woken?
//   return flag;
//}
//
//
///**
// * @brief KSZ8851 event handler
// * @param[in] interface Underlying network interface
// **/
//
//void ksz8851EventHandler(NetInterface *interface)
//{
//   uint16_t status;
//   uint8_t frameCount;
//
//   //Read interrupt status register
//   status = ksz8851_reg_read(INT_STATUS_REG);
//
//   //Check whether the link status has changed?
//   if(status & INT_LINK_CHANGE)
//   {
//      //Clear interrupt flag
//      ksz8851_reg_write(INT_STATUS_REG, INT_LINK_CHANGE);
//      //Read PHY status register
//      status = ksz8851_reg_read(REG_PORT_STATUS);
//
//      //Check link state
//      if(status & PORT_STATUS_LINK_GOOD)
//      {
//         //Get current speed
////         if(status & PORT_STAT_SPEED_100MBIT)
////            interface->linkSpeed = NIC_LINK_SPEED_100MBPS;
////         else
////            interface->linkSpeed = NIC_LINK_SPEED_10MBPS;
//
//         //Determine the new duplex mode
////         if(status & PORT_STAT_FULL_DUPLEX)
////            interface->duplexMode = NIC_FULL_DUPLEX_MODE;
////         else
////            interface->duplexMode = NIC_HALF_DUPLEX_MODE;
//
//         //Link is up
////         interface->linkState = TRUE;
//      }
//      else
//      {
//         //Link is down
////         interface->linkState = FALSE;
//      }
//
//      //Process link state change event
//      nicNotifyLinkChange(interface);
//   }
//
//   //Check whether a packet has been received?
//   if(status & INT_RX_DONE)
//   {
//      //Clear interrupt flag
//      ksz8851_reg_write(INT_STATUS_REG, INT_RX_DONE);
//      //Get the total number of frames that are pending in the buffer
//      frameCount = MSB(ksz8851_reg_read(RX_FRAME_THRES_REG));
//
//      //Process all pending packets
//      while(frameCount > 0)
//      {
//         //Read incoming packet
//         ksz8851ReceivePacket(interface);
//         //Decrement frame counter
//         frameCount--;
//      }
//   }
//
//   //Re-enable LCIE and RXIE interrupts
//   ksz8851_reg_setbits(INT_ENABLE_REG, INT_LINK_CHANGE | INT_RX_DONE);
//}
//
//
///**
// * @brief Send a packet
// * @param[in] interface Underlying network interface
// * @param[in] buffer Multi-part buffer containing the data to send
// * @param[in] offset Offset to the first data byte
// * @return Error code
// **/
//
//uint8_t ksz8851SendPacket(NetInterface *interface,
//   const NetBuffer *buffer, size_t offset)
//{
//   size_t n;
//   size_t length;
////   Ksz8851TxHeader header;
////   Ksz8851Context *context;
//
//   //Point to the driver context
////   context = (Ksz8851Context *) interface->nicContext;
//
//   //Retrieve the length of the packet
//   length = netBufferGetLength(buffer) - offset;
//
//   //Check the frame length
////   if(length > ETH_MAX_FRAME_SIZE)
////   {
////      //The transmitter can accept another packet
////      osSetEvent(&interface->nicTxEvent);
////      //Report an error
////      return ERROR_INVALID_LENGTH;
////   }
//
//   //Get the amount of free memory available in the TX FIFO
//   n = ksz8851_reg_read(TX_MEM_INFO_REG) & TX_MEM_AVAILABLE_MASK;
//
//   //Make sure enough memory is available
////   if((length + 8) > n)
////      return ERROR_FAILURE;
//
//   //Copy user data
////   netBufferRead(context->txBuffer, buffer, offset, length);
//
//   //Format control word
////   header.controlWord = TX_CTRL_TXIC | (context->frameId++ & TX_CTRL_TXFID);
//   //Total number of bytes to be transmitted
////   header.byteCount = length;
//
//   //Enable TXQ write access
//   ksz8851_reg_setbits(RXQ_CMD_REG, RXQ_START);
//   //Write TX packet header
////   ksz8851_fifo_write((uint8_t *) &header, sizeof(Ksz8851TxHeader));
//   //Write data
////   ksz8851_fifo_write(context->txBuffer, length);
//   //End TXQ write access
//   ksz8851_reg_clrbits(RXQ_CMD_REG, RXQ_START);
//
//   //Start transmission
//   ksz8851_reg_setbits(TXQ_CMD_REG, TXQ_ENQUEUE);
//
//   //Successful processing
////   return NO_ERROR;
//   return 0;
//}
//
//
///**
// * @brief Receive a packet
// * @param[in] interface Underlying network interface
// * @return Error code
// **/
//
//uint8_t ksz8851ReceivePacket(NetInterface *interface)
//{
//   size_t n;
//   uint16_t status;
////   Ksz8851Context *context;
//
//   //Point to the driver context
////   context = (Ksz8851Context *) interface->nicContext;
//
//   //Read received frame status from RXFHSR
//   status = ksz8851_reg_read(RX_FRH_STAT_REG);
//
//   //Make sure the frame is valid
//   if(status & RX_VALID)
//   {
//      //Check error flags
//      if(!(status & (RX_PHY_ERROR | RX_TOO_LONG | RX_RUNT_ERROR | RX_BAD_CRC)))
//      {
//         //Read received frame byte size from RXFHBCR
//         n = ksz8851_reg_read(RX_FRH_BC_REG) & RX_BYTE_CNT_MASK;
//
//         //Ensure the frame size is acceptable
//         if(n > 0 && n <= ETH_MAX_FRAME_SIZE)
//         {
//            //Reset QMU RXQ frame pointer to zero
//            ksz8851_reg_write(RX_FD_PTR_REG, RXFDPR_RXFPAI);
//            //Enable RXQ read access
//            ksz8851_reg_setbits(RXQ_CMD_REG, RXQ_START);
//            //Read data
////            ksz8851_fifo_read(context->rxBuffer, n);
//            //End RXQ read access
//            ksz8851_reg_clrbits(RXQ_CMD_REG, RXQ_START);
//
//            //Pass the packet to the upper layer
////            nicProcessPacket(interface, context->rxBuffer, n);
//            //Valid packet received
////            return NO_ERROR;
//            return 0;
//         }
//      }
//   }
//
//   //Release the current error frame from RXQ
//   ksz8851_reg_setbits(RXQ_CMD_REG, RXQ_CMD_FREE_PACKET);
//   //Report an error
////   return ERROR_INVALID_PACKET;
//   return 1;
//}
//
//
///**
// * @brief Configure multicast MAC address filtering
// * @param[in] interface Underlying network interface
// * @return Error code
// **/
//
//uint8_t ksz8851SetMulticastFilter(NetInterface *interface)
//{
//	uint8_t i;
//	uint8_t k;
//   uint32_t crc;
//   uint16_t hashTable[4];
////   MacFilterEntry *entry;
//
//   //Debug message
////   TRACE_DEBUG("Updating KSZ8851 hash table...\r\n");
//
//   //Clear hash table
//   memset(hashTable, 0, sizeof(hashTable));
//
//   //The MAC filter table contains the multicast MAC addresses
//   //to accept when receiving an Ethernet frame
////   for(i = 0; i < MAC_MULTICAST_FILTER_SIZE; i++)
////   {
////      //Point to the current entry
////      entry = &interface->macMulticastFilter[i];
////
////      //Valid entry?
////      if(entry->refCount > 0)
////      {
////         //Compute CRC over the current MAC address
////         crc = ksz8851_CalcCrc(&entry->addr, sizeof(MacAddr));
////         //Calculate the corresponding index in the table
////         k = (crc >> 26) & 0x3F;
////         //Update hash table contents
////         hashTable[k / 16] |= (1 << (k % 16));
////      }
////   }
//
//   //Write the hash table to the KSZ8851 controller
//   ksz8851_reg_write(REG_MAC_HASH_0, hashTable[0]);
//   ksz8851_reg_write(REG_MAC_HASH_2, hashTable[1]);
//   ksz8851_reg_write(REG_MAC_HASH_4, hashTable[2]);
//   ksz8851_reg_write(REG_MAC_HASH_6, hashTable[3]);
//
//   //Debug message
////   TRACE_DEBUG("  MAHTR0 = %04" PRIX16 "\r\n", ksz8851_reg_read(REG_MAC_HASH_0));
////   TRACE_DEBUG("  MAHTR1 = %04" PRIX16 "\r\n", ksz8851_reg_read(REG_MAC_HASH_2));
////   TRACE_DEBUG("  MAHTR2 = %04" PRIX16 "\r\n", ksz8851_reg_read(REG_MAC_HASH_4));
////   TRACE_DEBUG("  MAHTR3 = %04" PRIX16 "\r\n", ksz8851_reg_read(REG_MAC_HASH_6));
//
//   //Successful processing
////   return NO_ERROR;
//   return 0;
//}
//
//
//

void ksz8851_ResetIntHasOcurred(void)
{
	isr_ocurred = 0;
//	ksz8851_reg_write(REG_INT_STATUS, 0xFFFF);
//	ksz8851_reg_write(REG_POWER_CNTL, 0x003C);
}

uint8_t ksz8851_IntHasOcurred(void)
{
	return isr_ocurred;
}

uint16_t ksz8851_GetIntRegisterValue(void)
{
	return isr_reg;
}

void ksz8851_IntHandler()
{
//	dmc_puts(TERMINAL_LIGHT_BLUE);
//	dmc_puts("kszint\n");
//	dmc_puts(TERMINAL_DEFAULT);


//	isr_reg = ksz8851_CheckIrqStat();
//	return;
//	ksz8851_irq();

//	isr_reg = ksz8851_reg_read(REG_INT_STATUS);

//	if (isr_reg == 0)
//	{
//		return;
//	}
	isr_ocurred = 1;

////	if (isr_old == isr)
////	{
////		return;
////	}
//	dmc_puts("ksz8851_isr: ");
//	dmc_puthex4(isr_reg);
//	dmc_puts("\n");

//	if (isr & IRQ_LCI)
//	{
//		// LCIS Link Change Interrupt Status
//		// When this bit is set, it indicates that the link status has changed from link
//		// up to link down, or link down to link up.
//		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
//		dmc_puts("LCI ");
//		if (isr & IRQ_LDI)
//		{
//			// LDIS Linkup Detect Interrupt Status
//			// When this bit is set, it indicates that wake-up from linkup detect status
//			// has occurred. Write “0010” to PMECR[5:2] to clear this bit.
//			dmc_puts("LDI (Link up)\n");
//	    	uint16_t PHYStatusGet = ksz8851_PHYStatusGet();
////	    	if (PHYStatusGet & (1 < 15))
////	    		dmc_puts("HP Auto MDI-X mode.\n");
//	    	if (PHYStatusGet & (1 < 10))
//	    		dmc_puts("100 Mbps\n");
//	    	if (PHYStatusGet & (1 < 9))
//	    		dmc_puts("Full Duplex\n");
//	    	dmc_puthex4cr(PHYStatusGet);
//		}
//		else
//		{
//			ksz8851_reg_write(REG_INT_STATUS, isr);
//			dmc_puts("(Link down)\n");
//		}
////		ksz8851_reg_write(REG_INT_STATUS, isr);
//	}
//	else
//	if (isr & IRQ_LDI)
//	{
//		// LDIS Linkup Detect Interrupt Status
//		// When this bit is set, it indicates that wake-up from linkup detect status
//		// has occurred. Write “0010” to PMECR[5:2] to clear this bit.
////		ksz8851_reg_write(REG_INT_STATUS, isr);
//		dmc_puts("LDI (Link up)\n");
//	}
//	if (isr & IRQ_TXI)
//	{
//		// TXIS Transmit Interrupt Status
//		// When this bit is set, it indicates that the TXQ MAC has transmitted at
//		// least a frame on the MAC interface and the QMU TXQ is ready for new
//		// frames from the host.
//		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
////		ksz8851_reg_write(REG_INT_STATUS, isr);
//		dmc_puts("TXI\n");
////		led_net_tx();
//	}
//	if (isr & IRQ_RXI)
//	{
//		// When this bit is set, it indicates that the QMU RXQ has received at least
//		// a frame from the MAC interface and the frame is ready for the host CPU
//		// to process.
//		dmc_puts("RXI\n");
//    	dmc_puts("CurrFrameSize: ");
//		uint16_t CurrFrameSize = ksz8851_CurrFrameSize();
//		dmc_putintcr(CurrFrameSize);
//
//		ksz8851snl_reset_rx();
//
////		led_net_rx();
//	}
//	if (isr & IRQ_TXPSI)
//	{
//		// TXPSIS Transmit Process Stopped Interrupt Status
//		// When this bit is set, it indicates that the Transmit Process has stopped.
//		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
////		ksz8851_reg_write(REG_INT_STATUS, isr);
//		dmc_puts("IRQ_TXPSI\n");
//	}
//	if (isr & IRQ_RXPSI)
//	{
//		// RXPSIS Receive Process Stopped Interrupt Status
//		// When this bit is set, it indicates that the Receive Process has stopped.
//		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
//		dmc_puts("IRQ_RXPSI\n");
//	}
//	if (isr & IRQ_SPIBEI)
//	{
//		// SPIBEIS SPI Bus Error Interrupt Status
//		// When this bit is set, it indicates that SPI bus error status has occurred.
//		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
////		ksz8851_reg_write(REG_INT_STATUS, isr);
//		dmc_puts("SPIBEI\n");
//	}
//
////	ksz8851_reg_write(REG_INT_STATUS, isr);
//
//	// Write 1 (0xFFFF) to clear all interrupt status bits after an interrupt
//	// occurred in Interrupt Status Register.

//	ksz8851snl_reset_rx();

//	uint16_t usValue = ksz8851_reg_read(REG_RX_CTRL1);
//	usValue &= ~( ( uint16_t ) RX_CTRL_ENABLE | RX_CTRL_FLUSH_QUEUE );
//	ksz8851_reg_write(REG_RX_CTRL1, usValue );
////	HAL_Delay(2);
//	ksz8851_reg_write(REG_RX_CTRL1, usValue | RX_CTRL_FLUSH_QUEUE );
////	HAL_Delay(1);
//	ksz8851_reg_write(REG_RX_CTRL1, usValue );
////	HAL_Delay(1);
//	ksz8851_reg_write(REG_RX_CTRL1, usValue | RX_CTRL_ENABLE );
////	HAL_Delay(1);

//	ksz8851_reg_write(REG_INT_STATUS, 0xFFFF);
//	ksz8851_reg_write(REG_POWER_CNTL, 0x003C);
//
////	  uint8_t MACaddress[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00 };
////	  ksz8851_init(MACaddress);
//	isr_old = isr;
//
//	/* Enable INTN flag. */
////	g_intn_flag = 1;
}

void ksz8851_show_isr(void)
{
	if (isr_reg & IRQ_LCI)
	{
		// LCIS Link Change Interrupt Status
		// When this bit is set, it indicates that the link status has changed from link
		// up to link down, or link down to link up.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		dmc_puts("LCI ");
		if (isr_reg & IRQ_LDI)
		{
			// LDIS Linkup Detect Interrupt Status
			// When this bit is set, it indicates that wake-up from linkup detect status
			// has occurred. Write “0010” to PMECR[5:2] to clear this bit.
			dmc_puts("LDI (Link up)\n");
			uint16_t PHYStatusGet = ksz8851_PHYStatusGet();
			//	    	if (PHYStatusGet & (1 < 15))
			//	    		dmc_puts("HP Auto MDI-X mode.\n");
			if (PHYStatusGet & (1 < 10))
				dmc_puts("100 Mbps\n");
			if (PHYStatusGet & (1 < 9))
				dmc_puts("Full Duplex\n");
			dmc_puthex4cr(PHYStatusGet);
		}
		else
		{
		  ksz8851_reg_write(REG_INT_STATUS, isr_reg);
			dmc_puts("(Link down)\n");
		}
		//		ksz8851_reg_write(REG_INT_STATUS, isr);
	}
	else
		if (isr_reg & IRQ_LDI)
		{
			// LDIS Linkup Detect Interrupt Status
			// When this bit is set, it indicates that wake-up from linkup detect status
			// has occurred. Write “0010” to PMECR[5:2] to clear this bit.
			//		ksz8851_reg_write(REG_INT_STATUS, isr);
			dmc_puts("LDI (Link up)\n");
		}
	if (isr_reg & IRQ_TXI)
	{
		// TXIS Transmit Interrupt Status
		// When this bit is set, it indicates that the TXQ MAC has transmitted at
		// least a frame on the MAC interface and the QMU TXQ is ready for new
		// frames from the host.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("TXI\n");
		//		led_net_tx();
	}
	if (isr_reg & IRQ_RXI)
	{
		// When this bit is set, it indicates that the QMU RXQ has received at least
		// a frame from the MAC interface and the frame is ready for the host CPU
		// to process.
		dmc_puts("RXI\n");
		dmc_puts("CurrFrameSize: ");
		uint16_t CurrFrameSize = ksz8851_CurrFrameSize();
		dmc_putintcr(CurrFrameSize);

		ksz8851snl_reset_rx();

		//		led_net_rx();
	}
	if (isr_reg & IRQ_TXPSI)
	{
		// TXPSIS Transmit Process Stopped Interrupt Status
		// When this bit is set, it indicates that the Transmit Process has stopped.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("IRQ_TXPSI\n");
	}
	if (isr_reg & IRQ_RXPSI)
	{
		// RXPSIS Receive Process Stopped Interrupt Status
		// When this bit is set, it indicates that the Receive Process has stopped.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		dmc_puts("IRQ_RXPSI\n");
	}
	if (isr_reg & IRQ_SPIBEI)
	{
		// SPIBEIS SPI Bus Error Interrupt Status
		// When this bit is set, it indicates that SPI bus error status has occurred.
		// This edge-triggered interrupt status is cleared by writing 1 to this bit.
		//		ksz8851_reg_write(REG_INT_STATUS, isr);
		dmc_puts("SPIBEI\n");
	}

}

bool ksz8851_CheckQMUTXQHasSpace(uint16_t length)
{
	uint16_t txmir;
	uint16_t data, reqSize;
	uint8_t outbuf[4];

	// The Ethernet packet data length does not include CRC.
	// Read value from TXMIR to check if QMU TXQ has enough amount of memory for
	// the Ethernet packet data plus 4-byte frame header, plus 4-byte for DWORD alignment.
	// Compare the read value with (txPacketLength+4+4)
	/* Wait for previous frame to finish before setting up a new one */
	while (ksz8851_reg_read(REG_TXQ_CMD) & TXQ_ENQUEUE)
	{
		dmc_puts("wait...\n");
	}

	/* Make sure there is room for
	 *
	 * 4 bytes Control Word
	 * 4 bytes Byte Count
	 * n bytes Packet
	 * 4 bytes CRC (<- NO) 12 -> 8
	 */
	reqSize = length + 8;
	txmir = ksz8851_reg_read(REG_TX_MEM_INFO) & TX_MEM_AVAIL_MASK;
//	LWIP_DEBUGF(NETIF_DEBUG, ("KSZ8851SNL_LongTransmitInit: txmir =%hu  reqSize = %hu \n", txmir, reqSize));

	if (txmir < reqSize)
	{
		/* TXQ is out of memory */
//		LWIP_DEBUGF(NETIF_DEBUG | LWIP_DBG_LEVEL_WARNING,
//				("Not enough TXQ Memory, available=%u required=%u\n", txmir, reqSize));
		return false;
	}
	return true;
}

