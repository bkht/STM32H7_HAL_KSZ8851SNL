/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */

// MAC in ethernetif.c
// IP in lwip.c
/* Includes ------------------------------------------------------------------*/
//#include <KSZ8851SNL_reg.h>
#include <dmc_print.h>
#include <dmc_terminal.h>
#include <KSZ8851SNL.h>
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "lwip.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include <time.h>

#include "app_ethernet.h"
#include "tcp_echoserver.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
struct netif gnetif;
static void Netif_Config(void);

/* USER CODE BEGIN PV */
uint32_t msTick = 0;
uint32_t msTickPrevious = 0;
uint32_t msTickPrevious2 = 0;

//__attribute__((at(0x30040000))) uint8_t pTxData[2048] = { 0, };
//uint8_t pTxData[2048] __attribute__((section(".dma_buffer")));

//uint8_t pRxData[4096] __attribute__((section(".RxArraySection"))); /* Ethernet Rx DMA Descriptors */
//uint8_t pTxData[4096] __attribute__((section(".TxArraySection"))); /* Ethernet Tx DMA Descriptors */

//uint8_t pTXData[4096] = { 0, };
uint8_t pTXData[4096] __attribute__((section(".dma_buffer"))) = { 0, };
//uint8_t pRXData[4096] = { 0, };
uint8_t pRXData[4096] __attribute__((section(".dma_buffer"))) = { 0, };

/* Global Vars */
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void HAL_StartTicks(void);
//uint32_t HAL_GetTicks(void);
uint8_t HAL_GetTicks(uint32_t ms);
void HAL_StartTicks2(void);
//uint32_t HAL_GetTicks2(void);
uint8_t HAL_GetTicks2(uint32_t ms);

volatile uint32_t ping_time = 0;

#define ARP_CACHE_SIZE 16

struct KSZ8851ARP
{
  uint8_t IP[4];
  uint8_t MAC[6];
};

struct KSZ8851ARPCACHE
{
  struct KSZ8851ARP ArpTable[ARP_CACHE_SIZE];
  uint8_t ArpTableLength;
};

// If the data is a static or global variable, it is zero-filled by default, so just declare it
struct KSZ8851ARPCACHE ArpCache = { 0, };

/* Set the tm_t fields for the local time. */
struct tm *gmtime(timep)
  const time_t *timep;
{
  static struct tm tmbuf;
  register struct tm *tp = &tmbuf;
  time_t time = *timep;
  register long day, mins, secs, year, leap;
  day = time / (24L * 60 * 60);
  secs = time % (24L * 60 * 60);
  tp->tm_sec = secs % 60;
  mins = secs / 60;
  tp->tm_hour = mins / 60;
  tp->tm_min = mins % 60;
  tp->tm_wday = (day + 4) % 7;
  year = (((day * 4) + 2) / 1461);
  tp->tm_year = year + 70;
  leap = !(tp->tm_year & 3);
  day -= ((year * 1461) + 1) / 4;
  tp->tm_yday = day;
  day += (day > 58 + leap) ? ((leap) ? 1 : 2) : 0;
  tp->tm_mon = ((day * 12) + 6) / 367;
  tp->tm_mday = day + 1 - ((tp->tm_mon * 367) + 5) / 12;
  tp->tm_isdst = 0;
  return (tp);
}

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
  uint32_t tm;
  uint32_t t1;
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t m;
  int16_t year = 0;
  int16_t month = 0;
  int16_t dow = 0;
  int16_t mday = 0;
  int16_t hour = 0;
  int16_t min = 0;
  int16_t sec = 0;
  uint64_t JD = 0;
  uint64_t JDN = 0;

  // These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

  JD = ((epoch + 43200) / (86400 >> 1)) + (2440587 << 1) + 1;
  JDN = JD >> 1;

  tm = epoch;
  t1 = tm / 60;
  sec = tm - (t1 * 60);
  tm = t1;
  t1 = tm / 60;
  min = tm - (t1 * 60);
  tm = t1;
  t1 = tm / 24;
  hour = tm - (t1 * 24);

  dow = JDN % 7;
  a = JDN + 32044;
  b = ((4 * a) + 3) / 146097;
  c = a - ((146097 * b) / 4);
  d = ((4 * c) + 3) / 1461;
  e = c - ((1461 * d) / 4);
  m = ((5 * e) + 2) / 153;
  mday = e - (((153 * m) + 2) / 5) + 1;
  month = m + 3 - (12 * (m / 10));
  year = (100 * b) + d - 4800 + (m / 10);

  date->Year = year - 2000;
  date->Month = month;
  date->Date = mday;
  date->WeekDay = dow;
  time->Hours = hour;
  time->Minutes = min;
  time->Seconds = sec;
}

uint8_t ARP_Delete(uint8_t *IP)
{
  // Find ARP entry and delete it
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP, 4) == 0)
    {
      memset(ArpCache.ArpTable[i].IP, 0, 4);
      memset(ArpCache.ArpTable[i].MAC, 0, 6);
      return true; // We have deleted it!
    }
  }
  return false;
}

uint8_t ARP_Add(uint8_t *IP, uint8_t *MAC)
{
  // Look if the ARP entry is there already
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP, 4) == 0)
    {
      return false; // We have it already!
    }
  }
  // Find empty spot in the ARP table
  uint8_t IP_zero[] = { 0, 0, 0, 0 };
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP_zero, 4) == 0)
    {
      memcpy(ArpCache.ArpTable[i].IP, IP, 4);
      memcpy(ArpCache.ArpTable[i].MAC, MAC, 6);
    }
  }
  // Count used ARP entries
  ArpCache.ArpTableLength = 0;
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP_zero, 4) != 0)
    {
      ArpCache.ArpTableLength++;
    }
  }
  return true;
}

uint8_t ARP_GetMACfromIP(uint8_t *IP, uint8_t *MAC)
{
  // Find and return ARP entry
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP, 4) == 0)
    {
      memcpy(MAC, ArpCache.ArpTable[i].MAC, 6);
      return true;
    }
  }
  return false;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_USART3_UART_Init();
  dmc_puts("\n------------------------------------------------------------\n");
  dmc_puts("Start 013 KSZ8851SNL Test\n");
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_RNG_Init();
  MX_IWDG1_Init();
  MX_SPI4_Init();
  MX_SPI1_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_RTC_Init();
//  dmc_puts("MX_FATFS_Init\n");
  MX_FATFS_Init();
//  dmc_puts("MX_USB_DEVICE_Init\n");
  MX_USB_DEVICE_Init();

  dmc_puts("MX_LWIP_Init\n");
//  MX_LWIP_Init();

  dmc_puts("Done\n");
  HAL_Delay(2000);

  /* USER CODE BEGIN 2 */
  uint8_t ButtonState = GPIO_PIN_RESET;

  ksz8851_init();

  /* Configure the Network interface */
//    Netif_Config();
  /* TCP echo server Init */
//    tcp_echoserver_init();
  uint8_t IPNTP[] = { 192, 168, 25, 6 };

  uint8_t MACNTPNew[6] = { 0x00, };

  /* USER CODE END 2 */
  uint32_t NTP_Interval = 2000;

  // Packet data pointer
  // It points to the host CPU system memory space contains the complete Ethernet packet data.
  // The Ethernet packet data length does not include CRC.
  uint16_t txPacketLength = 200;

  uint8_t i = 0;

  uint8_t ntpdata[] =
  {
    // Ethernet II Header
    /*  0 */ 0x00, 0x15, 0x5d, 0x4c, 0x85, 0x01,	// Destination MAC Address
    /*  6 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00,	// Source MAC Address
    /* 12 */ 0x08, 0x00,	// Type: IPv4
    // IP Header
    /* 14 */ 0x45,// 0100 = Version 4, 0101 = Header Length: 20 bytes (5)
    /* 15 */ 0x00,		    // 0000 00 = Differential Service Code Point: Default (0),
                          // 00 = Explicit Congestion Notification: Not ECN-Capable Transport (0)
    /* 16 */ 0x00, 0x4c,	// Total Length: 76
    /* 18 */ 0x23, 0xc2,	// Identification: 0x23c2
    /* 20 */ 0x00, 0x00,	// Flags: 0x0000
    /* 22 */ 0x80,		    // Time to Live: 128
    /* 23 */ 0x11,		    // Protocol: UDP (17)
    /* 24 */ 0x00, 0x00,	// Header checksum: 0x0000 [validation disabled]
    /* 26 */ 192, 168, 25, 238,	// Source IP: 192.168.25.238
    /* 30 */ 192, 168, 25, 6,	  // Destination IP: 5.39.184.5
    // UDP (User Datagram Protocol)
    /* 34 */ 0xde, 0x6d,	// Source port: 0xde6d (56941)
    /* 36 */ 0x00, 0x7b,	// Destination port: 123
    /* 38 */ 0x00, 0x38,	// Length: 56
    /* 40 */ 0x00, 0x00,	// Checksum 0xb2ce [correct]
    // NTP Packet
    // Set the first byte's bits to 00,011,011 for li = 0, vn = 3, and mode = 3.
    /* 42 */ 0x1b,// Flags 0x1b = 0001 1011 = 00, 011, 011 = 0, 3, 3
    /* 43 */ 0x05,	// Stratum
    /* 44 */ 0x0a,	// Polling
    /* 45 */ 0xe9,	// Precision 1.0Sec
    /* 46 */ 0x00, 0x00, 0x00, 0x8c,	// Root delay
    /* 50 */ 0x00, 0x00, 0x48, 0x17,	// Root dispersion
    /* 54 */ 0x00, 0x00, 0x00, 0x00,	// Reference ID NULL
    /* 58 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  // 192.168.25.235 (c0.a8.19.eb) (00:80:e1:40:00:1f) F7
  // 192.168.25.236 (c0.a8.19.ec) (00:80:e1:00:00:00) H7
  // 192.168.25.156 (c0.a8.19.9c) (b4:b6:86:8e:57:bf) PC
  // 192.168.25.41  (c0.a8.19.29) (b4:b6:86:8e:57:bf) PC
  // Ping H7 -> F7
  // 192.168.25.236 (c0.a8.19.ec) (00:80:e1:00:00:00) H7 ->
  // 192.168.25.235 (c0.a8.19.eb) (00:80:e1:40:00:1f) F7

  // Ping H7 -> PC
  // 192.168.25.236 (c0.a8.19.ec) (00:80:e1:00:00:00) H7 ->
  // 192.168.25.156 (c0.a8.19.9c) (b4:b6:86:8e:57:bf) PC
  // 192.168.25.41  (c0.a8.19.29) (b4:b6:86:8e:57:bf) PC
  uint8_t pingdata[] =
  {
      // Ethernet II Header
      /*  0 */ 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf,	// Destination MAC Address
      /*  6 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00,	// Source MAC Address
      /* 12 */ 0x08, 0x00,	// Type: IPv4
      // IP Header
      /* 14 */ 0x45,        // 0100 = Version 4, 0101 = Header Length: 20 bytes (5)
      /* 15 */ 0x00,		    // 0000 00 = Differential Service Code Point: Default (0),
                            // 00 = Explicit Congestion Notification: Not ECN-Capable Transport (0)
      /* 16 */ 0x00, 0x3c,	// Total Length: 60
      /* 18 */ 0x68, 0x38,	// Identification: 0x6838
      /* 20 */ 0x00, 0x00,	// Flags: 0x0000
      /* 22 */ 0x80,		    // Time to Live: 128
      /* 23 */ 0x01,		    // Protocol: ICMP (1)
      /* 24 */ 0x1d, 0xb0,	// Header checksum: 0x1db0 [validation disabled]
      /* 26 */ 192, 168, 25, 238,	// Source IP: 192.168.25.238
      /* 30 */ 192, 168, 25, 142,	// Destination IP: 192.168.25.41
      // Ping packet
      /* 34 */ 0x08,        // Type: 8 (Echo (ping) request)
      /* 35 */ 0x00,		    // Code: 0
      /* 36 */ 0xb2, 0xce,	// Checksum 0xb2ce [correct]
      /* 38 */ 0x00, 0x01,	// Identifier (BE): 1 (0x0001), Identifier (LE): 256 (0x0100)
      /* 40 */ 0x02, 0xf6,	// Sequence number: 0x02f6
      /* 42 */ 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,	// Data (32 bytes) abc...
      0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70,
      0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x61,
      0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69
  };
//	// Copy frame, leave 5 bytes space in the head for the QMU header
//	for (uint16_t i = 0; i < sizeof(pingdata); i++)
//	{
//		pTXData[i + 5] = pingdata[i];
//	}
//	txPacketLength = sizeof(pingdata) + 5;

  uint16_t frameCount = 0;
  uint16_t irq = 0;

  uint8_t ping_count = 14;

  HAL_StartTicks();
  HAL_StartTicks2();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_IWDG_Refresh(&hiwdg1);

    if (HAL_GetTicks(NTP_Interval))
    {
//			HAL_StartTicks();
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

      if (ARP_GetMACfromIP(IPNTP, MACNTPNew))
      {
        dmc_puts(TERMINAL_CYAN);
        dmc_puts("ARP Cache: IP ");
        dmc_putint(IPNTP[0]);
        dmc_puts(".");
        dmc_putint(IPNTP[1]);
        dmc_puts(".");
        dmc_putint(IPNTP[2]);
        dmc_puts(".");
        dmc_putint(IPNTP[3]);
        dmc_puts(" has MAC ");
        dmc_puthex2(MACNTPNew[0]);
        dmc_puts(":");
        dmc_puthex(MACNTPNew[1]);
        dmc_puts(":");
        dmc_puthex2(MACNTPNew[2]);
        dmc_puts(":");
        dmc_puthex2(MACNTPNew[3]);
        dmc_puts(":");
        dmc_puthex2(MACNTPNew[4]);
        dmc_puts(":");
        dmc_puthex2(MACNTPNew[5]);
        dmc_puts("\n");
//				dmc_puts(TERMINAL_DEFAULT);
        ntpdata[0] = MACNTPNew[0];
        ntpdata[1] = MACNTPNew[1];
        ntpdata[2] = MACNTPNew[2];
        ntpdata[3] = MACNTPNew[3];
        ntpdata[4] = MACNTPNew[4];
        ntpdata[5] = MACNTPNew[5];

        ntpdata[30] = IPNTP[0];
        ntpdata[31] = IPNTP[1];
        ntpdata[32] = IPNTP[2];
        ntpdata[33] = IPNTP[3];

        // KSZ8851SNL Transmit test
        dmc_puts(TERMINAL_LIGHT_BLUE);
        dmc_puts("Send NTP request to IP ");
        dmc_putint(IPNTP[0]);
        dmc_puts(".");
        dmc_putint(IPNTP[1]);
        dmc_puts(".");
        dmc_putint(IPNTP[2]);
        dmc_puts(".");
        dmc_putint(IPNTP[3]);
//				dmc_putintcr(++frameCount);
        dmc_puts("\n");
        dmc_puts(TERMINAL_DEFAULT);
        for (uint16_t i = 0; i < sizeof(ntpdata); i++)
        {
          pTXData[i + 5] = ntpdata[i];
        }
        txPacketLength = sizeof(ntpdata) + 5;
        ksz8851snl_reset_tx();
        ksz8851_Send(pTXData, txPacketLength);
      }
      else
      {
        dmc_puts(TERMINAL_CYAN);
        dmc_puts("ARP Cache: No MAC found for IP ");
        dmc_putint(IPNTP[0]);
        dmc_puts(".");
        dmc_putint(IPNTP[1]);
        dmc_puts(".");
        dmc_putint(IPNTP[2]);
        dmc_puts(".");
        dmc_putint(IPNTP[3]);
        dmc_puts("\n");
        dmc_puts(TERMINAL_LIGHT_BLUE);
        dmc_puts("Send ARP request to IP ");
        dmc_putint(IPNTP[0]);
        dmc_puts(".");
        dmc_putint(IPNTP[1]);
        dmc_puts(".");
        dmc_putint(IPNTP[2]);
        dmc_puts(".");
        dmc_putint(IPNTP[3]);
        dmc_puts("\n");
        dmc_puts(TERMINAL_DEFAULT);
        // IP address recognized, respond back with our (hardware) MAC address
        uint8_t arpdata[] =
        {
          // Ethernet II Header
          /*  0 */ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,	// Destination MAC Address (Broadcast)
          /*  6 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00,	// Source MAC Address
          /* 12 */ 0x08, 0x06,	// Type: ARP
          // ARP
          /* 14 */ 0x00, 0x01,	// Hardware type: 0x0001 (Ethernet)
          /* 16 */ 0x08, 0x00,	// Protocol type: 0x0800 (IPv4)
          /* 18 */ 0x06,		// Hardware size: 6
          /* 19 */ 0x04,		// Protocol size: 4
          /* 20 */ 0x00, 0x01,	// Opcode: 0x0001 (ARP Request), 0x0002 (ARP Reply)
          /* 22 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00, // Sender MAC Address
          /* 28 */ 192, 168, 25, 238,				// Sender IP Address: 192.168.25.238
          /* 32 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Target MAC Address (left blank)
          /* 38 */ IPNTP[0], IPNTP[1], IPNTP[2], IPNTP[3]		// Target IP Address: 192.168.25.41
        };

        // Reverse addresses and change ping request to reply
        //		// Change MAC
        //		for (uint16_t i = 0; i < 6; i++)
        //		{
        //			arpdata[i + 0] = RxData[i + 6];
        //			arpdata[i + 6] = RxData[i + 0];
        ////							arpdata[i + 22] = RxData[i + 32];
        //			arpdata[i + 32] = RxData[i + 22];
        //		}
        //		// Change IP
        //		for (uint16_t i = 0; i < 4; i++)
        //		{
        //			arpdata[i + 28] = RxData[i + 38];
        //			arpdata[i + 38] = RxData[i + 28];
        //		}

        // Copy frame, leave 5 bytes space in the head for the QMU header
        for (uint16_t i = 0; i < sizeof(arpdata); i++)
        {
          pTXData[i + 5] = arpdata[i];
        }
        uint16_t txPacketLength = sizeof(arpdata) + 5;
        ksz8851snl_reset_tx();
        ksz8851_Send(pTXData, txPacketLength);
      }

//
//				HAL_Delay(2);
////			}

    }

    if (HAL_GetTicks2(2000))
    {
      if (ping_count > 0)
      {
        // Copy frame, leave 5 bytes space in the head for the QMU header
//        if (ping_count < 5)
        {
        for (uint16_t i = 0; i < sizeof(pingdata); i++)
        {
          pTXData[i + 5] = pingdata[i];
        }
        txPacketLength = sizeof(pingdata) + 5;
        dmc_puts("Send PING\n");
        ping_time = msTick;
        ksz8851snl_reset_tx();
        ksz8851_Send(pTXData, txPacketLength);
        ping_count--;
        }
      }
    }

//		if (HAL_GetTicks2(100))
    {
      // Executed every 500 mS
//			HAL_StartTicks2();

//			if (ksz8851_IntHasOcurred())
//			{
////				uint16_t isr_reg = ksz8851_GetIntRegisterValue();
////
////				dmc_puts("isr_reg: ");
////				dmc_puthex4cr(isr_reg);
////
////				if (isr_reg & (1 << 13))
////				{
////					dmc_puts("IRQ_RXI\n");
////				}
//				uint16_t pRXLength = ksz8851_Receive(pRxData, 4000);
//			}

//			dmc_puts(TERMINAL_GREEN);
//			dmc_puts("ksz8851_Receive ");
//			dmc_putintcr(++frameCount);
//			dmc_puts(TERMINAL_DEFAULT);

      // Ignore first 11 bytes
      uint16_t pRXLength = ksz8851_Receive(pRXData, 4000);
      uint8_t *RxData = &pRXData[11];
      uint16_t RxLength = pRXLength;

      if (pRXLength > 42)
      {
        ksz8851snl_reset_rx();

        uint8_t DestIPFilter[] =
        { 192, 168, 25, 238 };
        uint8_t SrcIP[4];
        uint8_t DstIP[4];
        uint16_t offset = 23;

        // Find PING (ICMP Protocol) Packet
        offset = 12;
        if ((RxData[offset + 0] == 0x08) && (RxData[offset + 1] == 0x00) && (RxData[offset + 2] == 0x45)
            && (RxData[offset + 3] == 0x00) && (RxData[offset + 11] == 0x01)	// ICMP
            )
        {
          uint16_t totlen = (RxData[16] << 8) | (RxData[17] & 0xff);
          RxLength = totlen + 14;
          uint8_t *SrcIP = &RxData[26];
          uint8_t *DstIP = &RxData[30];
          if (memcmp(DstIP, DestIPFilter, sizeof(DestIPFilter)) == 0)
          {
            if (RxData[34] == 0)	// Ping Reply
            {
              uint32_t PingResponse = msTick - ping_time + 1;
              dmc_puts(TERMINAL_GREEN);
              dmc_puts("Reply from ");  // Ping Reply
              dmc_putint(SrcIP[0]);
              dmc_putc('.');
              dmc_putint(SrcIP[1]);
              dmc_putc('.');
              dmc_putint(SrcIP[2]);
              dmc_putc('.');
              dmc_putint(SrcIP[3]);
              dmc_puts(": ");
              dmc_puts("bytes=");
              dmc_putint(totlen - 28);
              dmc_puts(" time<");
              dmc_putint(PingResponse);
              dmc_puts("ms ");
              dmc_puts("TTL=");
              dmc_putint(RxData[22]);
              dmc_puts("\n");
              dmc_puts(TERMINAL_DEFAULT);
            }
            if (RxData[34] == 8)	// Ping Request, Send Reply
            {
              // IP address recognized, respond back with our (hardware) MAC address
              uint8_t pingdata[] =
              {
                // Ethernet II Header
                /*  0 */ 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf,	// Destination MAC Address
                /*  6 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00,	// Source MAC Address
                /* 12 */ 0x08, 0x00,	// Type: IPv4
                // IP Header
                /* 14 */ 0x45,// IP Version: 0100 = Version: 4, 0101 = Header Length: 20 bytes (5)
                /* 15 */ 0x00,		// Differentiated Services Field: 0x00 (DSCP: CS0, ECN: Not-ECT)
                /* 16 */ 0x00, 0x3c, // Total length: 60 bytes
                /* 18 */ 0x3e, 0x21,	// Identification: 0x3e21
                /* 20 */ 0x00, 0x00,	// Flags: 0x0000
                /* 22 */ 0x80,		// Time to Live: 128
                /* 23 */ 0x01,		// Protocol: ICMP (1)
                /* 24 */ 0x00, 0x00,	// Header checksum: 574b
                /* 26 */ 192, 168, 25, 238,		// Source IP: 192.168.25.238
                /* 30 */ 192, 168, 25, 41,		// Destination IP: 192.168.25.41
                // Ping packet
                /* 34 */ 0x00,// Type: 0 (Echo (ping) reply), 8 (Echo (ping) request)
                /* 35 */ 0x00,		// Code 0
                /* 36 */ 0x00, 0x00,               // Checksum: 0xe0cf
                /* 38 */ 0x00, 0x01,               // Identifier: 0x0001
                /* 40 */ 0x01, 0x3a,  // Sequence number: 0x013a, same as original
                /* 42 */ 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, // Data 32 bytes
                0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70,
                0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x61,
                0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69
              };

              // Reverse addresses and change ping request to reply
              // Change MAC
              for (uint16_t i = 0; i < 6; i++)
              {
                pingdata[i + 0] = RxData[i + 6];
                pingdata[i + 6] = RxData[i + 0];
              }
              // Change IP
              for (uint16_t i = 0; i < 4; i++)
              {
                pingdata[i + 30] = RxData[i + 26];
                pingdata[i + 26] = RxData[i + 30];
              }
              // get sequence number
              pingdata[40] = RxData[40];
              pingdata[41] = RxData[41];

              // Copy frame, leave 5 bytes space in the head for the QMU header
              for (uint16_t i = 0; i < sizeof(pingdata); i++)
              {
                pTXData[i + 5] = pingdata[i];
              }

              txPacketLength = sizeof(pingdata) + 5;	// 5 bytes for QMU header
              ksz8851snl_reset_tx();
              ksz8851_Send(pTXData, txPacketLength);
            }

          }
        }

        // Find ARP (Address Resolution Protocol) Packet
        if ((RxData[12] == 0x08) && (RxData[13] == 0x06) && (RxData[14] == 0x00) && (RxData[15] == 0x01)
            && (RxData[16] == 0x08) && (RxData[17] == 0x00) && (RxData[18] == 0x06) && (RxData[19] == 0x04)
            && (RxData[20] == 0x00) && ((RxData[21] == 0x01) || (RxData[21] == 0x02)))
        {
//					dmc_puts("ARP\n");
          uint8_t *SrcIP = &RxData[28];
          uint8_t *DstIP = &RxData[38];
          // Check if this ARP packet has our IP address, defined above in DestIPFilter

          if (RxData[21] == 0x01)	// ARP Request
          {
            if (memcmp(DstIP, DestIPFilter, sizeof(DestIPFilter)) == 0)
            {
              dmc_puts(TERMINAL_GREEN);
              dmc_puts("Received ARP request from IP ");
              dmc_putint(SrcIP[0]);
              dmc_puts(".");
              dmc_putint(SrcIP[1]);
              dmc_puts(".");
              dmc_putint(SrcIP[2]);
              dmc_puts(".");
              dmc_putint(SrcIP[3]);
              dmc_puts("\n");
              dmc_puts(TERMINAL_DEFAULT);

              RxLength = 42;

              // IP address recognized, respond back with our (hardware) MAC address
              uint8_t arpdata[] =
              {
                // Ethernet II Header
                /*  0 */ 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf,	// Destination MAC Address
                /*  6 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00,	// Source MAC Address
                /* 12 */ 0x08, 0x06,	// Type: ARP
                // ARP
                /* 14 */ 0x00, 0x01,	// Hardware type: 0x0001 (Ethernet)
                /* 16 */ 0x08, 0x00,	// Protocol type: 0x0800 (IPv4)
                /* 18 */ 0x06,		// Hardware size: 6
                /* 19 */ 0x04,		// Protocol size: 4
                /* 20 */ 0x00, 0x02,	// Opcode: 0x0002 (ARP Reply), 0x0001 (ARP Request)
                /* 22 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00, // Sender MAC Address
                /* 28 */ 192, 168, 25, 238,				// Sender IP Address: 192.168.25.238
                /* 32 */ 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf,	// Target MAC Address
                /* 38 */ 192, 168, 25, 41				// Target IP Address: 192.168.25.41
              };

              // Reverse addresses and change ping request to reply
              // Change MAC
              for (uint16_t i = 0; i < 6; i++)
              {
                arpdata[i + 0] = RxData[i + 6];
                arpdata[i + 6] = RxData[i + 0];
                //							arpdata[i + 22] = RxData[i + 32];
                arpdata[i + 32] = RxData[i + 22];
              }
              // Change IP
              for (uint16_t i = 0; i < 4; i++)
              {
                arpdata[i + 28] = RxData[i + 38];
                arpdata[i + 38] = RxData[i + 28];
              }

              uint8_t *ArpIP = &arpdata[38];
              uint8_t *ArpMAC = &arpdata[22];
              dmc_puts(TERMINAL_LIGHT_BLUE);
              dmc_puts("Send ARP reply to IP ");
              dmc_putint(ArpIP[0]);
              dmc_puts(".");
              dmc_putint(ArpIP[1]);
              dmc_puts(".");
              dmc_putint(ArpIP[2]);
              dmc_puts(".");
              dmc_putint(ArpIP[3]);
              dmc_puts(" we have MAC ");
              dmc_puthex2(ArpMAC[0]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[1]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[2]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[3]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[4]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[5]);
              dmc_puts("\n");
              dmc_puts(TERMINAL_DEFAULT);

              // Copy frame, leave 5 bytes space in the head for the QMU header
              for (uint16_t i = 0; i < sizeof(arpdata); i++)
              {
                pTXData[i + 5] = arpdata[i];
              }
              txPacketLength = sizeof(arpdata) + 5;
              ksz8851snl_reset_tx();
              ksz8851_Send(pTXData, txPacketLength);
            }
          }

          if (RxData[21] == 0x02)	// ARP Reply
          {
            if (memcmp(DstIP, DestIPFilter, sizeof(DestIPFilter)) == 0)
            {
              RxLength = 42;
              // Get IP and MAC to put in in the table
              uint8_t *ArpIP = &RxData[28];
              uint8_t *ArpMAC = &RxData[22];
              dmc_puts(TERMINAL_GREEN);
              dmc_puts("Received ARP Reply from IP ");
              dmc_putint(ArpIP[0]);
              dmc_puts(".");
              dmc_putint(ArpIP[1]);
              dmc_puts(".");
              dmc_putint(ArpIP[2]);
              dmc_puts(".");
              dmc_putint(ArpIP[3]);
              dmc_puts(", it has MAC ");
              dmc_puthex2(ArpMAC[0]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[1]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[2]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[3]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[4]);
              dmc_puts(":");
              dmc_puthex2(ArpMAC[5]);
              dmc_puts("\n");
              dmc_puts(TERMINAL_DEFAULT);
              (void) ARP_Add(ArpIP, ArpMAC);
//							NTP_Interval = 100;
            }
          }

        }

        uint8_t found = 0;
        uint16_t offset1 = 0;
        uint16_t offset2 = 0;

        for (uint16_t i = 0; i < RxLength; i++)
        {
          if ((RxData[i] == 0xC0) && (RxData[i + 1] == 0xA8) && (RxData[i + 2] == 0x19))
          {
            if (offset1 == 0)
            {
              offset1 = i;
            }
            else
            {
              offset2 = i;
              found = 1;
              break;
            }
          }
        }

//				if (found)
        {
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(' ');
//					dmc_putint(pRXData[offset1++]);
//					dmc_putc('.');
//					dmc_putint(pRXData[offset1++]);
//					dmc_putc('.');
//					dmc_putint(pRXData[offset1++]);
//					dmc_putc('.');
          uint8_t ipa4 = RxData[offset1 + 3];
//					dmc_putint(ipa4);
//					dmc_puts(" -> ");

//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(':');
//				dmc_puthex2(pRXData[offset++]);
//				dmc_putc(' ');
//					dmc_putint(pRXData[offset2++]);
//					dmc_putc('.');
//					dmc_putint(pRXData[offset2++]);
//					dmc_putc('.');
//					dmc_putint(pRXData[offset2++]);
//					dmc_putc('.');
          uint8_t ipb4 = RxData[offset2 + 3];
//					dmc_putint(ipb4);
//					dmc_putc('\n');

          uint16_t start = 42;
          uint8_t ip4 = 238;
          if ((ipa4 == ip4) || (ipb4 == ip4))
          {
//						dmc_putintcr(RxLength);
            if ((RxData[23] == 17) &&	// 0x11 UDP
                (RxData[34] != 0x08) && (RxData[35] != 0x00))
            {
              uint16_t totlen = (RxData[16] << 8) | (RxData[17] & 0xff);
//							dmc_putintcr(totlen);
              uint16_t udplen = (RxData[38] << 8) | (RxData[39] & 0xff);
//							dmc_putintcr(udplen);
              uint16_t udpsrcport = (RxData[34] << 8) | (RxData[35] & 0xff);
              uint16_t udpdstport = (RxData[36] << 8) | (RxData[37] & 0xff);
              if (udpdstport == 5005)
              {
                // Send packet back
                uint8_t udpdata[] =
                {
                    // Ethernet II Header
                    /*  0 */ 0xb4, 0xb6, 0x86, 0x8e, 0x57, 0xbf,	// Destination MAC Address
                    /*  6 */ 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00,	// Source MAC Address
                    /* 12 */ 0x08, 0x00,	// Type: IPv4
                    // IP Header
                    /* 14 */ 0x45,// IP Version: 0100 = Version: 4, 0101 = Header Length: 20 bytes (5)
                    /* 15 */ 0x00,		// Differentiated Services Field: 0x00 (DSCP: CS0, ECN: Not-ECT)
                    /* 16 */ 0x00, 0x2a, // Total length: 42 bytes
                    /* 18 */ 0x3e, 0x21,	// Identification: 0x3e21
                    /* 20 */ 0x00, 0x00,	// Flags: 0x0000
                    /* 22 */ 0x80,		// Time to Live: 128
                    /* 23 */ 0x11,		// Protocol: UDP (17)
                    /* 24 */ 0x00, 0x00,	// Header checksum: 0000
                    /* 26 */ 192, 168, 25, 41,		// Source IP: 192.168.25.41
                    /* 30 */ 192, 168, 25, 238,		// Destination IP: 192.168.25.238
                    // UDP Header
                    /* 34 */ 0x00, 0x00,	// Source Port
                    /* 36 */ 0x08, 0x00,	// Destination Port
                    /* 38 */ 0x06, 0x04,	// Length
                    /* 40 */ 0x00, 0x00,	// Checksum
                    /* 42 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Data
                    };

                // Reverse addresses and change ping request to reply
                // Adjust Length
                udpdata[16] = RxData[16];
                udpdata[17] = RxData[17];
                udpdata[38] = RxData[38];
                udpdata[39] = RxData[39];
                // Identification
                udpdata[18] = RxData[18];
                udpdata[19] = RxData[19];
                // Change MAC
                for (uint16_t i = 0; i < 6; i++)
                {
                  udpdata[i + 0] = RxData[i + 6];
                  udpdata[i + 6] = RxData[i + 0];
                }
                // Change IP
                for (uint16_t i = 0; i < 4; i++)
                {
                  udpdata[i + 26] = RxData[i + 30];
                  udpdata[i + 30] = RxData[i + 26];
                }
                // Change Ports
                udpdata[34] = RxData[36];
                udpdata[35] = RxData[37];
                udpdata[36] = RxData[34];
                udpdata[37] = RxData[35];
                // Get total UDP Packet size
                txPacketLength = (RxData[16] << 8) | RxData[17];
                // Add Ethernet II Header Size
                txPacketLength += 14;
                // Copy Data
                for (uint16_t i = 41; i < txPacketLength; i++)
                {
                  udpdata[i] = RxData[i];
                }
                // Swap case
                uint8_t *data = &udpdata[42];
                dmc_swap_case_len(data, txPacketLength - 42);

                // Check Message
                if (memcmp(data, "led1", 4) == 0)
                {
                  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
                }
                if (memcmp(data, "led2", 4) == 0)
                {
                  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                }

                uint8_t *UdpDstIP = &udpdata[30];
                dmc_puts(TERMINAL_LIGHT_BLUE);
                dmc_puts("Send UDP reply to IP ");
                dmc_putint(UdpDstIP[0]);
                dmc_puts(".");
                dmc_putint(UdpDstIP[1]);
                dmc_puts(".");
                dmc_putint(UdpDstIP[2]);
                dmc_puts(".");
                dmc_putint(UdpDstIP[3]);
                dmc_puts("\n");
                uint16_t start = 42;
                uint8_t pcr = 0;
                for (uint16_t i = start; i < RxLength; i++)
                {
                  if ((udpdata[i] >= 0x20) && (udpdata[i] < 0x7F))
                  {
                    dmc_putc(udpdata[i]);
                    pcr = 1;
                  }
                }
                if (pcr)
                {
                  dmc_putc('\n');
                }
                dmc_puts(TERMINAL_DEFAULT);

                // Copy frame, leave 5 bytes space in the head for the QMU header
                for (uint16_t i = 0; i < sizeof(udpdata); i++)
                {
                  pTXData[i + 5] = udpdata[i];
                }
                txPacketLength = sizeof(udpdata) + 5;
                ksz8851snl_reset_tx();
                ksz8851_Send(pTXData, txPacketLength);
//								memset(RxData, 0, txPacketLength);
              }
              RxLength = totlen + 14;
              start = 42;
              if (udpsrcport == 123)
              {
                // NTP package
                uint32_t ts = (RxData[82] << 24) | (RxData[83] << 16) | (RxData[84] << 8) | (RxData[85]);
//										(RxData[62] << 24) |
//										(RxData[63] << 16) |
//										(RxData[64] << 8) |
//										(RxData[65]);
                ts -= 2208988800L;

                dmc_puts(TERMINAL_GREEN);
                dmc_puts("Received NTP reply from IP ");
                dmc_putint(IPNTP[0]);
                dmc_puts(".");
                dmc_putint(IPNTP[1]);
                dmc_puts(".");
                dmc_putint(IPNTP[2]);
                dmc_puts(".");
                dmc_putint(IPNTP[3]);
                //				dmc_putintcr(++frameCount);
                dmc_puts("\n");
//								dmc_puts(TERMINAL_DEFAULT);

//								dmc_puts("TS: ");
//								dmc_puts(" (0x");
//
//								dmc_puthex(RxData[58]);
//								dmc_puthex(RxData[59]);
//								dmc_puthex(RxData[60]);
//								dmc_puthex(RxData[61]);
//
//								dmc_puts(")\n");

                RTC_FromEpoch(ts + 3600, &currentTime, &currentDate);
                dmc_puts(TERMINAL_LIGHT_MAGENTA);
                dmc_puts("Date and time: ");
                dmc_putint2(currentDate.Date, '0');
                dmc_puts("-");
                dmc_putint2(currentDate.Month, '0');
                dmc_puts("-20");
                dmc_putint2(currentDate.Year, '0');
                dmc_puts(" ");
                dmc_putint2(currentTime.Hours, '0');
                dmc_puts(":");
                dmc_putint2(currentTime.Minutes, '0');
                dmc_puts(":");
                dmc_putint2(currentTime.Seconds, '0');
                dmc_puts(" (");
                dmc_putint(ts);
                dmc_puts(")\n");
                dmc_puts(TERMINAL_YELLOW);

                NTP_Interval = 60000;

//								HAL_RTC_SetTime(&hrtc, &currentTime, RTC_FORMAT_BIN)
//
//								HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
//								HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

//								time(ts);

//								time_t rawtime;
//								char buffer[50] = { 0 };
////								struct tm *_timeptr;
//
//								time(&rawtime);
//								sprintf (buffer, "%s\n", ctime(&rawtime) );
//								dmc_puts(buffer);
//								time_t timep;
//
//								timep = gmtime(ts);
//								timep->
              }
              dmc_puts(TERMINAL_YELLOW);
              dmc_puts("UDP packet details:\n");
              dmc_puts("Src Port: ");
              dmc_putintcr(udpsrcport);
              dmc_puts("Dst Port: ");
              dmc_putintcr(udpdstport);
              dmc_puts("Length  : ");
              dmc_putintcr(RxLength);
            }

            dmc_puts(TERMINAL_YELLOW);
            for (uint16_t i = 0; i < RxLength; i++)
            {
              dmc_puthex2(RxData[i]);
              dmc_putc(' ');
            }
            dmc_putc('\n');

            uint8_t pcr = 0;
            for (uint16_t i = start; i < RxLength; i++)
            {
              if ((RxData[i] >= 0x20) && (RxData[i] < 0x7F))
              {
                dmc_putc(RxData[i]);
                pcr = 1;
              }
            }
            if (pcr)
            {
              dmc_putc('\n');
            }
            dmc_puts(TERMINAL_DEFAULT);

          }
          else
          {
//						dmc_puts(TERMINAL_CYAN);
          }
        }

//				ksz8851snl_reset_rx();

      }
      memset(RxData, 0, RxLength);

//			ksz8851snl_reset_rx();

    }

//	    MX_LWIP_Process();
//	    HAL_Delay(2);
//	    /* Read a received packet from the Ethernet buffers and send it
//	       to the lwIP for handling */
//	    ethernetif_input(&gnetif);
//	    /* Handle timeouts */
//	    sys_check_timeouts();
//	#if LWIP_NETIF_LINK_CALLBACK
//	    Ethernet_Link_Periodic_Handle(&gnetif);
//	#endif
//	#if LWIP_DHCP
//	    DHCP_Periodic_Handle(&gnetif);
//	#endif

//	    MX_LWIP_Process();
//	    dmc_puts(".");

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief  Setup the network interface
 * @param  None
 * @retval None
 */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

#if LWIP_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else

  /* IP address default setting */
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

#endif

  /* add the network interface */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /*  Registers the default network interface */
  netif_set_default(&gnetif);

  ethernet_link_status_updated(&gnetif);

#if LWIP_NETIF_LINK_CALLBACK
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);
#endif
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

  /**Supply configuration update enable 
   */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage 
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY)
  {

  }
  /**Macro to configure the PLL clock source 
   */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /**Initializes the CPU, AHB and APB busses clocks 
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
      | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART5
      | RCC_PERIPHCLK_RNG | RCC_PERIPHCLK_SPI4 | RCC_PERIPHCLK_SPI1 | RCC_PERIPHCLK_SPI2 | RCC_PERIPHCLK_I2C2
      | RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief This function is called to increment  a global variable "uwTick"
 *        used as application time base.
 * @note In the default implementation, this variable is incremented each 1ms
 *       in Systick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
 *      implementations in user file.
 * @retval None
 */
void HAL_IncTick(void)
{
  msTick += (uint32_t) 1;
}

/**
 * @brief Provides a tick value in millisecond.
 * @note This function is declared as __weak to be overwritten in case of other
 *       implementations in user file.
 * @retval tick value
 */
uint32_t HAL_GetTick(void)
{
  return msTick;
}

void HAL_StartTicks(void)
{
  msTickPrevious = msTick;
}

uint8_t HAL_GetTicks(uint32_t ms)
{
  if ((msTick - msTickPrevious) >= ms)
  {
    msTickPrevious = msTick;
    return true;
  }
  return false;
}

void HAL_StartTicks2(void)
{
  msTickPrevious2 = msTick;
}

uint8_t HAL_GetTicks2(uint32_t ms)
{
  if ((msTick - msTickPrevious2) >= ms)
  {
    msTickPrevious2 = msTick;
    return true;
  }
  return false;
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct =
  { 0 };

  /* Disables the MPU */
  HAL_MPU_Disable();
  /**Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /**Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
