#ifndef __DMC_NET_H
#define __DMC_NET_H
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "main.h"

//void mch_timestamp_get(mch_timestamp *ts);
//int32_t mch_timestamp_diff(mch_timestamp *a, mch_timestamp *b);
void mch_timestamp_init();
//u32_t sys_now();
//uint8_t mchdrv_output(struct netif *netif, struct pbuf *p, ip_addr_t *ipaddr);
//err_t mchdrv_init(struct netif *netif);
//int mch_net_init(void);
void mch_net_poll(void);
//int mch_net_aton(char * str_addr, struct ip_addr * net_addr);
//void test_udp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, ip_addr_t *addr, u16_t port);
void test_udp();

#ifdef __cplusplus
}
#endif
#endif /* __DMC_NET_H */
