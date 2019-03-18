#include "dmc_net.h"

#include "lwip/inet.h"
#include "lwip/tcp.h"
//#include "lwip/ip_frag.h"
#include "lwip/netif.h"
#include "lwip/init.h"
#include "lwip/stats.h"
#include "netif/etharp.h"
//#include "lwip/timers.h"
//#include "lwip/tcp_impl.h"
#include "lwip/udp.h"

#include <time.h>
#include <assert.h>

#define NUM_INTERFACES				2

#define MCH_IPADDR_NETMASK 			"255.255.255.0"
#define MCH_IPADDR_GW	 			"192.168.2.253"

char *MCH_IP_ADDRS[NUM_INTERFACES] =
{ "192.168.2.11", "192.168.2.22" };

//struct ip_addr mch_myip_addr[NUM_INTERFACES];
static struct netif mchdrv_netif[NUM_INTERFACES];

#define MCH_ARP_TIMER_INTERVAL      (ARP_TMR_INTERVAL * 1000)
//#define MCH_TCP_TIMER_INTERVAL      (TCP_TMR_INTERVAL * 1000)
//#define MCH_IPREASS_TIMER_INTERVAL  (IP_TMR_INTERVAL * 1000)

//typedef struct timespec mch_timestamp;

//void mch_timestamp_get(mch_timestamp *ts)
//{
////	clock_gettime(CLOCK_MONOTONIC, ts);
//}
//
//s32_t mch_timestamp_diff(mch_timestamp *a, mch_timestamp *b)
//{
//	return 1000 * (b->tv_sec - a->tv_sec) + (b->tv_nsec - a->tv_nsec) / (1000 * 1000);
//}

void mch_timestamp_init()
{
	return;
}

// Use the sys_now function to obtain a timestamp.
// In order to operate, the stack needs to have certain functions called at regular intervals
// to perform house-keeping tasks, such as handling TCP timeouts, retransmissions and so forth.

//u32_t sys_now()
//{
//	struct timespec ts;
////	clock_gettime(CLOCK_MONOTONIC, &ts);
//	return ts.tv_sec * 1000 + (ts.tv_nsec / (1000 * 1000));
//}

//static mch_timestamp ts_etharp;
//static mch_timestamp ts_tcp;
//static mch_timestamp ts_ipreass;

//uint8_t mchdrv_output(struct netif *netif, struct pbuf *p, ip_addr_t *ipaddr)
//{
//	struct pbuf *dest_pbuf;
//	struct netif *dest_netif;
//	uint8_t err;
//
//	/* verbose print */
//	printf("%s: on interface %d, len %d\n", __func__, (int) (uint64_t) netif->state, p->tot_len);
//
//	/* we copy p to a new destination pbuf, because the tcp stack could change
//	 * buffers on retransmits (according to the wiki) */
//	dest_pbuf = pbuf_alloc(PBUF_RAW, p->tot_len, PBUF_RAM);
//	if (dest_pbuf == NULL)
//	{
//		printf("%s: could not allocate destination pbuf\n", __func__);
//		return ERR_MEM;
//	}
//
//	err = pbuf_copy(dest_pbuf, p);
//	if (err != ERR_OK)
//	{
//		printf("%s: buffer copy returned %d\n", __func__, err);
//		pbuf_free(dest_pbuf);
//		return err;
//	}
//
//	/* input the packet to the other netif */
//	assert((uint64_t )netif->state <= 1);
//	dest_netif = &mchdrv_netif[1 - (u8_t) (uint64_t) netif->state];
//	err = dest_netif->input(dest_pbuf, dest_netif);
//	if (err != ERR_OK)
//		printf("%s: dest_netif->input returned error %d", __func__, err);
//
//	return ERR_OK;
//}

//err_t mchdrv_init(struct netif *netif)
//{
//	/* we're not using hwaddr */
//	netif->hwaddr_len = 0;
//
//	netif->mtu = 1500;
//	netif->name[0] = 'i';
//	netif->name[1] = 'p';
//	netif->num = (u8_t) (uint64_t) netif->state;
//
//	netif->output = mchdrv_output;
//
//	return ERR_OK;
//}

//int mch_net_init(void)
//{
//	struct ip_addr gw_addr, netmask;
//	struct mch_pci_dev * mchdrv_pcidev;
//	void * mchdrvnet_priv;
//	uint8_t mac_addr[6];
//	int err = -1;
//	int i;
//
//	// Hard-coded IP for my address, gateway and netmask
//	for (i = 0; i < NUM_INTERFACES; i++)
//	{
//		if (mch_net_aton(MCH_IP_ADDRS[i], &mch_myip_addr[i]))
//			return -1;
//	}
//	if (mch_net_aton(MCH_IPADDR_GW, &gw_addr))
//		return -1;
//	if (mch_net_aton(MCH_IPADDR_NETMASK, &netmask))
//		return -1;
//
//	// Initialize LWIP
//	lwip_init();
//
//	// Add our netif to LWIP (netif_add calls our driver initialization function)
//	for (i = 0; i < NUM_INTERFACES; i++)
//	{
//		if (netif_add(&mchdrv_netif[i], &mch_myip_addr[i], &netmask, &gw_addr, (void *) (uint64_t) i, mchdrv_init,
//				ip_input) == NULL)
//		{
//			mch_printf("mch_net_init (%d): netif_add (mchdrv_init) failed\n", i);
//			return -1;
//		}
//	}
//
//	netif_set_default(&mchdrv_netif[0]);
//	netif_set_up(&mchdrv_netif[0]);
//	netif_set_up(&mchdrv_netif[1]);
//
//	// Initialize timer values
//	mch_timestamp_get(&ts_etharp);
//	mch_timestamp_get(&ts_tcp);
//	mch_timestamp_get(&ts_ipreass);
//
//	return 0;
//}

//
// Regular polling mechanism.  This should be called each time through
// the main application loop (after each interrupt, regardless of source).
//
// It handles any received packets, permits NIC device driver house-keeping
// and invokes timer-based TCP/IP functions (TCP retransmissions, delayed
// acks, IP reassembly timeouts, ARP timeouts, etc.)
//
void mch_net_poll(void)
{
//	mch_timestamp now;

	// Call network interface to process incoming packets and do housekeeping
	//mchdrv_poll(&mchdrv_netif);

	// Process lwip network-related timers.
//	mch_timestamp_get(&now);
//	if (mch_timestamp_diff(&ts_etharp, &now) >= MCH_ARP_TIMER_INTERVAL)
//	{
//		etharp_tmr();
//		ts_etharp = now;
//	}
//	if (mch_timestamp_diff(&ts_tcp, &now) >= MCH_TCP_TIMER_INTERVAL)
//	{
//		tcp_tmr();
//		ts_tcp = now;
//	}
//	if (mch_timestamp_diff(&ts_ipreass, &now) >= MCH_IPREASS_TIMER_INTERVAL)
//	{
//		ip_reass_tmr();
//		ts_ipreass = now;
//	}
}

//
// Convert address from string to internal format.
// Return 0 on success; else non-zero
//
//int mch_net_aton(char * str_addr, struct ip_addr * net_addr)
//{
//	struct in_addr a;
//	int i = inet_aton(str_addr, &net_addr->addr);
//	if (!i)
//		return -1;
//	return 0;
//}

//void test_udp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, ip_addr_t *addr, u16_t port)
//{
//	printf("%s: len %d, from %s port %d\n", __func__, p->tot_len, ipaddr_ntoa(addr), port);
//
//	/* it's our responsibility to dealloc the pbuf */
//	pbuf_free(p);
//}

//void test_udp()
//{
//	struct udp_pcb *udp_src;
//	struct udp_pcb *udp_dst;
//	struct pbuf *p;
//	err_t err;
//
//	/* note: we don't bother freeing memory on failure */
//
//	printf("testing udp.\n");
//
//	printf("\tallocating pcbs.\n");
//	udp_src = udp_new();
//	if (udp_src == NULL)
//	{
//		printf("%s: couldn't allocate src pcb\n", __func__);
//		return;
//	}
//	udp_dst = udp_new();
//	if (udp_dst == NULL)
//	{
//		printf("%s: couldn't allocate dst pcb\n", __func__);
//		return;
//	}
//
//	/* bind the server side */
//	err = udp_bind(udp_dst, &mch_myip_addr[1], 2222);
//	if (err != ERR_OK)
//	{
//		printf("%s: binding returned error %d\n", __func__, err);
//		return;
//	}
//	udp_recv(udp_dst, test_udp_recv, NULL);
//
//	/* make test pbuf */
//	p = pbuf_alloc(PBUF_TRANSPORT, 15, PBUF_RAM);
//	if (p == NULL)
//	{
//		printf("%s: couldn't allocate pbuf\n", __func__);
//		return;
//	}
//
//	/* send the pbuf */
//	err = udp_sendto_if(udp_src, p, &mch_myip_addr[1], 2222, &mchdrv_netif[0]);
//	if (err != ERR_OK)
//	{
//		printf("%s: udp_sendto_if returned %d\n", __func__, err);
//		return;
//	}
//}

////
//// Main entry point
////
//int main(void)
//{
//    mch_timestamp_init();       // Initialize timestamp generator
//    mch_net_init();
//    test_udp();
////    while (1) {
////        [snip other non-lwip functions]
////        mch_wait_for_interrupt();   // Awakened by network, timer or other interrupt
////        mch_net_poll();             // Poll network stack
////    }
//
//}

