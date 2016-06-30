/*
 * Copyright 2011 Carl-Benedikt Krueger, Chair for Operating Systems,
 *                                       RWTH Aachen University
 * Copyright 2015 Stefan Lankes, RWTH Aachen University
 * All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2 (https://www.gnu.org/licenses/gpl-2.0.txt)
 * or the BSD license below:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the University nor the names of its contributors
 *      may be used to endorse or promote products derived from this
 *      software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * mmnif.c	---	memmory mapped interface
 *
 * Virtual IP Interface initially designed for the concept processor SCC
 * and now adapted for HermitCore
 */

 /*
  * 14th September 2015:
  * - Adapted for Linux as interface to HermitCore (by Stefan Lankes)
  * - Removing of SCC related code (by Stefan Lankes)
  */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/ip.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <asm/atomic.h>

#include "hermit.h"

#define DRV_NAME	"mmnif"
#define DRV_VERSION	"1.0"

static struct net_device *mmnif_dev = NULL;

static void mmnif_get_drvinfo(struct net_device *dev,
                              struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
}

static const struct ethtool_ops mmnif_ethtool_ops __read_mostly = {
	.get_link    = ethtool_op_get_link,
	.get_drvinfo = mmnif_get_drvinfo,
};

/* mmnif_rxbuff_alloc():
 * this function allocates a continues chunk of memory
 * right inside of the buffer which is used for communication
 * with the remote end
 */
static size_t mmnif_rxbuff_alloc(struct mmnif_private *priv, uint8_t dest, uint16_t len)
{
	size_t ret = 0;
	volatile mm_rx_buffer_t *rb = (mm_rx_buffer_t *) ((char *)priv->header_start_address + dest * priv->header_size);
	char *memblock = (char *)priv->heap_start_address + dest * priv->heap_size;

//        if (rb->tail > rb->head)
//             if ((MMNIF_RX_BUFFERLEN - rb->tail < len)&&(rb->head < len))
//                 return NULL;
//        else
//            if ((rb->head - rb->tail < len)&&(rb->tail != rb->head))
//                return NULL;

	islelock_lock(priv->isle_locks + dest);
	if (rb->dcount)
	{
		if (rb->tail > rb->head)
		{
			if (MMNIF_RX_BUFFERLEN - rb->tail > len)
			{
				rb->desc_table[rb->dwrite].stat = MMNIF_STATUS_PENDING;
				ret = (size_t) (memblock + rb->tail);
				rb->desc_table[rb->dwrite].addr = virt_to_phys((void*)ret);
				rb->desc_table[rb->dwrite].len = len;
				rb->dcount--;
				rb->dwrite = (rb->dwrite + 1) % MMNIF_MAX_DESCRIPTORS;
				rb->tail = (rb->tail + len);
			} else if (rb->head > len) {
				rb->desc_table[rb->dwrite].stat = MMNIF_STATUS_PENDING;
				ret = (size_t) memblock;
				rb->desc_table[rb->dwrite].addr = virt_to_phys((void*)ret);
				rb->desc_table[rb->dwrite].len = len;
				rb->dcount--;
				rb->dwrite = (rb->dwrite + 1) % MMNIF_MAX_DESCRIPTORS;
				rb->tail = len;
			}
		} else {
			if (rb->head - rb->tail > len)
			{
				rb->desc_table[rb->dwrite].stat = MMNIF_STATUS_PENDING;
				ret = (size_t) (memblock + rb->tail);
				rb->desc_table[rb->dwrite].addr = virt_to_phys((void*)ret);
				rb->desc_table[rb->dwrite].len = len;
				rb->dcount--;
				rb->dwrite = (rb->dwrite + 1) % MMNIF_MAX_DESCRIPTORS;
				rb->tail = (rb->tail + len);
			} else if (rb->tail == rb->head) {
				if (MMNIF_RX_BUFFERLEN - rb->tail < len)
				{
					rb->tail = 0;
					if (rb->dread == rb->dwrite)
						rb->head = 0;
				}
				rb->desc_table[rb->dwrite].stat = MMNIF_STATUS_PENDING;
				ret = (size_t) (memblock + rb->tail);
				rb->desc_table[rb->dwrite].addr = virt_to_phys((void*)ret);
				rb->desc_table[rb->dwrite].len = len;
				rb->dcount--;
				rb->dwrite = (rb->dwrite + 1) % MMNIF_MAX_DESCRIPTORS;
				rb->tail = (rb->tail + len);
			}
		}
	}
	mb();
	islelock_unlock(priv->isle_locks + dest);

	return ret;
}

/* mmnif_commit_packet: this function set the state of the (in advance)
 * allocated packet to RDY so the recieve queue knows that it can be
 * processed further
 */
static int mmnif_commit_packet(struct mmnif_private *priv, uint8_t dest, size_t addr)
{
	volatile mm_rx_buffer_t *rb = (mm_rx_buffer_t *) ((char *)priv->header_start_address + dest * priv->header_size);
	uint32_t i;

	// be sure that the packet has been written
	mb();

	for(i=0; i < MMNIF_MAX_DESCRIPTORS; i++)
	{
		if ((rb->desc_table[i].addr == virt_to_phys((void*)addr))
		 && (rb->desc_table[i].stat == MMNIF_STATUS_PENDING))
		{
			rb->desc_table[i].stat = MMNIF_STATUS_RDY;

			return 0;
		}
	}

	return -1;
}

static netdev_tx_t mmnif_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mmnif_private *priv = netdev_priv(dev);
	struct iphdr* iph = ip_hdr(skb);
	size_t write_address;
	uint8_t dest_ip = (iph->daddr >> 24);
	unsigned long flags;

 	//pr_notice("mmnif_xmit: data %p, len %d, dest address %pI4, dest_ip %d\n", skb->data, skb->len-ETH_HLEN, &iph->daddr, (int)dest_ip);

	spin_lock_irqsave(&priv->lock, flags);

	/* allocate memory for the packet in the remote buffer */
realloc:
	write_address = mmnif_rxbuff_alloc(priv, dest_ip-1, skb->len-ETH_HLEN);
	if (!write_address)
	{
		//pr_notice("mmnif_tx(): concurrency");

		cpu_relax();
		goto realloc;
		//spin_unlock_irqrestore(&priv->lock, flags);
		//return NETDEV_TX_BUSY;
	}
	dev->trans_start = jiffies; /* save the timestamp */

#if 0 /* enable this conditional to look at the data */
	{
		int i;
		printk("TX len is %i, data:", skb->len);
		for (i=0 /*ETH_HLEN*/; i<skb->len; i++)
			printk(" %02x",skb->data[i]&0xff);
		printk("\n");
	}
#endif

	memcpy((char*) write_address, skb->data+ETH_HLEN, skb->len-ETH_HLEN);

	if (mmnif_commit_packet(priv, dest_ip-1, write_address))
	{
		pr_notice("mmnif_tx(): packet somehow lost during commit\n");
	}

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len-ETH_HLEN;

	spin_unlock_irqrestore(&priv->lock, flags);

	hermit_trigger_irq(dest_ip-1);

	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

/*
 * Return statistics to the caller
 */
struct net_device_stats *mmnif_stats(struct net_device *dev)
{
	struct mmnif_private *priv = netdev_priv(dev);
	return &priv->stats;
}

static int mmnif_change_carrier(struct net_device *dev, bool new_carrier)
{
	//pr_notice("mmnif_change_carrier: new_carrier %d\n", (int) new_carrier);

	if (new_carrier)
		netif_carrier_on(dev);
	else
		netif_carrier_off(dev);

	return 0;
}

int mmnif_set_carrier(bool new_carrier)
{
	return mmnif_change_carrier(mmnif_dev, new_carrier);
}

extern unsigned int isle_counter;

static int mmnif_get_iflink(const struct net_device *dev)
{
        //pr_notice("mmnif_get_iflink: %d\n", (isle_counter > 0));

	return (isle_counter > 0);
}

/*
 * This function is called to fill up an eth header, since arp is not
 * available on the interface
 */
int mmnif_header(struct sk_buff *skb, struct net_device *dev,
                unsigned short type, const void *daddr, const void *saddr,
                unsigned len)
{
	struct ethhdr *eth = (struct ethhdr *)skb_push(skb, ETH_HLEN);

	if (type != ETH_P_802_3 && type != ETH_P_802_2)
		eth->h_proto = htons(type);
	else
	        eth->h_proto = htons(len);

	memcpy(eth->h_source, saddr ? saddr : dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest,   daddr ? daddr : dev->dev_addr, dev->addr_len);

	return (dev->hard_header_len);
}

static const struct header_ops mmnif_header_ops = {
	.create  = mmnif_header
};

static const struct net_device_ops mmnif_ops __read_mostly = {
	.ndo_start_xmit = mmnif_xmit,
	.ndo_get_stats = mmnif_stats,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_carrier = mmnif_change_carrier,
	.ndo_get_iflink = mmnif_get_iflink,
};

/* mmnif_rxbuff_free() : the opposite to mmnif_rxbuff_alloc() a from the receiver
 * already processed chunk of memory is freed so that it can be allocated again
 */
static void mmnif_rxbuff_free(struct mmnif_private *priv)
{
	unsigned long flags;

	volatile mm_rx_buffer_t *b = priv->rx_buff;
	uint32_t i, j;
	uint32_t rpos;

	// be sure that we receive all data
	mb();

	local_irq_save(flags);
	islelock_lock(priv->isle_locks + 0);
	rpos = b->dread;

	for (i = 0, j = rpos; i < MMNIF_MAX_DESCRIPTORS; i++)
	{
		j = (j + i) % MMNIF_MAX_DESCRIPTORS;
		if (b->desc_table[j].stat == MMNIF_STATUS_PROC)
		{
			b->dcount++;
			b->dread = (b->dread + 1) % MMNIF_MAX_DESCRIPTORS;
			b->desc_table[j].stat = MMNIF_STATUS_FREE;
			if (b->tail > b->head)
			{
				b->head += b->desc_table[j].len;
			} else {
				if ((b->desc_table[(j + 1) % MMNIF_MAX_DESCRIPTORS].stat != MMNIF_STATUS_FREE)
				    && (b->desc_table[j].addr > b->desc_table[(j + 1) % MMNIF_MAX_DESCRIPTORS].addr))
				{
					b->head = 0;
				} else {
					b->head += b->desc_table[j].len;
				}
			}
		} else
			break;
	}

	mb();
	islelock_unlock(priv->isle_locks + 0);
	local_irq_restore(flags);
}

/*
 * Receive a packet : recieve, pack it up and pass over to higher levels
 */
static int mmnif_rx(struct net_device *dev, struct mmnif_private *priv)
{
	unsigned long flags;
	volatile mm_rx_buffer_t *b = priv->rx_buff;
	int npackets = 0;
	uint16_t length = 0;
	char *packet = NULL;
	uint32_t i, j;
	uint8_t rdesc;
	struct sk_buff *skb;

anotherpacket:
	local_irq_save(flags);
	rdesc = 0xFF;

	/* check if this call to mmnif_rx makes any sense
	 */
	if (b->desc_table[b->dread].stat == MMNIF_STATUS_FREE)
	{
		local_irq_restore(flags);
		goto out;
	}

	/* search the packet whose transmission is finished
	 */
	for (i = 0, j = b->dread; i < MMNIF_MAX_DESCRIPTORS; i++)
	{
		if (b->desc_table[(j + i) % MMNIF_MAX_DESCRIPTORS].stat == MMNIF_STATUS_RDY)
		{
			rdesc = (j + i) % MMNIF_MAX_DESCRIPTORS;
			b->desc_table[rdesc].stat = MMNIF_STATUS_INPROC;
			packet = (char *)b->desc_table[rdesc].addr;
			length = b->desc_table[rdesc].len;
			break;
		}

		if (b->desc_table[(j + i) % MMNIF_MAX_DESCRIPTORS].stat == MMNIF_STATUS_FREE)
		{
			local_irq_restore(flags);
			return npackets;
		}
	}

	local_irq_restore(flags);

	/* if there is no packet finished we encountered a random error
	 */
	if (rdesc == 0xFF)
		goto out;

	/* If length is zero return silently
	 */
	if (length == 0)
	{
		pr_notice("mmnif_rx(): empty packet error\n");
		goto out;
	}

	/* check for over/underflow */
        if ((length < 20 /* IP header size */) || (length > 1536))
	{
		pr_notice("mmnif_rx(): illegal packet length %d => drop the packet\n", length);
		goto drop_packet;
	}

	packet = phys_to_virt((phys_addr_t)packet);

	/* From now on there is a real packet and it
	 * has to be worked on
	 */
#ifdef DEBUG_MMNIF_PACKET
	pr_notice("\n RECIEVED - %p with legth: %d\n", packet, length);
	hex_dump(length, packet);
#endif

	/* Build the pbuf for the packet so a
	 * higher layer can handle it
	 */
	skb = dev_alloc_skb(length+ETH_HLEN+2);
	if (!skb)
	{
		pr_notice("mmnif_rx(): low on mem - packet dropped\n");
		goto drop_packet;
	}

	skb_reserve(skb, 2); /* align IP on 16B boundary */
	memcpy(skb_put(skb, length), packet, length); /* copy packet to sk_buff structure */
	skb->dev = dev;
	dev->header_ops->create(skb, dev, ETH_P_IP /*ETH_P_802_3*/, NULL, NULL, length+ETH_HLEN);
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY; //CHECKSUM_COMPLETE;

	/* indicate that the copy process is done and the packet can be freed
	 * note that we did not lock here because we are the only one editing this value
	 */
	priv->rx_buff->desc_table[rdesc].stat = MMNIF_STATUS_PROC;

	/* everything is copied to a new buffer so it's save to release
	 * the old one for new incoming packets
	 */
	mmnif_rxbuff_free(priv);

#if 0 /* enable this conditional to look at the data */
	{
		int i;
		printk("RX len is %i, data:", skb->len);
		for (i=0 /*ETH_HLEN*/; i<skb->len; i++)
			printk(" %02x",skb->data[i]&0xff);
		printk("\n");
	}
#endif

	if (unlikely(netif_rx(skb) == NET_RX_DROP)) {
		/* Maintain stats */
		npackets++;
		priv->stats.rx_dropped++;
		pr_notice("nnnif_rx(): receive corrupt message\n");
	} else {
	        /* Maintain stats */
		npackets++;
		priv->stats.rx_packets++;
		priv->stats.rx_bytes += length;
	 }

	goto anotherpacket;

drop_packet:
	/* TODO: error handling */
	priv->stats.rx_dropped++;

out:
	return npackets;
}

__visible void smp_mmnif_interrupt(struct pt_regs *regs)
{
	struct mmnif_private *priv;

	ipi_entering_ack_irq();
	inc_irq_stat(irq_mmnif_count);

	/* return if mmnif_dev is not yet initialized */
	if (!mmnif_dev)
	{
		pr_notice("mmnif_irqhandler(): the driver is not initialized yet\n");
		goto leave_handler;
	}

	priv = netdev_priv(mmnif_dev);
	if (priv)
		wake_up_process(priv->kthr);

leave_handler:
	irq_exit();
}

#if 0
__visible void smp_trace_mmnif_interrupt(struct pt_regs *regs)
{
	struct mmnif_private *priv;

	ipi_entering_ack_irq();
	inc_irq_stat(irq_mmnif_count);
	trace_mmnif_entry(MMNIF_VECTOR);

	/* return if mmnif_dev is not yet initialized */
	if (!mmnif_dev)
	{
		pr_notice("mmnif_irqhandler(): the driver is not initialized yet\n");
		goto leave_handler;
	}

	priv = netdev_priv(mmnif_dev);
	if (priv)
		wake_up_process(priv->kthr);

leave_handler:
        trace_mmnif_exit(MMNIF_VECTOR);
	irq_exit();
}
#endif

static int mmnif_thread(void* data)
{
	struct mmnif_private *priv;

	if (!mmnif_dev)
		return -EINVAL;

	priv = netdev_priv(mmnif_dev);
	if (!priv)
		return -EINVAL;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (!mmnif_dev)
			break;

		spin_lock(&priv->lock);
		mmnif_rx(mmnif_dev, priv);
		spin_unlock(&priv->lock);
	}

	return 0;
}

static void mmnif_setup(struct net_device *dev)
{
	pr_notice("mmnif: setup device\n");

	ether_setup(dev);

	dev->mtu 		= 1500;
	dev->hard_header_len	= ETH_HLEN;	/* 14	*/
	dev->addr_len		= ETH_ALEN;	/* 6	*/
	dev->tx_queue_len	= 0;
	dev->flags		|= IFF_NOARP;
	dev->flags		&= ~IFF_MULTICAST;
	dev->flags		&= ~IFF_BROADCAST;
	//dev->features 		|= NETIF_F_HW_CSUM;
	dev->ethtool_ops	= &mmnif_ethtool_ops;
	dev->header_ops		= &mmnif_header_ops;
	dev->netdev_ops		= &mmnif_ops;

	eth_random_addr(dev->dev_addr);
	netif_carrier_off(dev);
}

static int __init mmnif_init(void)
{
	struct net_device *dev = NULL;
	struct mmnif_private *priv = NULL;
	int ret = 0;

	pr_notice("mmnif: Initialize HermitCore's mmnif driver\n");

	if (mmnif_dev)
		return -EINVAL;

	if (!hermit_is_enabled())
		return -EINVAL;

	mmnif_dev = dev = alloc_netdev(0, "mmnif", NET_NAME_UNKNOWN, mmnif_setup);
	if (!dev) {
		printk(KERN_ERR "mmnif: Couldn't create device object!\n");
		return -ENOMEM;
	}

	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(struct mmnif_private));
	spin_lock_init(&priv->lock);
	priv->dev = dev;
	hermit_get_mmnif_data(priv);
	priv->kthr = kthread_run(mmnif_thread, NULL, "mmnif");

	if ((ret = register_netdev(dev))) {
		printk(KERN_ERR "%s: register network device failed\n", dev->name);
		goto err_out_free;
	}

	printk(KERN_INFO "%s: Initialized between HermitCore & Linux.\n", dev->name);

	return ret;

err_out_free:
	if (dev)
		free_netdev(dev);

	return ret;
}

static void mmnif_cleanup(void)
{
	if (mmnif_dev)
	{
		struct mmnif_private *priv = netdev_priv(mmnif_dev);

		if (priv)
			kthread_stop(priv->kthr);
		unregister_netdev(mmnif_dev);
		free_netdev(mmnif_dev);
		mmnif_dev = NULL;
	}
}

module_init(mmnif_init);
module_exit(mmnif_cleanup);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Stefan Lankes");
MODULE_DESCRIPTION("Memmory mapped network interface between HermitCore & Linux");
