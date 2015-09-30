/*
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
 */
#ifndef __HERMIT_H__
#define __HERMIT_H__

#include <linux/netdevice.h>

/*  define constants
 *  regarding the driver & its configuration
 */
#define MMNIF_MAX_DESCRIPTORS		64

#define MMNIF_STATUS_FREE		0x00
#define MMNIF_STATUS_PENDING            0x01
#define MMNIF_STATUS_RDY		0x02
#define MMNIF_STATUS_INPROC		0x03
#define MMNIF_STATUS_INPROC_BYPASS      0x04
#define MMNIF_STATUS_PROC		0x05

#define MMNIF_ACC_STAT_CLOSED           0x00
#define MMNIF_ACC_STAT_ACCEPTING        0x01
#define MMNIF_ACC_STAT_ACCEPT_ME        0x02
#define MMNIF_ACC_STAT_ACCEPTED         0x03

#define MMNIF_RX_BUFFERLEN      	(8*1024)
#define MMNIF_IRQ                       122
#define MMNIF_MAX_ACCEPTORS             0x20
#define MMNIF_MAX_DESCRIPTORS           64

typedef struct islelock {
        /// Internal queue
        atomic_t queue;
        /// Internal dequeue
        atomic_t dequeue;
} islelock_t;

/* accept struct
 */
typedef struct acceptor {
	/* stat: status of the acceptor
	 * src_ip: where did the connect request came from
	 * port: port on which the acceptor is listening
	 * nsock : next pseudo socket which is used in further connection
	 * rsock : remote socket which has to be assosicated with the nsock
	 */
	uint8_t stat;
	uint8_t src_ip;
	uint16_t port;
	int nsock;
	int rsock;
} acceptor_t;

/* receive descror structure */
typedef struct rx_desc {
	/* stat : status of the descriptor
	 * len  : length of the packet
	 * addr : memory address of the packet
	 * fast_sock: (-1) if no socket is associated
	 *             else the socket n of the fast socket
	 * id   : packet id
	 */
	uint8_t stat;
	uint16_t len;
	uint32_t fast_sock;
	size_t addr;
} rx_desc_t;

/* receive ring buffer structure */
typedef struct mm_rx_buffer {
	/* memory "pseudo-ring/heap"
	 * packets are always in one single chunk of memory
	 * head : head of allocated memory region
	 * tail : tail of allocated memory region
	 */
	uint16_t head;
	uint16_t tail;

	/* descritpor queue
	 * desc_table : descriptor table
	 * dcount : descriptor's free in queue
	 * dread : next descriptor to read
	 * dwrite : next descriptor to write
	 */
	rx_desc_t desc_table[MMNIF_MAX_DESCRIPTORS];
	uint8_t dcount;
	uint8_t dread;
	uint8_t dwrite;

	/* acceptors
	 * shared memory "hashtable" to realize
	 * fast socket accept/connect
	 */
	acceptor_t acceptors[MMNIF_MAX_ACCEPTORS];
} mm_rx_buffer_t;

struct mmnif_private {
	unsigned int header_size;
	unsigned int heap_size;
        char* header_start_address;
        char* heap_start_address;
	islelock_t* isle_locks;

        /* memory interaction variables:
	 * - pointer to recive buffer
	 */
	volatile mm_rx_buffer_t *rx_buff;
	uint8_t *rx_heap;

        // checks the TCPIP thread already the rx buffers?
        volatile char check_in_progress;

        struct net_device_stats stats;
	struct net_device *dev;
        struct napi_struct napi;
};

inline static int islelock_init(islelock_t* s)
{
        atomic_set(&s->queue, 0);
        atomic_set(&s->dequeue, 1);

        return 0;
}

inline static int islelock_destroy(islelock_t* s)
{
        return 0;
}

static inline int islelock_lock(islelock_t* s)
{
	int ticket;

	ticket = atomic_add_return(1, &s->queue);
	while(atomic_read(&s->dequeue) != ticket) {
		cpu_relax();
	}

	return 0;
}

static inline int islelock_unlock(islelock_t* s)
{
        atomic_inc(&s->dequeue);

        return 0;
}

int hermit_is_enabled(void);
void hermit_get_mmnif_data(struct mmnif_private*);
int hermit_trigger_irq(int dest_ip);

#endif
