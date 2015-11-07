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

#include <linux/kobject.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/memblock.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/cpu.h>
#include <asm/apic.h>
#include <asm/tsc.h>
#include <asm/msr.h>

#define NAME_SIZE	256
#define CMOS_PORT	0x70
#define CMOS_PORT_DATA	0x71

/* include code to switch from real mode to 64bit mode */
#include "boot.h"

#include "hermit.h"

static struct kobject *hermit_kobj = NULL;
static struct kobject *isle_kobj[NR_CPUS] = {[0 ... NR_CPUS-1] = NULL};
static int hcpu_online[NR_CPUS] = {[0 ... NR_CPUS-1] = -1};
static char path2hermit[NAME_SIZE] = "/hermit/hermit.bin";
static char* hermit_base[1 << NODES_SHIFT] = {[0 ... (1 << NODES_SHIFT)-1] = NULL};
static arch_spinlock_t boot_lock = __ARCH_SPIN_LOCK_UNLOCKED;
static size_t pool_size = 0x80000000ULL;

/* following variable are required for the ethernet driver */
static unsigned int header_size = PAGE_SIZE;
static unsigned int heap_size = MMNIF_RX_BUFFERLEN;
static char* header_phy_start_address = NULL;
static char* heap_phy_start_address = NULL;
static void* phy_isle_locks = NULL;
static void* phy_rcce_internals = NULL;
unsigned int isle_counter = 0;

/* tramploline to boot a CPU */
extern uint8_t* hermit_trampoline;

extern int mmnif_set_carrier(bool new_carrier);

void hermit_get_mmnif_data(struct mmnif_private* priv)
{
	int i;

	priv->header_size = header_size;
	priv->heap_size = heap_size;
	priv->header_start_address = phys_to_virt((phys_addr_t)header_phy_start_address);
	priv->heap_start_address = phys_to_virt((phys_addr_t)heap_phy_start_address);
	priv->isle_locks = (islelock_t*) phys_to_virt((phys_addr_t)phy_isle_locks);

	for(i=0; i<num_possible_nodes()+1; i++)
		islelock_init(priv->isle_locks + i);

	/* Linux uses always the id 0 */
	priv->rx_buff = (mm_rx_buffer_t *) (priv->header_start_address + header_size * (0));
	priv->rx_heap = (uint8_t*) priv->heap_start_address + heap_size * (0);

	memset((void*)priv->rx_buff, 0x00, header_size);
	memset((void*)priv->rx_heap, 0x00, heap_size);
	priv->rx_buff->dcount = MMNIF_MAX_DESCRIPTORS;
}

static inline void set_ipi_dest(uint32_t cpu_id)
{
	uint32_t tmp;

	tmp = apic_read(APIC_ICR2);
	tmp &= 0x00FFFFFF;
	tmp |= (cpu_id << 24);
	apic_write(APIC_ICR2, tmp);
}

/*
 * Wake up a core and boot HermitCore on it
 */
static int boot_hermit_core(int cpu, int isle, int cpu_counter, int total_cpus)
{
	int i;
	unsigned int start_eip;

	pr_notice("Isle %d tries to boot HermitCore on CPU %d (boot code size 0x%zx)\n", isle, cpu, sizeof(boot_code));

	if (!hermit_trampoline)
		return -EINVAL;
	start_eip = (unsigned int) virt_to_phys(hermit_trampoline);

	if (!hermit_base[isle])
		return -ENOMEM;

	if (apic_read(APIC_ICR) & APIC_ICR_BUSY) {
		pr_notice("ERROR: previous send not complete");
		return -EIO;
	}

	/* intialize trampoline code */
	memcpy(hermit_trampoline, boot_code, sizeof(boot_code));

	/* relocate code */
	*((uint16_t*) (hermit_trampoline+0x04)) += (unsigned short) start_eip;
	*((uint32_t*) (hermit_trampoline+0x10)) += start_eip;
	*((uint32_t*) (hermit_trampoline+0x29)) += start_eip;
	*((uint32_t*) (hermit_trampoline+0x2E)) += start_eip;
	*((uint32_t*) (hermit_trampoline+0x3A)) += start_eip;
	*((uint64_t*) (hermit_trampoline+0x5A)) += (uint64_t) start_eip;
	*((uint32_t*) (hermit_trampoline+0xCE)) += (uint32_t) (virt_to_phys(hermit_base[isle]) & 0xFFFFFFFF);
	*((uint32_t*) (hermit_trampoline+0xDD)) += (uint32_t) (virt_to_phys(hermit_base[isle]) >> 32);
	*((uint32_t*) (hermit_trampoline+0x188)) += start_eip;
	*((uint32_t*) (hermit_trampoline+0x18D)) += start_eip;

	if (!cpu_counter) {
		struct file * file;
		ssize_t sz, pos;
		ssize_t bytes;

		file = filp_open(path2hermit, O_RDONLY, 0);
		if (IS_ERR(file)) {
			int rc = PTR_ERR(file);
			pr_err("Unable to open file: %s (%d)\n", path2hermit, rc);
			return rc;
		}

		sz = i_size_read(file_inode(file));
		if (sz <= 0)
			return -EIO;
		pr_notice("Loading HermitCore's kernel image with a size of %d KiB\n", (int) sz >> 10);

		pos = 0;
		while (pos < sz) {
			bytes = kernel_read(file, pos, hermit_base[isle] + pos, sz - pos);
			if (bytes < 0) {
				pr_notice("Unable to load HermitCore's kernel\n");
				filp_close(file, NULL);
				return -EIO;
                	}
			if (bytes == 0)
				break;
			pos += bytes;
		}

		filp_close(file, NULL);

		/*
		 * set base, limit and cpu frequency in HermitCore's kernel
		 */
		*((uint64_t*) (hermit_base[isle] + 0x08)) = (uint64_t) virt_to_phys(hermit_base[isle]);
		*((uint64_t*) (hermit_base[isle] + 0x10)) = (uint64_t) virt_to_phys(hermit_base[isle]) + pool_size / num_possible_nodes();
		*((uint32_t*) (hermit_base[isle] + 0x18)) = cpu_khz / 1000;
		*((uint32_t*) (hermit_base[isle] + 0x1C)) = cpu;
		*((uint32_t*) (hermit_base[isle] + 0x24)) = total_cpus;
		*((uint32_t*) (hermit_base[isle] + 0x28)) = (uint64_t) phy_rcce_internals;
		*((uint32_t*) (hermit_base[isle] + 0x34)) = isle;
		*((uint64_t*) (hermit_base[isle] + 0x38)) = sz;
		*((uint64_t*) (hermit_base[isle] + 0x40)) = (uint64_t) phy_isle_locks;
		*((uint64_t*) (hermit_base[isle] + 0x48)) = (uint64_t) heap_phy_start_address;
		*((uint64_t*) (hermit_base[isle] + 0x50)) = (uint64_t) header_phy_start_address;
		*((uint32_t*) (hermit_base[isle] + 0x58)) = heap_size;
		*((uint32_t*) (hermit_base[isle] + 0x5c)) = header_size;
		*((uint32_t*) (hermit_base[isle] + 0x60)) = num_possible_nodes();
		*((uint64_t*) (hermit_base[isle] + 0x64)) = (uint64_t) heap_phy_start_address;
		*((uint64_t*) (hermit_base[isle] + 0x6c)) = (uint64_t) header_phy_start_address;
	}

	*((uint32_t*) (hermit_base[isle] + 0x30)) = cpu;

	local_irq_disable();

	/* set shutdown code to 0x0A */
	outb(CMOS_PORT, 0xF);  // offset 0xF is shutdown code
	outb(CMOS_PORT_DATA, 0x0A);

	/*
	 * use the universal startup algorithm of Intel's MultiProcessor Specification
	 */
	if (x2apic_enabled()) {
		uint64_t dest = ((uint64_t)cpu << 32);

		//pr_notice("X2APIC is enabled\n");

		wrmsrl(0x800 + (APIC_ICR >> 4), dest|APIC_INT_LEVELTRIG|APIC_INT_ASSERT|APIC_DM_INIT);
		udelay(200);
		/* reset INIT */
		wrmsrl(0x800 + (APIC_ICR >> 4), APIC_INT_LEVELTRIG|APIC_DM_INIT);
		udelay(10000);
		/* send out the startup */
		wrmsrl(0x800 + (APIC_ICR >> 4), dest|APIC_DM_STARTUP|(start_eip >> 12));
		udelay(200);
		/* do it again */
		wrmsrl(0x800 + (APIC_ICR >> 4), dest|APIC_DM_STARTUP|(start_eip >> 12));
		udelay(200);
	} else {
		//pr_notice("X2APIC is disabled\n");

		set_ipi_dest(cpu);
		apic_write(APIC_ICR, APIC_INT_LEVELTRIG|APIC_INT_ASSERT|APIC_DM_INIT);
		udelay(200);
		/* reset INIT */
		apic_write(APIC_ICR, APIC_INT_LEVELTRIG|APIC_DM_INIT);
		udelay(10000);
		/* send out the startup */
		set_ipi_dest(cpu);
		apic_write(APIC_ICR, APIC_DM_STARTUP|(start_eip >> 12));
		udelay(200);
		/* do it again */
		set_ipi_dest(cpu);
		apic_write(APIC_ICR, APIC_DM_STARTUP|(start_eip >> 12));
		udelay(200);
	}

	i = 0;
	while((apic_read(APIC_ICR) & APIC_ICR_BUSY) && (i < 1000))
		i++; /* wait for it to finish, give up eventualy tho */

	local_irq_enable();

	cpu_counter++;
	while(*((volatile uint32_t*) (hermit_base[isle] + 0x20)) < cpu_counter) { cpu_relax(); }

	return ((apic_read(APIC_ICR) & APIC_ICR_BUSY) ? -EIO : 0); // did it fail (still delivering) or succeed ?
}

/*
 * shows if HermitCore is running on a specific CPU
 */
static ssize_t hermit_is_cpus(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	char* path = kobject_get_path(kobj, GFP_KERNEL);
	int i, isle = NR_CPUS;
	int ret;

	if (!path)
		return -EINVAL;

	sscanf(path, "/hermit/isle%d", &isle);

	kfree(path);

	for(i=0, ret=0; i<NR_CPUS; i++){
		if (!ret && (hcpu_online[i] == isle))
			ret += sprintf(buf+ret, "%d", i);
		else if (hcpu_online[i] == isle)
			ret += sprintf(buf+ret, ", %d", i);
	}

	if (!ret)
		return sprintf(buf, "%d\n", -1);

	ret += sprintf(buf+ret, "\n");

	return ret;
}

static int shutdown_hermit_core(unsigned cpu)
{
	int ret = -EIO;

	local_irq_disable();

	if (x2apic_enabled()) {
		uint64_t dest = ((uint64_t)cpu << 32);

		wrmsrl(0x830, dest|APIC_INT_ASSERT|APIC_DM_FIXED|(81+32));
	} else {
		int j;

		if (apic_read(APIC_ICR) & APIC_ICR_BUSY) {
			pr_notice("ERROR: previous send not complete");
			goto Lerr;
		}

		set_ipi_dest(cpu);
		apic_write(APIC_ICR, APIC_INT_ASSERT|APIC_DM_FIXED|(81+32));

		j = 0;
		while((apic_read(APIC_ICR) & APIC_ICR_BUSY) && (j < 1000))
			j++; // wait for it to finish, give up eventualy tho

		if (j >= 1000) {
			pr_notice("ERROR: send not complete");
			ret = -EBUSY;
		} else ret = 0;
	}

Lerr:
	local_irq_enable();

	return ret;
}

/*
 * boot or shut down HermitCore
 */
static ssize_t hermit_set_cpus(struct kobject *kobj, struct kobj_attribute *attr,
 				const char *buf, size_t count)
{
	char* path = kobject_get_path(kobj, GFP_KERNEL);
	int i, isle = NR_CPUS;
	int ret, possible_cpus;
	cycles_t tick0, tick1;
	int cpus[NR_CPUS];

	if (!path)
		return -EINVAL;

	sscanf(path, "/hermit/isle%d", &isle);

	kfree(path);

	get_options(buf, NR_CPUS, cpus);

	// How many cpus do we activate?
	possible_cpus = cpus[0];
	if ((possible_cpus <= 0) || (possible_cpus >= NR_CPUS))
		return -EINVAL;

	if (cpus[1] < 0) {
		pr_notice("Try to shutdown HermitCore on isle %d\n", isle);

		arch_spin_lock(&boot_lock);

		// Ok, we have to shutdown the isle
		for(i=0; i<NR_CPUS; i++)  {
			if (hcpu_online[i] == isle) {
				if (!shutdown_hermit_core(i))
					hcpu_online[i] = -1;
			}
		}

		isle_counter--;
		if (isle_counter == 0)
			mmnif_set_carrier(false);

		arch_spin_unlock(&boot_lock);

		return count;
	}

	pr_notice("Try to boot HermitCore on isle %d (%d cpus)\n", isle, cpus[0]);

	/* Do Linux or HermitCore already use the CPUs? */
	for(i=0; i<possible_cpus; i++) {
		if ((cpus[i+1] < 0) || (cpus[i+1] >= NR_CPUS))
			return -EINVAL;
		if (hcpu_online[cpus[i+1]] >= 0)
			return -EBUSY;
		if (cpu_online(cpus[i+1]))
			return -EINVAL;
	}

	/* Do HermitCore already use the isle? */
	for(i=0; i<NR_CPUS; i++)  {
		if (hcpu_online[i] == isle)
			return -EBUSY;
	}

	//TODO: to avoid problems with Linux, we disable the hotplug feature
	cpu_hotplug_disable();

	tick0 = get_cycles();

	arch_spin_lock(&boot_lock);

	/* reserve CPU for our isle */
	for(i=0; i<possible_cpus; i++) {
		ret = boot_hermit_core(cpus[i+1], isle, i, possible_cpus);
		if (!ret)
			hcpu_online[cpus[i+1]] = isle;
	}

	isle_counter++;
	if (isle_counter == 1)
		mmnif_set_carrier(true);

	arch_spin_unlock(&boot_lock);

	tick1 = get_cycles();

	pr_notice("HermitCore requires %lld ms (%lld ticks) to boot the system\n", (tick1-tick0) / cpu_khz, tick1-tick0);

	return count;
}

/*
 * Interface to display log messages from HermitCore
 */
static ssize_t hermit_get_log(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
	int isle = NR_CPUS;
	char* path = kobject_get_path(kobj, GFP_KERNEL);

	if (!path)
		return -EINVAL;

	sscanf(path, "/hermit/isle%d", &isle);

	kfree(path);

	if ((isle >= 0) && (isle < NR_CPUS) && hermit_base[isle])
		return snprintf(buf, PAGE_SIZE, "%s\n", hermit_base[isle]+5*PAGE_SIZE);

	return -ENOMEM;
}

/*
 * Get path to the kernel
 */
static ssize_t hermit_get_path(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
	return sprintf(buf, "%s\n", path2hermit);
}

/*
 * Set path to the kernel
 */
static ssize_t hermit_set_path(struct kobject *kobj, struct kobj_attribute *attr,
                                const char *buf, size_t count)
{
	return snprintf(path2hermit, NAME_SIZE, "%s", buf);
}

/*
 * Get default size of HermitCore's memory
 */
static ssize_t hermit_get_memsize(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
	return sprintf(buf, "%zd K per isle\n", pool_size / (1024 * num_possible_nodes()));
}

/*
 * Determine base address of HermitCore's kernel
 */
static ssize_t hermit_get_base(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
	int isle = NR_CPUS;
	char* path = kobject_get_path(kobj, GFP_KERNEL);

	if (!path)
		return -EINVAL;

	sscanf(path, "/hermit/isle%d", &isle);

	kfree(path);

	if (isle >= NR_CPUS)
		return -EINVAL;

	return sprintf(buf, "0x%lx\n", (size_t) hermit_base[isle]);
}

/*
 * Create sysfs entries as communication interface between Linux user
 * and the HermitCore kernel
 *
 * Usage:
 * Boot CPU X            : echo 1 > /sys/hermit/isleX/cpus
 * Boot CPU X-Y		 : echo 1-2 > /sys/hermit/isleX/cpus
 * Shut down all CPUs    : echo -1 > /sys/hermit/isleX/online
 * Show log messages     : cat /sys/hermit/log
 * Start address         : cat /sys/hermit/base
 * Memory size           : cat /sys/hermit/memsize
 * Set path to HermitCore: echo "/hermit.bin" > /sys/hermit/path
 * Get path to HermitCore: cat /sys/hermit/path
 */

static struct kobj_attribute cpu_attribute =
	__ATTR(cpus, 0644, hermit_is_cpus, hermit_set_cpus);

static struct kobj_attribute log_attribute =
	__ATTR(log, 0644, hermit_get_log, NULL);

static struct kobj_attribute path_attribute =
	__ATTR(path, 0644, hermit_get_path, hermit_set_path);

static struct kobj_attribute memsize_attribute =
	__ATTR(memsize, 0644, hermit_get_memsize, NULL);

static struct kobj_attribute base_attribute =
	__ATTR(base, 0644, hermit_get_base, NULL);

static struct attribute * isle_attrs[] = {
	&cpu_attribute.attr,
	&log_attribute.attr,
	&base_attribute.attr,
	NULL
};

static struct attribute * hermit_attrs[] = {
	&path_attribute.attr,
	&memsize_attribute.attr,
	NULL
};

static struct attribute_group hermit_attr_group = {
	.attrs = hermit_attrs,
};

static struct attribute_group isle_attr_group = {
	.attrs = isle_attrs,
};

static int set_hermit_pool_size(char *arg)
{
	pool_size = memparse(arg, NULL);
        return 0;
}
early_param("hermit_pool_size", set_hermit_pool_size);

static int enable_hermit = 1;
static int disable_hermit(char *arg)
{
	enable_hermit = 0;
	return 0;
}
early_param("disable_hermit", disable_hermit);

int hermit_is_enabled(void)
{
	return enable_hermit;
}

int __init hermit_init(void)
{
	int i, ret;
	char name[NAME_SIZE];
	phys_addr_t mem;
	ulong flags = choose_memblock_flags();
	int* tmp;

	if (!enable_hermit)
		return 0;

	pr_notice("Initialize HermitCore\n");
	pr_notice("HermitCore trampoline at 0x%p (0x%zx)\n", hermit_trampoline, (size_t) virt_to_phys(hermit_trampoline));
	pr_notice("Number of available nodes: %d\n", num_possible_nodes());
	pr_notice("Pool size: 0x%zd KiB\n", pool_size / 1024);

	/* allocate for each HermitCore instance */
	for(i=0; i<num_possible_nodes(); i++) {
		mem = memblock_find_in_range_node(pool_size / num_possible_nodes(), 2 << 20, 4 << 20, MEMBLOCK_ALLOC_ACCESSIBLE, i, flags);
		//mem = memblock_find_in_range(4 << 20, 100<<20, CONFIG_HERMIT_SIZE, 2 << 20);
		if (!mem) {
			ret = -ENOMEM;
			goto _exit;
		}

		ret = memblock_reserve(mem, pool_size / num_possible_nodes());
		if (ret) {
			ret = -ENOMEM;
			goto _exit;
		}

		hermit_base[i] = (char*) phys_to_virt(mem);
		pr_notice("HermitCore %d at 0x%p (0x%zx)\n", i, hermit_base[i], (size_t) mem);
	}

	mem = memblock_find_in_range(4 << 20, 4 << 25, heap_size * (num_possible_nodes() + 1), PAGE_SIZE);
	if (!mem) {
		ret = -ENOMEM;
		goto _exit;
	}

	ret = memblock_reserve(mem, heap_size * (num_possible_nodes() + 1));
	if (ret) {
		ret = -ENOMEM;
		goto _exit;
	}

	heap_phy_start_address = (char*) mem;
	pr_notice("HermitCore's mmnif heap at 0x%p\n", heap_phy_start_address);

	mem = memblock_find_in_range(4 << 20, 4 << 25, header_size * (num_possible_nodes() + 1), PAGE_SIZE);
	if (!mem) {
		ret = -ENOMEM;
		goto _exit;
	}

	ret = memblock_reserve(mem, header_size * (num_possible_nodes() + 1));
	if (ret) {
		ret = -ENOMEM;
		goto _exit;
	}

	header_phy_start_address = (char*) mem;
	pr_notice("HermitCore's mmnif header at 0x%p\n", header_phy_start_address);

	mem = memblock_find_in_range(4 << 20, 4 << 25, 4 * sizeof(unsigned int) * (num_possible_nodes() + 1), PAGE_SIZE);
	if (!mem) {
		ret = -ENOMEM;
		goto _exit;
	}

	ret = memblock_reserve(mem, sizeof(islelock_t) * (num_possible_nodes() + 1));
	if (ret) {
		ret = -ENOMEM;
		goto _exit;
	}

	phy_isle_locks = (void*) mem;
	pr_notice("HermitCore's locks are mapped at 0x%p\n", phy_isle_locks);

	mem = memblock_find_in_range(4 << 20, 4 << 25, PAGE_SIZE, PAGE_SIZE);
	if (ret) {
		ret = -ENOMEM;
		goto _exit;
	}

	phy_rcce_internals = (void*) mem;
	pr_notice("RCCE infos are mapped at 0x%p\n", phy_rcce_internals);

	// init RCCE locks
	tmp = (int*) phys_to_virt(mem);
	memset(tmp, 0x00, PAGE_SIZE);
	tmp[1] = 1;

	/*
	 * Create a kobject for HermitCore and located
	 * under the sysfs directory (/sys)
 	 */
	hermit_kobj = kobject_create_and_add("hermit", NULL);
	if (!hermit_kobj) {
		ret = -ENOMEM;
		goto _exit;
	}

	ret = sysfs_create_group(hermit_kobj, &hermit_attr_group);
	if (ret)
		goto _exit;

	/*
	 * Create for each possible CPU an entry and located under /sys/hermit
	 */
	for(i=0; i<num_possible_nodes(); i++) {
		if (snprintf(name, NAME_SIZE, "isle%d", i) >= NAME_SIZE) {
			printk(KERN_WARNING "HermitCore: buffer is too small\n");
			continue;
		}

		isle_kobj[i] = kobject_create_and_add(name, hermit_kobj);
		if (!isle_kobj[i]) {
			ret = -ENOMEM;
			goto _exit;
		}

		ret = sysfs_create_group(isle_kobj[i], &isle_attr_group);
		if (ret)
			goto _exit;
	}

	return 0;

_exit:
	if (hermit_kobj)
		kobject_put(hermit_kobj);
	for_each_possible_cpu(i) {
		if (isle_kobj[i])
			kobject_put(isle_kobj[i]);
	}

	return ret;
}

static int apic_send_ipi(uint64_t dest, uint8_t irq)
{
	local_irq_disable();

        if (x2apic_enabled()) {
                wrmsrl(0x830, (dest << 32)|APIC_INT_ASSERT|APIC_DM_FIXED|irq);
        } else {
		uint32_t j;

                if (apic_read(APIC_ICR) & APIC_ICR_BUSY) {
                        printk("ERROR: previous send not complete");
                        return -EIO;
                }

                set_ipi_dest((uint32_t)dest);
                apic_write(APIC_ICR, APIC_INT_ASSERT|APIC_DM_FIXED|irq);

                j = 0;
                while((apic_read(APIC_ICR) & APIC_ICR_BUSY) && (j < 1000))
                        j++; // wait for it to finish, give up eventualy tho
        }

	local_irq_enable();

        return 0;
}

/* trigger an interrupt on the remote processor
 * so he knows there is a packet to read
 */
int hermit_trigger_irq(int dest)
{
	int i, cpu = -1;

	if (dest < 1)
	 	return -EINVAL;

	for(i=0; (cpu == -1) && (i<NR_CPUS); i++) {
		if (hcpu_online[i] == dest-1)
			cpu = i;
	}

	if (cpu == -1)
		return -EINVAL;

	return apic_send_ipi(cpu, MMNIF_IRQ);
}
