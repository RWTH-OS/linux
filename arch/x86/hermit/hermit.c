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

#define DEBUG

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
#include <linux/mc146818rtc.h>
#include <linux/elf.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/cpu.h>
#include <asm/apic.h>
#include <asm/tsc.h>
#include <asm/msr.h>
#include <asm/tlbflush.h>

#define NAME_SIZE	256

/* include code to switch from real mode to 64bit mode */
#include "boot.h"

#include "hermit.h"

static struct kobject *hermit_kobj = NULL;
static struct kobject *isle_kobj[NR_CPUS] = {[0 ... NR_CPUS-1] = NULL};
static int hcpu_online[NR_CPUS] = {[0 ... NR_CPUS-1] = -1};
static char* path2hermit[MAX_NUMNODES] = {[0 ... MAX_NUMNODES-1] = NULL};
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
extern int wakeup_secondary_cpu_via_init(int phys_apicid, unsigned long start_eip);

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

static inline void smpboot_setup_warm_reset_vector(unsigned long start_eip)
{
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	CMOS_WRITE(0xa, 0xf);
	spin_unlock_irqrestore(&rtc_lock, flags);
	local_flush_tlb();
	*((volatile unsigned short *)phys_to_virt(TRAMPOLINE_PHYS_HIGH)) =
							start_eip >> 4;
	*((volatile unsigned short *)phys_to_virt(TRAMPOLINE_PHYS_LOW)) =
							start_eip & 0xf;
}

static inline void smpboot_restore_warm_reset_vector(void)
{
	unsigned long flags;

	/*
	 * Install writable page 0 entry to set BIOS data area.
	 */
	local_flush_tlb();

	/*
	 * Paranoid:  Set warm reset code and vector here back
	 * to default values.
	 */
	spin_lock_irqsave(&rtc_lock, flags);
	CMOS_WRITE(0, 0xf);
	spin_unlock_irqrestore(&rtc_lock, flags);

	*((volatile u32 *)phys_to_virt(TRAMPOLINE_PHYS_LOW)) = 0;
}

static ssize_t load_elf(const char *filen, char *base)
{
	struct file *file;
	struct elfhdr ehdr;
	struct elf_phdr phdr;

	int i;
	ssize_t sz, pos, bytes;
	loff_t offset;

	if (!filen)
		return -EINVAL;

	file = filp_open(filen, O_RDONLY, 0);
	if (IS_ERR(file)) {
		int rc = PTR_ERR(file);
		pr_err("Unable to open file: %s (%d)\n", filen, rc);
		return rc;
	}

	if (kernel_read(file, 0, (char *) &ehdr, sizeof(ehdr)) != sizeof(ehdr)) {
		pr_err("Unable to load ELF header\n");
		goto out;
	}

	/* Check if this is an ELF file */
	if (memcmp(ehdr.e_ident, ELFMAG, SELFMAG) != 0) {
		pr_err("This is not an ELF binary\n");
		goto out;
	}

	/* Verify that it's really a Hermit app */
	if (ehdr.e_ident[EI_OSABI] != ELFOSABI_HERMIT) {
		pr_err("This is not a Hermit application\n");
		goto out;
	}

	if (ehdr.e_phoff == 0) {
		pr_err("Missing ELF program headers\n");
		goto out;
	}

	if (!elf_check_arch(&ehdr)) {
		pr_err("Hermit only supports x86 at the moment\n");
		goto out;
	}

	for (i=0; i<ehdr.e_phnum; i++) {
		offset = ehdr.e_phoff + i * ehdr.e_phentsize;

		if (kernel_read(file, offset, (char *) &phdr, sizeof(phdr)) != sizeof(phdr))
			goto out;

		if (phdr.p_type == PT_LOAD) {
			offset = phdr.p_offset;
			sz     = phdr.p_filesz;

			goto found;
		}
	}

	pr_err("Missing LOAD segment\n");

out:
	filp_close(file, NULL);

	return -EIO;

found:
	pr_debug("Loading HermitCore's kernel image with a size of %d KiB\n", (int) sz >> 10);

	/* Load kernel to hermit_base */
	for (pos=0; pos<sz; pos+=bytes) {
		bytes = kernel_read(file, offset, base + pos, sz - pos);
		if (bytes == 0)
			break;
		if (bytes < 0)
			goto out;
	}

	filp_close(file, NULL);

	return sz;
}

/*
 * Wake up a core and boot HermitCore on it
 */
static int boot_hermit_core(int cpu, int isle, int cpu_counter, int total_cpus)
{
	unsigned long flags;
	unsigned int start_eip;
	int apicid = apic->cpu_present_to_apicid(cpu);

	if (cpu_online(cpu)) {
		pr_debug("Shutdown cpu %d\n", cpu);
		cpu_down(cpu);
	}

	pr_debug("Isle %d tries to boot HermitCore on CPU %d (apicid %d, boot code size 0x%zx)\n", isle, cpu, apicid, sizeof(boot_code));

	if (!hermit_trampoline)
		return -EINVAL;

	start_eip = (unsigned int) virt_to_phys(hermit_trampoline);

	if (!hermit_base[isle])
		return -ENOMEM;

	if (apic_read(APIC_ICR) & APIC_ICR_BUSY) {
		pr_debug("ERROR: previous send not complete");
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
	*((uint32_t*) (hermit_trampoline+0x183)) += start_eip;
	*((uint32_t*) (hermit_trampoline+0x192)) += start_eip;

	if (!cpu_counter) {
		ssize_t sz;

		sz = load_elf(path2hermit[isle], hermit_base[isle]);
		if (sz < 0)
			return -EINVAL;

		pr_debug("CPU: %d MHz, TSC: %d MHz\n", cpu_khz / 1000, tsc_khz / 1000);

		/*
		 * set base, limit and cpu frequency in HermitCore's kernel
		 */
		*((uint64_t*) (hermit_base[isle] + 0x08)) = (uint64_t) virt_to_phys(hermit_base[isle]);
		*((uint64_t*) (hermit_base[isle] + 0x10)) = (uint64_t) virt_to_phys(hermit_base[isle]) + pool_size / num_possible_nodes();
		*((uint32_t*) (hermit_base[isle] + 0x18)) = tsc_khz / 1000;
		*((uint32_t*) (hermit_base[isle] + 0x1C)) = apicid;
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
		if (x2apic_enabled())
			*((uint32_t*) (hermit_base[isle] + 0x74)) = 0;
	}

	*((uint32_t*) (hermit_base[isle] + 0x30)) = apicid;

#if 1
	// reuse wakeup code from Linux
	local_irq_save(flags);
	wakeup_secondary_cpu_via_init(apicid, start_eip);
	local_irq_restore(flags);

	cpu_counter++;
	while(*((volatile uint32_t*) (hermit_base[isle] + 0x20)) < cpu_counter) { cpu_relax(); }
#else
	smpboot_setup_warm_reset_vector(start_eip);
	smp_mb();

	/*
	 * use the universal startup algorithm of Intel's MultiProcessor Specification
	 */
	apic_icr_write(APIC_INT_LEVELTRIG|APIC_INT_ASSERT|APIC_DM_INIT, apicid);
	udelay(200);
	/* reset INIT */
	apic_icr_write(APIC_INT_LEVELTRIG|APIC_DM_INIT, apicid);
	udelay(10000);
	/* send out the startup */
	apic_icr_write(APIC_DM_STARTUP|(start_eip >> 12), apicid);
	udelay(200);
	/* do it again */
	apic_icr_write(APIC_DM_STARTUP|(start_eip >> 12), apicid);
	udelay(200);

	cpu_counter++;
	while(*((volatile uint32_t*) (hermit_base[isle] + 0x20)) < cpu_counter) { cpu_relax(); }

	smpboot_restore_warm_reset_vector();
#endif

	return 0;
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

static inline int apic_send_ipi(int dest, uint8_t irq)
{
	int apicid = apic->cpu_present_to_apicid(dest);

	apic_icr_write(APIC_INT_ASSERT|APIC_DM_FIXED|irq, apicid);

        return 0;
}

static inline int shutdown_hermit_core(unsigned cpu)
{
	return apic_send_ipi(cpu, 81+32);
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
		pr_debug("Try to shutdown HermitCore on isle %d\n", isle);

		arch_spin_lock(&boot_lock);

		// Ok, we have to shutdown the isle
		for(i=0; i<NR_CPUS; i++)  {
			if (hcpu_online[i] == isle)
				shutdown_hermit_core(i);
		}

		while (*((volatile uint32_t*) (hermit_base[isle] + 0x20)) > 0) {
			cpu_relax();
		}

		// be sure that the processors are in halted state
		udelay(1000);

		// check if the system is down
		for(i=0; i<NR_CPUS; i++)  {
			if (hcpu_online[i] == isle) {
				int ret;

				hcpu_online[i] = -1;
				ret = cpu_up(i);

				if (ret)
					pr_notice("HermitCore isn't able to restart Linux! cpu_up(%d) returns %d\n", i, ret);
			}
		}

		isle_counter--;
		if (isle_counter == 0)
			mmnif_set_carrier(false);

		arch_spin_unlock(&boot_lock);

		pr_debug("HermitCore %d is down!\n", isle);

		return count;
	}

	pr_debug("Try to boot HermitCore on isle %d (%d cpus)\n", isle, cpus[0]);

	/* Do Linux or HermitCore already use the CPUs? */
	for(i=0; i<possible_cpus; i++) {
		if ((cpus[i+1] < 0) || (cpus[i+1] >= NR_CPUS))
			return -EINVAL;
		if (hcpu_online[cpus[i+1]] >= 0)
			return -EBUSY;
	}

	/* Do HermitCore already use the isle? */
	for(i=0; i<NR_CPUS; i++)  {
		if (hcpu_online[i] == isle)
			return -EBUSY;
	}

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

	pr_debug("HermitCore requires %lld ms (%lld ticks) to boot the system\n", (tick1-tick0) / cpu_khz, tick1-tick0);

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
	int isle = MAX_NUMNODES;
	char* path = kobject_get_path(kobj, GFP_KERNEL);

	if (!path)
		return -EINVAL;

	sscanf(path, "/hermit/isle%d", &isle);

	kfree(path);

	if ((isle >= 0) && (isle < MAX_NUMNODES)) {
		if (path2hermit[isle])
			return sprintf(buf, "%s\n", path2hermit[isle]);
		else
			return sprintf(buf, "\n");
	}

	return -EINVAL;
}

/*
 * Set path to the kernel
 */
static ssize_t hermit_set_path(struct kobject *kobj, struct kobj_attribute *attr,
                                const char *buf, size_t count)
{
	int isle = MAX_NUMNODES;
	char* path = kobject_get_path(kobj, GFP_KERNEL);

	if (!path)
		return -EINVAL;

	sscanf(path, "/hermit/isle%d", &isle);

	kfree(path);

	if ((isle >= 0) && (isle < MAX_NUMNODES)) {
		if (path2hermit[isle])
			kfree(path2hermit[isle]);

		path2hermit[isle] = kmalloc(strlen(buf)+1, GFP_KERNEL);
		if (!path2hermit[isle])
			return -ENOMEM;

		return sprintf(path2hermit[isle], "%s", buf);
	}

	return -EINVAL;
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
 * Shut down all CPUs    : echo -1 > /sys/hermit/isleX/cpus
 * Start address         : cat /sys/hermit/isleX/base
 * Show log messages     : cat /sys/hermit/isleX/log
 * Memory size           : cat /sys/hermit/memsize
 * Set path to the app   : echo "/demo" > /sys/hermit/isleX/path
 * Get path to the app   : cat /sys/hermit/isleX/path
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
	&path_attribute.attr,
	NULL
};

static struct attribute * hermit_attrs[] = {
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
	unsigned long long msr;

	if (!enable_hermit)
		return 0;

	pr_notice("Initialize HermitCore\n");
	pr_notice("HermitCore trampoline at 0x%p (0x%zx)\n", hermit_trampoline, (size_t) virt_to_phys(hermit_trampoline));
	pr_notice("Number of available nodes: %d\n", num_possible_nodes());
	pr_notice("Pool size: 0x%zx KiB\n", pool_size / 1024);

	rdmsrl(MSR_IA32_MISC_ENABLE, msr);
	pr_notice("MSR_IA32_MISC_ENABLE: 0x%llx\n", msr);

	mem = memblock_find_in_range(1 << 15, 1 << 23, heap_size * (num_possible_nodes() + 1), PAGE_SIZE);
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

	mem = memblock_find_in_range(1 << 15, 1 << 23, header_size * (num_possible_nodes() + 1), PAGE_SIZE);
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

	mem = memblock_find_in_range(1 << 15, 1 << 23, L1_CACHE_BYTES * (num_possible_nodes() + 1), PAGE_SIZE);
	if (!mem) {
		ret = -ENOMEM;
		goto _exit;
	}

	ret = memblock_reserve(mem, L1_CACHE_BYTES * (num_possible_nodes() + 1));
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

	ret = memblock_reserve(mem, PAGE_SIZE);
	if (ret) {
		ret = -ENOMEM;
		goto _exit;
	}

	phy_rcce_internals = (void*) mem;
	pr_notice("RCCE infos are mapped at 0x%p\n", phy_rcce_internals);

        // init RCCE locks
	tmp = (int*) phys_to_virt(mem);
	memset(tmp, 0x00, L1_CACHE_BYTES * (num_possible_nodes() + 1));
	tmp[1] = 1;

	/* allocate for each HermitCore instance */
	for(i=0; i<num_possible_nodes(); i++) {
		mem = memblock_find_in_range_node(pool_size / num_possible_nodes(), 2 << 20, 4 << 20, MEMBLOCK_ALLOC_ACCESSIBLE, i, flags);
		//mem = memblock_find_in_range(4 << 20, 100<<20, CONFIG_HERMIT_SIZE, 2 << 20);
		if (!mem) {
			pr_notice("HermitCore is not able to find a block of 0x%zx KiB\n", (pool_size / num_possible_nodes()) / 1024);
			ret = -ENOMEM;
			goto _exit;
		}

		ret = memblock_reserve(mem, pool_size / num_possible_nodes());
		if (ret) {
			pr_notice("HermitCore isn't able to reserve block\n");
			ret = -ENOMEM;
			goto _exit;
		}

		hermit_base[i] = (char*) phys_to_virt(mem);
		pr_notice("HermitCore %d at 0x%p (0x%zx)\n", i, hermit_base[i], (size_t) mem);
	}

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

/* trigger an interrupt on the remote processor
 * so he knows there is a packet to read
 */
int hermit_trigger_irq(int dest)
{
	int i, cpu = -1;

	if (dest < 1)
	 	return -EINVAL;

	arch_spin_lock(&boot_lock);

	for(i=0; (cpu == -1) && (i<NR_CPUS); i++) {
		if (hcpu_online[i] == dest-1)
			cpu = i;
	}

	arch_spin_unlock(&boot_lock);

	if (cpu == -1)
		return -EINVAL;

	return apic_send_ipi(cpu, MMNIF_IRQ);
}
