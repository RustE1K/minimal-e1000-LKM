#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/capability.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/pkt_sched.h>
#include <linux/list.h>
#include <linux/reboot.h>
#include <net/checksum.h>
#include <linux/mii.h>
#include <linux/init.h>
// TODO: remove excess imports

MODULE_AUTHOR("Madison Lester");
MODULE_DESCRIPTION("Minimal Intel(R) PRO/1000 Network Driver");
MODULE_LICENSE("GPL v2");

#define DRIVER_NAME "e1000"
#define E1000_VENDOR_ID 0x8086
#define E1000_DEVICE_ID 0x100e

#define NTXDESC 64
#define PKT_BUF_SIZE  1536  // minimum 1518, 16-byte aligned for performance
#define TOTAL_TX_BUF_SIZE  (PKT_BUF_SIZE * NUM_TX_SIZE)

/* Register Set. (82543, 82544)
 *
 * Registers are defined to be 32 bits and  should be accessed as 32 bit values.
 * These registers are physically located on the NIC, but are mapped into the
 * host memory address space.
 *
 * RW - register is both readable and writable
 * RO - register is read only
 * WO - register is write only
 * R/clr - register is read only and is cleared when read
 * A - register array
 *
 * All registers are divided by 4 so they can be used as uint32_t[] indices.
 */
#define E1000_CTRL     (0x00000/4)  /* Device Control - RW */
#define E1000_CTRL_DUP (0x00004/4)  /* Device Control Duplicate (Shadow) - RW */
#define E1000_STATUS   (0x00008/4)  /* Device Status - RO */
#define E1000_TDBAL    (0x03800/4)  /* TX Descriptor Base Address Low - RW */
#define E1000_TDBAH    (0x03804/4)  /* TX Descriptor Base Address High - RW */
#define E1000_TDLEN    (0x03808/4)  /* TX Descriptor Length - RW */
#define E1000_TDH      (0x03810/4)  /* TX Descriptor Head - RW */
#define E1000_TDT      (0x03818/4)  /* TX Descripotr Tail - RW */
#define E1000_TCTL     (0x00400/4)  /* TX Control - RW */
#define E1000_TIPG     (0x00410/4)  /* TX Inter-packet gap -RW */
#define E1000_RAL      (0x05400/4)  /* Receive Address Low 32 bits - RW Array */
#define E1000_RAH      (0x05404/4)  /* Receive Address High 32 bits- RW Array */
#define E1000_MTA      (0x05200/4)  /* Multicast Table Array - RW Array */
#define E1000_RDBAL    (0x02800/4)  /* RX Descriptor Base Address Low - RW */
#define E1000_RDBAH    (0x02804/4)  /* RX Descriptor Base Address High - RW */
#define E1000_RDLEN    (0x02808/4)  /* RX Descriptor Length - RW */
#define E1000_RDH      (0x02810/4)  /* RX Descriptor Head - RW */
#define E1000_RDT      (0x02818/4)  /* RX Descriptor Tail - RW */
#define E1000_RCTL     (0x00100/4)  /* RX Control - RW */
#define E1000_IMS      (0x000D0/4)  /* Interrupt Mask Set - RW */
#define E1000_ICR      (0x000C0/4)  /* Interrupt Cause Read - R/clr */
#define E1000_EERD     (0x00014/4)  /* EEPROM Read - RW */

// EEPROM
#define E1000_EERD_ADDR 8 /* num of bit shifts to get to addr section */
#define E1000_EERD_DATA 16 /* num of bit shifts to get to data section */
#define E1000_EERD_DONE 0x00000010 /* 4th bit */
#define E1000_EERD_READ 0x00000001 /* 0th bit */

// Transmission control register bits (TCTL)
#define E1000_TCTL_EN     0x00000002    /* enable tx */
#define E1000_TCTL_PSP    0x00000008    /* pad short packets */
#define E1000_TCTL_CT     0x00000100    /* collision threshold, set to 0x10 */
#define E1000_TCTL_COLD   0x00040000    /* collision distance, set to 0x40 */

// Transmission IPG offset bits (TIPG)
#define E1000_TIPG_IPGT 0
#define E1000_TIPG_IPGR1 10
#define E1000_TIPG_IPGR2 20

// Transmission descriptor bits
#define E1000_TXD_DEXT	0x20 /* bit 5 in CMD section */
#define E1000_TXD_RS	0x8 /* bit 3 in CMD section */
#define E1000_TXD_DD	0x1 /* bit 0 in STATUS section */
#define E1000_TXD_EOP	0x1 /* bit 0 of CMD section */

// MAC address related constants
#define E1000_RAH_AV	0x80000000
#define E1000_NUM_MAC_WORDS 3

// Receive control bits
#define E1000_RCTL_EN           0x00000002 /* enable receiver */
#define E1000_RCTL_LBM_NO       0xffffff3f /* no loopback mode, 6 & 7 bit set to 0 */
#define E1000_RCTL_BSIZE_2048   0xfffcffff /* buffer size at 2048 by setting 16 and 17 bit to 0 */
//#define E1000_RCTL_BSIZE_2048	0x00000000
#define E1000_RCTL_SECRC        0x04000000 /* strip CRC by setting 26 bit to 1 */
#define E1000_RCTL_LPE_NO       0xffffffdf /* disable long packet mode by sett If the E1000 receives a packet that is larger than the packet buffer in one receive descriptor, it will retrieve as many descriptors as necessary from the receive queue to store the entire contents of the packet. To indicate that this has happened, it will set the DD status bit on all of these descriptors, but only set the EOP status bit on the last of these descriptors. You can either deal with this possibility in your driver, or simply configure the card to not accept "long packets" (also known as jumbo frames) and make sure your receive buffers are large enough to store the largest possible standard Ethernet packet (1518 bytes). ing the 5th bit to 0 */

// Receive descriptor status bits
#define E1000_RXD_STATUS_DD		0x00000001
#define E1000_RXD_STATUS_EOP	0x00000002

// Receive Timer Interrupt mask
#define E1000_RXT0	0x00000080 /* 7th bit */

// Transmit descriptor struct. This is the type of each element in the
// transmit queue.
// Note that sizeof(struct tx_desc) = 16 bytes.
struct tx_desc
{
	uint64_t addr;
	uint16_t length;
	uint8_t cso;
	uint8_t cmd;
	uint8_t status;
	uint8_t css;
	uint16_t special;
};

struct e1000_private 
{
    struct pci_dev *pci_dev;  /* PCI device */
    void *mmio_addr; /* memory mapped I/O addr */
    unsigned long regs_len; /* length of I/O or MMI/O region */
    struct tx_desc txq[NTXDESC] __attribute__ ((aligned (16)));
    unsigned char tx_buf[NTXDESC][PKT_BUF_SIZE];
    dma_addr_t tx_bufs_dma;
};

static struct net_device *e1000_dev;

static struct pci_dev* probe_for_e1000(void) 
{
    struct pci_dev *pdev = NULL;

    /* ensure we are not working on a non-PCI system */
    if (!pci_present()) {
        printk(KERN_INFO "E1000 ERROR: PCI not present\n");
        return pdev;
    }

    /* look for E1000 */
    pdev = pci_find_device(E1000_VENDOR_ID, E1000_DEVICE_ID, NULL);
    if (pdev) {
        /* device found, enable it */
        if (pci_enable_device(pdev)) {
            printk(KERN_INFO "E1000 ERROR: could not enable devie\n");
            return NULL;
        }
        else
            printk(KERN_INFO "E1000 enabled\n");
    }
    else {
        printk(KERN_INFO "E1000 ERROR: device not found\n");
        return pdev;
    }
    return pdev;
}

static int e1000_init(struct pci_dev *pdev, struct net_device **dev_out) 
{
    struct net_device *dev;
    struct e1000_private *tp;

    /* 
    * alloc_etherdev allocates memory for dev and dev->priv.
    * dev->priv shall have sizeof(struct e1000_private) memory
    * allocated.
    */
    dev = alloc_etherdev(sizeof(struct e1000_private));
    if(!dev) {
        printk(KERN_INFO "E1000 ERROR: could not allocate etherdev\n");
        return -1;
    }

    tp = dev->priv;
    tp->pci_dev = pdev;
    *dev_out = dev;

    return 0;
}

static void e1000_tx_init()
{
    struct e1000_private *tp = dev->priv;
    void *ioaddr = tp->mmio_addr;

    // initialize tx queue
    int i;
    memset(tp->txq, 0, sizeof(tp->txq));
    for (i = 0; i < NTXDESC; i++) {
        tp->txq[i].addr = tp->tx_buf[i];
        tp->txq[i].cmd |= E1000_TXD_RS;			
		tp->txq[i].cmd &= ~E1000_TXD_DEXT;		
		tp->txq[i].status |= E1000_TXD_DD;
    }

    writel(tp->txq, ioaddr + E1000_TDBAL);
	writel(0x00000000, ioaddr + E1000_TDBAH);

    writel(NTXDESC * sizeof(struct tx_desc), ioaddr + E1000_TDLEN);

    writel(0x00000000, ioaddr + E1000_TDH);
    writel(0x00000000, ioaddr + E1000_TDT);

    unsigned long tctl_bits = 0x00000000;
    tctl_bits |= E1000_TCTL_EN;
    tctl_bits |= E1000_TCTL_PSP;
    tctl_bits |= E1000_TCTL_CT;
    tctl_bits |= E1000_TCTL_COLD;
    writel(tctl_bits, ioaddr + E1000_TCTL);

    unsigned long tipg_bits = 0x00000000;
    tipg_bits |= (0xA << E1000_TIPG_IPGT);
    tipg_bits |= (0x8 << E1000_TIPG_IPGR1);
    tipg_bits |= (0xC << E1000_TIPG_IPGR2);
    writel(tipg_bits, ioaddr + E1000_TIPG);
}

static int e1000_open(struct net_device *dev) 
{
    printk(KERN_INFO "e1000_open is called\n");
    int retval;
    struct e1000_private *tp = dev->priv;

    /* get memory for Tx buffers
    * memory must be DMAable
    */
    tp->tx_buf = pci_alloc_consistent(tp->pci_dev, TOTAL_TX_BUF_SIZE, &tp->tx_bufs_dma);
    
    if(!tp->tx_buf) {
        free_irq(dev->irq, dev);
        return -ENOMEM;
    }

    e1000_tx_init();
    netif_start_queue(dev);
    
    return 0;
}

static int e1000_stop(struct net_device *dev) 
{
    printk(KERN_INFO "e1000_stop is called\n");
    return 0;
}

static int e1000_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    printk(KERN_INFO "e1000_start_xmit is called\n");

    struct e1000_private *tp = dev->priv;
    void *ioaddr = tp->mmio_addr;
    struct tx_desc *txq = tp->txq;
    unsigned int entry = tp->cur_tx;
    size_t len = skb->len;

    if (len > PKT_BUF_SIZE)
		panic("e1000_transmit: size of packet to transmit (%d) larger than max (1518)\n", length);

	size_t tail_idx = readl(ioaddr + E1000_TDT);

	if (txq[tail_idx].status & E1000_TXD_DD) {
        skb_copy_and_csum_dev(skb, tp->tx_buf[tail_idx]);
        dev_kfree_skb(skb);
		txq[tail_idx].status &= ~E1000_TXD_DD;
		txq[tail_idx].cmd |= E1000_TXD_EOP;
		txq[tail_idx].length = len;
        writel((tail_idx + 1) % NTXDESC, ioaddr + E1000_TDT)
		return 0;
	} else {
        netif_stop_queue(dev);
		return -1;
	}
}

static struct net_device_stats* e1000_get_stats(struct net_device *dev) 
{
    printk(KERN_INFO "e1000_get_stats is called\n");
    return 0;
}

int init_module(void) 
{
    struct pci_dev *pdev;
    unsigned long mmio_start, mmio_end, mmio_len, mmio_flags;
    void *ioaddr;
    struct e1000_private *tp;
    int i;

    pdev = probe_for_e1000();
    if(!pdev)
        return 0;

    if(e1000_init(pdev, &e1000_dev)) {
        printk(KERN_INFO "E1000 ERROR: could not initialize device\n");
        return 0;
    }

    tp = e1000_dev->priv; /* e1000 private information */
    
    /* get PCI memory mapped I/O space base address from BAR1 */
    mmio_start = pci_resource_start(pdev, 1);
    mmio_end = pci_resource_end(pdev, 1);
    mmio_len = pci_resource_len(pdev, 1);
    mmio_flags = pci_resource_flags(pdev, 1);

    /* make sure above region is MMI/O */
    if(!(mmio_flags & I/ORESOURCE_MEM)) {
        printk(KERN_INFO "E1000 ERROR: region not MMI/O region\n");
        return 0;
    }
    
    /* get PCI memory space */
    if(pci_request_regions(pdev, DRIVER_NAME)) {
        printk(KERN_INFO "E1000 ERROR: could not get PCI region\n");
        return 0;
    }

    pci_set_master(pdev);

    /* ioremap MMI/O region */
    ioaddr = ioremap(mmio_start, mmio_len);
    if(!ioaddr) {
        printk(KERN_INFO "E1000 ERROR: could not ioremap\n");
        return 0;
    }

    e1000_dev->base_addr = (long) ioaddr;
    tp->mmio_addr = ioaddr;
    tp->regs_len = mmio_len;

    /* UPDATE NET_DEVICE */

    for (i = 0; i < 6; i++) {  /* Hardware Address */
        e1000_dev->dev_addr[i] = readb(e1000_dev->base_addr+i);
        e1000_dev->broadcast[i] = 0xff;
    }
    e1000_dev->hard_header_len = 14;

    memcpy(e1000_dev->name, DRIVER_NAME, sizeof(DRIVER_NAME)); /* Device Name */
    e1000_dev->irq = pdev->irq;  /* Interrupt Number */
    e1000_dev->open = e1000_open;
    e1000_dev->stop = e1000_stop;
    e1000_dev->hard_start_xmit = e1000_start_xmit;
    e1000_dev->get_stats = e1000_get_stats;

    /* register the device */
    if (register_netdev(e1000_dev)) {
        printk(KERN_INFO "E1000 ERROR: could not register netdevice\n");
        return 0;
    }

    return 0;
}

void cleanup_module(void) 
{
    struct e1000_private *tp;
    tp = e1000_dev->priv;

    iounmap(tp->mmio_addr);
    pci_release_regions(tp->pci_dev);

    unregister_netdev(e1000_dev);
    pci_disable_device(tp->pci_dev);
    return;
}

module_init(init_module);
module_exit(cleanup_module);