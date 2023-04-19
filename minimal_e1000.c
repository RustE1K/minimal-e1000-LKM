/* Issues:
* - pci_alloc_consistent() DNE -- use dma_alloc_coherent()?
*/

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/init.h>

MODULE_AUTHOR("Madison Lester");
MODULE_DESCRIPTION("Minimal Intel(R) PRO/1000 Network Driver");
MODULE_LICENSE("GPL v2");

#define DRIVER_NAME "e1000"
#define E1000_VENDOR_ID 0x8086
#define E1000_DEVICE_ID 0x100e

#define NTXDESC 64
#define PKT_BUF_SIZE  1536  // minimum 1518, 16-byte aligned for performance
#define TOTAL_TX_BUF_SIZE  (PKT_BUF_SIZE * NTXDESC)

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

static int e1000_open(struct net_device *dev);
static int e1000_stop(struct net_device *dev);
static int e1000_start_xmit(struct sk_buff *skb, struct net_device *dev);
static struct net_device_stats* e1000_get_stats(struct net_device *dev);

static const struct net_device_ops e1000_netdev_ops = {
	.ndo_open		= e1000_open,
	.ndo_stop		= e1000_stop,
	.ndo_start_xmit		= e1000_start_xmit,
	.ndo_get_stats	= e1000_get_stats,
};

static const struct pci_device_id e1000_id_table[] = {
    {PCI_DEVICE(E1000_VENDOR_ID, E1000_DEVICE_ID)},
    {0, },
};

static struct net_device *e1000_dev;

static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    unsigned long mmio_start, mmio_end, mmio_len, mmio_flags;
    void *ioaddr;
    struct e1000_private *tp;
    int i;

    dev = alloc_etherdev(sizeof(struct e1000_private));
    if(!dev) {
        printk(KERN_INFO "E1000 ERROR: could not allocate etherdev\n");
        return -1;
    }

    tp = netdev_priv(dev);
    tp->pci_dev = pdev;
    tp = netdev_priv(e1000_dev); /* e1000 private information */
    
    /* get PCI memory mapped I/O space base address from BAR1 */
    mmio_start = pci_resource_start(pdev, 1);
    mmio_end = pci_resource_end(pdev, 1);
    mmio_len = pci_resource_len(pdev, 1);
    mmio_flags = pci_resource_flags(pdev, 1);

    /* make sure above region is MMI/O */
    if(!(mmio_flags & IORESOURCE_MEM)) {
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
        // e1000_dev->dev_addr[i] = readb(e1000_dev->base_addr+i);
        e1000_dev->broadcast[i] = 0xff;
    }
    e1000_dev->hard_header_len = 14;

    memcpy(e1000_dev->name, DRIVER_NAME, sizeof(DRIVER_NAME)); /* Device Name */
    e1000_dev->irq = pdev->irq;  /* Interrupt Number */
    e1000_dev->netdev_ops = &e1000_netdev_ops;

    /* register the device */
    if (register_netdev(e1000_dev)) {
        printk(KERN_INFO "E1000 ERROR: could not register netdevice\n");
        return 0;
    }

    return 0;
}

static void e1000_tx_init(struct net_device *dev)
{
    struct e1000_private *tp = netdev_priv(dev);
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
    struct e1000_private *tp = netdev_priv(dev);

    /* get memory for Tx buffers
    * memory must be DMAable
    */
    **(tp->tx_buf) = pci_alloc_consistent(tp->pci_dev, TOTAL_TX_BUF_SIZE, &tp->tx_bufs_dma);
    
    if(!tp->tx_buf) {
        free_irq(dev->irq, dev);
        return -ENOMEM;
    }

    e1000_tx_init(dev);
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

    struct e1000_private *tp = netdev_priv(dev);
    void *ioaddr = tp->mmio_addr;
    struct tx_desc *txq = tp->txq;
    size_t len = skb->len;

    if (len > PKT_BUF_SIZE)
		panic("e1000_transmit: size of packet to transmit (%ld) larger than max (1518)\n", len);

	size_t tail_idx = readl(ioaddr + E1000_TDT);

	if (txq[tail_idx].status & E1000_TXD_DD) {
        skb_copy_and_csum_dev(skb, tp->tx_buf[tail_idx]);
        dev_kfree_skb(skb);
		txq[tail_idx].status &= ~E1000_TXD_DD;
		txq[tail_idx].cmd |= E1000_TXD_EOP;
		txq[tail_idx].length = len;
        writel((tail_idx + 1) % NTXDESC, ioaddr + E1000_TDT);
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

static struct pci_driver e1000_driver = {
	.name =         DRIVER_NAME,
    .id_table =     e1000_id_table,
	.probe =        e1000_probe,
};

int __init e1000_init_module(void) 
{
    return pci_register_driver(&e1000_driver);;
}

void __exit e1000_cleanup_module(void) 
{
    struct e1000_private *tp;
    tp = netdev_priv(e1000_dev);

    iounmap(tp->mmio_addr);
    pci_release_regions(tp->pci_dev);

    unregister_netdev(e1000_dev);
    pci_disable_device(tp->pci_dev);
    pci_unregister_driver(&e1000_driver);
    return;
}

module_init(e1000_init_module);
module_exit(e1000_cleanup_module);