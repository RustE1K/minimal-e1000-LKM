#include<linux/module.h>
#include<linux/kernel.h>
#include <linux/init.h>


MODULE_AUTHOR("Madison Lester and Jesse Wei");
MODULE_DESCRIPTION("Minimal Intel(R) PRO/1000 Network Driver");
MODULE_LICENSE("GPL v2");

#define DRV_NAME "e1000"

#define INTEL_E1000_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}

#define PCI_VENDOR_NETWORK 0x8086
#define PCI_PRODUCT_NETWORK 0x100e

#define NTXDESC 32
#define NRXDESC 128

#define NELEM_MTA 128

#define PKT_BUF_SIZE 2048

#define E1000_NUM_MAC_WORDS 3

int pci_network_attach(struct pci_func *pcif);
int e1000_transmit(char *pkt, size_t length);
int e1000_receive(char *pkt, size_t *length);
void e1000_trap_handler(void);
void e1000_get_mac(uint8_t *mac_addr);


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

// Receive descriptor struct. This is the type of each element in the
// receive queue.
// Note that sizeof(struct rx_desc) = 16 bytes.
struct rx_desc
{
	uint64_t addr;
	uint16_t length;
	uint16_t pkt_checksum;
	uint8_t status;
	uint8_t errors;
	uint16_t special;
};

// Packet buffer
struct packet
{
	char buf[PKT_BUF_SIZE];
};

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

struct tx_desc txq[NTXDESC] __attribute__ ((aligned (16)));
struct packet tx_pkts[NTXDESC];
struct rx_desc rxq[NRXDESC] __attribute__ ((aligned (16)));
struct packet rx_pkts[NRXDESC];
volatile uint32_t *network_regs;
uint16_t mac[E1000_NUM_MAC_WORDS];

static struct pci_driver e1000_driver = {
	.name =         DRV_NAME,
	.id_table =     e1000_pci_table,
	.probe =        e1000_probe,
	.remove =       e1000_remove,
	.shutdown =     e1000_shutdown,
};

static const struct net_device_ops e1000_netdev_ops = {
    .ndo_start_xmit = e1000_transmit,
};

int e1000_transmit(char* pkt, size_t length) {
	if (length > PKT_BUF_SIZE)
		panic("e1000_transmit: size of packet to transmit (%d) larger than max (2048)\n", length);

	size_t tail_idx = network_regs[E1000_TDT];

	if (txq[tail_idx].status & E1000_TXD_DD) {
		memmove((void *) &tx_pkts[tail_idx], (void *) pkt, length);
		txq[tail_idx].status &= ~E1000_TXD_DD;
		txq[tail_idx].cmd |= E1000_TXD_EOP;
		txq[tail_idx].length = length;
		network_regs[E1000_TDT] = (tail_idx + 1) % NTXDESC;

		return 0;
	} else {
		return -1;
	}
}

static int __init e1000_init_module(void)
{
    printk(KERN_INFO "Initialize e1000\n");
    return 0;
}

static void __exit e1000_exit_module(void)
{
    printk(KERN_INFO "Exit e1000\n");
}

module_init(e1000_init_module);
module_exit(e1000_exit_module);


/* e1000_pci_tbl - PCI Device ID Table
 *
 * Last entry must be all 0s
 *
 * Macro expands to...
 *   {PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}
 */
static const struct pci_device_id e1000_pci_tbl[] = {
	INTEL_E1000_ETHERNET_DEVICE(0x1000),
	INTEL_E1000_ETHERNET_DEVICE(0x1001),
	INTEL_E1000_ETHERNET_DEVICE(0x1004),
	INTEL_E1000_ETHERNET_DEVICE(0x1008),
	INTEL_E1000_ETHERNET_DEVICE(0x1009),
	INTEL_E1000_ETHERNET_DEVICE(0x100C),
	INTEL_E1000_ETHERNET_DEVICE(0x100D),
	INTEL_E1000_ETHERNET_DEVICE(0x100E),
	INTEL_E1000_ETHERNET_DEVICE(0x100F),
	INTEL_E1000_ETHERNET_DEVICE(0x1010),
	INTEL_E1000_ETHERNET_DEVICE(0x1011),
	INTEL_E1000_ETHERNET_DEVICE(0x1012),
	INTEL_E1000_ETHERNET_DEVICE(0x1013),
	INTEL_E1000_ETHERNET_DEVICE(0x1014),
	INTEL_E1000_ETHERNET_DEVICE(0x1015),
	INTEL_E1000_ETHERNET_DEVICE(0x1016),
	INTEL_E1000_ETHERNET_DEVICE(0x1017),
	INTEL_E1000_ETHERNET_DEVICE(0x1018),
	INTEL_E1000_ETHERNET_DEVICE(0x1019),
	INTEL_E1000_ETHERNET_DEVICE(0x101A),
	INTEL_E1000_ETHERNET_DEVICE(0x101D),
	INTEL_E1000_ETHERNET_DEVICE(0x101E),
	INTEL_E1000_ETHERNET_DEVICE(0x1026),
	INTEL_E1000_ETHERNET_DEVICE(0x1027),
	INTEL_E1000_ETHERNET_DEVICE(0x1028),
	INTEL_E1000_ETHERNET_DEVICE(0x1075),
	INTEL_E1000_ETHERNET_DEVICE(0x1076),
	INTEL_E1000_ETHERNET_DEVICE(0x1077),
	INTEL_E1000_ETHERNET_DEVICE(0x1078),
	INTEL_E1000_ETHERNET_DEVICE(0x1079),
	INTEL_E1000_ETHERNET_DEVICE(0x107A),
	INTEL_E1000_ETHERNET_DEVICE(0x107B),
	INTEL_E1000_ETHERNET_DEVICE(0x107C),
	INTEL_E1000_ETHERNET_DEVICE(0x108A),
	INTEL_E1000_ETHERNET_DEVICE(0x1099),
	INTEL_E1000_ETHERNET_DEVICE(0x10B5),
	INTEL_E1000_ETHERNET_DEVICE(0x2E6E),
	/* required last entry */
	{0,}
};

MODULE_DEVICE_TABLE(pci, e1000_pci_tbl);