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

struct e1000_tx_desc {
	__le64 buffer_addr;	/* Address of the descriptor's data buffer */
	union {
		__le32 data;
		struct {
			__le16 length;	/* Data buffer length */
			u8 cso;	/* Checksum offset */
			u8 cmd;	/* Descriptor control */
		} flags;
	} lower;
	union {
		__le32 data;
		struct {
			u8 status;	/* Descriptor status */
			u8 css;	/* Checksum start */
			__le16 special;
		} fields;
	} upper;
};

#define TX_BUF_SIZE 1536  // 16-byte aligned for performance
#define NTXDESC     64

struct e1000_private
{
	__iomem e1000_base;
	static struct e1000_tx_desc e1000_tx_queue[NTXDESC] __attribute__((aligned(16)));
	static uint8_t e1000_tx_buf[NTXDESC][TX_BUF_SIZE];
};

#define E1000_REG(offset)    (*(volatile uint32_t *)(e1000_base + offset))

static int e1000_open(struct net_device *netdev) 
{
    printk(KERN_INFO "e1000_open is called\n");

    struct e1000_private *e1000_private = netdev_priv(netdev);

	netif_carrier_off(netdev);
	netif_start_queue(netdev);

	return 0;
}

static int e1000_stop(struct net_device *netdev) 
{
    printk(KERN_INFO "e1000_stop is called\n");

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

    return 0;
}

static int e1000_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    printk(KERN_INFO "e1000_start_xmit is called\n");

	// TODO
}

static const struct net_device_ops e1000_netdev_ops = {
	.ndo_open		= e1000_open,
	.ndo_stop		= e1000_stop,
	.ndo_start_xmit	= e1000_start_xmit,
};

static const struct pci_device_id e1000_id_table[] = {
    {PCI_DEVICE(E1000_VENDOR_ID, E1000_DEVICE_ID)},
    {0, },
};

static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	printk(KERN_INFO "e1000_probe is called\n");

	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_private e1000_private;

	int bars = pci_select_bars(pdev, IORESOURCE_MEM);
	pci_enable_device_mem(pdev);
	pci_request_selected_regions(pdev, bars, DRIVER_NAME);
	pci_set_master(pdev);
	pci_save_state(pdev);
	
	netdev = alloc_etherdev(sizeof(struct e1000_private));
	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	e1000_private = netdev_priv(netdev);
	e1000_private->e1000_base = pci_ioremap_bar(pdev, 0);
	netdev->netdev_ops = &e1000_netdev_ops;
	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

	netdev->min_mtu = ETH_ZLEN - ETH_HLEN;
	netdev->max_mtu = MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);

	strcpy(netdev->name, "eth%d");
	register_netdev(netdev);

	return 0;
}

static void e1000_remove(struct pci_dev *pdev)
{
	printk(KERN_INFO "e1000_remove is called\n");

	struct net_device *netdev = pci_get_drvdata(pdev);

	if (netdev) {
		struct e1000_private *e1000_private = netdev_priv(netdev);

		unregister_netdev(netdev);
		// TODO: free e1000_private resources
		free_netdev(netdev);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
	}
}

static struct pci_driver e1000_driver = {
	.name =         DRIVER_NAME,
    .id_table =     e1000_id_table,
	.probe =        e1000_probe,
	.remove =       e1000_remove,
};

int __init e1000_init_module(void) 
{
    return pci_register_driver(&e1000_driver);;
}

module_init(e1000_init_module);

void __exit e1000_cleanup_module(void) 
{
    pci_unregister_driver(&e1000_driver);
}

module_exit(e1000_cleanup_module);