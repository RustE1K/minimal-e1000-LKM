#include <minimal_e1000.h>

MODULE_AUTHOR("Madison Lester and Jesse Wei");
MODULE_DESCRIPTION("Minimal Intel(R) PRO/1000 Network Driver");
MODULE_LICENSE("GPL v2");

char e1000_driver_name[] = "e1000";

/* e1000_pci_tbl - PCI Device ID Table
 *
 * Last entry must be all 0s
 */
static const struct pci_device_id e1000_pci_tbl[] = {
	PCI_DEVICE(PCI_VENDOR_NETWORK, PCI_PRODUCT_NETWORK),
	{0, 0, 0},
};

MODULE_DEVICE_TABLE(pci, e1000_pci_tbl);

struct tx_desc txq[NTXDESC] __attribute__ ((aligned (16)));
struct packet tx_pkts[NTXDESC];
struct rx_desc rxq[NRXDESC] __attribute__ ((aligned (16)));
struct packet rx_pkts[NRXDESC];
volatile uint32_t *network_regs;
uint16_t mac[E1000_NUM_MAC_WORDS];

static struct pci_driver e1000_driver = {
	.name =         e1000_driver_name,
	.id_table =     e1000_pci_table,
	.probe =        e1000_probe, // TODO
	.remove =       e1000_remove, // TODO
	.shutdown =     e1000_shutdown, // TODO
};

static const struct net_device_ops e1000_netdev_ops = {
	.ndo_open		= e1000_open, // TODO
	.ndo_stop		= e1000_close, // TODO
    .ndo_start_xmit = e1000_transmit,
};

/**
 * e1000_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in e1000_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * e1000_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct e1000_adapter *adapter = NULL;
	struct e1000_hw *hw;

	static int cards_found;
	static int global_quad_port_a; /* global ksp3 port a indication */
	int i, err, pci_using_dac;
	u16 eeprom_data = 0;
	u16 tmp = 0;
	u16 eeprom_apme_mask = E1000_EEPROM_APME;
	int bars, need_ioport;
	bool disable_dev = false;

	/* QEMU emulates 82540EM, need ioport bars */
	need_ioport = true;
	if (need_ioport) {
		bars = pci_select_bars(pdev, IORESOURCE_MEM | IORESOURCE_IO);
		err = pci_enable_device(pdev);
	}
	if (err)
		return err;

	err = pci_request_selected_regions(pdev, bars, e1000_driver_name);
	if (err)
		goto err_pci_reg;

	pci_set_master(pdev);
	err = pci_save_state(pdev);
	if (err)
		goto err_alloc_etherdev;

	err = -ENOMEM;
	netdev = alloc_etherdev(sizeof(struct e1000_adapter));
	if (!netdev)
		goto err_alloc_etherdev;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);
	adapter->bars = bars;
	adapter->need_ioport = need_ioport;

	hw = &adapter->hw;
	hw->back = adapter;

	err = -EIO;
	hw->hw_addr = pci_ioremap_bar(pdev, BAR_0);
	if (!hw->hw_addr)
		goto err_ioremap;

	if (adapter->need_ioport) {
		for (i = BAR_1; i < PCI_STD_NUM_BARS; i++) {
			if (pci_resource_len(pdev, i) == 0)
				continue;
			if (pci_resource_flags(pdev, i) & IORESOURCE_IO) {
				hw->io_base = pci_resource_start(pdev, i);
				break;
			}
		}
	}

	/* make ready for any if (hw->...) below */
	err = e1000_init_hw_struct(adapter, hw);
	if (err)
		goto err_sw_init;

	/* there is a workaround being applied below that limits
	 * 64-bit DMA addresses to 64-bit hardware.  There are some
	 * 32-bit adapters that Tx hang when given 64-bit DMA addresses
	 */
	pci_using_dac = 0;
	if ((hw->bus_type == e1000_bus_type_pcix) &&
	    !dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
		pci_using_dac = 1;
	} else {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			pr_err("No usable DMA config, aborting\n");
			goto err_dma;
		}
	}

	netdev->netdev_ops = &e1000_netdev_ops;
	e1000_set_ethtool_ops(netdev);
	netdev->watchdog_timeo = 5 * HZ;
	netif_napi_add(netdev, &adapter->napi, e1000_clean);

	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

	adapter->bd_number = cards_found;

	/* setup the private structure */

	err = e1000_sw_init(adapter);
	if (err)
		goto err_sw_init;

	err = -EIO;
	if (hw->mac_type == e1000_ce4100) {
		hw->ce4100_gbe_mdio_base_virt =
					ioremap(pci_resource_start(pdev, BAR_1),
						pci_resource_len(pdev, BAR_1));

		if (!hw->ce4100_gbe_mdio_base_virt)
			goto err_mdio_ioremap;
	}

	if (hw->mac_type >= e1000_82543) {
		netdev->hw_features = NETIF_F_SG |
				   NETIF_F_HW_CSUM |
				   NETIF_F_HW_VLAN_CTAG_RX;
		netdev->features = NETIF_F_HW_VLAN_CTAG_TX |
				   NETIF_F_HW_VLAN_CTAG_FILTER;
	}

	if ((hw->mac_type >= e1000_82544) &&
	   (hw->mac_type != e1000_82547))
		netdev->hw_features |= NETIF_F_TSO;

	netdev->priv_flags |= IFF_SUPP_NOFCS;

	netdev->features |= netdev->hw_features;
	netdev->hw_features |= (NETIF_F_RXCSUM |
				NETIF_F_RXALL |
				NETIF_F_RXFCS);

	if (pci_using_dac) {
		netdev->features |= NETIF_F_HIGHDMA;
		netdev->vlan_features |= NETIF_F_HIGHDMA;
	}

	netdev->vlan_features |= (NETIF_F_TSO |
				  NETIF_F_HW_CSUM |
				  NETIF_F_SG);

	/* Do not set IFF_UNICAST_FLT for VMWare's 82545EM */
	if (hw->device_id != E1000_DEV_ID_82545EM_COPPER ||
	    hw->subsystem_vendor_id != PCI_VENDOR_ID_VMWARE)
		netdev->priv_flags |= IFF_UNICAST_FLT;

	/* MTU range: 46 - 16110 */
	netdev->min_mtu = ETH_ZLEN - ETH_HLEN;
	netdev->max_mtu = MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);

	adapter->en_mng_pt = e1000_enable_mng_pass_thru(hw);

	/* initialize eeprom parameters */
	if (e1000_init_eeprom_params(hw)) {
		e_err(probe, "EEPROM initialization failed\n");
		goto err_eeprom;
	}

	/* before reading the EEPROM, reset the controller to
	 * put the device in a known good starting state
	 */

	e1000_reset_hw(hw);

	/* make sure the EEPROM is good */
	if (e1000_validate_eeprom_checksum(hw) < 0) {
		e_err(probe, "The EEPROM Checksum Is Not Valid\n");
		e1000_dump_eeprom(adapter);
		/* set MAC address to all zeroes to invalidate and temporary
		 * disable this device for the user. This blocks regular
		 * traffic while still permitting ethtool ioctls from reaching
		 * the hardware as well as allowing the user to run the
		 * interface after manually setting a hw addr using
		 * `ip set address`
		 */
		memset(hw->mac_addr, 0, netdev->addr_len);
	} else {
		/* copy the MAC address out of the EEPROM */
		if (e1000_read_mac_addr(hw))
			e_err(probe, "EEPROM Read Error\n");
	}
	/* don't block initialization here due to bad MAC address */
	eth_hw_addr_set(netdev, hw->mac_addr);

	if (!is_valid_ether_addr(netdev->dev_addr))
		e_err(probe, "Invalid MAC Address\n");


	INIT_DELAYED_WORK(&adapter->watchdog_task, e1000_watchdog);
	INIT_DELAYED_WORK(&adapter->fifo_stall_task,
			  e1000_82547_tx_fifo_stall_task);
	INIT_DELAYED_WORK(&adapter->phy_info_task, e1000_update_phy_info_task);
	INIT_WORK(&adapter->reset_task, e1000_reset_task);

	e1000_check_options(adapter);

	/* Initial Wake on LAN setting
	 * If APM wake is enabled in the EEPROM,
	 * enable the ACPI Magic Packet filter
	 */

	switch (hw->mac_type) {
	case e1000_82542_rev2_0:
	case e1000_82542_rev2_1:
	case e1000_82543:
		break;
	case e1000_82544:
		e1000_read_eeprom(hw,
			EEPROM_INIT_CONTROL2_REG, 1, &eeprom_data);
		eeprom_apme_mask = E1000_EEPROM_82544_APM;
		break;
	case e1000_82546:
	case e1000_82546_rev_3:
		if (er32(STATUS) & E1000_STATUS_FUNC_1) {
			e1000_read_eeprom(hw,
				EEPROM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		}
		fallthrough;
	default:
		e1000_read_eeprom(hw,
			EEPROM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & eeprom_apme_mask)
		adapter->eeprom_wol |= E1000_WUFC_MAG;

	/* now that we have the eeprom settings, apply the special cases
	 * where the eeprom may be wrong or the board simply won't support
	 * wake on lan on a particular port
	 */
	switch (pdev->device) {
	case E1000_DEV_ID_82546GB_PCIE:
		adapter->eeprom_wol = 0;
		break;
	case E1000_DEV_ID_82546EB_FIBER:
	case E1000_DEV_ID_82546GB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting
		 */
		if (er32(STATUS) & E1000_STATUS_FUNC_1)
			adapter->eeprom_wol = 0;
		break;
	case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
		/* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0)
			adapter->eeprom_wol = 0;
		else
			adapter->quad_port_a = true;
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4)
			global_quad_port_a = 0;
		break;
	}

	/* initialize the wol settings based on the eeprom settings */
	adapter->wol = adapter->eeprom_wol;
	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	/* Auto detect PHY address */
	if (hw->mac_type == e1000_ce4100) {
		for (i = 0; i < 32; i++) {
			hw->phy_addr = i;
			e1000_read_phy_reg(hw, PHY_ID2, &tmp);

			if (tmp != 0 && tmp != 0xFF)
				break;
		}

		if (i >= 32)
			goto err_eeprom;
	}

	/* reset the hardware with the new settings */
	e1000_reset(adapter);

	strcpy(netdev->name, "eth%d");
	err = register_netdev(netdev);
	if (err)
		goto err_register;

	e1000_vlan_filter_on_off(adapter, false);

	/* print bus type/speed/width info */
	e_info(probe, "(PCI%s:%dMHz:%d-bit) %pM\n",
	       ((hw->bus_type == e1000_bus_type_pcix) ? "-X" : ""),
	       ((hw->bus_speed == e1000_bus_speed_133) ? 133 :
		(hw->bus_speed == e1000_bus_speed_120) ? 120 :
		(hw->bus_speed == e1000_bus_speed_100) ? 100 :
		(hw->bus_speed == e1000_bus_speed_66) ? 66 : 33),
	       ((hw->bus_width == e1000_bus_width_64) ? 64 : 32),
	       netdev->dev_addr);

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

	e_info(probe, "Intel(R) PRO/1000 Network Connection\n");

	cards_found++;
	return 0;

	err_register:
	err_eeprom:
		e1000_phy_hw_reset(hw);

		if (hw->flash_address)
			iounmap(hw->flash_address);
		kfree(adapter->tx_ring);
		kfree(adapter->rx_ring);
	err_dma:
	err_sw_init:
	err_mdio_ioremap:
		iounmap(hw->ce4100_gbe_mdio_base_virt);
		iounmap(hw->hw_addr);
	err_ioremap:
		disable_dev = !test_and_set_bit(__E1000_DISABLED, &adapter->flags);
		free_netdev(netdev);
	err_alloc_etherdev:
		pci_release_selected_regions(pdev, bars);
	err_pci_reg:
		if (!adapter || disable_dev)
			pci_disable_device(pdev);
		return err;
}

int pci_network_attach(struct pci_func *pcif)
{
	pci_func_enable(pcif); // !!!
	network_regs = mmio_map_region((physaddr_t) pcif->reg_base[0], pcif->reg_size[0]); // !!!
	cprintf("DEV_STAT: 0x%08x\n", network_regs[E1000_STATUS]); // check device status register
	read_mac_from_eeprom();

	// TX init

	network_regs[E1000_TDBAL] = PADDR(txq);
	network_regs[E1000_TDBAH] = 0x00000000;

	network_regs[E1000_TDLEN] = NTXDESC * sizeof(struct tx_desc);

	network_regs[E1000_TDH] = 0x00000000;
	network_regs[E1000_TDT] = 0x00000000;
	
	network_regs[E1000_TCTL] |= E1000_TCTL_EN;
	network_regs[E1000_TCTL] |= E1000_TCTL_PSP;
	network_regs[E1000_TCTL] |= E1000_TCTL_CT;
	network_regs[E1000_TCTL] |= E1000_TCTL_COLD;

	network_regs[E1000_TIPG] |= (0xA << E1000_TIPG_IPGT);
	network_regs[E1000_TIPG] |= (0x8 << E1000_TIPG_IPGR1);
	network_regs[E1000_TIPG] |= (0xC << E1000_TIPG_IPGR2);
	
	memset(txq, 0, sizeof(struct tx_desc) * NTXDESC);

	int i;
	for (i = 0; i < NTXDESC; i++) {
		txq[i].addr = PADDR(&tx_pkts[i]); 	
		txq[i].cmd |= E1000_TXD_RS;			
		txq[i].cmd &= ~E1000_TXD_DEXT;		
		txq[i].status |= E1000_TXD_DD;		
											
	}
	
	// RX init

	network_regs[E1000_RAL] = 0x0;
	network_regs[E1000_RAL] |= mac[0];
	network_regs[E1000_RAL] |= (mac[1] << E1000_EERD_DATA);

	network_regs[E1000_RAH] = 0x0;
	network_regs[E1000_RAH] |= mac[2];
	network_regs[E1000_RAH] |= E1000_RAH_AV;

	for (i = 0; i < NELEM_MTA; i++) {
		network_regs[E1000_MTA + i] = 0x00000000;
	}

	network_regs[E1000_RDBAL] = PADDR(&rxq);
	network_regs[E1000_RDBAH] = 0x00000000;

	network_regs[E1000_RDLEN] = NRXDESC * sizeof(struct rx_desc);

	network_regs[E1000_RDH] = 0;
	network_regs[E1000_RDT] = NRXDESC - 1;

	memset(rxq, 0, sizeof(struct rx_desc) * NRXDESC);

	for (i = 0; i < NRXDESC; i++) {
		rxq[i].addr = PADDR(&rx_pkts[i]); 	// set packet buffer addr
	}

	network_regs[E1000_IMS] |= E1000_RXT0;
	network_regs[E1000_RCTL] &= E1000_RCTL_LBM_NO;
	network_regs[E1000_RCTL] &= E1000_RCTL_BSIZE_2048;
	network_regs[E1000_RCTL] |= E1000_RCTL_SECRC;
	network_regs[E1000_RCTL] &= E1000_RCTL_LPE_NO;
	network_regs[E1000_RCTL] |= E1000_RCTL_EN;
	
	return 0;
}

int e1000_transmit(char* pkt, size_t length)
{
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

int e1000_receive(char* pkt, size_t *length)
{
	size_t tail_idx = (network_regs[E1000_RDT] + 1) % NRXDESC;

	if ((rxq[tail_idx].status & E1000_RXD_STATUS_DD) == 0)
		return -E_E1000_RXBUF_EMPTY;

	if ((rxq[tail_idx].status & E1000_RXD_STATUS_EOP) == 0)
		panic("e1000_receive: EOP flag not set, all packets should fit in one buffer\n");

	*length = rxq[tail_idx].length;
	// Original: memmove(pkt, &rx_pkts[tail_idx], *length);
	// Note 1: not sure if kernel has memmove, so I changed to memcpy
	// Note 2: memmove can handle overlapping memory, memcpy cannot -- might cause error
	memcpy(pkt, &rx_pkts[tail_idx], *length);

	rxq[tail_idx].status &= ~(E1000_RXD_STATUS_DD);
	rxq[tail_idx].status &= ~(E1000_RXD_STATUS_EOP);

	network_regs[E1000_RDT] = tail_idx;

	return 0;
}

// TODO
void clear_e1000_interrupt(void)
{
	network_regs[E1000_ICR] |= E1000_RXT0;
	lapic_eoi(); // ??
	irq_eoi(); // ??
}

// TODO
void e1000_trap_handler(void)
{
	struct Env *receiver = NULL;
	int i;

	for (i = 0; i < NENV; i++) {
		if (envs[i].env_e1000_waiting_rx)
			receiver = &envs[i];
	}

	if (!receiver) {
		clear_e1000_interrupt();
		return;
	}
	else {
		receiver->env_status = ENV_RUNNABLE;
		receiver->env_e1000_waiting_rx = false;
		clear_e1000_interrupt();
		return;
	}
}

void read_mac_from_eeprom(void)
{
	uint8_t word_num;
	for (word_num = 0; word_num < E1000_NUM_MAC_WORDS; word_num++) {
		network_regs[E1000_EERD] |= (word_num << E1000_EERD_ADDR);
		network_regs[E1000_EERD] |= E1000_EERD_READ;
		while (!(network_regs[E1000_EERD] & E1000_EERD_DONE));
		mac[word_num] = network_regs[E1000_EERD] >> E1000_EERD_DATA;
		network_regs[E1000_EERD] = 0x0;
	}
}

void e1000_get_mac(uint8_t *mac_addr)
{
	*((uint32_t *) mac_addr) =  (uint32_t) network_regs[E1000_RAL];
	*((uint16_t*)(mac_addr + 4)) = (uint16_t) network_regs[E1000_RAH];
}

void print_mac()
{
	printk(KERN_INFO "MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0] & 0x00FF, mac[0] >> 8, mac[1] & 0x00FF, mac[1] >> 8, mac[2] & 0xFF, mac[2] >> 8);
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