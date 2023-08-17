#include "pcie.h"

// semaphores
enum  {
        SEM_READ,
        SEM_WRITE,
        SEM_WRITEREG,
        SEM_READREG,
        SEM_WAITFOR,
        SEM_DMA,
        NUM_SEMS
};

//semaphores
struct semaphore gSem[NUM_SEMS];


typedef struct _cfgrdwr{
	unsigned int reg;
	unsigned int value;
} cfgrdwr;

struct DMA_packet{
	unsigned char buf[4096];
	unsigned int offset;
	unsigned int count;
};

/* ycs >*/

MODULE_LICENSE("Dual BSD/GPL");



/*
static struct pci_device_id jw_pcie_tbl [] __initdata = { 
	{PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX_PCIE, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, 
	{0,} 
};
*/
/*  PCI Device ID Table */
static struct pci_device_id jw_pcie_tbl [] __initdata = { 
	{ PCI_DEVICE(PCIE_VENDOR_ID, PCIE_DEVICE_ID) }, 
	{ 0, },  
};
MODULE_DEVICE_TABLE(pci, jw_pcie_tbl);

//static struct pcie_dev *pcie_devp = NULL;

//Allocate and initiate pcie device
static struct pcie_cdev *alloc_pcie_dev( void )
{
	/* Allocate memory for the per-device structure */
    struct pcie_cdev *pcie_devp = (struct pcie_cdev *)kzalloc(sizeof(struct pcie_cdev), GFP_KERNEL);
    if (!pcie_devp) {
	  return NULL;
    }
	
	return pcie_devp;
}

static void free_pcie_dev( struct pcie_cdev *devp )
{	
	kfree(devp);
	return;
}

//-----------------------------------------------------------------------------
// The probe member of pci_driver
//-----------------------------------------------------------------------------
static int __devinit jw_pcie_probe( struct pci_dev *pdev,
				       const struct pci_device_id *pcie_id )
{
	int rc = 0;
	unsigned long mmio_start, mmio_end, mmio_flags, mmio_len;
	
	struct pcie_cdev *pcie_devp = NULL;


	pcie_devp = alloc_pcie_dev();
	if( NULL == pcie_devp ){
		rc = -ENODEV;
		return rc;
	}
	
	/* enable device (incl. PCI PM wakeup), and bus-mastering */
	rc = pci_enable_device (pdev);
	if (rc){
		goto err_out;
	}

	//used by temp test
	//pci_write_config_dword( pdev, BAR_NUM * 4, 0xF7C00000  );

	/*  读取 PCI 配置信息  */
	mmio_start = pci_resource_start (pdev, BAR_NUM);
	mmio_end = pci_resource_end (pdev, BAR_NUM);
	mmio_flags = pci_resource_flags (pdev, BAR_NUM);
	mmio_len = pci_resource_len (pdev, BAR_NUM);

	/* set this immediately, we need to know before
	* we talk to the chip directly */
	printk("MMIO mmio_start == 0x%08lX\n", mmio_start);
	printk("MMIO mmio_end == 0x%08lX\n", mmio_end);
	printk("MMIO mmio_flags == 0x%08lX\n", mmio_flags);
	printk("MMIO region size == 0x%08lX\n", mmio_len);
	
	/* make sure PCI base addr BAR_NUM is MMIO */
	if (!(mmio_flags & IORESOURCE_MEM)) {
		dev_err(&pdev->dev, "Bar #%d is not an MMIO resource, aborting\n", BAR_NUM);
		rc = -ENODEV;
		goto err_out;
	}

	/* check for weird/broken PCI region reporting */
//	if ( mmio_len < NETDRV_MIN_IO_SIZE ) {
//		dev_err(&pdev->dev, "Invalid PCI region size(s), aborting\n");
//		rc = -ENODEV;
//		goto err_out;
//	}

	rc = pci_request_regions(pdev, MODNAME);
	if (rc){
		goto err_out;
	}
	
	pci_set_master (pdev);
	
	/* ioremap MMIO region */
	pcie_devp->bar_memaddr = ioremap(mmio_start, mmio_len);
	if(pcie_devp->bar_memaddr == NULL) {
		rc = -1;
		goto err_out_unmap;
	}

	//set the member of pcie_devp
	pcie_devp->hwdev = pdev;
	//pcie_devp->xxx
	pcie_devp->bar_base_addr = mmio_start;
	pcie_devp->bar_addr_size = mmio_len;
	pci_set_drvdata(pdev, pcie_devp);
	
	/* register pcie to char device */
	rc = pcie_cdev_init(pcie_devp);
	if(rc != 0 ){
		goto err_out_unmap;
	}

	return 0;
	
err_out_unmap:
	pci_release_regions(pdev);

err_out:
	free_pcie_dev( pcie_devp );
	
	printk ("EXIT %s, returning %d.\n", __func__, rc);
	return rc;
}

static void __devexit jw_pcie_remove(struct pci_dev *pdev)
{

	struct pcie_cdev *pcie_devp = pci_get_drvdata(pdev);
	
	if( pcie_devp->bar_memaddr )
	{
		iounmap( pcie_devp->bar_memaddr );
	}
	
	//pcie char device release
	pcie_cdev_exit( pcie_devp );
	
	pci_release_regions (pdev);
	
	pci_set_drvdata (pdev, NULL);
	
	pci_disable_device (pdev);

	free_pcie_dev( pcie_devp );

	printk ("%s Module Remove.\n", MODNAME);
}


static struct pci_driver jw_pcie_driver = {
	.name		= MODNAME,
	.id_table	= jw_pcie_tbl,

	.probe		= jw_pcie_probe,
	.remove		= __devexit_p(jw_pcie_remove),
#ifdef CONFIG_PM
//	.suspend	= jw_pcie_suspend,
//	.resume		= jw_pcie_resume,
#endif /* CONFIG_PM */
};

static int __init jw_pcie_init_module (void)
{
	int ret;
/* when a module, this is printed whether or not devices are found in probe */
	printk(KERN_INFO "Javee PCIE Module v0.1.\n" );
	ret = pci_register_driver(&jw_pcie_driver);
	if( ret  ){
		printk(KERN_ERR  "Failed to register pcie driver.\n ");
	}
	return ret;
}


static void __exit jw_pcie_cleanup_module (void)
{
	pci_unregister_driver (&jw_pcie_driver);
	printk(KERN_INFO "PCIE Module Exit.\n");
}


module_init(jw_pcie_init_module);
module_exit(jw_pcie_cleanup_module);

