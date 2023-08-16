/* SPDX-License-Identifier:GPL-2.0-only
 * pcie_cdev.c the pcie device driver
 *
 *Copyright (C) 2023 <wangsongtao2007@163.com>
 *
 */
#include "pcie.h"


u32 read_reg( struct pcie_cdev *pcie_devp, u32 offset  )
{
	u32 val;
	val = readl( pcie_devp->bar_memaddr + (4 * offset) );
	return val;
}

void write_reg( struct pcie_cdev *pcie_devp, u32 val, u32 offset )
{
	writel( val, pcie_devp->bar_memaddr + (4 * offset) );

	return;
}

int init_dma_ring( struct pcie_cdev *pcie_devp )
{

	pcie_devp->dma_read_buf = (unsigned char*)pci_alloc_consistent( pcie_devp->hwdev, MAX_ELE_NUM * MAX_PKG_SIZE, &(pcie_devp->dma_read_bus_addr) );
	if( !pcie_devp->dma_read_buf ){
		return -1;
	}

	pcie_devp->dma_write_buf = (unsigned char *)pci_alloc_consistent( pcie_devp->hwdev, MAX_ELE_NUM * MAX_PKG_SIZE, &(pcie_devp->dma_write_bus_addr) );
	if( !pcie_devp->dma_write_buf ){
		return -1;
	}

	return 0;
}


int init_table_addr( struct pcie_cdev *pcie_devp   )
{
	int i = 0;


	if( 0 == pcie_devp->bar_base_addr ||
			0 == pcie_devp->dma_read_bus_addr || 0 == pcie_devp->dma_write_bus_addr ){
		printk(KERN_CRIT"%s:Failed to init table.\n",__func__);
		return -1;
	}
	
	pcie_devp->dma_table_buf = pci_alloc_consistent( pcie_devp->hwdev, 
			MAX_ELE_NUM * sizeof(struct table_ele), &(pcie_devp->dma_table_bus_addr) );
	if( !pcie_devp->dma_table_buf ){
		printk(KERN_ERR"Failed to allcoe dma table buf.\n"  );
		return -1;
	}
	
	
	pcie_devp->table  = (struct table_ele*)pcie_devp->dma_table_buf;
	/* init table buf  */
	for(i = 0;  i < MAX_ELE_NUM; i++  ){
		pcie_devp->table[i].addr = pcie_devp->dma_write_bus_addr + i * MAX_PKG_SIZE;
	}

	/*init stable_staddr_reg and ele_length_reg */
	write_reg( pcie_devp, pcie_devp->dma_table_bus_addr, TABLE_STADDR_REG );
	write_reg( pcie_devp, 8, ELE_LENGTH_REG);

	/*init rc_baseaddr_reg for pcie-sce-fifo */
	write_reg( pcie_devp, pcie_devp->dma_write_bus_addr, RC_BASEADDR_REG );


	return 0;

}


void init_msi_config( struct pcie_cdev *pcie_devp  )
{
	//config dflow msi data
	write_reg( pcie_devp, 0x1, MSI_REG );

	
	//Configure MSI Capablity register
	config_msi( pcie_devp->hwdev, DMA_WRITE, DMA_CHANNEL_0  );

	//Configure MSI Capablity register
	config_msi( pcie_devp->hwdev, DMA_READ, DMA_CHANNEL_0  );

	return;
}


/********************************************************************************************/

static irqreturn_t msi_handler(int irq, void *dev)
{
	u16 intr_id;
	struct pcie_cdev *pcie_cdevp = (struct pcie_cdev *)dev;
	
	intr_id = get_intr_id( pcie_cdevp->hwdev  );
	
	//generate async signal
	
		if(pcie_cdevp->fasync)
		{
	
			kill_fasync(&pcie_cdevp->fasync, SIGIO, POLL_IN);
			
			
		}
	


	return IRQ_HANDLED;
}

static int request_msi_irq( struct pcie_cdev *pcie_cdevp, DMA_CHANNEL_NO channel_no )
{
	u16 msi_id;
	int ret;
	
	int msi_cap_pos = pci_find_capability( pcie_cdevp->hwdev, PCI_CAP_ID_MSI );
	pci_read_config_word( pcie_cdevp->hwdev, msi_cap_pos, &msi_id );

	if( (msi_id & PCI_CAP_ID_MSI) != PCI_CAP_ID_MSI ){
		printk( KERN_ERR "%s: This pcie device don't support msi.\n", __func__ );
    	return -1;
	}
	
	
	ret = pci_enable_msi(pcie_cdevp->hwdev);
	if(ret) {	
		printk( KERN_ERR "%s: Failed to eanble MSI.\n", __func__ );

		return -1;
	}

	//before call request_irq(), must call firstly pci_enable_msi()	
	if( request_irq( pcie_cdevp->hwdev->irq, msi_handler, IRQF_SHARED, "javee_pcie", pcie_cdevp ) ){
		printk( KERN_ERR "%s: Failed to request irq.\n", __func__  );
		return -1;
	}

	
   return 0;
}

static void free_msi_irq( struct pcie_cdev *pcie_cdevp  )
{
		
	free_irq( pcie_cdevp->hwdev->irq, pcie_cdevp  );

	printk(KERN_INFO "%s: free msi irq:%d.\n", __func__, pcie_cdevp->hwdev->irq  );
	pci_disable_msi( pcie_cdevp->hwdev  );

		
	return;
}
/*============================================================================= */
 int pcie_fasync(int fd, struct file *filp, int mode)
{
	struct pcie_cdev *pcie_devp = (struct pcie_cdev *)filp->private_data;/*  Get device structure pointer */

	return fasync_helper(fd, filp, mode, &pcie_devp->fasync);
}

int pcie_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long pfn = 0;

	unsigned long len = 0;
	
	unsigned long start_vir_addr = 0;
	unsigned long start_phy_addr = 0; 
	unsigned long off = 0;

	struct pcie_cdev *pcie_devp =(struct pcie_cdev *) filp->private_data;/*  Get device structure pointer */

	if( vma->vm_pgoff > (~0UL >> PAGE_SHIFT ) )
		return -EINVAL;
	
	start_vir_addr = (unsigned long)pcie_devp->dma_write_buf;
	off = vma->vm_pgoff << PAGE_SHIFT;
	
	len = PAGE_ALIGN( (start_vir_addr & ~PAGE_MASK) + MAX_ELE_NUM * MAX_PKG_SIZE );
	if( off >= len  ){
		off -= len;
	}

	if( (vma->vm_end - vma->vm_start + off) > len  )
		return -EINVAL;

	//start_phy_addr = start_vir_addr & PAGE_MASK;  //virtual addr ==> phy addr  
	start_phy_addr = virt_to_phys( (void *) start_vir_addr );
	off += start_phy_addr;

	pfn = off >> PAGE_SHIFT;

	//vma->vm_flags |= VM_RESERVED;
//	dvma->vm_page_prot = pgprot_writecombine( vma->vm_page_prot );
	if(remap_pfn_range(vma, vma->vm_start, pfn, vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk("failed to remap page.\n");
		return -EAGAIN;	
	}
	

	return 0;
}


int pcie_open(struct inode *inode, struct file *filp)
{
 
	struct cdev *pcie_cdev = NULL;
	struct pcie_cdev *pcie_devp = NULL;
	

	pcie_cdev = inode->i_cdev;
	if( !pcie_cdev ){
		return -1;
	}

	pcie_devp = container_of( pcie_cdev, struct pcie_cdev, cdev );
	if( pcie_devp  ){
		/* structure pcie_cdev assign to private data */
		filp->private_data = pcie_devp;
	} else {
		return -1;
	}

	/*config pcie device register*/	
	pcie_inbound_iatu( pcie_devp->hwdev, pcie_devp->bar_base_addr, EP_IATU_ADDR, pcie_devp->bar_addr_size );
    
	
	if(request_msi_irq(pcie_devp,DMA_CHANNEL_0))
	{
		printk( KERN_ERR "%s: Error Fail to request msi irq.\n", __func__ );
	}
	
	if( init_dma_ring( pcie_devp ) ){
	   printk(KERN_ERR  "%s: Error init dma ring.\n", __func__  );
	   return -1;
    }

	if( init_table_addr( pcie_devp  ) ){
		printk( KERN_ERR "%s: Error init table address.\n", __func__ );
		return -1;
	}

	init_msi_config( pcie_devp  );
	
	JW_DEBUG("%s module opened\n", DEVICE_NAME);
	return 0;
}

int pcie_close(struct inode *inode, struct file *filp)
{
	

    struct pcie_cdev *pcie_devp = filp->private_data;

	//delete file from async notify list 
	pcie_fasync(-1, filp, 0 );	
	
	free_msi_irq( pcie_devp  );




	if( pcie_devp->dma_read_buf  ){
		pci_free_consistent(pcie_devp->hwdev, MAX_ELE_NUM * MAX_PKG_SIZE, pcie_devp->dma_read_buf, pcie_devp->dma_read_bus_addr);
		pcie_devp->dma_read_buf = NULL;
		pcie_devp->dma_read_bus_addr = 0x00;
	}
	if( pcie_devp->dma_write_buf  ){
		pci_free_consistent(pcie_devp->hwdev, MAX_ELE_NUM * MAX_PKG_SIZE, pcie_devp->dma_write_buf, pcie_devp->dma_write_bus_addr);
		pcie_devp->dma_write_buf = NULL;
		pcie_devp->dma_write_bus_addr = 0x00;
	}



	if( pcie_devp->dma_table_buf  ){
		pci_free_consistent(pcie_devp->hwdev, MAX_ELE_NUM * sizeof(struct table_ele), pcie_devp->dma_table_buf, pcie_devp->dma_table_bus_addr);
		pcie_devp->table = NULL;
		pcie_devp->dma_write_buf = NULL;
		pcie_devp->dma_write_bus_addr = 0x00;
	}

	
	return 0;
}

static long pcie_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	
	unsigned int table_index = 0;
	struct driver_fra_head *driver_head = NULL;
	
	struct pcie_cdev *pcie_devp = filp->private_data;    /* Get device struture */
	u32 value = 0;
	long ret = 0;


	switch(cmd){
		case MSI_REG_READ:
			//*(u32*)arg = read_reg( pcie_devp, MSI_REG  );
			value = read_reg( pcie_devp, MSI_REG  );
			break;	
		case MSI_REG_WRITE:
			write_reg( pcie_devp, (u32)arg, MSI_REG  );
			break;

		case RC_SEND_RPTR_REG_READ:
			value = read_reg( pcie_devp, RC_SEND_RPTR_REG  );
			break;
		
		case RC_SEND_WPTR_REG_WRITE:
			write_reg( pcie_devp, (u32)arg, RC_SEND_WPTR_REG);
			break;
		case RC_SEND_WPTR_REG_READ:
			value = read_reg( pcie_devp, RC_SEND_WPTR_REG );
			break;

		case RC_RSLT_WPTR_REG_READ:
			value = read_reg( pcie_devp,RC_RSLT_WPTR_REG);
			break;

		case RC_RSLT_RPTR_REG_READ:
			value = read_reg( pcie_devp, RC_RSLT_RPTR_REG  );
			break;

		case TABLE_STADDR_REG_READ:
			value = read_reg( pcie_devp, TABLE_STADDR_REG );
			break;

		case ELE_LENGTH_REG_READ:
			value = read_reg( pcie_devp, ELE_LENGTH_REG );
			break;

		case GET_WRITE_BACK_ADDR:
			value = pcie_devp->dma_write_bus_addr;
			break;

		case SET_INFO_TABLE:
			table_index = (u32)arg;
			driver_head = (struct driver_fra_head *)(pcie_devp->dma_write_buf + table_index * MAX_PKG_SIZE);   
			pcie_devp->table[table_index].addr = pcie_devp->dma_write_bus_addr + table_index * MAX_PKG_SIZE;
			pcie_devp->table[table_index].len = driver_head->total_length;
			break;
		
		case ELE_LENGTH_REG_WRITE:
			break;

		default:
			return -EINVAL;
	}

	ret = __put_user( value, (u32*)arg ); 

    return ret;
}

static ssize_t pcie_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned long offset =  *ppos;
	ssize_t ret = 0;
	
	//JW_DEBUG("Enter %s\n", __func__);
	/* Get device structure pointer */
	struct pcie_cdev *pcie_devp = (struct pcie_cdev *)filp->private_data;

	// Do DMA transfer here....
	start_dma_read( pcie_devp->hwdev, pcie_devp->dma_read_bus_addr, EP_OBUF_ADDR, size ); 
	while( get_dma_trans_size( pcie_devp->hwdev ) ) {}


	if( copy_to_user(buf, pcie_devp->dma_read_buf , size) ) { 
		ret =  -EFAULT;
		goto out;
	} else {
		//*ppos += (unsigned long)size;
		ret = size;
	}
	

out:
    return ret;

}


static ssize_t pcie_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned long offset =  *ppos;
	ssize_t ret = 0;



	
	/* Get device structure pointer */
	struct pcie_cdev *pcie_devp = (struct pcie_cdev *)filp->private_data;
	
	
	// Now it is safe to copy the data from user space.
    if(copy_from_user(pcie_devp->dma_write_buf + offset, buf, size)) {
		ret =  - EFAULT;
		goto exit;
	} else {
		//*ppos += (unsigned long)size;
		ret = size;
	}
    
	// Do DMA transfer here....
	start_dma_write( pcie_devp->hwdev, EP_IBUF_ADDR, pcie_devp->dma_write_bus_addr, size );

	while( get_dma_trans_size( pcie_devp->hwdev ) ) {}

exit:
	return ret;

}




static loff_t pcie_llseek(struct file *filp, loff_t offset,int orig)
{
		loff_t ret = 0;
		switch (orig)
		{
			case 0:	/* 相对文件开始位置偏移 */
				if(offset < 0){
					ret = -1;
					break;
				}
	
				if((unsigned int)offset > MAX_ELE_NUM * MAX_PKG_SIZE){
					ret =  -1;
					break;
				}

				filp->f_pos = (unsigned int)offset;
				ret = filp->f_pos;
				break;

			case 1:	/* 相对文件当前位置偏移 */
				if((filp->f_pos + offset) > MAX_ELE_NUM * MAX_PKG_SIZE) {
					ret =  -1;
					break;
				}
				filp->f_pos += offset;
				ret = filp->f_pos;
				break;
				
			default:
				ret =  -1;
				break;		
		}
		
		return ret;
}


ssize_t ioctl_iatu_read(struct pcie_cdev *pcie_devp,  struct iatu_packet *iatu_pkt )
{
    int ret = 0;



	// Now it is safe to copy the data to user space.
	if( copy_to_user( iatu_pkt->buf, pcie_devp->bar_memaddr + iatu_pkt->offset, iatu_pkt->count ) ){
		ret = -1;
		goto exit;
	}



exit:	
	return ret;
}

ssize_t ioctl_iatu_write(struct pcie_cdev *pcie_devp, struct iatu_packet *iatu_pkt)
{
    int ret = 0;
	int i;
    int *p =(int *) iatu_pkt->buf;

	
	/*copy data from user to kernel*/
	if( copy_from_user(pcie_devp->bar_memaddr + iatu_pkt->offset, 
				iatu_pkt->buf,  iatu_pkt->count) ){
		ret = -1;
        goto exit;
    }

	for( i = 0; i < IATU_MAX_SIZE / 4; i++ )
	{
		JW_DEBUG("p[%d]=0x%08x\n", i, p[i] );
	}	
		
exit:	
	return ret;
	
}


struct file_operations pcie_fops = {
    .owner  = THIS_MODULE,
	.llseek = pcie_llseek,
	.read   = pcie_read,
	.write  = pcie_write,
	.unlocked_ioctl  = pcie_ioctl,
	.fasync = pcie_fasync,
	.mmap   = pcie_mmap,
	.open   = pcie_open,
	.release = pcie_close,
};

//static dev_t dev_number;
//static struct class *pcie_class;


int pcie_cdev_init( struct pcie_cdev *pcie_devp )
{
	int rc = 0;
	dev_t dev_number;
	
	/* Populate sysfs entries */
	pcie_devp->pcie_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(pcie_devp->pcie_class)){	
		printk(KERN_ERR"%s:Bad class create.\n", DEVICE_NAME);
		return -1;
	}

	/* Request dynamic allocation of a device major number */
	if(alloc_chrdev_region(&dev_number, 0,	1, DEVICE_NAME) < 0) {
		printk(KERN_ERR "Can't register device\n"); 
		goto err_out_des_class;
	}
	pcie_devp->dev_major = MAJOR(dev_number);
	
	/* Connect the file operations with the cdev */
	cdev_init(&pcie_devp->cdev, &pcie_fops);
    pcie_devp->cdev.owner = THIS_MODULE;
	
	 /* Connect the major/minor number to the cdev */
    if(cdev_add(&pcie_devp->cdev, MKDEV( pcie_devp->dev_major, 0 ), 1)) {
      printk(KERN_ERR"%s:Error adding device.\n", DEVICE_NAME);
      rc = -1;
	  goto err_out_unreg;
    }

	/* Send uevents to udev, so it'll create /dev nodes */
	//pcie_devp->pcie_dev = device_create(pcie_devp->pcie_class, 
	//                             NULL, MKDEV(pcie_devp->dev_major, 0),  
	//                             NULL, DEVICE_NAME, "%d", 0); 
	pcie_devp->pcie_dev = device_create(pcie_devp->pcie_class, 
			                       NULL, MKDEV(pcie_devp->dev_major, 0),  
								   NULL, DEVICE_NAME); 
	if(IS_ERR(pcie_devp->pcie_dev)){
		printk(KERN_ERR"%s:Bad devcie create.\n", DEVICE_NAME);
		rc = -1;
		goto err_out_unreg;
	}

	//chmod	
	struct file *filp = filp_open( "/dev/"DEVICE_NAME, 
			O_RDONLY | O_CREAT, 0 );
	if( IS_ERR(filp) ) {
		printk( KERN_ERR"/dev/%s open error.\n", DEVICE_NAME);
		goto err_out_unreg;
	}
	//filp->f_dentry->d_inode->i_mode |= S_IRWXUGO;
	filp_close( filp, NULL );

	return 0;

err_out_unreg:
	unregister_chrdev_region( MKDEV(pcie_devp->dev_major, 0), 1);

err_out_des_class:
	class_destroy(pcie_devp->pcie_class);

	return rc;
}

int pcie_cdev_exit( struct pcie_cdev *pcie_devp )
{
	
	/* Remove the cdev */
	cdev_del(&pcie_devp->cdev);
	
	/* Release the major number */
	unregister_chrdev_region(MKDEV(pcie_devp->dev_major, 0), 1);
	
	/* Destroy pcie device */
	device_destroy( pcie_devp->pcie_class, MKDEV(pcie_devp->dev_major, 0) );
	
	/* Destroy pcie_class */
	class_destroy( pcie_devp->pcie_class );
	
	printk(KERN_INFO "Exit %s.\n", __func__);
	
	return 0;
}
