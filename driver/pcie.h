#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/wait.h>

#define __devexit
#define __devinitdata
#define __devinit
#define __devexit_p


#define DEBUG 0
#if (DEBUG)
#define JW_DEBUG(args...) printk(args)
#else
#define JW_DEBUG(args...)
#endif

#define DEVICE_NAME			"jw_pcie"
#define MODNAME				"jw_pcie_module"

#define BAR_NUM				3

// Max DMA Buffer Size
#define DMA_BUF_SIZE               (4 * 1024 * 1024)

#define PCIE_VENDOR_ID      	  0x0106
#define PCIE_DEVICE_ID 			  0x2016


#define KINBURN_REGISTER_SIZE     (4*8)    // There are eight registers, and each is 4 bytes wide.
#define HAVE_REGION               0x01     // I/O Memory region
#define HAVE_IRQ                  0x02     // Interupt
#define SUCCESS                   0
#define CRIT_ERR                  -1

#define ELE_LENGTH                8
#define MAX_ELE_NUM              128
#define MAX_PKG_SIZE             2048

#define EP_IATU_ADDR             0x10402000
#define EP_IBUF_ADDR             0x20000000
#define EP_OBUF_ADDR             0x20040000


#define SRAM_BASE_ADDR         	0x20000000
#define DDR_BASE_ADDR           0x80100000

#define PAYLOAD_SIZE_128B          (0x00)   //128Byte
#define PAYLOAD_SIZE_256B          (0x01)   //256Byte
#define PAYLOAD_SIZE_512B          (0x02)   //512Byte
#define PAYLOAD_SIZE_1K            (0x03)   //1024Byte
#define PAYLOAD_SIZE_2K            (0x04)   //2048Byte
#define PAYLOAD_SIZE_4K            (0x05)   //4096Byte



#define BAR2_MEM_SIZE  0x100000		/* bar memory max size: 1MB*/
#define IATU_MAX_SIZE   256    //Byte

#define IOCTL_MEM_CLEAR  0
#define IOCTL_IATU_READ   11
#define IOCTL_IATU_WRITE  12


/************************************************************************
*
* PCIE stand register 0-4KB
*
*************************************************************************/
#define Device_ID_Vendor_ID				        0x00
#define Status_Register_Command_Register		0x04
#define Class_Code_Revision_Size			    0x08
#define BAR0					 	            0x10
#define BAR1						            0x14
#define BAR2						            0x18
#define BAR3						            0x1c
#define BAR4						            0x20
#define BAR5						            0x24

/* iATU register */
#define iATU_Viewport_Register				    0x900
#define iATU_Controll1_Register				    0x904
#define iATU_Controll2_Register				    0x908
#define iATU_Controll3_Register				    0x920
#define iATU_Lower_Base_Address_Register		0x90c
#define iATU_Upper_Base_Address_Register		0x910
#define iATU_Limit_Address_Register			    0x914
#define iATU_Lower_Target_Address_Register		0x918
#define iATU_Upper_Target_Address_Register		0x91c


/*DMA register*/
#define DMA_Write_Engine_Enable   			    0x97c
#define DMA_Write_Doorbell        			    0x980
#define DMA_Read_Engine_Enable    			    0x99c
#define DMA_Read_Doorbell         			    0x9a0
#define DMA_write_interrupt_status			    0x9bc
#define DMA_write_interrupt_mask			    0x9c4
#define DMA_write_interrupt_clear			    0x9c8

#define DMA_write_error_status				    0x9cc
#define DMA_read_error_status_low			    0xa24
#define DMA_read_error_status_high			    0xa28

#define DMA_read_interrupt_status			    0xa10
#define DMA_read_interrupt_mask				    0xa18
#define DMA_read_interrupt_clear			    0xa1c
#define DMA_Channel_Context_Index 			    0xa6c
#define DMA_Transfer_Size         			    0xa78
#define DMA_SAR_Low               			    0xa7c
#define DMA_SAR_High              			    0xa80
#define DMA_DAR_Low               			    0xa84
#define DMA_DAR_High              			    0xa88
#define DMA_Linked_List_Pointer_Low         	0xa8c
#define DMA_Linked_List_Pointer_High        	0xa90
#define DMA_Channel_Control_Register        	0xa70

#define DMA_write_done_imwr_low_addr			0x9d0
#define DMA_write_done_imwr_high_addr			0x9d4
#define DMA_write_abort_imwr_low_addr			0x9d8
#define DMA_write_abort_imwr_high_addr			0x9dc
#define DMA_write_channel_imwr_data			    0x9e0

#define DMA_read_done_imwr_low_addr			    0xa3c
#define DMA_read_done_imwr_high_addr			0xa40
#define DMA_read_abort_imwr_low_addr			0xa44
#define DMA_read_abort_imwr_high_addr			0xa48
#define DMA_read_channel_imwr_data			    0xa4c


struct iatu_packet{
	unsigned char buf[IATU_MAX_SIZE];
	unsigned int offset;
	unsigned int count;
};

/*char device structure for pcie*/
struct pcie_cdev
{
	struct pci_dev *hwdev; 
	struct cdev cdev; /* The cdev structure */


	struct fasync_struct    *fasync;
	
	int dev_major;
	struct class *pcie_class;
	struct device *pcie_dev;

	void *bar_memaddr;
	
	unsigned long bar_base_addr;
	unsigned long bar_addr_size;
	
	//unsigned long *dma_read_buf;
	unsigned char *dma_read_buf;
	dma_addr_t dma_read_bus_addr;

	//unsigned long *dma_write_buf;
	unsigned char *dma_write_buf;
	dma_addr_t dma_write_bus_addr;


	/* dflow  */
	struct table_ele *table;
	unsigned long *dma_table_buf;
	dma_addr_t dma_table_bus_addr;

};

struct table_ele{
	u32 addr;
	u32 len;
};

typedef enum{
	DMA_CHANNEL_0 = 0,
	DMA_CHANNEL_1 = 1,
	DMA_CHANNEL_2 = 2,
	DMA_CHANNEL_3 = 3,
	DMA_CHANNEL_4 = 4,
	DMA_CHANNEL_5 = 5,
	DMA_CHANNEL_6 = 6,
	DMA_CHANNEL_7 = 7
}DMA_CHANNEL_NO;

typedef enum{
	DMA_READ = 0,
	DMA_WRITE = 1
}DMA_MODE;
/************************************************************************
*
* PCIE extend register  0x10402000 (0-24B)dlow control
*
*************************************************************************/
typedef enum{
	MSI_REG = 0,
	EXT_INT_REG,
	TXDEEMPHY_REG,
	RC_SEND_RPTR_REG,
	RC_RSLT_WPTR_REG,
	RC_SEND_WPTR_REG,
	RC_RSLT_RPTR_REG,
	TABLE_STADDR_REG,
	ELE_LENGTH_REG,
	RC_BASEADDR_REG
}DFLOW_REG;


typedef enum{

	/* register offset start */
	MSI_REG_READ = 0,
	MSI_REG_WRITE,

	EXT_INT_REG_READ,
	EXT_INT_REG_WRITE,

	TXDEEMPHY_REG_READ,
	TXDEEMPHY_REG_WRITE,

	RC_SEND_RPTR_REG_READ,
	RC_SEND_RPTR_REG_WRITE,

	RC_RSLT_WPTR_REG_READ,
	RC_RSLT_WPTR_REG_WRITE,
	
	RC_SEND_WPTR_REG_READ,
	RC_SEND_WPTR_REG_WRITE,

	RC_RSLT_RPTR_REG_READ,
	RC_RSLT_RPTR_REG_WRITE,

	TABLE_STADDR_REG_READ,
	TABLE_STADDR_REG_WRITE,

	ELE_LENGTH_REG_READ,
	ELE_LENGTH_REG_WRITE,
	
	RC_BASEADDR_REG_READ,
	RC_BASEADDR_REG_WRITE,

	GET_WRITE_BACK_ADDR,
	
	SET_INFO_TABLE,
	
	
}DFLOW_IOCTL;

 struct driver_fra_head
{
	unsigned short total_length;
	unsigned short flag;

	unsigned int wb_addr;
	
	unsigned char new_flag;
	unsigned char hdr_len;
	unsigned short pktid;

	unsigned short hdr_sum;
	unsigned short res;
};
 /************************************************************************
 *
 * PCIE function
 *
 *************************************************************************/

void pcie_inbound_iatu( struct pci_dev *hwdev, u32 bar_base_addr, u32 ep_phy_addr, u32 bar_addr_size );
void pcie_outbound_iatu( struct pci_dev *hwdev );

int init_dma_ring( struct pcie_cdev *pcie_devp );
void start_dma_read( struct pci_dev *hwdev,  u32 dar, u32 sar, u32 count );
void start_dma_write( struct pci_dev *hwdev, u32 dar, u32 sar, u32 count );

int get_dma_trans_size( struct pci_dev *hwdev );

u32 get_dma_read_intr_status( struct pci_dev *hwdev );
void clear_dma_read_intr( struct pci_dev *hwdev  );
u32 get_dma_write_intr_satus( struct pci_dev *hwdev );
void clear_dma_write_intr( struct pci_dev *hwdev  );

void set_payload_size( struct pci_dev *hwdev, u8 payload_size  );
u16 get_intr_id( struct pci_dev *hwdev  );
void config_msi( struct pci_dev *hwdev, DMA_MODE dma_mode, DMA_CHANNEL_NO channel_no );
void dflow_init(  struct pci_dev *hwdev );

/* funciton statement for pcie_cdev.c  */
int pcie_cdev_init( struct pcie_cdev *pcie_devp );
int pcie_cdev_exit( struct pcie_cdev *pcie_devp );

ssize_t ioctl_iatu_read(struct pcie_cdev *pcie_devp, struct iatu_packet *iatu_pkt);
ssize_t ioctl_iatu_write(struct pcie_cdev *pcie_devp, struct iatu_packet *iatu_pkt);





