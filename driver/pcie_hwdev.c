/* SPDX-License-Identifier:GPL-2.0-only
 * pcie_hwdev.c the pcie device driver
 *
 *Copyright (C) 2023 <wangsongtao2011@163.com>
 *
 */
#include "pcie.h"

void pcie_inbound_iatu( struct pci_dev *hwdev, u32 bar_base_addr, u32 ep_phy_addr, u32 bar_addr_size )
{	
	unsigned int value;

	pci_write_config_dword(hwdev, iATU_Viewport_Register, 0x80000000);

	pci_write_config_dword(hwdev, iATU_Lower_Target_Address_Register, ep_phy_addr);
	
	pci_write_config_dword(hwdev, iATU_Lower_Base_Address_Register, bar_base_addr);

	pci_write_config_dword(hwdev,  iATU_Limit_Address_Register,  bar_base_addr + (bar_addr_size - 1) );

	pci_write_config_dword(hwdev, iATU_Controll1_Register, 0x00000000);

	pci_write_config_dword(hwdev, iATU_Controll2_Register, 0x80000200);

}

void start_dma_read( struct pci_dev *hwdev, u32 dar, u32 sar, u32 count )
{
	/* DMA Write for EP */

	int value;
	
	//Configure DMA write( cpu read) region
	pci_write_config_dword(hwdev, DMA_Write_Engine_Enable, 0x1);
	
	//Configure MSI Capablity register
	config_msi( hwdev, DMA_WRITE, DMA_CHANNEL_0  );

	//bit[31] Channel Direction--0: write channel  1:read channel 0
	pci_write_config_dword(hwdev, DMA_Channel_Context_Index, 0x00000000);
	
	//Configure DMA write( cpu read) interrupt mask
	pci_write_config_dword( hwdev, DMA_write_interrupt_mask, 0xFF);
	
	pci_write_config_dword(hwdev, DMA_Channel_Control_Register, 0x04000018);//enable L&R interrupt
	
	//set pcie-EP DMA source address
	pci_write_config_dword(hwdev, DMA_SAR_Low, sar);
	pci_write_config_dword(hwdev, DMA_SAR_High, 0x0);
	
	//set pcie-EP DMA destination address
	//pci_write_config_dword(hwdev, DMA_DAR_Low, pcie_devp->dma_read_bus_addr);
	pci_write_config_dword(hwdev, DMA_DAR_Low, dar);
	pci_write_config_dword(hwdev, DMA_DAR_High, 0x0);
	
	//set dma transfer size
	pci_write_config_dword(hwdev, DMA_Transfer_Size, count);

}

void start_dma_write( struct pci_dev *hwdev, u32 dar, u32 sar, u32 count  )
{
	/* DMA Read for EP */
	int value;
	
	//Configure DMA read(cpu write) region	
	pci_write_config_dword(hwdev, DMA_Read_Engine_Enable, 0x1);
	
	//Configure MSI Capablity register
	config_msi( hwdev,DMA_READ, DMA_CHANNEL_0  );

	
	//bit[31] Channel Direction--0:write channel  1:read channel
	pci_write_config_dword(hwdev, DMA_Channel_Context_Index, 0x80000000);
	
	//Configure DMA read(cpu write) interrupt mask	
	pci_write_config_dword(hwdev, DMA_read_interrupt_mask, 0x0);
	pci_write_config_dword( hwdev, DMA_write_interrupt_mask, 0xFF);

	pci_write_config_dword(hwdev, DMA_Channel_Control_Register, 0x04000018);//enable L&R interrupt

	//set pcie-EP dma source address
	pci_write_config_dword(hwdev, DMA_SAR_Low, sar);
	pci_write_config_dword(hwdev, DMA_SAR_High, 0x0);

	//set pcie-EP dma destination address	
	pci_write_config_dword(hwdev, DMA_DAR_Low, dar);
	pci_write_config_dword(hwdev, DMA_DAR_High, 0x0);

        //set dma transfer size
	pci_write_config_dword(hwdev, DMA_Transfer_Size, count);
	
        //channel 0 start DMA write
	pci_write_config_dword(hwdev, DMA_Read_Doorbell, 0x0);

}


int get_dma_trans_size( struct pci_dev *hwdev  )
{
	int tran_size;
	pci_read_config_dword(hwdev, DMA_Transfer_Size, &tran_size);

	return tran_size;
}
u32 get_dma_read_intr_status( struct pci_dev *hwdev )
{
	u32 status;
	pci_read_config_dword( hwdev, DMA_write_interrupt_status, &status );
	
	return status;
}

void clear_dma_read_intr( struct pci_dev *hwdev  )
{
	u32 status;
	pci_read_config_dword( hwdev, DMA_write_interrupt_status, &status );

	pci_write_config_dword( hwdev, DMA_write_interrupt_clear, status );

}

u32 get_dma_write_intr_satus( struct pci_dev *hwdev )
{
	u32 status;
	pci_read_config_dword( hwdev, DMA_read_interrupt_status, &status );

	return status;
}

void clear_dma_write_intr( struct pci_dev *hwdev  )
{
	u32 status;
	pci_read_config_dword( hwdev, DMA_read_interrupt_status, &status );

	pci_write_config_dword( hwdev, DMA_read_interrupt_clear, status );
}

void set_payload_size( struct pci_dev *hwdev, u8 payload_size  )
{
	u16 devctl;
	u32 devcap;
	int pos = pci_find_capability(hwdev, PCI_CAP_ID_EXP);
	
	/*set devcie control register*/
	pci_read_config_word(hwdev, pos + PCI_EXP_DEVCTL, &devctl);

	devctl |= (payload_size << 5);
    	pci_write_config_word(hwdev, pos + PCI_EXP_DEVCTL,  devctl);

}

u16 get_intr_id( struct pci_dev *hwdev  )
{
	u16 msi_data;
	int msi_cap_pos = pci_find_capability( hwdev, PCI_CAP_ID_MSI  );
	
	pci_read_config_word( hwdev, msi_cap_pos + PCI_MSI_DATA_64, &msi_data );

	return msi_data;

}

void config_msi( struct pci_dev *hwdev,DMA_MODE dma_mode, DMA_CHANNEL_NO channel_no )
{
	u32 msi_addr_low, msi_addr_high;
   	u16 msi_data;
	u16 msi_ctl_reg;

        
	int msi_cap_pos = pci_find_capability( hwdev, PCI_CAP_ID_MSI );

	//get msi Message control register and Multiple Message Enable
	msi_ctl_reg |= (0x1 << 4); //Multiple Message Enable: 2 vector
	pci_write_config_word( hwdev, msi_cap_pos + PCI_MSI_FLAGS, msi_ctl_reg  );

	return;

}


