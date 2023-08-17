PCIE_HOME := $(shell pwd)
PCIE_KVER := $(shell uname -r)

MODULE = pcie


obj-m := $(MODULE).o
$(MODULE)-objs := pcie_hwdev.o pcie_cdev.o pcie_driver.o 

all:
	make -C /lib/modules/$(PCIE_KVER)/build M=$(PCIE_HOME) modules
clean:
	make -C /lib/modules/$(PCIE_KVER)/build M=$(PCIE_HOME) clean
