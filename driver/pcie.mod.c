#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x32e21920, "module_layout" },
	{ 0xc7466e35, "cdev_del" },
	{ 0x26087692, "kmalloc_caches" },
	{ 0x410253ad, "pci_write_config_dword" },
	{ 0x320c3a3f, "cdev_init" },
	{ 0x9ef55041, "pci_write_config_word" },
	{ 0xfb14ff3c, "pci_disable_device" },
	{ 0x8d6aff89, "__put_user_nocheck_4" },
	{ 0xfdd66e81, "device_destroy" },
	{ 0x9e2fb826, "filp_close" },
	{ 0xf288fcdc, "pci_release_regions" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x87cec3d1, "dma_free_attrs" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x1e323c29, "pci_set_master" },
	{ 0xde80cd09, "ioremap" },
	{ 0x4c9d28b0, "phys_base" },
	{ 0xdeddd266, "fasync_helper" },
	{ 0x29a90398, "pci_read_config_word" },
	{ 0x1d0107ac, "dma_alloc_attrs" },
	{ 0xa94ce2a9, "device_create" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x79161eb9, "_dev_err" },
	{ 0xc63095c, "pci_enable_msi" },
	{ 0x822b009f, "pci_find_capability" },
	{ 0x6a0a71c7, "cdev_add" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x92997ed8, "_printk" },
	{ 0xf5bd9a83, "pci_read_config_dword" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x233f929e, "pci_unregister_driver" },
	{ 0xf35141b2, "kmem_cache_alloc_trace" },
	{ 0x37a0cba, "kfree" },
	{ 0x44a3fa19, "remap_pfn_range" },
	{ 0x95b52781, "pci_request_regions" },
	{ 0xfc8d4362, "pci_disable_msi" },
	{ 0xedc03953, "iounmap" },
	{ 0xf2bb7d9, "__pci_register_driver" },
	{ 0x748154da, "class_destroy" },
	{ 0x344b27fd, "kill_fasync" },
	{ 0x8d929026, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xaf919f03, "__class_create" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xa0c4c91b, "filp_open" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v00000106d00002016sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "A81F5FA081893CDB3323AB9");
