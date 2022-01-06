/*
 *  linux/arch/arm/mach-cns3xxx/ts43xx.c
 *
 *  Copyright (c) 2012-2022 Technologic Systems, Inc. dba embeddedTS
 *  Copyright (c) 2008 Cavium Networks 
 *  Copyright (C) 2008 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  
 *  This file is free software; you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License, Version 2, as 
 *  published by the Free Software Foundation. 
 *
 *  This file is distributed in the hope that it will be useful, 
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or 
 *  NONINFRINGEMENT.  See the GNU General Public License for more details. 
 *
 *  You should have received a copy of the GNU General Public License 
 *  along with this file; if not, write to the Free Software 
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or 
 *  visit http://www.gnu.org/licenses/. 
 *
 *  This file may also be available under a different license from Cavium. 
 *  Contact Cavium Networks for more information
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/host.h>
#include <linux/amba/bus.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <mach/cns3xxx.h>
#include <mach/misc.h>
#include <mach/irqs.h>
#include <mach/pm.h>
#include <mach/sdhci.h>
#include "core.h"
#include "devices.h"
#include <mach/lm.h>

extern void __iomem *gic_cpu_base_addr;

extern void __init ts43xx_uart0_init(unsigned int membase, resource_size_t mapbase,
			     unsigned int irq,
			     unsigned int uartclk);



/* UART0 */
#ifdef CONFIG_SERIAL_8250_CONSOLE
static struct uart_port cns3xxx_serial_ports[] = {
	{
		.membase        = (char*) (CNS3XXX_UART0_BASE_VIRT),
		.mapbase        = (CNS3XXX_UART0_BASE),
		.irq            = IRQ_CNS3XXX_UART0,
		.iotype         = UPIO_MEM,
		.flags          = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.regshift       = 2,
		.uartclk        = 24000000,
		.line           = 0,
		.type           = PORT_16550A,
		.fifosize       = 16
	},
	{
		.membase        = (char*) (CNS3XXX_UART1_BASE_VIRT),
		.mapbase        = (CNS3XXX_UART1_BASE),
		.irq            = IRQ_CNS3XXX_UART1,
		.iotype         = UPIO_MEM,
		.flags          = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.regshift       = 2,
		.uartclk        = 24000000,
		.line           = 1,
		.type           = PORT_16550A,
		.fifosize       = 16
	},
	{
		.membase        = (char*) (CNS3XXX_UART2_BASE_VIRT),
		.mapbase        = (CNS3XXX_UART2_BASE),
		.irq            = IRQ_CNS3XXX_UART2,
		.iotype         = UPIO_MEM,
		.flags          = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.regshift       = 2,
		.uartclk        = 24000000,
		.line           = 2,
		.type           = PORT_16550A,
		.fifosize       = 16
	},
};
#endif

/**
   The bootloader will have read the flash OTP area and placed its contents
   in a known location.  ts43xx_fixup (see below) will have copied the data
   from that location to cns3xxx_flashOTP, which is then read by the 
   ethernet driver when it starts, allowing it to set the MAC address based 
   on the OTP data.
*/

static unsigned long cns3xxx_flashOTP[16];

unsigned long *getFlashOTP(void) {
   return cns3xxx_flashOTP;  
}
EXPORT_SYMBOL(getFlashOTP);

static void __init ts43xx_fixup(
				 struct tag *tags, char **cmdline,
				 struct meminfo *mi)
{
      
   unsigned long memSize;
   unsigned char *otpArea;    /* The contents of the flash OTP area should have
                                 been copied here by the bootloader */
   __pv_phys_offset = CNS3XXX_DDR2SDRAM_BASE;
      
   memSize = *(unsigned long *)__bus_to_virt(CNS3XXX_DDR2SDRAM_BASE + 0x20);
   otpArea = (unsigned char *)__bus_to_virt(CNS3XXX_DDR2SDRAM_BASE + 0x24);
   
   printk("Bootloader says it found %ld MB of DDR2\n", memSize);
            
	mi->nr_banks = 1;
	mi->bank[0].start =	CNS3XXX_DDR2SDRAM_BASE + 0x00000000;
	if (memSize == 512)
	   mi->bank[0].size = SZ_512M;
	else
	   mi->bank[0].size = SZ_256M;	
	
   memcpy(cns3xxx_flashOTP, otpArea, 64);	
	
   
   
}



static void __init ts43xx_map_io(void)
{   
	cns3xxx_map_io();
		
#ifdef CONFIG_SERIAL_8250_CONSOLE
   HAL_MISC_ENABLE_UART1_PINS();
	cns3xxx_pwr_power_up(CNS3XXX_PWR_PLL(PLL_USB));
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(UART1));	
	early_serial_setup(&cns3xxx_serial_ports[0]);	
#if (1 < CONFIG_SERIAL_8250_NR_UARTS)
	HAL_MISC_ENABLE_UART1_PINS();	
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(UART1));	
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(UART1));	
	early_serial_setup(&cns3xxx_serial_ports[1]);	
#endif
#if (2 < CONFIG_SERIAL_8250_NR_UARTS)
	HAL_MISC_ENABLE_UART2_PINS();
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(UART2));
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(UART2));
	early_serial_setup(&cns3xxx_serial_ports[2]);
#endif
#endif
}

/* RTC */
static struct resource cns3xxx_rtc_resources[] = {
	[0] = {
		.start = CNS3XXX_RTC_BASE,
		.end   = CNS3XXX_RTC_BASE + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_RTC,
		.end   = IRQ_CNS3XXX_RTC,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device cns3xxx_rtc_device = {
	.name		= "cns3xxx-rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cns3xxx_rtc_resources),
	.resource	= cns3xxx_rtc_resources,
};

/* GPIO */
static struct resource cns3xxx_gpio_resources[] = {
	[0] = {
		.start = CNS3XXX_GPIOA_BASE,
		.end   = CNS3XXX_GPIOA_BASE + 0x44,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = CNS3XXX_GPIOB_BASE,
		.end   = CNS3XXX_GPIOB_BASE + 0x44,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_CNS3XXX_GPIOA,
		.end   = IRQ_CNS3XXX_GPIOA,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_CNS3XXX_GPIOB,
		.end   = IRQ_CNS3XXX_GPIOB,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device cns3xxx_gpio_device = {
	.name		= "cns3xxx-gpio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cns3xxx_gpio_resources),
	.resource	= cns3xxx_gpio_resources,
};

/* Watchdog */
static struct resource cns3xxx_watchdog_resources[] = {
	[0] = {
		.start = CNS3XXX_TC11MP_TWD_BASE,
		.end   = CNS3XXX_TC11MP_TWD_BASE + PAGE_SIZE - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
		.start = IRQ_LOCALWDOG,
		.end   = IRQ_LOCALWDOG,
		.flags = IORESOURCE_IRQ,
        }
};

static struct platform_device cns3xxx_watchdog_device = {
	.name		= "cns3xxx-wdt",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cns3xxx_watchdog_resources),
	.resource	= cns3xxx_watchdog_resources,
};

/* SDIO, MMC/SD */
static struct resource cns3xxx_sdio_resource[] = {
	[0] = {
		.start = CNS3XXX_SDIO_BASE,
		.end   = CNS3XXX_SDIO_BASE + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_SDIO,
		.end   = IRQ_CNS3XXX_SDIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct cns3xxx_sdhci_platdata cns3xxx_sdio_platform_data = {
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
};

static u64 cns3xxx_device_sdhci_dmamask = 0xffffffffUL;

static struct platform_device cns3xxx_sdio_device = {
	.name		= "cns3xxx-sdhci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(cns3xxx_sdio_resource),
	.resource	= cns3xxx_sdio_resource,
	.dev		= {
		.dma_mask		= &cns3xxx_device_sdhci_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data		= &cns3xxx_sdio_platform_data,
	}
};

/* I2C */


static struct i2c_board_info __initdata cns3xxx_i2c_devices[] = {
	{
		I2C_BOARD_INFO ("24c16",0x50),
	},

#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
	{
		I2C_BOARD_INFO ("wm8991 ch1", 0x1a),
	},
	{
		I2C_BOARD_INFO ("wm8991 ch2", 0x1b),
	},
#endif
#if defined(CONFIG_SND_SOC_WM8580) || defined(CONFIG_SND_SOC_WM8580_MODULE)
	{
//		I2C_BOARD_INFO ("wm8580", 0x1b),
		I2C_BOARD_INFO ("wm8580", 0x1a),
	},
#endif
#if defined(CONFIG_SND_SOC_WM9081) || defined(CONFIG_SND_SOC_WM9081_MODULE)
	{
		I2C_BOARD_INFO ("wm9081", 0x6c),
	},
#endif
};

static struct resource cns3xxx_i2c_resource[] = {
	[0] = {
		.start		= CNS3XXX_SSP_BASE + 0x20,
		.end		= 0x7100003f,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_CNS3XXX_I2C,
		.flags		= IORESOURCE_IRQ,
	},
};



static struct platform_device cns3xxx_i2c_controller_device = {
	.name		= "cns3xxx-i2c",
	.num_resources	= 2,
	.resource	= cns3xxx_i2c_resource,
};

static struct i2c_board_info __initdata cns3xxx_i2c_gpio_devices[] = {
#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
  {
		I2C_BOARD_INFO("mt9v111", 0x48),
//		.platform_data = &iclink[0],
		.platform_data = &iclink, /* With extender */
  },
  {
        I2C_BOARD_INFO("ov2655", (0x60>>1)), /* slave address is 0x60 */
        .platform_data = &iclink, /* With extender */
  },
#endif
};

static struct i2c_gpio_platform_data cns3xxx_i2c_data = {
	.sda_pin		= 1,
	.sda_is_open_drain	= 0,
	.scl_pin		= 0,
	.scl_is_open_drain	= 0,
	.udelay			= 2,
};

static struct platform_device cns3xxx_i2c_gpio_controller_device = {
	.name			= "i2c-gpio",
	.id			= 1,
	.dev.platform_data	= &cns3xxx_i2c_data,
};



/* I2S */
static struct resource cns3xxx_i2s_resource = {
	.start		= CNS3XXX_I2S_BASE,
	.end		= CNS3XXX_I2S_BASE + 0x53,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device cns3xxx_i2s_device = {
	.name		= "cns3xxx-i2s",
	.num_resources	= 1,
	.resource	= &cns3xxx_i2s_resource,
};


#if (1)
/* PCM */
static struct resource cns3xxx_pcm_resource = {
	.start		= CNS3XXX_SSP_BASE + 0x180,
	.end		= CNS3XXX_SSP_BASE + 0x1ff,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device cns3xxx_pcm_device = {
	.name		= "cns3xxx-pcm1",
	.num_resources	= 1,
	.resource	= &cns3xxx_pcm_resource,
};

#endif // if 0

static struct resource cns3xxx_usb_ehci_resource[] = {
	[0] = {
		.start = CNS3XXX_USB_BASE,
		.end   = CNS3XXX_USB_BASE + SZ_16M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_USB_EHCI,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 cns3xxx_usb_dma_mask = 0xffffffffULL;

static struct platform_device cns3xxx_usb_ehci_device = {
	.name		= "cns3xxx-ehci",
	.num_resources	= ARRAY_SIZE(cns3xxx_usb_ehci_resource),
	.resource	= cns3xxx_usb_ehci_resource,
	.dev		= {
		.dma_mask		= &cns3xxx_usb_dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct resource cns3xxx_usb_ohci_resource[] = {
        [0] = {
                .start          = CNS3XXX_USB_OHCI_BASE,
                .end            = CNS3XXX_USB_OHCI_BASE + SZ_16M - 1,
                .flags          = IORESOURCE_MEM,
        },
        [1] = {
                .start          = IRQ_CNS3XXX_USB_OHCI,
                .flags          = IORESOURCE_IRQ,
        },
};

static u64 cns3xxx_usb_ohci_dma_mask = 0xffffffffULL;
static struct platform_device cns3xxx_usb_ohci_device = {
    .name = "cns3xxx-ohci",
    .dev                = {
        .dma_mask       = &cns3xxx_usb_ohci_dma_mask,
        .coherent_dma_mask = 0xffffffffULL,
     },
    .num_resources = 2,
    .resource = cns3xxx_usb_ohci_resource,
};



static u64 cns3xxx_usbotg_dma_mask = 0xffffffffULL;
static struct lm_device cns3xxx_usb_otg_device = {
    .dev                = {
        .dma_mask       = &cns3xxx_usbotg_dma_mask,
        .coherent_dma_mask = 0xffffffffULL,
     },
    .resource           = {
        .start          = CNS3XXX_USBOTG_BASE,
        .end            = CNS3XXX_USBOTG_BASE + SZ_16M - 1,
        .flags          = IORESOURCE_MEM,
     },
    .irq      = IRQ_CNS3XXX_USB_OTG,
};


static struct resource cns3xxx_ahci_resource[] = {
	[0] = {
		.start		= CNS3XXX_SATA2_BASE,
		.end		= CNS3XXX_SATA2_BASE + CNS3XXX_SATA2_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_CNS3XXX_SATA,
		.end		= IRQ_CNS3XXX_SATA,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64 cns3xxx_device_ahci_dmamask = 0xffffffffUL;

static struct platform_device cns3xxx_ahci = {
	.name		= "cns3xxx_ahci",
	.id		= -1,
	.dev		= {
		.dma_mask		= &cns3xxx_device_ahci_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
//		.platform_data	= &cns3xxx_sata_data,
	},
	.resource	= cns3xxx_ahci_resource,
	.num_resources	= ARRAY_SIZE(cns3xxx_ahci_resource),
};


/* SPI */
static struct mtd_partition cns3xxx_spi_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 0x4000,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	{
		.name		= "BootpImage",
		.offset		= 0x4000,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};
static struct flash_platform_data cns3xxx_spi_flash_data = {
	.parts		= cns3xxx_spi_partitions,
	.nr_parts	= ARRAY_SIZE(cns3xxx_spi_partitions),
};

static struct spi_board_info __initdata cns3xxx_spi_devices[] = {
	[0] = {
		.modalias		= "m25p80",
	        .bus_num		= 1,
		.chip_select		= 0,
	        .max_speed_hz		= 25 * 1000 * 1000,
	        .platform_data		=  &cns3xxx_spi_flash_data,
	}, {	
#if defined(CONFIG_LE8221_CONTROL)
		.modalias		= "le88221",
#elif defined(CONFIG_SI3226_CONTROL_API)
		.modalias		= "si3226",
#endif
	        .bus_num		= 1,
		.chip_select		= 1,
	        .max_speed_hz		= 25 * 1000 * 1000,
	},
};


static struct platform_device cns3xxx_spi_controller_device = {
	.name		= "cns3xxx_spi",
};


#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
static u64 cns3xxx_dma_mask_camera = DMA_BIT_MASK(32);

static struct cns3xxx_camera_platform_data camera_platform_data = {
	.flags  = CNS3XXX_CAMERA_MASTER | CNS3XXX_CAMERA_DATAWIDTH_8 | CNS3XXX_CAMERA_DATAWIDTH_10 |
		CNS3XXX_CAMERA_PCLK_EN | CNS3XXX_CAMERA_MCLK_EN/* | CNS3XXX_CAMERA_PCP*/,
	.mclk_10khz = 1000,
	.lcd_base = CNS3XXX_CLCD_BASE_VIRT,
	.misc_base = CNS3XXX_MISC_BASE_VIRT,
};

static struct resource cns3xxx_camera_resource[] = {
	[0] = {
		.start	= CNS3XXX_CAMERA_BASE,
		.end	= CNS3XXX_CAMERA_BASE+0x00000fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 40,
		.end	= 40,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device cns3xxx_camera_device = {
	.name		= "cns3xxx-camera",
	.id		= 0, /* This is used to put cameras on this interface */
	.dev		= {
		.dma_mask      		= &cns3xxx_dma_mask_camera,
		.coherent_dma_mask	= 0xffffffff,
        .platform_data = (void *) &camera_platform_data,
	},
	.num_resources	= ARRAY_SIZE(cns3xxx_camera_resource),
	.resource	= cns3xxx_camera_resource,
};
#endif


/* Define some components of CNS3XXX SoCs */
static struct platform_device *cns3xxx_devs[] __initdata = {
	&cns3xxx_rtc_device,
	&cns3xxx_gpio_device,
	&cns3xxx_watchdog_device,
	&cns3xxx_sdio_device,
	&cns3xxx_i2c_controller_device,
	&cns3xxx_i2c_gpio_controller_device,
	&cns3xxx_i2s_device,
#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
	&cns3xxx_camera_device,
#endif
	&cns3xxx_pcm_device,
	&cns3xxx_spi_controller_device,	
	&cns3xxx_usb_ehci_device,
	&cns3xxx_usb_ohci_device,
	&cns3xxx_ahci,
};




/*
 * Cavium Networks ARM11 MPCore AMBA devices
 */
#define CLCD_IRQ	{ 123, NO_IRQ }
#define CLCD_DMA	{ 0, 0 }

AMBA_DEVICE(clcd,	"dev:20",	CLCD,	&clcd_plat_data);

static struct amba_device *amba_devs[] __initdata = {
	&clcd_device,
};


#if (0)
/*
 * Cavium Networks ARM11 MPCore platform devices
 */
static struct resource cns3xxx_flash_resource[] = {
	[0] = {
		.start		= CNS3XXX_FLASH0_BASE,
		.end		= CNS3XXX_FLASH0_BASE + CNS3XXX_FLASH0_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	}, /*
	[1] = {
		.start		= CNS3XXX_FLASH1_BASE,
		.end		= CNS3XXX_FLASH1_BASE + CNS3XXX_FLASH1_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	}, */
};


static void __init gic_init_irq(void)
{
	/* irq mode with no DCC */


	
	/* ARM11 MPCore test chip GIC */
	gic_cpu_base_addr = (void __iomem *) CNS3XXX_TC11MP_GIC_CPU_BASE_VIRT;
	gic_dist_init(0, (void __iomem *) CNS3XXX_TC11MP_GIC_DIST_BASE_VIRT, 29);
	gic_cpu_init(0, gic_cpu_base_addr);
	
}
#endif // if 0



struct proc_dir_entry *cns3xxx_proc_dir=0;

#ifdef CONFIG_DEBUG_FS
struct dentry *cns3xxx_debugfs_dir=0;
#endif

#ifdef CONFIG_CNS3XXX_RAID
extern int cns_rdma_init(void);
#endif

#if (0)
void cns3xxx_power_off(void)
{
	__u32 clkctrl;

	printk(KERN_INFO "powering system down...\n");

        clkctrl = readl(CNS3XXX_PM_BASE_VIRT + PM_SYS_CLK_CTRL_OFFSET);
        clkctrl &= 0xfffff1ff;
        clkctrl |= (0x5 << 9);		/* Hibernate */
        writel(clkctrl, CNS3XXX_PM_BASE_VIRT + PM_SYS_CLK_CTRL_OFFSET);

}
#endif

#ifdef CONFIG_CACHE_L2CC

#define L2C_REG_VALUE(offset) (*((volatile unsigned int *)(CNS3XXX_L2C_BASE_VIRT+offset)))
#define MISC_REG_VALUE(offset) (*((volatile unsigned int *)(CNS3XXX_MISC_BASE_VIRT+offset)))
struct proc_dir_entry *l2cc_proc_entry;

static int l2cc_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int num = 0;
    num += sprintf(page + num, "CACHE_ID_REG             : 0x%.8x\n", L2C_REG_VALUE(0x000));
    num += sprintf(page + num, "CACHE_TYPE_REG           : 0x%.8x\n", L2C_REG_VALUE(0x004));
    num += sprintf(page + num, "CONTROL_REG              : 0x%.8x\n", L2C_REG_VALUE(0x100));
    num += sprintf(page + num, "EVENT_COUNTER_CTRL_REG   : 0x%.8x\n", L2C_REG_VALUE(0x200));
    num += sprintf(page + num, "EVENT_COUNTER_1_CONF_REG : 0x%.8x\n", L2C_REG_VALUE(0x204));
    num += sprintf(page + num, "EVENT_COUNTER_0_CONF_REG : 0x%.8x\n", L2C_REG_VALUE(0x208));
    num += sprintf(page + num, "EVENT_COUNTER_1_VAL_REG  : 0x%.8x\n", L2C_REG_VALUE(0x20C));
    num += sprintf(page + num, "EVENT_COUNTER_0_VAL_REG  : 0x%.8x\n", L2C_REG_VALUE(0x210));
	return num;
}

int l2cc_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define DRHIT 0x2
#define DRREQ 0x3
#define DWHIT 0x4
#define DWREQ 0x5
	unsigned int cnt_src_0, cnt_src_1;
	if (count) {
		if (buffer[0] == 'r') {
			cnt_src_0 = ((DRHIT << 2) | 0x1);
			cnt_src_1 = ((DRREQ << 2) | 0x1);
		} else if (buffer[0] == 'w') {
			cnt_src_0 = ((DWHIT << 2) | 0x1);
			cnt_src_1 = ((DWREQ << 2) | 0x1);
		} else {
			return count;
		}
	} else {
		return count;
	}


	MISC_REG_VALUE(0x200) |= 0x1;       /* enable cache event monitor */	

	L2C_REG_VALUE(0x200) &= ~(0x1);     /* disable event counter */
	L2C_REG_VALUE(0x200) |= (0x3 << 1); /* reset event counter */
	L2C_REG_VALUE(0x204) &= ~(0x3F);    /* clear event counter 1 source */
	L2C_REG_VALUE(0x208) &= ~(0x3F);    /* clear event counter 0 source */
	L2C_REG_VALUE(0x204) = cnt_src_1;   /* event counter 1 */
	L2C_REG_VALUE(0x208) = cnt_src_0;   /* event counter 0 */
	L2C_REG_VALUE(0x200) |= 0x1;        /* enable event counter */

	return count;

}

static int l2cc_proc_init(void)
{

	l2cc_proc_entry = create_proc_entry("l2cc", S_IFREG | S_IRUGO, cns3xxx_proc_dir);
	if (l2cc_proc_entry) {
		l2cc_proc_entry->read_proc = l2cc_read_proc;
		l2cc_proc_entry->write_proc = l2cc_write_proc;
	}
	return 0;
}
#endif

#define GPIOA_MEM_MAP_VALUE(reg_offset)	(*((uint32_t volatile *)(CNS3XXX_GPIOA_BASE_VIRT + reg_offset)))
#define GPIOA_INPUT				GPIOA_MEM_MAP_VALUE(0x004)
#define GPIOA_DIR					GPIOA_MEM_MAP_VALUE(0x008)

static void show_board_version(void)
{
   
	cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	cns3xxx_pwr_power_up(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	cns3xxx_pwr_soft_rst(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	
	printk("embeddedTS TS-4300\n");
	
}


static void __init ts43xx_ahci_init(void)
{
	u32 tmp;
	
	tmp = MISC_SATA_POWER_MODE;
	//tmp |= 0x1 << 16; /* Disable SATA PHY 0 from SLUMBER Mode */
	tmp &= ~(0x1 << 16); /* Force SATA PHY 0 into SLUMBER Mode */
	tmp |= 0x1 << 17; /* Disable SATA PHY 1 from SLUMBER Mode */
	MISC_SATA_POWER_MODE = tmp;
	
	/* Enable SATA PHY */
	// ts43xx only uses port 1
	//cns3xxx_pwr_power_up(0x1 << PM_PLL_HM_PD_CTRL_REG_OFFSET_SATA_PHY0);
	cns3xxx_pwr_power_up(0x1 << PM_PLL_HM_PD_CTRL_REG_OFFSET_SATA_PHY1);

	/* Enable SATA Clock */
	cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_SATA);
	
	/* De-Asscer SATA Reset */
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(SATA));
		
}


static void __init cns3xxx_init(void)
{
	int i;
 	
#ifdef CONFIG_CACHE_L2X0
	/* 1MB (128KB/way), 8-way associativity, evmon/parity/share enabled
	 * Bits:  .... ...0 0111 1001 0000 .... .... .... */
	cns3xxx_l2x0_init();
#endif

#ifdef CONFIG_CACHE_L2CC
	l2cc_init((void __iomem *) CNS3XXX_L2C_BASE_VIRT);
#endif

#ifdef CONFIG_CNS3XXX_DMAC
	dmac_init();
#endif

#ifdef CONFIG_CNS3XXX_RAID
	cns_rdma_init();
#endif

	show_board_version();
		
	{
	   volatile unsigned long *pciecfg0;
   
   pciecfg0 = (volatile unsigned long *)CNS3XXX_PCIE0_CFG0_BASE_VIRT;
   pciecfg0[1] |= 7;
   
   printk("usec at kernel start: 0x%08lX\n", *(volatile unsigned long *)(CNS3XXX_PCIE0_MEM_BASE_VIRT+0x14));
     
	}
	
   ts43xx_ahci_init();	
		
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		int ret;
		
		cns3xxx_pwr_power_up(CNS3XXX_PWR_PLL(PLL_LCD));
		cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(LCDC));
		cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(LCDC));

		ret = amba_device_register(d, &iomem_resource);
		if(ret)
			printk("%s=>%d: %d\n", __FUNCTION__, __LINE__, ret);
	}

	platform_add_devices(cns3xxx_devs, ARRAY_SIZE(cns3xxx_devs));
	
	lm_device_register(&cns3xxx_usb_otg_device);    
	i2c_register_board_info(0, cns3xxx_i2c_devices, ARRAY_SIZE(cns3xxx_i2c_devices));
	i2c_register_board_info(1, cns3xxx_i2c_gpio_devices, ARRAY_SIZE(cns3xxx_i2c_gpio_devices));
	spi_register_board_info(cns3xxx_spi_devices, ARRAY_SIZE(cns3xxx_spi_devices));

	
	cns3xxx_proc_dir = proc_mkdir("cns3xxx", NULL);
#ifdef CONFIG_DEBUG_FS
	cns3xxx_debugfs_dir = debugfs_create_dir("cns3xxx", NULL);
#endif

#ifdef CONFIG_CACHE_L2CC
	l2cc_proc_init();
#endif

	pm_power_off = cns3xxx_power_off;
	   
}


MACHINE_START(TS43XX, "embeddedTS TS-43XX ARM11 MPCore")
	.atag_offset	= CNS3XXX_DDR2SDRAM_BASE + 0x100,
	.fixup		= ts43xx_fixup,
	.map_io		= ts43xx_map_io,
	.init_irq	= cns3xxx_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &ts43xx_timer,
	.init_machine	= cns3xxx_init,
MACHINE_END

EXPORT_SYMBOL(cns3xxx_proc_dir);
