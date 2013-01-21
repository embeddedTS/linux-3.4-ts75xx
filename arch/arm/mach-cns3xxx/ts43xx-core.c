/*
 *  linux/arch/arm/mach-cns3xxx/core.c
 *
 *  Copyright (c) 2008 Cavium Networks 
 *  Copyright (C) 1999 - 2003 ARM Limited
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
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/delay.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/ata_platform.h>
#include <linux/jiffies.h>

#include <asm/clkdev.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/hardware/arm_timer.h>
//#include <asm/hardware/icst307.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <asm/hardware/gic.h>

#include <mach/cns3xxx.h>
#include <mach/irqs.h>
#include <mach/pm.h>

#include "core.h"

/* used by entry-macro.S */
void __iomem *gic_cpu_base_addr;

/*
 * This is the Cavium Networks sched_clock implementation.  This has
 * a resolution of 41.7ns, and a maximum value of about 179s.
 */
/*
unsigned long long sched_clock(void)
{
	unsigned long long v;

	v = (unsigned long long) 30000000 * 125;
	do_div(v, 3);

	return v;
}
*/
static int cns3xxx_flash_init(void)
{

	return 0;
}

static void cns3xxx_flash_exit(void)
{

}

static void cns3xxx_flash_set_vpp(int on)
{

}

static struct flash_platform_data cns3xxx_flash_data = {
	.map_name		= "cfi_probe",
	.width			= 2,
	.init			= cns3xxx_flash_init,
	.exit			= cns3xxx_flash_exit,
	.set_vpp		= cns3xxx_flash_set_vpp,
};

struct platform_device cns3xxx_flash_device = {
	.name			= "cns3xxxflash",
	.id			= 0,
	.dev			= {
		.platform_data	= &cns3xxx_flash_data,
	},
};

int cns3xxx_flash_register(struct resource *res, u32 num)
{
	cns3xxx_flash_device.resource = res;
	cns3xxx_flash_device.num_resources = num;
	return platform_device_register(&cns3xxx_flash_device);
}

/*
static const struct icst307_params cns3xxx_oscvco_params = {
	.ref = 24000,
	.vco_max = 200000,
	.vd_min = 4+8,
	.vd_max = 511+8,
	.rd_min = 1+2,
	.rd_max = 127+2,
};
*/
static u64 cns3xxx_sata_dma_mask = 0xffffffffULL;
struct platform_device cns3xxx_sata_device = {
	.name			= "sata dwc",
	.id			= 0,
	.dev			= {
		.dma_mask       = &cns3xxx_sata_dma_mask,
		.coherent_dma_mask = 0xffffffffULL,
	},
};

int cns3xxx_sata_register(struct resource *res, u32 num)
{
	cns3xxx_sata_device.resource = res;
	cns3xxx_sata_device.num_resources = num;
	return platform_device_register(&cns3xxx_sata_device);
}



/*
 * CLCD support.
 */
#define SYS_CLCD_NLCDIOON	(1 << 2)
#define SYS_CLCD_VDDPOSSWITCH	(1 << 3)
#define SYS_CLCD_PWR3V5SWITCH	(1 << 4)
#define SYS_CLCD_ID_MASK	(0x1f << 8)
#define SYS_CLCD_ID_SANYO_3_8	(0x00 << 8)
#define SYS_CLCD_ID_UNKNOWN_8_4	(0x01 << 8)
#define SYS_CLCD_ID_EPSON_2_2	(0x02 << 8)
#define SYS_CLCD_ID_AMPIRE_4_0	(0x03 << 8)
#define SYS_CLCD_ID_SANYO_2_5	(0x07 << 8)
#define SYS_CLCD_ID_XGA		(0x08 << 8)
#define SYS_CLCD_ID_SVGA	(0x09 << 8)
#define SYS_CLCD_ID_INNOLUX_7_0	(0x0A << 8)
#define SYS_CLCD_ID_VGA		(0x1f << 8)

#ifdef CONFIG_CNS3XXX_DISP_7INCH
static struct clcd_panel innolux_7_0_in = {
	.mode		= {
		.name		= "Innolux 7.0",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 30030,
		.left_margin	= 46,
		.right_margin	= 200,
		.upper_margin	= 23,
		.lower_margin	= 20,
		.hsync_len	= 10,
		.vsync_len	= 2,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 16,
};
#endif


#ifdef CONFIG_CNS3XXX_DISP_XGA
static struct clcd_panel xga = {
	.mode		= {
		.name		= "XGA",
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 768,
		.pixclock	= 15384,
		.left_margin	= 160,
		.right_margin	= 24,
		.upper_margin	= 29,
		.lower_margin	= 3,
		.hsync_len	= 136,
		.vsync_len	= 6,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 16,
};
#endif

#ifdef CONFIG_CNS3XXX_DISP_SVGA
static struct clcd_panel svga = {
	.mode		= {
		.name		= "SVGA",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 600,
		.pixclock	= 25000,
		.left_margin	= 88,
		.right_margin	= 40,
		.upper_margin	= 23,
		.lower_margin	= 1,
		.hsync_len	= 128,
		.vsync_len	= 4,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 16,
};
#endif

#ifdef CONFIG_CNS3XXX_DISP_VGA
static struct clcd_panel vga = {
	.mode		= {
		.name		= "VGA",
		.refresh	= 60,
		.xres		= 640,
		.yres		= 480,
		.pixclock	= 39721,
		.left_margin	= 40,
		.right_margin	= 24,
		.upper_margin	= 32,
		.lower_margin	= 11,
		.hsync_len	= 96,
		.vsync_len	= 2,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 32,
};
#endif

//#ifdef CONFIG_CNS3XXX_DISP_4INCH
static struct clcd_panel ampire_4_0_in = {
	.mode		= {
		.name		= "Ampire 4.0",
		.refresh	= 60,
		.xres		= 480,
		.yres		= 272,
		.pixclock	= 111111,
		.left_margin	= 6,
		.right_margin	= 6,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 33,
		.vsync_len	= 10,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 32,

};
//#endif

/*
 * Detect which LCD panel is connected, and return the appropriate
 * clcd_panel structure.  Note: we do not have any information on
 * the required timings for the 8.4in panel, so we presently assume
 * VGA timings.
 */
static struct clcd_panel *cns3xxx_clcd_panel(void)
{
	/* void __iomem *sys_clcd = __io_address(CNS3XXX_SYS_BASE) + CNS3XXX_SYS_CLCD_OFFSET; */
	struct clcd_panel *panel = &ampire_4_0_in;

#if defined (CONFIG_CNS3XXX_DISP_4INCH)
	panel = &ampire_4_0_in;
#elif defined (CONFIG_CNS3XXX_DISP_7INCH)
	panel = &innolux_7_0_in;
#elif defined (CONFIG_CNS3XXX_DISP_VGA)
	panel = &vga;
#elif defined (CONFIG_CNS3XXX_DISP_SVGA)
	panel = &svga;
#elif defined (CONFIG_CNS3XXX_DISP_XGA)
	panel = &xga;
#else
	panel = &ampire_4_0_in;
#endif

	return panel;
}


/*
 * Disable all display connectors on the interface module.
 */
static void cns3xxx_clcd_disable(struct clcd_fb *fb)
{
/*
	void __iomem *sys_clcd = __io_address(CNS3XXX_SYS_BASE) + CNS3XXX_SYS_CLCD_OFFSET;
	u32 val;

	val = readl(sys_clcd);
	val &= ~SYS_CLCD_NLCDIOON | SYS_CLCD_PWR3V5SWITCH;
	writel(val, sys_clcd);
*/
}

/*
 * Enable the relevant connector on the interface module.
 */
static void cns3xxx_clcd_enable(struct clcd_fb *fb)
{
/*
	void __iomem *sys_clcd = __io_address(CNS3XXX_SYS_BASE) + CNS3XXX_SYS_CLCD_OFFSET;
	u32 val;
*/

	/*
	 * Enable the PSUs
	 */
/*
	val = readl(sys_clcd);
	val |= SYS_CLCD_NLCDIOON | SYS_CLCD_PWR3V5SWITCH;
	writel(val, sys_clcd);
*/
}

static unsigned long framesize = SZ_4M;

static int cns3xxx_clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	fb->panel		= cns3xxx_clcd_panel();

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, framesize,
						    &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start	= dma;
	fb->fb.fix.smem_len	= framesize;

	return 0;
}

static int cns3xxx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
}

static void cns3xxx_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			      fb->fb.screen_base, fb->fb.fix.smem_start);
}

struct clcd_board clcd_plat_data = {
	.name		= "CNS3XXX",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.disable	= cns3xxx_clcd_disable,
	.enable		= cns3xxx_clcd_enable,
	.setup		= cns3xxx_clcd_setup,
	.mmap		= cns3xxx_clcd_mmap,
	.remove		= cns3xxx_clcd_remove,
};




/**********************************************************************
   ____            _                   _____ _                     
  / ___| _   _ ___| |_ ___ _ __ ___   |_   _(_)_ __ ___   ___ _ __ 
  \___ \| | | / __| __/ _ \ '_ ` _ \    | | | | '_ ` _ \ / _ \ '__|
   ___) | |_| \__ \ ||  __/ | | | | |   | | | | | | | | |  __/ |   
  |____/ \__, |___/\__\___|_| |_| |_|   |_| |_|_| |_| |_|\___|_|   
         |___/                         
	 
**********************************************************************/
/*
 * Where is the timer (VA)?
 */
void __iomem *timer1_va_base;
u32 timer1_reload;
u64 timer1_ticks         = 0;

#define KHZ			(1000)
#define MHZ			(1000*1000)
#define CNS3XXX_PCLK		(cns3xxx_cpu_clock() >> 3)
/* CONFIG_HZ = 100 => 1 tick = 10 ms */
#define NR_CYCLES_PER_TICK	((CNS3XXX_PCLK * MHZ) / CONFIG_HZ)

/* Timer 1, 2, and 3 Control Register */
#define TIMER1_ENABLE		(1 << 0)
#define TIMER2_ENABLE		(1 << 3)
#define TIMER3_ENABLE		(1 << 6)
#define TIMER1_USE_1KHZ_SOURCE	(1 << 1)
#define TIMER2_USE_1KHZ_SOURCE	(1 << 4)
#define TIMER3_USE_1KHZ_SOURCE	(1 << 7)
#define TIMER1_INTR_ENABLE	(1 << 2)
#define TIMER2_INTR_ENABLE	(1 << 5)
#define TIMER3_INTR_ENABLE	(1 << 8)
#define TIMER1_DOWN_COUNT	(1 << 9)
#define TIMER2_DOWN_COUNT	(1 << 10)
#define TIMER3_DOWN_COUNT	(1 << 11)

/* Timer 1, 2, and 3 Interrupt Status */
#define TIMER1_OVERFLOW		(1 << 2)
#define TIMER2_OVERFLOW		(1 << 5)
#define TIMER3_OVERFLOW		(1 << 8)

/* Free running Timer */
#define TIMER4_ENABLE		(1 << 17)
#define TIMER4_RESET		(1 << 16)
#define TIMER4_COUNTER_BITS	(0x0000FFFF)

static void timer1_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	u32 ctrl = readl(timer1_va_base + TIMER1_2_CONTROL_OFFSET);

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		ctrl |= TIMER1_ENABLE | TIMER1_INTR_ENABLE;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl |= TIMER1_INTR_ENABLE;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl &= ~(TIMER1_ENABLE | TIMER3_ENABLE);
	}

	writel(ctrl, timer1_va_base + TIMER1_2_CONTROL_OFFSET);
}

static int timer1_set_next_event(unsigned long evt,
				struct clock_event_device *unused)
{
	u32 ctrl = readl(timer1_va_base + TIMER1_2_CONTROL_OFFSET);

	writel(evt, timer1_va_base + TIMER1_AUTO_RELOAD_OFFSET);
	writel(ctrl | TIMER1_ENABLE, timer1_va_base + TIMER1_2_CONTROL_OFFSET);

	return 0;
}

static struct clock_event_device timer1_ce =	 {
	.name		= "timer1",
	.rating		= 350,
	.shift		= 8,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= timer1_set_mode,
	.set_next_event	= timer1_set_next_event,
	.cpumask	= cpu_all_mask,
};

static cycle_t timer1_get_cycles(struct clocksource *cs)
{
	u32 current_counter = readl(timer1_va_base + TIMER1_COUNTER_OFFSET);
	u64 tmp = timer1_ticks * NR_CYCLES_PER_TICK;

	return (cycle_t)(tmp + timer1_reload - current_counter);
}


/* PCLK clock source (default is 75 MHz @ CPU 300 MHz) */
static struct clocksource timer1_cs = {
	.name		= "timer1",
	.rating		= 500,
	.read		= timer1_get_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 8,
	.flags		= 0
};



static void __init cns3xxx_clockevents_init(unsigned int timer_irq)
{
	timer1_ce.irq = timer_irq;
	timer1_ce.mult =
		div_sc( (CNS3XXX_PCLK * MHZ), NSEC_PER_SEC, timer1_ce.shift);
	timer1_ce.max_delta_ns = clockevent_delta2ns(0xffffffff, &timer1_ce);
	timer1_ce.min_delta_ns = clockevent_delta2ns(0xf       , &timer1_ce);

	clockevents_register_device(&timer1_ce);
}

/*
 * IRQ handler for the timer
 */

static irqreturn_t cns3xxx_timer1_interrupt(int irq, void *dev_id)
{
   static int x;
	u32 val;
	struct clock_event_device *evt = dev_id;
	
	val = readl(timer1_va_base + TIMER1_2_INTERRUPT_STATUS_OFFSET);
	if (val & TIMER1_OVERFLOW) {
		timer1_ticks++;
		evt->event_handler(evt);
		/* Clear the interrupt */
		writel(val & ~(TIMER1_OVERFLOW), timer1_va_base + TIMER1_2_INTERRUPT_STATUS_OFFSET);
	}
	else {
		printk("%s Unexpected interrupt(status=%08x)....", __func__, val);
		writel(val, timer1_va_base + TIMER1_2_INTERRUPT_STATUS_OFFSET);
	}

	return IRQ_HANDLED;
}
 
static struct irqaction cns3xxx_timer1_irq = {
	.name		= "timer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= cns3xxx_timer1_interrupt,
	.dev_id		= &timer1_ce,
};

static cycle_t timer2_get_cycles(struct clocksource *cs)
{
	return (cycle_t)readl(timer1_va_base + TIMER2_COUNTER_OFFSET);
}

/* Configured to use 1K Hz clock source */
static struct clocksource timer2_cs = {
	.name		= "timer2_1khz",
	.rating		= 100,
	.read		= timer2_get_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 8,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static cycle_t timer4_get_cycles(struct clocksource *cs)
{
	u64 tmp;
	tmp = (readl(timer1_va_base + TIMER_FREERUN_CONTROL_OFFSET) & TIMER4_COUNTER_BITS) << 16;
	tmp |= readl(timer1_va_base + TIMER_FREERUN_OFFSET);

	return (cycle_t)tmp;
}

/* Timer4 is a free run 100K Hz timer. */
static struct clocksource timer4_cs = {
	.name		= "timer4_100khz",
	.rating		= 200,
	.read		= timer4_get_cycles,
	.mask		= CLOCKSOURCE_MASK(48),
	.shift		= 8,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init cns3xxx_clocksource_init(void)
{
	timer4_cs.mult = clocksource_hz2mult(100 * KHZ,            timer4_cs.shift);
	timer2_cs.mult = clocksource_hz2mult(1 * KHZ,              timer2_cs.shift);
	timer1_cs.mult = clocksource_hz2mult((CNS3XXX_PCLK * MHZ), timer1_cs.shift);

	clocksource_register(&timer4_cs);
	clocksource_register(&timer2_cs);
	clocksource_register(&timer1_cs);
}

#include <linux/tick.h>

//when we change the cpu clock, we need to call this function to change timer1's parameter.
void cns3xxx_timer1_change_clock(void)
{
	u32 counter=*(volatile unsigned int *) (CNS3XXX_TIMER1_2_3_BASE_VIRT + TIMER1_COUNTER_OFFSET);

	/************ timer1 ************/
#ifdef CONFIG_SILICON
	timer1_reload = NR_CYCLES_PER_TICK;
#else
	timer1_reload = 0x25000;
#endif

	if (timer1_reload < counter) {
		*(volatile unsigned int *) (CNS3XXX_TIMER1_2_3_BASE_VIRT + TIMER1_COUNTER_OFFSET)
			= timer1_reload;
	}

	/* for timer, because CPU clock is changed */
	*(volatile unsigned int *) (CNS3XXX_TIMER1_2_3_BASE_VIRT + TIMER1_AUTO_RELOAD_OFFSET)
			= timer1_reload;

	timer1_cs.mult = clocksource_hz2mult((CNS3XXX_PCLK * MHZ), timer1_cs.shift);
// FIX ME!	
//	timer1_cs.mult_orig = timer1_cs.mult;

	timer1_ce.mult =
		div_sc( (CNS3XXX_PCLK * MHZ), NSEC_PER_SEC, timer1_ce.shift);
	timer1_ce.max_delta_ns = clockevent_delta2ns(0xffffffff, &timer1_ce);
	timer1_ce.min_delta_ns = clockevent_delta2ns(0xf       , &timer1_ce);

	timer1_cs.cycle_last = 0;
//	timer1_cs.cycle_last = clocksource_read(&timer1_cs);
//	timer1_cs.error = 0;
// timer1_cs.xtime_nsec = 0;
//	clocksource_calculate_interval(&timer1_cs, NTP_INTERVAL_LENGTH);

	tick_clock_notify();

	//printk("timer1_reload v22: %u", timer1_reload);
}

/*
 * Set up the clock source and clock events devices
 */
void __init __ts43xx_timer_init(unsigned int timer_irq)
{
	u32 val, irq_mask; 

	/*
	 * Initialise to a known state (all timers off)
	 */
	writel(0, timer1_va_base + TIMER1_2_CONTROL_OFFSET);		/* Disable timer1, 2 and 3 */
	writel(0, timer1_va_base + TIMER_FREERUN_CONTROL_OFFSET);	/* Stop free running timer4 */


	/************ timer1 ************/
#ifdef CONFIG_SILICON
	timer1_reload = NR_CYCLES_PER_TICK;
#else
	timer1_reload = 0x25000;
#endif
	writel(timer1_reload, timer1_va_base + TIMER1_COUNTER_OFFSET);
	writel(timer1_reload, timer1_va_base + TIMER1_AUTO_RELOAD_OFFSET);

	writel(0xFFFFFFFF, timer1_va_base + TIMER1_MATCH_V1_OFFSET);
	writel(0xFFFFFFFF, timer1_va_base + TIMER1_MATCH_V2_OFFSET);
	/* mask irq, non-mask timer1 overflow */
	irq_mask = readl(timer1_va_base + TIMER1_2_INTERRUPT_MASK_OFFSET);
	irq_mask &= ~(1 << 2);
	irq_mask |= 0x03;
	writel(irq_mask, timer1_va_base + TIMER1_2_INTERRUPT_MASK_OFFSET);
	/* down counter */
	val = readl(timer1_va_base + TIMER1_2_CONTROL_OFFSET);
	val |= TIMER1_DOWN_COUNT;
	writel(val, timer1_va_base + TIMER1_2_CONTROL_OFFSET);


	/************ timer2 ************/
	/* Configure timer2 as periodic free-running clocksource, interrupt disabled. */
	writel(0,          timer1_va_base + TIMER2_COUNTER_OFFSET);
	writel(0xFFFFFFFF, timer1_va_base + TIMER2_AUTO_RELOAD_OFFSET);

	writel(0xFFFFFFFF, timer1_va_base + TIMER2_MATCH_V1_OFFSET);
	writel(0xFFFFFFFF, timer1_va_base + TIMER2_MATCH_V2_OFFSET);
	/* mask irq */
	irq_mask = readl(timer1_va_base + TIMER1_2_INTERRUPT_MASK_OFFSET);
	irq_mask |= ((1 << 3) | (1 << 4) | (1 << 5));
	writel(irq_mask,   timer1_va_base + TIMER1_2_INTERRUPT_MASK_OFFSET);
	/* Enable timer2 /Use 1K Hz clock source / Up count */
	val = readl(timer1_va_base + TIMER1_2_CONTROL_OFFSET);
	val |= TIMER2_ENABLE | TIMER2_USE_1KHZ_SOURCE;
	writel(val, timer1_va_base + TIMER1_2_CONTROL_OFFSET);


	/************ timer3 ************/
	/* Not enabled */

	/************ timer4 ************/
	writel(TIMER4_RESET,  timer1_va_base + TIMER_FREERUN_CONTROL_OFFSET);
	writel(TIMER4_ENABLE, timer1_va_base + TIMER_FREERUN_CONTROL_OFFSET);


	/* 
	 * Make irqs happen for the system timer
	 */
	/* Clear all interrupts */
	writel(0x000001FF, timer1_va_base + TIMER1_2_INTERRUPT_STATUS_OFFSET);
	setup_irq(timer_irq, &cns3xxx_timer1_irq);

	cns3xxx_clocksource_init();
	cns3xxx_clockevents_init(timer_irq);
}

static void __init ts43xx_timer_init(void)
{
      
	timer1_va_base = IOMEM(CNS3XXX_TIMER1_2_3_BASE_VIRT);
	
	__ts43xx_timer_init(IRQ_CNS3XXX_TIMER0);
	
		
}

struct sys_timer ts43xx_timer = {
	.init = ts43xx_timer_init,
};


