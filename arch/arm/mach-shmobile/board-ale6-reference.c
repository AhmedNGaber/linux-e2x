/*
 * Ale6 board support - Reference DT implementation
 *
 * Copyright (C) 2015-2016  Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_data/rcar-du.h>
#include <linux/platform_data/usb-rcar-gen2-phy.h>
#include <linux/platform_data/vsp1.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/usb/phy.h>
#include <linux/usb/renesas_usbhs.h>
#include <asm/mach/arch.h>
#include <sound/rcar_snd.h>
#include <sound/simple_card.h>

#include "clock.h"
#include "common.h"
#include "dma-register.h"
#include "irqs.h"
#include "r8a7794x.h"
#include "rcar-gen2.h"



/*
 * This is a really crude hack to provide clkdev support to platform
 * devices until they get moved to DT.
 */
static const struct clk_name clk_names[] __initconst = {
	{ "cmt0", NULL, "sh_cmt.0" },
	{ "scif0", NULL, "sh-sci.6" },
	{ "scif1", NULL, "sh-sci.7" },
	{ "hscif0", NULL, "sh-sci.8" },
	{ "hscif1", NULL, "sh-sci.9" },
	{ "scif2", NULL, "sh-sci.10" },
	{ "scif3", NULL, "sh-sci.11" },
	{ "scif4", NULL, "sh-sci.12" },
	{ "scif5", NULL, "sh-sci.13" },
	{ "hscif2", NULL, "sh-sci.17" },
};

/*
 * This is a really crude hack to work around core platform clock issues
 */
static const struct clk_name clk_enables[] __initconst = {
	{ "ether", NULL, "ee700000.ethernet" },
};

static void __init alex_add_standard_devices(void)
{
	shmobile_clk_workaround(clk_names, ARRAY_SIZE(clk_names), false);
	shmobile_clk_workaround(clk_enables, ARRAY_SIZE(clk_enables), true);
	r8a7794x_add_dt_devices();
	of_platform_populate(NULL, of_default_bus_match_table,
			     NULL, NULL);
}

static const char * const ale6_boards_compat_dt[] __initconst = {
	"renesas,ale6",
	"renesas,ale6-reference",
	NULL,
};

DT_MACHINE_START(ALEX_DT, "ale6")
	.smp		= smp_ops(r8a7794x_smp_ops),
	.init_early	= r8a7794x_init_early,
	.init_time	= r8a7794x_timer_init,
	.init_machine	= alex_add_standard_devices,
	.init_late	= shmobile_init_late,
	.dt_compat	= ale6_boards_compat_dt,
MACHINE_END
