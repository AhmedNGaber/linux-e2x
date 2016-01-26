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
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_data/rcar-du.h>
#include <linux/platform_data/usb-rcar-gen2-phy.h>
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
#include <linux/platform_data/vsp1.h>
#endif
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

/* DU */
static struct rcar_du_encoder_data alex_du_encoders[] = {
	{
		.type = RCAR_DU_ENCODER_HDMI,
		.output = RCAR_DU_OUTPUT_DPAD0,
	},
	{
		.type = RCAR_DU_ENCODER_NONE,
		.output = RCAR_DU_OUTPUT_LVDS0,
		.connector.lvds.panel = {
			.width_mm = 246,
			.height_mm = 185,
			.mode = {
				.clock = 71000,
				.hdisplay = 1280,
				.hsync_start = 1360,
				.hsync_end = 1392,
				.htotal = 1424,
				.vdisplay = 800,
				.vsync_start = 803,
				.vsync_end = 804,
				.vtotal = 812,
				.flags = 0,
			},
		},
	},
	{
		.type = RCAR_DU_ENCODER_NONE,
		.output = RCAR_DU_OUTPUT_CVBS,
		.connector.cvbs = {
			.tvsys = 0,
			.modes = {
				{	/* 480i(525i) mode */
					.clock = 13500,
					.hdisplay = 720,
					.hsync_start = 737,
					.hsync_end = 797,
					.htotal = 858,
					.vdisplay = 480,
					.vsync_start = 489,
					.vsync_end = 501,
					.vtotal = 525,
					.flags = DRM_MODE_FLAG_INTERLACE |
						 DRM_MODE_FLAG_PVSYNC |
						 DRM_MODE_FLAG_PHSYNC,
				},
				{	/* 576i(625i) mode */
					.clock = 13500,
					.hdisplay = 720,
					.hsync_start = 732,
					.hsync_end = 796,
					.htotal = 864,
					.vdisplay = 576,
					.vsync_start = 581,
					.vsync_end = 586,
					.vtotal = 625,
					.flags = DRM_MODE_FLAG_INTERLACE |
						 DRM_MODE_FLAG_PVSYNC |
						 DRM_MODE_FLAG_PHSYNC,
				},
			},
		},
	},
};

static struct rcar_du_crtc_data alex_du_crtcs[] = {
	{
		.exclk = 74250000,
		.init_conn_type = DRM_MODE_CONNECTOR_HDMIA,
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		.vsp = CONFIG_DRM_RCAR_DU0_USE_VSPDU_CH,
#endif
	},
	{
		.exclk = 74250000,
		.init_conn_type = DRM_MODE_CONNECTOR_Composite,
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		.vsp = CONFIG_DRM_RCAR_DU1_USE_VSPDU_CH,
#endif
	},
};

static int alex_lvds_backlight_on(void)
{
	int ret;
	struct device_node *np;
	int gpio;

	np = of_find_node_by_path("/gpio@e6055000");
	if (np) {
		gpio = of_get_gpio(np, 31);
		of_node_put(np);
	} else {
		pr_warn("Error: Unable to set backlight to ON\n");
		ret = -ENOTSUPP;
		goto error2;
	}

	gpio_request_one(gpio, GPIOF_INIT_HIGH, NULL);
	if (!gpio_get_value(gpio)) {
		pr_warn("Error: LVDS backlight is not enable\n");
		ret = -ENOTSUPP;
		goto error;
	}

	return 0;
 error:
	gpio_free(gpio);
 error2:
	return ret;
}

static int alex_lvds_backlight_off(void)
{
	int ret;
	struct device_node *np;
	int gpio;

	np = of_find_node_by_path("/gpio@e6055000");
	if (np) {
		gpio = of_get_gpio(np, 31);
		of_node_put(np);
	} else {
		pr_warn("Error: Unable to set backlight to OFF\n");
		ret = -ENOTSUPP;
		goto error2;
	}

	gpio_free(gpio);

	return 0;
 error2:
	return ret;
}

static struct rcar_du_platform_data alex_du_pdata = {
	.encoders = alex_du_encoders,
	.num_encoders = ARRAY_SIZE(alex_du_encoders),
	.crtcs = alex_du_crtcs,
	.num_crtcs = ARRAY_SIZE(alex_du_crtcs),
#ifdef CONFIG_DRM_FBDEV_CRTC
	.fbdev_crtc = 0,
#endif
	.backlight_on = alex_lvds_backlight_on,
	.backlight_off = alex_lvds_backlight_off,
	.i2c_ch = 1,
};

static const struct resource du_resources[] __initconst = {
	DEFINE_RES_MEM(0xfeb00000, 0x40000),
	DEFINE_RES_MEM_NAMED(0xfeb80000, 0x30a, "cvbs.0"),
	DEFINE_RES_MEM_NAMED(0xfeb90000, 0x1c, "lvds.0"),
	DEFINE_RES_IRQ(gic_spi(256)),
	DEFINE_RES_IRQ(gic_spi(268)),
};

static void __init alex_add_du_device(void)
{
	struct platform_device_info info = {
		.name = "rcar-du-r8a7794x",
		.id = -1,
		.res = du_resources,
		.num_res = ARRAY_SIZE(du_resources),
		.data = &alex_du_pdata,
		.size_data = sizeof(alex_du_pdata),
		.dma_mask = DMA_BIT_MASK(32),
	};

	platform_device_register_full(&info);
}


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
	{ "du0", "du.0", "rcar-du-r8a7794x" },
	{ "du1", "du.1", "rcar-du-r8a7794x" },
	{ "lvds0", "lvds.0", "rcar-du-r8a7794x" },
	{ "dvenc", "cvbs.0", "rcar-du-r8a7794x" },
	{ "vsps", NULL, NULL },
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
	{ "vsp1-du0", NULL, "vsp1.2" },
#else
	{ "vsp1-du0", NULL, NULL },
#endif
	{ "vpc0", NULL, "vpc1" },
	{ "2ddmac", NULL, "tddmac" },
	{ "fdp0", NULL, "fdp0" },
	{ "pvrsrvkm", NULL, "pvrsrvkm" },
};

/*
 * This is a really crude hack to work around core platform clock issues
 */
static const struct clk_name clk_enables[] __initconst = {
	{ "ether", NULL, "ee700000.ethernet" },
	{ "vcp0", NULL, "vcp1" },
	{ "dmal", NULL, "sh-dma-engine.0" },
};

/* POWER IC */
#define DA9063_REG_CONTROL_F	0x13
#define DA9063_REG_LDO5_CONT	0x2a

static struct i2c_board_info poweric_i2c[] = {
	{ I2C_BOARD_INFO("da9063", 0x5A), },
};

static void alex_restart(char mode, const char *cmd)
{
	struct i2c_adapter *adap;
	struct i2c_client *client;
	u8 val;
	int busnum = 1;
	s32 ret;

	adap = i2c_get_adapter(busnum);
	if (!adap) {
		pr_err("failed to get adapter i2c%d\n", busnum);
		return;
	}

	client = i2c_new_device(adap, &poweric_i2c[0]);
	if (!client)
		pr_err("failed to register %s to i2c%d\n",
		       poweric_i2c[0].type, busnum);

	i2c_put_adapter(adap);

	ret = i2c_smbus_read_byte_data(client, DA9063_REG_LDO5_CONT);

	if (ret < 0) {
		pr_err("couldn't access da9063 reg 0x%x err=%d, aborting\n",
			DA9063_REG_LDO5_CONT, ret);
		return;
	}

	val = ret | 0x08;

	i2c_smbus_write_byte_data(client, DA9063_REG_LDO5_CONT, val);

	ret = i2c_smbus_read_byte_data(client, DA9063_REG_CONTROL_F);

	if (ret < 0)
		pr_err("couldn't access da9063 reg 0x%x err=%d\n",
			DA9063_REG_CONTROL_F, ret);

	val = ret | 0x02;

	i2c_smbus_write_byte_data(client, DA9063_REG_CONTROL_F, val);
}

/* VSP1 */
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
static const struct vsp1_platform_data alex_vsps_pdata __initconst = {
	.features = 0,
	.rpf_count = 5,
	.uds_count = 3,
	.wpf_count = 4,
};

static const struct vsp1_platform_data alex_vspd0_pdata __initconst = {
	.features = VSP1_HAS_LIF,
	.rpf_count = 4,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data * const alex_vsp1_pdata[] __initconst
									= {
	&alex_vsps_pdata,
	&alex_vspd0_pdata,
};

static const struct resource vsp1_1_resources[] __initconst = {
	DEFINE_RES_MEM(0xfe928000, 0x8000),
	DEFINE_RES_IRQ(gic_spi(267)),
};

static const struct resource vsp1_2_resources[] __initconst = {
	DEFINE_RES_MEM(0xfe930000, 0x8000),
	DEFINE_RES_IRQ(gic_spi(246)),
};

static const struct resource * const vsp1_resources[] __initconst = {
	vsp1_1_resources,
	vsp1_2_resources,
};

static void __init alex_add_vsp1_devices(void)
{
	struct platform_device_info info = {
		.name = "vsp1",
		.size_data = sizeof(*alex_vsp1_pdata[0]),
		.num_res = 2,
		.dma_mask = DMA_BIT_MASK(32),
	};
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(vsp1_resources); ++i) {
		info.id = i + 1;
		info.data = alex_vsp1_pdata[i];
		info.res = vsp1_resources[i];

		platform_device_register_full(&info);
	}
}
#endif

static void __init alex_add_standard_devices(void)
{
	shmobile_clk_workaround(clk_names, ARRAY_SIZE(clk_names), false);
	shmobile_clk_workaround(clk_enables, ARRAY_SIZE(clk_enables), true);
	r8a7794x_add_dt_devices();
	of_platform_populate(NULL, of_default_bus_match_table,
			     NULL, NULL);
	alex_add_du_device();
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
	alex_add_vsp1_devices();
#endif
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
	.restart	= alex_restart,
	.dt_compat	= ale6_boards_compat_dt,
MACHINE_END
