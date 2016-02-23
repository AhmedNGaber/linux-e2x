/*
 * rcar_du_lvdsenc_r8a7794x.c  --  R-Car Display Unit LVDS Encoder for R8A7794X
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: TODO E2X
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <drm/drm_encoder_slave.h>

#include "rcar_du_drv.h"
#include "rcar_du_encoder.h"
#include "rcar_du_lvdsenc_r8a7794x.h"
#include "rcar_lvds_regs_r8a7794x.h"

struct rcar_du_lvdsenc {
	struct rcar_du_device *dev;

	unsigned int index;
	void __iomem *mmio;
	struct clk *clock;
	int dpms;
	int mode;

	enum rcar_lvds_input input;
};

static void rcar_lvds_write(struct rcar_du_lvdsenc *lvds, u32 reg, u32 data)
{
	iowrite32(data, lvds->mmio + reg);
}

static u32 rcar_lvds_read(struct rcar_du_lvdsenc *lvds, u32 reg)
{
	return ioread32(lvds->mmio + reg);
}

static int rcar_du_lvdsenc_init_reg(struct rcar_du_lvdsenc *lvds,
				    struct rcar_du_crtc *rcrtc)
{
	u32 lclkselr = 0;
	u32 lpllsetr = 0;
	u32 lpllmonr = 0;

	/* TODO E2X-LVDS:
	 *  Clear the DU and LVDS module standby states using CPG: MSTPCR */

	/* Make settings of LVDS PLL and select a channel. */
	lclkselr = LCLKSELR_LVDS_IN_CLK_SEL_2_1_DU_DOTCLKIN0;
	lclkselr |= LCLKSELR_LVDS_NIDIV_SET(2);
	lclkselr &= ~LCLKSELR_LVDS_CLK_EN;
	rcar_lvds_write(lvds, LCLKSELR, lclkselr);

	lpllsetr = LPLLSETR_LVDSPLL_BP_NORMAL;
	lpllsetr |= LPLLSETR_LVDSPLL_FD(312);
	lpllsetr |= LPLLSETR_LVDSPLL_RD(5 - 1);
	lpllsetr |= LPLLSETR_LVDSPLL_OD(2);
	lpllsetr |= LPLLSETR_LVDSPLL_PD;
	rcar_lvds_write(lvds, LPLLSETR, lpllsetr);

	lclkselr |= LCLKSELR_LVDS_BIAS_PDN;
	rcar_lvds_write(lvds, LCLKSELR, lclkselr);

	/* Wait for at least 0.5 us. -> Wait for 1 us. */
	usleep_range(1, 5);

	/* Release power-down state. */
	lpllsetr &= ~LPLLSETR_LVDSPLL_PD;
	rcar_lvds_write(lvds, LPLLSETR, lpllsetr);

	/* Wait until the LVDS PLL is locked. */
	do {
		lpllmonr = rcar_lvds_read(lvds, LPLLMONR);
		/* TODO E2X-LVDS: Tune the waiting time. */
		usleep_range(1, 5);
	} while (lpllmonr == 0);

	/* Enable LVDS PLL output. */
	lclkselr |= LCLKSELR_LVDS_CLK_EN;

	/* Select the DU clock. */
	if (lvds->dev->info->lvds0_crtc & (0x01 << rcrtc->index)) {
		void __iomem *padsa2r_reg = NULL;
		u32 padsa2r_val = 0;

		padsa2r_reg = ioremap_nocache(PADSA2R_REGISTER, 0x04);
		if (rcrtc->index == 1) {
			lclkselr |= LCLKSELR_LVDS_IN_CLK_SEL_0_DU1;
			padsa2r_val = PADSA2R_DU1_LVDS |
				      (readl_relaxed(padsa2r_reg) & 0x00000300);
			lvds->input = RCAR_LVDS_INPUT_DU1;
		} else {
			lclkselr |= LCLKSELR_LVDS_IN_CLK_SEL_0_DU0;
			padsa2r_val = PADSA2R_DU0_LVDS |
				      (readl_relaxed(padsa2r_reg) & 0x00000C00);
			lvds->input = RCAR_LVDS_INPUT_DU0;
		}
		rcar_lvds_write(lvds, LCLKSELR, lclkselr);

		writel_relaxed(padsa2r_val, padsa2r_reg);
		iounmap(padsa2r_reg);
	}

	return 0;
}

int rcar_du_lvdsenc_r8a7794x_start(struct rcar_du_lvdsenc *lvds,
				   struct rcar_du_crtc *rcrtc)
{
	u32 lvdcr0;
	u32 lvdhcr;
	int ret;

	if (lvds->dpms == DRM_MODE_DPMS_ON)
		return 0;

	rcrtc->lvds_ch = lvds->index;

	/* software reset release */
	if (lvds->dev->info->lvds0_crtc & (0x01 << rcrtc->index)) {
		void __iomem *srstclr7_reg;
		srstclr7_reg = ioremap_nocache(SRSTCLR7_REGISTER, 0x04);
		writel_relaxed(SRCR7_LVDS0, srstclr7_reg);
		iounmap(srstclr7_reg);
	}

	ret = clk_prepare_enable(lvds->clock);
	if (ret < 0)
		return ret;

	ret = rcar_du_lvdsenc_init_reg(lvds, rcrtc);
	if (ret < 0)
		return ret;

	/* Hardcode the channels and control signals routing for now.
	 *
	 * HSYNC -> CTRL0
	 * VSYNC -> CTRL1
	 * DISP  -> CTRL2
	 * 0     -> CTRL3
	 */
	rcar_lvds_write(lvds, LVDCTRCR, LVDCTRCR_CTR3SEL_ZERO |
			LVDCTRCR_CTR2SEL_DISP | LVDCTRCR_CTR1SEL_VSYNC |
			LVDCTRCR_CTR0SEL_HSYNC);

#ifdef R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND
	if (rcar_du_needs(lvds->dev, RCAR_DU_QUIRK_LVDS_LANES))
		lvdhcr = LVDCHCR_CHSEL_CH(0, 0) | LVDCHCR_CHSEL_CH(1, 3)
		       | LVDCHCR_CHSEL_CH(2, 2) | LVDCHCR_CHSEL_CH(3, 1);
	else
		lvdhcr = LVDCHCR_CHSEL_CH(0, 0) | LVDCHCR_CHSEL_CH(1, 1)
		       | LVDCHCR_CHSEL_CH(2, 2) | LVDCHCR_CHSEL_CH(3, 3);
#else
	lvdhcr = LVDCHCR_CHSEL_CH(0, 0) | LVDCHCR_CHSEL_CH(1, 1)
	       | LVDCHCR_CHSEL_CH(2, 2) | LVDCHCR_CHSEL_CH(3, 3);
#endif

	rcar_lvds_write(lvds, LVDCHCR, lvdhcr);

	/* Select the input, hardcode mode 0, enable LVDS operation and turn
	 * bias circuitry on.
	 */
	lvdcr0 = LVDCR0_LVMD(lvds->mode);
	if (rcrtc->index == 1)
		lvdcr0 |= LVDCR0_DUSEL;
	rcar_lvds_write(lvds, LVDCR0, lvdcr0);

	/* TODO E2X-LVDS: Unnecessary waiting? */
	usleep_range(100, 150);

	/* LVDS output on. */
	lvdcr0 |= LVDCR0_LVRES;
	rcar_lvds_write(lvds, LVDCR0, lvdcr0);

	lvds->dpms = DRM_MODE_DPMS_ON;
	return 0;
}

int rcar_du_lvdsenc_r8a7794x_stop_suspend(struct rcar_du_lvdsenc *lvds)
{
	u32 lclkselr;
	u32 lpllsetr;

	if (lvds->dpms == DRM_MODE_DPMS_OFF)
		return -1;

	/* LVDS output off */
	rcar_lvds_write(lvds, LVDCR0, 0);

	/* Disable LVDS PLL output */
	lclkselr = rcar_lvds_read(lvds, LCLKSELR);
	lclkselr &= ~LCLKSELR_LVDS_CLK_EN;
	rcar_lvds_write(lvds, LCLKSELR, lclkselr);

	/* LVDS PLL power-down */
	lpllsetr = rcar_lvds_read(lvds, LPLLSETR);
	lpllsetr |= LPLLSETR_LVDSPLL_PD;
	rcar_lvds_write(lvds, LPLLSETR, lpllsetr);

	/* Wait for 1 us. */
	usleep_range(1, 5);

	clk_disable_unprepare(lvds->clock);

	/* software reset */
	if (lvds->dev->info->lvds0_crtc) {
		void __iomem *srcr7_reg;
		srcr7_reg = ioremap_nocache(SRCR7_REGISTER, 0x04);
		writel_relaxed(readl_relaxed(srcr7_reg) |
				SRCR7_LVDS0, srcr7_reg);
		iounmap(srcr7_reg);
	}

	lvds->dpms = DRM_MODE_DPMS_OFF;

	return 0;
}

void rcar_du_lvdsenc_r8a7794x_stop(struct rcar_du_lvdsenc *lvds)
{
	int ret;
	unsigned int i;

	ret = rcar_du_lvdsenc_r8a7794x_stop_suspend(lvds);
	if (ret < 0)
		return;

	for (i = 0; i < lvds->dev->pdata->num_crtcs; ++i)
		if (lvds->index == lvds->dev->crtcs[i].lvds_ch)
			lvds->dev->crtcs[i].lvds_ch = -1;
}

int rcar_du_lvdsenc_r8a7794x_dpms(struct rcar_du_lvdsenc *lvds,
			 struct drm_crtc *crtc, int mode)
{
	if (mode == DRM_MODE_DPMS_OFF) {
		rcar_du_lvdsenc_r8a7794x_stop(lvds);
		return 0;
	} else if (crtc) {
		struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
		return rcar_du_lvdsenc_r8a7794x_start(lvds, rcrtc);
	} else
		return -EINVAL;
}
