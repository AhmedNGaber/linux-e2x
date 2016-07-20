/*
 * rcar_du_cvbsenc.c  --  R-Car Display Unit CVBS Encoder
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: TODO E2X: contact
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
#include "rcar_du_cvbsenc.h"
#include "rcar_cvbs_regs.h"

struct rcar_du_cvbsenc {
	struct rcar_du_device *dev;

	unsigned int index;
	void __iomem *mmio;
	struct clk *clock;
	int dpms;

	enum rcar_cvbs_input input;
	enum rcar_cvbs_output_tvsys tvsys;
};

static void rcar_cvbs_write(struct rcar_du_cvbsenc *cvbs, u32 reg, u16 data)
{
	iowrite16(data, cvbs->mmio + reg);
}

void rcar_du_cvbsenc_set_tvsys(struct rcar_du_cvbsenc *cvbs,
			       enum rcar_cvbs_output_tvsys tvsys)
{
	cvbs->tvsys = tvsys;

	return;
}

enum rcar_cvbs_output_tvsys
rcar_du_cvbsenc_get_tvsys(struct rcar_du_cvbsenc *cvbs)
{
	return cvbs->tvsys;
}

void rcar_du_cvbsenc_set_operating_mode(struct rcar_du_cvbsenc *cvbs)
{
	u16 val = 0;

	val = IDENCMD1_SRL_EN;

	switch (cvbs->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
		val |= IDENCMD1_WSS_OFF | IDENCMD1_SUBC_FREQ_3_58 |
		       IDENCMD1_SUBC_PHCTRL_ON | IDENCMD1_FLD_FREQ_525_60 |
		       IDENCMD1_CDM_NTSC;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
		val |= IDENCMD1_WSS_OFF | IDENCMD1_SUBC_FREQ_3_58 |
		       IDENCMD1_SUBC_PHCTRL_ON | IDENCMD1_FLD_FREQ_525_60 |
		       IDENCMD1_CDM_PAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
		val |= IDENCMD1_WSS_OFF | IDENCMD1_SUBC_FREQ_4_43 |
		       IDENCMD1_SUBC_PHCTRL_OFF | IDENCMD1_FLD_FREQ_525_60 |
		       IDENCMD1_CDM_PAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
		val |= IDENCMD1_WSS_OFF | IDENCMD1_SUBC_FREQ_4_43 |
		       IDENCMD1_SUBC_PHCTRL_ON | IDENCMD1_FLD_FREQ_625_50 |
		       IDENCMD1_CDM_PAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		val |= IDENCMD1_WSS_OFF | IDENCMD1_SUBC_FREQ_3_58 |
		       IDENCMD1_SUBC_PHCTRL_ON | IDENCMD1_FLD_FREQ_625_50 |
		       IDENCMD1_CDM_PAL;
		break;
	default:
		/* DO NOTHING */
		break;
	}

	rcar_cvbs_write(cvbs, IDENCMD1, val);

	val = IDENCMD2_ADJ_COL_UPLIM_ON | IDENCMD2_ADJ_COL_LOLIM_ON |
	      IDENCMD2_ADJ_LUM_UPLIM_ON | IDENCMD2_ADJ_LUM_LOLIM_ON |
	      IDENCMD2_COL_UPLIM_ON | IDENCMD2_COL_LOLIM_ON |
	      IDENCMD2_LUM_UPLIM_ON | IDENCMD2_LUM_LOLIM_ON |
	      IDENCMD2_Y1SETUP_OFF;

	rcar_cvbs_write(cvbs, IDENCMD2, val);
}

void rcar_du_cvbsenc_set_tint_burst_amplitude(struct rcar_du_cvbsenc *cvbs)
{
	u16 val = 0;

	switch (cvbs->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
		val = ITNTBLEV_BURST(0xc9) | ITNTBLEV_TINT(0x00);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
		val = ITNTBLEV_BURST(0xd9) | ITNTBLEV_TINT(0x00);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		val = ITNTBLEV_BURST(0xd7) | ITNTBLEV_TINT(0x00);
		break;
	default:
		/* DO NOTHING */
		break;
	}

	rcar_cvbs_write(cvbs, ITNTBLEV, val);
}

void rcar_du_cvbsenc_set_subcarrier_freq(struct rcar_du_cvbsenc *cvbs)
{
	u16 val_hi = 0;
	u16 val_lo = 0;

	switch (cvbs->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
		val_hi = IFSCH_M_NTSC;
		val_lo = IFSCL_M_NTSC;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
		val_hi = IFSCH_M_PAL;
		val_lo = IFSCL_M_PAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
		val_hi = IFSCH_BGI_PAL;
		val_lo = IFSCL_BGI_PAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
		val_hi = IFSCH_BGI_PAL;
		val_lo = IFSCL_BGI_PAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		val_hi = IFSCH_N_PAL;
		val_lo = IFSCL_N_PAL;
		break;
	default:
		/* DO NOTHING */
		break;
	}

	rcar_cvbs_write(cvbs, IFSCH, val_hi);
	rcar_cvbs_write(cvbs, IFSCL, val_lo);
}

void rcar_du_cvbsenc_set_lum_sync_level(struct rcar_du_cvbsenc *cvbs)
{
	u16 val = 0;

	val = ISYNSET_SYNDLY(0x15);

	switch (cvbs->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
		val |= ISYNSET_Y1SYNC_LEVEL(0xdb);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		val |= ISYNSET_Y1SYNC_LEVEL(0xea);
		break;
	default:
		/* DO NOTHING */
		break;
	}

	rcar_cvbs_write(cvbs, ISYNSET, val);
}

void rcar_du_cvbsenc_set_burst_insert_pos(struct rcar_du_cvbsenc *cvbs)
{
	u16 val = 0;

	switch (cvbs->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
		val = IBURST_START(0x45) | IBURST_END(0x65);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
		val = IBURST_START(0x49) | IBURST_END(0x6b);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		val = IBURST_START(0x47) | IBURST_END(0x65);
		break;
	default:
		/* DO NOTHING */
		break;
	}

	rcar_cvbs_write(cvbs, IBURST, val);
}


void rcar_du_cvbsenc_set_burst_adv_start_pos(struct rcar_du_cvbsenc *cvbs)
{
	u16 val = 0;

	val = ICSP_SCH(0x00);

	switch (cvbs->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
		val |= ICSP_START(0x3d);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
		val |= ICSP_START(0x42);
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		val |= ICSP_START(0x41);
		break;
	default:
		/* DO NOTHING */
		break;
	}

	rcar_cvbs_write(cvbs, ICSP, val);
}

int rcar_du_cvbsenc_start(struct rcar_du_cvbsenc *cvbs,
			  struct rcar_du_crtc *rcrtc)
{
	void __iomem *srstclr7_reg;
	void __iomem *padsa2r_reg;
	void __iomem *mod_sel5_reg;
	void __iomem *mod_sel5_mask_reg;
	u32 padsa2r_val;
	u32 mod_sel5_val;
	u16 val;
	int ret;

	if (cvbs->dpms == DRM_MODE_DPMS_ON)
		return 0;

	if (!(cvbs->dev->info->cvbs_crtc & (0x01 << rcrtc->index)))
		return -1;

	rcrtc->cvbs_ch = cvbs->index;

	cvbs->input = (rcrtc->index == 1) ? RCAR_CVBS_INPUT_DU1
					  : RCAR_CVBS_INPUT_DU0;

	/* software reset release */
	srstclr7_reg = ioremap_nocache(SRSTCLR7_REGISTER, 0x04);
	writel_relaxed(SRCR7_DVENC, srstclr7_reg);
	iounmap(srstclr7_reg);

	/* Set PADSA2R register */
	padsa2r_reg = ioremap_nocache(PADSA2R_REGISTER, 0x04);
	if (rcrtc->index == 1) {
		padsa2r_val = PADSA2R_DU1_DVENC |
			      (readl_relaxed(padsa2r_reg) & 0x00000300);
	} else {
		padsa2r_val = PADSA2R_DU0_DVENC |
			      (readl_relaxed(padsa2r_reg) & 0x00000C00);
	}
	writel_relaxed(padsa2r_val, padsa2r_reg);
	iounmap(padsa2r_reg);

	/* DAC power on and DU data select */
	mod_sel5_mask_reg = ioremap_nocache(MOD_SEL5_MASK_REGISTER, 0x04);
	mod_sel5_reg      = ioremap_nocache(MOD_SEL5_REGISTER, 0x04);
	mod_sel5_val = MOD_SEL5_DACPDB_NORMAL;
	mod_sel5_val |= (rcrtc->index == 1) ? MOD_SEL5_DU1_SEL_MODE_DU1
					    : MOD_SEL5_DU1_SEL_MODE_DU0;
	writel_relaxed(~mod_sel5_val, mod_sel5_mask_reg);
	writel_relaxed(mod_sel5_val, mod_sel5_reg);
	iounmap(mod_sel5_reg);
	iounmap(mod_sel5_mask_reg);

	ret = clk_prepare_enable(cvbs->clock);
	if (ret < 0)
		return ret;

	/* Use the NTSC encoder */
	rcar_cvbs_write(cvbs, VSET, VSET_VESEL);

	/* Set DAC output pin */
	rcar_cvbs_write(cvbs, DENCOUT2, DENCOUT2_HDPD0_VDAC);

	/* Initial state(not output): set to 0x3FF */
	rcar_cvbs_write(cvbs, DENCO20, DENCO20_YSEL_INIT);

	/* Set operating mode */
	rcar_du_cvbsenc_set_operating_mode(cvbs);

	/* Set TINT burst amplitude */
	rcar_du_cvbsenc_set_tint_burst_amplitude(cvbs);

	/* Set subcarrier frequency */
	rcar_du_cvbsenc_set_subcarrier_freq(cvbs);

	/* Set luminance signal synchronization level */
	rcar_du_cvbsenc_set_lum_sync_level(cvbs);

	/* Set Closed-Caption/CGMS/WSS mode */
	val = ICCCGWS_CC_ANDG_ON | ICCCGWS_VBI_BYPASS_BLNK |
	      ICCCGWS_Y1CC_OFF | ICCCGWS_CGMS_OFF;
	rcar_cvbs_write(cvbs, ICCCGWS, val);

	/* Set burst insertion position */
	rcar_du_cvbsenc_set_burst_insert_pos(cvbs);

	/* Set burst advanced start position */
	rcar_du_cvbsenc_set_burst_adv_start_pos(cvbs);

	/* Note: keep following registers initial value:
	 *   IDLYSET, ICCD1, ICCD2, ICGWSD, IBLKSHP, ISHPSET,
	 *   IBRTCON, IHADA, ITNTCOL, IY1GAIN, ICGAIN
	 */

	/* Start output to DAC */
	rcar_cvbs_write(cvbs, DENCO20, DENCO20_YSEL_OUT2DAC);

	cvbs->dpms = DRM_MODE_DPMS_ON;

	return 0;
}

int rcar_du_cvbsenc_stop_suspend(struct rcar_du_cvbsenc *cvbs)
{
	if (cvbs->dpms == DRM_MODE_DPMS_OFF)
		return -1;

	/* Stop output to DAC */
	clk_disable_unprepare(cvbs->clock);

	if (cvbs->dev->info->cvbs_crtc) {
		void __iomem *srcr7_reg;
		void __iomem *mod_sel5_reg;
		void __iomem *mod_sel5_mask_reg;
		u32 mod_sel5_val;

		/* DAC power down */
		mod_sel5_mask_reg = ioremap_nocache(MOD_SEL5_MASK_REGISTER,
						    0x04);
		mod_sel5_reg      = ioremap_nocache(MOD_SEL5_REGISTER, 0x04);
		mod_sel5_val = readl_relaxed(mod_sel5_reg);
		mod_sel5_val &= ~MOD_SEL5_DACPDB_MASK;
		mod_sel5_val |= MOD_SEL5_DACPDB_PD;
		writel_relaxed(~mod_sel5_val, mod_sel5_mask_reg);
		writel_relaxed(mod_sel5_val, mod_sel5_reg);
		iounmap(mod_sel5_reg);
		iounmap(mod_sel5_mask_reg);

		/* software reset */
		srcr7_reg = ioremap_nocache(SRCR7_REGISTER, 0x04);
		writel_relaxed(readl_relaxed(srcr7_reg) |
				SRCR7_DVENC, srcr7_reg);
		iounmap(srcr7_reg);
	}

	cvbs->dpms = DRM_MODE_DPMS_OFF;

	return 0;
}

void rcar_du_cvbsenc_stop(struct rcar_du_cvbsenc *cvbs)
{
	int ret;
	unsigned int i;

	ret = rcar_du_cvbsenc_stop_suspend(cvbs);
	if (ret < 0)
		return;

	for (i = 0; i < cvbs->dev->pdata->num_crtcs; ++i)
		if (cvbs->index == cvbs->dev->crtcs[i].cvbs_ch)
			cvbs->dev->crtcs[i].cvbs_ch = -1;
}

int rcar_du_cvbsenc_dpms(struct rcar_du_cvbsenc *cvbs,
			 struct drm_crtc *crtc, int mode)
{
	if (mode == DRM_MODE_DPMS_OFF) {
		rcar_du_cvbsenc_stop(cvbs);
		return 0;
	} else if (crtc) {
		struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
		return rcar_du_cvbsenc_start(cvbs, rcrtc);
	} else
		return -EINVAL;
}

static int rcar_du_cvbsenc_get_resources(struct rcar_du_cvbsenc *cvbs,
					 struct platform_device *pdev)
{
	struct resource *mem;
	char name[7];

	sprintf(name, "cvbs.%u", cvbs->index);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory resource for %s\n",
			name);
		return -EINVAL;
	}

	cvbs->mmio = devm_ioremap_resource(&pdev->dev, mem);
	if (cvbs->mmio == NULL) {
		dev_err(&pdev->dev, "failed to remap memory resource for %s\n",
			name);
		return -ENOMEM;
	}

	cvbs->clock = devm_clk_get(&pdev->dev, name);
	if (IS_ERR(cvbs->clock)) {
		dev_err(&pdev->dev, "failed to get clock for %s\n", name);
		return PTR_ERR(cvbs->clock);
	}

	return 0;
}

int rcar_du_cvbsenc_init(struct rcar_du_device *rcdu)
{
	struct platform_device *pdev = to_platform_device(rcdu->dev);
	const struct rcar_du_platform_data *pdata = rcdu->pdata;
	struct rcar_du_cvbsenc *cvbs;
	unsigned int j;
	int ret;

	if (rcdu->info->num_cvbs > 0) {
		cvbs = devm_kzalloc(&pdev->dev, sizeof(*cvbs), GFP_KERNEL);
		if (cvbs == NULL) {
			dev_err(&pdev->dev, "failed to allocate private data\n");
			return -ENOMEM;
		}

		cvbs->dev = rcdu;
		cvbs->index = 0;
		cvbs->input = RCAR_CVBS_INPUT_DU0;
		cvbs->dpms = DRM_MODE_DPMS_OFF;

		for (j = 0; j < pdata->num_encoders; j++) {
			if (pdata->encoders[j].output == RCAR_DU_OUTPUT_CVBS)
				cvbs->tvsys = pdata->encoders[j].connector
						.cvbs.tvsys;
		}

		ret = rcar_du_cvbsenc_get_resources(cvbs, pdev);
		if (ret < 0)
			return ret;

		rcdu->cvbs = cvbs;
	}

	return 0;
}
