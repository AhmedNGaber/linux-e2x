/*
 * rcar_lvds_regs_r8a7794x.h  --  R-Car LVDS Interface Registers Definitions
 *                                for R8A7794X
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: TODO E2X
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#ifndef __RCAR_LVDS_REGS_R8A7794X_H__
#define __RCAR_LVDS_REGS_R8A7794X_H__

#define LCLKSELR			0x0000
#define LCLKSELR_LVDS_IN_CLK_SEL_2_1_PERI		(0 << 25)
#define LCLKSELR_LVDS_IN_CLK_SEL_2_1_DU_DOTCLKIN0	(1 << 25)
#define LCLKSELR_LVDS_IN_CLK_SEL_2_1_DU_DOTCLKIN1	(2 << 25)
#define LCLKSELR_LVDS_IN_CLK_SEL_0_DU0	(0 << 24)
#define LCLKSELR_LVDS_IN_CLK_SEL_0_DU1	(1 << 24)
#define LCLKSELR_LVDS_NIDIV_SET(x)	(((x) & 0x3) << 16)
#define LCLKSELR_LVDS_CLK_EN		(1 << 4)
#define LCLKSELR_LVDS_BIAS_PDN		(1 << 1)

#define LPLLSETR			0x0004
#define LPLLSETR_LVDSPLL_BP_NORMAL	(0 << 31)
#define LPLLSETR_LVDSPLL_BP_BYPASS	(1 << 31)
#define LPLLSETR_LVDSPLL_FD(x)		(((x) & 0x7ff) << 16)
#define LPLLSETR_LVDSPLL_RD(x)		(((x) & 0x1f) << 8)
#define LPLLSETR_LVDSPLL_OD(x)		(((x) & 0x3) << 4)
#define LPLLSETR_LVDSPLL_PD		(1 << 0)

#define LPLLMONR			0x0008
#define LPLLMONR_LVDSPLL_LD_LOCKED	(1 << 0)

#define LVDCR0				0x0010
#define LVDCR0_DUSEL			(1 << 15)
#define LVDCR0_LVMD_SHIFT		8
#define LVDCR0_LVMD_MASK		(0xf << LVDCR0_LVMD_SHIFT)
#define LVDCR0_LVMD(x)			((x << LVDCR0_LVMD_SHIFT) \
						 & LVDCR0_LVMD_MASK)
#define LVDCR0_LVRES			(1 << 0)

#define LVDCTRCR			0x0014
#define LVDCTRCR_CTR3SEL_ZERO		(0 << 12)
#define LVDCTRCR_CTR3SEL_ODD		(1 << 12)
#define LVDCTRCR_CTR3SEL_CDE		(2 << 12)
#define LVDCTRCR_CTR3SEL_MASK		(7 << 12)
#define LVDCTRCR_CTR2SEL_DISP		(0 << 8)
#define LVDCTRCR_CTR2SEL_ODD		(1 << 8)
#define LVDCTRCR_CTR2SEL_CDE		(2 << 8)
#define LVDCTRCR_CTR2SEL_HSYNC		(3 << 8)
#define LVDCTRCR_CTR2SEL_VSYNC		(4 << 8)
#define LVDCTRCR_CTR2SEL_MASK		(7 << 8)
#define LVDCTRCR_CTR1SEL_VSYNC		(0 << 4)
#define LVDCTRCR_CTR1SEL_DISP		(1 << 4)
#define LVDCTRCR_CTR1SEL_ODD		(2 << 4)
#define LVDCTRCR_CTR1SEL_CDE		(3 << 4)
#define LVDCTRCR_CTR1SEL_HSYNC		(4 << 4)
#define LVDCTRCR_CTR1SEL_MASK		(7 << 4)
#define LVDCTRCR_CTR0SEL_HSYNC		(0 << 0)
#define LVDCTRCR_CTR0SEL_VSYNC		(1 << 0)
#define LVDCTRCR_CTR0SEL_DISP		(2 << 0)
#define LVDCTRCR_CTR0SEL_ODD		(3 << 0)
#define LVDCTRCR_CTR0SEL_CDE		(4 << 0)
#define LVDCTRCR_CTR0SEL_MASK		(7 << 0)

#define LVDCHCR				0x0018
#define LVDCHCR_CHSEL_CH(n, c)		((((c) - (n)) & 3) << ((n) * 4))
#define LVDCHCR_CHSEL_MASK(n)		(3 << ((n) * 4))

/* -----------------------------------------------------------------------------
 * Reset register
 */
#define SRCR7_REGISTER		0xE61501CC
#define SRSTCLR7_REGISTER	0xE615095C
#define SRCR7_LVDS0		(1 << 26)
#define SRCR7_LVDS1		(1 << 25)


#define DIDSR_REGISTER	0xFEB20028
#define DIDSR_DU0_LVDS	0x77900200
#define DIDSR_DU1_LVDS	0x77900800

#endif /* __RCAR_LVDS_REGS_R8A7794X_H__ */
