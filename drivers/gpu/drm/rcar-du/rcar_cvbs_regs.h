/*
 * rcar_cvbs_regs.h  --  R-Car CVBS Interface Registers Definitions
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: TODO E2X: contact
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#ifndef __RCAR_CVBS_REGS_H__
#define __RCAR_CVBS_REGS_H__

#define VSET				0x0120
#define VSET_VESEL			(1 << 14)

#define DENCOUT				0x0300

#define DENCOUT2			0x0302
#define DENCOUT2_HDPD0(x)		(((x) & 0xf) << 0)
#define DENCOUT2_HDPD0_VDAC		DENCOUT2_HDPD0(0x3)

#define DENCO20				0x0308
#define DENCO20_YSEL_INIT		(0x3ff << 0)
#define DENCO20_YSEL_OUT2DAC		(0x000 << 0)

#define IDENCMD1			0x00C0
#define IDENCMD1_SRL_EN			(1 << 12)
#define IDENCMD1_FLD2CC_TRANS_UPDATE	(0 << 9)
#define IDENCMD1_FLD2CC_TRANS_COMP	(1 << 9)
#define IDENCMD1_FLD1CC_TRANS_UPDATE	(0 << 8)
#define IDENCMD1_FLD1CC_TRANS_COMP	(1 << 8)
#define IDENCMD1_WSS_OFF		(0 << 6)
#define IDENCMD1_WSS_ON			(1 << 6)
#define IDENCMD1_SUBC_FREQ_3_58		(0 << 5)
#define IDENCMD1_SUBC_FREQ_4_43		(1 << 5)
#define IDENCMD1_SUBC_PHCTRL_ON		(0 << 4)
#define IDENCMD1_SUBC_PHCTRL_OFF	(1 << 4)
#define IDENCMD1_FLD_FREQ_525_60	(0 << 1)
#define IDENCMD1_FLD_FREQ_625_50	(1 << 1)
#define IDENCMD1_CDM_NTSC		(0 << 0)
#define IDENCMD1_CDM_PAL		(1 << 0)

#define IDENCMD2			0x00C2
#define IDENCMD2_ADJ_COL_UPLIM_ON	(1 << 15)
#define IDENCMD2_ADJ_COL_LOLIM_ON	(1 << 14)
#define IDENCMD2_ADJ_LUM_UPLIM_ON	(1 << 13)
#define IDENCMD2_ADJ_LUM_LOLIM_ON	(1 << 12)
#define IDENCMD2_COL_UPLIM_ON		(1 << 11)
#define IDENCMD2_COL_LOLIM_ON		(1 << 10)
#define IDENCMD2_LUM_UPLIM_ON		(1 << 9)
#define IDENCMD2_LUM_LOLIM_ON		(1 << 8)
#define IDENCMD2_Y1SETUP_OFF		(0 << 0)
#define IDENCMD2_Y1SETUP_POS_7_5IRE	(1 << 0)
#define IDENCMD2_Y1SETUP_NEG_7_5IRE	(2 << 0)

#define ITNTBLEV			0x00C4
#define ITNTBLEV_BURST(x)		(((x) & 0xff) << 8)
#define ITNTBLEV_TINT(x)		(((x) & 0xff) << 0)

#define IFSCH				0x00C6
#define IFSCH_M_NTSC			0x21F0
#define IFSCH_BGI_PAL			0x2A09
#define IFSCH_M_PAL			0x21E6
#define IFSCH_N_PAL			0x21F6

#define IFSCL				0x00C8
#define IFSCL_M_NTSC			0x7C1F
#define IFSCL_BGI_PAL			0x8ACB
#define IFSCL_M_PAL			0xEFE3
#define IFSCL_N_PAL			0x9447

#define IDLYSET				0x00CA
#define IDLYSET_Y1DLY(x)		(((x) & 0x3f) << 8)


#define ISYNSET				0x00CC
#define ISYNSET_SYNDLY(x)		(((x) & 0x1f) << 8)
#define ISYNSET_Y1SYNC_LEVEL(x)		(((x) & 0xff) << 0)

#define ICCCGWS				0x00CE
#define ICCCGWS_CC_ANDG_ON		(0 << 6)
#define ICCCGWS_CC_ANDG_OFF		(1 << 6)
#define ICCCGWS_VBI_BYPASS_BYPS		(0 << 5)
#define ICCCGWS_VBI_BYPASS_BLNK		(1 << 5)
#define ICCCGWS_Y1CC_OFF		(0 << 1)
#define ICCCGWS_Y1CC_FLD1		(1 << 1)
#define ICCCGWS_Y1CC_FLD2		(2 << 1)
#define ICCCGWS_Y1CC_BOTH		(3 << 1)
#define ICCCGWS_CGMS_OFF		(0 << 0)
#define ICCCGWS_CGMS_ON			(1 << 0)

#define ICCD1				0x00D0
#define ICCD1_CC11(x)			(((x) & 0x7f) << 8)
#define ICCD1_CC10(x)			(((x) & 0x7f) << 0)

#define ICCD2				0x00D2
#define ICCD2_CC21(x)			(((x) & 0x7f) << 8)
#define ICCD2_CC20(x)			(((x) & 0x7f) << 0)

#define ICGWSD				0x00D4
#define ICGWSD_CGWS(x)			(((x) & 0x3fff) << 0)

#define IBLKSHP				0x00D6
#define IBLKSHP_UNSHARP_ON		(1 << 15)
#define IBLKSHP_SHLMT(x)		(((x) & 0x7f) << 8)
#define IBLKSHP_GAMMA_MODE(x)		(((x) & 0x1f) << 3)
#define IBLKSHP_BKPNT_OFF		(0 << 0)
#define IBLKSHP_BKPNT_MODE1		(1 << 0)
#define IBLKSHP_BKPNT_MODE2		(2 << 0)
#define IBLKSHP_BKPNT_MODE3		(3 << 0)

#define ISHRPSET			0x00D8
#define ISHRPSET_PSW_0T			(0 << 14)
#define ISHRPSET_PSW_1T			(1 << 14)
#define ISHRPSET_PSW_2T			(2 << 14)
#define ISHRPSET_SHRPNSB(x)		(((x) & 0x3f) << 8)
#define ISHRPSET_OSW_0T			(0 << 6)
#define ISHRPSET_OSW_1T			(1 << 6)
#define ISHRPSET_OSW_2T			(2 << 6)
#define ISHRPSET_SHRPNSA(x)		(((x) & 0x3f) << 0)

#define IBRTCON				0x00DA
#define IBRTCON_BRIGHT(x)		(((x) & 0xff) << 8)
#define IBRTCON_CONT(x)			(((x) & 0xff) << 0)

#define IHADA				0x00DC
#define IHADA_WIND(x)			(((x) & 0x7f) << 8)
#define IHADA_STC_AREA_B_ON		(1 << 1)
#define IHADA_STC_AREA_A_ON		(1 << 0)

#define ITNTCOL				0x00DE
#define ITNTCOL_TINT(x)			(((x) & 0x7f) << 8)
#define ITNTCOL_COLOR(x)		(((x) & 0x7f) << 0)

#define ICGAIN				0x00F4
#define ICGAIN_CGAIN(x)			(((x) & 0xfff) << 0)

#define IBURST				0x00F8
#define IBURST_START(x)			(((x) & 0xff) << 8)
#define IBURST_END(x)			(((x) & 0xff) << 0)

#define ICSP				0x00FA
#define ICSP_SCH(x)			(((x) & 0xff) << 8)
#define ICSP_START(x)			(((x) & 0xff) << 0)

#define IY1GAIN		0x0108
#define IY1GAIN_Y1GAIN(x)		(((x) & 0x3ff) << 0)

/* -----------------------------------------------------------------------------
 * MOD_SEL5 register
 */
#define MOD_SEL5_REGISTER		0xE60600D0
#define MOD_SEL5_DACPDB_PD		(0 << 4)
#define MOD_SEL5_DACPDB_NORMAL		(1 << 4)
#define MOD_SEL5_DACPDB_MASK		(1 << 4)
#define MOD_SEL5_DU1_SEL_MODE_DU0	(0 << 0)
#define MOD_SEL5_DU1_SEL_MODE_DU1	(1 << 0)

#define MOD_SEL5_MASK_REGISTER		0xE6060000

/* -----------------------------------------------------------------------------
 * Reset register
 */
#define SRCR7_REGISTER			0xE61501CC
#define SRSTCLR7_REGISTER		0xE615095C
#define SRCR7_DVENC			(1 << 27)


#define DIDSR_REGISTER		0xFEB20028
#define DIDSR_DU0_DVENC		0x77900300
#define DIDSR_DU1_DVENC		0x77900C00

#endif /* __RCAR_CVBS_REGS_H__ */
