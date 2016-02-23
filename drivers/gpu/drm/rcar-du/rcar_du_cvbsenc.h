/*
 * rcar_du_cvbsenc.h  --  R-Car Display Unit CVBS Encoder
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

#ifndef __RCAR_DU_CVBSENC_H__
#define __RCAR_DU_CVBSENC_H__

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_data/rcar-du.h>

#ifdef R8A779X_ES2_DU_CVBS_CH_DATA_GAP_WORKAROUND
#define WAIT_PS_TIME_UNDER_61MHZ	100000
#define WAIT_PS_TIME_UPPER_61MHZ	50000
#define WAIT_PS_TIME_UPPER_121MHZ	25000
#endif

struct rcar_drm_crtc;
struct rcar_du_cvbsenc;

enum rcar_cvbs_input {
	RCAR_CVBS_INPUT_DU0,
	RCAR_CVBS_INPUT_DU1,
};

enum rcar_cvbs_output_tvsys {
	RCAR_CVBS_OUTPUT_TVSYS_NTSC = 0,
	RCAR_CVBS_OUTPUT_TVSYS_PAL,
	RCAR_CVBS_OUTPUT_TVSYS_PAL_M,
	RCAR_CVBS_OUTPUT_TVSYS_PAL_N,
	RCAR_CVBS_OUTPUT_TVSYS_PAL_60,
	RCAR_CVBS_OUTPUT_TVSYS_NUM
};

void rcar_du_cvbsenc_set_tvsys(struct rcar_du_cvbsenc *cvbs,
			       enum rcar_cvbs_output_tvsys tvsys);
enum rcar_cvbs_output_tvsys
rcar_du_cvbsenc_get_tvsys(struct rcar_du_cvbsenc *cvbs);

int rcar_du_cvbsenc_start(struct rcar_du_cvbsenc *cvbs,
			  struct rcar_du_crtc *rcrtc);
void rcar_du_cvbsenc_stop(struct rcar_du_cvbsenc *cvbs);
int rcar_du_cvbsenc_stop_suspend(struct rcar_du_cvbsenc *cvbs);

int rcar_du_cvbsenc_init(struct rcar_du_device *rcdu);
int rcar_du_cvbsenc_dpms(struct rcar_du_cvbsenc *cvbs,
			 struct drm_crtc *crtc, int mode);

#endif /* __RCAR_DU_CVBSENC_H__ */
