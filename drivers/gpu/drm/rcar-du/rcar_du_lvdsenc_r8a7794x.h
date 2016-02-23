/*
 * rcar_du_lvdsenc_r8a7794x.h  --  R-Car Display Unit LVDS Encoder for R8A7794X
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

#ifndef __RCAR_DU_LVDSENC_R8A7794X_H__
#define __RCAR_DU_LVDSENC_R8A7794X_H__

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_data/rcar-du.h>

#include "rcar_du_lvdsenc.h"

int rcar_du_lvdsenc_r8a7794x_start(struct rcar_du_lvdsenc *lvds,
				   struct rcar_du_crtc *rcrtc);
void rcar_du_lvdsenc_r8a7794x_stop(struct rcar_du_lvdsenc *lvds);
int rcar_du_lvdsenc_r8a7794x_stop_suspend(struct rcar_du_lvdsenc *lvds);

#if IS_ENABLED(CONFIG_DRM_RCAR_LVDS)
int rcar_du_lvdsenc_r8a7794x_dpms(struct rcar_du_lvdsenc *lvds,
				  struct drm_crtc *crtc, int mode);
#else
static inline int rcar_du_lvdsenc_r8a7794x_dpms(struct rcar_du_lvdsenc *lvds,
						struct drm_crtc *crtc, int mode)
{
	return 0;
}
#endif


#endif /* __RCAR_DU_LVDSENC_R8A7794X_H__ */
