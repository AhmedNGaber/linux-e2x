/*
 * rcar_du.h  --  R-Car Display Unit DRM driver
 *
 * Copyright (C) 2013-2014 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RCAR_DU_H__
#define __RCAR_DU_H__

#include <drm/drm_mode.h>

enum rcar_du_output {
	RCAR_DU_OUTPUT_DPAD0,
	RCAR_DU_OUTPUT_DPAD1,
	RCAR_DU_OUTPUT_LVDS0,
	RCAR_DU_OUTPUT_LVDS1,
	RCAR_DU_OUTPUT_TCON,
	RCAR_DU_OUTPUT_HDMI,
	RCAR_DU_OUTPUT_MAX,
};

enum rcar_du_encoder_type {
	RCAR_DU_ENCODER_UNUSED = 0,
	RCAR_DU_ENCODER_NONE,
	RCAR_DU_ENCODER_VGA,
	RCAR_DU_ENCODER_LVDS,
	RCAR_DU_ENCODER_HDMI,
};

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
enum rcar_du_connect_vsp {
	RCAR_DU_VSPD_UNUSED = -1,
	RCAR_DU_VSPD_0,
	RCAR_DU_VSPD_1,
	RCAR_DU_VSPD_0_RGB,
	RCAR_DU_VSPD_1_RGB,
	RCAR_DU_VSPD_MAX,
};
#endif

struct rcar_du_panel_data {
	unsigned int width_mm;		/* Panel width in mm */
	unsigned int height_mm;		/* Panel height in mm */
	unsigned int lvds_mode;
	struct drm_mode_modeinfo mode;
};

struct rcar_du_connector_lvds_data {
	struct rcar_du_panel_data panel;
};

struct rcar_du_connector_vga_data {
	/* TODO: Add DDC information for EDID retrieval */
};

struct rcar_du_connector_hdmi_data {
	/* TODO: Add DDC information for EDID retrieval */
};

/*
 * struct rcar_du_encoder_data - Encoder platform data
 * @type: the encoder type (RCAR_DU_ENCODER_*)
 * @output: the DU output the connector is connected to (RCAR_DU_OUTPUT_*)
 * @connector.lvds: platform data for LVDS connectors
 * @connector.vga: platform data for VGA connectors
 *
 * Encoder platform data describes an on-board encoder, its associated DU SoC
 * output, and the connector.
 */
struct rcar_du_encoder_data {
	enum rcar_du_encoder_type type;
	enum rcar_du_output output;

	union {
		struct rcar_du_connector_lvds_data lvds;
		struct rcar_du_connector_vga_data vga;
		struct rcar_du_connector_hdmi_data hdmi;
	} connector;
};

struct rcar_du_crtc_data {
	unsigned int exclk;
	unsigned int init_conn_type;
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	enum rcar_du_connect_vsp vsp;
#endif
};

struct rcar_du_platform_data {
	struct rcar_du_encoder_data *encoders;
	unsigned int num_encoders;
	struct rcar_du_crtc_data *crtcs;
	unsigned int num_crtcs;
#ifdef CONFIG_DRM_FBDEV_CRTC
	unsigned int fbdev_crtc;
#endif
	int (*backlight_on)(void);
	int (*backlight_off)(void);
	unsigned int i2c_ch;
};

#endif /* __RCAR_DU_H__ */
