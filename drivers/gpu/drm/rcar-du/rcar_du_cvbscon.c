/*
 * rcar_du_cvbscon.c  --  R-Car Display Unit CVBS Connector
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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#include "rcar_du_drv.h"
#include "rcar_du_encoder.h"
#include "rcar_du_kms.h"
#include "rcar_du_cvbscon.h"
#include "rcar_du_cvbsenc.h"

struct rcar_du_cvbs_connector {
	struct rcar_du_connector connector;

	const struct rcar_du_connector_cvbs_data *cvbs;

	struct drm_property *prop_tvsys;
	u32 tvsys;
};

#define to_rcar_cvbs_connector(c) \
	container_of(c, struct rcar_du_cvbs_connector, connector.connector)

static int rcar_du_cvbs_connector_get_modes(struct drm_connector *connector)
{
	struct rcar_du_cvbs_connector *cvbscon =
		to_rcar_cvbs_connector(connector);
	struct drm_display_mode *mode;
	u32 mode_num =
		sizeof(cvbscon->cvbs->modes)/sizeof(cvbscon->cvbs->modes[0]);
	u32 i;
	u32 count = 0;

	for (i = 0; i < mode_num; i++) {
		mode = drm_mode_create(connector->dev);
		if (mode == NULL)
			return count;

		mode->type = DRM_MODE_TYPE_PREFERRED | DRM_MODE_TYPE_DRIVER;
		mode->clock = cvbscon->cvbs->modes[i].clock;
		mode->hdisplay = cvbscon->cvbs->modes[i].hdisplay;
		mode->hsync_start = cvbscon->cvbs->modes[i].hsync_start;
		mode->hsync_end = cvbscon->cvbs->modes[i].hsync_end;
		mode->htotal = cvbscon->cvbs->modes[i].htotal;
		mode->vdisplay = cvbscon->cvbs->modes[i].vdisplay;
		mode->vsync_start = cvbscon->cvbs->modes[i].vsync_start;
		mode->vsync_end = cvbscon->cvbs->modes[i].vsync_end;
		mode->vtotal = cvbscon->cvbs->modes[i].vtotal;
		mode->flags = cvbscon->cvbs->modes[i].flags;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

static int rcar_du_cvbs_connector_mode_valid(struct drm_connector *connector,
					    struct drm_display_mode *mode)
{
	struct rcar_du_cvbs_connector *cvbscon =
			to_rcar_cvbs_connector(connector);

	if (mode->hdisplay != 720)
		return MODE_ONE_WIDTH;

	switch (cvbscon->tvsys) {
	case RCAR_CVBS_OUTPUT_TVSYS_NTSC:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_M:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_60:
		if (mode->vdisplay != 480)
			return MODE_V_ILLEGAL;
		break;
	case RCAR_CVBS_OUTPUT_TVSYS_PAL:
	case RCAR_CVBS_OUTPUT_TVSYS_PAL_N:
		if (mode->vdisplay != 576)
			return MODE_V_ILLEGAL;
		break;
	default:
		return MODE_ERROR;
	}

	return MODE_OK;
}

static const struct drm_connector_helper_funcs connector_helper_funcs = {
	.get_modes = rcar_du_cvbs_connector_get_modes,
	.mode_valid = rcar_du_cvbs_connector_mode_valid,
	.best_encoder = rcar_du_connector_best_encoder,
};

static void rcar_du_cvbs_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
rcar_du_cvbs_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int rcar_du_cvbs_connector_set_tvsys(
			struct rcar_du_cvbs_connector *cvbscon, u32 tvsys)
{
	if (cvbscon->tvsys == tvsys)
		return 0;

	if (tvsys < 0 || tvsys >= RCAR_CVBS_OUTPUT_TVSYS_NUM)
		return -EINVAL;

	rcar_du_cvbsenc_set_tvsys(cvbscon->connector.encoder->cvbs, tvsys);

	cvbscon->tvsys = tvsys;

	return 0;
}

static int rcar_du_cvbs_connector_set_property(struct drm_connector *connector,
		struct drm_property *property, uint64_t val)
{
	struct rcar_du_cvbs_connector *cvbscon =
		to_rcar_cvbs_connector(connector);

	if (property == cvbscon->prop_tvsys)
		rcar_du_cvbs_connector_set_tvsys(cvbscon, (u32)val);
	else
		return -EINVAL;

	return 0;
}


static const struct drm_connector_funcs connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rcar_du_cvbs_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = rcar_du_cvbs_connector_set_property,
	.destroy = rcar_du_cvbs_connector_destroy,
};

int rcar_du_cvbs_connector_init(struct rcar_du_device *rcdu,
				struct rcar_du_encoder *renc,
				const struct rcar_du_connector_cvbs_data *cvbs)
{
	struct rcar_du_cvbs_connector *cvbscon;
	struct drm_connector *connector;
	int ret;

	cvbscon = devm_kzalloc(rcdu->dev, sizeof(*cvbscon), GFP_KERNEL);
	if (cvbscon == NULL)
		return -ENOMEM;

	cvbscon->cvbs = cvbs;

	cvbscon->prop_tvsys = drm_property_create_range(
					rcdu->ddev, 0, "tvsys",
					0, RCAR_CVBS_OUTPUT_TVSYS_NUM - 1);
	if (cvbscon->prop_tvsys == NULL)
		return -ENOMEM;

	connector = &cvbscon->connector.connector;

	connector->interlace_allowed = true;

	ret = drm_connector_init(rcdu->ddev, connector, &connector_funcs,
				 DRM_MODE_CONNECTOR_Composite);
	if (ret < 0)
		return ret;

	drm_connector_helper_add(connector, &connector_helper_funcs);
	ret = drm_sysfs_connector_add(connector);
	if (ret < 0)
		return ret;

	drm_helper_connector_dpms(connector, DRM_MODE_DPMS_OFF);
	drm_object_property_set_value(&connector->base,
		rcdu->ddev->mode_config.dpms_property, DRM_MODE_DPMS_OFF);

	ret = drm_mode_connector_attach_encoder(connector, renc->encoder);
	if (ret < 0)
		return ret;

	connector->encoder = renc->encoder;
	cvbscon->connector.encoder = renc;

	cvbscon->tvsys =
		rcar_du_cvbsenc_get_tvsys(cvbscon->connector.encoder->cvbs);
	drm_object_attach_property(&connector->base,
			cvbscon->prop_tvsys, cvbscon->tvsys);

	return 0;
}
