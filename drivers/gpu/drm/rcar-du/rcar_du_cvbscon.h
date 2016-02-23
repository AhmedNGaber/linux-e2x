/*
 * rcar_du_cvbscon.h  --  R-Car Display Unit CVBS Connector
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

#ifndef __RCAR_DU_CVBSCON_H__
#define __RCAR_DU_CVBSCON_H__

struct rcar_du_device;
struct rcar_du_encoder;

int rcar_du_cvbs_connector_init(struct rcar_du_device *rcdu,
				struct rcar_du_encoder *renc,
				const struct rcar_du_connector_cvbs_data *cvbs);

#endif /* __RCAR_DU_CVBSCON_H__ */
