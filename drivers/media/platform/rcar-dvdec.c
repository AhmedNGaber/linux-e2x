/*
 * drivers/media/platform/rcar-dvdec.c
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * rcar-dvdec.c R-Car E2X Digital video decoder driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf-dma-contig.h>

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "rcar-dvdec.h"

const struct reg_color rcar_dvdec_reg_color[8] = {

	/* TGCR1,  TGCR2,   TGCR3,   HAFCCR1, HAFCCR2, HAFCCR3, */
	/* VCDWCR1,BTLCR,   ACCCR1,  AGCCR1,  YCSCR3,  YCSCR4,  */
	/* YCSCR5,  YCSCR6,  YCSCR7,  YCSCR9,  YCSCR12          */

	/* 0 : DVDEC_NTSC358 */
	{ 0x0100,  0x40F1,  0x0594,  0xC2B4,  0x0B18,  0x8A50,
	  0x53CF,  0x741F,  0x50DC,  0x08E6,  0x2204,  0x3408,
	  0x2006,  0x0806,  0x6325,  0x8660,  0x0205,  },

	/* 1 : DVDEC_NTSC443 */
	{ 0x0100,  0x4D20,  0x0584,  0xC2C0,  0x0B11,  0x8A76,
	  0x2BCF,  0x742F,  0x50DC,  0x08E6,  0x2204,  0x3408,
	  0x2006,  0x0806,  0x6325,  0x8660,  0x0205,  },

	/* 2 : DVDEC_PAL443 */
	{ 0x0100,  0x4D20,  0x0584,  0xC2C0,  0x0B11,  0x8A76,
	  0x2BCF,  0x747D,  0x50DC,  0x08F2,  0x2203,  0x4FC2,
	  0x200A,  0x0F0A,  0x3328,  0x0000,  0x0001,  },

	/* 3 : DVDEC_PALM */
	{ 0x0100,  0x40F1,  0x0594,  0xC2B4,  0x0B18,  0x8A50,
	  0x53CF,  0x7477,  0x50E6,  0x08F2,  0x2203,  0x4FC2,
	  0x200A,  0x0F0A,  0x3328,  0x0000,  0x0001,  },

	/* 4 : DVDEC_PALN */
	{ 0x0100,  0x4D20,  0x0584,  0xC2C0,  0x0B11,  0x8A76,
	  0x2BCF,  0x747B,  0x50E6,  0x08F2,  0x2203,  0x4FC2,
	  0x200A,  0x0F0A,  0x3328,  0x0000,  0x0001,  },

	/* 5 : DVDEC_SECAM */
	{ 0x0100,  0x4D20,  0x0584,  0xC2C0,  0x0B11,  0x8A76,
	  0x2BCF,  0x74BE,  0x50DC,  0x08F2,  0x2204,  0x3401,
	  0x2006,  0x0F06,  0x3328,  0x8660,  0x0005,  },

	/* 6 : NTSC-443 60 */
	{ 0x0100,  0x40F1,  0x0594,  0xC2B4,  0x0B18,  0x8A50,
	  0x53CF,  0x742F,  0x50DC,  0x08E6,  0x2204,  0x3408,
	  0x2006,  0x0806,  0x6325,  0x8660,  0x0205,  },

	/* 7 : PAL 60 */
	{ 0x0100,  0x40F1,  0x0594,  0xC2B4,  0x0B18,  0x8A50,
	  0x53CF,  0x747D,  0x50E6,  0x08F2,  0x2203,  0x4FC2,
	  0x200A,  0x0F0A,  0x3238,  0x0000,  0x0001,  },
};

const struct dvdec_rect rcar_dvdec_rect[8] = {
	{ 34, 480, 0, 1428},	/* 0 : DVDEC_NTSC358 */
	{ 34, 480, 0, 1428},	/* 1 : DVDEC_NTSC443 */
	{ 38, 576, 0, 1412},	/* 2 : DVDEC_PAL443 */
	{ 34, 480, 0, 1428},	/* 3 : DVDEC_PALM */
	{ 38, 576, 0, 1412},	/* 4 : DVDEC_PALN */
	{ 38, 576, 0, 1412},	/* 5 : DVDEC_SECAM */
	{ 34, 480, 0, 1428},	/* 6 : NTSC-443 60 */
	{ 34, 480, 0, 1428},	/* 7 : PAL 60 */
};

struct rcar_dvdec_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct mutex		mutex; /* mutual excl. when accessing chip */
	v4l2_std_id		curr_norm;
	bool			autodetect;
	u8			input;
	u8			output;
	u32			in_cfmt;
	const struct rcar_dvdec_color_format	*cfmt;
	void __iomem *base;
	u16			vsyncsr;
};

#define to_rcar_dvdec_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct rcar_dvdec_state,	\
					    ctrl_hdl)->sd)

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

struct rcar_dvdec_color_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

/*
 * supported color format list
 */
static const struct rcar_dvdec_color_format rcar_dvdec_cfmts[] = {
	{
		.code		= V4L2_MBUS_FMT_YUYV10_1X20,
		.colorspace	= V4L2_COLORSPACE_JPEG,
	},
};

static void rcar_dvdec_initcolor(struct v4l2_subdev *sd, u32 dvdec_fmt);

static inline struct rcar_dvdec_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rcar_dvdec_state, sd);
}

/*
 * rcar_dvdec_s_input() - Select input pin
 * @sd: pointer to standard V4L2 sub-device structure
 * @input: input pin(0:VIN1A, 1:VIN2A)
 *
 * Select and set input pin
 */
static int rcar_dvdec_s_input(struct v4l2_subdev *sd, u32 input)
{
	struct platform_device *pdev = v4l2_get_subdevdata(sd);
	struct rcar_dvdec_state *state = to_state(sd);
	u16 adccr2;

	if ((input & DVDEC_ADCCR2_ADC_VINSEL_MASK) != input) {
		dev_err(&pdev->dev, "Invalid input\n");
		return -EINVAL;
	}

	adccr2 = ioread16(state->base + DVDEC_ADCCR2_REG);
	adccr2 &= ~DVDEC_ADCCR2_ADC_VINSEL_MASK;
	adccr2 |= input;
	iowrite16(adccr2, state->base + DVDEC_ADCCR2_REG);

	state->input = input;
	return 0;
}

/*
 * rcar_dvdec_s_output() - Select output direction
 * @sd: pointer to standard V4L2 sub-device structure
 * @output: output id(0:VIN0, 1:VIN1)
 *
 * Select and set output direction.
 */
static int rcar_dvdec_s_output(struct v4l2_subdev *sd, u32 output)
{
	struct rcar_dvdec_state *state = to_state(sd);
	void __iomem *pmmr_reg;
	void __iomem *adccr4_reg;

	pmmr_reg = ioremap_nocache(DVDEC_PMMR_REG, 0x04);
	adccr4_reg = ioremap_nocache(DVDEC_ADCCR4_REG, 0x04);

	if (output == 0) {
		writel_relaxed(~0x0001, pmmr_reg);
		/* VIN0_SEL=1  (use VIN0-Module)*/
		writel_relaxed(0x0001, adccr4_reg);
	} else {
		writel_relaxed(~0x0002, pmmr_reg);
		/* VIN1_SEL=1  (use VIN1-Module)*/
		writel_relaxed(0x0002, adccr4_reg);
	}

	iounmap(adccr4_reg);
	iounmap(pmmr_reg);

	state->output = output;
	return 0;
}

/*
 * rcar_dvdec_to_v4l2_std() - Decide V4L2 video standard
 * @sd: ptr to v4l2_subdev struct
 * @cromasr1: Chroma Decoding Read Register 1 value
 * @vsyncsr: Sync Separation Status/Vertical Cycle Read Registar value
 *
 * Decide V4l2 video standard from the digital video decoder register
 */
static v4l2_std_id rcar_dvdec_to_v4l2_std(struct v4l2_subdev *sd,
	u16 cromasr1, u16 vsyncsr)
{
	struct platform_device *pdev = v4l2_get_subdevdata(sd);
	struct rcar_dvdec_state *state = to_state(sd);
	v4l2_std_id std;

	/* SECAM or UNDETECTABLE */
	std = V4L2_STD_UNKNOWN;

	state->vsyncsr = (vsyncsr & DVDEC_VSYNCSR_FVMODE_MASK);

	switch (cromasr1 & DVDEC_CROMASR1_COLORSYS_MASK) {
	case DVDEC_CROMASR1_COLORSYS_NTSC:
		switch (cromasr1 & DVDEC_CROMASR1_FSCMODE_MASK) {
		case DVDEC_CROMASR1_FSCMODE_358:
			/* NTSC-M */
			dev_info(&pdev->dev, "Detected NTSC 358\n");
			std = V4L2_STD_NTSC;
			break;

		case DVDEC_CROMASR1_FSCMODE_443:
			/* NTSC-4.43 */
			std = V4L2_STD_NTSC_443;
			switch	(vsyncsr & DVDEC_VSYNCSR_FVMODE_MASK) {
			case DVDEC_VSYNCSR_FVMODE_50:
				dev_info(&pdev->dev, "Detected PAL443 50Hz\n");
				break;

			case DVDEC_VSYNCSR_FVMODE_60:
				dev_info(&pdev->dev, "Detected PAL443 60Hz\n");
				break;
			}
			break;
		}
		break;

	case DVDEC_CROMASR1_COLORSYS_PAL:
		switch (cromasr1 & DVDEC_CROMASR1_FSCMODE_MASK) {
		case DVDEC_CROMASR1_FSCMODE_358:
			switch	(vsyncsr & DVDEC_VSYNCSR_FVMODE_MASK) {
			case DVDEC_VSYNCSR_FVMODE_50:
				/* PAL-N */
				dev_info(&pdev->dev, "Detected PALN\n");
				std = V4L2_STD_PAL_N;
				break;

			case DVDEC_VSYNCSR_FVMODE_60:
				/* PAL-M */
				dev_info(&pdev->dev, "Detected PALM\n");
				std = V4L2_STD_PAL_M;
				break;
			}
			break;

		case DVDEC_CROMASR1_FSCMODE_443:
			switch (vsyncsr & DVDEC_VSYNCSR_FVMODE_MASK) {
			case DVDEC_VSYNCSR_FVMODE_50:
				/* PAL-4.43 */
				dev_info(&pdev->dev, "Detected PAL443\n");
				std = V4L2_STD_PAL;
				break;

			case DVDEC_VSYNCSR_FVMODE_60:
				/* PAL-60 */
				dev_info(&pdev->dev, "Detected PAL60\n");
				std = V4L2_STD_PAL_60;
				break;
			}
		}
	}

	return std;
}

/*
 * v4l2_std_to_rcar_dvdec() - Decide control value
 * @v4l2_std_id: V4l2 video standard id
 *
 * Convert V4l2 video standard id to the control value of
 * the digital video decoder.
 */
static u16 v4l2_std_to_rcar_dvdec(v4l2_std_id std)
{
	if (std == V4L2_STD_NTSC)
		return DVDEC_BTLCR_DETECT_NTSC_M;
	if (std == V4L2_STD_NTSC_443)
		return DVDEC_BTLCR_DETECT_NTSC_443;
	if (std == V4L2_STD_PAL)
		return DVDEC_BTLCR_DETECT_PAL_443;
	if (std == V4L2_STD_PAL_M)
		return DVDEC_BTLCR_DETECT_PAL_M;
	if (std == V4L2_STD_PAL_N)
		return DVDEC_BTLCR_DETECT_PAL_N;
	if (std & V4L2_STD_PAL_60)
		return DVDEC_BTLCR_DETECT_PAL_60;

	return DVDEC_BTLCR_DETECT_AUTO;
}

/*
 * v4l2_std_to_rcar_dvdec_id() - Decide control value
 * @v4l2_std_id: V4l2 video standard id
 *
 * Convert V4l2 video standard id to private id
 */
static u32 v4l2_std_to_rcar_dvdec_id(
	struct rcar_dvdec_state *state, v4l2_std_id std)
{
	if (std == V4L2_STD_NTSC)
		return DVDEC_NTSC358;
	if (std == V4L2_STD_NTSC_443) {
		if (state->vsyncsr == DVDEC_VSYNCSR_FVMODE_50)
			return DVDEC_NTSC443;
		else
			return DVDEC_NTSC60;
	}
	if (std == V4L2_STD_PAL)
		return DVDEC_PAL443;
	if (std == V4L2_STD_PAL_M)
		return DVDEC_PALM;
	if (std == V4L2_STD_PAL_N)
		return DVDEC_PALN;
	if (std & V4L2_STD_PAL_60)
		return DVDEC_PAL60;

	return DVDEC_NTSC358;
}

/*
 * __rcar_dvdec_status() - get digital video decoder status
 * @sd: ptr to v4l2_subdev struct
 * @status: input status
 * @std: V4l2 video standard
 *
 * Get status and/or V4l2 video standard
 */
static int __rcar_dvdec_status(
	struct v4l2_subdev *sd, u32 *status, v4l2_std_id *std)
{
	struct rcar_dvdec_state *state = to_state(sd);
	u16 val, cromasr1, vsyncsr;

	/* Set auto detection mode */
	val = ioread16(state->base + DVDEC_VCDWCR1_REG);
	val &= ~DVDEC_VCDWCR1_AMASK;
	val |= DVDEC_VCDWCR1_AUTO;
	iowrite16(val, state->base + DVDEC_VCDWCR1_REG);

	cromasr1 = ioread16(state->base + DVDEC_CROMASR1_REG);
	vsyncsr  = ioread16(state->base + DVDEC_VSYNCSR_REG);
	if (status) {
		if (vsyncsr & DVDEC_VSYNCSR_NOSIGNAL)
			*status = V4L2_IN_ST_NO_SIGNAL;
		else
			*status = 0;
	}
	if (std) {
		*std = rcar_dvdec_to_v4l2_std(sd, cromasr1, vsyncsr);
		state->in_cfmt = v4l2_std_to_rcar_dvdec_id(state, *std);
		rcar_dvdec_initcolor(sd, state->in_cfmt);
	}

	return 0;
}

/*
 * rcar_dvdec_querystd() - V4L2 decoder i/f handler for querystd
 * @sd: ptr to v4l2_subdev struct
 * @std: standard input video id
 *
 * Obtains the video standard input id
 */
static int rcar_dvdec_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct rcar_dvdec_state *state = to_state(sd);
	int err = mutex_lock_interruptible(&state->mutex);

	if (err)
		return err;

	/* when we are interrupt driven we know the state */
	if (!state->autodetect)
		*std = state->curr_norm;
	else
		err = __rcar_dvdec_status(sd, NULL, std);

	mutex_unlock(&state->mutex);
	return err;
}

/*
 * rcar_dvdec_g_input_status() - V4L2 decoder i/f handler for g_input_status
 * @sd: pointer to standard V4L2 sub-device structure
 * @status: input status
 *
 * Get input status
 */
static int rcar_dvdec_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct rcar_dvdec_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	ret = __rcar_dvdec_status(sd, status, NULL);
	mutex_unlock(&state->mutex);
	return ret;
}

/*
 * rcar_dvdec_s_std() - V4L2 decoder i/f handler for s_std
 * @sd: ptr to v4l2_subdev struct
 * @std: standard input video id
 *
 * Select the video standard for the current input
 */
static int rcar_dvdec_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct rcar_dvdec_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	u16 val, dvdec_color;

	if (ret)
		return ret;

	/* all standards -> autodetect */
	if (std == V4L2_STD_ALL) {
		val = ioread16(state->base + DVDEC_BTLCR_REG);
		val &= ~DVDEC_BTLCR_DETECT_MASK;
		val |= DVDEC_BTLCR_DETECT_AUTO;
		iowrite16(val, state->base + DVDEC_BTLCR_REG);

		__rcar_dvdec_status(sd, NULL, &state->curr_norm);
		state->autodetect = true;
	} else {
		dvdec_color = v4l2_std_to_rcar_dvdec(std);

		val = ioread16(state->base + DVDEC_BTLCR_REG);
		val &= ~DVDEC_BTLCR_DETECT_MASK;
		val |= dvdec_color;
		iowrite16(val, state->base + DVDEC_BTLCR_REG);

		state->curr_norm = std;
		state->autodetect = false;
	}
	mutex_unlock(&state->mutex);
	return ret;
}

/*
 * rcar_dvdec_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 * Currently no implementation.
 */
static int rcar_dvdec_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct platform_device *pdev = v4l2_get_subdevdata(sd);
	struct rcar_dvdec_state *state = to_state(sd);

	dev_dbg(&pdev->dev, "format %d\n", state->cfmt->code);

	return 0;
}

/*
 * rcar_dvdec_g_crop() - V4L2 decoder i/f handler for g_crop
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets current cropping rectangle.
 */
static int rcar_dvdec_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct rcar_dvdec_state *state = to_state(sd);

	a->c.left = rcar_dvdec_rect[state->in_cfmt].left;
	a->c.top = rcar_dvdec_rect[state->in_cfmt].top;
	/* set current window size */
	a->c.width = rcar_dvdec_rect[state->in_cfmt].width;
	a->c.height = rcar_dvdec_rect[state->in_cfmt].height;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

/*
 * rcar_dvdec_cropcap() - V4L2 decoder i/f handler for cropcap
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets cropping limits, default cropping rectangle and pixel aspect.
 */
static int rcar_dvdec_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	struct rcar_dvdec_state *state = to_state(sd);

	a->bounds.left = rcar_dvdec_rect[state->in_cfmt].left;
	a->bounds.top = rcar_dvdec_rect[state->in_cfmt].top;
	/* set maximum window size */
	a->bounds.width = rcar_dvdec_rect[state->in_cfmt].width;
	a->bounds.height = rcar_dvdec_rect[state->in_cfmt].height;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator = 1;
	a->pixelaspect.denominator = 1;

	return 0;
}

/*
 * rcar_dvdec_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in rcar_dvdec decoder device.
 */
static int rcar_dvdec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_rcar_dvdec_sd(ctrl);
	struct rcar_dvdec_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	int val;

	if (ret)
		return ret;

	val = ctrl->val;
	switch (ctrl->id) {
	case V4L2_CID_CONTRAST:
		if ((ctrl->val < DVDEC_YGAINCR_MIN) ||
					(ctrl->val > DVDEC_YGAINCR_MAX))
			ret = -ERANGE;
		else
			iowrite16(val, state->base + DVDEC_YGAINCR_REG);
		break;
	case V4L2_CID_BLUE_BALANCE:
		if ((ctrl->val < DVDEC_CBGAINCR_MIN) ||
					(ctrl->val > DVDEC_CBGAINCR_MAX))
			ret = -ERANGE;
		else
			iowrite16(val, state->base + DVDEC_CBGAINCR_REG);
		break;
	case V4L2_CID_RED_BALANCE:
		if ((ctrl->val < DVDEC_CRGAINCR_MIN) ||
					(ctrl->val > DVDEC_CRGAINCR_MAX))
			ret = -ERANGE;
		else
			iowrite16(val, state->base + DVDEC_CRGAINCR_REG);
		break;
	case V4L2_CID_USER_R8A7794X_INPUT:
		rcar_dvdec_s_input(sd, ctrl->val);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&state->mutex);
	return ret;
}

/*
 * rcar_dvdec_initcolor() - Init for each color format
 * @sd: pointer to standard V4L2 sub-device structure
 * @sd: dvdec color format id
 *
 * Init recommendation setting of registers for each color format
 */
static void rcar_dvdec_initcolor(struct v4l2_subdev *sd, u32 dvdec_fmt)
{
	struct rcar_dvdec_state *state = to_state(sd);
	struct platform_device *pdev = v4l2_get_subdevdata(sd);

	dev_dbg(&pdev->dev, "Color format %d\n", dvdec_fmt);

	/* V Update Enable for TGCR1 to TGCR3 */
	iowrite16(0x8000, state->base + DVDEC_RUPDCR_REG);

	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].TGCR1,
		state->base + DVDEC_TGCR1_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].TGCR2,
		state->base + DVDEC_TGCR2_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].TGCR3,
		state->base + DVDEC_TGCR3_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].HAFCCR1,
		state->base + DVDEC_HAFCCR1_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].HAFCCR2,
		state->base + DVDEC_HAFCCR2_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].HAFCCR3,
		state->base + DVDEC_HAFCCR3_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].VCDWCR1,
		state->base + DVDEC_VCDWCR1_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].ACCCR1,
		state->base + DVDEC_ACCCR1_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].AGCCR1,
		state->base + DVDEC_AGCCR1_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR3,
		state->base + DVDEC_YCSCR3_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR4,
		state->base + DVDEC_YCSCR4_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR5,
		state->base + DVDEC_YCSCR5_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR6,
		state->base + DVDEC_YCSCR6_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR7,
		state->base + DVDEC_YCSCR7_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR9,
		state->base + DVDEC_YCSCR9_REG);
	iowrite16(rcar_dvdec_reg_color[dvdec_fmt].YCSCR12,
		state->base + DVDEC_YCSCR12_REG);

	return;
}

/*
 * rcar_dvdec_init2dYcNTSC() - Init coefficient for NTSC
 * @sd: pointer to standard V4L2 sub-device structure
 *
 * Init Chroma filter TAP coefficients for NTSC
 */
static void rcar_dvdec_init2dYcNTSC(struct v4l2_subdev *sd)
{
	struct rcar_dvdec_state *state = to_state(sd);
	u16 val, mode, narrow;

	val = ioread16(state->base + DVDEC_YCSCR12_REG);
	mode   = val & DVDEC_YCSCR12_MODE_MASK;
	narrow = val & DVDEC_YCSCR12_NARROW_MASK;

	if (mode == DVDEC_YCSCR12_MODE_BYPASS)
		return;

	if (mode == DVDEC_YCSCR12_MODE_CASCADE) {
		/* NTCS : Cascade Filter 1stage or 2stages */
		iowrite16(24, state->base + DVDEC_YCTWA_F0_REG);
		iowrite16(44, state->base + DVDEC_YCTWA_F1_REG);
		iowrite16(20, state->base + DVDEC_YCTWA_F2_REG);
		iowrite16(DVDEC_YCTMINUS|52, state->base + DVDEC_YCTWA_F3_REG);
		iowrite16(DVDEC_YCTMINUS|128, state->base + DVDEC_YCTWA_F4_REG);
		iowrite16(DVDEC_YCTMINUS|128, state->base + DVDEC_YCTWA_F5_REG);
		iowrite16(DVDEC_YCTMINUS|12, state->base + DVDEC_YCTWA_F6_REG);
		iowrite16(132, state->base + DVDEC_YCTWA_F7_REG);
		iowrite16(200, state->base + DVDEC_YCTWA_F8_REG);

		if (narrow == DVDEC_YCSCR12_NARROW_17TAP) {
			/* NTCS : Cascade Filter 2stages */
			iowrite16(24, state->base + DVDEC_YCTNA_F0_REG);
			iowrite16(44, state->base + DVDEC_YCTNA_F1_REG);
			iowrite16(20, state->base + DVDEC_YCTNA_F2_REG);
			iowrite16(DVDEC_YCTMINUS|52,
					state->base + DVDEC_YCTNA_F3_REG);
			iowrite16(DVDEC_YCTMINUS|128,
					state->base + DVDEC_YCTNA_F4_REG);
			iowrite16(DVDEC_YCTMINUS|128,
					state->base + DVDEC_YCTNA_F5_REG);
			iowrite16(DVDEC_YCTMINUS|12,
					state->base + DVDEC_YCTNA_F6_REG);
			iowrite16(132, state->base + DVDEC_YCTNA_F7_REG);
			iowrite16(200, state->base + DVDEC_YCTNA_F8_REG);
		}
	}

	if (mode == DVDEC_YCSCR12_MODE_TAKEOFF) {
		if (narrow == DVDEC_YCSCR12_NARROW_BYPASS) {
			/* NTCS : TAKE-OFF Filter Broad-band */
			iowrite16(0, state->base + DVDEC_YCTWA_F0_REG);
			iowrite16(0, state->base + DVDEC_YCTWA_F1_REG);
			iowrite16(0, state->base + DVDEC_YCTWA_F2_REG);
			iowrite16(DVDEC_YCTMINUS|28,
					state->base + DVDEC_YCTWA_F3_REG);
			iowrite16(96, state->base + DVDEC_YCTWA_F4_REG);
			iowrite16(228, state->base + DVDEC_YCTWA_F5_REG);
			iowrite16(916, state->base + DVDEC_YCTWA_F6_REG);
			iowrite16(204, state->base + DVDEC_YCTWA_F7_REG);
			iowrite16(1648, state->base + DVDEC_YCTWA_F8_REG);
		}

		if (narrow == DVDEC_YCSCR12_NARROW_17TAP) {
			/* NTCS : TAKE-OFF Filter Narrow-band */
			iowrite16(0, state->base + DVDEC_YCTWA_F0_REG);
			iowrite16(DVDEC_YCTMINUS|48,
					state->base + DVDEC_YCTWA_F1_REG);
			iowrite16(DVDEC_YCTMINUS|20,
					state->base + DVDEC_YCTWA_F2_REG);
			iowrite16(160, state->base + DVDEC_YCTWA_F3_REG);
			iowrite16(232, state->base + DVDEC_YCTWA_F4_REG);
			iowrite16(DVDEC_YCTMINUS|116,
					state->base + DVDEC_YCTWA_F5_REG);
			iowrite16(DVDEC_YCTMINUS|900,
					state->base + DVDEC_YCTWA_F6_REG);
			iowrite16(DVDEC_YCTMINUS|4,
					state->base + DVDEC_YCTWA_F7_REG);
			iowrite16(1392, state->base + DVDEC_YCTWA_F8_REG);
		}
	}
}

/*
 * rcar_dvdec_init2dYcPAL() - Init coefficient for PAL
 * @sd: pointer to standard V4L2 sub-device structure
 *
 * Init Chroma filter TAP coefficients for PAL
 */
static void rcar_dvdec_init2dYcPAL(struct v4l2_subdev *sd)
{
	struct rcar_dvdec_state *state = to_state(sd);
	u16 val, mode, narrow;

	val = ioread16(state->base + DVDEC_YCSCR12_REG);
	mode   = val & DVDEC_YCSCR12_MODE_MASK;
	narrow = val & DVDEC_YCSCR12_NARROW_MASK;

	if (mode == DVDEC_YCSCR12_MODE_BYPASS)
		return;

	if (mode == DVDEC_YCSCR12_MODE_CASCADE) {
		/* NTCS : Cascade Filter 1stage or 2stages */
		iowrite16(DVDEC_YCTMINUS|20, state->base + DVDEC_YCTWA_F0_REG);
		iowrite16(24, state->base + DVDEC_YCTWA_F1_REG);
		iowrite16(64, state->base + DVDEC_YCTWA_F2_REG);
		iowrite16(40, state->base + DVDEC_YCTWA_F3_REG);
		iowrite16(DVDEC_YCTMINUS|76, state->base + DVDEC_YCTWA_F4_REG);
		iowrite16(DVDEC_YCTMINUS|164, state->base + DVDEC_YCTWA_F5_REG);
		iowrite16(DVDEC_YCTMINUS|84, state->base + DVDEC_YCTWA_F6_REG);
		iowrite16(108, state->base + DVDEC_YCTWA_F7_REG);
		iowrite16(216, state->base + DVDEC_YCTWA_F8_REG);

		if (narrow == DVDEC_YCSCR12_NARROW_17TAP) {
			/* NTCS : Cascade Filter 2stages */
			iowrite16(DVDEC_YCTMINUS|20,
					state->base + DVDEC_YCTNA_F0_REG);
			iowrite16(24, state->base + DVDEC_YCTNA_F1_REG);
			iowrite16(64, state->base + DVDEC_YCTNA_F2_REG);
			iowrite16(40, state->base + DVDEC_YCTNA_F3_REG);
			iowrite16(DVDEC_YCTMINUS|76,
					state->base + DVDEC_YCTNA_F4_REG);
			iowrite16(DVDEC_YCTMINUS|164,
					state->base + DVDEC_YCTNA_F5_REG);
			iowrite16(DVDEC_YCTMINUS|84,
					state->base + DVDEC_YCTNA_F6_REG);
			iowrite16(108, state->base + DVDEC_YCTNA_F7_REG);
			iowrite16(216, state->base + DVDEC_YCTNA_F8_REG);
		}
	}

	if (mode == DVDEC_YCSCR12_MODE_TAKEOFF) {
		if (narrow == DVDEC_YCSCR12_NARROW_BYPASS) {
			/* NTCS : TAKE-OFF Filter Broad-band */
			iowrite16(0, state->base + DVDEC_YCTWA_F0_REG);
			iowrite16(0, state->base + DVDEC_YCTWA_F1_REG);
			iowrite16(0, state->base + DVDEC_YCTWA_F2_REG);
			iowrite16(16, state->base + DVDEC_YCTWA_F3_REG);
			iowrite16(59, state->base + DVDEC_YCTWA_F4_REG);
			iowrite16(85, state->base + DVDEC_YCTWA_F5_REG);
			iowrite16(DVDEC_YCTMINUS|498,
					state->base + DVDEC_YCTWA_F6_REG);
			iowrite16(DVDEC_YCTMINUS|101,
					state->base + DVDEC_YCTWA_F7_REG);
			iowrite16(878, state->base + DVDEC_YCTWA_F8_REG);
		}

		if (narrow == DVDEC_YCSCR12_NARROW_17TAP) {
			/* NTCS : TAKE-OFF Filter Narrow-band */
			iowrite16(0, state->base + DVDEC_YCTWA_F0_REG);
			iowrite16(0, state->base + DVDEC_YCTWA_F1_REG);
			iowrite16(DVDEC_YCTMINUS|23,
					state->base + DVDEC_YCTWA_F2_REG);
			iowrite16(DVDEC_YCTMINUS|46,
					state->base + DVDEC_YCTWA_F3_REG);
			iowrite16(145, state->base + DVDEC_YCTWA_F4_REG);
			iowrite16(409, state->base + DVDEC_YCTWA_F5_REG);
			iowrite16(DVDEC_YCTMINUS|918,
					state->base + DVDEC_YCTWA_F6_REG);
			iowrite16(DVDEC_YCTMINUS|363,
					state->base + DVDEC_YCTWA_F7_REG);
			iowrite16(1592, state->base + DVDEC_YCTWA_F8_REG);
		}
	}
}

/*
 * rcar_dvdec_set_params() - Set parameters
 * @sd: pointer to standard V4L2 sub-device structure
 * @width: pointer to width of input video
 * @height: pointer to height of input video
 * @code: V4l2 format code
 *
 * Set parameters of digital video decoder
 */
static int rcar_dvdec_set_params(struct v4l2_subdev *sd,
			u32 *width, u32 *height, enum v4l2_mbus_pixelcode code)
{
	struct rcar_dvdec_state *state = to_state(sd);
	struct platform_device *pdev = v4l2_get_subdevdata(sd);
	int i;
	u16 cromasr2;

	/*
	 * select format
	 */
	for (i = 0; i < ARRAY_SIZE(rcar_dvdec_cfmts); i++) {
		if (code == rcar_dvdec_cfmts[i].code) {
			state->cfmt = rcar_dvdec_cfmts + i;
			break;
		}
	}
	if (i >= ARRAY_SIZE(rcar_dvdec_cfmts))
		return -EINVAL;			/* no match format */

	cromasr2 = ioread16(state->base + DVDEC_CROMASR2_REG);
	if (cromasr2 & DVDEC_CROMASR2_ISNTSC) {
		dev_info(&pdev->dev, "Detected the NTSC video input signal\n");
		rcar_dvdec_init2dYcNTSC(sd);
	} else if (cromasr2 & DVDEC_CROMASR2_ISPAL) {
		dev_info(&pdev->dev, "Detected the PAL video input signal\n");
		rcar_dvdec_init2dYcPAL(sd);
	} else if (cromasr2 & DVDEC_CROMASR2_ISSECAM) {
		/* SECAM is not supported */
		dev_info(&pdev->dev, "Detected the SECAM video input signal\n");
	} else {
		dev_info(&pdev->dev, "Not detect any video input signal\n");
	}

	/*
	 * set window size
	 */
	*width  = rcar_dvdec_rect[state->in_cfmt].width;
	*height = rcar_dvdec_rect[state->in_cfmt].height;

	return 0;
}

/*
 * rcar_dvdec_enum_fmt() - V4L2 decoder i/f handler for enum_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @index: format index
 * @code: pointer to mediabus format
 *
 * Enumerate supported mediabus formats.
 */
static int rcar_dvdec_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(rcar_dvdec_cfmts))
		return -EINVAL;

	*code = rcar_dvdec_cfmts[index].code;

	return 0;
}

/*
 * rcar_dvdec_g_fmt() - V4L2 decoder i/f handler for g_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @mf: pointer to mediabus format structure
 *
 * Negotiate the image capture size and mediabus format.
 * Get the data format.
 */
static int rcar_dvdec_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct rcar_dvdec_state *state = to_state(sd);

	if (!state->cfmt)
		state->cfmt = rcar_dvdec_cfmts;

	mf->width	= rcar_dvdec_rect[state->in_cfmt].width;
	mf->height	= rcar_dvdec_rect[state->in_cfmt].height;
	mf->code	= state->cfmt->code;
	mf->colorspace	= state->cfmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

/*
 * rcar_dvdec_try_fmt() - V4L2 decoder i/f handler for try_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @mf: pointer to mediabus format structure
 *
 * Negotiate the image capture size and mediabus format.
 * Try a format.
 */
static int rcar_dvdec_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	struct rcar_dvdec_state *state = to_state(sd);
	int i;

	mf->width = rcar_dvdec_rect[state->in_cfmt].width;
	mf->height = rcar_dvdec_rect[state->in_cfmt].height;
	if (mf->field == V4L2_FIELD_NONE)	/* support only interlaced */
		mf->field = V4L2_FIELD_INTERLACED;

	for (i = 0; i < ARRAY_SIZE(rcar_dvdec_cfmts); i++)
		if (mf->code == rcar_dvdec_cfmts[i].code)
			break;

	if (i == ARRAY_SIZE(rcar_dvdec_cfmts)) {
		/* Unsupported format requested. Propose either */
		if (state->cfmt) {
			/* the current one or */
			mf->colorspace = state->cfmt->colorspace;
			mf->code = state->cfmt->code;
		} else {
			/* the default one */
			mf->colorspace = rcar_dvdec_cfmts[0].colorspace;
			mf->code = rcar_dvdec_cfmts[0].code;
		}
	} else {
		/* Also return the colorspace */
		mf->colorspace	= rcar_dvdec_cfmts[i].colorspace;
	}

	return 0;
}

/*
 * rcar_dvdec_s_fmt() - V4L2 decoder i/f handler for s_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @mf: pointer to mediabus format structure
 *
 * Negotiate the image capture size and mediabus format.
 * Set the data format.
 */
static int rcar_dvdec_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct rcar_dvdec_state *state = to_state(sd);
	int ret;

	ret = rcar_dvdec_set_params(sd, &mf->width, &mf->height, mf->code);

	if (!ret)
		mf->colorspace = state->cfmt->colorspace;

	return ret;
}

static int rcar_dvdec_g_mbus_config(struct v4l2_subdev *sd,
					struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_PCLK_SAMPLE_RISING |
		V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_HSYNC_ACTIVE_LOW |
		V4L2_MBUS_DATA_ACTIVE_HIGH | V4L2_MBUS_MASTER;
	cfg->type = V4L2_MBUS_BT656;

	return 0;
}

static struct platform_device_id rcar_dvdec_id_table[] = {
	{ "r8a7794x-dvdec",  RCAR_E2X },
	{},
};
MODULE_DEVICE_TABLE(platform, rcar_dvdec_id_table);

static const struct v4l2_subdev_video_ops rcar_dvdec_video_ops = {
	.querystd	= rcar_dvdec_querystd,
	.g_input_status = rcar_dvdec_g_input_status,
	.s_stream	= rcar_dvdec_s_stream,
	.cropcap	= rcar_dvdec_cropcap,
	.g_crop		= rcar_dvdec_g_crop,
	.enum_mbus_fmt	= rcar_dvdec_enum_fmt,
	.g_mbus_fmt	= rcar_dvdec_g_fmt,
	.try_mbus_fmt	= rcar_dvdec_try_fmt,
	.s_mbus_fmt	= rcar_dvdec_s_fmt,
	.g_mbus_config	= rcar_dvdec_g_mbus_config,
};

static const struct v4l2_subdev_core_ops rcar_dvdec_core_ops = {
	.s_std = rcar_dvdec_s_std,
	.queryctrl = v4l2_subdev_queryctrl,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
};

static const struct v4l2_subdev_ops rcar_dvdec_ops = {
	.core = &rcar_dvdec_core_ops,
	.video = &rcar_dvdec_video_ops,
};

static const struct v4l2_ctrl_ops rcar_dvdec_ctrl_ops = {
	.s_ctrl = rcar_dvdec_s_ctrl,
};

static const struct v4l2_ctrl_config rcar_dvdec_ctrls[] = {
	{
		.ops	= &rcar_dvdec_ctrl_ops,
		.id		= V4L2_CID_USER_R8A7794X_INPUT,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "Set Input Id",
		.min	= 0,
		.max	= 1,
		.step	= 1,
		.def	= 0,
		.flags	= 0,
	}
};

/*
 * rcar_dvdec_init_controls() - Init controls
 * @state: pointer to private state structure
 *
 * Init rcar_dvdec supported control handler.
 */
static int rcar_dvdec_init_controls(struct rcar_dvdec_state *state)
{
	int i;

	v4l2_ctrl_handler_init(&state->ctrl_hdl,
				ARRAY_SIZE(rcar_dvdec_ctrls) + 3);

	v4l2_ctrl_new_std(&state->ctrl_hdl, &rcar_dvdec_ctrl_ops,
			  V4L2_CID_CONTRAST, DVDEC_YGAINCR_MIN,
			  DVDEC_YGAINCR_MAX, 1, DVDEC_YGAINCR_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &rcar_dvdec_ctrl_ops,
			  V4L2_CID_BLUE_BALANCE, DVDEC_CBGAINCR_MIN,
			  DVDEC_CBGAINCR_MAX, 1, DVDEC_CBGAINCR_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &rcar_dvdec_ctrl_ops,
			  V4L2_CID_RED_BALANCE, DVDEC_CRGAINCR_MIN,
			  DVDEC_CRGAINCR_MAX, 1, DVDEC_CRGAINCR_DEF);
	for (i = 0; i < ARRAY_SIZE(rcar_dvdec_ctrls); ++i)
		v4l2_ctrl_new_custom(&state->ctrl_hdl,
					&rcar_dvdec_ctrls[i], NULL);

	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);

	return 0;
}

/*
 * rcar_dvdec_exit_controls() - Cleanup controls
 * @state: pointer to private state structure
 *
 * Free rcar_dvdec supported control handler.
 */
static void rcar_dvdec_exit_controls(struct rcar_dvdec_state *state)
{
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
}

/*
 * init_device - Init a digital video decoder
 * @sd: pointer to v4l2_subdev structure
 * @state: pointer to private state structure
 *
 * Initialize the digital video decoder
 */
static int init_device(struct v4l2_subdev *sd, struct rcar_dvdec_state *state)
{
	int ret;
	u16 val;

	iowrite16(DVDEC_ADCCR1_INIT, state->base + DVDEC_ADCCR1_REG);

	iowrite16(DVDEC_SYNSCR1_INIT, state->base + DVDEC_SYNSCR1_REG);
	iowrite16(DVDEC_SYNSCR2_INIT, state->base + DVDEC_SYNSCR2_REG);
	iowrite16(DVDEC_SYNSCR3_INIT, state->base + DVDEC_SYNSCR3_REG);
	iowrite16(DVDEC_SYNSCR4_INIT, state->base + DVDEC_SYNSCR4_REG);
	iowrite16(DVDEC_SYNSCR5_INIT, state->base + DVDEC_SYNSCR5_REG);

	iowrite16(DVDEC_HAFCCR1_INIT, state->base + DVDEC_HAFCCR1_REG);
	iowrite16(DVDEC_HAFCCR2_INIT, state->base + DVDEC_HAFCCR2_REG);
	iowrite16(DVDEC_HAFCCR3_INIT, state->base + DVDEC_HAFCCR3_REG);

	val = ioread16(state->base + DVDEC_VCDWCR1_REG);
	val &= ~DVDEC_VCDWCR1_IMASK;
	val |= DVDEC_VCDWCR1_INIT;
	iowrite16(val, state->base + DVDEC_VCDWCR1_REG);

	iowrite16(DVDEC_DCPCR1_INIT, state->base + DVDEC_DCPCR1_REG);
	iowrite16(DVDEC_DCPCR2_INIT, state->base + DVDEC_DCPCR2_REG);
	iowrite16(DVDEC_DCPCR3_INIT, state->base + DVDEC_DCPCR3_REG);
	iowrite16(DVDEC_DCPCR4_INIT, state->base + DVDEC_DCPCR4_REG);
	iowrite16(DVDEC_DCPCR5_INIT, state->base + DVDEC_DCPCR5_REG);
	iowrite16(DVDEC_DCPCR6_INIT, state->base + DVDEC_DCPCR6_REG);
	iowrite16(DVDEC_DCPCR7_INIT, state->base + DVDEC_DCPCR7_REG);
	iowrite16(DVDEC_DCPCR8_INIT, state->base + DVDEC_DCPCR8_REG);

	iowrite16(DVDEC_NSDCR_INIT, state->base + DVDEC_NSDCR_REG);

	/* Enable autodetection */
	if (state->autodetect) {
		iowrite16(DVDEC_BTLCR_INIT | DVDEC_BTLCR_DETECT_AUTO,
					state->base + DVDEC_BTLCR_REG);
	} else {
		ret = v4l2_std_to_rcar_dvdec(state->curr_norm);
		iowrite16(DVDEC_BTLCR_INIT | (u16)ret,
				state->base + DVDEC_BTLCR_REG);
	}

	iowrite16(DVDEC_BTGPCR_INIT, state->base + DVDEC_BTGPCR_REG);

	iowrite16(DVDEC_ACCCR1_INIT, state->base + DVDEC_ACCCR1_REG);
	iowrite16(DVDEC_ACCCR2_INIT, state->base + DVDEC_ACCCR2_REG);
	iowrite16(DVDEC_ACCCR3_INIT, state->base + DVDEC_ACCCR3_REG);

	iowrite16(DVDEC_TINTCR_INIT, state->base + DVDEC_TINTCR_REG);
	iowrite16(DVDEC_YCDCR_INIT, state->base + DVDEC_YCDCR_REG);

	iowrite16(DVDEC_AGCCR1_INIT, state->base + DVDEC_AGCCR1_REG);

	val = ioread16(state->base + DVDEC_AGCCR2_REG);
	val &= ~DVDEC_AGCCR2_IMASK;
	val |= DVDEC_AGCCR2_INIT;
	iowrite16(val, state->base + DVDEC_AGCCR2_REG);

	iowrite16(DVDEC_PKLIMITCR_INIT, state->base + DVDEC_PKLIMITCR_REG);

	iowrite16(DVDEC_RGORCR1_INIT, state->base + DVDEC_RGORCR1_REG);
	iowrite16(DVDEC_RGORCR2_INIT, state->base + DVDEC_RGORCR2_REG);
	iowrite16(DVDEC_RGORCR3_INIT, state->base + DVDEC_RGORCR3_REG);
	iowrite16(DVDEC_RGORCR4_INIT, state->base + DVDEC_RGORCR4_REG);
	iowrite16(DVDEC_RGORCR5_INIT, state->base + DVDEC_RGORCR5_REG);
	iowrite16(DVDEC_RGORCR6_INIT, state->base + DVDEC_RGORCR6_REG);
	iowrite16(DVDEC_RGORCR7_INIT, state->base + DVDEC_RGORCR7_REG);

	iowrite16(DVDEC_AFCPFCR_INIT, state->base + DVDEC_AFCPFCR_REG);
	iowrite16(DVDEC_RUPDCR_INIT, state->base + DVDEC_RUPDCR_REG);
	iowrite16(DVDEC_YCSCR8_INIT, state->base + DVDEC_YCSCR8_REG);
	iowrite16(DVDEC_YCSCR11_INIT, state->base + DVDEC_YCSCR11_REG);
	iowrite16(DVDEC_DCPCR9_INIT, state->base + DVDEC_DCPCR9_REG);

	iowrite16(DVDEC_YGAINCR_INIT, state->base + DVDEC_YGAINCR_REG);
	iowrite16(DVDEC_CBGAINCR_INIT, state->base + DVDEC_CBGAINCR_REG);
	iowrite16(DVDEC_CRGAINCR_INIT, state->base + DVDEC_CRGAINCR_REG);

	iowrite16(DVDEC_PGA_UPDATE_INIT, state->base + DVDEC_PGA_UPDATE_REG);
	iowrite16(DVDEC_PGACR_INIT, state->base + DVDEC_PGACR_REG);
	iowrite16(DVDEC_ADCCR2_INIT, state->base + DVDEC_ADCCR2_REG);

	/* read current norm */
	__rcar_dvdec_status(sd, NULL, &state->curr_norm);

	/* Select VIN0 as output direction */
	ret = rcar_dvdec_s_output(sd, 0);

	return ret;
}

/*
 * rcar_dvdec_probe - Probe a digital video decoder
 * @pdev: pointer to platform device
 *
 * Initialize the digital video decoder
 */
static int rcar_dvdec_probe(struct platform_device *pdev)
{
	struct rcar_dvdec_state *state;
	struct v4l2_subdev *sd;
	struct resource *res;
	struct soc_camera_platform_info *p = pdev->dev.platform_data;
	struct soc_camera_device *icd;
	struct soc_camera_host *ici;
	int ret;

	dev_info(&pdev->dev, "Probing rcar-dvdec\n");
	if (!p)
		return -EINVAL;
	if (!p->icd) {
		dev_err(&pdev->dev,
			"Platform has not set soc_camera_device pointer!\n");
		return -EINVAL;
	}

	state = kzalloc(sizeof(struct rcar_dvdec_state), GFP_KERNEL);
	if (state == NULL) {
		dev_err(&pdev->dev, "Failed to alloc memory\n");
		ret = -ENOMEM;
		goto err;
	}

	icd = p->icd;
	icd->control = &pdev->dev;
	ici = to_soc_camera_host(icd->parent);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Failed to get platform resource\n");
		ret = -EINVAL;
		goto err_free_state;
	}

	state->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(state->base)) {
		dev_err(&pdev->dev, "Failed to remap resource\n");
		ret = PTR_ERR(state->base);
		goto err_free_state;
	}

	pm_suspend_ignore_children(&pdev->dev, true);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	mutex_init(&state->mutex);
	state->autodetect = true;
	state->input = 0;
	state->output = 0;
	state->in_cfmt = DVDEC_NTSC358;
	state->cfmt = &rcar_dvdec_cfmts[0];
	sd = &state->sd;

	v4l2_subdev_init(sd, &rcar_dvdec_ops);
	v4l2_set_subdevdata(sd, pdev);
	strncpy(sd->name, dev_name(&pdev->dev), V4L2_SUBDEV_NAME_SIZE);

	ret = v4l2_device_register_subdev(&ici->v4l2_dev, sd);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register subdev\n");
		goto err_free_state;
	}

	ret = rcar_dvdec_init_controls(state);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init controls\n");
		goto err_unreg_subdev;
	}

	/* clock supply to the A/D converter */
	iowrite16(DVDEC_PSAV_ON, state->base + DVDEC_ADCCR3_REG);

	ret = init_device(sd, state);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init device\n");
		goto err_free_ctrl;
	}

	platform_set_drvdata(pdev, sd);
	return 0;

err_free_ctrl:
	rcar_dvdec_exit_controls(state);
err_unreg_subdev:
	pm_runtime_put(&pdev->dev);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
err_free_state:
	kfree(state);
err:
	dev_err(&pdev->dev, "Failed to probe: %d\n", ret);
	return ret;
}

/*
 * adv7180_remove - Remove digital video decoder support
 * @pdev: pointer to platform device
 *
 * Reset the digital video decoder
 */
static int rcar_dvdec_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct rcar_dvdec_state *state = to_state(sd);

	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	pm_runtime_put(&pdev->dev);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static struct platform_driver rcar_dvdec = {
	.probe   = rcar_dvdec_probe,
	.remove  = rcar_dvdec_remove,
	.driver  = {
		.name	= "rcar-dvdec",
		.owner	= THIS_MODULE,
	},
	.id_table	= rcar_dvdec_id_table,
};

module_platform_driver(rcar_dvdec);

MODULE_DESCRIPTION("R-Car E2X Digital Video Decoder driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1.0");
MODULE_ALIAS("platform:rcar-dvdec");
