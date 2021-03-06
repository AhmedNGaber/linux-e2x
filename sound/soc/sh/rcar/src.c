/*
 * Renesas R-Car SRC support
 *
 * Copyright (C) 2014-2015 Renesas Electronics Corporation
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "rsnd.h"

#define SRC_NAME "src"

#if 1
#define SYNC_SRC
#else
#undef SYNC_SRC
#endif

/* module params */
static unsigned int sync_src;
module_param(sync_src, uint, S_IRUGO);

struct rsnd_src {
	struct rsnd_src_platform_info *info; /* rcar_snd.h */
	struct rsnd_mod mod;
	struct rsnd_dai *rdai;
	int    err_uf_srco, err_of_srco, err_of_srci, err_uf_srci;
	long   reconvert_rate;
};

#define RSND_SRC_NAME_SIZE 16

#define rsnd_src_convert_rate(p) ((p)->info->convert_rate)
#define rsnd_mod_to_src(_mod)				\
	container_of((_mod), struct rsnd_src, mod)
#define rsnd_src_dma_available(src) \
	rsnd_dma_available(rsnd_mod_to_dma(&(src)->mod))

#define for_each_rsnd_src(pos, priv, i)				\
	for ((i) = 0;						\
	     ((i) < rsnd_src_nr(priv)) &&			\
	     ((pos) = (struct rsnd_src *)(priv)->src + i);	\
	     i++)

/* SRCx_INT_ENABLE0 */
#define	SRC_INT_EN0_UF_SRCO_IE	(1<<13)		/* for syncSRC  */
#define	SRC_INT_EN0_OF_SRCO_IE	(1<<12)		/* for asyncSRC */
#define	SRC_INT_EN0_OF_SRCI_IE	(1<<9)		/* for syncSRC  */
#define	SRC_INT_EN0_UF_SRCI_IE	(1<<8)		/* for asyncSRC */

/* SCU_SYSTEM_STATUS0 (for syncSRC) */
#define	SCU_SYS_ST0_OF_SRC_O		(1<<16)
#define	SCU_SYS_ST0_UF_SRC_I		(1<<0)

/* SCU_SYSTEM_INT_ENABLE0 (for syncSRC) */
#define	SCU_SYS_INTEN0_OF_SRC_O_IE	(1<<16)
#define	SCU_SYS_INTEN0_UF_SRC_I_IE	(1<<0)

/* SCU_SYSTEM_STATUS1 (for asyncSRC) */
#define	SCU_SYS_ST1_UF_SRC_O		(1<<16)
#define	SCU_SYS_ST1_OF_SRC_I		(1<<0)

/* SCU_SYSTEM_INT_ENABLE1 (for asyncSRC) */
#define	SCU_SYS_INTEN1_UF_SRC_O_IE	(1<<16)
#define	SCU_SYS_INTEN1_OF_SRC_I_IE	(1<<0)

/*
 *		image of SRC (Sampling Rate Converter)
 *
 * 96kHz   <-> +-----+	48kHz	+-----+	 48kHz	+-------+
 * 48kHz   <-> | SRC | <------>	| SSI |	<----->	| codec |
 * 44.1kHz <-> +-----+		+-----+		+-------+
 * ...
 *
 */

/*
 * src.c is caring...
 *
 * Gen1
 *
 * [mem] -> [SRU] -> [SSI]
 *        |--------|
 *
 * Gen2
 *
 * [mem] -> [SRC] -> [SSIU] -> [SSI]
 *        |-----------------|
 */

/*
 *	How to use SRC bypass mode for debugging
 *
 * SRC has bypass mode, and it is useful for debugging.
 * In Gen2 case,
 * SRCm_MODE controls whether SRC is used or not
 * SSI_MODE0 controls whether SSIU which receives SRC data
 * is used or not.
 * Both SRCm_MODE/SSI_MODE0 settings are needed if you use SRC,
 * but SRC bypass mode needs SSI_MODE0 only.
 *
 * This driver request
 * struct rsnd_src_platform_info {
 *	u32 convert_rate;
 *	int dma_id;
 * }
 *
 * rsnd_src_convert_rate() indicates
 * above convert_rate, and it controls
 * whether SRC is used or not.
 *
 * ex) doesn't use SRC
 * static struct rsnd_dai_platform_info rsnd_dai = {
 *	.playback = { .ssi = &rsnd_ssi[0], },
 * };
 *
 * ex) uses SRC
 * static struct rsnd_src_platform_info rsnd_src[] = {
 *	RSND_SCU(48000, 0),
 *	...
 * };
 * static struct rsnd_dai_platform_info rsnd_dai = {
 *	.playback = { .ssi = &rsnd_ssi[0], .src = &rsnd_src[0] },
 * };
 *
 * ex) uses SRC bypass mode
 * static struct rsnd_src_platform_info rsnd_src[] = {
 *	RSND_SCU(0, 0),
 *	...
 * };
 * static struct rsnd_dai_platform_info rsnd_dai = {
 *	.playback = { .ssi = &rsnd_ssi[0], .src = &rsnd_src[0] },
 * };
 *
 */

/*
 *		Gen1/Gen2 common functions
 */
int rsnd_src_ssiu_start(struct rsnd_mod *ssi_mod,
			struct rsnd_dai *rdai,
			int use_busif)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(ssi_mod);
	int ssi_id = rsnd_mod_id(ssi_mod);

	/*
	 * SSI_MODE0
	 */
	rsnd_mod_bset(ssi_mod, SSI_MODE0, (1 << ssi_id),
		      !use_busif << ssi_id);

	/*
	 * SSI_MODE1
	 */
	if (rsnd_ssi_is_pin_sharing(ssi_mod)) {
		int shift = -1;
		switch (ssi_id) {
		case 1:
			shift = 0;
			break;
		case 2:
			shift = 2;
			break;
		case 4:
			shift = 16;
			break;
		}

		if (shift >= 0)
			rsnd_mod_bset(ssi_mod, SSI_MODE1,
				      0x3 << shift,
				      rsnd_dai_is_clk_master(rdai) ?
				      0x2 << shift : 0x1 << shift);
	}

	/*
	 * DMA settings for SSIU
	 */
	if (use_busif) {
		u32 val = rsnd_get_dalign(ssi_mod, io);

		rsnd_mod_write(ssi_mod, BUSIF_DALIGN, val);
		rsnd_mod_write(ssi_mod, SSI_BUSIF_ADINR,
			       rsnd_get_adinr(ssi_mod));
		rsnd_mod_write(ssi_mod, SSI_BUSIF_MODE,  1);
		rsnd_mod_write(ssi_mod, SSI_CTRL, 0x1);
	}

	return 0;
}

int rsnd_src_ssiu_stop(struct rsnd_mod *ssi_mod,
			struct rsnd_dai *rdai,
			int use_busif)
{
	/*
	 * DMA settings for SSIU
	 */
	if (use_busif)
		rsnd_mod_write(ssi_mod, SSI_CTRL, 0);

	return 0;
}

int rsnd_src_enable_ssi_irq(struct rsnd_mod *ssi_mod,
			    struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(ssi_mod);

	/* enable PIO interrupt if Gen2 */
	if (rsnd_is_gen2(priv))
		rsnd_mod_write(ssi_mod, INT_ENABLE, 0x0f000000);

	return 0;
}

int rsnd_src_enable_dma_ssi_irq(struct rsnd_mod *ssi_mod,
				struct rsnd_dai *rdai,
				int use_busif)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(ssi_mod);

	/* enable SSI interrupt if Gen2 */
	if (rsnd_is_gen2(priv))
		rsnd_mod_write(ssi_mod, INT_ENABLE, 0x0e000000);

	return 0;
}

int rsnd_src_disable_dma_ssi_irq(struct rsnd_mod *ssi_mod,
				 struct rsnd_dai *rdai,
				 int use_busif)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(ssi_mod);

	/* enable SSI interrupt if Gen2 */
	if (rsnd_is_gen2(priv))
		rsnd_mod_write(ssi_mod, INT_ENABLE, 0x00000000);

	return 0;
}

unsigned int rsnd_src_get_ssi_rate(struct rsnd_priv *priv,
				   struct rsnd_dai_stream *io,
				   struct snd_pcm_runtime *runtime)
{
	struct rsnd_mod *src_mod = rsnd_io_to_mod_src(io);
	struct rsnd_src *src;
	unsigned int rate = 0;

	if (src_mod) {
		src = rsnd_mod_to_src(src_mod);

		/*
		 * return convert rate if SRC is used,
		 * otherwise, return runtime->rate as usual
		 */
		rate = rsnd_src_convert_rate(src);
	}

	if (!rate)
		rate = runtime->rate;

	return rate;
}

static int rsnd_src_use_syncsrc(struct rsnd_mod *mod)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);

#ifdef SYNC_SRC
	int use_syncsrc = 1;
#else
	int use_syncsrc = 0;
#endif
	sync_src = use_syncsrc;

	if (!rsnd_info_is_playback(priv, src) && rsnd_io_to_mod_dvc(io))
		use_syncsrc = 0;

	return use_syncsrc;
}

static inline unsigned int muldiv32(unsigned int a, unsigned int b,
				    unsigned int c, unsigned int *r)
{
	u_int64_t n = (u_int64_t) a * b;
	if (c == 0) {
		BUG_ON(!n);
		*r = 0;
		return UINT_MAX;
	}
	n = div_u64_rem(n, c, r);
	if (n >= UINT_MAX) {
		*r = 0;
		return UINT_MAX;
	}
	return n;
}

static int rsnd_src_set_convert_rate(struct rsnd_mod *mod,
				     struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	u32 convert_rate = rsnd_src_convert_rate(src);
	u32 fsrate = 0;
	u32 remain = 0;

	if (src->reconvert_rate)
		convert_rate = src->reconvert_rate;

	if (convert_rate) {
		if (rsnd_dai_is_play(rdai, io)) {
			fsrate = muldiv32(0x0400000, runtime->rate,
					  convert_rate, &remain);
		} else {
			fsrate = muldiv32(0x0400000, convert_rate,
					  runtime->rate, &remain);
		}
	} else if (rsnd_src_use_syncsrc(mod))
		fsrate = 0x0400000;

	/* set/clear soft reset */
	rsnd_mod_write(mod, SRC_SWRSR, 0);
	rsnd_mod_write(mod, SRC_SWRSR, 1);

	if (rsnd_src_convert_rate(src) || rsnd_src_use_syncsrc(mod))
		rsnd_mod_bset(mod, SRC_ROUTE_MODE0, 1, 1);

	/*
	 * Initialize the operation of the SRC internal circuits
	 */
	rsnd_mod_write(mod, SRC_SRCIR, 1);

	/* Set channel number and output bit length */
	rsnd_mod_write(mod, SRC_ADINR, rsnd_get_adinr(mod));

	/* Enable the initial value of IFS */
	if (fsrate) {
		rsnd_mod_write(mod, SRC_IFSCR, 1);

		/* Set initial value of IFS */
		rsnd_mod_write(mod, SRC_IFSVR, fsrate);
	}

	/* use DMA transfer */
	rsnd_mod_write(mod, SRC_BUSIF_MODE, 1);

	return 0;
}

static void rsnd_src_set_reconvert_rate(struct rsnd_mod *mod)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct snd_pcm_runtime *runtime;
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	u32 fsrate;
	u32 remain = 0;

	if (io->substream == NULL)
		return;

	runtime = rsnd_io_to_runtime(io);

	if (runtime == NULL)
		return;

	if (!runtime->rate)
		return;

	if (src->reconvert_rate) {
		if (io->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			fsrate = muldiv32(0x0400000, runtime->rate,
					  src->reconvert_rate, &remain);
		} else {
			fsrate = muldiv32(0x0400000, src->reconvert_rate,
					  runtime->rate, &remain);
		}
	} else
		fsrate = 0x0400000;

	/* Set initial value of IFS */
	rsnd_mod_write(mod, SRC_IFSVR, fsrate);
}

static int rsnd_src_init(struct rsnd_mod *mod,
			 struct rsnd_dai *rdai)
{
	rsnd_mod_hw_start(mod);

	return 0;
}

static int rsnd_src_quit(struct rsnd_mod *mod,
			 struct rsnd_dai *rdai)
{
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct device *dev = rsnd_priv_to_dev(priv);

	rsnd_mod_hw_stop(mod);

	if (src->err_uf_srco > 0)
		dev_warn(dev, "src uf_srco err = %d\n", src->err_uf_srco);
	if (src->err_of_srco > 0)
		dev_warn(dev, "src of_srco err = %d\n", src->err_of_srco);
	if (src->err_of_srci > 0)
		dev_warn(dev, "src of_srci err = %d\n", src->err_of_srci);
	if (src->err_uf_srci > 0)
		dev_warn(dev, "src uf_srci err = %d\n", src->err_uf_srci);

	return 0;
}

static int rsnd_src_stop(struct rsnd_mod *mod,
			 struct rsnd_dai *rdai)
{
	struct rsnd_src *src = rsnd_mod_to_src(mod);

	if (rsnd_src_convert_rate(src) || rsnd_src_use_syncsrc(mod))
		rsnd_mod_write(mod, SRC_ROUTE_MODE0, 0);

	return 0;
}

/*
 *		Gen1 functions
 */
static int rsnd_src_set_route_gen1(struct rsnd_mod *mod,
				   struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct src_route_config {
		u32 mask;
		int shift;
	} routes[] = {
		{ 0xF,  0, }, /* 0 */
		{ 0xF,  4, }, /* 1 */
		{ 0xF,  8, }, /* 2 */
		{ 0x7, 12, }, /* 3 */
		{ 0x7, 16, }, /* 4 */
		{ 0x7, 20, }, /* 5 */
		{ 0x7, 24, }, /* 6 */
		{ 0x3, 28, }, /* 7 */
		{ 0x3, 30, }, /* 8 */
	};
	u32 mask;
	u32 val;
	int id;

	id = rsnd_mod_id(mod);
	if (id < 0 || id >= ARRAY_SIZE(routes))
		return -EIO;

	/*
	 * SRC_ROUTE_SELECT
	 */
	val = rsnd_dai_is_play(rdai, io) ? 0x1 : 0x2;
	val = val		<< routes[id].shift;
	mask = routes[id].mask	<< routes[id].shift;

	rsnd_mod_bset(mod, SRC_ROUTE_SEL, mask, val);

	return 0;
}

static int rsnd_src_set_convert_timing_gen1(struct rsnd_mod *mod,
					    struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	u32 convert_rate = rsnd_src_convert_rate(src);
	u32 mask;
	u32 val;
	int shift;
	int id = rsnd_mod_id(mod);
	int ret;

	/*
	 * SRC_TIMING_SELECT
	 */
	shift	= (id % 4) * 8;
	mask	= 0x1F << shift;

	/*
	 * ADG is used as source clock if SRC was used,
	 * then, SSI WS is used as destination clock.
	 * SSI WS is used as source clock if SRC is not used
	 * (when playback, source/destination become reverse when capture)
	 */
	ret = 0;
	if (convert_rate) {
		/* use ADG */
		val = 0;
		ret = rsnd_adg_set_convert_clk_gen1(priv, mod,
						    runtime->rate,
						    convert_rate);
	} else if (8 == id) {
		/* use SSI WS, but SRU8 is special */
		val = id << shift;
	} else {
		/* use SSI WS */
		val = (id + 1) << shift;
	}

	if (ret < 0)
		return ret;

	switch (id / 4) {
	case 0:
		rsnd_mod_bset(mod, SRC_TMG_SEL0, mask, val);
		break;
	case 1:
		rsnd_mod_bset(mod, SRC_TMG_SEL1, mask, val);
		break;
	case 2:
		rsnd_mod_bset(mod, SRC_TMG_SEL2, mask, val);
		break;
	}

	return 0;
}

static int rsnd_src_set_convert_rate_gen1(struct rsnd_mod *mod,
					  struct rsnd_dai *rdai)
{
	int ret;

	ret = rsnd_src_set_convert_rate(mod, rdai);
	if (ret < 0)
		return ret;

	/* Select SRC mode (fixed value) */
	rsnd_mod_write(mod, SRC_SRCCR, 0x00010110);

	/* Set the restriction value of the FS ratio (98%) */
	rsnd_mod_write(mod, SRC_MNFSR,
		       rsnd_mod_read(mod, SRC_IFSVR) / 100 * 98);

	/* no SRC_BFSSR settings, since SRC_SRCCR::BUFMD is 0 */

	return 0;
}

static int rsnd_src_probe_gen1(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct device *dev = rsnd_priv_to_dev(priv);

	dev_dbg(dev, "%s (Gen1) is probed\n", rsnd_mod_name(mod));

	return 0;
}

static int rsnd_src_init_gen1(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	int ret;
	u32 val;

	ret = rsnd_src_init(mod, rdai);
	if (ret < 0)
		return ret;

	ret = rsnd_src_set_route_gen1(mod, rdai);
	if (ret < 0)
		return ret;

	ret = rsnd_src_set_convert_rate_gen1(mod, rdai);
	if (ret < 0)
		return ret;

	ret = rsnd_src_set_convert_timing_gen1(mod, rdai);
	if (ret < 0)
		return ret;

	val = rsnd_get_dalign(mod, io);
	rsnd_mod_write(mod, SRC_BUSIF_DALIGN, val);
	rsnd_mod_write(mod, SRC_SRCIR, 0);

	return 0;
}

static int rsnd_src_start_gen1(struct rsnd_mod *mod,
			       struct rsnd_dai *rdai)
{
	int id = rsnd_mod_id(mod);

	rsnd_mod_bset(mod, SRC_ROUTE_CTRL, (1 << id), (1 << id));
	rsnd_mod_write(mod, SRC_SRCIR, 0);

	return 0;
}

static int rsnd_src_stop_gen1(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	int id = rsnd_mod_id(mod);

	rsnd_mod_bset(mod, SRC_ROUTE_CTRL, (1 << id), 0);

	return rsnd_src_stop(mod, rdai);
}

static struct rsnd_mod_ops rsnd_src_gen1_ops = {
	.name	= SRC_NAME,
	.probe	= rsnd_src_probe_gen1,
	.init	= rsnd_src_init_gen1,
	.quit	= rsnd_src_quit,
	.start	= rsnd_src_start_gen1,
	.stop	= rsnd_src_stop_gen1,
};

/*
 *		Gen2 functions
 */
static int rsnd_src_stop_start_gen2(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai);

static void rsnd_src_irq_enable(struct rsnd_mod *mod, struct rsnd_dai *rdai)
{
	int is_use_syncsrc = rsnd_src_use_syncsrc(mod);
	int src_id = rsnd_mod_id(mod);

	if (is_use_syncsrc) {

		/* Clear SCU_SYSTEM_STATUS1 */
		/*  (Writing 1 initializes the bit.) */
		rsnd_mod_bset(mod, SCU_SYS_STATUS1,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
					SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
					SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id);

		/* Enable SRCx_INT_ENABLE0 */
		rsnd_mod_bset(mod, SRC_INT_EN0,
			      SRC_INT_EN0_UF_SRCO_IE | SRC_INT_EN0_OF_SRCI_IE,
			      SRC_INT_EN0_UF_SRCO_IE | SRC_INT_EN0_OF_SRCI_IE);

		/* Enable SCU_SYSTEM_INT_ENABLE1 */
		rsnd_mod_bset(mod, SCU_SYS_INT_EN1,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
					SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
					SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id);
	} else {
		/* Clear SCU_SYSTEM_STATUS0 */
		/*  (Writing 1 initializes the bit.) */
		rsnd_mod_bset(mod, SCU_SYS_STATUS0,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
					SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
					SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id);

		/* Enable SRCx_INT_ENABLE0 */
		rsnd_mod_bset(mod, SRC_INT_EN0,
			      SRC_INT_EN0_OF_SRCO_IE | SRC_INT_EN0_UF_SRCI_IE,
			      SRC_INT_EN0_OF_SRCO_IE | SRC_INT_EN0_UF_SRCI_IE);

		/* Enable SCU_SYSTEM_INT_ENABLE0 */
		rsnd_mod_bset(mod, SCU_SYS_INT_EN0,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
					SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
					SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id);
	}
}

static void rsnd_src_irq_disable(struct rsnd_mod *mod, struct rsnd_dai *rdai)
{
	int is_use_syncsrc = rsnd_src_use_syncsrc(mod);
	int src_id = rsnd_mod_id(mod);

	/* Disable SRCx_INT_ENABLE0 */
	rsnd_mod_write(mod, SRC_INT_EN0, 0);

	if (is_use_syncsrc) {
		/* Disable SCU_SYSTEM_INT_ENABLE1 */
		rsnd_mod_bset(mod, SCU_SYS_INT_EN1,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
				SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id, 0);

		/* Clear SCU_SYSTEM_STATUS1 */
		/*  (Writing 1 initializes the bit.) */
		rsnd_mod_bset(mod, SCU_SYS_STATUS1,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
					SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id,
			      (SCU_SYS_INTEN1_UF_SRC_O_IE |
					SCU_SYS_INTEN1_OF_SRC_I_IE) << src_id);

	} else {
		/* Disable SCU_SYSTEM_INT_ENABLE0 */
		rsnd_mod_bset(mod, SCU_SYS_INT_EN0,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
				SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id, 0);

		/* Clear SCU_SYSTEM_STATUS0 */
		/*  (Writing 1 initializes the bit.) */
		rsnd_mod_bset(mod, SCU_SYS_STATUS0,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
					SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id,
			      (SCU_SYS_INTEN0_OF_SRC_O_IE |
					SCU_SYS_INTEN0_UF_SRC_I_IE) << src_id);
	}
}

static irqreturn_t rsnd_src_interrupt_gen2(int irq, void *data)
{
	struct rsnd_src *src = data;
	struct rsnd_mod *mod = &src->mod;
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	u32 status, sys_status0, sys_status1;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;

	rsnd_lock(priv, flags);

	/* ignore all cases if not working */
	if (!rsnd_io_is_working(io))
		goto rsnd_src_interrupt_gen2_out;

	sys_status0 = rsnd_mod_read(mod, SCU_SYS_STATUS0);
	sys_status1 = rsnd_mod_read(mod, SCU_SYS_STATUS1);
	status = rsnd_mod_read(mod, SRC_STATUS);

	if (io && status) {
		struct rsnd_dai *rdai = src->rdai;

		dev_dbg(dev, "SRC%d interrupt\n", rsnd_mod_id(&src->mod));

		if (status & SRC_INT_EN0_UF_SRCO_IE)
			src->err_uf_srco++;
		if (status & SRC_INT_EN0_OF_SRCO_IE)
			src->err_of_srco++;
		if (status & SRC_INT_EN0_OF_SRCI_IE)
			src->err_of_srci++;
		if (status & SRC_INT_EN0_UF_SRCI_IE)
			src->err_uf_srci++;

		/* STOP SRC and STAR SRC */
		rsnd_src_stop_start_gen2(mod, rdai);

		ret = IRQ_HANDLED;
	}

rsnd_src_interrupt_gen2_out:
	rsnd_unlock(priv, flags);

	return ret;
}

static int rsnd_src_set_convert_rate_gen2(struct rsnd_mod *mod,
					  struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	uint ratio;
	int ret;

	/* 6 - 1/6 are very enough ratio for SRC_BSDSR */
	if (!rsnd_src_convert_rate(src))
		ratio = 0;
	else if (rsnd_src_convert_rate(src) > runtime->rate)
		ratio = 100 * rsnd_src_convert_rate(src) / runtime->rate;
	else
		ratio = 100 * runtime->rate / rsnd_src_convert_rate(src);

	if (ratio > 600) {
		dev_err(dev, "FSO/FSI ratio error\n");
		return -EINVAL;
	}

	ret = rsnd_src_set_convert_rate(mod, rdai);
	if (ret < 0)
		return ret;

	rsnd_mod_write(mod, SRC_SRCCR, 0x00011110);

	if (rsnd_src_use_syncsrc(mod)) {
		rsnd_mod_bset(mod, SRC_SRCCR, 0x1, 0x1);

		if (rsnd_info_is_playback(priv, src))
			rsnd_mod_bset(mod, SRC_ROUTE_MODE0, 0x01 << 24, 0x01 << 24);
		else
			rsnd_mod_bset(mod, SRC_ROUTE_MODE0, 0x01 << 25, 0x01 << 25);
	}

	switch (rsnd_mod_id(mod)) {
	case 5:
	case 6:
	case 7:
	case 8:
		rsnd_mod_write(mod, SRC_BSDSR, 0x02400000);
		break;
	default:
		rsnd_mod_write(mod, SRC_BSDSR, 0x01800000);
		break;
	}

	rsnd_mod_write(mod, SRC_BSISR, 0x00100060);

	return 0;
}

static int rsnd_src_set_convert_timing_gen2(struct rsnd_mod *mod,
					    struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	u32 convert_rate = rsnd_src_convert_rate(src);
	struct rsnd_mod *ssi_mod = rsnd_io_to_mod_ssi(io);
	int ret;

	if (convert_rate) {
		rsnd_ssi_access_enable(ssi_mod, rdai);
		ret = rsnd_adg_set_convert_clk_gen2(mod, rdai, io,
						    runtime->rate,
						    convert_rate);
		rsnd_ssi_access_disable(ssi_mod, rdai);
	} else
		ret = rsnd_adg_set_convert_timing_gen2(mod, rdai, io);

	return ret;
}

static int rsnd_src_probe_gen2(struct rsnd_mod *mod,
			       struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	int ret;
	int irq = src->info->irq_id;
	int len;
	char *name;

	len = strlen(dev_name(dev)) + 16;
	name = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	snprintf(name, len, "%s  src.%d", dev_name(dev),
					rsnd_mod_id(&src->mod));

	ret = devm_request_irq(dev, irq,
			       rsnd_src_interrupt_gen2,
			       0,
			       name, src);
	if (ret) {
		dev_err(dev, "SRC request interrupt failed\n");
		goto rsnd_src_probe_gen2_irq_err;
	}

	ret = rsnd_dma_init(priv,
			    rsnd_mod_to_dma(mod),
			    rsnd_info_is_playback(priv, src),
			    src->info->dma_id);
	if (ret < 0) {
		dev_err(dev, "SRC DMA failed\n");
		goto rsnd_src_probe_gen2_dma_err;
	}

	dev_dbg(dev, "%s (Gen2) is probed\n", rsnd_mod_name(mod));

	return ret;

rsnd_src_probe_gen2_dma_err:
	devm_free_irq(dev, irq, src);
rsnd_src_probe_gen2_irq_err:
	return ret;
}

static int rsnd_src_remove_gen2(struct rsnd_mod *mod,
				struct rsnd_dai *rdai)
{
	rsnd_dma_quit(rsnd_mod_to_priv(mod), rsnd_mod_to_dma(mod));

	return 0;
}

static int rsnd_src_init_gen2(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	int ret;
	u32 val;

	src->err_uf_srco = 0;
	src->err_of_srco = 0;
	src->err_of_srci = 0;
	src->err_uf_srci = 0;

	ret = rsnd_src_init(mod, rdai);
	if (ret < 0)
		return ret;

	ret = rsnd_src_set_convert_rate_gen2(mod, rdai);
	if (ret < 0)
		return ret;

	ret = rsnd_src_set_convert_timing_gen2(mod, rdai);
	if (ret < 0)
		return ret;

	val = rsnd_get_dalign(mod, io);
	rsnd_mod_write(mod, SRC_BUSIF_DALIGN, val);
	rsnd_mod_write(mod, SRC_SRCIR, 0);

	return 0;
}

static int rsnd_src_dma_start(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_src *src = rsnd_mod_to_src(mod);

	rsnd_dma_start(rsnd_mod_to_dma(&src->mod));

	return 0;
}

static int rsnd_src_start_gen2(struct rsnd_mod *mod,
			       struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	u32 val;

	if (rsnd_src_use_syncsrc(mod))
		val = 0x11;
	else
		val = rsnd_io_has_cmd(io) ? 0x01 : 0x11;

	rsnd_mod_write(mod, SRC_CTRL, val);

	rsnd_src_irq_enable(mod, rdai);

	return 0;
}

/*
 * stop 1st : clear src.start_out
 *  during --> audio dma-pp stop.
 * stop 2nd : clear src.start_in
 */
static int rsnd_src_stop_gen2(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	u32 status, val;

	status = rsnd_mod_read(mod, SRC_CTRL);
	if (!status)
		return 0;

	/*
	 * is playback
	 */
	if (rsnd_dai_is_play(rdai, io)) {
		rsnd_src_irq_disable(mod, rdai);
		rsnd_mod_write(mod, SRC_CTRL, 0);

		return rsnd_src_stop(mod, rdai);
	}

	/*
	 * is capture
	 */
	/* check stop status */
	val = rsnd_mod_read(mod, SRC_INT_EN0);
	if (val) {
		rsnd_src_irq_disable(mod, rdai);
		status = 0x01;		/* clear start_out */
	} else
		status = 0;		/* clear start_in */

	rsnd_mod_write(mod, SRC_CTRL, status);
	if (!status)
		return rsnd_src_stop(mod, rdai);

	return 1;	/* please re-call */
}

static int rsnd_src_dma_stop(struct rsnd_mod *mod,
			     struct rsnd_dai *rdai)
{
	struct rsnd_src *src = rsnd_mod_to_src(mod);

	rsnd_dma_stop(rsnd_mod_to_dma(&src->mod));

	return 0;
}

static int rsnd_src_stop_start_gen2(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	/* STOP, clear error flags. */
	rsnd_src_irq_disable(mod, rdai);

	/* START */
	rsnd_src_irq_enable(mod, rdai);

	return 0;
}

static int rsnd_src_reconvert_rate_info(struct snd_kcontrol *kctrl,
			       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 192000;
	uinfo->value.integer.step = 1;

	return 0;
}

static int rsnd_src_reconvert_rate_get(struct snd_kcontrol *kctrl,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct rsnd_mod *mod = snd_kcontrol_chip(kctrl);
	struct rsnd_src *src = rsnd_mod_to_src(mod);

	ucontrol->value.integer.value[0] = src->reconvert_rate;

	return 0;
}

static int rsnd_src_reconvert_rate_put(struct snd_kcontrol *kctrl,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct rsnd_mod *mod = snd_kcontrol_chip(kctrl);
	struct rsnd_src *src = rsnd_mod_to_src(mod);
	int change = 0;

	if (ucontrol->value.integer.value[0] < 0 ||
	    ucontrol->value.integer.value[0] > 192000)
		return -EINVAL;

	change |= (ucontrol->value.integer.value[0] != src->reconvert_rate);

	if (change) {
		src->reconvert_rate = ucontrol->value.integer.value[0];
		rsnd_src_set_reconvert_rate(mod);
	}
	return change;
}

static int rsnd_src_pcm_new_gen2(struct rsnd_mod *mod,
			    struct rsnd_dai *rdai,
			    struct snd_soc_pcm_runtime *rtd)
{
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct snd_card *card = rtd->card->snd_card;
	struct snd_kcontrol *kctrl;
	static struct snd_kcontrol_new knew = {
		.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
		.info		= rsnd_src_reconvert_rate_info,
		.get		= rsnd_src_reconvert_rate_get,
		.put		= rsnd_src_reconvert_rate_put,
	};
	int ret;

	if (rsnd_dai_is_play(rdai, io))
		knew.name = "Playback ReConvert Rate";
	else
		knew.name = "Capture ReConvert Rate";

	kctrl = snd_ctl_new1(&knew, mod);
	if (!kctrl)
		return -ENOMEM;

	ret = snd_ctl_add(card, kctrl);
	if (ret < 0)
		return ret;

	return 0;
}

static struct rsnd_mod_ops rsnd_src_gen2_ops = {
	.name		= SRC_NAME,
	.probe		= rsnd_src_probe_gen2,
	.remove		= rsnd_src_remove_gen2,
	.init		= rsnd_src_init_gen2,
	.quit		= rsnd_src_quit,
	.dma_start	= rsnd_src_dma_start,
	.start		= rsnd_src_start_gen2,
	.stop		= rsnd_src_stop_gen2,
	.dma_stop	= rsnd_src_dma_stop,
	.pcm_new	= rsnd_src_pcm_new_gen2,
};

struct rsnd_mod *rsnd_src_mod_get(struct rsnd_priv *priv, int id)
{
	if (WARN_ON(id < 0 || id >= rsnd_src_nr(priv)))
		id = 0;

	return &((struct rsnd_src *)(priv->src) + id)->mod;
}

static void rsnd_of_parse_src(struct platform_device *pdev,
			      const struct rsnd_of_data *of_data,
			      struct rsnd_priv *priv)
{
	struct device_node *src_node;
	struct device_node *np;
	struct rcar_snd_info *info = rsnd_priv_to_info(priv);
	struct rsnd_src_platform_info *src_info;
	struct device *dev = &pdev->dev;
	int nr;
	int i;

	if (!of_data)
		return;

	src_node = of_get_child_by_name(dev->of_node, "rcar_sound,src");
	if (!src_node)
		return;

	nr = of_get_child_count(src_node);
	if (!nr)
		goto rsnd_of_parse_src_end;

	src_info = devm_kzalloc(dev,
				sizeof(struct rsnd_src_platform_info) * nr,
				GFP_KERNEL);
	if (!src_info) {
		dev_err(dev, "src info allocation error\n");
		goto rsnd_of_parse_src_end;
	}

	info->src_info		= src_info;
	info->src_info_nr	= nr;

	i = -1;
	for_each_child_of_node(src_node, np) {
		i++;

		src_info = info->src_info + i;
		of_property_read_u32(np, "unused", &src_info->unused);
		if (src_info->unused)
			continue;

		src_info->irq_id = irq_of_parse_and_map(np, 0);
		of_property_read_u32(np, "convert-rate",
					&src_info->convert_rate);
	}

rsnd_of_parse_src_end:
	of_node_put(src_node);
}

int rsnd_src_probe(struct platform_device *pdev,
		   const struct rsnd_of_data *of_data,
		   struct rsnd_priv *priv)
{
	struct rcar_snd_info *info = rsnd_priv_to_info(priv);
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_src *src;
	struct rsnd_mod_ops *ops;
	struct clk *clk;
	char name[RSND_SRC_NAME_SIZE];
	int i, nr, ret;

	ops = NULL;
	if (rsnd_is_gen1(priv))
		ops = &rsnd_src_gen1_ops;
	if (rsnd_is_gen2(priv))
		ops = &rsnd_src_gen2_ops;
	if (!ops) {
		dev_err(dev, "unknown Generation\n");
		return -EIO;
	}

	rsnd_of_parse_src(pdev, of_data, priv);

	/*
	 * init SRC
	 */
	nr	= info->src_info_nr;
	if (!nr)
		return 0;

	src	= devm_kzalloc(dev, sizeof(*src) * nr, GFP_KERNEL);
	if (!src) {
		dev_err(dev, "SRC allocate failed\n");
		return -ENOMEM;
	}

	priv->src_nr	= nr;
	priv->src	= src;

	for_each_rsnd_src(src, priv, i) {
		if (info->src_info[i].unused)
			continue;

		snprintf(name, RSND_SRC_NAME_SIZE, "%s.%d",
			 SRC_NAME, i);

		clk = devm_clk_get(dev, name);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		src->info = &info->src_info[i];

		ret = rsnd_mod_init(priv, &src->mod, ops, clk, RSND_MOD_SRC, i);
		if (ret)
			return ret;

		dev_dbg(dev, "SRC%d probed\n", i);
	}

	return 0;
}

void rsnd_src_remove(struct platform_device *pdev,
		     struct rsnd_priv *priv)
{
	struct rsnd_src *src;
	int i;

	for_each_rsnd_src(src, priv, i) {
		rsnd_mod_quit(&src->mod);
	}
}
