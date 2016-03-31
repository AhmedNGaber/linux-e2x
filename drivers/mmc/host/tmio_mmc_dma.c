/*
 * linux/drivers/mmc/tmio_mmc_dma.c
 *
 * Copyright (C) 2014-2016 Renesas Electronics Corporation
 * Copyright (C) 2010-2011 Guennadi Liakhovetski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * DMA function for TMIO MMC implementations
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/mfd/tmio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/tmio.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>

#include "tmio_mmc.h"

#define TMIO_MMC_MIN_DMA_LEN 8

#define DM_CM_DTRAN_MODE	0x820
#define DM_CM_DTRAN_CTRL	0x828
#define DM_CM_RST		0x830
#define DM_CM_INFO1		0x840
#define DM_CM_INFO1_MASK	0x848
#define DM_CM_INFO2		0x850
#define DM_CM_INFO2_MASK	0x858
#define DM_DTRAN_ADDR		0x880

/* DM_CM_DTRAN_MODE */
#define DTRAN_MODE_CH_NUM_CH0	0	/* "downstream" = for write commands */
#define DTRAN_MODE_CH_NUM_CH1	BIT(16)	/* "uptream" = for read commands */
#define DTRAN_MODE_BUS_WID_TH	(BIT(5) | BIT(4))
#define DTRAN_MODE_ADDR_MODE	BIT(0)	/* 1 = Increment address */

/* DM_CM_DTRAN_CTRL */
#define DTRAN_CTRL_DM_START	BIT(0)

/* DM_CM_RST */
#define RST_DTRANRST1		BIT(9)
#define RST_DTRANRST0		BIT(8)
#define RST_RESERVED_BITS	GENMASK_ULL(32, 0)

/* DM_CM_INFO1 and DM_CM_INFO1_MASK */
#define INFO1_DTRANEND1		BIT(17)
#define INFO1_DTRANEND0		BIT(16)

/* DM_CM_INFO2 and DM_CM_INFO2_MASK */
#define INFO2_DTRANERR1		BIT(17)
#define INFO2_DTRANERR0		BIT(16)

/*
 * Specification of this driver:
 * - host->chan_{rx,tx} will be used as a flag of enabling/disabling the dma
 * - Since this SDHI DMAC register set has actual 32-bit and "bus_shift" is 2,
 *   this driver cannot use original sd_ctrl_{write,read}32 functions.
 */

static void tmio_dm_write(struct tmio_mmc_host *host, int addr, u32 val)
{
	writel(val, host->ctl + addr);
}

void tmio_mmc_enable_dma(struct tmio_mmc_host *host, bool enable)
{
	if (host->use_internal_dma) {
		if (!host->chan_tx || !host->chan_rx)
			return;

		if ((host->dma) && (host->dma->enable))
			host->dma->enable(host, enable);
	} else {
		if (!host->chan_tx || !host->chan_rx)
			return;

		if (host->dma->enable)
			host->dma->enable(host, enable);

#if defined(CONFIG_SUPERH) || defined(CONFIG_ARCH_SHMOBILE)
		/* Switch DMA mode on or off - SuperH specific? */
		sd_ctrl_write16(host, CTL_DMA_ENABLE, enable ? 2 : 0);
#endif
	}
}

void tmio_mmc_abort_dma(struct tmio_mmc_host *host)
{
	if (host->use_internal_dma) {
		u32 val = RST_DTRANRST1 | RST_DTRANRST0;

		dev_dbg(&host->pdev->dev, "%s\n", __func__);

		tmio_mmc_enable_dma(host, false);

		tmio_dm_write(host, DM_CM_RST, ~val);
		tmio_dm_write(host, DM_CM_RST, val);

		tmio_mmc_enable_dma(host, true);
	} else {
		tmio_mmc_enable_dma(host, false);

		if (host->chan_rx)
			dmaengine_terminate_all(host->chan_rx);
		if (host->chan_tx)
			dmaengine_terminate_all(host->chan_tx);

		tmio_mmc_enable_dma(host, true);
	}
}

static void tmio_mmc_rx_dma_complete(void *arg)
{
	struct tmio_mmc_host *host = arg;

	tmio_set_transtate(host, TMIO_TRANSTATE_DEND);
}

static void tmio_mmc_tx_dma_complete(void *arg)
{
	struct tmio_mmc_host *host = arg;

	tmio_set_transtate(host, TMIO_TRANSTATE_DEND);
}

static void tmio_mmc_start_dma_rx(struct tmio_mmc_host *host)
{
	struct scatterlist *sg = host->sg_ptr, *sg_tmp;
	struct dma_async_tx_descriptor *desc = NULL;
	struct dma_chan *chan = host->chan_rx;
	struct tmio_mmc_data *pdata = host->pdata;
	dma_cookie_t cookie;
	int ret, i;
	bool aligned = true, multiple = true;
	unsigned int align = (1 << pdata->dma->alignment_shift) - 1;

	if (host->force_pio) {
		ret = 0;
		goto pio;
	}

	for_each_sg(sg, sg_tmp, host->sg_len, i) {
		if (sg_tmp->offset & align)
			aligned = false;
		if (sg_tmp->length & align) {
			multiple = false;
			break;
		}
	}

	if ((!aligned && (host->sg_len > 1 || sg->length > PAGE_CACHE_SIZE ||
			  (align & PAGE_MASK))) || !multiple) {
		ret = -EINVAL;
		goto pio;
	}

	if (sg->length < TMIO_MMC_MIN_DMA_LEN) {
		host->force_pio = true;
		ret = 0;
		goto pio;
	}

	tmio_mmc_disable_mmc_irqs(host, TMIO_STAT_RXRDY);
	tmio_mmc_enable_dma(host, true);

	/* The only sg element can be unaligned, use our bounce buffer then */
	if (!aligned) {
		sg_init_one(&host->bounce_sg, host->bounce_buf, sg->length);
		host->sg_ptr = &host->bounce_sg;
		sg = host->sg_ptr;
	}

	ret = dma_map_sg(chan->device->dev, sg, host->sg_len, DMA_FROM_DEVICE);
	if (ret > 0)
		desc = dmaengine_prep_slave_sg(chan, sg, ret,
			DMA_DEV_TO_MEM, DMA_CTRL_ACK);

	if (desc) {
		desc->callback = tmio_mmc_rx_dma_complete;
		desc->callback_param = host;

		cookie = dmaengine_submit(desc);
		if (cookie < 0) {
			desc = NULL;
			ret = cookie;
		}
	}
pio:
	if (!desc) {
		/* DMA failed, fall back to PIO */
		tmio_mmc_enable_dma(host, false);
		if (ret >= 0)
			ret = -EIO;
		host->chan_rx = NULL;
		dma_release_channel(chan);
		/* Free the Tx channel too */
		chan = host->chan_tx;
		if (chan) {
			host->chan_tx = NULL;
			dma_release_channel(chan);
		}
		dev_dbg(&host->pdev->dev,
			 "DMA failed: %d, falling back to PIO\n", ret);
	}
}

static void tmio_mmc_start_dma_tx(struct tmio_mmc_host *host)
{
	struct scatterlist *sg = host->sg_ptr, *sg_tmp;
	struct dma_async_tx_descriptor *desc = NULL;
	struct dma_chan *chan = host->chan_tx;
	struct tmio_mmc_data *pdata = host->pdata;
	dma_cookie_t cookie;
	int ret, i;
	bool aligned = true, multiple = true;
	unsigned int align = (1 << pdata->dma->alignment_shift) - 1;

	if (host->force_pio) {
		ret = 0;
		goto pio;
	}

	for_each_sg(sg, sg_tmp, host->sg_len, i) {
		if (sg_tmp->offset & align)
			aligned = false;
		if (sg_tmp->length & align) {
			multiple = false;
			break;
		}
	}

	if ((!aligned && (host->sg_len > 1 || sg->length > PAGE_CACHE_SIZE ||
			  (align & PAGE_MASK))) || !multiple) {
		ret = -EINVAL;
		goto pio;
	}

	if (sg->length < TMIO_MMC_MIN_DMA_LEN) {
		host->force_pio = true;
		ret = 0;
			goto pio;
	}

	tmio_mmc_disable_mmc_irqs(host, TMIO_STAT_TXRQ);
	tmio_mmc_enable_dma(host, true);

	/* The only sg element can be unaligned, use our bounce buffer then */
	if (!aligned) {
		unsigned long flags;
		void *sg_vaddr = tmio_mmc_kmap_atomic(sg, &flags);
		sg_init_one(&host->bounce_sg, host->bounce_buf, sg->length);
		memcpy(host->bounce_buf, sg_vaddr, host->bounce_sg.length);
		tmio_mmc_kunmap_atomic(sg, &flags, sg_vaddr);
		host->sg_ptr = &host->bounce_sg;
		sg = host->sg_ptr;
	}

	ret = dma_map_sg(chan->device->dev, sg, host->sg_len, DMA_TO_DEVICE);
	if (ret > 0)
		desc = dmaengine_prep_slave_sg(chan, sg, ret,
			DMA_MEM_TO_DEV, DMA_CTRL_ACK);

	if (desc) {
		desc->callback = tmio_mmc_tx_dma_complete;
		desc->callback_param = host;

		cookie = dmaengine_submit(desc);
		if (cookie < 0) {
			desc = NULL;
			ret = cookie;
		}
	}
pio:
	if (!desc) {
		/* DMA failed, fall back to PIO */
		tmio_mmc_enable_dma(host, false);
		if (ret >= 0)
			ret = -EIO;
		host->chan_tx = NULL;
		dma_release_channel(chan);
		/* Free the Rx channel too */
		chan = host->chan_rx;
		if (chan) {
			host->chan_rx = NULL;
			dma_release_channel(chan);
		}
		dev_dbg(&host->pdev->dev,
			 "DMA failed: %d, falling back to PIO\n", ret);
	}
}

void tmio_mmc_start_dma(struct tmio_mmc_host *host,
			       struct mmc_data *data)
{
	if (host->use_internal_dma) {
		struct scatterlist *sg = host->sg_ptr;
		u32 dtran_mode = DTRAN_MODE_BUS_WID_TH | DTRAN_MODE_ADDR_MODE;
		enum dma_data_direction dir;
		int ret;
		u32 irq_mask;

		/* This DMAC cannot handle if sg_len is not 1 */
		BUG_ON(host->sg_len > 1);

		dev_dbg(&host->pdev->dev, "%s: %d, %x\n",
			__func__, host->sg_len,
			data->flags);

		/* This DMAC cannot handle if buffer is not 8-bytes alignment */
		if (!IS_ALIGNED(sg->offset, 8)) {
			host->force_pio = true;
			tmio_mmc_enable_dma(host, false);
			return;
		}

		if (data->flags & MMC_DATA_READ) {
			dtran_mode |= DTRAN_MODE_CH_NUM_CH1;
			dir = DMA_FROM_DEVICE;
			irq_mask = TMIO_STAT_RXRDY;
		} else {
			dtran_mode |= DTRAN_MODE_CH_NUM_CH0;
			dir = DMA_TO_DEVICE;
			irq_mask = TMIO_STAT_TXRQ;
		}

		ret = dma_map_sg(&host->pdev->dev, sg, host->sg_len, dir);
		if (ret < 0) {
			dev_err(&host->pdev->dev,
				"%s: dma_map_sg failed\n", __func__);
			return;
		}

		tmio_mmc_enable_dma(host, true);

		/* disable PIO irqs to avoid "PIO IRQ in DMA mode!" */
		tmio_mmc_disable_mmc_irqs(host, irq_mask);

		/* set dma parameters */
		tmio_dm_write(host, DM_CM_DTRAN_MODE, dtran_mode);
		tmio_dm_write(host, DM_DTRAN_ADDR, sg->dma_address);
	} else {
		struct tmio_mmc_data *pdata = host->pdata;

		tmio_clear_transtate(host);

		if (pdata->dma && (!host->chan_tx || !host->chan_rx)) {
			struct resource *res = platform_get_resource(host->pdev,
						     IORESOURCE_MEM,
						     0);
			struct dma_slave_config cfg = {};
			dma_cap_mask_t mask;
			int ret;

			if (!res)
				return;

			dma_cap_zero(mask);
			dma_cap_set(DMA_SLAVE, mask);

			host->chan_tx = dma_request_slave_channel_compat(mask,
						pdata->dma->filter,
						pdata->dma->chan_priv_tx,
						&host->pdev->dev, "tx");
			dev_dbg(&host->pdev->dev,
				"%s: TX: got channel %p\n", __func__,
				host->chan_tx);

			if (!host->chan_tx)
				return;

			if (pdata->dma->chan_priv_tx)
				cfg.slave_id = pdata->dma->slave_id_tx;
			cfg.direction = DMA_MEM_TO_DEV;
			cfg.dst_addr = res->start +
			       (CTL_SD_DATA_PORT << host->pdata->bus_shift);
			cfg.src_addr = 0;
			ret = dmaengine_slave_config(host->chan_tx, &cfg);
			if (ret < 0)
				goto ecfgtx;

			host->chan_rx = dma_request_slave_channel_compat(mask,
						pdata->dma->filter,
						pdata->dma->chan_priv_rx,
						&host->pdev->dev, "rx");
			dev_dbg(&host->pdev->dev,
				"%s: RX: got channel %p\n", __func__,
				host->chan_rx);

			if (!host->chan_rx)
				goto ereqrx;

			if (pdata->dma->chan_priv_rx)
				cfg.slave_id = pdata->dma->slave_id_rx;
			cfg.direction = DMA_DEV_TO_MEM;
			cfg.src_addr = cfg.dst_addr + pdata->dma->dma_rx_offset;
			cfg.dst_addr = 0;
			ret = dmaengine_slave_config(host->chan_rx, &cfg);
			if (ret < 0)
				goto ecfgrx;
		}

		if (data->flags & MMC_DATA_READ) {
			if (host->chan_rx)
				tmio_mmc_start_dma_rx(host);
		} else {
			if (host->chan_tx)
				tmio_mmc_start_dma_tx(host);
		}
		return;

ecfgrx:
		dma_release_channel(host->chan_rx);
		host->chan_rx = NULL;
ereqrx:
ecfgtx:
		dma_release_channel(host->chan_tx);
		host->chan_tx = NULL;
	}
}

static void tmio_mmc_issue_tasklet_fn(unsigned long priv)
{
	struct tmio_mmc_host *host = (struct tmio_mmc_host *)priv;
	if (host->use_internal_dma) {
		dev_dbg(&host->pdev->dev, "%s\n", __func__);

		tmio_mmc_enable_mmc_irqs(host, TMIO_MASK_DMA);

		/* start the DMAC */
		tmio_dm_write(host, DM_CM_DTRAN_CTRL, DTRAN_CTRL_DM_START);
	} else {
		struct dma_chan *chan = NULL;

		spin_lock_irq(&host->lock);

		if (host && host->data) {
			if (host->data->flags & MMC_DATA_READ)
				chan = host->chan_rx;
			else
				chan = host->chan_tx;
		}

		spin_unlock_irq(&host->lock);

		tmio_mmc_enable_mmc_irqs(host, TMIO_MASK_DMA);

		if (chan)
			dma_async_issue_pending(chan);
	}
}

static void tmio_mmc_tasklet_fn(unsigned long arg)
{
	struct tmio_mmc_host *host = (struct tmio_mmc_host *)arg;
	if (host->use_internal_dma) {
		enum dma_data_direction dir;

		dev_dbg(&host->pdev->dev, "%s: %p\n", __func__, host->data);

		if (!host->data)
			return;

		if (host->data->flags & MMC_DATA_READ)
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_TO_DEVICE;

		dma_unmap_sg(&host->pdev->dev, host->sg_ptr, host->sg_len, dir);
		tmio_mmc_enable_dma(host, false);
		tmio_mmc_do_data_irq(host);
	} else {

		spin_lock_irq(&host->lock);

		if (!host->data)
			goto out;

		if (host->data->flags & MMC_DATA_READ)
			dma_unmap_sg(host->chan_rx->device->dev,
				     host->sg_ptr, host->sg_len,
				     DMA_FROM_DEVICE);
		else
			dma_unmap_sg(host->chan_tx->device->dev,
				     host->sg_ptr, host->sg_len,
				     DMA_TO_DEVICE);

		tmio_mmc_do_data_irq(host);
out:
		spin_unlock_irq(&host->lock);
	}
}

void tmio_mmc_request_dma(struct tmio_mmc_host *host, struct tmio_mmc_data *pdata)
{
	if (host->use_internal_dma) {
		/* Each value is set to non-zero to
			assume "enabling" each DMA */
		host->chan_rx = host->chan_tx = (void *)0xdeadbeaf;

		tasklet_init(&host->dma_complete, tmio_mmc_tasklet_fn,
			     (unsigned long)host);
		tasklet_init(&host->dma_issue, tmio_mmc_issue_tasklet_fn,
			     (unsigned long)host);
	} else {
		/* We can only either use DMA for both Tx
			and Rx or not use it at all */
		if (!pdata->dma || (!host->pdev->dev.of_node &&
			(!pdata->dma->chan_priv_tx ||
				!pdata->dma->chan_priv_rx)))
			return;

		if (!host->chan_tx && !host->chan_rx) {
			struct resource *res = platform_get_resource(host->pdev,
							IORESOURCE_MEM, 0);
			struct dma_slave_config cfg = {};
			dma_cap_mask_t mask;
			int ret;

			if (!res)
				return;

			dma_cap_zero(mask);
			dma_cap_set(DMA_SLAVE, mask);

			host->chan_tx = dma_request_slave_channel_compat(mask,
						pdata->dma->filter,
						pdata->dma->chan_priv_tx,
						&host->pdev->dev, "tx");
			dev_dbg(&host->pdev->dev,
				"%s: TX: got channel %p\n", __func__,
				host->chan_tx);

			if (!host->chan_tx)
				return;

			if (pdata->dma->chan_priv_tx)
				cfg.slave_id = pdata->dma->slave_id_tx;
			cfg.direction = DMA_MEM_TO_DEV;
			cfg.dst_addr = res->start +
				(CTL_SD_DATA_PORT << host->pdata->bus_shift);
			cfg.src_addr = 0;
			ret = dmaengine_slave_config(host->chan_tx, &cfg);
			if (ret < 0)
				goto ecfgtx;

			host->chan_rx = dma_request_slave_channel_compat(mask,
						pdata->dma->filter,
						pdata->dma->chan_priv_rx,
						&host->pdev->dev, "rx");
			dev_dbg(&host->pdev->dev,
				"%s: RX: got channel %p\n", __func__,
				host->chan_rx);

			if (!host->chan_rx)
				goto ereqrx;

			if (pdata->dma->chan_priv_rx)
				cfg.slave_id = pdata->dma->slave_id_rx;
			cfg.direction = DMA_DEV_TO_MEM;
			cfg.src_addr = cfg.dst_addr + pdata->dma->dma_rx_offset;
			cfg.dst_addr = 0;
			ret = dmaengine_slave_config(host->chan_rx, &cfg);
			if (ret < 0)
				goto ecfgrx;

			host->bounce_buf =
				(u8 *)__get_free_page(GFP_KERNEL | GFP_DMA);
			if (!host->bounce_buf)
				goto ebouncebuf;

			tasklet_init(&host->dma_complete,
				tmio_mmc_tasklet_fn, (unsigned long)host);
			tasklet_init(&host->dma_issue,
				tmio_mmc_issue_tasklet_fn, (unsigned long)host);
		}

		tmio_mmc_enable_dma(host, true);
	}
	return;

ebouncebuf:
ecfgrx:
	dma_release_channel(host->chan_rx);
	host->chan_rx = NULL;
ereqrx:
ecfgtx:
	dma_release_channel(host->chan_tx);
	host->chan_tx = NULL;
}

void tmio_mmc_release_dma(struct tmio_mmc_host *host)
{
	if (host->use_internal_dma) {
		/* Each value is set to zero to assume "disabling" each DMA */
		host->chan_rx = host->chan_tx = NULL;
	} else {
		if (host->chan_tx) {
			struct dma_chan *chan = host->chan_tx;
			host->chan_tx = NULL;
			dma_release_channel(chan);
		}
		if (host->chan_rx) {
			struct dma_chan *chan = host->chan_rx;
			host->chan_rx = NULL;
			dma_release_channel(chan);
		}
		if (host->bounce_buf) {
			free_pages((unsigned long)host->bounce_buf, 0);
			host->bounce_buf = NULL;
		}
		tasklet_kill(&host->dma_complete);
		tasklet_kill(&host->dma_issue);
	}
}
