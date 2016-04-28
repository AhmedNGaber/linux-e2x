/*
 * Renesas R-Car Gen2 USB phy driver
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_data/usb-rcar-gen2-phy.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/usb/gadget.h>

struct rcar_gen2_usb_phy_priv {
	struct usb_phy phy;
	void __iomem *base;		/* USBHS  */
	void __iomem *usb2_base;	/* USB2.0 */
	struct clk *clk;		/* USBHS  */
	spinlock_t lock;
	int usecount;
	u32 ugctrl2;
	int irq;
	int gpio_vbus;
	struct delayed_work	work;
	struct workqueue_struct *work_queue;
	struct platform_device *pdev;
};

#define usb_phy_to_priv(p) container_of(p, struct rcar_gen2_usb_phy_priv, phy)

/* Low Power Status register */
#define USBHS_LPSTS_REG			0x02
#define USBHS_LPSTS_SUSPM		(1 << 14)

/* USB General control register */
#define USBHS_UGCTRL_REG		0x80
#define USBHS_UGCTRL_CONNECT		(1 << 2)
#define USBHS_UGCTRL_PLLRESET		(1 << 0)

/* USB General control register 2 */
#define USBHS_UGCTRL2_REG		0x84
#define USBHS_UGCTRL2_USB0		(3 << 4)
#define USBHS_UGCTRL2_USB0_PCI		(1 << 4)
#ifdef CONFIG_USB_ALEX
#define USBHS_UGCTRL2_USB0_HS		(2 << 4)
#else
#define USBHS_UGCTRL2_USB0_HS		(3 << 4)
#endif
#define USBHS_UGCTRL2_USB2_PCI		(0 << 31)
#define USBHS_UGCTRL2_USB2_SS		(1 << 31)
#define USBHS_UGCTRL2_USB0_HOST		USBHS_UGCTRL2_USB0_PCI
#define USBHS_UGCTRL2_MASK		0x00000031 /* bit[31:6] should be 0 */

/* USB General status register */
#define USBHS_UGSTS_REG			0x88
#define USBHS_UGSTS_LOCK		(1 << 8)

/* USB Control register */
#define USB2_USBCTR_REG			0x00c
#define USB2_USBCTR_DIRPD		(1 << 2)
#define USB2_USBCTR_PLL_RST		(1 << 1)

/* Overcurrent Detection Timer Setting register */
#define USB2_OC_TIMSET_REG		0x110
#define USB2_OC_TIMSET_INIT		0x000209ab

/* Suspend/Resume Timer Setting register */
#define USB2_SPD_RSM_TIMSET_REG		0x10c
#define USB2_SPD_RSM_TIMSET_INIT	0x014e029b

#define USB2_INT_ENABLE_REG		0x000
#define USB2_INT_ENABLE_USBH_INTB_EN	(1 << 2)
#define USB2_INT_ENABLE_USBH_INTA_EN	(1 << 1)
#define USB2_INT_ENABLE_INIT		(USB2_INT_ENABLE_USBH_INTB_EN | \
					 USB2_INT_ENABLE_USBH_INTA_EN)

#ifdef CONFIG_USB_ALEX
/* Enable USBHS internal phy */
static int __rcar_gen2_usbhs_phy_enable(void __iomem *base,
					void __iomem *usb2_base)
{
	u32 val;

	/* USBHS PHY power on */
	val = ioread32(base + USBHS_UGCTRL_REG);
	val &= ~USBHS_UGCTRL_PLLRESET;
	iowrite32(val, base + USBHS_UGCTRL_REG);

	val = readl(usb2_base + USB2_USBCTR_REG);
	val |= USB2_USBCTR_PLL_RST;
	writel(val, usb2_base + USB2_USBCTR_REG);
	val &= ~USB2_USBCTR_PLL_RST;
	writel(val, usb2_base + USB2_USBCTR_REG);

	/* Power on HSUSB PHY */
	val = readw(base + USBHS_LPSTS_REG);
	val |= USBHS_LPSTS_SUSPM;
	writew(val, base + USBHS_LPSTS_REG);

	return 0;
}

/* Disable USBHS internal phy */
static int __rcar_gen2_usbhs_phy_disable(void __iomem *base)
{
	u32 val;

	/* Power off HSUSB PHY */
	val = readw(base + USBHS_LPSTS_REG);
	val &= ~USBHS_LPSTS_SUSPM;
	writew(val, base + USBHS_LPSTS_REG);

	return 0;
}

/* Setup USB channels */
static void __rcar_gen2_usb_phy_init(struct rcar_gen2_usb_phy_priv *priv)
{
	void __iomem *usb2_base = priv->usb2_base;
	void __iomem *hsusb_base = priv->base;
	u32 val;

	/* Since ops->init() is called once, this driver enables both clocks */
	clk_prepare_enable(priv->clk);

	/* Initialize USB2 part */
	writel(USB2_INT_ENABLE_INIT, usb2_base + USB2_INT_ENABLE_REG);
	writel(USB2_SPD_RSM_TIMSET_INIT, usb2_base + USB2_SPD_RSM_TIMSET_REG);
	writel(USB2_OC_TIMSET_INIT, usb2_base + USB2_OC_TIMSET_REG);

	/* Initialize HSUSB part */
	val = readl(hsusb_base + USBHS_UGCTRL2_REG);
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
	val = (val & ~USBHS_UGCTRL2_USB0) |
	      USBHS_UGCTRL2_USB0_HS;
#else
	val = (val & ~USBHS_UGCTRL2_USB0) |
	      USBHS_UGCTRL2_USB0_HOST;
#endif
	writel(val & USBHS_UGCTRL2_MASK, hsusb_base + USBHS_UGCTRL2_REG);
}

/* Shutdown USB channels */
static void __rcar_gen2_usb_phy_shutdown(struct rcar_gen2_usb_phy_priv *priv)
{
	__rcar_gen2_usbhs_phy_disable(priv->base);
	writel(0, priv->usb2_base + USB2_INT_ENABLE_REG);
	clk_disable_unprepare(priv->clk);
}
#else
/* Enable USBHS internal phy */
static int __rcar_gen2_usbhs_phy_enable(void __iomem *base)
{
	u32 val;
	int i;

	/* USBHS PHY power on */
	val = ioread32(base + USBHS_UGCTRL_REG);
	val &= ~USBHS_UGCTRL_PLLRESET;
	iowrite32(val, base + USBHS_UGCTRL_REG);

	val = ioread16(base + USBHS_LPSTS_REG);
	val |= USBHS_LPSTS_SUSPM;
	iowrite16(val, base + USBHS_LPSTS_REG);

	for (i = 0; i < 20; i++) {
		val = ioread32(base + USBHS_UGSTS_REG);
		if ((val & USBHS_UGSTS_LOCK) == USBHS_UGSTS_LOCK) {
			val = ioread32(base + USBHS_UGCTRL_REG);
			val |= USBHS_UGCTRL_CONNECT;
			iowrite32(val, base + USBHS_UGCTRL_REG);
			return 0;
		}
		udelay(1);
	}

	/* Timed out waiting for the PLL lock */
	return -ETIMEDOUT;
}

/* Disable USBHS internal phy */
static int __rcar_gen2_usbhs_phy_disable(void __iomem *base)
{
	u32 val;

	/* USBHS PHY power off */
	val = ioread32(base + USBHS_UGCTRL_REG);
	val &= ~USBHS_UGCTRL_CONNECT;
	iowrite32(val, base + USBHS_UGCTRL_REG);

	val = ioread16(base + USBHS_LPSTS_REG);
	val &= ~USBHS_LPSTS_SUSPM;
	iowrite16(val, base + USBHS_LPSTS_REG);

	val = ioread32(base + USBHS_UGCTRL_REG);
	val |= USBHS_UGCTRL_PLLRESET;
	iowrite32(val, base + USBHS_UGCTRL_REG);
	return 0;
}

/* Setup USB channels */
static void __rcar_gen2_usb_phy_init(struct rcar_gen2_usb_phy_priv *priv)
{
	u32 val;

	clk_prepare_enable(priv->clk);

	/* Set USB channels in the USBHS UGCTRL2 register */
	val = ioread32(priv->base + USBHS_UGCTRL2_REG);
	val &= ~(USBHS_UGCTRL2_USB0_HS | USBHS_UGCTRL2_USB2_SS);
	val |= priv->ugctrl2;
	iowrite32(val, priv->base + USBHS_UGCTRL2_REG);
}

/* Shutdown USB channels */
static void __rcar_gen2_usb_phy_shutdown(struct rcar_gen2_usb_phy_priv *priv)
{
	__rcar_gen2_usbhs_phy_disable(priv->base);
	clk_disable_unprepare(priv->clk);
}
#endif

static int rcar_gen2_usb_phy_set_suspend(struct usb_phy *phy, int suspend)
{
	struct rcar_gen2_usb_phy_priv *priv = usb_phy_to_priv(phy);
	unsigned long flags;
	int retval;
	struct platform_device *pdev = priv->pdev;
	struct resource *res;
#ifdef CONFIG_USB_ALEX
	struct resource *usb2_res;
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

#ifdef CONFIG_USB_ALEX
	usb2_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->usb2_base = devm_ioremap_resource(&pdev->dev, usb2_res);
	if (IS_ERR(priv->usb2_base))
		return PTR_ERR(priv->usb2_base);
#endif

	spin_lock_irqsave(&priv->lock, flags);

#ifdef CONFIG_USB_ALEX
	retval = suspend ? __rcar_gen2_usbhs_phy_disable(priv->base) :
			   __rcar_gen2_usbhs_phy_enable(priv->base,
							priv->usb2_base);
#else
	retval = suspend ? __rcar_gen2_usbhs_phy_disable(priv->base) :
			   __rcar_gen2_usbhs_phy_enable(priv->base);
#endif

	devm_release_mem_region(&pdev->dev, res->start, resource_size(res));
	devm_iounmap(&pdev->dev, priv->base);
#ifdef CONFIG_USB_ALEX
	devm_release_mem_region(&pdev->dev, usb2_res->start,
				resource_size(usb2_res));
	devm_iounmap(&pdev->dev, priv->usb2_base);
#endif

	spin_unlock_irqrestore(&priv->lock, flags);

	return retval;
}

static int rcar_gen2_usb_phy_init(struct usb_phy *phy)
{
	struct rcar_gen2_usb_phy_priv *priv = usb_phy_to_priv(phy);
	unsigned long flags;

	/*
	 * Enable the clock and setup USB channels
	 * if it's the first user
	 */
	if (!priv->usecount++) {
		struct platform_device *pdev = priv->pdev;
		struct resource *res;
#ifdef CONFIG_USB_ALEX
		struct resource *usb2_res;
#endif

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		priv->base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->base))
			return PTR_ERR(priv->base);
#ifdef CONFIG_USB_ALEX
		usb2_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		priv->usb2_base = devm_ioremap_resource(&pdev->dev, usb2_res);
		if (IS_ERR(priv->usb2_base))
			return PTR_ERR(priv->usb2_base);
#endif

		spin_lock_irqsave(&priv->lock, flags);
		__rcar_gen2_usb_phy_init(priv);

		devm_release_mem_region(&pdev->dev, res->start,
							resource_size(res));
		devm_iounmap(&pdev->dev, priv->base);
#ifdef CONFIG_USB_ALEX
		devm_release_mem_region(&pdev->dev, usb2_res->start,
					resource_size(usb2_res));
		devm_iounmap(&pdev->dev, priv->usb2_base);
#endif
		spin_unlock_irqrestore(&priv->lock, flags);
	}
	return 0;
}

static void rcar_gen2_usb_phy_shutdown(struct usb_phy *phy)
{
	struct rcar_gen2_usb_phy_priv *priv = usb_phy_to_priv(phy);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	if (!priv->usecount) {
		dev_warn(phy->dev, "Trying to disable phy with 0 usecount\n");
		goto out;
	}

	/* Disable everything if it's the last user */
	if (!--priv->usecount) {
		struct platform_device *pdev = priv->pdev;
		struct resource *res;
#ifdef CONFIG_USB_ALEX
		struct resource *usb2_res;
#endif

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		priv->base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->base))
			dev_err(phy->dev, "ioremap failed\n");

#ifdef CONFIG_USB_ALEX
		usb2_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		priv->usb2_base = devm_ioremap_resource(&pdev->dev, usb2_res);
		if (IS_ERR(priv->usb2_base))
			dev_err(phy->dev, "ioremap failed\n");
#endif

		__rcar_gen2_usb_phy_shutdown(priv);

		devm_release_mem_region(&pdev->dev, res->start,
							resource_size(res));
		devm_iounmap(&pdev->dev, priv->base);
#ifdef CONFIG_USB_ALEX
		devm_release_mem_region(&pdev->dev, usb2_res->start,
					resource_size(usb2_res));
		devm_iounmap(&pdev->dev, priv->usb2_base);
#endif
	}
out:
	spin_unlock_irqrestore(&priv->lock, flags);
}

/* VBUS */
static void rcar_gen2_usbhs_gpio_vbus_work(struct work_struct *work)
{
	struct rcar_gen2_usb_phy_priv *priv =
		container_of(work, struct rcar_gen2_usb_phy_priv, work.work);
	int vbus;

	vbus = gpio_get_value(priv->gpio_vbus);

	if (vbus) {
		priv->phy.state = OTG_STATE_B_PERIPHERAL;
		priv->phy.last_event = USB_EVENT_VBUS;
		usb_gadget_vbus_connect(priv->phy.otg->gadget);
	} else {
		priv->phy.state = OTG_STATE_B_IDLE;
		priv->phy.last_event = USB_EVENT_NONE;
		usb_gadget_vbus_disconnect(priv->phy.otg->gadget);
	}

}

/* VBUS change IRQ handler */
static irqreturn_t rcar_gen2_usbhs_gpio_wakeup_isr(int irq, void *data)
{
	struct rcar_gen2_usb_phy_priv *priv = data;
	struct usb_otg *otg = priv->phy.otg;

	if (otg->gadget)
		queue_delayed_work(priv->work_queue,
			&priv->work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}


/* bind/unbind the peripheral controller */
static int rcar_gen2_usbhs_set_peripheral(struct usb_otg *otg,
					struct usb_gadget *gadget)
{
	struct rcar_gen2_usb_phy_priv *priv;
	struct platform_device *pdev;

	priv = container_of(otg->phy, struct rcar_gen2_usb_phy_priv, phy);
	pdev = priv->pdev;

	if (!gadget) {
		dev_dbg(&pdev->dev, "unregistering gadget '%s'\n",
			otg->gadget->name);

		usb_gadget_vbus_disconnect(otg->gadget);
		otg->phy->state = OTG_STATE_UNDEFINED;

		otg->gadget = NULL;
		return 0;
	}

	otg->gadget = gadget;
	dev_dbg(&pdev->dev, "registered gadget '%s'\n", gadget->name);

	/* initialize connection state */
	rcar_gen2_usbhs_gpio_wakeup_isr(priv->irq, priv);
	return 0;
}

static int rcar_gen2_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rcar_gen2_phy_platform_data *pdata;
	struct rcar_gen2_usb_phy_priv *priv;
	struct clk *clk;
#ifdef CONFIG_USB_ALEX
	char *clk_id;
#endif
	int retval;
	int gpio, irq;
	struct workqueue_struct *work_queue;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(dev, "No platform data\n");
		return -EINVAL;
	}

#ifdef CONFIG_USB_ALEX
	clk_id = (pdev->id == 0) ? "hsusb0" : "hsusb1";
	clk = devm_clk_get(&pdev->dev, clk_id);
#else
	clk = devm_clk_get(&pdev->dev, "usbhs");
#endif
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Can't get the clock\n");
		return PTR_ERR(clk);
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	priv->phy.otg = devm_kzalloc(dev, sizeof(struct usb_otg), GFP_KERNEL);
	if (!priv->phy.otg) {
		dev_err(dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&priv->lock);
	priv->clk = clk;
	priv->pdev = pdev;

#ifndef CONFIG_USB_ALEX
	priv->ugctrl2 = pdata->chan0_pci ?
			USBHS_UGCTRL2_USB0_PCI : USBHS_UGCTRL2_USB0_HS;
	priv->ugctrl2 |= pdata->chan2_pci ?
			USBHS_UGCTRL2_USB2_PCI : USBHS_UGCTRL2_USB2_SS;
#endif
	priv->phy.dev = dev;
	priv->phy.label = dev_name(dev);
	priv->phy.init = rcar_gen2_usb_phy_init;
	priv->phy.shutdown = rcar_gen2_usb_phy_shutdown;
	priv->phy.set_suspend = rcar_gen2_usb_phy_set_suspend;
	priv->phy.state = OTG_STATE_UNDEFINED;
	priv->phy.type = USB_PHY_TYPE_USB2;

	priv->phy.otg->phy = &priv->phy;
	priv->phy.otg->set_peripheral = rcar_gen2_usbhs_set_peripheral;

	/* VBUS irq */
	gpio = pdata->gpio_vbus;
	if (gpio_is_valid(gpio)) {
		retval = devm_gpio_request_one(dev, gpio, GPIOF_IN, pdev->name);
		if (retval < 0) {
			dev_warn(dev, "Unable to request GPIO %d: %d\n",
				 gpio, retval);
			return retval;
		}
		priv->gpio_vbus = gpio;

		irq = gpio_to_irq(priv->gpio_vbus);
		if (irq < 0) {
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				priv->gpio_vbus, irq);
			return irq;
		}
		priv->irq = irq;

		INIT_DELAYED_WORK(&priv->work, rcar_gen2_usbhs_gpio_vbus_work);
		work_queue = create_singlethread_workqueue(dev_name(&pdev->dev));
		if (!work_queue) {
			dev_err(dev, "failed to create workqueue\n");
			return -ENOMEM;
		}
		priv->work_queue = work_queue;

		retval = devm_request_irq(dev, irq,
					rcar_gen2_usbhs_gpio_wakeup_isr,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					pdev->name, priv);

		if (retval) {
			dev_err(dev, "can't request irq %i, err: %d\n",
				irq, retval);
			goto err_irq;
		}

		device_init_wakeup(&pdev->dev, pdata->wakeup);
	}

	retval = usb_add_phy_dev(&priv->phy);
	if (retval < 0) {
		dev_err(dev, "Failed to add USB phy\n");
		goto err_otg;
	}
	platform_set_drvdata(pdev, priv);

#ifdef CONFIG_USB_ALEX
	{
		retval = usb_phy_init(&priv->phy);

		if (!retval)
			retval = usb_phy_set_suspend(&priv->phy, 0);
	}
#endif

	return retval;

err_otg:
	if (gpio_is_valid(gpio))
		device_init_wakeup(&pdev->dev, 0);
err_irq:
	if (gpio_is_valid(gpio)) {
		cancel_delayed_work_sync(&priv->work);
		destroy_workqueue(priv->work_queue);
	}

	return retval;
}

static int rcar_gen2_usb_phy_remove(struct platform_device *pdev)
{
	struct rcar_gen2_usb_phy_priv *priv = platform_get_drvdata(pdev);
	struct rcar_gen2_phy_platform_data *pdata = dev_get_platdata(&pdev->dev);

	if (gpio_is_valid(pdata->gpio_vbus)) {
		device_init_wakeup(&pdev->dev, 0);
		cancel_delayed_work_sync(&priv->work);
		destroy_workqueue(priv->work_queue);
	}
	usb_remove_phy(&priv->phy);

	return 0;
}

#ifdef CONFIG_PM
static int phy_rcar_gen2_pm_suspend(struct device *dev)
{
	struct rcar_gen2_usb_phy_priv *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(priv->irq);

	return 0;
}

static int phy_rcar_gen2_pm_resume(struct device *dev)
{
	struct rcar_gen2_usb_phy_priv *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(priv->irq);

	return 0;
}

static const struct dev_pm_ops phy_rcar_gen2_dev_pm_ops = {
	.suspend	= phy_rcar_gen2_pm_suspend,
	.resume		= phy_rcar_gen2_pm_resume,
};
#endif

static struct platform_driver rcar_gen2_usb_phy_driver = {
	.driver = {
		.name = "usb_phy_rcar_gen2",
#ifdef CONFIG_PM
		.pm = &phy_rcar_gen2_dev_pm_ops,
#endif
	},
	.probe = rcar_gen2_usb_phy_probe,
	.remove = rcar_gen2_usb_phy_remove,
};

#ifdef CONFIG_USB_ALEX
static int __init phy_rcar_gen2_platform_init(void)
{
	return platform_driver_register(&rcar_gen2_usb_phy_driver);
}
subsys_initcall(phy_rcar_gen2_platform_init);

static void __exit phy_rcar_gen2_platform_cleanup(void)
{
	platform_driver_unregister(&rcar_gen2_usb_phy_driver);
}
module_exit(phy_rcar_gen2_platform_cleanup);
#else
module_platform_driver(rcar_gen2_usb_phy_driver);
#endif

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car Gen2 USB phy");
MODULE_AUTHOR("Valentine Barshak <valentine.barshak@cogentembedded.com>");
