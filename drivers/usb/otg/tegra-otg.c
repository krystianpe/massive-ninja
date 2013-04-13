/*
 * drivers/usb/otg/tegra-otg.c
 *
 * OTG transceiver driver for Tegra UTMI phy
 *
 * Copyright (C) 2010 NVIDIA Corp.
 * Copyright (C) 2010 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>

#define USB_PHY_WAKEUP		0x408
#define  USB_ID_INT_EN		(1 << 0)
#define  USB_ID_INT_STATUS	(1 << 1)
#define  USB_ID_STATUS		(1 << 2)
#define  USB_ID_PIN_WAKEUP_EN	(1 << 6)
#define  USB_VBUS_WAKEUP_EN	(1 << 30)
#define  USB_VBUS_INT_EN	(1 << 8)
#define  USB_VBUS_INT_STATUS	(1 << 9)
#define  USB_VBUS_STATUS	(1 << 10)
#define  USB_INTS		(USB_VBUS_INT_STATUS | USB_ID_INT_STATUS)

struct tegra_otg_data {
	struct otg_transceiver otg;
	unsigned long int_status;
	spinlock_t lock;
	struct mutex irq_work_mutex;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct platform_device *pdev;
	struct work_struct work;
	unsigned int intr_reg_data;
	bool detect_vbus;
	bool clk_enabled;
<<<<<<< HEAD
=======
	bool interrupt_mode;
	bool builtin_host;
	bool suspended;
>>>>>>> 04f2966... new changes
};
static struct tegra_otg_data *tegra_clone;

static inline unsigned long otg_readl(struct tegra_otg_data *tegra,
				      unsigned int offset)
{
	return readl(tegra->regs + offset);
}

static inline void otg_writel(struct tegra_otg_data *tegra, unsigned long val,
			      unsigned int offset)
{
	writel(val, tegra->regs + offset);
}

static void tegra_otg_enable_clk(void)
{
	if (!tegra_clone->clk_enabled)
		clk_enable(tegra_clone->clk);
	tegra_clone->clk_enabled = true;
}

static void tegra_otg_disable_clk(void)
{
<<<<<<< HEAD
	if (tegra_clone->clk_enabled)
		clk_disable(tegra_clone->clk);
	tegra_clone->clk_enabled = false;
}
=======
	unsigned long val;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	if (en) {
		if (tegra->builtin_host)
			val |= USB_INT_EN;
		else
			val |= USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN | USB_ID_PIN_WAKEUP_EN;
	}
	else
		val &= ~USB_INT_EN;
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	/* Add delay to make sure register is updated */
	udelay(1);
	clk_disable(tegra->clk);
>>>>>>> df27fc1... usb w_i_p

static const char *tegra_state_name(enum usb_otg_state state)
{
	if (state == OTG_STATE_A_HOST)
		return "HOST";
	if (state == OTG_STATE_B_PERIPHERAL)
		return "PERIPHERAL";
	if (state == OTG_STATE_A_SUSPEND)
		return "SUSPEND";
	return "INVALID";
}

static struct platform_device *
tegra_usb_otg_host_register(struct platform_device *ehci_device,
			    struct tegra_ehci_platform_data *pdata)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(ehci_device->name, ehci_device->id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, ehci_device->resource,
					    ehci_device->num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  ehci_device->dev.dma_mask;
	pdev->dev.coherent_dma_mask = ehci_device->dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
		GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, pdata, sizeof(struct tegra_ehci_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host controller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
<<<<<<< HEAD
	kfree(pdev->dev.platform_data);
	pdev->dev.platform_data = NULL;
	platform_device_unregister(pdev);
}

void tegra_start_host(struct tegra_otg_data *tegra)
=======
	struct platform_device *pdev = tegra->pdev;

	DBG("%s(%d) Begin\n", __func__, __LINE__);

	if (pdev) {
		/* unregister host from otg */
		platform_device_unregister(pdev);
		tegra->pdev = NULL;
	}

	DBG("%s(%d) End\n", __func__, __LINE__);
}

static void tegra_otg_notify_event(struct otg_transceiver *otg,
					enum usb_xceiv_events event)
{
	otg->last_event = event;
	atomic_notifier_call_chain(&otg->notifier, event, NULL);
}

static void tegra_change_otg_state(struct tegra_otg_data *tegra,
				enum usb_otg_state to)
>>>>>>> df27fc1... usb w_i_p
{
	struct tegra_otg_platform_data *pdata = tegra->otg.dev->platform_data;
	if (!tegra->pdev) {
		tegra->pdev = tegra_usb_otg_host_register(pdata->ehci_device,
							  pdata->ehci_pdata);
	}
}

<<<<<<< HEAD
void tegra_stop_host(struct tegra_otg_data *tegra)
{
	if (tegra->pdev) {
		tegra_usb_otg_host_unregister(tegra->pdev);
		tegra->pdev = NULL;
=======
	DBG("%s(%d) requested otg state %s-->%s\n", __func__,
		__LINE__, tegra_state_name(from), tegra_state_name(to));

	if (to != OTG_STATE_UNDEFINED && from != to) {
		otg->state = to;
		dev_info(tegra->otg.dev, "%s --> %s\n", tegra_state_name(from),
					      tegra_state_name(to));

		if (from == OTG_STATE_A_SUSPEND) {
			if (to == OTG_STATE_B_PERIPHERAL && otg->gadget) {
				usb_gadget_vbus_connect(otg->gadget);
				tegra_otg_notify_event(otg, USB_EVENT_VBUS);
			}
			else if (to == OTG_STATE_A_HOST) {
				tegra_start_host(tegra);
				tegra_otg_notify_event(otg, USB_EVENT_ID);
			}
		} else if (from == OTG_STATE_A_HOST) {
			if (to == OTG_STATE_A_SUSPEND) {
				tegra_stop_host(tegra);
				tegra_otg_notify_event(otg, USB_EVENT_NONE);
			}
		} else if (from == OTG_STATE_B_PERIPHERAL && otg->gadget) {
			if (to == OTG_STATE_A_SUSPEND) {
				usb_gadget_vbus_disconnect(otg->gadget);
				tegra_otg_notify_event(otg, USB_EVENT_NONE);
			}
		}
>>>>>>> df27fc1... usb w_i_p
	}
}

static void irq_work(struct work_struct *work)
{
	struct tegra_otg_data *tegra =
		container_of(work, struct tegra_otg_data, work);
	struct otg_transceiver *otg = &tegra->otg;
	enum usb_otg_state from;
	enum usb_otg_state to = OTG_STATE_UNDEFINED;
	unsigned long flags;
	unsigned long status;

<<<<<<< HEAD
	if (tegra->detect_vbus) {
		tegra->detect_vbus = false;
		tegra_otg_enable_clk();
		return;
	}

	clk_enable(tegra->clk);

	spin_lock_irqsave(&tegra->lock, flags);

	status = tegra->int_status;

	if (tegra->int_status & USB_ID_INT_STATUS) {
		if (status & USB_ID_STATUS) {
			if ((status & USB_VBUS_STATUS) && (from != OTG_STATE_A_HOST))
				to = OTG_STATE_B_PERIPHERAL;
			else
				to = OTG_STATE_A_SUSPEND;
		}
		else
			to = OTG_STATE_A_HOST;
	}
	if (from != OTG_STATE_A_HOST) {
		if (tegra->int_status & USB_VBUS_INT_STATUS) {
			if (status & USB_VBUS_STATUS)
				to = OTG_STATE_B_PERIPHERAL;
			else
				to = OTG_STATE_A_SUSPEND;
		}
=======
	mutex_lock(&tegra->irq_work_mutex);

	spin_lock_irqsave(&tegra->lock, flags);
	from = otg->state;
	status = tegra->int_status;

	/* Debug prints */
	DBG("%s(%d) status = 0x%lx\n", __func__, __LINE__, status);
	if ((status & USB_ID_INT_STATUS) &&
			(status & USB_VBUS_INT_STATUS))
		DBG("%s(%d) got vbus & id interrupt\n", __func__, __LINE__);
	else {
		if (status & USB_ID_INT_STATUS)
			DBG("%s(%d) got id interrupt\n", __func__, __LINE__);
		if (status & USB_VBUS_INT_STATUS)
			DBG("%s(%d) got vbus interrupt\n", __func__, __LINE__);
>>>>>>> df27fc1... usb w_i_p
	}
	spin_unlock_irqrestore(&tegra->lock, flags);

	if (to != OTG_STATE_UNDEFINED) {
		otg->state = to;

<<<<<<< HEAD
		dev_info(tegra->otg.dev, "%s --> %s\n", tegra_state_name(from),
					      tegra_state_name(to));

		if (to == OTG_STATE_A_SUSPEND) {
			if (from == OTG_STATE_A_HOST)
				tegra_stop_host(tegra);
			else if (from == OTG_STATE_B_PERIPHERAL && otg->gadget)
				usb_gadget_vbus_disconnect(otg->gadget);
		} else if (to == OTG_STATE_B_PERIPHERAL && otg->gadget) {
			if (from == OTG_STATE_A_SUSPEND)
				usb_gadget_vbus_connect(otg->gadget);
		} else if (to == OTG_STATE_A_HOST) {
			if (from == OTG_STATE_A_SUSPEND)
			tegra_start_host(tegra);
		}
	}
	clk_disable(tegra->clk);
	tegra_otg_disable_clk();
=======
	spin_unlock_irqrestore(&tegra->lock, flags);
	tegra_change_otg_state(tegra, to);
	mutex_unlock(&tegra->irq_work_mutex);
>>>>>>> df27fc1... usb w_i_p
}

static irqreturn_t tegra_otg_irq(int irq, void *data)
{
	struct tegra_otg_data *tegra = data;
	unsigned long flags;
	unsigned long val;

	spin_lock_irqsave(&tegra->lock, flags);
<<<<<<< HEAD
=======
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	DBG("%s(%d) interrupt val = 0x%lx\n", __func__, __LINE__, val);
>>>>>>> df27fc1... usb w_i_p

	val = otg_readl(tegra, USB_PHY_WAKEUP);
	if (val & (USB_VBUS_INT_EN | USB_ID_INT_EN)) {
<<<<<<< HEAD
=======
		DBG("%s(%d) PHY_WAKEUP = 0x%lx\n", __func__, __LINE__, val);
>>>>>>> df27fc1... usb w_i_p
		otg_writel(tegra, val, USB_PHY_WAKEUP);
		if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
			tegra->int_status = val;
			tegra->detect_vbus = false;
			schedule_work(&tegra->work);
		}
	}

	spin_unlock_irqrestore(&tegra->lock, flags);

	return IRQ_HANDLED;
}

void tegra_otg_check_vbus_detection(void)
{
	tegra_clone->detect_vbus = true;
	schedule_work(&tegra_clone->work);
}
EXPORT_SYMBOL(tegra_otg_check_vbus_detection);

static int tegra_otg_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget)
{
	struct tegra_otg_data *tegra;
	unsigned long val;

	tegra = container_of(otg, struct tegra_otg_data, otg);
	otg->gadget = gadget;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val |= (USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN);
	val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	/* Add delay to make sure register is updated */
	udelay(1);
	clk_disable(tegra->clk);

	if ((val & USB_ID_STATUS) && (val & USB_VBUS_STATUS)) {
		val |= USB_VBUS_INT_STATUS;
	} else if (!(val & USB_ID_STATUS)) {
		val |= USB_ID_INT_STATUS;
	} else {
		val &= ~(USB_ID_INT_STATUS | USB_VBUS_INT_STATUS);
	}

	if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
		tegra->int_status = val;
		tegra->detect_vbus = false;
		schedule_work (&tegra->work);
	}

	return 0;
}

static int tegra_otg_set_host(struct otg_transceiver *otg,
				struct usb_bus *host)
{
	struct tegra_otg_data *tegra;
	unsigned long val;

	tegra = container_of(otg, struct tegra_otg_data, otg);
	otg->host = host;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_VBUS_INT_STATUS | USB_ID_INT_STATUS);

	val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	clk_disable(tegra->clk);

	return 0;
}

static int tegra_otg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	return 0;
}

static int tegra_otg_set_suspend(struct otg_transceiver *otg, int suspend)
{
	return 0;
}

<<<<<<< HEAD
=======
static ssize_t show_host_en(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);

	*buf = tegra->interrupt_mode ? '0': '1';
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t store_host_en(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
	unsigned int host;

	if (sscanf(buf, "%d", &host) != 1 || host < 0 || host > 1)
		return -EINVAL;

	if (host) {
		enable_interrupt(tegra, false);
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		tegra_change_otg_state(tegra, OTG_STATE_A_HOST);
		tegra->interrupt_mode = false;
	} else {
		tegra->interrupt_mode = true;
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);
		enable_interrupt(tegra, true);
	}

	return count;
}

static DEVICE_ATTR(enable_host, 0644, show_host_en, store_host_en);

>>>>>>> 04f2966... new changes
static int tegra_otg_probe(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra;
	struct tegra_otg_platform_data *otg_pdata;
	struct tegra_ehci_platform_data *ehci_pdata;
	struct resource *res;
	int err;

	tegra = kzalloc(sizeof(struct tegra_otg_data), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->otg.dev = &pdev->dev;
	otg_pdata = tegra->otg.dev->platform_data;
	ehci_pdata = otg_pdata->ehci_pdata;
	tegra->otg.label = "tegra-otg";
	tegra->otg.state = OTG_STATE_UNDEFINED;
	tegra->otg.set_host = tegra_otg_set_host;
	tegra->otg.set_peripheral = tegra_otg_set_peripheral;
	tegra->otg.set_suspend = tegra_otg_set_suspend;
	tegra->otg.set_power = tegra_otg_set_power;
	spin_lock_init(&tegra->lock);
	mutex_init(&tegra->irq_work_mutex);

	platform_set_drvdata(pdev, tegra);
	tegra_clone = tegra;
	tegra->clk_enabled = false;

	tegra->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Can't get otg clock\n");
		err = PTR_ERR(tegra->clk);
		goto err_clk;
	}

	err = clk_enable(tegra->clk);
	if (err)
		goto err_clken;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto err_io;
	}
	tegra->regs = ioremap(res->start, resource_size(res));
	if (!tegra->regs) {
		err = -ENOMEM;
		goto err_io;
	}

	tegra->otg.state = OTG_STATE_A_SUSPEND;

	err = otg_set_transceiver(&tegra->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver (%d)\n", err);
		goto err_otg;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENXIO;
		goto err_irq;
	}
	tegra->irq = res->start;
	err = request_threaded_irq(tegra->irq, tegra_otg_irq,
				   NULL,
				   IRQF_SHARED, "tegra-otg", tegra);
	if (err) {
		dev_err(&pdev->dev, "Failed to register IRQ\n");
		goto err_irq;
	}
	INIT_WORK (&tegra->work, irq_work);

<<<<<<< HEAD
	if (!ehci_pdata->default_enable)
		clk_disable(tegra->clk);
=======
	if (pdata->ehci_pdata->u_data.host.remote_wakeup_supported) {
		err = enable_irq_wake(tegra->irq);
		if (err < 0) {
			dev_warn(&pdev->dev,
				"Couldn't enable USB otg mode wakeup,"
				" irq=%d, error=%d\n", tegra->irq, err);
			err = 0;
		}
	}

	INIT_WORK(&tegra->work, irq_work);

>>>>>>> df27fc1... usb w_i_p
	dev_info(&pdev->dev, "otg transceiver registered\n");
	return 0;

err_irq:
	otg_set_transceiver(NULL);
err_otg:
	iounmap(tegra->regs);
err_io:
	clk_disable(tegra->clk);
err_clken:
	clk_put(tegra->clk);
err_clk:
	platform_set_drvdata(pdev, NULL);
	kfree(tegra);
	return err;
}

static int __exit tegra_otg_remove(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);

	free_irq(tegra->irq, tegra);
	otg_set_transceiver(NULL);
	iounmap(tegra->regs);
	clk_disable(tegra->clk);
	clk_put(tegra->clk);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&tegra->irq_work_mutex);
	kfree(tegra);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_otg_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
<<<<<<< HEAD
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = &tegra_otg->otg;
	enum usb_otg_state from = otg->state;
	/* store the interupt enable for cable ID and VBUS */
	clk_enable(tegra_otg->clk);
	tegra_otg->intr_reg_data = readl(tegra_otg->regs + USB_PHY_WAKEUP);
	writel(0, (tegra_otg->regs + USB_PHY_WAKEUP));
	clk_disable(tegra_otg->clk);

	if (from == OTG_STATE_B_PERIPHERAL && otg->gadget) {
		usb_gadget_vbus_disconnect(otg->gadget);
		otg->state = OTG_STATE_A_SUSPEND;
	}
	tegra_otg_disable_clk();
=======
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = &tegra->otg;
	int val;

	mutex_lock(&tegra->irq_work_mutex);
	DBG("%s(%d) BEGIN state : %s\n", __func__, __LINE__,
					tegra_state_name(otg->state));

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_ID_INT_EN | USB_VBUS_INT_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	clk_disable(tegra->clk);

	/* Suspend peripheral mode, host mode is taken care by host driver */
	if (otg->state == OTG_STATE_B_PERIPHERAL)
		tegra_change_otg_state(tegra, OTG_STATE_A_SUSPEND);

	tegra->suspended = true;

	DBG("%s(%d) END\n", __func__, __LINE__);
	mutex_unlock(&tegra->irq_work_mutex);
>>>>>>> df27fc1... usb w_i_p
	return 0;
}

static void tegra_otg_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
<<<<<<< HEAD
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);
=======
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
>>>>>>> df27fc1... usb w_i_p
	int val;
	unsigned long flags;

<<<<<<< HEAD
	tegra_otg_enable_clk();

	/* Following delay is intentional.
	 * It is placed here after observing system hang.
	 * Root cause is not confirmed.
	 */
	msleep(1);
	/* restore the interupt enable for cable ID and VBUS */
	clk_enable(tegra_otg->clk);
	writel(tegra_otg->intr_reg_data, (tegra_otg->regs + USB_PHY_WAKEUP));
	val = readl(tegra_otg->regs + USB_PHY_WAKEUP);
	clk_disable(tegra_otg->clk);

	/* A device might be connected while CPU is in sleep mode. In this case no interrupt
	 * will be triggered
	 * force irq_work to recheck connected devices
	 */
	if (!(val & USB_ID_STATUS)) {
		spin_lock_irqsave(&tegra_otg->lock, flags);
		tegra_otg->int_status = (val | USB_ID_INT_STATUS );
		schedule_work(&tegra_otg->work);
		spin_unlock_irqrestore(&tegra_otg->lock, flags);
	}

	return;
=======
	mutex_lock(&tegra->irq_work_mutex);
	if (!tegra->suspended) {
		mutex_unlock(&tegra->irq_work_mutex);
		return;
	}

	/* Clear pending interrupts */
	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	DBG("%s(%d) PHY WAKEUP register : 0x%x\n", __func__, __LINE__, val);
	clk_disable(tegra->clk);

	/* Enable interrupt and call work to set to appropriate state */
	spin_lock_irqsave(&tegra->lock, flags);
	if (tegra->builtin_host)
		tegra->int_status = val | USB_INT_EN;
	else
		tegra->int_status = val | USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN |
			USB_ID_PIN_WAKEUP_EN;

	spin_unlock_irqrestore(&tegra->lock, flags);
	schedule_work(&tegra->work);
	enable_interrupt(tegra, true);

	tegra->suspended = false;

	DBG("%s(%d) END\n", __func__, __LINE__);
	mutex_unlock(&tegra->irq_work_mutex);
>>>>>>> df27fc1... usb w_i_p
}

static const struct dev_pm_ops tegra_otg_pm_ops = {
	.complete = tegra_otg_resume,
	.suspend = tegra_otg_suspend,
};
#endif

static struct platform_driver tegra_otg_driver = {
	.driver = {
		.name  = "tegra-otg",
#ifdef CONFIG_PM
		.pm    = &tegra_otg_pm_ops,
#endif
	},
	.remove  = __exit_p(tegra_otg_remove),
	.probe   = tegra_otg_probe,
};

static int __init tegra_otg_init(void)
{
	return platform_driver_register(&tegra_otg_driver);
}
subsys_initcall(tegra_otg_init);

static void __exit tegra_otg_exit(void)
{
	platform_driver_unregister(&tegra_otg_driver);
}
module_exit(tegra_otg_exit);
