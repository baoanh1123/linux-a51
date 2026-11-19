// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos SPEEDY I2C controller.
 *
 * SPEEDY is normally programmed through the Exynos IP batcher.  Keep the
 * batcher polling path here: direct writes to the SPEEDY FIFO are unsafe
 * while the APB semaphore is owned by the batcher.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SPEEDY_CTRL			0x000
#define SPEEDY_FIFO_CTRL		0x004
#define SPEEDY_CMD			0x008
#define SPEEDY_INT_ENABLE		0x00c
#define SPEEDY_INT_STATUS		0x010
#define SPEEDY_FIFO_STATUS		0x030
#define SPEEDY_TX_DATA			0x034
#define SPEEDY_RX_DATA			0x038

#define SPEEDY_ENABLE			BIT(0)
#define SPEEDY_REMOTE_RESET		BIT(30)
#define SPEEDY_SW_RST			BIT(31)
#define SPEEDY_FIFO_RESET		BIT(31)
#define SPEEDY_TRANSFER_DONE		BIT(0)
#define SPEEDY_TIMEOUT_MASK		GENMASK(3, 1)
#define SPEEDY_FIFO_TX_ALMOST_EMPTY_EN	BIT(4)
#define SPEEDY_RX_ERROR_MASK		GENMASK(18, 16)
#define SPEEDY_TX_ERROR_MASK		GENMASK(21, 20)
#define SPEEDY_FIFO_EMPTY		BIT(6)
#define SPEEDY_FIFO_RX_FULL_EN		BIT(8)
#define SPEEDY_RX_FIFO_INT_TRAILER_EN	BIT(9)
#define SPEEDY_TRANSFER_DONE_EN	BIT(0)
#define SPEEDY_TIMEOUT_EN		GENMASK(3, 1)
#define SPEEDY_RX_ERROR_EN		GENMASK(18, 16)
#define SPEEDY_TX_ERROR_EN		GENMASK(21, 20)
#define SPEEDY_ACCESS_RANDOM		BIT(19)
#define SPEEDY_DIRECTION_WRITE		BIT(20)
#define SPEEDY_BURST_INCR		BIT(5)
#define SPEEDY_ADDRESS_MASK		GENMASK(18, 7)
#define SPEEDY_BURST_LENGTH_MASK	GENMASK(4, 0)

#define IPBATCHER_CON			0x0500
#define IPBATCHER_STATE			0x0504
#define IPBATCHER_INT_EN		0x0508
#define IPBATCHER_FSM_UNEXPEN		0x050c
#define IPBATCHER_FSM_TXEN		0x0510
#define IPBATCHER_FSM_RXFIFO		0x0514
#define IPBATCHER_FSM_CON		0x0518
#define IP_FIFO_STATUS			0x051c
#define IP_INT_STATUS			0x0520
#define BATCHER_OPCODE			0x0600
#define BATCHER_START_PAYLOAD		0x1000
#define BATCHER_END_PAYLOAD		0x1060
#define IPBATCHER_SEMA_REL		0x0200

#define BATCHER_ENABLE			BIT(0)
#define DEDICATED_BATCHER_APB		BIT(1)
#define START_BATCHER			BIT(4)
#define IP_SW_RST			BIT(6)
#define MP_APBSEMA_SW_RST		BIT(7)
#define SW_RESET			BIT(31)
#define BATCHER_OPERATION_COMPLETE	BIT(0)
#define UNEXPECTED_IP_INTR		BIT(1)
#define BATCHER_FSM_STATE_IDLE		BIT(3)
#define BATCHER_FSM_STATE_INIT		BIT(4)
#define BATCHER_FSM_STATE_WAIT_INT	BIT(7)
#define MP_APBSEMA_CH_LOCK_STATUS	BIT(20)
#define BATCHER_INTERRUPT_ENABLE	BIT(0)
#define DISABLE_STOP_CMD		BIT(0)

#define BATCHER_INIT_CMD		0xffffffff
#define SPEEDY_CTRL_DEFAULT		0x30050
#define SPEEDY_FIFO_SIZE		16
#define SPEEDY_TIMEOUT_US		500000

struct speedy {
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	u32 cmd_buffer;
	unsigned int cmd_index;
	unsigned int cmd_pointer;
	unsigned int desc_pointer;
	unsigned int read_addr;
};

static void speedy_batcher_idle(struct speedy *speedy)
{
	writel(0, speedy->regs + IPBATCHER_CON);
}

static int speedy_batcher_wait(struct speedy *speedy)
{
	u32 state, status;
	int ret;

	ret = readl_poll_timeout_atomic(speedy->regs + IPBATCHER_STATE, state,
					state & (BATCHER_OPERATION_COMPLETE |
						 UNEXPECTED_IP_INTR),
					10, SPEEDY_TIMEOUT_US);
	if (ret)
		return ret;

	status = readl(speedy->regs + IP_INT_STATUS);
	if (state & UNEXPECTED_IP_INTR ||
	    status & (SPEEDY_TIMEOUT_MASK | SPEEDY_RX_ERROR_MASK |
		      SPEEDY_TX_ERROR_MASK))
		ret = -EIO;
	else
		ret = 0;

	if (state & BATCHER_OPERATION_COMPLETE)
		writel(BATCHER_OPERATION_COMPLETE, speedy->regs + IPBATCHER_STATE);
	return ret;
}

static int speedy_batcher_reset(struct speedy *speedy)
{
	u32 con, state;
	int ret;

	speedy_batcher_idle(speedy);
	con = readl(speedy->regs + IPBATCHER_CON);
	writel(con | SW_RESET, speedy->regs + IPBATCHER_CON);
	writel(con & ~SW_RESET, speedy->regs + IPBATCHER_CON);
	writel(con | MP_APBSEMA_SW_RST, speedy->regs + IPBATCHER_CON);
	writel(con & ~MP_APBSEMA_SW_RST, speedy->regs + IPBATCHER_CON);

	writel(SPEEDY_TIMEOUT_EN | SPEEDY_RX_ERROR_EN | SPEEDY_TX_ERROR_EN,
	       speedy->regs + IPBATCHER_FSM_UNEXPEN);
	writel(SPEEDY_TRANSFER_DONE_EN | SPEEDY_FIFO_TX_ALMOST_EMPTY_EN,
	       speedy->regs + IPBATCHER_FSM_TXEN);
	writel(SPEEDY_FIFO_EMPTY, speedy->regs + IPBATCHER_FSM_RXFIFO);
	writel(DISABLE_STOP_CMD, speedy->regs + IPBATCHER_FSM_CON);

	ret = readl_poll_timeout_atomic(speedy->regs + IPBATCHER_STATE, state,
					state & (BATCHER_FSM_STATE_IDLE |
						 BATCHER_FSM_STATE_INIT),
					10, SPEEDY_TIMEOUT_US);
	if (ret)
		return ret;
	writel(1, speedy->regs + IPBATCHER_SEMA_REL);
	return 0;
}

static int speedy_batcher_add(struct speedy *speedy, u32 payload, u8 opcode)
{
	u32 address;

	if (speedy->desc_pointer * 4 > BATCHER_END_PAYLOAD -
				      BATCHER_START_PAYLOAD)
		return -E2BIG;

	if (!speedy->cmd_index)
		speedy->cmd_buffer = BATCHER_INIT_CMD;
	speedy->cmd_buffer &= ~(0xff << (8 * speedy->cmd_index));
	speedy->cmd_buffer |= (u32)opcode << (8 * speedy->cmd_index);
	writel(speedy->cmd_buffer, speedy->regs + BATCHER_OPCODE +
	       speedy->cmd_pointer * 4);
	address = BATCHER_START_PAYLOAD + speedy->desc_pointer * 4;
	writel(payload, speedy->regs + address);
	speedy->desc_pointer++;
	if (++speedy->cmd_index == 4) {
		speedy->cmd_index = 0;
		speedy->cmd_pointer++;
	}
	return 0;
}

static int speedy_batcher_finish(struct speedy *speedy)
{
	int ret;

	ret = speedy_batcher_add(speedy, 0, 0xff);
	if (ret)
		return ret;
	writel(BATCHER_ENABLE | DEDICATED_BATCHER_APB,
	       speedy->regs + IPBATCHER_CON);
	writel(START_BATCHER, speedy->regs + IPBATCHER_CON);
	ret = speedy_batcher_wait(speedy);
	speedy_batcher_idle(speedy);
	return ret;
}

static int speedy_xfer_msg(struct speedy *speedy, struct i2c_msg *msg)
{
	u32 cmd, fifo, int_en;
	unsigned int i;
	int ret;

	if (!msg->len || msg->len > SPEEDY_FIFO_SIZE)
		return -EINVAL;
	speedy->cmd_buffer = BATCHER_INIT_CMD;
	speedy->cmd_index = 0;
	speedy->cmd_pointer = 0;
	speedy->desc_pointer = 0;

	ret = speedy_batcher_add(speedy, SPEEDY_CTRL_DEFAULT | SPEEDY_ENABLE,
				 SPEEDY_CTRL);
	if (ret)
		return ret;
	fifo = FIELD_PREP(GENMASK(4, 0), msg->len) |
	       FIELD_PREP(GENMASK(12, 8), 1);
	cmd = FIELD_PREP(SPEEDY_ADDRESS_MASK, msg->addr);
	if (msg->len == 1)
		cmd |= SPEEDY_ACCESS_RANDOM;
	else
		cmd |= SPEEDY_BURST_INCR |
		       FIELD_PREP(SPEEDY_BURST_LENGTH_MASK, msg->len - 1);

	ret = speedy_batcher_add(speedy, SPEEDY_FIFO_RESET | fifo,
				 SPEEDY_FIFO_CTRL);
	if (ret)
		return ret;
	if (msg->flags & I2C_M_RD) {
		int_en = SPEEDY_TIMEOUT_EN | SPEEDY_RX_ERROR_EN |
			 SPEEDY_FIFO_RX_FULL_EN | SPEEDY_RX_FIFO_INT_TRAILER_EN;
		speedy->read_addr = BATCHER_START_PAYLOAD + speedy->desc_pointer * 4;
		for (i = 0; i < msg->len; i++) {
			ret = speedy_batcher_add(speedy, 0x77, SPEEDY_RX_DATA);
			if (ret)
				return ret;
		}
	} else {
		int_en = SPEEDY_TRANSFER_DONE_EN | SPEEDY_FIFO_TX_ALMOST_EMPTY_EN |
			 SPEEDY_TIMEOUT_EN | SPEEDY_TX_ERROR_EN;
		cmd |= SPEEDY_DIRECTION_WRITE;
		for (i = 0; i < msg->len; i++) {
			ret = speedy_batcher_add(speedy, msg->buf[i], SPEEDY_TX_DATA);
			if (ret)
				return ret;
		}
	}
	ret = speedy_batcher_add(speedy, 0xffffffff, SPEEDY_INT_STATUS);
	if (ret)
		return ret;
	ret = speedy_batcher_add(speedy, int_en, SPEEDY_INT_ENABLE);
	if (ret)
		return ret;
	ret = speedy_batcher_add(speedy, cmd, SPEEDY_CMD);
	if (ret)
		return ret;
	ret = speedy_batcher_finish(speedy);
	if (ret)
		goto recover;

	if (msg->flags & I2C_M_RD)
		for (i = 0; i < msg->len; i++)
			msg->buf[i] = readl(speedy->regs + speedy->read_addr + i * 4);
	return 0;

recover:
	if (speedy_batcher_reset(speedy))
		return -ETIMEDOUT;
	return ret;
}

static int speedy_master_xfer(struct i2c_adapter *adap,
			      struct i2c_msg *msgs, int num)
{
	struct speedy *speedy = i2c_get_adapdata(adap);
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = speedy_xfer_msg(speedy, &msgs[i]);
		if (ret)
			return ret;
	}
	return num;
}

static u32 speedy_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm speedy_algorithm = {
	.master_xfer = speedy_master_xfer,
	.functionality = speedy_func,
};

static int speedy_probe(struct platform_device *pdev)
{
	struct speedy *speedy;
	int ret;

	speedy = devm_kzalloc(&pdev->dev, sizeof(*speedy), GFP_KERNEL);
	if (!speedy)
		return -ENOMEM;
	speedy->dev = &pdev->dev;
	speedy->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(speedy->regs))
		return PTR_ERR(speedy->regs);
	speedy->clk = devm_clk_get_optional(&pdev->dev, "speedy");
	if (IS_ERR(speedy->clk))
		return PTR_ERR(speedy->clk);
	ret = clk_prepare_enable(speedy->clk);
	if (ret)
		return ret;
	ret = speedy_batcher_reset(speedy);
	if (ret)
		goto disable_clk;
	speedy->adap.owner = THIS_MODULE;
	speedy->adap.algo = &speedy_algorithm;
	speedy->adap.dev.parent = &pdev->dev;
	speedy->adap.dev.of_node = pdev->dev.of_node;
	strscpy(speedy->adap.name, "Exynos SPEEDY", sizeof(speedy->adap.name));
	i2c_set_adapdata(&speedy->adap, speedy);
	platform_set_drvdata(pdev, speedy);
	ret = devm_i2c_add_adapter(&pdev->dev, &speedy->adap);
	if (ret)
		goto disable_clk;
	return 0;

disable_clk:
	clk_disable_unprepare(speedy->clk);
	return ret;
}

static void speedy_remove(struct platform_device *pdev)
{
	struct speedy *speedy = platform_get_drvdata(pdev);

	speedy_batcher_reset(speedy);
	clk_disable_unprepare(speedy->clk);
}

static const struct of_device_id speedy_of_match[] = {
	{ .compatible = "samsung,exynos-speedy" },
	{}
};
MODULE_DEVICE_TABLE(of, speedy_of_match);

static struct platform_driver speedy_driver = {
	.probe = speedy_probe,
	.remove = speedy_remove,
	.driver = {
		.name = "exynos-speedy",
		.of_match_table = speedy_of_match,
	},
};
module_platform_driver(speedy_driver);

MODULE_DESCRIPTION("Samsung Exynos SPEEDY I2C controller");
MODULE_LICENSE("GPL");
