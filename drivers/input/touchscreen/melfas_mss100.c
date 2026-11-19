// SPDX-License-Identifier: GPL-2.0-only
/*
 * Melfas MSS100 touchscreen controller (Samsung Galaxy A51)
 *
 * The MSS100 speaks Melfas' MIP protocol over a bare I2C byte stream: a read
 * is "send [page, offset] then receive N bytes", with no register addressing
 * at all, so i2c_master_send()/i2c_master_recv() are used instead of the SMBus
 * helpers.  Layout and offsets come from the vendor driver,
 * DOWNSTREAM_READONLY/drivers/input/touchscreen/melfas/mss100/
 * (melfas_mss100.c, melfas_mss100_reg.h).
 *
 * This is the read-only bring-up stage: it powers the panel rail, waits for
 * the controller and reads its identification.  It deliberately does not
 * request the interrupt or register an input device yet, so that "does the
 * chip answer at all" is answered separately from "is the event decoding
 * right".
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

/* MIP pages and INFO offsets (MIP_R0_*, MIP_R1_INFO_* in the vendor header) */
#define MSS100_R0_INFO			0x01
#define MSS100_INFO_PRODUCT_NAME	0x00
#define MSS100_INFO_VERSION_BOOT	0x20
#define MSS100_INFO_VERSION_CORE	0x22

#define MSS100_R0_EVENT			0x02
#define MSS100_EVENT_PACKET_INFO	0x10
#define MSS100_EVENT_PACKET_DATA	0x11

#define MSS100_EVENT_FORMAT_16BYTE	0x01
#define MSS100_EVENT_SIZE		16
#define MSS100_MAX_FINGERS		10
#define MSS100_ABS_MAX			4095
#define MSS100_PANEL_X			1080
#define MSS100_PANEL_Y			2400

/* packet[0]: bits 7:6 state, bits 5:2 finger id (1-based), bits 1:0 event id */
#define MSS100_STATE_RELEASE		3

#define MSS100_PRODUCT_NAME_LEN		16
#define MSS100_I2C_RETRY		3

/**
 * struct mss100 - controller state
 * @dev:	device pointer
 * @client:	I2C client
 * @vdd:	panel supply
 */
struct mss100 {
	struct device *dev;
	struct i2c_client *client;
	struct regulator *vdd;
	struct input_dev *input;
	bool slots[MSS100_MAX_FINGERS];
	struct delayed_work poll;
};

/**
 * mss100_read() - MIP read: send a page/offset pair, then receive
 * @ts: controller state
 * @page: MIP page (MSS100_R0_*)
 * @offset: offset within the page
 * @buf: receive buffer
 * @len: number of bytes to read
 *
 * Returns 0 on success or a negative error code.
 */
static int mss100_read(struct mss100 *ts, u8 page, u8 offset, u8 *buf, size_t len)
{
	u8 cmd[2] = { page, offset };
	int ret, i;

	for (i = 0; i < MSS100_I2C_RETRY; i++) {
		ret = i2c_master_send(ts->client, cmd, sizeof(cmd));
		if (ret == sizeof(cmd))
			break;
		dev_warn(ts->dev, "command %#x/%#x failed: %d\n", page,
			 offset, ret);
		usleep_range(1000, 2000);
	}
	if (ret != sizeof(cmd))
		return ret < 0 ? ret : -EIO;

	for (i = 0; i < MSS100_I2C_RETRY; i++) {
		ret = i2c_master_recv(ts->client, buf, len);
		if (ret == len)
			return 0;
		dev_warn(ts->dev, "read %#x/%#x failed: %d\n", page, offset,
			 ret);
		usleep_range(1000, 2000);
	}

	return ret < 0 ? ret : -EIO;
}

/**
 * mss100_irq() - threaded interrupt: read one event packet and report it
 * @irq: interrupt number
 * @dev_id: controller state
 *
 * The packet is a single info byte followed by event_size * fingers bytes of
 * events; the info byte's low 7 bits count events when bit 7 is clear, and
 * bit 7 marks an alert rather than touch data.
 */
static void mss100_poll(struct work_struct *work);

static irqreturn_t mss100_irq(int irq, void *dev_id)
{
	struct mss100 *ts = dev_id;

	mss100_poll(&ts->poll.work);
	return IRQ_HANDLED;
}

static void mss100_poll(struct work_struct *work)
{
	struct mss100 *ts = container_of(work, struct mss100, poll.work);
	u8 info, packet[MSS100_EVENT_SIZE * MSS100_MAX_FINGERS];
	unsigned int count, i;

	if (mss100_read(ts, MSS100_R0_EVENT, MSS100_EVENT_PACKET_INFO, &info, 1))
		return;

	if (info & 0x80)
		return;	/* alert packet, not touch data */

	count = info & 0x7f;
	if (!count || count > MSS100_MAX_FINGERS)
		return;

	if (mss100_read(ts, MSS100_R0_EVENT, MSS100_EVENT_PACKET_DATA, packet,
			count * MSS100_EVENT_SIZE))
		return;

	for (i = 0; i < count; i++) {
		const u8 *ev = &packet[i * MSS100_EVENT_SIZE];
		unsigned int id = ((ev[0] >> 2) & 0x0f) - 1;
		unsigned int state = ev[0] >> 6;
		unsigned int x = (ev[1] << 4) | ((ev[3] >> 4) & 0x0f);
		unsigned int y = (ev[2] << 4) | (ev[3] & 0x0f);

		if (id >= MSS100_MAX_FINGERS)
			continue;

		input_mt_slot(ts->input, id);

		if (state == MSS100_STATE_RELEASE) {
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						   false);
			ts->slots[id] = false;
			continue;
		}

		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
				 x * MSS100_PANEL_X / MSS100_ABS_MAX);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
				 y * MSS100_PANEL_Y / MSS100_ABS_MAX);
		input_report_abs(ts->input, ABS_MT_PRESSURE, ev[6] & 0x3f);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ev[4]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, ev[5]);
		ts->slots[id] = true;
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

/*
 * The GPIO banks do not expose an interrupt domain on this SoC, so the
 * controller is polled instead of using its attention line.  Reading the
 * packet info byte is cheap and 10 ms matches a 100 Hz touch scan.
 */
static void mss100_poll_work(struct work_struct *work)
{
	struct mss100 *ts = container_of(to_delayed_work(work),
					 struct mss100, poll);

	mss100_poll(&ts->poll.work);
	schedule_delayed_work(&ts->poll, msecs_to_jiffies(10));
}

static int mss100_probe(struct i2c_client *client)
{
	struct mss100 *ts;
	u8 name[MSS100_PRODUCT_NAME_LEN + 1] = { 0 };
	u8 boot[4] = { 0 }, core[4] = { 0 };
	int ret;

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->dev = &client->dev;
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(&client->dev, PTR_ERR(ts->vdd),
				     "Failed to get panel supply\n");

	ret = regulator_enable(ts->vdd);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to enable panel supply\n");

	/*
	 * The controller needs time after power-on before it answers; the
	 * vendor only reads identification 5 s after probe for this reason.
	 */
	msleep(100);

	ret = mss100_read(ts, MSS100_R0_INFO, MSS100_INFO_PRODUCT_NAME, name,
			  MSS100_PRODUCT_NAME_LEN);
	if (ret) {
		dev_err(&client->dev, "Controller did not answer: %d\n", ret);
		goto err_disable;
	}

	dev_info(&client->dev, "MSS100 product name: %s\n", name);

	if (!mss100_read(ts, MSS100_R0_INFO, MSS100_INFO_VERSION_BOOT, boot,
			 sizeof(boot)))
		dev_info(&client->dev, "boot version: %*ph\n",
			 (int)sizeof(boot), boot);

	if (!mss100_read(ts, MSS100_R0_INFO, MSS100_INFO_VERSION_CORE, core,
			 sizeof(core)))
		dev_info(&client->dev, "core version: %*ph\n",
			 (int)sizeof(core), core);

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input)
		goto err_disable;

	ts->input->name = "mss100";
	ts->input->id.bustype = BUS_I2C;
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0,
			     MSS100_PANEL_X - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0,
			     MSS100_PANEL_Y - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 0x3f, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 0xff, 0, 0);
	input_mt_init_slots(ts->input, MSS100_MAX_FINGERS,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	ret = input_register_device(ts->input);
	if (ret)
		goto err_disable;

	/*
	 * Prefer the attention interrupt now that the GPA0 bank exposes its
	 * EINT domain; fall back to polling if the line is not described.
	 */
	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						mss100_irq, IRQF_ONESHOT,
						"mss100", ts);
		if (ret) {
			dev_err(&client->dev, "Failed to request IRQ: %d\n", ret);
			goto err_disable;
		}
		dev_info(&client->dev, "Touch reporting enabled (IRQ %d)\n",
			 client->irq);
	} else {
		INIT_DELAYED_WORK(&ts->poll, mss100_poll_work);
		schedule_delayed_work(&ts->poll, msecs_to_jiffies(10));
		dev_info(&client->dev, "Touch reporting enabled (polled)\n");
	}

	return 0;

err_disable:
	regulator_disable(ts->vdd);
	return ret;
}

static const struct of_device_id mss100_of_match[] = {
	{ .compatible = "melfas,mss100" },
	{ }
};
MODULE_DEVICE_TABLE(of, mss100_of_match);

static const struct i2c_device_id mss100_i2c_id[] = {
	{ "mss100" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mss100_i2c_id);

static struct i2c_driver mss100_driver = {
	.driver = {
		.name = "mss100",
		.of_match_table = mss100_of_match,
	},
	.probe = mss100_probe,
	.id_table = mss100_i2c_id,
};
module_i2c_driver(mss100_driver);

MODULE_DESCRIPTION("Melfas MSS100 touchscreen controller");
MODULE_LICENSE("GPL");
