/* SPDX-License-Identifier: GPL-2.0 */
#include "exynos9610-wlbt.h"

static ssize_t wlbt_debugfs_boot(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;

	wlbt_boot(w);

	return len;
}

/*
 * Ask the peer to halt and wait for MM_HALT_RSP.  Nothing here needs the
 * reply, but a round trip is the only way to prove the outbound management
 * path works before HIP4 depends on it.
 */
static ssize_t wlbt_debugfs_halt(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int before = w->n_mgmt;
	bool sent;
	int i;

	mutex_lock(&w->lock);
	sent = wlbt_mgmt_send(w, MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
			      WLBT_MM_HALT_REQ, 0);

	/* Watch the doorbell before draining anything: the log acknowledgements
	 * write the same register, so a drain here would hide the result.
	 */
	for (i = 0; i < 10; i++) {
		if (!(readl(w->mbox + MBOX_INTSR1) & BIT(WLBT_BIT_MGMT_OUT_WRITE)))
			break;
		msleep(100);
	}
	dev_info(w->dev, "halt requested (%s), peer %s the doorbell, ring read %u write %u\n",
		 sent ? "written" : "failed",
		 i < 10 ? "took" : "ignored",
		 readl(w->dram + w->mgmt_out.ridx),
		 readl(w->dram + w->mgmt_out.widx));
	wlbt_mgmt_drain(w);
	dev_info(w->dev, "%u management message(s) in reply\n", w->n_mgmt - before);
	mutex_unlock(&w->lock);

	return sent ? len : -EIO;
}

/*
 * Send one management frame: write "<channel> <ma_msg> <arg>" where the three
 * fields are plain decimals and arg is optional.  This is how the peer is asked
 * to start a service (channel 1), and it reports what the peer did with the
 * frame so a silent peer cannot be mistaken for a working one.
 */
static ssize_t wlbt_debugfs_send(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int channel, msg, arg = 0;
	unsigned int before = w->n_mgmt;
	char text[64];
	bool sent;
	int i;

	if (len == 0 || len >= sizeof(text))
		return -EINVAL;
	if (copy_from_user(text, buf, len))
		return -EFAULT;
	text[len] = '\0';

	if (sscanf(text, "%u %u %u", &channel, &msg, &arg) < 2)
		return -EINVAL;
	if (channel > 255 || msg > 255)
		return -EINVAL;

	mutex_lock(&w->lock);
	sent = wlbt_mgmt_send(w, channel, msg, arg);

	/* Same doorbell trick as halt: look before draining, because the peer's
	 * log acknowledgements write the same register.
	 */
	for (i = 0; i < 10; i++) {
		if (!(readl(w->mbox + MBOX_INTSR1) & BIT(WLBT_BIT_MGMT_OUT_WRITE)))
			break;
		msleep(100);
	}
	dev_info(w->dev, "sent ch%u msg %u arg 0x%x (%s), peer %s the doorbell, ring read %u write %u\n",
		 channel, msg, arg, sent ? "written" : "failed",
		 i < 10 ? "took" : "ignored",
		 readl(w->dram + w->mgmt_out.ridx),
		 readl(w->dram + w->mgmt_out.widx));
	wlbt_mgmt_drain(w);
	dev_info(w->dev, "%u management message(s) in reply\n", w->n_mgmt - before);
	mutex_unlock(&w->lock);

	return sent ? len : -EIO;
}

static ssize_t wlbt_debugfs_reset(struct file *file, const char __user *buf,
				  size_t len, loff_t *ppos)
{	struct exynos_wlbt *w = file->private_data;
	int ret;

	mutex_lock(&w->lock);
	ret = wlbt_power_down(w);
	mutex_unlock(&w->lock);

	return ret ? ret : len;
}

/*
 * Start the WLAN service.  The peer reads the optional_data field of
 * SM_MSG_START_REQ as a MIF-relative address and walks it as a hip4_hip_control
 * (0x801185f4 converts the reference, 0x801178f8 hands it to the service), so
 * the HIP4 region has to exist before the request goes out - a zero reference
 * makes the peer read the firmware header as a control block and stop.
 */
/*
 * Send the WLAN service start and collect whatever the peer says in reply.
 * w->lock must be held.  Returns the write result: true when the request was
 * queued.
 */
int wlbt_wlan_start(struct exynos_wlbt *w)
{
	unsigned int before = w->n_mgmt;
	u8 frame[7];
	__le32 ref;
	int ret, i;

	ret = wlbt_hip4_build(w);
	if (ret)
		return ret;

	/* The vendor fills the peer's MIB database between opening the service
	 * and starting it (mgt.c:800-870), so the entries are in place before
	 * the WLAN driver there runs its own start.
	 */
	if (mib_early)
		wlbt_download_mibs(w);

	frame[0] = MMTRANS_CHAN_ID_SERVICE_MANAGEMENT;
	frame[1] = WLBT_SERVICE_ID_WLAN;
	frame[2] = WLBT_SM_MSG_START_REQ;
	ref = cpu_to_le32(w->hip4_control);
	memcpy(frame + 3, &ref, sizeof(ref));

	ret = wlbt_stream_write(w, &w->mgmt_out, frame, sizeof(frame));
	dev_info(w->dev, "WLAN start request (%s), control block 0x%x\n",
		 ret ? "written" : "failed", w->hip4_control);
	if (ret) {
		w->hip4_started = true;
		wlbt_hip4_read_limits(w);
	}

	/* A start confirmation comes back on the same channel; give the peer a
	 * moment and show whatever arrives.
	 */
	for (i = 0; i < 20; i++) {
		msleep(50);
		wlbt_mgmt_drain(w);
		if (w->n_mgmt != before)
			break;
	}
	dev_info(w->dev, "%u management message(s) after WLAN start\n",
		 w->n_mgmt - before);

	return ret;
}

static ssize_t wlbt_debugfs_wlan(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	int ret;

	mutex_lock(&w->lock);
	ret = wlbt_wlan_start(w);
	mutex_unlock(&w->lock);

	return ret ? len : -EIO;
}

/*
 * Talk to the peer's WLAN service over HIP4.  "send <hex signal id> [hex
 * payload bytes]" puts one FAPI signal on the from-host control queue,
 * "drain" collects whatever is waiting on the to-host queues.
 */
static ssize_t wlbt_debugfs_hip4(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	struct wlbt_fapi_hdr hdr = { };
	/*
	 * Static because w->lock serialises this handler and the buffers are
	 * too big for the stack.
	 */
	static u8 sig[sizeof(hdr) + 2048];
	static u8 dat[1024];
	static char kbuf[4608];
	char cmd[16];
	const char *p, *q;
	char *sep;
	unsigned int id, val, n = 0, dn = 0;
	unsigned long flags;
	int ret, used = 0;

	ret = simple_write_to_buffer(kbuf, sizeof(kbuf) - 1, ppos, buf, len);
	if (ret < 0)
		return ret;
	kbuf[ret] = '\0';

	mutex_lock(&w->lock);
	ret = wlbt_hip4_build(w);
	if (ret)
		goto out;

	if (sscanf(kbuf, "%15s", cmd) != 1) {
		ret = -EINVAL;
		goto out;
	}

	if (!strcmp(cmd, "state")) {
		wlbt_hip4_state(w);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "drain")) {
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	/*
	 * "sendd" queues one unitdata request on the data queue whatever the
	 * interface state is, so the peer can be asked whether it reads that
	 * queue before a connection exists as well as after one.  The frame is a
	 * real broadcast ARP request: if the peer does transmit it, it shows up
	 * in a capture on the access point.
	 */
	if (!strcmp(cmd, "sendd")) {
		static const u8 arp[42] = {
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc,
			0x08, 0x06,			/* ARP */
			0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
			0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc,
			0xc0, 0xa8, 0x63, 0x02,		/* 192.168.99.2 */
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xc0, 0xa8, 0x63, 0x01,		/* 192.168.99.1 */
		};
		u8 probe[sizeof(hdr) + 24];
		struct wlbt_fapi_hdr dh = { };

		dh.id = cpu_to_le16(0x1000);		/* MA_UNITDATA_REQ */
		dh.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
		dh.fw_reference = cpu_to_le32(send_ref);
		memcpy(probe, &dh, sizeof(dh));
		put_unaligned_le16(1, probe + sizeof(dh) + 0);	/* vif */
		put_unaligned_le16(w->host_tag += 4, probe + sizeof(dh) + 2);
		put_unaligned_le16(FAPI_PRIORITY_CONTENTION,
				   probe + sizeof(dh) + 4);
		put_unaligned_le16(1, probe + sizeof(dh) + 6);	/* peer */
		put_unaligned_le16(FAPI_DATAUNIT_IEEE802_3,
				   probe + sizeof(dh) + 8);

		ret = wlbt_hip4_send(w, false, probe, sizeof(probe), arp,
				     sizeof(arp));
		dev_info(w->dev, "HIP4 data probe queued: %d\n", ret);
		if (ret)
			goto out;
		msleep(1500);
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	/* Ring the from-host interrupt again without queueing anything. */
	if (!strcmp(cmd, "kick")) {
		writel(BIT(WLBT_BIT_HIP4_FROM_HOST), w->mbox + MBOX_INTGR1);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "scan")) {
		if (!w->hip4_started) {
			ret = -EINVAL;
			goto out;
		}

		ret = wlbt_mlme_add_scan(w, 0x0100, NULL);
		if (ret)
			goto out;
		msleep(500);
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "mibs")) {
		if (!w->hip4_started) {
			ret = -EINVAL;
			goto out;
		}
		ret = wlbt_download_mibs(w);
		if (ret < 0)
			goto out;
		msleep(200);
		wlbt_hip4_drain(w);
		ret = len;
		goto out;
	}

	/*
	 * "mibset <psid> <hex bytes>" writes one entry into the peer's MIB database
	 * at run time, which is how a setting is tried out without a reboot: the
	 * files in the HIP4 area are only read when the peer starts.
	 */
	if (!strncmp(cmd, "mibset", 6)) {
		u8 rec[4 + 64];
		unsigned int psid, dlen = 0;
		char *h;

		if (sscanf(kbuf + 6, "%x", &psid) != 1 ||
		    !w->hip4_started) {
			ret = -EINVAL;
			goto out;
		}
		h = strchr(kbuf + 6, ' ');
		if (h) {
			for (h++; *h && dlen < sizeof(rec) * 2; h++) {
				if (!isxdigit(*h))
					continue;
				val = hex_to_bin(*h);
				if (dlen & 1)
					rec[5 + dlen / 2] |= val;
				else
					rec[5 + dlen / 2] = val << 4;
				dlen++;
			}
		}
		if (dlen & 1) {
			ret = -EINVAL;
			goto out;
		}
		dlen /= 2;
		put_unaligned_le16(psid, rec);
		put_unaligned_le16(dlen, rec + 2);
		if (dlen & 1)
			rec[5 + dlen / 2] = 0;
		wlbt_mib_send_slice(w, rec, ALIGN(4 + dlen, 2));
		dev_info(w->dev, "MIB 0x%04x set to %u bytes\n", psid, dlen);
		ret = len;
		goto out;
	}

	if (!strcmp(cmd, "ret")) {
		unsigned int ref;

		if (sscanf(kbuf + strlen(cmd), "%x", &ref) != 1) {
			ret = -EINVAL;
			goto out;
		}
		spin_lock_irqsave(&w->hip4_lock, flags);
		wlbt_hip4_push(w, HIP4_Q_TH_RFB, ref);
		spin_unlock_irqrestore(&w->hip4_lock, flags);
		dev_info(w->dev, "HIP4 returned segment 0x%x\n", ref);
		ret = len;
		goto out;
	}

	if ((strcmp(cmd, "send") && strcmp(cmd, "sendp")) || !w->hip4_started) {
		ret = -EINVAL;
		goto out;
	}

	p = kbuf + strlen(cmd);
	if (sscanf(p, "%x%n", &id, &used) != 1) {
		ret = -EINVAL;
		goto out;
	}
	p += used;
	while (*p == ' ')
		p++;

	/*
	 * "sendp" takes a second, whitespace separated run of bytes: the
	 * payload that follows the signal.  Requests carrying information
	 * elements want them there, not inside the signal.
	 */
	if (!strcmp(cmd, "sendp")) {
		sep = strchr(p, ' ');
		if (!sep) {
			ret = -EINVAL;
			goto out;
		}
		*sep = '\0';
		q = sep + 1;
		for (; *q && dn < sizeof(dat); q++) {
			if (!isxdigit(*q))
				continue;
			val = hex_to_bin(*q);
			if (dn & 1)
				dat[dn / 2] |= val;
			else
				dat[dn / 2] = val << 4;
			dn++;
		}
		if (dn & 1) {
			ret = -EINVAL;
			goto out;
		}
		dn /= 2;
	}

	/* The signal is a hex string with the bytes run together. */
	for (; *p && n < sizeof(sig) - sizeof(hdr); p++) {
		if (!isxdigit(*p))
			continue;
		val = hex_to_bin(*p);
		if (n & 1)
			sig[sizeof(hdr) + n / 2] |= val;
		else
			sig[sizeof(hdr) + n / 2] = val << 4;
		n++;
	}
	if (n & 1) {
		ret = -EINVAL;
		goto out;
	}
	n /= 2;

	hdr.id = cpu_to_le16(id);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	hdr.fw_reference = cpu_to_le32(send_ref);
	memcpy(sig, &hdr, sizeof(hdr));

	ret = wlbt_hip4_send(w, true, sig, sizeof(hdr) + n,
			     dn ? dat : NULL, dn);
	dev_info(w->dev, "HIP4 signal 0x%04x (%zu byte signal, %u data): %d\n",
		 id, sizeof(hdr) + n, dn, ret);
	if (ret)
		goto out;

	msleep(100);
	wlbt_hip4_drain(w);
	ret = len;

out:
	mutex_unlock(&w->lock);
	return ret;
}



/*
 * The peer's RTOS keeps its kernel block, thread descriptors and stacks in MIF
 * DRAM, so a plain read of the right offset shows what the peer is doing
 * without touching a single doorbell.  Write "<hex offset> <words>" and read
 * the words back.
 */
static int wlbt_debugfs_peek_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	u32 i;

	if (!w->dram)
		return 0;

	for (i = 0; i < w->peek_words; i++) {
		u32 off = w->peek_offset + i * 4;

		if (off + 4 > w->mem_size)
			break;
		seq_printf(s, "%06x %08x%s", off, readl(w->dram + off),
			   i % 4 == 3 ? "\n" : "  ");
	}
	if (i % 4)
		seq_putc(s, '\n');

	return 0;
}

static int wlbt_peek_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_peek_show, inode->i_private);
}

static ssize_t wlbt_debugfs_peek(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	unsigned int off, words;
	char cmd[64];

	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (sscanf(cmd, "%x %u", &off, &words) != 2)
		return -EINVAL;

	w->peek_offset = off & ~3u;
	w->peek_words = min_t(unsigned int, words, WLBT_PEEK_MAX_WORDS);

	return len;
}

/*
 * MLME_GET asks the peer for MIB values: the request body is the interface and
 * the payload is a list of {le16 psid, u8 index count, u8 pad} entries, four
 * bytes when no index is wanted.  The answer comes back as MLME_GET_CFM with the
 * values appended, and it is worth having because the peer discards frames
 * silently - its own counters are the only place a discard shows up at all.
 */
static int wlbt_mib_get_send(struct exynos_wlbt *w, u16 psid)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };
	u8 mib[4] = { };

	hdr.id = cpu_to_le16(0x2001);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif 1 */
	put_unaligned_le16(psid, mib);			/* no indexes */

	w->mibresp_len = 0;
	w->mibresp_psid = psid;

	return wlbt_hip4_send(w, true, sig, sizeof(sig), mib, sizeof(mib));
}

static int wlbt_mib_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	u32 i;

	seq_printf(s, "psid %04x len %u:", w->mibresp_psid, w->mibresp_len);
	for (i = 0; i < w->mibresp_len && i < sizeof(w->mibresp); i++)
		seq_printf(s, " %02x", w->mibresp[i]);
	seq_putc(s, '\n');

	return 0;
}

static int wlbt_mib_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_mib_show, inode->i_private);
}

static ssize_t wlbt_mib_write(struct file *file, const char __user *buf,
			      size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	unsigned int psid;
	char cmd[32];
	int ret;

	if (!w->hip4_control)
		return -EIO;
	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	ret = kstrtouint(cmd, 16, &psid);
	if (ret)
		return ret;
	if (wlbt_mib_get_send(w, psid))
		return -EIO;

	return len;
}

static const struct file_operations wlbt_mib_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_mib_open,
	.read = seq_read,
	.write = wlbt_mib_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * Write "<hex offset> <hex value>" into the peer's DRAM.  Debug only, and the
 * narrow complement of peek: when a field the peer owns is the only thing
 * standing between a working and a broken data path, setting it by hand is how
 * the field is identified at all, before there is any code to read.
 */
static ssize_t wlbt_debugfs_poke(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct exynos_wlbt *w = file->private_data;
	unsigned int off, val;
	char cmd[64];

	if (!w->dram)
		return -ENODEV;
	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (sscanf(cmd, "%x %x", &off, &val) != 2)
		return -EINVAL;
	off &= ~3u;
	if (off + 4 > w->mem_size)
		return -EINVAL;

	writel(val, w->dram + off);
	/* The peer may be polling the word right now, so finish the store. */
	wmb();

	return len;
}

/*
 * The peer is a dual-core RTOS with a saved context per thread, so the PC it
 * will resume at is a plain word in DRAM.  Sampling that word from here shows
 * where the peer stops running, at the moment it stops, without a debugger.
 * "<hex offset> <hex field> <microseconds>": field 0 samples the word itself,
 * anything else treats the word as a pointer into MIF DRAM and samples the
 * word at that address plus the field.
 */
static u32 wlbt_trace_sample(struct exynos_wlbt *w)
{
	u32 v = readl(w->dram + w->trace_offset);
	u32 off;

	if (!w->trace_field)
		return v;

	if (v < 0x80000000 || v - 0x80000000 + w->trace_field + 4 > w->mem_size)
		return 0;
	off = (v - 0x80000000) + w->trace_field;

	return readl(w->dram + off);
}

static void wlbt_trace_run(struct exynos_wlbt *w)
{
	u64 start, now, end;
	u32 v, last = 0;
	unsigned int i = 0, n = 0;

	if (!w->dram || !w->trace_us || !w->trace)
		return;

	start = ktime_get_ns();
	end = start + (u64)w->trace_us * NSEC_PER_USEC;
	do {
		v = wlbt_trace_sample(w);
		if (v != last && i < WLBT_TRACE_MAX) {
			now = ktime_get_ns();
			w->trace[i].us = (u32)((now - start) / NSEC_PER_USEC);
			w->trace[i].val = v;
			i++;
			last = v;
		}
		now = ktime_get_ns();
		if (!(++n & 0x3fff))
			cond_resched();
	} while (now < end);

	w->n_trace = i;
}

static int wlbt_trace_thread(void *data)
{
	struct exynos_wlbt *w = data;

	wlbt_trace_run(w);
	atomic_set(&w->trace_running, 0);

	return 0;
}

bool wlbt_trace_start(struct exynos_wlbt *w)
{
	struct task_struct *task;

	if (atomic_cmpxchg(&w->trace_running, 0, 1))
		return false;

	task = kthread_run(wlbt_trace_thread, w, "wlbt-trace");
	if (IS_ERR(task)) {
		atomic_set(&w->trace_running, 0);
		return false;
	}

	return true;
}

static int wlbt_debugfs_trace_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;

	if (!w->dram)
		return 0;

	seq_printf(s, "%u us sampling %06x%s, %u changes%s\n",
		   w->trace_us, w->trace_offset,
		   w->trace_field ? " (indirect)" : "", w->n_trace,
		   atomic_read(&w->trace_running) ? " (still running)" : "");
	for (i = 0; i < w->n_trace; i++)
		seq_printf(s, "%8u us  %08x\n", w->trace[i].us, w->trace[i].val);

	return 0;
}

static int wlbt_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_trace_show, inode->i_private);
}

static ssize_t wlbt_debugfs_trace(struct file *file, const char __user *buf,
				  size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	unsigned int off, field, us;
	char cmd[64];

	if (len >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, len))
		return -EFAULT;
	cmd[len] = '\0';

	if (sscanf(cmd, "%x %x %u", &off, &field, &us) != 3)
		return -EINVAL;

	w->trace_offset = off & ~3u;
	w->trace_field = field & ~3u;
	w->trace_us = min_t(unsigned int, us, 30 * USEC_PER_SEC);

	if (!wlbt_trace_start(w))
		return -EBUSY;

	return len;
}


/*
 * The peer's thread descriptors carry their name inline, so looking for a name
 * in MIF DRAM finds the descriptor without knowing the RTOS internals.
 */
static void wlbt_dram_find(struct exynos_wlbt *w)
{
	u32 limit, off, n;
	u8 *buf;

	w->n_find = 0;
	if (!w->dram || !w->find_len)
		return;

	/* The peer allocates from the MIF heap above its runtime image, so the
	 * whole window has to be searched, not only the loaded image.
	 */
	limit = w->mem_size;
	buf = kzalloc(WLBT_FIND_CHUNK * 4, GFP_KERNEL);
	if (!buf)
		return;

	for (off = 0; off + 4 <= limit; off += WLBT_FIND_CHUNK * 4) {
		u32 i;

		n = min((u32)(WLBT_FIND_CHUNK * 4), limit - off);
		memcpy_fromio(buf, w->dram + off, n);
		for (i = 0; i + w->find_len <= n; i++) {
			if (buf[i] != w->find_needle[0])
				continue;
			if (memcmp(buf + i, w->find_needle, w->find_len))
				continue;
			if (w->n_find < ARRAY_SIZE(w->find_hits))
				w->find_hits[w->n_find++] = off + i;
		}
		if (w->n_find >= ARRAY_SIZE(w->find_hits))
			break;
	}

	kfree(buf);
}

static int wlbt_debugfs_find_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;

	seq_printf(s, "n_find %u over %llu bytes\n", w->n_find,
		   (unsigned long long)w->mem_size);
	for (i = 0; i < w->n_find; i++)
		seq_printf(s, "  %06x\n", w->find_hits[i]);

	return 0;
}

static int wlbt_find_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_find_show, inode->i_private);
}

static ssize_t wlbt_debugfs_find(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	char needle[96];
	size_t count = len;

	if (!len || len >= sizeof(needle))
		return -EINVAL;
	if (copy_from_user(needle, buf, len))
		return -EFAULT;
	if (needle[len - 1] == '\n')
		len--;
	if (!len || len >= sizeof(needle))
		return -EINVAL;

	/* "0x..." searches for bytes rather than text: a key is not a string. */
	if (len > 4 && needle[0] == '0' && needle[1] == 'x') {
		unsigned int i, n = 0;

		for (i = 2; i < len && n < sizeof(w->find_needle); i += 2) {
			int hi = hex_to_bin(needle[i]);
			int lo = (i + 1 < len) ? hex_to_bin(needle[i + 1]) : 0;

			if (hi < 0 || lo < 0)
				return -EINVAL;
			w->find_needle[n++] = (hi << 4) | lo;
		}
		w->find_len = n;
	} else {
		if (len >= sizeof(w->find_needle))
			return -EINVAL;
		memcpy(w->find_needle, needle, len);
		w->find_needle[len] = '\0';
		w->find_len = len;
	}

	wlbt_dram_find(w);

	/* Report the whole write: the needle is shortened, the write is not. */
	return count;
}

static const struct file_operations wlbt_find_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_find_open,
	.read = seq_read,
	.write = wlbt_debugfs_find,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wlbt_trace_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_trace_open,
	.read = seq_read,
	.write = wlbt_debugfs_trace,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_bss_show(struct seq_file *s, void *unused)
{
	wlbt_dram_bsses(s->private, s);
	return 0;
}

static int wlbt_bss_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_bss_show, inode->i_private);
}

static const struct file_operations wlbt_bss_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_bss_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wlbt_poke_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_poke);

static const struct file_operations wlbt_peek_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_peek_open,
	.read = seq_read,
	.write = wlbt_debugfs_peek,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations wlbt_boot_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_boot);
static const struct file_operations wlbt_reset_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_reset);

static const struct file_operations wlbt_halt_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_halt);
static const struct file_operations wlbt_send_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_send);
static const struct file_operations wlbt_wlan_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_wlan);
static const struct file_operations wlbt_hip4_fops =
	WLBT_DEBUGFS_OPS(wlbt_debugfs_hip4);

/*
 * What did the peer do with its memory?  The image occupies [0, fw_size) and
 * the runtime, heap and panic record live above it, so anything non-zero up
 * there was written by the peer - the cheapest proof that the firmware is
 * really running, and where it stopped if it is not.
 */
static void wlbt_dram_report(struct exynos_wlbt *w, struct seq_file *s)
{
	unsigned int i, nz = 0, first_nz = 0;
	u32 val;

	if (!w->dram || !w->fw_loaded)
		return;

	for (i = w->fw_size & ~3u; i + 4 <= w->fw_runtime_length; i += 4) {
		if (readl(w->dram + i)) {
			if (!nz)
				first_nz = i;
			nz++;
		}
	}

	seq_printf(s, "peer memory: %u non-zero words above the image (%u bytes), first at 0x%x\n",
		   nz, w->fw_size, first_nz);

	if (nz) {
		seq_printf(s, "peer words @0x%x:", first_nz);
		for (i = 0; i < 8; i++)
			seq_printf(s, " %08x",
				   readl(w->dram + first_nz + i * 4));
		seq_puts(s, "\n");
	}

	if (w->fw_panic_offset && w->fw_panic_offset + 32 <= w->mem_size) {
		seq_printf(s, "panic record @0x%x:", w->fw_panic_offset);
		for (i = 0; i < 8; i++) {
			val = readl(w->dram + w->fw_panic_offset + i * 4);
			seq_printf(s, " %08x", val);
		}
		seq_puts(s, "\n");
	}
}

static void wlbt_stream_status(struct exynos_wlbt *w, struct seq_file *s,
			       struct wlbt_stream *st)
{
	if (!w->dram)
		return;

	seq_printf(s, "stream %-10s buffer +0x%06x cells %4u x %3u  bits %u/%u  read %u write %u\n",
		   st->name, st->buf, st->num_packets, st->packet_size,
		   st->read_bit, st->write_bit,
		   readl(w->dram + st->ridx), readl(w->dram + st->widx));
}

static int wlbt_debugfs_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;
	u32 val;

	seq_printf(s, "state %s  MIF DRAM %pa  irq cfg-req %u bad %u wdog %u mbox %u\n",
		   wlbt_state_name(w->state), &w->mem_base, w->n_cfg_req,
		   w->n_cfg_bad, w->n_wdog, w->n_mbox);
	seq_printf(s, "peer state: %s\n", w->peer_dead ? "stopped" : "running");
	if (w->n_fw_event) {
		unsigned int i, n = min_t(unsigned int, w->n_fw_event, 32);

		seq_printf(s, "firmware events (%u):", w->n_fw_event);
		for (i = 0; i < n; i++) {
			unsigned int k = (w->n_fw_event - n + i) & 31;

			seq_printf(s, " %u/%u", w->fw_events[k].vif,
				   w->fw_events[k].event);
		}
		seq_putc(s, '\n');
	}
	if (w->n_mgmt_rx)
		seq_printf(s, "mgmt: %u management frame(s) received\n",
			   w->n_mgmt_rx);
	if (w->n_sae_ind)
		seq_printf(s, "sae: %u indication(s), last flag 0x%02x sig %u dat %u len %u head %u chain-ref 0x%x own-ref 0x%x chain %u\n",
			   w->n_sae_ind, w->sae_dbg[0], w->sae_dbg[1],
			   w->sae_dbg[2], w->sae_dbg[3], w->sae_dbg[4],
			   w->sae_dbg[6], w->sae_dbg[7], w->sae_chain);
	seq_printf(s, "firmware %s, entry 0x%x, runtime %u, dram %s\n",
		   firmware_name, w->fw_entry, w->fw_runtime_length,
		   w->fw_loaded ? "loaded" : "not loaded");
	seq_printf(s, "mailbox last status 0x%04x\n", w->last_mbox_status);
	/* The WLBT power-gate configuration: what was saved at probe and what the
	 * PMU holds now, so a restore can be checked against it.
	 */
	for (i = 0; i < WLBT_PW_REGS; i++) {
		u32 now = 0;

		regmap_read(w->pmu, w->pw_cfg[i].reg, &now);
		seq_printf(s, "pmu 0x%04x saved 0x%08x now 0x%08x\n",
			   w->pw_cfg[i].reg, w->pw_cfg[i].val, now);
	}
	seq_printf(s, "central seq: cfg 0x%08x status 0x%08x ctrl_ns 0x%08x\n",
		   ({ u32 v = 0; regmap_read(w->pmu, CENTRAL_SEQ_WLBT_CONFIGURATION, &v); v; }),
		   ({ u32 v = 0; regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &v); v; }),
		   ({ u32 v = 0; regmap_read(w->pmu, WLBT_CTRL_NS, &v); v; }));
	if (w->n_rxind)
		seq_printf(s, "rxind %u, sig %u dat %u len %u head %u, body %*phN\n",
			   w->n_rxind, w->peer_rxind_geom[0], w->peer_rxind_geom[1],
			   w->peer_rxind_geom[2], w->peer_rxind_geom[3],
			   (int)min_t(u32, w->peer_rxind_len, 18), w->peer_rxind);
	if (w->rxind_raw_len)
		seq_printf(s, "rxind segment: %*phN\n",
			   (int)min_t(u16, w->rxind_raw_len, 160), w->rxind_raw);
	if (w->chain_probe_len)
		seq_printf(s, "chain probe 0x%x: %*phN\n", w->chain_probe_ref,
			   (int)w->chain_probe_len, w->chain_probe);
	if (w->connect_reported)
		seq_printf(s, "connect result 0x%04x\n", w->connect_status);
	seq_printf(s, "link %s, tx %lu called %u dropped %u (unauthenticated %u), hip4 tx %u rx %u\n",
		   w->connected ? "up" : "down",
		   w->netdev ? w->netdev->stats.tx_packets : 0UL,
		   w->n_tx_calls, w->n_tx_dropped, w->n_tx_unauthed,
		   w->n_hip4_tx, w->n_hip4_rx);
	if (w->drain_calls)
		seq_printf(s, "drain %u calls %llu us %llu ns/call, amsdu frames %u\n",
			   w->drain_calls,
			   w->drain_ns / 1000,
			   w->drain_ns / w->drain_calls,
			   w->n_rx_amsdu_frames);
	if (w->hip4_ref)
		seq_printf(s, "hip4 0x%x control 0x%x mib 0x%x (%u bytes)\n",
			   w->hip4_ref, w->hip4_control,
			   w->hip4_ref + HIP4_MIB_OFFSET, HIP4_MIB_SIZE);

	regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &val);
	seq_printf(s, "CENTRAL_SEQ_WLBT_STATUS 0x%08x\n", val);
	regmap_read(w->pmu, WLBT_CTRL_NS, &val);
	seq_printf(s, "WLBT_CTRL_NS 0x%08x\n", val);
	regmap_read(w->pmu, WLBT_CTRL_S, &val);
	seq_printf(s, "WLBT_CTRL_S 0x%08x\n", val);
	regmap_read(w->pmu, CENTRAL_SEQ_WLBT_CONFIGURATION, &val);
	seq_printf(s, "CENTRAL_SEQ_WLBT_CONFIGURATION 0x%08x\n", val);

	seq_printf(s, "irq to peer: INTGR1 0x%08x INTMR1 0x%08x INTSR1 0x%08x\n",
		   readl(w->mbox + MBOX_INTGR1), readl(w->mbox + MBOX_INTMR1),
		   readl(w->mbox + MBOX_INTSR1));
	seq_printf(s, "mailbox IS_VERSION 0x%08x MCUCTRL 0x%08x INTSR0 0x%08x INTMR0 0x%08x\n",
		   readl(w->mbox + MBOX_IS_VERSION),
		   readl(w->mbox + MBOX_MCUCTRL),
		   readl(w->mbox + MBOX_INTSR0),
		   readl(w->mbox + MBOX_INTMR0));

	wlbt_dram_report(w, s);

	if (w->dram && w->mxconf_offset) {
		wlbt_stream_status(w, s, &w->mgmt_in);
		wlbt_stream_status(w, s, &w->mgmt_out);
		wlbt_stream_status(w, s, &w->gdb_r4_in);
		wlbt_stream_status(w, s, &w->gdb_r4_out);
		wlbt_stream_status(w, s, &w->gdb_m4_in);
		wlbt_stream_status(w, s, &w->gdb_m4_out);
		wlbt_stream_status(w, s, &w->log);
		seq_printf(s, "streams: log %u bytes queued (fw %u, bad %u), mgmt %u messages\n",
			   ((readl(w->dram + w->log.widx) + w->log.num_packets -
			     readl(w->dram + w->log.ridx)) % w->log.num_packets) *
			   w->log.packet_size,
			   w->n_fwlog, w->n_fwlog_bad, w->n_mgmt);
			seq_printf(s, "mxconf at MIF+0x%x, heap next 0x%x\n",
			   w->mxconf_offset, w->heap_next);

		/* The firmware reports its progress as 0x2222_0000 | stage in
		 * mailbox 0 (0x8011d568 in the image).
		 */
		val = readl(w->mbox + MBOX_ISSR_BASE);
		if ((val & 0xffff0000) == 0x22220000)
			seq_printf(s, "peer boot stage %u\n", val & 0xffff);
	}

	if (w->dram)
		for (i = 0; i < 8; i++)
			seq_printf(s, "dram[%u] 0x%08x%s", i,
				   readl(w->dram + i * 4),
				   i % 4 == 3 ? "\n" : "  ");

	/* The peer writes mailboxes 8 and 9 (image 0x80117f2a), past the eight
	 * the vendor's mbox manager knows about.
	 */
	for (i = 0; i < 12; i++)
		seq_printf(s, "ISSR%u 0x%08x%s", i,
			   readl(w->mbox + MBOX_ISSR_BASE + i * 4),
			   i % 4 == 3 ? "\n" : "  ");

	return 0;
}

static int wlbt_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_debugfs_show, inode->i_private);
}

static const struct file_operations wlbt_dump_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_frames_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i, n;

	if (!w->frame_total) {
		seq_puts(s, "no frames recorded\n");
		return 0;
	}

	seq_printf(s, "%u frame(s), last %u kept\n", w->frame_total,
		   min_t(unsigned int, w->frame_total, WLBT_FRAME_RING));
	n = min_t(unsigned int, w->frame_total, WLBT_FRAME_RING);
	for (i = 0; i < n; i++) {
		unsigned int slot = (w->frame_next - n + i) &
				    (WLBT_FRAME_RING - 1);

		seq_printf(s, "ref 0x%x flag 0x%02x id 0x%04x len %u\n",
			   w->frames[slot].ref, w->frames[slot].flag,
			   w->frames[slot].id, w->frames[slot].len);
		seq_hex_dump(s, "  ", DUMP_PREFIX_NONE, 32, 1,
			     w->frames[slot].data, w->frames[slot].len, false);
	}

	return 0;
}

static int wlbt_frames_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_frames_show, inode->i_private);
}

static const struct file_operations wlbt_frames_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_frames_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_ids_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i, k;

	seq_printf(s, "%u identifier(s)\n", w->n_ids);
	for (i = 0; i < w->n_ids; i++) {
		seq_printf(s, "0x%04x %u:", w->ids[i].id, w->ids[i].count);
		for (k = 0; k < w->ids[i].len; k++)
			seq_printf(s, " %02x", w->ids[i].body[k]);
		seq_putc(s, '\n');
	}

	return 0;
}

static int wlbt_evlog_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i;

	seq_printf(s, "%u record kind(s)\n", w->log_used);
	for (i = 0; i < WLBT_LOG_SLOTS; i++) {
		if (!w->log_slots[i].count)
			continue;
		seq_printf(s, "m %u s %u w0 %04x w1 %04x n %u\n",
			   w->log_slots[i].module, w->log_slots[i].sub,
			   w->log_slots[i].word0, w->log_slots[i].word1,
			   w->log_slots[i].count);
	}

	return 0;
}

static ssize_t wlbt_evlog_write(struct file *file, const char __user *ubuf,
				size_t len, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;

	memset(w->log_slots, 0, sizeof(w->log_slots));
	w->log_used = 0;

	return len;
}

/*
 * The control frames sent, oldest first, in the same shape the vendor's
 * bin_wifi_ctrl_tx records have on Android: id first, then the body.  Only the
 * head of each frame is kept - the signal id and the fields a comparison needs.
 */
static int wlbt_tx_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i, first;

	seq_printf(s, "%u frame(s) sent\n", w->tx_used);
	if (!w->tx)
		return 0;

	first = w->tx_next >= w->tx_used ? w->tx_next - w->tx_used : 0;
	for (i = first; i < w->tx_next; i++) {
		struct wlbt_tx_rec *r = &w->tx[i % WLBT_TX_SLOTS];

		seq_printf(s, "%14lld sig=0x%04x len=%u %*phN",
			   r->t, r->id, r->len, r->len, r->body);
		if (r->dlen)
			seq_printf(s, " | data %*phN", r->dlen, r->data);
		seq_putc(s, '\n');
	}

	return 0;
}

static ssize_t wlbt_tx_write(struct file *file, const char __user *ubuf,
			     size_t len, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	unsigned long flags;

	spin_lock_irqsave(&w->hip4_lock, flags);
	w->tx_next = 0;
	w->tx_used = 0;
	spin_unlock_irqrestore(&w->hip4_lock, flags);

	return len;
}

static int wlbt_tx_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_tx_show, inode->i_private);
}

static const struct file_operations wlbt_tx_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_tx_open,
	.read = seq_read,
	.write = wlbt_tx_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_evlog_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_evlog_show, inode->i_private);
}

static const struct file_operations wlbt_evlog_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_evlog_open,
	.read = seq_read,
	.write = wlbt_evlog_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* The keystore is only used below; the peer answers CFMs for it. */
int wlbt_mlme_setkeys(struct exynos_wlbt *w, u16 key_id, u16 key_type,
			     const u8 *address, u32 cipher, const u8 *key,
			     size_t key_len, const u8 *seq, size_t seq_len);

/*
 * Writing here sends the stored pairwise key to the peer again, from the same
 * code path the supplicant's key goes through.  Nothing else changes, so the
 * difference between this and an injected request is only the transport - and
 * that is what made the peer install one and merely store the other.
 */
static int wlbt_ptk_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;

	seq_printf(s, "stored %d id %u cipher 0x%x len %u addr %pM\n",
		   w->ptk_valid, w->ptk_id, w->ptk_cipher, w->ptk_len,
		   w->ptk_addr);

	return 0;
}

static ssize_t wlbt_ptk_write(struct file *file, const char __user *ubuf,
			      size_t len, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct exynos_wlbt *w = s->private;
	char buf[68];
	char hex[33];
	unsigned int i, n;

	if (!w->ptk_valid || !w->hip4_control)
		return -EINVAL;

	n = min(len, sizeof(buf) - 1);
	if (n && copy_from_user(buf, ubuf, n))
		return -EFAULT;
	buf[n] = '\0';

	/*
	 * A hex string replaces the stored key first, so the peer cannot take the
	 * "this exact key is already installed" shortcut and has to allocate.  An
	 * empty write re-sends the key the supplicant handed over.
	 */
	for (i = 0; i < sizeof(hex) - 1 && i < n; i++) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t')
			break;
		hex[i] = buf[i];
	}
	hex[i] = '\0';
	if (i == 2 * w->ptk_len)
		if (hex2bin(w->ptk_key, hex, w->ptk_len))
			return -EINVAL;

	dev_info(w->dev, "ptk: sending id %u type %u cipher 0x%x len %u %pM\n",
		 w->ptk_id, ptk_type, w->ptk_cipher, w->ptk_len, w->ptk_addr);

	wlbt_mlme_setkeys(w, w->ptk_id, ptk_type, w->ptk_addr, w->ptk_cipher,
			  w->ptk_key, w->ptk_len, NULL, 0);

	return len;
}

static int wlbt_ptk_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_ptk_show, inode->i_private);
}

static const struct file_operations wlbt_ptk_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_ptk_open,
	.read = seq_read,
	.write = wlbt_ptk_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_ids_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_ids_show, inode->i_private);
}

static const struct file_operations wlbt_ids_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_ids_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int wlbt_faults_show(struct seq_file *s, void *unused)
{
	struct exynos_wlbt *w = s->private;
	unsigned int i, n;

	if (!w->fault_next) {
		seq_puts(s, "no fault reports\n");
		return 0;
	}

	n = min_t(unsigned int, w->fault_next, WLBT_FAULT_RING);
	seq_printf(s, "%u report(s), last %u kept\n", w->fault_next, n);
	for (i = 0; i < n; i++) {
		unsigned int slot = (w->fault_next - n + i) &
				    (WLBT_FAULT_RING - 1);

		seq_printf(s, "id 0x%04x len %u\n", w->faults[slot].id,
			   w->faults[slot].len);
		seq_hex_dump(s, "  ", DUMP_PREFIX_NONE, 28, 1,
			     w->faults[slot].data, w->faults[slot].len, false);
	}

	return 0;
}

static int wlbt_faults_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlbt_faults_show, inode->i_private);
}

static const struct file_operations wlbt_faults_fops = {
	.owner = THIS_MODULE,
	.open = wlbt_faults_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void wlbt_debugfs_init(struct exynos_wlbt *w)
{
	w->debugfs = debugfs_create_dir("exynos9610-wlbt", NULL);
	if (IS_ERR_OR_NULL(w->debugfs)) {
		w->debugfs = NULL;
		return;
	}

	debugfs_create_file("dump", 0400, w->debugfs, w, &wlbt_dump_fops);
	debugfs_create_file("boot", 0200, w->debugfs, w, &wlbt_boot_fops);
	debugfs_create_file("off", 0200, w->debugfs, w, &wlbt_reset_fops);
	debugfs_create_file("halt", 0200, w->debugfs, w, &wlbt_halt_fops);
	debugfs_create_file("send", 0200, w->debugfs, w, &wlbt_send_fops);
	debugfs_create_file("wlan", 0200, w->debugfs, w, &wlbt_wlan_fops);
	debugfs_create_file("hip4", 0200, w->debugfs, w, &wlbt_hip4_fops);
	debugfs_create_file("peek", 0600, w->debugfs, w, &wlbt_peek_fops);
	debugfs_create_file("poke", 0200, w->debugfs, w, &wlbt_poke_fops);
	debugfs_create_file("mib", 0600, w->debugfs, w, &wlbt_mib_fops);
	debugfs_create_file("trace", 0600, w->debugfs, w, &wlbt_trace_fops);
	debugfs_create_file("find", 0600, w->debugfs, w, &wlbt_find_fops);
	debugfs_create_file("bss", 0400, w->debugfs, w, &wlbt_bss_fops);
	debugfs_create_file("frames", 0400, w->debugfs, w, &wlbt_frames_fops);
	debugfs_create_file("faults", 0400, w->debugfs, w, &wlbt_faults_fops);
	debugfs_create_file("ids", 0400, w->debugfs, w, &wlbt_ids_fops);
	debugfs_create_file("evlog", 0600, w->debugfs, w, &wlbt_evlog_fops);
	debugfs_create_file("tx", 0600, w->debugfs, w, &wlbt_tx_fops);
	debugfs_create_file("ptk", 0600, w->debugfs, w, &wlbt_ptk_fops);
}


