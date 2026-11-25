/* SPDX-License-Identifier: GPL-2.0 */
#include "exynos9610-wlbt.h"

int wlbt_hip4_build(struct exynos_wlbt *w)
{
	struct wlbt_hip4_control *c;
	u32 base, i, mib;

	if (w->hip4_control)
		return 0;

	base = wlbt_heap_alloc_aligned(w, HIP4_ALLOC_SIZE, 4096);
	if (!base)
		return -ENOMEM;

	w->hip4_ref = base;
	w->hip4_control = base + HIP4_CONFIG_OFFSET;
	c = (struct wlbt_hip4_control *)(w->dram + w->hip4_control);

	/* Scoreboard and queue arrays first: their refs go into both blocks. */
	c->init.magic = cpu_to_le32(HIP4_INIT_MAGIC);
	c->init.conf_ver = 0;
	c->init.version_a_ref = cpu_to_le32(w->hip4_control +
					     offsetof(struct wlbt_hip4_control, v4));
	c->init.version_b_ref = cpu_to_le32(w->hip4_control +
					     offsetof(struct wlbt_hip4_control, v5));

	c->v4.magic = cpu_to_le32(HIP4_CONFIG_MAGIC);
	c->v4.ver = cpu_to_le16(4);
	c->v4.len = cpu_to_le16(sizeof(c->v4));
	c->v4.host_cache_line = 64;
	c->v4.host_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_TX_OFFSET);
	c->v4.host_buf_sz = cpu_to_le32(HIP4_TX_SIZE);
	c->v4.fw_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_RX_OFFSET);
	c->v4.fw_buf_sz = cpu_to_le32(HIP4_RX_SIZE);
	c->v4.mif_fh_int_n = WLBT_BIT_HIP4_FROM_HOST;
	c->v4.scbrd_loc = cpu_to_le32(w->hip4_control +
				      offsetof(struct wlbt_hip4_control, scoreboard));
	c->v4.q_num = cpu_to_le16(HIP4_Q_NUM);
	c->v4.q_len = cpu_to_le16(HIP4_Q_LEN);
	c->v4.q_idx_sz = cpu_to_le16(HIP4_Q_IDX_SIZE);

	c->v5.magic = cpu_to_le32(HIP4_CONFIG_MAGIC);
	c->v5.ver = cpu_to_le16(5);
	c->v5.len = cpu_to_le16(sizeof(c->v5));
	c->v5.host_cache_line = 64;
	c->v5.host_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_TX_OFFSET);
	c->v5.host_buf_sz = cpu_to_le32(HIP4_TX_SIZE);
	c->v5.fw_buf_loc = cpu_to_le32(w->hip4_ref + HIP4_RX_OFFSET);
	c->v5.fw_buf_sz = cpu_to_le32(HIP4_RX_SIZE);
	c->v5.mif_fh_int_n = WLBT_BIT_HIP4_FROM_HOST;
	c->v5.mif_th_int_n = WLBT_BIT_HIP4_TO_HOST;
	c->v5.scbrd_loc = cpu_to_le32(w->hip4_control +
				      offsetof(struct wlbt_hip4_control, scoreboard));
	c->v5.q_num = cpu_to_le16(HIP4_Q_NUM);
	c->v5.q_len = cpu_to_le16(HIP4_Q_LEN);
	c->v5.q_idx_sz = cpu_to_le16(HIP4_Q_IDX_SIZE);

	for (i = 0; i < HIP4_Q_NUM; i++) {
		u32 q = cpu_to_le32(w->hip4_control +
				    offsetof(struct wlbt_hip4_control, q) +
				    i * sizeof(struct wlbt_hip4_q) +
				    offsetof(struct wlbt_hip4_q, array));

		c->v4.q_loc[i] = q;
		c->v5.q_loc[i] = q;
		c->v4.mif_th_int_n[i] = WLBT_BIT_HIP4_TO_HOST;
	}

	mib = load_mib ? wlbt_load_mib(w) : 0;
	if (mib) {
		c->v4.mib_loc = cpu_to_le32(w->hip4_ref + HIP4_MIB_OFFSET);
		c->v4.mib_sz = cpu_to_le32(mib);
		c->v5.mib_loc = cpu_to_le32(w->hip4_ref + HIP4_MIB_OFFSET);
		c->v5.mib_sz = cpu_to_le32(mib);
	}

	/* The peer reads the block over the bus as soon as the request lands. */
	wmb();

	/* Every segment is free until the peer hands one back on a release
	 * queue, so start with an empty scoreboard and an empty bitmap.
	 */
	w->hip4_pool[0].base = w->hip4_ref + HIP4_TX_OFFSET;
	w->hip4_pool[0].slots = HIP4_DAT_SLOTS;
	w->hip4_pool[0].next = 0;
	bitmap_zero(w->hip4_pool[0].used, HIP4_MAX_SLOTS);
	w->hip4_pool[1].base = w->hip4_ref + HIP4_TX_CTL_OFFSET;
	w->hip4_pool[1].slots = HIP4_CTL_SLOTS;
	w->hip4_pool[1].next = 0;
	bitmap_zero(w->hip4_pool[1].used, HIP4_MAX_SLOTS);

	memset_io(w->dram + w->hip4_control +
		  offsetof(struct wlbt_hip4_control, scoreboard), 0,
		  sizeof(__le32) * 256 +
		  HIP4_Q_NUM * sizeof(struct wlbt_hip4_q));
	/* Empty queues and a zero scoreboard before the peer starts using them. */
	wmb();

	dev_info(w->dev, "HIP4 at MIF+0x%x: control 0x%x, TX 0x%x, RX 0x%x, %u KiB\n",
		 w->hip4_ref, w->hip4_control, w->hip4_ref + HIP4_TX_OFFSET,
		 w->hip4_ref + HIP4_RX_OFFSET, HIP4_ALLOC_SIZE / 1024);

	return 0;
}

static u8 *wlbt_hip4_scb(struct exynos_wlbt *w)
{
	return w->dram + w->hip4_control +
	       offsetof(struct wlbt_hip4_control, scoreboard);
}

static u32 *wlbt_hip4_q_slot(struct exynos_wlbt *w, u32 q, u8 idx)
{
	return w->dram + w->hip4_control +
	       offsetof(struct wlbt_hip4_control, q) +
	       q * sizeof(struct wlbt_hip4_q) + idx * sizeof(__le32);
}

/*
 * A queue entry is free once the peer hands the segment back on the release
 * queue, so the slots are tracked with a bitmap rather than a cursor.
 */
static int wlbt_hip4_slot_get(struct exynos_wlbt *w, bool ctrl, u32 *ref)
{
	struct wlbt_hip4_pool *p = &w->hip4_pool[ctrl ? 1 : 0];
	unsigned int i;

	for (i = 0; i < p->slots; i++) {
		u32 slot = p->next;

		p->next = (p->next + 1) & (p->slots - 1);
		if (!test_and_set_bit(slot, p->used)) {
			*ref = p->base + slot * HIP4_SLOT_SIZE;
			return 0;
		}
	}

	return -ENOSPC;
}

static void wlbt_hip4_slot_put(struct exynos_wlbt *w, u32 ref)
{
	struct wlbt_hip4_pool *p;
	u32 off;

	for (off = 0; off < ARRAY_SIZE(w->hip4_pool); off++) {
		p = &w->hip4_pool[off];
		if (ref < p->base || ref >= p->base + p->slots * HIP4_SLOT_SIZE)
			continue;
		clear_bit((ref - p->base) / HIP4_SLOT_SIZE, p->used);
		return;
	}
}

/*
 * The peer checks every reference it is handed against the two buffers named in
 * the HIP4 configuration, and one that falls outside them is an assertion on its
 * side, so a segment that is not plainly inside those buffers is never queued.
 */
static bool wlbt_hip4_ref_valid(struct exynos_wlbt *w, u32 ref);
bool wlbt_bss_frame(const u8 *buf, size_t len, size_t *flen,
			   u16 *freq);

/*
 * Take a frame out of the mbulk at ref, if it holds one.  Returns the length
 * copied: the peer either fills the segment or leaves the frame in a chained
 * one, and both are worth trying before giving up.
 */
static size_t wlbt_mbulk_frame(struct exynos_wlbt *w, u32 ref, u8 *dst,
			       size_t cap)
{
	struct wlbt_mbulk mb;
	u16 len, head;

	if (!wlbt_hip4_ref_valid(w, ref))
		return 0;

	memcpy_fromio(&mb, w->dram + ref, sizeof(mb));
	len = le16_to_cpu(mb.len);
	head = le16_to_cpu(mb.head);
	if (len < 2 || len > HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - head || len > cap)
		return 0;

	memcpy_fromio(dst, w->dram + ref + HIP4_MBULK_SIZE + head, len);
	return len;
}

/*
 * Hand a frame over for delivery.  A single buffer plus schedule_work() lost
 * frames: scheduling an already pending work does nothing, and the work read
 * the buffer that later indications had already overwritten, so the
 * supplicant saw commits repeated and in the wrong order - which is what made
 * it restart a finished exchange and time out.
 */
static void wlbt_sae_queue(struct exynos_wlbt *w, const u8 *frame, size_t len,
			   s16 sig)
{
	unsigned long flags;
	unsigned int next;
	bool queued = false;

	spin_lock_irqsave(&w->sae_q_lock, flags);
	next = (w->sae_q_tail + 1) % ARRAY_SIZE(w->sae_q);
	if (next != w->sae_q_head) {
		memcpy(w->sae_q[w->sae_q_tail], frame, len);
		w->sae_q_len[w->sae_q_tail] = len;
		w->sae_q_sig[w->sae_q_tail] = sig;
		w->sae_q_tail = next;
		queued = true;
	}
	spin_unlock_irqrestore(&w->sae_q_lock, flags);

	if (queued)
		schedule_work(&w->sae_work);
	else
		dev_info(w->dev, "sae: delivery queue full, frame dropped\n");
}

/*
 * Where the peer keeps the frame an indication refers to is not announced.  It
 * can be in the segment itself, in a chained one (the way
 * MLME_RECEIVED_FRAME_IND hands its frames over), or behind the pointer the
 * indication carries.  Try all three and record which answered: this runs in
 * the drain, where nothing can be printed.
 */
static size_t wlbt_take_frame(struct exynos_wlbt *w, u32 ref,
			      const struct wlbt_mbulk *mb, const u8 *body)
{
	u16 len = le16_to_cpu(mb->len), head = le16_to_cpu(mb->head);
	size_t got = 0;

	w->sae_dbg[0] = mb->flag;
	w->sae_dbg[1] = le16_to_cpu(mb->sig_bufsz);
	w->sae_dbg[2] = le16_to_cpu(mb->dat_bufsz);
	w->sae_dbg[3] = len;
	w->sae_dbg[4] = head;

	if (len >= 24 && len <= HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - head)
		got = wlbt_mbulk_frame(w, ref, w->sae_frame, sizeof(w->sae_frame));

	if (!got && rx_chain_frame && (mb->flag & MBULK_F_CHAIN)) {
		u32 next = le32_to_cpu(mb->chain_next_offset);
		unsigned int n = 0;

		for (; n < MBULK_MAX_CHAIN && wlbt_hip4_ref_valid(w, next); n++) {
			struct wlbt_mbulk nb;

			got = wlbt_mbulk_frame(w, next, w->sae_frame,
					       sizeof(w->sae_frame));
			if (got) {
				w->sae_dbg[6] = next;
				break;
			}
			memcpy_fromio(&nb, w->dram + next, sizeof(nb));
			if (!(nb.flag & MBULK_F_CHAIN))
				break;
			next = le32_to_cpu(nb.chain_next_offset);
		}
		w->sae_chain = n;
	}

	if (!got && rx_ptr) {
		u32 ptr = get_unaligned_le32(body + 10);

		if (ptr != ref) {
			got = wlbt_mbulk_frame(w, ptr, w->sae_frame,
					       sizeof(w->sae_frame));
			if (got)
				w->sae_dbg[7] = ptr;
		}
	}

	return got;
}

/*
 * MLME_SYNCHRONISED_IND (0x231b) is how a WPA3 join starts: the peer hands up
 * the access point it is about to authenticate against, and the supplicant has
 * to be asked to run SAE.  The payload is a beacon or probe response - the
 * vendor's own handler overwrites its u.beacon.timestamp before feeding it to
 * cfg80211 - so it goes into the BSS table rather than to the data path.
 */
static void wlbt_sae_ind(struct exynos_wlbt *w, u32 ref, const struct wlbt_mbulk *mb,
			 const u8 *body)
{
	size_t got;

	w->n_sae_ind++;
	got = wlbt_take_frame(w, ref, mb, body);
	if (!got) {
		/* The peer sometimes names the access point without handing the
		 * frame over.  The join already said which one it is, and that is
		 * all the external-auth request needs, so ask the supplicant to run
		 * SAE rather than fall silent.
		 */
		if (!is_valid_ether_addr(w->connect_bssid) || !w->connect_ssid_len)
			return;
		w->sae_no_frame = true;
		wlbt_sae_queue(w, w->sae_frame, 0, 0);
		return;
	}

	w->sae_no_frame = false;
	wlbt_sae_queue(w, w->sae_frame, got,
		       (s16)get_unaligned_le16(body + 2));
}

static bool wlbt_hip4_ref_valid(struct exynos_wlbt *w, u32 ref)
{
	return ref >= w->hip4_ref + HIP4_TX_OFFSET &&
	       ref < w->hip4_ref + HIP4_RX_OFFSET + HIP4_RX_SIZE;
}

void wlbt_hip4_push(struct exynos_wlbt *w, u32 q, u32 ref)
{
	u8 *scb = wlbt_hip4_scb(w);
	u8 idx = readb(scb + wlbt_hip4_idx[q][0]);

	writel(ref, wlbt_hip4_q_slot(w, q, idx));
	/* The entry has to be visible before the index publishes it. */
	wmb();
	writeb((idx + 1) & (HIP4_Q_LEN - 1), scb + wlbt_hip4_idx[q][0]);
	/* Publish the index before the doorbell the peer reads. */
	wmb();

	writel(BIT(WLBT_BIT_HIP4_FROM_HOST), w->mbox + MBOX_INTGR1);
}

/*
 * Show the bookkeeping of every queue: the host half of the index pair and the
 * mirror the peer maintains.  When the peer wedges on a segment reference this
 * is what says whether it ran ahead, fell behind, or read a slot nobody wrote.
 */
void wlbt_hip4_state(struct exynos_wlbt *w)
{
	static const char *const names[HIP4_Q_NUM] = {
		"FH_CTL", "FH_DAT", "FH_RFB", "TH_CTL", "TH_DAT", "TH_RFB",
	};
	u8 *scb = wlbt_hip4_scb(w);
	unsigned int q;

	for (q = 0; q < HIP4_Q_NUM; q++) {
		u8 hw = readb(scb + wlbt_hip4_idx[q][0]);
		u8 hr = readb(scb + wlbt_hip4_idx[q][1]);
		u32 next = hw & (HIP4_Q_LEN - 1);

		dev_info(w->dev, "HIP4 %-6s host w %3u r %3u  next ref 0x%x\n",
			 names[q], hw, hr,
			 readl(wlbt_hip4_q_slot(w, q, next)));
	}
}

/*
 * The peer's scan validator rejects a request whose scan id is below 0x100
 * when the sender id carries the 0xc000 bits, which ours does, so scan ids
 * start at 0x100 and the upper byte names the interface - the same encoding
 * the vendor driver uses.  The information elements travel as payload: a
 * wildcard SSID, the scan timing element and the channel list, in that order.
 */

struct ieee80211_channel wlbt_2ghz_channels[] = {
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13 },
};


/*
 * The 5 GHz channels the vendor driver offers (cfg80211_ops.c,
 * slsi_5ghz_channels): UNII 1 to UNII 3, with no DFS flag of our own because
 * the peer decides what it may transmit on.  Channels sharing a primary channel
 * with a radar-detection band are scanned passively below.
 */
struct ieee80211_channel wlbt_5ghz_channels[] = {
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5180, .hw_value = 36 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5200, .hw_value = 40 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5220, .hw_value = 44 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5240, .hw_value = 48 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5260, .hw_value = 52 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5280, .hw_value = 56 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5300, .hw_value = 60 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5320, .hw_value = 64 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5500, .hw_value = 100 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5520, .hw_value = 104 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5540, .hw_value = 108 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5560, .hw_value = 112 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5580, .hw_value = 116 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5600, .hw_value = 120 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5620, .hw_value = 124 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5640, .hw_value = 128 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5660, .hw_value = 132 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5680, .hw_value = 136 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5700, .hw_value = 140 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5720, .hw_value = 144 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5745, .hw_value = 149 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5765, .hw_value = 153 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5785, .hw_value = 157 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5805, .hw_value = 161 },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5825, .hw_value = 165 },
};


/*
 * A channel the peer may only listen on: the radar detection bands, which
 * regulation forbids probing from.  The scan descriptor's policy byte says so
 * per channel, so one list can cover both kinds.
 */
bool wlbt_channel_passive(u16 freq)
{
	return freq >= 5260 && freq <= 5720;
}

int wlbt_mlme_add_scan(struct exynos_wlbt *w, u16 scan_id,
			      struct cfg80211_scan_request *req)
{
	static const u8 scan_mac[6] = { 0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc };
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 26];
	u8 dat[2 + 19 + 7 + 3 * (ARRAY_SIZE(wlbt_2ghz_channels) +
				 ARRAY_SIZE(wlbt_5ghz_channels))];
	struct wlbt_fapi_hdr hdr = { };
	unsigned int i, n = 0, n_chan = 0;
	u8 *c;

	/* Body: interface 0 for a scan, because a scan is not tied to a vif. */
	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(0, sig + sizeof(hdr) + 0);
	put_unaligned_le16(scan_id, sig + sizeof(hdr) + 2);
	put_unaligned_le16(2, sig + sizeof(hdr) + 4);	/* full scan */
	memcpy(sig + sizeof(hdr) + 6, scan_mac, sizeof(scan_mac));
	put_unaligned_le16(4, sig + sizeof(hdr) + 12);	/* real time results */

	/* Wildcard SSID: a broadcast scan always carries one. */
	dat[n++] = 0;
	dat[n++] = 0;

	/* Scan timing element, 19 bytes, body left at zero for a plain scan. */
	dat[n++] = 0xdd;
	dat[n++] = 0x11;
	dat[n++] = 0x00;
	dat[n++] = 0x16;
	dat[n++] = 0x32;
	dat[n++] = 0x01;
	dat[n++] = 0x01;
	memset(dat + n, 0, 12);
	n += 12;

	/* Channel list: the length byte starts at five and grows by three. */
	c = dat + n;
	c[0] = 0xdd;
	c[1] = 5;
	c[2] = 0x00;
	c[3] = 0x16;
	c[4] = 0x32;
	c[5] = 0x01;
	c[6] = 0x02;
	n += 7;

	/*
	 * The supplicant names the channels it wants, and both bands can be in
	 * one request, so take them from the request rather than from a fixed
	 * list.  A request without channels means every channel this wiphy has.
	 */
	if (req && req->n_channels) {
		for (i = 0; i < req->n_channels; i++) {
			u16 freq = req->channels[i]->center_freq;

			put_unaligned_le16(freq * 2, dat + n);
			dat[n + 2] = wlbt_channel_passive(freq) ? 0x01 : 0;
			n += 3;
			c[1] += 3;
			n_chan++;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(wlbt_2ghz_channels); i++) {
			put_unaligned_le16(wlbt_2ghz_channels[i].center_freq * 2,
					   dat + n);
			dat[n + 2] = 0;
			n += 3;
			c[1] += 3;
			n_chan++;
		}
		for (i = 0; i < ARRAY_SIZE(wlbt_5ghz_channels); i++) {
			u16 freq = wlbt_5ghz_channels[i].center_freq;

			put_unaligned_le16(freq * 2, dat + n);
			dat[n + 2] = wlbt_channel_passive(freq) ? 0x01 : 0;
			n += 3;
			c[1] += 3;
			n_chan++;
		}
	}

	if (!n_chan)
		return -EINVAL;

	hdr.id = cpu_to_le16(FAPI_MLME_ADD_SCAN_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 26, dat, n);
}

/*
 * A throwaway segment for the firmware reference to name.  It carries nothing
 * but a signal header, has one reference on it, and sits in the last data slot
 * so the ordinary allocator never hands it out.
 */
static u32 wlbt_hip4_scratch(struct exynos_wlbt *w, const void *data,
			     size_t data_len)
{
	/*
	 * The peer frees whatever the reference names, so each request gets its
	 * own segment from the tail of the data pool rather than all of them
	 * naming one buffer that is freed and then freed again.
	 */
	u32 slot = HIP4_DAT_SLOTS - 1 - w->hip4_scratch_seq % HIP4_SCRATCH_SLOTS;
	u32 ref = w->hip4_ref + HIP4_TX_OFFSET + slot * HIP4_SLOT_SIZE;
	struct wlbt_mbulk mb = { };
	void *m = w->dram + ref;

	if (hip4_ref_bare) {
		/*
		 * A plain zeroed buffer: the peer's range check skips a
		 * segment whose signal size is zero, and nothing else in the
		 * request points at the reference.
		 */
		memset_io(m, 0, HIP4_SLOT_SIZE);
	} else {
		mb.flag = MBULK_F_SIG | MBULK_F_OBOUND;
		mb.clas = MBULK_CLASS_FROM_HOST_DAT;
		mb.pid = MBULK_POOL_ID_DATA;
		mb.refcnt = 1;
		mb.sig_bufsz = cpu_to_le16(ALIGN(HIP4_SIG_RESERVED +
						 sizeof(struct wlbt_fapi_hdr), 4));
		mb.head = mb.sig_bufsz;
		/*
		 * The peer parses this segment: it reads a length and walks the
		 * contents, and a segment with no data parses to nothing, which
		 * it rejects.  Carry the payload the request carries so there is
		 * something to walk.
		 */
		if (data_len && data_len <= HIP4_SLOT_SIZE - HIP4_SIG_RESERVED -
						 sizeof(struct wlbt_fapi_hdr)) {
			mb.len = cpu_to_le16(data_len);
			mb.dat_bufsz = cpu_to_le16(data_len);
		}
		memset_io(m, 0, HIP4_SLOT_SIZE);
		memcpy_toio(m, &mb, sizeof(mb));
		if (le16_to_cpu(mb.len))
			memcpy_toio(m + HIP4_MBULK_SIZE + le16_to_cpu(mb.head),
				    data, data_len);
	}
	/* The descriptor must be in DRAM before the peer is handed the ref. */
	wmb();

	w->hip4_scratch_seq++;

	return ref;
}

/*
 * The peer owns two words of the configuration it is given: how much room it
 * wants reserved in front of and behind the frame of a unitdata request.  It
 * builds the 802.11 header there, so the reservation only matters for frames
 * that reach the air - a scan or a join carries information elements and works
 * without it, which is why this went unnoticed until a station transmitted.
 */
void wlbt_hip4_read_limits(struct exynos_wlbt *w)
{
	struct wlbt_hip4_control *c;

	if (!w->hip4_control)
		return;

	c = (struct wlbt_hip4_control *)(w->dram + w->hip4_control);
	if (le32_to_cpu(c->init.conf_ver) == 5) {
		w->tx_headroom = c->v5.unidat_req_headroom;
		w->tx_tailroom = c->v5.unidat_req_tailroom;
	} else {
		w->tx_headroom = c->v4.unidat_req_headroom;
		w->tx_tailroom = c->v4.unidat_req_tailroom;
	}

	dev_info(w->dev, "HIP4 unitdata limits: %u byte headroom, %u byte tailroom\n",
		 w->tx_headroom, w->tx_tailroom);
}

static unsigned int stall_watchdog;
module_param(stall_watchdog, uint, 0644);
MODULE_PARM_DESC(stall_watchdog, "Panic with every CPU's stack if a management send does not come back within a few seconds (0 off)");

static const char * const wlbt_stall_phases[] = {
	"nothing",
	"taking a segment from the pool",
	"writing the segment descriptor",
	"clearing the signal area",
	"writing the signal",
	"writing the frame",
	"handing the segment to the peer",
	"storing to the tx log",
};

/*
 * Breadcrumbs.  When the peer stops taking writes, the machine stops with no
 * console to show for it: the journal never flushes, there is no persistent
 * console region, and the supplicant's own log dies in its buffer.  These write
 * one line to a file on the root file system (NFS here) and sync it before
 * going on, so whatever happened last is still readable after the reset.  Only
 * call this from process context - the drain must never touch it.
 */
static char *trace_file;
module_param(trace_file, charp, 0644);
MODULE_PARM_DESC(trace_file, "File to append a synced line to at each step of a join (empty off)");

void wlbt_trace(struct exynos_wlbt *w, const char *fmt, ...)
{
	struct va_format vaf;
	struct file *f;
	char buf[192];
	va_list args;
	int n;

	if (!trace_file || !*trace_file)
		return;
	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	n = scnprintf(buf, sizeof(buf), "%u %pV\n", jiffies_to_msecs(jiffies), &vaf);
	va_end(args);

	f = filp_open(trace_file, O_WRONLY | O_CREAT | O_APPEND, 0644);
	if (IS_ERR(f))
		return;
	kernel_write(f, buf, n, &f->f_pos);
	vfs_fsync(f, 0);
	filp_close(f, NULL);
}

static char *stall_path = "/root/wlbt-stall.log";
module_param(stall_path, charp, 0644);
MODULE_PARM_DESC(stall_path, "File the stall watchdog leaves its finding in before panicking");

/*
 * Runs in process context, so unlike the timer it can write the finding where it
 * will survive the reset: the root file system here is NFS, and a panic leaves
 * nothing in the journal because journald never gets to flush.
 */
void wlbt_stall_report(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(work, struct exynos_wlbt, stall_report);
	unsigned int phase = READ_ONCE(w->stall_phase);
	char buf[200];
	int n;

	n = scnprintf(buf, sizeof(buf),
		      "stall: %s (phase %u), peer %s, state %s, tx %u rx %u, watchdog %u\n",
		      phase < ARRAY_SIZE(wlbt_stall_phases) ?
		      wlbt_stall_phases[phase] : "unknown", phase,
		      w->peer_dead ? "stopped" : "running",
		      wlbt_state_name(w->state), w->n_hip4_tx, w->n_hip4_rx,
		      w->n_wdog);
	if (stall_path && *stall_path) {
		struct file *f = filp_open(stall_path,
					   O_WRONLY | O_CREAT | O_APPEND, 0644);

		if (!IS_ERR(f)) {
			kernel_write(f, buf, n, &f->f_pos);
			vfs_fsync(f, 0);
			filp_close(f, NULL);
		} else {
			pr_emerg("wlbt: cannot open %s: %ld\n", stall_path,
				 PTR_ERR(f));
		}
	}
	pr_emerg("wlbt: %s", buf);
	panic("wlbt: the peer stopped taking writes");
}

void wlbt_stall_check(struct timer_list *t)
{
	struct exynos_wlbt *w = container_of(t, struct exynos_wlbt, stall_timer);
	unsigned long deadline;

	deadline = READ_ONCE(w->stall_deadline);
	if (deadline && time_after(jiffies, deadline)) {
		unsigned int phase = READ_ONCE(w->stall_phase);

		WRITE_ONCE(w->stall_deadline, 0);
		pr_emerg("wlbt: the send never came back, it was %s\n",
			 phase < ARRAY_SIZE(wlbt_stall_phases) ?
			 wlbt_stall_phases[phase] : "somewhere unexpected");
		if (stall_watchdog) {
			schedule_work(&w->stall_report);
			return;
		}
		/*
		 * Otherwise contain it: a management frame went out and nothing
		 * came back, which is what the peer does just before it stops
		 * reading its bus, and touching its memory again is what turns
		 * that into a dead machine.  Give the link up here instead; every
		 * path that would write to the peer then refuses.
		 */
		if (!w->peer_dead) {
			w->peer_dead = true;
			dev_err(w->dev,
				"marking the peer dead: a management send went unanswered\n");
		}
		return;
	}
	mod_timer(&w->stall_timer, jiffies + msecs_to_jiffies(500));
}

int wlbt_hip4_send(struct exynos_wlbt *w, bool ctrl, const void *sig,
			  size_t sig_len, const void *data, size_t data_len)
{
	struct wlbt_mbulk mb = { };
	u32 sig_bufsz, dat_bufsz, head, ref;
	unsigned long flags;
	void *m;
	int ret;

	if (!w->hip4_control)
		return -ENODEV;
	if (sig_len < sizeof(struct wlbt_fapi_hdr))
		return -EINVAL;

	/* The pools are shared with the drain that runs in interrupt context. */
	spin_lock_irqsave(&w->hip4_lock, flags);
	WRITE_ONCE(w->stall_phase, 1);
	ret = wlbt_hip4_slot_get(w, ctrl, &ref);
	if (ret) {
		spin_unlock_irqrestore(&w->hip4_lock, flags);
		return ret;
	}

	sig_bufsz = ALIGN(sig_len + HIP4_SIG_RESERVED, 4);
	/*
	 * A frame the peer will put on the air needs room for the header it
	 * builds in front of the payload, and for whatever it appends behind it.
	 * The reservation is expressed the way the peer's own allocator does:
	 * the data buffer covers headroom, payload and tailroom, and the head
	 * tells it where the payload starts.
	 */
	head = sig_bufsz;
	dat_bufsz = data_len;
	/*
	 * A signal whose payload the peer will put on the air - an EAPOL frame sent
	 * through MLME_SEND_FRAME_REQ just as much as a data frame - needs room for
	 * the header it builds in front of it and for whatever it appends behind
	 * it.  The vendor's hip4_skb_to_mbulk() says the same thing without looking
	 * at the queue: "if signal include payload, add headroom and tailroom".
	 */
	if (data_len) {
		head = sig_bufsz + w->tx_headroom;
		dat_bufsz = w->tx_headroom + data_len + w->tx_tailroom;
	}
	if (HIP4_MBULK_SIZE + head + data_len > HIP4_SLOT_SIZE) {
		spin_unlock_irqrestore(&w->hip4_lock, flags);
		return -ENOSPC;
	}

	mb.flag = MBULK_F_SIG | MBULK_F_OBOUND;
	mb.clas = ctrl ? MBULK_CLASS_FROM_HOST_CTL : MBULK_CLASS_FROM_HOST_DAT;
	mb.pid = ctrl ? MBULK_POOL_ID_CTRL : MBULK_POOL_ID_DATA;
	/*
	 * The peer's allocator ties the reference count to the data buffer,
	 * but its release path asserts when it finds zero, so the right value
	 * is not obvious from either side alone.  -1 keeps the allocator rule,
	 * anything else is written as given.
	 */
	mb.refcnt = sig_refcnt < 0 ? (data_len ? 1 : 0) : sig_refcnt;
	mb.dat_bufsz = cpu_to_le16(dat_bufsz);
	mb.sig_bufsz = cpu_to_le16(sig_bufsz);
	mb.len = cpu_to_le16(data_len);
	mb.head = cpu_to_le16(head);

	m = w->dram + ref;
	WRITE_ONCE(w->stall_phase, 2);
	memcpy_toio(m, &mb, sizeof(mb));
	WRITE_ONCE(w->stall_phase, 3);
	memset_io(m + HIP4_MBULK_SIZE, 0, HIP4_SIG_RESERVED);
	WRITE_ONCE(w->stall_phase, 4);
	memcpy_toio(m + HIP4_MBULK_SIZE + HIP4_SIG_RESERVED, sig, sig_len);
	if (data_len) {
		WRITE_ONCE(w->stall_phase, 5);
		memcpy_toio(m + HIP4_MBULK_SIZE + head, data, data_len);
	}
	if (sig_self_ref) {
		/*
		 * The peer validates this word against the transmit and receive
		 * bounds it kept from the HIP4 configuration, and then treats the
		 * segment it names as one of its own: it is handed back to the
		 * pool when the request is done.  So it has to name a disposable
		 * segment with a live reference count, not the frame the signal
		 * travels in - freeing that one leaves the frame double freed.
		 */
		__le32 self = cpu_to_le32(wlbt_hip4_scratch(w, data, data_len));

		memcpy_toio(m + HIP4_MBULK_SIZE + HIP4_SIG_RESERVED +
			    offsetof(struct wlbt_fapi_hdr, fw_reference),
			    &self, sizeof(self));
	}
	/* The whole segment has to be in DRAM before the peer sees the ref. */
	wmb();

	WRITE_ONCE(w->stall_phase, 6);
	wlbt_hip4_push(w, ctrl ? HIP4_Q_FH_CTL : HIP4_Q_FH_DAT, ref);
	w->n_hip4_tx++;
	if (ctrl && tx_log && w->tx) {
		struct wlbt_tx_rec *r = &w->tx[w->tx_next % WLBT_TX_SLOTS];

		r->t = ktime_get_ns();
		r->id = get_unaligned_le16(sig);
		r->len = min_t(size_t, sig_len, WLBT_TX_BODY);
		r->dlen = data ? min_t(size_t, data_len, WLBT_TX_DATA) : 0;
		memcpy(r->body, sig, r->len);
		if (r->dlen)
			memcpy(r->data, data, r->dlen);
		w->tx_next++;
		if (w->tx_used < WLBT_TX_SLOTS)
			w->tx_used++;
	}
	WRITE_ONCE(w->stall_phase, 0);
	spin_unlock_irqrestore(&w->hip4_lock, flags);

	return 0;
}

/*
 * The radio parameters live in the peer's MIB database, not in the buffers
 * above: service start only tells it where the hardware configuration files
 * are.  slsi_start() follows the service start with one MLME_SET_REQ per
 * slice of those files, leaving out the entries the firmware owns (PSID bit
 * 15) and keeping every signal inside the hostio limit (mgt.c:1679-1712).
 * A confirm carrying data means the peer rejected those PSIDs.
 */

void wlbt_mib_send_slice(struct exynos_wlbt *w, const u8 *mib,
				size_t len)
{
	struct wlbt_fapi_hdr hdr = { };
	u8 *buf;
	int ret;

	buf = kmalloc(sizeof(hdr) + 14 + len, GFP_KERNEL);
	if (!buf)
		return;

	hdr.id = cpu_to_le16(FAPI_MLME_SET_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(buf, &hdr, sizeof(hdr));
	memset(buf + sizeof(hdr), 0, 14);	/* vif and three spares */
	memcpy(buf + sizeof(hdr) + 14, mib, len);

	ret = wlbt_hip4_send(w, true, buf, sizeof(hdr) + 14 + len, NULL, 0);
	if (ret)
		dev_err(w->dev, "MIB slice of %zu bytes rejected: %d\n",
			len, ret);
	kfree(buf);

	/*
	 * The peer applies a slice asynchronously, so give it room to finish
	 * and to hand the segment back before the next one is queued: the
	 * vendor waits for the MLME_SET confirm here (mgt.c:1701).
	 */
	msleep(100);
	wlbt_hip4_drain(w);
}

/*
 * Walk the entry stream {u16 id, u16 len, data[len]} (padded to even) and hand
 * it over in slices the peer can take in one signal.
 */
int wlbt_download_mibs(struct exynos_wlbt *w)
{
	static const char *const names[] = {
		"mx140/mx140_wlan.hcf",
		"mx140/mx140_wlan_sw.hcf",
	};
	unsigned int slices = 0, entries = 0, skipped = 0;
	u8 *slice;
	size_t slice_len = 0;
	unsigned int i;

	if (mib_slice_max > MIB_SLICE_MAX)
		mib_slice_max = MIB_SLICE_MAX;
	slice = kmalloc(MIB_SLICE_MAX, GFP_KERNEL);
	if (!slice)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		const struct firmware *fw;
		size_t off, skip;

		if (request_firmware(&fw, names[i], w->dev)) {
			dev_warn(w->dev, "MIB %s not available\n", names[i]);
			continue;
		}

		skip = 0;
		if (fw->size > MIB_PROLOGUE_SIZE &&
		    get_unaligned_le16(fw->data + 6) == MIB_PROLOGUE_MARKER)
			skip = MIB_PROLOGUE_SIZE;

		for (off = skip; off + 4 <= fw->size;) {
			u16 psid = get_unaligned_le16(fw->data + off);
			size_t len = 4 + get_unaligned_le16(fw->data + off + 2);
			size_t padded = ALIGN(len, 2);

			if (off + padded > fw->size)
				break;

			if (psid & MIB_PSID_FW_ONLY) {
				skipped++;
				off += padded;
				continue;
			}

			if (slice_len + padded > mib_slice_max) {
				wlbt_mib_send_slice(w, slice, slice_len);
				slices++;
				slice_len = 0;
			}

			memcpy(slice + slice_len, fw->data + off, padded);
			slice_len += padded;
			entries++;
			off += padded;
		}

		release_firmware(fw);

		/* Each file ends its own slice, as the vendor does per file. */
		if (slice_len) {
			dev_info(w->dev, "MIB slice %u: %zu bytes\n",
				 slices, slice_len);
			wlbt_mib_send_slice(w, slice, slice_len);
			slices++;
			slice_len = 0;
		}
	}

	/* The golden host owned entries the firmware files do not carry. */
	for (i = 0; i < ARRAY_SIZE(wlbt_golden_mibs); i++) {
		size_t len = 4 + wlbt_golden_mibs[i].len;
		size_t padded = ALIGN(len, 2);

		if (slice_len + padded > mib_slice_max) {
			wlbt_mib_send_slice(w, slice, slice_len);
			slices++;
			slice_len = 0;
		}

		put_unaligned_le16(wlbt_golden_mibs[i].psid, slice + slice_len);
		put_unaligned_le16(wlbt_golden_mibs[i].len, slice + slice_len + 2);
		memcpy(slice + slice_len + 4, wlbt_golden_mibs[i].value,
		       wlbt_golden_mibs[i].len);
		memset(slice + slice_len + len, 0, padded - len);
		slice_len += padded;
		entries++;
	}

	kfree(slice);
	dev_info(w->dev,
		 "MIB download: %u entries (%u firmware owned) in %u slices\n",
		 entries, skipped, slices);

	return 0;
}

/* Take back the host segments the peer has finished with. */
static void wlbt_hip4_release(struct exynos_wlbt *w)
{
	u8 *scb = wlbt_hip4_scb(w);
	u8 q = HIP4_Q_FH_RFB;

	unsigned int n = 0;

	while (readb(scb + wlbt_hip4_idx[q][1]) !=
	       readb(scb + wlbt_hip4_idx[q][0])) {
		u8 idx = readb(scb + wlbt_hip4_idx[q][1]);

		/*
		 * This runs in interrupt context against memory the peer is
		 * still writing.  If the two indexes ever stop agreeing the
		 * loop below would spin forever with interrupts off, which no
		 * watchdog and no log can survive: the machine simply stops.
		 * Bound it instead, and contain the peer when the bound is hit.
		 */
		if (++n > 8 * HIP4_Q_LEN) {
			dev_err_ratelimited(w->dev,
					    "hip4: release stuck at %u/%u, peer contained\n",
					    readb(scb + wlbt_hip4_idx[q][1]),
					    readb(scb + wlbt_hip4_idx[q][0]));
			WRITE_ONCE(w->peer_dead, true);
			return;
		}

		wlbt_hip4_slot_put(w, readl(wlbt_hip4_q_slot(w, q, idx)));
		writeb((idx + 1) & (HIP4_Q_LEN - 1), scb + wlbt_hip4_idx[q][1]);
		/* Release the index before the segment is reused. */
		wmb();
	}
}

/*
 * One DEBUG_WORD12IND carries vif, module, sub module, a timestamp and twelve
 * words.  Counting the first two of those words by module is enough to say
 * which records a burst of traffic added.
 */
static void wlbt_log_count(struct exynos_wlbt *w, const u8 *data, size_t len)
{
	u16 module, sub, w0, w1;
	unsigned int i, slot;

	if (len < 14)
		return;
	module = get_unaligned_le16(data + 2);
	sub = get_unaligned_le16(data + 4);
	w0 = get_unaligned_le16(data + 10);
	w1 = get_unaligned_le16(data + 12);
	slot = (module * 31 + sub * 17 + w0 * 7 + w1) % WLBT_LOG_SLOTS;

	for (i = 0; i < 16; i++) {
		unsigned int s = (slot + i) % WLBT_LOG_SLOTS;

		if (!w->log_slots[s].count) {
			if (w->log_used >= WLBT_LOG_SLOTS)
				return;
			w->log_slots[s].module = module;
			w->log_slots[s].sub = sub;
			w->log_slots[s].word0 = w0;
			w->log_slots[s].word1 = w1;
			w->log_slots[s].count = 1;
			w->log_used++;
			return;
		}
		if (w->log_slots[s].module == module &&
		    w->log_slots[s].sub == sub &&
		    w->log_slots[s].word0 == w0 &&
		    w->log_slots[s].word1 == w1) {
			w->log_slots[s].count++;
			return;
		}
	}
}

static void wlbt_fault_record(struct exynos_wlbt *w, u16 id, const u8 *data,
			      size_t len)
{
	unsigned int slot = w->fault_next & (WLBT_FAULT_RING - 1);

	if (len > WLBT_FAULT_DATA)
		len = WLBT_FAULT_DATA;
	w->faults[slot].id = id;
	w->faults[slot].len = len;
	memcpy(w->faults[slot].data, data, len);
	w->fault_next++;
}

static void wlbt_id_record(struct exynos_wlbt *w, u16 id, const u8 *data,
			   size_t len)
{
	unsigned int i, slot = w->n_ids;

	for (i = 0; i < w->n_ids; i++)
		if (w->ids[i].id == id) {
			slot = i;
			break;
		}
	if (slot == w->n_ids) {
		if (w->n_ids >= WLBT_ID_SLOTS)
			return;
		w->n_ids++;
		memset(&w->ids[slot], 0, sizeof(w->ids[slot]));
		w->ids[slot].id = id;
	}

	if (len > WLBT_ID_BODY)
		len = WLBT_ID_BODY;
	w->ids[slot].len = len;
	if (len)
		memcpy(w->ids[slot].body, data, len);
	w->ids[slot].count++;
}

static void wlbt_frame_record(struct exynos_wlbt *w, u32 ref, u8 flag, u16 id,
			      const void *data, size_t len)
{
	unsigned int slot = w->frame_next & (WLBT_FRAME_RING - 1);

	memset(&w->frames[slot], 0, sizeof(w->frames[slot]));
	w->frames[slot].ref = ref;
	w->frames[slot].id = id;
	w->frames[slot].flag = flag;
	if (len > WLBT_FRAME_DATA)
		len = WLBT_FRAME_DATA;
	w->frames[slot].len = len;
	if (len)
		memcpy(&w->frames[slot].data, data, len);
	w->frame_next++;
	w->frame_total++;
}

/*
 * Print one segment of a scatter/gather frame.  The scan indications carry the
 * beacon in a chained segment, so the head alone says nothing about what was
 * received - but it is also the only place the information elements ever
 * appear, so they are worth dumping.
 */
static void wlbt_hip4_show_segment(struct exynos_wlbt *w, u32 ref,
				   const struct wlbt_mbulk *mb)
{
	/*
	 * Nothing is kept per segment: a chained frame would push everything
	 * else out of the ring, and the head record names the frame anyway.
	 */
	(void)mb;
	(void)ref;
}

/*
 * A received frame arrives as a unitdata indication whose payload is already an
 * ethernet frame.  It is copied into an skb and handed to the stack; nothing
 * else is needed because the peer did the 802.11 part.
 */
static void wlbt_rx_frame(struct exynos_wlbt *w, const void *data, size_t len);

/*
 * A received frame does not always sit in the indication that announces it: the
 * peer sends MLME_RECEIVED_FRAME_IND with an empty segment and puts the frame -
 * already ethernet, the 802.11 part is its business - in the segment chained
 * behind it.  That segment carries no FAPI header of its own, only a bulk
 * descriptor and the payload, so it has to be walked to be seen at all.
 */
static void wlbt_rx_chain(struct exynos_wlbt *w, u32 ref)
{
	unsigned int n;

	for (n = 0; n < MBULK_MAX_CHAIN; n++) {
		struct wlbt_mbulk mb;
		u16 len, head;

		memcpy_fromio(&mb, w->dram + ref, sizeof(mb));
		len = le16_to_cpu(mb.len);
		head = le16_to_cpu(mb.head);
		if (len && len <= HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - head)
			wlbt_rx_frame(w, (const u8 __iomem *)w->dram + ref +
				      HIP4_MBULK_SIZE + head, len);

		if (!(mb.flag & MBULK_F_CHAIN))
			break;
		ref = le32_to_cpu(mb.chain_next_offset);
		if (!wlbt_hip4_ref_valid(w, ref))
			break;
	}
}

/*
 * The answer to MLME_GET travels the way a received frame does: nothing in the
 * segment that carries the confirmation, the values in the data area.  Keep them
 * so a MIB counter can actually be read.
 */
static void wlbt_mib_capture(struct exynos_wlbt *w, u32 ref)
{
	unsigned int n;
	u32 len = 0;

	for (n = 0; n < MBULK_MAX_CHAIN && len < sizeof(w->mibresp); n++) {
		struct wlbt_mbulk mb;
		u16 l, head;

		memcpy_fromio(&mb, w->dram + ref, sizeof(mb));
		l = le16_to_cpu(mb.len);
		head = le16_to_cpu(mb.head);
		if (l && l <= HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - head) {
			if (l > sizeof(w->mibresp) - len)
				l = sizeof(w->mibresp) - len;
			memcpy_fromio(w->mibresp + len,
				      (const u8 __iomem *)w->dram + ref +
				      HIP4_MBULK_SIZE + head, l);
			len += l;
		}
		if (!(mb.flag & MBULK_F_CHAIN))
			break;
		ref = le32_to_cpu(mb.chain_next_offset);
	}

	w->mibresp_len = len;
}

static void wlbt_rx_frame(struct exynos_wlbt *w, const void *data, size_t len)
{
	struct sk_buff *skb;

	if (len < ETH_HLEN || len > 4096)
		return;

	skb = netdev_alloc_skb_ip_align(w->netdev, len);
	if (!skb) {
		w->n_tx_dropped++;
		return;
	}

	memcpy_fromio(skb_put(skb, len), data, len);
	skb->protocol = eth_type_trans(skb, w->netdev);
	skb->ip_summed = CHECKSUM_NONE;
	if (!w->port_authorized)
		dev_info(w->dev, "rx: %zu byte frame proto 0x%04x\n",
			 len, ntohs(skb->protocol));
	w->netdev->stats.rx_packets++;
	w->netdev->stats.rx_bytes += len;
	netif_rx(skb);
}

static void wlbt_hip4_show(struct exynos_wlbt *w, u32 q)
{
	u8 *scb = wlbt_hip4_scb(w);
	u8 ridx = readb(scb + wlbt_hip4_idx[q][1]);
	u8 widx = readb(scb + wlbt_hip4_idx[q][0]);
	u32 chain[MBULK_MAX_CHAIN];
	unsigned int n = 0, n_chain, i;

	while (ridx != widx && n < HIP4_Q_LEN) {
		u32 ref = readl(wlbt_hip4_q_slot(w, q, ridx));
		struct wlbt_mbulk mb;
		void *m;

		if (!wlbt_hip4_ref_valid(w, ref)) {
			dev_warn(w->dev, "HIP4 q%u[%u]: bogus ref 0x%x\n",
				 q, ridx, ref);
			ridx = (ridx + 1) & (HIP4_Q_LEN - 1);
			continue;
		}

		m = w->dram + ref;
		memcpy_fromio(&mb, m, sizeof(mb));
		if (hip4_log)
			dev_info(w->dev,
				 "HIP4 q%u[%u]: ref 0x%x flag 0x%02x clas %u pid %u sig %u len %u\n",
				 q, ridx, ref, mb.flag, mb.clas, mb.pid,
				 le16_to_cpu(mb.sig_bufsz), le16_to_cpu(mb.len));

		if (le16_to_cpu(mb.sig_bufsz) >= HIP4_SIG_RESERVED +
						  sizeof(struct wlbt_fapi_hdr)) {
			u8 body[HIP4_SIG_DUMP] = { };
			size_t len = le16_to_cpu(mb.sig_bufsz) -
				     HIP4_SIG_RESERVED;
			struct wlbt_fapi_hdr hdr;

			memcpy_fromio(&hdr, m + HIP4_MBULK_SIZE +
				      HIP4_SIG_RESERVED, sizeof(hdr));
			/*
			 * One line per frame is enough to stall the machine when
			 * the console is a serial port and this runs in the
			 * drain with interrupts off: hundreds of frames a second
			 * each wait for the UART, and everything behind them
			 * waits too.  Only print when it is asked for.
			 */
			if (hip4_log)
				dev_info(w->dev,
					 "  fapi id 0x%04x receiver 0x%04x sender 0x%04x ref 0x%x\n",
					 le16_to_cpu(hdr.id),
					 le16_to_cpu(hdr.receiver_pid),
					 le16_to_cpu(hdr.sender_pid),
					 le32_to_cpu(hdr.fw_reference));

			/*
			 * The signal body has to be in memory before any id specific
			 * handling looks at it: every branch below reads it, and
			 * reading it here rather than at the end is what keeps a
			 * connect result or a rejected confirmation from being
			 * taken out of whatever the previous frame left on the stack.
			 */
			memcpy_fromio(body, m + HIP4_MBULK_SIZE +
				      HIP4_SIG_RESERVED + sizeof(hdr),
				      len > sizeof(hdr) ?
				      min_t(size_t, len - sizeof(hdr), sizeof(body)) : 0);

			/*
			 * MA_UNITDATA_IND carries a received frame: the payload
			 * starts after the bulk header and runs for the length
			 * the segment records.
			 */
			if (le16_to_cpu(hdr.id) == FAPI_MA_UNITDATA_IND) {
				size_t dlen = le16_to_cpu(mb.len);
				u16 off = le16_to_cpu(mb.head);

				dev_info(w->dev,
					 "unitdata: len %zu head %u\n", dlen, off);

				if (dlen && dlen <= HIP4_SLOT_SIZE - HIP4_MBULK_SIZE - off)
					wlbt_rx_frame(w, m + HIP4_MBULK_SIZE + off,
						      dlen);
			}

			/*
			 * MLME_SYNCHRONISED_IND is an authentication frame the
			 * peer exchanged with the access point on its own: it
			 * is how WPA3 asks the host to run SAE.  The frame is in
			 * the data area, the word after the interface is the
			 * peer's signal for it.  Copy it out here - this runs in
			 * the drain - and let the work item do the talking.
			 */
			if (le16_to_cpu(hdr.id) == FAPI_MLME_SYNCHRONISED_IND)
				wlbt_sae_ind(w, ref, &mb, body);

			/*
			 * The received-frame indication is taken apart for its
			 * trailing word: if the peer keeps the frame rather than
			 * sending it, that word is the only pointer to it.
			 */
			/*
			 * MLME_RECEIVED_FRAME_IND is how the peer hands over a
			 * frame it received but did not fill into the
			 * indication itself.
			 */
			/*
			 * A received frame is either user data (an 802.3 frame) or
			 * a management frame the peer took off the air.  The second
			 * kind matters during a WPA3 join: the access point's half
			 * of the SAE exchange arrives this way and belongs to the
			 * supplicant, not to the networking stack.
			 */
			if (!rx_ind_off && le16_to_cpu(hdr.id) == 0x230d &&
			    len >= sizeof(hdr) + 6) {
				u16 desc = get_unaligned_le16(body + 2);

				dev_info(w->dev,
					 "ind230d: desc 0x%04x mblen %u\n",
					 desc, le16_to_cpu(mb.len));

				/*
				 * The vendor's indication carries the frame inline, with
				 * the segment's length field set to the frame length; ours
				 * has been seen with an empty segment and the frame in the
				 * one chained behind it.  take_frame() tries both, so the
				 * only thing that has to hold here is that there is
				 * something to take.
				 */
				/*
				 * The peer announces an 802.3 frame with an empty
				 * segment - those are the EAPOL-Key messages of a
				 * WPA2 or WPA3 handshake, one per retry - so the
				 * length test must not be the thing that decides
				 * whether the frame is worth taking.
				 */
				if (desc == 0x0001 ||
				    desc == FAPI_DATAUNIT_IEEE802_11_FRAME ||
				    le16_to_cpu(mb.len) >= 24) {
					size_t got = wlbt_take_frame(w, ref, &mb, body);

					dev_info(w->dev,
						 "ind230d: got %zu flag %08x sigb %u datb %u head %u\n",
						 got, w->sae_dbg[0], w->sae_dbg[1],
						 w->sae_dbg[2], w->sae_dbg[4]);

					/*
					 * A recovered 802.3 frame is a received
					 * frame like any other: EAPOL is just a
					 * protocol the stack routes to the
					 * supplicant.
					 */
					if (got >= ETH_HLEN && desc == 0x0001)
						wlbt_rx_frame(w, w->sae_frame, got);

					w->n_mgmt_rx++;
					if (got >= 24 &&
					    ((w->sae_frame[0] >> 2) & 0x3) == 2)
						dev_info(w->dev,
							 "ind: data frame %zu bytes fc %02x %02x\n",
							 got, w->sae_frame[0],
							 w->sae_frame[1]);
					if (got >= 24 &&
					    ((w->sae_frame[0] >> 4) & 0xf) ==
					    (IEEE80211_STYPE_AUTH >> 4)) {
						wlbt_sae_queue(w, w->sae_frame, got, 0);
						/*
						 * No continue: the segment still has to be
						 * handed back and the index advanced below,
						 * or this loop reads the same entry forever.
						 */
					}
				}
			}

			/*
			 * MLME_SCAN_IND (0x2300) carries one result the peer found:
			 * the scan id, an rssi and the 802.11 frame, usually in a
			 * chained segment.  Copy that frame read-only - the segment
			 * stays the peer's and is released here as usual - and hand
			 * it to cfg80211 with the rssi the peer measured.  The scan
			 * sweep in wlbt_scan_collect() stays as a fallback for a peer
			 * that sends no indications.
			 */
			if (!rx_ind_off && le16_to_cpu(hdr.id) == 0x2300 &&
			    len >= sizeof(hdr) + 6) {
				u8 fbuf[512];
				size_t got = 0;
				u16 flen = le16_to_cpu(mb.len);
				u16 fhead = le16_to_cpu(mb.head);
				s16 rssi = (s16)get_unaligned_le16(body + 4);

				if (flen >= 24 && flen <= sizeof(fbuf) &&
				    (u64)ref + HIP4_MBULK_SIZE + fhead + flen <=
				    w->mem_size) {
					memcpy_fromio(fbuf,
						      w->dram + ref +
						      HIP4_MBULK_SIZE + fhead,
						      flen);
					got = flen;
				} else if (mb.flag & MBULK_F_CHAIN) {
					got = wlbt_mbulk_frame(w,
						le32_to_cpu(mb.chain_next_offset),
						fbuf, sizeof(fbuf));
				}

				if (got >= 24 && got <= 512) {
					unsigned long flags;
					u8 slot;

					spin_lock_irqsave(&w->scan_rep_lock, flags);
					slot = w->scan_rep_head;
					if (((slot + 1) & 7) != w->scan_rep_tail) {
						memcpy(w->scan_rep_frame[slot], fbuf,
						       got);
						w->scan_rep_len[slot] = got;
						w->scan_rep_rssi[slot] = rssi;
						w->scan_rep_head = (slot + 1) & 7;
						schedule_work(&w->scan_rep_work);
					}
					spin_unlock_irqrestore(&w->scan_rep_lock,
							       flags);
				}
			}

			if (rx_chain_probe && le16_to_cpu(hdr.id) == 0x230d &&
			    (mb.flag & MBULK_F_CHAIN)) {
				u32 nxt = le32_to_cpu(mb.chain_next_offset);

				if (wlbt_hip4_ref_valid(w, nxt)) {
					memcpy_fromio(w->chain_probe, w->dram + nxt,
						      sizeof(w->chain_probe));
					w->chain_probe_ref = nxt;
					w->chain_probe_len = sizeof(w->chain_probe);
				}
			}

			if (rx_chain && !rx_ind_off &&
			    le16_to_cpu(hdr.id) == 0x230d &&
			    (mb.flag & MBULK_F_CHAIN))
				wlbt_rx_chain(w, ref);

			if (le16_to_cpu(hdr.id) == 0x230d) {
				size_t blen = len > sizeof(hdr) ?
					      len - sizeof(hdr) : 0;

				/* Whole segment, so the place the frame sits in can be
				 * read off line by line instead of guessed at.
				 */
				memcpy_fromio(w->rxind_raw, m, HIP4_SLOT_SIZE);
				w->rxind_raw_len = HIP4_SLOT_SIZE;

				blen = min_t(size_t, blen, sizeof(w->peer_rxind));
				memcpy(w->peer_rxind, body, blen);
				w->peer_rxind_len = blen;
				w->peer_rxind_geom[0] = le16_to_cpu(mb.sig_bufsz);
				w->peer_rxind_geom[1] = le16_to_cpu(mb.dat_bufsz);
				w->peer_rxind_geom[2] = le16_to_cpu(mb.len);
				w->peer_rxind_geom[3] = le16_to_cpu(mb.head);
				w->n_rxind++;
			}

			/*
			 * A confirmation carries the result code first; when it
			 * is not success the request was refused, and knowing
			 * which one is worth having.  It goes into the ring
			 * rather than the log: this runs with the queue lock
			 * held and interrupts off.
			 */
			if ((le16_to_cpu(hdr.id) & 0xff00) == 0x2100 &&
			    len >= sizeof(hdr) + 2 &&
			    get_unaligned_le16(body))
				wlbt_fault_record(w, le16_to_cpu(hdr.id), body,
						  min_t(size_t, len - sizeof(hdr),
							WLBT_FAULT_DATA));

			/*
			 * MLME_CONNECT_CFM is the peer saying it took the join
			 * request.  For an open network the vendor answers the
			 * connect at that point, not after the indication, and
			 * without the answer the peer drops the link again a few
			 * seconds after associating.
			 */
			if (le16_to_cpu(hdr.id) == 0x210b) {
				w->peer_connect_cfm = true;
				schedule_work(&w->peer_work);
			}

			if (le16_to_cpu(hdr.id) == 0x2101)
				wlbt_mib_capture(w, ref);

			/* MLME_CONNECT_IND is {le16 vif, le16 result} and is the
			 * only place the outcome of a join shows up.
			 */
			if (le16_to_cpu(hdr.id) == 0x2303 &&
			    len >= sizeof(hdr) + 4) {
				w->connect_status = get_unaligned_le16(body + 2);
				w->connect_reported = true;
				w->peer_connect_ind = true;
				schedule_work(&w->peer_work);
			}

			/*
			 * MLME_CONNECTED_IND is {le16 vif, le16 aid}: the link is
			 * up and the peer wants the association id confirmed.
			 */
			if (le16_to_cpu(hdr.id) == 0x2304) {
				if (len >= sizeof(hdr) + 4)
					w->peer_aid = get_unaligned_le16(body + 2);
				w->peer_connected_ind = true;
				schedule_work(&w->peer_work);
			}

			/* MLME_DISCONNECTED_IND is {le16 vif, u8 bssid[6], le16
			 * reason}: the link the peer had is gone.
			 */
			if (le16_to_cpu(hdr.id) == 0x2309) {
				if (len >= sizeof(hdr) + 10)
					w->peer_disconnect_reason =
						get_unaligned_le16(body + 8);
				w->peer_disconnected_ind = true;
				schedule_work(&w->peer_work);
			}

			if (len > sizeof(body))
				len = sizeof(body);
			wlbt_id_record(w, le16_to_cpu(hdr.id), body,
				       len > sizeof(hdr) ? len - sizeof(hdr) : 0);

			/*
			 * The peer's own log frames would fill the ring on their
			 * own, and it is the requests and indications that matter.
			 */
			if (le16_to_cpu(hdr.id) == 0x8301) {
				wlbt_log_count(w, body,
					       len > sizeof(hdr) ?
					       len - sizeof(hdr) : 0);
				if (log_module && len >= sizeof(hdr) + 4 &&
				    get_unaligned_le16(body + 2) == log_module)
					wlbt_fault_record(w, le16_to_cpu(hdr.id),
							  body,
							  min_t(size_t,
								len - sizeof(hdr),
								WLBT_FAULT_DATA));
			}

			if (le16_to_cpu(hdr.id) == 0x8301 ||
			    le16_to_cpu(hdr.id) == 0x8302)
				wlbt_fault_record(w, le16_to_cpu(hdr.id), body,
						  len > sizeof(hdr) ?
						  len - sizeof(hdr) : 0);

			if (hip4_log && le16_to_cpu(hdr.id) != 0x8301)
				dev_info(w->dev,
					 "ind: id 0x%04x len %zu flag %02x\n",
					 le16_to_cpu(hdr.id), len,
					 (unsigned int)mb.flag);
			/*
			 * The result of a key install and of an EAPOL
			 * transmission are what decide whether a link survives,
			 * and the peer reports both inside the confirmation
			 * payload rather than in the header.
			 */
			if (hip4_log && len > sizeof(hdr) &&
			    (le16_to_cpu(hdr.id) == 0x2118 ||
			     le16_to_cpu(hdr.id) == 0x2110 ||
			     le16_to_cpu(hdr.id) == 0x230c ||
			     le16_to_cpu(hdr.id) == 0x210e))
				dev_info(w->dev, "ind: body %*phN\n",
					 (int)min_t(size_t, len - sizeof(hdr), 16),
					 body);
			if (hip4_log && le16_to_cpu(hdr.id) != 0x8301)
				wlbt_frame_record(w, ref, mb.flag,
						  le16_to_cpu(hdr.id), body,
						  len > sizeof(hdr) ?
						  len - sizeof(hdr) : 0);
		}

		/*
		 * Hand every segment back so the peer can reuse it.  A
		 * scatter/gather frame is a chain: the head carries the
		 * chain_next_offset of the next segment and every segment but
		 * the last says it is chained, so the peer needs all of them
		 * or it walks into a segment it already recycled.
		 */
		chain[0] = ref;
		n_chain = 1;
		while (n_chain < MBULK_MAX_CHAIN && (mb.flag & MBULK_F_CHAIN)) {
			struct wlbt_mbulk prev;
			u32 next;

			memcpy_fromio(&prev, w->dram + chain[n_chain - 1],
				      sizeof(prev));
			next = le32_to_cpu(prev.chain_next_offset);
			/*
			 * The link is only worth following once it is known to
			 * name our buffers.  A last segment that still advertises
			 * a successor usually terminates with zero, and passing
			 * that on is what makes the peer assert when it drains
			 * the queue of returned segments.
			 */
			if (!wlbt_hip4_ref_valid(w, next))
				break;
			memcpy_fromio(&mb, w->dram + next, sizeof(mb));
			chain[n_chain++] = next;
			wlbt_hip4_show_segment(w, next, &mb);
		}
		if (hip4_return)
			for (i = 0; i < n_chain; i++)
				wlbt_hip4_push(w, HIP4_Q_TH_RFB, chain[i]);

		ridx = (ridx + 1) & (HIP4_Q_LEN - 1);
		writeb(ridx, scb + wlbt_hip4_idx[q][1]);
		/* Only advance once the segment is handed back. */
		wmb();
		w->n_hip4_rx++;
		n++;
	}
}

void wlbt_hip4_drain(struct exynos_wlbt *w)
{
	unsigned long flags;

	if (!w->hip4_control)
		return;

	if (rx_off)
		return;

	/*
	 * The queues are consumed by the mailbox interrupt and by hand from
	 * debugfs, and no entry may be read twice: the peer marks a segment it
	 * has been handed back as freed, so receiving the same one again is an
	 * assertion on its side.  Holding the lock with interrupts off is what
	 * keeps the two contexts off the same index.
	 */
	spin_lock_irqsave(&w->hip4_lock, flags);
	wlbt_hip4_release(w);
	wlbt_hip4_show(w, HIP4_Q_TH_CTL);
	wlbt_hip4_show(w, HIP4_Q_TH_DAT);
	spin_unlock_irqrestore(&w->hip4_lock, flags);
}


/*
 * One stream is a ring of fixed-size cells plus the two cursors the peer reads
 * (cpacket_buffer.c:15-45).  An input stream is pre-filled with 0xff like the
 * vendor does, so a mis-parsed header is obvious rather than looking like a
 * valid message.
 */
static int wlbt_stream_init(struct exynos_wlbt *w, struct wlbt_stream *s,
			    const char *name, u32 num_packets, u32 packet_size,
			    u8 read_bit, u8 write_bit, bool to_ap)
{
	s->name = name;
	s->num_packets = num_packets;
	s->packet_size = packet_size;
	s->read_bit = read_bit;
	s->write_bit = write_bit;

	s->buf = wlbt_heap_alloc(w, num_packets * packet_size);
	s->ridx = wlbt_heap_alloc(w, 4);
	s->widx = wlbt_heap_alloc(w, 4);
	if (!s->buf || !s->ridx || !s->widx)
		return -ENOMEM;

	if (to_ap)
		memset_io(w->dram + s->buf, 0xff, num_packets * packet_size);

	return 0;
}

static void wlbt_stream_serialise(struct wlbt_stream *s,
				  struct wlbt_stream_conf *conf)
{
	conf->buf_conf.buffer_loc = s->buf;
	conf->buf_conf.num_packets = s->num_packets;
	conf->buf_conf.packet_size = s->packet_size;
	conf->buf_conf.read_index_loc = s->ridx;
	conf->buf_conf.write_index_loc = s->widx;

	/* Verbatim, as mif_stream_config_serialise() does: the two indices
	 * already carry opposite direction on the two sides.
	 */
	conf->read_bit_idx = s->read_bit;
	conf->write_bit_idx = s->write_bit;
}

/**
 * wlbt_stream_read() - take bytes out of a stream
 * @bytes: how much to read; the result is rounded up to whole cells
 *
 * Mirrors cpacketbuffer_read(): the cursors are cell counts, the read advances
 * the read cursor and tells the peer with a doorbell so it may reuse the cells.
 *
 * Return: bytes actually copied.
 */
u32 wlbt_stream_read(struct exynos_wlbt *w, struct wlbt_stream *s,
			    void *dst, u32 bytes)
{
	u32 avail, cells, ridx, widx, first, off;

	if (!w->dram || !bytes)
		return 0;

	ridx = readl(w->dram + s->ridx);
	widx = readl(w->dram + s->widx);
	if (ridx >= s->num_packets || widx >= s->num_packets)
		return 0;

	avail = (widx + s->num_packets - ridx) % s->num_packets;
	if (!avail)
		return 0;

	cells = DIV_ROUND_UP(bytes, s->packet_size);
	if (cells > avail)
		cells = avail;
	bytes = min(bytes, cells * s->packet_size);

	off = s->buf + ridx * s->packet_size;
	first = min(cells, s->num_packets - ridx) * s->packet_size;
	memcpy_fromio(dst, w->dram + off, min(first, bytes));
	if (bytes > first)
		memcpy_fromio(dst + first, w->dram + s->buf, bytes - first);

	writel((ridx + cells) % s->num_packets, w->dram + s->ridx);

	/* Tell the peer those cells are free again (mifstream.c:163). */
	writel(BIT(s->read_bit), w->mbox + MBOX_INTGR1);

	return bytes;
}

/*
 * A short write still costs a whole cell: cpacketbuffer_write_gather() pads
 * the tail and advances the index once, so the peer never sees a half cell.
 */
bool wlbt_stream_write(struct exynos_wlbt *w, struct wlbt_stream *s,
			      const void *src, u32 bytes)
{
	const u8 *p = src;
	u8 *cell;
	u32 cells, i, ridx, widx, left;

	if (!w->dram || !bytes)
		return false;

	ridx = readl(w->dram + s->ridx);
	widx = readl(w->dram + s->widx);
	if (ridx >= s->num_packets || widx >= s->num_packets)
		return false;

	cells = DIV_ROUND_UP(bytes, s->packet_size);
	if (cells >= s->num_packets)
		return false;

	cell = kzalloc(s->packet_size, GFP_KERNEL);
	if (!cell)
		return false;

	for (i = 0; i < cells; i++) {
		left = bytes - i * s->packet_size;
		memcpy(cell, p + i * s->packet_size, min(left, s->packet_size));
		memcpy_toio(w->dram + s->buf + ((widx + i) % s->num_packets) *
			    s->packet_size, cell, s->packet_size);
	}
	kfree(cell);

	/* memcpy_toio() orders the copies against the index store below. */
	writel((widx + cells) % s->num_packets, w->dram + s->widx);

	/* Kick the peer the way mif_stream_write_gather() does. */
	writel(BIT(s->write_bit), w->mbox + MBOX_INTGR1);

	return true;
}

static const char *wlbt_fw_level(u8 level)
{
	switch (level) {
	case 0:
		return KERN_ERR;	/* firmware errors */
	case 1:
		return KERN_WARNING;
	case 2:
		return KERN_NOTICE;
	case 3:
		return KERN_INFO;
	default:
		return KERN_DEBUG;
	}
}

/*
 * Drain the firmware's log stream and print it.  Phase 4 payloads are plain
 * NUL-terminated strings; phase 5 replaces repeated strings with indices into a
 * table the firmware carries, which needs the string map from mxlog.c that has
 * not been ported, so those are reported as unknown for now.
 */
void wlbt_log_drain(struct exynos_wlbt *w)
{
	char text[260];
	u32 header, sync, len, level, got;

	if (rx_off)
		return;

	while (wlbt_stream_read(w, &w->log, &header, sizeof(header)) ==
	       sizeof(header)) {
		sync = header >> 16;
		level = (header >> 8) & 0xff;

		if (sync == MXLOG_SYNC_PHASE_4) {
			len = header & 0xff;
		} else if (sync == MXLOG_SYNC_PHASE_5) {
			len = (header & 0xff) * 4;
		} else {
			/* Not a header at all: the stream is out of step. */
			w->n_fwlog_bad++;
			continue;
		}

		if (!len)
			continue;

		/* The payload has to be consumed either way, otherwise the
		 * next read starts in the middle of this message.
		 */
		got = wlbt_stream_read(w, &w->log, text, min(len, sizeof(text) - 1));
		text[got] = '\0';

		if (sync == MXLOG_SYNC_PHASE_4) {
			w->n_fwlog++;
			if (fw_log)
				dev_printk(wlbt_fw_level(level), w->dev,
					   "firmware: %s\n", text);
		} else {
			w->n_fwlog_bad++;
			/* Phase 5 compresses repeated strings against a table
			 * the firmware carries; until that table is ported, show
			 * the raw words so the messages can still be read.
			 */
			if (w->n_fwlog_bad <= 64) {
				const __le32 *args = (const __le32 *)text;
				u32 nwords = got / 4;

				dev_info(w->dev, "fwlog5 lvl %u off 0x%x argc %u:%s\n",
					 level, nwords > 1 ? le32_to_cpu(args[1]) : 0,
					 nwords > 2 ? nwords - 2 : 0,
					 nwords > 2 ? "" : " (no args)");
				if (nwords > 2)
					dev_info(w->dev, "fwlog5 args:%*phN\n",
						 (int)((nwords - 2) * 4), text + 8);
			}
		}
	}
}

/*
 * Build the configuration the peer reads from mailbox 1: three streams (the
 * management channel in both directions and the log), the interrupt bits that
 * signal them, and an empty common HCF - this board ships no `conf/` directory
 * with the firmware, so the vendor runs with an empty one too
 * (mxfwconfig.c:283, "HW file is optional").
 */
int wlbt_build_mxconf(struct exynos_wlbt *w)
{
	struct wlbt_mxconf config;
	struct wlbt_mxconf *conf = &config;
	u32 offset;
	int ret;

	w->heap_next = ALIGN(w->fw_runtime_length, 4);
	if (w->heap_next >= w->mem_size) {
		dev_err(w->dev, "no room for a heap after the firmware\n");
		return -ENOSPC;
	}

	ret = wlbt_stream_init(w, &w->mgmt_in, "mgmt-in", MGMT_NUM_PACKETS,
			       MGMT_PACKET_SIZE, WLBT_BIT_MGMT_IN_READ,
			       WLBT_BIT_MGMT_IN_WRITE, true);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->mgmt_out, "mgmt-out", MGMT_NUM_PACKETS,
			       MGMT_PACKET_SIZE, WLBT_BIT_MGMT_OUT_READ,
			       WLBT_BIT_MGMT_OUT_WRITE, false);
	if (ret)
		return ret;
	/*
	 * The firmware validates the four GDB streams during bring-up and
	 * refuses to continue if one of them is missing, even though nothing
	 * here uses a debugger (0x801172fc in the image, "set up transports").
	 */
	ret = wlbt_stream_init(w, &w->gdb_r4_in, "gdb-r4-in", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_R4_IN_READ,
			       WLBT_BIT_GDB_R4_IN_WRITE, true);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->gdb_r4_out, "gdb-r4-out", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_R4_OUT_READ,
			       WLBT_BIT_GDB_R4_OUT_WRITE, false);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->gdb_m4_in, "gdb-m4-in", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_M4_IN_READ,
			       WLBT_BIT_GDB_M4_IN_WRITE, true);
	if (ret)
		return ret;
	ret = wlbt_stream_init(w, &w->gdb_m4_out, "gdb-m4-out", GDB_NUM_PACKETS,
			       GDB_PACKET_SIZE, WLBT_BIT_GDB_M4_OUT_READ,
			       WLBT_BIT_GDB_M4_OUT_WRITE, false);
	if (ret)
		return ret;

	ret = wlbt_stream_init(w, &w->log, "log", LOG_NUM_PACKETS,
			       LOG_PACKET_SIZE, WLBT_BIT_LOG_READ,
			       WLBT_BIT_LOG_WRITE, true);
	if (ret)
		return ret;

	offset = wlbt_heap_alloc(w, sizeof(*conf));
	if (!offset)
		return -ENOMEM;

	/* Build it in normal memory: the window is device memory, where byte and
	 * half-word stores are not something to rely on, and one block copy also
	 * makes the ordering against the peer's read obvious.
	 */
	memset(conf, 0, sizeof(*conf));
	conf->magic = MXCONF_MAGIC;
	conf->version_major = MXCONF_VERSION_MAJOR;
	conf->version_minor = MXCONF_VERSION_MINOR;
	conf->flags = 0;
	conf->soc_revision = wlbt_soc_revision(w);

	wlbt_stream_serialise(&w->mgmt_in, &conf->trans.to_ap);
	wlbt_stream_serialise(&w->mgmt_out, &conf->trans.from_ap);
	wlbt_stream_serialise(&w->gdb_r4_in, &conf->gdb_r4.to_ap);
	wlbt_stream_serialise(&w->gdb_r4_out, &conf->gdb_r4.from_ap);
	wlbt_stream_serialise(&w->gdb_m4_in, &conf->gdb_m4.to_ap);
	wlbt_stream_serialise(&w->gdb_m4_out, &conf->gdb_m4.from_ap);
	wlbt_stream_serialise(&w->log, &conf->logconf.stream_conf);

	conf->fwconfig.offset = MXCONF_FWCONFIG_UNUSED;
	conf->fwconfig.size = 0;

	memcpy_toio(w->dram + offset, conf, sizeof(*conf));
	/* The peer reads the configuration over the bus, so it has to be
	 * there before the doorbell rings.
	 */
	wmb();

	w->mxconf_offset = offset;

	dev_info(w->dev, "configuration at MIF+0x%x: mgmt %u/%u, log %u/%u, %zu bytes\n",
		 offset, MGMT_NUM_PACKETS, MGMT_PACKET_SIZE, LOG_NUM_PACKETS,
		 LOG_PACKET_SIZE, sizeof(*conf));

	return 0;
}

/*
 * Power up and let the peer run (platform_mif_9610.c:1043-1069).  The caller
 * holds w->lock and has already put the state machine in WAIT_CFG_REQ.
 */

