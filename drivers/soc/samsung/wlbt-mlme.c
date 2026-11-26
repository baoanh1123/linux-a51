/* SPDX-License-Identifier: GPL-2.0 */

#include "exynos9610-wlbt.h"

static void wlbt_scan_report_one(struct exynos_wlbt *w, const u8 *frame, size_t len, u16 freq, s16 rssi, gfp_t gfp);

void wlbt_sae_retry_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(to_delayed_work(work),
					     struct exynos_wlbt, sae_retry_work);

	if (!w->sae_pending || !w->defer_sig_len)
		return;

	dev_info(w->dev, "sae: re-issuing the join (attempt %u)\n",
		 w->sae_retries);
	wlbt_hip4_send(w, true, w->defer_sig, w->defer_sig_len,
		       w->defer_dat, w->defer_dat_len);
}

static void wlbt_sae_deliver(struct exynos_wlbt *w, const u8 *frame, size_t len,
			     s16 sig)
{
	struct cfg80211_external_auth_params auth = { };
	u8 stype;
	bool delivered = false;
	bool started;

	if (!w->netdev)
		return;

	/*
	 * The peer reports frames the station itself transmitted back as
	 * received indications.  Handing the supplicant its own commit or
	 * confirm makes its state machine read a reply that never came, so
	 * anything transmitted by this station is not a received frame.
	 */
	if (len >= 24 && ether_addr_equal(frame + 10, w->netdev->dev_addr)) {
		dev_info(w->dev, "sae: own transmission came back, dropped\n");
		return;
	}

	/*
	 * A commit that arrives after the confirm is a replay of an older
	 * attempt, and the supplicant discards the whole exchange when it
	 * sees one.  Drop them until it starts a new one of its own.
	 */
	if (len >= 32 && ((frame[0] >> 4) & 0xf) == (IEEE80211_STYPE_AUTH >> 4)) {
		u16 tseq = get_unaligned_le16(frame + 26);

		if (tseq == 2) {
			w->sae_confirmed = true;
		} else if (tseq == 1) {
			/*
			 * One commit per exchange.  A second one is the peer handing
			 * the same commit up again, and the supplicant answers it by
			 * starting a fresh exchange - which invalidates the confirm
			 * that is about to arrive.
			 */
			if (w->sae_commit_seen || w->sae_confirmed) {
				dev_info(w->dev,
					 "sae: repeated commit dropped\n");
				return;
			}
			w->sae_commit_seen = true;
		}
	}

	/* From here until the verdict, the exchange belongs to the
	 * supplicant and the peer's failures must not be reported as ours.
	 */
	started = w->sae_pending;
	w->sae_pending = true;

	/*
	 * Ask the supplicant to start before the frame is handed over, and only
	 * once per exchange.  Sent afterwards it can be processed after the
	 * exchange has already run, and the supplicant answers it by starting a
	 * new one - which is the second commit the peer sees right after every
	 * failure.
	 */
	if (!started && len >= 24) {
		auth.action = NL80211_EXTERNAL_AUTH_START;
		memcpy(auth.bssid, frame + 10, ETH_ALEN);
		memcpy(auth.ssid.ssid, w->connect_ssid, w->connect_ssid_len);
		auth.ssid.ssid_len = w->connect_ssid_len;
		auth.key_mgmt_suite = WLAN_AKM_SUITE_SAE;
		if (cfg80211_external_auth_request(w->netdev, &auth, GFP_KERNEL))
			dev_info(w->dev, "sae: start request refused\n");
	}

	if (!len || len < 24) {
		if (!w->sae_no_frame || !is_valid_ether_addr(w->connect_bssid) ||
		    !w->connect_ssid_len)
			return;

		w->sae_no_frame = false;
		auth.action = NL80211_EXTERNAL_AUTH_START;
		memcpy(auth.bssid, w->connect_bssid, ETH_ALEN);
		memcpy(auth.ssid.ssid, w->connect_ssid, w->connect_ssid_len);
		auth.ssid.ssid_len = w->connect_ssid_len;
		auth.key_mgmt_suite = WLAN_AKM_SUITE_SAE;
		dev_info(w->dev, "sae: no frame in the indication, asking for %pM anyway\n",
			 auth.bssid);
		if (cfg80211_external_auth_request(w->netdev, &auth, GFP_KERNEL))
			dev_info(w->dev, "sae: the supplicant did not ask for the frame\n");
		return;
	}

	stype = (frame[0] >> 4) & 0xf;

	if (stype == (IEEE80211_STYPE_AUTH >> 4)) {
		/* The access point's half of the exchange: the supplicant
		 * registered for these and answers through the management
		 * transmit.
		 */
		/*
		 * The peer hands the same frame up again while the exchange is
		 * running, and a duplicate at the confirm boundary is enough for
		 * the supplicant to treat the exchange as replayed and abort it.
		 * Identical means identical: same length, same bytes, close in
		 * time.
		 */
		if (len >= 32 && get_unaligned_le16(frame + 26) == 2) {
			/*
			 * The access point retransmits its whole exchange, so a
			 * confirm from the previous round can arrive after the
			 * supplicant started a new one, and its hash then fails
			 * against the new state.  A confirm only means anything
			 * after the station has sent its own, and only once.
			 */
			if (!w->sae_sent_confirm) {
				dev_info(w->dev,
					 "sae: confirm before ours dropped\n");
				return;
			}
			if (len == w->sae_last_confirm_len &&
			    w->sae_last_confirm_len &&
			    !memcmp(frame, w->sae_last_confirm, len) &&
			    time_before(jiffies, w->sae_last_confirm_jiffies +
						 msecs_to_jiffies(1500))) {
				dev_info(w->dev, "sae: repeated confirm dropped\n");
				return;
			}
			w->sae_last_confirm_len = min(len,
						      sizeof(w->sae_last_confirm));
			memcpy(w->sae_last_confirm, frame, w->sae_last_confirm_len);
			w->sae_last_confirm_jiffies = jiffies;
			dev_info(w->dev, "sae: confirm hash %*phN\n",
				 (int)min(len - 30, 32u), frame + 30);
		}

		if (len == w->sae_last_len &&
		    !memcmp(frame, w->sae_last, len) &&
		    time_before(jiffies, w->sae_last_jiffies + msecs_to_jiffies(400))) {
			delivered = false;
			dev_info(w->dev, "sae: duplicate %zu byte frame dropped\n",
				 len);
		} else {
			w->sae_last_len = len;
			memcpy(w->sae_last, frame, len);
			w->sae_last_jiffies = jiffies;
			delivered = cfg80211_rx_mgmt(w->netdev->ieee80211_ptr,
						     w->connect_freq, sig,
						     frame, len, 0);
		}
	} else {
		/* A beacon or probe response: the vendor feeds these into the BSS
		 * table as well, so the supplicant has an access point to run the
		 * exchange against even though this scan never returned it.
		 */
		/* The rssi of the frame being joined is not reported here. */
		wlbt_scan_report_one(w, frame, len, w->connect_freq,
				     (s16)-60, GFP_KERNEL);
	}

	auth.action = NL80211_EXTERNAL_AUTH_START;
	memcpy(auth.bssid, frame + 10, ETH_ALEN);	/* addr2: the access point */
	memcpy(auth.ssid.ssid, w->connect_ssid, w->connect_ssid_len);
	auth.ssid.ssid_len = w->connect_ssid_len;
	auth.key_mgmt_suite = WLAN_AKM_SUITE_SAE;

	dev_info(w->dev,
		 "sae: frame %zu bytes from %pM type 0x%02x stype %u (%s)\n",
		 len, auth.bssid, frame[0], stype,
		 stype == (IEEE80211_STYPE_AUTH >> 4) ?
		 (delivered ? "delivered" : "no listener") : "bss informed");
	if (stype == (IEEE80211_STYPE_AUTH >> 4) && len >= 32) {
		u16 tseq = get_unaligned_le16(frame + 26);

		dev_info(w->dev,
			 "sae: seq %u status %u %s %u hdr %6phN\n",
			 tseq, get_unaligned_le16(frame + 28),
			 tseq == 2 ? "send-confirm" : "group",
			 get_unaligned_le16(frame + 30), frame);
	}

	/*
	 * The request only exists to make the supplicant start SAE; the frames
	 * themselves go in through cfg80211_rx_mgmt().  Repeating it after the
	 * confirm tells the supplicant to begin a new exchange just as it
	 * finished one, which is how the confirm ended up processed against a
	 * state that had already been reset.
	 */
	if (stype == (IEEE80211_STYPE_AUTH >> 4) && len >= 32 &&
	    get_unaligned_le16(frame + 26) == 2)
		dev_info(w->dev, "sae: confirm delivered, not starting another exchange\n");
}

void wlbt_sae_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(work, struct exynos_wlbt, sae_work);
	unsigned long flags;
	u8 frame[WLBT_SAE_FRAME];
	size_t len;
	s16 sig;

	for (;;) {
		spin_lock_irqsave(&w->sae_q_lock, flags);
		if (w->sae_q_head == w->sae_q_tail) {
			spin_unlock_irqrestore(&w->sae_q_lock, flags);
			break;
		}
		len = w->sae_q_len[w->sae_q_head];
		sig = w->sae_q_sig[w->sae_q_head];
		memcpy(frame, w->sae_q[w->sae_q_head], len);
		w->sae_q_head = (w->sae_q_head + 1) % ARRAY_SIZE(w->sae_q);
		spin_unlock_irqrestore(&w->sae_q_lock, flags);

		wlbt_sae_deliver(w, frame, len, sig);
	}
}

/*
 * The peer's firmware stopped.  A panicking peer asserts its own reset and the
 * block will not come up again until the SoC reboots: wlbt_power_down() followed
 * by the boot sequence never produces another CFG_REQ, and neither does the boot
 * sequence on its own from the state the peer leaves behind (both measured, see
 * notes/scsc.md).  So the job here is to make the failure visible and safe: drop
 * the link, tell cfg80211, and refuse further work instead of handing frames to a
 * peer that will never answer.
 */
void wlbt_peer_gone_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(work, struct exynos_wlbt, peer_gone_work);
	bool was_connected;

	rtnl_lock();
	cancel_delayed_work_sync(&w->scan_work);
	cancel_delayed_work_sync(&w->connect_work);
	cancel_work_sync(&w->peer_work);

	if (w->scan_req) {
		struct cfg80211_scan_info info = { .aborted = true };

		cfg80211_scan_done(w->scan_req, &info);
		w->scan_req = NULL;
	}

	was_connected = w->connected;
	w->connected = false;
	w->connecting = false;
	w->connect_res_sent = false;
	w->vif_created = false;
	w->port_authorized = false;
	w->post_connect_done = false;
	wlbt_eapol_purge(w);
	w->unsync_vif = false;
	w->peer_dead = true;
	if (w->netdev)
		netif_carrier_off(w->netdev);
	if (was_connected && w->netdev)
		cfg80211_disconnected(w->netdev, 4, NULL, 0, true, GFP_KERNEL);
	rtnl_unlock();

	dev_err(w->dev, "peer stopped; wifi is off until this device is rebooted\n");
}



bool wlbt_bss_frame(const u8 *buf, size_t avail, size_t *frame_len,
			   u16 *freq)
{
	static const u8 bssid_ofs = 16;
	size_t off = 36;
	bool have_ssid = false;
	u8 channel = 0;
	u16 fc;

	if (avail < 36)
		return false;

	fc = get_unaligned_le16(buf);
	if (fc != (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON) &&
	    fc != (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_RESP))
		return false;
	if (memcmp(buf + 10, buf + bssid_ofs, ETH_ALEN))
		return false;
	/* A zeroed or multicast address is one of the peer's other buffers
	 * being mistaken for a frame, not an access point.
	 */
	if (is_zero_ether_addr(buf + bssid_ofs) ||
	    !is_valid_ether_addr(buf + bssid_ofs))
		return false;

	while (off + 2 <= avail) {
		u8 id = buf[off];
		u8 len = buf[off + 1];

		if (off + 2 + len > avail)
			break;
		if (id == WLAN_EID_SSID) {
			if (len > 32)
				break;
			have_ssid = true;
		} else if (id == WLAN_EID_DS_PARAMS && len >= 1) {
			channel = buf[off + 2];
		} else if (id == WLAN_EID_HT_OPERATION && len >= 1 && !channel) {
			/*
			 * A 5 GHz beacon usually has no DS parameters, so the
			 * primary channel comes from the HT operation element: its
			 * first payload byte.  The channel number it carries is what
			 * says which band the frame is from.
			 */
			channel = buf[off + 2];
		}
		off += 2 + len;
		if (off - 36 > WLBT_SCAN_IE_MAX)
			break;
	}

	/*
	 * Zero length elements are a legal way to pad, so the only hard
	 * requirement is that the information elements were there at all and
	 * that an SSID was among them.
	 */
	if (!have_ssid || off <= 36)
		return false;

	/*
	 * A channel the wiphy does not know makes cfg80211 dereference a NULL
	 * channel pointer, and since this runs from the scan work that oops
	 * takes cfg80211_scan_done() with it.  Anything the tables do not have is
	 * a misread buffer, not an access point.
	 */
	if (channel) {
		u16 f = ieee80211_channel_to_frequency(channel,
				channel <= 14 ? NL80211_BAND_2GHZ :
						NL80211_BAND_5GHZ);

		if (!f)
			return false;
		*freq = f;
	} else {
		*freq = 2437;
	}

	*frame_len = off;
	return true;
}

static void wlbt_scan_report_one(struct exynos_wlbt *w, const u8 *frame,
				 size_t len, u16 freq, s16 rssi, gfp_t gfp)
{
	struct cfg80211_inform_bss data = {
		.chan = ieee80211_get_channel(w->wiphy, freq),
		.signal = rssi,
	};
	struct cfg80211_bss *bss;

	if (!data.chan) {
		dev_warn_once(w->dev, "scan: frame on unknown frequency %u\n",
			      freq);
		return;
	}

	bss = cfg80211_inform_bss_frame_data(w->wiphy, &data,
					     (struct ieee80211_mgmt *)frame,
					     len, gfp);
	if (scan_log)
		dev_info(w->dev, "bss %pM freq %u len %zu chan %s -> %s\n",
		 frame + 16, freq, len, data.chan ? "known" : "UNKNOWN",
		 IS_ERR(bss) ? "rejected" : "ok");
	if (!IS_ERR_OR_NULL(bss))
		cfg80211_put_bss(w->wiphy, bss);
}

void wlbt_scan_rep_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(work, struct exynos_wlbt,
					     scan_rep_work);
	unsigned long flags;
	u8 frame[512];
	u16 len, freq;
	size_t blen;
	s16 rssi;
	u8 slot;

	for (;;) {
		spin_lock_irqsave(&w->scan_rep_lock, flags);
		if (w->scan_rep_tail == w->scan_rep_head) {
			spin_unlock_irqrestore(&w->scan_rep_lock, flags);
			break;
		}
		slot = w->scan_rep_tail;
		len = w->scan_rep_len[slot];
		rssi = w->scan_rep_rssi[slot];
		memcpy(frame, w->scan_rep_frame[slot], len);
		w->scan_rep_tail = (slot + 1) & 7;
		spin_unlock_irqrestore(&w->scan_rep_lock, flags);

		if (len >= 24 && wlbt_bss_frame(frame, len, &blen, &freq))
			wlbt_scan_report_one(w, frame, blen, freq, rssi,
					     GFP_KERNEL);
	}
}

static void wlbt_scan_collect(struct exynos_wlbt *w)
{
	u8 seen[WLBT_SCAN_SEEN][ETH_ALEN];
	unsigned int n_seen = 0, n_frame = 0;
	u32 off, limit = w->mem_size;
	u8 *buf;

	if (!w->dram || !w->wiphy)
		return;

	buf = kvmalloc(WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP, GFP_KERNEL);
	if (!buf)
		return;

	for (off = 0; off < limit; off += WLBT_SCAN_CHUNK) {
		size_t n = min_t(u32, WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP,
				 limit - off);
		size_t i;

		memcpy_fromio(buf, w->dram + off, n);
		for (i = 0; i + 36 <= n && i < WLBT_SCAN_CHUNK; i++) {
			size_t flen;
			u16 freq;
			unsigned int k;

			if (!wlbt_bss_frame(buf + i, n - i, &flen, &freq))
				continue;
			for (k = 0; k < n_seen; k++)
				if (!memcmp(seen[k], buf + i + 16, ETH_ALEN))
					break;
			if (k < n_seen)
				continue;
			if (n_seen < WLBT_SCAN_SEEN)
				memcpy(seen[n_seen++], buf + i + 16, ETH_ALEN);
			wlbt_scan_report_one(w, buf + i, flen, freq,
					     WLBT_SCAN_RSSI, GFP_KERNEL);
			n_frame++;
		}
	}

	kvfree(buf);
	dev_info(w->dev, "scan: %u beacon(s) reported from the peer's memory\n",
		 n_frame);
}

/*
 * List what the peer's memory holds, the same way the scan collector reads it.
 * Choosing a target for a join by hand needs to know which access points the
 * peer actually cached, and there is exactly one join attempt per boot - a failed
 * join wedges the join state machine until the next boot.
 */
void wlbt_dram_bsses(struct exynos_wlbt *w, struct seq_file *s)
{
	u8 seen[WLBT_SCAN_SEEN][ETH_ALEN];
	unsigned int n_seen = 0, n = 0;
	u32 off, limit = w->mem_size;
	u8 *buf;

	if (!w->dram)
		return;

	buf = kvmalloc(WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP, GFP_KERNEL);
	if (!buf)
		return;

	for (off = 0; off < limit; off += WLBT_SCAN_CHUNK) {
		size_t len = min_t(u32, WLBT_SCAN_CHUNK + WLBT_SCAN_OVERLAP,
				   limit - off);
		size_t i;

		memcpy_fromio(buf, w->dram + off, len);
		for (i = 0; i + 36 <= len && i < WLBT_SCAN_CHUNK; i++) {
			const u8 *f = buf + i;
			size_t flen;
			u16 freq;
			unsigned int k;
			unsigned int j = 36;
			const u8 *ssid = NULL, *rsn = NULL, *rsn_caps = NULL;
			u8 ssid_len = 0;
			bool wpa = false;

			if (!wlbt_bss_frame(f, len - i, &flen, &freq))
				continue;
			for (k = 0; k < n_seen; k++)
				if (!memcmp(seen[k], f + 16, ETH_ALEN))
					break;
			if (k < n_seen)
				continue;
			if (n_seen < WLBT_SCAN_SEEN)
				memcpy(seen[n_seen++], f + 16, ETH_ALEN);

			while (j + 2 <= flen) {
				const u8 *ie = f + j + 2;
				unsigned int ielen = f[j + 1];

				if (j + 2 + ielen > flen)
					break;
				if (f[j] == WLAN_EID_SSID && !ssid) {
					ssid = ie;
					ssid_len = ielen;
				} else if (f[j] == WLAN_EID_RSN) {
					rsn = ie;
					rsn_caps = ielen >= 4 ? ie + ielen - 2 : NULL;
				} else if (f[j] == WLAN_EID_VENDOR_SPECIFIC &&
					   ielen >= 4 && ie[0] == 0x00 &&
					   ie[1] == 0x50 && ie[2] == 0xf2 &&
					   ie[3] == 0x01) {
					wpa = true;
				}
				j += 2 + ielen;
			}

			seq_printf(s, "%pM  %u MHz  ", f + 16, freq);
			if (ssid)
				seq_printf(s, "\"%.*s\"  ", ssid_len, ssid);
			else
				seq_puts(s, "(no SSID)  ");
			seq_printf(s, "%s%s%s%s\n",
				   (f[34] & 0x10) ? "privacy " : "open ",
				   rsn ? "RSN " : "", wpa ? "WPA " : "",
				   rsn && rsn_caps[0] != 0 ? "MFP " : "");
			n++;
		}
	}

	kvfree(buf);
	seq_printf(s, "%u access point(s)\n", n);
}

void wlbt_scan_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(to_delayed_work(work),
					     struct exynos_wlbt, scan_work);
	struct cfg80211_scan_info info = { };
	struct cfg80211_scan_request *req = w->scan_req;

	w->scan_req = NULL;
	if (!req)
		return;

	mutex_lock(&w->lock);
	wlbt_scan_collect(w);
	mutex_unlock(&w->lock);

	w->scan_done = jiffies;
	cfg80211_scan_done(req, &info);
}

static int wlbt_cfg_scan(struct wiphy *wiphy,
			 struct cfg80211_scan_request *request)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	int ret;

	/* After the peer stopped there is nothing to ask: say so at once rather
	 * than waiting for a scan result that cannot arrive.
	 */
	if (w->peer_dead) {
		dev_info(w->dev, "scan refused: peer dead\n");
		return -EIO;
	}

	if (!w->hip4_control) {
		dev_info(w->dev, "scan refused: no hip4 control\n");
		return -EIO;
	}
	if (w->scan_req) {
		dev_info_ratelimited(w->dev, "scan refused: one is pending\n");
		return -EBUSY;
	}

	/*
	 * A scan request queued while the station is associated is never
	 * consumed: the peer stays on the channel of the connection, and its
	 * single reader walks the queues in order, so everything queued behind
	 * the scan - data included - waits with it.  Refuse instead of
	 * reporting a scan that will not happen.
	 */
	if (w->connected) {
		dev_info_ratelimited(w->dev, "scan refused: connected\n");
		return -EBUSY;
	}

	/* The scan id names the interface in its upper byte and the peer
	 * refuses anything below 0x100, so the low byte just wraps.
	 */
	w->scan_id = 0x0100 | ((w->scan_id + 1) & 0xff);
	ret = wlbt_mlme_add_scan(w, w->scan_id, request);
	dev_info(w->dev, "scan: req id 0x%04x chans %d ssids %d ret %d\n",
		 w->scan_id, request->n_channels, request->n_ssids, ret);
	if (ret)
		return ret;

	w->scan_req = request;
	schedule_delayed_work(&w->scan_work,
			      msecs_to_jiffies(WLBT_SCAN_DELAY_MS));
	return 0;
}

/*
 * Bringing the interface up is a firmware sequence, not a driver one: the peer
 * needs a vif record before it will accept a join against it, and it refuses
 * the join with INVALID_VIF_INDEX (0x800a, fault 0x2614) otherwise.  The vendor
 * driver sends MLME_ADD_VIF, the host state and the country when the net device
 * opens; all three are sent here on the first join instead, because the peer has
 * to be running before any of them makes sense.
 */
static int wlbt_mlme_add_vif_at(struct exynos_wlbt *w, u16 vif, u16 type)
{
	static const u8 mac[6] = { 0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc };
	/*
	 * The vendor passes the broadcast address as the vif's partner address for
	 * a station (cfg80211_ops.c:1030-1362: device_address is initialised to
	 * ff:ff:ff:ff:ff:ff and only rewritten for a P2P client), while this driver
	 * used to put its own address there.  The peer keeps both addresses in the
	 * vif record and the join path compares against them, so send the same
	 * thing the vendor sends.
	 */
	static const u8 partner[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 28];
	struct wlbt_fapi_hdr hdr = { };

	/* Body: interface, own address, type, partner address - the order the
	 * peer accepted when the vif was created by hand.
	 */
	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(vif, sig + sizeof(hdr) + 0);
	memcpy(sig + sizeof(hdr) + 2, mac, sizeof(mac));
	put_unaligned_le16(type, sig + sizeof(hdr) + 8);
	memcpy(sig + sizeof(hdr) + 10, partner, sizeof(partner));

	hdr.id = cpu_to_le16(FAPI_MLME_ADD_VIF_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 28, NULL, 0);
}

static int wlbt_mlme_add_vif(struct exynos_wlbt *w)
{
	/* FAPI_VIFTYPE_STATION */
	return wlbt_mlme_add_vif_at(w, 1, 2);
}

static int wlbt_mlme_set_channel(struct exynos_wlbt *w, u16 vif, u16 freq)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 22] = { };
	struct wlbt_fapi_hdr hdr = { };

	put_unaligned_le16(vif, sig + sizeof(hdr) + 0);
	/* availability_duration, availability_interval and count stay zero: the
	 * vendor asks for an unspecified duration on this path.
	 */
	put_unaligned_le16(freq * 2, sig + sizeof(hdr) + 8);

	hdr.id = cpu_to_le16(FAPI_MLME_SET_CHANNEL_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

static int wlbt_mlme_register_action_frame2(struct exynos_wlbt *w, u16 vif,
					    u32 bitmap, u32 suspended);

static int wlbt_mlme_register_action_frame(struct exynos_wlbt *w, u16 vif,
					    u32 bitmap)
{
	return wlbt_mlme_register_action_frame2(w, vif, bitmap, bitmap);
}

static int wlbt_mlme_register_action_frame2(struct exynos_wlbt *w, u16 vif,
					    u32 bitmap, u32 suspended)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 22] = { };
	struct wlbt_fapi_hdr hdr = { };

	put_unaligned_le16(vif, sig + sizeof(hdr) + 0);
	put_unaligned_le32(bitmap, sig + sizeof(hdr) + 2);
	put_unaligned_le32(suspended, sig + sizeof(hdr) + 6);

	hdr.id = cpu_to_le16(FAPI_MLME_REGISTER_ACTION_FRAME_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

static int wlbt_mlme_del_vif_at(struct exynos_wlbt *w, u16 vif)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };

	put_unaligned_le16(vif, sig + sizeof(hdr) + 0);

	hdr.id = cpu_to_le16(FAPI_MLME_DEL_VIF_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

static int wlbt_mlme_del_vif(struct exynos_wlbt *w);
static int wlbt_mlme_register_action_frame2(struct exynos_wlbt *w, u16 vif,
					    u32 bitmap, u32 suspended);

/*
 * The interface a management frame leaves by.  Before the station is on a
 * network the peer will not transmit from its own vif, so the vendor's answer is
 * a second interface of the unsynchronised kind, put on the channel the exchange
 * happens on.  It is created on the first frame and torn down when the join
 * finishes, because the peer associates on the station interface.
 */
static int wlbt_unsync_vif_up(struct exynos_wlbt *w, u16 freq)
{
	u16 vif = mgmt_tx_vif == 2 ? WLBT_UNSYNC_VIF : 1;
	int ret;

	if (w->unsync_vif)
		return 0;

	/*
	 * The vendor does not use a second index: it turns the station's own
	 * interface into an unsynchronised one (mgt.c: it sets vif_type before
	 * adding the interface, and the index it adds is the netdev's).  Keep
	 * that as an option, because which index the peer forwards received
	 * authentication frames on is the question this answers.
	 */
	if (vif == 1 && w->vif_created) {
		wlbt_mlme_del_vif(w);
		w->vif_created = false;
		msleep(60);
	}

	w->unsync_index = vif;
	ret = wlbt_mlme_add_vif_at(w, vif, FAPI_VIFTYPE_UNSYNCHRONISED);
	if (ret)
		return ret;
	msleep(60);

	ret = wlbt_mlme_register_action_frame(w, vif,
					      FAPI_ACTION_FRAME_PUBLIC |
					      FAPI_ACTION_FRAME_RADIO_MEAS);
	if (ret)
		return ret;
	msleep(30);

	ret = wlbt_mlme_set_channel(w, vif, freq);
	if (ret)
		return ret;
	msleep(30);

	w->unsync_vif = true;
	dev_info(w->dev, "sae: unsynchronised interface %u on %u MHz\n", vif, freq);
	return 0;
}

static void wlbt_unsync_vif_down(struct exynos_wlbt *w)
{
	if (!w->unsync_vif)
		return;

	wlbt_mlme_del_vif_at(w, w->unsync_index);
	w->unsync_vif = false;
	dev_info(w->dev, "sae: unsynchronised interface released\n");
}

static int wlbt_mlme_host_state(struct exynos_wlbt *w, u16 state)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16];
	struct wlbt_fapi_hdr hdr = { };

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);	/* vif */
	put_unaligned_le16(0xffff, sig + sizeof(hdr) + 2);
	put_unaligned_le16(state, sig + sizeof(hdr) + 4);

	hdr.id = cpu_to_le16(FAPI_MLME_SET_HOST_STATE_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 16, NULL, 0);
}

/*
 * The peer will not transmit on a channel its regulatory domain does not allow,
 * which shows up as a join refused with NOT_ALLOWED rather than as an error at
 * scan time.  The vendor sends this right after the host state.
 */
static int wlbt_mlme_set_country(struct exynos_wlbt *w, const char *alpha2)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14];
	struct wlbt_fapi_hdr hdr = { };

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(0, sig + sizeof(hdr) + 0);	/* not tied to a vif */
	put_unaligned_le16((alpha2[0] << 8) | alpha2[1],
			   sig + sizeof(hdr) + 2);

	hdr.id = cpu_to_le16(FAPI_MLME_SET_COUNTRY_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 14, NULL, 0);
}

static int wlbt_vif_up(struct exynos_wlbt *w)
{
	int ret;

	if (w->vif_created)
		return 0;

	ret = wlbt_mlme_add_vif(w);
	if (ret)
		return ret;
	/* The peer answers on its own thread; give it the moment it needs
	 * before the join that depends on the record.
	 */
	msleep(150);

	ret = wlbt_mlme_host_state(w, 2);	/* cellular active */
	if (ret)
		return ret;
	msleep(50);

	/* The vendor registers the station's action frame categories as part of
	 * bringing the interface up, not only when it wants a specific one.
	 */
	ret = wlbt_mlme_register_action_frame2(w, 1, WLBT_STA_ACTION_FRAME_BITMAP,
					       WLBT_STA_ACTION_FRAME_SUSPEND);
	if (ret)
		return ret;
	msleep(30);

	ret = wlbt_mlme_set_country(w, "VN");
	if (ret)
		return ret;
	msleep(50);


	w->vif_created = true;
	return 0;
}

/*
 * The join is the peer's job: the driver only builds MLME_CONNECT_REQ and waits
 * for the firmware to report the outcome.  The body is the vendor's layout -
 * interface, BSSID, authentication type at +8 and the channel frequency at +10
 * as twice the real value - which is the one the peer accepts and acts on.
 *
 * The information elements are the association request's.  cfg80211 does not
 * include the SSID in what it hands over, and the peer refuses a request without
 * it (INVALID_PARAMETERS) as well as one without an RSN element
 * (REJECTED_INVALID_IE).
 */
/*
 * The peer announces a connection with MLME_CONNECT_IND and then waits for the
 * host to say how to proceed; without an answer it drops the link again a few
 * seconds later (MLME_DISCONNECTED_IND, reason 3).  Both replies are the vif
 * number and three spare words.
 */
static void wlbt_mlme_connect_res(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };
	u16 vif = 1;
	int ret;

	hdr.id = cpu_to_le16(FAPI_MLME_CONNECT_RES);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(vif, sig + sizeof(hdr));

	ret = wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
	dev_info(w->dev, "connect: replied to MLME_CONNECT_IND (%d)\n", ret);
}

/*
 * The peer keeps a list of the channels the host considers usable and wants it
 * refreshed when a connection is up, not before: sent ahead of a join it makes
 * the peer refuse the join with NOT_ALLOWED.  The list travels as a vendor
 * element of channel entries.
 */
static int wlbt_mlme_set_cached_channels(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14];
	u8 dat[7 + 3 * (ARRAY_SIZE(wlbt_2ghz_channels) +
			ARRAY_SIZE(wlbt_5ghz_channels))];
	struct wlbt_fapi_hdr hdr = { };
	unsigned int i, n = 0;

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);	/* vif 1 */

	dat[n++] = 0xdd;			/* vendor specific element */
	dat[n++] = 5;				/* OUI, type and subtype */
	dat[n++] = 0x00;
	dat[n++] = 0x16;
	dat[n++] = 0x32;
	dat[n++] = 0x01;			/* scan parameters */
	dat[n++] = 0x02;			/* channel list */

	/*
	 * Both bands, because the peer keeps this list for its own scans and a
	 * 5 GHz access point is only found if its channel is in it.  The policy
	 * byte carries the band bits (0x08 for 2.4 GHz, 0x10 for 5 GHz) and, for
	 * the radar detection bands, the DFS bit as well.
	 */
	for (i = 0; i < ARRAY_SIZE(wlbt_2ghz_channels); i++) {
		put_unaligned_le16(wlbt_2ghz_channels[i].center_freq * 2, dat + n);
		dat[n + 2] = 0x08;
		n += 3;
		dat[1] += 3;
	}
	for (i = 0; i < ARRAY_SIZE(wlbt_5ghz_channels); i++) {
		u16 freq = wlbt_5ghz_channels[i].center_freq;

		put_unaligned_le16(freq * 2, dat + n);
		dat[n + 2] = wlbt_channel_passive(freq) ? 0x58 : 0x18;
		n += 3;
		dat[1] += 3;
	}

	hdr.id = cpu_to_le16(FAPI_MLME_SET_CACHED_CHANNELS);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 14, dat, n);
}

/*
 * MLME_SET_PACKET_FILTER with no patterns.  The vendor sends this on connect for
 * an open network, and the peer appears to need it before it will carry frames.
 */
static int wlbt_mlme_set_packet_filter(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };

	hdr.id = cpu_to_le16(0x2014);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif 1 */

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

static int wlbt_mlme_del_vif(struct exynos_wlbt *w)
{
	return wlbt_mlme_del_vif_at(w, 1);
}

/* MLME_SET_ACL with no entries: the station accepts its peer and nothing else. */
static int wlbt_mlme_set_acl(struct exynos_wlbt *w)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 18] = { };
	struct wlbt_fapi_hdr hdr = { };

	hdr.id = cpu_to_le16(FAPI_MLME_SET_ACL_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif 1 */

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

static void wlbt_mlme_connected_res(struct exynos_wlbt *w, u16 aid)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16] = { };
	struct wlbt_fapi_hdr hdr = { };
	int ret;

	hdr.id = cpu_to_le16(FAPI_MLME_CONNECTED_RES);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr));
	put_unaligned_le16(aid, sig + sizeof(hdr) + 2);

	ret = wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
	dev_info(w->dev, "connect: replied to MLME_CONNECTED_IND %u (%d)\n",
		 aid, ret);
}

/*
 * The vendor gives the peer the supplicant's information elements twice: once
 * named as the association request's elements (mlme.c:2430), and again inside
 * the connect request.  The copy in the connect request is what ends up in the
 * frame on air, which is why the association looks right without this; the
 * copy sent here is what the peer reads to work out what the link *is*, the RSN
 * element above all.  Sent only the connect request, the peer associates, runs
 * the four way handshake, stores the keys, confirms them - and still treats the
 * link as open, because it was never told that it is protected.
 */
static int wlbt_mlme_add_info_elements(struct exynos_wlbt *w, const u8 *ies,
				       size_t len)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 16] = { };
	struct wlbt_fapi_hdr hdr = { };

	if (!ies || !len)
		return 0;

	hdr.id = cpu_to_le16(FAPI_MLME_ADD_INFO_ELEMENTS_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	put_unaligned_le16(FAPI_PURPOSE_ASSOCIATION_REQUEST,
			   sig + sizeof(hdr) + 2);

	return wlbt_hip4_send(w, true, sig, sizeof(sig), ies, len);
}

static int wlbt_mlme_connect(struct exynos_wlbt *w,
			     const struct cfg80211_connect_params *sme,
			     const u8 *bssid, u16 freq)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 24];
	u8 dat[2 + IEEE80211_MAX_SSID_LEN + 512];
	struct wlbt_fapi_hdr hdr = { };
	size_t ssid_len = sme->ssid_len, n = 0;

	if (!bssid || ssid_len > IEEE80211_MAX_SSID_LEN)
		return -EINVAL;

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif 1 */
	memcpy(sig + sizeof(hdr) + 2, bssid, ETH_ALEN);
	/* A WPA3 join is not announced to the peer as SAE: the vendor rewrites
	 * NL80211_AUTHTYPE_SAE to NL80211_AUTHTYPE_NETWORK_EAP (5) in this
	 * field, which makes the peer hand authentication over to us and
	 * forward the access point's frames.  Announcing SAE itself makes it
	 * accept our commit and then drop it without a word.
	 */
	put_unaligned_le16(sme->auth_type == NL80211_AUTHTYPE_SAE ?
			   NL80211_AUTHTYPE_NETWORK_EAP : sme->auth_type,
			   sig + sizeof(hdr) + 8);
	put_unaligned_le16(freq * 2, sig + sizeof(hdr) + 10);

	dat[n++] = WLAN_EID_SSID;
	dat[n++] = ssid_len;
	memcpy(dat + n, sme->ssid, ssid_len);
	n += ssid_len;

	if (sme->ie_len) {
		const u8 *ie = sme->ie;
		size_t ie_len = sme->ie_len;

		if (sec_ie_only) {
			const u8 *rsn = cfg80211_find_ie(WLAN_EID_RSN, ie, ie_len);

			/*
			 * The vendor appends only the security element to the connect
			 * request, and nothing at all when the link has none - its
			 * access point on an open network sees the SSID element and
			 * no more.  sec_ie_only reproduces that shape, so the extra
			 * elements a supplicant adds (extended capabilities, vendor
			 * specific) do not reach a peer that may refuse them.
			 */
			ie = rsn;
			ie_len = rsn ? rsn[1] + 2 : 0;
		}

		if (ie_len) {
			if (ie_len > sizeof(dat) - n)
				return -EINVAL;
			memcpy(dat + n, ie, ie_len);
			n += ie_len;
		}
	}

	hdr.id = cpu_to_le16(FAPI_MLME_CONNECT_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	/*
	 * The peer abandons a join within tens of milliseconds, far less than
	 * the supplicant needs to run the SAE exchange, and it does not need
	 * to join at all to relay the frames.  Hold the request back, hand
	 * the exchange over, and issue it when the verdict and key are in.
	 */
	w->sae_retries = 0;
	w->sae_confirmed = false;
	w->sae_commit_seen = false;
	w->sae_sent_confirm = false;
	w->sae_last_confirm_len = 0;
	/* Kept so a refused join can be re-issued while SAE is still running. */
	memcpy(w->defer_sig, sig, sizeof(hdr) + 24);
	w->defer_sig_len = sizeof(hdr) + 24;
	memcpy(w->defer_dat, dat, n);
	w->defer_dat_len = n;

	if (sae_defer_join && sme->auth_type == NL80211_AUTHTYPE_SAE &&
	    !w->connect_pmk_len) {
		struct cfg80211_external_auth_params auth = { };

		w->sae_defer = true;

		auth.action = NL80211_EXTERNAL_AUTH_START;
		memcpy(auth.bssid, bssid, ETH_ALEN);
		memcpy(auth.ssid.ssid, sme->ssid, ssid_len);
		auth.ssid.ssid_len = ssid_len;
		auth.key_mgmt_suite = WLAN_AKM_SUITE_SAE;

		dev_info(w->dev,
			 "sae: join held back until the exchange finishes\n");
		if (cfg80211_external_auth_request(w->netdev, &auth,
						   GFP_KERNEL))
			dev_info(w->dev, "sae: the supplicant did not take it\n");
		/*
		 * The supplicant's exchange has ten seconds from the connect to
		 * finish, and the peer only relays frames while it is joining:
		 * start both, the exchange first and the join right behind it.
		 */
		w->sae_pending = true;
		schedule_delayed_work(&w->sae_retry_work, msecs_to_jiffies(30));
		return 0;
	}

	return wlbt_hip4_send(w, true, sig, sizeof(hdr) + 24, dat, n);
}

/*
 * Answer whatever the peer asked for in its connect indications, and tell
 * cfg80211 about the outcome.  This cannot run in the drain: the replies are
 * sends that take the queue lock, and cfg80211 may sleep.
 */
static int wlbt_mlme_set_pmk(struct exynos_wlbt *w, const u8 *pmk,
			     size_t len);
int wlbt_mlme_setkeys(struct exynos_wlbt *w, u16 key_id, u16 key_type,
			     const u8 *address, u32 cipher, const u8 *key,
			     size_t key_len, const u8 *seq, size_t seq_len);

/*
 * Drop the peer's per-vif state after a join the peer refused or timed out.
 * Leaving the half-built station behind is what lets its watchdog fire later;
 * deleting the vif also lets the next connect create a clean one.
 */
static void wlbt_join_abort(struct exynos_wlbt *w)
{
	w->post_connect_done = false;
	wlbt_eapol_purge(w);
	wlbt_unsync_vif_down(w);

	if (w->vif_created) {
		msleep(20);
		wlbt_mlme_del_vif(w);
		w->vif_created = false;
	}
}

void wlbt_peer_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(work, struct exynos_wlbt, peer_work);
	unsigned long flags;
	bool cind, cted, disc;
	u16 status, aid, reason;

	spin_lock_irqsave(&w->hip4_lock, flags);
	cind = w->peer_connect_ind;
	cted = w->peer_connected_ind;
	disc = w->peer_disconnected_ind;
	status = w->connect_status;
	aid = w->peer_aid;
	reason = w->peer_disconnect_reason;
	w->peer_connect_cfm = false;
	w->peer_connect_ind = false;
	w->peer_connected_ind = false;
	w->peer_disconnected_ind = false;
	spin_unlock_irqrestore(&w->hip4_lock, flags);

	/*
	 * The peer asks once the connection is there, and its data path stays
	 * shut until it is answered: without this the frames are taken off the
	 * data queue and none of them ever reaches the air.  The confirm that
	 * precedes the indication only says the request was accepted, so
	 * answering that is both too early and unnecessary.
	 */
	wlbt_trace(w, "peer_work: connect_ind %d connected_ind %d disc %d status 0x%04x",
		   cind, cted, disc, status);
	if (cind && !w->connect_res_sent) {
		/*
		 * Before the join the peer has no record to attach a key to, so
		 * the one sent with the connect may have gone nowhere; this is
		 * the first moment it can act on it.
		 */
		if (w->connect_pmk_len)
			wlbt_mlme_set_pmk(w, w->connect_pmk, w->connect_pmk_len);

		if (!connect_res_late) {
			wlbt_mlme_connect_res(w);
			w->connect_res_sent = true;
		}
	}

	if (cind) {
		if (status) {
			if (w->sae_pending) {
				dev_info(w->dev,
					 "connect: refusal 0x%04x during SAE, retrying the join\n",
					 status);
				if (sae_join_retry && w->sae_retries++ < 20)
					schedule_delayed_work(&w->sae_retry_work,
							      msecs_to_jiffies(120));
				return;
			}
			bool was_connected = w->connected;

			dev_info(w->dev, "connect: peer refused the join: 0x%04x\n",
				 status);
			wlbt_join_abort(w);
			w->connecting = false;
			w->connected = false;
			netif_carrier_off(w->netdev);
			if (was_connected)
				cfg80211_disconnected(w->netdev, status, NULL, 0,
						      true, GFP_KERNEL);
			else
				cfg80211_connect_result(w->netdev, w->connect_bssid,
							NULL, 0, NULL, 0,
							WLAN_STATUS_UNSPECIFIED_FAILURE,
							GFP_KERNEL);
			return;
		}

		/*
		 * The vendor reports the connection here, on the indication,
		 * and follows it with the access list and the channel list the
		 * peer keeps for its own scans.  MLME_CONNECTED_IND arrives
		 * later, if at all, so waiting for it means the supplicant has
		 * already given up.
		 */
		/* The peer starts the four way handshake as soon as it associates, and
		 * its message 1 lands while the housekeeping below is still sleeping.
		 * Report the connection first: mac80211 drops that frame otherwise and
		 * the handshake never starts.
		 */
		w->connected = true;
		if (w->connecting) {
			w->connecting = false;
			netif_carrier_on(w->netdev);
			dev_info(w->dev, "connect: connected to %pM\n", w->connect_bssid);
			cfg80211_connect_result(w->netdev, w->connect_bssid, NULL, 0,
						NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL);
		}

		wlbt_trace(w, "peer_work: setting packet filter");
		wlbt_mlme_set_packet_filter(w);
		msleep(20);
		wlbt_trace(w, "peer_work: setting acl");
		wlbt_mlme_set_acl(w);
		msleep(20);
		wlbt_trace(w, "peer_work: setting cached channels");
		wlbt_mlme_set_cached_channels(w);
		wlbt_trace(w, "peer_work: post-connect commands done");
		w->post_connect_done = true;
		wlbt_eapol_drain(w);
	}

	if (cted && !netif_carrier_ok(w->netdev)) {
		wlbt_mlme_connected_res(w, aid);
		netif_carrier_on(w->netdev);
	}

	if (disc) {
		bool was_connected = w->connected;
		bool was_connecting = w->connecting;

		/*
		 * The peer drops the link for two different reasons and both have
		 * to release the state: an established link that went away, and a
		 * join whose four way handshake timed out.  The second one used to
		 * leave "connecting" set, and then every retry the supplicant
		 * made was refused as busy - which its log reports as the driver
		 * failing to associate, not as a handshake that timed out.
		 */
		if (w->sae_pending) {
			dev_info(w->dev,
				 "connect: link lost (reason %u) during SAE, still waiting\n",
				 reason);
			return;
		}
		w->connected = false;
		w->connecting = false;
		w->post_connect_done = false;
		wlbt_eapol_purge(w);
		netif_carrier_off(w->netdev);
		dev_info(w->dev, "connect: peer lost the link, reason %u (was %s)\n",
			 reason, was_connected ? "connected" : "connecting");

		if (was_connected)
			cfg80211_disconnected(w->netdev, reason, NULL, 0,
					      !w->connect_reported, GFP_KERNEL);
		else if (was_connecting)
			cfg80211_connect_result(w->netdev, w->connect_bssid,
						NULL, 0, NULL, 0,
						WLAN_STATUS_UNSPECIFIED_FAILURE,
						GFP_KERNEL);
	}
}

void wlbt_connect_work(struct work_struct *work)
{
	struct exynos_wlbt *w = container_of(to_delayed_work(work),
					     struct exynos_wlbt, connect_work);

	if (!w->connecting)
		return;

	/*
	 * The exchange belongs to the supplicant for as long as it needs:
	 * reporting the join failed while it is still running is what makes
	 * it give up with a status 1 it never got to decide on.
	 */
	if (w->sae_pending) {
		dev_info(w->dev, "connect: join still waiting on the exchange\n");
		schedule_delayed_work(&w->connect_work, msecs_to_jiffies(1000));
		return;
	}

	w->connecting = false;

	if (w->connect_reported && !w->connect_status) {
		dev_info(w->dev, "connect: associated with %pM\n",
			 w->connect_bssid);
		cfg80211_connect_result(w->netdev, w->connect_bssid, NULL, 0,
					NULL, 0, WLAN_STATUS_SUCCESS, GFP_KERNEL);
		return;
	}

	dev_info(w->dev, "connect: failed, peer result 0x%04x%s\n",
		 w->connect_status,
		 w->connect_reported ? "" : " (no indication)");
	wlbt_join_abort(w);
	w->connected = false;
	netif_carrier_off(w->netdev);
	cfg80211_connect_result(w->netdev, w->connect_bssid, NULL, 0, NULL, 0,
				WLAN_STATUS_UNSPECIFIED_FAILURE, GFP_KERNEL);
}


static int wlbt_cfg_connect(struct wiphy *wiphy, struct net_device *ndev,
			    struct cfg80211_connect_params *sme)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	if (w->peer_dead)
		return -EIO;
	w->post_connect_done = false;
	wlbt_eapol_purge(w);
	struct cfg80211_bss *bss;
	u8 bssid[ETH_ALEN];
	u16 freq = 2437;
	int ret;

	/* cfg80211 can refuse a connect before this op is reached, and then the
	 * only trace of it is the supplicant complaining that the driver failed
	 * to associate: say whether the request arrived at all.
	 */
	dev_info(w->dev, "connect: request for \"%.*s\" (connected %d connecting %d)\n",
		 (int)sme->ssid_len, sme->ssid, w->connected, w->connecting);

	/* An SAE join is finished by the supplicant, and it needs to be told
	 * which network the authentication frames belong to.
	 */
	w->connect_ssid_len = min_t(size_t, sme->ssid_len, sizeof(w->connect_ssid));
	memcpy(w->connect_ssid, sme->ssid, w->connect_ssid_len);

	if (!w->hip4_control)
		return -EIO;
	if (w->connecting) {
		/* Refused without a word this looks like an association failure in
		 * the supplicant's log, so name the state that caused it.
		 */
		dev_info(w->dev, "connect: refused, one is already under way\n");
		return -EBUSY;
	}

	ret = wlbt_vif_up(w);
	if (ret)
		return ret;

	/*
	 * A PSK network hands the pairwise master key over with the connect, so
	 * that the peer's key database has it before the join.  The host still
	 * runs the handshake, but the peer is the one that holds the link's
	 * keys, and a key database that does not know the master key cannot be
	 * told what the pairwise key it is given belongs to.
	 */
	if (sme->crypto.psk) {
		dev_info(w->dev, "connect: pairwise master key supplied\n");
		memcpy(w->connect_pmk, sme->crypto.psk, WLAN_PMK_LEN);
		w->connect_pmk_len = WLAN_PMK_LEN;
		ret = wlbt_mlme_set_pmk(w, sme->crypto.psk, WLAN_PMK_LEN);
		if (ret)
			return ret;
	} else {
		w->connect_pmk_len = 0;
	}

	if (sme->channel && sme->channel->center_freq)
		freq = sme->channel->center_freq;

	/*
	 * A connect request does not have to name an access point: when it does
	 * not, the one the scan found is used, which is what "iw connect" and
	 * the supplicant's first attempt rely on.
	 */
	if (sme->bssid) {
		memcpy(bssid, sme->bssid, ETH_ALEN);
	} else {
		bss = cfg80211_get_bss(wiphy, sme->channel, NULL, sme->ssid,
				       sme->ssid_len, IEEE80211_BSS_TYPE_ESS,
				       IEEE80211_PRIVACY_ANY);
		if (!bss) {
			dev_info(w->dev, "connect: no access point for \"%.*s\"\n",
				 (int)sme->ssid_len, sme->ssid);
			return -ENOENT;
		}
		memcpy(bssid, bss->bssid, ETH_ALEN);
		if (!sme->channel && bss->channel)
			freq = bss->channel->center_freq;
		cfg80211_put_bss(wiphy, bss);
	}

	w->connect_freq = freq;

	dev_info(w->dev, "connect: %pM freq %u auth %u ssid \"%.*s\" ies %zu\n",
		 bssid, freq, sme->auth_type, (int)sme->ssid_len, sme->ssid,
		 sme->ie_len);

	memcpy(w->connect_bssid, bssid, ETH_ALEN);
	/*
	 * Nothing may leave the interface before the handshake is done.  The
	 * peer holds the link keys, and a data frame that reaches it while it has
	 * no pairwise key stops the firmware outright: at boot NetworkManager
	 * starts DHCP the moment the carrier comes up, which is well before the
	 * supplicant has finished the 4-way, and that is when the reset request
	 * was seen.  Keep the carrier down and the queue stopped until the key
	 * arrives; an open network is authorized from the start.
	 */
	w->port_authorized = !sme->privacy;
	dev_info(w->dev, "connect: privacy %d, data %s until a key is installed\n",
		 sme->privacy, w->port_authorized ? "allowed" : "held back");
	w->connect_reported = false;
	w->connect_status = 0;
	w->connect_res_sent = false;
	w->peer_connect_cfm = false;

	if (info_elements) {
		ret = wlbt_mlme_add_info_elements(w, sme->ie, sme->ie_len);
		if (ret)
			return ret;
	}

	if (prekey) {
		static const u8 placeholder[16] = { 1, 2, 3, 4, 5, 6, 7, 8,
						    9, 10, 11, 12, 13, 14, 15, 16 };

		wlbt_mlme_setkeys(w, 0, FAPI_KEYTYPE_PAIRWISE, bssid,
				  WLAN_CIPHER_SUITE_CCMP, placeholder,
				  sizeof(placeholder), NULL, 0);
	}

	wlbt_trace(w, "connect: request for \"%.*s\" auth %u",
		   (int)sme->ssid_len, sme->ssid, sme->auth_type);
	ret = wlbt_mlme_connect(w, sme, bssid, freq);
	if (ret)
		return ret;

	w->connecting = true;
	schedule_delayed_work(&w->connect_work, msecs_to_jiffies(10000));
	return 0;
}

/*
 * Telling the peer to leave is MLME_DISCONNECT_REQ: the vif, the access point's
 * address and the reason code, which is what the vendor sends from its
 * disconnect path (mlme.c:slsi_mlme_disconnect, and the 0x200e seen in a
 * capture when the vendor drops a link).
 */
/*
 * The peer keeps per-vif state that a disconnect does not clear: the vendor
 * deletes the interface and creates it again for the next join (mlme.c,
 * slsi_mlme_add_vif calls slsi_mlme_del_vif when require_vif_delete is set), and
 * without that a second join in the same boot never gets a
 * MLME_CONNECT_IND back.
 */

static int wlbt_mlme_disconnect(struct exynos_wlbt *w, const u8 *bssid, u16 reason)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 22] = { };
	struct wlbt_fapi_hdr hdr = { };

	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	memcpy(sig + sizeof(hdr) + 2, bssid, ETH_ALEN);
	put_unaligned_le16(reason, sig + sizeof(hdr) + 8);

	hdr.id = cpu_to_le16(FAPI_MLME_DISCONNECT_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

/*
 * A disconnect has two sides.  The peer has to be told, or it keeps the link
 * and the next join finds a station that is already associated; and cfg80211
 * has to be told, because when the station was connected it keeps both the
 * connection state and the SSID until the driver reports the disconnection
 * (sme.c:cfg80211_disconnect only clears them when the station was not
 * connected).  Without the report, every later connect is refused with
 * -EALREADY before this driver is called at all, which the supplicant shows as
 * "Association request to the driver failed" and which used to need a reboot.
 */
static int wlbt_cfg_disconnect(struct wiphy *wiphy, struct net_device *ndev,
			       u16 reason_code)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	bool was_connected = w->connected;

	cancel_delayed_work_sync(&w->connect_work);
	cancel_work_sync(&w->peer_work);

	if (was_connected) {
		wlbt_mlme_disconnect(w, w->connect_bssid,
				     reason_code ? reason_code : 3);
		dev_info(w->dev, "disconnect: left %pM, reason %u\n",
			 w->connect_bssid, reason_code);
	}

	/*
	 * The peer keeps the interface's own state across a disconnect, so drop
	 * it here and let the next join create it again.
	 */
	wlbt_unsync_vif_down(w);

	if (w->vif_created) {
		msleep(20);
		wlbt_mlme_del_vif(w);
		w->vif_created = false;
	}

	w->connecting = false;
	w->connected = false;
	w->port_authorized = false;
	w->post_connect_done = false;
	wlbt_eapol_purge(w);
	netif_carrier_off(ndev);

	if (was_connected)
		cfg80211_disconnected(ndev, reason_code, NULL, 0, true,
				      GFP_KERNEL);

	return 0;
}

/*
 * MLME_SET_PMK hands the peer the pairwise master key: it keeps a key database
 * of its own, and the key travels as the payload after a sixteen byte body, the
 * way the vendor's fapi_append_data() lays it out.  A zero length request is
 * how the vendor asks for an entry to be dropped.
 */
static int wlbt_mlme_set_pmk(struct exynos_wlbt *w, const u8 *pmk, size_t len)
{
	/*
	 * Fourteen bytes: the interface and three spare words (fapi.h:
	 * mlme_set_pmk_req), with the key appended behind the signal.  Sixteen
	 * put two bytes of padding in front of it.
	 */
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };

	if (len > 64)
		return -EINVAL;
	if (!send_pmk)
		return -EOPNOTSUPP;

	put_unaligned_le16(1, sig + sizeof(hdr));	/* vif */
	hdr.id = cpu_to_le16(FAPI_MLME_SET_PMK_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), pmk, len);
}

static int wlbt_cfg_set_pmk(struct wiphy *wiphy, struct net_device *dev,
			    const struct cfg80211_pmk_conf *conf)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	if (!w->hip4_control)
		return -EIO;

	dev_info(w->dev, "set_pmk: aa %pM len %u\n", conf->aa, conf->pmk_len);

	return wlbt_mlme_set_pmk(w, conf->pmk, conf->pmk_len);
}

static int wlbt_cfg_del_pmk(struct wiphy *wiphy, struct net_device *dev,
			    const u8 *aa)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	if (!w->hip4_control)
		return -EIO;

	dev_info(w->dev, "del_pmk: aa %pM\n", aa);

	return wlbt_mlme_set_pmk(w, NULL, 0);
}

/*
 * MLME_SETKEYS is how the peer is told about the keys a host supplicant
 * negotiated - the firmware does not do the four way handshake itself, so this
 * is the only way a WPA2 connection can carry traffic.  The length field is in
 * bits, the packet sequence numbers are packed as little endian half words, and
 * the key itself is the payload, the way the vendor's fapi_append_data() leaves
 * it.
 */
int wlbt_mlme_setkeys(struct exynos_wlbt *w, u16 key_id, u16 key_type,
			     const u8 *address, u32 cipher, const u8 *key,
			     size_t key_len, const u8 *seq, size_t seq_len)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 46];
	struct wlbt_fapi_hdr hdr = { };
	size_t n = sizeof(hdr);
	unsigned int i;

	if (!key || !key_len || key_len > 32 || !address)
		return -EINVAL;

	memset(sig, 0, sizeof(sig));
	put_unaligned_le16(1, sig + n + 0);		/* vif */
	put_unaligned_le16(key_len * 8, sig + n + 2);	/* length in bits */
	put_unaligned_le16(key_id, sig + n + 4);
	put_unaligned_le16(key_type, sig + n + 6);
	memcpy(sig + n + 8, address, ETH_ALEN);

	for (i = 0; seq && i + 1 < seq_len && i < 16; i += 2)
		put_unaligned_le16((seq[i + 1] << 8) | seq[i],
				   sig + n + 14 + i);

	put_unaligned_le32(cipher, sig + n + 30);

	hdr.id = cpu_to_le16(FAPI_MLME_SETKEYS_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	/*
	 * The key material travels behind the signal, not inside it: the peer
	 * reads it from the payload at the offset the signal length implies, so
	 * a key appended to the signal is read back as whatever follows it.
	 */
	return wlbt_hip4_send(w, true, sig, n + 46, key, key_len);
}

static int wlbt_cfg_add_key(struct wiphy *wiphy, struct net_device *ndev,
			    int link_id, u8 key_index, bool pairwise,
			    const u8 *mac_addr, struct key_params *params)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	u16 type;
	int ret;

	if (!w->hip4_control)
		return -EIO;

	if (!host_keys)
		return 0;

	if (params->cipher == WLAN_CIPHER_SUITE_WEP40 ||
	    params->cipher == WLAN_CIPHER_SUITE_WEP104)
		type = FAPI_KEYTYPE_WEP;
	if (pairwise)
		type = ptk_type;
	else if (params->cipher == WLAN_CIPHER_SUITE_AES_CMAC ||
		 params->cipher == WLAN_CIPHER_SUITE_BIP_GMAC_128 ||
		 params->cipher == WLAN_CIPHER_SUITE_BIP_GMAC_256)
		type = FAPI_KEYTYPE_IGTK;
	else
		type = FAPI_KEYTYPE_GROUP;

	/*
	 * A group key is not addressed to a peer: cfg80211 says so by leaving the
	 * address out, and the peer still wants the access point named, the same
	 * way the vendor takes the station peer record for a group key.
	 */
	if (!mac_addr) {
		/*
		 * IGTK (key index 4/5) also arrives without a peer address.  It
		 * is not a group key, so the check below used to reject it and
		 * the supplicant then tore the link down with reason 1 right
		 * after the 4-way had succeeded.
		 */
		if ((type != FAPI_KEYTYPE_GROUP && type != FAPI_KEYTYPE_IGTK) ||
		    !is_valid_ether_addr(w->connect_bssid))
			return -EINVAL;
		if (type == FAPI_KEYTYPE_GROUP && !install_gtk)
			return 0;
		mac_addr = w->connect_bssid;
	}

	if (pairwise && ptk_id)
		key_index = ptk_id;

	if (pairwise && params->key && params->key_len &&
	    params->key_len <= sizeof(w->ptk_key)) {
		memcpy(w->ptk_key, params->key, params->key_len);
		memcpy(w->ptk_addr, mac_addr, ETH_ALEN);
		w->ptk_cipher = params->cipher;
		w->ptk_len = params->key_len;
		w->ptk_id = key_index;
		w->ptk_valid = true;
	}

	dev_info(w->dev, "setkeys: id %u type %u cipher 0x%x len %d %pM\n",
		 key_index, type, params->cipher, params->key_len, mac_addr);

	ret = wlbt_mlme_setkeys(w, key_index, type, mac_addr, params->cipher,
				params->key, params->key_len, params->seq,
				params->seq_len);
	if (!ret && pairwise && set_ip) {
		u8 sig[sizeof(struct wlbt_fapi_hdr) + 20] = { };
		struct wlbt_fapi_hdr hdr = { };
		__be32 addr;

		hdr.id = cpu_to_le16(FAPI_MLME_SET_IP_ADDRESS_REQ);
		hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
		memcpy(sig, &hdr, sizeof(hdr));
		put_unaligned_le16(1, sig + sizeof(hdr) + 0);	/* vif */
		put_unaligned_le16(4, sig + sizeof(hdr) + 2);	/* IPv4 */
		memset(sig + sizeof(hdr) + 4, 0xff, ETH_ALEN);	/* multicast */
		if (!in4_pton(set_ip, -1, (u8 *)&addr, -1, NULL)) {
			dev_warn(w->dev, "set_ip: cannot parse %s\n", set_ip);
		} else {
			wlbt_hip4_send(w, true, sig, sizeof(sig),
				       (u8 *)&addr, sizeof(addr));
		}
	}

	if (!ret && pairwise && (connect_res_at_key || connect_res_late)) {
		wlbt_mlme_connect_res(w);
		w->connect_res_sent = true;
	}

	/*
	 * The 4-way is far enough along for the peer to take data frames: it has
	 * the pairwise key, so open the port the same way mac80211 does, and let
	 * the stack (DHCP first) at the link.
	 */
	if (!ret && pairwise && w->netdev && !w->port_authorized) {
		w->port_authorized = true;
		cfg80211_port_authorized(w->netdev, w->connect_bssid, NULL, 0,
					 GFP_KERNEL);
		dev_info(w->dev, "key: port authorized\n");
	}

	return ret;
}

static int wlbt_cfg_del_key(struct wiphy *wiphy, struct net_device *ndev,
			    int link_id, u8 key_index, bool pairwise,
			    const u8 *mac_addr)
{
	/* This firmware has no delete-keys request and the vendor driver does
	 * not send one either: a key is replaced by setting it again.
	 */
	return 0;
}
/*
 * cfg80211 asks for this whenever userspace marks a key as the default one.  The
 * peer takes the transmit key from the index in the key install request, so
 * there is nothing to send here, but the op has to exist: add_key without it is
 * an api violation that cfg80211 warns about at registration time.
 */
static int wlbt_cfg_set_default_key(struct wiphy *wiphy, struct net_device *ndev,
				    int link_id, u8 key_idx, bool unicast,
				    bool multicast)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	w->default_key_idx = key_idx;

	return 0;
}

/*
 * A management frame the supplicant wants on the air, which for WPA3 is its own
 * half of the SAE exchange.  The peer takes it through MLME_SEND_FRAME_REQ like
 * an EAPOL frame, only with the 802.11 descriptor and the management message
 * type; the channel is the one being joined, since the peer is sitting on it.
 */
static int wlbt_cfg_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
			    struct cfg80211_mgmt_tx_params *params, u64 *cookie)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 30] = { };
	struct wlbt_fapi_hdr hdr = { };
	u16 freq = params->chan ? params->chan->center_freq : w->connect_freq;
	bool auth;

	int ret;

	dev_info(w->dev, "mgmt_tx: called len %zu type 0x%02x connected %d peer_dead %d\n",
		 params->len, params->len ? params->buf[0] : 0,
		 w->connected, w->peer_dead);

	if (w->peer_dead)
		return -EIO;

	/*
	 * Ask for the interface the vendor uses for frames sent before the
	 * station is on a network; the peer refuses to transmit them from the
	 * station interface and reports it as FAPI_RESULTCODE 0x0001.
	 */
	if (!w->connected) {
		if (mgmt_tx_vif != 1) {
			wlbt_unsync_vif_up(w, freq);
		}
	}

	put_unaligned_le16(w->unsync_vif ? w->unsync_index : 1,
			   sig + sizeof(hdr) + 0);
	put_unaligned_le16(w->host_tag += 4, sig + sizeof(hdr) + 2);
	put_unaligned_le16(FAPI_DATAUNIT_IEEE802_11_FRAME,
			   sig + sizeof(hdr) + 4);
	/*
	 * The message type depends on the frame: the vendor sends authentication
	 * with the management type and everything else as an action frame
	 * (cfg80211_ops.c:slsi_wlan_mgmt_tx, "Transmit on the current
	 * frequency").  Sending authentication as an action frame is what made
	 * the peer accept our SAE commit with result 0 and then drop it.
	 */
	auth = (params->buf[0] & 0x00fc) == 0x00b0;	/* management, auth */
	if (auth && params->len >= 32 &&
	    get_unaligned_le16(params->buf + 26) == 2) {
		w->sae_sent_confirm = true;
		dev_info(w->dev, "sae: our confirm hash %*phN\n",
			 (int)min(params->len - 30, 32u), params->buf + 30);
	}
	put_unaligned_le16(auth ? mgmt_tx_auth_type :
			   FAPI_MESSAGETYPE_IEEE80211_ACTION,
			   sig + sizeof(hdr) + 6);
	if (auth && mgmt_tx_auth_type == FAPI_MESSAGETYPE_IEEE80211_MGMT) {
		/* Authentication, as the vendor sends it: the current frequency,
		 * no dwell time.
		 */
		if (mgmt_tx_auth_dwell_us > 0)
			put_unaligned_le32(mgmt_tx_auth_dwell_us,
					   sig + sizeof(hdr) + 10);
	} else if (mgmt_tx_freq_mode ||
		   (params->chan && freq != w->connect_freq)) {
		put_unaligned_le16(freq * 2, sig + sizeof(hdr) + 8);
		if (mgmt_tx_freq_mode == 2)
			put_unaligned_le32(mgmt_tx_dwell_ms * 1000,
					   sig + sizeof(hdr) + 10);
	}

	hdr.id = cpu_to_le16(FAPI_MLME_SEND_FRAME_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	*cookie = w->mgmt_cookie++;
	dev_info(w->dev, "mgmt_tx: %zu bytes type 0x%02x freq %u\n",
		 params->len, params->len ? params->buf[0] : 0, freq);
	WRITE_ONCE(w->stall_deadline, jiffies + msecs_to_jiffies(4000));
	wlbt_trace(w, "mgmt_tx: %zu bytes type 0x%02x before the send",
		   params->len, params->len ? params->buf[0] : 0);
	ret = wlbt_hip4_send(w, true, sig, sizeof(sig), params->buf, params->len);
	wlbt_trace(w, "mgmt_tx: send returned %d", ret);
	WRITE_ONCE(w->stall_deadline, 0);
	if (ret)
		return ret;

	if (!params->dont_wait_for_ack)
		cfg80211_mgmt_tx_status(wdev, *cookie, params->buf, params->len,
					true, GFP_ATOMIC);

	return 0;
}

/*
 * The supplicant's verdict on the SAE exchange, passed to the peer so it can
 * associate.  The status is the 802.11 status code it reports.
 */
static int wlbt_cfg_external_auth(struct wiphy *wiphy, struct net_device *ndev,
				  struct cfg80211_external_auth_params *params)
{
	struct exynos_wlbt *w = wlbt_from_wiphy(wiphy);

	w->sae_pending = false;
	/*
	 * The signal is fourteen bytes - interface, result code, the access
	 * point's address and one spare word (fapi.h: mlme_synchronised_res) -
	 * and the peer checks the size it is given.  This used to send sixteen,
	 * which is what it was handed when the answer follows a completed SAE,
	 * exactly where the peer stops answering.
	 */
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 14] = { };
	struct wlbt_fapi_hdr hdr = { };

	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	put_unaligned_le16(params->status, sig + sizeof(hdr) + 2);
	memcpy(sig + sizeof(hdr) + 4, params->bssid, ETH_ALEN);

	hdr.id = cpu_to_le16(FAPI_MLME_SYNCHRONISED_RES);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	dev_info(w->dev, "sae: supplicant finished with status %u for %pM\n",
		 params->status, params->bssid);
	wlbt_unsync_vif_down(w);

	if (w->sae_defer) {
		w->sae_defer = false;
		if (!params->status && w->defer_sig_len) {
			dev_info(w->dev,
				 "sae: exchange done, issuing the held join\n");
			wlbt_hip4_send(w, true, w->defer_sig,
				       w->defer_sig_len, w->defer_dat,
				       w->defer_dat_len);
		}
	}

	wlbt_trace(w, "external_auth: finished status %u", params->status);
	return wlbt_hip4_send(w, true, sig, sizeof(sig), NULL, 0);
}

/*
 * cfg80211 only forwards a management frame to a listener that the wiphy says
 * the interface may receive, so the station entry has to name the two kinds
 * that matter here: authentication frames for SAE, and action frames, which is
 * what a frame subscription defaults to.
 */
const struct ieee80211_txrx_stypes
wlbt_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
		.tx = BIT(IEEE80211_STYPE_AUTH >> 4) |
		      BIT(IEEE80211_STYPE_ACTION >> 4) |
		      BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
		.rx = BIT(IEEE80211_STYPE_AUTH >> 4) |
		      BIT(IEEE80211_STYPE_ACTION >> 4),
	},
};

const struct cfg80211_ops wlbt_cfg80211_ops = {
	.scan = wlbt_cfg_scan,
	.connect = wlbt_cfg_connect,
	.disconnect = wlbt_cfg_disconnect,
	.add_key = wlbt_cfg_add_key,
	.set_default_key = wlbt_cfg_set_default_key,
	.del_key = wlbt_cfg_del_key,
	.set_pmk = wlbt_cfg_set_pmk,
	.del_pmk = wlbt_cfg_del_pmk,
	.mgmt_tx = wlbt_cfg_mgmt_tx,
	.external_auth = wlbt_cfg_external_auth,
};

/*
 * The data path is not there yet, so the interface reports no carrier and
 * drops whatever it is handed.  It still has to exist: registering a netdev
 * goes through netdev_ops whatever the driver is able to do afterwards.
 */

