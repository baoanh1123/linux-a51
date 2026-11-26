/* SPDX-License-Identifier: GPL-2.0 */
#include "exynos9610-wlbt.h"

static int wlbt_net_open(struct net_device *ndev)
{
	struct exynos_wlbt *w = wlbt_from_netdev(ndev);

	if (w->peer_dead) {
		netif_carrier_off(ndev);
		return -EIO;
	}

	/*
	 * The interface is usually brought up before the join, but it can also be
	 * cycled afterwards, and the carrier then reflects the link that already
	 * exists rather than being cleared.
	 */
	if (w->connected)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int wlbt_net_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	return 0;
}

/*
 * Which message type an EAPOL key frame is, by the vendor's rule (tx.c): the
 * frame that ends the handshake carries no ACK, is a pairwise frame and has the
 * secure bit set - or, for a WPA1 key, the MIC bit and no key data.
 */
static u16 wlbt_eapol_message_type(const u8 *frame, size_t len)
{
	const u8 *eapol;

	if (len < ETH_HLEN + 99)
		return FAPI_MESSAGETYPE_EAPOL_KEY_M123;
	eapol = frame + ETH_HLEN;
	if (eapol[1] != 3)	/* 802.1X type: EAPOL-Key */
		return FAPI_MESSAGETYPE_EAPOL_KEY_M123;
	if (!(eapol[6] & 0x80) && (eapol[6] & 0x08) &&
	    ((eapol[4] == 2 && (eapol[5] & 0x02)) ||
	     (eapol[4] == 254 && (eapol[5] & 0x01) &&
	      !eapol[97] && !eapol[98])))
		return FAPI_MESSAGETYPE_EAPOL_KEY_M4;
	return FAPI_MESSAGETYPE_EAPOL_KEY_M123;
}

/*
 * One EAPOL frame, handed to the MLME: interface, a host tag, the descriptor
 * saying the payload is an 802.3 frame, the message type, and a dwell time the
 * vendor leaves at zero for key frames.
 */
static int wlbt_mlme_send_eapol(struct exynos_wlbt *w, const u8 *frame,
				 size_t len)
{
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 30] = { };
	struct wlbt_fapi_hdr hdr = { };

	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	put_unaligned_le16(w->host_tag += 4, sig + sizeof(hdr) + 2);
	put_unaligned_le16(FAPI_DATAUNIT_IEEE802_3_FRAME, sig + sizeof(hdr) + 4);
	put_unaligned_le16(wlbt_eapol_message_type(frame, len),
			   sig + sizeof(hdr) + 6);

	hdr.id = cpu_to_le16(FAPI_MLME_SEND_FRAME_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));

	return wlbt_hip4_send(w, true, sig, sizeof(sig), frame, len);
}

/*
 * The peer takes ethernet frames and does the 802.11 conversion itself, so a
 * transmit is the frame handed over on the data queue with a MA_UNITDATA_REQ in
 * front of it.  The body names the interface, a tag the confirm echoes, the
 * contention priority and the peer, and says the payload is an 802.3 frame.
 */
static netdev_tx_t wlbt_net_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct exynos_wlbt *w = wlbt_from_netdev(ndev);
	u8 sig[sizeof(struct wlbt_fapi_hdr) + 24] = { };
	struct wlbt_fapi_hdr hdr = { };
	int ret;

	if (!w->hip4_control || skb->len < ETH_HLEN) {
		dev_kfree_skb(skb);
		w->n_tx_dropped++;
		return NETDEV_TX_OK;
	}

	/*
	 * The access point sends the first message of the four-way exchange as
	 * soon as the association completes, which is before the peer tells us
	 * the connection is up: refusing everything until then drops the
	 * supplicant's answer and the handshake never starts.
	 */
	if (!w->connected && skb->protocol != htons(ETH_P_PAE)) {
		dev_kfree_skb(skb);
		w->n_tx_dropped++;
		return NETDEV_TX_OK;
	}

	w->n_tx_calls++;

	/*
	 * EAPOL has to get out through this function - it is how the handshake is
	 * completed - but nothing else may reach the peer before it holds the
	 * pairwise key.  Dropping here rather than stopping the queue keeps the
	 * supplicant's own frames flowing.
	 */
	if (!w->port_authorized && skb->protocol != htons(ETH_P_PAE)) {
		dev_kfree_skb(skb);
		w->n_tx_unauthed++;
		return NETDEV_TX_OK;
	}

	if (eapol_mlme && skb->protocol == htons(ETH_P_PAE)) {
		ret = wlbt_mlme_send_eapol(w, skb->data, skb->len);
		if (ret) {
			w->n_tx_dropped++;
			dev_info_ratelimited(w->dev, "tx: eapol send failed: %d\n", ret);
		} else {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;
		}
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	hdr.id = cpu_to_le16(FAPI_MA_UNITDATA_REQ);
	hdr.sender_pid = cpu_to_le16(FAPI_SENDER_PID);
	memcpy(sig, &hdr, sizeof(hdr));
	/*
	 * The header is the packed ten byte signal header, and the body of a
	 * unitdata request starts with the interface index, so this word is both
	 * the header's vif and the first word of the body.
	 */
	put_unaligned_le16(1, sig + sizeof(hdr) + 0);		/* vif */
	put_unaligned_le16(w->host_tag += 4, sig + sizeof(hdr) + 2);
	put_unaligned_le16(FAPI_PRIORITY_CONTENTION, sig + sizeof(hdr) + 4);
	put_unaligned_le16(1, sig + sizeof(hdr) + 6);		/* peer index */
	put_unaligned_le16(FAPI_DATAUNIT_IEEE802_3, sig + sizeof(hdr) + 8);

	ret = wlbt_hip4_send(w, false, sig, sizeof(sig), skb->data, skb->len);
	if (ret) {
		w->n_tx_dropped++;
		dev_info_ratelimited(w->dev, "tx: send failed: %d\n", ret);
	} else {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
	}

	if (skb->protocol == htons(ETH_P_PAE))
		dev_info(w->dev, "eapol tx: %u bytes ret %d auth %d\n",
			 skb->len, ret, w->port_authorized);

	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops wlbt_netdev_ops = {
	.ndo_open = wlbt_net_open,
	.ndo_stop = wlbt_net_stop,
	.ndo_start_xmit = wlbt_net_xmit,
};

static const u8 wlbt_mac[ETH_ALEN] = { 0x8c, 0x77, 0x12, 0xaa, 0xbb, 0xcc };

int wlbt_register_wiphy(struct exynos_wlbt *w)
{
	struct wireless_dev *wdev;
	struct net_device *ndev;
	struct wiphy *wiphy;
	int ret;

	wiphy = wiphy_new(&wlbt_cfg80211_ops, sizeof(struct exynos_wlbt *));
	wlbt_sae_wiphy = wiphy;
	if (!wiphy)
		return -ENOMEM;

	*(struct exynos_wlbt **)wiphy_priv(wiphy) = w;
	set_wiphy_dev(wiphy, w->dev);
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->max_scan_ssids = 1;
	/* A supplicant that attaches IEs to its scan request gets -EINVAL from
	 * cfg80211 when this is left at zero, which looks like a driver that
	 * cannot scan at all. */
	wiphy->max_scan_ie_len = 512;
	wiphy->bands[NL80211_BAND_2GHZ] = &wlbt_band_2ghz;
	wiphy->bands[NL80211_BAND_5GHZ] = &wlbt_band_5ghz;
	wiphy->cipher_suites = wlbt_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(wlbt_cipher_suites);
	wiphy->max_num_akm_suites = 8;
	/* The supplicant names the EAPOL ethertype it wants; accepting the
	 * attribute is enough, EAPOL leaves through the MLME path.
	 */
	wiphy->flags |= WIPHY_FLAG_CONTROL_PORT_PROTOCOL;

	/*
	 * Only claim SAE when asked to.  The host side works: the peer hands the
	 * access point's authentication frames up, the supplicant runs the
	 * exchange, and its answers leave through the management transmit.  A
	 * failed join can still end in AUTH_TX_FAIL and leave the peer stopped
	 * until the board is power cycled, so the boot default stays off and the
	 * flag is turned on for a WPA3 network.  See notes/wifi.md.
	 */
	if (sae_support)
		wiphy->features |= NL80211_FEATURE_SAE;
	/*
	 * The peer runs the 4-way handshake, so the supplicant has to hand the
	 * PMK over instead of keeping it: without this it derives the key from
	 * a completed SAE exchange and never tells anyone, and the join is
	 * refused for want of a key.
	 */
	if (fourway_offload)
		wiphy_ext_feature_set(wiphy,
				      NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	if (fourway_1x)
		wiphy_ext_feature_set(wiphy,
				      NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_1X);
	wiphy->mgmt_stypes = wlbt_mgmt_stypes;

	ret = wiphy_register(wiphy);
	if (ret)
		goto err_free;

	ndev = alloc_netdev(sizeof(*wdev), "wlan%d", NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		ret = -ENOMEM;
		goto err_unregister;
	}

	wdev = netdev_priv(ndev);
	wdev->wiphy = wiphy;
	wdev->iftype = NL80211_IFTYPE_STATION;
	ndev->ieee80211_ptr = wdev;
	ndev->netdev_ops = &wlbt_netdev_ops;
	SET_NETDEV_DEV(ndev, w->dev);
	eth_hw_addr_set(ndev, wlbt_mac);

	/* Registering an interface from a driver's own probe means taking the
	 * two locks cfg80211 expects its callers to hold; without them the netdev
	 * is created outside RTNL and the add never reaches userspace in a shape
	 * NetworkManager accepts.
	 */
	rtnl_lock();
	wiphy_lock(wiphy);
	ret = cfg80211_register_netdevice(ndev);
	wiphy_unlock(wiphy);
	rtnl_unlock();
	if (ret) {
		free_netdev(ndev);
		goto err_unregister;
	}

	w->wiphy = wiphy;
	w->netdev = ndev;
	INIT_DELAYED_WORK(&w->scan_work, wlbt_scan_work);
	INIT_DELAYED_WORK(&w->connect_work, wlbt_connect_work);
	timer_setup(&w->stall_timer, wlbt_stall_check, 0);
	INIT_WORK(&w->stall_report, wlbt_stall_report);
	mod_timer(&w->stall_timer, jiffies + msecs_to_jiffies(500));
	INIT_WORK(&w->peer_work, wlbt_peer_work);
	skb_queue_head_init(&w->eapol_hold);
	INIT_WORK(&w->peer_gone_work, wlbt_peer_gone_work);
	INIT_WORK(&w->sae_work, wlbt_sae_work);
	spin_lock_init(&w->sae_q_lock);
	INIT_DELAYED_WORK(&w->sae_retry_work, wlbt_sae_retry_work);
	INIT_WORK(&w->scan_rep_work, wlbt_scan_rep_work);
	spin_lock_init(&w->scan_rep_lock);
	dev_info(w->dev, "registered %s\n", netdev_name(ndev));
	return 0;

err_unregister:
	wiphy_unregister(wiphy);
err_free:
	wlbt_sae_wiphy = NULL;
	wiphy_free(wiphy);
	return ret;
}

void wlbt_unregister_wiphy(struct exynos_wlbt *w)
{
	if (!w->wiphy)
		return;

	cancel_delayed_work_sync(&w->scan_work);
	cancel_work_sync(&w->peer_gone_work);
	cancel_work_sync(&w->sae_work);
	cancel_delayed_work_sync(&w->connect_work);
	if (w->scan_req) {
		struct cfg80211_scan_info info = { .aborted = true };

		cfg80211_scan_done(w->scan_req, &info);
		w->scan_req = NULL;
	}
	if (w->netdev) {
		rtnl_lock();
		wiphy_lock(w->wiphy);
		cfg80211_unregister_netdevice(w->netdev);
		wiphy_unlock(w->wiphy);
		rtnl_unlock();
		free_netdev(w->netdev);
		w->netdev = NULL;
	}
	wiphy_unregister(w->wiphy);
	wiphy_free(w->wiphy);
	w->wiphy = NULL;
}

/*
 * A warm reboot does not reset the WLBT block: its power configuration lives in
 * the PMU and the peer keeps running across the reset of the SoC.  Leaving it
 * like that is how a boot turns into a hang, so shut the peer down on the way
 * out as well.
 */

