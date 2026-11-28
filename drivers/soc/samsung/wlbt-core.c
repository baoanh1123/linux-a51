/* SPDX-License-Identifier: GPL-2.0 */
#include "exynos9610-wlbt.h"

const u8 wlbt_hip4_idx[HIP4_Q_NUM][2] = {
	{ 0, HIP4_SCB_FW_OWN + 0 },		/* from host, control */
	{ 1, HIP4_SCB_FW_OWN + 1 },		/* from host, data */
	{ HIP4_SCB_FW_OWN + 2, 2 },		/* from host, freed */
	{ HIP4_SCB_FW_OWN + 3, 3 },		/* to host, control */
	{ HIP4_SCB_FW_OWN + 4, 4 },		/* to host, data */
	{ 5, HIP4_SCB_FW_OWN + 5 },		/* to host, freed */
};

const struct wlbt_pbus_window wlbt_pbus_windows[] = {
	{ 0xa0000000, 0xa0010000, 0x11950000 },
	{ 0xa0010000, 0xa0060000, 0x119a0000 },
	{ 0xa0060000, 0xa0100000, 0x11d00000 },
	{ 0xa0100000, 0xa0110000, 0x11c80000 },
	{ 0xa0110000, 0xa0120000, 0x11c20000 },
	{ 0xa0120000, 0xa0160000, 0x11200000 },
};

const u32 wlbt_rom_patch[] = {
	0x90750002, 0x11a4c218, 0x75671191, 0x9075e090, 0x54b3e5e7, 0x30f76001,
	0xb14315a2, 0xb4b2e503, 0xb153fb03, 0xa90185fc, 0xacf50774, 0x75fdadb5,
	0xb47508a0, 0x54b3e501, 0x75fa7001, 0x907500b4, 0x78cb8018, 0x80837982,
	0x07a075c5, 0xb0783779, 0xb40754e6, 0x0b800207, 0xc404f6d9, 0xaf7590f5,
	0x75038000, 0x53229090, 0xce53eff7, 0xd90479fe, 0xfdce53fe, 0xfed90c79,
	0x75fbce53, 0x91530b92, 0xf7ce53fd, 0x5308f943, 0xf922fef9, 0xfbd8fed9,
	0x019e7522, 0x75cfc175, 0xc375a4c2, 0x47c4754a, 0x75a4c575, 0xc7756dc6,
	0x03d27540, 0x7510d375, 0xca7500c9, 0x00cb75d0, 0x7500cc75, 0x9b75009a,
	0x009c75c0, 0x78009d75, 0x12827402, 0xc6438b80, 0x74097802, 0x8b8012e7,
	0x75d09075, 0x9e750291, 0x01a97502, 0x00000022,
};

char *firmware_name = "mx140/mx140.bin";
module_param(firmware_name, charp, 0444);
MODULE_PARM_DESC(firmware_name, "Firmware image for the WLBT subsystem");

/*
 * A station's group key is addressed to the access point, exactly like the
 * pairwise key, so a peer that indexed its key database by address alone would
 * keep only the last one written.  It does not - the pairwise key lands in a
 * key context of its own - but keeping the two installs separable is what made
 * that visible.
 */
/*
 * The peer has an EAPOL key handler of its own that can install a key without
 * the host, so the host's installs have to be separable from its handshake.
 */
/*
 * The vendor answers MLME_CONNECT_IND when the four way handshake's M4 arrives,
 * not when the peer announces the association, so the answer can be repeated at
 * the moment the pairwise key lands.
 */
/*
 * The vendor answers the connect from the M4 path, i.e. after the four way
 * handshake, not when the association is announced.  If the peer settles the
 * link's security when it is answered, answering first and installing keys
 * afterwards is the wrong order.
 */
/*
 * The vendor's MLME_ADD_INFO_ELEMENTS carries only the IEs it cannot express
 * itself - interworking, extension and vendor ones - and skips the request
 * entirely when there are none.  The RSN IE travels in the connect request
 * instead, which is the copy that also reaches the air.
 */
/*
 * The vendor appends only the RSN and RSNX elements to the connect request,
 * not the whole set the supplicant handed over: the peer parses that field as
 * the link's security.
 */
/*
 * The vendor tells the peer the station's address once the handshake is done
 * (slsi_ip_address_changed from the M4 path), which is the last thing it does
 * to the link before traffic is expected to flow.
 */
char *set_ip;
module_param(set_ip, charp, 0644);
MODULE_PARM_DESC(set_ip, "Send MLME_SET_IP_ADDRESS with this address once the pairwise key is installed");

bool sec_ie_only;
module_param(sec_ie_only, bool, 0644);
MODULE_PARM_DESC(sec_ie_only, "Append only the RSN and RSNX elements to the connect request");

bool info_elements = true;
module_param(info_elements, bool, 0644);
MODULE_PARM_DESC(info_elements, "Send the association IEs with MLME_ADD_INFO_ELEMENTS");

bool connect_res_late;
module_param(connect_res_late, bool, 0644);
MODULE_PARM_DESC(connect_res_late, "Answer MLME_CONNECT_IND only once the pairwise key is installed");

bool connect_res_at_key;
module_param(connect_res_at_key, bool, 0644);
MODULE_PARM_DESC(connect_res_at_key, "Answer MLME_CONNECT_IND again when the pairwise key is installed");

/*
 * A key installed before the join, with a placeholder body, tells whether the
 * peer decides the link's security when it joins (in which case a key database
 * filled afterwards never reaches the data path) or when the key arrives.
 */
bool prekey;
module_param(prekey, bool, 0644);
MODULE_PARM_DESC(prekey, "Install a placeholder pairwise key before the connect");

bool host_keys = true;
module_param(host_keys, bool, 0644);
MODULE_PARM_DESC(host_keys, "Send the supplicant's keys to the peer");

bool install_gtk = true;
module_param(install_gtk, bool, 0644);
MODULE_PARM_DESC(install_gtk, "Install the group key the supplicant hands over");

/*
 * The peer accepts a pairwise key with a success confirmation and then keeps
 * no key the data path can find, while the same request for a group key is
 * installed properly.  These two knobs change only the key type and the key
 * index of the pairwise install, so that path can be compared with the one
 * that works without going through the debugfs send path, which the peer does
 * not treat the same way as a request from the driver.
 */
uint ptk_type = FAPI_KEYTYPE_PAIRWISE;
module_param(ptk_type, uint, 0644);
MODULE_PARM_DESC(ptk_type, "Key type to use for the pairwise key (diagnostic)");

uint ptk_id;
module_param(ptk_id, uint, 0644);
MODULE_PARM_DESC(ptk_id, "Key index to use for the pairwise key, 0 keeps the supplicant's");

/*
 * The peer inspects an EAPOL key frame itself and can issue MLME_SETKEYS from
 * its own copy of the link, so the PMK hand-off is separable too.
 */
bool send_pmk = true;
module_param(send_pmk, bool, 0644);
MODULE_PARM_DESC(send_pmk, "Hand the PMK to the peer for its own 4-way handshake");

static bool load_firmware;
module_param(load_firmware, bool, 0644);
MODULE_PARM_DESC(load_firmware, "Copy the firmware image into MIF DRAM and let the peer run it");

static bool wlan_start_on_boot = true;
module_param(wlan_start_on_boot, bool, 0644);
MODULE_PARM_DESC(wlan_start_on_boot, "Start the peer's WLAN service once the firmware runs (default: on)");

/*
 * EAPOL does not travel on the data queue.  The vendor driver hands key
 * frames to the MLME instead (tx.c: slsi_tx_eapol() ends in
 * slsi_mlme_send_frame_data()), and the message type tells the peer which of
 * them finishes a handshake - M4 - and which of them starts one.  The peer runs
 * the 4-way state machine itself and only settles a link whose M4 arrived this
 * way, which is why the pairwise key never landed while these frames went out
 * as ordinary data.
 */

/*
 * WPA3 is not the peer's to finish on its own.  The join goes out with
 * FAPI_AUTHENTICATIONTYPE_SAE, and the peer then runs the exchange with its own
 * MLME but hands every authentication frame it receives up in an
 * MLME_SYNCHRONISED_IND: the supplicant owns the derivation, so it must see the
 * frame (cfg80211_rx_mgmt) and be told to start (cfg80211_external_auth_request).
 * Its answer comes back through NL80211_CMD_EXTERNAL_AUTH and is passed on as
 * MLME_SYNCHRONISED_RES, after which the peer associates and the 4-way runs as
 * it does for WPA2.
 *
 * The vendor driver is built the same way (CONFIG_SCSC_WLAN_SAE_CONFIG):
 * cfg80211_ops.c sets NL80211_FEATURE_SAE, rx.c turns MLME_SYNCHRONISED_IND into
 * an external auth request, and mlme.c answers it with MLME_SYNCHRONISED_RES.
 */
/*
 * The vendor keeps a second interface for the frames a station has to send
 * before it is on a network: SAE authentication frames.  It creates one with
 * FAPI_VIFTYPE_UNSYNCHRONISED, registers the action frame categories it wants
 * and puts it on the channel (mgt.c: slsi_wlan_unsync_vif_activate).  The index
 * is this driver's own - the vendor reuses the station's because in its flow
 * that one does not exist yet, while here it is already carrying the join.
 */
/*
 * What the vendor registers for a station: public, WMM, WNM, QoS, protected
 * dual and radio measurement (dev.h: SLSI_STA_ACTION_FRAME_BITMAP), with radio
 * measurement left out of the suspended set.  Registration is part of bringing
 * the interface up there, and this driver was skipping it.
 */


unsigned int mgmt_tx_freq_mode;
module_param(mgmt_tx_freq_mode, uint, 0644);
MODULE_PARM_DESC(mgmt_tx_freq_mode, "Management frame transmits: 0 leaves the frequency zero (send on the channel the interface is on), 1 always names the channel, 2 names it and asks for the dwell time");

/*
 * What the vendor puts in an authentication frame, read off its own logring
 * capture of an SAE join (the vendor driver copies every control frame into it
 * and tools/testing/vendor-samlog.py decodes it):
 *
 *	vif=1 data_unit=0x0000 message_type=0x0011 freq=0 dwell=0 period=0
 *
 * data unit 0x0000 is the 802.11 frame descriptor and 0x0011 the management
 * message type; the dwell time is zero, not the supplicant's wait.  Sending the
 * same frame with a non-zero dwell is what made the peer reset before.
 */
unsigned int mgmt_tx_auth_type = FAPI_MESSAGETYPE_IEEE80211_MGMT;
module_param(mgmt_tx_auth_type, uint, 0644);
MODULE_PARM_DESC(mgmt_tx_auth_type, "Message type authentication frames are sent with: 0x10 action or 0x11 management (the default, and what the vendor sends)");

int mgmt_tx_auth_dwell_us;
module_param(mgmt_tx_auth_dwell_us, int, 0644);
MODULE_PARM_DESC(mgmt_tx_auth_dwell_us, "Dwell time in microseconds for authentication frames; 0 is what the vendor sends");

unsigned int mgmt_tx_dwell_ms = 100;
module_param(mgmt_tx_dwell_ms, uint, 0644);
MODULE_PARM_DESC(mgmt_tx_dwell_ms, "Dwell time asked for when mgmt_tx_freq_mode names the channel");

unsigned int mgmt_tx_vif = 3;
module_param(mgmt_tx_vif, uint, 0644);
MODULE_PARM_DESC(mgmt_tx_vif, "Which interface management frames leave by before the station is on a network: 2 a second, unsynchronised interface, 3 the station's own index turned unsynchronised the way the vendor does it, 1 the station interface as it is");

bool sae_support;

/*
 * Advertising SAE is read when the wiphy registers, so a boot-time
 * advertisement cannot be taken back and every boot repeats it.  That is what
 * turned one failed join into a boot loop.  Keep the boot default off and let
 * the flag be turned on afterwards, on a wiphy that is already up.
 */
struct wiphy *wlbt_sae_wiphy;

static int wlbt_sae_support_set(const char *val, const struct kernel_param *kp)
{
	bool on;
	int ret = kstrtobool(val, &on);

	if (ret)
		return ret;
	WRITE_ONCE(sae_support, on);
	if (wlbt_sae_wiphy) {
		if (on)
			wlbt_sae_wiphy->features |= NL80211_FEATURE_SAE;
		else
			wlbt_sae_wiphy->features &= ~NL80211_FEATURE_SAE;
		dev_info(&wlbt_sae_wiphy->dev, "sae: support %s\n",
			 on ? "advertised" : "withdrawn");
	}
	return 0;
}

static const struct kernel_param_ops wlbt_sae_support_ops = {
	.set = wlbt_sae_support_set,
	.get = param_get_bool,
};
module_param_cb(sae_support, &wlbt_sae_support_ops, &sae_support, 0644);
MODULE_PARM_DESC(sae_support, "Advertise WPA3/SAE support.  Off at boot; can be turned on at runtime, because a boot-time advertisement is retried on every boot");

bool eapol_mlme = true;
module_param(eapol_mlme, bool, 0644);
MODULE_PARM_DESC(eapol_mlme, "Send EAPOL key frames through MLME_SEND_FRAME_REQ like the vendor driver");

bool hip4_return = true;
module_param(hip4_return, bool, 0644);
MODULE_PARM_DESC(hip4_return, "Hand received HIP4 segments back to the peer");

/*
 * Diagnostic: acknowledge the peer's doorbells and drain nothing.  A send that
 * resets the peer also resets the machine within microseconds, and this is how
 * the two are told apart: with the receive paths off, a reset after the send
 * can only come from the peer side.
 */
bool rx_off;
module_param(rx_off, bool, 0644);
MODULE_PARM_DESC(rx_off, "Acknowledge mailbox interrupts without draining any stream (diagnostic)");

/*
 * Diagnostic: service the queues but leave the frames a received-frame
 * indication carries where they are.  The queue still moves, so the peer stays
 * happy, and a reset that only follows when the frame is read points at the
 * extraction.
 */
bool rx_ind_off;
module_param(rx_ind_off, bool, 0644);
MODULE_PARM_DESC(rx_ind_off, "Do not take frames out of MLME_RECEIVED_FRAME_IND (diagnostic)");

/*
 * Handing the frames in a received-frame chain to the network stack happens
 * inside the drain, with the queue lock held and interrupts off, and whatever
 * the stack sends in reply goes back out through this driver.  Off by default:
 * the chain is read for the management frame and nothing else.
 */
bool rx_chain;
module_param(rx_chain, bool, 0644);
MODULE_PARM_DESC(rx_chain, "Follow the chain of a received-frame indication, and hand its ethernet frames to the stack");

/*
 * The indication names the frame with a word in its body, and that is how a
 * synchronisation indication hands over the beacon or probe response.
 */
bool rx_ptr = true;
module_param(rx_ptr, bool, 0644);
MODULE_PARM_DESC(rx_ptr, "Follow the frame pointer in a received-frame indication (default: on)");

/*
 * Read the segment a received-frame indication chains to, and keep it for the
 * dump.  Nothing is delivered and nothing is parsed, so a freeze here means the
 * read itself is what the peer will not tolerate.
 */
/*
 * Taking the frame out of a chain and delivering it is what a hard freeze has
 * been reproduced on twice, so it is separate from reading the chain (which is
 * safe) and from handing the chain's ethernet frames to the stack.
 */
bool rx_chain_frame;
bool fourway_offload;
bool sae_defer_join;
bool sae_join_retry;
module_param(sae_join_retry, bool, 0644);
MODULE_PARM_DESC(sae_join_retry, "Re-issue a join the peer abandoned during an SAE exchange");
module_param(sae_defer_join, bool, 0644);
MODULE_PARM_DESC(sae_defer_join, "Hold the peer's join until the SAE exchange finishes instead of letting it time out mid-exchange");
module_param(fourway_offload, bool, 0644);
MODULE_PARM_DESC(fourway_offload, "Advertise that the device runs the 4-way handshake, so the supplicant hands over the PMK instead of doing the handshake itself");
/*
 * The PSK flavour of that flag is not enough for SAE: wpa_supplicant only
 * forwards the PMK (set_key with KEY_FLAG_PMK) when the driver advertises the
 * 802.1X variant, and silently drops it otherwise - "The driver does not have
 * any offload mechanism for PMK, so there is no need to configure this key".
 * Kept separate so it can be turned on without changing the PSK behaviour.
 */
bool fourway_1x;
module_param(fourway_1x, bool, 0644);
MODULE_PARM_DESC(fourway_1x, "Advertise 802.1X 4-way handshake offload, which is what makes a supplicant hand the SAE PMK over");
module_param(rx_chain_frame, bool, 0644);
MODULE_PARM_DESC(rx_chain_frame, "Take the frame out of a received-frame chain and deliver it");

bool rx_chain_probe;
module_param(rx_chain_probe, bool, 0644);
MODULE_PARM_DESC(rx_chain_probe, "Copy the chained segment of a received-frame indication into the dump");

/*
 * The receive queue is walked with the queue lock held and interrupts off, so
 * a frame by frame trace is only printed when it is asked for: at a few hundred
 * frames the console time alone is enough to lose the mailbox interrupts.
 */
/*
 * Data frames are sent on the data queue, which is what the class in the bulk
 * header says; the control queue is an experiment because the peer does not
 * consume the data queue yet.
 */
bool hip4_log;
module_param(hip4_log, bool, 0644);
MODULE_PARM_DESC(hip4_log, "Record every received HIP4 frame for debugfs frames");

/*
 * The peer logs through the same transport as everything else, and a busy link
 * produces thousands of records a second.  Printing them from the drain stops
 * the machine for as long as the console takes, so they are counted always and
 * printed only when asked.
 */
bool fw_log;
module_param(fw_log, bool, 0644);
MODULE_PARM_DESC(fw_log, "Print the peer's own log records");

bool scan_log;
module_param(scan_log, bool, 0644);
MODULE_PARM_DESC(scan_log, "Print one line per access point a scan reports");

/* One line per control frame sent; cheap enough to leave on by default. */
bool tx_log = true;
module_param(tx_log, bool, 0644);
MODULE_PARM_DESC(tx_log, "Record every control frame sent for debugfs tx");

/*
 * The peer's log records are far too many to keep, but the ones that a burst
 * of traffic provokes come from one module.  Naming it keeps the ring for
 * those records alone, with all twelve words intact.
 */
uint log_module;
module_param(log_module, uint, 0644);
MODULE_PARM_DESC(log_module, "Keep the peer's log records from this module in the fault ring");

bool load_mib = true;
module_param(load_mib, bool, 0644);
MODULE_PARM_DESC(load_mib, "Hand the peer its hardware configuration files (MIB)");

/*
 * The peer calibrates its PHY against the chip revision the host reports, and
 * the vendor passes the value from the chipid block (mxman.c:1095).  The
 * register lives at 0x10000000 with the revision in bits 23:16 for this
 * family (drv_data_exynos9 in the vendor chipid driver).
 */

static int soc_rev = -1;
module_param(soc_rev, int, 0644);
MODULE_PARM_DESC(soc_rev, "SoC revision reported to the peer (-1 reads the chipid block)");

u32 wlbt_soc_revision(struct exynos_wlbt *w)
{
	void __iomem *id;
	u32 val;

	if (soc_rev >= 0)
		return soc_rev;

	id = ioremap(WLBT_CHIPID_PHYS, 0x100);
	if (!id)
		return 0;
	val = readl(id + WLBT_CHIPID_REV);
	iounmap(id);
	dev_info(w->dev, "SoC revision 0x%02x (chipid 0x%08x)\n",
		 (val >> 16) & 0xff, val);

	/*
	 * The vendor packs it as (main_rev << 4) | sub_rev out of bits 23:16
	 * (exynos-chipid.c:167, drv_data_exynos9).
	 */
	return ((val >> 16) & 0xff);
}

unsigned int mib_slice_max = 1024;
module_param(mib_slice_max, uint, 0644);
MODULE_PARM_DESC(mib_slice_max, "Largest MIB batch handed over in one signal");

/*
 * The peer validates the reference in fw_reference against the HIP4 transmit
 * and receive windows only (0x802e44f0..0x802e44fc hold their bounds), so it
 * is a pointer into the segment memory rather than a cookie.  Send the
 * address of the segment the signal itself travels in.
 */
bool sig_self_ref;
module_param(sig_self_ref, bool, 0644);
MODULE_PARM_DESC(sig_self_ref, "Point fw_reference at the signal's own segment");

unsigned int send_ref;
int sig_refcnt = -1;
module_param(send_ref, uint, 0644);

/* Name a plain zeroed buffer instead of a segment in the firmware reference. */
bool hip4_ref_bare;

/* Override the mbulk reference count of frames sent on the datapath. */
module_param(sig_refcnt, int, 0644);
module_param(hip4_ref_bare, bool, 0644);
MODULE_PARM_DESC(send_ref, "fw_reference to put in signals sent from the hip4 node");

bool mib_early;
module_param(mib_early, bool, 0644);
MODULE_PARM_DESC(mib_early, "Download the MIB entries before the WLAN start request");

static char *trace_boot;
module_param(trace_boot, charp, 0444);
MODULE_PARM_DESC(trace_boot, "Sample one peer word from the moment the firmware boots: <hex offset>:<hex field>:<microseconds>");

static const u32 wlbt_pw_regs[WLBT_PW_REGS] = {
	CENTRAL_SEQ_WLBT_CONFIGURATION,
	RESET_AHEAD_WLBT_SYS_PWR_REG,
	CLEANY_BUS_WLBT_SYS_PWR_REG,
	LOGIC_RESET_WLBT_SYS_PWR_REG,
	TCXO_GATE_WLBT_SYS_PWR_REG,
	WLBT_DISABLE_ISO_SYS_PWR_REG,
	WLBT_RESET_ISO_SYS_PWR_REG,
};

static void wlbt_pw_cfg_save(struct exynos_wlbt *w)
{
	unsigned int i;

	for (i = 0; i < WLBT_PW_REGS; i++) {
		w->pw_cfg[i].reg = wlbt_pw_regs[i];
		if (regmap_read(w->pmu, wlbt_pw_regs[i], &w->pw_cfg[i].val))
			w->pw_cfg[i].val = 0;
	}
}

static void wlbt_pw_cfg_restore(struct exynos_wlbt *w)
{
	unsigned int i;

	if (!w->pw_cfg_dirty)
		return;

	for (i = 0; i < WLBT_PW_REGS; i++) {
		if (regmap_write(w->pmu, w->pw_cfg[i].reg, w->pw_cfg[i].val))
			dev_err(w->dev, "cannot restore PMU 0x%x to 0x%x\n",
				w->pw_cfg[i].reg, w->pw_cfg[i].val);
	}
	w->pw_cfg_dirty = false;
	dev_dbg(w->dev, "restored the WLBT sequencer configuration\n");
}

const char *wlbt_state_name(enum wlbt_state state)
{
	switch (state) {
	case WLBT_IN_RESET:
		return "in-reset";
	case WLBT_WAIT_CFG_REQ:
		return "wait-cfg-req";
	case WLBT_ACK_CFG_REQ:
		return "ack-cfg-req";
	case WLBT_CFG_DONE:
		return "cfg-done";
	case WLBT_CFG_ERROR:
		return "cfg-error";
	}

	return "?";
}

/*
 * Everything the vendor prints when the handshake goes wrong
 * (platform_mif_9610.c:715-733).  The state machine value is the useful one:
 * 0x80 means "WLBT is in reset", anything else means it is somewhere in its
 * boot sequence.
 */
static void wlbt_dump(struct exynos_wlbt *w, const char *why)
{
	u32 val;

	regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &val);
	dev_info(w->dev, "%s: CENTRAL_SEQ_WLBT_STATUS 0x%08x (state 0x%02x)\n",
		 why, val, (val & STATES) >> 16);
	regmap_read(w->pmu, WLBT_CTRL_NS, &val);
	dev_info(w->dev, "%s: WLBT_CTRL_NS 0x%08x\n", why, val);
	regmap_read(w->pmu, WLBT_CTRL_S, &val);
	dev_info(w->dev, "%s: WLBT_CTRL_S 0x%08x\n", why, val);
	regmap_read(w->pmu, WLBT_DEBUG, &val);
	dev_info(w->dev, "%s: WLBT_DEBUG 0x%08x\n", why, val);
	dev_info(w->dev, "%s: mailbox IS_VERSION 0x%08x INTSR0 0x%08x INTMSR0 0x%08x\n",
		 why, readl(w->mbox + MBOX_IS_VERSION),
		 readl(w->mbox + MBOX_INTSR0), readl(w->mbox + MBOX_INTMSR0));
}

/*
 * The WLBT TZPC is secure by default; the vendor makes it non-secure through
 * the EL3 monitor at the top of the CFG_REQ handler (platform_mif_9610.c:741).
 * The GNSS block does the same thing (DOWNSTREAM_READONLY/drivers/misc/
 * gnss_if/pmu-gnss.c:322), and our own GPU power domain driver proves that the
 * monitor is still there under uniLoader.
 */
static int wlbt_request_tzpc(struct exynos_wlbt *w)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_CONN_IF, ((u64)EXYNOS_WLBT << 32) |
		      EXYNOS_SET_CONN_TZPC, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_warn(w->dev, "TZPC SMC returned %ld\n", (long)res.a0);

	return res.a0;
}

/*
 * Reproduce the address space the peer expects: 16 MiB of DRAM at 0x80000000
 * over the data bus, and the peripheral windows on the peripheral bus
 * (platform_mif_9610.c:748-805).  Until this is done the peer cannot read its
 * own configuration, let alone firmware.
 */
static int wlbt_program_windows(struct exynos_wlbt *w)
{
	unsigned int i;
	u32 id;
	int ret;

	if (!w->mem_base || !w->mem_size) {
		dev_err(w->dev, "no MIF DRAM window\n");
		return -EINVAL;
	}
	if (w->mem_size < BAAW_0_END - BAAW_0_START) {
		dev_err(w->dev, "MIF DRAM too small: %pa size %pa\n",
			&w->mem_base, &w->mem_size);
		return -EINVAL;
	}

	ret = regmap_write(w->remap, REMAP_BAAW_0, BAAW_0_START >> 12);
	if (ret)
		return ret;

	/* Keep the version field, replace the EMA parameters. */
	ret = regmap_read(w->remap, REMAP_CHIP_VERSION_ID, &id);
	if (ret)
		return ret;
	id &= ~CHIP_VERSION_ID_EMA_MASK;
	id |= CHIP_VERSION_ID_EMA_VALUE;
	ret = regmap_write(w->remap, REMAP_CHIP_VERSION_ID, id);
	if (ret)
		return ret;

	ret = regmap_write(w->dbus_baaw, 0x00, BAAW_0_START >> 12);
	ret |= regmap_write(w->dbus_baaw, 0x04, BAAW_0_END >> 12);
	ret |= regmap_write(w->dbus_baaw, 0x08, w->mem_base >> 12);
	ret |= regmap_write(w->dbus_baaw, 0x0c, BAAW_ACCESS_CTRL);
	if (ret) {
		dev_err(w->dev, "DBUS_BAAW write failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(wlbt_pbus_windows); i++) {
		const struct wlbt_pbus_window *win = &wlbt_pbus_windows[i];

		ret = regmap_write(w->pbus_baaw, i * 0x10 + 0x00, win->start >> 12);
		ret |= regmap_write(w->pbus_baaw, i * 0x10 + 0x04, win->end >> 12);
		ret |= regmap_write(w->pbus_baaw, i * 0x10 + 0x08, win->target >> 12);
		ret |= regmap_write(w->pbus_baaw, i * 0x10 + 0x0c, BAAW_ACCESS_CTRL);
		if (ret) {
			dev_err(w->dev, "PBUS_BAAW window %u failed: %d\n", i, ret);
			return ret;
		}
	}

	dev_info(w->dev, "windows programmed: MIF DRAM %pa at 0x%08x\n",
		 &w->mem_base, BAAW_0_START);

	return 0;
}

/*
 * Answer the peer's configuration request (platform_mif_9610.c:806-846): hold
 * it, write the ROM patch, release it, then ACK.  The peer may crash as soon as
 * the ACK lands - the vendor notes exactly that - so the state is moved first.
 */
static int wlbt_cfg_ack(struct exynos_wlbt *w)
{
	unsigned int i;
	int ret;

	ret = regmap_write(w->boot_cfg, BOOT_CFG_HOLD, 1);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(wlbt_rom_patch); i++) {
		ret = regmap_write(w->boot_cfg, BOOT_CFG_ROM + i * 4,
				   wlbt_rom_patch[i]);
		if (ret)
			return ret;
	}

	ret = regmap_write(w->boot_cfg, BOOT_CFG_HOLD, 0);
	if (ret)
		return ret;

	w->state = WLBT_ACK_CFG_REQ;

	ret = regmap_write(w->boot_cfg, BOOT_CFG_ACK, 1);
	if (ret)
		return ret;

	dev_info(w->dev, "BOOT_CFG_ACK done\n");

	/* The peer clears CFG_REQ in response to the ACK; give it a moment so
	 * the level interrupt is gone before it is enabled again.
	 */
	udelay(100);

	/* Hand the power request back to the firmware, as the vendor does. */
	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, MASK_PWR_REQ, 0);
	if (ret)
		return ret;

	w->state = WLBT_CFG_DONE;

	return 0;
}

static irqreturn_t wlbt_cfg_req_isr(int irq, void *data)
{
	struct exynos_wlbt *w = data;
	int ret;

	disable_irq_nosync(irq);
	w->n_cfg_req++;

	/* A CFG_REQ out of nowhere means the previous shutdown left the peer
	 * running; the vendor refuses to touch the hardware after that.
	 */
	if (w->state != WLBT_WAIT_CFG_REQ) {
		w->n_cfg_bad++;
		dev_err(w->dev, "spurious CFG_REQ in state %s\n",
			wlbt_state_name(w->state));
		wlbt_dump(w, "spurious-cfg-req");
		w->state = WLBT_CFG_ERROR;
		complete(&w->cfg_ack);
		return IRQ_HANDLED;
	}

	dev_info(w->dev, "CFG_REQ received, configuring the peer\n");

	/* The CBUS should be ready when CFG_REQ arrives, but the vendor waits
	 * 100 us anyway (platform_mif_9610.c:737).
	 */
	udelay(100);

	wlbt_request_tzpc(w);

	ret = wlbt_program_windows(w);
	if (!ret)
		ret = wlbt_cfg_ack(w);
	if (ret) {
		dev_err(w->dev, "configuration failed: %d\n", ret);
		w->state = WLBT_CFG_ERROR;
		complete(&w->cfg_ack);
		return IRQ_HANDLED;
	}

	dev_info(w->dev, "peer configured, state %s\n",
		 wlbt_state_name(w->state));
	complete(&w->cfg_ack);

	return IRQ_HANDLED;
}

/*
 * The peer asks to be reset when its own watchdog fires
 * (platform_mif_9610.c:592-614).  Clearing the request keeps it alive; a full
 * reboot of the subsystem belongs to the transport driver that will sit on top.
 */
static irqreturn_t wlbt_wdog_isr(int irq, void *data)
{
	struct exynos_wlbt *w = data;
	int ret;

	/* During the config handshake the SoC can raise this spuriously; the
	 * vendor just consumes it (platform_mif_9610.c:591).
	 */
	if (w->state == WLBT_WAIT_CFG_REQ)
		return IRQ_HANDLED;

	w->n_wdog++;

	/* The peer holds the line until something resets it, so a level
	 * interrupt here storms the AP - which is exactly what the vendor
	 * guards against by disabling the line when no reset handler is
	 * registered.
	 */
	disable_irq_nosync(irq);
	dev_err(w->dev, "peer watchdog: the firmware requested a reset\n");

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, WLBT_RESET_REQ_CLR,
				 WLBT_RESET_REQ_CLR);
	if (ret)
		dev_err(w->dev, "cannot clear the reset request: %d\n", ret);

	/* The vendor treats this as a firmware panic (panicmon.c).  Its recovery
	 * cannot be reproduced here, so the work item only takes the link down.
	 */
	schedule_work(&w->peer_gone_work);

	return IRQ_HANDLED;
}

/*
 * The management channel carries the infrastructure protocol (firmware version,
 * halt, panic notification, ...).  Without the message table ported yet, dump
 * what arrives so the first exchange can be read off the wire.
 */
/*
 * One management message: the channel byte, then a packed
 * struct ma_msg_packet { u8 ma_msg; u32 arg; } (mxman.c:547).
 */
bool wlbt_mgmt_send(struct exynos_wlbt *w, u8 channel, u8 ma_msg, u32 arg)
{
	u8 frame[6] = { channel, ma_msg };
	__le32 le = cpu_to_le32(arg);

	memcpy(frame + 2, &le, sizeof(le));

	return wlbt_stream_write(w, &w->mgmt_out, frame, sizeof(frame));
}

void wlbt_mgmt_drain(struct exynos_wlbt *w)
{
	char hex[3 * 32 + 1];
	u8 msg[MGMT_PACKET_SIZE];
	u32 got;
	int i;

	if (rx_off)
		return;

	while ((got = wlbt_stream_read(w, &w->mgmt_in, msg, MGMT_PACKET_SIZE))) {
		w->n_mgmt++;
		if (w->bt && wlbt_bt_mgmt(w->bt, msg, got))
			continue;
		memset(hex, 0, sizeof(hex));
		for (i = 1; i < got && i < 32; i++)
			snprintf(hex + (i - 1) * 3, 4, " %02x", msg[i]);
		dev_info(w->dev, "mgmt message: channel %u:%s\n", msg[0], hex);
	}
}

/*
 * Doorbell from the peer: one bit per stream, INTSR0[31:16].
 */

static irqreturn_t wlbt_mbox_isr(int irq, void *data)
{
	struct exynos_wlbt *w = data;
	u32 status;

	/* INTMSR0 is the pending set after the mask, which is what the vendor
	 * reads too (platform_mif_9610.c:1334).
	 */
	status = readl(w->mbox + MBOX_INTMSR0) >> 16;
	w->n_mbox++;
	w->last_mbox_status = status;

	if (!status) {
		dev_warn(w->dev, "mailbox IRQ without a status bit\n");
		return IRQ_HANDLED;
	}

	/* Acknowledge before reading: any message that lands in the meantime
	 * raises the bit again and the streams are drained until empty anyway.
	 */
	writel(status << 16, w->mbox + MBOX_INTCR0);

	if (rx_off)
		return IRQ_HANDLED;

	if (status & BIT(w->log.write_bit))
		wlbt_log_drain(w);
	if (status & BIT(w->mgmt_in.write_bit))
		wlbt_mgmt_drain(w);
	if (status & BIT(WLBT_BIT_HIP4_TO_HOST))
		wlbt_hip4_drain(w);
	if (w->bt && (status & (BIT(WLBT_BIT_BT_BG_TO_HOST) |
				 BIT(WLBT_BIT_BT_FG_TO_HOST))))
		wlbt_bt_irq(w->bt, status);

	return IRQ_HANDLED;
}

/*
 * Copy the firmware into the MIF DRAM window the peer boots from.  The peer
 * reads it over the data bus, which does not snoop our caches, so the mapping
 * has to be write-combining like the vendor's platform_mif_map_region(); a
 * cached copy would leave the peer with stale lines.
 *
 * The image is placed at offset 0 and the peer is entered at the entry point in
 * its header.  Everything past the image stays zeroed so the firmware's BSS is
 * clean, which is what the vendor gets from freshly allocated DMA memory.
 */
static int wlbt_load_firmware(struct exynos_wlbt *w)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, firmware_name, w->dev);
	if (ret) {
		dev_warn(w->dev, "firmware %s not available: %d\n",
			 firmware_name, ret);
		return ret;
	}

	if (fw->size < FW_HDR_MIN_SIZE ||
	    memcmp(fw->data + FW_HDR_MAGIC_OFFSET, "smxf", 4)) {
		dev_err(w->dev, "%s is not a Maxwell firmware image\n",
			firmware_name);
		ret = -EINVAL;
		goto out;
	}

	w->fw_entry = get_unaligned_le32(fw->data + FW_HDR_ENTRY_OFFSET);
	w->fw_runtime_length =
		get_unaligned_le32(fw->data + FW_HDR_RUNTIME_LENGTH_OFFSET);

	if (w->fw_runtime_length > w->mem_size) {
		dev_err(w->dev, "firmware needs %u bytes, the window is %pa\n",
			w->fw_runtime_length, &w->mem_size);
		ret = -ENOSPC;
		goto out;
	}

	w->fw_size = fw->size;
	if (fw->size >= FW_HDR_PANIC_OFFSET + 4)
		w->fw_panic_offset =
			get_unaligned_le32(fw->data + FW_HDR_PANIC_OFFSET);

	if (!w->dram) {
		w->dram = memremap(w->mem_base, w->mem_size, MEMREMAP_WC);
		if (!w->dram) {
			ret = -ENOMEM;
			goto out;
		}
	}

	memset_io(w->dram, 0, w->mem_size);
	memcpy_toio(w->dram, fw->data, fw->size);
	/* The peer reads the image over the bus; drain the writes first. */
	wmb();

	w->fw_loaded = true;
	ret = 0;

	dev_info(w->dev, "firmware loaded: %zu bytes, runtime %u, entry 0x%x\n",
		 fw->size, w->fw_runtime_length, w->fw_entry);
	dev_info(w->dev, "firmware build: %.*s\n", FW_HDR_BUILD_ID_MAX,
		 fw->data + FW_HDR_BUILD_ID_OFFSET);
out:
	release_firmware(fw);

	return ret;
}

/*
 * Hand the peer its start parameters (mxman.c:1155-1183).
 */
static void wlbt_mbox_init(struct exynos_wlbt *w)
{
	writel(w->fw_entry, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_ENTRY * 4);
	writel(w->mxconf_offset, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_CONFIG * 4);
	writel(MBOX_START_MAGIC, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_MAGIC * 4);
	writel(0, w->mbox + MBOX_ISSR_BASE + MBOX_ISSR_FLAGS * 4);
	/* The peer reads these through the bus; make sure they are there. */
	wmb();
}

/*
 * The MIF DRAM is split the way the vendor splits it (mxman.c:1494): the
 * firmware image occupies [0, fw_runtime_length), and everything else - heaps,
 * streams, configuration - lives above it.  A bump allocator is all that is
 * needed, because nothing is ever freed while the peer is running.
 */
/*
 * miframman.c allocates size + align + sizeof(void *) per block and parks a
 * back pointer just below the aligned block, with the alignment clamped up to
 * sizeof(void *).  Spending the same slack puts every stream at the address
 * the vendor's own driver would have picked, which is one less difference to
 * explain when the peer refuses to read one of them.
 */
u32 wlbt_heap_alloc(struct exynos_wlbt *w, u32 bytes)
{
	const u32 align = sizeof(void *);
	u32 offset, end;

	end = w->heap_next + bytes + align + align;
	if (!w->heap_next || end > w->mem_size) {
		dev_err(w->dev, "MIF heap is full (%u of %pa used)\n",
			w->heap_next, &w->mem_size);
		return 0;
	}

	offset = ALIGN(w->heap_next + align + align, align);
	w->heap_next = end;
	memset_io(w->dram + offset, 0, bytes);

	return offset;
}

/*
 * Same, but honouring a larger alignment.  HIP4 wants its control block on a
 * 4 KiB boundary (hip4.h), which is the only caller of this.
 */
u32 wlbt_heap_alloc_aligned(struct exynos_wlbt *w, u32 bytes, u32 align)
{
	u32 offset, end;

	if (align < sizeof(void *))
		align = sizeof(void *);

	end = w->heap_next + bytes + align + align;
	if (!w->heap_next || end > w->mem_size) {
		dev_err(w->dev, "MIF heap is full (%u of %pa used)\n",
			w->heap_next, &w->mem_size);
		return 0;
	}

	offset = ALIGN(w->heap_next + align, align);
	w->heap_next = end;
	memset_io(w->dram + offset, 0, bytes);

	return offset;
}

/*
 * Copy the hardware configuration files (HCF) into the HIP4 MIB area.  The
 * vendor concatenates them there and hands the firmware the address; without
 * them the WLAN service has no RF/driver parameters and its start never
 * completes (hip4.c:2416-2455).
 */
/*
 * The MIB files only carry the entries above PSID 0x08cf.  The golden, host owned
 * ones below that range are not in any of the shipped variants, and the peer will
 * not join a network without them: its join builds the basic rate set of the
 * association from SLSI_PSID_UNIFI_SUPPORTED_DATA_RATES and refuses the request
 * with fault 0x2c1f - reported as 0x0030 NOT_ALLOWED - when that set is empty.
 *
 * The rate list is N followed by N rates in units of 500 kbit/s, which is the
 * 802.11b/g set: 1, 2, 5.5 and 11 Mbit/s then 6, 9, 12, 18, 24, 36, 48 and 54.
 */

static const u8 wlbt_golden_rates[] = {
	12, 0x02, 0x04, 0x0b, 0x16, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c,
};

/* The vendor driver advertises these extended capabilities and clears the rest. */
static const u8 wlbt_golden_ext_caps[] = {
	9, 0xff, 0xff, 0xf7, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
};

const struct wlbt_golden_mib wlbt_golden_mibs[] = {
	{ MIB_PSID_SUPPORTED_DATA_RATES, wlbt_golden_rates,
	  sizeof(wlbt_golden_rates) },
	{ MIB_PSID_EXTENDED_CAPABILITIES, wlbt_golden_ext_caps,
	  sizeof(wlbt_golden_ext_caps) },
};

u32 wlbt_load_mib(struct exynos_wlbt *w)
{
	static const char *const names[] = {
		"mx140/mx140_wlan.hcf",
		"mx140/mx140_wlan_sw.hcf",
	};
	u32 total = 0;
	size_t skip;
	unsigned int i;

	/* Clear the area so the peer never picks up a stale MIB. */
	memset_io(w->dram + w->hip4_ref + HIP4_MIB_OFFSET, 0, HIP4_MIB_SIZE);

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		const struct firmware *fw;

		if (request_firmware(&fw, names[i], w->dev)) {
			dev_warn(w->dev, "MIB %s not available\n", names[i]);
			continue;
		}
		/*
		 * The files carry an eight byte prologue (a hash, a checksum
		 * and a format version) in front of the entry stream, but the
		 * peer is handed the start of the stream: its walker reads
		 * {u16 id, u16 len} from the first byte it is pointed at and
		 * would otherwise take the prologue for an entry with a bogus
		 * length and assert.
		 */
		skip = 0;
		if (fw->size > MIB_PROLOGUE_SIZE &&
		    get_unaligned_le16(fw->data + 6) == MIB_PROLOGUE_MARKER)
			skip = MIB_PROLOGUE_SIZE;

		if (total + fw->size - skip > HIP4_MIB_SIZE) {
			dev_warn(w->dev, "MIB %s does not fit (%u+%zu > %u)\n",
				 names[i], total, fw->size - skip,
				 HIP4_MIB_SIZE);
			release_firmware(fw);
			break;
		}

		memcpy_toio(w->dram + w->hip4_ref + HIP4_MIB_OFFSET + total,
			    fw->data + skip, fw->size - skip);
		total += fw->size - skip;
		release_firmware(fw);
	}

	/*
	 * The peer reads this area when it is started and caches what it finds,
	 * so the golden entries have to be here and not only in a later request.
	 */
	for (i = 0; i < ARRAY_SIZE(wlbt_golden_mibs); i++) {
		size_t len = 4 + wlbt_golden_mibs[i].len;
		size_t padded = ALIGN(len, 2);
		u8 rec[4 + 16];

		if (len > sizeof(rec) || total + padded > HIP4_MIB_SIZE)
			break;

		put_unaligned_le16(wlbt_golden_mibs[i].psid, rec);
		put_unaligned_le16(wlbt_golden_mibs[i].len, rec + 2);
		memcpy(rec + 4, wlbt_golden_mibs[i].value, wlbt_golden_mibs[i].len);
		memset(rec + len, 0, padded - len);
		memcpy_toio(w->dram + w->hip4_ref + HIP4_MIB_OFFSET + total,
			    rec, padded);
		total += padded;
	}

	if (total)
		dev_info(w->dev, "MIB in the HIP4 area: %u bytes at 0x%x\n",
			 total, w->hip4_ref + HIP4_MIB_OFFSET);

	return total;
}

/*
 * Build the HIP4 shared region the WLAN service is started with.  The vendor
 * allocates 2.5 MiB, drops a hip4_hip_control at the base and points the peer at
 * it through the optional_data field of SM_MSG_START_REQ (hip4.c:2263-2560).
 *
 * The peer walks init.version_a_ref / version_b_ref to the two configuration
 * blocks, so both versions are filled in even though this firmware build will
 * pick one of them.  No MIB and no logging block are supplied: those are the
 * hardware configuration files, and zero means "use what you were built with".
 */

static int wlbt_power_up(struct exynos_wlbt *w)
{
	int ret;

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, MASK_PWR_REQ,
				 MASK_PWR_REQ);
	if (ret) {
		dev_err(w->dev, "cannot request WLBT power: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, WLBT_RESET_SET, 0);
	if (ret) {
		dev_err(w->dev, "cannot release the reset: %d\n", ret);
		return ret;
	}

	dev_info(w->dev, "reset released, starting the peer\n");

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_S, WLBT_START, WLBT_START);
	if (ret) {
		dev_err(w->dev, "cannot start the peer: %d\n", ret);
		return ret;
	}

	if (!wait_for_completion_timeout(&w->cfg_ack, HZ)) {
		dev_err(w->dev, "timeout waiting for CFG_REQ\n");
		wlbt_dump(w, "cfg-timeout");
		return -ETIMEDOUT;
	}

	if (w->state != WLBT_CFG_DONE) {
		dev_err(w->dev, "peer configuration failed, state %s\n",
			wlbt_state_name(w->state));
		return -EIO;
	}

	dev_info(w->dev, "WLBT boot handshake complete\n");

	return 0;
}

/*
 * Put the peer back into reset and drop the WLBT low-power configuration, as
 * platform_mif_9610.c:1105-1187 does before asserting reset.  Only used by the
 * debugfs reset control; the boot path never needs it.
 */
int wlbt_power_down(struct exynos_wlbt *w)
{
	unsigned long timeout;
	u32 val;
	int ret;

	/*
	 * Mask both directions first: the peer can be mid-transfer when this is
	 * called from a reboot notifier, and a line left asserted survives the
	 * reset of the SoC in the WLBT block's own registers.
	 */
	if (w->mbox) {
		writel(0xffff0000, w->mbox + MBOX_INTMR0);
		writel(0x0000ffff, w->mbox + MBOX_INTMR1);
	}

	ret = regmap_update_bits(w->pmu, CENTRAL_SEQ_WLBT_CONFIGURATION,
				 SYS_PWR_CFG_16, 0);
	ret |= regmap_update_bits(w->pmu, RESET_AHEAD_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG_2, 0);
	ret |= regmap_update_bits(w->pmu, CLEANY_BUS_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(w->pmu, LOGIC_RESET_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG_2, 0);
	ret |= regmap_update_bits(w->pmu, TCXO_GATE_WLBT_SYS_PWR_REG,
				  SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(w->pmu, WLBT_DISABLE_ISO_SYS_PWR_REG,
				  SYS_PWR_CFG, 1);
	ret |= regmap_update_bits(w->pmu, WLBT_RESET_ISO_SYS_PWR_REG,
				  SYS_PWR_CFG, 0);
	if (ret) {
		dev_err(w->dev, "cannot drop the WLBT power configuration: %d\n",
			ret);
		return ret;
	}

	ret = regmap_update_bits(w->pmu, WLBT_CTRL_NS, WLBT_RESET_SET,
				 WLBT_RESET_SET);
	if (ret) {
		dev_err(w->dev, "cannot assert the reset: %d\n", ret);
		return ret;
	}

	w->state = WLBT_IN_RESET;
	w->pw_cfg_dirty = true;

	/* The sequencer reports 0x80 once the domain is really down. */
	timeout = jiffies + msecs_to_jiffies(500);
	do {
		regmap_read(w->pmu, CENTRAL_SEQ_WLBT_STATUS, &val);
		if (((val & STATES) >> 16) == 0x80) {
			regmap_update_bits(w->pmu, WLBT_CTRL_NS, MASK_PWR_REQ, 0);
			dev_info(w->dev, "peer in reset\n");
			return 0;
		}
	} while (time_before(jiffies, timeout));

	dev_err(w->dev, "timeout waiting for the reset sequencer\n");
	wlbt_dump(w, "reset-timeout");

	return -ETIME;
}

int wlbt_wlan_start(struct exynos_wlbt *w);

/*
 * One authentication frame for SAE, and the request that goes with it.  Both
 * matter: the frame is what the supplicant answers (it sends its own commit or
 * confirm back through cfg80211's management transmit), and the request is what
 * tells it that this network is being joined with SAE at all.  The peer keeps
 * the exchange moving by itself; the supplicant only owns the derivation of the
 * key from the password.
 */
/*
 * The peer abandons a join it started well before the supplicant can finish
 * the exchange, so a refused join during SAE is not the end of it: re-issue
 * the same request and let the frames keep crossing, the way the vendor's
 * driver retries a failed connect indication.
 */

void wlbt_boot(struct exynos_wlbt *w)
{
	int ret;

	mutex_lock(&w->lock);
	w->state = WLBT_WAIT_CFG_REQ;
	reinit_completion(&w->cfg_ack);

	/*
	 * Loading firmware is opt-in until the transport that talks to it
	 * exists: without HIP4 nothing answers the peer, and a boot path that
	 * starts a subsystem nothing services is not worth having.
	 */
	if (load_firmware && !w->fw_loaded)
		wlbt_load_firmware(w);
	if (w->fw_loaded) {
		/* Only allow the interrupt bits the streams actually use. */
		writel((~WLBT_IRQ_BITS & 0xffff) << 16, w->mbox + MBOX_INTMR0);
		if (!wlbt_build_mxconf(w))
			wlbt_mbox_init(w);
		else
			dev_err(w->dev, "no configuration: the peer will not find its streams\n");
	}

	wlbt_pw_cfg_restore(w);

	if (trace_boot && w->trace) {
		unsigned int off, field, us;

		if (sscanf(trace_boot, "%x:%x:%u", &off, &field, &us) == 3) {
			w->trace_offset = off & ~3u;
			w->trace_field = field & ~3u;
			w->trace_us = min_t(unsigned int, us, 30 * USEC_PER_SEC);
			if (!wlbt_trace_start(w))
				dev_warn(w->dev, "trace already running\n");
		}
	}

	ret = wlbt_power_up(w);
	if (ret) {
		dev_err(w->dev, "boot failed: %d\n", ret);
	} else if (w->fw_loaded) {
		/* Give the firmware a moment to boot and say something, then
		 * take whatever it left in the log even if the doorbell was
		 * missed.
		 */
		msleep(300);
		wlbt_log_drain(w);
		dev_info(w->dev, "after boot: %u log messages, %u undecodable, INTSR0 0x%08x\n",
			 w->n_fwlog, w->n_fwlog_bad,
			 readl(w->mbox + MBOX_INTSR0));

		/*
		 * Ask for the WLAN service as part of booting, so that loading the
		 * module is all it takes: nothing else on the system knows about
		 * the debugfs knob, and an interface without the service behind it
		 * just looks like a card that scans nothing.
		 */
		if (wlan_start_on_boot && !wlbt_wlan_start(w))
			dev_warn(w->dev, "WLAN start not accepted\n");
	}
	mutex_unlock(&w->lock);
}


static struct regmap *wlbt_syscon(struct exynos_wlbt *w, const char *name)
{
	struct regmap *map;

	map = syscon_regmap_lookup_by_phandle(w->dev->of_node, name);
	if (IS_ERR(map)) {
		dev_err(w->dev, "cannot find %s: %ld\n", name, PTR_ERR(map));
		return map;
	}

	return map;
}

static int wlbt_parse_memory(struct exynos_wlbt *w)
{
	struct device_node *node;
	struct resource res;
	int ret;

	node = of_parse_phandle(w->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(w->dev, "no memory-region for the MIF DRAM\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (ret) {
		dev_err(w->dev, "cannot read the MIF DRAM window: %d\n", ret);
		return ret;
	}

	w->mem_base = res.start;
	w->mem_size = resource_size(&res);

	return 0;
}

/*
 * cfg80211 front end
 *
 * The peer is a full MAC: it scans and joins by itself and the driver only
 * carries requests in and results out.  Its scan indications are metadata
 * only - frequency, RSSI and a scan id, never a frame - so the beacons and
 * probe responses it keeps for its own use are read back out of its DRAM once
 * the scan is done.  A scan result without information elements is worthless
 * to a supplicant, and this is the only place they can be had.
 */


static struct ieee80211_rate wlbt_2ghz_rates[] = {
	{ .bitrate = 10 },
	{ .bitrate = 20, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60 },
	{ .bitrate = 90 },
	{ .bitrate = 120 },
	{ .bitrate = 180 },
	{ .bitrate = 240 },
	{ .bitrate = 360 },
	{ .bitrate = 480 },
	{ .bitrate = 540 },
};

/*
 * cfg80211 validates the cipher and AKM suites a supplicant asks for against
 * these before it ever calls the driver, so a wiphy that advertises none of
 * them makes every connection attempt fail with -EINVAL.  The peer does the
 * encryption, so all of them can be offered.
 */
const u32 wlbt_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

struct ieee80211_supported_band wlbt_band_2ghz = {
	.band = NL80211_BAND_2GHZ,
	.channels = wlbt_2ghz_channels,
	.n_channels = ARRAY_SIZE(wlbt_2ghz_channels),
	.bitrates = wlbt_2ghz_rates,
	.n_bitrates = ARRAY_SIZE(wlbt_2ghz_rates),
};


static struct ieee80211_rate wlbt_5ghz_rates[] = {
	{ .bitrate = 60 },
	{ .bitrate = 90 },
	{ .bitrate = 120 },
	{ .bitrate = 180 },
	{ .bitrate = 240 },
	{ .bitrate = 360 },
	{ .bitrate = 480 },
	{ .bitrate = 540 },
};

struct ieee80211_supported_band wlbt_band_5ghz = {
	.band = NL80211_BAND_5GHZ,
	.channels = wlbt_5ghz_channels,
	.n_channels = ARRAY_SIZE(wlbt_5ghz_channels),
	.bitrates = wlbt_5ghz_rates,
	.n_bitrates = ARRAY_SIZE(wlbt_5ghz_rates),
};

struct exynos_wlbt *wlbt_from_wiphy(struct wiphy *wiphy)
{
	return *(struct exynos_wlbt **)wiphy_priv(wiphy);
}

/*
 * The private area of the net device is its wireless_dev, so the driver data is
 * reached through the wiphy and not by casting that area.
 */
struct exynos_wlbt *wlbt_from_netdev(struct net_device *ndev)
{
	struct wireless_dev *wdev = netdev_priv(ndev);

	if (!wdev || !wdev->wiphy)
		return NULL;

	return wlbt_from_wiphy(wdev->wiphy);
}

/*
 * A management frame worth reporting: a beacon or a probe response, both
 * addresses being the access point, with elements that parse to the end.  The
 * peer's DRAM is full of everything else, so the test has to be strict.
 */

static int wlbt_reboot_notifier(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct exynos_wlbt *w = container_of(nb, struct exynos_wlbt, reboot_nb);

	if (!mutex_trylock(&w->lock))
		return NOTIFY_DONE;
	wlbt_power_down(w);
	mutex_unlock(&w->lock);

	return NOTIFY_DONE;
}

static int wlbt_probe(struct platform_device *pdev)
{
	struct exynos_wlbt *w;
	int irq_cfg, irq_mbox, irq_wdog;
	int ret;

	w = devm_kzalloc(&pdev->dev, sizeof(*w), GFP_KERNEL);
	if (w)
		w->trace = devm_kzalloc(&pdev->dev,
					WLBT_TRACE_MAX * sizeof(*w->trace),
					GFP_KERNEL);
	if (w)
		w->tx = devm_kzalloc(&pdev->dev,
				     WLBT_TX_SLOTS * sizeof(*w->tx), GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	w->dev = &pdev->dev;
	platform_set_drvdata(pdev, w);
	mutex_init(&w->lock);
	spin_lock_init(&w->hip4_lock);
	init_completion(&w->cfg_ack);
	w->state = WLBT_IN_RESET;

	w->mbox = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(w->mbox))
		return PTR_ERR(w->mbox);

	w->pmu = wlbt_syscon(w, "samsung,syscon-phandle");
	if (IS_ERR(w->pmu))
		return PTR_ERR(w->pmu);
	wlbt_pw_cfg_save(w);

	/*
	 * The window controllers are optional here: they are only needed once
	 * the peer asks for its configuration, and a board without them should
	 * still be able to probe and report why the handshake cannot happen.
	 */
	w->dbus_baaw = wlbt_syscon(w, "samsung,dbus_baaw-syscon-phandle");
	w->pbus_baaw = wlbt_syscon(w, "samsung,pbus_baaw-syscon-phandle");
	w->remap = wlbt_syscon(w, "samsung,wlbt_remap-syscon-phandle");
	w->boot_cfg = wlbt_syscon(w, "samsung,boot_cfg-syscon-phandle");
	if (IS_ERR(w->dbus_baaw) || IS_ERR(w->pbus_baaw) ||
	    IS_ERR(w->remap) || IS_ERR(w->boot_cfg))
		return -EPROBE_DEFER;

	ret = wlbt_parse_memory(w);
	if (ret)
		return ret;

	irq_cfg = platform_get_irq_byname(pdev, "CFG_REQ");
	if (irq_cfg < 0)
		return irq_cfg;
	irq_mbox = platform_get_irq_byname(pdev, "MBOX");
	if (irq_mbox < 0)
		return irq_mbox;
	irq_wdog = platform_get_irq_byname(pdev, "WDOG");
	if (irq_wdog < 0)
		return irq_wdog;

	/* The state has to be correct before the handler can run; requesting
	 * the interrupt is itself the barrier that guarantees it.
	 */
	w->state = WLBT_IN_RESET;

	ret = devm_request_irq(w->dev, irq_cfg, wlbt_cfg_req_isr, IRQF_TRIGGER_HIGH,
			       "wlbt-cfg-req", w);
	if (ret) {
		dev_err(w->dev, "cannot request the CFG_REQ interrupt: %d\n", ret);
		return ret;
	}

	/* WLBT watchdog and doorbell will only matter once the transport is
	 * there, but taking them keeps a level interrupt from storming.
	 */
	w->wdog_irq = irq_wdog;
	ret = devm_request_irq(w->dev, irq_wdog, wlbt_wdog_isr, IRQF_TRIGGER_HIGH,
			       "wlbt-wdog", w);
	if (ret)
		return ret;

	ret = devm_request_irq(w->dev, irq_mbox, wlbt_mbox_isr, IRQF_TRIGGER_HIGH,
			       "wlbt-mbox", w);
	if (ret)
		return ret;

	wlbt_debugfs_init(w);

	w->reboot_nb.notifier_call = wlbt_reboot_notifier;
	if (devm_register_reboot_notifier(w->dev, &w->reboot_nb))
		dev_warn(w->dev, "cannot register the reboot notifier\n");

	/* The interface is registered even when the peer is not started: the
	 * scan op reports -EIO until the transport exists, which is a better
	 * answer than no interface at all.
	 */
	ret = wlbt_register_wiphy(w);
	if (ret)
		dev_warn(w->dev, "no wireless interface: %d\n", ret);

	dev_info(w->dev, "mailbox at %pR, MIF DRAM %pa (%pa)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 &w->mem_base, &w->mem_size);

	wlbt_boot(w);

	/*
	 * Register the HCI device after the peer's WLAN service is up: the BT
	 * service start is part of the same firmware and needs the MIF heap that
	 * wlbt_hip4_build() lays out.  The device stays down until userspace
	 * opens it.
	 */
	w->bt = wlbt_bt_init(w);
	if (IS_ERR(w->bt)) {
		dev_warn(w->dev, "no Bluetooth interface: %ld\n",
			 PTR_ERR(w->bt));
		w->bt = NULL;
	}

	return 0;
}

static void wlbt_remove(struct platform_device *pdev)
{
	struct exynos_wlbt *w = platform_get_drvdata(pdev);

	wlbt_bt_exit(w->bt);
	w->bt = NULL;
	wlbt_unregister_wiphy(w);

	mutex_lock(&w->lock);
	w->vif_created = false;
	if (w->peer_dead)
		dev_info(w->dev, "peer already stopped, leaving it in reset\n");
	else
		wlbt_power_down(w);
	mutex_unlock(&w->lock);

	debugfs_remove(w->debugfs);

	if (w->dram)
		memunmap(w->dram);
}

static const struct of_device_id wlbt_of_match[] = {
	{ .compatible = "samsung,scsc_wifibt" },
	{ }
};
MODULE_DEVICE_TABLE(of, wlbt_of_match);

static struct platform_driver wlbt_driver = {
	.probe = wlbt_probe,
	.remove = wlbt_remove,
	.driver = {
		.name = "exynos9610-wlbt",
		.of_match_table = wlbt_of_match,
	},
};
module_platform_driver(wlbt_driver);

/* The Bluetooth service only reports START_CFM once the ABOX is running. */
MODULE_SOFTDEP("pre: exynos9610_abox");
MODULE_DESCRIPTION("Samsung Exynos 9610 SCSC/WLBT power and boot handshake");
MODULE_LICENSE("GPL");

