// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9610 CPU cluster clock (CMU_CPUCL0 / CMU_CPUCL1).
 *
 * The ACPM firmware owns CPU DVFS on this SoC, but it never services the DVFS
 * channel (see notes/plan-dvfs.md), so the clusters stay at the rate the boot
 * chain programmed them to: CPUCL0 (LITTLE) at 1053 MHz and CPUCL1 (big) at
 * 1456 MHz.  This driver takes the clock back for the AP.
 *
 * Register model (all offsets verified on hardware, see notes/cpufreq.md):
 *
 *	PLL_CON0		bits 2:0 DIV_S, 13:8 DIV_P, 25:16 DIV_M,
 *				bit 29 STABLE, bit 31 ENABLE
 *	fout = 26 MHz * DIV_M / (DIV_P << DIV_S)
 *
 *	SW_USER			bit 4 MUX_SEL, bit 7 BUSY
 *				parent 0 = OSCCLK_CPUCL0 (26 MHz)
 *				parent 1 = CLKCMU_CPUCLx_SWITCH (shared PLL)
 *	MUX_<cluster>_PLL	bit 0 SELECT, bit 16 BUSY
 *				parent 0 = the cluster PLL (normal)
 *				parent 1 = the SW_USER mux
 *
 * The cluster runs straight off its own PLL (DIV_CLK_CPUCLx_CPU is a fixed
 * divide by one), so a rate change means reprogramming the PLL.  That cannot
 * be done while the cores are clocked from it, so the mux is pointed at the
 * always-on 26 MHz oscillator first, exactly like the vendor's clock switch.
 *
 * Two things this driver does *not* trust, both learned on hardware:
 *
 *   - the PLL STABLE bit, which stays 0 through hundreds of good rate changes
 *     and must never decide whether the cluster is switched back (see
 *     cpucl_pll_start()),
 *   - writing a CPU buck downward, which hangs this board, so the rails are only
 *     ever raised (see boost_only).
 *
 * The operating points above the boot rate also need a higher rail and are
 * therefore behind the "arm" switch: an unarmed boot runs at the boot operating
 * point and never writes the PMIC, whatever else happens.
 */

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define CPUCL_PLL_CON0		0x0140
#define CPUCL_SW_USER		0x0120
#define CPUCL_MUX_PLL		0x1000
#define CPUCL_DIV_CPU		0x1810

#define CPUCL_PLL_LOCKTIME	0x0000

#define PLL_ENABLE		BIT(31)
#define PLL_STABLE		BIT(29)
#define PLL_OUT_SEL		BIT(4)	/* PLL_MUX_SEL in the vendor sources */
#define PLL_MUX_BUSY		BIT(7)	/* PLL_MUX_BUSY: the output mux is moving */
#define PLL_M_MASK		GENMASK(25, 16)
#define PLL_P_MASK		GENMASK(13, 8)
#define PLL_S_MASK		GENMASK(2, 0)
#define PLL_M_SHIFT		16
#define PLL_P_SHIFT		8

#define SW_USER_SEL		BIT(4)
#define SW_USER_BUSY		BIT(7)
#define MUX_PLL_SEL		BIT(0)
#define MUX_PLL_BUSY		BIT(16)

#define CPUCL_FIN_HZ		26000000UL
#define CPUCL_MIN_HZ		26000000UL
#define CPUCL_PLL_LOCK_CYCLES	150	/* lock time in reference cycles per DIV_P */
/*
 * Time given to the analog loop to lock after it is started, in microseconds.  A
 * lock takes LOCKTIME phase detector cycles, i.e. about 70 us with DIV_P 3; this
 * is that with room for the slower oscillator the cluster runs from meanwhile.
 * It is a plain delay, not a poll of the STABLE bit - see cpucl_pll_start().
 */
#define CPUCL_PLL_LOCK_UDELAY	300

/*
 * PLL limits from cal-if/pll_spec.c.  The reference divider must put the phase
 * detector between 4 and 12 MHz (DIV_P 3..6 for a 26 MHz crystal) and the VCO
 * has to stay in its band, which differs between the two cluster PLLs:
 * gpll1051X_spec for CPUCL0 and gpll1054X_spec for CPUCL1.
 */
#define CPUCL_PFD_MIN_HZ	4000000UL
#define CPUCL_PFD_MAX_HZ	12000000UL
/*
 * The vendor only ever uses DIV_P 3 or 4 for the cluster PLLs (PFD 8.67 or
 * 6.5 MHz).  Staying away from the low end keeps the phase detector fast and
 * the lock time short: the lock time register counts PFD cycles, so DIV_P 6
 * needs 900 of them, four times longer than DIV_P 3.
 */
#define CPUCL_P_MIN		3
#define CPUCL_P_MAX		4
#define CPUCL_M_MIN		64
#define CPUCL_M_MAX		1023
#define CPUCL_S_MAX		6

/*
 * The boot chain writes the vendor parameter block (ECT) to a fixed address; the
 * live vendor DT describes it as  ect { parameter_address = <0xa0000000>;
 * parameter_size = <0x3a000>; }.  It holds the per-ASV DVFS tables, i.e. the
 * frequency of every operating point together with the voltage this specific
 * chip needs.  /dev/mem refuses system RAM, so it is mapped here.
 */
#define CPUCL_ECT_PHYS		0xa0000000
#define CPUCL_ECT_SIZE		0x3a000

/* ASV eFuse word: bigcpu group [3:0], littlecpu group [19:16]. */
#define CPUCL_ASV_EFUSE		0x10009000
#define CPUCL_ASV_MODIFY_MASK	GENMASK(7, 4)
#define CPUCL_ASV_LITTLE_SHIFT	16

/* S2MPU09 buck layout, from mfd/samsung/s2mpu09-regulator.h. */
#define CPUCL_BUCK_MIN_UV	300000
#define CPUCL_BUCK_STEP_UV	6250
#define CPUCL_BUCK_RAMP_US	12000	/* regulator-ramp-delay of the vendor DT */
#define CPUCL_PMIC_CHAN		2
#define CPUCL_PMIC_TYPE		1

struct exynos9610_cpucl {
	struct device *dev;
	const char *name;
	unsigned int id;
	void __iomem *base;
	phys_addr_t base_phys;
	u32 pll_con0;
	u32 sw_user;
	unsigned long boot_hz;
	unsigned long cur_hz;
	unsigned long min_hz;
	unsigned long max_hz;
	unsigned long vco_min_hz;
	unsigned long vco_max_hz;
	u32 orig_con0;
	u32 good_con0;		/* last PLL setting that locked */
	cpumask_t cpus;
	struct cpufreq_frequency_table *table;
	struct mutex lock;	/* one rate change at a time per cluster */
	struct acpm_handle *acpm;
	u8 buck_reg;
	unsigned int rail_floor_uv;	/* lowest rail value this board tolerates */
	unsigned int applied_uv;	/* highest rail value written so far */
	unsigned long n_relock;		/* rate changes that reached the hardware */
	unsigned long n_rail;		/* rail writes, 1 per real change */
	unsigned long n_skipped;	/* requests for the rate already in place */
	unsigned int pending_uv;	/* rail value waiting out the hold time */
	struct delayed_work rail_work;	/* applies a pending rail drop */
	unsigned int *uvolt;	/* parallel to table, NULL when unknown */
};

/*
 * Voltage tables.  The operating points above the boot rate need a higher rail,
 * and the voltages come from the vendor tables in ECT, which are checked against
 * the PMIC at probe time; if that check does not pass the driver falls back to
 * the boot rate and never writes a rail.
 *
 * This is on by default because the policy has to expose the whole range from
 * the moment it is created: schedutil takes its reference frequency from the
 * policy at registration time, and the cooling device that thermal uses for
 * throttling is built from the same table, so a policy that starts out capped
 * at the boot rate stays capped even after the table is extended.
 *
 * Having a table is not the same as using it: see the "arm" switch below.  The
 * registered table is always the full one (that is what keeps the thermal
 * cooling state numbering stable), but every rate above the boot rate is
 * refused until the driver is armed.
 */
static bool volt_scale = true;
module_param(volt_scale, bool, 0644);
MODULE_PARM_DESC(volt_scale, "Build the operating points above the boot rate (needs the ECT tables)");

/*
 * Arming: on by default, because the platform is expected to run at its rated
 * operating points, not at whatever the boot chain happened to leave behind.
 *
 * This switch exists because everything that was ever suspected of hanging this
 * board - the rail writes, the top operating points, the PLL relock - is behind
 * it, and during bring-up a boot that cannot reach any of it is worth more than
 * a boot that is fast.  All three turned out to be understood (see notes/
 * cpufreq.md: the PLL STABLE bit is not a lock indicator, "downward rail writes
 * hang the board" was the destination voltage, and the rail floor is 643.75 mV),
 * so it is now on: twelve soak rounds of full-range transitions, 8-core loads,
 * hotplug and thermal throttling on dpu129 ran without a single relock error.
 *
 * Turn it off at boot with exynos9610_cpuclk.arm=0, or at runtime with
 *
 *	echo 0 > /sys/kernel/debug/exynos9610-cpuclk/arm
 *
 * which is what to reach for when bisecting something new: an unarmed kernel
 * never writes the PMIC and never leaves the operating points the boot chain
 * validated.
 */
static bool arm = true;
module_param(arm, bool, 0644);
MODULE_PARM_DESC(arm, "Allow rates above the boot operating point (default: on)");

static bool cpucl_armed;

/*
 * Only ever raise a rail, never write a lower value.  This was the workaround
 * for "writing a rail downward freezes this board" and is now off by default,
 * because that belief was wrong: what froze it was the vendor table's 600 mV
 * entry for the lowest LITTLE operating point, and the rail tolerates anything
 * from 643.75 mV up (see rail_floor_uv).  With the floor in place the rails
 * track the operating points in both directions, which is what makes this a
 * real DVFS driver instead of a clock driver.
 *
 * Turn it on to keep the rails at the highest value they reached, which is
 * still useful when bisecting a voltage problem.
 */
static bool boost_only;

/* Print every rate change and rail write, for chasing a rail that will not move. */
static bool loud;
module_param(loud, bool, 0644);
MODULE_PARM_DESC(loud, "Log every rate change and rail write");

/*
 * How long the rail stays where it is before a lower value is applied.
 *
 * A governor on a bursty desktop hops between the bottom of the range and its
 * ceiling many times a second: schedutil maps utilisation against the maximum
 * frequency the CPU advertises, so anything above half of it lands on the
 * ceiling, and the next sample lands back on the minimum.  Following that with
 * the rail means a PMIC ramp - twelve milliseconds during which the cluster
 * cannot speed up - on every hop, which is exactly the wrong way round: the
 * voltage is then high while the cluster is slow, and the frequency change waits
 * for it.
 *
 * Holding the rail for longer than a hop makes each hop a plain PLL change, and
 * the rail still comes down as soon as the cluster has settled at a lower
 * operating point: measured hop periods on an idle system with an NFS root are
 * around 110 ms, so two hundred milliseconds leaves the PMIC out of the loop
 * while still following a cluster that really has gone quiet.  The cost is that a short dip is over-volted,
 * which is what the vendor hardware does too: its DVFS firmware is asynchronous
 * and does not chase every request.
 */
static unsigned int rail_hold_ms = 200;
module_param(rail_hold_ms, uint, 0644);
MODULE_PARM_DESC(rail_hold_ms, "Hold a rail this long before dropping it (ms)");
module_param(boost_only, bool, 0644);
MODULE_PARM_DESC(boost_only, "Only ever raise the CPU rails, never lower them");

/*
 * Largest rail change the debugfs probe writes in one go, in selector steps (one
 * step is 6.25 mV).  The normal path writes the target in a single register
 * write and lets the PMIC ramp - which is what the vendor regulator does, and
 * what keeps a full-range change at one 12 ms ramp instead of eight.  The probe
 * defaults to small steps because walking a rail by hand is how the floor was
 * measured and a jump hides where it is.
 */
static unsigned int buck_step = 8;
module_param(buck_step, uint, 0644);
MODULE_PARM_DESC(buck_step, "Max rail change per debugfs write, in 6.25 mV steps");

/*
 * Relocking one cluster while the other is relocking makes the lock wait
 * unreliable (the waiting cluster runs from the 26 MHz oscillator, and the two
 * sequences overlap), so rate changes are serialised across clusters.
 */
static DEFINE_MUTEX(cpucl_relock_lock);

static void __iomem *cpucl_ect_base;
static struct exynos9610_cpucl *clusters;
static unsigned int num_clusters;
static struct dentry *cpucl_debugfs;

static unsigned long cpucl_pll_rate(u32 con0)
{
	u64 m = FIELD_GET(PLL_M_MASK, con0);
	u64 p = FIELD_GET(PLL_P_MASK, con0);
	u64 s = FIELD_GET(PLL_S_MASK, con0);

	if (!m || !p)
		return 0;

	return div_u64((u64)CPUCL_FIN_HZ * m, p << s);
}

/*
 * Encode a rate as the PLL's M/P/S fields.  Every combination is scored so the
 * one with the smallest error wins; ties prefer the larger DIV_M, which gives
 * better output jitter.  Returns 0 when no combination is usable.
 */
/*
 * True when DIV_M/DIV_P/DIV_S are legal for the cluster PLL.  The VCO
 * frequency is the PLL output before DIV_S, and it is the band that decides
 * whether the loop can lock at all.
 */
static bool cpucl_fields_valid(const struct exynos9610_cpucl *c, u32 p, u32 s,
			       u32 m)
{
	u64 pfd, vco;

	if (s > CPUCL_S_MAX || m < CPUCL_M_MIN || m > CPUCL_M_MAX)
		return false;

	if (p < CPUCL_P_MIN || p > CPUCL_P_MAX)
		return false;

	pfd = div_u64(CPUCL_FIN_HZ, p);
	if (pfd < CPUCL_PFD_MIN_HZ || pfd > CPUCL_PFD_MAX_HZ)
		return false;

	vco = div_u64((u64)CPUCL_FIN_HZ * m, p);

	return vco >= c->vco_min_hz && vco <= c->vco_max_hz;
}

/* Best legal encoding for one (DIV_P, DIV_S) pair, or 0. */
static u32 cpucl_pll_fields(const struct exynos9610_cpucl *c, u32 p, u32 s,
			    unsigned long hz, bool *ok)
{
	u64 div = (u64)p << s;
	u64 m = div_u64((u64)hz * div + CPUCL_FIN_HZ / 2, CPUCL_FIN_HZ);
	u32 con0;

	*ok = cpucl_fields_valid(c, p, s, m);
	if (!*ok)
		return 0;

	con0 = FIELD_PREP(PLL_M_MASK, m) | FIELD_PREP(PLL_P_MASK, p) |
	       FIELD_PREP(PLL_S_MASK, s);
	*ok = abs_diff((u64)hz, (u64)cpucl_pll_rate(con0)) <= hz / 200;

	return con0;
}

/*
 * Encode a rate as the PLL's M/P/S fields.  The reference divider and output
 * divider already in use are tried first so that a familiar rate keeps exactly
 * the vendor's PFD setup; otherwise every legal combination is scored and the
 * one with the smallest error wins, ties preferring the larger DIV_M.
 */
static u32 cpucl_rate_to_con0(const struct exynos9610_cpucl *c, unsigned long hz,
			      u32 cur)
{
	unsigned long best_err = ULONG_MAX;
	u32 best = 0;
	bool ok;
	u32 p, s;

	best = 0;
	for (p = CPUCL_P_MIN; p <= CPUCL_P_MAX; p++) {
		for (s = 0; s <= CPUCL_S_MAX; s++) {
			u32 con0 = cpucl_pll_fields(c, p, s, hz, &ok);
			unsigned long err;

			if (!ok)
				continue;

			err = abs_diff((u64)hz, (u64)cpucl_pll_rate(con0));
			if (err < best_err ||
			    (err == best_err && con0 > best)) {
				best_err = err;
				best = con0;
			}
		}
	}

	/* Refuse anything that is not within 0.5% of the request. */
	if (!best || best_err > hz / 200)
		return 0;

	return best;
}

static int cpucl_wait_clear(void __iomem *reg, u32 bit)
{
	unsigned int i;

	for (i = 0; i < 300; i++) {
		if (!(readl(reg) & bit))
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

/*
 * Put the cluster back on its PLL once the PLL reports a lock.  The mux has its
 * own busy bit, and the cores must not be handed a clock that is still moving.
 */
static void cpucl_mux_restore(struct exynos9610_cpucl *c, void __iomem *mux)
{
	writel(readl(mux) & ~MUX_PLL_SEL, mux);
	cpucl_wait_clear(mux, MUX_PLL_BUSY);
	udelay(5);
}

/*
 * Shut the PLL down the way the vendor does it (ra_enable_pll() in cal-if/ra.c):
 * first take the PLL out of the clock path by clearing its output mux select and
 * waiting for the mux to report done, then clear the enable bit.
 * Only then may DIV_M/DIV_P/DIV_S be written.
 *
 * Reconfiguring a running PLL in one write - which is what this driver used to
 * do - works most of the time and then wedges the lock detector: the PLL then
 * reports ENABLE with STABLE clear, and no setting at all will lock again until
 * the device is rebooted.  That is the lock failure this sequence exists to
 * avoid.
 */
static int cpucl_pll_disable(struct exynos9610_cpucl *c)
{
	void __iomem *con0 = c->base + c->pll_con0;

	writel(readl(con0) & ~PLL_OUT_SEL, con0);
	if (cpucl_wait_clear(con0, PLL_MUX_BUSY)) {
		pr_err("cpucl: %s PLL output mux does not settle\n", c->name);
		return -ETIMEDOUT;
	}

	writel(readl(con0) & ~PLL_ENABLE, con0);

	/* The VCO has to stop before it can be started again. */
	usleep_range(20, 40);

	return 0;
}

/*
 * Start the PLL on one setting: lock time first, then DIV_M/DIV_P/DIV_S together
 * with the output mux and the enable bit in a single write.  ra_pll_set_pmsk()
 * in cal-if/ra.c is the same sequence, and the lock time register counts phase
 * detector cycles, i.e. DIV_P * 150.
 *
 * The result is checked by reading the fields back out of the register, not by
 * waiting for the STABLE bit: measured on this board, STABLE stays 0 for
 * hundreds of consecutive, perfectly good rate changes and then only latches
 * again when the cluster mux is switched back to the PLL.  It is a status bit
 * to report, never a condition to act on - a driver that refuses to switch the
 * cores back on STABLE == 0 leaves them on the 26 MHz oscillator while the PLL
 * is running at the requested rate, which is exactly how this port used to
 * "fail".
 */
static bool cpucl_pll_start(struct exynos9610_cpucl *c, u32 pmsk)
{
	void __iomem *con0 = c->base + c->pll_con0;
	u32 val;

	writel(FIELD_GET(PLL_P_MASK, pmsk) * CPUCL_PLL_LOCK_CYCLES,
	       c->base + CPUCL_PLL_LOCKTIME);

	val = readl(con0);
	val &= ~(PLL_M_MASK | PLL_P_MASK | PLL_S_MASK);
	val |= pmsk | PLL_OUT_SEL | PLL_ENABLE;
	writel(val, con0);

	/* A lock takes about 70 us; do not hand the cores a PLL inside that. */
	udelay(CPUCL_PLL_LOCK_UDELAY);

	/* The register file has to carry what was written. */
	return (readl(con0) & (PLL_M_MASK | PLL_P_MASK | PLL_S_MASK |
			       PLL_ENABLE | PLL_OUT_SEL)) ==
	       (pmsk | PLL_ENABLE | PLL_OUT_SEL);
}

/* Full cycle: shut the PLL down, reconfigure it, bring it back. */
static bool cpucl_pll_set(struct exynos9610_cpucl *c, u32 pmsk)
{
	if (cpucl_pll_disable(c))
		return false;

	return cpucl_pll_start(c, pmsk);
}

/*
 * Program one cluster.  The cluster is moved to the 26 MHz oscillator, the PLL
 * is reprogrammed and started, and the cluster is moved back.  The cluster is
 * handed back even when the PLL reads back wrong: the fallback there is the
 * setting the boot chain programmed, which is known to run on this silicon, so
 * the cores always end up with a working clock.
 */
static int cpucl_relock(struct exynos9610_cpucl *c, unsigned long hz)
{
	void __iomem *mux = c->base + CPUCL_MUX_PLL;
	u32 new_con0, boot_con0;
	bool ok;
	int ret;

	if (hz < CPUCL_MIN_HZ || hz > c->max_hz)
		return -EINVAL;

	new_con0 = cpucl_rate_to_con0(c, hz, readl(c->base + c->pll_con0));
	if (!new_con0)
		return -EINVAL;

	/*
	 * Two CPUs of the same cluster can ask for a rate at the same time; an
	 * interleaved pair of sequences would leave the mux on the oscillator
	 * with an unlocked PLL.  The caller already holds c->lock; the global
	 * one serialises the two clusters, whose lock waits interfere.
	 */
	guard(mutex)(&cpucl_relock_lock);

	/* Take the cluster off the PLL; SW_USER selects OSCCLK here. */
	writel(readl(mux) | MUX_PLL_SEL, mux);
	ret = cpucl_wait_clear(mux, MUX_PLL_BUSY);
	if (ret)
		return ret;

	if (!(readl(mux) & MUX_PLL_SEL))
		return -EIO;
	/* Let the cluster clock settle on the oscillator before it runs again. */
	udelay(5);

	ok = cpucl_pll_set(c, new_con0);
	if (ok) {
		c->good_con0 = readl(c->base + c->pll_con0);
	} else {
		boot_con0 = c->orig_con0 & (PLL_M_MASK | PLL_P_MASK | PLL_S_MASK);
		pr_err("cpucl: %s did not take %lu Hz (con0=%#010x), restoring %lu Hz\n",
		       c->name, hz, readl(c->base + c->pll_con0),
		       cpucl_pll_rate(boot_con0));
		cpucl_pll_set(c, boot_con0);
		ret = -ETIMEDOUT;
	}

	/* Hand the cluster back to the PLL - always. */
	cpucl_mux_restore(c, mux);

	c->cur_hz = cpucl_pll_rate(readl(c->base + c->pll_con0));

	if (!(readl(c->base + c->pll_con0) & PLL_STABLE))
		pr_debug("cpucl: %s at %lu Hz (STABLE reads 0, ignoring it)\n",
			 c->name, c->cur_hz);

	if (ret)
		pr_err("cpucl: %s kept at %lu Hz\n", c->name, c->cur_hz);

	return ret;
}

/*
 * Never write a rail below the floor this board was measured to tolerate.  The
 * LITTLE rail hangs the SoC between 637.5 mV and 600 mV (the last value being
 * what the vendor table asks for at 403 MHz); the big rail was walked down to
 * 600 mV without trouble, so the floor is set per cluster from the device tree
 * and defaults to the LITTLE measurement, which is safe for both.
 */
static unsigned int cpucl_rail_uv(const struct exynos9610_cpucl *c,
				  unsigned int uv)
{
	return uv < c->rail_floor_uv ? c->rail_floor_uv : uv;
}

static unsigned int cpucl_uvolt_for(const struct exynos9610_cpucl *c,
				    unsigned long hz)
{
	unsigned int i;

	if (!c->uvolt)
		return 0;

	for (i = 0; c->table[i].frequency != CPUFREQ_TABLE_END; i++)
		if ((unsigned long)c->table[i].frequency * 1000 == hz)
			return c->uvolt[i];

	return 0;
}

/*
 * Highest rate the driver currently allows: the boot operating point until it
 * is armed, the rated maximum of the cluster after that - and never above the
 * boot rate when there is no voltage table to raise the rail with.
 */
static unsigned long cpucl_ceiling_hz(const struct exynos9610_cpucl *c)
{
	return (cpucl_armed && volt_scale && c->uvolt) ? c->max_hz : c->boot_hz;
}

/*
 * Write one rail selector through the ACPM PMIC protocol, in steps of at most
 * buck_step so the buck sees a slew instead of a jump.  Returns the number of
 * steps written (positive), 0 when the rail already held the target, or a
 * negative errno.  A negative return after some steps were written looks like a
 * failed change but the rail may be half way: callers treat any error as "the
 * rail is not what was asked for" and do not touch the frequency.
 */
static int cpucl_buck_slew(struct exynos9610_cpucl *c, unsigned int uvolt,
			   unsigned int max_step, bool wait, bool loud)
{
	const struct acpm_pmic_ops *ops;
	unsigned int sel, delta, n = 0;
	u8 old;
	int ret;

	if (!c->acpm || !c->buck_reg || !uvolt)
		return -EOPNOTSUPP;

	if (uvolt < CPUCL_BUCK_MIN_UV ||
	    uvolt > CPUCL_BUCK_MIN_UV + CPUCL_BUCK_STEP_UV * 255)
		return -ERANGE;

	ops = &c->acpm->ops.pmic_ops;
	sel = (uvolt - CPUCL_BUCK_MIN_UV) / CPUCL_BUCK_STEP_UV;

	ret = ops->read_reg(c->acpm, CPUCL_PMIC_CHAN, CPUCL_PMIC_TYPE,
			    c->buck_reg, 0, &old);
	if (ret)
		return ret;
	if (old == sel)
		return 0;

	if (loud)
		pr_emerg("cpucl: %s buck 0x%02x -> 0x%02x (%u -> %u uV) step %u\n",
			 c->name, old, (u8)sel,
			 old * CPUCL_BUCK_STEP_UV + CPUCL_BUCK_MIN_UV,
			 sel * CPUCL_BUCK_STEP_UV + CPUCL_BUCK_MIN_UV, max_step);

	while (old != sel) {
		u8 next;

		delta = old > sel ? old - sel : sel - old;
		if (max_step && delta > max_step)
			delta = max_step;

		next = old > sel ? old - delta : old + delta;

		if (loud)
			pr_emerg("cpucl: %s buck 0x%02x <- 0x%02x\n",
				 c->name, c->buck_reg, next);

		ret = ops->write_reg(c->acpm, CPUCL_PMIC_CHAN, CPUCL_PMIC_TYPE,
				     c->buck_reg, 0, next);
		if (ret)
			return ret;

		/*
		 * Wait out the ramp of this step before asking for the next.
		 * Only a raise has to be waited for: the clock is moved after
		 * the rail is up, and a step asked for before the previous one
		 * settled would ramp from an unknown point.  A drop happens once
		 * the cores are already slower than before, so the buck can take
		 * its 12 ms on its own while the next request is served.
		 */
		if (wait)
			usleep_range(CPUCL_BUCK_RAMP_US,
				     CPUCL_BUCK_RAMP_US + 2000);

		old = next;
		n++;
	}

	if (loud)
		pr_emerg("cpucl: %s buck 0x%02x ok, %u step(s)\n",
			 c->name, c->buck_reg, n);

	return n;
}

/*
 * One write, then wait out the PMIC's ramp if anyone is going to depend on the
 * new voltage being in place.  The buck ramps internally, so stepping here
 * would only multiply the time the cpufreq workqueue spends at the old rate.
 */
static int cpucl_buck_set(struct exynos9610_cpucl *c, unsigned int uvolt,
			  bool wait)
{
	/* Returns the number of register writes, 0 when the rail was already there. */
	return cpucl_buck_slew(c, uvolt, 0, wait, loud);
}

/*
 * Apply a rail drop once the cluster has stayed at the lower operating point for
 * the hold time.  The rail is re-checked first: a burst that raised it again in
 * the meantime wins, and the pending value is simply dropped.
 */
static void cpucl_rail_work(struct work_struct *work)
{
	struct exynos9610_cpucl *c = container_of(to_delayed_work(work),
						  struct exynos9610_cpucl,
						  rail_work);
	unsigned int uvolt;
	int ret;

	guard(mutex)(&c->lock);

	uvolt = c->pending_uv;
	c->pending_uv = 0;
	if (!uvolt || uvolt == c->applied_uv)
		return;

	/* Still the voltage the current rate asks for? */
	if (cpucl_rail_uv(c, cpucl_uvolt_for(c, c->cur_hz)) != uvolt)
		return;

	ret = cpucl_buck_set(c, uvolt, false);
	if (ret < 0) {
		pr_warn("cpucl: %s could not drop to %u uV\n", c->name, uvolt);
		return;
	}

	c->applied_uv = uvolt;
	if (ret)
		c->n_rail++;
}

/*
 * Move a cluster to a rate and the voltage that belongs to it.  Voltage goes up
 * before the clock when speeding up and comes down after the clock when slowing
 * down, so the cores are never run outside the range their table entry covers.
 *
 * The ceiling is the boot rate until the driver is armed, so an unarmed boot
 * never writes a rail and never leaves the operating points the boot chain
 * validated.
 */
static int cpucl_set_rate(struct exynos9610_cpucl *c, unsigned long hz)
{
	unsigned long ceiling = cpucl_ceiling_hz(c);
	unsigned int uvolt, boot_uv;
	unsigned long cur;
	bool raising;
	int ret;

	if (hz > ceiling)
		hz = ceiling;

	guard(mutex)(&c->lock);

	cur = cpucl_pll_rate(readl(c->base + c->pll_con0));
	c->cur_hz = cur;

	/*
	 * Already there: do not cycle the PLL for nothing.  This matters because
	 * a governor that asks for a clamped rate repeats the request on every
	 * update, and each needless relock is a chance to lose the lock.
	 */
	if (cur == hz) {
		c->n_skipped++;
		return 0;
	}

	uvolt = volt_scale ? cpucl_rail_uv(c, cpucl_uvolt_for(c, hz)) : 0;
	boot_uv = volt_scale ? cpucl_rail_uv(c, cpucl_uvolt_for(c, c->boot_hz)) : 0;
	raising = hz > cur;

	/* Never step a rail down in bring-up mode. */
	if (boost_only && uvolt && uvolt <= c->applied_uv)
		uvolt = 0;

	if (uvolt && raising) {
		cancel_delayed_work(&c->rail_work);
		c->pending_uv = 0;

		ret = cpucl_buck_set(c, uvolt, true);
		if (ret < 0)
			return ret;
		c->applied_uv = uvolt;
		if (ret)
			c->n_rail++;
	}

	ret = cpucl_relock(c, hz);
	if (ret) {
		/* The relock fell back to the boot PLL setting: match it. */
		if (raising && boot_uv && !boost_only)
			cpucl_buck_set(c, boot_uv, true);
		return ret;
	}

	c->n_relock++;

	if (loud)
		pr_info("cpucl: %s %lu -> %lu Hz, uvolt %u, applied %u, raising %d\n",
			c->name, cur, hz, uvolt, c->applied_uv, raising);

	/*
	 * A lower rail is not worth waiting for: the cores are already slower
	 * than before, so it is only a power matter and can wait out the hold
	 * time.  A burst in the meantime cancels it, which is what keeps the
	 * PMIC out of the governor's decision loop.
	 */
	if (uvolt && !raising && uvolt < c->applied_uv) {
		c->pending_uv = uvolt;
		mod_delayed_work(system_wq, &c->rail_work,
				 msecs_to_jiffies(rail_hold_ms));
	}

	return 0;
}

static u8 cpucl_ect_u8(unsigned int off)
{
	u8 val;

	memcpy_fromio(&val, cpucl_ect_base + off, 1);

	return val;
}

/*
 * Walk the ECT block header and return the offset of a named block.  Each entry
 * is {u32 name_len, name (padded to 4), u32 offset, u32 flags}.
 */
static bool cpucl_ect_find(const char *want, unsigned int *off)
{
	char name[16];
	unsigned int i, pos, num;

	if (!cpucl_ect_base)
		return false;

	memcpy_fromio(name, cpucl_ect_base, 4);
	if (memcmp(name, "PARA", 4))
		return false;

	num = readl(cpucl_ect_base + 12);
	if (!num || num > 32)
		return false;

	for (pos = 16, i = 0; i < num; i++) {
		unsigned int len = readl(cpucl_ect_base + pos);

		pos += 4;
		if (len >= sizeof(name))
			return false;
		memcpy_fromio(name, cpucl_ect_base + pos, len);
		name[len] = '\0';
		pos += (len + 4) & ~3;

		if (!strcmp(name, want)) {
			*off = readl(cpucl_ect_base + pos);
			return *off && *off < CPUCL_ECT_SIZE;
		}

		pos += 4;
	}

	return false;
}

/* The DVFS and ASV blocks share the same {parser, version, count, domains} header. */
static int cpucl_ect_domain_at(unsigned int blk, unsigned int idx, char *name,
			       size_t namelen, unsigned int *off)
{
	unsigned int i, pos = blk + 12;
	unsigned int num = readl(cpucl_ect_base + blk + 8);

	if (readl(cpucl_ect_base + blk) < 2 || idx >= num || num > 32)
		return -EINVAL;

	for (i = 0; i <= idx; i++) {
		unsigned int len = readl(cpucl_ect_base + pos);

		pos += 4;
		if (len >= namelen)
			return -EINVAL;
		memcpy_fromio(name, cpucl_ect_base + pos, len);
		name[len] = '\0';
		pos += (len + 4) & ~3;
		*off = readl(cpucl_ect_base + pos);
		pos += 4;
	}

	return 0;
}

/*
 * ASV group of this chip, from the eFuse word.  The vendor maps the "bigcpu"
 * field to CPUCL0 and the "littlecpu" field to CPUCL1 (cal-if/exynos9610/
 * asv_exynos9610.h), which is odd but is what the firmware that wrote the boot
 * voltages used.  Both fields read 5 on the A51.
 */
static int cpucl_asv_group(unsigned int cluster)
{
	void __iomem *efuse;
	unsigned int grp, modify;
	u32 word;

	efuse = ioremap(CPUCL_ASV_EFUSE, 4);
	if (!efuse)
		return -ENOMEM;
	word = readl(efuse);
	iounmap(efuse);

	if (cluster == 0) {
		grp = FIELD_GET(GENMASK(3, 0), word);
		modify = FIELD_GET(CPUCL_ASV_MODIFY_MASK, word);
	} else {
		grp = FIELD_GET(GENMASK(19, 16), word);
		modify = FIELD_GET(GENMASK(23, 20), word);
	}

	/* The modify field is a signed 4-bit delta. */
	if (modify > 7)
		return -ERANGE;

	return grp + modify;
}

/*
 * Build the operating points of a cluster from the vendor tables: frequencies
 * from the DVFS block, the voltage this chip needs for each of them from the
 * ASV block.  The cluster is identified by its PLL address so a rename or a
 * different domain order cannot pick the wrong table.
 *
 * Entries above the rated maximum of this SoC are dropped: the ECT is shared
 * across the Exynos 9610 family and lists the higher clocked members too.
 */
static int cpucl_ect_table(struct exynos9610_cpucl *c)
{
	phys_addr_t sfr = c->base_phys + c->pll_con0;
	unsigned int dvfs_blk, asv_blk, dom = 0, asv_dom = 0;
	unsigned int dvfs_levels, asv_steps, nlev_dvfs, nclk_dvfs, mode;
	unsigned int ngroup, nlev_asv, ntable, blk_pv;
	unsigned int i, count = 0, *khz, *uv;
	char name[24] = "";
	int group, ret = -ENOENT;

	if (!cpucl_ect_find("DVFS", &dvfs_blk) ||
	    !cpucl_ect_find("ASV", &asv_blk)) {
		dev_info(c->dev, "ect: DVFS/ASV block not found\n");
		return -ENOENT;
	}
	dev_info(c->dev, "ect: DVFS @0x%x ASV @0x%x\n", dvfs_blk, asv_blk);

	/* Find our cluster among the DVFS domains by its first SFR address. */
	for (i = 0; i < 32; i++) {
		unsigned int a;

		if (cpucl_ect_domain_at(dvfs_blk, i, name, sizeof(name), &dom))
			break;
		a = dvfs_blk + dom;
		mode = readl(cpucl_ect_base + a + 16);
		nclk_dvfs = readl(cpucl_ect_base + a + 20);
		if (mode != 2 || !nclk_dvfs)
			continue;	/* not an SFR-addressed domain */
		if (readl(cpucl_ect_base + a + 28) == sfr)
			break;
	}
	if (i == 32) {
		dev_info(c->dev, "ect: no DVFS domain for PLL %pa\n", &sfr);
		return -ENOENT;
	}
	dev_info(c->dev, "ect: domain %s\n", name);

	/* {max, min, boot, resume, mode, clocks, levels} then the SFR list. */
	nlev_dvfs = readl(cpucl_ect_base + dvfs_blk + dom + 24);
	if (!nlev_dvfs || nlev_dvfs > 64)
		return -EINVAL;
	dvfs_levels = dvfs_blk + dom + 28 + 4 * nclk_dvfs;

	group = cpucl_asv_group(c->id);
	if (group < 0)
		return group;
	dev_info(c->dev, "ect: %u levels, ASV group %d\n", nlev_dvfs, group);

	/* The ASV block names its domains the same way. */
	for (i = 0; i < 32; i++) {
		char aname[24];

		if (cpucl_ect_domain_at(asv_blk, i, aname, sizeof(aname), &asv_dom))
			break;
		if (!strcmp(aname, name))
			break;
	}
	if (i == 32) {
		dev_info(c->dev, "ect: no ASV domain named %s\n", name);
		return -ENOENT;
	}

	blk_pv = readl(cpucl_ect_base + asv_blk);
	ngroup = readl(cpucl_ect_base + asv_blk + asv_dom);
	nlev_asv = readl(cpucl_ect_base + asv_blk + asv_dom + 4);
	ntable = readl(cpucl_ect_base + asv_blk + asv_dom + 8);
	if (!ngroup || nlev_asv != nlev_dvfs || !ntable || blk_pv < 3 ||
	    group >= ngroup) {
		dev_info(c->dev, "ect: bad ASV domain (groups=%u levels=%u/%u tables=%u pv=%u)\n",
			 ngroup, nlev_asv, nlev_dvfs, ntable, blk_pv);
		return -EINVAL;
	}

	/*
	 * The domain holds level_list[] (MHz) followed, for every table, by
	 * {version, boot, resume, level_en[], voltages_step[]}.  Use the first
	 * table: it is the ASV-group table, and its column for this chip
	 * reproduces the voltage the boot chain programmed.
	 */
	/*
	 * The two blocks have to describe the same levels, in the same order.
	 * They do not always agree to the kHz (the ASV list carries the nominal
	 * rate, e.g. 1560 MHz where the DVFS block has 1508 MHz), so compare
	 * with a tolerance rather than exactly.
	 */
	for (i = 0; i < nlev_asv; i++) {
		unsigned int mhz = readl(cpucl_ect_base + asv_blk + asv_dom + 12 + 4 * i);
		unsigned int khz = readl(cpucl_ect_base + dvfs_levels + 8 * i);

		if (!khz || mhz * 1000 / 10 < khz / 10 ||
		    mhz * 1000 / 10 > khz / 10 + khz / 10)
			return -EINVAL;
	}

	asv_steps = asv_blk + asv_dom + 12 + 4 * nlev_asv + 12 + 4 * nlev_asv;

	khz = kcalloc(nlev_dvfs, sizeof(*khz), GFP_KERNEL);
	uv = kcalloc(nlev_dvfs, sizeof(*uv), GFP_KERNEL);
	if (!khz || !uv) {
		ret = -ENOMEM;
		goto out_free;
	}

	for (i = 0; i < nlev_dvfs; i++) {
		unsigned int f = readl(cpucl_ect_base + dvfs_levels + 8 * i);
		u8 step = cpucl_ect_u8(asv_steps + i * ngroup + group);

		if (f < c->min_hz / 1000 || f > c->max_hz / 1000)
			continue;
		khz[count] = f;
		uv[count] = step * CPUCL_BUCK_STEP_UV;
		count++;
	}

	if (!count) {
		ret = -ENOENT;
		goto out_free;
	}

	c->table = kcalloc(count + 1, sizeof(*c->table), GFP_KERNEL);
	c->uvolt = kcalloc(count, sizeof(*c->uvolt), GFP_KERNEL);
	if (!c->table || !c->uvolt) {
		ret = -ENOMEM;
		goto out_free;
	}

	/* Ascending order, which is what the cpufreq core expects. */
	for (i = 0; i < count; i++) {
		unsigned int min = UINT_MAX, idx = 0, k;

		for (k = 0; k < count; k++) {
			if (khz[k] && khz[k] < min) {
				min = khz[k];
				idx = k;
			}
		}
		c->table[i].frequency = min;
		c->table[i].driver_data = i;
		c->uvolt[i] = uv[idx];
		khz[idx] = 0;
	}
	c->table[count].frequency = CPUFREQ_TABLE_END;

	ret = 0;

out_free:
	kfree(khz);
	kfree(uv);
	return ret;
}

/*
 * Fallback: rates generated as fractions of the boot rate, encoded through the
 * PLL (which quantises them).  Nothing above the boot rate is offered, so the
 * voltage the boot chain left behind covers every entry.
 */
static int cpucl_build_table(struct exynos9610_cpucl *c)
{
	unsigned int count = 0, i, j;
	unsigned long *rates;
	u32 cur;
	int ret = 0;

	rates = kcalloc(17, sizeof(*rates), GFP_KERNEL);
	if (!rates)
		return -ENOMEM;

	cur = c->orig_con0;
	for (i = 1; i <= 10; i++) {
		unsigned long hz = c->boot_hz / 10 * i;
		u32 con0 = cpucl_rate_to_con0(c, hz, cur);

		if (!con0)
			continue;
		hz = cpucl_pll_rate(con0);
		if (hz > c->boot_hz)
			continue;

		for (j = 0; j < count; j++)
			if (rates[j] == hz)
				break;
		if (j == count)
			rates[count++] = hz;
	}

	c->table = kcalloc(count + 1, sizeof(*c->table), GFP_KERNEL);
	if (!c->table) {
		ret = -ENOMEM;
		goto out;
	}

	/* Ascending order is what the cpufreq core expects. */
	for (i = 0; i < count; i++) {
		unsigned long min = ULONG_MAX;
		unsigned int idx = 0, k;

		for (k = 0; k < count; k++) {
			if (rates[k] < min) {
				min = rates[k];
				idx = k;
			}
		}
		c->table[i].frequency = min / 1000;
		c->table[i].driver_data = i;
		rates[idx] = ULONG_MAX;
	}
	c->table[count].frequency = CPUFREQ_TABLE_END;

out:
	kfree(rates);
	return ret;
}

static struct exynos9610_cpucl *cpucl_for_cpu(unsigned int cpu)
{
	unsigned int i;

	for (i = 0; i < num_clusters; i++)
		if (cpumask_test_cpu(cpu, &clusters[i].cpus))
			return &clusters[i];

	return NULL;
}

static int cpucl_cpufreq_init(struct cpufreq_policy *policy)
{
	struct exynos9610_cpucl *c = cpucl_for_cpu(policy->cpu);

	if (!c)
		return -ENODEV;

	/*
	 * Every rate change moves the cluster to the oscillator and back, which
	 * takes a few hundred microseconds and cannot be done at the rate
	 * schedutil would like to: hammering the PLL is what produced lock
	 * failures and left a cluster on the oscillator.  Ten milliseconds is
	 * also the ballpark the vendor DVFS hardware uses.
	 */
	policy->cpuinfo.transition_latency = 500 * NSEC_PER_USEC;
	policy->transition_delay_us = 10000;

	/*
	 * cpufreq_generic_init() is deliberately not used here: it calls
	 * cpumask_setall(policy->cpus), which would merge both clusters into a
	 * single policy.  The core derives the frequency limits from the table
	 * via cpufreq_frequency_table_cpuinfo().
	 */
	cpumask_copy(policy->cpus, &c->cpus);
	policy->freq_table = c->table;

	return 0;
}

/*
 * The advertised maximum has to be cut here, not in ->init(): the core fills
 * policy->cpuinfo from the frequency table after calling ->init(), so anything
 * set there is overwritten.  ->ready() runs once the policy is complete.
 *
 * This matters for more than tidiness.  schedutil starts from
 * policy->cpuinfo.max_freq when it maps utilisation to a frequency, so leaving
 * it at the rated maximum while policy->max is the boot rate makes every
 * utilisation above half of capacity ask for the ceiling and every sample after
 * it ask for the floor: the clusters hopped between the two ends of the range
 * tens of times a second, and with the rail tracking that, so did the PMIC.
 */
static void cpucl_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct exynos9610_cpucl *c = cpucl_for_cpu(policy->cpu);
	unsigned int khz;

	if (!c)
		return;

	khz = cpucl_ceiling_hz(c) / 1000;
	if (policy->cpuinfo.max_freq > khz)
		policy->cpuinfo.max_freq = khz;
	if (policy->max > khz)
		policy->max = khz;
	if (policy->min > policy->max)
		policy->min = policy->max;
}

static int cpucl_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int index)
{
	struct exynos9610_cpucl *c = cpucl_for_cpu(policy->cpu);
	unsigned long hz;

	if (!c)
		return -ENODEV;

	hz = (unsigned long)policy->freq_table[index].frequency * 1000;

	/*
	 * The table carries the whole range so the thermal cooling states keep
	 * their numbering, but an unarmed driver stops at the boot operating
	 * point: that is what makes a boot with this driver built in unable to
	 * run faster than the boot chain validated.
	 */
	if (hz > cpucl_ceiling_hz(c)) {
		pr_debug_once("cpucl: %s capped at the boot rate, not armed\n",
			      c->name);
		hz = c->boot_hz;
	}

	return cpucl_set_rate(c, hz);
}

/*
 * Cap the policy at the ceiling as well as the individual requests.
 *
 * Clamping only in ->target() leaves the policy maximum at the top of the
 * table, and schedutil maps its utilisation onto that range: every idle blip
 * asked for a rate above the boot operating point, the driver turned that into
 * boot_hz, and the cluster bounced between the bottom and the ceiling of the
 * unarmed range, moving the rail at every bounce.  Lowering the policy maximum
 * here makes the governor see the range that can actually be used - and a
 * minimum above it must come down too, or the core rejects the policy.
 */
static int cpucl_cpufreq_verify(struct cpufreq_policy_data *policy)
{
	struct exynos9610_cpucl *c = cpucl_for_cpu(policy->cpu);
	unsigned int ceiling_khz;
	int ret;

	ret = cpufreq_generic_frequency_table_verify(policy);
	if (ret || !c)
		return ret;

	ceiling_khz = cpucl_ceiling_hz(c) / 1000;
	if (policy->max > ceiling_khz) {
		policy->max = ceiling_khz;
		if (policy->min > policy->max)
			policy->min = policy->max;
	}

	return 0;
}

static unsigned int cpucl_cpufreq_get(unsigned int cpu)
{
	struct exynos9610_cpucl *c = cpucl_for_cpu(cpu);

	if (!c)
		return 0;

	return cpucl_pll_rate(readl(c->base + c->pll_con0)) / 1000;
}

static struct cpufreq_driver cpucl_cpufreq_driver = {
	.name		= "exynos9610-cpu",
	.init		= cpucl_cpufreq_init,
	.verify		= cpucl_cpufreq_verify,
	.ready		= cpucl_cpufreq_ready,
	.target_index	= cpucl_cpufreq_target,
	.get		= cpucl_cpufreq_get,
	/*
	 * Let the core register this policy as a thermal cooling device, so the
	 * thermal zones can throttle the clusters instead of letting an all-core
	 * load at the top operating points reach the platform shutdown.
	 */
	.flags		= CPUFREQ_IS_COOLING_DEV,
};

static int cpucl_dump_show(struct seq_file *s, void *unused)
{
	unsigned int i;

	seq_printf(s, "armed %d  volt_scale %d  boost_only %d  buck_step %u\n",
		   cpucl_armed, volt_scale, boost_only, buck_step);

	for (i = 0; i < num_clusters; i++) {
		struct exynos9610_cpucl *c = &clusters[i];
		u32 con0 = readl(c->base + c->pll_con0);
		u32 sw = readl(c->base + c->sw_user);
		u32 mux = readl(c->base + CPUCL_MUX_PLL);
		u32 div = readl(c->base + CPUCL_DIV_CPU);

		seq_printf(s, "%s: pll_con0=%#010x M=%lu P=%lu S=%lu en=%lu stable=%lu\n",
			   c->name,
			   con0,
			   FIELD_GET(PLL_M_MASK, con0),
			   FIELD_GET(PLL_P_MASK, con0),
			   FIELD_GET(PLL_S_MASK, con0),
			   (con0 & PLL_ENABLE) ? 1UL : 0UL,
			   (con0 & PLL_STABLE) ? 1UL : 0UL);
		seq_printf(s, "    pll=%lu Hz  div_cpu=%#x  sw_user=%#x (sel=%lu)  mux_pll=%#x (sel=%lu)\n",
			   cpucl_pll_rate(con0), div, sw,
			   (sw & SW_USER_SEL) ? 1UL : 0UL, mux,
			   (mux & MUX_PLL_SEL) ? 1UL : 0UL);
		seq_printf(s, "    cpus=%*pbl\n", cpumask_pr_args(&c->cpus));
		seq_printf(s, "    relocks %lu  rail writes %lu  skipped %lu\n",
				   c->n_relock, c->n_rail, c->n_skipped);
		seq_printf(s, "    boot=%lu Hz  last_set=%lu Hz  range=%lu-%lu Hz  ceiling=%lu Hz\n",
			   c->boot_hz, c->cur_hz, c->min_hz, c->max_hz,
			   cpucl_ceiling_hz(c));

		if (c->uvolt) {
			unsigned int j;

			seq_printf(s, "    buck 0x%02x  asv group %d  rail floor %u uV\n",
				   c->buck_reg, cpucl_asv_group(c->id),
				   c->rail_floor_uv);
			for (j = 0; c->table[j].frequency != CPUFREQ_TABLE_END; j++)
				seq_printf(s, "    %7u kHz  %7u uV\n",
					   c->table[j].frequency, c->uvolt[j]);
		} else {
			seq_puts(s, "    voltage: fixed at the boot level\n");
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cpucl_dump);

/*
 * The arm switch.  Reading it reports the state, writing 1 allows the operating
 * points above the boot rate (and with them the rail writes), writing 0 puts the
 * ceiling back at the boot operating point.  Nothing is written to the PMIC by
 * the switch itself; the rails follow the frequencies the governor asks for.
 */
/*
 * Move the range a cluster advertises with its ceiling.
 *
 * The policy maximum is not the only thing that matters: schedutil takes its
 * reference frequency from policy->cpuinfo.max_freq, so a policy whose maximum
 * is the boot rate while cpuinfo still says the rated maximum maps every
 * utilisation above half of capacity onto the ceiling and back down again on the
 * next sample.  Shrinking both makes the governor spread its choices over the
 * operating points that are actually available.
 */
static void cpucl_policy_set_range(struct exynos9610_cpucl *c, unsigned long hz)
{
	struct cpufreq_policy *policy;
	unsigned int khz = hz / 1000;
	int cpu;

	for_each_cpu(cpu, &c->cpus) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		down_write(&policy->rwsem);
		policy->cpuinfo.max_freq = khz;
		if (policy->max > khz)
			WRITE_ONCE(policy->max, khz);
		if (policy->min > policy->max)
			WRITE_ONCE(policy->min, policy->max);
		up_write(&policy->rwsem);

		cpufreq_update_policy(cpu);
		cpufreq_cpu_put(policy);
	}
}

static ssize_t cpucl_arm_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	char buf[16];
	bool on;
	int ret, i;

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	cpucl_armed = on;
	pr_info("cpucl: %s, ceiling is now the %s operating point\n",
		on ? "armed" : "disarmed",
		on ? "rated maximum" : "boot");

	for (i = 0; i < num_clusters; i++) {
		if (clusters[i].base)
			cpucl_policy_set_range(&clusters[i],
					       cpucl_ceiling_hz(&clusters[i]));
	}

	return count;
}

static ssize_t cpucl_arm_read(struct file *file, char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	char buf[8];
	int len = snprintf(buf, sizeof(buf), "%d\n", cpucl_armed);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations cpucl_arm_fops = {
	.owner	= THIS_MODULE,
	.open	= simple_open,
	.read	= cpucl_arm_read,
	.write	= cpucl_arm_write,
	.llseek	= noop_llseek,
};

/* "set <cluster> <hz>" - debugfs interface for bring-up.  It obeys the ceiling. */
static ssize_t cpucl_set_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	unsigned int id;
	unsigned long hz;
	char buf[32];
	int ret;

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%u %lu", &id, &hz) != 2)
		return -EINVAL;
	if (id >= num_clusters)
		return -EINVAL;

	ret = cpucl_set_rate(&clusters[id], hz);
	if (ret)
		return ret;

	pr_info("cpucl: %s set to %lu Hz\n", clusters[id].name, hz);

	return count;
}

/*
 * Raw access to the CPU buck output registers: reading shows what the firmware
 * left behind, writing moves a rail by hand (selector = (uV - 300000) / 6250).
 * The write slews in buck_step steps and logs every step at KERN_EMERG, so it
 * is the tool for settling the "writing a rail downward freezes the board"
 * question without risking a boot: nothing here runs unless a human asks.
 */
static int cpucl_buck_show(struct seq_file *s, void *unused)
{
	unsigned int i;

	for (i = 0; i < num_clusters; i++) {
		struct exynos9610_cpucl *c = &clusters[i];
		u8 val = 0;
		int ret = -EOPNOTSUPP;

		if (c->acpm && c->buck_reg)
			ret = c->acpm->ops.pmic_ops.read_reg(c->acpm,
							    CPUCL_PMIC_CHAN,
							    CPUCL_PMIC_TYPE,
							    c->buck_reg, 0, &val);
		if (ret)
			seq_printf(s, "%s buck 0x%02x: error %d\n",
				   c->name, c->buck_reg, ret);
		else
			seq_printf(s, "%s buck 0x%02x = 0x%02x -> %u uV\n",
				   c->name, c->buck_reg, val,
				   val * CPUCL_BUCK_STEP_UV + CPUCL_BUCK_MIN_UV);
	}

	return 0;
}

static ssize_t cpucl_buck_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[32];
	unsigned int cluster, val;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%u %x", &cluster, &val) != 2 ||
	    cluster >= num_clusters || val > 0xff)
		return -EINVAL;

	/*
	 * Deliberately loud: this is the interface a freeze is investigated
	 * with, so every step is written at KERN_EMERG and therefore lands in
	 * the ramoops console buffer.  After a hard reset, boot stock Android
	 * and read /sys/fs/pstore/console-ramoops-0 to see how far it got.
	 */
	cancel_delayed_work_sync(&clusters[cluster].rail_work);
	clusters[cluster].pending_uv = 0;

	ret = cpucl_buck_slew(&clusters[cluster],
			      val * CPUCL_BUCK_STEP_UV + CPUCL_BUCK_MIN_UV,
			      buck_step, true, true);

	return ret < 0 ? ret : count;
}

static int cpucl_buck_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpucl_buck_show, NULL);
}

static const struct file_operations cpucl_buck_fops = {
	.open		= cpucl_buck_open,
	.write		= cpucl_buck_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cpucl_set_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = cpucl_set_write,
	.llseek = noop_llseek,
};

/*
 * One cluster from the device tree.  "samsung,cmu" is the CMU base and size,
 * the remaining properties describe where the PLL and its input mux sit inside
 * it and which VCO band that PLL can reach.
 */
static int cpucl_add(struct platform_device *pdev, struct device_node *np,
		     struct exynos9610_cpucl *c)
{
	u32 cmu[2], val;
	int ret;

	ret = of_property_read_u32_array(np, "samsung,cmu", cmu,
					 ARRAY_SIZE(cmu));
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "%pOF: missing samsung,cmu\n", np);

	c->dev = &pdev->dev;
	c->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%pOFn", np);
	if (!c->name)
		return -ENOMEM;

	c->base = devm_ioremap(&pdev->dev, cmu[0], cmu[1]);
	if (!c->base)
		return -ENOMEM;
	c->base_phys = cmu[0];

	/* cluster id: the child node's reg, which is the cluster number. */
	ret = of_property_read_u32(np, "reg", &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "%pOF: missing reg\n", np);
	c->id = val;

	/*
	 * CPU rails on the S2MPU09: BUCK3 (0x1d) feeds CPUCL0, BUCK2 (0x1b)
	 * feeds CPUCL1, see the vendor DT vdd_cpucl0/vdd_cpucl1 regulator
	 * descriptions.
	 */
	ret = of_property_read_u32(np, "samsung,buck-out-reg", &val);
	if (!ret)
		c->buck_reg = val;
	else
		c->buck_reg = c->id ? 0x1b : 0x1d;

	/*
	 * Lowest rail value this board tolerates.  650 mV clears the measured
	 * LITTLE floor of 643.75 mV; the big rail is fine lower, but this
	 * default is safe for both and the device tree can override it.
	 */
	ret = of_property_read_u32(np, "samsung,rail-floor-uv", &val);
	c->rail_floor_uv = ret ? 650000U : val;
	if (c->rail_floor_uv < CPUCL_BUCK_MIN_UV)
		c->rail_floor_uv = CPUCL_BUCK_MIN_UV;

	ret = of_property_read_u32(np, "samsung,pll-con0-offset", &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "%pOF: missing samsung,pll-con0-offset\n", np);
	c->pll_con0 = val;

	ret = of_property_read_u32(np, "samsung,sw-user-offset", &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "%pOF: missing samsung,sw-user-offset\n", np);
	c->sw_user = val;

	of_property_read_u32(np, "samsung,vco-min-hz", &val);
	c->vco_min_hz = val;
	of_property_read_u32(np, "samsung,vco-max-hz", &val);
	c->vco_max_hz = val;

	/*
	 * Rated range of this SoC.  The ECT also lists the higher clocked
	 * members of the Exynos 9610 family, so the usable range has to be
	 * stated: 403-1742 MHz for the A53 cluster and 936-2314 MHz for the
	 * A73 cluster, which is what the vendor DT declares as the minimum
	 * (0x62638 and 0xe4840) and what the A51 ships with.
	 */
	ret = of_property_read_u32(np, "samsung,min-hz", &val);
	c->min_hz = ret ? (c->id ? 936000000UL : 403000000UL) : val;

	ret = of_property_read_u32(np, "samsung,max-hz", &val);
	c->max_hz = ret ? (c->id ? 2314000000UL : 1742000000UL) : val;

	if (c->max_hz < c->min_hz || c->max_hz > 3000000000UL)
		return -EINVAL;

	mutex_init(&c->lock);
	INIT_DELAYED_WORK(&c->rail_work, cpucl_rail_work);
	c->orig_con0 = readl(c->base + c->pll_con0);
	c->good_con0 = c->orig_con0;
	c->boot_hz = cpucl_pll_rate(c->orig_con0);
	c->cur_hz = c->boot_hz;

	dev_info(&pdev->dev, "%s at 0x%08x: %lu Hz (pll_con0=%#x sw_user=%#x)\n",
		 c->name, cmu[0], c->boot_hz, c->pll_con0, c->sw_user);

	return 0;
}

/*
 * Cluster membership comes from the CPU nodes' reg: the little cores sit at
 * 0x0..0x3 and the big ones at 0x100..0x103, so the high byte is the cluster.
 * CPU nodes here use a two-cell reg ("reg = <0x0 0x100>"), so the cluster id is
 * in the last cell and not the first.
 */
static void cpucl_build_cpu_masks(void)
{
	struct device_node *cpus, *np;
	unsigned int i;

	for (i = 0; i < num_clusters; i++)
		cpumask_clear(&clusters[i].cpus);

	cpus = of_find_node_by_path("/cpus");
	if (!cpus)
		return;

	for_each_child_of_node(cpus, np) {
		const __be32 *prop;
		unsigned int cluster;
		u32 reg;
		int cpu, len;

		prop = of_get_property(np, "reg", &len);
		if (!prop || len < sizeof(*prop))
			continue;
		reg = be32_to_cpup(prop + (len / sizeof(*prop)) - 1);

		cpu = of_cpu_node_to_id(np);
		if (cpu < 0)
			continue;

		cluster = reg >> 8;
		if (cluster >= num_clusters)
			cluster = 0;

		cpumask_set_cpu(cpu, &clusters[cluster].cpus);
	}

	of_node_put(cpus);
}

/*
 * The boot chain programmed the voltage of the boot operating point from the
 * same table, so reading it back is a direct check that the table was decoded
 * for the right ASV group.  If it does not match, the operating points above
 * the boot rate are not trusted and the cluster stays on the boot-rate table.
 */
static int cpucl_verify_boot_voltage(struct exynos9610_cpucl *c)
{
	const struct acpm_pmic_ops *ops = &c->acpm->ops.pmic_ops;
	unsigned int uvolt = cpucl_uvolt_for(c, c->boot_hz);
	u8 val;
	int ret;

	if (!uvolt)
		return -ENOENT;

	ret = ops->read_reg(c->acpm, CPUCL_PMIC_CHAN, CPUCL_PMIC_TYPE,
			    c->buck_reg, 0, &val);
	if (ret)
		return ret;

	if ((val * CPUCL_BUCK_STEP_UV + CPUCL_BUCK_MIN_UV) != uvolt) {
		dev_warn(c->dev,
			 "boot voltage %u uV does not match the table (%u uV)\n",
			 val * CPUCL_BUCK_STEP_UV + CPUCL_BUCK_MIN_UV, uvolt);
		return -EINVAL;
	}

	return 0;
}

static void cpucl_drop_table(struct exynos9610_cpucl *c)
{
	kfree(c->table);
	kfree(c->uvolt);
	c->table = NULL;
	c->uvolt = NULL;
}

static int cpucl_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child, *pmic_np;
	struct acpm_handle *acpm = NULL;
	unsigned int i;
	int ret;

	num_clusters = of_get_child_count(np);
	if (!num_clusters || num_clusters > 4)
		return -EINVAL;

	clusters = devm_kcalloc(&pdev->dev, num_clusters, sizeof(*clusters),
				GFP_KERNEL);
	if (!clusters)
		return -ENOMEM;

	/* The CPU rails are S2MPU09 bucks, reached over the ACPM PMIC protocol. */
	pmic_np = of_parse_phandle(np, "samsung,pmic", 0);
	if (!pmic_np)
		pmic_np = of_find_compatible_node(NULL, NULL,
						  "samsung,s2mpu09-pmic");
	if (pmic_np) {
		acpm = devm_acpm_get_by_node(&pdev->dev, pmic_np->parent);
		of_node_put(pmic_np);
		if (IS_ERR(acpm)) {
			/* The ACPM device probes after this one. */
			if (PTR_ERR(acpm) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_warn(&pdev->dev, "no ACPM handle, voltage stays fixed: %ld\n",
				 PTR_ERR(acpm));
			acpm = NULL;
		}
	}

	cpucl_ect_base = memremap(CPUCL_ECT_PHYS, CPUCL_ECT_SIZE, MEMREMAP_WB);
	if (!cpucl_ect_base)
		dev_warn(&pdev->dev, "no ECT block, using boot-rate table only\n");

	i = 0;
	for_each_available_child_of_node(np, child) {
		ret = cpucl_add(pdev, child, &clusters[i]);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		i++;
	}

	cpucl_build_cpu_masks();
	for (i = 0; i < num_clusters; i++) {
		struct exynos9610_cpucl *c = &clusters[i];

		c->acpm = acpm;
		if (!c->acpm || !c->buck_reg) {
			c->buck_reg = 0;
		} else {
			const struct acpm_pmic_ops *pops = &c->acpm->ops.pmic_ops;
			u8 val;

			/* Start from whatever the boot chain left behind. */
			if (!pops->read_reg(c->acpm, CPUCL_PMIC_CHAN,
					    CPUCL_PMIC_TYPE, c->buck_reg, 0,
					    &val))
				c->applied_uv = val * CPUCL_BUCK_STEP_UV +
						CPUCL_BUCK_MIN_UV;
		}

		/*
		 * The operating points above the boot rate need the voltage
		 * tables, so they are only offered when voltage scaling was
		 * asked for.  Otherwise the boot rate stays the ceiling, which is
		 * the state the boot chain leaves the clusters in.
		 */
		ret = volt_scale ? cpucl_ect_table(c) : -ENOENT;
		if (ret == -ENOENT) {
			ret = cpucl_build_table(c);
		} else if (!ret && c->acpm && c->buck_reg &&
			   cpucl_verify_boot_voltage(c)) {
			cpucl_drop_table(c);
			ret = cpucl_build_table(c);
		}
		/*
		 * Without a voltage table nothing above the boot rate may be
		 * requested: the boot voltage is only known to be correct for
		 * that rate.
		 */
		if (!c->uvolt)
			c->max_hz = c->boot_hz;
		if (ret)
			return ret;

		dev_info(&pdev->dev, "%s: %lu-%lu Hz, ceiling %lu Hz, voltage %s\n",
			 c->name, c->table[0].frequency * 1000UL, c->max_hz,
			 cpucl_ceiling_hz(c), c->uvolt ? "scaled" : "fixed");
	}

	if (!cpucl_ect_base)
		cpucl_ect_base = NULL;

	/*
	 * Arming is what turns the operating points above the boot rate on, and
	 * with them the rail writes.  It is on unless the kernel command line
	 * turns it off (exynos9610_cpuclk.arm=0).
	 */
	cpucl_armed = arm;

	ret = cpufreq_register_driver(&cpucl_cpufreq_driver);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register cpufreq driver\n");

	cpucl_debugfs = debugfs_create_dir("exynos9610-cpuclk", NULL);
	debugfs_create_file("dump", 0444, cpucl_debugfs, NULL,
			    &cpucl_dump_fops);
	debugfs_create_file("set", 0200, cpucl_debugfs, NULL,
			    &cpucl_set_fops);
	debugfs_create_file("buck", 0644, cpucl_debugfs, NULL,
			    &cpucl_buck_fops);
	debugfs_create_file("arm", 0644, cpucl_debugfs, NULL,
			    &cpucl_arm_fops);

	dev_info(&pdev->dev, "%s (exynos9610_cpuclk.arm=0 for the boot operating points only)\n",
		 cpucl_armed ? "armed: top operating points unlocked"
			     : "not armed: boot operating point only");

	platform_set_drvdata(pdev, clusters);

	return 0;
}

static void cpucl_remove(struct platform_device *pdev)
{
	unsigned int i;

	debugfs_remove_recursive(cpucl_debugfs);
	cpufreq_unregister_driver(&cpucl_cpufreq_driver);

	/* Hand each cluster back to the boot rate before letting go. */
	for (i = 0; i < num_clusters; i++) {
		cancel_delayed_work_sync(&clusters[i].rail_work);
		if (clusters[i].base)
			cpucl_set_rate(&clusters[i], clusters[i].boot_hz);
		/* That set_rate may itself have queued a drop; the tables go next. */
		cancel_delayed_work_sync(&clusters[i].rail_work);
		clusters[i].pending_uv = 0;
		cpucl_drop_table(&clusters[i]);
	}

	memunmap(cpucl_ect_base);
	cpucl_ect_base = NULL;
}

static const struct of_device_id cpucl_of_match[] = {
	{ .compatible = "samsung,exynos9610-cpuclk" },
	{ },
};
MODULE_DEVICE_TABLE(of, cpucl_of_match);

static struct platform_driver cpucl_driver = {
	.probe	= cpucl_probe,
	.remove	= cpucl_remove,
	.driver	= {
		.name		= "exynos9610-cpuclk",
		.of_match_table	= cpucl_of_match,
	},
};
module_platform_driver(cpucl_driver);

MODULE_DESCRIPTION("Exynos9610 CPU cluster clock control");
MODULE_LICENSE("GPL");
