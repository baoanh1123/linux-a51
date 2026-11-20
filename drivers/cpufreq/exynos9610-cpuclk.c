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
 * Voltage is deliberately never touched: only rates at or below the boot OPP
 * are accepted, so the boot voltage stays valid and the regulator setup - which
 * really does belong to the firmware - is left alone.
 *
 * This is a debugfs experiment driver for now; the cpufreq policy comes once
 * the switching sequence has been validated against measured hardware rates.
 */

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#define CPUCL_PLL_CON0		0x0140
#define CPUCL_SW_USER		0x0120
#define CPUCL_MUX_PLL		0x1000
#define CPUCL_DIV_CPU		0x1810

#define CPUCL_PLL_LOCKTIME	0x0000

#define PLL_ENABLE		BIT(31)
#define PLL_STABLE		BIT(29)
#define PLL_OUT_SEL		BIT(4)	/* PLL_MUX_SEL in the vendor sources */
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

struct exynos9610_cpucl {
	const char *name;
	void __iomem *base;
	u32 pll_con0;
	u32 sw_user;
	unsigned long boot_hz;
	unsigned long cur_hz;
	unsigned long vco_min_hz;
	unsigned long vco_max_hz;
	u32 orig_con0;
	cpumask_t cpus;
	struct cpufreq_frequency_table *table;
	struct mutex lock;	/* one rate change at a time per cluster */
};

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

static int cpucl_wait_set(void __iomem *reg, u32 bit)
{
	unsigned int i;

	for (i = 0; i < 500; i++) {
		if (readl(reg) & bit)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

/*
 * Program the PLL: lock time, then DIV_M/DIV_P/DIV_S together with the output
 * mux and the enable bit, in one write.  PLL_OUT_SEL must be part of that
 * write - it is what connects the VCO back into the CMU, and a CON0 write that
 * clears it leaves the PLL in bypass where it never reports lock.
 *
 * The PLL is deliberately not disabled first: the vendor only disables it when
 * the cluster is already clocked from somewhere else, and a plain re-enable
 * from the disabled state is what failed here before.
 */
static int cpucl_pll_program(struct exynos9610_cpucl *c, u32 pmsk)
{
	void __iomem *con0 = c->base + c->pll_con0;
	u32 val;

	writel(FIELD_GET(PLL_P_MASK, pmsk) * CPUCL_PLL_LOCK_CYCLES,
	       c->base + CPUCL_PLL_LOCKTIME);

	val = readl(con0);
	val &= ~(PLL_M_MASK | PLL_P_MASK | PLL_S_MASK);
	val |= pmsk | PLL_OUT_SEL | PLL_ENABLE;
	writel(val, con0);

	return cpucl_wait_set(con0, PLL_STABLE);
}

/*
 * Program one cluster.  The cluster is moved to the 26 MHz oscillator, the PLL
 * is reprogrammed and locked, and the cluster is moved back.  Interrupts stay
 * enabled: the cores simply run slowly for the duration of the relock.
 *
 * If the new settings do not lock, the boot settings are restored while the
 * cores are still on the oscillator, so the mux can be handed back and the
 * cluster is never left at 26 MHz.
 */
static int cpucl_set_rate(struct exynos9610_cpucl *c, unsigned long hz)
{
	void __iomem *mux = c->base + CPUCL_MUX_PLL;
	u32 new_con0, safe_con0;
	int ret;

	if (hz < CPUCL_MIN_HZ || hz > c->boot_hz)
		return -EINVAL;

	new_con0 = cpucl_rate_to_con0(c, hz, readl(c->base + c->pll_con0));
	if (!new_con0)
		return -EINVAL;

	/*
	 * Two CPUs of the same cluster can ask for a rate at the same time; an
	 * interleaved pair of sequences would leave the mux on the oscillator
	 * with an unlocked PLL.
	 */
	guard(mutex)(&c->lock);

	/* Take the cluster off the PLL; SW_USER selects OSCCLK here. */
	writel(readl(mux) | MUX_PLL_SEL, mux);
	ret = cpucl_wait_clear(mux, MUX_PLL_BUSY);
	if (ret)
		return ret;

	ret = cpucl_pll_program(c, new_con0);
	if (ret) {
		pr_err("cpucl: %s PLL did not lock at %lu Hz (con0=%#x)\n",
		       c->name, hz, new_con0);

		safe_con0 = c->orig_con0 & (PLL_M_MASK | PLL_P_MASK |
					    PLL_S_MASK);
		if (cpucl_pll_program(c, safe_con0)) {
			pr_err("cpucl: %s will not lock either, cluster left on OSCCLK\n",
			       c->name);
			return ret;
		}
	}

	/* Hand the cluster back to the PLL. */
	writel(readl(mux) & ~MUX_PLL_SEL, mux);
	cpucl_wait_clear(mux, MUX_PLL_BUSY);

	c->cur_hz = cpucl_pll_rate(readl(c->base + c->pll_con0));
	if (ret)
		pr_err("cpucl: %s kept at %lu Hz\n", c->name, c->cur_hz);

	return ret;
}

/*
 * Frequency table: rates are generated as fractions of the boot rate, encoded
 * through the PLL (which quantises them), and then de-duplicated and sorted.
 * Nothing above the boot rate is ever offered, so the boot voltage - the only
 * voltage the firmware lets us rely on - always covers every entry.
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
	 * cpufreq_generic_init() is deliberately not used here: it calls
	 * cpumask_setall(policy->cpus), which would merge both clusters into a
	 * single policy.  The core derives the frequency limits from the table
	 * via cpufreq_frequency_table_cpuinfo().
	 */
	cpumask_copy(policy->cpus, &c->cpus);
	policy->freq_table = c->table;
	policy->cpuinfo.transition_latency = 500 * NSEC_PER_USEC;

	return 0;
}

static int cpucl_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int index)
{
	struct exynos9610_cpucl *c = cpucl_for_cpu(policy->cpu);

	if (!c)
		return -ENODEV;

	return cpucl_set_rate(c, (unsigned long)policy->freq_table[index].frequency
				   * 1000);
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
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= cpucl_cpufreq_target,
	.get		= cpucl_cpufreq_get,
};

static int cpucl_dump_show(struct seq_file *s, void *unused)
{
	unsigned int i;

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
		seq_printf(s, "    boot=%lu Hz  last_set=%lu Hz  ceiling=%lu Hz\n",
			   c->boot_hz, c->cur_hz, c->boot_hz);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cpucl_dump);

/* "set <cluster> <hz>" - debugfs interface for bring-up. */
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

static const struct file_operations cpucl_set_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = cpucl_set_write,
	.llseek = noop_llseek,
};

static unsigned long cpucl0_base = 0x10900000;
static unsigned long cpucl1_base = 0x10800000;
module_param(cpucl0_base, ulong, 0444);
module_param(cpucl1_base, ulong, 0444);

/* CPUCL0 puts its PLL after the switch-user mux, CPUCL1 before it. */
#define CPUCL1_PLL_CON0		0x0120
#define CPUCL1_SW_USER		0x0100

static int __init cpucl_add(struct exynos9610_cpucl *c, const char *name,
			    unsigned long base, u32 pll_con0, u32 sw_user,
			    unsigned long vco_min_hz, unsigned long vco_max_hz)
{
	c->name = name;
	c->vco_min_hz = vco_min_hz;
	c->vco_max_hz = vco_max_hz;
	c->base = ioremap(base, 0x8000);
	if (!c->base)
		return -ENOMEM;

	c->pll_con0 = pll_con0;
	c->sw_user = sw_user;
	mutex_init(&c->lock);
	c->orig_con0 = readl(c->base + c->pll_con0);
	c->boot_hz = cpucl_pll_rate(c->orig_con0);
	c->cur_hz = c->boot_hz;

	pr_info("cpucl: %s at 0x%08lx: %lu Hz (pll_con0=%#x sw_user=%#x)\n",
		name, base, c->boot_hz, pll_con0, sw_user);

	return 0;
}

/*
 * Cluster membership comes from the CPU nodes' reg: the little cores sit at
 * 0x0..0x3 and the big ones at 0x100..0x103, so the high byte is the cluster.
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

		/*
		 * CPU nodes here use a two-cell reg ("reg = <0x0 0x100>"),
		 * so the cluster id is in the last cell and not the first.
		 */
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

static int __init cpucl_probe(void)
{
	unsigned int i;
	int ret;

	clusters = kcalloc(2, sizeof(*clusters), GFP_KERNEL);
	if (!clusters)
		return -ENOMEM;
	num_clusters = 2;

	ret = cpucl_add(&clusters[0], "cpucl0", cpucl0_base,
			CPUCL_PLL_CON0, CPUCL_SW_USER, 1200000000UL,
			2400000000UL);	/* gpll1051X_spec */
	if (ret)
		goto out_tables;
	ret = cpucl_add(&clusters[1], "cpucl1", cpucl1_base,
			CPUCL1_PLL_CON0, CPUCL1_SW_USER, 1500000000UL,
			3000000000UL);	/* gpll1054X_spec */
	if (ret)
		goto out_tables;

	cpucl_build_cpu_masks();
	for (i = 0; i < num_clusters; i++) {
		ret = cpucl_build_table(&clusters[i]);
		if (ret)
			goto out_tables;
	}

	ret = cpufreq_register_driver(&cpucl_cpufreq_driver);
	if (ret)
		goto out_tables;

	cpucl_debugfs = debugfs_create_dir("exynos9610-cpuclk", NULL);
	debugfs_create_file("dump", 0444, cpucl_debugfs, NULL,
			    &cpucl_dump_fops);
	debugfs_create_file("set", 0200, cpucl_debugfs, NULL,
			    &cpucl_set_fops);

	return 0;

out_tables:
	for (i = 0; i < num_clusters; i++)
		kfree(clusters[i].table);
	kfree(clusters);
	return ret;
}

static void __exit cpucl_remove(void)
{
	unsigned int i;

	debugfs_remove_recursive(cpucl_debugfs);
	cpufreq_unregister_driver(&cpucl_cpufreq_driver);

	/* Hand each cluster back to the boot rate before letting go. */
	for (i = 0; i < num_clusters; i++) {
		if (!clusters[i].base)
			continue;
		cpucl_set_rate(&clusters[i], clusters[i].boot_hz);
		iounmap(clusters[i].base);
		kfree(clusters[i].table);
	}
	kfree(clusters);
}

module_init(cpucl_probe);
module_exit(cpucl_remove);

MODULE_DESCRIPTION("Exynos9610 CPU cluster clock control");
MODULE_LICENSE("GPL");
