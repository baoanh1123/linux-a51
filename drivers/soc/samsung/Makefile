# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_EXYNOS_ASV_ARM)	+= exynos5422-asv.o
obj-$(CONFIG_EXYNOS_CHIPID)	+= exynos_chipid.o
exynos_chipid-y			+= exynos-chipid.o exynos-asv.o

obj-$(CONFIG_EXYNOS_USI)	+= exynos-usi.o

obj-$(CONFIG_EXYNOS_PMU)	+= exynos-pmu.o

obj-$(CONFIG_EXYNOS_PMU_ARM_DRIVERS)	+= exynos3250-pmu.o exynos4-pmu.o \
					exynos5250-pmu.o exynos5420-pmu.o
obj-$(CONFIG_EXYNOS_REGULATOR_COUPLER) += exynos-regulator-coupler.o
obj-$(CONFIG_EXYNOS9611_MSHC_SNAPSHOT) += exynos9611-mshc-snapshot.o

obj-$(CONFIG_SAMSUNG_PM_CHECK)	+= s3c-pm-check.o
obj-$(CONFIG_EXYNOS9610_CPUHP)		+= exynos9610_cpuhp.o
obj-$(CONFIG_EXYNOS9610_G3D_PD)		+= exynos9610-g3d-pd.o
obj-$(CONFIG_EXYNOS9610_WLBT)		+= exynos9610-wlbt.o
exynos9610-wlbt-y := wlbt-core.o wlbt-hip4.o wlbt-mlme.o wlbt-debug.o wlbt-net.o
CFLAGS_wlbt-debug.o := -Os
