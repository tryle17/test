// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019-2021 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/sched/sysctl.h>
#include <linux/version.h>
#include <linux/freezer.h>
#include <linux/workqueue.h>
#include "cpufreq_bouncing.h"

#define NSEC_TO_MSEC(val) (val / NSEC_PER_MSEC)
#define MSEC_TO_NSEC(val) (val * NSEC_PER_MSEC)
#define MSEC_TO_USEC(val) (val * USEC_PER_MSEC)
#define NR_FREQ 64 /* since MTK platform could reach 32 levels */
#define NR_CLUS_MAX 3
#define NR_CORE_MAX 8

struct freq_qos_request *cpu_req;

/* cluster based */
struct cpufreq_bouncing {
	int first_cpu;

	/* statistics */
	u64 last_ts;
	u64 last_freq_update_ts;
	u64 acc;
	s64 update_ts_ns;

	/* restriction */
	int limit_freq;
	int limit_level;
	u64 limit_thres;

	/* freqs */
	int freq_sorting;
	int freq_levels;
	unsigned int freqs[NR_FREQ]; /* quick mapping */

	/* trace info */
	long long freqs_resident[NR_FREQ]; /* for record how long freqs stay */

	/* config */
	bool enable;
	int cur_level;

	/* config: ceil */
	int max_level;
	int down_speed;
	s64 down_limit_ns;
	unsigned int max_freq;

	/* config: floor */
	int min_level;
	int up_speed;
	s64 up_limit_ns;
	unsigned int min_freq;

	/*
	 * check current limitation
	 * if limitation higher than target, not count
	 */

	/* control freq boundary */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	struct freq_qos_request qos_req;
	struct kthread_work qos_work;
#endif
	bool ctl_inited;
} cb_stuff[NR_CLUS_MAX] = {
	/* silver */
	{
		.first_cpu     = -1,
		.enable        = false,
		.freqs_resident[0 ... NR_FREQ - 1] = -1,
		.freqs[0 ... NR_FREQ - 1] = FREQ_QOS_MAX_DEFAULT_VALUE,
	},
	/* gold */
	{
		.first_cpu     = -1,
		.enable        = true,
		.down_limit_ns = 50 * NSEC_PER_MSEC,
		.up_limit_ns   = 50 * NSEC_PER_MSEC,
		.limit_thres   = 30 * NSEC_PER_MSEC,
		.limit_freq    = 2100000,
		.limit_level   = 8,
		.down_speed    = 2,
		.up_speed      = 1,
		.freqs_resident[0 ... NR_FREQ - 1] = -1,
		.freqs[0 ... NR_FREQ - 1] = FREQ_QOS_MAX_DEFAULT_VALUE,
	},
	/* gold prime */
	{
		.first_cpu     = -1,
		.enable        = true,
		.down_limit_ns = 50 * NSEC_PER_MSEC,
		.up_limit_ns   = 50 * NSEC_PER_MSEC,
		.limit_thres   = 30 * NSEC_PER_MSEC,
		.limit_freq    = 2300000,
		.limit_level   = 10,
		.down_speed    = 2,
		.up_speed      = 1,
		.freqs_resident[0 ... NR_FREQ - 1] = -1,
		.freqs[0 ... NR_FREQ - 1] = FREQ_QOS_MAX_DEFAULT_VALUE,
	}
};

/* main thread */
static struct task_struct *cb_task;

/* sleep range */
static unsigned long sleep_range_ms[2] = {20, 30};
module_param_array(sleep_range_ms, ulong, NULL, 0664);

/* core boost */
static u64 last_core_boost_ts;
static bool last_core_boost;
static bool is_sbe_rescue;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
static bool freq_qos_check = true;
module_param_named(freq_qos_check, freq_qos_check, bool, 0664);
static struct kthread_worker *cb_qos_kw;
#endif

static int update_delay = 8 * NSEC_PER_MSEC; /* 8ms update window */
module_param_named(update_delay, update_delay, int, 0664);

static bool update_delay_check;
module_param_named(update_delay_check, update_delay_check, bool, 0664);

static unsigned int core_boost_lat_ns = MSEC_TO_NSEC(50);
module_param_named(core_boost_lat_ns, core_boost_lat_ns, uint, 0664);

static bool core_ctl_check;
module_param_named(core_ctl_check, core_ctl_check, bool, 0664);

static void cb_reset_qos(int cpu)
{
	struct cpufreq_bouncing *cb = &cb_stuff[cpu];
#ifdef CONFIG_OPLUS_UAG_SOFT_LIMIT
	struct cpufreq_policy *pol = cpufreq_cpu_get_raw(cb->first_cpu);
#endif /* CONFIG_OPLUS_UAG_SOFT_LIMIT */

	if (!cb->ctl_inited)
		return;

#ifdef CONFIG_OPLUS_UAG_SOFT_LIMIT
	if (pol)
		set_soft_limit_freq(pol, pol->cpuinfo.max_freq);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	/* reset qos */
	if (freq_qos_update_request(&cb->qos_req, FREQ_QOS_MAX_DEFAULT_VALUE) < 0)
		pr_warn("cluster %d reset qos req failed.\n", cpu);
#endif
#endif /* CONFIG_OPLUS_UAG_SOFT_LIMIT */
}

/* init config */
static bool self_activate;

static int self_activate_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", self_activate);
}

static int self_activate_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;
	int i;

	if (sscanf(buf, "%u\n", &val) == 0)
		return -EINVAL;

	self_activate = !!val;

	if (self_activate)
		wake_up_process(cb_task);
	else {
		for (i = 0; i < NR_CLUS_MAX; ++i)
			cb_reset_qos(i);
	}

	return 0;
}

static struct kernel_param_ops self_activate_ops = {
	.get = self_activate_show,
	.set = self_activate_store,
};
module_param_cb(self_activate, &self_activate_ops, NULL, 0664);

#ifdef CONFIG_ARCH_MEDIATEK
static bool enable = false;
#else
static bool enable = true;
#endif

static int enable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", enable);
}

static int enable_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;
	int i;

	if (sscanf(buf, "%u\n", &val) == 0) {
		pr_warn("cb input invalid (%s)\n", buf);
		return -EINVAL;
	}

	enable = (bool) val;

	/* reset all */
	is_sbe_rescue = false;
	for (i = 0; i < NR_CLUS_MAX; ++i) {
		struct cpufreq_bouncing *cb = &cb_stuff[i];

		cb_reset_qos(i);

		/* reset core boost */
		last_core_boost = false;
		last_core_boost_ts = 0;
		/* cb_do_core_boost_work(); */

		/* reset acc */
		cb->last_ts = 0;
		cb->last_freq_update_ts = 0;
		cb->acc = 0;
		cb->cur_level = cb->max_level ? cb->max_level : 0;
	}

	if (self_activate)
		wake_up_process(cb_task);

	pr_info("cb %s\n", enable ? "enabled" : "disabled");

	return 0;
}

static struct kernel_param_ops enable_ops = {
	.get = enable_show,
	.set = enable_store,
};
module_param_cb(enable, &enable_ops, NULL, 0664);

static bool debug;
module_param_named(debug, debug, bool, 0664);

static unsigned int decay = 80;
module_param_named(decay, decay, uint, 0664);

static DEFINE_PER_CPU(struct cpufreq_bouncing*, cbs);

static int cb_pol_idx;

static bool cb_switch;

static inline struct cpufreq_bouncing *cb_get(int cpu)
{
	if (cpu < 0 || cpu >= NR_CORE_MAX)
		return NULL;

	if (likely(cb_switch)) {
		struct cpufreq_bouncing *cb = per_cpu(cbs, cpu);

		if (unlikely(!cb))
			return NULL;

		if (unlikely(!cb->ctl_inited))
			return NULL;

		if (!enable || !cb->enable)
			return NULL;

		return cb;
	}

	return NULL;
}

/* module parameters */
static int cb_config_store(const char *buf, const struct kernel_param *kp)
{
	/*
	 * for limit_thres, down/up limit will use ms as unit
	 * format:
	 * echo "2,0,5,3000,3,2000,3,2000" > config
	 */
	struct pack {
		int clus;
		int enable;

		int limit_level;
		u64 limit_thres_ms;

		int down_speed;
		s64 down_limit_ms;

		int up_speed;
		s64 up_limit_ms;
	} v;
	struct cpufreq_bouncing *cb;

	if (debug)
		pr_info("%s\n", buf);

	if (sscanf(buf, "%d,%d,%d,%llu,%d,%lld,%d,%lld\n",
		&v.clus,
		&v.enable,
		&v.limit_level,
		&v.limit_thres_ms,
		&v.down_speed,
		&v.down_limit_ms,
		&v.up_speed,
		&v.up_limit_ms) != 8)
		goto out;

	if (v.clus < 0 || v.clus >= cb_pol_idx)
		goto out;

	cb = &cb_stuff[v.clus];

	if (v.limit_level < 0 || v.limit_level >= cb->freq_levels)
		goto out;

	if (v.down_speed < 0 || v.down_speed > cb->freq_levels)
		goto out;

	if (v.up_speed < 0 || v.up_speed > cb->freq_levels)
		goto out;

	if (v.down_limit_ms < 0 || v.up_limit_ms < 0 ||
		v.limit_thres_ms < 0)
		goto out;

	/* begin update config */
	cb->enable = false;
	cb->last_ts = 0;
	cb->last_freq_update_ts = 0;
	cb->acc = 0;
	cb->limit_level = v.limit_level;
	cb->limit_freq = cb->freqs[cb->limit_level];
	cb->limit_thres = MSEC_TO_NSEC(v.limit_thres_ms);
	cb->down_speed = v.down_speed;
	cb->down_limit_ns = MSEC_TO_NSEC(v.down_limit_ms);
	cb->up_speed = v.up_speed;
	cb->up_limit_ns = MSEC_TO_NSEC(v.up_limit_ms);
	cb->enable = v.enable;

	return 0;
out:
	pr_warn("config: invalid:%s\n", buf);
	return 0;
}

static int cb_config_show(char *buf, const struct kernel_param *kp)
{
	struct cpufreq_bouncing *cb;
	int i, j, cnt = 0;

	for (i = 0; i < min(NR_CLUS_MAX, cb_pol_idx); ++i) {
		cb = &cb_stuff[i];
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "=====\n");
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "clus %d (switch %d) first_cpu %d ctl %d\n", i, cb_switch, cb->first_cpu, cb->ctl_inited);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "last_ts: %llu\n", cb->last_ts);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "last_freq_update_ts: %llu\n", cb->last_freq_update_ts);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "acc: %llu\n", cb->acc);

		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "limit_freq: %d\n", cb->limit_freq);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "limit_level: %d\n", cb->limit_level);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "limit_thres: %llu ms\n", NSEC_TO_MSEC(cb->limit_thres));

		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "enable: %d\n", cb->enable);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "cur_level: %d\n", cb->cur_level);

		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "max_freq: %u %d\n", cb->max_freq, cb->max_level);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "down_speed: %d\n", cb->down_speed);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "down_limit_lat: %lld ms\n", NSEC_TO_MSEC(cb->down_limit_ns));

		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "min_freq: %u %d\n", cb->min_freq, cb->min_level);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "up_speed: %d\n", cb->up_speed);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "up_limit_lat: %lld ms\n", NSEC_TO_MSEC(cb->up_limit_ns));

		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "freq_sorting: %d\n", cb->freq_sorting);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "freq_levels: %d\n", cb->freq_levels);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "freq idx:");
		for (j = 0; j < cb->freq_levels; ++j)
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\t%u", j);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "freq clk:");
		for (j = 0; j < cb->freq_levels; ++j)
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\t%u", cb->freqs[j]);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "last_core_boost: %d\n", last_core_boost);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "last_core_boost_ts: %llu\n", last_core_boost_ts);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "freq resident:");
		for (j = 0; j < cb->freq_levels; ++j)
			cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\t%lld", cb->freqs_resident[j]);
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	}

	return cnt;
}

static struct kernel_param_ops cb_config_ops = {
	.set = cb_config_store,
	.get = cb_config_show,
};
module_param_cb(config, &cb_config_ops, NULL, 0664);

/* for info collection */
static int cb_trace_show(char *buf, const struct kernel_param *kp)
{
	struct cpufreq_bouncing *cb;
	int ret = 0;
	int i, j;

	for (i = 0; i < min(NR_CLUS_MAX, cb_pol_idx); ++i) {
		long long total = 0;
		long long freq_avg = 0;
		long long freq_max = 0;
		long long freq_min = UINT_MAX;
		long long freqs_resident[NR_FREQ] = {0};

		cb = &cb_stuff[i];

		for (j = cb->limit_level; j < NR_FREQ; ++j) {
			if (cb->freqs[j] == UINT_MAX)
				break;

			if (cb->freqs_resident[j] > 0) {
				total += NSEC_TO_MSEC(cb->freqs_resident[j]);
				freqs_resident[j] = NSEC_TO_MSEC(cb->freqs_resident[j]);
				cb->freqs_resident[j] = 0;
			}
		}

		for (j = cb->limit_level; j < NR_FREQ; ++j) {
			if (cb->freqs[j] == UINT_MAX)
				break;

			if (freqs_resident[j]) {
				freq_avg += cb->freqs[j] * freqs_resident[j] / total;
				freq_max = max_t(long long, freq_max, cb->freqs[j]);
				freq_min = min_t(long long, freq_min, cb->freqs[j]);
				cb->freqs_resident[j] = 0;
			}
		}

		/* output, format: clus_id, status, min, max, avg */
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d,%d,%lld,%lld,%lld,",
			i, cb->enable, freq_min, freq_max, freq_avg);
	}
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

	return ret;
}

static struct kernel_param_ops cb_trace_ops = {
	.get = cb_trace_show,
};
module_param_cb(trace, &cb_trace_ops, NULL, 0444);

void cb_ceiling_free_enable(bool rescue_enable)
{
	int i;

	if (!enable)
		return;

	if (is_sbe_rescue == rescue_enable)
		return;

	is_sbe_rescue = rescue_enable;

	for (i = 0; i < NR_CLUS_MAX; ++i) {
		struct cpufreq_bouncing *cb = &cb_stuff[i];
		cb = cb_get(cb->first_cpu);
		if (!cb)
			continue;

		if (rescue_enable)
			cb_reset_qos(i);
		else if (likely(cb_qos_kw))
			kthread_queue_work(cb_qos_kw, &cb->qos_work);
	}
}
EXPORT_SYMBOL(cb_ceiling_free_enable);

static inline bool clus_isolated(struct cpufreq_policy *pol)
{
	cpumask_t active;

	cpumask_and(&active, pol->related_cpus, cpu_active_mask);
	return !cpumask_weight(&active);
}

unsigned int cb_cap(struct cpufreq_policy *pol, unsigned int freq)
{
	struct cpufreq_bouncing *cb;
	unsigned int capped = freq;
	int cpu;

	if (!enable)
		return freq;

	/* can remove since we're calling from sugov_fast_switch path */
	if (!pol->fast_switch_enabled)
		return freq;

	if (freq_qos_check)
		return freq;

	cpu = cpumask_first(pol->related_cpus);
	cb = cb_get(cpu);

	if (unlikely(!cb))
		return freq;

	if (!cb->enable)
		return freq;

	capped = min(freq, cb->freqs[cb->cur_level]);

	if (debug)
		pr_info("cpu %d, orig %u, capped %d\n", cpu, freq, capped);

	return capped;
}

void cb_reset(int cpu, u64 time)
{
	struct cpufreq_bouncing *cb;

	if (!enable)
		return;

	cb = cb_get(cpu);
	if (!cb)
		return;

	/* reset only when cluster has no active cpu */
	if (!clus_isolated(cpufreq_cpu_get_raw(cpu)))
		return;

	cb->last_ts = time;
	cb->last_freq_update_ts = time;
	cb->acc = 0;
	cb->cur_level = cb->max_level;
}

void cb_update(struct cpufreq_policy *pol, u64 time)
{
	struct cpufreq_bouncing *cb;
	u64 delta, update_delta;
	int cpu, prev_level;
	bool min_over_target_freq, isolated;

	if (unlikely(!pol->fast_switch_enabled))
		return;

	if (!enable)
		return;

	cpu = cpumask_first(pol->related_cpus);
	cb = cb_get(cpu);

	if (unlikely(!cb))
		return;

	if (update_delay_check) {
		/* update per update window (8ms - 1ms margin by default) */
		if (time < cb->update_ts_ns + update_delay - NSEC_PER_MSEC)
			return;
		cb->update_ts_ns = time;
	}

	/* check isolated status */
	isolated = clus_isolated(pol);

	/* check current min_freq and limit target freq, if min_freq large than limited, reset acc */
	min_over_target_freq = pol->min >= cb->limit_freq;
	if (min_over_target_freq || isolated)
		cb->acc = 0;

	/* for first update */
	if (unlikely(!cb->last_ts))
		cb->last_ts = cb->last_freq_update_ts = time;

	/*
	 * not count flag only affects to delta.
	 * keep update_delta to let limit_freq has time to restore
	 */
	delta = ((min_over_target_freq || isolated) ? 0 : (time - cb->last_ts));
	update_delta = time - cb->last_freq_update_ts;

	/* check cpufreq */
	if (pol->cur >= cb->limit_freq) {
		/* accumulate delta time */
		cb->acc += delta;
	} else {
		/* decay accumulate time */
		cb->acc = cb->acc * decay / 100;
	}

	/* check if need to update limitation */
	prev_level = cb->cur_level;
	if (cb->acc >= cb->limit_thres) {
		/* check last update */
		if (update_delta >= cb->down_limit_ns) {
			if (cb->freq_sorting == CPUFREQ_TABLE_SORTED_DESCENDING)
				cb->cur_level = min(prev_level + cb->down_speed, cb->limit_level);
			if (cb->freq_sorting == CPUFREQ_TABLE_SORTED_ASCENDING)
				cb->cur_level = max(prev_level - cb->down_speed, cb->limit_level);

			cb->last_freq_update_ts = time;
			cb->freqs_resident[prev_level] += update_delta;
		}
	} else {
		/* check last update */
		if (update_delta >= cb->up_limit_ns) {
			if (cb->freq_sorting == CPUFREQ_TABLE_SORTED_DESCENDING)
				cb->cur_level = max(prev_level - cb->up_speed, cb->max_level);
			if (cb->freq_sorting == CPUFREQ_TABLE_SORTED_ASCENDING)
				cb->cur_level = min(prev_level + cb->up_speed, cb->max_level);

			cb->last_freq_update_ts = time;
			cb->freqs_resident[prev_level] += update_delta;
		}
	}

	/* when min bar is higher than cb limit, unlock immediately */
	if (min_over_target_freq || isolated)
		cb->cur_level = cb->max_level;

	/* update core_ctl boost status */
	/* cb_core_boost(time); */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	/* queue qos request */
	if (freq_qos_check &&
			cb->qos_req.pnode.prio != cb->freqs[cb->cur_level] &&
			update_delta >= max(cb->down_limit_ns, cb->up_limit_ns) &&
			likely(cb_qos_kw))
		kthread_queue_work(cb_qos_kw, &cb->qos_work);
#endif

	if (debug)
		pr_info("cpu %d update: ts now %llu last %llu last_update %llu delta %llu update_d %llu cur %u acc %llu cur_level %id"
			"min_over_target_freq %d isolated %d last_core_boost %d\n",
			cpu,
			NSEC_TO_MSEC(time),
			NSEC_TO_MSEC(cb->last_ts),
			NSEC_TO_MSEC(cb->last_freq_update_ts),
			NSEC_TO_MSEC(delta),
			NSEC_TO_MSEC(update_delta),
			pol->cur,
			NSEC_TO_MSEC(cb->acc),
			cb->cur_level,
			min_over_target_freq,
			isolated,
			last_core_boost);

	cb->last_ts = time;
}
EXPORT_SYMBOL(cb_update);

static int __cpufreq_policy_parser(int cpu, int cb_idx)
{
	struct cpufreq_policy *pol = cpufreq_cpu_get_raw(cpu);
	struct cpufreq_frequency_table *table, *pos;
	struct cpufreq_bouncing *cb;

	unsigned int freq = 0, max_freq = 0, min_freq = UINT_MAX;
	int idx, freq_levels = 0;

	if (unlikely(!pol)) {
		pr_err("cpu %d can't find realted cpufreq policy\n", cpu);
		return -EINVAL;
	}

	if (cb_idx >= NR_CLUS_MAX) {
		pr_err("clus %d out of limit\n", cb_idx);
		return -EINVAL;
	}

	cb = &cb_stuff[cb_idx];
	if (cb->first_cpu == -1)
		cb->first_cpu = cpu;

	/* get & setup freq_levels */
	table = pol->freq_table;
	cb->freq_sorting = pol->freq_table_sorted;
	cpufreq_for_each_valid_entry_idx(pos, table, idx) {
		++freq_levels;
		freq = pos->frequency;
		cb->freqs[idx] = freq;
		if (freq > max_freq) {
			max_freq = freq;
			cb->max_level = idx;
			cb->cur_level = idx;
			cb->max_freq = max_freq;
		}
		if (freq < min_freq) {
			min_freq = freq;
			cb->min_level = idx;
			cb->min_freq = min_freq;
		}
	}
	cb->freq_levels = freq_levels;
	return cpu + cpumask_weight(pol->related_cpus);
}

static void cb_parse_cpufreq(void)
{
	int i = 0, j, prev = 0;
	bool valid = true;

	while (i != NR_CORE_MAX && cb_pol_idx != NR_CLUS_MAX) {
		i = __cpufreq_policy_parser(i, cb_pol_idx);
		if (i < 0)
			break;
		for (j = prev; j < i; ++j)
			per_cpu(cbs, j) = &cb_stuff[cb_pol_idx];
		prev = i;
		++cb_pol_idx;
	}

	for (i = 0; i < NR_CORE_MAX && valid; ++i) {
		if (!per_cpu(cbs, i)) {
			pr_warn("break on cpu%d\n", i);
			valid = false;
		}
	}

	cb_switch = valid;
	smp_wmb();
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
static void cb_do_boundary_change_work(struct kthread_work *qos_work)
{
	struct cpufreq_bouncing *cb =
		container_of(qos_work, struct cpufreq_bouncing, qos_work);
	struct cpufreq_policy *pol = cpufreq_cpu_get_raw(cb->first_cpu);
	unsigned int target;

	if (!pol)
		return;

	target = max(cb->freqs[cb->cur_level], pol->min);

	if (debug)
		pr_info("processing work cpu %d min %u max %u target %u req max %u\n",
			cb->first_cpu, pol->min, pol->max, target, cb->qos_req.pnode.prio);

	if (is_sbe_rescue)
		target = FREQ_QOS_MAX_DEFAULT_VALUE;

#ifdef CONFIG_OPLUS_UAG_SOFT_LIMIT
	set_soft_limit_freq(pol, target);
#else
	if (freq_qos_update_request(&cb->qos_req, target) < 0)
		pr_err("failed to update freq constraint. cpu %d cb_limit %u\n", cb->first_cpu, target);
#endif /* CONFIG_OPLUS_UAG_SOFT_LIMIT */
}
#endif

static int cb_init_ctl(void)
{
	struct cpufreq_bouncing *cb;
	struct cpufreq_policy *pol;
	int cpu;

	for_each_possible_cpu(cpu) {
		pol = cpufreq_cpu_get_raw(cpu);
		if (unlikely(!pol))
			return -EFAULT;

		cb = per_cpu(cbs, cpu);
		if (unlikely(!cb))
			return -EFAULT;

		if (!cb->ctl_inited) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
			if (!cb_qos_kw) {
				cb_qos_kw = kthread_create_worker(0, "%s", "cb_qos_kw");
				if (!cb_qos_kw)
					return -EFAULT;
			}
			kthread_init_work(&cb->qos_work, cb_do_boundary_change_work);
#ifndef CONFIG_OPLUS_UAG_SOFT_LIMIT
			if (freq_qos_add_request(
					&pol->constraints, &cb->qos_req,
					FREQ_QOS_MAX,
					FREQ_QOS_MAX_DEFAULT_VALUE) < 0) {
				pr_err("add qos request failed. cpu %d\n", cb->first_cpu);
				return -EFAULT;
			}
#endif /* CONFIG_OPLUS_UAG_SOFT_LIMIT */
#endif
			cb->ctl_inited = true;
		}
	}
	return 0;
}

static int cb_main(void *arg)
{
	struct cpufreq_bouncing *cb;
	struct cpufreq_policy *pol;
	int i;

	while (!kthread_should_stop()) {
		set_current_state(TASK_RUNNING);
		if (!enable || !self_activate) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
		}

		for (i = 0; i < min(NR_CLUS_MAX, cb_pol_idx); ++i) {
			cb = &cb_stuff[i];
			pol = cpufreq_cpu_get(cb->first_cpu);
			if (pol) {
				cb_update(pol, ktime_to_ns(ktime_get()));
				cpufreq_cpu_put(pol);
			}
		}

		if (self_activate) {
			usleep_range(MSEC_TO_USEC(sleep_range_ms[0]), MSEC_TO_USEC(sleep_range_ms[1])); /* min ~ max */
		}
	}

	return 0;
}

static void cb_init_arch(void)
{
#ifdef CONFIG_ARCH_MEDIATEK
	struct cpufreq_bouncing *cb;

	/* customized here */
	/* clus 1 */
	cb = &cb_stuff[1];
	cb->limit_freq = 2100000;
	cb->limit_level = 4;

	/* clus 2 */
	cb = &cb_stuff[2];
	cb->limit_freq = 2300000;
	cb->limit_level = 8;

	update_delay_check = true;
#endif
	pr_info("cb init arch\n");
}

static int __init cb_init(void)
{
	int cpu;
	int num = 0;
	struct cpufreq_policy *policy;

	cb_init_arch();

	pr_info("cb init\n");

	cpu_req = kcalloc(NR_CLUS_MAX,
		sizeof(struct freq_qos_request), GFP_KERNEL);

	if(!cpu_req) {
		pr_err("Failed to allocate memory for cpu_req\n");
		return -ENOMEM;
	}

	for_each_possible_cpu(cpu) {
		if (num >= NR_CLUS_MAX)
			break;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
		freq_qos_add_request(&policy->constraints,
			&(cpu_req[num]), FREQ_QOS_MIN, 500000);
		freq_qos_update_request(&(cpu_req[num]), 500000);
#endif
		num++;
		cpu = cpumask_last(policy->related_cpus);
		cpufreq_cpu_put(policy);
	}

	cb_parse_cpufreq();
	if (cb_init_ctl()) {
		pr_warn("cb init ctl failed\n");
		if (cb_qos_kw) {
			kthread_destroy_worker(cb_qos_kw);
			cb_qos_kw = NULL;
		}
		for_each_possible_cpu(cpu) {
			if (per_cpu(cbs, cpu))
				per_cpu(cbs, cpu)->ctl_inited = false;
		}
		cb_switch = false;
		pr_warn("cb finish init with disabled by default\n");
		return 0;
	}

	cb_task = kthread_run(cb_main, NULL, "cb_task");
	pr_info("cb inited\n");
	return 0;
}

static void __exit cb_exit(void)
{
	kfree(cpu_req);
	pr_info("cpufreq bouncing exit\n");
}
module_init(cb_init);
module_exit(cb_exit);
MODULE_DESCRIPTION("OPLUS CPUFREQ BOUNCING Driver");
MODULE_LICENSE("GPL v2");
