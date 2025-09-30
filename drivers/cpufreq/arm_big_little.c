/*
 * drivers/cpufreq/arm_big_little.c
 *
 * Smart arm big.LITTLE cluster balancer (4.19 backport-friendly)
 *
 * - Periodic sampling (abl_sample_ms)
 * - Per-cluster EWMA utilization prediction
 * - Hysteresis counters to avoid oscillation
 * - Actuation: prefer BIG cluster (max freq) or LITTLE cluster (min freq)
 *
 * Copyright (C) ChatGPT-assisted (2025)
 * License: GPL v2
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <trace/events/power.h>
#include <linux/sched.h> /* cpu_util_freq, SCHED_CAPACITY_SHIFT */
#include "cpufreq.h"    /* internal header if present - but <linux/cpufreq.h> is main */

#define ABL_PREFIX "arm_big_little: "
#define ABL_INFO(fmt, ...) pr_info(ABL_PREFIX fmt, ##__VA_ARGS__)
#define ABL_DBG(fmt, ...)  pr_debug(ABL_PREFIX fmt, ##__VA_ARGS__)

MODULE_DESCRIPTION("Smart arm big.LITTLE balancer (EWMA + hysteresis) - in-tree");
MODULE_AUTHOR("ChatGPT-assisted");
MODULE_LICENSE("GPL");

static unsigned int abl_sample_ms = 100;      /* sampling interval (ms) */
static unsigned int abl_ewma_shift = 3;       /* EWMA weight = 1/(2^shift) */
static unsigned int abl_up_thresh = 70;       /* percent to prefer BIG */
static unsigned int abl_down_thresh = 30;     /* percent to prefer LITTLE */
static unsigned int abl_hysteresis = 3;       /* consecutive windows to flip */
static bool abl_enabled = true;

module_param(abl_sample_ms, uint, 0644);
MODULE_PARM_DESC(abl_sample_ms, "arm_big_little sample interval in ms");
module_param(abl_ewma_shift, uint, 0644);
MODULE_PARM_DESC(abl_ewma_shift, "EWMA shift (1/(2^shift) weight)");
module_param(abl_up_thresh, uint, 0644);
MODULE_PARM_DESC(abl_up_thresh, "Percent threshold to prefer BIG cluster");
module_param(abl_down_thresh, uint, 0644);
MODULE_PARM_DESC(abl_down_thresh, "Percent threshold to prefer LITTLE cluster");
module_param(abl_hysteresis, uint, 0644);
MODULE_PARM_DESC(abl_hysteresis, "Consecutive windows required to change preference");
module_param(abl_enabled, bool, 0644);
MODULE_PARM_DESC(abl_enabled, "Enable arm_big_little balancer");

/* Per-cluster structure */
struct abl_cluster {
	int cluster_id;               /* arch cluster id */
	cpumask_t cpus;               /* cpus in cluster */
	unsigned long ewma_pct;       /* EWMA in percent (0..100) */
	unsigned int above_cnt;       /* consecutive windows above up_thresh */
	unsigned int below_cnt;       /* consecutive windows below down_thresh */
	bool prefer_big;              /* current preference: true->BIG */
	struct mutex act_lock;        /* protect actuation */
};

/* Global state */
struct abl_state {
	struct workqueue_struct *wq;
	struct delayed_work work;
	struct abl_cluster *clusters;
	int nclusters;
	bool inited;
};

static struct abl_state *g;

/* Forward */
static void abl_do_work(struct work_struct *work);

/* Utility: sample per-cpu freq-util percent (0..100)
 * Uses cpu_util_freq() (capacity scale) -> convert to percent.
 * Returns 0 when information missing.
 */
static unsigned int sample_cpu_pct(int cpu)
{
#ifdef CONFIG_SCHED_UTIL
	unsigned long util = cpu_util_freq(cpu); /* capacity scaled value */
	unsigned long cap = arch_scale_cpu_capacity(NULL, cpu);
	if (!cap)
		return 0;
	/* convert to percent: util / cap * 100
	 * util and cap are in SCHED_CAPACITY_SCALE (1<<SCHED_CAPACITY_SHIFT)
	 */
	return (unsigned int)((util * 100UL) >> SCHED_CAPACITY_SHIFT);
#else
	/* If scheduler util helpers not available, return 0 */
	return 0;
#endif
}

/* Compute cluster utilization percent (we use max per-cpu percent as conservative) */
static unsigned int cluster_sample_pct(struct abl_cluster *c)
{
	unsigned int max = 0;
	int cpu;
	for_each_cpu(cpu, &c->cpus) {
		unsigned int pct = sample_cpu_pct(cpu);
		if (pct > max)
			max = pct;
	}
	return max;
}

/* EWMA update: ewma' = (ewma*(2^shift - 1) + sample) / 2^shift */
static unsigned long ewma_update(unsigned long prev, unsigned int sample)
{
	if (abl_ewma_shift == 0)
		return sample;
	return ((prev * ((1UL << abl_ewma_shift) - 1)) + sample) >> abl_ewma_shift;
}

/* Actuate cluster policy: when prefer_big true -> target max freq; else target min freq.
 * For safety we call cpufreq_cpu_get/put and __cpufreq_driver_target.
 * We attempt best-effort: if no policy, skip.
 */
static void cluster_actuate(struct abl_cluster *c)
{
	int rep_cpu;
	struct cpufreq_policy *policy;
	unsigned int target;

	/* pick a representative cpu (first in mask) */
	rep_cpu = cpumask_first(&c->cpus);
	if (rep_cpu >= nr_cpu_ids)
		return;

	/* get policy for that CPU */
	policy = cpufreq_cpu_get(rep_cpu);
	if (!policy)
		return;

	/* choose target freq */
	if (c->prefer_big) {
		/* prefer high performance */
		target = policy->cpuinfo.max_freq;
	} else {
		/* prefer energy saving */
		target = policy->cpuinfo.min_freq;
	}

	/* protect concurrent actuations */
	if (!mutex_trylock(&c->act_lock)) {
		/* another thread acting; skip */
		cpufreq_cpu_put(policy);
		return;
	}

	/* If the policy already at target (cur may be approximate), skip attempt */
	if (policy->cur != target) {
		/* Use internal driver call - in-tree this is allowed */
#ifdef __cpufreq_driver_target
		/* __cpufreq_driver_target may be available in-tree; prefer it */
		__cpufreq_driver_target(policy, target, CPUFREQ_RELATION_L);
#else
		/* Fall-back to generic exported API if available */
		cpufreq_driver_target(policy, target, CPUFREQ_RELATION_L);
#endif
		/* For traceability */
		trace_cpu_frequency(target, rep_cpu);
		ABL_INFO("cluster %d: actuated to %u (%s)\n",
			 c->cluster_id, target, c->prefer_big ? "BIG" : "LITTLE");
	}

	mutex_unlock(&c->act_lock);
	cpufreq_cpu_put(policy);
}

/* Build clusters by scanning CPUs and grouping by arch_cpu_cluster_id.
 * If arch_cpu_cluster_id is unavailable, fallback: group by online cpumasks
 * into two clusters (heuristic), but most ARM platforms provide it.
 */
static int build_clusters(struct abl_state *st)
{
	int cpu;
	int ids[nr_cpu_ids];
	int nids = 0;
	int i, j;

	memset(ids, -1, sizeof(ids));

	/* collect unique cluster ids */
	for_each_possible_cpu(cpu) {
#ifdef arch_cpu_cluster_id
		int cid = arch_cpu_cluster_id(cpu);
#else
		/* Fallback heuristic: cpu/ (nr_cpu_ids/2) -> two clusters */
		int half = max(1, nr_cpu_ids / 2);
		int cid = cpu / half;
#endif
		/* see if seen */
		for (i = 0; i < nids; i++)
			if (ids[i] == cid)
				break;
		if (i == nids)
			ids[nids++] = cid;
	}

	/* allocate clusters */
	st->clusters = kcalloc(nids, sizeof(*st->clusters), GFP_KERNEL);
	if (!st->clusters)
		return -ENOMEM;
	st->nclusters = nids;

	/* populate cluster masks */
	for (i = 0; i < nids; i++)
		cpumask_clear(&st->clusters[i].cpus);

	for_each_possible_cpu(cpu) {
#ifdef arch_cpu_cluster_id
		int cid = arch_cpu_cluster_id(cpu);
#else
		int half = max(1, nr_cpu_ids / 2);
		int cid = cpu / half;
#endif
		/* find index */
		for (j = 0; j < nids; j++) {
			if (ids[j] == cid) {
				cpumask_set_cpu(cpu, &st->clusters[j].cpus);
				st->clusters[j].cluster_id = cid;
				break;
			}
		}
	}

	/* init cluster fields */
	for (i = 0; i < st->nclusters; i++) {
		struct abl_cluster *c = &st->clusters[i];
		c->ewma_pct = 0;
		c->above_cnt = c->below_cnt = 0;
		c->prefer_big = false; /* default prefer LITTLE for power saving */
		mutex_init(&c->act_lock);
	}

	ABL_INFO("arm_big_little: detected %d cluster(s)\n", st->nclusters);
	return 0;
}

/* Work handler: sample, update EWMA, decide, actuate if needed */
static void abl_do_work(struct work_struct *work)
{
	int i;
	unsigned int sample;
	struct abl_state *st = g;

	if (!abl_enabled)
		goto schedule_next;

	if (!st || !st->inited)
		goto schedule_next;

	for (i = 0; i < st->nclusters; i++) {
		struct abl_cluster *c = &st->clusters[i];

		/* sample cluster percent */
		sample = cluster_sample_pct(c);

		/* update EWMA */
		c->ewma_pct = ewma_update(c->ewma_pct, sample);

		ABL_DBG("cluster %d: sample=%u ewma=%lu\n", c->cluster_id, sample, c->ewma_pct);

		/* hysteresis logic */
		if (c->ewma_pct >= abl_up_thresh) {
			c->above_cnt++;
			c->below_cnt = 0;
		} else if (c->ewma_pct <= abl_down_thresh) {
			c->below_cnt++;
			c->above_cnt = 0;
		} else {
			/* in between thresholds -> decay counters */
			if (c->above_cnt)
				c->above_cnt = max(0U, c->above_cnt - 1);
			if (c->below_cnt)
				c->below_cnt = max(0U, c->below_cnt - 1);
		}

		/* decide flip */
		if (!c->prefer_big && c->above_cnt >= abl_hysteresis) {
			c->prefer_big = true;
			c->above_cnt = 0;
			c->below_cnt = 0;
			ABL_INFO("cluster %d: switching preference -> BIG\n", c->cluster_id);
			cluster_actuate(c);
		} else if (c->prefer_big && c->below_cnt >= abl_hysteresis) {
			c->prefer_big = false;
			c->above_cnt = 0;
			c->below_cnt = 0;
			ABL_INFO("cluster %d: switching preference -> LITTLE\n", c->cluster_id);
			cluster_actuate(c);
		}
	}

schedule_next:
	/* schedule next */
	if (abl_enabled && st && st->wq) {
		unsigned long delay = msecs_to_jiffies(max(20U, abl_sample_ms));
		queue_delayed_work(st->wq, &st->work, delay);
	}
}

/* Start/stop helpers */
static int abl_start(struct abl_state *st)
{
	int ret;

	if (!st || st->inited)
		return -EINVAL;

	st->wq = alloc_workqueue("arm_bl_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 0);
	if (!st->wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&st->work, abl_do_work);

	/* build clusters */
	ret = build_clusters(st);
	if (ret) {
		destroy_workqueue(st->wq);
		st->wq = NULL;
		return ret;
	}

	st->inited = true;

	/* kick off */
	queue_delayed_work(st->wq, &st->work, msecs_to_jiffies(max(20U, abl_sample_ms)));
	ABL_INFO("arm_big_little: started (sample_ms=%u, ewma_shift=%u)\n", abl_sample_ms, abl_ewma_shift);
	return 0;
}

static void abl_stop(struct abl_state *st)
{
	if (!st || !st->inited)
		return;

	cancel_delayed_work_sync(&st->work);

	if (st->clusters)
		kfree(st->clusters);
	st->clusters = NULL;
	st->nclusters = 0;

	if (st->wq) {
		destroy_workqueue(st->wq);
		st->wq = NULL;
	}

	st->inited = false;
	ABL_INFO("arm_big_little: stopped\n");
}

static int __init arm_bl_init(void)
{
	int ret;
	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;

	ret = abl_start(g);
	if (ret) {
		kfree(g);
		g = NULL;
		return ret;
	}

	return 0;
}

static void __exit arm_bl_exit(void)
{
	if (!g)
		return;
	abl_stop(g);
	kfree(g);
	g = NULL;
}

module_init(arm_bl_init);
module_exit(arm_bl_exit);

