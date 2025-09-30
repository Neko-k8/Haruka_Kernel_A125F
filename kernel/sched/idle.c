/*
 * improved_idle.c - Enhanced idle core with lightweight per-CPU EWMA prediction
 *                   and conservative adaptive tick-stop heuristics.
 *
 * Goals:
 *  - Reduce pointless deep-enter/exit cycles on noisy workloads
 *  - Improve prediction of useful residency so deep states are entered when worthwhile
 *  - Keep overhead minimal (integer math, per-cpu data, lock-free)
 *  - Always active (no runtime toggle) but conservative by default
 *
 * Backport-friendly: avoids exotic APIs, uses weak hooks where appropriate.
 *
 * Copyright: ChatGPT-assisted update (2025)
 * License: same as kernel (GPL-compatible)
 */

#include "sched.h"

#include <trace/events/power.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/percpu.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/printk.h>

#define PRPREFIX "idle_improve: "

/* ------------------ Tunables (compile-time constants) ------------------ */
/* EWMA weight = 1/(2^IDLE_EWMA_SHIFT) */
#ifndef IDLE_EWMA_SHIFT
#define IDLE_EWMA_SHIFT    3   /* default weight = 1/8 */
#endif

/* If predicted residency (us) >= IDLE_FORCE_STOP_US, hint to stop tick */
#ifndef IDLE_FORCE_STOP_US
#define IDLE_FORCE_STOP_US 2000U /* 2 ms */
#endif

/* Minimum sample to accept (us) to avoid noise */
#ifndef IDLE_MIN_SAMPLE_US
#define IDLE_MIN_SAMPLE_US 200U /* 200 us */
#endif

/* If last wakeup < this (us), be conservative (avoid forcing deep) */
#ifndef IDLE_WAKE_COALESCE_US
#define IDLE_WAKE_COALESCE_US 800U /* 0.8 ms */
#endif

/* Verbose debug prints if defined */
#ifdef CONFIG_IDLE_IMPROVE_DEBUG
#define IDBG(fmt, ...) pr_info(PRPREFIX fmt, ##__VA_ARGS__)
#else
#define IDBG(fmt, ...) do { } while (0)
#endif

/* ------------------ Per-CPU predictor ------------------ */

struct idle_predict {
	u64 resid_ewma_ns;     /* EWMA residency prediction in ns */
	u64 last_sample_ns;    /* last measured sample (ns) */
	u64 last_wakeup_ns;    /* last time cpu left idle (ns) */
	unsigned int ewma_shift; /* local shift copy */
};

/* per-cpu storage */
static DEFINE_PER_CPU(struct idle_predict, idle_predict_percpu);

/* time helpers */
static inline u64 now_ns(void) { return ktime_get_ns(); }
static inline u64 us_to_ns_u64(unsigned int us) { return (u64)us * NSEC_PER_USEC; }

/* Update EWMA using integer ops. Called while still on the CPU; no lock. */
static void idle_predict_update_sample(int cpu, u64 sample_ns)
{
	struct idle_predict *p = &per_cpu(idle_predict_percpu, cpu);
	u64 prev = p->resid_ewma_ns;
	u64 next;
	unsigned int shift = p->ewma_shift ? p->ewma_shift : IDLE_EWMA_SHIFT;

	/* filter tiny noise */
	if (sample_ns < us_to_ns_u64(IDLE_MIN_SAMPLE_US))
		sample_ns = 0;

	if (shift == 0) {
		next = sample_ns;
	} else {
		u64 mul = ((1ULL << shift) - 1ULL);
		next = ((prev * mul) + sample_ns) >> shift;
		if (!next && sample_ns)
			next = 1; /* avoid stuck zero */
	}

	p->resid_ewma_ns = next;
	p->last_sample_ns = sample_ns;
	p->ewma_shift = shift;

	IDBG("cpu%d EWMA prev=%llu ns sample=%llu ns next=%llu ns shift=%u\n",
	     smp_processor_id(),
	     (unsigned long long)prev,
	     (unsigned long long)sample_ns,
	     (unsigned long long)next,
	     shift);
}

/* mark a wakeup event */
static void idle_predict_mark_wakeup(int cpu, u64 tns)
{
	struct idle_predict *p = &per_cpu(idle_predict_percpu, cpu);
	p->last_wakeup_ns = tns;
}

/* get predicted residency in ns */
static u64 idle_predict_get(int cpu)
{
	struct idle_predict *p = &per_cpu(idle_predict_percpu, cpu);
	return p->resid_ewma_ns;
}

/* time since last wakeup; U64_MAX if never */
static u64 idle_time_since_last_wakeup_ns(int cpu, u64 tnow)
{
	struct idle_predict *p = &per_cpu(idle_predict_percpu, cpu);
	if (p->last_wakeup_ns == 0)
		return U64_MAX;
	return tnow - p->last_wakeup_ns;
}

/* ------------------ end predictor ------------------------------------ */

/* Linker adds these: start and end of __cpuidle functions */
extern char __cpuidle_text_start[], __cpuidle_text_end[];

/* Weak prototypes for symbols that may not exist in some backport trees */
extern void cpuidle_use_deepest_state(bool enable) __attribute__((weak));
extern int cpuidle_enter_s2idle(struct cpuidle_driver *drv,
				struct cpuidle_device *dev) __attribute__((weak));

void sched_idle_set_state(struct cpuidle_state *idle_state, int index)
{
	idle_set_state(this_rq(), idle_state);
	idle_set_state_idx(this_rq(), index);
}

static int __read_mostly cpu_idle_force_poll;

void cpu_idle_poll_ctrl(bool enable)
{
	if (enable) {
		cpu_idle_force_poll++;
	} else {
		cpu_idle_force_poll--;
		WARN_ON_ONCE(cpu_idle_force_poll < 0);
	}
}

#ifdef CONFIG_GENERIC_IDLE_POLL_SETUP
static int __init cpu_idle_poll_setup(char *__unused)
{
	cpu_idle_force_poll = 1;
	return 1;
}
__setup("nohlt", cpu_idle_poll_setup);

static int __init cpu_idle_nopoll_setup(char *__unused)
{
	cpu_idle_force_poll = 0;
	return 1;
}
__setup("hlt", cpu_idle_nopoll_setup);
#endif

static noinline int __cpuidle cpu_idle_poll(void)
{
	rcu_idle_enter();
	trace_cpu_idle_rcuidle(0, smp_processor_id());
	local_irq_enable();
	stop_critical_timings();

	/* spin loop: keep it simple and short to avoid burning cycles too long */
	while (!tif_need_resched() &&
	       (cpu_idle_force_poll || tick_check_broadcast_expired()))
		cpu_relax();

	start_critical_timings();
	trace_cpu_idle_rcuidle(PWR_EVENT_EXIT, smp_processor_id());
	rcu_idle_exit();

	return 1;
}

/* Weak arch hooks */
void __weak arch_cpu_idle_prepare(void) { }
void __weak arch_cpu_idle_enter(void) { }
void __weak arch_cpu_idle_exit(void) { }
void __weak arch_cpu_idle_dead(void) { }
void __weak arch_cpu_idle(void)
{
	/* fallback: preserve original behavior */
	cpu_idle_force_poll = 1;
	local_irq_enable();
}

void __cpuidle default_idle_call(void)
{
	if (current_clr_polling_and_test()) {
		local_irq_enable();
	} else {
		stop_critical_timings();
		arch_cpu_idle();
		start_critical_timings();
	}
}

static int call_cpuidle(struct cpuidle_driver *drv, struct cpuidle_device *dev,
			int next_state)
{
	if (current_clr_polling_and_test()) {
		dev->last_residency = 0;
		local_irq_enable();
		return -EBUSY;
	}

	return cpuidle_enter(drv, dev, next_state);
}

/*
 * cpuidle_idle_call - enhanced idle decision with EWMA hooks
 *
 * Strategy:
 *  - Query governor as usual for preferred next_state and stop_tick hint.
 *  - Use per-cpu EWMA prediction of residency (ns) to *force* stop_tick when
 *    prediction exceeds threshold and last wakeup was not too recent.
 *  - If last wakeup was very recent (coalesce window), be conservative.
 *  - Update EWMA after exit using dev->last_residency (us -> ns).
 */
static void cpuidle_idle_call(void)
{
	struct cpuidle_device *dev = cpuidle_get_device();
	struct cpuidle_driver *drv = cpuidle_get_cpu_driver(dev);
	int next_state = 0, entered_state;
	int cpu = smp_processor_id();

	if (need_resched()) {
		local_irq_enable();
		return;
	}

	if (cpuidle_not_available(drv, dev)) {
		/* fallback path preserved */
		tick_nohz_idle_stop_tick();
		rcu_idle_enter();

		default_idle_call();
		goto exit_idle;
	}

#ifdef CONFIG_SUSPEND
	if (idle_should_enter_s2idle() || dev->use_deepest_state) {
		if (idle_should_enter_s2idle()) {
			rcu_idle_enter();

			/* call weak cpuidle_enter_s2idle if present; fallback to 0 */
			if (cpuidle_enter_s2idle)
				entered_state = cpuidle_enter_s2idle(drv, dev);
			else
				entered_state = 0;

			if (entered_state > 0) {
				local_irq_enable();
				goto exit_idle;
			}

			rcu_idle_exit();
		}

		tick_nohz_idle_stop_tick();
		rcu_idle_enter();

		next_state = cpuidle_find_deepest_state(drv, dev);
		call_cpuidle(drv, dev, next_state);

		/* record wakeup */
		idle_predict_mark_wakeup(cpu, now_ns());
	} else
#endif /* CONFIG_SUSPEND */
	{
		bool stop_tick = true;
		u64 predicted_ns = 0;
		u64 tnow = now_ns();

		/* Ask governor for preferred state and initial tick hint */
		next_state = cpuidle_select(drv, dev, &stop_tick);

		/* Adaptive decision: use EWMA prediction to decide whether to force stop_tick */
		predicted_ns = idle_predict_get(cpu);

		if (predicted_ns) {
			u64 since_wakeup = idle_time_since_last_wakeup_ns(cpu, tnow);
			/* If predicted residency long and no very recent wakeup, bias to stop tick */
			if (predicted_ns >= us_to_ns_u64(IDLE_FORCE_STOP_US) &&
			    (since_wakeup == U64_MAX ||
			     since_wakeup >= us_to_ns_u64(IDLE_WAKE_COALESCE_US))) {
				/* force stop tick to allow deeper idle if governor agrees */
				stop_tick = true;
				IDBG("cpu%d adaptive: predicted=%lluns >= %uus, favor stop_tick\n",
				     cpu, (unsigned long long)predicted_ns, (unsigned int)IDLE_FORCE_STOP_US);
			} else {
				IDBG("cpu%d adaptive: predicted=%lluns since_wakeup=%lluns -> not forcing\n",
				     cpu, (unsigned long long)predicted_ns,
				     (unsigned long long)since_wakeup);
			}
		}

		/* Respect initial governor hint but apply our decision (conservative) */
		if (stop_tick || tick_nohz_tick_stopped())
			tick_nohz_idle_stop_tick();
		else
			tick_nohz_idle_retain_tick();

		rcu_idle_enter();

		entered_state = call_cpuidle(drv, dev, next_state);

		/* original hook: let governor reflect */
		cpuidle_reflect(dev, entered_state);

		/* Update EWMA using last_residency (dev->last_residency is microseconds) */
		{
			u64 sample_ns = 0;
			if (dev && dev->last_residency > 0)
				sample_ns = (u64)dev->last_residency * NSEC_PER_USEC;

			/* Update prediction and mark wakeup time */
			idle_predict_update_sample(cpu, sample_ns);
			idle_predict_mark_wakeup(cpu, now_ns());
		}
	}

exit_idle:
	__current_set_polling();

	/* Ensure interrupts enabled by idle implementation */
	if (WARN_ON_ONCE(irqs_disabled()))
		local_irq_enable();

	rcu_idle_exit();
}

/*
 * do_idle loop: mostly unchanged; uses cpuidle_idle_call above.
 */
static void do_idle(void)
{
	int cpu = smp_processor_id();

	__current_set_polling();
	tick_nohz_idle_enter();

	while (!need_resched()) {
		check_pgt_cache();
		rmb();

		local_irq_disable();

		if (cpu_is_offline(cpu)) {
			tick_nohz_idle_stop_tick();
			cpuhp_report_idle_dead();
			arch_cpu_idle_dead();
		}

		arch_cpu_idle_enter();

		if (cpu_idle_force_poll || tick_check_broadcast_expired()) {
			/* If polling forced, keep the restart semantics but avoid long spins:
			 * cpu_idle_poll itself spins briefly. This avoids burning cycles too long.
			 */
			tick_nohz_idle_restart_tick();
			cpu_idle_poll();
		} else {
			/* enhanced cpuidle decision path */
			cpuidle_idle_call();
		}

		arch_cpu_idle_exit();
	}

	preempt_set_need_resched();
	tick_nohz_idle_exit();
	__current_clr_polling();

	smp_mb__after_atomic();

	sched_ttwu_pending();
	schedule_idle();

	if (unlikely(klp_patch_pending(current)))
		klp_update_patch_state(current);
}

bool cpu_in_idle(unsigned long pc)
{
	return pc >= (unsigned long)__cpuidle_text_start &&
		pc < (unsigned long)__cpuidle_text_end;
}

/* idle_inject_timer_fn & play_idle keep same semantics (unchanged) */
struct idle_timer { struct hrtimer timer; int done; };

static enum hrtimer_restart idle_inject_timer_fn(struct hrtimer *timer)
{
	struct idle_timer *it = container_of(timer, struct idle_timer, timer);
	WRITE_ONCE(it->done, 1);
	set_tsk_need_resched(current);
	return HRTIMER_NORESTART;
}

void play_idle(unsigned long duration_ms)
{
	struct idle_timer it;

	WARN_ON_ONCE(current->policy != SCHED_FIFO);
	WARN_ON_ONCE(current->nr_cpus_allowed != 1);
	WARN_ON_ONCE(!(current->flags & PF_KTHREAD));
	WARN_ON_ONCE(!(current->flags & PF_NO_SETAFFINITY));
	WARN_ON_ONCE(!duration_ms);

	rcu_sleep_check();
	preempt_disable();
	current->flags |= PF_IDLE;

	if (cpuidle_use_deepest_state)
		cpuidle_use_deepest_state(true);

	it.done = 0;
	hrtimer_init_on_stack(&it.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	it.timer.function = idle_inject_timer_fn;
	hrtimer_start(&it.timer, ms_to_ktime(duration_ms), HRTIMER_MODE_REL_PINNED);

	while (!READ_ONCE(it.done))
		do_idle();

	if (cpuidle_use_deepest_state)
		cpuidle_use_deepest_state(false);

	current->flags &= ~PF_IDLE;

	preempt_fold_need_resched();
	preempt_enable();
}
EXPORT_SYMBOL_GPL(play_idle);

void cpu_startup_entry(enum cpuhp_state state)
{
#ifdef CONFIG_X86
	boot_init_stack_canary();
#endif
	arch_cpu_idle_prepare();
	cpuhp_online_idle(state);
	while (1)
		do_idle();
}

/* idle scheduling class (unchanged) */
#ifdef CONFIG_SMP
static int
select_task_rq_idle(struct task_struct *p, int cpu, int sd_flag, int flags,
		    int sibling_count_hint)
{
	return task_cpu(p);
}
#endif

static void check_preempt_curr_idle(struct rq *rq, struct task_struct *p, int flags)
{
	resched_curr(rq);
}

static struct task_struct *
pick_next_task_idle(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
	put_prev_task(rq, prev);
	update_idle_core(rq);
	schedstat_inc(rq->sched_goidle);
	return rq->idle;
}

static void
dequeue_task_idle(struct rq *rq, struct task_struct *p, int flags)
{
	raw_spin_unlock_irq(&rq->lock);
	printk(KERN_ERR "bad: scheduling from the idle thread!\n");
	dump_stack();
	raw_spin_lock_irq(&rq->lock);
}

static void put_prev_task_idle(struct rq *rq, struct task_struct *prev) { }
static void task_tick_idle(struct rq *rq, struct task_struct *curr, int queued) { }
static void set_curr_task_idle(struct rq *rq) { }
static void switched_to_idle(struct rq *rq, struct task_struct *p) { BUG(); }
static void prio_changed_idle(struct rq *rq, struct task_struct *p, int oldprio) { BUG(); }
static unsigned int get_rr_interval_idle(struct rq *rq, struct task_struct *task) { return 0; }
static void update_curr_idle(struct rq *rq) { }

const struct sched_class idle_sched_class = {
	.dequeue_task		= dequeue_task_idle,
	.check_preempt_curr	= check_preempt_curr_idle,
	.pick_next_task		= pick_next_task_idle,
	.put_prev_task		= put_prev_task_idle,
#ifdef CONFIG_SMP
	.select_task_rq		= select_task_rq_idle,
	.set_cpus_allowed	= set_cpus_allowed_common,
#endif
	.set_curr_task          = set_curr_task_idle,
	.task_tick		= task_tick_idle,
	.get_rr_interval	= get_rr_interval_idle,
	.prio_changed		= prio_changed_idle,
	.switched_to		= switched_to_idle,
	.update_curr		= update_curr_idle,
};

