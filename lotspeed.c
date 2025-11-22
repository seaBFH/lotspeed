/*
 * lotspeed_zeta_v4.c
 * "公路超跑" Zeta-TCP 深度复刻版
 * Author: uk0 (Modified by Gemini)
 * * Features:
 * 1. Zeta-Like Learning Mode (历史带宽记忆，实现 0-RTT 满速启动)
 * 2. RTT-based Loss Immunity (基于延迟梯度的丢包免疫)
 * 3. BBR-style Pacing & Probing
 * 4. Turbo Mode (可选的暴力模式)
 */

#include <linux/module.h>
#include <linux/version.h>
#include <net/tcp.h>
#include <linux/math64.h>
#include <linux/moduleparam.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/rtc.h>
#include <linux/hashtable.h> // 新增：哈希表支持
#include <linux/slab.h>      // 新增：内存分配
#include <linux/spinlock.h>  // 新增：锁机制

// --- 基础宏定义 ---
#define CURRENT_TIMESTAMP ({ \
    static char __ts[32]; \
    struct timespec64 ts; \
    struct tm tm; \
    ktime_get_real_ts64(&ts); \
    time64_to_tm(ts.tv_sec, 0, &tm); \
    snprintf(__ts, sizeof(__ts), "%04ld-%02d-%02d %02d:%02d:%02d", \
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
            tm.tm_hour, tm.tm_min, tm.tm_sec); \
    __ts; \
})

// --- Zeta-TCP 核心配置 ---
#define HISTORY_BITS 10          // 哈希表大小 2^10 = 1024 桶
#define HISTORY_TTL_SEC 3600     // 记忆有效期 1小时
#define LOSS_IMMUNITY_RATIO 125  // RTT 容忍度 1.25倍 (125%)

// --- BBR/LotSpeed 参数 ---
#define LOTSPEED_BETA_SCALE 1024
#define LOTSPEED_PROBE_RTT_INTERVAL_MS 10000
#define LOTSPEED_PROBE_RTT_DURATION_MS 500
#define LOTSPEED_STARTUP_GROWTH_TARGET 1280
#define LOTSPEED_STARTUP_EXIT_ROUNDS 2

// API 兼容性
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
#define LOTSPEED_NEW_CONG_CONTROL_API 1
#else
#define LOTSPEED_OLD_CONG_CONTROL_API 1
#endif

// --- 模块参数 ---
static unsigned long lotserver_rate = 125000000ULL;  // 1Gbps
static unsigned int lotserver_gain = 20;             // 2.0x
static unsigned int lotserver_min_cwnd = 16;
static unsigned int lotserver_max_cwnd = 20000;      // 调大默认最大窗口
static unsigned int lotserver_beta = 717;
static bool lotserver_adaptive = true;
static bool lotserver_turbo = false;
static bool lotserver_verbose = false;
static bool force_unload = false;

// --- 参数回调 (保留v2.1的详细日志格式) ---
static int param_set_rate(const char *val, const struct kernel_param *kp)
{
    unsigned long old_val = lotserver_rate;
    int ret = param_set_ulong(val, kp);

    if (ret == 0 && old_val != lotserver_rate && lotserver_verbose) {
        unsigned long gbps_int = lotserver_rate / 125000000;
        unsigned long gbps_frac = (lotserver_rate % 125000000) * 100 / 125000000;
        pr_info("lotspeed: [uk0@%s] rate changed: %lu -> %lu (%lu.%02lu Gbps)\n",
                CURRENT_TIMESTAMP, old_val, lotserver_rate, gbps_int, gbps_frac);
    }
    return ret;
}

static int param_set_gain(const char *val, const struct kernel_param *kp)
{
    unsigned int old_val = lotserver_gain;
    int ret = param_set_uint(val, kp);

    if (ret == 0 && old_val != lotserver_gain && lotserver_verbose) {
        unsigned int gain_int = lotserver_gain / 10;
        unsigned int gain_frac = lotserver_gain % 10;
        pr_info("lotspeed: [uk0@%s] gain changed: %u -> %u (%u.%ux)\n",
                CURRENT_TIMESTAMP, old_val, lotserver_gain, gain_int, gain_frac);
    }
    return ret;
}

static int param_set_min_cwnd(const char *val, const struct kernel_param *kp)
{
    unsigned int old_val = lotserver_min_cwnd;
    int ret = param_set_uint(val, kp);

    if (ret == 0 && old_val != lotserver_min_cwnd && lotserver_verbose) {
        pr_info("lotspeed: [uk0@%s] min_cwnd changed: %u -> %u\n",
                CURRENT_TIMESTAMP, old_val, lotserver_min_cwnd);
    }
    return ret;
}

static int param_set_max_cwnd(const char *val, const struct kernel_param *kp)
{
    unsigned int old_val = lotserver_max_cwnd;
    int ret = param_set_uint(val, kp);

    if (ret == 0 && old_val != lotserver_max_cwnd && lotserver_verbose) {
        pr_info("lotspeed: [uk0@%s] max_cwnd changed: %u -> %u\n",
                CURRENT_TIMESTAMP, old_val, lotserver_max_cwnd);
    }
    return ret;
}

static int param_set_adaptive(const char *val, const struct kernel_param *kp)
{
    bool old_val = lotserver_adaptive;
    int ret = param_set_bool(val, kp);

    if (ret == 0 && old_val != lotserver_adaptive && lotserver_verbose) {
        pr_info("lotspeed: [uk0@%s] adaptive mode: %s -> %s\n",
                CURRENT_TIMESTAMP, old_val ? "ON" : "OFF", lotserver_adaptive ? "ON" : "OFF");
    }
    return ret;
}

static int param_set_turbo(const char *val, const struct kernel_param *kp)
{
    bool old_val = lotserver_turbo;
    int ret = param_set_bool(val, kp);

    if (ret == 0 && old_val != lotserver_turbo && lotserver_verbose) {
        if (lotserver_turbo) {
            pr_info("lotspeed: [uk0@%s] ⚡⚡⚡ TURBO MODE ACTIVATED ⚡⚡⚡\n", CURRENT_TIMESTAMP);
            pr_info("lotspeed: WARNING: Ignoring ALL congestion signals!\n");
        } else {
            pr_info("lotspeed: [uk0@%s] Turbo mode DEACTIVATED\n", CURRENT_TIMESTAMP);
        }
    }
    return ret;
}

static int param_set_beta(const char *val, const struct kernel_param *kp)
{
    unsigned int old_val = lotserver_beta;
    int ret = param_set_uint(val, kp);
    if (ret == 0 && old_val != lotserver_beta && lotserver_verbose) {
        pr_info("lotspeed: [uk0@%s] fairness beta changed: %u -> %u (%u/1024)\n",
                CURRENT_TIMESTAMP, old_val, lotserver_beta, lotserver_beta);
    }
    return ret;
}

static const struct kernel_param_ops param_ops_rate = { .set = param_set_rate, .get = param_get_ulong, };
static const struct kernel_param_ops param_ops_gain = { .set = param_set_gain, .get = param_get_uint, };
static const struct kernel_param_ops param_ops_min_cwnd = { .set = param_set_min_cwnd, .get = param_get_uint, };
static const struct kernel_param_ops param_ops_max_cwnd = { .set = param_set_max_cwnd, .get = param_get_uint, };
static const struct kernel_param_ops param_ops_adaptive = { .set = param_set_adaptive, .get = param_get_bool, };
static const struct kernel_param_ops param_ops_turbo = { .set = param_set_turbo, .get = param_get_bool, };
static const struct kernel_param_ops param_ops_beta = { .set = param_set_beta, .get = param_get_uint, };

// --- 参数定义 ---
module_param(force_unload, bool, 0644);
MODULE_PARM_DESC(force_unload, "Force unload module ignoring references");

module_param_cb(lotserver_rate, &param_ops_rate, &lotserver_rate, 0644);
MODULE_PARM_DESC(lotserver_rate, "Target rate in bytes/sec (default 1Gbps)");

module_param_cb(lotserver_gain, &param_ops_gain, &lotserver_gain, 0644);
MODULE_PARM_DESC(lotserver_gain, "Gain multiplier x10 (20 = 2.0x)");

module_param_cb(lotserver_min_cwnd, &param_ops_min_cwnd, &lotserver_min_cwnd, 0644);
MODULE_PARM_DESC(lotserver_min_cwnd, "Minimum congestion window");

module_param_cb(lotserver_max_cwnd, &param_ops_max_cwnd, &lotserver_max_cwnd, 0644);
MODULE_PARM_DESC(lotserver_max_cwnd, "Maximum congestion window");

module_param_cb(lotserver_adaptive, &param_ops_adaptive, &lotserver_adaptive, 0644);
MODULE_PARM_DESC(lotserver_adaptive, "Enable adaptive rate control");

module_param_cb(lotserver_turbo, &param_ops_turbo, &lotserver_turbo, 0644);
MODULE_PARM_DESC(lotserver_turbo, "Turbo mode - ignore all congestion signals");

module_param_cb(lotserver_beta, &param_ops_beta, &lotserver_beta, 0644);
MODULE_PARM_DESC(lotserver_beta, "Beta for fairness backoff on loss (default 717, i.e. 0.7 * 1024)");

module_param(lotserver_verbose, bool, 0644);
MODULE_PARM_DESC(lotserver_verbose, "Enable verbose logging");

// --- 全局统计 ---
static atomic_t active_connections = ATOMIC_INIT(0);
static atomic64_t total_bytes_sent = ATOMIC64_INIT(0);
static atomic_t total_losses = ATOMIC_INIT(0);

// --- ZETA 学习引擎结构 ---
struct zeta_history_entry {
    u32 daddr;           // 目标 IP
    u64 cached_bw;       // 历史带宽 (Bytes/sec)
    u32 cached_rtt;      // 历史最小 RTT (us)
    u64 last_update;     // 时间戳
    struct hlist_node node;
};

// 全局哈希表与锁
static DEFINE_HASHTABLE(zeta_history_map, HISTORY_BITS);
static DEFINE_SPINLOCK(zeta_history_lock);

// --- 核心状态机 ---
enum lotspeed_state {
    STARTUP,
    PROBING,
    CRUISING,
    AVOIDING,
    PROBE_RTT
};

static const char* state_to_str(enum lotspeed_state state) {
    switch (state) {
        case STARTUP: return "STARTUP";
        case PROBING: return "PROBING";
        case CRUISING: return "CRUISING";
        case AVOIDING: return "AVOIDING";
        case PROBE_RTT: return "PROBE_RTT";
        default: return "UNKNOWN";
    }
}

// --- 私有数据结构 ---
struct lotspeed {
    u64 target_rate;
    u64 actual_rate;
    u32 cwnd_gain;

    enum lotspeed_state state;
    u32 last_state_ts;
    u32 probe_rtt_ts;
    u32 last_cruise_ts;

    u32 rtt_min;
    u32 rtt_cnt;
    u32 loss_count;

    u64 last_bw;
    u32 bw_stalled_rounds;

    bool ss_mode;
    u32 probe_cnt;
    bool history_hit; // 新增：标记是否命中了历史记录

    u64 bytes_sent;
    u64 start_time;
};

// --- Zeta 历史引擎函数 ---
static struct zeta_history_entry *find_history(u32 daddr) {
    struct zeta_history_entry *entry;
    hash_for_each_possible(zeta_history_map, entry, node, daddr) {
        if (entry->daddr == daddr) return entry;
    }
    return NULL;
}

static void update_history(u32 daddr, u64 bw, u32 rtt) {
    struct zeta_history_entry *entry;
    bool found = false;

    spin_lock_bh(&zeta_history_lock);
    entry = find_history(daddr);
    if (entry) {
        // 平滑更新：新数据占 30%
        if (bw > 0) entry->cached_bw = (entry->cached_bw * 7 + bw * 3) / 10;
        if (rtt > 0 && rtt < entry->cached_rtt) entry->cached_rtt = rtt;
        else entry->cached_rtt = (entry->cached_rtt * 7 + rtt * 3) / 10;
        entry->last_update = get_jiffies_64();
        found = true;
    }

    if (!found) {
        entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
        if (entry) {
            entry->daddr = daddr;
            entry->cached_bw = bw;
            entry->cached_rtt = (rtt > 0) ? rtt : 20000;
            entry->last_update = get_jiffies_64();
            hash_add(zeta_history_map, &entry->node, daddr);
            if (lotserver_verbose) pr_info("lotspeed: [Zeta] Learned path to %pI4\n", &daddr);
        }
    }
    spin_unlock_bh(&zeta_history_lock);
}

// --- 状态切换 ---
static void enter_state(struct sock *sk, enum lotspeed_state new_state) {
    struct lotspeed *ca = inet_csk_ca(sk);
    if (ca->state != new_state) {
        if (lotserver_verbose) {
            pr_info("lotspeed: [uk0@%s] state %s -> %s\n",
                    CURRENT_TIMESTAMP, state_to_str(ca->state), state_to_str(new_state));
        }
        ca->state = new_state;
        ca->last_state_ts = tcp_jiffies32;
        if (new_state == CRUISING) ca->last_cruise_ts = tcp_jiffies32;
    }
}

// --- 初始化：集成 Zeta 学习模式 ---
static void lotspeed_init(struct sock *sk)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct lotspeed *ca = inet_csk_ca(sk);
    struct zeta_history_entry *history;
    u32 daddr = sk->sk_daddr;

    memset(ca, 0, sizeof(struct lotspeed));

    // 默认设置
    ca->state = STARTUP;
    ca->last_state_ts = tcp_jiffies32;
    ca->probe_rtt_ts = tcp_jiffies32;
    ca->target_rate = lotserver_rate;
    ca->cwnd_gain = lotserver_gain;
    ca->start_time = ktime_get_real_seconds();
    ca->ss_mode = true;
    ca->history_hit = false;

    tp->snd_ssthresh = lotserver_turbo ? TCP_INFINITE_SSTHRESH : tp->snd_cwnd * 2;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
    cmpxchg(&sk->sk_pacing_status, SK_PACING_NONE, SK_PACING_NEEDED);
#endif

    atomic_inc(&active_connections);

    // === Zeta Learning Logic ===
    rcu_read_lock();
    history = find_history(daddr);
    if (history) {
        u64 age_ms = jiffies_to_msecs(get_jiffies_64() - history->last_update);
        // 记忆未过期且数据有效
        if (age_ms < HISTORY_TTL_SEC * 1000 && history->cached_bw > 0) {
            // 1. 继承带宽 (85% 历史值，留安全余量)
            ca->target_rate = (history->cached_bw * 85) / 100;
            ca->rtt_min = history->cached_rtt;
            ca->history_hit = true;

            // 2. 计算初始窗口 (跳过慢启动)
            if (tp->mss_cache > 0) {
                u64 bdp = (ca->target_rate * (u64)ca->rtt_min) / 1000000;
                u32 init_cwnd = div64_u64(bdp, tp->mss_cache);

                init_cwnd = min_t(u32, init_cwnd, lotserver_max_cwnd);
                init_cwnd = max_t(u32, init_cwnd, 10);

                tp->snd_cwnd = init_cwnd;
                tp->snd_ssthresh = init_cwnd; // 提高门槛

                // 3. 直接进入探测阶段
                ca->state = PROBING;
                ca->ss_mode = false;

                if (lotserver_verbose) {
                    pr_info("lotspeed: [Zeta] HIT! %pI4 | Skip Start | Rate=%llu Mbps | CWND=%u\n",
                            &daddr, ca->target_rate * 8 / 1000000, init_cwnd);
                }
            }
        }
    }
    rcu_read_unlock();
    // ===========================

    if (lotserver_verbose && !ca->history_hit) {
        pr_info("lotspeed: [uk0@%s] NEW connection #%d (Fresh)\n",
                CURRENT_TIMESTAMP, atomic_read(&active_connections));
    }
}

// --- 释放连接：存储学习结果 ---
static void lotspeed_release(struct sock *sk)
{
    struct lotspeed *ca = inet_csk_ca(sk);
    u64 duration = 0;

    if (!ca) {
        atomic_dec(&active_connections);
        return;
    }

    if (ca->start_time > 0) duration = ktime_get_real_seconds() - ca->start_time;
    atomic_dec(&active_connections);

    if (ca->bytes_sent > 0) atomic64_add(ca->bytes_sent, &total_bytes_sent);
    if (ca->loss_count > 0) atomic_add(ca->loss_count, &total_losses);

    // === Zeta Learning Logic ===
    // 只有传输量 > 1MB 且有有效采样时才记录
    if (ca->bytes_sent > 1048576 && ca->actual_rate > 0 && ca->rtt_min > 0) {
        update_history(sk->sk_daddr, ca->actual_rate, ca->rtt_min);
    }
    // ===========================

    if (lotserver_verbose) {
        pr_info("lotspeed: [uk0@%s] Released. Sent=%llu MB. Active=%d\n",
                CURRENT_TIMESTAMP, ca->bytes_sent >> 20, atomic_read(&active_connections));
    }
}

static void lotspeed_update_rtt(struct sock *sk, u32 rtt_us)
{
    struct lotspeed *ca = inet_csk_ca(sk);
    if (!rtt_us) return;
    if (ca->state == PROBE_RTT || !ca->rtt_min || rtt_us < ca->rtt_min) {
        ca->rtt_min = rtt_us;
    }
    ca->rtt_cnt++;
}

// --- 核心算法逻辑 (保持 v3.3 结构，逻辑一致) ---
static void lotspeed_adapt_and_control(struct sock *sk, const struct rate_sample *rs, int flag)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct lotspeed *ca = inet_csk_ca(sk);
    u64 bw = 0;
    u32 rtt_us = tp->srtt_us >> 3;
    u32 cwnd, target_cwnd;
    u32 mss = tp->mss_cache ? : 1460;
    bool congestion_detected = false;

    lotspeed_update_rtt(sk, rtt_us);
    if (!rtt_us) rtt_us = 1000;

    if (rs && rs->delivered > 0) {
        ca->bytes_sent += rs->delivered;
        if (rs->interval_us > 0) {
            bw = (u64)rs->delivered * USEC_PER_SEC;
            do_div(bw, rs->interval_us);
            ca->actual_rate = bw;
        }
    }

    // 拥塞信号检测 (Turbo 模式忽略)
    if (!lotserver_turbo) {
        if (flag & CA_ACK_ECE) congestion_detected = true;
        // Zeta 优化：只有当 RTT 显著增加时，才把丢包视为拥塞
        // 这里先只标记 ECN 和大幅 RTT 膨胀，具体的丢包处理在 ssthresh 中做细分
        if (ca->rtt_min > 0 && rtt_us > ca->rtt_min * 2) congestion_detected = true;
    }

    // 状态机转换
    if (ca->state != PROBE_RTT && ca->rtt_min > 0 &&
        time_after32(tcp_jiffies32, ca->probe_rtt_ts + msecs_to_jiffies(LOTSPEED_PROBE_RTT_INTERVAL_MS))) {
        enter_state(sk, PROBE_RTT);
    }

    switch (ca->state) {
        case STARTUP:
            if (congestion_detected) enter_state(sk, AVOIDING);
            else if (bw > 0) {
                if (bw * 1024 > ca->last_bw * LOTSPEED_STARTUP_GROWTH_TARGET) {
                    ca->last_bw = bw;
                    ca->bw_stalled_rounds = 0;
                } else ca->bw_stalled_rounds++;

                if (ca->bw_stalled_rounds >= LOTSPEED_STARTUP_EXIT_ROUNDS) {
                    ca->target_rate = bw;
                    ca->ss_mode = false;
                    enter_state(sk, PROBING);
                }
            }
            break;
        case PROBING:
            if (congestion_detected) enter_state(sk, AVOIDING);
            else if (bw > ca->target_rate * 9 / 10) enter_state(sk, CRUISING);
            ca->probe_cnt++;
            if (ca->probe_cnt >= 100) ca->probe_cnt = 0;
            break;
        case CRUISING:
            if (congestion_detected) enter_state(sk, AVOIDING);
            else if (time_after32(tcp_jiffies32, ca->last_cruise_ts + msecs_to_jiffies(200))) enter_state(sk, PROBING);
            break;
        case AVOIDING:
            if (!congestion_detected) enter_state(sk, PROBING);
            break;
        case PROBE_RTT:
            if (time_after32(tcp_jiffies32, ca->last_state_ts + msecs_to_jiffies(LOTSPEED_PROBE_RTT_DURATION_MS))) {
                ca->probe_rtt_ts = tcp_jiffies32;
                enter_state(sk, STARTUP);
            }
            break;
    }

    // 速率调整
    switch (ca->state) {
        case STARTUP:
            ca->cwnd_gain = lotserver_gain * 12 / 10;
            ca->target_rate = lotserver_rate; // 尽可能跑满
            break;
        case PROBING:
            ca->target_rate = ca->target_rate * 11 / 10;
            ca->cwnd_gain = lotserver_gain;
            break;
        case CRUISING:
            ca->target_rate = bw * 11 / 10;
            ca->cwnd_gain = lotserver_gain;
            break;
        case AVOIDING:
            ca->target_rate = max_t(u64, bw * 9 / 10, lotserver_rate / 20);
            ca->cwnd_gain = max_t(u32, ca->cwnd_gain * 8 / 10, 10);
            break;
        case PROBE_RTT: break;
    }

    if (lotserver_adaptive) {
        ca->target_rate = min_t(u64, ca->target_rate, lotserver_rate);
        ca->target_rate = max_t(u64, ca->target_rate, lotserver_rate / 20);
    } else {
        ca->target_rate = lotserver_rate;
    }

    // CWND 计算
    target_cwnd = 0;
    if (mss > 0 && rtt_us > 0) {
        target_cwnd = div64_u64(ca->target_rate * (u64)rtt_us, (u64)mss * 1000000);
        target_cwnd = div_u64((u64)target_cwnd * ca->cwnd_gain, 10);
    }

    if (ca->state == PROBE_RTT) {
        cwnd = lotserver_min_cwnd;
    } else if (ca->ss_mode && tp->snd_cwnd < tp->snd_ssthresh) {
        cwnd = tp->snd_cwnd * 2;
        if (target_cwnd > 0 && cwnd >= (u32)target_cwnd) {
            ca->ss_mode = false;
            cwnd = (u32)target_cwnd;
        }
    } else {
        if (ca->state == STARTUP && rs) cwnd = tp->snd_cwnd + rs->acked_sacked;
        else cwnd = target_cwnd;

        if (ca->probe_cnt > 0 && ca->probe_cnt % 100 == 0) cwnd = cwnd * 11 / 10;
    }

    tp->snd_cwnd = clamp(cwnd, lotserver_min_cwnd, lotserver_max_cwnd);
    tp->snd_cwnd = min_t(u32, tp->snd_cwnd, tp->snd_cwnd_clamp);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
    sk->sk_pacing_rate = (ca->target_rate * 6) / 5;
#endif
}

#ifdef LOTSPEED_NEW_CONG_CONTROL_API
static void lotspeed_cong_control(struct sock *sk, u32 ack, int flag, const struct rate_sample *rs) {
    lotspeed_adapt_and_control(sk, rs, flag);
}
#else
static void lotspeed_cong_control(struct sock *sk, const struct rate_sample *rs) {
    lotspeed_adapt_and_control(sk, rs, 0);
}
#endif

// --- SSTHRESH: Zeta 核心智能丢包判断 ---
static u32 lotspeed_ssthresh(struct sock *sk)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct lotspeed *ca = inet_csk_ca(sk);
    u32 rtt_us = tp->srtt_us >> 3;
    u32 tolerance;

    if (lotserver_turbo) return TCP_INFINITE_SSTHRESH;

    // === Zeta Loss Differentiation Logic ===
    // 如果没有 min_rtt，暂时信任当前 rtt
    u32 base_rtt = ca->rtt_min ? ca->rtt_min : rtt_us;

    // 容忍度：MinRTT * 1.25 + 2ms (jitter buffer)
    tolerance = base_rtt + (base_rtt >> 2) + 2000;

    // 判断：丢包时 RTT 是否稳定？
    if (rtt_us <= tolerance) {
        // RTT 很低，说明管道没堵，这极大概率是物理线路丢包
        // 策略：不降速 (Loss Immunity)
        if (lotserver_verbose) {
            pr_info("lotspeed: [Zeta] Immune! Loss at %uus (Target %uus). Ignoring.\n",
                    rtt_us, tolerance);
        }
        // 返回当前窗口，不减半
        return tp->snd_cwnd;
    }
    // =======================================

    // RTT 变大了，说明真堵了，正常降速
    ca->loss_count++;
    ca->cwnd_gain = max_t(u32, ca->cwnd_gain * 8 / 10, 10);

    return max_t(u32, (tp->snd_cwnd * lotserver_beta) / LOTSPEED_BETA_SCALE, lotserver_min_cwnd);
}

static void lotspeed_set_state_hook(struct sock *sk, u8 new_state)
{
    struct lotspeed *ca = inet_csk_ca(sk);
    switch (new_state) {
        case TCP_CA_Loss:
            // 注意：真正的降速逻辑主要由 ssthresh 控制
            // 这里主要处理状态标记
            if (!lotserver_turbo) {
                ca->loss_count++;
                enter_state(sk, AVOIDING);
            }
            break;
        case TCP_CA_Recovery:
            if (!lotserver_turbo) ca->cwnd_gain = max_t(u32, ca->cwnd_gain * 9 / 10, 15);
            break;
        case TCP_CA_Open:
            ca->ss_mode = false;
            break;
    }
}

static u32 lotspeed_undo_cwnd(struct sock *sk) {
    struct tcp_sock *tp = tcp_sk(sk);
    struct lotspeed *ca = inet_csk_ca(sk);
    ca->loss_count = 0;
    ca->ss_mode = false;
    return max(tp->snd_cwnd, tp->prior_cwnd);
}

static void lotspeed_cwnd_event(struct sock *sk, enum tcp_ca_event event) {
    struct lotspeed *ca = inet_csk_ca(sk);
    switch (event) {
        case CA_EVENT_LOSS:
            ca->loss_count++;
            if (!lotserver_turbo) ca->cwnd_gain = max_t(u32, ca->cwnd_gain - 5, 10);
            break;
        case CA_EVENT_TX_START:
        case CA_EVENT_CWND_RESTART:
            ca->ss_mode = true;
            ca->probe_cnt = 0;
            break;
    }
}

static struct tcp_congestion_ops lotspeed_ops __read_mostly = {
        .name           = "lotspeed",
        .owner          = THIS_MODULE,
        .init           = lotspeed_init,
        .release        = lotspeed_release,
        .cong_control   = lotspeed_cong_control,
        .ssthresh       = lotspeed_ssthresh,
        .set_state      = lotspeed_set_state_hook,
        .undo_cwnd      = lotspeed_undo_cwnd,
        .cwnd_event     = lotspeed_cwnd_event,
        .flags          = TCP_CONG_NON_RESTRICTED,
};

// --- 模块生命周期 ---
static int __init lotspeed_module_init(void)
{
    BUILD_BUG_ON(sizeof(struct lotspeed) > ICSK_CA_PRIV_SIZE);

    pr_info("╔════════════════════════════════════════════════════════╗\n");
    pr_info("║      LotSpeed Zeta v4.0 - AI-Driven CCA                ║\n");
    pr_info("║      Features: Learning Mode, RTT Loss Immunity        ║\n");
    pr_info("╚════════════════════════════════════════════════════════╝\n");

    return tcp_register_congestion_control(&lotspeed_ops);
}

static void __exit lotspeed_module_exit(void)
{
    struct zeta_history_entry *entry;
    struct hlist_node *tmp;
    int bkt;

    tcp_unregister_congestion_control(&lotspeed_ops);

    // 等待连接清理
    int retry = 0;
    while (atomic_read(&active_connections) > 0 && retry < 50) {
        msleep(100);
        retry++;
    }

    // === 清理 Zeta 历史记录 ===
    spin_lock_bh(&zeta_history_lock);
    hash_for_each_safe(zeta_history_map, bkt, tmp, entry, node) {
        hash_del(&entry->node);
        kfree(entry);
    }
    spin_unlock_bh(&zeta_history_lock);
    // =========================

    pr_info("lotspeed: Module unloaded. Memory cleared.\n");
}

module_init(lotspeed_module_init);
module_exit(lotspeed_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("uk0 (Gemini Mod)");
MODULE_VERSION("4.0");
MODULE_DESCRIPTION("LotSpeed Zeta - Learning based Congestion Control");
MODULE_ALIAS("tcp_lotspeed");