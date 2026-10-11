#include "spce061_bridge.h"

/* dynarec 准备（改造计划 §35）：指令直方图计数在 unsp.c 里，dump 在 nc3000_dsp.c 里 */
extern "C" {
extern int  unsp_hist_enabled;
void        n3_opcode_hist_dump(void);
}

#include <cstdio>
#include <cstring>
#include <chrono>

#include "comm.h"
#include "state.h"

extern nc2k_states_t nc2k_states;

extern "C" {
#include "spce061a.h"
#include "unsp.h"
#include "unsp_jit.h"
#include "nc3000_dsp.h"
}

/*
 * 可选加速器：061 的 x86-64 dynarec（spce061a/emu/unsp_jit_x64.c）。
 *
 * 注册进来之后，`n3_run_steps()` 在 `NC3_JIT=1` 时会先让 JIT 跑一片（JIT 内部
 * 逐条调用与解释器同一个 pre/post 钩子，语义逐字节一致），剩下的条数照旧走
 * 解释器；JIT 不可用（32 位 XP 版 / 非 x86-64）时 unsp_jit_run() 恒返回 0，
 * 一切照旧。不设 NC3_JIT 时走的是解释器，和以前完全一样。
 */
static const n3_jit_ops k_n3_jit_ops = {
    unsp_jit_create, unsp_jit_destroy, unsp_jit_set_hooks,
    unsp_jit_reset, unsp_jit_run, unsp_jit_run_one, unsp_jit_stats
};

/* firmware images (spce061a/host/firmware_061_nand.c and spce061a/emu/firmware_061.c) */
extern "C" const uint16_t firmware_061_nand[];
extern "C" const uint16_t firmware_061[];

#define N3_FW_BASE   0x8200u
#define N3_FW_END    0x10000u
#define N3_VEC_TABLE 0xFC00u

/* How many 061 instructions we execute per emulated second: its crystal is
 * 49.152 MHz and the core runs one instruction per clock ⇒ 49.152 M.  The main
 * CPU runs at 14.7456 MHz and hands the 061 10/3 instructions per main cycle
 * (see nc2000.cpp), so the two rates agree exactly.  The audio FIQ must fire
 * once per DAC sample, so the sample count for a slice is
 *     insns_this_slice * sample_rate / N3_INSN_PER_SEC. */
#define N3_INSN_PER_SEC 49152000u

static spce_t        g_dsp;
static nc3000_link_t g_link;
static int           g_up = 0;

/*
 * 061 固件映像的**可写副本**。
 *
 * 原因见 boot_fw() 里那段注释：NAND 里的 升级061.bin 是"待烧写映像"，
 * 头两个字（0x8200/0x8201）是全 0，直接加载会让固件的自检判校验和不符。
 * 而 firmware_*.c 里的数组是 const（只读段），不能就地改。
 */
static uint16_t g_fw_img[32256];

/*
 * 用哪份 061 固件？
 *
 * 2026-09-27 用户把真机自测菜单 `9.SPCE061检测` 的读数报了过来：
 *
 *     Check_Sum: f7 10 3b 30      → [0x8200]=0x10F7, [0x8201]=0x303B
 *     Version:   3d 28            → [0x8204]=0x283D
 *
 * 这正是 `spce061a/rom/061.dat`（出厂 283D）的头，一字节不差。
 * 也就是说**真机那颗 061 跑的是出厂固件，没被 NAND 里的升级文件覆盖过**。
 *
 * 而 `sysdir/升级061.bin`（284F）只是机器里放着的升级包：它不但版本不同，
 * 代码布局也整片不一样（同一个自检处理程序，地址从 0xCF63 挪到了 0xCF7E），
 * 而且自检那条路上行为也不同（见 docs/NC3000模拟器_改造计划.md 第 22 节）。
 * 所以默认必须用 061.dat；想研究 284F 的用环境变量 NC3_DSP_NAND=1 切回去。
 */
static int g_use_nand = 0;
int nc3_dsp_verbose = 0;

/* NC3_PERF：跑 061 累计花掉的墙上时间（微秒），见 spce061_bridge.h。 */
double nc3_dsp_busy_us = 0.0;
/* 061 第一次启动的墙上时刻，用来在退出时报告"061 占了多少墙上时间" */
static std::chrono::steady_clock::time_point g_dsp_t0;

int nc3_dsp_insns_for(uint64_t main_cycles) {
    uint64_t insns = (main_cycles * (uint64_t)N3_INSN_PER_SEC) / (uint64_t)CYCLES_SECOND;
    if (insns > 0x7FFFFFFFULL) insns = 0x7FFFFFFFULL;   /* 追赶片别溢出 int */
    return (int)insns;
}

namespace {
/* 析构时累加 —— nc3_dsp_run() 有多个 return，这样不必在每个出口手写一遍。 */
struct PerfTimer {
    std::chrono::steady_clock::time_point t0;
    PerfTimer() : t0(std::chrono::steady_clock::now()) {}
    ~PerfTimer() {
        nc3_dsp_busy_us += (double)std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - t0).count();
    }
};
}  /* namespace */

/* fractional DAC-sample accumulator: the audio clock is derived from the main
 * CPU's time base (see nc3_dsp_run), not from the 061's instruction count. */
static double g_dsp_sample_acc = 0.0;
static uint64_t g_dsp_fired = 0;      /* DAC samples we clocked in */
static uint32_t dsp_push_last = 0;    /* 上一份日志时的 audio_head（算产率用） */

/*
 * "主控刚跟 061 说过话"的活跃窗口（主控周期计）。
 *
 * 为什么需要它：没有音频流时（解码门 gate=0）`nc3_dsp_run` 会把每片给 061 的指令数
 * 限到 3000 条（061 的核是模拟器里最贵的一块，空闲时没必要全速跑）。但**命令回包
 * 恰恰只在 gate=0 时发生** —— 自测的 `BB 0B` 要让 061 现算一遍 32 位字和（3 万多字），
 * 实测被限速拖成 ~37 ms 才回；而主控 $FB49 那个读循环只等 ~14 ms 就重发命令，
 * 于是在队列里堆出好几份回包，主控第二次读拿到的是陈旧字节 → 自检报
 * "SPCE061 ERR!"（哪怕第一份回包完全正确）。
 *
 * 所以：每次真的往 061 塞了字节（或从它那儿读到字节）就开一个窗口，窗口里不限速。
 * 键盘扫描那些 $3C 轮询不写 $3A、队列也是空的，所以不受影响（实测 $3A 的写入
 * PC 分布里只有协议路径会走到 nc3_dsp_write）。
 */
static uint64_t g_link_active_until = 0;
#define N3_LINK_ACTIVE_MS 150

static void link_mark_active(void)
{
    g_link_active_until = nc2k_states.cycles + (uint64_t)CYCLES_MS * N3_LINK_ACTIVE_MS;
}

static int link_is_active(void)
{
    return nc2k_states.cycles < g_link_active_until;
}

/*
 * 061 每秒能执行的指令是固定的（晶振 49.152 MHz ≈ 主控 10.24 MHz 的 4.8 倍，
 * 本项目按 5 倍给：`nc3_dsp_run(主控周期 × 5)`）。
 * 固件在 UART 轮询里会"顺便"让 061 多跑几步（这样忙线才是实时的），
 * 但这些步数必须**从这一片的预算里扣**，不能额外加 —— 否则 061 跑得比真机快，
 * 解码器就会跑到 DAC 前面：实测每帧只出 637 个样本（正常 992），
 * 听感就是"语速加快 + 一层杂音"。
 * 所以这里记账：本片已经跑掉多少条指令，`nc3_dsp_run` 只补剩下的差额。
 */
static int64_t g_slice_charged = 0;

static void run_charged(int n) {
    uint64_t before = g_link.steps;
    n3_run_steps(&g_link, (uint32_t)n);
    g_slice_charged += (int64_t)(g_link.steps - before);
}

static void boot_fw(int use_nand) {
    const uint16_t *fw = use_nand ? firmware_061_nand : firmware_061;
    int i;

    memcpy(g_fw_img, fw, sizeof g_fw_img);

    /*
     * 修 0x8200/0x8201 这对"校验和"。
     *
     * 两版 061 固件对自检命令 `BB 0B` 的处理不一样，但都要看这两对标头：
     *
     *  061.dat（283D，出厂版）—— 反汇编 out/061_disasm_full.txt，处理程序在 0xCF63：
     *      r1 = FCBB                              ; 默认：校验和不符
     *      if ([0x8200]==[0x8202] && [0x8201]==[0x8203]) r1 = FBBB
     *      回 r1，再回 [0x8200]、[0x8201]
     *    → 只看两对是否相等（061.dat 里两对都是 10F7/303B，所以自检 OK）。
     *
     *  升级061.bin（284F）—— 处理程序在 0xCF7E，实测反汇编：
     *      r3:r4 = 0
     *      for (r2 = 0x8000; r2 != 0x8200; r2++) r3 += [r2]; r4 += carry;   ; 0x8000-0x81FF
     *      for (r2 = 0x8204; r2 != 0xFC00; r2++) r3 += [r2]; r4 += carry;   ; 0x8204-0xFBFF
     *      for (r2 = 0xFFF5; r2 != 0x0000; r2++) r3 += [r2]; r4 += carry;   ; 0xFFF5-0xFFFF
     *      r1 = (r3==[0x8202] && r4==[0x8203]) ? FBBB : FCBB                 ; ① 比 32 位字和
     *      回 r1，再回 r3、r4                                                 ; ② 回"算出来"的值
     *    → **自己算**一遍，和 [0x8202]/[0x8203] 里存的期望值比。
     *
     * 也就是说自检要显示 "SPCE061 OK!!!"，必须让固件**自己算出来的那个字和**
     * 等于 [0x8202]/[0x8203]。061.dat 满足（两对相同），所以 061.dat 一直是 OK。
     *
     * 而 NAND 里的 升级061.bin 是**升级用的映像文件**，不是任何一颗芯片的 flash dump：
     * 它的 0x8000-0x81FF（升级不重写、所以不在文件里）在我们的模拟器里读出来是全 0，
     * 算出来的字和（实测 0x306A:0x95EB）当然不等于文件里写的期望值（0x793B:0x3131）
     * ⇒ 自检报 ERR。文件里那份期望值是在**真芯片的 0x8000-0x81FF 引导扇区**上算的，
     * 而那份数据我们手里没有（061.dat 是另一台机器/另一版，代进去也对不上：
     * 实测 0x313D:0x8893）。
     *
     * 曾经的做法是"按这颗虚拟芯片已经被正确烧写并自检通过"来收尾：用固件自己的算法
     * 把算出来的 32 位字和写回两对标头（位置在求和区间之外，写回去是稳定的）。
     *
     * ⚠️ **现在默认关掉了**（2026-09-27 用户要求）：那等于替真芯片"补一个漂亮的答案"，
     * 属于自己骗自己。默认**一个字节都不改**，只有显式设
     *     set NC3_DSP_FIX_HEADER=1
     * 才会做这个补写（纯粹为了研究 284F 那版固件时看它"如果自检通过会显示什么"）。
     * 默认固件是 061.dat（283D），它两对标头本来就相等，两种情况都不受影响。
     */
    if (getenv("NC3_DSP_FIX_HEADER") &&
        !(g_fw_img[0] == g_fw_img[2] && g_fw_img[1] == g_fw_img[3])) {
        uint32_t sum = 0;
        uint16_t old0 = g_fw_img[0], old1 = g_fw_img[1];
        uint16_t old2 = g_fw_img[2], old3 = g_fw_img[3];
        /* 0xFFFF 在本模拟器里是 SACM 内核当 bank 用的 shadow 寄存器（读回 0），
         * 而 flash 里那位是 0xFFFF。求和区间正好覆盖 0xFFF5-0xFFFF，不抹平的话
         * "照 flash 算"会比"固件自己算"多 0xFFFF（实测差正好是这个值）。 */
        g_fw_img[0xFFFF - N3_FW_BASE] = 0x0000;
        for (i = 0x8204; i < 0xFC00; i++) sum += g_fw_img[i - N3_FW_BASE];
        for (i = 0xFFF5; i <= 0xFFFF; i++) sum += g_fw_img[i - N3_FW_BASE];
        g_fw_img[0] = g_fw_img[2] = (uint16_t)(sum & 0xFFFFu);
        g_fw_img[1] = g_fw_img[3] = (uint16_t)(sum >> 16);
        printf("[061] 标头两对不一致([8200]=%04X%04X [8202]=%04X%04X) -> "
               "NC3_DSP_FIX_HEADER=1：按固件算法重算字和并写回: %04X %04X\n",
               old1, old0, old3, old2, g_fw_img[0], g_fw_img[1]);
    } else if (!(g_fw_img[0] == g_fw_img[2] && g_fw_img[1] == g_fw_img[3])) {
        /* 默认：原样加载，并说明为什么自检会报 ERR（不改任何字节）*/
        printf("[061] 标头两对不一致([8200]=%04X%04X [8202]=%04X%04X)："
               "按原样加载、不改映像 ⇒ 自检 `BB 0B` 会回 BB FC(ERR)\n",
               g_fw_img[1], g_fw_img[0], g_fw_img[3], g_fw_img[2]);
    }

    spce_init(&g_dsp, g_fw_img, N3_FW_BASE, N3_FW_END);
    n3_jit_reset();     /* 换固件/重新 init：丢掉按旧 ROM 翻译出来的块 */
    /* the dumped image has no vector table at 0xFFF4..0xFFFF; rebuild it from 0xFC00 */
    for (i = 0; i < 11; i++) g_dsp.cpu.vec_area[i] = g_fw_img[N3_VEC_TABLE + i - N3_FW_BASE];
    g_dsp.cpu.vec_loaded = 1;
    g_dsp.cpu.ld_sets_flags = 1;   /* MAME load semantics, the firmware relies on it */
    g_dsp.khz_on = 1;
    g_dsp.cpu.r[0] = 0x07FF;       /* SP */
    g_dsp.cpu.r[7] = g_dsp.cpu.vec_area[2];   /* vec 2 = RESET */

    n3_init(&g_link, &g_dsp, 40);
    g_link.no_hold = 1;
    /* The 061's DAC clock is driven by *our* time base (see nc3_dsp_run): the
     * firmware's UART polling hands the chip far more instructions than the
     * 4.8:1 crystal ratio, so an instruction-counted audio clock runs the
     * decoder ~2x too fast and the resampler turns that into noise. */
    g_link.audio_off = 1;
}

int nc3_dsp_boot(void) {
    if (getenv("NC3_DSP_NAND")) g_use_nand = 1;   /* 想看 284F 那版固件时用 */
    if (g_dsp_t0.time_since_epoch().count() == 0)
        g_dsp_t0 = std::chrono::steady_clock::now();
    n3_set_jit(&k_n3_jit_ops);    /* 告诉 nc3000_dsp.c“有加速器可用”（NC3_JIT=1 才真的用）*/
    boot_fw(g_use_nand);
    printf("[061] %s firmware, reset entry $%04X\n",
           g_use_nand ? "NAND 升级061.bin(284F)" : "061.dat(283D,真机同款)", g_dsp.cpu.r[7]);

    /* run the 061 through its own start-up */
    n3_run_steps(&g_link, 400000);
    g_up = 1;
    {
        /* dynarec 准备：NC3_JIT_HIST=1 → 统计指令直方图（见改造计划 §35） */
        unsp_hist_enabled = getenv("NC3_JIT_HIST") ? 1 : 0;
    }
    return 1;
}

int nc3_dsp_ready(void) {
    if (!g_up) return 1;
    /*
     * 固件是"死等"这根 ready 线的（$FABE 无超时），所以每次它来读，都必须让
     * 061 先往前走一小段、把它的忙线刷新到当前状态 —— 否则固件会在一毫秒内
     * 把整段话（几百字节）一次性灌进去：实测 550 字节省略在 1 ms 内发完，
     * 061 的环被灌爆 → 每帧只出 663 个样本（正常 992），听起来就是"加快+杂音"。
     */
    run_charged(24);
    return !n3_busy(&g_link);
}

int nc3_dsp_rx_ready(void) {
    if (!g_up) return 0;
    if (g_link.rx_head != g_link.rx_tail) {
        link_mark_active();          /* 还在收包：继续放它跑，别限速 */
        return 1;
    }
    /* the host is waiting: give the 061 a chance to answer before we report "no" */
    run_charged(link_is_active() ? 512 : 64);
    return g_link.rx_head != g_link.rx_tail;
}

uint8_t nc3_dsp_read(void) {
    uint8_t b = 0xFF;
    if (!g_up) return 0xFF;
    if (n3_recv(&g_link, &b)) {
        link_mark_active();
        if (nc3_dsp_verbose >= 1)
            printf("[061>6502] %02X  @%llums\n", b,
                   (unsigned long long)(nc2k_states.cycles / 10240));
        return b;
    }
    /* keep 0x3A reads side effect free: return the last byte if the queue is empty */
    return 0xFF;
}

int nc3_dsp_write(uint8_t byte) {
    if (!g_up) return 0;
    if (nc3_dsp_verbose >= 1)
        printf("[6502>061] %02X  @%llums\n", byte,
               (unsigned long long)(nc2k_states.cycles / 10240));
    link_mark_active();
    int ok;
    {
        uint64_t before = g_link.steps;
        ok = n3_send(&g_link, byte);
        g_slice_charged += (int64_t)(g_link.steps - before);
    }
    /* 让 061 立刻消化这个字节（真机是 UART 中断），忙线才会随环的占用实时变化 */
    run_charged(24);
    return ok;
}

void nc3_dsp_run(int steps) {
    if (!g_up || steps <= 0) return;
    PerfTimer _perf;                    /* NC3_PERF：量这一片跑 061 用掉的墙上时间 */
    bool gate = (g_dsp.cpu.ram[0x0000] & 0x1000u) != 0;
    /* 061 的"解码门"从 1 落到 0 = 这一首/这一段放完了（或主控让它停了）：
     * 把还没播出去的残留淡出丢掉，否则会被停止后的低采样率拉长成怪声（见 sound.cpp）。*/
    static bool last_gate = false;
    if (last_gate && !gate) {
        extern void dsp061_flush_queue(void);
        extern int dsp061_queue_len(void);
        if (nc3_dsp_verbose >= 1) {
            printf("[061] decode gate 1->0 @%llums: flush host queue (%d samples)\n",
                   (unsigned long long)(nc2k_states.cycles / 10240), dsp061_queue_len());
            fflush(stdout);
        }
        dsp061_flush_queue();
    }
    if (!last_gate && gate && nc3_dsp_verbose >= 1) {
        printf("[061] decode gate 0->1 @%llums\n",
               (unsigned long long)(nc2k_states.cycles / 10240));
        fflush(stdout);
    }
    last_gate = gate;
    /*
     * `budget` = 由主控时间换算出来的"这一片 061 该执行多少条"，
     * 同时也是 **DAC 节拍的基准**（下面按它算该点几次音频中断）。
     * `exec` = 真正交给 061 执行的指令数，可以被背压压小。
     */
    int budget = steps;
    /* When no stream is armed the chip only has to service the UART, so cap the
     * time we spend interpreting it (the 061 core is the expensive part). */
    if (!gate && !link_is_active() && budget > 3000 && getenv("NC3_DSP_NO_CAP") == NULL)
        budget = 3000;
    int exec = budget;
    /*
     * 背压（2026-09-27）：真机 061 的 DAC 只有很小的 FIFO，解码器**不可能跑到 DAC 前面**。
     * 我们的模拟里 061 是按指令数自由跑的，实测每秒多产 5% 样本 ⇒ 多出来的全堆在
     * 主机侧队列（满仓 24000 样本），退出游戏时主控都发停止命令了，队列里还在放，
     * 而且停止后 TimerA 从 63.8 kHz 改回 8 kHz、重采样比一变，就成了"退出后一段怪声"。
     * 这里用"主机侧队列水位"当 DAC FIFO 的近似：积压超过目标值就按比例少给指令。
     */
    {
        extern int dsp061_queue_len(void);
        const int q_target = 3000;      /* ≈47 ms @64 kHz，和真机的听感延迟相当 */
        int q = dsp061_queue_len();
        if (q > q_target) {
            exec = (int)((int64_t)budget * q_target / q);
            if (exec < 600) exec = 600; /* 至少留够服务 UART 的指令 */
        }
    }
    int remain;
    {
        int64_t r2 = (int64_t)exec - g_slice_charged;   /* 轮询已经跑掉的部分 */
        g_slice_charged = 0;
        if (r2 < 0) r2 = 0;
        remain = (int)r2;
    }
    /*
     * DAC 节拍由主控时基决定，而不是"061 执行了多少条指令"。
     * 样本数按 **budget**（主控时间）算，不按实际补跑的 remain 算；
     * 两者加起来才是"真机这一片会执行的指令数"。
     */
    uint32_t r = spce_dac_sample_rate(&g_dsp);
    if (nc3_dsp_verbose >= 1) {
        static uint64_t last = 0;
        uint64_t now = nc2k_states.cycles / 10240;
        if (now - last >= 500) {
            last = now;
            printf("[061] t=%llums gate=%d rate=%u avail=%d fired=%llu dac1=%04X dac2=%04X\n",
                   (unsigned long long)now, (int)gate, r,
                   spce_audio_avail(&g_dsp), (unsigned long long)g_dsp_fired,
                   g_dsp.dac1, g_dsp.dac2);
            {
                extern int dsp061_queue_len(void);
                printf("[061]     host queue = %d samples (~%.0f ms)\n",
                       dsp061_queue_len(), dsp061_queue_len() * 1000.0 / (double)(r ? r : 32000));
                printf("[061]     dac pushes total = %u (%.1f samples/s over this 0.5s)\n",
                       (unsigned)g_dsp.audio_head,
                       ((double)g_dsp.audio_head - (double)dsp_push_last) * 2.0);
                dsp_push_last = g_dsp.audio_head;
                printf("[061]     dac_ctrl(P_DAC_Ctrl $702A) = %04X\n", g_dsp.dac_ctrl);
            }
            fflush(stdout);
        }
    }
    if (!gate || r < 4000 || r > 96000) {
        run_charged(remain);
        g_slice_charged = 0;
        g_dsp_sample_acc = 0.0;
        return;
    }
    g_dsp_sample_acc += (double)budget * (double)r / (double)N3_INSN_PER_SEC;
    run_charged(remain);
    g_slice_charged = 0;                /* 本片结清 */
    {
        int fire = (int)g_dsp_sample_acc;
        if (fire > 4096) fire = 4096;              /* keep up after a long stall */
        g_dsp_sample_acc -= fire;
        g_dsp_fired += (uint32_t)fire;
        while (fire-- > 0) spce_timer_fire_audio(&g_dsp);
    }
}

uint32_t nc3_dsp_sample_rate(void) {
    uint32_t r;
    if (!g_up) return 0;
    r = spce_dac_sample_rate(&g_dsp);
    if (r < 4000 || r > 96000) r = 31958;   /* before TimerA is programmed */
    return r;
}

/* ------------------------------------------------------------------ */
static const uint8_t *g_play;
static uint32_t g_play_len, g_play_pos;   /* 按字节流走，不按 18 字节帧 */
static uint32_t g_play_acc;               /* 欠的字节数 ×1000 */
static int      g_play_on;

int nc3_dsp_play_begin(const uint8_t *data, uint32_t len) {
    uint32_t n;
    if (!g_up || !data || len < 18) return 0;
    n = len / 18;
    g_play = data; g_play_len = n * 18; g_play_pos = 0; g_play_acc = 0;

    if (!n3_link(&g_link)) { printf("[061] link failed\n"); return 0; }
    n3_volume(&g_link, 8);
    n3_speed(&g_link, 2);
    n3_run_steps(&g_link, 4000);
    n3_start(&g_link, 0x21);                 /* 0x99 0x21 = S600 decoder */
    n3_run_steps(&g_link, 4000);
    n3_tts_header(&g_link, 1);
    while (n3_busy(&g_link)) n3_run_steps(&g_link, 64);
    n3_tts_record(&g_link, 0, (uint8_t)(n > 255 ? 255 : n), 0);
    while (n3_busy(&g_link)) n3_run_steps(&g_link, 64);

    g_play_on = 1;
    printf("[061] playing %u frames (%u ms of audio)\n", n, n * 24);
    return 1;
}

void nc3_dsp_play_tick(void) {
    if (!g_play_on) return;
    if (g_play_pos >= g_play_len) {
        n3_end(&g_link);
        n3_run_steps(&g_link, 20000);
        g_play_on = 0;
        printf("[061] stream finished\n");
        return;
    }
    /*
     * 主控真实传输（bank 0x0B $9CAA-$9D50）：`0x33` + **15 字节块**，尾部不足 15
     * 用 `0x22 <len>`；每块之前死等 ready 线（IO 0x0E bit4）。
     * 走 0x22 一次发 18 字节会越过 061 的 16 字节暂存环，把暂存指针踩掉 ——
     * 那正是"全量程削顶噪音"的来源（见 docs/archive/交接文档_2026-09-24_09-26.md
     * 里源文件 交接文档_20260925.md 的 §3.8/§3.11）。
     *
     * 节流完全交给 ready 线：芯片的 DAC 以它自己的 TimerA 速率（S600 单词 =
     * 36790 Hz）从环里取数，环满了才把 ready 拉高。所以这里**不能**再人为按
     * 6000 bps 限速（那会让环周期性排空 → 每个帧边界一次咔哒 = 高频噪声），
     * 只要 ready 为低就一直填（和真机 6502 死等 ready 的行为一致）。
     */
    g_play_acc += 750;                  /* 只用来限制本毫秒最多补几块，防止卡死 */
    {
    int budget = 64;                    /* 1 ms 内最多 64 块（960 B），够用 */
    while (g_play_pos < g_play_len && budget-- > 0) {
        uint32_t left = g_play_len - g_play_pos;
        if (n3_busy(&g_link)) {
            n3_run_steps(&g_link, 32);   /* 让 061 往前跑一点再看 ready */
            if (n3_busy(&g_link)) break;
        }
        if (left >= 15) {
            n3_stream2(&g_link, g_play + g_play_pos);
            g_play_pos += 15;
        } else {
            n3_stream1(&g_link, (uint16_t)left, g_play + g_play_pos);
            g_play_pos += left;
        }
    }
    g_play_acc = 0;
    }
}

int nc3_dsp_play_active(void) { return g_play_on; }

void nc3_dsp_stats(void) {
    if (!g_up) { printf("[061] not running\n"); return; }
    printf("[061] sent=%u recvd=%u timeouts=%u rx_overrun=%u steps=%u "
           "hist=%02X %02X %02X %02X  audio_overruns=%u ring_used=%u rate=%u\n",
           g_link.sent, g_link.recvd, g_link.timeouts, g_link.rx_overrun, g_link.steps,
           g_link.hist[0], g_link.hist[1], g_link.hist[2], g_link.hist[3],
           g_dsp.audio_overruns,
           (unsigned)(g_dsp.audio_head - g_dsp.audio_tail),
           spce_dac_sample_rate(&g_dsp));
    printf("[061] dac fired=%llu (host-clocked)\n", (unsigned long long)g_dsp_fired);
    printf("[061] int_ctrl=%04X timerA_ctrl=%04X timerA_data=%04X timerB_ctrl=%04X khz_on=%u\n",
           g_dsp.int_ctrl, g_dsp.timer_a_ctrl, g_dsp.timer_a_data, g_dsp.timer_b_ctrl,
           (unsigned)g_dsp.khz_on);
    {
        /* 061 到底占了多少墙上时间：nc3_dsp_busy_us 由 nc3_dsp_run() 自己累计，
         * 分母是"061 第一次启动到现在"。慢机器上排查"优化哪一块"看这行。 */
        double wall = (double)std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::steady_clock::now() - g_dsp_t0).count();
        printf("[061] busy=%.3fs / wall=%.3fs = %.1f%%\n",
               nc3_dsp_busy_us / 1e6, wall / 1e6,
               wall > 0.0 ? 100.0 * nc3_dsp_busy_us / wall : 0.0);
    }
    if (g_up) {
        if (unsp_hist_enabled) n3_opcode_hist_dump();
    }
    if (getenv("NC3_JIT")) {
        uint32_t blk = 0, bytes = 0, fl = 0;
        n3_jit_stats(&blk, &bytes, &fl);
        printf("[jit] dynarec blocks=%u code=%.1f KB flushes=%u\n",
               (unsigned)blk, (double)bytes / 1024.0, (unsigned)fl);
    }
    {
        /* NC3_FASTMEM_CHECK=1（w65c02.h 的访存快路差分校验）跑了多少条读 */
        extern int nc3_fastmem_check;
        extern uint32_t nc3_fastmem_rd_hits;
        if (nc3_fastmem_check)
            printf("[fastmem] 差分校验：快路读 %u 次，无差异（有差异会即时打印 [fastmem] RD ...）\n",
                   (unsigned)nc3_fastmem_rd_hits);
    }
}

int nc3_dsp_audio(int16_t *out, int n) {
    int i, got = 0;
    if (!g_up) { for (i = 0; i < n; i++) out[i] = 0; return 0; }
    for (i = 0; i < n; i++) {
        if (spce_audio_avail(&g_dsp) > 0) { spce_audio_get(&g_dsp, &out[i]); got++; }
        else out[i] = 0;
    }
    return got;
}
