# -*- coding: utf-8 -*-
"""把 ansi/w65c02op.cpp 的 switch 派发改写成 threaded（computed goto）派发。

机械改写，只有三条规则（指令体原样搬，不改语义）：
    "    case 0xNN:"         -> "op_NN:"
    "        cycle = (...);" -> "        cycle += (...);"   # 批量执行要累加
    "        break;"         -> "        W65_END();"
再套上循环骨架 + 256 项 op_table，并保留两个入口：
    CpuExecuteOPBatch()  = 线程化批量版本（cpu.cpp 的 execute() 用）
    CpuExecuteOP()       = 老的单条语义（CpuExecute()/NMI/IRQ 那条老路用）

指令体只有这一份，所以不用维护两套实现。写完用 git diff 复核。
用法：python tools/mk_w65c02_threaded.py
"""
import io
import re
import sys

PATH = "src_nc3000/ansi/w65c02op.cpp"

HEADER = r"""/* ---- 线程化（computed goto）派发 -----------------------------------------
 * 原版每条指令要过一次 CpuExecuteOP() 调用 + 一次 switch 跳转表派发，cpu.cpp 的
 * execute() 循环里还要再调一次 debug_pc()。这里把 execute() 的循环体搬进同一个
 * 函数，用 computed goto 直接派发 —— 派发开销只剩“取指 + 一次间接跳”。
 *
 *   HOST_BITS = 1 : cpu.cpp 的 execute() 语义（IRQ / mi_clear_delay / WAIT，含 debug_pc）
 *   HOST_BITS = 0 : 老的 CpuExecuteOP() 语义（只跑一条，不做宿主簿记）
 * 一旦 cycle > max_cycles 就返回，等价于原来的 do{...}while(cycle<=max_cycles)。
 *
 * 依赖 GCC/Clang 的 &&label（本项目三个工具链都满足）。case→label、cycle=→cycle+=、
 * break→W65_END() 这三处由 tools/mk_w65c02_threaded.py 机械生成。
 */
#if !defined(__GNUC__) && !defined(__clang__)
#error "w65c02op.cpp needs GCC/Clang computed goto (&&label)"
#endif

#define W65_END() do { \
        if (HOST_BITS && old_mi == true && mI == false) *pmcd = mcdv; \
        if (cycle > max_cycles) return cycle; \
        goto w65_fetch; \
    } while (0)

template <int HOST_BITS>
static DWORD w65c02_run(DWORD cycle, DWORD max_cycles, int *pmcd, int mcdv)
{
    /* 256 条指令的派发表（GCC/Clang 的 label 地址）。函数内 static：只初始化一次，
     * 免得每次调用都在栈上重建 2 KB 指针表。 */
    static void *const op_table[256] = {
"""

FOOTER = r"""    };
    bool old_mi;

    for (;;) {
w65_fetch:
        if (HOST_BITS) {
            if (*pmcd) {
                (*pmcd)--;
            } else if (g_irq && !mI) {
                g_irq = false;
                { extern uint32_t nc3_irq_taken; nc3_irq_taken++; }
                if(enable_dyn_debug_next_n||enable_dyn_debug||enable_debug_pc) printf("execute irq!!!!!!!\n");
                cycle+=CpuExecuteIRQ();
            }
            debug_pc();
        }
        if (g_wai) {
            if (HOST_BITS) { cycle = max_cycles; if (cycle <= 0) cycle = jam_cycles; }
            return cycle;
        }

        old_mi = mI;

        // Fetch opcode
        mOpcode = CPU_PEEK(mPC);
        mPC++;

        if(enable_illegal_op_fix && illegal_op_byte[mOpcode]) {
            assert(illegal_op_cycle[mOpcode]!=0);

            uint8_t & Peek16Debug(uint16_t addr);
            if(debug_level>=1 || enable_dyn_debug || enable_dyn_debug_next_n>0) {
                printf("illegal opcode %02x at pc=$%04x, bs=%02x roabbs=%02x vol=%02x, skipped %d extra bytes\n",mOpcode,mPC-1, Peek16Debug(0), Peek16Debug(0xa), Peek16Debug(0xd), illegal_op_byte[mOpcode]-1);
            }

            mPC += illegal_op_byte[mOpcode]-1;
            cycle += illegal_op_cycle[mOpcode];
            W65_END();
        } //below CPU code is a w65c02 implementation, so the illegal opcodes has to be put before

        goto *op_table[mOpcode];

"""

TAIL = r"""    }
}

/* 线程化批量入口（cpu.cpp 的 CPUInterface::execute 用）*/
DWORD CpuExecuteOPBatch(DWORD cycle_in, DWORD max_cycles, int *pmcd, int mcdv)
{
    return w65c02_run<1>(cycle_in, max_cycles, pmcd, mcdv);
}

/* 老的“只跑一条”入口：max_cycles=0 ⇒ 跑完一条就返回；不做宿主簿记 */
DWORD CpuExecuteOP(void)
{
    return w65c02_run<0>(0, 0, (int *)0, 0);
}
"""


def main():
    raw = io.open(PATH, "rb").read()
    lines = raw.split(b"\n")

    def txt(b):
        return b[:-1] if b.endswith(b"\r") else b

    i_func = i_switch = i_end = None
    for i, ln in enumerate(lines):
        t = txt(ln)
        if i_func is None and t.strip() == b"DWORD CpuExecuteOP(void)":
            i_func = i
        elif i_func is not None and i_switch is None and t.strip() == b"switch (mOpcode) {":
            i_switch = i
        elif i_switch is not None and t == b"    }" and txt(lines[i + 1]).strip() == b"return cycle;":
            i_end = i + 1
            break
    assert i_func is not None and i_switch is not None and i_end is not None, "structure not matched"

    body = [txt(l) for l in lines[i_switch + 1:i_end - 1]]

    out_body = []
    ncase = nbreak = 0
    for b in body:
        m = re.match(rb"^\s*case 0x([0-9A-Fa-f]{2}):\s*$", b)
        if m:
            out_body.append(b"op_" + m.group(1).upper() + b":")
            ncase += 1
            continue
        if b.strip() == b"break;":
            out_body.append(b"        W65_END();")
            nbreak += 1
            continue
        mb = re.match(rb"^(\s*)cycle\s*=\s*(.+?);\s*$", b)
        if mb:
            out_body.append(mb.group(1) + b"cycle += " + mb.group(2) + b";")
            continue
        out_body.append(b)
    assert ncase == 256, "case count %d" % ncase
    assert nbreak == 256, "break count %d" % nbreak

    tbl = []
    for k in range(0, 256, 8):
        tbl.append(b"        " + b" ".join(b"&&op_%02X," % (k + j) for j in range(8)))

    new = []
    new += HEADER.encode("utf-8").split(b"\n")
    new = new[:-1] + tbl          # HEADER 末尾的换行已被 split 掉，接上表
    new += FOOTER.encode("utf-8").split(b"\n")[:-1]
    new += out_body
    new += TAIL.encode("utf-8").split(b"\n")[:-1]

    lines2 = lines[:i_func] + new + lines[i_end + 2:]
    out = b"\n".join(lines2)
    io.open(PATH, "wb").write(out)
    print("rewrote %s: %d cases, %d breaks" % (PATH, ncase, nbreak))


if __name__ == "__main__":
    main()
