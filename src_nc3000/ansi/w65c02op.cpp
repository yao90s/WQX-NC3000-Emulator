#ifdef HANDYPSP

extern "C" {
#include "w65c02.h"
#include "w65c02macro.h"
}

#include <stdio.h>
#include <cassert>
#include "cpu.h"
#include "comm.h"

const bool enable_illegal_op_fix=true;
const bool enable_irq_nmi_cycle_fix=true;

// Read/Write Cycle definitions
#define CPU_RDWR_CYC    1
//#define DMA_RDWR_CYC    4
//#define SPR_RDWR_CYC    3

void xILLEGAL(void);

/* 宿主侧的每指令调试钩子（定义在 cpu_loop_new.cpp；线程化主循环里照旧每条调一次）*/
void debug_pc();

/* ---- 线程化（computed goto）派发 -----------------------------------------
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
        &&op_00, &&op_01, &&op_02, &&op_03, &&op_04, &&op_05, &&op_06, &&op_07,
        &&op_08, &&op_09, &&op_0A, &&op_0B, &&op_0C, &&op_0D, &&op_0E, &&op_0F,
        &&op_10, &&op_11, &&op_12, &&op_13, &&op_14, &&op_15, &&op_16, &&op_17,
        &&op_18, &&op_19, &&op_1A, &&op_1B, &&op_1C, &&op_1D, &&op_1E, &&op_1F,
        &&op_20, &&op_21, &&op_22, &&op_23, &&op_24, &&op_25, &&op_26, &&op_27,
        &&op_28, &&op_29, &&op_2A, &&op_2B, &&op_2C, &&op_2D, &&op_2E, &&op_2F,
        &&op_30, &&op_31, &&op_32, &&op_33, &&op_34, &&op_35, &&op_36, &&op_37,
        &&op_38, &&op_39, &&op_3A, &&op_3B, &&op_3C, &&op_3D, &&op_3E, &&op_3F,
        &&op_40, &&op_41, &&op_42, &&op_43, &&op_44, &&op_45, &&op_46, &&op_47,
        &&op_48, &&op_49, &&op_4A, &&op_4B, &&op_4C, &&op_4D, &&op_4E, &&op_4F,
        &&op_50, &&op_51, &&op_52, &&op_53, &&op_54, &&op_55, &&op_56, &&op_57,
        &&op_58, &&op_59, &&op_5A, &&op_5B, &&op_5C, &&op_5D, &&op_5E, &&op_5F,
        &&op_60, &&op_61, &&op_62, &&op_63, &&op_64, &&op_65, &&op_66, &&op_67,
        &&op_68, &&op_69, &&op_6A, &&op_6B, &&op_6C, &&op_6D, &&op_6E, &&op_6F,
        &&op_70, &&op_71, &&op_72, &&op_73, &&op_74, &&op_75, &&op_76, &&op_77,
        &&op_78, &&op_79, &&op_7A, &&op_7B, &&op_7C, &&op_7D, &&op_7E, &&op_7F,
        &&op_80, &&op_81, &&op_82, &&op_83, &&op_84, &&op_85, &&op_86, &&op_87,
        &&op_88, &&op_89, &&op_8A, &&op_8B, &&op_8C, &&op_8D, &&op_8E, &&op_8F,
        &&op_90, &&op_91, &&op_92, &&op_93, &&op_94, &&op_95, &&op_96, &&op_97,
        &&op_98, &&op_99, &&op_9A, &&op_9B, &&op_9C, &&op_9D, &&op_9E, &&op_9F,
        &&op_A0, &&op_A1, &&op_A2, &&op_A3, &&op_A4, &&op_A5, &&op_A6, &&op_A7,
        &&op_A8, &&op_A9, &&op_AA, &&op_AB, &&op_AC, &&op_AD, &&op_AE, &&op_AF,
        &&op_B0, &&op_B1, &&op_B2, &&op_B3, &&op_B4, &&op_B5, &&op_B6, &&op_B7,
        &&op_B8, &&op_B9, &&op_BA, &&op_BB, &&op_BC, &&op_BD, &&op_BE, &&op_BF,
        &&op_C0, &&op_C1, &&op_C2, &&op_C3, &&op_C4, &&op_C5, &&op_C6, &&op_C7,
        &&op_C8, &&op_C9, &&op_CA, &&op_CB, &&op_CC, &&op_CD, &&op_CE, &&op_CF,
        &&op_D0, &&op_D1, &&op_D2, &&op_D3, &&op_D4, &&op_D5, &&op_D6, &&op_D7,
        &&op_D8, &&op_D9, &&op_DA, &&op_DB, &&op_DC, &&op_DD, &&op_DE, &&op_DF,
        &&op_E0, &&op_E1, &&op_E2, &&op_E3, &&op_E4, &&op_E5, &&op_E6, &&op_E7,
        &&op_E8, &&op_E9, &&op_EA, &&op_EB, &&op_EC, &&op_ED, &&op_EE, &&op_EF,
        &&op_F0, &&op_F1, &&op_F2, &&op_F3, &&op_F4, &&op_F5, &&op_F6, &&op_F7,
        &&op_F8, &&op_F9, &&op_FA, &&op_FB, &&op_FC, &&op_FD, &&op_FE, &&op_FF,
    };
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
            /* debug_pc() 的快路：常见情况（没开任何调试）只把 PC 交给宿主回调，
             * 省掉每条指令一次函数调用；真要看反汇编时才进 debug_pc()（它自己会再
             * 调一次 headless_pc_cb —— 这里用 else if 保证不会重复回调）。 */
            if (debug_level >= 9 || enable_debug_pc || enable_dyn_debug || enable_dyn_debug_next_n)
                debug_pc();
            else if (headless_pc_cb)
                headless_pc_cb((uint16_t)mPC);
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


//
// 0x00
//
op_00:
        cycle += (1 + (6));
        // IMPLIED
        xBRK();
        W65_END();
op_01:
        cycle += (1 + (5));
        xINDIRECT_X();
        xORA();
        W65_END();
op_02:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_03:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_04:
        cycle += (1 + (4));
        xZEROPAGE();
        xTSB();
        W65_END();
op_05:
        cycle += (1 + (2));
        xZEROPAGE();
        xORA();
        W65_END();
op_06:
        cycle += (1 + (4));
        xZEROPAGE();
        xASL();
        W65_END();
op_07:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_08:
        cycle += (1 + (2));
        // IMPLIED
        xPHP();
        W65_END();
op_09:
        cycle += 2;
        xIMMEDIATE();
        xORA();
        W65_END();
op_0A:
        cycle += (1 + (1));
        // IMPLIED
        xASLA();
        W65_END();
op_0B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_0C:
        cycle += (1 + (5));
        xABSOLUTE();
        xTSB();
        W65_END();
op_0D:
        cycle += (1 + (3));
        xABSOLUTE();
        xORA();
        W65_END();
op_0E:
        cycle += (1 + (5));
        xABSOLUTE();
        xASL();
        W65_END();
op_0F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x10
//
op_10:
        cycle += (1 + (1));
        // RELATIVE (IN FUNCTION)
        xBPL();
        W65_END();
op_11:
        cycle += 5;
        xINDIRECT_Y2();
        xORA();
        W65_END();
op_12:
        cycle += (1 + (4));
        xINDIRECT();
        xORA();
        W65_END();
op_13:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_14:
        cycle += (1 + (4));
        xZEROPAGE();
        xTRB();
        W65_END();
op_15:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xORA();
        W65_END();
op_16:
        cycle += (1 + (5));
        xZEROPAGE_X();
        xASL();
        W65_END();
op_17:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_18:
        cycle += (1 + (1));
        // IMPLIED
        xCLC();
        W65_END();
op_19:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xORA();
        W65_END();
op_1A:
        cycle += (1 + (1));
        // IMPLIED
        xINCA();
        W65_END();
op_1B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_1C:
        cycle += (1 + (5));
        xABSOLUTE();
        xTRB();
        W65_END();
op_1D:
        cycle += 4;
        xABSOLUTE_X();
        xORA();
        W65_END();
op_1E:
        cycle += (1 + (6));
        xABSOLUTE_X();
        xASL();
        W65_END();
op_1F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x20
//
op_20:
        cycle += (1 + (5));
        xABSOLUTE();
        xJSR();
        W65_END();
op_21:
        cycle += (1 + (5));
        xINDIRECT_X();
        xAND();
        W65_END();
op_22:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_23:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_24:
        cycle += (1 + (2));
        xZEROPAGE();
        xBIT();
        W65_END();
op_25:
        cycle += (1 + (2));
        xZEROPAGE();
        xAND();
        W65_END();
op_26:
        cycle += (1 + (4));
        xZEROPAGE();
        xROL();
        W65_END();
op_27:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_28:
        cycle += (1 + (3));
        // IMPLIED
        xPLP();
        W65_END();
op_29:
        cycle += (1 + (1));
        xIMMEDIATE();
        xAND();
        W65_END();
op_2A:
        cycle += (1 + (1));
        // IMPLIED
        xROLA();
        W65_END();
op_2B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_2C:
        cycle += (1 + (3));
        xABSOLUTE();
        xBIT();
        W65_END();
op_2D:
        cycle += (1 + (3));
        xABSOLUTE();
        xAND();
        W65_END();
op_2E:
        cycle += (1 + (5));
        xABSOLUTE();
        xROL();
        W65_END();
op_2F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x30
//
op_30:
        cycle += (1 + (1));
        // RELATIVE (IN FUNCTION)
        xBMI();
        W65_END();
op_31:
        cycle += 5;
        xINDIRECT_Y2();
        xAND();
        W65_END();
op_32:
        cycle += (1 + (4));
        xINDIRECT();
        xAND();
        W65_END();
op_33:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_34:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xBIT();
        W65_END();
op_35:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xAND();
        W65_END();
op_36:
        cycle += (1 + (5));
        xZEROPAGE_X();
        xROL();
        W65_END();
op_37:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_38:
        cycle += (1 + (1));
        // IMPLIED
        xSEC();
        W65_END();
op_39:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xAND();
        W65_END();
op_3A:
        cycle += (1 + (1));
        // IMPLIED
        xDECA();
        W65_END();
op_3B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_3C:
        cycle += (1 + (3));
        xABSOLUTE_X();
        xBIT();
        W65_END();
op_3D:
        cycle += (1 + (3));
        xABSOLUTE_X();
        xAND();
        W65_END();
op_3E:
        cycle += (1 + (6));
        xABSOLUTE_X();
        xROL();
        W65_END();
op_3F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x40
//
op_40:
        cycle += (1 + (5));
        // Only clear IRQ if this is not a BRK instruction based RTI

        // B flag is on the stack cant test the flag
        int tmp;
        PULL(tmp);
        PUSH(tmp);
        if (!(tmp & 0x10)) {
            g_wai = g_wai_saved;

            // If were in sleep mode then we need to push the
            // wakeup counter along by the same number of cycles
            // we have used during the sleep period
            if (g_wai) {
                // FIXME: CC800 wakeup
                //gCPUWakeupTime += gSystemCycleCount - gIRQEntryCycle;
            }
        }
        // IMPLIED
        xRTI();
        W65_END();
op_41:
        cycle += (1 + (5));
        xINDIRECT_X();
        xEOR();
        W65_END();
op_42:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_43:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_44:
        cycle += (1 + (2));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_45:
        cycle += (1 + (2));
        xZEROPAGE();
        xEOR();
        W65_END();
op_46:
        cycle += (1 + (4));
        xZEROPAGE();
        xLSR();
        W65_END();
op_47:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_48:
        cycle += (1 + (2));
        // IMPLIED
        xPHA();
        W65_END();
op_49:
        cycle += (1 + (1));
        xIMMEDIATE();
        xEOR();
        W65_END();
op_4A:
        cycle += (1 + (1));
        // IMPLIED
        xLSRA();
        W65_END();
op_4B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_4C:
        cycle += (1 + (2));
        xABSOLUTE();
        xJMP();
        W65_END();
op_4D:
        cycle += (1 + (3));
        xABSOLUTE();
        xEOR();
        W65_END();
op_4E:
        cycle += (1 + (5));
        xABSOLUTE();
        xLSR();
        W65_END();
op_4F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x50
//
op_50:
        cycle += (1 + (1));
        // RELATIVE (IN FUNCTION)
        xBVC();
        W65_END();
op_51:
        cycle += 5;
        xINDIRECT_Y2();
        xEOR();
        W65_END();
op_52:
        cycle += (1 + (4));
        xINDIRECT();
        xEOR();
        W65_END();
op_53:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_54:
        cycle += (1 + (3));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_55:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xEOR();
        W65_END();
op_56:
        cycle += (1 + (5));
        xZEROPAGE_X();
        xLSR();
        W65_END();
op_57:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_58:
        cycle += (1 + (1));
        // IMPLIED
        xCLI();
        W65_END();
op_59:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xEOR();
        W65_END();
op_5A:
        cycle += (1 + (2));
        // IMPLIED
        xPHY();
        W65_END();
op_5B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_5C:
        cycle += (1 + (7));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_5D:
        cycle += (1 + (3));
        xABSOLUTE_X();
        xEOR();
        W65_END();
op_5E:
        cycle += (1 + (6));
        xABSOLUTE_X();
        xLSR();
        W65_END();
op_5F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x60
//
op_60:
        cycle += (1 + (5));
        // IMPLIED
        xRTS();
        W65_END();
op_61:
        cycle += (1 + (5));
        xINDIRECT_X();
        xADC();
        W65_END();
op_62:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_63:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_64:
        cycle += (1 + (2));
        xZEROPAGE();
        xSTZ();
        W65_END();
op_65:
        cycle += (1 + (2));
        xZEROPAGE();
        xADC();
        W65_END();
op_66:
        cycle += (1 + (4));
        xZEROPAGE();
        xROR();
        W65_END();
op_67:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_68:
        cycle += (1 + (3));
        // IMPLIED
        xPLA();
        W65_END();
op_69:
        cycle += (1 + (1));
        xIMMEDIATE();
        xADC();
        W65_END();
op_6A:
        cycle += (1 + (1));
        // IMPLIED
        xRORA();
        W65_END();
op_6B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_6C:
        cycle += 5;
        xINDIRECT_ABSOLUTE();
        xJMP();
        W65_END();
op_6D:
        cycle += (1 + (3));
        xABSOLUTE();
        xADC();
        W65_END();
op_6E:
        cycle += (1 + (5));
        xABSOLUTE();
        xROR();
        W65_END();
op_6F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x70
//
op_70:
        cycle += (1 + (1));
        // RELATIVE (IN FUNCTION)
        xBVS();
        W65_END();
op_71:
        cycle += 5;
        xINDIRECT_Y2();
        xADC();
        W65_END();
op_72:
        cycle += (1 + (4));
        xINDIRECT();
        xADC();
        W65_END();
op_73:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_74:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xSTZ();
        W65_END();
op_75:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xADC();
        W65_END();
op_76:
        cycle += (1 + (5));
        xZEROPAGE_X();
        xROR();
        W65_END();
op_77:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_78:
        cycle += (1 + (1));
        // IMPLIED
        xSEI();
        W65_END();
op_79:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xADC();
        W65_END();
op_7A:
        cycle += (1 + (3));
        // IMPLIED
        xPLY();
        W65_END();
op_7B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_7C:
        cycle += (1 + (5));
        xINDIRECT_ABSOLUTE_X();
        xJMP();
        W65_END();
op_7D:
        cycle += (1 + (3));
        xABSOLUTE_X();
        xADC();
        W65_END();
op_7E:
        cycle += (1 + (6));
        xABSOLUTE_X();
        xROR();
        W65_END();
op_7F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x80
//
op_80:
        cycle += (1 + (2));
        // RELATIVE (IN FUNCTION)
        xBRA();
        W65_END();
op_81:
        cycle += (1 + (5));
        xINDIRECT_X();
        xSTA();
        W65_END();
op_82:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_83:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_84:
        cycle += (1 + (2));
        xZEROPAGE();
        xSTY();
        W65_END();
op_85:
        cycle += (1 + (2));
        xZEROPAGE();
        xSTA();
        W65_END();
op_86:
        cycle += (1 + (2));
        xZEROPAGE();
        xSTX();
        W65_END();
op_87:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_88:
        cycle += (1 + (1));
        // IMPLIED
        xDEY();
        W65_END();
op_89:
        cycle += (1 + (1));
        xIMMEDIATE();
        xBIT();
        W65_END();
op_8A:
        cycle += (1 + (1));
        // IMPLIED
        xTXA();
        W65_END();
op_8B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_8C:
        cycle += (1 + (3));
        xABSOLUTE();
        xSTY();
        W65_END();
op_8D:
        cycle += (1 + (3));
        xABSOLUTE();
        xSTA();
        W65_END();
op_8E:
        cycle += (1 + (3));
        xABSOLUTE();
        xSTX();
        W65_END();
op_8F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0x90
//
op_90:
        cycle += (1 + (1));
        // RELATIVE (IN FUNCTION)
        xBCC();
        W65_END();
op_91:
        cycle += 6;
        xINDIRECT_Y();
        xSTA();
        W65_END();
op_92:
        cycle += (1 + (4));
        xINDIRECT();
        xSTA();
        W65_END();
op_93:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_94:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xSTY();
        W65_END();
op_95:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xSTA();
        W65_END();
op_96:
        cycle += (1 + (3));
        xZEROPAGE_Y();
        xSTX();
        W65_END();
op_97:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_98:
        cycle += (1 + (1));
        // IMPLIED
        xTYA();
        W65_END();
op_99:
        cycle += (1 + (4));
        xABSOLUTE_Y();
        xSTA();
        W65_END();
op_9A:
        cycle += (1 + (1));
        // IMPLIED
        xTXS();
        W65_END();
op_9B:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_9C:
        cycle += (1 + (3));
        xABSOLUTE();
        xSTZ();
        W65_END();
op_9D:
        cycle += (1 + (4));
        xABSOLUTE_X();
        xSTA();
        W65_END();
op_9E:
        cycle += (1 + (4));
        xABSOLUTE_X();
        xSTZ();
        W65_END();
op_9F:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0xA0
//
op_A0:
        cycle += (1 + (1));
        xIMMEDIATE();
        xLDY();
        W65_END();
op_A1:
        cycle += (1 + (5));
        xINDIRECT_X();
        xLDA();
        W65_END();
op_A2:
        cycle += (1 + (1));
        xIMMEDIATE();
        xLDX();
        W65_END();
op_A3:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_A4:
        cycle += (1 + (2));
        xZEROPAGE();
        xLDY();
        W65_END();
op_A5:
        cycle += (1 + (2));
        xZEROPAGE();
        xLDA();
        W65_END();
op_A6:
        cycle += (1 + (2));
        xZEROPAGE();
        xLDX();
        W65_END();
op_A7:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_A8:
        cycle += (1 + (1));
        // IMPLIED
        xTAY();
        W65_END();
op_A9:
        cycle += (1 + (1));
        xIMMEDIATE();
        xLDA();
        W65_END();
op_AA:
        cycle += (1 + (1));
        // IMPLIED
        xTAX();
        W65_END();
op_AB:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_AC:
        cycle += (1 + (3));
        xABSOLUTE();
        xLDY();
        W65_END();
op_AD:
        cycle += (1 + (3));
        xABSOLUTE();
        xLDA();
        W65_END();
op_AE:
        cycle += (1 + (3));
        xABSOLUTE();
        xLDX();
        W65_END();
op_AF:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0xB0
//
op_B0:
        cycle += (1 + (1));
        // RELATIVE (IN FUNCTION)
        xBCS();
        W65_END();
op_B1:
        cycle += 5;
        xINDIRECT_Y2();
        xLDA();
        W65_END();
op_B2:
        cycle += (1 + (4));
        xINDIRECT();
        xLDA();
        W65_END();
op_B3:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_B4:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xLDY();
        W65_END();
op_B5:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xLDA();
        W65_END();
op_B6:
        cycle += (1 + (3));
        xZEROPAGE_Y();
        xLDX();
        W65_END();
op_B7:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_B8:
        cycle += (1 + (1));
        // IMPLIED
        xCLV();
        W65_END();
op_B9:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xLDA();
        W65_END();
op_BA:
        cycle += (1 + (1));
        // IMPLIED
        xTSX();
        W65_END();
op_BB:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_BC:
        cycle += 4;
        xABSOLUTE_X();
        xLDY();
        W65_END();
op_BD:
        cycle += 4;
        xABSOLUTE_X();
        xLDA();
        W65_END();
op_BE:
        cycle += 4;
        xABSOLUTE_Y();
        xLDX();
        W65_END();
op_BF:
        cycle += (1 + (3));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0xC0
//
op_C0:
        cycle += (1 + (1));
        xIMMEDIATE();
        xCPY();
        W65_END();
op_C1:
        cycle += (1 + (5));
        xINDIRECT_X();
        xCMP();
        W65_END();
op_C2:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_C3:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_C4:
        cycle += (1 + (2));
        xZEROPAGE();
        xCPY();
        W65_END();
op_C5:
        cycle += (1 + (2));
        xZEROPAGE();
        xCMP();
        W65_END();
op_C6:
        cycle += (1 + (4));
        xZEROPAGE();
        xDEC();
        W65_END();
op_C7:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_C8:
        cycle += (1 + (1));
        // IMPLIED
        xINY();
        W65_END();
op_C9:
        cycle += (1 + (1));
        xIMMEDIATE();
        xCMP();
        W65_END();
op_CA:
        cycle += (1 + (1));
        // IMPLIED
        xDEX();
        W65_END();
op_CB:
        cycle += (1 + (1));
        // IMPLIED
        xWAI();
        W65_END();
op_CC:
        cycle += (1 + (3));
        xABSOLUTE();
        xCPY();
        W65_END();
op_CD:
        cycle += (1 + (3));
        xABSOLUTE();
        xCMP();
        W65_END();
op_CE:
        cycle += (1 + (5));
        xABSOLUTE();
        xDEC();
        W65_END();
op_CF:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0xD0
//
op_D0:
        cycle += 2;
        // RELATIVE (IN FUNCTION)
        xBNE();
        W65_END();
op_D1:
        cycle += 5;
        xINDIRECT_Y2();
        xCMP();
        W65_END();
op_D2:
        cycle += (1 + (4));
        xINDIRECT();
        xCMP();
        W65_END();
op_D3:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_D4:
        cycle += (1 + (3));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_D5:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xCMP();
        W65_END();
op_D6:
        cycle += (1 + (5));
        xZEROPAGE_X();
        xDEC();
        W65_END();
op_D7:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_D8:
        cycle += (1 + (1));
        // IMPLIED
        xCLD();
        W65_END();
op_D9:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xCMP();
        W65_END();
op_DA:
        cycle += (1 + (2));
        // IMPLIED
        xPHX();
        W65_END();
op_DB:
        cycle += (1 + (1));
        // IMPLIED
        xSTP();
        W65_END();
op_DC:
        cycle += (1 + (3));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_DD:
        cycle += (1 + (3));
        xABSOLUTE_X();
        xCMP();
        W65_END();
op_DE:
        cycle += (1 + (6));
        xABSOLUTE_X();
        xDEC();
        W65_END();
op_DF:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0xE0
//
op_E0:
        cycle += (1 + (1));
        xIMMEDIATE();
        xCPX();
        W65_END();
op_E1:
        cycle += (1 + (5));
        xINDIRECT_X();
        xSBC();
        W65_END();
op_E2:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_E3:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_E4:
        cycle += (1 + (2));
        xZEROPAGE();
        xCPX();
        W65_END();
op_E5:
        cycle += (1 + (2));
        xZEROPAGE();
        xSBC();
        W65_END();
op_E6:
        cycle += (1 + (4));
        xZEROPAGE();
        xINC();
        W65_END();
op_E7:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_E8:
        cycle += (1 + (1));
        // IMPLIED
        xINX();
        W65_END();
op_E9:
        cycle += (1 + (1));
        xIMMEDIATE();
        xSBC();
        W65_END();
op_EA:
        cycle += (1 + (1));
        // IMPLIED
        xNOP();
        W65_END();
op_EB:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_EC:
        cycle += (1 + (3));
        xABSOLUTE();
        xCPX();
        W65_END();
op_ED:
        cycle += (1 + (3));
        xABSOLUTE();
        xSBC();
        W65_END();
op_EE:
        cycle += (1 + (5));
        xABSOLUTE();
        xINC();
        W65_END();
op_EF:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

//
// 0xF0
//
op_F0:
        cycle += 2;
        // RELATIVE (IN FUNCTION)
        xBEQ();
        W65_END();
op_F1:
        cycle += 5;
        xINDIRECT_Y2();
        xSBC();
        W65_END();
op_F2:
        cycle += (1 + (4));
        xINDIRECT();
        xSBC();
        W65_END();
op_F3:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_F4:
        cycle += (1 + (3));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_F5:
        cycle += (1 + (3));
        xZEROPAGE_X();
        xSBC();
        W65_END();
op_F6:
        cycle += (1 + (5));
        xZEROPAGE_X();
        xINC();
        W65_END();
op_F7:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();

op_F8:
        cycle += (1 + (1));
        // IMPLIED
        xSED();
        W65_END();
op_F9:
        cycle += (1 + (3));
        xABSOLUTE_Y();
        xSBC();
        W65_END();
op_FA:
        cycle += (1 + (3));
        // IMPLIED
        xPLX();
        W65_END();
op_FB:
        cycle += (1 + (1));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_FC:
        cycle += (1 + (3));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
op_FD:
        cycle += (1 + (3));
        xABSOLUTE_X();
        xSBC();
        W65_END();
op_FE:
        cycle += (1 + (6));
        xABSOLUTE_X();
        xINC();
        W65_END();
op_FF:
        cycle += (1 + (4));
        // *** ILLEGAL ***
        xILLEGAL();
        W65_END();
    }
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


DWORD CpuExecuteNMI(void) {
    DWORD cycle = 0;
    // FIXME: GET Latest GGV Simulator
    // NC3000 have same behavior
    if (true || g_nmi) {
        //printf("NMI!!!\n");
        // Mark the NMI as services
        //g_nmi = FALSE;
        cycle += 6; // NMI?

        if(enable_irq_nmi_cycle_fix) cycle++;
        //mProcessingInterrupt++;

        // Push processor status
        CPU_POKE(0x0100 + mSP, mPC >> 8); // don't use ++ in poke or will inc/dec 4 times
        mSP--;
        CPU_POKE(0x0100 + mSP, mPC & 0x00ff);
        mSP--;
#ifdef MERGEGGVSIM
        mI = TRUE; // FIXME: MERGE
#endif
        CPU_POKE(0x0100 + mSP, PS());
        mSP--;

        // Pick up the new PC
        mPC = CPU_PEEKW(NMI_VECTOR);
    }
    return cycle;
}

DWORD CpuExecuteIRQ(void) {
    DWORD cycle = 0;
    if ( true|| (g_irq && !mI)) {
        //printf("irq!!\n");
        TRACE_CPU1("Update() IRQ taken at PC=%04x", mPC);
        // IRQ signal clearance is handled by CMikie::Update() as this
        // is the only source of interrupts

        // Push processor status
        PUSH(mPC >> 8);
        PUSH(mPC & 0xff);
#ifdef MERGEGGVSIM
        mB = FALSE; // MERGE
        PUSH(PS());
#else
        PUSH(PS() & 0xef);      // Clear B flag on stack
#endif

        mI = TRUE;              // Stop further interrupts
        mD = FALSE;             // Clear decimal mode

        // Pick up the new PC
        mPC = CPU_PEEKW(IRQ_VECTOR);

        // Save the sleep state as an irq has possibly woken the processor
        g_wai_saved = g_wai;
        g_wai = FALSE;

        // Log the irq entry time
        // FIXME: cc800 wakeup IRQ
        //gIRQEntryCycle = gSystemCycleCount;

        // Clear the interrupt status line
        //g_irq = FALSE;
        cycle += 6;
        if(enable_irq_nmi_cycle_fix) cycle++;
    }
    return cycle;
}

DWORD CpuExecuteUnused(void) {
    DWORD cycle = 0;
#ifdef _LYNXDBG

    // Trigger breakpoint if required

    for (int loop = 0; loop < MAX_CPU_BREAKPOINTS; loop++) {
        if (mPcBreakpoints[loop] == mPC) {
            gBreakpointHit = TRUE;
            mSystem.DebugTrace(0);
        }
    }

    // Check code level debug features
    // back to back CPX ($Absolute)
    // on the 2nd Occurance we do some debug
    if (mOpcode == 0xec) {
        if (mDbgFlag) {
            // We shoud do some debug now
            if (!mOperand) {
                // Trigger a breakpoint
                gBreakpointHit = TRUE;
                // Generate a debug trail output
                mSystem.DebugTrace(0);
            } else {
                // Generate a debug trail output
                mSystem.DebugTrace(mOperand);
            }
            mDbgFlag = 0;
        } else {
            if (mOperand == 0x5aa5) mDbgFlag = 1;
            else mDbgFlag = 0;
        }
    } else {
        mDbgFlag = 0;
    }
#endif
    return cycle;
}

DWORD CpuExecute(void){
    DWORD cycle = CpuExecuteOP();
    if (g_nmi) {
        g_nmi = FALSE; // Clear NMI
        cycle += CpuExecuteNMI();
    }
    if(g_irq && !mI) {
        cycle += CpuExecuteIRQ();
        g_irq = FALSE;
    }
    return cycle;
}

#endif
