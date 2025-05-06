/* $Id$ */
/** @file
 * EM - Execution Monitor(/Manager) - All contexts
 */

/*
 * Copyright (C) 2006-2024 Oracle and/or its affiliates.
 *
 * This file is part of VirtualBox base platform packages, as
 * available from https://www.virtualbox.org.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, in version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses>.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */


/*********************************************************************************************************************************
*   Header Files                                                                                                                 *
*********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_EM
#include <VBox/vmm/em.h>
#include <VBox/vmm/mm.h>
#include <VBox/vmm/selm.h>
#include <VBox/vmm/pgm.h>
#include <VBox/vmm/iem.h>
#include <VBox/vmm/iom.h>
#include <VBox/vmm/hm.h>
#include <VBox/vmm/pdmapi.h>
#include <VBox/vmm/vmm.h>
#include <VBox/vmm/stam.h>
#include "EMInternal.h"
#include <VBox/vmm/vmcc.h>
#include <VBox/param.h>
#include <VBox/err.h>
#include <VBox/dis.h>
#include <VBox/log.h>
#include <iprt/assert.h>
#include <iprt/string.h>




/**
 * Get the current execution manager status.
 *
 * @returns Current status.
 * @param   pVCpu         The cross context virtual CPU structure.
 */
VMM_INT_DECL(EMSTATE) EMGetState(PVMCPU pVCpu)
{
    return pVCpu->em.s.enmState;
}


/**
 * Sets the current execution manager status. (use only when you know what you're doing!)
 *
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   enmNewState The new state, EMSTATE_WAIT_SIPI or EMSTATE_HALTED.
 */
VMM_INT_DECL(void)    EMSetState(PVMCPU pVCpu, EMSTATE enmNewState)
{
    /* Only allowed combination: */
    Assert(pVCpu->em.s.enmState == EMSTATE_WAIT_SIPI && enmNewState == EMSTATE_HALTED);
    pVCpu->em.s.enmState = enmNewState;
}


/**
 * Enables / disable hypercall instructions.
 *
 * This interface is used by GIM to tell the execution monitors whether the
 * hypercall instruction (VMMCALL & VMCALL) are allowed or should \#UD.
 *
 * @param   pVCpu       The cross context virtual CPU structure this applies to.
 * @param   fEnabled    Whether hypercall instructions are enabled (true) or not.
 */
VMMDECL(void) EMSetHypercallInstructionsEnabled(PVMCPU pVCpu, bool fEnabled)
{
    pVCpu->em.s.fHypercallEnabled = fEnabled;
}


/**
 * Checks if hypercall instructions (VMMCALL & VMCALL) are enabled or not.
 *
 * @returns true if enabled, false if not.
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @note    If this call becomes a performance factor, we can make the data
 *          field available thru a read-only view in VMCPU.  See VM::cpum.ro.
 */
VMMDECL(bool) EMAreHypercallInstructionsEnabled(PVMCPU pVCpu)
{
    return pVCpu->em.s.fHypercallEnabled;
}


#if !defined(VBOX_VMM_TARGET_ARMV8)
/**
 * Prepare an MWAIT - essentials of the MONITOR instruction.
 *
 * @returns VINF_SUCCESS
 * @param   pVCpu               The cross context virtual CPU structure of the calling EMT.
 * @param   rax                 The content of RAX.
 * @param   rcx                 The content of RCX.
 * @param   rdx                 The content of RDX.
 * @param   GCPhys              The physical address corresponding to rax.
 */
VMM_INT_DECL(int) EMMonitorWaitPrepare(PVMCPU pVCpu, uint64_t rax, uint64_t rcx, uint64_t rdx, RTGCPHYS GCPhys)
{
    pVCpu->em.s.MWait.uMonitorRAX = rax;
    pVCpu->em.s.MWait.uMonitorRCX = rcx;
    pVCpu->em.s.MWait.uMonitorRDX = rdx;
    pVCpu->em.s.MWait.fWait |= EMMWAIT_FLAG_MONITOR_ACTIVE;
    /** @todo Make use of GCPhys. */
    NOREF(GCPhys);
    /** @todo Complete MONITOR implementation.  */
    return VINF_SUCCESS;
}


/**
 * Checks if the monitor hardware is armed / active.
 *
 * @returns true if armed, false otherwise.
 * @param   pVCpu               The cross context virtual CPU structure of the calling EMT.
 */
VMM_INT_DECL(bool) EMMonitorIsArmed(PVMCPU pVCpu)
{
    return RT_BOOL(pVCpu->em.s.MWait.fWait & EMMWAIT_FLAG_MONITOR_ACTIVE);
}


/**
 * Checks if we're in a MWAIT.
 *
 * @retval  1 if regular,
 * @retval  > 1 if MWAIT with EMMWAIT_FLAG_BREAKIRQIF0
 * @retval  0 if not armed
 * @param   pVCpu               The cross context virtual CPU structure of the calling EMT.
 */
VMM_INT_DECL(unsigned) EMMonitorWaitIsActive(PVMCPU pVCpu)
{
    uint32_t fWait = pVCpu->em.s.MWait.fWait;
    AssertCompile(EMMWAIT_FLAG_ACTIVE == 1);
    AssertCompile(EMMWAIT_FLAG_BREAKIRQIF0 == 2);
    AssertCompile((EMMWAIT_FLAG_ACTIVE << 1) == EMMWAIT_FLAG_BREAKIRQIF0);
    return fWait & (EMMWAIT_FLAG_ACTIVE | ((fWait & EMMWAIT_FLAG_ACTIVE) << 1));
}


/**
 * Performs an MWAIT.
 *
 * @returns VINF_SUCCESS
 * @param   pVCpu               The cross context virtual CPU structure of the calling EMT.
 * @param   rax                 The content of RAX.
 * @param   rcx                 The content of RCX.
 */
VMM_INT_DECL(int) EMMonitorWaitPerform(PVMCPU pVCpu, uint64_t rax, uint64_t rcx)
{
    pVCpu->em.s.MWait.uMWaitRAX = rax;
    pVCpu->em.s.MWait.uMWaitRCX = rcx;
    pVCpu->em.s.MWait.fWait |= EMMWAIT_FLAG_ACTIVE;
    if (rcx)
        pVCpu->em.s.MWait.fWait |= EMMWAIT_FLAG_BREAKIRQIF0;
    else
        pVCpu->em.s.MWait.fWait &= ~EMMWAIT_FLAG_BREAKIRQIF0;
    /** @todo not completely correct?? */
    return VINF_EM_HALT;
}


/**
 * Clears any address-range monitoring that is active.
 *
 * @param   pVCpu   The cross context virtual CPU structure of the calling EMT.
 */
VMM_INT_DECL(void) EMMonitorWaitClear(PVMCPU pVCpu)
{
    LogFlowFunc(("Clearing MWAIT\n"));
    pVCpu->em.s.MWait.fWait &= ~(EMMWAIT_FLAG_ACTIVE | EMMWAIT_FLAG_BREAKIRQIF0);
}


/**
 * Determine if we should continue execution in HM after encountering an mwait
 * instruction.
 *
 * Clears MWAIT flags if returning @c true.
 *
 * @returns true if we should continue, false if we should halt.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pCtx            Current CPU context.
 */
VMM_INT_DECL(bool) EMMonitorWaitShouldContinue(PVMCPU pVCpu, PCPUMCTX pCtx)
{
    if (CPUMGetGuestGif(pCtx))
    {
        if (   CPUMIsGuestPhysIntrEnabled(pVCpu)
            || (   CPUMIsGuestInNestedHwvirtMode(pCtx)
                && CPUMIsGuestVirtIntrEnabled(pVCpu))
            || (   (pVCpu->em.s.MWait.fWait & (EMMWAIT_FLAG_ACTIVE | EMMWAIT_FLAG_BREAKIRQIF0))
                ==                            (EMMWAIT_FLAG_ACTIVE | EMMWAIT_FLAG_BREAKIRQIF0)) )
        {
            if (VMCPU_FF_IS_ANY_SET(pVCpu, (  VMCPU_FF_UPDATE_APIC | VMCPU_FF_INTERRUPT_APIC | VMCPU_FF_INTERRUPT_PIC
                                            | VMCPU_FF_INTERRUPT_NESTED_GUEST)))
            {
                pVCpu->em.s.MWait.fWait &= ~(EMMWAIT_FLAG_ACTIVE | EMMWAIT_FLAG_BREAKIRQIF0);
                return true;
            }
        }
    }

    return false;
}


/**
 * Determine if we should continue execution in HM after encountering a hlt
 * instruction.
 *
 * @returns true if we should continue, false if we should halt.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pCtx            Current CPU context.
 */
VMM_INT_DECL(bool) EMShouldContinueAfterHalt(PVMCPU pVCpu, PCPUMCTX pCtx)
{
    if (CPUMGetGuestGif(pCtx))
    {
        if (CPUMIsGuestPhysIntrEnabled(pVCpu))
            return VMCPU_FF_IS_ANY_SET(pVCpu, (VMCPU_FF_UPDATE_APIC | VMCPU_FF_INTERRUPT_APIC | VMCPU_FF_INTERRUPT_PIC));

        if (   CPUMIsGuestInNestedHwvirtMode(pCtx)
            && CPUMIsGuestVirtIntrEnabled(pVCpu))
            return VMCPU_FF_IS_SET(pVCpu, VMCPU_FF_INTERRUPT_NESTED_GUEST);
    }
    return false;
}
#endif


/**
 * Unhalts and wakes up the given CPU.
 *
 * This is an API for assisting the KVM hypercall API in implementing KICK_CPU.
 * It sets VMCPU_FF_UNHALT for @a pVCpuDst and makes sure it is woken up.   If
 * the CPU isn't currently in a halt, the next HLT instruction it executes will
 * be affected.
 *
 * @returns GVMMR0SchedWakeUpEx result or VINF_SUCCESS depending on context.
 * @param   pVM             The cross context VM structure.
 * @param   pVCpuDst        The cross context virtual CPU structure of the
 *                          CPU to unhalt and wake up.  This is usually not the
 *                          same as the caller.
 * @thread  EMT
 */
VMM_INT_DECL(int) EMUnhaltAndWakeUp(PVMCC pVM, PVMCPUCC pVCpuDst)
{
    /*
     * Flag the current(/next) HLT to unhalt immediately.
     */
    VMCPU_FF_SET(pVCpuDst, VMCPU_FF_UNHALT);

    /*
     * Wake up the EMT (technically should be abstracted by VMM/VMEmt, but
     * just do it here for now).
     */
#ifdef IN_RING0
    /* We might be here with preemption disabled or enabled (i.e. depending on
       thread-context hooks being used), so don't try obtaining the GVMMR0 used
       lock here. See @bugref{7270#c148}. */
    int rc = GVMMR0SchedWakeUpNoGVMNoLock(pVM, pVCpuDst->idCpu);
    AssertRC(rc);

#elif defined(IN_RING3)
    VMR3NotifyCpuFFU(pVCpuDst->pUVCpu, 0 /*fFlags*/);
    int rc = VINF_SUCCESS;
    RT_NOREF(pVM);

#else
    /* Nothing to do for raw-mode, shouldn't really be used by raw-mode guests anyway. */
    Assert(pVM->cCpus == 1); NOREF(pVM);
    int rc = VINF_SUCCESS;
#endif
    return rc;
}

#ifndef IN_RING3

/**
 * Makes an I/O port write pending for ring-3 processing.
 *
 * @returns VINF_EM_PENDING_R3_IOPORT_READ
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uPort           The I/O port.
 * @param   cbInstr         The instruction length (for RIP updating).
 * @param   cbValue         The write size.
 * @param   uValue          The value being written.
 * @sa      emR3ExecutePendingIoPortWrite
 *
 * @note    Must not be used when I/O port breakpoints are pending or when single stepping.
 */
VMMRZ_INT_DECL(VBOXSTRICTRC)
EMRZSetPendingIoPortWrite(PVMCPU pVCpu, RTIOPORT uPort, uint8_t cbInstr, uint8_t cbValue, uint32_t uValue)
{
    Assert(pVCpu->em.s.PendingIoPortAccess.cbValue == 0);
    pVCpu->em.s.PendingIoPortAccess.uPort     = uPort;
    pVCpu->em.s.PendingIoPortAccess.cbValue   = cbValue;
    pVCpu->em.s.PendingIoPortAccess.cbInstr   = cbInstr;
    pVCpu->em.s.PendingIoPortAccess.uValue    = uValue;
    return VINF_EM_PENDING_R3_IOPORT_WRITE;
}


/**
 * Makes an I/O port read pending for ring-3 processing.
 *
 * @returns VINF_EM_PENDING_R3_IOPORT_READ
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uPort           The I/O port.
 * @param   cbInstr         The instruction length (for RIP updating).
 * @param   cbValue         The read size.
 * @sa      emR3ExecutePendingIoPortRead
 *
 * @note    Must not be used when I/O port breakpoints are pending or when single stepping.
 */
//标记待处理的 I/O 端口读操作
/*
  pVCpu：指向当前虚拟 CPU（VCPU）的上下文结构体。
  uPort：目标 I/O 端口号（如 0x60 对应键盘控制器）。
  cbInstr：触发该操作的指令长度（字节数）。
  cbValue：待读取数据的字节宽度（1/2/4 字节）。
 * */
/*
I/O 虚拟化机制
    硬件辅助虚拟化（VT-x）：Guest 执行 IN 指令时触发 VM-Exit，由 VMM 处理端口访问
    软件模拟（IEM）：若未启用硬件加速，EM 通过指令解释器捕获 I/O 操作
    延迟处理原因

    上下文切换开销：直接处理 I/O 可能导致频繁的 Ring-0 ↔ Ring-3 切换，通过标记挂起操作批量处理可提升性能
    复杂模拟需求：某些端口（如 0x3F8 串口）需调用宿主系统的驱动或模拟设备，需在用户态（Ring-3）完成
*/

/*
特性            VirtualBox (EM)                         KVM
I/O 处理位置	混合模式（Ring-0 标记 → Ring-3 执行）   内核态直接模拟（通过 QEMU 设备模型）57
性能优化	    延迟批量处理	                        事件fd 异步通知机制57
适用场景	    复杂设备模拟（如 USB 控制器）	        高性能直通设备（如 VT-d）
 * */
VMMRZ_INT_DECL(VBOXSTRICTRC)
EMRZSetPendingIoPortRead(PVMCPU pVCpu, RTIOPORT uPort, uint8_t cbInstr, uint8_t cbValue)
{
    Assert(pVCpu->em.s.PendingIoPortAccess.cbValue == 0);//确保当前无其他挂起的 I/O 操作
    pVCpu->em.s.PendingIoPortAccess.uPort     = uPort;
    pVCpu->em.s.PendingIoPortAccess.cbValue   = cbValue;
    pVCpu->em.s.PendingIoPortAccess.cbInstr   = cbInstr;
    pVCpu->em.s.PendingIoPortAccess.uValue    = UINT32_C(0x52454144); /* 'READ' *///通过魔数 0x52454144（ASCII "READ"）标识读操作。
    return VINF_EM_PENDING_R3_IOPORT_READ;//通知调用方需切换到 Ring-3 emR3NemHandleRC 处理
}

#endif /* IN_RING3 */


/**
 * Worker for EMHistoryExec that checks for ring-3 returns and flags
 * continuation of the EMHistoryExec run there.
 */
DECL_FORCE_INLINE(void) emHistoryExecSetContinueExitRecIdx(PVMCPU pVCpu, VBOXSTRICTRC rcStrict, PCEMEXITREC pExitRec)
{
    pVCpu->em.s.idxContinueExitRec = UINT16_MAX;
#ifdef IN_RING3
    RT_NOREF_PV(rcStrict); RT_NOREF_PV(pExitRec);
#else
    switch (VBOXSTRICTRC_VAL(rcStrict))
    {
        case VINF_SUCCESS:
        default:
            break;

        /*
         * Only status codes that EMHandleRCTmpl.h will resume EMHistoryExec with.
         */
            //仅以下状态码会触发记录（需后续 Ring-3 emR3NemHandleRC 处理）：
        case VINF_IOM_R3_IOPORT_READ:           /* -> emR3ExecuteIOInstruction */
        case VINF_IOM_R3_IOPORT_WRITE:          /* -> emR3ExecuteIOInstruction */
        case VINF_IOM_R3_IOPORT_COMMIT_WRITE:   /* -> VMCPU_FF_IOM -> VINF_EM_RESUME_R3_HISTORY_EXEC -> emR3ExecuteIOInstruction */
        case VINF_IOM_R3_MMIO_READ:             /* -> emR3ExecuteInstruction */
        case VINF_IOM_R3_MMIO_WRITE:            /* -> emR3ExecuteInstruction */
        case VINF_IOM_R3_MMIO_READ_WRITE:       /* -> emR3ExecuteInstruction */
        case VINF_IOM_R3_MMIO_COMMIT_WRITE:     /* -> VMCPU_FF_IOM -> VINF_EM_RESUME_R3_HISTORY_EXEC -> emR3ExecuteIOInstruction */
        case VINF_CPUM_R3_MSR_READ:             /* -> emR3ExecuteInstruction */
        case VINF_CPUM_R3_MSR_WRITE:            /* -> emR3ExecuteInstruction */
        case VINF_GIM_R3_HYPERCALL:             /* -> emR3ExecuteInstruction */
            pVCpu->em.s.idxContinueExitRec = (uint16_t)(pExitRec - &pVCpu->em.s.aExitRecords[0]);
            break;
    }
#endif /* !IN_RING3 */
}


/**
 * Execute using history.
 *
 * This function will be called when EMHistoryAddExit() and friends returns a
 * non-NULL result.  This happens in response to probing or when probing has
 * uncovered adjacent exits which can more effectively be reached by using IEM
 * than restarting execution using the main execution engine and fielding an
 * regular exit.
 *
 * @returns VBox strict status code, see IEMExecForExits.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pExitRec        The exit record return by a previous history add
 *                          or update call.
 * @param   fWillExit       Flags indicating to IEM what will cause exits, TBD.
 */
//根据退出记录（Exit Record）动态管理虚拟 CPU 的指令执行策略，优化虚拟机性能。
/*
  pVCpu：当前虚拟 CPU 的上下文。
  pExitRec：退出记录，包含执行行为配置（如最大无退出指令数）。
  fWillExit：标志位，指示可能的退出条件。
 * */
/*
  批量执行：通过 EXEC_WITH_MAX 减少频繁退出带来的性能损耗。
  自适应探测：通过 EXEC_PROBE 分析指令流特征，动态选择最优执行模式。
  容错处理：忽略非关键错误（如未实现指令），确保虚拟机持续运行。
 * */
/*
 * (1) EMEXITACTION_EXEC_WITH_MAX（批量执行模式）
目标：尝试至少执行 cMinInstructions 条指令，但允许提前退出。
处理流程：
  若在 cMinInstructions 内触发退出：
  通过 emHistoryExecSetContinueExitRecIdx 记录退出位置（pVCpu->em.s.idxContinueExitRec）。
  不强制完成剩余指令，直接返回当前状态码（如 VINF_EM_HALT）。
  若退出原因是 未实现指令（VERR_IEM_INSTR_NOT_IMPLEMENTED）：
  忽略错误并返回 VINF_SUCCESS（除非一条指令都未执行）。
统计调整：
  仍会记录已执行的指令数（ExecStats.cInstructions），但不会计入“节省的退出次数”（因为未达到优化目标）。
(2) EMEXITACTION_EXEC_PROBE（探测模式）
目标：动态探测最佳执行窗口，可能调整后续策略。
处理流程：
  若在 cHistoryProbeMinInstructions 内触发退出：
  检查退出次数（ExecStats.cExits）：
  如果退出 ≥ 2 次：转换为 EXEC_WITH_MAX 模式，并基于实际退出间隔（ExecStats.cMaxExitDistance）设置 cMaxInstructionsWithoutExit。
  如果退出仅 1 次：降级为普通模式（NORMAL_PROBED），后续不再优化。
  若在非 Ring-3 环境且需进一步处理，触发 Ring-3 回退（StatHistoryProbedToRing3）。
统计记录：
  无论是否完成最小指令数，均记录已执行的指令数（StatHistoryProbeInstructions）。
 * */

/*
 * 实际场景示例
假设配置 cMinInstructions = 10，但执行到第 5 条指令时遇到 IN 指令（触发 I/O 退出）：
EXEC_WITH_MAX 模式：
  立即停止执行，返回 VINF_EM_HALT。
  记录已执行的 5 条指令，但不标记为“优化成功”。
EXEC_PROBE 模式：
  如果这是近期第 2 次退出，可能将 cMaxInstructionsWithoutExit 设为 5，后续改用更保守的批量执行。
 * */
VMM_INT_DECL(VBOXSTRICTRC) EMHistoryExec(PVMCPUCC pVCpu, PCEMEXITREC pExitRec, uint32_t fWillExit)
{
    Assert(pExitRec);
    VMCPU_ASSERT_EMT(pVCpu);
    IEMEXECFOREXITSTATS ExecStats;
    switch (pExitRec->enmAction)
    {
        /*
         * Executes multiple instruction stopping only when we've gone a given
         * number without perceived exits.
         */
        case EMEXITACTION_EXEC_WITH_MAX:
        {
            STAM_REL_PROFILE_START(&pVCpu->em.s.StatHistoryExec, a);
            LogFlow(("EMHistoryExec/EXEC_WITH_MAX: %RX64, max %u\n", pExitRec->uFlatPC, pExitRec->cMaxInstructionsWithoutExit));
            VBOXSTRICTRC rcStrict = IEMExecForExits(pVCpu, fWillExit,
                                                    pExitRec->cMaxInstructionsWithoutExit /* cMinInstructions*/,//最小指令数,确保至少尝试执行这么多条指令
                                                    pVCpu->em.s.cHistoryExecMaxInstructions,//最大指令数
                                                    pExitRec->cMaxInstructionsWithoutExit,
                                                    &ExecStats);
            LogFlow(("EMHistoryExec/EXEC_WITH_MAX: %Rrc cExits=%u cMaxExitDistance=%u cInstructions=%u\n",
                     VBOXSTRICTRC_VAL(rcStrict), ExecStats.cExits, ExecStats.cMaxExitDistance, ExecStats.cInstructions));
            emHistoryExecSetContinueExitRecIdx(pVCpu, rcStrict, pExitRec);

            /* Ignore instructions IEM doesn't know about. */
            //忽略未实现的指令错误（如 VERR_IEM_INSTR_NOT_IMPLEMENTED），除非一条指令都未执行。
            if (   (   rcStrict != VERR_IEM_INSTR_NOT_IMPLEMENTED
                    && rcStrict != VERR_IEM_ASPECT_NOT_IMPLEMENTED)
                || ExecStats.cInstructions == 0)
            { /* likely */ }
            else
                rcStrict = VINF_SUCCESS;// 忽略未实现指令的错误

            if (ExecStats.cExits > 1)
                STAM_REL_COUNTER_ADD(&pVCpu->em.s.StatHistoryExecSavedExits, ExecStats.cExits - 1);//记录节省的退出次数
            STAM_REL_COUNTER_ADD(&pVCpu->em.s.StatHistoryExecInstructions, ExecStats.cInstructions);//记录总执行指令数
            STAM_REL_PROFILE_STOP(&pVCpu->em.s.StatHistoryExec, a);
            return rcStrict;
        }

        /*
         * Probe a exit for close by exits.
         */
        //探测当前退出点附近的指令流，动态调整后续执行策略。
        case EMEXITACTION_EXEC_PROBE:
        {
            STAM_REL_PROFILE_START(&pVCpu->em.s.StatHistoryProbe, b);
            LogFlow(("EMHistoryExec/EXEC_PROBE: %RX64\n", pExitRec->uFlatPC));
            PEMEXITREC   pExitRecUnconst = (PEMEXITREC)pExitRec;
            VBOXSTRICTRC rcStrict = IEMExecForExits(pVCpu, fWillExit,
                                                    pVCpu->em.s.cHistoryProbeMinInstructions,//最小指令数（探测的最低要求）
                                                    pVCpu->em.s.cHistoryExecMaxInstructions,//最大允许执行的指令数
                                                    pVCpu->em.s.cHistoryProbeMaxInstructionsWithoutExit,//最大无退出指令数 
                                                    &ExecStats);
            LogFlow(("EMHistoryExec/EXEC_PROBE: %Rrc cExits=%u cMaxExitDistance=%u cInstructions=%u\n",
                     VBOXSTRICTRC_VAL(rcStrict), ExecStats.cExits, ExecStats.cMaxExitDistance, ExecStats.cInstructions));
            emHistoryExecSetContinueExitRecIdx(pVCpu, rcStrict, pExitRecUnconst);
            if (   ExecStats.cExits >= 2//如果发现多次退出（cExits >= 2），切换为 EXEC_WITH_MAX 模式，并基于实际退出间隔（cMaxExitDistance）调整参数。
                && RT_SUCCESS(rcStrict))
            {
                Assert(ExecStats.cMaxExitDistance > 0 && ExecStats.cMaxExitDistance <= 32);
                pExitRecUnconst->cMaxInstructionsWithoutExit = ExecStats.cMaxExitDistance;
                pExitRecUnconst->enmAction = EMEXITACTION_EXEC_WITH_MAX;// 切换为批量执行模式
                LogFlow(("EMHistoryExec/EXEC_PROBE: -> EXEC_WITH_MAX %u\n", ExecStats.cMaxExitDistance));
                STAM_REL_COUNTER_INC(&pVCpu->em.s.StatHistoryProbedExecWithMax);//成功优化次数
            }
#ifndef IN_RING3
            //触发 Ring-3 处理（非 Ring-3 环境时）
            else if (   pVCpu->em.s.idxContinueExitRec != UINT16_MAX
                     && RT_SUCCESS(rcStrict))
            {
                STAM_REL_COUNTER_INC(&pVCpu->em.s.StatHistoryProbedToRing3);//Ring-3 回退次数
                LogFlow(("EMHistoryExec/EXEC_PROBE: -> ring-3\n"));
            }
#endif
            else
            {
                //否则降级为普通模式（NORMAL_PROBED）
                pExitRecUnconst->enmAction = EMEXITACTION_NORMAL_PROBED;
                pVCpu->em.s.idxContinueExitRec = UINT16_MAX;
                LogFlow(("EMHistoryExec/EXEC_PROBE: -> PROBED\n"));
                STAM_REL_COUNTER_INC(&pVCpu->em.s.StatHistoryProbedNormal);//降级次数
                if (   rcStrict == VERR_IEM_INSTR_NOT_IMPLEMENTED
                    || rcStrict == VERR_IEM_ASPECT_NOT_IMPLEMENTED)
                    rcStrict = VINF_SUCCESS;
            }
            STAM_REL_COUNTER_ADD(&pVCpu->em.s.StatHistoryProbeInstructions, ExecStats.cInstructions);
            STAM_REL_PROFILE_STOP(&pVCpu->em.s.StatHistoryProbe, b);
            return rcStrict;
        }

        /* We shouldn't ever see these here! */
        case EMEXITACTION_FREE_RECORD:
        case EMEXITACTION_NORMAL:
        case EMEXITACTION_NORMAL_PROBED:
            break;

        /* No default case, want compiler warnings. */
    }
    AssertLogRelFailedReturn(VERR_EM_INTERNAL_ERROR);
}


/**
 * Worker for emHistoryAddOrUpdateRecord.
 */
//初始化虚拟机退出记录(EMEXITREC)
//记录虚拟机执行过程中发生的退出事件信息
DECL_FORCE_INLINE(PCEMEXITREC) emHistoryRecordInit(PEMEXITREC pExitRec, uint64_t uFlatPC, uint32_t uFlagsAndType, uint64_t uExitNo)
{
    pExitRec->uFlatPC                     = uFlatPC;//记录退出时的指令地址
    pExitRec->uFlagsAndType               = uFlagsAndType;//退出标志和类型组合值
    pExitRec->enmAction                   = EMEXITACTION_NORMAL;//设置为默认动作
    pExitRec->bUnused                     = 0;
    pExitRec->cMaxInstructionsWithoutExit = 64;
    pExitRec->uLastExitNo                 = uExitNo;//记录最后一次退出序号
    pExitRec->cHits                       = 1;//命中计数器初始化为1
    return NULL;
}


/**
 * Worker for emHistoryAddOrUpdateRecord.
 */
DECL_FORCE_INLINE(PCEMEXITREC) emHistoryRecordInitNew(PVMCPU pVCpu, PEMEXITENTRY pHistEntry, uintptr_t idxSlot,
                                                      PEMEXITREC pExitRec, uint64_t uFlatPC,
                                                      uint32_t uFlagsAndType, uint64_t uExitNo)
{
    pHistEntry->idxSlot = (uint32_t)idxSlot;
    pVCpu->em.s.cExitRecordUsed++;
    LogFlow(("emHistoryRecordInitNew: [%#x] = %#07x %016RX64; (%u of %u used)\n", idxSlot, uFlagsAndType, uFlatPC,
             pVCpu->em.s.cExitRecordUsed, RT_ELEMENTS(pVCpu->em.s.aExitRecords) ));
    return emHistoryRecordInit(pExitRec, uFlatPC, uFlagsAndType, uExitNo);
}


/**
 * Worker for emHistoryAddOrUpdateRecord.
 */
DECL_FORCE_INLINE(PCEMEXITREC) emHistoryRecordInitReplacement(PEMEXITENTRY pHistEntry, uintptr_t idxSlot,
                                                              PEMEXITREC pExitRec, uint64_t uFlatPC,
                                                              uint32_t uFlagsAndType, uint64_t uExitNo)
{
    pHistEntry->idxSlot = (uint32_t)idxSlot;
    LogFlow(("emHistoryRecordInitReplacement: [%#x] = %#07x %016RX64 replacing %#07x %016RX64 with %u hits, %u exits old\n",
             idxSlot, uFlagsAndType, uFlatPC, pExitRec->uFlagsAndType, pExitRec->uFlatPC, pExitRec->cHits,
             uExitNo - pExitRec->uLastExitNo));
    return emHistoryRecordInit(pExitRec, uFlatPC, uFlagsAndType, uExitNo);
}


/**
 * Adds or updates the EMEXITREC for this PC/type and decide on an action.
 *
 * @returns Pointer to an exit record if special action should be taken using
 *          EMHistoryExec().  Take normal exit action when NULL.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uFlagsAndType   Combined flags and type, EMEXIT_F_KIND_EM set and
 *                          both EMEXIT_F_CS_EIP and EMEXIT_F_UNFLATTENED_PC are clear.
 * @param   uFlatPC         The flattened program counter.
 * @param   pHistEntry      The exit history entry.
 * @param   uExitNo         The current exit number.
 */
/*
  管理虚拟CPU的退出记录（Exit Records），通过哈希表动态跟踪指令执行路径的退出行为，并基于历史数据优化后续执行策略
    查找/更新记录：根据指令地址（uFlatPC）匹配或创建退出记录。
    冲突处理：哈希碰撞时采用二次探测 + LRU（最近最少使用）策略。
    策略升级：根据命中次数动态调整退出动作类型（如 NORMAL → EXEC_PROBE）。

    渐进式升级：NORMAL → PROBE → WITH_MAX，避免过早优化不稳定路径。
    退化机制：PROBE 失败后降级，防止无效探测拖累性能。

    场景：客户机循环执行 CPUID 指令（频繁触发VM Exit）。

首次退出：
  创建 NORMAL 记录，cHits=1。
第256次退出：
  升级为 EXEC_PROBE，后续尝试批量执行（IEMExecForExits）。
若探测成功：
  转换为 EXEC_WITH_MAX，减少退出频率。
若探测失败：
  512次后降级为 NORMAL_PROBED，回退到默认处理。

 注意： 当启用EPT（Extended Page Tables）硬件功能时，它会自动处理许多原本需要通过VM Exit由VMM
       （虚拟机监控器）处理的页表访问操作，从而减少了某些类型的VM Exit。
        这种减少会间接影响 EMHistoryExec 这类优化机制的效果。

        EPT 减少了内存相关退出，使得 EMHistoryExec 的优化目标（减少退出）主要集中在 非内存操作（如设备 I/O）。
 * */
static PCEMEXITREC emHistoryAddOrUpdateRecord(PVMCPU pVCpu, uint64_t uFlagsAndType, uint64_t uFlatPC,
                                              PEMEXITENTRY pHistEntry, uint64_t uExitNo)
{
# ifdef IN_RING0
    /* Disregard the hm flag. */
    uFlagsAndType &= ~EMEXIT_F_HM;
# endif

    /*
     * Work the hash table.
     */
    AssertCompile(RT_ELEMENTS(pVCpu->em.s.aExitRecords) == 1024);
# define EM_EXIT_RECORDS_IDX_MASK 0x3ff
    uintptr_t  idxSlot  = ((uintptr_t)uFlatPC >> 1) & EM_EXIT_RECORDS_IDX_MASK;//计算哈希索引
    PEMEXITREC pExitRec = &pVCpu->em.s.aExitRecords[idxSlot];
    if (pExitRec->uFlatPC == uFlatPC)//直接命中
    {
        Assert(pExitRec->enmAction != EMEXITACTION_FREE_RECORD);
        pHistEntry->idxSlot = (uint32_t)idxSlot;
        if (pExitRec->uFlagsAndType == uFlagsAndType)
        {
            pExitRec->uLastExitNo = uExitNo;//标志相同：更新最后退出号(uLastExitNo)
            STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecHits[0]);
        }
        else
        {
            STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecTypeChanged[0]);
            return emHistoryRecordInit(pExitRec, uFlatPC, uFlagsAndType, uExitNo);//标志不同：重新初始化记录
        }
    }
    else if (pExitRec->enmAction == EMEXITACTION_FREE_RECORD)//空闲记录
    {
        STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecNew[0]);
        return emHistoryRecordInitNew(pVCpu, pHistEntry, idxSlot, pExitRec, uFlatPC, uFlagsAndType, uExitNo);//初始化新记录
    }
    else//冲突,在这个slot上已经有其他记录了
    {
        /*
         * Collision.  We calculate a new hash for stepping away from the first,
         * doing up to 8 steps away before replacing the least recently used record.
         */
        uintptr_t idxOldest     = idxSlot;
        uint64_t  uOldestExitNo = pExitRec->uLastExitNo;
        unsigned  iOldestStep   = 0;
        unsigned  iStep         = 1;
        uintptr_t const idxAdd  = (uintptr_t)(uFlatPC >> 11) & (EM_EXIT_RECORDS_IDX_MASK / 4);//二次探测法：步长由地址高位（>> 11）动态计算，减少聚集。
        /*
         * 发生哈希冲突时：
             采用二次探测法（最多探测8次）
             记录最久未使用的槽位(LRU策略)
             8次探测后替换最久未使用的记录(emHistoryRecordInitReplacement)
         * */
        for (;;)
        {
            Assert(iStep < RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecHits));
            AssertCompile(RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecNew)         == RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecHits));
            AssertCompile(RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecReplaced)    == RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecHits));
            AssertCompile(RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecTypeChanged) == RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecHits));

            /* Step to the next slot. */
            idxSlot += idxAdd;
            idxSlot &= EM_EXIT_RECORDS_IDX_MASK;
            pExitRec = &pVCpu->em.s.aExitRecords[idxSlot];

            /* Does it match? */
            if (pExitRec->uFlatPC == uFlatPC)
            {
                Assert(pExitRec->enmAction != EMEXITACTION_FREE_RECORD);
                pHistEntry->idxSlot = (uint32_t)idxSlot;
                if (pExitRec->uFlagsAndType == uFlagsAndType)
                {
                    pExitRec->uLastExitNo = uExitNo;
                    STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecHits[iStep]);//命中计数
                    break;
                }
                STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecTypeChanged[iStep]);
                return emHistoryRecordInit(pExitRec, uFlatPC, uFlagsAndType, uExitNo);
            }

            /* Is it free? */
            if (pExitRec->enmAction == EMEXITACTION_FREE_RECORD)
            {
                STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecNew[iStep]);//新记录计数
                return emHistoryRecordInitNew(pVCpu, pHistEntry, idxSlot, pExitRec, uFlatPC, uFlagsAndType, uExitNo);
            }

            /* Is it the least recently used one? */
            if (pExitRec->uLastExitNo < uOldestExitNo)
            {
                uOldestExitNo = pExitRec->uLastExitNo;
                idxOldest     = idxSlot;
                iOldestStep   = iStep;
            }

            /* Next iteration? */
            iStep++;
            Assert(iStep < RT_ELEMENTS(pVCpu->em.s.aStatHistoryRecReplaced));
            if (RT_LIKELY(iStep < 8 + 1))
            { /* likely */ }
            else
            {
                /* Replace the least recently used slot. */
                STAM_REL_COUNTER_INC(&pVCpu->em.s.aStatHistoryRecReplaced[iOldestStep]);//替换计数
                pExitRec = &pVCpu->em.s.aExitRecords[idxOldest];
                //LRU策略：在8次探测内记录最久未使用的槽位（uLastExitNo最小），最终替换它（emHistoryRecordInitReplacement）。
                return emHistoryRecordInitReplacement(pHistEntry, idxOldest, pExitRec, uFlatPC, uFlagsAndType, uExitNo);
            }
        }
    }

    /*
     * Found an existing record.
     */
    switch (pExitRec->enmAction)
    {
        case EMEXITACTION_NORMAL:
        {
            uint64_t const cHits = ++pExitRec->cHits;
            if (cHits < 256)//命中256次以下：保持EMEXITACTION_NORMAL
                return NULL;
            LogFlow(("emHistoryAddOrUpdateRecord: [%#x] %#07x %16RX64: -> EXEC_PROBE\n", idxSlot, uFlagsAndType, uFlatPC));
            pExitRec->enmAction = EMEXITACTION_EXEC_PROBE;//命中256次以上：升级为EMEXITACTION_EXEC_PROBE
            return pExitRec;
        }

        case EMEXITACTION_NORMAL_PROBED:
            pExitRec->cHits += 1;
            return NULL;

        default:
            pExitRec->cHits += 1;
            return pExitRec;

        /* This will happen if the caller ignores or cannot serve the probe
           request (forced to ring-3, whatever).  We retry this 256 times. */
        case EMEXITACTION_EXEC_PROBE:
        {
            uint64_t const cHits = ++pExitRec->cHits;
            if (cHits < 512)
                return pExitRec;
            //命中次数≥512时，降级为 NORMAL_PROBED（放弃优化，避免无效探测）。
            pExitRec->enmAction = EMEXITACTION_NORMAL_PROBED;
            LogFlow(("emHistoryAddOrUpdateRecord: [%#x] %#07x %16RX64: -> PROBED\n", idxSlot, uFlagsAndType, uFlatPC));
            return NULL;
        }
    }
}


/**
 * Adds an exit to the history for this CPU.
 *
 * @returns Pointer to an exit record if special action should be taken using
 *          EMHistoryExec().  Take normal exit action when NULL.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uFlagsAndType   Combined flags and type (see EMEXIT_MAKE_FT).
 * @param   uFlatPC         The flattened program counter (RIP).  UINT64_MAX if not available.
 * @param   uTimestamp      The TSC value for the exit, 0 if not available.
 * @thread  EMT(pVCpu)
 */
VMM_INT_DECL(PCEMEXITREC) EMHistoryAddExit(PVMCPUCC pVCpu, uint32_t uFlagsAndType, uint64_t uFlatPC, uint64_t uTimestamp)
{
    VMCPU_ASSERT_EMT(pVCpu);

    /*
     * Add the exit history entry.
     */
    AssertCompile(RT_ELEMENTS(pVCpu->em.s.aExitHistory) == 256);
    uint64_t uExitNo = pVCpu->em.s.iNextExit++;
    //将退出时的关键信息（如指令地址 uFlatPC、时间戳 uTimestamp、
    //退出类型 uFlagsAndType）存入环形缓冲区（aExitHistory）。
    PEMEXITENTRY pHistEntry = &pVCpu->em.s.aExitHistory[(uintptr_t)uExitNo & 0xff];
    pHistEntry->uFlatPC       = uFlatPC;
    pHistEntry->uTimestamp    = uTimestamp;
    pHistEntry->uFlagsAndType = uFlagsAndType;
    pHistEntry->idxSlot       = UINT32_MAX;

    /*
     * If common exit type, we will insert/update the exit into the exit record hash table.
     */
    //优化高频退出路径：如果退出类型符合优化条件（如 EMEXIT_F_KIND_EM），
    //则进一步调用 emHistoryAddOrUpdateRecord 更新哈希表，以减少未来相同路径的退出开销
    if (   (uFlagsAndType & (EMEXIT_F_KIND_MASK | EMEXIT_F_CS_EIP | EMEXIT_F_UNFLATTENED_PC)) == EMEXIT_F_KIND_EM
#ifdef IN_RING0
        && pVCpu->em.s.fExitOptimizationEnabledR0
        && ( !(uFlagsAndType & EMEXIT_F_HM) || pVCpu->em.s.fExitOptimizationEnabledR0PreemptDisabled)
#else
        && pVCpu->em.s.fExitOptimizationEnabled
#endif
        && uFlatPC != UINT64_MAX
       )
        return emHistoryAddOrUpdateRecord(pVCpu, uFlagsAndType, uFlatPC, pHistEntry, uExitNo);
    return NULL;
}


/**
 * Interface that VT-x uses to supply the PC of an exit when CS:RIP is being read.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uFlatPC         The flattened program counter (RIP).
 * @param   fFlattened      Set if RIP was subjected to CS.BASE, clear if not.
 */
//用于更新虚拟机退出(VMExit)历史记录中的程序计数器(PC)信息
VMM_INT_DECL(void) EMHistoryUpdatePC(PVMCPUCC pVCpu, uint64_t uFlatPC, bool fFlattened)
{
    VMCPU_ASSERT_EMT(pVCpu);

    AssertCompile(RT_ELEMENTS(pVCpu->em.s.aExitHistory) == 256);
    uint64_t     uExitNo    = pVCpu->em.s.iNextExit - 1;/// 获取最近一次退出序号
    PEMEXITENTRY pHistEntry = &pVCpu->em.s.aExitHistory[(uintptr_t)uExitNo & 0xff];
    pHistEntry->uFlatPC = uFlatPC;// 更新PC值
    if (fFlattened)
        pHistEntry->uFlagsAndType &= ~EMEXIT_F_UNFLATTENED_PC;//表示地址已平坦化
    else
        pHistEntry->uFlagsAndType |= EMEXIT_F_UNFLATTENED_PC;//表示原始地址
}


/**
 * Interface for convering a engine specific exit to a generic one and get guidance.
 *
 * @returns Pointer to an exit record if special action should be taken using
 *          EMHistoryExec().  Take normal exit action when NULL.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uFlagsAndType   Combined flags and type (see EMEXIT_MAKE_FLAGS_AND_TYPE).
 * @thread  EMT(pVCpu)
 */
VMM_INT_DECL(PCEMEXITREC) EMHistoryUpdateFlagsAndType(PVMCPUCC pVCpu, uint32_t uFlagsAndType)
{
    VMCPU_ASSERT_EMT(pVCpu);

    /*
     * Do the updating.
     */
    AssertCompile(RT_ELEMENTS(pVCpu->em.s.aExitHistory) == 256);
    uint64_t     uExitNo    = pVCpu->em.s.iNextExit - 1;
    PEMEXITENTRY pHistEntry = &pVCpu->em.s.aExitHistory[(uintptr_t)uExitNo & 0xff];
    pHistEntry->uFlagsAndType = uFlagsAndType | (pHistEntry->uFlagsAndType & (EMEXIT_F_CS_EIP | EMEXIT_F_UNFLATTENED_PC));

    /*
     * If common exit type, we will insert/update the exit into the exit record hash table.
     */
    if (   (uFlagsAndType & (EMEXIT_F_KIND_MASK | EMEXIT_F_CS_EIP | EMEXIT_F_UNFLATTENED_PC)) == EMEXIT_F_KIND_EM
#ifdef IN_RING0
        && pVCpu->em.s.fExitOptimizationEnabledR0
        && ( !(uFlagsAndType & EMEXIT_F_HM) || pVCpu->em.s.fExitOptimizationEnabledR0PreemptDisabled)
#else
        && pVCpu->em.s.fExitOptimizationEnabled
#endif
        && pHistEntry->uFlatPC != UINT64_MAX
       )
        return emHistoryAddOrUpdateRecord(pVCpu, uFlagsAndType, pHistEntry->uFlatPC, pHistEntry, uExitNo);
    return NULL;
}


/**
 * Interface for convering a engine specific exit to a generic one and get
 * guidance, supplying flattened PC too.
 *
 * @returns Pointer to an exit record if special action should be taken using
 *          EMHistoryExec().  Take normal exit action when NULL.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   uFlagsAndType   Combined flags and type (see EMEXIT_MAKE_FLAGS_AND_TYPE).
 * @param   uFlatPC         The flattened program counter (RIP).
 * @thread  EMT(pVCpu)
 */
VMM_INT_DECL(PCEMEXITREC) EMHistoryUpdateFlagsAndTypeAndPC(PVMCPUCC pVCpu, uint32_t uFlagsAndType, uint64_t uFlatPC)
{
    VMCPU_ASSERT_EMT(pVCpu);
    //Assert(uFlatPC != UINT64_MAX); - disable to make the pc wrapping tests in bs3-cpu-weird-1 work.

    /*
     * Do the updating.
     */
    AssertCompile(RT_ELEMENTS(pVCpu->em.s.aExitHistory) == 256);
    uint64_t     uExitNo    = pVCpu->em.s.iNextExit - 1;
    PEMEXITENTRY pHistEntry = &pVCpu->em.s.aExitHistory[(uintptr_t)uExitNo & 0xff];
    pHistEntry->uFlagsAndType = uFlagsAndType;
    pHistEntry->uFlatPC       = uFlatPC;

    /*
     * If common exit type, we will insert/update the exit into the exit record hash table.
     */
    if (   (uFlagsAndType & (EMEXIT_F_KIND_MASK | EMEXIT_F_CS_EIP | EMEXIT_F_UNFLATTENED_PC)) == EMEXIT_F_KIND_EM
#ifdef IN_RING0
        && pVCpu->em.s.fExitOptimizationEnabledR0
        && ( !(uFlagsAndType & EMEXIT_F_HM) || pVCpu->em.s.fExitOptimizationEnabledR0PreemptDisabled)
#else
        && pVCpu->em.s.fExitOptimizationEnabled
#endif
       )
        return emHistoryAddOrUpdateRecord(pVCpu, uFlagsAndType, uFlatPC, pHistEntry, uExitNo);
    return NULL;
}


/**
 * @callback_method_impl{FNDISREADBYTES}
 */
/*
 * 指令解码器（Disassembler）的辅助函数，用于 从客户机物理内存中安全读取指令字节流，主要用途包括：
     指令解码：为后续的指令模拟或分析提供原始字节数据。
     分页异常处理：处理客户机页表缺失或权限错误，必要时触发主机 TLB 无效化。
 * */
static DECLCALLBACK(int) emReadBytes(PDISSTATE pDis, uint8_t offInstr, uint8_t cbMinRead, uint8_t cbMaxRead)
{
    PVMCPUCC    pVCpu    = (PVMCPUCC)pDis->pvUser;
    RTUINTPTR   uSrcAddr = pDis->uInstrAddr + offInstr;

    /*
     * Figure how much we can or must read.
     */
    size_t      cbToRead = GUEST_PAGE_SIZE - (uSrcAddr & (GUEST_PAGE_SIZE - 1));//确保读取不跨页
    if (cbToRead > cbMaxRead)
        cbToRead = cbMaxRead;
    else if (cbToRead < cbMinRead)
        cbToRead = cbMinRead;

    /*
     * 底层读取：调用 PGMPhysSimpleReadGCPtr 通过客户机页表翻译物理地址。
       结果缓存：将读取的字节存入 pDis->Instr.ab 缓冲区，更新已缓存长度（cbCachedInstr）。
     * */
    int rc = PGMPhysSimpleReadGCPtr(pVCpu, &pDis->Instr.ab[offInstr], uSrcAddr, cbToRead);
    //若剩余空间不足 cbMinRead，强制读取最小所需字节（可能跨页）。
    if (RT_FAILURE(rc))
    {
        if (cbToRead > cbMinRead)
        {
            cbToRead = cbMinRead;
            rc = PGMPhysSimpleReadGCPtr(pVCpu, &pDis->Instr.ab[offInstr], uSrcAddr, cbToRead);
        }
        if (RT_FAILURE(rc))
        {
#if defined(VBOX_VMM_TARGET_ARMV8)
            AssertReleaseFailed();
#else
            /*
             * If we fail to find the page via the guest's page tables
             * we invalidate the page in the host TLB (pertaining to
             * the guest in the NestedPaging case). See @bugref{6043}.
             */
            if (rc == VERR_PAGE_TABLE_NOT_PRESENT || rc == VERR_PAGE_NOT_PRESENT)
            {
                HMInvalidatePage(pVCpu, uSrcAddr);// 无效化主机TLB
                if (((uSrcAddr + cbToRead - 1) >> GUEST_PAGE_SHIFT) != (uSrcAddr >> GUEST_PAGE_SHIFT))
                    HMInvalidatePage(pVCpu, uSrcAddr + cbToRead - 1);//跨页
            }
#endif
        }
    }

    pDis->cbCachedInstr = offInstr + (uint8_t)cbToRead;
    return rc;
}


/**
 * Disassembles the current instruction.
 *
 * @returns VBox status code, see SELMToFlatEx and EMInterpretDisasOneEx for
 *          details.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pDis            Where to return the parsed instruction info.
 * @param   pcbInstr        Where to return the instruction size. (optional)
 */
//虚拟化引擎中用于动态反汇编当前Guest指令的核心函数，其实现涉及地址转换、权限校验和指令解码三阶段处理流程
//获取当前Guest RIP指向的指令并进行反汇编
#if 0
  pDis：存储反汇编结果的结构体（操作码、操作数等）
  pcbInstr：返回指令长度
#endif
VMM_INT_DECL(int) EMInterpretDisasCurrent(PVMCPUCC pVCpu, PDISSTATE pDis, unsigned *pcbInstr)
{
#if defined(VBOX_VMM_TARGET_ARMV8)
    return EMInterpretDisasOneEx(pVCpu, (RTGCUINTPTR)CPUMGetGuestFlatPC(pVCpu), pDis, pcbInstr);
#else
    PCPUMCTX pCtx = CPUMQueryGuestCtxPtr(pVCpu);//获取当前vCPU的寄存器上下文
    RTGCPTR  GCPtrInstr;

# if 0
    int rc = SELMToFlatEx(pVCpu, DISSELREG_CS, pCtx, pCtx->rip, 0, &GCPtrInstr);
# else
/** @todo Get the CPU mode as well while we're at it! */
    /*
  CS段有效性（包括DPL/RPL权限检查）
  根据EFLAGS.VM标志判断虚拟8086模式5
  将逻辑地址CS:RIP转换为平坦地址GCPtrInstr
     * */
    int rc = SELMValidateAndConvertCSAddr(pVCpu, pCtx->eflags.u, pCtx->ss.Sel, pCtx->cs.Sel, &pCtx->cs, pCtx->rip, &GCPtrInstr);// 段地址转换
# endif
    if (RT_SUCCESS(rc))
        /*
         * (RTGCUINTPTR)GCPtrInstr：经过验证的指令物理地址
           pDis：输出反汇编结果（操作码、前缀、操作数等）
         * */
        return EMInterpretDisasOneEx(pVCpu, (RTGCUINTPTR)GCPtrInstr, pDis, pcbInstr);//反汇编核心调用

    Log(("EMInterpretDisasOne: Failed to convert %RTsel:%RGv (cpl=%d) - rc=%Rrc !!\n",
         pCtx->cs.Sel, (RTGCPTR)pCtx->rip, pCtx->ss.Sel & X86_SEL_RPL, rc));
    return rc;
#endif
}


/**
 * Disassembles one instruction.
 *
 * This is used by internally by the interpreter and by trap/access handlers.
 *
 * @returns VBox status code.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   GCPtrInstr      The flat address of the instruction.
 * @param   pDis            Where to return the parsed instruction info.
 * @param   pcbInstr        Where to return the instruction size. (optional)
 */
VMM_INT_DECL(int) EMInterpretDisasOneEx(PVMCPUCC pVCpu, RTGCUINTPTR GCPtrInstr, PDISSTATE pDis, unsigned *pcbInstr)
{
    DISCPUMODE enmCpuMode = CPUMGetGuestDisMode(pVCpu);
    /** @todo Deal with too long instruction (=> \#GP), opcode read errors (=>
     *        \#PF, \#GP, \#??), undefined opcodes (=> \#UD), and such. */
    int rc = DISInstrWithReader(GCPtrInstr, enmCpuMode, emReadBytes, pVCpu, pDis, pcbInstr);
    if (RT_SUCCESS(rc))
        return VINF_SUCCESS;
    AssertMsg(rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("DISCoreOne failed to GCPtrInstr=%RGv rc=%Rrc\n", GCPtrInstr, rc));
    return rc;
}


/**
 * Interprets the current instruction.
 *
 * @returns VBox status code.
 * @retval  VINF_*                  Scheduling instructions.
 * @retval  VERR_EM_INTERPRETER     Something we can't cope with.
 * @retval  VERR_*                  Fatal errors.
 *
 * @param   pVCpu       The cross context virtual CPU structure.
 *
 * @remark  Invalid opcode exceptions have a higher priority than \#GP (see
 *          Intel Architecture System Developers Manual, Vol 3, 5.5) so we don't
 *          need to worry about e.g. invalid modrm combinations (!)
 */
VMM_INT_DECL(VBOXSTRICTRC) EMInterpretInstruction(PVMCPUCC pVCpu)
{
#if defined(VBOX_VMM_TARGET_ARMV8)
    LogFlow(("EMInterpretInstruction %RGv\n", (RTGCPTR)CPUMGetGuestFlatPC(pVCpu)));
#else
    LogFlow(("EMInterpretInstruction %RGv\n", (RTGCPTR)CPUMGetGuestRIP(pVCpu)));
#endif

    VBOXSTRICTRC rc = IEMExecOneBypassEx(pVCpu, NULL /*pcbWritten*/);
    if (RT_UNLIKELY(   rc == VERR_IEM_ASPECT_NOT_IMPLEMENTED
                    || rc == VERR_IEM_INSTR_NOT_IMPLEMENTED))
        rc = VERR_EM_INTERPRETER;//模拟失败，需切换至原始模拟模式
    if (rc != VINF_SUCCESS)
        Log(("EMInterpretInstruction: returns %Rrc\n", VBOXSTRICTRC_VAL(rc)));

    return rc;
}


/**
 * Interprets the current instruction using the supplied DISSTATE structure.
 *
 * IP/EIP/RIP *IS* updated!
 *
 * @returns VBox strict status code.
 * @retval  VINF_*                  Scheduling instructions. When these are returned, it
 *                                  starts to get a bit tricky to know whether code was
 *                                  executed or not... We'll address this when it becomes a problem.
 * @retval  VERR_EM_INTERPRETER     Something we can't cope with.
 * @retval  VERR_*                  Fatal errors.
 *
 * @param   pVCpu       The cross context virtual CPU structure of the calling EMT.
 * @param   pDis        The disassembler cpu state for the instruction to be
 *                      interpreted.
 * @param   rip         The instruction pointer value.
 *
 * @remark  Invalid opcode exceptions have a higher priority than GP (see Intel
 *          Architecture System Developers Manual, Vol 3, 5.5) so we don't need
 *          to worry about e.g. invalid modrm combinations (!)
 *
 * @todo    At this time we do NOT check if the instruction overwrites vital information.
 *          Make sure this can't happen!! (will add some assertions/checks later)
 */
//pDis	PDISSTATE	预解码指令状态(含操作码/操作数等)
//用于 基于预解码状态（DISSTATE）执行单条客户机指令
/*
 * 实际工作流程示例
场景：客户机执行一条 CPUID 指令。
指令预读取：
  前置调用 emReadBytes 将 CPUID 的字节码（0F A2）存入 pDis->Instr.ab。
解释执行：
  IEMExecOneBypassWithPrefetchedByPC 识别 CPUID，模拟其行为（返回虚拟 CPU 信息）。
成功返回：
  若模拟成功，返回 VINF_SUCCESS。
错误处理：
  若 IEM 未实现该指令（如新扩展指令），返回 VERR_EM_INTERPRETER，触发上层回退。
 * */
VMM_INT_DECL(VBOXSTRICTRC) EMInterpretInstructionDisasState(PVMCPUCC pVCpu, PDISSTATE pDis, uint64_t rip)
{
    LogFlow(("EMInterpretInstructionDisasState %RGv\n", (RTGCPTR)rip));

    //直接解释执行指令。
    VBOXSTRICTRC rc = IEMExecOneBypassWithPrefetchedByPC(pVCpu, rip, pDis->Instr.ab, pDis->cbCachedInstr);//直接使用预解码的指令字节(pDis->Instr.ab)执行，避免重复解码
    if (RT_UNLIKELY(   rc == VERR_IEM_ASPECT_NOT_IMPLEMENTED
                    || rc == VERR_IEM_INSTR_NOT_IMPLEMENTED))
        rc = VERR_EM_INTERPRETER;//统一错误码, 重新编译或陷入 Ring-3,显式处理未实现指令，避免隐式行为。

    if (rc != VINF_SUCCESS)
        Log(("EMInterpretInstructionDisasState: returns %Rrc\n", VBOXSTRICTRC_VAL(rc)));

    return rc;
}

