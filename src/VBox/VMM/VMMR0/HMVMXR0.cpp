/* $Id$ */
/** @file
 * HM VMX (Intel VT-x) - Host Context Ring-0.
 */

/*
 * Copyright (C) 2012-2024 Oracle and/or its affiliates.
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
#define LOG_GROUP LOG_GROUP_HM
#define VMCPU_INCL_CPUM_GST_CTX
#include <iprt/x86.h>
#include <iprt/asm-amd64-x86.h>
#include <iprt/thread.h>
#include <iprt/mem.h>
#include <iprt/mp.h>

#include <VBox/vmm/pdmapi.h>
#include <VBox/vmm/dbgf.h>
#include <VBox/vmm/iem.h>
#include <VBox/vmm/iom.h>
#include <VBox/vmm/tm.h>
#include <VBox/vmm/em.h>
#include <VBox/vmm/gcm.h>
#include <VBox/vmm/gim.h>
#include <VBox/vmm/apic.h>
#include "HMInternal.h"
#include <VBox/vmm/vmcc.h>
#include <VBox/vmm/hmvmxinline.h>
#include "HMVMXR0.h"
#include "VMXInternal.h"
#include "dtrace/VBoxVMM.h"

/*
    1. 实现VMX根模式（Root Mode）与非根模式（Non-Root Mode）的切换，管理虚拟机的运行状态
    2. 处理VMExit事件（如I/O操作、外部中断等），将控制权交还VMM进行模拟或调度
    3. 配置和维护VMCS（Virtual Machine Control Structure），存储虚拟化上下文信息（如寄存器状态、退出原因）
*/

/*********************************************************************************************************************************
*   Defined Constants And Macros                                                                                                 *
*********************************************************************************************************************************/
#ifdef DEBUG_ramshankar
# define HMVMX_ALWAYS_SAVE_GUEST_RFLAGS
# define HMVMX_ALWAYS_SAVE_RO_GUEST_STATE
# define HMVMX_ALWAYS_SAVE_FULL_GUEST_STATE
# define HMVMX_ALWAYS_SYNC_FULL_GUEST_STATE
# define HMVMX_ALWAYS_CLEAN_TRANSIENT
# define HMVMX_ALWAYS_CHECK_GUEST_STATE
# define HMVMX_ALWAYS_TRAP_ALL_XCPTS
# define HMVMX_ALWAYS_TRAP_PF
# define HMVMX_ALWAYS_FLUSH_TLB
# define HMVMX_ALWAYS_SWAP_EFER
#endif

/** Enables the fAlwaysInterceptMovDRx related code. */
#define VMX_WITH_MAYBE_ALWAYS_INTERCEPT_MOV_DRX 1


/*********************************************************************************************************************************
*   Structures and Typedefs                                                                                                      *
*********************************************************************************************************************************/
/**
 * VMX page allocation information.
 */
typedef struct
{
    uint32_t    fValid;       /**< Whether to allocate this page (e.g, based on a CPU feature). */
    uint32_t    uPadding0;    /**< Padding to ensure array of these structs are aligned to a multiple of 8. */
    PRTHCPHYS   pHCPhys;      /**< Where to store the host-physical address of the allocation. */
    PRTR0PTR    ppVirt;       /**< Where to store the host-virtual address of the allocation. */
} VMXPAGEALLOCINFO;
/** Pointer to VMX page-allocation info. */
typedef VMXPAGEALLOCINFO *PVMXPAGEALLOCINFO;
/** Pointer to a const VMX page-allocation info. */
typedef const VMXPAGEALLOCINFO *PCVMXPAGEALLOCINFO;
AssertCompileSizeAlignment(VMXPAGEALLOCINFO, 8);


/*********************************************************************************************************************************
*   Internal Functions                                                                                                           *
*********************************************************************************************************************************/
static bool     hmR0VmxShouldSwapEferMsr(PCVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient);
static int      hmR0VmxExitHostNmi(PVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo);


/*********************************************************************************************************************************
*   Global Variables                                                                                                             *
*********************************************************************************************************************************/
/** The DR6 value after writing zero to the register.
 * Set by VMXR0GlobalInit(). */
static uint64_t g_fDr6Zeroed = 0;


/**
 * Checks if the given MSR is part of the lastbranch-from-IP MSR stack.
 * @returns @c true if it's part of LBR stack, @c false otherwise.
 *
 * @param   pVM         The cross context VM structure.
 * @param   idMsr       The MSR.
 * @param   pidxMsr     Where to store the index of the MSR in the LBR MSR array.
 *                      Optional, can be NULL.
 *
 * @remarks Must only be called when LBR is enabled.
 */
DECL_FORCE_INLINE(bool) hmR0VmxIsLbrBranchFromMsr(PCVMCC pVM, uint32_t idMsr, uint32_t *pidxMsr)
{
    Assert(pVM->hmr0.s.vmx.fLbr);
    Assert(pVM->hmr0.s.vmx.idLbrFromIpMsrFirst);
    uint32_t const cLbrStack = pVM->hmr0.s.vmx.idLbrFromIpMsrLast - pVM->hmr0.s.vmx.idLbrFromIpMsrFirst + 1;
    uint32_t const idxMsr    = idMsr - pVM->hmr0.s.vmx.idLbrFromIpMsrFirst;
    if (idxMsr < cLbrStack)
    {
        if (pidxMsr)
            *pidxMsr = idxMsr;
        return true;
    }
    return false;
}


/**
 * Checks if the given MSR is part of the lastbranch-to-IP MSR stack.
 * @returns @c true if it's part of LBR stack, @c false otherwise.
 *
 * @param   pVM         The cross context VM structure.
 * @param   idMsr       The MSR.
 * @param   pidxMsr     Where to store the index of the MSR in the LBR MSR array.
 *                      Optional, can be NULL.
 *
 * @remarks Must only be called when LBR is enabled and when lastbranch-to-IP MSRs
 *          are supported by the CPU (see hmR0VmxSetupLbrMsrRange).
 */
DECL_FORCE_INLINE(bool) hmR0VmxIsLbrBranchToMsr(PCVMCC pVM, uint32_t idMsr, uint32_t *pidxMsr)
{
    Assert(pVM->hmr0.s.vmx.fLbr);
    if (pVM->hmr0.s.vmx.idLbrToIpMsrFirst)
    {
        uint32_t const cLbrStack = pVM->hmr0.s.vmx.idLbrToIpMsrLast - pVM->hmr0.s.vmx.idLbrToIpMsrFirst + 1;
        uint32_t const idxMsr    = idMsr - pVM->hmr0.s.vmx.idLbrToIpMsrFirst;
        if (idxMsr < cLbrStack)
        {
            if (pidxMsr)
                *pidxMsr = idxMsr;
            return true;
        }
    }
    return false;
}


/**
 * Gets the active (in use) VMCS info. object for the specified VCPU.
 *
 * This is either the guest or nested-guest VMCS info. and need not necessarily
 * pertain to the "current" VMCS (in the VMX definition of the term). For instance,
 * if the VM-entry failed due to an invalid-guest state, we may have "cleared" the
 * current VMCS while returning to ring-3. However, the VMCS info. object for that
 * VMCS would still be active and returned here so that we could dump the VMCS
 * fields to ring-3 for diagnostics. This function is thus only used to
 * distinguish between the nested-guest or guest VMCS.
 *
 * @returns The active VMCS information.
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @thread  EMT.
 * @remarks This function may be called with preemption or interrupts disabled!
 */
DECLINLINE(PVMXVMCSINFO) hmGetVmxActiveVmcsInfo(PVMCPUCC pVCpu)
{
    if (!pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs)
        return &pVCpu->hmr0.s.vmx.VmcsInfo;
    return &pVCpu->hmr0.s.vmx.VmcsInfoNstGst;
}


/**
 * Returns whether the VM-exit MSR-store area differs from the VM-exit MSR-load
 * area.
 *
 * @returns @c true if it's different, @c false otherwise.
 * @param   pVmcsInfo   The VMCS info. object.
 */
DECL_FORCE_INLINE(bool) hmR0VmxIsSeparateExitMsrStoreAreaVmcs(PCVMXVMCSINFO pVmcsInfo)
{
    return RT_BOOL(   pVmcsInfo->pvGuestMsrStore != pVmcsInfo->pvGuestMsrLoad
                   && pVmcsInfo->pvGuestMsrStore);
}


/**
 * Sets the given Processor-based VM-execution controls.
 *
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   uProcCtls       The Processor-based VM-execution controls to set.
 */
static void hmR0VmxSetProcCtlsVmcs(PVMXTRANSIENT pVmxTransient, uint32_t uProcCtls)
{
    PVMXVMCSINFO pVmcsInfo = pVmxTransient->pVmcsInfo;
    if ((pVmcsInfo->u32ProcCtls & uProcCtls) != uProcCtls)
    {
        pVmcsInfo->u32ProcCtls |= uProcCtls;
        int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC, pVmcsInfo->u32ProcCtls);
        AssertRC(rc);
    }
}


/**
 * Removes the given Processor-based VM-execution controls.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   uProcCtls       The Processor-based VM-execution controls to remove.
 *
 * @remarks When executing a nested-guest, this will not remove any of the specified
 *          controls if the nested hypervisor has set any one of them.
 */
static void hmR0VmxRemoveProcCtlsVmcs(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient, uint32_t uProcCtls)
{
    PVMXVMCSINFO pVmcsInfo = pVmxTransient->pVmcsInfo;
    if (pVmcsInfo->u32ProcCtls & uProcCtls)
    {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        if (   !pVmxTransient->fIsNestedGuest
            || !CPUMIsGuestVmxProcCtlsSet(&pVCpu->cpum.GstCtx, uProcCtls))
#else
        NOREF(pVCpu);
        if (!pVmxTransient->fIsNestedGuest)
#endif
        {
            pVmcsInfo->u32ProcCtls &= ~uProcCtls;
            int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC, pVmcsInfo->u32ProcCtls);
            AssertRC(rc);
        }
    }
}


/**
 * Sets the TSC offset for the current VMCS.
 *
 * @param   uTscOffset  The TSC offset to set.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static void hmR0VmxSetTscOffsetVmcs(PVMXVMCSINFO pVmcsInfo, uint64_t uTscOffset)
{
    if (pVmcsInfo->u64TscOffset != uTscOffset)
    {
        int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_TSC_OFFSET_FULL, uTscOffset);
        AssertRC(rc);
        pVmcsInfo->u64TscOffset = uTscOffset;
    }
}


/**
 * Loads the VMCS specified by the VMCS info. object.
 *
 * @returns VBox status code.
 * @param   pVmcsInfo       The VMCS info. object.
 *
 * @remarks Can be called with interrupts disabled.
 */
static int hmR0VmxLoadVmcs(PVMXVMCSINFO pVmcsInfo)
{
    Assert(pVmcsInfo->HCPhysVmcs != 0 && pVmcsInfo->HCPhysVmcs != NIL_RTHCPHYS);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    return VMXLoadVmcs(pVmcsInfo->HCPhysVmcs);
}


/**
 * Clears the VMCS specified by the VMCS info. object.
 *
 * @returns VBox status code.
 * @param   pVmcsInfo   The VMCS info. object.
 *
 * @remarks Can be called with interrupts disabled.
 */
static int hmR0VmxClearVmcs(PVMXVMCSINFO pVmcsInfo)
{
    Assert(pVmcsInfo->HCPhysVmcs != 0 && pVmcsInfo->HCPhysVmcs != NIL_RTHCPHYS);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    int rc = VMXClearVmcs(pVmcsInfo->HCPhysVmcs);
    if (RT_SUCCESS(rc))
        pVmcsInfo->fVmcsState = VMX_V_VMCS_LAUNCH_STATE_CLEAR;
    return rc;
}


/**
 * Checks whether the MSR belongs to the set of guest MSRs that we restore
 * lazily while leaving VT-x.
 *
 * @returns true if it does, false otherwise.
 * @param   pVCpu   The cross context virtual CPU structure.
 * @param   idMsr   The MSR to check.
 */
static bool hmR0VmxIsLazyGuestMsr(PCVMCPUCC pVCpu, uint32_t idMsr)
{
    if (pVCpu->CTX_SUFF(pVM)->hmr0.s.fAllow64BitGuests)
    {
        switch (idMsr)
        {
            case MSR_K8_LSTAR:
            case MSR_K6_STAR:
            case MSR_K8_SF_MASK:
            case MSR_K8_KERNEL_GS_BASE:
                return true;
        }
    }
    return false;
}


/**
 * Loads a set of guests MSRs to allow read/passthru to the guest.
 *
 * The name of this function is slightly confusing. This function does NOT
 * postpone loading, but loads the MSR right now. "hmR0VmxLazy" is simply a
 * common prefix for functions dealing with "lazy restoration" of the shared
 * MSRs.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
/*
减少冗余MSR写入：通过惰性加载机制避免每次VM Entry/Exit时不必要的MSR切换
安全验证：确保Guest MSR值符合CPU规范（如地址必须为Canonical Form）
性能优化：通过条件判断跳过已匹配的MSR写入操作
 * */
/*
MSR_K8_KERNEL_GS_BASE	内核GS基址寄存器（用于快速切换内核/用户空间）
MSR_K8_LSTAR	系统调用目标地址（SYSCALL指令跳转地址）
MSR_K6_STAR	系统调用CS/SS选择器（SYSCALL/SYSRET时的段寄存器设置）
MSR_K8_SF_MASK	系统调用标志掩码（控制RFLAGS的哪些位在SYSCALL时被清除）
 * */
static void hmR0VmxLazyLoadGuestMsrs(PVMCPUCC pVCpu)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));

    Assert(pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_SAVED_HOST);
    if (pVCpu->CTX_SUFF(pVM)->hmr0.s.fAllow64BitGuests)
    {
        /*
         * If the guest MSRs are not loaded -and- if all the guest MSRs are identical
         * to the MSRs on the CPU (which are the saved host MSRs, see assertion above) then
         * we can skip a few MSR writes.
         *
         * Otherwise, it implies either 1. they're not loaded, or 2. they're loaded but the
         * guest MSR values in the guest-CPU context might be different to what's currently
         * loaded in the CPU. In either case, we need to write the new guest MSR values to the
         * CPU, see @bugref{8728}.
         */
        //触发条件：
          //Guest MSR未加载（!VMX_LAZY_MSRS_LOADED_GUEST）
          //Guest MSR值与Host MSR值完全一致
          //避免4次MSR写入（约节省100+ CPU周期/次）
        PCCPUMCTX pCtx = &pVCpu->cpum.GstCtx;
        if (   !(pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_LOADED_GUEST)
            && pCtx->msrKERNELGSBASE == pVCpu->hmr0.s.vmx.u64HostMsrKernelGsBase
            && pCtx->msrLSTAR        == pVCpu->hmr0.s.vmx.u64HostMsrLStar
            && pCtx->msrSTAR         == pVCpu->hmr0.s.vmx.u64HostMsrStar
            && pCtx->msrSFMASK       == pVCpu->hmr0.s.vmx.u64HostMsrSfMask)
        {
#ifdef VBOX_STRICT
            Assert(ASMRdMsr(MSR_K8_KERNEL_GS_BASE) == pCtx->msrKERNELGSBASE);
            Assert(ASMRdMsr(MSR_K8_LSTAR)          == pCtx->msrLSTAR);
            Assert(ASMRdMsr(MSR_K6_STAR)           == pCtx->msrSTAR);
            Assert(ASMRdMsr(MSR_K8_SF_MASK)        == pCtx->msrSFMASK);
#endif
        }
        else
        {
            /* Avoid raising #GP caused by writing illegal values to these MSRs. */
            //验证MSR值符合Canonical Address规范（x64地址规范）
            if (   X86_IS_CANONICAL(pCtx->msrKERNELGSBASE)
                && X86_IS_CANONICAL(pCtx->msrLSTAR)) //对SFMASK进行32位掩码处理（防止非法值触发#GP异常）
            {
                ASMWrMsr(MSR_K8_KERNEL_GS_BASE, pCtx->msrKERNELGSBASE);
                ASMWrMsr(MSR_K8_LSTAR,          pCtx->msrLSTAR);
                ASMWrMsr(MSR_K6_STAR,           pCtx->msrSTAR);
                /* The system call flag mask register isn't as benign and accepting of all
                   values as the above, so mask it to avoid #GP'ing on corrupted input. */
                Assert(!(pCtx->msrSFMASK & ~(uint64_t)UINT32_MAX));
                ASMWrMsr(MSR_K8_SF_MASK,        pCtx->msrSFMASK & UINT32_MAX);
            }
            else
                AssertMsgFailed(("Incompatible lazily-loaded guest MSR values\n"));
        }
    }
    pVCpu->hmr0.s.vmx.fLazyMsrs |= VMX_LAZY_MSRS_LOADED_GUEST;
}


/**
 * Checks if the specified guest MSR is part of the VM-entry MSR-load area.
 *
 * @returns @c true if found, @c false otherwise.
 * @param   pVmcsInfo   The VMCS info. object.
 * @param   idMsr       The MSR to find.
 */
static bool hmR0VmxIsAutoLoadGuestMsr(PCVMXVMCSINFO pVmcsInfo, uint32_t idMsr)
{
    PCVMXAUTOMSR   pMsrs = (PCVMXAUTOMSR)pVmcsInfo->pvGuestMsrLoad;
    uint32_t const cMsrs = pVmcsInfo->cEntryMsrLoad;
    Assert(pMsrs);
    Assert(sizeof(*pMsrs) * cMsrs <= X86_PAGE_4K_SIZE);
    for (uint32_t i = 0; i < cMsrs; i++)
    {
        if (pMsrs[i].u32Msr == idMsr)
            return true;
    }
    return false;
}


/**
 * Performs lazy restoration of the set of host MSRs if they were previously
 * loaded with guest MSR values.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 * @remarks The guest MSRs should have been saved back into the guest-CPU
 *          context by vmxHCImportGuestState()!!!
 */
static void hmR0VmxLazyRestoreHostMsrs(PVMCPUCC pVCpu)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));

    if (pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_LOADED_GUEST)
    {
        Assert(pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_SAVED_HOST);
        if (pVCpu->CTX_SUFF(pVM)->hmr0.s.fAllow64BitGuests)
        {
            ASMWrMsr(MSR_K8_LSTAR,          pVCpu->hmr0.s.vmx.u64HostMsrLStar);
            ASMWrMsr(MSR_K6_STAR,           pVCpu->hmr0.s.vmx.u64HostMsrStar);
            ASMWrMsr(MSR_K8_SF_MASK,        pVCpu->hmr0.s.vmx.u64HostMsrSfMask);
            ASMWrMsr(MSR_K8_KERNEL_GS_BASE, pVCpu->hmr0.s.vmx.u64HostMsrKernelGsBase);
        }
    }
    pVCpu->hmr0.s.vmx.fLazyMsrs &= ~(VMX_LAZY_MSRS_LOADED_GUEST | VMX_LAZY_MSRS_SAVED_HOST);
}


/**
 * Sets pfnStartVm to the best suited variant.
 *
 * This must be called whenever anything changes relative to the hmR0VmXStartVm
 * variant selection:
 *      - pVCpu->hm.s.fLoadSaveGuestXcr0
 *      - HM_WSF_IBPB_ENTRY in pVCpu->hmr0.s.fWorldSwitcher
 *      - HM_WSF_IBPB_EXIT  in pVCpu->hmr0.s.fWorldSwitcher
 *      - Perhaps: CPUMIsGuestFPUStateActive() (windows only)
 *      - Perhaps: CPUMCTX.fXStateMask (windows only)
 *
 * We currently ASSUME that neither HM_WSF_IBPB_ENTRY nor HM_WSF_IBPB_EXIT
 * cannot be changed at runtime.
 */
//HMR0A.asm
//BEGINPROC RT_CONCAT(hmR0VmxStartVm,%1)中实现
static void hmR0VmxUpdateStartVmFunction(PVMCPUCC pVCpu)
{
    static const struct CLANGWORKAROUND { PFNHMVMXSTARTVM pfn; } s_aHmR0VmxStartVmFunctions[] =
    {
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_SansL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_SansL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_SansL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_SansL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_WithL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_WithL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_WithL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_WithL1dEntry_SansMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_SansL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_SansL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_SansL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_SansL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_WithL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_WithL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_WithL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_WithL1dEntry_WithMdsEntry_SansIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_SansL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_SansL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_SansL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_SansL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_WithL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_WithL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_WithL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_WithL1dEntry_SansMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_SansL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_SansL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_SansL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_SansL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_SansIbpbEntry_WithL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_SansIbpbEntry_WithL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_SansXcr0_WithIbpbEntry_WithL1dEntry_WithMdsEntry_WithIbpbExit },
        { hmR0VmxStartVm_WithXcr0_WithIbpbEntry_WithL1dEntry_WithMdsEntry_WithIbpbExit },
    };
    //5 个二进制标志位（XCR0、IBPB_ENTRY、L1D_ENTRY、MDS_ENTRY、IBPB_EXIT）组合成 2^5 = 32 种可能。
    uintptr_t const idx = (pVCpu->hmr0.s.fLoadSaveGuestXcr0                 ?  1 : 0)
                        | (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_IBPB_ENTRY ?  2 : 0)
                        | (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_L1D_ENTRY  ?  4 : 0)
                        | (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_MDS_ENTRY  ?  8 : 0)
                        | (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_IBPB_EXIT  ? 16 : 0);
    PFNHMVMXSTARTVM const pfnStartVm = s_aHmR0VmxStartVmFunctions[idx].pfn;
    if (pVCpu->hmr0.s.vmx.pfnStartVm != pfnStartVm)
        pVCpu->hmr0.s.vmx.pfnStartVm = pfnStartVm;
}


/**
 * Pushes a 2-byte value onto the real-mode (in virtual-8086 mode) guest's
 * stack.
 *
 * @returns Strict VBox status code (i.e. informational status codes too).
 * @retval  VINF_EM_RESET if pushing a value to the stack caused a triple-fault.
 * @param   pVCpu   The cross context virtual CPU structure.
 * @param   uValue  The value to push to the guest stack.
 */
static VBOXSTRICTRC hmR0VmxRealModeGuestStackPush(PVMCPUCC pVCpu, uint16_t uValue)
{
    /*
     * The stack limit is 0xffff in real-on-virtual 8086 mode. Real-mode with weird stack limits cannot be run in
     * virtual 8086 mode in VT-x. See Intel spec. 26.3.1.2 "Checks on Guest Segment Registers".
     * See Intel Instruction reference for PUSH and Intel spec. 22.33.1 "Segment Wraparound".
     */
    PCPUMCTX pCtx = &pVCpu->cpum.GstCtx;
    if (pCtx->sp == 1)
        return VINF_EM_RESET;
    pCtx->sp -= sizeof(uint16_t);       /* May wrap around which is expected behaviour. */
    int rc = PGMPhysSimpleWriteGCPhys(pVCpu->CTX_SUFF(pVM), pCtx->ss.u64Base + pCtx->sp, &uValue, sizeof(uint16_t));
    AssertRC(rc);
    return rc;
}


/**
 * Wrapper around VMXWriteVmcs16 taking a pVCpu parameter so VCC doesn't complain about
 * unreferenced local parameters in the template code...
 */
DECL_FORCE_INLINE(int) hmR0VmxWriteVmcs16(PCVMCPUCC pVCpu, uint32_t uFieldEnc, uint16_t u16Val)
{
    RT_NOREF(pVCpu);
    return VMXWriteVmcs16(uFieldEnc, u16Val);
}


/**
 * Wrapper around VMXWriteVmcs32 taking a pVCpu parameter so VCC doesn't complain about
 * unreferenced local parameters in the template code...
 */
DECL_FORCE_INLINE(int) hmR0VmxWriteVmcs32(PCVMCPUCC pVCpu, uint32_t uFieldEnc, uint32_t u32Val)
{
    RT_NOREF(pVCpu);
    return VMXWriteVmcs32(uFieldEnc, u32Val);
}


/**
 * Wrapper around VMXWriteVmcs64 taking a pVCpu parameter so VCC doesn't complain about
 * unreferenced local parameters in the template code...
 */
DECL_FORCE_INLINE(int) hmR0VmxWriteVmcs64(PCVMCPUCC pVCpu, uint32_t uFieldEnc, uint64_t u64Val)
{
    RT_NOREF(pVCpu);
    return VMXWriteVmcs64(uFieldEnc, u64Val);
}


/**
 * Wrapper around VMXReadVmcs16 taking a pVCpu parameter so VCC doesn't complain about
 * unreferenced local parameters in the template code...
 */
DECL_FORCE_INLINE(int) hmR0VmxReadVmcs16(PCVMCPUCC pVCpu, uint32_t uFieldEnc, uint16_t *pu16Val)
{
    RT_NOREF(pVCpu);
    return VMXReadVmcs16(uFieldEnc, pu16Val);
}


/**
 * Wrapper around VMXReadVmcs32 taking a pVCpu parameter so VCC doesn't complain about
 * unreferenced local parameters in the template code...
 */
DECL_FORCE_INLINE(int) hmR0VmxReadVmcs32(PCVMCPUCC pVCpu, uint32_t uFieldEnc, uint32_t *pu32Val)
{
    RT_NOREF(pVCpu);
    return VMXReadVmcs32(uFieldEnc, pu32Val);
}


/**
 * Wrapper around VMXReadVmcs64 taking a pVCpu parameter so VCC doesn't complain about
 * unreferenced local parameters in the template code...
 */
DECL_FORCE_INLINE(int) hmR0VmxReadVmcs64(PCVMCPUCC pVCpu, uint32_t uFieldEnc, uint64_t *pu64Val)
{
    RT_NOREF(pVCpu);
    return VMXReadVmcs64(uFieldEnc, pu64Val);
}


/*
 * Instantiate the code we share with the NEM darwin backend.
 */
#define VCPU_2_VMXSTATE(a_pVCpu)            (a_pVCpu)->hm.s
#define VCPU_2_VMXSTATS(a_pVCpu)            (a_pVCpu)->hm.s

#define VM_IS_VMX_UNRESTRICTED_GUEST(a_pVM) (a_pVM)->hmr0.s.vmx.fUnrestrictedGuest
#define VM_IS_VMX_NESTED_PAGING(a_pVM)      (a_pVM)->hmr0.s.fNestedPaging
#define VM_IS_VMX_PREEMPT_TIMER_USED(a_pVM) (a_pVM)->hmr0.s.vmx.fUsePreemptTimer
#define VM_IS_VMX_LBR(a_pVM)                (a_pVM)->hmr0.s.vmx.fLbr

#define VMX_VMCS_WRITE_16(a_pVCpu, a_FieldEnc, a_Val) hmR0VmxWriteVmcs16((a_pVCpu), (a_FieldEnc), (a_Val))
#define VMX_VMCS_WRITE_32(a_pVCpu, a_FieldEnc, a_Val) hmR0VmxWriteVmcs32((a_pVCpu), (a_FieldEnc), (a_Val))
#define VMX_VMCS_WRITE_64(a_pVCpu, a_FieldEnc, a_Val) hmR0VmxWriteVmcs64((a_pVCpu), (a_FieldEnc), (a_Val))
#define VMX_VMCS_WRITE_NW(a_pVCpu, a_FieldEnc, a_Val) hmR0VmxWriteVmcs64((a_pVCpu), (a_FieldEnc), (a_Val))

#define VMX_VMCS_READ_16(a_pVCpu, a_FieldEnc, a_pVal) hmR0VmxReadVmcs16((a_pVCpu), (a_FieldEnc), (a_pVal))
#define VMX_VMCS_READ_32(a_pVCpu, a_FieldEnc, a_pVal) hmR0VmxReadVmcs32((a_pVCpu), (a_FieldEnc), (a_pVal))
#define VMX_VMCS_READ_64(a_pVCpu, a_FieldEnc, a_pVal) hmR0VmxReadVmcs64((a_pVCpu), (a_FieldEnc), (a_pVal))
#define VMX_VMCS_READ_NW(a_pVCpu, a_FieldEnc, a_pVal) hmR0VmxReadVmcs64((a_pVCpu), (a_FieldEnc), (a_pVal))

#include "../VMMAll/VMXAllTemplate.cpp.h"

#undef VMX_VMCS_WRITE_16
#undef VMX_VMCS_WRITE_32
#undef VMX_VMCS_WRITE_64
#undef VMX_VMCS_WRITE_NW

#undef VMX_VMCS_READ_16
#undef VMX_VMCS_READ_32
#undef VMX_VMCS_READ_64
#undef VMX_VMCS_READ_NW

#undef VM_IS_VMX_PREEMPT_TIMER_USED
#undef VM_IS_VMX_NESTED_PAGING
#undef VM_IS_VMX_UNRESTRICTED_GUEST
#undef VCPU_2_VMXSTATS
#undef VCPU_2_VMXSTATE


/**
 * Updates the VM's last error record.
 *
 * If there was a VMX instruction error, reads the error data from the VMCS and
 * updates VCPU's last error record as well.
 *
 * @param   pVCpu   The cross context virtual CPU structure of the calling EMT.
 *                  Can be NULL if @a rc is not VERR_VMX_UNABLE_TO_START_VM or
 *                  VERR_VMX_INVALID_VMCS_FIELD.
 * @param   rc      The error code.
 */
static void hmR0VmxUpdateErrorRecord(PVMCPUCC pVCpu, int rc)
{
    if (   rc == VERR_VMX_INVALID_VMCS_FIELD
        || rc == VERR_VMX_UNABLE_TO_START_VM)
    {
        AssertPtrReturnVoid(pVCpu);
        VMXReadVmcs32(VMX_VMCS32_RO_VM_INSTR_ERROR, &pVCpu->hm.s.vmx.LastError.u32InstrError);
    }
    pVCpu->CTX_SUFF(pVM)->hm.s.ForR3.rcInit = rc;
}


/**
 * Enters VMX root mode operation on the current CPU.
 *
 * @returns VBox status code.
 * @param   pHostCpu        The HM physical-CPU structure.
 * @param   pVM             The cross context VM structure. Can be
 *                          NULL, after a resume.
 * @param   HCPhysCpuPage   Physical address of the VMXON region.
 * @param   pvCpuPage       Pointer to the VMXON region.
 */
static int hmR0VmxEnterRootMode(PHMPHYSCPU pHostCpu, PVMCC pVM, RTHCPHYS HCPhysCpuPage, void *pvCpuPage)
{
    Assert(pHostCpu);
    Assert(HCPhysCpuPage && HCPhysCpuPage != NIL_RTHCPHYS);
    Assert(RT_ALIGN_T(HCPhysCpuPage, _4K, RTHCPHYS) == HCPhysCpuPage); // 必须 4KB 对齐
    Assert(pvCpuPage);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD)); // 必须禁用抢占

    if (pVM)
    {
        /* Write the VMCS revision identifier to the VMXON region. */
        *(uint32_t *)pvCpuPage = RT_BF_GET(g_HmMsrs.u.vmx.u64Basic, VMX_BF_BASIC_VMCS_ID);
    }

    /* Paranoid: Disable interrupts as, in theory, interrupt handlers might mess with CR4. */
    RTCCUINTREG const fEFlags = ASMIntDisableFlags();//禁用中断

    /* Enable the VMX bit in CR4 if necessary. */
    RTCCUINTREG const uOldCr4 = SUPR0ChangeCR4(X86_CR4_VMXE, RTCCUINTREG_MAX);// use asm to 设置 CR4.VMXE 位（启用 VT-x）。

    /* Record whether VMXE was already prior to us enabling it above. */
    pHostCpu->fVmxeAlreadyEnabled = RT_BOOL(uOldCr4 & X86_CR4_VMXE); 

    /* Enter VMX root mode. */
    int rc = VMXEnable(HCPhysCpuPage); //VMXON 
    if (RT_FAILURE(rc))
    {
        /* Restore CR4.VMXE if it was not set prior to our attempt to set it above. */
        if (!pHostCpu->fVmxeAlreadyEnabled)
            SUPR0ChangeCR4(0 /* fOrMask */, ~(uint64_t)X86_CR4_VMXE);

        if (pVM)
            pVM->hm.s.ForR3.vmx.HCPhysVmxEnableError = HCPhysCpuPage;
    }

    /* Restore interrupts. */
    ASMSetFlags(fEFlags);
    return rc;
}


/**
 * Exits VMX root mode operation on the current CPU.
 *
 * @returns VBox status code.
 * @param   pHostCpu        The HM physical-CPU structure.
 */
static int hmR0VmxLeaveRootMode(PHMPHYSCPU pHostCpu)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    /* Paranoid: Disable interrupts as, in theory, interrupts handlers might mess with CR4. */
    RTCCUINTREG const fEFlags = ASMIntDisableFlags();

    /* If we're for some reason not in VMX root mode, then don't leave it. */
    RTCCUINTREG const uHostCr4 = ASMGetCR4();

    int rc;
    if (uHostCr4 & X86_CR4_VMXE)
    {
        /* Exit VMX root mode and clear the VMX bit in CR4. */
        VMXDisable();

        /* Clear CR4.VMXE only if it was clear prior to use setting it. */
        if (!pHostCpu->fVmxeAlreadyEnabled)
            SUPR0ChangeCR4(0 /* fOrMask */, ~(uint64_t)X86_CR4_VMXE);

        rc = VINF_SUCCESS;
    }
    else
        rc = VERR_VMX_NOT_IN_VMX_ROOT_MODE;

    /* Restore interrupts. */
    ASMSetFlags(fEFlags);
    return rc;
}


/**
 * Allocates pages specified as specified by an array of VMX page allocation info
 * objects.
 *
 * The pages contents are zero'd after allocation.
 *
 * @returns VBox status code.
 * @param   phMemObj        Where to return the handle to the allocation.
 * @param   paAllocInfo     The pointer to the first element of the VMX
 *                          page-allocation info object array.
 * @param   cEntries        The number of elements in the @a paAllocInfo array.
 */
static int hmR0VmxPagesAllocZ(PRTR0MEMOBJ phMemObj, PVMXPAGEALLOCINFO paAllocInfo, uint32_t cEntries)
{
    *phMemObj = NIL_RTR0MEMOBJ;

    /* Figure out how many pages to allocate. */
    uint32_t cPages = 0;
    for (uint32_t iPage = 0; iPage < cEntries; iPage++)
        cPages += !!paAllocInfo[iPage].fValid;

    /* Allocate the pages. */
    if (cPages)
    {
        size_t const cbPages = cPages << HOST_PAGE_SHIFT;
        int rc = RTR0MemObjAllocPage(phMemObj, cbPages, false /* fExecutable */);
        if (RT_FAILURE(rc))
            return rc;

        /* Zero the contents and assign each page to the corresponding VMX page-allocation entry. */
        void *pvFirstPage = RTR0MemObjAddress(*phMemObj);
        RT_BZERO(pvFirstPage, cbPages);

        uint32_t iPage = 0;
        for (uint32_t i = 0; i < cEntries; i++)
            if (paAllocInfo[i].fValid)
            {
                RTHCPHYS const HCPhysPage = RTR0MemObjGetPagePhysAddr(*phMemObj, iPage);
                void          *pvPage     = (void *)((uintptr_t)pvFirstPage + (iPage << X86_PAGE_4K_SHIFT));
                Assert(HCPhysPage && HCPhysPage != NIL_RTHCPHYS);
                AssertPtr(pvPage);

                Assert(paAllocInfo[iPage].pHCPhys);
                Assert(paAllocInfo[iPage].ppVirt);
                *paAllocInfo[iPage].pHCPhys = HCPhysPage;
                *paAllocInfo[iPage].ppVirt  = pvPage;

                /* Move to next page. */
                ++iPage;
            }

        /* Make sure all valid (requested) pages have been assigned. */
        Assert(iPage == cPages);
    }
    return VINF_SUCCESS;
}


/**
 * Frees pages allocated using hmR0VmxPagesAllocZ.
 *
 * @param   phMemObj    Pointer to the memory object handle.  Will be set to
 *                      NIL.
 */
DECL_FORCE_INLINE(void) hmR0VmxPagesFree(PRTR0MEMOBJ phMemObj)
{
    /* We can cleanup wholesale since it's all one allocation. */
    if (*phMemObj != NIL_RTR0MEMOBJ)
    {
        RTR0MemObjFree(*phMemObj, true /* fFreeMappings */);
        *phMemObj = NIL_RTR0MEMOBJ;
    }
}


/**
 * Initializes a VMCS info. object.
 *
 * @param   pVmcsInfo           The VMCS info. object.
 * @param   pVmcsInfoShared     The VMCS info. object shared with ring-3.
 */
static void hmR0VmxVmcsInfoInit(PVMXVMCSINFO pVmcsInfo, PVMXVMCSINFOSHARED pVmcsInfoShared)
{
    RT_ZERO(*pVmcsInfo);
    RT_ZERO(*pVmcsInfoShared);

    pVmcsInfo->pShared             = pVmcsInfoShared;
    Assert(pVmcsInfo->hMemObj == NIL_RTR0MEMOBJ);
    pVmcsInfo->HCPhysVmcs          = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysShadowVmcs    = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysMsrBitmap     = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysGuestMsrLoad  = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysGuestMsrStore = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysHostMsrLoad   = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysVirtApic      = NIL_RTHCPHYS;
    pVmcsInfo->HCPhysEPTP          = NIL_RTHCPHYS;
    pVmcsInfo->u64VmcsLinkPtr      = NIL_RTHCPHYS;
    pVmcsInfo->idHostCpuState      = NIL_RTCPUID;
    pVmcsInfo->idHostCpuExec       = NIL_RTCPUID;
}


/**
 * Frees the VT-x structures for a VMCS info. object.
 *
 * @param   pVmcsInfo           The VMCS info. object.
 * @param   pVmcsInfoShared     The VMCS info. object shared with ring-3.
 */
static void hmR0VmxVmcsInfoFree(PVMXVMCSINFO pVmcsInfo, PVMXVMCSINFOSHARED pVmcsInfoShared)
{
    hmR0VmxPagesFree(&pVmcsInfo->hMemObj);
    hmR0VmxVmcsInfoInit(pVmcsInfo, pVmcsInfoShared);
}


/**
 * Allocates the VT-x structures for a VMCS info. object.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmcsInfo       The VMCS info. object.
 * @param   fIsNstGstVmcs   Whether this is a nested-guest VMCS.
 *
 * @remarks The caller is expected to take care of any and all allocation failures.
 *          This function will not perform any cleanup for failures half-way
 *          through.
 */
/*
根据处理器特性决定是否需要MSR位图和影子VMCS
分配多个VMX相关的内存页面，包括：
  VMCS本身
  客户机MSR加载区域
  主机MSR加载区域
  MSR位图(如果需要)
  影子VMCS(如果需要)
将VM-entry MSR加载区域和VM-exit MSR存储区域设置为同一物理页面
如果需要TPR(任务优先级寄存器)影子，则获取虚拟APIC页面
 * */
static int hmR0VmxAllocVmcsInfo(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo, bool fIsNstGstVmcs)
{
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);

    bool const fMsrBitmaps = RT_BOOL(g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_MSR_BITMAPS);
    bool const fShadowVmcs = !fIsNstGstVmcs ? pVM->hmr0.s.vmx.fUseVmcsShadowing : pVM->cpum.ro.GuestFeatures.fVmxVmcsShadowing;
    Assert(!pVM->cpum.ro.GuestFeatures.fVmxVmcsShadowing);  /* VMCS shadowing is not yet exposed to the guest. */
    //条目包含分配标志、物理地址指针和虚拟地址指针
    VMXPAGEALLOCINFO aAllocInfo[] =
    {
        { true,        0 /* Unused */, &pVmcsInfo->HCPhysVmcs,         &pVmcsInfo->pvVmcs         },// VMCS主体
        { true,        0 /* Unused */, &pVmcsInfo->HCPhysGuestMsrLoad, &pVmcsInfo->pvGuestMsrLoad },// Guest MSR加载区
        { true,        0 /* Unused */, &pVmcsInfo->HCPhysHostMsrLoad,  &pVmcsInfo->pvHostMsrLoad  },// Host MSR加载区
        { fMsrBitmaps, 0 /* Unused */, &pVmcsInfo->HCPhysMsrBitmap,    &pVmcsInfo->pvMsrBitmap    },// MSR位图
        { fShadowVmcs, 0 /* Unused */, &pVmcsInfo->HCPhysShadowVmcs,   &pVmcsInfo->pvShadowVmcs   },// 影子VMCS
    };

    //批量分配所有需要的页面
    int rc = hmR0VmxPagesAllocZ(&pVmcsInfo->hMemObj, &aAllocInfo[0], RT_ELEMENTS(aAllocInfo));
    if (RT_FAILURE(rc))
        return rc;

    /*
     * We use the same page for VM-entry MSR-load and VM-exit MSR store areas.
     * Because they contain a symmetric list of guest MSRs to load on VM-entry and store on VM-exit.
     */
    AssertCompile(RT_ELEMENTS(aAllocInfo) > 0);
    Assert(pVmcsInfo->HCPhysGuestMsrLoad != NIL_RTHCPHYS);
    //将Guest MSR的加载区和存储区设为同一物理页面
    pVmcsInfo->pvGuestMsrStore     = pVmcsInfo->pvGuestMsrLoad;
    pVmcsInfo->HCPhysGuestMsrStore = pVmcsInfo->HCPhysGuestMsrLoad;

    /*
     * Get the virtual-APIC page rather than allocating them again.
     */
    if (g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_TPR_SHADOW)
    {
        if (!fIsNstGstVmcs)
        {
            if (PDMHasApic(pVM))
            {
                rc = APICGetApicPageForCpu(pVCpu, &pVmcsInfo->HCPhysVirtApic, (PRTR0PTR)&pVmcsInfo->pbVirtApic, NULL /*pR3Ptr*/);
                if (RT_FAILURE(rc))
                    return rc;
                Assert(pVmcsInfo->pbVirtApic);
                Assert(pVmcsInfo->HCPhysVirtApic && pVmcsInfo->HCPhysVirtApic != NIL_RTHCPHYS);
            }
        }
        else
        {
            /* These are setup later while marging the nested-guest VMCS. */
            Assert(pVmcsInfo->pbVirtApic == NULL);
            Assert(pVmcsInfo->HCPhysVirtApic == NIL_RTHCPHYS);
        }
    }

    return VINF_SUCCESS;
}


/**
 * Free all VT-x structures for the VM.
 *
 * @param   pVM     The cross context VM structure.
 */
static void hmR0VmxStructsFree(PVMCC pVM)
{
    hmR0VmxPagesFree(&pVM->hmr0.s.vmx.hMemObj);
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    if (pVM->hmr0.s.vmx.fUseVmcsShadowing)
    {
        RTMemFree(pVM->hmr0.s.vmx.paShadowVmcsFields);
        pVM->hmr0.s.vmx.paShadowVmcsFields = NULL;
        RTMemFree(pVM->hmr0.s.vmx.paShadowVmcsRoFields);
        pVM->hmr0.s.vmx.paShadowVmcsRoFields = NULL;
    }
#endif

    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PVMCPUCC pVCpu = VMCC_GET_CPU(pVM, idCpu);
        hmR0VmxVmcsInfoFree(&pVCpu->hmr0.s.vmx.VmcsInfo, &pVCpu->hm.s.vmx.VmcsInfo);
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        if (pVM->cpum.ro.GuestFeatures.fVmx)
            hmR0VmxVmcsInfoFree(&pVCpu->hmr0.s.vmx.VmcsInfoNstGst, &pVCpu->hm.s.vmx.VmcsInfoNstGst);
#endif
    }
}


/**
 * Allocate all VT-x structures for the VM.
 *
 * @returns IPRT status code.
 * @param   pVM     The cross context VM structure.
 *
 * @remarks This functions will cleanup on memory allocation failures.
 */
static int hmR0VmxStructsAlloc(PVMCC pVM)
{
    /*
     * Sanity check the VMCS size reported by the CPU as we assume 4KB allocations.
     * The VMCS size cannot be more than 4096 bytes.
     *
     * See Intel spec. Appendix A.1 "Basic VMX Information".
     */
    uint32_t const cbVmcs = RT_BF_GET(g_HmMsrs.u.vmx.u64Basic, VMX_BF_BASIC_VMCS_SIZE);
    if (cbVmcs <= X86_PAGE_4K_SIZE)
    { /* likely */ }
    else
    {
        VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_INVALID_VMCS_SIZE;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    /*
     * Allocate per-VM VT-x structures.
     */
    bool const fVirtApicAccess   = RT_BOOL(g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS);
    bool const fUseVmcsShadowing = pVM->hmr0.s.vmx.fUseVmcsShadowing;
    VMXPAGEALLOCINFO aAllocInfo[] =
    {
        //APIC访问页面：当启用虚拟APIC访问时(fVirtApicAccess)，会分配物理页面用于加速APIC寄存器访问
        { fVirtApicAccess,   0 /* Unused */, &pVM->hmr0.s.vmx.HCPhysApicAccess,    (PRTR0PTR)&pVM->hmr0.s.vmx.pbApicAccess },
        //当位图中某位被置1时，对应VMCS字段的访问将不会触发VM Exit，而是由硬件自动处理
        //控制VMREAD指令的监控
        { fUseVmcsShadowing, 0 /* Unused */, &pVM->hmr0.s.vmx.HCPhysVmreadBitmap,  &pVM->hmr0.s.vmx.pvVmreadBitmap         },
        //控制VMWRITE指令的监控,这些位图用于VMCS影子功能，可减少VM-exit次数
        { fUseVmcsShadowing, 0 /* Unused */, &pVM->hmr0.s.vmx.HCPhysVmwriteBitmap, &pVM->hmr0.s.vmx.pvVmwriteBitmap        },
#ifdef VBOX_WITH_CRASHDUMP_MAGIC
        //在启用崩溃转储魔法值(VBOX_WITH_CRASHDUMP_MAGIC)时，会分配临时内存区域用于调试信息存储
        { true,              0 /* Unused */, &pVM->hmr0.s.vmx.HCPhysScratch,       (PRTR0PTR)&pVM->hmr0.s.vmx.pbScratch    },
#endif
    };

    int rc = hmR0VmxPagesAllocZ(&pVM->hmr0.s.vmx.hMemObj, &aAllocInfo[0], RT_ELEMENTS(aAllocInfo));
    if (RT_SUCCESS(rc))
    {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        /* Allocate the shadow VMCS-fields array. */
        if (fUseVmcsShadowing)
        {
            Assert(!pVM->hmr0.s.vmx.cShadowVmcsFields);
            Assert(!pVM->hmr0.s.vmx.cShadowVmcsRoFields);
            pVM->hmr0.s.vmx.paShadowVmcsFields   = (uint32_t *)RTMemAllocZ(sizeof(g_aVmcsFields));
            pVM->hmr0.s.vmx.paShadowVmcsRoFields = (uint32_t *)RTMemAllocZ(sizeof(g_aVmcsFields));
            if (!pVM->hmr0.s.vmx.paShadowVmcsFields || !pVM->hmr0.s.vmx.paShadowVmcsRoFields)
                rc = VERR_NO_MEMORY;
        }
#endif

        /*
         * Allocate per-VCPU VT-x structures.
         */
        for (VMCPUID idCpu = 0; idCpu < pVM->cCpus && RT_SUCCESS(rc); idCpu++)
        {
            /* Allocate the guest VMCS structures. */
            PVMCPUCC pVCpu = VMCC_GET_CPU(pVM, idCpu);
            rc = hmR0VmxAllocVmcsInfo(pVCpu, &pVCpu->hmr0.s.vmx.VmcsInfo, false /* fIsNstGstVmcs */);

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
            /* Allocate the nested-guest VMCS structures, when the VMX feature is exposed to the guest. */
            if (pVM->cpum.ro.GuestFeatures.fVmx && RT_SUCCESS(rc))
                rc = hmR0VmxAllocVmcsInfo(pVCpu, &pVCpu->hmr0.s.vmx.VmcsInfoNstGst, true /* fIsNstGstVmcs */);
#endif
        }
        if (RT_SUCCESS(rc))
            return VINF_SUCCESS;
    }
    hmR0VmxStructsFree(pVM);
    return rc;
}


/**
 * Pre-initializes non-zero fields in VMX structures that will be allocated.
 *
 * @param   pVM     The cross context VM structure.
 */
static void hmR0VmxStructsInit(PVMCC pVM)
{
    /* Paranoia. */
    Assert(pVM->hmr0.s.vmx.pbApicAccess == NULL);
#ifdef VBOX_WITH_CRASHDUMP_MAGIC
    Assert(pVM->hmr0.s.vmx.pbScratch == NULL);
#endif

    /*
     * Initialize members up-front so we can cleanup en masse on allocation failures.
     */
#ifdef VBOX_WITH_CRASHDUMP_MAGIC
    pVM->hmr0.s.vmx.HCPhysScratch       = NIL_RTHCPHYS;
#endif
    pVM->hmr0.s.vmx.HCPhysApicAccess    = NIL_RTHCPHYS;
    pVM->hmr0.s.vmx.HCPhysVmreadBitmap  = NIL_RTHCPHYS;
    pVM->hmr0.s.vmx.HCPhysVmwriteBitmap = NIL_RTHCPHYS;
    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PVMCPUCC pVCpu = VMCC_GET_CPU(pVM, idCpu);
        hmR0VmxVmcsInfoInit(&pVCpu->hmr0.s.vmx.VmcsInfo,       &pVCpu->hm.s.vmx.VmcsInfo);
        hmR0VmxVmcsInfoInit(&pVCpu->hmr0.s.vmx.VmcsInfoNstGst, &pVCpu->hm.s.vmx.VmcsInfoNstGst);
    }
}

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
/**
 * Returns whether an MSR at the given MSR-bitmap offset is intercepted or not.
 *
 * @returns @c true if the MSR is intercepted, @c false otherwise.
 * @param   pbMsrBitmap     The MSR bitmap.
 * @param   offMsr          The MSR byte offset.
 * @param   iBit            The bit offset from the byte offset.
 */
DECLINLINE(bool) hmR0VmxIsMsrBitSet(uint8_t const *pbMsrBitmap, uint16_t offMsr, int32_t iBit)
{
    Assert(offMsr + (iBit >> 3) <= X86_PAGE_4K_SIZE);
    return ASMBitTest(pbMsrBitmap, (offMsr << 3) + iBit);
}
#endif

/**
 * Sets the permission bits for the specified MSR in the given MSR bitmap.
 *
 * If the passed VMCS is a nested-guest VMCS, this function ensures that the
 * read/write intercept is cleared from the MSR bitmap used for hardware-assisted
 * VMX execution of the nested-guest, only if nested-guest is also not intercepting
 * the read/write access of this MSR.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmcsInfo       The VMCS info. object.
 * @param   fIsNstGstVmcs   Whether this is a nested-guest VMCS.
 * @param   idMsr           The MSR value.
 * @param   fMsrpm          The MSR permissions (see VMXMSRPM_XXX). This must
 *                          include both a read -and- a write permission!
 *
 * @sa      CPUMGetVmxMsrPermission.
 * @remarks Can be called with interrupts disabled.
 */
static void hmR0VmxSetMsrPermission(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo, bool fIsNstGstVmcs, uint32_t idMsr, uint32_t fMsrpm)
{
    uint8_t *pbMsrBitmap = (uint8_t *)pVmcsInfo->pvMsrBitmap;
    Assert(pbMsrBitmap);
    Assert(VMXMSRPM_IS_FLAG_VALID(fMsrpm));

    /*
     * MSR-bitmap Layout:
     *   Byte index            MSR range            Interpreted as
     * 0x000 - 0x3ff    0x00000000 - 0x00001fff    Low MSR read bits.
     * 0x400 - 0x7ff    0xc0000000 - 0xc0001fff    High MSR read bits.
     * 0x800 - 0xbff    0x00000000 - 0x00001fff    Low MSR write bits.
     * 0xc00 - 0xfff    0xc0000000 - 0xc0001fff    High MSR write bits.
     *
     * A bit corresponding to an MSR within the above range causes a VM-exit
     * if the bit is 1 on executions of RDMSR/WRMSR.  If an MSR falls out of
     * the MSR range, it always cause a VM-exit.
     *
     * See Intel spec. 24.6.9 "MSR-Bitmap Address".
     */
    uint16_t const offBitmapRead  = 0;
    uint16_t const offBitmapWrite = 0x800;
    uint16_t       offMsr;
    int32_t        iBit;
    if (idMsr <= UINT32_C(0x00001fff))
    {
        offMsr = 0;
        iBit   = idMsr;
    }
    else if (idMsr - UINT32_C(0xc0000000) <= UINT32_C(0x00001fff))
    {
        offMsr = 0x400;
        iBit   = idMsr - UINT32_C(0xc0000000);
    }
    else
        AssertMsgFailedReturnVoid(("Invalid MSR %#RX32\n", idMsr));

    /*
     * Set the MSR read permission.
     */
    uint16_t const offMsrRead = offBitmapRead + offMsr;
    Assert(offMsrRead + (iBit >> 3) < offBitmapWrite);
    if (fMsrpm & VMXMSRPM_ALLOW_RD)
    {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        bool const fClear = !fIsNstGstVmcs ? true
                          : !hmR0VmxIsMsrBitSet(pVCpu->cpum.GstCtx.hwvirt.vmx.abMsrBitmap, offMsrRead, iBit);
#else
        RT_NOREF2(pVCpu, fIsNstGstVmcs);
        bool const fClear = true;
#endif
        if (fClear)
            ASMBitClear(pbMsrBitmap, (offMsrRead << 3) + iBit);
    }
    else
        ASMBitSet(pbMsrBitmap, (offMsrRead << 3) + iBit);

    /*
     * Set the MSR write permission.
     */
    uint16_t const offMsrWrite = offBitmapWrite + offMsr;
    Assert(offMsrWrite + (iBit >> 3) < X86_PAGE_4K_SIZE);
    if (fMsrpm & VMXMSRPM_ALLOW_WR)
    {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        bool const fClear = !fIsNstGstVmcs ? true
                          : !hmR0VmxIsMsrBitSet(pVCpu->cpum.GstCtx.hwvirt.vmx.abMsrBitmap, offMsrWrite, iBit);
#else
        RT_NOREF2(pVCpu, fIsNstGstVmcs);
        bool const fClear = true;
#endif
        if (fClear)
            ASMBitClear(pbMsrBitmap, (offMsrWrite << 3) + iBit);
    }
    else
        ASMBitSet(pbMsrBitmap, (offMsrWrite << 3) + iBit);
}


/**
 * Updates the VMCS with the number of effective MSRs in the auto-load/store MSR
 * area.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 * @param   cMsrs       The number of MSRs.
 */
static int hmR0VmxSetAutoLoadStoreMsrCount(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo, uint32_t cMsrs)
{
    /* Shouldn't ever happen but there -is- a number. We're well within the recommended 512. */
    uint32_t const cMaxSupportedMsrs = VMX_MISC_MAX_MSRS(g_HmMsrs.u.vmx.u64Misc);
    if (RT_LIKELY(cMsrs < cMaxSupportedMsrs))
    {
        /* Commit the MSR counts to the VMCS and update the cache. */
        if (pVmcsInfo->cEntryMsrLoad != cMsrs)
        {
            int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_ENTRY_MSR_LOAD_COUNT, cMsrs);   AssertRC(rc);
            rc     = VMXWriteVmcs32(VMX_VMCS32_CTRL_EXIT_MSR_STORE_COUNT, cMsrs);   AssertRC(rc);
            rc     = VMXWriteVmcs32(VMX_VMCS32_CTRL_EXIT_MSR_LOAD_COUNT,  cMsrs);   AssertRC(rc);
            pVmcsInfo->cEntryMsrLoad = cMsrs;
            pVmcsInfo->cExitMsrStore = cMsrs;
            pVmcsInfo->cExitMsrLoad  = cMsrs;
        }
        return VINF_SUCCESS;
    }

    LogRel(("Auto-load/store MSR count exceeded! cMsrs=%u MaxSupported=%u\n", cMsrs, cMaxSupportedMsrs));
    pVCpu->hm.s.u32HMError = VMX_UFC_INSUFFICIENT_GUEST_MSR_STORAGE;
    return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
}


/**
 * Adds a new (or updates the value of an existing) guest/host MSR
 * pair to be swapped during the world-switch as part of the
 * auto-load/store MSR area in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   idMsr           The MSR.
 * @param   uGuestMsrValue  Value of the guest MSR.
 * @param   fSetReadWrite   Whether to set the guest read/write access of this
 *                          MSR (thus not causing a VM-exit).
 * @param   fUpdateHostMsr  Whether to update the value of the host MSR if
 *                          necessary.
 */
static int hmR0VmxAddAutoLoadStoreMsr(PVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient, uint32_t idMsr, uint64_t uGuestMsrValue,
                                      bool fSetReadWrite, bool fUpdateHostMsr)
{
    PVMXVMCSINFO  pVmcsInfo     = pVmxTransient->pVmcsInfo;
    bool const      fIsNstGstVmcs = pVmxTransient->fIsNestedGuest;
    PVMXAUTOMSR     pGuestMsrLoad = (PVMXAUTOMSR)pVmcsInfo->pvGuestMsrLoad;
    uint32_t        cMsrs         = pVmcsInfo->cEntryMsrLoad;
    uint32_t        i;

    /* Paranoia. */
    Assert(pGuestMsrLoad);

#ifndef DEBUG_bird
    LogFlowFunc(("pVCpu=%p idMsr=%#RX32 uGuestMsrValue=%#RX64\n", pVCpu, idMsr, uGuestMsrValue));
#endif

    /* Check if the MSR already exists in the VM-entry MSR-load area. */
    for (i = 0; i < cMsrs; i++)
    {
        if (pGuestMsrLoad[i].u32Msr == idMsr)
            break;
    }

    bool fAdded = false;
    if (i == cMsrs)
    {
        /* The MSR does not exist, bump the MSR count to make room for the new MSR. */
        ++cMsrs;
        int rc = hmR0VmxSetAutoLoadStoreMsrCount(pVCpu, pVmcsInfo, cMsrs);
        AssertMsgRCReturn(rc, ("Insufficient space to add MSR to VM-entry MSR-load/store area %u\n", idMsr), rc);

        /* Set the guest to read/write this MSR without causing VM-exits. */
        if (   fSetReadWrite
            && (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS))
            hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, fIsNstGstVmcs, idMsr, VMXMSRPM_ALLOW_RD_WR);

        Log4Func(("Added MSR %#RX32, cMsrs=%u\n", idMsr, cMsrs));
        fAdded = true;
    }

    /* Update the MSR value for the newly added or already existing MSR. */
    pGuestMsrLoad[i].u32Msr   = idMsr;
    pGuestMsrLoad[i].u64Value = uGuestMsrValue;

    /* Create the corresponding slot in the VM-exit MSR-store area if we use a different page. */
    if (hmR0VmxIsSeparateExitMsrStoreAreaVmcs(pVmcsInfo))
    {
        PVMXAUTOMSR pGuestMsrStore = (PVMXAUTOMSR)pVmcsInfo->pvGuestMsrStore;
        pGuestMsrStore[i].u32Msr   = idMsr;
        pGuestMsrStore[i].u64Value = uGuestMsrValue;
    }

    /* Update the corresponding slot in the host MSR area. */
    PVMXAUTOMSR pHostMsr = (PVMXAUTOMSR)pVmcsInfo->pvHostMsrLoad;
    Assert(pHostMsr != pVmcsInfo->pvGuestMsrLoad);
    Assert(pHostMsr != pVmcsInfo->pvGuestMsrStore);
    pHostMsr[i].u32Msr = idMsr;

    /*
     * Only if the caller requests to update the host MSR value AND we've newly added the
     * MSR to the host MSR area do we actually update the value. Otherwise, it will be
     * updated by hmR0VmxUpdateAutoLoadHostMsrs().
     *
     * We do this for performance reasons since reading MSRs may be quite expensive.
     */
    if (fAdded)
    {
        if (fUpdateHostMsr)
        {
            Assert(!VMMRZCallRing3IsEnabled(pVCpu));
            Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
            pHostMsr[i].u64Value = ASMRdMsr(idMsr);
        }
        else
        {
            /* Someone else can do the work. */
            pVCpu->hmr0.s.vmx.fUpdatedHostAutoMsrs = false;
        }
    }
    return VINF_SUCCESS;
}


/**
 * Removes a guest/host MSR pair to be swapped during the world-switch from the
 * auto-load/store MSR area in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   idMsr           The MSR.
 */
static int hmR0VmxRemoveAutoLoadStoreMsr(PVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient, uint32_t idMsr)
{
    PVMXVMCSINFO  pVmcsInfo     = pVmxTransient->pVmcsInfo;
    bool const      fIsNstGstVmcs = pVmxTransient->fIsNestedGuest;
    PVMXAUTOMSR     pGuestMsrLoad = (PVMXAUTOMSR)pVmcsInfo->pvGuestMsrLoad;
    uint32_t        cMsrs         = pVmcsInfo->cEntryMsrLoad;

#ifndef DEBUG_bird
    LogFlowFunc(("pVCpu=%p idMsr=%#RX32\n", pVCpu, idMsr));
#endif

    for (uint32_t i = 0; i < cMsrs; i++)
    {
        /* Find the MSR. */
        if (pGuestMsrLoad[i].u32Msr == idMsr)
        {
            /*
             * If it's the last MSR, we only need to reduce the MSR count.
             * If it's -not- the last MSR, copy the last MSR in place of it and reduce the MSR count.
             */
            if (i < cMsrs - 1)
            {
                /* Remove it from the VM-entry MSR-load area. */
                pGuestMsrLoad[i].u32Msr   = pGuestMsrLoad[cMsrs - 1].u32Msr;
                pGuestMsrLoad[i].u64Value = pGuestMsrLoad[cMsrs - 1].u64Value;

                /* Remove it from the VM-exit MSR-store area if it's in a different page. */
                if (hmR0VmxIsSeparateExitMsrStoreAreaVmcs(pVmcsInfo))
                {
                    PVMXAUTOMSR pGuestMsrStore = (PVMXAUTOMSR)pVmcsInfo->pvGuestMsrStore;
                    Assert(pGuestMsrStore[i].u32Msr == idMsr);
                    pGuestMsrStore[i].u32Msr   = pGuestMsrStore[cMsrs - 1].u32Msr;
                    pGuestMsrStore[i].u64Value = pGuestMsrStore[cMsrs - 1].u64Value;
                }

                /* Remove it from the VM-exit MSR-load area. */
                PVMXAUTOMSR pHostMsr = (PVMXAUTOMSR)pVmcsInfo->pvHostMsrLoad;
                Assert(pHostMsr[i].u32Msr == idMsr);
                pHostMsr[i].u32Msr   = pHostMsr[cMsrs - 1].u32Msr;
                pHostMsr[i].u64Value = pHostMsr[cMsrs - 1].u64Value;
            }

            /* Reduce the count to reflect the removed MSR and bail. */
            --cMsrs;
            break;
        }
    }

    /* Update the VMCS if the count changed (meaning the MSR was found and removed). */
    if (cMsrs != pVmcsInfo->cEntryMsrLoad)
    {
        int rc = hmR0VmxSetAutoLoadStoreMsrCount(pVCpu, pVmcsInfo, cMsrs);
        AssertRCReturn(rc, rc);

        /* We're no longer swapping MSRs during the world-switch, intercept guest read/writes to them. */
        if (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS)
            hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, fIsNstGstVmcs, idMsr, VMXMSRPM_EXIT_RD | VMXMSRPM_EXIT_WR);

        Log4Func(("Removed MSR %#RX32, cMsrs=%u\n", idMsr, cMsrs));
        return VINF_SUCCESS;
    }

    return VERR_NOT_FOUND;
}


/**
 * Updates the value of all host MSRs in the VM-exit MSR-load area.
 *
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 *
 * @remarks No-long-jump zone!!!
 */
//更新 Host（宿主机）MSR 自动加载区域
static void hmR0VmxUpdateAutoLoadHostMsrs(PCVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo)
{
    RT_NOREF(pVCpu);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    //指向 Host MSR 自动加载区域的指针（VMCS 中配置的 MSR 列表）。
    PVMXAUTOMSR pHostMsrLoad = (PVMXAUTOMSR)pVmcsInfo->pvHostMsrLoad;
    //需要更新的 MSR 数量（从 VMCS 中读取的 cExitMsrLoad）。
    uint32_t const cMsrs     = pVmcsInfo->cExitMsrLoad;
    Assert(pHostMsrLoad);
    Assert(sizeof(*pHostMsrLoad) * cMsrs <= X86_PAGE_4K_SIZE);
    LogFlowFunc(("pVCpu=%p cMsrs=%u\n", pVCpu, cMsrs));
    for (uint32_t i = 0; i < cMsrs; i++)
    {
        /*
         * Performance hack for the host EFER MSR. We use the cached value rather than re-read it.
         * Strict builds will catch mismatches in hmR0VmxCheckAutoLoadStoreMsrs(). See @bugref{7368}.
         */
        //优化点：
           //使用 缓存的 Host EFER MSR 值（g_uHmVmxHostMsrEfer）减少 RDMSR 开销。
        if (pHostMsrLoad[i].u32Msr == MSR_K6_EFER)
            pHostMsrLoad[i].u64Value = g_uHmVmxHostMsrEfer;
        else
            pHostMsrLoad[i].u64Value = ASMRdMsr(pHostMsrLoad[i].u32Msr);
    }
}


/**
 * Saves a set of host MSRs to allow read/write passthru access to the guest and
 * perform lazy restoration of the host MSRs while leaving VT-x.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
//主要用于 VMX（Intel VT-x）模式下的上下文切换优化
/*
作用：在 VM Entry（进入虚拟机）之前，保存 Host（宿主机）的关键 MSR 值，以便在 VM Exit（退出虚拟机）时恢复。
"Lazy"（延迟）的含义：
  不是每次 VM Entry 都保存，而是 仅在首次需要时保存，减少不必要的 MSR 读写开销。
  通过 fLazyMsrs 标志位管理状态（VMX_LAZY_MSRS_SAVED_HOST）。
 * */
static void hmR0VmxLazySaveHostMsrs(PVMCPUCC pVCpu)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    /*
     * Note: If you're adding MSRs here, make sure to update the MSR-bitmap accesses in hmR0VmxSetupVmcsProcCtls().
     */
    if (!(pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_SAVED_HOST))
    {
        Assert(!(pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_LOADED_GUEST));  /* Guest MSRs better not be loaded now. *///确保当前没有加载 Guest MSR
        if (pVCpu->CTX_SUFF(pVM)->hmr0.s.fAllow64BitGuests)
        {
            pVCpu->hmr0.s.vmx.u64HostMsrLStar        = ASMRdMsr(MSR_K8_LSTAR);
            pVCpu->hmr0.s.vmx.u64HostMsrStar         = ASMRdMsr(MSR_K6_STAR);
            pVCpu->hmr0.s.vmx.u64HostMsrSfMask       = ASMRdMsr(MSR_K8_SF_MASK);
            pVCpu->hmr0.s.vmx.u64HostMsrKernelGsBase = ASMRdMsr(MSR_K8_KERNEL_GS_BASE);
        }
        pVCpu->hmr0.s.vmx.fLazyMsrs |= VMX_LAZY_MSRS_SAVED_HOST;
    }
}


#ifdef VBOX_STRICT

/**
 * Verifies that our cached host EFER MSR value has not changed since we cached it.
 *
 * @param   pVmcsInfo   The VMCS info. object.
 */
static void hmR0VmxCheckHostEferMsr(PCVMXVMCSINFO pVmcsInfo)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    if (pVmcsInfo->u32ExitCtls & VMX_EXIT_CTLS_LOAD_EFER_MSR)
    {
        uint64_t const uHostEferMsr      = ASMRdMsr(MSR_K6_EFER);
        uint64_t const uHostEferMsrCache = g_uHmVmxHostMsrEfer;
        uint64_t       uVmcsEferMsrVmcs;
        int rc = VMXReadVmcs64(VMX_VMCS64_HOST_EFER_FULL, &uVmcsEferMsrVmcs);
        AssertRC(rc);

        AssertMsgReturnVoid(uHostEferMsr == uVmcsEferMsrVmcs,
                            ("EFER Host/VMCS mismatch! host=%#RX64 vmcs=%#RX64\n", uHostEferMsr, uVmcsEferMsrVmcs));
        AssertMsgReturnVoid(uHostEferMsr == uHostEferMsrCache,
                            ("EFER Host/Cache mismatch! host=%#RX64 cache=%#RX64\n", uHostEferMsr, uHostEferMsrCache));
    }
}


/**
 * Verifies whether the guest/host MSR pairs in the auto-load/store area in the
 * VMCS are correct.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmcsInfo       The VMCS info. object.
 * @param   fIsNstGstVmcs   Whether this is a nested-guest VMCS.
 */
//该函数 hmR0VmxCheckAutoLoadStoreMsrs 的主要目的是在 VM 进入/退出（VM Entry/Exit） 时，
//确保硬件辅助虚拟化中 MSR 的自动加载和存储操作符合预期，避免因 MSR 状态不一致导致虚拟机运行异常。

static void hmR0VmxCheckAutoLoadStoreMsrs(PVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo, bool fIsNstGstVmcs)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    /* Read the various MSR-area counts from the VMCS. */
    uint32_t cEntryLoadMsrs;
    uint32_t cExitStoreMsrs;
    uint32_t cExitLoadMsrs;
    /*
       cEntryLoadMsrs（VM 进入时加载的 MSR 数量）
       cExitStoreMsrs（VM 退出时存储的 MSR 数量）
       cExitLoadMsrs（VM 退出时重新加载的 MSR 数量）
    */
    int rc = VMXReadVmcs32(VMX_VMCS32_CTRL_ENTRY_MSR_LOAD_COUNT, &cEntryLoadMsrs);  AssertRC(rc);
    rc     = VMXReadVmcs32(VMX_VMCS32_CTRL_EXIT_MSR_STORE_COUNT, &cExitStoreMsrs);  AssertRC(rc);
    rc     = VMXReadVmcs32(VMX_VMCS32_CTRL_EXIT_MSR_LOAD_COUNT,  &cExitLoadMsrs);   AssertRC(rc);

    /* Verify all the MSR counts are the same. */
    Assert(cEntryLoadMsrs == cExitStoreMsrs);
    Assert(cExitStoreMsrs == cExitLoadMsrs);
    uint32_t const cMsrs = cExitLoadMsrs;

    /* Verify the MSR counts do not exceed the maximum count supported by the hardware. */
    //确保 MSR 数量不超过 CPU 支持的最大值（VMX_MISC_MAX_MSRS）。
    Assert(cMsrs < VMX_MISC_MAX_MSRS(g_HmMsrs.u.vmx.u64Misc));

    //确保 MSR 数据不超过 4KB 页面大小（sizeof(VMXAUTOMSR) * cMsrs <= X86_PAGE_4K_SIZE）。
    /* Verify the MSR counts are within the allocated page size. */
    Assert(sizeof(VMXAUTOMSR) * cMsrs <= X86_PAGE_4K_SIZE);

    /* Verify the relevant contents of the MSR areas match. */
    PCVMXAUTOMSR pGuestMsrLoad  = (PCVMXAUTOMSR)pVmcsInfo->pvGuestMsrLoad;
    PCVMXAUTOMSR pGuestMsrStore = (PCVMXAUTOMSR)pVmcsInfo->pvGuestMsrStore;
    PCVMXAUTOMSR pHostMsrLoad   = (PCVMXAUTOMSR)pVmcsInfo->pvHostMsrLoad;
    bool const   fSeparateExitMsrStorePage = hmR0VmxIsSeparateExitMsrStoreAreaVmcs(pVmcsInfo);
    for (uint32_t i = 0; i < cMsrs; i++)
    {
        /* Verify that the MSRs are paired properly and that the host MSR has the correct value. */
        if (fSeparateExitMsrStorePage)
        {
            AssertMsgReturnVoid(pGuestMsrLoad->u32Msr == pGuestMsrStore->u32Msr,
                                ("GuestMsrLoad=%#RX32 GuestMsrStore=%#RX32 cMsrs=%u\n",
                                 pGuestMsrLoad->u32Msr, pGuestMsrStore->u32Msr, cMsrs));
        }

        AssertMsgReturnVoid(pHostMsrLoad->u32Msr == pGuestMsrLoad->u32Msr,
                            ("HostMsrLoad=%#RX32 GuestMsrLoad=%#RX32 cMsrs=%u\n",
                             pHostMsrLoad->u32Msr, pGuestMsrLoad->u32Msr, cMsrs));

        uint64_t const u64HostMsr = ASMRdMsr(pHostMsrLoad->u32Msr);
        AssertMsgReturnVoid(pHostMsrLoad->u64Value == u64HostMsr,
                            ("u32Msr=%#RX32 VMCS Value=%#RX64 ASMRdMsr=%#RX64 cMsrs=%u\n",
                             pHostMsrLoad->u32Msr, pHostMsrLoad->u64Value, u64HostMsr, cMsrs));

        /* Verify that cached host EFER MSR matches what's loaded on the CPU. */
        //检查是否与缓存的宿主机 EFER 值一致
        bool const fIsEferMsr = RT_BOOL(pHostMsrLoad->u32Msr == MSR_K6_EFER);
        AssertMsgReturnVoid(!fIsEferMsr || u64HostMsr == g_uHmVmxHostMsrEfer,
                            ("Cached=%#RX64 ASMRdMsr=%#RX64 cMsrs=%u\n", g_uHmVmxHostMsrEfer, u64HostMsr, cMsrs));

        /* Verify that the accesses are as expected in the MSR bitmap for auto-load/store MSRs. */
        if (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS)
        {
            uint32_t const fMsrpm = CPUMGetVmxMsrPermission(pVmcsInfo->pvMsrBitmap, pGuestMsrLoad->u32Msr);
            if (fIsEferMsr)
            {
                AssertMsgReturnVoid((fMsrpm & VMXMSRPM_EXIT_RD), ("Passthru read for EFER MSR!?\n"));
                AssertMsgReturnVoid((fMsrpm & VMXMSRPM_EXIT_WR), ("Passthru write for EFER MSR!?\n"));
            }
            else
            {
                /* Verify LBR MSRs (used only for debugging) are intercepted. We don't passthru these MSRs to the guest yet. */
                /*
                 * LBR（Last Branch Record）MSR（用于调试）：
                   如果启用了 LBR 功能，确保这些 MSR 的访问被正确拦截（不能直接透传给 Guest）。
                   否则，检查 MSR 是否允许读写透传
                 * */
                PCVMCC pVM = pVCpu->CTX_SUFF(pVM);
                if (   pVM->hmr0.s.vmx.fLbr
                    && (   hmR0VmxIsLbrBranchFromMsr(pVM, pGuestMsrLoad->u32Msr, NULL /* pidxMsr */)
                        || hmR0VmxIsLbrBranchToMsr(pVM, pGuestMsrLoad->u32Msr, NULL /* pidxMsr */)
                        || pGuestMsrLoad->u32Msr == pVM->hmr0.s.vmx.idLbrTosMsr))
                {
                    AssertMsgReturnVoid((fMsrpm & VMXMSRPM_MASK) == VMXMSRPM_EXIT_RD_WR,
                                        ("u32Msr=%#RX32 cMsrs=%u Passthru read/write for LBR MSRs!\n",
                                         pGuestMsrLoad->u32Msr, cMsrs));
                }
                else if (!fIsNstGstVmcs)
                {
                    AssertMsgReturnVoid((fMsrpm & VMXMSRPM_MASK) == VMXMSRPM_ALLOW_RD_WR,
                                        ("u32Msr=%#RX32 cMsrs=%u No passthru read/write!\n", pGuestMsrLoad->u32Msr, cMsrs));
                }
                else
                {
                    /*
                     * A nested-guest VMCS must -also- allow read/write passthrough for the MSR for us to
                     * execute a nested-guest with MSR passthrough.
                     *
                     * Check if the nested-guest MSR bitmap allows passthrough, and if so, assert that we
                     * allow passthrough too.
                     */
                    /*
                     * 如果启用了 MSR 位图（VMX_PROC_CTLS_USE_MSR_BITMAPS），确保每个 MSR 的访问权限（读/写）符合预期：
                       EFER MSR 必须拦截读写（不能透传）。
                       普通 MSR 默认允许透传读写（除非是 LBR MSR 或嵌套虚拟化场景）。
                    */
                    /*
                     * MSR Bitmap 是一个 4KB 的内存区域，每个 bit 对应一个 MSR 的访问权限（读/写）。
                       如果某个 MSR 的访问权限设置为 允许（passthrough），则 Guest 访问该 MSR 时 不会触发 VM-Exit，而是直接在 CPU 上执行。
                       如果权限设置为 拦截（intercept），则访问该 MSR 会触发 VM-Exit，由 Hypervisor 处理。
                     * */
                    void const *pvMsrBitmapNstGst = pVCpu->cpum.GstCtx.hwvirt.vmx.abMsrBitmap;
                    Assert(pvMsrBitmapNstGst);
                    uint32_t const fMsrpmNstGst = CPUMGetVmxMsrPermission(pvMsrBitmapNstGst, pGuestMsrLoad->u32Msr);
                    AssertMsgReturnVoid(fMsrpm == fMsrpmNstGst,
                                        ("u32Msr=%#RX32 cMsrs=%u Permission mismatch fMsrpm=%#x fMsrpmNstGst=%#x!\n",
                                         pGuestMsrLoad->u32Msr, cMsrs, fMsrpm, fMsrpmNstGst));
                }
            }
        }

        /* Move to the next MSR. */
        pHostMsrLoad++;
        pGuestMsrLoad++;
        pGuestMsrStore++;
    }
}

#endif /* VBOX_STRICT */

/**
 * Flushes the TLB using EPT.
 *
 * @param   pVCpu           The cross context virtual CPU structure of the calling
 *                          EMT.  Can be NULL depending on @a enmTlbFlush.
 * @param   pVmcsInfo       The VMCS info. object. Can be NULL depending on @a
 *                          enmTlbFlush.
 * @param   enmTlbFlush     Type of flush.
 *
 * @remarks Caller is responsible for making sure this function is called only
 *          when NestedPaging is supported and providing @a enmTlbFlush that is
 *          supported by the CPU.
 * @remarks Can be called with interrupts disabled.
 */
static void hmR0VmxFlushEpt(PVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo, VMXTLBFLUSHEPT enmTlbFlush)
{
    uint64_t au64Descriptor[2];
    if (enmTlbFlush == VMXTLBFLUSHEPT_ALL_CONTEXTS)
        au64Descriptor[0] = 0;
    else
    {
        Assert(pVCpu);
        Assert(pVmcsInfo);
        au64Descriptor[0] = pVmcsInfo->HCPhysEPTP;
    }
    au64Descriptor[1] = 0;                       /* MBZ. Intel spec. 33.3 "VMX Instructions" */

    int rc = VMXR0InvEPT(enmTlbFlush, &au64Descriptor[0]);
    AssertMsg(rc == VINF_SUCCESS, ("VMXR0InvEPT %#x %#RHp failed. rc=%Rrc\n", enmTlbFlush, au64Descriptor[0], rc));

    if (   RT_SUCCESS(rc)
        && pVCpu)
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushNestedPaging);
}


/**
 * Flushes the TLB using VPID.
 *
 * @param   pVCpu           The cross context virtual CPU structure of the calling
 *                          EMT.  Can be NULL depending on @a enmTlbFlush.
 * @param   enmTlbFlush     Type of flush.
 * @param   GCPtr           Virtual address of the page to flush (can be 0 depending
 *                          on @a enmTlbFlush).
 *
 * @remarks Can be called with interrupts disabled.
 */
static void hmR0VmxFlushVpid(PVMCPUCC pVCpu, VMXTLBFLUSHVPID enmTlbFlush, RTGCPTR GCPtr)
{
    Assert(pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fVpid);

    uint64_t au64Descriptor[2];
    if (enmTlbFlush == VMXTLBFLUSHVPID_ALL_CONTEXTS)
    {
        au64Descriptor[0] = 0;
        au64Descriptor[1] = 0;
    }
    else
    {
        AssertPtr(pVCpu);
        AssertMsg(pVCpu->hmr0.s.uCurrentAsid != 0, ("VMXR0InvVPID: invalid ASID %lu\n", pVCpu->hmr0.s.uCurrentAsid));
        AssertMsg(pVCpu->hmr0.s.uCurrentAsid <= UINT16_MAX, ("VMXR0InvVPID: invalid ASID %lu\n", pVCpu->hmr0.s.uCurrentAsid));
        au64Descriptor[0] = pVCpu->hmr0.s.uCurrentAsid;
        au64Descriptor[1] = GCPtr;
    }

    int rc = VMXR0InvVPID(enmTlbFlush, &au64Descriptor[0]);
    AssertMsg(rc == VINF_SUCCESS,
              ("VMXR0InvVPID %#x %u %RGv failed with %Rrc\n", enmTlbFlush, pVCpu ? pVCpu->hmr0.s.uCurrentAsid : 0, GCPtr, rc));

    if (   RT_SUCCESS(rc)
        && pVCpu)
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushAsid);
    NOREF(rc);
}


/**
 * Invalidates a guest page by guest virtual address. Only relevant for EPT/VPID,
 * otherwise there is nothing really to invalidate.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 * @param   GCVirt  Guest virtual address of the page to invalidate.
 */
VMMR0DECL(int) VMXR0InvalidatePage(PVMCPUCC pVCpu, RTGCPTR GCVirt)
{
    AssertPtr(pVCpu);
    LogFlowFunc(("pVCpu=%p GCVirt=%RGv\n", pVCpu, GCVirt));

    if (!VMCPU_FF_IS_SET(pVCpu, VMCPU_FF_TLB_FLUSH))
    {
        /*
         * We must invalidate the guest TLB entry in either case, we cannot ignore it even for
         * the EPT case. See @bugref{6043} and @bugref{6177}.
         *
         * Set the VMCPU_FF_TLB_FLUSH force flag and flush before VM-entry in hmR0VmxFlushTLB*()
         * as this function maybe called in a loop with individual addresses.
         */
        PVMCC pVM = pVCpu->CTX_SUFF(pVM);
        if (pVM->hmr0.s.vmx.fVpid)
        {
            if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVVPID_INDIV_ADDR)
            {
                hmR0VmxFlushVpid(pVCpu, VMXTLBFLUSHVPID_INDIV_ADDR, GCVirt);
                STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbInvlpgVirt);
            }
            else
                VMCPU_FF_SET(pVCpu, VMCPU_FF_TLB_FLUSH);
        }
        else if (pVM->hmr0.s.fNestedPaging)
            VMCPU_FF_SET(pVCpu, VMCPU_FF_TLB_FLUSH);
    }

    return VINF_SUCCESS;
}


/**
 * Dummy placeholder for tagged-TLB flush handling before VM-entry. Used in the
 * case where neither EPT nor VPID is supported by the CPU.
 *
 * @param   pHostCpu    The HM physical-CPU structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 *
 * @remarks Called with interrupts disabled.
 */
static void hmR0VmxFlushTaggedTlbNone(PHMPHYSCPU pHostCpu, PVMCPUCC pVCpu)
{
    AssertPtr(pVCpu);
    AssertPtr(pHostCpu);

    VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_TLB_FLUSH);

    Assert(pHostCpu->idCpu != NIL_RTCPUID);
    pVCpu->hmr0.s.idLastCpu     = pHostCpu->idCpu;
    pVCpu->hmr0.s.cTlbFlushes   = pHostCpu->cTlbFlushes;
    pVCpu->hmr0.s.fForceTLBFlush  = false;
    return;
}


/**
 * Flushes the tagged-TLB entries for EPT+VPID CPUs as necessary.
 *
 * @param   pHostCpu    The HM physical-CPU structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 *
 * @remarks  All references to "ASID" in this function pertains to "VPID" in Intel's
 *           nomenclature. The reason is, to avoid confusion in compare statements
 *           since the host-CPU copies are named "ASID".
 *
 * @remarks  Called with interrupts disabled.
 */
/*
 * 以下是针对 hmR0VmxFlushTaggedTlbBoth 函数的深度解析，结合 Intel VT-x 中 ‌EPT（Extended Page Tables）‌ 和 ‌VPID（Virtual Processor Identifier）‌ 的协同工作机制：

核心作用：在 EPT 和 VPID 同时启用的场景下，管理 TLB 刷新逻辑，确保虚拟机内存访问的正确性与性能。
关键特性：
双重刷新机制：同时处理 EPT 的 INVEPT 和 VPID 的 INVVPID 指令需求，但实际仅需执行 INVEPT（因其覆盖 VPID 的刷新需求）
智能刷新策略：根据跨 CPU 迁移、显式请求、嵌套虚拟化等场景动态决策刷新方式。
性能优化
  通过 VPID 避免不必要的全局 TLB 刷新。
  通过统计计数器（STAM_COUNTER_INC）量化不同场景的刷新频率，辅助调优‌12。

 * */
static void hmR0VmxFlushTaggedTlbBoth(PHMPHYSCPU pHostCpu, PVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo)
{
#ifdef VBOX_WITH_STATISTICS
    bool fTlbFlushed = false;
# define HMVMX_SET_TAGGED_TLB_FLUSHED()       do { fTlbFlushed = true; } while (0)
# define HMVMX_UPDATE_FLUSH_SKIPPED_STAT()    do { \
                                                if (!fTlbFlushed) \
                                                    STAM_COUNTER_INC(&pVCpu->hm.s.StatNoFlushTlbWorldSwitch); \
                                              } while (0)
#else
# define HMVMX_SET_TAGGED_TLB_FLUSHED()       do { } while (0)
# define HMVMX_UPDATE_FLUSH_SKIPPED_STAT()    do { } while (0)
#endif

    AssertPtr(pVCpu);
    AssertPtr(pHostCpu);
    Assert(pHostCpu->idCpu != NIL_RTCPUID);

    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    //EPT 和 VPID 必须同时启用
    //EPT 管理物理地址翻译（GPA->HPA），VPID 管理线性地址翻译（GVA ->GPA）
    AssertMsg(pVM->hmr0.s.fNestedPaging && pVM->hmr0.s.vmx.fVpid,
              ("hmR0VmxFlushTaggedTlbBoth cannot be invoked unless NestedPaging & VPID are enabled."
               "fNestedPaging=%RTbool fVpid=%RTbool", pVM->hmr0.s.fNestedPaging, pVM->hmr0.s.vmx.fVpid));

    /*
     * Force a TLB flush for the first world-switch if the current CPU differs from the one we
     * ran on last. If the TLB flush count changed, another VM (VCPU rather) has hit the ASID
     * limit while flushing the TLB or the host CPU is online after a suspend/resume, so we
     * cannot reuse the current ASID anymore.
     */
    if (   pVCpu->hmr0.s.idLastCpu   != pHostCpu->idCpu
        || pVCpu->hmr0.s.cTlbFlushes != pHostCpu->cTlbFlushes)
    {
        ++pHostCpu->uCurrentAsid;
        if (pHostCpu->uCurrentAsid >= g_uHmMaxAsid)
        {
            pHostCpu->uCurrentAsid = 1;            /* Wraparound to 1; host uses 0. */
            pHostCpu->cTlbFlushes++;               /* All VCPUs that run on this host CPU must use a new VPID. */
            // 新 ASID 需先刷新
            pHostCpu->fFlushAsidBeforeUse = true;  /* All VCPUs that run on this host CPU must flush their new VPID before use. */
        }

        pVCpu->hmr0.s.uCurrentAsid = pHostCpu->uCurrentAsid;
        pVCpu->hmr0.s.idLastCpu    = pHostCpu->idCpu;
        pVCpu->hmr0.s.cTlbFlushes  = pHostCpu->cTlbFlushes;

        /*
         * Flush by EPT when we get rescheduled to a new host CPU to ensure EPT-only tagged mappings are also
         * invalidated. We don't need to flush-by-VPID here as flushing by EPT covers it. See @bugref{6568}.
         */
        //EPT 优先：跨 CPU 时仅需 INVEPT，因其会连带刷新 VPID 关联的 TLB 条目（见 Intel SDM 28.3.3.4）‌34。
        // 同步状态并执行 EPT 刷新
        hmR0VmxFlushEpt(pVCpu, pVmcsInfo, pVM->hmr0.s.vmx.enmTlbFlushEpt);
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbWorldSwitch);
        HMVMX_SET_TAGGED_TLB_FLUSHED();
        VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_TLB_FLUSH);
    }
    //显式 TLB 刷新请求
    //EPT 页表被修改（如内存热插拔），需刷新所有 GPA->HPA 映射
    else if (VMCPU_FF_TEST_AND_CLEAR(pVCpu, VMCPU_FF_TLB_FLUSH))    /* Check for explicit TLB flushes. */
    {
        /*
         * Changes to the EPT paging structure by VMM requires flushing-by-EPT as the CPU
         * creates guest-physical (ie. only EPT-tagged) mappings while traversing the EPT
         * tables when EPT is in use. Flushing-by-VPID will only flush linear (only
         * VPID-tagged) and combined (EPT+VPID tagged) mappings but not guest-physical
         * mappings, see @bugref{6568}.
         *
         * See Intel spec. 28.3.2 "Creating and Using Cached Translation Information".
         */
        hmR0VmxFlushEpt(pVCpu, pVmcsInfo, pVM->hmr0.s.vmx.enmTlbFlushEpt);
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlb);
        HMVMX_SET_TAGGED_TLB_FLUSHED();
    }
    else if (pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb)
    {
        /*
         * The nested-guest specifies its own guest-physical address to use as the APIC-access
         * address which requires flushing the TLB of EPT cached structures.
         *
         * See Intel spec. 28.3.3.4 "Guidelines for Use of the INVEPT Instruction".
         */
        //嵌套虚拟化：L1↔ L2 切换时需刷新 EPT TLB，确保 L2 的 APIC-access 地址等变更生效
        hmR0VmxFlushEpt(pVCpu, pVmcsInfo, pVM->hmr0.s.vmx.enmTlbFlushEpt);
        pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb = false;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbNstGst);
        HMVMX_SET_TAGGED_TLB_FLUSHED();
    }


    pVCpu->hmr0.s.fForceTLBFlush = false;
    HMVMX_UPDATE_FLUSH_SKIPPED_STAT();
    //确保 vCPU 的 ASID、CPU 绑定和刷新计数与物理 CPU 状态一致
    Assert(pVCpu->hmr0.s.idLastCpu == pHostCpu->idCpu);
    Assert(pVCpu->hmr0.s.cTlbFlushes == pHostCpu->cTlbFlushes);
    AssertMsg(pVCpu->hmr0.s.cTlbFlushes == pHostCpu->cTlbFlushes,
              ("Flush count mismatch for cpu %d (%u vs %u)\n", pHostCpu->idCpu, pVCpu->hmr0.s.cTlbFlushes, pHostCpu->cTlbFlushes));
    AssertMsg(pHostCpu->uCurrentAsid >= 1 && pHostCpu->uCurrentAsid < g_uHmMaxAsid,
              ("Cpu[%u] uCurrentAsid=%u cTlbFlushes=%u pVCpu->idLastCpu=%u pVCpu->cTlbFlushes=%u\n", pHostCpu->idCpu,
               pHostCpu->uCurrentAsid, pHostCpu->cTlbFlushes, pVCpu->hmr0.s.idLastCpu, pVCpu->hmr0.s.cTlbFlushes));
    AssertMsg(pVCpu->hmr0.s.uCurrentAsid >= 1 && pVCpu->hmr0.s.uCurrentAsid < g_uHmMaxAsid,
              ("Cpu[%u] pVCpu->uCurrentAsid=%u\n", pHostCpu->idCpu, pVCpu->hmr0.s.uCurrentAsid));

    /* Update VMCS with the VPID. */
    int rc  = VMXWriteVmcs16(VMX_VMCS16_VPID, pVCpu->hmr0.s.uCurrentAsid);
    AssertRC(rc);

#undef HMVMX_SET_TAGGED_TLB_FLUSHED
}


/**
 * Flushes the tagged-TLB entries for EPT CPUs as necessary.
 *
 * @param   pHostCpu    The HM physical-CPU structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 *
 * @remarks Called with interrupts disabled.
 */
static void hmR0VmxFlushTaggedTlbEpt(PHMPHYSCPU pHostCpu, PVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo)
{
    AssertPtr(pVCpu);
    AssertPtr(pHostCpu);
    Assert(pHostCpu->idCpu != NIL_RTCPUID);
    AssertMsg(pVCpu->CTX_SUFF(pVM)->hmr0.s.fNestedPaging, ("hmR0VmxFlushTaggedTlbEpt cannot be invoked without NestedPaging."));
    AssertMsg(!pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fVpid, ("hmR0VmxFlushTaggedTlbEpt cannot be invoked with VPID."));

    /*
     * Force a TLB flush for the first world-switch if the current CPU differs from the one we ran on last.
     * A change in the TLB flush count implies the host CPU is online after a suspend/resume.
     */
    if (   pVCpu->hmr0.s.idLastCpu   != pHostCpu->idCpu
        || pVCpu->hmr0.s.cTlbFlushes != pHostCpu->cTlbFlushes)
    {
        pVCpu->hmr0.s.fForceTLBFlush = true;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbWorldSwitch);
    }

    /* Check for explicit TLB flushes. */
    if (VMCPU_FF_TEST_AND_CLEAR(pVCpu, VMCPU_FF_TLB_FLUSH))
    {
        pVCpu->hmr0.s.fForceTLBFlush = true;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlb);
    }

    /* Check for TLB flushes while switching to/from a nested-guest. */
    if (pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb)
    {
        pVCpu->hmr0.s.fForceTLBFlush = true;
        pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb = false;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbNstGst);
    }

    pVCpu->hmr0.s.idLastCpu = pHostCpu->idCpu;
    pVCpu->hmr0.s.cTlbFlushes = pHostCpu->cTlbFlushes;

    if (pVCpu->hmr0.s.fForceTLBFlush)
    {
        hmR0VmxFlushEpt(pVCpu, pVmcsInfo, pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.enmTlbFlushEpt);
        pVCpu->hmr0.s.fForceTLBFlush = false;
    }
}


/**
 * Flushes the tagged-TLB entries for VPID CPUs as necessary.
 *
 * @param   pHostCpu    The HM physical-CPU structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 *
 * @remarks Called with interrupts disabled.
 */
/*
作用：管理基于 VPID 的 TLB（Translation Lookaside Buffer）刷新逻辑，优化虚拟机上下文切换时的内存访问性能
调用时机：在 vCPU 调度到新物理 CPU 或显式请求 TLB 刷新时调用（如 VMCPU_FF_TLB_FLUSH 标志置位
核心任务：
  判断是否需要强制刷新 TLB（跨 CPU 迁移、ASID 耗尽等场景）。
  分配 VPID（ASID）并执行硬件级 TLB 刷新指令（INVVPID）
  维护全局 ASID 状态，避免多 vCPU 间的 VPID 冲突
*/
static void hmR0VmxFlushTaggedTlbVpid(PHMPHYSCPU pHostCpu, PVMCPUCC pVCpu)
{
    AssertPtr(pVCpu);
    AssertPtr(pHostCpu);
    Assert(pHostCpu->idCpu != NIL_RTCPUID);
    AssertMsg(pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fVpid, ("hmR0VmxFlushTlbVpid cannot be invoked without VPID."));//需 CPU 支持 VPID 特性
    AssertMsg(!pVCpu->CTX_SUFF(pVM)->hmr0.s.fNestedPaging, ("hmR0VmxFlushTlbVpid cannot be invoked with NestedPaging"));

    /*
     * Force a TLB flush for the first world switch if the current CPU differs from the one we
     * ran on last. If the TLB flush count changed, another VM (VCPU rather) has hit the ASID
     * limit while flushing the TLB or the host CPU is online after a suspend/resume, so we
     * cannot reuse the current ASID anymore.
     */
    //CPU 迁移：vCPU 上次运行的物理 CPU 与当前不同（idLastCpu 不匹配）
    //全局 ASID 失效：物理 CPU 的 TLB 刷新计数（cTlbFlushes）变化（如 ASID 耗尽或主机休眠恢复）
    if (   pVCpu->hmr0.s.idLastCpu != pHostCpu->idCpu
        || pVCpu->hmr0.s.cTlbFlushes != pHostCpu->cTlbFlushes)
    {
        pVCpu->hmr0.s.fForceTLBFlush = true;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbWorldSwitch);
    }

    //内存映射变更（如 CR3 修改）或外部强制刷新
    /* Check for explicit TLB flushes. */
    if (VMCPU_FF_TEST_AND_CLEAR(pVCpu, VMCPU_FF_TLB_FLUSH))
    {
        /*
         * If we ever support VPID flush combinations other than ALL or SINGLE-context (see
         * hmR0VmxSetupTaggedTlb()) we would need to explicitly flush in this case (add an
         * fExplicitFlush = true here and change the pHostCpu->fFlushAsidBeforeUse check below to
         * include fExplicitFlush's too) - an obscure corner case.
         */
        pVCpu->hmr0.s.fForceTLBFlush = true;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlb);
    }

    /* Check for TLB flushes while switching to/from a nested-guest. */
    //嵌套虚拟化：L1 <--> L2 虚拟机切换时需刷新 TLB 以保证隔离性
    if (pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb)
    {
        pVCpu->hmr0.s.fForceTLBFlush = true;
        pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb = false;
        STAM_COUNTER_INC(&pVCpu->hm.s.StatFlushTlbNstGst);
    }

    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    pVCpu->hmr0.s.idLastCpu = pHostCpu->idCpu;
    if (pVCpu->hmr0.s.fForceTLBFlush)
    {
        /* ASID 分配逻辑 */
        ++pHostCpu->uCurrentAsid;
        if (pHostCpu->uCurrentAsid >= g_uHmMaxAsid)// ASID 耗尽
        {
            pHostCpu->uCurrentAsid        = 1;     /* Wraparound to 1; host uses 0 */// 回绕到 1（0 保留给主机）
            pHostCpu->cTlbFlushes++;               /* All VCPUs that run on this host CPU must use a new VPID. */// 全局失效计数递增
            pHostCpu->fFlushAsidBeforeUse = true;  /* All VCPUs that run on this host CPU must flush their new VPID before use. */// 新 ASID 需先刷新
        }

         /* 同步状态到 vCPU */
        pVCpu->hmr0.s.fForceTLBFlush = false;
        pVCpu->hmr0.s.cTlbFlushes    = pHostCpu->cTlbFlushes;
        pVCpu->hmr0.s.uCurrentAsid   = pHostCpu->uCurrentAsid;
        /* 执行硬件刷新, INVVPID：根据类型刷新 TLB： */
        if (pHostCpu->fFlushAsidBeforeUse)
        {
            if (pVM->hmr0.s.vmx.enmTlbFlushVpid == VMXTLBFLUSHVPID_SINGLE_CONTEXT)
                hmR0VmxFlushVpid(pVCpu, VMXTLBFLUSHVPID_SINGLE_CONTEXT, 0 /* GCPtr */);
            else if (pVM->hmr0.s.vmx.enmTlbFlushVpid == VMXTLBFLUSHVPID_ALL_CONTEXTS)
            {
                hmR0VmxFlushVpid(pVCpu, VMXTLBFLUSHVPID_ALL_CONTEXTS, 0 /* GCPtr */);
                pHostCpu->fFlushAsidBeforeUse = false;// 全局刷新后无需单上下文刷新
            }
            else
            {
                /* hmR0VmxSetupTaggedTlb() ensures we never get here. Paranoia. */
                AssertMsgFailed(("Unsupported VPID-flush context type.\n"));
            }
        }
    }

    AssertMsg(pVCpu->hmr0.s.cTlbFlushes == pHostCpu->cTlbFlushes,
              ("Flush count mismatch for cpu %d (%u vs %u)\n", pHostCpu->idCpu, pVCpu->hmr0.s.cTlbFlushes, pHostCpu->cTlbFlushes));
    AssertMsg(pHostCpu->uCurrentAsid >= 1 && pHostCpu->uCurrentAsid < g_uHmMaxAsid,
              ("Cpu[%u] uCurrentAsid=%u cTlbFlushes=%u pVCpu->idLastCpu=%u pVCpu->cTlbFlushes=%u\n", pHostCpu->idCpu,
               pHostCpu->uCurrentAsid, pHostCpu->cTlbFlushes, pVCpu->hmr0.s.idLastCpu, pVCpu->hmr0.s.cTlbFlushes));
    AssertMsg(pVCpu->hmr0.s.uCurrentAsid >= 1 && pVCpu->hmr0.s.uCurrentAsid < g_uHmMaxAsid,
              ("Cpu[%u] pVCpu->uCurrentAsid=%u\n", pHostCpu->idCpu, pVCpu->hmr0.s.uCurrentAsid));

    int rc  = VMXWriteVmcs16(VMX_VMCS16_VPID, pVCpu->hmr0.s.uCurrentAsid);
    AssertRC(rc);
}


/**
 * Flushes the guest TLB entry based on CPU capabilities.
 *
 * @param   pHostCpu    The HM physical-CPU structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 *
 * @remarks Called with interrupts disabled.
 */
static void hmR0VmxFlushTaggedTlb(PHMPHYSCPU pHostCpu, PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
#ifdef HMVMX_ALWAYS_FLUSH_TLB
    VMCPU_FF_SET(pVCpu, VMCPU_FF_TLB_FLUSH);
#endif
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    switch (pVM->hmr0.s.vmx.enmTlbFlushType)
    {
        case VMXTLBFLUSHTYPE_EPT_VPID: hmR0VmxFlushTaggedTlbBoth(pHostCpu, pVCpu, pVmcsInfo); break;
        case VMXTLBFLUSHTYPE_EPT:      hmR0VmxFlushTaggedTlbEpt(pHostCpu, pVCpu, pVmcsInfo);  break;
        case VMXTLBFLUSHTYPE_VPID:     hmR0VmxFlushTaggedTlbVpid(pHostCpu, pVCpu);            break;
        case VMXTLBFLUSHTYPE_NONE:     hmR0VmxFlushTaggedTlbNone(pHostCpu, pVCpu);            break;
        default:
            AssertMsgFailed(("Invalid flush-tag function identifier\n"));
            break;
    }
    /* Don't assert that VMCPU_FF_TLB_FLUSH should no longer be pending. It can be set by other EMTs. */
}


/**
 * Sets up the appropriate tagged TLB-flush level and handler for flushing guest
 * TLB entries from the host TLB before VM-entry.
 *
 * @returns VBox status code.
 * @param   pVM     The cross context VM structure.
 */
//根据 CPU 支持的虚拟化特性（EPT 和 VPID），
//选择最优的 TLB 刷新策略，避免不必要的全局 TLB 刷新，提高虚拟机性能。
/*
关键概念：
  EPT（Extended Page Tables）：Intel 的嵌套分页技术，用于加速虚拟机内存地址转换。
  VPID（Virtual Processor Identifier）：允许 TLB 条目与特定虚拟 CPU 关联，减少 TLB 刷新开销。
  TLB 刷新：当内存映射发生变化时（如页表更新），需要刷新 TLB 以确保一致性。
*/
static int hmR0VmxSetupTaggedTlb(PVMCC pVM)
{
    /*
     * Determine optimal flush type for nested paging.
     * We cannot ignore EPT if no suitable flush-types is supported by the CPU as we've already setup
     * unrestricted guest execution (see hmR3InitFinalizeR0()).
     */
    if (pVM->hmr0.s.fNestedPaging)
    {
        if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVEPT)
        {
            //INVEPT_SINGLE_CONTEXT：仅刷新当前 VPID 的 TLB（更高效）。
            if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVEPT_SINGLE_CONTEXT)
                pVM->hmr0.s.vmx.enmTlbFlushEpt = VMXTLBFLUSHEPT_SINGLE_CONTEXT;
            //INVEPT_ALL_CONTEXTS：刷新所有 VPID 的 TLB（保守但安全）。
            else if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVEPT_ALL_CONTEXTS)
                pVM->hmr0.s.vmx.enmTlbFlushEpt = VMXTLBFLUSHEPT_ALL_CONTEXTS;
            else
            {
                /* Shouldn't happen. EPT is supported but no suitable flush-types supported. */
                /* 不支持任何有效的 EPT 刷新方式 */
                pVM->hmr0.s.vmx.enmTlbFlushEpt = VMXTLBFLUSHEPT_NOT_SUPPORTED;
                VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_EPT_FLUSH_TYPE_UNSUPPORTED;
                return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
            }

            /* Make sure the write-back cacheable memory type for EPT is supported. */
            /* 检查 EPT 是否支持 WB（Write-Back）内存类型 */
            if (RT_UNLIKELY(!(g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_MEMTYPE_WB)))
            {
                pVM->hmr0.s.vmx.enmTlbFlushEpt = VMXTLBFLUSHEPT_NOT_SUPPORTED;
                VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_EPT_MEM_TYPE_NOT_WB;
                return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
            }

            /* EPT requires a page-walk length of 4. */
            /* 检查 EPT 页表遍历深度是否为 4 */
            if (RT_UNLIKELY(!(g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_PAGE_WALK_LENGTH_4)))
            {
                pVM->hmr0.s.vmx.enmTlbFlushEpt = VMXTLBFLUSHEPT_NOT_SUPPORTED;
                VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_EPT_PAGE_WALK_LENGTH_UNSUPPORTED;
                return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
            }
        }
        else
        {
            /* Shouldn't happen. EPT is supported but INVEPT instruction is not supported. */
            pVM->hmr0.s.vmx.enmTlbFlushEpt = VMXTLBFLUSHEPT_NOT_SUPPORTED;
            VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_EPT_INVEPT_UNAVAILABLE;
            return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
        }
    }

    /*
     * Determine optimal flush type for VPID.
     */
    if (pVM->hmr0.s.vmx.fVpid)
    {
        if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVVPID)
        {
            if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVVPID_SINGLE_CONTEXT)
                pVM->hmr0.s.vmx.enmTlbFlushVpid = VMXTLBFLUSHVPID_SINGLE_CONTEXT;
            else if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVVPID_ALL_CONTEXTS)
                pVM->hmr0.s.vmx.enmTlbFlushVpid = VMXTLBFLUSHVPID_ALL_CONTEXTS;
            else
            {
                /* Neither SINGLE nor ALL-context flush types for VPID is supported by the CPU. Ignore VPID capability. */
                if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVVPID_INDIV_ADDR)
                    LogRelFunc(("Only INDIV_ADDR supported. Ignoring VPID.\n"));
                if (g_HmMsrs.u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVVPID_SINGLE_CONTEXT_RETAIN_GLOBALS)
                    LogRelFunc(("Only SINGLE_CONTEXT_RETAIN_GLOBALS supported. Ignoring VPID.\n"));
                pVM->hmr0.s.vmx.enmTlbFlushVpid = VMXTLBFLUSHVPID_NOT_SUPPORTED;
                pVM->hmr0.s.vmx.fVpid           = false;
            }
        }
        else
        {
            /*  Shouldn't happen. VPID is supported but INVVPID is not supported by the CPU. Ignore VPID capability. */
            Log4Func(("VPID supported without INVEPT support. Ignoring VPID.\n"));
            pVM->hmr0.s.vmx.enmTlbFlushVpid = VMXTLBFLUSHVPID_NOT_SUPPORTED;
            pVM->hmr0.s.vmx.fVpid           = false;
        }
    }

    /*
     * Setup the handler for flushing tagged-TLBs.
     */
    //最终 TLB 刷新策略选择
    if (pVM->hmr0.s.fNestedPaging && pVM->hmr0.s.vmx.fVpid)
        pVM->hmr0.s.vmx.enmTlbFlushType = VMXTLBFLUSHTYPE_EPT_VPID;//EPT + VPID（最优，减少 TLB 刷新次数）
    else if (pVM->hmr0.s.fNestedPaging)
        pVM->hmr0.s.vmx.enmTlbFlushType = VMXTLBFLUSHTYPE_EPT; //仅 EPT（次优，但仍有硬件加速）。
    else if (pVM->hmr0.s.vmx.fVpid)
        pVM->hmr0.s.vmx.enmTlbFlushType = VMXTLBFLUSHTYPE_VPID;//仅 VPID（较少使用，通常 EPT 更重要）。
    else
        pVM->hmr0.s.vmx.enmTlbFlushType = VMXTLBFLUSHTYPE_NONE;//无优化（完全软件刷新，性能最差）。


    /*
     * Copy out the result to ring-3.
     */
    pVM->hm.s.ForR3.vmx.fVpid           = pVM->hmr0.s.vmx.fVpid;
    pVM->hm.s.ForR3.vmx.enmTlbFlushType = pVM->hmr0.s.vmx.enmTlbFlushType;
    pVM->hm.s.ForR3.vmx.enmTlbFlushEpt  = pVM->hmr0.s.vmx.enmTlbFlushEpt;
    pVM->hm.s.ForR3.vmx.enmTlbFlushVpid = pVM->hmr0.s.vmx.enmTlbFlushVpid;
    return VINF_SUCCESS;
}


/**
 * Sets up the LBR MSR ranges based on the host CPU.
 *
 * @returns VBox status code.
 * @param   pVM     The cross context VM structure.
 *
 * @sa nemR3DarwinSetupLbrMsrRange
 */
static int hmR0VmxSetupLbrMsrRange(PVMCC pVM)
{
    Assert(pVM->hmr0.s.vmx.fLbr);
    uint32_t idLbrFromIpMsrFirst;
    uint32_t idLbrFromIpMsrLast;
    uint32_t idLbrToIpMsrFirst;
    uint32_t idLbrToIpMsrLast;
    uint32_t idLbrTosMsr;

    /*
     * Determine the LBR MSRs supported for this host CPU family and model.
     *
     * See Intel spec. 17.4.8 "LBR Stack".
     * See Intel "Model-Specific Registers" spec.
     */
    uint32_t const uFamilyModel = (g_CpumHostFeatures.s.uFamily << 8)
                                | g_CpumHostFeatures.s.uModel;
    switch (uFamilyModel)
    {
        case 0x0f01: case 0x0f02:
            idLbrFromIpMsrFirst = MSR_P4_LASTBRANCH_0;
            idLbrFromIpMsrLast  = MSR_P4_LASTBRANCH_3;
            idLbrToIpMsrFirst   = 0x0;
            idLbrToIpMsrLast    = 0x0;
            idLbrTosMsr         = MSR_P4_LASTBRANCH_TOS;
            break;

        case 0x065c: case 0x065f: case 0x064e: case 0x065e: case 0x068e:
        case 0x069e: case 0x0655: case 0x0666: case 0x067a: case 0x0667:
        case 0x066a: case 0x066c: case 0x067d: case 0x067e:
            idLbrFromIpMsrFirst = MSR_LASTBRANCH_0_FROM_IP;
            idLbrFromIpMsrLast  = MSR_LASTBRANCH_31_FROM_IP;
            idLbrToIpMsrFirst   = MSR_LASTBRANCH_0_TO_IP;
            idLbrToIpMsrLast    = MSR_LASTBRANCH_31_TO_IP;
            idLbrTosMsr         = MSR_LASTBRANCH_TOS;
            break;

        case 0x063d: case 0x0647: case 0x064f: case 0x0656: case 0x063c:
        case 0x0645: case 0x0646: case 0x063f: case 0x062a: case 0x062d:
        case 0x063a: case 0x063e: case 0x061a: case 0x061e: case 0x061f:
        case 0x062e: case 0x0625: case 0x062c: case 0x062f:
            idLbrFromIpMsrFirst = MSR_LASTBRANCH_0_FROM_IP;
            idLbrFromIpMsrLast  = MSR_LASTBRANCH_15_FROM_IP;
            idLbrToIpMsrFirst   = MSR_LASTBRANCH_0_TO_IP;
            idLbrToIpMsrLast    = MSR_LASTBRANCH_15_TO_IP;
            idLbrTosMsr         = MSR_LASTBRANCH_TOS;
            break;

        case 0x0617: case 0x061d: case 0x060f:
            idLbrFromIpMsrFirst = MSR_CORE2_LASTBRANCH_0_FROM_IP;
            idLbrFromIpMsrLast  = MSR_CORE2_LASTBRANCH_3_FROM_IP;
            idLbrToIpMsrFirst   = MSR_CORE2_LASTBRANCH_0_TO_IP;
            idLbrToIpMsrLast    = MSR_CORE2_LASTBRANCH_3_TO_IP;
            idLbrTosMsr         = MSR_CORE2_LASTBRANCH_TOS;
            break;

        /* Atom and related microarchitectures we don't care about:
        case 0x0637: case 0x064a: case 0x064c: case 0x064d: case 0x065a:
        case 0x065d: case 0x061c: case 0x0626: case 0x0627: case 0x0635:
        case 0x0636: */
        /* All other CPUs: */
        default:
        {
            LogRelFunc(("Could not determine LBR stack size for the CPU model %#x\n", uFamilyModel));
            VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_LBR_STACK_SIZE_UNKNOWN;
            return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
        }
    }

    /*
     * Validate.
     */
    uint32_t const cLbrStack = idLbrFromIpMsrLast - idLbrFromIpMsrFirst + 1;
    PCVMCPU pVCpu0 = VMCC_GET_CPU_0(pVM);
    AssertCompile(   RT_ELEMENTS(pVCpu0->hm.s.vmx.VmcsInfo.au64LbrFromIpMsr)
                  == RT_ELEMENTS(pVCpu0->hm.s.vmx.VmcsInfo.au64LbrToIpMsr));
    if (cLbrStack > RT_ELEMENTS(pVCpu0->hm.s.vmx.VmcsInfo.au64LbrFromIpMsr))
    {
        LogRelFunc(("LBR stack size of the CPU (%u) exceeds our buffer size\n", cLbrStack));
        VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_LBR_STACK_SIZE_OVERFLOW;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }
    NOREF(pVCpu0);

    /*
     * Update the LBR info. to the VM struct. for use later.
     */
    pVM->hmr0.s.vmx.idLbrTosMsr = idLbrTosMsr;

    pVM->hm.s.ForR3.vmx.idLbrFromIpMsrFirst = pVM->hmr0.s.vmx.idLbrFromIpMsrFirst = idLbrFromIpMsrFirst;
    pVM->hm.s.ForR3.vmx.idLbrFromIpMsrLast  = pVM->hmr0.s.vmx.idLbrFromIpMsrLast  = idLbrFromIpMsrLast;

    pVM->hm.s.ForR3.vmx.idLbrToIpMsrFirst   = pVM->hmr0.s.vmx.idLbrToIpMsrFirst   = idLbrToIpMsrFirst;
    pVM->hm.s.ForR3.vmx.idLbrToIpMsrLast    = pVM->hmr0.s.vmx.idLbrToIpMsrLast    = idLbrToIpMsrLast;
    return VINF_SUCCESS;
}

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX

/**
 * Sets up the shadow VMCS fields arrays.
 *
 * This function builds arrays of VMCS fields to sync the shadow VMCS later while
 * executing the guest.
 *
 * @returns VBox status code.
 * @param   pVM     The cross context VM structure.
 */
static int hmR0VmxSetupShadowVmcsFieldsArrays(PVMCC pVM)
{
    /*
     * Paranoia. Ensure we haven't exposed the VMWRITE-All VMX feature to the guest
     * when the host does not support it.
     */
    bool const fGstVmwriteAll = pVM->cpum.ro.GuestFeatures.fVmxVmwriteAll;
    if (   !fGstVmwriteAll
        || (g_HmMsrs.u.vmx.u64Misc & VMX_MISC_VMWRITE_ALL))
    { /* likely. */ }
    else
    {
        LogRelFunc(("VMX VMWRITE-All feature exposed to the guest but host CPU does not support it!\n"));
        VMCC_GET_CPU_0(pVM)->hm.s.u32HMError = VMX_UFC_GST_HOST_VMWRITE_ALL;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    uint32_t const cVmcsFields = RT_ELEMENTS(g_aVmcsFields);
    uint32_t       cRwFields   = 0;
    uint32_t       cRoFields   = 0;
    for (uint32_t i = 0; i < cVmcsFields; i++)
    {
        VMXVMCSFIELD VmcsField;
        VmcsField.u = g_aVmcsFields[i];

        /*
         * We will be writing "FULL" (64-bit) fields while syncing the shadow VMCS.
         * Therefore, "HIGH" (32-bit portion of 64-bit) fields must not be included
         * in the shadow VMCS fields array as they would be redundant.
         *
         * If the VMCS field depends on a CPU feature that is not exposed to the guest,
         * we must not include it in the shadow VMCS fields array. Guests attempting to
         * VMREAD/VMWRITE such VMCS fields would cause a VM-exit and we shall emulate
         * the required behavior.
         */
        if (   VmcsField.n.fAccessType == VMX_VMCSFIELD_ACCESS_FULL
            && CPUMIsGuestVmxVmcsFieldValid(pVM, VmcsField.u))
        {
            /*
             * Read-only fields are placed in a separate array so that while syncing shadow
             * VMCS fields later (which is more performance critical) we can avoid branches.
             *
             * However, if the guest can write to all fields (including read-only fields),
             * we treat it a as read/write field. Otherwise, writing to these fields would
             * cause a VMWRITE instruction error while syncing the shadow VMCS.
             */
            if (   fGstVmwriteAll
                || !VMXIsVmcsFieldReadOnly(VmcsField.u))
                pVM->hmr0.s.vmx.paShadowVmcsFields[cRwFields++] = VmcsField.u;
            else
                pVM->hmr0.s.vmx.paShadowVmcsRoFields[cRoFields++] = VmcsField.u;
        }
    }

    /* Update the counts. */
    pVM->hmr0.s.vmx.cShadowVmcsFields   = cRwFields;
    pVM->hmr0.s.vmx.cShadowVmcsRoFields = cRoFields;
    return VINF_SUCCESS;
}


/**
 * Sets up the VMREAD and VMWRITE bitmaps.
 *
 * @param   pVM     The cross context VM structure.
 */
static void hmR0VmxSetupVmreadVmwriteBitmaps(PVMCC pVM)
{
    /*
     * By default, ensure guest attempts to access any VMCS fields cause VM-exits.
     */
    uint32_t const cbBitmap        = X86_PAGE_4K_SIZE;
    uint8_t       *pbVmreadBitmap  = (uint8_t *)pVM->hmr0.s.vmx.pvVmreadBitmap;
    uint8_t       *pbVmwriteBitmap = (uint8_t *)pVM->hmr0.s.vmx.pvVmwriteBitmap;
    ASMMemFill32(pbVmreadBitmap,  cbBitmap, UINT32_C(0xffffffff));
    ASMMemFill32(pbVmwriteBitmap, cbBitmap, UINT32_C(0xffffffff));

    /*
     * Skip intercepting VMREAD/VMWRITE to guest read/write fields in the
     * VMREAD and VMWRITE bitmaps.
     */
    {
        uint32_t const *paShadowVmcsFields = pVM->hmr0.s.vmx.paShadowVmcsFields;
        uint32_t const  cShadowVmcsFields  = pVM->hmr0.s.vmx.cShadowVmcsFields;
        for (uint32_t i = 0; i < cShadowVmcsFields; i++)
        {
            uint32_t const uVmcsField = paShadowVmcsFields[i];
            Assert(!(uVmcsField & VMX_VMCSFIELD_RSVD_MASK));
            Assert(uVmcsField >> 3 < cbBitmap);
            ASMBitClear(pbVmreadBitmap,  uVmcsField & 0x7fff);
            ASMBitClear(pbVmwriteBitmap, uVmcsField & 0x7fff);
        }
    }

    /*
     * Skip intercepting VMREAD for guest read-only fields in the VMREAD bitmap
     * if the host supports VMWRITE to all supported VMCS fields.
     */
    if (g_HmMsrs.u.vmx.u64Misc & VMX_MISC_VMWRITE_ALL)
    {
        uint32_t const *paShadowVmcsRoFields = pVM->hmr0.s.vmx.paShadowVmcsRoFields;
        uint32_t const  cShadowVmcsRoFields  = pVM->hmr0.s.vmx.cShadowVmcsRoFields;
        for (uint32_t i = 0; i < cShadowVmcsRoFields; i++)
        {
            uint32_t const uVmcsField = paShadowVmcsRoFields[i];
            Assert(!(uVmcsField & VMX_VMCSFIELD_RSVD_MASK));
            Assert(uVmcsField >> 3 < cbBitmap);
            ASMBitClear(pbVmreadBitmap, uVmcsField & 0x7fff);
        }
    }
}

#endif /* VBOX_WITH_NESTED_HWVIRT_VMX */

/**
 * Sets up the virtual-APIC page address for the VMCS.
 *
 * @param   pVmcsInfo   The VMCS info. object.
 */
DECLINLINE(void) hmR0VmxSetupVmcsVirtApicAddr(PCVMXVMCSINFO pVmcsInfo)
{
    RTHCPHYS const HCPhysVirtApic = pVmcsInfo->HCPhysVirtApic;
    Assert(HCPhysVirtApic != NIL_RTHCPHYS);
    Assert(!(HCPhysVirtApic & 0xfff));                       /* Bits 11:0 MBZ. */
    int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_VIRT_APIC_PAGEADDR_FULL, HCPhysVirtApic);
    AssertRC(rc);
}


/**
 * Sets up the MSR-bitmap address for the VMCS.
 *
 * @param   pVmcsInfo   The VMCS info. object.
 */
DECLINLINE(void) hmR0VmxSetupVmcsMsrBitmapAddr(PCVMXVMCSINFO pVmcsInfo)
{
    RTHCPHYS const HCPhysMsrBitmap = pVmcsInfo->HCPhysMsrBitmap;
    Assert(HCPhysMsrBitmap != NIL_RTHCPHYS);
    Assert(!(HCPhysMsrBitmap & 0xfff));                      /* Bits 11:0 MBZ. */
    int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_MSR_BITMAP_FULL, HCPhysMsrBitmap);
    AssertRC(rc);
}


/**
 * Sets up the APIC-access page address for the VMCS.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 */
DECLINLINE(void) hmR0VmxSetupVmcsApicAccessAddr(PVMCPUCC pVCpu)
{
    RTHCPHYS const HCPhysApicAccess = pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.HCPhysApicAccess;
    Assert(HCPhysApicAccess != NIL_RTHCPHYS);
    Assert(!(HCPhysApicAccess & 0xfff));                     /* Bits 11:0 MBZ. */
    int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_APIC_ACCESSADDR_FULL, HCPhysApicAccess);
    AssertRC(rc);
}

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX

/**
 * Sets up the VMREAD bitmap address for the VMCS.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 */
DECLINLINE(void) hmR0VmxSetupVmcsVmreadBitmapAddr(PVMCPUCC pVCpu)
{
    RTHCPHYS const HCPhysVmreadBitmap = pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.HCPhysVmreadBitmap;
    Assert(HCPhysVmreadBitmap != NIL_RTHCPHYS);
    Assert(!(HCPhysVmreadBitmap & 0xfff));                     /* Bits 11:0 MBZ. */
    int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_VMREAD_BITMAP_FULL, HCPhysVmreadBitmap);
    AssertRC(rc);
}


/**
 * Sets up the VMWRITE bitmap address for the VMCS.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 */
DECLINLINE(void) hmR0VmxSetupVmcsVmwriteBitmapAddr(PVMCPUCC pVCpu)
{
    RTHCPHYS const HCPhysVmwriteBitmap = pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.HCPhysVmwriteBitmap;
    Assert(HCPhysVmwriteBitmap != NIL_RTHCPHYS);
    Assert(!(HCPhysVmwriteBitmap & 0xfff));                     /* Bits 11:0 MBZ. */
    int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_VMWRITE_BITMAP_FULL, HCPhysVmwriteBitmap);
    AssertRC(rc);
}

#endif

/**
 * Sets up the VM-entry MSR load, VM-exit MSR-store and VM-exit MSR-load addresses
 * in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVmcsInfo   The VMCS info. object.
 */
DECLINLINE(int) hmR0VmxSetupVmcsAutoLoadStoreMsrAddrs(PVMXVMCSINFO pVmcsInfo)
{
    RTHCPHYS const HCPhysGuestMsrLoad = pVmcsInfo->HCPhysGuestMsrLoad;
    Assert(HCPhysGuestMsrLoad != NIL_RTHCPHYS);
    Assert(!(HCPhysGuestMsrLoad & 0xf));                     /* Bits 3:0 MBZ. */

    RTHCPHYS const HCPhysGuestMsrStore = pVmcsInfo->HCPhysGuestMsrStore;
    Assert(HCPhysGuestMsrStore != NIL_RTHCPHYS);
    Assert(!(HCPhysGuestMsrStore & 0xf));                    /* Bits 3:0 MBZ. */

    RTHCPHYS const HCPhysHostMsrLoad = pVmcsInfo->HCPhysHostMsrLoad;
    Assert(HCPhysHostMsrLoad != NIL_RTHCPHYS);
    Assert(!(HCPhysHostMsrLoad & 0xf));                      /* Bits 3:0 MBZ. */

    int rc = VMXWriteVmcs64(VMX_VMCS64_CTRL_ENTRY_MSR_LOAD_FULL, HCPhysGuestMsrLoad);   AssertRC(rc);
    rc     = VMXWriteVmcs64(VMX_VMCS64_CTRL_EXIT_MSR_STORE_FULL, HCPhysGuestMsrStore);  AssertRC(rc);
    rc     = VMXWriteVmcs64(VMX_VMCS64_CTRL_EXIT_MSR_LOAD_FULL,  HCPhysHostMsrLoad);    AssertRC(rc);
    return VINF_SUCCESS;
}


/**
 * Sets up MSR permissions in the MSR bitmap of a VMCS info. object.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmcsInfo       The VMCS info. object.
 */
static void hmR0VmxSetupVmcsMsrPermissions(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
    Assert(pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS);

    /*
     * By default, ensure guest attempts to access any MSR cause VM-exits.
     * This shall later be relaxed for specific MSRs as necessary.
     *
     * Note: For nested-guests, the entire bitmap will be merged prior to
     * executing the nested-guest using hardware-assisted VMX and hence there
     * is no need to perform this operation. See hmR0VmxMergeMsrBitmapNested.
     */
    Assert(pVmcsInfo->pvMsrBitmap);
    ASMMemFill32(pVmcsInfo->pvMsrBitmap, X86_PAGE_4K_SIZE, UINT32_C(0xffffffff));

    /*
     * The guest can access the following MSRs (read, write) without causing
     * VM-exits; they are loaded/stored automatically using fields in the VMCS.
     */
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_IA32_SYSENTER_CS,  VMXMSRPM_ALLOW_RD_WR);
    hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_IA32_SYSENTER_ESP, VMXMSRPM_ALLOW_RD_WR);
    hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_IA32_SYSENTER_EIP, VMXMSRPM_ALLOW_RD_WR);
    hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_K8_GS_BASE,        VMXMSRPM_ALLOW_RD_WR);
    hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_K8_FS_BASE,        VMXMSRPM_ALLOW_RD_WR);

    /*
     * The IA32_PRED_CMD and IA32_FLUSH_CMD MSRs are write-only and has no state
     * associated with then. We never need to intercept access (writes need to be
     * executed without causing a VM-exit, reads will #GP fault anyway).
     *
     * The IA32_SPEC_CTRL MSR is read/write and has state. We allow the guest to
     * read/write them. We swap the guest/host MSR value using the
     * auto-load/store MSR area.
     */
    if (pVM->cpum.ro.GuestFeatures.fIbpb)
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_IA32_PRED_CMD,  VMXMSRPM_ALLOW_RD_WR);
    if (pVM->cpum.ro.GuestFeatures.fFlushCmd)
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_IA32_FLUSH_CMD, VMXMSRPM_ALLOW_RD_WR);
    if (pVM->cpum.ro.GuestFeatures.fIbrs)
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_IA32_SPEC_CTRL, VMXMSRPM_ALLOW_RD_WR);

    /*
     * Allow full read/write access for the following MSRs (mandatory for VT-x)
     * required for 64-bit guests.
     */
    if (pVM->hmr0.s.fAllow64BitGuests)
    {
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_K8_LSTAR,          VMXMSRPM_ALLOW_RD_WR);
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_K6_STAR,           VMXMSRPM_ALLOW_RD_WR);
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_K8_SF_MASK,        VMXMSRPM_ALLOW_RD_WR);
        hmR0VmxSetMsrPermission(pVCpu, pVmcsInfo, false, MSR_K8_KERNEL_GS_BASE, VMXMSRPM_ALLOW_RD_WR);
    }

    /*
     * IA32_EFER MSR is always intercepted, see @bugref{9180#c37}.
     */
#ifdef VBOX_STRICT
    Assert(pVmcsInfo->pvMsrBitmap);
    uint32_t const fMsrpmEfer = CPUMGetVmxMsrPermission(pVmcsInfo->pvMsrBitmap, MSR_K6_EFER);
    Assert(fMsrpmEfer == VMXMSRPM_EXIT_RD_WR);
#endif
}


/**
 * Sets up pin-based VM-execution controls in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static int hmR0VmxSetupVmcsPinCtls(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    uint32_t       fVal = g_HmMsrs.u.vmx.PinCtls.n.allowed0;      /* Bits set here must always be set. */
    uint32_t const fZap = g_HmMsrs.u.vmx.PinCtls.n.allowed1;      /* Bits cleared here must always be cleared. */

    fVal |= VMX_PIN_CTLS_EXT_INT_EXIT                        /* External interrupts cause a VM-exit. */
         |  VMX_PIN_CTLS_NMI_EXIT;                           /* Non-maskable interrupts (NMIs) cause a VM-exit. */

    if (g_HmMsrs.u.vmx.PinCtls.n.allowed1 & VMX_PIN_CTLS_VIRT_NMI)
        fVal |= VMX_PIN_CTLS_VIRT_NMI;                       /* Use virtual NMIs and virtual-NMI blocking features. */

    /* Enable the VMX-preemption timer. */
    if (pVM->hmr0.s.vmx.fUsePreemptTimer)
    {
        Assert(g_HmMsrs.u.vmx.PinCtls.n.allowed1 & VMX_PIN_CTLS_PREEMPT_TIMER);
        fVal |= VMX_PIN_CTLS_PREEMPT_TIMER;
    }

#if 0
    /* Enable posted-interrupt processing. */
    if (pVM->hm.s.fPostedIntrs)
    {
        Assert(g_HmMsrs.u.vmx.PinCtls.n.allowed1  & VMX_PIN_CTLS_POSTED_INT);
        Assert(g_HmMsrs.u.vmx.ExitCtls.n.allowed1 & VMX_EXIT_CTLS_ACK_EXT_INT);
        fVal |= VMX_PIN_CTLS_POSTED_INT;
    }
#endif

    if ((fVal & fZap) != fVal)
    {
        LogRelFunc(("Invalid pin-based VM-execution controls combo! Cpu=%#RX32 fVal=%#RX32 fZap=%#RX32\n",
                    g_HmMsrs.u.vmx.PinCtls.n.allowed0, fVal, fZap));
        pVCpu->hm.s.u32HMError = VMX_UFC_CTRL_PIN_EXEC;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    /* Commit it to the VMCS and update our cache. */
    int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PIN_EXEC, fVal);
    AssertRC(rc);
    pVmcsInfo->u32PinCtls = fVal;

    return VINF_SUCCESS;
}


/**
 * Sets up secondary processor-based VM-execution controls in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static int hmR0VmxSetupVmcsProcCtls2(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    uint32_t       fVal = g_HmMsrs.u.vmx.ProcCtls2.n.allowed0;    /* Bits set here must be set in the VMCS. */
    uint32_t const fZap = g_HmMsrs.u.vmx.ProcCtls2.n.allowed1;    /* Bits cleared here must be cleared in the VMCS. */

    /* WBINVD causes a VM-exit. */
    if (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_WBINVD_EXIT)
        fVal |= VMX_PROC_CTLS2_WBINVD_EXIT;

    /* Enable EPT (aka nested-paging). */
    if (pVM->hmr0.s.fNestedPaging)
        fVal |= VMX_PROC_CTLS2_EPT;

    /* Enable the INVPCID instruction if we expose it to the guest and is supported
       by the hardware. Without this, guest executing INVPCID would cause a #UD. */
    if (   pVM->cpum.ro.GuestFeatures.fInvpcid
        && (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_INVPCID))
        fVal |= VMX_PROC_CTLS2_INVPCID;

    /* Enable VPID. */
    if (pVM->hmr0.s.vmx.fVpid)
        fVal |= VMX_PROC_CTLS2_VPID;

    /* Enable unrestricted guest execution. */
    if (pVM->hmr0.s.vmx.fUnrestrictedGuest)
        fVal |= VMX_PROC_CTLS2_UNRESTRICTED_GUEST;

#if 0
    if (pVM->hm.s.fVirtApicRegs)
    {
        /* Enable APIC-register virtualization. */
        Assert(g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_APIC_REG_VIRT);
        fVal |= VMX_PROC_CTLS2_APIC_REG_VIRT;

        /* Enable virtual-interrupt delivery. */
        Assert(g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VIRT_INTR_DELIVERY);
        fVal |= VMX_PROC_CTLS2_VIRT_INTR_DELIVERY;
    }
#endif

    /* Virtualize-APIC accesses if supported by the CPU. The virtual-APIC page is
       where the TPR shadow resides. */
    /** @todo VIRT_X2APIC support, it's mutually exclusive with this. So must be
     *        done dynamically. */
    if (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS)
    {
        fVal |= VMX_PROC_CTLS2_VIRT_APIC_ACCESS;
        hmR0VmxSetupVmcsApicAccessAddr(pVCpu);
   }

    /* Enable the RDTSCP instruction if we expose it to the guest and is supported
       by the hardware. Without this, guest executing RDTSCP would cause a #UD. */
    if (   pVM->cpum.ro.GuestFeatures.fRdTscP
        && (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_RDTSCP))
        fVal |= VMX_PROC_CTLS2_RDTSCP;

    /* Enable Pause-Loop exiting. */
    if (   (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_PAUSE_LOOP_EXIT)
        && pVM->hm.s.vmx.cPleGapTicks
        && pVM->hm.s.vmx.cPleWindowTicks)
    {
        fVal |= VMX_PROC_CTLS2_PAUSE_LOOP_EXIT;

        int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PLE_GAP, pVM->hm.s.vmx.cPleGapTicks);          AssertRC(rc);
        rc     = VMXWriteVmcs32(VMX_VMCS32_CTRL_PLE_WINDOW, pVM->hm.s.vmx.cPleWindowTicks);    AssertRC(rc);
    }

    if ((fVal & fZap) != fVal)
    {
        LogRelFunc(("Invalid secondary processor-based VM-execution controls combo! cpu=%#RX32 fVal=%#RX32 fZap=%#RX32\n",
                    g_HmMsrs.u.vmx.ProcCtls2.n.allowed0, fVal, fZap));
        pVCpu->hm.s.u32HMError = VMX_UFC_CTRL_PROC_EXEC2;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    /* Commit it to the VMCS and update our cache. */
    int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC2, fVal);
    AssertRC(rc);
    pVmcsInfo->u32ProcCtls2 = fVal;

    return VINF_SUCCESS;
}


/**
 * Sets up processor-based VM-execution controls in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static int hmR0VmxSetupVmcsProcCtls(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    uint32_t       fVal = g_HmMsrs.u.vmx.ProcCtls.n.allowed0;     /* Bits set here must be set in the VMCS. */
    uint32_t const fZap = g_HmMsrs.u.vmx.ProcCtls.n.allowed1;     /* Bits cleared here must be cleared in the VMCS. */

    fVal |= VMX_PROC_CTLS_HLT_EXIT                                    /* HLT causes a VM-exit. */
         |  VMX_PROC_CTLS_USE_TSC_OFFSETTING                          /* Use TSC-offsetting. */
         |  VMX_PROC_CTLS_MOV_DR_EXIT                                 /* MOV DRx causes a VM-exit. */
         |  VMX_PROC_CTLS_UNCOND_IO_EXIT                              /* All IO instructions cause a VM-exit. */
         |  VMX_PROC_CTLS_RDPMC_EXIT                                  /* RDPMC causes a VM-exit. */
         |  VMX_PROC_CTLS_MONITOR_EXIT                                /* MONITOR causes a VM-exit. */
         |  VMX_PROC_CTLS_MWAIT_EXIT;                                 /* MWAIT causes a VM-exit. */

    /* We toggle VMX_PROC_CTLS_MOV_DR_EXIT later, check if it's not -always- needed to be set or clear. */
    if (   !(g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_MOV_DR_EXIT)
        ||  (g_HmMsrs.u.vmx.ProcCtls.n.allowed0 & VMX_PROC_CTLS_MOV_DR_EXIT))
    {
        pVCpu->hm.s.u32HMError = VMX_UFC_CTRL_PROC_MOV_DRX_EXIT;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    /* Without nested paging, INVLPG (also affects INVPCID) and MOV CR3 instructions should cause VM-exits. */
    if (!pVM->hmr0.s.fNestedPaging)
    {
        Assert(!pVM->hmr0.s.vmx.fUnrestrictedGuest);
        fVal |= VMX_PROC_CTLS_INVLPG_EXIT
             |  VMX_PROC_CTLS_CR3_LOAD_EXIT
             |  VMX_PROC_CTLS_CR3_STORE_EXIT;
    }

    /* Use TPR shadowing if supported by the CPU. */
    if (   PDMHasApic(pVM)
        && (g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_TPR_SHADOW))
    {
        fVal |= VMX_PROC_CTLS_USE_TPR_SHADOW;                /* CR8 reads from the Virtual-APIC page. */
                                                             /* CR8 writes cause a VM-exit based on TPR threshold. */
        Assert(!(fVal & VMX_PROC_CTLS_CR8_STORE_EXIT));
        Assert(!(fVal & VMX_PROC_CTLS_CR8_LOAD_EXIT));
        hmR0VmxSetupVmcsVirtApicAddr(pVmcsInfo);
    }
    else
    {
        /* Some 32-bit CPUs do not support CR8 load/store exiting as MOV CR8 is
           invalid on 32-bit Intel CPUs. Set this control only for 64-bit guests. */
        if (pVM->hmr0.s.fAllow64BitGuests)
            fVal |= VMX_PROC_CTLS_CR8_STORE_EXIT             /* CR8 reads cause a VM-exit. */
                 |  VMX_PROC_CTLS_CR8_LOAD_EXIT;             /* CR8 writes cause a VM-exit. */
    }

    /* Use MSR-bitmaps if supported by the CPU. */
    if (g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_MSR_BITMAPS)
    {
        fVal |= VMX_PROC_CTLS_USE_MSR_BITMAPS;
        hmR0VmxSetupVmcsMsrBitmapAddr(pVmcsInfo);
    }

    /* Use the secondary processor-based VM-execution controls if supported by the CPU. */
    if (g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_SECONDARY_CTLS)
        fVal |= VMX_PROC_CTLS_USE_SECONDARY_CTLS;

    if ((fVal & fZap) != fVal)
    {
        LogRelFunc(("Invalid processor-based VM-execution controls combo! cpu=%#RX32 fVal=%#RX32 fZap=%#RX32\n",
                    g_HmMsrs.u.vmx.ProcCtls.n.allowed0, fVal, fZap));
        pVCpu->hm.s.u32HMError = VMX_UFC_CTRL_PROC_EXEC;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    /* Commit it to the VMCS and update our cache. */
    int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC, fVal);
    AssertRC(rc);
    pVmcsInfo->u32ProcCtls = fVal;

    /* Set up MSR permissions that don't change through the lifetime of the VM. */
    if (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS)
        hmR0VmxSetupVmcsMsrPermissions(pVCpu, pVmcsInfo);

    /* Set up secondary processor-based VM-execution controls if the CPU supports it. */
    if (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_SECONDARY_CTLS)
        return hmR0VmxSetupVmcsProcCtls2(pVCpu, pVmcsInfo);

    /* Sanity check, should not really happen. */
    if (RT_LIKELY(!pVM->hmr0.s.vmx.fUnrestrictedGuest))
    { /* likely */ }
    else
    {
        pVCpu->hm.s.u32HMError = VMX_UFC_INVALID_UX_COMBO;
        return VERR_HM_UNSUPPORTED_CPU_FEATURE_COMBO;
    }

    /* Old CPUs without secondary processor-based VM-execution controls would end up here. */
    return VINF_SUCCESS;
}


/**
 * Sets up miscellaneous (everything other than Pin, Processor and secondary
 * Processor-based VM-execution) control fields in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static int hmR0VmxSetupVmcsMiscCtls(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    if (pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fUseVmcsShadowing)
    {
        hmR0VmxSetupVmcsVmreadBitmapAddr(pVCpu);
        hmR0VmxSetupVmcsVmwriteBitmapAddr(pVCpu);
    }
#endif

    //安全要求：确保未嵌套时 VMCS Link Pointer 为无效值（NIL_RTHCPHYS）。
    Assert(pVmcsInfo->u64VmcsLinkPtr == NIL_RTHCPHYS);
    int rc = VMXWriteVmcs64(VMX_VMCS64_GUEST_VMCS_LINK_PTR_FULL, NIL_RTHCPHYS);
    AssertRC(rc);

    //自动加载/存储的 MSR 地址：优化 VM-Exit/Entry 时的 MSR 保存恢复。
    //设置 VMX_VMCS64_GUEST_MSR_LOAD_FULL 等字段，指定 VM-Entry/Exit 时自动保存/恢复的 MSR 列表。
    rc = hmR0VmxSetupVmcsAutoLoadStoreMsrAddrs(pVmcsInfo);
    if (RT_SUCCESS(rc))
    {
        //CR0/CR4 掩码：强制虚拟机使用的控制寄存器固定位。
        /*掩码生成逻辑
           CR0：强制 PE（保护模式）、PG（分页）等位为 1，WP（写保护）等位可配置。
           CR4：强制 VMXE（VMX 启用）、PAE（物理地址扩展）等位。

         硬件强制：虚拟机运行时若尝试修改掩码位，触发 VM-Exit。
        */
        uint64_t const u64Cr0Mask = vmxHCGetFixedCr0Mask(pVCpu);
        uint64_t const u64Cr4Mask = vmxHCGetFixedCr4Mask(pVCpu);

        rc = VMXWriteVmcsNw(VMX_VMCS_CTRL_CR0_MASK, u64Cr0Mask);    AssertRC(rc);
        rc = VMXWriteVmcsNw(VMX_VMCS_CTRL_CR4_MASK, u64Cr4Mask);    AssertRC(rc);

        pVmcsInfo->u64Cr0Mask = u64Cr0Mask;
        pVmcsInfo->u64Cr4Mask = u64Cr4Mask;

        //调试控制：LBR（Last Branch Record）功能配置。
        if (pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fLbr)
        {
            rc = VMXWriteVmcsNw(VMX_VMCS64_GUEST_DEBUGCTL_FULL, MSR_IA32_DEBUGCTL_LBR);
            AssertRC(rc);
        }
        return VINF_SUCCESS;
    }
    else
        LogRelFunc(("Failed to initialize VMCS auto-load/store MSR addresses. rc=%Rrc\n", rc));
    return rc;
}


/**
 * Sets up the initial exception bitmap in the VMCS based on static conditions.
 *
 * We shall setup those exception intercepts that don't change during the
 * lifetime of the VM here. The rest are done dynamically while loading the
 * guest state.
 *
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static void hmR0VmxSetupVmcsXcptBitmap(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo)
{
    /*
     * The following exceptions are always intercepted:
     *
     * #AC - To prevent the guest from hanging the CPU and for dealing with
     *       split-lock detecting host configs.
     * #DB - To maintain the DR6 state even when intercepting DRx reads/writes and
     *       recursive #DBs can cause a CPU hang.
     * #PF - To sync our shadow page tables when nested-paging is not used.
     */
    bool const fNestedPaging = pVCpu->CTX_SUFF(pVM)->hmr0.s.fNestedPaging;
    uint32_t const uXcptBitmap = RT_BIT(X86_XCPT_AC)
                               | RT_BIT(X86_XCPT_DB)
                               | (fNestedPaging ? 0 : RT_BIT(X86_XCPT_PF));

    /* Commit it to the VMCS. */
    int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_EXCEPTION_BITMAP, uXcptBitmap);
    AssertRC(rc);

    /* Update our cache of the exception bitmap. */
    pVmcsInfo->u32XcptBitmap = uXcptBitmap;
}


#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
/**
 * Sets up the VMCS for executing a nested-guest using hardware-assisted VMX.
 *
 * @returns VBox status code.
 * @param   pVmcsInfo   The VMCS info. object.
 */
static int hmR0VmxSetupVmcsCtlsNested(PVMXVMCSINFO pVmcsInfo)
{
    Assert(pVmcsInfo->u64VmcsLinkPtr == NIL_RTHCPHYS);
    int rc = VMXWriteVmcs64(VMX_VMCS64_GUEST_VMCS_LINK_PTR_FULL, NIL_RTHCPHYS);
    AssertRC(rc);

    rc = hmR0VmxSetupVmcsAutoLoadStoreMsrAddrs(pVmcsInfo);
    if (RT_SUCCESS(rc))
    {
        if (g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_MSR_BITMAPS)
            hmR0VmxSetupVmcsMsrBitmapAddr(pVmcsInfo);

        /* Paranoia - We've not yet initialized these, they shall be done while merging the VMCS. */
        Assert(!pVmcsInfo->u64Cr0Mask);
        Assert(!pVmcsInfo->u64Cr4Mask);
        return VINF_SUCCESS;
    }
    LogRelFunc(("Failed to set up the VMCS link pointer in the nested-guest VMCS. rc=%Rrc\n", rc));
    return rc;
}
#endif


/**
 * Selector FNHMSVMVMRUN implementation.
 */
static DECLCALLBACK(int) hmR0VmxStartVmSelector(PVMXVMCSINFO pVmcsInfo, PVMCPUCC pVCpu, bool fResume)
{
    hmR0VmxUpdateStartVmFunction(pVCpu);
    return pVCpu->hmr0.s.vmx.pfnStartVm(pVmcsInfo, pVCpu, fResume);
}


/**
 * Sets up the VMCS for executing a guest (or nested-guest) using hardware-assisted
 * VMX.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmcsInfo       The VMCS info. object.
 * @param   fIsNstGstVmcs   Whether this is a nested-guest VMCS.
 */
/*
作用：配置 VMCS（Virtual Machine Control Structure），为虚拟机的硬件辅助执行准备控制环境。
调用时机：在 vCPU 启动或迁移到新物理 CPU 时调用，属于 Ring-0 层级的硬件虚拟化操作。
核心任务：
  初始化 VMCS 的硬件修订标识符。
  设置三类关键控制字段（Pin-Based、Processor-Based、Miscellaneous）。
  处理嵌套虚拟化（VMX Shadowing）的特殊配置。
  同步 VMCS 状态并记录错误信息。
*/
/*
 *VMCS 生命周期管理
初始化流程：VMCLEAR → VMPTRLD → VMWRITE 确保字段正确配置。
字段分类：
  Guest-State Area：保存 vCPU 寄存器状态（如 RIP、RSP）。
  Host-State Area：定义 VM-Exit 后处理器的恢复状态。
  Control Fields：控制虚拟化行为（如 CPU_BASED_HLT_EXIT）。

2) 嵌套虚拟化支持‌
  Shadow VMCS：充当 L1 虚拟机与 L2 虚拟机间的桥梁，减少 VM-Exit 开销。
  启用条件：PROCBASED_CTLS2_VMCS_SHADOWING 置位，且 VMCS 修订 ID 匹配。

3) 异常与中断处理
  Exception Bitmap：32 位掩码，每位对应一个异常（如 bit 14 对应 #PF）。
  Pin-Based Controls：NMI_EXITING 控制非屏蔽中断的虚拟化方式。

 * */
static int hmR0VmxSetupVmcs(PVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfo, bool fIsNstGstVmcs)
{
    Assert(pVmcsInfo->pvVmcs);// 确保 VMCS 内存区域有效
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));// 必须禁用抢占（原子操作）

    /* Set the CPU specified revision identifier at the beginning of the VMCS structure. */
    *(uint32_t *)pVmcsInfo->pvVmcs = RT_BF_GET(g_HmMsrs.u.vmx.u64Basic, VMX_BF_BASIC_VMCS_ID);//从 IA32_VMX_BASIC MSR 获取，标识硬件兼容性版本。
    const char * const pszVmcs     = fIsNstGstVmcs ? "nested-guest VMCS" : "guest VMCS";

    LogFlowFunc(("\n"));

    /*
     * Initialize the VMCS using VMCLEAR before loading the VMCS.
     * See Intel spec. 31.6 "Preparation And Launching A Virtual Machine".
     */
    //将 VMCS 置为未激活状态，清除内部缓存数据。
    int rc = hmR0VmxClearVmcs(pVmcsInfo);// 执行 VMCLEAR 初始化 VMCS
    if (RT_SUCCESS(rc))
    {
        //将 VMCS 绑定到当前 CPU，后续操作针对此 VMCS。
        rc = hmR0VmxLoadVmcs(pVmcsInfo); // 执行 VMPTRLD 加载 VMCS
        if (RT_SUCCESS(rc))
        {
            /*
             * Initialize the hardware-assisted VMX execution handler for guest and nested-guest VMCS.
             * The host is always 64-bit since we no longer support 32-bit hosts.
             * Currently we have just a single handler for all guest modes as well, see @bugref{6208#c73}.
             */
            if (!fIsNstGstVmcs)//非嵌套虚拟化场景
            {
                //处理外部中断（如 NMI）的虚拟化行为。
                rc = hmR0VmxSetupVmcsPinCtls(pVCpu, pVmcsInfo); // 设置 Pin-Based 控制
                if (RT_SUCCESS(rc))
                {
                    //控制关键指令（如 HLT、MOV DRx）的拦截。
                    rc = hmR0VmxSetupVmcsProcCtls(pVCpu, pVmcsInfo);// 设置 Processor-Based 控制
                    if (RT_SUCCESS(rc))
                    {
                        //配置 CR3 目标值数量、MSR 加载列表等。
                        rc = hmR0VmxSetupVmcsMiscCtls(pVCpu, pVmcsInfo);// 设置 Miscellaneous 控制
                        if (RT_SUCCESS(rc))
                        {
                            //指定哪些异常触发 VM-Exit（如 #PF）。
                            hmR0VmxSetupVmcsXcptBitmap(pVCpu, pVmcsInfo);// 配置异常位图
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
                            /*
                             * If a shadow VMCS is allocated for the VMCS info. object, initialize the
                             * VMCS revision ID and shadow VMCS indicator bit. Also, clear the VMCS
                             * making it fit for use when VMCS shadowing is later enabled.
                             */
                            if (pVmcsInfo->pvShadowVmcs)
                            {
                                VMXVMCSREVID VmcsRevId;
                                VmcsRevId.u = RT_BF_GET(g_HmMsrs.u.vmx.u64Basic, VMX_BF_BASIC_VMCS_ID);
                                VmcsRevId.n.fIsShadowVmcs = 1;
                                *(uint32_t *)pVmcsInfo->pvShadowVmcs = VmcsRevId.u;
                                rc = vmxHCClearShadowVmcs(pVmcsInfo);
                                if (RT_SUCCESS(rc))
                                { /* likely */ }
                                else
                                    LogRelFunc(("Failed to initialize shadow VMCS. rc=%Rrc\n", rc));
                            }
#endif
                        }
                        else
                            LogRelFunc(("Failed to setup miscellaneous controls. rc=%Rrc\n", rc));
                    }
                    else
                        LogRelFunc(("Failed to setup processor-based VM-execution controls. rc=%Rrc\n", rc));
                }
                else
                    LogRelFunc(("Failed to setup pin-based controls. rc=%Rrc\n", rc));
            }
            else
            {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
                rc = hmR0VmxSetupVmcsCtlsNested(pVmcsInfo);
                if (RT_SUCCESS(rc))
                { /* likely */ }
                else
                    LogRelFunc(("Failed to initialize nested-guest VMCS. rc=%Rrc\n", rc));
#else
                AssertFailed();
#endif
            }
        }
        else
            LogRelFunc(("Failed to load the %s. rc=%Rrc\n", pszVmcs, rc));
    }
    else
        LogRelFunc(("Failed to clear the %s. rc=%Rrc\n", rc, pszVmcs));

    /* Sync any CPU internal VMCS data back into our VMCS in memory. */
    if (RT_SUCCESS(rc))
    {
        //VMCLEAR 确保硬件缓存数据写回内存。
        rc = hmR0VmxClearVmcs(pVmcsInfo);//同步 VMCS 数据回内存
        if (RT_SUCCESS(rc))
        { /* likely */ }
        else
            LogRelFunc(("Failed to clear the %s post setup. rc=%Rrc\n", rc, pszVmcs));
    }

    /*
     * Update the last-error record both for failures and success, so we
     * can propagate the status code back to ring-3 for diagnostics.
     */
    //通过 pVCpu->hm.s.vmx.LastError 传递错误码至用户态。
    hmR0VmxUpdateErrorRecord(pVCpu, rc);
    NOREF(pszVmcs);
    return rc;
}


/**
 * Does global VT-x initialization (called during module initialization).
 *
 * @returns VBox status code.
 */
//全局初始化 VMX（Intel VT-x）模块，为后续虚拟机运行准备必要的硬件环境
VMMR0DECL(int) VMXR0GlobalInit(void)
{
#ifdef HMVMX_USE_FUNCTION_TABLE
    //验证 VM-Exit 处理函数表的完整性（调试模式下）。
    AssertCompile(VMX_EXIT_MAX + 1 == RT_ELEMENTS(g_aVMExitHandlers));
# ifdef VBOX_STRICT
    for (unsigned i = 0; i < RT_ELEMENTS(g_aVMExitHandlers); i++)
        Assert(g_aVMExitHandlers[i].pfn);// 检查每个处理函数指针非空
# endif
#endif

    /*
     * For detecting whether DR6.RTM is writable or not (done in VMXR0InitVM).
     */
    //检测调试寄存器 DR6 的可写性，确定 RTM（Restricted Transactional Memory）位的处理策略
    RTTHREADPREEMPTSTATE Preempt = RTTHREADPREEMPTSTATE_INITIALIZER;
    RTThreadPreemptDisable(&Preempt);
    RTCCUINTXREG const fSavedDr6 = ASMGetDR6(); // 保存原始 DR6
    ASMSetDR6(0);// 尝试清零 DR6
    RTCCUINTXREG const fZeroDr6  = ASMGetDR6();//读取实际值
    ASMSetDR6(fSavedDr6);//恢复原始值
    RTThreadPreemptRestore(&Preempt);

    g_fDr6Zeroed = fZeroDr6;// 记录 DR6 的可清零位掩码

    return VINF_SUCCESS;
}


/**
 * Does global VT-x termination (called during module termination).
 */
VMMR0DECL(void) VMXR0GlobalTerm()
{
    /* Nothing to do currently. */
}


/**
 * Sets up and activates VT-x on the current CPU.
 *
 * @returns VBox status code.
 * @param   pHostCpu        The HM physical-CPU structure.
 * @param   pVM             The cross context VM structure.  Can be
 *                          NULL after a host resume operation.
 * @param   pvCpuPage       Pointer to the VMXON region (can be NULL if @a
 *                          fEnabledByHost is @c true).
 * @param   HCPhysCpuPage   Physical address of the VMXON region (can be 0 if
 *                          @a fEnabledByHost is @c true).
 * @param   fEnabledByHost  Set if SUPR0EnableVTx() or similar was used to
 *                          enable VT-x on the host.
 * @param   pHwvirtMsrs     Pointer to the hardware-virtualization MSRs.
 */
/*
  启用 VMX 根模式（若主机未全局启用）。
  刷新 EPT TLB 以防止残留映射。
  标记 VPID（Virtual Processor Identifier）需要刷新，确保隔离性。
*/
VMMR0DECL(int) VMXR0EnableCpu(PHMPHYSCPU pHostCpu, PVMCC pVM, void *pvCpuPage, RTHCPHYS HCPhysCpuPage, bool fEnabledByHost,
                              PCSUPHWVIRTMSRS pHwvirtMsrs)
{
    AssertPtr(pHostCpu);
    AssertPtr(pHwvirtMsrs); // 确保硬件虚拟化 MSR 指针有效
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD)); // 必须禁用抢占

    /* Enable VT-x if it's not already enabled by the host. */
    if (!fEnabledByHost)
    {
        int rc = hmR0VmxEnterRootMode(pHostCpu, pVM, HCPhysCpuPage, pvCpuPage);
        if (RT_FAILURE(rc))
            return rc;
    }

    /*
     * Flush all EPT tagged-TLB entries (in case VirtualBox or any other hypervisor have been
     * using EPTPs) so we don't retain any stale guest-physical mappings which won't get
     * invalidated when flushing by VPID.
     */
    if (pHwvirtMsrs->u.vmx.u64EptVpidCaps & MSR_IA32_VMX_EPT_VPID_CAP_INVEPT_ALL_CONTEXTS)
    {
        // 刷新所有 EPT 上下文
        hmR0VmxFlushEpt(NULL /* pVCpu */, NULL /* pVmcsInfo */, VMXTLBFLUSHEPT_ALL_CONTEXTS);
        //标记无需后续刷新
        pHostCpu->fFlushAsidBeforeUse = false;
    }
    else
        //如果按VCPU刷新TLB的话，设置标志，不在此处处理
        pHostCpu->fFlushAsidBeforeUse = true;

    /* Ensure each VCPU scheduled on this CPU gets a new VPID on resume. See @bugref{6255}. */
    ++pHostCpu->cTlbFlushes;

    return VINF_SUCCESS;
}


/**
 * Deactivates VT-x on the current CPU.
 *
 * @returns VBox status code.
 * @param   pHostCpu        The HM physical-CPU structure.
 * @param   pvCpuPage       Pointer to the VMXON region.
 * @param   HCPhysCpuPage   Physical address of the VMXON region.
 *
 * @remarks This function should never be called when SUPR0EnableVTx() or
 *          similar was used to enable VT-x on the host.
 */
VMMR0DECL(int) VMXR0DisableCpu(PHMPHYSCPU pHostCpu, void *pvCpuPage, RTHCPHYS HCPhysCpuPage)
{
    RT_NOREF2(pvCpuPage, HCPhysCpuPage);

    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    return hmR0VmxLeaveRootMode(pHostCpu);
}


/**
 * Does per-VM VT-x initialization.
 *
 * @returns VBox status code.
 * @param   pVM             The cross context VM structure.
 */
/*
核心任务：
  分配并初始化 VMX 专用结构（如 VMCS、EPT 页表）。
  配置调试寄存器（DR6）的拦截策略。
  传递关键配置信息至 Ring-3（用户态）
*/
VMMR0DECL(int) VMXR0InitVM(PVMCC pVM)
{
    AssertPtr(pVM);
    LogFlowFunc(("pVM=%p\n", pVM));

    // 设置默认的 VMCS 字段值
    hmR0VmxStructsInit(pVM);
    //EPT：通过 hmR0VmxStructsAlloc 分配页表，后续由 VMXR0SetupTaggedTlb 配置刷新策略。
    int rc = hmR0VmxStructsAlloc(pVM);// 分配内存（VMCS、EPT , VPID等）
    if (RT_FAILURE(rc))
    {
        LogRelFunc(("Failed to allocated VMX structures. rc=%Rrc\n", rc));
        return rc;
    }

    /* Setup the crash dump page. */
#ifdef VBOX_WITH_CRASHDUMP_MAGIC
    strcpy((char *)pVM->hmr0.s.vmx.pbScratch, "SCRATCH Magic");
    *(uint64_t *)(pVM->hmr0.s.vmx.pbScratch + 16) = UINT64_C(0xdeadbeefdeadbeef);
#endif

    /*
     * Copy out stuff that's for ring-3 and determin default configuration.
     */
    pVM->hm.s.ForR3.vmx.u64HostDr6Zeroed = g_fDr6Zeroed;

    /* Since we do not emulate RTM, make sure DR6.RTM cannot be cleared by the
       guest and cause confusion there.  It appears that the DR6.RTM bit can be
       cleared even if TSX-NI is disabled (microcode update / system / whatever). */
#ifdef VMX_WITH_MAYBE_ALWAYS_INTERCEPT_MOV_DRX
    //默认策略：若未显式配置（fAlwaysInterceptMovDRxCfg == 0），
    //根据主机 DR6 的保留位（RA1）决定是否拦截 MOV DRx 指令。
    //若主机 DR6 的 RA1 位未被清零，需拦截以防止客户机修改保留位。
    if (pVM->hm.s.vmx.fAlwaysInterceptMovDRxCfg == 0)
        pVM->hmr0.s.vmx.fAlwaysInterceptMovDRx = g_fDr6Zeroed != X86_DR6_RA1_MASK;
    else
#endif
    //显式配置：若用户指定（fAlwaysInterceptMovDRxCfg > 0），强制启用拦截。
        pVM->hmr0.s.vmx.fAlwaysInterceptMovDRx = pVM->hm.s.vmx.fAlwaysInterceptMovDRxCfg > 0;
    //通过 hm.s.ForR3 结构传递配置（如 fAlwaysInterceptMovDRx），供用户态组件（如 VBoxSVC）使用。
    pVM->hm.s.ForR3.vmx.fAlwaysInterceptMovDRx = pVM->hmr0.s.vmx.fAlwaysInterceptMovDRx;

    return VINF_SUCCESS;
}


/**
 * Does per-VM VT-x termination.
 *
 * @returns VBox status code.
 * @param   pVM     The cross context VM structure.
 */
VMMR0DECL(int) VMXR0TermVM(PVMCC pVM)
{
    AssertPtr(pVM);
    LogFlowFunc(("pVM=%p\n", pVM));

//在支持崩溃转储的调试版本中，清除临时内存（pbScratch）的内容，避免敏感信息泄漏
#ifdef VBOX_WITH_CRASHDUMP_MAGIC
    if (pVM->hmr0.s.vmx.pbScratch)
        RT_BZERO(pVM->hmr0.s.vmx.pbScratch, X86_PAGE_4K_SIZE);
#endif
    hmR0VmxStructsFree(pVM);
    return VINF_SUCCESS;
}


/**
 * Sets up the VM for execution using hardware-assisted VMX.
 * This function is only called once per-VM during initialization.
 *
 * @returns VBox status code.
 * @param   pVM     The cross context VM structure.
 */
VMMR0DECL(int) VMXR0SetupVM(PVMCC pVM)
{
    AssertPtr(pVM);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    LogFlowFunc(("pVM=%p\n", pVM));

    /*
     * At least verify if VMX is enabled, since we can't check if we're in VMX root mode or not
     * without causing a #GP.
     */
    RTCCUINTREG const uHostCr4 = ASMGetCR4();
    //检查 VMX 是否在 CR4 寄存器中启用
    if (RT_LIKELY(uHostCr4 & X86_CR4_VMXE))
    { /* likely */ }
    else
        return VERR_VMX_NOT_IN_VMX_ROOT_MODE;

    /*
     * Check that nested paging is supported if enabled and copy over the flag to the
     * ring-0 only structure.
     */
    //验证嵌套分页（Nested Paging/EPT）是否被正确配置
    bool const fNestedPaging = pVM->hm.s.fNestedPagingCfg;
    AssertReturn(   !fNestedPaging
                 || (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_EPT), /** @todo use a ring-0 copy of ProcCtls2.n.allowed1 */
                 VERR_INCOMPATIBLE_CONFIG);
    pVM->hmr0.s.fNestedPaging = fNestedPaging;
    pVM->hmr0.s.fAllow64BitGuests = pVM->hm.s.fAllow64BitGuestsCfg;

    /*
     * Without unrestricted guest execution, pRealModeTSS and pNonPagingModeEPTPageTable *must*
     * always be allocated. We no longer support the highly unlikely case of unrestricted guest
     * without pRealModeTSS, see hmR3InitFinalizeR0Intel().
     */
    //检查 unrestricted guest 模式（不需要分页/实模式转换）的兼容性
    bool const fUnrestrictedGuest = pVM->hm.s.vmx.fUnrestrictedGuestCfg;
    AssertReturn(   !fUnrestrictedGuest
                || (   (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_UNRESTRICTED_GUEST)
                    && fNestedPaging),
                    VERR_INCOMPATIBLE_CONFIG);
    if (   !fUnrestrictedGuest
        &&  (   !pVM->hm.s.vmx.pNonPagingModeEPTPageTable
             || !pVM->hm.s.vmx.pRealModeTSS))
    {
        LogRelFunc(("Invalid real-on-v86 state.\n"));
        return VERR_INTERNAL_ERROR;
    }
    pVM->hmr0.s.vmx.fUnrestrictedGuest = fUnrestrictedGuest;

    /* Initialize these always, see hmR3InitFinalizeR0().*/
    //设置 EPT 和 VPID 的 TLB 刷新方式
    pVM->hm.s.ForR3.vmx.enmTlbFlushEpt  = pVM->hmr0.s.vmx.enmTlbFlushEpt  = VMXTLBFLUSHEPT_NONE;
    pVM->hm.s.ForR3.vmx.enmTlbFlushVpid = pVM->hmr0.s.vmx.enmTlbFlushVpid = VMXTLBFLUSHVPID_NONE;

    /* Setup the tagged-TLB flush handlers. */
    //初始化标记 TLB（tagged TLB）处理程序
    int rc = hmR0VmxSetupTaggedTlb(pVM);
    if (RT_FAILURE(rc))
    {
        LogRelFunc(("Failed to setup tagged TLB. rc=%Rrc\n", rc));
        return rc;
    }

    /* Determine LBR capabilities. */
    //初始化标记 TLB（tagged TLB）处理程序
    pVM->hmr0.s.vmx.fLbr = pVM->hm.s.vmx.fLbrCfg;
    if (pVM->hmr0.s.vmx.fLbr)
    {
        rc = hmR0VmxSetupLbrMsrRange(pVM);
        if (RT_FAILURE(rc))
        {
            LogRelFunc(("Failed to setup LBR MSR range. rc=%Rrc\n", rc));
            return rc;
        }
    }

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    /* Setup the shadow VMCS fields array and VMREAD/VMWRITE bitmaps. */
    if (pVM->hmr0.s.vmx.fUseVmcsShadowing)
    {
        rc = hmR0VmxSetupShadowVmcsFieldsArrays(pVM);
        if (RT_SUCCESS(rc))
            hmR0VmxSetupVmreadVmwriteBitmaps(pVM);
        else
        {
            LogRelFunc(("Failed to setup shadow VMCS fields arrays. rc=%Rrc\n", rc));
            return rc;
        }
    }
#endif

    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PVMCPUCC pVCpu = VMCC_GET_CPU(pVM, idCpu);
        Log4Func(("pVCpu=%p idCpu=%RU32\n", pVCpu, pVCpu->idCpu));

        pVCpu->hmr0.s.vmx.pfnStartVm = hmR0VmxStartVmSelector;

        //为每个虚拟 CPU 设置 VMCS（虚拟机控制结构）
        rc = hmR0VmxSetupVmcs(pVCpu, &pVCpu->hmr0.s.vmx.VmcsInfo,  false /* fIsNstGstVmcs */);
        if (RT_SUCCESS(rc))
        {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
            if (pVM->cpum.ro.GuestFeatures.fVmx)
            {
                rc = hmR0VmxSetupVmcs(pVCpu, &pVCpu->hmr0.s.vmx.VmcsInfoNstGst, true /* fIsNstGstVmcs */);
                if (RT_SUCCESS(rc))
                { /* likely */ }
                else
                {
                    LogRelFunc(("Nested-guest VMCS setup failed. rc=%Rrc\n", rc));
                    return rc;
                }
            }
#endif
        }
        else
        {
            LogRelFunc(("VMCS setup failed. rc=%Rrc\n", rc));
            return rc;
        }
    }

    return VINF_SUCCESS;
}


/**
 * Saves the host control registers (CR0, CR3, CR4) into the host-state area in
 * the VMCS.
 * @returns CR4 for passing along to hmR0VmxExportHostSegmentRegs.
 */
static uint64_t hmR0VmxExportHostControlRegs(void)
{
    int rc = VMXWriteVmcsNw(VMX_VMCS_HOST_CR0, ASMGetCR0());    AssertRC(rc);
    rc     = VMXWriteVmcsNw(VMX_VMCS_HOST_CR3, ASMGetCR3());    AssertRC(rc);
    uint64_t uHostCr4 = ASMGetCR4();
    rc     = VMXWriteVmcsNw(VMX_VMCS_HOST_CR4, uHostCr4);       AssertRC(rc);
    return uHostCr4;
}


/**
 * Saves the host segment registers and GDTR, IDTR, (TR, GS and FS bases) into
 * the host-state area in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   uHostCr4    The host CR4 value.
 */
//该函数在 VM-entry 前 将宿主机的段寄存器（Segment Registers）和系统表基址（GDTR/IDTR/TR）
//导出到 VMCS 的 Host-State Area，确保 VM-exit 时能正确恢复宿主机状态。
/*
  段寄存器（CS/SS/DS/ES/FS/GS/TR）的选择子和基址。
  系统描述符表（GDTR/IDTR）的基址和限制。
  宿主机状态恢复标志的维护（如需要手动恢复的寄存器）。
*/
//fRestoreHostFlags |= xxxx; 配置这个标志的意思是,vmx exit的时候，由软件恢复host的这些寄存器
//这是由 Intel VT-x 硬件行为与宿主机需求之间的差异所决定的。
static int hmR0VmxExportHostSegmentRegs(PVMCPUCC pVCpu, uint64_t uHostCr4)
{
    /*
     * If we've executed guest code using hardware-assisted VMX, the host-state bits
     * will be messed up. We should -not- save the messed up state without restoring
     * the original host-state, see @bugref{7240}.
     *
     * This apparently can happen (most likely the FPU changes), deal with it rather than
     * asserting. Was observed booting Solaris 10u10 32-bit guest.
     */
    //若之前执行客户机代码时污染了宿主机状态（如 FPU 寄存器），需先恢复原始值。
    //典型问题：
    //Solaris 客户机可能触发此路径（参考 Bug #7240）。
    if (pVCpu->hmr0.s.vmx.fRestoreHostFlags > VMX_RESTORE_HOST_REQUIRED)
    {
        Log4Func(("Restoring Host State: fRestoreHostFlags=%#RX32 HostCpuId=%u\n", pVCpu->hmr0.s.vmx.fRestoreHostFlags,
                  pVCpu->idCpu));
        VMXRestoreHostState(pVCpu->hmr0.s.vmx.fRestoreHostFlags, &pVCpu->hmr0.s.vmx.RestoreHost); // 恢复被客户机污染的宿主机状态
        pVCpu->hmr0.s.vmx.fRestoreHostFlags = 0;
    }

    /*
     * Get all the host info.
     * ASSUME it is safe to use rdfsbase and friends if the CR4.FSGSBASE bit is set
     * without also checking the cpuid bit.
     */
    uint32_t fRestoreHostFlags;
#if RT_INLINE_ASM_EXTERNAL
    //根据 CR4.FSGSBASE 决定使用 RDFSBASE 指令还是 RDMSR 读取 FS/GS 基址。
    if (uHostCr4 & X86_CR4_FSGSBASE)
    {
        //若支持 FSGSBASE，设置 VMX_RESTORE_HOST_CAN_USE_WRFSBASE_AND_WRGSBASE 标志，加速后续恢复。
        hmR0VmxExportHostSegmentRegsAsmHlp(&pVCpu->hmr0.s.vmx.RestoreHost, true /*fHaveFsGsBase*/);
        fRestoreHostFlags = VMX_RESTORE_HOST_CAN_USE_WRFSBASE_AND_WRGSBASE;
    }
    else
    {
        hmR0VmxExportHostSegmentRegsAsmHlp(&pVCpu->hmr0.s.vmx.RestoreHost, false /*fHaveFsGsBase*/);
        fRestoreHostFlags = 0;
    }
    RTSEL uSelES = pVCpu->hmr0.s.vmx.RestoreHost.uHostSelES;
    RTSEL uSelDS = pVCpu->hmr0.s.vmx.RestoreHost.uHostSelDS;
    RTSEL uSelFS = pVCpu->hmr0.s.vmx.RestoreHost.uHostSelFS;
    RTSEL uSelGS = pVCpu->hmr0.s.vmx.RestoreHost.uHostSelGS;
#else
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelTR = ASMGetTR();
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelSS = ASMGetSS();
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelCS = ASMGetCS();
    ASMGetGDTR((PRTGDTR)&pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr);
    ASMGetIDTR((PRTIDTR)&pVCpu->hmr0.s.vmx.RestoreHost.HostIdtr);
    if (uHostCr4 & X86_CR4_FSGSBASE)
    {
        pVCpu->hmr0.s.vmx.RestoreHost.uHostFSBase = ASMGetFSBase();
        pVCpu->hmr0.s.vmx.RestoreHost.uHostGSBase = ASMGetGSBase();
        fRestoreHostFlags = VMX_RESTORE_HOST_CAN_USE_WRFSBASE_AND_WRGSBASE;
    }
    else
    {
        pVCpu->hmr0.s.vmx.RestoreHost.uHostFSBase = ASMRdMsr(MSR_K8_FS_BASE);
        pVCpu->hmr0.s.vmx.RestoreHost.uHostGSBase = ASMRdMsr(MSR_K8_GS_BASE);
        fRestoreHostFlags = 0;
    }
    RTSEL uSelES, uSelDS, uSelFS, uSelGS;
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelDS = uSelDS = ASMGetDS();
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelES = uSelES = ASMGetES();
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelFS = uSelFS = ASMGetFS();
    pVCpu->hmr0.s.vmx.RestoreHost.uHostSelGS = uSelGS = ASMGetGS();
#endif

    /*
     * Determine if the host segment registers are suitable for VT-x. Otherwise use zero to
     * gain VM-entry and restore them before we get preempted.
     *
     * See Intel spec. 26.2.3 "Checks on Host Segment and Descriptor-Table Registers".
     */
    RTSEL const uSelAll = uSelFS | uSelGS | uSelES | uSelDS;
    //段寄存器合法性校验
    //Intel 规范要求：
    //VM-entry 时宿主机段选择子的 RPL（Requested Privilege Level）必须为 0，且不能指向 LDT。
    /*
     * 解决方案：
         非法选择子临时置零，通过 VM-entry设置恢复标志，在 VM-exit 后恢复原始值。
    */
    if (uSelAll & (X86_SEL_RPL | X86_SEL_LDT))
    {
        if (!(uSelAll & X86_SEL_LDT))
        {
                // 调整非法选择子（RPL≠0 或 LDT 选择子）
#define VMXLOCAL_ADJUST_HOST_SEG(a_Seg, a_uVmcsVar) \
                do { \
                    (a_uVmcsVar) = pVCpu->hmr0.s.vmx.RestoreHost.uHostSel##a_Seg; \
                    if ((a_uVmcsVar) & X86_SEL_RPL) \
                    { \
                        fRestoreHostFlags |= VMX_RESTORE_HOST_SEL_##a_Seg; \
                        (a_uVmcsVar) = 0; \
                    } \
                } while (0)
            // 检查 (a_uVmcsVar) = 0 : 临时置零以通过 VM-entry 
            VMXLOCAL_ADJUST_HOST_SEG(DS, uSelDS);
            VMXLOCAL_ADJUST_HOST_SEG(ES, uSelES);
            VMXLOCAL_ADJUST_HOST_SEG(FS, uSelFS);
            VMXLOCAL_ADJUST_HOST_SEG(GS, uSelGS);
#undef VMXLOCAL_ADJUST_HOST_SEG
        }
        else
        {
#define VMXLOCAL_ADJUST_HOST_SEG(a_Seg, a_uVmcsVar) \
                do { \
                    (a_uVmcsVar) = pVCpu->hmr0.s.vmx.RestoreHost.uHostSel##a_Seg; \
                    if ((a_uVmcsVar) & (X86_SEL_RPL | X86_SEL_LDT)) \
                    { \
                        if (!((a_uVmcsVar) & X86_SEL_LDT)) \
                            fRestoreHostFlags |= VMX_RESTORE_HOST_SEL_##a_Seg; \
                        else \
                        { \
                            uint32_t const fAttr = ASMGetSegAttr(a_uVmcsVar); \
                            if ((fAttr & X86_DESC_P) && fAttr != UINT32_MAX) \
                                fRestoreHostFlags |= VMX_RESTORE_HOST_SEL_##a_Seg; \
                        } \
                        (a_uVmcsVar) = 0; \
                    } \
                } while (0)
            VMXLOCAL_ADJUST_HOST_SEG(DS, uSelDS);
            VMXLOCAL_ADJUST_HOST_SEG(ES, uSelES);
            VMXLOCAL_ADJUST_HOST_SEG(FS, uSelFS);
            VMXLOCAL_ADJUST_HOST_SEG(GS, uSelGS);
#undef VMXLOCAL_ADJUST_HOST_SEG
        }
    }

    /* Verification based on Intel spec. 26.2.3 "Checks on Host Segment and Descriptor-Table Registers"  */
    Assert(!(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelTR & X86_SEL_RPL)); Assert(!(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelTR & X86_SEL_LDT)); Assert(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelTR);
    Assert(!(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelCS & X86_SEL_RPL)); Assert(!(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelCS & X86_SEL_LDT)); Assert(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelCS);
    Assert(!(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelSS & X86_SEL_RPL)); Assert(!(pVCpu->hmr0.s.vmx.RestoreHost.uHostSelSS & X86_SEL_LDT));
    Assert(!(uSelDS & X86_SEL_RPL)); Assert(!(uSelDS & X86_SEL_LDT));
    Assert(!(uSelES & X86_SEL_RPL)); Assert(!(uSelES & X86_SEL_LDT));
    Assert(!(uSelFS & X86_SEL_RPL)); Assert(!(uSelFS & X86_SEL_LDT));
    Assert(!(uSelGS & X86_SEL_RPL)); Assert(!(uSelGS & X86_SEL_LDT));

    /*
     * Determine if we need to manually need to restore the GDTR and IDTR limits as VT-x zaps
     * them to the maximum limit (0xffff) on every VM-exit.
     */
    /*
     硬件行为：
       VT-x 在 VM-exit 时强制将 GDTR/IDTR 限制设为 0xFFFF（可能破坏宿主机的精心配置）。
     兼容性处理：
       Linux/Solaris 允许 IDTR.LIMIT=0xFFF，其他系统需严格恢复。
       Windows 依赖精确的 GDTR 限制（如 Patch Guard）。
     * */
    if (pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr.cb != 0xffff)
        fRestoreHostFlags |= VMX_RESTORE_HOST_GDTR;

    /*
     * IDT limit is effectively capped at 0xfff. (See Intel spec. 6.14.1 "64-Bit Mode IDT" and
     * Intel spec. 6.2 "Exception and Interrupt Vectors".)  Therefore if the host has the limit
     * as 0xfff, VT-x bloating the limit to 0xffff shouldn't cause any different CPU behavior.
     * However, several hosts either insists on 0xfff being the limit (Windows Patch Guard) or
     * uses the limit for other purposes (darwin puts the CPU ID in there but botches sidt
     * alignment in at least one consumer).  So, we're only allowing the IDTR.LIMIT to be left
     * at 0xffff on hosts where we are sure it won't cause trouble.
     */
    //如果内存中的IDTR值有效，配置flag，vmx exit的时候，由软件恢复host的这些寄存器
#if defined(RT_OS_LINUX) || defined(RT_OS_SOLARIS)
    if (pVCpu->hmr0.s.vmx.RestoreHost.HostIdtr.cb <  0x0fff)
#else
    if (pVCpu->hmr0.s.vmx.RestoreHost.HostIdtr.cb != 0xffff)
#endif
        fRestoreHostFlags |= VMX_RESTORE_HOST_IDTR;

    /*
     * Host TR base. Verify that TR selector doesn't point past the GDT. Masking off the TI
     * and RPL bits is effectively what the CPU does for "scaling by 8". TI is always 0 and
     * RPL should be too in most cases.
     */
    RTSEL const uSelTR = pVCpu->hmr0.s.vmx.RestoreHost.uHostSelTR;
    AssertMsgReturn((uSelTR | X86_SEL_RPL_LDT) <= pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr.cb,
                    ("TR selector exceeds limit. TR=%RTsel cbGdt=%#x\n", uSelTR, pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr.cb),
                    VERR_VMX_INVALID_HOST_STATE);

    PCX86DESCHC pDesc = (PCX86DESCHC)(pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr.uAddr + (uSelTR & X86_SEL_MASK));
    uintptr_t const uTRBase = X86DESC64_BASE(pDesc);

    /*
     * VT-x unconditionally restores the TR limit to 0x67 and type to 11 (32-bit busy TSS) on
     * all VM-exits. The type is the same for 64-bit busy TSS[1]. The limit needs manual
     * restoration if the host has something else. Task switching is not supported in 64-bit
     * mode[2], but the limit still matters as IOPM is supported in 64-bit mode. Restoring the
     * limit lazily while returning to ring-3 is safe because IOPM is not applicable in ring-0.
     *
     * [1] See Intel spec. 3.5 "System Descriptor Types".
     * [2] See Intel spec. 7.2.3 "TSS Descriptor in 64-bit mode".
     */
    Assert(pDesc->System.u4Type == 11);
    if (   pDesc->System.u16LimitLow != 0x67
        || pDesc->System.u4LimitHigh)
    {
        fRestoreHostFlags |= VMX_RESTORE_HOST_SEL_TR;

        /* If the host has made GDT read-only, we would need to temporarily toggle CR0.WP before writing the GDT. */
        if (g_fHmHostKernelFeatures & SUPKERNELFEATURES_GDT_READ_ONLY)
            fRestoreHostFlags |= VMX_RESTORE_HOST_GDT_READ_ONLY;
        if (g_fHmHostKernelFeatures & SUPKERNELFEATURES_GDT_NEED_WRITABLE)
        {
            /* The GDT is read-only but the writable GDT is available. */
            fRestoreHostFlags |= VMX_RESTORE_HOST_GDT_NEED_WRITABLE;
            pVCpu->hmr0.s.vmx.RestoreHost.HostGdtrRw.cb = pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr.cb;
            int rc = SUPR0GetCurrentGdtRw(&pVCpu->hmr0.s.vmx.RestoreHost.HostGdtrRw.uAddr);
            AssertRCReturn(rc, rc);
        }
    }

    pVCpu->hmr0.s.vmx.fRestoreHostFlags = fRestoreHostFlags;

    /*
     * Do all the VMCS updates in one block to assist nested virtualization.
     */
    int rc;
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_CS_SEL,  pVCpu->hmr0.s.vmx.RestoreHost.uHostSelCS);       AssertRC(rc);
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_SS_SEL,  pVCpu->hmr0.s.vmx.RestoreHost.uHostSelSS);       AssertRC(rc);
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_DS_SEL,  uSelDS);                                         AssertRC(rc);
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_ES_SEL,  uSelES);                                         AssertRC(rc);
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_FS_SEL,  uSelFS);                                         AssertRC(rc);
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_GS_SEL,  uSelGS);                                         AssertRC(rc);
    rc = VMXWriteVmcs16(VMX_VMCS16_HOST_TR_SEL,  pVCpu->hmr0.s.vmx.RestoreHost.uHostSelTR);       AssertRC(rc);
    rc = VMXWriteVmcsNw(VMX_VMCS_HOST_GDTR_BASE, pVCpu->hmr0.s.vmx.RestoreHost.HostGdtr.uAddr);   AssertRC(rc);
    rc = VMXWriteVmcsNw(VMX_VMCS_HOST_IDTR_BASE, pVCpu->hmr0.s.vmx.RestoreHost.HostIdtr.uAddr);   AssertRC(rc);
    rc = VMXWriteVmcsNw(VMX_VMCS_HOST_TR_BASE,   uTRBase);                                        AssertRC(rc);
    rc = VMXWriteVmcsNw(VMX_VMCS_HOST_FS_BASE,   pVCpu->hmr0.s.vmx.RestoreHost.uHostFSBase);      AssertRC(rc);
    rc = VMXWriteVmcsNw(VMX_VMCS_HOST_GS_BASE,   pVCpu->hmr0.s.vmx.RestoreHost.uHostGSBase);      AssertRC(rc);

    return VINF_SUCCESS;
}


/**
 * Exports certain host MSRs in the VM-exit MSR-load area and some in the
 * host-state area of the VMCS.
 *
 * These MSRs will be automatically restored on the host after every successful
 * VM-exit.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
static void hmR0VmxExportHostMsrs(PVMCPUCC pVCpu)
{
    AssertPtr(pVCpu);

    /*
     * Save MSRs that we restore lazily (due to preemption or transition to ring-3)
     * rather than swapping them on every VM-entry.
     */
    hmR0VmxLazySaveHostMsrs(pVCpu);

    /*
     * Host Sysenter MSRs.
     */
    int rc = VMXWriteVmcs32(VMX_VMCS32_HOST_SYSENTER_CS, ASMRdMsr_Low(MSR_IA32_SYSENTER_CS));   AssertRC(rc);
    rc     = VMXWriteVmcsNw(VMX_VMCS_HOST_SYSENTER_ESP,  ASMRdMsr(MSR_IA32_SYSENTER_ESP));      AssertRC(rc);
    rc     = VMXWriteVmcsNw(VMX_VMCS_HOST_SYSENTER_EIP,  ASMRdMsr(MSR_IA32_SYSENTER_EIP));      AssertRC(rc);

    /*
     * Host EFER MSR.
     *
     * If the CPU supports the newer VMCS controls for managing EFER, use it. Otherwise it's
     * done as part of auto-load/store MSR area in the VMCS, see hmR0VmxExportGuestMsrs().
     */
    if (g_fHmVmxSupportsVmcsEfer)
    {
        rc = VMXWriteVmcs64(VMX_VMCS64_HOST_EFER_FULL, g_uHmVmxHostMsrEfer);
        AssertRC(rc);
    }

    /** @todo IA32_PERF_GLOBALCTRL, IA32_PAT also see
     *        vmxHCExportGuestEntryExitCtls(). */
}


/**
 * Figures out if we need to swap the EFER MSR which is particularly expensive.
 *
 * We check all relevant bits. For now, that's everything besides LMA/LME, as
 * these two bits are handled by VM-entry, see vmxHCExportGuestEntryExitCtls().
 *
 * @returns true if we need to load guest EFER, false otherwise.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks Requires EFER, CR4.
 * @remarks No-long-jump zone!!!
 */
/*
  在 VM-entry/exit 时，检查是否需要将 Guest 的 EFER MSR 与 Host 的 EFER MSR 进行交换。
  主要考虑 嵌套虚拟化（Nested VMX）、长模式（Long Mode）和 分页模式（Paging Mode）的影响。
*/
/*
 场景                     触发条件
强制调试                  编译宏 HMVMX_ALWAYS_SWAP_EFER 启用时。
嵌套虚拟化	              L1 Hypervisor 要求加载/保存 EFER（通过 VMX Entry/Exit Controls）。
长模式 SYSCALL 兼容性	  Guest 是 64-bit 且 EFER.SCE 位与 Host 不同。
分页模式 NX 权限	      Guest 使用 PAE/AMD64 分页且 EFER.NXE 位与 Host 不同。
*/
static bool hmR0VmxShouldSwapEferMsr(PCVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient)
{
    //用途：在调试或特殊配置时，强制启用 EFER 交换（如排查兼容性问题）。
#ifdef HMVMX_ALWAYS_SWAP_EFER
    RT_NOREF2(pVCpu, pVmxTransient);
    return true;
#else
    PCCPUMCTX pCtx = &pVCpu->cpum.GstCtx;
    uint64_t const u64HostEfer  = g_uHmVmxHostMsrEfer;
    uint64_t const u64GuestEfer = pCtx->msrEFER;

# ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    /*
     * For nested-guests, we shall honor swapping the EFER MSR when requested by
     * the nested-guest.
     */
    //嵌套虚拟化需要精确控制 EFER 的加载/保存，以维护 L1 和 L2 Hypervisor 的状态隔离。
    if (   pVmxTransient->fIsNestedGuest
        && (   CPUMIsGuestVmxEntryCtlsSet(pCtx, VMX_ENTRY_CTLS_LOAD_EFER_MSR)
            || CPUMIsGuestVmxExitCtlsSet(pCtx, VMX_EXIT_CTLS_SAVE_EFER_MSR)
            || CPUMIsGuestVmxExitCtlsSet(pCtx, VMX_EXIT_CTLS_LOAD_EFER_MSR)))
        return true;// 嵌套 Hypervisor 要求交换 EFER
# else
    RT_NOREF(pVmxTransient);
#endif

    /*
     * For 64-bit guests, if EFER.SCE bit differs, we need to swap the EFER MSR
     * to ensure that the guest's SYSCALL behaviour isn't broken, see @bugref{7386}.
     */
/*
    EFER.SCE 控制 SYSCALL/SYSRET指令的启用（AMD64 特性）。
    若 Guest 处于长模式（64-bit），且 SCE 位与 Host 不一致，必须交换 EFER 以确保 Guest 的 SYSCALL 行为正确。
*/
    //参考 @bugref{7386}（早期 VirtualBox 因未处理 SCE 导致 SYSCALL 失效）
    if (   CPUMIsGuestInLongModeEx(pCtx)
        && (u64GuestEfer & MSR_K6_EFER_SCE) != (u64HostEfer & MSR_K6_EFER_SCE))
        return true;// Guest 和 Host 的 SYSCALL 配置不同，需交换 EFER

    /*
     * If the guest uses PAE and EFER.NXE bit differs, we need to swap the EFER MSR
     * as it affects guest paging. 64-bit paging implies CR4.PAE as well.
     *
     * See Intel spec. 4.5 "IA-32e Paging".
     * See Intel spec. 4.1.1 "Three Paging Modes".
     *
     * Verify that we always intercept CR4.PAE and CR0.PG bits, so we don't need to
     * import CR4 and CR0 from the VMCS here as those bits are always up to date.
     */
    //分页模式（PAE/NX）下的 EFER.NXE 检查
    //PAE 分页：Guest 启用了 CR4.PAE 和 CR0.PG（分页）。
    Assert(vmxHCGetFixedCr4Mask(pVCpu) & X86_CR4_PAE);
    Assert(vmxHCGetFixedCr0Mask(pVCpu) & X86_CR0_PG);
    if (   (pCtx->cr4 & X86_CR4_PAE)
        && (pCtx->cr0 & X86_CR0_PG))
    {
        /*
         * If nested paging is not used, verify that the guest paging mode matches the
         * shadow paging mode which is/will be placed in the VMCS (which is what will
         * actually be used while executing the guest and not the CR4 shadow value).
         */
        AssertMsg(   pVCpu->CTX_SUFF(pVM)->hmr0.s.fNestedPaging
                  || pVCpu->hm.s.enmShadowMode == PGMMODE_PAE
                  || pVCpu->hm.s.enmShadowMode == PGMMODE_PAE_NX
                  || pVCpu->hm.s.enmShadowMode == PGMMODE_AMD64
                  || pVCpu->hm.s.enmShadowMode == PGMMODE_AMD64_NX,
                  ("enmShadowMode=%u\n", pVCpu->hm.s.enmShadowMode));
        if ((u64GuestEfer & MSR_K6_EFER_NXE) != (u64HostEfer & MSR_K6_EFER_NXE))
        {
            /* Verify that the host is NX capable. */
            Assert(g_CpumHostFeatures.s.fNoExecute); // 确保 Host 支持 NX
            return true; // Guest 和 Host 的 NX 配置不同，需交换 EFER
        }
    }

    return false;
#endif
}


/**
 * Exports the guest's RSP into the guest-state area in the VMCS.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
static void hmR0VmxExportGuestRsp(PVMCPUCC pVCpu)
{
    if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_RSP)
    {
        HMVMX_CPUMCTX_ASSERT(pVCpu, CPUMCTX_EXTRN_RSP);

        int rc = VMXWriteVmcsNw(VMX_VMCS_GUEST_RSP, pVCpu->cpum.GstCtx.rsp);
        AssertRC(rc);

        ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_RSP);
        Log4Func(("rsp=%#RX64\n", pVCpu->cpum.GstCtx.rsp));
    }
}


/**
 * Exports the guest hardware-virtualization state.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
static int hmR0VmxExportGuestHwvirtState(PVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient)
{
    if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_HWVIRT)
    {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        /*
         * Check if the VMX feature is exposed to the guest and if the host CPU supports
         * VMCS shadowing.
         */
        if (pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fUseVmcsShadowing)
        {
            /*
             * If the nested hypervisor has loaded a current VMCS and is in VMX root mode,
             * copy the nested hypervisor's current VMCS into the shadow VMCS and enable
             * VMCS shadowing to skip intercepting some or all VMREAD/VMWRITE VM-exits.
             *
             * We check for VMX root mode here in case the guest executes VMXOFF without
             * clearing the current VMCS pointer and our VMXOFF instruction emulation does
             * not clear the current VMCS pointer.
             */
            PVMXVMCSINFO pVmcsInfo = pVmxTransient->pVmcsInfo;
            if (   CPUMIsGuestInVmxRootMode(&pVCpu->cpum.GstCtx)
                && !CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx)
                && CPUMIsGuestVmxCurrentVmcsValid(&pVCpu->cpum.GstCtx))
            {
                /* Paranoia. */
                Assert(!pVmxTransient->fIsNestedGuest);

                /*
                 * For performance reasons, also check if the nested hypervisor's current VMCS
                 * was newly loaded or modified before copying it to the shadow VMCS.
                 */
                if (!pVCpu->hm.s.vmx.fCopiedNstGstToShadowVmcs)
                {
                    int rc = vmxHCCopyNstGstToShadowVmcs(pVCpu, pVmcsInfo);
                    AssertRCReturn(rc, rc);
                    pVCpu->hm.s.vmx.fCopiedNstGstToShadowVmcs = true;
                }
                vmxHCEnableVmcsShadowing(pVCpu, pVmcsInfo);
            }
            else
                vmxHCDisableVmcsShadowing(pVCpu, pVmcsInfo);
        }
#else
        NOREF(pVmxTransient);
#endif
        ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_HWVIRT);
    }
    return VINF_SUCCESS;
}


/**
 * Exports the guest debug registers into the guest-state area in the VMCS.
 * The guest debug bits are partially shared with the host (e.g. DR6, DR0-3).
 *
 * This also sets up whether \#DB and MOV DRx accesses cause VM-exits.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
/*
 *
  在 VM-entry 前（即 Guest 代码执行前），根据 Guest 和 Hypervisor 的调试状态，
    配置 VMCS 中的调试寄存器相关字段（如 DR7、MOV DRx 拦截等）。
  处理 单步调试（TF 标志位）和 嵌套虚拟化（Nested Guest）的特殊情况。
 * */
/*
  在虚拟化环境中，“Hypervisor 调试器激活”是指通过调试工具（如 WinDbg、GDB 等）
    对运行在 Hypervisor 层的代码进行动态跟踪、断点调试或内存分析的过程。这一机制主要用于开发或安全分析场景，允许开发者深入监控 Hypervisor 和虚拟机的底层行为‌35。以下是其核心要点：
 调试目标
  Hypervisor 层：直接调试虚拟化监控程序（如 VirtualBox 的 VMM、Windows Hyper-V 的 Hypervisor），
    分析其对 Guest OS 的管理逻辑（如 VM-exit 处理、EPT 配置等）
  Guest 内核：通过 Hypervisor 调试器间接监控 Guest OS 的内核行为（如内存访问、特权指令执行）
*/
static int hmR0VmxExportSharedDebugState(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    /** @todo NSTVMX: Figure out what we want to do with nested-guest instruction
     *        stepping. */
    PVMXVMCSINFO pVmcsInfo = pVmxTransient->pVmcsInfo;
    if (pVmxTransient->fIsNestedGuest)
    {
        int rc = VMXWriteVmcsNw(VMX_VMCS_GUEST_DR7, CPUMGetGuestDR7(pVCpu));
        AssertRC(rc);

        /*
         * We don't want to always intercept MOV DRx for nested-guests as it causes
         * problems when the nested hypervisor isn't intercepting them, see @bugref{10080}.
         * Instead, they are strictly only requested when the nested hypervisor intercepts
         * them -- handled while merging VMCS controls.
         *
         * If neither the outer nor the nested-hypervisor is intercepting MOV DRx,
         * then the nested-guest debug state should be actively loaded on the host so that
         * nested-guest reads its own debug registers without causing VM-exits.
         */
        // 如果外层和嵌套 Hypervisor 均不拦截 MOV DRx，则主动加载 Guest 的调试状态
        /*
         * 背景：在嵌套虚拟化中，Guest 的 MOV DRx 指令可能被外层或内层 Hypervisor 拦截。
           逻辑：
             若 两者均不拦截，则直接加载 Guest 的调试寄存器到 Host CPU，避免不必要的 VM-exit。
             否则依赖 VM-exit 由 VMM 模拟（参考 @bugref{10080} 的修复）。
         * */
        if (   !(pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_MOV_DR_EXIT)
            && !CPUMIsGuestDebugStateActive(pVCpu))
            CPUMR0LoadGuestDebugState(pVCpu, true /* include DR6 */);
        return VINF_SUCCESS;
    }

#ifdef VBOX_STRICT
    /* Validate. Intel spec. 26.3.1.1 "Checks on Guest Controls Registers, Debug Registers, MSRs" */
    if (pVmcsInfo->u32EntryCtls & VMX_ENTRY_CTLS_LOAD_DEBUG)
    {
        /* Validate. Intel spec. 17.2 "Debug Registers", recompiler paranoia checks. */
        Assert((pVCpu->cpum.GstCtx.dr[7] & (X86_DR7_MBZ_MASK | X86_DR7_RAZ_MASK)) == 0);
        Assert((pVCpu->cpum.GstCtx.dr[7] & X86_DR7_RA1_MASK) == X86_DR7_RA1_MASK);
    }
#endif

    bool     fSteppingDB      = false;
    uint32_t uProcCtls        = pVmcsInfo->u32ProcCtls;
    // 单步调试（Single-Step）处理
    if (pVCpu->hm.s.fSingleInstruction)
    {
        /* If the CPU supports the monitor trap flag, use it for single stepping in DBGF and avoid intercepting #DB. */
        // 优先使用 CPU 的 Monitor Trap Flag（MTF）硬件特性
        // MTF 优先：若 CPU 支持 Monitor Trap Flag，则通过 VMCS 启用，在 Guest 执行一条指令后自动触发 VM-exit。
        if (g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_MONITOR_TRAP_FLAG)
        {
            uProcCtls |= VMX_PROC_CTLS_MONITOR_TRAP_FLAG;
            Assert(fSteppingDB == false);
        }
        else
        {
            // 若无 MTF，则设置 Guest 的 EFLAGS.TF，通过 #DB 异常实现单步调试（需后续清除 TF 防止递归异常）。
            pVCpu->cpum.GstCtx.eflags.u |= X86_EFL_TF;
            pVCpu->hm.s.fCtxChanged |= HM_CHANGED_GUEST_RFLAGS;
            pVCpu->hmr0.s.fClearTrapFlag = true;
            fSteppingDB = true;
        }
    }

#ifdef VMX_WITH_MAYBE_ALWAYS_INTERCEPT_MOV_DRX
    bool     fInterceptMovDRx = pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fAlwaysInterceptMovDRx;
#else
    bool     fInterceptMovDRx = false;
#endif
    uint64_t u64GuestDr7;
    // Hypervisor 调试器激活时，加载 Hypervisor 的 DRx 状态
    if (   fSteppingDB
        || (CPUMGetHyperDR7(pVCpu) & X86_DR7_ENABLED_MASK))
    {
        /*
         * Use the combined guest and host DRx values found in the hypervisor register set
         * because the hypervisor debugger has breakpoints active or someone is single stepping
         * on the host side without a monitor trap flag.
         *
         * Note! DBGF expects a clean DR6 state before executing guest code.
         */
        if (!CPUMIsHyperDebugStateActive(pVCpu))
        {
            CPUMR0LoadHyperDebugState(pVCpu, true /* include DR6 */);
            Assert(CPUMIsHyperDebugStateActive(pVCpu));
            Assert(!CPUMIsGuestDebugStateActive(pVCpu));
        }

        /* Update DR7 with the hypervisor value (other DRx registers are handled by CPUM one way or another). */
        u64GuestDr7 = CPUMGetHyperDR7(pVCpu);
        pVCpu->hmr0.s.fUsingHyperDR7 = true;
        fInterceptMovDRx = true;
    }
    else
    {
        /*
         * If the guest has enabled debug registers, we need to load them prior to
         * executing guest code so they'll trigger at the right time.
         */
        HMVMX_CPUMCTX_ASSERT(pVCpu, CPUMCTX_EXTRN_DR7);
        if (pVCpu->cpum.GstCtx.dr[7] & (X86_DR7_ENABLED_MASK | X86_DR7_GD))
        {
            // Guest 启用调试寄存器时，加载 Guest 的 DRx 状态
            if (!CPUMIsGuestDebugStateActive(pVCpu))
            {
                CPUMR0LoadGuestDebugState(pVCpu, true /* include DR6 */);
                Assert(CPUMIsGuestDebugStateActive(pVCpu));
                Assert(!CPUMIsHyperDebugStateActive(pVCpu));
                STAM_COUNTER_INC(&pVCpu->hm.s.StatDRxArmed);
            }
#ifndef VMX_WITH_MAYBE_ALWAYS_INTERCEPT_MOV_DRX
            Assert(!fInterceptMovDRx);
#endif
        }
        // 无调试时，延迟加载 DR0-DR3，但必须拦截 MOV DRx 和 #DB
        else if (!CPUMIsGuestDebugStateActive(pVCpu))
        {
            /*
             * If no debugging enabled, we'll lazy load DR0-3.  Unlike on AMD-V, we
             * must intercept #DB in order to maintain a correct DR6 guest value, and
             * because we need to intercept it to prevent nested #DBs from hanging the
             * CPU, we end up always having to intercept it. See hmR0VmxSetupVmcsXcptBitmap().
             */
            fInterceptMovDRx = true;
        }

        /* Update DR7 with the actual guest value. */
        u64GuestDr7 = pVCpu->cpum.GstCtx.dr[7];
        pVCpu->hmr0.s.fUsingHyperDR7 = false;
    }

    //根据 fInterceptMovDRx 决定是否在 VMCS 中启用 MOV DRx 拦截。
    if (fInterceptMovDRx)
        uProcCtls |= VMX_PROC_CTLS_MOV_DR_EXIT;
    else
        uProcCtls &= ~VMX_PROC_CTLS_MOV_DR_EXIT;

    /*
     * Update the processor-based VM-execution controls with the MOV-DRx intercepts and the
     * monitor-trap flag and update our cache.
     */
    if (uProcCtls != pVmcsInfo->u32ProcCtls)
    {
        int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC, uProcCtls);
        AssertRC(rc);
        pVmcsInfo->u32ProcCtls = uProcCtls;
    }

    /*
     * Update guest DR7.
     */
    int rc = VMXWriteVmcsNw(VMX_VMCS_GUEST_DR7, u64GuestDr7);
    AssertRC(rc);

    /*
     * If we have forced EFLAGS.TF to be set because we're single-stepping in the hypervisor debugger,
     * we need to clear interrupt inhibition if any as otherwise it causes a VM-entry failure.
     *
     * See Intel spec. 26.3.1.5 "Checks on Guest Non-Register State".
     */
    if (fSteppingDB)
    {
        Assert(pVCpu->hm.s.fSingleInstruction);
        Assert(pVCpu->cpum.GstCtx.eflags.Bits.u1TF);

        uint32_t fIntrState = 0;
        rc = VMXReadVmcs32(VMX_VMCS32_GUEST_INT_STATE, &fIntrState);
        AssertRC(rc);

        if (fIntrState & (VMX_VMCS_GUEST_INT_STATE_BLOCK_STI | VMX_VMCS_GUEST_INT_STATE_BLOCK_MOVSS))
        {
            fIntrState &= ~(VMX_VMCS_GUEST_INT_STATE_BLOCK_STI | VMX_VMCS_GUEST_INT_STATE_BLOCK_MOVSS);
            rc = VMXWriteVmcs32(VMX_VMCS32_GUEST_INT_STATE, fIntrState);
            AssertRC(rc);
        }
    }

    return VINF_SUCCESS;
}


/**
 * Exports certain guest MSRs into the VM-entry MSR-load and VM-exit MSR-store
 * areas.
 *
 * These MSRs will automatically be loaded to the host CPU on every successful
 * VM-entry and stored from the host CPU on every successful VM-exit.
 *
 * We creates/updates MSR slots for the host MSRs in the VM-exit MSR-load area. The
 * actual host MSR values are not- updated here for performance reasons. See
 * hmR0VmxExportHostMsrs().
 *
 * We also exports the guest sysenter MSRs into the guest-state area in the VMCS.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
//在 VM-entry 前 将客户机的 MSR（Model-Specific Register）状态导出到 VMCS 中，
//确保客户机执行时能正确访问这些寄存器
/*
  主要处理三类 MSR：
    自动加载/存储的 MSR（当前未实现，保留扩展点）。
    Sysenter 相关 MSR（SYSENTER_CS/EIP/ESP）。
    EFER MSR（控制长模式等关键功能）。
    其他 MSR（如分支记录 LBR、推测控制 SPEC_CTRL）。
*/
/*
自动加载区域（Auto-Load/Store MSR Area）
工作原理
  VMCS 中有一个 MSR Load/Store List，可配置一组 MSR 及其值。
硬件行为：
  在 VM-entry 时，CPU 自动从该区域加载 MSR 值到物理寄存器。
  在 VM-exit 时，CPU 自动将物理寄存器的值保存回该区域。
*/
static int hmR0VmxExportGuestMsrs(PVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient)
{
    AssertPtr(pVCpu);
    AssertPtr(pVmxTransient);

    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    PCCPUMCTX pCtx = &pVCpu->cpum.GstCtx;

    /*
     * MSRs that we use the auto-load/store MSR area in the VMCS.
     * For 64-bit hosts, we load/restore them lazily, see hmR0VmxLazyLoadGuestMsrs(),
     * nothing to do here. The host MSR values are updated when it's safe in
     * hmR0VmxLazySaveHostMsrs().
     *
     * For nested-guests, the guests MSRs from the VM-entry MSR-load area are already
     * loaded (into the guest-CPU context) by the VMLAUNCH/VMRESUME instruction
     * emulation. The merged MSR permission bitmap will ensure that we get VM-exits
     * for any MSR that are not part of the lazy MSRs so we do not need to place
     * those MSRs into the auto-load/store MSR area. Nothing to do here.
     */
    if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_VMX_GUEST_AUTO_MSRS)
    {
        /* No auto-load/store MSRs currently. */
        //当前无实际逻辑，仅为未来扩展预留接口。若需支持动态 MSR 加载（如性能监控 MSR），可在此扩展。
        ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_VMX_GUEST_AUTO_MSRS);
    }

    /*
     * Guest Sysenter MSRs.
     */
    /*
       通过 VMXWriteVmcsXX 直接写入 VMCS 的 guest 区域。
       使用 CPUMCTX_EXTRN_SYSENTER_MSRS 确保上下文已加载。
       清除对应的 fCtxChanged 标志位，避免重复操作。
    */
    if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_SYSENTER_MSR_MASK)
    {
        HMVMX_CPUMCTX_ASSERT(pVCpu, CPUMCTX_EXTRN_SYSENTER_MSRS);

        /* 分别处理 SYSENTER_CS/EIP/ESP */
        if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_SYSENTER_CS_MSR)
        {
            int rc = VMXWriteVmcs32(VMX_VMCS32_GUEST_SYSENTER_CS, pCtx->SysEnter.cs);
            AssertRC(rc);
            ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_SYSENTER_CS_MSR);
        }

        if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_SYSENTER_EIP_MSR)
        {
            int rc = VMXWriteVmcsNw(VMX_VMCS_GUEST_SYSENTER_EIP, pCtx->SysEnter.eip);
            AssertRC(rc);
            ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_SYSENTER_EIP_MSR);
        }

        if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_SYSENTER_ESP_MSR)
        {
            int rc = VMXWriteVmcsNw(VMX_VMCS_GUEST_SYSENTER_ESP, pCtx->SysEnter.esp);
            AssertRC(rc);
            ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_SYSENTER_ESP_MSR);
        }
    }

    /*
     * Guest/host EFER MSR.
     */
    if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_EFER_MSR)
    {
        /* Whether we are using the VMCS to swap the EFER MSR must have been
           determined earlier while exporting VM-entry/VM-exit controls. */
        Assert(!(ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_VMX_ENTRY_EXIT_CTLS));
        HMVMX_CPUMCTX_ASSERT(pVCpu, CPUMCTX_EXTRN_EFER);

        if (hmR0VmxShouldSwapEferMsr(pVCpu, pVmxTransient))
        {
            /*
             * EFER.LME is written by software, while EFER.LMA is set by the CPU to (CR0.PG & EFER.LME).
             * This means a guest can set EFER.LME=1 while CR0.PG=0 and EFER.LMA can remain 0.
             * VT-x requires that "IA-32e mode guest" VM-entry control must be identical to EFER.LMA
             * and to CR0.PG. Without unrestricted execution, CR0.PG (used for VT-x, not the shadow)
             * must always be 1. This forces us to effectively clear both EFER.LMA and EFER.LME until
             * the guest has also set CR0.PG=1. Otherwise, we would run into an invalid-guest state
             * during VM-entry.
             */
            uint64_t uGuestEferMsr = pCtx->msrEFER;
            //在非 Unrestricted Guest 模式下，强制 EFER.LME=LMA，避免 VM-entry 因无效状态失败。
            if (!pVM->hmr0.s.vmx.fUnrestrictedGuest)
            {
                if (!(pCtx->msrEFER & MSR_K6_EFER_LMA))
                    uGuestEferMsr &= ~MSR_K6_EFER_LME;
                else
                    Assert((pCtx->msrEFER & (MSR_K6_EFER_LMA | MSR_K6_EFER_LME)) == (MSR_K6_EFER_LMA | MSR_K6_EFER_LME));
            }

            /*
             * If the CPU supports VMCS controls for swapping EFER, use it. Otherwise, we have no option
             * but to use the auto-load store MSR area in the VMCS for swapping EFER. See @bugref{7368}.
             */
            //若 CPU 支持 VMCS 直接管理 EFER（g_fHmVmxSupportsVmcsEfer），优先使用；否则通过自动加载区域实现。
            if (g_fHmVmxSupportsVmcsEfer)
            {
                int rc = VMXWriteVmcs64(VMX_VMCS64_GUEST_EFER_FULL, uGuestEferMsr);
                AssertRC(rc);
            }
            else
            {
                /*
                 * We shall use the auto-load/store MSR area only for loading the EFER MSR but we must
                 * continue to intercept guest read and write accesses to it, see @bugref{7386#c16}.
                 */
                int rc = hmR0VmxAddAutoLoadStoreMsr(pVCpu, pVmxTransient, MSR_K6_EFER, uGuestEferMsr,
                                                    false /* fSetReadWrite */, false /* fUpdateHostMsr */);
                AssertRCReturn(rc, rc);
            }

            Log4Func(("efer=%#RX64 shadow=%#RX64\n", uGuestEferMsr, pCtx->msrEFER));
        }
        else if (!g_fHmVmxSupportsVmcsEfer)
            hmR0VmxRemoveAutoLoadStoreMsr(pVCpu, pVmxTransient, MSR_K6_EFER);

        ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_EFER_MSR);
    }

    /*
     * Other MSRs.
     */
    /*
      安全特性：SPEC_CTRL 用于缓解 Spectre 漏洞，需确保客户机值正确加载。
       调试支持：LBR（Last Branch Record）记录分支历史，用于性能分析或故障诊断。
       动态管理：通过 hmR0VmxAddAutoLoadStoreMsr 将 MSR 添加到 VMCS 的自动加载区域。
    */
    if (ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged) & HM_CHANGED_GUEST_OTHER_MSRS)
    {
        /* Speculation Control (R/W). */
        HMVMX_CPUMCTX_ASSERT(pVCpu, HM_CHANGED_GUEST_OTHER_MSRS);
        /* 推测控制 MSR（如 SPEC_CTRL）*/
        if (pVM->cpum.ro.GuestFeatures.fIbrs)
        {
            int rc = hmR0VmxAddAutoLoadStoreMsr(pVCpu, pVmxTransient, MSR_IA32_SPEC_CTRL, CPUMGetGuestSpecCtrl(pVCpu),
                                                false /* fSetReadWrite */, false /* fUpdateHostMsr */);
            AssertRCReturn(rc, rc);
        }

        /* 分支记录 MSR（LBR）*/
        /* Last Branch Record. */
        if (pVM->hmr0.s.vmx.fLbr)
        {
            PVMXVMCSINFOSHARED const pVmcsInfoShared = pVmxTransient->pVmcsInfo->pShared;
            uint32_t const idFromIpMsrStart = pVM->hmr0.s.vmx.idLbrFromIpMsrFirst;
            uint32_t const idToIpMsrStart   = pVM->hmr0.s.vmx.idLbrToIpMsrFirst;
            uint32_t const cLbrStack        = pVM->hmr0.s.vmx.idLbrFromIpMsrLast - pVM->hmr0.s.vmx.idLbrFromIpMsrFirst + 1;
            Assert(cLbrStack <= 32);
            for (uint32_t i = 0; i < cLbrStack; i++)
            {
                int rc = hmR0VmxAddAutoLoadStoreMsr(pVCpu, pVmxTransient, idFromIpMsrStart + i,
                                                    pVmcsInfoShared->au64LbrFromIpMsr[i],
                                                    false /* fSetReadWrite */, false /* fUpdateHostMsr */);
                AssertRCReturn(rc, rc);

                /* Some CPUs don't have a Branch-To-IP MSR (P4 and related Xeons). */
                if (idToIpMsrStart != 0)
                {
                    rc = hmR0VmxAddAutoLoadStoreMsr(pVCpu, pVmxTransient, idToIpMsrStart + i,
                                                    pVmcsInfoShared->au64LbrToIpMsr[i],
                                                    false /* fSetReadWrite */, false /* fUpdateHostMsr */);
                    AssertRCReturn(rc, rc);
                }
            }

            /* Add LBR top-of-stack MSR (which contains the index to the most recent record). */
            int rc = hmR0VmxAddAutoLoadStoreMsr(pVCpu, pVmxTransient, pVM->hmr0.s.vmx.idLbrTosMsr,
                                                pVmcsInfoShared->u64LbrTosMsr, false /* fSetReadWrite */,
                                                false /* fUpdateHostMsr */);
            AssertRCReturn(rc, rc);
        }

        ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~HM_CHANGED_GUEST_OTHER_MSRS);
    }

    return VINF_SUCCESS;
}


/**
 * Wrapper for running the guest code in VT-x.
 *
 * @returns VBox status code, no informational status codes.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
// 执行 VMX 非根模式（Guest 模式） 的核心接口，
// 负责通过 VMLAUNCH 或 VMRESUME 指令启动/恢复客户机 CPU 执行。
/*
  标记寄存器所有权：声明 Hypervisor 对客户机寄存器状态的管理权。
  选择启动方式：根据 VMCS 状态决定使用 VMLAUNCH（首次启动）还是 VMRESUME（恢复运行）。
  调用底层 VMX 指令：通过函数指针 pfnStartVm 触发硬件虚拟化操作。
*/
DECLINLINE(int) hmR0VmxRunGuest(PVMCPUCC pVCpu, PCVMXTRANSIENT pVmxTransient)
{
    /* Mark that HM is the keeper of all guest-CPU registers now that we're going to execute guest code. */
    //标记所有客户机寄存器由 Hypervisor 管理（避免冗余加载）。
    //明确 Hypervisor 是寄存器状态的“维护者”。
    //fExtrn 标志位：
      //VirtualBox 通过此标志避免不必要的 VMCS 字段写入。例如，若某寄存器未被修改（!CPUMCTX_EXTRN_xxx），则跳过对应的 VMWRITE 操作。
    //确保在进入客户机前，所有寄存器状态已从内存加载到 VMCS 中，且后续由 Hypervisor 负责同步。
    pVCpu->cpum.GstCtx.fExtrn |= HMVMX_CPUMCTX_EXTRN_ALL | CPUMCTX_EXTRN_KEEPER_HM;

    PVMXVMCSINFO pVmcsInfo = pVmxTransient->pVmcsInfo;
    //VMRESUME 或 VMLAUNCH
    bool const   fResumeVM = RT_BOOL(pVmcsInfo->fVmcsState == VMX_V_VMCS_LAUNCH_STATE_LAUNCHED);
#ifdef VBOX_WITH_STATISTICS
    if (fResumeVM)
        STAM_COUNTER_INC(&pVCpu->hm.s.StatVmxVmResume);
    else
        STAM_COUNTER_INC(&pVCpu->hm.s.StatVmxVmLaunch);
#endif
    //hmR0VmxStartVm64
    int rc = pVCpu->hmr0.s.vmx.pfnStartVm(pVmcsInfo, pVCpu, fResumeVM);
    AssertMsg(rc <= VINF_SUCCESS, ("%Rrc\n", rc));
    return rc;
}


/**
 * Reports world-switch error and dumps some useful debug info.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   rcVMRun         The return code from VMLAUNCH/VMRESUME.
 * @param   pVmxTransient   The VMX-transient structure (only
 *                          exitReason updated).
 */
//在虚拟机切换（World Switch）失败时，记录详细的错误信息，
//并输出与 VMX 相关的硬件状态，用于调试和分析。
/*
典型应用场景
  VM-entry 失败调试
    当客户机启动失败时，通过日志分析 u32InstrError 和 u32ExitReason 定位问题根源（如客户机状态无效、MSR 加载失败）。
  硬件兼容性验证
    检查 VMCS 字段是否被正确支持（如缺少 VPID 支持时跳过相关字段）。
  Hypervisor 开发
    开发者可通过该函数快速验证 VMX 配置是否符合 Intel 手册规范。
*/
static void hmR0VmxReportWorldSwitchError(PVMCPUCC pVCpu, int rcVMRun, PVMXTRANSIENT pVmxTransient)
{
    Assert(pVCpu);
    Assert(pVmxTransient);
    HMVMX_ASSERT_PREEMPT_SAFE(pVCpu);

    Log4Func(("VM-entry failure: %Rrc\n", rcVMRun));
    switch (rcVMRun)
    {
        case VERR_VMX_INVALID_VMXON_PTR: //表示 VMXON 指针无效（硬件初始化问题）。
            AssertFailed();
            break;
        case VINF_SUCCESS:                  /* VMLAUNCH/VMRESUME succeeded but VM-entry failed... yeah, true story. */
        case VERR_VMX_UNABLE_TO_START_VM:   /* VMLAUNCH/VMRESUME itself failed. */
        {
            //读取 VMCS 退出原因 
            //u32ExitReason 的高 16 位标识具体事件类型（如外部中断、EPT 违规等）。
            int rc = VMXReadVmcs32(VMX_VMCS32_RO_EXIT_REASON, &pVCpu->hm.s.vmx.LastError.u32ExitReason);
            rc    |= VMXReadVmcs32(VMX_VMCS32_RO_VM_INSTR_ERROR, &pVCpu->hm.s.vmx.LastError.u32InstrError);
            AssertRC(rc);
            //获取退出时的附加信息（如退出资格 Exit Qualification）。
            vmxHCReadToTransientSlow<HMVMX_READ_EXIT_QUALIFICATION>(pVCpu, pVmxTransient);

            //记录发生错误的 CPU ID
            pVCpu->hm.s.vmx.LastError.idEnteredCpu = pVCpu->hmr0.s.idEnteredCpu;
            /* LastError.idCurrentCpu was already updated in hmR0VmxPreRunGuestCommitted().
               Cannot do it here as we may have been long preempted. */

#ifdef VBOX_STRICT
                PVMXVMCSINFO pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
                Log4(("uExitReason        %#RX32 (VmxTransient %#RX16)\n", pVCpu->hm.s.vmx.LastError.u32ExitReason,
                     pVmxTransient->uExitReason));
                Log4(("Exit Qualification %#RX64\n", pVmxTransient->uExitQual));
                Log4(("InstrError         %#RX32\n", pVCpu->hm.s.vmx.LastError.u32InstrError));
                //通过 g_apszVmxInstrErrors 数组将 u32InstrError 转换为可读字符串
                //（如错误码 7 对应 "Invalid guest state"）。
                if (pVCpu->hm.s.vmx.LastError.u32InstrError <= HMVMX_INSTR_ERROR_MAX)
                    Log4(("InstrError Desc.  \"%s\"\n", g_apszVmxInstrErrors[pVCpu->hm.s.vmx.LastError.u32InstrError]));
                else
                    Log4(("InstrError Desc.    Range exceeded %u\n", HMVMX_INSTR_ERROR_MAX));
                Log4(("Entered host CPU   %u\n", pVCpu->hm.s.vmx.LastError.idEnteredCpu));
                Log4(("Current host CPU   %u\n", pVCpu->hm.s.vmx.LastError.idCurrentCpu));

                /*
                 VMCS 控制字段
                   检查所有关键 VMCS 字段（如 CTRL_PIN_EXEC、CTRL_PROC_EXEC），确保其值符合 Intel VT-x 规范。
                 宿主/客户机状态
                   宿主状态：CR0/CR3/CR4、段寄存器（CS/DS/ES等）、GDTR/IDTR、RIP/RSP。
                   客户机状态：RIP、RSP、RFLAGS。
                 段描述符验证
                   通过 ASMGetGDTR 读取 GDT 表，验证宿主段选择器（如 VMX_VMCS16_HOST_CS_SEL）是否指向合法的描述符。
                 MSR 寄存器
                   输出 MSR_K6_EFER、MSR_K8_CSTAR 等寄存器值，分析 CPU 扩展功能状态。
                */ 
                static struct
                {
                    /** Name of the field to log. */
                    const char     *pszName;
                    /** The VMCS field. */
                    uint32_t        uVmcsField;
                    /** Whether host support of this field needs to be checked. */
                    bool            fCheckSupport;
                } const s_aVmcsFields[] =
                {
                    { "VMX_VMCS32_CTRL_PIN_EXEC",                 VMX_VMCS32_CTRL_PIN_EXEC,                   false  },
                    { "VMX_VMCS32_CTRL_PROC_EXEC",                VMX_VMCS32_CTRL_PROC_EXEC,                  false  },
                    { "VMX_VMCS32_CTRL_PROC_EXEC2",               VMX_VMCS32_CTRL_PROC_EXEC2,                 true   },
                    { "VMX_VMCS32_CTRL_ENTRY",                    VMX_VMCS32_CTRL_ENTRY,                      false  },
                    { "VMX_VMCS32_CTRL_EXIT",                     VMX_VMCS32_CTRL_EXIT,                       false  },
                    { "VMX_VMCS32_CTRL_CR3_TARGET_COUNT",         VMX_VMCS32_CTRL_CR3_TARGET_COUNT,           false  },
                    { "VMX_VMCS32_CTRL_ENTRY_INTERRUPTION_INFO",  VMX_VMCS32_CTRL_ENTRY_INTERRUPTION_INFO,    false  },
                    { "VMX_VMCS32_CTRL_ENTRY_EXCEPTION_ERRCODE",  VMX_VMCS32_CTRL_ENTRY_EXCEPTION_ERRCODE,    false  },
                    { "VMX_VMCS32_CTRL_ENTRY_INSTR_LENGTH",       VMX_VMCS32_CTRL_ENTRY_INSTR_LENGTH,         false  },
                    { "VMX_VMCS32_CTRL_TPR_THRESHOLD",            VMX_VMCS32_CTRL_TPR_THRESHOLD,              false  },
                    { "VMX_VMCS32_CTRL_EXIT_MSR_STORE_COUNT",     VMX_VMCS32_CTRL_EXIT_MSR_STORE_COUNT,       false  },
                    { "VMX_VMCS32_CTRL_EXIT_MSR_LOAD_COUNT",      VMX_VMCS32_CTRL_EXIT_MSR_LOAD_COUNT,        false  },
                    { "VMX_VMCS32_CTRL_ENTRY_MSR_LOAD_COUNT",     VMX_VMCS32_CTRL_ENTRY_MSR_LOAD_COUNT,       false  },
                    { "VMX_VMCS32_CTRL_EXCEPTION_BITMAP",         VMX_VMCS32_CTRL_EXCEPTION_BITMAP,           false  },
                    { "VMX_VMCS32_CTRL_PAGEFAULT_ERROR_MASK",     VMX_VMCS32_CTRL_PAGEFAULT_ERROR_MASK,       false  },
                    { "VMX_VMCS32_CTRL_PAGEFAULT_ERROR_MATCH",    VMX_VMCS32_CTRL_PAGEFAULT_ERROR_MATCH,      false  },
                    { "VMX_VMCS_CTRL_CR0_MASK",                   VMX_VMCS_CTRL_CR0_MASK,                     false  },
                    { "VMX_VMCS_CTRL_CR0_READ_SHADOW",            VMX_VMCS_CTRL_CR0_READ_SHADOW,              false  },
                    { "VMX_VMCS_CTRL_CR4_MASK",                   VMX_VMCS_CTRL_CR4_MASK,                     false  },
                    { "VMX_VMCS_CTRL_CR4_READ_SHADOW",            VMX_VMCS_CTRL_CR4_READ_SHADOW,              false  },
                    { "VMX_VMCS64_CTRL_EPTP_FULL",                VMX_VMCS64_CTRL_EPTP_FULL,                  true   },
                    { "VMX_VMCS_GUEST_RIP",                       VMX_VMCS_GUEST_RIP,                         false  },
                    { "VMX_VMCS_GUEST_RSP",                       VMX_VMCS_GUEST_RSP,                         false  },
                    { "VMX_VMCS_GUEST_RFLAGS",                    VMX_VMCS_GUEST_RFLAGS,                      false  },
                    { "VMX_VMCS16_VPID",                          VMX_VMCS16_VPID,                            true,  },
                    { "VMX_VMCS_HOST_CR0",                        VMX_VMCS_HOST_CR0,                          false  },
                    { "VMX_VMCS_HOST_CR3",                        VMX_VMCS_HOST_CR3,                          false  },
                    { "VMX_VMCS_HOST_CR4",                        VMX_VMCS_HOST_CR4,                          false  },
                    /* The order of selector fields below are fixed! */
                    { "VMX_VMCS16_HOST_ES_SEL",                   VMX_VMCS16_HOST_ES_SEL,                     false  },
                    { "VMX_VMCS16_HOST_CS_SEL",                   VMX_VMCS16_HOST_CS_SEL,                     false  },
                    { "VMX_VMCS16_HOST_SS_SEL",                   VMX_VMCS16_HOST_SS_SEL,                     false  },
                    { "VMX_VMCS16_HOST_DS_SEL",                   VMX_VMCS16_HOST_DS_SEL,                     false  },
                    { "VMX_VMCS16_HOST_FS_SEL",                   VMX_VMCS16_HOST_FS_SEL,                     false  },
                    { "VMX_VMCS16_HOST_GS_SEL",                   VMX_VMCS16_HOST_GS_SEL,                     false  },
                    { "VMX_VMCS16_HOST_TR_SEL",                   VMX_VMCS16_HOST_TR_SEL,                     false  },
                    /* End of ordered selector fields. */
                    { "VMX_VMCS_HOST_TR_BASE",                    VMX_VMCS_HOST_TR_BASE,                      false  },
                    { "VMX_VMCS_HOST_GDTR_BASE",                  VMX_VMCS_HOST_GDTR_BASE,                    false  },
                    { "VMX_VMCS_HOST_IDTR_BASE",                  VMX_VMCS_HOST_IDTR_BASE,                    false  },
                    { "VMX_VMCS32_HOST_SYSENTER_CS",              VMX_VMCS32_HOST_SYSENTER_CS,                false  },
                    { "VMX_VMCS_HOST_SYSENTER_EIP",               VMX_VMCS_HOST_SYSENTER_EIP,                 false  },
                    { "VMX_VMCS_HOST_SYSENTER_ESP",               VMX_VMCS_HOST_SYSENTER_ESP,                 false  },
                    { "VMX_VMCS_HOST_RSP",                        VMX_VMCS_HOST_RSP,                          false  },
                    { "VMX_VMCS_HOST_RIP",                        VMX_VMCS_HOST_RIP,                          false  }
                };

                RTGDTR      HostGdtr;
                ASMGetGDTR(&HostGdtr);

                uint32_t const cVmcsFields = RT_ELEMENTS(s_aVmcsFields);
                for (uint32_t i = 0; i < cVmcsFields; i++)
                {
                    uint32_t const uVmcsField = s_aVmcsFields[i].uVmcsField;

                    bool fSupported;
                    if (!s_aVmcsFields[i].fCheckSupport)
                        fSupported = true;
                    else
                    {
                        PVMCC pVM = pVCpu->CTX_SUFF(pVM);
                        switch (uVmcsField)
                        {
                            //某些字段（如 VMX_VMCS64_CTRL_EPTP_FULL）需要硬件支持（如嵌套分页），
                            //通过 pVM->hmr0.s.fNestedPaging 判断是否生效。
                            case VMX_VMCS64_CTRL_EPTP_FULL:  fSupported = pVM->hmr0.s.fNestedPaging;    break;
                            case VMX_VMCS16_VPID:            fSupported = pVM->hmr0.s.vmx.fVpid;          break;
                            case VMX_VMCS32_CTRL_PROC_EXEC2:
                                fSupported = RT_BOOL(pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_SECONDARY_CTLS);
                                break;
                            default:
                                AssertMsgFailedReturnVoid(("Failed to provide VMCS field support for %#RX32\n", uVmcsField));
                        }
                    }

                    if (fSupported)
                    {
                        uint8_t const uWidth = RT_BF_GET(uVmcsField, VMX_BF_VMCSFIELD_WIDTH);
                        //根据 VMCS 字段的宽度（16/32/64位）调用对应的 VMXReadVmcsXX 函数读取。
                        switch (uWidth)
                        {
                            case VMX_VMCSFIELD_WIDTH_16BIT:
                            {
                                uint16_t u16Val;
                                rc = VMXReadVmcs16(uVmcsField, &u16Val);
                                AssertRC(rc);
                                Log4(("%-40s = %#RX16\n", s_aVmcsFields[i].pszName, u16Val));

                                if (   uVmcsField >= VMX_VMCS16_HOST_ES_SEL
                                    && uVmcsField <= VMX_VMCS16_HOST_TR_SEL)
                                {
                                    if (u16Val < HostGdtr.cbGdt)
                                    {
                                        /* Order of selectors in s_apszSel is fixed and matches the order in s_aVmcsFields. */
                                        static const char * const s_apszSel[] = { "Host ES", "Host CS", "Host SS", "Host DS",
                                                                                  "Host FS", "Host GS", "Host TR" };
                                        uint8_t const idxSel = RT_BF_GET(uVmcsField, VMX_BF_VMCSFIELD_INDEX);
                                        Assert(idxSel < RT_ELEMENTS(s_apszSel));
                                        PCX86DESCHC pDesc = (PCX86DESCHC)(HostGdtr.pGdt + (u16Val & X86_SEL_MASK));
                                        hmR0DumpDescriptor(pDesc, u16Val, s_apszSel[idxSel]);
                                    }
                                    else
                                        Log4(("  Selector value exceeds GDT limit!\n"));
                                }
                                break;
                            }

                            case VMX_VMCSFIELD_WIDTH_32BIT:
                            {
                                uint32_t u32Val;
                                rc = VMXReadVmcs32(uVmcsField, &u32Val);
                                AssertRC(rc);
                                Log4(("%-40s = %#RX32\n", s_aVmcsFields[i].pszName, u32Val));
                                break;
                            }

                            case VMX_VMCSFIELD_WIDTH_64BIT:
                            case VMX_VMCSFIELD_WIDTH_NATURAL:
                            {
                                uint64_t u64Val;
                                rc = VMXReadVmcs64(uVmcsField, &u64Val);
                                AssertRC(rc);
                                Log4(("%-40s = %#RX64\n", s_aVmcsFields[i].pszName, u64Val));
                                break;
                            }
                        }
                    }
                }

                Log4(("MSR_K6_EFER            = %#RX64\n", ASMRdMsr(MSR_K6_EFER)));
                Log4(("MSR_K8_CSTAR           = %#RX64\n", ASMRdMsr(MSR_K8_CSTAR)));
                Log4(("MSR_K8_LSTAR           = %#RX64\n", ASMRdMsr(MSR_K8_LSTAR)));
                Log4(("MSR_K6_STAR            = %#RX64\n", ASMRdMsr(MSR_K6_STAR)));
                Log4(("MSR_K8_SF_MASK         = %#RX64\n", ASMRdMsr(MSR_K8_SF_MASK)));
                Log4(("MSR_K8_KERNEL_GS_BASE  = %#RX64\n", ASMRdMsr(MSR_K8_KERNEL_GS_BASE)));
#endif /* VBOX_STRICT */
            break;
        }

        default:
            /* Impossible */
            AssertMsgFailed(("hmR0VmxReportWorldSwitchError %Rrc (%#x)\n", rcVMRun, rcVMRun));
            break;
    }
}


/**
 * Sets up the usage of TSC-offsetting and updates the VMCS.
 *
 * If offsetting is not possible, cause VM-exits on RDTSC(P)s. Also sets up the
 * VMX-preemption timer.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   idCurrentCpu    The current CPU number.
 *
 * @remarks No-long-jump zone!!!
 */
//更新 TSC（Time Stamp Counter）偏移和 配置 VMX 抢占定时器（Preemption Timer），
//确保客户机时间管理正确且高效。
/*
  计算客户机 TSC 偏移（uTscOffset），用于 时间虚拟化（避免客户机直接读取宿主机 TSC）。
  根据情况决定是否启用 RDTSC 退出（VM-Exit on RDTSC）。
*/

/*
   TSC偏移作用：让客户机 RDTSC 读取 (宿主 TSC + uTscOffset)，避免修改客户机代码。
   RDTSC 退出控制：
     如果 能使用 TSC 偏移，则禁用 RDTSC 退出（性能优化）。
     否则强制客户机 RDTSC 触发 VM-Exit，由 VMM 模拟返回虚拟时间。
*/
static void hmR0VmxUpdateTscOffsettingAndPreemptTimer(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient, RTCPUID idCurrentCpu)
{
    bool         fOffsettedTsc;
    bool         fParavirtTsc;
    uint64_t     uTscOffset;
    PVMCC        pVM = pVCpu->CTX_SUFF(pVM);

    /*
       抢占定时器作用：在指定的 TSC ticks 后强制触发 VM-Exit，确保客户机不会独占 CPU。
       性能优化：
         如果 仍在同一 CPU 核心且 deadline 未过期，则复用上次计算结果。
         否则调用 TMCpuTickGetDeadlineAndTscOffset 重新计算。
       范围限制：
         上限 15.625ms（避免长时间不调度）。
         下限 30μs（避免频繁抢占）。
         通过 cPreemptTimerShift 调整精度（右移减少值）。
    */
    if (pVM->hmr0.s.vmx.fUsePreemptTimer)// 检查是否启用抢占定时器
    {
        /* The TMCpuTickGetDeadlineAndTscOffset function is expensive (calling it on
           every entry slowed down the bs2-test1 CPUID testcase by ~33% (on an 10980xe). */
        uint64_t cTicksToDeadline;
        if (   idCurrentCpu == pVCpu->hmr0.s.idLastCpu
            && TMVirtualSyncIsCurrentDeadlineVersion(pVM, pVCpu->hmr0.s.vmx.uTscDeadlineVersion))
        {
            // 复用上次计算的 deadline（性能优化）
            STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatVmxPreemptionReusingDeadline);
            fOffsettedTsc = TMCpuTickCanUseRealTSC(pVM, pVCpu, &uTscOffset, &fParavirtTsc);
            cTicksToDeadline = pVCpu->hmr0.s.vmx.uTscDeadline - SUPReadTsc(); // 剩余 ticks
            if ((int64_t)cTicksToDeadline > 0)
            { /* hopefully */ }
            else
            {
                STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatVmxPreemptionReusingDeadlineExpired);
                cTicksToDeadline = 0; // 已过期，立即触发抢占
            }
        }
        else
        {
            // 重新计算 deadline 和 TSC 偏移
            STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatVmxPreemptionRecalcingDeadline);
            cTicksToDeadline = TMCpuTickGetDeadlineAndTscOffset(pVM, pVCpu, &uTscOffset, &fOffsettedTsc, &fParavirtTsc,
                                                                &pVCpu->hmr0.s.vmx.uTscDeadline,
                                                                &pVCpu->hmr0.s.vmx.uTscDeadlineVersion);
            pVCpu->hmr0.s.vmx.uTscDeadline += cTicksToDeadline;// 更新 deadline
            if (cTicksToDeadline >= 128)
            { /* hopefully */ }
            else
                STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatVmxPreemptionRecalcingDeadlineExpired);
        }

        /* Make sure the returned values have sane upper and lower boundaries. */
        uint64_t const u64CpuHz = SUPGetCpuHzFromGipBySetIndex(g_pSUPGlobalInfoPage, pVCpu->iHostCpuSet);
        // 上限
        cTicksToDeadline   = RT_MIN(cTicksToDeadline, u64CpuHz / 64);      /* 1/64th of a second,  15.625ms. */ /** @todo r=bird: Once real+virtual timers move to separate thread, we can raise the upper limit (16ms isn't much). ASSUMES working poke cpu function. */
        // 下限
        cTicksToDeadline   = RT_MAX(cTicksToDeadline, u64CpuHz / 32678);   /* 1/32768th of a second,  ~30us. */
        // 调整精度
        cTicksToDeadline >>= pVM->hm.s.vmx.cPreemptTimerShift;

        /** @todo r=ramshankar: We need to find a way to integrate nested-guest
         *        preemption timers here. We probably need to clamp the preemption timer,
         *        after converting the timer value to the host. */
        uint32_t const cPreemptionTickCount = (uint32_t)RT_MIN(cTicksToDeadline, UINT32_MAX - 16);
        //写入 VMCS 抢占定时器字段
        int rc = VMXWriteVmcs32(VMX_VMCS32_PREEMPT_TIMER_VALUE, cPreemptionTickCount);
        AssertRC(rc);
    }
    else
        fOffsettedTsc = TMCpuTickCanUseRealTSC(pVM, pVCpu, &uTscOffset, &fParavirtTsc);

    if (fParavirtTsc)// 检查是否使用半虚拟化 TSC（如 Hyper-V/KVM）
    {
        /* Currently neither Hyper-V nor KVM need to update their paravirt. TSC
           information before every VM-entry, hence disable it for performance sake. */
#if 0
        int rc = GIMR0UpdateParavirtTsc(pVM, 0 /* u64Offset */);
        AssertRC(rc);
#endif
        // 注：当前未实际更新半虚拟化 TSC 偏移（性能优化）
        STAM_COUNTER_INC(&pVCpu->hm.s.StatTscParavirt);
    }

    if (   fOffsettedTsc
        && RT_LIKELY(!pVCpu->hmr0.s.fDebugWantRdTscExit))
    {
        if (pVmxTransient->fIsNestedGuest)// 嵌套虚拟化情况
            uTscOffset = CPUMApplyNestedGuestTscOffset(pVCpu, uTscOffset);
        hmR0VmxSetTscOffsetVmcs(pVmxTransient->pVmcsInfo, uTscOffset); // 写入 VMCS
        hmR0VmxRemoveProcCtlsVmcs(pVCpu, pVmxTransient, VMX_PROC_CTLS_RDTSC_EXIT);// 禁用 RDTSC 退出
    }
    else
    {
        /* We can't use TSC-offsetting (non-fixed TSC, warp drive active etc.), VM-exit on RDTSC(P). */
        // 无法使用 TSC 偏移（如非固定 TSC 或时间扭曲激活），需启用 RDTSC 退出
        hmR0VmxSetProcCtlsVmcs(pVmxTransient, VMX_PROC_CTLS_RDTSC_EXIT);
    }
}


/**
 * Saves the guest state from the VMCS into the guest-CPU context.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 * @param   fWhat   What to import, CPUMCTX_EXTRN_XXX.
 */
VMMR0DECL(int) VMXR0ImportStateOnDemand(PVMCPUCC pVCpu, uint64_t fWhat)
{
    AssertPtr(pVCpu);
    PVMXVMCSINFO pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
    return vmxHCImportGuestStateEx(pVCpu, pVmcsInfo, fWhat);
}


/**
 * Gets VMX VM-exit auxiliary information.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxExitAux     Where to store the VM-exit auxiliary info.
 * @param   fWhat           What to fetch, HMVMX_READ_XXX.
 */
VMMR0DECL(int) VMXR0GetExitAuxInfo(PVMCPUCC pVCpu, PVMXEXITAUX pVmxExitAux, uint32_t fWhat)
{
    PVMXTRANSIENT pVmxTransient = pVCpu->hmr0.s.vmx.pVmxTransient;
    if (RT_LIKELY(pVmxTransient))
    {
        AssertCompile(sizeof(fWhat) == sizeof(pVmxTransient->fVmcsFieldsRead));

        /* The exit reason is always available. */
        pVmxExitAux->uReason = pVmxTransient->uExitReason;


        if (fWhat & HMVMX_READ_EXIT_QUALIFICATION)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_EXIT_QUALIFICATION>(pVCpu, pVmxTransient);
            pVmxExitAux->u64Qual = pVmxTransient->uExitQual;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_EXIT_QUALIFICATION;
#endif
        }

        if (fWhat & HMVMX_READ_IDT_VECTORING_INFO)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_IDT_VECTORING_INFO>(pVCpu, pVmxTransient);
            pVmxExitAux->uIdtVectoringInfo = pVmxTransient->uIdtVectoringInfo;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_IDT_VECTORING_INFO;
#endif
        }

        if (fWhat & HMVMX_READ_IDT_VECTORING_ERROR_CODE)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_IDT_VECTORING_ERROR_CODE>(pVCpu, pVmxTransient);
            pVmxExitAux->uIdtVectoringErrCode = pVmxTransient->uIdtVectoringErrorCode;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_IDT_VECTORING_ERROR_CODE;
#endif
        }

        if (fWhat & HMVMX_READ_EXIT_INSTR_LEN)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_EXIT_INSTR_LEN>(pVCpu, pVmxTransient);
            pVmxExitAux->cbInstr = pVmxTransient->cbExitInstr;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_EXIT_INSTR_LEN;
#endif
        }

        if (fWhat & HMVMX_READ_EXIT_INTERRUPTION_INFO)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_EXIT_INTERRUPTION_INFO>(pVCpu, pVmxTransient);
            pVmxExitAux->uExitIntInfo = pVmxTransient->uExitIntInfo;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_EXIT_INTERRUPTION_INFO;
#endif
        }

        if (fWhat & HMVMX_READ_EXIT_INTERRUPTION_ERROR_CODE)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_EXIT_INTERRUPTION_ERROR_CODE>(pVCpu, pVmxTransient);
            pVmxExitAux->uExitIntErrCode = pVmxTransient->uExitIntErrorCode;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_EXIT_INTERRUPTION_ERROR_CODE;
#endif
        }

        if (fWhat & HMVMX_READ_EXIT_INSTR_INFO)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_EXIT_INSTR_INFO>(pVCpu, pVmxTransient);
            pVmxExitAux->InstrInfo.u = pVmxTransient->ExitInstrInfo.u;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_EXIT_INSTR_INFO;
#endif
        }

        if (fWhat & HMVMX_READ_GUEST_LINEAR_ADDR)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_GUEST_LINEAR_ADDR>(pVCpu, pVmxTransient);
            pVmxExitAux->u64GuestLinearAddr = pVmxTransient->uGuestLinearAddr;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_GUEST_LINEAR_ADDR;
#endif
        }

        if (fWhat & HMVMX_READ_GUEST_PHYSICAL_ADDR)
        {
            vmxHCReadToTransientSlow<HMVMX_READ_GUEST_PHYSICAL_ADDR>(pVCpu, pVmxTransient);
            pVmxExitAux->u64GuestPhysAddr = pVmxTransient->uGuestPhysicalAddr;
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_GUEST_PHYSICAL_ADDR;
#endif
        }

        if (fWhat & HMVMX_READ_GUEST_PENDING_DBG_XCPTS)
        {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
            vmxHCReadToTransientSlow<HMVMX_READ_GUEST_PENDING_DBG_XCPTS>(pVCpu, pVmxTransient);
            pVmxExitAux->u64GuestPendingDbgXcpts = pVmxTransient->uGuestPendingDbgXcpts;
#else
            pVmxExitAux->u64GuestPendingDbgXcpts = 0;
#endif
#ifdef VBOX_STRICT
            fWhat &= ~HMVMX_READ_GUEST_PENDING_DBG_XCPTS;
#endif
        }

        AssertMsg(!fWhat, ("fWhat=%#RX32 fVmcsFieldsRead=%#RX32\n", fWhat, pVmxTransient->fVmcsFieldsRead));
        return VINF_SUCCESS;
    }
    return VERR_NOT_AVAILABLE;
}


/**
 * Does the necessary state syncing before returning to ring-3 for any reason
 * (longjmp, preemption, voluntary exits to ring-3) from VT-x.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   fImportState    Whether to import the guest state from the VMCS back
 *                          to the guest-CPU context.
 *
 * @remarks No-long-jmp zone!!!
 */
/*
 *关键设计总结
    原子性保障：通过禁用抢占、断言检查确保操作不可分割。
    状态一致性：强制同步Guest/Host状态，避免残留（如FPU、调试寄存器）。
    性能优化：延迟恢复MSR、条件执行VMCS清理减少开销。
    安全隔离：清除VMCS及嵌套状态，防止跨CPU或跨VM干扰。
    可维护性：通过注释提示代码依赖（如VMXR0CallRing3Callback需同步修改）。
*/
static int hmR0VmxLeave(PVMCPUCC pVCpu, bool fImportState)
{
    //防止在敏感操作中被抢占或切换到用户态，确保原子性。
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));// 确保线程抢占已禁用
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));// 禁止Ring3调用

    RTCPUID const idCpu = RTMpCpuId();//获取当前物理CPU ID
    Log4Func(("HostCpuId=%u\n", idCpu));

    /*
     * !!! IMPORTANT !!!
     * If you modify code here, check whether VMXR0CallRing3Callback() needs to be updated too.
     */

    /* Save the guest state if necessary. */
    //获取活动VMCS
    PVMXVMCSINFO pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
    if (fImportState)
    {
        //将VMCS中的Guest寄存器/状态同步到pVCpu->cpum.GstCtx
        int rc = vmxHCImportGuestStateEx(pVCpu, pVmcsInfo, HMVMX_CPUMCTX_EXTRN_ALL);
        AssertRCReturn(rc, rc);
    }

    /* Restore host FPU state if necessary. We will resync on next R0 reentry. */
    //保存Guest FPU状态，恢复Host FPU。
    CPUMR0FpuStateMaybeSaveGuestAndRestoreHost(pVCpu);
    Assert(!CPUMIsGuestFPUStateActive(pVCpu));// 确认Guest FPU未激活

    /* Restore host debug registers if necessary. We will resync on next R0 reentry. */
#ifdef VMX_WITH_MAYBE_ALWAYS_INTERCEPT_MOV_DRX
    Assert(   (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_MOV_DR_EXIT)
           ||  pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs
           || (!CPUMIsHyperDebugStateActive(pVCpu) && !pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fAlwaysInterceptMovDRx));
#else
    Assert(   (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_MOV_DR_EXIT)
           ||  pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs
           || !CPUMIsHyperDebugStateActive(pVCpu)); // Guest调试状态已清除
#endif
    //保存Guest调试寄存器（如DR0-DR7），恢复Host值。
	//安全：防止Guest通过调试寄存器干扰Host。
    CPUMR0DebugStateMaybeSaveGuestAndRestoreHost(pVCpu, true /* save DR6 */);
    Assert(!CPUMIsGuestDebugStateActive(pVCpu));
    Assert(!CPUMIsHyperDebugStateActive(pVCpu));

    /* Restore host-state bits that VT-x only restores partially. */
    if (pVCpu->hmr0.s.vmx.fRestoreHostFlags > VMX_RESTORE_HOST_REQUIRED)
    {
        Log4Func(("Restoring Host State: fRestoreHostFlags=%#RX32 HostCpuId=%u\n", pVCpu->hmr0.s.vmx.fRestoreHostFlags, idCpu));
        //场景：某些Host寄存器（如CR0/CR4）在VMX模式下可能被修改，需手动还原。
        VMXRestoreHostState(pVCpu->hmr0.s.vmx.fRestoreHostFlags, &pVCpu->hmr0.s.vmx.RestoreHost);
    }
    pVCpu->hmr0.s.vmx.fRestoreHostFlags = 0;//CR4.VMXE位需关闭以退出VMX模式。

    /* Restore the lazy host MSRs as we're leaving VT-x context. */
    //延迟加载（Lazy）‌：优化性能，仅在必要时处理MSR。
    if (pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_LOADED_GUEST)
    {
        /* We shouldn't restore the host MSRs without saving the guest MSRs first. */
        if (!fImportState)
        {
            // 确保导入Guest MSR后再恢复Host MSR
			//关键MSR：如KERNEL_GS_BASE（x86-64系统调用相关）。
            int rc = vmxHCImportGuestStateEx(pVCpu, pVmcsInfo, CPUMCTX_EXTRN_KERNEL_GS_BASE | CPUMCTX_EXTRN_SYSCALL_MSRS);
            AssertRCReturn(rc, rc);
        }
        hmR0VmxLazyRestoreHostMsrs(pVCpu);// 恢复Host MSR
        Assert(!pVCpu->hmr0.s.vmx.fLazyMsrs);
    }
    else
        pVCpu->hmr0.s.vmx.fLazyMsrs = 0;

    /* Update auto-load/store host MSRs values when we re-enter VT-x (as we could be on a different CPU). */
    pVCpu->hmr0.s.vmx.fUpdatedHostAutoMsrs = false;

    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatEntry);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatImportGuestState);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatExportGuestState);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatPreExit);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatExitHandling);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatExitIO);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatExitMovCRx);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatExitXcptNmi);
    STAM_PROFILE_ADV_SET_STOPPED(&pVCpu->hm.s.StatExitVmentry);
    STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchLongJmpToR3);

    //原子操作：从“硬件虚拟化中”切换到“执行中”，防止状态竞争。
    VMCPU_CMPXCHG_STATE(pVCpu, VMCPUSTATE_STARTED_HM, VMCPUSTATE_STARTED_EXEC);

    /** @todo This partially defeats the purpose of having preemption hooks.
     *  The problem is, deregistering the hooks should be moved to a place that
     *  lasts until the EMT is about to be destroyed not everytime while leaving HM
     *  context.
     */
    // 执行VMCLEAR指令
    int rc = hmR0VmxClearVmcs(pVmcsInfo);
    AssertRCReturn(rc, rc);

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    /*
     * A valid shadow VMCS is made active as part of VM-entry. It is necessary to
     * clear a shadow VMCS before allowing that VMCS to become active on another
     * logical processor. We may or may not be importing guest state which clears
     * it, so cover for it here.
     *
     * See Intel spec. 24.11.1 "Software Use of Virtual-Machine Control Structures".
     */
    if (   pVmcsInfo->pvShadowVmcs
        && pVmcsInfo->fShadowVmcsState != VMX_V_VMCS_LAUNCH_STATE_CLEAR)
    {
        rc = vmxHCClearShadowVmcs(pVmcsInfo);
        AssertRCReturn(rc, rc);
    }

    /*
     * Flag that we need to re-export the host state if we switch to this VMCS before
     * executing guest or nested-guest code.
     */
    pVmcsInfo->idHostCpuState = NIL_RTCPUID;
#endif

    Log4Func(("Cleared Vmcs. HostCpuId=%u\n", idCpu));
    NOREF(idCpu);
    return VINF_SUCCESS;
}


/**
 * Leaves the VT-x session.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jmp zone!!!
 */
//这是一个静态函数，用于安全地退出VT-x虚拟化会话
//在x86架构中，从Ring0（内核态）到Ring3（用户态）的切换需要长跳转（Far Jump）
static int hmR0VmxLeaveSession(PVMCPUCC pVCpu)
{
    HM_DISABLE_PREEMPT(pVCpu);
    HMVMX_ASSERT_CPU_SAFE(pVCpu);//验证当前CPU与虚拟CPU（vCPU）绑定，避免跨CPU操作导致状态不一致。
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));//确保未启用Ring3调用，防止在敏感路径中触发用户态代码。
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));//确认线程抢占已禁用，避免并发问题

    /* When thread-context hooks are used, we can avoid doing the leave again if we had been preempted before
       and done this from the VMXR0ThreadCtxCallback(). */
    if (!pVCpu->hmr0.s.fLeaveDone)//通过fLeaveDone标志避免重复退出操作
    {
        int rc2 = hmR0VmxLeave(pVCpu, true /* fImportState */);//调用hmR0VmxLeave()执行实际退出操作,并导入Guest状态
        AssertRCReturnStmt(rc2, HM_RESTORE_PREEMPT(), rc2);
        pVCpu->hmr0.s.fLeaveDone = true;
    }
    Assert(!pVCpu->cpum.GstCtx.fExtrn);//确保Guest状态已完全导出，无残留外部状态。

    /*
     * !!! IMPORTANT !!!
     * If you modify code here, make sure to check whether VMXR0CallRing3Callback() needs to be updated too.
     */

    /* Deregister hook now that we've left HM context before re-enabling preemption. */
    /** @todo Deregistering here means we need to VMCLEAR always
     *        (longjmp/exit-to-r3) in VT-x which is not efficient, eliminate need
     *        for calling VMMR0ThreadCtxHookDisable here! */
	//当前设计导致每次退出必须执行VMCLEAR指令（清理VMCS），可能带来性能损耗，未来需优化以减少不必要的调用。
    VMMR0ThreadCtxHookDisable(pVCpu);//：注销线程上下文切换钩子，避免后续无效回调。

    /* Leave HM context. This takes care of local init (term) and deregistering the longjmp-to-ring-3 callback. */
    int rc = HMR0LeaveCpu(pVCpu);//执行CPU级别的清理工作，包括取消长跳转至Ring3的回调注册。
    HM_RESTORE_PREEMPT();//恢复线程抢占，结束关键区。
    return rc;
}


/**
 * Take necessary actions before going back to ring-3.
 *
 * An action requires us to go back to ring-3. This function does the necessary
 * steps before we can safely return to ring-3. This is not the same as longjmps
 * to ring-3, this is voluntary and prepares the guest so it may continue
 * executing outside HM (recompiler/IEM).
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 * @param   rcExit  The reason for exiting to ring-3. Can be
 *                  VINF_VMM_UNKNOWN_RING3_CALL.
 */
//在 VMX 非根模式（Non-Root Mode）下发生 VM-Exit后，需要 返回 Ring-3（宿主机用户态）时的处理逻辑。
//核心目标是 安全退出 VMX 模式，并确保 客户机状态正确同步。
/*
    处理 VM-Exit 错误（如 VERR_VMX_INVALID_VMCS_PTR）。
    转换未处理的 HM 事件为 TRPM 陷阱（供后续 Ring-3 处理）。
    清理 VMCS 控制字段（如中断窗口、NMI 窗口）。
    保存客户机状态并恢复宿主机状态（hmR0VmxLeaveSession）。
    同步客户机 CPU 状态（如 MSR、段寄存器、TLB 等）。
    更新退出原因（pVCpu->hm.s.rcLastExitToR3）。
*/
static int hmR0VmxExitToRing3(PVMCPUCC pVCpu, VBOXSTRICTRC rcExit)
{
    HMVMX_ASSERT_PREEMPT_SAFE(pVCpu);

    PVMXVMCSINFO pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
    //如果 VM-Exit 是由于 VMCS 指针无效触发的，记录当前 VMCS 物理地址、版本号和 CPU ID，便于调试。
    if (RT_UNLIKELY(rcExit == VERR_VMX_INVALID_VMCS_PTR))
    {
        VMXGetCurrentVmcs(&pVCpu->hm.s.vmx.LastError.HCPhysCurrentVmcs);
        pVCpu->hm.s.vmx.LastError.u32VmcsRev   = *(uint32_t *)pVmcsInfo->pvVmcs;
        pVCpu->hm.s.vmx.LastError.idEnteredCpu = pVCpu->hmr0.s.idEnteredCpu;
        /* LastError.idCurrentCpu was updated in hmR0VmxPreRunGuestCommitted(). */
    }

    /* Please, no longjumps here (any logging shouldn't flush jump back to ring-3). NO LOGGING BEFORE THIS POINT! */
    VMMRZCallRing3Disable(pVCpu);// 防止跳转到 Ring-3
    Log4Func(("rcExit=%d\n", VBOXSTRICTRC_VAL(rcExit))); // 记录退出原因

    /*
     * Convert any pending HM events back to TRPM due to premature exits to ring-3.
     * We need to do this only on returns to ring-3 and not for longjmps to ring3.
     *
     * This is because execution may continue from ring-3 and we would need to inject
     * the event from there (hence place it back in TRPM).
     */
    if (pVCpu->hm.s.Event.fPending)
    {
        //如果 HM 有未处理的事件（如外部中断、异常），将其转换为 TRPM（Trap Manager）陷阱，供 Ring-3 后续处理。
        vmxHCPendingEventToTrpmTrap(pVCpu); // 转换为 TRPM 陷阱
        Assert(!pVCpu->hm.s.Event.fPending);

        /* Clear the events from the VMCS. */
        //清理 VMCS 相关字段，避免重复注入。
        int rc = VMXWriteVmcs32(VMX_VMCS32_CTRL_ENTRY_INTERRUPTION_INFO, 0);    AssertRC(rc);
        rc     = VMXWriteVmcs32(VMX_VMCS_GUEST_PENDING_DEBUG_XCPTS, 0);         AssertRC(rc);
    }
#ifdef VBOX_STRICT
    /*
     * We check for rcExit here since for errors like VERR_VMX_UNABLE_TO_START_VM (which are
     * fatal), we don't care about verifying duplicate injection of events. Errors like
     * VERR_EM_INTERPRET are converted to their VINF_* counterparts -prior- to  calling this
     * function so those should and will be checked below.
     */
    else if (RT_SUCCESS(rcExit))
    {
        /*
         * Ensure we don't accidentally clear a pending HM event without clearing the VMCS.
         * This can be pretty hard to debug otherwise, interrupts might get injected twice
         * occasionally, see @bugref{9180#c42}.
         *
         * However, if the VM-entry failed, any VM entry-interruption info. field would
         * be left unmodified as the event would not have been injected to the guest. In
         * such cases, don't assert, we're not going to continue guest execution anyway.
         */
        /* 检查是否意外清除了未处理的 HM 事件 */
        uint32_t uExitReason;
        uint32_t uEntryIntInfo;
        int rc = VMXReadVmcs32(VMX_VMCS32_RO_EXIT_REASON, &uExitReason);
        rc    |= VMXReadVmcs32(VMX_VMCS32_CTRL_ENTRY_INTERRUPTION_INFO, &uEntryIntInfo);
        AssertRC(rc);
        AssertMsg(VMX_EXIT_REASON_HAS_ENTRY_FAILED(uExitReason) || !VMX_ENTRY_INT_INFO_IS_VALID(uEntryIntInfo),
                  ("uExitReason=%#RX32 uEntryIntInfo=%#RX32 rcExit=%d\n", uExitReason, uEntryIntInfo, VBOXSTRICTRC_VAL(rcExit)));
    }
#endif

    /*
     * Clear the interrupt-window and NMI-window VMCS controls as we could have got
     * a VM-exit with higher priority than interrupt-window or NMI-window VM-exits
     * (e.g. TPR below threshold).
     */
    if (!CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx))
    {
        //中断窗口（Interrupt-Window）和 NMI 窗口（NMI-Window）是 VMCS 的控制字段，用于在特定时机触发 VM-Exit。
        //在退出前 显式清除，避免影响后续 VM-Entry。
        Assert(!pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs);
        vmxHCClearIntWindowExitVmcs(pVCpu, pVmcsInfo); // 清除中断窗口
        vmxHCClearNmiWindowExitVmcs(pVCpu, pVmcsInfo); // 清除 NMI 窗口
    }

    /* If we're emulating an instruction, we shouldn't have any TRPM traps pending
       and if we're injecting an event we should have a TRPM trap pending. */
    //如果退出原因是 注入 TRPM 事件（如中断/异常），则必须有 待处理的陷阱。
    AssertMsg(rcExit != VINF_EM_RAW_INJECT_TRPM_EVENT || TRPMHasTrap(pVCpu), ("%Rrc\n", VBOXSTRICTRC_VAL(rcExit)));
#ifndef DEBUG_bird /* Triggered after firing an NMI against NT4SP1, possibly a triple fault in progress. */
    //如果退出原因是 模拟指令，则 不能有未处理的陷阱。
    AssertMsg(rcExit != VINF_EM_RAW_EMULATE_INSTR || !TRPMHasTrap(pVCpu), ("%Rrc\n", VBOXSTRICTRC_VAL(rcExit)));
#endif

    /* Save guest state and restore host state bits. */
    /*hmR0VmxLeaveSession:
       保存客户机寄存器、MSR、FPU 状态。
       恢复宿主机 FPU、调试寄存器。
       清理 VMCS 并退出 VMX 模式。
    */
    int rc = hmR0VmxLeaveSession(pVCpu);// 保存客户机状态，恢复宿主机状态
    AssertRCReturn(rc, rc);
    STAM_COUNTER_DEC(&pVCpu->hm.s.StatSwitchLongJmpToR3);

    /* Thread-context hooks are unregistered at this point!!! */
    /* Ring-3 callback notifications are unregistered at this point!!! */

    /* Sync recompiler state. */
    VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_TO_R3);
    //标记客户机 CPU 状态已更改，确保后续操作（如内存访问）能正确同步。
    CPUMSetChangedFlags(pVCpu, CPUM_CHANGED_SYSENTER_MSR
                             | CPUM_CHANGED_LDTR
                             | CPUM_CHANGED_GDTR
                             | CPUM_CHANGED_IDTR
                             | CPUM_CHANGED_TR
                             | CPUM_CHANGED_HIDDEN_SEL_REGS);
    //如果启用了 嵌套分页（Nested Paging, EPT/NPT），且客户机启用了分页，则 刷新全局 TLB。
    if (   pVCpu->CTX_SUFF(pVM)->hmr0.s.fNestedPaging
        && CPUMIsGuestPagingEnabledEx(&pVCpu->cpum.GstCtx))
        CPUMSetChangedFlags(pVCpu, CPUM_CHANGED_GLOBAL_TLB_FLUSH);

    Assert(!pVCpu->hmr0.s.fClearTrapFlag);

    /* Update the exit-to-ring 3 reason. */
    //记录 VM-Exit 原因，供 Ring-3 后续处理（如模拟 I/O 或调度决策）。
    pVCpu->hm.s.rcLastExitToR3 = VBOXSTRICTRC_VAL(rcExit);

    /* On our way back from ring-3 reload the guest state if there is a possibility of it being changed. */
    if (   rcExit != VINF_EM_RAW_INTERRUPT
        || CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx))
    {
        Assert(!(pVCpu->cpum.GstCtx.fExtrn & HMVMX_CPUMCTX_EXTRN_ALL));
        ASMAtomicUoOrU64(&pVCpu->hm.s.fCtxChanged, HM_CHANGED_ALL_GUEST);
    }

    STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchExitToR3);
    VMMRZCallRing3Enable(pVCpu);
    return rc;
}


/**
 * VMMRZCallRing3() callback wrapper which saves the guest state before we
 * longjump due to a ring-0 assertion.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 */
//这段代码是 VirtualBox‌ 在 Intel VT-x
//模式下处理 断言（Assertion）失败时的回调函数，
//用于 紧急退出 VMX 模式，确保 主机状态恢复，
//避免因断言失败导致系统崩溃或数据损坏。
#if 0
    快速退出 VMX 模式，避免触发更多断言（Assert）。
    恢复主机状态（FPU、调试寄存器、MSR等），防止客户机状态污染宿主机。
    清理 VMCS（Virtual Machine Control Structure），确保后续 VMX 操作能正确执行。
#endif
VMMR0DECL(int) VMXR0AssertionCallback(PVMCPUCC pVCpu)
{
    /*
     * !!! IMPORTANT !!!
     * If you modify code here, check whether hmR0VmxLeave() and hmR0VmxLeaveSession() needs to be updated too.
     * This is a stripped down version which gets out ASAP, trying to not trigger any further assertions.
     */
    VMMR0AssertionRemoveNotification(pVCpu);//移除可能的通知机制，防止进一步触发断言。
    VMMRZCallRing3Disable(pVCpu);//禁用 Ring-3 调用，避免陷入用户态。
    HM_DISABLE_PREEMPT(pVCpu);//禁用抢占，确保当前线程不会被调度出去。

    //获取当前活动的 VMCS（可能是普通 VMCS 或嵌套 VMCS）。
    PVMXVMCSINFO pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
    //强制导入 所有客户机状态（包括 GPRs、控制寄存器等），确保 VMCS 数据同步到内存。
    vmxHCImportGuestStateEx(pVCpu, pVmcsInfo, HMVMX_CPUMCTX_EXTRN_ALL);
    //保存客户机 FPU 状态，恢复主机 FPU 状态。
    CPUMR0FpuStateMaybeSaveGuestAndRestoreHost(pVCpu);
    //保存客户机调试寄存器（DR6），恢复主机调试状态。
    CPUMR0DebugStateMaybeSaveGuestAndRestoreHost(pVCpu, true /* save DR6 */);

    /* Restore host-state bits that VT-x only restores partially. */
    //检查是否需要额外恢复主机状态（如 FS/GS 基址、Kernel GS 基址等）。
    if (pVCpu->hmr0.s.vmx.fRestoreHostFlags > VMX_RESTORE_HOST_REQUIRED)
        VMXRestoreHostState(pVCpu->hmr0.s.vmx.fRestoreHostFlags, &pVCpu->hmr0.s.vmx.RestoreHost);
    pVCpu->hmr0.s.vmx.fRestoreHostFlags = 0;

    /* Restore the lazy host MSRs as we're leaving VT-x context. */
    //如果客户机 MSR 已加载，则恢复主机 MSR（如 SYSCALL 相关 MSR）。
    if (pVCpu->hmr0.s.vmx.fLazyMsrs & VMX_LAZY_MSRS_LOADED_GUEST)
        hmR0VmxLazyRestoreHostMsrs(pVCpu);

    /* Update auto-load/store host MSRs values when we re-enter VT-x (as we could be on a different CPU). */
    //标记 主机 MSR 自动加载状态为无效（下次进入 VMX 时需要重新加载）。
    pVCpu->hmr0.s.vmx.fUpdatedHostAutoMsrs = false;
    VMCPU_CMPXCHG_STATE(pVCpu, VMCPUSTATE_STARTED_HM, VMCPUSTATE_STARTED_EXEC);//更新 CPU 状态，表示已退出 VMX 模式。

    /* Clear the current VMCS data back to memory (shadow VMCS if any would have been
       cleared as part of importing the guest state above. */
    hmR0VmxClearVmcs(pVmcsInfo);//将 VMCS 数据写回内存（如果是 Shadow VMCS，则已通过 vmxHCImportGuestStateEx 清理）。

    /** @todo eliminate the need for calling VMMR0ThreadCtxHookDisable here!  */
    VMMR0ThreadCtxHookDisable(pVCpu);

    /* Leave HM context. This takes care of local init (term). */
    HMR0LeaveCpu(pVCpu);
    HM_RESTORE_PREEMPT();
    return VINF_SUCCESS;
}


/**
 * Enters the VT-x session.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 */
//虚拟机进入函数VMXR0Enter的实现，主要负责在切换到VMX non-root模式（客户机模式）前的准备工作
VMMR0DECL(int) VMXR0Enter(PVMCPUCC pVCpu)
{
    AssertPtr(pVCpu);
    Assert(pVCpu->CTX_SUFF(pVM)->hm.s.vmx.fSupported);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));

    LogFlowFunc(("pVCpu=%p\n", pVCpu));
    Assert((pVCpu->hm.s.fCtxChanged &  (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE))
                                    == (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE));

#ifdef VBOX_STRICT
    /* At least verify VMX is enabled, since we can't check if we're in VMX root mode without #GP'ing. */
    RTCCUINTREG uHostCr4 = ASMGetCR4();//在严格模式（VBOX_STRICT）下检查CR4寄存器的VMXE位是否启用（Intel VT-x的必要条件）
    if (!(uHostCr4 & X86_CR4_VMXE))
    {
        LogRelFunc(("X86_CR4_VMXE bit in CR4 is not set!\n"));
        return VERR_VMX_X86_CR4_VMXE_CLEARED;
    }
#endif

    /*
     * Do the EMT scheduled L1D and MDS flush here if needed.
     */
    if (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_L1D_SCHED)
        ASMWrMsr(MSR_IA32_FLUSH_CMD, MSR_IA32_FLUSH_CMD_F_L1D);
    else if (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_MDS_SCHED)
        hmR0MdsClear();

    /*
     * Load the appropriate VMCS as the current and active one.
     */
    PVMXVMCSINFO pVmcsInfo;
    //嵌套虚拟化判断
    bool const fInNestedGuestMode = CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx);
    //选择VMCS
    if (!fInNestedGuestMode)
        pVmcsInfo = &pVCpu->hmr0.s.vmx.VmcsInfo;
    else
        pVmcsInfo = &pVCpu->hmr0.s.vmx.VmcsInfoNstGst;
    //加载VMCS
    int rc = hmR0VmxLoadVmcs(pVmcsInfo);
    if (RT_SUCCESS(rc))
    {
        pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs           = fInNestedGuestMode;
        pVCpu->hm.s.vmx.fSwitchedToNstGstVmcsCopyForRing3 = fInNestedGuestMode;
        //通过fLeaveDone=false标记VMCS未退出状态，确保后续VMXR0Leave能正确处理状态恢复
        pVCpu->hmr0.s.fLeaveDone = false;
        Log4Func(("Loaded %s Vmcs. HostCpuId=%u\n", fInNestedGuestMode ? "nested-guest" : "guest", RTMpCpuId()));
    }
    return rc;
}


/**
 * The thread-context callback.
 *
 * This is used together with RTThreadCtxHookCreate() on platforms which
 * supports it, and directly from VMMR0EmtPrepareForBlocking() and
 * VMMR0EmtResumeAfterBlocking() on platforms which don't.
 *
 * @param   enmEvent        The thread-context event.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   fGlobalInit     Whether global VT-x/AMD-V init. was used.
 * @thread  EMT(pVCpu)
 */
//enmEvent：线程上下文事件类型（IN/OUT）
//pVCpu：当前vCPU上下文指针
//这个函数是VirtualBox中处理线程上下文切换回调的核心实现，
//主要用于在硬件虚拟化（VT-x）环境下处理线程被抢占或恢复时的状态管理
VMMR0DECL(void) VMXR0ThreadCtxCallback(RTTHREADCTXEVENT enmEvent, PVMCPUCC pVCpu, bool fGlobalInit)
{
    AssertPtr(pVCpu);
    RT_NOREF1(fGlobalInit);

    switch (enmEvent)
    {
        //RTTHREADCTXEVENT_OUT因抢占触发，需优先恢复主机状态（避免死锁和状态污染）
        case RTTHREADCTXEVENT_OUT:
        {
            Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
            VMCPU_ASSERT_EMT(pVCpu);//确保在EMT（Emulation Thread）上下文中执行

            /* No longjmps (logger flushes, locks) in this fragile context. */
            VMMRZCallRing3Disable(pVCpu); // 禁用Ring3回调
            Log4Func(("Preempting: HostCpuId=%u\n", RTMpCpuId()));

            /* Restore host-state (FPU, debug etc.) */
            if (!pVCpu->hmr0.s.fLeaveDone)
            {
                /*
                 * Do -not- import the guest-state here as we might already be in the middle of importing
                 * it, esp. bad if we're holding the PGM lock, see comment at the end of vmxHCImportGuestStateEx().
                 */
                // 退出VMX上下文（不导入客户机状态）,确保主机状态（FPU/调试寄存器等）正确恢复即可
                hmR0VmxLeave(pVCpu, false /* fImportState */); 
                pVCpu->hmr0.s.fLeaveDone = true;
            }

            /* Leave HM context, takes care of local init (term). */
            int rc = HMR0LeaveCpu(pVCpu);// 清理CPU本地状态
            AssertRC(rc);

            /* Restore longjmp state. */
            VMMRZCallRing3Enable(pVCpu); // 恢复Ring3回调
            STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatSwitchPreempt);
            break;
        }

        case RTTHREADCTXEVENT_IN:
        {
            Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
            VMCPU_ASSERT_EMT(pVCpu);

            /* Do the EMT scheduled L1D and MDS flush here if needed. */
            //根据需要执行L1D和MDS缓存清除(安全相关)
            if (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_L1D_SCHED)
                ASMWrMsr(MSR_IA32_FLUSH_CMD, MSR_IA32_FLUSH_CMD_F_L1D);
            else if (pVCpu->hmr0.s.fWorldSwitcher & HM_WSF_MDS_SCHED)
                hmR0MdsClear();

            /* No longjmps here, as we don't want to trigger preemption (& its hook) while resuming. */
            VMMRZCallRing3Disable(pVCpu);//禁用Ring3回调
            Log4Func(("Resumed: HostCpuId=%u\n", RTMpCpuId()));

            /* Initialize the bare minimum state required for HM. This takes care of
               initializing VT-x if necessary (onlined CPUs, local init etc.) */
            //初始化HM(Hardware Virtualization)所需的最小状态
            int rc = hmR0EnterCpu(pVCpu);
            AssertRC(rc);
            Assert(   (pVCpu->hm.s.fCtxChanged & (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE))
                                              == (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE));

            /* Load the active VMCS as the current one. */
            //加载活动的VMCS(虚拟机控制结构)
            PVMXVMCSINFO pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
            rc = hmR0VmxLoadVmcs(pVmcsInfo);
            AssertRC(rc);
            Log4Func(("Resumed: Loaded Vmcs. HostCpuId=%u\n", RTMpCpuId()));
            pVCpu->hmr0.s.fLeaveDone = false;

            /* Restore longjmp state. */
            //重新启用Ring3回调
            VMMRZCallRing3Enable(pVCpu);
            break;
        }

        default:
            break;
    }
}


/**
 * Exports the host state into the VMCS host-state area.
 * Sets up the VM-exit MSR-load area.
 *
 * The CPU state will be loaded from these fields on every successful VM-exit.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
//VM-exit时同步宿主机状态到VMCS
/*
  VMCS字段映射
    CR4导出对应VMCS_HOST_CR4字段（0x6C14）
    段寄存器基址写入VMCS_HOST_ES_BASE等字段（0x6C00~0x6C0E）
*/
static int hmr0vmxexporthoststate(pvmcpucc pvcpu)
{
    assert(!rtthreadpreemptisenabled(nil_rtthread));

    int rc = vinf_success;
    if (pvcpu->hm.s.fctxchanged & hm_changed_host_context)
    {
        uint64_t uhostcr4 = hmr0vmxexporthostcontrolregs();// 控制寄存器(CR4)

        rc = hmr0vmxexporthostsegmentregs(pvcpu, uhostcr4);// 段寄存器
        assertlogrelmsgrcreturn(rc, ("rc=%rrc\n", rc), rc);

        hmr0vmxexporthostmsrs(pvcpu);// MSR寄存器

        pvcpu->hm.s.fctxchanged &= ~hm_changed_host_context;
    }
    return rc;
}


/**
 * Saves the host state in the VMCS host-state.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
/*
graph TB
  A[VM-entry] --> B[导出客户机状态]
  C[VM-exit] --> D[导出主机状态]
  D --> E[恢复Host上下文]
*/
//该函数实现图中D阶段操作
VMMR0DECL(int) VMXR0ExportHostState(PVMCPUCC pVCpu)
{
    AssertPtr(pVCpu);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));//确保处于非抢占上下文, 线程调度禁止

    /*
     * Export the host state here while entering HM context.
     * When thread-context hooks are used, we might get preempted and have to re-save the host
     * state but most of the time we won't be, so do it here before we disable interrupts.
     */
    return hmR0VmxExportHostState(pVCpu);
}


/**
 * Exports the guest state into the VMCS guest-state area.
 *
 * The will typically be done before VM-entry when the guest-CPU state and the
 * VMCS state may potentially be out of sync.
 *
 * Sets up the VM-entry MSR-load and VM-exit MSR-store areas. Sets up the
 * VM-entry controls.
 * Sets up the appropriate VMX non-root function to execute guest code based on
 * the guest CPU mode.
 *
 * @returns VBox strict status code.
 * @retval  VINF_EM_RESCHEDULE_REM if we try to emulate non-paged guest code
 *          without unrestricted guest execution and the VMMDev is not presently
 *          mapped (e.g. EFI32).
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
static VBOXSTRICTRC hmR0VmxExportGuestState(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient)
{
    AssertPtr(pVCpu);
    HMVMX_ASSERT_PREEMPT_SAFE(pVCpu);
    LogFlowFunc(("pVCpu=%p\n", pVCpu));

    STAM_PROFILE_ADV_START(&pVCpu->hm.s.StatExportGuestState, x);

    /*
     * Determine real-on-v86 mode.
     * Used when the guest is in real-mode and unrestricted guest execution is not used.
     */
    PVMXVMCSINFOSHARED pVmcsInfoShared = pVmxTransient->pVmcsInfo->pShared;
    if (    pVCpu->CTX_SUFF(pVM)->hmr0.s.vmx.fUnrestrictedGuest
        || !CPUMIsGuestInRealModeEx(&pVCpu->cpum.GstCtx)) //根据 fUnrestrictedGuest 能力和客户机模式,设置 RealMode.fRealOnV86Active 标志
        pVmcsInfoShared->RealMode.fRealOnV86Active = false;
    else
    {
        Assert(!pVmxTransient->fIsNestedGuest);
        //影响后续段寄存器和 CR0 的导出方式（如虚拟 8086 模式处理）
        pVmcsInfoShared->RealMode.fRealOnV86Active = true;
    }

    /*
     * Any ordering dependency among the sub-functions below must be explicitly stated using comments.
     * Ideally, assert that the cross-dependent bits are up-to-date at the point of using it.
     */
    int rc = vmxHCExportGuestEntryExitCtls(pVCpu, pVmxTransient);
    AssertLogRelMsgRCReturn(rc, ("rc=%Rrc\n", rc), rc);

    //控制寄存器组 CR0
    rc = vmxHCExportGuestCR0(pVCpu, pVmxTransient);
    AssertLogRelMsgRCReturn(rc, ("rc=%Rrc\n", rc), rc);

    //控制寄存器组 CR3 CR4
    VBOXSTRICTRC rcStrict = vmxHCExportGuestCR3AndCR4(pVCpu, pVmxTransient);
    if (rcStrict == VINF_SUCCESS)
    { /* likely */ }
    else
    {
        Assert(rcStrict == VINF_EM_RESCHEDULE_REM || RT_FAILURE_NP(rcStrict));
        return rcStrict;
    }

    //内存管理单元, （含 GDTR/IDTR)
    rc = vmxHCExportGuestSegRegsXdtr(pVCpu, pVmxTransient);
    AssertLogRelMsgRCReturn(rc, ("rc=%Rrc\n", rc), rc);

    //扩展状态
    rc = hmR0VmxExportGuestMsrs(pVCpu, pVmxTransient);
    AssertLogRelMsgRCReturn(rc, ("rc=%Rrc\n", rc), rc);

    vmxHCExportGuestApicTpr(pVCpu, pVmxTransient);
    vmxHCExportGuestXcptIntercepts(pVCpu, pVmxTransient);

    //执行上下文：RIP/RSP/RFLAGS 的最终同步
    vmxHCExportGuestRip(pVCpu);
    hmR0VmxExportGuestRsp(pVCpu);
    vmxHCExportGuestRflags(pVCpu, pVmxTransient);

    //处理嵌套虚拟化相关状态（如 VMCS 链接指针）
    rc = hmR0VmxExportGuestHwvirtState(pVCpu, pVmxTransient);
    AssertLogRelMsgRCReturn(rc, ("rc=%Rrc\n", rc), rc);

    /* Clear any bits that may be set but exported unconditionally or unused/reserved bits. */
    ASMAtomicUoAndU64(&pVCpu->hm.s.fCtxChanged, ~(  (HM_CHANGED_GUEST_GPRS_MASK & ~HM_CHANGED_GUEST_RSP)
                                                  |  HM_CHANGED_GUEST_CR2
                                                  | (HM_CHANGED_GUEST_DR_MASK & ~HM_CHANGED_GUEST_DR7)
                                                  |  HM_CHANGED_GUEST_X87
                                                  |  HM_CHANGED_GUEST_SSE_AVX
                                                  |  HM_CHANGED_GUEST_OTHER_XSAVE
                                                  |  HM_CHANGED_GUEST_XCRx
                                                  |  HM_CHANGED_GUEST_KERNEL_GS_BASE /* Part of lazy or auto load-store MSRs. */
                                                  |  HM_CHANGED_GUEST_SYSCALL_MSRS   /* Part of lazy or auto load-store MSRs. */
                                                  |  HM_CHANGED_GUEST_TSC_AUX
                                                  |  HM_CHANGED_GUEST_OTHER_MSRS
                                                  | (HM_CHANGED_KEEPER_STATE_MASK & ~HM_CHANGED_VMX_MASK)));

    STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatExportGuestState, x);
    return rc;
}


/**
 * Exports the state shared between the host and guest into the VMCS.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
//处理 VMX 模式下共享状态导出的核心函数，主要用于在 VM-exit 时同步 Guest 与 Host 的共享硬件状态
static void hmR0VmxExportSharedState(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));

    if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_GUEST_DR_MASK)
    {
        int rc = hmR0VmxExportSharedDebugState(pVCpu, pVmxTransient); //通过 hmR0VmxExportSharedDebugState 同步 DRx 寄存器，并可能触发 RFLAGS.TF 位更新
        AssertRC(rc);
        pVCpu->hm.s.fCtxChanged &= ~HM_CHANGED_GUEST_DR_MASK;

        /* Loading shared debug bits might have changed eflags.TF bit for debugging purposes. */
        if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_GUEST_RFLAGS)
            vmxHCExportGuestRflags(pVCpu, pVmxTransient);
    }

    if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_VMX_GUEST_LAZY_MSRS)
    {
        hmR0VmxLazyLoadGuestMsrs(pVCpu); //调用 hmR0VmxLazyLoadGuestMsrs 延迟加载客户机模型特定寄存器
        pVCpu->hm.s.fCtxChanged &= ~HM_CHANGED_VMX_GUEST_LAZY_MSRS;
    }

    AssertMsg(!(pVCpu->hm.s.fCtxChanged & HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE),
              ("fCtxChanged=%#RX64\n", pVCpu->hm.s.fCtxChanged));
}


/**
 * Worker for loading the guest-state bits in the inner VT-x execution loop.
 *
 * @returns Strict VBox status code (i.e. informational status codes too).
 * @retval  VINF_EM_RESCHEDULE_REM if we try to emulate non-paged guest code
 *          without unrestricted guest execution and the VMMDev is not presently
 *          mapped (e.g. EFI32).
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks No-long-jump zone!!!
 */
/*
 函数目的：
    根据CPU状态变化情况，选择最优方式导出客户机状态到VMCS(虚拟机控制结构)
    尽可能减少不必要的状态导出以提高性能
*/
static VBOXSTRICTRC hmR0VmxExportGuestStateOptimal(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient)
{
    HMVMX_ASSERT_PREEMPT_SAFE(pVCpu);
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));

#ifdef HMVMX_ALWAYS_SYNC_FULL_GUEST_STATE
    ASMAtomicUoOrU64(&pVCpu->hm.s.fCtxChanged, HM_CHANGED_ALL_GUEST);
#endif

    /*
     * For many VM-exits only RIP/RSP/RFLAGS (and HWVIRT state when executing a nested-guest)
     * changes. First try to export only these without going through all other changed-flag checks.
     */
    VBOXSTRICTRC   rcStrict;
    uint64_t const fCtxMask     = HM_CHANGED_ALL_GUEST & ~HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE;
    uint64_t const fMinimalMask = HM_CHANGED_GUEST_RIP | HM_CHANGED_GUEST_RSP | HM_CHANGED_GUEST_RFLAGS | HM_CHANGED_GUEST_HWVIRT;
    uint64_t const fCtxChanged  = ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged);

    /* If only RIP/RSP/RFLAGS/HWVIRT changed, export only those (quicker, happens more often).*/
    //定义了几种状态掩码：
      //fCtxMask - 所有需要导出的状态位，排除主机/客户机共享状态
      //fMinimalMask - 仅包含RIP/RSP/RFLAGS/HWVIRT这些最常变化的状态
    if (    (fCtxChanged & fMinimalMask)
        && !(fCtxChanged & (fCtxMask & ~fMinimalMask)))
    {
        vmxHCExportGuestRip(pVCpu);//仅导出这些寄存器(vmxHCExportGuestRip等函数)
        hmR0VmxExportGuestRsp(pVCpu);
        vmxHCExportGuestRflags(pVCpu, pVmxTransient);
        rcStrict = hmR0VmxExportGuestHwvirtState(pVCpu, pVmxTransient);
        STAM_COUNTER_INC(&pVCpu->hm.s.StatExportMinimal); //增加最小导出计数器
    }
    /* If anything else also changed, go through the full export routine and export as required. */
    else if (fCtxChanged & fCtxMask)
    {
        rcStrict = hmR0VmxExportGuestState(pVCpu, pVmxTransient);//执行完整状态导出(hmR0VmxExportGuestState)
        if (RT_LIKELY(rcStrict == VINF_SUCCESS))
        { /* likely */}
        else
        {
            AssertMsg(rcStrict == VINF_EM_RESCHEDULE_REM, ("Failed to export guest state! rc=%Rrc\n",
                                                           VBOXSTRICTRC_VAL(rcStrict)));
            Assert(!VMMRZCallRing3IsEnabled(pVCpu));
            return rcStrict;
        }
        STAM_COUNTER_INC(&pVCpu->hm.s.StatExportFull);//增加完整导出计数器
    }
    /* Nothing changed, nothing to load here. */
    else
        rcStrict = VINF_SUCCESS;

#ifdef VBOX_STRICT
    /* All the guest state bits should be loaded except maybe the host context and/or the shared host/guest bits. */
    uint64_t const fCtxChangedCur = ASMAtomicUoReadU64(&pVCpu->hm.s.fCtxChanged);//在严格检查模式下(VBOX_STRICT)会验证所有状态位是否已正确导出
    AssertMsg(!(fCtxChangedCur & fCtxMask), ("fCtxChangedCur=%#RX64\n", fCtxChangedCur));
#endif
    return rcStrict;
}


/**
 * Map the APIC-access page for virtualizing APIC accesses.
 *
 * This can cause a longjumps to R3 due to the acquisition of the PGM lock. Hence,
 * this not done as part of exporting guest state, see @bugref{8721}.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   GCPhysApicBase  The guest-physical address of the APIC access page.
 */
//将客户机物理地址 GCPhysApicBase 映射到主机预分配的 APIC-access page 物理页
//（pVM->hmr0.s.vmx.HCPhysApicAccess），实现虚拟 APIC 访问的硬件加速
static int hmR0VmxMapHCApicAccessPage(PVMCPUCC pVCpu, RTGCPHYS GCPhysApicBase)
{
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    Assert(GCPhysApicBase);

    LogFunc(("Mapping HC APIC-access page at %#RGp\n", GCPhysApicBase));

    /* Unalias the existing mapping. */
    int rc = PGMHandlerPhysicalReset(pVM, GCPhysApicBase); //解除原有MMIO映射
    AssertRCReturn(rc, rc);

    /* Map the HC APIC-access page in place of the MMIO page, also updates the shadow page tables if necessary. */
    Assert(pVM->hmr0.s.vmx.HCPhysApicAccess != NIL_RTHCPHYS);
    //建立新物理页映射
    //X86_PTE_RW | X86_PTE_P：设置页表项为可读写且存在
    //HCPhysApicAccess 是预先通过 SUPR0ContAlloc 分配的连续物理内存
    rc = IOMR0MmioMapMmioHCPage(pVM, pVCpu, GCPhysApicBase, pVM->hmr0.s.vmx.HCPhysApicAccess, X86_PTE_RW | X86_PTE_P);
    AssertRCReturn(rc, rc);

    return VINF_SUCCESS;
}


/**
 * Worker function passed to RTMpOnSpecific() that is to be called on the target
 * CPU.
 *
 * @param   idCpu       The ID for the CPU the function is called on.
 * @param   pvUser1     Null, not used.
 * @param   pvUser2     Null, not used.
 */
static DECLCALLBACK(void) hmR0DispatchHostNmi(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    RT_NOREF3(idCpu, pvUser1, pvUser2);
    VMXDispatchHostNmi();
}


/**
 * Dispatching an NMI on the host CPU that received it.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pVmcsInfo   The VMCS info. object corresponding to the VMCS that was
 *                      executing when receiving the host NMI in VMX non-root
 *                      operation.
 */
/*
当虚拟机因 NMI 触发 VM-exit 时，需将 NMI 正确传递到主机物理 CPU。该函数处理两种场景：
  当前 CPU 就是目标 CPU：直接处理
  目标 CPU 是其他核心：通过 IPI（处理器间中断）跨核传递
*/
static int hmR0VmxExitHostNmi(PVMCPUCC pVCpu, PCVMXVMCSINFO pVmcsInfo)
{
    RTCPUID const idCpu = pVmcsInfo->idHostCpuExec;
    Assert(idCpu != NIL_RTCPUID);

    /*
     * We don't want to delay dispatching the NMI any more than we have to. However,
     * we have already chosen -not- to dispatch NMIs when interrupts were still disabled
     * after executing guest or nested-guest code for the following reasons:
     *
     *   - We would need to perform VMREADs with interrupts disabled and is orders of
     *     magnitude worse when we run as a nested hypervisor without VMCS shadowing
     *     supported by the host hypervisor.
     *
     *   - It affects the common VM-exit scenario and keeps interrupts disabled for a
     *     longer period of time just for handling an edge case like host NMIs which do
     *     not occur nearly as frequently as other VM-exits.
     *
     * Let's cover the most likely scenario first. Check if we are on the target CPU
     * and dispatch the NMI right away. This should be much faster than calling into
     * RTMpOnSpecific() machinery.
     */
    bool fDispatched = false;
    RTCCUINTREG const fEFlags = ASMIntDisableFlags();
    if (idCpu == RTMpCpuId())
    {
        VMXDispatchHostNmi();
        fDispatched = true;
    }
    ASMSetFlags(fEFlags);
    if (fDispatched)
    {
        STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatExitHostNmiInGC);
        return VINF_SUCCESS;
    }

    /*
     * RTMpOnSpecific() waits until the worker function has run on the target CPU. So
     * there should be no race or recursion even if we are unlucky enough to be preempted
     * (to the target CPU) without dispatching the host NMI above.
     */
    STAM_REL_COUNTER_INC(&pVCpu->hm.s.StatExitHostNmiInGCIpi);
    //IPI 机制, 使用 RTMpOnSpecific() 向目标 CPU 发送请求：
    return RTMpOnSpecific(idCpu, &hmR0DispatchHostNmi, NULL /* pvUser1 */,  NULL /* pvUser2 */);
}


#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
/**
 * Merges the guest with the nested-guest MSR bitmap in preparation of executing the
 * nested-guest using hardware-assisted VMX.
 *
 * @param   pVCpu               The cross context virtual CPU structure.
 * @param   pVmcsInfoNstGst     The nested-guest VMCS info. object.
 * @param   pVmcsInfoGst        The guest VMCS info. object.
 */
static void hmR0VmxMergeMsrBitmapNested(PCVMCPUCC pVCpu, PVMXVMCSINFO pVmcsInfoNstGst, PCVMXVMCSINFO pVmcsInfoGst)
{
    uint32_t const cbMsrBitmap    = X86_PAGE_4K_SIZE;
    uint64_t       *pu64MsrBitmap = (uint64_t *)pVmcsInfoNstGst->pvMsrBitmap;
    Assert(pu64MsrBitmap);

    /*
     * We merge the guest MSR bitmap with the nested-guest MSR bitmap such that any
     * MSR that is intercepted by the guest is also intercepted while executing the
     * nested-guest using hardware-assisted VMX.
     *
     * Note! If the nested-guest is not using an MSR bitmap, every MSR must cause a
     *       nested-guest VM-exit even if the outer guest is not intercepting some
     *       MSRs. We cannot assume the caller has initialized the nested-guest
     *       MSR bitmap in this case.
     *
     *       The nested hypervisor may also switch whether it uses MSR bitmaps for
     *       each of its VM-entry, hence initializing it once per-VM while setting
     *       up the nested-guest VMCS is not sufficient.
     */
    PCVMXVVMCS const pVmcsNstGst  = &pVCpu->cpum.GstCtx.hwvirt.vmx.Vmcs;
    if (pVmcsNstGst->u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS)
    {
        uint64_t const *pu64MsrBitmapNstGst = (uint64_t const *)&pVCpu->cpum.GstCtx.hwvirt.vmx.abMsrBitmap[0];
        uint64_t const *pu64MsrBitmapGst    = (uint64_t const *)pVmcsInfoGst->pvMsrBitmap;
        Assert(pu64MsrBitmapNstGst);
        Assert(pu64MsrBitmapGst);

        /** @todo Detect and use EVEX.POR? */
        uint32_t const cFrags = cbMsrBitmap / sizeof(uint64_t);
        for (uint32_t i = 0; i < cFrags; i++)
            pu64MsrBitmap[i] = pu64MsrBitmapNstGst[i] | pu64MsrBitmapGst[i];
    }
    else
        ASMMemFill32(pu64MsrBitmap, cbMsrBitmap, UINT32_C(0xffffffff));
}


/**
 * Merges the guest VMCS in to the nested-guest VMCS controls in preparation of
 * hardware-assisted VMX execution of the nested-guest.
 *
 * For a guest, we don't modify these controls once we set up the VMCS and hence
 * this function is never called.
 *
 * For nested-guests since the nested hypervisor provides these controls on every
 * nested-guest VM-entry and could potentially change them everytime we need to
 * merge them before every nested-guest VM-entry.
 *
 * @returns VBox status code.
 * @param   pVCpu   The cross context virtual CPU structure.
 */
static int hmR0VmxMergeVmcsNested(PVMCPUCC pVCpu)
{
    PVMCC const         pVM          = pVCpu->CTX_SUFF(pVM);
    PCVMXVMCSINFO const pVmcsInfoGst = &pVCpu->hmr0.s.vmx.VmcsInfo;
    PCVMXVVMCS const    pVmcsNstGst  = &pVCpu->cpum.GstCtx.hwvirt.vmx.Vmcs;

    /*
     * Merge the controls with the requirements of the guest VMCS.
     *
     * We do not need to validate the nested-guest VMX features specified in the nested-guest
     * VMCS with the features supported by the physical CPU as it's already done by the
     * VMLAUNCH/VMRESUME instruction emulation.
     *
     * This is because the VMX features exposed by CPUM (through CPUID/MSRs) to the guest are
     * derived from the VMX features supported by the physical CPU.
     */

    /* Pin-based VM-execution controls. */
    uint32_t const u32PinCtls = pVmcsNstGst->u32PinCtls | pVmcsInfoGst->u32PinCtls;

    /* Processor-based VM-execution controls. */
    uint32_t       u32ProcCtls = (pVmcsNstGst->u32ProcCtls  & ~VMX_PROC_CTLS_USE_IO_BITMAPS)
                               | (pVmcsInfoGst->u32ProcCtls & ~(  VMX_PROC_CTLS_INT_WINDOW_EXIT
                                                                | VMX_PROC_CTLS_NMI_WINDOW_EXIT
                                                                | VMX_PROC_CTLS_MOV_DR_EXIT /* hmR0VmxExportSharedDebugState makes
                                                                                               sure guest DRx regs are loaded. */
                                                                | VMX_PROC_CTLS_USE_TPR_SHADOW
                                                                | VMX_PROC_CTLS_MONITOR_TRAP_FLAG));

    /* Secondary processor-based VM-execution controls. */
    uint32_t const u32ProcCtls2 = (pVmcsNstGst->u32ProcCtls2  & ~VMX_PROC_CTLS2_VPID)
                                | (pVmcsInfoGst->u32ProcCtls2 & ~(  VMX_PROC_CTLS2_VIRT_APIC_ACCESS
                                                                  | VMX_PROC_CTLS2_INVPCID
                                                                  | VMX_PROC_CTLS2_VMCS_SHADOWING
                                                                  | VMX_PROC_CTLS2_RDTSCP
                                                                  | VMX_PROC_CTLS2_XSAVES_XRSTORS
                                                                  | VMX_PROC_CTLS2_APIC_REG_VIRT
                                                                  | VMX_PROC_CTLS2_VIRT_INT_DELIVERY
                                                                  | VMX_PROC_CTLS2_VMFUNC));

    /*
     * VM-entry controls:
     * These controls contains state that depends on the nested-guest state (primarily
     * EFER MSR) and is thus not constant between VMLAUNCH/VMRESUME and the nested-guest
     * VM-exit. Although the nested hypervisor cannot change it, we need to in order to
     * properly continue executing the nested-guest if the EFER MSR changes but does not
     * cause a nested-guest VM-exits.
     *
     * VM-exit controls:
     * These controls specify the host state on return. We cannot use the controls from
     * the nested hypervisor state as is as it would contain the guest state rather than
     * the host state. Since the host state is subject to change (e.g. preemption, trips
     * to ring-3, longjmp and rescheduling to a different host CPU) they are not constant
     * through VMLAUNCH/VMRESUME and the nested-guest VM-exit.
     *
     * VM-entry MSR-load:
     * The guest MSRs from the VM-entry MSR-load area are already loaded into the guest-CPU
     * context by the VMLAUNCH/VMRESUME instruction emulation.
     *
     * VM-exit MSR-store:
     * The VM-exit emulation will take care of populating the MSRs from the guest-CPU context
     * back into the VM-exit MSR-store area.
     *
     * VM-exit MSR-load areas:
     * This must contain the real host MSRs with hardware-assisted VMX execution. Hence, we
     * can entirely ignore what the nested hypervisor wants to load here.
     */

    /*
     * Exception bitmap.
     *
     * We could remove #UD from the guest bitmap and merge it with the nested-guest bitmap
     * here (and avoid doing anything while exporting nested-guest state), but to keep the
     * code more flexible if intercepting exceptions become more dynamic in the future we do
     * it as part of exporting the nested-guest state.
     */
    uint32_t const u32XcptBitmap = pVmcsNstGst->u32XcptBitmap | pVmcsInfoGst->u32XcptBitmap;

    /*
     * CR0/CR4 guest/host mask.
     *
     * Modifications by the nested-guest to CR0/CR4 bits owned by the host and the guest must
     * cause VM-exits, so we need to merge them here.
     */
    uint64_t const u64Cr0Mask = pVmcsNstGst->u64Cr0Mask.u | pVmcsInfoGst->u64Cr0Mask;
    uint64_t const u64Cr4Mask = pVmcsNstGst->u64Cr4Mask.u | pVmcsInfoGst->u64Cr4Mask;

    /*
     * Page-fault error-code mask and match.
     *
     * Although we require unrestricted guest execution (and thereby nested-paging) for
     * hardware-assisted VMX execution of nested-guests and thus the outer guest doesn't
     * normally intercept #PFs, it might intercept them for debugging purposes.
     *
     * If the outer guest is not intercepting #PFs, we can use the nested-guest #PF filters.
     * If the outer guest is intercepting #PFs, we must intercept all #PFs.
     */
    uint32_t u32XcptPFMask;
    uint32_t u32XcptPFMatch;
    if (!(pVmcsInfoGst->u32XcptBitmap & RT_BIT(X86_XCPT_PF)))
    {
        u32XcptPFMask  = pVmcsNstGst->u32XcptPFMask;
        u32XcptPFMatch = pVmcsNstGst->u32XcptPFMatch;
    }
    else
    {
        u32XcptPFMask  = 0;
        u32XcptPFMatch = 0;
    }

    /*
     * Pause-Loop exiting.
     */
    /** @todo r=bird: given that both pVM->hm.s.vmx.cPleGapTicks and
     *        pVM->hm.s.vmx.cPleWindowTicks defaults to zero, I cannot see how
     *        this will work... */
    uint32_t const cPleGapTicks    = RT_MIN(pVM->hm.s.vmx.cPleGapTicks,    pVmcsNstGst->u32PleGap);
    uint32_t const cPleWindowTicks = RT_MIN(pVM->hm.s.vmx.cPleWindowTicks, pVmcsNstGst->u32PleWindow);

    /*
     * Pending debug exceptions.
     * Currently just copy whatever the nested-guest provides us.
     */
    uint64_t const uPendingDbgXcpts = pVmcsNstGst->u64GuestPendingDbgXcpts.u;

    /*
     * I/O Bitmap.
     *
     * We do not use the I/O bitmap that may be provided by the nested hypervisor as we always
     * intercept all I/O port accesses.
     */
    Assert(u32ProcCtls & VMX_PROC_CTLS_UNCOND_IO_EXIT);
    Assert(!(u32ProcCtls & VMX_PROC_CTLS_USE_IO_BITMAPS));

    /*
     * VMCS shadowing.
     *
     * We do not yet expose VMCS shadowing to the guest and thus VMCS shadowing should not be
     * enabled while executing the nested-guest.
     */
    Assert(!(u32ProcCtls2 & VMX_PROC_CTLS2_VMCS_SHADOWING));

    /*
     * APIC-access page.
     */
    RTHCPHYS HCPhysApicAccess;
    if (u32ProcCtls2 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS)
    {
        Assert(g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS);
        RTGCPHYS const GCPhysApicAccess = pVmcsNstGst->u64AddrApicAccess.u;

        void          *pvPage;
        PGMPAGEMAPLOCK PgLockApicAccess;
        int rc = PGMPhysGCPhys2CCPtr(pVM, GCPhysApicAccess, &pvPage, &PgLockApicAccess);
        if (RT_SUCCESS(rc))
        {
            rc = PGMPhysGCPhys2HCPhys(pVM, GCPhysApicAccess, &HCPhysApicAccess);
            AssertMsgRCReturn(rc, ("Failed to get host-physical address for APIC-access page at %#RGp\n", GCPhysApicAccess), rc);

            /** @todo Handle proper releasing of page-mapping lock later. */
            PGMPhysReleasePageMappingLock(pVCpu->CTX_SUFF(pVM), &PgLockApicAccess);
        }
        else
            return rc;
    }
    else
        HCPhysApicAccess = 0;

    /*
     * Virtual-APIC page and TPR threshold.
     */
    RTHCPHYS HCPhysVirtApic;
    uint32_t u32TprThreshold;
    if (u32ProcCtls & VMX_PROC_CTLS_USE_TPR_SHADOW)
    {
        Assert(g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_TPR_SHADOW);
        RTGCPHYS const GCPhysVirtApic = pVmcsNstGst->u64AddrVirtApic.u;

        void          *pvPage;
        PGMPAGEMAPLOCK PgLockVirtApic;
        int rc = PGMPhysGCPhys2CCPtr(pVM, GCPhysVirtApic, &pvPage, &PgLockVirtApic);
        if (RT_SUCCESS(rc))
        {
            rc = PGMPhysGCPhys2HCPhys(pVM, GCPhysVirtApic, &HCPhysVirtApic);
            AssertMsgRCReturn(rc, ("Failed to get host-physical address for virtual-APIC page at %#RGp\n", GCPhysVirtApic), rc);

            /** @todo Handle proper releasing of page-mapping lock later. */
            PGMPhysReleasePageMappingLock(pVCpu->CTX_SUFF(pVM), &PgLockVirtApic);
        }
        else
            return rc;

        u32TprThreshold = pVmcsNstGst->u32TprThreshold;
    }
    else
    {
        HCPhysVirtApic  = 0;
        u32TprThreshold = 0;

        /*
         * We must make sure CR8 reads/write must cause VM-exits when TPR shadowing is not
         * used by the nested hypervisor. Preventing MMIO accesses to the physical APIC will
         * be taken care of by EPT/shadow paging.
         */
        if (pVM->hmr0.s.fAllow64BitGuests)
            u32ProcCtls |= VMX_PROC_CTLS_CR8_STORE_EXIT
                        |  VMX_PROC_CTLS_CR8_LOAD_EXIT;
    }

    /*
     * Validate basic assumptions.
     */
    PVMXVMCSINFO pVmcsInfoNstGst = &pVCpu->hmr0.s.vmx.VmcsInfoNstGst;
    Assert(pVM->hmr0.s.vmx.fUnrestrictedGuest);
    Assert(g_HmMsrs.u.vmx.ProcCtls.n.allowed1 & VMX_PROC_CTLS_USE_SECONDARY_CTLS);
    Assert(hmGetVmxActiveVmcsInfo(pVCpu) == pVmcsInfoNstGst);

    /*
     * Commit it to the nested-guest VMCS.
     */
    int rc = VINF_SUCCESS;
    if (pVmcsInfoNstGst->u32PinCtls != u32PinCtls)
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PIN_EXEC, u32PinCtls);
    if (pVmcsInfoNstGst->u32ProcCtls != u32ProcCtls)
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC, u32ProcCtls);
    if (pVmcsInfoNstGst->u32ProcCtls2 != u32ProcCtls2)
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PROC_EXEC2, u32ProcCtls2);
    if (pVmcsInfoNstGst->u32XcptBitmap != u32XcptBitmap)
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_EXCEPTION_BITMAP, u32XcptBitmap);
    if (pVmcsInfoNstGst->u64Cr0Mask != u64Cr0Mask)
        rc |= VMXWriteVmcsNw(VMX_VMCS_CTRL_CR0_MASK, u64Cr0Mask);
    if (pVmcsInfoNstGst->u64Cr4Mask != u64Cr4Mask)
        rc |= VMXWriteVmcsNw(VMX_VMCS_CTRL_CR4_MASK, u64Cr4Mask);
    if (pVmcsInfoNstGst->u32XcptPFMask != u32XcptPFMask)
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PAGEFAULT_ERROR_MASK, u32XcptPFMask);
    if (pVmcsInfoNstGst->u32XcptPFMatch != u32XcptPFMatch)
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PAGEFAULT_ERROR_MATCH, u32XcptPFMatch);
    if (   !(u32ProcCtls  & VMX_PROC_CTLS_PAUSE_EXIT)
        &&  (u32ProcCtls2 & VMX_PROC_CTLS2_PAUSE_LOOP_EXIT))
    {
        Assert(g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_PAUSE_LOOP_EXIT);
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PLE_GAP, cPleGapTicks);
        rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_PLE_WINDOW, cPleWindowTicks);
    }
    if (pVmcsInfoNstGst->HCPhysVirtApic != HCPhysVirtApic)
        rc |= VMXWriteVmcs64(VMX_VMCS64_CTRL_VIRT_APIC_PAGEADDR_FULL, HCPhysVirtApic);
    rc |= VMXWriteVmcs32(VMX_VMCS32_CTRL_TPR_THRESHOLD, u32TprThreshold);
    if (u32ProcCtls2 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS)
        rc |= VMXWriteVmcs64(VMX_VMCS64_CTRL_APIC_ACCESSADDR_FULL, HCPhysApicAccess);
    rc |= VMXWriteVmcsNw(VMX_VMCS_GUEST_PENDING_DEBUG_XCPTS, uPendingDbgXcpts);
    AssertRC(rc);

    /*
     * Update the nested-guest VMCS cache.
     */
    pVmcsInfoNstGst->u32PinCtls     = u32PinCtls;
    pVmcsInfoNstGst->u32ProcCtls    = u32ProcCtls;
    pVmcsInfoNstGst->u32ProcCtls2   = u32ProcCtls2;
    pVmcsInfoNstGst->u32XcptBitmap  = u32XcptBitmap;
    pVmcsInfoNstGst->u64Cr0Mask     = u64Cr0Mask;
    pVmcsInfoNstGst->u64Cr4Mask     = u64Cr4Mask;
    pVmcsInfoNstGst->u32XcptPFMask  = u32XcptPFMask;
    pVmcsInfoNstGst->u32XcptPFMatch = u32XcptPFMatch;
    pVmcsInfoNstGst->HCPhysVirtApic = HCPhysVirtApic;

    /*
     * We need to flush the TLB if we are switching the APIC-access page address.
     * See Intel spec. 28.3.3.4 "Guidelines for Use of the INVEPT Instruction".
     */
    if (u32ProcCtls2 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS)
        pVCpu->hm.s.vmx.fSwitchedNstGstFlushTlb = true;

    /*
     * MSR bitmap.
     *
     * The MSR bitmap address has already been initialized while setting up the nested-guest
     * VMCS, here we need to merge the MSR bitmaps.
     */
    if (u32ProcCtls & VMX_PROC_CTLS_USE_MSR_BITMAPS)
        hmR0VmxMergeMsrBitmapNested(pVCpu, pVmcsInfoNstGst, pVmcsInfoGst);

    return VINF_SUCCESS;
}
#endif /* VBOX_WITH_NESTED_HWVIRT_VMX */


/**
 * Does the preparations before executing guest code in VT-x.
 *
 * This may cause longjmps to ring-3 and may even result in rescheduling to the
 * recompiler/IEM. We must be cautious what we do here regarding committing
 * guest-state information into the VMCS assuming we assuredly execute the
 * guest in VT-x mode.
 *
 * If we fall back to the recompiler/IEM after updating the VMCS and clearing
 * the common-state (TRPM/forceflags), we must undo those changes so that the
 * recompiler/IEM can (and should) use them when it resumes guest execution.
 * Otherwise such operations must be done when we can no longer exit to ring-3.
 *
 * @returns Strict VBox status code (i.e. informational status codes too).
 * @retval  VINF_SUCCESS if we can proceed with running the guest, interrupts
 *          have been disabled.
 * @retval  VINF_VMX_VMEXIT if a nested-guest VM-exit occurs (e.g., while evaluating
 *          pending events).
 * @retval  VINF_EM_RESET if a triple-fault occurs while injecting a
 *          double-fault into the guest.
 * @retval  VINF_EM_DBG_STEPPED if @a fStepping is true and an event was
 *          dispatched directly.
 * @retval  VINF_* scheduling changes, we have to go back to ring-3.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   fStepping       Whether we are single-stepping the guest in the
 *                          hypervisor debugger. Makes us ignore some of the reasons
 *                          for returning to ring-3, and return VINF_EM_DBG_STEPPED
 *                          if event dispatching took place.
 */
/*
 hmR0VmxPreRunGuest 负责在进入 VMX non-root 模式运行客户机代码之前，完成必要的准备工作。主要处理：
    嵌套虚拟化检查
    强制标志(force flags)处理
    APIC 虚拟化设置
    待处理事件注入
    控制寄存器状态同步
*/

/*
关键机制与技术
1. 中断虚拟化优化
  虚拟 APIC：通过 VMX_PROC_CTLS2_VIRT_APIC_ACCESS 将 APIC 访问重定向到内存页，减少 VM-exit。
  中断窗口：在 VMCS 中设置 Interrupt Window，允许客户机在合适时机接收中断。
2. 嵌套虚拟化支持
  VMCS 合并：将嵌套客户机的控制字段与当前 VMCS 结合，确保二级 Hypervisor 的正确行为。
  VMX 指令模拟：捕获 VMLAUNCH/VMRESUME 等指令，由 Hypervisor 模拟执行。
3. 事件优先级管理
  事件优先级：评估中断、NMI、异常的执行顺序，确保符合 CPU 行为。
  注入冲突处理：检测无效注入（如注入 #PF 时 CR2 未更新），触发三重故障。
4. 状态同步策略
  按需导出：仅同步被修改的客户机状态（通过 fCtxChanged 标志）。
  惰性加载：推迟 FPU/调试寄存器的保存/恢复，直到首次使用。
*/
static VBOXSTRICTRC hmR0VmxPreRunGuest(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient, bool fStepping)
{
    Assert(VMMRZCallRing3IsEnabled(pVCpu));// 确保处于可调用 Ring3 的上下文

    Log4Func(("fIsNested=%RTbool fStepping=%RTbool\n", pVmxTransient->fIsNestedGuest, fStepping));// 调试日志

#ifdef VBOX_WITH_NESTED_HWVIRT_ONLY_IN_IEM
    if (pVmxTransient->fIsNestedGuest)
    {
        RT_NOREF2(pVCpu, fStepping);
        Log2Func(("Rescheduling to IEM due to nested-hwvirt or forced IEM exec -> VINF_EM_RESCHEDULE_REM\n"));
        return VINF_EM_RESCHEDULE_REM;
    }
#endif

    /*
     * Check and process force flag actions, some of which might require us to go back to ring-3.
     */
    //处理可能要求返回到 ring-3 的特殊情况（如需要模拟某些指令）
    /*
       处理场景：
         VM 重置请求（VM_FF_RESET）。
         定时器同步（VM_FF_TM_VIRTUAL_SYNC）。
         调试事件（单步执行 fStepping）。
    */
    VBOXSTRICTRC rcStrict = vmxHCCheckForceFlags(pVCpu, pVmxTransient->fIsNestedGuest, fStepping);
    if (rcStrict == VINF_SUCCESS)
    {
        /* FFs don't get set all the time. */
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        if (   pVmxTransient->fIsNestedGuest
            && !CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx))
        {
            STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchNstGstVmexit);
            return VINF_VMX_VMEXIT;
        }
#endif
    }
    else
        return rcStrict;

    /*
     * Virtualize memory-mapped accesses to the physical APIC (may take locks).
     */
    //通过硬件虚拟化支持（VMX_PROC_CTLS2_VIRT_APIC_ACCESS）减少 APIC 访问的 VM-exit
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    if (   !pVCpu->hm.s.vmx.u64GstMsrApicBase
        && (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VIRT_APIC_ACCESS)
        && PDMHasApic(pVM))
    {
        /* Get the APIC base MSR from the virtual APIC device. */
        uint64_t const uApicBaseMsr = APICGetBaseMsrNoCheck(pVCpu);// 获取虚拟 APIC 基址

        /* Map the APIC access page. */
        int rc = hmR0VmxMapHCApicAccessPage(pVCpu, uApicBaseMsr & ~(RTGCPHYS)GUEST_PAGE_OFFSET_MASK); // 映射 APIC 访问页
        AssertRCReturn(rc, rc);

        /* Update the per-VCPU cache of the APIC base MSR corresponding to the mapped APIC access page. */
        pVCpu->hm.s.vmx.u64GstMsrApicBase = uApicBaseMsr; // 缓存基址
    }

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    /*
     * Merge guest VMCS controls with the nested-guest VMCS controls.
     *
     * Even if we have not executed the guest prior to this (e.g. when resuming from a
     * saved state), we should be okay with merging controls as we initialize the
     * guest VMCS controls as part of VM setup phase.
     */
    if (   pVmxTransient->fIsNestedGuest
        && !pVCpu->hm.s.vmx.fMergedNstGstCtls)
    {
        int rc = hmR0VmxMergeVmcsNested(pVCpu);
        AssertRCReturn(rc, rc);
        pVCpu->hm.s.vmx.fMergedNstGstCtls = true;
    }
#endif

    /*
     * Evaluate events to be injected into the guest.
     *
     * Events in TRPM can be injected without inspecting the guest state.
     * If any new events (interrupts/NMI) are pending currently, we try to set up the
     * guest to cause a VM-exit the next time they are ready to receive the event.
     */
    /*
     事件类型：
          硬件中断：通过 APIC 传递的外部中断。
          软件异常：如 #PF 页面错误。
          调试异常：单步执行时的 #DB。
     注入逻辑：通过 VMCS 的 VMX_ENTRY_INT_INFO 字段注入事件，若失败触发三重故障（VINF_EM_RESET）。
    */
    if (TRPMHasTrap(pVCpu))
        vmxHCTrpmTrapToPendingEvent(pVCpu);// 转换陷阱为待处理事件

    uint32_t fIntrState;
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    if (!pVmxTransient->fIsNestedGuest)
        rcStrict = vmxHCEvaluatePendingEvent(pVCpu, pVmxTransient->pVmcsInfo, &fIntrState);  // 评估事件优先级
    else
        rcStrict = vmxHCEvaluatePendingEventNested(pVCpu, pVmxTransient->pVmcsInfo, &fIntrState);

    /*
     * While evaluating pending events if something failed (unlikely) or if we were
     * preparing to run a nested-guest but performed a nested-guest VM-exit, we should bail.
     */
    if (rcStrict != VINF_SUCCESS)
        return rcStrict;
    if (   pVmxTransient->fIsNestedGuest
        && !CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx))
    {
        STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchNstGstVmexit);
        return VINF_VMX_VMEXIT;
    }
#else
    rcStrict = vmxHCEvaluatePendingEvent(pVCpu, pVmxTransient->pVmcsInfo, &fIntrState);
    Assert(rcStrict == VINF_SUCCESS);
#endif

    /*
     * Event injection may take locks (currently the PGM lock for real-on-v86 case) and thus
     * needs to be done with longjmps or interrupts + preemption enabled. Event injection might
     * also result in triple-faulting the VM.
     *
     * With nested-guests, the above does not apply since unrestricted guest execution is a
     * requirement. Regardless, we do this here to avoid duplicating code elsewhere.
     */
    rcStrict = vmxHCInjectPendingEvent(pVCpu, pVmxTransient->pVmcsInfo, pVmxTransient->fIsNestedGuest, fIntrState, fStepping); // 注入事件
    if (RT_LIKELY(rcStrict == VINF_SUCCESS))
    { /* likely */ }
    else
    {
        AssertMsg(rcStrict == VINF_EM_RESET || (rcStrict == VINF_EM_DBG_STEPPED && fStepping),
                  ("%Rrc\n", VBOXSTRICTRC_VAL(rcStrict)));
        return rcStrict;
    }

    /*
     * A longjump might result in importing CR3 even for VM-exits that don't necessarily
     * import CR3 themselves. We will need to update them here, as even as late as the above
     * hmR0VmxInjectPendingEvent() call may lazily import guest-CPU state on demand causing
     * the below force flags to be set.
     */
    /*
       关键状态：
         CR3：确保客户机页表正确加载。
         RFLAGS/CS:RIP：指令指针与标志寄存器。
         段寄存器：CS/DS/ES 等保护模式配置。
    */
    if (VMCPU_FF_IS_SET(pVCpu, VMCPU_FF_HM_UPDATE_CR3))
    {
        Assert(!(ASMAtomicUoReadU64(&pVCpu->cpum.GstCtx.fExtrn) & CPUMCTX_EXTRN_CR3));
        int rc2 = PGMUpdateCR3(pVCpu, CPUMGetGuestCR3(pVCpu)); // 更新分页结构
        AssertMsgReturn(rc2 == VINF_SUCCESS || rc2 == VINF_PGM_SYNC_CR3,
                        ("%Rrc\n", rc2), RT_FAILURE_NP(rc2) ? rc2 : VERR_IPE_UNEXPECTED_INFO_STATUS);
        Assert(!VMCPU_FF_IS_SET(pVCpu, VMCPU_FF_HM_UPDATE_CR3));
    }

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    /* Paranoia. */
    Assert(!pVmxTransient->fIsNestedGuest || CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx));
#endif

    /*
     * No longjmps to ring-3 from this point on!!!
     * Asserts() will still longjmp to ring-3 (but won't return), which is intentional, better than a kernel panic.
     * This also disables flushing of the R0-logger instance (if any).
     */
    //执行环境加固
    //原子性保障：在 VMLAUNCH/VMRESUME 前确保无抢占或中断干扰。
    VMMRZCallRing3Disable(pVCpu); // 禁止 Ring3 长跳转

    /*
     * Export the guest state bits.
     *
     * We cannot perform longjmps while loading the guest state because we do not preserve the
     * host/guest state (although the VMCS will be preserved) across longjmps which can cause
     * CPU migration.
     *
     * If we are injecting events to a real-on-v86 mode guest, we would have updated RIP and some segment
     * registers. Hence, exporting of the guest state needs to be done -after- injection of events.
     */
    rcStrict = hmR0VmxExportGuestStateOptimal(pVCpu, pVmxTransient); // 导出寄存器到 VMCS
    if (RT_LIKELY(rcStrict == VINF_SUCCESS))
    { /* likely */ }
    else
    {
        VMMRZCallRing3Enable(pVCpu);
        return rcStrict;
    }

    /*
     * We disable interrupts so that we don't miss any interrupts that would flag preemption
     * (IPI/timers etc.) when thread-context hooks aren't used and we've been running with
     * preemption disabled for a while.  Since this is purely to aid the
     * RTThreadPreemptIsPending() code, it doesn't matter that it may temporarily reenable and
     * disable interrupt on NT.
     *
     * We need to check for force-flags that could've possible been altered since we last
     * checked them (e.g. by PDMGetInterrupt() leaving the PDM critical section,
     * see @bugref{6398}).
     *
     * We also check a couple of other force-flags as a last opportunity to get the EMT back
     * to ring-3 before executing guest code.
     */
    pVmxTransient->fEFlags = ASMIntDisableFlags(); // 禁用中断

    if (   (   !VM_FF_IS_ANY_SET(pVM, VM_FF_EMT_RENDEZVOUS | VM_FF_TM_VIRTUAL_SYNC)
            && !VMCPU_FF_IS_ANY_SET(pVCpu, VMCPU_FF_HM_TO_R3_MASK))
        || (   fStepping /* Optimized for the non-stepping case, so a bit of unnecessary work when stepping. */
            && !VMCPU_FF_IS_ANY_SET(pVCpu, VMCPU_FF_HM_TO_R3_MASK & ~(VMCPU_FF_TIMER | VMCPU_FF_PDM_CRITSECT))) )
    {
        if (!RTThreadPreemptIsPending(NIL_RTTHREAD))
        {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
            /*
             * If we are executing a nested-guest make sure that we should intercept subsequent
             * events. The one we are injecting might be part of VM-entry. This is mainly to keep
             * the VM-exit instruction emulation happy.
             */
            if (pVmxTransient->fIsNestedGuest)
                CPUMSetGuestVmxInterceptEvents(&pVCpu->cpum.GstCtx, true);
#endif

            /*
             * We've injected any pending events. This is really the point of no return (to ring-3).
             *
             * Note! The caller expects to continue with interrupts & longjmps disabled on successful
             *       returns from this function, so do -not- enable them here.
             */
            pVCpu->hm.s.Event.fPending = false;
            return VINF_SUCCESS; // 安全进入客户机执行
        }

        STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchPendingHostIrq);
        rcStrict = VINF_EM_RAW_INTERRUPT;
    }
    else
    {
        STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchHmToR3FF);
        rcStrict = VINF_EM_RAW_TO_R3;
    }

    ASMSetFlags(pVmxTransient->fEFlags); // 恢复中断标志
    VMMRZCallRing3Enable(pVCpu); // 允许 Ring3 调用

    return rcStrict;    // VINF_EM_RAW_INTERRUPT 或 VINF_EM_RAW_TO_R3
}


/**
 * Final preparations before executing guest code using hardware-assisted VMX.
 *
 * We can no longer get preempted to a different host CPU and there are no returns
 * to ring-3. We ignore any errors that may happen from this point (e.g. VMWRITE
 * failures), this function is not intended to fail sans unrecoverable hardware
 * errors.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 *
 * @remarks Called with preemption disabled.
 * @remarks No-long-jump zone!!!
 */
/*
hmR0VmxPreRunGuestCommitted 是 VirtualBox 在 VT-x 模式下执行客户机代码前的 
最终准备阶段，确保所有硬件状态、控制字段及上下文均已正确配置，为 VMLAUNCH/VMRESUME 指令的执行铺平道路。其核心任务包括：
    状态验证与切换：确保 CPU 处于非抢占、非 Ring3 调用的安全状态。
    FPU 状态加载：按需激活客户机 FPU 上下文，避免宿主 FPU 污染。
    宿主/客户机共享状态同步：更新 VMCS 中的主机控制字段及共享寄存器。
    性能优化配置：动态调整 TSC 拦截策略、预emption 定时器及 TLB 管理。
    关键资源预加载：处理 MSR 自动加载、APIC TPR 缓存等。
*/
/*
    性能影响点
    操作	            潜在开销	                优化策略
    VMCS 字段更新	    频繁 VMREAD/VMWRITE 指令	批量处理、条件更新
    TLB 刷新	        全局刷新导致缓存失效	    VPID 标签化局部刷新
    FPU 切换	        保存/恢复大寄存器集	        惰性加载、避免冗余切换
    MSR 自动加载区配置	多 MSR 操作延迟	            缓存配置、按需更新
    RDTSC/P 拦截	    频繁 VM-exit	            TSC 偏移 + preemption 定时器动态决策

 * */
static void hmR0VmxPreRunGuestCommitted(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient)
{
    //确保代码在 原子化环境 中执行，避免并发干扰。
    Assert(!VMMRZCallRing3IsEnabled(pVCpu)); // 确保不在 Ring3 调用上下文中
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));// 确认抢占已禁用
    Assert(!pVCpu->hm.s.Event.fPending);  // 无挂起事件待处理

    /*
     * Indicate start of guest execution and where poking EMT out of guest-context is recognized.
     */
    VMCPU_ASSERT_STATE(pVCpu, VMCPUSTATE_STARTED_HM);
    //从 STARTED_HM（硬件虚拟化管理就绪）转为 STARTED_EXEC（正式执行客户机代码）。
    VMCPU_SET_STATE(pVCpu, VMCPUSTATE_STARTED_EXEC);// 标记为“开始执行”状态

    PVMCC         pVM          = pVCpu->CTX_SUFF(pVM);
    PVMXVMCSINFO  pVmcsInfo    = pVmxTransient->pVmcsInfo;
    PHMPHYSCPU    pHostCpu     = hmR0GetCurrentCpu();
    RTCPUID const idCurrentCpu = pHostCpu->idCpu;

    //惰性加载：仅在客户机 FPU 未激活时切换，减少不必要的上下文保存/恢复。
    if (!CPUMIsGuestFPUStateActive(pVCpu))
    {
        STAM_PROFILE_ADV_START(&pVCpu->hm.s.StatLoadGuestFpuState, x);
        if (CPUMR0LoadGuestFPU(pVM, pVCpu) == VINF_CPUM_HOST_CR0_MODIFIED)// 加载客户机 FPU 状态. FPU 加载可能修改宿主 CR0，需触发后续主机状态重导出。
            pVCpu->hm.s.fCtxChanged |= HM_CHANGED_HOST_CONTEXT; // 若 CR0 被修改则标记
        STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatLoadGuestFpuState, x);
        STAM_COUNTER_INC(&pVCpu->hm.s.StatLoadGuestFpu);
    }

    /*
     * Re-export the host state bits as we may've been preempted (only happens when
     * thread-context hooks are used or when the VM start function changes) or if
     * the host CR0 is modified while loading the guest FPU state above.
     *
     * The 64-on-32 switcher saves the (64-bit) host state into the VMCS and if we
     * changed the switcher back to 32-bit, we *must* save the 32-bit host state here,
     * see @bugref{8432}.
     *
     * This may also happen when switching to/from a nested-guest VMCS without leaving
     * ring-0.
     */
    //宿主状态重导出
    /*
      跨 CPU 迁移：若虚拟 CPU 被调度到不同物理核，需重新配置 VMCS 主机状态。
      FPU 加载副作用：如 CR0.TS（任务切换）位被修改。
    */
    if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_HOST_CONTEXT)
    {
        /*
          导出包括 CR0/CR3/CR4、RSP、GDTR/IDTR 等关键寄存器。
          处理 64-on-32 切换器差异，确保 VMCS 中保存正确的宿主架构状态。
        */
        hmR0VmxExportHostState(pVCpu); // 更新 VMCS 中的主机状态字段
        STAM_COUNTER_INC(&pVCpu->hm.s.StatExportHostState);
    }
    Assert(!(pVCpu->hm.s.fCtxChanged & HM_CHANGED_HOST_CONTEXT));

    /*
     * Export the state shared between host and guest (FPU, debug, lazy MSRs).
     */
    //共享状态同步
    /*
      同步对象：
         调试寄存器（DR7）：若客户机/宿主调试状态变更。
         Lazy MSR：部分 MSR 的惰性加载策略，减少 VMREAD/VMWRITE 开销。
    */
    if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE)
        hmR0VmxExportSharedState(pVCpu, pVmxTransient); // 导出调试、FPU 等共享状态
    AssertMsg(!pVCpu->hm.s.fCtxChanged, ("fCtxChanged=%#RX64\n", pVCpu->hm.s.fCtxChanged));

    /*
     * Store status of the shared guest/host debug state at the time of VM-entry.
     */
    pVmxTransient->fWasGuestDebugStateActive = CPUMIsGuestDebugStateActive(pVCpu);
    pVmxTransient->fWasHyperDebugStateActive = CPUMIsHyperDebugStateActive(pVCpu);

    /*
     * Always cache the TPR-shadow if the virtual-APIC page exists, thereby skipping
     * more than one conditional check. The post-run side of our code shall determine
     * if it needs to sync. the virtual APIC TPR with the TPR-shadow.
     */
    //优化目的：避免每次访问虚拟 APIC 页面，通过缓存提升性能。
    //后续同步：在 hmR0VmxPostRunGuest 中对比并更新实际 TPR 值。
    if (pVmcsInfo->pbVirtApic)
        pVmxTransient->u8GuestTpr = pVmcsInfo->pbVirtApic[XAPIC_OFF_TPR];  // 缓存虚拟 APIC 的 TPR

    /*
     * Update the host MSRs values in the VM-exit MSR-load area.
     */
    /*
       作用：确保 VM-exit 后宿主 MSR（如 FSBASE、KERNEL_GSBASE）正确恢复。
       触发条件：首次执行或检测到 CPU 迁移。
    */
    if (!pVCpu->hmr0.s.vmx.fUpdatedHostAutoMsrs)
    {
        if (pVmcsInfo->cExitMsrLoad > 0)
            hmR0VmxUpdateAutoLoadHostMsrs(pVCpu, pVmcsInfo); // 刷新 VM-exit 时加载的宿主 MSR 值
        pVCpu->hmr0.s.vmx.fUpdatedHostAutoMsrs = true;
    }

    /*
     * Evaluate if we need to intercept guest RDTSC/P accesses. Set up the
     * VMX-preemption timer based on the next virtual sync clock deadline.
     */
    //TSC 拦截策略与preemption 定时器
    if (   !pVmxTransient->fUpdatedTscOffsettingAndPreemptTimer
        || idCurrentCpu != pVCpu->hmr0.s.idLastCpu)
    {
        /*
          动态决策：
             TSC 偏移：通过计算 TSC = Host_TSC + Offset 避免拦截 RDTSC/P。
             拦截 RDTSC/P：当无法通过偏移满足时间同步需求时（如高频率时钟）。
          preemption 定时器：设置倒计时值（基于虚拟同步时钟），触发 VM-exit 实现时间片轮转。
        */
        hmR0VmxUpdateTscOffsettingAndPreemptTimer(pVCpu, pVmxTransient, idCurrentCpu);
        pVmxTransient->fUpdatedTscOffsettingAndPreemptTimer = true;
    }

    /* Record statistics of how often we use TSC offsetting as opposed to intercepting RDTSC/P. */
    bool const fIsRdtscIntercepted = RT_BOOL(pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_RDTSC_EXIT);
    if (!fIsRdtscIntercepted)
        STAM_COUNTER_INC(&pVCpu->hm.s.StatTscOffset);
    else
        STAM_COUNTER_INC(&pVCpu->hm.s.StatTscIntercept);

    ASMAtomicUoWriteBool(&pVCpu->hm.s.fCheckedTLBFlush, true);  /* Used for TLB flushing, set this across the world switch. */
    /*
     策略选择：
       VPID（Virtual Processor ID）：若支持，仅刷新当前 VPID 关联的 TLB 条目。
       Full Invalidation：无 VPID 时执行完整 TLB 刷新。
     触发条件：客户机 CR3 变更或强制刷新标记（如跨 CPU 迁移后）。
    */
    hmR0VmxFlushTaggedTlb(pHostCpu, pVCpu, pVmcsInfo);          /* Invalidate the appropriate guest entries from the TLB. */ // 按需刷新 TLB
    Assert(idCurrentCpu == pVCpu->hmr0.s.idLastCpu);
    pVCpu->hm.s.vmx.LastError.idCurrentCpu = idCurrentCpu;      /* Record the error reporting info. with the current host CPU. */
    pVmcsInfo->idHostCpuState = idCurrentCpu;                   /* Record the CPU for which the host-state has been exported. */
    pVmcsInfo->idHostCpuExec  = idCurrentCpu;                   /* Record the CPU on which we shall execute. */

    STAM_PROFILE_ADV_STOP_START(&pVCpu->hm.s.StatEntry, &pVCpu->hm.s.StatInGC, x);

    TMNotifyStartOfExecution(pVM, pVCpu);                       /* Notify TM to resume its clocks when TSC is tied to execution,
                                                                   as we're about to start executing the guest. */

    /*
     * Load the guest TSC_AUX MSR when we are not intercepting RDTSCP.
     *
     * This is done this late as updating the TSC offsetting/preemption timer above
     * figures out if we can skip intercepting RDTSCP by calculating the number of
     * host CPU ticks till the next virtual sync deadline (for the dynamic case).
     */
    if (   (pVmcsInfo->u32ProcCtls2 & VMX_PROC_CTLS2_RDTSCP)
        && !fIsRdtscIntercepted)
    {
        vmxHCImportGuestStateEx(pVCpu, pVmcsInfo, CPUMCTX_EXTRN_TSC_AUX);

        /* NB: Because we call hmR0VmxAddAutoLoadStoreMsr with fUpdateHostMsr=true,
           it's safe even after hmR0VmxUpdateAutoLoadHostMsrs has already been done. */
        /*
           条件：启用 RDTSCP 且未拦截 RDTSC/P。
           目的：允许客户机直接读取正确的 TSC_AUX 值，无需 Hypervisor 介入。
        */
        int rc = hmR0VmxAddAutoLoadStoreMsr(pVCpu, pVmxTransient, MSR_K8_TSC_AUX, CPUMGetGuestTscAux(pVCpu),
                                            true /* fSetReadWrite */, true /* fUpdateHostMsr */);// 动态添加 TSC_AUX 到自动加载区
        AssertRC(rc);
        Assert(!pVmxTransient->fRemoveTscAuxMsr);
        pVmxTransient->fRemoveTscAuxMsr = true;
    }

    /*
     在调试版本中捕获以下问题：
        无效的客户机控制寄存器配置。
        MSR 自动加载区未正确初始化。
        宿主 EFER MSR 被意外修改。
    */
#ifdef VBOX_STRICT
    Assert(pVCpu->hmr0.s.vmx.fUpdatedHostAutoMsrs);
    hmR0VmxCheckAutoLoadStoreMsrs(pVCpu, pVmcsInfo, pVmxTransient->fIsNestedGuest);// 检查自动加载 MSR 配置
    hmR0VmxCheckHostEferMsr(pVmcsInfo);
    AssertRC(vmxHCCheckCachedVmcsCtls(pVCpu, pVmcsInfo, pVmxTransient->fIsNestedGuest));
#endif

#ifdef HMVMX_ALWAYS_CHECK_GUEST_STATE
    /** @todo r=ramshankar: We can now probably use iemVmxVmentryCheckGuestState here.
     *        Add a PVMXMSRS parameter to it, so that IEM can look at the host MSRs,
     *        see @bugref{9180#c54}. */
    uint32_t const uInvalidReason = hmR0VmxCheckGuestState(pVCpu, pVmcsInfo);// 验证客户机状态合法性
    if (uInvalidReason != VMX_IGS_REASON_NOT_FOUND)
        Log4(("hmR0VmxCheckGuestState returned %#x\n", uInvalidReason));
#endif
}


/**
 * First C routine invoked after running guest code using hardware-assisted VMX.
 *
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pVmxTransient   The VMX-transient structure.
 * @param   rcVMRun         Return code of VMLAUNCH/VMRESUME.
 *
 * @remarks Called with interrupts disabled, and returns with interrupts enabled!
 *
 * @remarks No-long-jump zone!!! This function will however re-enable longjmps
 *          unconditionally when it is safe to do so.
 */
/*
hmR0VmxPostRunGuest 负责处理客户机代码通过 VMLAUNCH/VMRESUME 执行后的 VM-exit 场景，主要完成以下任务：
  状态清理：重置全局标记、恢复主机状态。
  性能统计：记录 Guest 执行时间。
  VM-exit 处理：分析退出原因并记录日志。
  上下文同步：同步 Guest 寄存器状态（如 RFLAGS、APIC TPR）。
  错误处理：检测 VM-entry 失败等异常情况。
*/
static void hmR0VmxPostRunGuest(PVMCPUCC pVCpu, PVMXTRANSIENT pVmxTransient, int rcVMRun)
{
    //为下一次 VM-entry 做准备，确保状态一致性。
    ASMAtomicUoWriteBool(&pVCpu->hm.s.fCheckedTLBFlush, false); /* See HMInvalidatePageOnAllVCpus(): used for TLB flushing. */ // 清除 TLB 刷新标记
    ASMAtomicIncU32(&pVCpu->hmr0.s.cWorldSwitchExits);          /* Initialized in vmR3CreateUVM(): used for EMT poking. */// 增加世界切换计数器
    pVCpu->hm.s.fCtxChanged            = 0;                     /* Exits/longjmps to ring-3 requires saving the guest state. */// 清除上下文变更标记
    pVmxTransient->fVmcsFieldsRead     = 0;                     /* Transient fields need to be read from the VMCS. */// 标记 VMCS 字段需重新读取
    pVmxTransient->fVectoringPF        = false;                 /* Vectoring page-fault needs to be determined later. */
    pVmxTransient->fVectoringDoublePF  = false;                 /* Vectoring double page-fault needs to be determined later. */

    PVMXVMCSINFO pVmcsInfo = pVmxTransient->pVmcsInfo;
    //当未拦截 RDTSC 指令时，需手动计算 Guest 的 TSC 值（基于主机 TSC + 偏移量）。
    if (!(pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_RDTSC_EXIT))
    {
        uint64_t uGstTsc;
        if (!pVmxTransient->fIsNestedGuest)
            uGstTsc = pVCpu->hmr0.s.uTscExit + pVmcsInfo->u64TscOffset;
        else
        {
            //若为嵌套 Guest（Nested Guest），需额外移除嵌套层的 TSC 偏移。
            uint64_t const uNstGstTsc = pVCpu->hmr0.s.uTscExit + pVmcsInfo->u64TscOffset;
            uGstTsc = CPUMRemoveNestedGuestTscOffset(pVCpu, uNstGstTsc);
        }
        TMCpuTickSetLastSeen(pVCpu, uGstTsc);                           /* Update TM with the guest TSC. */
    }

    //StatInGC: 记录 Guest 执行时间。
    //StatPreExit：开始统计退出处理时间。
    STAM_PROFILE_ADV_STOP_START(&pVCpu->hm.s.StatInGC, &pVCpu->hm.s.StatPreExit, x);
    TMNotifyEndOfExecution(pVCpu->CTX_SUFF(pVM), pVCpu, pVCpu->hmr0.s.uTscExit); /* Notify TM that the guest is no longer running. */
    VMCPU_SET_STATE(pVCpu, VMCPUSTATE_STARTED_HM);

    pVCpu->hmr0.s.vmx.fRestoreHostFlags |= VMX_RESTORE_HOST_REQUIRED;   /* Some host state messed up by VMX needs restoring. *///标记需恢复主机状态（如调试寄存器）。
    pVmcsInfo->fVmcsState = VMX_V_VMCS_LAUNCH_STATE_LAUNCHED;           /* Use VMRESUME instead of VMLAUNCH in the next run. */  // 下次使用 VMRESUME
#ifdef VBOX_STRICT
    hmR0VmxCheckHostEferMsr(pVmcsInfo);                                 /* Verify that the host EFER MSR wasn't modified. */
#endif
    Assert(!ASMIntAreEnabled());
    ASMSetFlags(pVmxTransient->fEFlags);                                /* Enable interrupts. */   // 恢复中断状态
    Assert(!VMMRZCallRing3IsEnabled(pVCpu));

#ifdef HMVMX_ALWAYS_CLEAN_TRANSIENT
    /*
     * Clean all the VMCS fields in the transient structure before reading
     * anything from the VMCS.
     */
    pVmxTransient->uExitReason            = 0;
    pVmxTransient->uExitIntErrorCode      = 0;
    pVmxTransient->uExitQual              = 0;
    pVmxTransient->uGuestLinearAddr       = 0;
    pVmxTransient->uExitIntInfo           = 0;
    pVmxTransient->cbExitInstr            = 0;
    pVmxTransient->ExitInstrInfo.u        = 0;
    pVmxTransient->uEntryIntInfo          = 0;
    pVmxTransient->uEntryXcptErrorCode    = 0;
    pVmxTransient->cbEntryInstr           = 0;
    pVmxTransient->uIdtVectoringInfo      = 0;
    pVmxTransient->uIdtVectoringErrorCode = 0;
#endif

    /*
     * Save the basic VM-exit reason and check if the VM-entry failed.
     * See Intel spec. 24.9.1 "Basic VM-exit Information".
     */
    uint32_t uExitReason;
    //从 VMCS 中获取退出原因（如外部中断、EPT 违规等）。
    int rc = VMXReadVmcs32(VMX_VMCS32_RO_EXIT_REASON, &uExitReason);
    AssertRC(rc);
    pVmxTransient->uExitReason    = VMX_EXIT_REASON_BASIC(uExitReason);
    //若 fVMEntryFailed 为真，表示 VM-entry 因无效状态或 MSR 加载失败。
    pVmxTransient->fVMEntryFailed = VMX_EXIT_REASON_HAS_ENTRY_FAILED(uExitReason);

    /*
     * Log the VM-exit before logging anything else as otherwise it might be a
     * tad confusing what happens before and after the world-switch.
     */
    HMVMX_LOG_EXIT(pVCpu, uExitReason);// 记录退出日志

    /*
     * Remove the TSC_AUX MSR from the auto-load/store MSR area and reset any MSR
     * bitmap permissions, if it was added before VM-entry.
     */
    //若之前为调试临时添加了 TSC_AUX MSR，现在需移除，避免影响非调试执行
    if (pVmxTransient->fRemoveTscAuxMsr)
    {
        hmR0VmxRemoveAutoLoadStoreMsr(pVCpu, pVmxTransient, MSR_K8_TSC_AUX);
        pVmxTransient->fRemoveTscAuxMsr = false;
    }

    /*
     * Check if VMLAUNCH/VMRESUME succeeded.
     * If this failed, we cause a guru meditation and cease further execution.
     */
    if (RT_LIKELY(rcVMRun == VINF_SUCCESS))
    {
        /*
         * Update the VM-exit history array here even if the VM-entry failed due to:
         *   - Invalid guest state.
         *   - MSR loading.
         *   - Machine-check event.
         *
         * In any of the above cases we will still have a "valid" VM-exit reason
         * despite @a fVMEntryFailed being false.
         *
         * See Intel spec. 26.7 "VM-Entry failures during or after loading guest state".
         *
         * Note! We don't have CS or RIP at this point.  Will probably address that later
         *       by amending the history entry added here.
         */
        EMHistoryAddExit(pVCpu, EMEXIT_MAKE_FT(EMEXIT_F_KIND_VMX, pVmxTransient->uExitReason & EMEXIT_F_TYPE_MASK),
                         UINT64_MAX, pVCpu->hmr0.s.uTscExit);

        if (RT_LIKELY(!pVmxTransient->fVMEntryFailed))
        {
            VMMRZCallRing3Enable(pVCpu);
            Assert(!VMCPU_FF_IS_SET(pVCpu, VMCPU_FF_HM_UPDATE_CR3));

#ifdef HMVMX_ALWAYS_SAVE_RO_GUEST_STATE
            vmxHCReadAllRoFieldsVmcs(pVCpu, pVmxTransient);
#endif

            /*
             * Always import the guest-interruptibility state as we need it while evaluating
             * injecting events on re-entry.  We could in *theory* postpone reading it for
             * exits that does not involve instruction emulation, but since most exits are
             * for instruction emulation (exceptions being external interrupts, shadow
             * paging building page faults and EPT violations, and interrupt window stuff)
             * this is a reasonable simplification.
             *
             * We don't import CR0 (when unrestricted guest execution is unavailable) despite
             * checking for real-mode while exporting the state because all bits that cause
             * mode changes wrt CR0 are intercepted.
             *
             * Note! This mask _must_ match the default value for the default a_fDonePostExit
             *       value for the vmxHCImportGuestState template!
             */
            /** @todo r=bird: consider dropping the INHIBIT_XXX and fetch the state
             * explicitly in the exit handlers and injection function.  That way we have
             * fewer clusters of vmread spread around the code, because the EM history
             * executor won't execute very many non-exiting instructions before stopping. */
            //Guest 状态同步
            //中断/NMI 屏蔽状态：确保后续事件注入正确。
            rc = vmxHCImportGuestState<  CPUMCTX_EXTRN_INHIBIT_INT
                                       | CPUMCTX_EXTRN_INHIBIT_NMI
#if defined(HMVMX_ALWAYS_SYNC_FULL_GUEST_STATE) || defined(HMVMX_ALWAYS_SAVE_FULL_GUEST_STATE)
                                       | HMVMX_CPUMCTX_EXTRN_ALL
                                       //RFLAGS：部分配置下强制同步（如 HMVMX_ALWAYS_SAVE_GUEST_RFLAGS）。
#elif defined(HMVMX_ALWAYS_SAVE_GUEST_RFLAGS)
                                       | CPUMCTX_EXTRN_RFLAGS
#endif
                                       , 0 /*a_fDoneLocal*/, 0 /*a_fDonePostExit*/>(pVCpu, pVmcsInfo, __FUNCTION__);
            AssertRC(rc);

            /*
             * Sync the TPR shadow with our APIC state.
             */
            // APIC TPR 同步
            if (   !pVmxTransient->fIsNestedGuest
                && (pVmcsInfo->u32ProcCtls & VMX_PROC_CTLS_USE_TPR_SHADOW))
            {
                Assert(pVmcsInfo->pbVirtApic);
                if (pVmxTransient->u8GuestTpr != pVmcsInfo->pbVirtApic[XAPIC_OFF_TPR])
                {
                    //当使用 TPR 影子机制时，需确保 Guest 的 TPR（任务优先级）与虚拟 APIC 一致。
                    rc = APICSetTpr(pVCpu, pVmcsInfo->pbVirtApic[XAPIC_OFF_TPR]);// 同步虚拟 APIC 的 TPR
                    AssertRC(rc);
                    ASMAtomicUoOrU64(&pVCpu->hm.s.fCtxChanged, HM_CHANGED_GUEST_APIC_TPR);
                }
            }

            Assert(VMMRZCallRing3IsEnabled(pVCpu));
            Assert(   pVmxTransient->fWasGuestDebugStateActive == false
                   || pVmxTransient->fWasHyperDebugStateActive == false);
            return;
        }
    }
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    else if (pVmxTransient->fIsNestedGuest)
        AssertMsgFailed(("VMLAUNCH/VMRESUME failed but shouldn't happen when VMLAUNCH/VMRESUME was emulated in IEM!\n"));
#endif
    else
        Log4Func(("VM-entry failure: rcVMRun=%Rrc fVMEntryFailed=%RTbool\n", rcVMRun, pVmxTransient->fVMEntryFailed));

    VMMRZCallRing3Enable(pVCpu);
}


/**
 * Runs the guest code using hardware-assisted VMX the normal way.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pcLoops     Pointer to the number of executed loops.
 */
/*
  非嵌套虚拟化场景下的Guest OS代码执行
  完整的VMEntry/VMExit生命周期管理
  性能关键路径优化（通过Normal后缀区别于调试路径）
*/
static VBOXSTRICTRC hmR0VmxRunGuestCodeNormal(PVMCPUCC pVCpu, uint32_t *pcLoops)
{
    uint32_t const cMaxResumeLoops = pVCpu->CTX_SUFF(pVM)->hmr0.s.cMaxResumeLoops;
    Assert(pcLoops);
    Assert(*pcLoops <= cMaxResumeLoops);
    Assert(!CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx));

#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
    /*
     * Switch to the guest VMCS as we may have transitioned from executing the nested-guest
     * without leaving ring-0. Otherwise, if we came from ring-3 we would have loaded the
     * guest VMCS while entering the VMX ring-0 session.
     */
    if (pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs)
    {
        int rc = vmxHCSwitchToGstOrNstGstVmcs(pVCpu, false /* fSwitchToNstGstVmcs */);
        if (RT_SUCCESS(rc))
        { /* likely */ }
        else
        {
            LogRelFunc(("Failed to switch to the guest VMCS. rc=%Rrc\n", rc));
            return rc;
        }
    }
#endif

    VMXTRANSIENT VmxTransient;
    RT_ZERO(VmxTransient);
    //获取当前VMCS
    VmxTransient.pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);
    //嵌套虚拟化切换检查
    Assert(!pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs);

    /* Paranoia. */
    //断言确保VMCS一致性：
    Assert(VmxTransient.pVmcsInfo == &pVCpu->hmr0.s.vmx.VmcsInfo);

    VBOXSTRICTRC rcStrict = VERR_INTERNAL_ERROR_5;
    for (;;)
    {
        Assert(!HMR0SuspendPending());
        //保证CPU处于安全状态
        HMVMX_ASSERT_CPU_SAFE(pVCpu);
        //整体执行耗时
        STAM_PROFILE_ADV_START(&pVCpu->hm.s.StatEntry, x);

        /*
         * Preparatory work for running nested-guest code, this may force us to
         * return to ring-3.
         *
         * Warning! This bugger disables interrupts on VINF_SUCCESS!
         */
        //  准备阶段
        rcStrict = hmR0VmxPreRunGuest(pVCpu, &VmxTransient, false /* fStepping */);
        if (rcStrict != VINF_SUCCESS)
            break;

        //  关键执行区（中断禁用）
        /* Interrupts are disabled at this point! */
        hmR0VmxPreRunGuestCommitted(pVCpu, &VmxTransient);
        int rcRun = hmR0VmxRunGuest(pVCpu, &VmxTransient); // 实际执行VMLAUNCH/VMRESUME
        hmR0VmxPostRunGuest(pVCpu, &VmxTransient, rcRun);
        /* Interrupts are re-enabled at this point! */

        /*
         * Check for errors with running the VM (VMLAUNCH/VMRESUME).
         */
        // 错误处理
        if (RT_SUCCESS(rcRun))
        { /* very likely */ }
        else
        {
            //VMExit前处理时间
            STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatPreExit, x);
            hmR0VmxReportWorldSwitchError(pVCpu, rcRun, &VmxTransient);
            return rcRun;
        }

        /*
         * Profile the VM-exit.
         */
        AssertMsg(VmxTransient.uExitReason <= VMX_EXIT_MAX, ("%#x\n", VmxTransient.uExitReason));
        STAM_COUNTER_INC(&pVCpu->hm.s.StatExitAll);
        //各Exit原因出现次数
        STAM_COUNTER_INC(&pVCpu->hm.s.aStatExitReason[VmxTransient.uExitReason & MASK_EXITREASON_STAT]);
        //Exit原因处理时间
        STAM_PROFILE_ADV_STOP_START(&pVCpu->hm.s.StatPreExit, &pVCpu->hm.s.StatExitHandling, x);
        HMVMX_START_EXIT_DISPATCH_PROF();

        VBOXVMM_R0_HMVMX_VMEXIT_NOCTX(pVCpu, &pVCpu->cpum.GstCtx, VmxTransient.uExitReason);

        /*
         * Handle the VM-exit.
         */
        // 通过函数表或统一处理
#ifdef HMVMX_USE_FUNCTION_TABLE
        rcStrict = g_aVMExitHandlers[VmxTransient.uExitReason].pfn(pVCpu, &VmxTransient);
#else
        //  VMExit处理
        rcStrict = hmR0VmxHandleExit(pVCpu, &VmxTransient);
#endif
        // 性能统计
        STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatExitHandling, x);
        if (rcStrict == VINF_SUCCESS)
        {
            //  循环控制
            if (++(*pcLoops) <= cMaxResumeLoops)
                continue;
            STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchMaxResumeLoops);
            rcStrict = VINF_EM_RAW_INTERRUPT;
        }
        break;
    }

    STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatEntry, x);
    return rcStrict;
}


#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
/**
 * Runs the nested-guest code using hardware-assisted VMX.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pcLoops     Pointer to the number of executed loops.
 *
 * @sa      hmR0VmxRunGuestCodeNormal.
 */
static VBOXSTRICTRC hmR0VmxRunGuestCodeNested(PVMCPUCC pVCpu, uint32_t *pcLoops)
{
    uint32_t const cMaxResumeLoops = pVCpu->CTX_SUFF(pVM)->hmr0.s.cMaxResumeLoops;
    Assert(pcLoops);
    Assert(*pcLoops <= cMaxResumeLoops);
    Assert(CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx));

    /*
     * Switch to the nested-guest VMCS as we may have transitioned from executing the
     * guest without leaving ring-0. Otherwise, if we came from ring-3 we would have
     * loaded the nested-guest VMCS while entering the VMX ring-0 session.
     */
    if (!pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs)
    {
        int rc = vmxHCSwitchToGstOrNstGstVmcs(pVCpu, true /* fSwitchToNstGstVmcs */);
        if (RT_SUCCESS(rc))
        { /* likely */ }
        else
        {
            LogRelFunc(("Failed to switch to the nested-guest VMCS. rc=%Rrc\n", rc));
            return rc;
        }
    }

    VMXTRANSIENT VmxTransient;
    RT_ZERO(VmxTransient);
    VmxTransient.pVmcsInfo      = hmGetVmxActiveVmcsInfo(pVCpu);
    VmxTransient.fIsNestedGuest = true;
    Assert(pVCpu->hmr0.s.vmx.fSwitchedToNstGstVmcs);

    /* Paranoia. */
    Assert(VmxTransient.pVmcsInfo == &pVCpu->hmr0.s.vmx.VmcsInfoNstGst);

    /* Setup pointer so PGM/IEM can query VM-exit auxiliary info on demand in ring-0. */
    pVCpu->hmr0.s.vmx.pVmxTransient = &VmxTransient;

    VBOXSTRICTRC rcStrict = VERR_INTERNAL_ERROR_5;
    for (;;)
    {
        Assert(!HMR0SuspendPending());
        HMVMX_ASSERT_CPU_SAFE(pVCpu);
        STAM_PROFILE_ADV_START(&pVCpu->hm.s.StatEntry, x);

        /*
         * Preparatory work for running guest code, this may force us to
         * return to ring-3.
         *
         * Warning! This bugger disables interrupts on VINF_SUCCESS!
         */
        rcStrict = hmR0VmxPreRunGuest(pVCpu, &VmxTransient, false /* fStepping */);
        if (rcStrict != VINF_SUCCESS)
            break;

        /* Interrupts are disabled at this point! */
        hmR0VmxPreRunGuestCommitted(pVCpu, &VmxTransient);
        int rcRun = hmR0VmxRunGuest(pVCpu, &VmxTransient);
        hmR0VmxPostRunGuest(pVCpu, &VmxTransient, rcRun);
        /* Interrupts are re-enabled at this point! */

        /*
         * Check for errors with running the VM (VMLAUNCH/VMRESUME).
         */
        if (RT_SUCCESS(rcRun))
        { /* very likely */ }
        else
        {
            STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatPreExit, x);
            hmR0VmxReportWorldSwitchError(pVCpu, rcRun, &VmxTransient);
            rcStrict = rcRun;
            break;
        }

        /*
         * Profile the VM-exit.
         */
        AssertMsg(VmxTransient.uExitReason <= VMX_EXIT_MAX, ("%#x\n", VmxTransient.uExitReason));
        STAM_COUNTER_INC(&pVCpu->hm.s.StatNestedExitAll);
        STAM_COUNTER_INC(&pVCpu->hm.s.aStatNestedExitReason[VmxTransient.uExitReason & MASK_EXITREASON_STAT]);
        STAM_PROFILE_ADV_STOP_START(&pVCpu->hm.s.StatPreExit, &pVCpu->hm.s.StatExitHandling, x);
        HMVMX_START_EXIT_DISPATCH_PROF();

        VBOXVMM_R0_HMVMX_VMEXIT_NOCTX(pVCpu, &pVCpu->cpum.GstCtx, VmxTransient.uExitReason);

        /*
         * Handle the VM-exit.
         */
        rcStrict = vmxHCHandleExitNested(pVCpu, &VmxTransient);
        STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatExitHandling, x);
        if (rcStrict == VINF_SUCCESS)
        {
            if (!CPUMIsGuestInVmxNonRootMode(&pVCpu->cpum.GstCtx))
            {
                STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchNstGstVmexit);
                rcStrict = VINF_VMX_VMEXIT;
            }
            else
            {
                if (++(*pcLoops) <= cMaxResumeLoops)
                    continue;
                STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchMaxResumeLoops);
                rcStrict = VINF_EM_RAW_INTERRUPT;
            }
        }
        else
            Assert(rcStrict != VINF_VMX_VMEXIT);
        break;
    }

    /* Ensure VM-exit auxiliary info. is no longer available. */
    pVCpu->hmr0.s.vmx.pVmxTransient = NULL;

    STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatEntry, x);
    return rcStrict;
}
#endif /* VBOX_WITH_NESTED_HWVIRT_VMX */


/** @name Execution loop for single stepping, DBGF events and expensive Dtrace
 *  probes.
 *
 * The following few functions and associated structure contains the bloat
 * necessary for providing detailed debug events and dtrace probes as well as
 * reliable host side single stepping.  This works on the principle of
 * "subclassing" the normal execution loop and workers.  We replace the loop
 * method completely and override selected helpers to add necessary adjustments
 * to their core operation.
 *
 * The goal is to keep the "parent" code lean and mean, so as not to sacrifice
 * any performance for debug and analysis features.
 *
 * @{
 */

/**
 * Single steps guest code using hardware-assisted VMX.
 *
 * This is -not- the same as the guest single-stepping itself (say using EFLAGS.TF)
 * but single-stepping through the hypervisor debugger.
 *
 * @returns Strict VBox status code (i.e. informational status codes too).
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   pcLoops     Pointer to the number of executed loops.
 *
 * @note    Mostly the same as hmR0VmxRunGuestCodeNormal().
 */
/*
  1.单步调试支持（Single-stepping）
  2.调试相关 VM-exit 处理
  3.控制循环执行（防止无限循环）
  4.状态保存与恢复（确保调试不影响正常执行流）
*/
/*
Trap Flag（TF）
  通过设置 EFLAGS.TF 使 CPU 在执行一条指令后触发 #DB 异常（实现单步）。
VMCS 控制字段
  动态调整 VMCS 中的 Exception Bitmap 和 Debug Controls 以捕获调试事件。
DTrace 集成
  通过 VBOXVMM_GET_SETTINGS_SEQ_NO 检测动态跟踪配置变更。
*/
static VBOXSTRICTRC hmR0VmxRunGuestCodeDebug(PVMCPUCC pVCpu, uint32_t *pcLoops)
{
    uint32_t const cMaxResumeLoops = pVCpu->CTX_SUFF(pVM)->hmr0.s.cMaxResumeLoops;
    Assert(pcLoops);
    Assert(*pcLoops <= cMaxResumeLoops);

    VMXTRANSIENT VmxTransient;
    RT_ZERO(VmxTransient);
    //临时存储 VMX 执行状态的上下文结构体（如 VMCS 指针、退出原因等）
    VmxTransient.pVmcsInfo = hmGetVmxActiveVmcsInfo(pVCpu);

    /* Set HMCPU indicators.  */
    //fSingleInstruction标记是否启用单步执行（通过调试器或内部标志触发）。
    bool const fSavedSingleInstruction = pVCpu->hm.s.fSingleInstruction;
    pVCpu->hm.s.fSingleInstruction     = pVCpu->hm.s.fSingleInstruction || DBGFIsStepping(pVCpu);
    pVCpu->hmr0.s.fDebugWantRdTscExit    = false;
    //fUsingDebugLoop：进入调试专用循环模式。
    pVCpu->hmr0.s.fUsingDebugLoop        = true;

    /* State we keep to help modify and later restore the VMCS fields we alter, and for detecting steps.  */
    VMXRUNDBGSTATE DbgState;
    vmxHCRunDebugStateInit(pVCpu, &VmxTransient, &DbgState);
    vmxHCPreRunGuestDebugStateUpdate(pVCpu, &VmxTransient, &DbgState);

    /*
     * The loop.
     */
    VBOXSTRICTRC rcStrict  = VERR_INTERNAL_ERROR_5;
    for (;;)
    {
        // 检查中断挂起、确保 CPU 安全状态
        Assert(!HMR0SuspendPending());
        HMVMX_ASSERT_CPU_SAFE(pVCpu);

        STAM_PROFILE_ADV_START(&pVCpu->hm.s.StatEntry, x);
        bool fStepping = pVCpu->hm.s.fSingleInstruction;

        // 应用调试相关的 VMCS 控制字段
        /* Set up VM-execution controls the next two can respond to. */
        vmxHCPreRunGuestDebugStateApply(pVCpu, &VmxTransient, &DbgState);

        /*
         * Preparatory work for running guest code, this may force us to
         * return to ring-3.
         *
         * Warning! This bugger disables interrupts on VINF_SUCCESS!
         */
        // 准备执行客户机代码（可能触发环3切换）
        rcStrict = hmR0VmxPreRunGuest(pVCpu, &VmxTransient, fStepping);
        if (rcStrict != VINF_SUCCESS)
            break;

        /* Interrupts are disabled at this point! */
        hmR0VmxPreRunGuestCommitted(pVCpu, &VmxTransient);

        /* Override any obnoxious code in the above two calls. */
        vmxHCPreRunGuestDebugStateApply(pVCpu, &VmxTransient, &DbgState);

        /*
         * Finally execute the guest.
         */
        // 提交状态并执行客户机（VMLAUNCH/VMRESUME）
        // 通过 VT-x 指令（如 VMRESUME）实际运行客户机代码。
        int rcRun = hmR0VmxRunGuest(pVCpu, &VmxTransient);

        hmR0VmxPostRunGuest(pVCpu, &VmxTransient, rcRun);
        /* Interrupts are re-enabled at this point! */

        /* Check for errors with running the VM (VMLAUNCH/VMRESUME). */
        if (RT_SUCCESS(rcRun))
        { /* very likely */ }
        else
        {
            STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatPreExit, x);
            hmR0VmxReportWorldSwitchError(pVCpu, rcRun, &VmxTransient);
            return rcRun;
        }

        /* Profile the VM-exit. */
        AssertMsg(VmxTransient.uExitReason <= VMX_EXIT_MAX, ("%#x\n", VmxTransient.uExitReason));
        STAM_COUNTER_INC(&pVCpu->hm.s.StatDebugExitAll);
        STAM_COUNTER_INC(&pVCpu->hm.s.aStatExitReason[VmxTransient.uExitReason & MASK_EXITREASON_STAT]);
        STAM_PROFILE_ADV_STOP_START(&pVCpu->hm.s.StatPreExit, &pVCpu->hm.s.StatExitHandling, x);
        HMVMX_START_EXIT_DISPATCH_PROF();

        VBOXVMM_R0_HMVMX_VMEXIT_NOCTX(pVCpu, &pVCpu->cpum.GstCtx, VmxTransient.uExitReason);

        /*
         * Handle the VM-exit - we quit earlier on certain VM-exits, see hmR0VmxHandleExitDebug().
         */
        // 处理 VM-exit
        rcStrict = vmxHCRunDebugHandleExit(pVCpu, &VmxTransient, &DbgState);
        STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatExitHandling, x);
        if (rcStrict != VINF_SUCCESS)
            break;
        // 防止无限循环
        //cMaxResumeLoops 最大循环次数限制，避免因频繁 VM-exit 导致死循环。
        if (++(*pcLoops) > cMaxResumeLoops)
        {
            STAM_COUNTER_INC(&pVCpu->hm.s.StatSwitchMaxResumeLoops);
            rcStrict = VINF_EM_RAW_INTERRUPT;
            break;
        }

        /*
         * Stepping: Did the RIP change, if so, consider it a single step.
         * Otherwise, make sure one of the TFs gets set.
         */
        if (fStepping)
        {
            int rc = vmxHCImportGuestStateEx(pVCpu, VmxTransient.pVmcsInfo, CPUMCTX_EXTRN_CS | CPUMCTX_EXTRN_RIP);
            AssertRC(rc);
            //比较 RIP/CS 寄存器是否变化，若变化则判定为单步完成。
            if (   pVCpu->cpum.GstCtx.rip    != DbgState.uRipStart
                || pVCpu->cpum.GstCtx.cs.Sel != DbgState.uCsStart)
            {
                rcStrict = VINF_EM_DBG_STEPPED;
                break;
            }
            ASMAtomicUoOrU64(&pVCpu->hm.s.fCtxChanged, HM_CHANGED_GUEST_DR7);
        }

        /*
         * Update when dtrace settings changes (DBGF kicks us, so no need to check).
         */
        if (VBOXVMM_GET_SETTINGS_SEQ_NO() != DbgState.uDtraceSettingsSeqNo)
            vmxHCPreRunGuestDebugStateUpdate(pVCpu, &VmxTransient, &DbgState);

        /* Restore all controls applied by hmR0VmxPreRunGuestDebugStateApply above. */
        rcStrict = vmxHCRunDebugStateRevert(pVCpu, &VmxTransient, &DbgState, rcStrict);
        Assert(rcStrict == VINF_SUCCESS);
    }

    /*
     * Clear the X86_EFL_TF if necessary.
     */
    //恢复 CPU 标志位（如 TF）并重置调试状态。
    if (pVCpu->hmr0.s.fClearTrapFlag)
    {
        int rc = vmxHCImportGuestStateEx(pVCpu, VmxTransient.pVmcsInfo, CPUMCTX_EXTRN_RFLAGS);
        AssertRC(rc);
        pVCpu->hmr0.s.fClearTrapFlag = false;
        pVCpu->cpum.GstCtx.eflags.Bits.u1TF = 0;
    }
    /** @todo there seems to be issues with the resume flag when the monitor trap
     *        flag is pending without being used. Seen early in bios init when
     *        accessing APIC page in protected mode. */

/** @todo we need to do hmR0VmxRunDebugStateRevert here too, in case we broke
 *        out of the above loop. */

    /* Restore HMCPU indicators. */
    pVCpu->hmr0.s.fUsingDebugLoop     = false;
    pVCpu->hmr0.s.fDebugWantRdTscExit = false;
    pVCpu->hm.s.fSingleInstruction  = fSavedSingleInstruction;

    STAM_PROFILE_ADV_STOP(&pVCpu->hm.s.StatEntry, x);
    return rcStrict;
}

/** @} */


/**
 * Checks if any expensive dtrace probes are enabled and we should go to the
 * debug loop.
 *
 * @returns true if we should use debug loop, false if not.
 */
//该函数用于快速检测是否启用了任何昂贵的 VMX 探针（probes），
//这些探针会影响虚拟机的性能。如果启用了任何探针，VirtualBox 可能需要切换到调试模式
//（hmR0VmxRunGuestCodeDebug）而不是快速路径模式（hmR0VmxRunGuestCodeNormal）。
//避免逐个条件判断：
//  直接对多个探针标志进行位或（OR）运算，而不是逐个检查，以减少分支预测开销。
//缓存友好性：
//  由于探针标志存储在相邻的内存位置（数组或结构体），一次性读取多个标志可以减少缓存行（cache line）访问次数。
/*
 *. VMExit事件分类‌
这些事件可分为以下几类：

特权指令执行
  指令	           作用
  LTR	           加载任务寄存器
  GETSEC	       进入安全模式（Intel TXT）
  RSM	           从系统管理模式（SMM）返回
  RDRAND/RDSEED	   读取硬件随机数
  XSAVES/XRSTORS   保存/恢复扩展处理器状态（用于AVX等指令集）

  VMX相关操作
  指令	             作用
  VMCLEAR	         初始化VMCS区域
  VMLAUNCH/VMRESUME	 启动/恢复虚拟机
  VMPTRLD/VMPTRST	 加载/存储VMCS指针
  VMREAD/VMWRITE	 读写VMCS字段
  VMXOFF/VMXON	     关闭/开启VMX模式
  VMFUNC	         VMX功能调用（用于嵌套虚拟化）

  内存虚拟化事件
  事件	            触发条件
  INVEPT	        扩展页表（EPT）失效操作
  INVVPID/INVPCID	虚拟处理器ID/TLB失效操作
  EPT_VIOLATION	    EPT页访问违规（如权限不足）
  EPT_MISCONFIG	    EPT配置错误
  VAPIC_ACCESS/VAPIC_WRITE	虚拟APIC（高级可编程中断控制器）访问/写
 * */
static bool hmR0VmxAnyExpensiveProbesEnabled(void)
{
    /* It's probably faster to OR the raw 32-bit counter variables together.
       Since the variables are in an array and the probes are next to one
       another (more or less), we have good locality.  So, better read
       eight-nine cache lines ever time and only have one conditional, than
       128+ conditionals, right? */
    return (  VBOXVMM_R0_HMVMX_VMEXIT_ENABLED_RAW() /* expensive too due to context */// VMExit 事件
            | VBOXVMM_XCPT_DE_ENABLED_RAW()// #DE（除零异常）
            | VBOXVMM_XCPT_DB_ENABLED_RAW()// #DB（调试异常）
            | VBOXVMM_XCPT_BP_ENABLED_RAW()// #BP（断点异常）
            | VBOXVMM_XCPT_OF_ENABLED_RAW()// #OF（溢出异常）
            | VBOXVMM_XCPT_BR_ENABLED_RAW()
            | VBOXVMM_XCPT_UD_ENABLED_RAW()//#UD（无效指令）
            | VBOXVMM_XCPT_NM_ENABLED_RAW()//#NM（设备不可用）
            | VBOXVMM_XCPT_DF_ENABLED_RAW()// #DF（双重错误）
            | VBOXVMM_XCPT_TS_ENABLED_RAW()// #TS（无效 TSS）
            | VBOXVMM_XCPT_NP_ENABLED_RAW()// #NP（段不存在）
            | VBOXVMM_XCPT_SS_ENABLED_RAW()// #SS（栈错误）
            | VBOXVMM_XCPT_GP_ENABLED_RAW() // #GP（一般保护错误）
            | VBOXVMM_XCPT_PF_ENABLED_RAW()// #PF（页错误）
            | VBOXVMM_XCPT_MF_ENABLED_RAW()// #MF（浮点错误）
            | VBOXVMM_XCPT_AC_ENABLED_RAW()// #AC（对齐检查）
            | VBOXVMM_XCPT_XF_ENABLED_RAW()// #XF（SIMD 浮点异常）
            | VBOXVMM_XCPT_VE_ENABLED_RAW() // #VE（虚拟化异常）
            | VBOXVMM_XCPT_SX_ENABLED_RAW() // #SX（安全异常）
            | VBOXVMM_INT_SOFTWARE_ENABLED_RAW()
            | VBOXVMM_INT_HARDWARE_ENABLED_RAW()
           ) != 0
           //特权指令
        || (  VBOXVMM_INSTR_HALT_ENABLED_RAW() // HLT（停机指令）
            | VBOXVMM_INSTR_MWAIT_ENABLED_RAW()// MWAIT（等待指令）
            | VBOXVMM_INSTR_MONITOR_ENABLED_RAW() // MONITOR（监控指令）
            | VBOXVMM_INSTR_CPUID_ENABLED_RAW() // CPUID（获取 CPU 信息）
            | VBOXVMM_INSTR_INVD_ENABLED_RAW()// INVD（缓存失效）
            | VBOXVMM_INSTR_WBINVD_ENABLED_RAW()// WBINVD（写回缓存失效）
            | VBOXVMM_INSTR_INVLPG_ENABLED_RAW()// INVLPG（TLB 失效）
            | VBOXVMM_INSTR_RDTSC_ENABLED_RAW()// RDTSC（读取时间戳）
            | VBOXVMM_INSTR_RDTSCP_ENABLED_RAW() // RDTSCP（带处理器 ID 的时间戳）
            | VBOXVMM_INSTR_RDPMC_ENABLED_RAW()// RDPMC（读取性能计数器）
            | VBOXVMM_INSTR_RDMSR_ENABLED_RAW()// RDMSR（读取 MSR）
            | VBOXVMM_INSTR_WRMSR_ENABLED_RAW() // WRMSR（写入 MSR）
            | VBOXVMM_INSTR_CRX_READ_ENABLED_RAW()// 读取控制寄存器（CR0-CR4）
            | VBOXVMM_INSTR_CRX_WRITE_ENABLED_RAW() // 写入控制寄存器
            | VBOXVMM_INSTR_DRX_READ_ENABLED_RAW()// 读取调试寄存器（DR0-DR7）
            | VBOXVMM_INSTR_DRX_WRITE_ENABLED_RAW()// 写入调试寄存器
            | VBOXVMM_INSTR_PAUSE_ENABLED_RAW()// PAUSE（暂停指令）
            | VBOXVMM_INSTR_XSETBV_ENABLED_RAW()// XSETBV（设置扩展控制寄存器）
            | VBOXVMM_INSTR_SIDT_ENABLED_RAW()// SIDT（存储 IDT）
            | VBOXVMM_INSTR_LIDT_ENABLED_RAW()// LIDT（加载 IDT）
            | VBOXVMM_INSTR_SGDT_ENABLED_RAW() // SGDT（存储 GDT）
            | VBOXVMM_INSTR_LGDT_ENABLED_RAW() // LGDT（加载 GDT）
            | VBOXVMM_INSTR_SLDT_ENABLED_RAW() // SLDT（存储 LDT）
            | VBOXVMM_INSTR_LLDT_ENABLED_RAW()// LLDT（加载 LDT）
            | VBOXVMM_INSTR_STR_ENABLED_RAW() // STR（存储任务寄存器）
            | VBOXVMM_INSTR_LTR_ENABLED_RAW()  // LTR（加载任务寄存器）
            | VBOXVMM_INSTR_GETSEC_ENABLED_RAW() // GETSEC（安全指令）
            | VBOXVMM_INSTR_RSM_ENABLED_RAW()// RSM（恢复系统管理模式）
            | VBOXVMM_INSTR_RDRAND_ENABLED_RAW() // RDRAND（随机数指令）
            | VBOXVMM_INSTR_RDSEED_ENABLED_RAW() // RDSEED（种子随机数指令）
            | VBOXVMM_INSTR_XSAVES_ENABLED_RAW() // XSAVES（保存扩展状态）
            | VBOXVMM_INSTR_XRSTORS_ENABLED_RAW()// XRSTORS（恢复扩展状态）
            | VBOXVMM_INSTR_VMM_CALL_ENABLED_RAW() // VMMCALL（虚拟机管理调用）
            | VBOXVMM_INSTR_VMX_VMCLEAR_ENABLED_RAW()// VMCLEAR（VMX 清理）
            | VBOXVMM_INSTR_VMX_VMLAUNCH_ENABLED_RAW() // VMLAUNCH（VMX 启动）
            | VBOXVMM_INSTR_VMX_VMPTRLD_ENABLED_RAW() // VMPTRLD（加载 VMCS 指针）
            | VBOXVMM_INSTR_VMX_VMPTRST_ENABLED_RAW()// VMPTRST（存储 VMCS 指针）
            | VBOXVMM_INSTR_VMX_VMREAD_ENABLED_RAW()  // VMREAD（读取 VMCS 字段）
            | VBOXVMM_INSTR_VMX_VMRESUME_ENABLED_RAW() // VMRESUME（恢复 VMX 状态）
            | VBOXVMM_INSTR_VMX_VMWRITE_ENABLED_RAW()// VMWRITE（写入 VMCS 字段）
            | VBOXVMM_INSTR_VMX_VMXOFF_ENABLED_RAW() // VMXOFF（关闭 VMX）
            | VBOXVMM_INSTR_VMX_VMXON_ENABLED_RAW()// VMXON（开启 VMX）
            | VBOXVMM_INSTR_VMX_VMFUNC_ENABLED_RAW() // VMFUNC（VMX 功能调用）
            | VBOXVMM_INSTR_VMX_INVEPT_ENABLED_RAW() // INVEPT（EPT 失效）
            | VBOXVMM_INSTR_VMX_INVVPID_ENABLED_RAW()// INVVPID（VPID 失效）
            | VBOXVMM_INSTR_VMX_INVPCID_ENABLED_RAW()// INVPCID（PCID 失效）
           ) != 0
           //VMExit 事件（VM Exits）
        || (  VBOXVMM_EXIT_TASK_SWITCH_ENABLED_RAW() // 任务切换
            | VBOXVMM_EXIT_HALT_ENABLED_RAW()// HLT 退出
            | VBOXVMM_EXIT_MWAIT_ENABLED_RAW()// MWAIT 退出
            | VBOXVMM_EXIT_MONITOR_ENABLED_RAW()  // MONITOR 退出
            | VBOXVMM_EXIT_CPUID_ENABLED_RAW() // CPUID 退出
            | VBOXVMM_EXIT_INVD_ENABLED_RAW()// INVD 退出
            | VBOXVMM_EXIT_WBINVD_ENABLED_RAW()// WBINVD 退出
            | VBOXVMM_EXIT_INVLPG_ENABLED_RAW() // INVLPG 退出
            | VBOXVMM_EXIT_RDTSC_ENABLED_RAW() // RDTSC 退出
            | VBOXVMM_EXIT_RDTSCP_ENABLED_RAW() // RDTSCP 退出
            | VBOXVMM_EXIT_RDPMC_ENABLED_RAW()
            | VBOXVMM_EXIT_RDMSR_ENABLED_RAW()
            | VBOXVMM_EXIT_WRMSR_ENABLED_RAW()
            | VBOXVMM_EXIT_CRX_READ_ENABLED_RAW()// CR 读取退出
            | VBOXVMM_EXIT_CRX_WRITE_ENABLED_RAW()
            | VBOXVMM_EXIT_DRX_READ_ENABLED_RAW()
            | VBOXVMM_EXIT_DRX_WRITE_ENABLED_RAW()
            | VBOXVMM_EXIT_PAUSE_ENABLED_RAW()
            | VBOXVMM_EXIT_XSETBV_ENABLED_RAW()
            | VBOXVMM_EXIT_SIDT_ENABLED_RAW()
            | VBOXVMM_EXIT_LIDT_ENABLED_RAW()
            | VBOXVMM_EXIT_SGDT_ENABLED_RAW()
            | VBOXVMM_EXIT_LGDT_ENABLED_RAW()
            | VBOXVMM_EXIT_SLDT_ENABLED_RAW()
            | VBOXVMM_EXIT_LLDT_ENABLED_RAW()
            | VBOXVMM_EXIT_STR_ENABLED_RAW()
            | VBOXVMM_EXIT_LTR_ENABLED_RAW()
            | VBOXVMM_EXIT_GETSEC_ENABLED_RAW()
            | VBOXVMM_EXIT_RSM_ENABLED_RAW()
            | VBOXVMM_EXIT_RDRAND_ENABLED_RAW()
            | VBOXVMM_EXIT_RDSEED_ENABLED_RAW()
            | VBOXVMM_EXIT_XSAVES_ENABLED_RAW()
            | VBOXVMM_EXIT_XRSTORS_ENABLED_RAW()
            | VBOXVMM_EXIT_VMM_CALL_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMCLEAR_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMLAUNCH_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMPTRLD_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMPTRST_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMREAD_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMRESUME_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMWRITE_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMXOFF_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMXON_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VMFUNC_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_INVEPT_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_INVVPID_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_INVPCID_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_EPT_VIOLATION_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_EPT_MISCONFIG_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VAPIC_ACCESS_ENABLED_RAW()
            | VBOXVMM_EXIT_VMX_VAPIC_WRITE_ENABLED_RAW()
           ) != 0;
}


/**
 * Runs the guest using hardware-assisted VMX.
 *
 * @returns Strict VBox status code (i.e. informational status codes too).
 * @param   pVCpu   The cross context virtual CPU structure.
 */
/*
这是VirtualBox中Intel VMX硬件虚拟化的核心执行循环，负责：
  在非根模式（Non-Root Mode）下运行Guest OS代码
  处理VMExit事件
  支持嵌套虚拟化（Nested Virtualization）
  实现调试/性能优化分支
*/
VMMR0DECL(VBOXSTRICTRC) VMXR0RunGuestCode(PVMCPUCC pVCpu)
{
    AssertPtr(pVCpu);
    PCPUMCTX pCtx = &pVCpu->cpum.GstCtx;
    Assert(VMMRZCallRing3IsEnabled(pVCpu));
    Assert(!ASMAtomicUoReadU64(&pCtx->fExtrn));
    HMVMX_ASSERT_PREEMPT_SAFE(pVCpu);

    VBOXSTRICTRC rcStrict;
    uint32_t     cLoops = 0;
    for (;;)
    {
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        bool const fInNestedGuestMode = CPUMIsGuestInVmxNonRootMode(pCtx);
#else
        NOREF(pCtx);
        bool const fInNestedGuestMode = false;
#endif
        if (!fInNestedGuestMode)//!嵌套模式
        {
            if (   !pVCpu->hm.s.fUseDebugLoop
                && (!VBOXVMM_ANY_PROBES_ENABLED() || !hmR0VmxAnyExpensiveProbesEnabled())
                && !DBGFIsStepping(pVCpu)
                && !pVCpu->CTX_SUFF(pVM)->dbgf.ro.cEnabledInt3Breakpoints)//无需调试
                rcStrict = hmR0VmxRunGuestCodeNormal(pVCpu, &cLoops);// 快速路径
            else
                rcStrict = hmR0VmxRunGuestCodeDebug(pVCpu, &cLoops);// 调试路径
        }
#ifdef VBOX_WITH_NESTED_HWVIRT_VMX
        else
            rcStrict = hmR0VmxRunGuestCodeNested(pVCpu, &cLoops);// 嵌套虚拟化路径

        if (rcStrict == VINF_VMX_VMLAUNCH_VMRESUME)//需要重新进入嵌套Guest
        {
            Assert(CPUMIsGuestInVmxNonRootMode(pCtx));
            continue;
        }
        if (rcStrict == VINF_VMX_VMEXIT) //需要退出嵌套Guest
        {
            Assert(!CPUMIsGuestInVmxNonRootMode(pCtx));
            continue;
        }
#endif
        break;
    }

    int const rcLoop = VBOXSTRICTRC_VAL(rcStrict);
    switch (rcLoop)
    {
        case VERR_EM_INTERPRETER:   rcStrict = VINF_EM_RAW_EMULATE_INSTR;   break;
        case VINF_EM_RESET:         rcStrict = VINF_EM_TRIPLE_FAULT;        break;
    }

    int rc2 = hmR0VmxExitToRing3(pVCpu, rcStrict);//处理需要返回到Ring3的情况
    if (RT_FAILURE(rc2))
    {
        pVCpu->hm.s.u32HMError = (uint32_t)VBOXSTRICTRC_VAL(rcStrict);
        rcStrict = rc2;
    }
    Assert(!ASMAtomicUoReadU64(&pCtx->fExtrn));
    Assert(!VMMR0AssertionIsNotificationSet(pVCpu));
    return rcStrict;
}

