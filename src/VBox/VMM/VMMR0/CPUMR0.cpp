/* $Id$ */
/** @file
 * CPUM - Host Context Ring 0.
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
#define LOG_GROUP LOG_GROUP_CPUM
#define CPUM_WITH_NONCONST_HOST_FEATURES
#include <VBox/vmm/cpum.h>
#include <VBox/vmm/hm.h>
#include "CPUMInternal.h"
#include <VBox/vmm/vmcc.h>
#include <VBox/vmm/gvm.h>
#include <VBox/err.h>
#include <VBox/log.h>
#include <VBox/vmm/hm.h>
#include <iprt/assert.h>
#include <iprt/asm-amd64-x86.h>
#include <iprt/mem.h>
#include <iprt/x86.h>


/*********************************************************************************************************************************
*   Global Variables                                                                                                             *
*********************************************************************************************************************************/
/** Host CPU features. */
DECL_HIDDEN_DATA(CPUHOSTFEATURES)  g_CpumHostFeatures;
/** Static storage for host MSRs. */
static CPUMMSRS     g_CpumHostMsrs;

/**
 * CPUID bits to unify among all cores.
 */
static struct
{
    uint32_t uLeaf;  /**< Leaf to check. */
    uint32_t uEcx;   /**< which bits in ecx to unify between CPUs. */
    uint32_t uEdx;   /**< which bits in edx to unify between CPUs. */
}
const g_aCpuidUnifyBits[] =
{
    {
        0x00000001,
        X86_CPUID_FEATURE_ECX_CX16 | X86_CPUID_FEATURE_ECX_MONITOR,
        X86_CPUID_FEATURE_EDX_CX8
    }
};



/*********************************************************************************************************************************
*   Internal Functions                                                                                                           *
*********************************************************************************************************************************/
static int  cpumR0SaveHostDebugState(PVMCPUCC pVCpu);


/**
 * Check the CPUID features of this particular CPU and disable relevant features
 * for the guest which do not exist on this CPU.
 *
 * We have seen systems where the X86_CPUID_FEATURE_ECX_MONITOR feature flag is
 * only set on some host CPUs, see @bugref{5436}.
 *
 * @note This function might be called simultaneously on more than one CPU!
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     Leaf array.
 * @param   pvUser2     Number of leaves.
 */
static DECLCALLBACK(void) cpumR0CheckCpuid(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    PCPUMCPUIDLEAF const paLeaves = (PCPUMCPUIDLEAF)pvUser1;
    uint32_t const       cLeaves  = (uint32_t)(uintptr_t)pvUser2;
    RT_NOREF(idCpu);

    for (uint32_t i = 0; i < RT_ELEMENTS(g_aCpuidUnifyBits); i++)
    {
        PCPUMCPUIDLEAF pLeaf = cpumCpuIdGetLeafInt(paLeaves, cLeaves, g_aCpuidUnifyBits[i].uLeaf, 0);
        if (pLeaf)
        {
            uint32_t uEax, uEbx, uEcx, uEdx;
            ASMCpuIdExSlow(g_aCpuidUnifyBits[i].uLeaf, 0, 0, 0, &uEax, &uEbx, &uEcx, &uEdx);

            ASMAtomicAndU32(&pLeaf->uEcx, uEcx | ~g_aCpuidUnifyBits[i].uEcx);
            ASMAtomicAndU32(&pLeaf->uEdx, uEdx | ~g_aCpuidUnifyBits[i].uEdx);
        }
    }
}


/**
 * Does the Ring-0 CPU initialization once during module load.
 * XXX Host-CPU hot-plugging?
 */
VMMR0_INT_DECL(int) CPUMR0ModuleInit(void)
{
    /*
     * Query the hardware virtualization capabilities of the host CPU first.
     */
    uint32_t fHwCaps = 0;
    int rc = SUPR0GetVTSupport(&fHwCaps);
    AssertLogRelMsg(RT_SUCCESS(rc) || rc == VERR_UNSUPPORTED_CPU || rc == VERR_SVM_NO_SVM || rc == VERR_VMX_NO_VMX,
                    ("SUPR0GetHwvirtMsrs -> %Rrc\n", rc));
    if (RT_SUCCESS(rc))
    {
        SUPHWVIRTMSRS HwvirtMsrs;
        rc = SUPR0GetHwvirtMsrs(&HwvirtMsrs, fHwCaps, false /*fIgnored*/);
        AssertLogRelRC(rc);
        if (RT_SUCCESS(rc))
        {
            if (fHwCaps & SUPVTCAPS_VT_X)
                HMGetVmxMsrsFromHwvirtMsrs(&HwvirtMsrs, &g_CpumHostMsrs.hwvirt.vmx);
            else
                HMGetSvmMsrsFromHwvirtMsrs(&HwvirtMsrs, &g_CpumHostMsrs.hwvirt.svm);
        }
    }

    /*
     * Collect CPUID leaves.
     */
    PCPUMCPUIDLEAF  paLeaves;
    uint32_t        cLeaves;
    rc = CPUMCpuIdCollectLeavesX86(&paLeaves, &cLeaves);
    AssertLogRelRCReturn(rc, rc);

    /*
     * Unify/cross check some CPUID feature bits on all available CPU cores
     * and threads.  We've seen CPUs where the monitor support differed.
     */
    RTMpOnAll(cpumR0CheckCpuid, paLeaves, (void *)(uintptr_t)cLeaves);

    /*
     * Populate the host CPU feature global variable.
     */
    rc = cpumCpuIdExplodeFeaturesX86(paLeaves, cLeaves, &g_CpumHostMsrs, &g_CpumHostFeatures.s);
    RTMemFree(paLeaves);
    AssertLogRelRCReturn(rc, rc);

    /*
     * Get MSR_IA32_ARCH_CAPABILITIES and expand it into the host feature structure.
     */
    if (ASMHasCpuId())
    {
        /** @todo Should add this MSR to CPUMMSRS and expose it via SUPDrv... */
        g_CpumHostFeatures.s.fArchRdclNo             = 0;
        g_CpumHostFeatures.s.fArchIbrsAll            = 0;
        g_CpumHostFeatures.s.fArchRsbOverride        = 0;
        g_CpumHostFeatures.s.fArchVmmNeedNotFlushL1d = 0;
        g_CpumHostFeatures.s.fArchMdsNo              = 0;
        uint32_t const cStdRange = ASMCpuId_EAX(0);
        if (   RTX86IsValidStdRange(cStdRange)
            && cStdRange >= 7)
        {
            uint32_t const fStdFeaturesEdx = ASMCpuId_EDX(1);
            uint32_t fStdExtFeaturesEdx;
            ASMCpuIdExSlow(7, 0, 0, 0, NULL, NULL, NULL, &fStdExtFeaturesEdx);
            if (   (fStdExtFeaturesEdx & X86_CPUID_STEXT_FEATURE_EDX_ARCHCAP)
                && (fStdFeaturesEdx    & X86_CPUID_FEATURE_EDX_MSR))
            {
                uint64_t fArchVal = ASMRdMsr(MSR_IA32_ARCH_CAPABILITIES);
                g_CpumHostFeatures.s.fArchRdclNo             = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_RDCL_NO);
                g_CpumHostFeatures.s.fArchIbrsAll            = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_IBRS_ALL);
                g_CpumHostFeatures.s.fArchRsbOverride        = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_RSBO);
                g_CpumHostFeatures.s.fArchVmmNeedNotFlushL1d = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_VMM_NEED_NOT_FLUSH_L1D);
                g_CpumHostFeatures.s.fArchMdsNo              = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_MDS_NO);
            }
            else
                g_CpumHostFeatures.s.fArchCap = 0;
        }
    }

    return VINF_SUCCESS;
}


/**
 * Terminate the module.
 */
VMMR0_INT_DECL(int) CPUMR0ModuleTerm(void)
{
    return VINF_SUCCESS;
}


/**
 * Initializes the CPUM data in the VM structure.
 *
 * @param   pGVM        The global VM structure.
 */
VMMR0_INT_DECL(void) CPUMR0InitPerVMData(PGVM pGVM)
{
    /* Copy the ring-0 host feature set to the shared part so ring-3 can pick it up. */
    pGVM->cpum.s.HostFeatures = g_CpumHostFeatures.s;
}


/**
 * Check the CPUID features of this particular CPU and disable relevant features
 * for the guest which do not exist on this CPU. We have seen systems where the
 * X86_CPUID_FEATURE_ECX_MONITOR feature flag is only set on some host CPUs, see
 * @bugref{5436}.
 *
 * @note This function might be called simultaneously on more than one CPU!
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     Pointer to the VM structure.
 * @param   pvUser2     Ignored.
 */
static DECLCALLBACK(void) cpumR0CheckCpuidLegacy(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    PVMCC     pVM   = (PVMCC)pvUser1;

    NOREF(idCpu); NOREF(pvUser2);
    for (uint32_t i = 0; i < RT_ELEMENTS(g_aCpuidUnifyBits); i++)
    {
        /* Note! Cannot use cpumCpuIdGetLeaf from here because we're not
                 necessarily in the VM process context.  So, we using the
                 legacy arrays as temporary storage. */

        uint32_t   uLeaf = g_aCpuidUnifyBits[i].uLeaf;
        PCPUMCPUID pLegacyLeaf;
        if (uLeaf < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdPatmStd))
            pLegacyLeaf = &pVM->cpum.s.aGuestCpuIdPatmStd[uLeaf];
        else if (uLeaf - UINT32_C(0x80000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdPatmExt))
            pLegacyLeaf = &pVM->cpum.s.aGuestCpuIdPatmExt[uLeaf - UINT32_C(0x80000000)];
        else if (uLeaf - UINT32_C(0xc0000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdPatmCentaur))
            pLegacyLeaf = &pVM->cpum.s.aGuestCpuIdPatmCentaur[uLeaf - UINT32_C(0xc0000000)];
        else
            continue;

        uint32_t eax, ebx, ecx, edx;
        ASMCpuIdExSlow(uLeaf, 0, 0, 0, &eax, &ebx, &ecx, &edx);

        ASMAtomicAndU32(&pLegacyLeaf->uEcx, ecx | ~g_aCpuidUnifyBits[i].uEcx);
        ASMAtomicAndU32(&pLegacyLeaf->uEdx, edx | ~g_aCpuidUnifyBits[i].uEdx);
    }
}


/**
 * Does Ring-0 CPUM initialization.
 *
 * This is mainly to check that the Host CPU mode is compatible
 * with VBox.
 *
 * @returns VBox status code.
 * @param   pVM         The cross context VM structure.
 */
VMMR0_INT_DECL(int) CPUMR0InitVM(PVMCC pVM)
{
    LogFlow(("CPUMR0Init: %p\n", pVM));
    AssertCompile(sizeof(pVM->aCpus[0].cpum.s.Host.abXState) >= sizeof(pVM->aCpus[0].cpum.s.Guest.abXState));

    /*
     * Check CR0 & CR4 flags.
     */
    uint32_t u32CR0 = ASMGetCR0();
    if ((u32CR0 & (X86_CR0_PE | X86_CR0_PG)) != (X86_CR0_PE | X86_CR0_PG)) /* a bit paranoid perhaps.. */
    {
        Log(("CPUMR0Init: PE or PG not set. cr0=%#x\n", u32CR0));
        return VERR_UNSUPPORTED_CPU_MODE;
    }

    /*
     * Check for sysenter and syscall usage.
     */
    if (ASMHasCpuId())
    {
        /*
         * SYSENTER/SYSEXIT
         *
         * Intel docs claim you should test both the flag and family, model &
         * stepping because some Pentium Pro CPUs have the SEP cpuid flag set,
         * but don't support it.  AMD CPUs may support this feature in legacy
         * mode, they've banned it from long mode.  Since we switch to 32-bit
         * mode when entering raw-mode context the feature would become
         * accessible again on AMD CPUs, so we have to check regardless of
         * host bitness.
         */
        uint32_t u32CpuVersion;
        uint32_t u32Dummy;
        uint32_t fFeatures; /* (Used further down to check for MSRs, so don't clobber.) */
        ASMCpuId(1, &u32CpuVersion, &u32Dummy, &u32Dummy, &fFeatures);
        uint32_t const u32Family   = u32CpuVersion >> 8;
        uint32_t const u32Model    = (u32CpuVersion >> 4) & 0xF;
        uint32_t const u32Stepping = u32CpuVersion & 0xF;
        if (    (fFeatures & X86_CPUID_FEATURE_EDX_SEP)
            &&  (   u32Family   != 6    /* (> pentium pro) */
                 || u32Model    >= 3
                 || u32Stepping >= 3
                 || !ASMIsIntelCpu())
           )
        {
            /*
             * Read the MSR and see if it's in use or not.
             */
            uint32_t u32 = ASMRdMsr_Low(MSR_IA32_SYSENTER_CS);
            if (u32)
            {
                pVM->cpum.s.fHostUseFlags |= CPUM_USE_SYSENTER;
                Log(("CPUMR0Init: host uses sysenter cs=%08x%08x\n", ASMRdMsr_High(MSR_IA32_SYSENTER_CS), u32));
            }
        }

        /*
         * SYSCALL/SYSRET
         *
         * This feature is indicated by the SEP bit returned in EDX by CPUID
         * function 0x80000001.  Intel CPUs only supports this feature in
         * long mode.  Since we're not running 64-bit guests in raw-mode there
         * are no issues with 32-bit intel hosts.
         */
        uint32_t cExt = 0;
        ASMCpuId(0x80000000, &cExt, &u32Dummy, &u32Dummy, &u32Dummy);
        if (RTX86IsValidExtRange(cExt))
        {
            uint32_t fExtFeaturesEDX = ASMCpuId_EDX(0x80000001);
            if (fExtFeaturesEDX & X86_CPUID_EXT_FEATURE_EDX_SYSCALL)
            {
#ifdef RT_ARCH_X86
                if (!ASMIsIntelCpu())
#endif
                {
                    uint64_t fEfer = ASMRdMsr(MSR_K6_EFER);
                    if (fEfer & MSR_K6_EFER_SCE)
                    {
                        pVM->cpum.s.fHostUseFlags |= CPUM_USE_SYSCALL;
                        Log(("CPUMR0Init: host uses syscall\n"));
                    }
                }
            }
        }

        /*
         * Copy MSR_IA32_ARCH_CAPABILITIES bits over into the host and guest feature
         * structure and as well as the guest MSR.
         * Note! we assume this happens after the CPUMR3Init is done, so CPUID bits are settled.
         */
        pVM->cpum.s.HostFeatures.fArchRdclNo             = 0;
        pVM->cpum.s.HostFeatures.fArchIbrsAll            = 0;
        pVM->cpum.s.HostFeatures.fArchRsbOverride        = 0;
        pVM->cpum.s.HostFeatures.fArchVmmNeedNotFlushL1d = 0;
        pVM->cpum.s.HostFeatures.fArchMdsNo              = 0;
        uint32_t const cStdRange = ASMCpuId_EAX(0);
        if (   RTX86IsValidStdRange(cStdRange)
            && cStdRange >= 7)
        {
            uint32_t fEdxFeatures = ASMCpuId_EDX(7);
            if (   (fEdxFeatures & X86_CPUID_STEXT_FEATURE_EDX_ARCHCAP)
                && (fFeatures & X86_CPUID_FEATURE_EDX_MSR))
            {
                /* Host: */
                uint64_t fArchVal = ASMRdMsr(MSR_IA32_ARCH_CAPABILITIES);
                pVM->cpum.s.HostFeatures.fArchRdclNo             = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_RDCL_NO);
                pVM->cpum.s.HostFeatures.fArchIbrsAll            = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_IBRS_ALL);
                pVM->cpum.s.HostFeatures.fArchRsbOverride        = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_RSBO);
                pVM->cpum.s.HostFeatures.fArchVmmNeedNotFlushL1d = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_VMM_NEED_NOT_FLUSH_L1D);
                pVM->cpum.s.HostFeatures.fArchMdsNo              = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_MDS_NO);

                /* guest: */
                if (!pVM->cpum.s.GuestFeatures.fArchCap)
                    fArchVal = 0;
                else if (!pVM->cpum.s.GuestFeatures.fIbrs)
                    fArchVal &= ~MSR_IA32_ARCH_CAP_F_IBRS_ALL;
                VMCC_FOR_EACH_VMCPU_STMT(pVM, pVCpu->cpum.s.GuestMsrs.msr.ArchCaps = fArchVal);
                pVM->cpum.s.GuestFeatures.fArchRdclNo             = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_RDCL_NO);
                pVM->cpum.s.GuestFeatures.fArchIbrsAll            = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_IBRS_ALL);
                pVM->cpum.s.GuestFeatures.fArchRsbOverride        = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_RSBO);
                pVM->cpum.s.GuestFeatures.fArchVmmNeedNotFlushL1d = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_VMM_NEED_NOT_FLUSH_L1D);
                pVM->cpum.s.GuestFeatures.fArchMdsNo              = RT_BOOL(fArchVal & MSR_IA32_ARCH_CAP_F_MDS_NO);
            }
            else
                pVM->cpum.s.HostFeatures.fArchCap = 0;
        }

        /*
         * Unify/cross check some CPUID feature bits on all available CPU cores
         * and threads.  We've seen CPUs where the monitor support differed.
         *
         * Because the hyper heap isn't always mapped into ring-0, we cannot
         * access it from a RTMpOnAll callback.  We use the legacy CPUID arrays
         * as temp ring-0 accessible memory instead, ASSUMING that they're all
         * up to date when we get here.
         */
        RTMpOnAll(cpumR0CheckCpuidLegacy, pVM, NULL);

        for (uint32_t i = 0; i < RT_ELEMENTS(g_aCpuidUnifyBits); i++)
        {
            bool            fIgnored;
            uint32_t        uLeaf = g_aCpuidUnifyBits[i].uLeaf;
            PCPUMCPUIDLEAF  pLeaf = cpumCpuIdGetLeafEx(pVM, uLeaf, 0, &fIgnored);
            if (pLeaf)
            {
                PCPUMCPUID pLegacyLeaf;
                if (uLeaf < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdPatmStd))
                    pLegacyLeaf = &pVM->cpum.s.aGuestCpuIdPatmStd[uLeaf];
                else if (uLeaf - UINT32_C(0x80000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdPatmExt))
                    pLegacyLeaf = &pVM->cpum.s.aGuestCpuIdPatmExt[uLeaf - UINT32_C(0x80000000)];
                else if (uLeaf - UINT32_C(0xc0000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdPatmCentaur))
                    pLegacyLeaf = &pVM->cpum.s.aGuestCpuIdPatmCentaur[uLeaf - UINT32_C(0xc0000000)];
                else
                    continue;

                pLeaf->uEcx = pLegacyLeaf->uEcx;
                pLeaf->uEdx = pLegacyLeaf->uEdx;
            }
        }

    }


    /*
     * Check if debug registers are armed.
     * This ASSUMES that DR7.GD is not set, or that it's handled transparently!
     */
    uint32_t u32DR7 = ASMGetDR7();
    if (u32DR7 & X86_DR7_ENABLED_MASK)
    {
        VMCC_FOR_EACH_VMCPU_STMT(pVM, pVCpu->cpum.s.fUseFlags |= CPUM_USE_DEBUG_REGS_HOST);
        Log(("CPUMR0Init: host uses debug registers (dr7=%x)\n", u32DR7));
    }

    return VINF_SUCCESS;
}


/**
 * Trap handler for device-not-available fault (\#NM).
 * Device not available, FP or (F)WAIT instruction.
 *
 * @returns VBox status code.
 * @retval VINF_SUCCESS           if the guest FPU state is loaded.
 * @retval VINF_EM_RAW_GUEST_TRAP if it is a guest trap.
 * @retval VINF_CPUM_HOST_CR0_MODIFIED if we modified the host CR0.
 *
 * @param   pVM         The cross context VM structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 */
//处理 #NM（设备不可用）异常的 Ring-0 函数
/*
    区分异常来源（宿主或 Guest）。
    根据 Guest CR0 状态决策处理逻辑（加载 FPU 或注入 Guest 异常）。
    处理 FPU 状态切换（通过调用 CPUMR0LoadGuestFPU）。 
*/
VMMR0_INT_DECL(int) CPUMR0Trap07Handler(PVMCC pVM, PVMCPUCC pVCpu)
{
    Assert(pVM->cpum.s.HostFeatures.fFxSaveRstor);//确认宿主 CPU 支持 FXSAVE/FXRSTOR 指令。
    Assert(ASMGetCR4() & X86_CR4_OSFXSR);//检查 CR4 寄存器的 OSFXSR 位是否启用（必须为 1 才能使用 SSE/XMM 指令）。

    /* If the FPU state has already been loaded, then it's a guest trap. */
    if (CPUMIsGuestFPUStateActive(pVCpu))//若 Guest FPU 状态已激活（CPUM_USED_FPU_GUEST 标志被设置），说明异常来自 Guest。
    {
        //断言 Guest CR0 必须满足以下组合之一：
        //MP=1 + TS=1（典型配置，触发 #NM）。
        //MP=1 + TS=1 + EM=1（模拟模式）。
        Assert(    ((pVCpu->cpum.s.Guest.cr0 & (X86_CR0_MP | X86_CR0_EM | X86_CR0_TS)) == (X86_CR0_MP | X86_CR0_TS))
               ||  ((pVCpu->cpum.s.Guest.cr0 & (X86_CR0_MP | X86_CR0_EM | X86_CR0_TS)) == (X86_CR0_MP | X86_CR0_TS | X86_CR0_EM)));
        return VINF_EM_RAW_GUEST_TRAP;//返回 VINF_EM_RAW_GUEST_TRAP，由 VirtualBox 将异常注入 Guest。
    }

    /*
     * There are two basic actions:
     *   1. Save host fpu and restore guest fpu.
     *   2. Generate guest trap.
     *
     * When entering the hypervisor we'll always enable MP (for proper wait
     * trapping) and TS (for intercepting all fpu/mmx/sse stuff). The EM flag
     * is taken from the guest OS in order to get proper SSE handling.
     *
     *
     * Actions taken depending on the guest CR0 flags:
     *
     *   3    2    1
     *  TS | EM | MP | FPUInstr | WAIT :: VMM Action
     * ------------------------------------------------------------------------
     *   0 |  0 |  0 | Exec     | Exec :: Clear TS & MP, Save HC, Load GC.
     *   0 |  0 |  1 | Exec     | Exec :: Clear TS, Save HC, Load GC.
     *   0 |  1 |  0 | #NM      | Exec :: Clear TS & MP, Save HC, Load GC.
     *   0 |  1 |  1 | #NM      | Exec :: Clear TS, Save HC, Load GC.
     *   1 |  0 |  0 | #NM      | Exec :: Clear MP, Save HC, Load GC. (EM is already cleared.)
     *   1 |  0 |  1 | #NM      | #NM  :: Go to guest taking trap there.
     *   1 |  1 |  0 | #NM      | Exec :: Clear MP, Save HC, Load GC. (EM is already set.)
     *   1 |  1 |  1 | #NM      | #NM  :: Go to guest taking trap there.
     */
    //CR0 标志作用：
    //MP（Monitor Coprocessor）：与 TS 配合控制 WAIT 指令行为。
    //EM（Emulation）：若置 1，则浮点指令触发 #UD（无效操作码）。
    //TS（Task Switched）：若置 1，则浮点指令触发 #NM。
    switch (pVCpu->cpum.s.Guest.cr0 & (X86_CR0_MP | X86_CR0_EM | X86_CR0_TS))
    {
        //若 MP=1 + TS=1（无论 EM），直接注入 Guest 异常。
        case X86_CR0_MP | X86_CR0_TS:
        case X86_CR0_MP | X86_CR0_TS | X86_CR0_EM:
            return VINF_EM_RAW_GUEST_TRAP;
        default:
            //其他情况（如 TS=0），继续执行 FPU 状态加载
            break;
    }

    //保存宿主 FPU 状态，加载 Guest FPU 状态。
    //返回状态码（如 VINF_SUCCESS 或错误码）。
    return CPUMR0LoadGuestFPU(pVM, pVCpu);
}


/**
 * Saves the host-FPU/XMM state (if necessary) and (always) loads the guest-FPU
 * state into the CPU.
 *
 * @returns VINF_SUCCESS on success, host CR0 unmodified.
 * @returns VINF_CPUM_HOST_CR0_MODIFIED on success when the host CR0 was
 *          modified and VT-x needs to update the value in the VMCS.
 *
 * @param   pVM     The cross context VM structure.
 * @param   pVCpu   The cross context virtual CPU structure.
 */
//加载 Guest FPU 状态‌ 的 Ring-0 函数
/*
    保存宿主 FPU 状态（确保宿主状态不被 Guest 破坏）。
    加载 Guest FPU 状态（恢复 Guest 的浮点运算环境）。
    处理特殊 CPU 特性（如 FFXSR 模式下的 XMM 寄存器加载问题）。
    原子化更新状态标志（避免多核竞争）。
*/
VMMR0_INT_DECL(int) CPUMR0LoadGuestFPU(PVMCC pVM, PVMCPUCC pVCpu)
{
    int rc;
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));//确保当前线程不可被抢占（防止 FPU 状态切换过程中被中断
    Assert(!(pVCpu->cpum.s.fUseFlags & CPUM_USED_FPU_GUEST));//断言 Guest 未占用 FPU

    /* Notify the support driver prior to loading the guest-FPU register state. */
    //若启用（VMMR0ThreadCtxHookIsEnabled），需额外处理线程绑定。
    SUPR0FpuBegin(VMMR0ThreadCtxHookIsEnabled(pVCpu));//通知 VirtualBox 支持驱动（如 VBoxDrv）即将加载 Guest FPU 状态。
    /** @todo use return value? Currently skipping that to be on the safe side
     *        wrt. extended state (linux). */

    //fLeakyFxSR‌：
    //标记某些 CPU（如早期 AMD）的 FXSAVE/FXRSTOR 指令存在缺陷，需特殊处理。
    if (!pVM->cpum.s.HostFeatures.fLeakyFxSR)
    {
        Assert(!(pVCpu->cpum.s.fUseFlags & CPUM_USED_MANUAL_XMM_RESTORE));
        /*
            内部调用 FXSAVE 保存宿主 FPU 状态到 pVCpu->cpum.s.Host。
            调用 FXRSTOR 从 pVCpu->cpum.s.Guest 加载 Guest FPU 状态。
        */
        rc = cpumR0SaveHostRestoreGuestFPUState(&pVCpu->cpum.s);
    }
    else
    {
        Assert(!(pVCpu->cpum.s.fUseFlags & CPUM_USED_MANUAL_XMM_RESTORE) || (pVCpu->cpum.s.fUseFlags & CPUM_USED_FPU_HOST));
        /** @todo r=ramshankar: Can't we used a cached value here
         *        instead of reading the MSR? host EFER doesn't usually
         *        change. */
        uint64_t uHostEfer = ASMRdMsr(MSR_K6_EFER);
        if (!(uHostEfer & MSR_K6_EFER_FFXSR))
            rc = cpumR0SaveHostRestoreGuestFPUState(&pVCpu->cpum.s);
        else
        {
            RTCCUINTREG const uSavedFlags = ASMIntDisableFlags();//禁用中断（ASMIntDisableFlags）。
            pVCpu->cpum.s.fUseFlags |= CPUM_USED_MANUAL_XMM_RESTORE;//设置 CPUM_USED_MANUAL_XMM_RESTORE 标志（标记需手动处理 XMM 状态）。
            ASMWrMsr(MSR_K6_EFER, uHostEfer & ~MSR_K6_EFER_FFXSR); //AMD CPU 的 FFXSR（Fast FXSAVE/RSTOR）模式会跳过 XMM 寄存器的加载，需临时禁用。
            rc = cpumR0SaveHostRestoreGuestFPUState(&pVCpu->cpum.s);
            ASMWrMsr(MSR_K6_EFER, uHostEfer | MSR_K6_EFER_FFXSR);
            ASMSetFlags(uSavedFlags);//恢复 FFXSR 和中断状态。
        }
    }
    Assert(   (pVCpu->cpum.s.fUseFlags & (CPUM_USED_FPU_GUEST | CPUM_USED_FPU_HOST | CPUM_USED_FPU_SINCE_REM))
           ==                            (CPUM_USED_FPU_GUEST | CPUM_USED_FPU_HOST | CPUM_USED_FPU_SINCE_REM));
    Assert(pVCpu->cpum.s.Guest.fUsedFpuGuest);
    return rc;
}


/**
 * Saves the guest FPU/XMM state if needed, restores the host FPU/XMM state as
 * needed.
 *
 * @returns true if we saved the guest state.
 * @param   pVCpu       The cross context virtual CPU structure.
 */
//是 VirtualBox 中用于 管理 FPU（浮点单元）状态切换的 Ring-0 函数
/*
    保存 Guest 的 FPU 状态（若 Guest 使用了 FPU）。
    恢复宿主的 FPU 状态（确保宿主浮点运算不受干扰）。
    处理特殊 CPU 特性（如 AMD CPU 的 FFXSR 模式）。
    原子化更新状态标志（避免多核竞争）。
*/
VMMR0_INT_DECL(bool) CPUMR0FpuStateMaybeSaveGuestAndRestoreHost(PVMCPUCC pVCpu)
{
    bool fSavedGuest;
	//确认宿主 CPU 支持 FXSAVE/FXRSTOR 指令（现代 x86 CPU 均支持）。
    Assert(pVCpu->CTX_SUFF(pVM)->cpum.s.HostFeatures.fFxSaveRstor);
    //检查 CR4 寄存器的 OSFXSR 位是否启用（必须为 1 才能使用 SSE/XMM 指令）。
    Assert(ASMGetCR4() & X86_CR4_OSFXSR);

    //若 CPUM_USED_FPU_GUEST 被标记，表示 Guest 曾使用 FPU，需保存其状态。
    if (pVCpu->cpum.s.fUseFlags & (CPUM_USED_FPU_GUEST | CPUM_USED_FPU_HOST))
    {
        fSavedGuest = RT_BOOL(pVCpu->cpum.s.fUseFlags & CPUM_USED_FPU_GUEST);
        //断言 fSavedGuest 与 Guest.fUsedFpuGuest 一致（防止状态不一致）。
        Assert(fSavedGuest == pVCpu->cpum.s.Guest.fUsedFpuGuest);
        if (!(pVCpu->cpum.s.fUseFlags & CPUM_USED_MANUAL_XMM_RESTORE))
        /*
            cpumR0SaveGuestRestoreHostFPUState：
              内部调用 FXSAVE 保存 Guest FPU 状态到 pVCpu->cpum.s.Guest。
              调用 FXRSTOR 从 pVCpu->cpum.s.Host 恢复宿主 FPU 状态。
        */
            cpumR0SaveGuestRestoreHostFPUState(&pVCpu->cpum.s);
        else
        {
            /* Temporarily clear MSR_K6_EFER_FFXSR or else we'll be unable to
               save/restore the XMM state with fxsave/fxrstor. */
            uint64_t uHostEfer = ASMRdMsr(MSR_K6_EFER);
            if (uHostEfer & MSR_K6_EFER_FFXSR)
            {
                RTCCUINTREG const uSavedFlags = ASMIntDisableFlags();//禁用中断
                ASMWrMsr(MSR_K6_EFER, uHostEfer & ~MSR_K6_EFER_FFXSR);// 临时禁用 FFXSR
                cpumR0SaveGuestRestoreHostFPUState(&pVCpu->cpum.s);//执行常规 FPU 状态保存/恢复。
                ASMWrMsr(MSR_K6_EFER, uHostEfer | MSR_K6_EFER_FFXSR); // 恢复 FFXSR
                ASMSetFlags(uSavedFlags);//恢复 FFXSR 和中断状态。
            }
            else
                cpumR0SaveGuestRestoreHostFPUState(&pVCpu->cpum.s);
            pVCpu->cpum.s.fUseFlags &= ~CPUM_USED_MANUAL_XMM_RESTORE;
        }

        /* Notify the support driver after loading the host-FPU register state. */
        SUPR0FpuEnd(VMMR0ThreadCtxHookIsEnabled(pVCpu));//通知 VirtualBox 支持驱动（如 VBoxDrv）宿主 FPU 状态已恢复。
    }
    else
        fSavedGuest = false;
    Assert(!(  pVCpu->cpum.s.fUseFlags
             & (CPUM_USED_FPU_GUEST | CPUM_USED_FPU_HOST | CPUM_USED_MANUAL_XMM_RESTORE)));//确保所有 FPU 相关标志已被清除
    Assert(!pVCpu->cpum.s.Guest.fUsedFpuGuest);//断言 Guest 不再占用 FPU（fUsedFpuGuest=0）。
    return fSavedGuest;
}


/**
 * Saves the host debug state, setting CPUM_USED_HOST_DEBUG_STATE and loading
 * DR7 with safe values.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 */
static int cpumR0SaveHostDebugState(PVMCPUCC pVCpu)
{
    /*
     * Save the host state.
     */
    pVCpu->cpum.s.Host.dr0 = ASMGetDR0();
    pVCpu->cpum.s.Host.dr1 = ASMGetDR1();
    pVCpu->cpum.s.Host.dr2 = ASMGetDR2();
    pVCpu->cpum.s.Host.dr3 = ASMGetDR3();
    pVCpu->cpum.s.Host.dr6 = ASMGetDR6();
    /** @todo dr7 might already have been changed to 0x400; don't care right now as it's harmless. */
    pVCpu->cpum.s.Host.dr7 = ASMGetDR7();

    /* Preemption paranoia. */
    ASMAtomicOrU32(&pVCpu->cpum.s.fUseFlags, CPUM_USED_DEBUG_REGS_HOST);

    /*
     * Make sure DR7 is harmless or else we could trigger breakpoints when
     * load guest or hypervisor DRx values later.
     */
    if (pVCpu->cpum.s.Host.dr7 != X86_DR7_INIT_VAL)
        ASMSetDR7(X86_DR7_INIT_VAL);

    return VINF_SUCCESS;
}


/**
 * Saves the guest DRx state residing in host registers and restore the host
 * register values.
 *
 * The guest DRx state is only saved if CPUMR0LoadGuestDebugState was called,
 * since it's assumed that we're shadowing the guest DRx register values
 * accurately when using the combined hypervisor debug register values
 * (CPUMR0LoadHyperDebugState).
 *
 * @returns true if either guest or hypervisor debug registers were loaded.
 * @param   pVCpu       The cross context virtual CPU structure of the calling EMT.
 * @param   fDr6        Whether to include DR6 or not.
 * @thread  EMT(pVCpu)
 */
VMMR0_INT_DECL(bool) CPUMR0DebugStateMaybeSaveGuestAndRestoreHost(PVMCPUCC pVCpu, bool fDr6)
{
    //禁止抢占：确保当前线程不会被调度到其他 CPU 核（调试寄存器是 CPU 核特定的资源）。
    //关键性：若抢占未被禁用，可能导致寄存器状态保存/恢复到错误的 CPU 核。
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    //返回 true 表示当前有 Guest 或 Hypervisor 的调试寄存器被加载到宿主 CPU。
    bool const fDrXLoaded = RT_BOOL(pVCpu->cpum.s.fUseFlags & (CPUM_USED_DEBUG_REGS_GUEST | CPUM_USED_DEBUG_REGS_HYPER));

    /*
     * Do we need to save the guest DRx registered loaded into host registers?
     * (DR7 and DR6 (if fDr6 is true) are left to the caller.)
     */
	//保存 Guest 的断点地址
    //确保 Guest 调试状态在 VM-Exit 后不丢失，同时避免泄漏宿主状态。
    if (pVCpu->cpum.s.fUseFlags & CPUM_USED_DEBUG_REGS_GUEST)
    {
        pVCpu->cpum.s.Guest.dr[0] = ASMGetDR0();
        pVCpu->cpum.s.Guest.dr[1] = ASMGetDR1();
        pVCpu->cpum.s.Guest.dr[2] = ASMGetDR2();
        pVCpu->cpum.s.Guest.dr[3] = ASMGetDR3();
        if (fDr6)
            pVCpu->cpum.s.Guest.dr[6] = ASMGetDR6() | X86_DR6_RA1_MASK; /* ASSUMES no guest supprot for TSX-NI / RTM. */
    }

    //标记 Guest/Hypervisor 不再占用调试寄存器，允许后续操作（如宿主调试）。
    ASMAtomicAndU32(&pVCpu->cpum.s.fUseFlags, ~(CPUM_USED_DEBUG_REGS_GUEST | CPUM_USED_DEBUG_REGS_HYPER));

    /*
     * Restore the host's debug state. DR0-3, DR6 and only then DR7!
     */
    //恢复宿主调试寄存器状态
    if (pVCpu->cpum.s.fUseFlags & CPUM_USED_DEBUG_REGS_HOST)
    {
        /* A bit of paranoia first... */
        uint64_t uCurDR7 = ASMGetDR7();
        if (uCurDR7 != X86_DR7_INIT_VAL)
            ASMSetDR7(X86_DR7_INIT_VAL); // 强制禁用所有断点

        //恢复 0-3: 宿主断点地址
        ASMSetDR0(pVCpu->cpum.s.Host.dr0);
        ASMSetDR1(pVCpu->cpum.s.Host.dr1);
        ASMSetDR2(pVCpu->cpum.s.Host.dr2);
        ASMSetDR3(pVCpu->cpum.s.Host.dr3);
        /** @todo consider only updating if they differ, esp. DR6. Need to figure how
         *        expensive DRx reads are over DRx writes.  */
        //恢复 DR6/DR7：宿主调试状态和控制寄存器。
        ASMSetDR6(pVCpu->cpum.s.Host.dr6);
        ASMSetDR7(pVCpu->cpum.s.Host.dr7);

        ASMAtomicAndU32(&pVCpu->cpum.s.fUseFlags, ~CPUM_USED_DEBUG_REGS_HOST);
    }

    return fDrXLoaded;
}


/**
 * Saves the guest DRx state if it resides host registers.
 *
 * This does NOT clear any use flags, so the host registers remains loaded with
 * the guest DRx state upon return.  The purpose is only to make sure the values
 * in the CPU context structure is up to date.
 *
 * @returns true if the host registers contains guest values, false if not.
 * @param   pVCpu       The cross context virtual CPU structure of the calling EMT.
 * @param   fDr6        Whether to include DR6 or not.
 * @thread  EMT(pVCpu)
 */
VMMR0_INT_DECL(bool) CPUMR0DebugStateMaybeSaveGuest(PVMCPUCC pVCpu, bool fDr6)
{
    /*
     * Do we need to save the guest DRx registered loaded into host registers?
     * (DR7 and DR6 (if fDr6 is true) are left to the caller.)
     */
    if (pVCpu->cpum.s.fUseFlags & CPUM_USED_DEBUG_REGS_GUEST)
    {
        pVCpu->cpum.s.Guest.dr[0] = ASMGetDR0();
        pVCpu->cpum.s.Guest.dr[1] = ASMGetDR1();
        pVCpu->cpum.s.Guest.dr[2] = ASMGetDR2();
        pVCpu->cpum.s.Guest.dr[3] = ASMGetDR3();
        if (fDr6)
            pVCpu->cpum.s.Guest.dr[6] = ASMGetDR6();
        return true;
    }
    return false;
}


/**
 * Lazily sync in the debug state.
 *
 * @param   pVCpu       The cross context virtual CPU structure of the calling EMT.
 * @param   fDr6        Whether to include DR6 or not.
 * @thread  EMT(pVCpu)
 */
//加载 Guest（虚拟机）调试寄存器状态
VMMR0_INT_DECL(void) CPUMR0LoadGuestDebugState(PVMCPUCC pVCpu, bool fDr6)
{
    /*
     * Save the host state and disarm all host BPs.
     */
    //保存宿主调试寄存器状态（防止破坏宿主调试环境）。
    cpumR0SaveHostDebugState(pVCpu);
    Assert(ASMGetDR7() == X86_DR7_INIT_VAL);

    /*
     * Activate the guest state DR0-3.
     * DR7 and DR6 (if fDr6 is true) are left to the caller.
     */
    //加载 Guest 的调试寄存器值（DR0-DR3）。
    ASMSetDR0(pVCpu->cpum.s.Guest.dr[0]);
    ASMSetDR1(pVCpu->cpum.s.Guest.dr[1]);
    ASMSetDR2(pVCpu->cpum.s.Guest.dr[2]);
    ASMSetDR3(pVCpu->cpum.s.Guest.dr[3]);
    if (fDr6)
        ASMSetDR6(pVCpu->cpum.s.Guest.dr[6]);

    //标记调试寄存器已用于 Guest（避免冲突）。
    ASMAtomicOrU32(&pVCpu->cpum.s.fUseFlags, CPUM_USED_DEBUG_REGS_GUEST);
}


/**
 * Lazily sync in the hypervisor debug state
 *
 * @param   pVCpu       The cross context virtual CPU structure of the calling EMT.
 * @param   fDr6        Whether to include DR6 or not.
 * @thread  EMT(pVCpu)
 */
/*
    加载 Hypervisor（VMM）调试寄存器状态的 Ring-0 函数，核心职责包括：
        保存宿主调试寄存器状态, 防止破坏宿主调试环境）。
        更新 Hypervisor 的调试寄存器值DR0-DR3）。
        选择性加载 DR6(根据 fDr6 参数）。
        标记调试寄存器已使用（避免冲突）。
 * */
/*
调试寄存器上下文切换
    宿主 → Hypervisor：
    保存宿主状态 → 加载 Hypervisor 状态。
    Hypervisor → Guest：
    通过 VMCS 的 DR_EXITING 字段自动保存/恢复（硬件辅助）。 
*/
VMMR0_INT_DECL(void) CPUMR0LoadHyperDebugState(PVMCPUCC pVCpu, bool fDr6)
{
    /*
     * Save the host state and disarm all host BPs.
     */
    cpumR0SaveHostDebugState(pVCpu);//将当前 CPU 的 DR0-DR7 保存到 pVCpu->cpum.s.Host.dr[]。
    //确保 DR7（调试控制寄存器）处于初始值（X86_DR7_INIT_VAL），即所有断点禁用。
    Assert(ASMGetDR7() == X86_DR7_INIT_VAL);//通过 ASMGetDRx() 读取寄存器值

    /*
     * Make sure the hypervisor values are up to date.
     */
    /*
	   重新计算 Hypervisor 的 DR0-DR7 值（基于虚拟 CPU 状态）。
       参数 UINT8_MAX：表示不立即加载到物理寄存器（仅更新内存中的值）。
	*/
    CPUMRecalcHyperDRx(pVCpu, UINT8_MAX /* no loading, please */);

    /*
     * Activate the guest state DR0-3.
     * DR7 and DR6 (if fDr6 is true) are left to the caller.
     */
    //DR0-DR3 无条件加载 Hypervisor 的断点地址（用于 VMM 自身调试）。
    ASMSetDR0(pVCpu->cpum.s.Hyper.dr[0]);
    ASMSetDR1(pVCpu->cpum.s.Hyper.dr[1]);
    ASMSetDR2(pVCpu->cpum.s.Hyper.dr[2]);
    ASMSetDR3(pVCpu->cpum.s.Hyper.dr[3]);
	//仅在 fDr6=true 时初始化为 X86_DR6_INIT_VAL（清除断点触发状态）。
    //典型场景：在 VM-Entry 前重置 DR6，避免遗留状态干扰。
    if (fDr6)
        ASMSetDR6(X86_DR6_INIT_VAL);

    //原子操作设置 CPUM_USED_DEBUG_REGS_HYPER 标志，表示 Hypervisor 正在使用调试寄存器。
    //作用防止 Guest OS 误修改调试寄存器（需通过 VMM 拦截）。
    ASMAtomicOrU32(&pVCpu->cpum.s.fUseFlags, CPUM_USED_DEBUG_REGS_HYPER);
}

