/* $Id$ */
/** @file
 * Hardware Assisted Virtualization Manager (HM) - Host Context Ring-0.
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
#define LOG_GROUP LOG_GROUP_HM
#define VMCPU_INCL_CPUM_GST_CTX
#include <VBox/vmm/hm.h>
#include <VBox/vmm/pgm.h>
#include "HMInternal.h"
#include <VBox/vmm/vmcc.h>
#include <VBox/vmm/hm_svm.h>
#include <VBox/vmm/hmvmxinline.h>
#include <VBox/err.h>
#include <VBox/log.h>
#include <iprt/assert.h>
#include <iprt/asm.h>
#include <iprt/asm-amd64-x86.h>
#include <iprt/cpuset.h>
#include <iprt/mem.h>
#include <iprt/memobj.h>
#include <iprt/once.h>
#include <iprt/param.h>
#include <iprt/power.h>
#include <iprt/string.h>
#include <iprt/thread.h>
#include <iprt/x86.h>
#include "HMVMXR0.h"
#include "HMSVMR0.h"


/*********************************************************************************************************************************
*   Internal Functions                                                                                                           *
*********************************************************************************************************************************/
static DECLCALLBACK(void) hmR0EnableCpuCallback(RTCPUID idCpu, void *pvUser1, void *pvUser2);
static DECLCALLBACK(void) hmR0DisableCpuCallback(RTCPUID idCpu, void *pvUser1, void *pvUser2);
static DECLCALLBACK(void) hmR0PowerCallback(RTPOWEREVENT enmEvent, void *pvUser);
static DECLCALLBACK(void) hmR0MpEventCallback(RTMPEVENT enmEvent, RTCPUID idCpu, void *pvData);


/*********************************************************************************************************************************
*   Structures and Typedefs                                                                                                      *
*********************************************************************************************************************************/
/**
 * This is used to manage the status code of a RTMpOnAll in HM.
 */
typedef struct HMR0FIRSTRC
{
    /** The status code. */
    int32_t volatile    rc;
    /** The ID of the CPU reporting the first failure. */
    RTCPUID volatile    idCpu;
} HMR0FIRSTRC;
/** Pointer to a first return code structure. */
typedef HMR0FIRSTRC *PHMR0FIRSTRC;

/**
 * Ring-0 method table for AMD-V and VT-x specific operations.
 */
typedef struct HMR0VTABLE
{
    DECLR0CALLBACKMEMBER(int,          pfnEnterSession, (PVMCPUCC pVCpu));
    DECLR0CALLBACKMEMBER(void,         pfnThreadCtxCallback, (RTTHREADCTXEVENT enmEvent, PVMCPUCC pVCpu, bool fGlobalInit));
    DECLR0CALLBACKMEMBER(int,          pfnAssertionCallback, (PVMCPUCC pVCpu));
    DECLR0CALLBACKMEMBER(int,          pfnExportHostState, (PVMCPUCC pVCpu));
    DECLR0CALLBACKMEMBER(VBOXSTRICTRC, pfnRunGuestCode, (PVMCPUCC pVCpu));
    DECLR0CALLBACKMEMBER(int,          pfnEnableCpu, (PHMPHYSCPU pHostCpu, PVMCC pVM, void *pvCpuPage, RTHCPHYS HCPhysCpuPage,
                                                      bool fEnabledByHost, PCSUPHWVIRTMSRS pHwvirtMsrs));
    DECLR0CALLBACKMEMBER(int,          pfnDisableCpu, (PHMPHYSCPU pHostCpu, void *pvCpuPage, RTHCPHYS HCPhysCpuPage));
    DECLR0CALLBACKMEMBER(int,          pfnInitVM, (PVMCC pVM));
    DECLR0CALLBACKMEMBER(int,          pfnTermVM, (PVMCC pVM));
    DECLR0CALLBACKMEMBER(int,          pfnSetupVM, (PVMCC pVM));
} HMR0VTABLE;


/*********************************************************************************************************************************
*   Global Variables                                                                                                             *
*********************************************************************************************************************************/
/** The active ring-0 HM operations (copied from one of the table at init). */
static HMR0VTABLE       g_HmR0Ops;
/** Indicates whether the host is suspending or not.  We'll refuse a few
 *  actions when the host is being suspended to speed up the suspending and
 *  avoid trouble. */
static bool volatile    g_fHmSuspended;
/** If set, VT-x/AMD-V is enabled globally at init time, otherwise it's
 * enabled and disabled each time it's used to execute guest code. */
static bool             g_fHmGlobalInit;
/** Host kernel flags that HM might need to know (SUPKERNELFEATURES_XXX). */
uint32_t                g_fHmHostKernelFeatures;
/** Maximum allowed ASID/VPID (inclusive).
 * @todo r=bird: This is exclusive for VT-x according to source code comment.
 *       Couldn't immediately find any docs on AMD-V, but suspect it is
 *       exclusive there as well given how hmR0SvmFlushTaggedTlb() use it. */
uint32_t                g_uHmMaxAsid;


/** Set if VT-x (VMX) is supported by the CPU. */
bool                    g_fHmVmxSupported = false;
/** VMX: Whether we're using the preemption timer or not. */
bool                    g_fHmVmxUsePreemptTimer;
/** VMX: The shift mask employed by the VMX-Preemption timer. */
uint8_t                 g_cHmVmxPreemptTimerShift;
/** VMX: Set if swapping EFER is supported.  */
bool                    g_fHmVmxSupportsVmcsEfer = false;
/** VMX: Whether we're using SUPR0EnableVTx or not. */
static bool             g_fHmVmxUsingSUPR0EnableVTx = false;
/** VMX: Set if we've called SUPR0EnableVTx(true) and should disable it during
 * module termination. */
static bool             g_fHmVmxCalledSUPR0EnableVTx = false;
/** VMX: Host CR0 value (set by ring-0 VMX init) */
uint64_t                g_uHmVmxHostCr0;
/** VMX: Host CR4 value (set by ring-0 VMX init) */
uint64_t                g_uHmVmxHostCr4;
/** VMX: Host EFER value (set by ring-0 VMX init) */
uint64_t                g_uHmVmxHostMsrEfer;
/** VMX: Host SMM monitor control (used for logging/diagnostics) */
uint64_t                g_uHmVmxHostSmmMonitorCtl;
/** VMX: Host core capabilities (set by ring-0 VMX init) */
uint64_t                g_uHmVmxHostCoreCap;
/** VMX: Host memory control register (set by ring-0 VMX init) */
uint64_t                g_uHmVmxHostMemoryCtrl;


/** Set if AMD-V is supported by the CPU. */
bool                    g_fHmSvmSupported = false;
/** SVM revision. */
uint32_t                g_uHmSvmRev;
/** SVM feature bits from cpuid 0x8000000a */
uint32_t                g_fHmSvmFeatures;


/** MSRs. */
SUPHWVIRTMSRS           g_HmMsrs;

/** Last recorded error code during HM ring-0 init. */
static int32_t          g_rcHmInit = VINF_SUCCESS;

/** Per CPU globals. */
static HMPHYSCPU        g_aHmCpuInfo[RTCPUSET_MAX_CPUS];

/** Whether we've already initialized all CPUs.
 * @remarks We could check the EnableAllCpusOnce state, but this is
 *          simpler and hopefully easier to understand. */
static bool             g_fHmEnabled = false;
/** Serialize initialization in HMR0EnableAllCpus. */
static RTONCE           g_HmEnableAllCpusOnce = RTONCE_INITIALIZER;


/** HM ring-0 operations for VT-x. */
static HMR0VTABLE const g_HmR0OpsVmx =
{
    /* .pfnEnterSession = */        VMXR0Enter,
    /* .pfnThreadCtxCallback = */   VMXR0ThreadCtxCallback,
    /* .pfnAssertionCallback = */   VMXR0AssertionCallback,
    /* .pfnExportHostState = */     VMXR0ExportHostState,
    /* .pfnRunGuestCode = */        VMXR0RunGuestCode,
    /* .pfnEnableCpu = */           VMXR0EnableCpu,
    /* .pfnDisableCpu = */          VMXR0DisableCpu,
    /* .pfnInitVM = */              VMXR0InitVM,
    /* .pfnTermVM = */              VMXR0TermVM,
    /* .pfnSetupVM = */             VMXR0SetupVM,
};

/** HM ring-0 operations for AMD-V. */
static HMR0VTABLE const g_HmR0OpsSvm =
{
    /* .pfnEnterSession = */        SVMR0Enter,
    /* .pfnThreadCtxCallback = */   SVMR0ThreadCtxCallback,
    /* .pfnAssertionCallback = */   SVMR0AssertionCallback,
    /* .pfnExportHostState = */     SVMR0ExportHostState,
    /* .pfnRunGuestCode = */        SVMR0RunGuestCode,
    /* .pfnEnableCpu = */           SVMR0EnableCpu,
    /* .pfnDisableCpu = */          SVMR0DisableCpu,
    /* .pfnInitVM = */              SVMR0InitVM,
    /* .pfnTermVM = */              SVMR0TermVM,
    /* .pfnSetupVM = */             SVMR0SetupVM,
};


/** @name Dummy callback handlers for when neither VT-x nor AMD-V is supported.
 * @{ */

static DECLCALLBACK(int) hmR0DummyEnter(PVMCPUCC pVCpu)
{
    RT_NOREF(pVCpu);
    return VINF_SUCCESS;
}

static DECLCALLBACK(void) hmR0DummyThreadCtxCallback(RTTHREADCTXEVENT enmEvent, PVMCPUCC pVCpu, bool fGlobalInit)
{
    RT_NOREF(enmEvent, pVCpu, fGlobalInit);
}

static DECLCALLBACK(int) hmR0DummyEnableCpu(PHMPHYSCPU pHostCpu, PVMCC pVM, void *pvCpuPage, RTHCPHYS HCPhysCpuPage,
                                            bool fEnabledBySystem, PCSUPHWVIRTMSRS pHwvirtMsrs)
{
    RT_NOREF(pHostCpu, pVM, pvCpuPage, HCPhysCpuPage, fEnabledBySystem, pHwvirtMsrs);
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) hmR0DummyDisableCpu(PHMPHYSCPU pHostCpu, void *pvCpuPage, RTHCPHYS HCPhysCpuPage)
{
    RT_NOREF(pHostCpu, pvCpuPage, HCPhysCpuPage);
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) hmR0DummyInitVM(PVMCC pVM)
{
    RT_NOREF(pVM);
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) hmR0DummyTermVM(PVMCC pVM)
{
    RT_NOREF(pVM);
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) hmR0DummySetupVM(PVMCC pVM)
{
    RT_NOREF(pVM);
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) hmR0DummyAssertionCallback(PVMCPUCC pVCpu)
{
    RT_NOREF(pVCpu);
    return VINF_SUCCESS;
}

static DECLCALLBACK(VBOXSTRICTRC) hmR0DummyRunGuestCode(PVMCPUCC pVCpu)
{
    RT_NOREF(pVCpu);
    return VERR_NOT_SUPPORTED;
}

static DECLCALLBACK(int) hmR0DummyExportHostState(PVMCPUCC pVCpu)
{
    RT_NOREF(pVCpu);
    return VINF_SUCCESS;
}

/** Dummy ops.    */
static HMR0VTABLE const g_HmR0OpsDummy =
{
    /* .pfnEnterSession = */        hmR0DummyEnter,
    /* .pfnThreadCtxCallback = */   hmR0DummyThreadCtxCallback,
    /* .pfnAssertionCallback = */   hmR0DummyAssertionCallback,
    /* .pfnExportHostState = */     hmR0DummyExportHostState,
    /* .pfnRunGuestCode = */        hmR0DummyRunGuestCode,
    /* .pfnEnableCpu = */           hmR0DummyEnableCpu,
    /* .pfnDisableCpu = */          hmR0DummyDisableCpu,
    /* .pfnInitVM = */              hmR0DummyInitVM,
    /* .pfnTermVM = */              hmR0DummyTermVM,
    /* .pfnSetupVM = */             hmR0DummySetupVM,
};

/** @} */


/**
 * Initializes a first return code structure.
 *
 * @param   pFirstRc            The structure to init.
 */
static void hmR0FirstRcInit(PHMR0FIRSTRC pFirstRc)
{
    pFirstRc->rc    = VINF_SUCCESS;
    pFirstRc->idCpu = NIL_RTCPUID;
}


/**
 * Try set the status code (success ignored).
 *
 * @param   pFirstRc            The first return code structure.
 * @param   rc                  The status code.
 */
static void hmR0FirstRcSetStatus(PHMR0FIRSTRC pFirstRc, int rc)
{
    if (   RT_FAILURE(rc)
        && ASMAtomicCmpXchgS32(&pFirstRc->rc, rc, VINF_SUCCESS))
        pFirstRc->idCpu = RTMpCpuId();
}


/**
 * Get the status code of a first return code structure.
 *
 * @returns The status code; VINF_SUCCESS or error status, no informational or
 *          warning errors.
 * @param   pFirstRc            The first return code structure.
 */
static int hmR0FirstRcGetStatus(PHMR0FIRSTRC pFirstRc)
{
    return pFirstRc->rc;
}


#ifdef VBOX_STRICT
# ifndef DEBUG_bird
/**
 * Get the CPU ID on which the failure status code was reported.
 *
 * @returns The CPU ID, NIL_RTCPUID if no failure was reported.
 * @param   pFirstRc            The first return code structure.
 */
static RTCPUID hmR0FirstRcGetCpuId(PHMR0FIRSTRC pFirstRc)
{
    return pFirstRc->idCpu;
}
# endif
#endif /* VBOX_STRICT */


/**
 * Verify if VMX is really usable by entering and exiting VMX root mode.
 *
 * @returns VBox status code.
 * @param   uVmxBasicMsr    The host's IA32_VMX_BASIC_MSR value.
 */
//验证当前CPU能否安全进入和退出Intel VT-x（VMX）根模式
/*
    硬件支持性：VMX指令是否可用
    环境冲突：是否已被其他虚拟化软件（如KVM）占用
    内存/寄存器状态：VMXON区域配置是否正确
*/
static int hmR0InitIntelVerifyVmxUsability(uint64_t uVmxBasicMsr)
{
    /* Allocate a temporary VMXON region. */
    RTR0MEMOBJ hScatchMemObj;
    //VT-x规范要求VMXON区域必须位于4KB对齐的物理连续内存
    //安全考虑（false参数），防止该区域被误执行为代码。
    int rc = RTR0MemObjAllocCont(&hScatchMemObj, HOST_PAGE_SIZE, NIL_RTHCPHYS /* PhysHighest */, false /* fExecutable */);//分配临时VMXON区域
    if (RT_FAILURE(rc))
    {
        LogRelFunc(("RTR0MemObjAllocCont(,HOST_PAGE_SIZE,false) -> %Rrc\n", rc));
        return rc;
    }

    //设置VMXON区域头
    void          *pvScatchPage      = RTR0MemObjAddress(hScatchMemObj);
    RTHCPHYS const HCPhysScratchPage = RTR0MemObjGetPagePhysAddr(hScatchMemObj, 0);
    RT_BZERO(pvScatchPage, HOST_PAGE_SIZE);

    /* Set revision dword at the beginning of the VMXON structure. */
    //uVmxBasicMsr来源：MSR_IA32_VMX_BASIC（通过CPUID读取）
    //VMX_BF_BASIC_VMCS_ID是VMCS修订标识符，必须写入VMXON区域首4字节。
    *(uint32_t *)pvScatchPage = RT_BF_GET(uVmxBasicMsr, VMX_BF_BASIC_VMCS_ID);

    /* Make sure we don't get rescheduled to another CPU during this probe. */
    RTCCUINTREG const fEFlags = ASMIntDisableFlags(); // 关闭中断

    /* Enable CR4.VMXE if it isn't already set. */
    //若原未启用（!(uOldCr4 & X86_CR4_VMXE)），后续必须恢复，否则可能导致宿主系统崩溃。
    RTCCUINTREG const uOldCr4 = SUPR0ChangeCR4(X86_CR4_VMXE, RTCCUINTREG_MAX);// 启用CR4.VMXE

    /*
     * The only way of checking if we're in VMX root mode is to try and enter it.
     * There is no instruction or control bit that tells us if we're in VMX root mode.
     * Therefore, try and enter and exit VMX root mode.
     */
    //VMXON区域仅存在数微秒，操作后立即释放资源。
    rc = VMXEnable(HCPhysScratchPage);// 执行VMXON指令
    if (RT_SUCCESS(rc))
        VMXDisable(); // 执行VMXOFF指令
    else
    {
        /*
         * KVM leaves the CPU in VMX root mode. Not only is this not allowed,
         * it will crash the host when we enter raw mode, because:
         *
         *   (a) clearing X86_CR4_VMXE in CR4 causes a #GP (we no longer modify
         *       this bit), and
         *   (b) turning off paging causes a #GP  (unavoidable when switching
         *       from long to 32 bits mode or 32 bits to PAE).
         *
         * They should fix their code, but until they do we simply refuse to run.
         */
        /*
            当CPU处于VMX根模式时：
            清除CR4.VMXE会触发#GP异常（违反Intel SDM规则）
            关闭分页（如切换到实模式）也会触发#GP
            直接拒绝启动（而非强制清理，避免宿主崩溃）。
        */
        rc = VERR_VMX_IN_VMX_ROOT_MODE;
    }

    /* Restore CR4.VMXE if it wasn't set prior to us setting it above. */
    if (!(uOldCr4 & X86_CR4_VMXE))
        SUPR0ChangeCR4(0 /* fOrMask */, ~(uint64_t)X86_CR4_VMXE);

    /* Restore interrupts. */
    ASMSetFlags(fEFlags);

    RTR0MemObjFree(hScatchMemObj, false);

    return rc;
}


/**
 * Worker function used by hmR0PowerCallback() and HMR0Init() to initalize VT-x
 * on a CPU.
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     Pointer to the first RC structure.
 * @param   pvUser2     Ignored.
 */
static DECLCALLBACK(void) hmR0InitIntelCpu(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    PHMR0FIRSTRC pFirstRc = (PHMR0FIRSTRC)pvUser1;
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(idCpu == (RTCPUID)RTMpCpuIdToSetIndex(idCpu)); /** @todo fix idCpu == index assumption (rainy day) */
    NOREF(idCpu); NOREF(pvUser2);

    int rc = SUPR0GetVmxUsability(NULL /* pfIsSmxModeAmbiguous */);
    hmR0FirstRcSetStatus(pFirstRc, rc);
}


/**
 * Intel specific initialization code.
 *
 * @returns VBox status code (will only fail if out of memory).
 */
/*
    检测并启用 VT-x 支持通过主机OS接口或直接操作CPU）。
    读取关键 CPU 寄存器与 MSR如 CR4、EFER、VMX_BASIC）。
    验证 VT-x 可用性检查 VMX 根模式能否正常进入）。
    初始化全局 VT-x 状态如 VPID、抢占定时器、EFER 交换支持）。
    安装 VT-x 操作函数集替换默认的 g_HmR0Ops）。
 * */
static int hmR0InitIntel(void)
{
    /* Read this MSR now as it may be useful for error reporting when initializing VT-x fails. */
    g_HmMsrs.u.vmx.u64FeatCtrl = ASMRdMsr(MSR_IA32_FEATURE_CONTROL);

    /*
     * First try use native kernel API for controlling VT-x.
     * (This is only supported by some Mac OS X kernels atm.)
     */
    int rc;
    g_rcHmInit = rc = SUPR0EnableVTx(true /* fEnable */);//调用宿主内核接口尝试启用 VT-x。
    g_fHmVmxUsingSUPR0EnableVTx = rc != VERR_NOT_SUPPORTED;//成功（VINF_SUCCESS）：标记 g_fHmVmxUsingSUPR0EnableVTx = true，后续依赖宿主OS管理。
    if (g_fHmVmxUsingSUPR0EnableVTx)
    {
        AssertLogRelMsg(rc == VINF_SUCCESS || rc == VERR_VMX_IN_VMX_ROOT_MODE || rc == VERR_VMX_NO_VMX, ("%Rrc\n", rc));
        if (RT_SUCCESS(rc))
        {
            g_fHmVmxSupported = true;
            rc = SUPR0EnableVTx(false /* fEnable */);
            AssertLogRelRC(rc);
            rc = VINF_SUCCESS;
        }
    }
    else
    {
        HMR0FIRSTRC FirstRc;
        hmR0FirstRcInit(&FirstRc);
        /*
            hmR0InitIntelCpu的职责：
              执行 VMXON 指令进入 VMX 根模式。
              检查 BIOS 是否启用 VT-x（MSR_IA32_FEATURE_CONTROL）。
              返回 VERR_VMX_NO_VMX（硬件不支持）或 VERR_VMX_IN_VMX_ROOT_MODE（已被占用）。
        */
        g_rcHmInit = rc = RTMpOnAll(hmR0InitIntelCpu, &FirstRc, NULL); // 在所有CPU核上初始化VT-x
        if (RT_SUCCESS(rc))
            g_rcHmInit = rc = hmR0FirstRcGetStatus(&FirstRc); // 获取首个错误
    }

    if (RT_SUCCESS(rc))
    {
        /* Read CR4 and EFER for logging/diagnostic purposes. */
        g_uHmVmxHostCr0     = ASMGetCR0();
        g_uHmVmxHostCr4     = ASMGetCR4();// 读取CR4（含VMXE标志）
        g_uHmVmxHostMsrEfer = ASMRdMsr(MSR_K6_EFER)// 读取EFER MSR

        /* Get VMX MSRs (and feature control MSR) for determining VMX features we can ultimately use. */
        SUPR0GetHwvirtMsrs(&g_HmMsrs, SUPVTCAPS_VT_X, false /* fForce */);// 获取VMX相关MSR

        /*
         * Nested KVM workaround: Intel SDM section 34.15.5 describes that
         * MSR_IA32_SMM_MONITOR_CTL depends on bit 49 of MSR_IA32_VMX_BASIC while
         * table 35-2 says that this MSR is available if either VMX or SMX is supported.
         */
        uint64_t const uVmxBasicMsr = g_HmMsrs.u.vmx.u64Basic;//确定 VMCS 字段宽度、支持的功能。
        if (RT_BF_GET(uVmxBasicMsr, VMX_BF_BASIC_DUAL_MON))
            g_uHmVmxHostSmmMonitorCtl = ASMRdMsr(MSR_IA32_SMM_MONITOR_CTL);

        /*
         * Host core and memory capabilities MSRs.
         * Primarily for logging split-lock disable status.
         */
        uint32_t uDummy, uStdExtFeatEdx;
        ASMCpuId_Idx_ECX(7, 0, &uDummy, &uDummy, &uDummy, &uStdExtFeatEdx);
        if (uStdExtFeatEdx & X86_CPUID_STEXT_FEATURE_EDX_CORECAP)
        {
            g_uHmVmxHostCoreCap = ASMRdMsr(MSR_IA32_CORE_CAPABILITIES);
            if (g_uHmVmxHostCoreCap & MSR_IA32_CORE_CAP_SPLIT_LOCK_DISABLE)
                g_uHmVmxHostMemoryCtrl = ASMRdMsr(MSR_MEMORY_CTRL);
        }

        /* Initialize VPID - 16 bits ASID. */
        g_uHmMaxAsid = 0x10000; /* exclusive */

        /*
         * If the host OS has not enabled VT-x for us, try enter VMX root mode
         * to really verify if VT-x is usable.
         */
        if (!g_fHmVmxUsingSUPR0EnableVTx)
        {
            /*
             * We don't verify VMX root mode on all CPUs here because the verify
             * function exits VMX root mode thus potentially allowing other
             * programs to grab VT-x. Our global init's entering and staying in
             * VMX root mode (until our module termination) is done later when
             * the first VM powers up (after module initialization) using
             * VMMR0_DO_HM_ENABLE which calls HMR0EnableAllCpus().
             *
             * This is just a quick sanity check.
             */
            //临时进入 VMX 根模式，验证是否能正常执行 VMLAUNCH。验证后立即退出，避免长期占用 VT-x。
            rc = hmR0InitIntelVerifyVmxUsability(uVmxBasicMsr);
            if (RT_SUCCESS(rc))
                g_fHmVmxSupported = true;
            else
            {
                g_rcHmInit = rc;
                Assert(g_fHmVmxSupported == false);
            }
        }

        if (g_fHmVmxSupported)
        {
            rc = VMXR0GlobalInit(); // 初始化VMX全局资源
            if (RT_SUCCESS(rc))
            {
                /*
                 * Install the VT-x methods.
                 */
                /*
                  pfnEnterCpu → VMXEnableCpu
                  pfnExitCpu → VMXDisableCpu
                  pfnRunVm → VMXR0RunGuest
                */
                g_HmR0Ops = g_HmR0OpsVmx;// 绑定VT-x操作函数

                /*
                 * Check for the VMX-Preemption Timer and adjust for the "VMX-Preemption
                 * Timer Does Not Count Down at the Rate Specified" CPU erratum.
                 */
                // 抢占定时器校准
                if (g_HmMsrs.u.vmx.PinCtls.n.allowed1 & VMX_PIN_CTLS_PREEMPT_TIMER)
                {
                    g_fHmVmxUsePreemptTimer   = true;
                    g_cHmVmxPreemptTimerShift = RT_BF_GET(g_HmMsrs.u.vmx.u64Misc, VMX_BF_MISC_PREEMPT_TIMER_TSC);
                    if (HMIsSubjectToVmxPreemptTimerErratum())
                        g_cHmVmxPreemptTimerShift = 0; /* This is about right most of the time here. */// 根据CPU勘误调整
                }
                else
                    g_fHmVmxUsePreemptTimer   = false;

                /*
                 * Check for EFER swapping support.
                 */
                //EFER交换支持
                g_fHmVmxSupportsVmcsEfer = (g_HmMsrs.u.vmx.EntryCtls.n.allowed1 & VMX_ENTRY_CTLS_LOAD_EFER_MSR)
                                        && (g_HmMsrs.u.vmx.ExitCtls.n.allowed1  & VMX_EXIT_CTLS_LOAD_EFER_MSR)
                                        && (g_HmMsrs.u.vmx.ExitCtls.n.allowed1  & VMX_EXIT_CTLS_SAVE_EFER_MSR);
            }
            else
            {
                g_rcHmInit = rc;
                g_fHmVmxSupported = false;
            }
        }
    }
#ifdef LOG_ENABLED
    else
        SUPR0Printf("hmR0InitIntelCpu failed with rc=%Rrc\n", g_rcHmInit);
#endif
    return VINF_SUCCESS;
}


/**
 * Worker function used by hmR0PowerCallback() and HMR0Init() to initalize AMD-V
 * on a CPU.
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     Pointer to the first RC structure.
 * @param   pvUser2     Ignored.
 */
static DECLCALLBACK(void) hmR0InitAmdCpu(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    PHMR0FIRSTRC pFirstRc = (PHMR0FIRSTRC)pvUser1;
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(idCpu == (RTCPUID)RTMpCpuIdToSetIndex(idCpu)); /** @todo fix idCpu == index assumption (rainy day) */
    NOREF(idCpu); NOREF(pvUser2);

    int rc = SUPR0GetSvmUsability(true /* fInitSvm */);
    hmR0FirstRcSetStatus(pFirstRc, rc);
}


/**
 * AMD-specific initialization code.
 *
 * @returns VBox status code (will only fail if out of memory).
 */
/*
    全局 AMD-V 初始化调用 SVMR0GlobalInit）。
    安装 AMD-V 操作函数集替换默认的 g_HmR0Ops）。
    检测 CPU 的 AMD-V 特性通过 CPUID 指令）。
    验证所有 CPU 核的 AMD-V 状态防止 BIOS 配置错误）。
    处理错误降级（如 AMD-V 被禁用或被其他程序占用）。 
*/
static int hmR0InitAmd(void)
{
    /* Call the global AMD-V initialization routine (should only fail in out-of-memory situations). */
    int rc = SVMR0GlobalInit();
    if (RT_SUCCESS(rc))
    {
        /*
         * Install the AMD-V methods.
         */
        g_HmR0Ops = g_HmR0OpsSvm;

        /* Query AMD features. */
        uint32_t u32Dummy;
        ASMCpuId(0x8000000a, &g_uHmSvmRev, &g_uHmMaxAsid, &u32Dummy, &g_fHmSvmFeatures);

        /*
         * We need to check if AMD-V has been properly initialized on all CPUs.
         * Some BIOSes might do a poor job.
         */
        HMR0FIRSTRC FirstRc;
        hmR0FirstRcInit(&FirstRc);
        rc = RTMpOnAll(hmR0InitAmdCpu, &FirstRc, NULL);
        AssertRC(rc);
        if (RT_SUCCESS(rc))
            rc = hmR0FirstRcGetStatus(&FirstRc);
#ifndef DEBUG_bird
        AssertMsg(rc == VINF_SUCCESS || rc == VERR_SVM_IN_USE,
                  ("hmR0InitAmdCpu failed for cpu %d with rc=%Rrc\n", hmR0FirstRcGetCpuId(&FirstRc), rc));
#endif
        if (RT_SUCCESS(rc))
        {
            SUPR0GetHwvirtMsrs(&g_HmMsrs, SUPVTCAPS_AMD_V, false /* fForce */);
            g_fHmSvmSupported = true;
        }
        else
        {
            g_rcHmInit = rc;
            if (rc == VERR_SVM_DISABLED || rc == VERR_SVM_IN_USE)
                rc = VINF_SUCCESS; /* Don't fail if AMD-V is disabled or in use. */
        }
    }
    else
        g_rcHmInit = rc;
    return rc;
}


/**
 * Does global Ring-0 HM initialization (at module init).
 *
 * @returns VBox status code.
 */
/*
    HMR0Init 是 VirtualBox 硬件虚拟化模块（HM）的全局初始化函数，负责在 Ring-0（内核态）完成以下核心任务：

    初始化全局状态（如 CPU 信息数组、虚拟化支持标志）。
    检测硬件虚拟化能力（Intel VT-x 或 AMD-V）。
    注册系统回调（CPU 热插拔、电源事件）。
    架构特定初始化（调用 hmR0InitIntel 或 hmR0InitAmd）。
*/
VMMR0_INT_DECL(int) HMR0Init(void)
{
    /*
     * Initialize the globals.
     */
    g_fHmEnabled = false;// 标记虚拟化模块未启用
    for (unsigned i = 0; i < RT_ELEMENTS(g_aHmCpuInfo); i++)
    {
        g_aHmCpuInfo[i].idCpu        = NIL_RTCPUID; // 重置CPU ID
        g_aHmCpuInfo[i].hMemObj      = NIL_RTR0MEMOBJ;// 释放内存对象
        g_aHmCpuInfo[i].HCPhysMemObj = NIL_RTHCPHYS;
        g_aHmCpuInfo[i].pvMemObj     = NULL;
#ifdef VBOX_WITH_NESTED_HWVIRT_SVM
        g_aHmCpuInfo[i].n.svm.hNstGstMsrpm      = NIL_RTR0MEMOBJ;
        g_aHmCpuInfo[i].n.svm.HCPhysNstGstMsrpm = NIL_RTHCPHYS;
        g_aHmCpuInfo[i].n.svm.pvNstGstMsrpm     = NULL;
#endif
    }

    /* Fill in all callbacks with placeholders. */
    g_HmR0Ops          = g_HmR0OpsDummy; // 设置虚拟化操作为空实现

    /* Default is global VT-x/AMD-V init. */
    g_fHmGlobalInit    = true;// 标记全局初始化开始

    g_fHmVmxSupported  = false;
    g_fHmSvmSupported  = false;
    g_uHmMaxAsid       = 0;

    /*
     * Get host kernel features that HM might need to know in order
     * to co-operate and function properly with the host OS (e.g. SMAP).
     */
    g_fHmHostKernelFeatures = SUPR0GetKernelFeatures();//获取宿主内核特性（如 SMAP/SMEP），用于后续虚拟化配置的兼容性处理。

    /*
     * Make sure aCpuInfo is big enough for all the CPUs on this system.
     */
    if (RTMpGetArraySize() > RT_ELEMENTS(g_aHmCpuInfo))
    {
        LogRel(("HM: Too many real CPUs/cores/threads - %u, max %u\n", RTMpGetArraySize(), RT_ELEMENTS(g_aHmCpuInfo)));
        return VERR_TOO_MANY_CPUS;
    }

    /*
     * Check for VT-x or AMD-V support.
     * Return failure only in out-of-memory situations.
     */
    uint32_t fCaps = 0;
    int rc = SUPR0GetVTSupport(&fCaps); // 调用底层接口检测VT-x/AMD-V
    if (RT_SUCCESS(rc))
    {
        if (fCaps & SUPVTCAPS_VT_X)
            rc = hmR0InitIntel(); // 初始化Intel VT-x
        else
        {
            Assert(fCaps & SUPVTCAPS_AMD_V);
            rc = hmR0InitAmd();// 初始化AMD-V
        }
        if (RT_SUCCESS(rc))
        {
            /*
             * Register notification callbacks that we can use to disable/enable CPUs
             * when brought offline/online or suspending/resuming.
             */
            if (!g_fHmVmxUsingSUPR0EnableVTx)
            {
                rc = RTMpNotificationRegister(hmR0MpEventCallback, NULL);// CPU热插拔回调
                if (RT_SUCCESS(rc))
                {
                    rc = RTPowerNotificationRegister(hmR0PowerCallback, NULL);// 电源事件回调,响应系统休眠/唤醒事件
                    if (RT_FAILURE(rc))
                        RTMpNotificationDeregister(hmR0MpEventCallback, NULL);
                }
                if (RT_FAILURE(rc))
                {
                    /* There shouldn't be any per-cpu allocations at this point,
                       so just have to call SVMR0GlobalTerm and VMXR0GlobalTerm. */
                    if (fCaps & SUPVTCAPS_VT_X)
                        VMXR0GlobalTerm();// 清理VT-x资源
                    else
                        SVMR0GlobalTerm();// 清理AMD-V资源
                    g_HmR0Ops         = g_HmR0OpsDummy;// 重置操作为空实现
                    g_rcHmInit     = rc;// 保存错误码
                    g_fHmSvmSupported = false;
                    g_fHmVmxSupported = false;
                }
            }
        }
    }
    else
    {
        g_rcHmInit = rc;
        rc = VINF_SUCCESS; /* We return success here because module init shall not fail if HM fails to initialize. */
    }
    return rc;
}


/**
 * Does global Ring-0 HM termination (at module termination).
 *
 * @returns VBox status code (ignored).
 */
/*
    全局终止函数，负责清理硬件虚拟化（VT-x/AMD-V）相关的所有资源，包括：
        禁用 CPU 核的虚拟化功能如执行 VMXOFF）
        释放内存资源（如 VMXON 区域、VMCB 结构）
        注销系统回调（如 CPU 热插拔、电源事件）
        调用架构特定的终止逻辑（Intel VT-x 或 AMD SVM）
 */
VMMR0_INT_DECL(int) HMR0Term(void)
{
    int rc;
    if (   g_fHmVmxSupported
        && g_fHmVmxUsingSUPR0EnableVTx)//当 VirtualBox ‌依赖主机OS‌（如 macOS 的 vmm.kext）管理 VT-x 时。
    {
        /*
         * Simple if the host OS manages VT-x.
         */
        Assert(g_fHmGlobalInit);

        if (g_fHmVmxCalledSUPR0EnableVTx)
        {
            rc = SUPR0EnableVTx(false /* fEnable */);//禁用 VT-x
            g_fHmVmxCalledSUPR0EnableVTx = false;//重置 CPU 状态
        }
        else
            rc = VINF_SUCCESS;

        for (unsigned iCpu = 0; iCpu < RT_ELEMENTS(g_aHmCpuInfo); iCpu++)
        {
            g_aHmCpuInfo[iCpu].fConfigured = false;
            Assert(g_aHmCpuInfo[iCpu].hMemObj == NIL_RTR0MEMOBJ);//确保内存对象已释放
        }
    }
    else//默认路径，VirtualBox 自行管理 VT-x/AMD-V 资源。
    {
        Assert(!g_fHmVmxSupported || !g_fHmVmxUsingSUPR0EnableVTx);

        /* Doesn't really matter if this fails. */
        RTMpNotificationDeregister(hmR0MpEventCallback, NULL);// CPU热插拔
        RTPowerNotificationDeregister(hmR0PowerCallback, NULL);// 电源事件
        rc = VINF_SUCCESS;

        /*
         * Disable VT-x/AMD-V on all CPUs if we enabled it before.
         */
        if (g_fHmGlobalInit)
        {
            HMR0FIRSTRC FirstRc;
            hmR0FirstRcInit(&FirstRc);
            /*
                通过 RTMpOnAll 在所有 CPU 核上调用 hmR0DisableCpuCallback。
                使用 HMR0FIRSTRC 记录首个错误（多核同步）。
            */
            rc = RTMpOnAll(hmR0DisableCpuCallback, NULL /* pvUser 1 */, &FirstRc);
            Assert(RT_SUCCESS(rc) || rc == VERR_NOT_SUPPORTED);
            if (RT_SUCCESS(rc))
                rc = hmR0FirstRcGetStatus(&FirstRc);
        }

        /*
         * Free the per-cpu pages used for VT-x and AMD-V.
         */
        //释放对象：
        //VT-x 的 VMXON 区域或 AMD-V 的 VMCB 结构。
        //嵌套虚拟化相关资源（如 SVM 的 MSRPM 页）。
        for (unsigned i = 0; i < RT_ELEMENTS(g_aHmCpuInfo); i++)
        {
            if (g_aHmCpuInfo[i].hMemObj != NIL_RTR0MEMOBJ)
            {
                RTR0MemObjFree(g_aHmCpuInfo[i].hMemObj, false);
                g_aHmCpuInfo[i].hMemObj      = NIL_RTR0MEMOBJ;
                g_aHmCpuInfo[i].HCPhysMemObj = NIL_RTHCPHYS;
                g_aHmCpuInfo[i].pvMemObj     = NULL;
            }
#ifdef VBOX_WITH_NESTED_HWVIRT_SVM
            if (g_aHmCpuInfo[i].n.svm.hNstGstMsrpm != NIL_RTR0MEMOBJ)
            {
                RTR0MemObjFree(g_aHmCpuInfo[i].n.svm.hNstGstMsrpm, false);
                g_aHmCpuInfo[i].n.svm.hNstGstMsrpm      = NIL_RTR0MEMOBJ;
                g_aHmCpuInfo[i].n.svm.HCPhysNstGstMsrpm = NIL_RTHCPHYS;
                g_aHmCpuInfo[i].n.svm.pvNstGstMsrpm     = NULL;
            }
#endif
        }
    }

    /** @todo This needs cleaning up. There's no matching
     *        hmR0TermIntel()/hmR0TermAmd() and all the VT-x/AMD-V specific bits
     *        should move into their respective modules. */
    /* Finally, call global VT-x/AMD-V termination. */
    if (g_fHmVmxSupported)
        VMXR0GlobalTerm();
    else if (g_fHmSvmSupported)
        SVMR0GlobalTerm();

    return rc;
}


/**
 * Enable VT-x or AMD-V on the current CPU
 *
 * @returns VBox status code.
 * @param   pVM     The cross context VM structure. Can be NULL.
 * @param   idCpu   The identifier for the CPU the function is called on.
 *
 * @remarks Maybe called with interrupts disabled!
 */
static int hmR0EnableCpu(PVMCC pVM, RTCPUID idCpu)
{
    PHMPHYSCPU pHostCpu = &g_aHmCpuInfo[idCpu];// 获取当前CPU核的硬件虚拟化状态结构体

    Assert(idCpu == (RTCPUID)RTMpCpuIdToSetIndex(idCpu)); /** @todo fix idCpu == index assumption (rainy day) */// 确保CPU逻辑ID与索引一致
    Assert(idCpu < RT_ELEMENTS(g_aHmCpuInfo));// 防止数组越界
    Assert(!pHostCpu->fConfigured);// 确保当前CPU未配置过虚拟化
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));// 必须在非抢占上下文中执行

    pHostCpu->idCpu = idCpu;// 记录当前CPU核ID
    /* Do NOT reset cTlbFlushes here, see @bugref{6255}. */

    int rc;
    if (   g_fHmVmxSupported
        && g_fHmVmxUsingSUPR0EnableVTx)
        // 情况1：使用SUPR0模块直接启用VT-x（依赖主机OS支持）
        // 手动路径需确保内存对象有效（hMemObj != NIL_RTR0MEMOBJ）
        rc = g_HmR0Ops.pfnEnableCpu(pHostCpu, pVM, NULL /* pvCpuPage */, NIL_RTHCPHYS, true, &g_HmMsrs);
    else
    {
        //情况2：手动配置虚拟化资源（默认路径）
        AssertLogRelMsgReturn(pHostCpu->hMemObj != NIL_RTR0MEMOBJ, ("hmR0EnableCpu failed idCpu=%u.\n", idCpu), VERR_HM_IPE_1);
        rc = g_HmR0Ops.pfnEnableCpu(pHostCpu, pVM, pHostCpu->pvMemObj, pHostCpu->HCPhysMemObj, false, &g_HmMsrs);
    }
    if (RT_SUCCESS(rc))
        pHostCpu->fConfigured = true;// 标记当前CPU已配置虚拟化
    return rc;
}


/**
 * Worker function passed to RTMpOnAll() that is to be called on all CPUs.
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     Opaque pointer to the VM (can be NULL!).
 * @param   pvUser2     The 2nd user argument.
 */
//在指定 CPU 核上启用硬件虚拟化功
//pvUser2: 用户参数 2，此处为多核同步结构体指针 PHMR0FIRSTRC。
static DECLCALLBACK(void) hmR0EnableCpuCallback(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    PVMCC           pVM      = (PVMCC)pvUser1;     /* can be NULL! */
    PHMR0FIRSTRC    pFirstRc = (PHMR0FIRSTRC)pvUser2;//多核同步结构体，用于记录首个 CPU 核的启用状态（其他核会跳过重复操作）。
    AssertReturnVoid(g_fHmGlobalInit); // 确保全局初始化已完成
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));// 禁止抢占,硬件虚拟化操作（如执行 VMXON）需要原子性，抢占可能导致状态不一致。
    /*
      hmR0EnableCpu(pVM, idCpu)
        实际启用目标 CPU 核的硬件虚拟化功能：
        配置 CPU 专属的虚拟化资源（如 VMXON 区域、VMCS）。
        执行硬件指令（如 VMXON 进入 VT-x 模式）。
        返回状态码（VINF_SUCCESS 或错误码）。
      hmR0FirstRcSetStatus
        将结果记录到 pFirstRc 结构体中：
        如果当前是首个执行的 CPU 核，保存其状态码。
        其他核直接复用首个核的结果（避免重复操作）。
    */
    hmR0FirstRcSetStatus(pFirstRc, hmR0EnableCpu(pVM, idCpu));
}


/**
 * RTOnce callback employed by HMR0EnableAllCpus.
 *
 * @returns VBox status code.
 * @param   pvUser          Pointer to the VM.
 */
//在所有 CPU 核上启用硬件虚拟化功能
/*
    初始化全局状态g_fHmEnabled、g_fHmGlobalInit）。
    检查 CPU 虚拟化支持VT-x/AMD-V）。
    分配和管理 CPU 虚拟化资源如 VMXON 区域、MSRPM 页）。
    处理嵌套虚拟化（Nested Virtualization）如 SVM 的 MSRPM 页）。
    错误处理和状态同步多核环境下的原子操作）。
*/
static DECLCALLBACK(int32_t) hmR0EnableAllCpuOnce(void *pvUser)
{
    PVMCC pVM = (PVMCC)pvUser;// 获取虚拟机控制块

    /*
     * Indicate that we've initialized.
     *
     * Note! There is a potential race between this function and the suspend
     *       notification.  Kind of unlikely though, so ignored for now.
     */
    AssertReturn(!g_fHmEnabled, VERR_HM_ALREADY_ENABLED_IPE);// 确保 HM 未启用
    ASMAtomicWriteBool(&g_fHmEnabled, true);// 原子操作标记 HM 已启用

    /*
     * The global init variable is set by the first VM.
     */
    g_fHmGlobalInit = pVM->hm.s.fGlobalInit;// 同步全局初始化标志

//仅在调试模式下启用，用于检查内部一致性。
#ifdef VBOX_STRICT
    for (unsigned i = 0; i < RT_ELEMENTS(g_aHmCpuInfo); i++)
    {
        Assert(g_aHmCpuInfo[i].hMemObj      == NIL_RTR0MEMOBJ);// 检查内存对象未分配
        Assert(g_aHmCpuInfo[i].HCPhysMemObj == NIL_RTHCPHYS);// 检查物理地址未设置
        Assert(g_aHmCpuInfo[i].pvMemObj     == NULL);// 检查虚拟地址未映射
        Assert(!g_aHmCpuInfo[i].fConfigured);// 检查 CPU 未配置
        Assert(!g_aHmCpuInfo[i].cTlbFlushes);// 检查 TLB 刷新计数为 0
        Assert(!g_aHmCpuInfo[i].uCurrentAsid);// 检查 ASID（地址空间 ID）未设置
# ifdef VBOX_WITH_NESTED_HWVIRT_SVM
        Assert(g_aHmCpuInfo[i].n.svm.hNstGstMsrpm      == NIL_RTR0MEMOBJ); // 嵌套虚拟化 MSRPM 页未分配
        Assert(g_aHmCpuInfo[i].n.svm.HCPhysNstGstMsrpm == NIL_RTHCPHYS); // 物理地址未设置
        Assert(g_aHmCpuInfo[i].n.svm.pvNstGstMsrpm     == NULL); // 虚拟地址未映射
# endif
    }
#endif

    int rc;
    if (   g_fHmVmxSupported
        && g_fHmVmxUsingSUPR0EnableVTx)
    {
        /*
         * Global VT-x initialization API (only darwin for now).
         */
        rc = SUPR0EnableVTx(true /* fEnable */);// 调用宿主系统的 VT-x 启用 API
        if (RT_SUCCESS(rc))
        {
            g_fHmVmxCalledSUPR0EnableVTx = true; // 标记已调用宿主 API
            /* If the host provides a VT-x init API, then we'll rely on that for global init. */
            g_fHmGlobalInit = pVM->hm.s.fGlobalInit = true; // 更新全局初始化标志
        }
        else
            AssertMsgFailed(("hmR0EnableAllCpuOnce/SUPR0EnableVTx: rc=%Rrc\n", rc));
    }
    else
    {
        /*
         * We're doing the job ourselves.
         */
        /* Allocate one page per cpu for the global VT-x and AMD-V pages */
        for (unsigned i = 0; i < RT_ELEMENTS(g_aHmCpuInfo); i++)
        {
            Assert(g_aHmCpuInfo[i].hMemObj == NIL_RTR0MEMOBJ);
#ifdef VBOX_WITH_NESTED_HWVIRT_SVM
            Assert(g_aHmCpuInfo[i].n.svm.hNstGstMsrpm == NIL_RTR0MEMOBJ);
#endif
            if (RTMpIsCpuPossible(RTMpCpuIdFromSetIndex(i)))// 检查 CPU 是否可用
            {
                /** @todo NUMA */
				//分配 VMXON/AMD-V 区域（每个 CPU 一页）
                rc = RTR0MemObjAllocCont(&g_aHmCpuInfo[i].hMemObj, HOST_PAGE_SIZE, NIL_RTHCPHYS /*PhysHighest*/, false /* executable R0 mapping */);
                AssertLogRelRCReturn(rc, rc);// 错误则终止

                g_aHmCpuInfo[i].HCPhysMemObj = RTR0MemObjGetPagePhysAddr(g_aHmCpuInfo[i].hMemObj, 0);
                Assert(g_aHmCpuInfo[i].HCPhysMemObj != NIL_RTHCPHYS);
                Assert(!(g_aHmCpuInfo[i].HCPhysMemObj & HOST_PAGE_OFFSET_MASK));

                g_aHmCpuInfo[i].pvMemObj     = RTR0MemObjAddress(g_aHmCpuInfo[i].hMemObj);// 获取物理地址
                AssertPtr(g_aHmCpuInfo[i].pvMemObj);// 获取虚拟地址
                RT_BZERO(g_aHmCpuInfo[i].pvMemObj, HOST_PAGE_SIZE);// 清零内存

#ifdef VBOX_WITH_NESTED_HWVIRT_SVM
                //分配嵌套虚拟化的 MSRPM 页（SVM 专用）
                rc = RTR0MemObjAllocCont(&g_aHmCpuInfo[i].n.svm.hNstGstMsrpm, SVM_MSRPM_PAGES << X86_PAGE_4K_SHIFT,
                                         NIL_RTHCPHYS /*PhysHighest*/, false /* executable R0 mapping */);
                AssertLogRelRCReturn(rc, rc);

                g_aHmCpuInfo[i].n.svm.HCPhysNstGstMsrpm = RTR0MemObjGetPagePhysAddr(g_aHmCpuInfo[i].n.svm.hNstGstMsrpm, 0);
                Assert(g_aHmCpuInfo[i].n.svm.HCPhysNstGstMsrpm != NIL_RTHCPHYS);
                Assert(!(g_aHmCpuInfo[i].n.svm.HCPhysNstGstMsrpm & HOST_PAGE_OFFSET_MASK));

                g_aHmCpuInfo[i].n.svm.pvNstGstMsrpm    = RTR0MemObjAddress(g_aHmCpuInfo[i].n.svm.hNstGstMsrpm);
                AssertPtr(g_aHmCpuInfo[i].n.svm.pvNstGstMsrpm);
                ASMMemFill32(g_aHmCpuInfo[i].n.svm.pvNstGstMsrpm, SVM_MSRPM_PAGES << X86_PAGE_4K_SHIFT, UINT32_C(0xffffffff)); // 初始化 MSRPM 页
#endif
            }
        }

        rc = VINF_SUCCESS;// 标记成功
    }

    if (   RT_SUCCESS(rc)
        && g_fHmGlobalInit)
    {
        /*
         * It's possible we end up here with VMX (and perhaps SVM) not supported, see @bugref{9918}.
         * In that case, our HMR0 function table contains the dummy placeholder functions which pretend
         * success. However, we must not pretend success any longer (like we did during HMR0Init called
         * during VMMR0 module init) as the HM init error code (g_rcHmInit) should be propagated to
         * ring-3 especially since we now have a VM instance.
         */
        if (   !g_fHmVmxSupported
            && !g_fHmSvmSupported) // 检查是否无硬件虚拟化支持
        {
            Assert(g_HmR0Ops.pfnEnableCpu == hmR0DummyEnableCpu); // 确认使用的是空操作函数
            Assert(RT_FAILURE(g_rcHmInit));// 确认初始化已失败
            rc = g_rcHmInit;// 返回错误码
        }
        else
        {
            /* First time, so initialize each cpu/core. */
            HMR0FIRSTRC FirstRc;
            hmR0FirstRcInit(&FirstRc);// 初始化多核同步结构体
            Assert(g_HmR0Ops.pfnEnableCpu != hmR0DummyEnableCpu);
            // 后续逻辑：在所有 CPU 上执行实际启用操作（如 VMXON）
            rc = RTMpOnAll(hmR0EnableCpuCallback, (void *)pVM, &FirstRc);
            if (RT_SUCCESS(rc))
                rc = hmR0FirstRcGetStatus(&FirstRc);
        }
    }

    return rc;
}


/**
 * Sets up HM on all cpus.
 *
 * @returns VBox status code.
 * @param   pVM                 The cross context VM structure.
 */
//用于在所有 CPU 核上启用硬件虚拟化功能
//PVMCC pVM
//指向虚拟机控制块（VM Control Context）的指针，包含虚拟机的运行时状态。
VMMR0_INT_DECL(int) HMR0EnableAllCpus(PVMCC pVM)
{
    /* Make sure we don't touch HM after we've disabled HM in preparation of a suspend. */
    if (ASMAtomicReadBool(&g_fHmSuspended))
        return VERR_HM_SUSPEND_PENDING;

	//RTOnce: VirtualBox 的一次性执行机制，确保 hmR0EnableAllCpuOnce 函数仅在所有 CPU 上执行一次。
	//g_HmEnableAllCpusOnce全局控制结构，用于同步多核初始化。
	//hmR0EnableAllCpuOnce
    //实际执行函数，负责在每个 CPU 核上启用虚拟化（如执行 VMXON 指令）。
    return RTOnce(&g_HmEnableAllCpusOnce, hmR0EnableAllCpuOnce, pVM);
}


/**
 * Disable VT-x or AMD-V on the current CPU.
 *
 * @returns VBox status code.
 * @param   idCpu       The identifier for the CPU this function is called on.
 *
 * @remarks Must be called with preemption disabled.
 */
static int hmR0DisableCpu(RTCPUID idCpu)
{
    PHMPHYSCPU pHostCpu = &g_aHmCpuInfo[idCpu];

    Assert(!g_fHmVmxSupported || !g_fHmVmxUsingSUPR0EnableVTx);
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(idCpu == (RTCPUID)RTMpCpuIdToSetIndex(idCpu)); /** @todo fix idCpu == index assumption (rainy day) */
    Assert(idCpu < RT_ELEMENTS(g_aHmCpuInfo));
    Assert(!pHostCpu->fConfigured || pHostCpu->hMemObj != NIL_RTR0MEMOBJ);
    AssertRelease(idCpu == RTMpCpuId());

    if (pHostCpu->hMemObj == NIL_RTR0MEMOBJ)
        return pHostCpu->fConfigured ? VERR_NO_MEMORY : VINF_SUCCESS /* not initialized. */;
    AssertPtr(pHostCpu->pvMemObj);
    Assert(pHostCpu->HCPhysMemObj != NIL_RTHCPHYS);

    int rc;
    if (pHostCpu->fConfigured)
    {
        rc = g_HmR0Ops.pfnDisableCpu(pHostCpu, pHostCpu->pvMemObj, pHostCpu->HCPhysMemObj);
        AssertRCReturn(rc, rc);

        pHostCpu->fConfigured = false;
        pHostCpu->idCpu = NIL_RTCPUID;
    }
    else
        rc = VINF_SUCCESS; /* nothing to do */
    return rc;
}


/**
 * Worker function passed to RTMpOnAll() that is to be called on the target
 * CPUs.
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     The 1st user argument.
 * @param   pvUser2     Opaque pointer to the FirstRc.
 */
static DECLCALLBACK(void) hmR0DisableCpuCallback(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    PHMR0FIRSTRC pFirstRc = (PHMR0FIRSTRC)pvUser2; NOREF(pvUser1);
    AssertReturnVoid(g_fHmGlobalInit);
    hmR0FirstRcSetStatus(pFirstRc, hmR0DisableCpu(idCpu));
}


/**
 * Worker function passed to RTMpOnSpecific() that is to be called on the target
 * CPU.
 *
 * @param   idCpu       The identifier for the CPU the function is called on.
 * @param   pvUser1     Null, not used.
 * @param   pvUser2     Null, not used.
 */
static DECLCALLBACK(void) hmR0DisableCpuOnSpecificCallback(RTCPUID idCpu, void *pvUser1, void *pvUser2)
{
    NOREF(pvUser1);
    NOREF(pvUser2);
    hmR0DisableCpu(idCpu);
}


/**
 * Callback function invoked when a cpu goes online or offline.
 *
 * @param   enmEvent            The Mp event.
 * @param   idCpu               The identifier for the CPU the function is called on.
 * @param   pvData              Opaque data (PVMCC pointer).
 */
//主要用于处理CPU的离线事件（RTMPEVENT_OFFLINE），属于虚拟化或硬件监控模块（如Hypervisor或CPU热插拔管理模块）
static DECLCALLBACK(void) hmR0MpEventCallback(RTMPEVENT enmEvent, RTCPUID idCpu, void *pvData)
{
    NOREF(pvData);
    //当前环境不支持硬件虚拟化（g_fHmVmxSupported为false）。
    //或未通过SUPR0EnableVTx启用虚拟化扩展。
    Assert(!g_fHmVmxSupported || !g_fHmVmxUsingSUPR0EnableVTx);

    /*
     * We only care about uninitializing a CPU that is going offline. When a
     * CPU comes online, the initialization is done lazily in HMR0Enter().
     */
    switch (enmEvent)
    {
        case RTMPEVENT_OFFLINE:
        {
            RTTHREADPREEMPTSTATE PreemptState = RTTHREADPREEMPTSTATE_INITIALIZER;
            //通过RTThreadPreemptDisable禁止当前线程被抢占，确保操作的原子性。
            RTThreadPreemptDisable(&PreemptState);
            if (idCpu == RTMpCpuId())
            {
                //调用hmR0DisableCpu禁用当前CPU的虚拟化功能
                int rc = hmR0DisableCpu(idCpu);
                AssertRC(rc);
                RTThreadPreemptRestore(&PreemptState);
            }
            else
            {
                RTThreadPreemptRestore(&PreemptState);
                //恢复抢占后，通过RTMpOnSpecific在目标CPU上异步执行hmR0DisableCpuOnSpecificCallback，确保操作在目标CPU的上下文中完成。
                RTMpOnSpecific(idCpu, hmR0DisableCpuOnSpecificCallback, NULL /* pvUser1 */, NULL /* pvUser2 */);
            }
            break;
        }

        default:
            break;
    }
}


/**
 * Called whenever a system power state change occurs.
 *
 * @param   enmEvent        The Power event.
 * @param   pvUser          User argument.
 */
//处理宿主系统电源事件（挂起/恢复），确保硬件虚拟化状态的一致性。
/*
  挂起（SUSPEND）：禁用所有 CPU 的虚拟化扩展（VT-x/AMD-V）。
  恢复（RESUME）：重新初始化 CPU 虚拟化配置并重新启用扩展。
*/
static DECLCALLBACK(void) hmR0PowerCallback(RTPOWEREVENT enmEvent, void *pvUser)
{
    NOREF(pvUser);
    Assert(!g_fHmVmxSupported || !g_fHmVmxUsingSUPR0EnableVTx);

#ifdef LOG_ENABLED
    if (enmEvent == RTPOWEREVENT_SUSPEND)
        SUPR0Printf("hmR0PowerCallback RTPOWEREVENT_SUSPEND\n");
    else
        SUPR0Printf("hmR0PowerCallback RTPOWEREVENT_RESUME\n");
#endif

    if (enmEvent == RTPOWEREVENT_SUSPEND)
        ASMAtomicWriteBool(&g_fHmSuspended, true);// 标记全局挂起状态

    if (g_fHmEnabled)
    {
        int         rc;
        HMR0FIRSTRC FirstRc;
        hmR0FirstRcInit(&FirstRc);

        if (enmEvent == RTPOWEREVENT_SUSPEND)
        {
            if (g_fHmGlobalInit)
            {
                /* Turn off VT-x or AMD-V on all CPUs. */
                //多核禁用虚拟化扩展
                //在所有 CPU 核心上异步执行回调函数。
                //hmR0DisableCpuCallback：执行硬件操作（如 Intel 的 VMXOFF），关闭虚拟化扩展。
                rc = RTMpOnAll(hmR0DisableCpuCallback, NULL /* pvUser 1 */, &FirstRc);
                //错误处理：VERR_NOT_SUPPORTED 表示某些 CPU 不支持操作，但允许继续执行。
                Assert(RT_SUCCESS(rc) || rc == VERR_NOT_SUPPORTED);
            }
            /* else nothing to do here for the local init case */
        }
        else
        {
            /* Reinit the CPUs from scratch as the suspend state might have
               messed with the MSRs. (lousy BIOSes as usual) */
            if (g_fHmVmxSupported)
                rc = RTMpOnAll(hmR0InitIntelCpu, &FirstRc, NULL);// 重置 VMX 相关配置（如 VMX_CR4_FIXED0 掩码）。
            else
                rc = RTMpOnAll(hmR0InitAmdCpu, &FirstRc, NULL);// 重配 SVM 相关 MSR
            Assert(RT_SUCCESS(rc) || rc == VERR_NOT_SUPPORTED);
            if (RT_SUCCESS(rc))
                //获取首个错误码
                rc = hmR0FirstRcGetStatus(&FirstRc);
#ifdef LOG_ENABLED
            if (RT_FAILURE(rc))
                SUPR0Printf("hmR0PowerCallback hmR0InitXxxCpu failed with %Rc\n", rc);
#endif
            //全局初始化时重新启用虚拟化扩展
            if (g_fHmGlobalInit)
            {
                /* Turn VT-x or AMD-V back on on all CPUs. */
                //多核启用虚拟化扩展
                //启用虚拟化扩展（如 Intel 的 VMXON）。
                //RTMpOnAll：并行执行回调，依赖硬件原子指令或 IPI（处理器间中断）确保操作到达所有核心。
                rc = RTMpOnAll(hmR0EnableCpuCallback, NULL /* pVM */, &FirstRc /* output ignored */);
                Assert(RT_SUCCESS(rc) || rc == VERR_NOT_SUPPORTED);
            }
            /* else nothing to do here for the local init case */
        }
    }

    if (enmEvent == RTPOWEREVENT_RESUME)
        ASMAtomicWriteBool(&g_fHmSuspended, false); //清除挂起标志
}


/**
 * Does ring-0 per-VM HM initialization.
 *
 * This will call the CPU specific init. routine which may initialize and allocate
 * resources for virtual CPUs.
 *
 * @returns VBox status code.
 * @param   pVM         The cross context VM structure.
 *
 * @remarks This is called after HMR3Init(), see vmR3CreateU() and
 *          vmR3InitRing3().
 */
//负责将全局硬件虚拟化配置复制到虚拟机结构体，并初始化虚拟化相关的安全防护机制。
/*
    硬件特性同步：将全局的 VT-x/AMD-V 能力配置复制到 VM 结构体中。
    虚拟化参数初始化：设置虚拟化控制字段（如 VPID、VMCS shadowing）。
    安全防护配置：针对 Spectre 等 CPU 漏洞启用缓解措施。
    vCPU 状态初始化：重置每个虚拟 CPU 的硬件虚拟化相关字段。
*/
VMMR0_INT_DECL(int) HMR0InitVM(PVMCC pVM)
{
    AssertCompile(sizeof(pVM->hm.s) <= sizeof(pVM->hm.padding));// 确保结构体不越界
    AssertCompile(sizeof(pVM->hmr0.s) <= sizeof(pVM->hmr0.padding));// 校验传入的 VM 指针有效性
    AssertCompile(sizeof(pVM->aCpus[0].hm.s) <= sizeof(pVM->aCpus[0].hm.padding));
    AssertCompile(sizeof(pVM->aCpus[0].hmr0.s) <= sizeof(pVM->aCpus[0].hmr0.padding));
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    /* Make sure we don't touch HM after we've disabled HM in preparation of a suspend. */
    if (ASMAtomicReadBool(&g_fHmSuspended)) //检查全局挂起标志
        return VERR_HM_SUSPEND_PENDING;

    /*
     * Copy globals to the VM structure.
     */
    Assert(!(pVM->hm.s.vmx.fSupported && pVM->hm.s.svm.fSupported));
    //代码根据虚拟化技术类型（VMX 或 SVM）将全局配置复制到 VM 结构体中：
    if (pVM->hm.s.vmx.fSupported)
    {
        /*
          预抢占计时器：通过 VMX_PREEMPTION_TIMER 控制 Guest 执行时间片。
          VPID：避免在 VMEntry/VMExit 时刷新 TLB，提升性能。
          VMCS shadowing：允许嵌套虚拟化中直接操作影子 VMCS，减少 VMExit 频率。*
        */
        pVM->hmr0.s.vmx.fUsePreemptTimer            = pVM->hm.s.vmx.fUsePreemptTimerCfg && g_fHmVmxUsePreemptTimer;// 启用预抢占计时器（基于配置和硬件支持）
        pVM->hm.s.vmx.fUsePreemptTimerCfg           = pVM->hmr0.s.vmx.fUsePreemptTimer;// 同步主机 CR0 值
        pVM->hm.s.vmx.cPreemptTimerShift            = g_cHmVmxPreemptTimerShift;
        pVM->hm.s.ForR3.vmx.u64HostCr0              = g_uHmVmxHostCr0;
        pVM->hm.s.ForR3.vmx.u64HostCr4              = g_uHmVmxHostCr4;
        pVM->hm.s.ForR3.vmx.u64HostMsrEfer          = g_uHmVmxHostMsrEfer;
        pVM->hm.s.ForR3.vmx.u64HostSmmMonitorCtl    = g_uHmVmxHostSmmMonitorCtl;
        pVM->hm.s.ForR3.vmx.u64HostCoreCap          = g_uHmVmxHostCoreCap;
        pVM->hm.s.ForR3.vmx.u64HostMemoryCtrl       = g_uHmVmxHostMemoryCtrl;
        pVM->hm.s.ForR3.vmx.u64HostFeatCtrl         = g_HmMsrs.u.vmx.u64FeatCtrl;
        // 复制 VMX 相关 MSR
        HMGetVmxMsrsFromHwvirtMsrs(&g_HmMsrs, &pVM->hm.s.ForR3.vmx.Msrs);
        /* If you need to tweak host MSRs for testing VMX R0 code, do it here. */

        /* Enable VPID if supported and configured. */
        //启用 VPID（虚拟处理器 ID）
        if (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VPID)
            pVM->hm.s.ForR3.vmx.fVpid = pVM->hmr0.s.vmx.fVpid = pVM->hm.s.vmx.fAllowVpid; /* Can be overridden by CFGM in HMR3Init(). */

        /* Use VMCS shadowing if supported. */
        //配置 VMCS shadowing（提升嵌套虚拟化性能）
        pVM->hmr0.s.vmx.fUseVmcsShadowing = pVM->cpum.ro.GuestFeatures.fVmx
                                         && (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VMCS_SHADOWING);
        pVM->hm.s.ForR3.vmx.fUseVmcsShadowing = pVM->hmr0.s.vmx.fUseVmcsShadowing;

        /* Use the VMCS controls for swapping the EFER MSR if supported. */
        pVM->hm.s.ForR3.vmx.fSupportsVmcsEfer = g_fHmVmxSupportsVmcsEfer;

#if 0
        /* Enable APIC register virtualization and virtual-interrupt delivery if supported. */
        if (   (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_APIC_REG_VIRT)
            && (g_HmMsrs.u.vmx.ProcCtls2.n.allowed1 & VMX_PROC_CTLS2_VIRT_INTR_DELIVERY))
            pVM->hm.s.fVirtApicRegs = true;

        /* Enable posted-interrupt processing if supported. */
        /** @todo Add and query IPRT API for host OS support for posted-interrupt IPI
         *        here. */
        if (   (g_HmMsrs.u.vmx.PinCtls.n.allowed1  & VMX_PIN_CTLS_POSTED_INT)
            && (g_HmMsrs.u.vmx.ExitCtls.n.allowed1 & VMX_EXIT_CTLS_ACK_EXT_INT))
            pVM->hm.s.fPostedIntrs = true;
#endif
    }
    else if (pVM->hm.s.svm.fSupported)
    {
        pVM->hm.s.ForR3.svm.u32Rev      = g_uHmSvmRev// 记录 SVM 版本
        pVM->hm.s.ForR3.svm.fFeatures   = g_fHmSvmFeatures;// 同步 SVM 特性（如 NRIP）
        pVM->hm.s.ForR3.svm.u64MsrHwcr  = g_HmMsrs.u.svm.u64MsrHwcr;
        /* If you need to tweak host MSRs for testing SVM R0 code, do it here. */
    }
    pVM->hm.s.ForR3.rcInit      = g_rcHmInit;
    pVM->hm.s.ForR3.uMaxAsid    = g_uHmMaxAsid;

    /*
     * Set default maximum inner loops in ring-0 before returning to ring-3.
     * Can be overriden using CFGM.
     */
    // 设置最大恢复循环次数（防止长时间占用 CPU）
    // 控制虚拟机在 Ring-0 中的最大连续执行循环次数，避免因长时间运行导致宿主机调度延迟。
    uint32_t cMaxResumeLoops = pVM->hm.s.cMaxResumeLoopsCfg;
    if (!cMaxResumeLoops)
    {
        cMaxResumeLoops     = 1024; // 默认值
        if (RTThreadPreemptIsPendingTrusty())// 若抢占延迟较高，增大循环次数
            cMaxResumeLoops = 8192;
    }
    else if (cMaxResumeLoops > 16384)
        cMaxResumeLoops = 16384;
    else if (cMaxResumeLoops < 32)
        cMaxResumeLoops = 32;
    pVM->hm.s.cMaxResumeLoopsCfg = pVM->hmr0.s.cMaxResumeLoops = cMaxResumeLoops;

    /*
     * Initialize some per-VCPU fields.
     */
    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PVMCPUCC pVCpu = VMCC_GET_CPU(pVM, idCpu);
        pVCpu->hmr0.s.idEnteredCpu  = NIL_RTCPUID; // 标记当前未进入任何物理 CPU
        pVCpu->hmr0.s.idLastCpu     = NIL_RTCPUID; // 重置上一次运行的物理 CPU

        /* We'll aways increment this the first time (host uses ASID 0). */
        //ASID（Address Space Identifier）‌：用于 TLB 隔离，避免不同 vCPU 间 TLB 冲突。
        AssertReturn(!pVCpu->hmr0.s.uCurrentAsid, VERR_HM_IPE_3);// 确保 ASID 初始为 0
    }

    /*
     * Configure defences against spectre and other CPU bugs.
     */
    /*
        IBRS（Indirect Branch Restricted Speculation）：防止基于间接分支预测的 Spectre 攻击。
        PCID（Process Context ID）：减少 TLB 刷新开销，提升安全性。
    */
    uint32_t fWorldSwitcher = 0;
    uint32_t cLastStdLeaf   = ASMCpuId_EAX(0);
    if (cLastStdLeaf >= 0x00000007 && RTX86IsValidStdRange(cLastStdLeaf))
    {
        uint32_t uEdx = 0;
        ASMCpuIdExSlow(0x00000007, 0, 0, 0, NULL, NULL, NULL, &uEdx);

        if (uEdx & X86_CPUID_STEXT_FEATURE_EDX_IBRS_IBPB)
        {
            if (pVM->hm.s.fIbpbOnVmExit)
                fWorldSwitcher |= HM_WSF_IBPB_EXIT;
            if (pVM->hm.s.fIbpbOnVmEntry)
                fWorldSwitcher |= HM_WSF_IBPB_ENTRY;
        }
        if (uEdx & X86_CPUID_STEXT_FEATURE_EDX_FLUSH_CMD)
        {
            if (pVM->hm.s.fL1dFlushOnVmEntry)
                fWorldSwitcher |= HM_WSF_L1D_ENTRY;
            else if (pVM->hm.s.fL1dFlushOnSched)
                fWorldSwitcher |= HM_WSF_L1D_SCHED;
        }
        if (uEdx & X86_CPUID_STEXT_FEATURE_EDX_MD_CLEAR)
        {
            if (pVM->hm.s.fMdsClearOnVmEntry)
                fWorldSwitcher |= HM_WSF_MDS_ENTRY;
            else if (pVM->hm.s.fMdsClearOnSched)
                fWorldSwitcher |= HM_WSF_MDS_SCHED;
        }
    }
    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PVMCPUCC pVCpu = VMCC_GET_CPU(pVM, idCpu);
        pVCpu->hmr0.s.fWorldSwitcher = fWorldSwitcher;
    }
    pVM->hm.s.ForR3.fWorldSwitcher = fWorldSwitcher;


    /*
     * Call the hardware specific initialization method.
     */
    return g_HmR0Ops.pfnInitVM(pVM);
}


/**
 * Does ring-0 per VM HM termination.
 *
 * @returns VBox status code.
 * @param   pVM         The cross context VM structure.
 */
VMMR0_INT_DECL(int) HMR0TermVM(PVMCC pVM)
{
    Log(("HMR0TermVM: %p\n", pVM));
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    /*
     * Call the hardware specific method.
     *
     * Note! We might be preparing for a suspend, so the pfnTermVM() functions should probably not
     * mess with VT-x/AMD-V features on the CPU, currently all they do is free memory so this is safe.
     */
    return g_HmR0Ops.pfnTermVM(pVM);
}


/**
 * Sets up a VT-x or AMD-V session.
 *
 * This is mostly about setting up the hardware VM state.
 *
 * @returns VBox status code.
 * @param   pVM         The cross context VM structure.
 */
//在Ring-0层完成全局虚拟化环境的配置
/*
    硬件虚拟化扩展的启用（如 Intel VT-x/AMD-V）。
    虚拟化数据结构初始化（如 VMCS/VMCB）。
    全局状态同步（如主机上下文、Guest 寄存器状态）。

    HMR0SetupVM：虚拟机级别的全局初始化（一次调用，影响所有 vCPU）。

    hmR0EnterCpu：vCPU 级别的上下文准备（每次 VMEntry 前调用，仅处理单个 vCPU）。
*/
VMMR0_INT_DECL(int) HMR0SetupVM(PVMCC pVM)
{
    Log(("HMR0SetupVM: %p\n", pVM));
    //确保传入的虚拟机指针 pVM 有效
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    //确保传入的虚拟机指针 pVM 有效，并检查全局挂起标志 g_fHmSuspended，防止在系统挂起（如休眠）过程中操作虚拟机。
    /* Make sure we don't touch HM after we've disabled HM in preparation of a suspend. */
    AssertReturn(!ASMAtomicReadBool(&g_fHmSuspended), VERR_HM_SUSPEND_PENDING);

    /* On first entry we'll sync everything. */
	/*
	 遍历所有虚拟 CPU（vCPU），标记其上下文状态需要更新：
     HM_CHANGED_HOST_CONTEXT：强制后续操作重新加载宿主 CPU 状态（如 CR3、GDTR）。
     HM_CHANGED_ALL_GUEST：强制同步所有 Guest 状态（如 MSR、控制寄存器）。
     设计意图：在虚拟机启动时，强制执行全量状态同步，避免因热迁移或挂起恢复导致的状态不一致。
	*/
    VMCC_FOR_EACH_VMCPU_STMT(pVM, pVCpu->hm.s.fCtxChanged |= HM_CHANGED_HOST_CONTEXT | HM_CHANGED_ALL_GUEST);

    /*
     * Call the hardware specific setup VM method. This requires the CPU to be
     * enabled for AMD-V/VT-x and preemption to be prevented.
     */
    RTTHREADPREEMPTSTATE PreemptState = RTTHREADPREEMPTSTATE_INITIALIZER;
    //禁用抢占（RTThreadPreemptDisable）：确保当前线程不会被调度到其他 CPU 执行，保障操作的原子性。
    RTThreadPreemptDisable(&PreemptState);
    RTCPUID const idCpu = RTMpCpuId();

    /* Enable VT-x or AMD-V if local init is required. */
    int rc;
    //g_fHmGlobalInit 表示全局虚拟化是否已初始化（如首次启动时未初始化）
    if (!g_fHmGlobalInit)
    {
        Assert(!g_fHmVmxSupported || !g_fHmVmxUsingSUPR0EnableVTx);
        //调用 hmR0EnableCpu 启用当前 CPU 的硬件虚拟化扩展（如执行 Intel 的 VMXON 或 AMD 的 SVML）
        rc = hmR0EnableCpu(pVM, idCpu);
        if (RT_FAILURE(rc))
        {
            //若启用失败（如硬件不支持 VT-x），立即恢复抢占状态并返回错误码
            RTThreadPreemptRestore(&PreemptState);
            return rc;
        }
    }

    /* Setup VT-x or AMD-V. */
    /*

    动态分发：g_HmR0Ops 是硬件操作函数表，根据虚拟化技术类型指向不同实现：
       Intel VT-x：hmR0VmxSetupVM，负责分配 VMCS 区域、初始化 VMXON 区域。
       AMD-V：hmR0SvmSetupVM，负责配置 VMCB 和主机保存区域。
    */
    rc = g_HmR0Ops.pfnSetupVM(pVM);

    /* Disable VT-x or AMD-V if local init was done before. */
    /*
        条件：仅在未完成全局初始化时执行。
        操作：调用 hmR0DisableCpu 关闭当前 CPU 的虚拟化扩展（如执行 VMXOFF），释放相关资源。
        设计意图：避免在多 CPU 环境下遗留未清理的硬件状态（如 VMX 模式残留）。
    */
    if (!g_fHmGlobalInit)
    {
        Assert(!g_fHmVmxSupported || !g_fHmVmxUsingSUPR0EnableVTx);
        int rc2 = hmR0DisableCpu(idCpu);
        AssertRC(rc2);
    }

    RTThreadPreemptRestore(&PreemptState);
    return rc;
}


/**
 * Notification callback before an assertion longjump and guru mediation.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context virtual CPU structure.
 * @param   pvUser          User argument, currently unused, NULL.
 */
static DECLCALLBACK(int) hmR0AssertionCallback(PVMCPUCC pVCpu, void *pvUser)
{
    RT_NOREF(pvUser);
    Assert(pVCpu);
    Assert(g_HmR0Ops.pfnAssertionCallback);
    return g_HmR0Ops.pfnAssertionCallback(pVCpu);
}


/**
 * Turns on HM on the CPU if necessary and initializes the bare minimum state
 * required for entering HM context.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
//负责在Ring-0层完成物理CPU的虚拟化环境准备，确保当前CPU能够安全进入虚拟机监控模式（VMX non-root模式或SVM guest模式
VMMR0_INT_DECL(int) hmR0EnterCpu(PVMCPUCC pVCpu)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));//通过Assert(!RTThreadPreemptIsEnabled)确保执行期间不被抢占，避免并发问题

    int              rc       = VINF_SUCCESS;
    RTCPUID const    idCpu    = RTMpCpuId();
    //g_aHmCpuInfo数组存储所有物理CPU的虚拟化状态（如VMCS/SVM区域指针），通过RTMpCpuId获取当前CPU索引
    PHMPHYSCPU       pHostCpu = &g_aHmCpuInfo[idCpu];
    AssertPtr(pHostCpu);

    /* Enable VT-x or AMD-V if local init is required, or enable if it's a freshly onlined CPU. */
    if (!pHostCpu->fConfigured)
        rc = hmR0EnableCpu(pVCpu->CTX_SUFF(pVM), idCpu);//若未配置，调用hmR0EnableCpu执行硬件初始化（如执行VMXON指令）

    /* Register a callback to fire prior to performing a longjmp to ring-3 so HM can disable VT-x/AMD-V if needed. */
    //VMMR0AssertionSetNotification注册hmR0AssertionCallback，
	//在进程切换或异常退出到Ring-3时，自动执行虚拟化扩展禁用操作（如VMXOFF），防止遗留敏感状态
    VMMR0AssertionSetNotification(pVCpu, hmR0AssertionCallback, NULL /*pvUser*/);

    /* Reload host-state (back from ring-3/migrated CPUs) and shared guest/host bits. */
    //根据虚拟化技术类型（VMX/SVM），设置fCtxChanged标志位，触发后续操作：
	//上下文同步：在VMEntry前，通过hmR0VmxExportGuestStateOptimal等函数导出Guest状态到VMCS/VMCB，
	//确保CPU寄存器、控制字段与虚拟化数据结构一致
	/*
        if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_HOST_CONTEXT) {
            hmR0VmxExportHostState(pVCpu);      // 导出主机状态到VMCS
        }
        if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE) {
            hmR0VmxExportSharedState(pVCpu);    // 导出共享状态到VMCS
        }
        if (pVCpu->hm.s.fCtxChanged & HM_CHANGED_GUEST_STATE) {
            hmR0VmxExportGuestStateOptimal(pVCpu); // 导出Guest状态到VMCS
        }
	*/
    if (g_fHmVmxSupported)
        //HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE，表示需重新加载VMCS中的主机状态及共享字段
        pVCpu->hm.s.fCtxChanged |= HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE;
    else
        //类似逻辑，更新VMCB相关状态
        pVCpu->hm.s.fCtxChanged |= HM_CHANGED_HOST_CONTEXT | HM_CHANGED_SVM_HOST_GUEST_SHARED_STATE;

    Assert(pHostCpu->idCpu == idCpu && pHostCpu->idCpu != NIL_RTCPUID);
    //pVCpu->hmr0.s.idEnteredCpu记录当前VCPU所在的物理CPU，确保后续操作不发生跨CPU迁移
    pVCpu->hmr0.s.idEnteredCpu = idCpu;
    return rc;
}


/**
 * Enters the VT-x or AMD-V session.
 *
 * @returns VBox status code.
 * @param   pVCpu      The cross context virtual CPU structure.
 *
 * @remarks This is called with preemption disabled.
 */
//准备并进入硬件虚拟化执行环境。
/*
 *  在 HMR0Enter 函数中，并没有直接包含进入虚拟机的代码。
    这个函数是进入硬件虚拟化环境的准备阶段，真正的虚拟机
    入口是在后续的 HMR0RunGuestCode 或类似函数中通过执行
    VMLAUNCH/VMRESUME（Intel VT-x）或 VMRUN（AMD-V）指令实现的。
 * */
VMMR0_INT_DECL(int) HMR0Enter(PVMCPUCC pVCpu)
{
    /* Make sure we can't enter a session after we've disabled HM in preparation of a suspend. */
    AssertReturn(!ASMAtomicReadBool(&g_fHmSuspended), VERR_HM_SUSPEND_PENDING);//确保系统未处于挂起流程中（原子读取g_fHmSuspended）
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));//必须处于不可抢占状态（关键路径要求）

    /* Load the bare minimum state required for entering HM. */
	/*物理CPU虚拟化功能激活（如执行VMXON）
      基础主机状态配置
      必要的全局/本地状态初始化
    */
    int rc = hmR0EnterCpu(pVCpu);
    if (RT_SUCCESS(rc))
    {
		/*验证fCtxChanged标志确保：
          主机上下文已正确保存（HM_CHANGED_HOST_CONTEXT）
          技术特定共享状态已更新（VT-x/SVM对应标记）
        */
        if (g_fHmVmxSupported)
            Assert(   (pVCpu->hm.s.fCtxChanged & (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE))
                   ==                            (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_VMX_HOST_GUEST_SHARED_STATE));
        else
            Assert(   (pVCpu->hm.s.fCtxChanged & (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_SVM_HOST_GUEST_SHARED_STATE))
                   ==                            (HM_CHANGED_HOST_CONTEXT | HM_CHANGED_SVM_HOST_GUEST_SHARED_STATE));

        /* Keep track of the CPU owning the VMCS for debugging scheduling weirdness and ring-3 calls. */
		/*
		 *通过函数指针调用技术特定实现：
          VT-x：加载VMCS并验证状态
          AMD-V：配置VMCB
          记录当前物理CPU到idEnteredCpu
		 * */
        rc = g_HmR0Ops.pfnEnterSession(pVCpu);
        AssertMsgRCReturnStmt(rc, ("rc=%Rrc pVCpu=%p\n", rc, pVCpu),  pVCpu->hmr0.s.idEnteredCpu = NIL_RTCPUID, rc);

        /* Exports the host-state as we may be resuming code after a longjmp and quite
           possibly now be scheduled on a different CPU. */
		//主机状态导出
		/*
		 *主机寄存器状态正确保存
          可能处理CPU迁移后的状态同步
          为可能的VM-Exit做好准备
		 * */
        rc = g_HmR0Ops.pfnExportHostState(pVCpu);
        AssertMsgRCReturnStmt(rc, ("rc=%Rrc pVCpu=%p\n", rc, pVCpu),  pVCpu->hmr0.s.idEnteredCpu = NIL_RTCPUID, rc);
    }
    return rc;
}


/**
 * Deinitializes the bare minimum state used for HM context and if necessary
 * disable HM on the CPU.
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context virtual CPU structure.
 *
 * @remarks No-long-jump zone!!!
 */
//在vCPU离开物理CPU时执行清理操作
//vCPU线程被调度到其他物理核心
//虚拟机暂停/关闭
//动态资源调整（如CPU热移除）
VMMR0_INT_DECL(int) HMR0LeaveCpu(PVMCPUCC pVCpu)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    VMCPU_ASSERT_EMT_RETURN(pVCpu, VERR_HM_WRONG_CPU); //VMCPU_ASSERT_EMT_RETURN宏验证调用者是否为正确的vCPU仿真线程（Emulation Thread）

    RTCPUID const idCpu    = RTMpCpuId();
    PCHMPHYSCPU   pHostCpu = &g_aHmCpuInfo[idCpu];//获取当前物理CPU ID和对应的HMPHYSCPU结构
    if (   !g_fHmGlobalInit
        && pHostCpu->fConfigured)//仅在全局未初始化但当前CPU已配置时执行
    {
        int rc = hmR0DisableCpu(idCpu);//VT-x：清除VMCS状态，执行VMXOFF指令 AMD-V：清理VMCB，执行VMRUN相关清理
        AssertRCReturn(rc, rc); //状态验证：确保配置标志和CPU ID被正确重置
        Assert(!pHostCpu->fConfigured);
        Assert(pHostCpu->idCpu == NIL_RTCPUID);

        /* For obtaining a non-zero ASID/VPID on next re-entry. */
        pVCpu->hmr0.s.idLastCpu = NIL_RTCPUID; //重置最后使用的CPU ID，强制下次获取新ASID/VPID
    }

    /* Clear it while leaving HM context, hmPokeCpuForTlbFlush() relies on this. */
    pVCpu->hmr0.s.idEnteredCpu = NIL_RTCPUID; //清除进入标记，影响TLB刷新逻辑（hmPokeCpuForTlbFlush依赖此状态）

    /* De-register the longjmp-to-ring 3 callback now that we have reliquished hardware resources. */
	/*
	  移除ring3回调通知（如异常处理回调）
      因为硬件资源已释放，不再需要这些通知
	*/
    VMMR0AssertionRemoveNotification(pVCpu);
    return VINF_SUCCESS;
}


/**
 * Thread-context hook for HM.
 *
 * This is used together with RTThreadCtxHookCreate() on platforms which
 * supports it, and directly from VMMR0EmtPrepareForBlocking() and
 * VMMR0EmtResumeAfterBlocking() on platforms which don't.
 *
 * @param   enmEvent        The thread-context event.
 * @param   pvUser          Opaque pointer to the VMCPU.
 */
//一个线程上下文切换回调函数，
/*
 *典型事件类型
RTTHREADCTXEVENT 可能包含：
    RTTHREADCTXEVENT_OUT: 线程即将离开当前CPU
    RTTHREADCTXEVENT_IN: 线程即将进入新CPU

实现要求
底层实现需要处理：
  VMCS/VMCB 状态:
  保存/恢复虚拟化控制结构
  处理可能的迁移后重新初始化
TLB 一致性:
  确保地址转换缓存正确
  必要时执行 INVEPT/INVVPID
APIC 虚拟化:
  维护中断控制器状态
*/
VMMR0_INT_DECL(void) HMR0ThreadCtxCallback(RTTHREADCTXEVENT enmEvent, void *pvUser)
{
    PVMCPUCC pVCpu = (PVMCPUCC)pvUser;
    Assert(pVCpu);
    Assert(g_HmR0Ops.pfnThreadCtxCallback);
/*
   事件类型
   vCPU 上下文
   全局初始化状态标志
*/
    g_HmR0Ops.pfnThreadCtxCallback(enmEvent, pVCpu, g_fHmGlobalInit);
}


/**
 * Runs guest code in a hardware accelerated VM.
 *
 * @returns Strict VBox status code. (VBOXSTRICTRC isn't used because it's
 *          called from setjmp assembly.)
 * @param   pVM         The cross context VM structure.
 * @param   pVCpu       The cross context virtual CPU structure.
 *
 * @remarks Can be called with preemption enabled if thread-context hooks are
 *          used!!!
 */
//pVM: 指向虚拟机（VM）实例的指针
//pVCpu: 指向当前虚拟CPU（vCPU）上下文的指针
VMMR0_INT_DECL(int) HMR0RunGuestCode(PVMCC pVM, PVMCPUCC pVCpu)
{
    RT_NOREF(pVM);

//严格模式检查（DEBUG构建）:
#ifdef VBOX_STRICT
    /* With thread-context hooks we would be running this code with preemption enabled. */
    if (!RTThreadPreemptIsEnabled(NIL_RTTHREAD))
    {
        PCHMPHYSCPU pHostCpu = &g_aHmCpuInfo[RTMpCpuId()];
        Assert(!VMCPU_FF_IS_ANY_SET(pVCpu, VMCPU_FF_PGM_SYNC_CR3 | VMCPU_FF_PGM_SYNC_CR3_NON_GLOBAL));//确保没有设置关键强制标志（CR3同步相关）
        Assert(pHostCpu->fConfigured);//确认当前物理CPU已正确配置虚拟化环境
        AssertReturn(!ASMAtomicReadBool(&g_fHmSuspended), VERR_HM_SUSPEND_PENDING);//检查全局挂起状态,（防止在挂起过程中运行客户机代码）
    }
#endif

	//执行客户机代码:
    VBOXSTRICTRC rcStrict = g_HmR0Ops.pfnRunGuestCode(pVCpu);
    return VBOXSTRICTRC_VAL(rcStrict);
}


/**
 * Notification from CPUM that it has unloaded the guest FPU/SSE/AVX state from
 * the host CPU and that guest access to it must be intercepted.
 *
 * @param   pVCpu   The cross context virtual CPU structure of the calling EMT.
 */
VMMR0_INT_DECL(void) HMR0NotifyCpumUnloadedGuestFpuState(PVMCPUCC pVCpu)
{
    ASMAtomicUoOrU64(&pVCpu->hm.s.fCtxChanged, HM_CHANGED_GUEST_CR0);
}


/**
 * Notification from CPUM that it has modified the host CR0 (because of FPU).
 *
 * @param   pVCpu   The cross context virtual CPU structure of the calling EMT.
 */
VMMR0_INT_DECL(void) HMR0NotifyCpumModifiedHostCr0(PVMCPUCC pVCpu)
{
    ASMAtomicUoOrU64(&pVCpu->hm.s.fCtxChanged, HM_CHANGED_HOST_CONTEXT);
}


/**
 * Returns suspend status of the host.
 *
 * @returns Suspend pending or not.
 */
VMMR0_INT_DECL(bool) HMR0SuspendPending(void)
{
    return ASMAtomicReadBool(&g_fHmSuspended);
}


/**
 * Invalidates a guest page from the host TLB.
 *
 * @param   pVCpu       The cross context virtual CPU structure.
 * @param   GCVirt      Page to invalidate.
 */
//指定虚拟页的硬件虚拟化缓存失效
//GCVirt: 客户机虚拟地址，表示需要失效的页面地址
VMMR0_INT_DECL(int) HMR0InvalidatePage(PVMCPUCC pVCpu, RTGCPTR GCVirt)
{
    PVMCC pVM = pVCpu->CTX_SUFF(pVM);
    if (pVM->hm.s.vmx.fSupported)
        return VMXR0InvalidatePage(pVCpu, GCVirt);
    return SVMR0InvalidatePage(pVCpu, GCVirt);
}


/**
 * Returns the cpu structure for the current cpu.
 * Keep in mind that there is no guarantee it will stay the same (long jumps to ring 3!!!).
 *
 * @returns The cpu structure pointer.
 */
VMMR0_INT_DECL(PHMPHYSCPU) hmR0GetCurrentCpu(void)
{
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    //调用 RTMpCpuId() 获取当前物理 CPU 的逻辑 ID（通常为 0 到 N-1，N 为系统 CPU 核心数）
    RTCPUID const idCpu = RTMpCpuId();
    Assert(idCpu < RT_ELEMENTS(g_aHmCpuInfo));
    return &g_aHmCpuInfo[idCpu];
}


/**
 * Interface for importing state on demand (used by IEM).
 *
 * @returns VBox status code.
 * @param   pVCpu       The cross context CPU structure.
 * @param   fWhat       What to import, CPUMCTX_EXTRN_XXX.
 */
//用于按需导入虚拟 CPU（vCPU）的状态
//当虚拟机因中断、异常或指令模拟退出到 Hypervisor 时，可能需要读取 vCPU 的某些状态（如 RIP、RFLAGS）。
//此函数确保这些状态已从硬件同步到 Hypervisor 的软件上下文。
//避免在每次 VM-Exit 时全量导入所有状态，仅导入必要的字段以减少开销。
VMMR0_INT_DECL(int) HMR0ImportStateOnDemand(PVMCPUCC pVCpu, uint64_t fWhat)
{
    if (pVCpu->CTX_SUFF(pVM)->hm.s.vmx.fSupported)
        return VMXR0ImportStateOnDemand(pVCpu, fWhat);
    return SVMR0ImportStateOnDemand(pVCpu, fWhat);
}


/**
 * Gets HM VM-exit auxiliary information.
 *
 * @returns VBox status code.
 * @param   pVCpu           The cross context CPU structure.
 * @param   pHmExitAux      Where to store the auxiliary info.
 * @param   fWhat           What to get, see HMVMX_READ_XXX. This is ignored/unused
 *                          on AMD-V.
 *
 * @remarks Currently this works only when executing a nested-guest using
 *          hardware-assisted execution as it's where the auxiliary information is
 *          required outside of HM. In the future we can make this available while
 *          executing a regular (non-nested) guest if necessary.
 */
//主要用于获取虚拟机退出时的辅助信息
VMMR0_INT_DECL(int) HMR0GetExitAuxInfo(PVMCPUCC pVCpu, PHMEXITAUX pHmExitAux, uint32_t fWhat)
{
    Assert(pHmExitAux);
    Assert(!(fWhat & ~HMVMX_READ_VALID_MASK));
    if (pVCpu->CTX_SUFF(pVM)->hm.s.vmx.fSupported) //intel vmx/amd svm
        return VMXR0GetExitAuxInfo(pVCpu, &pHmExitAux->Vmx, fWhat);
    return SVMR0GetExitAuxInfo(pVCpu, &pHmExitAux->Svm);
}


#ifdef VBOX_STRICT

/**
 * Dumps a descriptor.
 *
 * @param   pDesc    Descriptor to dump.
 * @param   Sel      The selector.
 * @param   pszSel   The name of the selector.
 */
VMMR0_INT_DECL(void) hmR0DumpDescriptor(PCX86DESCHC pDesc, RTSEL Sel, const char *pszSel)
{
    /*
     * Make variable description string.
     */
    static struct
    {
        unsigned    cch;
        const char *psz;
/*
  这个数组定义了 32 种可能的描述符类型，包括：
  系统描述符类型(0x00-0x0F)：
  TSS(任务状态段)
  LDT(局部描述符表)
  调用门
  中断门
  陷阱门
  非系统描述符类型(0x10-0x1F)：
  数据段(只读/读写)
  代码段(只执行/可读)
  各种访问权限组合
*/
    } const s_aTypes[32] =
    {
# define STRENTRY(str) { sizeof(str) - 1, str }

        /* system */
# if HC_ARCH_BITS == 64
        STRENTRY("Reserved0 "),                  /* 0x00 */
        STRENTRY("Reserved1 "),                  /* 0x01 */
        STRENTRY("LDT "),                        /* 0x02 */
        STRENTRY("Reserved3 "),                  /* 0x03 */
        STRENTRY("Reserved4 "),                  /* 0x04 */
        STRENTRY("Reserved5 "),                  /* 0x05 */
        STRENTRY("Reserved6 "),                  /* 0x06 */
        STRENTRY("Reserved7 "),                  /* 0x07 */
        STRENTRY("Reserved8 "),                  /* 0x08 */
        STRENTRY("TSS64Avail "),                 /* 0x09 */
        STRENTRY("ReservedA "),                  /* 0x0a */
        STRENTRY("TSS64Busy "),                  /* 0x0b */
        STRENTRY("Call64 "),                     /* 0x0c */
        STRENTRY("ReservedD "),                  /* 0x0d */
        STRENTRY("Int64 "),                      /* 0x0e */
        STRENTRY("Trap64 "),                     /* 0x0f */
# else
        STRENTRY("Reserved0 "),                  /* 0x00 */
        STRENTRY("TSS16Avail "),                 /* 0x01 */
        STRENTRY("LDT "),                        /* 0x02 */
        STRENTRY("TSS16Busy "),                  /* 0x03 */
        STRENTRY("Call16 "),                     /* 0x04 */
        STRENTRY("Task "),                       /* 0x05 */
        STRENTRY("Int16 "),                      /* 0x06 */
        STRENTRY("Trap16 "),                     /* 0x07 */
        STRENTRY("Reserved8 "),                  /* 0x08 */
        STRENTRY("TSS32Avail "),                 /* 0x09 */
        STRENTRY("ReservedA "),                  /* 0x0a */
        STRENTRY("TSS32Busy "),                  /* 0x0b */
        STRENTRY("Call32 "),                     /* 0x0c */
        STRENTRY("ReservedD "),                  /* 0x0d */
        STRENTRY("Int32 "),                      /* 0x0e */
        STRENTRY("Trap32 "),                     /* 0x0f */
# endif
        /* non system */
        STRENTRY("DataRO "),                     /* 0x10 */
        STRENTRY("DataRO Accessed "),            /* 0x11 */
        STRENTRY("DataRW "),                     /* 0x12 */
        STRENTRY("DataRW Accessed "),            /* 0x13 */
        STRENTRY("DataDownRO "),                 /* 0x14 */
        STRENTRY("DataDownRO Accessed "),        /* 0x15 */
        STRENTRY("DataDownRW "),                 /* 0x16 */
        STRENTRY("DataDownRW Accessed "),        /* 0x17 */
        STRENTRY("CodeEO "),                     /* 0x18 */
        STRENTRY("CodeEO Accessed "),            /* 0x19 */
        STRENTRY("CodeER "),                     /* 0x1a */
        STRENTRY("CodeER Accessed "),            /* 0x1b */
        STRENTRY("CodeConfEO "),                 /* 0x1c */
        STRENTRY("CodeConfEO Accessed "),        /* 0x1d */
        STRENTRY("CodeConfER "),                 /* 0x1e */
        STRENTRY("CodeConfER Accessed ")         /* 0x1f */
# undef SYSENTRY
    };
# define ADD_STR(psz, pszAdd) do { strcpy(psz, pszAdd); psz += strlen(pszAdd); } while (0)
    char        szMsg[128];
    char       *psz = &szMsg[0];
    unsigned    i = pDesc->Gen.u1DescType << 4 | pDesc->Gen.u4Type;
    memcpy(psz, s_aTypes[i].psz, s_aTypes[i].cch);
    psz += s_aTypes[i].cch;

/*
  函数会检查并输出以下描述符属性：
  Present/Not-Present (存在位)
  64-bit/Comp (64位模式特有)
  Page/Granularity (粒度)
  32-bit/16-bit (默认操作数大小)

  64 bit: CS { 0x0010 - 0x00000000 0x00209b00 - base=0x00000000 limit=0x000fffff dpl=0 } CodeER Present 64-bit
  32 bit: DS { 0x0023 - 0x0000ffff 0x00cf9300 - base=0x00000000 limit=0xffffffff dpl=3 } DataRW Present Page 32-bit
*/
    if (pDesc->Gen.u1Present)
        ADD_STR(psz, "Present ");
    else
        ADD_STR(psz, "Not-Present ");
# if HC_ARCH_BITS == 64
    if (pDesc->Gen.u1Long)
        ADD_STR(psz, "64-bit ");
    else
        ADD_STR(psz, "Comp ");
# else
    if (pDesc->Gen.u1Granularity)
        ADD_STR(psz, "Page ");
    if (pDesc->Gen.u1DefBig)
        ADD_STR(psz, "32-bit ");
    else
        ADD_STR(psz, "16-bit ");//拼接字符串的辅助宏
# endif
# undef ADD_STR
    *psz = '\0';

    /*
     * Limit and Base and format the output.
     */
#ifdef LOG_ENABLED
    uint32_t u32Limit = X86DESC_LIMIT_G(pDesc); //计算考虑粒度的段界限

# if HC_ARCH_BITS == 64
    uint64_t const u64Base  = X86DESC64_BASE(pDesc);//获取段基址
    Log(("  %s { %#04x - %#RX64 %#RX64 - base=%#RX64 limit=%#08x dpl=%d } %s\n", pszSel,
         Sel, pDesc->au64[0], pDesc->au64[1], u64Base, u32Limit, pDesc->Gen.u2Dpl, szMsg));
# else
    uint32_t const u32Base  = X86DESC_BASE(pDesc);//获取段基址
    Log(("  %s { %#04x - %#08x %#08x - base=%#08x limit=%#08x dpl=%d } %s\n", pszSel,
         Sel, pDesc->au32[0], pDesc->au32[1], u32Base, u32Limit, pDesc->Gen.u2Dpl, szMsg));
# endif
#else
    NOREF(Sel); NOREF(pszSel);
#endif
}


/**
 * Formats a full register dump.
 *
 * @param   pVCpu   The cross context virtual CPU structure.
 * @param   fFlags  The dumping flags (HM_DUMP_REG_FLAGS_XXX).
 */
/*
  通用寄存器(GPRs)
  控制寄存器(CR0-CR4)
  段寄存器(CS,DS,ES等)
  调试寄存器(DR0-DR7)
  标志寄存器(EFLAGS)
  FPU/MMX/SSE 状态
  特殊模式寄存器(MSRs)
*/
VMMR0_INT_DECL(void) hmR0DumpRegs(PVMCPUCC pVCpu, uint32_t fFlags)
{
    /*
     * Format the flags.
     */
    static struct
    {
        const char *pszSet;
        const char *pszClear;
        uint32_t    fFlag;
    } const s_aFlags[] =
    {
        { "vip", NULL, X86_EFL_VIP },
        { "vif", NULL, X86_EFL_VIF },
        { "ac",  NULL, X86_EFL_AC  },
        { "vm",  NULL, X86_EFL_VM  },
        { "rf",  NULL, X86_EFL_RF  },
        { "nt",  NULL, X86_EFL_NT  },
		//"ov"/"nv" 表示溢出标志(OF)
        { "ov",  "nv", X86_EFL_OF  },
        { "dn",  "up", X86_EFL_DF  },
		//"ei"/"di" 表示中断使能标志(IF)
        { "ei",  "di", X86_EFL_IF  },
        { "tf",  NULL, X86_EFL_TF  },
        { "nt",  "pl", X86_EFL_SF  },
        { "nz",  "zr", X86_EFL_ZF  },
        { "ac",  "na", X86_EFL_AF  },
        { "po",  "pe", X86_EFL_PF  },
		//"cy"/"nc" 表示进位标志(CF)
        { "cy",  "nc", X86_EFL_CF  },
    };
    char szEFlags[80];
    char *psz = szEFlags;
    PCCPUMCTX pCtx = &pVCpu->cpum.GstCtx;
    uint32_t fEFlags = pCtx->eflags.u;
    for (unsigned i = 0; i < RT_ELEMENTS(s_aFlags); i++)
    {
        const char *pszAdd = s_aFlags[i].fFlag & fEFlags ? s_aFlags[i].pszSet : s_aFlags[i].pszClear;
        if (pszAdd)
        {
            strcpy(psz, pszAdd);
            psz += strlen(pszAdd);
            *psz++ = ' ';
        }
    }
    psz[-1] = '\0';

    if (fFlags & HM_DUMP_REG_FLAGS_GPRS)
    {
        /*
         * Format the registers.
         */
        // 输出通用寄存器
        if (CPUMIsGuestIn64BitCode(pVCpu))
            // 64位模式寄存器输出
            Log(("rax=%016RX64 rbx=%016RX64 rcx=%016RX64 rdx=%016RX64\n"
                 "rsi=%016RX64 rdi=%016RX64 r8 =%016RX64 r9 =%016RX64\n"
                 "r10=%016RX64 r11=%016RX64 r12=%016RX64 r13=%016RX64\n"
                 "r14=%016RX64 r15=%016RX64\n"
                 "rip=%016RX64 rsp=%016RX64 rbp=%016RX64 iopl=%d %*s\n"
                 "cs={%04x base=%016RX64 limit=%08x flags=%08x}\n"
                 "ds={%04x base=%016RX64 limit=%08x flags=%08x}\n"
                 "es={%04x base=%016RX64 limit=%08x flags=%08x}\n"
                 "fs={%04x base=%016RX64 limit=%08x flags=%08x}\n"
                 "gs={%04x base=%016RX64 limit=%08x flags=%08x}\n"
                 "ss={%04x base=%016RX64 limit=%08x flags=%08x}\n"
                 "cr0=%016RX64 cr2=%016RX64 cr3=%016RX64 cr4=%016RX64\n"
                 "dr0=%016RX64 dr1=%016RX64 dr2=%016RX64 dr3=%016RX64\n"
                 "dr4=%016RX64 dr5=%016RX64 dr6=%016RX64 dr7=%016RX64\n"
                 "gdtr=%016RX64:%04x  idtr=%016RX64:%04x  eflags=%08x\n"
                 "ldtr={%04x base=%08RX64 limit=%08x flags=%08x}\n"
                 "tr  ={%04x base=%08RX64 limit=%08x flags=%08x}\n"
                 "SysEnter={cs=%04llx eip=%08llx esp=%08llx}\n"
                 ,
                 pCtx->rax, pCtx->rbx, pCtx->rcx, pCtx->rdx, pCtx->rsi, pCtx->rdi,
                 pCtx->r8, pCtx->r9, pCtx->r10, pCtx->r11, pCtx->r12, pCtx->r13,
                 pCtx->r14, pCtx->r15,
                 pCtx->rip, pCtx->rsp, pCtx->rbp, X86_EFL_GET_IOPL(fEFlags), 31, szEFlags,
                 pCtx->cs.Sel, pCtx->cs.u64Base, pCtx->cs.u32Limit, pCtx->cs.Attr.u,
                 pCtx->ds.Sel, pCtx->ds.u64Base, pCtx->ds.u32Limit, pCtx->ds.Attr.u,
                 pCtx->es.Sel, pCtx->es.u64Base, pCtx->es.u32Limit, pCtx->es.Attr.u,
                 pCtx->fs.Sel, pCtx->fs.u64Base, pCtx->fs.u32Limit, pCtx->fs.Attr.u,
                 pCtx->gs.Sel, pCtx->gs.u64Base, pCtx->gs.u32Limit, pCtx->gs.Attr.u,
                 pCtx->ss.Sel, pCtx->ss.u64Base, pCtx->ss.u32Limit, pCtx->ss.Attr.u,
                 pCtx->cr0,  pCtx->cr2, pCtx->cr3,  pCtx->cr4,
                 pCtx->dr[0],  pCtx->dr[1], pCtx->dr[2],  pCtx->dr[3],
                 pCtx->dr[4],  pCtx->dr[5], pCtx->dr[6],  pCtx->dr[7],
                 pCtx->gdtr.pGdt, pCtx->gdtr.cbGdt, pCtx->idtr.pIdt, pCtx->idtr.cbIdt, fEFlags,
                 pCtx->ldtr.Sel, pCtx->ldtr.u64Base, pCtx->ldtr.u32Limit, pCtx->ldtr.Attr.u,
                 pCtx->tr.Sel, pCtx->tr.u64Base, pCtx->tr.u32Limit, pCtx->tr.Attr.u,
                 pCtx->SysEnter.cs, pCtx->SysEnter.eip, pCtx->SysEnter.esp));
        else
            Log(("eax=%08x ebx=%08x ecx=%08x edx=%08x esi=%08x edi=%08x\n"
                 "eip=%08x esp=%08x ebp=%08x iopl=%d %*s\n"
                 "cs={%04x base=%016RX64 limit=%08x flags=%08x} dr0=%08RX64 dr1=%08RX64\n"
                 "ds={%04x base=%016RX64 limit=%08x flags=%08x} dr2=%08RX64 dr3=%08RX64\n"
                 "es={%04x base=%016RX64 limit=%08x flags=%08x} dr4=%08RX64 dr5=%08RX64\n"
                 "fs={%04x base=%016RX64 limit=%08x flags=%08x} dr6=%08RX64 dr7=%08RX64\n"
                 "gs={%04x base=%016RX64 limit=%08x flags=%08x} cr0=%08RX64 cr2=%08RX64\n"
                 "ss={%04x base=%016RX64 limit=%08x flags=%08x} cr3=%08RX64 cr4=%08RX64\n"
                 "gdtr=%016RX64:%04x  idtr=%016RX64:%04x  eflags=%08x\n"
                 "ldtr={%04x base=%08RX64 limit=%08x flags=%08x}\n"
                 "tr  ={%04x base=%08RX64 limit=%08x flags=%08x}\n"
                 "SysEnter={cs=%04llx eip=%08llx esp=%08llx}\n"
                 ,
                 pCtx->eax, pCtx->ebx, pCtx->ecx, pCtx->edx, pCtx->esi, pCtx->edi,
                 pCtx->eip, pCtx->esp, pCtx->ebp, X86_EFL_GET_IOPL(fEFlags), 31, szEFlags,
                 pCtx->cs.Sel, pCtx->cs.u64Base, pCtx->cs.u32Limit, pCtx->cs.Attr.u, pCtx->dr[0],  pCtx->dr[1],
                 pCtx->ds.Sel, pCtx->ds.u64Base, pCtx->ds.u32Limit, pCtx->ds.Attr.u, pCtx->dr[2],  pCtx->dr[3],
                 pCtx->es.Sel, pCtx->es.u64Base, pCtx->es.u32Limit, pCtx->es.Attr.u, pCtx->dr[4],  pCtx->dr[5],
                 pCtx->fs.Sel, pCtx->fs.u64Base, pCtx->fs.u32Limit, pCtx->fs.Attr.u, pCtx->dr[6],  pCtx->dr[7],
                 pCtx->gs.Sel, pCtx->gs.u64Base, pCtx->gs.u32Limit, pCtx->gs.Attr.u, pCtx->cr0,  pCtx->cr2,
                 pCtx->ss.Sel, pCtx->ss.u64Base, pCtx->ss.u32Limit, pCtx->ss.Attr.u, pCtx->cr3,  pCtx->cr4,
                 pCtx->gdtr.pGdt, pCtx->gdtr.cbGdt, pCtx->idtr.pIdt, pCtx->idtr.cbIdt, fEFlags,
                 pCtx->ldtr.Sel, pCtx->ldtr.u64Base, pCtx->ldtr.u32Limit, pCtx->ldtr.Attr.u,
                 pCtx->tr.Sel, pCtx->tr.u64Base, pCtx->tr.u32Limit, pCtx->tr.Attr.u,
                 pCtx->SysEnter.cs, pCtx->SysEnter.eip, pCtx->SysEnter.esp));
    }

    if (fFlags & HM_DUMP_REG_FLAGS_FPU)
    {
        PCX86FXSTATE pFpuCtx = &pCtx->XState.x87;
        Log(("FPU:\n"
            "FCW=%04x FSW=%04x FTW=%02x\n"
            "FOP=%04x FPUIP=%08x CS=%04x Rsrvd1=%04x\n"
            "FPUDP=%04x DS=%04x Rsvrd2=%04x MXCSR=%08x MXCSR_MASK=%08x\n"
            ,
            pFpuCtx->FCW,   pFpuCtx->FSW,   pFpuCtx->FTW,
            pFpuCtx->FOP,   pFpuCtx->FPUIP, pFpuCtx->CS, pFpuCtx->Rsrvd1,
            pFpuCtx->FPUDP, pFpuCtx->DS,    pFpuCtx->Rsrvd2,
            pFpuCtx->MXCSR, pFpuCtx->MXCSR_MASK));
        NOREF(pFpuCtx);
    }

    if (fFlags & HM_DUMP_REG_FLAGS_MSRS)
        Log(("MSR:\n"
            "EFER         =%016RX64\n"
            "PAT          =%016RX64\n"
            "STAR         =%016RX64\n"
            "CSTAR        =%016RX64\n"
            "LSTAR        =%016RX64\n"
            "SFMASK       =%016RX64\n"
            "KERNELGSBASE =%016RX64\n",
            pCtx->msrEFER,
            pCtx->msrPAT,
            pCtx->msrSTAR,
            pCtx->msrCSTAR,
            pCtx->msrLSTAR,
            pCtx->msrSFMASK,
            pCtx->msrKERNELGSBASE));
}

#endif /* VBOX_STRICT */

