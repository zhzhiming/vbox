 ; $Id$
;; @file
; CPUM - Ring-0 Assembly Routines (supporting HM and IEM).
;

;
; Copyright (C) 2006-2024 Oracle and/or its affiliates.
;
; This file is part of VirtualBox base platform packages, as
; available from https://www.virtualbox.org.
;
; This program is free software; you can redistribute it and/or
; modify it under the terms of the GNU General Public License
; as published by the Free Software Foundation, in version 3 of the
; License.
;
; This program is distributed in the hope that it will be useful, but
; WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
; General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program; if not, see <https://www.gnu.org/licenses>.
;
; SPDX-License-Identifier: GPL-3.0-only
;


;*******************************************************************************
;* Header Files                                                                *
;*******************************************************************************
%define RT_ASM_WITH_SEH64
%include "iprt/asmdefs.mac"
%include "VBox/asmdefs.mac"
%include "VBox/vmm/vm.mac"
%include "VBox/err.mac"
%include "VBox/vmm/stam.mac"
%include "CPUMInternal.mac"
%include "iprt/x86.mac"
%include "VBox/vmm/cpum.mac"


BEGINCODE

;;
; Makes sure the EMTs have a FPU state associated with them on hosts where we're
; allowed to use it in ring-0 too.
;
; This ensure that we don't have to allocate the state lazily while trying to execute
; guest code with preemption disabled or worse.
;
; @cproto VMMR0_INT_DECL(void) CPUMR0RegisterVCpuThread(PVMCPU pVCpu);
;
;初始化 VCPU 线程的 FPU 状态（可选）。
;确保线程切换时的 FPU 安全性（防止 FPU 状态泄漏）。
;兼容性处理（跨平台和不同配置）。
;问题：
    ;首次使用 FPU 指令时，CPU 会检查 CR0.TS 位（Task Switched）。
    ;若 TS=1，触发 #NM 异常（需操作系统处理）。
;解决：
    ;提前执行 FPU 指令（如 movdqa）强制加载 FPU 状态。
    ;避免后续 Guest 代码因 TS 位导致性能损失。
BEGINPROC CPUMR0RegisterVCpuThread
        push    xBP
        SEH64_PUSH_xBP
        mov     xBP, xSP
        SEH64_SET_FRAME_xBP 0
SEH64_END_PROLOGUE

;VMM_R0_TOUCH_FPU 宏控制是否初始化 FPU（默认启用）。
%ifdef VMM_R0_TOUCH_FPU
        movdqa  xmm0, xmm0              ; hope this is harmless. ;无实际效果，但会触发 FPU 初始化。 确保 FPU 状态被正确加载
%endif

.return:
        xor     eax, eax                ; paranoia
        leave
        ret
ENDPROC   CPUMR0RegisterVCpuThread


%ifdef VMM_R0_TOUCH_FPU
;;
; Touches the host FPU state.
;
; @uses nothing (well, maybe cr0)
;
 %ifndef RT_ASM_WITH_SEH64 ; workaround for yasm 1.3.0 bug (error: prologue -1 bytes, must be <256)
ALIGNCODE(16)
 %endif
BEGINPROC CPUMR0TouchHostFpu
        push    xBP
        SEH64_PUSH_xBP
        mov     xBP, xSP
        SEH64_SET_FRAME_xBP 0
SEH64_END_PROLOGUE

        movdqa  xmm0, xmm0              ; Hope this is harmless.

        leave
        ret
ENDPROC   CPUMR0TouchHostFpu
%endif ; VMM_R0_TOUCH_FPU


;;
; Saves the host FPU/SSE/AVX state and restores the guest FPU/SSE/AVX state.
;
; @returns  VINF_SUCCESS (0) or VINF_CPUM_HOST_CR0_MODIFIED. (EAX)
; @param    pCpumCpu  x86:[ebp+8] gcc:rdi msc:rcx     CPUMCPU pointer
;
; @remarks  64-bit Windows drivers shouldn't use AVX registers without saving+loading:
;               https://msdn.microsoft.com/en-us/library/windows/hardware/ff545910%28v=vs.85%29.aspx?f=255&MSPPError=-2147217396
;           However the compiler docs have different idea:
;               https://msdn.microsoft.com/en-us/library/9z1stfyw.aspx
;           We'll go with the former for now.
;
%ifndef RT_ASM_WITH_SEH64 ; workaround for yasm 1.3.0 bug (error: prologue -1 bytes, must be <256)
ALIGNCODE(16)
%endif
;保存 Host FPU 状态并恢复 Guest FPU 状态的底层汇编函数，核心职责包括：
;保存 Host FPU/SSE 状态（通过 FXSAVE 或 XSAVE）。
;恢复 Guest FPU/SSE 状态（通过 FXRSTOR 或 XRSTOR）。
;原子化切换 FPU 上下文（确保多核安全）。
;更新 FPU 使用标志（标记 Guest 和 Host 的 FPU 使用状态）。
BEGINPROC cpumR0SaveHostRestoreGuestFPUState
        push    xBP
        SEH64_PUSH_xBP                    ; Windows SEH 栈帧注册（64位）
        mov     xBP, xSP                  ;保存 xBP 并设置栈帧指针（兼容 32/64 位）。
        SEH64_SET_FRAME_xBP 0
SEH64_END_PROLOGUE

        ;
        ; Prologue - xAX+xDX must be free for XSAVE/XRSTOR input.
        ;
%ifdef RT_ARCH_AMD64
 %ifdef RT_OS_WINDOWS
        mov     r11, rcx                  ; Windows x64: 参数1在 rcx
 %else
        mov     r11, rdi                  ; Linux/Mac x64: 参数1在 rdi
 %endif
;pCpumCpu 指向 CPUMCPU 结构体。
;pXState 用于临时存储 XMM 状态指针。
 %define pCpumCpu   r11
 %define pXState    r10
%else
        push    ebx
        push    esi
        mov     ebx, dword [ebp + 8]           ; 参数 pCpumCpu 在 [ebp+8]
 %define pCpumCpu ebx
 %define pXState  esi
%endif

        ; 保存 EFLAGS
        pushf                           ; The darwin kernel can get upset or upset things if an
        ; 禁用中断（避免竞争）
        cli                             ; interrupt occurs while we're doing fxsave/fxrstor/cr0.

        ;
        ; Save the host state.
        ;
        test    dword [pCpumCpu + CPUMCPU.fUseFlags], CPUM_USED_FPU_HOST
        jnz     .already_saved_host            ; 若 Host 已保存 FPU 状态，跳过保存

        ; 确保 FPU 可用, （清除 CR0 的 TS/EM 位）。
        CPUMRZ_TOUCH_FPU_CLEAR_CR0_FPU_TRAPS_SET_RC xCX, xAX, pCpumCpu ; xCX is the return value for VT-x; xAX is scratch.

        ; 宏：保存 Host FPU 状态（FXSAVE/XSAVE）
        CPUMR0_SAVE_HOST

%ifdef VBOX_WITH_KERNEL_USING_XMM
        jmp     .load_guest
%endif
.already_saved_host:
%ifdef VBOX_WITH_KERNEL_USING_XMM
        ; If we didn't save the host state, we must save the non-volatile XMM registers.
        lea     pXState, [pCpumCpu + CPUMCPU.Host.XState]
        stmxcsr [pXState + X86FXSTATE.MXCSR]                  ; 保存 Host MXCSR
        movdqa  [pXState + X86FXSTATE.xmm6 ], xmm6            ; 保存非易失 XMM6-XMM15
        movdqa  [pXState + X86FXSTATE.xmm7 ], xmm7
        movdqa  [pXState + X86FXSTATE.xmm8 ], xmm8
        movdqa  [pXState + X86FXSTATE.xmm9 ], xmm9
        movdqa  [pXState + X86FXSTATE.xmm10], xmm10
        movdqa  [pXState + X86FXSTATE.xmm11], xmm11
        movdqa  [pXState + X86FXSTATE.xmm12], xmm12
        movdqa  [pXState + X86FXSTATE.xmm13], xmm13
        movdqa  [pXState + X86FXSTATE.xmm14], xmm14
        movdqa  [pXState + X86FXSTATE.xmm15], xmm15

        ;
        ; Load the guest state.
        ;
.load_guest:
%endif
        CPUMR0_LOAD_GUEST                                    ; 宏：加载 Guest FPU 状态（FXRSTOR/XRSTOR）

%ifdef VBOX_WITH_KERNEL_USING_XMM
        ; Restore the non-volatile xmm registers. ASSUMING 64-bit host.
        lea     pXState, [pCpumCpu + CPUMCPU.Host.XState]
        movdqa  xmm6,  [pXState + X86FXSTATE.xmm6]          ; 恢复非易失 XMM6-XMM15
        movdqa  xmm7,  [pXState + X86FXSTATE.xmm7]
        movdqa  xmm8,  [pXState + X86FXSTATE.xmm8]
        movdqa  xmm9,  [pXState + X86FXSTATE.xmm9]
        movdqa  xmm10, [pXState + X86FXSTATE.xmm10]
        movdqa  xmm11, [pXState + X86FXSTATE.xmm11]
        movdqa  xmm12, [pXState + X86FXSTATE.xmm12]
        movdqa  xmm13, [pXState + X86FXSTATE.xmm13]
        movdqa  xmm14, [pXState + X86FXSTATE.xmm14]
        movdqa  xmm15, [pXState + X86FXSTATE.xmm15]
        ldmxcsr        [pXState + X86FXSTATE.MXCSR]         ; 恢复 Host MXCSR
%endif

        ;CPUM_USED_FPU_GUEST：标记 Guest 正在使用 FPU。
        ;CPUM_USED_FPU_SINCE_REM：标记自上次 REM（重定向执行模式）后 FPU 被使用。
        ;CPUM_USED_FPU_HOST：标记 Host 曾使用 FPU（需后续恢复）。
        or      dword [pCpumCpu + CPUMCPU.fUseFlags], (CPUM_USED_FPU_GUEST | CPUM_USED_FPU_SINCE_REM | CPUM_USED_FPU_HOST)
        ;fUsedFpuGuest：用于 Guest FPU 使用追踪。
        mov     byte [pCpumCpu + CPUMCPU.Guest.fUsedFpuGuest], 1
        popf

        mov     eax, ecx                         ; 返回值（CR0 相关状态）
.return:
%ifdef RT_ARCH_X86
        pop     esi
        pop     ebx
%endif
        leave                                    ; 恢复栈帧（mov esp, ebp + pop ebp）
        ret
ENDPROC   cpumR0SaveHostRestoreGuestFPUState


;;
; Saves the guest FPU/SSE/AVX state and restores the host FPU/SSE/AVX state.
;
; @param    pCpumCpu  x86:[ebp+8] gcc:rdi msc:rcx     CPUMCPU pointer
;
; @remarks  64-bit Windows drivers shouldn't use AVX registers without saving+loading:
;               https://msdn.microsoft.com/en-us/library/windows/hardware/ff545910%28v=vs.85%29.aspx?f=255&MSPPError=-2147217396
;           However the compiler docs have different idea:
;               https://msdn.microsoft.com/en-us/library/9z1stfyw.aspx
;           We'll go with the former for now.
;
%ifndef RT_ASM_WITH_SEH64 ; workaround for yasm 1.3.0 bug (error: prologue -1 bytes, must be <256)
ALIGNCODE(16)
%endif
;
;用于 保存 Guest FPU 状态并恢复 Host FPU 状态的底层汇编函数，核心职责包括：
;保存 Guest FPU/SSE 状态（通过 FXSAVE 或 XSAVE）。
;恢复 Host FPU/SSE 状态（通过 FXRSTOR 或 XRSTOR）。
;原子化切换 FPU 上下文（确保多核安全）。
BEGINPROC cpumR0SaveGuestRestoreHostFPUState
        push    xBP
        SEH64_PUSH_xBP            ; Windows SEH 栈帧注册（64位）
        mov     xBP, xSP          ;保存 xBP 并设置栈帧指针（兼容 32/64 位）。
        SEH64_SET_FRAME_xBP 0     ;
SEH64_END_PROLOGUE

        ;
        ; Prologue - xAX+xDX must be free for XSAVE/XRSTOR input.
        ;
%ifdef RT_ARCH_AMD64
 ;x64: 根据 ABI 从 rcx (Windows) 或 rdi (SysV) 加载参数。
 ;x86: 通过栈传递参数（[ebp+8]）。
 %ifdef RT_OS_WINDOWS
        mov     r11, rcx         ; Windows x64: 参数1在 rcx
 %else
        mov     r11, rdi         ; Linux/Mac x64: 参数1在 rdi
 %endif
 ;寄存器分配：
     ;pCpumCpu 指向 CPUMCPU 结构体。
     ;pXState 用于临时存储 XMM 状态指针。
 %define pCpumCpu   r11          ; pCpumCpu 存入 r11
 %define pXState    r10          ; pXState 使用 r10
%else
        push    ebx
        push    esi
        mov     ebx, dword [ebp + 8]           ; 参数 pCpumCpu 在 [ebp+8]
 %define pCpumCpu   ebx
 %define pXState    esi
%endif
        pushf                           ; The darwin kernel can get upset or upset things if an
        cli                             ; interrupt occurs while we're doing fxsave/fxrstor/cr0.

 ;处理 Host XMM 寄存器（仅内核使用 XMM 时）
 ;若宿主机内核使用 XMM 寄存器（如 Windows 某些驱动），需先保存 Host 的 XMM 状态。
;操作：
  ;将非易失 XMM 寄存器（xmm6-xmm15）保存到 Host.XState。
 %ifdef VBOX_WITH_KERNEL_USING_XMM
        ;
        ; Copy non-volatile XMM registers to the host state so we can use
        ; them while saving the guest state (we've gotta do this anyway).
        ;
        lea     pXState, [pCpumCpu + CPUMCPU.Host.XState]
        stmxcsr [pXState + X86FXSTATE.MXCSR]
        movdqa  [pXState + X86FXSTATE.xmm6], xmm6
        movdqa  [pXState + X86FXSTATE.xmm7], xmm7
        movdqa  [pXState + X86FXSTATE.xmm8], xmm8
        movdqa  [pXState + X86FXSTATE.xmm9], xmm9
        movdqa  [pXState + X86FXSTATE.xmm10], xmm10
        movdqa  [pXState + X86FXSTATE.xmm11], xmm11
        movdqa  [pXState + X86FXSTATE.xmm12], xmm12
        movdqa  [pXState + X86FXSTATE.xmm13], xmm13
        movdqa  [pXState + X86FXSTATE.xmm14], xmm14
        movdqa  [pXState + X86FXSTATE.xmm15], xmm15
 %endif

        ;
        ; Save the guest state if necessary.
        ;
        test    dword [pCpumCpu + CPUMCPU.fUseFlags], CPUM_USED_FPU_GUEST    ; 若 Guest 未使用 FPU，跳过保存
        jz      .load_only_host

 %ifdef VBOX_WITH_KERNEL_USING_XMM
        ; Load the guest XMM register values we already saved in HMR0VMXStartVMWrapXMM.
        lea     pXState, [pCpumCpu + CPUMCPU.Guest.XState]
        movdqa  xmm0,  [pXState + X86FXSTATE.xmm0]                           ; 恢复 Guest XMM0-XMM15
        movdqa  xmm1,  [pXState + X86FXSTATE.xmm1]
        movdqa  xmm2,  [pXState + X86FXSTATE.xmm2]
        movdqa  xmm3,  [pXState + X86FXSTATE.xmm3]
        movdqa  xmm4,  [pXState + X86FXSTATE.xmm4]
        movdqa  xmm5,  [pXState + X86FXSTATE.xmm5]
        movdqa  xmm6,  [pXState + X86FXSTATE.xmm6]
        movdqa  xmm7,  [pXState + X86FXSTATE.xmm7]
        movdqa  xmm8,  [pXState + X86FXSTATE.xmm8]
        movdqa  xmm9,  [pXState + X86FXSTATE.xmm9]
        movdqa  xmm10, [pXState + X86FXSTATE.xmm10]
        movdqa  xmm11, [pXState + X86FXSTATE.xmm11]
        movdqa  xmm12, [pXState + X86FXSTATE.xmm12]
        movdqa  xmm13, [pXState + X86FXSTATE.xmm13]
        movdqa  xmm14, [pXState + X86FXSTATE.xmm14]
        movdqa  xmm15, [pXState + X86FXSTATE.xmm15]
        ldmxcsr        [pXState + X86FXSTATE.MXCSR]                         ; 恢复 Guest MXCSR
 %endif
        CPUMR0_SAVE_GUEST                                                    ; 宏：保存 Guest FPU 状态（FXSAVE/XSAVE）

        ;
        ; Load the host state.
        ;
.load_only_host:
        CPUMR0_LOAD_HOST                                                    ;从 Host.XState 恢复 FPU/SSE 寄存器。

        ; Restore the CR0 value we saved in cpumR0SaveHostRestoreGuestFPUState or
        ; in cpumRZSaveHostFPUState.
        mov     xCX, [pCpumCpu + CPUMCPU.Host.cr0Fpu]
		;恢复 Host 的 CR0 值（确保 FPU 指令可用）。
        ;优化：仅当 TS (Task Switched) 或 EM (Emulation) 位变化时才写入 CR0。
        CPUMRZ_RESTORE_CR0_IF_TS_OR_EM_SET xCX                              ; 仅当 TS/EM 置位时写 CR0
        ;清除 Guest 和 Host 的 FPU 使用标记。
        and     dword [pCpumCpu + CPUMCPU.fUseFlags], ~(CPUM_USED_FPU_GUEST | CPUM_USED_FPU_HOST)
        ;重置 fUsedFpuGuest（用于 Guest FPU 使用追踪）。
        mov     byte [pCpumCpu + CPUMCPU.Guest.fUsedFpuGuest], 0

        popf                                                                 ; 恢复 EFLAGS（包括中断状态）
%ifdef RT_ARCH_X86
        pop     esi                                                          ; 32位下恢复 esi/ebx
        pop     ebx
%endif
        leave                                                                 ; 恢复栈帧（mov esp, ebp + pop ebp）
        ret
%undef pCpumCpu
%undef pXState
ENDPROC   cpumR0SaveGuestRestoreHostFPUState


%if ARCH_BITS == 32
 %ifdef VBOX_WITH_64_BITS_GUESTS
;;
; Restores the host's FPU/SSE/AVX state from pCpumCpu->Host.
;
; @param    pCpumCpu  x86:[ebp+8] gcc:rdi msc:rcx     CPUMCPU pointer
;
;恢复宿主 FPU/SSE 状态（通过 FXRSTOR 或 XRSTOR）。
;恢复宿主 CR0 寄存器（确保 FPU 控制位正确）。
;原子化更新状态标志（避免多核竞争）。
;
  %ifndef RT_ASM_WITH_SEH64 ; workaround for yasm 1.3.0 bug (error: prologue -1 bytes, must be <256)
ALIGNCODE(16)
  %endif
BEGINPROC cpumR0RestoreHostFPUState
        ;
        ; Prologue - xAX+xDX must be free for XSAVE/XRSTOR input.
        ;
        push    ebp
        mov     ebp, esp   ;ebp=esp
        push    ebx
        push    esi
        mov     ebx, dword [ebp + 8] ; 参数 pCpumCpu 存入 ebx
  %define pCpumCpu ebx
  %define pXState  esi

        ;
        ; Restore host CPU state.
        ;
		;pushf + cli 确保操作原子性（防止中断干扰 FPU 状态恢复）。
        pushf                           ; The darwin kernel can get upset or upset things if an ; 保存 EFLAGS
        cli                             ; interrupt occurs while we're doing fxsave/fxrstor/cr0.; 禁用中断（避免竞争）

        ;展开后通常为 FXRSTOR 或 XRSTOR 指令，从 pCpumCpu->Host.fpuState 恢复宿主 FPU/SSE 状态。
        CPUMR0_LOAD_HOST                ; 宏：加载宿主 FPU 状态

        ; Restore the CR0 value we saved in cpumR0SaveHostRestoreGuestFPUState or
        ; in cpumRZSaveHostFPUState.
        ;; @todo What about XCR0?
		;cr0Fpu：保存的宿主 CR0 值（可能包含 TS（Task Switched）或 EM（Emulation）标志位）。
        mov     xCX, [pCpumCpu + CPUMCPU.Host.cr0Fpu]          ; 加载保存的宿主 CR0
        ;CPUMRZ_RESTORE_CR0_IF_TS_OR_EM_SET 宏：
          ;若 TS=1 或 EM=1，则恢复 CR0（否则跳过以优化性能）
        CPUMRZ_RESTORE_CR0_IF_TS_OR_EM_SET xCX                 ; 条件恢复 CR0

        and     dword [pCpumCpu + CPUMCPU.fUseFlags], ~CPUM_USED_FPU_HOST   ;清除 CPUM_USED_FPU_HOST 标志（标记宿主 FPU 状态已恢复）。
        popf                                                                ;popf 恢复原始中断状态

        pop     esi
        pop     ebx
        leave
        ret
  %undef pCpumCPu
  %undef pXState
ENDPROC   cpumR0RestoreHostFPUState
 %endif ; VBOX_WITH_64_BITS_GUESTS
%endif  ; ARCH_BITS == 32

