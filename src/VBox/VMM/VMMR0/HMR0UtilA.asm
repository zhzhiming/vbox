; $Id$
;; @file
; HM - Ring-0 VMX & SVM Helpers.
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

;*********************************************************************************************************************************
;*  Header Files                                                                                                                 *
;*********************************************************************************************************************************
%include "VBox/asmdefs.mac"
%include "VBox/err.mac"
%include "VBox/vmm/hm_vmx.mac"
%include "iprt/x86.mac"



BEGINCODE

;;
; Executes VMWRITE, 64-bit value.
;
; @returns VBox status code.
; @param   idxField   x86: [ebp + 08h]  msc: rcx  gcc: rdi   VMCS index.
; @param   u64Data    x86: [ebp + 0ch]  msc: rdx  gcc: rsi   VM field value.
;
;向VMCS（Virtual Machine Control Structure）写入64位数据
;vmwrite指令用于将数据（源操作数）写入VMCS的指定字段（目标操作数）
;架构        参数传递方式                            关键代码段
;x64 (GCC)	寄存器传参（rdi=字段索引，rsi=数据）	and edi, 0ffffffffh  vmwrite rdi, rsi
;x64 (其他)	寄存器传参（rcx=字段索引，rdx=数据）	and ecx, 0ffffffffh  vmwrite rcx, rdx
;x86	     栈传参（[esp+4]=字段索引）	           vmwrite ecx, [edx]（低32位）  vmwrite ecx, [edx + 4]（高32位）
ALIGNCODE(16)
BEGINPROC VMXWriteVmcs64
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    and         edi, 0ffffffffh ;确保32位索引值的高32位清零（x64架构）
    xor         rax, rax
    vmwrite     rdi, rsi
 %else
    and         ecx, 0ffffffffh
    xor         rax, rax
    vmwrite     rcx, rdx
 %endif
%else  ; RT_ARCH_X86            ;：x86模式下分两次写入低/高32位，通过inc ecx递增字段索引
    mov         ecx, [esp + 4]          ; idxField
    lea         edx, [esp + 8]          ; &u64Data
    vmwrite     ecx, [edx]              ; low dword
    jz          .done
    jc          .done
    inc         ecx
    xor         eax, eax
    vmwrite     ecx, [edx + 4]          ; high dword
.done:
%endif ; RT_ARCH_X86
    ;通过检查CF（进位标志）和ZF（零标志）判断操作是否成功，
    ;jnc和jnz分别检测CF和ZF，若CF=1表示VMCS指针无效，ZF=1表示字段无效
    jnc         .valid_vmcs
    mov         eax, VERR_VMX_INVALID_VMCS_PTR
    ret
.valid_vmcs:
    jnz         .the_end
    mov         eax, VERR_VMX_INVALID_VMCS_FIELD
.the_end:
    ret
ENDPROC VMXWriteVmcs64


;;
; Executes VMREAD, 64-bit value.
;
; @returns VBox status code.
; @param   idxField        VMCS index.
; @param   pData           Where to store VM field value.
;
;DECLASM(int) VMXReadVmcs64(uint32_t idxField, uint64_t *pData);
ALIGNCODE(16)
BEGINPROC VMXReadVmcs64
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    and         edi, 0ffffffffh
    xor         rax, rax   ;xor rax/eax, rax/eax用于清除返回值寄存器
    vmread      [rsi], rdi
 %else
    and         ecx, 0ffffffffh
    xor         rax, rax
    vmread      [rdx], rcx
 %endif
%else  ; RT_ARCH_X86
    mov         ecx, [esp + 4]          ; idxField
    mov         edx, [esp + 8]          ; pData
    vmread      [edx], ecx              ; low dword
    jz          .done
    jc          .done
    inc         ecx                     ;x86模式下通过inc ecx递增字段索引，分两次读取64位数据
    xor         eax, eax
    vmread      [edx + 4], ecx          ; high dword
.done:
%endif ; RT_ARCH_X86
    jnc         .valid_vmcs
    mov         eax, VERR_VMX_INVALID_VMCS_PTR
    ret
.valid_vmcs:
    jnz         .the_end
    mov         eax, VERR_VMX_INVALID_VMCS_FIELD
.the_end:
    ret
ENDPROC VMXReadVmcs64


;;
; Executes VMREAD, 32-bit value.
;
; @returns VBox status code.
; @param   idxField        VMCS index.
; @param   pu32Data        Where to store VM field value.
;
;DECLASM(int) VMXReadVmcs32(uint32_t idxField, uint32_t *pu32Data);
;vmread指令从VMCS的指定字段（目标操作数）读取数据到目标地址（源操作数）
ALIGNCODE(16)
BEGINPROC VMXReadVmcs32
%ifdef RT_ARCH_AMD64
 ;GCC: ：edi（字段索引），rsi（数据存储地址）
 %ifdef ASM_CALL64_GCC
    and     edi, 0ffffffffh ;and edi, 0ffffffffh确保索引为32位，vmread r10, rdi读取数据并存储到[rsi]
    xor     rax, rax
    vmread  r10, rdi
    mov     [rsi], r10d
 %else
    and     ecx, 0ffffffffh ;ecx（字段索引），rdx（数据存储地址）
    xor     rax, rax
    vmread  r10, rcx
    mov     [rdx], r10d
 %endif
%else  ; RT_ARCH_X86
    mov     ecx, [esp + 4]              ; idxField
    mov     edx, [esp + 8]              ; pu32Data
    xor     eax, eax
    vmread  [edx], ecx
%endif ; RT_ARCH_X86
    jnc     .valid_vmcs
    mov     eax, VERR_VMX_INVALID_VMCS_PTR
    ret
.valid_vmcs:
    jnz     .the_end
    mov     eax, VERR_VMX_INVALID_VMCS_FIELD
.the_end:
    ret
ENDPROC VMXReadVmcs32


;;
; Executes VMWRITE, 32-bit value.
;
; @returns VBox status code.
; @param   idxField        VMCS index.
; @param   u32Data         Where to store VM field value.
;
;DECLASM(int) VMXWriteVmcs32(uint32_t idxField, uint32_t u32Data);
ALIGNCODE(16)
BEGINPROC VMXWriteVmcs32
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    and     edi, 0ffffffffh
    and     esi, 0ffffffffh
    xor     rax, rax
    vmwrite rdi, rsi
 %else
    and     ecx, 0ffffffffh
    and     edx, 0ffffffffh
    xor     rax, rax
    vmwrite rcx, rdx
 %endif
%else  ; RT_ARCH_X86
    mov     ecx, [esp + 4]              ; idxField
    mov     edx, [esp + 8]              ; u32Data
    xor     eax, eax
    vmwrite ecx, edx
%endif ; RT_ARCH_X86
    jnc     .valid_vmcs
    mov     eax, VERR_VMX_INVALID_VMCS_PTR
    ret
.valid_vmcs:
    jnz     .the_end
    mov     eax, VERR_VMX_INVALID_VMCS_FIELD
.the_end:
    ret
ENDPROC VMXWriteVmcs32


;;
; Executes VMXON.
;
; @returns VBox status code.
; @param   HCPhysVMXOn      Physical address of VMXON structure.
;
;DECLASM(int) VMXEnable(RTHCPHYS HCPhysVMXOn);
;vmxon指令用于激活VMX操作模式，需要传入VMXON区域的物理地址作为参数
BEGINPROC VMXEnable
%ifdef RT_ARCH_AMD64
    xor     rax, rax
 %ifdef ASM_CALL64_GCC
    push    rdi ;（rdi=VMXON区域地址）
 %else
    push    rcx ;（rcx=VMXON区域地址）
 %endif
    vmxon   [rsp]
%else  ; RT_ARCH_X86
    xor     eax, eax
    vmxon   [esp + 4]    ;（[esp+4]=VMXON区域地址）
%endif ; RT_ARCH_X86
    jnc     .good
    mov     eax, VERR_VMX_INVALID_VMXON_PTR
    jmp     .the_end

.good:
    jnz     .the_end
    mov     eax, VERR_VMX_VMXON_FAILED

.the_end:
%ifdef RT_ARCH_AMD64
    add     rsp, 8
%endif
    ret
ENDPROC VMXEnable


;;
; Executes VMXOFF.
;
;DECLASM(void) VMXDisable(void);
BEGINPROC VMXDisable
    vmxoff
.the_end:
    ret
ENDPROC VMXDisable


;;
; Executes VMCLEAR.
;
; @returns VBox status code.
; @param   HCPhysVmcs     Physical address of VM control structure.
;
;DECLASM(int) VMXClearVmcs(RTHCPHYS HCPhysVmcs);
ALIGNCODE(16)
BEGINPROC VMXClearVmcs
%ifdef RT_ARCH_AMD64
    xor     rax, rax
 %ifdef ASM_CALL64_GCC
    push    rdi
 %else
    push    rcx
 %endif
    vmclear [rsp]
%else  ; RT_ARCH_X86
    xor     eax, eax
    vmclear [esp + 4]
%endif ; RT_ARCH_X86
    jnc     .the_end
    mov     eax, VERR_VMX_INVALID_VMCS_PTR
.the_end:
%ifdef RT_ARCH_AMD64
    add     rsp, 8
%endif
    ret
ENDPROC VMXClearVmcs


;;
; Executes VMPTRLD.
;
; @returns VBox status code.
; @param   HCPhysVmcs     Physical address of VMCS structure.
;
;DECLASM(int) VMXLoadVmcs(RTHCPHYS HCPhysVmcs);
ALIGNCODE(16)
BEGINPROC VMXLoadVmcs
%ifdef RT_ARCH_AMD64
    xor     rax, rax
 %ifdef ASM_CALL64_GCC
    push    rdi
 %else
    push    rcx
 %endif
    vmptrld [rsp]
%else
    xor     eax, eax
    vmptrld [esp + 4]
%endif
    jnc     .the_end
    mov     eax, VERR_VMX_INVALID_VMCS_PTR
.the_end:
%ifdef RT_ARCH_AMD64
    add     rsp, 8
%endif
    ret
ENDPROC VMXLoadVmcs


;;
; Executes VMPTRST.
;
; @returns VBox status code.
; @param    [esp + 04h]  gcc:rdi  msc:rcx   Param 1 - First parameter - Address that will receive the current pointer.
;
;DECLASM(int) VMXGetCurrentVmcs(RTHCPHYS *pVMCS);
BEGINPROC VMXGetCurrentVmcs
%ifdef RT_OS_OS2
    mov     eax, VERR_NOT_SUPPORTED
    ret
%else
 %ifdef RT_ARCH_AMD64
  %ifdef ASM_CALL64_GCC
    vmptrst qword [rdi]
  %else
    vmptrst qword [rcx]
  %endif
 %else
    vmptrst qword [esp+04h]
 %endif
    xor     eax, eax
.the_end:
    ret
%endif
ENDPROC VMXGetCurrentVmcs


;;
; Invalidate a page using INVEPT.
;
; @param   enmTlbFlush  msc:ecx  gcc:edi  x86:[esp+04]  Type of flush.
; @param   pDescriptor  msc:edx  gcc:esi  x86:[esp+08]  Descriptor pointer.
;
;DECLASM(int) VMXR0InvEPT(VMXTLBFLUSHEPT enmTlbFlush, uint64_t *pDescriptor);
;INVEPT指令用于使EPT缓存失效，需传入EPT指针和操作类型（单上下文或全局刷新
BEGINPROC VMXR0InvEPT
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    and         edi, 0ffffffffh
    xor         rax, rax
;    invept      rdi, qword [rsi]
    DB          0x66, 0x0F, 0x38, 0x80, 0x3E ;手动编码 31.3 VMX INSTRUCTIONS 0x3E:指定操作数类型，如[ESI]内存寻址
 %else
    and         ecx, 0ffffffffh
    xor         rax, rax
;    invept      rcx, qword [rdx]
    DB          0x66, 0x0F, 0x38, 0x80, 0xA
 %endif
%else
    mov         ecx, [esp + 4]
    mov         edx, [esp + 8]
    xor         eax, eax
;    invept      ecx, qword [edx]
    DB          0x66, 0x0F, 0x38, 0x80, 0xA
%endif
    jnc         .valid_vmcs
    mov         eax, VERR_VMX_INVALID_VMCS_PTR
    ret
.valid_vmcs:
    jnz         .the_end
    mov         eax, VERR_INVALID_PARAMETER
.the_end:
    ret
ENDPROC VMXR0InvEPT


;;
; Invalidate a page using INVVPID.
;
; @param   enmTlbFlush  msc:ecx  gcc:edi  x86:[esp+04]  Type of flush
; @param   pDescriptor  msc:edx  gcc:esi  x86:[esp+08]  Descriptor pointer
;
;DECLASM(int) VMXR0InvVPID(VMXTLBFLUSHVPID enmTlbFlush, uint64_t *pDescriptor);
;用于刷新虚拟处理器标识符（VPID）相关的TLB条目
BEGINPROC VMXR0InvVPID
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    and         edi, 0ffffffffh
    xor         rax, rax
;    invvpid     rdi, qword [rsi]
    DB          0x66, 0x0F, 0x38, 0x81, 0x3E
 %else
    and         ecx, 0ffffffffh
    xor         rax, rax
;    invvpid     rcx, qword [rdx]
    DB          0x66, 0x0F, 0x38, 0x81, 0xA
 %endif
%else
    mov         ecx, [esp + 4]
    mov         edx, [esp + 8]
    xor         eax, eax
;    invvpid     ecx, qword [edx]
    DB          0x66, 0x0F, 0x38, 0x81, 0xA
%endif
    jnc         .valid_vmcs
    mov         eax, VERR_VMX_INVALID_VMCS_PTR
    ret
.valid_vmcs:
    jnz         .the_end
    mov         eax, VERR_INVALID_PARAMETER
.the_end:
    ret
ENDPROC VMXR0InvVPID


%if GC_ARCH_BITS == 64
;;
; Executes INVLPGA.
;
; @param   pPageGC  msc:rcx  gcc:rdi  x86:[esp+04]  Virtual page to invalidate
; @param   uASID    msc:rdx  gcc:rsi  x86:[esp+0C]  Tagged TLB id
;
;DECLASM(void) SVMR0InvlpgA(RTGCPTR pPageGC, uint32_t uASID);
;是用于 AMD SVM（Secure Virtual Machine）架构的 TLB
;（Translation Lookaside Buffer）管理指令封装，其核心功能为通过 
;invlpga 指令刷新指定虚拟地址及 ASID（Address Space Identifier）
;关联的 TLB 条目，确保虚拟化环境中内存隔离性
BEGINPROC SVMR0InvlpgA
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    mov     rax, rdi
    mov     rcx, rsi
 %else
    mov     rax, rcx
    mov     rcx, rdx
 %endif
%else
    mov     eax, [esp + 4]
    mov     ecx, [esp + 0Ch]
%endif
    invlpga [xAX], ecx  ;invlpga [address], ASID，其中： address：需刷新的虚拟地址（由 rax 提供）。 ASID：地址空间标识符（由 ecx 提供）
    ret
ENDPROC SVMR0InvlpgA

%else ; GC_ARCH_BITS != 64
;;
; Executes INVLPGA
;
; @param   pPageGC  msc:ecx  gcc:edi  x86:[esp+04]  Virtual page to invalidate
; @param   uASID    msc:edx  gcc:esi  x86:[esp+08]  Tagged TLB id
;
;DECLASM(void) SVMR0InvlpgA(RTGCPTR pPageGC, uint32_t uASID);
BEGINPROC SVMR0InvlpgA
%ifdef RT_ARCH_AMD64
 %ifdef ASM_CALL64_GCC
    movzx   rax, edi
    mov     ecx, esi
 %else
    ; from http://www.cs.cmu.edu/~fp/courses/15213-s06/misc/asm64-handout.pdf:
    ; "Perhaps unexpectedly, instructions that move or generate 32-bit register
    ;  values also set the upper 32 bits of the register to zero. Consequently
    ;  there is no need for an instruction movzlq."
    mov     eax, ecx
    mov     ecx, edx
 %endif
%else
    mov     eax, [esp + 4]
    mov     ecx, [esp + 8]
%endif
    invlpga [xAX], ecx
    ret
ENDPROC SVMR0InvlpgA

%endif ; GC_ARCH_BITS != 64

