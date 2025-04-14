/* $Id$ */
/** @file
 * DBGF - Debugger Facility, R0 breakpoint management part.
 */

/*
 * Copyright (C) 2020-2024 Oracle and/or its affiliates.
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
#define LOG_GROUP LOG_GROUP_DBGF
#include "DBGFInternal.h"
#include <VBox/vmm/gvm.h>
#include <VBox/vmm/gvmm.h>
#include <VBox/vmm/vmm.h>

#include <VBox/log.h>
#include <VBox/sup.h>
#include <iprt/asm.h>
#include <iprt/assert.h>
#include <iprt/errcore.h>
#include <iprt/ctype.h>
#include <iprt/mem.h>
#include <iprt/memobj.h>
#include <iprt/process.h>
#include <iprt/string.h>

#include "dtrace/VBoxVMM.h"


/*********************************************************************************************************************************
*   Internal Functions                                                                                                           *
*********************************************************************************************************************************/

/**
 * Used by DBGFR0InitPerVM() to initialize the breakpoint manager.
 *
 * @param   pGVM        The global (ring-0) VM structure.
 */
#if 0
    对断点管理相关的所有内核内存对象进行原子化清零初始化。其设计体现了三个核心原则：
    确定状态原则：所有内存句柄必须初始化为NIL_RTR0MEMOBJ
    最小权限原则：指针字段在初始化阶段强制置NULL（注释状态）
    拓扑隔离原则：L1/L2内存结构分层初始化
#endif

//L1 Location Cache--指向->L2 Chunk Table--管理->BP Chunks--包含->BP Owner Metadata
DECLHIDDEN(void) dbgfR0BpInit(PGVM pGVM)
{
    pGVM->dbgfr0.s.hMemObjBpOwners = NIL_RTR0MEMOBJ;
    pGVM->dbgfr0.s.hMapObjBpOwners = NIL_RTR0MEMOBJ;
    //pGVM->dbgfr0.s.paBpOwnersR0    = NULL;

    for (uint32_t i = 0; i < RT_ELEMENTS(pGVM->dbgfr0.s.aBpChunks); i++)
    {
        PDBGFBPCHUNKR0 pBpChunk = &pGVM->dbgfr0.s.aBpChunks[i];

        pBpChunk->hMemObj          = NIL_RTR0MEMOBJ;
        pBpChunk->hMapObj          = NIL_RTR0MEMOBJ;
        //pBpChunk->paBpBaseSharedR0 = NULL;
        //pBpChunk->paBpBaseR0Only   = NULL;
    }

    for (uint32_t i = 0; i < RT_ELEMENTS(pGVM->dbgfr0.s.aBpL2TblChunks); i++)
    {
        PDBGFBPL2TBLCHUNKR0 pL2Chunk = &pGVM->dbgfr0.s.aBpL2TblChunks[i];

        pL2Chunk->hMemObj          = NIL_RTR0MEMOBJ;
        pL2Chunk->hMapObj          = NIL_RTR0MEMOBJ;
        //pL2Chunk->paBpL2TblBaseSharedR0 = NULL;
    }

    pGVM->dbgfr0.s.hMemObjBpLocL1     = NIL_RTR0MEMOBJ;
    pGVM->dbgfr0.s.hMapObjBpLocL1     = NIL_RTR0MEMOBJ;
    pGVM->dbgfr0.s.hMemObjBpLocPortIo = NIL_RTR0MEMOBJ;
    pGVM->dbgfr0.s.hMapObjBpLocPortIo = NIL_RTR0MEMOBJ;
    //pGVM->dbgfr0.s.paBpLocL1R0      = NULL;
    //pGVM->dbgfr0.s.paBpLocPortIoR0  = NULL;
    //pGVM->dbgfr0.s.fInit            = false;
}


/**
 * Used by DBGFR0CleanupVM to destroy the breakpoint manager.
 *
 * This is done during VM cleanup so that we're sure there are no active threads
 * using the breakpoint code.
 *
 * @param   pGVM        The global (ring-0) VM structure.
 */
DECLHIDDEN(void) dbgfR0BpDestroy(PGVM pGVM)
{
    if (pGVM->dbgfr0.s.hMemObjBpOwners != NIL_RTR0MEMOBJ)
    {
        Assert(pGVM->dbgfr0.s.hMapObjBpOwners != NIL_RTR0MEMOBJ);
        AssertPtr(pGVM->dbgfr0.s.paBpOwnersR0);

        RTR0MEMOBJ hMemObj = pGVM->dbgfr0.s.hMapObjBpOwners;
        pGVM->dbgfr0.s.hMapObjBpOwners = NIL_RTR0MEMOBJ;
        RTR0MemObjFree(hMemObj, true);

        hMemObj = pGVM->dbgfr0.s.hMemObjBpOwners;
        pGVM->dbgfr0.s.hMemObjBpOwners = NIL_RTR0MEMOBJ;
        RTR0MemObjFree(hMemObj, true);
    }

    if (pGVM->dbgfr0.s.fInit)
    {
        Assert(pGVM->dbgfr0.s.hMemObjBpLocL1 != NIL_RTR0MEMOBJ);
        AssertPtr(pGVM->dbgfr0.s.paBpLocL1R0);

        /*
         * Free all allocated memory and ring-3 mapping objects.
         */
        RTR0MEMOBJ hMemObj = pGVM->dbgfr0.s.hMemObjBpLocL1;
        pGVM->dbgfr0.s.hMemObjBpLocL1 = NIL_RTR0MEMOBJ;
        pGVM->dbgfr0.s.paBpLocL1R0    = NULL;
        RTR0MemObjFree(hMemObj, true);

        if (pGVM->dbgfr0.s.paBpLocPortIoR0)
        {
            Assert(pGVM->dbgfr0.s.hMemObjBpLocPortIo != NIL_RTR0MEMOBJ);
            Assert(pGVM->dbgfr0.s.hMapObjBpLocPortIo != NIL_RTR0MEMOBJ);

            hMemObj = pGVM->dbgfr0.s.hMapObjBpLocPortIo;
            pGVM->dbgfr0.s.hMapObjBpLocPortIo = NIL_RTR0MEMOBJ;
            RTR0MemObjFree(hMemObj, true);

            hMemObj = pGVM->dbgfr0.s.hMemObjBpLocPortIo;
            pGVM->dbgfr0.s.hMemObjBpLocPortIo = NIL_RTR0MEMOBJ;
            pGVM->dbgfr0.s.paBpLocPortIoR0    = NULL;
            RTR0MemObjFree(hMemObj, true);
        }

        for (uint32_t i = 0; i < RT_ELEMENTS(pGVM->dbgfr0.s.aBpChunks); i++)
        {
            PDBGFBPCHUNKR0 pBpChunk = &pGVM->dbgfr0.s.aBpChunks[i];

            if (pBpChunk->hMemObj != NIL_RTR0MEMOBJ)
            {
                Assert(pBpChunk->hMapObj != NIL_RTR0MEMOBJ);

                pBpChunk->paBpBaseSharedR0 = NULL;
                pBpChunk->paBpBaseR0Only   = NULL;

                hMemObj = pBpChunk->hMapObj;
                pBpChunk->hMapObj = NIL_RTR0MEMOBJ;
                RTR0MemObjFree(hMemObj, true);

                hMemObj = pBpChunk->hMemObj;
                pBpChunk->hMemObj = NIL_RTR0MEMOBJ;
                RTR0MemObjFree(hMemObj, true);
            }
        }

        for (uint32_t i = 0; i < RT_ELEMENTS(pGVM->dbgfr0.s.aBpL2TblChunks); i++)
        {
            PDBGFBPL2TBLCHUNKR0 pL2Chunk = &pGVM->dbgfr0.s.aBpL2TblChunks[i];

            if (pL2Chunk->hMemObj != NIL_RTR0MEMOBJ)
            {
                Assert(pL2Chunk->hMapObj != NIL_RTR0MEMOBJ);

                pL2Chunk->paBpL2TblBaseSharedR0 = NULL;

                hMemObj = pL2Chunk->hMapObj;
                pL2Chunk->hMapObj = NIL_RTR0MEMOBJ;
                RTR0MemObjFree(hMemObj, true);

                hMemObj = pL2Chunk->hMemObj;
                pL2Chunk->hMemObj = NIL_RTR0MEMOBJ;
                RTR0MemObjFree(hMemObj, true);
            }
        }

        pGVM->dbgfr0.s.fInit = false;
    }
#ifdef RT_STRICT
    else
    {
        Assert(pGVM->dbgfr0.s.hMemObjBpLocL1 == NIL_RTR0MEMOBJ);
        Assert(!pGVM->dbgfr0.s.paBpLocL1R0);

        Assert(pGVM->dbgfr0.s.hMemObjBpLocPortIo == NIL_RTR0MEMOBJ);
        Assert(!pGVM->dbgfr0.s.paBpLocPortIoR0);

        for (uint32_t i = 0; i < RT_ELEMENTS(pGVM->dbgfr0.s.aBpChunks); i++)
        {
            PDBGFBPCHUNKR0 pBpChunk = &pGVM->dbgfr0.s.aBpChunks[i];

            Assert(pBpChunk->hMemObj == NIL_RTR0MEMOBJ);
            Assert(pBpChunk->hMapObj == NIL_RTR0MEMOBJ);
            Assert(!pBpChunk->paBpBaseSharedR0);
            Assert(!pBpChunk->paBpBaseR0Only);
        }

        for (uint32_t i = 0; i < RT_ELEMENTS(pGVM->dbgfr0.s.aBpL2TblChunks); i++)
        {
            PDBGFBPL2TBLCHUNKR0 pL2Chunk = &pGVM->dbgfr0.s.aBpL2TblChunks[i];

            Assert(pL2Chunk->hMemObj == NIL_RTR0MEMOBJ);
            Assert(pL2Chunk->hMapObj == NIL_RTR0MEMOBJ);
            Assert(!pL2Chunk->paBpL2TblBaseSharedR0);
        }
    }
#endif
}


/**
 * Worker for DBGFR0BpInitReqHandler() that does the actual initialization.
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   ppaBpLocL1R3    Where to return the ring-3 L1 lookup table address on success.
 * @thread  EMT(0)
 */
static int dbgfR0BpInitWorker(PGVM pGVM, R3PTRTYPE(volatile uint32_t *) *ppaBpLocL1R3)
{
    /*
     * Figure out how much memory we need for the L1 lookup table and allocate it.
     */
    uint32_t const cbL1Loc = RT_ALIGN_32(UINT16_MAX * sizeof(uint32_t), HOST_PAGE_SIZE);

    RTR0MEMOBJ hMemObj;
    int rc = RTR0MemObjAllocPage(&hMemObj, cbL1Loc, false /*fExecutable*/);
    if (RT_FAILURE(rc))
        return rc;
    RT_BZERO(RTR0MemObjAddress(hMemObj), cbL1Loc);

    /* Map it. */
    RTR0MEMOBJ hMapObj;
    rc = RTR0MemObjMapUserEx(&hMapObj, hMemObj, (RTR3PTR)-1, 0, RTMEM_PROT_READ | RTMEM_PROT_WRITE, RTR0ProcHandleSelf(),
                             0 /*offSub*/, cbL1Loc);
    if (RT_SUCCESS(rc))
    {
        pGVM->dbgfr0.s.hMemObjBpLocL1 = hMemObj;
        pGVM->dbgfr0.s.hMapObjBpLocL1 = hMapObj;
        pGVM->dbgfr0.s.paBpLocL1R0    = (volatile uint32_t *)RTR0MemObjAddress(hMemObj);

        /*
         * We're done.
         */
        *ppaBpLocL1R3 = RTR0MemObjAddressR3(hMapObj);
        pGVM->dbgfr0.s.fInit = true;
        return rc;
    }

    RTR0MemObjFree(hMemObj, true);
    return rc;
}


/**
 * Worker for DBGFR0BpPortIoInitReqHandler() that does the actual initialization.
 *
 * @returns VBox status code.
 * @param   pGVM                The global (ring-0) VM structure.
 * @param   ppaBpLocPortIoR3    Where to return the ring-3 L1 lookup table address on success.
 * @thread  EMT(0)
 */
static int dbgfR0BpPortIoInitWorker(PGVM pGVM, R3PTRTYPE(volatile uint32_t *) *ppaBpLocPortIoR3)
{
    /*
     * Figure out how much memory we need for the I/O port breakpoint lookup table and allocate it.
     */
    uint32_t const cbPortIoLoc = RT_ALIGN_32(UINT16_MAX * sizeof(uint32_t), HOST_PAGE_SIZE);

    RTR0MEMOBJ hMemObj;
    int rc = RTR0MemObjAllocPage(&hMemObj, cbPortIoLoc, false /*fExecutable*/);
    if (RT_FAILURE(rc))
        return rc;
    RT_BZERO(RTR0MemObjAddress(hMemObj), cbPortIoLoc);

    /* Map it. */
    RTR0MEMOBJ hMapObj;
    rc = RTR0MemObjMapUserEx(&hMapObj, hMemObj, (RTR3PTR)-1, 0, RTMEM_PROT_READ | RTMEM_PROT_WRITE, RTR0ProcHandleSelf(),
                             0 /*offSub*/, cbPortIoLoc);
    if (RT_SUCCESS(rc))
    {
        pGVM->dbgfr0.s.hMemObjBpLocPortIo = hMemObj;
        pGVM->dbgfr0.s.hMapObjBpLocPortIo = hMapObj;
        pGVM->dbgfr0.s.paBpLocPortIoR0    = (volatile uint32_t *)RTR0MemObjAddress(hMemObj);

        /*
         * We're done.
         */
        *ppaBpLocPortIoR3 = RTR0MemObjAddressR3(hMapObj);
        return rc;
    }

    RTR0MemObjFree(hMemObj, true);
    return rc;
}


/**
 * Worker for DBGFR0BpOwnerInitReqHandler() that does the actual initialization.
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   ppaBpOwnerR3    Where to return the ring-3 breakpoint owner table base address on success.
 * @thread  EMT(0)
 */
//初始化 断点所有者表（Breakpoint Owner Table）
//设计目标：为断点所有者分配 双重内存区域（内核私有 + 用户态共享），并建立安全映射。
//调用场景：在调试子系统初始化时（如 DBGFR0Init）被调用。
static int dbgfR0BpOwnerInitWorker(PGVM pGVM, R3PTRTYPE(void *) *ppaBpOwnerR3)
{
    /*
     * Figure out how much memory we need for the owner tables and allocate it.
     */
#if 0
    +-----------------------------------+
    | 内核私有区域 (DBGFBPOWNERINTR0)   | ← paBpOwnersR0
    | 每个所有者的内核私有数据          |
    | 大小: cbBpOwnerR0                 |
    +-----------------------------------+
    | 共享区域 (DBGFBPOWNERINT)         | ← *ppaBpOwnerR3（用户态可见）
    | 每个所有者的共享数据              |
    | 大小: cbBpOwnerR3                 |
    +-----------------------------------+
#endif
    uint32_t const cbBpOwnerR0 = RT_ALIGN_32(DBGF_BP_OWNER_COUNT_MAX * sizeof(DBGFBPOWNERINTR0), HOST_PAGE_SIZE);
    uint32_t const cbBpOwnerR3 = RT_ALIGN_32(DBGF_BP_OWNER_COUNT_MAX * sizeof(DBGFBPOWNERINT), HOST_PAGE_SIZE);
    uint32_t const cbTotal     = RT_ALIGN_32(cbBpOwnerR0 + cbBpOwnerR3, HOST_PAGE_SIZE);

    RTR0MEMOBJ hMemObj;
    int rc = RTR0MemObjAllocPage(&hMemObj, cbTotal, false /*fExecutable*/);
    if (RT_FAILURE(rc))
        return rc;
    RT_BZERO(RTR0MemObjAddress(hMemObj), cbTotal);

    /* Map it. */
    RTR0MEMOBJ hMapObj;
    rc = RTR0MemObjMapUserEx(&hMapObj, hMemObj, (RTR3PTR)-1, 0, RTMEM_PROT_READ | RTMEM_PROT_WRITE, RTR0ProcHandleSelf(),
                             cbBpOwnerR0 /*offSub*/, cbBpOwnerR3);
    if (RT_SUCCESS(rc))
    {
        pGVM->dbgfr0.s.hMemObjBpOwners = hMemObj;
        pGVM->dbgfr0.s.hMapObjBpOwners = hMapObj;
        pGVM->dbgfr0.s.paBpOwnersR0    = (PDBGFBPOWNERINTR0)RTR0MemObjAddress(hMemObj);

        /*
         * We're done.
         */
        *ppaBpOwnerR3 = RTR0MemObjAddressR3(hMapObj);
        return rc;
    }

    RTR0MemObjFree(hMemObj, true);
    return rc;
}


/**
 * Worker for DBGFR0BpChunkAllocReqHandler() that does the actual chunk allocation.
 *
 * Allocates a memory object and divides it up as follows:
 * @verbatim
   --------------------------------------
   ring-0 chunk data
   --------------------------------------
   page alignment padding
   --------------------------------------
   shared chunk data
   --------------------------------------
   @endverbatim
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idChunk         The chunk ID to allocate.
 * @param   ppBpChunkBaseR3 Where to return the ring-3 chunk base address on success.
 * @thread  EMT(0)
 */
//断点内存块（Breakpoint Chunk）分配与映射
/*
  核心任务：
    计算所需内存大小（内核专用 + 共享区域）。
    分配物理内存并初始化清零。
    映射到用户态（R3）供调试器使用。
    更新 GVM 中的断点管理结构。
*/
static int dbgfR0BpChunkAllocWorker(PGVM pGVM, uint32_t idChunk, R3PTRTYPE(void *) *ppBpChunkBaseR3)
{
    /*
     * Figure out how much memory we need for the chunk and allocate it.
     */

#if 0
    +-----------------------------------+
    | 内核专用区域 (DBGFBPINTR0)        | ← paBpBaseR0Only
    | 每个断点的私有数据                |
    | 大小: cbRing0                     |
    +-----------------------------------+
    | 共享区域 (DBGFBPINT)              | ← paBpBaseSharedR0 (用户态可见)
    | 每个断点的共享数据                |
    | 大小: cbShared                    |
    +-----------------------------------+
#endif
    //每个断点的内核态私有数据
    uint32_t const cbRing0  = RT_ALIGN_32(DBGF_BP_COUNT_PER_CHUNK * sizeof(DBGFBPINTR0), HOST_PAGE_SIZE);
    //用户态和内核态共享的断点数据
    uint32_t const cbShared = RT_ALIGN_32(DBGF_BP_COUNT_PER_CHUNK * sizeof(DBGFBPINT), HOST_PAGE_SIZE);
    uint32_t const cbTotal  = cbRing0 + cbShared;

    RTR0MEMOBJ hMemObj;
    //分配物理连续的不可执行内存
    int rc = RTR0MemObjAllocPage(&hMemObj, cbTotal, false /*fExecutable*/);
    if (RT_FAILURE(rc))
        return rc;
    //初始化清零，避免残留数据干扰断点逻辑
    RT_BZERO(RTR0MemObjAddress(hMemObj), cbTotal);

    /* Map it. */
    RTR0MEMOBJ hMapObj;
    //用户态可读可写（不可执行）
    rc = RTR0MemObjMapUserEx(&hMapObj, hMemObj, (RTR3PTR)-1, 0, RTMEM_PROT_READ | RTMEM_PROT_WRITE, RTR0ProcHandleSelf(),
                             cbRing0 /*offSub*/, cbTotal - cbRing0);
    if (RT_SUCCESS(rc))
    {
        PDBGFBPCHUNKR0 pBpChunkR0 = &pGVM->dbgfr0.s.aBpChunks[idChunk];

        pBpChunkR0->hMemObj          = hMemObj;
        pBpChunkR0->hMapObj          = hMapObj;
        pBpChunkR0->paBpBaseR0Only   = (PDBGFBPINTR0)RTR0MemObjAddress(hMemObj);
        pBpChunkR0->paBpBaseSharedR0 = (PDBGFBPINT)&pBpChunkR0->paBpBaseR0Only[DBGF_BP_COUNT_PER_CHUNK];

        /*
         * We're done.
         */
        *ppBpChunkBaseR3 = RTR0MemObjAddressR3(hMapObj);
        return rc;
    }

    RTR0MemObjFree(hMemObj, true);
    return rc;
}


/**
 * Worker for DBGFR0BpL2TblChunkAllocReqHandler() that does the actual chunk allocation.
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idChunk         The chunk ID to allocate.
 * @param   ppL2ChunkBaseR3 Where to return the ring-3 chunk base address on success.
 * @thread  EMT(0)
 */
/*
    用户态（R3）调用 DBGFR3BpL2TblChunkAlloc 发起请求。
    内核态（R0）通过 VMMR0_DO_DBGF_BP_L2_TBL_CHUNK_ALLOC 调用 DBGFR0BpL2TblChunkAllocReqHandler。
    dbgfR0BpL2TblChunkAllocWorker完成实际分配和映射。
    返回 用户态可访问的 L2 Chunk 基地址，供调试器使用。
*/
/*
  分配物理内存（RTR0MemObjAllocPage），用于存储 L2 断点表的 Chunk。
  映射到用户态（R3）（RTR0MemObjMapUserEx），使调试器可以访问该内存。
  更新 GVM 状态，记录分配的内存对象和地址。
*/
static int dbgfR0BpL2TblChunkAllocWorker(PGVM pGVM, uint32_t idChunk, R3PTRTYPE(void *) *ppL2ChunkBaseR3)
{
    /*
     * Figure out how much memory we need for the chunk and allocate it.
     */
    //DBGF_BP_L2_TBL_ENTRIES_PER_CHUNK 每个 Chunk 的 L2 表条目数。
    //HOST_PAGE_SIZE：对齐到宿主机的页大小（如 4KB）。
    //RT_ALIGN_32：确保分配的内存是页大小的整数倍。
	/*
	  typedef struct {
          uint64_t uPackedData;  // 可能包含断点类型、地址、状态等
      } DBGFBPL2ENTRY;*/
    uint32_t const cbTotal = RT_ALIGN_32(DBGF_BP_L2_TBL_ENTRIES_PER_CHUNK * sizeof(DBGFBPL2ENTRY), HOST_PAGE_SIZE);

    RTR0MEMOBJ hMemObj;
    //RTR0MemObjAllocPage：分配不可执行的物理内存（fExecutable = false）。
    int rc = RTR0MemObjAllocPage(&hMemObj, cbTotal, false /*fExecutable*/);
    if (RT_FAILURE(rc))
        return rc;
    //RT_BZERO：清零内存，防止残留数据干扰调试逻辑。
    RT_BZERO(RTR0MemObjAddress(hMemObj), cbTotal);

    /* Map it. */
    RTR0MEMOBJ hMapObj;
    /*
      RTR0MemObjMapUserEx：将物理内存映射到 当前进程的用户态地址空间。
          (RTR3PTR)-1：让系统自动选择映射地址。
          RTMEM_PROT_READ | RTMEM_PROT_WRITE：映射为可读可写（不可执行）。
          RTR0ProcHandleSelf()：映射到当前进程。
	*/
    rc = RTR0MemObjMapUserEx(&hMapObj, hMemObj, (RTR3PTR)-1, 0, RTMEM_PROT_READ | RTMEM_PROT_WRITE, RTR0ProcHandleSelf(),
                             0 /*offSub*/, cbTotal);
    if (RT_SUCCESS(rc))
    {
        PDBGFBPL2TBLCHUNKR0 pL2ChunkR0 = &pGVM->dbgfr0.s.aBpL2TblChunks[idChunk];

        //hMemObj：物理内存对象句柄（用于后续释放）。
        pL2ChunkR0->hMemObj               = hMemObj;
        //hMapObj：用户态映射对象句柄（用于取消映射）。
        pL2ChunkR0->hMapObj               = hMapObj;
        //paBpL2TblBaseSharedR0：内核态（R0）可访问的 L2 表基地址。
        pL2ChunkR0->paBpL2TblBaseSharedR0 = (PDBGFBPL2ENTRY)RTR0MemObjAddress(hMemObj);

        /*
         * We're done.
         */
		//返回用户态（R3）可访问的虚拟地址。
        *ppL2ChunkBaseR3 = RTR0MemObjAddressR3(hMapObj);
        return rc;
    }

    RTR0MemObjFree(hMemObj, true);
    return rc;
}


/**
 * Used by ring-3 DBGF to fully initialize the breakpoint manager for operation.
 *
 * @returns VBox status code.
 * @param   pGVM    The global (ring-0) VM structure.
 * @param   pReq    Pointer to the request buffer.
 * @thread  EMT(0)
 */
VMMR0_INT_DECL(int) DBGFR0BpInitReqHandler(PGVM pGVM, PDBGFBPINITREQ pReq)
{
    LogFlow(("DBGFR0BpInitReqHandler:\n"));

    /*
     * Validate the request.
     */
    AssertReturn(pReq->Hdr.cbReq == sizeof(*pReq), VERR_INVALID_PARAMETER);

    int rc = GVMMR0ValidateGVMandEMT(pGVM, 0);
    AssertRCReturn(rc, rc);

    AssertReturn(!pGVM->dbgfr0.s.fInit, VERR_WRONG_ORDER);

    return dbgfR0BpInitWorker(pGVM, &pReq->paBpLocL1R3);
}


/**
 * Used by ring-3 DBGF to initialize the breakpoint manager for port I/O breakpoint operation.
 *
 * @returns VBox status code.
 * @param   pGVM    The global (ring-0) VM structure.
 * @param   pReq    Pointer to the request buffer.
 * @thread  EMT(0)
 */
VMMR0_INT_DECL(int) DBGFR0BpPortIoInitReqHandler(PGVM pGVM, PDBGFBPINITREQ pReq)
{
    LogFlow(("DBGFR0BpPortIoInitReqHandler:\n"));

    /*
     * Validate the request.
     */
    AssertReturn(pReq->Hdr.cbReq == sizeof(*pReq), VERR_INVALID_PARAMETER);

    int rc = GVMMR0ValidateGVMandEMT(pGVM, 0);
    AssertRCReturn(rc, rc);

    AssertReturn(pGVM->dbgfr0.s.fInit, VERR_WRONG_ORDER);
    AssertReturn(!pGVM->dbgfr0.s.paBpLocPortIoR0, VERR_WRONG_ORDER);

    return dbgfR0BpPortIoInitWorker(pGVM, &pReq->paBpLocL1R3);
}


/**
 * Used by ring-3 DBGF to initialize the breakpoint owner table for operation.
 *
 * @returns VBox status code.
 * @param   pGVM    The global (ring-0) VM structure.
 * @param   pReq    Pointer to the request buffer.
 * @thread  EMT(0)
 */
VMMR0_INT_DECL(int) DBGFR0BpOwnerInitReqHandler(PGVM pGVM, PDBGFBPOWNERINITREQ pReq)
{
    LogFlow(("DBGFR0BpOwnerInitReqHandler:\n"));

    /*
     * Validate the request.
     */
    AssertReturn(pReq->Hdr.cbReq == sizeof(*pReq), VERR_INVALID_PARAMETER);

    int rc = GVMMR0ValidateGVMandEMT(pGVM, 0);
    AssertRCReturn(rc, rc);

    AssertReturn(!pGVM->dbgfr0.s.paBpOwnersR0, VERR_WRONG_ORDER);

    return dbgfR0BpOwnerInitWorker(pGVM, &pReq->paBpOwnerR3);
}


/**
 * Used by ring-3 DBGF to allocate a given chunk in the global breakpoint table.
 *
 * @returns VBox status code.
 * @param   pGVM    The global (ring-0) VM structure.
 * @param   pReq    Pointer to the request buffer.
 * @thread  EMT(0)
 */
/*
   处理 断点内存块（Breakpoint Chunk）分配请求的核心函数。
   调用场景：当用户态（R3）调试器需要分配新的 L2 断点表内存块时触发。
   核心任务：验证请求合法性，并调用底层 dbgfR0BpChunkAllocWorker 完成实际内存分配。
*/
VMMR0_INT_DECL(int) DBGFR0BpChunkAllocReqHandler(PGVM pGVM, PDBGFBPCHUNKALLOCREQ pReq)
{
    LogFlow(("DBGFR0BpChunkAllocReqHandler:\n"));

    /*
     * Validate the request.
     */
    //检查请求大小：确保用户态传入的请求结构体大小与内核期望一致，防止内存越界。
    AssertReturn(pReq->Hdr.cbReq == sizeof(*pReq), VERR_INVALID_PARAMETER);

    uint32_t const idChunk = pReq->idChunk;
    AssertReturn(idChunk < DBGF_BP_CHUNK_COUNT, VERR_INVALID_PARAMETER);

    int rc = GVMMR0ValidateGVMandEMT(pGVM, 0);
    AssertRCReturn(rc, rc);

    AssertReturn(pGVM->dbgfr0.s.fInit, VERR_WRONG_ORDER);
    AssertReturn(pGVM->dbgfr0.s.aBpChunks[idChunk].hMemObj == NIL_RTR0MEMOBJ, VERR_INVALID_PARAMETER);

    return dbgfR0BpChunkAllocWorker(pGVM, idChunk, &pReq->pChunkBaseR3);
}


/**
 * Used by ring-3 DBGF to allocate a given chunk in the global L2 lookup table.
 *
 * @returns VBox status code.
 * @param   pGVM    The global (ring-0) VM structure.
 * @param   pReq    Pointer to the request buffer.
 * @thread  EMT(0)
 */
/*
  负责在 Ring-0（内核态）分配一块内存，用于存储 L2 断点表的某个 Chunk（内存块）
  L2 断点表：VirtualBox 调试子系统用于管理断点的二级表结构（类似页表的层级设计）。
  Chunk：L2 表的固定大小内存块（通常为 4KB 或类似分页大小）。
*/
/*
  pGVM	PGVM	                    指向虚拟机监控器（GVM）的指针，包含全局状态。
  pReq	PDBGFBPL2TBLCHUNKALLOCREQ	分配请求结构体，包含目标 Chunk ID 和返回的 R3 指针。
*/
/*
  typedef struct DBGFBPL2TBLCHUNKALLOCREQ {
      DBGFREQHDR    Hdr;           // 请求头（包含大小、类型等）
      uint32_t      idChunk;       // 目标 Chunk ID
      PRTR3PTR      pChunkBaseR3;  // 返回的 R3 基地址指针
  } DBGFBPL2TBLCHUNKALLOCREQ;
*/
VMMR0_INT_DECL(int) DBGFR0BpL2TblChunkAllocReqHandler(PGVM pGVM, PDBGFBPL2TBLCHUNKALLOCREQ pReq)
{
    LogFlow(("DBGFR0BpL2TblChunkAllocReqHandler:\n"));

    /*
     * Validate the request.
     */
    AssertReturn(pReq->Hdr.cbReq == sizeof(*pReq), VERR_INVALID_PARAMETER);

    uint32_t const idChunk = pReq->idChunk;
    AssertReturn(idChunk < DBGF_BP_L2_TBL_CHUNK_COUNT, VERR_INVALID_PARAMETER);

    //调用 GVMMR0ValidateGVMandEMT 验证 pGVM 的合法性，并确认当前线程是 EMT（Emulation Thread）。
    int rc = GVMMR0ValidateGVMandEMT(pGVM, 0);
    AssertRCReturn(rc, rc);

    //确保调试子系统已初始化（fInit 标志为真）。
    AssertReturn(pGVM->dbgfr0.s.fInit, VERR_WRONG_ORDER);
    //检查目标 Chunk 是否未被分配过（hMemObj == NIL_RTR0MEMOBJ）。
    AssertReturn(pGVM->dbgfr0.s.aBpL2TblChunks[idChunk].hMemObj == NIL_RTR0MEMOBJ, VERR_INVALID_PARAMETER);

    //调用 dbgfR0BpL2TblChunkAllocWorker 完成实际内存分配，
	//并将 R3（用户态）可访问的基地址写入 pReq->pChunkBaseR3。
    return dbgfR0BpL2TblChunkAllocWorker(pGVM, idChunk, &pReq->pChunkBaseR3);
}

