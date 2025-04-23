/* $Id$ */
/** @file
 * GMM - Global Memory Manager.
 */

/*
 * Copyright (C) 2007-2024 Oracle and/or its affiliates.
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


/** @page pg_gmm    GMM - The Global Memory Manager
 *
 * As the name indicates, this component is responsible for global memory
 * management. Currently only guest RAM is allocated from the GMM, but this
 * may change to include shadow page tables and other bits later.
 *
 * Guest RAM is managed as individual pages, but allocated from the host OS
 * in chunks for reasons of portability / efficiency. To minimize the memory
 * footprint all tracking structure must be as small as possible without
 * unnecessary performance penalties.
 *
 * The allocation chunks has fixed sized, the size defined at compile time
 * by the #GMM_CHUNK_SIZE \#define.
 *
 * Each chunk is given an unique ID. Each page also has a unique ID. The
 * relationship between the two IDs is:
 * @code
 *  GMM_CHUNK_SHIFT = log2(GMM_CHUNK_SIZE / GUEST_PAGE_SIZE);
 *  idPage = (idChunk << GMM_CHUNK_SHIFT) | iPage;
 * @endcode
 * Where iPage is the index of the page within the chunk. This ID scheme
 * permits for efficient chunk and page lookup, but it relies on the chunk size
 * to be set at compile time. The chunks are organized in an AVL tree with their
 * IDs being the keys.
 *
 * @todo Scope the chunk+page IDs based on config setting: per VM,
 *       per user (default), or global.  This will prevent ring-3 code screwing
 *       around with random page IDs from accessing someone else's data in the
 *       default config.  This would let us move HCPhys out of PGMPAGE when
 *       restricting it to ring-0 only, w/o requiring any additional ring-0 per
 *       page data (prereq mmio2 must go via GMM).  See @bugref{10696} for more.
 *
 * The physical address of each page in an allocation chunk is maintained by
 * the #RTR0MEMOBJ and obtained using #RTR0MemObjGetPagePhysAddr. There is no
 * need to duplicate this information (it'll cost 8-bytes per page if we did).
 *
 * So what do we need to track per page? Most importantly we need to know
 * which state the page is in:
 *   - Private - Allocated for (eventually) backing one particular VM page.
 *   - Shared  - Readonly page that is used by one or more VMs and treated
 *               as COW by PGM.
 *   - Free    - Not used by anyone.
 *
 * For the page replacement operations (sharing, defragmenting and freeing)
 * to be somewhat efficient, private pages needs to be associated with a
 * particular page in a particular VM.
 *
 * Tracking the usage of shared pages is impractical and expensive, so we'll
 * settle for a reference counting system instead.
 *
 * Free pages will be chained on LIFOs
 *
 * On 64-bit systems we will use a 64-bit bitfield per page, while on 32-bit
 * systems a 32-bit bitfield will have to suffice because of address space
 * limitations. The #GMMPAGE structure shows the details.
 *
 *
 * @section sec_gmm_alloc_strat Page Allocation Strategy
 *
 * The strategy for allocating pages has to take fragmentation and shared
 * pages into account, or we may end up with with 2000 chunks with only
 * a few pages in each. Shared pages cannot easily be reallocated because
 * of the inaccurate usage accounting (see above). Private pages can be
 * reallocated by a defragmentation thread in the same manner that sharing
 * is done.
 *
 * The first approach is to manage the free pages in two sets depending on
 * whether they are mainly for the allocation of shared or private pages.
 * In the initial implementation there will be almost no possibility for
 * mixing shared and private pages in the same chunk (only if we're really
 * stressed on memory), but when we implement forking of VMs and have to
 * deal with lots of COW pages it'll start getting kind of interesting.
 *
 * The sets are lists of chunks with approximately the same number of
 * free pages. Say the chunk size is 1MB, meaning 256 pages, and a set
 * consists of 16 lists. So, the first list will contain the chunks with
 * 1-7 free pages, the second covers 8-15, and so on. The chunks will be
 * moved between the lists as pages are freed up or allocated.
 *
 *
 * @section sec_gmm_costs       Costs
 *
 * The per page cost in kernel space is 32-bit plus whatever RTR0MEMOBJ
 * entails. In addition there is the chunk cost of approximately
 * (sizeof(RT0MEMOBJ) + sizeof(CHUNK)) / 2^CHUNK_SHIFT bytes per page.
 *
 * On Windows the per page #RTR0MEMOBJ cost is 32-bit on 32-bit windows
 * and 64-bit on 64-bit windows (a PFN_NUMBER in the MDL). So, 64-bit per page.
 * The cost on Linux is identical, but here it's because of sizeof(struct page *).
 *
 *
 * @section sec_gmm_legacy      Legacy Mode for Non-Tier-1 Platforms
 *
 * In legacy mode the page source is locked user pages and not
 * #RTR0MemObjAllocPhysNC, this means that a page can only be allocated
 * by the VM that locked it. We will make no attempt at implementing
 * page sharing on these systems, just do enough to make it all work.
 *
 * @note With 6.1 really dropping 32-bit support, the legacy mode is obsoleted
 *       under the assumption that there is sufficient kernel virtual address
 *       space to map all of the guest memory allocations.  So, we'll be using
 *       #RTR0MemObjAllocPage on some platforms as an alternative to
 *       #RTR0MemObjAllocPhysNC.
 *
 *
 * @subsection sub_gmm_locking  Serializing
 *
 * One simple fast mutex will be employed in the initial implementation, not
 * two as mentioned in @ref sec_pgmPhys_Serializing.
 *
 * @see @ref sec_pgmPhys_Serializing
 *
 *
 * @section sec_gmm_overcommit  Memory Over-Commitment Management
 *
 * The GVM will have to do the system wide memory over-commitment
 * management. My current ideas are:
 *      - Per VM oc policy that indicates how much to initially commit
 *        to it and what to do in a out-of-memory situation.
 *      - Prevent overtaxing the host.
 *
 * There are some challenges here, the main ones are configurability and
 * security. Should we for instance permit anyone to request 100% memory
 * commitment? Who should be allowed to do runtime adjustments of the
 * config. And how to prevent these settings from being lost when the last
 * VM process exits? The solution is probably to have an optional root
 * daemon the will keep VMMR0.r0 in memory and enable the security measures.
 *
 *
 *
 * @section sec_gmm_numa        NUMA
 *
 * NUMA considerations will be designed and implemented a bit later.
 *
 * The preliminary guesses is that we will have to try allocate memory as
 * close as possible to the CPUs the VM is executed on (EMT and additional CPU
 * threads). Which means it's mostly about allocation and sharing policies.
 * Both the scheduler and allocator interface will to supply some NUMA info
 * and we'll need to have a way to calc access costs.
 *
 */


/*********************************************************************************************************************************
*   Header Files                                                                                                                 *
*********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_GMM
#include <VBox/rawpci.h>
#include <VBox/vmm/gmm.h>
#include "GMMR0Internal.h"
#include <VBox/vmm/vmcc.h>
#include <VBox/vmm/pgm.h>
#include <VBox/log.h>
#include <VBox/param.h>
#include <VBox/err.h>
#include <VBox/VMMDev.h>
#include <iprt/asm.h>
#include <iprt/avl.h>
#ifdef VBOX_STRICT
# include <iprt/crc.h>
#endif
#include <iprt/critsect.h>
#include <iprt/list.h>
#include <iprt/mem.h>
#include <iprt/memobj.h>
#include <iprt/mp.h>
#include <iprt/semaphore.h>
#include <iprt/spinlock.h>
#include <iprt/string.h>
#include <iprt/time.h>

/* This is 64-bit only code now. */
#if HC_ARCH_BITS != 64 || ARCH_BITS != 64
# error "This is 64-bit only code"
#endif


/*********************************************************************************************************************************
*   Defined Constants And Macros                                                                                                 *
*********************************************************************************************************************************/
/** @def VBOX_USE_CRIT_SECT_FOR_GIANT
 * Use a critical section instead of a fast mutex for the giant GMM lock.
 *
 * @remarks This is primarily a way of avoiding the deadlock checks in the
 *          windows driver verifier. */
#if defined(RT_OS_WINDOWS) || defined(RT_OS_DARWIN) || defined(DOXYGEN_RUNNING)
# define VBOX_USE_CRIT_SECT_FOR_GIANT
#endif


/*********************************************************************************************************************************
*   Structures and Typedefs                                                                                                      *
*********************************************************************************************************************************/
/** Pointer to set of free chunks.  */
typedef struct GMMCHUNKFREESET *PGMMCHUNKFREESET;

/**
 * The per-page tracking structure employed by the GMM.
 *
 * Because of the different layout on 32-bit and 64-bit hosts in earlier
 * versions of the code, macros are used to get and set some of the data.
 */
typedef union GMMPAGE
{
    /** Unsigned integer view. */
    uint64_t u;

    /** The common view. */
    struct GMMPAGECOMMON
    {
        uint32_t    uStuff1 : 32;
        uint32_t    uStuff2 : 30;
        /** The page state. */
        uint32_t    u2State : 2;
    } Common;

    /** The view of a private page. */
    struct GMMPAGEPRIVATE
    {
        /** The guest page frame number. (Max addressable: 2 ^ 44 - 16) */
        uint32_t    pfn;
        /** The GVM handle. (64K VMs) */
        uint32_t    hGVM : 16;
        /** Reserved. */
        uint32_t    u16Reserved : 14;
        /** The page state. */
        uint32_t    u2State : 2;
    } Private;

    /** The view of a shared page. */
    struct GMMPAGESHARED
    {
        /** The host page frame number. (Max addressable: 2 ^ 44 - 16) */
        uint32_t    pfn;
        /** The reference count (64K VMs). */
        uint32_t    cRefs : 16;
        /** Used for debug checksumming. */
        uint32_t    u14Checksum : 14;
        /** The page state. */
        uint32_t    u2State : 2;
    } Shared;

    /** The view of a free page. */
    struct GMMPAGEFREE
    {
        /** The index of the next page in the free list. UINT16_MAX is NIL. */
        uint16_t    iNext;
        /** Reserved. Checksum or something? */
        uint16_t    u16Reserved0;
        /** Reserved. Checksum or something? */
        uint32_t    u30Reserved1 : 29;
        /** Set if the page was zeroed. */
        uint32_t    fZeroed : 1;
        /** The page state. */
        uint32_t    u2State : 2;
    } Free;
} GMMPAGE;
AssertCompileSize(GMMPAGE, sizeof(RTHCUINTPTR));
/** Pointer to a GMMPAGE. */
typedef GMMPAGE *PGMMPAGE;


/** @name The Page States.
 * @{ */
/** A private page. */
#define GMM_PAGE_STATE_PRIVATE          0
/** A shared page. */
#define GMM_PAGE_STATE_SHARED           2
/** A free page. */
#define GMM_PAGE_STATE_FREE             3
/** @} */


/** @def GMM_PAGE_IS_PRIVATE
 *
 * @returns true if private, false if not.
 * @param   pPage       The GMM page.
 */
#define GMM_PAGE_IS_PRIVATE(pPage)  ( (pPage)->Common.u2State == GMM_PAGE_STATE_PRIVATE )

/** @def GMM_PAGE_IS_SHARED
 *
 * @returns true if shared, false if not.
 * @param   pPage       The GMM page.
 */
#define GMM_PAGE_IS_SHARED(pPage)   ( (pPage)->Common.u2State == GMM_PAGE_STATE_SHARED )

/** @def GMM_PAGE_IS_FREE
 *
 * @returns true if free, false if not.
 * @param   pPage       The GMM page.
 */
#define GMM_PAGE_IS_FREE(pPage)     ( (pPage)->Common.u2State == GMM_PAGE_STATE_FREE )

/** @def GMM_PAGE_PFN_LAST
 * The last valid guest pfn range.
 * @remark Some of the values outside the range has special meaning,
 *         see GMM_PAGE_PFN_UNSHAREABLE.
 */
#define GMM_PAGE_PFN_LAST            UINT32_C(0xfffffff0)
AssertCompile(GMM_PAGE_PFN_LAST == (GMM_GCPHYS_LAST >> GUEST_PAGE_SHIFT));

/** @def GMM_PAGE_PFN_UNSHAREABLE
 * Indicates that this page isn't used for normal guest memory and thus isn't shareable.
 */
#define GMM_PAGE_PFN_UNSHAREABLE    UINT32_C(0xfffffff1)
AssertCompile(GMM_PAGE_PFN_UNSHAREABLE == (GMM_GCPHYS_UNSHAREABLE >> GUEST_PAGE_SHIFT));


/**
 * A GMM allocation chunk ring-3 mapping record.
 *
 * This should really be associated with a session and not a VM, but
 * it's simpler to associated with a VM and cleanup with the VM object
 * is destroyed.
 */
typedef struct GMMCHUNKMAP
{
    /** The mapping object. */
    RTR0MEMOBJ          hMapObj;
    /** The VM owning the mapping. */
    PGVM                pGVM;
} GMMCHUNKMAP;
/** Pointer to a GMM allocation chunk mapping. */
typedef struct GMMCHUNKMAP *PGMMCHUNKMAP;


/**
 * A GMM allocation chunk.
 */
typedef struct GMMCHUNK
{
    /** The AVL node core.
     * The Key is the chunk ID.  (Giant mtx.) */
    AVLU32NODECORE      Core;
    /** The memory object.
     * Either from RTR0MemObjAllocPhysNC or RTR0MemObjLockUser depending on
     * what the host can dish up with.  (Chunk mtx protects mapping accesses
     * and related frees.) */
    RTR0MEMOBJ          hMemObj;
#ifndef VBOX_WITH_LINEAR_HOST_PHYS_MEM
    /** Pointer to the kernel mapping. */
    uint8_t            *pbMapping;
#endif
    /** Pointer to the next chunk in the free list.  (Giant mtx.) */
    PGMMCHUNK           pFreeNext;
    /** Pointer to the previous chunk in the free list. (Giant mtx.) */
    PGMMCHUNK           pFreePrev;
    /** Pointer to the free set this chunk belongs to.  NULL for
     * chunks with no free pages. (Giant mtx.) */
    PGMMCHUNKFREESET    pSet;
    /** List node in the chunk list (GMM::ChunkList).  (Giant mtx.) */
    RTLISTNODE          ListNode;
    /** Pointer to an array of mappings.  (Chunk mtx.) */
    PGMMCHUNKMAP        paMappingsX;
    /** The number of mappings.  (Chunk mtx.) */
    uint16_t            cMappingsX;
    /** The mapping lock this chunk is using using.  UINT8_MAX if nobody is mapping
     * or freeing anything.  (Giant mtx.) */
    uint8_t volatile    iChunkMtx;
    /** GMM_CHUNK_FLAGS_XXX. (Giant mtx.) */
    uint8_t             fFlags;
    /** The head of the list of free pages. UINT16_MAX is the NIL value.
     *  (Giant mtx.) */
    uint16_t            iFreeHead;
    /** The number of free pages.  (Giant mtx.) */
    uint16_t            cFree;
    /** The GVM handle of the VM that first allocated pages from this chunk, this
     * is used as a preference when there are several chunks to choose from.
     * When in bound memory mode this isn't a preference any longer.  (Giant
     * mtx.) */
    uint16_t            hGVM;
    /** The ID of the NUMA node the memory mostly resides on.  (Reserved for
     *  future use.)  (Giant mtx.) */
    uint16_t            idNumaNode;
    /** The number of private pages.  (Giant mtx.) */
    uint16_t            cPrivate;
    /** The number of shared pages.  (Giant mtx.) */
    uint16_t            cShared;
    /** The UID this chunk is associated with. */
    RTUID               uidOwner;
    uint32_t            u32Padding;
    /** The pages.  (Giant mtx.) */
    GMMPAGE             aPages[GMM_CHUNK_NUM_PAGES];
} GMMCHUNK;

/** Indicates that the NUMA properies of the memory is unknown. */
#define GMM_CHUNK_NUMA_ID_UNKNOWN   UINT16_C(0xfffe)

/** @name GMM_CHUNK_FLAGS_XXX - chunk flags.
 * @{ */
/** Indicates that the chunk is a large page (2MB). */
#define GMM_CHUNK_FLAGS_LARGE_PAGE  UINT16_C(0x0001)
/** @}  */


/**
 * An allocation chunk TLB entry.
 */
typedef struct GMMCHUNKTLBE
{
    /** The chunk id. */
    uint32_t            idChunk;
    /** Pointer to the chunk. */
    PGMMCHUNK           pChunk;
} GMMCHUNKTLBE;
/** Pointer to an allocation chunk TLB entry. */
typedef GMMCHUNKTLBE *PGMMCHUNKTLBE;


/** The number of entries in the allocation chunk TLB. */
#define GMM_CHUNKTLB_ENTRIES        32
/** Gets the TLB entry index for the given Chunk ID. */
#define GMM_CHUNKTLB_IDX(idChunk)   ( (idChunk) & (GMM_CHUNKTLB_ENTRIES - 1) )

/**
 * An allocation chunk TLB.
 */
typedef struct GMMCHUNKTLB
{
    /** The TLB entries. */
    GMMCHUNKTLBE    aEntries[GMM_CHUNKTLB_ENTRIES];
} GMMCHUNKTLB;
/** Pointer to an allocation chunk TLB. */
typedef GMMCHUNKTLB *PGMMCHUNKTLB;


/**
 * The GMM instance data.
 */
typedef struct GMM
{
    /** Magic / eye catcher. GMM_MAGIC */
    uint32_t            u32Magic;
    /** The number of threads waiting on the mutex. */
    uint32_t            cMtxContenders;
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    /** The critical section protecting the GMM.
     * More fine grained locking can be implemented later if necessary. */
    RTCRITSECT          GiantCritSect;
#else
    /** The fast mutex protecting the GMM.
     * More fine grained locking can be implemented later if necessary. */
    RTSEMFASTMUTEX      hMtx;
#endif
#ifdef VBOX_STRICT
    /** The current mutex owner. */
    RTNATIVETHREAD      hMtxOwner;
#endif
    /** Spinlock protecting the AVL tree.
     * @todo Make this a read-write spinlock as we should allow concurrent
     *       lookups. */
    RTSPINLOCK          hSpinLockTree;
    /** The chunk tree.
     * Protected by hSpinLockTree. */
    PAVLU32NODECORE     pChunks;
    /** Chunk freeing generation - incremented whenever a chunk is freed.  Used
     * for validating the per-VM chunk TLB entries.  Valid range is 1 to 2^62
     * (exclusive), though higher numbers may temporarily occure while
     * invalidating the individual TLBs during wrap-around processing. */
    uint64_t volatile   idFreeGeneration;
    /** The chunk TLB.
     * Protected by hSpinLockTree. */
    GMMCHUNKTLB         ChunkTLB;
    /** The private free set. */
    GMMCHUNKFREESET     PrivateX;
    /** The shared free set. */
    GMMCHUNKFREESET     Shared;

    /** Shared module tree (global).
     * @todo separate trees for distinctly different guest OSes. */
    PAVLLU32NODECORE    pGlobalSharedModuleTree;
    /** Sharable modules (count of nodes in pGlobalSharedModuleTree). */
    uint32_t            cShareableModules;

    /** The chunk list.  For simplifying the cleanup process and avoid tree
     * traversal. */
    RTLISTANCHOR        ChunkList;

    /** The maximum number of pages we're allowed to allocate.
     * @gcfgm{GMM/MaxPages,64-bit, Direct.}
     * @gcfgm{GMM/PctPages,32-bit, Relative to the number of host pages.} */
    uint64_t            cMaxPages;
    /** The number of pages that has been reserved.
     * The deal is that cReservedPages - cOverCommittedPages <= cMaxPages. */
    uint64_t            cReservedPages;
    /** The number of pages that we have over-committed in reservations. */
    uint64_t            cOverCommittedPages;
    /** The number of actually allocated (committed if you like) pages. */
    uint64_t            cAllocatedPages;
    /** The number of pages that are shared. A subset of cAllocatedPages. */
    uint64_t            cSharedPages;
    /** The number of pages that are actually shared between VMs. */
    uint64_t            cDuplicatePages;
    /** The number of pages that are shared that has been left behind by
     * VMs not doing proper cleanups. */
    uint64_t            cLeftBehindSharedPages;
    /** The number of allocation chunks.
     * (The number of pages we've allocated from the host can be derived from this.) */
    uint32_t            cChunks;
    /** The number of current ballooned pages. */
    uint64_t            cBalloonedPages;

#ifdef VBOX_WITH_LINEAR_HOST_PHYS_MEM
    /** Whether #RTR0MemObjAllocPhysNC works.   */
    bool                fHasWorkingAllocPhysNC;
#else
    bool                fPadding;
#endif
    /** The bound memory mode indicator.
     * When set, the memory will be bound to a specific VM and never
     * shared. This is always set if fLegacyAllocationMode is set.
     * (Also determined at initialization time.) */
    bool                fBoundMemoryMode;
    /** The number of registered VMs. */
    uint16_t            cRegisteredVMs;

    /** The index of the next mutex to use. */
    uint32_t            iNextChunkMtx;
    /** Chunk locks for reducing lock contention without having to allocate
     * one lock per chunk. */
    struct
    {
        /** The mutex */
        RTSEMFASTMUTEX      hMtx;
        /** The number of threads currently using this mutex. */
        uint32_t volatile   cUsers;
    } aChunkMtx[64];

    /** The number of freed chunks ever.  This is used as list generation to
     * avoid restarting the cleanup scanning when the list wasn't modified. */
    uint32_t volatile   cFreedChunks;
    /** The previous allocated Chunk ID.
     * Used as a hint to avoid scanning the whole bitmap. */
    uint32_t            idChunkPrev;
    /** Spinlock protecting idChunkPrev & bmChunkId.  */
    RTSPINLOCK          hSpinLockChunkId;
    /** Chunk ID allocation bitmap.
     * Bits of allocated IDs are set, free ones are clear.
     * The NIL id (0) is marked allocated. */
    uint32_t            bmChunkId[(GMM_CHUNKID_LAST + 1 + 31) / 32];
} GMM;
/** Pointer to the GMM instance. */
typedef GMM *PGMM;

/** The value of GMM::u32Magic (Katsuhiro Otomo). */
#define GMM_MAGIC       UINT32_C(0x19540414)


/**
 * GMM chunk mutex state.
 *
 * This is returned by gmmR0ChunkMutexAcquire and is used by the other
 * gmmR0ChunkMutex* methods.
 */
typedef struct GMMR0CHUNKMTXSTATE
{
    PGMM                pGMM;
    /** The index of the chunk mutex. */
    uint8_t             iChunkMtx;
    /** The relevant flags (GMMR0CHUNK_MTX_XXX). */
    uint8_t             fFlags;
} GMMR0CHUNKMTXSTATE;
/** Pointer to a chunk mutex state. */
typedef GMMR0CHUNKMTXSTATE *PGMMR0CHUNKMTXSTATE;

/** @name GMMR0CHUNK_MTX_XXX
 * @{ */
#define GMMR0CHUNK_MTX_INVALID          UINT32_C(0)
#define GMMR0CHUNK_MTX_KEEP_GIANT       UINT32_C(1)
#define GMMR0CHUNK_MTX_RETAKE_GIANT     UINT32_C(2)
#define GMMR0CHUNK_MTX_DROP_GIANT       UINT32_C(3)
#define GMMR0CHUNK_MTX_END              UINT32_C(4)
/** @} */


/** The maximum number of shared modules per-vm. */
#define GMM_MAX_SHARED_PER_VM_MODULES   2048
/** The maximum number of shared modules GMM is allowed to track. */
#define GMM_MAX_SHARED_GLOBAL_MODULES   16834


/**
 * Argument packet for gmmR0SharedModuleCleanup.
 */
typedef struct GMMR0SHMODPERVMDTORARGS
{
    PGVM    pGVM;
    PGMM    pGMM;
} GMMR0SHMODPERVMDTORARGS;

/**
 * Argument packet for gmmR0CheckSharedModule.
 */
typedef struct GMMCHECKSHAREDMODULEINFO
{
    PGVM                    pGVM;
    VMCPUID                 idCpu;
} GMMCHECKSHAREDMODULEINFO;


/*********************************************************************************************************************************
*   Global Variables                                                                                                             *
*********************************************************************************************************************************/
/** Pointer to the GMM instance data. */
static PGMM g_pGMM = NULL;

/** Macro for obtaining and validating the g_pGMM pointer.
 *
 * On failure it will return from the invoking function with the specified
 * return value.
 *
 * @param   pGMM    The name of the pGMM variable.
 * @param   rc      The return value on failure. Use VERR_GMM_INSTANCE for VBox
 *                  status codes.
 */
#define GMM_GET_VALID_INSTANCE(pGMM, rc) \
    do { \
        (pGMM) = g_pGMM; \
        AssertPtrReturn((pGMM), (rc)); \
        AssertMsgReturn((pGMM)->u32Magic == GMM_MAGIC, ("%p - %#x\n", (pGMM), (pGMM)->u32Magic), (rc)); \
    } while (0)

/** Macro for obtaining and validating the g_pGMM pointer, void function
 * variant.
 *
 * On failure it will return from the invoking function.
 *
 * @param   pGMM    The name of the pGMM variable.
 */
#define GMM_GET_VALID_INSTANCE_VOID(pGMM) \
    do { \
        (pGMM) = g_pGMM; \
        AssertPtrReturnVoid((pGMM)); \
        AssertMsgReturnVoid((pGMM)->u32Magic == GMM_MAGIC, ("%p - %#x\n", (pGMM), (pGMM)->u32Magic)); \
    } while (0)


/** @def GMM_CHECK_SANITY_UPON_ENTERING
 * Checks the sanity of the GMM instance data before making changes.
 *
 * This is macro is a stub by default and must be enabled manually in the code.
 *
 * @returns true if sane, false if not.
 * @param   pGMM    The name of the pGMM variable.
 */
#if defined(VBOX_STRICT) && defined(GMMR0_WITH_SANITY_CHECK) && 0
# define GMM_CHECK_SANITY_UPON_ENTERING(pGMM)   (RT_LIKELY(gmmR0SanityCheck((pGMM), __PRETTY_FUNCTION__, __LINE__) == 0))
#else
# define GMM_CHECK_SANITY_UPON_ENTERING(pGMM)   (true)
#endif

/** @def GMM_CHECK_SANITY_UPON_LEAVING
 * Checks the sanity of the GMM instance data after making changes.
 *
 * This is macro is a stub by default and must be enabled manually in the code.
 *
 * @returns true if sane, false if not.
 * @param   pGMM    The name of the pGMM variable.
 */
#if defined(VBOX_STRICT) && defined(GMMR0_WITH_SANITY_CHECK) && 0
# define GMM_CHECK_SANITY_UPON_LEAVING(pGMM)    (gmmR0SanityCheck((pGMM), __PRETTY_FUNCTION__, __LINE__) == 0)
#else
# define GMM_CHECK_SANITY_UPON_LEAVING(pGMM)    (true)
#endif

/** @def GMM_CHECK_SANITY_IN_LOOPS
 * Checks the sanity of the GMM instance in the allocation loops.
 *
 * This is macro is a stub by default and must be enabled manually in the code.
 *
 * @returns true if sane, false if not.
 * @param   pGMM    The name of the pGMM variable.
 */
#if defined(VBOX_STRICT) && defined(GMMR0_WITH_SANITY_CHECK) && 0
# define GMM_CHECK_SANITY_IN_LOOPS(pGMM)        (gmmR0SanityCheck((pGMM), __PRETTY_FUNCTION__, __LINE__) == 0)
#else
# define GMM_CHECK_SANITY_IN_LOOPS(pGMM)        (true)
#endif


/*********************************************************************************************************************************
*   Internal Functions                                                                                                           *
*********************************************************************************************************************************/
static DECLCALLBACK(int)    gmmR0TermDestroyChunk(PAVLU32NODECORE pNode, void *pvGMM);
static bool                 gmmR0CleanupVMScanChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk);
DECLINLINE(void)            gmmR0UnlinkChunk(PGMMCHUNK pChunk);
DECLINLINE(void)            gmmR0LinkChunk(PGMMCHUNK pChunk, PGMMCHUNKFREESET pSet);
DECLINLINE(void)            gmmR0SelectSetAndLinkChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk);
#ifdef GMMR0_WITH_SANITY_CHECK
static uint32_t             gmmR0SanityCheck(PGMM pGMM, const char *pszFunction, unsigned uLineNo);
#endif
static bool                 gmmR0FreeChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, bool fRelaxedSem);
DECLINLINE(void)            gmmR0FreePrivatePage(PGMM pGMM, PGVM pGVM, uint32_t idPage, PGMMPAGE pPage);
DECLINLINE(void)            gmmR0FreeSharedPage(PGMM pGMM, PGVM pGVM, uint32_t idPage, PGMMPAGE pPage);
static int                  gmmR0UnmapChunkLocked(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk);
#ifdef VBOX_WITH_PAGE_SHARING
static void                 gmmR0SharedModuleCleanup(PGMM pGMM, PGVM pGVM);
# ifdef VBOX_STRICT
static uint32_t             gmmR0StrictPageChecksum(PGMM pGMM, PGVM pGVM, uint32_t idPage);
# endif
#endif



/**
 * Initializes the GMM component.
 *
 * This is called when the VMMR0.r0 module is loaded and protected by the
 * loader semaphore.
 *
 * @returns VBox status code.
 */
/*
分层锁设计
锁类型	            用途	        初始化方式
块级锁池	        保护独立内存块	RTSemFastMutexCreate循环创建
自旋锁（树）	    保护AVL树结构	RTSpinlockCreate中断安全模式
自旋锁（ChunkID）	保护ID分配位图	同上
 * */
GMMR0DECL(int) GMMR0Init(void)
{
    LogFlow(("GMMInit:\n"));

    /* Currently assuming same host and guest page size here.  Can change it to
       dish out guest pages with different size from the host page later if
       needed, though a restriction would be the host page size must be larger
       than the guest page size. */
    AssertCompile(GUEST_PAGE_SIZE == HOST_PAGE_SIZE); // 静态断言确保页大小一致
    AssertCompile(GUEST_PAGE_SIZE <= HOST_PAGE_SIZE);// 防御性二次验证

    /*
     * Allocate the instance data and the locks.
     */
    PGMM pGMM = (PGMM)RTMemAllocZ(sizeof(*pGMM));
    if (!pGMM)
        return VERR_NO_MEMORY;

    pGMM->u32Magic = GMM_MAGIC;
    //TLB预热：避免首次访问时的冷启动问题
    /*
       如果TLB未初始化，首次访问可能触发页表查询，导致不可预测的延迟。
       通过显式初始化，确保TLB的初始状态可控，减少运行时的不确定性。
    */
    for (unsigned i = 0; i < RT_ELEMENTS(pGMM->ChunkTLB.aEntries); i++)
        pGMM->ChunkTLB.aEntries[i].idChunk = NIL_GMM_CHUNKID; // TLB初始化
    RTListInit(&pGMM->ChunkList);
    ASMBitSet(&pGMM->bmChunkId[0], NIL_GMM_CHUNKID);// 位图标记保留ID

    //通过编译宏选择锁实现（关键区段适合复杂操作，互斥量侧重性能）
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    int rc = RTCritSectInit(&pGMM->GiantCritSect);
#else
    int rc = RTSemFastMutexCreate(&pGMM->hMtx);
#endif
    if (RT_SUCCESS(rc))
    {
        unsigned iMtx;
        for (iMtx = 0; iMtx < RT_ELEMENTS(pGMM->aChunkMtx); iMtx++)
        {
            rc = RTSemFastMutexCreate(&pGMM->aChunkMtx[iMtx].hMtx);
            if (RT_FAILURE(rc))
                break;
        }
        pGMM->hSpinLockTree = NIL_RTSPINLOCK;
        if (RT_SUCCESS(rc))
            //中断安全：自旋锁标记RTSPINLOCK_FLAGS_INTERRUPT_SAFE确保内核态可用
            rc = RTSpinlockCreate(&pGMM->hSpinLockTree, RTSPINLOCK_FLAGS_INTERRUPT_SAFE, "gmm-chunk-tree");
        pGMM->hSpinLockChunkId = NIL_RTSPINLOCK;
        if (RT_SUCCESS(rc))
            rc = RTSpinlockCreate(&pGMM->hSpinLockChunkId, RTSPINLOCK_FLAGS_INTERRUPT_SAFE, "gmm-chunk-id");
        if (RT_SUCCESS(rc))
        {
            /*
             * Figure out how we're going to allocate stuff (only applicable to
             * host with linear physical memory mappings).
             */
            pGMM->fBoundMemoryMode = false;
#ifdef VBOX_WITH_LINEAR_HOST_PHYS_MEM
            pGMM->fHasWorkingAllocPhysNC = false;

            RTR0MEMOBJ hMemObj;
            // 能力检测：运行时检查宿主是否支持非连续物理内存映射
            // 尝试非连续物理内存分配
            rc = RTR0MemObjAllocPhysNC(&hMemObj, GMM_CHUNK_SIZE, NIL_RTHCPHYS);
            if (RT_SUCCESS(rc))
            {
                rc = RTR0MemObjFree(hMemObj, true);
                AssertRC(rc);
                //成功时设置标志位，如果失败没有设置标志位,后续逻辑切换至保守分配策略
                pGMM->fHasWorkingAllocPhysNC = true;
            }
            else if (rc != VERR_NOT_SUPPORTED)
                SUPR0Printf("GMMR0Init: Warning! RTR0MemObjAllocPhysNC(, %u, NIL_RTHCPHYS) -> %d!\n", GMM_CHUNK_SIZE, rc);
# endif

            /*
             * Query system page count and guess a reasonable cMaxPages value.
             */
            pGMM->cMaxPages = UINT32_MAX; /** @todo IPRT function for query ram size and such. */

            /*
             * The idFreeGeneration value should be set so we actually trigger the
             * wrap-around invalidation handling during a typical test run.
             */
            pGMM->idFreeGeneration = UINT64_MAX / 4 - 128;

            g_pGMM = pGMM;
#ifdef VBOX_WITH_LINEAR_HOST_PHYS_MEM
            LogFlow(("GMMInit: pGMM=%p fBoundMemoryMode=%RTbool fHasWorkingAllocPhysNC=%RTbool\n", pGMM, pGMM->fBoundMemoryMode, pGMM->fHasWorkingAllocPhysNC));
#else
            LogFlow(("GMMInit: pGMM=%p fBoundMemoryMode=%RTbool\n", pGMM, pGMM->fBoundMemoryMode));
#endif
            return VINF_SUCCESS;
        }

        /*
         * Bail out.
         */
        RTSpinlockDestroy(pGMM->hSpinLockChunkId);
        RTSpinlockDestroy(pGMM->hSpinLockTree);
        while (iMtx-- > 0)
            RTSemFastMutexDestroy(pGMM->aChunkMtx[iMtx].hMtx);// 逆向销毁已创建的块级锁
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
        RTCritSectDelete(&pGMM->GiantCritSect);
#else
        RTSemFastMutexDestroy(pGMM->hMtx);
#endif
    }

    pGMM->u32Magic = 0;
    RTMemFree(pGMM);
    SUPR0Printf("GMMR0Init: failed! rc=%d\n", rc);
    return rc;
}


/**
 * Terminates the GMM component.
 */
/*
  销毁阶段：作为GMM模块的终止函数，负责逆向执行GMMR0Init的初始化流程
  资源清理：释放所有动态分配的资源（内存、锁、数据结构等）
  状态终结：确保系统处于可安全卸载状态（避免内存泄漏或悬垂指针）
 * */
GMMR0DECL(void) GMMR0Term(void)
{
    LogFlow(("GMMTerm:\n"));

    /*
     * Take care / be paranoid...
     */
    PGMM pGMM = g_pGMM;
    if (!RT_VALID_PTR(pGMM))
        return;
    if (pGMM->u32Magic != GMM_MAGIC)// 魔数校验
    {
        SUPR0Printf("GMMR0Term: u32Magic=%#x\n", pGMM->u32Magic);
        return;
    }

    /*
     * Undo what init did and free all the resources we've acquired.
     */
    /* Destroy the fundamentals. */
    g_pGMM = NULL; // 清除全局指针
    pGMM->u32Magic    = ~GMM_MAGIC;// 反转魔数
    /*
     * 锁销毁序列
锁类型	    销毁方式	                               注意事项
全局锁	    RTCritSectDelete/RTSemFastMutexDestroy	根据编译选项选择实现
自旋锁	    RTSpinlockDestroy	                    销毁树状结构锁和ChunkID锁
块级锁池	遍历aChunkMtx数组销毁	                需断言检查cUsers == 0（无残留用户）
     * */
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    RTCritSectDelete(&pGMM->GiantCritSect);
#else
    RTSemFastMutexDestroy(pGMM->hMtx);
    pGMM->hMtx        = NIL_RTSEMFASTMUTEX;
#endif
    RTSpinlockDestroy(pGMM->hSpinLockTree);
    pGMM->hSpinLockTree = NIL_RTSPINLOCK;
    RTSpinlockDestroy(pGMM->hSpinLockChunkId);
    pGMM->hSpinLockChunkId = NIL_RTSPINLOCK;

    /* Free any chunks still hanging around. */
    //使用AVL树销毁回调gmmR0TermDestroyChunk释放所有残留内存块
    RTAvlU32Destroy(&pGMM->pChunks, gmmR0TermDestroyChunk, pGMM);

    /* Destroy the chunk locks. */
    for (unsigned iMtx = 0; iMtx < RT_ELEMENTS(pGMM->aChunkMtx); iMtx++)
    {
        Assert(pGMM->aChunkMtx[iMtx].cUsers == 0);
        RTSemFastMutexDestroy(pGMM->aChunkMtx[iMtx].hMtx);
        pGMM->aChunkMtx[iMtx].hMtx = NIL_RTSEMFASTMUTEX;
    }

    /* Finally the instance data itself. */
    RTMemFree(pGMM);// 最终释放GMM实例自身
    LogFlow(("GMMTerm: done\n"));
}


/**
 * RTAvlU32Destroy callback.
 *
 * @returns 0
 * @param   pNode   The node to destroy.
 * @param   pvGMM   The GMM handle.
 */
static DECLCALLBACK(int) gmmR0TermDestroyChunk(PAVLU32NODECORE pNode, void *pvGMM)
{
    PGMMCHUNK pChunk = (PGMMCHUNK)pNode;

    if (pChunk->cFree != GMM_CHUNK_NUM_PAGES)
        SUPR0Printf("GMMR0Term: %RKv/%#x: cFree=%d cPrivate=%d cShared=%d cMappings=%d\n", pChunk,
                    pChunk->Core.Key, pChunk->cFree, pChunk->cPrivate, pChunk->cShared, pChunk->cMappingsX);

    int rc = RTR0MemObjFree(pChunk->hMemObj, true /* fFreeMappings */);
    if (RT_FAILURE(rc))
    {
        SUPR0Printf("GMMR0Term: %RKv/%#x: RTRMemObjFree(%RKv,true) -> %d (cMappings=%d)\n", pChunk,
                    pChunk->Core.Key, pChunk->hMemObj, rc, pChunk->cMappingsX);
        AssertRC(rc);
    }
    pChunk->hMemObj = NIL_RTR0MEMOBJ;

    RTMemFree(pChunk->paMappingsX);
    pChunk->paMappingsX = NULL;

    RTMemFree(pChunk);
    NOREF(pvGMM);
    return 0;
}


/**
 * Initializes the per-VM data for the GMM.
 *
 * This is called from within the GVMM lock (from GVMMR0CreateVM)
 * and should only initialize the data members so GMMR0CleanupVM
 * can deal with them. We reserve no memory or anything here,
 * that's done later in GMMR0InitVM.
 *
 * @param   pGVM    Pointer to the Global VM structure.
 */
GMMR0DECL(int) GMMR0InitPerVMData(PGVM pGVM)
{
    AssertCompile(RT_SIZEOFMEMB(GVM,gmm.s) <= RT_SIZEOFMEMB(GVM,gmm.padding));

    pGVM->gmm.s.Stats.enmPolicy = GMMOCPOLICY_INVALID;
    pGVM->gmm.s.Stats.enmPriority = GMMPRIORITY_INVALID;
    pGVM->gmm.s.Stats.fMayAllocate = false;

    pGVM->gmm.s.hChunkTlbSpinLock = NIL_RTSPINLOCK;
    int rc = RTSpinlockCreate(&pGVM->gmm.s.hChunkTlbSpinLock, RTSPINLOCK_FLAGS_INTERRUPT_SAFE, "per-vm-chunk-tlb");
    AssertRCReturn(rc, rc);

    return VINF_SUCCESS;
}


/**
 * Acquires the GMM giant lock.
 *
 * @returns Assert status code from RTSemFastMutexRequest.
 * @param   pGMM        Pointer to the GMM instance.
 */
static int gmmR0MutexAcquire(PGMM pGMM)
{
    ASMAtomicIncU32(&pGMM->cMtxContenders);
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    int rc = RTCritSectEnter(&pGMM->GiantCritSect);
#else
    int rc = RTSemFastMutexRequest(pGMM->hMtx);
#endif
    ASMAtomicDecU32(&pGMM->cMtxContenders);
    AssertRC(rc);
#ifdef VBOX_STRICT
    pGMM->hMtxOwner = RTThreadNativeSelf();
#endif
    return rc;
}


/**
 * Releases the GMM giant lock.
 *
 * @returns Assert status code from RTSemFastMutexRequest.
 * @param   pGMM        Pointer to the GMM instance.
 */
static int gmmR0MutexRelease(PGMM pGMM)
{
#ifdef VBOX_STRICT
    pGMM->hMtxOwner = NIL_RTNATIVETHREAD;
#endif
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    int rc = RTCritSectLeave(&pGMM->GiantCritSect);
#else
    int rc = RTSemFastMutexRelease(pGMM->hMtx);
    AssertRC(rc);
#endif
    return rc;
}


/**
 * Yields the GMM giant lock if there is contention and a certain minimum time
 * has elapsed since we took it.
 *
 * @returns @c true if the mutex was yielded, @c false if not.
 * @param   pGMM            Pointer to the GMM instance.
 * @param   puLockNanoTS    Where the lock acquisition time stamp is kept
 *                          (in/out).
 */
//全局锁主动让出
/*
  协作式并发控制：通过主动让出全局锁（Giant Lock）减少线程争用
  自适应性：仅在检测到实际竞争（cMtxContenders > 0）且持有锁超时（≥2ms）时触发
  公平性保障：通过RTThreadYield()让出CPU时间片，避免线程饥饿
 * */
static bool gmmR0MutexYield(PGMM pGMM, uint64_t *puLockNanoTS)
{
    /*
     * If nobody is contending the mutex, don't bother checking the time.
     */
    if (ASMAtomicReadU32(&pGMM->cMtxContenders) == 0)
        return false;

    /*
     * Don't yield if we haven't executed for at least 2 milliseconds.
     */
    uint64_t uNanoNow = RTTimeSystemNanoTS();
    if (uNanoNow - *puLockNanoTS < UINT32_C(2000000))
        return false;

    /*
     * Yield the mutex.
     */
#ifdef VBOX_STRICT
    pGMM->hMtxOwner = NIL_RTNATIVETHREAD;
#endif
    //为何在释放全局锁之前要标记为竞争状态?
    /*
      原子递增必要性：在释放锁与后续重获锁的间隙期，其他线程可能修改共享资源。
        通过预先增加竞争者计数，确保后续线程能感知当前存在并发访问需求
      状态同步保障：避免出现“释放锁后→ 其他线程误判无竞争→ 跳过等待”的竞态场景
    */
    ASMAtomicIncU32(&pGMM->cMtxContenders); // 标记为竞争状态
#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    int rc1 = RTCritSectLeave(&pGMM->GiantCritSect); AssertRC(rc1);
#else
    int rc1 = RTSemFastMutexRelease(pGMM->hMtx); AssertRC(rc1);// 释放全局锁
#endif

    RTThreadYield();                         // 主动让出CPU

#ifdef VBOX_USE_CRIT_SECT_FOR_GIANT
    int rc2 = RTCritSectEnter(&pGMM->GiantCritSect); AssertRC(rc2);
#else
    int rc2 = RTSemFastMutexRequest(pGMM->hMtx); AssertRC(rc2);// 重新获取锁
#endif
    *puLockNanoTS = RTTimeSystemNanoTS();
    ASMAtomicDecU32(&pGMM->cMtxContenders);// 清除竞争标记
#ifdef VBOX_STRICT
    pGMM->hMtxOwner = RTThreadNativeSelf();
#endif

    return true;
}


/**
 * Acquires a chunk lock.
 *
 * The caller must own the giant lock.
 *
 * @returns Assert status code from RTSemFastMutexRequest.
 * @param   pMtxState   The chunk mutex state info.  (Avoids
 *                      passing the same flags and stuff around
 *                      for subsequent release and drop-giant
 *                      calls.)
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pChunk      Pointer to the chunk.
 * @param   fFlags      Flags regarding the giant lock, GMMR0CHUNK_MTX_XXX.
 */
//分块互斥锁获取
/*
 *两锁协作机制
交互阶段	全局锁状态	        块锁状态	目的
初始操作	持有	            未持有	    安全查找/分配块锁索引56
核心操作	可释放（DROP_GIANT）持有	    允许其他线程并发操作非冲突内存块6
收尾操作	可重获（RETAKE）	释放	    确保全局状态一致性（如引用计数归零检查）

允许其他线程并发操作非冲突内存块 内存块不冲突了为何还需要块锁?
1. 内存块元数据共享冲突
     元数据竞争：不同内存块可能共享全局管理结构（如空闲链表、引用计数表），修改非冲突块的元数据仍可能引发数据竞争12
     示例：两个线程同时修改不同chunk的cUsers引用计数时，若不加锁会导致原子操作失效
   全局状态同步：内存分配/释放操作需要更新全局位图或树状结构，这些共享数据结构必须受保护58
2. 操作原子性要求
     复合操作保护：单个内存块操作（如分配）可能包含多个不可分割的子步骤：
         1. 检查空闲位图 → 2. 修改映射表 → 3. 更新统计计数器
     即使操作不同内存块，这些步骤仍需保证原子性15
     内存一致性：防止CPU缓存未及时刷新导致其他线程读取到中间状态

 * */
static int gmmR0ChunkMutexAcquire(PGMMR0CHUNKMTXSTATE pMtxState, PGMM pGMM, PGMMCHUNK pChunk, uint32_t fFlags)
{
    Assert(fFlags > GMMR0CHUNK_MTX_INVALID && fFlags < GMMR0CHUNK_MTX_END);
    Assert(pGMM->hMtxOwner == RTThreadNativeSelf());//调用线程必须已持有全局GMM锁

    pMtxState->pGMM   = pGMM;
    pMtxState->fFlags = (uint8_t)fFlags;

    /*
     * Get the lock index and reference the lock.
     */
    Assert(pGMM->hMtxOwner == RTThreadNativeSelf());
    uint32_t iChunkMtx = pChunk->iChunkMtx;
    if (iChunkMtx == UINT8_MAX) // 首次分配索引
    {
        //动态分配策略：采用轮询分配（iNextChunkMtx）避免热点争用
        //冲突解决：最多三次回退尝试寻找空闲锁槽（cUsers == 0）
        iChunkMtx = pGMM->iNextChunkMtx++;
        iChunkMtx %= RT_ELEMENTS(pGMM->aChunkMtx);

        /* Try get an unused one... */
        if (pGMM->aChunkMtx[iChunkMtx].cUsers)
        {
            iChunkMtx = pGMM->iNextChunkMtx++;
            iChunkMtx %= RT_ELEMENTS(pGMM->aChunkMtx);
            if (pGMM->aChunkMtx[iChunkMtx].cUsers)
            {
                iChunkMtx = pGMM->iNextChunkMtx++;
                iChunkMtx %= RT_ELEMENTS(pGMM->aChunkMtx);
                if (pGMM->aChunkMtx[iChunkMtx].cUsers)
                {
                    iChunkMtx = pGMM->iNextChunkMtx++;
                    iChunkMtx %= RT_ELEMENTS(pGMM->aChunkMtx);
                }
            }
        }

        pChunk->iChunkMtx = iChunkMtx;
    }
    AssertCompile(RT_ELEMENTS(pGMM->aChunkMtx) < UINT8_MAX);
    pMtxState->iChunkMtx = (uint8_t)iChunkMtx;
    ASMAtomicIncU32(&pGMM->aChunkMtx[iChunkMtx].cUsers);//引用计数

    /*
     * Drop the giant?
     */
    if (fFlags != GMMR0CHUNK_MTX_KEEP_GIANT)
    {
        /** @todo GMM life cycle cleanup (we may race someone
         *        destroying and cleaning up GMM)? */
        gmmR0MutexRelease(pGMM); // 释放全局锁
    }

    /*
     * Take the chunk mutex.
     */
    int rc = RTSemFastMutexRequest(pGMM->aChunkMtx[iChunkMtx].hMtx);
    AssertRC(rc);
    return rc;
}


/**
 * Releases the GMM giant lock.
 *
 * @returns Assert status code from RTSemFastMutexRequest.
 * @param   pMtxState   Pointer to the chunk mutex state.
 * @param   pChunk      Pointer to the chunk if it's still
 *                      alive, NULL if it isn't.  This is used to deassociate
 *                      the chunk from the mutex on the way out so a new one
 *                      can be selected next time, thus avoiding contented
 *                      mutexes.
 */
//分块互斥锁释放
static int gmmR0ChunkMutexRelease(PGMMR0CHUNKMTXSTATE pMtxState, PGMMCHUNK pChunk)
{
    PGMM pGMM = pMtxState->pGMM;

    /*
     * Release the chunk mutex and reacquire the giant if requested.
     */
    //先释放块级锁（RTSemFastMutexRelease），再根据标志位决定是否重获全局锁（gmmR0MutexAcquire）
    int rc = RTSemFastMutexRelease(pGMM->aChunkMtx[pMtxState->iChunkMtx].hMtx);
    AssertRC(rc);
    if (pMtxState->fFlags == GMMR0CHUNK_MTX_RETAKE_GIANT)
        rc = gmmR0MutexAcquire(pGMM);
    else
        Assert((pMtxState->fFlags != GMMR0CHUNK_MTX_DROP_GIANT) == (pGMM->hMtxOwner == RTThreadNativeSelf()));

    /*
     * Drop the chunk mutex user reference and deassociate it from the chunk
     * when possible.
     */
    //减少块锁的引用计数
    //当引用归零时，将块结构中的iChunkMtx置为UINT8_MAX解除关联
    if (   ASMAtomicDecU32(&pGMM->aChunkMtx[pMtxState->iChunkMtx].cUsers) == 0
        && pChunk
        && RT_SUCCESS(rc) )
    {
        if (pMtxState->fFlags != GMMR0CHUNK_MTX_DROP_GIANT)
            pChunk->iChunkMtx = UINT8_MAX;
        else
        {
            //在DROP_GIANT模式下需重获全局锁后二次检查引用计数，避免竞态条件
            /*
             * 当线程A释放块锁后、重获全局锁前，可能发生：
               线程B同时获取全局锁并修改块锁引用计数
               线程A的旧引用计数检查结果失效（cUsers可能被其他线程修改)
               所以线程A需要重新检查cUsers
             * */
            rc = gmmR0MutexAcquire(pGMM);// 重获全局锁
            if (RT_SUCCESS(rc))
            {
                if (pGMM->aChunkMtx[pMtxState->iChunkMtx].cUsers == 0) // 二次检查
                    pChunk->iChunkMtx = UINT8_MAX;
                rc = gmmR0MutexRelease(pGMM);
            }
        }
    }

    pMtxState->pGMM = NULL;
    return rc;
}


/**
 * Drops the giant GMM lock we kept in gmmR0ChunkMutexAcquire while keeping the
 * chunk locked.
 *
 * This only works if gmmR0ChunkMutexAcquire was called with
 * GMMR0CHUNK_MTX_KEEP_GIANT.  gmmR0ChunkMutexRelease will retake the giant
 * mutex, i.e. behave as if GMMR0CHUNK_MTX_RETAKE_GIANT was used.
 *
 * @returns VBox status code (assuming success is ok).
 * @param   pMtxState   Pointer to the chunk mutex state.
 */
static int gmmR0ChunkMutexDropGiant(PGMMR0CHUNKMTXSTATE pMtxState)
{
    AssertReturn(pMtxState->fFlags == GMMR0CHUNK_MTX_KEEP_GIANT, VERR_GMM_MTX_FLAGS);
    Assert(pMtxState->pGMM->hMtxOwner == RTThreadNativeSelf());
    pMtxState->fFlags = GMMR0CHUNK_MTX_RETAKE_GIANT;
    /** @todo GMM life cycle cleanup (we may race someone
     *        destroying and cleaning up GMM)? */
    return gmmR0MutexRelease(pMtxState->pGMM);
}


/**
 * For experimenting with NUMA affinity and such.
 *
 * @returns The current NUMA Node ID.
 */
static uint16_t gmmR0GetCurrentNumaNodeId(void)
{
#if 1
    return GMM_CHUNK_NUMA_ID_UNKNOWN;
#else
    return RTMpCpuId() / 16;
#endif
}



/**
 * Cleans up when a VM is terminating.
 *
 * @param   pGVM    Pointer to the Global VM structure.
 */
//虚拟机清理
/*
  内存资源回收：彻底清理虚拟机占用的私有页、共享页和固定内存页
  全局状态维护：更新GMM全局统计信息（cRegisteredVMs、cAllocatedPages等）
  异常处理：检测并报告"内存泄漏"（未正确释放的页面）
*/
GMMR0DECL(void) GMMR0CleanupVM(PGVM pGVM)
{
    LogFlow(("GMMR0CleanupVM: pGVM=%p:{.hSelf=%#x}\n", pGVM, pGVM->hSelf));

    PGMM pGMM;
    GMM_GET_VALID_INSTANCE_VOID(pGMM);

#ifdef VBOX_WITH_PAGE_SHARING
    /*
     * Clean up all registered shared modules first.
     */
    gmmR0SharedModuleCleanup(pGMM, pGVM);
#endif

    gmmR0MutexAcquire(pGMM);
    uint64_t uLockNanoTS = RTTimeSystemNanoTS();
    GMM_CHECK_SANITY_UPON_ENTERING(pGMM);

    /*
     * The policy is 'INVALID' until the initial reservation
     * request has been serviced.
     */
    if (    pGVM->gmm.s.Stats.enmPolicy > GMMOCPOLICY_INVALID
        &&  pGVM->gmm.s.Stats.enmPolicy < GMMOCPOLICY_END)
    {
        /*
         * If it's the last VM around, we can skip walking all the chunk looking
         * for the pages owned by this VM and instead flush the whole shebang.
         *
         * This takes care of the eventuality that a VM has left shared page
         * references behind (shouldn't happen of course, but you never know).
         */
        Assert(pGMM->cRegisteredVMs);
        pGMM->cRegisteredVMs--;

        /*
         * Walk the entire pool looking for pages that belong to this VM
         * and leftover mappings.  (This'll only catch private pages,
         * shared pages will be 'left behind'.)
         */
        /** @todo r=bird: This scanning+freeing could be optimized in bound mode! */
        uint64_t    cPrivatePages = pGVM->gmm.s.Stats.cPrivatePages; /* save */

        unsigned    iCountDown = 64;
        bool        fRedoFromStart;
        PGMMCHUNK   pChunk;
        do
        {
            fRedoFromStart = false;
            //反向遍历：从最新分配的内存块开始扫描（RTListForEachReverse）
            RTListForEachReverse(&pGMM->ChunkList, pChunk, GMMCHUNK, ListNode)
            {
                //绑定模式优化：在绑定内存模式下只处理属于当前VM的chunk
                uint32_t const cFreeChunksOld = pGMM->cFreedChunks;
                if (   (   !pGMM->fBoundMemoryMode
                        || pChunk->hGVM == pGVM->hSelf)
                    && gmmR0CleanupVMScanChunk(pGMM, pGVM, pChunk))
                {
                    /* We left the giant mutex, so reset the yield counters. */
                    uLockNanoTS = RTTimeSystemNanoTS();
                    iCountDown  = 64;
                }
                else
                {
                    /* Didn't leave it, so do normal yielding. */
                    //锁控制：通过iCountDown实现周期性锁让步（gmmR0MutexYield）
                    if (!iCountDown)
                        gmmR0MutexYield(pGMM, &uLockNanoTS);
                    else
                        iCountDown--;
                }
                if (pGMM->cFreedChunks != cFreeChunksOld)
                {
                    fRedoFromStart = true;
                    break;
                }
            }
        } while (fRedoFromStart);

        if (pGVM->gmm.s.Stats.cPrivatePages)
            SUPR0Printf("GMMR0CleanupVM: hGVM=%#x has %#x private pages that cannot be found!\n", pGVM->hSelf, pGVM->gmm.s.Stats.cPrivatePages);

        pGMM->cAllocatedPages -= cPrivatePages;

        /*
         * Free empty chunks.
         */
        PGMMCHUNKFREESET pPrivateSet = pGMM->fBoundMemoryMode ? &pGVM->gmm.s.Private : &pGMM->PrivateX;
        do
        {
            fRedoFromStart = false;
            iCountDown = 10240;
            pChunk = pPrivateSet->apLists[GMM_CHUNK_FREE_SET_UNUSED_LIST];
            while (pChunk)
            {
                PGMMCHUNK pNext = pChunk->pFreeNext;
                //专门回收完全空闲的chunk
                Assert(pChunk->cFree == GMM_CHUNK_NUM_PAGES);
                if (   !pGMM->fBoundMemoryMode
                    || pChunk->hGVM == pGVM->hSelf)
                {
                    //通过idGeneration检测链表结构变化
                    uint64_t const idGenerationOld = pPrivateSet->idGeneration;
                    if (gmmR0FreeChunk(pGMM, pGVM, pChunk, true /*fRelaxedSem*/))
                    {
                        /* We've left the giant mutex, restart? (+1 for our unlink) */
                        fRedoFromStart = pPrivateSet->idGeneration != idGenerationOld + 1;
                        if (fRedoFromStart)
                            break;
                        uLockNanoTS = RTTimeSystemNanoTS();
                        iCountDown = 10240;
                    }
                }

                /* Advance and maybe yield the lock. */
                pChunk = pNext;
                if (--iCountDown == 0)
                {
                    uint64_t const idGenerationOld = pPrivateSet->idGeneration;
                    fRedoFromStart = gmmR0MutexYield(pGMM, &uLockNanoTS)
                                  && pPrivateSet->idGeneration != idGenerationOld;
                    if (fRedoFromStart)
                        break;
                    iCountDown = 10240;
                }
            }
        } while (fRedoFromStart);

        /*
         * Account for shared pages that weren't freed.
         */
        //强制扣除未释放的共享页计数（最后手段）
        if (pGVM->gmm.s.Stats.cSharedPages)
        {
            Assert(pGMM->cSharedPages >= pGVM->gmm.s.Stats.cSharedPages);
            SUPR0Printf("GMMR0CleanupVM: hGVM=%#x left %#x shared pages behind!\n", pGVM->hSelf, pGVM->gmm.s.Stats.cSharedPages);
            pGMM->cLeftBehindSharedPages += pGVM->gmm.s.Stats.cSharedPages;
        }

        /*
         * Clean up balloon statistics in case the VM process crashed.
         */
        Assert(pGMM->cBalloonedPages >= pGVM->gmm.s.Stats.cBalloonedPages);
        pGMM->cBalloonedPages -= pGVM->gmm.s.Stats.cBalloonedPages;

        /*
         * Update the over-commitment management statistics.
         */
        pGMM->cReservedPages -= pGVM->gmm.s.Stats.Reserved.cBasePages
                              + pGVM->gmm.s.Stats.Reserved.cFixedPages
                              + pGVM->gmm.s.Stats.Reserved.cShadowPages;
        switch (pGVM->gmm.s.Stats.enmPolicy)
        {
            case GMMOCPOLICY_NO_OC:
                break;
            default:
                /** @todo Update GMM->cOverCommittedPages */
                break;
        }
    }

    /* zap the GVM data. */
    pGVM->gmm.s.Stats.enmPolicy    = GMMOCPOLICY_INVALID;
    pGVM->gmm.s.Stats.enmPriority  = GMMPRIORITY_INVALID;
    pGVM->gmm.s.Stats.fMayAllocate = false;

    GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    gmmR0MutexRelease(pGMM);

    /*
     * Destroy the spinlock.
     */
    RTSPINLOCK hSpinlock = NIL_RTSPINLOCK;
    ASMAtomicXchgHandle(&pGVM->gmm.s.hChunkTlbSpinLock, NIL_RTSPINLOCK, &hSpinlock);
    RTSpinlockDestroy(hSpinlock);

    LogFlow(("GMMR0CleanupVM: returns\n"));
}


/**
 * Scan one chunk for private pages belonging to the specified VM.
 *
 * @note    This function may drop the giant mutex!
 *
 * @returns @c true if we've temporarily dropped the giant mutex, @c false if
 *          we didn't.
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        The global VM handle.
 * @param   pChunk      The chunk to scan.
 */
//虚拟机终止时清理其占用的内存资源
/*
  内存页回收：扫描指定内存块(chunk)，释放属于目标虚拟机(pGVM)的私有页
  状态校验：验证内存块统计信息(cFree/cPrivate/cShared)的准确性
  映射清理：解除虚拟机在该内存块上的所有内存映射
*/
/*
  struct GMMCHUNK {
      uint32_t            cFree;          // 空闲页计数
      uint32_t            cPrivate;       // 私有页计数
      uint32_t            cShared;        // 共享页计数
      uint16_t            iFreeHead;      // 空闲链表头
      struct {
          uint16_t        hGVM;           // 所属虚拟机句柄
          // ...其他字段
      } aPages[GMM_CHUNK_NUM_PAGES];     // 页描述符数组
      // ...其他字段
  };
*/
static bool gmmR0CleanupVMScanChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk)
{
    Assert(!pGMM->fBoundMemoryMode || pChunk->hGVM == pGVM->hSelf);

    /*
     * Look for pages belonging to the VM.
     * (Perform some internal checks while we're scanning.)
     */
#ifndef VBOX_STRICT
    if (pChunk->cFree != GMM_CHUNK_NUM_PAGES)
#endif
    {
        unsigned cPrivate = 0;
        unsigned cShared = 0;
        unsigned cFree = 0;

        gmmR0UnlinkChunk(pChunk);       /* avoiding cFreePages updates. */

        uint16_t hGVM = pGVM->hSelf;
        unsigned iPage = (GMM_CHUNK_SIZE >> GUEST_PAGE_SHIFT);
        while (iPage-- > 0)
            if (GMM_PAGE_IS_PRIVATE(&pChunk->aPages[iPage]))
            {
                // 释放私有页
                if (pChunk->aPages[iPage].Private.hGVM == hGVM)
                {
                    /*
                     * Free the page.
                     *
                     * The reason for not using gmmR0FreePrivatePage here is that we
                     * must *not* cause the chunk to be freed from under us - we're in
                     * an AVL tree walk here.
                     */
                    pChunk->aPages[iPage].u = 0;
                    pChunk->aPages[iPage].Free.u2State = GMM_PAGE_STATE_FREE; //将页面状态标记为GMM_PAGE_STATE_FREE
                    pChunk->aPages[iPage].Free.fZeroed = false;
                    pChunk->aPages[iPage].Free.iNext   = pChunk->iFreeHead;//更新空闲链表头(iFreeHead)
                    pChunk->iFreeHead = iPage;
                    pChunk->cPrivate--; //调整计数
                    pChunk->cFree++;
                    pGVM->gmm.s.Stats.cPrivatePages--;
                    cFree++;
                }
                else
                    cPrivate++;
            }
            // 统计其他类型页面
            else if (GMM_PAGE_IS_FREE(&pChunk->aPages[iPage]))
                cFree++;
            else
                cShared++;

        gmmR0SelectSetAndLinkChunk(pGMM, pGVM, pChunk);

        /*
         * Did it add up?
         */
        // 强制修正统计值
        if (RT_UNLIKELY(    pChunk->cFree != cFree
                        ||  pChunk->cPrivate != cPrivate
                        ||  pChunk->cShared != cShared))
        {
            SUPR0Printf("gmmR0CleanupVMScanChunk: Chunk %RKv/%#x has bogus stats - free=%d/%d private=%d/%d shared=%d/%d\n",
                        pChunk, pChunk->Core.Key, pChunk->cFree, cFree, pChunk->cPrivate, cPrivate, pChunk->cShared, cShared);
            pChunk->cFree = cFree;
            pChunk->cPrivate = cPrivate;
            pChunk->cShared = cShared;
        }
    }

    /*
     * If not in bound memory mode, we should reset the hGVM field
     * if it has our handle in it.
     */
    if (pChunk->hGVM == pGVM->hSelf)
    {
        if (!g_pGMM->fBoundMemoryMode)
            pChunk->hGVM = NIL_GVM_HANDLE;
        else if (pChunk->cFree != GMM_CHUNK_NUM_PAGES)
        {
            SUPR0Printf("gmmR0CleanupVMScanChunk: %RKv/%#x: cFree=%#x - it should be 0 in bound mode!\n",
                        pChunk, pChunk->Core.Key, pChunk->cFree);//通过SUPR0Printf输出错误详情（生产环境可见）
            AssertMsgFailed(("%p/%#x: cFree=%#x - it should be 0 in bound mode!\n", pChunk, pChunk->Core.Key, pChunk->cFree));

            gmmR0UnlinkChunk(pChunk);
            pChunk->cFree = GMM_CHUNK_NUM_PAGES;
            gmmR0SelectSetAndLinkChunk(pGMM, pGVM, pChunk);
        }
    }

    /*
     * Look for a mapping belonging to the terminating VM.
     */
    GMMR0CHUNKMTXSTATE MtxState;
    gmmR0ChunkMutexAcquire(&MtxState, pGMM, pChunk, GMMR0CHUNK_MTX_KEEP_GIANT);
    unsigned cMappings = pChunk->cMappingsX;
    for (unsigned i = 0; i < cMappings; i++)
        //// 从映射数组中移除该项
        if (pChunk->paMappingsX[i].pGVM == pGVM)
        {
            gmmR0ChunkMutexDropGiant(&MtxState);

            RTR0MEMOBJ hMemObj = pChunk->paMappingsX[i].hMapObj;

            cMappings--;
            if (i < cMappings)
                 pChunk->paMappingsX[i] = pChunk->paMappingsX[cMappings];
            pChunk->paMappingsX[cMappings].pGVM    = NULL;
            pChunk->paMappingsX[cMappings].hMapObj = NIL_RTR0MEMOBJ;
            Assert(pChunk->cMappingsX - 1U == cMappings);
            pChunk->cMappingsX = cMappings;

            //释放宿主内存对象
            int rc = RTR0MemObjFree(hMemObj, false /* fFreeMappings (NA) */);
            if (RT_FAILURE(rc))
            {
                SUPR0Printf("gmmR0CleanupVMScanChunk: %RKv/%#x: mapping #%x: RTRMemObjFree(%RKv,false) -> %d \n",
                            pChunk, pChunk->Core.Key, i, hMemObj, rc);
                AssertRC(rc);
            }

            gmmR0ChunkMutexRelease(&MtxState, pChunk);
            return true;
        }

    gmmR0ChunkMutexRelease(&MtxState, pChunk);
    return false;
}


/**
 * The initial resource reservations.
 *
 * This will make memory reservations according to policy and priority. If there aren't
 * sufficient resources available to sustain the VM this function will fail and all
 * future allocations requests will fail as well.
 *
 * These are just the initial reservations made very very early during the VM creation
 * process and will be adjusted later in the GMMR0UpdateReservation call after the
 * ring-3 init has completed.
 *
 * @returns VBox status code.
 * @retval  VERR_GMM_MEMORY_RESERVATION_DECLINED
 * @retval  VERR_GMM_
 *
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idCpu           The VCPU id - must be zero.
 * @param   cBasePages      The number of pages that may be allocated for the base RAM and ROMs.
 *                          This does not include MMIO2 and similar.
 * @param   cShadowPages    The number of pages that may be allocated for shadow paging structures.
 * @param   cFixedPages     The number of pages that may be allocated for fixed objects like the
 *                          hyper heap, MMIO2 and similar.
 * @param   enmPolicy       The OC policy to use on this VM.
 * @param   enmPriority     The priority in an out-of-memory situation.
 *
 * @thread  The creator thread / EMT(0).
 */
GMMR0DECL(int) GMMR0InitialReservation(PGVM pGVM, VMCPUID idCpu, uint64_t cBasePages, uint32_t cShadowPages,
                                       uint32_t cFixedPages, GMMOCPOLICY enmPolicy, GMMPRIORITY enmPriority)
{
    LogFlow(("GMMR0InitialReservation: pGVM=%p cBasePages=%#llx cShadowPages=%#x cFixedPages=%#x enmPolicy=%d enmPriority=%d\n",
             pGVM, cBasePages, cShadowPages, cFixedPages, enmPolicy, enmPriority));

    /*
     * Validate, get basics and take the semaphore.
     */
    AssertReturn(idCpu == 0, VERR_INVALID_CPU_ID);
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    AssertReturn(cBasePages, VERR_INVALID_PARAMETER);
    AssertReturn(cShadowPages, VERR_INVALID_PARAMETER);
    AssertReturn(cFixedPages, VERR_INVALID_PARAMETER);
    AssertReturn(enmPolicy > GMMOCPOLICY_INVALID && enmPolicy < GMMOCPOLICY_END, VERR_INVALID_PARAMETER);
    AssertReturn(enmPriority > GMMPRIORITY_INVALID && enmPriority < GMMPRIORITY_END, VERR_INVALID_PARAMETER);

    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        if (    !pGVM->gmm.s.Stats.Reserved.cBasePages
            &&  !pGVM->gmm.s.Stats.Reserved.cFixedPages
            &&  !pGVM->gmm.s.Stats.Reserved.cShadowPages)
        {
            /*
             * Check if we can accommodate this.
             */
            /* ... later ... */
            if (RT_SUCCESS(rc))
            {
                /*
                 * Update the records.
                 */
                pGVM->gmm.s.Stats.Reserved.cBasePages   = cBasePages;
                pGVM->gmm.s.Stats.Reserved.cFixedPages  = cFixedPages;
                pGVM->gmm.s.Stats.Reserved.cShadowPages = cShadowPages;
                pGVM->gmm.s.Stats.enmPolicy             = enmPolicy;
                pGVM->gmm.s.Stats.enmPriority           = enmPriority;
                pGVM->gmm.s.Stats.fMayAllocate          = true;

                pGMM->cReservedPages += cBasePages + cFixedPages + cShadowPages;
                pGMM->cRegisteredVMs++;
            }
        }
        else
            rc = VERR_WRONG_ORDER;
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;
    gmmR0MutexRelease(pGMM);
    LogFlow(("GMMR0InitialReservation: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0InitialReservation.
 *
 * @returns see GMMR0InitialReservation.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idCpu           The VCPU id.
 * @param   pReq            Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0InitialReservationReq(PGVM pGVM, VMCPUID idCpu, PGMMINITIALRESERVATIONREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pGVM, VERR_INVALID_POINTER);
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GMMR0InitialReservation(pGVM, idCpu, pReq->cBasePages, pReq->cShadowPages,
                                   pReq->cFixedPages, pReq->enmPolicy, pReq->enmPriority);
}


/**
 * This updates the memory reservation with the additional MMIO2 and ROM pages.
 *
 * @returns VBox status code.
 * @retval  VERR_GMM_MEMORY_RESERVATION_DECLINED
 *
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idCpu           The VCPU id.
 * @param   cBasePages      The number of pages that may be allocated for the base RAM and ROMs.
 *                          This does not include MMIO2 and similar.
 * @param   cShadowPages    The number of pages that may be allocated for shadow paging structures.
 * @param   cFixedPages     The number of pages that may be allocated for fixed objects like the
 *                          hyper heap, MMIO2 and similar.
 *
 * @thread  EMT(idCpu)
 */
//更新虚拟机内存预留
/*
  内存预留更新：调整虚拟机(GVM)的三种内存页预留数量：
    cBasePages：普通客户机物理内存页
    cShadowPages：影子页表（用于硬件虚拟化加速）
    cFixedPages：固定用途内存页（如DMA缓冲区）
*/
/*
典型应用场景
 该函数被以下场景调用：
   虚拟机启动配置：初始化内存预留
   动态内存调整：通过VMMDev实现热调整
   内存气球驱动：Ballooning机制调整预留值
*/
GMMR0DECL(int) GMMR0UpdateReservation(PGVM pGVM, VMCPUID idCpu, uint64_t cBasePages,
                                      uint32_t cShadowPages, uint32_t cFixedPages)
{
    LogFlow(("GMMR0UpdateReservation: pGVM=%p cBasePages=%#llx cShadowPages=%#x cFixedPages=%#x\n",
             pGVM, cBasePages, cShadowPages, cFixedPages));

    /*
     * Validate, get basics and take the semaphore.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    AssertReturn(cBasePages, VERR_INVALID_PARAMETER);
    AssertReturn(cShadowPages, VERR_INVALID_PARAMETER);
    AssertReturn(cFixedPages, VERR_INVALID_PARAMETER);

    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        if (    pGVM->gmm.s.Stats.Reserved.cBasePages
            &&  pGVM->gmm.s.Stats.Reserved.cFixedPages
            &&  pGVM->gmm.s.Stats.Reserved.cShadowPages)
        {
            /*
             * Check if we can accommodate this.
             */
            /* ... later ... */
            if (RT_SUCCESS(rc))
            {
                /*
                 * Update the records.
                 */
                pGMM->cReservedPages -= pGVM->gmm.s.Stats.Reserved.cBasePages// 先减去旧值
                                      + pGVM->gmm.s.Stats.Reserved.cFixedPages
                                      + pGVM->gmm.s.Stats.Reserved.cShadowPages;
                pGMM->cReservedPages += cBasePages + cFixedPages + cShadowPages;// 再加新值

                pGVM->gmm.s.Stats.Reserved.cBasePages   = cBasePages;
                pGVM->gmm.s.Stats.Reserved.cFixedPages  = cFixedPages;
                pGVM->gmm.s.Stats.Reserved.cShadowPages = cShadowPages;
            }
        }
        else
            rc = VERR_WRONG_ORDER;
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;
    gmmR0MutexRelease(pGMM);
    LogFlow(("GMMR0UpdateReservation: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0UpdateReservation.
 *
 * @returns see GMMR0UpdateReservation.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idCpu           The VCPU id.
 * @param   pReq            Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0UpdateReservationReq(PGVM pGVM, VMCPUID idCpu, PGMMUPDATERESERVATIONREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GMMR0UpdateReservation(pGVM, idCpu, pReq->cBasePages, pReq->cShadowPages, pReq->cFixedPages);
}

#ifdef GMMR0_WITH_SANITY_CHECK

/**
 * Performs sanity checks on a free set.
 *
 * @returns Error count.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pSet        Pointer to the set.
 * @param   pszSetName  The set name.
 * @param   pszFunction The function from which it was called.
 * @param   uLine       The line number.
 */
/*
 * 内存一致性检查：验证空闲内存页的实际数量(cPages)与记录值(pSet->cFreePages)是否匹配
 * */
static uint32_t gmmR0SanityCheckSet(PGMM pGMM, PGMMCHUNKFREESET pSet, const char *pszSetName,
                                    const char *pszFunction, unsigned uLineNo)
{
    uint32_t cErrors = 0;

    /*
     * Count the free pages in all the chunks and match it against pSet->cFreePages.
     */
    uint32_t cPages = 0;
    //循环遍历所有空闲链表
    for (unsigned i = 0; i < RT_ELEMENTS(pSet->apLists); i++)
    {
        //遍历每个链表节点，累加各内存块的空闲页数(cFree)
        for (PGMMCHUNK pCur = pSet->apLists[i]; pCur; pCur = pCur->pFreeNext)
        {
            /** @todo check that the chunk is hash into the right set. */
            cPages += pCur->cFree;
        }
    }
    //一致性验证
    if (RT_UNLIKELY(cPages != pSet->cFreePages))
    {
        SUPR0Printf("GMM insanity: found %#x pages in the %s set, expected %#x. (%s, line %u)\n",
                    cPages, pszSetName, pSet->cFreePages, pszFunction, uLineNo);
        cErrors++;
    }

    return cErrors;
}


/**
 * Performs some sanity checks on the GMM while owning lock.
 *
 * @returns Error count.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pszFunction The function from which it is called.
 * @param   uLineNo     The line number.
 */
static uint32_t gmmR0SanityCheck(PGMM pGMM, const char *pszFunction, unsigned uLineNo)
{
    uint32_t cErrors = 0;

    cErrors += gmmR0SanityCheckSet(pGMM, &pGMM->PrivateX, "private", pszFunction, uLineNo);
    cErrors += gmmR0SanityCheckSet(pGMM, &pGMM->Shared,   "shared",  pszFunction, uLineNo);
    /** @todo add more sanity checks. */

    return cErrors;
}

#endif /* GMMR0_WITH_SANITY_CHECK */

/**
 * Looks up a chunk in the tree and fill in the TLB entry for it.
 *
 * This is not expected to fail and will bitch if it does.
 *
 * @returns Pointer to the allocation chunk, NULL if not found.
 * @param   pGMM        Pointer to the GMM instance.
 * @param   idChunk     The ID of the chunk to find.
 * @param   pTlbe       Pointer to the TLB entry.
 *
 * @note    Caller owns spinlock.
 */
static PGMMCHUNK gmmR0GetChunkSlow(PGMM pGMM, uint32_t idChunk, PGMMCHUNKTLBE pTlbe)
{
    PGMMCHUNK pChunk = (PGMMCHUNK)RTAvlU32Get(&pGMM->pChunks, idChunk);
    AssertMsgReturn(pChunk, ("Chunk %#x not found!\n", idChunk), NULL);
    pTlbe->idChunk = idChunk;
    pTlbe->pChunk = pChunk;
    return pChunk;
}


/**
 * Finds a allocation chunk, spin-locked.
 *
 * This is not expected to fail and will bitch if it does.
 *
 * @returns Pointer to the allocation chunk, NULL if not found.
 * @param   pGMM        Pointer to the GMM instance.
 * @param   idChunk     The ID of the chunk to find.
 */
//通过TLB（Translation Lookaside Buffer）加速查找内存块，若TLB未命中则触发慢路径查询
DECLINLINE(PGMMCHUNK) gmmR0GetChunkLocked(PGMM pGMM, uint32_t idChunk)
{
    /*
     * Do a TLB lookup, branch if not in the TLB.
     */
    PGMMCHUNKTLBE pTlbe  = &pGMM->ChunkTLB.aEntries[GMM_CHUNKTLB_IDX(idChunk)];
    PGMMCHUNK     pChunk = pTlbe->pChunk;
    if (   pChunk == NULL
        || pTlbe->idChunk != idChunk)
        pChunk = gmmR0GetChunkSlow(pGMM, idChunk, pTlbe);
    return pChunk;
}


/**
 * Finds a allocation chunk.
 *
 * This is not expected to fail and will bitch if it does.
 *
 * @returns Pointer to the allocation chunk, NULL if not found.
 * @param   pGMM        Pointer to the GMM instance.
 * @param   idChunk     The ID of the chunk to find.
 */
DECLINLINE(PGMMCHUNK) gmmR0GetChunk(PGMM pGMM, uint32_t idChunk)
{
    RTSpinlockAcquire(pGMM->hSpinLockTree);
    PGMMCHUNK pChunk = gmmR0GetChunkLocked(pGMM, idChunk);
    RTSpinlockRelease(pGMM->hSpinLockTree);
    return pChunk;
}


/**
 * Finds a page.
 *
 * This is not expected to fail and will bitch if it does.
 *
 * @returns Pointer to the page, NULL if not found.
 * @param   pGMM        Pointer to the GMM instance.
 * @param   idPage      The ID of the page to find.
 */
DECLINLINE(PGMMPAGE) gmmR0GetPage(PGMM pGMM, uint32_t idPage)
{
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
    if (RT_LIKELY(pChunk))
        return &pChunk->aPages[idPage & GMM_PAGEID_IDX_MASK];
    return NULL;
}


#if 0 /* unused */
/**
 * Gets the host physical address for a page given by it's ID.
 *
 * @returns The host physical address or NIL_RTHCPHYS.
 * @param   pGMM        Pointer to the GMM instance.
 * @param   idPage      The ID of the page to find.
 */
DECLINLINE(RTHCPHYS) gmmR0GetPageHCPhys(PGMM pGMM,  uint32_t idPage)
{
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
    if (RT_LIKELY(pChunk))
        return RTR0MemObjGetPagePhysAddr(pChunk->hMemObj, idPage & GMM_PAGEID_IDX_MASK);
    return NIL_RTHCPHYS;
}
#endif /* unused */


/**
 * Selects the appropriate free list given the number of free pages.
 *
 * @returns Free list index.
 * @param   cFree       The number of free pages in the chunk.
 */
DECLINLINE(unsigned) gmmR0SelectFreeSetList(unsigned cFree)
{
    unsigned iList = cFree >> GMM_CHUNK_FREE_SET_SHIFT;
    AssertMsg(iList < RT_SIZEOFMEMB(GMMCHUNKFREESET, apLists) / RT_SIZEOFMEMB(GMMCHUNKFREESET, apLists[0]),
              ("%d (%u)\n", iList, cFree));
    return iList;
}


/**
 * Unlinks the chunk from the free list it's currently on (if any).
 *
 * @param   pChunk      The allocation chunk.
 */
DECLINLINE(void) gmmR0UnlinkChunk(PGMMCHUNK pChunk)
{
    PGMMCHUNKFREESET pSet = pChunk->pSet;
    if (RT_LIKELY(pSet))
    {
        pSet->cFreePages -= pChunk->cFree;
        pSet->idGeneration++;

        PGMMCHUNK pPrev = pChunk->pFreePrev;
        PGMMCHUNK pNext = pChunk->pFreeNext;
        if (pPrev)
            pPrev->pFreeNext = pNext;
        else
            pSet->apLists[gmmR0SelectFreeSetList(pChunk->cFree)] = pNext;
        if (pNext)
            pNext->pFreePrev = pPrev;

        pChunk->pSet = NULL;
        pChunk->pFreeNext = NULL;
        pChunk->pFreePrev = NULL;
    }
    else
    {
        Assert(!pChunk->pFreeNext);
        Assert(!pChunk->pFreePrev);
        Assert(!pChunk->cFree);
    }
}


/**
 * Links the chunk onto the appropriate free list in the specified free set.
 *
 * If no free entries, it's not linked into any list.
 *
 * @param   pChunk      The allocation chunk.
 * @param   pSet        The free set.
 */
//这个有点像linux内核内存的伙伴系统
DECLINLINE(void) gmmR0LinkChunk(PGMMCHUNK pChunk, PGMMCHUNKFREESET pSet)
{
    // 断言检查：确保该chunk当前不属于任何free set，且没有前后链接
    Assert(!pChunk->pSet);
    Assert(!pChunk->pFreeNext);
    Assert(!pChunk->pFreePrev);

    // 只有chunk中有空闲页时才进行链接
    if (pChunk->cFree > 0)
    {
        // 设置chunk所属的free set
        pChunk->pSet = pSet;
        pChunk->pFreePrev = NULL;
        // 根据空闲页数量选择适当的链表索引
        unsigned const iList = gmmR0SelectFreeSetList(pChunk->cFree);
        // 将chunk插入到链表头部
        pChunk->pFreeNext = pSet->apLists[iList];
        if (pChunk->pFreeNext)
            pChunk->pFreeNext->pFreePrev = pChunk;
        pSet->apLists[iList] = pChunk;

        // 更新free set的总空闲页数和生成ID
        pSet->cFreePages += pChunk->cFree;
        pSet->idGeneration++;
    }
}


/**
 * Links the chunk onto the appropriate free list in the specified free set.
 *
 * If no free entries, it's not linked into any list.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        Pointer to the kernel-only VM instace data.
 * @param   pChunk      The allocation chunk.
 */
/*
 PrivateX
动态扩展性
  通过位图（bmChunkId）和空闲链表（iFreeHead）实现高效内存块分配
  支持按需扩展，避免固定大小限制
隔离性
  内存块一旦分配为私有，仅归属单个虚拟机（通过 hGVM 标记），防止跨 VM 访问
性能优化
  与共享集合（Shared）分离，减少多虚拟机竞争全局锁的开销
  通过顺序分配（idChunkPrev）和局部扫描（ASMBitNextClear）降低查找延迟
典型应用
  内存热插拔：动态扩展虚拟机内存时，从私有扩展集合分配新块
  内存碎片整理：将碎片化空闲页合并后重新链入私有扩展集合
*/
DECLINLINE(void) gmmR0SelectSetAndLinkChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk)
{
    PGMMCHUNKFREESET pSet;
    if (pGMM->fBoundMemoryMode)
        pSet = &pGVM->gmm.s.Private;// 绑定模式：链接到虚拟机私有集合
    else if (pChunk->cShared)
        pSet = &pGMM->Shared;       // 共享页：链接到全局共享集合
    else
        pSet = &pGMM->PrivateX;   // 默认：链接到全局私有扩展集合
    gmmR0LinkChunk(pChunk, pSet);
}


/**
 * Frees a Chunk ID.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   idChunk     The Chunk ID to free.
 */
static void gmmR0FreeChunkId(PGMM pGMM, uint32_t idChunk)
{
    AssertReturnVoid(idChunk != NIL_GMM_CHUNKID);
    RTSpinlockAcquire(pGMM->hSpinLockChunkId); /* We could probably skip the locking here, I think. */

    AssertMsg(ASMBitTest(&pGMM->bmChunkId[0], idChunk), ("%#x\n", idChunk));
    ASMAtomicBitClear(&pGMM->bmChunkId[0], idChunk);

    RTSpinlockRelease(pGMM->hSpinLockChunkId);
}


/**
 * Allocates a new Chunk ID.
 *
 * @returns The Chunk ID.
 * @param   pGMM        Pointer to the GMM instance.
 */
//从全局位图（pGMM->bmChunkId）中分配一个 未被使用的 Chunk ID，用于标识新注册的内存块。
/*
  线程安全：通过自旋锁（pGMM->hSpinLockChunkId）保护位图操作。
  高效查找：顺序扫描 + 位图优化（ASMBitNextClear/ASMBitFirstClear）。
  ID 复用：避免频繁分配导致位图耗尽。
*/
/*
  三级查找策略：
    顺序分配（O(1) 常见情况）。
    局部扫描（O(n) 局部碎片时）。
    全局扫描（O(n) 极端情况）。
    缓存友好：idChunkPrev 减少扫描范围。
*/
static uint32_t gmmR0AllocateChunkId(PGMM pGMM)
{
    AssertCompile(!((GMM_CHUNKID_LAST + 1) & 31)); /* must be a multiple of 32 */ // 确保位图大小为 32 的倍数
    AssertCompile(NIL_GMM_CHUNKID == 0); // 空 ID 必须为 0

    RTSpinlockAcquire(pGMM->hSpinLockChunkId);// 获取锁,防止多线程同时修改位图。

    /*
     * Try the next sequential one.
     */
    int32_t idChunk = ++pGMM->idChunkPrev;
    if (   (uint32_t)idChunk <= GMM_CHUNKID_LAST
        && idChunk > NIL_GMM_CHUNKID)
    {
        //若该 ID 未被占用（位图对应位为 0），则通过 ASMAtomicBitTestAndSet 原子标记为已用。
        if (!ASMAtomicBitTestAndSet(&pGMM->bmChunkId[0], idChunk))
        {
            RTSpinlockRelease(pGMM->hSpinLockChunkId);
            return idChunk;// 成功分配
        }

        /*
         * Scan sequentially from the last one.
         */
        //若顺序分配失败，调用 ASMBitNextClear 从当前位置向后查找第一个空闲 ID。
        //确保找到的 ID 有效（> NIL_GMM_CHUNKID 且 <= GMM_CHUNKID_LAST）。
        if ((uint32_t)idChunk < GMM_CHUNKID_LAST)
        {
            idChunk = ASMBitNextClear(&pGMM->bmChunkId[0], GMM_CHUNKID_LAST + 1, idChunk);
            if (   idChunk > NIL_GMM_CHUNKID
                && (uint32_t)idChunk <= GMM_CHUNKID_LAST)
            {
                AssertMsgReturnStmt(!ASMAtomicBitTestAndSet(&pGMM->bmChunkId[0], idChunk), ("%#x\n", idChunk), // 标记为已用
                                    RTSpinlockRelease(pGMM->hSpinLockChunkId), NIL_GMM_CHUNKID);

                pGMM->idChunkPrev = idChunk;// 更新上次分配的 ID
                RTSpinlockRelease(pGMM->hSpinLockChunkId);
                return idChunk;
            }
        }
    }

    /*
     * Ok, scan from the start.
     * We're not racing anyone, so there is no need to expect failures or have restart loops.
     */
    //全局扫描（从头开始）
    /*
      若局部扫描仍失败，调用 ASMBitFirstClear 从位图起始位置查找。
      更新 idChunkPrev 以优化下一次分配。
    */
    idChunk = ASMBitFirstClear(&pGMM->bmChunkId[0], GMM_CHUNKID_LAST + 1);
    AssertMsgReturnStmt(idChunk > NIL_GMM_CHUNKID && (uint32_t)idChunk <= GMM_CHUNKID_LAST, ("%#x\n", idChunk),
                        RTSpinlockRelease(pGMM->hSpinLockChunkId), NIL_GVM_HANDLE);
    AssertMsgReturnStmt(!ASMAtomicBitTestAndSet(&pGMM->bmChunkId[0], idChunk), ("%#x\n", idChunk),
                        RTSpinlockRelease(pGMM->hSpinLockChunkId), NIL_GMM_CHUNKID);

    pGMM->idChunkPrev = idChunk;
    RTSpinlockRelease(pGMM->hSpinLockChunkId);
    return idChunk;
}


/**
 * Allocates one private page.
 *
 * Worker for gmmR0AllocatePages.
 *
 * @param   pChunk      The chunk to allocate it from.
 * @param   hGVM        The GVM handle of the VM requesting memory.
 * @param   pPageDesc   The page descriptor.
 */
//从指定内存块（pChunk）中分配 一个空闲物理页，
//绑定到目标虚拟机（hGVM），并填充页描述符（pPageDesc）。
/*
  更新内存块的空闲页统计信息。
  从空闲链表中摘取一个页。
  初始化页状态为私有（GMM_PAGE_STATE_PRIVATE）。
  填充输出描述符（物理地址、页ID等）。
*/
static void gmmR0AllocatePage(PGMMCHUNK pChunk, uint32_t hGVM, PGMMPAGEDESC pPageDesc)
{
    /* update the chunk stats. */
    if (pChunk->hGVM == NIL_GVM_HANDLE)
        pChunk->hGVM = hGVM;// 若内存块未绑定虚拟机，则绑定当前请求的VM
    Assert(pChunk->cFree);// 确保有空闲页
    pChunk->cFree--; // 空闲页计数减1
    pChunk->cPrivate++;  // 私有页计数加1

    /* unlink the first free page. */
    const uint32_t iPage = pChunk->iFreeHead;   // 获取空闲链表头索引
    AssertReleaseMsg(iPage < RT_ELEMENTS(pChunk->aPages), ("%d\n", iPage));
    PGMMPAGE pPage = &pChunk->aPages[iPage];    // 获取页指针
    Assert(GMM_PAGE_IS_FREE(pPage));            // 确保页状态为“空闲”
    pChunk->iFreeHead = pPage->Free.iNext;      // 更新链表头为下一页,通过 pPage->Free.iNext 获取下一个空闲页索引
    Log3(("A pPage=%p iPage=%#x/%#x u2State=%d iFreeHead=%#x iNext=%#x\n",
          pPage, iPage, (pChunk->Core.Key << GMM_CHUNKID_SHIFT) | iPage,
          pPage->Common.u2State, pChunk->iFreeHead, pPage->Free.iNext));

    bool const fZeroed = pPage->Free.fZeroed;// 记录页是否已清零

    /* make the page private. */
    pPage->u = 0; // 重置页结构体
    AssertCompile(GMM_PAGE_STATE_PRIVATE == 0);
    pPage->Private.hGVM = hGVM;// 设置归属虚拟机
    AssertCompile(NIL_RTHCPHYS >= GMM_GCPHYS_LAST);
    AssertCompile(GMM_GCPHYS_UNSHAREABLE >= GMM_GCPHYS_LAST);
    //若描述符中指定了物理地址（HCPhysGCPhys），将其转换为页帧号（pfn）。
    if (pPageDesc->HCPhysGCPhys <= GMM_GCPHYS_LAST)
        pPage->Private.pfn = pPageDesc->HCPhysGCPhys >> GUEST_PAGE_SHIFT;// 设置物理页帧号
    else
        //否则标记为不可共享（UNSHAREABLE）。
        pPage->Private.pfn = GMM_PAGE_PFN_UNSHAREABLE; /* unshareable / unassigned - same thing. */ // 标记为不可共享

    /* update the page descriptor. */
    pPageDesc->idSharedPage = NIL_GMM_PAGEID;// 非共享页
    pPageDesc->idPage       = (pChunk->Core.Key << GMM_CHUNKID_SHIFT) | iPage;// 全局唯一页ID
    RTHCPHYS const HCPhys = RTR0MemObjGetPagePhysAddr(pChunk->hMemObj, iPage);// 获取物理地址
    Assert(HCPhys != NIL_RTHCPHYS); Assert(HCPhys < NIL_GMMPAGEDESC_PHYS);
    pPageDesc->HCPhysGCPhys = HCPhys;// 写入描述符
    pPageDesc->fZeroed      = fZeroed;// 记录清零状态
}


/**
 * Picks the free pages from a chunk.
 *
 * @returns The new page descriptor table index.
 * @param   pChunk      The chunk.
 * @param   hGVM        The affinity of the chunk. NIL_GVM_HANDLE for no
 *                      affinity.
 * @param   iPage       The current page descriptor table index.
 * @param   cPages      The total number of pages to allocate.
 * @param   paPages     The page descriptor table (input + ouput).
 */
//从指定内存块（Chunk）分配连续物理页的核心逻辑，
//主要职责是从已注册的 pChunk 中分配指定数量的页（cPages）并填充页描述符（paPages）
static uint32_t gmmR0AllocatePagesFromChunk(PGMMCHUNK pChunk, uint16_t const hGVM, uint32_t iPage, uint32_t cPages,
                                            PGMMPAGEDESC paPages)
{
    PGMMCHUNKFREESET pSet = pChunk->pSet; Assert(pSet);
    gmmR0UnlinkChunk(pChunk);

    /*
       遍历空闲页链表（pChunk->aPages），调用 gmmR0AllocatePage 填充 paPages
       更新 pChunk->cFree（剩余空闲页数）和 pChunk->iFreeHead（空闲链表头
    */
    for (; pChunk->cFree && iPage < cPages; iPage++)
        gmmR0AllocatePage(pChunk, hGVM, &paPages[iPage]);

    //若仍有剩余空闲页，将 pChunk 重新插入空闲集合（pSet）
    gmmR0LinkChunk(pChunk, pSet);
    return iPage;
}


/**
 * Registers a new chunk of memory.
 *
 * This is called by gmmR0AllocateOneChunk and GMMR0AllocateLargePage.
 *
 * In the  GMMR0AllocateLargePage case the GMM_CHUNK_FLAGS_LARGE_PAGE flag is
 * set and the chunk will be registered as fully allocated to save time.
 *
 * @returns VBox status code.  On success, the giant GMM lock will be held, the
 *          caller must release it (ugly).
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pSet        Pointer to the set.
 * @param   hMemObj     The memory object for the chunk.
 * @param   hGVM        The affinity of the chunk. NIL_GVM_HANDLE for no
 *                      affinity.
 * @param   pSession    Same as @a hGVM.
 * @param   fChunkFlags The chunk flags, GMM_CHUNK_FLAGS_XXX.
 * @param   cPages      The number of pages requested.  Zero for large pages.
 * @param   paPages     The page descriptor table (input + output).  NULL for
 *                      large pages.
 * @param   piPage      The pointer to the page descriptor table index variable.
 *                      This will be updated.  NULL for large pages.
 * @param   ppChunk     Chunk address (out).
 *
 * @remarks The caller must not own the giant GMM mutex.
 *          The giant GMM mutex will be acquired and returned acquired in
 *          the success path.   On failure, no locks will be held.
 */
//负责 注册新分配的内存块（Chunk）到全局管理结构中，并关联到指定虚拟机（hGVM）。
static int gmmR0RegisterChunk(PGMM pGMM, PGMMCHUNKFREESET pSet, RTR0MEMOBJ hMemObj, uint16_t hGVM, PSUPDRVSESSION pSession,
                              uint16_t fChunkFlags, uint32_t cPages, PGMMPAGEDESC paPages, uint32_t *piPage, PGMMCHUNK *ppChunk)
{
    /*
     * Validate input & state.
     */
    Assert(pGMM->hMtxOwner != RTThreadNativeSelf()); // 确保未持有全局锁
    Assert(hGVM != NIL_GVM_HANDLE || pGMM->fBoundMemoryMode);// 共享块需显式启用fBoundMemoryMode
    Assert(fChunkFlags == 0 || fChunkFlags == GMM_CHUNK_FLAGS_LARGE_PAGE);// 标志合法性检查
    if (!(fChunkFlags &= GMM_CHUNK_FLAGS_LARGE_PAGE))
    {
        AssertPtr(paPages);
        AssertPtr(piPage);
        Assert(cPages > 0);
        Assert(cPages > *piPage);
    }
    else
    {
        Assert(cPages == 0);
        Assert(!paPages);
        Assert(!piPage);
    }

#ifndef VBOX_WITH_LINEAR_HOST_PHYS_MEM
    /*
     * Get a ring-0 mapping of the object.
     */
    //VBOX_WITH_LINEAR_HOST_PHYS_MEM 模式下跳过此步骤（直接使用物理地址）。
    uint8_t *pbMapping = (uint8_t *)RTR0MemObjAddress(hMemObj); // 获取内核虚拟地址
    if (!pbMapping)
    {
        RTR0MEMOBJ hMapObj;
        int rc = RTR0MemObjMapKernel(&hMapObj, hMemObj, (void *)-1, 0,  RTMEM_PROT_READ | RTMEM_PROT_WRITE);
        if (RT_SUCCESS(rc))
            pbMapping = (uint8_t *)RTR0MemObjAddress(hMapObj);
        else
            return rc;
        AssertPtr(pbMapping);
    }
#endif

    /*
     * Allocate a chunk and an ID for it.
     */
    int rc;
    PGMMCHUNK pChunk = (PGMMCHUNK)RTMemAllocZ(sizeof(*pChunk));
    if (pChunk)
    {
        pChunk->Core.Key = gmmR0AllocateChunkId(pGMM);// 分配唯一块 ID
        if (   pChunk->Core.Key != NIL_GMM_CHUNKID
            && pChunk->Core.Key <= GMM_CHUNKID_LAST)
        {
            /*
             * Initialize it.
             */
            pChunk->hMemObj     = hMemObj;// 绑定内存对象
#ifndef VBOX_WITH_LINEAR_HOST_PHYS_MEM
            pChunk->pbMapping   = pbMapping;
#endif
            pChunk->hGVM        = hGVM; // 关联虚拟机,通过 hGVM 区分私有块与共享块
            pChunk->idNumaNode  = gmmR0GetCurrentNumaNodeId();// 设置 NUMA 节点,NUMA 亲和性优化。
            pChunk->iChunkMtx   = UINT8_MAX;
            pChunk->fFlags      = fChunkFlags;// 设置标志（如大页）
            pChunk->uidOwner    = pSession ? SUPR0GetSessionUid(pSession) : NIL_RTUID;//会话 UID（权限控制）。会话权限：uidOwner 确保只有创建者会话可操作内存块。
            /*pChunk->cShared   = 0; */

            uint32_t const iDstPageFirst = piPage ? *piPage : cPages;
            if (!(fChunkFlags & GMM_CHUNK_FLAGS_LARGE_PAGE))//普通页模式处理
            {
                /*
                 * Allocate the requested number of pages from the start of the chunk,
                 * queue the rest (if any) on the free list.
                 */
                uint32_t const cPagesAlloc = RT_MIN(cPages - iDstPageFirst, GMM_CHUNK_NUM_PAGES);
                pChunk->cPrivate    = cPagesAlloc;// 已分配页数
                pChunk->cFree       = GMM_CHUNK_NUM_PAGES - cPagesAlloc;// 剩余空闲页数
                pChunk->iFreeHead   = GMM_CHUNK_NUM_PAGES > cPagesAlloc ? cPagesAlloc : UINT16_MAX;  // 空闲链表头索引,通过链表管理（iFreeHead 指向首个空闲页）。

                /* Alloc pages: */
                uint32_t const idPageChunk = pChunk->Core.Key << GMM_CHUNKID_SHIFT;
                uint32_t       iDstPage    = iDstPageFirst;
                uint32_t       iPage;
                // 填充页描述符（paPages）
                for (iPage = 0; iPage < cPagesAlloc; iPage++, iDstPage++)
                {
                    if (paPages[iDstPage].HCPhysGCPhys <= GMM_GCPHYS_LAST)
                        pChunk->aPages[iPage].Private.pfn = paPages[iDstPage].HCPhysGCPhys >> GUEST_PAGE_SHIFT;
                    else
                        pChunk->aPages[iPage].Private.pfn = GMM_PAGE_PFN_UNSHAREABLE; /* unshareable / unassigned - same thing. */
                    pChunk->aPages[iPage].Private.hGVM    = hGVM;
                    pChunk->aPages[iPage].Private.u2State = GMM_PAGE_STATE_PRIVATE;

                    paPages[iDstPage].HCPhysGCPhys = RTR0MemObjGetPagePhysAddr(hMemObj, iPage);
                    paPages[iDstPage].fZeroed      = true;
                    paPages[iDstPage].idPage       = idPageChunk | iPage; // 全局页 ID
                    paPages[iDstPage].idSharedPage = NIL_GMM_PAGEID;
                }
                *piPage = iDstPage;

                /* Build free list: */
                if (iPage < RT_ELEMENTS(pChunk->aPages))
                {
                    Assert(pChunk->iFreeHead == iPage);
                    for (; iPage < RT_ELEMENTS(pChunk->aPages) - 1; iPage++)
                    {
                        pChunk->aPages[iPage].Free.u2State = GMM_PAGE_STATE_FREE;
                        pChunk->aPages[iPage].Free.fZeroed = true;
                        pChunk->aPages[iPage].Free.iNext   = iPage + 1; // 链式连接空闲页
                    }
                    pChunk->aPages[RT_ELEMENTS(pChunk->aPages) - 1].Free.u2State = GMM_PAGE_STATE_FREE;
                    pChunk->aPages[RT_ELEMENTS(pChunk->aPages) - 1].Free.fZeroed = true;
                    pChunk->aPages[RT_ELEMENTS(pChunk->aPages) - 1].Free.iNext   = UINT16_MAX;
                }
                else
                    Assert(pChunk->iFreeHead == UINT16_MAX);
            }
            else//大页模式处理
            //大页模式下不拆分块，直接整块管理（无空闲链表）。
            {
                /*
                 * Large page: Mark all pages as privately allocated (watered down gmmR0AllocatePage).
                 */
                pChunk->cFree       = 0;
                pChunk->cPrivate    = GMM_CHUNK_NUM_PAGES;
                pChunk->iFreeHead   = UINT16_MAX;

                for (unsigned iPage = 0; iPage < RT_ELEMENTS(pChunk->aPages); iPage++)
                {
                    pChunk->aPages[iPage].Private.pfn     = GMM_PAGE_PFN_UNSHAREABLE;
                    pChunk->aPages[iPage].Private.hGVM    = hGVM;
                    pChunk->aPages[iPage].Private.u2State = GMM_PAGE_STATE_PRIVATE;
                }
            }

            /*
             * Zero the memory if it wasn't zeroed by the host already.
             * This simplifies keeping secret kernel bits from userland and brings
             * everyone to the same level wrt allocation zeroing.
             */
            rc = VINF_SUCCESS;
            if (!RTR0MemObjWasZeroInitialized(hMemObj))
            {
#ifdef VBOX_WITH_LINEAR_HOST_PHYS_MEM
                if (!(fChunkFlags & GMM_CHUNK_FLAGS_LARGE_PAGE))
                {
                    for (uint32_t iPage = 0; iPage < GMM_CHUNK_SIZE / HOST_PAGE_SIZE; iPage++)
                    {
                        void *pvPage = NULL;
                        rc = SUPR0HCPhysToVirt(RTR0MemObjGetPagePhysAddr(hMemObj, iPage), &pvPage);
                        AssertRCBreak(rc);
                        RT_BZERO(pvPage, HOST_PAGE_SIZE);
                    }
                }
                else
                {
                    /* Can do the whole large page in one go. */
                    void *pvPage = NULL;
                    rc = SUPR0HCPhysToVirt(RTR0MemObjGetPagePhysAddr(hMemObj, 0), &pvPage);
                    AssertRC(rc);
                    if (RT_SUCCESS(rc))
                        RT_BZERO(pvPage, GMM_CHUNK_SIZE);
                }
#else
                RT_BZERO(pbMapping, GMM_CHUNK_SIZE);
#endif
            }
            if (RT_SUCCESS(rc))
            {
                *ppChunk = pChunk;

                /*
                 * Allocate a Chunk ID and insert it into the tree.
                 * This has to be done behind the mutex of course.
                 */
                rc = gmmR0MutexAcquire(pGMM);
                if (RT_SUCCESS(rc))
                {
                    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
                    {
                        RTSpinlockAcquire(pGMM->hSpinLockTree);
                        if (RTAvlU32Insert(&pGMM->pChunks, &pChunk->Core))
                        {
                            pGMM->cChunks++;
                            RTListAppend(&pGMM->ChunkList, &pChunk->ListNode);
                            RTSpinlockRelease(pGMM->hSpinLockTree);

                            gmmR0LinkChunk(pChunk, pSet);

                            LogFlow(("gmmR0RegisterChunk: pChunk=%p id=%#x cChunks=%d\n", pChunk, pChunk->Core.Key, pGMM->cChunks));
                            GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
                            return VINF_SUCCESS;
                        }

                        /*
                         * Bail out.
                         */
                        RTSpinlockRelease(pGMM->hSpinLockTree);
                        rc = VERR_GMM_CHUNK_INSERT;
                    }
                    else
                        rc = VERR_GMM_IS_NOT_SANE;
                    gmmR0MutexRelease(pGMM);
                }
                *ppChunk = NULL;
            }

            /* Undo any page allocations. */
            if (!(fChunkFlags & GMM_CHUNK_FLAGS_LARGE_PAGE))
            {
                uint32_t const cToFree = pChunk->cPrivate;
                Assert(*piPage - iDstPageFirst == cToFree);
                for (uint32_t iDstPage = iDstPageFirst, iPage = 0; iPage < cToFree; iPage++, iDstPage++)
                {
                    paPages[iDstPageFirst].fZeroed = false;
                    if (pChunk->aPages[iPage].Private.pfn == GMM_PAGE_PFN_UNSHAREABLE)
                        paPages[iDstPageFirst].HCPhysGCPhys = NIL_GMMPAGEDESC_PHYS;
                    else
                        paPages[iDstPageFirst].HCPhysGCPhys = (RTHCPHYS)pChunk->aPages[iPage].Private.pfn << GUEST_PAGE_SHIFT;
                    paPages[iDstPageFirst].idPage       = NIL_GMM_PAGEID;
                    paPages[iDstPageFirst].idSharedPage = NIL_GMM_PAGEID;
                }
                *piPage = iDstPageFirst;
            }

            gmmR0FreeChunkId(pGMM, pChunk->Core.Key);
        }
        else
            rc = VERR_GMM_CHUNK_INSERT;
        RTMemFree(pChunk);
    }
    else
        rc = VERR_NO_MEMORY;
    return rc;
}


/**
 * Allocate a new chunk, immediately pick the requested pages from it, and adds
 * what's remaining to the specified free set.
 *
 * @note    This will leave the giant mutex while allocating the new chunk!
 *
 * @returns VBox status code.
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the kernel-only VM instace data.
 * @param   pSet        Pointer to the free set.
 * @param   cPages      The number of pages requested.
 * @param   paPages     The page descriptor table (input + output).
 * @param   piPage      The pointer to the page descriptor table index variable.
 *                      This will be updated.
 */
static int gmmR0AllocateChunkNew(PGMM pGMM, PGVM pGVM, PGMMCHUNKFREESET pSet, uint32_t cPages,
                                 PGMMPAGEDESC paPages, uint32_t *piPage)
{
    gmmR0MutexRelease(pGMM);

    RTR0MEMOBJ hMemObj;
    int rc;
#ifdef VBOX_WITH_LINEAR_HOST_PHYS_MEM
    if (pGMM->fHasWorkingAllocPhysNC)
        rc = RTR0MemObjAllocPhysNC(&hMemObj, GMM_CHUNK_SIZE, NIL_RTHCPHYS);
    else
#endif
        rc = RTR0MemObjAllocPage(&hMemObj, GMM_CHUNK_SIZE, false /*fExecutable*/);
    if (RT_SUCCESS(rc))
    {
        PGMMCHUNK pIgnored;
        rc = gmmR0RegisterChunk(pGMM, pSet, hMemObj, pGVM->hSelf, pGVM->pSession, 0 /*fChunkFlags*/,
                                cPages, paPages, piPage, &pIgnored);
        if (RT_SUCCESS(rc))
            return VINF_SUCCESS;

        /* bail out */
        RTR0MemObjFree(hMemObj, true /* fFreeMappings */);
    }

    int rc2 = gmmR0MutexAcquire(pGMM);
    AssertRCReturn(rc2, RT_FAILURE(rc) ? rc : rc2);
    return rc;

}


/**
 * As a last restort we'll pick any page we can get.
 *
 * @returns The new page descriptor table index.
 * @param   pSet        The set to pick from.
 * @param   pGVM        Pointer to the global VM structure.
 * @param   uidSelf     The UID of the caller.
 * @param   iPage       The current page descriptor table index.
 * @param   cPages      The total number of pages to allocate.
 * @param   paPages     The page descriptor table (input + ouput).
 */
/*
 无差别分配物理页的核心逻辑，主要目标是从内存池中快速分配连续物理页
 ，优先复用空闲块或未映射的共享块，适用于 紧急内存需求或 低优先级分配场景

适用场景：
  虚拟机无法从本地 NUMA 节点或关联块分配足够内存（如 gmmR0AllocatePagesFromSameNode 失败后回退）
  需要快速分配大块连续内存（如 DMA 缓冲区或临时映射）
*/
//在内存紧张或碎片化严重时，绕过 NUMA 亲和性和虚拟机关联性检查，直接从全局内存池分配物理页
static uint32_t gmmR0AllocatePagesIndiscriminately(PGMMCHUNKFREESET pSet, PGVM pGVM, RTUID uidSelf,
                                                   uint32_t iPage, uint32_t cPages, PGMMPAGEDESC paPages)
{
    unsigned iList = RT_ELEMENTS(pSet->apLists);
    while (iList-- > 0)
    {
        PGMMCHUNK pChunk = pSet->apLists[iList];
        while (pChunk)
        {
            PGMMCHUNK pNext = pChunk->pFreeNext;
            if (   pChunk->uidOwner == uidSelf // 条件1：属于当前用户
                || (   pChunk->cMappingsX == 0 // 条件2：未被映射的共享块
                    && pChunk->cFree == (GMM_CHUNK_SIZE >> GUEST_PAGE_SHIFT)))// 且完全空闲
            {
                iPage = gmmR0AllocatePagesFromChunk(pChunk, pGVM->hSelf, iPage, cPages, paPages);
                if (iPage >= cPages)
                    return iPage;
            }

            pChunk = pNext;
        }
    }
    return iPage;
}


/**
 * Pick pages from empty chunks on the same NUMA node.
 *
 * @returns The new page descriptor table index.
 * @param   pSet        The set to pick from.
 * @param   pGVM        Pointer to the global VM structure.
 * @param   uidSelf     The UID of the caller.
 * @param   iPage       The current page descriptor table index.
 * @param   cPages      The total number of pages to allocate.
 * @param   paPages     The page descriptor table (input + ouput).
 */
static uint32_t gmmR0AllocatePagesFromEmptyChunksOnSameNode(PGMMCHUNKFREESET pSet, PGVM pGVM, RTUID uidSelf,
                                                            uint32_t iPage, uint32_t cPages, PGMMPAGEDESC paPages)
{
    PGMMCHUNK pChunk = pSet->apLists[GMM_CHUNK_FREE_SET_UNUSED_LIST];
    if (pChunk)
    {
        uint16_t const idNumaNode = gmmR0GetCurrentNumaNodeId();
        while (pChunk)
        {
            PGMMCHUNK pNext = pChunk->pFreeNext;

            if (   pChunk->idNumaNode == idNumaNode
                && (   pChunk->uidOwner == uidSelf
                    || pChunk->cMappingsX == 0))
            {
                pChunk->hGVM     = pGVM->hSelf;
                pChunk->uidOwner = uidSelf;
                iPage = gmmR0AllocatePagesFromChunk(pChunk, pGVM->hSelf, iPage, cPages, paPages);
                if (iPage >= cPages)
                {
                    pGVM->gmm.s.idLastChunkHint = pChunk->cFree ? pChunk->Core.Key : NIL_GMM_CHUNKID;
                    return iPage;
                }
            }

            pChunk = pNext;
        }
    }
    return iPage;
}


/**
 * Pick pages from non-empty chunks on the same NUMA node.
 *
 * @returns The new page descriptor table index.
 * @param   pSet        The set to pick from.
 * @param   pGVM        Pointer to the global VM structure.
 * @param   uidSelf     The UID of the caller.
 * @param   iPage       The current page descriptor table index.
 * @param   cPages      The total number of pages to allocate.
 * @param   paPages     The page descriptor table (input + ouput).
 */
//用于从 同一 NUMA 节点且 同一用户（uidSelf）的内存块（Chunk）中分配物理页
/*
      PGMMCHUNKFREESET pSet,   // 内存块集合（如 PrivateX 或 Shared 池）
      PGVM pGVM,               // 目标虚拟机
      RTUID const uidSelf,     // 当前用户 ID（如 QEMU 进程的 UID）
      uint32_t iPage,          // 当前已分配的页数（初始为0）
      uint32_t cPages,         // 需分配的总页数
      PGMMPAGEDESC paPages     // 页描述符数组（存储分配结果）*/
static uint32_t gmmR0AllocatePagesFromSameNode(PGMMCHUNKFREESET pSet, PGVM pGVM, RTUID const uidSelf,
                                               uint32_t iPage, uint32_t cPages, PGMMPAGEDESC paPages)
{
    /** @todo start by picking from chunks with about the right size first?  */
    uint16_t const  idNumaNode = gmmR0GetCurrentNumaNodeId();// 获取当前 CPU 的 NUMA 节点 ID
    unsigned        iList      = GMM_CHUNK_FREE_SET_UNUSED_LIST;
    while (iList-- > 0)
    {
        PGMMCHUNK pChunk = pSet->apLists[iList];
        while (pChunk)
        {
            PGMMCHUNK pNext = pChunk->pFreeNext;

            //内存块必须位于当前 CPU 的 NUMA 节点
            if (   pChunk->idNumaNode == idNumaNode
                    //内存块必须属于同一用户（避免跨进程安全风险
                && pChunk->uidOwner   == uidSelf)
            {
                iPage = gmmR0AllocatePagesFromChunk(pChunk, pGVM->hSelf, iPage, cPages, paPages);
                if (iPage >= cPages)
                {
                    pGVM->gmm.s.idLastChunkHint = pChunk->cFree ? pChunk->Core.Key : NIL_GMM_CHUNKID;
                    return iPage;// 分配成功
                }
            }

            pChunk = pNext;
        }
    }
    return iPage;
}


/**
 * Pick pages that are in chunks already associated with the VM.
 *
 * @returns The new page descriptor table index.
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the global VM structure.
 * @param   pSet        The set to pick from.
 * @param   iPage       The current page descriptor table index.
 * @param   cPages      The total number of pages to allocate.
 * @param   paPages     The page descriptor table (input + ouput).
 */
//从与当前虚拟机（VM）关联的内存块（Chunk）中分配物理页
static uint32_t gmmR0AllocatePagesAssociatedWithVM(PGMM pGMM, PGVM pGVM, PGMMCHUNKFREESET pSet,
                                                   uint32_t iPage, uint32_t cPages, PGMMPAGEDESC paPages)
{
    uint16_t const hGVM = pGVM->hSelf;

    /* Hint. */
    //优化策略：利用 idLastChunkHint 记录上次成功分配的块 ID，优先尝试从该块分配
    //性能意义：减少链表遍历开销，提升局部性
    if (pGVM->gmm.s.idLastChunkHint != NIL_GMM_CHUNKID)
    {
        PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, pGVM->gmm.s.idLastChunkHint);
        if (pChunk && pChunk->cFree)
        {
            iPage = gmmR0AllocatePagesFromChunk(pChunk, hGVM, iPage, cPages, paPages);
            if (iPage >= cPages)
                return iPage;
        }
    }

    /* Scan. */
    for (unsigned iList = 0; iList < RT_ELEMENTS(pSet->apLists); iList++)
    {
        PGMMCHUNK pChunk = pSet->apLists[iList];
        while (pChunk)
        {
            PGMMCHUNK pNext = pChunk->pFreeNext;

            //通过 hGVM 匹配，优先复用当前虚拟机曾使用的内存块，减少跨虚拟机共享带来的 NUMA 延迟
            if (pChunk->hGVM == hGVM)// 检查是否属于当前VM
            {
                iPage = gmmR0AllocatePagesFromChunk(pChunk, hGVM, iPage, cPages, paPages);
                if (iPage >= cPages)
                {
                    pGVM->gmm.s.idLastChunkHint = pChunk->cFree ? pChunk->Core.Key : NIL_GMM_CHUNKID;
                    return iPage;
                }
            }

            pChunk = pNext;
        }
    }
    return iPage;
}



/**
 * Pick pages in bound memory mode.
 *
 * @returns The new page descriptor table index.
 * @param   pGVM        Pointer to the global VM structure.
 * @param   iPage       The current page descriptor table index.
 * @param   cPages      The total number of pages to allocate.
 * @param   paPages     The page descriptor table (input + ouput).
 */
static uint32_t gmmR0AllocatePagesInBoundMode(PGVM pGVM, uint32_t iPage, uint32_t cPages, PGMMPAGEDESC paPages)
{
    for (unsigned iList = 0; iList < RT_ELEMENTS(pGVM->gmm.s.Private.apLists); iList++)
    {
        PGMMCHUNK pChunk = pGVM->gmm.s.Private.apLists[iList];
        while (pChunk)
        {
            Assert(pChunk->hGVM == pGVM->hSelf);
            PGMMCHUNK pNext = pChunk->pFreeNext;
            iPage = gmmR0AllocatePagesFromChunk(pChunk, pGVM->hSelf, iPage, cPages, paPages);
            if (iPage >= cPages)
                return iPage;
            pChunk = pNext;
        }
    }
    return iPage;
}


/**
 * Checks if we should start picking pages from chunks of other VMs because
 * we're getting close to the system memory or reserved limit.
 *
 * @returns @c true if we should, @c false if we should first try allocate more
 *          chunks.
 */
static bool gmmR0ShouldAllocatePagesInOtherChunksBecauseOfLimits(PGVM pGVM)
{
    /*
     * Don't allocate a new chunk if we're
     */
    uint64_t cPgReserved  = pGVM->gmm.s.Stats.Reserved.cBasePages
                          + pGVM->gmm.s.Stats.Reserved.cFixedPages
                          - pGVM->gmm.s.Stats.cBalloonedPages
                          /** @todo what about shared pages? */;
    uint64_t cPgAllocated = pGVM->gmm.s.Stats.Allocated.cBasePages
                          + pGVM->gmm.s.Stats.Allocated.cFixedPages;
    uint64_t cPgDelta = cPgReserved - cPgAllocated;
    if (cPgDelta < GMM_CHUNK_NUM_PAGES * 4)
        return true;
    /** @todo make the threshold configurable, also test the code to see if
     *        this ever kicks in (we might be reserving too much or smth). */

    /*
     * Check how close we're to the max memory limit and how many fragments
     * there are?...
     */
    /** @todo  */

    return false;
}


/**
 * Checks if we should start picking pages from chunks of other VMs because
 * there is a lot of free pages around.
 *
 * @returns @c true if we should, @c false if we should first try allocate more
 *          chunks.
 */
static bool gmmR0ShouldAllocatePagesInOtherChunksBecauseOfLotsFree(PGMM pGMM)
{
    /*
     * Setting the limit at 16 chunks (32 MB) at the moment.
     */
    if (pGMM->PrivateX.cFreePages >= GMM_CHUNK_NUM_PAGES * 16)
        return true;
    return false;
}


/**
 * Common worker for GMMR0AllocateHandyPages and GMMR0AllocatePages.
 *
 * @returns VBox status code:
 * @retval  VINF_SUCCESS on success.
 * @retval  VERR_GMM_HIT_GLOBAL_LIMIT if we've exhausted the available pages.
 * @retval  VERR_GMM_HIT_VM_ACCOUNT_LIMIT if we've hit the VM account limit,
 *          that is we're trying to allocate more than we've reserved.
 *
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the VM.
 * @param   cPages      The number of pages to allocate.
 * @param   paPages     Pointer to the page descriptors. See GMMPAGEDESC for
 *                      details on what is expected on input.
 * @param   enmAccount  The account to charge.
 *
 * @remarks Caller owns the giant GMM lock.
 */
/*
  多账户类型：支持三种内存账户（基础内存、影子页表内存、固定内存）。
  两级限制检查：先检查全局内存上限，再校验VM专属的账户配额。
  两种分配模式：绑定模式（NUMA亲和）和共享模式（跨VM复用内存块）。
*/
static int gmmR0AllocatePagesNew(PGMM pGMM, PGVM pGVM, uint32_t cPages, PGMMPAGEDESC paPages, GMMACCOUNT enmAccount)
{
    Assert(pGMM->hMtxOwner == RTThreadNativeSelf());

    /*
     * Check allocation limits.
     */
    if (RT_LIKELY(pGMM->cAllocatedPages + cPages <= pGMM->cMaxPages))
    { /* likely */ }
    else
        return VERR_GMM_HIT_GLOBAL_LIMIT; // 全局内存不足

    switch (enmAccount)
    {
        case GMMACCOUNT_BASE:
            if (RT_LIKELY(   pGVM->gmm.s.Stats.Allocated.cBasePages + pGVM->gmm.s.Stats.cBalloonedPages + cPages
                          <= pGVM->gmm.s.Stats.Reserved.cBasePages))
            { /* likely */ }
            else
            {
                Log(("gmmR0AllocatePages:Base: Reserved=%#llx Allocated+Ballooned+Requested=%#llx+%#llx+%#x!\n",
                     pGVM->gmm.s.Stats.Reserved.cBasePages, pGVM->gmm.s.Stats.Allocated.cBasePages,
                     pGVM->gmm.s.Stats.cBalloonedPages, cPages));
                return VERR_GMM_HIT_VM_ACCOUNT_LIMIT;// VM的基础内存配额不足
            }
            break;
        case GMMACCOUNT_SHADOW:
            if (RT_LIKELY(pGVM->gmm.s.Stats.Allocated.cShadowPages + cPages <= pGVM->gmm.s.Stats.Reserved.cShadowPages))
            { /* likely */ }
            else
            {
                Log(("gmmR0AllocatePages:Shadow: Reserved=%#x Allocated+Requested=%#x+%#x!\n",
                     pGVM->gmm.s.Stats.Reserved.cShadowPages, pGVM->gmm.s.Stats.Allocated.cShadowPages, cPages));
                return VERR_GMM_HIT_VM_ACCOUNT_LIMIT;
            }
            break;
        case GMMACCOUNT_FIXED:
            if (RT_LIKELY(pGVM->gmm.s.Stats.Allocated.cFixedPages + cPages <= pGVM->gmm.s.Stats.Reserved.cFixedPages))
            { /* likely */ }
            else
            {
                Log(("gmmR0AllocatePages:Fixed: Reserved=%#x Allocated+Requested=%#x+%#x!\n",
                     pGVM->gmm.s.Stats.Reserved.cFixedPages, pGVM->gmm.s.Stats.Allocated.cFixedPages, cPages));
                return VERR_GMM_HIT_VM_ACCOUNT_LIMIT;
            }
            break;
        default:
            AssertMsgFailedReturn(("enmAccount=%d\n", enmAccount), VERR_IPE_NOT_REACHED_DEFAULT_CASE);
    }

    /*
     * Update the accounts before we proceed because we might be leaving the
     * protection of the global mutex and thus run the risk of permitting
     * too much memory to be allocated.
     */
    switch (enmAccount)
    {
        case GMMACCOUNT_BASE:   pGVM->gmm.s.Stats.Allocated.cBasePages   += cPages; break;
        case GMMACCOUNT_SHADOW: pGVM->gmm.s.Stats.Allocated.cShadowPages += cPages; break;
        case GMMACCOUNT_FIXED:  pGVM->gmm.s.Stats.Allocated.cFixedPages  += cPages; break;
        default:                AssertMsgFailedReturn(("enmAccount=%d\n", enmAccount), VERR_IPE_NOT_REACHED_DEFAULT_CASE);
    }
    pGVM->gmm.s.Stats.cPrivatePages += cPages;
    pGMM->cAllocatedPages           += cPages;

    /*
     * Bound mode is also relatively straightforward.
     */
    uint32_t iPage = 0;
    int rc = VINF_SUCCESS;
    //绑定模式（fBoundMemoryMode = true）
    //直接分配：优先从VM关联的NUMA节点获取内存。
    //后备机制：若不足，调用gmmR0AllocateChunkNew申请新内存块。
    if (pGMM->fBoundMemoryMode)
    {
        iPage = gmmR0AllocatePagesInBoundMode(pGVM, iPage, cPages, paPages);
        if (iPage < cPages)
            do
                rc = gmmR0AllocateChunkNew(pGMM, pGVM, &pGVM->gmm.s.Private, cPages, paPages, &iPage);
            while (iPage < cPages && RT_SUCCESS(rc));
    }
    /*
     * Shared mode is trickier as we should try archive the same locality as
     * in bound mode, but smartly make use of non-full chunks allocated by
     * other VMs if we're low on memory.
     */
    //共享模式（默认）
    /*
       最优路径：从当前VM的私有内存池分配（gmmR0AllocatePagesAssociatedWithVM）。
       同级借用：从同一NUMA节点的其他VM内存池借用（gmmR0AllocatePagesFromSameNode）。
       空块复用：查找同节点的空闲内存块（gmmR0AllocatePagesFromEmptyChunksOnSameNode）。
       全局共享：尝试分配共享内存池的空闲块。
       最终手段：申请全新的内存块（gmmR0AllocateChunkNew）。
    */
    else
    {
        RTUID const uidSelf = SUPR0GetSessionUid(pGVM->pSession);

        /* Pick the most optimal pages first. */
        // 策略1：优先从当前虚拟机的关联内存块分配
        iPage = gmmR0AllocatePagesAssociatedWithVM(pGMM, pGVM, &pGMM->PrivateX, iPage, cPages, paPages);
        if (iPage < cPages)
        {
            /* Maybe we should try getting pages from chunks "belonging" to
               other VMs before allocating more chunks? */
            bool fTriedOnSameAlready = false;
            if (gmmR0ShouldAllocatePagesInOtherChunksBecauseOfLimits(pGVM))
            {
                // 策略2：从同NUMA节点的其他虚拟机块分配
                iPage = gmmR0AllocatePagesFromSameNode(&pGMM->PrivateX, pGVM, uidSelf, iPage, cPages, paPages);
                fTriedOnSameAlready = true;
            }

            /* Allocate memory from empty chunks. */
            // 策略3：从同节点的空闲块分配
            if (iPage < cPages)
                iPage = gmmR0AllocatePagesFromEmptyChunksOnSameNode(&pGMM->PrivateX, pGVM, uidSelf, iPage, cPages, paPages);

            /* Grab empty shared chunks. */
            if (iPage < cPages)
                iPage = gmmR0AllocatePagesFromEmptyChunksOnSameNode(&pGMM->Shared, pGVM, uidSelf, iPage, cPages, paPages);

            /* If there is a lof of free pages spread around, try not waste
               system memory on more chunks. (Should trigger defragmentation.) */
            if (   !fTriedOnSameAlready
                && gmmR0ShouldAllocatePagesInOtherChunksBecauseOfLotsFree(pGMM))
            {
                 // 策略4：从全局共享池分配
                iPage = gmmR0AllocatePagesFromSameNode(&pGMM->PrivateX, pGVM, uidSelf, iPage, cPages, paPages);
                if (iPage < cPages)
                    iPage = gmmR0AllocatePagesIndiscriminately(&pGMM->PrivateX, pGVM, uidSelf, iPage, cPages, paPages);
            }

            /*
             * Ok, try allocate new chunks.
             */
            if (iPage < cPages)
            {
                do
                    rc = gmmR0AllocateChunkNew(pGMM, pGVM, &pGMM->PrivateX, cPages, paPages, &iPage);
                while (iPage < cPages && RT_SUCCESS(rc));

#if 0 /* We cannot mix chunks with different UIDs. */
                /* If the host is out of memory, take whatever we can get. */
                if (   (rc == VERR_NO_MEMORY || rc == VERR_NO_PHYS_MEMORY)
                    && pGMM->PrivateX.cFreePages + pGMM->Shared.cFreePages >= cPages - iPage)
                {
                    iPage = gmmR0AllocatePagesIndiscriminately(&pGMM->PrivateX, pGVM, iPage, cPages, paPages);
                    if (iPage < cPages)
                        iPage = gmmR0AllocatePagesIndiscriminately(&pGMM->Shared, pGVM, iPage, cPages, paPages);
                    AssertRelease(iPage == cPages);
                    rc = VINF_SUCCESS;
                }
#endif
            }
        }
    }

    /*
     * Clean up on failure.  Since this is bound to be a low-memory condition
     * we will give back any empty chunks that might be hanging around.
     */
    if (RT_SUCCESS(rc))
    { /* likely */ }
    else
    {
        /* Update the statistics. */
        pGVM->gmm.s.Stats.cPrivatePages -= cPages;
        pGMM->cAllocatedPages           -= cPages - iPage;
        switch (enmAccount)
        {
            case GMMACCOUNT_BASE:   pGVM->gmm.s.Stats.Allocated.cBasePages   -= cPages; break;
            case GMMACCOUNT_SHADOW: pGVM->gmm.s.Stats.Allocated.cShadowPages -= cPages; break;
            case GMMACCOUNT_FIXED:  pGVM->gmm.s.Stats.Allocated.cFixedPages  -= cPages; break;
            default:                AssertMsgFailedReturn(("enmAccount=%d\n", enmAccount), VERR_IPE_NOT_REACHED_DEFAULT_CASE);
        }

        /* Release the pages. */
        while (iPage-- > 0)
        {
            uint32_t idPage = paPages[iPage].idPage;
            PGMMPAGE pPage = gmmR0GetPage(pGMM, idPage);
            if (RT_LIKELY(pPage))
            {
                Assert(GMM_PAGE_IS_PRIVATE(pPage));
                Assert(pPage->Private.hGVM == pGVM->hSelf);
                gmmR0FreePrivatePage(pGMM, pGVM, idPage, pPage);
            }
            else
                AssertMsgFailed(("idPage=%#x\n", idPage));

            paPages[iPage].idPage       = NIL_GMM_PAGEID;
            paPages[iPage].idSharedPage = NIL_GMM_PAGEID;
            paPages[iPage].HCPhysGCPhys = NIL_GMMPAGEDESC_PHYS;
            paPages[iPage].fZeroed      = false;
        }

        /* Free empty chunks. */
        /** @todo  */

        /* return the fail status on failure */
        return rc;
    }
    return VINF_SUCCESS;
}


/**
 * Updates the previous allocations and allocates more pages.
 *
 * The handy pages are always taken from the 'base' memory account.
 * The allocated pages are not cleared and will contains random garbage.
 *
 * @returns VBox status code:
 * @retval  VINF_SUCCESS on success.
 * @retval  VERR_NOT_OWNER if the caller is not an EMT.
 * @retval  VERR_GMM_PAGE_NOT_FOUND if one of the pages to update wasn't found.
 * @retval  VERR_GMM_PAGE_NOT_PRIVATE if one of the pages to update wasn't a
 *          private page.
 * @retval  VERR_GMM_PAGE_NOT_SHARED if one of the pages to update wasn't a
 *          shared page.
 * @retval  VERR_GMM_NOT_PAGE_OWNER if one of the pages to be updated wasn't
 *          owned by the VM.
 * @retval  VERR_GMM_HIT_GLOBAL_LIMIT if we've exhausted the available pages.
 * @retval  VERR_GMM_HIT_VM_ACCOUNT_LIMIT if we've hit the VM account limit,
 *          that is we're trying to allocate more than we've reserved.
 *
 * @param   pGVM                The global (ring-0) VM structure.
 * @param   idCpu               The VCPU id.
 * @param   cPagesToUpdate      The number of pages to update (starting from the head).
 * @param   cPagesToAlloc       The number of pages to allocate (starting from the head).
 * @param   paPages             The array of page descriptors.
 *                              See GMMPAGEDESC for details on what is expected on input.
 * @thread  EMT(idCpu)
 */
GMMR0DECL(int) GMMR0AllocateHandyPages(PGVM pGVM, VMCPUID idCpu, uint32_t cPagesToUpdate,
                                       uint32_t cPagesToAlloc, PGMMPAGEDESC paPages)
{
    LogFlow(("GMMR0AllocateHandyPages: pGVM=%p cPagesToUpdate=%#x cPagesToAlloc=%#x paPages=%p\n",
             pGVM, cPagesToUpdate, cPagesToAlloc, paPages));

    /*
     * Validate & get basics.
     * (This is a relatively busy path, so make predictions where possible.)
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    AssertPtrReturn(paPages, VERR_INVALID_PARAMETER);
    AssertMsgReturn(    (cPagesToUpdate && cPagesToUpdate < 1024)
                    ||  (cPagesToAlloc  && cPagesToAlloc < 1024),
                    ("cPagesToUpdate=%#x cPagesToAlloc=%#x\n", cPagesToUpdate, cPagesToAlloc),
                    VERR_INVALID_PARAMETER);

    unsigned iPage = 0;
    for (; iPage < cPagesToUpdate; iPage++)
    {
        AssertMsgReturn(    (    paPages[iPage].HCPhysGCPhys <= GMM_GCPHYS_LAST
                             && !(paPages[iPage].HCPhysGCPhys & GUEST_PAGE_OFFSET_MASK))
                        ||  paPages[iPage].HCPhysGCPhys == NIL_GMMPAGEDESC_PHYS
                        ||  paPages[iPage].HCPhysGCPhys == GMM_GCPHYS_UNSHAREABLE,
                        ("#%#x: %RHp\n", iPage, paPages[iPage].HCPhysGCPhys),
                        VERR_INVALID_PARAMETER);
        /* ignore fZeroed here */
        AssertMsgReturn(    paPages[iPage].idPage <= GMM_PAGEID_LAST
                        /*||  paPages[iPage].idPage == NIL_GMM_PAGEID*/,
                        ("#%#x: %#x\n", iPage, paPages[iPage].idPage), VERR_INVALID_PARAMETER);
        AssertMsgReturn(   paPages[iPage].idSharedPage == NIL_GMM_PAGEID
                        || paPages[iPage].idSharedPage <= GMM_PAGEID_LAST,
                        ("#%#x: %#x\n", iPage, paPages[iPage].idSharedPage), VERR_INVALID_PARAMETER);
    }

    for (; iPage < cPagesToAlloc; iPage++)
    {
        AssertMsgReturn(paPages[iPage].HCPhysGCPhys == NIL_GMMPAGEDESC_PHYS, ("#%#x: %RHp\n", iPage, paPages[iPage].HCPhysGCPhys), VERR_INVALID_PARAMETER);
        AssertMsgReturn(paPages[iPage].fZeroed      == false,          ("#%#x: %#x\n", iPage, paPages[iPage].fZeroed),       VERR_INVALID_PARAMETER);
        AssertMsgReturn(paPages[iPage].idPage       == NIL_GMM_PAGEID, ("#%#x: %#x\n", iPage, paPages[iPage].idPage),        VERR_INVALID_PARAMETER);
        AssertMsgReturn(paPages[iPage].idSharedPage == NIL_GMM_PAGEID, ("#%#x: %#x\n", iPage, paPages[iPage].idSharedPage),  VERR_INVALID_PARAMETER);
    }

    /*
     * Take the semaphore
     */
    VMMR0EMTBLOCKCTX Ctx;
    PGVMCPU          pGVCpu = &pGVM->aCpus[idCpu];
    rc = VMMR0EmtPrepareToBlock(pGVCpu, VINF_SUCCESS, "GMMR0AllocateHandyPages", pGMM, &Ctx);
    AssertRCReturn(rc, rc);

    rc = gmmR0MutexAcquire(pGMM);
    if (   RT_SUCCESS(rc)
        && GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        /* No allocations before the initial reservation has been made! */
        if (RT_LIKELY(    pGVM->gmm.s.Stats.Reserved.cBasePages
                      &&  pGVM->gmm.s.Stats.Reserved.cFixedPages
                      &&  pGVM->gmm.s.Stats.Reserved.cShadowPages))
        {
            /*
             * Perform the updates.
             * Stop on the first error.
             */
            for (iPage = 0; iPage < cPagesToUpdate; iPage++)
            {
                if (paPages[iPage].idPage != NIL_GMM_PAGEID)
                {
                    PGMMPAGE pPage = gmmR0GetPage(pGMM, paPages[iPage].idPage);
                    if (RT_LIKELY(pPage))
                    {
                        if (RT_LIKELY(GMM_PAGE_IS_PRIVATE(pPage)))
                        {
                            if (RT_LIKELY(pPage->Private.hGVM == pGVM->hSelf))
                            {
                                AssertCompile(NIL_RTHCPHYS > GMM_GCPHYS_LAST && GMM_GCPHYS_UNSHAREABLE > GMM_GCPHYS_LAST);
                                if (RT_LIKELY(paPages[iPage].HCPhysGCPhys <= GMM_GCPHYS_LAST))
                                    pPage->Private.pfn = paPages[iPage].HCPhysGCPhys >> GUEST_PAGE_SHIFT;
                                else if (paPages[iPage].HCPhysGCPhys == GMM_GCPHYS_UNSHAREABLE)
                                    pPage->Private.pfn = GMM_PAGE_PFN_UNSHAREABLE;
                                /* else: NIL_RTHCPHYS nothing */

                                paPages[iPage].idPage       = NIL_GMM_PAGEID;
                                paPages[iPage].HCPhysGCPhys = NIL_GMMPAGEDESC_PHYS;
                                paPages[iPage].fZeroed      = false;
                            }
                            else
                            {
                                Log(("GMMR0AllocateHandyPages: #%#x/%#x: Not owner! hGVM=%#x hSelf=%#x\n",
                                     iPage, paPages[iPage].idPage, pPage->Private.hGVM, pGVM->hSelf));
                                rc = VERR_GMM_NOT_PAGE_OWNER;
                                break;
                            }
                        }
                        else
                        {
                            Log(("GMMR0AllocateHandyPages: #%#x/%#x: Not private! %.*Rhxs (type %d)\n", iPage, paPages[iPage].idPage, sizeof(*pPage), pPage, pPage->Common.u2State));
                            rc = VERR_GMM_PAGE_NOT_PRIVATE;
                            break;
                        }
                    }
                    else
                    {
                        Log(("GMMR0AllocateHandyPages: #%#x/%#x: Not found! (private)\n", iPage, paPages[iPage].idPage));
                        rc = VERR_GMM_PAGE_NOT_FOUND;
                        break;
                    }
                }

                if (paPages[iPage].idSharedPage == NIL_GMM_PAGEID)
                { /* likely */ }
                else
                {
                    PGMMPAGE pPage = gmmR0GetPage(pGMM, paPages[iPage].idSharedPage);
                    if (RT_LIKELY(pPage))
                    {
                        if (RT_LIKELY(GMM_PAGE_IS_SHARED(pPage)))
                        {
                            AssertCompile(NIL_RTHCPHYS > GMM_GCPHYS_LAST && GMM_GCPHYS_UNSHAREABLE > GMM_GCPHYS_LAST);
                            Assert(pPage->Shared.cRefs);
                            Assert(pGVM->gmm.s.Stats.cSharedPages);
                            Assert(pGVM->gmm.s.Stats.Allocated.cBasePages);

                            Log(("GMMR0AllocateHandyPages: free shared page %x cRefs=%d\n", paPages[iPage].idSharedPage, pPage->Shared.cRefs));
                            pGVM->gmm.s.Stats.cSharedPages--;
                            pGVM->gmm.s.Stats.Allocated.cBasePages--;
                            if (!--pPage->Shared.cRefs)
                                gmmR0FreeSharedPage(pGMM, pGVM, paPages[iPage].idSharedPage, pPage);
                            else
                            {
                                Assert(pGMM->cDuplicatePages);
                                pGMM->cDuplicatePages--;
                            }

                            paPages[iPage].idSharedPage = NIL_GMM_PAGEID;
                        }
                        else
                        {
                            Log(("GMMR0AllocateHandyPages: #%#x/%#x: Not shared!\n", iPage, paPages[iPage].idSharedPage));
                            rc = VERR_GMM_PAGE_NOT_SHARED;
                            break;
                        }
                    }
                    else
                    {
                        Log(("GMMR0AllocateHandyPages: #%#x/%#x: Not found! (shared)\n", iPage, paPages[iPage].idSharedPage));
                        rc = VERR_GMM_PAGE_NOT_FOUND;
                        break;
                    }
                }
            } /* for each page to update */

            if (RT_SUCCESS(rc) && cPagesToAlloc > 0)
            {
#ifdef VBOX_STRICT
                for (iPage = 0; iPage < cPagesToAlloc; iPage++)
                {
                    Assert(paPages[iPage].HCPhysGCPhys  == NIL_GMMPAGEDESC_PHYS);
                    Assert(paPages[iPage].fZeroed       == false);
                    Assert(paPages[iPage].idPage        == NIL_GMM_PAGEID);
                    Assert(paPages[iPage].idSharedPage  == NIL_GMM_PAGEID);
                }
#endif

                /*
                 * Join paths with GMMR0AllocatePages for the allocation.
                 * Note! gmmR0AllocateMoreChunks may leave the protection of the mutex!
                 */
                rc = gmmR0AllocatePagesNew(pGMM, pGVM, cPagesToAlloc, paPages, GMMACCOUNT_BASE);
            }
        }
        else
            rc = VERR_WRONG_ORDER;
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
        gmmR0MutexRelease(pGMM);
    }
    else if (RT_SUCCESS(rc))
    {
        gmmR0MutexRelease(pGMM);
        rc = VERR_GMM_IS_NOT_SANE;
    }
    VMMR0EmtResumeAfterBlocking(pGVCpu, &Ctx);

    LogFlow(("GMMR0AllocateHandyPages: returns %Rrc\n", rc));
    return rc;
}


/**
 * Allocate one or more pages.
 *
 * This is typically used for ROMs and MMIO2 (VRAM) during VM creation.
 * The allocated pages are not cleared and will contain random garbage.
 *
 * @returns VBox status code:
 * @retval  VINF_SUCCESS on success.
 * @retval  VERR_NOT_OWNER if the caller is not an EMT.
 * @retval  VERR_GMM_HIT_GLOBAL_LIMIT if we've exhausted the available pages.
 * @retval  VERR_GMM_HIT_VM_ACCOUNT_LIMIT if we've hit the VM account limit,
 *          that is we're trying to allocate more than we've reserved.
 *
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   cPages      The number of pages to allocate.
 * @param   paPages     Pointer to the page descriptors.
 *                      See GMMPAGEDESC for details on what is expected on
 *                      input.
 * @param   enmAccount  The account to charge.
 *
 * @thread  EMT.
 */
//负责为虚拟机分配 物理内存页，支持多种内存类型（基础页、固定页、影子页等）
/*
  核心任务：为指定虚拟机（pGVM）分配 cPages 个物理页，填充到 paPages 描述符数组中。
  内存类型（enmAccount）：
    GMMACCOUNT_BASE：普通虚拟机内存（可共享或独占）。
    GMMACCOUNT_FIXED：固定内存（如 DMA 缓冲区）。
    GMMACCOUNT_SHADOW：影子页表内存。
  设计目标：
    确保分配不超过虚拟机预留配额（Reserved.c*Pages）。
    支持物理地址预绑定（HCPhysGCPhys）和零页初始化需求。
*/

/*
 * paPages	PGMMPAGEDESC	页描述符数组（输出物理页信息）
*/

GMMR0DECL(int) GMMR0AllocatePages(PGVM pGVM, VMCPUID idCpu, uint32_t cPages, PGMMPAGEDESC paPages, GMMACCOUNT enmAccount)
{
    LogFlow(("GMMR0AllocatePages: pGVM=%p cPages=%#x paPages=%p enmAccount=%d\n", pGVM, cPages, paPages, enmAccount));

    /*
     * Validate, get basics and take the semaphore.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    AssertPtrReturn(paPages, VERR_INVALID_PARAMETER);
    AssertMsgReturn(enmAccount > GMMACCOUNT_INVALID && enmAccount < GMMACCOUNT_END, ("%d\n", enmAccount), VERR_INVALID_PARAMETER);
    AssertMsgReturn(cPages > 0 && cPages < RT_BIT(32 - GUEST_PAGE_SHIFT), ("%#x\n", cPages), VERR_INVALID_PARAMETER);

    for (unsigned iPage = 0; iPage < cPages; iPage++)
    {
        AssertMsgReturn(    paPages[iPage].HCPhysGCPhys == NIL_GMMPAGEDESC_PHYS
                        ||  paPages[iPage].HCPhysGCPhys == GMM_GCPHYS_UNSHAREABLE
                        ||  (    enmAccount == GMMACCOUNT_BASE
                             &&  paPages[iPage].HCPhysGCPhys <= GMM_GCPHYS_LAST
                             && !(paPages[iPage].HCPhysGCPhys & GUEST_PAGE_OFFSET_MASK)),
                        ("#%#x: %RHp enmAccount=%d\n", iPage, paPages[iPage].HCPhysGCPhys, enmAccount),
                        VERR_INVALID_PARAMETER);
        AssertMsgReturn(paPages[iPage].fZeroed      == false,          ("#%#x: %#x\n", iPage, paPages[iPage].fZeroed), VERR_INVALID_PARAMETER);
        AssertMsgReturn(paPages[iPage].idPage       == NIL_GMM_PAGEID, ("#%#x: %#x\n", iPage, paPages[iPage].idPage), VERR_INVALID_PARAMETER);
        AssertMsgReturn(paPages[iPage].idSharedPage == NIL_GMM_PAGEID, ("#%#x: %#x\n", iPage, paPages[iPage].idSharedPage), VERR_INVALID_PARAMETER);
    }

    /*
     * Grab the giant mutex and get working.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {

        /* No allocations before the initial reservation has been made! */
        if (RT_LIKELY(    pGVM->gmm.s.Stats.Reserved.cBasePages
                      &&  pGVM->gmm.s.Stats.Reserved.cFixedPages
                      &&  pGVM->gmm.s.Stats.Reserved.cShadowPages))
            /*
            分配策略：
              优先从 空闲页链表（pGMM->FreeList）获取页面。
              若空闲页不足，从主机 OS 申请新内存（RTR0MemObjAllocPage）。
            页描述符填充：
              设置 HCPhysGCPhys（主机物理地址）和 idPage（GMM 页 ID）。
              标记零页（fZeroed）若需初始化。
             * */
            rc = gmmR0AllocatePagesNew(pGMM, pGVM, cPages, paPages, enmAccount);
        else
            rc = VERR_WRONG_ORDER;// 未预留内存配
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;
    gmmR0MutexRelease(pGMM);

    LogFlow(("GMMR0AllocatePages: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0AllocatePages.
 *
 * @returns see GMMR0AllocatePages.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0AllocatePagesReq(PGVM pGVM, VMCPUID idCpu, PGMMALLOCATEPAGESREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq >= RT_UOFFSETOF(GMMALLOCATEPAGESREQ, aPages[0]),
                    ("%#x < %#x\n", pReq->Hdr.cbReq, RT_UOFFSETOF(GMMALLOCATEPAGESREQ, aPages[0])),
                    VERR_INVALID_PARAMETER);
    AssertMsgReturn(pReq->Hdr.cbReq == RT_UOFFSETOF_DYN(GMMALLOCATEPAGESREQ, aPages[pReq->cPages]),
                    ("%#x != %#x\n", pReq->Hdr.cbReq, RT_UOFFSETOF_DYN(GMMALLOCATEPAGESREQ, aPages[pReq->cPages])),
                    VERR_INVALID_PARAMETER);

    return GMMR0AllocatePages(pGVM, idCpu, pReq->cPages, &pReq->aPages[0], pReq->enmAccount);
}


/**
 * Allocate a large page to represent guest RAM
 *
 * The allocated pages are zeroed upon return.
 *
 * @returns VBox status code:
 * @retval  VINF_SUCCESS on success.
 * @retval  VERR_NOT_OWNER if the caller is not an EMT.
 * @retval  VERR_GMM_HIT_GLOBAL_LIMIT if we've exhausted the available pages.
 * @retval  VERR_GMM_HIT_VM_ACCOUNT_LIMIT if we've hit the VM account limit,
 *          that is we're trying to allocate more than we've reserved.
 * @retval  VERR_TRY_AGAIN if the host is temporarily out of large pages.
 * @returns see GMMR0AllocatePages.
 *
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   cbPage      Large page size.
 * @param   pIdPage     Where to return the GMM page ID of the page.
 * @param   pHCPhys     Where to return the host physical address of the page.
 */
//该函数是 VirtualBox 大页内存分配的核心接口，用于为虚拟机分配 2MB 大页,以提升内存访问性能
//支持 大页内存映射（减少 TLB 缺失，提升性能）。
//确保内存分配不超过虚拟机配额（Reserved.cBasePages）。
GMMR0DECL(int)  GMMR0AllocateLargePage(PGVM pGVM, VMCPUID idCpu, uint32_t cbPage, uint32_t *pIdPage, RTHCPHYS *pHCPhys)
{
    LogFlow(("GMMR0AllocateLargePage: pGVM=%p cbPage=%x\n", pGVM, cbPage));

    AssertPtrReturn(pIdPage, VERR_INVALID_PARAMETER);
    *pIdPage = NIL_GMM_PAGEID;
    AssertPtrReturn(pHCPhys, VERR_INVALID_PARAMETER);
    *pHCPhys = NIL_RTHCPHYS;
    //GMM_CHUNK_SIZE，即 2MB
    AssertReturn(cbPage == GMM_CHUNK_SIZE, VERR_INVALID_PARAMETER);

    /*
     * Validate GVM + idCpu, get basics and take the semaphore.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    AssertRCReturn(rc, rc);

    VMMR0EMTBLOCKCTX Ctx;
    PGVMCPU          pGVCpu = &pGVM->aCpus[idCpu];
    rc = VMMR0EmtPrepareToBlock(pGVCpu, VINF_SUCCESS, "GMMR0AllocateLargePage", pGMM, &Ctx);
    AssertRCReturn(rc, rc);

    rc = gmmR0MutexAcquire(pGMM);
    if (RT_SUCCESS(rc))
    {
        if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
        {
            /*
             * Check the quota.
             */
            /** @todo r=bird: Quota checking could be done w/o the giant mutex but using
             *        a VM specific mutex... */
            // 已分配页数（Allocated.cBasePages） + 本次请求（GMM_CHUNK_NUM_PAGES） ≤ 预留配额（Reserved.cBasePages）。
            if (RT_LIKELY(   pGVM->gmm.s.Stats.Allocated.cBasePages + pGVM->gmm.s.Stats.cBalloonedPages + GMM_CHUNK_NUM_PAGES
                          <= pGVM->gmm.s.Stats.Reserved.cBasePages))
            {
                /*
                 * Allocate a new large page chunk.
                 *
                 * Note! We leave the giant GMM lock temporarily as the allocation might
                 *       take a long time.  gmmR0RegisterChunk will retake it (ugly).
                 */
                AssertCompile(GMM_CHUNK_SIZE == _2M);
                gmmR0MutexRelease(pGMM);

                RTR0MEMOBJ hMemObj;
                //RTMEMOBJ_ALLOC_LARGE_F_FAST 标志优先使用大页
                rc = RTR0MemObjAllocLarge(&hMemObj, GMM_CHUNK_SIZE, GMM_CHUNK_SIZE, RTMEMOBJ_ALLOC_LARGE_F_FAST);
                if (RT_SUCCESS(rc))
                {
                    *pHCPhys = RTR0MemObjGetPagePhysAddr(hMemObj, 0);

                    /*
                     * Register the chunk as fully allocated.
                     * Note! As mentioned above, this will return owning the mutex on success.
                     */
                    PGMMCHUNK              pChunk = NULL;
                    PGMMCHUNKFREESET const pSet   = pGMM->fBoundMemoryMode ? &pGVM->gmm.s.Private : &pGMM->PrivateX;
                    /*
                      将内存块注册为 大页类型（GMM_CHUNK_FLAGS_LARGE_PAGE）。
                      绑定到虚拟机的 私有内存池（pGVM->gmm.s.Private 或 pGMM->PrivateX）。
                      返回 pChunk 对象（包含页 ID 和元数据）。
                    */
                    rc = gmmR0RegisterChunk(pGMM, pSet, hMemObj, pGVM->hSelf, pGVM->pSession, GMM_CHUNK_FLAGS_LARGE_PAGE,
                                            0 /*cPages*/, NULL /*paPages*/, NULL /*piPage*/, &pChunk);
                    if (RT_SUCCESS(rc))
                    {
                        /*
                         * The gmmR0RegisterChunk call already marked all pages allocated,
                         * so we just have to fill in the return values and update stats now.
                         */
                        *pIdPage = pChunk->Core.Key << GMM_CHUNKID_SHIFT;

                        /* Update accounting. */
                        pGVM->gmm.s.Stats.Allocated.cBasePages += GMM_CHUNK_NUM_PAGES;
                        pGVM->gmm.s.Stats.cPrivatePages        += GMM_CHUNK_NUM_PAGES;
                        pGMM->cAllocatedPages                  += GMM_CHUNK_NUM_PAGES;

                        gmmR0LinkChunk(pChunk, pSet); // 将内存块加入空闲集合
                        gmmR0MutexRelease(pGMM);

                        VMMR0EmtResumeAfterBlocking(pGVCpu, &Ctx);// 恢复 vCPU 执行
                        LogFlow(("GMMR0AllocateLargePage: returns VINF_SUCCESS\n"));
                        return VINF_SUCCESS;
                    }

                    /*
                     * Bail out.
                     */
                    RTR0MemObjFree(hMemObj, true /* fFreeMappings */);
                    *pHCPhys = NIL_RTHCPHYS;
                }
                /** @todo r=bird: Turn VERR_NO_MEMORY etc into VERR_TRY_AGAIN?  Docs say we
                 *        return it, but I am sure IPRT doesn't... */
            }
            else
            {
                Log(("GMMR0AllocateLargePage: Reserved=%#llx Allocated+Requested=%#llx+%#x!\n",
                     pGVM->gmm.s.Stats.Reserved.cBasePages, pGVM->gmm.s.Stats.Allocated.cBasePages, GMM_CHUNK_NUM_PAGES));
                gmmR0MutexRelease(pGMM);
                rc = VERR_GMM_HIT_VM_ACCOUNT_LIMIT;// 超出配额
            }
        }
        else
        {
            gmmR0MutexRelease(pGMM);
            rc = VERR_GMM_IS_NOT_SANE;
        }
    }

    VMMR0EmtResumeAfterBlocking(pGVCpu, &Ctx);
    LogFlow(("GMMR0AllocateLargePage: returns %Rrc\n", rc));
    return rc;
}


/**
 * Free a large page.
 *
 * @returns VBox status code:
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   idPage      The large page id.
 */
GMMR0DECL(int)  GMMR0FreeLargePage(PGVM pGVM, VMCPUID idCpu, uint32_t idPage)
{
    LogFlow(("GMMR0FreeLargePage: pGVM=%p idPage=%x\n", pGVM, idPage));

    /*
     * Validate, get basics and take the semaphore.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        const unsigned cPages = GMM_CHUNK_NUM_PAGES;

        if (RT_UNLIKELY(pGVM->gmm.s.Stats.Allocated.cBasePages < cPages))
        {
            Log(("GMMR0FreeLargePage: allocated=%#llx cPages=%#x!\n", pGVM->gmm.s.Stats.Allocated.cBasePages, cPages));
            gmmR0MutexRelease(pGMM);
            return VERR_GMM_ATTEMPT_TO_FREE_TOO_MUCH;
        }

        PGMMPAGE pPage = gmmR0GetPage(pGMM, idPage);
        if (RT_LIKELY(   pPage
                      && GMM_PAGE_IS_PRIVATE(pPage)))
        {
            PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
            Assert(pChunk);
            Assert(pChunk->cFree < GMM_CHUNK_NUM_PAGES);
            Assert(pChunk->cPrivate > 0);

            /* Release the memory immediately. */
            gmmR0FreeChunk(pGMM, NULL, pChunk, false /*fRelaxedSem*/); /** @todo this can be relaxed too! */

            /* Update accounting. */
            pGVM->gmm.s.Stats.Allocated.cBasePages -= cPages;
            pGVM->gmm.s.Stats.cPrivatePages        -= cPages;
            pGMM->cAllocatedPages                  -= cPages;
        }
        else
            rc = VERR_GMM_PAGE_NOT_FOUND;
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    LogFlow(("GMMR0FreeLargePage: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0FreeLargePage.
 *
 * @returns see GMMR0FreeLargePage.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0FreeLargePageReq(PGVM pGVM, VMCPUID idCpu, PGMMFREELARGEPAGEREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(GMMFREEPAGESREQ),
                    ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(GMMFREEPAGESREQ)),
                    VERR_INVALID_PARAMETER);

    return GMMR0FreeLargePage(pGVM, idCpu, pReq->idPage);
}


/**
 * @callback_method_impl{FNGVMMR0ENUMCALLBACK,
 * Used by gmmR0FreeChunkFlushPerVmTlbs().}
 */
//强制刷新单个虚拟机的 Chunk TLB（Translation Lookaside Buffer）缓存
/*
轻量级无效化：
  仅重置代次和指针，不涉及内存释放，性能开销极低
惰性验证：
  后续访问 TLB 时，若发现 idGeneration 不匹配当前全局代次（pGMM->idFreeGeneration），自动触发缓存重填
并发安全
  自旋锁保护确保多虚拟机环境下的原子性
 * */
static DECLCALLBACK(int) gmmR0InvalidatePerVmChunkTlbCallback(PGVM pGVM, void *pvUser)
{
    RT_NOREF(pvUser);
    if (pGVM->gmm.s.hChunkTlbSpinLock != NIL_RTSPINLOCK)
    {
        RTSpinlockAcquire(pGVM->gmm.s.hChunkTlbSpinLock);
        //清空 TLB 条目
        /*
         aChunkTlbEntries 是固定大小的数组（通常 4~16 项），缓存虚拟机最近访问的 ‌Chunk 物理地址映射‌‌67。
         每个条目包含：
           idGeneration：内存块释放代次（用于验证缓存有效性）。
           pChunk：指向关联的 GMMCHUNK 对象。

         清空逻辑：
           将代次设为最大值（UINT64_MAX），确保后续访问必然失效
           清空内存块指针（pChunk = NULL），避免悬垂引用。
         * */
        uintptr_t i = RT_ELEMENTS(pGVM->gmm.s.aChunkTlbEntries);
        while (i-- > 0)
        {
            pGVM->gmm.s.aChunkTlbEntries[i].idGeneration = UINT64_MAX;// 标记代次无效
            pGVM->gmm.s.aChunkTlbEntries[i].pChunk       = NULL;     // 清除内存块指针
        }
        RTSpinlockRelease(pGVM->gmm.s.hChunkTlbSpinLock);
    }
    return VINF_SUCCESS;
}


/**
 * Called by gmmR0FreeChunk when we reach the threshold for wrapping around the
 * free generation ID value.
 *
 * This is done at 2^62 - 1, which allows us to drop all locks and as it will
 * take a while before 12 exa (2 305 843 009 213 693 952) calls to
 * gmmR0FreeChunk can be made and causes a real wrap-around.  We do two
 * invalidation passes and resets the generation ID between then.  This will
 * make sure there are no false positives.
 *
 * @param   pGMM        Pointer to the GMM instance.
 */
//刷新所有虚拟机的 Chunk TLB（Translation Lookaside Buffer），
//确保内存释放后各虚拟机的地址映射一致性。
//在 全局内存释放代次（idFreeGeneration）接近溢出时，强制刷新所有虚拟机的 Chunk TLB 缓存
//gmmR0FreeChunk 中检测到 idFreeGeneration == UINT64_MAX / 4（即将溢出）
static void gmmR0FreeChunkFlushPerVmTlbs(PGMM pGMM)
{
    /*
     * First invalidation pass.
     */
    /*
     调用 GVMMR0EnumVMs 遍历所有虚拟机，
     对每个 VM 执行回调函数 gmmR0InvalidatePerVmChunkTlbCallback
       回调函数会清除目标 VM 的 Chunk TLB 缓存（类似 Linux 的 flush_tlb 操作）
    */
    int rc = GVMMR0EnumVMs(gmmR0InvalidatePerVmChunkTlbCallback, NULL);
    AssertRCSuccess(rc);

    /*
     * Reset the generation number.
     */
    //重置代次计数器
    RTSpinlockAcquire(pGMM->hSpinLockTree);
    ASMAtomicWriteU64(&pGMM->idFreeGeneration, 1);// 重置为 1
    RTSpinlockRelease(pGMM->hSpinLockTree);

    /*
     * Second invalidation pass.
     */
    //第二次 TLB 无效化
    /*
       防止在第一次无效化和计数器重置之间，有虚拟机缓存了旧的代次信息
       双重刷新确保所有 VM 的 TLB 完全同步
    */
    rc = GVMMR0EnumVMs(gmmR0InvalidatePerVmChunkTlbCallback, NULL);
    AssertRCSuccess(rc);
}


/**
 * Frees a chunk, giving it back to the host OS.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        This is set when called from GMMR0CleanupVM so we can
 *                      unmap and free the chunk in one go.
 * @param   pChunk      The chunk to free.
 * @param   fRelaxedSem Whether we can release the semaphore while doing the
 *                      freeing (@c true) or not.
 */
//负责 释放整个内存块（Chunk）并归还物理内存给主机 OS。
/*
  检查内存块是否可释放（无映射）。
  从全局数据结构中解链。
  释放物理内存和元数据。
*/
/*
  pGMM	PGMM	全局内存管理器
  pGVM	PGVM	关联的虚拟机（可为 NULL）
  pChunk	PGMMCHUNK	目标内存块
  fRelaxedSem	bool	是否临时释放全局锁（避免死锁）
 * */
//每个 GMMCHUNK 对象包含 页数组（aPages）和 映射表（paMappingsX）。
static bool gmmR0FreeChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, bool fRelaxedSem)
{
    Assert(pChunk->Core.Key != NIL_GMM_CHUNKID);

    GMMR0CHUNKMTXSTATE MtxState;
    //获取 内存块级锁（pChunk->Mtx）和 全局锁（pGMM->Mtx）
    gmmR0ChunkMutexAcquire(&MtxState, pGMM, pChunk, GMMR0CHUNK_MTX_KEEP_GIANT);

    /*
     * Cleanup hack! Unmap the chunk from the callers address space.
     * This shouldn't happen, so screw lock contention...
     */
    if (pChunk->cMappingsX && pGVM)
        gmmR0UnmapChunkLocked(pGMM, pGVM, pChunk);// 强制解除当前 VM 的映射

    /*
     * If there are current mappings of the chunk, then request the
     * VMs to unmap them. Reposition the chunk in the free list so
     * it won't be a likely candidate for allocations.
     */
    if (pChunk->cMappingsX)
    {
        /** @todo R0 -> VM request */
        /* The chunk can be mapped by more than one VM if fBoundMemoryMode is false! */
        Log(("gmmR0FreeChunk: chunk still has %d mappings; don't free!\n", pChunk->cMappingsX));
        gmmR0ChunkMutexRelease(&MtxState, pChunk);
        return false;// 存在映射则放弃释放
    }


    /*
     * Save and trash the handle.
     */
    RTR0MEMOBJ const hMemObj = pChunk->hMemObj;
    pChunk->hMemObj = NIL_RTR0MEMOBJ;// 标记为无效

    /*
     * Unlink it from everywhere.
     */
    gmmR0UnlinkChunk(pChunk); // 从空闲集合解链

    RTSpinlockAcquire(pGMM->hSpinLockTree);

    RTListNodeRemove(&pChunk->ListNode);// 从全局链表移除

    PAVLU32NODECORE pCore = RTAvlU32Remove(&pGMM->pChunks, pChunk->Core.Key);// 从 AVL 树移除
    Assert(pCore == &pChunk->Core); NOREF(pCore);

    //ChunkTLB 缓存最近访问的内存块，加速查找
    PGMMCHUNKTLBE pTlbe = &pGMM->ChunkTLB.aEntries[GMM_CHUNKTLB_IDX(pChunk->Core.Key)];
    //清除 ChunkTLB 缓存（若命中）
    if (pTlbe->pChunk == pChunk)
    {
        pTlbe->idChunk = NIL_GMM_CHUNKID;
        pTlbe->pChunk = NULL;
    }

    Assert(pGMM->cChunks > 0);
    pGMM->cChunks--;// 减少全局内存块计数

    uint64_t const idFreeGeneration = ASMAtomicIncU64(&pGMM->idFreeGeneration);// 递增释放代次

    RTSpinlockRelease(pGMM->hSpinLockTree);

    pGMM->cFreedChunks++; // 增加释放计数

    /* Drop the lock. */
    gmmR0ChunkMutexRelease(&MtxState, NULL);
    if (fRelaxedSem)
        gmmR0MutexRelease(pGMM);

    /*
     * Flush per VM chunk TLBs if we're getting remotely close to a generation wraparound.
     */
    if (idFreeGeneration == UINT64_MAX / 4)
        gmmR0FreeChunkFlushPerVmTlbs(pGMM);

    /*
     * Free the Chunk ID and all memory associated with the chunk.
     */
    gmmR0FreeChunkId(pGMM, pChunk->Core.Key); // 回收 ChunkID
    pChunk->Core.Key = NIL_GMM_CHUNKID;// 标记为无效

    RTMemFree(pChunk->paMappingsX);// 释放映射数组
    pChunk->paMappingsX = NULL;

    RTMemFree(pChunk); // 释放内存块对象

    //归还主机物理内存
#ifndef VBOX_WITH_LINEAR_HOST_PHYS_MEM
    int rc = RTR0MemObjFree(hMemObj, true /* fFreeMappings */);
#else
    //模式下需保留映射
    int rc = RTR0MemObjFree(hMemObj, false /* fFreeMappings */);
#endif
    AssertLogRelRC(rc);

    if (fRelaxedSem)
        gmmR0MutexAcquire(pGMM);
    return fRelaxedSem;
}


/**
 * Free page worker.
 *
 * The caller does all the statistic decrementing, we do all the incrementing.
 *
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the GVM instance.
 * @param   pChunk      Pointer to the chunk this page belongs to.
 * @param   idPage      The Page ID.
 * @param   pPage       Pointer to the page.
 */
//虚拟机释放私有页（gmmR0FreePrivatePage）。
//共享页引用计数归零（gmmR0FreeSharedPage）。
/*
  将页加入空闲链表。
  更新内存块和全局统计。
  在特定条件下触发 内存块释放（归还给主机 OS）。
*/
static void gmmR0FreePageWorker(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, uint32_t idPage, PGMMPAGE pPage)
{
    Log3(("F pPage=%p iPage=%#x/%#x u2State=%d iFreeHead=%#x\n",
          pPage, pPage - &pChunk->aPages[0], idPage, pPage->Common.u2State, pChunk->iFreeHead)); NOREF(idPage);

    /*
     * Put the page on the free list.
     */
    pPage->u = 0;// 清空页的所有字段
    pPage->Free.u2State = GMM_PAGE_STATE_FREE;// 标记为“空闲”
    pPage->Free.fZeroed = false;// 标记为“未清零”（后续分配时需清零）
    Assert(pChunk->iFreeHead < RT_ELEMENTS(pChunk->aPages) || pChunk->iFreeHead == UINT16_MAX);
    //加入空闲链表
    pPage->Free.iNext = pChunk->iFreeHead;// 新页指向当前链表头部
    pChunk->iFreeHead = pPage - &pChunk->aPages[0];// 更新链表头部为当前页

    /*
     * Update statistics (the cShared/cPrivate stats are up to date already),
     * and relink the chunk if necessary.
     */
    unsigned const cFree = pChunk->cFree;
    if (   !cFree
            // 根据空闲页数（cFree）决定内存块所属的集合（pSet）。
            // 当 cFree 跨越阈值（如 0→1 或 127→128）时，需 切换集合
        || gmmR0SelectFreeSetList(cFree) != gmmR0SelectFreeSetList(cFree + 1))
    {
        gmmR0UnlinkChunk(pChunk); // 从当前集合解链
        pChunk->cFree++;// 增加空闲页计数
        gmmR0SelectSetAndLinkChunk(pGMM, pGVM, pChunk);// 重新选择集合并链接
    }
    else
    {
        pChunk->cFree = cFree + 1;// 仅增加空闲页计数
        pChunk->pSet->cFreePages++;// 更新集合的空闲页总数
    }

    /*
     * If the chunk becomes empty, consider giving memory back to the host OS.
     *
     * The current strategy is to try give it back if there are other chunks
     * in this free list, meaning if there are at least 240 free pages in this
     * category. Note that since there are probably mappings of the chunk,
     * it won't be freed up instantly, which probably screws up this logic
     * a bit...
     */
    /** @todo Do this on the way out. */
    if (RT_LIKELY(   pChunk->cFree != GMM_CHUNK_NUM_PAGES// 内存块完全空闲
                  || pChunk->pFreeNext == NULL// 存在其他空闲块
                  || pChunk->pFreePrev == NULL /** @todo this is probably misfiring, see reset... */))// 避免误判
    { /* likely */ }
    else
        //当内存块完全空闲 且同类型空闲块充足（≥240 页）时，归还内存给主机 OS。
        //由于内存块可能仍被映射（paMappingsX），实际释放可能延迟
        gmmR0FreeChunk(pGMM, NULL, pChunk, false);// 尝试释放内存块
}


/**
 * Frees a shared page, the page is known to exist and be valid and such.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        Pointer to the GVM instance.
 * @param   idPage      The page id.
 * @param   pPage       The page structure.
 */
DECLINLINE(void) gmmR0FreeSharedPage(PGMM pGMM, PGVM pGVM, uint32_t idPage, PGMMPAGE pPage)
{
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
    Assert(pChunk);
    Assert(pChunk->cFree < GMM_CHUNK_NUM_PAGES);
    Assert(pChunk->cShared > 0);
    Assert(pGMM->cSharedPages > 0);
    Assert(pGMM->cAllocatedPages > 0);
    Assert(!pPage->Shared.cRefs);

    pChunk->cShared--;
    pGMM->cAllocatedPages--;
    pGMM->cSharedPages--;
    gmmR0FreePageWorker(pGMM, pGVM, pChunk, idPage, pPage);
}


/**
 * Frees a private page, the page is known to exist and be valid and such.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        Pointer to the GVM instance.
 * @param   idPage      The page id.
 * @param   pPage       The page structure.
 */
DECLINLINE(void) gmmR0FreePrivatePage(PGMM pGMM, PGVM pGVM, uint32_t idPage, PGMMPAGE pPage)
{
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
    Assert(pChunk);
    Assert(pChunk->cFree < GMM_CHUNK_NUM_PAGES);
    Assert(pChunk->cPrivate > 0);
    Assert(pGMM->cAllocatedPages > 0);

    pChunk->cPrivate--;
    pGMM->cAllocatedPages--;
    gmmR0FreePageWorker(pGMM, pGVM, pChunk, idPage, pPage);
}


/**
 * Common worker for GMMR0FreePages and GMMR0BalloonedPages.
 *
 * @returns VBox status code:
 * @retval  xxx
 *
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the VM.
 * @param   cPages      The number of pages to free.
 * @param   paPages     Pointer to the page descriptors.
 * @param   enmAccount  The account this relates to.
 */
//释放虚拟机（VM）的内存页，处理 私有页（Private）和 共享页（Shared）的不同逻辑
/*
paPages: 页描述符数组（包含 idPage 标识符）。
enmAccount: 内存账户类型（BASE / SHADOW / FIXED）
 * */
static int gmmR0FreePages(PGMM pGMM, PGVM pGVM, uint32_t cPages, PGMMFREEPAGEDESC paPages, GMMACCOUNT enmAccount)
{
    /*
     * Check that the request isn't impossible wrt to the account status.
     */
    switch (enmAccount)
    {
        case GMMACCOUNT_BASE:
            //确保 已分配页数 ≥ 请求释放页数
            if (RT_UNLIKELY(pGVM->gmm.s.Stats.Allocated.cBasePages < cPages))
            {
                Log(("gmmR0FreePages: allocated=%#llx cPages=%#x!\n", pGVM->gmm.s.Stats.Allocated.cBasePages, cPages));
                return VERR_GMM_ATTEMPT_TO_FREE_TOO_MUCH;
            }
            break;
        case GMMACCOUNT_SHADOW:
            if (RT_UNLIKELY(pGVM->gmm.s.Stats.Allocated.cShadowPages < cPages))
            {
                Log(("gmmR0FreePages: allocated=%#llx cPages=%#x!\n", pGVM->gmm.s.Stats.Allocated.cShadowPages, cPages));
                return VERR_GMM_ATTEMPT_TO_FREE_TOO_MUCH;
            }
            break;
        case GMMACCOUNT_FIXED:
            if (RT_UNLIKELY(pGVM->gmm.s.Stats.Allocated.cFixedPages < cPages))
            {
                Log(("gmmR0FreePages: allocated=%#llx cPages=%#x!\n", pGVM->gmm.s.Stats.Allocated.cFixedPages, cPages));
                return VERR_GMM_ATTEMPT_TO_FREE_TOO_MUCH;
            }
            break;
        default:
            AssertMsgFailedReturn(("enmAccount=%d\n", enmAccount), VERR_IPE_NOT_REACHED_DEFAULT_CASE);
    }

    /*
     * Walk the descriptors and free the pages.
     *
     * Statistics (except the account) are being updated as we go along,
     * unlike the alloc code. Also, stop on the first error.
     */
    int rc = VINF_SUCCESS;
    uint32_t iPage;
    //遍历页描述符并释放内存
    for (iPage = 0; iPage < cPages; iPage++)
    {
        uint32_t idPage = paPages[iPage].idPage;
        PGMMPAGE pPage = gmmR0GetPage(pGMM, idPage);//获取页对象
        if (RT_LIKELY(pPage))
        {
            //处理私有页
            if (RT_LIKELY(GMM_PAGE_IS_PRIVATE(pPage)))
            {
                if (RT_LIKELY(pPage->Private.hGVM == pGVM->hSelf))
                {
                    Assert(pGVM->gmm.s.Stats.cPrivatePages);
                    pGVM->gmm.s.Stats.cPrivatePages--; // 更新私有页计数
                    gmmR0FreePrivatePage(pGMM, pGVM, idPage, pPage);// 实际释放
                }
                else
                {
                    Log(("gmmR0AllocatePages: #%#x/%#x: not owner! hGVM=%#x hSelf=%#x\n", iPage, idPage,
                         pPage->Private.hGVM, pGVM->hSelf));
                    rc = VERR_GMM_NOT_PAGE_OWNER;// 非当前 VM 的私有页
                    break;
                }
            }
            // 处理共享页
            else if (RT_LIKELY(GMM_PAGE_IS_SHARED(pPage)))
            {
                Assert(pGVM->gmm.s.Stats.cSharedPages);
                Assert(pPage->Shared.cRefs);// 引用计数必须 > 0
#if defined(VBOX_WITH_PAGE_SHARING) && defined(VBOX_STRICT)
                //确保共享页内容未被篡改（仅调试模式生效）。
                if (pPage->Shared.u14Checksum)
                {
                    uint32_t uChecksum = gmmR0StrictPageChecksum(pGMM, pGVM, idPage);
                    uChecksum &= UINT32_C(0x00003fff);
                    AssertMsg(!uChecksum || uChecksum == pPage->Shared.u14Checksum,
                              ("%#x vs %#x - idPage=%#x\n", uChecksum, pPage->Shared.u14Checksum, idPage));
                }
#endif
                pGVM->gmm.s.Stats.cSharedPages--;// 更新共享页计数
                if (!--pPage->Shared.cRefs) // 引用归零时释放
                    gmmR0FreeSharedPage(pGMM, pGVM, idPage, pPage);
                else
                {
                    Assert(pGMM->cDuplicatePages);
                    pGMM->cDuplicatePages--; // 仅减少全局重复页计数
                }
            }
            else
            {
                Log(("gmmR0AllocatePages: #%#x/%#x: already free!\n", iPage, idPage));
                rc = VERR_GMM_PAGE_ALREADY_FREE; // 页已空闲
                break;
            }
        }
        else
        {
            Log(("gmmR0AllocatePages: #%#x/%#x: not found!\n", iPage, idPage));
            rc = VERR_GMM_PAGE_NOT_FOUND;// 页不存在
            break;
        }
        paPages[iPage].idPage = NIL_GMM_PAGEID;// 标记为已释放
    }

    /*
     * Update the account.
     */
    switch (enmAccount)
    {
        case GMMACCOUNT_BASE:   pGVM->gmm.s.Stats.Allocated.cBasePages   -= iPage; break;
        case GMMACCOUNT_SHADOW: pGVM->gmm.s.Stats.Allocated.cShadowPages -= iPage; break;
        case GMMACCOUNT_FIXED:  pGVM->gmm.s.Stats.Allocated.cFixedPages  -= iPage; break;
        default:
            AssertMsgFailedReturn(("enmAccount=%d\n", enmAccount), VERR_IPE_NOT_REACHED_DEFAULT_CASE);
    }

    /*
     * Any threshold stuff to be done here?
     */

    return rc;
}


/**
 * Free one or more pages.
 *
 * This is typically used at reset time or power off.
 *
 * @returns VBox status code:
 * @retval  xxx
 *
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   cPages      The number of pages to allocate.
 * @param   paPages     Pointer to the page descriptors containing the page IDs
 *                      for each page.
 * @param   enmAccount  The account this relates to.
 * @thread  EMT.
 */
GMMR0DECL(int) GMMR0FreePages(PGVM pGVM, VMCPUID idCpu, uint32_t cPages, PGMMFREEPAGEDESC paPages, GMMACCOUNT enmAccount)
{
    LogFlow(("GMMR0FreePages: pGVM=%p cPages=%#x paPages=%p enmAccount=%d\n", pGVM, cPages, paPages, enmAccount));

    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    AssertPtrReturn(paPages, VERR_INVALID_PARAMETER);
    AssertMsgReturn(enmAccount > GMMACCOUNT_INVALID && enmAccount < GMMACCOUNT_END, ("%d\n", enmAccount), VERR_INVALID_PARAMETER);
    AssertMsgReturn(cPages > 0 && cPages < RT_BIT(32 - GUEST_PAGE_SHIFT), ("%#x\n", cPages), VERR_INVALID_PARAMETER);

    for (unsigned iPage = 0; iPage < cPages; iPage++)
        AssertMsgReturn(    paPages[iPage].idPage <= GMM_PAGEID_LAST
                        /*||  paPages[iPage].idPage == NIL_GMM_PAGEID*/,
                        ("#%#x: %#x\n", iPage, paPages[iPage].idPage), VERR_INVALID_PARAMETER);

    /*
     * Take the semaphore and call the worker function.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        rc = gmmR0FreePages(pGMM, pGVM, cPages, paPages, enmAccount);
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;
    gmmR0MutexRelease(pGMM);
    LogFlow(("GMMR0FreePages: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0FreePages.
 *
 * @returns see GMMR0FreePages.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0FreePagesReq(PGVM pGVM, VMCPUID idCpu, PGMMFREEPAGESREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq >= RT_UOFFSETOF(GMMFREEPAGESREQ, aPages[0]),
                    ("%#x < %#x\n", pReq->Hdr.cbReq, RT_UOFFSETOF(GMMFREEPAGESREQ, aPages[0])),
                    VERR_INVALID_PARAMETER);
    AssertMsgReturn(pReq->Hdr.cbReq == RT_UOFFSETOF_DYN(GMMFREEPAGESREQ, aPages[pReq->cPages]),
                    ("%#x != %#x\n", pReq->Hdr.cbReq, RT_UOFFSETOF_DYN(GMMFREEPAGESREQ, aPages[pReq->cPages])),
                    VERR_INVALID_PARAMETER);

    return GMMR0FreePages(pGVM, idCpu, pReq->cPages, &pReq->aPages[0], pReq->enmAccount);
}


/**
 * Report back on a memory ballooning request.
 *
 * The request may or may not have been initiated by the GMM. If it was initiated
 * by the GMM it is important that this function is called even if no pages were
 * ballooned.
 *
 * @returns VBox status code:
 * @retval  VERR_GMM_ATTEMPT_TO_FREE_TOO_MUCH
 * @retval  VERR_GMM_ATTEMPT_TO_DEFLATE_TOO_MUCH
 * @retval  VERR_GMM_OVERCOMMITTED_TRY_AGAIN_IN_A_BIT - reset condition
 *          indicating that we won't necessarily have sufficient RAM to boot
 *          the VM again and that it should pause until this changes (we'll try
 *          balloon some other VM).  (For standard deflate we have little choice
 *          but to hope the VM won't use the memory that was returned to it.)
 *
 * @param   pGVM                The global (ring-0) VM structure.
 * @param   idCpu               The VCPU id.
 * @param   enmAction           Inflate/deflate/reset.
 * @param   cBalloonedPages     The number of pages that was ballooned.
 *
 * @thread  EMT(idCpu)
 */
//即动态调整客户机（Guest VM）的内存占用，以优化主机（Host）内存利用率
//根据 enmAction 执行 膨胀（Inflate）、收缩（Deflate）或 重置（Reset）内存气球。
/*
参数
  pGVM：目标虚拟机控制块。
  idCpu：当前 CPU ID（用于验证 EMT 线程）。
  enmAction：操作类型（GMMBALLOONACTION_INFLATE / DEFLATE / RESET）。
  cBalloonedPages：要调整的内存页数（以 4KB 页为单位）。
 * */
GMMR0DECL(int) GMMR0BalloonedPages(PGVM pGVM, VMCPUID idCpu, GMMBALLOONACTION enmAction, uint32_t cBalloonedPages)
{
    LogFlow(("GMMR0BalloonedPages: pGVM=%p enmAction=%d cBalloonedPages=%#x\n",
             pGVM, enmAction, cBalloonedPages));

    AssertMsgReturn(cBalloonedPages < RT_BIT(32 - GUEST_PAGE_SHIFT), ("%#x\n", cBalloonedPages), VERR_INVALID_PARAMETER);

    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    //检查当前线程是否为 EMT（Emulation Thread）。
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Take the semaphore and do some more validations.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        switch (enmAction)
        {
            //膨胀（Inflate）
            case GMMBALLOONACTION_INFLATE:
            {
                //检查是否超出预留内存
                //已分配内存 + 气球内存 + 新请求 ≤ 预留内存。
                if (RT_LIKELY(pGVM->gmm.s.Stats.Allocated.cBasePages + pGVM->gmm.s.Stats.cBalloonedPages + cBalloonedPages
                              <= pGVM->gmm.s.Stats.Reserved.cBasePages))
                {
                    /*
                     * Record the ballooned memory.
                     */
                    pGMM->cBalloonedPages += cBalloonedPages;
                    if (pGVM->gmm.s.Stats.cReqBalloonedPages)
                    {
                        /* Codepath never taken. Might be interesting in the future to request ballooned memory from guests in low memory conditions.. */
                        AssertFailed();

                        //全局和 VM 级别的气球内存计数增加。
                        pGVM->gmm.s.Stats.cBalloonedPages            += cBalloonedPages;
                        pGVM->gmm.s.Stats.cReqActuallyBalloonedPages += cBalloonedPages;
                        Log(("GMMR0BalloonedPages: +%#x - Global=%#llx / VM: Total=%#llx Req=%#llx Actual=%#llx (pending)\n",
                             cBalloonedPages, pGMM->cBalloonedPages, pGVM->gmm.s.Stats.cBalloonedPages,
                             pGVM->gmm.s.Stats.cReqBalloonedPages, pGVM->gmm.s.Stats.cReqActuallyBalloonedPages));
                    }
                    else
                    {
                        pGVM->gmm.s.Stats.cBalloonedPages += cBalloonedPages;
                        Log(("GMMR0BalloonedPages: +%#x - Global=%#llx / VM: Total=%#llx (user)\n",
                             cBalloonedPages, pGMM->cBalloonedPages, pGVM->gmm.s.Stats.cBalloonedPages));
                    }
                }
                else
                {
                    Log(("GMMR0BalloonedPages: cBasePages=%#llx Total=%#llx cBalloonedPages=%#llx Reserved=%#llx\n",
                         pGVM->gmm.s.Stats.Allocated.cBasePages, pGVM->gmm.s.Stats.cBalloonedPages, cBalloonedPages,
                         pGVM->gmm.s.Stats.Reserved.cBasePages));
                    //超出预留内存
                    rc = VERR_GMM_ATTEMPT_TO_FREE_TOO_MUCH;
                }
                break;
            }

            //收缩（Deflate）
            case GMMBALLOONACTION_DEFLATE:
            {
                /* Deflate. */
                //检查是否有足够气球内存可释放
                //当前气球内存 ≥ 请求释放的页数。
                if (pGVM->gmm.s.Stats.cBalloonedPages >= cBalloonedPages)
                {
                    /*
                     * Record the ballooned memory.
                     */
                    Assert(pGMM->cBalloonedPages >= cBalloonedPages);
                    //减少全局和 VM 的气球内存计数。
                    pGMM->cBalloonedPages             -= cBalloonedPages;
                    pGVM->gmm.s.Stats.cBalloonedPages -= cBalloonedPages;
                    if (pGVM->gmm.s.Stats.cReqDeflatePages)
                    {
                        AssertFailed(); /* This is path is for later. */
                        Log(("GMMR0BalloonedPages: -%#x - Global=%#llx / VM: Total=%#llx Req=%#llx\n",
                             cBalloonedPages, pGMM->cBalloonedPages, pGVM->gmm.s.Stats.cBalloonedPages, pGVM->gmm.s.Stats.cReqDeflatePages));

                        /*
                         * Anything we need to do here now when the request has been completed?
                         */
                        pGVM->gmm.s.Stats.cReqDeflatePages = 0;
                    }
                    else
                        Log(("GMMR0BalloonedPages: -%#x - Global=%#llx / VM: Total=%#llx (user)\n",
                             cBalloonedPages, pGMM->cBalloonedPages, pGVM->gmm.s.Stats.cBalloonedPages));
                }
                else
                {
                    Log(("GMMR0BalloonedPages: Total=%#llx cBalloonedPages=%#llx\n", pGVM->gmm.s.Stats.cBalloonedPages, cBalloonedPages));
                    //若请求释放的页数超过当前气球内存
                    rc = VERR_GMM_ATTEMPT_TO_DEFLATE_TOO_MUCH;
                }
                break;
            }

            case GMMBALLOONACTION_RESET:
            //强制清空气球内存
            {
                /* Reset to an empty balloon. */
                Assert(pGMM->cBalloonedPages >= pGVM->gmm.s.Stats.cBalloonedPages);

                //将 VM 的气球内存归零，并调整全局计数。
                pGMM->cBalloonedPages             -= pGVM->gmm.s.Stats.cBalloonedPages;
                pGVM->gmm.s.Stats.cBalloonedPages  = 0;
                break;
            }

            default:
                rc = VERR_INVALID_PARAMETER;
                break;
        }
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    LogFlow(("GMMR0BalloonedPages: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0BalloonedPages.
 *
 * @returns see GMMR0BalloonedPages.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0BalloonedPagesReq(PGVM pGVM, VMCPUID idCpu, PGMMBALLOONEDPAGESREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(GMMBALLOONEDPAGESREQ),
                    ("%#x < %#x\n", pReq->Hdr.cbReq, sizeof(GMMBALLOONEDPAGESREQ)),
                    VERR_INVALID_PARAMETER);

    return GMMR0BalloonedPages(pGVM, idCpu, pReq->enmAction, pReq->cBalloonedPages);
}


/**
 * Return memory statistics for the hypervisor
 *
 * @returns VBox status code.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0QueryHypervisorMemoryStatsReq(PGMMMEMSTATSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(GMMMEMSTATSREQ),
                    ("%#x < %#x\n", pReq->Hdr.cbReq, sizeof(GMMMEMSTATSREQ)),
                    VERR_INVALID_PARAMETER);

    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    pReq->cAllocPages     = pGMM->cAllocatedPages;
    pReq->cFreePages      = (pGMM->cChunks << (GMM_CHUNK_SHIFT - GUEST_PAGE_SHIFT)) - pGMM->cAllocatedPages;
    pReq->cBalloonedPages = pGMM->cBalloonedPages;
    pReq->cMaxPages       = pGMM->cMaxPages;
    pReq->cSharedPages    = pGMM->cDuplicatePages;
    GMM_CHECK_SANITY_UPON_LEAVING(pGMM);

    return VINF_SUCCESS;
}


/**
 * Return memory statistics for the VM
 *
 * @returns VBox status code.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       Cpu id.
 * @param   pReq        Pointer to the request packet.
 *
 * @thread  EMT(idCpu)
 */
GMMR0DECL(int) GMMR0QueryMemoryStatsReq(PGVM pGVM, VMCPUID idCpu, PGMMMEMSTATSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(GMMMEMSTATSREQ),
                    ("%#x < %#x\n", pReq->Hdr.cbReq, sizeof(GMMMEMSTATSREQ)),
                    VERR_INVALID_PARAMETER);

    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Take the semaphore and do some more validations.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        pReq->cAllocPages     = pGVM->gmm.s.Stats.Allocated.cBasePages;
        pReq->cBalloonedPages = pGVM->gmm.s.Stats.cBalloonedPages;
        pReq->cMaxPages       = pGVM->gmm.s.Stats.Reserved.cBasePages;
        pReq->cFreePages      = pReq->cMaxPages - pReq->cAllocPages;
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    LogFlow(("GMMR3QueryVMMemoryStats: returns %Rrc\n", rc));
    return rc;
}


/**
 * Worker for gmmR0UnmapChunk and gmmr0FreeChunk.
 *
 * Don't call this in legacy allocation mode!
 *
 * @returns VBox status code.
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the Global VM structure.
 * @param   pChunk      Pointer to the chunk to be unmapped.
 */
static int gmmR0UnmapChunkLocked(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk)
{
    RT_NOREF_PV(pGMM);

    /*
     * Find the mapping and try unmapping it.
     */
    uint32_t cMappings = pChunk->cMappingsX;
    for (uint32_t i = 0; i < cMappings; i++)
    {
        Assert(pChunk->paMappingsX[i].pGVM && pChunk->paMappingsX[i].hMapObj != NIL_RTR0MEMOBJ);
        if (pChunk->paMappingsX[i].pGVM == pGVM)
        {
            /* unmap */
            int rc = RTR0MemObjFree(pChunk->paMappingsX[i].hMapObj, false /* fFreeMappings (NA) */);
            if (RT_SUCCESS(rc))
            {
                /* update the record. */
                cMappings--;
                if (i < cMappings)
                    pChunk->paMappingsX[i] = pChunk->paMappingsX[cMappings];
                pChunk->paMappingsX[cMappings].hMapObj = NIL_RTR0MEMOBJ;
                pChunk->paMappingsX[cMappings].pGVM    = NULL;
                Assert(pChunk->cMappingsX - 1U == cMappings);
                pChunk->cMappingsX = cMappings;
            }

            return rc;
        }
    }

    Log(("gmmR0UnmapChunk: Chunk %#x is not mapped into pGVM=%p/%#x\n", pChunk->Core.Key, pGVM, pGVM->hSelf));
    return VERR_GMM_CHUNK_NOT_MAPPED;
}


/**
 * Unmaps a chunk previously mapped into the address space of the current process.
 *
 * @returns VBox status code.
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the Global VM structure.
 * @param   pChunk      Pointer to the chunk to be unmapped.
 * @param   fRelaxedSem Whether we can release the semaphore while doing the
 *                      mapping (@c true) or not.
 */
static int gmmR0UnmapChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, bool fRelaxedSem)
{
    /*
     * Lock the chunk and if possible leave the giant GMM lock.
     */
    GMMR0CHUNKMTXSTATE MtxState;
    int rc = gmmR0ChunkMutexAcquire(&MtxState, pGMM, pChunk,
                                    fRelaxedSem ? GMMR0CHUNK_MTX_RETAKE_GIANT : GMMR0CHUNK_MTX_KEEP_GIANT);
    if (RT_SUCCESS(rc))
    {
        rc = gmmR0UnmapChunkLocked(pGMM, pGVM, pChunk);
        gmmR0ChunkMutexRelease(&MtxState, pChunk);
    }
    return rc;
}


/**
 * Worker for gmmR0MapChunk.
 *
 * @returns VBox status code.
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the Global VM structure.
 * @param   pChunk      Pointer to the chunk to be mapped.
 * @param   ppvR3       Where to store the ring-3 address of the mapping.
 *                      In the VERR_GMM_CHUNK_ALREADY_MAPPED case, this will be
 *                      contain the address of the existing mapping.
 */
// 将内存块（pChunk）映射到指定虚拟机（pGVM）的用户态地址空间，
// 并返回映射后的虚拟地址
static int gmmR0MapChunkLocked(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, PRTR3PTR ppvR3)
{
    RT_NOREF(pGMM);

    /*
     * Check to see if the chunk is already mapped.
     */
    for (uint32_t i = 0; i < pChunk->cMappingsX; i++)
    {
        Assert(pChunk->paMappingsX[i].pGVM && pChunk->paMappingsX[i].hMapObj != NIL_RTR0MEMOBJ);
        if (pChunk->paMappingsX[i].pGVM == pGVM)
        {
            *ppvR3 = RTR0MemObjAddressR3(pChunk->paMappingsX[i].hMapObj);
            Log(("gmmR0MapChunk: chunk %#x is already mapped at %p!\n", pChunk->Core.Key, *ppvR3));
#ifdef VBOX_WITH_PAGE_SHARING
            //若启用 VBOX_WITH_PAGE_SHARING，允许重复映射（因 R3 缓存可能不同步）。
            /* The ring-3 chunk cache can be out of sync; don't fail. */
            return VINF_SUCCESS;
#else
            return VERR_GMM_CHUNK_ALREADY_MAPPED;
#endif
        }
    }

    /*
     * Do the mapping.
     */
    RTR0MEMOBJ hMapObj;
    //将内存块（pChunk->hMemObj）映射到用户态
    int rc = RTR0MemObjMapUser(&hMapObj, pChunk->hMemObj, (RTR3PTR)-1, 0, RTMEM_PROT_READ | RTMEM_PROT_WRITE, NIL_RTR0PROCESS);
    if (RT_SUCCESS(rc))
    {
         /*
          空间预分配策略：
            小规模优化：若当前映射数 iMapping ≤ 3，仅扩容 1 个条目。
            批量扩容：若映射数为 4 的倍数（(iMapping & 3) == 0），扩容 4 个条目。
        */
        /* reallocate the array? assumes few users per chunk (usually one). */
        unsigned iMapping = pChunk->cMappingsX;
        if (   iMapping <= 3
            || (iMapping & 3) == 0)
        {
            unsigned cNewSize = iMapping <= 3
                              ? iMapping + 1
                              : iMapping + 4;
            Assert(cNewSize < 4 || RT_ALIGN_32(cNewSize, 4) == cNewSize);
            if (RT_UNLIKELY(cNewSize > UINT16_MAX))
            {
                rc = RTR0MemObjFree(hMapObj, false /* fFreeMappings (NA) */); AssertRC(rc);
                return VERR_GMM_TOO_MANY_CHUNK_MAPPINGS;
            }

            void *pvMappings = RTMemRealloc(pChunk->paMappingsX, cNewSize * sizeof(pChunk->paMappingsX[0]));
            if (RT_UNLIKELY(!pvMappings))
            {
                rc = RTR0MemObjFree(hMapObj, false /* fFreeMappings (NA) */); AssertRC(rc);
                return VERR_NO_MEMORY;
            }
            pChunk->paMappingsX = (PGMMCHUNKMAP)pvMappings;
        }

        /* insert new entry */
        //更新映射表
        pChunk->paMappingsX[iMapping].hMapObj = hMapObj;
        pChunk->paMappingsX[iMapping].pGVM    = pGVM;
        Assert(pChunk->cMappingsX == iMapping);
        pChunk->cMappingsX = iMapping + 1;

        //通过 RTR0MemObjAddressR3 获取用户态虚拟地址并写入 ppvR3。
        *ppvR3 = RTR0MemObjAddressR3(hMapObj);
    }

    return rc;
}


/**
 * Maps a chunk into the user address space of the current process.
 *
 * @returns VBox status code.
 * @param   pGMM        Pointer to the GMM instance data.
 * @param   pGVM        Pointer to the Global VM structure.
 * @param   pChunk      Pointer to the chunk to be mapped.
 * @param   fRelaxedSem Whether we can release the semaphore while doing the
 *                      mapping (@c true) or not.
 * @param   ppvR3       Where to store the ring-3 address of the mapping.
 *                      In the VERR_GMM_CHUNK_ALREADY_MAPPED case, this will be
 *                      contain the address of the existing mapping.
 */
static int gmmR0MapChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, bool fRelaxedSem, PRTR3PTR ppvR3)
{
    /*
     * Take the chunk lock and leave the giant GMM lock when possible, then
     * call the worker function.
     */
    GMMR0CHUNKMTXSTATE MtxState;
    int rc = gmmR0ChunkMutexAcquire(&MtxState, pGMM, pChunk,
                                    fRelaxedSem ? GMMR0CHUNK_MTX_RETAKE_GIANT : GMMR0CHUNK_MTX_KEEP_GIANT);
    if (RT_SUCCESS(rc))
    {
        rc = gmmR0MapChunkLocked(pGMM, pGVM, pChunk, ppvR3);
        gmmR0ChunkMutexRelease(&MtxState, pChunk);
    }

    return rc;
}



#if defined(VBOX_WITH_PAGE_SHARING) || defined(VBOX_STRICT)
/**
 * Check if a chunk is mapped into the specified VM
 *
 * @returns mapped yes/no
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        Pointer to the Global VM structure.
 * @param   pChunk      Pointer to the chunk to be mapped.
 * @param   ppvR3       Where to store the ring-3 address of the mapping.
 */
// 验证指定内存块（pChunk）是否已映射到目标虚拟机（pGVM）的地址空间
// 并返回映射后的主机虚拟地址（ppvR3）
static bool gmmR0IsChunkMapped(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, PRTR3PTR ppvR3)
{
    GMMR0CHUNKMTXSTATE MtxState;
    gmmR0ChunkMutexAcquire(&MtxState, pGMM, pChunk, GMMR0CHUNK_MTX_KEEP_GIANT);
    //遍历内存块的扩展映射表
    for (uint32_t i = 0; i < pChunk->cMappingsX; i++)
    {
        Assert(pChunk->paMappingsX[i].pGVM && pChunk->paMappingsX[i].hMapObj != NIL_RTR0MEMOBJ);
        if (pChunk->paMappingsX[i].pGVM == pGVM)
        {
            *ppvR3 = RTR0MemObjAddressR3(pChunk->paMappingsX[i].hMapObj);
            gmmR0ChunkMutexRelease(&MtxState, pChunk);
            return true;
        }
    }
    *ppvR3 = NULL;
    gmmR0ChunkMutexRelease(&MtxState, pChunk);
    return false;
}
#endif /* VBOX_WITH_PAGE_SHARING || VBOX_STRICT */


/**
 * Map a chunk and/or unmap another chunk.
 *
 * The mapping and unmapping applies to the current process.
 *
 * This API does two things because it saves a kernel call per mapping when
 * when the ring-3 mapping cache is full.
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idChunkMap      The chunk to map. NIL_GMM_CHUNKID if nothing to map.
 * @param   idChunkUnmap    The chunk to unmap. NIL_GMM_CHUNKID if nothing to unmap.
 * @param   ppvR3           Where to store the address of the mapped chunk. NULL is ok if nothing to map.
 * @thread  EMT ???
 */
//核心逻辑是 原子化地处理内存块的映射（Map）
//与解除映射（Unmap），确保客户机内存管理的安全性和一致性
GMMR0DECL(int) GMMR0MapUnmapChunk(PGVM pGVM, uint32_t idChunkMap, uint32_t idChunkUnmap, PRTR3PTR ppvR3)
{
    LogFlow(("GMMR0MapUnmapChunk: pGVM=%p idChunkMap=%#x idChunkUnmap=%#x ppvR3=%p\n",
             pGVM, idChunkMap, idChunkUnmap, ppvR3));

    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);//通过 GMM_GET_VALID_INSTANCE 验证全局内存管理器（pGMM）有效性。
    int rc = GVMMR0ValidateGVM(pGVM);
    if (RT_FAILURE(rc))
        return rc;

    AssertCompile(NIL_GMM_CHUNKID == 0);
    AssertMsgReturn(idChunkMap <= GMM_CHUNKID_LAST, ("%#x\n", idChunkMap), VERR_INVALID_PARAMETER);
    AssertMsgReturn(idChunkUnmap <= GMM_CHUNKID_LAST, ("%#x\n", idChunkUnmap), VERR_INVALID_PARAMETER);

    if (    idChunkMap == NIL_GMM_CHUNKID
        &&  idChunkUnmap == NIL_GMM_CHUNKID)
        return VERR_INVALID_PARAMETER;

    if (idChunkMap != NIL_GMM_CHUNKID)
    {
        AssertPtrReturn(ppvR3, VERR_INVALID_POINTER);
        *ppvR3 = NIL_RTR3PTR;
    }

    /*
     * Take the semaphore and do the work.
     *
     * The unmapping is done last since it's easier to undo a mapping than
     * undoing an unmapping. The ring-3 mapping cache cannot not be so big
     * that it pushes the user virtual address space to within a chunk of
     * it it's limits, so, no problem here.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        PGMMCHUNK pMap = NULL;
        if (idChunkMap != NIL_GVM_HANDLE)
        {
            //调用 gmmR0GetChunk 根据 idChunkMap 获取块描述符 pMap
            pMap = gmmR0GetChunk(pGMM, idChunkMap);
            //若块有效（pMap != NULL），调用 gmmR0MapChunk 将其映射到客户机地址空间，并通过 ppvR3 返回映射后的主机虚拟地址。
            if (RT_LIKELY(pMap))
                rc = gmmR0MapChunk(pGMM, pGVM, pMap, true /*fRelaxedSem*/, ppvR3);
            else
            {
                Log(("GMMR0MapUnmapChunk: idChunkMap=%#x\n", idChunkMap));
                rc = VERR_GMM_CHUNK_NOT_FOUND;
            }
        }
/** @todo split this operation, the bail out might (theoretcially) not be
 *        entirely safe. */

        if (    idChunkUnmap != NIL_GMM_CHUNKID
            &&  RT_SUCCESS(rc))
        {
            //通过 gmmR0GetChunk 获取 idChunkUnmap 对应的块 pUnmap
            PGMMCHUNK pUnmap = gmmR0GetChunk(pGMM, idChunkUnmap);
            if (RT_LIKELY(pUnmap))
                //若块有效，调用 gmmR0UnmapChunk 解除映射。
                rc = gmmR0UnmapChunk(pGMM, pGVM, pUnmap, true /*fRelaxedSem*/);
            else
            {
                Log(("GMMR0MapUnmapChunk: idChunkUnmap=%#x\n", idChunkUnmap));
                rc = VERR_GMM_CHUNK_NOT_FOUND;
            }


            ///若块无效或卸载失败，回滚已完成的映射操作（若存在）
            if (RT_FAILURE(rc) && pMap)
                gmmR0UnmapChunk(pGMM, pGVM, pMap, false /*fRelaxedSem*/);
        }

        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;
    gmmR0MutexRelease(pGMM);

    LogFlow(("GMMR0MapUnmapChunk: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0MapUnmapChunk.
 *
 * @returns see GMMR0MapUnmapChunk.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int)  GMMR0MapUnmapChunkReq(PGVM pGVM, PGMMMAPUNMAPCHUNKREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GMMR0MapUnmapChunk(pGVM, pReq->idChunkMap, pReq->idChunkUnmap, &pReq->pvR3);
}


#ifndef VBOX_WITH_LINEAR_HOST_PHYS_MEM
/**
 * Gets the ring-0 virtual address for the given page.
 *
 * This is used by PGM when IEM and such wants to access guest RAM from ring-0.
 * One of the ASSUMPTIONS here is that the @a idPage is used by the VM and the
 * corresponding chunk will remain valid beyond the call (at least till the EMT
 * returns to ring-3).
 *
 * @returns VBox status code.
 * @param   pGVM        Pointer to the kernel-only VM instace data.
 * @param   idPage      The page ID.
 * @param   ppv         Where to store the address.
 * @thread  EMT
 */
//将 客户机物理页 ID（idPage）转换为 主机虚拟地址（ppv），同时确保页所有权和访问安全性。
GMMR0DECL(int)  GMMR0PageIdToVirt(PGVM pGVM, uint32_t idPage, void **ppv)
{
    *ppv = NULL;
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);

    //提取 内存块 ID
    uint32_t const idChunk = idPage >> GMM_CHUNKID_SHIFT;

    /*
     * Start with the per-VM TLB.
     */
    RTSpinlockAcquire(pGVM->gmm.s.hChunkTlbSpinLock);

    //首先检查 VM 本地 TLB（pGVM->gmm.s.aChunkTlbEntries），
    //若命中（pChunk 有效且代际匹配 idGeneration），直接复用缓存结果，减少全局锁竞争。
    PGMMPERVMCHUNKTLBE pTlbe = &pGVM->gmm.s.aChunkTlbEntries[GMMPERVM_CHUNKTLB_IDX(idChunk)];
    PGMMCHUNK pChunk = pTlbe->pChunk;
    if (   pChunk              != NULL
        && pTlbe->idGeneration == ASMAtomicUoReadU64(&pGMM->idFreeGeneration)
        && pChunk->Core.Key    == idChunk)
        pGVM->R0Stats.gmm.cChunkTlbHits++; /* hopefully this is a likely outcome */
    else
        //若未命中，需全局锁（pGMM->hSpinLockTree）保护下查询 全局内存块树（gmmR0GetChunkLocked），
        //并更新 TLB 条目（pTlbe->pChunk 和 idGeneration）。
    {
        pGVM->R0Stats.gmm.cChunkTlbMisses++;

        /*
         * Look it up in the chunk tree.
         */
        RTSpinlockAcquire(pGMM->hSpinLockTree);
        pChunk = gmmR0GetChunkLocked(pGMM, idChunk);
        if (RT_LIKELY(pChunk))
        {
            pTlbe->idGeneration = pGMM->idFreeGeneration;
            RTSpinlockRelease(pGMM->hSpinLockTree);
            pTlbe->pChunk       = pChunk;
        }
        else
        {
            RTSpinlockRelease(pGMM->hSpinLockTree);
            RTSpinlockRelease(pGVM->gmm.s.hChunkTlbSpinLock);
            AssertMsgFailed(("idPage=%#x\n", idPage));
            return VERR_GMM_PAGE_NOT_FOUND;
        }
    }

    RTSpinlockRelease(pGVM->gmm.s.hChunkTlbSpinLock);

    /*
     * Got a chunk, now validate the page ownership and calcuate it's address.
     */
    //idPage & GMM_PAGEID_IDX_MASK 提取 页在块内的索引
    const GMMPAGE * const pPage = &pChunk->aPages[idPage & GMM_PAGEID_IDX_MASK];
    if (RT_LIKELY(   (   GMM_PAGE_IS_PRIVATE(pPage)
                      && pPage->Private.hGVM == pGVM->hSelf)
                  || GMM_PAGE_IS_SHARED(pPage)))
    {
        AssertPtr(pChunk->pbMapping);
        //通过块映射基地址（pChunk->pbMapping）和页索引偏移量（<< GUEST_PAGE_SHIFT）得到主机虚拟地址
        *ppv = &pChunk->pbMapping[(idPage & GMM_PAGEID_IDX_MASK) << GUEST_PAGE_SHIFT];
        return VINF_SUCCESS;
    }
    AssertMsgFailed(("idPage=%#x is-private=%RTbool Private.hGVM=%u pGVM->hGVM=%u\n",
                     idPage, GMM_PAGE_IS_PRIVATE(pPage), pPage->Private.hGVM, pGVM->hSelf));
    return VERR_GMM_NOT_PAGE_OWNER;
}
#endif /* !VBOX_WITH_LINEAR_HOST_PHYS_MEM */

#ifdef VBOX_WITH_PAGE_SHARING

# ifdef VBOX_STRICT
/**
 * For checksumming shared pages in strict builds.
 *
 * The purpose is making sure that a page doesn't change.
 *
 * @returns Checksum, 0 on failure.
 * @param   pGMM        The GMM instance data.
 * @param   pGVM        Pointer to the kernel-only VM instace data.
 * @param   idPage      The page ID.
 */
//通过计算客户机内存页的 CRC32 校验和来验证页面内容的完整性
static uint32_t gmmR0StrictPageChecksum(PGMM pGMM, PGVM pGVM, uint32_t idPage)
{
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
    AssertMsgReturn(pChunk, ("idPage=%#x\n", idPage), 0);

    uint8_t *pbChunk;
    // 验证内存块是否已映射到当前 VM 的地址空间
    if (!gmmR0IsChunkMapped(pGMM, pGVM, pChunk, (PRTR3PTR)&pbChunk))
        return 0;
    uint8_t const *pbPage = pbChunk + ((idPage & GMM_PAGEID_IDX_MASK) << GUEST_PAGE_SHIFT);

    return RTCrc32(pbPage, GUEST_PAGE_SIZE);
}
# endif /* VBOX_STRICT */


/**
 * Calculates the module hash value.
 *
 * @returns Hash value.
 * @param   pszModuleName   The module name.
 * @param   pszVersion      The module version string.
 */
static uint32_t gmmR0ShModCalcHash(const char *pszModuleName, const char *pszVersion)
{
    return RTStrHash1ExN(3, pszModuleName, RTSTR_MAX, "::", (size_t)2, pszVersion, RTSTR_MAX);
}


/**
 * Finds a global module.
 *
 * @returns Pointer to the global module on success, NULL if not found.
 * @param   pGMM            The GMM instance data.
 * @param   uHash           The hash as calculated by gmmR0ShModCalcHash.
 * @param   cbModule        The module size.
 * @param   enmGuestOS      The guest OS type.
 * @param   cRegions        The number of regions.
 * @param   pszModuleName   The module name.
 * @param   pszVersion      The module version.
 * @param   paRegions       The region descriptions.
 */
static PGMMSHAREDMODULE gmmR0ShModFindGlobal(PGMM pGMM, uint32_t uHash, uint32_t cbModule, VBOXOSFAMILY enmGuestOS,
                                             uint32_t cRegions, const char *pszModuleName, const char *pszVersion,
                                             struct VMMDEVSHAREDREGIONDESC const *paRegions)
{
    for (PGMMSHAREDMODULE pGblMod = (PGMMSHAREDMODULE)RTAvllU32Get(&pGMM->pGlobalSharedModuleTree, uHash);
         pGblMod;
         pGblMod = (PGMMSHAREDMODULE)pGblMod->Core.pList)
    {
        if (pGblMod->cbModule   != cbModule)
            continue;
        if (pGblMod->enmGuestOS != enmGuestOS)
            continue;
        if (pGblMod->cRegions   != cRegions)
            continue;
        if (strcmp(pGblMod->szName, pszModuleName))
            continue;
        if (strcmp(pGblMod->szVersion, pszVersion))
            continue;

        uint32_t i;
        for (i = 0; i < cRegions; i++)
        {
            uint32_t off = paRegions[i].GCRegionAddr & GUEST_PAGE_OFFSET_MASK;
            if (pGblMod->aRegions[i].off != off)
                break;

            uint32_t cb  = RT_ALIGN_32(paRegions[i].cbRegion + off, GUEST_PAGE_SIZE);
            if (pGblMod->aRegions[i].cb != cb)
                break;
        }

        if (i == cRegions)
            return pGblMod;
    }

    return NULL;
}


/**
 * Creates a new global module.
 *
 * @returns VBox status code.
 * @param   pGMM            The GMM instance data.
 * @param   uHash           The hash as calculated by gmmR0ShModCalcHash.
 * @param   cbModule        The module size.
 * @param   enmGuestOS      The guest OS type.
 * @param   cRegions        The number of regions.
 * @param   pszModuleName   The module name.
 * @param   pszVersion      The module version.
 * @param   paRegions       The region descriptions.
 * @param   ppGblMod        Where to return the new module on success.
 */
static int gmmR0ShModNewGlobal(PGMM pGMM, uint32_t uHash, uint32_t cbModule, VBOXOSFAMILY enmGuestOS,
                               uint32_t cRegions, const char *pszModuleName, const char *pszVersion,
                               struct VMMDEVSHAREDREGIONDESC const *paRegions, PGMMSHAREDMODULE *ppGblMod)
{
    Log(("gmmR0ShModNewGlobal: %s %s size %#x os %u rgn %u\n", pszModuleName, pszVersion, cbModule, enmGuestOS, cRegions));
    if (pGMM->cShareableModules >= GMM_MAX_SHARED_GLOBAL_MODULES)
    {
        Log(("gmmR0ShModNewGlobal: Too many modules\n"));
        return VERR_GMM_TOO_MANY_GLOBAL_MODULES;
    }

    PGMMSHAREDMODULE pGblMod = (PGMMSHAREDMODULE)RTMemAllocZ(RT_UOFFSETOF_DYN(GMMSHAREDMODULE, aRegions[cRegions]));
    if (!pGblMod)
    {
        Log(("gmmR0ShModNewGlobal: No memory\n"));
        return VERR_NO_MEMORY;
    }

    pGblMod->Core.Key   = uHash;
    pGblMod->cbModule   = cbModule;
    pGblMod->cRegions   = cRegions;
    pGblMod->cUsers     = 1;
    pGblMod->enmGuestOS = enmGuestOS;
    strcpy(pGblMod->szName, pszModuleName);
    strcpy(pGblMod->szVersion, pszVersion);

    for (uint32_t i = 0; i < cRegions; i++)
    {
        Log(("gmmR0ShModNewGlobal: rgn[%u]=%RGvLB%#x\n", i, paRegions[i].GCRegionAddr, paRegions[i].cbRegion));
        pGblMod->aRegions[i].off        = paRegions[i].GCRegionAddr & GUEST_PAGE_OFFSET_MASK;
        pGblMod->aRegions[i].cb         = paRegions[i].cbRegion + pGblMod->aRegions[i].off;
        pGblMod->aRegions[i].cb         = RT_ALIGN_32(pGblMod->aRegions[i].cb, GUEST_PAGE_SIZE);
        pGblMod->aRegions[i].paidPages  = NULL; /* allocated when needed. */
    }

    bool fInsert = RTAvllU32Insert(&pGMM->pGlobalSharedModuleTree, &pGblMod->Core);
    Assert(fInsert); NOREF(fInsert);
    pGMM->cShareableModules++;

    *ppGblMod = pGblMod;
    return VINF_SUCCESS;
}


/**
 * Deletes a global module which is no longer referenced by anyone.
 *
 * @param   pGMM                The GMM instance data.
 * @param   pGblMod             The module to delete.
 */
static void gmmR0ShModDeleteGlobal(PGMM pGMM, PGMMSHAREDMODULE pGblMod)
{
    Assert(pGblMod->cUsers == 0);
    Assert(pGMM->cShareableModules > 0 && pGMM->cShareableModules <= GMM_MAX_SHARED_GLOBAL_MODULES);

    void *pvTest = RTAvllU32RemoveNode(&pGMM->pGlobalSharedModuleTree, &pGblMod->Core);
    Assert(pvTest == pGblMod); NOREF(pvTest);
    pGMM->cShareableModules--;

    uint32_t i = pGblMod->cRegions;
    while (i-- > 0)
    {
        if (pGblMod->aRegions[i].paidPages)
        {
            /* We don't doing anything to the pages as they are handled by the
               copy-on-write mechanism in PGM. */
            RTMemFree(pGblMod->aRegions[i].paidPages);
            pGblMod->aRegions[i].paidPages = NULL;
        }
    }
    RTMemFree(pGblMod);
}


//为单个 VM 注册共享模块的 VM 级记录
static int gmmR0ShModNewPerVM(PGVM pGVM, RTGCPTR GCBaseAddr, uint32_t cRegions, const VMMDEVSHAREDREGIONDESC *paRegions,
                              PGMMSHAREDMODULEPERVM *ppRecVM)
{
    if (pGVM->gmm.s.Stats.cShareableModules >= GMM_MAX_SHARED_PER_VM_MODULES)
        return VERR_GMM_TOO_MANY_PER_VM_MODULES;

    PGMMSHAREDMODULEPERVM pRecVM;
    pRecVM = (PGMMSHAREDMODULEPERVM)RTMemAllocZ(RT_UOFFSETOF_DYN(GMMSHAREDMODULEPERVM, aRegionsGCPtrs[cRegions]));
    if (!pRecVM)
        return VERR_NO_MEMORY;

    //将输入参数 GCBaseAddr 作为 AVL 树节点的键值（Core.Key）
    pRecVM->Core.Key = GCBaseAddr;
    for (uint32_t i = 0; i < cRegions; i++)
        //遍历 paRegions 数组，将每个区域的客户机物理地址（GCRegionAddr）
        //复制到 aRegionsGCPtrs 中
        pRecVM->aRegionsGCPtrs[i] = paRegions[i].GCRegionAddr;

    //将新节点插入 VM 的共享模块树
    bool fInsert = RTAvlGCPtrInsert(&pGVM->gmm.s.pSharedModuleTree, &pRecVM->Core);
    Assert(fInsert); NOREF(fInsert);
    pGVM->gmm.s.Stats.cShareableModules++;

    *ppRecVM = pRecVM;
    return VINF_SUCCESS;
}


static void gmmR0ShModDeletePerVM(PGMM pGMM, PGVM pGVM, PGMMSHAREDMODULEPERVM pRecVM, bool fRemove)
{
    /*
     * Free the per-VM module.
     */
    PGMMSHAREDMODULE pGblMod = pRecVM->pGlobalModule;
    pRecVM->pGlobalModule    = NULL;

    if (fRemove)
    {
        void *pvTest = RTAvlGCPtrRemove(&pGVM->gmm.s.pSharedModuleTree, pRecVM->Core.Key);
        Assert(pvTest == &pRecVM->Core); NOREF(pvTest);
    }

    RTMemFree(pRecVM);

    /*
     * Release the global module.
     * (In the registration bailout case, it might not be.)
     */
    if (pGblMod)
    {
        Assert(pGblMod->cUsers > 0);
        pGblMod->cUsers--;
        if (pGblMod->cUsers == 0)
            gmmR0ShModDeleteGlobal(pGMM, pGblMod);
    }
}

#endif /* VBOX_WITH_PAGE_SHARING */

/**
 * Registers a new shared module for the VM.
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idCpu           The VCPU id.
 * @param   enmGuestOS      The guest OS type.
 * @param   pszModuleName   The module name.
 * @param   pszVersion      The module version.
 * @param   GCPtrModBase    The module base address.
 * @param   cbModule        The module size.
 * @param   cRegions        The mumber of shared region descriptors.
 * @param   paRegions       Pointer to an array of shared region(s).
 * @thread  EMT(idCpu)
 */
/*
用于注册和管理共享模块，以支持内存页共享（Page Sharing）功能。该函数的主要职责包括：
  验证输入参数（模块名称、版本、内存区域等）。
  检查模块是否已注册（避免重复注册）。
  管理全局和 VM 级别的模块记录
  处理模块哈希匹配，以支持跨 VM 的页面去重（Page Deduplication）。
*/
/*
 virtualbox的VM在共享相同OS的时候，镜像大小通常大于1G， GMMR0RegisterSharedModule 是如何处理的
   模块大小限制：函数会验证模块大小（cbModule ≤ 1GB），但实际支持更大的模块通过分区域处理（cRegions 和 paRegions 描述分段）
   区域校验：每个内存区域（paRegions[i].cbRegion）需满足 ≤ 1GB，总和可超过 1GB，但需通过循环累加校验（cbTotal）
 * */
GMMR0DECL(int) GMMR0RegisterSharedModule(PGVM pGVM, VMCPUID idCpu, VBOXOSFAMILY enmGuestOS, char *pszModuleName,
                                         char *pszVersion, RTGCPTR GCPtrModBase, uint32_t cbModule,
                                         uint32_t cRegions, struct VMMDEVSHAREDREGIONDESC const *paRegions)
{
#ifdef VBOX_WITH_PAGE_SHARING
    /*
     * Validate input and get the basics.
     *
     * Note! Turns out the module size does necessarily match the size of the
     *       regions. (iTunes on XP)
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

    /* 检查区域数量是否超过限制 */
    if (RT_UNLIKELY(cRegions > VMMDEVSHAREDREGIONDESC_MAX))
        return VERR_GMM_TOO_MANY_REGIONS;

    /* 检查模块大小是否合法 */
    if (RT_UNLIKELY(cbModule == 0 || cbModule > _1G))
        return VERR_GMM_BAD_SHARED_MODULE_SIZE;

    /* 检查每个区域的大小是否合法 */
    uint32_t cbTotal = 0;
    for (uint32_t i = 0; i < cRegions; i++)
    {
        if (RT_UNLIKELY(paRegions[i].cbRegion == 0 || paRegions[i].cbRegion > _1G))
            return VERR_GMM_SHARED_MODULE_BAD_REGIONS_SIZE;

        cbTotal += paRegions[i].cbRegion;
        if (RT_UNLIKELY(cbTotal > _1G))
            return VERR_GMM_SHARED_MODULE_BAD_REGIONS_SIZE;
    }

    /* 检查模块名称和版本字符串是否合法 */
    AssertPtrReturn(pszModuleName, VERR_INVALID_POINTER);
    if (RT_UNLIKELY(!memchr(pszModuleName, '\0', GMM_SHARED_MODULE_MAX_NAME_STRING)))
        return VERR_GMM_MODULE_NAME_TOO_LONG;

    AssertPtrReturn(pszVersion, VERR_INVALID_POINTER);
    if (RT_UNLIKELY(!memchr(pszVersion, '\0', GMM_SHARED_MODULE_MAX_VERSION_STRING)))
        return VERR_GMM_MODULE_NAME_TOO_LONG;

    //计算模块名称和版本的哈希值，用于全局模块匹配。
    uint32_t const uHash = gmmR0ShModCalcHash(pszModuleName, pszVersion);
    Log(("GMMR0RegisterSharedModule %s %s base %RGv size %x hash %x\n", pszModuleName, pszVersion, GCPtrModBase, cbModule, uHash));

    /*
     * Take the semaphore and do some more validations.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        /*
         * Check if this module is already locally registered and register
         * it if it isn't.  The base address is a unique module identifier
         * locally.
         */
        //在 VM 的共享模块树中查找是否已注册相同基地址的模块。
        PGMMSHAREDMODULEPERVM pRecVM = (PGMMSHAREDMODULEPERVM)RTAvlGCPtrGet(&pGVM->gmm.s.pSharedModuleTree, GCPtrModBase);
        bool fNewModule = pRecVM == NULL;
        //处理新模块注册
        if (fNewModule)
        {
            rc = gmmR0ShModNewPerVM(pGVM, GCPtrModBase, cRegions, paRegions, &pRecVM);
            if (RT_SUCCESS(rc))
            {
                /*
                 * Find a matching global module, register a new one if needed.
                 */
                /* 查找全局匹配的模块 */
                PGMMSHAREDMODULE pGblMod = gmmR0ShModFindGlobal(pGMM, uHash, cbModule, enmGuestOS, cRegions,
                                                                pszModuleName, pszVersion, paRegions);
                if (!pGblMod)
                {
                    Assert(fNewModule);
                    rc = gmmR0ShModNewGlobal(pGMM, uHash, cbModule, enmGuestOS, cRegions,
                                             pszModuleName, pszVersion, paRegions, &pGblMod);
                    if (RT_SUCCESS(rc))
                    {
                        pRecVM->pGlobalModule = pGblMod; /* (One referenced returned by gmmR0ShModNewGlobal.) */
                        Log(("GMMR0RegisterSharedModule: new module %s %s\n", pszModuleName, pszVersion));
                    }
                    else
                        gmmR0ShModDeletePerVM(pGMM, pGVM, pRecVM, true /*fRemove*/);
                }
                else
                {
                    /* 全局模块已存在，增加引用计数 */
                    Assert(pGblMod->cUsers > 0 && pGblMod->cUsers < UINT32_MAX / 2);
                    pGblMod->cUsers++;
                    pRecVM->pGlobalModule = pGblMod;

                    Log(("GMMR0RegisterSharedModule: new per vm module %s %s, gbl users %d\n", pszModuleName, pszVersion, pGblMod->cUsers));
                }
            }
        }
        else
        {
            /*
             * Attempt to re-register an existing module.
             */
            /* 检查是否与全局模块匹配 */
            PGMMSHAREDMODULE pGblMod = gmmR0ShModFindGlobal(pGMM, uHash, cbModule, enmGuestOS, cRegions,
                                                            pszModuleName, pszVersion, paRegions);
            if (pRecVM->pGlobalModule == pGblMod)
            {
                Log(("GMMR0RegisterSharedModule: already registered %s %s, gbl users %d\n", pszModuleName, pszVersion, pGblMod->cUsers));
                rc = VINF_GMM_SHARED_MODULE_ALREADY_REGISTERED;
            }
            else
            {
                /** @todo may have to unregister+register when this happens in case it's caused
                 * by VBoxService crashing and being restarted... */
                 /* 地址冲突！可能是 VBoxService 崩溃后重新启动 */
                Log(("GMMR0RegisterSharedModule: Address clash!\n"
                     "  incoming at %RGvLB%#x %s %s rgns %u\n"
                     "  existing at %RGvLB%#x %s %s rgns %u\n",
                     GCPtrModBase, cbModule, pszModuleName, pszVersion, cRegions,
                     pRecVM->Core.Key, pRecVM->pGlobalModule->cbModule, pRecVM->pGlobalModule->szName,
                     pRecVM->pGlobalModule->szVersion, pRecVM->pGlobalModule->cRegions));
                rc = VERR_GMM_SHARED_MODULE_ADDRESS_CLASH;
            }
        }
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    return rc;
#else

    NOREF(pGVM); NOREF(idCpu); NOREF(enmGuestOS); NOREF(pszModuleName); NOREF(pszVersion);
    NOREF(GCPtrModBase); NOREF(cbModule); NOREF(cRegions); NOREF(paRegions);
    return VERR_NOT_IMPLEMENTED;
#endif
}


/**
 * VMMR0 request wrapper for GMMR0RegisterSharedModule.
 *
 * @returns see GMMR0RegisterSharedModule.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0RegisterSharedModuleReq(PGVM pGVM, VMCPUID idCpu, PGMMREGISTERSHAREDMODULEREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(   pReq->Hdr.cbReq >= sizeof(*pReq)
                    && pReq->Hdr.cbReq == RT_UOFFSETOF_DYN(GMMREGISTERSHAREDMODULEREQ, aRegions[pReq->cRegions]),
                    ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    /* Pass back return code in the request packet to preserve informational codes. (VMMR3CallR0 chokes on them) */
    pReq->rc = GMMR0RegisterSharedModule(pGVM, idCpu, pReq->enmGuestOS, pReq->szName, pReq->szVersion,
                                         pReq->GCBaseAddr, pReq->cbModule, pReq->cRegions, pReq->aRegions);
    return VINF_SUCCESS;
}


/**
 * Unregisters a shared module for the VM
 *
 * @returns VBox status code.
 * @param   pGVM            The global (ring-0) VM structure.
 * @param   idCpu           The VCPU id.
 * @param   pszModuleName   The module name.
 * @param   pszVersion      The module version.
 * @param   GCPtrModBase    The module base address.
 * @param   cbModule        The module size.
 */
/*  注销虚拟机（VM）的共享模块 */
GMMR0DECL(int) GMMR0UnregisterSharedModule(PGVM pGVM, VMCPUID idCpu, char *pszModuleName, char *pszVersion,
                                           RTGCPTR GCPtrModBase, uint32_t cbModule)
{
#ifdef VBOX_WITH_PAGE_SHARING
    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE); // 获取GMM实例
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu); // 验证VM和CPU ID有效性
    if (RT_FAILURE(rc))
        return rc;

    AssertPtrReturn(pszModuleName, VERR_INVALID_POINTER);// 模块名称非空
    AssertPtrReturn(pszVersion, VERR_INVALID_POINTER);
    if (RT_UNLIKELY(!memchr(pszModuleName, '\0', GMM_SHARED_MODULE_MAX_NAME_STRING)))
        return VERR_GMM_MODULE_NAME_TOO_LONG;// 检查名称长度
    if (RT_UNLIKELY(!memchr(pszVersion, '\0', GMM_SHARED_MODULE_MAX_VERSION_STRING)))
        return VERR_GMM_MODULE_NAME_TOO_LONG; // 检查版本长度

    Log(("GMMR0UnregisterSharedModule %s %s base=%RGv size %x\n", pszModuleName, pszVersion, GCPtrModBase, cbModule));

    /*
     * Take the semaphore and do some more validations.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        /*
         * Locate and remove the specified module.
         */
        PGMMSHAREDMODULEPERVM pRecVM = (PGMMSHAREDMODULEPERVM)RTAvlGCPtrGet(&pGVM->gmm.s.pSharedModuleTree, GCPtrModBase);
        if (pRecVM)
        {
            /** @todo Do we need to do more validations here, like that the
             *        name + version + cbModule matches? */
            NOREF(cbModule);
            Assert(pRecVM->pGlobalModule);
            /*
             * 查找逻辑：基于模块基地址（GCPtrModBase）在AVL树中快速定位。
            删除操作：调用 gmmR0ShModDeletePerVM 清理VM与模块的关联：
              减少全局模块的引用计数。
              若引用归零，释放全局模块描述符。
            */
            gmmR0ShModDeletePerVM(pGMM, pGVM, pRecVM, true /*fRemove*/);// 执行删除
        }
        else
            rc = VERR_GMM_SHARED_MODULE_NOT_FOUND;

        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    return rc;
#else

    NOREF(pGVM); NOREF(idCpu); NOREF(pszModuleName); NOREF(pszVersion); NOREF(GCPtrModBase); NOREF(cbModule);
    return VERR_NOT_IMPLEMENTED;
#endif
}


/**
 * VMMR0 request wrapper for GMMR0UnregisterSharedModule.
 *
 * @returns see GMMR0UnregisterSharedModule.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int)  GMMR0UnregisterSharedModuleReq(PGVM pGVM, VMCPUID idCpu, PGMMUNREGISTERSHAREDMODULEREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GMMR0UnregisterSharedModule(pGVM, idCpu, pReq->szName, pReq->szVersion, pReq->GCBaseAddr, pReq->cbModule);
}

#ifdef VBOX_WITH_PAGE_SHARING

/**
 * Increase the use count of a shared page, the page is known to exist and be valid and such.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        Pointer to the GVM instance.
 * @param   pPage       The page structure.
 */
/*
示例场景
  场景：3个VM运行相同的应用程序：
    VM1 加载应用代码页 → 转为共享页（cRefs=1）。
    VM2/VM3 加载相同代码页 → 分别调用 gmmR0UseSharedPage：
    cRefs 递增至3。
    cDuplicatePages 增加2（节省2个物理页）。
  统计结果：
    pGMM->cSharedPages=1（1个共享页）。
    pGMM->cDuplicatePages=2（节省2页内存）。
    每个VM的 cSharedPages 和 cBasePages 各+1。
*/
DECLINLINE(void) gmmR0UseSharedPage(PGMM pGMM, PGVM pGVM, PGMMPAGE pPage)
{
    Assert(pGMM->cSharedPages > 0);
    Assert(pGMM->cAllocatedPages > 0);

    pGMM->cDuplicatePages++;//统计因共享而节省的物理页数。

    pPage->Shared.cRefs++; //共享页的引用计数，表示当前被多少VM使用。
    pGVM->gmm.s.Stats.cSharedPages++; //当前VM使用的共享页数+1。
    pGVM->gmm.s.Stats.Allocated.cBasePages++;//当前VM的总分配页数+1（共享页也计入VM的内存占用）。
}


/**
 * Converts a private page to a shared page, the page is known to exist and be valid and such.
 *
 * @param   pGMM        Pointer to the GMM instance.
 * @param   pGVM        Pointer to the GVM instance.
 * @param   HCPhys      Host physical address
 * @param   idPage      The Page ID
 * @param   pPage       The page structure.
 * @param   pPageDesc   Shared page descriptor
 */
/*
负责将私有内存页（Private Page）转换为共享页（Shared Page），实现多虚拟机间的内存去重（Deduplication）。其核心操作包括：
  状态转换：私有页 → 共享页
  引用计数初始化：标记首次共享的VM
  统计信息更新：全局和VM级别的计数器维护
  调试支持：可选的内存校验和验证
*/
DECLINLINE(void) gmmR0ConvertToSharedPage(PGMM pGMM, PGVM pGVM, RTHCPHYS HCPhys, uint32_t idPage, PGMMPAGE pPage,
                                          PGMMSHAREDPAGEDESC pPageDesc)
{
    //获取所属内存块
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, idPage >> GMM_CHUNKID_SHIFT);
    Assert(pChunk);
    Assert(pChunk->cFree < GMM_CHUNK_NUM_PAGES);
    Assert(GMM_PAGE_IS_PRIVATE(pPage));

    pChunk->cPrivate--; //内存块私有页数-1
    pChunk->cShared++; //内存块共享页数+1

    pGMM->cSharedPages++;// 全局共享页总数+1

    pGVM->gmm.s.Stats.cSharedPages++; // 当前VM共享页数+1
    pGVM->gmm.s.Stats.cPrivatePages--;// 当前VM私有页数-1

    //共享页元数据设置
    /* Modify the page structure. */
    pPage->Shared.pfn         = (uint32_t)(uint64_t)(HCPhys >> GUEST_PAGE_SHIFT);
    pPage->Shared.cRefs       = 1;
#ifdef VBOX_STRICT
    pPageDesc->u32StrictChecksum = gmmR0StrictPageChecksum(pGMM, pGVM, idPage);
    pPage->Shared.u14Checksum = pPageDesc->u32StrictChecksum;
#else
    NOREF(pPageDesc);
    pPage->Shared.u14Checksum = 0;
#endif
    pPage->Shared.u2State     = GMM_PAGE_STATE_SHARED;
}


/*
PGMM pGMM,                      // GMM全局管理实例
    PGVM pGVM,                      // 目标虚拟机控制块
    PGMMSHAREDMODULE pModule,       // 共享模块信息（未使用）
    unsigned idxRegion,             // 内存区域索引（未使用）
    unsigned idxPage,               // 页面索引
    PGMMSHAREDPAGEDESC pPageDesc,   // 页面描述符（输入/输出）
    PGMMSHAREDREGIONDESC pGlobalRegion // 全局区域描述符
*/
//处理首次发现的共享页面，将其从私有页转换为共享页。
static int gmmR0SharedModuleCheckPageFirstTime(PGMM pGMM, PGVM pGVM, PGMMSHAREDMODULE pModule,
                                               unsigned idxRegion, unsigned idxPage,
                                               PGMMSHAREDPAGEDESC pPageDesc, PGMMSHAREDREGIONDESC pGlobalRegion)
{
    NOREF(pModule);

    /* Easy case: just change the internal page type. */
    //通过 pPageDesc->idPage 从GMM中获取页面结构体 PGMMPAGE。
    PGMMPAGE pPage = gmmR0GetPage(pGMM, pPageDesc->idPage);
    AssertMsgReturn(pPage, ("idPage=%#x (GCPhys=%RGp HCPhys=%RHp idxRegion=%#x idxPage=%#x) #1\n",
                            pPageDesc->idPage, pPageDesc->GCPhys, pPageDesc->HCPhys, idxRegion, idxPage),
                    VERR_PGM_PHYS_INVALID_PAGE_ID);
    NOREF(idxRegion);

    //验证页面描述符中的客户机物理地址（GCPhys）是否与GMM记录的地址一致。
    //pPage->Private.pfn << 12 将页面帧号（PFN）转换为物理地址（4KB页对齐）。
    AssertMsg(pPageDesc->GCPhys == (pPage->Private.pfn << 12), ("desc %RGp gmm %RGp\n", pPageDesc->HCPhys, (pPage->Private.pfn << 12)));

    // 转换为共享页面
    // pPageDesc->HCPhys：主机物理地址（用于共享映射）。
    //pPage：GMM页面结构体（更新其状态为 GMM_PAGE_STATE_SHARED）。
      //更新GMM页面状态和引用计数。
      //可能将页面加入全局共享哈希表。
    gmmR0ConvertToSharedPage(pGMM, pGVM, pPageDesc->HCPhys, pPageDesc->idPage, pPage, pPageDesc);

    /* Keep track of these references. */
    //记录共享引用
    //后续访问该页面时，可通过 idxPage 直接找到共享页ID，跳过重复检查。
    pGlobalRegion->paidPages[idxPage] = pPageDesc->idPage;

    return VINF_SUCCESS;
}

/**
 * Checks specified shared module range for changes
 *
 * Performs the following tasks:
 *  - If a shared page is new, then it changes the GMM page type to shared and
 *    returns it in the pPageDesc descriptor.
 *  - If a shared page already exists, then it checks if the VM page is
 *    identical and if so frees the VM page and returns the shared page in
 *    pPageDesc descriptor.
 *
 * @remarks ASSUMES the caller has acquired the GMM semaphore!!
 *
 * @returns VBox status code.
 * @param   pGVM        Pointer to the GVM instance data.
 * @param   pModule     Module description
 * @param   idxRegion   Region index
 * @param   idxPage     Page index
 * @param   pPageDesc   Page descriptor
 */
//该函数实现了 内存去重（Memory Deduplication），即在多个虚拟机运行相同操作系统或应用程序时，
//让它们共享相同的内存页，从而减少总体内存占用。
//例如：10 个 Windows 10 VM 运行时，内核代码页、系统 DLL 等完全相同。通过共享这些页，可减少物理内存占用。
/*
  pGVM：指向虚拟机控制结构的指针
  pModule：共享模块信息结构体
  idxRegion：内存区域索引
  idxPage：页面索引
  pPageDesc：页面描述符（输出参数）
*/
GMMR0DECL(int) GMMR0SharedModuleCheckPage(PGVM pGVM, PGMMSHAREDMODULE pModule, uint32_t idxRegion, uint32_t idxPage,
                                          PGMMSHAREDPAGEDESC pPageDesc)
{
    int     rc;
    PGMM    pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    pPageDesc->u32StrictChecksum = 0;

    AssertMsgReturn(idxRegion < pModule->cRegions,// 检查区域索引是否越界
                    ("idxRegion=%#x cRegions=%#x %s %s\n", idxRegion, pModule->cRegions, pModule->szName, pModule->szVersion),
                    VERR_INVALID_PARAMETER); // 越界则返回无效参数错误

    uint32_t const cPages = pModule->aRegions[idxRegion].cb >> GUEST_PAGE_SHIFT;// 计算该区域的页数（字节数转页数）
    AssertMsgReturn(idxPage < cPages,// 检查页面索引是否越界
                    ("idxRegion=%#x cRegions=%#x %s %s\n", idxRegion, pModule->cRegions, pModule->szName, pModule->szVersion),
                    VERR_INVALID_PARAMETER);

    LogFlow(("GMMR0SharedModuleCheckRange %s base %RGv region %d idxPage %d\n", pModule->szName, pModule->Core.Key, idxRegion, idxPage));

    /*
     * First time; create a page descriptor array.
     */
    PGMMSHAREDREGIONDESC pGlobalRegion = &pModule->aRegions[idxRegion]; // 获取目标区域描述符
    if (!pGlobalRegion->paidPages)// 如果描述符数组未分配
    {
        Log(("Allocate page descriptor array for %d pages\n", cPages));// 记录分配日志
        pGlobalRegion->paidPages = (uint32_t *)RTMemAlloc(cPages * sizeof(pGlobalRegion->paidPages[0])); // 分配数组内存
        AssertReturn(pGlobalRegion->paidPages, VERR_NO_MEMORY);

        /* Invalidate all descriptors. */
        /* 将所有描述符初始化为无效值 */
        uint32_t i = cPages;
        while (i-- > 0)
            pGlobalRegion->paidPages[i] = NIL_GMM_PAGEID;
    }

    /*
     * We've seen this shared page for the first time?
     */
    /* 首次见到这个共享页面？ */
    if (pGlobalRegion->paidPages[idxPage] == NIL_GMM_PAGEID)
    {
        Log(("New shared page guest %RGp host %RHp\n", pPageDesc->GCPhys, pPageDesc->HCPhys));
        return gmmR0SharedModuleCheckPageFirstTime(pGMM, pGVM, pModule, idxRegion, idxPage, pPageDesc, pGlobalRegion);//调用首次处理函数
    }

    /*
     * We've seen it before...
     */
    Log(("Replace existing page guest %RGp host %RHp id %#x -> id %#x\n",
         pPageDesc->GCPhys, pPageDesc->HCPhys, pPageDesc->idPage, pGlobalRegion->paidPages[idxPage]));
    Assert(pPageDesc->idPage != pGlobalRegion->paidPages[idxPage]); //确保新旧页面ID不同

    /*
     * Get the shared page source.
     */
    PGMMPAGE pPage = gmmR0GetPage(pGMM, pGlobalRegion->paidPages[idxPage]); //通过ID获取共享页面结构
    AssertMsgReturn(pPage, ("idPage=%#x (idxRegion=%#x idxPage=%#x) #2\n", pPageDesc->idPage, idxRegion, idxPage),
                    VERR_PGM_PHYS_INVALID_PAGE_ID);

    if (pPage->Common.u2State != GMM_PAGE_STATE_SHARED) // 检查页面状态是否仍为"共享"
    {
        /*
         * Page was freed at some point; invalidate this entry.
         */
        /** @todo this isn't really bullet proof. */
        Log(("Old shared page was freed -> create a new one\n"));
        pGlobalRegion->paidPages[idxPage] = NIL_GMM_PAGEID;// 重置为无效状态
        return gmmR0SharedModuleCheckPageFirstTime(pGMM, pGVM, pModule, idxRegion, idxPage, pPageDesc, pGlobalRegion);
    }

    //记录物理地址替换
    Log(("Replace existing page guest host %RHp -> %RHp\n", pPageDesc->HCPhys, ((uint64_t)pPage->Shared.pfn) << GUEST_PAGE_SHIFT));

    /*
     * Calculate the virtual address of the local page.
     */
    /* 计算本地页面的虚拟地址 */
    PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, pPageDesc->idPage >> GMM_CHUNKID_SHIFT);// 获取内存块
    AssertMsgReturn(pChunk, ("idPage=%#x (idxRegion=%#x idxPage=%#x) #4\n", pPageDesc->idPage, idxRegion, idxPage),
                    VERR_PGM_PHYS_INVALID_PAGE_ID);

    uint8_t *pbChunk;
    //检查内存块是否已映射
    AssertMsgReturn(gmmR0IsChunkMapped(pGMM, pGVM, pChunk, (PRTR3PTR)&pbChunk),
                    ("idPage=%#x (idxRegion=%#x idxPage=%#x) #3\n", pPageDesc->idPage, idxRegion, idxPage),
                    VERR_PGM_PHYS_INVALID_PAGE_ID);
     // 计算本地页面虚拟地址
    uint8_t  *pbLocalPage = pbChunk + ((pPageDesc->idPage & GMM_PAGEID_IDX_MASK) << GUEST_PAGE_SHIFT);

    /*
     * Calculate the virtual address of the shared page.
     */
     /* 计算共享页面的虚拟地址 */
     // 获取共享页面对应的内存块
    pChunk = gmmR0GetChunk(pGMM, pGlobalRegion->paidPages[idxPage] >> GMM_CHUNKID_SHIFT);
    Assert(pChunk); /* can't fail as gmmR0GetPage succeeded. */

    /*
     * Get the virtual address of the physical page; map the chunk into the VM
     * process if not already done.
     */
     /* 获取物理页面的虚拟地址：如果未映射则映射到VM进程 */
    if (!gmmR0IsChunkMapped(pGMM, pGVM, pChunk, (PRTR3PTR)&pbChunk))// 检查共享内存块是否已映射
    {
        Log(("Map chunk into process!\n"));
        rc = gmmR0MapChunk(pGMM, pGVM, pChunk, false /*fRelaxedSem*/, (PRTR3PTR)&pbChunk);// 执行映射
        AssertRCReturn(rc, rc);
    }
    // 计算共享页面虚拟地址
    uint8_t *pbSharedPage = pbChunk + ((pGlobalRegion->paidPages[idxPage] & GMM_PAGEID_IDX_MASK) << GUEST_PAGE_SHIFT);

#ifdef VBOX_STRICT
    pPageDesc->u32StrictChecksum = RTCrc32(pbSharedPage, GUEST_PAGE_SIZE);//计算共享页面CRC32
    uint32_t uChecksum = pPageDesc->u32StrictChecksum & UINT32_C(0x00003fff);// 取低14位
     // 校验和必须匹配
    AssertMsg(!uChecksum || uChecksum == pPage->Shared.u14Checksum || !pPage->Shared.u14Checksum,
              ("%#x vs %#x - idPage=%#x - %s %s\n", uChecksum, pPage->Shared.u14Checksum,
               pGlobalRegion->paidPages[idxPage], pModule->szName, pModule->szVersion));
#endif

    // 比较本地页和共享页内容
    if (memcmp(pbSharedPage, pbLocalPage, GUEST_PAGE_SIZE))
    {
        // 内容不同则跳过
        Log(("Unexpected differences found between local and shared page; skip\n"));
        /* Signal to the caller that this one hasn't changed. */
        pPageDesc->idPage = NIL_GMM_PAGEID; // 标记为无效页面
        return VINF_SUCCESS;// 返回成功但要求调用方忽略
    }

    /*
     * Free the old local page.
     */
     /* 释放旧的本地页面 */
    GMMFREEPAGEDESC PageDesc;
    PageDesc.idPage = pPageDesc->idPage;// 设置待释放的页面ID
    rc = gmmR0FreePages(pGMM, pGVM, 1, &PageDesc, GMMACCOUNT_BASE);// 释放页面
    AssertRCReturn(rc, rc); // 失败则返回错误码

    gmmR0UseSharedPage(pGMM, pGVM, pPage);// 标记共享页面为已使用

    /*
     * Pass along the new physical address & page id.
     */
     /* 返回新的物理地址和页面ID */
    pPageDesc->HCPhys = ((uint64_t)pPage->Shared.pfn) << GUEST_PAGE_SHIFT;// 更新物理地址
    pPageDesc->idPage = pGlobalRegion->paidPages[idxPage]; // 更新页面ID

    return VINF_SUCCESS;
}


/**
 * RTAvlGCPtrDestroy callback.
 *
 * @returns 0 or VERR_GMM_INSTANCE.
 * @param   pNode       The node to destroy.
 * @param   pvArgs      Pointer to an argument packet.
 */
//作为RTAvlGCPtrDoWithAll或RTAvlGCPtrDestroy的回调函数，用于销毁虚拟机（VM）的单个共享模块节点
/*
pNode
  类型为PAVLGCPTRNODECORE，指向AVL树中待清理的共享模块节点，实际类型为PGMMSHAREDMODULEPERVM（虚拟机级别的共享模块描述结构）
pvArgs
  类型为void*，实际传入GMMR0SHMODPERVMDTORARGS结构体指针，包含：
  pGMM：全局内存管理器实例。
  pGVM：目标虚拟机指针
*/
static DECLCALLBACK(int) gmmR0CleanupSharedModule(PAVLGCPTRNODECORE pNode, void *pvArgs)
{
    gmmR0ShModDeletePerVM(((GMMR0SHMODPERVMDTORARGS *)pvArgs)->pGMM,
                          ((GMMR0SHMODPERVMDTORARGS *)pvArgs)->pGVM,
                          (PGMMSHAREDMODULEPERVM)pNode,
                          false /*fRemove*/);
    return VINF_SUCCESS;
}


/**
 * Used by GMMR0CleanupVM to clean up shared modules.
 *
 * This is called without taking the GMM lock so that it can be yielded as
 * needed here.
 *
 * @param   pGMM        The GMM handle.
 * @param   pGVM        The global VM handle.
 */
static void gmmR0SharedModuleCleanup(PGMM pGMM, PGVM pGVM)
{
    gmmR0MutexAcquire(pGMM);
    GMM_CHECK_SANITY_UPON_ENTERING(pGMM);

    GMMR0SHMODPERVMDTORARGS Args;
    Args.pGVM = pGVM;
    Args.pGMM = pGMM;
    RTAvlGCPtrDestroy(&pGVM->gmm.s.pSharedModuleTree, gmmR0CleanupSharedModule, &Args);

    AssertMsg(pGVM->gmm.s.Stats.cShareableModules == 0, ("%d\n", pGVM->gmm.s.Stats.cShareableModules));
    pGVM->gmm.s.Stats.cShareableModules = 0;

    gmmR0MutexRelease(pGMM);
}

#endif /* VBOX_WITH_PAGE_SHARING */

/**
 * Removes all shared modules for the specified VM
 *
 * @returns VBox status code.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The VCPU id.
 */
/*
作用：销毁虚拟机的共享模块树（pSharedModuleTree），并清理相关资源
触发场景：虚拟机重置、关闭或内存管理策略变更时调用
 * */
GMMR0DECL(int) GMMR0ResetSharedModules(PGVM pGVM, VMCPUID idCpu)
{
#ifdef VBOX_WITH_PAGE_SHARING
    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE); //验证GMM实例（pGMM）和虚拟机（pGVM）的有效性
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);//检查CPU线程（idCpu）是否属于目标虚拟机
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Take the semaphore and do some more validations.
     */
    gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        Log(("GMMR0ResetSharedModules\n"));
        GMMR0SHMODPERVMDTORARGS Args;
        Args.pGVM = pGVM;
        Args.pGMM = pGMM;
        RTAvlGCPtrDestroy(&pGVM->gmm.s.pSharedModuleTree, gmmR0CleanupSharedModule, &Args);
        pGVM->gmm.s.Stats.cShareableModules = 0;

        rc = VINF_SUCCESS;
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    return rc;
#else
    RT_NOREF(pGVM, idCpu);
    return VERR_NOT_IMPLEMENTED;
#endif
}

#ifdef VBOX_WITH_PAGE_SHARING

/**
 * Tree enumeration callback for checking a shared module.
 */
static DECLCALLBACK(int) gmmR0CheckSharedModule(PAVLGCPTRNODECORE pNode, void *pvUser)
{
    GMMCHECKSHAREDMODULEINFO   *pArgs   = (GMMCHECKSHAREDMODULEINFO*)pvUser;
    PGMMSHAREDMODULEPERVM       pRecVM  = (PGMMSHAREDMODULEPERVM)pNode;
    PGMMSHAREDMODULE            pGblMod = pRecVM->pGlobalModule;

    Log(("gmmR0CheckSharedModule: check %s %s base=%RGv size=%x\n",
         pGblMod->szName, pGblMod->szVersion, pGblMod->Core.Key, pGblMod->cbModule));

    int rc = PGMR0SharedModuleCheck(pArgs->pGVM, pArgs->pGVM, pArgs->idCpu, pGblMod, pRecVM->aRegionsGCPtrs);
    if (RT_FAILURE(rc))
        return rc;
    return VINF_SUCCESS;
}

#endif /* VBOX_WITH_PAGE_SHARING */

/**
 * Check all shared modules for the specified VM.
 *
 * @returns VBox status code.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   idCpu       The calling EMT number.
 * @thread  EMT(idCpu)
 */
/*
  遍历虚拟机（pGVM）的共享模块树（pSharedModuleTree），检查每个模块的有效性
  通过回调函数gmmR0CheckSharedModule执行具体检查逻辑
*/
GMMR0DECL(int) GMMR0CheckSharedModules(PGVM pGVM, VMCPUID idCpu)
{
#ifdef VBOX_WITH_PAGE_SHARING
    /*
     * Validate input and get the basics.
     */
    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);
    int rc = GVMMR0ValidateGVMandEMT(pGVM, idCpu);
    if (RT_FAILURE(rc))
        return rc;

# ifndef DEBUG_sandervl
    /*
     * Take the semaphore and do some more validations.
     */
    gmmR0MutexAcquire(pGMM);
# endif
    //GMM_CHECK_SANITY_UPON_ENTERING和GMM_CHECK_SANITY_UPON_LEAVING确保内存管理数据结构的一致性
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        /*
         * Walk the tree, checking each module.
         */
        Log(("GMMR0CheckSharedModules\n"));

        GMMCHECKSHAREDMODULEINFO Args;
        Args.pGVM     = pGVM;
        Args.idCpu    = idCpu;
        rc = RTAvlGCPtrDoWithAll(&pGVM->gmm.s.pSharedModuleTree, true /* fFromLeft */, gmmR0CheckSharedModule, &Args);

        Log(("GMMR0CheckSharedModules done (rc=%Rrc)!\n", rc));
        GMM_CHECK_SANITY_UPON_LEAVING(pGMM);
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

# ifndef DEBUG_sandervl
    gmmR0MutexRelease(pGMM);
# endif
    return rc;
#else
    RT_NOREF(pGVM, idCpu);
    return VERR_NOT_IMPLEMENTED;
#endif
}

#ifdef VBOX_STRICT

/**
 * Worker for GMMR0FindDuplicatePageReq.
 *
 * @returns true if duplicate, false if not.
 */
static bool gmmR0FindDupPageInChunk(PGMM pGMM, PGVM pGVM, PGMMCHUNK pChunk, uint8_t const *pbSourcePage)
{
    bool fFoundDuplicate = false;
    /* Only take chunks not mapped into this VM process; not entirely correct. */
    uint8_t *pbChunk;
    if (!gmmR0IsChunkMapped(pGMM, pGVM, pChunk, (PRTR3PTR)&pbChunk))
    {
        int rc = gmmR0MapChunk(pGMM, pGVM, pChunk, false /*fRelaxedSem*/, (PRTR3PTR)&pbChunk);
        if (RT_SUCCESS(rc))
        {
            /*
             * Look for duplicate pages
             */
            uintptr_t iPage = GMM_CHUNK_NUM_PAGES;
            while (iPage-- > 0)
            {
                if (GMM_PAGE_IS_PRIVATE(&pChunk->aPages[iPage]))
                {
                    uint8_t *pbDestPage = pbChunk + (iPage  << GUEST_PAGE_SHIFT);
                    if (!memcmp(pbSourcePage, pbDestPage, GUEST_PAGE_SIZE))
                    {
                        fFoundDuplicate = true;
                        break;
                    }
                }
            }
            gmmR0UnmapChunk(pGMM, pGVM, pChunk, false /*fRelaxedSem*/);
        }
    }
    return fFoundDuplicate;
}


/**
 * Find a duplicate of the specified page in other active VMs
 *
 * @returns VBox status code.
 * @param   pGVM        The global (ring-0) VM structure.
 * @param   pReq        Pointer to the request packet.
 */
//主要用于在虚拟机（VM）内存管理中查找重复页（Duplicate Page）
/*
  检查请求参数（pReq）是否合法。
  获取 GMM 实例（pGMM）并验证其状态。
  检查目标页（pReq->idPage）是否存在，并获取其物理地址（pbSourcePage）。
  遍历所有内存块（Chunk），查找是否存在与 pbSourcePage 内容相同的页。
  返回结果（pReq->fDuplicate 表示是否找到重复页）。
 * */
GMMR0DECL(int) GMMR0FindDuplicatePageReq(PGVM pGVM, PGMMFINDDUPLICATEPAGEREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    PGMM pGMM;
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);

    int rc = GVMMR0ValidateGVM(pGVM);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Take the semaphore and do some more validations.
     */
    rc = gmmR0MutexAcquire(pGMM);
    if (GMM_CHECK_SANITY_UPON_ENTERING(pGMM))
    {
        uint8_t  *pbChunk;
        PGMMCHUNK pChunk = gmmR0GetChunk(pGMM, pReq->idPage >> GMM_CHUNKID_SHIFT);
        if (pChunk)
        {
            if (gmmR0IsChunkMapped(pGMM, pGVM, pChunk, (PRTR3PTR)&pbChunk))
            {
                uint8_t *pbSourcePage = pbChunk + ((pReq->idPage & GMM_PAGEID_IDX_MASK) << GUEST_PAGE_SHIFT);
                PGMMPAGE pPage = gmmR0GetPage(pGMM, pReq->idPage);
                if (pPage)
                {
                    /*
                     * Walk the chunks
                     */
                    pReq->fDuplicate = false;
                    RTListForEach(&pGMM->ChunkList, pChunk, GMMCHUNK, ListNode)
                    {
                        if (gmmR0FindDupPageInChunk(pGMM, pGVM, pChunk, pbSourcePage))
                        {
                            pReq->fDuplicate = true;
                            break;
                        }
                    }
                }
                else
                {
                    AssertFailed();
                    rc = VERR_PGM_PHYS_INVALID_PAGE_ID;
                }
            }
            else
                AssertFailed();
        }
        else
            AssertFailed();
    }
    else
        rc = VERR_GMM_IS_NOT_SANE;

    gmmR0MutexRelease(pGMM);
    return rc;
}

#endif /* VBOX_STRICT */


/**
 * Retrieves the GMM statistics visible to the caller.
 *
 * @returns VBox status code.
 *
 * @param   pStats      Where to put the statistics.
 * @param   pSession    The current session.
 * @param   pGVM        The GVM to obtain statistics for. Optional.
 */
//用于查询内存管理统计信息
GMMR0DECL(int) GMMR0QueryStatistics(PGMMSTATS pStats, PSUPDRVSESSION pSession, PGVM pGVM)
{
    LogFlow(("GVMMR0QueryStatistics: pStats=%p pSession=%p pGVM=%p\n", pStats, pSession, pGVM));

    /*
     * Validate input.
     */
    AssertPtrReturn(pSession, VERR_INVALID_POINTER);
    AssertPtrReturn(pStats, VERR_INVALID_POINTER);
    pStats->cMaxPages = 0; /* (crash before taking the mutex...) */

    PGMM pGMM;
    //获取全局 GMM (Guest Memory Manager) 实例
    GMM_GET_VALID_INSTANCE(pGMM, VERR_GMM_INSTANCE);

    /*
     * Validate the VM handle, if not NULL, and lock the GMM.
     */
    int rc;
    if (pGVM)
    {
        rc = GVMMR0ValidateGVM(pGVM);
        if (RT_FAILURE(rc))
            return rc;
    }

    rc = gmmR0MutexAcquire(pGMM);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Copy out the GMM statistics.
     */
    pStats->cMaxPages                   = pGMM->cMaxPages; //虚拟机可使用的最大物理页面数，反映内存资源池总容量
    pStats->cReservedPages              = pGMM->cReservedPages;//已预留但未实际分配的页面数，用于预占内存配额`
    pStats->cOverCommittedPages         = pGMM->cOverCommittedPages;//超额提交的页面数，表示超出物理内存的虚拟内存量
    pStats->cAllocatedPages             = pGMM->cAllocatedPages;//实际已分配的物理页面总数，包含独占和共享内存
                                                                //
    pStats->cSharedPages                = pGMM->cSharedPages; //跨虚拟机共享的只读页面数，如相同系统镜像的共享
    pStats->cDuplicatePages             = pGMM->cDuplicatePages; //写时复制(COW)产生的副本页面数‌
    pStats->cLeftBehindSharedPages      = pGMM->cLeftBehindSharedPages; //虚拟机退出后遗留的共享页面数，可能被其他VM复用
    pStats->cBalloonedPages             = pGMM->cBalloonedPages; //通过内存气球技术回收的页面数，用于动态调整内存占用
    pStats->cChunks                     = pGMM->cChunks; //当前活跃的内存块(chunk)数量，反映内存碎片化程度
    pStats->cFreedChunks                = pGMM->cFreedChunks; //已释放但尚未回收的内存块数，用于延迟回收统计
    pStats->cShareableModules           = pGMM->cShareableModules; //可共享内存模块(如DLL)的数量
    pStats->idFreeGeneration            = pGMM->idFreeGeneration; //内存块回收代标识符，用于LRU算法实现‌
    RT_ZERO(pStats->au64Reserved);

    /*
     * Copy out the VM statistics.
     */
    if (pGVM)
        pStats->VMStats = pGVM->gmm.s.Stats;
    else
        RT_ZERO(pStats->VMStats);

    gmmR0MutexRelease(pGMM);
    return rc;
}


/**
 * VMMR0 request wrapper for GMMR0QueryStatistics.
 *
 * @returns see GMMR0QueryStatistics.
 * @param   pGVM        The global (ring-0) VM structure. Optional.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0QueryStatisticsReq(PGVM pGVM, PGMMQUERYSTATISTICSSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GMMR0QueryStatistics(&pReq->Stats, pReq->pSession, pGVM);
}


/**
 * Resets the specified GMM statistics.
 *
 * @returns VBox status code.
 *
 * @param   pStats      Which statistics to reset, that is, non-zero fields
 *                      indicates which to reset.
 * @param   pSession    The current session.
 * @param   pGVM        The GVM to reset statistics for. Optional.
 */
GMMR0DECL(int) GMMR0ResetStatistics(PCGMMSTATS pStats, PSUPDRVSESSION pSession, PGVM pGVM)
{
    NOREF(pStats); NOREF(pSession); NOREF(pGVM);
    /* Currently nothing we can reset at the moment. */
    return VINF_SUCCESS;
}


/**
 * VMMR0 request wrapper for GMMR0ResetStatistics.
 *
 * @returns see GMMR0ResetStatistics.
 * @param   pGVM        The global (ring-0) VM structure. Optional.
 * @param   pReq        Pointer to the request packet.
 */
GMMR0DECL(int) GMMR0ResetStatisticsReq(PGVM pGVM, PGMMRESETSTATISTICSSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GMMR0ResetStatistics(&pReq->Stats, pReq->pSession, pGVM);
}

