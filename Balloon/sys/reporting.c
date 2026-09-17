/*
 * This file implements the free page reporting feature of the
 * virtio balloon device (VIRTIO_BALLOON_F_PAGE_REPORTING).
 *
 * Copyright (c) 2026  Red Hat, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met :
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and / or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of their contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Design overview
 * ===============
 *
 * Windows does not expose any driver-visible free page list - free pages
 * live on OS-internal lists - so free page reporting is implemented by
 * allocating pages out with MmAllocatePagesForMdlEx and reporting those
 * pages to the device through the reporting virtqueue. Unlike the inflate
 * and deflate queues, the scatter-gather list submitted to the reporting
 * queue describes the reported pages themselves (virtio 1.x, section
 * 5.5.6.7).
 *
 * Report-then-release does not work on Windows: handing the pages back to
 * the OS re-touches them (the memory manager dirties pages when they are
 * freed), so the host-side reclaim effect collapses immediately. The
 * driver therefore uses report-then-hold: reported pages stay allocated
 * inside the driver, where the guest cannot touch them, and are only
 * handed back to the guest when the guest needs the memory. Effectively
 * this is an adaptive balloon driven by the free page reporting queue.
 *
 * Only complete 2MB blocks are reported, in both size and boundary,
 * matching the granularity the host can reclaim (transparent huge pages)
 * and the default reporting granularity of the Linux implementation
 * (pageblock_order). The alignment comes for free from the memory manager:
 * MmAllocatePagesForMdlEx called with MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS
 * and SkipBytes = 2MB returns chunks that are each guaranteed to be
 * exactly 2MB long and aligned on a 2MB boundary, preferably taken from
 * the system's large page cache.
 *
 * Held pages are returned to the guest when the system signals a low
 * memory condition or when the available memory falls below a watermark.
 *
 * The watermark defaults to max(RAM/8, 256MB) and can be overridden per
 * deployment with MinFreeMb in the driver service Parameters registry
 * key (the Windows counterpart of the Linux page_reporting module
 * parameters), clamped to [64MB, RAM/2]. The release logic itself
 * (LowMemoryCondition handling, hysteresis band, gradual release) is
 * deliberately not configurable.
 *
 * Note on commit charge: the held pages are committed and pinned, so they
 * consume commit charge as well as physical memory. Commitment does not
 * occupy physical pages until first access (demand zero), so the physical
 * watermark alone says nothing about the remaining commit limit - the
 * reporting cycle therefore also hands pages back once the remaining
 * commit limit (RAM + pagefile - committed) runs low. A system-managed
 * pagefile absorbs most commit pressure by growing on its own.
 */

#include "precomp.h"
#include "ntddkex.h"

#if defined(EVENT_TRACING)
#include "reporting.tmh"
#endif

#ifdef ALLOC_PRAGMA
#pragma alloc_text(PAGE, BalloonReportInitialize)
#endif // ALLOC_PRAGMA

static NTSTATUS ReportingQueryMemoryState(OUT PULONG AvailablePages,
                                          OUT PULONG CommitHeadroomPages,
                                          OUT PULONG CommitLimitPages)
{
    SYSTEM_PERFORMANCE_INFORMATION perfInfo;
    ULONG outLen = 0;
    NTSTATUS status;

    RtlZeroMemory(&perfInfo, sizeof(perfInfo));
    status = ZwQuerySystemInformation(SystemPerformanceInformation, &perfInfo, sizeof(perfInfo), &outLen);
    if (NT_SUCCESS(status))
    {
        *AvailablePages = perfInfo.AvailablePages;
        *CommitLimitPages = perfInfo.CommitLimit;
        *CommitHeadroomPages = (perfInfo.CommitLimit > perfInfo.CommittedPages) ? (perfInfo.CommitLimit -
                                                                                   perfInfo.CommittedPages)
                                                                                : 0;
    }
    return status;
}

/*
 * The held pages are committed and pinned, so they consume commit charge as
 * well as physical memory. Commitment does not occupy physical pages until
 * first access (demand zero), so the available-memory watermark says nothing
 * about the remaining commit limit: workloads that reserve a lot of memory
 * without touching it can leave plenty of available pages while the commit
 * limit is nearly exhausted. Hand pages back before the held charge can eat
 * into that last reserve - built-in, deliberately not configurable.
 *
 * The reserve is anchored to the physical memory size rather than to the
 * commit limit: a large pagefile inflates the commit limit without making
 * a low headroom any more dangerous (materialization can be paged out),
 * so the reserve must not grow with the pagefile.
 */
static __inline BOOLEAN ReportingCommitHeadroomLow(IN PDEVICE_CONTEXT devCtx, IN ULONG CommitHeadroomPages)
{
    ULONG threshold;

    if (devCtx->ReportingTotalPages == 0)
    {
        return FALSE;
    }

    threshold = devCtx->ReportingTotalPages / REPORTING_COMMIT_HEADROOM_FRACTION;
    if (threshold < REPORTING_MIN_COMMIT_HEADROOM_PAGES)
    {
        threshold = REPORTING_MIN_COMMIT_HEADROOM_PAGES;
    }
    return CommitHeadroomPages < threshold;
}

static ULONG ReportingAllocWatermark(IN PDEVICE_CONTEXT devCtx)
{
    if (devCtx->ReportingMinFreePages != 0)
    {
        return devCtx->ReportingMinFreePages;
    }

    ULONG watermark = devCtx->ReportingTotalPages / REPORTING_AVAILABLE_FRACTION;

    if (watermark < REPORTING_MIN_AVAILABLE_PAGES)
    {
        watermark = REPORTING_MIN_AVAILABLE_PAGES;
    }
    return watermark;
}

/*
 * Splits an MDL returned by MmAllocatePagesForMdlEx into one scatter-gather
 * segment per 2MB block. The pages of a block are physically contiguous and
 * appear consecutively in the MDL, a well-formed block covers exactly 512
 * consecutive pages starting at the 2MB boundary. Anything else (which the
 * memory manager does not produce today) is never reported.
 */
static VOID ReportingExtractBlocks(IN PMDL Mdl, OUT PVIO_SG Segments, IN OUT PULONG SegmentCount)
{
    PPFN_NUMBER pfnArray = MmGetMdlPfnArray(Mdl);
    ULONG pageCount = MmGetMdlByteCount(Mdl) >> PAGE_SHIFT;
    ULONG i = 0;

    while (i < pageCount)
    {
        ULONGLONG blockIndex = pfnArray[i] >> REPORTING_BLOCK_SHIFT;
        ULONG start = i;

        while (i < pageCount && (pfnArray[i] >> REPORTING_BLOCK_SHIFT) == blockIndex)
        {
            i++;
        }

        if (i - start == REPORTING_BLOCK_PAGES && (pfnArray[start] & (REPORTING_BLOCK_PAGES - 1)) == 0)
        {
            ULONG segment = (*SegmentCount)++;

            Segments[segment].physAddr.QuadPart = blockIndex << (PAGE_SHIFT + REPORTING_BLOCK_SHIFT);
            Segments[segment].length = REPORTING_BLOCK_SIZE;
        }
        else
        {
            TraceEvents(TRACE_LEVEL_WARNING,
                        DBG_REPORTING,
                        "Skipping a malformed 2MB block (PFN 0x%I64x, %d pages)\n",
                        (ULONGLONG)pfnArray[start],
                        i - start);
        }
    }
}

static VOID ReportingReleaseMdl(IN PDEVICE_CONTEXT devCtx, IN PPAGE_LIST_ENTRY PageListEntry)
{
    PMDL mdl = PageListEntry->PageMdl;

    devCtx->ReportingHeldPages -= MmGetMdlByteCount(mdl) >> PAGE_SHIFT;
    devCtx->ReportingMdlCount--;

    MmFreePagesFromMdl(mdl);
    ExFreePool(mdl);
    ExFreeToNPagedLookasideList(&devCtx->LookAsideList, PageListEntry);
}

/* hands the MaxMdls most recently held MDLs back to the guest */
static VOID ReportingReleasePages(IN PDEVICE_CONTEXT devCtx, IN ULONG MaxMdls)
{
    while (devCtx->ReportingMdlCount != 0 && MaxMdls-- > 0)
    {
        PPAGE_LIST_ENTRY pageListEntry = (PPAGE_LIST_ENTRY)PopEntryList(&devCtx->ReportingMdlList);

        if (pageListEntry == NULL)
        {
            ASSERT(pageListEntry != NULL);
            break;
        }
        ReportingReleaseMdl(devCtx, pageListEntry);
    }
}

/*
 * Adds one report request to the reporting virtqueue and waits for the
 * host to acknowledge it. Returns STATUS_UNSUCCESSFUL if the request did
 * not fit into the virtqueue, any other error indicates a host timeout.
 */
static NTSTATUS ReportingSendRequest(IN PDEVICE_CONTEXT devCtx, IN PVIO_SG Segments, IN ULONG SegmentCount)
{
    LARGE_INTEGER timeout;
    NTSTATUS status;
    unsigned int len;
    bool doNotify;

    WdfSpinLockAcquire(devCtx->InfDefQueueLock);
    if (virtqueue_add_buf(devCtx->RepVirtQueue, Segments, 0, SegmentCount, devCtx, NULL, 0) < 0)
    {
        WdfSpinLockRelease(devCtx->InfDefQueueLock);
        return STATUS_UNSUCCESSFUL;
    }
    doNotify = virtqueue_kick_prepare(devCtx->RepVirtQueue);
    WdfSpinLockRelease(devCtx->InfDefQueueLock);

    if (doNotify)
    {
        virtqueue_notify(devCtx->RepVirtQueue);
    }

    timeout.QuadPart = Int32x32To64(1000, -10000);
    status = KeWaitForSingleObject(&devCtx->HostAckEvent, Executive, KernelMode, FALSE, &timeout);
    if (status == STATUS_TIMEOUT)
    {
        /* the host may have acknowledged the request without an interrupt,
         * poll the virtqueue once before giving up */
        WdfSpinLockAcquire(devCtx->InfDefQueueLock);
        if (virtqueue_get_buf(devCtx->RepVirtQueue, &len) != NULL)
        {
            status = STATUS_WAIT_0;
        }
        WdfSpinLockRelease(devCtx->InfDefQueueLock);
    }
    if (status == STATUS_TIMEOUT)
    {
        TraceEvents(TRACE_LEVEL_ERROR, DBG_REPORTING, "%s :: host did not acknowledge the report\n", __FUNCTION__);
    }
    return status;
}

static NTSTATUS ReportingSendBlocks(IN PDEVICE_CONTEXT devCtx, IN PVIO_SG Segments, IN ULONG SegmentCount)
{
    NTSTATUS status;
    ULONG i;

    status = ReportingSendRequest(devCtx, Segments, SegmentCount);
    if (status != STATUS_UNSUCCESSFUL)
    {
        return status;
    }

    /* the request did not fit into the virtqueue, report block by block */
    for (i = 0; i < SegmentCount; i++)
    {
        status = ReportingSendRequest(devCtx, &Segments[i], 1);
        if (!NT_SUCCESS(status))
        {
            return status;
        }
    }
    return STATUS_SUCCESS;
}

/* reports the collected segments and resets the segment counter */
static BOOLEAN ReportingFlushSegments(IN PDEVICE_CONTEXT devCtx, IN PVIO_SG Segments, IN OUT PULONG SegmentCount)
{
    if (*SegmentCount == 0)
    {
        return TRUE;
    }

    if (NT_SUCCESS(ReportingSendBlocks(devCtx, Segments, *SegmentCount)))
    {
        devCtx->ReportingReportedPages += *SegmentCount * REPORTING_BLOCK_PAGES;
        *SegmentCount = 0;
        return TRUE;
    }

    TraceEvents(TRACE_LEVEL_ERROR, DBG_REPORTING, "Failed to report free pages\n");
    return FALSE;
}

/*
 * Reads the optional EnableFpr value from the driver service Parameters
 * registry key. 1 (default) negotiates VIRTIO_BALLOON_F_PAGE_REPORTING
 * whenever the device offers it and no IOMMU is in the way, 0 keeps the
 * driver to the traditional balloon behavior - an escape hatch for
 * deployments that want free page reporting off without touching the
 * host-side device configuration.
 */
BOOLEAN ReportingIsEnabled(IN WDFDEVICE Device)
{
    DECLARE_CONST_UNICODE_STRING(valueName, L"EnableFpr");
    WDFKEY parametersKey = NULL;
    ULONG enable = 1;

    if (NT_SUCCESS(WdfDriverOpenParametersRegistryKey(WdfDeviceGetDriver(Device),
                                                      KEY_READ,
                                                      WDF_NO_OBJECT_ATTRIBUTES,
                                                      &parametersKey)))
    {
        WdfRegistryQueryULong(parametersKey, &valueName, &enable);
        WdfObjectDelete(parametersKey);
    }
    return enable != 0;
}

/*
 * Reads the optional MinFreeMb value from the driver service Parameters
 * registry key and converts it to the watermark override in pages, clamped
 * to [64MB, RAM/2]. A missing or zero value keeps the automatic default.
 */
static VOID ReportingReadWatermarkOverride(IN WDFDEVICE Device, IN PDEVICE_CONTEXT devCtx)
{
    DECLARE_CONST_UNICODE_STRING(valueName, L"MinFreeMb");
    WDFKEY parametersKey = NULL;
    ULONG minFreeMb = 0;

    devCtx->ReportingMinFreePages = 0;

    if (NT_SUCCESS(WdfDriverOpenParametersRegistryKey(WdfDeviceGetDriver(Device),
                                                      KEY_READ,
                                                      WDF_NO_OBJECT_ATTRIBUTES,
                                                      &parametersKey)))
    {
        if (NT_SUCCESS(WdfRegistryQueryULong(parametersKey, &valueName, &minFreeMb)) && minFreeMb != 0)
        {
            ULONGLONG pages = (ULONGLONG)minFreeMb * 1024 * 1024 / PAGE_SIZE;

            if (pages < REPORTING_HARD_MIN_AVAILABLE_PAGES)
            {
                pages = REPORTING_HARD_MIN_AVAILABLE_PAGES;
            }
            if (devCtx->ReportingTotalPages != 0 && pages > devCtx->ReportingTotalPages / 2)
            {
                pages = devCtx->ReportingTotalPages / 2;
            }
            devCtx->ReportingMinFreePages = (ULONG)pages;

            TraceEvents(TRACE_LEVEL_INFORMATION,
                        DBG_REPORTING,
                        "MinFreeMb=%u override, watermark %u pages\n",
                        minFreeMb,
                        devCtx->ReportingMinFreePages);
        }
        WdfObjectDelete(parametersKey);
    }
}

NTSTATUS
BalloonReportInitialize(IN WDFDEVICE Device)
{
    PDEVICE_CONTEXT devCtx = GetDeviceContext(Device);
    SYSTEM_BASIC_INFORMATION basicInfo;
    ULONG outLen = 0;

    PAGED_CODE();

    TraceEvents(TRACE_LEVEL_INFORMATION, DBG_REPORTING, "--> %s\n", __FUNCTION__);

    devCtx->ReportingMdlList.Next = NULL;
    devCtx->ReportingMdlCount = 0;
    devCtx->ReportingHeldPages = 0;
    devCtx->ReportingReportedPages = 0;

    RtlZeroMemory(&basicInfo, sizeof(basicInfo));
    if (!NT_SUCCESS(ZwQuerySystemInformation(SystemBasicInformation, &basicInfo, sizeof(basicInfo), &outLen)))
    {
        basicInfo.NumberOfPhysicalPages = 0;
    }
    devCtx->ReportingTotalPages = basicInfo.NumberOfPhysicalPages;

    ReportingReadWatermarkOverride(Device, devCtx);

    /* per-request segment limit: the negotiated reporting virtqueue size,
     * capped by the on-stack segment array bound */
    devCtx->ReportingMaxSegments = min(virtqueue_get_vring_size(devCtx->RepVirtQueue), REPORTING_MAX_SEGMENTS);
    TraceEvents(TRACE_LEVEL_INFORMATION,
                DBG_REPORTING,
                "Reporting virtqueue size %u, per-request limit %u segments\n",
                virtqueue_get_vring_size(devCtx->RepVirtQueue),
                devCtx->ReportingMaxSegments);

    TraceEvents(TRACE_LEVEL_INFORMATION, DBG_REPORTING, "<-- %s\n", __FUNCTION__);
    return STATUS_SUCCESS;
}

VOID BalloonReportReleaseAll(IN WDFOBJECT WdfDevice)
{
    PDEVICE_CONTEXT devCtx = GetDeviceContext(WdfDevice);

    TraceEvents(TRACE_LEVEL_INFORMATION, DBG_REPORTING, "--> %s\n", __FUNCTION__);

    ReportingReleasePages(devCtx, (ULONG)-1);

    TraceEvents(TRACE_LEVEL_INFORMATION,
                DBG_REPORTING,
                "<-- %s :: %d pages still held\n",
                __FUNCTION__,
                devCtx->ReportingHeldPages);
}

/*
 * Called by the balloon worker thread once per reporting cycle. Allocates
 * free pages in 2MB-aligned blocks, reports them to the host and hands
 * held pages back to the guest if the guest is running low on memory.
 */
VOID BalloonReportStep(IN WDFOBJECT WdfDevice)
{
    PDEVICE_CONTEXT devCtx = GetDeviceContext(WdfDevice);
    ULONG availablePages = 0;
    ULONG commitHeadroomPages = 0;
    ULONG commitLimitPages = 0;
    ULONG allocWatermark;
    ULONG batches = 0;
    VIO_SG segments[REPORTING_MAX_SEGMENTS];
    ULONG segmentCount = 0;

    TraceEvents(TRACE_LEVEL_VERBOSE, DBG_REPORTING, "--> %s\n", __FUNCTION__);

    if (devCtx->RepVirtQueue == NULL || devCtx->SurpriseRemoval || devCtx->bShutDown)
    {
        return;
    }

    if (!NT_SUCCESS(ReportingQueryMemoryState(&availablePages, &commitHeadroomPages, &commitLimitPages)))
    {
        return;
    }

    allocWatermark = ReportingAllocWatermark(devCtx);

    TraceEvents(TRACE_LEVEL_VERBOSE,
                DBG_REPORTING,
                "State: available %lu pages, watermark %lu pages, commit headroom %lu of %lu pages, %lu pages held\n",
                availablePages,
                allocWatermark,
                commitHeadroomPages,
                commitLimitPages,
                devCtx->ReportingHeldPages);

#ifndef BALLOON_INFLATE_IGNORE_LOWMEM
    if (IsLowMemory(WdfDevice))
    {
        TraceEvents(TRACE_LEVEL_WARNING,
                    DBG_REPORTING,
                    "Low memory condition, releasing %d held pages\n",
                    devCtx->ReportingHeldPages);
        BalloonReportReleaseAll(WdfDevice);
        return;
    }
#endif // !BALLOON_INFLATE_IGNORE_LOWMEM

    if (devCtx->ReportingMdlCount != 0 && ReportingCommitHeadroomLow(devCtx, commitHeadroomPages))
    {
        TraceEvents(TRACE_LEVEL_WARNING,
                    DBG_REPORTING,
                    "Commit headroom low (%d of %d pages), releasing half of the held pages\n",
                    commitHeadroomPages,
                    commitLimitPages);
        ReportingReleasePages(devCtx, devCtx->ReportingMdlCount / 2 + 1);
        return;
    }

    if (devCtx->ReportingMdlCount != 0 && availablePages < allocWatermark / 2)
    {
        TraceEvents(TRACE_LEVEL_WARNING,
                    DBG_REPORTING,
                    "Available memory low (%d pages), releasing half of the held pages\n",
                    availablePages);
        ReportingReleasePages(devCtx, devCtx->ReportingMdlCount / 2 + 1);
        return;
    }

    if (availablePages <= allocWatermark)
    {
        TraceEvents(TRACE_LEVEL_VERBOSE,
                    DBG_REPORTING,
                    "Available %lu pages <= watermark %lu pages, idle\n",
                    availablePages,
                    allocWatermark);
        return;
    }

    while (batches < REPORTING_BATCHES_PER_CYCLE)
    {
        PHYSICAL_ADDRESS LowAddress;
        PHYSICAL_ADDRESS HighAddress;
        PHYSICAL_ADDRESS SkipBytes;
        PPAGE_LIST_ENTRY pageListEntry;
        PMDL mdl;

        /* re-check the available memory and commit headroom while filling up */
        if (!NT_SUCCESS(ReportingQueryMemoryState(&availablePages, &commitHeadroomPages, &commitLimitPages)) ||
            availablePages <= allocWatermark || ReportingCommitHeadroomLow(devCtx, commitHeadroomPages))
        {
            break;
        }

        /* flush the pending segments, the next batch produces up to
         * ReportingMaxSegments more (one segment per 2MB block) */
        if (!ReportingFlushSegments(devCtx, segments, &segmentCount))
        {
            return;
        }

        LowAddress.QuadPart = 0;
        HighAddress.QuadPart = (ULONGLONG)-1;
        SkipBytes.QuadPart = REPORTING_BLOCK_SIZE;

        /* MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS with SkipBytes = 2MB
         * guarantees that every chunk in the returned MDL is exactly 2MB
         * long and aligned on a 2MB boundary, preferably taken from the
         * system's large page cache. MM_DONT_ZERO_ALLOCATION keeps the
         * pages clean on the host, the cache type is irrelevant as the
         * pages are never mapped. The batch matches the per-request
         * segment limit, so a full batch always fits into one request. */
        mdl = MmAllocatePagesForMdlEx(LowAddress,
                                      HighAddress,
                                      SkipBytes,
                                      (ULONGLONG)devCtx->ReportingMaxSegments * REPORTING_BLOCK_SIZE,
                                      MmCached,
                                      MM_DONT_ZERO_ALLOCATION | MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS);
        if (mdl == NULL || MmGetMdlByteCount(mdl) == 0)
        {
            /* allocation failure means memory pressure or no more aligned
             * blocks, stop for this cycle */
            if (mdl != NULL)
            {
                MmFreePagesFromMdl(mdl);
                ExFreePool(mdl);
            }
            break;
        }

        pageListEntry = (PPAGE_LIST_ENTRY)ExAllocateFromNPagedLookasideList(&devCtx->LookAsideList);
        if (pageListEntry == NULL)
        {
            TraceEvents(TRACE_LEVEL_ERROR, DBG_REPORTING, "Failed to allocate list entry.\n");
            MmFreePagesFromMdl(mdl);
            ExFreePool(mdl);
            return;
        }

        pageListEntry->PageMdl = mdl;
        PushEntryList(&devCtx->ReportingMdlList, &pageListEntry->SingleListEntry);
        devCtx->ReportingMdlCount++;
        devCtx->ReportingHeldPages += MmGetMdlByteCount(mdl) >> PAGE_SHIFT;

        ReportingExtractBlocks(mdl, segments, &segmentCount);

        TraceEvents(TRACE_LEVEL_VERBOSE,
                    DBG_REPORTING,
                    "Batch %u: %u blocks, %u pending segments, %u pages held\n",
                    batches,
                    MmGetMdlByteCount(mdl) / REPORTING_BLOCK_SIZE,
                    segmentCount,
                    devCtx->ReportingHeldPages);
        batches++;
    }

    if (!ReportingFlushSegments(devCtx, segments, &segmentCount))
    {
        return;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION,
                DBG_REPORTING,
                "Held %d pages, reported %d pages in total\n",
                devCtx->ReportingHeldPages,
                devCtx->ReportingReportedPages);

    TraceEvents(TRACE_LEVEL_VERBOSE, DBG_REPORTING, "<-- %s\n", __FUNCTION__);
}

#ifndef BALLOON_INFLATE_IGNORE_LOWMEM
/*
 * Dedicated watch thread for the LowMemoryCondition kernel event. The
 * memory manager signals the event the moment the system enters a low
 * memory state; instead of waiting for the next reporting cycle, the
 * thread wakes the balloon worker immediately so that it can hand the
 * held pages back. The event stays signaled for as long as the condition
 * holds, so while it is signaled the thread re-checks its state at a
 * fixed interval instead of busy waiting. The worker's periodic check
 * remains the fallback if the thread cannot be created.
 */
VOID BalloonReportLowMemWatchRoutine(IN PVOID pContext)
{
    WDFOBJECT Device = (WDFOBJECT)pContext;
    PDEVICE_CONTEXT devCtx = GetDeviceContext(Device);
    PVOID waitObjects[2];
    LARGE_INTEGER oneSecond;
    LARGE_INTEGER zeroTimeout;

    oneSecond.QuadPart = -10000; /* 1s, relative */
    zeroTimeout.QuadPart = 0;

    waitObjects[0] = devCtx->evLowMem;
    waitObjects[1] = &devCtx->WatchStopEvent;

    for (;;)
    {
        NTSTATUS status = KeWaitForMultipleObjects(2, waitObjects, WaitAny, Executive, KernelMode, FALSE, NULL, NULL);

        if (status != STATUS_WAIT_0 || devCtx->bShutDown)
        {
            break; /* stop event signaled or shutdown */
        }

        /* low memory condition: wake the worker, it releases the pages */
        TraceEvents(TRACE_LEVEL_WARNING, DBG_REPORTING, "LowMemoryCondition set, waking the worker\n");
        KeSetEvent(&devCtx->WakeUpThread, EVENT_INCREMENT, FALSE);

        while (devCtx->bShutDown == FALSE &&
               KeWaitForSingleObject(devCtx->evLowMem, Executive, KernelMode, FALSE, &zeroTimeout) == STATUS_WAIT_0)
        {
            KeDelayExecutionThread(KernelMode, FALSE, &oneSecond);
        }

        if (devCtx->bShutDown == FALSE)
        {
            TraceEvents(TRACE_LEVEL_INFORMATION, DBG_REPORTING, "LowMemoryCondition cleared\n");
        }
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, DBG_REPORTING, "Low memory watch thread exiting\n");
    PsTerminateSystemThread(STATUS_SUCCESS);
}

NTSTATUS BalloonReportCreateLowMemWatch(IN WDFDEVICE Device)
{
    PDEVICE_CONTEXT devCtx = GetDeviceContext(Device);
    NTSTATUS status;
    HANDLE hThread = 0;
    OBJECT_ATTRIBUTES oa;

    if (devCtx->LowMemWatchThread != NULL)
    {
        return STATUS_SUCCESS;
    }

    InitializeObjectAttributes(&oa, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
    status = PsCreateSystemThread(&hThread,
                                  THREAD_ALL_ACCESS,
                                  &oa,
                                  NULL,
                                  NULL,
                                  BalloonReportLowMemWatchRoutine,
                                  Device);
    if (!NT_SUCCESS(status))
    {
        TraceEvents(TRACE_LEVEL_ERROR,
                    DBG_REPORTING,
                    "Failed to create the low memory watch thread, status 0x%08x\n",
                    status);
        return status;
    }

    status = ObReferenceObjectByHandle(hThread,
                                       THREAD_ALL_ACCESS,
                                       NULL,
                                       KernelMode,
                                       (PVOID *)&devCtx->LowMemWatchThread,
                                       NULL);
    if (!NT_SUCCESS(status))
    {
        TraceEvents(TRACE_LEVEL_ERROR, DBG_REPORTING, "Failed to reference the watch thread, status 0x%08x\n", status);
        KeSetEvent(&devCtx->WatchStopEvent, EVENT_INCREMENT, FALSE);
        ZwWaitForSingleObject(hThread, FALSE, NULL);
    }
    ZwClose(hThread);
    return status;
}

NTSTATUS BalloonReportCloseLowMemWatch(IN WDFDEVICE Device)
{
    PDEVICE_CONTEXT devCtx = GetDeviceContext(Device);
    NTSTATUS status = STATUS_SUCCESS;

    if (devCtx->LowMemWatchThread != NULL)
    {
        KeSetEvent(&devCtx->WatchStopEvent, EVENT_INCREMENT, FALSE);
        status = KeWaitForSingleObject(devCtx->LowMemWatchThread, Executive, KernelMode, FALSE, NULL);
        if (!NT_SUCCESS(status))
        {
            TraceEvents(TRACE_LEVEL_ERROR, DBG_REPORTING, "Watch thread join failed, status 0x%08x\n", status);
        }
        ObDereferenceObject(devCtx->LowMemWatchThread);
        devCtx->LowMemWatchThread = NULL;
    }
    return status;
}
#endif // !BALLOON_INFLATE_IGNORE_LOWMEM
