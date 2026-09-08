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
 */

#include "precomp.h"
#include "ntddkex.h"

#if defined(EVENT_TRACING)
#include "reporting.tmh"
#endif

#ifdef ALLOC_PRAGMA
#pragma alloc_text(PAGE, BalloonReportInitialize)
#endif // ALLOC_PRAGMA

static NTSTATUS ReportingQueryAvailablePages(OUT PULONG AvailablePages)
{
    SYSTEM_PERFORMANCE_INFORMATION perfInfo;
    ULONG outLen = 0;
    NTSTATUS status;

    RtlZeroMemory(&perfInfo, sizeof(perfInfo));
    status = ZwQuerySystemInformation(SystemPerformanceInformation, &perfInfo, sizeof(perfInfo), &outLen);
    if (NT_SUCCESS(status))
    {
        *AvailablePages = perfInfo.AvailablePages;
    }
    return status;
}

static ULONG ReportingAllocWatermark(IN PDEVICE_CONTEXT devCtx)
{
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
    ULONG allocWatermark;
    ULONG batches = 0;
    VIO_SG segments[REPORTING_MAX_SEGMENTS];
    ULONG segmentCount = 0;

    TraceEvents(TRACE_LEVEL_VERBOSE, DBG_REPORTING, "--> %s\n", __FUNCTION__);

    if (devCtx->RepVirtQueue == NULL || devCtx->SurpriseRemoval || devCtx->bShutDown)
    {
        return;
    }

    if (!NT_SUCCESS(ReportingQueryAvailablePages(&availablePages)))
    {
        return;
    }

    allocWatermark = ReportingAllocWatermark(devCtx);

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
        return;
    }

    while (batches < REPORTING_BATCHES_PER_CYCLE)
    {
        PHYSICAL_ADDRESS LowAddress;
        PHYSICAL_ADDRESS HighAddress;
        PHYSICAL_ADDRESS SkipBytes;
        PPAGE_LIST_ENTRY pageListEntry;
        PMDL mdl;

        /* re-check the available memory while filling up */
        if (!NT_SUCCESS(ReportingQueryAvailablePages(&availablePages)) || availablePages <= allocWatermark)
        {
            break;
        }

        /* flush the pending segments, the next batch produces up to
         * REPORTING_BATCH_BYTES / REPORTING_BLOCK_SIZE more */
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
         * pages are never mapped. */
        mdl = MmAllocatePagesForMdlEx(LowAddress,
                                      HighAddress,
                                      SkipBytes,
                                      REPORTING_BATCH_BYTES,
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
