/*
 * Main include file
 * This file contains various routines and globals
 *
 * Copyright (c) 2009-2017 Red Hat, Inc.
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
#if !defined(_PROTOTYPES_H_)
#define _PROTOTYPES_H_

#include "virtio.h"
#include "public.h"
#include "trace.h"

/* The ID for virtio_balloon */
#define VIRTIO_ID_BALLOON                  5

/* The feature bitmap for virtio balloon */
#define VIRTIO_BALLOON_F_MUST_TELL_HOST    0 /* Tell before reclaiming pages */
#define VIRTIO_BALLOON_F_STATS_VQ          1 /* Memory status virtqueue */
#define VIRTIO_BALLOON_F_PAGE_REPORTING    5 /* Free page reporting virtqueue */

/*
 * Free page reporting (VIRTIO_BALLOON_F_PAGE_REPORTING) tuning parameters.
 *
 * Reported blocks are always 2MB in size and 2MB-aligned. This matches the
 * host-side transparent huge page granularity and the default reporting
 * granularity of the Linux free page reporting implementation, which only
 * reports blocks of pageblock_order and larger (order 9, i.e. 2MB, on
 * architectures with 4KB base pages). The virtio specification requires
 * the driver to "attempt to report large pages rather than smaller ones".
 *
 * The alignment is provided by MmAllocatePagesForMdlEx called with
 * MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS and SkipBytes = 2MB: the memory
 * manager returns complete 2MB blocks, each guaranteed to be exactly 2MB
 * long and aligned on a 2MB boundary, preferably taken from the system's
 * large page cache.
 */
#define REPORTING_BLOCK_SHIFT              9                              /* log2(512) */
#define REPORTING_BLOCK_PAGES              (1UL << REPORTING_BLOCK_SHIFT) /* 4KB pages per 2MB block */
#define REPORTING_BLOCK_SIZE               (REPORTING_BLOCK_PAGES << PAGE_SHIFT)

/* Total size of a single MmAllocatePagesForMdlEx call, a multiple of the
 * 2MB reporting block size (32 blocks per allocation) */
#define REPORTING_BATCH_BYTES              (32 * REPORTING_BLOCK_SIZE)
/* Max number of allocation batches per reporting cycle */
#define REPORTING_BATCHES_PER_CYCLE        8
/* Max number of 2MB blocks reported per virtqueue request (QEMU vring size) */
#define REPORTING_MAX_SEGMENTS             32
/* Reporting cycle interval, matches Linux page_reporting_delay_ms default */
#define REPORTING_INTERVAL_MS              2000

/*
 * Watermarks (in 4KB pages) controlling when pages are taken from and
 * returned to the guest. Pages are only allocated while at least an
 * eighth of the physical RAM (but never less than 256MB) remains
 * available to the guest. Once half of that amount is left, held pages
 * are handed back. This default can be overridden per deployment with
 * the MinFreeMb value in the driver service Parameters registry key,
 * clamped to [64MB, RAM/2] - in the spirit of the Linux page_reporting
 * module parameters. The mechanism itself (LowMemoryCondition handling,
 * hysteresis band, gradual release) is not configurable.
 */
#define REPORTING_AVAILABLE_FRACTION       8
#define REPORTING_MIN_AVAILABLE_PAGES      (256UL * 1024 * 1024 / PAGE_SIZE)
#define REPORTING_HARD_MIN_AVAILABLE_PAGES (64UL * 1024 * 1024 / PAGE_SIZE)

typedef struct _VIRTIO_BALLOON_CONFIG
{
    u32 num_pages;
    u32 actual;
} VIRTIO_BALLOON_CONFIG, *PVIRTIO_BALLOON_CONFIG;

typedef struct virtqueue VIOQUEUE, *PVIOQUEUE;
typedef struct VirtIOBufferDescriptor VIO_SG, *PVIO_SG;

#define __DRIVER_NAME "BALLOON: "

typedef struct
{
    SINGLE_LIST_ENTRY SingleListEntry;
    PMDL PageMdl;
} PAGE_LIST_ENTRY, *PPAGE_LIST_ENTRY;

typedef struct _DEVICE_CONTEXT
{
    WDFINTERRUPT WdfInterrupt;
    PUCHAR PortBase;
    ULONG PortCount;
    BOOLEAN PortMapped;
    BOOLEAN SurpriseRemoval;
#ifndef BALLOON_INFLATE_IGNORE_LOWMEM
    PKEVENT evLowMem;
    HANDLE hLowMem;
#endif // !BALLOON_INFLATE_IGNORE_LOWMEM
    VIRTIO_WDF_DRIVER VDevice;
    PVIOQUEUE InfVirtQueue;
    PVIOQUEUE DefVirtQueue;
    PVIOQUEUE StatVirtQueue;
    PVIOQUEUE RepVirtQueue;

    WDFSPINLOCK StatQueueLock;
    WDFSPINLOCK InfDefQueueLock;

    KEVENT HostAckEvent;

    volatile ULONG num_pages;
    ULONG num_pfns;
    PPFN_NUMBER pfns_table;
    NPAGED_LOOKASIDE_LIST LookAsideList;
    BOOLEAN bListInitialized;
    SINGLE_LIST_ENTRY PageListHead;
    PBALLOON_STAT MemStats;

    /*
     * Free page reporting state. The held MDL list is only accessed from
     * the balloon worker thread (reporting) and, after that thread has
     * been stopped, from the power-management path (release), so it needs
     * no additional lock. The reporting virtqueue itself is protected by
     * InfDefQueueLock, like the inflate and deflate queues.
     */
    ULONG ReportingTotalPages;          /* NumberOfPhysicalPages, cached */
    ULONG ReportingMinFreePages;        /* watermark override from MinFreeMb, 0 = automatic */
    SINGLE_LIST_ENTRY ReportingMdlList; /* held PAGE_LIST_ENTRY chain */
    ULONG ReportingMdlCount;
    ULONG ReportingHeldPages;     /* pages currently held */
    ULONG ReportingReportedPages; /* pages reported so far (cumulative) */

    KEVENT WakeUpThread;
    PKTHREAD Thread;
    BOOLEAN bShutDown;

#ifdef USE_BALLOON_SERVICE
    WDFREQUEST PendingWriteRequest;
    BOOLEAN HandleWriteRequest;
#else  // USE_BALLOON_SERVICE
    WDFWORKITEM StatWorkItem;
    LONG WorkCount;
#endif // USE_BALLOON_SERVICE

} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, GetDeviceContext);

#define BALLOON_MGMT_POOL_TAG 'mtlB'

#ifndef _IRQL_requires_
#define _IRQL_requires_(level)
#endif

EVT_WDF_DRIVER_DEVICE_ADD BalloonDeviceAdd;
KSTART_ROUTINE BalloonRoutine;
DRIVER_INITIALIZE DriverEntry;

// Context cleanup callbacks generally run at IRQL <= DISPATCH_LEVEL but
// WDFDRIVER and WDFDEVICE cleanup is guaranteed to run at PASSIVE_LEVEL.
// Annotate the prototypes to make static analysis happy.
EVT_WDF_OBJECT_CONTEXT_CLEANUP _IRQL_requires_(PASSIVE_LEVEL) EvtDriverContextCleanup;
EVT_WDF_DEVICE_CONTEXT_CLEANUP _IRQL_requires_(PASSIVE_LEVEL) BalloonEvtDeviceContextCleanup;

EVT_WDF_DEVICE_PREPARE_HARDWARE BalloonEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE BalloonEvtDeviceReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY BalloonEvtDeviceD0Entry;
EVT_WDF_DEVICE_D0_EXIT BalloonEvtDeviceD0Exit;
EVT_WDF_DEVICE_D0_EXIT_PRE_INTERRUPTS_DISABLED BalloonEvtDeviceD0ExitPreInterruptsDisabled;
EVT_WDF_DEVICE_SURPRISE_REMOVAL BalloonEvtDeviceSurpriseRemoval;
EVT_WDF_INTERRUPT_ISR BalloonInterruptIsr;
EVT_WDF_INTERRUPT_DPC BalloonInterruptDpc;
EVT_WDF_INTERRUPT_ENABLE BalloonInterruptEnable;
EVT_WDF_INTERRUPT_DISABLE BalloonInterruptDisable;
#ifdef USE_BALLOON_SERVICE
EVT_WDF_FILE_CLOSE BalloonEvtFileClose;
#else  // USE_BALLOON_SERVICE
EVT_WDF_WORKITEM StatWorkItemWorker;
#endif // USE_BALLOON_SERVICE

VOID BalloonInterruptDpc(IN WDFINTERRUPT WdfInterrupt, IN WDFOBJECT WdfDevice);

BOOLEAN
BalloonInterruptIsr(IN WDFINTERRUPT Interrupt, IN ULONG MessageID);

NTSTATUS
BalloonInterruptEnable(IN WDFINTERRUPT WdfInterrupt, IN WDFDEVICE WdfDevice);

NTSTATUS
BalloonInterruptDisable(IN WDFINTERRUPT WdfInterrupt, IN WDFDEVICE WdfDevice);

NTSTATUS
BalloonInit(IN WDFOBJECT WdfDevice);

VOID BalloonTerm(IN WDFOBJECT WdfDevice);

NTSTATUS
BalloonFill(IN WDFOBJECT WdfDevice, IN size_t num);

NTSTATUS
BalloonLeak(IN WDFOBJECT WdfDevice, IN size_t num);

VOID BalloonMemStats(IN WDFOBJECT WdfDevice);

NTSTATUS
BalloonTellHost(IN WDFOBJECT WdfDevice, IN PVIOQUEUE vq);

/* Free page reporting (VIRTIO_BALLOON_F_PAGE_REPORTING) routines */
BOOLEAN ReportingIsEnabled(IN WDFDEVICE Device);

NTSTATUS
BalloonReportInitialize(IN WDFDEVICE Device);

VOID BalloonReportStep(IN WDFOBJECT WdfDevice);

VOID BalloonReportReleaseAll(IN WDFOBJECT WdfDevice);

__inline VOID EnableInterrupt(IN WDFINTERRUPT WdfInterrupt, IN WDFCONTEXT Context)
{
    PDEVICE_CONTEXT devCtx = (PDEVICE_CONTEXT)Context;
    UNREFERENCED_PARAMETER(WdfInterrupt);

    virtqueue_enable_cb(devCtx->InfVirtQueue);
    virtqueue_kick(devCtx->InfVirtQueue);
    virtqueue_enable_cb(devCtx->DefVirtQueue);
    virtqueue_kick(devCtx->DefVirtQueue);

    if (devCtx->StatVirtQueue)
    {
        virtqueue_enable_cb(devCtx->StatVirtQueue);
        virtqueue_kick(devCtx->StatVirtQueue);
    }

    if (devCtx->RepVirtQueue)
    {
        virtqueue_enable_cb(devCtx->RepVirtQueue);
        virtqueue_kick(devCtx->RepVirtQueue);
    }
}

__inline VOID DisableInterrupt(IN PDEVICE_CONTEXT devCtx)
{
    virtqueue_disable_cb(devCtx->InfVirtQueue);
    virtqueue_disable_cb(devCtx->DefVirtQueue);
    if (devCtx->StatVirtQueue)
    {
        virtqueue_disable_cb(devCtx->StatVirtQueue);
    }
    if (devCtx->RepVirtQueue)
    {
        virtqueue_disable_cb(devCtx->RepVirtQueue);
    }
}

VOID BalloonSetSize(IN WDFOBJECT WdfDevice, IN size_t num);

LONGLONG
BalloonGetSize(IN WDFOBJECT WdfDevice);

NTSTATUS
BalloonCloseWorkerThread(IN WDFDEVICE Device);

VOID BalloonRoutine(IN PVOID pContext);

#ifndef BALLOON_INFLATE_IGNORE_LOWMEM
__inline BOOLEAN IsLowMemory(IN WDFOBJECT WdfDevice)
{
    LARGE_INTEGER TimeOut = {0};
    PDEVICE_CONTEXT devCtx = GetDeviceContext(WdfDevice);

    if (devCtx->evLowMem)
    {
        return (STATUS_WAIT_0 == KeWaitForSingleObject(devCtx->evLowMem, Executive, KernelMode, FALSE, &TimeOut));
    }
    return FALSE;
}
#endif // !BALLOON_INFLATE_IGNORE_LOWMEM

#ifdef USE_BALLOON_SERVICE
NTSTATUS
BalloonQueueInitialize(IN WDFDEVICE hDevice);
#else  // USE_BALLOON_SERVICE
NTSTATUS
StatInitializeWorkItem(IN WDFDEVICE Device);
#endif // USE_BALLOON_SERVICE

#endif // _PROTOTYPES_H_
