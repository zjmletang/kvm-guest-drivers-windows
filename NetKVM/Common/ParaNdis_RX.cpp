#include "ndis56common.h"
#include "kdebugprint.h"
#include "ParaNdis_DebugHistory.h"
#include "Trace.h"
#ifdef NETKVM_WPP_ENABLED
#include "ParaNdis_RX.tmh"
#endif

// Additional includes for merge buffer support
extern "C" {
#include <ntddk.h>
}

// define as 0 to allocate all the required buffer at once
// #define INITIAL_RX_BUFFERS  0
#define INITIAL_RX_BUFFERS 16

static FORCEINLINE VOID ParaNdis_ReceiveQueueAddBuffer(PPARANDIS_RECEIVE_QUEUE pQueue, pRxNetDescriptor pBuffer)
{
    NdisInterlockedInsertTailList(&pQueue->BuffersList, &pBuffer->ReceiveQueueListEntry, &pQueue->Lock);
}

static void ParaNdis_UnbindRxBufferFromPacket(pRxNetDescriptor p)
{
    PMDL NextMdlLinkage = p->Holder;
    ULONG ulPageDescIndex = PARANDIS_FIRST_RX_DATA_PAGE;

    while (NextMdlLinkage != NULL)
    {
        PMDL pThisMDL = NextMdlLinkage;
        NextMdlLinkage = NDIS_MDL_LINKAGE(pThisMDL);

        NdisFreeMdl(pThisMDL);
        ulPageDescIndex++;
    }
}

static BOOLEAN ParaNdis_BindRxBufferToPacket(PARANDIS_ADAPTER *pContext, pRxNetDescriptor p)
{
    ULONG i, offset = p->DataStartOffset;
    PMDL *NextMdlLinkage = &p->Holder;

    // All buffers use PARANDIS_FIRST_RX_DATA_PAGE (index 1) as start page:
    // - PhysicalPages[0] = header area (or full buffer for mergeable)
    // - PhysicalPages[1] = data area (points to full buffer for mergeable subsequent buffers)
    ULONG startPage = PARANDIS_FIRST_RX_DATA_PAGE;
    
    // for first page adjust the start and size of the MDL.
    // It would be better to span the MDL on entire page and
    // create the NBL with offset. But in 2 NDIS tests (RSS and
    // SendReceiveReply) the protocol driver fails to recognize
    // the packet pattern because it is looking for it in wrong
    // place, i.e. the driver fails to process the NB with offset
    // that is not zero. TODO: open the bug report.
    for (i = startPage; i < p->NumPages; i++)
    {
        *NextMdlLinkage = NdisAllocateMdl(pContext->MiniportHandle,
                                          RtlOffsetToPointer(p->PhysicalPages[i].Virtual, offset),
                                          p->PhysicalPages[i].size - offset);
        offset = 0;

        if (*NextMdlLinkage == NULL)
        {
            goto error_exit;
        }

        NextMdlLinkage = &(NDIS_MDL_LINKAGE(*NextMdlLinkage));
    }
    *NextMdlLinkage = NULL;

    return TRUE;

error_exit:

    ParaNdis_UnbindRxBufferFromPacket(p);
    return FALSE;
}

static bool IsRegionInside(const tCompletePhysicalAddress &a1, const tCompletePhysicalAddress &a2)
{
    const LONGLONG &p1 = a1.Physical.QuadPart;
    const LONGLONG &p2 = a2.Physical.QuadPart;
    return p1 >= p2 && p1 <= p2 + a2.size;
}

static void ParaNdis_FreeRxBufferDescriptor(PARANDIS_ADAPTER *pContext, pRxNetDescriptor p)
{
    ULONG i;

    ParaNdis_UnbindRxBufferFromPacket(p);
    for (i = 0; i < p->NumPages; i++)
    {
        if (!p->PhysicalPages[i].Virtual)
        {
            break;
        }
        // do not try do free the region derived from header block
        if (i != 0 && IsRegionInside(p->PhysicalPages[i], p->PhysicalPages[0]))
        {
            continue;
        }
        ParaNdis_FreePhysicalMemory(pContext, &p->PhysicalPages[i]);
    }

    if (p->BufferSGArray)
    {
        NdisFreeMemory(p->BufferSGArray, 0, 0);
    }
    if (p->PhysicalPages)
    {
        NdisFreeMemory(p->PhysicalPages, 0, 0);
    }
    NdisFreeMemory(p, 0, 0);
}

CParaNdisRX::CParaNdisRX()
{
    InitializeListHead(&m_NetReceiveBuffers);
    
    // Initialize merge buffer context
    NdisZeroMemory(&m_MergeContext, sizeof(m_MergeContext));
    m_MergeContext.IsActive = FALSE;
}

CParaNdisRX::~CParaNdisRX()
{
    // Clean up any active merge context
    CleanupMergeContext();
}

// called during initialization
// also later during additional allocations under m_Lock
// when we update m_NetMaxReceiveBuffers, we also update
// m_nReusedRxBuffersLimit, set m_nReusedRxBuffersLimit to zero
// and kick the rx queue
void CParaNdisRX::RecalculateLimits()
{
    m_nReusedRxBuffersLimit = m_NetMaxReceiveBuffers / 4 + 1;
    m_nReusedRxBuffersCounter = 0;
    m_MinRxBufferLimit = m_NetMaxReceiveBuffers * m_Context->MinRxBufferPercent / 100;
    DPrintf(0, "m_NetMaxReceiveBuffers %d, m_MinRxBufferLimit %u", m_NetMaxReceiveBuffers, m_MinRxBufferLimit);
}

bool CParaNdisRX::Create(PPARANDIS_ADAPTER Context, UINT DeviceQueueIndex)
{
    m_Context = Context;
    m_queueIndex = (u16)DeviceQueueIndex;
    m_NetMaxReceiveBuffers = Context->bFastInit ? INITIAL_RX_BUFFERS : 0;
    if (!m_NetMaxReceiveBuffers || m_NetMaxReceiveBuffers > Context->maxRxBufferPerQueue)
    {
        m_NetMaxReceiveBuffers = Context->maxRxBufferPerQueue;
    }

    DPrintf(0, "[RX] CParaNdisRX::Create: Queue=%u, MaxBuffers=%u, MergeableBuffers=%s",
            DeviceQueueIndex, m_NetMaxReceiveBuffers, 
            Context->bUseMergedBuffers ? "ENABLED" : "DISABLED");
    
    if (Context->bUseMergedBuffers)
    {
        DPrintf(0, "[MERGEABLE] RX Queue configuration:");
        DPrintf(0, "[MERGEABLE] - VirtIO header size: %u bytes", Context->nVirtioHeaderSize);
        DPrintf(0, "[MERGEABLE] - Buffer size: %u bytes (1 page)", PAGE_SIZE);
        DPrintf(0, "[MERGEABLE] - Data space per buffer: ~%u bytes", 
                PAGE_SIZE - Context->RxLayout.ReserveForHeader);
        DPrintf(0, "[MERGEABLE] - Layout mode: %s", 
                Context->bAnyLayout ? "Combined (header+data)" : "Separate (header|data)");
        DPrintf(0, "[MERGEABLE] - Expected merge for packets > %u bytes", 
                PAGE_SIZE - Context->RxLayout.ReserveForHeader);
    }

    if (!m_VirtQueue.Create(DeviceQueueIndex, &m_Context->IODevice, m_Context->MiniportHandle))
    {
        DPrintf(0, "CParaNdisRX::Create - virtqueue creation failed");
        return false;
    }

    PrepareReceiveBuffers();

    CreatePath();

    return true;
}

static void DumpDescriptor(pRxNetDescriptor p, int level)
{
    USHORT i;
    for (i = 0; i < p->NumPages; ++i)
    {
        auto &page = p->PhysicalPages[i];
        DPrintf(level, "page[%d]: %p of %d", i, (PVOID)page.Physical.QuadPart, page.size);
    }
    for (i = 0; i < p->BufferSGLength; ++i)
    {
        auto &sg = p->BufferSGArray[i];
        DPrintf(level, "sg[%d]: %p of %d", i, (PVOID)sg.physAddr.QuadPart, sg.length);
    }
}

int CParaNdisRX::PrepareReceiveBuffers()
{
    int nRet = 0;
    UINT i;
    DEBUG_ENTRY(4);

    DPrintf(1, "[RX] PrepareReceiveBuffers: Allocating %u buffers (mergeable=%d)", 
            m_NetMaxReceiveBuffers, m_Context->bUseMergedBuffers);

    for (i = 0; i < m_NetMaxReceiveBuffers; ++i)
    {
        pRxNetDescriptor pBuffersDescriptor = CreateRxDescriptorOnInit();
        if (!pBuffersDescriptor)
        {
            DPrintf(0, "[RX] ERROR: Failed to create RX descriptor %u/%u", i, m_NetMaxReceiveBuffers);
            break;
        }

        pBuffersDescriptor->Queue = this;

        if (!AddRxBufferToQueue(pBuffersDescriptor))
        {
            DPrintf(0, "[RX] ERROR: Failed to add buffer %u to queue", i);
            ParaNdis_FreeRxBufferDescriptor(m_Context, pBuffersDescriptor);
            break;
        }

        InsertTailList(&m_NetReceiveBuffers, &pBuffersDescriptor->listEntry);

        DumpDescriptor(pBuffersDescriptor, i == 0 ? 2 : 7);

        m_NetNofReceiveBuffers++;
    }
    m_NetMaxReceiveBuffers = m_NetNofReceiveBuffers;

    RecalculateLimits();
    
    DPrintf(1, "[RX] Successfully allocated %u/%u RX buffers (mergeable=%d)", 
            m_NetNofReceiveBuffers, m_NetMaxReceiveBuffers, m_Context->bUseMergedBuffers);
    
    // Show initial mergeable buffer statistics
    if (m_Context->bUseMergedBuffers)
    {
        TraceMergeableStatistics();
    }

    if (m_Context->extraStatistics.minFreeRxBuffers == 0 ||
        m_Context->extraStatistics.minFreeRxBuffers > m_NetNofReceiveBuffers)
    {
        m_Context->extraStatistics.minFreeRxBuffers = m_NetNofReceiveBuffers;
    }
    m_Reinsert = true;

    return nRet;
}

// Simplified buffer creation for mergeable buffers - just 1 page with header + data
pRxNetDescriptor CParaNdisRX::CreateMergeableRxDescriptor()
{
    DPrintf(4, "[MERGEABLE] Creating mergeable RX descriptor (1 page buffer)");
    
    pRxNetDescriptor p = (pRxNetDescriptor)ParaNdis_AllocateMemory(m_Context, sizeof(*p));
    if (p == NULL)
    {
        DPrintf(0, "[MERGEABLE] ERROR: Failed to allocate memory for RX descriptor");
        return NULL;
    }

    NdisZeroMemory(p, sizeof(*p));

    // For mergeable buffers: 1 or 2 buffer descriptors depending on combineHeaderAndData
    ULONG sgArraySize = m_Context->bAnyLayout ? 1 : 2;
    p->BufferSGArray = (struct VirtIOBufferDescriptor *)ParaNdis_AllocateMemory(m_Context, 
                                                                               sizeof(VirtIOBufferDescriptor) * sgArraySize);
    if (p->BufferSGArray == NULL)
    {
        DPrintf(0, "[MERGEABLE] ERROR: Failed to allocate SG array (size=%u)", sgArraySize);
        goto error_exit;
    }

    // Single physical page allocation (arrange PhysicalPages to match ParaNdis_BindRxBufferToPacket expectations)
    // ParaNdis_BindRxBufferToPacket always starts from PARANDIS_FIRST_RX_DATA_PAGE (1)
    // So PhysicalPages[0] should be header, PhysicalPages[1] should be data
    p->PhysicalPages = (tCompletePhysicalAddress *)ParaNdis_AllocateMemory(m_Context, 
                                                                           sizeof(tCompletePhysicalAddress) * 2);
    if (p->PhysicalPages == NULL)
    {
        DPrintf(0, "[MERGEABLE] ERROR: Failed to allocate PhysicalPages array");
        goto error_exit;
    }

    NdisZeroMemory(p->PhysicalPages, sizeof(tCompletePhysicalAddress) * 2);

    // Allocate exactly 1 page for mergeable buffer
    if (!ParaNdis_InitialAllocatePhysicalMemory(m_Context, PAGE_SIZE, &p->PhysicalPages[0]))
    {
        DPrintf(0, "[MERGEABLE] ERROR: Failed to allocate physical memory (1 page = %u bytes)", PAGE_SIZE);
        goto error_exit;
    }

    DPrintf(4, "[MERGEABLE] Allocated 1 page: Virtual=%p, Physical=0x%llx", 
            p->PhysicalPages[0].Virtual, p->PhysicalPages[0].Physical.QuadPart);

    // Setup for mergeable buffer - arrange to match ParaNdis_BindRxBufferToPacket expectations
    p->NumPages = 2;  // Always 2: PhysicalPages[0]=header, PhysicalPages[1]=data
    p->HeaderPage = 0;  // Header at PhysicalPages[0]
    p->DataStartOffset = 0;  // For subsequent buffers: full buffer is data (no header skip)
    
    if (m_Context->bAnyLayout)
    {
        // Combined header and data in single buffer descriptor
        p->BufferSGLength = 1;
         p->DataStartOffset = (USHORT)m_Context->nVirtioHeaderSize;  // Offset past header to find data
        
        p->BufferSGArray[0].physAddr = p->PhysicalPages[0].Physical;
        p->BufferSGArray[0].length = PAGE_SIZE;
        
        // Setup PhysicalPages[1] to point to data portion for ParaNdis_BindRxBufferToPacket
        // For first buffer: this skips the virtio header area
        // For subsequent buffers: since host writes data from offset 0, we need full buffer
        p->PhysicalPages[1].Physical.QuadPart = p->PhysicalPages[0].Physical.QuadPart;
        p->PhysicalPages[1].Virtual = p->PhysicalPages[0].Virtual;
        p->PhysicalPages[1].size = PAGE_SIZE;
        
        DPrintf(4, "[MERGEABLE] Combined layout: SGLength=1, BufferSize=%u (full buffer)",
                PAGE_SIZE);
    }
    else
    {
        // Separate header and data buffer descriptors
        p->BufferSGLength = 2;
        
        // First SG entry: VirtIO header
        p->BufferSGArray[0].physAddr = p->PhysicalPages[0].Physical;
        p->BufferSGArray[0].length = m_Context->nVirtioHeaderSize;
        
        // Second SG entry: Data portion (for first buffer: after header; for subsequent: full buffer)
        p->PhysicalPages[1].Physical.QuadPart = p->PhysicalPages[0].Physical.QuadPart;
        p->PhysicalPages[1].Virtual = p->PhysicalPages[0].Virtual;
        p->PhysicalPages[1].size = PAGE_SIZE;
        
        p->BufferSGArray[1].physAddr = p->PhysicalPages[1].Physical;
        p->BufferSGArray[1].length = p->PhysicalPages[1].size;
        
        DPrintf(4, "[MERGEABLE] Separate layout: SGLength=2, HeaderSize=%u, DataSize=%u",
                m_Context->nVirtioHeaderSize, PAGE_SIZE);
    }

    // No indirect area for mergeable buffers
    p->IndirectArea.Physical.QuadPart = 0;
    p->IndirectArea.Virtual = NULL;
    p->IndirectArea.size = 0;

    if (!ParaNdis_BindRxBufferToPacket(m_Context, p))
    {
        DPrintf(0, "[MERGEABLE] ERROR: Failed to bind RX buffer to packet");
        goto error_exit;
    }

    DPrintf(4, "[MERGEABLE] Successfully created mergeable RX descriptor: NumPages=%u, SGLength=%u",
            p->NumPages, p->BufferSGLength);
    return p;

error_exit:
    ParaNdis_FreeRxBufferDescriptor(m_Context, p);
    return NULL;
}

pRxNetDescriptor CParaNdisRX::CreateRxDescriptorOnInit()
{
    // For mergeable buffers, use simplified allocation - just 1 page per buffer
    if (m_Context->bUseMergedBuffers)
    {
        DPrintf(4, "[MERGEABLE] Using mergeable buffer allocation (small buffers to encourage merging)");
        return CreateMergeableRxDescriptor();
    }
    
    DPrintf(4, "[NON-MERGEABLE] Using traditional buffer allocation (large buffers)");
    // Original complex logic for non-mergeable buffers
    // For RX packets we allocate following pages
    //   X pages needed to fit most of data payload (or all the payload)
    //   1 page or less for virtio header, indirect buffers array and the data tail if any
    //   virtio header and indirect buffers take ~300 bytes
    //   if the data tail (payload % page size) is small it is also goes to the header block
    ULONG ulNumDataPages = m_Context->RxLayout.TotalAllocationsPerBuffer; // including header block
    ULONG sgArraySize = m_Context->RxLayout.IndirectEntries;
    bool bLargeSingleAllocation = ulNumDataPages > 1 && m_Context->bRxSeparateTail == 0 &&
                                  m_Context->RxLayout.ReserveForPacketTail;

    pRxNetDescriptor p = (pRxNetDescriptor)ParaNdis_AllocateMemory(m_Context, sizeof(*p));
    if (p == NULL)
    {
        return NULL;
    }

    NdisZeroMemory(p, sizeof(*p));

    p->BufferSGArray = (struct
                        VirtIOBufferDescriptor *)ParaNdis_AllocateMemory(m_Context,
                                                                         sizeof(*p->BufferSGArray) * sgArraySize);
    if (p->BufferSGArray == NULL)
    {
        goto error_exit;
    }

    p->PhysicalPages = (tCompletePhysicalAddress *)ParaNdis_AllocateMemory(m_Context,
                                                                           sizeof(*p->PhysicalPages) * sgArraySize);
    if (p->PhysicalPages == NULL)
    {
        goto error_exit;
    }

    // must initialize for case of exit in the middle of the loop
    NdisZeroMemory(p->PhysicalPages, sizeof(*p->PhysicalPages) * sgArraySize);

    p->BufferSGLength = 0;
    p->HeaderPage = m_Context->RxLayout.ReserveForHeader ? 0 : 1;
    p->DataStartOffset = (p->HeaderPage == 0) ? 0 : (USHORT)m_Context->nVirtioHeaderSize;
    auto &pageNumber = p->NumPages;

    while (ulNumDataPages > 0)
    {
        // Allocate the first block separately, the rest can be one contiguous block
        ULONG ulPagesToAlloc = (pageNumber == 0) ? 1 : ulNumDataPages;
        ULONG sizeToAlloc = (pageNumber == 0) ? m_Context->RxLayout.HeaderPageAllocation : PAGE_SIZE * ulPagesToAlloc;
        if (pageNumber > 0 && bLargeSingleAllocation)
        {
            sizeToAlloc += m_Context->RxLayout.ReserveForPacketTail;
        }

        while (!ParaNdis_InitialAllocatePhysicalMemory(m_Context, sizeToAlloc, &p->PhysicalPages[pageNumber]))
        {
            // Retry with half the pages
            if (ulPagesToAlloc == 1)
            {
                goto error_exit;
            }
            else
            {
                ulPagesToAlloc /= 2;
                sizeToAlloc = PAGE_SIZE * ulPagesToAlloc;
                bLargeSingleAllocation = false;
            }
        }

        if (pageNumber || p->HeaderPage == 0)
        {
            p->BufferSGArray[p->BufferSGLength].physAddr = p->PhysicalPages[pageNumber].Physical;
            p->BufferSGArray[p->BufferSGLength].length = p->PhysicalPages[pageNumber].size;
            p->BufferSGLength++;
        }
        pageNumber++;
        ulNumDataPages -= ulPagesToAlloc;
    }

    // First page is for virtio header, size needs to be adjusted correspondingly
    if (p->HeaderPage == 0)
    {
        p->BufferSGArray[0].length = m_Context->nVirtioHeaderSize;
    }

    ULONG offsetInTheHeader = m_Context->RxLayout.ReserveForHeader;
    // Pre-cache indirect area addresses
    p->IndirectArea.Physical.QuadPart = p->PhysicalPages[0].Physical.QuadPart + offsetInTheHeader;
    p->IndirectArea.Virtual = RtlOffsetToPointer(p->PhysicalPages[0].Virtual, offsetInTheHeader);
    p->IndirectArea.size = m_Context->RxLayout.ReserveForIndirectArea;

    if (m_Context->RxLayout.ReserveForPacketTail && !bLargeSingleAllocation)
    {
        // the payload tail is located in the header block
        offsetInTheHeader += m_Context->RxLayout.ReserveForIndirectArea;

        // fill the tail's physical page fields
        p->PhysicalPages[pageNumber].Physical.QuadPart = p->PhysicalPages[0].Physical.QuadPart + offsetInTheHeader;
        p->PhysicalPages[pageNumber].Virtual = RtlOffsetToPointer(p->PhysicalPages[0].Virtual, offsetInTheHeader);
        p->PhysicalPages[pageNumber].size = m_Context->RxLayout.ReserveForPacketTail;

        // fill the tail's SG buffer fields
        p->BufferSGArray[p->BufferSGLength].physAddr.QuadPart = p->PhysicalPages[pageNumber].Physical.QuadPart;
        p->BufferSGArray[p->BufferSGLength].length = p->PhysicalPages[pageNumber].size;
        p->BufferSGLength++;
        pageNumber++;
    }
    else
    {
        // the payload tail is located in the full data block
        // and was already allocated and counted
    }

    if (!ParaNdis_BindRxBufferToPacket(m_Context, p))
    {
        goto error_exit;
    }

    return p;

error_exit:
    ParaNdis_FreeRxBufferDescriptor(m_Context, p);
    return NULL;
}

/* must be called on PASSIVE from system thread */
BOOLEAN CParaNdisRX::AllocateMore()
{
    BOOLEAN result = false;

    // if the queue is not ready, try again later
    if (!m_pVirtQueue->IsValid() || !m_Reinsert)
    {
        DPrintf(1, "Queue is not ready, try later");
        return true;
    }

    if (m_NetMaxReceiveBuffers >= m_Context->maxRxBufferPerQueue ||
        m_NetMaxReceiveBuffers >= m_pVirtQueue->GetRingSize())
    {
        return result;
    }
    
    pRxNetDescriptor pBuffersDescriptor = CreateRxDescriptorOnInit();

    TPassiveSpinLocker autoLock(m_Lock);

    if (pBuffersDescriptor)
    {
        pBuffersDescriptor->Queue = this;
        if (m_pVirtQueue->CanTouchHardware() && AddRxBufferToQueue(pBuffersDescriptor))
        {
            InsertTailList(&m_NetReceiveBuffers, &pBuffersDescriptor->listEntry);
            m_NetNofReceiveBuffers++;
            m_NetMaxReceiveBuffers++;
            RecalculateLimits();
            KickRXRing();
            result = true;
            
            DPrintf(4, "[RX] Successfully allocated additional buffer: total=%u, max=%u",
                    m_NetNofReceiveBuffers, m_NetMaxReceiveBuffers);
        }
        else
        {
            DPrintf(0, "[RX] ERROR: Failed to add additional buffer to queue");
            ParaNdis_FreeRxBufferDescriptor(m_Context, pBuffersDescriptor);
        }
    }
    else
    {
        DPrintf(0, "[RX] ERROR: Failed to allocate additional RX descriptor");
    }
    return result;
}

/* TODO - make it method in pRXNetDescriptor */
BOOLEAN CParaNdisRX::AddRxBufferToQueue(pRxNetDescriptor pBufferDescriptor)
{
    return 0 <=
           pBufferDescriptor->Queue->m_VirtQueue.AddBuf(pBufferDescriptor->BufferSGArray,
                                                        0,
                                                        pBufferDescriptor->BufferSGLength,
                                                        pBufferDescriptor,
                                                        m_Context->bUseIndirect ? pBufferDescriptor->IndirectArea.Virtual
                                                                                : NULL,
                                                        m_Context->bUseIndirect ? pBufferDescriptor->IndirectArea.Physical.QuadPart
                                                                                : 0);
}

void CParaNdisRX::FreeRxDescriptorsFromList()
{
    while (!IsListEmpty(&m_NetReceiveBuffers))
    {
        pRxNetDescriptor pBufferDescriptor = (pRxNetDescriptor)RemoveHeadList(&m_NetReceiveBuffers);
        ParaNdis_FreeRxBufferDescriptor(m_Context, pBufferDescriptor);
    }
}

void CParaNdisRX::ReuseReceiveBufferNoLock(pRxNetDescriptor pBuffersDescriptor)
{
    DEBUG_ENTRY(4);

    // Handle merged buffers: reuse all constituent buffers
    if (pBuffersDescriptor->MergedBufferCount > 0)
    {
        DPrintf(4, "[MERGEABLE] Reusing merged packet with %u additional buffers", 
                pBuffersDescriptor->MergedBufferCount);
        
        // Reuse additional buffers from inline array
        // Note: We created NEW MDLs for the assembled packet (in AssembleMergedPacket),
        // so the additional buffers still have their original Holder MDLs intact
        for (USHORT i = 0; i < pBuffersDescriptor->MergedBufferCount; i++)
        {
            pRxNetDescriptor pAdditionalBuffer = pBuffersDescriptor->MergedBuffersInline[i];
            if (pAdditionalBuffer)
            {
                // Additional buffers retain their original MDLs, just reuse them normally
                ReuseReceiveBufferNoLock(pAdditionalBuffer);
            }
        }
        
        // No need to free anything - inline array is part of the descriptor
        pBuffersDescriptor->MergedBufferCount = 0;
    }

    if (!m_Reinsert)
    {
        InsertTailList(&m_NetReceiveBuffers, &pBuffersDescriptor->listEntry);
        m_NetNofReceiveBuffers++;
        return;
    }
    else if (AddRxBufferToQueue(pBuffersDescriptor))
    {
        InsertTailList(&m_NetReceiveBuffers, &pBuffersDescriptor->listEntry);
        m_NetNofReceiveBuffers++;

        if (m_NetNofReceiveBuffers > m_NetMaxReceiveBuffers)
        {
            DPrintf(0,
                    "Error: m_NetNofReceiveBuffers > m_NetMaxReceiveBuffers (%d>%d)",
                    m_NetNofReceiveBuffers,
                    m_NetMaxReceiveBuffers);
        }

        /* TODO - nReusedRXBuffers per queue or per context ?*/
        if (++m_nReusedRxBuffersCounter >= m_nReusedRxBuffersLimit)
        {
            m_nReusedRxBuffersCounter = 0;
            m_VirtQueue.Kick();
        }
    }
    else
    {
        /* TODO - NetMaxReceiveBuffers per queue or per context ?*/
        DPrintf(0, "FAILED TO REUSE THE BUFFER!!!!");
        ParaNdis_FreeRxBufferDescriptor(m_Context, pBuffersDescriptor);
        m_NetMaxReceiveBuffers--;
    }
}

VOID CParaNdisRX::KickRXRing()
{
    m_VirtQueue.Kick();
}

// Add a statistics tracing function for debugging
void CParaNdisRX::TraceMergeableStatistics()
{
    if (m_Context->bUseMergedBuffers)
    {
        DPrintf(1, "[MERGEABLE STATS] Total merged packets: %u, Max buffers in packet: %u",
                m_Context->extraStatistics.framesMergedTotal,
                m_Context->extraStatistics.framesMergeMaxBuffers);
        DPrintf(1, "[MERGEABLE STATS] Errors: %u, Timeouts: %u",
                m_Context->extraStatistics.framesMergeErrors,
                m_Context->extraStatistics.framesMergeTimeouts);
        DPrintf(1, "[MERGEABLE STATS] RX buffers: %u free, %u max, %u min",
                m_NetNofReceiveBuffers,
                m_NetMaxReceiveBuffers,
                m_Context->extraStatistics.minFreeRxBuffers);
    }
}

#if PARANDIS_SUPPORT_RSS
static FORCEINLINE VOID ParaNdis_QueueRSSDpc(PARANDIS_ADAPTER *pContext,
                                             ULONG MessageIndex,
                                             PGROUP_AFFINITY pTargetAffinity)
{
    NdisMQueueDpcEx(pContext->InterruptHandle, MessageIndex, pTargetAffinity, NULL);
}

static FORCEINLINE CCHAR ParaNdis_GetScalingDataForPacket(PARANDIS_ADAPTER *pContext,
                                                          PNET_PACKET_INFO pPacketInfo,
                                                          PPROCESSOR_NUMBER pTargetProcessor)
{
    return ParaNdis6_RSSGetScalingDataForPacket(&pContext->RSSParameters, pPacketInfo, pTargetProcessor);
}
#endif

static ULONG ShallPassPacket(PARANDIS_ADAPTER *pContext, PNET_PACKET_INFO pPacketInfo)
{
    ULONG i;

    if (pPacketInfo->dataLength > pContext->MaxPacketSize.nMaxFullSizeOsRx + ETH_PRIORITY_HEADER_SIZE)
    {
        return FALSE;
    }

    if ((pPacketInfo->dataLength > pContext->MaxPacketSize.nMaxFullSizeOsRx) && !pPacketInfo->hasVlanHeader)
    {
        return FALSE;
    }

    if (IsVlanSupported(pContext) && pPacketInfo->hasVlanHeader)
    {
        if (pContext->VlanId && pContext->VlanId != pPacketInfo->Vlan.VlanId)
        {
            return FALSE;
        }
    }

    if (pContext->PacketFilter & NDIS_PACKET_TYPE_PROMISCUOUS)
    {
        return TRUE;
    }

    if (pPacketInfo->isUnicast)
    {
        ULONG Res;

        if (!(pContext->PacketFilter & NDIS_PACKET_TYPE_DIRECTED))
        {
            return FALSE;
        }

        ETH_COMPARE_NETWORK_ADDRESSES_EQ_SAFE(pPacketInfo->ethDestAddr, pContext->CurrentMacAddress, &Res);
        return !Res;
    }

    if (pPacketInfo->isBroadcast)
    {
        return (pContext->PacketFilter & NDIS_PACKET_TYPE_BROADCAST);
    }

    // Multi-cast

    if (pContext->PacketFilter & NDIS_PACKET_TYPE_ALL_MULTICAST)
    {
        return TRUE;
    }

    if (!(pContext->PacketFilter & NDIS_PACKET_TYPE_MULTICAST))
    {
        return FALSE;
    }

    for (i = 0; i < pContext->MulticastData.nofMulticastEntries; i++)
    {
        ULONG Res;
        PUCHAR CurrMcastAddr = &pContext->MulticastData.MulticastList[i * ETH_ALEN];

        ETH_COMPARE_NETWORK_ADDRESSES_EQ_SAFE(pPacketInfo->ethDestAddr, CurrMcastAddr, &Res);

        if (!Res)
        {
            return TRUE;
        }
    }

    return FALSE;
}

#define LogRedirectedPacket(p)
#if !defined(LogRedirectedPacket)
static void LogRedirectedPacket(pRxNetDescriptor pBufferDescriptor)
{
    NET_PACKET_INFO *pi = &pBufferDescriptor->PacketInfo;
    LPCSTR packetType = "Unknown";
    IPv4Header *pIp4Header = NULL;
    TCPHeader *pTcpHeader = NULL;
    UDPHeader *pUdpHeader = NULL;
    // IPv6Header *pIp6Header = NULL;
    switch (pi->RSSHash.Type)
    {
        case NDIS_HASH_TCP_IPV4:
            packetType = "TCP_IPV4";
            pIp4Header = (IPv4Header *)RtlOffsetToPointer(pi->headersBuffer, pi->L2HdrLen);
            pTcpHeader = (TCPHeader *)RtlOffsetToPointer(pIp4Header, pi->L3HdrLen);
            break;
        case NDIS_HASH_IPV4:
            packetType = "IPV4";
            pIp4Header = (IPv4Header *)RtlOffsetToPointer(pi->headersBuffer, pi->L2HdrLen);
            break;
        case NDIS_HASH_TCP_IPV6:
            packetType = "TCP_IPV6";
            break;
        case NDIS_HASH_TCP_IPV6_EX:
            packetType = "TCP_IPV6EX";
            break;
        case NDIS_HASH_IPV6_EX:
            packetType = "IPV6EX";
            break;
        case NDIS_HASH_IPV6:
            packetType = "IPV6";
            break;
#if (NDIS_SUPPORT_NDIS680)
        case NDIS_HASH_UDP_IPV4:
            packetType = "UDP_IPV4";
            pIp4Header = (IPv4Header *)RtlOffsetToPointer(pi->headersBuffer, pi->L2HdrLen);
            pUdpHeader = (UDPHeader *)RtlOffsetToPointer(pIp4Header, pi->L3HdrLen);
            break;
        case NDIS_HASH_UDP_IPV6:
            packetType = "UDP_IPV6";
            break;
        case NDIS_HASH_UDP_IPV6_EX:
            packetType = "UDP_IPV6EX";
            break;
#endif
        default:
            break;
    }
    if (pTcpHeader)
    {
        TraceNoPrefix(0,
                      "%s: %s %d.%d.%d.%d:%d->%d.%d.%d.%d:%d\n",
                      __FUNCTION__,
                      packetType,
                      pIp4Header->ip_srca[0],
                      pIp4Header->ip_srca[1],
                      pIp4Header->ip_srca[2],
                      pIp4Header->ip_srca[3],
                      RtlUshortByteSwap(pTcpHeader->tcp_src),
                      pIp4Header->ip_desta[0],
                      pIp4Header->ip_desta[1],
                      pIp4Header->ip_desta[2],
                      pIp4Header->ip_desta[3],
                      RtlUshortByteSwap(pTcpHeader->tcp_dest));
    }
    else if (pUdpHeader)
    {
        TraceNoPrefix(0,
                      "%s: %s %d.%d.%d.%d:%d->%d.%d.%d.%d:%d\n",
                      __FUNCTION__,
                      packetType,
                      pIp4Header->ip_srca[0],
                      pIp4Header->ip_srca[1],
                      pIp4Header->ip_srca[2],
                      pIp4Header->ip_srca[3],
                      RtlUshortByteSwap(pUdpHeader->udp_src),
                      pIp4Header->ip_desta[0],
                      pIp4Header->ip_desta[1],
                      pIp4Header->ip_desta[2],
                      pIp4Header->ip_desta[3],
                      RtlUshortByteSwap(pUdpHeader->udp_dest));
    }
    else if (pIp4Header)
    {
        TraceNoPrefix(0,
                      "%s: %s %d.%d.%d.%d(%d)->%d.%d.%d.%d\n",
                      __FUNCTION__,
                      packetType,
                      pIp4Header->ip_srca[0],
                      pIp4Header->ip_srca[1],
                      pIp4Header->ip_srca[2],
                      pIp4Header->ip_srca[3],
                      pIp4Header->ip_protocol,
                      pIp4Header->ip_desta[0],
                      pIp4Header->ip_desta[1],
                      pIp4Header->ip_desta[2],
                      pIp4Header->ip_desta[3]);
    }
    else
    {
        TraceNoPrefix(0, "%s: %s\n", __FUNCTION__, packetType);
    }
}
#endif

VOID CParaNdisRX::ProcessRxRing(CCHAR nCurrCpuReceiveQueue)
{
    pRxNetDescriptor pBufferDescriptor;
    unsigned int nFullLength;

#ifndef PARANDIS_SUPPORT_RSS
    UNREFERENCED_PARAMETER(nCurrCpuReceiveQueue);
#endif

    TDPCSpinLocker autoLock(m_Lock);

    if (m_Context->extraStatistics.minFreeRxBuffers > m_NetNofReceiveBuffers)
    {
        m_Context->extraStatistics.minFreeRxBuffers = m_NetNofReceiveBuffers;
    }

    while (NULL != (pBufferDescriptor = (pRxNetDescriptor)m_VirtQueue.GetBuf(&nFullLength)))
    {
        RemoveEntryList(&pBufferDescriptor->listEntry);
        m_NetNofReceiveBuffers--;

        // Check for merge buffer scenario if feature is enabled
        if (m_Context->bUseMergedBuffers)
        {
            // Check if this is a merged buffer packet
            virtio_net_hdr_mrg_rxbuf *pHeader = (virtio_net_hdr_mrg_rxbuf *)pBufferDescriptor->PhysicalPages[pBufferDescriptor->HeaderPage].Virtual;
            UINT16 numBuffers = pHeader->num_buffers;
            
            DPrintf(5, "[MERGEABLE] Received buffer: numBuffers=%u, packetLength=%u", 
                    numBuffers, nFullLength);
            
            if (numBuffers > 1)
            {
                DPrintf(3, "[MERGEABLE] Multi-buffer packet detected: %u buffers required", numBuffers);
                // This is a multi-buffer packet, handle merge logic
                if (ProcessMergedBuffers(pBufferDescriptor, nFullLength, nCurrCpuReceiveQueue))
                {
                    // Successfully processed merged packet, continue to next buffer
                    DPrintf(4, "[MERGEABLE] Successfully processed merged packet");
                    continue;
                }
                else
                {
                    // Error in merge processing, treat as single buffer
                    DPrintf(0, "[MERGEABLE] Merge processing failed, reverting to single buffer mode");
                    // Fall through to single buffer processing
                }
            }
            else
            {
                DPrintf(5, "[MERGEABLE] Single buffer packet (no merge required)");
            }
            // numBuffers == 1 or merge processing failed, handle as single buffer
        }

        // Single buffer processing (original logic)
        PVOID data = pBufferDescriptor->PhysicalPages[PARANDIS_FIRST_RX_DATA_PAGE].Virtual;
        data = RtlOffsetToPointer(data, pBufferDescriptor->DataStartOffset);

        // basic MAC-based analysis + L3 header info
        BOOLEAN packetAnalysisRC = ParaNdis_AnalyzeReceivedPacket(data,
                                                                  nFullLength - m_Context->nVirtioHeaderSize,
                                                                  &pBufferDescriptor->PacketInfo);

        if (!packetAnalysisRC)
        {
            pBufferDescriptor->Queue->ReuseReceiveBufferNoLock(pBufferDescriptor);
            m_Context->Statistics.ifInErrors++;
            m_Context->Statistics.ifInDiscards++;
            continue;
        }

        // filtering based on prev stage analysis
        if (!ShallPassPacket(m_Context, &pBufferDescriptor->PacketInfo))
        {
            pBufferDescriptor->Queue->ReuseReceiveBufferNoLock(pBufferDescriptor);
            m_Context->Statistics.ifInDiscards++;
            m_Context->extraStatistics.framesFilteredOut++;
            continue;
        }
#ifdef PARANDIS_SUPPORT_RSS
        if (m_Context->RSSParameters.RSSMode != PARANDIS_RSS_MODE::PARANDIS_RSS_DISABLED)
        {
            ParaNdis6_RSSAnalyzeReceivedPacket(&m_Context->RSSParameters, data, &pBufferDescriptor->PacketInfo);
        }
        CCHAR nTargetReceiveQueueNum;
        GROUP_AFFINITY TargetAffinity;
        PROCESSOR_NUMBER TargetProcessor;

        nTargetReceiveQueueNum = ParaNdis_GetScalingDataForPacket(m_Context,
                                                                  &pBufferDescriptor->PacketInfo,
                                                                  &TargetProcessor);

        if (nTargetReceiveQueueNum == PARANDIS_RECEIVE_UNCLASSIFIED_PACKET)
        {
            ParaNdis_ReceiveQueueAddBuffer(&m_UnclassifiedPacketsQueue, pBufferDescriptor);
            m_Context->extraStatistics.framesRSSUnclassified++;
        }
        else
        {
            ParaNdis_ReceiveQueueAddBuffer(&m_Context->ReceiveQueues[nTargetReceiveQueueNum], pBufferDescriptor);

            if (nTargetReceiveQueueNum != nCurrCpuReceiveQueue)
            {
                if (m_Context->bPollModeEnabled)
                {
                    // ensure the NDIS just schedules the other poll and does not do anything
                    // otherwise if both polls are configured to the same CPU
                    // this may cause a deadlock in return nbl path
                    KIRQL prev = KeRaiseIrqlToSynchLevel();
                    ParaNdisPollNotify(m_Context, nTargetReceiveQueueNum, "RSS");
                    KeLowerIrql(prev);
                }
                else
                {
                    ParaNdis_ProcessorNumberToGroupAffinity(&TargetAffinity, &TargetProcessor);
                    ParaNdis_QueueRSSDpc(m_Context, m_messageIndex, &TargetAffinity);
                }
                m_Context->extraStatistics.framesRSSMisses++;
                LogRedirectedPacket(pBufferDescriptor);
            }
            else
            {
                m_Context->extraStatistics.framesRSSHits++;
            }
        }
#else
        ParaNdis_ReceiveQueueAddBuffer(&m_UnclassifiedPacketsQueue, pBufferDescriptor);
#endif
    }
}

void CParaNdisRX::PopulateQueue()
{
    LIST_ENTRY TempList;
    TPassiveSpinLocker autoLock(m_Lock);

    InitializeListHead(&TempList);

    while (!IsListEmpty(&m_NetReceiveBuffers))
    {
        pRxNetDescriptor pBufferDescriptor = (pRxNetDescriptor)RemoveHeadList(&m_NetReceiveBuffers);
        InsertTailList(&TempList, &pBufferDescriptor->listEntry);
    }
    m_NetNofReceiveBuffers = 0;
    while (!IsListEmpty(&TempList))
    {
        pRxNetDescriptor pBufferDescriptor = (pRxNetDescriptor)RemoveHeadList(&TempList);
        if (AddRxBufferToQueue(pBufferDescriptor))
        {
            InsertTailList(&m_NetReceiveBuffers, &pBufferDescriptor->listEntry);
            m_NetNofReceiveBuffers++;
        }
        else
        {
            /* TODO - NetMaxReceiveBuffers should take into account all queues */
            DPrintf(0, "FAILED TO REUSE THE BUFFER!!!!");
            ParaNdis_FreeRxBufferDescriptor(m_Context, pBufferDescriptor);
            m_NetMaxReceiveBuffers--;
        }
    }
    m_Reinsert = true;
}

BOOLEAN CParaNdisRX::RestartQueue()
{
    return ParaNdis_SynchronizeWithInterrupt(m_Context, m_messageIndex, RestartQueueSynchronously, this);
}

#ifdef PARANDIS_SUPPORT_RSS
VOID ParaNdis_ResetRxClassification(PARANDIS_ADAPTER *pContext)
{
    ULONG i;

    for (i = PARANDIS_FIRST_RSS_RECEIVE_QUEUE; i < ARRAYSIZE(pContext->ReceiveQueues); i++)
    {
        PPARANDIS_RECEIVE_QUEUE pCurrQueue = &pContext->ReceiveQueues[i];
        NdisAcquireSpinLock(&pCurrQueue->Lock);

        while (!IsListEmpty(&pCurrQueue->BuffersList))
        {
            PLIST_ENTRY pListEntry = RemoveHeadList(&pCurrQueue->BuffersList);
            pRxNetDescriptor pBufferDescriptor = CONTAINING_RECORD(pListEntry, RxNetDescriptor, ReceiveQueueListEntry);
            ParaNdis_ReceiveQueueAddBuffer(&pBufferDescriptor->Queue->UnclassifiedPacketsQueue(), pBufferDescriptor);
        }

        NdisReleaseSpinLock(&pCurrQueue->Lock);
    }
}
#endif

//
// Merge Buffer Implementation Functions
//

BOOLEAN CParaNdisRX::ProcessMergedBuffers(pRxNetDescriptor pFirstBuffer, UINT nFullLength, CCHAR nCurrCpuReceiveQueue)
{
    virtio_net_hdr_mrg_rxbuf *pHeader = (virtio_net_hdr_mrg_rxbuf *)pFirstBuffer->PhysicalPages[pFirstBuffer->HeaderPage].Virtual;
    UINT16 numBuffers = pHeader->num_buffers;
    
    DPrintf(3, "[MERGEABLE] ProcessMergedBuffers: Starting merge for %u buffers", numBuffers);
    
    // Validate buffer count with enhanced error handling
    if (numBuffers < 2 || numBuffers > VIRTIO_NET_MAX_MRG_BUFS)
    {
        DPrintf(0, "[MERGEABLE] ERROR: Invalid buffer count in merge request: %d (valid range: 2-%d)", 
                numBuffers, VIRTIO_NET_MAX_MRG_BUFS);
        m_Context->extraStatistics.framesMergeErrors++;
        // Ensure graceful failure handling
        return FALSE;
    }
    
    // Clean up any existing merge context (timeout case)
    if (m_MergeContext.IsActive && IsMergeContextTimedOut())
    {
        DPrintf(0, "[MERGEABLE] WARNING: Merge context timeout detected, performing cleanup (previous sequence incomplete)");
        CleanupMergeContext();
    }
    
    // Initialize new merge context
    if (!m_MergeContext.IsActive)
    {
        DPrintf(4, "[MERGEABLE] Initializing new merge context for %u buffers", numBuffers);
        NdisZeroMemory(&m_MergeContext, sizeof(m_MergeContext));
        m_MergeContext.ExpectedBuffers = numBuffers;
        m_MergeContext.CollectedBuffers = 0;
        m_MergeContext.TotalPacketLength = 0;
        m_MergeContext.IsActive = TRUE;
        KeQuerySystemTime(&m_MergeContext.FirstBufferTimestamp);
    }
    
    // Add first buffer to merge context
    m_MergeContext.BufferSequence[m_MergeContext.CollectedBuffers] = pFirstBuffer;
    m_MergeContext.CollectedBuffers++;
    // Calculate actual data length (subtract virtio header from first buffer)
    UINT32 firstBufferDataLength = nFullLength - m_Context->nVirtioHeaderSize;
    pFirstBuffer->PacketInfo.dataLength = firstBufferDataLength;
    m_MergeContext.TotalPacketLength += firstBufferDataLength;
    
    DPrintf(4, "[MERGEABLE] Added first buffer (%u/%u), current total length: %u bytes",
            m_MergeContext.CollectedBuffers, m_MergeContext.ExpectedBuffers, m_MergeContext.TotalPacketLength);
    
    // Try to collect remaining buffers
    if (CollectMergeBuffers(pFirstBuffer))
    {
        // All buffers collected, assemble the packet
        DPrintf(3, "[MERGEABLE] All %u buffers collected, assembling packet (total: %u bytes)",
                m_MergeContext.CollectedBuffers, m_MergeContext.TotalPacketLength);
        
        pRxNetDescriptor pAssembledBuffer = AssembleMergedPacket();
        if (pAssembledBuffer)
        {
            // Update statistics
            m_Context->extraStatistics.framesMergedTotal++;
            if (m_MergeContext.CollectedBuffers > m_Context->extraStatistics.framesMergeMaxBuffers)
            {
                m_Context->extraStatistics.framesMergeMaxBuffers = m_MergeContext.CollectedBuffers;
                DPrintf(2, "[MERGEABLE] New maximum buffer count record: %u buffers", 
                        m_Context->extraStatistics.framesMergeMaxBuffers);
            }
            
            DPrintf(3, "[MERGEABLE] Successfully assembled packet: %u buffers -> %u bytes (total merged packets: %u)",
                    m_MergeContext.CollectedBuffers, 
                    m_MergeContext.TotalPacketLength,
                    m_Context->extraStatistics.framesMergedTotal);
            
#if DBG
            // Debug logging for merge buffer performance monitoring
            LARGE_INTEGER currentInterruptTime;
            KeQuerySystemTime(&currentInterruptTime);
            LONGLONG mergeTimeMicroseconds = (currentInterruptTime.QuadPart - m_MergeContext.FirstBufferTimestamp.QuadPart) / 10;
            DPrintf(4, "[MERGEABLE] Merge timing: %I64d microseconds for %u buffers", 
                    mergeTimeMicroseconds, m_MergeContext.CollectedBuffers);
#endif
            
            // Process the assembled packet through the normal path
            PVOID data = pAssembledBuffer->PhysicalPages[PARANDIS_FIRST_RX_DATA_PAGE].Virtual;
            data = RtlOffsetToPointer(data, pAssembledBuffer->DataStartOffset);
            
            BOOLEAN packetAnalysisRC = ParaNdis_AnalyzeReceivedPacket(data,
                                                                      pAssembledBuffer->PacketInfo.dataLength,
                                                                      &pAssembledBuffer->PacketInfo);
            
            if (packetAnalysisRC && ShallPassPacket(m_Context, &pAssembledBuffer->PacketInfo))
            {
                // Add to receive queue for processing
#ifdef PARANDIS_SUPPORT_RSS
                if (m_Context->RSSParameters.RSSMode != PARANDIS_RSS_MODE::PARANDIS_RSS_DISABLED)
                {
                    ParaNdis6_RSSAnalyzeReceivedPacket(&m_Context->RSSParameters, data, &pAssembledBuffer->PacketInfo);
                }
                
                CCHAR nTargetReceiveQueueNum;
                GROUP_AFFINITY TargetAffinity;
                PROCESSOR_NUMBER TargetProcessor;
                
                nTargetReceiveQueueNum = ParaNdis_GetScalingDataForPacket(m_Context,
                                                                          &pAssembledBuffer->PacketInfo,
                                                                          &TargetProcessor);
                
                if (nTargetReceiveQueueNum == PARANDIS_RECEIVE_UNCLASSIFIED_PACKET)
                {
                    ParaNdis_ReceiveQueueAddBuffer(&m_UnclassifiedPacketsQueue, pAssembledBuffer);
                    m_Context->extraStatistics.framesRSSUnclassified++;
                }
                else
                {
                    ParaNdis_ReceiveQueueAddBuffer(&m_Context->ReceiveQueues[nTargetReceiveQueueNum], pAssembledBuffer);
                    
                    if (nTargetReceiveQueueNum != nCurrCpuReceiveQueue)
                    {
                        if (m_Context->bPollModeEnabled)
                        {
                            KIRQL prev = KeRaiseIrqlToSynchLevel();
                            ParaNdisPollNotify(m_Context, nTargetReceiveQueueNum, "RSS");
                            KeLowerIrql(prev);
                        }
                        else
                        {
                            ParaNdis_ProcessorNumberToGroupAffinity(&TargetAffinity, &TargetProcessor);
                            ParaNdis_QueueRSSDpc(m_Context, m_messageIndex, &TargetAffinity);
                        }
                        m_Context->extraStatistics.framesRSSMisses++;
                    }
                    else
                    {
                        m_Context->extraStatistics.framesRSSHits++;
                    }
                }
#else
                ParaNdis_ReceiveQueueAddBuffer(&m_UnclassifiedPacketsQueue, pAssembledBuffer);
#endif
            }
            else
            {
                // Packet analysis failed or filtered out
                ReuseReceiveBufferNoLock(pAssembledBuffer);
                if (!packetAnalysisRC)
                {
                    m_Context->Statistics.ifInErrors++;
                    m_Context->Statistics.ifInDiscards++;
                }
                else
                {
                    m_Context->Statistics.ifInDiscards++;
                    m_Context->extraStatistics.framesFilteredOut++;
                }
            }
            
            CleanupMergeContext();
            return TRUE;
        }
        else
        {
            // Assembly failed
            CleanupMergeContext();
            return FALSE;
        }
    }
    
    // Still collecting buffers, this is normal
    return TRUE;
}

BOOLEAN CParaNdisRX::CollectMergeBuffers(pRxNetDescriptor pFirstBuffer)
{
    UNREFERENCED_PARAMETER(pFirstBuffer);
    
    unsigned int nFullLength;
    pRxNetDescriptor pBufferDescriptor;
    
    DPrintf(5, "[MERGEABLE] CollectMergeBuffers: Collecting remaining %u buffers (have %u, need %u)",
            m_MergeContext.ExpectedBuffers - m_MergeContext.CollectedBuffers,
            m_MergeContext.CollectedBuffers,
            m_MergeContext.ExpectedBuffers);
    
    // Try to collect remaining buffers from the queue
    while (m_MergeContext.CollectedBuffers < m_MergeContext.ExpectedBuffers)
    {
        // Fast path: skip timeout check for first few buffers to improve performance
        if (m_MergeContext.CollectedBuffers > MERGE_BUFFER_FAST_COLLECT_COUNT && 
            IsMergeContextTimedOut())
        {
            DPrintf(0, "[MERGEABLE] ERROR: Merge buffer collection timed out after %u collected (expected %u)",
                    m_MergeContext.CollectedBuffers, m_MergeContext.ExpectedBuffers);
            m_Context->extraStatistics.framesMergeTimeouts++;
            return FALSE;
        }
        
        // Try to get next buffer
        pBufferDescriptor = (pRxNetDescriptor)m_VirtQueue.GetBuf(&nFullLength);
        if (!pBufferDescriptor)
        {
            // No more buffers available yet - this is normal for incomplete sequences
            DPrintf(5, "[MERGEABLE] No more buffers available, waiting (have %u/%u)",
                    m_MergeContext.CollectedBuffers, m_MergeContext.ExpectedBuffers);
            return FALSE;
        }
        
        RemoveEntryList(&pBufferDescriptor->listEntry);
        m_NetNofReceiveBuffers--;
        
        DPrintf(5, "[MERGEABLE] Collected buffer %u/%u: length=%u bytes",
                m_MergeContext.CollectedBuffers + 1, m_MergeContext.ExpectedBuffers, nFullLength);
        
        // Add to merge context with enhanced bounds checking and resource limits
        if (m_MergeContext.CollectedBuffers < VIRTIO_NET_MAX_MRG_BUFS && 
            m_MergeContext.CollectedBuffers < m_MergeContext.ExpectedBuffers)
        {
            m_MergeContext.BufferSequence[m_MergeContext.CollectedBuffers] = pBufferDescriptor;
            m_MergeContext.CollectedBuffers++;
            
            // For subsequent buffers, all data is payload (no virtio header)
            m_MergeContext.TotalPacketLength += nFullLength;
            
            DPrintf(5, "[MERGEABLE] Buffer %u added: dataLength=%u, totalLength=%u",
                    m_MergeContext.CollectedBuffers, nFullLength, m_MergeContext.TotalPacketLength);
        }
        else
        {
            // Resource limit enforcement - too many buffers detected
            DPrintf(0, "[MERGEABLE] ERROR: Excessive merge buffers detected: %d (max allowed: %d)", 
                    m_MergeContext.CollectedBuffers, VIRTIO_NET_MAX_MRG_BUFS);
            m_Context->extraStatistics.framesMergeErrors++;
            // Graceful failure handling with resource cleanup
            ReuseReceiveBufferNoLock(pBufferDescriptor);
            return FALSE;
        }
    }
    
    DPrintf(4, "[MERGEABLE] All %u buffers collected successfully, total packet length: %u bytes",
            m_MergeContext.CollectedBuffers, m_MergeContext.TotalPacketLength);
    return TRUE;
}

pRxNetDescriptor CParaNdisRX::AssembleMergedPacket()
{
    if (!m_MergeContext.IsActive || m_MergeContext.CollectedBuffers == 0)
    {
        return NULL;
    }
    
    // Use the first buffer as the base for the assembled packet
    pRxNetDescriptor pAssembledBuffer = m_MergeContext.BufferSequence[0];
    
    // Update packet info with total length
    pAssembledBuffer->PacketInfo.dataLength = m_MergeContext.TotalPacketLength;
    
    // For single buffer case, no merging needed
    if (m_MergeContext.CollectedBuffers == 1)
    {
        pAssembledBuffer->MergedBufferCount = 0;
        return pAssembledBuffer;
    }
    
    // Multi-buffer case: save references to merged buffers using inline storage
    // Most packets use 2-4 buffers, inline array is sufficient for common case
    UINT additionalBuffers = m_MergeContext.CollectedBuffers - 1;
    
    // CRITICAL: Prevent buffer overflow - inline array has limited capacity
    if (additionalBuffers > MAX_INLINE_MERGED_BUFFERS)
    {
        DPrintf(0, "[MERGEABLE] ERROR: Too many merged buffers %u (max inline: %u) - dropping packet",
                m_MergeContext.CollectedBuffers, MAX_INLINE_MERGED_BUFFERS + 1);
        m_Context->extraStatistics.framesMergeErrors++;
        
        // Return all collected buffers to the free pool
        for (UINT i = 0; i < m_MergeContext.CollectedBuffers; i++)
        {
            if (m_MergeContext.BufferSequence[i])
            {
                ReuseReceiveBufferNoLock(m_MergeContext.BufferSequence[i]);
            }
        }
        return NULL;
    }
    
    // Copy buffer pointers to inline array (no allocation needed!)
    for (UINT i = 0; i < additionalBuffers; i++)
    {
        pAssembledBuffer->MergedBuffersInline[i] = m_MergeContext.BufferSequence[i + 1];
    }
    // MergedBufferCount = number of ADDITIONAL buffers (not including this one)
    pAssembledBuffer->MergedBufferCount = (USHORT)additionalBuffers;
    
    // Multi-buffer case: rebuild PhysicalPages array and chain existing MDLs
    // PhysicalPages MUST be updated because:
    // 1. Checksum offload calculations traverse PhysicalPages array
    // 2. Buffer cleanup (ParaNdis_FreeRxBufferDescriptor) uses NumPages to free memory
    // 3. PhysicalPages must match MDL chain for correctness
    USHORT totalPages = 0;
    
    // Count total pages across all buffers
    for (UINT i = 0; i < m_MergeContext.CollectedBuffers; i++)
    {
        pRxNetDescriptor pBuffer = m_MergeContext.BufferSequence[i];
        if (i == 0)
        {
            // First buffer: include all pages
            totalPages += pBuffer->NumPages;
        }
        else
        {
            // Subsequent buffers: skip header page, only count data pages
            totalPages += (pBuffer->NumPages > PARANDIS_FIRST_RX_DATA_PAGE) ? 
                          (pBuffer->NumPages - PARANDIS_FIRST_RX_DATA_PAGE) : 0;
        }
    }
    
    // Reallocate PhysicalPages array if needed to hold all pages
    if (totalPages > pAssembledBuffer->NumPages)
    {
        tCompletePhysicalAddress *pNewPhysicalPages = 
            (tCompletePhysicalAddress *)ParaNdis_AllocateMemory(m_Context,
                                                                sizeof(tCompletePhysicalAddress) * totalPages);
        if (!pNewPhysicalPages)
        {
            DPrintf(0, "[MERGEABLE] ERROR: Failed to allocate PhysicalPages array for %u pages", totalPages);
            return NULL;
        }
        
        // Copy existing pages from first buffer
        NdisMoveMemory(pNewPhysicalPages, 
                      pAssembledBuffer->PhysicalPages, 
                      sizeof(tCompletePhysicalAddress) * pAssembledBuffer->NumPages);
        
        // Free old PhysicalPages array
        NdisFreeMemory(pAssembledBuffer->PhysicalPages, 0, 0);
        pAssembledBuffer->PhysicalPages = pNewPhysicalPages;
    }
    
    // Rebuild PhysicalPages array
    USHORT destPageIdx = pAssembledBuffer->NumPages;
    for (UINT i = 1; i < m_MergeContext.CollectedBuffers; i++)
    {
        pRxNetDescriptor pBuffer = m_MergeContext.BufferSequence[i];
        
        // Copy data pages (skip header page at index 0)
        for (USHORT srcPageIdx = PARANDIS_FIRST_RX_DATA_PAGE; 
             srcPageIdx < pBuffer->NumPages && destPageIdx < totalPages; 
             srcPageIdx++, destPageIdx++)
        {
            pAssembledBuffer->PhysicalPages[destPageIdx].Virtual = pBuffer->PhysicalPages[srcPageIdx].Virtual;
            pAssembledBuffer->PhysicalPages[destPageIdx].Physical = pBuffer->PhysicalPages[srcPageIdx].Physical;
            pAssembledBuffer->PhysicalPages[destPageIdx].size = pBuffer->PhysicalPages[srcPageIdx].size;
        }
    }
    
    // Update page count
    pAssembledBuffer->NumPages = totalPages;
    
    // Now create NEW MDLs for additional buffers covering their FULL payload
    // IMPORTANT: For subsequent buffers in mergeable mode, the ENTIRE buffer contains payload data
    // (no virtio header), so we must create MDLs covering the full buffer from offset 0,
    // NOT reuse the existing Holder MDLs which may have been created with wrong assumptions
    PMDL pPreviousMDL = NULL;
    PMDL pCurrentMDL = pAssembledBuffer->Holder;
    
    // Find the end of the first buffer's MDL chain
    while (pCurrentMDL)
    {
        pPreviousMDL = pCurrentMDL;
        pCurrentMDL = NDIS_MDL_LINKAGE(pCurrentMDL);
    }
    
    // Create new MDLs for additional buffers covering their FULL payload
    // (subsequent buffers in mergeable RX have NO virtio header, just pure data)
    for (UINT i = 1; i < m_MergeContext.CollectedBuffers; i++)
    {
        pRxNetDescriptor pAdditionalBuffer = m_MergeContext.BufferSequence[i];
        if (!pAdditionalBuffer)
        {
            continue;
        }
        
        // For subsequent buffers: create MDL for FULL buffer (no header to skip)
        // The entire buffer is payload data, starting from PhysicalPages[0]
        PMDL pNewMDL = NdisAllocateMdl(m_Context->MiniportHandle,
                                       pAdditionalBuffer->PhysicalPages[0].Virtual,
                                       pAdditionalBuffer->PhysicalPages[0].size);
        if (!pNewMDL)
        {
            DPrintf(0, "[MERGEABLE] ERROR: Failed to allocate MDL for buffer %u", i);
            // Continue with what we have - partial packet better than nothing
            continue;
        }
        
        // Chain this new MDL to the assembled packet's MDL chain
        if (pPreviousMDL)
        {
            NDIS_MDL_LINKAGE(pPreviousMDL) = pNewMDL;
        }
        else
        {
            // This should not happen as first buffer should have MDLs
            pAssembledBuffer->Holder = pNewMDL;
        }
        
        NDIS_MDL_LINKAGE(pNewMDL) = NULL;
        pPreviousMDL = pNewMDL;
        
        // Note: pAdditionalBuffer->Holder will be freed when ReuseReceiveBufferNoLock is called
        // The new MDL we created will be freed when pAssembledBuffer's packet is returned
    }
    
    DPrintf(4, "[MERGEABLE] Assembled packet: %u buffers, %u total pages, %u bytes",
            m_MergeContext.CollectedBuffers, totalPages, m_MergeContext.TotalPacketLength);
    
    return pAssembledBuffer;
}

void CParaNdisRX::CleanupMergeContext()
{
    if (m_MergeContext.IsActive)
    {
        // Note: For successfully assembled packets, the merged buffers are now owned by
        // pAssembledBuffer->MergedBuffersInline and will be freed when the NBL is returned.
        // Only clean up on error/timeout cases where assembly didn't complete.
        
        // Return any collected buffers to the free pool
        for (UINT i = 0; i < m_MergeContext.CollectedBuffers; i++)
        {
            if (m_MergeContext.BufferSequence[i])
            {
                ReuseReceiveBufferNoLock(m_MergeContext.BufferSequence[i]);
            }
        }
        
        NdisZeroMemory(&m_MergeContext, sizeof(m_MergeContext));
        m_MergeContext.IsActive = FALSE;
    }
}

BOOLEAN CParaNdisRX::IsMergeContextTimedOut()
{
    if (!m_MergeContext.IsActive)
    {
        return FALSE;
    }
    
    LARGE_INTEGER currentTime;
    KeQuerySystemTime(&currentTime);
    
    LARGE_INTEGER elapsedTime;
    elapsedTime.QuadPart = currentTime.QuadPart - m_MergeContext.FirstBufferTimestamp.QuadPart;
    
    return (elapsedTime.QuadPart > MERGE_BUFFER_TIMEOUT_TICKS);
}
