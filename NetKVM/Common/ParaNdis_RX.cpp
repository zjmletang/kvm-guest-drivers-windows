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
    for (i = 0; i < p->NumOwnedPages; i++)
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
    CleanupMergeContext(TRUE);
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
        DPrintf(0, "ERROR: Failed to allocate memory for RX descriptor");
        return NULL;
    }

    NdisZeroMemory(p, sizeof(*p));

    // For mergeable buffers: 1 or 2 buffer descriptors depending on combineHeaderAndData
    ULONG sgArraySize = m_Context->bAnyLayout ? 1 : 2;
    p->BufferSGArray = (struct VirtIOBufferDescriptor *)ParaNdis_AllocateMemory(m_Context, 
                                                                               sizeof(VirtIOBufferDescriptor) * sgArraySize);
    if (p->BufferSGArray == NULL)
    {
        DPrintf(0, "ERROR: Failed to allocate SG array (size=%u)", sgArraySize);
        goto error_exit;
    }

    // Single physical page allocation (arrange PhysicalPages to match ParaNdis_BindRxBufferToPacket expectations)
    // ParaNdis_BindRxBufferToPacket always starts from PARANDIS_FIRST_RX_DATA_PAGE (1)
    // So PhysicalPages[0] should be header, PhysicalPages[1] should be data
    p->PhysicalPages = (tCompletePhysicalAddress *)ParaNdis_AllocateMemory(m_Context, 
                                                                           sizeof(tCompletePhysicalAddress) * 2);
    if (p->PhysicalPages == NULL)
    {
        DPrintf(0, "ERROR: Failed to allocate PhysicalPages array");
        goto error_exit;
    }

    NdisZeroMemory(p->PhysicalPages, sizeof(tCompletePhysicalAddress) * 2);

    // Allocate exactly 1 page for mergeable buffer
    if (!ParaNdis_InitialAllocatePhysicalMemory(m_Context, PAGE_SIZE, &p->PhysicalPages[0]))
    {
        DPrintf(0, "ERROR: Failed to allocate physical memory (1 page = %u bytes)", PAGE_SIZE);
        goto error_exit;
    }

    DPrintf(4, "Allocated 1 page: Virtual=%p, Physical=0x%llx", 
            p->PhysicalPages[0].Virtual, p->PhysicalPages[0].Physical.QuadPart);

    // Setup for mergeable buffer - arrange to match ParaNdis_BindRxBufferToPacket expectations
    p->NumPages = 2;  // Always 2: PhysicalPages[0]=header, PhysicalPages[1]=data
    p->HeaderPage = 0;  // Header at PhysicalPages[0]
    p->DataStartOffset = 0;  // For subsequent buffers: full buffer is data (no header skip)
    p->NumOwnedPages = p->NumPages;
    
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
        
        DPrintf(4, "Combined layout: SGLength=1, BufferSize=%u (full buffer)",
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
        
        DPrintf(4, "Separate layout: SGLength=2, HeaderSize=%u, DataSize=%u",
                m_Context->nVirtioHeaderSize, PAGE_SIZE);
    }

    // No indirect area for mergeable buffers
    p->IndirectArea.Physical.QuadPart = 0;
    p->IndirectArea.Virtual = NULL;
    p->IndirectArea.size = 0;

    p->NumOwnedPages = p->NumPages;

    if (!ParaNdis_BindRxBufferToPacket(m_Context, p))
    {
        DPrintf(0, "ERROR: Failed to bind RX buffer to packet");
        goto error_exit;
    }

    DPrintf(4, "Successfully created mergeable RX descriptor: NumPages=%u, SGLength=%u",
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
        DPrintf(4, "Using mergeable buffer allocation (small buffers to encourage merging)");
        return CreateMergeableRxDescriptor();
    }
    
    DPrintf(4, "Using traditional buffer allocation (large buffers)");;
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

    p->NumOwnedPages = p->NumPages;

    if (!ParaNdis_BindRxBufferToPacket(m_Context, p))
    {
        goto error_exit;
    }

    // Mark owned pages for cleanup (do not count borrowed pages from merge)
    p->NumOwnedPages = p->NumPages;

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
        DPrintf(4, "Reusing merged packet with %u additional buffers", 
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
        
        // CRITICAL: Restore the first buffer to its original state
        // The PhysicalPages array and MDL chain were expanded during AssembleMergedPacket
        if (m_Context->bUseMergedBuffers)
        {
            // Step 1: Free the extended MDL chain created during merge assembly
            // The original buffer's MDL still exists and will be reused
            // We only need to free MDLs for pages beyond the original 2 pages
            PMDL pMDL = pBuffersDescriptor->Holder;
            USHORT mdlCount = 0;
            
            // Skip the first buffer's original MDL (which covers the original 2 pages)
            while (pMDL && mdlCount < 1)
            {
                pMDL = NDIS_MDL_LINKAGE(pMDL);
                mdlCount++;
            }
            
            // Free extended MDLs (for merged additional buffers)
            while (pMDL)
            {
                PMDL pNextMDL = NDIS_MDL_LINKAGE(pMDL);
                NdisFreeMdl(pMDL);
                pMDL = pNextMDL;
            }
            
            // Terminate the MDL chain after the first MDL
            pMDL = pBuffersDescriptor->Holder;
            if (pMDL)
            {
                NDIS_MDL_LINKAGE(pMDL) = NULL;
            }
            
            // Step 2: Restore NumPages and NumOwnedPages to original values (2 for mergeable)
            pBuffersDescriptor->NumPages = 2;
            pBuffersDescriptor->NumOwnedPages = 2;
            
            DPrintf(5, "Restored first buffer: NumPages=%u, NumOwnedPages=%u, extended MDLs freed",
                    pBuffersDescriptor->NumPages, pBuffersDescriptor->NumOwnedPages);
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
        DPrintf(1, "Mergeable Stats: Total=%u, MaxBuffers=%u, Errors=%u",
                m_Context->extraStatistics.framesMergedTotal,
                m_Context->extraStatistics.framesMergeMaxBuffers,
                m_Context->extraStatistics.framesMergeErrors);
        DPrintf(1, "RX Buffers: Free=%u, Max=%u, Min=%u",
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

// Common packet processing logic - extracted from ProcessRxRing and ProcessMergedBuffers
// Handles packet analysis, filtering, RSS processing, and queue assignment
void CParaNdisRX::ProcessReceivedPacket(pRxNetDescriptor pBufferDescriptor, CCHAR nCurrCpuReceiveQueue)
{
    // Get data pointer (skip virtio header)
    PVOID data = pBufferDescriptor->PhysicalPages[PARANDIS_FIRST_RX_DATA_PAGE].Virtual;
    data = RtlOffsetToPointer(data, pBufferDescriptor->DataStartOffset);

    // Analyze packet (MAC-based analysis + L3 header info)
    BOOLEAN packetAnalysisRC = ParaNdis_AnalyzeReceivedPacket(data,
                                                              pBufferDescriptor->PacketInfo.dataLength,
                                                              &pBufferDescriptor->PacketInfo);

    if (!packetAnalysisRC)
    {
        ReuseReceiveBufferNoLock(pBufferDescriptor);
        m_Context->Statistics.ifInErrors++;
        m_Context->Statistics.ifInDiscards++;
        return;
    }

    // Apply packet filtering
    if (!ShallPassPacket(m_Context, &pBufferDescriptor->PacketInfo))
    {
        ReuseReceiveBufferNoLock(pBufferDescriptor);
        m_Context->Statistics.ifInDiscards++;
        m_Context->extraStatistics.framesFilteredOut++;
        return;
    }

#ifdef PARANDIS_SUPPORT_RSS
    // RSS hash analysis
    if (m_Context->RSSParameters.RSSMode != PARANDIS_RSS_MODE::PARANDIS_RSS_DISABLED)
    {
        ParaNdis6_RSSAnalyzeReceivedPacket(&m_Context->RSSParameters, data, &pBufferDescriptor->PacketInfo);
    }

    // Determine target receive queue based on RSS
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
            // Cross-CPU packet - need to notify target queue
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
            LogRedirectedPacket(pBufferDescriptor);
        }
        else
        {
            m_Context->extraStatistics.framesRSSHits++;
        }
    }
#else
    UNREFERENCED_PARAMETER(nCurrCpuReceiveQueue);
    ParaNdis_ReceiveQueueAddBuffer(&m_UnclassifiedPacketsQueue, pBufferDescriptor);
#endif
}

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

        pRxNetDescriptor pProcessBuffer = pBufferDescriptor;

        // Mergeable buffer handling: always delegate to ProcessMergedBuffers if feature enabled
        // It will handle both single-buffer (numBuffers=1) and multi-buffer (numBuffers>1) cases
        if (m_Context->bUseMergedBuffers)
        {
            pProcessBuffer = ProcessMergedBuffers(pBufferDescriptor, nFullLength);
            if (!pProcessBuffer)
            {
                // Assembly failed, already handled internally
                continue;
            }
        }
        else
        {
            // Non-mergeable mode: set packet data length (subtract virtio header)
            pProcessBuffer->PacketInfo.dataLength = nFullLength - m_Context->nVirtioHeaderSize;
        }

        // Unified processing path for both single and merged packets
        // Note: For mergeable buffers, dataLength is already set in ProcessMergedBuffers
        ProcessReceivedPacket(pProcessBuffer, nCurrCpuReceiveQueue);
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

// ProcessMergedBuffers: Pure assembly function
// Handles both single-buffer and multi-buffer packets when mergeable feature is enabled
// Returns assembled descriptor or original descriptor (for single buffer) or NULL on failure
// Does NOT process the packet - caller is responsible for calling ProcessReceivedPacket
pRxNetDescriptor CParaNdisRX::ProcessMergedBuffers(pRxNetDescriptor pFirstBuffer, UINT nFullLength)
{
    virtio_net_hdr_mrg_rxbuf *pHeader = (virtio_net_hdr_mrg_rxbuf *)pFirstBuffer->PhysicalPages[pFirstBuffer->HeaderPage].Virtual;
    UINT16 numBuffers = pHeader->num_buffers;
    
    DPrintf(5, "ProcessMergedBuffers: numBuffers=%u, packetLength=%u", numBuffers, nFullLength);
    
    // Validate buffer count
    if (numBuffers == 0 || numBuffers > VIRTIO_NET_MAX_MRG_BUFS)
    {
        DPrintf(0, "ERROR: Invalid buffer count: %d (valid range: 1-%d)", 
                numBuffers, VIRTIO_NET_MAX_MRG_BUFS);
        m_Context->extraStatistics.framesMergeErrors++;
        ReuseReceiveBufferNoLock(pFirstBuffer);
        return NULL;
    }
    
    // Single buffer case - no assembly needed, just return the original
    if (numBuffers == 1)
    {
        DPrintf(5, "Single buffer packet (no merge required)");
        // Set packet data length for single buffer (subtract virtio header)
        pFirstBuffer->PacketInfo.dataLength = nFullLength - m_Context->nVirtioHeaderSize;
        return pFirstBuffer;
    }
    
    // Multi-buffer case - assemble the packet
    DPrintf(3, "Multi-buffer packet detected: %u buffers required", numBuffers);
    
    // Clean up any existing incomplete merge context (error recovery)
    // Per VirtIO spec, all buffers for a packet are atomically available
    // If we have an active context, it indicates a previous error
    if (m_MergeContext.IsActive)
    {
        DPrintf(0, "ERROR: Stale merge context detected, incomplete packet discarded (had %u/%u buffers)",
                m_MergeContext.CollectedBuffers, m_MergeContext.ExpectedBuffers);
        m_Context->extraStatistics.framesMergeErrors++;
        CleanupMergeContext(TRUE);
    }
    
    // Initialize new merge context
    NdisZeroMemory(&m_MergeContext, sizeof(m_MergeContext));
    m_MergeContext.ExpectedBuffers = numBuffers;
    m_MergeContext.CollectedBuffers = 0;
    m_MergeContext.TotalPacketLength = 0;
    m_MergeContext.IsActive = TRUE;
    
    // Add first buffer to merge context
    m_MergeContext.BufferSequence[m_MergeContext.CollectedBuffers] = pFirstBuffer;
    m_MergeContext.BufferActualLengths[m_MergeContext.CollectedBuffers] = nFullLength;
    m_MergeContext.CollectedBuffers++;
    // Calculate actual data length (subtract virtio header from first buffer)
    UINT32 firstBufferDataLength = nFullLength - m_Context->nVirtioHeaderSize;
    pFirstBuffer->PacketInfo.dataLength = firstBufferDataLength;
    m_MergeContext.TotalPacketLength += firstBufferDataLength;
    
    DPrintf(4, "Added first buffer (%u/%u), current total length: %u bytes",
            m_MergeContext.CollectedBuffers, m_MergeContext.ExpectedBuffers, m_MergeContext.TotalPacketLength);
    
    // Collect remaining buffers
    if (!CollectMergeBuffers())
    {
        // Failed to collect all buffers - protocol violation
        DPrintf(0, "ERROR: Incomplete buffer collection (have %u/%u) - packet corrupted",
                m_MergeContext.CollectedBuffers, m_MergeContext.ExpectedBuffers);
        m_Context->extraStatistics.framesMergeErrors++;
        CleanupMergeContext(TRUE);
        return NULL;
    }
    
    // All buffers collected, assemble the packet
    DPrintf(3, "All %u buffers collected, assembling packet (total: %u bytes)",
            m_MergeContext.CollectedBuffers, m_MergeContext.TotalPacketLength);
    
    pRxNetDescriptor pAssembledBuffer = AssembleMergedPacket();
    if (!pAssembledBuffer)
    {
        // Assembly failed
        DPrintf(0, "ERROR: Failed to assemble merged packet");
        CleanupMergeContext(TRUE);
        return NULL;
    }
    
    // Update statistics
    m_Context->extraStatistics.framesMergedTotal++;
    if (m_MergeContext.CollectedBuffers > m_Context->extraStatistics.framesMergeMaxBuffers)
    {
        m_Context->extraStatistics.framesMergeMaxBuffers = m_MergeContext.CollectedBuffers;
        DPrintf(2, "New maximum buffer count record: %u buffers", 
                m_Context->extraStatistics.framesMergeMaxBuffers);
    }
    
    DPrintf(3, "Successfully assembled packet: %u buffers -> %u bytes (total merged packets: %u)",
            m_MergeContext.CollectedBuffers, 
            m_MergeContext.TotalPacketLength,
            m_Context->extraStatistics.framesMergedTotal);
    
    CleanupMergeContext(FALSE);
    return pAssembledBuffer;
}

BOOLEAN CParaNdisRX::CollectMergeBuffers()
{
    unsigned int nFullLength;
    pRxNetDescriptor pBufferDescriptor;
    
    DPrintf(5, "Collecting remaining %u buffers (have %u, need %u)",
            m_MergeContext.ExpectedBuffers - m_MergeContext.CollectedBuffers,
            m_MergeContext.CollectedBuffers,
            m_MergeContext.ExpectedBuffers);
    
    // Per VirtIO spec: all buffers for a merged packet are atomically available
    // Collect remaining buffers - they must all be present in the queue
    while (m_MergeContext.CollectedBuffers < m_MergeContext.ExpectedBuffers)
    {
        // Get next buffer - it MUST be available per VirtIO spec
        pBufferDescriptor = (pRxNetDescriptor)m_VirtQueue.GetBuf(&nFullLength);
        if (!pBufferDescriptor)
        {
            // Buffer unavailable = protocol violation or device error
            DPrintf(0, "ERROR: Buffer %u/%u unavailable - VirtIO protocol violation",
                    m_MergeContext.CollectedBuffers + 1, m_MergeContext.ExpectedBuffers);
            m_Context->extraStatistics.framesMergeErrors++;
            return FALSE;
        }
        
        RemoveEntryList(&pBufferDescriptor->listEntry);
        m_NetNofReceiveBuffers--;
        
        DPrintf(5, "Collected buffer %u/%u: length=%u bytes",
                m_MergeContext.CollectedBuffers + 1, m_MergeContext.ExpectedBuffers, nFullLength);
        
        // Add to merge context with enhanced bounds checking and resource limits
        if (m_MergeContext.CollectedBuffers < VIRTIO_NET_MAX_MRG_BUFS && 
            m_MergeContext.CollectedBuffers < m_MergeContext.ExpectedBuffers)
        {
            m_MergeContext.BufferSequence[m_MergeContext.CollectedBuffers] = pBufferDescriptor;
            m_MergeContext.BufferActualLengths[m_MergeContext.CollectedBuffers] = nFullLength;  // Store actual received length
            m_MergeContext.CollectedBuffers++;
            
            // For subsequent buffers, all data is payload (no virtio header)
            m_MergeContext.TotalPacketLength += nFullLength;
            
            DPrintf(5, "Buffer %u added: dataLength=%u, totalLength=%u",
                    m_MergeContext.CollectedBuffers, nFullLength, m_MergeContext.TotalPacketLength);
        }
        else
        {
            // Resource limit enforcement - too many buffers detected
            DPrintf(0, "ERROR: Excessive merge buffers detected: %d (max allowed: %d)", 
                    m_MergeContext.CollectedBuffers, VIRTIO_NET_MAX_MRG_BUFS);
            m_Context->extraStatistics.framesMergeErrors++;
            // Graceful failure handling with resource cleanup
            ReuseReceiveBufferNoLock(pBufferDescriptor);
            return FALSE;
        }
    }
    
    DPrintf(4, "All %u buffers collected successfully, total packet length: %u bytes",
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
        DPrintf(0, "ERROR: Too many merged buffers %u (max inline: %u) - dropping packet",
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
            DPrintf(0, "ERROR: Failed to allocate PhysicalPages array for %u pages", totalPages);
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
        UINT32 actualBufferLength = m_MergeContext.BufferActualLengths[i];
        
        // Copy data pages (skip header page at index 0)
        // Use actual received length for all pages from subsequent buffers
        for (USHORT srcPageIdx = PARANDIS_FIRST_RX_DATA_PAGE; 
             srcPageIdx < pBuffer->NumPages && destPageIdx < totalPages; 
             srcPageIdx++, destPageIdx++)
        {
            pAssembledBuffer->PhysicalPages[destPageIdx].Virtual = pBuffer->PhysicalPages[srcPageIdx].Virtual;
            pAssembledBuffer->PhysicalPages[destPageIdx].Physical = pBuffer->PhysicalPages[srcPageIdx].Physical;
            // For mergeable buffers, subsequent buffers contain only payload data
            // Use the actual received length directly
            pAssembledBuffer->PhysicalPages[destPageIdx].size = actualBufferLength;
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
        // Use actual received length instead of allocated buffer size
        UINT32 actualBufferLength = m_MergeContext.BufferActualLengths[i];
        PMDL pNewMDL = NdisAllocateMdl(m_Context->MiniportHandle,
                                       pAdditionalBuffer->PhysicalPages[0].Virtual,
                                       actualBufferLength);
        if (!pNewMDL)
        {
            DPrintf(0, "ERROR: Failed to allocate MDL for buffer %u", i);
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
    
    DPrintf(4, "Assembled packet: %u buffers, %u total pages, %u bytes",
            m_MergeContext.CollectedBuffers, totalPages, m_MergeContext.TotalPacketLength);
    
    return pAssembledBuffer;
}

void CParaNdisRX::CleanupMergeContext(BOOLEAN returnBuffers)
{
    if (m_MergeContext.IsActive)
    {
        if (returnBuffers)
        {
            // Return any collected buffers to the free pool (error path)
            for (UINT i = 0; i < m_MergeContext.CollectedBuffers; i++)
            {
                if (m_MergeContext.BufferSequence[i])
                {
                    ReuseReceiveBufferNoLock(m_MergeContext.BufferSequence[i]);
                }
            }
        }
        // Reset context state
        NdisZeroMemory(&m_MergeContext, sizeof(m_MergeContext));
        m_MergeContext.IsActive = FALSE;
    }
}
