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

    // for first page adjust the start and size of the MDL.
    // It would be better to span the MDL on entire page and
    // create the NBL with offset. But in 2 NDIS tests (RSS and
    // SendReceiveReply) the protocol driver fails to recognize
    // the packet pattern because it is looking for it in wrong
    // place, i.e. the driver fails to process the NB with offset
    // that is not zero. TODO: open the bug report.
    for (i = PARANDIS_FIRST_RX_DATA_PAGE; i < p->NumPages; i++)
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

    for (i = 0; i < m_NetMaxReceiveBuffers; ++i)
    {
        pRxNetDescriptor pBuffersDescriptor = CreateRxDescriptorOnInit();
        if (!pBuffersDescriptor)
        {
            break;
        }

        pBuffersDescriptor->Queue = this;

        if (!AddRxBufferToQueue(pBuffersDescriptor))
        {
            ParaNdis_FreeRxBufferDescriptor(m_Context, pBuffersDescriptor);
            break;
        }

        InsertTailList(&m_NetReceiveBuffers, &pBuffersDescriptor->listEntry);

        DumpDescriptor(pBuffersDescriptor, i == 0 ? 2 : 7);

        m_NetNofReceiveBuffers++;
    }
    m_NetMaxReceiveBuffers = m_NetNofReceiveBuffers;

    RecalculateLimits();

    if (m_Context->extraStatistics.minFreeRxBuffers == 0 ||
        m_Context->extraStatistics.minFreeRxBuffers > m_NetNofReceiveBuffers)
    {
        m_Context->extraStatistics.minFreeRxBuffers = m_NetNofReceiveBuffers;
    }
    m_Reinsert = true;

    return nRet;
}

pRxNetDescriptor CParaNdisRX::CreateRxDescriptorOnInit()
{
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
        }
        else
        {
            ParaNdis_FreeRxBufferDescriptor(m_Context, pBuffersDescriptor);
        }
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
            
            if (numBuffers > 1)
            {
                // This is a multi-buffer packet, handle merge logic
                if (ProcessMergedBuffers(pBufferDescriptor, nCurrCpuReceiveQueue))
                {
                    // Successfully processed merged packet, continue to next buffer
                    continue;
                }
                else
                {
                    // Error in merge processing, treat as single buffer
                    DPrintf(0, "Merge processing failed, reverting to single buffer mode");
                    // Fall through to single buffer processing
                }
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

BOOLEAN CParaNdisRX::ProcessMergedBuffers(pRxNetDescriptor pFirstBuffer, CCHAR nCurrCpuReceiveQueue)
{
    virtio_net_hdr_mrg_rxbuf *pHeader = (virtio_net_hdr_mrg_rxbuf *)pFirstBuffer->PhysicalPages[pFirstBuffer->HeaderPage].Virtual;
    UINT16 numBuffers = pHeader->num_buffers;
    
    // Validate buffer count with enhanced error handling
    if (numBuffers < 2 || numBuffers > VIRTIO_NET_MAX_MRG_BUFS)
    {
        DPrintf(0, "Invalid buffer count in merge request: %d", numBuffers);
        m_Context->extraStatistics.framesMergeErrors++;
        // Ensure graceful failure handling
        return FALSE;
    }
    
    // Clean up any existing merge context (timeout case)
    if (m_MergeContext.IsActive && IsMergeContextTimedOut())
    {
        DPrintf(0, "Merge context timeout detected, performing cleanup");
        CleanupMergeContext();
    }
    
    // Initialize new merge context
    if (!m_MergeContext.IsActive)
    {
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
    m_MergeContext.TotalPacketLength += pFirstBuffer->PacketInfo.dataLength;
    
    // Try to collect remaining buffers
    if (CollectMergeBuffers(pFirstBuffer))
    {
        // All buffers collected, assemble the packet
        pRxNetDescriptor pAssembledBuffer = AssembleMergedPacket();
        if (pAssembledBuffer)
        {
            // Update statistics
            m_Context->extraStatistics.framesMergedTotal++;
            if (m_MergeContext.CollectedBuffers > m_Context->extraStatistics.framesMergeMaxBuffers)
            {
                m_Context->extraStatistics.framesMergeMaxBuffers = m_MergeContext.CollectedBuffers;
            }
            
#if DBG
            // Debug logging for merge buffer performance monitoring
            LARGE_INTEGER currentInterruptTime;
            KeQuerySystemTime(&currentInterruptTime);
            DPrintf(3, "Merged packet: %d buffers, %d bytes total, sequence time: %I64d", 
                    m_MergeContext.CollectedBuffers, 
                    m_MergeContext.TotalPacketLength,
                    (currentInterruptTime.QuadPart - m_MergeContext.FirstBufferTimestamp.QuadPart) / 10); // in microseconds
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
    
    // Try to collect remaining buffers from the queue
    while (m_MergeContext.CollectedBuffers < m_MergeContext.ExpectedBuffers)
    {
        // Fast path: skip timeout check for first few buffers to improve performance
        if (m_MergeContext.CollectedBuffers > MERGE_BUFFER_FAST_COLLECT_COUNT && 
            IsMergeContextTimedOut())
        {
        DPrintf(0, "Merge buffer collection has timed out");
            m_Context->extraStatistics.framesMergeTimeouts++;
            return FALSE;
        }
        
        // Try to get next buffer
        pBufferDescriptor = (pRxNetDescriptor)m_VirtQueue.GetBuf(&nFullLength);
        if (!pBufferDescriptor)
        {
            // No more buffers available yet - this is normal for incomplete sequences
            return FALSE;
        }
        
        RemoveEntryList(&pBufferDescriptor->listEntry);
        m_NetNofReceiveBuffers--;
        
        // Add to merge context with enhanced bounds checking and resource limits
        if (m_MergeContext.CollectedBuffers < VIRTIO_NET_MAX_MRG_BUFS && 
            m_MergeContext.CollectedBuffers < m_MergeContext.ExpectedBuffers)
        {
            m_MergeContext.BufferSequence[m_MergeContext.CollectedBuffers] = pBufferDescriptor;
            m_MergeContext.CollectedBuffers++;
            
            // Calculate actual data length (subtract header size for non-first buffers)
            UINT32 dataLength = (m_MergeContext.CollectedBuffers == 1) ? 
                nFullLength - m_Context->nVirtioHeaderSize : nFullLength;
            m_MergeContext.TotalPacketLength += dataLength;
        }
        else
        {
            // Resource limit enforcement - too many buffers detected
            DPrintf(0, "Excessive merge buffers detected: %d", m_MergeContext.CollectedBuffers);
            m_Context->extraStatistics.framesMergeErrors++;
            // Graceful failure handling with resource cleanup
            ReuseReceiveBufferNoLock(pBufferDescriptor);
            return FALSE;
        }
    }
    
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
    
    // For single buffer case, no chaining needed
    if (m_MergeContext.CollectedBuffers == 1)
    {
        return pAssembledBuffer;
    }
    
    // Multi-buffer case: create MDL chain
    PMDL pPreviousMDL = NULL;
    PMDL pCurrentMDL = pAssembledBuffer->Holder;
    
    // Find the end of the first buffer's MDL chain
    while (pCurrentMDL)
    {
        pPreviousMDL = pCurrentMDL;
        pCurrentMDL = NDIS_MDL_LINKAGE(pCurrentMDL);
    }
    
    // Chain additional buffers' data pages (skip headers)
    for (UINT i = 1; i < m_MergeContext.CollectedBuffers; i++)
    {
        pRxNetDescriptor pAdditionalBuffer = m_MergeContext.BufferSequence[i];
        if (!pAdditionalBuffer)
        {
            continue;
        }
        
        // Create MDLs for data pages of additional buffers (skip header page)
        for (USHORT pageIdx = PARANDIS_FIRST_RX_DATA_PAGE; pageIdx < pAdditionalBuffer->NumPages; pageIdx++)
        {
            if (pAdditionalBuffer->PhysicalPages[pageIdx].Virtual)
            {
                PMDL pNewMDL = NdisAllocateMdl(m_Context->MiniportHandle,
                                               pAdditionalBuffer->PhysicalPages[pageIdx].Virtual,
                                               pAdditionalBuffer->PhysicalPages[pageIdx].size);
                if (pNewMDL)
                {
                    if (pPreviousMDL)
                    {
                        NDIS_MDL_LINKAGE(pPreviousMDL) = pNewMDL;
                    }
                    else
                    {
                        // This should not happen as first buffer should have MDLs
                        pAssembledBuffer->Holder = pNewMDL;
                    }
                    pPreviousMDL = pNewMDL;
                }
                else
                {
                    DPrintf(0, "MDL allocation failed during merge buffer assembly");
                    // Continue with partial assembly
                }
            }
        }
        
        // Mark additional buffer's MDL as NULL to prevent double-free
        // We've created new MDLs pointing to the same physical memory
        pAdditionalBuffer->Holder = NULL;
    }
    
    return pAssembledBuffer;
}

void CParaNdisRX::CleanupMergeContext()
{
    if (m_MergeContext.IsActive)
    {
        // Return any collected buffers to the free pool (except the first one if successful)
        for (UINT i = 1; i < m_MergeContext.CollectedBuffers; i++)
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
