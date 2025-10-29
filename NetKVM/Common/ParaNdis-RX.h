#pragma once
#include "ParaNdis-VirtQueue.h"
#include "ParaNdis-AbstractPath.h"

class CParaNdisRX : public CParaNdisTemplatePath<CVirtQueue>, public CNdisAllocatable<CParaNdisRX, 'XRHR'>
{
  public:
    CParaNdisRX();
    ~CParaNdisRX();

    bool Create(PPARANDIS_ADAPTER Context, UINT DeviceQueueIndex);

    BOOLEAN AddRxBufferToQueue(pRxNetDescriptor pBufferDescriptor);

    void PopulateQueue();

    void FreeRxDescriptorsFromList();

    void ReuseReceiveBuffer(pRxNetDescriptor pBuffersDescriptor)
    {
        TPassiveSpinLocker autoLock(m_Lock);

        ReuseReceiveBufferNoLock(pBuffersDescriptor);
    }

    BOOLEAN IsRxBuffersShortage()
    {
        return m_NetNofReceiveBuffers < m_MinRxBufferLimit;
    }

    VOID ProcessRxRing(CCHAR nCurrCpuReceiveQueue);

    BOOLEAN RestartQueue();

    void Shutdown()
    {
        TPassiveSpinLocker autoLock(m_Lock);

        m_VirtQueue.Shutdown();
        m_Reinsert = false;
    }

    void KickRXRing();

    PARANDIS_RECEIVE_QUEUE &UnclassifiedPacketsQueue()
    {
        return m_UnclassifiedPacketsQueue;
    }
    UINT GetFreeRxBuffers() const
    {
        return m_NetNofReceiveBuffers;
    }
    BOOLEAN AllocateMore();

  private:
    /* list of Rx buffers available for data (under VIRTIO management) */
    LIST_ENTRY m_NetReceiveBuffers;
    UINT m_NetNofReceiveBuffers = 0;
    UINT m_NetMaxReceiveBuffers = 0;
    UINT m_MinRxBufferLimit;

    UINT m_nReusedRxBuffersCounter = 0;
    UINT m_nReusedRxBuffersLimit = 0;

    bool m_Reinsert = true;

    PARANDIS_RECEIVE_QUEUE m_UnclassifiedPacketsQueue;

    // Maximum mergeable packet size per VirtIO spec: 65562 bytes (including virtio header)
    // This requires 17 PAGE-sized buffers to accommodate
    #define VIRTIO_NET_MAX_MRG_BUFS 17

    // Merge buffer support structures
    struct _MergeBufferContext
    {
        pRxNetDescriptor BufferSequence[VIRTIO_NET_MAX_MRG_BUFS];
        UINT32 BufferActualLengths[VIRTIO_NET_MAX_MRG_BUFS];  // Actual received length for each buffer
        UINT16 ExpectedBuffers;
        UINT16 CollectedBuffers;
        UINT32 TotalPacketLength;
    } m_MergeContext;

    void ReuseReceiveBufferNoLock(pRxNetDescriptor pBuffersDescriptor);
    pRxNetDescriptor ProcessMergedBuffers(pRxNetDescriptor pFirstBuffer, UINT nFullLength);
    BOOLEAN CollectMergeBuffers();
    pRxNetDescriptor AssembleMergedPacket();
    void ReturnCollectedBuffers();
    void ProcessReceivedPacket(pRxNetDescriptor pBufferDescriptor, CCHAR nCurrCpuReceiveQueue);
    void TraceMergeableStatistics();  // Debug function to trace mergeable buffer statistics

  private:
    int PrepareReceiveBuffers();
    pRxNetDescriptor CreateRxDescriptorOnInit();
    pRxNetDescriptor CreateMergeableRxDescriptor();  // Simplified descriptor for mergeable buffers
    void RecalculateLimits();
};

#ifdef PARANDIS_SUPPORT_RSS
VOID ParaNdis_ResetRxClassification(PARANDIS_ADAPTER *pContext);
#endif
