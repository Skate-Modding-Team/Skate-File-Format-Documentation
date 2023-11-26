// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/unitclusterstack.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


void UnitClusterStack::Initialize(EA::Allocator::ICoreAllocator * alloc, uint32_t numUnits)
{
    m_allocator = alloc;

    // Determine allocation requirements for UnitList
    m_sizeUnitList = numUnits * sizeof(uint32_t);

    uint8_t * buffer = static_cast<uint8_t*>(m_allocator->Alloc(m_sizeUnitList, NULL, 0, 4));

    if(NULL != buffer)
    {
        // Initialize the unitIdList buffer
        m_unitIdListBase = reinterpret_cast<uint32_t*>(buffer);

        m_isValid = TRUE;
    }
    else
    {
        m_isValid = FALSE;
    }
}


void UnitClusterStack::Release()
{
    if (m_isValid)
    {

        UnitClusterListNode* temp = m_peakClusterNode;

        if (NULL != unitClusterListNodeArray)
        {
            m_allocator->Free(unitClusterListNodeArray);
        }

        while (NULL != temp->m_previousNode)
        {
            UnitClusterListNode* del = temp;
            temp = temp->m_previousNode;
            m_allocator->Free(del);
        }
        m_allocator->Free(temp);

        // Deallocate unitIdList and finalUnitData
        m_allocator->Free(m_unitIdListBase);

        m_isValid = FALSE;
    }
}


UnitCluster *UnitClusterStack::GetUnitCluster()
{
    // If no clusters have been allocated
    if (m_peakClusterNode == NULL)
    {
        m_rootClusterNode = static_cast<UnitClusterListNode*>(m_allocator->Alloc(sizeof(UnitClusterListNode), NULL, 0, 4u));
        if (NULL == m_rootClusterNode)
        {
            m_isValid = FALSE;
            return NULL;
        }
        else
        {
            ++m_unitClusterCount;

            m_rootClusterNode = new (m_rootClusterNode) UnitClusterListNode();
            m_rootClusterNode->m_unitCluster.Reset(0 ,m_unitIdListBase);
            m_rootClusterNode->m_previousNode = NULL;
            m_rootClusterNode->m_nextNode = NULL;

            m_peakClusterNode = m_currentClusterNode = m_rootClusterNode;
        }
        return &m_rootClusterNode->m_unitCluster;
    }
    // If there are spare clusters in the list
    else if (m_currentClusterNode != m_peakClusterNode)
    {
        ++m_unitClusterCount;

        UnitCluster *unitCluster = &m_currentClusterNode->m_nextNode->m_unitCluster;
        UnitCluster *previousUnitCluster = &m_currentClusterNode->m_unitCluster;

        m_currentClusterNode = m_currentClusterNode->m_nextNode;

        unitCluster->Reset(previousUnitCluster->clusterID + 1,
                           previousUnitCluster->unitIDs + previousUnitCluster->numUnits);

        return unitCluster;
    }
    // If there are no spare clusters in the list
    else
    {
        // Allocate a new cluster
        UnitClusterListNode* newNode = static_cast<UnitClusterListNode*>(m_allocator->Alloc(sizeof(UnitClusterListNode), NULL, 0, 4u));
        if (NULL == newNode)
        {
            m_isValid = FALSE;
            return NULL;
        }
        else
        {
            ++m_unitClusterCount;

            newNode = new (newNode) UnitClusterListNode();
            UnitCluster *previousUnitCluster = &m_currentClusterNode->m_unitCluster;

            newNode->m_unitCluster.Reset(previousUnitCluster->clusterID + 1,
                                         previousUnitCluster->unitIDs + previousUnitCluster->numUnits);

            newNode->m_previousNode = m_peakClusterNode;

            m_peakClusterNode->m_nextNode = newNode;

            m_peakClusterNode = m_currentClusterNode = newNode;
            return &m_peakClusterNode->m_unitCluster;
        }
    }
}


void UnitClusterStack::MergeLastTwoClusters()
{
    // merge g1 unitID list into g0 unitID list
    if (NULL != m_currentClusterNode && NULL != m_currentClusterNode->m_previousNode)
    {
        // Extend the collection of unit IDs of the penultimate cluster.
        m_currentClusterNode->m_previousNode->m_unitCluster.numUnits += m_currentClusterNode->m_unitCluster.numUnits;
        // Remove the last cluster
        RemoveLastCluster();
    }
}


void UnitClusterStack::RemoveLastCluster()
{
    EA_ASSERT_MSG(NULL != m_rootClusterNode, ("No Clusters have been allocated"));
    m_currentClusterNode = m_currentClusterNode->m_previousNode;
    --m_unitClusterCount;
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

