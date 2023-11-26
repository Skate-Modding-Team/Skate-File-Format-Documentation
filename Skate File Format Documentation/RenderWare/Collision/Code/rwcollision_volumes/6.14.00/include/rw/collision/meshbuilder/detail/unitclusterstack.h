// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTERSTACK_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTERSTACK_H

/*************************************************************************************************************

File: unitclusterstack.h

Purpose: UnitClusterStack class.

A container which deals with allocating UnitClusters in a stack like way.

Initialize must be called before use, with at least the requirements specified by GetMemoryRequirements.

*/

#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/meshbuilder/detail/unitcluster.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class UnitClusterStack
{
public:

    /*
    Maximum size of a single final unit's data supported by this custom version of the ClusteredMeshBuilder.
    */
    static const uint32_t MAX_FINAL_UNIT_DATA_SIZE = 13u;

    /*
    Default Constructor.
    */
    UnitClusterStack()
        : m_rootClusterNode(NULL)
        , m_currentClusterNode(NULL)
        , m_peakClusterNode(NULL)
        , m_unitClusterCount(0)
        , unitClusterListNodeArray(NULL)
        , m_unitIdListBase(NULL)
        , m_sizeUnitList(0)
        , m_allocator(NULL)
        , m_isValid(FALSE)
    {
    }

    struct UnitClusterListNode
    {
        UnitClusterListNode()
            : m_unitCluster()
            , m_previousNode(NULL)
            , m_nextNode(NULL)
        {
        }

        UnitCluster m_unitCluster;
        UnitClusterListNode *m_previousNode;
        UnitClusterListNode *m_nextNode;
    };

    class ClusterIterator
    {
    public:

        ClusterIterator(UnitClusterListNode * node)
            : m_currentNode(node)
        {
        }

        /**
        \brief Prefix increment operator overload
        \return a ForwardIterator
        */
        ClusterIterator & operator++()
        {
            if (NULL != m_currentNode)
            {
                m_currentNode = m_currentNode->m_nextNode;
            }

            return *this;
        }

        ClusterIterator & operator--()
        {
            if (NULL != m_currentNode)
            {
                m_currentNode = m_currentNode->m_previousNode;
            }

            return *this;
        }

        UnitCluster* operator*() const
        {
            if (NULL != m_currentNode)
            {
                return &m_currentNode->m_unitCluster;
            }

            return NULL;
        }

        /**
        \brief Equality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are equal
        */
        bool operator==(const ClusterIterator & other) const
        {
            return !(*this != other);
        }

        /**
        \brief Inequality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are not equal
        */
        bool operator!=(const ClusterIterator & other) const
        {
            if(other.m_currentNode != m_currentNode)
            {
                return true;
            }

            return false;
        }

    private:
        UnitClusterListNode * m_currentNode;
    };

    class ReverseClusterIterator
    {
    public:

        ReverseClusterIterator(UnitClusterListNode * node)
            : m_currentNode(node)
        {
        }

        /**
        \brief Prefix increment operator overload
        \return a ForwardIterator
        */
        ReverseClusterIterator & operator++()
        {
            if (NULL != m_currentNode)
            {
                m_currentNode = m_currentNode->m_previousNode;
            }

            return *this;
        }

        ReverseClusterIterator & operator--()
        {
            if (NULL != m_currentNode)
            {
                m_currentNode = m_currentNode->m_nextNode;
            }

            return *this;
        }

        UnitCluster* operator*()
        {
            if (NULL != m_currentNode)
            {
                return &m_currentNode->m_unitCluster;
            }

            return NULL;
        }

        /**
        \brief Equality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are equal
        */
        bool operator==(const ReverseClusterIterator & other) const
        {
            return !(*this != other);
        }

        /**
        \brief Inequality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are not equal
        */
        bool operator!=(const ReverseClusterIterator & other) const
        {
            if(other.m_currentNode != m_currentNode)
            {
                return true;
            }

            return false;
        }

    private:

        UnitClusterListNode * m_currentNode;
    };

    ClusterIterator Begin()
    {
        return ClusterIterator(m_rootClusterNode);
    }

    ClusterIterator End()
    {
        if (NULL != m_currentClusterNode)
        {
            if (NULL != m_currentClusterNode->m_nextNode)
            {
                return ClusterIterator(m_currentClusterNode->m_nextNode);
            }
        }

        return ClusterIterator(NULL);
    }

    ClusterIterator Begin() const
    {
        return ClusterIterator(m_rootClusterNode);
    }

    ClusterIterator End() const
    {
        if (NULL != m_currentClusterNode)
        {
            if (NULL != m_currentClusterNode->m_nextNode)
            {
                return ClusterIterator(m_currentClusterNode->m_nextNode);
            }
        }

        return ClusterIterator(NULL);
    }

    ReverseClusterIterator RBegin()
    {
        return ReverseClusterIterator(m_currentClusterNode);
    }

    ReverseClusterIterator REnd()
    {
        return ReverseClusterIterator(NULL);
    }

    ReverseClusterIterator RBegin() const
    {
        return ReverseClusterIterator(m_currentClusterNode);
    }

    ReverseClusterIterator REnd() const
    {
        return ReverseClusterIterator(NULL);
    }

    /*
    Initialize method must be called before first use.
    */
    void Initialize(EA::Allocator::ICoreAllocator * alloc, uint32_t numUnits);

    void Release();

    UnitCluster *GetUnitCluster();

    void MergeLastTwoClusters();

    void RemoveLastCluster();

    /*
    Returns the current number of used clusters
    */
    uint32_t Size() const
    {
        return m_unitClusterCount;
    }

    /*
    Returns a bool indicating whether or not the UnitClusterStack is in a valid state.
    */
    RwpBool IsValid() const
    {
        return m_isValid;
    }

    /*
    Returns a value indicating the peak memory use of the Allocator.
    */
    uint32_t GetMemUsed() const
    {
        return (sizeof(UnitCluster) * m_unitClusterCount + m_sizeUnitList);
    }

private:

    // UnitCluster container
    UnitClusterListNode *m_rootClusterNode;
    UnitClusterListNode *m_currentClusterNode;
    UnitClusterListNode *m_peakClusterNode;

    uint32_t m_unitClusterCount;

    UnitClusterListNode ** unitClusterListNodeArray;

    // UnitId buffer pointers
    uint32_t *m_unitIdListBase;
    uint32_t m_sizeUnitList;

    // Allocator
    EA::Allocator::ICoreAllocator * m_allocator;

    RwpBool m_isValid;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // #define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTERSTACK_H
