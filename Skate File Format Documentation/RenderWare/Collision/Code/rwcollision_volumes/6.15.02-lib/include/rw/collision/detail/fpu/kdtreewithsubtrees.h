// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEWITHSUBTREES_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEWITHSUBTREES_H

#include "rw/collision/common.h"
#include "rw/collision/kdtreebase.h"
#include "aabbox.h"
#include "kdtreebase.h"
#include "kdsubtree.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::KDTreeWithSubTrees when built using fpu
* rwmath.
*
* This class can be used for creating memory imaged fpu versions of rw::collision::KDTreeWithSubTrees
* which can be deserialized using the LLSerializable framework for loading on platforms
* using fpu rwmath.
*
* As the serialization function matches that of rw::collision::KDTreeWithSubTrees it is possible to
* convert between the two using the Serialization framework. As this class also implements the
* ObjectDescriptor/EA::Physics::SizeAndAlignment framework so HLSerializable can also be used.
*
* Changes to data members in rw::collision::KDTreeWithSubTrees or its serialization function should be
* mirrored in this class.
*/
class KDTreeWithSubTrees : public KDTreeBase
{
public:

    /// Short, fixed-size structure used to define memory requirements for a KDTreeWithSubTrees.
    /// Used to allow class to be serialized with backwards-compatibility.
    struct ObjectDescriptor
    {
        /// (Default) constructor
        ObjectDescriptor(uint32_t maxBranchNodes = 0, uint32_t maxSubTrees = 0)
            : mMaxBranchNodes(maxBranchNodes), mMaxSubTrees(maxSubTrees)
        {
            EA_ASSERT(maxSubTrees <= maxBranchNodes);   // Try to trap reversed arguments
        }

        /// Serialization method.
        template <class Archive>
        void Serialize(Archive& ar, const uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(mMaxBranchNodes);
            ar & EA_SERIALIZATION_NAMED_VALUE(mMaxSubTrees);
        }

        uint32_t mMaxBranchNodes;   ///< Maximum number of branch nodes.
        uint32_t mMaxSubTrees;      ///< Maximum number of KDSubTrees.
    };

    inline
        KDTreeWithSubTrees(KDTreeBase::BranchNode * branchNodes, KDSubTree * subTrees = 0, uint32_t numSubTrees = 0)
        : KDTreeBase(branchNodes), m_numSubTrees(numSubTrees), m_subTrees(subTrees)
    {   
        EA_ASSERT((m_numSubTrees == 0) || (m_subTrees != 0));
        // KDTreeBase data and KDSubTree data uninitialized
        m_numBranchNodes = 0;
        for (uint32_t c = 0; c < m_numSubTrees; ++c)
        {
            m_subTrees[c].m_numBranchNodes = 0;   // indicate not initialized
        }
    }

    inline uint32_t GetNumKDSubTrees() const
    {
        return m_numSubTrees;
    }

    inline void SetKDSubTrees(KDSubTree * subTrees, uint32_t numSubTrees)
    {
        m_subTrees = subTrees;
        m_numSubTrees = numSubTrees;
    }

    static inline EA::Physics::SizeAndAlignment
        GetResourceDescriptor(const ObjectDescriptor & objDesc)
    {
        EA::Physics::SizeAndAlignment rd(sizeof(KDTreeWithSubTrees), rwcKDTREE_ALIGNMENT);

        if (objDesc.mMaxBranchNodes)
        {
            rd += EA::Physics::SizeAndAlignment(sizeof(BranchNode)*objDesc.mMaxBranchNodes, rwcKDTREE_ALIGNMENT);
        }
        if (objDesc.mMaxSubTrees)
        {
            rd += EA::Physics::SizeAndAlignment(sizeof(KDSubTree)*objDesc.mMaxSubTrees, rwcKDTREE_ALIGNMENT);
        }

        return rd;
    }

    static inline KDTreeWithSubTrees * 
        Initialize(const EA::Physics::MemoryPtr & resource, const ObjectDescriptor & objDesc)
    {
        rwcASSERTALIGN(resource.GetMemory(), rwcKDTREE_ALIGNMENT);

        // Allocate branch nodes and KDTree array after the KDTreeWithSubTrees structure
        uintptr_t addr = reinterpret_cast<uintptr_t>(resource.GetMemory());
        addr += sizeof(KDTreeWithSubTrees);

        BranchNode * branchNodes = 0;
        KDSubTree * subTrees = 0;
        if (objDesc.mMaxBranchNodes)
        {
            addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcKDTREE_ALIGNMENT);
            branchNodes = reinterpret_cast<BranchNode *>(addr);

            addr += sizeof(BranchNode) * objDesc.mMaxBranchNodes;
        }
        if (objDesc.mMaxSubTrees)
        {
            addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcKDTREE_ALIGNMENT);
            subTrees = reinterpret_cast<KDSubTree *>(addr);
        }
        return new (resource.GetMemory()) KDTreeWithSubTrees(branchNodes, subTrees, objDesc.mMaxSubTrees);
    }

    void Release()
    {
    }

    ObjectDescriptor GetObjectDescriptor() const
    {
        return ObjectDescriptor(m_numBranchNodes, m_numSubTrees);
    }

    /// Serialization method.
    template <class Archive>
    void Serialize(Archive& ar, const uint32_t version)
    {
        // Nodes are stored within this object as an offset
        ar.TrackInternalPointer(m_branchNodes);
        KDTreeBase::SerializeData(ar, version);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_branchNodes, m_numBranchNodes);

        // Array of subtrees explicitly serialized to use internal pointer tracking for 
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numSubTrees);
        ar.TrackInternalPointer(m_subTrees);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_subTrees, m_numSubTrees);
        // Attach the de-serialized KDSubTrees to this branch nodes in the main KDTree.
        if (ar.IsLoading())
        {
            for (uint32_t c = 0; c < m_numSubTrees; ++c)
            {
                m_subTrees[c].AttachToKDTree(this);
            }
        }
    }


private:
    /// The number of subtrees
    uint32_t m_numSubTrees;
    /// Pointer to array of subtrees
    KDSubTree * m_subTrees;

};

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEWITHSUBTREES_H
