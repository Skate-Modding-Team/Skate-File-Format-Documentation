// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREEWITHSUBTREES_H
#define PUBLIC_RW_COLLISION_KDTREEWITHSUBTREES_H

#include "rw/collision/common.h"
#include "rw/collision/kdtreebase.h"
#include "rw/collision/kdtree.h"
#include "rw/collision/kdsubtree.h"

namespace rw
{
namespace collision
{
    // Forward declare ClusteredMesh in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class KDTreeWithSubtrees;

} // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition due to a problem with ps2 gcc.
// This version MUST be updated if the Serialize function is modified.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::KDTreeWithSubtrees, 1)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::KDTreeWithSubtrees, "rw::collision::KDTreeWithSubtrees")

/*************************************************************************************************************/

namespace rw
{
namespace collision
{

/// A KD-Tree that can manage, serialize, and provide access to, an array of KDSubTrees.
class KDTreeWithSubTrees : public KDTreeBase
{
public:

    /// Short, fixed-size structure used to define memory requirements for a KDTreeWithSubTrees.
    /// Used to allow class to be serialized with backwards-compatibility.
    struct ObjectDescriptor
    {
        /// (Default) constructor
        ObjectDescriptor(uint32_t maxBranchNodes = 0, uint32_t maxSubTrees = 0);

        /// Serialization method.
        template <class Archive>
        void Serialize(Archive& archive, const uint32_t version);

        uint32_t mMaxBranchNodes;   ///< Maximum number of branch nodes.
        uint32_t mMaxSubTrees;      ///< Maximum number of KDSubTrees.
    };

    /// Return memory requirements for an instance of a KDTreeWithSubtrees defined by objDesc
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    /// Initialize memory layout for a new instance in the given memory defined by the objDesc.
    /// Memory must exceed alignment and size returned from GetResourceDescriptror(objDesc).
    static KDTreeWithSubTrees * Initialize(const EA::Physics::MemoryPtr & resource, const ObjectDescriptor & objDesc);

    /// Release object when finished with
    void Release() { }

    /// Return the object descriptor required to describe the current instance
    ObjectDescriptor GetObjectDescriptor() const;

    /// Return the number of KDSubTrees stored. May be zero if none have been defined.
    uint32_t GetNumKDSubTrees() const;

    /// Return one of the KDSubTrees.
    /// @param index should be less than GetNumKDSubTrees().
    const KDSubTree & GetKDSubTree(uint32_t index) const;

    /// Return pointer to array of subtrees (for creation only)
    KDSubTree * GetKDSubTrees();

    /// Set pointer to array of subtrees to externally managed memory.
    void SetKDSubTrees(KDSubTree * subTrees, uint32_t numSubTrees);

    /// Perform consistency check on data
    RwpBool IsValid() const;

    /// Serialization method.
    template <class Archive>
    void Serialize(Archive& archive, const uint32_t version);

private:

    /// Internal constructor
    inline KDTreeWithSubTrees(BranchNode * branchNodes, KDSubTree * subTrees, uint32_t numSubTrees = 0);

    /// The number of subtrees
    uint32_t m_numSubTrees;
    /// Pointer to array of subtrees
    KDSubTree * m_subTrees;
};

/*************************************************************************************************************/

inline KDTreeWithSubTrees::ObjectDescriptor::ObjectDescriptor(uint32_t maxBranchNodes, uint32_t maxSubTrees)
: mMaxBranchNodes(maxBranchNodes), mMaxSubTrees(maxSubTrees)
{
    // Try to trap reversed arguments - we must have at least one leaf node for each subtree
    EA_ASSERT(maxSubTrees <= maxBranchNodes+1);
}

template <class Archive>
void KDTreeWithSubTrees::ObjectDescriptor::Serialize(Archive& archive, const uint32_t /* version */)
{
    archive & EA_SERIALIZATION_NAMED_VALUE(mMaxBranchNodes);
    archive & EA_SERIALIZATION_NAMED_VALUE(mMaxSubTrees);
}

inline EA::Physics::SizeAndAlignment
KDTreeWithSubTrees::GetResourceDescriptor(const KDTreeWithSubTrees::ObjectDescriptor & objDesc)
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

inline KDTreeWithSubTrees * 
KDTreeWithSubTrees::Initialize(const EA::Physics::MemoryPtr & resource, const KDTreeWithSubTrees::ObjectDescriptor & objDesc)
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

inline KDTreeWithSubTrees::ObjectDescriptor KDTreeWithSubTrees::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_numBranchNodes, m_numSubTrees);
}

inline uint32_t KDTreeWithSubTrees::GetNumKDSubTrees() const
{
    return m_numSubTrees;
}

inline const KDSubTree & KDTreeWithSubTrees::GetKDSubTree(uint32_t index) const
{
    EA_ASSERT(index < m_numSubTrees);
    EA_ASSERT(m_subTrees);
    return m_subTrees[index];
}

inline KDSubTree * KDTreeWithSubTrees::GetKDSubTrees()
{
    return m_subTrees;
}

inline void KDTreeWithSubTrees::SetKDSubTrees(KDSubTree * subTrees, uint32_t numSubTrees)
{
    m_subTrees = subTrees;
    m_numSubTrees = numSubTrees;
}

inline
KDTreeWithSubTrees::KDTreeWithSubTrees(KDTreeBase::BranchNode * branchNodes, KDSubTree * subTrees, uint32_t numSubTrees)
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

// NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
// make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
template <class Archive>
void KDTreeWithSubTrees::Serialize(Archive& ar, const uint32_t version)
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
            EA_ASSERT(m_subTrees[c].IsValid());
        }
    }
}

inline RwpBool KDTreeWithSubTrees::IsValid() const
{
    const KDTree * kdtree = static_cast<const KDTree *>(static_cast<const KDTreeBase *>(this));
    RwpBool ok = kdtree->IsValid();
    if (m_numSubTrees && !m_subTrees)
    {
        return FALSE;
    }
    // Check each subtree
    for (uint32_t c = 0; c < m_numSubTrees; ++c)
    {
        ok = static_cast<RwpBool>(ok && m_subTrees[c].IsValid());
    }
    return ok;
}

} // namespace collision
} // namespace rw



#endif // PUBLIC_RW_COLLISION_KDTREEWITHSUBTREES_H
