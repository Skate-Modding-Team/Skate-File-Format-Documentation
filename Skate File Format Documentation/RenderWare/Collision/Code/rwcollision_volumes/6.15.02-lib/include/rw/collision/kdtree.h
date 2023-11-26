// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREE_H
#define PUBLIC_RW_COLLISION_KDTREE_H

/*************************************************************************************************************

 File: rwckdtree.hpp

 Purpose: KDTree spatial map.
 */

#include "rw/collision/common.h"
#include "rw/collision/kdtreebase.h"

namespace rw
{
    namespace collision
    {
        // Forward declare KDTree in the rw::collision namespace so
        // that we can use it in the EA_SERIALIZATION_CLASS_* macros
        class KDTree;

    } // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
// Version 2 serializes the nodes after all other data members
EA_SERIALIZATION_CLASS_VERSION(rw::collision::KDTree, 2)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::KDTree, "rw::collision::KDTree")

namespace rw
{
namespace collision
{

/**
A KDTreeBase layed out in a single block of memory.

\importlib rwccore
*/
class KDTree : public KDTreeBase
{
private:

    // Private constructor - use Initialize() instead
    KDTree(uint32_t numBranchNodes,
        uint32_t numEntries,
        const rw::collision::AABBox &bbox,
        BranchNode * branchNodes) : KDTreeBase(numBranchNodes, numEntries, bbox, branchNodes)
    {
    }

public:

    /**
    Get the resource requirements of a KDTree

    \param numBranchNodes        Number of branch nodes required.
    \param numEntries            Total number of entries reference by leaf nodes of the tree.
    \param bbox                    Outer extent of the kd tree contents.

    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
        GetResourceDescriptor(uint32_t numBranchNodes,
        uint32_t /*numEntries*/,
        const rw::collision::AABBox &/*bbox*/)
    {
        uint32_t size = EA::Physics::SizeAlign<uint32_t>(sizeof(KDTree), rwcKDTREE_ALIGNMENT) + numBranchNodes * sizeof(BranchNode);
        return EA::Physics::SizeAndAlignment(size, rwcKDTREE_ALIGNMENT);
    }


    /**
    \brief
    Initializes a KDTree at the given memory location.

    The node data is not initialized by this method.  You must call GraphKDTree::InitializeRuntimeKDTree
    to complete the initialization of the KDTree.

    \param resource                Memory resource for KDTree
    \param numBranchNodes        Number of branch nodes required.
    \param numEntries            Total number of entries reference by leaf nodes of the tree.
    \param bbox                    Outer extent of the kd tree contents.

    \see GraphKDTree::Build, GraphKDTree::InitializeRuntimeKDTree, GraphKDTree::GetSortedEntryIndices
    */
    static KDTree *
        Initialize(const EA::Physics::MemoryPtr& resource,
        uint32_t numBranchNodes,
        uint32_t numEntries,
        const rw::collision::AABBox &bbox)
    {
                rwcASSERTALIGN(resource.GetMemory(), rwcKDTREE_ALIGNMENT);
        KDTree *kdtree = static_cast<KDTree *>(resource.GetMemory());
        BranchNode * branchNodes = 0;
        if (numBranchNodes > 0)
        {
            branchNodes = (BranchNode *)EA::Physics::MemAlign(reinterpret_cast<void *>(kdtree + 1), rwcKDTREE_ALIGNMENT);
        }
        return new (resource.GetMemory()) KDTree(numBranchNodes, numEntries, bbox, branchNodes);
    }

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
                ar.TrackInternalPointer(m_branchNodes);
        if (version > 1)
        {
            KDTreeBase::SerializeData(ar, version);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_branchNodes, m_numBranchNodes);
        }
        else
        { 
            // Old order intersperses the branch nodes with the KDTreeBase data
            ar & EA_SERIALIZATION_NAMED_VALUE(m_numBranchNodes);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_numEntries);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_branchNodes, m_numBranchNodes);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
        }


    }

};

    } // collision
} // rw

//These are included here for backwards compatibility.  Previously, all the queries were in the same file.
#include "rw/collision/kdtreelinequerybase.h"
#include "rw/collision/kdtreelinequery.h"
#include "rw/collision/kdtreebboxquerybase.h"
#include "rw/collision/kdtreebboxquery.h"

#endif // PUBLIC_RW_COLLISION_KDTREE_H
