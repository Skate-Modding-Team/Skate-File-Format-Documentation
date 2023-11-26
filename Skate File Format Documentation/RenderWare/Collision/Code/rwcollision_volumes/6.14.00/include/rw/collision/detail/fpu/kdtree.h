// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREE_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREE_H

#include "rw/collision/common.h"
#include "rw/collision/kdtreebase.h"
#include "aabbox.h"
#include "kdtreebase.h"

namespace rw
{
    namespace collision
    {
        namespace detail
        {
            namespace fpu
            {
                // Forward declare KDTree in the rw::collision namespace so
                // that we can use it in the EA_SERIALIZATION_CLASS_* macros
                class KDTree;
            } // namspace fpu
        } // namespace detail 
    } // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
// Version 2 serializes the nodes after all other data members
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::KDTree, 2)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::detail::fpu::KDTree, "rw::collision::KDTree")

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::KDTree when built using fpu
* rwmath.
*
* This class can be used for creating memory imaged fpu versions of rw::collision::KDTree
* which can be deserialized using the LLSerializable framework for loading on platforms
* using fpu rwmath.
*
* As the serialization function matches that of rw::collision::KDTree it is possible to
* convert between the two using the Serialization framework. As this class also implements the
* ObjectDescriptor/EA::Physics::SizeAndAlignment framework so HLSerializable can also be used.
*
* Changes to data members in rw::collision::KDTree or its serialization function should be
* mirrored in this class.
*/
class KDTree : public KDTreeBase
{
public:
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t numBranchNodes,
                          uint32_t /*numEntries*/,
                          const AABBox& /*bbox*/)
    {
        uint32_t size = EA::Physics::SizeAlign<uint32_t>(sizeof(KDTree), rwcKDTREE_ALIGNMENT) + numBranchNodes * sizeof(BranchNode);
        return EA::Physics::SizeAndAlignment(size, rwcKDTREE_ALIGNMENT);
    }

    static KDTree *
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t numBranchNodes,
               uint32_t numEntries,
               const AABBox &bbox)
    {
        KDTree *kdtree = static_cast<KDTree *>(resource.GetMemory());
        BranchNode * branchNodes = 0;
        if (numBranchNodes > 0)
        {
            branchNodes = (BranchNode *)EA::Physics::MemAlign(reinterpret_cast<void *>(kdtree + 1), rwcKDTREE_ALIGNMENT);
        }
        return new (resource.GetMemory()) KDTree(numBranchNodes, numEntries, bbox, branchNodes);
    }

    KDTree(uint32_t numBranchNodes,
        uint32_t numEntries,
        const AABBox &bbox,
        BranchNode * branchNodes) : KDTreeBase(numBranchNodes, numEntries, bbox, branchNodes)
    {
    }

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
            ar & EA_SERIALIZATION_NAMED_VALUE(m_numBranchNodes);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_numEntries);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_branchNodes, m_numBranchNodes);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
        }
    }
};
} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREE_H
