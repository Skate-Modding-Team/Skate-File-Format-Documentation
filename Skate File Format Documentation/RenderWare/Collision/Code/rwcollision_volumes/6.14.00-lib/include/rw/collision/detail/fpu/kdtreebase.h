// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEBASE_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEBASE_H

/*************************************************************************************************************

 File: rw/collision/detail/fpu/kdtreebase.h

 Purpose:
 */

#include "aabbox.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

class KDTreeBase
{
public:
    struct NodeRef
    {
        uint32_t    m_content;
        uint32_t    m_index;

        template <class Archive>
            void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_content);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_index);
        }
    };


    struct BranchNode
    {
        uint32_t    m_parent;
        uint32_t    m_axis;
        NodeRef     m_childRefs[2];
        float     m_extents[2];

        template <class Archive>
            void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_parent);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_axis);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_childRefs, 2);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_extents, 2);
        }
    };

    /// Serialize data members, but not structural members
    template <class Archive>
    void SerializeData(Archive& ar, const uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numBranchNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numEntries);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
    }

    /// Memory layout constructor - no other data initialized.
    KDTreeBase(BranchNode * branchNodes = 0)
        : m_branchNodes(branchNodes)
    {        
    }

    /// Constructor
    KDTreeBase(uint32_t numBranchNodes,
        uint32_t numEntries,
        const AABBox &bbox,
        BranchNode * branchNodes)
        : m_branchNodes(branchNodes)
        , m_numBranchNodes(numBranchNodes)
        , m_numEntries(numEntries)
        , m_bbox(bbox)
    {
    }

    BranchNode *      m_branchNodes;
    uint32_t          m_numBranchNodes;
    uint32_t          m_numEntries;
    AABBox            m_bbox;
};

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEBASE_H
