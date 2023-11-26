// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_KDSUBTREE_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_KDSUBTREE_H

/*************************************************************************************************************

 File: rw/collision/detail/fpu/kdsubtree.h

 Purpose:
 */

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


/** \brief This class mimics the layout of rw::collision::KDSubTree when built using fpu
* rwmath.
*
* This class can be used for creating memory imaged fpu versions of rw::collision::KDSubTree
* which can be deserialized using the LLSerializable framework for loading on platforms
* using fpu rwmath.
*
* As the serialization function matches that of rw::collision::KDSubTree it is possible to
* convert between the two using the Serialization framework. As this class also implements the
* ObjectDescriptor/EA::Physics::SizeAndAlignment framework so HLSerializable can also be used.
*
* Changes to data members in rw::collision::KDSubTree or its serialization function should be
* mirrored in this class.
*/
class KDSubTree : public KDTreeBase
{
private:
    uint32_t    m_branchNodeOffset;
    uint32_t    m_defaultEntry;

public:

    KDSubTree() : KDTreeBase() {}

    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
        // Does *not* serialize the branch nodes (we don't own these) or even the pointer
        // to them. Expect caller to use AttachToKDTree() to set this up.
        KDTreeBase::SerializeData(ar, version);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_branchNodeOffset);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_defaultEntry);
    }

    inline void AttachToKDTree(KDTreeBase * kdtree)
    {
        m_branchNodes = kdtree->m_branchNodes + m_branchNodeOffset;
    }
};


} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_KDSUBTREE_H
