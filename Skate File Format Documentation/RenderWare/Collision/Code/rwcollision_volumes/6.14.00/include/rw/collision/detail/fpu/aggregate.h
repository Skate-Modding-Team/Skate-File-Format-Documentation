// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_AGGREGATE_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_AGGREGATE_H

/*************************************************************************************************************

 File: aggregate.h

 Purpose:
 */

#include "rw/collision/aggregate.h"
#include "aabbox.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::Aggregate when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::Aggregate
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::Aggregate it is possible to
 * convert between the two using the Serialization framework.
 *
 * Changes to data members in rw::collision::Aggregate or its serialization function should be
 * mirrored in this class.
 */
class Aggregate
{
public:
    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numTagBits);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVolumes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_AABB);

        if (version > 1)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_flags);
        }
        else
        {
            EA_ASSERT(ar.IsLoading());
            if (ar.IsLoading())
            {
                m_flags = 0;
            }
        }
    }

    AABBox                                  m_AABB;
    rw::collision::Aggregate::VTable *      m_vTable;
    uint32_t                                m_numTagBits;
    uint32_t                                m_numVolumes;
    uint32_t                                m_flags;

    // Hard-coded padding to ensure 8 byte alignment for Unix64 builds
#if (EA_PLATFORM_PTR_SIZE == 8)
    uint32_t pad[1];
#endif
};

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

// Version 2 - Added aggregate flags
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::Aggregate, 2)

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_AGGREGATE_H
