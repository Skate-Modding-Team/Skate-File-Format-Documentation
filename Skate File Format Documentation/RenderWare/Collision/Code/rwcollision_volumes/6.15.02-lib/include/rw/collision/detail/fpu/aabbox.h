// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_AABBOX_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_AABBOX_H

#include "rw/collision/common.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::AABBox when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::AABBox
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::AABBox it is possible to
 * convert between the two using the Serialization framework.
 *
 * Changes to data members in rw::collision::AABBox or its serialization function should be
 * mirrored in this class.
 */
class AABBox
{
public:
    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_min);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_max);
    }

    rw::math::fpu::Vector3 m_min;
    rw::math::fpu::Vector3 m_max;
};

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_AABBOX_H
