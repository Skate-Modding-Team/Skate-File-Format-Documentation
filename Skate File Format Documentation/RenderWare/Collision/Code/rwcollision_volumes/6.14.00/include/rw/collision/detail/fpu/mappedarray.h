// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_MAPPEDARRAY_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_MAPPEDARRAY_H

/*************************************************************************************************************

 File: mappedarray.h

 Purpose:
 */

#include "rw/collision/common.h"
#include "aggregate.h"
#include "volume.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::MappedArray when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::MappedArray
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::MappedArray it is possible to
 * convert between the two using the Serialization framework.
 *
 * Changes to data members in rw::collision::MappedArray or its serialization function should be
 * mirrored in this class.
 */
class MappedArray : public Aggregate
{
public:
    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA::Serialization::MakeNamedValue(*static_cast<Aggregate*>(this), "Aggregate");

        ar.TrackInternalPointer(m_volumes);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_volumes, m_numVolumes);
    }

    Volume * m_volumes;
    uint32_t padma[3];
};

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_MAPPEDARRAY_H
