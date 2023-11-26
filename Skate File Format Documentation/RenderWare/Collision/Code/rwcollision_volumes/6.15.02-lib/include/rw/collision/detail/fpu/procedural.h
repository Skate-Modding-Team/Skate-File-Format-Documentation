// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_PROCEDURAL_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_PROCEDURAL_H

/*************************************************************************************************************

 File: procedural.h

 Purpose:
*/

#include "aggregate.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::Procedural when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::Procedural
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::Procedural it is possible to
 * convert between the two using the Serialization framework.
 *
 * Changes to data members in rw::collision::Procedural or its serialization function should be
 * mirrored in this class.
 */
class Procedural : public Aggregate
{
public:
    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA::Serialization::MakeNamedValue(*static_cast<Aggregate*>(this), "Aggregate");
    }

};



} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif //PUBLIC_RW_COLLISION_DETAIL_FPU_PROCEDURAL_H
