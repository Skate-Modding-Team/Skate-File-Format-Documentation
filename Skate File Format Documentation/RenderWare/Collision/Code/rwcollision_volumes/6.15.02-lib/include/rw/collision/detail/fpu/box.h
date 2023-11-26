// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_BOX_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_BOX_H

/*************************************************************************************************************

 File: box.h

 Purpose:
*/

#include "rw/collision/common.h"
#include "rw/collision/volume.h"
#include "rw/collision/box.h"
#include "volume.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::BoxVolume when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::BoxVolume
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::BoxVolume it is possible to
 * convert between the two using the Serialization framework. As this class also implements the
 * ObjectDescriptor/EA::Physics::SizeAndAlignment framework so HLSerializable can also be used.
 *
 * Changes to data members in rw::collision::BoxVolume or its serialization function should be
 * mirrored in this class.
 */
class BoxVolume : public Volume
{
public:
    typedef rw::collision::BoxVolume::ObjectDescriptor ObjectDescriptor;

    const ObjectDescriptor GetObjectDescriptor()
    {
        return BoxVolume::ObjectDescriptor();
    }

    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor& /*objDesc*/)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static BoxVolume * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor& /*objDesc*/)
    {
        BoxVolume* volume = reinterpret_cast<BoxVolume*>(resource.memory);
        volume->volumeType = rw::collision::VOLUMETYPEBOX;
        return volume;
    }

    void Release()
    {
    }
};

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_BOX_H

