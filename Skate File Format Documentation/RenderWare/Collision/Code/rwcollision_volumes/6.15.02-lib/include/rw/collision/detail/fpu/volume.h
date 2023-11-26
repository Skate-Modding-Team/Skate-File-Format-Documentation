// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_VOLUME_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_VOLUME_H

/*************************************************************************************************************

 File: volume.h

 Purpose:
 */


#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{


/** \brief This class mimics the layout of rw::collision::Volume when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::Volume
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::Volume it is possible to
 * convert between the two using the Serialization framework.
 *
 * Changes to data members in rw::collision::Volume or its serialization function should be
 * mirrored in this class.
 */
class Volume
{
public:
    typedef rw::collision::Volume::ObjectDescriptor ObjectDescriptor;

    const ObjectDescriptor GetObjectDescriptor()
    {
        return Volume::ObjectDescriptor();
    }

    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor& /*objDesc*/)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static Volume * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor& /*objDesc*/)
    {
        rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
        return(new (resource.GetMemory()) Volume());
    }

    void Release() {};

    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(groupID);
        ar & EA_SERIALIZATION_NAMED_VALUE(surfaceID);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_flags);
        ar & EA_SERIALIZATION_NAMED_VALUE(radius);
        ar & EA_SERIALIZATION_NAMED_VALUE(transform);

        ar & ::EA::Serialization::MakeNamedValue(volumeType, "vTable");

        switch (volumeType)
        {
        case rw::collision::VOLUMETYPECAPSULE:
            ar & EA_SERIALIZATION_NAMED_VALUE(capsuleData.hh);
            break;
        case rw::collision::VOLUMETYPECYLINDER:
            ar & EA_SERIALIZATION_NAMED_VALUE(cylinderData.hh);
            ar & EA_SERIALIZATION_NAMED_VALUE(cylinderData.innerRadius);

            break;
        case rw::collision::VOLUMETYPETRIANGLE:
            ar & EA_SERIALIZATION_NAMED_VALUE(triangleData.edgeCos0);
            ar & EA_SERIALIZATION_NAMED_VALUE(triangleData.edgeCos1);
            ar & EA_SERIALIZATION_NAMED_VALUE(triangleData.edgeCos2);
            break;
        case rw::collision::VOLUMETYPEBOX:
            ar & EA_SERIALIZATION_NAMED_VALUE(boxData.hx);
            ar & EA_SERIALIZATION_NAMED_VALUE(boxData.hy);
            ar & EA_SERIALIZATION_NAMED_VALUE(boxData.hz);
            break;
        case rw::collision::VOLUMETYPEAGGREGATE:
            ar.TrackPointer(aggregateData.agg);
            break;
        case VOLUMETYPECONVEXHULL:
        case VOLUMETYPECUSTOM:
            ar.TrackPointer(customData.data);
            ar & EA_SERIALIZATION_NAMED_VALUE(customData.type);
            break;
        default:
            break;
        }
    }

    rw::math::fpu::Matrix44Affine               transform;

    uint32_t volumeType;		///< type of volume. This type is used to fetch the table of function pointers to common methods that all volume types must support.
#if (EA_PLATFORM_PTR_SIZE == 8)
    uint32_t padding[1];
#endif

    union
    {
        rw::collision::AggregateSpecificData    aggregateData;
        rw::collision::SphereSpecificData       sphereData;
        rw::collision::CapsuleSpecificData      capsuleData;
        rw::collision::TriangleSpecificData     triangleData;
        rw::collision::BoxSpecificData          boxData;
        rw::collision::CylinderSpecificData     cylinderData;
        rw::collision::CustomSpecificData       customData;
        uintptr_t                               sizeOfTargetPointer[2];
    };

    float                                     radius;
    uint32_t                                    groupID;
    uint32_t                                    surfaceID;
    uint32_t                                    m_flags;
};




} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_VOLUME_H
