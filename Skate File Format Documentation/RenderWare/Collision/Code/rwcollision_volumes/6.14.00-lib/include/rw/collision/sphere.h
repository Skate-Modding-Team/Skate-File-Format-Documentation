// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_SPHERE_H
#define PUBLIC_RW_COLLISION_SPHERE_H

/*************************************************************************************************************

 File: rwcsphere.hpp

 Purpose: Declaration of the Sphere primitive class
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

/* Include Deprecated API */
#include "rw/collision/deprecated/linesphere.h"

namespace rw
{
namespace collision
{

class SphereVolume;


extern Volume::VTable globalSphereVTable;


/**
\brief The SphereVolume represents a ball collision shape.

The sphere volume is the most efficient shape because it is simply one point with a uniform radius
added to it.  The origin of the sphere volume is its center point.

\importlib rwccore
*/
class SphereVolume : public Volume
{
protected:
    /**
    \brief Sphere volume constructor.
    \param rad Sphere radius
    */
    SphereVolume(float rad)
    : Volume(rw::collision::VOLUMETYPESPHERE, rad)
    {

    }

public:
    /**
    Gets the resource requirements of the volume
    \param r radius optional
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(float /*r*/ = 0)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static SphereVolume *
    Initialize(const EA::Physics::MemoryPtr& resource);

    static SphereVolume *
    Initialize(const EA::Physics::MemoryPtr& resource, float radius);

    RwpBool
    GetBBox(const rwpmath::Matrix44Affine *tm, RwpBool tight, AABBox &bBox) const;

    rwpmath::Vector3
    GetBBoxDiag() const;

    RwpBool
    CreateGPInstance(GPInstance &instance, const rwpmath::Matrix44Affine *tm) const;

    RwpBool
    LineSegIntersect(rwpmath::Vector3::InParam pt1,
                     rwpmath::Vector3::InParam pt2,
                     const rwpmath::Matrix44Affine *tm,
                     VolumeLineSegIntersectResult &result,
                     const float fatness=0.0f) const;

    struct ObjectDescriptor
    {
        template <class Archive>
            void Serialize(Archive & /*ar*/, uint32_t /*version*/)
        {}
    };

    static SphereVolume *
        Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & /*objDesc*/)
    {
        return (Initialize(resource));
    }

    static EA::Physics::SizeAndAlignment
        GetResourceDescriptor(const ObjectDescriptor & /*objDesc*/)
    {
        return (GetResourceDescriptor());
    }

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const
    {
        return ObjectDescriptor();
    }

    /**
    */
    void Release() {}

    void ApplyUniformScale(float scale, bool useProcessedFlags = false);
private:
};

int32_t
rwcSphereLineSegIntersect(Fraction *dist,
                          const rwpmath::Vector3 & orig,
                          const rwpmath::Vector3 & seg,
                          const rwpmath::Vector3 & center,
                          const float radius);


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_SPHERE_H
