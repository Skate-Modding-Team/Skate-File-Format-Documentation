// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CAPSULE_H
#define PUBLIC_RW_COLLISION_CAPSULE_H

/*************************************************************************************************************

 File: rwccapsule.hpp

 Purpose: Declaration of the Capsule primitive class
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

/* Include Deprecated API */
#include "rw/collision/deprecated/linecapsule.h"

namespace rw
{
namespace collision
{

class CapsuleVolume;


extern Volume::VTable globalCapsuleVTable;


/**
\brief The CapsuleVolume represents a simple collision shape for a cylinder with rounded end-caps.

The capsule volume is an efficient shape because it is simply a line segment axis with a uniform radius
added to it.  The origin of the capsule volume is the center of the axis segment.  By default the axis
direction is the Z axis, although you can change this using the volume relative transformation.
The size of the capsule is defined by the radius and the half-height, which is the distance from the center
of the capsule to the center of one of its endcaps.  For example, a capsule with half-height = 3 and
radius = 1 has an overall length along the Z axis of 8.

\importlib rwccore
*/
class CapsuleVolume : public Volume
{
protected:
    /**
    \internal
    Default constructor
    */
    CapsuleVolume()
    : Volume(rw::collision::VOLUMETYPECAPSULE)
    {
        radius = 0;
        capsuleData.hh = 0;
    }

    /**
    \brief Capsule ctor.
    \param r Fatness radius
    \param hh Capsule's half-height
    */
    CapsuleVolume(float r, float hh)
    : Volume(rw::collision::VOLUMETYPECAPSULE, r)
    {
        capsuleData.hh = hh;
    }

public:
    /**
    Gets the resource requirements of the volume
    \param r Fatness radius
    \param hh Capsule's half-height
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(float /*r*/ = 0, float /*hh*/ = 0)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static CapsuleVolume *
    Initialize(const EA::Physics::MemoryPtr& resource);

    static CapsuleVolume *
    Initialize(const EA::Physics::MemoryPtr& resource, float radius, float halfHeight);

    /**
        Gets the capsule's half-height.

    The half-height is the distance from the center of the capsule to the center of one
    of its endcaps.  By default the axis direction is the Z axis, although you can change this using
    the volume relative transformation.
    \return The half-height of the capsule, not including the radius.
    \see CapsuleVolume::SetHalfHeight
    */
    const float &
    GetHalfHeight() const
    {
        return capsuleData.hh;
    }

    /**
        Sets the capsule's half-height.

    The half-height is the distance from the center of the capsule to the center of one
    of its endcaps.  By default the axis direction is the Z axis, although you can change this using
    the volume relative transformation.
    \param halfHeight New half-height.
    \see CapsuleVolume::SetHalfHeight, Volume::GetRelativeTransform
    */
    void
    SetHalfHeight(const float halfHeight)
    {
        capsuleData.hh = halfHeight;
    }

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
                     const float fatness = 0.0f) const;

    struct ObjectDescriptor
    {
        template <class Archive>
        void Serialize(Archive & /*ar*/, uint32_t /*version*/)
        {}
    };

    static CapsuleVolume *
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

    void SetEndCap0Disabled(const bool disabled);
    void SetEndCap1Disabled(const bool disabled);

    bool IsEndCap0Disabled() const;
    bool IsEndCap1Disabled() const;
private:
};

int32_t
rwcCylinderLineSegIntersect(Fraction * dist,
                            rwpmath::Vector3::InParam orig,
                            rwpmath::Vector3::InParam seg,
                            rwpmath::Vector3::InParam center,
                            rwpmath::Vector3::InParam axis,
                            float axisLengthSq,
                            float radius,
                            RwpBool invert,
                            RwpBool ignoreInside);

/**
    Disable the capsule's End Cap 0.

    \see http://globaltechdocs.ea.com/RWPhysics:Disabling_CapsuleVolume_End_Caps
*/
inline void CapsuleVolume::SetEndCap0Disabled(const bool disabled)
{
    if(disabled)
    {
        m_flags |= VOLUMEFLAG_CAPSULEEND_0_DISABLED;
    }
    else
    {
        m_flags &= ~VOLUMEFLAG_CAPSULEEND_0_DISABLED;
    }
}

/**
    Disable the capsule's End Cap 1.

    \see http://globaltechdocs.ea.com/RWPhysics:Disabling_CapsuleVolume_End_Caps
*/
inline void CapsuleVolume::SetEndCap1Disabled(const bool disabled)
{
    if(disabled)
    {
        m_flags |= VOLUMEFLAG_CAPSULEEND_1_DISABLED;
    }
    else
    {
        m_flags &= ~VOLUMEFLAG_CAPSULEEND_1_DISABLED;
    }
}

/**
    Query whether the capsule's End Cap 0 is disabled.

    \see http://globaltechdocs.ea.com/RWPhysics:Disabling_CapsuleVolume_End_Caps
*/
inline bool CapsuleVolume::IsEndCap0Disabled() const
{
    return (m_flags & VOLUMEFLAG_CAPSULEEND_0_DISABLED) == VOLUMEFLAG_CAPSULEEND_0_DISABLED;
}

/**
    Query whether the capsule's End Cap 1 is disabled.

    \see http://globaltechdocs.ea.com/RWPhysics:Disabling_CapsuleVolume_End_Caps
*/
inline bool CapsuleVolume::IsEndCap1Disabled() const
{
    return (m_flags & VOLUMEFLAG_CAPSULEEND_1_DISABLED) == VOLUMEFLAG_CAPSULEEND_1_DISABLED;
}

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_CAPSULE_H
