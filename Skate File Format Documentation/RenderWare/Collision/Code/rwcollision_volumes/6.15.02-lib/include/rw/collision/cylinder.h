// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CYLINDER_H
#define PUBLIC_RW_COLLISION_CYLINDER_H
/*************************************************************************************************************

 File: rwccylinder.hpp

 Purpose: Declaration of the Cylinder primitive class

 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

/* Include Deprecated API */
#include "rw/collision/deprecated/linecylinder.h"

namespace rw
{
namespace collision
{

class CylinderVolume;


/**
\internal

\brief Constants used internally for indices into the GPInstance.box_sizes array.

\enum HALFHEIGHT_INDEX index of hh in box_sizes
\enum RADIUS_INDEX index of inner radius in box_sizes
*/
enum CylinderBoxSizeIndex
{
    HALFHEIGHT_INDEX = 0,
    RADIUS_INDEX = 1,

    CYLINDERBOXSIZEINDEX_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


extern Volume::VTable globalCylinderVTable;



// ***********************************************************************************************************
//                                                CylinderVolume CLASS
// ***********************************************************************************************************

/**
\brief The CylinderVolume represents a simple collision shape for a cylinder with rounded end-caps.

The cylinder volume is typically used for barrels or car wheels.  You should only use the cylinder if it is
necessary to have flat end faces, otherwise a capsule is much more efficient.
The origin of the cylinder volume is the center of the axis segment.  By default the axis
direction is the Z axis, although you can change this using the volume relative transformation.
The size of the cylinder is defined by the half-height, the inner radius, and the outer radius.
The total radius is the sum of the inner and outer radii.  The inner radius is the radius of the flat
face at the end of the cylinder.  The outer radius is added to the cylinder to make it have rounded rims.
By default the outer radius is zero.  A cylinder with zero inner radius is the same as
a capsule (but less efficient).  A cylinder with zero outer radius has a sharp corner at the rim.
The actual cylinder length is two times the half height plus two times the outer radius.

\importlib rwccore
*/
class CylinderVolume : public Volume
{
protected:
    /**

    \internal
    \brief Default constructor
    */
    CylinderVolume()
    : Volume(rw::collision::VOLUMETYPECYLINDER)
    {
        cylinderData.innerRadius = 0.0f;
        cylinderData.hh = 0.0f;
    }

    /**
        \brief Cylinder ctor.
    \param innerRadius inner radius
    \param hh Cylinder's half-height
    \param outerRadius outer radius (fatness)
    */
    CylinderVolume(float innerRadius, float hh, float outerRadius = 0.0f)
    : Volume(rw::collision::VOLUMETYPECYLINDER, outerRadius)
    {
        cylinderData.innerRadius = innerRadius;
        cylinderData.hh = hh;
    }

public:
    /**
    \brief Gets the resource requirements of the volume
    \param innerRadius inner radius
    \param hh Cylinder's half-height
    \param outerRadius fatness
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(float /*innerRadius*/ = 0, float /*hh*/ = 0, float /*outerRadius*/ = 0)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static CylinderVolume *
        Initialize(const EA::Physics::MemoryPtr& resource);

    static CylinderVolume *
        Initialize(const EA::Physics::MemoryPtr&, float innerRadius, float halfHeight, float outerRadius = 0.0f);

    /**
        \brief Gets the cylinder's half-height.

    The half-height is the distance from the center of the cylinder to the center of one
    of its endcaps.  By default the axis direction is the Z axis, although you can change this using
    the volume relative transformation.
    \return The half-height of the cylinder, not including the radius.
    \see CylinderVolume::SetHalfHeight
    */
    float
    GetHalfHeight() const
    {
        return cylinderData.hh;
    }

    /**
        \brief Sets the cylinder's half-height.

    The half-height is the distance from the center of the cylinder to the center of one
    of its endcaps.  By default the axis direction is the Z axis, although you can change this using
    the volume relative transformation.
    \param halfHeight New half-height.
    \see CylinderVolume::SetHalfHeight, Volume::GetRelativeTransform
    */
    void
    SetHalfHeight(const float halfHeight)
    {
        cylinderData.hh = halfHeight;
    }

    /**
    \brief Gets the cylinder's inner radius.
    The inner radius is the radius of the flat part of the end cap.

    \return The inner radius of the cylinder, not including the outer radius.
    \see CylinderVolume::SetInnerRadius
    */
    float
    GetInnerRadius() const
    {
        return cylinderData.innerRadius;
    }

    /**
        \brief Sets the cylinder's inner radius.

    The inner radius is the radius of the flat part of the end cap.
    \param innerRadius New inner radius.
    \see CylinderVolume::GetInnerRadius, Volume::GetRelativeTransform
    */
    void
    SetInnerRadius(const float innerRadius)
    {
        cylinderData.innerRadius = innerRadius;
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
                     float fatness = 0.0f) const;
    RwpBool
    ThinLineSegIntersect(const rwpmath::Vector3 & pt1,
                         const rwpmath::Vector3 & pt2,
                         const rwpmath::Matrix44Affine *tm,
                         VolumeLineSegIntersectResult &result) const;


    RwpBool
    FatLineSegIntersect(const rwpmath::Vector3 & pt1,
                        const rwpmath::Vector3 & pt2,
                        const rwpmath::Matrix44Affine *tm,
                        VolumeLineSegIntersectResult &result,
                        float fatness = 0.0f) const;

    struct ObjectDescriptor
    {
        template <class Archive>
        void Serialize(Archive & /*ar*/, uint32_t /*version*/)
        {}
    };

    static CylinderVolume *
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
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_CYLINDER_H
