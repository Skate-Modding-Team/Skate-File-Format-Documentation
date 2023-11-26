// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_BOX_H
#define PUBLIC_RW_COLLISION_BOX_H

/*************************************************************************************************************

 File: rwcbox.hpp

 Purpose: Declare the BoxVolume class.
*/

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

/* Include Deprecated API */
#include "rw/collision/deprecated/linebox.h"

namespace rw
{
namespace collision
{

class BoxVolume;


extern Volume::VTable globalBoxVTable;


/**
\brief The BoxVolume represents a simple collision shape for a rectangular six-sided prism.

The box volume is a rectangular box with three size dimensions, length, width, and height.  As the other
primitive shapes, the box may also have a radius.  The effect of the radius on the box is to make it have
rounded edges and corners.

The origin of the box volume is the center of the box.  The faces of the box (ignoring the volume
relative transform) are perpendicular to the three coordinate axis.  The dimensions of the box are defined
by the distance from the center of the box to the face of the box along each axis (ignoring the radius).
If the radius is not zero, it is added to the dimensions of the box to make the full rounded shape.

For example, a box with dimensions (2, 3, 4) and radius = 1, will produce a box that has rounded corners and
edges, and the total length of the box in the X dimension is 1+2+2+1 = 6, and in the Z dimension,
1+4+4+1 = 10.

\importlib rwccore
*/
class BoxVolume : public Volume
{
protected:
    /**
    Box volume constructor.
    \param dimensions A vector containing the box X,Y,Z half lengths.
    */
    BoxVolume(rwpmath::Vector3::InParam dimensions, float r = 0.0f)
    : Volume(rw::collision::VOLUMETYPEBOX, r)
    {
        EA_ASSERT(static_cast<float>(dimensions.X()) >= 0);
        EA_ASSERT(static_cast<float>(dimensions.Y()) >= 0);
        EA_ASSERT(static_cast<float>(dimensions.Z()) >= 0);
        boxData.hx = dimensions.X();
        boxData.hy = dimensions.Y();
        boxData.hz = dimensions.Z();
    }

public:



    /**
    Gets the resource requirements of the volume
    \param dimensions A vector containing the box X,Y,Z half lengths.
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(rwpmath::Vector3::InParam /*dimensions*/, float /*radius*/ = 0.0f)
    {
        return GetResourceDescriptor();
    }

    /**

    Gets the resource requirements of the volume
    \param halfX The box X half length.
    \param halfY The box Y half length.
    \param halfZ The box Z half length.
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(float /*halfX*/ = 0.0f, float /*halfY*/ = 0.0f,
                          float /*halfZ*/ = 0.0f, float /*radius*/ = 0.0f)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static BoxVolume *
    Initialize(const EA::Physics::MemoryPtr& resource);

    static BoxVolume *
    Initialize(const EA::Physics::MemoryPtr& resource, float halfX, float halfY, float halfZ, float radius = 0.0f);

    static BoxVolume *
    Initialize(const EA::Physics::MemoryPtr& resource, rwpmath::Vector3::InParam halfDimensions, float radius = 0.0f);

    /**
    Returns the dimensions of a box volume in a rw::math::Vector3

    The dimensions of the box are the distances from the center of the box to the the face along each
    axis, ignoring radius.
    \return rw::math::vector3 containing the box X,Y,Z half lengths.
    \see BoxVolume::SetDimensions
    */
    rwpmath::Vector3
    GetDimensions() const
    {
        rwpmath::Vector3 dimensions(rwpmath::VecFloat(boxData.hx), rwpmath::VecFloat(boxData.hy), rwpmath::VecFloat(boxData.hz));
        return dimensions;
    }




    /**
    Gets the dimensions of a box volume

    The dimensions of the box are the distances from the center of the box to the the face along each
    axis, ignoring radius.
    \param dimensions Reference to a vector to be filled with the box X,Y,Z half lengths.
    \see BoxVolume::SetDimensions
    */
    void
    GetDimensions(rwpmath::Vector3 &dimensions) const
    {
        dimensions.Set(boxData.hx,boxData.hy,boxData.hz);
    }

    /**
    Sets the dimensions of a box volume.

    The dimensions of the box are the distances from the center of the box to the the face along each
    axis, ignoring radius.
    \param dimensions A vector which contains the box X,Y,X half lengths.
    \see BoxVolume::GetDimensions
    */
    void
    SetDimensions(rwpmath::Vector3::InParam dimensions)
    {
        EA_ASSERT(static_cast<float>(dimensions.X()) >= 0);
        EA_ASSERT(static_cast<float>(dimensions.Y()) >= 0);
        EA_ASSERT(static_cast<float>(dimensions.Z()) >= 0);
        boxData.hx = dimensions.X();
        boxData.hy = dimensions.Y();
        boxData.hz = dimensions.Z();
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
                     const float = 0.0f) const;

    struct ObjectDescriptor
    {
        template <class Archive>
        void Serialize(Archive & /*ar*/, uint32_t /*version*/)
        {}
    };

    static BoxVolume *
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
    const ObjectDescriptor GetObjectDescriptor()
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
rwcPlaneLineSegIntersect(Fraction *dist, float orig_i, float seg_i, float sign, float disp);


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_BOX_H
