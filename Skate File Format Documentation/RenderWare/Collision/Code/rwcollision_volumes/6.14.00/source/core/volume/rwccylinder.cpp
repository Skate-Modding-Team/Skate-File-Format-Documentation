// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwccylinder.cpp

 Purpose: Implementation of the Cylinder primitive class

 */

#include "rw/collision/cylinder.h"
#include "eacollision/linecylinder.h"

/* Use platform specific maths */
using namespace rwpmath;

namespace rw
{
namespace collision
{
// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
This is the virtual function table that is shared by all cylinder volumes.
\see CylinderVolume
*/
Volume::VTable
globalCylinderVTable = {
    rw::collision::VOLUMETYPECYLINDER,
    (Volume::GetBBoxFn)(&CylinderVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&CylinderVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&CylinderVolume::CreateGPInstance),
    (Volume::LineSegIntersectFn)(&CylinderVolume::LineSegIntersect),
    (Volume::ReleaseFn)(&CylinderVolume::Release),
    "CylinderVolume",
    0,
    0,
    0,
    0,
    (Volume::ApplyUniformScaleFn)(&CylinderVolume::ApplyUniformScale)
};

/**
Initializes a CylinderVolume of zero diameter, half-height and fatness at the given memory location.
\param resource     Memory resource
\return             Pointer to the new CylinderVolume.
*/
CylinderVolume *
CylinderVolume::Initialize(const EA::Physics::MemoryPtr& resource)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) CylinderVolume(0.0f, 0.0f, 0.0f);
}


/**
Initializes a CylinderVolume at the given memory location.
\param resource     Memory resource
\param innerRadius  Inner radius
\param halfHeight   Cylinder's half-height
\param outerRadius  Outer radius (fatness)
\return             Pointer to the new CylinderVolume.
*/

CylinderVolume *
CylinderVolume::Initialize(const EA::Physics::MemoryPtr& resource, float innerRadius, float halfHeight, float outerRadius)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) CylinderVolume(innerRadius, halfHeight, outerRadius);
}


inline Vector3 SqrtFastVec3(Vector3::InParam v)
{
#if RW_MATH_VERSION >= RW_MATH_CREATE_VERSION_NUMBER( 1, 9, 0 )
    return rwpmath::SqrtFast(v);
#else // if RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER( 1, 9, 0 )
    return Vector3(SqrtFast(v.X()), SqrtFast(v.Y()), SqrtFast(v.Z()));
#endif // RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER( 1, 9, 0 )
}


/**
\brief Gets an axis aligned bounding box for the volume

The bounding box of the volume is transformed by the volume relative transform and by the input parent
transform if it is not NULL.  The transformations may translate and/or rotate the bbox which may increase
the size of the bbox.

\param tm parent transformation to applied to the result bbox.  May be NULL.
\param tight if true, a slower more precise algorithm may be used to compute the bbox.
\param bBox reference to a bbox to store the result.

\return TRUE on success
*/
RwpBool
CylinderVolume::GetBBox(const rwpmath::Matrix44Affine *tm,
                        RwpBool /* tight */, AABBox &bBox) const
{

    VecFloat r(GetInnerRadius());
    VecFloat hh(GetHalfHeight());
    VecFloat fatness(GetRadius());

    Matrix44Affine otm = transform;

    if (tm)
    {
        otm = otm * (*tm);
    }

    Vector3 axis = otm.GetZ();

    Vector3 halfbox = SqrtFastVec3(Abs(GetVector3_One() - axis * axis)) * r + Abs(axis) * hh + fatness;

    bBox.Set(otm.GetW() - halfbox, otm.GetW() + halfbox);

    return TRUE;
}


rwpmath::Vector3
CylinderVolume::GetBBoxDiag() const
{

    Vector3   f;
    float r = GetInnerRadius();
    float hh = GetHalfHeight();

    f = Vector3(r, r, hh);

    return GetVecFloat_Two() * f;
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

/**
This function does nothing.
*/
void
GPCylinder::GetBBox(AABBox &/*bbox*/) const
{
    EA_ASSERT(Type() == GPInstance::CYLINDER);
}

// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\internal
\brief Create the generalized primitive instance data.

This precomputes some data that is used for the generalized primitive intersection test.
The cylinder axis is stored as an edge direction in world coordinates,
the radius, and several virtual function pointers are loaded into the instance structure.

The face_normals store the x,y and z directions from the transform that it's given.
\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
RwpBool
CylinderVolume::CreateGPInstance(GPInstance &instance, const Matrix44Affine *tm) const
{

    Matrix44Affine otm;

    if (tm)
    {
        otm = Mult(transform, *tm);
    }
    else
    {
        otm = transform;
    }

    // initialize GP instance
    GPCylinder *cylinder = static_cast<GPCylinder*>(&instance);
    cylinder->Initialize(otm.GetW(), GetInnerRadius(), otm.GetZ(), GetHalfHeight(), GetRadius(), GetFlags(), reinterpret_cast<uintptr_t>(this), 0, otm.GetY(), otm.GetX());

    return TRUE;
}

/**
\brief
Test whether a cylinder volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.
For example, if the line is 10 units long, and intersects the volume 8 units from the start, then the
\e  lineParam is 0.8.

\param inPt1    The origin of the line segment.
\param inPt2    The end of the line segment.
\param tm       Transformation Matrix of the parent reference frame.
\param result   This will be set to the result of the test.
\param fatness  The padding added around the cylinder

\return TRUE if the segment intersects the volume.
*/
RwpBool
CylinderVolume::LineSegIntersect(Vector3::InParam pt1, 
                                Vector3::InParam pt2,
                                const Matrix44Affine *mtx,
                                VolumeLineSegIntersectResult &result,
                                const float fatness /*=0.0f*/) const
{

    const Vector3 &lineStart = pt1;
    const Vector3 lineDelta = pt2 - pt1;
    Vector3 lineUnitDirection;
    VecFloat lineLengthReciprocal;
    const VecFloat lineLength = EA::Collision::xmath::NormalizeReciprocalReturnMagnitudeFast(lineDelta, lineUnitDirection, lineLengthReciprocal);
    Vector3 cylinderCenter;
    VecFloat cylinderOuterRadius = GetRadius() + fatness;
    VecFloat cylinderInnerRadius = GetInnerRadius();
    Vector3 cylinderUnitAxis;
    VecFloat cylinderHalfLength = GetHalfHeight();

    if (mtx)
    {
        cylinderCenter = TransformPoint(transform.GetW(), *mtx);
        cylinderUnitAxis = TransformVector(transform.GetZ(), *mtx);
    }
    else
    {
        cylinderCenter = transform.GetW();
        cylinderUnitAxis = transform.GetZ();
    }

    VecFloat intersectDistance;
    unsigned int startsInside;
    Vector3 innerIntersectionPoint;
    Vector3 normal;

    unsigned int intersectResult = EA::Collision::IntersectLineCylinder(intersectDistance, innerIntersectionPoint, normal, startsInside, 
        lineStart, lineUnitDirection, lineLength,
        cylinderCenter, NormalizeFast(cylinderUnitAxis), cylinderHalfLength, cylinderInnerRadius, cylinderOuterRadius);

    result.v = this;
    
    result.volParam = GetVector3_Zero(); //NOT SET
    if(intersectResult | startsInside)
    {
        result.normal = normal;
        result.lineParam = intersectDistance * lineLengthReciprocal;
        result.position = lineStart + intersectDistance * lineUnitDirection - fatness * result.normal;
        return TRUE;

    }
    else
    {
        result.lineParam = 0.0f;
        result.position = GetVector3_Zero();
        result.normal = GetVector3_Zero();

        return FALSE;
    }
}


/**
\brief Applies a uniform scale factor to the dimensions of the cylinder volume

This function is used by the Volume class ApplyUniformScale function. If
useProcessedFlags is enabled then the volume processed flag will be respected
and scaling performed if flag is not set. The volume processed flag will be set
after the scaling operation.

\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
CylinderVolume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    // Apply scale if processing is required OR if we are to ignore flags
    if (!useProcessedFlags || !(m_flags & VOLUMEFLAG_ISPROCESSED))
    {
        transform.Pos() *= scale;
        radius *= scale;
        cylinderData.hh *= scale;
        cylinderData.innerRadius *= scale;
    }

    // Set processed flag to identify that scaling has been performed on this volume
    if (useProcessedFlags)
    {
        SetProcessedFlag();
    }
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

} // namespace collision
} // namespace rw
