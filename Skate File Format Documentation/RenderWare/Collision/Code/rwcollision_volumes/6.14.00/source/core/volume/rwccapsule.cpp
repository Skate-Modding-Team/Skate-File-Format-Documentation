// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwccapsule.cpp

 Purpose: Implementation of the Capsule primitive class

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/capsule.h"
#include "eacollision/linecapsule.h"

/* Use platform specific maths */
using namespace rwpmath;

namespace rw
{
namespace collision
{


// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\brief This is the virtual function table that is shared by all capsule volumes.
\see CapsuleVolume
*/
Volume::VTable
globalCapsuleVTable = {
    rw::collision::VOLUMETYPECAPSULE,
    (Volume::GetBBoxFn)(&CapsuleVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&CapsuleVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&CapsuleVolume::CreateGPInstance),
    (Volume::LineSegIntersectFn)(&CapsuleVolume::LineSegIntersect),
    (Volume::ReleaseFn)(&CapsuleVolume::Release),
    "CapsuleVolume",
    0,
    0,
    0,
    0,
    (Volume::ApplyUniformScaleFn)(&CapsuleVolume::ApplyUniformScale)
};

/**
\brief Initializes a CapsuleVolume of radius 0.0f and halfHeight 0.0f at the given memory location.

\param resource Memory resource
\return a pointer to the new CapsuleVolume.
*/
CapsuleVolume *
CapsuleVolume::Initialize(const EA::Physics::MemoryPtr& resource)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) CapsuleVolume(0.0f, 0.0f);
}

/**
\brief Initializes a CapsuleVolume of specified radius and halfHeight at the given memory location.

\param resource Memory resource
\param radius Fatness radius
\param halfHeight Capsule's half-height
\return a pointer to the new CapsuleVolume.
*/
CapsuleVolume *
CapsuleVolume::Initialize(const EA::Physics::MemoryPtr& resource, float radius, float halfHeight)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) CapsuleVolume(radius, halfHeight);
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
CapsuleVolume::GetBBox(const rwpmath::Matrix44Affine *tm,
                       RwpBool /*tight*/, AABBox &bBox) const
{

    Matrix44Affine otm;

    if (tm)
    {
        otm = transform * (*tm);
    }
    else
    {
        otm = transform;
    }

// TODO: PAB: This is temporary until the standalone rwpmath:: package is used
// Unaligned vector load removes a LHS
#if defined(EA_PLATFORM_PS3_PPU)
    VecFloat hh       ((vector float)vec_splat((vector float)vec_lvlx(0, &GetHalfHeight()), 0));
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));
#elif defined(EA_PLATFORM_XENON)
    VecFloat hh       (__vspltw(__lvlx(&GetHalfHeight(), 0),0));
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));
#else // this case should be equivalent with updated math library
    VecFloat hh       (GetHalfHeight());
    VecFloat radiusVec(GetRadius());
#endif

    // original capsule ori
    // Fatness of bbox
    Vector3 f = Abs(otm.GetZ()) * hh + radiusVec;

    // Offset by position
    bBox.Set(otm.GetW() - f, otm.GetW() + f);
    return TRUE;
}

rwpmath::Vector3
CapsuleVolume::GetBBoxDiag() const
{

    // TODO: PAB: This is temporary until the standalone rwpmath:: package is used
    // Unaligned vector load removes a LHS
#if defined(EA_PLATFORM_PS3_PPU)
    VecFloat hh       ((vector float)vec_splat((vector float)vec_lvlx(0, &GetHalfHeight()), 0));
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));
#elif defined(EA_PLATFORM_XENON)
    VecFloat hh       (__vspltw(__lvlx(&GetHalfHeight(), 0),0));
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));
#else // this case should be equivalent with updated math library
    VecFloat hh       (GetHalfHeight());
    VecFloat radiusVec(GetRadius());
#endif

    // original capsule ori
    // Fatness of bbox
    Vector3 f = Abs(transform.GetZ()) * hh + radiusVec;
    return GetVecFloat_Two() * f;
}

#endif // !defined(EA_PLATFORM_PS3_SPU)


void
GPCapsule::GetBBox( AABBox &/*bbox*/ ) const
{
    EA_ASSERT(Type() == GPInstance::CAPSULE);
}

// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\internal

\brief Create the generalized primitive instance data.

This precomputes some data that is used for the generalized primitive intersection test.
The capsule axis is stored as an edge direction in world coordinates,
the radius, and several virtual function pointers are loaded into the instance structure.
\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
RwpBool
CapsuleVolume::CreateGPInstance(GPInstance &instance, const Matrix44Affine *tm) const
{

    Matrix44Affine otm;

    if (tm)
    {
        otm = Mult( transform, *tm );
    }
    else
    {
        otm = transform;
    }

    // initialize GP instance
    GPCapsule *capsule = static_cast<GPCapsule*>(&instance);
    capsule->Initialize(otm.GetW(), GetRadius(), otm.GetZ(), GetHalfHeight(), GetFlags(), reinterpret_cast<uintptr_t>(this), 0);

    return TRUE;
}


/**
\brief
Test whether a capsule volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.
For example, if the line is 10 units long, and intersects the volume 8 units from the start, then the
\e  lineParam is 0.8.

\param inPt1    The origin of the line segment.
\param inPt2    The end of the line segment.
\param tm       Transformation Matrix of the parent reference frame.
\param result   This will be set to the result of the test.
\param fatness  The fatness of the line

\return TRUE if the segment intersects the volume.
 */
RwpBool
CapsuleVolume::LineSegIntersect(Vector3::InParam pt1, 
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
    Vector3 capsuleCenter;
    VecFloat capsuleRadius = GetRadius() + fatness;
    Vector3 capsuleUnitAxis;
    VecFloat capsuleHalfLength = GetHalfHeight();

    if (mtx)
    {
        capsuleCenter = TransformPoint(transform.GetW(), *mtx);
        capsuleUnitAxis = TransformVector(transform.GetZ(), *mtx);
    }
    else
    {
        capsuleCenter = transform.GetW();
        capsuleUnitAxis = transform.GetZ();
    }

    VecFloat intersectDistance;
    VecFloat axisDistance;
    Vector3 normal;
    unsigned int startsInside;

    unsigned int intersectResult = EA::Collision::IntersectLineCapsule(intersectDistance, axisDistance, normal, startsInside, 
        lineStart, lineUnitDirection, lineLength,
        capsuleCenter, NormalizeFast(capsuleUnitAxis), capsuleHalfLength, capsuleRadius);

    result.v = this;
   

    if(intersectResult | startsInside)
    { 
        result.normal = normal;
        result.lineParam = intersectDistance * lineLengthReciprocal;
        result.position = lineStart + intersectDistance * lineUnitDirection - fatness * result.normal;
        VecFloat regionIntersection = Select(CompEqual(Abs(axisDistance), capsuleHalfLength), GetVecFloat_One() * SgnNonZero(axisDistance), GetVecFloat_Zero());
        result.volParam = Vector3(regionIntersection, GetVecFloat_Zero(), GetVecFloat_Zero());
        return TRUE;
    }
    else
    {
        result.lineParam = 0.0f;
        result.position = GetVector3_Zero();
        result.normal = GetVector3_Zero();
        result.volParam = GetVector3_Zero();
        return FALSE;
    }
}


/**
\brief Applies a uniform scale factor to the dimensions of the capsule volume

This function is used by the Volume class ApplyUniformScale function. If
useProcessedFlags is enabled then the volume processed flag will be respected
and scaling performed if flag is not set. The volume processed flag will be set
after the scaling operation.

\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
CapsuleVolume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    // Apply scale if processing is required OR if we are to ignore flags
    if (!useProcessedFlags || !(m_flags & VOLUMEFLAG_ISPROCESSED))
    {
        transform.Pos() *= scale;
        radius *= scale;
        capsuleData.hh *= scale;
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
