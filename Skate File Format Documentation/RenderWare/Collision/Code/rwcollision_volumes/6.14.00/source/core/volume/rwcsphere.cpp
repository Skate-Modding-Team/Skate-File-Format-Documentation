// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcsphere.cpp

 Purpose: Implementation of the Sphere primitive class

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/sphere.h"
#include "eacollision/linesphere.h"

/* Use platform specific maths */
using namespace rwpmath;

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts


// ***********************************************************************************************************
// Global Variables

// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
This is the virtual function table that is shared by all sphere volumes.
\see SphereVolume
*/
Volume::VTable 
globalSphereVTable = {
    rw::collision::VOLUMETYPESPHERE,
    (Volume::GetBBoxFn)(&SphereVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&SphereVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&SphereVolume::CreateGPInstance),
    (Volume::LineSegIntersectFn)(&SphereVolume::LineSegIntersect),
    (Volume::ReleaseFn)(&SphereVolume::Release),
    "SphereVolume",
    0,
    0,
    0,
    0,
    (Volume::ApplyUniformScaleFn)(&SphereVolume::ApplyUniformScale)
};

// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions

 
// ***********************************************************************************************************
// Structs + Unions + Classes


// ***********************************************************************************************************
// Static Functions




/**
Initializes a SphereVolume of radius 0.0f at the given memory location.    
\param resource Memory resource
\return a pointer to the new SphereVolume.
*/
SphereVolume *
SphereVolume::Initialize(const EA::Physics::MemoryPtr& resource)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) SphereVolume(0.0f);
}

/**
Initializes a SphereVolume of a specified radius at the given memory location.    
\param resource Memory resource
\param radius Fatness radius
\return a pointer to the new SphereVolume.
*/
SphereVolume *
SphereVolume::Initialize(const EA::Physics::MemoryPtr& resource, float radius)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) SphereVolume(radius);
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
SphereVolume::GetBBox(const rwpmath::Matrix44Affine *tm, RwpBool /*tight*/,
                      AABBox &bBox) const
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
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));
#elif defined(EA_PLATFORM_XENON)
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));
#else // this case should be equivalent with updated math library
    VecFloat radiusVec(GetRadius());
#endif

    bBox.m_min = otm.GetW() - radiusVec;
    bBox.m_max = otm.GetW() + radiusVec;

    return TRUE;
}

rwpmath::Vector3
SphereVolume::GetBBoxDiag() const
{

    // TODO: PAB: This is temporary until the standalone rwpmath:: package is used
    // Unaligned vector load removes a LHS
#if defined(EA_PLATFORM_PS3_PPU)
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));
#elif defined(EA_PLATFORM_XENON)
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));
#else // this case should be equivalent with updated math library
    VecFloat radiusVec(GetRadius());
#endif

    VecFloat f = GetVecFloat_Two() * radiusVec;

    return Vector3(f,f,f);
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

/**
This function does nothing.
*/
void
GPSphere::GetBBox( AABBox &/*bbox*/ ) const
{
    EA_ASSERT(Type() == GPInstance::SPHERE);
}



// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\internal
\brief Create the generalized primitive instance data.

This precomputes some data that is used for the generalized primitive intersection test.
The radius, and several virtual function pointers are loaded into the instance structure.
\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
RwpBool
SphereVolume::CreateGPInstance(GPInstance &instance, const Matrix44Affine *tm) const
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

    // initialize GP instance
    GPSphere *sphere = static_cast<GPSphere*>(&instance);
    sphere->Initialize(otm.GetW(), GetRadius(), GetFlags(), reinterpret_cast<uintptr_t>(this), 0);

    return TRUE;
}


/**


\brief
Test whether a sphere volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.

\param pt1    The origin of the line segment.
\param pt2    The end of the line segment.
\param mtx    Transformation Matrix of the parent reference frame.
\param result This will be set to the result of the test.
\param fatness How fat the line is (default=0.0)

\return TRUE if the segment intersects the volume.
*/
RwpBool
SphereVolume::LineSegIntersect(Vector3::InParam pt1, 
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
    Vector3 sphereCenter;
    VecFloat sphereRadius = GetRadius() + fatness;

    if (mtx)
    {
        sphereCenter = TransformPoint(transform.GetW(), *mtx);
    }
    else
    {
        sphereCenter = transform.GetW();
    }

    VecFloat intersectDistance;
    Vector3 normal;
    unsigned int startsInside;

    unsigned int intersectResult = EA::Collision::IntersectLineSphere(intersectDistance, normal, startsInside, 
        lineStart, lineUnitDirection, lineLength,
        sphereCenter, sphereRadius);
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
\brief Applies a uniform scale factor to the dimensions of the sphere volume

This function is used by the Volume class ApplyUniformScale function. If
useProcessedFlags is enabled then the volume processed flag will be respected
and scaling performed if flag is not set. The volume processed flag will be set
after the scaling operation.

\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
SphereVolume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    // Apply scale if processing is required OR if we are to ignore flags
    if (!useProcessedFlags || !(m_flags & VOLUMEFLAG_ISPROCESSED))
    {
        transform.Pos() *= scale;
        radius *= scale;
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
