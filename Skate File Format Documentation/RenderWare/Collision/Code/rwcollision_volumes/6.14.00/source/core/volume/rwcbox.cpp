// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcbox.cpp

 Purpose: Implementation of the Box primitive class

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/box.h"
#include "eacollision/linebox.h"

#if defined(EA_PLATFORM_PS3_PPU)
#include <altivec.h>
#endif // defined(EA_PLATFORM_PS3_PPU)

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

/**
\internal

\brief This macro maps integers (0, 1, 2) into (1, 2, 0).
*/
#define rwc_NextMod3(i) ((1<<(i))&3)

// ***********************************************************************************************************
// Global Variables

// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
This is the virtual function table that is shared by all box volumes.
\see BoxVolume
*/
Volume::VTable
globalBoxVTable = {
    rw::collision::VOLUMETYPEBOX,
    (Volume::GetBBoxFn)(&BoxVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&BoxVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&BoxVolume::CreateGPInstance),
    (Volume::LineSegIntersectFn)(&BoxVolume::LineSegIntersect),
    (Volume::ReleaseFn)(&BoxVolume::Release),
    "BoxVolume",
    0,
    0,
    0,
    0,
    (Volume::ApplyUniformScaleFn)(&BoxVolume::ApplyUniformScale)
};

/**
Initializes a BoxVolume at the given memory location.

The radius and half dimensions of the box are set to zero.
\param resource Memory resource
\return a pointer to the new BoxVolume.
*/
BoxVolume *
BoxVolume::Initialize(const EA::Physics::MemoryPtr& resource)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) BoxVolume(rwpmath::GetVector3_Zero());
}

/**
Initializes a fat BoxVolume at the given memory location.

\param resource     Memory resource
\param halfX        The box X half length.
\param halfY        The box Y half length.
\param halfZ        The box Z half length.
\param radius       The box fatness

\return             Pointer to the new BoxVolume.
*/
BoxVolume *
BoxVolume::Initialize(const EA::Physics::MemoryPtr& resource, float halfX, float halfY, float halfZ, float radius)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) BoxVolume(rwpmath::Vector3(halfX, halfY, halfZ), radius);
}

/**
Initializes a fat BoxVolume at the given memory location.

\param resource     Memory resource
\param dimensions   Vector containing the box X,Y,Z half lengths.
\param radius       The box fatness

\return             Pointer to the new BoxVolume.
*/
BoxVolume *
BoxVolume::Initialize(const EA::Physics::MemoryPtr& resource, rwpmath::Vector3::InParam halfDimensions, float radius)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) BoxVolume(halfDimensions, radius);
}
#endif // defined(EA_PLATFORM_PS3_SPU)

/**
\internal

\deprecated This is deprecated functionality

This function does nothing.
*/
void
GPBox::GetBBox( AABBox &/*bbox*/ ) const
{
    EA_ASSERT(Type() == GPInstance::BOX);
}


// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\internal

\brief Create the generalized primitive instance data.

This precomputes some data that is used for the generalized primitive intersection test.
The three face directions and three edge directions are computed in world coordinates,
the radius, and several virtual function pointers are loaded into the instance structure.
\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
RwpBool
BoxVolume::CreateGPInstance(GPInstance &instance, const Matrix44Affine *tm) const
{

    Matrix44Affine otm;

    if ( tm )
    {
        otm = transform * (*tm);
    }
    else
    {
        otm = transform;
    }

    // initialize GP instance
    GPBox *box = static_cast<GPBox*>(&instance);
    box->Initialize(otm.GetW(), otm.GetX(), otm.GetY(), otm.GetZ(), GetDimensions(), GetRadius(), GetFlags(), reinterpret_cast<uintptr_t>(this), 0);

    return TRUE;
}






/**
\brief
Test whether a box volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.
For example, if the line is 10 units long, and intersects the volume 8 units from the start, then the
\e  lineParam is 0.8.

\param inPt1      The origin of the line segment.
\param inPt2      The end of the line segment.
\param tm       Transformation Matrix of the parent reference frame.
\param result   This will be set to the result of the test.
\param fatness  The fatness of the line.

\return TRUE if the segment intersects the volume.
 */

RwpBool
BoxVolume::LineSegIntersect(Vector3::InParam pt1, 
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
    Vector3 boxCenter;
    VecFloat boxRadius = GetRadius() + fatness;
    Vector3 boxUnitAxis0;
    Vector3 boxUnitAxis1;
    Vector3 boxUnitAxis2;
    Vector3 boxDimensions = GetDimensions();
    VecFloat boxHalfLength0 = boxDimensions.GetX();
    VecFloat boxHalfLength1 = boxDimensions.GetY();
    VecFloat boxHalfLength2 = boxDimensions.GetZ();

    if (mtx)
    {
        boxCenter = TransformPoint(transform.GetW(), *mtx);
        boxUnitAxis0 = TransformVector(transform.GetX(), *mtx);
        boxUnitAxis2 = TransformVector(transform.GetZ(), *mtx);
    }
    else
    {
        boxCenter = transform.GetW();
        boxUnitAxis0 = transform.GetX();
        boxUnitAxis2 = transform.GetZ();
    }

    VecFloat intersectDistance;
    unsigned int startsInside;
    Vector3 innerIntersectionPoint;
    Vector3 normal;

    Vector3 xFace = NormalizeFast(boxUnitAxis0);
    Vector3 yFace = NormalizeFast(Cross(boxUnitAxis2, boxUnitAxis0));
    Vector3 zFace = Cross(xFace, yFace);

    unsigned int intersectResult = EA::Collision::IntersectLineBox(intersectDistance, innerIntersectionPoint, normal, startsInside, 
        lineStart, lineUnitDirection, lineLength,
        boxCenter, xFace, yFace, zFace,
        boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius);

    result.v = this;
    

    if(intersectResult | startsInside)
    {
        result.normal = normal;
        result.lineParam = intersectDistance * lineLengthReciprocal;
        result.position = lineStart + intersectDistance * lineUnitDirection - fatness * result.normal;
        Vector3 innerIntersectionSgns = Select(CompGreaterEqual(innerIntersectionPoint, GetVector3_Zero()), GetVector3_One(), -GetVector3_One());
        Mask3 faces = CompLessThan(Abs(Abs(innerIntersectionPoint) - boxDimensions), Vector3(1e-05f));
        Vector3 faceOrEdge = Select(faces, GetVector3_One(), GetVector3_Zero());
        result.volParam = faceOrEdge * innerIntersectionSgns;
     
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
BoxVolume::GetBBox(const rwpmath::Matrix44Affine *tm,
                   RwpBool /*tight*/, AABBox &bBox) const
{
    Matrix44Affine otm;

    if ( tm )
    {
        otm = transform * (*tm);
    }
    else
    {
        otm = transform;
    }

// TODO: PAB: This is temporary until the standalone rwpmath:: package is used
// Unaligned vector loads removes LHS
#if defined(EA_PLATFORM_PS3_PPU)

    // Get half extent of transformed box
    VecFloat boxHalfX((vector float)vec_splat((vector float)vec_lvlx(0, &boxData.hx), 0));
    VecFloat boxHalfY((vector float)vec_splat((vector float)vec_lvlx(0, &boxData.hy), 0));
    VecFloat boxHalfZ((vector float)vec_splat((vector float)vec_lvlx(0, &boxData.hz), 0));
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));

#elif defined(EA_PLATFORM_XENON)

    VecFloat boxHalfX(__vspltw(__lvlx(&boxData.hx, 0),0));
    VecFloat boxHalfY(__vspltw(__lvlx(&boxData.hy, 0),0));
    VecFloat boxHalfZ(__vspltw(__lvlx(&boxData.hz, 0),0));
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));

#else // this case should be equivalent with updated math library

    VecFloat boxHalfX(boxData.hx);
    VecFloat boxHalfY(boxData.hy);
    VecFloat boxHalfZ(boxData.hz);
    VecFloat radiusVec(GetRadius());

#endif

    // Get half extent of transformed box
    Vector3 diag =  Abs(otm.GetX()) * boxHalfX
                 +  Abs(otm.GetY()) * boxHalfY
                 +  Abs(otm.GetZ()) * boxHalfZ;

    // Fatten by radius
    diag += radiusVec;

    bBox.Set(otm.GetW() - diag, otm.GetW() + diag);

    return TRUE;
}


/*
\toedit
\brief Gets an axis aligned bounding box for the volume
Gets an axis aligned bounding box for the volume


The bounding box of the volume is transformed by the volume relative transform and by the input parent
transform if it is not NULL.  The transformations may translate and/or rotate the bbox which may increase
the size of the bbox.

\param tm parent transformation to applied to the result bbox.  May be NULL.
\param tight if true, a slower more precise algorithm may be used to compute the bbox.
\param bBox reference to a bbox to store the result.

\return TRUE on success
*/
rwpmath::Vector3
BoxVolume::GetBBoxDiag() const
{

    // TODO: PAB: This is temporary until the standalone rwpmath:: package is used
    // Unaligned vector loads removes LHS
#if defined(EA_PLATFORM_PS3_PPU)

    // Get half extent of transformed box
    VecFloat boxHalfX((vector float)vec_splat((vector float)vec_lvlx(0, &boxData.hx), 0));
    VecFloat boxHalfY((vector float)vec_splat((vector float)vec_lvlx(0, &boxData.hy), 0));
    VecFloat boxHalfZ((vector float)vec_splat((vector float)vec_lvlx(0, &boxData.hz), 0));
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));

#elif defined(EA_PLATFORM_XENON)

    VecFloat boxHalfX(__vspltw(__lvlx(&boxData.hx, 0),0));
    VecFloat boxHalfY(__vspltw(__lvlx(&boxData.hy, 0),0));
    VecFloat boxHalfZ(__vspltw(__lvlx(&boxData.hz, 0),0));
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));

#else // this case should be equivalent with updated math library

    VecFloat boxHalfX(boxData.hx);
    VecFloat boxHalfY(boxData.hy);
    VecFloat boxHalfZ(boxData.hz);
    VecFloat radiusVec(GetRadius());

#endif

    // Get half extent of transformed box
    Vector3 diag =  Abs(transform.GetX()) * boxHalfX
                 +  Abs(transform.GetY()) * boxHalfY
                 +  Abs(transform.GetZ()) * boxHalfZ;
    diag += radiusVec;
    diag *= GetVecFloat_Two();

    return diag;
}


/**
\brief Applies a uniform scale factor to the dimensions of the box volume

This function is used by the Volume class ApplyUniformScale function. If
useProcessedFlags is enabled then the volume processed flag will be respected
and scaling performed if flag is not set. The volume processed flag will be set
after the scaling operation.

\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
BoxVolume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    // Apply scale if processing is required OR if we are to ignore flags
    if (!useProcessedFlags || !(m_flags & VOLUMEFLAG_ISPROCESSED))
    {
        transform.Pos() *= scale;
        radius *= scale;

        Vector3 dims = GetDimensions();
        dims *= scale;
        SetDimensions(dims);
    }

    // Set processed flag to identify that scaling has been performed on this volume
    if (useProcessedFlags)
    {
        SetProcessedFlag();
    }
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

// ***********************************************************************************************************
// External Functions

// ***********************************************************************************************************
// Class Member Functions

// ***********************************************************************************************************
//                                                Rw... CLASS
// ***********************************************************************************************************
} // namespace collision
} // namespace rw
