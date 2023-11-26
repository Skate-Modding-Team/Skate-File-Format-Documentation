// (c) Electronic Arts. All Rights Reserved.
 /*************************************************************************************************************

 File: rwctriangle.cpp

 Purpose: Implementation of the Triangle primitive class

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include <rw/math/version.h>

#include "rw/collision/triangle.h"
#include "eacollision/linetriangle.h"
#include "eacollision_primitives/version.h"

// Use platform specific maths 
using namespace rwpmath;

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts

  


// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\brief This is the virtual function table that is shared by all triangle volumes.
\see TriangleVolume
*/
Volume::VTable 
globalTriangleVTable = {
    rw::collision::VOLUMETYPETRIANGLE,
    (Volume::GetBBoxFn)(&TriangleVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&TriangleVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&TriangleVolume::CreateGPInstance),
    (Volume::LineSegIntersectFn)(&TriangleVolume::LineSegIntersect),
    (Volume::ReleaseFn)(&TriangleVolume::Release),
    "TriangleVolume",
    0,
    0,
    0,
    0,
    (Volume::ApplyUniformScaleFn)(&TriangleVolume::ApplyUniformScale)
};

// Compose Result - Function Prototype (needed for wii)
RwpBool ComposeResult(VolumeLineSegIntersectResult &result, 
                     const unsigned int intersectResult, 
                     const unsigned int startsInside, 
                     rwpmath::VecFloatInParam intersectDistance,
                     rwpmath::VecFloatInParam lineLengthReciprocal,
                     rwpmath::VecFloatInParam lineFatness,
                     rwpmath::VecFloatInParam triFatness,
                     rwpmath::Vector3::InParam lineStart,
                     rwpmath::Vector3::InParam lineUnitDirection,
                     rwpmath::Vector3::InParam normal,
                     rwpmath::Vector3::InParam v0,
                     rwpmath::Vector3::InParam v1,
                     rwpmath::Vector3::InParam v2,
                     rwpmath::Vector3::InParam barycentricCoords);

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
TriangleVolume::GetBBox(const rwpmath::Matrix44Affine *tm, 
                        RwpBool /*tight*/, AABBox &bBox) const
{
    Matrix44Affine otm;
    
    if ( tm )
    {
        otm.GetX() = TransformPoint(transform.GetX(), *tm);
        otm.GetY() = TransformPoint(transform.GetY(), *tm);
        otm.GetZ() = TransformPoint(transform.GetZ(), *tm);
    }
    else
    {
        otm = transform;
    }

    bBox.m_min = Min( otm.GetX(), Min( otm.GetY(), otm.GetZ() ) );
    bBox.m_max = Max( otm.GetX(), Max( otm.GetY(), otm.GetZ() ) );


// TODO: PAB: This is temporary until the standalone rwpmath:: package is used
// Unaligned vector load removes a LHS
#if defined(EA_PLATFORM_PS3_PPU)
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));
#elif defined(EA_PLATFORM_XENON)
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));
#else // this case should be equivalent with updated math library
    VecFloat radiusVec(GetRadius());
#endif

    // Fatten by radius
    bBox.m_min -= radiusVec;
    bBox.m_max += radiusVec;

    return  TRUE ;
}


rwpmath::Vector3
TriangleVolume::GetBBoxDiag() const
{

    Vector3 min = Min( transform.GetX(), Min( transform.GetY(), transform.GetZ() ) );
    Vector3 max = Max( transform.GetX(), Max( transform.GetY(), transform.GetZ() ) );

    // TODO: PAB: This is temporary until the standalone rwpmath:: package is used
    // Unaligned vector load removes a LHS
#if defined(EA_PLATFORM_PS3_PPU)
    VecFloat radiusVec((vector float)vec_splat((vector float)vec_lvlx(0, &GetRadius()), 0));
#elif defined(EA_PLATFORM_XENON)
    VecFloat radiusVec(__vspltw(__lvlx(&GetRadius(), 0),0));
#else // this case should be equivalent with updated math library
    VecFloat radiusVec(GetRadius());
#endif

    // Fatten by radius
    min -= radiusVec;
    max += radiusVec;

    return  max - min ;
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

/**
This function does nothing.
*/
void
GPTriangle::GetBBox( AABBox &/*bbox*/ ) const
{
    EA_ASSERT(Type() == GPInstance::TRIANGLE);
}


// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

/**
\internal
\brief Create the generalized primitive instance data.

This precomputes some data that is used for the generalized primitive intersection test.
The face direction and three edge directions are computed in world coordinates,
the radius, and several virtual function pointers are loaded into the instance structure.

Instance data is loaded as follows

    pos = v0
    face_normal2 = v1
    face_normal1 = v2
    face_normal0 = normal
    edge_direction0 = Normalize(v2 - v0)
    edge_direction1 = Normalize(v1 - v2)
    edge_direction2 = Normalize(v0 - v1)
    box_size0 = Magnitude(v2 - v0)
    box_size1 = Magnitude(v1 - v2)
    box_size2 = Magnitude(v0 - v1)

Note that the edges directions are all pointing backwards (i.e. clockwise).

\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
RwpBool
TriangleVolume::CreateGPInstance(GPInstance &instance, const Matrix44Affine *tm) const
{

    Matrix44 mtx = GetMatrix44_Identity();
    if(tm)
    {
        mtx = Matrix44(*tm);
    }

    Vector3 v0 = transform.GetRow(0);
    Vector3 v1 = transform.GetRow(1);
    Vector3 v2 = transform.GetRow(2);
    Vector3 normal;
    GetNormal(normal);

    if(tm)
    {
        v0 = TransformPoint(v0, mtx);
        v1 = TransformPoint(v1, mtx);
        v2 = TransformPoint(v2, mtx);
        normal = TransformVector(normal, mtx);
    }

    // initialize GP instance
    GPTriangle *triangle = static_cast<GPTriangle*>(&instance);
    triangle->Initialize(v0, v1, v2, GetRadius(), GetFlags(), GetEdgeCos(0), GetEdgeCos(1), GetEdgeCos(2), reinterpret_cast<uintptr_t>(this), 0, normal);

    return  TRUE ;
}


/**
\brief
Test whether a triangle volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.
For example, if the line is 10 units long, and intersects the volume 8 units from the start, then the 
\e  lineParam is 0.8.

\par Implementation
This algorithm is basically a region walk.  We start at the line start point and determine what region
of the triangle it is in.  Possible region types are 
\li "endcap" regions, where the closest point is on an end cap.
\li "cylindrical" regions, where the closest point lies on an edge
This finds the parameteric distance until (a) the distance from the line to the feature is equal to the
radius of the volume, or (b) the line exits the region of the feature.  If (a) is less than (b), we compute
the intersection and return.  If (b) is less, we advance the line start \e  pt1 and continue the region walk.

\param pt1      The origin of the line segment.
\param pt2      The end of the line segment.
\param tm       Transformation Matrix of the parent reference frame.
\param result   This will be set to the result of the test.
\param fatness  The fatness of the line

\return TRUE if the segment intersects the volume.

\see BoxVolume::LineSegIntersect
*/

RwpBool
TriangleVolume::LineSegIntersect(Vector3::InParam pt1, 
                                 Vector3::InParam pt2,
                                 const Matrix44Affine *tm,
                                 VolumeLineSegIntersectResult &result,
                                 const float fatness /*=0.0f*/) const
{
    Vector3 v0, v1, v2;
    GetPoints(v0, v1, v2, tm);
    RwpBool hit;

    if(VOLUMEFLAG_TRIANGLEONESIDED & GetFlags())
    {
        hit = TriangleLineSegIntersect(result, pt1, pt2-pt1, v0, v1, v2, fatness, GetRadius());
    }
    else
    {
        hit = TriangleLineSegIntersectTwoSided(result, pt1, pt2-pt1, v0, v1, v2, fatness, GetRadius());
    }
    
    result.v = this;

    return hit;
}
#endif // !defined(EA_PLATFORM_PS3_SPU)

/** 
\brief
\Completes a VolumeLineSegIntersectResult struct based on the results of the eacollision_primitives triangle linequery.

\param  result                  The VolumeLineSegIntersect result structure to be completed. 
\param  intersectResult         int signifying if the line intersects the volume from outside. 
\param  startsInside            int signifying if the lineStart is within the volume. 
\param  intersectDistance       The distance from linestart to point of intersection. 
\param  lineLengthReciprocal    The reciprocal of the line length . 
\param  lineFatness             The line fatness. 
\param  triFatness              The triangle fatness. 
\param  lineStart               The line start. 
\param  lineUnitDirection       The line unit direction. 
\param  normal                  The intersection normal. 
\param  v0                      Triangle Vertex0 
\param  v1                      Triangle Vertex1 
\param  v2                      Triangle Vertex2
\param  barycentricCoords       The barycentric coords. 

\return TRUE if the segment intersects the volume.
 */
RwpBool ComposeResult(VolumeLineSegIntersectResult &result, 
                     const unsigned int intersectResult, 
                     const unsigned int startsInside, 
                     rwpmath::VecFloatInParam intersectDistance,
                     rwpmath::VecFloatInParam lineLengthReciprocal,
                     rwpmath::VecFloatInParam lineFatness,
                     rwpmath::VecFloatInParam triFatness,
                     rwpmath::Vector3::InParam lineStart,
                     rwpmath::Vector3::InParam lineUnitDirection,
                     rwpmath::Vector3::InParam normal,
                     rwpmath::Vector3::InParam v0,
                     rwpmath::Vector3::InParam v1,
                     rwpmath::Vector3::InParam v2,
                     rwpmath::Vector3::InParam barycentricCoords)                     
{
    if(intersectResult | startsInside)
    {
        result.normal = normal;
        result.lineParam = intersectDistance * lineLengthReciprocal;
        result.position = lineStart + intersectDistance * lineUnitDirection - normal * lineFatness;

#if EACOLLISION_PRIMITIVES_VERSION >= EACOLLISION_PRIMITIVES_CREATE_VERSION_NUMBER( 1, 8, 1)
        Vector3 innerIntersectPosition = barycentricCoords.GetX() * v0 +
            barycentricCoords.GetY() * v1 +
            barycentricCoords.GetZ() * v2;
#else
        Vector3 innerIntersectPosition = barycentricCoords.GetX() * v1 +
            barycentricCoords.GetY() * v0 +
            barycentricCoords.GetZ() * v2;
#endif

        Vector3 innerToReportedIntersect = result.position - innerIntersectPosition;
        VecFloat depth = triFatness - Dot(innerToReportedIntersect, normal);
#if EACOLLISION_PRIMITIVES_VERSION >= EACOLLISION_PRIMITIVES_CREATE_VERSION_NUMBER( 1, 8, 1)
        result.volParam = Vector3(barycentricCoords.GetY(), barycentricCoords.GetZ(), depth * depth);
#else
        result.volParam = Vector3(barycentricCoords.GetX(), barycentricCoords.GetZ(), depth * depth);
#endif

        return TRUE;
    }
    else
    {
        //compute the normal
        Cross(v0-v1, v0-v2, result.normal);
        result.normal = Normalize(result.normal);
        result.lineParam = 0.0f;
        result.position = GetVector3_Zero();
        result.volParam = GetVector3_Zero();

        return FALSE;
    }
}

/**
\brief
Test whether a triangle volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.

\param pt1     The origin of the line segment.
\param pt2     The end of the line segment.
\param mtx     Transformation Matrix of the parent reference frame.
\param result  This will be set to the result of the test.
\param fatness How fat the line is (default=0.0)

\return TRUE if the segment intersects the volume.
 */
RwpBool
TriangleLineSegIntersect(VolumeLineSegIntersectResult &result,
                         rwpmath::Vector3::InParam lineStart,
                         rwpmath::Vector3::InParam lineDelta,
                         rwpmath::Vector3::InParam v0,
                         rwpmath::Vector3::InParam v1,
                         rwpmath::Vector3::InParam v2,
                         const float lineFatness,
                         const float triFatness)
{
    Vector3 lineUnitDirection;
    VecFloat lineLengthReciprocal;
    const VecFloat lineLength = EA::Collision::xmath::NormalizeReciprocalReturnMagnitudeFast(lineDelta, lineUnitDirection, lineLengthReciprocal);
    const VecFloat lineFatnessV(lineFatness);
    const VecFloat triFatnessV(triFatness);

    VecFloat intersectDistance;
    unsigned int intersectResult;
    unsigned int startsInside = 0;
    Vector3 barycentricCoords;
    Vector3 normal;
    

    if(lineFatness + triFatness == 0.0f)
    {

#if EACOLLISION_PRIMITIVES_VERSION >= EACOLLISION_PRIMITIVES_CREATE_VERSION_NUMBER( 1, 8, 1)
        intersectResult = EA::Collision::IntersectLineOneSidedTriangle(intersectDistance, barycentricCoords, normal, 
            lineStart, lineUnitDirection, lineLength,
            v0, v1, v2);
#else
        intersectResult = EA::Collision::IntersectLineOneSidedTriangle(intersectDistance, barycentricCoords, normal, 
            lineStart, lineUnitDirection, lineLength,
            v1, v0, v2);   
#endif
    }
    else
    {
#if EACOLLISION_PRIMITIVES_VERSION >= EACOLLISION_PRIMITIVES_CREATE_VERSION_NUMBER( 1, 8, 1)
        intersectResult = EA::Collision::IntersectLineOneSidedTriangle(intersectDistance, barycentricCoords, normal, startsInside, 
            lineStart, lineUnitDirection, lineLength,
            v0, v1, v2, lineFatnessV + triFatnessV);
#else
        intersectResult = EA::Collision::IntersectLineOneSidedTriangle(intersectDistance, barycentricCoords, normal, startsInside, 
            lineStart, lineUnitDirection, lineLength,
            v1, v0, v2, lineFatnessV + triFatnessV);
#endif
    }

    return (ComposeResult(result, intersectResult, startsInside, 
                        intersectDistance, lineLengthReciprocal, lineFatnessV, triFatnessV,
                        lineStart, lineUnitDirection, normal,
                        v0, v1, v2, barycentricCoords)); 
}
    

/**
\brief
Test whether a two sided triangle volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.

\param pt1     The origin of the line segment.
\param pt2     The end of the line segment.
\param mtx     Transformation Matrix of the parent reference frame.
\param result  This will be set to the result of the test.
\param fatness How fat the line is (default=0.0)

\return TRUE if the segment intersects the volume.
*/
RwpBool
TriangleLineSegIntersectTwoSided(VolumeLineSegIntersectResult &result,
                         rwpmath::Vector3::InParam lineStart,
                         rwpmath::Vector3::InParam lineDelta,
                         rwpmath::Vector3::InParam v0,
                         rwpmath::Vector3::InParam v1,
                         rwpmath::Vector3::InParam v2,
                         const float lineFatness,
                         const float triFatness)
{
    Vector3 lineUnitDirection;
    VecFloat lineLengthReciprocal;
    const VecFloat lineLength = EA::Collision::xmath::NormalizeReciprocalReturnMagnitudeFast(lineDelta, lineUnitDirection, lineLengthReciprocal);
    const VecFloat lineFatnessV(lineFatness);
    const VecFloat triFatnessV(triFatness);

    VecFloat intersectDistance;
    unsigned int intersectResult;
    unsigned int startsInside = 0;
    Vector3 barycentricCoords;
    Vector3 normal;
    
    if(lineFatness + triFatness == 0.0f)
    {
#if EACOLLISION_PRIMITIVES_VERSION >= EACOLLISION_PRIMITIVES_CREATE_VERSION_NUMBER( 1, 8, 1)
        intersectResult = EA::Collision::IntersectLineTwoSidedTriangle(intersectDistance, barycentricCoords, normal, 
            lineStart, lineUnitDirection, lineLength,
            v0, v1, v2);
#else
        intersectResult = EA::Collision::IntersectLineTwoSidedTriangle(intersectDistance, barycentricCoords, normal, 
            lineStart, lineUnitDirection, lineLength,
            v1, v0, v2);
#endif
    }
    else
    {
#if EACOLLISION_PRIMITIVES_VERSION >= EACOLLISION_PRIMITIVES_CREATE_VERSION_NUMBER( 1, 8, 1)
        intersectResult = EA::Collision::IntersectLineTwoSidedTriangle(intersectDistance, barycentricCoords, normal, startsInside, 
            lineStart, lineUnitDirection, lineLength,
            v0, v1, v2, lineFatnessV + triFatnessV);
#else
        intersectResult = EA::Collision::IntersectLineTwoSidedTriangle(intersectDistance, barycentricCoords, normal, startsInside, 
            lineStart, lineUnitDirection, lineLength,
            v1, v0, v2, lineFatnessV + triFatnessV);
#endif
    }

    return (ComposeResult(result, intersectResult, startsInside, 
        intersectDistance, lineLengthReciprocal, lineFatnessV, triFatnessV,
        lineStart, lineUnitDirection, normal,
        v0, v1, v2, barycentricCoords)); 
}


/**
\brief Applies a uniform scale factor to the dimensions of the triangle volume

This function is used by the Volume class ApplyUniformScale function. If
useProcessedFlags is enabled then the volume processed flag will be respected
and scaling performed if flag is not set. The volume processed flag will be set
after the scaling operation.

\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
TriangleVolume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    // Apply scale if processing is required OR if we are to ignore flags
    if (!useProcessedFlags || !(m_flags & VOLUMEFLAG_ISPROCESSED))
    {
        radius *= scale;

        Vector3 p1,p2,p3;
        GetPoints(p1,p2,p3);
        p1 *= scale;
        p2 *= scale;
        p3 *= scale;
        SetPoints(p1,p2,p3);
    }

    // Set processed flag to identify that scaling has been performed on this volume
    if (useProcessedFlags)
    {
        SetProcessedFlag();
    }
}

} // namespace collision
} // namespace rw
