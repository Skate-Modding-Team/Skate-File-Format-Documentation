// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINETRIANGLE_H
#define RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINETRIANGLE_H

#include "rw/collision/volume.h"

#include "rw/collision/deprecated/linesphere.h"
#include "rw/collision/deprecated/linecylinder.h"


namespace rw
{
namespace collision
{
/**
\internal

\brief The region enumeration is used by rwc_TriangleNearestPoint and other line segment intersection code to
denote the feature regions around a triangle.
*/
enum rwc_Region
{
    rwc_REGION_VERT0 = 0,          ///< the nearest point is vert0 of the triangle
    rwc_REGION_VERT1 = 1,          ///< the nearest point is vert1
    rwc_REGION_VERT2 = 2,          ///< the nearest point is vert2
    rwc_REGION_EDGE0 = 3,          ///< the nearest point is edge0
    rwc_REGION_EDGE1 = 4,          ///< the nearest point is edge1
    rwc_REGION_EDGE2 = 5,          ///< the nearest point is edge2
    rwc_REGION_FACE  = 6,          ///< the nearest point is on the face of the triangle
    rwc_REGION_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};
         

rwc_Region rwc_TriangleNearestPoint(rwpmath::Vector3 &outPoint,
                                   float &outU,
                                   float &outV,
                                   rwpmath::Vector3::InParam inPoint,
                                   rwpmath::Vector3::InParam v0,
                                   rwpmath::Vector3::InParam v1,
                                   rwpmath::Vector3::InParam v2);

// ***********************************************************************************************************
//  Line intersection


/**
\internal
These tolerance used by \e  ThinTriangleLineSegIntersect to test if the line lies in the plane of
the triangle.
*/
#define RTINTSECEPSILON (float)(1e-8)

/**
\internal
These tolerance used by \e  ThinTriangleLineSegIntersect to test if the line start point lies in the plane
of the triangle.
*/
#define RTINTSECEDGEEPS (float)(1e-5)

/**
\internal
This function determines if a line segment hits the front face of a triangle.

The point of intersection is returned in the result.  However, the result
normal and volume are not set.  The triangle does not have any fatness radius, hence the name "thin".
The \e  lineDelta parameter is the offset from the start point of the line to the end.  The line end
point is \e  lineStart + \e  lineDelta.  Intersection with the backface of the triangle is ignored.
If you want to test for backface intersection, you can reverse the line start and end points and call
the function again. If the line lies in the plane of the triangle, false is returned.

\param result output the intersection position, lineParam, and volParam
\param lineStart the starting point of the line
\param lineDelta the offset of the line from start to end
\param v0 a corner of the triangle
\param v1 a corner of the triangle
\param v2 a corner of the triangle
\return true if the line intersects triangle

\see rwcPlaneLineSegIntersect, rwcSphereLineSegIntersect, rwcCylinderLineSegIntersect
*/
inline
RwpBool
ThinTriangleLineSegIntersect(VolumeLineSegIntersectResult &result,
                             rwpmath::Vector3::InParam lineStart,
                             rwpmath::Vector3::InParam lineDelta,
                             rwpmath::Vector3::InParam v0,
                             rwpmath::Vector3::InParam v1,
                             rwpmath::Vector3::InParam v2 )
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineOneSidedTriangle()");
    rwpmath::Vector3  edge1, edge2, tVec, pVec, qVec;
    float    det;
    float    lo, hi, u, t;

    u = t = 0.0f;

    //  Find vectors for two edges sharing vert0
    edge1 = v1 - v0;
    edge2 = v2 - v0;

    //  Begin calculating determinant - also used to calculate U parameter
    pVec = rwpmath::Cross( lineDelta, edge2 );

    //  If determinant is near zero, ray lies in plane of triangle, if negative, triangle is backfacing
    det = rwpmath::Dot( edge1, pVec );

    if (det > RTINTSECEPSILON)
    {
        //  Calculate bounds for parameters with tolerance
        lo = - det*RTINTSECEDGEEPS;
        hi = det - lo;

        //  Calculate U parameter and test bounds
        tVec = lineStart - v0;
        u = rwpmath::Dot( tVec, pVec );

        if (u >= lo && u <= hi)
        {
            //  Calculate V parameter and test bounds
            qVec = rwpmath::Cross( tVec, edge1 );
            t = rwpmath::Dot( lineDelta, qVec );

            if (t >= lo && (u + t) <= hi)
            {
                //  Calculate t, and make sure intersection is in bounds of line
                result.lineParam = rwpmath::Dot(edge2, qVec);

                //  Within bounds of line?
                if (result.lineParam >= lo && result.lineParam <= hi)
                {
                    hi = rw::math::fpu::Reciprocal(det);
                    result.lineParam *= hi;
                    result.position = lineStart + lineDelta * rwpmath::VecFloat(result.lineParam);
                    result.volParam.Set(u*hi, t*hi, 0.0f);
                    return TRUE;
                }
            }
        }
    }
    return FALSE;
}
}
}
#endif //!RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINETRIANGLE_H
