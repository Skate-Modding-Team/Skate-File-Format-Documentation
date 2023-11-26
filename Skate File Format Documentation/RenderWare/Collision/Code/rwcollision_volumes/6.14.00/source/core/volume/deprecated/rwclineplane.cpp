// (c) Electronic Arts. All Rights Reserved.

#include "rw/collision/deprecated/lineplane.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{
/**
\internal

\brief
Utility for intersecting a line segment with an axis-aligned plane side (half-space).

Returns intersect distance (scaled by the length of the segment).

\note This value will be accurate only if there is an intersection.
If there is an intersection, then dist.den > minRecip and num >= 0.

\param dist Intersect parametric distance (returned as a fraction with positive denominator).
\param orig_i Start of line segment, i-th coordinate
\param seg_i displacement of end of line segment i-th coordinate from orig_i.
\param sign 1 if the normal points in the axis' positive direction, -1 otherwise
\param disp The plane displacement, equal to dot product of normal and a point in the plane.

\return 1 if the segment intersects the volume, -1 if it does not intersect and
 the distance to the volume is not decreasing at orig, in the seg direction.  Returns
 zero otherwise.
\see rwcCylinderLineSegIntersect, rwcSphereLineSegIntersect, ThinTriangleLineSegIntersect
*/

int32_t
rwcPlaneLineSegIntersect(Fraction *dist, float orig_i, float seg_i, float sign, float disp)
{
    rwcDEPRECATED("Use EA::Collision::detail::linequery::IntersectLineOneSidedPlane_Branching()");
    EA_ASSERT(dist);

    float c;

    //  Surface intersect:
    //  p = o + t*r
    //  (p-p0).n = R  (R = radius)
    //  o.n + t*r.n - p0.n = R
    //  t = (R + (p0-o).n)/r.n

    c = orig_i*sign - disp;

    if (c <= 0.0f)        // Segment origin lies inside of plane
    {
        dist->num = 0.0f;
        dist->den = 1.0f;
        return 1;
    }
    // Segment origin lies outside of plane
    dist->num = c;
    dist->den = -seg_i*sign;

    if (dist->den < MINIMUM_RECIPROCAL || dist->den < c * MINIMUM_RECIPROCAL)
    {
        return -1;      // seg is pointing away from plane
    }

    // return 1=plane is hit, and 0=plane is too far away.

    return c < dist->den;
}
}
}
