// (c) Electronic Arts. All Rights Reserved.

#include "rw/collision/deprecated/linesphere.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{
/**
\internal
\brief Utility for intersecting a line segment with a sphere.

Returns intersect distance (scaled by the length of the segment).
This value will be accurate only if there is an intersection.
If there is an intersection, then dist.den >= 0.

If dist == NULL, then the function returns true iff orig is inside the volume.

\param dist Intersect distance (returned as a fraction).
\param orig Start of line segment
\param seg displacement of end of line segment from \e  orig.
\param center The sphere's center.
\param radius The sphere's radius.

\return 1 if the segment intersects the volume, -1 if it does not intersect and
 the distance to the volume is not decreasing at orig, in the seg direction.  Returns
 zero otherwise.

\see rwcPlaneLineSegIntersect, rwcCylinderLineSegIntersect
 */
int32_t
rwcSphereLineSegIntersect(Fraction *dist,
                          const Vector3 & orig,
                          const Vector3 & seg,
                          const Vector3 & center,
                          const float radius)
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineSphere()");
    Vector3 D, xp;
    float rr, ds, dd, ss, arg, drMr2;

    /*  Surface intersect:
        p = o + t*r
        |p-p0|^2 = R^2
        |r|^2*t^2 - 2*(p0-o).r*t + |p0-o|^2 - R^2 = 0
     */

    D = center - orig;
    dd = Dot(D, D);
    rr = radius*radius;

    EA_ASSERT(dist);

    if (dd < rr)
    {   // Segment origin lies inside sphere
        dist->num = 0.0f;
        dist->den = 1.0f;
        return 1;
    }

    // Segment origin lies outside of sphere
    ds = Dot(D, seg);
    if (ds <= 0)
    {   // Segment is tangent to or points away from sphere
        return -1;
    }

    ss = Dot(seg, seg);
    xp = Cross(D, seg);

    //  Use the formula (D x S)^2 = DD*SS - (DS)^2

    arg = -static_cast<float>(Dot(xp, xp)) + ss * rr;

    if (arg < 0)
    {   // Ray does not intersect sphere
        return 0;
    }

    drMr2 = ds - ss;
    if (drMr2 > 0 && drMr2*drMr2 > arg)
    {
        // Segment does not reach sphere
        return 0;
    }

    dist->num = ds - rw::math::fpu::Sqrt(arg);
    dist->den = ss;

    EA_ASSERT(ss >= MINIMUM_RECIPROCAL);

    return 1;
}
}
}
