// (c) Electronic Arts. All Rights Reserved.

#include "rw/collision/deprecated/linetriangle.h"
#include "rw/collision/triangle.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{


/**
\internal

\brief
 rwc_TriangleNearestPoint computes nearest point on triangle to inPoint.

This algorithm is based on the paper by David Eberly at
http://www.magic-software.com/Documentation/pt3tri3.pdf

\note This computes the point nearest to inPoint on the triangle defined by
the three point (v0,v1,v2).  The region number is returned, and the
point on the triangle nearest to inPoint is returned.  This does not return
the distance because it is just as efficient for the caller to compute it.

\param outPoint point on triangle nearest to inPoint
\param outU parametric coordinates of the outPoint on the triangle
\param outV parametric coordinates of the outPoint on the triangle
\param inPoint input point
\param v0 a triangle vertex
\param v1 a triangle vertex
\param v2 a triangle vertex

\return Returns region number, see rwc_Region enum.

For example, if the nearest point is on the edge connecting v1 and v2, then
the region would be 5 and the u+v would equal 1.  The value of outPoint is
always v0*(1-u-v)+v1*u+v2*v which is the same as v0+e0*u+e1*v.

 */
rwc_Region
rwc_TriangleNearestPoint(Vector3 &outPoint,
                     float &outU,
                     float &outV,
                     Vector3::InParam inPoint,
                     Vector3::InParam v0,
                     Vector3::InParam v1,
                     Vector3::InParam v2)
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineTriangle()");
    Vector3 bp, edge0, edge1;
    float a, b, c, d, e;
    //float f;
    float det, s, t;
    float num, den;
    uint32_t nf;
    rwc_Region region;

    edge0 = v1 - v0;
    edge1 = v2 - v0;
    bp = v0 - inPoint;

    a = Dot(edge0, edge0);
    b = Dot(edge0, edge1);
    c = Dot(edge1, edge1);
    d = Dot(edge0, bp);
    e = Dot(edge1, bp);
    //f = Dot(bp, bp);

    //  The triangle is parameterized by s[0,det] along edge0 and t[0,det] along
    //    edge1.  We compute the nearest point (s,t) and then looking at the
    //    values of s and t, we can tell what region the point is in.

    det = a*c - b*b;
    s = b*e - c*d;
    t = b*d - a*e;

    if (det < MINIMUM_RECIPROCAL)
    {
        // This is special code to handle triangles with zero area.
        // Just choose the longest side.

        num = a - 2*b + c;      // this is dot(edge2,edge2)
        if (a > c)
        {
            nf = (a > num ? 0u : 2u);
        }
        else
        {
            nf = (c > num ? 1u : 2u);
        }
    }
    else if (s+t > det)
    {
        if (s < 0.0f)
        {
            nf = (c+e < b+d ? 1u : 2u);
        }
        else if (t < 0.0f)
        {
            nf = (a+d < b+e ? 0u : 2u);
        }
        else
        {
            nf = (2u);
        }
    }
    else if (s < 0.0f)
    {
        nf = (-e > 0.0f ? 1u : 0u);
    }
    else if (t < 0.0f)
    {
        nf = (-d > 0.0f ? 0u : 1u);
    }
    else
    {
        nf = 3;
    }

    //  Having determined which feature is nearest to the point,
    //    normalize (s,t) to the range [0,1] such that s+t <= 1.

    switch (nf)
    {
    case 0:                  // nearest point is on edge0
        if (d >= 0.0f)
        {
            s = 0.0f;
            region = rwc_REGION_VERT0;
        }
        else if (-d >= a)
        {
            s = 1.0f;
            region = rwc_REGION_VERT1;
        }
        else
        {
            s = -d/a;
            region = rwc_REGION_EDGE0;
        }
        t = 0.0f;

        break;
    case 1:                  // nearest point is on edge1
        s = 0.0f;

        if (e >= 0.0f)
        {
            t = 0.0f;
            region = rwc_REGION_VERT0;
        }
        else if (-e >= c)
        {
            t = 1.0f;
            region = rwc_REGION_VERT2;
        }
        else
        {
            t = -e/c;
            region = rwc_REGION_EDGE1;
        }
        break;
    case 2:                  // nearest point is on edge2
        num = c + e - b - d;
        if (num <= 0.0f)
        {
            s = 0.0f;
            t = 1.0f;
            region = rwc_REGION_VERT2;
        }
        else
        {
            den = a - 2*b + c;
            if (num >= den)
            {
                s = 1.0f;
                t = 0.0f;
                region = rwc_REGION_VERT1;
            }
            else
            {
                s = num/den;
                t = 1.0f - s;
                region = rwc_REGION_EDGE2;
            }
        }
        break;
    default:                 // nearest point is interior
        num = rw::math::fpu::Reciprocal(det);
        s *= num;
        t *= num;
        region = rwc_REGION_FACE;
        break;
    }

    //  Convert the parameterized point (s,t) to an actual 3D point, and return the region number.

    outU = s;
    outV = t;
    outPoint = v0 + edge0 * VecFloat(s) + edge1 * VecFloat(t);

    return region;
}


/**
\internal

\brief
Test whether a fat triangle is intersected by a line segment.

On entry you must set the result.normal to the triangle normal.
If the function returns true, the result position is set to the intersection position.
The result normal is the surface normal of the volume at the point of intersection.
The result lineParam is the parametric distance from lineStart to intersection.
The result volParam (x,y) is the barycentric coordinates of the point of nearest
point on the triangle to the point of intersection.  The volParam(z) is the
penetration distance of lineStart into the trangle, squared.

\param lineStart The origin of the line segment.
\param lineDelta The offset to the end of the line segment.
\param result   input: You must set the normal, output: the result of the test.
\param v0       Triangle vertex 0
\param v1       Triangle vertex 1
\param v2       Triangle vertex 2
\param radius   Triangle fatness

\return TRUE if the segment intersects the volume.
 */
RwpBool
FatTriangleLineSegIntersect(VolumeLineSegIntersectResult &result,
                            Vector3::InParam lineStart, Vector3::InParam lineDelta,
                            Vector3::InParam v0, Vector3::InParam v1,
                            Vector3::InParam v2, float radius )
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineTriangle()");
    Vector3 a, b, d, s, t, axd, dxb, bxa;
    float u, v, w, det, invdet, sign, q, r;
    rwc_Region region;
    uint32_t regionCount = 5;
    Fraction distA, distB;
    int32_t iA;

    result.lineParam = 0.0f;

    s  = lineStart - v0;
    d = lineDelta;

    a = v1 - v0;
    b = v2 - v0;

    // calc determinant = a*(dxb) = b*(axd) = d*(bxa)

    dxb = Cross(d, b);
    det = Dot(a, dxb);
    sign = 1.0f;

    if (det < 0.0f)
    {
        det = -det;
        sign = -1.0f;
    }

    // Test if lineStart is beyond the back face

    r = sign * static_cast<float>(Dot(result.normal, s));
    if (r < -radius)
    {
        return FALSE;
    }

    // calc barycentric coordinates and test in range
    // this also rejects cases where line is parallel to triangle face and is outside of the triangle slab

    u = sign * static_cast<float>(Dot(s, dxb));
#if RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER(1,6,0)
    q = radius * Magnitude(dxb);
#else
    q = radius * MagnitudeFast(dxb);
#endif

    if (u < -q || u > det + q)
    {
        return FALSE;
    }

    axd = Cross(a, d);
    v = sign * static_cast<float>(Dot(s, axd));
#if RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER(1,6,0)
    r = radius * Magnitude(axd);
#else
    r = radius * MagnitudeFast(axd);
#endif

    if (v < -r || v > det + r || u + v > det + q + r)
    {
        return FALSE;
    }

    // Test for intersection with near face of fat triangle
    // If det < MINIMUM_RECIPROCAL, the line is parallel to triangle face, and we already know from previous check
    // that it must be inside the triangle slab

    if (det > MINIMUM_RECIPROCAL)
    {
        t = s - result.normal * rwpmath::VecFloat(radius * sign);
        bxa = Cross(b, a);
        w = -sign * static_cast<float>(Dot(t, bxa));

        if (w > det)
        {
            //  Line delta is not long enough to reach the near face

            return FALSE;
        }

        if (w >= 0.0f)
        {
            // Advance the lineParam to the front face plane

            invdet = rw::math::fpu::Reciprocal(det);
            result.lineParam += w * invdet;

            u = sign * static_cast<float>(Dot(t, dxb));

            if (u >= 0.0f && u <= det)
            {
                v = sign * static_cast<float>(Dot(t, axd));

                if (v >= 0.0f && u + v <= det)
                {
                    // We hit the flat front face

                    result.position = lineStart + d * rwpmath::VecFloat(result.lineParam);
                    result.normal *= rwpmath::VecFloat(sign);
                    result.volParam.Set(u * invdet, v * invdet, 0.0f);
                    return TRUE;
                }
            }

            s += d * rwpmath::VecFloat(result.lineParam);
            d *= rwpmath::VecFloat(1.0f - result.lineParam);
        }
    }

    // If we arrive here it means that the line is near the triangle, but doesn't hit
    // the front face.  We first determine if the lineStart is between the flat faces.
    // If so, return, else begin region walk to test the curved parts.

    s += v0;            // this means s = lineStart + lineDelta*lineParam

    region = rwc_TriangleNearestPoint(t, u, v, s, v0, v1, v2);

    // test if s is inside the fat triangle
    w = radius*radius - static_cast<float>(MagnitudeSquared(s - t));
    if (w > 0.0f)
    {
        result.position = s;
        Vector3 sep = s - t;
        VecFloat distSq = MagnitudeSquared(sep);
        static const float ftolSq = 10.0f * EPSILON * EPSILON;
        result.normal= Select(CompGreaterThan(distSq, VecFloat(ftolSq) * MagnitudeSquared(s)), InvSqrtFast(distSq)*sep, result.normal);
        result.volParam.Set(u, v, w);
        return TRUE;
    }

    if (region==rwc_REGION_FACE)    // face region, no walking is needed.
    {
        result.position = s;

        if (det < 0.0f)
        {
            result.normal *= rwpmath::VecFloat(-1.0f);
        }
        result.volParam.Set(u, v, 0.0f);
        return TRUE;
    }

    // Begin region walk

    while (regionCount--)
    {
        if (region <= rwc_REGION_VERT2)        // vertex region
        {
            iA = rwcSphereLineSegIntersect(&distA, s, d, t, radius);

            if (iA < 0)      // d is pointing AWAY from the sphere
            {
                return FALSE;
            }

            /**
                At this point we know that the line hits the sphere, or it misses
                it but is pointing toward it (iA==0).  The vertex region is actually
                a wedge of a sphere, like a piece of an orange, sectioned off by
                two planes which are perpendicular to the two edge vectors.
                If we have a hit (iA==1) then we check that the distance to the
                intersection is less than the distance to either plane.
            */

            //  Compute e0,e1 the two edges adjacent to the current vertex
            switch (region)
            {
            case rwc_REGION_VERT0:
                a = v1 - v0;
                b = v2 - v0;
                break;
            case rwc_REGION_VERT1:
                a = v0 - v1;
                b = v2 - v1;
                break;
            default:
                a = v0 - v2;
                b = v1 - v2;
            }

            // compare region exit distance
            if ((distB.den = Dot(d, a)) > MINIMUM_RECIPROCAL &&
                (distB.num = Dot(t - s, a)) > 0.0f &&
                (iA == 0 || !FracLT(distA, distB)))
            {
                distA = distB;
                iA = (region + 6)/2;  // go to edge region
            }

            // test the other edge region
            if ((distB.den = Dot(d, b)) > MINIMUM_RECIPROCAL &&
                (distB.num = Dot(t - s, b)) > 0.0f &&
                (iA == 0 || !FracLT(distA, distB)))
            {
                distA = distB;
                iA = (region + 9)/2;  // go to edge region
            }

            if (iA == 0)
            {
                return FALSE;
            }

            if (iA > 2)             // walk to edge region 3, 4, or 5
            {
                region = rwc_Region(iA);
            }
            else            // Hits fat vertex first.  We are done.
            {
                w = distA.num*rw::math::fpu::Reciprocal(distA.den);
                result.lineParam += w * (1.0f - result.lineParam);
                result.position = s + d*rwpmath::VecFloat(w);
                result.normal = result.position - t;
                result.normal *= rwpmath::Reciprocal(rwpmath::VecFloat(radius));
                result.volParam.Set(
                    float(region==rwc_REGION_VERT1),
                    float(region==rwc_REGION_VERT2), 0.0f);
                break;                                         //  BREAK LOOP
            }
        }
        else                        // edge region
        {
            switch (region)
            {
            case rwc_REGION_EDGE0:
                t = v0;
                a = v1-v0;
                break;
            case rwc_REGION_EDGE1:
                t = v0;
                a = v2-v0;
                break;
            default:
                t = v1;
                a = v2-v1;
            }

            iA = rwcCylinderLineSegIntersect(&distA, s, d, t, a, MagnitudeSquared(a), radius, 0, 0);

            if (iA < 0)   // d points AWAY from the edge tube
            {
                return FALSE;
            }

            // compare to region exit distance

            if ((distB.den = Dot(d, a)) > MINIMUM_RECIPROCAL &&
                (distB.num = Dot(t + a - s, a)) > 0.0f &&
                (iA == 0 || !FracLT(distA, distB)))
            {
                region = rwc_Region(region / 2);       // go to vertex region
                t += a;
                distA = distB;
                iA = 1;
            }
            else
            {
                if ((distB.den *= -1.0f) > MINIMUM_RECIPROCAL &&
                    (distB.num = Dot(s - t, a)) > 0.0f &&
                    (iA == 0 || !FracLT(distA, distB)))
                {
                    region = rwc_Region((region - 3) / 2);     // go to vertex region
                    distA = distB;
                    iA = 1;
                }
            }

            if (iA == 0)
            {
                return FALSE;
            }
            // Hits fattened edge first.  We are done.
            if (region > rwc_REGION_VERT2)
            {
                w = distA.num*rw::math::fpu::Reciprocal(distA.den);
                result.lineParam += w * (1.0f - result.lineParam);
                s += d*rwpmath::VecFloat(w);
                r = MagnitudeSquared(a);
                q = Dot(s - t, a) * rw::math::fpu::Reciprocal(r);
                b = t + a*rwpmath::VecFloat(q);
                result.normal = s - b;
                result.normal *= Reciprocal(rwpmath::VecFloat(radius));
                result.position = s;
                if (region==rwc_REGION_EDGE0)
                {
                    result.volParam.Set(q, 0.0f, 0.0f);
                }
                else if (region==rwc_REGION_EDGE1)
                {
                    result.volParam.Set(0.0f, q, 0.0f);
                }
                else
                {
                    result.volParam.Set(1.0f - q, q, 0.0f);
                }
                break;                                         //  BREAK LOOP
            }
        }


        // push the point forward to the next region

        w = distA.num*rw::math::fpu::Reciprocal(distA.den);
        result.lineParam += w * (1.0f - result.lineParam);
        s += d*rwpmath::VecFloat(w);
        d *= rwpmath::VecFloat(1.0f - w);

        if (w > 1.0f)
        {
            return FALSE;
        }

        EA_ASSERT(result.lineParam < 1.1f);

    }   // END REGION WALK LOOP

    return TRUE;
}
}
}
