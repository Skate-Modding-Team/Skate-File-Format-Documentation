// (c) Electronic Arts. All Rights Reserved.

#include "rw/collision/deprecated/linecylinder.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{

/**
\internal

\brief
This computes the intersection of a line segment with an infinite tube.

Optionally returns intersect distance (scaled by the length of the segment).  The distance is represented
as a \e  Fraction object for performance reasons to avoid using division as long as possible.
If the numerator of the fraction is 0 the orig is inside the tube,
otherwise t=num/den and p = orig + t*seg is the point of entry (or exit, when invert=TRUE).

\par Implementation
Finding the point of intersection of a line and a tube requires finding the roots of a quadratic equation.
If the roots are imaginary, the line does not intersect.  Otherwise there may be up to two roots, one for
the point of entry, and the other for the point of exit.

\param dist                Output parametric distance from orig to the point of entry or exit. can be NULL.
\param orig                Start of segment
\param seg                End of segment relative to orig
\param center            Middle of tube
\param axis                Tube axis vector
\param axisLengthSq        Length squared of tube axis vector
\param radius            Radius of tube
\param invert            FALSE=return entry pt, TRUE=return exit point
\param ignoreInside        When orig is inside the tube: FALSE=return orig, TRUE=return neg dist.

\return true 1=intersect, 0=near miss, -1=far miss.

\see rwcPlaneLineSegIntersect, rwcSphereLineSegIntersect, ThinTriangleLineSegIntersect
*/

int32_t rwcCylinderLineSegIntersect(Fraction * dist,
                            Vector3::InParam orig,
                            Vector3::InParam seg,
                            Vector3::InParam center,
                            Vector3::InParam axis,
                            float axisLengthSq,
                            float radius,
                            RwpBool invert,
                            RwpBool ignoreInside)
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineCylinder()");
    Vector3 D, sxa, dxa;
    float a, b, c1, c2, arg, drMr2;
//    float da, sa;
    float s;

    /*  Surface intersect:

    p = orig + t*seg
    x = p - center
    a = axis
    |x - (x.a).a|^2 = R^2

    D = center - orig
    (|s|^2 - (s.a)^2)t^2 - 2(s.D - s.a*D.a)t + |D|^2 - (D.a)^2 - R^2 = 0
    a*t^2 - 2b*t + c = 0
    t = (b +/- sqrt(b*b - a*c)) / a

    The c is separated into c1-c2 to avoid numerical roundoff problems.

    Note these maths identities:
    (a x b)^2 = aa*bb - (ab)^2
    (a x b)*(c x d) = ac*bd - ad*bc
    */

    D = center - orig;
//    da = Dot(D, axis);
    dxa = Cross(D, axis);
    c1 = Dot(dxa, dxa);
    c2 = axisLengthSq*radius*radius;

    EA_ASSERT(dist);

    if (c1 < c2 && !ignoreInside)
    {   // Segment origin lies inside(outside) cylinder
        dist->num = 0.0f;
        dist->den = 1.0f;
        return 1;
    }

//    sa = Dot(seg, axis);
    sxa = Cross(seg, axis);
    b = Dot(dxa, sxa);
    if (!invert && b <= 0)
    {   // Segment is tangent to or points away from cylinder
        return -1;
    }

    // Segment origin lies outside(inside) of cylinder
    a = Dot(sxa, sxa);
    arg = (b*b - a*c1) + a*c2;
    if (arg < 0)
    {   // Ray does not intersect cylinder
        return 0;
    }

    drMr2 = b - a;
    if ((!invert && drMr2 >= 0 && drMr2*drMr2 >= arg) ||
        (invert && (drMr2 >= 0 || drMr2*drMr2 <= arg)))
    {
        // Segment does not reach cylinder
        return 0;
    }

    s = invert ? -1.0f : 1.0f;

    dist->num = b - s*rw::math::fpu::Sqrt(arg);
    dist->den = a;

    EA_ASSERT(a >= MINIMUM_RECIPROCAL);

    return 1;
}

/**

\brief
Test whether a fat cylinder volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.
For example, if the line is 10 units long, and intersects the volume 8 units from the start, then the
\e  lineParam is 0.8.  This version can also cope with fat cylinders, but it is a lot more expensive if
the line hits the vicinity of the rim.

\par Implementation
The algorithm first culls any lines that are entirely on the far side of either end cap.  It then checks whether
the line intersects the endcaps, then finally checks whether the line intersects the cylinder body.  The worst case
is if the line intersects with the fat rim of the cylinder, which is effectively a torus and is quite expensive to compute.

\param inPt1    The origin of the line segment.
\param inPt2    The end of the line segment.
\param tm       Transformation Matrix of the parent reference frame.
\param result   This will be set to the result of the test.
\param fatness  The padding added around the cylinder

\return TRUE if the segment intersects the volume.
*/

RwpBool
CylinderVolume::FatLineSegIntersect(const Vector3 &inPt1, const Vector3 &inPt2,
                                     const Matrix44Affine *tm,
                                     VolumeLineSegIntersectResult &result,
                                     float fatness /* =0.0f */) const
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineCylinder()");
    Vector3 pt1, pt2, rayDirection;
    Vector3 zero = GetVector3_Zero();
    Vector3 zAxis = GetVector3_ZAxis();
    Matrix44Affine invTm, mtx;
    float halfHeight, radiusSquared, totalFatness;

    result.v = static_cast<const Volume *>(this);

    // Map line ends into cylinder space
    if (tm)
    {
        mtx = invTm = transform * (*tm);
    }
    else
    {
        mtx = GetMatrix44Affine_Identity();
        invTm = transform;
    }
    invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
    pt1 = TransformPoint(inPt1, invTm);
    pt2 = TransformPoint(inPt2, invTm);

    halfHeight = GetHalfHeight();
    rayDirection = pt2 - pt1;
    radiusSquared = GetInnerRadius() * GetInnerRadius();
    totalFatness = fatness + GetRadius();


    //Check whether both points of the line are on the same side as one of the ends, if so, no collision
    if (pt1.GetZ() > halfHeight + totalFatness && pt2.GetZ() > halfHeight + totalFatness)
    {
        return 0;
    }

    if (pt1.GetZ() < -halfHeight - totalFatness && pt2.GetZ() < -halfHeight - totalFatness)
    {
        return 0;
    }

    //If the start point is outside of the cylinder and the direction is pointing away from it, then it can't collide
    Vector3 pt1ToCylinder =  transform.GetW() - pt1;
    Vector3 tmpPt1 = pt1;
    tmpPt1.SetZ(0.0f);
    if (MagnitudeSquared(tmpPt1) > (radiusSquared+totalFatness) && Dot(pt1ToCylinder, rayDirection) < 0.0f)
    {
        return 0;
    }


    RwpBool foundContact = FALSE;
    RwpBool possibleTorusContact = FALSE;
    Vector3 axis = zAxis;

    if (rayDirection.GetZ() > 0.0f)
    {
        axis = -axis;
    }
    //Project the line onto the end cap plane and check the squared distance to see if it's inside the circle
    float lineParam = ((halfHeight+totalFatness) - (Dot(axis, pt1)))/(Dot(axis, rayDirection));
    Vector3 planeIntersection = pt1 + (lineParam * rayDirection);
    float distSquared = (planeIntersection.GetX() * planeIntersection.GetX()) + (planeIntersection.GetY() * planeIntersection.GetY());
    if (distSquared < (radiusSquared * 1.001))
    {
        //We've hit the end cap
        result.position = planeIntersection;
        result.lineParam = lineParam;
        result.normal = axis;
        foundContact = TRUE;
    }
    else if (distSquared < (totalFatness + GetInnerRadius()) * (totalFatness + GetInnerRadius()))
    {
        //We didn't hit the flat part of the cylinder cap, but we might have hit the rounded rim
        //Save this till last - it's very expensive
        possibleTorusContact = TRUE;
    }

    //Check if we have made contact with the cylinder body
    if (!foundContact)
    {
        Fraction dist;
        if (rwcCylinderLineSegIntersect(&dist, pt1, rayDirection, zero, zAxis, 1.0f, GetInnerRadius()+totalFatness, false, false) == 1)
        {
            //It's hit the theoretical infinitely long cylinder somewhere, so check if it's within our finitely sized cylinder
            float lineParamBody = dist.num / dist.den;
            Vector3 cylinderIntersection = pt1 + (lineParamBody * rayDirection);
            if (cylinderIntersection.GetZ() < halfHeight && cylinderIntersection.GetZ() > -halfHeight)
            {
                //We've hit the cylinder body
                result.position = cylinderIntersection;
                result.lineParam = lineParamBody;
                result.normal = cylinderIntersection;
                result.normal.SetZ(0.0f);
                result.normal = Normalize(result.normal);
                foundContact = TRUE;
            }
            else if (cylinderIntersection.GetZ() < (halfHeight + totalFatness) && cylinderIntersection.GetZ() > -(halfHeight + totalFatness))
            {
                //We might have hit the torus
                possibleTorusContact=TRUE;

                //It's possible that we might be looking at the wrong torus, so reset the axis based on the
                //cylinder intersection to make sure
                if(cylinderIntersection.GetZ()>0.0f)
                {
                    axis = zAxis;
                }
                else
                {
                    axis=-zAxis;
                }

            }

        }
    }

    //If we might have hit a torus earlier then do the expensive torus intersection test
    if(possibleTorusContact)
    {
        //Clip the line to the torus's aabb to improve precision
        Vector3 lineOrigTorusSpace = pt1 - (axis*halfHeight);
        AABBox torusBB = AABBox(Vector3(-GetInnerRadius()-totalFatness, -GetInnerRadius()-totalFatness, -totalFatness),
                                Vector3(GetInnerRadius()+totalFatness, GetInnerRadius()+totalFatness, totalFatness));
        torusBB.m_max *= 1.5f;
        torusBB.m_min *= 1.5f;
        AALineClipper clipper(lineOrigTorusSpace, lineOrigTorusSpace+rayDirection, torusBB);
        float pa1=0.0f;
        float pa2=1.0f;
        if(clipper.ClipToAABBox(pa1, pa2, torusBB))
        {
            float lineParamTorus=0.0f;
            Vector3 from, dir;
            from = lineOrigTorusSpace + pa1*rayDirection;
            dir = (pa2 -pa1) * rayDirection;
            if(rwcTorusLineSegIntersect(lineParamTorus, from, dir, GetInnerRadius(), totalFatness) == 1)
            //if(rwcTorusLineSegIntersect(lineParamTorus, lineOrigTorusSpace, rayDirection, GetInnerRadius()+(fatness/2.0f), (fatness/2.0f)) == 1)
            {
                //We've hit the torus
                if(lineParamTorus < result.lineParam || !foundContact)
                {
                    //result.lineParam = lineParamTorus;
                    result.lineParam = pa1 + (lineParamTorus * (pa2-pa1));
                    result.position = pt1 + (result.lineParam * rayDirection);

                    Vector3 centreToOuter = result.position - (axis*halfHeight);
                    centreToOuter.SetZ(0.0f);
                    centreToOuter = (centreToOuter/Magnitude(centreToOuter)) * GetInnerRadius();
                    Vector3 innerEdgeToOuterPoint = (result.position - (axis*halfHeight)) - centreToOuter;
                    result.normal = Normalize(innerEdgeToOuterPoint);
                    foundContact = TRUE;
                }
            }
        }
    }

    //Transform result back into world space
    if (tm)
    {
        result.position = TransformPoint(result.position, mtx);
        result.normal = TransformVector(result.normal, mtx);
    }
    else
    {
        result.position = TransformPoint(result.position, transform);
        result.normal = TransformVector(result.normal, transform);
    }

    result.position -= result.normal * fatness;

    return foundContact;
}

/**
\brief
Test whether a cylinder volume is intersected by a line segment.

This returns the location of the intersection point in world space, the normal of the volume surface at
the point of intersection, and the parametric distance of the intersection from the start of the line.
For example, if the line is 10 units long, and intersects the volume 8 units from the start, then the
\e  lineParam is 0.8.

\par Implementation
The algorithm first culls any lines that are entirely on the far side of either end cap.  It then checks whether
the line intersects the endcaps, then finally checks whether the line intersects the cylinder body.

\param inPt1    The origin of the line segment.
\param inPt2    The end of the line segment.
\param tm       Transformation Matrix of the parent reference frame.
\param result   This will be set to the result of the test.

\return TRUE if the segment intersects the volume.
 */
RwpBool
CylinderVolume::ThinLineSegIntersect(const Vector3 &inPt1, const Vector3 &inPt2,
                                 const Matrix44Affine *tm,
                                 VolumeLineSegIntersectResult &result) const
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineCylinder()");
    Vector3 pt1, pt2, rayDirection;
    Vector3 zero = GetVector3_Zero();
    Vector3 zAxis = GetVector3_ZAxis();
    Matrix44Affine invTm, mtx;
    float halfHeight, radiusSquared;

    result.v = static_cast<const Volume *>(this);

    // Map line ends into cylinder space
    if (tm)
    {
        mtx = invTm = transform * (*tm);
    }
    else
    {
        mtx = GetMatrix44Affine_Identity();
        invTm = transform;
    }
    invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
    pt1 = TransformPoint(inPt1, invTm);
    pt2 = TransformPoint(inPt2, invTm);

    halfHeight = GetHalfHeight();
    rayDirection = pt2 - pt1;
    radiusSquared = GetInnerRadius() * GetInnerRadius();

    //Check whether both points of the line are on the same side as one of the ends, if so, no collision
    if (pt1.GetZ() > halfHeight && pt2.GetZ() > halfHeight)
    {
        return 0;
    }

    if (pt1.GetZ() < -halfHeight && pt2.GetZ() < -halfHeight)
    {
        return 0;
    }

    //If the start point is outside of the cylinder and the direction is pointing away from it, then it can't collide
    Vector3 pt1ToCylinder =  transform.GetW() - pt1;
    pt1ToCylinder.SetZ(0.0f);
    Vector3 tmpPt1 = pt1;
    tmpPt1.SetZ(0.0f);
    if (MagnitudeSquared(tmpPt1) > radiusSquared && Dot(pt1ToCylinder, rayDirection) < 0.0f)
    {
        return 0;
    }

    RwpBool foundContact = FALSE;

    //Check for collisions with the end caps only if the points are on either side of the end cap's plane
    if (pt1.GetZ() > halfHeight || pt1.GetZ() < -halfHeight)
    {
        Vector3 axis = zAxis;
        if (rayDirection.GetZ() > 0.0f)
        {
            axis = -axis;
        }
        //Project the line onto the end cpa plane and check the squared distance to see if it's inside the circle
        float lineParam = (halfHeight - (Dot(axis, pt1)))/(Dot(axis, rayDirection));
        Vector3 planeIntersection = pt1 + (lineParam * rayDirection);
        float distSquared = (planeIntersection.GetX() * planeIntersection.GetX()) + (planeIntersection.GetY() * planeIntersection.GetY());
        if (distSquared < radiusSquared)
        {
            //We've hit the end cap
            result.position = planeIntersection;
            result.lineParam = lineParam;
            result.normal = axis;
            foundContact = TRUE;
        }
    }

    //Finally, check if we have made contact with the cylinder body
    if (!foundContact)
    {
        Fraction dist;
        if (rwcCylinderLineSegIntersect(&dist, pt1, rayDirection, zero, zAxis, 1.0f, GetInnerRadius(), false, false) == 1)
        {
            //It's hit the theoretical infinately long cylinder somewhere, so check if it's within our finitely sized cylinder
            float lineParam = dist.num / dist.den;
            Vector3 cylinderIntersection = pt1 + (lineParam * rayDirection);
            if (cylinderIntersection.GetZ() < halfHeight && cylinderIntersection.GetZ() > -halfHeight)
            {
                //We've hit the cylinder body
                result.position = cylinderIntersection;
                result.lineParam = lineParam;
                result.normal = cylinderIntersection;
                result.normal.SetZ(0.0f);
                result.normal = Normalize(result.normal);
                foundContact = TRUE;
            }
            else
            {
                //No collision
                return 0;
            }
        }
        else
        {
            //No collision
            return 0;

        }

    }

    //Transform result back into world space
    if (tm)
    {
        result.position = TransformPoint(result.position, mtx);
        result.normal = TransformVector(result.normal, mtx);
    }
    else
    {
        result.position = TransformPoint(result.position, transform);
        result.normal = TransformVector(result.normal, transform);
    }

    return 1;
}
}
}
