// (c) Electronic Arts. All Rights Reserved.

#include "rw/collision/deprecated/linetorus.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{

#define RWC_QUARTIC_ROOT_MAX_ITERATION 50
#define RWC_QUARTIC_ROOT_TOLERANCE (float)1e-6

/**
\brief Method for solving quartic roots using newton's method.

This will attempt to converge on a parameter which makes the quartic function zero.
It will only attempt to find a single root, and will just return the first thing that it finds.

\par Implementation
This method will iteratively try to converge on one of the roots to within RWC_QUARTIC_ROOT_TOLERANCE.  If it hasn't found
the root in RWC_QUARTIC_ROOT_MAX_ITERATION iterations, then it will return false.

\param coefficients  The coefficients to the quartic, e.g. coefficients[4] * x^4 + coefficients[3] * x^3, etc
\param root          (out) This gets set to the value of x that gives a root (if a root is found)

\return TRUE if a root can be found, FALSE if not
*/


RwpBool
SolveQuarticRoots(float coefficients[5], float & root)
{
    float t = 0.0f;
    float num = MAX_FLOAT;

    float derivatives[4];
    derivatives[3] = 4.0f * coefficients[4];
    derivatives[2] = 3.0f * coefficients[3];
    derivatives[1] = 2.0f * coefficients[2];
    derivatives[0] = coefficients[1];

    int count = 0;
    while(Abs(num) > RWC_QUARTIC_ROOT_TOLERANCE && count < RWC_QUARTIC_ROOT_MAX_ITERATION)
    {
         num = (coefficients[4] * Pow(t, 4.0f))
             + (coefficients[3] * Pow(t, 3.0f))
             + (coefficients[2] * Pow(t, 2.0f))
             + (coefficients[1] * t)
             +  coefficients[0];

        float denom = derivatives[3] * Pow(t, 3.0f)
            + (derivatives[2] * Pow(t, 2.0f))
            + (derivatives[1] * t)
            +  derivatives[0];

        t += - (num/denom) * (((float)RWC_QUARTIC_ROOT_MAX_ITERATION - (float)count)/(float)RWC_QUARTIC_ROOT_MAX_ITERATION) ;
        count ++;
    }
    root = t;
    return static_cast<RwpBool>(Abs(num) < 0.001f);
}


/**
\brief
Test whether a torus is intersected by a line segment.

This assumes that the torus is lying in the x-y plane and is centered at the origin, so the line orig and dir must
be in torus space.  It returns whether the line intersects, and if it does then dist gets set to the parameter of the line
at the first intersection.  The actual point in world space
can be found using p = o + td (o = line origin, t = line param, d=line direction)

\par Implementation
The method works by computing the coefficients of the line-torus equation (which is a quartic), then solving it using
a polynomial root solver.

\param dist        Gets set to the line parameter
\param orig        The origin of the line in torus space
\param dir         The line direction in torus space
\param majorRadius The outer radius of the torus (the circle about which the minor circle is swept)
\param minorRadius The inner radius of the torus (the circle which is swept)

\return TRUE if the segment intersects the torus.
*/
int32_t
rwcTorusLineSegIntersect(float & dist,
                         rwpmath::Vector3::InParam orig,
                         rwpmath::Vector3::InParam dir,
                         float majorRadius,
                         float minorRadius)
{
    rwcDEPRECATED("Use EA::Collision::IntersectLineTorus()");
    dist = 0.0f;

    /*float coefficients[4];
    float majorRadius2 = majorRadius * majorRadius;
    float minorRadius2 = minorRadius * minorRadius;
    float origDotDir = Dot(orig, dir);
    float origDotOrig = Dot(orig, orig);

    float K = origDotOrig - minorRadius2 - majorRadius2;

    coefficients[0] = 4.0f * origDotDir;
    coefficients[1] = 2.0f * ((2 * origDotDir * origDotDir) + K + (2.0f * majorRadius2 * dir.GetZ() * dir.GetZ()));
    coefficients[2] = 4.0f * ((K * origDotDir) + (2.0f * majorRadius2 * orig.GetZ() * dir.GetZ()));
    coefficients[3] = (K * K) + (4.0f * majorRadius2 * ((orig.GetZ() * orig.GetZ()) -minorRadius2));*/

    // compute coefficients of quartic polynomial
    float fRo2 = majorRadius*majorRadius;
    float fRi2 = minorRadius*minorRadius;
    float fDD = Dot(dir, dir);
    float fDE = Dot(orig, dir);
    float fVal = Dot(orig, orig) - (fRo2 + fRi2);

    float coefficients[5];
    coefficients[0] = fVal*fVal - ((float)4.0)*fRo2*(fRi2 -
                      orig.GetZ()*orig.GetZ());
    coefficients[1] = ((float)4.0)*fDE*fVal +
                      ((float)8.0)*fRo2*dir.GetZ()*orig.GetZ();
    coefficients[2] = ((float)2.0)*fDD*fVal + ((float)4.0)*fDE*fDE +
                      ((float)4.0)*fRo2*dir.GetZ()*dir.GetZ();
    coefficients[3] = ((float)4.0)*fDD*fDE;
    coefficients[4] = fDD*fDD;

    if(SolveQuarticRoots(coefficients, dist))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
}
}
