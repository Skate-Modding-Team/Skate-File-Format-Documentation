// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcplane.cpp

 Purpose: Implementation of the Plane class
 */

// ***********************************************************************************************************
// Includes

#include "rw/collision/plane.h"
#include "rw/collision/frustum.h"

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


// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions


// ***********************************************************************************************************
// Structs + Unions + Classes

// ***********************************************************************************************************
// Static Functions


// ***********************************************************************************************************
// External Functions

// ***********************************************************************************************************
// Class Member Functions

// ***********************************************************************************************************
//                                                Frustum CLASS
// ***********************************************************************************************************

void Plane::Transform(const rwpmath::Matrix44Affine &transform)
{
    // Find a point on the plane.
    Vector3 pointOnPlane = GetNormal() * rwpmath::VecFloat(GetDistance());

    // Rotate the Normal, trash the distance.
    Vector3 n = TransformVector(m_data.GetVector3(), transform);
    m_data.SetVector3(n);

    // Transform the point on the plane.
    pointOnPlane = TransformPoint(pointOnPlane, transform);

    SetDistance(Dot(n, pointOnPlane));

}


} // namespace collision
} // namespace rw
