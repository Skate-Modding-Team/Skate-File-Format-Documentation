// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcfrustum.cpp

 Purpose: Implementation of the Frustum class
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

void Frustum::TransformPlanes(const rwpmath::Matrix44Affine &transform)
{
    uint32_t index = 0;
    while(index < PLANE_MAX)
    {
        m_planes[index].Transform(transform);

        index++;
    }

}

/**
May return false positives
*/
RwpBool Frustum::IsSphereInFrustum(const rwpmath::Vector3 &center, float radius)
{
    uint32_t index = 0;
    while(index < PLANE_MAX)
    {
        if(!m_planes[index].SphereTest(center, radius))
        {
            return FALSE;
        }

        index++;
    }

    return TRUE;
}

/**
May return false positives
*/
RwpBool Frustum::IsBoxInFrustum(const rwpmath::Vector3 corners[8])
{
    uint32_t planeIndex = 0;
    while(planeIndex < PLANE_MAX)
    {
        RwpBool allOutside = TRUE;
        uint32_t pointIndex = 0;
        while(pointIndex < 8)
        {
            allOutside &= !m_planes[planeIndex].PointTest(corners[pointIndex]);
            pointIndex++;
        }

        if(allOutside)
        {
            return FALSE;
        }

        planeIndex++;
    }

    return TRUE;
}
} // namespace collision
} // namespace rw
