// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_FRUSTUM_H
#define PUBLIC_RW_COLLISION_FRUSTUM_H

/*************************************************************************************************************

 File: rwcfrustum.hpp

 Purpose: A short description of the file.
 */


#include "rw/collision/common.h"
#include "rw/collision/plane.h"

namespace rw
{
namespace collision
{

namespace math { class Frustum; }


using Frustum = fb::math::Frustum;


// ***********************************************************************************************************
//                                                Frustum CLASS
// ***********************************************************************************************************

/**
\importlib rwccore
*/
class Frustum
{
public:
    Frustum() {};

    enum PlaneIndex
    {
        PLANE_NA,
        PLANE_FRONT = 0,
        PLANE_BACK,
        PLANE_LEFT,
        PLANE_RIGHT,
        PLANE_TOP,
        PLANE_BOTTOM,
        PLANE_MAX,
        PLANE_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
    };

    void SetPlane(uint32_t index, const Plane &plane) { m_planes[index] = plane; }
    const Plane &GetPlane(uint32_t index) const { return m_planes[index]; }
    Plane GetPlane(uint32_t index) { return m_planes[index]; }

    void TransformPlanes(const rwpmath::Matrix44Affine &transform);

    RwpBool IsSphereInFrustum(const rwpmath::Vector3 &center, float radius);
    RwpBool IsBoxInFrustum(const rwpmath::Vector3 corners[8]);

private:
    Plane   m_planes[6];
};

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_FRUSTUM_H
