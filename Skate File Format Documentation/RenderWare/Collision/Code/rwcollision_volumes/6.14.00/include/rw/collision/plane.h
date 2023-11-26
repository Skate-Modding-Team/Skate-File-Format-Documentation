// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_PLANE_H
#define PUBLIC_RW_COLLISION_PLANE_H

/*************************************************************************************************************

 File: rwcfrustum.hpp

 Purpose: A short description of the file.
 */


#include "rw/collision/common.h"

namespace rw
{
namespace collision
{

class Plane;


// ***********************************************************************************************************
//                                                Plane CLASS
// ***********************************************************************************************************

/**
\importlib rwccore
*/
class Plane
{
public:
    Plane() {};
    Plane(rwpmath::Vector3::InParam normal, float distance) : m_data(normal.X(), normal.Y(), normal.Z(), distance) {};
    rwpmath::Vector3        GetNormal() { return m_data.GetVector3(); }
    float                 GetDistance() { return m_data.UserData(); }

    void                    SetDistance(float distance) { m_data.UserData()= distance; }
    void                    SetNormal(rwpmath::Vector3::InParam normal) { m_data.SetVector3(normal); }

    void                    Transform(const rwpmath::Matrix44Affine &transform);

    inline RwpBool       PointTest (rwpmath::Vector3::InParam center);
    inline RwpBool       SphereTest(rwpmath::Vector3::InParam center, float radius);

private:
    rwpmath::Vector3Plus   m_data;
};


/*
\todoc
*/
inline RwpBool
Plane::PointTest(rwpmath::Vector3::InParam center)
{
    float distance = Dot(GetNormal(), center);

    return static_cast<RwpBool>(GetDistance() < distance);
}


/*
\todoc
*/
inline RwpBool
Plane::SphereTest(rwpmath::Vector3::InParam center, float radius)
{
    float distance = Dot(GetNormal(), center);

    return static_cast<RwpBool>(GetDistance() < distance + radius);
}


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_PLANE_H
