// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/trianglenormal.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
\brief Computes the normal of a triangle.

\param p0 The 1st vertex of a triangle.
\param p1 The 2nd vertex of a triangle.
\param p2 The 3rd vertex of a triangle.

\return Returns the normal vector.
*/
rwpmath::Vector3 TriangleNormal::ComputeTriangleNormalFast(
    rwpmath::Vector3::InParam p0,
    rwpmath::Vector3::InParam p1,
    rwpmath::Vector3::InParam p2)
{
    return NormalizeFast(Cross(p1 - p0, p2 - p0));
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

