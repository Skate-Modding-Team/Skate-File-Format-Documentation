// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/trianglevalidator.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
\brief Determine if a triangle is valid by calculating its area.
A triangle is deemed valid if its area is greater than a tolerance.

TODO: This tolerance is set to an arbitrarily small value related to
rwpmath::MINUMUM_RECIPROAL. This tolerance may be insufficient
(too small), validating triangles which could be considered degenerate
by the collision technology.

\param p0 1st triangle vertex.
\param p1 2st triangle vertex.
\param p2 3st triangle vertex.

\return True if triangle is valid, else false.
*/
bool TriangleValidator::IsTriangleValid(
    rwpmath::Vector3::InParam p0,
    rwpmath::Vector3::InParam p1,
    rwpmath::Vector3::InParam p2)
{
    float lengthSquared;

    rwpmath::Vector3 normal = rwpmath::Cross(p1 - p0, p2 - p0);
    lengthSquared = rwpmath::MagnitudeSquared(normal);

    if (lengthSquared > rwpmath::MINIMUM_RECIPROCAL)
    {
        return TRUE;
    }

    return FALSE;
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

