// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_TRIANGLEVALIDATOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_TRIANGLEVALIDATOR_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class TriangleValidator
{

public:

    static bool IsTriangleValid(
        rwpmath::Vector3::InParam p0,
        rwpmath::Vector3::InParam p1,
        rwpmath::Vector3::InParam p2);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_TRIANGLEVALIDATOR_H
